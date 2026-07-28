#include "onnx_inference.hpp"

#include <onnxruntime_cxx_api.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <filesystem>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#ifdef AUTO_AIM_WITH_CUDA
#include <cuda_runtime_api.h>

#include "cuda_preprocess.hpp"
#endif

namespace auto_aim
{
namespace
{
constexpr int64_t INPUT_HEIGHT = 640;
constexpr int64_t INPUT_WIDTH = 640;
constexpr int64_t OUTPUT_ROWS = 25200;
constexpr int64_t OUTPUT_COLUMNS = 22;
constexpr std::size_t PIXEL_COUNT = INPUT_HEIGHT * INPUT_WIDTH;
constexpr std::size_t INPUT_ELEMENT_COUNT = 3 * PIXEL_COUNT;
constexpr std::size_t OUTPUT_ELEMENT_COUNT = OUTPUT_ROWS * OUTPUT_COLUMNS;
constexpr std::array<int64_t, 4> INPUT_SHAPE{1, 3, INPUT_HEIGHT, INPUT_WIDTH};
constexpr std::array<int64_t, 3> OUTPUT_SHAPE{1, OUTPUT_ROWS, OUTPUT_COLUMNS};

std::string shape_string(const std::vector<int64_t> & shape)
{
  std::ostringstream stream;
  stream << '[';
  for (std::size_t index = 0; index < shape.size(); ++index) {
    if (index != 0) stream << ',';
    stream << shape[index];
  }
  stream << ']';
  return stream.str();
}

template <std::size_t Size>
bool shape_matches(const std::vector<int64_t> & actual, const std::array<int64_t, Size> & expected)
{
  return actual.size() == expected.size() &&
         std::equal(actual.begin(), actual.end(), expected.begin());
}

#ifdef AUTO_AIM_WITH_CUDA
void check_cuda(cudaError_t result, const std::string & operation)
{
  if (result == cudaSuccess) return;
  throw std::runtime_error(operation + " failed: " + cudaGetErrorString(result));
}

class CudaStream
{
public:
  ~CudaStream()
  {
    if (stream_ != nullptr) cudaStreamDestroy(stream_);
  }

  void create()
  {
    check_cuda(cudaStreamCreateWithFlags(&stream_, cudaStreamNonBlocking), "cudaStreamCreate");
  }

  cudaStream_t get() const { return stream_; }

private:
  cudaStream_t stream_{nullptr};
};

class CudaBuffer
{
public:
  ~CudaBuffer()
  {
    if (data_ != nullptr) cudaFree(data_);
  }

  void allocate(std::size_t bytes)
  {
    check_cuda(cudaMalloc(&data_, bytes), "cudaMalloc");
  }

  void * get() const { return data_; }

private:
  void * data_{nullptr};
};
#endif
}  // namespace

class ONNXInference::Impl
{
public:
  Impl(std::string model_path, std::string device)
  : device_(std::move(device)),
    env_(ORT_LOGGING_LEVEL_WARNING, "sp_vision_onnx"),
    cpu_memory_info_(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault))
  {
    if (device_ != "CPU" && device_ != "GPU") {
      throw std::invalid_argument("Unsupported inference device: " + device_ + "; use CPU or GPU");
    }
    if (!std::filesystem::is_regular_file(model_path)) {
      throw std::runtime_error("ONNX model does not exist: " + model_path);
    }

    session_options_.SetExecutionMode(ExecutionMode::ORT_SEQUENTIAL);
    session_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

    if (device_ == "GPU") configure_cuda_provider();

    try {
      session_ = Ort::Session(env_, model_path.c_str(), session_options_);
      validate_contract();
      prepare_tensors();
      warmup();
    } catch (const Ort::Exception & error) {
      throw std::runtime_error(
        "Failed to initialize ONNX Runtime " + device_ + " inference: " + error.what());
    }
  }

  cv::Mat run(const cv::Mat & letterboxed_bgr)
  {
    if (
      letterboxed_bgr.type() != CV_8UC3 || letterboxed_bgr.rows != INPUT_HEIGHT ||
      letterboxed_bgr.cols != INPUT_WIDTH || !letterboxed_bgr.isContinuous()) {
      throw std::invalid_argument("ONNX input must be a continuous 640x640 CV_8UC3 image");
    }

    if (device_ == "CPU") return run_cpu(letterboxed_bgr);
#ifdef AUTO_AIM_WITH_CUDA
    return run_gpu(letterboxed_bgr);
#else
    throw std::runtime_error("GPU inference was not compiled into this build");
#endif
  }

  const std::string & device() const { return device_; }

private:
  std::string device_;

#ifdef AUTO_AIM_WITH_CUDA
  CudaStream cuda_stream_;
  CudaBuffer device_bgr_;
  CudaBuffer device_input_;
  CudaBuffer device_output_;
#endif

  Ort::Env env_;
  Ort::SessionOptions session_options_;
  Ort::Session session_{nullptr};
  Ort::MemoryInfo cpu_memory_info_;
  Ort::MemoryInfo cuda_memory_info_{nullptr};
  std::vector<Ort::Float16_t> cpu_input_;
  std::vector<float> host_output_;
  Ort::Value input_tensor_{nullptr};
  Ort::Value output_tensor_{nullptr};
  Ort::IoBinding io_binding_{nullptr};
  std::string input_name_;
  std::string output_name_;

  void configure_cuda_provider()
  {
#ifdef AUTO_AIM_WITH_CUDA
    const auto providers = Ort::GetAvailableProviders();
    if (std::find(providers.begin(), providers.end(), "CUDAExecutionProvider") == providers.end()) {
      throw std::runtime_error(
        "ONNX Runtime CUDAExecutionProvider is unavailable; install the CUDA ONNX Runtime package");
    }

    int device_count = 0;
    check_cuda(cudaGetDeviceCount(&device_count), "cudaGetDeviceCount");
    if (device_count <= 0) throw std::runtime_error("CUDA device 0 is unavailable");
    check_cuda(cudaSetDevice(0), "cudaSetDevice(0)");
    cuda_stream_.create();

    OrtCUDAProviderOptions options{};
    options.device_id = 0;
    options.has_user_compute_stream = 1;
    options.user_compute_stream = cuda_stream_.get();
    options.do_copy_in_default_stream = 0;
    session_options_.AppendExecutionProvider_CUDA(options);
    session_options_.AddConfigEntry("session.disable_cpu_ep_fallback", "1");
#else
    throw std::runtime_error(
      "GPU inference was requested, but this build has no CUDA support; reconfigure with "
      "SP_VISION_ENABLE_CUDA=ON");
#endif
  }

  void validate_contract()
  {
    if (session_.GetInputCount() != 1 || session_.GetOutputCount() != 1) {
      throw std::runtime_error("0526 model must have exactly one input and one output");
    }

    const auto input_type_info = session_.GetInputTypeInfo(0);
    const auto output_type_info = session_.GetOutputTypeInfo(0);
    const auto input_info = input_type_info.GetTensorTypeAndShapeInfo();
    const auto output_info = output_type_info.GetTensorTypeAndShapeInfo();
    const auto input_shape = input_info.GetShape();
    const auto output_shape = output_info.GetShape();

    if (
      input_info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT16 ||
      !shape_matches(input_shape, INPUT_SHAPE)) {
      throw std::runtime_error(
        "0526 input contract mismatch: expected FP16 [1,3,640,640], got type " +
        std::to_string(input_info.GetElementType()) + " " + shape_string(input_shape));
    }
    if (
      output_info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
      !shape_matches(output_shape, OUTPUT_SHAPE)) {
      throw std::runtime_error(
        "0526 output contract mismatch: expected FP32 [1,25200,22], got type " +
        std::to_string(output_info.GetElementType()) + " " + shape_string(output_shape));
    }

    Ort::AllocatorWithDefaultOptions allocator;
    input_name_ = session_.GetInputNameAllocated(0, allocator).get();
    output_name_ = session_.GetOutputNameAllocated(0, allocator).get();
  }

  void prepare_tensors()
  {
    host_output_.resize(OUTPUT_ELEMENT_COUNT);
    if (device_ == "CPU") {
      cpu_input_.resize(INPUT_ELEMENT_COUNT);
      input_tensor_ = Ort::Value::CreateTensor<Ort::Float16_t>(
        cpu_memory_info_, cpu_input_.data(), cpu_input_.size(), INPUT_SHAPE.data(),
        INPUT_SHAPE.size());
      output_tensor_ = Ort::Value::CreateTensor<float>(
        cpu_memory_info_, host_output_.data(), host_output_.size(), OUTPUT_SHAPE.data(),
        OUTPUT_SHAPE.size());
      return;
    }

#ifdef AUTO_AIM_WITH_CUDA
    device_bgr_.allocate(PIXEL_COUNT * 3 * sizeof(std::uint8_t));
    device_input_.allocate(INPUT_ELEMENT_COUNT * sizeof(std::uint16_t));
    device_output_.allocate(OUTPUT_ELEMENT_COUNT * sizeof(float));
    cuda_memory_info_ = Ort::MemoryInfo("Cuda", OrtDeviceAllocator, 0, OrtMemTypeDefault);
    input_tensor_ = Ort::Value::CreateTensor(
      cuda_memory_info_, device_input_.get(), INPUT_ELEMENT_COUNT * sizeof(std::uint16_t),
      INPUT_SHAPE.data(), INPUT_SHAPE.size(), ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT16);
    output_tensor_ = Ort::Value::CreateTensor(
      cuda_memory_info_, device_output_.get(), OUTPUT_ELEMENT_COUNT * sizeof(float),
      OUTPUT_SHAPE.data(), OUTPUT_SHAPE.size(), ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT);
    io_binding_ = Ort::IoBinding(session_);
    io_binding_.BindInput(input_name_.c_str(), input_tensor_);
    io_binding_.BindOutput(output_name_.c_str(), output_tensor_);
#endif
  }

  cv::Mat run_cpu(const cv::Mat & bgr)
  {
    constexpr float scale = 1.0F / 255.0F;
    for (int row = 0; row < bgr.rows; ++row) {
      const auto * pixels = bgr.ptr<cv::Vec3b>(row);
      for (int column = 0; column < bgr.cols; ++column) {
        const auto index = static_cast<std::size_t>(row) * INPUT_WIDTH + column;
        cpu_input_[index] = Ort::Float16_t(static_cast<float>(pixels[column][2]) * scale);
        cpu_input_[PIXEL_COUNT + index] =
          Ort::Float16_t(static_cast<float>(pixels[column][1]) * scale);
        cpu_input_[2 * PIXEL_COUNT + index] =
          Ort::Float16_t(static_cast<float>(pixels[column][0]) * scale);
      }
    }

    const char * input_names[] = {input_name_.c_str()};
    const char * output_names[] = {output_name_.c_str()};
    session_.Run(
      Ort::RunOptions{nullptr}, input_names, &input_tensor_, 1, output_names, &output_tensor_, 1);
    return cv::Mat(OUTPUT_ROWS, OUTPUT_COLUMNS, CV_32F, host_output_.data());
  }

#ifdef AUTO_AIM_WITH_CUDA
  cv::Mat run_gpu(const cv::Mat & bgr)
  {
    check_cuda(
      cudaMemcpyAsync(
        device_bgr_.get(), bgr.data, PIXEL_COUNT * 3 * sizeof(std::uint8_t),
        cudaMemcpyHostToDevice, cuda_stream_.get()),
      "CUDA input upload");
    check_cuda(
      launch_bgr_to_rgb_fp16(
        static_cast<const std::uint8_t *>(device_bgr_.get()), device_input_.get(), PIXEL_COUNT,
        cuda_stream_.get()),
      "CUDA preprocessing kernel");

    session_.Run(Ort::RunOptions{nullptr}, io_binding_);
    check_cuda(
      cudaMemcpyAsync(
        host_output_.data(), device_output_.get(), OUTPUT_ELEMENT_COUNT * sizeof(float),
        cudaMemcpyDeviceToHost, cuda_stream_.get()),
      "CUDA output download");
    check_cuda(cudaStreamSynchronize(cuda_stream_.get()), "CUDA inference synchronization");
    return cv::Mat(OUTPUT_ROWS, OUTPUT_COLUMNS, CV_32F, host_output_.data());
  }
#endif

  void warmup()
  {
    const cv::Mat black(INPUT_HEIGHT, INPUT_WIDTH, CV_8UC3, cv::Scalar::all(0));
    for (int iteration = 0; iteration < 3; ++iteration) run(black);
  }
};

ONNXInference::ONNXInference(const std::string & model_path, const std::string & device)
: impl_(std::make_unique<Impl>(model_path, device))
{
}

ONNXInference::~ONNXInference() = default;

cv::Mat ONNXInference::run(const cv::Mat & letterboxed_bgr)
{
  return impl_->run(letterboxed_bgr);
}

const std::string & ONNXInference::device() const { return impl_->device(); }
}  // namespace auto_aim
