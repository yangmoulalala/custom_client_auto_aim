#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <list>
#include <opencv2/opencv.hpp>
#include <stdexcept>
#include <string>
#include <vector>

#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tasks/auto_aim/yolos/yolov5.hpp"

namespace
{
using Clock = std::chrono::steady_clock;

int failures = 0;

void expect(bool condition, const std::string & message)
{
  if (condition) return;
  std::cerr << "FAIL: " << message << '\n';
  ++failures;
}

template <typename Function>
void expect_throw(Function function, const std::string & message)
{
  try {
    function();
    expect(false, message);
  } catch (const std::exception &) {
  }
}

class TemporaryConfig
{
public:
  TemporaryConfig(
    const std::string & device, const std::string & model_path = AUV_MODEL_PATH,
    const std::string & yolo_name = "yolov5")
  {
    static int sequence = 0;
    auto yaml = YAML::LoadFile(CUSTOM_CLIENT_CONFIG);
    yaml["device"] = device;
    yaml["yolov5_model_path"] = model_path;
    yaml["yolo_name"] = yolo_name;
    path_ = std::filesystem::temp_directory_path() /
            ("sp_vision_yolo_test_" + std::to_string(sequence++) + ".yaml");
    std::ofstream output(path_);
    output << yaml;
    if (!output) throw std::runtime_error("Failed to write temporary YOLO test configuration");
  }

  ~TemporaryConfig()
  {
    std::error_code error;
    std::filesystem::remove(path_, error);
  }

  std::string path() const { return path_.string(); }

private:
  std::filesystem::path path_;
};

cv::Mat make_output(int color_id, int number_id = 3)
{
  cv::Mat output = cv::Mat::zeros(1, 22, CV_32F);
  output.at<float>(0, 0) = 10;
  output.at<float>(0, 1) = 10;
  output.at<float>(0, 2) = 10;
  output.at<float>(0, 3) = 30;
  output.at<float>(0, 4) = 30;
  output.at<float>(0, 5) = 30;
  output.at<float>(0, 6) = 30;
  output.at<float>(0, 7) = 10;
  output.at<float>(0, 8) = 10;
  output.at<float>(0, 9 + color_id) = 1;
  output.at<float>(0, 13 + number_id) = 1;
  return output;
}

void test_armor_layout()
{
  const std::vector<cv::Point2f> points{{10, 10}, {30, 10}, {30, 20}, {10, 20}};
  const cv::Rect box(10, 10, 20, 10);

  const auto_aim::Armor red_sentry(0, 0, 0.9F, box, points);
  const auto_aim::Armor blue_hero(1, 1, 0.9F, box, points);
  const auto_aim::Armor red_three(0, 3, 0.9F, box, points);
  const auto_aim::Armor gray_base(2, 7, 0.9F, box, points);
  const auto_aim::Armor purple_legacy_base(3, 8, 0.9F, box, points);
  const auto_aim::Armor retired_balance(29, 0.9F, box, points);

  expect(
    red_sentry.color == auto_aim::Color::red && red_sentry.name == auto_aim::ArmorName::sentry &&
      red_sentry.type == auto_aim::ArmorType::small,
    "class G must map to a small red sentry armor");
  expect(
    blue_hero.color == auto_aim::Color::blue && blue_hero.name == auto_aim::ArmorName::one &&
      blue_hero.type == auto_aim::ArmorType::big,
    "class 1 must map to a big blue hero armor");
  expect(
    red_three.color == auto_aim::Color::red && red_three.name == auto_aim::ArmorName::three &&
      red_three.type == auto_aim::ArmorType::small,
    "class 3 must map to a small red infantry armor");
  expect(
    gray_base.color == auto_aim::Color::extinguish &&
      gray_base.name == auto_aim::ArmorName::base &&
      gray_base.type == auto_aim::ArmorType::small,
    "class Bs must map to a small gray base armor");
  expect(
    purple_legacy_base.color == auto_aim::Color::purple &&
      purple_legacy_base.name == auto_aim::ArmorName::base &&
      purple_legacy_base.type == auto_aim::ArmorType::small,
    "legacy class Bb must map to a small purple base armor");
  expect(
    retired_balance.name == auto_aim::ArmorName::not_armor,
    "retired balance-infantry classes must be rejected");
}

void test_active_config()
{
  const auto yaml = YAML::LoadFile(CUSTOM_CLIENT_CONFIG);
  expect(yaml["yolo_name"].as<std::string>() == "yolov5", "AUV must select yolov5");
  expect(
    yaml["yolov5_model_path"].as<std::string>() == "auto_aim/models/0526.onnx",
    "AUV must reference only models/0526.onnx");
  expect(yaml["device"].as<std::string>() == "GPU", "AUV must default to GPU");
  expect(
    yaml["yolov5_color_order"].as<std::string>() == "blue_red_gray_purple",
    "0526 must use blue_red_gray_purple color order");
}

void test_startup_failures()
{
  const TemporaryConfig invalid_device("AUTO");
  expect_throw(
    [&] { auto_aim::YOLOV5 detector(invalid_device.path(), false); },
    "device other than CPU or GPU must fail startup");

  const TemporaryConfig invalid_model("CPU", INVALID_MODEL_PATH);
  expect_throw(
    [&] { auto_aim::YOLOV5 detector(invalid_model.path(), false); },
    "model with the wrong tensor contract must fail startup");

  const TemporaryConfig invalid_factory("CPU", AUV_MODEL_PATH, "yolov8");
  expect_throw(
    [&] { auto_aim::YOLO detector(invalid_factory.path(), false); },
    "active YOLO factory must reject dormant detector types");

#ifndef SP_VISION_TEST_WITH_CUDA
  const TemporaryConfig unavailable_gpu("GPU");
  expect_throw(
    [&] { auto_aim::YOLOV5 detector(unavailable_gpu.path(), false); },
    "CPU-only build must reject GPU instead of falling back to CPU");
#endif
}

cv::Mat detection_frame(auto_aim::YOLOV5 & detector)
{
  cv::VideoCapture video(AUV_DEMO_PATH);
  if (!video.isOpened()) throw std::runtime_error("Failed to open the model test video");

  cv::Mat frame;
  for (int index = 0; index < 300 && video.read(frame); ++index) {
    if (!detector.detect(frame, index).empty()) return frame.clone();
  }
  throw std::runtime_error("No 0526 detection found in the first 300 demo frames");
}

void validate_detections(const std::list<auto_aim::Armor> & detections)
{
  for (const auto & armor : detections) {
    expect(armor.points.size() == 4, "each detection must contain four keypoints");
    expect(std::isfinite(armor.confidence), "detection confidence must be finite");
    for (const auto & point : armor.points) {
      expect(
        std::isfinite(point.x) && std::isfinite(point.y),
        "detection keypoints must be finite");
    }
  }
}

std::vector<double> benchmark(auto_aim::YOLOV5 & detector, const cv::Mat & frame)
{
  constexpr int iterations = 200;
  std::vector<double> timings;
  timings.reserve(iterations);
  for (int iteration = 0; iteration < iterations; ++iteration) {
    const auto begin = Clock::now();
    detector.detect(frame, iteration);
    const auto end = Clock::now();
    timings.push_back(std::chrono::duration<double, std::milli>(end - begin).count());
  }
  std::sort(timings.begin(), timings.end());
  return timings;
}

double median(const std::vector<double> & sorted)
{
  const auto middle = sorted.size() / 2;
  return sorted.size() % 2 == 0 ? (sorted[middle - 1] + sorted[middle]) / 2.0 : sorted[middle];
}

double p95(const std::vector<double> & sorted)
{
  return sorted[static_cast<std::size_t>(std::ceil(sorted.size() * 0.95)) - 1];
}

void compare_detections(
  const std::list<auto_aim::Armor> & cpu, const std::list<auto_aim::Armor> & gpu)
{
  expect(cpu.size() == gpu.size(), "CPU and GPU must return the same number of detections");
  std::vector<bool> matched(gpu.size(), false);
  for (const auto & cpu_armor : cpu) {
    bool found = false;
    std::size_t gpu_index = 0;
    for (const auto & gpu_armor : gpu) {
      bool equivalent =
        !matched[gpu_index] && cpu_armor.name == gpu_armor.name &&
        cpu_armor.color == gpu_armor.color && cpu_armor.type == gpu_armor.type &&
        std::abs(cpu_armor.confidence - gpu_armor.confidence) <= 0.02 &&
        cpu_armor.points.size() == gpu_armor.points.size();
      for (std::size_t point = 0; equivalent && point < cpu_armor.points.size(); ++point) {
        equivalent = cv::norm(cpu_armor.points[point] - gpu_armor.points[point]) <= 2.0;
      }
      if (equivalent) {
        matched[gpu_index] = true;
        found = true;
        break;
      }
      ++gpu_index;
    }
    expect(found, "CPU/GPU class, color, confidence, and keypoints must match");
  }
}
}  // namespace

int main()
{
  test_active_config();
  test_armor_layout();
  test_startup_failures();

  try {
    const TemporaryConfig cpu_config("CPU");
    auto_aim::YOLOV5 cpu_detector(cpu_config.path(), false);
    const cv::Mat image(270, 480, CV_8UC3, cv::Scalar::all(0));

    auto blue_output = make_output(0);
    const auto blue_detections = cpu_detector.postprocess(1.0, blue_output, image, 0);
    expect(
      blue_detections.size() == 1 && blue_detections.front().color == auto_aim::Color::blue,
      "0526 color output 0 must map to blue");

    auto red_output = make_output(1);
    const auto red_detections = cpu_detector.postprocess(1.0, red_output, image, 0);
    expect(
      red_detections.size() == 1 && red_detections.front().color == auto_aim::Color::red,
      "0526 color output 1 must map to red");

    for (const int base_class : {7, 8}) {
      auto base_output = make_output(0, base_class);
      const auto base_detections = cpu_detector.postprocess(1.0, base_output, image, 0);
      expect(
        base_detections.size() == 1 &&
          base_detections.front().name == auto_aim::ArmorName::base &&
          base_detections.front().type == auto_aim::ArmorType::small,
        "both 0526 base classes must use the small-armor geometry");
    }

    const auto frame = detection_frame(cpu_detector);
    const auto cpu_detections = cpu_detector.detect(frame, 0);
    expect(!cpu_detections.empty(), "0526 CPU inference must detect the demo target");
    validate_detections(cpu_detections);
    const auto cpu_timings = benchmark(cpu_detector, frame);
    std::cout << "ORT CPU detect median=" << median(cpu_timings)
              << " ms p95=" << p95(cpu_timings) << " ms\n";

#ifdef SP_VISION_TEST_WITH_CUDA
    const TemporaryConfig gpu_config("GPU");
    auto_aim::YOLOV5 gpu_detector(gpu_config.path(), false);
    const auto gpu_detections = gpu_detector.detect(frame, 0);
    validate_detections(gpu_detections);
    compare_detections(cpu_detections, gpu_detections);

    const auto gpu_timings = benchmark(gpu_detector, frame);
    const auto cpu_median = median(cpu_timings);
    const auto cpu_p95 = p95(cpu_timings);
    const auto gpu_median = median(gpu_timings);
    const auto gpu_p95 = p95(gpu_timings);
    std::cout << "ORT GPU detect median=" << gpu_median << " ms p95=" << gpu_p95 << " ms\n";
    expect(gpu_median < cpu_median, "GPU detect median must be lower than CPU");
    expect(gpu_p95 < cpu_p95, "GPU detect P95 must be lower than CPU");
#endif
  } catch (const std::exception & error) {
    std::cerr << "FAIL: 0526 model inference failed: " << error.what() << '\n';
    ++failures;
  }

  return failures == 0 ? 0 : 1;
}
