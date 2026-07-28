#ifndef AUTO_AIM__ONNX_INFERENCE_HPP
#define AUTO_AIM__ONNX_INFERENCE_HPP

#include <memory>
#include <opencv2/core.hpp>
#include <string>

namespace auto_aim
{
class ONNXInference
{
public:
  ONNXInference(const std::string & model_path, const std::string & device);
  ~ONNXInference();

  ONNXInference(const ONNXInference &) = delete;
  ONNXInference & operator=(const ONNXInference &) = delete;

  cv::Mat run(const cv::Mat & letterboxed_bgr);
  const std::string & device() const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace auto_aim

#endif  // AUTO_AIM__ONNX_INFERENCE_HPP
