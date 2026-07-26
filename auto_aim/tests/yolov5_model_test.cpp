#include <cmath>
#include <exception>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/yolos/yolov5.hpp"

namespace
{
bool require(bool condition, const std::string & message)
{
  if (!condition) std::cerr << message << '\n';
  return condition;
}

cv::Mat make_output(int color_id)
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
  output.at<float>(0, 13 + 3) = 1;
  return output;
}
}  // namespace

int main()
{
  const std::vector<cv::Point2f> points{{10, 10}, {30, 10}, {30, 20}, {10, 20}};
  const cv::Rect box(10, 10, 20, 10);

  const auto_aim::Armor red_sentry(0, 0, 0.9F, box, points);
  const auto_aim::Armor blue_hero(1, 1, 0.9F, box, points);
  const auto_aim::Armor gray_base(2, 7, 0.9F, box, points);
  const auto_aim::Armor purple_big_base(3, 8, 0.9F, box, points);

  bool ok = true;
  ok &= require(
    red_sentry.color == auto_aim::Color::red && red_sentry.name == auto_aim::ArmorName::sentry &&
      red_sentry.type == auto_aim::ArmorType::small,
    "class G must map to a small red sentry armor");
  ok &= require(
    blue_hero.color == auto_aim::Color::blue && blue_hero.name == auto_aim::ArmorName::one &&
      blue_hero.type == auto_aim::ArmorType::big,
    "class 1 must map to a big blue hero armor");
  ok &= require(
    gray_base.color == auto_aim::Color::extinguish &&
      gray_base.name == auto_aim::ArmorName::base &&
      gray_base.type == auto_aim::ArmorType::small,
    "class Bs must map to a small gray base armor");
  ok &= require(
    purple_big_base.color == auto_aim::Color::purple &&
      purple_big_base.name == auto_aim::ArmorName::base &&
      purple_big_base.type == auto_aim::ArmorType::big,
    "class Bb must map to a big purple base armor");
  if (!ok) return 1;

  try {
    auto_aim::YOLOV5 detector(AUV_CLIENT_CONFIG, false);
    const cv::Mat image(270, 480, CV_8UC3, cv::Scalar::all(0));

    auto blue_output = make_output(0);
    const auto blue_detections = detector.postprocess(1.0, blue_output, image, 0);
    ok &= require(
      blue_detections.size() == 1 && blue_detections.front().color == auto_aim::Color::blue,
      "AUV 0526 color output 0 must map to blue");

    auto red_output = make_output(1);
    const auto red_detections = detector.postprocess(1.0, red_output, image, 0);
    ok &= require(
      red_detections.size() == 1 && red_detections.front().color == auto_aim::Color::red,
      "AUV 0526 color output 1 must map to red");

    const auto detections = detector.detect(image, 0);
    for (const auto & armor : detections) {
      ok &= require(armor.points.size() == 4, "each detection must contain four keypoints");
      for (const auto & point : armor.points) {
        ok &= require(
          std::isfinite(point.x) && std::isfinite(point.y),
          "detection keypoints must be finite");
      }
    }
  } catch (const std::exception & e) {
    std::cerr << "0526 model inference failed: " << e.what() << '\n';
    return 1;
  }

  return ok ? 0 : 1;
}
