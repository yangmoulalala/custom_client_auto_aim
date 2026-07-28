#include "yolov5.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "tools/img_tools.hpp"
#include "tools/logger.hpp"

namespace auto_aim
{
YOLOV5::YOLOV5(const std::string & config_path, bool debug)
: debug_(debug), letterbox_(640, 640, CV_8UC3)
{
  auto yaml = YAML::LoadFile(config_path);

  model_path_ = yaml["yolov5_model_path"].as<std::string>();
  device_ = yaml["device"].as<std::string>();
  min_confidence_ = yaml["min_confidence"].as<double>();
  if (min_confidence_ < 0.0 || min_confidence_ > 1.0) {
    throw std::invalid_argument("min_confidence must be in [0, 1]");
  }
  int x = 0, y = 0, width = 0, height = 0;
  x = yaml["roi"]["x"].as<int>();
  y = yaml["roi"]["y"].as<int>();
  width = yaml["roi"]["width"].as<int>();
  height = yaml["roi"]["height"].as<int>();
  use_roi_ = yaml["use_roi"].as<bool>();
  auto color_order = std::string{"red_blue_gray_purple"};
  if (yaml["yolov5_color_order"]) {
    color_order = yaml["yolov5_color_order"].as<std::string>();
  }
  if (color_order == "red_blue_gray_purple") {
    swap_red_blue_ = false;
  } else if (color_order == "blue_red_gray_purple") {
    swap_red_blue_ = true;
  } else {
    throw std::invalid_argument("Unsupported yolov5_color_order: " + color_order);
  }
  if (x < 0 || y < 0 || width == 0 || width < -1 || height == 0 || height < -1) {
    throw std::invalid_argument(
      "ROI x/y must be non-negative and width/height must be -1 or positive");
  }
  roi_ = cv::Rect(x, y, width, height);
  offset_ = cv::Point2f(x, y);

  inference_ = std::make_unique<ONNXInference>(model_path_, device_);
  tools::logger()->info("[YOLOV5] ONNX Runtime {} initialized: {}", device_, model_path_);
}

YOLOV5::~YOLOV5() = default;

std::list<Armor> YOLOV5::detect(const cv::Mat & raw_img, int frame_count)
{
  if (raw_img.empty()) {
    tools::logger()->warn("Empty image received from camera");
    return {};
  }
  if (raw_img.type() != CV_8UC3) {
    tools::logger()->error("YOLO input must be a CV_8UC3 BGR image");
    return std::list<Armor>();
  }

  cv::Mat bgr_img;
  if (use_roi_) {
    const auto width = roi_.width == -1 ? raw_img.cols - roi_.x : roi_.width;
    const auto height = roi_.height == -1 ? raw_img.rows - roi_.y : roi_.height;
    const cv::Rect active_roi(roi_.x, roi_.y, width, height);
    if (
      width <= 0 || height <= 0 || active_roi.x + active_roi.width > raw_img.cols ||
      active_roi.y + active_roi.height > raw_img.rows) {
      tools::logger()->error("Configured YOLO ROI is outside the input image");
      return {};
    }
    roi_ = active_roi;
    bgr_img = raw_img(active_roi);
  } else {
    bgr_img = raw_img;
  }

  auto x_scale = static_cast<double>(640) / bgr_img.rows;
  auto y_scale = static_cast<double>(640) / bgr_img.cols;
  auto scale = std::min(x_scale, y_scale);
  auto h = static_cast<int>(bgr_img.rows * scale);
  auto w = static_cast<int>(bgr_img.cols * scale);

  letterbox_.setTo(cv::Scalar::all(0));
  auto roi = cv::Rect(0, 0, w, h);
  cv::resize(bgr_img, letterbox_(roi), {w, h});
  auto output = inference_->run(letterbox_);

  return parse(scale, output, raw_img, frame_count);
}

std::list<Armor> YOLOV5::parse(
  double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count)
{
  // for each row: xywh + classess
  std::vector<int> color_ids, num_ids;
  std::vector<float> confidences;
  std::vector<cv::Rect> boxes;
  std::vector<std::vector<cv::Point2f>> armors_key_points;
  for (int r = 0; r < output.rows; r++) {
    double score = output.at<float>(r, 8);
    if (!std::isfinite(score)) continue;
    score = sigmoid(score);

    if (score < score_threshold_) continue;

    std::vector<cv::Point2f> armor_key_points;

    //颜色和类别独热向量
    cv::Mat color_scores = output.row(r).colRange(9, 13);     //color
    cv::Mat classes_scores = output.row(r).colRange(13, 22);  //num
    bool classification_finite = true;
    for (int column = 9; column < 22; ++column) {
      classification_finite &= std::isfinite(output.at<float>(r, column));
    }
    if (!classification_finite) continue;
    cv::Point class_id, color_id;
    int _class_id, _color_id;
    cv::minMaxLoc(classes_scores, nullptr, nullptr, nullptr, &class_id);
    cv::minMaxLoc(color_scores, nullptr, nullptr, nullptr, &color_id);
    _class_id = class_id.x;
    _color_id = color_id.x;

    const auto inverse_scale = static_cast<float>(1.0 / scale);
    const std::vector<cv::Point2f> points{
      {output.at<float>(r, 0) * inverse_scale, output.at<float>(r, 1) * inverse_scale},
      {output.at<float>(r, 6) * inverse_scale, output.at<float>(r, 7) * inverse_scale},
      {output.at<float>(r, 4) * inverse_scale, output.at<float>(r, 5) * inverse_scale},
      {output.at<float>(r, 2) * inverse_scale, output.at<float>(r, 3) * inverse_scale}};
    if (std::any_of(points.begin(), points.end(), [](const cv::Point2f & point) {
          return !std::isfinite(point.x) || !std::isfinite(point.y);
        })) {
      continue;
    }
    armor_key_points = points;

    float min_x = armor_key_points[0].x;
    float max_x = armor_key_points[0].x;
    float min_y = armor_key_points[0].y;
    float max_y = armor_key_points[0].y;

    for (std::size_t i = 1; i < armor_key_points.size(); ++i) {
      if (armor_key_points[i].x < min_x) min_x = armor_key_points[i].x;
      if (armor_key_points[i].x > max_x) max_x = armor_key_points[i].x;
      if (armor_key_points[i].y < min_y) min_y = armor_key_points[i].y;
      if (armor_key_points[i].y > max_y) max_y = armor_key_points[i].y;
    }

    cv::Rect rect(min_x, min_y, max_x - min_x, max_y - min_y);

    color_ids.emplace_back(_color_id);
    num_ids.emplace_back(_class_id);
    boxes.emplace_back(rect);
    confidences.emplace_back(score);
    armors_key_points.emplace_back(armor_key_points);
  }

  std::vector<int> indices;
  cv::dnn::NMSBoxes(boxes, confidences, score_threshold_, nms_threshold_, indices);

  std::list<Armor> armors;
  for (const auto & i : indices) {
    auto color_id = color_ids[i];
    if (swap_red_blue_ && color_id <= 1) color_id = 1 - color_id;

    if (use_roi_) {
      armors.emplace_back(
        color_id, num_ids[i], confidences[i], boxes[i], armors_key_points[i], offset_);
    } else {
      armors.emplace_back(color_id, num_ids[i], confidences[i], boxes[i], armors_key_points[i]);
    }
  }

  for (auto it = armors.begin(); it != armors.end();) {
    if (!check_name(*it)) {
      it = armors.erase(it);
      continue;
    }

    if (!check_type(*it)) {
      it = armors.erase(it);
      continue;
    }
    it->center_norm = get_center_norm(bgr_img, it->center);
    ++it;
  }

  if (debug_) draw_detections(bgr_img, armors, frame_count);

  return armors;
}

bool YOLOV5::check_name(const Armor & armor) const
{
  auto name_ok = armor.name != ArmorName::not_armor;
  auto confidence_ok = armor.confidence > min_confidence_;

  // 保存不确定的图案，用于神经网络的迭代
  // if (name_ok && !confidence_ok) save(armor);

  return name_ok && confidence_ok;
}

bool YOLOV5::check_type(const Armor & armor) const
{
  bool name_ok;
  switch (armor.name) {
    case ArmorName::one:
      name_ok = armor.type == ArmorType::big;
      break;
    case ArmorName::sentry:
    case ArmorName::two:
    case ArmorName::outpost:
      name_ok = armor.type == ArmorType::small;
      break;
    case ArmorName::three:
    case ArmorName::four:
    case ArmorName::five:
    case ArmorName::base:
      name_ok = armor.type == ArmorType::small;
      break;
    default:
      name_ok = false;
  }

  // 保存异常的图案，用于神经网络的迭代
  // if (!name_ok) save(armor);

  return name_ok;
}

cv::Point2f YOLOV5::get_center_norm(const cv::Mat & bgr_img, const cv::Point2f & center) const
{
  auto h = bgr_img.rows;
  auto w = bgr_img.cols;
  return {center.x / w, center.y / h};
}

void YOLOV5::draw_detections(
  const cv::Mat & img, const std::list<Armor> & armors, int frame_count) const
{
  auto detection = img.clone();
  tools::draw_text(detection, fmt::format("[{}]", frame_count), {10, 30}, {255, 255, 255});
  for (const auto & armor : armors) {
    auto info = fmt::format(
      "{:.2f} {} {} {}", armor.confidence, COLORS[armor.color], ARMOR_NAMES[armor.name],
      ARMOR_TYPES[armor.type]);
    tools::draw_points(detection, armor.points, {0, 255, 0});
    tools::draw_text(detection, info, armor.center, {0, 255, 0});
  }

  if (use_roi_) {
    cv::Scalar green(0, 255, 0);
    cv::rectangle(detection, roi_, green, 2);
  }
  cv::resize(detection, detection, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
  cv::imshow("detection", detection);
}

double YOLOV5::sigmoid(double x)
{
  if (x > 0)
    return 1.0 / (1.0 + exp(-x));
  else
    return exp(x) / (1.0 + exp(x));
}

std::list<Armor> YOLOV5::postprocess(
  double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count)
{
  return parse(scale, output, bgr_img, frame_count);
}

}  // namespace auto_aim
