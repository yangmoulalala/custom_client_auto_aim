#include <fmt/core.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cfloat>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <opencv2/opencv.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <stdexcept>
#include <string>
#include <vector>

#include "tools/logger.hpp"

const std::string keys =
  "{help h usage ?  |                               | 输出命令行参数说明}"
  "{@config-path    | auto_aim/configs/calibration.yaml      | YAML配置文件路径}"
  "{output-folder o | auto_aim/assets/ros_camera_calibration | 图片和标定结果输出目录}";

namespace
{
using ImageMsg = sensor_msgs::msg::Image;

std::vector<cv::Point3f> make_object_points(
  const cv::Size & pattern_size, double square_size_mm)
{
  std::vector<cv::Point3f> points;
  points.reserve(static_cast<std::size_t>(pattern_size.area()));
  for (int row = 0; row < pattern_size.height; ++row) {
    for (int col = 0; col < pattern_size.width; ++col) {
      points.emplace_back(
        static_cast<float>(col * square_size_mm), static_cast<float>(row * square_size_mm),
        0.0F);
    }
  }
  return points;
}

bool find_chessboard_corners(
  const cv::Mat & image, const cv::Size & pattern_size, std::vector<cv::Point2f> & corners)
{
  cv::Mat gray;
  cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
  corners.clear();
  const auto found = cv::findChessboardCorners(
    gray, pattern_size, corners, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
  if (!found) return false;

  cv::cornerSubPix(
    gray, corners, {11, 11}, {-1, -1},
    {cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 30, 0.001});
  return true;
}

bool image_to_bgr(const ImageMsg::ConstSharedPtr & msg, cv::Mat & image)
{
  if (!msg || msg->width == 0 || msg->height == 0) return false;

  int cv_type = 0;
  std::size_t bytes_per_pixel = 0;
  if (
    msg->encoding == sensor_msgs::image_encodings::BGR8 ||
    msg->encoding == sensor_msgs::image_encodings::RGB8) {
    cv_type = CV_8UC3;
    bytes_per_pixel = 3;
  } else if (msg->encoding == sensor_msgs::image_encodings::MONO8) {
    cv_type = CV_8UC1;
    bytes_per_pixel = 1;
  } else {
    return false;
  }

  const auto minimum_step = static_cast<std::size_t>(msg->width) * bytes_per_pixel;
  const auto required_size = static_cast<std::size_t>(msg->step) * msg->height;
  if (msg->step < minimum_step || msg->data.size() < required_size) return false;

  cv::Mat source(
    static_cast<int>(msg->height), static_cast<int>(msg->width), cv_type,
    const_cast<std::uint8_t *>(msg->data.data()), static_cast<std::size_t>(msg->step));
  if (msg->encoding == sensor_msgs::image_encodings::BGR8) {
    image = source;
  } else if (msg->encoding == sensor_msgs::image_encodings::RGB8) {
    cv::cvtColor(source, image, cv::COLOR_RGB2BGR);
  } else {
    cv::cvtColor(source, image, cv::COLOR_GRAY2BGR);
  }
  return true;
}

int next_image_index(const std::filesystem::path & output_folder)
{
  int index = 1;
  while (std::filesystem::exists(output_folder / fmt::format("{}.jpg", index))) ++index;
  return index;
}

std::string calibration_yaml(
  const cv::Size & image_size, const cv::Mat & camera_matrix, const cv::Mat & distort_coeffs,
  double reprojection_error)
{
  cv::Mat camera_matrix_64f;
  cv::Mat distort_coeffs_64f;
  camera_matrix.convertTo(camera_matrix_64f, CV_64F);
  distort_coeffs.convertTo(distort_coeffs_64f, CV_64F);
  if (!camera_matrix_64f.isContinuous()) camera_matrix_64f = camera_matrix_64f.clone();
  if (!distort_coeffs_64f.isContinuous()) distort_coeffs_64f = distort_coeffs_64f.clone();

  const auto * camera_begin = camera_matrix_64f.ptr<double>();
  const auto * distort_begin = distort_coeffs_64f.ptr<double>();
  std::vector<double> camera_data(camera_begin, camera_begin + camera_matrix_64f.total());
  std::vector<double> distort_data(distort_begin, distort_begin + distort_coeffs_64f.total());

  YAML::Emitter result;
  result << YAML::BeginMap;
  result << YAML::Comment(fmt::format("平均重投影误差: {:.4f}px", reprojection_error));
  result << YAML::Key << "calibration_image_width" << YAML::Value << image_size.width;
  result << YAML::Key << "calibration_image_height" << YAML::Value << image_size.height;
  result << YAML::Key << "camera_matrix" << YAML::Value << YAML::Flow << camera_data;
  result << YAML::Key << "distort_coeffs" << YAML::Value << YAML::Flow << distort_data;
  result << YAML::EndMap;
  return result.c_str();
}

double mean_reprojection_error(
  const std::vector<std::vector<cv::Point3f>> & object_points,
  const std::vector<std::vector<cv::Point2f>> & image_points, const cv::Mat & camera_matrix,
  const cv::Mat & distort_coeffs, const std::vector<cv::Mat> & rvecs,
  const std::vector<cv::Mat> & tvecs)
{
  double error_sum = 0.0;
  std::size_t total_points = 0;
  for (std::size_t i = 0; i < object_points.size(); ++i) {
    std::vector<cv::Point2f> reprojected;
    cv::projectPoints(
      object_points[i], rvecs[i], tvecs[i], camera_matrix, distort_coeffs, reprojected);
    total_points += reprojected.size();
    for (std::size_t j = 0; j < reprojected.size(); ++j) {
      error_sum += cv::norm(image_points[i][j] - reprojected[j]);
    }
  }
  return total_points == 0 ? std::numeric_limits<double>::infinity()
                           : error_sum / static_cast<double>(total_points);
}
}  // namespace

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  const auto config_path = cli.get<std::string>("@config-path");
  const std::filesystem::path output_folder = cli.get<std::string>("output-folder");

  YAML::Node yaml;
  cv::Size pattern_size;
  double square_size_mm = 0.0;
  std::string image_topic;
  try {
    yaml = YAML::LoadFile(config_path);
    pattern_size = {yaml["pattern_cols"].as<int>(), yaml["pattern_rows"].as<int>()};
    square_size_mm = yaml["square_size_mm"].as<double>();
    image_topic = yaml["ros_image_topic"] ? yaml["ros_image_topic"].as<std::string>()
                                          : std::string("/camera/image_raw");
  } catch (const std::exception & e) {
    tools::logger()->error("[ROSCalibration] Failed to load configuration: {}", e.what());
    return 1;
  }

  if (
    pattern_size.width <= 0 || pattern_size.height <= 0 || !std::isfinite(square_size_mm) ||
    square_size_mm <= 0.0) {
    tools::logger()->error("[ROSCalibration] Invalid chessboard configuration");
    return 1;
  }
  try {
    std::filesystem::create_directories(output_folder);
  } catch (const std::exception & e) {
    tools::logger()->error("[ROSCalibration] Failed to create output directory: {}", e.what());
    return 1;
  }

  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    auto node = std::make_shared<rclcpp::Node>("ros_camera_calibration");
    ImageMsg::ConstSharedPtr latest_image;
    std::uint64_t latest_sequence = 0;
    auto qos = rclcpp::SensorDataQoS().keep_last(1).best_effort();
    auto subscription =
      node->create_subscription<ImageMsg>(image_topic, qos, [&](ImageMsg::ConstSharedPtr msg) {
        latest_image = std::move(msg);
        ++latest_sequence;
      });

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    std::vector<std::vector<cv::Point3f>> object_points;
    std::vector<std::vector<cv::Point2f>> image_points;
    const auto board_points = make_object_points(pattern_size, square_size_mm);
    cv::Size image_size;
    cv::Mat current_image;
    std::vector<cv::Point2f> current_corners;
    ImageMsg::ConstSharedPtr current_owner;
    std::uint64_t processed_sequence = 0;
    std::uint64_t current_sequence = 0;
    std::uint64_t saved_sequence = 0;
    bool current_detection_ok = false;
    bool window_created = false;
    int image_index = next_image_index(output_folder);

    tools::logger()->info(
      "[ROSCalibration] Listening on {} for a {}x{} chessboard (inner corners)", image_topic,
      pattern_size.width, pattern_size.height);
    tools::logger()->info("[ROSCalibration] Press s to save a detected frame, q to calibrate");

    while (rclcpp::ok()) {
      executor.spin_some();

      if (latest_image && latest_sequence != processed_sequence) {
        current_owner = latest_image;
        current_sequence = latest_sequence;
        processed_sequence = latest_sequence;

        if (!image_to_bgr(current_owner, current_image)) {
          RCLCPP_WARN_THROTTLE(
            node->get_logger(), *node->get_clock(), 1000,
            "Unsupported or invalid image; expected bgr8, rgb8 or mono8");
          current_image.release();
          current_detection_ok = false;
        } else if (!image_size.empty() && current_image.size() != image_size) {
          RCLCPP_WARN_THROTTLE(
            node->get_logger(), *node->get_clock(), 1000,
            "Image resolution changed from %dx%d to %dx%d; frame rejected", image_size.width,
            image_size.height, current_image.cols, current_image.rows);
          current_image.release();
          current_detection_ok = false;
        } else {
          if (image_size.empty()) {
            image_size = current_image.size();
            tools::logger()->info(
              "[ROSCalibration] Input resolution: {}x{}", image_size.width, image_size.height);
          }

          current_detection_ok =
            find_chessboard_corners(current_image, pattern_size, current_corners);
          auto preview = current_image.clone();
          cv::drawChessboardCorners(preview, pattern_size, current_corners, current_detection_ok);
          const auto detection_text = current_detection_ok ? "board detected" : "board not found";
          const auto detection_color =
            current_detection_ok ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
          cv::putText(
            preview, detection_text, {20, 40}, cv::FONT_HERSHEY_SIMPLEX, 0.9, detection_color, 2);
          cv::putText(
            preview, fmt::format("accepted: {} | s: save | q: calibrate", image_points.size()),
            {20, 80}, cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);

          const auto preview_scale = std::min(
            {1.0, 1280.0 / static_cast<double>(preview.cols),
             800.0 / static_cast<double>(preview.rows)});
          if (preview_scale < 1.0) cv::resize(preview, preview, {}, preview_scale, preview_scale);
          cv::imshow("ROS camera calibration", preview);
          window_created = true;
        }
      }

      const auto key = cv::waitKey(1);
      if (key == 'q') break;
      if (key != 's') continue;
      if (current_image.empty() || !current_detection_ok) {
        tools::logger()->warn("[ROSCalibration] Frame not saved: calibration board not detected");
        continue;
      }
      if (saved_sequence == current_sequence) {
        tools::logger()->warn("[ROSCalibration] This frame has already been saved");
        continue;
      }

      const auto image_path = output_folder / fmt::format("{}.jpg", image_index);
      if (!cv::imwrite(image_path.string(), current_image)) {
        tools::logger()->error("[ROSCalibration] Failed to save {}", image_path.string());
        continue;
      }
      image_points.push_back(current_corners);
      object_points.push_back(board_points);
      saved_sequence = current_sequence;
      ++image_index;
      tools::logger()->info(
        "[ROSCalibration] Accepted sample {}: {}", image_points.size(), image_path.string());
    }

    if (window_created) cv::destroyWindow("ROS camera calibration");
    if (image_points.size() < 3) {
      tools::logger()->error(
        "[ROSCalibration] At least 3 valid samples are required; captured {}", image_points.size());
      exit_code = 1;
    } else {
      if (image_points.size() < 10) {
        tools::logger()->warn(
          "[ROSCalibration] Only {} samples captured; 10 or more varied poses are recommended",
          image_points.size());
      }

      cv::Mat camera_matrix;
      cv::Mat distort_coeffs;
      std::vector<cv::Mat> rvecs;
      std::vector<cv::Mat> tvecs;
      const auto criteria =
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, DBL_EPSILON);
      cv::calibrateCamera(
        object_points, image_points, image_size, camera_matrix, distort_coeffs, rvecs, tvecs,
        cv::CALIB_FIX_K3, criteria);

      const auto reprojection_error = mean_reprojection_error(
        object_points, image_points, camera_matrix, distort_coeffs, rvecs, tvecs);
      if (!std::isfinite(reprojection_error)) {
        throw std::runtime_error("Camera calibration produced a non-finite reprojection error");
      }
      auto result = calibration_yaml(image_size, camera_matrix, distort_coeffs, reprojection_error);
      const auto result_path = output_folder / "calibration_result.yaml";
      std::ofstream result_file(result_path);
      if (!result_file.is_open()) {
        throw std::runtime_error("Failed to open calibration result file for writing");
      }
      result_file << result << '\n';
      result_file.close();

      fmt::print("\n{}\n", result);
      tools::logger()->info(
        "[ROSCalibration] Calibration result saved to {}", result_path.string());
    }

    executor.remove_node(node);
  } catch (const std::exception & e) {
    tools::logger()->error("[ROSCalibration] Fatal error: {}", e.what());
    exit_code = 1;
  }

  if (rclcpp::ok()) rclcpp::shutdown();
  return exit_code;
}
