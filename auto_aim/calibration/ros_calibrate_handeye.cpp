#include <fmt/core.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>  // Must be included before opencv2/core/eigen.hpp.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <fstream>
#include <memory>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <stdexcept>
#include <string>
#include <vector>

#include "calibration/ros_calibration_utils.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

const std::string keys =
  "{help h usage ?  |                              | 输出命令行参数说明}"
  "{@config-path    | auto_aim/configs/calibration.yaml     | YAML配置文件路径}"
  "{output-folder o | auto_aim/assets/ros_handeye_calibration | 样本和标定结果输出目录}";

namespace
{
using ImageMsg = sensor_msgs::msg::CompressedImage;
using ImuMsg = sensor_msgs::msg::Imu;

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

int next_sample_index(const std::filesystem::path & output_folder)
{
  int index = 1;
  while (std::filesystem::exists(output_folder / fmt::format("{}.jpg", index)) ||
         std::filesystem::exists(output_folder / fmt::format("{}.txt", index))) {
    ++index;
  }
  return index;
}

bool save_sample(
  const std::filesystem::path & output_folder, int index, const cv::Mat & image,
  const Eigen::Quaterniond & q)
{
  const auto image_path = output_folder / fmt::format("{}.jpg", index);
  const auto imu_path = output_folder / fmt::format("{}.txt", index);
  if (!cv::imwrite(image_path.string(), image)) return false;

  std::ofstream imu_file(imu_path);
  if (!imu_file.is_open()) return false;
  imu_file << fmt::format("{} {} {} {}", q.w(), q.x(), q.y(), q.z());
  return static_cast<bool>(imu_file);
}

std::vector<double> mat_data(const cv::Mat & matrix)
{
  cv::Mat matrix_64f;
  matrix.convertTo(matrix_64f, CV_64F);
  if (!matrix_64f.isContinuous()) matrix_64f = matrix_64f.clone();
  const auto * begin = matrix_64f.ptr<double>();
  return {begin, begin + matrix_64f.total()};
}

std::string handeye_yaml(
  const std::vector<double> & R_gimbal2imubody_data, const cv::Mat & R_camera2gimbal,
  const cv::Mat & t_camera2gimbal, const Eigen::Vector3d & camera_ypr)
{
  YAML::Emitter result;
  result << YAML::BeginMap;
  result << YAML::Comment(
    "R_gimbal2imubody is the configured gimbal/IMU axis mapping; verify it in the preview");
  result << YAML::Key << "R_gimbal2imubody" << YAML::Value << YAML::Flow << R_gimbal2imubody_data;
  result << YAML::Comment(fmt::format(
    "相机同理想情况的偏角: yaw{:.2f} pitch{:.2f} roll{:.2f} degree", camera_ypr[0], camera_ypr[1],
    camera_ypr[2]));
  result << YAML::Key << "R_camera2gimbal" << YAML::Value << YAML::Flow
         << mat_data(R_camera2gimbal);
  result << YAML::Key << "t_camera2gimbal" << YAML::Value << YAML::Flow
         << mat_data(t_camera2gimbal);
  result << YAML::EndMap;
  return result.c_str();
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

  cv::Size pattern_size;
  cv::Size calibration_image_size;
  double square_size_mm = 0.0;
  double sync_tolerance_ms = 5.0;
  double sync_wait_ms = 10.0;
  std::string image_topic;
  std::string imu_topic;
  std::vector<double> R_gimbal2imubody_data;
  std::vector<double> camera_matrix_data;
  std::vector<double> distort_coeffs_data;
  try {
    const auto yaml = YAML::LoadFile(config_path);
    pattern_size = {yaml["pattern_cols"].as<int>(), yaml["pattern_rows"].as<int>()};
    square_size_mm = yaml["square_size_mm"].as<double>();
    image_topic = yaml["ros_image_topic"] ? yaml["ros_image_topic"].as<std::string>()
                                          : std::string("/rm_video/image_processed");
    imu_topic =
      yaml["ros_imu_topic"] ? yaml["ros_imu_topic"].as<std::string>() : std::string("/imu/data");
    sync_tolerance_ms = yaml["sync_tolerance_ms"] ? yaml["sync_tolerance_ms"].as<double>() : 5.0;
    sync_wait_ms = yaml["sync_wait_ms"] ? yaml["sync_wait_ms"].as<double>() : 10.0;
    calibration_image_size = {
      yaml["calibration_image_width"].as<int>(), yaml["calibration_image_height"].as<int>()};
    R_gimbal2imubody_data = yaml["R_gimbal2imubody"].as<std::vector<double>>();
    camera_matrix_data = yaml["camera_matrix"].as<std::vector<double>>();
    distort_coeffs_data = yaml["distort_coeffs"].as<std::vector<double>>();
  } catch (const std::exception & e) {
    tools::logger()->error("[ROSHandEye] Failed to load configuration: {}", e.what());
    return 1;
  }

  if (
    pattern_size.width <= 0 || pattern_size.height <= 0 || calibration_image_size.width <= 0 ||
    calibration_image_size.height <= 0 || !std::isfinite(square_size_mm) ||
    square_size_mm <= 0.0 || !std::isfinite(sync_tolerance_ms) || sync_tolerance_ms < 0.0 ||
    !std::isfinite(sync_wait_ms) || sync_wait_ms < 0.0 || R_gimbal2imubody_data.size() != 9 ||
    camera_matrix_data.size() != 9 || distort_coeffs_data.size() < 4) {
    tools::logger()->error(
      "[ROSHandEye] Invalid configuration; run camera calibration and copy its result first");
    return 1;
  }

  Eigen::Matrix<double, 3, 3, Eigen::RowMajor> R_gimbal2imubody(R_gimbal2imubody_data.data());
  const auto orthogonal_error =
    (R_gimbal2imubody.transpose() * R_gimbal2imubody - Eigen::Matrix3d::Identity()).norm();
  if (orthogonal_error > 1e-6 || std::abs(R_gimbal2imubody.determinant() - 1.0) > 1e-6) {
    tools::logger()->error("[ROSHandEye] R_gimbal2imubody must be a right-handed rotation matrix");
    return 1;
  }

  cv::Mat camera_matrix(3, 3, CV_64F, camera_matrix_data.data());
  camera_matrix = camera_matrix.clone();
  cv::Mat distort_coeffs(
    1, static_cast<int>(distort_coeffs_data.size()), CV_64F, distort_coeffs_data.data());
  distort_coeffs = distort_coeffs.clone();
  if (
    !cv::checkRange(camera_matrix) || !cv::checkRange(distort_coeffs) ||
    camera_matrix.at<double>(0, 0) <= 0.0 || camera_matrix.at<double>(1, 1) <= 0.0) {
    tools::logger()->error("[ROSHandEye] camera_matrix or distort_coeffs is invalid");
    return 1;
  }

  try {
    std::filesystem::create_directories(output_folder);
  } catch (const std::exception & e) {
    tools::logger()->error("[ROSHandEye] Failed to create output directory: {}", e.what());
    return 1;
  }

  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    auto node = std::make_shared<rclcpp::Node>("ros_handeye_calibration");
    ImageMsg::ConstSharedPtr latest_image;
    std::uint64_t latest_image_sequence = 0;
    std::deque<ImuMsg::ConstSharedPtr> imu_buffer;
    auto qos = rclcpp::SensorDataQoS().keep_last(1).best_effort();
    auto image_subscription =
      node->create_subscription<ImageMsg>(image_topic, qos, [&](ImageMsg::ConstSharedPtr msg) {
        latest_image = std::move(msg);
        ++latest_image_sequence;
      });
    auto imu_subscription =
      node->create_subscription<ImuMsg>(imu_topic, qos, [&](ImuMsg::ConstSharedPtr msg) {
        imu_buffer.push_back(std::move(msg));
        constexpr std::size_t MAX_IMU_BUFFER_SIZE = 200;
        while (imu_buffer.size() > MAX_IMU_BUFFER_SIZE) imu_buffer.pop_front();
      });

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    const auto tolerance_ns = static_cast<std::int64_t>(sync_tolerance_ms * 1e6);
    const auto sync_wait = std::chrono::duration<double, std::milli>(sync_wait_ms);
    const auto board_points = make_object_points(pattern_size, square_size_mm);
    std::vector<cv::Mat> R_gimbal2world_list;
    std::vector<cv::Mat> t_gimbal2world_list;
    std::vector<cv::Mat> board_rvecs;
    std::vector<cv::Mat> board_tvecs;

    ImageMsg::ConstSharedPtr pending_image;
    std::uint64_t pending_sequence = 0;
    std::uint64_t processed_sequence = 0;
    std::chrono::steady_clock::time_point pending_since;
    ImageMsg::ConstSharedPtr current_owner;
    cv::Mat current_image;
    Eigen::Quaterniond current_q = Eigen::Quaterniond::Identity();
    Eigen::Matrix3d current_R_gimbal2world = Eigen::Matrix3d::Identity();
    std::vector<cv::Point2f> current_corners;
    std::uint64_t current_sequence = 0;
    std::uint64_t saved_sequence = 0;
    bool current_detection_ok = false;
    bool window_created = false;
    int sample_index = next_sample_index(output_folder);

    tools::logger()->info(
      "[ROSHandEye] Listening on image={} imu={} (tolerance {:.3f} ms)", image_topic, imu_topic,
      sync_tolerance_ms);
    tools::logger()->info(
      "[ROSHandEye] Keep the board fixed; press s at varied gimbal poses, q to calibrate");

    while (rclcpp::ok()) {
      executor.spin_some();

      if (
        latest_image && latest_image_sequence != processed_sequence &&
        latest_image_sequence != pending_sequence) {
        pending_image = latest_image;
        pending_sequence = latest_image_sequence;
        pending_since = std::chrono::steady_clock::now();
      }

      if (pending_image) {
        const auto image_stamp_ns = calibration::stamp_to_ns(pending_image->header.stamp);
        auto imu = calibration::nearest_imu(imu_buffer, image_stamp_ns, tolerance_ns);
        if (imu) {
          processed_sequence = pending_sequence;
          current_sequence = pending_sequence;
          current_owner = pending_image;
          pending_image.reset();

          if (!calibration::decode_compressed_image(current_owner, current_image)) {
            RCLCPP_WARN_THROTTLE(
              node->get_logger(), *node->get_clock(), 1000,
              "Invalid compressed image; expected a decodable JPEG payload");
            current_image.release();
            current_detection_ok = false;
          } else if (current_image.size() != calibration_image_size) {
            throw std::runtime_error(fmt::format(
              "Input resolution {}x{} differs from calibrated resolution {}x{}", current_image.cols,
              current_image.rows, calibration_image_size.width, calibration_image_size.height));
          } else if (!calibration::imu_orientation(imu, current_q)) {
            RCLCPP_WARN_THROTTLE(
              node->get_logger(), *node->get_clock(), 1000, "Invalid IMU orientation");
            current_image.release();
            current_detection_ok = false;
          } else {
            const Eigen::Matrix3d R_imubody2imuabs = current_q.toRotationMatrix();
            current_R_gimbal2world =
              R_gimbal2imubody.transpose() * R_imubody2imuabs * R_gimbal2imubody;
            const Eigen::Vector3d ypr = tools::eulers(current_R_gimbal2world, 2, 1, 0) * 57.3;

            current_detection_ok =
              find_chessboard_corners(current_image, pattern_size, current_corners);
            auto preview = current_image.clone();
            cv::drawChessboardCorners(preview, pattern_size, current_corners, current_detection_ok);
            cv::putText(
              preview, fmt::format("yaw {:.2f} pitch {:.2f} roll {:.2f}", ypr[0], ypr[1], ypr[2]),
              {20, 40}, cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
            cv::putText(
              preview,
              fmt::format(
                "accepted: {} | {} | s: save | q: calibrate", board_rvecs.size(),
                current_detection_ok ? "board detected" : "board not found"),
              {20, 80}, cv::FONT_HERSHEY_SIMPLEX, 0.7,
              current_detection_ok ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255), 2);
            const auto preview_scale = std::min(
              {1.0, 1280.0 / static_cast<double>(preview.cols),
               800.0 / static_cast<double>(preview.rows)});
            if (preview_scale < 1.0) cv::resize(preview, preview, {}, preview_scale, preview_scale);
            cv::imshow("ROS hand-eye calibration", preview);
            window_created = true;
          }
        } else if (std::chrono::steady_clock::now() - pending_since >= sync_wait) {
          RCLCPP_WARN_THROTTLE(
            node->get_logger(), *node->get_clock(), 1000,
            "Dropping image without a matching IMU sample");
          processed_sequence = pending_sequence;
          pending_image.reset();
        }
      }

      const auto key = cv::waitKey(1);
      if (key == 'q') break;
      if (key != 's') continue;
      if (current_image.empty() || !current_detection_ok) {
        tools::logger()->warn("[ROSHandEye] Sample not saved: calibration board not detected");
        continue;
      }
      if (saved_sequence == current_sequence) {
        tools::logger()->warn("[ROSHandEye] This synchronized frame has already been saved");
        continue;
      }

      cv::Mat rvec;
      cv::Mat tvec;
      if (!cv::solvePnP(
            board_points, current_corners, camera_matrix, distort_coeffs, rvec, tvec, false,
            cv::SOLVEPNP_IPPE)) {
        tools::logger()->warn("[ROSHandEye] solvePnP failed; sample not saved");
        continue;
      }
      if (!save_sample(output_folder, sample_index, current_image, current_q)) {
        tools::logger()->error("[ROSHandEye] Failed to save sample {}", sample_index);
        continue;
      }

      cv::Mat R_gimbal2world_cv;
      cv::eigen2cv(current_R_gimbal2world, R_gimbal2world_cv);
      R_gimbal2world_list.push_back(R_gimbal2world_cv);
      t_gimbal2world_list.push_back(cv::Mat::zeros(3, 1, CV_64F));
      board_rvecs.push_back(rvec);
      board_tvecs.push_back(tvec);
      saved_sequence = current_sequence;
      ++sample_index;
      tools::logger()->info("[ROSHandEye] Accepted sample {}", board_rvecs.size());
    }

    if (window_created) cv::destroyWindow("ROS hand-eye calibration");
    if (board_rvecs.size() < 3) {
      tools::logger()->error(
        "[ROSHandEye] At least 3 valid poses are required; captured {}", board_rvecs.size());
      exit_code = 1;
    } else {
      if (board_rvecs.size() < 10) {
        tools::logger()->warn(
          "[ROSHandEye] Only {} poses captured; 10 or more diverse rotations are recommended",
          board_rvecs.size());
      }

      cv::Mat R_camera2gimbal;
      cv::Mat t_camera2gimbal_mm;
      cv::calibrateHandEye(
        R_gimbal2world_list, t_gimbal2world_list, board_rvecs, board_tvecs, R_camera2gimbal,
        t_camera2gimbal_mm);
      cv::Mat t_camera2gimbal = t_camera2gimbal_mm / 1000.0;
      if (
        R_camera2gimbal.empty() || t_camera2gimbal.empty() || !cv::checkRange(R_camera2gimbal) ||
        !cv::checkRange(t_camera2gimbal) ||
        std::abs(cv::determinant(R_camera2gimbal) - 1.0) > 1e-3) {
        throw std::runtime_error("Hand-eye calibration produced an invalid transform");
      }

      Eigen::Matrix3d R_camera2gimbal_eigen;
      cv::cv2eigen(R_camera2gimbal, R_camera2gimbal_eigen);
      const Eigen::Matrix3d R_gimbal2ideal{{0, -1, 0}, {0, 0, -1}, {1, 0, 0}};
      const Eigen::Matrix3d R_camera2ideal = R_gimbal2ideal * R_camera2gimbal_eigen;
      const Eigen::Vector3d camera_ypr = tools::eulers(R_camera2ideal, 1, 0, 2) * 57.3;

      const auto result =
        handeye_yaml(R_gimbal2imubody_data, R_camera2gimbal, t_camera2gimbal, camera_ypr);
      const auto result_path = output_folder / "handeye_result.yaml";
      std::ofstream result_file(result_path);
      if (!result_file.is_open()) {
        throw std::runtime_error("Failed to open hand-eye result file for writing");
      }
      result_file << result << '\n';
      result_file.close();
      fmt::print("\n{}\n", result);
      tools::logger()->info("[ROSHandEye] Result saved to {}", result_path.string());
    }

    executor.remove_node(node);
  } catch (const std::exception & e) {
    tools::logger()->error("[ROSHandEye] Fatal error: {}", e.what());
    exit_code = 1;
  }

  if (rclcpp::ok()) rclcpp::shutdown();
  return exit_code;
}
