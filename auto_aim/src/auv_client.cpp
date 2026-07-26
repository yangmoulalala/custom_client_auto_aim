#include "io/auv_client.hpp"

#include <fmt/format.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <exception>
#include <list>
#include <opencv2/opencv.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/img_tools.hpp"
#include "tools/math_tools.hpp"

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明}"
  "{debug d        | false                  | 每秒输出输入、识别、跟踪和控制调试数据}"
  "{show           | false                  | 显示检测、EKF预测和瞄准结果，按q退出}"
  "{@config-path   | auto_aim/configs/AUVClient.yaml | YAML配置文件路径}";

namespace
{
constexpr std::size_t READ_STATUS_COUNT = 6;
constexpr char DEBUG_WINDOW_NAME[] = "AUV auto aim";
const cv::Scalar DETECTION_COLOR{0, 220, 0};
const cv::Scalar EKF_COLOR{255, 255, 0};
const cv::Scalar AIM_COLOR{0, 0, 255};

bool valid_projection(const std::vector<cv::Point2f> & points)
{
  return points.size() == 4 &&
         std::all_of(points.begin(), points.end(), [](const cv::Point2f & point) {
           return std::isfinite(point.x) && std::isfinite(point.y) && std::abs(point.x) < 1e6 &&
                  std::abs(point.y) < 1e6;
         });
}

cv::Point projection_center(const std::vector<cv::Point2f> & points)
{
  cv::Point2f center;
  for (const auto & point : points) center += point;
  center *= 1.0F / static_cast<float>(points.size());
  return {cvRound(center.x), cvRound(center.y)};
}

void draw_text_with_shadow(
  cv::Mat & image, const std::string & text, const cv::Point & origin,
  const cv::Scalar & color = {255, 255, 255}, double font_scale = 0.5, int thickness = 1)
{
  cv::putText(
    image, text, origin, cv::FONT_HERSHEY_SIMPLEX, font_scale, {0, 0, 0}, thickness + 2,
    cv::LINE_AA);
  cv::putText(
    image, text, origin, cv::FONT_HERSHEY_SIMPLEX, font_scale, color, thickness, cv::LINE_AA);
}

void draw_detections(cv::Mat & image, const std::list<auto_aim::Armor> & detections)
{
  for (const auto & armor : detections) {
    if (!valid_projection(armor.points)) continue;

    tools::draw_points(image, armor.points, DETECTION_COLOR, 1);
    const auto center = projection_center(armor.points);
    const auto label = fmt::format(
      "{:.2f} {} {}", armor.confidence, auto_aim::ARMOR_NAMES[armor.name],
      auto_aim::ARMOR_TYPES[armor.type]);
    draw_text_with_shadow(image, label, center + cv::Point(5, -5), DETECTION_COLOR, 0.45);
  }
}

void draw_cross(cv::Mat & image, const cv::Point & center, const cv::Scalar & color, int size)
{
  cv::line(image, center - cv::Point(size, 0), center + cv::Point(size, 0), color, 2, cv::LINE_AA);
  cv::line(image, center - cv::Point(0, size), center + cv::Point(0, size), color, 2, cv::LINE_AA);
}

void draw_ekf_model(
  cv::Mat & image, const auto_aim::Target & target, const auto_aim::Solver & solver)
{
  std::vector<cv::Point> armor_centers;
  const auto armor_xyza_list = target.armor_xyza_list();
  for (std::size_t index = 0; index < armor_xyza_list.size(); ++index) {
    const auto & xyza = armor_xyza_list[index];
    const auto points =
      solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
    if (!valid_projection(points)) continue;

    tools::draw_points(image, points, EKF_COLOR, 2);
    const auto center = projection_center(points);
    armor_centers.push_back(center);
    if (center.x >= 0 && center.x < image.cols && center.y >= 0 && center.y < image.rows) {
      draw_text_with_shadow(
        image, fmt::format("EKF {}", index), center + cv::Point(5, 15), EKF_COLOR, 0.4);
    }
  }

  if (armor_centers.size() > 1) {
    const std::vector<std::vector<cv::Point>> model_contour{armor_centers};
    cv::polylines(image, model_contour, true, EKF_COLOR, 1, cv::LINE_AA);
  }
}

void draw_aimed_armor(
  cv::Mat & image, const auto_aim::Target & target, const auto_aim::Aimer & aimer,
  const auto_aim::Solver & solver, const io::Command & command)
{
  if (!command.control || !aimer.debug_aim_point.valid) return;

  const auto & xyza = aimer.debug_aim_point.xyza;
  const auto points = solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
  if (!valid_projection(points)) return;

  tools::draw_points(image, points, AIM_COLOR, 3);
  const auto center = projection_center(points);
  draw_cross(image, center, AIM_COLOR, 8);
  draw_text_with_shadow(image, "AIM", center + cv::Point(10, -10), AIM_COLOR, 0.55, 2);
}

void draw_status_panel(
  cv::Mat & image, const std::list<auto_aim::Target> & targets, const std::string & tracker_state,
  std::uint64_t frame_count)
{
  std::vector<std::string> lines;
  if (targets.empty()) {
    lines = {"Target: none", "Spin: --"};
  } else {
    const auto & target = targets.front();
    const auto state = target.ekf_x();
    const double angular_velocity = state[7];
    const double rpm = angular_velocity * 60.0 / (2.0 * CV_PI);
    lines = {
      fmt::format(
        "Target: {} / {}", auto_aim::ARMOR_NAMES[target.name],
        auto_aim::ARMOR_TYPES[target.armor_type]),
      fmt::format("Spin: {:+.3f} rad/s ({:+.1f} rpm)", angular_velocity, rpm)};
  }
  lines.push_back(fmt::format("Tracker: {}  Frame: {}", tracker_state, frame_count));

  constexpr int padding = 8;
  constexpr int line_height = 19;
  constexpr double font_scale = 0.5;
  int max_text_width = 0;
  for (const auto & line : lines) {
    int baseline = 0;
    max_text_width = std::max(
      max_text_width,
      cv::getTextSize(line, cv::FONT_HERSHEY_SIMPLEX, font_scale, 1, &baseline).width);
  }

  const int panel_width = std::min(image.cols, max_text_width + padding * 2);
  const int panel_height =
    std::min(image.rows, static_cast<int>(lines.size()) * line_height + padding * 2 - 3);
  if (panel_width <= 0 || panel_height <= 0) return;

  cv::Mat panel = image(cv::Rect(0, 0, panel_width, panel_height));
  const cv::Mat background(panel.size(), panel.type(), cv::Scalar(0, 0, 0));
  cv::addWeighted(panel, 0.35, background, 0.65, 0.0, panel);

  for (std::size_t index = 0; index < lines.size(); ++index) {
    draw_text_with_shadow(
      image, lines[index], {padding, padding + 13 + static_cast<int>(index) * line_height},
      {255, 255, 255}, font_scale);
  }
}

cv::Mat make_debug_image(
  const cv::Mat & image, const std::list<auto_aim::Armor> & detections,
  const std::list<auto_aim::Target> & targets, const auto_aim::Aimer & aimer,
  const auto_aim::Solver & solver, const io::Command & command, const std::string & tracker_state,
  std::uint64_t frame_count)
{
  auto debug_image = image.clone();
  draw_detections(debug_image, detections);
  if (!targets.empty()) {
    const auto & target = targets.front();
    draw_ekf_model(debug_image, target, solver);
    draw_aimed_armor(debug_image, target, aimer, solver, command);
  }
  draw_status_panel(debug_image, targets, tracker_state, frame_count);
  return debug_image;
}

void initialize_debug_window(const cv::Size & image_size)
{
  cv::namedWindow(DEBUG_WINDOW_NAME, cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
  cv::resizeWindow(DEBUG_WINDOW_NAME, image_size.width, image_size.height);
}

const char * read_status_name(io::AUVReadStatus status)
{
  switch (status) {
    case io::AUVReadStatus::ok:
      return "ok";
    case io::AUVReadStatus::timeout:
      return "timeout";
    case io::AUVReadStatus::stale:
      return "stale";
    case io::AUVReadStatus::unmatched_imu:
      return "unmatched_imu";
    case io::AUVReadStatus::invalid_image:
      return "invalid_image";
    case io::AUVReadStatus::invalid_imu:
      return "invalid_imu";
  }
  return "unknown";
}
}  // namespace

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  const auto config_path = cli.get<std::string>("@config-path");
  const auto debug = cli.get<bool>("debug");
  const auto show = cli.get<bool>("show");
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  rclcpp::init(argc, argv);
  const auto logger = rclcpp::get_logger("auto_aim");

  try {
    const auto yaml = YAML::LoadFile(config_path);
    const auto calibration_width = yaml["calibration_image_width"].as<int>(0);
    const auto calibration_height = yaml["calibration_image_height"].as<int>(0);
    if (calibration_width <= 0 || calibration_height <= 0) {
      RCLCPP_ERROR(
        logger,
        "calibration_image_width and calibration_image_height must be set to the resolution "
        "used by camera_matrix");
      rclcpp::shutdown();
      return 1;
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger, "Failed to load configuration: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  int exit_code = 0;

  try {
    io::AUVClient client(config_path);
    auto_aim::YOLO detector(config_path, false);
    auto_aim::Solver solver(config_path);
    auto_aim::Tracker tracker(config_path, solver);
    auto_aim::Aimer aimer(config_path);
    auto_aim::Shooter shooter(config_path);

    cv::Size input_size;
    std::optional<bool> active_self_is_red;
    std::array<std::uint64_t, READ_STATUS_COUNT> read_status_counts{};
    auto last_debug_report = std::chrono::steady_clock::now();
    std::uint64_t frame_count = 0;
    while (rclcpp::ok()) {
      const auto iteration_start = std::chrono::steady_clock::now();
      io::AUVFrame frame;
      const auto read_status = client.read(frame);
      ++read_status_counts[static_cast<std::size_t>(read_status)];
      if (read_status != io::AUVReadStatus::ok) {
        const auto now = std::chrono::steady_clock::now();
        if (debug && tools::delta_time(now, last_debug_report) >= 1.0) {
          RCLCPP_INFO(
            logger, "%s",
            fmt::format(
              "input ok={} timeout={} stale={} unmatched_imu={} invalid_image={} "
              "invalid_imu={}; last_status={}",
              read_status_counts[0], read_status_counts[1], read_status_counts[2],
              read_status_counts[3], read_status_counts[4], read_status_counts[5],
              read_status_name(read_status))
              .c_str());
          read_status_counts.fill(0);
          last_debug_report = now;
        }
        client.publish(
          {false, false, 0.0, 0.0, 0.0},
          frame.source_stamp.sec == 0 && frame.source_stamp.nanosec == 0 ? nullptr : &frame);
        continue;
      }

      const auto team_color = client.team_color();
      if (!team_color) {
        static auto last_warning = std::chrono::steady_clock::time_point::min();
        const auto now = std::chrono::steady_clock::now();
        if (tools::delta_time(now, last_warning) >= 1.0) {
          RCLCPP_WARN(logger, "Waiting for self team color; publishing safe results");
          last_warning = now;
        }
        client.publish({false, false, 0.0, 0.0, 0.0}, &frame);
        continue;
      }

      if (!active_self_is_red || *active_self_is_red != team_color->self_is_red) {
        const auto enemy_color =
          team_color->self_is_red ? auto_aim::Color::blue : auto_aim::Color::red;
        tracker.set_enemy_color(enemy_color);
        active_self_is_red = team_color->self_is_red;
        RCLCPP_INFO(
          logger, "Self team is %s; targeting %s armors", team_color->self_is_red ? "red" : "blue",
          team_color->self_is_red ? "blue" : "red");
      }

      if (input_size.empty()) {
        if (!solver.set_image_size(frame.image.size())) {
          RCLCPP_ERROR(
            logger, "Input resolution %dx%d is incompatible with camera calibration",
            frame.image.cols, frame.image.rows);
          client.publish({false, false, 0.0, 0.0, 0.0}, &frame);
          exit_code = 1;
          break;
        }
        input_size = frame.image.size();
        if (show) initialize_debug_window(input_size);
        RCLCPP_INFO(
          logger, "Using fixed input resolution %dx%d", input_size.width, input_size.height);
      } else if (frame.image.size() != input_size) {
        static auto last_warning = std::chrono::steady_clock::time_point::min();
        const auto now = std::chrono::steady_clock::now();
        if (tools::delta_time(now, last_warning) >= 1.0) {
          RCLCPP_WARN(
            logger, "Resolution changed from %dx%d to %dx%d; frame rejected", input_size.width,
            input_size.height, frame.image.cols, frame.image.rows);
          last_warning = now;
        }
        client.publish({false, false, 0.0, 0.0, 0.0}, &frame);
        continue;
      }

      solver.set_R_gimbal2world(frame.orientation);
      const Eigen::Vector3d gimbal_ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);
      auto armors = detector.detect(frame.image);
      const auto detections = armors;
      auto targets = tracker.track(armors, frame.timestamp);
      auto command = aimer.aim(targets, frame.timestamp, client.bullet_speed(), true);
      command.shoot = shooter.shoot(command, aimer, targets, gimbal_ypr);
      client.publish(command, &frame, team_color->revision);
      ++frame_count;

      bool quit_requested = false;
      const bool publish_debug = client.debug_publish_ready();
      if (show || publish_debug) {
        auto debug_image = make_debug_image(
          frame.image, detections, targets, aimer, solver, command, tracker.state(), frame_count);
        if (publish_debug) client.publish_debug(debug_image, frame.source_stamp);
        if (show) {
          cv::imshow(DEBUG_WINDOW_NAME, debug_image);
          const int key = cv::waitKey(1) & 0xff;
          quit_requested = key == 'q' || key == 'Q' || key == 27;
        }
      }

      const auto now = std::chrono::steady_clock::now();
      if (debug && tools::delta_time(now, last_debug_report) >= 1.0) {
        const auto frame_age_ms =
          std::chrono::duration<double, std::milli>(now - frame.timestamp).count();
        const auto processing_ms =
          std::chrono::duration<double, std::milli>(now - iteration_start).count();
        RCLCPP_INFO(
          logger, "%s",
          fmt::format(
            "frame={} input(ok={} timeout={} stale={} unmatched_imu={} invalid_image={} "
            "invalid_imu={}) age_ms={:.2f} imu_ypr_deg=[{:.2f},{:.2f},{:.2f}] armors={} "
            "tracker={} targets={} command(control={} shoot={} yaw_deg={:.2f} pitch_deg={:.2f} "
            "distance_m={:.2f}) processing_ms={:.2f}",
            frame_count, read_status_counts[0], read_status_counts[1], read_status_counts[2],
            read_status_counts[3], read_status_counts[4], read_status_counts[5], frame_age_ms,
            gimbal_ypr[0] * 57.3, gimbal_ypr[1] * 57.3, gimbal_ypr[2] * 57.3, armors.size(),
            tracker.state(), targets.size(), command.control, command.shoot, command.yaw * 57.3,
            command.pitch * 57.3, command.horizon_distance, processing_ms)
            .c_str());
        read_status_counts.fill(0);
        last_debug_report = now;
      }
      if (quit_requested) break;
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger, "Fatal error: %s", e.what());
    exit_code = 1;
  }

  if (rclcpp::ok()) rclcpp::shutdown();
  return exit_code;
}
