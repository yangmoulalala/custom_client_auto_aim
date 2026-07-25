#include "io/auv_client.hpp"

#include <yaml-cpp/yaml.h>

#include <array>
#include <chrono>
#include <exception>
#include <opencv2/opencv.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/detector.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明}"
  "{debug d        | false                  | 每秒输出输入、识别、跟踪和控制调试数据}"
  "{show           | false                  | 显示二值图和装甲板检测结果，按q退出}"
  "{@config-path   | configs/AUVClient.yaml | YAML配置文件路径}";

namespace
{
constexpr std::size_t READ_STATUS_COUNT = 6;

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

  try {
    const auto yaml = YAML::LoadFile(config_path);
    const auto calibration_width = yaml["calibration_image_width"].as<int>(0);
    const auto calibration_height = yaml["calibration_image_height"].as<int>(0);
    if (calibration_width <= 0 || calibration_height <= 0) {
      tools::logger()->error(
        "[AUVClient] calibration_image_width and calibration_image_height must be set to the "
        "resolution used by camera_matrix");
      return 1;
    }
  } catch (const std::exception & e) {
    tools::logger()->error("[AUVClient] Failed to load configuration: {}", e.what());
    return 1;
  }

  rclcpp::init(argc, argv);
  int exit_code = 0;

  try {
    io::AUVClient client(config_path);
    auto_aim::Detector detector(config_path, show);
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
          tools::logger()->info(
            "[AUVClient][debug] input ok={} timeout={} stale={} unmatched_imu={} "
            "invalid_image={} invalid_imu={}; last_status={}",
            read_status_counts[0], read_status_counts[1], read_status_counts[2],
            read_status_counts[3], read_status_counts[4], read_status_counts[5],
            read_status_name(read_status));
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
          tools::logger()->warn(
            "[AUVClient] Waiting for self team color; publishing safe results");
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
        tools::logger()->info(
          "[AUVClient] Self team is {}; targeting {} armors",
          team_color->self_is_red ? "red" : "blue",
          team_color->self_is_red ? "blue" : "red");
      }

      if (input_size.empty()) {
        if (!solver.set_image_size(frame.image.size())) {
          tools::logger()->error(
            "[AUVClient] Input resolution {}x{} is incompatible with camera calibration",
            frame.image.cols, frame.image.rows);
          client.publish({false, false, 0.0, 0.0, 0.0}, &frame);
          exit_code = 1;
          break;
        }
        input_size = frame.image.size();
        tools::logger()->info(
          "[AUVClient] Using fixed input resolution {}x{}", input_size.width, input_size.height);
      } else if (frame.image.size() != input_size) {
        static auto last_warning = std::chrono::steady_clock::time_point::min();
        const auto now = std::chrono::steady_clock::now();
        if (tools::delta_time(now, last_warning) >= 1.0) {
          tools::logger()->warn(
            "[AUVClient] Resolution changed from {}x{} to {}x{}; frame rejected", input_size.width,
            input_size.height, frame.image.cols, frame.image.rows);
          last_warning = now;
        }
        client.publish({false, false, 0.0, 0.0, 0.0}, &frame);
        continue;
      }

      solver.set_R_gimbal2world(frame.orientation);
      const Eigen::Vector3d gimbal_ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);
      auto armors = detector.detect(frame.image);
      auto targets = tracker.track(armors, frame.timestamp);
      auto command = aimer.aim(targets, frame.timestamp, client.bullet_speed(), true);
      command.shoot = shooter.shoot(command, aimer, targets, gimbal_ypr);
      client.publish(command, &frame, team_color->revision);
      ++frame_count;

      const auto now = std::chrono::steady_clock::now();
      if (debug && tools::delta_time(now, last_debug_report) >= 1.0) {
        const auto frame_age_ms =
          std::chrono::duration<double, std::milli>(now - frame.timestamp).count();
        const auto processing_ms =
          std::chrono::duration<double, std::milli>(now - iteration_start).count();
        tools::logger()->info(
          "[AUVClient][debug] frame={} input(ok={} timeout={} stale={} unmatched_imu={} "
          "invalid_image={} invalid_imu={}) age_ms={:.2f} imu_ypr_deg=[{:.2f},{:.2f},{:.2f}] "
          "armors={} tracker={} targets={} command(control={} shoot={} yaw_deg={:.2f} "
          "pitch_deg={:.2f} distance_m={:.2f}) processing_ms={:.2f}",
          frame_count, read_status_counts[0], read_status_counts[1], read_status_counts[2],
          read_status_counts[3], read_status_counts[4], read_status_counts[5], frame_age_ms,
          gimbal_ypr[0] * 57.3, gimbal_ypr[1] * 57.3, gimbal_ypr[2] * 57.3, armors.size(),
          tracker.state(), targets.size(), command.control, command.shoot, command.yaw * 57.3,
          command.pitch * 57.3, command.horizon_distance, processing_ms);
        read_status_counts.fill(0);
        last_debug_report = now;
      }

      if (show && cv::waitKey(1) == 'q') break;
    }
  } catch (const std::exception & e) {
    tools::logger()->error("[AUVClient] Fatal error: {}", e.what());
    exit_code = 1;
  }

  if (rclcpp::ok()) rclcpp::shutdown();
  return exit_code;
}
