#ifndef IO__AUV_CLIENT_HPP
#define IO__AUV_CLIENT_HPP

#include <Eigen/Geometry>
#include <builtin_interfaces/msg/time.hpp>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <opencv2/core.hpp>
#include <optional>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <thread>

#include "io/command.hpp"

namespace io
{
struct AUVFrame
{
  cv::Mat image;
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
  std::chrono::steady_clock::time_point timestamp;
  builtin_interfaces::msg::Time source_stamp;
};

enum class AUVReadStatus
{
  ok,
  timeout,
  stale,
  unmatched_imu,
  invalid_image,
  invalid_imu
};

struct AUVTeamColor
{
  bool self_is_red;
  std::uint64_t revision;
};

class AUVClient
{
public:
  explicit AUVClient(const std::string & config_path);
  ~AUVClient();

  AUVClient(const AUVClient &) = delete;
  AUVClient & operator=(const AUVClient &) = delete;

  AUVReadStatus read(AUVFrame & frame);
  void publish(
    const Command & command, const AUVFrame * frame = nullptr,
    std::optional<std::uint64_t> team_color_revision = std::nullopt);
  bool debug_publish_ready();
  void publish_debug(cv::Mat image, const builtin_interfaces::msg::Time & source_stamp);

  double bullet_speed() const;
  std::optional<AUVTeamColor> team_color() const;

private:
  using ImageMsg = sensor_msgs::msg::CompressedImage;
  using ImuMsg = sensor_msgs::msg::Imu;
  using BoolMsg = std_msgs::msg::Bool;

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Subscription<ImageMsg>::SharedPtr image_subscription_;
  rclcpp::Subscription<ImuMsg>::SharedPtr imu_subscription_;
  rclcpp::Subscription<BoolMsg>::SharedPtr self_is_red_subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr result_publisher_;
  rclcpp::Publisher<ImageMsg>::SharedPtr debug_publisher_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spin_thread_;

  std::mutex data_mutex_;
  std::condition_variable data_condition_;
  ImageMsg::ConstSharedPtr latest_image_;
  std::deque<ImuMsg::ConstSharedPtr> imu_buffer_;
  std::uint64_t latest_image_sequence_{0};
  std::uint64_t consumed_image_sequence_{0};
  bool stopping_{false};

  mutable std::mutex team_color_mutex_;
  std::optional<bool> self_is_red_;
  std::uint64_t team_color_revision_{0};

  double bullet_speed_{23.0};
  double upstream_latency_ms_{0.0};
  double sync_tolerance_ms_{5.0};
  double sync_wait_ms_{10.0};
  double max_frame_age_ms_{100.0};
  double command_timeout_ms_{200.0};

  std::mutex result_mutex_;
  std::int64_t last_result_steady_ns_{0};
  bool watchdog_result_sent_{false};

  struct DebugFrame
  {
    cv::Mat image;
    builtin_interfaces::msg::Time source_stamp;
  };
  std::mutex debug_mutex_;
  std::condition_variable debug_condition_;
  std::optional<DebugFrame> pending_debug_frame_;
  bool debug_stopping_{false};
  std::thread debug_thread_;

  void image_callback(ImageMsg::ConstSharedPtr msg);
  void imu_callback(ImuMsg::ConstSharedPtr msg);
  void self_is_red_callback(BoolMsg::ConstSharedPtr msg);
  ImuMsg::ConstSharedPtr nearest_imu_locked(std::int64_t image_stamp_ns) const;
  bool has_matching_imu_locked(std::int64_t image_stamp_ns) const;
  AUVReadStatus convert_image(const ImageMsg::ConstSharedPtr & msg, AUVFrame & frame) const;
  AUVReadStatus convert_imu(const ImuMsg::ConstSharedPtr & msg, AUVFrame & frame) const;
  void publish_json(const Command & command, const builtin_interfaces::msg::Time * stamp);
  bool has_debug_subscribers() const;
  void debug_worker();
  void watchdog_callback();
};

}  // namespace io

#endif  // IO__AUV_CLIENT_HPP
