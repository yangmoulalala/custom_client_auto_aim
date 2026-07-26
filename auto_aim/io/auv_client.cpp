#include "auv_client.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <stdexcept>
#include <vector>

namespace io
{
namespace
{
template <typename T>
T yaml_value_or(const YAML::Node & yaml, const std::string & key, const T & fallback)
{
  return yaml[key] ? yaml[key].as<T>() : fallback;
}

std::int64_t stamp_to_ns(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<std::int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
}

std::int64_t steady_now_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
           std::chrono::steady_clock::now().time_since_epoch())
    .count();
}

Command safe_command() { return {false, false, 0.0, 0.0, 0.0}; }

rclcpp::Logger auto_aim_logger() { return rclcpp::get_logger("auto_aim"); }
}  // namespace

AUVClient::AUVClient(const std::string & config_path)
{
  if (!rclcpp::ok()) {
    throw std::runtime_error("rclcpp must be initialized before constructing AUVClient");
  }

  const auto yaml = YAML::LoadFile(config_path);
  const auto image_topic = yaml_value_or<std::string>(yaml, "ros_image_topic", "/camera/image_raw");
  const auto imu_topic = yaml_value_or<std::string>(yaml, "ros_imu_topic", "/imu/data");
  const auto result_topic =
    yaml_value_or<std::string>(yaml, "ros_result_topic", "/auto_aim/result");
  const auto debug_topic =
    yaml_value_or<std::string>(yaml, "ros_debug_topic", "/auto_aim/debug");
  const auto self_is_red_topic =
    yaml_value_or<std::string>(yaml, "ros_self_is_red_topic", "/rm_mqtt/self_is_red");

  bullet_speed_ = yaml_value_or<double>(yaml, "bullet_speed", 23.0);
  upstream_latency_ms_ = yaml_value_or<double>(yaml, "upstream_latency_ms", 0.0);
  sync_tolerance_ms_ = yaml_value_or<double>(yaml, "sync_tolerance_ms", 5.0);
  sync_wait_ms_ = yaml_value_or<double>(yaml, "sync_wait_ms", 10.0);
  max_frame_age_ms_ = yaml_value_or<double>(yaml, "max_frame_age_ms", 100.0);
  command_timeout_ms_ = yaml_value_or<double>(yaml, "command_timeout_ms", 200.0);

  if (
    !std::isfinite(bullet_speed_) || !std::isfinite(upstream_latency_ms_) ||
    !std::isfinite(sync_tolerance_ms_) || !std::isfinite(sync_wait_ms_) ||
    !std::isfinite(max_frame_age_ms_) || !std::isfinite(command_timeout_ms_) ||
    bullet_speed_ <= 0.0 || upstream_latency_ms_ < 0.0 || sync_tolerance_ms_ < 0.0 ||
    sync_wait_ms_ < 0.0 || max_frame_age_ms_ <= 0.0 || command_timeout_ms_ <= 0.0 ||
    upstream_latency_ms_ >= max_frame_age_ms_) {
    throw std::runtime_error("Invalid AUVClient timing or bullet-speed configuration");
  }

  node_ = std::make_shared<rclcpp::Node>("auv_client");
  auto sensor_qos = rclcpp::SensorDataQoS().keep_last(1).best_effort();

  image_subscription_ = node_->create_subscription<ImageMsg>(
    image_topic, sensor_qos,
    [this](ImageMsg::ConstSharedPtr msg) { image_callback(std::move(msg)); });

  imu_subscription_ = node_->create_subscription<ImuMsg>(
    imu_topic, sensor_qos, [this](ImuMsg::ConstSharedPtr msg) { imu_callback(std::move(msg)); });

  self_is_red_subscription_ = node_->create_subscription<BoolMsg>(
    self_is_red_topic, sensor_qos,
    [this](BoolMsg::ConstSharedPtr msg) { self_is_red_callback(std::move(msg)); });

  result_publisher_ =
    node_->create_publisher<std_msgs::msg::String>(result_topic, rclcpp::QoS(1).best_effort());
  debug_publisher_ = node_->create_publisher<ImageMsg>(debug_topic, sensor_qos);

  const auto watchdog_period_ms = std::max(10.0, command_timeout_ms_ / 2.0);
  watchdog_timer_ = node_->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double, std::milli>(watchdog_period_ms)),
    [this]() { watchdog_callback(); });

  last_result_steady_ns_ = steady_now_ns();
  executor_.add_node(node_);
  try {
    spin_thread_ = std::thread([this]() { executor_.spin(); });
    debug_thread_ = std::thread([this]() { debug_worker(); });
  } catch (...) {
    {
      std::lock_guard<std::mutex> lock(debug_mutex_);
      debug_stopping_ = true;
    }
    debug_condition_.notify_all();
    if (debug_thread_.joinable()) debug_thread_.join();
    executor_.cancel();
    if (spin_thread_.joinable()) spin_thread_.join();
    executor_.remove_node(node_);
    throw;
  }

  RCLCPP_INFO(
    auto_aim_logger(), "Started: image=%s imu=%s self_is_red=%s result=%s debug=%s",
    image_topic.c_str(), imu_topic.c_str(), self_is_red_topic.c_str(), result_topic.c_str(),
    debug_topic.c_str());
}

AUVClient::~AUVClient()
{
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    stopping_ = true;
  }
  data_condition_.notify_all();

  {
    std::lock_guard<std::mutex> lock(debug_mutex_);
    debug_stopping_ = true;
    pending_debug_frame_.reset();
  }
  debug_condition_.notify_all();
  if (debug_thread_.joinable()) debug_thread_.join();

  if (watchdog_timer_) watchdog_timer_->cancel();
  executor_.cancel();
  if (spin_thread_.joinable()) spin_thread_.join();
  if (node_) executor_.remove_node(node_);
}

void AUVClient::image_callback(ImageMsg::ConstSharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_image_ = std::move(msg);
    ++latest_image_sequence_;
  }
  data_condition_.notify_all();
}

void AUVClient::imu_callback(ImuMsg::ConstSharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    imu_buffer_.push_back(std::move(msg));
    constexpr std::size_t MAX_IMU_BUFFER_SIZE = 200;
    while (imu_buffer_.size() > MAX_IMU_BUFFER_SIZE) imu_buffer_.pop_front();
  }
  data_condition_.notify_all();
}

void AUVClient::self_is_red_callback(BoolMsg::ConstSharedPtr msg)
{
  if (!msg) return;

  bool changed = false;
  {
    std::lock_guard<std::mutex> lock(team_color_mutex_);
    changed = !self_is_red_.has_value() || *self_is_red_ != msg->data;
    if (!changed) return;
    self_is_red_ = msg->data;
    ++team_color_revision_;
  }

  RCLCPP_INFO(
    auto_aim_logger(), "Team color updated: self=%s enemy=%s", msg->data ? "red" : "blue",
    msg->data ? "blue" : "red");

  // Immediately revoke any command produced for the previous team color.
  std::lock_guard<std::mutex> lock(result_mutex_);
  publish_json(safe_command(), nullptr);
  last_result_steady_ns_ = steady_now_ns();
  watchdog_result_sent_ = false;
}

AUVClient::ImuMsg::ConstSharedPtr AUVClient::nearest_imu_locked(std::int64_t image_stamp_ns) const
{
  ImuMsg::ConstSharedPtr nearest;
  auto nearest_delta = std::numeric_limits<std::int64_t>::max();

  for (const auto & imu : imu_buffer_) {
    const auto imu_stamp_ns = stamp_to_ns(imu->header.stamp);
    const auto delta = imu_stamp_ns >= image_stamp_ns ? imu_stamp_ns - image_stamp_ns
                                                      : image_stamp_ns - imu_stamp_ns;
    if (delta < nearest_delta) {
      nearest_delta = delta;
      nearest = imu;
      if (delta == 0) break;
    }
  }

  const auto tolerance_ns = static_cast<std::int64_t>(sync_tolerance_ms_ * 1e6);
  return nearest && nearest_delta <= tolerance_ns ? nearest : nullptr;
}

bool AUVClient::has_matching_imu_locked(std::int64_t image_stamp_ns) const
{
  return static_cast<bool>(nearest_imu_locked(image_stamp_ns));
}

AUVReadStatus AUVClient::convert_image(const ImageMsg::ConstSharedPtr & msg, AUVFrame & frame) const
{
  if (!msg || msg->data.empty()) return AUVReadStatus::invalid_image;

  try {
    frame.image = cv::imdecode(msg->data, cv::IMREAD_COLOR);
  } catch (const cv::Exception & e) {
    RCLCPP_WARN_THROTTLE(
      auto_aim_logger(), *node_->get_clock(), 1000, "Failed to decode compressed image: %s",
      e.what());
    return AUVReadStatus::invalid_image;
  }

  if (frame.image.empty()) {
    RCLCPP_WARN_THROTTLE(
      auto_aim_logger(), *node_->get_clock(), 1000,
      "Compressed image payload could not be decoded (format: %s)", msg->format.c_str());
    return AUVReadStatus::invalid_image;
  }
  return AUVReadStatus::ok;
}

AUVReadStatus AUVClient::convert_imu(const ImuMsg::ConstSharedPtr & msg, AUVFrame & frame) const
{
  if (!msg || msg->orientation_covariance[0] == -1.0) return AUVReadStatus::invalid_imu;

  const auto & orientation = msg->orientation;
  if (
    !std::isfinite(orientation.w) || !std::isfinite(orientation.x) ||
    !std::isfinite(orientation.y) || !std::isfinite(orientation.z)) {
    return AUVReadStatus::invalid_imu;
  }

  Eigen::Quaterniond q(orientation.w, orientation.x, orientation.y, orientation.z);
  if (!std::isfinite(q.squaredNorm()) || q.squaredNorm() < 1e-12) {
    return AUVReadStatus::invalid_imu;
  }
  frame.orientation = q.normalized();
  return AUVReadStatus::ok;
}

AUVReadStatus AUVClient::read(AUVFrame & frame)
{
  frame.image.release();
  frame.orientation = Eigen::Quaterniond::Identity();
  frame.timestamp = std::chrono::steady_clock::time_point{};
  frame.source_stamp.sec = 0;
  frame.source_stamp.nanosec = 0;
  ImageMsg::ConstSharedPtr image_msg;
  ImuMsg::ConstSharedPtr imu_msg;

  {
    std::unique_lock<std::mutex> lock(data_mutex_);
    const auto input_timeout = std::chrono::duration<double, std::milli>(command_timeout_ms_);
    const auto has_image = data_condition_.wait_for(lock, input_timeout, [this]() {
      return stopping_ || (latest_image_ && latest_image_sequence_ != consumed_image_sequence_);
    });
    if (!has_image || stopping_) return AUVReadStatus::timeout;

    image_msg = latest_image_;
    consumed_image_sequence_ = latest_image_sequence_;
    frame.source_stamp = image_msg->header.stamp;

    const auto image_stamp_ns = stamp_to_ns(image_msg->header.stamp);
    const auto sync_wait = std::chrono::duration<double, std::milli>(sync_wait_ms_);
    data_condition_.wait_for(lock, sync_wait, [this, image_stamp_ns]() {
      return stopping_ || has_matching_imu_locked(image_stamp_ns);
    });
    if (stopping_) return AUVReadStatus::timeout;

    imu_msg = nearest_imu_locked(image_stamp_ns);
    if (!imu_msg) return AUVReadStatus::unmatched_imu;

    const auto oldest_useful_stamp =
      image_stamp_ns - static_cast<std::int64_t>((sync_tolerance_ms_ + 1000.0) * 1e6);
    while (!imu_buffer_.empty() &&
           stamp_to_ns(imu_buffer_.front()->header.stamp) < oldest_useful_stamp) {
      imu_buffer_.pop_front();
    }
  }

  const auto image_status = convert_image(image_msg, frame);
  if (image_status != AUVReadStatus::ok) return image_status;
  const auto imu_status = convert_imu(imu_msg, frame);
  if (imu_status != AUVReadStatus::ok) return imu_status;

  const auto source_time = rclcpp::Time(frame.source_stamp);
  const auto ros_now = node_->now();
  auto ros_age_ms = static_cast<double>(ros_now.nanoseconds() - source_time.nanoseconds()) / 1e6;
  if (ros_age_ms < -sync_tolerance_ms_) {
    RCLCPP_WARN_THROTTLE(
      auto_aim_logger(), *node_->get_clock(), 1000, "Image timestamp is %.3f ms in the future",
      -ros_age_ms);
    return AUVReadStatus::stale;
  }
  ros_age_ms = std::max(0.0, ros_age_ms);
  const auto effective_age_ms = ros_age_ms + upstream_latency_ms_;
  if (effective_age_ms > max_frame_age_ms_) {
    RCLCPP_WARN_THROTTLE(
      auto_aim_logger(), *node_->get_clock(), 1000, "Dropping stale image: effective age %.3f ms",
      effective_age_ms);
    return AUVReadStatus::stale;
  }

  frame.timestamp = std::chrono::steady_clock::now() -
                    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                      std::chrono::duration<double, std::milli>(effective_age_ms));
  return AUVReadStatus::ok;
}

void AUVClient::publish_json(const Command & command, const builtin_interfaces::msg::Time * stamp)
{
  double latency_ms = -1.0;
  builtin_interfaces::msg::Time output_stamp;
  if (stamp) {
    output_stamp = *stamp;
    latency_ms =
      static_cast<double>(node_->now().nanoseconds() - rclcpp::Time(*stamp).nanoseconds()) / 1e6;
    latency_ms = std::max(0.0, latency_ms);
  }

  nlohmann::json json;
  json["stamp"] = {{"sec", output_stamp.sec}, {"nanosec", output_stamp.nanosec}};
  json["control"] = command.control;
  json["shoot"] = command.shoot;
  json["yaw_rad"] = command.yaw;
  json["pitch_rad"] = command.pitch;
  json["horizon_distance_m"] = command.horizon_distance;
  json["latency_ms"] = latency_ms;

  std_msgs::msg::String message;
  message.data = json.dump();
  result_publisher_->publish(message);
}

void AUVClient::publish(
  const Command & command, const AUVFrame * frame, std::optional<std::uint64_t> team_color_revision)
{
  std::lock_guard<std::mutex> lock(result_mutex_);
  bool team_color_is_current = true;
  if (team_color_revision) {
    std::lock_guard<std::mutex> team_lock(team_color_mutex_);
    team_color_is_current =
      self_is_red_.has_value() && *team_color_revision == team_color_revision_;
  }
  const auto command_is_finite = std::isfinite(command.yaw) && std::isfinite(command.pitch) &&
                                 std::isfinite(command.horizon_distance);
  if (command_is_finite && team_color_is_current) {
    publish_json(command, frame ? &frame->source_stamp : nullptr);
  } else {
    if (!command_is_finite) {
      RCLCPP_WARN_THROTTLE(
        auto_aim_logger(), *node_->get_clock(), 1000,
        "Non-finite auto-aim command rejected; publishing a safe result");
    }
    publish_json(safe_command(), frame ? &frame->source_stamp : nullptr);
  }
  last_result_steady_ns_ = steady_now_ns();
  watchdog_result_sent_ = false;
}

void AUVClient::publish_debug(
  cv::Mat image, const builtin_interfaces::msg::Time & source_stamp)
{
  if (image.empty()) return;

  {
    std::lock_guard<std::mutex> lock(debug_mutex_);
    if (debug_stopping_) return;
    pending_debug_frame_ = DebugFrame{std::move(image), source_stamp};
  }
  debug_condition_.notify_one();
}

void AUVClient::debug_worker()
{
  while (true) {
    DebugFrame frame;
    {
      std::unique_lock<std::mutex> lock(debug_mutex_);
      debug_condition_.wait(
        lock, [this]() { return debug_stopping_ || pending_debug_frame_.has_value(); });
      if (debug_stopping_) return;
      frame = std::move(*pending_debug_frame_);
      pending_debug_frame_.reset();
    }

    ImageMsg message;
    message.header.stamp = frame.source_stamp;
    message.format = "bgr8; jpeg compressed bgr8";
    try {
      static const std::vector<int> parameters{cv::IMWRITE_JPEG_QUALITY, 80};
      if (cv::imencode(".jpg", frame.image, message.data, parameters)) {
        debug_publisher_->publish(std::move(message));
      }
    } catch (const cv::Exception & e) {
      RCLCPP_WARN_THROTTLE(
        auto_aim_logger(), *node_->get_clock(), 1000, "Failed to encode debug image: %s",
        e.what());
    }
  }
}

void AUVClient::watchdog_callback()
{
  std::lock_guard<std::mutex> lock(result_mutex_);
  const auto elapsed_ms = static_cast<double>(steady_now_ns() - last_result_steady_ns_) / 1e6;
  if (elapsed_ms < command_timeout_ms_ || watchdog_result_sent_) return;
  watchdog_result_sent_ = true;
  publish_json(safe_command(), nullptr);
}

double AUVClient::bullet_speed() const { return bullet_speed_; }

std::optional<AUVTeamColor> AUVClient::team_color() const
{
  std::lock_guard<std::mutex> lock(team_color_mutex_);
  if (!self_is_red_) return std::nullopt;
  return AUVTeamColor{*self_is_red_, team_color_revision_};
}

}  // namespace io
