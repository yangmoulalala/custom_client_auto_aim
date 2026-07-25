#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <iostream>
#include <mutex>
#include <nlohmann/json.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <thread>

#include "io/auv_client.hpp"

using namespace std::chrono_literals;

namespace
{
bool wait_for_graph(
  const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr & image_publisher,
  const rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr & imu_publisher,
  const rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr & self_is_red_publisher,
  const rclcpp::Subscription<std_msgs::msg::String>::SharedPtr & result_subscription)
{
  const auto deadline = std::chrono::steady_clock::now() + 3s;
  while (std::chrono::steady_clock::now() < deadline) {
    if (
      image_publisher->get_subscription_count() > 0 &&
      imu_publisher->get_subscription_count() > 0 &&
      self_is_red_publisher->get_subscription_count() > 0 &&
      result_subscription->get_publisher_count() > 0) {
      return true;
    }
    std::this_thread::sleep_for(10ms);
  }
  return false;
}

std::optional<io::AUVTeamColor> wait_for_team_color(
  const io::AUVClient & client, bool self_is_red, std::uint64_t minimum_revision = 0)
{
  const auto deadline = std::chrono::steady_clock::now() + 1s;
  while (std::chrono::steady_clock::now() < deadline) {
    const auto color = client.team_color();
    if (
      color && color->self_is_red == self_is_red && color->revision >= minimum_revision) {
      return color;
    }
    std::this_thread::sleep_for(5ms);
  }
  return std::nullopt;
}

sensor_msgs::msg::Image make_image(
  const builtin_interfaces::msg::Time & stamp,
  const std::string & encoding = sensor_msgs::image_encodings::BGR8, std::uint8_t seed = 0)
{
  sensor_msgs::msg::Image image;
  image.header.stamp = stamp;
  image.height = 2;
  image.width = 4;
  image.encoding = encoding;
  image.is_bigendian = false;
  const auto channels = encoding == sensor_msgs::image_encodings::MONO8 ? 1U : 3U;
  image.step = image.width * channels;
  image.data.resize(image.step * image.height);
  for (std::size_t i = 0; i < image.data.size(); ++i) {
    image.data[i] = static_cast<std::uint8_t>(seed + i);
  }
  return image;
}

sensor_msgs::msg::Imu make_imu(
  const builtin_interfaces::msg::Time & stamp, double w = 2.0, double x = 0.0, double y = 0.0,
  double z = 0.0)
{
  sensor_msgs::msg::Imu imu;
  imu.header.stamp = stamp;
  imu.orientation.w = w;
  imu.orientation.x = x;
  imu.orientation.y = y;
  imu.orientation.z = z;
  return imu;
}

builtin_interfaces::msg::Time to_msg(const rclcpp::Time & time)
{
  return static_cast<builtin_interfaces::msg::Time>(time);
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int failures = 0;
  const auto check = [&failures](bool condition, const std::string & message) {
    if (condition) return;
    std::cerr << "FAIL: " << message << std::endl;
    ++failures;
  };

  {
    io::AUVClient client(AUV_CLIENT_TEST_CONFIG);
    auto helper_node = std::make_shared<rclcpp::Node>("auv_client_io_test");
    auto sensor_qos = rclcpp::SensorDataQoS().keep_last(1).best_effort();
    auto image_publisher =
      helper_node->create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", sensor_qos);
    auto imu_publisher =
      helper_node->create_publisher<sensor_msgs::msg::Imu>("/imu/data", sensor_qos);
    auto self_is_red_publisher = helper_node->create_publisher<std_msgs::msg::Bool>(
      "/auv_client_test/self_is_red", rclcpp::QoS(1).reliable());

    std::mutex result_mutex;
    std::condition_variable result_condition;
    std::string result_json;
    auto result_subscription = helper_node->create_subscription<std_msgs::msg::String>(
      "/auto_aim/result", rclcpp::QoS(10).best_effort(),
      [&](std_msgs::msg::String::ConstSharedPtr msg) {
        {
          std::lock_guard<std::mutex> lock(result_mutex);
          result_json = msg->data;
        }
        result_condition.notify_all();
      });

    rclcpp::executors::SingleThreadedExecutor helper_executor;
    helper_executor.add_node(helper_node);
    std::thread helper_spin([&helper_executor]() { helper_executor.spin(); });

    check(
      wait_for_graph(
        image_publisher, imu_publisher, self_is_red_publisher, result_subscription),
      "ROS publishers and subscriptions were not discovered");

    check(!client.team_color().has_value(), "team color should be unknown before the first message");
    std_msgs::msg::Bool self_is_red_message;
    self_is_red_message.data = true;
    self_is_red_publisher->publish(self_is_red_message);
    const auto red_team = wait_for_team_color(client, true, 1);
    check(red_team.has_value(), "self_is_red=true should select the red team");

    self_is_red_message.data = false;
    self_is_red_publisher->publish(self_is_red_message);
    const auto blue_team =
      wait_for_team_color(client, false, red_team ? red_team->revision + 1 : 2);
    check(blue_team.has_value(), "self_is_red=false should select the blue team");
    if (red_team && blue_team) {
      check(
        blue_team->revision > red_team->revision,
        "changing team color should advance its revision");
    }

    auto exact_stamp = to_msg(helper_node->now());
    imu_publisher->publish(make_imu(exact_stamp));
    image_publisher->publish(make_image(exact_stamp));

    io::AUVFrame frame;
    auto status = client.read(frame);
    check(status == io::AUVReadStatus::ok, "exact timestamp pair should be accepted");
    check(frame.image.cols == 4 && frame.image.rows == 2, "BGR image dimensions should be kept");
    check(frame.image.type() == CV_8UC3, "BGR image type should be CV_8UC3");
    check(
      frame.image_owner && frame.image.data == frame.image_owner->data.data(),
      "BGR image should reference the received ROS message without another full-frame copy");
    check(std::abs(frame.orientation.w() - 1.0) < 1e-12, "IMU quaternion should be normalized");
    const auto compensated_age_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - frame.timestamp)
        .count();
    check(
      compensated_age_ms >= 18.0,
      "configured upstream latency should move the effective frame timestamp backwards");

    std::this_thread::sleep_for(10ms);
    const auto approximate_image_time = helper_node->now();
    const auto approximate_imu_time =
      approximate_image_time + rclcpp::Duration::from_nanoseconds(3000000);
    imu_publisher->publish(make_imu(to_msg(approximate_imu_time)));
    image_publisher->publish(make_image(to_msg(approximate_image_time)));
    status = client.read(frame);
    check(status == io::AUVReadStatus::ok, "3 ms timestamp difference should be accepted");

    std::this_thread::sleep_for(10ms);
    const auto unmatched_image_time = helper_node->now();
    const auto unmatched_imu_time =
      unmatched_image_time + rclcpp::Duration::from_nanoseconds(20000000);
    imu_publisher->publish(make_imu(to_msg(unmatched_imu_time)));
    image_publisher->publish(make_image(to_msg(unmatched_image_time)));
    status = client.read(frame);
    check(
      status == io::AUVReadStatus::unmatched_imu, "20 ms timestamp difference should be rejected");

    const auto stale_stamp =
      to_msg(helper_node->now() - rclcpp::Duration::from_nanoseconds(150000000));
    imu_publisher->publish(make_imu(stale_stamp));
    image_publisher->publish(make_image(stale_stamp));
    status = client.read(frame);
    check(status == io::AUVReadStatus::stale, "150 ms old image should be rejected");

    std::this_thread::sleep_for(10ms);
    const auto rgb_stamp = to_msg(helper_node->now());
    imu_publisher->publish(make_imu(rgb_stamp));
    image_publisher->publish(make_image(rgb_stamp, sensor_msgs::image_encodings::RGB8, 10));
    status = client.read(frame);
    check(status == io::AUVReadStatus::ok, "RGB8 image should be accepted");
    if (status == io::AUVReadStatus::ok) {
      const auto pixel = frame.image.at<cv::Vec3b>(0, 0);
      check(pixel == cv::Vec3b(12, 11, 10), "RGB8 image should be converted to BGR");
    }

    std::this_thread::sleep_for(10ms);
    const auto mono_stamp = to_msg(helper_node->now());
    imu_publisher->publish(make_imu(mono_stamp));
    image_publisher->publish(make_image(mono_stamp, sensor_msgs::image_encodings::MONO8, 20));
    status = client.read(frame);
    check(status == io::AUVReadStatus::ok, "MONO8 image should be accepted");
    if (status == io::AUVReadStatus::ok) {
      check(
        frame.image.at<cv::Vec3b>(0, 0) == cv::Vec3b(20, 20, 20),
        "MONO8 image should be expanded to BGR");
    }

    std::this_thread::sleep_for(10ms);
    const auto older_stamp = to_msg(helper_node->now());
    const auto newer_stamp =
      to_msg(rclcpp::Time(older_stamp) + rclcpp::Duration::from_nanoseconds(1000000));
    imu_publisher->publish(make_imu(older_stamp));
    image_publisher->publish(make_image(older_stamp, sensor_msgs::image_encodings::BGR8, 30));
    imu_publisher->publish(make_imu(newer_stamp));
    image_publisher->publish(make_image(newer_stamp, sensor_msgs::image_encodings::BGR8, 40));
    std::this_thread::sleep_for(10ms);
    status = client.read(frame);
    check(status == io::AUVReadStatus::ok, "latest queued image should be readable");
    if (status == io::AUVReadStatus::ok) {
      check(
        frame.source_stamp.sec == newer_stamp.sec &&
          frame.source_stamp.nanosec == newer_stamp.nanosec,
        "newer image should overwrite the older unconsumed image");
      check(frame.image.at<cv::Vec3b>(0, 0)[0] == 40, "latest image payload should be returned");
    }

    std::this_thread::sleep_for(10ms);
    const auto invalid_imu_stamp = to_msg(helper_node->now());
    imu_publisher->publish(make_imu(invalid_imu_stamp, 0.0, 0.0, 0.0, 0.0));
    image_publisher->publish(make_image(invalid_imu_stamp));
    status = client.read(frame);
    check(status == io::AUVReadStatus::invalid_imu, "zero quaternion should be rejected");

    {
      std::lock_guard<std::mutex> lock(result_mutex);
      result_json.clear();
    }
    const io::Command expected_command{true, true, 0.12, -0.34, 5.6};
    if (red_team && blue_team) {
      client.publish(expected_command, &frame, red_team->revision);
      {
        std::unique_lock<std::mutex> lock(result_mutex);
        result_condition.wait_for(lock, 1s, [&result_json]() { return !result_json.empty(); });
      }
      check(!result_json.empty(), "a stale team-color result should still publish a safe result");
      if (!result_json.empty()) {
        const auto stale_team_result = nlohmann::json::parse(result_json);
        check(
          !stale_team_result.at("control").get<bool>(),
          "a result calculated for an old team color must not control the gimbal");
        check(
          !stale_team_result.at("shoot").get<bool>(),
          "a result calculated for an old team color must not permit firing");
      }
      std::lock_guard<std::mutex> lock(result_mutex);
      result_json.clear();
    }

    client.publish(
      expected_command, &frame,
      blue_team ? std::optional<std::uint64_t>(blue_team->revision) : std::nullopt);
    {
      std::unique_lock<std::mutex> lock(result_mutex);
      result_condition.wait_for(lock, 1s, [&result_json]() { return !result_json.empty(); });
    }
    check(!result_json.empty(), "result JSON should be published");
    if (!result_json.empty()) {
      const auto result = nlohmann::json::parse(result_json);
      check(result.at("control").get<bool>(), "control field should be true");
      check(result.at("shoot").get<bool>(), "shoot field should be true");
      check(std::abs(result.at("yaw_rad").get<double>() - 0.12) < 1e-12, "yaw should match");
      check(std::abs(result.at("pitch_rad").get<double>() + 0.34) < 1e-12, "pitch should match");
      check(
        std::abs(result.at("horizon_distance_m").get<double>() - 5.6) < 1e-12,
        "horizon distance should match");
      check(result.find("status") == result.end(), "result JSON must not contain status");
    }

    {
      std::lock_guard<std::mutex> lock(result_mutex);
      result_json.clear();
    }
    {
      std::unique_lock<std::mutex> lock(result_mutex);
      result_condition.wait_for(lock, 1s, [&result_json]() { return !result_json.empty(); });
    }
    check(!result_json.empty(), "watchdog should publish after the command timeout");
    if (!result_json.empty()) {
      const auto watchdog_result = nlohmann::json::parse(result_json);
      check(!watchdog_result.at("control").get<bool>(), "watchdog control should be false");
      check(!watchdog_result.at("shoot").get<bool>(), "watchdog shoot should be false");
      check(
        watchdog_result.at("stamp").at("sec").get<int>() == 0 &&
          watchdog_result.at("stamp").at("nanosec").get<unsigned int>() == 0,
        "watchdog result should use a zero source timestamp");
    }

    helper_executor.cancel();
    helper_spin.join();
    helper_executor.remove_node(helper_node);
  }

  if (rclcpp::ok()) rclcpp::shutdown();
  if (failures == 0) std::cout << "AUV client I/O tests passed" << std::endl;
  return failures == 0 ? 0 : 1;
}
