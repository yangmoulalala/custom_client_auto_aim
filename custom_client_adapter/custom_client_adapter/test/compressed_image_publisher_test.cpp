#include "rm_video/compressed_image_publisher.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

using namespace std::chrono_literals;

namespace {

template <typename Predicate>
bool wait_for(Predicate predicate) {
  const auto deadline = std::chrono::steady_clock::now() + 2s;
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(10ms);
  }
  return false;
}

rm_video::CompressionFrame make_frame(std::uint8_t seed) {
  rm_video::CompressionFrame frame;
  frame.header.frame_id = "test_camera";
  frame.raw = rm_video::BgrImage{2, 2, std::vector<std::uint8_t>(12, seed)};
  frame.processed =
      rm_video::BgrImage{2, 2, std::vector<std::uint8_t>(12, seed + 1)};
  return frame;
}

} // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  int failures = 0;
  const auto check = [&failures](bool condition, const std::string &message) {
    if (condition) {
      return;
    }
    std::cerr << "FAIL: " << message << std::endl;
    ++failures;
  };

  auto publisher_node =
      std::make_shared<rclcpp::Node>("compressed_image_publisher_test_source");
  auto subscriber_node =
      std::make_shared<rclcpp::Node>("compressed_image_publisher_test_sink");
  auto qos = rclcpp::SensorDataQoS().keep_last(1).best_effort();
  rm_video::CompressedImagePublisher publisher(
      *publisher_node, "/test/image_raw", "/test/image_processed", qos, 80);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(publisher_node);
  executor.add_node(subscriber_node);
  std::thread spin_thread([&executor]() { executor.spin(); });
  publisher.start();

  auto demand = publisher.output_demand();
  check(!demand.raw && !demand.processed,
        "image outputs should be idle without subscribers");

  std::atomic<int> processed_messages{0};
  auto processed_subscription =
      subscriber_node->create_subscription<sensor_msgs::msg::CompressedImage>(
          "/test/image_processed", qos,
          [&processed_messages](
              sensor_msgs::msg::CompressedImage::ConstSharedPtr message) {
            if (!message->data.empty()) {
              processed_messages.fetch_add(1);
            }
          });
  check(wait_for([&publisher]() {
          const auto current = publisher.output_demand();
          return !current.raw && current.processed;
        }),
        "processed output demand should follow graph discovery");

  publisher.submit(make_frame(10));
  check(wait_for([&processed_messages]() {
          return processed_messages.load() >= 1;
        }),
        "processed JPEG should be published to its subscriber");

  std::atomic<int> raw_messages{0};
  auto raw_subscription =
      subscriber_node->create_subscription<sensor_msgs::msg::CompressedImage>(
          "/test/image_raw", qos,
          [&raw_messages](
              sensor_msgs::msg::CompressedImage::ConstSharedPtr message) {
            if (!message->data.empty()) {
              raw_messages.fetch_add(1);
            }
          });
  check(wait_for([&publisher]() {
          const auto current = publisher.output_demand();
          return current.raw && current.processed;
        }),
        "raw output demand should follow graph discovery");

  publisher.submit(make_frame(20));
  check(wait_for([&raw_messages, &processed_messages]() {
          return raw_messages.load() >= 1 && processed_messages.load() >= 2;
        }),
        "each requested JPEG output should be published");

  (void)raw_subscription;
  (void)processed_subscription;
  publisher.stop();
  executor.cancel();
  spin_thread.join();
  executor.remove_node(subscriber_node);
  executor.remove_node(publisher_node);
  rclcpp::shutdown();

  if (failures == 0) {
    std::cout << "Compressed image publisher tests passed" << std::endl;
  }
  return failures == 0 ? 0 : 1;
}
