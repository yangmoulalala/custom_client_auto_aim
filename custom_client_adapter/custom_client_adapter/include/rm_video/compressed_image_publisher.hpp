#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/header.hpp>

namespace rm_video {

struct BgrImage {
  int width = 0;
  int height = 0;
  std::vector<std::uint8_t> data;
};

struct CompressionFrame {
  std_msgs::msg::Header header;
  BgrImage raw;
  BgrImage processed;
};

class CompressedImagePublisher {
public:
  CompressedImagePublisher(rclcpp::Node &node, const std::string &raw_topic,
                           const std::string &processed_topic,
                           const rclcpp::QoS &qos, int jpeg_quality);
  ~CompressedImagePublisher();

  CompressedImagePublisher(const CompressedImagePublisher &) = delete;
  CompressedImagePublisher &
  operator=(const CompressedImagePublisher &) = delete;

  void start();
  void stop();
  void submit(CompressionFrame frame);

private:
  void worker_loop();
  void publish_image(
      const BgrImage &image, const std_msgs::msg::Header &header,
      const rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr
          &publisher);

  int jpeg_quality_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr
      raw_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr
      processed_publisher_;

  std::atomic<bool> running_{false};
  std::mutex mutex_;
  std::condition_variable condition_;
  std::optional<CompressionFrame> pending_frame_;
  std::thread worker_;
};

} // namespace rm_video
