#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <string>
#include <utility>

#include "rm_video/compressed_image_publisher.hpp"
#include "rm_video/video_decoder.hpp"

namespace rm_video
{
namespace
{

struct ImageConfig
{
  std::string raw_topic;
  std::string processed_topic;
  std::string frame_id;
  int qos_depth;
  int jpeg_quality;
  double roi_center_x;
  double roi_center_y;
  double roi_crop_ratio;
  int rotation_quarter_turns;
  double brightness_gain;
  double timestamp_offset_sec;
};

void require(bool condition, const std::string & message)
{
  if (!condition) {
    throw std::invalid_argument(message);
  }
}

}  // namespace

class RmVideoNode : public rclcpp::Node
{
public:
  RmVideoNode() : Node("rm_video"), last_statistics_time_(std::chrono::steady_clock::now())
  {
    const DecoderConfig decoder_config = load_decoder_config();
    image_config_ = load_image_config();
    validate_image_config(image_config_);
    initialize_brightness_lookup();

    auto qos = rclcpp::QoS(rclcpp::KeepLast(image_config_.qos_depth));
    qos.best_effort().durability_volatile();
    compressed_publisher_ = std::make_unique<CompressedImagePublisher>(
      *this, image_config_.raw_topic, image_config_.processed_topic, qos,
      image_config_.jpeg_quality);
    compressed_publisher_->start();

    decoder_ = std::make_unique<VideoDecoder>(
      decoder_config, [this](DecodedFrame && frame) { publish_frame(std::move(frame)); },
      get_logger());
    if (!decoder_->start()) {
      throw std::runtime_error("Failed to start video decoder");
    }

    statistics_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() { log_statistics(); });
    RCLCPP_INFO(
      get_logger(),
      "Publishing JPEG images on %s and %s with quality %d and "
      "best-effort QoS depth %d",
      image_config_.raw_topic.c_str(), image_config_.processed_topic.c_str(),
      image_config_.jpeg_quality, image_config_.qos_depth);
  }

  ~RmVideoNode() override
  {
    if (decoder_) {
      decoder_->stop();
    }
    if (compressed_publisher_) {
      compressed_publisher_->stop();
    }
  }

private:
  DecoderConfig load_decoder_config()
  {
    const std::string bind_address = declare_parameter<std::string>("udp.bind_address", "0.0.0.0");
    const auto port = declare_parameter<std::int64_t>("udp.port", 3334);
    const auto receive_buffer =
      declare_parameter<std::int64_t>("udp.receive_buffer_bytes", 4 * 1024 * 1024);
    const auto receive_timeout = declare_parameter<std::int64_t>("udp.receive_timeout_ms", 100);
    const std::string codec_name = declare_parameter<std::string>("decoder.codec", "hevc");
    const auto codec_id = parse_codec(codec_name);
    require(codec_id.has_value(), "decoder.codec must be hevc, h264, or mjpeg");
    const auto decoder_threads = declare_parameter<std::int64_t>("decoder.threads", 0);
    const auto decode_queue_size = declare_parameter<std::int64_t>("decoder.decode_queue_size", 4);

    const auto max_frame_bytes =
      declare_parameter<std::int64_t>("decoder.max_frame_bytes", 16 * 1024 * 1024);
    const auto reassembly_slots = declare_parameter<std::int64_t>("decoder.reassembly_slots", 8);
    const auto reassembly_timeout =
      declare_parameter<std::int64_t>("decoder.reassembly_timeout_ms", 200);

    require(port > 0 && port <= 65535, "udp.port must be between 1 and 65535");
    require(
      receive_buffer >= 65536 && receive_buffer <= std::numeric_limits<int>::max(),
      "udp.receive_buffer_bytes must be between 65536 and INT_MAX");
    require(
      receive_timeout > 0 && receive_timeout <= 1000,
      "udp.receive_timeout_ms must be between 1 and 1000");
    require(
      decoder_threads >= 0 && decoder_threads <= 32, "decoder.threads must be between 0 and 32");
    require(
      decode_queue_size > 0 && decode_queue_size <= 64,
      "decoder.decode_queue_size must be between 1 and 64");
    require(
      max_frame_bytes >= 1024 && max_frame_bytes <= 256 * 1024 * 1024,
      "decoder.max_frame_bytes must be between 1024 and 268435456");
    require(
      reassembly_slots > 0 && reassembly_slots <= 64,
      "decoder.reassembly_slots must be between 1 and 64");
    require(
      reassembly_timeout > 0 && reassembly_timeout <= 5000,
      "decoder.reassembly_timeout_ms must be between 1 and 5000");

    return DecoderConfig{
      bind_address,
      static_cast<int>(port),
      static_cast<int>(receive_buffer),
      static_cast<int>(receive_timeout),
      *codec_id,
      static_cast<int>(decoder_threads),
      static_cast<std::size_t>(decode_queue_size),
      static_cast<std::size_t>(max_frame_bytes),
      static_cast<std::size_t>(reassembly_slots),
      std::chrono::milliseconds(reassembly_timeout)};
  }

  ImageConfig load_image_config()
  {
    const auto qos_depth = declare_parameter<std::int64_t>("publisher.qos_depth", 1);
    const auto jpeg_quality = declare_parameter<std::int64_t>("compression.jpeg_quality", 80);
    const auto rotation_quarter_turns =
      declare_parameter<std::int64_t>("processed.rotation_quarter_turns", 0);
    require(qos_depth > 0 && qos_depth <= 100, "publisher.qos_depth must be between 1 and 100");
    require(
      jpeg_quality >= 1 && jpeg_quality <= 100,
      "compression.jpeg_quality must be between 1 and 100");
    require(
      rotation_quarter_turns >= 0 && rotation_quarter_turns <= 3,
      "processed.rotation_quarter_turns must be between 0 and 3");
    return ImageConfig{
      declare_parameter<std::string>("publisher.raw_topic", "/rm_video/image_raw"),
      declare_parameter<std::string>("publisher.processed_topic", "/rm_video/image_processed"),
      declare_parameter<std::string>("publisher.frame_id", "rm_video_camera"),
      static_cast<int>(qos_depth),
      static_cast<int>(jpeg_quality),
      declare_parameter<double>("roi.center_x", 0.5),
      declare_parameter<double>("roi.center_y", 0.5),
      declare_parameter<double>("roi.crop_ratio", 1.0),
      static_cast<int>(rotation_quarter_turns),
      declare_parameter<double>("brightness_gain", 1.0),
      declare_parameter<double>("timestamp_offset_sec", 0.0)};
  }

  static void validate_image_config(const ImageConfig & image_config)
  {
    const std::array<std::string, 2> topics{image_config.raw_topic, image_config.processed_topic};
    for (std::size_t index = 0; index < topics.size(); ++index) {
      require(!topics[index].empty(), "publisher topics must not be empty");
      for (std::size_t other = index + 1; other < topics.size(); ++other) {
        require(topics[index] != topics[other], "publisher topics must be different");
      }
    }
    require(
      image_config.roi_center_x >= 0.0 && image_config.roi_center_x <= 1.0,
      "roi.center_x must be between 0 and 1");
    require(
      image_config.roi_center_y >= 0.0 && image_config.roi_center_y <= 1.0,
      "roi.center_y must be between 0 and 1");
    require(
      image_config.roi_crop_ratio > 0.0 && image_config.roi_crop_ratio <= 1.0,
      "roi.crop_ratio must be greater than 0 and at most 1");
    require(
      std::isfinite(image_config.brightness_gain) && image_config.brightness_gain >= 0.0,
      "brightness_gain must be finite and non-negative");
    require(
      std::isfinite(image_config.timestamp_offset_sec) &&
        std::abs(image_config.timestamp_offset_sec) <= 86400.0,
      "timestamp_offset_sec must be finite and within one day");
  }

  BgrImage make_processed_image(const DecodedFrame & frame) const
  {
    // 宽高使用同一比例裁剪，中心靠近边界时整体平移 ROI。
    const int crop_width = std::clamp(
      static_cast<int>(std::lround(frame.width * image_config_.roi_crop_ratio)), 1, frame.width);
    const int crop_height = std::clamp(
      static_cast<int>(std::lround(frame.height * image_config_.roi_crop_ratio)), 1, frame.height);
    const int center_x = static_cast<int>(std::lround(frame.width * image_config_.roi_center_x));
    const int center_y = static_cast<int>(std::lround(frame.height * image_config_.roi_center_y));
    const int start_x = std::clamp(center_x - crop_width / 2, 0, frame.width - crop_width);
    const int start_y = std::clamp(center_y - crop_height / 2, 0, frame.height - crop_height);

    const bool swap_dimensions = image_config_.rotation_quarter_turns % 2 != 0;
    const int output_width = swap_dimensions ? crop_height : crop_width;
    const int output_height = swap_dimensions ? crop_width : crop_height;
    const std::size_t output_step = static_cast<std::size_t>(output_width) * 3;

    BgrImage processed;
    processed.width = output_width;
    processed.height = output_height;
    processed.data.resize(output_step * output_height);

    const std::size_t source_stride = static_cast<std::size_t>(frame.width) * 3;
    const std::size_t source_row_bytes = static_cast<std::size_t>(crop_width) * 3;
    const std::ptrdiff_t output_stride = static_cast<std::ptrdiff_t>(output_step);
    for (int row = 0; row < crop_height; ++row) {
      const std::uint8_t * source = frame.bgr.data() +
                                    static_cast<std::size_t>(start_y + row) * source_stride +
                                    static_cast<std::size_t>(start_x) * 3;
      if (image_config_.rotation_quarter_turns == 0 && image_config_.brightness_gain == 1.0) {
        std::memcpy(
          processed.data.data() + static_cast<std::size_t>(row) * source_row_bytes, source,
          source_row_bytes);
        continue;
      }

      // 每行只计算一次旋转后的起点和步长，逐像素处理时不再判断方向。
      std::uint8_t * destination = nullptr;
      std::ptrdiff_t destination_step = 0;
      switch (image_config_.rotation_quarter_turns) {
        case 0:
          destination = processed.data.data() + static_cast<std::size_t>(row) * source_row_bytes;
          destination_step = 3;
          break;
        case 1:
          destination = processed.data.data() +
                        static_cast<std::size_t>(crop_width - 1) * output_step +
                        static_cast<std::size_t>(row) * 3;
          destination_step = -output_stride;
          break;
        case 2:
          destination = processed.data.data() +
                        static_cast<std::size_t>(crop_height - 1 - row) * output_step +
                        static_cast<std::size_t>(crop_width - 1) * 3;
          destination_step = -3;
          break;
        case 3:
          destination = processed.data.data() + static_cast<std::size_t>(crop_height - 1 - row) * 3;
          destination_step = output_stride;
          break;
      }
      for (int column = 0; column < crop_width; ++column) {
        destination[0] = brightness_lookup_[source[0]];
        destination[1] = brightness_lookup_[source[1]];
        destination[2] = brightness_lookup_[source[2]];
        source += 3;
        if (column + 1 < crop_width) {
          destination += destination_step;
        }
      }
    }

    return processed;
  }

  void publish_frame(DecodedFrame frame)
  {
    const auto demand = compressed_publisher_->output_demand();
    if (!demand.raw && !demand.processed) {
      return;
    }

    CompressionFrame output;
    output.header.stamp =
      now() + rclcpp::Duration::from_seconds(image_config_.timestamp_offset_sec);
    output.header.frame_id = image_config_.frame_id;
    if (demand.processed) {
      output.processed = make_processed_image(frame);
    }
    if (demand.raw) {
      output.raw = BgrImage{frame.width, frame.height, std::move(frame.bgr)};
    }
    compressed_publisher_->submit(std::move(output));
  }

  void log_statistics()
  {
    const auto now_steady = std::chrono::steady_clock::now();
    const double elapsed =
      std::chrono::duration<double>(now_steady - last_statistics_time_).count();
    last_statistics_time_ = now_steady;
    const Statistics statistics = decoder_->take_statistics();
    const auto fps =
      static_cast<unsigned long long>(std::llround(statistics.decoded_frames / elapsed));
    const auto udp_rate =
      static_cast<unsigned long long>(std::llround(statistics.udp_packets / elapsed));
    RCLCPP_INFO(
      get_logger(), "fps=%llu udp=%llupackets/s dropped=%llupackets error=%lluframes", fps,
      udp_rate, static_cast<unsigned long long>(statistics.dropped),
      static_cast<unsigned long long>(statistics.error));
  }

  void initialize_brightness_lookup()
  {
    for (std::size_t value = 0; value < brightness_lookup_.size(); ++value) {
      brightness_lookup_[value] = static_cast<std::uint8_t>(
        std::min(255.0, std::round(value * image_config_.brightness_gain)));
    }
  }

  ImageConfig image_config_;
  std::unique_ptr<CompressedImagePublisher> compressed_publisher_;
  std::unique_ptr<VideoDecoder> decoder_;
  rclcpp::TimerBase::SharedPtr statistics_timer_;
  std::chrono::steady_clock::time_point last_statistics_time_;
  std::array<std::uint8_t, 256> brightness_lookup_{};
};

}  // namespace rm_video

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<rm_video::RmVideoNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("rm_video"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
