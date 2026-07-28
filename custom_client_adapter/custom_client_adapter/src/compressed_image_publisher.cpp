#include "rm_video/compressed_image_publisher.hpp"

#include <jpeglib.h>

#include <csetjmp>
#include <cstdlib>
#include <utility>

namespace rm_video
{
namespace
{

bool has_subscribers(
  const rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr & publisher)
{
  return publisher->get_subscription_count() > 0 ||
         publisher->get_intra_process_subscription_count() > 0;
}

struct JpegErrorManager
{
  jpeg_error_mgr base;
  std::jmp_buf jump_buffer;
};

void handle_jpeg_error(j_common_ptr context)
{
  auto * error = reinterpret_cast<JpegErrorManager *>(context->err);
  std::longjmp(error->jump_buffer, 1);
}

bool encode_jpeg(const BgrImage & image, int quality, std::vector<std::uint8_t> & output)
{
  if (
    image.width <= 0 || image.height <= 0 ||
    image.data.size() != static_cast<std::size_t>(image.width) * image.height * 3) {
    return false;
  }

  jpeg_compress_struct compressor{};
  JpegErrorManager error{};
  compressor.err = jpeg_std_error(&error.base);
  error.base.error_exit = handle_jpeg_error;

  unsigned char * encoded_data = nullptr;
  unsigned long encoded_size = 0;
  if (setjmp(error.jump_buffer) != 0) {
    jpeg_destroy_compress(&compressor);
    std::free(encoded_data);
    return false;
  }

  jpeg_create_compress(&compressor);
  jpeg_mem_dest(&compressor, &encoded_data, &encoded_size);
  compressor.image_width = image.width;
  compressor.image_height = image.height;
  compressor.input_components = 3;
  compressor.in_color_space = JCS_EXT_BGR;
  jpeg_set_defaults(&compressor);
  jpeg_set_quality(&compressor, quality, TRUE);
  compressor.dct_method = JDCT_FASTEST;
  compressor.optimize_coding = FALSE;
  jpeg_start_compress(&compressor, TRUE);

  const std::size_t stride = static_cast<std::size_t>(image.width) * 3;
  while (compressor.next_scanline < compressor.image_height) {
    JSAMPROW row = const_cast<JSAMPROW>(image.data.data() + compressor.next_scanline * stride);
    jpeg_write_scanlines(&compressor, &row, 1);
  }

  jpeg_finish_compress(&compressor);
  output.assign(encoded_data, encoded_data + encoded_size);
  jpeg_destroy_compress(&compressor);
  std::free(encoded_data);
  return true;
}

}  // namespace

CompressedImagePublisher::CompressedImagePublisher(
  rclcpp::Node & node, const std::string & raw_topic, const std::string & processed_topic,
  const rclcpp::QoS & qos, int jpeg_quality)
: jpeg_quality_(jpeg_quality),
  raw_publisher_(node.create_publisher<sensor_msgs::msg::CompressedImage>(raw_topic, qos)),
  processed_publisher_(
    node.create_publisher<sensor_msgs::msg::CompressedImage>(processed_topic, qos))
{
}

CompressedImagePublisher::~CompressedImagePublisher() { stop(); }

void CompressedImagePublisher::start()
{
  if (running_.exchange(true)) {
    return;
  }
  worker_ = std::thread(&CompressedImagePublisher::worker_loop, this);
}

void CompressedImagePublisher::stop()
{
  running_.store(false);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_frame_.reset();
  }
  condition_.notify_all();
  if (worker_.joinable()) {
    worker_.join();
  }
}

CompressionDemand CompressedImagePublisher::output_demand() const
{
  return CompressionDemand{has_subscribers(raw_publisher_), has_subscribers(processed_publisher_)};
}

void CompressedImagePublisher::submit(CompressionFrame frame)
{
  const auto demand = output_demand();
  if (!running_.load() || (!demand.raw && !demand.processed)) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    // 压缩跟不上时覆盖旧帧，预览始终面向最新画面。
    pending_frame_ = std::move(frame);
  }
  condition_.notify_one();
}

void CompressedImagePublisher::worker_loop()
{
  while (true) {
    CompressionFrame frame;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      condition_.wait(lock, [this]() { return !running_.load() || pending_frame_.has_value(); });
      if (!running_.load()) {
        return;
      }
      frame = std::move(*pending_frame_);
      pending_frame_.reset();
    }
    const auto demand = output_demand();
    if (demand.raw) {
      publish_image(frame.raw, frame.header, raw_publisher_);
    }
    if (demand.processed) {
      publish_image(frame.processed, frame.header, processed_publisher_);
    }
  }
}

void CompressedImagePublisher::publish_image(
  const BgrImage & image, const std_msgs::msg::Header & header,
  const rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr & publisher)
{
  sensor_msgs::msg::CompressedImage message;
  message.header = header;
  message.format = "bgr8; jpeg compressed bgr8";
  if (encode_jpeg(image, jpeg_quality_, message.data)) {
    publisher->publish(std::move(message));
  }
}

}  // namespace rm_video
