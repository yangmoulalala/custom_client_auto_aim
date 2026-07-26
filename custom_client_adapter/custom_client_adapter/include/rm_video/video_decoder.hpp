#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/logger.hpp>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
}

namespace rm_video {

struct DecoderConfig {
  std::string bind_address;
  int port;
  int receive_buffer_bytes;
  int receive_timeout_ms;
  AVCodecID codec_id;
  int decoder_threads;
  std::size_t decode_queue_size;
  std::size_t max_frame_bytes;
  std::size_t reassembly_slots;
  std::chrono::milliseconds reassembly_timeout;
};

struct DecodedFrame {
  int width = 0;
  int height = 0;
  std::vector<std::uint8_t> bgr;
};

struct Statistics {
  std::uint64_t decoded_frames = 0;
  std::uint64_t udp_packets = 0;
  std::uint64_t dropped = 0;
  std::uint64_t error = 0;
};

class VideoDecoder {
public:
  using FrameCallback = std::function<void(DecodedFrame &&)>;

  VideoDecoder(DecoderConfig config, FrameCallback callback,
               rclcpp::Logger logger);
  ~VideoDecoder();

  VideoDecoder(const VideoDecoder &) = delete;
  VideoDecoder &operator=(const VideoDecoder &) = delete;

  bool start();
  void stop();
  Statistics take_statistics();

private:
  struct FrameAssembly {
    std::uint16_t frame_sequence = 0;
    std::uint32_t total_size = 0;
    std::size_t received_size = 0;
    bool active = false;
    std::chrono::steady_clock::time_point last_update;
    std::map<std::uint16_t, std::vector<std::uint8_t>> fragments;

    void reset();
  };

  struct PendingFrame {
    std::vector<std::uint8_t> data;
    std::size_t packet_count = 0;
    std::uint16_t frame_sequence = 0;
    bool discontinuity = false;
  };

  struct AvFrameDeleter {
    void operator()(AVFrame *frame) const { av_frame_free(&frame); }
  };
  using AvFramePtr = std::unique_ptr<AVFrame, AvFrameDeleter>;

  bool initialize_decoder();
  bool setup_socket();
  void receive_loop();
  void decode_loop();
  void process_datagram(const std::uint8_t *data, std::size_t size);
  FrameAssembly &get_or_create_assembly(std::uint16_t frame_sequence,
                                        std::uint32_t total_size);
  void expire_assemblies(std::chrono::steady_clock::time_point now);
  void submit_frame(PendingFrame frame);
  void decode_frame(PendingFrame frame);
  bool submit_output_frame(const AVFrame &frame);
  void conversion_loop();
  bool convert_frame(const AVFrame &source, DecodedFrame &output);
  void discard_assembly(FrameAssembly &assembly, const char *reason);
  void cleanup();

  DecoderConfig config_;
  FrameCallback callback_;
  rclcpp::Logger logger_;

  int socket_fd_ = -1;
  const AVCodec *codec_ = nullptr;
  AVCodecContext *codec_context_ = nullptr;
  AVPacket *packet_ = nullptr;
  AVFrame *frame_ = nullptr;
  SwsContext *sws_context_ = nullptr;

  std::vector<FrameAssembly> assemblies_;
  std::atomic<bool> running_{false};
  std::thread receive_thread_;
  std::thread decode_thread_;
  std::thread conversion_thread_;

  std::mutex pending_mutex_;
  std::condition_variable pending_condition_;
  std::deque<PendingFrame> pending_frames_;

  std::mutex output_mutex_;
  std::condition_variable output_condition_;
  AvFramePtr pending_output_frame_;

  std::atomic<bool> stream_discontinuity_{false};
  std::atomic<std::uint64_t> decoded_frames_window_{0};
  std::atomic<std::uint64_t> udp_packets_window_{0};
  std::atomic<std::uint64_t> dropped_{0};
  std::atomic<std::uint64_t> error_{0};
};

std::optional<AVCodecID> parse_codec(const std::string &name);

} // namespace rm_video
