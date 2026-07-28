#include "rm_video/video_decoder.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cstring>
#include <rclcpp/rclcpp.hpp>
#include <utility>

extern "C" {
#include <libavutil/error.h>
#include <libavutil/imgutils.h>
#include <libavutil/log.h>
}

namespace rm_video
{
namespace
{

constexpr std::size_t kPacketHeaderSize = 8;
constexpr std::size_t kMaximumDatagramSize = 65535;

struct PacketHeader
{
  std::uint16_t frame_sequence;
  std::uint16_t fragment_sequence;
  std::uint32_t total_size;
};

std::string ffmpeg_error(int error)
{
  std::array<char, AV_ERROR_MAX_STRING_SIZE> buffer{};
  av_strerror(error, buffer.data(), buffer.size());
  return buffer.data();
}

bool parse_header(
  const std::uint8_t * data, std::size_t size, std::size_t max_frame_bytes, PacketHeader & output)
{
  if (size < kPacketHeaderSize) {
    return false;
  }

  std::uint16_t frame_sequence = 0;
  std::uint16_t fragment_sequence = 0;
  std::uint32_t total_size = 0;
  std::memcpy(&frame_sequence, data, sizeof(frame_sequence));
  std::memcpy(&fragment_sequence, data + 2, sizeof(fragment_sequence));
  std::memcpy(&total_size, data + 4, sizeof(total_size));

  const PacketHeader network_order{
    ntohs(frame_sequence), ntohs(fragment_sequence), ntohl(total_size)};
  const PacketHeader host_order{frame_sequence, fragment_sequence, total_size};
  const auto plausible = [max_frame_bytes](const PacketHeader & header) {
    return header.total_size > 0 && header.total_size <= max_frame_bytes;
  };

  if (plausible(network_order)) {
    output = network_order;
    return true;
  }
  if (plausible(host_order)) {
    output = host_order;
    return true;
  }
  return false;
}

}  // namespace

std::optional<AVCodecID> parse_codec(const std::string & name)
{
  if (name == "hevc" || name == "h265") {
    return AV_CODEC_ID_HEVC;
  }
  if (name == "h264") {
    return AV_CODEC_ID_H264;
  }
  if (name == "mjpeg") {
    return AV_CODEC_ID_MJPEG;
  }
  return std::nullopt;
}

void VideoDecoder::FrameAssembly::reset()
{
  frame_sequence = 0;
  total_size = 0;
  received_size = 0;
  active = false;
  fragments.clear();
}

VideoDecoder::VideoDecoder(DecoderConfig config, FrameCallback callback, rclcpp::Logger logger)
: config_(std::move(config)),
  callback_(std::move(callback)),
  logger_(std::move(logger)),
  assemblies_(config_.reassembly_slots)
{
}

VideoDecoder::~VideoDecoder()
{
  stop();
  cleanup();
}

bool VideoDecoder::start()
{
  if (running_.load()) {
    return true;
  }
  if (!initialize_decoder() || !setup_socket()) {
    cleanup();
    return false;
  }

  running_.store(true);
  try {
    receive_thread_ = std::thread(&VideoDecoder::receive_loop, this);
    conversion_thread_ = std::thread(&VideoDecoder::conversion_loop, this);
    decode_thread_ = std::thread(&VideoDecoder::decode_loop, this);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(logger_, "Failed to start worker threads: %s", error.what());
    stop();
    cleanup();
    return false;
  }

  RCLCPP_INFO(
    logger_, "Listening for %s video on %s:%d", codec_->name, config_.bind_address.c_str(),
    config_.port);
  return true;
}

void VideoDecoder::stop()
{
  running_.store(false);
  pending_condition_.notify_all();
  output_condition_.notify_all();
  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }

  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    if (!pending_frames_.empty()) {
      std::size_t packet_count = 0;
      for (const auto & frame : pending_frames_) {
        packet_count += frame.packet_count;
      }
      dropped_.fetch_add(packet_count);
      RCLCPP_WARN(
        logger_,
        "Dropped %zu queued frames (%zu UDP packets): decoder "
        "stopping",
        pending_frames_.size(), packet_count);
      pending_frames_.clear();
    }
  }
  pending_condition_.notify_all();
  if (decode_thread_.joinable()) {
    decode_thread_.join();
  }
  {
    std::lock_guard<std::mutex> lock(output_mutex_);
    pending_output_frame_.reset();
  }
  output_condition_.notify_all();
  if (conversion_thread_.joinable()) {
    conversion_thread_.join();
  }
}

Statistics VideoDecoder::take_statistics()
{
  return Statistics{
    decoded_frames_window_.exchange(0), udp_packets_window_.exchange(0), dropped_.load(),
    error_.load()};
}

bool VideoDecoder::initialize_decoder()
{
  av_log_set_level(AV_LOG_QUIET);
  codec_ = avcodec_find_decoder(config_.codec_id);
  if (codec_ == nullptr) {
    RCLCPP_ERROR(logger_, "FFmpeg decoder was not found");
    return false;
  }

  codec_context_ = avcodec_alloc_context3(codec_);
  if (codec_context_ == nullptr) {
    RCLCPP_ERROR(logger_, "Failed to allocate FFmpeg decoder context");
    return false;
  }

  // 只启用切片并行，利用多核解码且不引入帧级缓存延迟。
  codec_context_->thread_count = config_.decoder_threads;
  codec_context_->thread_type = FF_THREAD_SLICE;
  codec_context_->flags |= AV_CODEC_FLAG_LOW_DELAY;
  codec_context_->flags2 |= AV_CODEC_FLAG2_FAST;
  codec_context_->err_recognition = AV_EF_CAREFUL;

  const int open_result = avcodec_open2(codec_context_, codec_, nullptr);
  if (open_result < 0) {
    RCLCPP_ERROR(logger_, "Failed to open FFmpeg decoder: %s", ffmpeg_error(open_result).c_str());
    return false;
  }

  packet_ = av_packet_alloc();
  frame_ = av_frame_alloc();
  if (packet_ == nullptr || frame_ == nullptr) {
    RCLCPP_ERROR(logger_, "Failed to allocate FFmpeg packet or frame");
    return false;
  }
  return true;
}

bool VideoDecoder::setup_socket()
{
  socket_fd_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (socket_fd_ < 0) {
    RCLCPP_ERROR(logger_, "Failed to create UDP socket: %s", std::strerror(errno));
    return false;
  }

  const int reuse_address = 1;
  setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse_address, sizeof(reuse_address));
  if (
    setsockopt(
      socket_fd_, SOL_SOCKET, SO_RCVBUF, &config_.receive_buffer_bytes,
      sizeof(config_.receive_buffer_bytes)) < 0) {
    RCLCPP_WARN(logger_, "Failed to set UDP receive buffer: %s", std::strerror(errno));
  }

  const timeval timeout{
    config_.receive_timeout_ms / 1000, (config_.receive_timeout_ms % 1000) * 1000};
  if (setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
    RCLCPP_ERROR(logger_, "Failed to set UDP receive timeout: %s", std::strerror(errno));
    return false;
  }

  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_port = htons(static_cast<std::uint16_t>(config_.port));
  if (inet_pton(AF_INET, config_.bind_address.c_str(), &address.sin_addr) != 1) {
    RCLCPP_ERROR(logger_, "Invalid IPv4 bind address: %s", config_.bind_address.c_str());
    return false;
  }
  if (bind(socket_fd_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0) {
    RCLCPP_ERROR(logger_, "Failed to bind UDP socket: %s", std::strerror(errno));
    return false;
  }
  return true;
}

void VideoDecoder::receive_loop()
{
  std::array<std::uint8_t, kMaximumDatagramSize> buffer{};
  while (running_.load()) {
    const ssize_t received =
      recvfrom(socket_fd_, buffer.data(), buffer.size(), 0, nullptr, nullptr);
    if (received >= 0) {
      udp_packets_window_.fetch_add(1);
      process_datagram(buffer.data(), static_cast<std::size_t>(received));
      continue;
    }
    if (received < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
      expire_assemblies(std::chrono::steady_clock::now());
      continue;
    }
    if (running_.load()) {
      RCLCPP_ERROR(logger_, "UDP receive failed: %s", std::strerror(errno));
      running_.store(false);
      pending_condition_.notify_all();
    }
  }
}

void VideoDecoder::decode_loop()
{
  while (true) {
    PendingFrame pending;
    {
      std::unique_lock<std::mutex> lock(pending_mutex_);
      pending_condition_.wait(
        lock, [this]() { return !running_.load() || !pending_frames_.empty(); });
      if (!running_.load()) {
        return;
      }
      pending = std::move(pending_frames_.front());
      pending_frames_.pop_front();
    }
    decode_frame(std::move(pending));
  }
}

void VideoDecoder::process_datagram(const std::uint8_t * data, std::size_t size)
{
  expire_assemblies(std::chrono::steady_clock::now());

  PacketHeader header{};
  if (!parse_header(data, size, config_.max_frame_bytes, header)) {
    dropped_.fetch_add(1);
    RCLCPP_WARN(logger_, "Dropped UDP packet: invalid packet header");
    return;
  }

  const std::size_t payload_size = size - kPacketHeaderSize;
  if (payload_size == 0 || payload_size > header.total_size) {
    dropped_.fetch_add(1);
    RCLCPP_WARN(
      logger_, "Dropped UDP packet from frame %u: invalid payload size", header.frame_sequence);
    return;
  }

  FrameAssembly & assembly = get_or_create_assembly(header.frame_sequence, header.total_size);
  if (assembly.fragments.find(header.fragment_sequence) != assembly.fragments.end()) {
    dropped_.fetch_add(1);
    RCLCPP_WARN(
      logger_, "Dropped UDP packet from frame %u: duplicate fragment %u", header.frame_sequence,
      header.fragment_sequence);
    return;
  }

  assembly.fragments.emplace(
    header.fragment_sequence, std::vector<std::uint8_t>(data + kPacketHeaderSize, data + size));
  assembly.received_size += payload_size;
  assembly.last_update = std::chrono::steady_clock::now();
  if (assembly.received_size < assembly.total_size) {
    return;
  }

  PendingFrame complete_frame;
  complete_frame.data.reserve(assembly.total_size + AV_INPUT_BUFFER_PADDING_SIZE);
  complete_frame.packet_count = assembly.fragments.size();
  for (const auto & [sequence, fragment] : assembly.fragments) {
    (void)sequence;
    complete_frame.data.insert(complete_frame.data.end(), fragment.begin(), fragment.end());
  }

  if (complete_frame.data.size() != assembly.total_size) {
    discard_assembly(assembly, "reassembled size mismatch");
    return;
  }
  assembly.reset();
  complete_frame.frame_sequence = header.frame_sequence;
  complete_frame.discontinuity = stream_discontinuity_.exchange(false);
  submit_frame(std::move(complete_frame));
}

VideoDecoder::FrameAssembly & VideoDecoder::get_or_create_assembly(
  std::uint16_t frame_sequence, std::uint32_t total_size)
{
  for (auto & assembly : assemblies_) {
    if (assembly.active && assembly.frame_sequence == frame_sequence) {
      if (assembly.total_size != total_size) {
        discard_assembly(assembly, "frame size changed during reassembly");
      } else {
        return assembly;
      }
      break;
    }
  }

  auto available = std::find_if(
    assemblies_.begin(), assemblies_.end(),
    [](const FrameAssembly & assembly) { return !assembly.active; });
  if (available == assemblies_.end()) {
    available = std::min_element(
      assemblies_.begin(), assemblies_.end(),
      [](const FrameAssembly & left, const FrameAssembly & right) {
        return left.last_update < right.last_update;
      });
    discard_assembly(*available, "reassembly slots exhausted");
  }

  available->active = true;
  available->frame_sequence = frame_sequence;
  available->total_size = total_size;
  available->received_size = 0;
  available->last_update = std::chrono::steady_clock::now();
  return *available;
}

void VideoDecoder::expire_assemblies(std::chrono::steady_clock::time_point now)
{
  for (auto & assembly : assemblies_) {
    if (assembly.active && now - assembly.last_update > config_.reassembly_timeout) {
      discard_assembly(assembly, "reassembly timeout");
    }
  }
}

void VideoDecoder::submit_frame(PendingFrame frame)
{
  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    // 小型有界队列吸收关键帧解码峰值；真正溢出时清空积压恢复实时性。
    if (pending_frames_.size() >= config_.decode_queue_size) {
      std::size_t packet_count = 0;
      for (const auto & pending : pending_frames_) {
        packet_count += pending.packet_count;
      }
      const std::uint16_t first_sequence = pending_frames_.front().frame_sequence;
      const std::uint16_t last_sequence = pending_frames_.back().frame_sequence;
      dropped_.fetch_add(packet_count);
      RCLCPP_WARN(
        logger_,
        "Dropped %zu queued frames %u-%u (%zu UDP packets): decoder "
        "queue full before frame %u",
        pending_frames_.size(), first_sequence, last_sequence, packet_count, frame.frame_sequence);
      pending_frames_.clear();
      frame.discontinuity = true;
    }
    pending_frames_.push_back(std::move(frame));
  }
  pending_condition_.notify_one();
}

void VideoDecoder::decode_frame(PendingFrame pending)
{
  if (pending.discontinuity) {
    avcodec_flush_buffers(codec_context_);
  }
  const std::size_t encoded_size = pending.data.size();
  pending.data.resize(encoded_size + AV_INPUT_BUFFER_PADDING_SIZE, 0);

  av_packet_unref(packet_);
  packet_->data = pending.data.data();
  packet_->size = static_cast<int>(encoded_size);
  const int send_result = avcodec_send_packet(codec_context_, packet_);
  packet_->data = nullptr;
  packet_->size = 0;
  if (send_result < 0) {
    error_.fetch_add(1);
    return;
  }

  bool packet_failed = false;
  while (true) {
    const int receive_result = avcodec_receive_frame(codec_context_, frame_);
    if (receive_result == AVERROR(EAGAIN) || receive_result == AVERROR_EOF) {
      break;
    }
    if (receive_result < 0) {
      packet_failed = true;
      break;
    }

    if (
      (frame_->flags & AV_FRAME_FLAG_CORRUPT) != 0 || frame_->decode_error_flags != 0 ||
      !submit_output_frame(*frame_)) {
      packet_failed = true;
      av_frame_unref(frame_);
      break;
    }
    av_frame_unref(frame_);
  }
  if (packet_failed) {
    error_.fetch_add(1);
  }
}

bool VideoDecoder::submit_output_frame(const AVFrame & frame)
{
  AvFramePtr output(av_frame_clone(&frame));
  if (!output) {
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(output_mutex_);
    if (!running_.load()) {
      return true;
    }
    // 已完成 FFmpeg 解码的画面可以安全覆盖，不会破坏后续帧的参考链。
    pending_output_frame_ = std::move(output);
  }
  output_condition_.notify_one();
  return true;
}

void VideoDecoder::conversion_loop()
{
  while (true) {
    AvFramePtr source;
    {
      std::unique_lock<std::mutex> lock(output_mutex_);
      output_condition_.wait(
        lock, [this]() { return !running_.load() || pending_output_frame_ != nullptr; });
      if (!running_.load()) {
        return;
      }
      source = std::move(pending_output_frame_);
    }

    DecodedFrame output;
    if (!convert_frame(*source, output)) {
      error_.fetch_add(1);
      continue;
    }
    decoded_frames_window_.fetch_add(1);
    callback_(std::move(output));
  }
}

bool VideoDecoder::convert_frame(const AVFrame & source, DecodedFrame & output)
{
  if (source.width <= 0 || source.height <= 0 || source.format < 0) {
    return false;
  }

  sws_context_ = sws_getCachedContext(
    sws_context_, source.width, source.height, static_cast<AVPixelFormat>(source.format),
    source.width, source.height, AV_PIX_FMT_BGR24, SWS_FAST_BILINEAR, nullptr, nullptr, nullptr);
  if (sws_context_ == nullptr) {
    return false;
  }

  output.width = source.width;
  output.height = source.height;
  output.bgr.resize(static_cast<std::size_t>(output.width) * output.height * 3);
  std::uint8_t * destination[] = {output.bgr.data(), nullptr, nullptr, nullptr};
  const int destination_stride[] = {output.width * 3, 0, 0, 0};
  const int converted_height = sws_scale(
    sws_context_, source.data, source.linesize, 0, source.height, destination, destination_stride);
  if (converted_height != output.height) {
    return false;
  }
  return true;
}

void VideoDecoder::discard_assembly(FrameAssembly & assembly, const char * reason)
{
  const std::size_t packet_count = assembly.fragments.size();
  dropped_.fetch_add(packet_count);
  if (packet_count > 0) {
    stream_discontinuity_.store(true);
    RCLCPP_WARN(
      logger_, "Dropped incomplete frame %u (%zu UDP packets): %s", assembly.frame_sequence,
      packet_count, reason);
  }
  assembly.reset();
}

void VideoDecoder::cleanup()
{
  if (socket_fd_ >= 0) {
    close(socket_fd_);
    socket_fd_ = -1;
  }
  if (sws_context_ != nullptr) {
    sws_freeContext(sws_context_);
    sws_context_ = nullptr;
  }
  if (frame_ != nullptr) {
    av_frame_free(&frame_);
  }
  if (packet_ != nullptr) {
    av_packet_free(&packet_);
  }
  if (codec_context_ != nullptr) {
    avcodec_free_context(&codec_context_);
  }
  codec_ = nullptr;
}

}  // namespace rm_video
