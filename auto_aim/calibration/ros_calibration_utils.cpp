#include "calibration/ros_calibration_utils.hpp"

#include <cmath>
#include <limits>
#include <opencv2/imgcodecs.hpp>

namespace calibration
{
std::int64_t stamp_to_ns(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<std::int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
}

bool decode_compressed_image(const ImageMsg::ConstSharedPtr & msg, cv::Mat & image)
{
  image.release();
  if (!msg || msg->data.empty()) return false;

  try {
    image = cv::imdecode(msg->data, cv::IMREAD_COLOR);
  } catch (const cv::Exception &) {
    image.release();
    return false;
  }
  return !image.empty();
}

bool imu_orientation(const ImuMsg::ConstSharedPtr & msg, Eigen::Quaterniond & q)
{
  if (!msg || msg->orientation_covariance[0] == -1.0) return false;
  const auto & orientation = msg->orientation;
  if (
    !std::isfinite(orientation.w) || !std::isfinite(orientation.x) ||
    !std::isfinite(orientation.y) || !std::isfinite(orientation.z)) {
    return false;
  }

  q = {orientation.w, orientation.x, orientation.y, orientation.z};
  if (!std::isfinite(q.squaredNorm()) || q.squaredNorm() < 1e-12) return false;
  q.normalize();
  return true;
}

ImuMsg::ConstSharedPtr nearest_imu(
  const std::deque<ImuMsg::ConstSharedPtr> & buffer, std::int64_t image_stamp_ns,
  std::int64_t tolerance_ns)
{
  ImuMsg::ConstSharedPtr nearest;
  auto nearest_delta = std::numeric_limits<std::int64_t>::max();
  for (const auto & imu : buffer) {
    if (!imu) continue;
    const auto imu_stamp_ns = stamp_to_ns(imu->header.stamp);
    const auto delta = imu_stamp_ns >= image_stamp_ns ? imu_stamp_ns - image_stamp_ns
                                                      : image_stamp_ns - imu_stamp_ns;
    if (delta < nearest_delta) {
      nearest = imu;
      nearest_delta = delta;
      if (delta == 0) break;
    }
  }
  return nearest && nearest_delta <= tolerance_ns ? nearest : nullptr;
}
}  // namespace calibration
