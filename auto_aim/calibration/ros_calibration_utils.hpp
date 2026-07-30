#ifndef CALIBRATION__ROS_CALIBRATION_UTILS_HPP_
#define CALIBRATION__ROS_CALIBRATION_UTILS_HPP_

#include <Eigen/Geometry>
#include <builtin_interfaces/msg/time.hpp>
#include <cstdint>
#include <deque>
#include <opencv2/core.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace calibration
{
using ImageMsg = sensor_msgs::msg::CompressedImage;
using ImuMsg = sensor_msgs::msg::Imu;

std::int64_t stamp_to_ns(const builtin_interfaces::msg::Time & stamp);
bool decode_compressed_image(const ImageMsg::ConstSharedPtr & msg, cv::Mat & image);
bool imu_orientation(const ImuMsg::ConstSharedPtr & msg, Eigen::Quaterniond & q);
ImuMsg::ConstSharedPtr nearest_imu(
  const std::deque<ImuMsg::ConstSharedPtr> & buffer, std::int64_t image_stamp_ns,
  std::int64_t tolerance_ns);
}  // namespace calibration

#endif  // CALIBRATION__ROS_CALIBRATION_UTILS_HPP_
