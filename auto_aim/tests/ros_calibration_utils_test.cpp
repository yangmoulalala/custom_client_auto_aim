#include "calibration/ros_calibration_utils.hpp"

#include <cmath>
#include <cstdint>
#include <deque>
#include <iostream>
#include <limits>
#include <memory>
#include <opencv2/imgcodecs.hpp>
#include <string>
#include <vector>

namespace
{
int failures = 0;

void check(bool condition, const std::string & message)
{
  if (condition) return;
  std::cerr << "FAIL: " << message << '\n';
  ++failures;
}

calibration::ImuMsg::ConstSharedPtr make_imu(
  std::int64_t stamp_ns, double w = 1.0, double x = 0.0, double y = 0.0, double z = 0.0)
{
  auto imu = std::make_shared<calibration::ImuMsg>();
  imu->header.stamp.sec = static_cast<std::int32_t>(stamp_ns / 1000000000LL);
  imu->header.stamp.nanosec = static_cast<std::uint32_t>(stamp_ns % 1000000000LL);
  imu->orientation.w = w;
  imu->orientation.x = x;
  imu->orientation.y = y;
  imu->orientation.z = z;
  return imu;
}

void test_decode_compressed_image()
{
  cv::Mat source(7, 11, CV_8UC3, cv::Scalar(20, 80, 160));
  std::vector<unsigned char> encoded;
  check(cv::imencode(".jpg", source, encoded), "test JPEG should encode");

  auto message = std::make_shared<calibration::ImageMsg>();
  message->format = "jpeg";
  message->data = encoded;
  cv::Mat decoded;
  check(
    calibration::decode_compressed_image(message, decoded), "valid JPEG should decode");
  check(decoded.size() == source.size(), "decoded JPEG dimensions should match");
  check(decoded.type() == CV_8UC3, "decoded JPEG should be BGR8");

  auto empty = std::make_shared<calibration::ImageMsg>();
  check(
    !calibration::decode_compressed_image(empty, decoded), "empty payload should be rejected");
  check(decoded.empty(), "failed decode should clear the previous image");

  auto corrupt = std::make_shared<calibration::ImageMsg>();
  corrupt->data = {0x01, 0x02, 0x03, 0x04};
  check(
    !calibration::decode_compressed_image(corrupt, decoded),
    "corrupt payload should be rejected");
}

void test_nearest_imu()
{
  const auto before = make_imu(970000000LL);
  const auto exact = make_imu(1000000000LL);
  const auto after = make_imu(1020000000LL);
  const std::deque<calibration::ImuMsg::ConstSharedPtr> buffer{before, exact, after};

  check(
    calibration::nearest_imu(buffer, 1000000000LL, 0) == exact,
    "equal timestamp should match with zero tolerance");

  const std::deque<calibration::ImuMsg::ConstSharedPtr> surrounding{before, after};
  check(
    calibration::nearest_imu(surrounding, 1000000000LL, 20000000LL) == after,
    "nearest sample after the image should be selected");
  check(
    calibration::nearest_imu(surrounding, 1000000000LL, 19999999LL) == nullptr,
    "sample beyond tolerance should be rejected");

  const auto closer_before = make_imu(990000000LL);
  const auto farther_after = make_imu(1030000000LL);
  const std::deque<calibration::ImuMsg::ConstSharedPtr> reverse_surrounding{
    farther_after, closer_before};
  check(
    calibration::nearest_imu(reverse_surrounding, 1000000000LL, 30000000LL) == closer_before,
    "nearest sample before the image should be selected regardless of buffer order");
  check(
    calibration::nearest_imu({}, 1000000000LL, 100000000LL) == nullptr,
    "empty IMU buffer should not match");

  const std::deque<calibration::ImuMsg::ConstSharedPtr> with_null{nullptr, before};
  check(
    calibration::nearest_imu(with_null, 1000000000LL, 30000000LL) == before,
    "null IMU entries should be ignored");
}

void test_imu_orientation()
{
  Eigen::Quaterniond orientation;
  check(
    calibration::imu_orientation(make_imu(0, 2.0, 0.0, 0.0, 0.0), orientation),
    "finite nonzero quaternion should be accepted");
  check(std::abs(orientation.norm() - 1.0) < 1e-12, "accepted quaternion should normalize");

  check(
    !calibration::imu_orientation(make_imu(0, 0.0, 0.0, 0.0, 0.0), orientation),
    "zero quaternion should be rejected");
  check(
    !calibration::imu_orientation(
      make_imu(0, std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0, 0.0), orientation),
    "non-finite quaternion should be rejected");

  auto unavailable = std::make_shared<calibration::ImuMsg>();
  unavailable->orientation_covariance[0] = -1.0;
  check(
    !calibration::imu_orientation(unavailable, orientation),
    "unavailable orientation should be rejected");
  check(
    !calibration::imu_orientation(nullptr, orientation), "null IMU should be rejected");
}
}  // namespace

int main()
{
  test_decode_compressed_image();
  test_nearest_imu();
  test_imu_orientation();
  if (failures != 0) return 1;
  std::cout << "ROS calibration utility tests passed\n";
  return 0;
}
