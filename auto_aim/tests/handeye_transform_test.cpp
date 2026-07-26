#include <Eigen/Geometry>
#include <cmath>
#include <iostream>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace
{
cv::Mat eigen_to_cv(const Eigen::Matrix3d & matrix)
{
  cv::Mat result;
  cv::eigen2cv(matrix, result);
  return result;
}

cv::Mat eigen_to_cv(const Eigen::Vector3d & vector)
{
  cv::Mat result;
  cv::eigen2cv(vector, result);
  return result;
}
}  // namespace

int main()
{
  int failures = 0;
  const auto check = [&failures](bool condition, const std::string & message) {
    if (condition) return;
    std::cerr << "FAIL: " << message << std::endl;
    ++failures;
  };

  const Eigen::Matrix3d R_camera2gimbal_ideal{{0.0, 0.0, 1.0}, {-1.0, 0.0, 0.0}, {0.0, -1.0, 0.0}};
  const Eigen::Matrix3d R_camera2gimbal =
    Eigen::AngleAxisd(0.04, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
    Eigen::AngleAxisd(-0.03, Eigen::Vector3d::UnitY()).toRotationMatrix() * R_camera2gimbal_ideal;
  const Eigen::Vector3d t_camera2gimbal(120.0, -15.0, 45.0);

  const Eigen::Matrix3d R_board2world =
    Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
    Eigen::AngleAxisd(-0.2, Eigen::Vector3d::UnitY()).toRotationMatrix();
  const Eigen::Vector3d t_board2world(3000.0, 200.0, 800.0);

  const std::vector<Eigen::Vector3d> ypr_list{
    {0.0, 0.0, 0.0},     {0.25, 0.05, -0.02},  {-0.3, 0.08, 0.03}, {0.45, -0.12, 0.04},
    {-0.5, -0.1, -0.05}, {0.15, 0.2, 0.08},    {-0.2, -0.22, 0.1}, {0.6, 0.15, -0.1},
    {-0.65, 0.18, 0.12}, {0.35, -0.28, -0.08}, {-0.4, 0.3, 0.06},  {0.1, -0.32, 0.14}};

  std::vector<cv::Mat> R_gimbal2world_list;
  std::vector<cv::Mat> t_gimbal2world_list;
  std::vector<cv::Mat> R_board2camera_list;
  std::vector<cv::Mat> t_board2camera_list;
  for (const auto & ypr : ypr_list) {
    const Eigen::Matrix3d R_gimbal2world =
      Eigen::AngleAxisd(ypr.x(), Eigen::Vector3d::UnitZ()).toRotationMatrix() *
      Eigen::AngleAxisd(ypr.y(), Eigen::Vector3d::UnitY()).toRotationMatrix() *
      Eigen::AngleAxisd(ypr.z(), Eigen::Vector3d::UnitX()).toRotationMatrix();
    const Eigen::Matrix3d R_camera2world = R_gimbal2world * R_camera2gimbal;
    const Eigen::Vector3d t_camera2world = R_gimbal2world * t_camera2gimbal;
    const Eigen::Matrix3d R_world2camera = R_camera2world.transpose();
    const Eigen::Vector3d t_world2camera = -R_world2camera * t_camera2world;
    const Eigen::Matrix3d R_board2camera = R_world2camera * R_board2world;
    const Eigen::Vector3d t_board2camera = R_world2camera * t_board2world + t_world2camera;

    R_gimbal2world_list.push_back(eigen_to_cv(R_gimbal2world));
    t_gimbal2world_list.push_back(cv::Mat::zeros(3, 1, CV_64F));
    R_board2camera_list.push_back(eigen_to_cv(R_board2camera));
    t_board2camera_list.push_back(eigen_to_cv(t_board2camera));
  }

  cv::Mat estimated_R_camera2gimbal;
  cv::Mat estimated_t_camera2gimbal;
  cv::calibrateHandEye(
    R_gimbal2world_list, t_gimbal2world_list, R_board2camera_list, t_board2camera_list,
    estimated_R_camera2gimbal, estimated_t_camera2gimbal);

  Eigen::Matrix3d estimated_R;
  Eigen::Vector3d estimated_t;
  cv::cv2eigen(estimated_R_camera2gimbal, estimated_R);
  cv::cv2eigen(estimated_t_camera2gimbal, estimated_t);
  const Eigen::Matrix3d rotation_error = estimated_R.transpose() * R_camera2gimbal;
  const auto rotation_error_rad = Eigen::AngleAxisd(rotation_error).angle();

  check(rotation_error_rad < 1e-6, "camera-to-gimbal rotation direction should match Solver");
  check(
    (estimated_t - t_camera2gimbal).norm() < 1e-5,
    "camera-to-gimbal translation direction should match Solver");

  if (failures == 0) std::cout << "Hand-eye transform test passed" << std::endl;
  return failures == 0 ? 0 : 1;
}
