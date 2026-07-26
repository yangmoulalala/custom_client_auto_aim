#include <Eigen/Geometry>
#include <cmath>
#include <iostream>
#include <opencv2/core.hpp>
#include <string>
#include <vector>

#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/solver.hpp"

namespace
{
auto_aim::Armor make_armor(const std::vector<cv::Point2f> & points)
{
  return auto_aim::Armor(0, 1.0F, cv::boundingRect(points), points);
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

  auto_aim::Solver reference_solver(SOLVER_IMAGE_SIZE_TEST_CONFIG);
  auto_aim::Solver scaled_solver(SOLVER_IMAGE_SIZE_TEST_CONFIG);
  auto_aim::Solver invalid_solver(SOLVER_IMAGE_SIZE_TEST_CONFIG);
  check(reference_solver.set_image_size({1280, 720}), "reference resolution should be accepted");
  check(
    scaled_solver.set_image_size({640, 360}), "same-aspect scaled resolution should be accepted");
  check(!invalid_solver.set_image_size({640, 480}), "different aspect ratio should be rejected");
  check(
    !reference_solver.set_image_size({640, 360}),
    "runtime resolution change should be rejected after initialization");

  reference_solver.set_R_gimbal2world(Eigen::Quaterniond::Identity());
  scaled_solver.set_R_gimbal2world(Eigen::Quaterniond::Identity());

  const std::vector<cv::Point2f> reference_points{
    {590.0F, 330.0F}, {690.0F, 330.0F}, {690.0F, 390.0F}, {590.0F, 390.0F}};
  std::vector<cv::Point2f> scaled_points;
  for (const auto & point : reference_points) scaled_points.emplace_back(point * 0.5F);

  auto reference_armor = make_armor(reference_points);
  auto scaled_armor = make_armor(scaled_points);
  reference_solver.solve(reference_armor);
  scaled_solver.solve(scaled_armor);
  check(
    (reference_armor.xyz_in_gimbal - scaled_armor.xyz_in_gimbal).norm() < 1e-6,
    "scaled intrinsics and image points should preserve the solved 3D position");

  if (failures == 0) std::cout << "Solver image-size tests passed" << std::endl;
  return failures == 0 ? 0 : 1;
}
