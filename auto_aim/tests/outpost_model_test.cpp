#include <Eigen/Dense>
#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <list>
#include <string>
#include <vector>

#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/target.hpp"
#include "tools/math_tools.hpp"

namespace
{
using auto_aim::Armor;
using auto_aim::ArmorPriority;
using auto_aim::Aimer;
using auto_aim::OUTPOST_HEIGHT_STEP;
using auto_aim::OUTPOST_RADIUS;
using auto_aim::Target;

int failures = 0;

void expect(bool condition, const std::string & message)
{
  if (condition) return;
  std::cerr << "FAIL: " << message << '\n';
  failures++;
}

void expect_near(double actual, double expected, double tolerance, const std::string & message)
{
  expect(std::abs(actual - expected) <= tolerance, message);
}

Armor make_outpost_armor(const Eigen::Vector2d & center, double angle, double z)
{
  const std::vector<cv::Point2f> points{{0, 0}, {1, 0}, {1, 1}, {0, 1}};
  Armor armor(18, 1.0F, cv::Rect{}, points);
  armor.priority = ArmorPriority::fifth;
  armor.xyz_in_world = {
    center.x() - OUTPOST_RADIUS * std::cos(angle),
    center.y() - OUTPOST_RADIUS * std::sin(angle), z};
  armor.ypr_in_world = {angle, 0, 0};
  armor.ypd_in_world = tools::xyz2ypd(armor.xyz_in_world);
  return armor;
}

Eigen::VectorXd outpost_covariance()
{
  return Eigen::VectorXd{{1e-6, 64, 1e-6, 64, 1e-6, 64, 0.4, 100, 1e-4, 0, 0}};
}

Target make_outpost_target(double initial_angle, double initial_z = 1.0)
{
  const Eigen::Vector2d center{4.0, 0.0};
  auto armor = make_outpost_armor(center, initial_angle, initial_z);
  return Target(
    armor, std::chrono::steady_clock::time_point{}, OUTPOST_RADIUS, 3, outpost_covariance());
}

void observe_armor(Target & target, int id, double initial_angle, double z)
{
  const Eigen::Vector2d center{4.0, 0.0};
  const auto angle = tools::limit_rad(initial_angle + id * 2 * CV_PI / 3);
  target.update(make_outpost_armor(center, angle, z));
}

void test_height_layouts()
{
  const std::array<std::array<double, 3>, 6> layouts{{
    {{0, OUTPOST_HEIGHT_STEP, 2 * OUTPOST_HEIGHT_STEP}},
    {{0, 2 * OUTPOST_HEIGHT_STEP, OUTPOST_HEIGHT_STEP}},
    {{0, -OUTPOST_HEIGHT_STEP, OUTPOST_HEIGHT_STEP}},
    {{0, OUTPOST_HEIGHT_STEP, -OUTPOST_HEIGHT_STEP}},
    {{0, -2 * OUTPOST_HEIGHT_STEP, -OUTPOST_HEIGHT_STEP}},
    {{0, -OUTPOST_HEIGHT_STEP, -2 * OUTPOST_HEIGHT_STEP}},
  }};

  for (std::size_t layout_index = 0; layout_index < layouts.size(); layout_index++) {
    constexpr double BASE_Z = 1.2;
    auto target = make_outpost_target(0, BASE_Z);
    expect(!target.outpost_layout_complete(), "new outpost layout must be incomplete");

    observe_armor(target, 1, 0, BASE_Z + layouts[layout_index][1]);
    expect(!target.outpost_layout_complete(), "two observed armors must be incomplete");
    observe_armor(target, 2, 0, BASE_Z + layouts[layout_index][2]);
    expect(target.outpost_layout_complete(), "three observed armors must complete the layout");

    const auto armors = target.armor_xyza_list();
    expect(armors.size() == 3, "outpost must predict three armors");
    for (int id = 0; id < 3; id++) {
      expect_near(
        armors[id].z() - armors[0].z(), layouts[layout_index][id], 1e-6,
        "layout height offset must match the 2026 geometry");
      const Eigen::Vector2d center{target.ekf_x()[0], target.ekf_x()[2]};
      expect_near(
        (armors[id].head<2>() - center).norm(), OUTPOST_RADIUS, 1e-6,
        "outpost armor radius must be 0.275 m");
    }
    expect_near(
      tools::limit_rad(armors[1][3] - armors[0][3]), 2 * CV_PI / 3, 1e-9,
      "adjacent outpost armors must be separated by 120 degrees");
  }
}

void test_partial_and_complete_selection()
{
  constexpr double BASE_Z = 1.0;
  const auto timestamp = std::chrono::steady_clock::time_point{};

  auto initial_front = make_outpost_target(0, BASE_Z);
  Aimer initial_front_aimer(OUTPOST_MODEL_TEST_CONFIG);
  const auto initial_front_command = initial_front_aimer.aim({initial_front}, timestamp, 23.0, false);
  expect(initial_front_command.control, "stationary frontal outpost armor must be controllable");
  expect(initial_front_aimer.debug_aim_point.valid, "frontal outpost armor must permit firing");

  constexpr double OUTSIDE_ANGLE = 40.0 * CV_PI / 180.0;
  auto initial_outside = make_outpost_target(OUTSIDE_ANGLE, BASE_Z);
  Aimer initial_outside_aimer(OUTPOST_MODEL_TEST_CONFIG);
  const auto initial_outside_command =
    initial_outside_aimer.aim({initial_outside}, timestamp, 23.0, false);
  expect(!initial_outside_command.control, "initial armor outside 30 degrees must not be controlled");
  expect(!initial_outside_aimer.debug_aim_point.valid, "initial outside armor must not permit firing");

  constexpr double INITIAL_ANGLE = -2 * CV_PI / 3;
  auto partial = make_outpost_target(INITIAL_ANGLE, BASE_Z);
  observe_armor(partial, 1, INITIAL_ANGLE, BASE_Z + OUTPOST_HEIGHT_STEP);
  expect(!partial.outpost_layout_complete(), "partial selection setup must remain incomplete");

  Aimer partial_aimer(OUTPOST_MODEL_TEST_CONFIG);
  const auto partial_command = partial_aimer.aim({partial}, timestamp, 23.0, false);
  expect(partial_command.control, "stationary current outpost armor must remain controllable");
  expect(partial_aimer.debug_aim_point.valid, "stationary current armor must be a valid aim point");
  expect_near(
    partial_aimer.debug_aim_point.xyza.z() - BASE_Z, OUTPOST_HEIGHT_STEP, 1e-4,
    "incomplete layout must aim at the current observed armor");

  auto complete = partial;
  observe_armor(complete, 2, INITIAL_ANGLE, BASE_Z + 2 * OUTPOST_HEIGHT_STEP);
  expect(complete.outpost_layout_complete(), "complete selection setup must observe all armors");
  Aimer complete_aimer(OUTPOST_MODEL_TEST_CONFIG);
  const auto complete_command = complete_aimer.aim({complete}, timestamp, 23.0, false);
  expect(complete_command.control, "complete stationary layout must select a frontal armor");
  expect_near(
    complete_aimer.debug_aim_point.xyza.z() - BASE_Z, OUTPOST_HEIGHT_STEP, 1e-4,
    "complete layout must preserve the frontal armor height");

  auto outside = make_outpost_target(OUTSIDE_ANGLE - 2 * CV_PI / 3, BASE_Z);
  observe_armor(outside, 1, OUTSIDE_ANGLE - 2 * CV_PI / 3, BASE_Z + OUTPOST_HEIGHT_STEP);
  Aimer outside_aimer(OUTPOST_MODEL_TEST_CONFIG);
  const auto outside_command = outside_aimer.aim({outside}, timestamp, 23.0, false);
  expect(!outside_command.control, "stationary armor outside 30 degrees must not be controlled");
  expect(!outside_aimer.debug_aim_point.valid, "outside stationary armor must not permit shooting");
}

void test_bidirectional_delay()
{
  const auto timestamp = std::chrono::steady_clock::time_point{};
  Target positive(4.0, 1.5, 0.2, 0.0);
  Target negative(4.0, -1.5, 0.2, 0.0);
  Aimer positive_aimer(OUTPOST_MODEL_TEST_CONFIG);
  Aimer negative_aimer(OUTPOST_MODEL_TEST_CONFIG);

  const auto positive_command = positive_aimer.aim({positive}, timestamp, 23.0, false);
  const auto negative_command = negative_aimer.aim({negative}, timestamp, 23.0, false);
  expect(positive_command.control && negative_command.control, "both spin directions must be controllable");
  expect_near(
    positive_command.yaw, -negative_command.yaw, 1e-9,
    "equal absolute spin speeds must use symmetric delay prediction");
}

Target tracked_outpost(double angular_velocity)
{
  constexpr double DT = 0.02;
  constexpr int UPDATE_COUNT = 12;
  const Eigen::Vector2d center{4.0, 0.0};
  auto target = make_outpost_target(0);
  const auto t0 = std::chrono::steady_clock::time_point{};

  for (int i = 1; i <= UPDATE_COUNT; i++) {
    const auto elapsed = i * DT;
    const auto timestamp = t0 + std::chrono::microseconds(static_cast<int>(elapsed * 1e6));
    target.predict(timestamp);
    target.update(make_outpost_armor(center, angular_velocity * elapsed, 1.0));
  }
  target.predict(0.0);
  return target;
}

void test_speed_snap()
{
  const auto stationary = tracked_outpost(0.0);
  expect_near(stationary.ekf_x()[7], 0.0, 1e-6, "stationary outpost speed must remain zero");

  const auto low_speed = tracked_outpost(1.5);
  expect(
    low_speed.ekf_x()[7] > 1.0 && low_speed.ekf_x()[7] < 2.0,
    "outpost speed below 2 rad/s must not snap");

  const auto positive = tracked_outpost(3.0);
  expect_near(positive.ekf_x()[7], 2.51, 1e-6, "positive high speed must snap to 2.51 rad/s");

  const auto negative = tracked_outpost(-3.0);
  expect_near(negative.ekf_x()[7], -2.51, 1e-6, "negative high speed must snap to -2.51 rad/s");
}
}  // namespace

int main()
{
  test_height_layouts();
  test_partial_and_complete_selection();
  test_bidirectional_delay();
  test_speed_snap();

  if (failures != 0) {
    std::cerr << failures << " outpost model checks failed\n";
    return 1;
  }
  std::cout << "All outpost model checks passed\n";
  return 0;
}
