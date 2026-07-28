#include <Eigen/Geometry>

#include <chrono>
#include <cmath>
#include <iostream>
#include <list>
#include <opencv2/core.hpp>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"

namespace
{
int failures = 0;

void expect(bool condition, const std::string & message)
{
  if (condition) return;
  std::cerr << "FAIL: " << message << '\n';
  ++failures;
}

auto_aim::Armor make_armor(
  auto_aim::Solver & solver, auto_aim::ArmorName name, auto_aim::ArmorType type)
{
  const Eigen::Vector3d position{0, 0, 5};
  auto points = solver.reproject_armor(position, 0, type, name);
  auto_aim::Armor armor(0, 3, 1.0F, cv::boundingRect(points), points);
  armor.name = name;
  armor.type = type;
  armor.color = auto_aim::Color::red;
  armor.priority = auto_aim::ArmorPriority::fifth;
  armor.center_norm = {0.5F, 0.5F};
  return armor;
}

std::optional<auto_aim::Target> track_once(auto_aim::Solver & solver, auto_aim::Armor armor)
{
  auto_aim::Tracker tracker(TARGET_LAYOUT_TEST_CONFIG, solver);
  std::list<auto_aim::Armor> armors{std::move(armor)};
  const auto targets = tracker.track(armors, std::chrono::steady_clock::time_point{});
  expect(targets.size() == 1, "valid first detection must initialize one target");
  if (targets.empty()) return std::nullopt;
  return targets.front();
}
}  // namespace

int main()
{
  const std::vector<cv::Point2f> classification_points{{0, 0}, {1, 0}, {1, 1}, {0, 1}};
  const cv::Rect classification_box(0, 0, 1, 1);
  const auto_aim::Armor infantry_class(0, 3, 1.0F, classification_box, classification_points);
  const auto_aim::Armor base_class(0, 7, 1.0F, classification_box, classification_points);
  const auto_aim::Armor legacy_base_class(0, 8, 1.0F, classification_box, classification_points);
  const auto_aim::Armor retired_balance_class(
    29, 1.0F, classification_box, classification_points);
  expect(
    infantry_class.type == auto_aim::ArmorType::small,
    "3/4/5 classes must use small armor");
  expect(base_class.type == auto_aim::ArmorType::small, "base class must use small armor");
  expect(
    legacy_base_class.type == auto_aim::ArmorType::small,
    "legacy base class must use small armor");
  expect(
    retired_balance_class.name == auto_aim::ArmorName::not_armor,
    "retired balance-infantry classes must be invalid");

  auto_aim::Solver solver(TARGET_LAYOUT_TEST_CONFIG);
  expect(solver.set_image_size({1280, 720}), "test image size must match calibration");
  solver.set_R_gimbal2world(Eigen::Quaterniond::Identity());

  const auto infantry = track_once(
    solver, make_armor(solver, auto_aim::ArmorName::three, auto_aim::ArmorType::big));
  if (infantry) {
    expect(
      infantry->armor_xyza_list().size() == 4,
      "3/4/5 targets must not initialize the retired two-armor layout");
  }

  const auto base =
    track_once(solver, make_armor(solver, auto_aim::ArmorName::base, auto_aim::ArmorType::small));
  if (base) {
    expect(base->armor_type == auto_aim::ArmorType::small, "base target must use small armor");
    expect(base->armor_xyza_list().size() == 3, "base target must keep the three-armor layout");
    expect(std::abs(base->ekf_x()[8] - 0.3205) < 1e-9, "base radius must remain 0.3205 m");
  }

  if (failures == 0) std::cout << "Target layout tests passed\n";
  return failures == 0 ? 0 : 1;
}
