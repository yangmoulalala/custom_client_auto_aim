#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/target.hpp"
#include "tools/math_tools.hpp"

namespace
{
int failures = 0;

void expect(bool condition, const std::string & message)
{
  if (condition) return;
  std::cerr << "FAIL: " << message << '\n';
  ++failures;
}

double selected_delta_angle(const auto_aim::Aimer & aimer)
{
  return tools::limit_rad(aimer.debug_aim_point.xyza[3]);
}

void test_fast_spin_uses_rotation_direction()
{
  const auto timestamp = std::chrono::steady_clock::time_point{};

  auto_aim::Aimer positive_aimer(AIMER_SPIN_SELECTION_TEST_CONFIG);
  const auto positive_command =
    positive_aimer.aim({auto_aim::Target(4.0, 3.0, 0.2, 0.0)}, timestamp, 23.0, false);
  expect(positive_command.control, "positive fast spin must produce a control command");
  expect(
    selected_delta_angle(positive_aimer) < 0,
    "positive fast spin must select the armor approaching from negative delta angle");

  auto_aim::Aimer negative_aimer(AIMER_SPIN_SELECTION_TEST_CONFIG);
  const auto negative_command =
    negative_aimer.aim({auto_aim::Target(4.0, -3.0, 0.2, 0.0)}, timestamp, 23.0, false);
  expect(negative_command.control, "negative fast spin must produce a control command");
  expect(
    selected_delta_angle(negative_aimer) > 0,
    "negative fast spin must select the armor approaching from positive delta angle");
}

void test_configured_spin_threshold()
{
  const auto timestamp = std::chrono::steady_clock::time_point{};
  auto_aim::Aimer aimer(AIMER_SPIN_SELECTION_TEST_CONFIG);
  const auto command = aimer.aim({auto_aim::Target(4.0, 2.25, 0.2, 0.0)}, timestamp, 23.0, false);
  const auto delta_angle = selected_delta_angle(aimer);

  expect(command.control, "speed below the configured threshold must remain controllable");
  expect(delta_angle > 0, "speed below the configured threshold must keep the frontal armor");
  expect(
    std::abs(delta_angle) <= 60.0 / 57.3,
    "speed below the configured threshold must use the fixed 60 degree window");
}

void test_empty_target_is_safe()
{
  const auto timestamp = std::chrono::steady_clock::time_point{};
  auto_aim::Aimer aimer(AIMER_SPIN_SELECTION_TEST_CONFIG);
  const auto command = aimer.aim({}, timestamp, 23.0, false);

  expect(!command.control, "empty target input must disable control");
  expect(!command.shoot, "empty target input must disable shooting");
}

void test_invalid_spin_threshold_is_rejected()
{
  bool rejected = false;
  try {
    auto_aim::Aimer aimer(AIMER_SPIN_SELECTION_INVALID_CONFIG);
  } catch (const std::invalid_argument &) {
    rejected = true;
  }
  expect(rejected, "negative min_spin_speed must be rejected");
}
}  // namespace

int main()
{
  test_fast_spin_uses_rotation_direction();
  test_configured_spin_threshold();
  test_empty_target_is_safe();
  test_invalid_spin_threshold_is_rejected();

  if (failures != 0) {
    std::cerr << failures << " aimer spin selection checks failed\n";
    return 1;
  }
  std::cout << "Aimer spin selection tests passed\n";
  return 0;
}
