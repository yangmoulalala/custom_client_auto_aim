#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/target.hpp"

namespace
{
constexpr double RADIAN_PER_DEGREE = 1.0 / 57.3;

int failures = 0;

void expect(bool condition, const std::string & message)
{
  if (condition) return;
  std::cerr << "FAIL: " << message << '\n';
  ++failures;
}

void expect_near(double actual, double expected, double tolerance, const std::string & message)
{
  expect(std::abs(actual - expected) <= tolerance, message);
}

std::string config_path(const std::string & name)
{
  return std::string(AIMER_PITCH_OFFSET_TEST_CONFIG_DIR) + "/" + name;
}

io::Command aim_at_distance(const std::string & config_name, double distance)
{
  auto_aim::Aimer aimer(config_path(config_name));
  const auto timestamp = std::chrono::steady_clock::time_point{};
  return aimer.aim({auto_aim::Target(distance, 0.0, 0.0, 0.0)}, timestamp, 23.0, false);
}

void expect_offset(double distance, double expected_offset_degrees, const std::string & message)
{
  const auto calibrated = aim_at_distance("aimer_pitch_offset_test.yaml", distance);
  const auto zero = aim_at_distance("aimer_pitch_offset_zero.yaml", distance);
  expect(calibrated.control && zero.control, message + " must remain controllable");
  expect_near(
    calibrated.pitch - zero.pitch, -expected_offset_degrees * RADIAN_PER_DEGREE, 1e-9,
    message);
}

void test_two_point_mapping()
{
  expect_offset(1.0, -2.0, "distance below near point must use the near offset");
  expect_offset(2.0, -2.0, "near point must use the near offset");
  expect_offset(4.0, 0.0, "midpoint must linearly interpolate the offset");
  expect_offset(6.0, 2.0, "far point must use the far offset");
  expect_offset(7.0, 2.0, "distance above far point must use the far offset");
}

void test_legacy_scalar_offset()
{
  constexpr double LEGACY_OFFSET_DEGREES = -1.25;
  const auto legacy = aim_at_distance("aimer_pitch_offset_legacy.yaml", 4.0);
  const auto zero = aim_at_distance("aimer_pitch_offset_zero.yaml", 4.0);
  expect(legacy.control && zero.control, "legacy scalar pitch offset must remain controllable");
  expect_near(
    legacy.pitch - zero.pitch, -LEGACY_OFFSET_DEGREES * RADIAN_PER_DEGREE, 1e-9,
    "legacy scalar pitch offset must preserve the fixed-offset behavior");
}

void expect_invalid_config(const std::string & config_name, const std::string & message)
{
  bool rejected = false;
  try {
    auto_aim::Aimer aimer(config_path(config_name));
  } catch (const std::invalid_argument &) {
    rejected = true;
  }
  expect(rejected, message);
}

void test_invalid_calibrations()
{
  expect_invalid_config(
    "aimer_pitch_offset_equal_distance.yaml", "equal calibration distances must be rejected");
  expect_invalid_config(
    "aimer_pitch_offset_reversed_distance.yaml",
    "reversed calibration distances must be rejected");
  expect_invalid_config(
    "aimer_pitch_offset_negative_distance.yaml", "negative near distance must be rejected");
  expect_invalid_config(
    "aimer_pitch_offset_nonfinite_distance.yaml", "non-finite distance must be rejected");
  expect_invalid_config(
    "aimer_pitch_offset_nonfinite_offset.yaml", "non-finite offset must be rejected");
}
}  // namespace

int main()
{
  test_two_point_mapping();
  test_legacy_scalar_offset();
  test_invalid_calibrations();

  if (failures != 0) {
    std::cerr << failures << " aimer pitch offset checks failed\n";
    return 1;
  }
  std::cout << "Aimer pitch offset tests passed\n";
  return 0;
}
