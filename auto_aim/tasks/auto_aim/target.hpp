#ifndef AUTO_AIM__TARGET_HPP
#define AUTO_AIM__TARGET_HPP

#include <Eigen/Dense>
#include <array>
#include <chrono>
#include <optional>
#include <queue>
#include <string>
#include <vector>

#include "armor.hpp"
#include "tools/extended_kalman_filter.hpp"

namespace auto_aim
{

inline constexpr double OUTPOST_RADIUS = 0.275;
inline constexpr double OUTPOST_HEIGHT_STEP = 0.102;

class Target
{
public:
  ArmorName name;
  ArmorType armor_type;
  ArmorPriority priority;
  bool jumped;
  int last_id;  // debug only

  Target() = default;
  Target(
    const Armor & armor, std::chrono::steady_clock::time_point t, double radius, int armor_num,
    Eigen::VectorXd P0_dig);
  Target(double x, double vyaw, double radius, double h);

  void predict(std::chrono::steady_clock::time_point t);
  void predict(double dt);
  void update(const Armor & armor);

  Eigen::VectorXd ekf_x() const;
  const tools::ExtendedKalmanFilter & ekf() const;
  Eigen::Vector4d armor_xyza(int id) const;
  std::vector<Eigen::Vector4d> armor_xyza_list() const;
  bool outpost_layout_complete() const;

  bool diverged() const;

  bool convergened();

  bool isinit = false;

  bool checkinit();

private:
  int armor_num_;
  int switch_count_;
  int update_count_;

  bool is_switch_, is_converged_;

  tools::ExtendedKalmanFilter ekf_;
  std::chrono::steady_clock::time_point t_;

  std::array<bool, 3> outpost_observed_{{false, false, false}};
  std::array<double, 3> outpost_measured_heights_{{0, 0, 0}};
  std::array<int, 3> outpost_height_observation_counts_{{0, 0, 0}};
  int outpost_layout_index_{0};

  void update_ypda(const Armor & armor, int id);  // yaw pitch distance angle
  void update_outpost_layout(const Armor & armor, int id);
  double outpost_height_offset(int id) const;

  Eigen::Vector3d h_armor_xyz(const Eigen::VectorXd & x, int id) const;
  Eigen::MatrixXd h_jacobian(const Eigen::VectorXd & x, int id) const;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__TARGET_HPP
