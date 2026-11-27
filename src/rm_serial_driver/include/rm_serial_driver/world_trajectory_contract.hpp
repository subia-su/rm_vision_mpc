#ifndef RM_SERIAL_DRIVER__WORLD_TRAJECTORY_CONTRACT_HPP_
#define RM_SERIAL_DRIVER__WORLD_TRAJECTORY_CONTRACT_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include "rm_serial_driver/mpc_planner.hpp"
#include "rm_serial_driver/trajectory_packet.hpp"

namespace rm_serial_driver
{
struct WorldTrajectoryPolicy
{
  double pitch_min = -std::acos(-1.0) / 2.0;
  double pitch_max = std::acos(-1.0) / 2.0;
  double target_timeout_ms = 50.0;
  int ttl_ms = 40;
  int transport_age_ms = 2;
  MpcLimits limits;
};

inline void validate_world_policy(const WorldTrajectoryPolicy & p, const std::string & frame)
{
  const double half_pi = std::acos(-1.0) / 2.0;
  if (frame != "world" || p.ttl_ms != 40 || p.transport_age_ms < 0 ||
    p.transport_age_ms > 37 || !std::isfinite(p.pitch_min) || !std::isfinite(p.pitch_max) ||
    p.pitch_min < -half_pi || p.pitch_max > half_pi || p.pitch_min >= p.pitch_max ||
    !std::isfinite(p.target_timeout_ms) || p.target_timeout_ms <= 0.0 ||
    p.target_timeout_ms > 50.0 ||
    !std::isfinite(p.limits.yaw_speed) || p.limits.yaw_speed <= 0.0 || p.limits.yaw_speed > 5.0 ||
    !std::isfinite(p.limits.pitch_speed) || p.limits.pitch_speed <= 0.0 || p.limits.pitch_speed > 0.6 ||
    !std::isfinite(p.limits.yaw_acc) || p.limits.yaw_acc <= 0.0 || p.limits.yaw_acc > 20.0 ||
    !std::isfinite(p.limits.pitch_acc) || p.limits.pitch_acc <= 0.0 || p.limits.pitch_acc > 8.0)
  {
    throw std::invalid_argument("invalid world-Pitch/A6 contract (world frame, TTL=40ms required)");
  }
}

inline void revoke_trajectory(TrajectoryCommand & c)
{
  c.valid = false;
  c.yaw = c.pitch = c.yaw_speed = c.pitch_speed = c.yaw_acc = c.pitch_acc = 0.0F;
}

inline std::string target_age_error(double age_s, const WorldTrajectoryPolicy & p)
{
  if (!std::isfinite(age_s) || age_s < 0.0) {return "invalid_or_future_target_stamp";}
  if (age_s * 1000.0 > p.target_timeout_ms) {return "stale_target";}
  return {};
}

// No mechanical clipping here: the MCU converts world Pitch using real DM/IMU feedback.
// elapsed_ms is steady-clock time since this reference's host origin, not target age.
inline std::string finalize_world_trajectory(
  TrajectoryCommand & c, const WorldTrajectoryPolicy & p, double elapsed_ms)
{
  c.ttl_ms = static_cast<std::uint16_t>(p.ttl_ms);
  if (!std::isfinite(elapsed_ms) || elapsed_ms < 0.0) {
    revoke_trajectory(c); c.age_ms = 41; return "invalid_planning_clock";
  }
  const double age = std::ceil(elapsed_ms) + p.transport_age_ms;
  c.age_ms = static_cast<std::uint16_t>(std::min(
    age, static_cast<double>(std::numeric_limits<std::uint16_t>::max())));
  if (!c.valid) {revoke_trajectory(c); return {};}
  std::string error;
  const float values[] = {c.yaw, c.pitch, c.yaw_speed, c.pitch_speed, c.yaw_acc, c.pitch_acc};
  for (float value : values) {
    if (!std::isfinite(value)) {error = "nonfinite_plan"; break;}
  }
  if (error.empty() && (c.pitch < static_cast<float>(p.pitch_min) ||
    c.pitch > static_cast<float>(p.pitch_max))) {error = "world_pitch_out_of_domain";}
  if (error.empty() && std::abs(c.yaw) > static_cast<float>(std::acos(-1.0))) {
    error = "world_yaw_out_of_domain";
  }
  if (error.empty() && (std::abs(c.yaw_speed) > static_cast<float>(p.limits.yaw_speed) ||
    std::abs(c.pitch_speed) > static_cast<float>(p.limits.pitch_speed) ||
    std::abs(c.yaw_acc) > static_cast<float>(p.limits.yaw_acc) ||
    std::abs(c.pitch_acc) > static_cast<float>(p.limits.pitch_acc))) {error = "plan_derivative_limit";}
  // Reserve the MCU's documented 3ms CAN budget; do not extend the 40ms lifetime.
  if (error.empty() && age + 3.0 > p.ttl_ms) {error = "a6_age_budget_exhausted";}
  if (!error.empty()) {revoke_trajectory(c);}
  return error;
}
}  // namespace rm_serial_driver
#endif
