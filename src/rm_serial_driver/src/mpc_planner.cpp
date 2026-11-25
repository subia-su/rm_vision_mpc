#include "rm_serial_driver/mpc_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace rm_serial_driver
{
namespace
{
constexpr unsigned kMaxIterations = 60;
constexpr unsigned kMaxLineSearchIterations = 12;
constexpr double kGradientTolerance = 1.0e-6;

double clamp(double value, double lower, double upper)
{
  return std::max(lower, std::min(upper, value));
}

double wrap(double angle)
{
  return std::remainder(angle, 2.0 * std::acos(-1.0));
}

bool finite_non_negative(double value)
{
  return std::isfinite(value) && value >= 0.0;
}
}  // namespace

MpcPlanner::MpcPlanner(std::size_t horizon, double dt, MpcLimits limits, MpcWeights weights)
: horizon_(horizon), dt_(dt), limits_(limits), weights_(weights)
{
  if (horizon < 2 || !std::isfinite(dt) || dt <= 0.0 ||
    !std::isfinite(limits.yaw_speed) || limits.yaw_speed <= 0.0 ||
    !std::isfinite(limits.pitch_speed) || limits.pitch_speed <= 0.0 ||
    !std::isfinite(limits.yaw_acc) || limits.yaw_acc <= 0.0 ||
    !std::isfinite(limits.pitch_acc) || limits.pitch_acc <= 0.0 ||
    !finite_non_negative(weights.angle) || !finite_non_negative(weights.speed) ||
    !finite_non_negative(weights.acceleration) ||
    !finite_non_negative(weights.acceleration_delta))
  {
    throw std::invalid_argument("invalid MPC configuration");
  }
}

void MpcPlanner::reset()
{
  // The planner currently has no warm-start state.
}

MpcPlanner::AxisState MpcPlanner::solve_axis(
  double angle, double speed, double speed_limit, double acceleration_limit,
  const std::vector<double> & angle_reference,
  const std::vector<double> & speed_reference)
{
  if (!std::isfinite(angle) || !std::isfinite(speed)) {
    throw std::invalid_argument("non-finite MPC state");
  }

  const auto project = [&](std::vector<double> & acceleration) {
      double projected_speed = clamp(speed, -speed_limit, speed_limit);
      for (std::size_t k = 0; k < horizon_; ++k) {
        const double velocity_lower = (-speed_limit - projected_speed) / dt_;
        const double velocity_upper = (speed_limit - projected_speed) / dt_;
        acceleration[k] = clamp(
          acceleration[k], std::max(-acceleration_limit, velocity_lower),
          std::min(acceleration_limit, velocity_upper));
        projected_speed += acceleration[k] * dt_;
      }
    };

  std::vector<double> acceleration(horizon_, 0.0);
  std::vector<double> candidate(horizon_, 0.0);
  std::vector<double> gradient(horizon_, 0.0);
  std::vector<double> predicted_angle(horizon_, 0.0);
  std::vector<double> predicted_speed(horizon_, 0.0);

  const auto evaluate = [&](const std::vector<double> & input, bool calculate_gradient) {
      double predicted_a = angle;
      double predicted_v = clamp(speed, -speed_limit, speed_limit);
      double cost = 0.0;
      for (std::size_t k = 0; k < horizon_; ++k) {
        predicted_a += predicted_v * dt_ + 0.5 * input[k] * dt_ * dt_;
        predicted_v += input[k] * dt_;
        predicted_angle[k] = predicted_a;
        predicted_speed[k] = predicted_v;
        const double angle_error = wrap(predicted_a - angle_reference[k]);
        const double speed_error = predicted_v - speed_reference[k];
        const double delta = input[k] - (k == 0 ? 0.0 : input[k - 1]);
        cost += weights_.angle * angle_error * angle_error +
          weights_.speed * speed_error * speed_error +
          weights_.acceleration * input[k] * input[k] +
          weights_.acceleration_delta * delta * delta;
      }
      if (!calculate_gradient) {
        return cost;
      }

      std::fill(gradient.begin(), gradient.end(), 0.0);
      for (std::size_t j = 0; j < horizon_; ++j) {
        double derivative = 2.0 * weights_.acceleration * input[j];
        const double previous = j == 0 ? 0.0 : input[j - 1];
        derivative += 2.0 * weights_.acceleration_delta * (input[j] - previous);
        if (j + 1 < horizon_) {
          derivative -= 2.0 * weights_.acceleration_delta * (input[j + 1] - input[j]);
        }
        for (std::size_t k = j; k < horizon_; ++k) {
          const double angle_sensitivity =
            dt_ * dt_ * (static_cast<double>(k - j) + 0.5);
          derivative += 2.0 * weights_.angle *
            wrap(predicted_angle[k] - angle_reference[k]) * angle_sensitivity;
          derivative += 2.0 * weights_.speed *
            (predicted_speed[k] - speed_reference[k]) * dt_;
        }
        gradient[j] = derivative;
      }
      return cost;
    };

  project(acceleration);
  double cost = evaluate(acceleration, true);
  if (!std::isfinite(cost)) {
    throw std::domain_error("non-finite MPC cost");
  }

  for (unsigned iteration = 0; iteration < kMaxIterations; ++iteration) {
    double gradient_norm_squared = 0.0;
    for (double value : gradient) {
      gradient_norm_squared += value * value;
    }
    if (gradient_norm_squared <= kGradientTolerance * kGradientTolerance) {
      break;
    }

    double step = 0.25;
    double candidate_cost = std::numeric_limits<double>::infinity();
    bool accepted = false;
    for (unsigned search = 0; search < kMaxLineSearchIterations; ++search) {
      for (std::size_t k = 0; k < horizon_; ++k) {
        candidate[k] = acceleration[k] - step * gradient[k];
      }
      project(candidate);
      candidate_cost = evaluate(candidate, false);
      if (std::isfinite(candidate_cost) && candidate_cost < cost) {
        accepted = true;
        break;
      }
      step *= 0.5;
    }
    if (!accepted) {
      break;
    }
    acceleration.swap(candidate);
    cost = evaluate(acceleration, true);
  }

  AxisState output{angle, clamp(speed, -speed_limit, speed_limit), 0.0};
  const std::size_t output_steps = horizon_ / 2;
  for (std::size_t k = 0; k < output_steps; ++k) {
    output.angle += output.speed * dt_ + 0.5 * acceleration[k] * dt_ * dt_;
    output.speed += acceleration[k] * dt_;
    output.acceleration = acceleration[k];
  }
  return output;
}

PlannedState MpcPlanner::solve(const std::vector<AimReference> & reference)
{
  if (reference.size() < horizon_) {
    throw std::invalid_argument("short MPC reference");
  }
  std::vector<double> yaw(horizon_), pitch(horizon_), yaw_speed(horizon_), pitch_speed(horizon_);
  double unwrapped_yaw = reference.front().yaw;
  for (std::size_t k = 0; k < horizon_; ++k) {
    if (!std::isfinite(reference[k].yaw) || !std::isfinite(reference[k].pitch)) {
      throw std::invalid_argument("non-finite MPC reference");
    }
    unwrapped_yaw += wrap(reference[k].yaw - unwrapped_yaw);
    yaw[k] = unwrapped_yaw;
    pitch[k] = reference[k].pitch;
  }
  for (std::size_t k = 0; k + 1 < horizon_; ++k) {
    yaw_speed[k] = clamp((yaw[k + 1] - yaw[k]) / dt_, -limits_.yaw_speed, limits_.yaw_speed);
    pitch_speed[k] = clamp(
      (pitch[k + 1] - pitch[k]) / dt_, -limits_.pitch_speed, limits_.pitch_speed);
  }
  yaw_speed.back() = yaw_speed[horizon_ - 2];
  pitch_speed.back() = pitch_speed[horizon_ - 2];

  const auto planned_yaw = solve_axis(
    yaw.front(), yaw_speed.front(), limits_.yaw_speed, limits_.yaw_acc, yaw, yaw_speed);
  const auto planned_pitch = solve_axis(
    pitch.front(), pitch_speed.front(), limits_.pitch_speed, limits_.pitch_acc,
    pitch, pitch_speed);
  PlannedState output;
  output.yaw = wrap(planned_yaw.angle);
  output.yaw_speed = planned_yaw.speed;
  output.yaw_acc = planned_yaw.acceleration;
  output.pitch = planned_pitch.angle;
  output.pitch_speed = planned_pitch.speed;
  output.pitch_acc = planned_pitch.acceleration;
  return output;
}
}  // namespace rm_serial_driver
