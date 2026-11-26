#pragma once

#include <array>
#include <cmath>
#include <stdexcept>

#include <Eigen/Geometry>
#include <ros/ros.h>
#include <tof_velocity_guard/directional_limits.h>

class TofGuard
{
public:
  explicit TofGuard(ros::NodeHandle &nh)
      : enabled_(nh.param("tof/enabled", false)),
        stop_(nh.param("tof/stop_distance", 0.35)),
        slow_(nh.param("tof/slow_distance", 0.9)),
        speed_(nh.param("tof/max_horizontal_speed", 0.3)),
        acceleration_(nh.param("tof/max_horizontal_acceleration", 0.6)),
        timeout_(nh.param("tof/range_timeout", 0.2)),
        braking_gain_(nh.param("tof/braking_gain", 2.0)),
        braking_deceleration_(nh.param("tof/braking_deceleration", 0.3)),
        braking_delay_(nh.param("tof/braking_delay", 0.3))
  {
    if (!enabled_) return;
    const std::array<double, 8> config{{stop_, slow_, speed_, acceleration_, timeout_, braking_gain_,
                                       braking_deceleration_, braking_delay_}};
    if (!std::all_of(config.begin(), config.end(), [](double value) { return std::isfinite(value); }) ||
        stop_ <= 0.0 || slow_ <= stop_ || speed_ <= 0.0 || acceleration_ <= 0.0 ||
        timeout_ <= 0.0 || braking_gain_ <= 0.0 || braking_deceleration_ <= 0.0 ||
        braking_delay_ < 0.0) {
      throw std::invalid_argument("Invalid px4ctrl ToF limits");
    }
    // Keep the modeled per-axis deceleration achievable when both axes need braking.
    braking_deceleration_ = std::min(braking_deceleration_, acceleration_ / std::sqrt(2.0));
    for (std::size_t i = 0; i < subscribers_.size(); ++i) {
      subscribers_[i] = nh.subscribe<sensor_msgs::Range>(
          tof_velocity_guard::kRangeTopics[i], 1,
          [this, i](const sensor_msgs::Range::ConstPtr &msg) { ranges_[i] = msg; });
    }
  }

  bool enabled() const { return enabled_; }

  Eigen::Vector2d limit(double yaw, const Eigen::Vector2d &measured,
                        const Eigen::Vector2d &feedforward_accel,
                        const Eigen::Vector2d &velocity_gain, Eigen::Vector2d &target_velocity) const
  {
    using namespace tof_velocity_guard;
    const ros::Time now = ros::Time::now();

    // Use heading-aligned horizontal axes so the ToF limits don't alter the vertical loop.
    const Eigen::Rotation2Dd to_body(-yaw);
    const Eigen::Vector2d body_velocity = to_body * measured;
    Eigen::Vector2d body_target = to_body * target_velocity;
    Eigen::Vector2d lower, upper;
    Eigen::Vector2d emergency_brake = Eigen::Vector2d::Zero();
    const std::array<Direction, 2> positive{{kFront, kLeft}};
    const std::array<Direction, 2> negative{{kRear, kRight}};
    for (int axis = 0; axis < 2; ++axis) {
      const double brake_forward = capForDirection(positive[axis], body_velocity(axis), now, upper(axis));
      const double brake_backward = capForDirection(negative[axis], -body_velocity(axis), now, lower(axis));
      emergency_brake(axis) = brake_backward - brake_forward;
      body_target(axis) = clamp(body_target(axis), -lower(axis), upper(axis));
    }
    const double target_speed = body_target.norm();
    if (target_speed > speed_)
      body_target *= speed_ / target_speed;
    target_velocity = to_body.inverse() * body_target;

    Eigen::Vector2d body_accel = emergency_brake;
    if (emergency_brake.squaredNorm() == 0.0) {
      body_accel = to_body * (feedforward_accel + velocity_gain.cwiseProduct(target_velocity - measured));
      for (int axis = 0; axis < 2; ++axis) {
        body_accel(axis) = limitAcceleration(body_accel(axis), body_velocity(axis),
                                             lower(axis), upper(axis), braking_gain_);
      }
    }
    const double accel_norm = body_accel.norm();
    if (accel_norm > acceleration_)
      body_accel *= acceleration_ / accel_norm;
    return to_body.inverse() * body_accel;
  }

private:
  double capForDirection(tof_velocity_guard::Direction direction, double approach_speed,
                         const ros::Time &now, double &cap) const
  {
    using namespace tof_velocity_guard;
    const auto &measurement = ranges_[direction];
    double safe_speed = 0.0;
    if (isValidRange(measurement, now, timeout_)) {
      const double available_distance = measurement->range - stop_;
      const double delay = braking_delay_ + (now - measurement->header.stamp).toSec();
      safe_speed = brakingSpeed(available_distance, braking_deceleration_, delay);
    }
    if (approach_speed > 0.0 && approach_speed >= safe_speed) {
      cap = 0.0;
      // Fade the brake near rest if clearance is exhausted or the sensor is invalid.
      return safe_speed == 0.0 ? std::min(acceleration_, braking_gain_ * approach_speed) : acceleration_;
    }
    cap = std::min(safe_speed, allowedSpeed(measurement, now, timeout_, stop_, slow_,
                                            speed_, SpeedLimitProfile::Smoothstep));
    return 0.0;
  }

  bool enabled_;
  double stop_, slow_, speed_, acceleration_, timeout_, braking_gain_;
  double braking_deceleration_, braking_delay_;
  std::array<ros::Subscriber, 4> subscribers_;
  std::array<sensor_msgs::Range::ConstPtr, 4> ranges_;
};
