#pragma once

#include <algorithm>
#include <array>
#include <cmath>

#include <ros/time.h>
#include <sensor_msgs/Range.h>

namespace tof_velocity_guard {

enum Direction { kFront, kRear, kLeft, kRight };
enum class SpeedLimitProfile { Linear, Smoothstep };
constexpr std::array<const char*, 4> kRangeTopics{{
    "/tof/front", "/tof/rear", "/tof/left", "/tof/right"}};

inline double clamp(double value, double lower, double upper) {
  return std::max(lower, std::min(upper, value));
}

inline bool isFresh(const ros::Time& stamp, const ros::Time& now, double timeout) {
  return !stamp.isZero() && stamp <= now && (now - stamp).toSec() <= timeout;
}

inline bool isValidRange(const sensor_msgs::Range::ConstPtr& measurement, const ros::Time& now,
                         double timeout) {
  return measurement && isFresh(measurement->header.stamp, now, timeout) &&
         std::isfinite(measurement->range) &&
         measurement->min_range <= measurement->range && measurement->range <= measurement->max_range;
}

inline double allowedSpeed(const sensor_msgs::Range::ConstPtr& measurement, const ros::Time& now,
                           double timeout, double stop, double slow, double max_speed,
                           SpeedLimitProfile profile = SpeedLimitProfile::Linear) {
  if (!isValidRange(measurement, now, timeout)) {
    return 0.0;
  }
  const double clearance = clamp((measurement->range - stop) / (slow - stop), 0.0, 1.0);
  if (profile == SpeedLimitProfile::Smoothstep) {
    // Zero slope at the stop and full-speed thresholds.
    return max_speed * clearance * clearance * (3.0 - 2.0 * clearance);
  }
  return max_speed * clearance;
}

inline double brakingSpeed(double distance, double deceleration, double delay) {
  if (distance <= 0.0) {
    return 0.0;
  }
  // Positive root of distance = speed * delay + speed² / (2 * deceleration).
  const double delay_speed = deceleration * delay;
  return 2.0 * deceleration * distance /
         (std::hypot(delay_speed, std::sqrt(2.0 * deceleration * distance)) + delay_speed);
}

inline double limitAcceleration(double requested, double measured, double negative_limit,
                                double positive_limit, double gain) {
  const double lower = gain * (-negative_limit - measured);
  const double upper = gain * (positive_limit - measured);
  return clamp(requested, lower, upper);
}

}  // namespace tof_velocity_guard
