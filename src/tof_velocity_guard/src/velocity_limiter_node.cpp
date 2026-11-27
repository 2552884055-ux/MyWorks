#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

#include <geometry_msgs/AccelStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tof_velocity_guard/directional_limits.h>

#ifdef TOF_HAS_POSITION_COMMAND
#include <quadrotor_msgs/PositionCommand.h>
#endif

namespace {

using namespace tof_velocity_guard;

bool isFinite(const tf2::Vector3& vector) {
  return std::isfinite(vector.x()) && std::isfinite(vector.y()) && std::isfinite(vector.z());
}

void limitHorizontal(geometry_msgs::Vector3& vector, double limit) {
  const double magnitude = std::hypot(vector.x, vector.y);
  if (magnitude > limit) {
    vector.x *= limit / magnitude;
    vector.y *= limit / magnitude;
  }
}

class VelocityLimiter {
 public:
  VelocityLimiter(ros::NodeHandle& nh, ros::NodeHandle& params)
      : body_frame_(params.param<std::string>("body_frame", "body")),
        stop_distance_(params.param("stop_distance", 0.35)),
        slow_distance_(params.param("slow_distance", 0.9)),
        max_horizontal_(params.param("max_horizontal_speed", 0.3)),
        max_vertical_(params.param("max_vertical_speed", 0.0)),
        range_timeout_(params.param("range_timeout", 0.2)),
        command_timeout_(params.param("command_timeout", 0.2)),
        tf_timeout_(params.param("tf_timeout", 0.15)),
        odometry_topic_(params.param<std::string>("odometry_topic", "")),
        odometry_timeout_(params.param("odometry_timeout", 0.2)),
        position_gain_(params.param("position_gain", 0.8)),
        velocity_gain_(params.param("velocity_gain", 2.0)),
        max_acceleration_(params.param("max_horizontal_acceleration", 0.6)),
        tf_listener_(tf_buffer_) {
    const std::array<double, 11> numeric{{stop_distance_, slow_distance_, max_horizontal_,
        max_vertical_, range_timeout_, command_timeout_, tf_timeout_, odometry_timeout_,
        position_gain_, velocity_gain_, max_acceleration_}};
    if (!std::all_of(numeric.begin(), numeric.end(), [](double value) { return std::isfinite(value); }) ||
        stop_distance_ <= 0.0 || slow_distance_ <= stop_distance_ ||
        max_horizontal_ < 0.0 || max_vertical_ < 0.0 ||
        range_timeout_ <= 0.0 || command_timeout_ <= 0.0 || tf_timeout_ <= 0.0 ||
        odometry_timeout_ <= 0.0 || position_gain_ < 0.0 || velocity_gain_ <= 0.0 ||
        max_acceleration_ < 0.0 ||
        body_frame_.empty() || body_frame_.front() == '/') {
      throw std::invalid_argument("Invalid velocity limiter distances, speeds, timeouts or body_frame");
    }

    const auto input = params.param<std::string>("input_topic", "/position_cmd");
    const auto output = params.param<std::string>("output_topic", "/tof/limited_velocity");
    const auto input_type = params.param<std::string>("input_type", "position_command");
    if (nh.resolveName(input) == nh.resolveName(output)) {
      throw std::invalid_argument("Input and output topics must differ");
    }
    if (input_type == "position_command") {
#ifdef TOF_HAS_POSITION_COMMAND
      command_sub_ = nh.subscribe(input, 1, &VelocityLimiter::onPositionCommand, this);
#else
      throw std::invalid_argument("PositionCommand requires sourcing the EGO-Planner workspace "
                                  "and rebuilding tof_velocity_guard with quadrotor_msgs");
#endif
    } else if (input_type == "twist_stamped") {
      command_sub_ = nh.subscribe(input, 1, &VelocityLimiter::onTwist, this);
    } else {
      throw std::invalid_argument("input_type must be position_command or twist_stamped");
    }

    publisher_ = nh.advertise<geometry_msgs::TwistStamped>(output, 1);
    speed_limits_pub_ = params.advertise<std_msgs::Float64MultiArray>("directional_speed_limits", 1);
    range_speed_pub_ = params.advertise<std_msgs::Float64MultiArray>("directional_range_speed", 1);
    if (!odometry_topic_.empty()) {
      const auto acceleration_topic =
          params.param<std::string>("acceleration_topic", "/tof/limited_acceleration");
      if (input_type != "position_command" || nh.resolveName(acceleration_topic) == nh.resolveName(output) ||
          nh.resolveName(acceleration_topic) == nh.resolveName(input)) {
        throw std::invalid_argument("Odometry tracking needs PositionCommand and a distinct acceleration topic");
      }
      odometry_sub_ = nh.subscribe<nav_msgs::Odometry>(
          odometry_topic_, 1, [this](const nav_msgs::Odometry::ConstPtr& msg) { odometry_ = msg; });
      acceleration_pub_ = nh.advertise<geometry_msgs::AccelStamped>(acceleration_topic, 1);
    }
    for (std::size_t i = 0; i < kRangeTopics.size(); ++i) {
      range_subs_[i] = nh.subscribe<sensor_msgs::Range>(
          kRangeTopics[i], 1, [this, i](const sensor_msgs::Range::ConstPtr& msg) { ranges_[i] = msg; });
    }
  }

  void publish() {
    geometry_msgs::TwistStamped output;
    output.header.stamp = ros::Time::now();
    output.header.frame_id = body_frame_;
    geometry_msgs::AccelStamped acceleration;
    acceleration.header = output.header;
    std::array<double, kRangeTopics.size()> speed_limits;
    for (std::size_t i = 0; i < speed_limits.size(); ++i) {
      speed_limits[i] = towardLimit(ranges_[i], output.header.stamp);
    }

    tf2::Quaternion rotation;
    tf2::Vector3 velocity;
    tf2::fromMsg(command_.vector, velocity);
    if (command_ready_ && isFresh(command_.header.stamp, output.header.stamp, command_timeout_) &&
        isFinite(velocity) && bodyRotation(output.header.stamp, rotation)) {
      velocity = tf2::quatRotate(rotation, velocity);
      tf2::Vector3 measured(0, 0, 0);
      tf2::Vector3 feedforward(0, 0, 0);
      if (isFinite(velocity) &&
          (odometry_topic_.empty() || trackingState(output.header.stamp, rotation, velocity, measured, feedforward))) {
        const double front = speed_limits[kFront], rear = speed_limits[kRear];
        const double left = speed_limits[kLeft], right = speed_limits[kRight];
        auto& linear = output.twist.linear;
        linear.x = clamp(velocity.x(), -rear, front);
        linear.y = clamp(velocity.y(), -right, left);
        linear.z = clamp(velocity.z(), -max_vertical_, max_vertical_);
        limitHorizontal(linear, max_horizontal_);
        if (acceleration_pub_) {
          acceleration.accel.linear.x = limitedAcceleration(linear.x, measured.x(), feedforward.x(), rear, front);
          acceleration.accel.linear.y = limitedAcceleration(linear.y, measured.y(), feedforward.y(), right, left);
          limitHorizontal(acceleration.accel.linear, max_acceleration_);
        }
      }
    }
    publisher_.publish(output);
    if (acceleration_pub_) {
      acceleration_pub_.publish(acceleration);
    }
    if (speed_limits_pub_.getNumSubscribers() > 0) {
      std_msgs::Float64MultiArray message;
      message.data.assign(speed_limits.begin(), speed_limits.end());
      speed_limits_pub_.publish(message);
    }
    if (range_speed_pub_.getNumSubscribers() > 0) {
      std_msgs::Float64MultiArray message;
      message.data.reserve(2 * speed_limits.size());
      for (std::size_t i = 0; i < speed_limits.size(); ++i) {
        const auto& measurement = ranges_[i];
        const double distance = isValidRange(measurement, output.header.stamp, range_timeout_)
                                    ? measurement->range : std::numeric_limits<double>::quiet_NaN();
        message.data.push_back(distance);
        message.data.push_back(speed_limits[i]);
      }
      range_speed_pub_.publish(message);
    }
  }

 private:
  void onTwist(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    command_.header = msg->header;
    command_.vector = msg->twist.linear;
    command_ready_ = true;
  }

#ifdef TOF_HAS_POSITION_COMMAND
  void onPositionCommand(const quadrotor_msgs::PositionCommand::ConstPtr& msg) {
    command_.header = msg->header;
    command_.vector = msg->velocity;
    target_position_.setValue(msg->position.x, msg->position.y, msg->position.z);
    tf2::fromMsg(msg->acceleration, feedforward_acceleration_);
    command_ready_ = msg->trajectory_flag == quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
  }
#endif

  double towardLimit(const sensor_msgs::Range::ConstPtr& measurement, const ros::Time& now) const {
    return allowedSpeed(measurement, now, range_timeout_, stop_distance_, slow_distance_, max_horizontal_);
  }

  bool trackingState(const ros::Time& now, const tf2::Quaternion& rotation, tf2::Vector3& velocity,
                     tf2::Vector3& measured, tf2::Vector3& feedforward) const {
    if (!odometry_ || !isFresh(odometry_->header.stamp, now, odometry_timeout_) ||
        odometry_->header.frame_id != command_.header.frame_id) {
      return false;
    }
    const auto& position = odometry_->pose.pose.position;
    const tf2::Vector3 error = target_position_ - tf2::Vector3(position.x, position.y, position.z);
    // FAST-LIO publishes its linear velocity in the odometry parent frame.
    tf2::fromMsg(odometry_->twist.twist.linear, measured);
    const tf2::Vector3 body_error = tf2::quatRotate(rotation, error);
    measured = tf2::quatRotate(rotation, measured);
    feedforward = tf2::quatRotate(rotation, feedforward_acceleration_);
    velocity.setX(velocity.x() + position_gain_ * body_error.x());
    velocity.setY(velocity.y() + position_gain_ * body_error.y());
    return isFinite(velocity) && isFinite(measured) && isFinite(feedforward);
  }

  double limitedAcceleration(double target, double measured, double feedforward,
                             double negative_limit, double positive_limit) const {
    const double feedback = velocity_gain_ * (target - measured);
    // If the measured velocity is already outside the interval, allow braking toward it.
    return limitAcceleration(feedback + feedforward, measured, negative_limit, positive_limit, velocity_gain_);
  }

  bool bodyRotation(const ros::Time& now, tf2::Quaternion& rotation) const {
    const std::string& frame = command_.header.frame_id;
    if (frame.empty()) {
      ROS_WARN_THROTTLE(5.0, "Velocity command does not specify a frame_id");
      return false;
    }
    if (frame == body_frame_) {
      rotation.setValue(0, 0, 0, 1);
      return true;
    }
    try {
      // Planner commands can arrive faster than TF. Use current attitude within the age limit.
      const auto transform = tf_buffer_.lookupTransform(body_frame_, frame, ros::Time(0));
      if (!transform.header.stamp.isZero() && !isFresh(transform.header.stamp, now, tf_timeout_)) {
        ROS_WARN_THROTTLE(5.0, "Velocity transform into %s is stale", body_frame_.c_str());
        return false;
      }
      tf2::fromMsg(transform.transform.rotation, rotation);
      return true;
    } catch (const tf2::TransformException& error) {
      ROS_WARN_THROTTLE(5.0, "Cannot transform velocity into %s: %s", body_frame_.c_str(), error.what());
      return false;
    }
  }

  std::string body_frame_;
  double stop_distance_, slow_distance_, max_horizontal_, max_vertical_;
  double range_timeout_, command_timeout_, tf_timeout_;
  std::string odometry_topic_;
  double odometry_timeout_, position_gain_, velocity_gain_, max_acceleration_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  ros::Publisher publisher_, acceleration_pub_, speed_limits_pub_, range_speed_pub_;
  ros::Subscriber command_sub_, odometry_sub_;
  std::array<ros::Subscriber, 4> range_subs_;
  std::array<sensor_msgs::Range::ConstPtr, 4> ranges_;
  geometry_msgs::Vector3Stamped command_;
  nav_msgs::Odometry::ConstPtr odometry_;
  tf2::Vector3 target_position_{0, 0, 0};
  tf2::Vector3 feedforward_acceleration_{0, 0, 0};
  bool command_ready_ = false;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "tof_velocity_limiter");
  ros::NodeHandle nh;
  ros::NodeHandle params("~");
  try {
    VelocityLimiter limiter(nh, params);
    ros::Rate rate(30);
    while (ros::ok()) {
      ros::spinOnce();
      limiter.publish();
      rate.sleep();
    }
  } catch (const std::exception& error) {
    ROS_FATAL("%s", error.what());
    return 1;
  }
  return 0;
}
