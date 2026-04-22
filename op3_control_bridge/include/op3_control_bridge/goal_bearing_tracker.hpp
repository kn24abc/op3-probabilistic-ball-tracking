#pragma once

#include <cmath>
#include <limits>
#include <memory>

#include "op3_vision_msgs/msg/detection.hpp"
#include "rclcpp/rclcpp.hpp"

namespace stride::op3
{

struct GoalBearingTrackerConfig
{
  double horizontal_fov_rad{35.2 * M_PI / 180.0};
  double detection_timeout_sec{1.5};
  double field_length_m{14.0};
  double field_width_m{9.0};
  double goal_width_m{6.0};
  bool attack_positive_x{true};
};

class GoalBearingTracker
{
public:
  GoalBearingTracker(const rclcpp::Clock::SharedPtr &clock,
                     GoalBearingTrackerConfig config);

  void updateDetection(const op3_vision_msgs::msg::Detection &detection,
                       const rclcpp::Time &stamp);

  void updateImuYaw(double yaw_rad, const rclcpp::Time &stamp);

  bool hasFreshDetection(const rclcpp::Time &now,
                         double max_age_sec) const;

  double relativeBearingRad() const;
  double absoluteHeadingRad() const;
  double expectedHeadingRad() const;
  double timeSinceDetection(const rclcpp::Time &now) const;

private:
  static double wrapPi(double value);

  rclcpp::Clock::SharedPtr clock_;
  GoalBearingTrackerConfig config_;
  double last_relative_bearing_{0.0};
  double last_absolute_heading_{0.0};
  double last_imu_yaw_{0.0};
  rclcpp::Time last_detection_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_imu_time_{0, 0, RCL_ROS_TIME};
  bool have_detection_{false};
  bool have_imu_{false};
};

}  // namespace stride::op3
