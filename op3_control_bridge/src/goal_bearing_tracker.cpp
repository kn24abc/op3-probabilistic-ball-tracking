#include "op3_control_bridge/goal_bearing_tracker.hpp"

namespace stride::op3
{

GoalBearingTracker::GoalBearingTracker(const rclcpp::Clock::SharedPtr &clock,
                                       GoalBearingTrackerConfig config)
  : clock_(clock),
    config_(config)
{
  if (!clock_)
  {
    throw std::runtime_error("GoalBearingTracker requires a valid clock");
  }
  last_absolute_heading_ = expectedHeadingRad();
}

void GoalBearingTracker::updateDetection(
    const op3_vision_msgs::msg::Detection &detection,
    const rclcpp::Time &stamp)
{
  last_detection_time_ = stamp;
  have_detection_ = true;

  const double bearing = std::clamp(static_cast<double>(detection.bearing.x),
                                    -config_.horizontal_fov_rad,
                                    config_.horizontal_fov_rad);
  last_relative_bearing_ = bearing;

  if (have_imu_)
  {
    last_absolute_heading_ = wrapPi(last_imu_yaw_ + last_relative_bearing_);
  }
  else
  {
    last_absolute_heading_ = wrapPi(expectedHeadingRad() + last_relative_bearing_);
  }
}

void GoalBearingTracker::updateImuYaw(double yaw_rad, const rclcpp::Time &stamp)
{
  last_imu_time_ = stamp;
  last_imu_yaw_ = wrapPi(yaw_rad);
  have_imu_ = true;

  if (have_detection_)
  {
    last_absolute_heading_ = wrapPi(last_imu_yaw_ + last_relative_bearing_);
  }
}

bool GoalBearingTracker::hasFreshDetection(const rclcpp::Time &now,
                                           double max_age_sec) const
{
  if (!have_detection_)
    return false;
  const double age = (now - last_detection_time_).seconds();
  return age <= max_age_sec;
}

double GoalBearingTracker::relativeBearingRad() const
{
  return last_relative_bearing_;
}

double GoalBearingTracker::absoluteHeadingRad() const
{
  if (have_detection_)
    return last_absolute_heading_;
  return expectedHeadingRad();
}

double GoalBearingTracker::expectedHeadingRad() const
{
  return config_.attack_positive_x ? 0.0 : M_PI;
}

double GoalBearingTracker::timeSinceDetection(const rclcpp::Time &now) const
{
  if (!have_detection_)
    return std::numeric_limits<double>::infinity();
  return (now - last_detection_time_).seconds();
}

double GoalBearingTracker::wrapPi(double value)
{
  while (value > M_PI)
    value -= 2.0 * M_PI;
  while (value < -M_PI)
    value += 2.0 * M_PI;
  return value;
}

}  // namespace stride::op3
