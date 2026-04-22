#pragma once

#include <functional>
#include <memory>
#include <string>

#include "op3_vision_msgs/msg/detection.hpp"
#include "rclcpp/rclcpp.hpp"

namespace stride::op3
{

enum class TrackingState{Found, Waiting, Lost};

struct BallTrackerConfig
{
  std::string input_topic{"/perception/ball_cam"};
  std::string visibility_topic{"/perception/ball/visible"};
  std::string scan_command_topic{"/robotis/head_control/scan_command"};
  std::string scan_command{"scan"};
  std::function<void(double, double)> head_command_cb;
  std::function<void(bool)> visibility_cb;
  std::function<bool(const std::string &)> scan_request_cb;
  std::function<bool()> scan_in_progress_cb;
  std::function<void(TrackingState)> state_change_cb;
  double p_gain{0.45};
  double i_gain{0.0};
  double d_gain{0.045};
  double integral_limit_rad{0.0};
  double min_command_rad{0.0};
  double deadband_rad{0.0};
  double max_pan_rad{0.0};
  double max_tilt_rad{0.0};
  double lost_timeout{0.5};
  double scan_interval{2.0};
  bool use_scan{true};
  double control_rate_hz{30.0};
  int wait_cycles{5};
  int pre_scan_cycles{45};

};

class BallTracker
{
public:
  BallTracker(rclcpp::Node *node, const BallTrackerConfig &config);

  void enable(bool enable);
  bool isEnabled() const;

private:
  void onDetection(const op3_vision_msgs::msg::Detection::SharedPtr msg);
  void controlLoop();
  void resetIntegrators();
  double computeDt(const rclcpp::Time &now);
  void setVisibility(bool visible);
  void transitionState(TrackingState new_state, const char *reason = nullptr);
  static const char *stateToString(TrackingState state);


  rclcpp::Node *node_;
  BallTrackerConfig config_;
  rclcpp::Subscription<op3_vision_msgs::msg::Detection>::SharedPtr detection_sub_;

  bool enabled_{false};
  bool have_detection_{false};
  double last_pan_error_{0.0};
  double last_tilt_error_{0.0};
  op3_vision_msgs::msg::Detection last_detection_;
  rclcpp::Time last_detection_time_{0, 0, RCL_ROS_TIME};
  bool last_visibility_state_{false};

  TrackingState state_{TrackingState::Lost};
  int missed_count_{0};
  bool scan_sent_{false};
  bool scan_unavailable_warned_{false};

  double pan_integral_{0.0};
  double tilt_integral_{0.0};
  double prev_pan_error_{0.0};
  double prev_tilt_error_{0.0};
  rclcpp::Time last_control_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_scan_command_time_{0, 0, RCL_ROS_TIME};
  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace stride::op3
