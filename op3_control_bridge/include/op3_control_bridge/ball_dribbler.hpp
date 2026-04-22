#pragma once

#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <string>

#include "op3_vision_msgs/msg/detection.hpp"
#include "rclcpp/rclcpp.hpp"

namespace stride::op3
{

struct BallDribblerConfig
{
  // Primary body-frame detection topic; defaults to legacy input_topic for compatibility.
  std::string input_topic{"/perception/ball"};
  std::string body_input_topic{"/perception/ball"};
  std::string camera_input_topic{"/perception/ball_cam"};
  // Blend between camera yaw and body yaw when computing heading; 0=camera only, 1=body only.
  double body_yaw_weight{0.5};
  double forward_step{0.02};
  double min_forward_step{0.008};
  double max_turn_rad{10.0 * M_PI / 180.0};
  double stop_timeout_sec{0.75};
  double lock_bbox_min_area{1500.0};
  double close_bbox_area{3200.0};
  double lock_max_abs_bearing{0.12};
  std::function<void(double fb_move, double rl_turn)> walking_param_cb;
  std::function<void(bool start)> walking_command_cb;
  std::function<void()> lost_ball_cb;
  std::function<std::optional<double>()> goal_bearing_cb;
  double goal_alignment_weight{1.0};
  double goal_alignment_deadband_rad{5.0 * M_PI / 180.0};
  std::function<void()> goal_alignment_hint_cb;
};

class BallDribbler
{
public:
  BallDribbler(rclcpp::Node *node, const BallDribblerConfig &config);

  void enable(bool enable);
  bool isEnabled() const;
  bool hasBallLocked() const;

private:
  void onBodyDetection(const op3_vision_msgs::msg::Detection::SharedPtr msg);
  void onCameraDetection(const op3_vision_msgs::msg::Detection::SharedPtr msg);
  void processDetections(const rclcpp::Time &now);
  bool detectionFresh(const rclcpp::Time &stamp, const rclcpp::Time &now) const;
  void timeoutCheck();
  void stopWalking();
  double computeForwardStep(double bbox_area) const;

  rclcpp::Node *node_{nullptr};
  BallDribblerConfig config_;
  rclcpp::Subscription<op3_vision_msgs::msg::Detection>::SharedPtr detection_body_sub_;
  rclcpp::Subscription<op3_vision_msgs::msg::Detection>::SharedPtr detection_camera_sub_;
  rclcpp::TimerBase::SharedPtr timeout_timer_;

  bool enabled_{false};
  bool walking_active_{false};
  bool ball_locked_{false};
  rclcpp::Time last_detection_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_body_detection_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_camera_detection_time_{0, 0, RCL_ROS_TIME};
  double last_body_yaw_{0.0};
  double last_camera_yaw_{0.0};
  double last_body_bbox_area_{0.0};
  double last_camera_bbox_area_{0.0};
};

}  // namespace stride::op3
