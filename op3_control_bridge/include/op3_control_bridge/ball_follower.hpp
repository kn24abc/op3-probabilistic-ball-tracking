#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "op3_vision_msgs/msg/detection.hpp"
#include "rclcpp/rclcpp.hpp"

namespace stride::op3
{

struct BallFollowerConfig
{
  std::string body_input_topic{"/perception/ball"};
  std::string camera_input_topic{"/perception/ball_cam"};
  // Blend between camera yaw and body yaw when computing heading; 0=camera only, 1=body only.
  double body_yaw_weight{0.5};
  double max_forward_step{0.04};
  double nominal_forward_step{0.02};
  double max_turn_rad{15.0 * M_PI / 180.0};
  double stop_timeout_sec{1.0};
  double ball_equipped_bbox_min_area{22000.0};
  double ball_equipped_max_abs_bearing{0.2};  // normalized bearing units
  double ball_equipped_hold_timeout_sec{0.75};
  std::function<std::optional<std::pair<double, double>>()> head_pose_cb;
  std::function<void(double fb_move, double rl_turn)> walking_param_cb;
  std::function<void(bool start)> walking_command_cb;
  std::function<void(bool equipped)> ball_equipped_cb;
};

class BallFollower
{
public:
  BallFollower(rclcpp::Node *node, const BallFollowerConfig &config);

  void enable(bool enable);
  bool isEnabled() const;
  bool isBallEquipped() const;
  void handleBodyDetection(const op3_vision_msgs::msg::Detection::SharedPtr &msg);
  void handleCameraDetection(const op3_vision_msgs::msg::Detection::SharedPtr &msg);

private:
  void processDetections(const rclcpp::Time &now);
  void timeoutCheck();
  void updateBallEquipped(bool equipped);
  bool detectionFresh(const rclcpp::Time &stamp, const rclcpp::Time &now) const;

  rclcpp::Node *node_;
  BallFollowerConfig config_;
  rclcpp::TimerBase::SharedPtr timeout_timer_;

  bool enabled_{false};
  bool walking_active_{false};
  bool paused_for_timeout_{false};
  rclcpp::Time last_body_detection_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_camera_detection_time_{0, 0, RCL_ROS_TIME};
  double last_body_yaw_{0.0};
  double last_camera_yaw_{0.0};
  double last_body_bbox_area_{0.0};
  double last_camera_bbox_area_{0.0};
  rclcpp::Time last_detection_time_{0, 0, RCL_ROS_TIME};
  bool ball_equipped_{false};
  rclcpp::Time last_equipped_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace stride::op3
