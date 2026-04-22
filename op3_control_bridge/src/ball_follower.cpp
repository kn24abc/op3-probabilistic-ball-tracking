#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

#include "op3_control_bridge/ball_follower.hpp"

namespace stride::op3
{

BallFollower::BallFollower(rclcpp::Node *node, const BallFollowerConfig &config)
  : node_(node),
    config_(config)
{
  if (!node_)
    throw std::runtime_error("BallFollower requires a valid node pointer");

  timeout_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&BallFollower::timeoutCheck, this));

  RCLCPP_INFO(node_->get_logger(),
              "BallFollower awaiting detections (body: %s, cam: %s)",
              config_.body_input_topic.c_str(),
              config_.camera_input_topic.c_str());
}

void BallFollower::enable(bool enable)
{
  enabled_ = enable;
  if (!enabled_ && walking_active_ && config_.walking_command_cb)
  {
    config_.walking_command_cb(false);
    walking_active_ = false;
  }
  if (!enabled_)
  {
    updateBallEquipped(false);
    last_equipped_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }
  RCLCPP_INFO(node_->get_logger(), "BallFollower %s", enable ? "enabled" : "disabled");
}

bool BallFollower::isEnabled() const
{
  return enabled_;
}

bool BallFollower::isBallEquipped() const
{
  return ball_equipped_;
}

void BallFollower::handleBodyDetection(const op3_vision_msgs::msg::Detection::SharedPtr &msg)
{
  if (!enabled_)
    return;

  last_detection_time_ = node_->get_clock()->now();
  last_body_detection_time_ = last_detection_time_;
  last_body_yaw_ = msg->bearing.y;   // body-frame bearing expected on /perception/ball
  last_body_bbox_area_ = msg->bbox.size_x * msg->bbox.size_y;
  processDetections(last_detection_time_);
}

void BallFollower::handleCameraDetection(const op3_vision_msgs::msg::Detection::SharedPtr &msg)
{
  if (!enabled_)
    return;

  last_detection_time_ = node_->get_clock()->now();
  last_camera_detection_time_ = last_detection_time_;
  last_camera_yaw_ = msg->bearing.x; // camera-frame yaw on /perception/ball_cam
  last_camera_bbox_area_ = msg->bbox.size_x * msg->bbox.size_y;
  processDetections(last_detection_time_);
}

void BallFollower::timeoutCheck()
{
  if (!enabled_ || !walking_active_)
    return;

  if (last_detection_time_.nanoseconds() == 0)
    return;

  const auto elapsed = (node_->get_clock()->now() - last_detection_time_).seconds();
  
  if (elapsed > config_.stop_timeout_sec)
  {
    if (!paused_for_timeout_)
    {
      RCLCPP_DEBUG(node_->get_logger(),
                   "BallFollower paused: last detection %.2fs ago (timeout %.2fs)",
                   elapsed, config_.stop_timeout_sec);
      paused_for_timeout_ = true;
    }
    // hold pose without toggling walking; resumed on next detection
    if (config_.walking_param_cb)
      config_.walking_param_cb(0.0, 0.0);
  }

  if (ball_equipped_ && last_equipped_time_.nanoseconds() > 0)
  {
    const auto equipped_elapsed = (node_->get_clock()->now() - last_equipped_time_).seconds();
    if (equipped_elapsed > config_.ball_equipped_hold_timeout_sec)
    {
      RCLCPP_INFO(node_->get_logger(),
                  "Ball unequipped (no close detection for %.2fs)",
                  equipped_elapsed);
      updateBallEquipped(false);
    }
  }
}

void BallFollower::updateBallEquipped(bool equipped)
{
  if (ball_equipped_ == equipped)
    return;

  ball_equipped_ = equipped;
  if (config_.ball_equipped_cb)
  {
    config_.ball_equipped_cb(equipped);
  }
}

bool BallFollower::detectionFresh(const rclcpp::Time &stamp, const rclcpp::Time &now) const
{
  if (stamp.nanoseconds() == 0)
    return false;
  return (now - stamp).seconds() <= config_.stop_timeout_sec;
}

void BallFollower::processDetections(const rclcpp::Time &now)
{
  const bool have_cam = detectionFresh(last_camera_detection_time_, now);
  const bool have_body = detectionFresh(last_body_detection_time_, now);
  if (!have_cam && !have_body)
    return;

  const double yaw_cam = have_cam ? last_camera_yaw_ : 0.0;
  const double yaw_body = have_body ? last_body_yaw_ : yaw_cam;

  const double bbox_area =
    have_body ? last_body_bbox_area_ :
    (have_cam ? last_camera_bbox_area_ : 0.0);

  // Adaptive blend: fade body yaw out when we are side-on (large same-sign disagreement)
  // and when close to the ball (large bbox).
  const double base_k = std::clamp(config_.body_yaw_weight, 0.0, 1.0);

  double geom_factor = 1.0;
  if (have_cam && have_body && yaw_cam * yaw_body > 0.0)
  {
    const double diff = std::fabs(yaw_body - yaw_cam);  // disagreement magnitude
    constexpr double diff_start = 0.2;  // rad: start fading body influence
    constexpr double diff_full = 0.7;   // rad: body influence gone
    if (diff <= diff_start)
    {
      geom_factor = 1.0;
    }
    else if (diff >= diff_full)
    {
      geom_factor = 0.0;
    }
    else
    {
      const double t = (diff - diff_start) / (diff_full - diff_start);
      geom_factor = 1.0 - t;
    }
  }

  double dist_factor = 1.0;
  if (bbox_area > 0.0)
  {
    constexpr double area_far = 800.0;
    constexpr double area_close = 2000.0;
    if (bbox_area <= area_far)
      dist_factor = 1.0;
    else if (bbox_area >= area_close)
      dist_factor = 0.0;
    else
    {
      const double t = (bbox_area - area_far) / (area_close - area_far);
      dist_factor = 1.0 - t;
    }
  }

  const double k = base_k * geom_factor * dist_factor;
  double heading_error = 0.0;
  if (have_cam && have_body)
    heading_error = (1.0 - k) * yaw_cam + k * yaw_body;
  else if (have_cam)
    heading_error = yaw_cam;
  else
    heading_error = yaw_body;

  // simple heading control
  double turn = std::clamp(heading_error,
                           -config_.max_turn_rad,
                           config_.max_turn_rad);

  double fb = 0.0;
  const double align_threshold = 0.25; // ~14 deg
  if (std::fabs(heading_error) < align_threshold)
  {
    fb = std::clamp(config_.nominal_forward_step,
                    -config_.max_forward_step,
                    config_.max_forward_step);
  }

  RCLCPP_DEBUG(node_->get_logger(),
               "BallFollower cmd MIXED: fb=%.4f turn=%.4f (heading_error=%.3f, k=%.2f, geom=%.2f, dist=%.2f, cam=%d body=%d)",
               fb, turn, heading_error, k, geom_factor, dist_factor,
               static_cast<int>(have_cam), static_cast<int>(have_body));

  config_.walking_param_cb(fb, turn);

  const bool resume_after_timeout = paused_for_timeout_;
  if (config_.walking_command_cb && (!walking_active_ || resume_after_timeout))
  {
    config_.walking_command_cb(true);
    walking_active_ = true;
    paused_for_timeout_ = false;
  }
  else
  {
    paused_for_timeout_ = false;
  }

  const double equipped_yaw = have_cam ? yaw_cam : yaw_body;
  const bool equipped_now =
      (bbox_area >= config_.ball_equipped_bbox_min_area) &&
      (std::fabs(equipped_yaw) <= config_.ball_equipped_max_abs_bearing);

  if (equipped_now)
  {
    last_equipped_time_ = now;
    if (!ball_equipped_)
    {
      RCLCPP_INFO(node_->get_logger(),
                  "Ball equipped (bbox %.0f px^2, yaw %.3f, source=%s)",
                  bbox_area, equipped_yaw, have_cam ? "cam" : "body");
    }
    updateBallEquipped(true);
  }
}

}  // namespace stride::op3
