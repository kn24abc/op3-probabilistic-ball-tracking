#include "op3_control_bridge/ball_dribbler.hpp"

#include <algorithm>
#include <cmath>
#include <chrono>
#include <stdexcept>

namespace stride::op3
{

BallDribbler::BallDribbler(rclcpp::Node *node, const BallDribblerConfig &config)
: node_(node),
  config_(config)
{
  if (!node_)
    throw std::runtime_error("BallDribbler requires a valid node pointer");

  // Backward compatibility: if body_input_topic not set explicitly, use input_topic.
  if (config_.body_input_topic.empty())
    config_.body_input_topic = config_.input_topic;
  if (config_.body_input_topic.empty())
    config_.body_input_topic = "/perception/ball";

  detection_body_sub_ = node_->create_subscription<op3_vision_msgs::msg::Detection>(
      config_.body_input_topic, 10,
      std::bind(&BallDribbler::onBodyDetection, this, std::placeholders::_1));

  if (!config_.camera_input_topic.empty())
  {
    detection_camera_sub_ = node_->create_subscription<op3_vision_msgs::msg::Detection>(
        config_.camera_input_topic, 10,
        std::bind(&BallDribbler::onCameraDetection, this, std::placeholders::_1));
  }

  using namespace std::chrono_literals;
  timeout_timer_ = node_->create_wall_timer(
      100ms,
      std::bind(&BallDribbler::timeoutCheck, this));

  RCLCPP_INFO(node_->get_logger(), "BallDribbler listening to %s",
              config_.input_topic.c_str());
}

void BallDribbler::enable(bool enable)
{
  if (enabled_ == enable)
    return;

  enabled_ = enable;
  if (!enabled_)
  {
    stopWalking();
    ball_locked_ = false;
    last_detection_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    RCLCPP_INFO(node_->get_logger(), "BallDribbler disabled");
    return;
  }

  RCLCPP_INFO(node_->get_logger(), "BallDribbler enabled");
}

bool BallDribbler::isEnabled() const
{
  return enabled_;
}

bool BallDribbler::hasBallLocked() const
{
  return ball_locked_;
}

void BallDribbler::onBodyDetection(const op3_vision_msgs::msg::Detection::SharedPtr msg)
{
  if (!enabled_ || !msg)
    return;

  last_body_detection_time_ = node_->get_clock()->now();
  last_body_yaw_ = msg->bearing.y;   // body-frame bearing expected on /perception/ball
  last_body_bbox_area_ = msg->bbox.size_x * msg->bbox.size_y;
  last_detection_time_ = last_body_detection_time_;
  processDetections(last_detection_time_);
}

void BallDribbler::onCameraDetection(const op3_vision_msgs::msg::Detection::SharedPtr msg)
{
  if (!enabled_ || !msg)
    return;

  last_camera_detection_time_ = node_->get_clock()->now();
  last_camera_yaw_ = msg->bearing.x; // camera-frame yaw on /perception/ball_cam
  last_camera_bbox_area_ = msg->bbox.size_x * msg->bbox.size_y;
  last_detection_time_ = last_camera_detection_time_;
  processDetections(last_detection_time_);
}

bool BallDribbler::detectionFresh(const rclcpp::Time &stamp, const rclcpp::Time &now) const
{
  if (stamp.nanoseconds() == 0)
    return false;
  return (now - stamp).seconds() <= config_.stop_timeout_sec;
}

void BallDribbler::processDetections(const rclcpp::Time &now)
{
  if (!enabled_)
    return;

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

  const bool lock_ready =
      bbox_area >= config_.lock_bbox_min_area &&
      std::fabs(heading_error) <= config_.lock_max_abs_bearing;

  if (!lock_ready)
  {
    if (ball_locked_)
    {
      RCLCPP_INFO(node_->get_logger(),
                  "BallDribbler lost lock (bbox %.0f, bearing %.3f)",
                  bbox_area, heading_error);
      ball_locked_ = false;
      if (config_.lost_ball_cb)
        config_.lost_ball_cb();
    }
    // Turn in place to acquire/centre the ball before engaging dribble.
    if (config_.walking_param_cb)
    {
      const double turn_cmd = std::clamp(
          0.05 * heading_error, -config_.max_turn_rad, config_.max_turn_rad);
      config_.walking_param_cb(0.0, turn_cmd);
      if (!walking_active_ && config_.walking_command_cb)
      {
        config_.walking_command_cb(true);
        walking_active_ = true;
        RCLCPP_DEBUG(node_->get_logger(), "BallDribbler turning to reacquire ball");
      }
    }
    return;
  }

  if (!ball_locked_)
  {
    RCLCPP_INFO(node_->get_logger(),
                "BallDribbler engaged ball (bbox %.0f, bearing %.3f)",
                bbox_area, heading_error);
  }
  ball_locked_ = true;

  if (!config_.walking_param_cb)
    return;

  if (config_.goal_bearing_cb)
  {
    auto goal_bearing_opt = config_.goal_bearing_cb();
    if (goal_bearing_opt)
    {
      const double goal_bearing = *goal_bearing_opt;
      if (std::abs(goal_bearing) > config_.goal_alignment_deadband_rad)
      {
        heading_error -= config_.goal_alignment_weight * goal_bearing;
        if (config_.goal_alignment_hint_cb)
          config_.goal_alignment_hint_cb();
      }
    }
  }

  // Reduce turn rate to keep balance; spread the rotation over multiple timesteps.
  double turn = std::clamp(0.05 * heading_error, -config_.max_turn_rad, config_.max_turn_rad);
  double fb = 0.0;
  // Advance only when we are well aligned; otherwise keep turning in place.
  if (std::fabs(heading_error) < 0.7 * config_.lock_max_abs_bearing)
  {
    fb = computeForwardStep(bbox_area);
  }
  config_.walking_param_cb(fb, turn);

  if (!walking_active_ && config_.walking_command_cb)
  {
    config_.walking_command_cb(true);
    walking_active_ = true;
    RCLCPP_DEBUG(node_->get_logger(), "BallDribbler issuing walk start");
  }
}

void BallDribbler::timeoutCheck()
{
  if (!enabled_ || !walking_active_)
    return;

  if (last_detection_time_.nanoseconds() == 0)
    return;

  const auto elapsed = (node_->get_clock()->now() - last_detection_time_).seconds();
  if (elapsed > config_.stop_timeout_sec)
  {
    RCLCPP_INFO(node_->get_logger(),
                "BallDribbler timed out (%.2fs since last lock)", elapsed);
    stopWalking();
    ball_locked_ = false;
  }
}

void BallDribbler::stopWalking()
{
  if (!walking_active_)
    return;

  if (config_.walking_command_cb)
  {
    config_.walking_command_cb(false);
  }
  walking_active_ = false;
}

double BallDribbler::computeForwardStep(double bbox_area) const
{
  const double max_step = std::max(config_.forward_step, config_.min_forward_step);
  const double min_step = std::min(config_.forward_step, config_.min_forward_step);
  const double span = std::max(1.0, config_.close_bbox_area - config_.lock_bbox_min_area);
  double t = (bbox_area - config_.lock_bbox_min_area) / span;
  t = std::clamp(t, 0.0, 1.0);
  return max_step - (max_step - min_step) * t;
}

}  // namespace stride::op3
