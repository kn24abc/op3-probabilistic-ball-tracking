// ball_tracker.cpp
// Component that manages head tracking based on filtered ball detections.

#include <algorithm>

#include <chrono>
#include <cmath>
#include "op3_control_bridge/ball_tracker.hpp"

namespace stride::op3
{

BallTracker::BallTracker(rclcpp::Node *node, const BallTrackerConfig &config)
  : node_(node),
    config_(config)
{
  if (!node_)
  {
    throw std::runtime_error("BallTracker requires a valid node pointer");
  }

  // changed from SensorQoS to 10 to match ball_follower
  detection_sub_ = node_->create_subscription<op3_vision_msgs::msg::Detection>(
      config_.input_topic, 10,
      std::bind(&BallTracker::onDetection, this, std::placeholders::_1));

  auto period = std::chrono::duration<double>(1.0 / std::max(1.0, config_.control_rate_hz));
  control_timer_ = node_->create_wall_timer(period, std::bind(&BallTracker::controlLoop, this));

  RCLCPP_INFO(node_->get_logger(),
              "BallTracker listening to %s (params: control_bridge_params.yaml)",
              config_.input_topic.c_str());
}

void BallTracker::enable(bool enable)
{
  enabled_ = enable;
  RCLCPP_INFO(node_->get_logger(), "BallTracker %s", enable ? "enabled" : "disabled");
}

bool BallTracker::isEnabled() const
{
  return enabled_;
}

void BallTracker::onDetection(const op3_vision_msgs::msg::Detection::SharedPtr msg)
{
  if (!enabled_)
    return;

  /*
    RCLCPP_DEBUG(node_->get_logger(),
               "BallTracker got detection yaw=%.3f pitch=%.3f",
               msg->bearing.x, msg->bearing.y);
  */

  last_pan_error_ = std::clamp(
      static_cast<double>(msg->bearing.x),
      -config_.max_pan_rad, config_.max_pan_rad);

  last_tilt_error_ = std::clamp(
      static_cast<double>(msg->bearing.y),
      -config_.max_tilt_rad, config_.max_tilt_rad);

  last_detection_ = *msg;
  last_detection_time_ = node_->get_clock()->now();
  have_detection_ = true;
  missed_count_ = 0;
  transitionState(TrackingState::Found, "fresh detection");
  scan_sent_ = false;

  setVisibility(true);
}

void BallTracker::controlLoop()
{
  if (!enabled_)
    return;
    
  const auto now = node_->get_clock()->now();
  const bool detection_fresh = have_detection_ &&
      ((now - last_detection_time_).seconds() <= config_.lost_timeout);

  if (!detection_fresh)
  {
    missed_count_++;

    const int wait_limit = std::max(0, config_.wait_cycles);
    const int pre_scan_limit =
        std::max(wait_limit, wait_limit + std::max(0, config_.pre_scan_cycles));

    if (missed_count_ < wait_limit)
    {
      transitionState(TrackingState::Waiting, "waiting grace period");
      return;
    }

    transitionState(TrackingState::Lost, "ball not detected");
    setVisibility(false);
    resetIntegrators();

    const double seconds_missing = have_detection_
      ? (now - last_detection_time_).seconds()
      : missed_count_ * (1.0 / std::max(1.0, config_.control_rate_hz));
    (void)seconds_missing;

    if (scan_sent_ && config_.scan_in_progress_cb)
    {
      if (!config_.scan_in_progress_cb())
      {
        scan_sent_ = false;
      }
    }

    if (scan_sent_ && config_.scan_interval > 0.0 &&
        last_scan_command_time_.nanoseconds() > 0)
    {
      const double since_scan =
        (now - last_scan_command_time_).seconds();
      if (since_scan >= config_.scan_interval)
      {
        scan_sent_ = false;
        RCLCPP_DEBUG(node_->get_logger(),
                     "BallTracker scan window expired (%.2fs), allowing new scan.",
                     since_scan);
      }
    }

    if (missed_count_ < pre_scan_limit)
    {
      return;
    }

    if (!scan_sent_ && config_.use_scan && config_.scan_request_cb &&
        missed_count_ >= pre_scan_limit)
    {
      const bool accepted = config_.scan_request_cb(config_.scan_command);
      if (accepted)
      {
        RCLCPP_INFO(node_->get_logger(),
                    "BallTracker requesting scan '%s' after %d lost cycles",
                    config_.scan_command.c_str(), missed_count_);
        scan_sent_ = true;
        last_scan_command_time_ = now;
        return;
      }
    }

    if (!scan_sent_ && missed_count_ >= pre_scan_limit)
    {
      if (!config_.use_scan)
      {
        RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                              "Scan disabled; remaining in lost state.");
      }
      else if (!config_.scan_request_cb && !scan_unavailable_warned_)
      {
        RCLCPP_WARN(node_->get_logger(),
                    "Scan requested but no callback is wired; enable head scan publisher.");
        scan_unavailable_warned_ = true;
      }
    }

    return;
  }

  const double dt = computeDt(now);
  if (dt <= 0.0)
    return;

  auto apply_deadband = [this](double value) {
    if (std::fabs(value) < config_.deadband_rad)
      return 0.0;
    return value;
  };

  const double pan_error = apply_deadband(last_pan_error_);
  const double tilt_error = apply_deadband(last_tilt_error_);

  static const double epsilon = 1e-6;
  if (std::fabs(pan_error) < epsilon && std::fabs(tilt_error) < epsilon)
  {
    resetIntegrators();
    return;
  }

  pan_integral_ = std::clamp(pan_integral_ + pan_error * dt,
                             -config_.integral_limit_rad, config_.integral_limit_rad);
  tilt_integral_ = std::clamp(tilt_integral_ + tilt_error * dt,
                              -config_.integral_limit_rad, config_.integral_limit_rad);

  const double pan_derivative = (pan_error - prev_pan_error_) / dt;
  const double tilt_derivative = (tilt_error - prev_tilt_error_) / dt;

  double pan_cmd = config_.p_gain * pan_error +
                   config_.i_gain * pan_integral_ +
                   config_.d_gain * pan_derivative;
  double tilt_cmd = config_.p_gain * tilt_error +
                    config_.i_gain * tilt_integral_ +
                    config_.d_gain * tilt_derivative;

  pan_cmd = std::clamp(pan_cmd, -config_.max_pan_rad, config_.max_pan_rad);
  tilt_cmd = std::clamp(tilt_cmd, -config_.max_tilt_rad, config_.max_tilt_rad);

  if (std::fabs(pan_cmd) < config_.min_command_rad)
    pan_cmd = 0.0;
  if (std::fabs(tilt_cmd) < config_.min_command_rad)
    tilt_cmd = 0.0;
  if (pan_cmd == 0.0 && tilt_cmd == 0.0)
    return;

  // {
  //   auto logger = node_->get_logger();
  //   RCLCPP_DEBUG(logger, "BallTracker PID status");
  //   RCLCPP_DEBUG(logger, "error(deg): pan=%.2f tilt=%.2f", last_pan_error_ * 180.0 / M_PI, last_tilt_error_ * 180.0 / M_PI);
  //   RCLCPP_DEBUG(logger, "derivative(deg/s): pan=%.2f tilt=%.2f (dt=%.3f)", pan_derivative * 180.0 / M_PI, tilt_derivative * 180.0 / M_PI, dt);
  //   RCLCPP_DEBUG(logger, "integral(deg): pan=%.2f tilt=%.2f", pan_integral_ * 180.0 / M_PI, tilt_integral_ * 180.0 / M_PI);
  //   RCLCPP_DEBUG(logger, "command(deg): pan=%.2f tilt=%.2f | P=%.2f I=%.2f D=%.2f", pan_cmd * 180.0 / M_PI, tilt_cmd * 180.0 / M_PI, config_.p_gain, config_.i_gain, config_.d_gain);
  // }

  prev_pan_error_ = last_pan_error_;
  prev_tilt_error_ = last_tilt_error_;

  const bool pan_saturated = std::abs(pan_cmd) >= config_.max_pan_rad * 0.999;
  const bool tilt_saturated = std::abs(tilt_cmd) >= config_.max_tilt_rad * 0.999;
  if (pan_saturated && ((pan_cmd > 0 && pan_error > 0) || (pan_cmd < 0 && pan_error < 0)))
    return;
  if (tilt_saturated && ((tilt_cmd > 0 && tilt_error > 0) || (tilt_cmd < 0 && tilt_error < 0)))
    return;

  if (config_.head_command_cb)
    config_.head_command_cb(pan_cmd, tilt_cmd);
}

void BallTracker::resetIntegrators()
{
  pan_integral_ = 0.0;
  tilt_integral_ = 0.0;
  prev_pan_error_ = 0.0;
  prev_tilt_error_ = 0.0;
}

double BallTracker::computeDt(const rclcpp::Time &now)
{
  double dt = 1.0 / std::max(1.0, config_.control_rate_hz);
  if (last_control_time_.nanoseconds() > 0)
  {
    dt = (now - last_control_time_).seconds();
    if (dt <= 0.0)
      dt = 1.0 / std::max(1.0, config_.control_rate_hz);
  }
  last_control_time_ = now;
  return dt;
}

void BallTracker::setVisibility(bool visible)
{
  if (last_visibility_state_ == visible)
    return;

  last_visibility_state_ = visible;
  if (config_.visibility_cb)
    config_.visibility_cb(visible);
}

void BallTracker::transitionState(TrackingState new_state, const char *reason)
{
  if (state_ == new_state)
    return;

  if (config_.state_change_cb)
    config_.state_change_cb(new_state);

  const char *reason_text = reason ? reason : "";
  RCLCPP_INFO(node_->get_logger(),
              "BallTracker state -> %s %s",
              stateToString(new_state),
              reason_text);
  state_ = new_state;
}

const char *BallTracker::stateToString(TrackingState state)
{
  switch (state)
  {
    case TrackingState::Found:
      return "FOUND";
    case TrackingState::Waiting:
      return "WAITING";
    case TrackingState::Lost:
      return "LOST";
    default:
      return "UNKNOWN";
  }
}



}  // namespace stride::op3
