#include "op3_control_bridge/ball_searcher.hpp"

#include <chrono>

namespace stride::op3
{

BallSearcher::BallSearcher(rclcpp::Node *node, const BallSearcherConfig &config)
: node_(node),
  config_(config),
  rng_(std::random_device{}())
{
  if (!node_)
    throw std::runtime_error("BallSearcher requires a valid node pointer");

  using namespace std::chrono_literals;
  const auto period = std::chrono::duration<double>(
      std::max(0.1, config_.command_period_sec));

  timer_ = node_->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&BallSearcher::onTimer, this));

  timer_->cancel();
  RCLCPP_INFO(node_->get_logger(), "BallSearch ready (period %.2fs)",
              config_.command_period_sec);
}

void BallSearcher::enable(bool enable)
{
  if (enabled_ == enable)
    return;

  enabled_ = enable;
  if (!enabled_)
  {
    if (walking_ && config_.walking_command_cb)
    {
      config_.walking_command_cb(false);
    }
    walking_ = false;
    timer_->cancel();
    if (config_.scan_request_cb)
      config_.scan_request_cb("stop");
    RCLCPP_INFO(node_->get_logger(), "BallSearch disabled");
    return;
  }

  timer_->reset();
  turn_cycle_count_ = 0;
  turn_direction_ = config_.alternate_turn
                      ? ((rng_() & 1) ? 1 : -1)
                      : 1;
  last_scan_request_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  if (config_.scan_request_cb)
    config_.scan_request_cb("scan");
  RCLCPP_INFO(node_->get_logger(), "BallSearch enabled");
}

bool BallSearcher::isEnabled() const
{
  return enabled_;
}

void BallSearcher::onTimer()
{
  if (!enabled_ || !config_.walking_param_cb)
    return;

  const double fb = config_.search_forward_step;
  double turn = config_.search_turn_rate;
  if (config_.alternate_turn)
    turn *= static_cast<double>(turn_direction_);

  config_.walking_param_cb(fb, turn);

  if (!walking_ && config_.walking_command_cb)
  {
    config_.walking_command_cb(true);
    walking_ = true;
    RCLCPP_DEBUG(node_->get_logger(), "BallSearch issuing walking command");
  }

  if (config_.alternate_turn && config_.turn_hold_cycles > 0)
  {
    turn_cycle_count_++;
    if (turn_cycle_count_ >= config_.turn_hold_cycles)
    {
      turn_cycle_count_ = 0;
      turn_direction_ *= -1;
    }
  }

  if (config_.scan_request_cb && config_.scan_request_period_sec > 0.0)
  {
    const auto now = node_->get_clock()->now();
    if (last_scan_request_.nanoseconds() == 0 ||
        (now - last_scan_request_).seconds() >= config_.scan_request_period_sec)
    {
      config_.scan_request_cb("scan");
      last_scan_request_ = now;
    }
  }
}

}  // namespace stride::op3
