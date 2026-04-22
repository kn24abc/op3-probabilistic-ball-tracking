#pragma once

#include <functional>
#include <memory>
#include <random>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace stride::op3
{

struct BallSearcherConfig
{
  double search_turn_rate{0.2};      // radians per second equivalent
  double search_forward_step{0.0};   // meters per step
  double command_period_sec{1.0};    // how often to refresh the gait command
  int turn_hold_cycles{4};           // how many timer ticks to hold each turn direction
  bool alternate_turn{true};         // toggle turn direction to sweep area
  double scan_request_period_sec{2.5};
  std::function<void(const std::string &cmd)> scan_request_cb;
  std::function<void(double fb_move, double rl_turn)> walking_param_cb;
  std::function<void(bool start)> walking_command_cb;
};

class BallSearcher
{
public:
  BallSearcher(rclcpp::Node *node, const BallSearcherConfig &config);

  void enable(bool enable);
  bool isEnabled() const;

private:
  void onTimer();

  rclcpp::Node *node_{nullptr};
  BallSearcherConfig config_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool enabled_{false};
  bool walking_{false};
  int turn_direction_{1};
  int turn_cycle_count_{0};
  rclcpp::Time last_scan_request_{0, 0, RCL_ROS_TIME};
  std::mt19937 rng_;
};

}  // namespace stride::op3
