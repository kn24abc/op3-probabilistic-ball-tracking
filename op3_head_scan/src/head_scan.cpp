#include "op3_head_scan/head_scan.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <numeric>
#include <stdexcept>

namespace op3_head_scan
{

namespace
{
constexpr double kDefaultStepRad = 1.0 * M_PI / 180.0;
}

HeadScan::HeadScan(rclcpp::Node *node, HeadScanConfig config)
  : node_(node),
    config_(std::move(config)),
    rng_(std::random_device{}())
{
  if (!node_)
  {
    throw std::runtime_error("HeadScan requires a valid node pointer");
  }

  head_offset_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
      config_.head_offset_topic, rclcpp::QoS(5));
  buildCandidateTargets();
}

void HeadScan::start(double pan_hint, double tilt_hint)
{
  if (!head_offset_pub_)
    return;

  if (timer_)
    timer_->cancel();

  targets_ = planTargets(pan_hint, tilt_hint);
  if (targets_.empty())
  {
    RCLCPP_WARN(node_->get_logger(),
                "HeadScan has no targets to sweep (check limits/steps).");
    return;
  }

  active_ = true;
  have_current_pose_ = true;
  current_pan_cmd_ = clampPan(pan_hint);
  current_tilt_cmd_ = clampTilt(tilt_hint);
  publishCurrentPose();

  next_index_ = 0;

  const double period = std::max(config_.command_period_sec, 0.05);
  timer_ = node_->create_wall_timer(
      std::chrono::duration<double>(period),
      std::bind(&HeadScan::handleTimer, this));
}

void HeadScan::cancel()
{
  active_ = false;
  targets_.clear();
  next_index_ = 0;
  have_current_pose_ = false;
  if (timer_)
    timer_->cancel();
}

bool HeadScan::isActive() const
{
  return active_;
}

void HeadScan::publishTarget(const Target &target)
{
  if (!head_offset_pub_)
    return;

  sensor_msgs::msg::JointState msg;
  msg.header.stamp = node_->now();
  msg.name = {"head_pan", "head_tilt"};
  msg.position = {target.first, target.second};
  head_offset_pub_->publish(msg);

  if (config_.command_feedback_cb)
    config_.command_feedback_cb(target.first, target.second);
}

void HeadScan::publishCurrentPose()
{
  if (!have_current_pose_)
    return;

  RCLCPP_DEBUG(node_->get_logger(),
               "HeadScan command pan=%.1fdeg tilt=%.1fdeg",
               current_pan_cmd_ * 180.0 / M_PI,
               current_tilt_cmd_ * 180.0 / M_PI);
  publishTarget({current_pan_cmd_, current_tilt_cmd_});
}

bool HeadScan::advanceToward(const Target &target)
{
  if (!have_current_pose_)
  {
    current_pan_cmd_ = clampPan(target.first);
    current_tilt_cmd_ = clampTilt(target.second);
    have_current_pose_ = true;
    return true;
  }

  const double pan_delta = target.first - current_pan_cmd_;
  const double tilt_delta = target.second - current_tilt_cmd_;

  const double pan_step_limit = std::max(config_.pan_slew_step_rad, kDefaultStepRad);
  const double tilt_step_limit = std::max(config_.tilt_slew_step_rad, kDefaultStepRad);

  double new_pan = current_pan_cmd_;
  double new_tilt = current_tilt_cmd_;

  if (std::fabs(pan_delta) > pan_step_limit)
    new_pan += (pan_delta > 0.0 ? pan_step_limit : -pan_step_limit);
  else
    new_pan = target.first;

  if (std::fabs(tilt_delta) > tilt_step_limit)
    new_tilt += (tilt_delta > 0.0 ? tilt_step_limit : -tilt_step_limit);
  else
    new_tilt = target.second;

  current_pan_cmd_ = clampPan(new_pan);
  current_tilt_cmd_ = clampTilt(new_tilt);

  const bool pan_reached = std::fabs(target.first - current_pan_cmd_) <= 1e-4;
  const bool tilt_reached = std::fabs(target.second - current_tilt_cmd_) <= 1e-4;
  return pan_reached && tilt_reached;
}

void HeadScan::handleTimer()
{
  if (!active_)
    return;

  if (targets_.empty())
  {
    targets_ = planTargets(current_pan_cmd_, current_tilt_cmd_);
    next_index_ = 0;
    if (targets_.empty())
    {
      cancel();
      return;
    }
  }

  if (next_index_ >= targets_.size())
  {
    if (config_.repeat)
    {
      next_index_ = 0;
    }
    else
    {
      targets_.clear();
      return;
    }
  }

  if (next_index_ >= targets_.size())
    return;

  const bool reached = advanceToward(targets_[next_index_]);
  publishCurrentPose();

  if (reached)
  {
    next_index_++;
    if (next_index_ >= targets_.size() && !config_.repeat)
    {
      targets_.clear();
    }
  }
}

std::vector<HeadScan::Target> HeadScan::planTargets(double pan_hint, double tilt_hint) const
{
  std::vector<Target> planned;
  if (candidate_targets_.empty())
  {
    RCLCPP_WARN(node_->get_logger(), "HeadScan has no candidate targets configured.");
    return planned;
  }

  // Sort candidates by distance to the hint so the scan starts near the
  // particle filter's predicted ball location.
  auto bag = candidate_targets_;
  std::sort(bag.begin(), bag.end(), [pan_hint, tilt_hint](const Target &a, const Target &b) {
    const double da = std::hypot(a.first - pan_hint, a.second - tilt_hint);
    const double db = std::hypot(b.first - pan_hint, b.second - tilt_hint);
    return da < db;
  });

  // If random_target_count <= 0, use all cells; otherwise cap to that many
  const size_t desired_count =
      (config_.random_target_count <= 0)
          ? bag.size()
          : std::min(bag.size(),
                     static_cast<size_t>(std::max(1, config_.random_target_count)));

  planned.insert(planned.end(), bag.begin(), bag.begin() + desired_count);
  RCLCPP_DEBUG(node_->get_logger(),
               "HeadScan planned %zu targets (hint pan=%.1fdeg tilt=%.1fdeg)",
               planned.size(), pan_hint * 180.0 / M_PI, tilt_hint * 180.0 / M_PI);
  return planned;
}

double HeadScan::clampPan(double pan) const
{
  return std::clamp(pan, -config_.max_pan_rad, config_.max_pan_rad);
}

double HeadScan::clampTilt(double tilt) const
{
  return std::clamp(tilt, config_.min_tilt_rad, config_.max_tilt_rad);
}

void HeadScan::buildCandidateTargets()
{
  candidate_targets_.clear();
  const int pan_cells = std::max(1, config_.pan_cells);
  const int tilt_cells = std::max(1, config_.tilt_cells);

  auto generate_values = [](double min_val, double max_val, int cells) {
    std::vector<double> values;
    if (cells <= 1)
    {
      values.push_back((min_val + max_val) * 0.5);
      return values;
    }
    const double step = (max_val - min_val) / static_cast<double>(cells - 1);
    for (int i = 0; i < cells; ++i)
    {
      values.push_back(min_val + step * static_cast<double>(i));
    }
    return values;
  };

  const auto pan_values = generate_values(-config_.max_pan_rad, config_.max_pan_rad, pan_cells);
  const double tilt_max = std::min(config_.max_upward_tilt_rad, config_.max_tilt_rad);
  const auto tilt_values = generate_values(config_.min_tilt_rad, tilt_max, tilt_cells);

  for (double pan : pan_values)
  {
    for (double tilt : tilt_values)
    {
      candidate_targets_.emplace_back(clampPan(pan), clampTilt(tilt));
    }
  }
  if (candidate_targets_.empty())
  {
    RCLCPP_WARN(node_->get_logger(), "HeadScan generated zero candidate targets.");
  }
}

bool HeadScan::almostEqual(double lhs, double rhs)
{
  return std::fabs(lhs - rhs) <= 1e-4;
}

}  // namespace op3_head_scan
