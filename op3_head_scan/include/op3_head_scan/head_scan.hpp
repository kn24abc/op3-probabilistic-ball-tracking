#pragma once

#include <functional>
#include <utility>
#include <vector>
#include <string>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace op3_head_scan
{

struct HeadScanConfig
{
  std::string head_offset_topic{"/robotis/head_control/set_joint_states_offset"};
  double command_period_sec{0.35};
  double pan_step_rad{24.0 * M_PI / 180.0};
  double tilt_step_rad{18.0 * M_PI / 180.0};
  double sweep_step_rad{2.0 * M_PI / 180.0};
  double max_pan_rad{80.0 * M_PI / 180.0};
  double min_tilt_rad{-5.0 * M_PI / 180.0};
  double max_tilt_rad{15.0 * M_PI / 180.0};
  double max_upward_tilt_rad{15.0 * M_PI / 180.0};
  double start_pan_rad{0.0};
  double start_tilt_rad{0.0};
  int max_layers{4};
  bool repeat{false};
  double pan_slew_step_rad{4.0 * M_PI / 180.0};
  double tilt_slew_step_rad{3.0 * M_PI / 180.0};
  int random_target_count{0};  // 0 = use all grid cells
  int pan_cells{3};
  int tilt_cells{3};
  std::function<void(double pan, double tilt)> command_feedback_cb;
};

class HeadScan
{
public:
  HeadScan(rclcpp::Node *node, HeadScanConfig config);

  void start(double pan_hint, double tilt_hint);
  void cancel();
  bool isActive() const;
  const HeadScanConfig &getConfig() const { return config_; }

private:
  using Target = std::pair<double, double>;

  void publishTarget(const Target &target);
  void handleTimer();
  void publishCurrentPose();
  bool advanceToward(const Target &target);
  std::vector<Target> planTargets(double pan_hint, double tilt_hint) const;
  double clampPan(double pan) const;
  double clampTilt(double tilt) const;
  static bool almostEqual(double lhs, double rhs);
  void buildCandidateTargets();

  rclcpp::Node *node_;
  HeadScanConfig config_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr head_offset_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<Target> targets_;
  size_t next_index_{0};
  bool active_{false};
  bool have_current_pose_{false};
  double current_pan_cmd_{0.0};
  double current_tilt_cmd_{0.0};
  std::vector<Target> candidate_targets_;
  mutable std::mt19937 rng_;
};

}  // namespace op3_head_scan
