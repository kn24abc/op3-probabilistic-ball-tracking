// bt_control_bridge.cpp
// bridge between behavior tree and control modules
// includes command velocity and action handling 
// common control module switcher for cmd_vel and cmd_action

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <unordered_map>
#include <string>
#include <cmath>
#include <memory>
#include <optional>
#include <sstream>
#include <thread>
#include <cctype>
#include <deque>
#include <mutex>
#include <atomic>
#include "std_srvs/srv/trigger.hpp"
#include <atomic>
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "robotis_controller_msgs/srv/get_joint_module.hpp"
#include "robotis_controller_msgs/srv/set_joint_module.hpp"
#include "robotis_controller_msgs/srv/set_module.hpp"
#include "op3_walking_module_msgs/msg/walking_param.hpp"
#include "op3_action_module_msgs/srv/is_running.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "op3_vision_msgs/msg/detection.hpp"
#include <eigen3/Eigen/Geometry>
#include <robotis_math/robotis_linear_algebra.h>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "op3_control_bridge/ball_tracker.hpp"
#include "op3_control_bridge/ball_follower.hpp"
#include "op3_control_bridge/ball_dribbler.hpp"
#include "op3_control_bridge/ball_searcher.hpp"
#include "op3_control_bridge/goal_bearing_tracker.hpp"
#include "op3_control_bridge/gamestate_listener.hpp"
#include "op3_head_scan/head_scan.hpp"

namespace stride::op3 {

constexpr double DEG2RAD = M_PI / 180.0;

// TODO: move code to fall_detection.hpp
enum class FallDirection
{
  None,
  Forward,
  Backward
};
static const char* to_string(FallDirection fall_dir)
{
  switch(fall_dir)
  {
    case FallDirection::Forward:
      return "Forward";
    case FallDirection::Backward:
      return "Backward";
    case FallDirection::None:
      return "Upright";
    default:
      return "Unknown";
  }
}

static double getImuYaw(const sensor_msgs::msg::Imu &imu)
{
  Eigen::Quaterniond orientation(
      imu.orientation.w,
      imu.orientation.x,
      imu.orientation.y,
      imu.orientation.z);

  Eigen::MatrixXd rpy_orientation =
      robotis_framework::convertQuaternionToRPY(orientation);
  return rpy_orientation.coeff(2, 0);
}


// TODO replace this with a fusion Madgwick Robocup Compliant method
// disable magnetometer on OpenCR for matches
static FallDirection getFallDirection(const sensor_msgs::msg::Imu& imu)
{
  Eigen::Quaterniond orientation(
      imu.orientation.w,
      imu.orientation.x,
      imu.orientation.y,
      imu.orientation.z);

  Eigen::MatrixXd rpy_orientation =
      robotis_framework::convertQuaternionToRPY(orientation);

  const double roll_deg  = rpy_orientation.coeff(0, 0) * (180.0 / M_PI);
  const double pitch_deg = rpy_orientation.coeff(1, 0) * (180.0 / M_PI);

  constexpr double kFallLimitDeg = 60.0;

  if(pitch_deg > kFallLimitDeg)
  {
    return FallDirection::Forward;
  }
  if(pitch_deg < -kFallLimitDeg)
  {
    return FallDirection::Backward;
  }
  if(roll_deg > kFallLimitDeg || roll_deg < -kFallLimitDeg)
  {
    return FallDirection::Forward;
  }
  return FallDirection::None;
}



// control module names
inline constexpr const char* kBaseModule = "base_module";
inline constexpr const char* kActionModule = "action_module";
inline constexpr const char* kHeadControlModule = "head_control_module";
inline constexpr const char* kWalkingModule = "walking_module";
inline constexpr const char* kDirectControlModule = "direct_control_module";
inline constexpr const char* kOnlineWalkingModule = "online_walking_module";
inline constexpr const char* kBallTrackEnableTopic = "/control_bridge/ball_track_enable";

class ControlBridge : public rclcpp::Node {
public:
  static inline constexpr const char* kGetJointService = "/robotis/get_present_joint_ctrl_modules";
  static inline constexpr const char* kSetJointService = "/robotis/set_present_joint_ctrl_modules";
  static inline constexpr const char* kSetCtrlModuleService = "/robotis/set_present_ctrl_modules";
  static inline constexpr const char* kIsActionRunningService = "/robotis/action/is_action_running";
  static inline constexpr const char* kBallFollowerEnableTopic = "/control_bridge/ball_follow_enable";
  static inline constexpr const char* kBallSearchEnableTopic = "/control_bridge/ball_search_enable";
  static inline constexpr const char* kBallDribbleEnableTopic = "/control_bridge/ball_dribble_enable";

  using GetJointModule = robotis_controller_msgs::srv::GetJointModule;
  using SetJointModule = robotis_controller_msgs::srv::SetJointModule;
  using SetCtrlModule = robotis_controller_msgs::srv::SetModule;
  using IsActionRunning = op3_action_module_msgs::srv::IsRunning;

  ControlBridge() : Node("control_bridge_node") {
    present_cache_time_ = this->now();
    // yaml file for joint id <-> name mapping
    const std::string default_joint_map_path =
      ament_index_cpp::get_package_share_directory("op3_control_bridge") +
      "/config/control_bridge_params.yaml";
    const std::string joint_map_path = this->declare_parameter<std::string>(
      "joint_map.config_path", default_joint_map_path);

    // load joint tables
    if (!loadJointTablesFromYAML(joint_map_path)) {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to load joint tables from %s, shutting down node.",
                   joint_map_path.c_str());
      rclcpp::shutdown();
      return;
    }

    // tf buffer/listener for frame transforms
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // service clients
    get_joint_cli_ = create_client<GetJointModule>(kGetJointService);
    set_joint_cli_ = create_client<SetJointModule>(kSetJointService);
    set_ctrl_cli_ = create_client<SetCtrlModule>(kSetCtrlModuleService);
    is_action_running_cli_ = create_client<IsActionRunning>(kIsActionRunningService);

    // cmd_vel pipeline
    // subscribe to /cmd_vel (Twist)
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>
      ("/cmd_vel", 10, std::bind(&ControlBridge::on_cmd_vel, this, std::placeholders::_1));
    // RCLCPP_INFO(this->get_logger(), "control bridge subscribed to /cmd_vel");
    
    // publishers to walking module
    // publish to /robotis/walking/set_params (WalkingParam)
    walking_param_pub_ = create_publisher<op3_walking_module_msgs::msg::WalkingParam>(
      "/robotis/walking/set_params", 10);
    
    walking_command_pub_ = create_publisher<std_msgs::msg::String>(
      "/robotis/walking/command", 10);

    walking_timeout_timer_ = this->create_wall_timer(
      walking_timeout_,
      [this]() {
        std_msgs::msg::String stop_msg;
        stop_msg.data = "stop";
        walking_command_pub_->publish(stop_msg);
        // RCLCPP_INFO(this->get_logger(), "No cmd_vel received: published 'stop' to walking_command.");
        walking_timeout_timer_->cancel(); // Only fire once until next cmd_vel
      }
    );
    walking_timeout_timer_->cancel(); // Don't run until first cmd_vel

    // cmd_action pipeline
    // subscribe to /cmd_action/page_num (std_msgs::msg::Int32) - later republished on /robotis/action/page_num
    cmd_action_sub_ = this->create_subscription<std_msgs::msg::Int32>
      ("/cmd_action/page_num", 10, std::bind(&ControlBridge::on_cmd_action, this, std::placeholders::_1));
    // RCLCPP_INFO(this->get_logger(), "control bridge subscribed to /cmd_action/page_num");

    // local publisher so we can enqueue actions internally (e.g., stand when ball lost)
    cmd_action_pub_ = create_publisher<std_msgs::msg::Int32>("/cmd_action/page_num", 10);

    // publisher to /robotis/action/page_num (std_msgs::msg::Int32)
    action_page_num_pub_ = create_publisher<std_msgs::msg::Int32>(
    "/robotis/action/page_num", 10);

    // startup timer to delay initial debug log
    startup_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(200), [this]() {
        // Wait until the service appears, without blocking.
        if (!get_joint_cli_->service_is_ready()) {
          // RCLCPP_INFO(this->get_logger(), "Waiting for %s ...", kGetJointService);
          return; // keep timer alive; it will check again on the next tick
        }
        // Service is ready: cancel timer and fire the async request
        startup_timer_->cancel();
        this->populatePresentCacheAsync(this->all_joints_);
      }
    );

    // imu data pipeline
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/robotis/open_cr/imu");

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, 10, std::bind(&ControlBridge::onImuMsg, this, std::placeholders::_1));

    // RCLCPP_INFO(get_logger(), "control_bridge subscribed to %s", imu_topic_.c_str());

    // run a one-shot test after a short delay to switch all joints to walking
    // this->scheduleSwitchModulesForJoints(walking_joints_, kWalkingModule);
    // this->ensureModulesForJoints(walking_joints_, kWalkingModule);
 
    // ball tracker (parameters defined in share/op3_control_bridge/config/control_bridge_params.yaml)
    std::string role = declare_parameter<std::string>("gameplay.role", "attacker");
    std::transform(role.begin(), role.end(), role.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });
    role_is_attacker_ = (role != "goalkeeper");

    const double head_max_pan_deg =
      declare_parameter<double>("head_control.max_pan_deg", 70.0);
    const double head_min_tilt_deg =
      declare_parameter<double>("head_control.min_tilt_deg", -35.0);
    const double head_max_tilt_deg =
      declare_parameter<double>("head_control.max_tilt_deg", 20.0);
    const double head_max_upward_tilt_deg =
      declare_parameter<double>("head_control.max_upward_tilt_deg",
                                std::min(10.0, head_max_tilt_deg));
    const double head_pan_slew_deg =
      declare_parameter<double>("head_control.pan_slew_step_deg", 4.0);
    const double head_tilt_slew_deg =
      declare_parameter<double>("head_control.tilt_slew_step_deg", 3.0);
    const double head_max_pan_rad = head_max_pan_deg * DEG2RAD;
    const double head_min_tilt_rad = head_min_tilt_deg * DEG2RAD;
    const double head_max_tilt_rad = head_max_tilt_deg * DEG2RAD;
    goal_peek_tilt_rad_ =
      declare_parameter<double>("head_control.goal_peek_tilt_deg", -1.0) * DEG2RAD;
    goal_peek_interval_sec_ =
      declare_parameter<double>("head_control.goal_peek_interval_sec", 5.0);
    goal_peek_slow_scale_ =
      declare_parameter<double>("head_control.goal_peek_slow_scale", 0.6);
    goal_peek_slow_duration_sec_ =
      declare_parameter<double>("head_control.goal_peek_slow_duration_sec", 0.6);
    goal_peek_slow_scale_ = std::clamp(goal_peek_slow_scale_, 0.1, 1.0);
    goal_peek_tilt_rad_ = std::clamp(goal_peek_tilt_rad_, head_min_tilt_rad, head_max_tilt_rad);
    last_goal_hint_time_ = this->now();
    head_recentre_turn_threshold_rad_ =
      declare_parameter<double>("head_control.recentre_turn_min_deg", 6.0) * DEG2RAD;
    head_recentre_blend_ =
      declare_parameter<double>("head_control.recentre_blend", 0.35);
    head_recentre_blend_ = std::clamp(head_recentre_blend_, 0.0, 1.0);

    BallTrackerConfig tracker_config;
    tracker_config.input_topic = declare_parameter<std::string>(
        "ball_tracker.input_topic", "/perception/ball_cam");
    tracker_config.visibility_topic = declare_parameter<std::string>("ball_tracker.visibility_topic", "/perception/ball/visible");
    tracker_config.scan_command_topic = declare_parameter<std::string>("ball_tracker.scan_command_topic", "/robotis/head_control/scan_command");
    tracker_config.scan_command = declare_parameter<std::string>("ball_tracker.scan_command", "scan");
    tracker_config.p_gain = declare_parameter<double>("ball_tracker.p_gain", 0.45);
    tracker_config.i_gain = declare_parameter<double>("ball_tracker.i_gain", 0.0);
    tracker_config.d_gain = declare_parameter<double>("ball_tracker.d_gain", 0.045);
    tracker_config.integral_limit_rad =
      declare_parameter<double>("ball_tracker.integral_limit_deg", 30.0) * DEG2RAD;
    tracker_config.min_command_rad =
      declare_parameter<double>("ball_tracker.min_command_deg", 1.0) * DEG2RAD;
    tracker_config.deadband_rad =
      declare_parameter<double>("ball_tracker.deadband_deg", 1.0) * DEG2RAD;
    tracker_config.max_pan_rad = head_max_pan_rad;
    tracker_config.max_tilt_rad = head_max_tilt_rad;
    tracker_config.lost_timeout = declare_parameter<double>("ball_tracker.lost_timeout", 0.5);
    tracker_config.scan_interval = declare_parameter<double>("ball_tracker.scan_interval", 8.0);
    tracker_config.use_scan = declare_parameter<bool>("ball_tracker.use_scan", true);
    tracker_config.control_rate_hz = declare_parameter<double>("ball_tracker.control_rate_hz", 30.0);
    tracker_config.wait_cycles = declare_parameter<int>("ball_tracker.wait_cycles", 5);
    tracker_config.pre_scan_cycles = declare_parameter<int>("ball_tracker.pre_scan_cycles", 45);
    particle_hint_timeout_sec_ =
      declare_parameter<double>("ball_tracker.particle_hint_timeout_sec", 2.0);

    tracker_config.head_command_cb = [this](double pan, double tilt) {
      const double adjusted_pan = this->applyHeadRecentering(pan);
      this->publishHeadOffset(adjusted_pan, tilt);
    };
    tracker_config.visibility_cb = [this](bool visible) {
      this->publishBallVisibility(visible);
      if (visible && ball_search_active_)
      {
        this->setBallSearch(false);
      }
    };
    tracker_config.scan_request_cb = [this](const std::string &cmd) -> bool {
      if (cmd == "scan" && head_scan_ && head_scan_->isActive())
      {
        return false;
      }
      this->publishHeadScan(cmd);
      return true;
    };
    tracker_config.scan_in_progress_cb = [this]() -> bool {
      return head_scan_ && head_scan_->isActive();
    };
    tracker_config.state_change_cb = [this](TrackingState state) {
      if (state == TrackingState::Lost)
      {
        if (cmd_action_pub_)
        {
          std_msgs::msg::Int32 msg;
          msg.data = 9;  // walk-ready / stand
          cmd_action_pub_->publish(msg);
        }
      }
    };

    ball_tracker_ = std::make_unique<BallTracker>(this, tracker_config);

    action_page_retry_attempts_ =
      declare_parameter<int>("action.page_retry_count", action_page_retry_attempts_);
    action_page_retry_attempts_ = std::max(0, action_page_retry_attempts_);
    const double action_retry_period =
      declare_parameter<double>("action.page_retry_interval_sec", action_page_retry_interval_sec_);
    action_page_retry_interval_sec_ = std::max(0.01, action_retry_period);
    action_page_kick_low_left_ =
      declare_parameter<int>("action.kick_low_left_page", action_page_kick_low_left_);
    action_page_kick_low_right_ =
      declare_parameter<int>("action.kick_low_right_page", action_page_kick_low_right_);
    action_page_kick_high_left_ =
      declare_parameter<int>("action.kick_high_left_page", action_page_kick_high_left_);
    action_page_kick_high_right_ =
      declare_parameter<int>("action.kick_high_right_page", action_page_kick_high_right_);
    kick_low_left_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "control_bridge/kick_low_left",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        this->handleKickRequest(action_page_kick_low_left_, req, res);
      });

    kick_low_right_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "control_bridge/kick_low_right",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        this->handleKickRequest(action_page_kick_low_right_, req, res);
      });

    kick_high_left_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "control_bridge/kick_high_left",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        this->handleKickRequest(action_page_kick_high_left_, req, res);
      });

    kick_high_right_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "control_bridge/kick_high_right",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        this->handleKickRequest(action_page_kick_high_right_, req, res);
      });

    startup_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      [this]() {
        bool head_ready = haveAllJointsOnModule(kHeadControlModule);
        bool walk_ready = haveAllJointsOnModule(kWalkingModule);
        if (!head_ready)
          requireHeadControl();
        if (!walk_ready)
          requireWalkingControl();
        if (head_ready && walk_ready)
        {
          startup_timer_->cancel();
          startup_timer_.reset();
        }
      });

    const std::string head_offset_topic = declare_parameter<std::string>(
      "head_scan.head_offset_topic", "/robotis/head_control/set_joint_states_offset");
    head_offset_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      head_offset_topic, 10);
    ball_visibility_pub_ = create_publisher<std_msgs::msg::Bool>(
      tracker_config.visibility_topic, 10);

    op3_head_scan::HeadScanConfig head_scan_config;
    head_scan_config.head_offset_topic = head_offset_topic;
  head_scan_config.command_period_sec =
    declare_parameter<double>("head_scan.command_period_sec", 0.50);
  head_scan_config.pan_step_rad =
    declare_parameter<double>("head_scan.pan_step_deg", 24.0) * DEG2RAD;
  head_scan_config.tilt_step_rad =
    declare_parameter<double>("head_scan.tilt_step_deg", 18.0) * DEG2RAD;
  head_scan_config.sweep_step_rad =
    declare_parameter<double>("head_scan.sweep_step_deg", 2.0) * DEG2RAD;
    head_scan_config.max_pan_rad =
      declare_parameter<double>("head_scan.max_pan_deg", head_max_pan_deg) * DEG2RAD;
  const double head_scan_max_tilt_deg =
    declare_parameter<double>("head_scan.max_tilt_deg", 15.0);
  const double head_scan_min_tilt_deg =
    declare_parameter<double>("head_scan.min_tilt_deg", -5.0);
    head_scan_config.max_tilt_rad = head_scan_max_tilt_deg * DEG2RAD;
    head_scan_config.min_tilt_rad = head_scan_min_tilt_deg * DEG2RAD;
    if (head_scan_config.min_tilt_rad > head_scan_config.max_tilt_rad)
      head_scan_config.min_tilt_rad = head_scan_config.max_tilt_rad;
    head_scan_config.max_upward_tilt_rad =
      declare_parameter<double>("head_scan.max_upward_tilt_deg",
                                std::min(head_max_upward_tilt_deg, head_scan_max_tilt_deg)) * DEG2RAD;
    head_scan_config.max_upward_tilt_rad = std::clamp(
      head_scan_config.max_upward_tilt_rad,
      head_scan_config.min_tilt_rad,
      head_scan_config.max_tilt_rad);
  head_scan_config.max_layers =
    declare_parameter<int>("head_scan.layers", 4);
  head_scan_config.repeat =
    declare_parameter<bool>("head_scan.repeat", false);
    head_scan_config.pan_slew_step_rad =
      declare_parameter<double>("head_scan.pan_slew_step_deg", head_pan_slew_deg) * DEG2RAD;
    head_scan_config.tilt_slew_step_rad =
      declare_parameter<double>("head_scan.tilt_slew_step_deg", head_tilt_slew_deg) * DEG2RAD;
  head_scan_config.pan_cells =
    declare_parameter<int>("head_scan.pan_cells", 3);
  head_scan_config.tilt_cells =
    declare_parameter<int>("head_scan.tilt_cells", 3);
  head_scan_config.random_target_count =
    declare_parameter<int>(
      "head_scan.random_target_count",
      std::max(1, head_scan_config.pan_cells * head_scan_config.tilt_cells));
    head_scan_config.command_feedback_cb = [this](double pan, double tilt) {
      last_head_pan_cmd_ = pan;
      last_head_tilt_cmd_ = tilt;
      head_pose_hint_valid_ = true;
    };
    head_scan_max_pan_rad_ = head_scan_config.max_pan_rad;
    head_scan_min_tilt_rad_ = head_scan_config.min_tilt_rad;
    head_scan_max_tilt_rad_ = head_scan_config.max_tilt_rad;
  head_scan_default_tilt_rad_ =
    declare_parameter<double>("head_scan.default_search_tilt_deg", 0.0) * DEG2RAD;
    head_scan_default_tilt_rad_ = std::clamp(
      head_scan_default_tilt_rad_,
      head_scan_config.min_tilt_rad,
      head_scan_config.max_tilt_rad);
    last_head_pan_cmd_ = 0.0;
    last_head_tilt_cmd_ = head_scan_default_tilt_rad_;
    head_scan_ = std::make_unique<op3_head_scan::HeadScan>(this, head_scan_config);

    // Subscribe to particle filter bearing hint for directed ball search
    particle_bearing_sub_ = create_subscription<geometry_msgs::msg::Point>(
      "/perception/ball_search_bearing", 10,
      std::bind(&ControlBridge::onParticleBearing, this, std::placeholders::_1));

    ball_tracker_->enable(false);  // start disabled
    head_track_sub_ = create_subscription<std_msgs::msg::Bool>(
      kBallTrackEnableTopic, 1,
      std::bind(&ControlBridge::onHeadTrackEnable, this, std::placeholders::_1));

    GoalBearingTrackerConfig goal_tracker_config;
    goal_tracker_config.horizontal_fov_rad =
      declare_parameter<double>("goal_tracker.horizontal_fov_deg", 35.2) * DEG2RAD;
    goal_tracker_config.detection_timeout_sec =
      declare_parameter<double>("goal_tracker.detection_timeout_sec", 1.5);
    goal_tracker_config.field_length_m =
      declare_parameter<double>("goal_tracker.field_length_m", 14.0);
    goal_tracker_config.field_width_m =
      declare_parameter<double>("goal_tracker.field_width_m", 9.0);
    goal_tracker_config.goal_width_m =
      declare_parameter<double>("goal_tracker.goal_width_m", 6.0);
    goal_tracker_config.attack_positive_x =
      declare_parameter<bool>("goal_tracker.attack_positive_x", true);
    goal_tracker_ = std::make_unique<GoalBearingTracker>(this->get_clock(), goal_tracker_config);
    const std::string goal_detection_topic =
      declare_parameter<std::string>("goal_tracker.detection_topic", "/perception/goalpost");
    goal_detection_sub_ = create_subscription<op3_vision_msgs::msg::Detection>(
      goal_detection_topic, 10,
      std::bind(&ControlBridge::onGoalDetection, this, std::placeholders::_1));
    // RCLCPP_INFO(this->get_logger(), "Goal tracker listening to %s",
    //             goal_detection_topic.c_str());

    kick_enable_topic_ = declare_parameter<std::string>(
      "ball_kicker.enable_topic", "/control_bridge/ball_kick_enable");
    // Align default kick pages with soccer demo defaults: left=120, right=121.
    kick_default_page_ = declare_parameter<int>("ball_kicker.default_page", 120);
    kick_left_page_ = declare_parameter<int>("ball_kicker.left_page", 120);
    kick_right_page_ = declare_parameter<int>("ball_kicker.right_page", 121);
    kick_use_grass_offset_ = declare_parameter<bool>("ball_kicker.use_grass_offset", true);
    kick_grass_offset_ = declare_parameter<int>("ball_kicker.grass_offset", 20);
    kick_detection_timeout_ = declare_parameter<double>("ball_kicker.detection_timeout_sec", 0.5);
    kick_max_ball_offset_rad_ =
      declare_parameter<double>("ball_kicker.max_ball_offset_deg", 25.0) * DEG2RAD;
    kick_enforce_conditions_ =
      declare_parameter<bool>("ball_kicker.enforce_conditions", true);
    const std::string kick_ball_topic = declare_parameter<std::string>(
      "ball_kicker.ball_topic", "/perception/ball");
    ball_kicker_sub_ = create_subscription<std_msgs::msg::Bool>(
      kick_enable_topic_, 1,
      std::bind(&ControlBridge::onBallKickRequest, this, std::placeholders::_1));
    kick_ball_detection_sub_ = create_subscription<op3_vision_msgs::msg::Detection>(
      kick_ball_topic, 10,
      std::bind(&ControlBridge::onKickBallDetection, this, std::placeholders::_1));

    auto makeGoalBearingSupplier =
      [this](double timeout_sec) -> std::function<std::optional<double>()>
    {
      return [this, timeout_sec]() -> std::optional<double> {
        if (!goal_tracker_)
          return std::nullopt;
        if (!goal_tracker_->hasFreshDetection(this->now(), timeout_sec))
          return std::nullopt;
        return goal_tracker_->relativeBearingRad();
      };
    };

    auto_kick_enabled_ = declare_parameter<bool>("auto_kick.enabled", true);
    auto_kick_goal_tolerance_rad_ =
      declare_parameter<double>("auto_kick.goal_tolerance_deg", 7.0) * DEG2RAD;
    auto_kick_cooldown_sec_ =
      declare_parameter<double>("auto_kick.cooldown_sec", 4.0);
    auto_kick_require_goal_detection_ =
      declare_parameter<bool>("auto_kick.require_goal_detection", false);

    const double ball_walk_nominal_step =
      declare_parameter<double>("ball_walking.nominal_forward_step", 0.02);
    const double ball_walk_max_step =
      declare_parameter<double>("ball_walking.max_forward_step", 0.04);
    const double ball_walk_min_step =
      declare_parameter<double>("ball_walking.min_forward_step", 0.006);
    const double ball_walk_max_turn_deg =
      declare_parameter<double>("ball_walking.max_turn_deg", 15.0);
    const double ball_walk_stop_timeout =
      declare_parameter<double>("ball_walking.stop_timeout_sec", 1.0);
    BallFollowerConfig follower_config;
    follower_config.body_input_topic =
      declare_parameter<std::string>("ball_follower.body_input_topic", "/perception/ball");
    follower_config.camera_input_topic =
      declare_parameter<std::string>("ball_follower.camera_input_topic", "/perception/ball_cam");
    follower_config.body_yaw_weight =
      declare_parameter<double>("ball_follower.body_yaw_weight", 0.5);
    follower_config.nominal_forward_step =
      declare_parameter<double>("ball_follower.nominal_forward_step", ball_walk_nominal_step);
    follower_config.max_forward_step =
      declare_parameter<double>("ball_follower.max_forward_step", ball_walk_max_step);
    follower_config.max_turn_rad =
      declare_parameter<double>("ball_follower.max_turn_deg", ball_walk_max_turn_deg) * DEG2RAD;
    follower_config.stop_timeout_sec =
      declare_parameter<double>("ball_follower.stop_timeout_sec", ball_walk_stop_timeout);
    follower_config.ball_equipped_bbox_min_area =
      declare_parameter<double>("ball_follower.ball_equipped_bbox_min_area", 18000.0);
    follower_config.ball_equipped_max_abs_bearing =
      declare_parameter<double>("ball_follower.ball_equipped_max_abs_bearing", 0.3);
    follower_config.ball_equipped_hold_timeout_sec =
      declare_parameter<double>("ball_follower.ball_equipped_hold_timeout_sec", 0.75);
    follower_config.walking_param_cb = [this](double fb, double turn) {
      this->publishWalkingParams(fb, turn);
    };
    follower_config.walking_command_cb = [this](bool start) {
      this->publishWalkingCommand(start);
    };
    const std::string ball_equipped_topic = declare_parameter<std::string>(
      "ball_follower.ball_equipped_topic", "/control_bridge/ball_equipped");
    ball_equipped_pub_ = create_publisher<std_msgs::msg::Bool>(ball_equipped_topic, 10);
    follower_config.ball_equipped_cb = [this](bool equipped) {
      this->publishBallEquipped(equipped);
    };
    follower_config.head_pose_cb = [this]() -> std::optional<std::pair<double, double>> {
      if (!head_pose_hint_valid_)
      {
        return std::nullopt;
      }
      return std::make_pair(last_head_pan_cmd_, last_head_tilt_cmd_);
    };

    ball_follower_ = std::make_unique<BallFollower>(this, follower_config);
    ball_follower_->enable(false);
    ball_follow_body_detection_sub_ = create_subscription<op3_vision_msgs::msg::Detection>(
      follower_config.body_input_topic, 10,
      std::bind(&ControlBridge::onBallBodyDetection, this, std::placeholders::_1));
    ball_follow_camera_detection_sub_ = create_subscription<op3_vision_msgs::msg::Detection>(
      follower_config.camera_input_topic, 10,
      std::bind(&ControlBridge::onBallCameraDetection, this, std::placeholders::_1));
    ball_follow_sub_ = create_subscription<std_msgs::msg::Bool>(
      kBallFollowerEnableTopic, 1,
      std::bind(&ControlBridge::onBallFollowEnable, this, std::placeholders::_1));

    BallDribblerConfig dribbler_config;
    dribbler_config.input_topic = declare_parameter<std::string>(
      "ball_dribbler.input_topic", follower_config.body_input_topic);
    dribbler_config.body_input_topic = declare_parameter<std::string>(
      "ball_dribbler.body_input_topic", dribbler_config.input_topic);
    dribbler_config.camera_input_topic = declare_parameter<std::string>(
      "ball_dribbler.camera_input_topic", follower_config.camera_input_topic);
    dribbler_config.body_yaw_weight = declare_parameter<double>(
      "ball_dribbler.body_yaw_weight", follower_config.body_yaw_weight);
    dribbler_config.forward_step = declare_parameter<double>(
      "ball_dribbler.forward_step", ball_walk_nominal_step);
    dribbler_config.min_forward_step = declare_parameter<double>(
      "ball_dribbler.min_forward_step", ball_walk_min_step);
    dribbler_config.max_turn_rad = declare_parameter<double>(
      "ball_dribbler.max_turn_deg", ball_walk_max_turn_deg) * DEG2RAD;
    dribbler_config.stop_timeout_sec = declare_parameter<double>(
      "ball_dribbler.stop_timeout_sec", ball_walk_stop_timeout);
    dribbler_config.lock_bbox_min_area = declare_parameter<double>(
      "ball_dribbler.lock_bbox_min_area", 2000.0);
    dribbler_config.close_bbox_area = declare_parameter<double>(
      "ball_dribbler.close_bbox_area", 3200.0);
    dribbler_config.lock_max_abs_bearing = declare_parameter<double>(
      "ball_dribbler.lock_max_abs_bearing", 0.12);
    dribbler_config.walking_param_cb = [this](double fb, double turn) {
      this->publishWalkingParams(fb, turn);
    };
    dribbler_config.walking_command_cb = [this](bool start) {
      this->publishWalkingCommand(start);
    };
    dribbler_config.lost_ball_cb = [this]() {
      this->onDribblerLostBall();
    };
    dribbler_config.goal_alignment_weight =
      declare_parameter<double>("ball_dribbler.goal_alignment_weight", 1.0);
    dribbler_config.goal_alignment_deadband_rad =
      declare_parameter<double>("ball_dribbler.goal_alignment_deadband_deg", 5.0) * DEG2RAD;
    const double goal_alignment_timeout =
      declare_parameter<double>("ball_dribbler.goal_alignment_timeout_sec", 1.5);
    dribbler_config.goal_bearing_cb = makeGoalBearingSupplier(goal_alignment_timeout);
    dribbler_config.goal_alignment_hint_cb = [this]() {
      last_goal_hint_time_ = this->now();
    };

    ball_dribbler_ = std::make_unique<BallDribbler>(this, dribbler_config);
    ball_dribbler_->enable(false);
    ball_dribble_sub_ = create_subscription<std_msgs::msg::Bool>(
      kBallDribbleEnableTopic, 1,
      std::bind(&ControlBridge::onBallDribbleEnable, this, std::placeholders::_1));

    BallSearcherConfig search_config;
    search_config.search_forward_step = declare_parameter<double>("ball_searcher.search_forward_step", 0.015);
    search_config.search_turn_rate =
      declare_parameter<double>("ball_searcher.search_turn_deg", 8.0) * DEG2RAD;
    search_config.command_period_sec =
      declare_parameter<double>("ball_searcher.command_period_sec", 1.0);
    search_config.turn_hold_cycles =
      declare_parameter<int>("ball_searcher.turn_hold_cycles", 4);
    search_config.alternate_turn =
      declare_parameter<bool>("ball_searcher.alternate_turn", true);
    search_config.scan_request_period_sec =
      declare_parameter<double>("ball_searcher.scan_request_period_sec", 3.0);
    search_enter_delay_sec_ =
      declare_parameter<double>("ball_searcher.enter_delay_sec", 1.5);
    search_config.scan_request_cb = [this](const std::string &cmd) {
      this->publishHeadScan(cmd);
    };
    search_config.walking_param_cb = [this](double fb, double turn) {
      this->publishWalkingParams(fb, turn);
    };
    search_config.walking_command_cb = [this](bool start) {
      this->publishWalkingCommand(start);
    };
    ball_searcher_ = std::make_unique<BallSearcher>(this, search_config);
    ball_searcher_->enable(false);
    ball_search_sub_ = create_subscription<std_msgs::msg::Bool>(
      kBallSearchEnableTopic, rclcpp::QoS(1),
      std::bind(&ControlBridge::onBallSearchEnable, this, std::placeholders::_1));

    const std::string gamestate_topic = declare_parameter<std::string>(
      "gamestate.topic", "/gamestate");
    const int gamestate_init_pose_page = declare_parameter<int>(
      "gamestate.init_pose_page", 80);
    const int gamestate_walk_ready_page = declare_parameter<int>(
      "gamestate.walk_ready_page", 9);

    GameStateListener::Config gamestate_config;
    gamestate_config.topic = gamestate_topic;
    gamestate_config.init_pose_page = gamestate_init_pose_page;
    gamestate_config.walk_ready_page = gamestate_walk_ready_page;
    gamestate_config.use_default_transitions = false;
    gamestate_init_pose_page_ = gamestate_init_pose_page;
    gamestate_walk_ready_page_ = gamestate_walk_ready_page;

    GameStateListener::Callbacks gamestate_callbacks;
    gamestate_callbacks.head_tracking_cb = [this](bool enable) {
      this->setHeadTracking(enable);
    };
    gamestate_callbacks.ball_follow_cb = [this](bool enable) {
      this->setBallFollowing(enable);
    };
    gamestate_callbacks.action_page_cb = [this](int page) {
      this->publishActionPage(page);
    };
    gamestate_callbacks.state_update_cb =
      [this](const game_controller_hl_interfaces::msg::GameState &msg) {
        this->onGameStateUpdate(msg);
      };

    gamestate_listener_ = std::make_unique<GameStateListener>(
      this, gamestate_config, gamestate_callbacks);
    // RCLCPP_INFO(this->get_logger(),
    //             "GameState listener active on %s",
    //             gamestate_topic.c_str());

    // RCLCPP_INFO(this->get_logger(), "control_bridge node has started.");

  }

  bool loadJointTablesFromYAML(const std::string& path);
  
  // populate present_ cache by querying get_joint_cli_ with all_joints_
  void populatePresentCacheAsync(const std::vector<std::string>& joint_names = {});

  // Compare present_ vs target; if any differ, call the setter.
  // Pass an empty vector to apply to all_joints_.
  void ensureModulesForJoints(std::vector<std::string> joint_names,
                              const std::string& target_module);

  private:

  // cache the control modules for joints
  std::unordered_map<std::string, std::string> present_; // joint -> current module (cache)
  rclcpp::Time present_cache_time_;
  // rclcpp::Duration(seconds, nanoseconds);
  // For example, rclcpp::Duration(5, 0) sets the duration to 5 seconds.
  rclcpp::Duration present_cache_max_age_{1, 0};
  // protect present_ writes from async callback
  std::mutex present_mtx_;

  // tf helper
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  // timers/timer
  rclcpp::TimerBase::SharedPtr startup_timer_;
  rclcpp::TimerBase::SharedPtr module_switch_timer_;
  rclcpp::TimerBase::SharedPtr walking_timeout_timer_;
  std::chrono::milliseconds walking_timeout_{500};
  struct PendingModuleSwitch
  {
    std::vector<std::string> joints;
    std::string target_module;
  };
  std::deque<PendingModuleSwitch> pending_module_switches_;
  std::mutex module_switch_queue_mtx_;
  rclcpp::TimerBase::SharedPtr head_tracking_resume_timer_;
  std::atomic_bool head_tracking_resume_query_pending_{false};
  bool head_tracking_suspended_for_action_{false};
  bool head_tracking_was_enabled_before_action_{false};
  std::deque<int> pending_action_pages_;
  std::mutex action_page_queue_mtx_;
  rclcpp::TimerBase::SharedPtr action_page_retry_timer_;
  bool action_page_publish_in_progress_{false};
  int action_page_retry_attempts_{2};
  double action_page_retry_interval_sec_{0.05};
  int last_action_page_{-1};
  int pending_action_page_retries_{0};
  rclcpp::TimerBase::SharedPtr action_page_republish_timer_;
  double head_recentre_blend_{0.0};
  double head_recentre_turn_threshold_rad_{0.0};
  std::atomic<double> last_turn_command_rad_{0.0};
  int action_page_kick_low_left_{83};
  int action_page_kick_low_right_{84};
  int action_page_kick_high_left_{120};
  int action_page_kick_high_right_{121};
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr kick_low_left_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr kick_low_right_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr kick_high_left_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr kick_high_right_srv_;
  bool pending_kick_request_{false};
  int pending_kick_page_{0};
  rclcpp::TimerBase::SharedPtr kick_retry_timer_;

  // cmd_vel pipeline
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg); 
  rclcpp::Publisher<op3_walking_module_msgs::msg::WalkingParam>::SharedPtr walking_param_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr walking_command_pub_;

  // cmd_action pipeline
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr cmd_action_sub_;
  void on_cmd_action(const std_msgs::msg::Int32::SharedPtr msg);
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr cmd_action_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr action_page_num_pub_;

  // service clients
  rclcpp::Client<GetJointModule>::SharedPtr get_joint_cli_;
  rclcpp::Client<SetJointModule>::SharedPtr set_joint_cli_;
  rclcpp::Client<SetCtrlModule>::SharedPtr set_ctrl_cli_;
  rclcpp::Client<op3_action_module_msgs::srv::IsRunning>::SharedPtr is_action_running_cli_;

  // id <-> joint name maps
  std::unordered_map<int, std::string> id_joint_table_;
  std::unordered_map<std::string, int> joint_id_table_;

  // joint lists
  // when not using arms don't switch {'walking_module': r_sho_roll l_sho_roll r_el l_el head_pan head_tilt}
  const std::vector<std::string> head_joints_ = {"head_pan", "head_tilt"};
  std::vector<std::string> non_walking_joints_ = {"head_pan", "head_tilt", "r_sho_roll", "l_sho_roll", "r_el", "l_el"};
  std::vector<std::string> walking_joints_;
  std::vector<std::string> all_joints_;

  // pure lookup: which of these joints are NOT on target_module (uses present_)
  std::vector<std::string> mismatchedJoints(const std::vector<std::string>& joint_names,
                                            const std::string& target_module);

  // async setter: set these joints to target_module, and update present_ on success
  void setJointsModuleAsync(std::vector<std::string> joints,
                            const std::string& target_module);

  void scheduleSwitchModulesForJoints(
    const std::vector<std::string>& joint_names,
    const std::string& target_module,
    int delay_ms = 1000);

  // fall detection and recovery
  // flag variables currently not implemented
  bool recovering_{false};           // true while a get-up action is in progress
  bool pending_recovery_{false};     // fall detected, set if an action running when fall detected 
  
  FallDirection fall_direction_{FallDirection::None}; // enum Forward, Backward, None
  
  // imu topic and data
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  void onImuMsg(const sensor_msgs::msg::Imu::SharedPtr msg);
  sensor_msgs::msg::Imu imu_data_;
  std::string imu_topic_;

  void triggerRecovery(FallDirection fall_dir);
  void publishRecovery(const std_msgs::msg::Int32 &msg, FallDirection fall_direction_);
  void startRecoveryResumeWatcher();
  void checkRecoveryComplete();
  void publishHeadOffset(double pan, double tilt);
  void publishBallVisibility(bool visible);
  void publishHeadScan(const std::string &cmd);
  void publishWalkingParams(double fb_move, double rl_turn);
  void publishWalkingCommand(bool start);
  void publishActionPage(int page);
  void enqueueActionPage(int page);
  void processPendingActionPages();
  void ensureActionQueueTimer();
  bool haveAllJointsOnModule(const std::string &module);
  void publishActionPageMessage(int page);
  void scheduleActionPageRepublish();
  void publishActionPageWhenIdle(int page);
  void handleKickRequest(int page,
                         const std::shared_ptr<std_srvs::srv::Trigger::Request> &req,
                         std::shared_ptr<std_srvs::srv::Trigger::Response> res);
  void triggerKickPage(int page);
  void executeKickAction(int page);
  void requireHeadControl();
  void requireWalkingControl();
  void requireActionControl();
  double applyHeadRecentering(double pan) const;
  void publishBallEquipped(bool equipped);
  void evaluateBehaviorState();
  void maybeRequestGoalPeek();
  bool attemptAutoKick(double goal_bearing, bool relaxed = false);
  void suspendHeadTrackingForAction();
  void startHeadTrackingResumeWatcher();
  void checkHeadTrackingResume();
  void onGameStateUpdate(const game_controller_hl_interfaces::msg::GameState &msg);
  void handlePrimaryGameState(uint8_t state);
  void handleSecondaryState(const game_controller_hl_interfaces::msg::GameState &msg);
  void prepareForRecovery();

  // ball tracking
  std::unique_ptr<BallTracker> ball_tracker_;
  void setHeadTracking(bool enable);
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr head_track_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr head_offset_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ball_visibility_pub_;
  std::unique_ptr<op3_head_scan::HeadScan> head_scan_;
  double last_head_pan_cmd_{0.0};
  double last_head_tilt_cmd_{0.0};
  double head_scan_default_tilt_rad_{-20.0 * DEG2RAD};
  double head_scan_max_pan_rad_{80.0 * DEG2RAD};
  double head_scan_min_tilt_rad_{-5.0 * DEG2RAD};
  double head_scan_max_tilt_rad_{5.0 * DEG2RAD};
  bool head_pose_hint_valid_{false};
  void onHeadTrackEnable(const std_msgs::msg::Bool::SharedPtr msg);

  // Particle filter bearing hint from ball_particle_filter_node
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr particle_bearing_sub_;
  double particle_pan_hint_{0.0};
  double particle_tilt_hint_{0.0};
  rclcpp::Time particle_hint_time_{0, 0, RCL_ROS_TIME};
  double particle_hint_timeout_sec_{2.0};  // configurable via ball_tracker.particle_hint_timeout_sec
  void onParticleBearing(const geometry_msgs::msg::Point::SharedPtr msg);

  std::unique_ptr<GoalBearingTracker> goal_tracker_;
  rclcpp::Subscription<op3_vision_msgs::msg::Detection>::SharedPtr goal_detection_sub_;
  void onGoalDetection(const op3_vision_msgs::msg::Detection::SharedPtr msg);

  std::unique_ptr<BallFollower> ball_follower_;
  void setBallFollowing(bool enable);
  void onBallFollowEnable(const std_msgs::msg::Bool::SharedPtr msg);
  void onBallBodyDetection(const op3_vision_msgs::msg::Detection::SharedPtr msg);
  void onBallCameraDetection(const op3_vision_msgs::msg::Detection::SharedPtr msg);
  rclcpp::Subscription<op3_vision_msgs::msg::Detection>::SharedPtr ball_follow_body_detection_sub_;
  rclcpp::Subscription<op3_vision_msgs::msg::Detection>::SharedPtr ball_follow_camera_detection_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ball_follow_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ball_equipped_pub_;
  bool last_ball_equipped_{false};
  bool ball_equipped_state_{false};
  double goal_peek_interval_sec_{0.0};
  double goal_peek_tilt_rad_{0.0};
  double goal_peek_slow_scale_{1.0};
  double goal_peek_slow_duration_sec_{0.0};
  rclcpp::Time last_goal_hint_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_goal_peek_request_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time goal_peek_slow_until_{0, 0, RCL_ROS_TIME};
  bool auto_kick_enabled_{false};
  double auto_kick_goal_tolerance_rad_{0.0};
  double auto_kick_cooldown_sec_{3.0};
  bool auto_kick_require_goal_detection_{false};
  rclcpp::Time last_auto_kick_time_{0, 0, RCL_ROS_TIME};
  bool ball_visible_state_{false};
  rclcpp::Time last_ball_visible_time_{0, 0, RCL_ROS_TIME};
  std::unique_ptr<BallDribbler> ball_dribbler_;
  void setBallDribble(bool enable);
  void onBallDribbleEnable(const std_msgs::msg::Bool::SharedPtr msg);
  void onDribblerLostBall();
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ball_dribble_sub_;

  std::unique_ptr<BallSearcher> ball_searcher_;
  void setBallSearch(bool enable);
  void onBallSearchEnable(const std_msgs::msg::Bool::SharedPtr msg);
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ball_search_sub_;
  bool ball_search_active_{false};
  bool resume_head_tracking_{false};
  bool resume_ball_following_{false};
  bool resume_ball_dribble_{false};
  std::unique_ptr<GameStateListener> gamestate_listener_;
  int gamestate_init_pose_page_{80};
  int gamestate_walk_ready_page_{9};
  bool have_game_state_{false};
  game_controller_hl_interfaces::msg::GameState latest_game_state_{};
  uint8_t current_game_state_{game_controller_hl_interfaces::msg::GameState::GAMESTATE_INITIAL};
  bool allow_play_actions_{false};
  bool in_secondary_phase_{false};
  uint8_t active_secondary_state_{game_controller_hl_interfaces::msg::GameState::STATE_NORMAL};
  uint8_t active_secondary_mode_{game_controller_hl_interfaces::msg::GameState::MODE_PREPARATION};
  bool role_is_attacker_{true};
  bool kickoff_bias_active_{false};
  rclcpp::TimerBase::SharedPtr recovery_retry_timer_;
  FallDirection pending_recovery_direction_{FallDirection::None};
  double search_enter_delay_sec_{1.0};

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ball_kicker_sub_;
  std::string kick_enable_topic_;
  int kick_default_page_{83};
  int kick_left_page_{83};
  int kick_right_page_{-1};
  bool kick_use_grass_offset_{false};
  int kick_grass_offset_{20};
  double kick_detection_timeout_{0.5};
  double kick_max_ball_offset_rad_{25.0 * DEG2RAD};
  bool kick_enforce_conditions_{true};
  bool have_kick_ball_detection_{false};
  double last_kick_ball_bearing_{0.0};
  rclcpp::Time last_kick_ball_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<op3_vision_msgs::msg::Detection>::SharedPtr kick_ball_detection_sub_;
  void onBallKickRequest(const std_msgs::msg::Bool::SharedPtr msg);
  void onKickBallDetection(const op3_vision_msgs::msg::Detection::SharedPtr msg);

};

bool ControlBridge::loadJointTablesFromYAML(const std::string& path) {
  try {
    YAML::Node config = YAML::LoadFile(path);
    YAML::Node joint_map = config["control_bridge"];
    if (joint_map) {
      joint_map = joint_map["ros__parameters"];
    }
    if (joint_map) {
      joint_map = joint_map["joint_map"];
    }
    if (!joint_map) {
      joint_map = config["joint_map"];
    }
    YAML::Node map = joint_map ? joint_map["id_joint"] : YAML::Node();
    if (!map || !map.IsMap()) {
      RCLCPP_ERROR(this->get_logger(),
                   "'id_joint' field is missing or not a map in %s",
                   path.c_str());
      return false;
    }
    id_joint_table_.clear();
    joint_id_table_.clear();
    all_joints_.clear();
    walking_joints_.clear();
    for (auto it = map.begin(); it != map.end(); ++it) {
      int id = it->first.as<int>();
      std::string name = it->second.as<std::string>();
      if (name == "opencr") continue;  // not a joint, skip
      id_joint_table_[id] = name;
      joint_id_table_[name] = id;
      all_joints_.push_back(name);
      if (std::find(non_walking_joints_.begin(), non_walking_joints_.end(), name) == non_walking_joints_.end()) {
        walking_joints_.push_back(name);
      }
      RCLCPP_DEBUG(this->get_logger(), "  Added joint: id=%d, name='%s'", id, name.c_str());
    }
    // RCLCPP_INFO(this->get_logger(), "Loaded %zu joints from %s", id_joint_table_.size(), path.c_str());
    // checking contents of walking joints
    
    /*RCLCPP_DEBUG(this->get_logger(), "walking_joints:");
      for (const auto& joint : walking_joints_) {
        RCLCPP_DEBUG(this->get_logger(), "  %s", joint.c_str());
      }*/
    return true;
  } catch (const YAML::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load YAML file %s: %s", path.c_str(), e.what());
    return false;
  }
}

  void ControlBridge::on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg) {
    // RCLCPP_INFO(this->get_logger(), 
    //             "Twist  lin[x y z]=[%.3f %.3f %.3f]  ang[x y z]=[%.3f %.3f %.3f]",
    //             msg->linear.x, msg->linear.y, msg->linear.z,
    //             msg->angular.x, msg->angular.y, msg->angular.z);

    const auto nontrivial = 
      std::fabs(msg->linear.x) > 1e-6 || std::fabs(msg->linear.y) > 1e-6 ||
      std::fabs(msg->linear.z) > 1e-6 || std::fabs(msg->angular.x) > 1e-6 ||
      std::fabs(msg->angular.y) > 1e-6 || std::fabs(msg->angular.z) > 1e-6;
    
    // assume walking module is already active
    // publish "stop" command to walking module
    std_msgs::msg::String command_msg;
    command_msg.data = "stop";

    if (nontrivial) {
      // check and switch joints for walking
      this->requireWalkingControl();

      constexpr double DEG_TO_RAD = M_PI / 180.0;
      constexpr double LINEAR_SCALE = 0.1;

      double lin_x = msg->linear.x;
      double lin_y = msg->linear.y;
      //double lin_z = msg->linear.z;
      double ang_z = msg->angular.z;

      op3_walking_module_msgs::msg::WalkingParam param_msg;

      param_msg.x_move_amplitude = lin_x * LINEAR_SCALE;
      param_msg.y_move_amplitude = lin_y * LINEAR_SCALE;
      // param_msg.z_move_amplitude = lin_z * LINEAR_SCALE;
      param_msg.z_move_amplitude = 0.06; // constant for now
      param_msg.angle_move_amplitude = ang_z;

      param_msg.init_x_offset = -0.020;
      param_msg.init_y_offset = 0.015;
      param_msg.init_z_offset = 0.035;
      param_msg.init_roll_offset = 0.0;
      param_msg.init_pitch_offset = 0.0;
      param_msg.init_yaw_offset = 0.0 ;
      param_msg.hip_pitch_offset = 5.0 * DEG_TO_RAD;

      param_msg.period_time = 0.650;
      param_msg.dsp_ratio = 0.20;
      param_msg.step_fb_ratio = 0.28;

      param_msg.move_aim_on = false;
      param_msg.balance_enable = true;
      param_msg.balance_hip_roll_gain = 0.35;
      param_msg.balance_knee_gain = 0.30;
      param_msg.balance_ankle_roll_gain = 0.70;
      param_msg.balance_ankle_pitch_gain = 0.90;

      param_msg.y_swap_amplitude = 0.028;
      param_msg.z_swap_amplitude = 0.006;
      param_msg.arm_swing_gain = 0.20;
      param_msg.pelvis_offset = 0.0;

      param_msg.p_gain = 0;
      param_msg.i_gain = 0;
      param_msg.d_gain = 0;

      // publish params to walking module
      walking_param_pub_->publish(param_msg);
      command_msg.data = "start";

      // reset/restart the timeout timer
      // if no new cmd_vel arrives before it fires, we will publish "stop"
      walking_timeout_timer_->reset();
    }

    walking_command_pub_->publish(command_msg);

  }

void ControlBridge::on_cmd_action(const std_msgs::msg::Int32::SharedPtr msg) {
  // RCLCPP_INFO(get_logger(), "ControlBridge::on_cmd_action received page %d", msg->data);
  publishActionPage(msg->data);
}

void ControlBridge::populatePresentCacheAsync(const std::vector<std::string>& joint_names) {
  std::vector<std::string> query_joints = joint_names.empty() ? all_joints_ : joint_names;
  auto req = std::make_shared<GetJointModule::Request>();
  req->joint_name = query_joints;

  // RCLCPP_INFO(get_logger(), "populatePresentAsync: querying %zu joint(s)...", query_joints.size());

  get_joint_cli_->async_send_request(
    req,
    [this](rclcpp::Client<GetJointModule>::SharedFuture future) {
      try {
        auto resp = future.get();
        const auto &names = resp->joint_name;
        const auto &mods  = resp->module_name;

        if (names.size() != mods.size()) {
          RCLCPP_ERROR(this->get_logger(),
                       "Response size mismatch: joint_name=%zu, module_name=%zu",
                       names.size(), mods.size());
          return;
        }
        std::scoped_lock lk(this->present_mtx_);

        std::unordered_map<std::string, size_t> module_counts;
        for (size_t i = 0; i < names.size(); ++i) {
          present_[names[i]] = mods[i];
          module_counts[mods[i]]++;
          // RCLCPP_DEBUG(this->get_logger(), "  %s on %s",
          //             names[i].c_str(), mods[i].c_str());
        }
        present_cache_time_ = this->now();

        if (!module_counts.empty()) {
          std::ostringstream summary;
          bool first = true;
          for (const auto &entry : module_counts) {
            if (!first) {
              summary << ", ";
            }
            summary << entry.first << ":" << entry.second;
            first = false;
          }
          /*
          RCLCPP_DEBUG(this->get_logger(),
                       "populatePresentAsync cached %zu joints (%s)",
                       names.size(), summary.str().c_str());*/
        }
      } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(),
                     "GetJointModule async callback error: %s", e.what());
      }
    }
  );
}

std::vector<std::string> ControlBridge::mismatchedJoints(
    const std::vector<std::string>& joint_names,
    const std::string& target_module)
{
  std::vector<std::string> names = joint_names;
  if (names.empty())
    names = all_joints_;

  std::vector<std::string> out;
  std::scoped_lock lk(present_mtx_);
  // check whether cached joint modules has expired
  auto now = this->now();
  if (( now - present_cache_time_) > present_cache_max_age_) {
    // RCLCPP_INFO(this->get_logger(), "repopulating joint control module cache");
    this ->populatePresentCacheAsync();
  }
  out.reserve(names.size());
  for (const auto& j : names) {
    auto it = present_.find(j);
    // If we don't have it in cache yet, or it's different -> needs change
    if (it == present_.end() || it->second != target_module) {
      out.push_back(j);
    }
  }
  return out;
}

void ControlBridge::setJointsModuleAsync(
    std::vector<std::string> joints,
    const std::string& target_module)
{
  if (joints.empty()) return;

  if (!set_joint_cli_->service_is_ready()) {
    scheduleSwitchModulesForJoints(joints, target_module);
    return; 
  }

  auto req = std::make_shared<SetJointModule::Request>();
  req->joint_name  = joints;
  req->module_name = std::vector<std::string>(joints.size(), target_module);

  // RCLCPP_INFO(get_logger(), "Switching %zu joint(s) to '%s'...",
  //             joints.size(), target_module.c_str());

  // Capture by value so we know exactly which joints we changed
  set_joint_cli_->async_send_request(
    req,
    [this, joints, target_module](rclcpp::Client<SetJointModule>::SharedFuture fut) {
      try {
        auto resp = fut.get();
        if (!resp->result) {
          RCLCPP_ERROR(this->get_logger(),
                       "SetJointModule returned result=false for %zu joint(s).", joints.size());
          return;
        }

        // Update present_ cache on success
        {
          std::scoped_lock lk(this->present_mtx_);
          for (const auto& j : joints) {
            present_[j] = target_module;
          }
        }
        // RCLCPP_INFO(this->get_logger(),
        //             "Updated %zu joint(s) to '%s'.", joints.size(), target_module.c_str());
      } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "SetJointModule async callback error: %s", e.what());
      }
    }
  );
}

// Helper function to schedule a one-shot switch of joints to a module after a short delayo
void ControlBridge::scheduleSwitchModulesForJoints(
    const std::vector<std::string>& joint_names,
    const std::string& target_module,
    int delay_ms)
{
  {
    std::scoped_lock lk(module_switch_queue_mtx_);
    pending_module_switches_.push_back(PendingModuleSwitch{joint_names, target_module});
  }

  if (module_switch_timer_)
    return;

  auto interval = std::chrono::milliseconds(delay_ms);
  module_switch_timer_ = this->create_wall_timer(
    interval,
    [this]() {
      PendingModuleSwitch pending;
      {
        std::scoped_lock lk(module_switch_queue_mtx_);
        if (pending_module_switches_.empty())
        {
          module_switch_timer_->cancel();
          module_switch_timer_.reset();
          return;
        }
        pending = std::move(pending_module_switches_.front());
        pending_module_switches_.pop_front();
      }

      if (!set_joint_cli_->service_is_ready())
      {
        std::scoped_lock lk(module_switch_queue_mtx_);
        pending_module_switches_.push_front(std::move(pending));
        return;
      }
      this->ensureModulesForJoints(pending.joints, pending.target_module);
    });
}

void ControlBridge::ensureModulesForJoints(
    std::vector<std::string> joint_names,
    const std::string& target_module)
{
  // Empty means "use all joints"
  if (joint_names.empty()) joint_names = all_joints_;

  // diff  desired module against present_
  auto need = mismatchedJoints(joint_names, target_module);

  /*
  // Debug log: show output of mismatched joints (the ones that need switching)
  std::string need_str;
  for (const auto& j : need) {
    need_str += j + " ";
  }
  RCLCPP_DEBUG(get_logger(), "Joints needing switch to '%s': %s",
               target_module.c_str(), need_str.c_str());
  */

  if (need.empty()) {
    // RCLCPP_DEBUG(get_logger(), "All %zu joint(s) already on '%s'. No changes.",
    //              joint_names.size(), target_module.c_str());
    return;
  }

  //  use async only if joints need changing
  setJointsModuleAsync(std::move(need), target_module);
}

void ControlBridge::onImuMsg(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  imu_data_ = *msg;
  fall_direction_ = getFallDirection(imu_data_);
  if (fall_direction_ != FallDirection::None)
  {
    RCLCPP_DEBUG(get_logger(), "Calling triggerRecovery(%s)", to_string(fall_direction_));
    triggerRecovery(fall_direction_);
  }

  if (goal_tracker_)
  {
    const double yaw = getImuYaw(imu_data_);
    rclcpp::Time stamp = msg->header.stamp;
    if (stamp.nanoseconds() == 0)
      stamp = this->now();
    goal_tracker_->updateImuYaw(yaw, stamp);
  }
  /*
  RCLCPP_DEBUG(get_logger(),
              "IMU data received. Fall: %s  Orientation: x=%.4f, y=%.4f, z=%.4f, w=%.4f",
              to_string(fall_direction_),
              msg->orientation.x,
              msg->orientation.y,
              msg->orientation.z,
              msg->orientation.w);
  */
}

void ControlBridge::publishRecovery(const std_msgs::msg::Int32 &msg, FallDirection fall_direction_)
{
  (void)fall_direction_;
  requireActionControl();
  action_page_num_pub_->publish(msg);
  recovering_ = true;
  pending_recovery_ = false;
  pending_recovery_direction_ = FallDirection::None;
  // RCLCPP_DEBUG(get_logger(),
  //             "Triggering recovery action %d (%s)",
  //             msg.data, to_string(fall_direction_));
}

void ControlBridge::publishHeadOffset(double pan, double tilt)
{
  if (!head_offset_pub_)
    return;

  if (head_scan_ && head_scan_->isActive())
    head_scan_->cancel();

  sensor_msgs::msg::JointState cmd;
  cmd.header.stamp = this->now();
  cmd.name = {"head_pan", "head_tilt"};
  cmd.position = {pan, tilt};
  head_offset_pub_->publish(cmd);

  last_head_pan_cmd_ = pan;
  last_head_tilt_cmd_ = tilt;
  head_pose_hint_valid_ = true;
}

void ControlBridge::publishBallVisibility(bool visible)
{
  if (!ball_visibility_pub_)
    return;

  if (visible && head_scan_)
  {
    head_scan_->cancel();
  }

  std_msgs::msg::Bool msg;
  msg.data = visible;
  ball_visibility_pub_->publish(msg);

  if (visible)
  {
    last_ball_visible_time_ = this->now();
  }

  if (ball_visible_state_ != visible)
  {
    ball_visible_state_ = visible;
    evaluateBehaviorState();
  }
}

void ControlBridge::onParticleBearing(const geometry_msgs::msg::Point::SharedPtr msg)
{
  particle_pan_hint_  = msg->x;
  particle_tilt_hint_ = msg->y;
  particle_hint_time_ = this->get_clock()->now();
}

void ControlBridge::publishHeadScan(const std::string &cmd)
{
  if (!head_scan_)
  {
    RCLCPP_WARN(get_logger(),
                "Head scan component unavailable; ignoring command '%s'",
                cmd.c_str());
    return;
  }

  requireHeadControl();

  if (cmd == "stop")
  {
    if (head_scan_->isActive())
    {
      // RCLCPP_INFO(get_logger(), "Stopping adaptive head scan.");
      head_scan_->cancel();
    }
    return;
  }

  // Use particle filter bearing hint when fresh, otherwise fall back to default.
  // Freshness window is configurable via ball_tracker.particle_hint_timeout_sec (default 2 s).
  const bool hint_fresh = particle_hint_time_.nanoseconds() > 0 &&
      (this->get_clock()->now() - particle_hint_time_).seconds() < particle_hint_timeout_sec_;
  const double pan_hint  = hint_fresh ? particle_pan_hint_  : head_scan_->getConfig().start_pan_rad;
  const double tilt_hint = hint_fresh ? particle_tilt_hint_ : head_scan_->getConfig().start_tilt_rad;
  if (hint_fresh)
    RCLCPP_INFO(get_logger(), "Head scan using particle hint: pan=%.1f° tilt=%.1f°",
                pan_hint * 180.0 / M_PI, tilt_hint * 180.0 / M_PI);

  auto clamp_with_margin = [](double value, double min_lim, double max_lim, double margin) {
    const double low = min_lim + margin;
    const double high = max_lim - margin;
    if (low > high) return std::clamp(value, min_lim, max_lim);
    return std::clamp(value, low, high);
  };

  constexpr double PAN_MARGIN_RAD = 5.0 * DEG2RAD;
  constexpr double TILT_MARGIN_RAD = 2.0 * DEG2RAD;
  double clamped_pan = clamp_with_margin(
      pan_hint,
      -head_scan_max_pan_rad_,
      head_scan_max_pan_rad_,
      PAN_MARGIN_RAD);
  double clamped_tilt = clamp_with_margin(
      tilt_hint,
      head_scan_min_tilt_rad_,
      head_scan_max_tilt_rad_,
      TILT_MARGIN_RAD);

  // RCLCPP_INFO(get_logger(),
  //             "Starting adaptive head scan from pan=%.1fdeg tilt=%.1fdeg",
  //             clamped_pan * 180.0 / M_PI,
  //             clamped_tilt * 180.0 / M_PI);
  head_scan_->start(clamped_pan, clamped_tilt);
}

void ControlBridge::publishWalkingParams(double fb_move, double rl_turn)
{
  if (!walking_param_pub_)
    return;

  last_turn_command_rad_.store(rl_turn, std::memory_order_relaxed);

  if (goal_peek_slow_scale_ < 1.0 && goal_peek_slow_duration_sec_ > 0.0)
  {
    const auto now = this->now();
    if (goal_peek_slow_until_.nanoseconds() != 0 &&
        now < goal_peek_slow_until_)
    {
      fb_move *= goal_peek_slow_scale_;
    }
  }

  op3_walking_module_msgs::msg::WalkingParam param;
  param.init_x_offset = -0.020;
  param.init_y_offset = 0.015;
  param.init_z_offset = 0.035;
  param.init_roll_offset = 0.0;
  param.init_pitch_offset = 0.0;
  param.init_yaw_offset = 0.0;
  param.hip_pitch_offset = 5.0 * DEG2RAD;
  param.period_time = 0.650;
  param.dsp_ratio = 0.20;
  param.step_fb_ratio = 0.28;
  param.z_move_amplitude = 0.06;
  param.y_swap_amplitude = 0.028;
  param.z_swap_amplitude = 0.006;
  param.arm_swing_gain = 0.20;
  param.pelvis_offset = 0.0;
  param.balance_enable = true;
  param.x_move_amplitude = fb_move;
  param.y_move_amplitude = 0.0;
  param.angle_move_amplitude = rl_turn;

  walking_param_pub_->publish(param);
}

void ControlBridge::publishWalkingCommand(bool start)
{
  if (!walking_command_pub_)
    return;

  std_msgs::msg::String msg;
  msg.data = start ? "start" : "stop";
  walking_command_pub_->publish(msg);
}

void ControlBridge::publishActionPage(int page)
{
  if (!action_page_num_pub_)
  {
    RCLCPP_WARN(get_logger(),
                "Action page publisher not ready; ignoring request for page %d",
                page);
    return;
  }

  ensureModulesForJoints({}, kActionModule);
  if (!haveAllJointsOnModule(kActionModule))
  {
    enqueueActionPage(page);
    return;
  }

  publishActionPageMessage(page);
}

void ControlBridge::enqueueActionPage(int page)
{
  {
    std::scoped_lock lk(action_page_queue_mtx_);
    pending_action_pages_.push_back(page);
  }

  ensureActionQueueTimer();
}

void ControlBridge::ensureActionQueueTimer()
{
  if (action_page_retry_timer_)
    return;

  action_page_retry_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),
    [this]() {
      this->processPendingActionPages();
    });
}

void ControlBridge::processPendingActionPages()
{
  int page = 0;
  {
    std::scoped_lock lk(action_page_queue_mtx_);
    if (action_page_publish_in_progress_)
      return;
    if (pending_action_pages_.empty())
    {
      if (action_page_retry_timer_)
      {
        action_page_retry_timer_->cancel();
        action_page_retry_timer_.reset();
      }
      return;
    }
    if (!haveAllJointsOnModule(kActionModule))
    {
      return;
    }
    page = pending_action_pages_.front();
    pending_action_pages_.pop_front();
    if (pending_action_pages_.empty())
    {
      action_page_retry_timer_->cancel();
      action_page_retry_timer_.reset();
    }
    action_page_publish_in_progress_ = true;
  }

  publishActionPageWhenIdle(page);
}

bool ControlBridge::haveAllJointsOnModule(const std::string &module)
{
  return mismatchedJoints({}, module).empty();
}

void ControlBridge::requireHeadControl()
{
  ensureModulesForJoints(head_joints_, kHeadControlModule);
}

void ControlBridge::requireWalkingControl()
{
  ensureModulesForJoints(walking_joints_, kWalkingModule);
  ensureModulesForJoints(head_joints_, kHeadControlModule);
}

void ControlBridge::requireActionControl()
{
  ensureModulesForJoints({}, kActionModule);
}

void ControlBridge::publishActionPageMessage(int page)
{
  requireActionControl();
  std_msgs::msg::Int32 msg;
  msg.data = page;
  action_page_num_pub_->publish(msg);
  last_action_page_ = page;
  pending_action_page_retries_ = action_page_retry_attempts_;
  scheduleActionPageRepublish();
}

void ControlBridge::publishActionPageWhenIdle(int page)
{
  if (!is_action_running_cli_ || !is_action_running_cli_->service_is_ready())
  {
    requireActionControl();
    publishActionPageMessage(page);
    {
      std::scoped_lock lk(action_page_queue_mtx_);
      action_page_publish_in_progress_ = false;
    }
    return;
  }

  auto req = std::make_shared<IsActionRunning::Request>();
  is_action_running_cli_->async_send_request(
    req,
    [this, page](rclcpp::Client<IsActionRunning>::SharedFuture fut) {
      bool busy = false;
      try
      {
        busy = fut.get()->is_running;
      }
      catch (const std::exception &e)
      {
        RCLCPP_ERROR(get_logger(), "IsActionRunning check failed: %s", e.what());
        busy = false;
      }

      if (!busy)
      {
        requireActionControl();
        publishActionPageMessage(page);
      }
      else
      {
        std::scoped_lock lk(action_page_queue_mtx_);
        pending_action_pages_.push_front(page);
        ensureActionQueueTimer();
      }

      bool has_pending = false;
      {
        std::scoped_lock lk(action_page_queue_mtx_);
        action_page_publish_in_progress_ = false;
        has_pending = !pending_action_pages_.empty();
      }

      if (has_pending)
        ensureActionQueueTimer();
    });
}

void ControlBridge::scheduleActionPageRepublish()
{
  if (pending_action_page_retries_ <= 0)
  {
    if (action_page_republish_timer_)
    {
      action_page_republish_timer_->cancel();
      action_page_republish_timer_.reset();
    }
    return;
  }

  if (!action_page_republish_timer_)
  {
    auto period = std::chrono::duration<double>(action_page_retry_interval_sec_);
    action_page_republish_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      [this]() {
        if (pending_action_page_retries_ <= 0)
        {
          action_page_republish_timer_->cancel();
          action_page_republish_timer_.reset();
          return;
        }
        if (!haveAllJointsOnModule(kActionModule))
          return;
        std_msgs::msg::Int32 msg;
        msg.data = last_action_page_;
        action_page_num_pub_->publish(msg);
        pending_action_page_retries_--;
        if (pending_action_page_retries_ <= 0)
        {
          action_page_republish_timer_->cancel();
          action_page_republish_timer_.reset();
        }
      });
  }
}

void ControlBridge::handleKickRequest(
    int page,
    const std::shared_ptr<std_srvs::srv::Trigger::Request> &/*req*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  triggerKickPage(page);
  res->success = true;
  res->message = "Kick page requested: " + std::to_string(page);
}

void ControlBridge::triggerKickPage(int page)
{
  if (page <= 0)
    return;

  if (!is_action_running_cli_ || !is_action_running_cli_->service_is_ready())
  {
    executeKickAction(page);
    return;
  }

  auto req = std::make_shared<IsActionRunning::Request>();
  is_action_running_cli_->async_send_request(
    req,
    [this, page](rclcpp::Client<IsActionRunning>::SharedFuture fut) {
      bool running = false;
      try
      {
        running = fut.get()->is_running;
      }
      catch (const std::exception &e)
      {
        RCLCPP_ERROR(get_logger(), "IsActionRunning call failed: %s", e.what());
        running = false;
      }

      if (!running)
      {
        executeKickAction(page);
        return;
      }

      pending_kick_request_ = true;
      pending_kick_page_ = page;
      if (!kick_retry_timer_)
      {
        kick_retry_timer_ = this->create_wall_timer(
          std::chrono::milliseconds(250),
          [this]() {
            if (!pending_kick_request_)
            {
              if (kick_retry_timer_)
              {
                kick_retry_timer_->cancel();
                kick_retry_timer_.reset();
              }
              return;
            }
            pending_kick_request_ = false;
            this->triggerKickPage(pending_kick_page_);
          });
      }
    });
}

void ControlBridge::executeKickAction(int page)
{
  pending_kick_request_ = false;
  if (kick_retry_timer_)
  {
    kick_retry_timer_->cancel();
    kick_retry_timer_.reset();
  }

  suspendHeadTrackingForAction();
  setBallDribble(false);
  setBallFollowing(false);
  setBallSearch(false);
  goal_peek_slow_until_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  requireActionControl();
  publishActionPage(page);
}

double ControlBridge::applyHeadRecentering(double pan) const
{
  if (head_recentre_blend_ <= 0.0)
    return pan;
  const double turn = last_turn_command_rad_.load(std::memory_order_relaxed);
  if (std::fabs(turn) < head_recentre_turn_threshold_rad_)
    return pan;
  if (pan * turn >= 0.0)
    return pan;
  const double blend = std::clamp(head_recentre_blend_, 0.0, 1.0);
  return pan * (1.0 - blend);
}

void ControlBridge::publishBallEquipped(bool equipped)
{
  if (!ball_equipped_pub_)
    return;
  if (last_ball_equipped_ == equipped)
    return;

  std_msgs::msg::Bool msg;
  msg.data = equipped;
  ball_equipped_pub_->publish(msg);
  last_ball_equipped_ = equipped;
  ball_equipped_state_ = equipped;

  /* 
  TODO: revisit this code
  */
  if (equipped)
  {
    // Ball underfoot: stop looking around, stop follower, and bring up dribbler.
    setHeadTracking(false);
    if (ball_search_active_)
      setBallSearch(false);
    if (ball_follower_ && ball_follower_->isEnabled())
      // setBallFollowing(false);
    if (ball_dribbler_ && !ball_dribbler_->isEnabled())
      setBallDribble(true);
  }
  else
  {
    // Ball not equipped; ensure head tracking resumes for follow/search.
    setHeadTracking(true);
  }
  evaluateBehaviorState();
  // RCLCPP_INFO(get_logger(), "Ball equipped state: %s", equipped ? "true" : "false");
}

void ControlBridge::evaluateBehaviorState()
{
  // No automatic gating; explicit controls only.
  (void)ball_searcher_;
  (void)ball_follower_;
  (void)ball_dribbler_;
}

void ControlBridge::maybeRequestGoalPeek()
{
  if (goal_peek_interval_sec_ <= 0.0)
    return;
  const auto now = this->now();
  if (goal_tracker_ && goal_tracker_->hasFreshDetection(now, goal_peek_interval_sec_))
    return;
  if (last_goal_peek_request_time_.nanoseconds() != 0 &&
      (now - last_goal_peek_request_time_).seconds() < goal_peek_interval_sec_)
    return;
  publishHeadOffset(last_head_pan_cmd_, goal_peek_tilt_rad_);
  last_goal_peek_request_time_ = now;
  if (goal_peek_slow_duration_sec_ > 0.0)
  {
    auto duration = rclcpp::Duration::from_seconds(goal_peek_slow_duration_sec_);
    goal_peek_slow_until_ = now + duration;
  }
}

void ControlBridge::suspendHeadTrackingForAction()
{
  if (!ball_tracker_)
    return;

  head_tracking_was_enabled_before_action_ = ball_tracker_->isEnabled();
  if (!head_tracking_was_enabled_before_action_)
  {
    head_tracking_suspended_for_action_ = false;
    return;
  }

  ball_tracker_->enable(false);
  publishHeadScan("stop");
  if (head_scan_ && head_scan_->isActive())
    head_scan_->cancel();
  head_pose_hint_valid_ = false;
  head_tracking_suspended_for_action_ = true;
  startHeadTrackingResumeWatcher();
}

void ControlBridge::startHeadTrackingResumeWatcher()
{
  if (head_tracking_resume_timer_)
    head_tracking_resume_timer_->cancel();

  head_tracking_resume_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(250),
    [this]() {
      this->checkHeadTrackingResume();
    });
}

void ControlBridge::checkHeadTrackingResume()
{
  if (!head_tracking_suspended_for_action_)
  {
    if (head_tracking_resume_timer_)
    {
      head_tracking_resume_timer_->cancel();
      head_tracking_resume_timer_.reset();
    }
    head_tracking_resume_query_pending_ = false;
    return;
  }

  if (!is_action_running_cli_ || !is_action_running_cli_->service_is_ready())
    return;
  if (head_tracking_resume_query_pending_)
    return;

  head_tracking_resume_query_pending_ = true;
  auto req = std::make_shared<IsActionRunning::Request>();
  is_action_running_cli_->async_send_request(
    req,
    [this](rclcpp::Client<IsActionRunning>::SharedFuture fut) {
      head_tracking_resume_query_pending_ = false;
      try
      {
        const auto running = fut.get()->is_running;
        if (!running)
        {
          head_tracking_suspended_for_action_ = false;
          if (head_tracking_was_enabled_before_action_)
          {
            setHeadTracking(true);
          }
          head_tracking_was_enabled_before_action_ = false;
          if (head_tracking_resume_timer_)
          {
            head_tracking_resume_timer_->cancel();
            head_tracking_resume_timer_.reset();
          }
          evaluateBehaviorState();
        }
      }
      catch (const std::exception &e)
      {
        RCLCPP_ERROR(get_logger(), "IsActionRunning check failed: %s", e.what());
      }
    });
}

bool ControlBridge::attemptAutoKick(double goal_bearing, bool relaxed)
{
  if (!auto_kick_enabled_ || !allow_play_actions_)
    return false;
  const auto now = this->now();
  if (last_auto_kick_time_.nanoseconds() != 0 &&
      (now - last_auto_kick_time_).seconds() < auto_kick_cooldown_sec_)
    return false;
  if (!relaxed && std::abs(goal_bearing) > auto_kick_goal_tolerance_rad_)
    return false;
  int page = (goal_bearing >= 0.0) ? kick_left_page_ : kick_right_page_;
  if (page <= 0)
    page = kick_default_page_;
  if (page <= 0)
    return false;
  suspendHeadTrackingForAction();
  triggerKickPage(page);
  last_auto_kick_time_ = now;
  kickoff_bias_active_ = false;
  return true;
}

void ControlBridge::onGameStateUpdate(const game_controller_hl_interfaces::msg::GameState &msg)
{
  bool state_changed = !have_game_state_ || msg.game_state != current_game_state_;
  latest_game_state_ = msg;
  have_game_state_ = true;
  if (!role_is_attacker_)
    kickoff_bias_active_ = false;
  else
    kickoff_bias_active_ = msg.has_kick_off;

  if (state_changed)
  {
    current_game_state_ = msg.game_state;
    handlePrimaryGameState(msg.game_state);
  }

  handleSecondaryState(msg);
}

void ControlBridge::handlePrimaryGameState(uint8_t state)
{
  using GameStateMsg = game_controller_hl_interfaces::msg::GameState;
  allow_play_actions_ = false;
  kickoff_bias_active_ = kickoff_bias_active_ && role_is_attacker_;
  switch (state)
  {
    case GameStateMsg::GAMESTATE_INITIAL:
      setHeadTracking(false);
      setBallFollowing(false);
      break;
    case GameStateMsg::GAMESTATE_READY:
      setHeadTracking(false);
      setBallFollowing(false);
      break;
    case GameStateMsg::GAMESTATE_SET:
      setHeadTracking(true);
      setBallFollowing(false);
      break;
    case GameStateMsg::GAMESTATE_PLAYING:
      setHeadTracking(true);
      allow_play_actions_ = role_is_attacker_;
      setBallFollowing(true);
      break;
    case GameStateMsg::GAMESTATE_FINISHED:
    default:
      setHeadTracking(false);
      setBallFollowing(false);
      break;
  }
}

void ControlBridge::handleSecondaryState(const game_controller_hl_interfaces::msg::GameState &msg)
{
  using GameStateMsg = game_controller_hl_interfaces::msg::GameState;
  const bool active = msg.secondary_state != GameStateMsg::STATE_NORMAL;
  const bool changed = (active != in_secondary_phase_) ||
                       (msg.secondary_state != active_secondary_state_) ||
                       (msg.secondary_state_mode != active_secondary_mode_);
  if (!changed)
    return;

  in_secondary_phase_ = active;
  active_secondary_state_ = msg.secondary_state;
  active_secondary_mode_ = msg.secondary_state_mode;

  if (!active)
  {
    evaluateBehaviorState();
    return;
  }

  setBallDribble(false);
  setBallFollowing(false);
  setBallSearch(false);
  publishWalkingCommand(false);

  switch (msg.secondary_state)
  {
    case GameStateMsg::STATE_THROW_IN:
    case GameStateMsg::STATE_DIRECT_FREEKICK:
    case GameStateMsg::STATE_INDIRECT_FREEKICK:
    case GameStateMsg::STATE_CORNER_KICK:
    case GameStateMsg::STATE_GOAL_KICK:
    case GameStateMsg::STATE_PENALTYKICK:
      setHeadTracking(true);
      if (msg.secondary_state_mode == GameStateMsg::MODE_PLACING)
      {
        publishHeadOffset(last_head_pan_cmd_, goal_peek_tilt_rad_);
      }
      break;
    default:
      break;
  }
}

void ControlBridge::setBallFollowing(bool enable)
{
  if (!ball_follower_)
    return;

  if (enable && ball_search_active_)
  {
    // RCLCPP_INFO(get_logger(), "Disabling ball search before enabling ball follower.");
    setBallSearch(false);
  }
  if (enable && ball_dribbler_ && ball_dribbler_->isEnabled())
  {
    // RCLCPP_INFO(get_logger(), "Disabling ball dribbler before enabling ball follower.");
    setBallDribble(false);
  }

  if (enable)
  {
    requireWalkingControl();
  }
  else
    publishWalkingCommand(false);

  ball_follower_->enable(enable);
}

void ControlBridge::onBallFollowEnable(const std_msgs::msg::Bool::SharedPtr msg)
{
  setBallFollowing(msg->data);
}

void ControlBridge::onBallBodyDetection(const op3_vision_msgs::msg::Detection::SharedPtr msg)
{
  if (ball_follower_)
    ball_follower_->handleBodyDetection(msg);
}

void ControlBridge::onBallCameraDetection(const op3_vision_msgs::msg::Detection::SharedPtr msg)
{
  if (ball_follower_)
    ball_follower_->handleCameraDetection(msg);
}

void ControlBridge::setBallDribble(bool enable)
{
  if (!ball_dribbler_)
    return;

  if (enable && ball_search_active_)
  {
    // RCLCPP_INFO(get_logger(), "Disabling ball search before enabling ball dribbler.");
    setBallSearch(false);
  }
  if (enable && ball_follower_ && ball_follower_->isEnabled())
  {
    // RCLCPP_INFO(get_logger(), "Disabling ball follower before enabling ball dribbler.");
    setBallFollowing(false);
  }

  if (enable)
    requireWalkingControl();

  ball_dribbler_->enable(enable);
}

void ControlBridge::onBallDribbleEnable(const std_msgs::msg::Bool::SharedPtr msg)
{
  setBallDribble(msg->data);
}

void ControlBridge::onKickBallDetection(const op3_vision_msgs::msg::Detection::SharedPtr msg)
{
  if (!msg)
    return;
  last_kick_ball_time_ = this->now();
  last_kick_ball_bearing_ = msg->bearing.x;
  have_kick_ball_detection_ = true;
}

void ControlBridge::onBallKickRequest(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (!msg->data)
    return;

  const auto now = this->now();
  double bearing_rad = last_kick_ball_bearing_;
  bool have_valid_detection =
    have_kick_ball_detection_ &&
    (now - last_kick_ball_time_).seconds() <= kick_detection_timeout_;

  // Prefer goal bearing if available to choose kick side.
  bool have_goal_bearing = false;
  double goal_bearing_rad = 0.0;
  const double goal_timeout = std::max(0.5, kick_detection_timeout_);
  if (goal_tracker_ && goal_tracker_->hasFreshDetection(now, goal_timeout))
  {
    goal_bearing_rad = goal_tracker_->relativeBearingRad();
    have_goal_bearing = true;
  }

  int page = kick_right_page_ > 0 ? kick_right_page_ : kick_default_page_;
  if (have_goal_bearing)
  {
    if (goal_bearing_rad >= 0.0 && kick_left_page_ > 0)
    {
      page = kick_left_page_;
    }
    else if (goal_bearing_rad < 0.0 && kick_right_page_ > 0)
    {
      page = kick_right_page_;
    }
  }
  else if (have_valid_detection)
  {
    if (bearing_rad >= 0.0 && kick_left_page_ > 0)
    {
      page = kick_left_page_;
    }
    else if (bearing_rad < 0.0 && kick_right_page_ > 0)
    {
      page = kick_right_page_;
    }
  }

  // Apply grass offset only for the dedicated left/right kick pages.
  if (kick_use_grass_offset_ &&
      (page == kick_left_page_ || page == kick_right_page_))
  {
    page += kick_grass_offset_;
  }

  setBallDribble(false);
  setBallFollowing(false);
  setBallSearch(false);
  publishWalkingCommand(false);
  suspendHeadTrackingForAction();

  if (page <= 0)
  {
    RCLCPP_WARN(get_logger(), "Ball kicker action page not configured.");
    return;
  }

  // One-shot publish to /cmd_action/page_num with no gating/retry.
  if (cmd_action_pub_)
  {
    std_msgs::msg::Int32 kick_msg;
    kick_msg.data = page;
    cmd_action_pub_->publish(kick_msg);
    RCLCPP_INFO(get_logger(), "Published kick action page %d", page);
  }
}

void ControlBridge::onGoalDetection(const op3_vision_msgs::msg::Detection::SharedPtr msg)
{
  if (!goal_tracker_ || !msg)
    return;
  goal_tracker_->updateDetection(*msg, this->now());
}

void ControlBridge::onDribblerLostBall()
{
  RCLCPP_WARN(get_logger(),
              "BallDribbler lost ball, switching to search.");
  // Ball is no longer under control; clear equipped flag so follower can resume.
  publishBallEquipped(false);
  setHeadTracking(true);
  setBallDribble(false);
  setBallSearch(true);
}

void ControlBridge::setBallSearch(bool enable)
{
  if (!ball_searcher_)
    return;

  if (enable)
  {
    ball_search_active_ = true;
    setBallFollowing(false);
    setBallDribble(false);
    requireWalkingControl();
    publishHeadScan("scan");
  }
  else
  {
    ball_search_active_ = false;
    publishHeadScan("stop");
  }

  ball_searcher_->enable(enable);
}

void ControlBridge::onBallSearchEnable(const std_msgs::msg::Bool::SharedPtr msg)
{
  // RCLCPP_INFO(get_logger(), "Ball search enable request: %s", msg->data ? "true" : "false");
  setBallSearch(msg->data);
}

void ControlBridge::triggerRecovery(FallDirection fall_direction)
{
  if (fall_direction == FallDirection::None)
    return;

  // Guard: do not interrupt a recovery that is already in progress.
  if (recovering_)
    return;

  // Select get-up page: 122 = face-down (forward fall), 123 = face-up (backward fall)
  std_msgs::msg::Int32 recovery_msg;
  recovery_msg.data = (fall_direction == FallDirection::Forward) ? 122 : 123;

  RCLCPP_WARN(get_logger(), "Fall detected (%s) – executing recovery page %d",
              to_string(fall_direction), recovery_msg.data);

  // Suspend active behaviours so they do not interfere during recovery.
  prepareForRecovery();
  suspendHeadTrackingForAction();
  publishWalkingCommand(false);

  if (!is_action_running_cli_->service_is_ready()) {
    publishRecovery(recovery_msg, fall_direction);
    startRecoveryResumeWatcher();
    return;
  }

  auto req = std::make_shared<IsActionRunning::Request>();
  is_action_running_cli_->async_send_request(
    req,
    [this, recovery_msg, fall_direction](rclcpp::Client<IsActionRunning>::SharedFuture fut) {
      try {
        const auto action_is_running = fut.get()->is_running;
        if (action_is_running) {
          RCLCPP_WARN(get_logger(), "Action module busy; issuing recovery anyway.");
        }
        publishRecovery(recovery_msg, fall_direction);
        startRecoveryResumeWatcher();
      } catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(), "IsRunning call failed: %s", e.what());
      }
    });
}

void ControlBridge::setHeadTracking(bool enable)
{
  if (!ball_tracker_)
    return;
  ball_tracker_->enable(enable);
  if (enable)
  {
    requireHeadControl();
    // Keep the current head pose; don't command a new offset when enabling tracking.
  }
  else
  {
    publishHeadScan("stop");
    if (head_scan_ && head_scan_->isActive())
      head_scan_->cancel();
    head_pose_hint_valid_ = false;
  }
}

void ControlBridge::onHeadTrackEnable(const std_msgs::msg::Bool::SharedPtr msg)
{
  setHeadTracking(msg->data);
}

void ControlBridge::startRecoveryResumeWatcher()
{
  if (recovery_retry_timer_)
    recovery_retry_timer_->cancel();

  recovery_retry_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(250),
    [this]() { this->checkRecoveryComplete(); });
}

void ControlBridge::checkRecoveryComplete()
{
  if (!recovering_) {
    if (recovery_retry_timer_) {
      recovery_retry_timer_->cancel();
      recovery_retry_timer_.reset();
    }
    return;
  }

  if (!is_action_running_cli_ || !is_action_running_cli_->service_is_ready())
    return;

  auto req = std::make_shared<IsActionRunning::Request>();
  is_action_running_cli_->async_send_request(
    req,
    [this](rclcpp::Client<IsActionRunning>::SharedFuture fut) {
      try {
        const bool running = fut.get()->is_running;
        if (!running) {
          recovering_ = false;
          pending_recovery_ = false;

          if (recovery_retry_timer_) {
            recovery_retry_timer_->cancel();
            recovery_retry_timer_.reset();
          }

          // Restore the walking module so the robot can walk again.
          requireWalkingControl();

          // Resume whichever body behaviours were active before the fall.
          if (resume_ball_following_) {
            setBallFollowing(true);
          }
          if (resume_ball_dribble_) {
            setBallDribble(true);
          }
          resume_ball_following_ = false;
          resume_ball_dribble_ = false;

          RCLCPP_INFO(get_logger(),
                      "Fall recovery complete – resuming normal operation.");
          evaluateBehaviorState();
        }
      } catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(),
                     "Recovery: IsActionRunning check failed: %s", e.what());
      }
    });
}

void ControlBridge::prepareForRecovery()
{
  if (recovering_)
    return;

  resume_head_tracking_ = ball_tracker_ && ball_tracker_->isEnabled();
  resume_ball_following_ = ball_follower_ && ball_follower_->isEnabled();
  resume_ball_dribble_ = ball_dribbler_ && ball_dribbler_->isEnabled();

  if (resume_head_tracking_)
    setHeadTracking(false);
  if (resume_ball_following_)
    setBallFollowing(false);
  if (resume_ball_dribble_)
    setBallDribble(false);
}

}  // namespace stride::op3

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<stride::op3::ControlBridge>();

 
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
