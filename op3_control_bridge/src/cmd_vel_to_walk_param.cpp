// cmd_vel_to_walk_paramcpp
// converts twist messages on /cmd_vel to walking_param messages
// subscribes to /cmd_vel
// publishes to /robotis/walking/set_params

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "op3_walking_module_msgs/msg/walking_param.hpp"


class CmdVelToWalkParam: public rclcpp::Node 
{
public:
  CmdVelToWalkParam(): Node("cmd_vel_walk_param")
  {
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>
      ("/cmd_vel", 10, std::bind(&CmdVelToWalkParam::on_cmd_vel, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "cmd_vel_echo node has started.");

    walking_param_pub_ = create_publisher<op3_walking_module_msgs::msg::WalkingParam>(
      "/robotis/walking/set_params", 10);
    walking_command_pub_ = create_publisher<std_msgs::msg::String>(
      "/robotis/walking/command", 10);
  }


private:
  void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), 
                "Twist  lin[x y z]=[%.3f %.3f %.3f]  ang[x y z]=[%.3f %.3f %.3f]",
                msg->linear.x, msg->linear.y, msg->linear.z,
                msg->angular.x, msg->angular.y, msg->angular.z);

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
  param_msg.balance_enable = false;
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

  walking_param_pub_->publish(param_msg);

  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<op3_walking_module_msgs::msg::WalkingParam>::SharedPtr walking_param_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr walking_command_pub_;

};


int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<CmdVelToWalkParam>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
