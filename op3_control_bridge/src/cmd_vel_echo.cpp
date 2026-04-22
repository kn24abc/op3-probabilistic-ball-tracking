// cmd_vel_echo.cpp
// simply echoes twist messages on /cmd_vel to [INFO]

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"


class CmdVelEcho: public rclcpp::Node 
{
public:
  CmdVelEcho(): Node("cmd_vel_echo")
  {
    sub_ = this->create_subscription<geometry_msgs::msg::Twist>
      ("/cmd_vel", 10, std::bind(&CmdVelEcho::on_cmd_vel, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "cmd_vel_echo node has started.");
  }

private:
  void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), 
                "Twist  lin[x y z]=[%.3f %.3f %.3f]  ang[x y z]=[%.3f %.3f %.3f]",
                msg->linear.x, msg->linear.y, msg->linear.z,
                msg->angular.x, msg->angular.y, msg->angular.z);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;

};


int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<CmdVelEcho>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
