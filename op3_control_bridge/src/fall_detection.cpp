// fall_detection.cpp
// ros2 run op3_control_bridge fall_detector_node --ros-args -p imu_topic:=/robotis_op3/imu

#include <string>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <eigen3/Eigen/Geometry>
#include <robotis_math/robotis_linear_algebra.h>

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

class FallDetector : public rclcpp::Node
{
public:
  FallDetector()
  : rclcpp::Node("fall_detector_node")
  {
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/robotis/open_cr/imu");

    sub_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, 10, std::bind(&FallDetector::onImuMsg, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "fall_detector_node has started, watching %s", imu_topic_.c_str());
  }

private:
  void onImuMsg(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    imu_data_ = *msg;
    fall_direction_ = getFallDirection(imu_data_);

    RCLCPP_INFO(get_logger(),
                "IMU data received. Fall: %s  Orientation: x=%.4f, y=%.4f, z=%.4f, w=%.4f",
                to_string(fall_direction_),
                msg->orientation.x,
                msg->orientation.y,
                msg->orientation.z,
                msg->orientation.w);
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
  sensor_msgs::msg::Imu imu_data_;
  std::string imu_topic_;
  FallDirection fall_direction_{FallDirection::None};
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FallDetector>());
  rclcpp::shutdown();
  return 0;
}
