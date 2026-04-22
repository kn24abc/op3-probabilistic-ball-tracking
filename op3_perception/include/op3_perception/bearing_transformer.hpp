#pragma once

#include <memory>
#include <string>

#include "op3_vision_msgs/msg/detection.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/node.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace op3_perception
{

class BearingTransformer
{
public:
  explicit BearingTransformer(
      rclcpp::Node * node,
      const std::string & param_namespace = "bearing_transform",
      const std::string & default_target_frame = "");

  bool toCameraAngles(op3_vision_msgs::msg::Detection & detection);
  bool transform(op3_vision_msgs::msg::Detection & detection);
  bool enabled() const { return enabled_; }

private:
  std::string parameter_prefix(const std::string & name) const;
  bool convertToCameraAngles(
      op3_vision_msgs::msg::Detection & detection,
      double & yaw_cam,
      double & pitch_cam,
      bool log_conversion);

  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::string target_frame_;
  double horizontal_fov_rad_{0.0};
  double vertical_fov_rad_{0.0};
  bool enabled_{false};
  std::string param_namespace_;
};

}  // namespace op3_perception
