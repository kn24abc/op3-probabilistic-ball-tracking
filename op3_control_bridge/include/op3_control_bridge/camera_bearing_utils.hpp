#pragma once

#include <optional>
#include <string>

#include <Eigen/Geometry>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "op3_vision_msgs/msg/detection.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace stride::op3
{

inline std::optional<double> cameraBearingToBaseYaw(
    const op3_vision_msgs::msg::Detection &msg,
    double horizontal_fov_rad,
    double vertical_fov_rad,
    const std::function<std::optional<geometry_msgs::msg::TransformStamped>(const std::string &, const std::string &)> &tf_lookup_cb,
    const std::string &target_frame,
    rclcpp::Logger logger,
    rclcpp::Clock &clock)
{
  if (!tf_lookup_cb || target_frame.empty() || msg.header.frame_id.empty())
    return std::nullopt;

  const double yaw_cam = msg.bearing.x * horizontal_fov_rad;
  const double pitch_cam = msg.bearing.y * vertical_fov_rad;
  const double cos_pitch = std::cos(pitch_cam);
  Eigen::Vector3d ray_cam(
      cos_pitch * std::sin(yaw_cam),
      -std::sin(pitch_cam),
      cos_pitch * std::cos(yaw_cam));

  geometry_msgs::msg::Vector3Stamped cam_vec;
  cam_vec.header = msg.header;
  cam_vec.vector.x = ray_cam.x();
  cam_vec.vector.y = ray_cam.y();
  cam_vec.vector.z = ray_cam.z();

  auto tf_opt = tf_lookup_cb(target_frame, msg.header.frame_id);
  if (!tf_opt)
  {
    RCLCPP_WARN_THROTTLE(logger, clock, 2000,
                         "TF lookup failed %s -> %s",
                         target_frame.c_str(), msg.header.frame_id.c_str());
    return std::nullopt;
  }

  geometry_msgs::msg::Vector3Stamped base_vec;
  tf2::doTransform(cam_vec, base_vec, *tf_opt);
  // RCLCPP_DEBUG(logger,
  //              "Transformed ray %s -> %s : x=%.3f y=%.3f z=%.3f",
  //              msg.header.frame_id.c_str(),
  //              target_frame.c_str(),
  //              base_vec.vector.x,
  //              base_vec.vector.y,
  //              base_vec.vector.z);
  return std::atan2(base_vec.vector.y, base_vec.vector.x);
}

}  // namespace stride::op3
