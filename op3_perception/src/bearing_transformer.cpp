#include "op3_perception/bearing_transformer.hpp"

#include <cmath>

#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace op3_perception
{

namespace
{

constexpr double kDefaultHorizontalFovDeg = 35.2;
constexpr double kDefaultVerticalFovDeg = 21.6;

}  // namespace

BearingTransformer::BearingTransformer(
    rclcpp::Node * node,
    const std::string & param_namespace,
    const std::string & default_target_frame)
: logger_(node ? node->get_logger()
               : rclcpp::get_logger("BearingTransformer")),
  clock_(node ? node->get_clock() : nullptr),
  param_namespace_(param_namespace)
{
  if (!node)
  {
    RCLCPP_WARN(logger_, "BearingTransformer constructed without node context; disabled.");
    return;
  }

  const auto target_key = parameter_prefix("target_frame");
  const auto horiz_key = parameter_prefix("horizontal_fov_deg");
  const auto vert_key = parameter_prefix("vertical_fov_deg");

  target_frame_ = node->declare_parameter<std::string>(
      target_key, default_target_frame);
  const double horizontal_fov_deg =
      node->declare_parameter<double>(horiz_key, kDefaultHorizontalFovDeg);
  const double vertical_fov_deg =
      node->declare_parameter<double>(vert_key, kDefaultVerticalFovDeg);

  horizontal_fov_rad_ = horizontal_fov_deg * (M_PI / 180.0);
  vertical_fov_rad_ = vertical_fov_deg * (M_PI / 180.0);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  enabled_ = !target_frame_.empty();
  if (!enabled_)
  {
    RCLCPP_INFO(
        logger_,
        "Bearing transform disabled (%s parameter empty).",
        target_key.c_str());
  }
}

std::string BearingTransformer::parameter_prefix(const std::string & name) const
{
  if (param_namespace_.empty())
    return name;
  if (param_namespace_.back() == '.')
    return param_namespace_ + name;
  return param_namespace_ + "." + name;
}

bool BearingTransformer::convertToCameraAngles(
    op3_vision_msgs::msg::Detection & detection,
    double & yaw_cam,
    double & pitch_cam,
    bool log_conversion)
{
  const double normalized_x = detection.bearing.x;
  const double normalized_y = detection.bearing.y;
  if (!std::isfinite(detection.bearing.x) ||
      !std::isfinite(detection.bearing.y))
    return false;

  // Camera image is mirrored, so positive normalized X (=right in image)
  // corresponds to a negative yaw in optical space (right turn).
  yaw_cam = -normalized_x * horizontal_fov_rad_;
  pitch_cam = normalized_y * vertical_fov_rad_;
  detection.bearing.x = yaw_cam;
  detection.bearing.y = pitch_cam;
  if (log_conversion)
  {
    RCLCPP_DEBUG(
        logger_,
        "BearingTransformer normalized=(%.4f, %.4f) -> cam_rad=(%.4f, %.4f)",
        normalized_x, normalized_y, yaw_cam, pitch_cam);
  }
  return true;
}

bool BearingTransformer::toCameraAngles(op3_vision_msgs::msg::Detection & detection)
{
  double yaw_cam = 0.0;
  double pitch_cam = 0.0;
  return convertToCameraAngles(detection, yaw_cam, pitch_cam, true);
}

bool BearingTransformer::transform(op3_vision_msgs::msg::Detection & detection)
{
  double yaw_cam = 0.0;
  double pitch_cam = 0.0;
  if (!convertToCameraAngles(detection, yaw_cam, pitch_cam, false))
    return false;

  RCLCPP_DEBUG(
      logger_,
      "BearingTransformer normalized converted -> cam_rad=(%.4f, %.4f)",
      yaw_cam, pitch_cam);

  if (!enabled_ || !tf_buffer_)
    return false;
  if (detection.header.frame_id.empty())
    return false;

  geometry_msgs::msg::Vector3Stamped cam_vec;
  cam_vec.header = detection.header;
  const double cos_pitch = std::cos(pitch_cam);
  cam_vec.vector.x = cos_pitch * std::sin(yaw_cam); // x right
  cam_vec.vector.y = std::sin(pitch_cam); // y down
  cam_vec.vector.z = std::cos(yaw_cam) * cos_pitch; // z forward

  try
  {
    const auto tf = tf_buffer_->lookupTransform(
        target_frame_, detection.header.frame_id, tf2::TimePointZero);
    geometry_msgs::msg::Vector3Stamped base_vec;
    tf2::doTransform(cam_vec, base_vec, tf);
    const double yaw = std::atan2(base_vec.vector.y, base_vec.vector.x);
    const double xy_norm = std::hypot(base_vec.vector.x, base_vec.vector.y);
    const double pitch = std::atan2(-base_vec.vector.z, xy_norm);
    detection.bearing.x = yaw;
    detection.bearing.y = pitch;
    detection.header.frame_id = target_frame_;
    RCLCPP_DEBUG(
        logger_,
        "BearingTransformer cam_rad=(%.4f, %.4f) -> %s=(%.4f, %.4f)",
        yaw_cam, pitch_cam, target_frame_.c_str(), yaw, pitch);
    return true;
  }
  catch (const tf2::TransformException & ex)
  {
    if (clock_)
    {
      RCLCPP_WARN_THROTTLE(
          logger_, *clock_, 2000,
          "Bearing transform %s -> %s failed: %s",
          detection.header.frame_id.c_str(),
          target_frame_.c_str(),
          ex.what());
    }
    else
    {
      RCLCPP_WARN(
          logger_,
          "Bearing transform %s -> %s failed: %s",
          detection.header.frame_id.c_str(),
          target_frame_.c_str(),
          ex.what());
    }
    return false;
  }
}

}  // namespace op3_perception
