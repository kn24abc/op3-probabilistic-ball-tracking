// ball_filter_node.cpp
// filters raw detections down to a single "best" ball target with temporal smoothing

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "op3_perception/bearing_transformer.hpp"
#include "op3_vision_msgs/msg/vision.hpp"

class BallFilterNode : public rclcpp::Node
{
public:
  BallFilterNode()
  : Node("ball_filter_node"),
    bearing_transformer_(this, "bearing_transform", "body_link")
  {
    camera_frame_ = this->declare_parameter<std::string>("cam_link", "");
    ball_class_id_ = this->declare_parameter<std::string>("ball_class_id", "ball");
    min_confidence_ = this->declare_parameter<double>("min_confidence", 0.5);
    smoothing_alpha_ = this->declare_parameter<double>("smoothing_alpha", 0.5);
    double sustain_time = this->declare_parameter<double>("sustain_time", 0.8);
    double heartbeat_period = this->declare_parameter<double>("heartbeat_period", 0.1);
    std::string input_topic = this->declare_parameter<std::string>("input_topic", "/camera/detections");
    std::string output_topic = this->declare_parameter<std::string>("output_topic", "/perception/ball");
    std::string camera_output_topic = this->declare_parameter<std::string>(
        "camera_output_topic", "/perception/ball_cam");

    min_confidence_ = std::clamp(min_confidence_, 0.0, 1.0);
    smoothing_alpha_ = std::clamp(smoothing_alpha_, 0.0, 1.0);
    if (sustain_time < 0.0)
      sustain_time = 0.0;
    if (heartbeat_period <= 0.0)
      heartbeat_period = 0.1;

    sustain_duration_ = rclcpp::Duration::from_seconds(sustain_time);

    detections_sub_ = this->create_subscription<op3_vision_msgs::msg::Vision>(
        input_topic, rclcpp::SensorDataQoS(),
        std::bind(&BallFilterNode::onDetectionsMsg, this, std::placeholders::_1));

    ball_pub_ = this->create_publisher<op3_vision_msgs::msg::Detection>(output_topic, 10);
    camera_ball_pub_ = this->create_publisher<op3_vision_msgs::msg::Detection>(
        camera_output_topic, 10);

    heartbeat_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(heartbeat_period),
        std::bind(&BallFilterNode::republishIfFresh, this));

    RCLCPP_INFO(this->get_logger(), "Ball filter node ready (input: %s, outputs: %s, %s)",
                input_topic.c_str(), output_topic.c_str(), camera_output_topic.c_str());
  }

private:
  void onDetectionsMsg(const op3_vision_msgs::msg::Vision::SharedPtr msg)
  {
    const op3_vision_msgs::msg::Detection *best_detection = nullptr;
    const vision_msgs::msg::ObjectHypothesisWithPose *best_hypothesis = nullptr;
    double best_area = 0.0;

    for (const auto &detection : msg->detections)
    {
      for (const auto &result : detection.results)
      {
        if (result.hypothesis.class_id != ball_class_id_)
          continue;

        if (result.hypothesis.score < min_confidence_)
          continue;

        const double area = detection.bbox.size_x * detection.bbox.size_y;
        if (best_detection == nullptr || area > best_area)
        {
          best_detection = &detection;
          best_hypothesis = &result;
          best_area = area;
        }
      }
    }

    if (best_detection == nullptr || best_hypothesis == nullptr)
      return;

    auto filtered = *best_detection;
    filtered.header = msg->header;
    // Fallback if detections arrive without a frame_id.
    if (filtered.header.frame_id.empty() && !camera_frame_.empty())
      filtered.header.frame_id = camera_frame_;

    if (have_detection_)
    {
      auto smooth = [this](double previous, double current) {
        return previous + smoothing_alpha_ * (current - previous);
      };

      filtered.bearing.x = smooth(last_detection_.bearing.x, filtered.bearing.x);
      filtered.bearing.y = smooth(last_detection_.bearing.y, filtered.bearing.y);
      filtered.bbox.center.position.x = smooth(last_detection_.bbox.center.position.x, filtered.bbox.center.position.x);
      filtered.bbox.center.position.y = smooth(last_detection_.bbox.center.position.y, filtered.bbox.center.position.y);
    }

    last_detection_ = filtered;
    have_detection_ = true;
    last_detection_time_ = this->get_clock()->now();

    auto camera_detection = filtered;
    if (bearing_transformer_.toCameraAngles(camera_detection))
    {
      last_camera_detection_ = camera_detection;
      have_camera_detection_ = true;
      camera_ball_pub_->publish(last_camera_detection_);
    }

    auto transformed = filtered;
    if (bearing_transformer_.transform(transformed))
    {
      last_body_detection_ = transformed;
      have_body_detection_ = true;
      ball_pub_->publish(transformed);
    }

    RCLCPP_DEBUG(this->get_logger(), "Best ball score %.2f area %.3f", best_hypothesis->hypothesis.score, best_area);
  }

  void republishIfFresh()
  {
    if ((this->get_clock()->now() - last_detection_time_) > sustain_duration_)
      return;

    if (have_camera_detection_)
      camera_ball_pub_->publish(last_camera_detection_);

    if (have_body_detection_)
      ball_pub_->publish(last_body_detection_);
  }

  std::string ball_class_id_;
  double min_confidence_{};
  double smoothing_alpha_{};
  rclcpp::Duration sustain_duration_{0, 0};
  std::string camera_frame_;

  rclcpp::Subscription<op3_vision_msgs::msg::Vision>::SharedPtr detections_sub_;
  rclcpp::Publisher<op3_vision_msgs::msg::Detection>::SharedPtr ball_pub_;
  rclcpp::Publisher<op3_vision_msgs::msg::Detection>::SharedPtr camera_ball_pub_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;

  op3_vision_msgs::msg::Detection last_detection_;
  op3_vision_msgs::msg::Detection last_camera_detection_;
  op3_vision_msgs::msg::Detection last_body_detection_;
  rclcpp::Time last_detection_time_{0, 0, RCL_ROS_TIME};
  bool have_detection_{false};
  bool have_camera_detection_{false};
  bool have_body_detection_{false};
  op3_perception::BearingTransformer bearing_transformer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BallFilterNode>());
  rclcpp::shutdown();
  return 0;
}
