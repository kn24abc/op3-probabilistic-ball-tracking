// goalpost_filter_node.cpp
// reduces raw detections to a single stabilized goalpost target

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "op3_perception/bearing_transformer.hpp"
#include "op3_vision_msgs/msg/vision.hpp"

class GoalpostFilterNode : public rclcpp::Node
{
public:
  GoalpostFilterNode()
  : Node("goalpost_filter_node"),
    bearing_transformer_(this)
  {
    goal_class_id_ = this->declare_parameter<std::string>("goal_class_id", "goalpost");
    min_confidence_ = this->declare_parameter<double>("min_confidence", 0.3);
    min_height_ = this->declare_parameter<double>("min_bbox_height", 0.02);
    smoothing_alpha_ = this->declare_parameter<double>("smoothing_alpha", 0.5);
    double sustain_time = this->declare_parameter<double>("sustain_time", 0.6);
    double heartbeat_period = this->declare_parameter<double>("heartbeat_period", 0.15);
    vote_window_sec_ = this->declare_parameter<double>("vote_window_sec", 1.0);
    max_votes_ = this->declare_parameter<int>("vote_window_max_samples", 8);
    std::string input_topic = this->declare_parameter<std::string>("input_topic", "/camera/detections");
    std::string output_topic = this->declare_parameter<std::string>("output_topic", "/perception/goalpost");
    std::string camera_output_topic = this->declare_parameter<std::string>(
        "camera_output_topic", "/perception/goalpost_cam");

    min_confidence_ = std::clamp(min_confidence_, 0.0, 1.0);
    min_height_ = std::max(0.0, min_height_);
    smoothing_alpha_ = std::clamp(smoothing_alpha_, 0.0, 1.0);
    if (sustain_time < 0.0)
      sustain_time = 0.0;
    if (heartbeat_period <= 0.0)
      heartbeat_period = 0.1;

    sustain_duration_ = rclcpp::Duration::from_seconds(sustain_time);

    detections_sub_ = this->create_subscription<op3_vision_msgs::msg::Vision>(
        input_topic, rclcpp::SensorDataQoS(),
        std::bind(&GoalpostFilterNode::onDetectionsMsg, this, std::placeholders::_1));

    goal_pub_ = this->create_publisher<op3_vision_msgs::msg::Detection>(output_topic, 10);
    camera_goal_pub_ = this->create_publisher<op3_vision_msgs::msg::Detection>(
        camera_output_topic, 10);

    heartbeat_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(heartbeat_period),
        std::bind(&GoalpostFilterNode::republishIfFresh, this));

    RCLCPP_INFO(this->get_logger(), "Goalpost filter node ready (input: %s, outputs: %s, %s)",
                input_topic.c_str(), output_topic.c_str(), camera_output_topic.c_str());
  }

private:
  void onDetectionsMsg(const op3_vision_msgs::msg::Vision::SharedPtr msg)
  {
    struct Candidate
    {
      const op3_vision_msgs::msg::Detection *detection{nullptr};
      double score{0.0};
      double bearing_x{std::numeric_limits<double>::quiet_NaN()};
      double bearing_y{std::numeric_limits<double>::quiet_NaN()};
      double center_x{0.0};
      double center_y{0.0};
      double width{0.0};
      double height{0.0};
      double left_edge{0.0};
      double right_edge{0.0};
    };

    std::vector<Candidate> candidates;
    candidates.reserve(msg->detections.size());

    for (const auto &detection : msg->detections)
    {
      const double bbox_height = detection.bbox.size_y;
      const double bbox_width = detection.bbox.size_x;
      double best_class_score = std::numeric_limits<double>::lowest();

      for (const auto &result : detection.results)
      {
        if (result.hypothesis.class_id != goal_class_id_)
          continue;
        if (result.hypothesis.score < min_confidence_)
          continue;
        if (bbox_height < min_height_)
          continue;

        best_class_score = std::max(best_class_score, result.hypothesis.score);
      }

      if (best_class_score == std::numeric_limits<double>::lowest())
        continue;

      Candidate cand;
      cand.detection = &detection;
      cand.score = bbox_height * best_class_score;
      cand.bearing_x = detection.bearing.x;
      cand.bearing_y = detection.bearing.y;
      cand.center_x = detection.bbox.center.position.x;
      cand.center_y = detection.bbox.center.position.y;
      cand.width = (bbox_width > 0.0) ? bbox_width : min_height_;
      cand.height = (bbox_height > 0.0) ? bbox_height : min_height_;
      cand.left_edge = cand.center_x - cand.width * 0.5;
      cand.right_edge = cand.center_x + cand.width * 0.5;
      candidates.push_back(cand);
    }

    if (candidates.empty())
      return;

    const Candidate *best_single = nullptr;
    const Candidate *pair_a = nullptr;
    const Candidate *pair_b = nullptr;
    double best_span = std::numeric_limits<double>::lowest();

    for (const auto &cand : candidates)
    {
      if (!best_single || cand.score > best_single->score)
        best_single = &cand;
    }

    if (candidates.size() >= 2)
    {
      for (size_t i = 0; i < candidates.size(); ++i)
      {
        for (size_t j = i + 1; j < candidates.size(); ++j)
        {
          const auto &first = candidates[i];
          const auto &second = candidates[j];
          double span = std::numeric_limits<double>::quiet_NaN();
          if (std::isfinite(first.bearing_x) && std::isfinite(second.bearing_x))
          {
            span = std::abs(first.bearing_x - second.bearing_x);
          }
          else
          {
            span = std::abs(first.center_x - second.center_x);
          }
          if (span > best_span)
          {
            best_span = span;
            pair_a = &first;
            pair_b = &second;
          }
        }
      }
    }

    const Candidate *template_cand = pair_a ? ((pair_a->score >= pair_b->score) ? pair_a : pair_b) : best_single;
    if (!template_cand)
      return;

    auto filtered = *(template_cand->detection);
    filtered.header = msg->header;

    if (pair_a && pair_b)
    {
      const Candidate *left = pair_a;
      const Candidate *right = pair_b;
      if (right->center_x < left->center_x)
        std::swap(left, right);

      const double cover_left = std::min(left->left_edge, right->left_edge);
      const double cover_right = std::max(left->right_edge, right->right_edge);
      const double width = std::max(cover_right - cover_left, std::max(left->width, right->width));
      const double center_x = 0.5 * (cover_left + cover_right);
      const double center_y = 0.5 * (left->center_y + right->center_y);
      const double height = std::max(left->height, right->height);

      double bearing_x = template_cand->bearing_x;
      if (std::isfinite(left->bearing_x) && std::isfinite(right->bearing_x))
        bearing_x = 0.5 * (left->bearing_x + right->bearing_x);

      double bearing_y = template_cand->bearing_y;
      if (std::isfinite(left->bearing_y) && std::isfinite(right->bearing_y))
        bearing_y = 0.5 * (left->bearing_y + right->bearing_y);

      filtered.bearing.x = bearing_x;
      filtered.bearing.y = bearing_y;
      filtered.bbox.center.position.x = center_x;
      filtered.bbox.center.position.y = center_y;
      filtered.bbox.size_x = width;
      filtered.bbox.size_y = height;
    }

    filtered.bearing.x = applyVote(filtered.bearing.x, filtered.header.stamp);

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

    auto camera_detection = filtered;
    if (bearing_transformer_.toCameraAngles(camera_detection))
    {
      last_camera_detection_ = camera_detection;
      have_camera_detection_ = true;
      camera_goal_pub_->publish(last_camera_detection_);
    }

    auto transformed = filtered;
    if (bearing_transformer_.transform(transformed))
    {
      last_body_detection_ = transformed;
      have_body_detection_ = true;
      goal_pub_->publish(transformed);
    }

    last_detection_time_ = this->get_clock()->now();
  }

  void republishIfFresh()
  {
    if ((this->get_clock()->now() - last_detection_time_) > sustain_duration_)
      return;

    if (have_camera_detection_)
      camera_goal_pub_->publish(last_camera_detection_);

    if (have_body_detection_)
      goal_pub_->publish(last_body_detection_);
  }

  std::string goal_class_id_;
  double min_confidence_{};
  double min_height_{};
  double smoothing_alpha_{};
  rclcpp::Duration sustain_duration_{0, 0};
  double vote_window_sec_{1.0};
  int max_votes_{8};

  rclcpp::Subscription<op3_vision_msgs::msg::Vision>::SharedPtr detections_sub_;
  rclcpp::Publisher<op3_vision_msgs::msg::Detection>::SharedPtr goal_pub_;
  rclcpp::Publisher<op3_vision_msgs::msg::Detection>::SharedPtr camera_goal_pub_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;

  op3_vision_msgs::msg::Detection last_detection_;
  op3_vision_msgs::msg::Detection last_camera_detection_;
  op3_vision_msgs::msg::Detection last_body_detection_;
  rclcpp::Time last_detection_time_{0, 0, RCL_ROS_TIME};
  bool have_detection_{false};
  bool have_camera_detection_{false};
  bool have_body_detection_{false};

  std::deque<std::pair<double, rclcpp::Time>> bearing_votes_;

  double applyVote(double bearing, const rclcpp::Time &stamp)
  {
    bearing_votes_.emplace_back(bearing, stamp);
    while (!bearing_votes_.empty())
    {
      const auto age = (stamp - bearing_votes_.front().second).seconds();
      if (age <= vote_window_sec_)
        break;
      bearing_votes_.pop_front();
    }
    while ((int)bearing_votes_.size() > max_votes_)
      bearing_votes_.pop_front();

    if (bearing_votes_.empty())
      return bearing;

    double sum = 0.0;
    for (const auto &entry : bearing_votes_)
      sum += entry.first;
    return sum / static_cast<double>(bearing_votes_.size());
  }

  op3_perception::BearingTransformer bearing_transformer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoalpostFilterNode>());
  rclcpp::shutdown();
  return 0;
}
