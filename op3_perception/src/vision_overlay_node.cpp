#include <algorithm>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <limits>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include "opencv2/imgproc.hpp"
#include "op3_vision_msgs/msg/detection.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace stride::op3
{

namespace
{

constexpr double kDefaultTimeout = 0.6;

struct TimedDetection
{
  op3_vision_msgs::msg::Detection detection;
  rclcpp::Time stamp;
};

std::string format_confidence(const op3_vision_msgs::msg::Detection &detection)
{
  double best_score = std::numeric_limits<double>::lowest();
  for (const auto &result : detection.results)
  {
    if (result.hypothesis.score > best_score)
    {
      best_score = result.hypothesis.score;
    }
  }

  if (best_score == std::numeric_limits<double>::lowest())
    return {};

  std::ostringstream oss;
  oss << std::fixed << std::setprecision(2) << best_score;
  return oss.str();
}

cv::Rect clamp_rect(const cv::Rect &rect, const cv::Size &frame_size)
{
  cv::Rect bounds(0, 0, frame_size.width, frame_size.height);
  return rect & bounds;
}

bool compute_detection_center_px(
    const op3_vision_msgs::msg::Detection &detection,
    const cv::Size &frame_size,
    cv::Point &center_px,
    double fallback_horizontal_fov_rad,
    double fallback_vertical_fov_rad)
{
  if (frame_size.width <= 0 || frame_size.height <= 0)
    return false;

  int center_x = static_cast<int>(std::round(detection.bbox.center.position.x));
  int center_y = static_cast<int>(std::round(detection.bbox.center.position.y));

  if (center_x == 0 && center_y == 0 &&
      std::isfinite(detection.bearing.x) &&
      std::isfinite(detection.bearing.y) &&
      fallback_horizontal_fov_rad > 0.0 &&
      fallback_vertical_fov_rad > 0.0)
  {
    const double norm_x = std::clamp(
        -detection.bearing.x / fallback_horizontal_fov_rad,
        -1.0, 1.0);
    const double norm_y = std::clamp(
        detection.bearing.y / fallback_vertical_fov_rad,
        -1.0, 1.0);
    center_x = static_cast<int>(std::round(
        (0.5 + norm_x) * static_cast<double>(frame_size.width)));
    center_y = static_cast<int>(std::round(
        (0.5 + norm_y) * static_cast<double>(frame_size.height)));
  }

  center_x = std::clamp(center_x, 0, std::max(0, frame_size.width - 1));
  center_y = std::clamp(center_y, 0, std::max(0, frame_size.height - 1));
  center_px = cv::Point(center_x, center_y);
  return true;
}

bool detection_to_rect(
    const op3_vision_msgs::msg::Detection &detection,
    const cv::Size &frame_size,
    cv::Rect &rect,
    cv::Point &center_px,
    double fallback_horizontal_fov_rad,
    double fallback_vertical_fov_rad)
{
  if (!compute_detection_center_px(
          detection, frame_size, center_px,
          fallback_horizontal_fov_rad,
          fallback_vertical_fov_rad))
    return false;

  int width = static_cast<int>(std::round(std::abs(detection.bbox.size_x)));
  int height = static_cast<int>(std::round(std::abs(detection.bbox.size_y)));
  if (width <= 0)
    width = std::max(12, frame_size.width / 40);
  if (height <= 0)
    height = std::max(12, frame_size.height / 40);

  cv::Rect raw(center_px.x - width / 2, center_px.y - height / 2, width, height);
  rect = clamp_rect(raw, frame_size);
  return rect.width > 0 && rect.height > 0;
}

std::optional<int> bearing_column_px(
    const op3_vision_msgs::msg::Detection &detection,
    int image_width,
    double fallback_horizontal_fov_rad)
{
  if (image_width <= 0)
    return std::nullopt;
  if (!std::isfinite(detection.bearing.x))
    return std::nullopt;

  if (fallback_horizontal_fov_rad <= 0.0)
    return std::nullopt;

  const double normalized = std::clamp(
      0.5 - static_cast<double>(detection.bearing.x / fallback_horizontal_fov_rad),
      0.0, 1.0);
  int column = static_cast<int>(std::round(normalized * static_cast<double>(image_width)));
  column = std::clamp(column, 0, std::max(0, image_width - 1));
  return column;
}

}  // namespace

class VisionOverlayNode : public rclcpp::Node
{
public:
  VisionOverlayNode()
    : Node("vision_overlay_node")
  {
    image_topic_ = declare_parameter<std::string>("image_topic", "/camera/image_raw");
    detection_topic_ = declare_parameter<std::string>("detection_topic", "/perception/ball_cam");
    goalpost_topic_ = declare_parameter<std::string>("goalpost_topic", "/perception/goalpost_cam");
    overlay_topic_ = declare_parameter<std::string>("overlay_topic", "/camera/overlay");
    detection_timeout_ = declare_parameter<double>("detection_timeout", kDefaultTimeout);
    goalpost_timeout_ = declare_parameter<double>("goalpost_timeout", kDefaultTimeout);
    goalpost_group_window_ = declare_parameter<double>("goalpost_group_window", 0.2);
    if (goalpost_group_window_ <= 0.0)
      goalpost_group_window_ = 0.2;
    bearing_horizontal_fov_rad_ =
      declare_parameter<double>("bearing_horizontal_fov_deg", 35.2) * (M_PI / 180.0);
    bearing_vertical_fov_rad_ =
      declare_parameter<double>("bearing_vertical_fov_deg", 21.6) * (M_PI / 180.0);

    detection_sub_ = create_subscription<op3_vision_msgs::msg::Detection>(
        detection_topic_, 10,
        std::bind(&VisionOverlayNode::onBallDetection, this, std::placeholders::_1));

    if (!goalpost_topic_.empty())
    {
      goalpost_sub_ = create_subscription<op3_vision_msgs::msg::Detection>(
          goalpost_topic_, 10,
          std::bind(&VisionOverlayNode::onGoalpostDetection, this, std::placeholders::_1));
      RCLCPP_INFO(get_logger(), "Goalpost overlay enabled from %s", goalpost_topic_.c_str());
    }
    else
    {
      RCLCPP_INFO(get_logger(), "Goalpost overlay disabled (goalpost_topic empty)");
    }

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        image_topic_, rclcpp::SensorDataQoS(),
        std::bind(&VisionOverlayNode::onImage, this, std::placeholders::_1));

    overlay_pub_ = create_publisher<sensor_msgs::msg::Image>(overlay_topic_, rclcpp::QoS(10));

    RCLCPP_INFO(get_logger(),
                "Overlaying detections from %s onto %s (publishing %s)",
                detection_topic_.c_str(), image_topic_.c_str(), overlay_topic_.c_str());
  }

private:
  void onBallDetection(const op3_vision_msgs::msg::Detection::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_ball_detection_ = *msg;
    last_ball_detection_time_ = now();
    have_ball_detection_ = true;
  }

  void onGoalpostDetection(const op3_vision_msgs::msg::Detection::SharedPtr msg)
  {
    if (!msg)
      return;

    std::lock_guard<std::mutex> lock(mutex_);
    const rclcpp::Time stamp = now();
    have_goal_detection_ = true;
    last_goal_detection_time_ = stamp;
    goalpost_buffer_.push_back(TimedDetection{*msg, stamp});
    pruneGoalpostsLocked(stamp);
  }

  void onImage(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv::Mat frame;
    if (!convertToMat(*msg, frame))
      return;

    op3_vision_msgs::msg::Detection ball_copy;
    bool draw_ball = false;
    std::vector<op3_vision_msgs::msg::Detection> goalposts_copy;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto current_time = now();
      if (have_ball_detection_ &&
          (current_time - last_ball_detection_time_).seconds() <= detection_timeout_)
      {
        ball_copy = last_ball_detection_;
        draw_ball = true;
      }

      if (have_goal_detection_ &&
          (current_time - last_goal_detection_time_).seconds() <= goalpost_timeout_)
      {
        pruneGoalpostsLocked(current_time);
        for (const auto &entry : goalpost_buffer_)
        {
          if ((current_time - entry.stamp).seconds() <= goalpost_group_window_)
          {
            goalposts_copy.push_back(entry.detection);
          }
        }
      }
    }

    cv::Point ball_center;
    bool have_ball_center = false;
    if (draw_ball)
    {
      have_ball_center = drawBallDetection(frame, ball_copy, ball_center);
    }

    bool has_goal_column = false;
    int goal_column = 0;
    if (!goalposts_copy.empty())
    {
      has_goal_column = drawGoalposts(frame, goalposts_copy, goal_column);
    }

    if (has_goal_column)
    {
      drawGoalArrow(frame, goal_column);
      if (have_ball_center)
      {
        drawGoalSideHint(frame, ball_center, goal_column);
      }
    }

    auto output_image = makeImageMsg(frame, msg->header);
    overlay_pub_->publish(output_image);
  }

  bool drawBallDetection(cv::Mat &frame,
                         const op3_vision_msgs::msg::Detection &detection,
                         cv::Point &center_out)
  {
    if (frame.empty())
      return false;

    const auto &bbox = detection.bbox;
    cv::Point center;
    if (!compute_detection_center_px(detection,
                                     frame.size(),
                                     center,
                                     bearing_horizontal_fov_rad_,
                                     bearing_vertical_fov_rad_))
      return false;

    const cv::Scalar accent_color(0, 200, 255);  // amber
    int box_w = static_cast<int>(std::round(std::abs(bbox.size_x)));
    int box_h = static_cast<int>(std::round(std::abs(bbox.size_y)));
    int radius = std::max(box_w, box_h) / 2;
    if (radius <= 0)
      radius = std::max(10, std::max(frame.cols, frame.rows) / 20);

    cv::circle(frame, center, radius, accent_color, 2);
    cv::circle(frame, center, 4, accent_color, cv::FILLED);
    center_out = center;

    auto confidence_text = format_confidence(detection);
    if (!confidence_text.empty())
    {
      int baseline = 0;
      const double font_scale = 0.6;
      const int font = cv::FONT_HERSHEY_SIMPLEX;
      cv::Size text_size = cv::getTextSize(confidence_text, font, font_scale, 2, &baseline);

      cv::Point text_origin(
          std::max(0, center.x - text_size.width / 2),
          std::max(0, center.y - radius - 10));

      cv::Rect text_bg(
          text_origin.x - 3,
          std::max(0, text_origin.y - text_size.height - baseline - 3),
          text_size.width + 6,
          text_size.height + baseline + 4);
      text_bg = clamp_rect(text_bg, frame.size());
      cv::rectangle(frame, text_bg, cv::Scalar(0, 0, 0), cv::FILLED);
      cv::putText(frame, confidence_text,
                  cv::Point(text_bg.x + 3, text_bg.y + text_size.height),
                  font, font_scale, cv::Scalar(255, 255, 255), 2);
    }
    return true;
  }

  bool drawGoalposts(
      cv::Mat &frame,
      const std::vector<op3_vision_msgs::msg::Detection> &detections,
      int &column_out)
  {
    if (frame.empty() || detections.empty())
      return false;

    const cv::Scalar rect_color(80, 255, 80);
    const cv::Scalar line_color(0, 180, 255);
    const cv::Scalar text_color(0, 0, 0);

    double left_edge = std::numeric_limits<double>::infinity();
    double right_edge = -std::numeric_limits<double>::infinity();
    bool have_edges = false;
    double left_center_px = std::numeric_limits<double>::infinity();
    double right_center_px = -std::numeric_limits<double>::infinity();
    std::optional<int> left_bearing_column;
    std::optional<int> right_bearing_column;

    for (const auto &detection : detections)
    {
      cv::Rect rect;
      cv::Point center;
      if (!detection_to_rect(detection,
                             frame.size(),
                             rect,
                             center,
                             bearing_horizontal_fov_rad_,
                             bearing_vertical_fov_rad_))
        continue;

      cv::rectangle(frame, rect, rect_color, 2, cv::LINE_AA);
      cv::circle(frame, center, 3, rect_color, cv::FILLED);
      left_edge = std::min(left_edge, static_cast<double>(rect.x));
      right_edge = std::max(right_edge, static_cast<double>(rect.x + rect.width));
      have_edges = true;

      auto confidence_text = format_confidence(detection);
      if (!confidence_text.empty())
      {
        int baseline = 0;
        constexpr int font = cv::FONT_HERSHEY_DUPLEX;
        constexpr double font_scale = 0.5;
        cv::Size text_size = cv::getTextSize(confidence_text, font, font_scale, 1, &baseline);
        cv::Rect label_rect(
            rect.x,
            std::max(0, rect.y - text_size.height - baseline - 4),
            text_size.width + 6,
            text_size.height + baseline + 4);
        label_rect = clamp_rect(label_rect, frame.size());
        cv::rectangle(frame, label_rect, rect_color, cv::FILLED);
        cv::putText(frame, confidence_text,
                    cv::Point(label_rect.x + 3, label_rect.y + text_size.height),
                    font, font_scale, text_color, 1);
      }

      if (center.x < left_center_px)
      {
        left_center_px = static_cast<double>(center.x);
        left_bearing_column = bearing_column_px(
            detection, frame.cols, bearing_horizontal_fov_rad_);
      }
      if (center.x > right_center_px)
      {
        right_center_px = static_cast<double>(center.x);
        right_bearing_column = bearing_column_px(
            detection, frame.cols, bearing_horizontal_fov_rad_);
      }
    }

    int line_x = -1;
    if (left_bearing_column && right_bearing_column)
    {
      line_x = static_cast<int>(std::round(
          (*left_bearing_column + *right_bearing_column) * 0.5));
    }
    else if (std::isfinite(left_center_px) &&
             std::isfinite(right_center_px) &&
             right_center_px >= left_center_px)
    {
      line_x = static_cast<int>(std::round((left_center_px + right_center_px) * 0.5));
    }
    else if (have_edges &&
             std::isfinite(left_edge) &&
             std::isfinite(right_edge) &&
             right_edge >= left_edge)
    {
      line_x = static_cast<int>(std::round((left_edge + right_edge) * 0.5));
    }

    if (line_x >= 0)
    {
      line_x = std::clamp(line_x, 0, std::max(0, frame.cols - 1));
      cv::line(frame,
               cv::Point(line_x, 0),
               cv::Point(line_x, frame.rows - 1),
               line_color, 2, cv::LINE_AA);
      column_out = line_x;
      return true;
    }
    return false;
  }

  void drawGoalArrow(cv::Mat &frame, int column)
  {
    if (frame.empty())
      return;
    column = std::clamp(column, 0, std::max(0, frame.cols - 1));
    const cv::Point start(frame.cols / 2, frame.rows - 20);
    const cv::Point end(column, std::max(0, frame.rows / 2));
    cv::arrowedLine(frame, start, end, cv::Scalar(0, 200, 255), 3, cv::LINE_AA, 0, 0.15);
  }

  void drawGoalSideHint(cv::Mat &frame, const cv::Point &ball_center, int goal_column)
  {
    const int arrow_len = 25;
    const cv::Scalar color(255, 200, 0);
    const int dx = goal_column - ball_center.x;
    if (std::abs(dx) < 5)
      return;
    const int dir = (dx > 0) ? 1 : -1;
    cv::Point start(ball_center.x, ball_center.y + 20);
    cv::Point end(ball_center.x + dir * arrow_len, ball_center.y + 20);
    start.x = std::clamp(start.x, 0, std::max(0, frame.cols - 1));
    end.x = std::clamp(end.x, 0, std::max(0, frame.cols - 1));
    start.y = std::clamp(start.y, 0, std::max(0, frame.rows - 1));
    end.y = std::clamp(end.y, 0, std::max(0, frame.rows - 1));
    cv::arrowedLine(frame, start, end, color, 2, cv::LINE_AA, 0, 0.25);
  }

  void pruneGoalpostsLocked(const rclcpp::Time &now_time)
  {
    goalpost_buffer_.erase(
        std::remove_if(
            goalpost_buffer_.begin(), goalpost_buffer_.end(),
            [&](const TimedDetection &entry) {
              return (now_time - entry.stamp).seconds() > goalpost_group_window_;
            }),
        goalpost_buffer_.end());
  }

  std::string image_topic_;
  std::string detection_topic_;
  std::string goalpost_topic_;
  std::string overlay_topic_;
  double detection_timeout_{kDefaultTimeout};
  double goalpost_timeout_{kDefaultTimeout};
  double goalpost_group_window_{0.2};
  double bearing_horizontal_fov_rad_{35.2 * M_PI / 180.0};
  double bearing_vertical_fov_rad_{21.6 * M_PI / 180.0};

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<op3_vision_msgs::msg::Detection>::SharedPtr detection_sub_;
  rclcpp::Subscription<op3_vision_msgs::msg::Detection>::SharedPtr goalpost_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr overlay_pub_;

  std::mutex mutex_;
  op3_vision_msgs::msg::Detection last_ball_detection_;
  rclcpp::Time last_ball_detection_time_{0, 0, RCL_ROS_TIME};
  bool have_ball_detection_{false};

  std::vector<TimedDetection> goalpost_buffer_;
  rclcpp::Time last_goal_detection_time_{0, 0, RCL_ROS_TIME};
  bool have_goal_detection_{false};

  bool convertToMat(const sensor_msgs::msg::Image &msg, cv::Mat &output)
  {
    const auto type_rgb = sensor_msgs::image_encodings::RGB8;
    const auto type_bgr = sensor_msgs::image_encodings::BGR8;
    const auto type_bgra = sensor_msgs::image_encodings::BGRA8;
    const auto type_rgba = sensor_msgs::image_encodings::RGBA8;
    const auto type_mono = sensor_msgs::image_encodings::MONO8;
    if (msg.encoding == type_rgb || msg.encoding == type_bgr ||
        msg.encoding == type_bgra || msg.encoding == type_rgba)
    {
      const bool has_alpha = (msg.encoding == type_bgra || msg.encoding == type_rgba);
      const int cv_type = has_alpha ? CV_8UC4 : CV_8UC3;
      cv::Mat wrapped(msg.height, msg.width, cv_type,
                      const_cast<uint8_t *>(msg.data.data()), msg.step);
      wrapped.copyTo(output);
      if (msg.encoding == type_rgb)
        cv::cvtColor(output, output, cv::COLOR_RGB2BGR);
      else if (msg.encoding == type_rgba)
        cv::cvtColor(output, output, cv::COLOR_RGBA2BGR);
      else if (msg.encoding == type_bgra)
        cv::cvtColor(output, output, cv::COLOR_BGRA2BGR);
      return true;
    }
    if (msg.encoding == type_mono)
    {
      cv::Mat wrapped(msg.height, msg.width, CV_8UC1,
                      const_cast<uint8_t *>(msg.data.data()), msg.step);
      cv::cvtColor(wrapped, output, cv::COLOR_GRAY2BGR);
      return true;
    }

    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Unsupported image encoding '%s' (expect bgr8/rgb8/bgra8/rgba8/mono8)", msg.encoding.c_str());
    return false;
  }

  sensor_msgs::msg::Image makeImageMsg(const cv::Mat &frame, const std_msgs::msg::Header &header)
  {
    sensor_msgs::msg::Image output;
    output.header = header;
    output.height = frame.rows;
    output.width = frame.cols;
    output.encoding = sensor_msgs::image_encodings::BGR8;
    output.is_bigendian = false;
    output.step = static_cast<uint32_t>(frame.cols * frame.elemSize());
    const size_t data_size = static_cast<size_t>(frame.total() * frame.elemSize());
    output.data.resize(data_size);
    if (data_size > 0)
    {
      std::memcpy(output.data.data(), frame.data, data_size);
    }
    return output;
  }
};

}  // namespace stride::op3

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<stride::op3::VisionOverlayNode>());
  rclcpp::shutdown();
  return 0;
}
