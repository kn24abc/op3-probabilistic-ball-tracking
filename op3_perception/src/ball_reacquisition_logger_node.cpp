// ball_reacquisition_logger_node.cpp
// Measures ball re-acquisition time for quantitative evaluation of the
// particle filter + HeadScan hint biasing pipeline.
//
// Subscribes to:
//   /perception/ball/visible  (std_msgs/Bool) – published by BallTracker
//
// Behaviour:
//   - On false → true transition: computes and logs re-acquisition time.
//   - Running statistics (count, mean, min, max) are printed after each event.
//   - Results are also appended to a CSV file for offline analysis.
//
// Usage:
//   ros2 run op3_perception ball_reacquisition_logger_node
//   With custom CSV path:
//     ros2 run op3_perception ball_reacquisition_logger_node
//         --ros-args -p csv_path:=/tmp/reacq_results.csv

#include <chrono>
#include <cmath>
#include <fstream>
#include <limits>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

class BallReacquisitionLoggerNode : public rclcpp::Node
{
public:
  BallReacquisitionLoggerNode()
  : Node("ball_reacquisition_logger_node")
  {
    visibility_topic_ =
      declare_parameter<std::string>("visibility_topic", "/perception/ball/visible");
    csv_path_ =
      declare_parameter<std::string>("csv_path", "/tmp/ball_reacquisition_log.csv");

    visible_sub_ = create_subscription<std_msgs::msg::Bool>(
      visibility_topic_, rclcpp::QoS(10),
      std::bind(&BallReacquisitionLoggerNode::onVisibility, this, std::placeholders::_1));

    // Open CSV file and write header if the file is new / empty.
    csv_stream_.open(csv_path_, std::ios::app);
    if (csv_stream_.is_open()) {
      // Write header only when file is empty (new file).
      csv_stream_.seekp(0, std::ios::end);
      if (csv_stream_.tellp() == 0) {
        csv_stream_ << "trial,lost_time_s,found_time_s,reacquisition_time_s\n";
      }
      RCLCPP_INFO(get_logger(), "Logging to %s", csv_path_.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "Could not open CSV at %s – logging to console only",
                  csv_path_.c_str());
    }

    RCLCPP_INFO(get_logger(),
                "ball_reacquisition_logger_node ready (topic: %s)",
                visibility_topic_.c_str());
  }

  ~BallReacquisitionLoggerNode() override
  {
    if (csv_stream_.is_open()) {
      csv_stream_.close();
    }
    if (trial_count_ > 0) {
      printSummary();
    }
  }

private:
  void onVisibility(const std_msgs::msg::Bool::SharedPtr msg)
  {
    const bool visible = msg->data;
    const rclcpp::Time now = this->now();

    if (last_visible_ && !visible) {
      // Ball has just been lost.
      lost_time_ = now;
      ball_currently_lost_ = true;
      RCLCPP_INFO(get_logger(), "Ball LOST at t=%.3f s", now.seconds());
    } else if (!last_visible_ && visible && ball_currently_lost_) {
      // Ball has just been re-acquired.
      const double reacq_s = (now - lost_time_).seconds();
      trial_count_++;

      // Update running statistics.
      sum_reacq_s_  += reacq_s;
      sum_sq_reacq_ += reacq_s * reacq_s;
      if (reacq_s < min_reacq_s_) min_reacq_s_ = reacq_s;
      if (reacq_s > max_reacq_s_) max_reacq_s_ = reacq_s;

      const double mean = sum_reacq_s_ / trial_count_;

      RCLCPP_INFO(get_logger(),
                  "Ball FOUND  – trial %d  reacq=%.3f s  "
                  "[mean=%.3f s  min=%.3f s  max=%.3f s  n=%d]",
                  trial_count_, reacq_s, mean, min_reacq_s_, max_reacq_s_, trial_count_);

      // Append to CSV.
      if (csv_stream_.is_open()) {
        csv_stream_ << trial_count_ << ","
                    << std::fixed << lost_time_.seconds() << ","
                    << now.seconds() << ","
                    << reacq_s << "\n";
        csv_stream_.flush();
      }

      ball_currently_lost_ = false;
    }

    last_visible_ = visible;
  }

  void printSummary() const
  {
    if (trial_count_ == 0) return;
    const double mean = sum_reacq_s_ / trial_count_;
    // Population standard deviation
    const double variance = (sum_sq_reacq_ / trial_count_) - (mean * mean);
    const double stddev = (variance > 0.0) ? std::sqrt(variance) : 0.0;

    RCLCPP_INFO(get_logger(),
                "\n===== Re-acquisition Summary =====\n"
                "  Trials : %d\n"
                "  Mean   : %.3f s\n"
                "  Std dev: %.3f s\n"
                "  Min    : %.3f s\n"
                "  Max    : %.3f s\n"
                "  CSV    : %s\n"
                "==================================",
                trial_count_, mean, stddev, min_reacq_s_, max_reacq_s_,
                csv_path_.c_str());
  }

  // Parameters
  std::string visibility_topic_;
  std::string csv_path_;

  // State
  bool last_visible_{true};    // assume visible at start to avoid false trigger
  bool ball_currently_lost_{false};
  rclcpp::Time lost_time_{0, 0, RCL_ROS_TIME};

  // Statistics
  int    trial_count_{0};
  double sum_reacq_s_{0.0};
  double sum_sq_reacq_{0.0};
  double min_reacq_s_{std::numeric_limits<double>::max()};
  double max_reacq_s_{0.0};

  // ROS interfaces
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr visible_sub_;
  std::ofstream csv_stream_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BallReacquisitionLoggerNode>());
  rclcpp::shutdown();
  return 0;
}
