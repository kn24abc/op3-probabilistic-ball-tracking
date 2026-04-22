// ball_particle_filter_node.cpp
// Particle filter for ball position recovery when the ball is out of sight.
//
// Subscribes to:
//   /camera/detections             (op3_vision_msgs/Vision)    – bearing updates
//   /robotis/walking/set_params    (WalkingParam)              – odometry integration
//   /robotis/present_joint_states  (sensor_msgs/JointState)    – head tilt for range estimate
//
// Publishes:
//   /perception/ball_search_bearing (geometry_msgs/Point)
//     x = pan  (radians, +right)
//     y = tilt (radians, +down)  – fixed ground-search angle when no detection
//
// The bearing hint can be consumed by BallTracker to direct HeadScan toward
// the most probable ball location rather than a random grid cell.

#include <algorithm>
#include <cmath>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "op3_walking_module_msgs/msg/walking_param.hpp"
#include "op3_vision_msgs/msg/vision.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "rclcpp/rclcpp.hpp"

// ---------------------------------------------------------------------------
// Particle state – 2D ball position in robot body frame (metres)
// ---------------------------------------------------------------------------
struct Particle
{
  double x{};  // forward from robot (metres)
  double y{};  // lateral (metres, +left)
  double w{};  // normalised weight
};

// ---------------------------------------------------------------------------
// BallParticleFilterNode
// ---------------------------------------------------------------------------
class BallParticleFilterNode : public rclcpp::Node
{
public:
  BallParticleFilterNode()
  : Node("ball_particle_filter_node"),
    rng_(std::random_device{}()),
    uniform_(0.0, 1.0)
  {
    N_             = declare_parameter<int>("num_particles", 300);
    sigma_x_       = declare_parameter<double>("sigma_x", 0.10);
    sigma_y_       = declare_parameter<double>("sigma_y", 0.10);
    sigma_phi_     = declare_parameter<double>("sigma_phi", 0.15);
    hfov_rad_      = declare_parameter<double>("hfov_rad", 1.047);     // ~60 deg
    vfov_rad_      = declare_parameter<double>("vfov_rad", 0.785);     // ~45 deg
    min_conf_      = declare_parameter<double>("min_confidence", 0.55);
    ball_class_    = declare_parameter<std::string>("ball_class_id", "ball");
    search_tilt_   = declare_parameter<double>("search_tilt_rad", -0.35);  // ~-20 deg
    camera_height_ = declare_parameter<double>("camera_height_m", 0.55);
    sigma_range_   = declare_parameter<double>("sigma_range_m", 0.50);
    min_tilt_for_range_ =
      declare_parameter<double>("min_tilt_for_range_rad", 0.09);  // ~5 deg downward

    particles_.resize(N_);
    initParticles();

    detections_sub_ = create_subscription<op3_vision_msgs::msg::Vision>(
      "/camera/detections", rclcpp::SensorDataQoS(),
      std::bind(&BallParticleFilterNode::onDetections, this, std::placeholders::_1));

    walking_sub_ = create_subscription<op3_walking_module_msgs::msg::WalkingParam>(
      "/robotis/walking/set_params", rclcpp::SystemDefaultsQoS(),
      std::bind(&BallParticleFilterNode::onWalkingParam, this, std::placeholders::_1));

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/robotis/present_joint_states", rclcpp::SensorDataQoS(),
      std::bind(&BallParticleFilterNode::onJointState, this, std::placeholders::_1));

    bearing_pub_ = create_publisher<geometry_msgs::msg::Point>(
      "/perception/ball_search_bearing", 10);

    last_odom_time_ = get_clock()->now();

    RCLCPP_INFO(get_logger(),
      "ball_particle_filter_node ready "
      "(N=%d, sigma_phi=%.2f rad, hfov=%.2f rad, "
      "camera_height=%.2f m, sigma_range=%.2f m)",
      N_, sigma_phi_, hfov_rad_, camera_height_, sigma_range_);
  }

private:
  // ---------------------------------------------------------------------------
  // Initialise N particles uniformly over the forward half-field
  // ---------------------------------------------------------------------------
  void initParticles()
  {
    std::uniform_real_distribution<double> dx(0.1, 9.0);
    std::uniform_real_distribution<double> dy(-4.5, 4.5);
    for (auto & p : particles_) {
      p.x = dx(rng_);
      p.y = dy(rng_);
      p.w = 1.0 / N_;
    }
  }

  // ---------------------------------------------------------------------------
  // Predict: propagate particles by inverse of robot motion (body-frame shift)
  // delta_x   – forward displacement of robot (metres) since last call
  // delta_yaw – yaw rotation of robot (radians, +CCW) since last call
  // ---------------------------------------------------------------------------
  void predict(double delta_x, double delta_yaw)
  {
    std::normal_distribution<double> nx(0.0, sigma_x_);
    std::normal_distribution<double> ny(0.0, sigma_y_);
    for (auto & p : particles_) {
      // Rotate ball position by -delta_yaw (robot turned, so ball appears rotated)
      double rx =  p.x * std::cos(-delta_yaw) - p.y * std::sin(-delta_yaw);
      double ry =  p.x * std::sin(-delta_yaw) + p.y * std::cos(-delta_yaw);
      // Subtract robot forward motion and add process noise
      p.x = rx - delta_x + nx(rng_);
      p.y = ry            + ny(rng_);
    }
  }

  // ---------------------------------------------------------------------------
  // Update: weight each particle by bearing likelihood and (when camera is
  // tilted sufficiently downward) by ground-plane range likelihood.
  //
  // meas_pan     – measured horizontal bearing to ball (radians, +right)
  // abs_tilt_rad – absolute camera tilt in body frame (radians, negative = down)
  //                Set to 0.0 when head joint state is unavailable.
  // ---------------------------------------------------------------------------
  void update(double meas_pan, double abs_tilt_rad)
  {
    // Pre-compute range estimate from camera geometry when head is looking
    // down enough for the estimate to be reliable.
    const bool use_range = (abs_tilt_rad < -min_tilt_for_range_);
    double range_est = 0.0;
    if (use_range) {
      range_est = camera_height_ / std::tan(-abs_tilt_rad);
      // Sanity check: reject implausible ranges
      if (range_est < 0.05 || range_est > 12.0) {
        // Out of usable range; fall back to bearing-only update
        range_est = 0.0;
      }
    }
    const bool apply_range = use_range && (range_est > 0.0);

    double total_w = 0.0;
    for (auto & p : particles_) {
      // --- bearing likelihood ---
      double expected_pan = std::atan2(p.y, p.x);
      double err = meas_pan - expected_pan;
      // Wrap to [-pi, pi]
      while (err >  M_PI) err -= 2.0 * M_PI;
      while (err < -M_PI) err += 2.0 * M_PI;
      p.w *= std::exp(-0.5 * err * err / (sigma_phi_ * sigma_phi_));

      // --- range likelihood (tilt-derived ground-plane distance) ---
      if (apply_range) {
        const double particle_range = std::sqrt(p.x * p.x + p.y * p.y);
        const double range_err = particle_range - range_est;
        p.w *= std::exp(-0.5 * range_err * range_err / (sigma_range_ * sigma_range_));
      }

      total_w += p.w;
    }

    if (total_w < 1e-12) {
      RCLCPP_WARN(get_logger(), "Particle weight collapse – reinitialising filter");
      initParticles();
      return;
    }

    // Normalise weights
    for (auto & p : particles_) {
      p.w /= total_w;
    }

    // Resample when effective sample size drops below N/2
    double sum_sq = 0.0;
    for (const auto & p : particles_) {
      sum_sq += p.w * p.w;
    }
    if ((1.0 / sum_sq) < (N_ / 2.0)) {
      resample();
    }
  }

  // ---------------------------------------------------------------------------
  // Low-variance systematic resampler
  // ---------------------------------------------------------------------------
  void resample()
  {
    double r = uniform_(rng_) / N_;
    double c = particles_[0].w;
    int i = 0;
    std::vector<Particle> new_p;
    new_p.reserve(N_);
    for (int m = 0; m < N_; ++m) {
      double U = r + static_cast<double>(m) / N_;
      while (U > c && i < N_ - 1) {
        c += particles_[++i].w;
      }
      new_p.push_back(particles_[i]);
      new_p.back().w = 1.0 / N_;
    }
    particles_ = std::move(new_p);
  }

  // ---------------------------------------------------------------------------
  // Publish weighted mean bearing of the particle cloud
  // ---------------------------------------------------------------------------
  void publishSearchBearing()
  {
    double mean_x = 0.0;
    double mean_y = 0.0;
    for (const auto & p : particles_) {
      mean_x += p.w * p.x;
      mean_y += p.w * p.y;
    }

    geometry_msgs::msg::Point msg;
    msg.x = std::atan2(mean_y, mean_x);  // pan  (radians)
    msg.y = search_tilt_;                 // tilt (radians, fixed downward)
    msg.z = 0.0;
    bearing_pub_->publish(msg);
  }

  // ---------------------------------------------------------------------------
  // Joint state callback – extract head_tilt for range estimate
  // ---------------------------------------------------------------------------
  void onJointState(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    for (size_t i = 0; i < msg->name.size(); ++i) {
      if (msg->name[i] == "head_tilt" && i < msg->position.size()) {
        head_tilt_rad_ = msg->position[i];
        have_head_tilt_ = true;
        return;
      }
    }
  }

  // ---------------------------------------------------------------------------
  // Walking param callback – integrate odometry using ROS clock (/clock)
  // period_time is in milliseconds (ROBOTIS framework convention)
  // ---------------------------------------------------------------------------
  void onWalkingParam(const op3_walking_module_msgs::msg::WalkingParam::SharedPtr msg)
  {
    // period_time is a full step cycle in milliseconds
    double period_s = msg->period_time / 1000.0;
    if (period_s < 1e-6) return;

    rclcpp::Time now = get_clock()->now();
    double dt = (now - last_odom_time_).seconds();
    last_odom_time_ = now;

    if (dt <= 0.0 || dt > 2.0) return;  // skip stale or first callback

    // Each half-cycle the robot advances x_move_amplitude metres and rotates
    // angle_move_amplitude degrees — integrate over elapsed time
    double half_period_s = period_s / 2.0;
    double steps = dt / half_period_s;

    double delta_x   = static_cast<double>(msg->x_move_amplitude) * steps;
    double delta_yaw = static_cast<double>(msg->angle_move_amplitude) * (M_PI / 180.0) * steps;

    if (std::abs(delta_x) > 1e-6 || std::abs(delta_yaw) > 1e-6) {
      predict(delta_x, delta_yaw);
    }
  }

  // ---------------------------------------------------------------------------
  // Detection callback – update filter on each ball sighting
  // ---------------------------------------------------------------------------
  void onDetections(const op3_vision_msgs::msg::Vision::SharedPtr msg)
  {
    const op3_vision_msgs::msg::Detection * best = nullptr;
    double best_score = min_conf_;

    for (const auto & det : msg->detections) {
      for (const auto & result : det.results) {
        if (result.hypothesis.class_id != ball_class_) continue;
        if (result.hypothesis.score > best_score) {
          best_score = result.hypothesis.score;
          best = &det;
        }
      }
    }

    if (best != nullptr) {
      // bearing.x is a fraction of hFOV (-0.5 … +0.5) → convert to radians
      const double meas_pan = static_cast<double>(best->bearing.x) * hfov_rad_;

      // bearing.y is a fraction of vFOV (-0.5 … +0.5); positive = above optical axis
      // Combine with head joint tilt to get absolute camera elevation in body frame
      double abs_tilt_rad = 0.0;
      if (have_head_tilt_) {
        const double tilt_in_cam_rad = static_cast<double>(best->bearing.y) * vfov_rad_;
        abs_tilt_rad = head_tilt_rad_ + tilt_in_cam_rad;
      }

      update(meas_pan, abs_tilt_rad);
    }

    publishSearchBearing();
  }

  // Parameters
  int    N_{300};
  double sigma_x_{0.10};
  double sigma_y_{0.10};
  double sigma_phi_{0.15};
  double hfov_rad_{1.047};
  double vfov_rad_{0.785};
  double min_conf_{0.55};
  double search_tilt_{-0.35};
  double camera_height_{0.55};
  double sigma_range_{0.50};
  double min_tilt_for_range_{0.09};
  std::string ball_class_{"ball"};

  // Particle cloud
  std::vector<Particle> particles_;

  // RNG
  std::mt19937 rng_;
  std::uniform_real_distribution<double> uniform_;

  // Head tilt state (updated from /robotis/present_joint_states)
  double head_tilt_rad_{0.0};
  bool   have_head_tilt_{false};

  // Odometry integration state (using ROS clock to avoid wall-time drift)
  rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};

  // ROS interfaces
  rclcpp::Subscription<op3_vision_msgs::msg::Vision>::SharedPtr detections_sub_;
  rclcpp::Subscription<op3_walking_module_msgs::msg::WalkingParam>::SharedPtr walking_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr bearing_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BallParticleFilterNode>());
  rclcpp::shutdown();
  return 0;
}
