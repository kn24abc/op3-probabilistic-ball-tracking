# Probabilistic Ball Tracking for Humanoid Robot Soccer

**Platform:** ROBOTIS OP3 | **Middleware:** ROS2 Jazzy | **Simulator:** Webots

A ball re-acquisition system for humanoid robot soccer that combines **deep learning object detection**, **Bayesian particle filter state estimation**, and a **hint-biased reactive head scan** to recover the ball faster after losing sight of it.

---

## Problem

In RoboCup Humanoid soccer, the ball frequently leaves the robot's camera field of view. A naive approach scans the head blindly across the grid until the ball reappears — wasting critical game time. This project uses a particle filter to maintain a probability distribution over ball position during the blind period and directs the head scan toward the most probable location.

---

## My Contributions

This repository contains the packages I wrote or substantially modified. The upstream ROBOTIS hardware framework (`ROBOTIS-Framework`, `op3_manager`, walking/action modules) is not included here.

| Package | File | What I did |
|---|---|---|
| `op3_perception` | `ball_particle_filter_node.cpp` | **Wrote from scratch** — full SIR particle filter (predict, bearing update, range update, resampler) |
| `op3_perception` | `ball_reacquisition_logger_node.cpp` | **Wrote from scratch** — measurement node logging lost→found times to CSV |
| `op3_perception` | `test/test_ball_particle_filter.cpp` | **Wrote from scratch** — 15 GTest unit tests covering all filter components |
| `op3_head_scan` | `head_scan.cpp` → `planTargets()` | **Modified** — added hint-proximity sort so scan sweeps toward predicted ball location first |
| `op3_control_bridge` | `control_bridge_node.cpp` | **Modified** — added `onParticleBearing()` subscription, hint storage, fall recovery execution (`triggerRecovery()`, `startRecoveryResumeWatcher()`) |
| `op3_control_bridge` | `config/control_bridge_params.yaml` | **Tuned** — wait_cycles 35→15, tilt range -1°→-35°, exposed `particle_hint_timeout_sec` as ROS2 param |
| `vision_model` | `bh-v10.cfg`, `class.names` | YOLO network architecture config for ball/robot detection (288×288, 8 classes) |

---

## System Architecture

```
Camera (/camera/image_raw)
  └─► op3_vision_darknet (YOLO)
        └─► /camera/detections
              ├─► ball_filter_node ──► /perception/ball (body frame)
              │                   └─► /perception/ball_cam (camera frame)
              │
              └─► ball_particle_filter_node ──► /perception/ball_search_bearing
                    ▲ also subscribes to:              (pan/tilt hint)
                    │  /robotis/walking/set_params (odometry)
                    └─ /robotis/present_joint_states  (head tilt → range)

/perception/ball + /perception/ball_search_bearing
  └─► control_bridge_node
        ├─► BallTracker (PID) ──► /robotis/head_control/set_joint_states_offset
        ├─► BallFollower      ──► /robotis/walking/set_params
        └─► HeadScan (hint-biased grid) ──► head joint commands
```

---

## Algorithms

### 1. Particle Filter (Bayesian State Estimation)

Maintains N=300 particles representing a probability distribution over ball position `(x, y)` in robot body frame.

**Predict** — propagates particles through inverse robot odometry with Gaussian noise:
```
rx = p.x·cos(-δψ) − p.y·sin(-δψ)
ry = p.x·sin(-δψ) + p.y·cos(-δψ)
p.x = rx − δx + N(0, σ_x=0.10)
p.y = ry      + N(0, σ_y=0.10)
```

**Update: bearing likelihood** — weights particles by Gaussian bearing error:
```
expected_pan = atan2(p.y, p.x)
p.w *= exp(-0.5 · (meas_pan - expected_pan)² / σ_φ²)    σ_φ = 0.15 rad
```

**Update: range likelihood** (novel) — uses camera tilt angle for 2D fix:
```
range_est = camera_height / tan(-abs_tilt_rad)           camera_height = 0.55 m
p.w *= exp(-0.5 · (√(p.x²+p.y²) - range_est)² / σ_r²)  σ_r = 0.50 m
```
This upgrades the filter from a bearing-only line estimate to a 2D position fix.

**Resample** — low-variance systematic resampler when N_eff = 1/Σ(w²) < N/2.

### 2. Hint-Biased Head Scan

The particle filter publishes a weighted-mean bearing. `planTargets()` sorts the 5×4 scan grid by angular distance to that hint — head visits the predicted location first instead of scanning in a fixed raster order.

### 3. YOLO Object Detection

Custom-trained `bh-v10` network (Darknet, 288×288 RGB input) detects ball, goal, robots, and field lines. Confidence threshold 0.50, NMS IoU 0.45. Bearing computed from pixel offset and camera FOV.

---

## Unit Tests

15 GTest unit tests covering the mathematical core of the particle filter:

| Test Suite | Tests | Covers |
|---|---|---|
| `PredictTest` | 3 | Forward motion, yaw rotation, combined transform |
| `UpdateTest` | 4 | Bearing likelihood, range likelihood, tilt cutoff |
| `NormalisationTest` | 2 | Weights sum to 1.0 after init and update |
| `CollapseTest` | 1 | Weight collapse triggers uniform reinitialisation |
| `ResampleTest` | 2 | Particle count preserved, equal weights after resample |
| `OdometryTest` | 3 | Step calculation, half-cycle integration, zero amplitude |

Build and run:
```bash
colcon build --packages-select op3_perception --symlink-install --cmake-args -DBUILD_TESTING=ON
./build/op3_perception/test_ball_particle_filter
# Expected: 15 tests, 15 passed
```

---

## Key Parameters

All tunable without recompiling via `op3_control_bridge/config/control_bridge_params.yaml`:

| Parameter | Value | Effect |
|---|---|---|
| `num_particles` | 300 | Filter population |
| `sigma_x / sigma_y` | 0.10 m | Odometry process noise |
| `sigma_phi` | 0.15 rad | Bearing measurement noise |
| `sigma_range_m` | 0.50 m | Range measurement noise |
| `camera_height_m` | 0.55 m | Camera mounting height (geometry model) |
| `particle_hint_timeout_sec` | 2.0 s | How long hint stays fresh before expiry |
| `wait_cycles` | 15 @ 30Hz (~0.5 s) | Grace period before declaring ball lost |
| `min_tilt_deg` | -35° | Scan covers ground-level ball (was -1°) |
| `tilt_cells` | 4 | Four tilt layers in scan grid (was 2) |

---

## Evaluation

Run the re-acquisition logger to measure how long the robot takes to find the ball after losing it:

```bash
# Terminal 1 — launch simulation
ros2 launch op3_webots robot_manager.launch.py

# Terminal 2 — launch adapter
ros2 launch op3_adapter op3_adapter_sim.launch.py

# Terminal 3 — start logger
ros2 run op3_perception ball_reacquisition_logger_node
# Results saved to /tmp/ball_reacquisition_log.csv

# Terminal 4 — enable behaviours
ros2 topic pub --once /control_bridge/ball_follow_enable std_msgs/msg/Bool '{data: true}'
ros2 topic pub --once /control_bridge/ball_search_enable std_msgs/msg/Bool '{data: true}'
```

Evaluation protocol: 10 trials × 4 distances (0.3 m, 1 m, 2 m, 3 m), with vs without hint biasing (`particle_hint_timeout_sec: 0.0` disables hint).

---

## Build

```bash
# Clone into a ROS2 Jazzy workspace src/ directory
cd ~/your_ws/src && git clone https://github.com/jadstrike/op3-probabilistic-ball-tracking.git

# Build
cd ~/your_ws
source /opt/ros/jazzy/setup.zsh
colcon build --packages-select op3_perception op3_head_scan op3_control_bridge --symlink-install
```

**Dependencies:** `rclcpp`, `op3_vision_msgs`, `op3_walking_module_msgs`, `geometry_msgs`, `sensor_msgs`, `tf2_ros`, `yaml-cpp`, `game_controller_hl_interfaces`

---

## Degree Context

Built as part of a BSc AI and Robotics final project demonstrating:
- **AI** — Bayesian state estimation (Sequential Monte Carlo / particle filter)
- **Computer Vision** — deep learning object detection (YOLO), monocular range estimation, bearing-to-body-frame transform
- **Robotics** — ROS2 C++ node development, sensor fusion, reactive subsumption architecture, physical humanoid platform (ROBOTIS OP3)
