# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

C-SLAM+ is a two-robot collaborative SLAM system. Each robot is an ESP32 (differential drive, wheel encoders, BMI160 IMU, LD19 LiDAR, SD card) that streams sensor data over WiFi/UDP via micro-ROS to a Raspberry Pi 5 / laptop coordinator running ROS 2 **Jazzy**. The research goal is fault tolerance on low-cost hardware (ESP-NOW robot-to-robot messaging, SD buffering during WiFi loss, decentralized frontier auctions, quadtree maps), evaluated against a centralized baseline. See [README.md](README.md) for the full motivation and evaluation metrics.

## Repository layout

- `robot_ws/` — main ROS 2 colcon workspace (all coordinator-side code). Run all `colcon`/`ros2` commands from here.
- `microros_ws/` — micro-ROS agent build (`micro_ros_setup`); must be sourced for the agent.
- `ldlidar_ros2_ws/` — vendor LD19 LiDAR driver.
- `main_ino_copy_*/` — ESP32 Arduino firmware (modular: `RosComms`, `Encoders`, `ImuSensor`, `LidarReader`, `MotorControl`, `GapFollow`, `SdLogger`, `StateMachine`). Agent IP is hardcoded in `RosComms.cpp`; the robot namespace comes from `ROBOT_NAMESPACE`. This is a snapshot copy and may lag the firmware actually flashed.
- `Unit Testing/` — standalone Arduino sketches for individual sensors/motors.
- `maps_1/`, `robot_ws/maps/` — saved occupancy maps (`.pgm` + `.yaml`) used for comparisons.
- `Weekly_Progress/`, `Timeline.md` — progress notes, not code.

## Build

```bash
source /opt/ros/jazzy/setup.bash
cd robot_ws
colcon build --packages-select my_navigation ss multi_robot_sim robot_localization_config   # typical set
source install/setup.bash
```

Most packages are `ament_python`; Python nodes must be registered in that package's `setup.py` `entry_points` to be runnable. `robot_ws/src/build`, `robot_ws/src/install`, `robot_ws/src/log` are stray build artifacts — ignore them.

Tests are only the ament boilerplate (flake8/pep257/copyright): `colcon test --packages-select my_navigation && colcon test-result --verbose`.

## Running

[robot_ws/commands.txt](robot_ws/commands.txt) is the authoritative runbook — terminal-by-terminal startup order for both stacks on hardware and in simulation, plus stop commands. Key points:

**Simulation (Gazebo, no hardware):**
```bash
ros2 launch multi_robot_sim multi_robot_sim.launch.py merge_mode:=known   # or unknown | ransac_icp
ros2 launch multi_robot_sim baseline_sim.launch.py                         # ss baseline
```

**Hardware:** micro-ROS agent first (`ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888`), then odometry → static TF → `time_node` → EKF ×2 → SLAM → map merge → navigation. Order and staggering matter (slam_toolbox lifecycle races). Shortcut: `robot_ws/scripts/hw_proposed_start.sh [known|unknown]` (logs in `/tmp/hw_logs`), stop with `hw_stop.sh`.

**Experiments:**
- `scripts/run_trials.sh <pkg> <launch> <strategy> <csv_dir> <N> <duration_s> [launch args]` — repeated sim trials with full process-group cleanup between runs; `metrics_logger` appends one CSV row per trial.
- `scripts/analyze_trials.py <csvs>` — mean ± std per column.
- `scripts/compute_map_iou.py <ground_truth.yaml> <map.yaml>` — map IoU vs ground truth.

Leftover processes from a previous run silently corrupt the next one; kill everything (see `pkill` lists in commands.txt) before switching stacks or trials.

## Architecture

Two competing navigation stacks share the same sensor/localization front end:

**Front end (per robot, namespaced `/robot1`, `/robot2`):**
ESP32 → `scan_raw`/`imu_raw`/`encoder` → `robot_control/time_node` (restamps messages because the firmware sends zeroed `header.stamp`, which breaks TF/slam_toolbox — must run before EKF/SLAM) → `robot_odometry/wheel_odometry_node` (`wheel_odom`) → `robot_localization` EKF (`odometry/filtered`, config in `robot_localization_config`) → slam_toolbox per robot (`/robotN/map`).

**Proposed stack (`my_navigation`)** — centralized nodes that each handle both robots with hardcoded `/robot1`/`/robot2` topics:
- Map merge: `multirobot_map_merge` (vendored in `m-explore-ros2/map_merge`) produces global `/map`. `known_init_poses:=true` uses offsets in `my_navigation/config/map_merge_params.yaml`; `false` uses feature matching. `ransac.py` (`ransac_icp_map_aligner`) is the custom RANSAC+ICP alternative.
- `frontier.py` (`frontier_explorer_node`) → `/robotN/frontier_goals` → `astar.py` (`astar_node`) → `/robotN/planned_path` → `pure_pursuit_node.py` → `/robotN/cmd_vel`. Pure pursuit reports back via `/robotN/goal_reached` and `/robotN/planning_failed`, which drive frontier reassignment. `frontier1.py` is an older version not wired into entry points.
- Hardware needs safer params than sim defaults (see commands.txt: `bootstrap_enabled:=false`, reduced speeds, larger inflation).

**Baseline stack (`ss`)** — `multirobot_slam`, `robot_coordinator`, `robot_controller`. It hardcodes `/robotN/scan` and `/robotN/odom`, so on hardware every `ss` node needs `-r /robotN/odom:=/robotN/wheel_odom`, and the two `robot_controller` instances need distinct `__node:` names.

**Initial poses must match physical placement** in two places, kept in sync by hand: `my_navigation/config/map_merge_params.yaml` (proposed) and `SPAWN_POSES` in `ss/ss/frames.py` (baseline).

`multi_robot_sim` (CMake package) holds the Gazebo world, URDF, ros_gz bridge configs and `*_sim.yaml` copies of the EKF/SLAM/map-merge configs — sim and hardware configs are separate files, so a tuning change usually needs to be made in both. `auto_nav`, `cslam_simulation`, `fake_sensors` are earlier experiments.
