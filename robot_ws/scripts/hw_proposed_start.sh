#!/usr/bin/env bash
# Starts the full hardware proposed stack (agent + odometry + TF + restamper
# + EKF x2 + SLAM x2 + map merge + frontier/A*/pure pursuit) in the
# background, in the same order as commands.txt, each node logging to its
# own file under /tmp/hw_logs. The micro-ROS agent is restarted
# automatically if it ever crashes. Stop everything with hw_stop.sh.
#
# Usage: ./hw_proposed_start.sh [known|unknown]   (map merge mode, default known)

# (no set -u: ROS setup scripts reference unset variables)
MERGE_MODE="${1:-known}"
KNOWN=true
[ "$MERGE_MODE" = "unknown" ] && KNOWN=false

LOG_DIR=/tmp/hw_logs
PID_FILE="$LOG_DIR/pids"
mkdir -p "$LOG_DIR"
rm -f "$LOG_DIR"/*.log "$PID_FILE"

source /opt/ros/jazzy/setup.bash
source "$HOME/Desktop/C-SLAM-X/microros_ws/install/local_setup.bash"
source "$HOME/Desktop/C-SLAM-X/robot_ws/install/local_setup.bash"

start() {
    local name="$1"; shift
    setsid bash -c "$*" > "$LOG_DIR/$name.log" 2>&1 &
    echo "$!" >> "$PID_FILE"
    echo "started $name (log: $LOG_DIR/$name.log)"
}

start agent 'while true; do ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888; echo "=== AGENT EXITED - restarting in 1s ==="; sleep 1; done'
sleep 2
start wheel_odom_robot1 'ros2 run robot_odometry wheel_odometry_node --ros-args -p robot_name:=robot1'
start wheel_odom_robot2 'ros2 run robot_odometry wheel_odometry_node --ros-args -p robot_name:=robot2'
start static_tf 'ros2 launch robot_bringup static_tf.launch.py'
start time_node 'ros2 run robot_control time_node'
sleep 2
start ekf_robot1 'ros2 launch robot_localization_config localization.launch.py namespace:=robot1'
start ekf_robot2 'ros2 launch robot_localization_config localization.launch.py namespace:=robot2'
sleep 2
start slam 'ros2 launch robot_localization_config slam.launch.py'
sleep 5
start map_merge "ros2 launch my_navigation map_merge.launch.py known_init_poses:=$KNOWN"
sleep 3
start frontier_explorer 'ros2 run my_navigation frontier_explorer_node'
# Hardware safety margins (sim defaults are tighter: 0.6 m/s, 0.15 m stop,
# 0.15 m inflation). With encoder speed control the real robots actually
# reach the commanded speed, and scans arrive ~0.1 s late over Wi-Fi.
start astar 'ros2 run my_navigation astar_node --ros-args -p obstacle_inflation_radius:=0.22'
start pure_pursuit 'ros2 run my_navigation pure_pursuit_node --ros-args -p bootstrap_enabled:=false -p front_cone_deg:=45.0 -p linear_speed:=0.25 -p max_angular_speed:=1.0 -p emergency_distance:=0.25 -p safe_distance:=0.45'

echo
echo "All started. Logs: $LOG_DIR/*.log   Stop with: ./hw_stop.sh"
