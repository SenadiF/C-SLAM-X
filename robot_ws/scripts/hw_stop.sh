#!/usr/bin/env bash
# Stops everything started by hw_proposed_start.sh (and any stray copies
# of the same nodes started by hand), so the next run starts clean.

PID_FILE=/tmp/hw_logs/pids
if [ -f "$PID_FILE" ]; then
    while read -r pid; do
        kill -INT -- -"$pid" 2>/dev/null
    done < "$PID_FILE"
    sleep 3
    while read -r pid; do
        kill -9 -- -"$pid" 2>/dev/null
    done < "$PID_FILE"
    rm -f "$PID_FILE"
fi

for p in micro_ros_agent wheel_odometry_node static_transform_publisher time_node \
         ekf_node slam_toolbox lifecycle_manager map_merge frontier_explorer \
         astar_node pure_pursuit multirobot_slam robot_controller robot_coordinator; do
    pkill -9 -f "$p" 2>/dev/null
done
echo "stopped"
