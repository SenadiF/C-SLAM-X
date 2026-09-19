#!/usr/bin/env bash
# run_trials.sh - run N repeated trials of a sim launch with metrics
# logging, with proper process-group cleanup between each one.
#
# Why this exists: doing this by hand (launch, wait, Ctrl+C, check
# nothing was left running, repeat) is exactly what went wrong more than
# once during manual testing this session - a leftover process from one
# trial silently corrupted the next one's data. This script always starts
# each trial from a verified-clean process tree, and kills the WHOLE
# process group at the end of each trial (not just the launch parent),
# so ros2 launch's own children can't be left behind either.
#
# Usage:
#   ./run_trials.sh <package> <launch_file> <strategy_name> <csv_dir> <num_trials> <trial_duration_s> [extra launch args...]
#
# Examples:
#   ./run_trials.sh multi_robot_sim single_robot_sim.launch.py single_robot ~/comparison_results 5 90
#   ./run_trials.sh multi_robot_sim multi_robot_sim.launch.py two_robot_known ~/comparison_results 5 90 merge_mode:=known
#   ./run_trials.sh multi_robot_sim multi_robot_sim.launch.py two_robot_unknown ~/comparison_results 5 90 merge_mode:=unknown
#   ./run_trials.sh multi_robot_sim baseline_sim.launch.py ss_reactive ~/comparison_results 5 90
#
# Each trial appends one row to <csv_dir>/<strategy_name>_robot1.csv (and
# _robot2.csv for two-robot launches) - run this 5 times and you have 5
# rows ready for analyze_trials.py to turn into mean +/- std.

set -u

if [ "$#" -lt 6 ]; then
    echo "Usage: $0 <package> <launch_file> <strategy_name> <csv_dir> <num_trials> <trial_duration_s> [extra launch args...]"
    exit 1
fi

PACKAGE="$1"
LAUNCH_FILE="$2"
STRATEGY="$3"
CSV_DIR="$4"
NUM_TRIALS="$5"
DURATION="$6"
shift 6
EXTRA_ARGS=("$@")

mkdir -p "$CSV_DIR"

cleanup() {
    pkill -9 -f "ros2 launch" 2>/dev/null
    pkill -9 -f "gz sim" 2>/dev/null
    pkill -9 -f "gz-sim" 2>/dev/null
    pkill -9 -f "parameter_bridge" 2>/dev/null
    pkill -9 -f "robot_state_publisher" 2>/dev/null
    pkill -9 -f "ekf_node" 2>/dev/null
    pkill -9 -f "slam_toolbox" 2>/dev/null
    pkill -9 -f "lifecycle_manager" 2>/dev/null
    pkill -9 -f "multirobot_map_merge" 2>/dev/null
    pkill -9 -f "frontier_explorer" 2>/dev/null
    pkill -9 -f "astar_node" 2>/dev/null
    pkill -9 -f "pure_pursuit" 2>/dev/null
    pkill -9 -f "rviz2" 2>/dev/null
    pkill -9 -f "static_transform_publisher" 2>/dev/null
    pkill -9 -f "multirobot_slam" 2>/dev/null
    pkill -9 -f "robot_controller" 2>/dev/null
    pkill -9 -f "robot_coordinator" 2>/dev/null
    pkill -9 -f "metrics_logger" 2>/dev/null
    pkill -9 -f "ransac_icp" 2>/dev/null
    sleep 2
}

echo "=== Running $NUM_TRIALS trial(s) of strategy '$STRATEGY' (${DURATION}s each) ==="
echo "=== Results -> $CSV_DIR/${STRATEGY}_robot1.csv (and _robot2.csv if applicable) ==="

for i in $(seq 1 "$NUM_TRIALS"); do
    echo ""
    echo "--- Trial $i/$NUM_TRIALS: cleaning up before launch ---"
    cleanup

    LOG_FILE="$CSV_DIR/${STRATEGY}_trial${i}.log"

    echo "--- Trial $i/$NUM_TRIALS: launching (log: $LOG_FILE) ---"
    setsid ros2 launch "$PACKAGE" "$LAUNCH_FILE" \
        log_metrics:=true "strategy_name:=$STRATEGY" "metrics_csv_dir:=$CSV_DIR" \
        "${EXTRA_ARGS[@]}" > "$LOG_FILE" 2>&1 &
    LAUNCH_PID=$!

    echo "Launched (pid $LAUNCH_PID), running for ${DURATION}s..."
    sleep "$DURATION"

    echo "Stopping trial $i (SIGINT to the whole process group)..."
    kill -INT -- -"$LAUNCH_PID" 2>/dev/null

    # Give it up to 20s to shut down cleanly (this is what actually
    # triggers metrics_logger to save its CSV row) before forcing.
    WAITED=0
    while kill -0 "$LAUNCH_PID" 2>/dev/null && [ "$WAITED" -lt 20 ]; do
        sleep 1
        WAITED=$((WAITED + 1))
    done

    if kill -0 "$LAUNCH_PID" 2>/dev/null; then
        echo "Didn't exit within 20s, forcing (kill -9 on the process group)..."
        kill -9 -- -"$LAUNCH_PID" 2>/dev/null
    fi

    cleanup
    echo "--- Trial $i/$NUM_TRIALS done ---"
done

echo ""
echo "=== All $NUM_TRIALS trials of '$STRATEGY' complete. ==="
echo "=== Run analyze_trials.py on the CSVs in $CSV_DIR next. ==="
