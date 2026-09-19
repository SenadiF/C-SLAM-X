#!/usr/bin/env python3
"""
metrics_logger.py
==================
Universal navigation-performance metrics logger. Works with ANY of the
three strategies being compared (reactive baseline, custom A* + pure
pursuit, or Nav2) without modification, since it detects "goal reached"
by proximity rather than requiring each stack to publish its own
reached-signal.

Covers Table 1 (Navigation Performance) metrics:
  1. Goal-reaching success rate
  2. Mean time-to-goal
  3. Total path length traveled
  4. Path efficiency ratio
  5. Stuck events
  6. Near-collision events / minimum obstacle clearance
  7. Mean planning time per cycle (optional, needs one line added
     to the planner being tested - see bottom of this file)
  8. Trajectory smoothness
  9. Recovery behavior count (Nav2 only, optional)
  10. Cross-track error (mean / RMSE / max) against the current
      reference path - only meaningful for a stack that publishes an
      explicit global path (e.g. astar_node's planned_path). A stack
      with no such path (the reactive baseline) will naturally show
      this as empty/nan - that's a real result, not a missing feature.

Also logs "time-to-map" (Table 2, metric 10) since it's a free
by-product of subscribing to /map anyway.

Map quality metrics (SSIM, IoU, Hausdorff, ICP RMSE - Table 2,
metrics 11-15) are computed separately, AFTER a trial, by comparing
saved .pgm map files - see compute_map_iou.py and the SSIM/Hausdorff/
ICP snippets discussed separately. This node only handles live,
per-trial navigation metrics.

Usage:
  ros2 run my_navigation metrics_logger --ros-args \
      -p robot_name:=robot1 \
      -p strategy_name:=custom_astar \
      -p output_csv:=custom_astar_trial1.csv

Run this ALONGSIDE whichever navigation stack you're testing (reactive,
your custom stack, or Nav2). No changes needed to those stacks except
the optional planning-time hook described at the bottom. Press Ctrl+C
at the end of each trial - it prints and appends a summary row to the
CSV.

NOTE: the "ss" baseline stack has no EKF, so its odometry topic is
/robot1/odom, not /robot1/odometry/filtered. Override with
-p odom_topic:=odom when logging an ss trial.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray, PoseStamped
from std_msgs.msg import Bool, Float32

import math
import time
import csv


class MetricsLogger(Node):

    def __init__(self):
        super().__init__('metrics_logger')

        # ------------------------------------------------------------
        # PARAMETERS
        # ------------------------------------------------------------
        self.declare_parameter('robot_name', 'robot1')
        self.declare_parameter('strategy_name', 'unknown_strategy')
        self.declare_parameter('output_csv', 'trial_metrics.csv')

        self.declare_parameter('goal_tolerance_m', 0.3)
        self.declare_parameter('collision_distance_m', 0.05)
        self.declare_parameter('stuck_speed_threshold', 0.02)
        self.declare_parameter('stuck_duration_s', 5.0)
        self.declare_parameter('map_coverage_target', 0.8)

        # Topic names - override these if a given stack uses different
        # topic names than your custom stack's defaults.
        self.declare_parameter('odom_topic', 'odometry/filtered')
        self.declare_parameter('scan_topic', 'scan')
        self.declare_parameter('goal_array_topic', 'frontier_goals')   # PoseArray, e.g. your frontier_explorer
        self.declare_parameter('goal_pose_topic', 'goal_pose')          # PoseStamped, e.g. baseline coordinator / Nav2 goal
        self.declare_parameter('planning_time_topic', 'planning_time_ms')  # optional, Float32
        self.declare_parameter('recovery_topic', 'recovery_triggered')      # optional, Bool
        # Path topic for cross-track error - only meaningful for a stack
        # that publishes an explicit global path (your A* stack does; the
        # reactive baseline doesn't, so cross-track error naturally comes
        # out empty for it - that's a real result, not a missing feature).
        self.declare_parameter('path_topic', 'planned_path')

        self.robot_name = self.get_parameter('robot_name').value
        self.strategy_name = self.get_parameter('strategy_name').value
        self.output_csv = self.get_parameter('output_csv').value

        self.goal_tolerance_m = self.get_parameter('goal_tolerance_m').value
        self.collision_distance_m = self.get_parameter('collision_distance_m').value
        self.stuck_speed_threshold = self.get_parameter('stuck_speed_threshold').value
        self.stuck_duration_s = self.get_parameter('stuck_duration_s').value
        self.map_coverage_target = self.get_parameter('map_coverage_target').value

        odom_topic = f"/{self.robot_name}/{self.get_parameter('odom_topic').value}"
        scan_topic = f"/{self.robot_name}/{self.get_parameter('scan_topic').value}"
        goal_array_topic = f"/{self.robot_name}/{self.get_parameter('goal_array_topic').value}"
        goal_pose_topic = f"/{self.robot_name}/{self.get_parameter('goal_pose_topic').value}"
        planning_time_topic = f"/{self.robot_name}/{self.get_parameter('planning_time_topic').value}"
        recovery_topic = f"/{self.robot_name}/{self.get_parameter('recovery_topic').value}"
        path_topic = f"/{self.robot_name}/{self.get_parameter('path_topic').value}"

        # ------------------------------------------------------------
        # STATE
        # ------------------------------------------------------------
        self.trial_start = time.time()

        self.prev_x = None
        self.prev_y = None
        self.prev_yaw = None
        self.path_length = 0.0
        self.smoothness_sum = 0.0
        self.smoothness_samples = 0

        self.current_linear_speed = 0.0

        self.current_goal = None            # (x, y)
        self.goal_start_x = None            # robot position when goal was assigned
        self.goal_start_y = None
        self.goal_assigned_time = None

        self.goals_assigned = 0
        self.goals_reached = 0
        self.time_to_goal_list = []
        self.path_efficiency_list = []
        self.distance_since_goal_assigned = 0.0

        self.collision_count = 0

        self.stuck_count = 0
        self.stuck_timer_start = None
        self.stuck_already_counted = False

        self.time_to_map_logged = False
        self.time_to_map = None

        self.planning_times_ms = []

        self.recovery_count = 0

        # Cross-track error: perpendicular distance from the robot's
        # actual position to the nearest segment of the currently
        # published reference path.
        self.current_path_points = []   # [(x, y), ...]
        self.cross_track_sum = 0.0
        self.cross_track_sum_sq = 0.0
        self.cross_track_samples = 0
        self.cross_track_max = 0.0

        # Minimum obstacle clearance observed over the whole trial.
        self.min_clearance_m = float('inf')

        # ------------------------------------------------------------
        # SUBSCRIPTIONS
        # ------------------------------------------------------------
        self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
        self.create_subscription(LaserScan, scan_topic, self.scan_callback, 10)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)

        self.create_subscription(PoseArray, goal_array_topic, self.goal_array_callback, 10)
        self.create_subscription(PoseStamped, goal_pose_topic, self.goal_pose_callback, 10)

        # Optional - only useful if you've added the one-line hooks
        # described at the bottom of this file.
        self.create_subscription(Float32, planning_time_topic, self.planning_time_callback, 10)
        self.create_subscription(Bool, recovery_topic, self.recovery_callback, 10)

        # Optional - only present for stacks that publish an explicit
        # global path (e.g. astar_node's /robotN/planned_path).
        self.create_subscription(Path, path_topic, self.path_callback, 10)

        self.get_logger().info(
            f'Metrics logger started - robot: {self.robot_name}, '
            f'strategy: {self.strategy_name}. Press Ctrl+C at end of trial.'
        )
        self.get_logger().info(f'Watching odom: {odom_topic}')
        self.get_logger().info(f'Watching scan: {scan_topic}')
        self.get_logger().info(f'Watching goals: {goal_array_topic} and {goal_pose_topic}')

    # ------------------------------------------------------------------
    # ODOMETRY - path length, smoothness, stuck detection, goal-reached
    # detection (proximity-based, works for all three strategies)
    # ------------------------------------------------------------------
    def odom_callback(self, msg):

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)
        self.current_linear_speed = abs(msg.twist.twist.linear.x)

        if self.prev_x is not None:
            step = math.hypot(x - self.prev_x, y - self.prev_y)
            self.path_length += step
            self.distance_since_goal_assigned += step

        if self.prev_yaw is not None:
            dyaw = self.normalize_angle(yaw - self.prev_yaw)
            self.smoothness_sum += abs(dyaw)
            self.smoothness_samples += 1

        self.prev_x, self.prev_y, self.prev_yaw = x, y, yaw

        # ---- Cross-track error against the current reference path ----
        if len(self.current_path_points) >= 1:
            error = self.distance_to_path(x, y, self.current_path_points)
            self.cross_track_sum += error
            self.cross_track_sum_sq += error * error
            self.cross_track_samples += 1
            if error > self.cross_track_max:
                self.cross_track_max = error

        # ---- Stuck detection ----
        if self.current_goal is not None:
            if self.current_linear_speed < self.stuck_speed_threshold:
                if self.stuck_timer_start is None:
                    self.stuck_timer_start = time.time()
                elif (time.time() - self.stuck_timer_start > self.stuck_duration_s
                      and not self.stuck_already_counted):
                    self.stuck_count += 1
                    self.stuck_already_counted = True
                    self.get_logger().warn(f'Stuck event #{self.stuck_count} detected.')
            else:
                self.stuck_timer_start = None
                self.stuck_already_counted = False

        # ---- Proximity-based goal-reached detection ----
        # Works uniformly whether the underlying stack publishes an
        # explicit "reached" signal or not (Nav2's action interface
        # doesn't expose a simple topic for this, so proximity is the
        # most robust common denominator across all three strategies).
        if self.current_goal is not None:
            gx, gy = self.current_goal
            dist_to_goal = math.hypot(x - gx, y - gy)

            if dist_to_goal <= self.goal_tolerance_m:
                self.goals_reached += 1
                elapsed = time.time() - self.goal_assigned_time
                self.time_to_goal_list.append(elapsed)

                straight_line = math.hypot(
                    gx - self.goal_start_x, gy - self.goal_start_y
                )
                if self.distance_since_goal_assigned > 0.01:
                    efficiency = straight_line / self.distance_since_goal_assigned
                    self.path_efficiency_list.append(min(efficiency, 1.0))

                self.get_logger().info(
                    f'Goal reached in {elapsed:.1f}s '
                    f'(efficiency={self.path_efficiency_list[-1]:.2f})'
                    if self.path_efficiency_list else
                    f'Goal reached in {elapsed:.1f}s'
                )

                self.current_goal = None
                self.goal_assigned_time = None
                self.stuck_timer_start = None
                self.stuck_already_counted = False
                # Stale now - measuring cross-track error against a
                # completed path in the gap before the next one arrives
                # isn't meaningful.
                self.current_path_points = []

    # ------------------------------------------------------------------
    def scan_callback(self, msg):
        valid = [r for r in msg.ranges if r > msg.range_min]
        if valid:
            closest = min(valid)
            if closest < self.collision_distance_m:
                self.collision_count += 1
            if closest < self.min_clearance_m:
                self.min_clearance_m = closest

    # ------------------------------------------------------------------
    def path_callback(self, msg):
        self.current_path_points = [
            (pose.pose.position.x, pose.pose.position.y)
            for pose in msg.poses
        ]

    # ------------------------------------------------------------------
    # Perpendicular distance from (x, y) to the nearest segment of the
    # given polyline path (falls back to point distance for a 1-point path).
    def distance_to_path(self, x, y, path_points):
        if len(path_points) == 1:
            px, py = path_points[0]
            return math.hypot(x - px, y - py)

        min_dist = float('inf')
        for i in range(len(path_points) - 1):
            x1, y1 = path_points[i]
            x2, y2 = path_points[i + 1]
            dist = self.point_to_segment_distance(x, y, x1, y1, x2, y2)
            if dist < min_dist:
                min_dist = dist
        return min_dist

    @staticmethod
    def point_to_segment_distance(px, py, x1, y1, x2, y2):
        dx = x2 - x1
        dy = y2 - y1
        length_sq = dx * dx + dy * dy
        if length_sq < 1e-12:
            return math.hypot(px - x1, py - y1)
        t = ((px - x1) * dx + (py - y1) * dy) / length_sq
        t = max(0.0, min(1.0, t))
        nearest_x = x1 + t * dx
        nearest_y = y1 + t * dy
        return math.hypot(px - nearest_x, py - nearest_y)

    # ------------------------------------------------------------------
    def map_callback(self, msg):
        if self.time_to_map_logged:
            return
        total = len(msg.data)
        if total == 0:
            return
        known = sum(1 for c in msg.data if c != -1)
        ratio = known / total
        if ratio >= self.map_coverage_target:
            self.time_to_map = time.time() - self.trial_start
            self.time_to_map_logged = True
            self.get_logger().info(
                f'Reached {self.map_coverage_target*100:.0f}% map coverage '
                f'in {self.time_to_map:.1f}s'
            )

    # ------------------------------------------------------------------
    # GOAL TRACKING - handles both PoseArray (your custom stack's
    # frontier_goals) and PoseStamped (baseline's goal_pose, or Nav2's
    # goal if you republish it to this topic).
    # ------------------------------------------------------------------
    def goal_array_callback(self, msg):
        if len(msg.poses) > 0:
            self._register_new_goal(
                msg.poses[0].position.x, msg.poses[0].position.y
            )

    def goal_pose_callback(self, msg):
        self._register_new_goal(
            msg.pose.position.x, msg.pose.position.y
        )

    def _register_new_goal(self, x, y):
        # Avoid double-counting if the same goal is republished
        if self.current_goal is not None:
            gx, gy = self.current_goal
            if math.hypot(x - gx, y - gy) < 0.05:
                return  # same goal, not a new one

        self.current_goal = (x, y)
        self.goal_assigned_time = time.time()
        self.goal_start_x = self.prev_x if self.prev_x is not None else x
        self.goal_start_y = self.prev_y if self.prev_y is not None else y
        self.distance_since_goal_assigned = 0.0
        self.goals_assigned += 1
        self.stuck_timer_start = None
        self.stuck_already_counted = False

    # ------------------------------------------------------------------
    def planning_time_callback(self, msg):
        self.planning_times_ms.append(msg.data)

    def recovery_callback(self, msg):
        if msg.data:
            self.recovery_count += 1

    # ------------------------------------------------------------------
    def quaternion_to_yaw(self, q):
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    # ------------------------------------------------------------------
    def save_summary(self):

        success_rate = (
            self.goals_reached / self.goals_assigned
            if self.goals_assigned > 0 else 0.0
        )
        avg_time_to_goal = (
            sum(self.time_to_goal_list) / len(self.time_to_goal_list)
            if self.time_to_goal_list else float('nan')
        )
        avg_path_efficiency = (
            sum(self.path_efficiency_list) / len(self.path_efficiency_list)
            if self.path_efficiency_list else float('nan')
        )
        avg_smoothness = (
            self.smoothness_sum / self.smoothness_samples
            if self.smoothness_samples > 0 else float('nan')
        )
        avg_planning_time = (
            sum(self.planning_times_ms) / len(self.planning_times_ms)
            if self.planning_times_ms else float('nan')
        )

        mean_cross_track = (
            self.cross_track_sum / self.cross_track_samples
            if self.cross_track_samples > 0 else float('nan')
        )
        rmse_cross_track = (
            math.sqrt(self.cross_track_sum_sq / self.cross_track_samples)
            if self.cross_track_samples > 0 else float('nan')
        )
        max_cross_track = (
            self.cross_track_max if self.cross_track_samples > 0 else float('nan')
        )
        min_clearance = (
            self.min_clearance_m if self.min_clearance_m != float('inf') else float('nan')
        )

        summary = {
            'strategy': self.strategy_name,
            'robot_name': self.robot_name,
            'trial_duration_s': round(time.time() - self.trial_start, 2),
            'goals_assigned': self.goals_assigned,
            'goals_reached': self.goals_reached,
            'success_rate': round(success_rate, 3),
            'avg_time_to_goal_s': round(avg_time_to_goal, 2),
            'total_path_length_m': round(self.path_length, 3),
            'avg_path_efficiency_ratio': round(avg_path_efficiency, 3),
            'stuck_events': self.stuck_count,
            'near_collision_events': self.collision_count,
            'avg_planning_time_ms': round(avg_planning_time, 2),
            'avg_trajectory_smoothness_rad': round(avg_smoothness, 4),
            'mean_cross_track_error_m': round(mean_cross_track, 4),
            'rmse_cross_track_error_m': round(rmse_cross_track, 4),
            'max_cross_track_error_m': round(max_cross_track, 4),
            'min_obstacle_clearance_m': round(min_clearance, 4),
            'recovery_behavior_count': self.recovery_count,
            'time_to_map_coverage_s': (
                round(self.time_to_map, 2) if self.time_to_map else 'not_reached'
            ),
        }

        print('\n===== TRIAL SUMMARY =====')
        for k, v in summary.items():
            print(f'{k}: {v}')
        print('==========================\n')

        file_exists = False
        try:
            with open(self.output_csv, 'r'):
                file_exists = True
        except FileNotFoundError:
            pass

        with open(self.output_csv, 'a', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=summary.keys())
            if not file_exists:
                writer.writeheader()
            writer.writerow(summary)

        print(f'Saved to {self.output_csv}')


def main(args=None):
    rclpy.init(args=args)
    node = MetricsLogger()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        # Save on every exit path, not just Ctrl+C - if this node gets
        # SIGTERM'd instead (e.g. as part of shutting down the whole
        # launch, or from a scripted trial runner), rclpy raises
        # ExternalShutdownException rather than KeyboardInterrupt, and a
        # trial's worth of data would otherwise be silently lost.
        node.save_summary()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()


# ============================================================================
# OPTIONAL INSTRUMENTATION HOOKS
# ============================================================================
#
# For metric #7 (mean planning time per cycle), add this to your A*
# planner's create_path() function (or equivalent point in Nav2's
# planner if you can hook it):
#
#   import time
#   from std_msgs.msg import Float32
#
#   # in __init__:
#   self.planning_time_pub = self.create_publisher(
#       Float32, f'/{self.robot_name}/planning_time_ms', 10)
#
#   # at the start of create_path():
#   t0 = time.time()
#   ... existing planning code ...
#   # at the end, right before returning:
#   msg = Float32()
#   msg.data = (time.time() - t0) * 1000.0
#   self.planning_time_pub.publish(msg)
#
# For metric #9 (Nav2 recovery behavior count), publish a Bool to
# /<robot_name>/recovery_triggered whenever you detect a recovery
# behavior firing (e.g. by monitoring Nav2's behavior_server logs or
# action feedback) - this is optional and only applies to the Nav2 trial.
# ============================================================================
