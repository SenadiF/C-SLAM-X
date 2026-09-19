import math
import numpy as np
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from nav_msgs.msg import Odometry

from geometry_msgs.msg import Twist

from sensor_msgs.msg import LaserScan

from std_msgs.msg import Bool


class PurePursuit(Node):

    def __init__(self):

        super().__init__('pure_pursuit')

        self.declare_parameter('lookahead_distance', 0.30)
        self.declare_parameter('linear_speed', 0.6)
        self.declare_parameter('max_angular_speed', 1.5)
        self.declare_parameter('goal_tolerance', 0.10)
        self.declare_parameter('minimum_linear_speed', 0.05)
        self.declare_parameter('emergency_distance', 0.15)
        self.declare_parameter('safe_distance', 0.30)

        # NEW - bootstrap gap-following parameters
        self.declare_parameter('bootstrap_speed', 0.12)
        self.declare_parameter('bootstrap_safe_distance_mm', 400)

        # If a robot spends this long pursuing the same goal without
        # reaching it (e.g. oscillating near an obstacle on the way),
        # give up and stop instead of wandering forever. This matters most
        # once exploration is complete and frontier_explorer has stopped
        # assigning replacement goals - nothing else will ever interrupt it.
        self.declare_parameter('goal_pursuit_timeout', 25.0)
        self.goal_pursuit_timeout = self.get_parameter('goal_pursuit_timeout').value

        self.lookahead_distance = self.get_parameter('lookahead_distance').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.minimum_linear_speed = self.get_parameter('minimum_linear_speed').value
        self.emergency_distance = self.get_parameter('emergency_distance').value
        self.safe_distance = self.get_parameter('safe_distance').value
        self.bootstrap_speed = self.get_parameter('bootstrap_speed').value
        self.bootstrap_safe_distance_mm = self.get_parameter('bootstrap_safe_distance_mm').value

        self.robot1_path = None
        self.robot1_x = None
        self.robot1_y = None
        self.robot1_yaw = None
        self.robot1_scan = None
        self.robot1_goal_reached = False

        self.robot2_path = None
        self.robot2_x = None
        self.robot2_y = None
        self.robot2_yaw = None
        self.robot2_scan = None
        self.robot2_goal_reached = False

        self.robot1_path_sub = self.create_subscription(
            Path, '/robot1/planned_path', self.robot1_path_callback, 10
        )
        self.robot2_path_sub = self.create_subscription(
            Path, '/robot2/planned_path', self.robot2_path_callback, 10
        )
        self.robot1_scan_sub = self.create_subscription(
            LaserScan, '/robot1/scan', self.robot1_scan_callback, 10
        )
        self.robot2_scan_sub = self.create_subscription(
            LaserScan, '/robot2/scan', self.robot2_scan_callback, 10
        )
        self.robot1_odom_sub = self.create_subscription(
            Odometry, '/robot1/odometry/filtered', self.robot1_odom_callback, 10
        )
        self.robot2_odom_sub = self.create_subscription(
            Odometry, '/robot2/odometry/filtered', self.robot2_odom_callback, 10
        )

        self.robot1_cmd_pub = self.create_publisher(Twist, '/robot1/cmd_vel', 10)
        self.robot2_cmd_pub = self.create_publisher(Twist, '/robot2/cmd_vel', 10)

        self.robot1_goal_reached_pub = self.create_publisher(
            Bool, '/robot1/goal_reached', 10
        )
        self.robot2_goal_reached_pub = self.create_publisher(
            Bool, '/robot2/goal_reached', 10
        )

        # Reuses astar_node's own "planning_failed" topic - frontier_explorer
        # already treats that as "blacklist this goal and try another," which
        # is exactly what should happen when a goal times out here too. Without
        # this, frontier_explorer never learns the goal was abandoned and
        # never assigns a replacement, permanently stalling exploration for
        # that robot after a single timeout.
        self.robot1_planning_failed_pub = self.create_publisher(
            Bool, '/robot1/planning_failed', 10
        )
        self.robot2_planning_failed_pub = self.create_publisher(
            Bool, '/robot2/planning_failed', 10
        )

        # NEW - track how long each robot has had no path, for logging
        self.robot1_bootstrap_since = None
        self.robot2_bootstrap_since = None

        # Track how long each robot has been pursuing its current goal, and
        # whether it's given up on it.
        self.robot1_goal_start_time = None
        self.robot2_goal_start_time = None
        self.robot1_goal_abandoned = False
        self.robot2_goal_abandoned = False

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Pure Pursuit Active (with bootstrap gap-following).')

    def robot1_path_callback(self, msg):
        if self.robot1_path is None and len(msg.poses) > 0:
            self.get_logger().info('Robot 1: real path received, leaving bootstrap mode.')
        self.robot1_path = msg
        self.robot1_goal_reached = False
        self.robot1_bootstrap_since = None
        self.robot1_goal_start_time = self.get_clock().now()
        self.robot1_goal_abandoned = False

    def robot2_path_callback(self, msg):
        if self.robot2_path is None and len(msg.poses) > 0:
            self.get_logger().info('Robot 2: real path received, leaving bootstrap mode.')
        self.robot2_path = msg
        self.robot2_goal_reached = False
        self.robot2_bootstrap_since = None
        self.robot2_goal_start_time = self.get_clock().now()
        self.robot2_goal_abandoned = False

    def robot1_odom_callback(self, msg):
        self.robot1_x = msg.pose.pose.position.x
        self.robot1_y = msg.pose.pose.position.y
        self.robot1_yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)

    def robot2_odom_callback(self, msg):
        self.robot2_x = msg.pose.pose.position.x
        self.robot2_y = msg.pose.pose.position.y
        self.robot2_yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)

    def robot1_scan_callback(self, msg):
        self.robot1_scan = msg

    def robot2_scan_callback(self, msg):
        self.robot2_scan = msg

    def control_loop(self):

        # ---- ROBOT 1 ----
        if self.robot1_x is not None and self.robot1_y is not None and self.robot1_yaw is not None:

            if self.robot1_path is not None and len(self.robot1_path.poses) > 0:

                obstacle, direction, obstacle_distance = self.detect_obstacle(self.robot1_scan)

                if obstacle and obstacle_distance < self.emergency_distance:
                    cmd1 = self.reactive_avoidance(direction, obstacle_distance)
                else:
                    cmd1 = self.calculate_control(
                        self.robot1_path, self.robot1_x, self.robot1_y,
                        self.robot1_yaw, robot_number=1
                    )
                    if obstacle:
                        cmd1.linear.x *= 0.3

                self.robot1_cmd_pub.publish(cmd1)

            else:
                # NEW - no path yet (map not connected/available). Bootstrap
                # by gap-following instead of sitting idle, so the robot
                # explores locally and builds enough map for A* to work.
                if self.robot1_bootstrap_since is None:
                    self.robot1_bootstrap_since = self.get_clock().now()
                    self.get_logger().info('Robot 1: no path yet, entering bootstrap gap-following.')

                cmd1 = self.gap_follow(self.robot1_scan)
                self.robot1_cmd_pub.publish(cmd1)

        # ---- ROBOT 2 ----
        if self.robot2_x is not None and self.robot2_y is not None and self.robot2_yaw is not None:

            if self.robot2_path is not None and len(self.robot2_path.poses) > 0:

                obstacle, direction, obstacle_distance = self.detect_obstacle(self.robot2_scan)

                if obstacle and obstacle_distance < self.emergency_distance:
                    cmd2 = self.reactive_avoidance(direction, obstacle_distance)
                else:
                    cmd2 = self.calculate_control(
                        self.robot2_path, self.robot2_x, self.robot2_y,
                        self.robot2_yaw, robot_number=2
                    )
                    if obstacle:
                        cmd2.linear.x *= 0.3

                self.robot2_cmd_pub.publish(cmd2)

            else:
                if self.robot2_bootstrap_since is None:
                    self.robot2_bootstrap_since = self.get_clock().now()
                    self.get_logger().info('Robot 2: no path yet, entering bootstrap gap-following.')

                cmd2 = self.gap_follow(self.robot2_scan)
                self.robot2_cmd_pub.publish(cmd2)

    # ------------------------------------------------------------------
    # NEW - simple gap-following: find the widest open angular sector in
    # the scan and steer toward its center. No map/path needed at all,
    # so this works even with zero prior exploration - exactly the
    # bootstrap behaviour needed before the map has enough connected
    # territory for A* to plan through.
    # ------------------------------------------------------------------
    def gap_follow(self, scan):

        cmd = Twist()

        if scan is None:
            return cmd

        ranges_mm = []
        for r in scan.ranges:
            if math.isfinite(r):
                ranges_mm.append(r * 1000.0)
            else:
                ranges_mm.append(0.0)  # treat inf/nan as "no reading", handled below

        n = len(ranges_mm)
        if n == 0:
            return cmd

        best_start, best_len = 0, 0
        cur_start, cur_len = -1, 0

        for i in range(n):
            clear = (ranges_mm[i] == 0.0) or (ranges_mm[i] > self.bootstrap_safe_distance_mm)
            if clear:
                if cur_start == -1:
                    cur_start = i
                cur_len += 1
            else:
                if cur_len > best_len:
                    best_len, best_start = cur_len, cur_start
                cur_start, cur_len = -1, 0
        if cur_len > best_len:
            best_len, best_start = cur_len, cur_start

        if best_len == 0:
            # Nothing clear anywhere - stop rather than guess.
            return cmd

        gap_center_index = (best_start + best_len // 2) % n
        gap_angle = scan.angle_min + gap_center_index * scan.angle_increment
        gap_angle = math.atan2(math.sin(gap_angle), math.cos(gap_angle))  # normalize

        # Steer toward the gap center. Small forward speed always, plus
        # angular correction proportional to how far off-center it is.
        cmd.linear.x = self.bootstrap_speed
        cmd.angular.z = max(
            -self.max_angular_speed,
            min(self.max_angular_speed, 1.5 * gap_angle)
        )

        return cmd

    # ------------------------------------------------------------------
    def detect_obstacle(self, scan):
        if scan is None:
            return False, None, float('inf')

        ranges = np.array(scan.ranges, dtype=float)
        ranges = np.where(np.isfinite(ranges), ranges, scan.range_max)

        front_ranges = []
        left_ranges = []
        right_ranges = []

        for i, distance in enumerate(ranges):
            angle = scan.angle_min + i * scan.angle_increment
            angle = angle % (2.0 * math.pi)

            if distance < scan.range_min:
                continue
            if distance > scan.range_max:
                continue

            if angle >= math.radians(330) or angle <= math.radians(30):
                front_ranges.append(distance)
            elif math.radians(30) < angle <= math.radians(90):
                left_ranges.append(distance)
            elif math.radians(270) <= angle < math.radians(330):
                right_ranges.append(distance)

        if not front_ranges:
            return False, None, float('inf')

        front_distance = min(front_ranges)

        if front_distance < self.safe_distance:
            left_distance = min(left_ranges) if left_ranges else scan.range_max
            right_distance = min(right_ranges) if right_ranges else scan.range_max
            direction = 'left' if left_distance > right_distance else 'right'
            return True, direction, front_distance

        return False, None, front_distance

    def reactive_avoidance(self, direction, obstacle_distance):
        cmd = Twist()
        if obstacle_distance < self.emergency_distance:
            cmd.linear.x = 0.0
        else:
            cmd.linear.x = 0.05
        if direction == 'left':
            cmd.angular.z = 0.8
        else:
            cmd.angular.z = -0.8
        return cmd

    def calculate_control(self, path, robot_x, robot_y, robot_yaw, robot_number):

        cmd = Twist()
        if len(path.poses) == 0:
            return cmd

        goal_start_time = (
            self.robot1_goal_start_time if robot_number == 1
            else self.robot2_goal_start_time
        )

        if goal_start_time is not None:

            elapsed = (self.get_clock().now() - goal_start_time).nanoseconds / 1e9

            if elapsed > self.goal_pursuit_timeout:

                already_abandoned = (
                    self.robot1_goal_abandoned if robot_number == 1
                    else self.robot2_goal_abandoned
                )

                if not already_abandoned:

                    self.get_logger().warn(
                        f'Robot {robot_number}: giving up on current goal '
                        f'after {elapsed:.1f}s without reaching it.'
                    )

                    fail_msg = Bool()
                    fail_msg.data = True

                    if robot_number == 1:
                        self.robot1_goal_abandoned = True
                        self.robot1_planning_failed_pub.publish(fail_msg)
                    else:
                        self.robot2_goal_abandoned = True
                        self.robot2_planning_failed_pub.publish(fail_msg)

                return cmd

        final_pose = path.poses[-1].pose
        goal_x = final_pose.position.x
        goal_y = final_pose.position.y

        goal_distance = self.distance(robot_x, robot_y, goal_x, goal_y)

        if goal_distance <= self.goal_tolerance:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

            if robot_number == 1:
                if not self.robot1_goal_reached:
                    self.robot1_goal_reached = True
                    self.get_logger().info(f'ROBOT 1 GOAL REACHED: distance={goal_distance:.3f} m')
                    msg = Bool(); msg.data = True
                    self.robot1_goal_reached_pub.publish(msg)
            else:
                if not self.robot2_goal_reached:
                    self.robot2_goal_reached = True
                    self.get_logger().info(f'ROBOT 2 GOAL REACHED: distance={goal_distance:.3f} m')
                    msg = Bool(); msg.data = True
                    self.robot2_goal_reached_pub.publish(msg)

            return cmd

        target = self.find_lookahead_point(path, robot_x, robot_y)
        if target is None:
            return cmd
        target_x, target_y = target

        target_angle = math.atan2(target_y - robot_y, target_x - robot_x)
        angle_error = self.normalize_angle(target_angle - robot_yaw)

        lookahead = self.distance(robot_x, robot_y, target_x, target_y)
        if lookahead < 0.001:
            return cmd

        curvature = 2.0 * math.sin(angle_error) / lookahead
        angular_velocity = self.linear_speed * curvature
        angular_velocity = max(-self.max_angular_speed, min(angular_velocity, self.max_angular_speed))

        speed = self.linear_speed
        angle_size = abs(angle_error)
        if angle_size > 1.0:
            speed *= 0.3
        elif angle_size > 0.6:
            speed *= 0.6
        if speed < self.minimum_linear_speed:
            speed = self.minimum_linear_speed

        cmd.linear.x = speed
        cmd.angular.z = angular_velocity
        return cmd

    def find_lookahead_point(self, path, robot_x, robot_y):
        for pose_stamped in path.poses:
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y
            distance = self.distance(robot_x, robot_y, x, y)
            if distance >= self.lookahead_distance:
                return (x, y)
        final_pose = path.poses[-1].pose
        return (final_pose.position.x, final_pose.position.y)

    def distance(self, x1, y1, x2, y2):
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def quaternion_to_yaw(self, q):
        x, y, z, w = q.x, q.y, q.z, q.w
        sin_yaw = 2.0 * (w * z + x * y)
        cos_yaw = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(sin_yaw, cos_yaw)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuit()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()