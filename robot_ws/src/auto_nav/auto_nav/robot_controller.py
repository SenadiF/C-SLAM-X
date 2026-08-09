import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan

import math
import numpy as np

from tf2_ros import Buffer, TransformListener


class RobotController(Node):

    def __init__(self):

        super().__init__('robot_controller')

        # ============================================================
        # PARAMETERS
        # ============================================================

        self.declare_parameter('robot_name', 'robot1')
        self.declare_parameter('linear_speed', 0.12)
        self.declare_parameter('angular_speed', 0.5)
        self.declare_parameter('safe_distance', 0.35)
        self.declare_parameter('goal_tolerance', 0.15)

        self.robot_name = self.get_parameter(
            'robot_name'
        ).value

        self.linear_speed = self.get_parameter(
            'linear_speed'
        ).value

        self.angular_speed = self.get_parameter(
            'angular_speed'
        ).value

        self.safe_distance = self.get_parameter(
            'safe_distance'
        ).value

        self.goal_tolerance = self.get_parameter(
            'goal_tolerance'
        ).value

        # ============================================================
        # FRAMES
        # ============================================================

        self.map_frame = 'map'
        self.base_frame = f'{self.robot_name}/base_link'

        # ============================================================
        # TF
        # ============================================================

        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        # ============================================================
        # PUBLISHER
        # ============================================================

        self.cmd_pub = self.create_publisher(
            Twist,
            f'/{self.robot_name}/cmd_vel',
            10
        )

        # ============================================================
        # SUBSCRIBERS
        # ============================================================

        self.scan_sub = self.create_subscription(
            LaserScan,
            f'/{self.robot_name}/scan',
            self.scan_callback,
            10
        )

        self.goal_sub = self.create_subscription(
            PoseStamped,
            f'/{self.robot_name}/goal_pose',
            self.goal_callback,
            10
        )

        # ============================================================
        # STATE
        # ============================================================

        self.scan = None
        self.goal = None

        self.goal_active = False

        self.last_goal_time = self.get_clock().now()

        # ============================================================
        # CONTROL LOOP
        # ============================================================

        self.timer = self.create_timer(
            0.05,
            self.control_loop
        )

        self.get_logger().info(
            f'RobotController ready for {self.robot_name}'
        )

    # ================================================================
    # LASER
    # ================================================================

    def scan_callback(self, msg):

        self.scan = msg

    # ================================================================
    # GOAL
    # ================================================================

    def goal_callback(self, msg):

        self.goal = (
            msg.pose.position.x,
            msg.pose.position.y
        )

        self.goal_active = True

        self.last_goal_time = self.get_clock().now()

        self.get_logger().info(
            f'New goal: ({self.goal[0]:.2f}, {self.goal[1]:.2f})'
        )

    # ================================================================
    # ROBOT POSE IN MAP
    # ================================================================

    def get_robot_pose(self):

        try:

            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time()
            )

            x = transform.transform.translation.x
            y = transform.transform.translation.y

            q = transform.transform.rotation

            siny = 2.0 * (
                q.w * q.z +
                q.x * q.y
            )

            cosy = 1.0 - 2.0 * (
                q.y * q.y +
                q.z * q.z
            )

            yaw = math.atan2(
                siny,
                cosy
            )

            return x, y, yaw

        except Exception as e:

            return None

    # ================================================================
    # OBSTACLE DETECTION
    # ================================================================

    def get_obstacle_direction(self):

        if self.scan is None:
            return False, 0.0

        ranges = np.array(
            self.scan.ranges,
            dtype=float
        )

        ranges[
            ~np.isfinite(ranges)
        ] = self.scan.range_max

        if len(ranges) < 10:
            return False, 0.0

        # LaserScan angles are used directly.
        angles = (
            self.scan.angle_min +
            np.arange(len(ranges)) *
            self.scan.angle_increment
        )

        # Front sector ±30 degrees

        front_mask = (
            np.abs(angles) < math.radians(30)
        )

        front = ranges[front_mask]

        if len(front) == 0:
            return False, 0.0

        min_front = np.min(front)

        if min_front >= self.safe_distance:
            return False, 0.0

        # Compare left and right free space

        left_mask = (
            (angles > math.radians(30)) &
            (angles < math.radians(120))
        )

        right_mask = (
            (angles < math.radians(-30)) &
            (angles > math.radians(-120))
        )

        left = ranges[left_mask]
        right = ranges[right_mask]

        left_mean = (
            np.mean(left)
            if len(left) > 0
            else 0.0
        )

        right_mean = (
            np.mean(right)
            if len(right) > 0
            else 0.0
        )

        # Positive = turn left
        # Negative = turn right

        if left_mean > right_mean:

            return True, 1.0

        else:

            return True, -1.0

    # ================================================================
    # CONTROL
    # ================================================================

    def control_loop(self):

        cmd = Twist()

        # ------------------------------------------------------------
        # No goal
        # ------------------------------------------------------------

        if not self.goal_active or self.goal is None:

            self.cmd_pub.publish(cmd)

            return

        # ------------------------------------------------------------
        # Get pose from TF
        # ------------------------------------------------------------

        pose = self.get_robot_pose()

        if pose is None:

            self.get_logger().warn(
                'Waiting for map -> base_link TF...',
                throttle_duration_sec=2.0
            )

            self.cmd_pub.publish(cmd)

            return

        rx, ry, yaw = pose

        gx, gy = self.goal

        # ------------------------------------------------------------
        # Distance to goal
        # ------------------------------------------------------------

        dx = gx - rx
        dy = gy - ry

        distance = math.hypot(dx, dy)

        # ------------------------------------------------------------
        # Goal reached
        # ------------------------------------------------------------

        if distance < self.goal_tolerance:

            self.get_logger().info(
                'GOAL REACHED!'
            )

            self.goal_active = False

            self.goal = None

            self.cmd_pub.publish(cmd)

            return

        # ------------------------------------------------------------
        # Goal direction
        # ------------------------------------------------------------

        target_angle = math.atan2(
            dy,
            dx
        )

        angle_error = (
            target_angle - yaw
        )

        angle_error = math.atan2(
            math.sin(angle_error),
            math.cos(angle_error)
        )

        # ------------------------------------------------------------
        # OBSTACLE
        # ------------------------------------------------------------

        obstacle, direction = \
            self.get_obstacle_direction()

        if obstacle:

            # Stop forward motion.
            cmd.linear.x = 0.0

            if direction > 0:

                cmd.angular.z = self.angular_speed

            else:

                cmd.angular.z = -self.angular_speed

            self.cmd_pub.publish(cmd)

            return

        # ------------------------------------------------------------
        # TURN TOWARD GOAL
        # ------------------------------------------------------------

        if abs(angle_error) > math.radians(25):

            cmd.linear.x = 0.0

            if angle_error > 0:

                cmd.angular.z = self.angular_speed

            else:

                cmd.angular.z = -self.angular_speed

        # ------------------------------------------------------------
        # MOVE TOWARD GOAL
        # ------------------------------------------------------------

        else:

            cmd.linear.x = self.linear_speed

            # proportional steering

            cmd.angular.z = (
                1.5 * angle_error
            )

            cmd.angular.z = max(
                -self.angular_speed,
                min(
                    self.angular_speed,
                    cmd.angular.z
                )
            )

        self.cmd_pub.publish(cmd)

    # ================================================================
    # SHUTDOWN
    # ================================================================

    def destroy_node(self):

        cmd = Twist()

        self.cmd_pub.publish(cmd)

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = RobotController()

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