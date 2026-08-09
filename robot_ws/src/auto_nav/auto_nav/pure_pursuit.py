
import rclpy
from rclpy.node import Node

import numpy as np
import math

from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from tf2_ros import Buffer, TransformListener


class PurePursuit(Node):

    def __init__(self):
        super().__init__('pure_pursuit')

        # ============================================================
        # PARAMETERS
        # ============================================================

        self.declare_parameter('robot_name', 'robot1')
        self.declare_parameter('base_lookahead', 0.40)
        self.declare_parameter('max_linear_speed', 0.30)
        self.declare_parameter('max_angular_speed', 1.5)
        self.declare_parameter('goal_tolerance', 0.20)

        # IMPORTANT:
        # Keep this reasonably small because frontier goals can
        # occasionally be close to obstacles.
        self.declare_parameter('obstacle_stop_distance', 0.20)
        self.declare_parameter('obstacle_slow_distance', 0.50)

        self.robot_name = self.get_parameter('robot_name').value

        self.base_lookahead = self.get_parameter(
            'base_lookahead'
        ).value

        self.max_v = self.get_parameter(
            'max_linear_speed'
        ).value

        self.max_w = self.get_parameter(
            'max_angular_speed'
        ).value

        self.goal_tol = self.get_parameter(
            'goal_tolerance'
        ).value

        self.obstacle_stop = self.get_parameter(
            'obstacle_stop_distance'
        ).value

        self.obstacle_slow = self.get_parameter(
            'obstacle_slow_distance'
        ).value

        # ============================================================
        # FRAMES
        # ============================================================

        self.map_frame = 'map'
        self.base_frame = f'{self.robot_name}/base_link'

        # ============================================================
        # STATE
        # ============================================================

        self.path = None

        self.robot_x = None
        self.robot_y = None
        self.yaw = 0.0

        self.min_scan_range = float('inf')

        # ============================================================
        # TF
        # ============================================================

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        # ============================================================
        # SUBSCRIBERS
        # ============================================================

        self.path_sub = self.create_subscription(
            Path,
            f'/{self.robot_name}/planned_path',
            self.path_callback,
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            f'/{self.robot_name}/scan',
            self.scan_callback,
            10
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
        # CONTROL LOOP
        # ============================================================

        self.create_timer(
            0.05,
            self.control_loop
        )

        self.get_logger().info(
            'Pure Pursuit Tracking Engine Active.'
        )

    # ================================================================
    # PATH CALLBACK
    # ================================================================

    def path_callback(self, msg):

        if len(msg.poses) == 0:
            self.get_logger().warn(
                'Received empty path.'
            )
            return

        self.path = msg

        self.get_logger().info(
            f'Received path with {len(msg.poses)} points.'
        )

    # ================================================================
    # LASER CALLBACK
    # ================================================================

    def scan_callback(self, msg):

        ranges = np.array(
            msg.ranges,
            dtype=np.float32
        )

        angles = (
            np.arange(len(ranges))
            * msg.angle_increment
            + msg.angle_min
        )

        # Forward region: +/- 30 degrees

        stop_region = (
            (angles >= -math.pi / 6)
            &
            (angles <= math.pi / 6)
        )

        valid = (
            stop_region
            &
            np.isfinite(ranges)
            &
            (ranges >= msg.range_min)
            &
            (ranges <= msg.range_max)
        )

        if np.any(valid):
            self.min_scan_range = float(
                np.min(ranges[valid])
            )
        else:
            self.min_scan_range = float('inf')

    # ================================================================
    # GET ROBOT POSE IN MAP FRAME
    # ================================================================

    def update_robot_pose(self):

        try:

            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time()
            )

            self.robot_x = (
                transform.transform.translation.x
            )

            self.robot_y = (
                transform.transform.translation.y
            )

            q = transform.transform.rotation

            siny = 2.0 * (
                q.w * q.z
                +
                q.x * q.y
            )

            cosy = 1.0 - 2.0 * (
                q.y * q.y
                +
                q.z * q.z
            )

            self.yaw = math.atan2(
                siny,
                cosy
            )

            return True

        except Exception as e:

            self.get_logger().warn(
                f'Waiting for TF map -> {self.base_frame}'
            )

            return False

    # ================================================================
    # FIND LOOKAHEAD POINT
    # ================================================================

    def find_lookahead_point(self, lookahead):

        if self.path is None:
            return None

        if len(self.path.poses) == 0:
            return None

        rx = self.robot_x
        ry = self.robot_y

        # Find first point farther than lookahead distance

        for pose in self.path.poses:

            px = pose.pose.position.x
            py = pose.pose.position.y

            distance = math.hypot(
                px - rx,
                py - ry
            )

            if distance >= lookahead:
                return px, py

        # If no point is far enough,
        # use final point.

        final_pose = self.path.poses[-1]

        return (
            final_pose.pose.position.x,
            final_pose.pose.position.y
        )

    # ================================================================
    # CONTROL LOOP
    # ================================================================

    def control_loop(self):

        # ------------------------------------------------------------
        # Get robot pose in MAP frame
        # ------------------------------------------------------------

        if not self.update_robot_pose():
            return

        # ------------------------------------------------------------
        # No path
        # ------------------------------------------------------------

        if self.path is None:
            self.stop_robot()
            return

        if len(self.path.poses) == 0:
            self.stop_robot()
            return

        rx = self.robot_x
        ry = self.robot_y

        # ------------------------------------------------------------
        # Check final goal
        # ------------------------------------------------------------

        goal_pose = self.path.poses[-1]

        gx = goal_pose.pose.position.x
        gy = goal_pose.pose.position.y

        goal_distance = math.hypot(
            gx - rx,
            gy - ry
        )

        if goal_distance < self.goal_tol:

            self.get_logger().info(
                'Goal reached. Stopping robot.'
            )

            self.stop_robot()

            self.path = None

            return

        # ------------------------------------------------------------
        # Emergency obstacle stop
        # ------------------------------------------------------------

        if self.min_scan_range < self.obstacle_stop:

            self.get_logger().warn(
                f'Obstacle detected at '
                f'{self.min_scan_range:.2f} m. STOP.'
            )

            self.stop_robot()

            return

        # ------------------------------------------------------------
        # Adaptive lookahead
        # ------------------------------------------------------------

        lookahead = self.base_lookahead

        if self.min_scan_range < self.obstacle_slow:

            ratio = (
                self.min_scan_range
                / self.obstacle_slow
            )

            ratio = max(
                0.4,
                min(1.0, ratio)
            )

            lookahead *= ratio

        lookahead = max(
            0.20,
            min(0.55, lookahead)
        )

        # ------------------------------------------------------------
        # Find target
        # ------------------------------------------------------------

        target = self.find_lookahead_point(
            lookahead
        )

        if target is None:
            self.stop_robot()
            return

        tx, ty = target

        # ------------------------------------------------------------
        # Calculate heading error
        # ------------------------------------------------------------

        target_angle = math.atan2(
            ty - ry,
            tx - rx
        )

        angle_error = (
            target_angle
            - self.yaw
        )

        # Normalize to [-pi, pi]

        angle_error = math.atan2(
            math.sin(angle_error),
            math.cos(angle_error)
        )

        # ------------------------------------------------------------
        # Velocity control
        # ------------------------------------------------------------

        abs_error = abs(angle_error)

        # If robot needs a very large turn,
        # rotate slowly instead of driving forward.

        if abs_error > math.radians(70):

            linear_velocity = 0.0

        elif abs_error > math.radians(40):

            linear_velocity = 0.05

        else:

            linear_velocity = self.max_v

        # Slow down near obstacles

        if self.min_scan_range < self.obstacle_slow:

            obstacle_factor = (
                self.min_scan_range
                / self.obstacle_slow
            )

            obstacle_factor = max(
                0.25,
                min(1.0, obstacle_factor)
            )

            linear_velocity *= obstacle_factor

        # ------------------------------------------------------------
        # Angular velocity
        # ------------------------------------------------------------

        angular_velocity = (
            2.5 * angle_error
        )

        angular_velocity = max(
            -self.max_w,
            min(
                self.max_w,
                angular_velocity
            )
        )

        # ------------------------------------------------------------
        # Publish command
        # ------------------------------------------------------------

        cmd = Twist()

        cmd.linear.x = linear_velocity
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0

        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = angular_velocity

        self.cmd_pub.publish(cmd)

        # Debug information

        self.get_logger().debug(
            f'Robot=({rx:.2f},{ry:.2f}) '
            f'Target=({tx:.2f},{ty:.2f}) '
            f'Error={math.degrees(angle_error):.1f}deg '
            f'V={linear_velocity:.2f} '
            f'W={angular_velocity:.2f}'
        )

    # ================================================================
    # STOP
    # ================================================================

    def stop_robot(self):

        self.cmd_pub.publish(
            Twist()
        )


# ====================================================================
# MAIN
# ====================================================================

def main(args=None):

    rclpy.init(args=args)

    node = PurePursuit()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.stop_robot()

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

