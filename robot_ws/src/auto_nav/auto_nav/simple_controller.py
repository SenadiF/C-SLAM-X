
import rclpy
from rclpy.node import Node

import math

from nav_msgs.msg import Path
from geometry_msgs.msg import Twist

from tf2_ros import Buffer, TransformListener


class SimpleController(Node):

    def __init__(self):

        super().__init__('simple_controller')

        # ============================================================
        # PARAMETERS
        # ============================================================

        self.declare_parameter('robot_name', 'robot1')

        self.declare_parameter('linear_speed', 0.20)
        self.declare_parameter('angular_speed', 1.0)

        self.declare_parameter('waypoint_tolerance', 0.12)
        self.declare_parameter('final_goal_tolerance', 0.15)

        self.robot_name = self.get_parameter(
            'robot_name'
        ).value

        self.linear_speed = self.get_parameter(
            'linear_speed'
        ).value

        self.angular_speed = self.get_parameter(
            'angular_speed'
        ).value

        self.waypoint_tolerance = self.get_parameter(
            'waypoint_tolerance'
        ).value

        self.final_goal_tolerance = self.get_parameter(
            'final_goal_tolerance'
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
        # STATE
        # ============================================================

        self.path = None
        self.current_waypoint = 0

        # ============================================================
        # ROS
        # ============================================================

        self.path_sub = self.create_subscription(
            Path,
            f'/{self.robot_name}/planned_path',
            self.path_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            f'/{self.robot_name}/cmd_vel',
            10
        )

        self.create_timer(
            0.05,
            self.control_loop
        )

        self.get_logger().info(
            'Simple autonomous controller ready.'
        )

    # ================================================================
    # PATH CALLBACK
    # ================================================================

    def path_callback(self, msg):

        if len(msg.poses) == 0:
            return

        self.path = msg

        # Start from beginning of new path

        self.current_waypoint = 0

        self.get_logger().info(
            f'Received path with '
            f'{len(msg.poses)} waypoints.'
        )

    # ================================================================
    # GET ROBOT POSE IN MAP
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

        except Exception:

            return None

    # ================================================================
    # STOP
    # ================================================================

    def stop_robot(self):

        cmd = Twist()

        cmd.linear.x = 0.0
        cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)

    # ================================================================
    # CONTROL LOOP
    # ================================================================

    def control_loop(self):

        # ------------------------------------------------------------
        # No path
        # ------------------------------------------------------------

        if self.path is None:
            self.stop_robot()
            return

        if len(self.path.poses) == 0:
            self.stop_robot()
            return

        # ------------------------------------------------------------
        # Robot pose
        # ------------------------------------------------------------

        pose = self.get_robot_pose()

        if pose is None:
            self.stop_robot()
            return

        rx, ry, yaw = pose

        # ------------------------------------------------------------
        # Make sure waypoint index is valid
        # ------------------------------------------------------------

        if self.current_waypoint >= len(self.path.poses):

            self.get_logger().info(
                'Path completed.'
            )

            self.stop_robot()

            self.path = None

            return

        # ------------------------------------------------------------
        # Current waypoint
        # ------------------------------------------------------------

        waypoint = self.path.poses[
            self.current_waypoint
        ]

        wx = waypoint.pose.position.x
        wy = waypoint.pose.position.y

        distance = math.hypot(
            wx - rx,
            wy - ry
        )

        # ------------------------------------------------------------
        # Check waypoint reached
        # ------------------------------------------------------------

        if distance < self.waypoint_tolerance:

            self.current_waypoint += 1

            # Final waypoint reached

            if self.current_waypoint >= len(
                self.path.poses
            ):

                self.get_logger().info(
                    'Final goal reached.'
                )

                self.stop_robot()

                self.path = None

                return

            return

        # ------------------------------------------------------------
        # Direction to waypoint
        # ------------------------------------------------------------

        target_angle = math.atan2(
            wy - ry,
            wx - rx
        )

        # ------------------------------------------------------------
        # Heading error
        # ------------------------------------------------------------

        angle_error = (
            target_angle - yaw
        )

        # Normalize

        angle_error = math.atan2(
            math.sin(angle_error),
            math.cos(angle_error)
        )

        # ------------------------------------------------------------
        # TURN FIRST
        # ------------------------------------------------------------

        cmd = Twist()

        # Large heading error:
        # rotate in place

        if abs(angle_error) > math.radians(25):

            cmd.linear.x = 0.0

            if angle_error > 0:

                cmd.angular.z = self.angular_speed

            else:

                cmd.angular.z = -self.angular_speed

        # ------------------------------------------------------------
        # DRIVE FORWARD
        # ------------------------------------------------------------

        else:

            cmd.linear.x = self.linear_speed

            # Small heading correction

            cmd.angular.z = max(
                -self.angular_speed,
                min(
                    self.angular_speed,
                    2.0 * angle_error
                )
            )

        self.cmd_pub.publish(cmd)

    # ================================================================
    # SHUTDOWN
    # ================================================================

    def destroy_node(self):

        self.stop_robot()

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = SimpleController()

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
