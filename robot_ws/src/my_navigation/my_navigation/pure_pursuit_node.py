
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

        self.declare_parameter(
            'lookahead_distance',
            0.30
        )

        self.declare_parameter(
            'linear_speed',
            0.15
        )

        self.declare_parameter(
            'max_angular_speed',
            1.5
        )

        self.declare_parameter(
            'goal_tolerance',
            0.10
        )

        self.declare_parameter(
            'minimum_linear_speed',
            0.05
        )

        self.declare_parameter(
            'emergency_distance',
            0.15
        )

        self.declare_parameter(
            'safe_distance',
            0.30
        )

        self.lookahead_distance = (
            self.get_parameter(
                'lookahead_distance'
            ).value
        )

        self.linear_speed = (
            self.get_parameter(
                'linear_speed'
            ).value
        )

        self.max_angular_speed = (
            self.get_parameter(
                'max_angular_speed'
            ).value
        )

        self.goal_tolerance = (
            self.get_parameter(
                'goal_tolerance'
            ).value
        )

        self.minimum_linear_speed = (
            self.get_parameter(
                'minimum_linear_speed'
            ).value
        )

        self.emergency_distance = (
            self.get_parameter(
                'emergency_distance'
            ).value
        )

        self.safe_distance = (
            self.get_parameter(
                'safe_distance'
            ).value
        )

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
            Path,
            '/robot1/planned_path',
            self.robot1_path_callback,
            10
        )

        self.robot2_path_sub = self.create_subscription(
            Path,
            '/robot2/planned_path',
            self.robot2_path_callback,
            10
        )
        self.robot1_scan_sub = self.create_subscription(
            LaserScan,
            '/robot1/scan',
            self.robot1_scan_callback,
            10
        )
        self.robot2_scan_sub = self.create_subscription(
            LaserScan,
            '/robot2/scan',
            self.robot2_scan_callback,
            10
        )
        self.robot1_odom_sub = self.create_subscription(
            Odometry,
            '/robot1/odometry/filtered',
            self.robot1_odom_callback,
            10
        )
        self.robot2_odom_sub = self.create_subscription(
            Odometry,
            '/robot2/odometry/filtered',
            self.robot2_odom_callback,
            10
        )
        self.robot1_cmd_pub = self.create_publisher(
            Twist,
            '/robot1/cmd_vel',
            10
        )

        self.robot2_cmd_pub = self.create_publisher(
            Twist,
            '/robot2/cmd_vel',
            10
        )
        self.robot1_goal_reached_pub = (
            self.create_publisher(
                Bool,
                '/robot1/goal_reached',
                10
            )
        )

        self.robot2_goal_reached_pub = (
            self.create_publisher(
                Bool,
                '/robot2/goal_reached',
                10
            )
        )
        self.timer = self.create_timer(
            0.05,
            self.control_loop
        )

        self.get_logger().info(
            'Pure Pursuit  Active.'
        )

    def robot1_path_callback(self, msg):

        self.robot1_path = msg

        self.robot1_goal_reached = False

        self.get_logger().info(
            f'New Robot 1 path received: '
            f'{len(msg.poses)} poses'
        )

    def robot2_path_callback(self, msg):

        self.robot2_path = msg

        self.robot2_goal_reached = False

        self.get_logger().info(
            f'New Robot 2 path received: '
            f'{len(msg.poses)} poses'
        )
    def robot1_odom_callback(self, msg):

        self.robot1_x = (
            msg.pose.pose.position.x
        )

        self.robot1_y = (
            msg.pose.pose.position.y
        )

        self.robot1_yaw = (
            self.quaternion_to_yaw(
                msg.pose.pose.orientation
            )
        )

    def robot2_odom_callback(self, msg):

        self.robot2_x = (
            msg.pose.pose.position.x
        )

        self.robot2_y = (
            msg.pose.pose.position.y
        )

        self.robot2_yaw = (
            self.quaternion_to_yaw(
                msg.pose.pose.orientation
            )
        )

    def robot1_scan_callback(self, msg):

        self.robot1_scan = msg

    def robot2_scan_callback(self, msg):

        self.robot2_scan = msg

    def control_loop(self):


        if (
            self.robot1_path is not None
            and self.robot1_x is not None
            and self.robot1_y is not None
            and self.robot1_yaw is not None
        ):

            obstacle, direction, obstacle_distance = (
                self.detect_obstacle(
                    self.robot1_scan
                )
            )

            if (
                obstacle
                and obstacle_distance
                < self.emergency_distance
            ):

                cmd1 = self.reactive_avoidance(
                    direction,
                    obstacle_distance
                )

            else:

                cmd1 = self.calculate_control(
                    self.robot1_path,
                    self.robot1_x,
                    self.robot1_y,
                    self.robot1_yaw,
                    robot_number=1
                )
 #Slow down if there is an obstacle in front

                if obstacle:

                    cmd1.linear.x *= 0.3

            self.robot1_cmd_pub.publish(
                cmd1
            )

        if (
            self.robot2_path is not None
            and self.robot2_x is not None
            and self.robot2_y is not None
            and self.robot2_yaw is not None
        ):

            obstacle, direction, obstacle_distance = (
                self.detect_obstacle(
                    self.robot2_scan
                )
            )

            if (
                obstacle
                and obstacle_distance
                < self.emergency_distance
            ):

                cmd2 = self.reactive_avoidance(
                    direction,
                    obstacle_distance
                )

            else:

                cmd2 = self.calculate_control(
                    self.robot2_path,
                    self.robot2_x,
                    self.robot2_y,
                    self.robot2_yaw,
                    robot_number=2
                )

                if obstacle:

                    cmd2.linear.x *= 0.3

            self.robot2_cmd_pub.publish(
                cmd2
            )
#Obstacle detection function to check for obstacles in front of the robot using LIDAR data

    def detect_obstacle(self, scan):

        if scan is None:

            return (
                False,
                None,
                float('inf')
            )

        ranges = np.array(
            scan.ranges,
            dtype=float
        )

        # Replace invalid readings with max range
        ranges = np.where(
            np.isfinite(ranges),
            ranges,
            scan.range_max
        )

        front_ranges = []
        left_ranges = []
        right_ranges = []
        #Lidar data 

        for i, distance in enumerate(
            ranges
        ):

            angle = (
                scan.angle_min
                + i * scan.angle_increment
            )

            # Convert to [0, 2*pi]
            angle = (
                angle
                % (2.0 * math.pi)
            )

            # Ignore invalid distance
            if (
                distance
                < scan.range_min
            ):

                continue

            if (
                distance
                > scan.range_max
            ):

                continue
 #Front from 30 to -30 

            if (
                angle >= math.radians(330)
                or angle <= math.radians(30)
            ):

                front_ranges.append(
                    distance
                )
#Left from 30 to 90 

            elif (
                angle > math.radians(30)
                and angle <= math.radians(90)
            ):

                left_ranges.append(
                    distance
                )
#Right from -30 to -90

            elif (
                angle >= math.radians(270)
                and angle < math.radians(330)
            ):

                right_ranges.append(
                    distance
                )


        if not front_ranges:

            return (
                False,
                None,
                float('inf')
            )

        # Closest front obstacle
        front_distance = min(
            front_ranges
        )


        if (
            front_distance
            < self.safe_distance
        ):

            left_distance = (
                min(left_ranges)
                if left_ranges
                else scan.range_max
            )

            right_distance = (
                min(right_ranges)
                if right_ranges
                else scan.range_max
            )

            # Turn toward the side
            # with MORE free space.

            if (
                left_distance
                > right_distance
            ):

                direction = 'left'

            else:

                direction = 'right'

            return (
                True,
                direction,
                front_distance
            )

        return (
            False,
            None,
            front_distance
        )
#Reactive obstacle avoidance 

    def reactive_avoidance(
        self,
        direction,
        obstacle_distance
    ):

        cmd = Twist()
#If obstacle vwry close linear becomes zero

        if (
            obstacle_distance
            < self.emergency_distance
        ):

            cmd.linear.x = 0.0

        else:

            cmd.linear.x = 0.05
#Turn away from the obstacle based on the direction determined by the detect_obstacle function

        if direction == 'left':

            cmd.angular.z = 0.8

        else:

            cmd.angular.z = -0.8

        return cmd

    def calculate_control(
        self,
        path,
        robot_x,
        robot_y,
        robot_yaw,
        robot_number
    ):

        cmd = Twist()

        if len(path.poses) == 0:

            return cmd


        final_pose = (
            path.poses[-1].pose
        )

        goal_x = (
            final_pose.position.x
        )

        goal_y = (
            final_pose.position.y
        )

        goal_distance = self.distance(
            robot_x,
            robot_y,
            goal_x,
            goal_y
        )

#Check whether the goal is reached 
        if (
            goal_distance
            <= self.goal_tolerance
        ):

            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

            if robot_number == 1:

                if not self.robot1_goal_reached:

                    self.robot1_goal_reached = True

                    self.get_logger().info(
                        f'ROBOT 1 GOAL REACHED: '
                        f'distance='
                        f'{goal_distance:.3f} m'
                    )

                    msg = Bool()
                    msg.data = True

                    self.robot1_goal_reached_pub.publish(
                        msg
                    )

            else:

                if not self.robot2_goal_reached:

                    self.robot2_goal_reached = True

                    self.get_logger().info(
                        f'ROBOT 2 GOAL REACHED: '
                        f'distance='
                        f'{goal_distance:.3f} m'
                    )

                    msg = Bool()
                    msg.data = True

                    self.robot2_goal_reached_pub.publish(
                        msg
                    )

            return cmd
#Lookahead point selection

        target = self.find_lookahead_point(
            path,
            robot_x,
            robot_y
        )

        if target is None:

            return cmd

        target_x, target_y = target
#Target angle 

        target_angle = math.atan2(
            target_y - robot_y,
            target_x - robot_x
        )

        angle_error = (
            target_angle
            - robot_yaw
        )

        angle_error = (
            self.normalize_angle(
                angle_error
            )
        )
     # LOOKAHEAD DISTANCE

        lookahead = self.distance(
            robot_x,
            robot_y,
            target_x,
            target_y
        )

        if lookahead < 0.001:

            return cmd
#Calculate curvature 
        curvature = (
            2.0
            * math.sin(angle_error)
            / lookahead
        )

        angular_velocity = (
            self.linear_speed
            * curvature
        )


        angular_velocity = max(
            -self.max_angular_speed,
            min(
                angular_velocity,
                self.max_angular_speed
            )
        )
#linear speed 

        speed = self.linear_speed

        angle_size = abs(
            angle_error
        )

        if angle_size > 1.0:

            speed *= 0.3

        elif angle_size > 0.6:

            speed *= 0.6

        if (
            speed
            < self.minimum_linear_speed
        ):

            speed = (
                self.minimum_linear_speed
            )


        cmd.linear.x = speed
        cmd.angular.z = angular_velocity

        return cmd

    def find_lookahead_point(
        self,
        path,
        robot_x,
        robot_y
    ):

        for pose_stamped in path.poses:

            x = (
                pose_stamped.pose
                .position.x
            )

            y = (
                pose_stamped.pose
                .position.y
            )

            distance = self.distance(
                robot_x,
                robot_y,
                x,
                y
            )

            if (
                distance
                >= self.lookahead_distance
            ):

                return (
                    x,
                    y
                )

        # If no point is far enough,
        # use final point.

        final_pose = (
            path.poses[-1].pose
        )

        return (
            final_pose.position.x,
            final_pose.position.y
        )


    def distance(
        self,
        x1,
        y1,
        x2,
        y2
    ):

        return math.sqrt(
            (x2 - x1) ** 2
            + (y2 - y1) ** 2
        )


    def normalize_angle(
        self,
        angle
    ):

        while angle > math.pi:

            angle -= (
                2.0 * math.pi
            )

        while angle < -math.pi:

            angle += (
                2.0 * math.pi
            )

        return angle
#Quaternion to yaw conversion function to extract the yaw angle from a quaternion orientation

    def quaternion_to_yaw(
        self,
        q
    ):

        x = q.x
        y = q.y
        z = q.z
        w = q.w

        sin_yaw = (
            2.0
            * (
                w * z
                + x * y
            )
        )

        cos_yaw = (
            1.0
            - 2.0
            * (
                y * y
                + z * z
            )
        )

        return math.atan2(
            sin_yaw,
            cos_yaw
        )


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

