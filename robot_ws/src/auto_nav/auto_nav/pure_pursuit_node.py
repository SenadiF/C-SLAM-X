import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

import math

class PurePursuit(Node):

    def __init__(self):


        super().__init__(
            'pure_pursuit'
        )

        self.declare_parameter(
            'lookahead_distance',
            0.30
        )


        self.declare_parameter(
            'linear_speed',
            0.60
        )

        self.declare_parameter(
            'max_angular_speed',
            1.5
        )
        self.declare_parameter(
            'goal_tolerance',
            0.15
        )

        self.declare_parameter(
            'minimum_linear_speed',
            0.05
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


        # Current Robot 1 path
        self.robot1_path = None

        # Robot 1 position
        self.robot1_x = None
        self.robot1_y = None
        self.robot1_yaw = None


        self.robot2_path = None

        self.robot2_x = None
        self.robot2_y = None

        self.robot2_yaw = None



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
        # Robot 1 velocity command
        self.robot1_cmd_pub = self.create_publisher(
            Twist,
            '/robot1/cmd_vel',
            10
        )


        # Robot 2 velocity command
        self.robot2_cmd_pub = self.create_publisher(
            Twist,
            '/robot2/cmd_vel',
            10
        )
        self.timer = self.create_timer(
            0.05,
            self.control_loop
        )


        self.get_logger().info(
            'Pure Pursuit Tracking Engine Active.'
        )
    def robot1_path_callback(
        self,
        msg
    ):

        # Save the newest path.
        self.robot1_path = msg

    def robot2_path_callback(
        self,
        msg
    ):

        self.robot2_path = msg
    def robot1_odom_callback(
        self,
        msg
    ):



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
    def robot2_odom_callback(
        self,
        msg
    ):

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


    def control_loop(self):



        if (
            self.robot1_path is not None
            and
            self.robot1_x is not None
            and
            self.robot1_yaw is not None
        ):

            cmd1 = self.calculate_control(

                self.robot1_path,

                self.robot1_x,
                self.robot1_y,
                self.robot1_yaw
            )


            self.robot1_cmd_pub.publish(
                cmd1
            )
        if (
            self.robot2_path is not None
            and
            self.robot2_x is not None
            and
            self.robot2_yaw is not None
        ):

            cmd2 = self.calculate_control(

                self.robot2_path,

                self.robot2_x,
                self.robot2_y,
                self.robot2_yaw
            )


            self.robot2_cmd_pub.publish(
                cmd2
            )
    def calculate_control(
        self,
        path,
        robot_x,
        robot_y,
        robot_yaw
    ):

        cmd = Twist()
        if len(path.poses) == 0:

            return cmd
        final_pose = path.poses[-1].pose


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
        if goal_distance < self.goal_tolerance:


            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

            return cmd

        target = self.find_lookahead_point(

            path,

            robot_x,
            robot_y
        )


        if target is None:

            return cmd


        target_x, target_y = target


        # Angle from robot to target.
        target_angle = math.atan2(

            target_y - robot_y,

            target_x - robot_x

        )

        angle_error = (
            target_angle
            -
            robot_yaw
        )


        angle_error = self.normalize_angle(
            angle_error
        )


        lookahead = self.distance(

            robot_x,
            robot_y,

            target_x,
            target_y
        )


        # Prevent division by zero.
        if lookahead < 0.001:

            return cmd


        curvature = (

            2.0
            *
            math.sin(angle_error)
            /
            lookahead

        )

        angular_velocity = (

            self.linear_speed
            *
            curvature

        )


        angular_velocity = max(

            -self.max_angular_speed,

            min(
                angular_velocity,
                self.max_angular_speed
            )

        )

        speed = self.linear_speed


        angle_size = abs(
            angle_error
        )


        if angle_size > 1.0:

            speed *= 0.3

        elif angle_size > 0.6:

            speed *= 0.6


        if speed < self.minimum_linear_speed:

            speed = self.minimum_linear_speed
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
                pose_stamped.pose.position.x
            )

            y = (
                pose_stamped.pose.position.y
            )


            distance = self.distance(

                robot_x,
                robot_y,

                x,
                y
            )


            if distance >= self.lookahead_distance:

                return (
                    x,
                    y
                )


        final_pose = path.poses[-1].pose


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
            +
            (y2 - y1) ** 2

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
            *
            (
                w * z
                +
                x * y
            )

        )


        cos_yaw = (

            1.0
            -
            2.0
            *
            (
                y * y
                +
                z * z
            )

        )


        yaw = math.atan2(

            sin_yaw,
            cos_yaw

        )


        return yaw


def main(args=None):


    rclpy.init(
        args=args
    )

    node = PurePursuit()


    try:

      
        rclpy.spin(
            node
        )


    except KeyboardInterrupt:

        pass


    finally:

        
        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':

    main()