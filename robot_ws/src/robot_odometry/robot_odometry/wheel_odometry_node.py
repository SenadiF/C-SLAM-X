import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

from tf_transformations import quaternion_from_euler


class WheelOdometryNode(Node):

    def __init__(self):
        super().__init__('wheel_odometry_node')




        self.declare_parameter('robot_name', 'robot1')

        # Distance between left and right wheels
        self.declare_parameter('wheel_base', 0.099)

        # Calibrated encoder value
        self.declare_parameter('ticks_per_meter', 13313.0)

        self.declare_parameter('left_encoder_sign', -1.0)
        self.declare_parameter('right_encoder_sign', 1.0)

        self.robot_name = self.get_parameter(
            'robot_name'
        ).value

        self.wheel_base = float(
            self.get_parameter('wheel_base').value
        )

        self.ticks_per_meter = float(
            self.get_parameter('ticks_per_meter').value
        )

        self.left_sign = float(
            self.get_parameter('left_encoder_sign').value
        )

        self.right_sign = float(
            self.get_parameter('right_encoder_sign').value
        )




        self.initialized = False

        self.prev_left_ticks = 0
        self.prev_right_ticks = 0

        # Robot pose in odom frame
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Velocities
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        self.last_time = self.get_clock().now()




        encoder_topic = f'/{self.robot_name}/encoder'
        odom_topic = f'/{self.robot_name}/wheel_odom'

        self.encoder_subscriber = self.create_subscription(
            Int32MultiArray,
            encoder_topic,
            self.encoder_callback,
            10
        )

        self.odom_publisher = self.create_publisher(
            Odometry,
            odom_topic,
            10
        )




        self.get_logger().info(
            'Wheel odometry node started.'
        )

        self.get_logger().info(
            f'Robot: {self.robot_name}'
        )

        self.get_logger().info(
            f'Wheel base: {self.wheel_base:.4f} m'
        )

        self.get_logger().info(
            f'Ticks per meter: {self.ticks_per_meter:.2f}'
        )

        self.get_logger().info(
            f'Encoder signs: L={self.left_sign}, '
            f'R={self.right_sign}'
        )




    def encoder_callback(self, msg):




        if len(msg.data) < 2:
            self.get_logger().warning(
                'Encoder message does not contain two values.'
            )
            return

        current_left_ticks = int(msg.data[0])
        current_right_ticks = int(msg.data[1])

        current_time = self.get_clock().now()




        if not self.initialized:

            self.prev_left_ticks = current_left_ticks
            self.prev_right_ticks = current_right_ticks

            self.last_time = current_time

            self.initialized = True

            self.get_logger().info(
                f'Initial encoder values: '
                f'L={current_left_ticks}, '
                f'R={current_right_ticks}'
            )

            # Publish initial zero odometry
            self.publish_odometry()

            return


        raw_left_diff = (
            current_left_ticks -
            self.prev_left_ticks
        )

        raw_right_diff = (
            current_right_ticks -
            self.prev_right_ticks
        )

        # Apply wheel direction signs
        left_tick_diff = (
            self.left_sign * raw_left_diff
        )

        right_tick_diff = (
            self.right_sign * raw_right_diff
        )



        dt = (
            current_time - self.last_time
        ).nanoseconds / 1e9

        if dt <= 0.0:
            return




        left_distance = (
            left_tick_diff /
            self.ticks_per_meter
        )

        right_distance = (
            right_tick_diff /
            self.ticks_per_meter
        )

       

        distance = (
            left_distance +
            right_distance
        ) / 2.0

        delta_theta = (
            right_distance -
            left_distance
        ) / self.wheel_base

    

        theta_mid = (
            self.theta +
            delta_theta / 2.0
        )

        self.x += (
            distance *
            math.cos(theta_mid)
        )

        self.y += (
            distance *
            math.sin(theta_mid)
        )

        self.theta += delta_theta

        # Normalize angle to [-pi, pi]
        self.theta = math.atan2(
            math.sin(self.theta),
            math.cos(self.theta)
        )

        


        self.linear_velocity = distance / dt

        self.angular_velocity = delta_theta / dt


        self.prev_left_ticks = current_left_ticks
        self.prev_right_ticks = current_right_ticks

        self.last_time = current_time





        self.publish_odometry()

    
    
    

        self.get_logger().info(
            f'L={current_left_ticks} '
            f'R={current_right_ticks} | '
            f'dL={left_tick_diff} '
            f'dR={right_tick_diff} | '
            f'd={distance:.4f} m | '
            f'v={self.linear_velocity:.3f} m/s | '
            f'w={self.angular_velocity:.3f} rad/s | '
            f'pose=({self.x:.2f}, '
            f'{self.y:.2f}, '
            f'{self.theta:.2f})'
        )

    
    
    

    def publish_odometry(self):

        msg = Odometry()

        msg.header.stamp = self.get_clock().now().to_msg()

        msg.header.frame_id = (
            f'{self.robot_name}/odom'
        )

        msg.child_frame_id = (
            f'{self.robot_name}/base_link'
        )


        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0

        q = quaternion_from_euler(
            0.0,
            0.0,
            self.theta
        )

        msg.pose.pose.orientation = Quaternion(
            x=q[0],
            y=q[1],
            z=q[2],
            w=q[3]
        )


        msg.pose.covariance = [
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 999.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.05
        ]

    
    
    

        msg.twist.twist.linear.x = (
            self.linear_velocity
        )

        msg.twist.twist.linear.y = 0.0
        msg.twist.twist.linear.z = 0.0

        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0

        msg.twist.twist.angular.z = (
            self.angular_velocity
        )

        msg.twist.covariance = [
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 999.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.05
        ]

        self.odom_publisher.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    node = WheelOdometryNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        # Prevent the "rcl_shutdown already called" error
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()