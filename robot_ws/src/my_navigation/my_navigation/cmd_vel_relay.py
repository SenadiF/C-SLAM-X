import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped


class CmdVelRelay(Node):

    def __init__(self):

        super().__init__('cmd_vel_relay')

        self.subscriber = self.create_subscription(
            Twist,
            '/robot1/cmd_vel',
            self.cmd_callback,
            10
        )

        self.publisher = self.create_publisher(
            TwistStamped,
            '/cmd_vel',
            10
        )

        self.get_logger().info(
            'Command velocity relay active.'
        )

    def cmd_callback(self, msg):

        cmd = TwistStamped()

        cmd.header.stamp = self.get_clock().now().to_msg()

        cmd.header.frame_id = 'base_link'

        cmd.twist.linear.x = msg.linear.x
        cmd.twist.linear.y = msg.linear.y
        cmd.twist.linear.z = msg.linear.z

        cmd.twist.angular.x = msg.angular.x
        cmd.twist.angular.y = msg.angular.y
        cmd.twist.angular.z = msg.angular.z

        self.publisher.publish(cmd)


def main(args=None):

    rclpy.init(args=args)

    node = CmdVelRelay()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()