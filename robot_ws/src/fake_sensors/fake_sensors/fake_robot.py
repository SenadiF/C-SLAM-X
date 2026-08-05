import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math


class FakeRobot(Node):

    def __init__(self):
        super().__init__('fake_robot')

        self.declare_parameter("robot_name", "robot1")
        self.robot = self.get_parameter("robot_name").value

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.v = 0.0
        self.w = 0.0

        self.last_time = self.get_clock().now()

        self.cmd_sub = self.create_subscription(
            Twist,
            f'/{self.robot}/cmd_vel',
            self.cmd_callback,
            10)

        self.odom_pub = self.create_publisher(
            Odometry,
            f'/{self.robot}/odometry/filtered',
            10)

        self.scan_pub = self.create_publisher(
            LaserScan,
            f'/{self.robot}/scan',
            10)

        self.tf = TransformBroadcaster(self)

        self.timer = self.create_timer(0.05, self.update)

        self.get_logger().info("Fake robot started")

    def cmd_callback(self, msg):
        self.v = msg.linear.x
        self.w = msg.angular.z

    def update(self):

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.w * dt

        qz = math.sin(self.yaw / 2)
        qw = math.cos(self.yaw / 2)

        tf = TransformStamped()

        tf.header.stamp = now.to_msg()
        tf.header.frame_id = "map"
        tf.child_frame_id = f"{self.robot}/base_link"

        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw

        self.tf.sendTransform(tf)

        odom = Odometry()

        odom.header = tf.header
        odom.child_frame_id = tf.child_frame_id

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        self.odom_pub.publish(odom)

        scan = LaserScan()

        scan.header.stamp = now.to_msg()
        scan.header.frame_id = f"{self.robot}/base_link"

        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = math.pi / 180

        scan.range_min = 0.12
        scan.range_max = 10.0

        scan.ranges = [10.0] * 360

        self.scan_pub.publish(scan)


def main(args=None):

    rclpy.init(args=args)

    node = FakeRobot()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()