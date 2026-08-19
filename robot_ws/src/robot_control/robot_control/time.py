import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu


class Restamper(Node):

    def __init__(self):
        super().__init__('restamper')


        self.robot1_scan_sub = self.create_subscription(
            LaserScan,
            '/robot1/scan_raw',
            self.robot1_scan_callback,
            10
        )

        self.robot1_scan_pub = self.create_publisher(
            LaserScan,
            '/robot1/scan',
            10
        )

        self.robot1_imu_sub = self.create_subscription(
            Imu,
            '/robot1/imu_raw',
            self.robot1_imu_callback,
            10
        )

        self.robot1_imu_pub = self.create_publisher(
            Imu,
            '/robot1/imu',
            10
        )



        self.robot2_scan_sub = self.create_subscription(
            LaserScan,
            '/robot2/scan_raw',
            self.robot2_scan_callback,
            10
        )

        self.robot2_scan_pub = self.create_publisher(
            LaserScan,
            '/robot2/scan',
            10
        )

        self.robot2_imu_sub = self.create_subscription(
            Imu,
            '/robot2/imu_raw',
            self.robot2_imu_callback,
            10
        )

        self.robot2_imu_pub = self.create_publisher(
            Imu,
            '/robot2/imu',
            10
        )

        self.get_logger().info(
            'Restamper started for robot1 and robot2'
        )






    def robot1_scan_callback(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()
        self.robot1_scan_pub.publish(msg)

    def robot1_imu_callback(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()
        self.robot1_imu_pub.publish(msg)

    def robot2_scan_callback(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()
        self.robot2_scan_pub.publish(msg)

    def robot2_imu_callback(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()
        self.robot2_imu_pub.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    node = Restamper()

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