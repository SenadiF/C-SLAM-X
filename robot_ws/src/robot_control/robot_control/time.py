import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu

class Restamper(Node):
    def __init__(self):
        super().__init__('restamper')

        self.scan_sub = self.create_subscription(
            LaserScan, '/robot1/scan_raw', self.scan_callback, 10)
        self.scan_pub = self.create_publisher(LaserScan, '/robot1/scan', 10)

        self.imu_sub = self.create_subscription(
            Imu, '/robot1/imu_raw', self.imu_callback, 10)
        self.imu_pub = self.create_publisher(Imu, '/robot1/imu', 10)

    def scan_callback(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()
        self.scan_pub.publish(msg)

    def imu_callback(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()
        self.imu_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = Restamper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()