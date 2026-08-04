import rclpy
from rclpy.node import Node
import numpy as np
import math

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit')

        self.declare_parameter('robot_name', 'robot1')
        self.declare_parameter('lookahead_distance', 0.3)
        self.declare_parameter('max_linear_speed', 0.2)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('goal_tolerance', 0.15)
        self.declare_parameter('obstacle_stop_distance', 0.25)

        self.robot_name = self.get_parameter('robot_name').value
        self.lookahead = self.get_parameter('lookahead_distance').value
        self.max_v = self.get_parameter('max_linear_speed').value
        self.max_w = self.get_parameter('max_angular_speed').value
        self.goal_tol = self.get_parameter('goal_tolerance').value
        self.obstacle_stop = self.get_parameter('obstacle_stop_distance').value

        self.path = None
        self.pose = None
        self.yaw = 0.0
        self.min_scan_range = float('inf')

        self.odom_sub = self.create_subscription(
            Odometry, f'/{self.robot_name}/odometry/filtered', self.odom_callback, 10)
        self.path_sub = self.create_subscription(
            Path, f'/{self.robot_name}/planned_path', self.path_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, f'/{self.robot_name}/scan', self.scan_callback, 10)
        self.cmd_pub = self.create_publisher(
            Twist, f'/{self.robot_name}/cmd_vel', 10)

        self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Pure pursuit ready.')

    def odom_callback(self, msg):
        self.pose = msg.pose.pose.position
        q = msg.pose.pose.orientation
        siny = 2 * (q.w * q.z + q.x * q.y)
        cosy = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny, cosy)

    def path_callback(self, msg):
        self.path = msg

    def scan_callback(self, msg):
        # Only check a forward-facing cone, ignore inf/out-of-range readings
        valid = [r for r in msg.ranges if r > msg.range_min and r < msg.range_max]
        self.min_scan_range = min(valid) if valid else float('inf')

    def find_lookahead_point(self):
        if not self.path or not self.path.poses or self.pose is None:
            return None
        rx, ry = self.pose.x, self.pose.y
        for p in self.path.poses:
            d = math.hypot(p.pose.position.x - rx, p.pose.position.y - ry)
            if d >= self.lookahead:
                return p.pose.position
        return self.path.poses[-1].pose.position

    def control_loop(self):
        if self.pose is None or self.path is None or not self.path.poses:
            return

        goal = self.path.poses[-1].pose.position
        dist_to_goal = math.hypot(goal.x - self.pose.x, goal.y - self.pose.y)
        if dist_to_goal < self.goal_tol:
            self.cmd_pub.publish(Twist())  # stop, goal reached
            return

        # Obstacle avoidance override — safety first
        if self.min_scan_range < self.obstacle_stop:
            self.get_logger().warn('Obstacle too close — stopping.')
            self.cmd_pub.publish(Twist())
            return

        target = self.find_lookahead_point()
        if target is None:
            return

        dx = target.x - self.pose.x
        dy = target.y - self.pose.y
        target_angle = math.atan2(dy, dx)
        angle_error = target_angle - self.yaw
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        twist = Twist()
        twist.linear.x = self.max_v * max(0.3, 1.0 - abs(angle_error))
        twist.angular.z = max(-self.max_w, min(self.max_w, 2.0 * angle_error))
        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuit()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()