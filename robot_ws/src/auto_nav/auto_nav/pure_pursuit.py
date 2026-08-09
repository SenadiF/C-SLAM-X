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
        self.declare_parameter('base_lookahead', 0.40)  # Safe tracking lookahead
        self.declare_parameter('max_linear_speed', 0.4)  # Balanced exploration speed
        self.declare_parameter('max_angular_speed', 1.5) # Fast steering recovery
        self.declare_parameter('goal_tolerance', 0.20)
        self.declare_parameter('obstacle_stop_distance', 0.30)

        self.robot_name = self.get_parameter('robot_name').value
        self.base_lookahead = self.get_parameter('base_lookahead').value
        self.max_v = self.get_parameter('max_linear_speed').value
        self.max_w = self.get_parameter('max_angular_speed').value
        self.goal_tol = self.get_parameter('goal_tolerance').value
        self.obstacle_stop = self.get_parameter('obstacle_stop_distance').value

        self.path = None
        self.pose = None
        self.yaw = 0.0
        self.min_scan_range = float('inf')
        self.min_slow_range = float('inf')

        self.odom_sub = self.create_subscription(Odometry, f'/{self.robot_name}/odometry/filtered', self.odom_callback, 10)
        self.path_sub = self.create_subscription(Path, f'/{self.robot_name}/planned_path', self.path_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, f'/{self.robot_name}/scan', self.scan_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, f'/{self.robot_name}/cmd_vel', 10)

        self.create_timer(0.05, self.control_loop)  # Fast 20Hz loop for immediate wall avoidance
        self.get_logger().info("Pure Pursuit Tracking Engine Active.")

    def odom_callback(self, msg):
        self.pose = msg.pose.pose.position
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny, cosy)

    def path_callback(self, msg):
        self.path = msg

    def scan_callback(self, msg):
        ranges = np.array(msg.ranges)
        angles = np.arange(len(ranges)) * msg.angle_increment + msg.angle_min

        # Narrow Stop Zone (+/- 30 degrees)
        stop_region = (angles > -math.pi/6) & (angles < math.pi/6)
        valid_stop = ranges[stop_region & (ranges > msg.range_min) & (ranges < msg.range_max) & (np.isfinite(ranges))]
        self.min_scan_range = np.min(valid_stop) if len(valid_stop) > 0 else float('inf')

        # Wide Slow/Proximity Zone (+/- 45 degrees)
        slow_region = (angles > -math.pi/4) & (angles < math.pi/4)
        valid_slow = ranges[slow_region & (ranges > msg.range_min) & (ranges < msg.range_max) & (np.isfinite(ranges))]
        self.min_slow_range = np.min(valid_slow) if len(valid_slow) > 0 else float('inf')

    def find_lookahead_point(self, dynamic_lookahead):
        if not self.path or len(self.path.poses) == 0 or not self.pose:
            return None
        rx, ry = self.pose.x, self.pose.y
        for p in self.path.poses:
            if math.hypot(p.pose.position.x - rx, p.pose.position.y - ry) >= dynamic_lookahead:
                return p.pose.position
        return self.path.poses[-1].pose.position

    def control_loop(self):
        if not self.pose or not self.path or len(self.path.poses) == 0:
            return

        # Destination Goal Check
        goal = self.path.poses[-1].pose.position
        if math.hypot(goal.x - self.pose.x, goal.y - self.pose.y) < self.goal_tol:
            self.cmd_pub.publish(Twist())
            self.path = None  # Clear path to complete action
            return

        # 1. Emergency Stop Check
        if self.min_scan_range < self.obstacle_stop:
            self.cmd_pub.publish(Twist())
            return

        # 2. Compute Adaptive Velocity Scaling
        proximity_factor = max(0.2, self.min_slow_range / 0.6) if self.min_slow_range < 0.6 else 1.0

        # Calculate Lookahead dynamically tuned to velocity factors
        dynamic_lookahead = max(0.25, min(0.55, self.base_lookahead * proximity_factor))
        target = self.find_lookahead_point(dynamic_lookahead)
        if not target:
            return

        # Angular Steering Mathematics
        angle_error = math.atan2(target.y - self.pose.y, target.x - self.pose.x) - self.yaw
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        # 3. Curve Scaling Law: Slow down on sharp angles to turn in place
        curve_factor = max(0.15, 1.0 - (abs(angle_error) / (math.pi / 2)))

        twist = Twist()
        twist.linear.x = self.max_v * proximity_factor * curve_factor
        twist.angular.z = max(-self.max_w, min(self.max_w, 2.5 * angle_error))  # Highly responsive heading correction

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuit()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
