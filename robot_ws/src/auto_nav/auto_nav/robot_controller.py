import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import numpy as np


class RobotController(Node):

    def __init__(self):
        super().__init__('robot_controller')
        
        # Parameters
        self.declare_parameter('robot_name', 'robot1')
        self.declare_parameter('linear_speed', 0.6)
        self.declare_parameter('angular_speed', 0.4)  # Boosted slightly for quicker turns
        self.declare_parameter('safe_distance', 0.4)
        self.declare_parameter('goal_tolerance', 0.05)
        
        self.robot_name = self.get_parameter('robot_name').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.safe_distance = self.get_parameter('safe_distance').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, f'/{self.robot_name}/cmd_vel', 10)
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, f'/{self.robot_name}/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, f'/{self.robot_name}/odometry/filtered', self.odom_callback, 10)
        self.goal_sub = self.create_subscription(
            PoseStamped, f'/{self.robot_name}/goal_pose', self.goal_callback, 10)
        
        # State variables
        self.current_scan = None
        self.current_pose = None
        self.current_goal = None
        self.goal_reached = True
        
        # Control loop timer (10Hz)
        self.control_timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info(f'Robot Controller initialized for {self.robot_name}')

    def scan_callback(self, msg):
        self.current_scan = msg

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def goal_callback(self, msg):
        self.current_goal = (msg.pose.position.x, msg.pose.position.y)
        self.goal_reached = False
        self.get_logger().info(f'{self.robot_name} received new target from coordinator: {self.current_goal}')

    def control_loop(self):
        if self.current_scan is None or self.current_pose is None:
            return
        
        cmd = Twist()
        
        if self.current_goal is not None and not self.goal_reached:
            if self.is_goal_reached():
                self.goal_reached = True
                self.get_logger().info(f'{self.robot_name} reached assigned goal!')
                self.publish_cmd_vel(cmd)
                return
            
            # FIX: Explicit tuple index access [1] for Y and [0] for X
            goal_angle = math.atan2(
                self.current_goal[1] - self.current_pose.position.y,
                self.current_goal[0] - self.current_pose.position.x
            )
            
            current_yaw = self.get_yaw_from_pose(self.current_pose)
            angle_diff = self.normalize_angle(goal_angle - current_yaw)
            
            obstacle_detected, obstacle_direction = self.detect_obstacles()
            
            if obstacle_detected:
                cmd = self.obstacle_avoidance_behavior(obstacle_direction)
            else:
                # If facing completely away from target, rotate in place first
                if abs(angle_diff) > 0.5:
                    cmd.angular.z = self.angular_speed if angle_diff > 0 else -self.angular_speed
                    cmd.linear.x = 0.0
                else:
                    cmd.linear.x = self.linear_speed
                    cmd.angular.z = self.angular_speed * 1.5 * angle_diff
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        
        self.publish_cmd_vel(cmd)

    def detect_obstacles(self):
        if self.current_scan is None:
            return False, None
        
        ranges = np.array(self.current_scan.ranges)
        ranges = np.where(np.isfinite(ranges), ranges, self.current_scan.range_max)
        
        if len(ranges) == 0:
            return False, None

        # Divide into 3 simple regional windows for front-centered blocking check
        front_ranges = ranges[len(ranges)//3 : 2*len(ranges)//3]
        min_front_dist = np.min(front_ranges) if len(front_ranges) > 0 else self.current_scan.range_max
        
        if min_front_dist < self.safe_distance:
            left_ranges = ranges[:len(ranges)//4]
            right_ranges = ranges[3*len(ranges)//4:]
            
            avg_left = np.mean(left_ranges) if len(left_ranges) > 0 else 0.0
            avg_right = np.mean(right_ranges) if len(right_ranges) > 0 else 0.0
            
            direction = 'left' if avg_left > avg_right else 'right'
            return True, direction
        
        return False, None

    def obstacle_avoidance_behavior(self, obstacle_direction):
        cmd = Twist()
        # Pivot away from the blocked side cleanly
        if obstacle_direction == 'left':
            cmd.angular.z = -self.angular_speed
        else:
            cmd.angular.z = self.angular_speed
        
        cmd.linear.x = 0.01  # Crawl forward very slowly while turning
        return cmd

    def is_goal_reached(self):
        if self.current_goal is None:
            return True
        
        # FIX: Explicit tuple index access [0] and [1]
        dist = math.sqrt(
            (self.current_goal[0] - self.current_pose.position.x)**2 +
            (self.current_goal[1] - self.current_pose.position.y)**2
        )
        return dist < self.goal_tolerance

    def get_yaw_from_pose(self, pose):
        siny_cosp = 2.0 * (pose.orientation.w * pose.orientation.z + pose.orientation.x * pose.orientation.y)
        cosy_cosp = 1.0 - 2.0 * (pose.orientation.y * pose.orientation.y + pose.orientation.z * pose.orientation.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def publish_cmd_vel(self, cmd):
        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
