import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
import math
import numpy as np
from collections import deque
import time

from ss.frames import to_world_frame, yaw_from_quaternion

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        # Parameters
        self.declare_parameter('robot_name', 'robot1')
        self.declare_parameter('linear_speed', 0.50)
        self.declare_parameter('angular_speed', 0.3)
        self.declare_parameter('safe_distance', 0.4)
        self.declare_parameter('goal_tolerance', 0.3)
        
        # Dynamic obstacle detection parameters
        self.declare_parameter('obstacle_threshold', 0.5)
        self.declare_parameter('safety_distance', 0.3)
        self.declare_parameter('reaction_time', 0.2)
        self.declare_parameter('max_linear_velocity', 0.5)
        self.declare_parameter('max_angular_velocity', 1.0)
        
        # Automatic exploration parameters
        self.declare_parameter('auto_explore', True)
        self.declare_parameter('exploration_goal_timeout', 30.0)
        
        self.robot_name = self.get_parameter('robot_name').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.safe_distance = self.get_parameter('safe_distance').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value
        self.safety_distance = self.get_parameter('safety_distance').value
        self.reaction_time = self.get_parameter('reaction_time').value
        self.max_linear_velocity = self.get_parameter('max_linear_velocity').value
        self.max_angular_velocity = self.get_parameter('max_angular_velocity').value
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, f'/{self.robot_name}/cmd_vel', 10)
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, f'/{self.robot_name}/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, f'/{self.robot_name}/odom', self.odom_callback, 10)
        self.goal_sub = self.create_subscription(
            PoseStamped, f'/{self.robot_name}/goal_pose', self.goal_callback, 10)
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)
        
        # State variables
        self.current_scan = None
        self.current_pose = None
        self.current_goal = None
        self.goal_reached = True
        self.current_map = None
        
        # Dynamic obstacle tracking
        self.previous_scan = None
        self.obstacle_velocities = {}
        self.emergency_stop = False
        self.scan_history = deque(maxlen=5)  # Keep last 5 scans for trend analysis
        
        # Exploration parameters
        self.auto_explore = self.get_parameter('auto_explore').value
        self.exploration_goal_timeout = self.get_parameter('exploration_goal_timeout').value
        self.last_goal_time = time.time()
        self.exploration_active = self.auto_explore
        
        # Control timer
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info(f'Robot Controller initialized for {self.robot_name}')

    def scan_callback(self, msg):
        self.current_scan = msg
        self.scan_history.append((msg, time.time()))
        
        if self.previous_scan is not None:
            self.track_dynamic_obstacles(msg, self.previous_scan)
        self.previous_scan = msg
    
    def map_callback(self, msg):
        self.current_map = msg

    def odom_callback(self, msg):
        pose = msg.pose.pose
        local_yaw = yaw_from_quaternion(pose.orientation)
        world_x, world_y, world_yaw = to_world_frame(self.robot_name, pose.position.x, pose.position.y, local_yaw)
        self.current_pose = {'x': world_x, 'y': world_y, 'yaw': world_yaw}

    def goal_callback(self, msg):
        self.current_goal = (msg.pose.position.x, msg.pose.position.y)
        self.goal_reached = False
        self.last_goal_time = time.time()
        self.get_logger().info(f'{self.robot_name} received new exploration goal: {self.current_goal}')

    def control_loop(self):
        """Enhanced control loop with dynamic obstacle avoidance"""
        if self.current_scan is None or self.current_pose is None:
            self.get_logger().debug(f'{self.robot_name}: Waiting for scan or pose data...')
            return
        
        cmd = Twist()
        
        # Debug current state
        if self.current_goal is not None:
            goal_dist = math.sqrt((self.current_pose['x'] - self.current_goal[0])**2 +
                                (self.current_pose['y'] - self.current_goal[1])**2)
            self.get_logger().debug(f'{self.robot_name}: Goal distance: {goal_dist:.2f}m, Goal: {self.current_goal}')
        else:
            self.get_logger().debug(f'{self.robot_name}: No current goal')
        
        # Check for immediate collision threats from dynamic obstacles
        immediate_threat = self.check_immediate_threats()
        
        if immediate_threat:
            # Emergency stop
            cmd = Twist()
            self.emergency_stop = True
            self.get_logger().warn(f"{self.robot_name}: Emergency stop due to dynamic obstacle!")
        elif self.current_goal is not None and not self.goal_reached:
            # Check for goal timeout
            if time.time() - self.last_goal_time > self.exploration_goal_timeout:
                self.get_logger().warn(f'{self.robot_name}: Goal timeout, marking as reached')
                self.goal_reached = True
                self.emergency_stop = False
                self.publish_cmd_vel(cmd)
                return
                
            if self.is_goal_reached():
                self.goal_reached = True
                self.get_logger().info(f'{self.robot_name} reached exploration goal!')
                self.emergency_stop = False
                self.publish_cmd_vel(cmd)
                return
            
            goal_angle = math.atan2(
                self.current_goal[1] - self.current_pose['y'],
                self.current_goal[0] - self.current_pose['x']
            )

            current_yaw = self.current_pose['yaw']
            angle_diff = self.normalize_angle(goal_angle - current_yaw)
            
            # Enhanced obstacle detection (static + dynamic)
            static_obstacle, obstacle_direction = self.detect_obstacles()
            dynamic_obstacle = self.detect_dynamic_obstacles()
            
            if static_obstacle or dynamic_obstacle:
                cmd = self.enhanced_obstacle_avoidance(obstacle_direction, dynamic_obstacle)
            else:
                # Normal navigation with dynamic awareness
                cmd = self.compute_safe_velocity(angle_diff)
                self.emergency_stop = False
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.emergency_stop = False
        
        self.publish_cmd_vel(cmd)

    def detect_obstacles(self):
        if self.current_scan is None:
            return False, 0
        
        ranges = np.array(self.current_scan.ranges)
        ranges = np.where(np.isfinite(ranges), ranges, self.current_scan.range_max)
        
        front_ranges = ranges[len(ranges)//3:2*len(ranges)//3]
        min_front_dist = np.min(front_ranges)
        
        if min_front_dist < self.safe_distance:
            left_ranges = ranges[:len(ranges)//4]
            right_ranges = ranges[3*len(ranges)//4:]
            
            avg_left = np.mean(left_ranges)
            avg_right = np.mean(right_ranges)
            
            direction = 'left' if avg_left > avg_right else 'right'
            return True, direction
        
        return False, None

    def track_dynamic_obstacles(self, current_scan, previous_scan):
        """Track moving obstacles by comparing consecutive scans"""
        if len(current_scan.ranges) != len(previous_scan.ranges):
            return
        
        current_time = time.time()
        
        for i in range(len(current_scan.ranges)):
            current_range = current_scan.ranges[i]
            previous_range = previous_scan.ranges[i]
            
            if (current_range < self.obstacle_threshold and 
                previous_range < self.obstacle_threshold):
                
                # Calculate apparent velocity
                range_change = current_range - previous_range
                time_diff = 0.1  # Assuming 10Hz scan rate
                
                if abs(range_change) > 0.05:  # Significant change
                    angle = current_scan.angle_min + i * current_scan.angle_increment
                    
                    # Store obstacle info
                    self.obstacle_velocities[i] = {
                        'angle': angle,
                        'velocity': range_change / time_diff,
                        'distance': current_range,
                        'timestamp': current_time
                    }
    
    def predict_obstacle_position(self, obstacle_info, prediction_time):
        """Predict where a dynamic obstacle will be"""
        current_dist = obstacle_info['distance']
        velocity = obstacle_info['velocity']
        angle = obstacle_info['angle']
        
        # Simple linear prediction
        predicted_dist = current_dist + velocity * prediction_time
        
        return {
            'distance': predicted_dist,
            'angle': angle,
            'x': predicted_dist * math.cos(angle),
            'y': predicted_dist * math.sin(angle)
        }
    
    def check_immediate_threats(self):
        """Check for immediate collision threats from dynamic obstacles"""
        if not self.obstacle_velocities:
            return False
        
        current_time = time.time()
        
        for obs_id, obs_info in self.obstacle_velocities.items():
            # Check if obstacle data is recent
            age = current_time - obs_info['timestamp']
            if age > 1.0:  # Data older than 1 second
                continue
            
            # Predict obstacle position
            predicted_pos = self.predict_obstacle_position(
                obs_info, self.reaction_time
            )
            
            # Check if obstacle is approaching and close
            if (predicted_pos['distance'] < self.safety_distance and 
                obs_info['velocity'] < 0):  # Approaching (negative velocity)
                return True
        
        return False
    
    def detect_dynamic_obstacles(self):
        """Enhanced dynamic obstacle detection using scan history"""
        if len(self.scan_history) < 3:
            return False
        
        # Analyze recent scans for movement patterns
        recent_scans = list(self.scan_history)[-3:]
        
        for i in range(len(recent_scans[0][0].ranges)):
            ranges = [scan[0].ranges[i] for scan in recent_scans if 
                     i < len(scan[0].ranges) and scan[0].ranges[i] < self.obstacle_threshold]
            
            if len(ranges) >= 2:
                # Check for significant variance indicating movement
                variance = np.var(ranges)
                if variance > 0.1:  # Threshold for dynamic detection
                    return True
        
        return False
    
    def enhanced_obstacle_avoidance(self, static_direction, dynamic_detected):
        """Enhanced obstacle avoidance considering both static and dynamic obstacles"""
        cmd = Twist()
        
        if dynamic_detected:
            # More aggressive avoidance for dynamic obstacles
            if static_direction == 'left':
                cmd.angular.z = -self.angular_speed * 1.5
            else:
                cmd.angular.z = self.angular_speed * 1.5
            cmd.linear.x = 0.02  # Slower speed for safety
        else:
            # Standard static obstacle avoidance
            if static_direction == 'left':
                cmd.angular.z = -self.angular_speed
            else:
                cmd.angular.z = self.angular_speed
            cmd.linear.x = 0.05
        
        return cmd
    
    def compute_safe_velocity(self, angle_diff):
        """Compute safe velocity considering dynamic obstacles"""
        cmd = Twist()
        
        # Base navigation
        if abs(angle_diff) > 0.2:
            cmd.angular.z = self.angular_speed if angle_diff > 0 else -self.angular_speed
            cmd.linear.x = 0.05
        else:
            cmd.linear.x = self.linear_speed
            cmd.angular.z = 0.3 * angle_diff
        
        # Adjust for dynamic obstacles
        current_time = time.time()
        for obs_info in self.obstacle_velocities.values():
            age = current_time - obs_info['timestamp']
            if age > 0.5:  # Skip old data
                continue
                
            if obs_info['distance'] < 1.0:  # Within 1 meter
                # Reduce speed based on obstacle proximity and velocity
                speed_factor = max(0.1, obs_info['distance'] / 1.0)
                cmd.linear.x *= speed_factor
                
                # Add avoidance turning
                if abs(obs_info['angle']) < math.pi / 4:  # Front sector
                    cmd.angular.z += 0.5 * math.copysign(1, obs_info['angle'])
        
        # Clamp velocities to safe limits
        cmd.linear.x = max(-self.max_linear_velocity, min(self.max_linear_velocity, cmd.linear.x))
        cmd.angular.z = max(-self.max_angular_velocity, min(self.max_angular_velocity, cmd.angular.z))
        
        return cmd

    def is_goal_reached(self):
        if self.current_goal is None:
            return True
        
        dist = math.sqrt(
            (self.current_goal[0] - self.current_pose['x'])**2 +
            (self.current_goal[1] - self.current_pose['y'])**2
        )

        return dist < self.goal_tolerance

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def publish_cmd_vel(self, cmd):
        # Clean up old obstacle data
        current_time = time.time()
        self.obstacle_velocities = {k: v for k, v in self.obstacle_velocities.items() 
                                   if current_time - v['timestamp'] < 2.0}
        
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()