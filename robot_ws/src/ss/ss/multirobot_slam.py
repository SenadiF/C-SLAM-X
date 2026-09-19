import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
import tf2_ros
import numpy as np
import math
from collections import deque
import time

from ss.frames import to_world_frame

class MultiRobotSLAM(Node):
    def __init__(self):
        super().__init__('multi_robot_slam')
        
        # Parameters
        self.declare_parameter('map_resolution', 0.05)
        self.declare_parameter('map_width', 2000)
        self.declare_parameter('map_height', 2000)
        self.declare_parameter('update_rate', 5.0)
        
        # Dynamic environment parameters
        self.declare_parameter('decay_rate', 0.95)
        self.declare_parameter('min_observations', 3)
        self.declare_parameter('temporal_window', 30.0)
        self.declare_parameter('dynamic_threshold', 0.3)
        self.declare_parameter('occupied_threshold', 0.7)
        self.declare_parameter('free_threshold', 0.3)
        self.declare_parameter('prior_probability', 0.5)
        
        self.map_resolution = self.get_parameter('map_resolution').value
        self.map_width = self.get_parameter('map_width').value
        self.map_height = self.get_parameter('map_height').value
        self.update_rate = self.get_parameter('update_rate').value
        self.decay_rate = self.get_parameter('decay_rate').value
        self.min_observations = self.get_parameter('min_observations').value
        self.temporal_window = self.get_parameter('temporal_window').value
        self.dynamic_threshold = self.get_parameter('dynamic_threshold').value
        self.occupied_threshold = self.get_parameter('occupied_threshold').value
        self.free_threshold = self.get_parameter('free_threshold').value
        self.prior_probability = self.get_parameter('prior_probability').value
        
        # Publishers
        # Map topics conventionally use TRANSIENT_LOCAL durability (what
        # nav2_map_server/slam_toolbox both do) so a subscriber that
        # attaches after this node has already been publishing - like
        # `ros2 run nav2_map_server map_saver_cli`, or RViz's map display -
        # still gets the latest map immediately instead of waiting
        # indefinitely for the next publish and timing out.
        map_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', map_qos)
        
        # Subscribers
        self.robot1_scan_sub = self.create_subscription(
            LaserScan, '/robot1/scan', self.robot1_scan_callback, 10)
        self.robot2_scan_sub = self.create_subscription(
            LaserScan, '/robot2/scan', self.robot2_scan_callback, 10)
        
        # Odometry subscribers
        self.robot1_odom_sub = self.create_subscription(
            Odometry, '/robot1/odom', self.robot1_odom_callback, 10)
        self.robot2_odom_sub = self.create_subscription(
            Odometry, '/robot2/odom', self.robot2_odom_callback, 10)
        
        # Robot pose tracking from odometry
        self.robot1_pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'timestamp': 0.0}
        self.robot2_pose = {'x': -2.0, 'y': -1.5, 'yaw': math.pi, 'timestamp': 0.0}
        self.pose_lock = False  # Simple mutex for pose updates
        
        # TF (kept for compatibility but not used for localization)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Map data - probabilistic mapping
        self.occupancy_prob = np.full((self.map_height, self.map_width), self.prior_probability, dtype=np.float32)
        self.observation_count = np.zeros((self.map_height, self.map_width), dtype=np.int32)
        self.last_observation_time = np.zeros((self.map_height, self.map_width), dtype=np.float64)
        self.observation_history = {}  # Store recent observations for dynamic detection
        self.map_origin_x = -self.map_width * self.map_resolution / 2
        self.map_origin_y = -self.map_height * self.map_resolution / 2
        
        # Update timer
        self.update_timer = self.create_timer(1.0/self.update_rate, self.publish_map)
        
        # Status monitoring timer
        self.status_timer = self.create_timer(5.0, self.log_system_status)
        
        # Temporal decay timer for dynamic environments
        self.decay_timer = self.create_timer(1.0, self.apply_temporal_decay)
        
        # Transform broadcaster for publishing odometry-based transforms
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Transform publishing timer
        self.transform_timer = self.create_timer(0.1, self.publish_transforms)
        
        # Initialize robot poses
        self.initialize_robot_poses()
        
        self.get_logger().info('Multi-Robot Dynamic SLAM initialized with odometry-based localization')
        self.get_logger().info('Robot1 initial pose: (0.0, 0.0, 0.0°)')
        self.get_logger().info('Robot2 initial pose: (-2.0, -1.5, 180.0°)')
        self.get_logger().info('SLAM now uses real-time odometry data for accurate robot tracking')

    def robot1_scan_callback(self, msg):
        self.process_scan(msg, 'robot1')

    def robot2_scan_callback(self, msg):
        self.process_scan(msg, 'robot2')
    
    def robot1_odom_callback(self, msg):
        """Update robot1 pose from odometry"""
        if self.pose_lock:
            return
        
        pose = msg.pose.pose
        # Convert quaternion to yaw
        qx, qy, qz, qw = pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        world_x, world_y, world_yaw = to_world_frame('robot1', pose.position.x, pose.position.y, yaw)
        self.robot1_pose = {
            'x': world_x,
            'y': world_y,
            'yaw': world_yaw,
            'timestamp': time.time()
        }
        
    def robot2_odom_callback(self, msg):
        """Update robot2 pose from odometry"""
        if self.pose_lock:
            return
            
        pose = msg.pose.pose
        # Convert quaternion to yaw
        qx, qy, qz, qw = pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        world_x, world_y, world_yaw = to_world_frame('robot2', pose.position.x, pose.position.y, yaw)
        self.robot2_pose = {
            'x': world_x,
            'y': world_y,
            'yaw': world_yaw,
            'timestamp': time.time()
        }

    def process_scan(self, scan_msg, robot_name):
        """Process laser scan using odometry-based pose"""
        try:
            # Get robot pose from odometry data
            if robot_name == 'robot1':
                robot_pose = self.robot1_pose
            elif robot_name == 'robot2':
                robot_pose = self.robot2_pose
            else:
                self.get_logger().warn(f'Unknown robot name: {robot_name}')
                return
                
            # Check if pose data is recent (within last 2 seconds)
            current_time = time.time()
            if current_time - robot_pose['timestamp'] > 2.0:
                self.get_logger().warn(f'Stale odometry data for {robot_name}, age: {current_time - robot_pose["timestamp"]:.1f}s')
                return
                
            robot_x = robot_pose['x']
            robot_y = robot_pose['y']
            robot_yaw = robot_pose['yaw']
            
            self.update_map_with_scan(scan_msg, robot_x, robot_y, robot_yaw)
            
        except Exception as e:
            self.get_logger().warn(f'Failed to process scan from {robot_name}: {e}')

    def update_map_with_scan(self, scan, robot_x, robot_y, robot_yaw):
        current_time = time.time()
        
        for i, range_val in enumerate(scan.ranges):
            if not (scan.range_min <= range_val <= scan.range_max):
                continue
            
            angle = scan.angle_min + i * scan.angle_increment + robot_yaw
            
            end_x = robot_x + range_val * math.cos(angle)
            end_y = robot_y + range_val * math.sin(angle)
            
            # Probabilistic ray tracing
            self.probabilistic_ray_trace(robot_x, robot_y, end_x, end_y, current_time)
            self.update_obstacle_probability(end_x, end_y, current_time, True)

    def probabilistic_ray_trace(self, x0, y0, x1, y1, timestamp):
        """Update free space probabilities along ray"""
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        
        steps = max(int(dx / self.map_resolution), int(dy / self.map_resolution))
        
        if steps == 0:
            return
        
        x_step = (x1 - x0) / steps
        y_step = (y1 - y0) / steps
        
        for i in range(steps - 1):  # Don't mark endpoint as free
            x = x0 + i * x_step
            y = y0 + i * y_step
            
            self.update_obstacle_probability(x, y, timestamp, False)

    def update_obstacle_probability(self, x, y, timestamp, is_obstacle):
        """Update cell probability using Bayesian updates"""
        grid_x = int((x - self.map_origin_x) / self.map_resolution)
        grid_y = int((y - self.map_origin_y) / self.map_resolution)
        
        if not (0 <= grid_x < self.map_width and 0 <= grid_y < self.map_height):
            return
        
        # Store observation in history for dynamic detection
        cell_key = (grid_x, grid_y)
        if cell_key not in self.observation_history:
            self.observation_history[cell_key] = deque()
        
        # Add new observation
        self.observation_history[cell_key].append((timestamp, is_obstacle))
        
        # Remove old observations outside temporal window
        while (self.observation_history[cell_key] and 
               timestamp - self.observation_history[cell_key][0][0] > self.temporal_window):
            self.observation_history[cell_key].popleft()
        
        # Bayesian update
        if is_obstacle:
            # P(occupied | sensor reading) = 0.9
            sensor_prob = 0.9
        else:
            # P(free | sensor reading) = 0.1
            sensor_prob = 0.1
        
        prior = self.occupancy_prob[grid_y, grid_x]
        
        # Bayesian update formula
        posterior = (sensor_prob * prior) / (
            sensor_prob * prior + (1 - sensor_prob) * (1 - prior)
        )
        
        self.occupancy_prob[grid_y, grid_x] = posterior
        self.observation_count[grid_y, grid_x] += 1
        self.last_observation_time[grid_y, grid_x] = timestamp

    def initialize_robot_poses(self):
        """Initialize robot starting poses"""
        from geometry_msgs.msg import PoseWithCovarianceStamped
        import math
        
        # Create pose publishers for initial poses
        self.robot1_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/robot1/initialpose', 10)
        self.robot2_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/robot2/initialpose', 10)
        
        # Timer to publish initial poses
        self.initial_pose_timer = self.create_timer(1.0, self.publish_initial_poses)
        self.pose_published_count = 0
    
    def publish_initial_poses(self):
        """Publish initial robot poses"""
        if self.pose_published_count >= 5:  # Publish 5 times then stop
            self.initial_pose_timer.cancel()
            return
            
        import math
        from geometry_msgs.msg import PoseWithCovarianceStamped
        
        # Robot 1: (0,0,0) facing +X (0 degrees)
        pose1_msg = PoseWithCovarianceStamped()
        pose1_msg.header.stamp = self.get_clock().now().to_msg()
        pose1_msg.header.frame_id = 'map'
        pose1_msg.pose.pose.position.x = 0.0
        pose1_msg.pose.pose.position.y = 0.0
        pose1_msg.pose.pose.position.z = 0.0
        pose1_msg.pose.pose.orientation.x = 0.0
        pose1_msg.pose.pose.orientation.y = 0.0
        pose1_msg.pose.pose.orientation.z = 0.0
        pose1_msg.pose.pose.orientation.w = 1.0
        
        # Robot 2: (-2.0, -1.5) facing -X (180 degrees)
        pose2_msg = PoseWithCovarianceStamped()
        pose2_msg.header.stamp = self.get_clock().now().to_msg()
        pose2_msg.header.frame_id = 'map'
        pose2_msg.pose.pose.position.x = -2.0
        pose2_msg.pose.pose.position.y = -1.5
        pose2_msg.pose.pose.position.z = 0.0
        pose2_msg.pose.pose.orientation.x = 0.0
        pose2_msg.pose.pose.orientation.y = 0.0
        pose2_msg.pose.pose.orientation.z = 1.0  # 180 degrees (sin(π/2) = 1)
        pose2_msg.pose.pose.orientation.w = 0.0  # 180 degrees (cos(π/2) = 0)
        
        self.robot1_pose_pub.publish(pose1_msg)
        self.robot2_pose_pub.publish(pose2_msg)
        
        self.pose_published_count += 1
        self.get_logger().info(f'Published initial poses (attempt {self.pose_published_count})')

    def apply_temporal_decay(self):
        """Apply temporal decay to reduce confidence in old observations"""
        current_time = time.time()
        
        # Apply decay to cells not recently observed (older than 5 seconds)
        mask = (current_time - self.last_observation_time) > 5.0
        
        # Decay towards unknown (prior probability)
        self.occupancy_prob[mask] = (
            self.occupancy_prob[mask] * self.decay_rate + 
            self.prior_probability * (1 - self.decay_rate)
        )

    def detect_dynamic_objects(self):
        """Detect cells that change frequently (dynamic objects)"""
        dynamic_mask = np.zeros((self.map_height, self.map_width), dtype=bool)
        
        for (grid_x, grid_y), observations in self.observation_history.items():
            if len(observations) < self.min_observations:
                continue
            
            # Calculate variance in observations
            obs_values = [obs[1] for obs in observations]
            variance = np.var(obs_values)
            
            if variance > self.dynamic_threshold:
                dynamic_mask[grid_y, grid_x] = True
        
        return dynamic_mask

    def publish_map(self):
        """Convert probabilistic map to occupancy grid"""
        # Convert probabilities to occupancy values
        occupancy_grid = np.full((self.map_height, self.map_width), -1, dtype=np.int8)
        
        # Mark as occupied if probability > occupied_threshold
        occupied_mask = self.occupancy_prob > self.occupied_threshold
        occupancy_grid[occupied_mask] = 100
        
        # Mark as free if probability < free_threshold and observed enough times
        free_mask = (self.occupancy_prob < self.free_threshold) & (self.observation_count > 2)
        occupancy_grid[free_mask] = 0
        
        # Create and publish map message
        map_msg = OccupancyGrid()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = 'map'
        
        map_msg.info.resolution = self.map_resolution
        map_msg.info.width = self.map_width
        map_msg.info.height = self.map_height
        map_msg.info.origin.position.x = self.map_origin_x
        map_msg.info.origin.position.y = self.map_origin_y
        map_msg.info.origin.position.z = 0.0
        map_msg.info.origin.orientation.w = 1.0
        
        map_msg.data = occupancy_grid.flatten().tolist()
        
        self.map_pub.publish(map_msg)
        
        # Log dynamic detection statistics
        dynamic_cells = self.detect_dynamic_objects()
        if np.any(dynamic_cells):
            self.get_logger().info(f'Detected {np.sum(dynamic_cells)} dynamic cells')

    def publish_transforms(self):
        """Publish transforms based on odometry data"""
        from geometry_msgs.msg import TransformStamped
        
        current_time = self.get_clock().now()
        
        # Publish robot1 transform
        t1 = TransformStamped()
        t1.header.stamp = current_time.to_msg()
        t1.header.frame_id = 'map'
        t1.child_frame_id = 'robot1/base_link'
        t1.transform.translation.x = self.robot1_pose['x']
        t1.transform.translation.y = self.robot1_pose['y']
        t1.transform.translation.z = 0.0
        
        # Convert yaw to quaternion
        yaw = self.robot1_pose['yaw']
        t1.transform.rotation.x = 0.0
        t1.transform.rotation.y = 0.0
        t1.transform.rotation.z = math.sin(yaw / 2.0)
        t1.transform.rotation.w = math.cos(yaw / 2.0)
        
        # Publish robot2 transform
        t2 = TransformStamped()
        t2.header.stamp = current_time.to_msg()
        t2.header.frame_id = 'map'
        t2.child_frame_id = 'robot2/base_link'
        t2.transform.translation.x = self.robot2_pose['x']
        t2.transform.translation.y = self.robot2_pose['y']
        t2.transform.translation.z = 0.0
        
        # Convert yaw to quaternion
        yaw = self.robot2_pose['yaw']
        t2.transform.rotation.x = 0.0
        t2.transform.rotation.y = 0.0
        t2.transform.rotation.z = math.sin(yaw / 2.0)
        t2.transform.rotation.w = math.cos(yaw / 2.0)
        
        # Broadcast transforms
        self.tf_broadcaster.sendTransform([t1, t2])

    def log_system_status(self):
        """Log system status for monitoring"""
        current_time = time.time()
        
        # Check odometry data freshness
        r1_age = current_time - self.robot1_pose['timestamp']
        r2_age = current_time - self.robot2_pose['timestamp']
        
        # Count occupied and free cells
        occupied_cells = np.sum(self.occupancy_prob > self.occupied_threshold)
        free_cells = np.sum(self.occupancy_prob < self.free_threshold)
        total_observed = np.sum(self.observation_count > 0)
        
        self.get_logger().info(
            f"SLAM Status - Robot1: ({self.robot1_pose['x']:.2f}, {self.robot1_pose['y']:.2f}, {math.degrees(self.robot1_pose['yaw']):.1f}°) "
            f"age:{r1_age:.1f}s | Robot2: ({self.robot2_pose['x']:.2f}, {self.robot2_pose['y']:.2f}, {math.degrees(self.robot2_pose['yaw']):.1f}°) "
            f"age:{r2_age:.1f}s"
        )
        self.get_logger().info(
            f"Map Stats - Occupied: {occupied_cells}, Free: {free_cells}, Total observed: {total_observed}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = MultiRobotSLAM()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()