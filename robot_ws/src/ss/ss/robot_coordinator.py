import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
import math
import numpy as np

from ss.frames import to_world_frame, yaw_from_quaternion

class RobotCoordinator(Node):
    def __init__(self):
        super().__init__('robot_coordinator')
        
        # Parameters
        self.declare_parameter('coordination_strategy', 'frontier_based')
        self.declare_parameter('min_robot_distance', 1.5)
        self.declare_parameter('exploration_complete_threshold', 0.90)
        self.declare_parameter('auto_start_exploration', True)
        self.declare_parameter('frontier_min_size', 5)
        self.declare_parameter('frontier_cluster_distance', 1.0)
        self.declare_parameter('goal_assignment_interval', 3.0)
        self.declare_parameter('max_exploration_range', 10.0)
        
        self.coordination_strategy = self.get_parameter('coordination_strategy').value
        self.min_robot_distance = self.get_parameter('min_robot_distance').value
        self.exploration_threshold = self.get_parameter('exploration_complete_threshold').value
        self.auto_start_exploration = self.get_parameter('auto_start_exploration').value
        self.frontier_min_size = self.get_parameter('frontier_min_size').value
        self.frontier_cluster_distance = self.get_parameter('frontier_cluster_distance').value
        self.goal_assignment_interval = self.get_parameter('goal_assignment_interval').value
        self.max_exploration_range = self.get_parameter('max_exploration_range').value
        
        # Publishers for robot commands
        self.robot1_cmd_pub = self.create_publisher(Twist, '/robot1/cmd_vel', 10)
        self.robot2_cmd_pub = self.create_publisher(Twist, '/robot2/cmd_vel', 10)
        self.robot1_goal_pub = self.create_publisher(PoseStamped, '/robot1/goal_pose', 10)
        self.robot2_goal_pub = self.create_publisher(PoseStamped, '/robot2/goal_pose', 10)
        
        # Subscribers for robot states
        self.robot1_odom_sub = self.create_subscription(
            Odometry, '/robot1/odom', self.robot1_odom_callback, 10)
        self.robot2_odom_sub = self.create_subscription(
            Odometry, '/robot2/odom', self.robot2_odom_callback, 10)
        self.robot1_scan_sub = self.create_subscription(
            LaserScan, '/robot1/scan', self.robot1_scan_callback, 10)
        self.robot2_scan_sub = self.create_subscription(
            LaserScan, '/robot2/scan', self.robot2_scan_callback, 10)
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)
        
        # Robot states with initial poses
        self.robot1_pose = None
        self.robot2_pose = None
        self.robot1_scan = None
        self.robot2_scan = None
        self.current_map = None
        self.exploration_frontiers = []
        self.robot1_goal = None
        self.robot2_goal = None
        
        # Exploration state
        self.exploration_active = False
        self.last_goal_assignment = 0
        self.robot1_goal_reached = True
        self.robot2_goal_reached = True
        self.exploration_started = False
        
        # Initial poses
        self.robot1_initial = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        self.robot2_initial = {'x': -2.0, 'y': -1.5, 'yaw': math.pi}
        
        # Coordination timer - use goal_assignment_interval
        self.coordination_timer = self.create_timer(
            self.goal_assignment_interval, self.coordinate_robots)
        
        # Safety timer for collision avoidance
        self.safety_timer = self.create_timer(0.2, self.safety_check)
        
        # Startup timer to begin exploration
        self.startup_timer = self.create_timer(5.0, self.start_exploration)
        
        self.get_logger().info('Robot Coordinator initialized - Auto exploration enabled')

    def robot1_odom_callback(self, msg):
        pose = msg.pose.pose
        yaw = yaw_from_quaternion(pose.orientation)
        world_x, world_y, world_yaw = to_world_frame('robot1', pose.position.x, pose.position.y, yaw)
        self.robot1_pose = {'x': world_x, 'y': world_y, 'yaw': world_yaw}

    def robot2_odom_callback(self, msg):
        pose = msg.pose.pose
        yaw = yaw_from_quaternion(pose.orientation)
        world_x, world_y, world_yaw = to_world_frame('robot2', pose.position.x, pose.position.y, yaw)
        self.robot2_pose = {'x': world_x, 'y': world_y, 'yaw': world_yaw}

    def robot1_scan_callback(self, msg):
        self.robot1_scan = msg

    def robot2_scan_callback(self, msg):
        self.robot2_scan = msg

    def map_callback(self, msg):
        self.current_map = msg
        self.find_exploration_frontiers()

    def find_exploration_frontiers(self):
        if self.current_map is None:
            return
        
        frontiers = []
        width = self.current_map.info.width
        height = self.current_map.info.height
        resolution = self.current_map.info.resolution
        origin_x = self.current_map.info.origin.position.x
        origin_y = self.current_map.info.origin.position.y
        
        # Find frontier cells
        for y in range(1, height-1):
            for x in range(1, width-1):
                index = y * width + x
                if self.current_map.data[index] == -1:  # Unknown cell
                    # Check if adjacent to free space
                    adjacent_free = False
                    for dx in [-1, 0, 1]:
                        for dy in [-1, 0, 1]:
                            if dx == 0 and dy == 0:
                                continue
                            adj_index = (y+dy) * width + (x+dx)
                            if 0 <= adj_index < len(self.current_map.data):
                                if self.current_map.data[adj_index] == 0:
                                    adjacent_free = True
                                    break
                        if adjacent_free:
                            break
                    
                    if adjacent_free:
                        world_x = x * resolution + origin_x
                        world_y = y * resolution + origin_y
                        frontiers.append((world_x, world_y))
        
        self.exploration_frontiers = self.cluster_frontiers(frontiers)

    def cluster_frontiers(self, frontiers):
        if not frontiers:
            return []
        
        clusters = []
        used = set()
        
        for i, frontier in enumerate(frontiers):
            if i in used:
                continue
            
            cluster = [frontier]
            used.add(i)
            
            for j, other_frontier in enumerate(frontiers):
                if j in used:
                    continue
                
                dist = math.sqrt((frontier[0] - other_frontier[0])**2 + 
                               (frontier[1] - other_frontier[1])**2)
                if dist < self.frontier_cluster_distance:
                    cluster.append(other_frontier)
                    used.add(j)
            
            if len(cluster) >= self.frontier_min_size:
                center_x = sum(f[0] for f in cluster) / len(cluster)
                center_y = sum(f[1] for f in cluster) / len(cluster)
                
                # Filter by exploration range from origin
                dist_from_origin = math.sqrt(center_x**2 + center_y**2)
                if dist_from_origin <= self.max_exploration_range:
                    clusters.append((center_x, center_y))
        
        return clusters

    def start_exploration(self):
        """Start automatic exploration after initial setup"""
        if self.auto_start_exploration and not self.exploration_started:
            self.exploration_active = True
            self.exploration_started = True
            self.startup_timer.cancel()
            self.get_logger().info('Starting automatic frontier exploration!')
            
            # Give initial exploration goals immediately
            self.assign_initial_goals()
    
    def coordinate_robots(self):
        if not all([self.robot1_pose, self.robot2_pose]):
            self.get_logger().debug('Waiting for robot poses...')
            return
        
        if not self.exploration_active:
            self.get_logger().debug('Exploration not active yet...')
            return
        
        # Check if goals are reached
        self.check_goal_completion()
        
        if self.is_exploration_complete():
            self.stop_robots()
            self.exploration_active = False
            self.get_logger().info('Exploration complete!')
            return
        
        if self.coordination_strategy == 'frontier_based':
            self.assign_frontier_goals()
        
        # Debug logging
        self.get_logger().debug(f'Frontiers found: {len(self.exploration_frontiers)}, Robot1 goal reached: {self.robot1_goal_reached}, Robot2 goal reached: {self.robot2_goal_reached}')
    
    def check_goal_completion(self):
        """Check if robots have reached their goals"""
        if self.robot1_goal and self.robot1_pose:
            dist1 = math.sqrt(
                (self.robot1_pose['x'] - self.robot1_goal[0])**2 +
                (self.robot1_pose['y'] - self.robot1_goal[1])**2
            )
            if dist1 < 0.5:  # Goal tolerance
                self.robot1_goal_reached = True
                self.get_logger().info('Robot1 reached goal!')

        if self.robot2_goal and self.robot2_pose:
            dist2 = math.sqrt(
                (self.robot2_pose['x'] - self.robot2_goal[0])**2 +
                (self.robot2_pose['y'] - self.robot2_goal[1])**2
            )
            if dist2 < 0.5:  # Goal tolerance
                self.robot2_goal_reached = True
                self.get_logger().info('Robot2 reached goal!')

    def assign_frontier_goals(self):
        # Only assign new goals if robots have reached their current goals
        if not (self.robot1_goal_reached or self.robot2_goal_reached):
            self.get_logger().debug('Both robots have active goals, waiting...')
            return
        
        if len(self.exploration_frontiers) < 1:
            self.get_logger().info('No frontiers found, sending simple movement goals...')
            self.assign_simple_goals()
            return
        
        robot1_pos = (self.robot1_pose['x'], self.robot1_pose['y'])
        robot2_pos = (self.robot2_pose['x'], self.robot2_pose['y'])

        # Assign goals to robots that need them
        available_frontiers = list(self.exploration_frontiers)
        
        if self.robot1_goal_reached and available_frontiers:
            # Find closest frontier for robot1
            best_frontier1 = min(available_frontiers, 
                               key=lambda f: math.sqrt((robot1_pos[0] - f[0])**2 + (robot1_pos[1] - f[1])**2))
            self.send_goal_to_robot('robot1', best_frontier1)
            self.robot1_goal_reached = False
            available_frontiers.remove(best_frontier1)
        
        if self.robot2_goal_reached and available_frontiers:
            # Find closest frontier for robot2 that maintains minimum distance
            valid_frontiers = []
            for frontier in available_frontiers:
                if self.robot1_goal:
                    dist_to_robot1_goal = math.sqrt(
                        (frontier[0] - self.robot1_goal[0])**2 + 
                        (frontier[1] - self.robot1_goal[1])**2
                    )
                    if dist_to_robot1_goal >= self.min_robot_distance:
                        valid_frontiers.append(frontier)
                else:
                    valid_frontiers.append(frontier)
            
            if valid_frontiers:
                best_frontier2 = min(valid_frontiers, 
                                   key=lambda f: math.sqrt((robot2_pos[0] - f[0])**2 + (robot2_pos[1] - f[1])**2))
                self.send_goal_to_robot('robot2', best_frontier2)
                self.robot2_goal_reached = False

    def send_goal_to_robot(self, robot_name, goal_pos):
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'map'
        goal_msg.pose.position.x = goal_pos[0]
        goal_msg.pose.position.y = goal_pos[1]
        goal_msg.pose.position.z = 0.0
        goal_msg.pose.orientation.w = 1.0
        
        if robot_name == 'robot1':
            self.robot1_goal_pub.publish(goal_msg)
            self.robot1_goal = goal_pos
        elif robot_name == 'robot2':
            self.robot2_goal_pub.publish(goal_msg)
            self.robot2_goal = goal_pos
        
        self.get_logger().info(f'Sent goal to {robot_name}: ({goal_pos[0]:.2f}, {goal_pos[1]:.2f})')

    def safety_check(self):
        if not all([self.robot1_pose, self.robot2_pose]):
            return
        
        dist = math.sqrt((self.robot1_pose['x'] - self.robot2_pose['x'])**2 +
                        (self.robot1_pose['y'] - self.robot2_pose['y'])**2)
        
        if dist < self.min_robot_distance:
            self.emergency_stop()
            self.get_logger().warn(f'Robots too close ({dist:.2f}m)! Emergency stop.')

    def emergency_stop(self):
        stop_msg = Twist()
        self.robot1_cmd_pub.publish(stop_msg)
        self.robot2_cmd_pub.publish(stop_msg)

    def stop_robots(self):
        stop_msg = Twist()
        self.robot1_cmd_pub.publish(stop_msg)
        self.robot2_cmd_pub.publish(stop_msg)

    def assign_initial_goals(self):
        """Assign initial goals to get robots moving"""
        self.get_logger().info('Assigning initial exploration goals...')
        
        # Robot1 moves forward in +X direction
        initial_goal1 = (2.0, 0.0)
        self.send_goal_to_robot('robot1', initial_goal1)
        self.robot1_goal_reached = False
        
        # Robot2 moves forward in -X direction (it's facing 180 degrees)
        initial_goal2 = (-1.0, 0.0)
        self.send_goal_to_robot('robot2', initial_goal2) 
        self.robot2_goal_reached = False
        
        self.get_logger().info('Initial goals assigned to start exploration!')
    
    def assign_simple_goals(self):
        """Assign simple movement goals when no frontiers are detected"""
        robot1_pos = (self.robot1_pose['x'], self.robot1_pose['y'])
        robot2_pos = (self.robot2_pose['x'], self.robot2_pose['y'])

        if self.robot1_goal_reached:
            # Move robot1 in a different direction to find new areas
            goal_x = robot1_pos[0] + 2.0 * math.cos(self.robot1_pose['yaw'])
            goal_y = robot1_pos[1] + 2.0 * math.sin(self.robot1_pose['yaw'])
            self.send_goal_to_robot('robot1', (goal_x, goal_y))
            self.robot1_goal_reached = False

        if self.robot2_goal_reached:
            # Move robot2 in a different direction
            goal_x = robot2_pos[0] + 2.0 * math.cos(self.robot2_pose['yaw'] + math.pi)
            goal_y = robot2_pos[1] + 2.0 * math.sin(self.robot2_pose['yaw'] + math.pi)
            self.send_goal_to_robot('robot2', (goal_x, goal_y))
            self.robot2_goal_reached = False

    def is_exploration_complete(self):
        if self.current_map is None:
            return False
        
        total_cells = len(self.current_map.data)
        known_cells = sum(1 for cell in self.current_map.data if cell != -1)
        exploration_ratio = known_cells / total_cells if total_cells > 0 else 0
        
        return exploration_ratio >= self.exploration_threshold

def main(args=None):
    rclpy.init(args=args)
    node = RobotCoordinator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()