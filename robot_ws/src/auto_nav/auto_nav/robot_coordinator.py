import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
import math
import numpy as np


class RobotCoordinator(Node):
    def __init__(self):
        super().__init__('robot_coordinator')
        
        # Parameters
        self.declare_parameter('coordination_strategy', 'frontier_based')
        self.declare_parameter('min_robot_distance', 1.5)
        self.declare_parameter('exploration_complete_threshold', 0.90)
        
        self.coordination_strategy = self.get_parameter('coordination_strategy').value
        self.min_robot_distance = self.get_parameter('min_robot_distance').value
        self.exploration_threshold = self.get_parameter('exploration_complete_threshold').value
        
        # Command publishers
        self.robot1_cmd_pub = self.create_publisher(Twist, '/robot1/cmd_vel', 10)
        self.robot2_cmd_pub = self.create_publisher(Twist, '/robot2/cmd_vel', 10)
        self.robot1_goal_pub = self.create_publisher(PoseStamped, '/robot1/goal_pose', 10)
        self.robot2_goal_pub = self.create_publisher(PoseStamped, '/robot2/goal_pose', 10)
        
        # Subscribers (Remapped to filtered odometry frame)
        self.robot1_odom_sub = self.create_subscription(
            Odometry, '/robot1/odometry/filtered', self.robot1_odom_callback, 10)
        self.robot2_odom_sub = self.create_subscription(
            Odometry, '/robot2/odometry/filtered', self.robot2_odom_callback, 10)
        self.robot1_scan_sub = self.create_subscription(
            LaserScan, '/robot1/scan', self.robot1_scan_callback, 10)
        self.robot2_scan_sub = self.create_subscription(
            LaserScan, '/robot2/scan', self.robot2_scan_callback, 10)
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)
        
        # Robot states
        self.robot1_pose = None
        self.robot2_pose = None
        self.robot1_scan = None
        self.robot2_scan = None
        self.current_map = None
        self.exploration_frontiers = []
        self.robot1_goal = None
        self.robot2_goal = None
        
        # Coordination loops
        self.coordination_timer = self.create_timer(2.0, self.coordinate_robots)
        self.safety_timer = self.create_timer(0.2, self.safety_check)
        self.get_logger().info('Robot Coordinator operational.')

    def robot1_odom_callback(self, msg):
        self.robot1_pose = msg.pose.pose

    def robot2_odom_callback(self, msg):
        self.robot2_pose = msg.pose.pose

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
        
        for y in range(1, height-1):
            for x in range(1, width-1):
                index = y * width + x
                if self.current_map.data[index] == -1:  
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

    def cluster_frontiers(self, frontiers, cluster_distance=1.0):
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
                
                # FIX: Added explicit index mapping to resolve tuple subtraction errors
                dist = math.sqrt((frontier[0] - other_frontier[0])**2 + 
                                 (frontier[1] - other_frontier[1])**2)
                if dist < cluster_distance:
                    cluster.append(other_frontier)
                    used.add(j)
            
            if len(cluster) >= 3:
                center_x = sum(f[0] for f in cluster) / len(cluster)
                center_y = sum(f[1] for f in cluster) / len(cluster)
                clusters.append((center_x, center_y))
        
        return clusters

    def coordinate_robots(self):
        # FIX: Removed the restrictive check to allow robot1 to run independently
        if self.robot1_pose is None and self.robot2_pose is None:
            return
        
        if self.is_exploration_complete():
            self.stop_robots()
            self.get_logger().info('Exploration complete!')
            return
        
        if self.coordination_strategy == 'frontier_based':
            self.assign_frontier_goals()

    def assign_frontier_goals(self):
        if not self.exploration_frontiers:
            return
        
        # Single-Robot fallback optimization
        robot1_pos = (self.robot1_pose.position.x, self.robot1_pose.position.y) if self.robot1_pose else None
        robot2_pos = (self.robot2_pose.position.x, self.robot2_pose.position.y) if self.robot2_pose else None
        
        if robot1_pos and len(self.exploration_frontiers) > 0:
            # Sort frontiers by distance to robot1 and assign the closest one
            self.exploration_frontiers.sort(key=lambda f: math.sqrt((robot1_pos[0]-f[0])**2 + (robot1_pos[1]-f[1])**2))
            self.send_goal_to_robot('robot1', self.exploration_frontiers[0])
            
        if robot2_pos and len(self.exploration_frontiers) > 1:
            # Sort frontiers by distance to robot2 and assign its closest target
            self.exploration_frontiers.sort(key=lambda f: math.sqrt((robot2_pos[0]-f[0])**2 + (robot2_pos[1]-f[1])**2))
            self.send_goal_to_robot('robot2', self.exploration_frontiers[1])

    def send_goal_to_robot(self, robot_name, goal_pos):
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'map'
        goal_msg.pose.position.x = goal_pos[0]
        goal_msg.pose.position.y = goal_pos[1]
        goal_msg.pose.orientation.w = 1.0
        
        if robot_name == 'robot1':
            self.robot1_goal_pub.publish(goal_msg)
            self.robot1_goal = goal_pos
        elif robot_name == 'robot2':
            self.robot2_goal_pub.publish(goal_msg)
            self.robot2_goal = goal_pos
        
        self.get_logger().info(f'Sent goal to {robot_name}: ({goal_pos[0]:.2f}, {goal_pos[1]:.2f})')

    def is_exploration_complete(self):
        if self.current_map is None:
            return False
        data = np.array(self.current_map.data)
        known_cells = np.count_nonzero(data != -1)
        return (known_cells / len(data)) >= self.exploration_threshold

    def safety_check(self):
        if not self.robot1_pose or not self.robot2_pose:
            return
        dist = math.sqrt(
            (self.robot1_pose.position.x - self.robot2_pose.position.x)**2 +
            (self.robot1_pose.position.y - self.robot2_pose.position.y)**2
        )
        if dist < 0.6:
            self.get_logger().warn("Proximity limit breached! Braking wheels.")
            self.stop_robots()

    def stop_robots(self):
        stop_cmd = Twist()
        self.robot1_cmd_pub.publish(stop_cmd)
        self.robot2_cmd_pub.publish(stop_cmd)


def main(args=None):
    rclpy.init(args=args)
    node = RobotCoordinator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
