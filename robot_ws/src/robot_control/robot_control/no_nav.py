import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist
import tf2_ros
import math
import numpy as np

class AutonomousExplorer(Node):
    def __init__(self):
        super().__init__('autonomous_explorer')
        
        # Subscribers and Publishers
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # TF Buffer to get robot's current position
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Timer for control loop (Run at 5Hz)
        self.timer = self.create_timer(0.2, self.control_loop)
        
        self.latest_map = None

    def map_callback(self, msg):
        self.latest_map = msg

    def get_robot_pose(self):
        try:
            # Look up transform from map to base_link
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            x = t.transform.translation.x
            y = t.transform.translation.y
            
            # Convert quaternion to yaw angle
            q = t.transform.rotation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            
            return x, y, yaw
        except Exception:
            return None

    def control_loop(self):
        if self.latest_map is None:
            return
            
        pose = self.get_robot_pose()
        if pose is None:
            self.get_logger().info('Waiting for map->base_link transform...')
            return
            
        rx, ry, ryaw = pose
        
        # --- 1. FIND FRONTIERS ---
        data = np.array(self.latest_map.data).reshape((self.latest_map.info.height, self.latest_map.info.width))
        res = self.latest_map.info.resolution
        origin_x = self.latest_map.info.origin.position.x
        origin_y = self.latest_map.info.origin.position.y
        
        frontiers = []
        # Basic scanning for cells matching frontier criteria
        for y in range(1, data.shape[0] - 1, 2):  # Step by 2 to save CPU cycles
            for x in range(1, data.shape[1] - 1, 2):
                if data[y, x] == 0:  # Free Space
                    # Check neighbors for unknown space (-1)
                    neighbors = data[y-1:y+2, x-1:x+2]
                    if -1 in neighbors:
                        # Convert grid cell to world coordinates
                        wx = x * res + origin_x
                        wy = y * res + origin_y
                        frontiers.append((wx, wy))
                        
        if not frontiers:
            self.get_logger().info('Mapping Complete! No frontiers remaining.')
            self.stop_robot()
            return

        # --- 2. SELECT THE CLOSEST FRONTIER ---
        distances = [math.hypot(fx - rx, fy - ry) for fx, fy in frontiers]
        closest_idx = np.argmin(distances)
        goal_x, goal_y = frontiers[closest_idx]

        # --- 3. DRIVE TO GOAL (PURE PURSUIT CONTROLLER) ---
        angle_to_goal = math.atan2(goal_y - ry, goal_x - rx)
        heading_error = angle_to_goal - ryaw
        
        # Normalize heading error between -pi and +pi
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))
        
        twist = Twist()
        # If orientation is way off, spin in place first
        if abs(heading_error) > 0.4:
            twist.linear.x = 0.0
            twist.angular.z = 0.5 if heading_error > 0 else -0.5
        else:
            # Move forward and scale angular velocity proportionally
            twist.linear.x = 0.15  # Slow and safe speed for mapping
            twist.angular.z = 1.2 * heading_error
            
        self.cmd_pub.publish(twist)

    def stop_robot(self):
        twist = Twist()
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
