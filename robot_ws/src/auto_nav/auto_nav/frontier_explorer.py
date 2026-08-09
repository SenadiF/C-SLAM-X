import rclpy
from rclpy.node import Node

import numpy as np
import time
import math
from collections import deque

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
from std_msgs.msg import Bool


class FrontierExplorer(Node):

    def __init__(self):
        super().__init__('frontier_explorer')

        # Parameters
        self.declare_parameter('robot_name', 'robot1')
        self.declare_parameter('min_frontier_size', 8)
        self.declare_parameter('revisit_radius', 0.5)
        self.declare_parameter('poll_period', 2.0)
        self.declare_parameter('goal_timeout', 15.0)  
        self.declare_parameter('stuck_distance', 0.10)

        self.robot_name = self.get_parameter('robot_name').value
        self.min_frontier_size = self.get_parameter('min_frontier_size').value
        self.revisit_radius = self.get_parameter('revisit_radius').value
        poll_period = self.get_parameter('poll_period').value
        self.goal_timeout = self.get_parameter('goal_timeout').value
        self.stuck_distance = self.get_parameter('stuck_distance').value

        self.map_frame = "map"
        self.base_frame = f"{self.robot_name}/base_link"

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.map = None
        self.visited = []
        self.current_goal = None

        # Goal tracking states
        self.goal_start_time = None
        self.last_robot_pose = None
        self.stuck_counter = 0

        # Subscribers
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 1)

        # Publishers
        self.goal_pub = self.create_publisher(PoseStamped, f'/{self.robot_name}/frontier_goal', 10)
        self.done_pub = self.create_publisher(Bool, f'/{self.robot_name}/exploration_done', 10)

        self.create_timer(poll_period, self.explore)
        self.get_logger().info("Frontier explorer ready.")

    def map_callback(self, msg):
        self.map = msg

    def get_robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
            return (tf.transform.translation.x, tf.transform.translation.y)
        except Exception:
            return None

    def world_to_grid(self, x, y):
        if self.map is None:
            return 0, 0
        info = self.map.info
        gx = int(math.floor((x - info.origin.position.x) / info.resolution))
        gy = int(math.floor((y - info.origin.position.y) / info.resolution))
        return gx, gy

    def grid_to_world(self, gx, gy):
        info = self.map.info
        x = (gx + 0.5) * info.resolution + info.origin.position.x
        y = (gy + 0.5) * info.resolution + info.origin.position.y
        return x, y

    def find_frontiers(self):
        if self.map is None:
            return []

        info = self.map.info
        width, height = info.width, info.height
        grid = np.array(self.map.data, dtype=np.int16).reshape(height, width)

        free = grid == 0
        unknown = grid == -1

        unknown_adjacent = np.zeros_like(unknown)
        unknown_adjacent[:-1, :] |= unknown[1:, :]
        unknown_adjacent[1:, :] |= unknown[:-1, :]
        unknown_adjacent[:, :-1] |= unknown[:, 1:]
        unknown_adjacent[:, 1:] |= unknown[:, :-1]

        frontier_mask = free & unknown_adjacent

        if not frontier_mask.any():
            return []

        visited_cells = np.zeros_like(frontier_mask)
        frontiers = []
        cells = np.argwhere(frontier_mask)

        for sy, sx in cells:
            if visited_cells[sy, sx]:
                continue

            queue = deque([(int(sy), int(sx))])
            visited_cells[sy, sx] = True
            cluster = []

            while queue:
                y, x = queue.popleft()
                cluster.append((y, x))
                neighbours = [(y-1, x), (y+1, x), (y, x-1), (y, x+1)]

                for ny, nx in neighbours:
                    if (0 <= ny < height and 0 <= nx < width and 
                            frontier_mask[ny, nx] and not visited_cells[ny, nx]):
                        visited_cells[ny, nx] = True
                        queue.append((ny, nx))

            if len(cluster) < self.min_frontier_size:
                continue

            arr = np.array(cluster)
            cy, cx = arr.mean(axis=0)
            frontiers.append(self.grid_to_world(int(cx), int(cy)))

        return frontiers

    def already_visited(self, x, y):
        for vx, vy in self.visited:
            if math.hypot(x - vx, y - vy) < self.revisit_radius:
                return True
        return False

    def goal_reached(self, x, y):
        if self.current_goal is None:
            return False
        gx, gy = self.current_goal
        return math.hypot(gx - x, gy - y) < 0.25

    def goal_timeout_check(self, rx, ry):
        if self.current_goal is None:
            return False

        if self.goal_start_time is None:
            self.goal_start_time = time.time()
            self.last_robot_pose = (rx, ry)
            return False

        moved = math.hypot(rx - self.last_robot_pose[0], ry - self.last_robot_pose[1])
        self.last_robot_pose = (rx, ry)

        if moved < self.stuck_distance:
            self.stuck_counter += 1
        else:
            self.stuck_counter = 0

        elapsed = time.time() - self.goal_start_time

        if elapsed > self.goal_timeout:
            self.get_logger().warn("Goal timeout reached. Forfeiting target.")
            return True

        if self.stuck_counter > 4:
            self.get_logger().warn("Robot physically stuck. Forfeiting target.")
            return True

        return False

    def explore(self):
        if self.map is None:
            return

        pose = self.get_robot_pose()
        if pose is None:
            return

        rx, ry = pose

        if self.current_goal is not None:
            if self.goal_reached(rx, ry):
                self.get_logger().info("Goal reached successfully.")
                self.visited.append(self.current_goal)
                self.reset_goal_states()
            elif self.goal_timeout_check(rx, ry):
                self.get_logger().warn("Goal failed or timed out. Blacklisting.")
                self.visited.append(self.current_goal)
                self.reset_goal_states()
            else:
                return

        frontiers = self.find_frontiers()

        # FIX: Evaluates complete coordinate sets properly inside sequential array passes
        valid_frontiers = [f for f in frontiers if not self.already_visited(f[0], f[1])]

        if not valid_frontiers:
            self.get_logger().info("Exploration complete! No more valid frontiers.")
            done_msg = Bool()
            done_msg.data = True
            self.done_pub.publish(done_msg)
            return

        # Distance Sorting
        valid_frontiers.sort(key=lambda f: math.hypot(f[0] - rx, f[1] - ry))
        best_frontier = valid_frontiers[0]

        self.current_goal = best_frontier
        self.goal_start_time = time.time()
        self.last_robot_pose = (rx, ry)
        self.stuck_counter = 0

        self.get_logger().info(f"Dispatched target frontier: ({best_frontier[0]:.2f}, {best_frontier[1]:.2f})")

        goal_msg = PoseStamped()
        goal_msg.header.frame_id = self.map_frame
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = best_frontier[0]
        goal_msg.pose.position.y = best_frontier[1]
        goal_msg.pose.orientation.w = 1.0

        self.goal_pub.publish(goal_msg)

    def reset_goal_states(self):
        self.current_goal = None
        self.goal_start_time = None
        self.last_robot_pose = None
        self.stuck_counter = 0


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
