import rclpy
from rclpy.node import Node
import numpy as np
import heapq
import math
from collections import deque

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener


class AStarPlanner(Node):

    def __init__(self):
        super().__init__('astar_planner')

        self.declare_parameter('robot_name', 'robot1')
        self.declare_parameter('inflation_cells', 4)  
        self.declare_parameter('occupied_threshold', 50)

        self.robot_name = self.get_parameter('robot_name').value
        self.inflation_cells = self.get_parameter('inflation_cells').value
        self.occ_thresh = self.get_parameter('occupied_threshold').value

        self.map_frame = "map"
        self.base_frame = f"{self.robot_name}/base_link"

        self.map = None
        self.last_goal = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.map_sub = self.create_subscription(OccupancyGrid, "/map", self.map_callback, 1)
        self.goal_sub = self.create_subscription(PoseStamped, f"/{self.robot_name}/frontier_goal", self.goal_callback, 10)
        self.path_pub = self.create_publisher(Path, f"/{self.robot_name}/planned_path", 10)

        self.get_logger().info("A* Cost-Gradient Planner ready.")

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

    def build_cspace_and_costs(self, grid):
        occupied = (grid >= self.occ_thresh) | (grid == -1)
        cspace = occupied.copy()
        h, w = grid.shape
        cost_gradient = np.zeros((h, w), dtype=np.float32)
        
        k = self.inflation_cells
        for dy in range(-k, k + 1):
            for dx in range(-k, k + 1):
                if dx == 0 and dy == 0:
                    continue
                y_start, y_end = max(0, dy), min(h, h + dy)
                x_start, x_end = max(0, dx), min(w, w + dx)
                oy_start, oy_end = max(0, -dy), min(h, h - dy)
                ox_start, ox_end = max(0, -dx), min(w, w - dx)
                
                if max(abs(dx), abs(dy)) <= 2: 
                    cspace[y_start:y_end, x_start:x_end] |= occupied[oy_start:oy_end, ox_start:ox_end]
                
                weight = 8.0 / (math.hypot(dx, dy) + 0.1)
                cost_gradient[y_start:y_end, x_start:x_end] += occupied[oy_start:oy_end, ox_start:ox_end] * weight

        return cspace, cost_gradient

    def is_walkable(self, cspace, x, y):
        h, w = cspace.shape
        return (0 <= x < w and 0 <= y < h and not cspace[y, x])

    def find_nearest_free(self, cell, cspace):
        if cell is None:
            return None
        x, y = cell
        h, w = cspace.shape

        x = max(0, min(w - 1, x))
        y = max(0, min(h - 1, y))

        if self.is_walkable(cspace, x, y):
            return x, y

        queue = deque([(x, y)])
        visited = set([(x, y)])
        
        while queue:
            cx, cy = queue.popleft()
            if self.is_walkable(cspace, cx, cy):
                return cx, cy

            for dx, dy in [(-1,0), (1,0), (0,-1), (0,1), (-1,-1), (-1,1), (1,-1), (1,1)]:
                nx = max(0, min(w - 1, cx + dx))
                ny = max(0, min(h - 1, cy + dy))
                
                if (nx, ny) not in visited:
                    visited.add((nx, ny))
                    queue.append((nx, ny))
        return None

    def heuristic(self, a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def a_star(self, start, goal, cspace, cost_gradient):
        neighbors = [(-1,0), (1,0), (0,-1), (0,1), (-1,-1), (-1,1), (1,-1), (1,1)]
        open_set = []
        heapq.heappush(open_set, (0.0, start))
        came_from = {}
        g_score = {start: 0.0}

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                path.reverse()
                return path

            for dx, dy in neighbors:
                nx, ny = current[0] + dx, current[1] + dy
                if not self.is_walkable(cspace, nx, ny):
                    continue

                neighbour = (nx, ny)
                base_cost = 1.414 if (dx != 0 and dy != 0) else 1.0
                new_cost = g_score[current] + base_cost + cost_gradient[ny, nx]

                if neighbour not in g_score or new_cost < g_score[neighbour]:
                    g_score[neighbour] = new_cost
                    priority = new_cost + self.heuristic(neighbour, goal)
                    heapq.heappush(open_set, (priority, neighbour))
                    came_from[neighbour] = current
        return None

    def goal_callback(self, msg):
        if self.map is None:
            return

        pose = self.get_robot_pose()
        if pose is None:
            self.get_logger().warn("A* waiting for TF pose...")
            return

        goal_xy = (msg.pose.position.x, msg.pose.position.y)
        if goal_xy == self.last_goal:
            return
        self.last_goal = goal_xy

        info = self.map.info
        grid = np.array(self.map.data, dtype=np.int16).reshape(info.height, info.width)
        cspace, cost_gradient = self.build_cspace_and_costs(grid)

        start = self.find_nearest_free(self.world_to_grid(*pose), cspace)
        goal = self.find_nearest_free(self.world_to_grid(*goal_xy), cspace)

        if start is None or goal is None:
            self.get_logger().warn("A* could not resolve grid placement bounds.")
            return

        self.get_logger().info(f"Planning Path: Start {start} -> Goal {goal}")
        path = self.a_star(start, goal, cspace, cost_gradient)
        
        if path is None:
            self.get_logger().error("A* search tracking calculation failed.")
            return

        # Path smoothing filter (Removes redundant linear points)
        smoothed_path = []
        if len(path) > 2:
            smoothed_path.append(path[0])
            for i in range(1, len(path) - 1):
                dx1 = path[i][0] - path[i-1][0]
                dy1 = path[i][1] - path[i-1][1]
                dx2 = path[i+1][0] - path[i][0]
                dy2 = path[i+1][1] - path[i][1]
                if dx1 == dx2 and dy1 == dy2:
                    continue  
                smoothed_path.append(path[i])
            smoothed_path.append(path[-1])
        else:
            smoothed_path = path

        path_msg = Path()
        path_msg.header.frame_id = self.map_frame
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for gx, gy in smoothed_path:
            x, y = self.grid_to_world(gx, gy)
            p = PoseStamped()
            p.header.frame_id = self.map_frame
            p.header.stamp = path_msg.header.stamp
            p.pose.position.x = x
            p.pose.position.y = y
            p.pose.orientation.w = 1.0
            path_msg.poses.append(p)

        self.path_pub.publish(path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = AStarPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
