import rclpy
from rclpy.node import Node
import numpy as np
import heapq

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener


class AStarPlanner(Node):
    def __init__(self):
        super().__init__('astar_planner')

        self.declare_parameter('robot_name', 'robot1')
        self.declare_parameter('inflation_cells', 3)
        self.declare_parameter('occupied_threshold', 50)

        self.robot_name = self.get_parameter('robot_name').value
        self.inflation_cells = self.get_parameter('inflation_cells').value
        self.occ_thresh = self.get_parameter('occupied_threshold').value

        self.map_frame = 'map'
        self.base_frame = f'{self.robot_name}/base_link'

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.map = None

        self.map_sub = self.create_subscription(
            OccupancyGrid, f'/{self.robot_name}/map', self.map_callback, 1)
        self.goal_sub = self.create_subscription(
            PoseStamped, f'/{self.robot_name}/frontier_goal', self.goal_callback, 10)
        self.path_pub = self.create_publisher(
            Path, f'/{self.robot_name}/planned_path', 10)

        self.get_logger().info('A* planner ready.')

    def map_callback(self, msg):
        self.map = msg

    def get_robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time())
            return tf.transform.translation.x, tf.transform.translation.y
        except Exception:
            return None

    def world_to_grid(self, x, y):
        info = self.map.info
        return (int((x - info.origin.position.x) / info.resolution),
                int((y - info.origin.position.y) / info.resolution))

    def grid_to_world(self, gx, gy):
        info = self.map.info
        return ((gx + 0.5) * info.resolution + info.origin.position.x,
                (gy + 0.5) * info.resolution + info.origin.position.y)

    def build_cspace(self, grid, width, height):
        occupied = grid >= self.occ_thresh
        cspace = occupied.copy()
        k = self.inflation_cells
        for dy in range(-k, k + 1):
            for dx in range(-k, k + 1):
                if dx == 0 and dy == 0:
                    continue
                shifted = np.roll(np.roll(occupied, dy, axis=0), dx, axis=1)
                cspace |= shifted
        return cspace

    def is_walkable(self, cspace, x, y, width, height):
        return 0 <= x < width and 0 <= y < height and not cspace[y, x]

    def a_star(self, start, goal, cspace, width, height):
        if not self.is_walkable(cspace, *start, width, height):
            self.get_logger().warn('Start cell not walkable — path may fail.')
        if not self.is_walkable(cspace, *goal, width, height):
            self.get_logger().warn('Goal cell not walkable — path may fail.')

        neighbors = [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]

        open_set = [(0, start)]
        came_from = {}
        g_score = {start: 0}

        def h(a, b):
            return ((a[0]-b[0])**2 + (a[1]-b[1])**2) ** 0.5

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
                if not self.is_walkable(cspace, nx, ny, width, height):
                    continue
                step_cost = (dx*dx + dy*dy) ** 0.5
                tentative = g_score[current] + step_cost
                neighbor = (nx, ny)
                if neighbor not in g_score or tentative < g_score[neighbor]:
                    g_score[neighbor] = tentative
                    priority = tentative + h(neighbor, goal)
                    heapq.heappush(open_set, (priority, neighbor))
                    came_from[neighbor] = current
        return None

    def goal_callback(self, msg):
        if self.map is None:
            self.get_logger().warn('No map yet — cannot plan.')
            return
        pose = self.get_robot_pose()
        if pose is None:
            self.get_logger().warn('No robot pose yet — cannot plan.')
            return

        info = self.map.info
        width, height = info.width, info.height
        grid = np.array(self.map.data, dtype=np.int16).reshape((height, width))
        cspace = self.build_cspace(grid, width, height)

        start = self.world_to_grid(*pose)
        goal = self.world_to_grid(msg.pose.position.x, msg.pose.position.y)

        path_cells = self.a_star(start, goal, cspace, width, height)
        if path_cells is None:
            self.get_logger().warn('No path found to frontier goal.')
            return

        path_msg = Path()
        path_msg.header.frame_id = self.map_frame
        path_msg.header.stamp = self.get_clock().now().to_msg()
        for gx, gy in path_cells:
            wx, wy = self.grid_to_world(gx, gy)
            p = PoseStamped()
            p.header = path_msg.header
            p.pose.position.x = wx
            p.pose.position.y = wy
            p.pose.orientation.w = 1.0
            path_msg.poses.append(p)

        self.path_pub.publish(path_msg)
        self.get_logger().info(f'Published path with {len(path_cells)} waypoints.')


def main(args=None):
    rclpy.init(args=args)
    node = AStarPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()