import rclpy
from rclpy.node import Node
import numpy as np
from collections import deque

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')

        self.declare_parameter('robot_name', 'robot1')
        #Frontier size selection to explore
        self.declare_parameter('min_frontier_size', 8)
        #if it visited near 50cm of the frontier, it will not revisit it
        self.declare_parameter('revisit_radius', 0.5)
        self.declare_parameter('poll_period', 2.0)

        self.robot_name = self.get_parameter('robot_name').value
        self.min_frontier_size = self.get_parameter('min_frontier_size').value
        self.revisit_radius = self.get_parameter('revisit_radius').value
        poll_period = self.get_parameter('poll_period').value

        self.map_frame = 'map'
        self.base_frame = f'{self.robot_name}/base_link'

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.map = None
        self.visited = []  

        self.map_sub = self.create_subscription(
            OccupancyGrid, f'/{self.robot_name}/map', self.map_callback, 1)
        self.goal_pub = self.create_publisher(
            PoseStamped, f'/{self.robot_name}/frontier_goal', 10)
        self.done_pub = self.create_publisher(bool,
    '/robot1/exploration_done',
    10
)
        self.create_timer(poll_period, self.explore)
        self.get_logger().info('Frontier explorer ready.')

    def map_callback(self, msg):
        self.map = msg

    def get_robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time())
            return tf.transform.translation.x, tf.transform.translation.y
        except Exception:
            self.get_logger().debug('Waiting for TF...')
            return None

    def world_to_grid(self, x, y):
        info = self.map.info
        gx = int((x - info.origin.position.x) / info.resolution)
        gy = int((y - info.origin.position.y) / info.resolution)
        return gx, gy

    def grid_to_world(self, gx, gy):
        info = self.map.info
        x = (gx + 0.5) * info.resolution + info.origin.position.x
        y = (gy + 0.5) * info.resolution + info.origin.position.y
        return x, y

    def find_frontiers(self):
        info = self.map.info
        width, height = info.width, info.height
        grid = np.array(self.map.data, dtype=np.int16).reshape((height, width))

        free_mask = (grid == 0)
        unknown_mask = (grid == -1)

        unknown_adjacent = np.zeros_like(unknown_mask)
        unknown_adjacent[:-1, :] |= unknown_mask[1:, :]
        unknown_adjacent[1:, :] |= unknown_mask[:-1, :]
        unknown_adjacent[:, :-1] |= unknown_mask[:, 1:]
        unknown_adjacent[:, 1:] |= unknown_mask[:, :-1]

        frontier_mask = free_mask & unknown_adjacent
        if not frontier_mask.any():
            return []

        visited = np.zeros_like(frontier_mask)
        centroids = []
        cells = np.argwhere(frontier_mask)

        for sy, sx in cells:
            if visited[sy, sx]:
                continue
            queue = deque([(int(sy), int(sx))])
            visited[sy, sx] = True
            cluster = []
            while queue:
                y, x = queue.popleft()
                cluster.append((y, x))
                for ny, nx in ((y-1, x), (y+1, x), (y, x-1), (y, x+1)):
                    if 0 <= ny < height and 0 <= nx < width and not visited[ny, nx] and frontier_mask[ny, nx]:
                        visited[ny, nx] = True
                        queue.append((ny, nx))
            if len(cluster) < self.min_frontier_size:
                continue
            arr = np.array(cluster)
            cy, cx = arr.mean(axis=0)
            centroids.append(self.grid_to_world(cx, cy))
        return centroids

    def already_visited(self, x, y):
        return any(
            ((x - vx) ** 2 + (y - vy) ** 2) ** 0.5 < self.revisit_radius
            for vx, vy in self.visited)

    def explore(self):
        if self.map is None:
            return
        pose = self.get_robot_pose()
        if pose is None:
            return
        rx, ry = pose

        frontiers = self.find_frontiers()
        if not frontiers:
            self.get_logger().info('No frontiers found — exploration may be complete.')
            msg = bool()
            msg.data = True
            self.done_pub.publish(msg)
    
            return

     
       
        best, best_dist = None, float('inf')
        for fx, fy in frontiers:
            if self.already_visited(fx, fy):
                continue
            d = ((fx - rx) ** 2 + (fy - ry) ** 2) ** 0.5
            if d < best_dist:
                best_dist, best = d, (fx, fy)

        if best is None:
            self.get_logger().info('All frontiers already visited.')
            return

        fx, fy = best
        self.visited.append(best)

        goal = PoseStamped()
        goal.header.frame_id = self.map_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = fx
        goal.pose.position.y = fy
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)
        self.get_logger().info(f'New frontier goal: ({fx:.2f}, {fy:.2f})')


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()