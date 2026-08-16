import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry

from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose

from std_msgs.msg import Bool


class FrontierExplorer(Node):

    def __init__(self):

        super().__init__('frontier_explorer')

        # ============================================================
        # PARAMETERS
        # ============================================================

        self.declare_parameter('frontier_cluster_distance', 0.30)
        self.declare_parameter('minimum_frontier_distance', 0.40)
        self.declare_parameter('max_frontier_retries', 3)

        self.cluster_distance = self.get_parameter(
            'frontier_cluster_distance'
        ).value

        self.minimum_frontier_distance = self.get_parameter(
            'minimum_frontier_distance'
        ).value

        self.max_frontier_retries = self.get_parameter(
            'max_frontier_retries'
        ).value

        # ============================================================
        # ROBOT STATE
        # ============================================================

        self.robot1_x = None
        self.robot1_y = None

        self.map_msg = None

        # Current goal
        self.robot1_goals = []

        # ============================================================
        # FAILED FRONTIERS
        # key: (x, y) rounded tuple -> failure count
        # ============================================================

        self.failed_frontiers = {}

        # ============================================================
        # MAP
        # ============================================================

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        # ============================================================
        # ODOMETRY
        # ============================================================

        self.robot1_odom_sub = self.create_subscription(
            Odometry,
            '/robot1/odometry/filtered',
            self.robot1_odom_callback,
            10
        )

        # ============================================================
        # FRONTIER GOAL
        # ============================================================

        self.robot1_goal_pub = self.create_publisher(
            PoseArray,
            '/robot1/frontier_goals',
            10
        )

        # ============================================================
        # GOAL REACHED (from pure_pursuit)
        # ============================================================

        self.robot1_goal_reached_sub = self.create_subscription(
            Bool,
            '/robot1/goal_reached',
            self.robot1_goal_reached_callback,
            10
        )

        # ============================================================
        # PLANNING FAILED (from astar_planner)  -- NEW
        # ============================================================

        self.robot1_planning_failed_sub = self.create_subscription(
            Bool,
            '/robot1/planning_failed',
            self.robot1_planning_failed_callback,
            10
        )

        # ============================================================
        # EXPLORATION TIMER
        # ============================================================

        self.timer = self.create_timer(
            2.0,
            self.explore
        )

        self.get_logger().info(
            'Frontier Explorer started - SINGLE ROBOT MODE'
        )

    # ================================================================
    # MAP CALLBACK
    # ================================================================

    def map_callback(self, msg):

        self.map_msg = msg

    # ================================================================
    # ODOMETRY CALLBACK
    # ================================================================

    def robot1_odom_callback(self, msg):

        self.robot1_x = msg.pose.pose.position.x
        self.robot1_y = msg.pose.pose.position.y

    # ================================================================
    # GOAL REACHED CALLBACK
    # ================================================================

    def robot1_goal_reached_callback(self, msg):

        if msg.data:

            self.get_logger().info(
                'Robot 1 reached frontier goal. '
                'Searching for next frontier.'
            )

            # Successful reach -> record as visited/failed-permanently
            # so we never target this exact spot again.
            if self.robot1_goals:
                self._record_failure(self.robot1_goals[0], permanent=True)

            self.robot1_goals = []

    # ================================================================
    # PLANNING FAILED CALLBACK  -- NEW
    # ================================================================

    def robot1_planning_failed_callback(self, msg):

        if msg.data and self.robot1_goals:

            failed_goal = self.robot1_goals[0]

            self.get_logger().warn(
                f'A* could not plan to {failed_goal}. '
                f'Recording failure and picking a new frontier.'
            )

            self._record_failure(failed_goal, permanent=False)

            # Clear so explore() selects a new frontier next cycle.
            self.robot1_goals = []

    # ================================================================
    # RECORD FAILURE HELPER  -- NEW
    # ================================================================

    def _record_failure(self, frontier, permanent=False):

        key = (round(frontier[0], 2), round(frontier[1], 2))

        if permanent:
            # Force it above the retry threshold so is_failed() always
            # rejects it from now on.
            self.failed_frontiers[key] = self.max_frontier_retries
        else:
            self.failed_frontiers[key] = (
                self.failed_frontiers.get(key, 0) + 1
            )

    # ================================================================
    # MAIN EXPLORATION
    # ================================================================

    def explore(self):

        if self.map_msg is None:

            self.get_logger().info('Waiting for /map...')
            return

        if self.robot1_x is None:

            self.get_logger().info('Waiting for Robot 1 position...')
            return

        # ============================================================
        # Do not create another goal while current goal exists.
        # ============================================================

        if self.robot1_goals:

            return

        # ============================================================
        # FIND FRONTIERS
        # ============================================================

        frontier_cells = self.find_frontier_cells()

        if not frontier_cells:

            self.get_logger().info('No frontiers found.')
            return

        # ============================================================
        # CLUSTER
        # ============================================================

        clusters = self.cluster_frontiers(frontier_cells)

        frontier_points = []

        for cluster in clusters:

            point = self.cluster_center(cluster)

            if point is not None:

                # NEW: snap the centroid to the nearest actually-free
                # cell, so A* never gets handed a bad goal cell.
                snapped = self.snap_to_free(point)

                if snapped is not None:
                    frontier_points.append(snapped)

        # ============================================================
        # REMOVE FRONTIERS TOO CLOSE TO ROBOT
        # ============================================================

        useful_frontiers = []

        for frontier in frontier_points:

            x, y = frontier

            distance = self.distance(
                self.robot1_x, self.robot1_y, x, y
            )

            if distance >= self.minimum_frontier_distance:

                useful_frontiers.append(frontier)

        if not useful_frontiers:

            self.get_logger().info('No useful frontiers found.')
            return

        # ============================================================
        # SORT BY DISTANCE
        # ============================================================

        useful_frontiers.sort(
            key=lambda frontier: self.distance(
                self.robot1_x, self.robot1_y,
                frontier[0], frontier[1]
            )
        )

        # ============================================================
        # SELECT FRONTIER
        # ============================================================

        selected_frontier = None

        for frontier in useful_frontiers:

            if not self.is_failed(frontier):

                selected_frontier = frontier
                break

        if selected_frontier is None:

            self.get_logger().info(
                'All available frontiers have failed. '
                'Exploration may be complete.'
            )
            return

        # ============================================================
        # SAVE CURRENT GOAL
        # ============================================================

        self.robot1_goals = [selected_frontier]

        self.get_logger().info(
            f'New Robot 1 frontier goal: '
            f'x={selected_frontier[0]:.2f}, '
            f'y={selected_frontier[1]:.2f}'
        )

        self.publish_robot1_goals()

    # ================================================================
    # SNAP CENTROID TO NEAREST FREE CELL  -- NEW
    # ================================================================

    def snap_to_free(self, point):
        """
        Frontier cluster centroids can land on unknown/occupied cells
        (since they're an average of many cells). This searches an
        expanding ring around the centroid for the nearest actually
        FREE (value == 0) cell, and returns its world coordinates.
        """

        info = self.map_msg.info
        resolution = info.resolution
        origin_x = info.origin.position.x
        origin_y = info.origin.position.y
        width = info.width
        height = info.height
        data = self.map_msg.data

        gx = int((point[0] - origin_x) / resolution)
        gy = int((point[1] - origin_y) / resolution)

        def is_free(x, y):
            if x < 0 or x >= width or y < 0 or y >= height:
                return False
            return data[y * width + x] == 0

        if is_free(gx, gy):
            return point

        for radius in range(1, 15):
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    nx, ny = gx + dx, gy + dy
                    if is_free(nx, ny):
                        world_x = origin_x + (nx + 0.5) * resolution
                        world_y = origin_y + (ny + 0.5) * resolution
                        return (world_x, world_y)

        # Nothing free found nearby -- drop this frontier.
        return None

    # ================================================================
    # FIND FRONTIER CELLS
    # ================================================================

    def find_frontier_cells(self):

        width = self.map_msg.info.width
        height = self.map_msg.info.height

        data = self.map_msg.data

        frontier_cells = []

        for y in range(1, height - 1):

            for x in range(1, width - 1):

                index = y * width + x

                if data[index] != 0:
                    continue

                neighbours = [
                    data[index - 1],
                    data[index + 1],
                    data[index - width],
                    data[index + width]
                ]

                if -1 in neighbours:
                    frontier_cells.append((x, y))

        return frontier_cells

    # ================================================================
    # CLUSTER FRONTIERS
    # ================================================================

    def cluster_frontiers(self, cells):

        clusters = []

        resolution = self.map_msg.info.resolution

        threshold_cells = max(
            1, int(self.cluster_distance / resolution)
        )

        unused = set(cells)

        while unused:

            seed = unused.pop()
            cluster = [seed]
            queue = [seed]

            while queue:

                current = queue.pop()
                cx, cy = current

                for dx in range(-threshold_cells, threshold_cells + 1):
                    for dy in range(-threshold_cells, threshold_cells + 1):

                        if dx == 0 and dy == 0:
                            continue

                        neighbour = (cx + dx, cy + dy)

                        if neighbour in unused:
                            unused.remove(neighbour)
                            cluster.append(neighbour)
                            queue.append(neighbour)

            if len(cluster) >= 3:
                clusters.append(cluster)

        return clusters

    # ================================================================
    # CLUSTER CENTER
    # ================================================================

    def cluster_center(self, cluster):

        if not cluster:
            return None

        avg_x = sum(cell[0] for cell in cluster) / len(cluster)
        avg_y = sum(cell[1] for cell in cluster) / len(cluster)

        resolution = self.map_msg.info.resolution
        origin_x = self.map_msg.info.origin.position.x
        origin_y = self.map_msg.info.origin.position.y

        world_x = origin_x + (avg_x + 0.5) * resolution
        world_y = origin_y + (avg_y + 0.5) * resolution

        return world_x, world_y

    # ================================================================
    # FAILED FRONTIER
    # ================================================================

    def is_failed(self, frontier):

        x, y = frontier

        for (failed_x, failed_y), attempts in self.failed_frontiers.items():

            distance = self.distance(x, y, failed_x, failed_y)

            if distance < 0.30 and attempts >= self.max_frontier_retries:
                return True

        return False

    # ================================================================
    # DISTANCE
    # ================================================================

    def distance(self, x1, y1, x2, y2):

        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    # ================================================================
    # PUBLISH GOAL
    # ================================================================

    def publish_robot1_goals(self):

        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        for x, y in self.robot1_goals:
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.orientation.w = 1.0
            msg.poses.append(pose)

        self.robot1_goal_pub.publish(msg)


def main(args=None):

    rclpy.init(args=args)
    node = FrontierExplorer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()