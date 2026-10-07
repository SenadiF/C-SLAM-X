import math
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from my_navigation.robot_pose import MapFramePose

from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry

from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose

from std_msgs.msg import Bool


# Goals and paths are sent once per goal. Latched (transient local) on both
# ends, so a subscriber that connects late - DDS matching right after start-up,
# or a restarted node - still gets the current goal/path. Volatile, a path
# published before pure_pursuit had matched was lost and every node waited on
# the others forever.
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class FrontierExplorer(Node):

    def __init__(self):

        super().__init__('frontier_explorer')
        self.pose_tracker = MapFramePose(self)

        self.declare_parameter(
            'frontier_cluster_distance',
            0.30
        )

        self.declare_parameter(
            'minimum_frontier_distance',
            0.40
        )

        self.declare_parameter(
            'max_frontier_retries',
            3
        )

        # Minimum distance between goals assigned to the two robots. Thereby they wont explore the same frontier.

        self.declare_parameter(
            'robot_goal_separation',
            0.60
        )

        # Spread the robots out: a frontier within spread_radius (m) of the
        # other robot or its goal costs spread_weight * (spread_radius - d)
        # extra metres, and when both robots need a goal the pair is chosen
        # together (select_goal_pair). 0 = off, the old nearest-frontier
        # behaviour (kept as the default so simulation results compare).
        self.declare_parameter('spread_radius', 0.0)
        self.declare_parameter('spread_weight', 1.0)
        self.spread_radius = self.get_parameter('spread_radius').value
        self.spread_weight = self.get_parameter('spread_weight').value

        # Must match astar_planner's obstacle_inflation_radius - otherwise
        # this picks goals that are bare-free but still fall inside A*'s
        # safety margin near a wall, and A* rejects every one of them.
        self.declare_parameter(
            'obstacle_inflation_radius',
            0.15
        )

        self.obstacle_inflation_radius = self.get_parameter(
            'obstacle_inflation_radius'
        ).value

        # Must match astar_planner's inflate_unknown (see there).
        self.declare_parameter('inflate_unknown', True)
        self.inflate_unknown = self.get_parameter('inflate_unknown').value

        # 'centroid': one goal per frontier cluster, its center snapped to
        # the nearest safe cell. 'reachable': goals are safe cells the
        # robot can actually reach that touch a frontier, one per
        # frontier_tile_size tile. On a real LiDAR map every ray's edges are
        # frontier, so all frontier cells merge into one cluster whose
        # center lies in unknown space - use 'reachable' on hardware.
        self.declare_parameter('goal_selection', 'centroid')
        self.goal_selection = self.get_parameter('goal_selection').value

        self.declare_parameter('frontier_tile_size', 1.0)
        self.frontier_tile_size = self.get_parameter('frontier_tile_size').value

        # Loop-closure corrections mean the map keeps subtly changing even
        # once real exploration is done, so "zero frontier cells left" is
        # too strict a stop condition to ever reliably trigger. Stop once
        # this fraction of the map is known (occupied or free), matching
        # the ss stack's own is_exploration_complete threshold.
        self.declare_parameter(
            'exploration_complete_threshold',
            0.90
        )

        self.exploration_complete_threshold = self.get_parameter(
            'exploration_complete_threshold'
        ).value

        self.cluster_distance = self.get_parameter(
            'frontier_cluster_distance'
        ).value

        self.minimum_frontier_distance = self.get_parameter(
            'minimum_frontier_distance'
        ).value

        self.max_frontier_retries = self.get_parameter(
            'max_frontier_retries'
        ).value

        self.robot_goal_separation = self.get_parameter(
            'robot_goal_separation'
        ).value

        self.robot1_x = None
        self.robot1_y = None

        self.robot1_goals = []

        self.robot2_x = None
        self.robot2_y = None

        self.robot2_goals = []

        self.map_msg = None
        self.failed_frontiers = {}

        # Latched once true - a map that briefly dips back under the
        # threshold from loop-closure jitter shouldn't resume exploring.
        self.exploration_complete = False

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        self.robot1_odom_sub = self.create_subscription(
            Odometry,
            '/robot1/odometry/filtered',
            self.robot1_odom_callback,
            10
        )

        self.robot2_odom_sub = self.create_subscription(
            Odometry,
            '/robot2/odometry/filtered',
            self.robot2_odom_callback,
            10
        )

        self.robot1_goal_pub = self.create_publisher(
            PoseArray,
            '/robot1/frontier_goals',
            LATCHED
        )

        self.robot2_goal_pub = self.create_publisher(
            PoseArray,
            '/robot2/frontier_goals',
            LATCHED
        )

        self.robot1_goal_reached_sub = self.create_subscription(
            Bool,
            '/robot1/goal_reached',
            self.robot1_goal_reached_callback,
            10
        )

        self.robot2_goal_reached_sub = self.create_subscription(
            Bool,
            '/robot2/goal_reached',
            self.robot2_goal_reached_callback,
            10
        )

        self.robot1_planning_failed_sub = self.create_subscription(
            Bool,
            '/robot1/planning_failed',
            self.robot1_planning_failed_callback,
            10
        )

        self.robot2_planning_failed_sub = self.create_subscription(
            Bool,
            '/robot2/planning_failed',
            self.robot2_planning_failed_callback,
            10
        )

        self.timer = self.create_timer(
            2.0,
            self.explore
        )

        self.get_logger().info(
            'Frontier Explorer started - TWO ROBOT MODE'
        )


    def map_callback(self, msg):

        self.map_msg = msg

    def robot1_odom_callback(self, msg):

        self.robot1_x, self.robot1_y, _ = self.pose_tracker.get('robot1', msg)

    def robot2_odom_callback(self, msg):

        self.robot2_x, self.robot2_y, _ = self.pose_tracker.get('robot2', msg)

    def robot1_goal_reached_callback(self, msg):

        if not msg.data:
            return

        self.get_logger().info(
            'Robot 1 reached frontier goal.'
        )

        if self.robot1_goals:

            self._record_failure(
                self.robot1_goals[0],
                permanent=True
            )

        self.robot1_goals = []


    def robot2_goal_reached_callback(self, msg):

        if not msg.data:
            return

        self.get_logger().info(
            'Robot 2 reached frontier goal.'
        )

        if self.robot2_goals:

            self._record_failure(
                self.robot2_goals[0],
                permanent=True
            )

        self.robot2_goals = []

#Robot 1 planning failed callback function to handle the case when robot 1 fails to plan a path to its assigned frontier goal. It records the failure and clears the robot's goals.
    def robot1_planning_failed_callback(self, msg):

        if msg.data and self.robot1_goals:

            failed_goal = self.robot1_goals[0]

            self.get_logger().warn(
                f'Robot 1 A* failed for '
                f'{failed_goal}'
            )

            self._record_failure(
                failed_goal,
                permanent=False
            )

            self.robot1_goals = []


    def robot2_planning_failed_callback(self, msg):

        if msg.data and self.robot2_goals:

            failed_goal = self.robot2_goals[0]

            self.get_logger().warn(
                f'Robot 2 A* failed for '
                f'{failed_goal}'
            )

            self._record_failure(
                failed_goal,
                permanent=False
            )

            self.robot2_goals = []

#Record the failed frontiers
    def _record_failure(
        self,
        frontier,
        permanent=False
    ):

        # Merge into an existing nearby failure bucket if one is close by,
        # so a cluster center that jitters slightly cycle to cycle (normal
        # as the map fills in) still accumulates toward the same retry
        # budget instead of each variant getting a fresh one.
        key = None

        for existing_key in self.failed_frontiers:

            if (
                self.distance(
                    frontier[0],
                    frontier[1],
                    existing_key[0],
                    existing_key[1]
                )
                < 0.30
            ):

                key = existing_key
                break

        if key is None:

            key = (
                round(frontier[0], 2),
                round(frontier[1], 2)
            )

        if permanent:

            self.failed_frontiers[key] = (
                self.max_frontier_retries
            )

        else:

            self.failed_frontiers[key] = (
                self.failed_frontiers.get(key, 0) + 1
            )

    def explore(self):

        if self.exploration_complete:
            return

        if self.map_msg is None:

            self.get_logger().info(
                'Waiting for /map...'
            )

            return

        if (
            self.robot1_x is None
            and self.robot2_x is None
        ):

            self.get_logger().info(
                'Waiting for at least one robot position...'
            )

            return

        known_ratio = self.known_map_ratio()

        if known_ratio >= self.exploration_complete_threshold:

            self.exploration_complete = True

            self.robot1_goals = []
            self.robot2_goals = []

            self.get_logger().info(
                f'Exploration complete! {known_ratio * 100:.1f}% of map known '
                f'(threshold {self.exploration_complete_threshold * 100:.0f}%). '
                f'No further frontier goals will be assigned.'
            )

            return

#Find the frontier cells

        frontier_cells = (
            self.find_frontier_cells()
        )

        if not frontier_cells:

            self.get_logger().info(
                'No frontiers found.'
            )

            return

        if self.goal_selection == 'reachable':

            frontier_points = self.reachable_frontier_goals(
                frontier_cells
            )

            clusters = []

        else:

            frontier_points = []

#Cluster

            clusters = self.cluster_frontiers(
                frontier_cells
            )

        for cluster in clusters:

            point = self.cluster_center(
                cluster
            )

            if point is None:
                continue

            snapped = self.snap_to_free(
                point
            )

            if snapped is not None:

                frontier_points.append(
                    snapped
                )

        if not frontier_points:

            self.get_logger().info(
                'No valid frontier points.'
            )

            return

        # REMOVE FRONTIERS TOO CLOSE TO ROBOTS
        useful_frontiers = []

        for frontier in frontier_points:

            x, y = frontier

            robot_distances = []

            if self.robot1_x is not None:

                distance_robot1 = self.distance(
                    self.robot1_x,
                    self.robot1_y,
                    x,
                    y
                )

                robot_distances.append(
                    distance_robot1
                )

            if self.robot2_x is not None:

                distance_robot2 = self.distance(
                    self.robot2_x,
                    self.robot2_y,
                    x,
                    y
                )

                robot_distances.append(
                    distance_robot2
                )

            if any(
                distance >= self.minimum_frontier_distance
                for distance in robot_distances
            ):

                useful_frontiers.append(
                    frontier
                )

        if not useful_frontiers:

            self.get_logger().info(
                'No useful frontiers.'
            )

            return

        #currently reserved frontiers

        reserved = []

        if self.robot1_goals:

            reserved.append(
                self.robot1_goals[0]
            )

        if self.robot2_goals:

            reserved.append(
                self.robot2_goals[0]
            )

        need1 = self.robot1_x is not None and not self.robot1_goals
        need2 = self.robot2_x is not None and not self.robot2_goals

        # Both free (e.g. at the start): choose the two goals together.
        pair = None
        if need1 and need2 and self.spread_radius > 0.0:
            pair = self.select_goal_pair(useful_frontiers, reserved)

#Assign robot 1 frontiers

        if need1:

            # Stay away from robot 2 and where it is heading.
            avoid1 = [(self.robot2_x, self.robot2_y)] if self.robot2_x is not None else []
            avoid1 += self.robot2_goals[:1]

            robot1_frontier = pair[0] if pair else (
                self.select_frontier_for_robot(
                    useful_frontiers,
                    self.robot1_x,
                    self.robot1_y,
                    reserved,
                    avoid1
                )
            )

            if robot1_frontier is not None:

                self.robot1_goals = [
                    robot1_frontier
                ]

                reserved.append(
                    robot1_frontier
                )

                self.get_logger().info(
                    f'Robot 1 new frontier: '
                    f'x={robot1_frontier[0]:.2f}, '
                    f'y={robot1_frontier[1]:.2f}'
                )

                self.publish_robot1_goals()

#Assign robot 2 frontiers

        if need2:

            # Stay away from robot 1 and where it is heading.
            avoid2 = [(self.robot1_x, self.robot1_y)] if self.robot1_x is not None else []
            avoid2 += self.robot1_goals[:1]

            robot2_frontier = pair[1] if pair else (
                self.select_frontier_for_robot(
                    useful_frontiers,
                    self.robot2_x,
                    self.robot2_y,
                    reserved,
                    avoid2
                )
            )

            if robot2_frontier is not None:

                self.robot2_goals = [
                    robot2_frontier
                ]

                self.get_logger().info(
                    f'Robot 2 new frontier: '
                    f'x={robot2_frontier[0]:.2f}, '
                    f'y={robot2_frontier[1]:.2f}'
                )

                self.publish_robot2_goals()


    def select_frontier_for_robot(
        self,
        frontiers,
        robot_x,
        robot_y,
        reserved,
        avoid=()
    ):

        candidates = self.frontier_candidates(
            frontiers, robot_x, robot_y, reserved, avoid)

        if not candidates:

            return None

        # Lowest cost first: distance, plus the spread penalty near the
        # other robot.
        candidates.sort(
            key=lambda item: item[0]
        )

        return candidates[0][1]

    def spread_penalty(self, frontier, avoid):
        """Extra cost (m) for a frontier within spread_radius of `avoid`.

        `avoid` holds the other robot's position and goal. Without it both
        robots, starting close together, each took their nearest frontier
        and explored the same area - the 0.6 m robot_goal_separation only
        stops two goals from being on top of each other.
        """
        if self.spread_radius <= 0.0 or not avoid:
            return 0.0
        nearest = min(self.distance(frontier[0], frontier[1], a[0], a[1]) for a in avoid)
        return self.spread_weight * max(0.0, self.spread_radius - nearest)

    def frontier_candidates(self, frontiers, robot_x, robot_y, reserved, avoid=()):
        """(cost, frontier) for every frontier this robot may be given."""

        candidates = []

        for frontier in frontiers:

            if self.is_failed(frontier):
                continue

            already_reserved = False

            for other_goal in reserved:

                if (
                    self.distance(
                        frontier[0],
                        frontier[1],
                        other_goal[0],
                        other_goal[1]
                    )
                    < self.robot_goal_separation
                ):

                    already_reserved = True
                    break

            if already_reserved:
                continue

            distance = self.distance(
                robot_x,
                robot_y,
                frontier[0],
                frontier[1]
            )

            if (
                distance
                >= self.minimum_frontier_distance
            ):

                candidates.append(
                    (
                        distance + self.spread_penalty(frontier, avoid),
                        frontier
                    )
                )

        return candidates

    def select_goal_pair(self, frontiers, reserved):
        """Goals for both robots at once, or None if no valid pair exists.

        Picking robot1's best goal first and then robot2's left robot2 with
        whatever was next to robot1's goal. Here every pair is scored
        together: both travel distances plus the spread penalty between the
        two goals, and the two goals must be robot_goal_separation apart.
        """
        c1 = self.frontier_candidates(frontiers, self.robot1_x, self.robot1_y, reserved)
        c2 = self.frontier_candidates(frontiers, self.robot2_x, self.robot2_y, reserved)

        best = None
        for d1, f1 in c1:
            for d2, f2 in c2:
                apart = self.distance(f1[0], f1[1], f2[0], f2[1])
                if apart < self.robot_goal_separation:
                    continue
                cost = d1 + d2 + self.spread_weight * max(0.0, self.spread_radius - apart)
                if best is None or cost < best[0]:
                    best = (cost, f1, f2)

        return None if best is None else (best[1], best[2])

#Check from the neighboring cells if the frontier is free and return the snapped point in world coordinates. If no free cell is found within a certain radius, return None.

    def snap_to_free(
        self,
        point
    ):

        info = self.map_msg.info

        resolution = (
            info.resolution
        )

        origin_x = (
            info.origin.position.x
        )

        origin_y = (
            info.origin.position.y
        )

        width = info.width
        height = info.height

        data = self.map_msg.data

        gx = int(
            (point[0] - origin_x)
            / resolution
        )

        gy = int(
            (point[1] - origin_y)
            / resolution
        )

        # Same inflation disc check as astar_planner's is_free, so we never
        # hand it a goal it's just going to reject.
        inflation_cells = max(
            1,
            int(math.ceil(self.obstacle_inflation_radius / resolution))
        )

        def is_free(x, y):

            if (
                x < 0
                or x >= width
                or y < 0
                or y >= height
            ):

                return False

            if data[y * width + x] != 0:
                return False

            for dx in range(-inflation_cells, inflation_cells + 1):

                for dy in range(-inflation_cells, inflation_cells + 1):

                    if math.sqrt(dx * dx + dy * dy) > inflation_cells:
                        continue

                    nx, ny = x + dx, y + dy

                    if (
                        nx < 0
                        or nx >= width
                        or ny < 0
                        or ny >= height
                    ):
                        return False

                    value = data[ny * width + nx]

                    if value > 50 or (value == -1 and self.inflate_unknown):
                        return False

            return True

        if is_free(gx, gy):

            return point

        for radius in range(1, 15):

            for dx in range(
                -radius,
                radius + 1
            ):

                for dy in range(
                    -radius,
                    radius + 1
                ):

                    nx = gx + dx
                    ny = gy + dy

                    if is_free(nx, ny):

                        world_x = (
                            origin_x
                            + (nx + 0.5)
                            * resolution
                        )

                        world_y = (
                            origin_y
                            + (ny + 0.5)
                            * resolution
                        )

                        return (
                            world_x,
                            world_y
                        )

        return None

    # REACHABLE FRONTIER GOALS (goal_selection == 'reachable')

    def safe_grid(self):
        """Cells astar_planner's is_free accepts, as a (height, width) bool array."""

        info = self.map_msg.info
        width, height = info.width, info.height

        grid = np.array(self.map_msg.data, dtype=np.int16).reshape(height, width)

        blocked = grid > 50

        if self.inflate_unknown:
            blocked |= grid == -1

        r = int(math.ceil(self.obstacle_inflation_radius / info.resolution))

        # Outside the map counts as blocked, like in is_free.
        padded = np.pad(blocked, r, constant_values=True)
        inflated = np.zeros_like(blocked)

        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if dx * dx + dy * dy <= r * r:
                    inflated |= padded[r + dy:r + dy + height, r + dx:r + dx + width]

        return (grid >= 0) & (grid <= 50) & ~inflated

    def reachable_frontier_goals(self, frontier_cells):

        info = self.map_msg.info
        width, height = info.width, info.height
        resolution = info.resolution
        origin_x = info.origin.position.x
        origin_y = info.origin.position.y

        safe = self.safe_grid()

        # Flood fill (8-connected, like A*) from each robot's start cell,
        # chosen the same way as astar_planner's find_nearest_free.
        reachable = np.zeros_like(safe)
        queue = deque()

        for rx, ry in (
            (self.robot1_x, self.robot1_y),
            (self.robot2_x, self.robot2_y),
        ):

            if rx is None:
                continue

            gx = int((rx - origin_x) / resolution)
            gy = int((ry - origin_y) / resolution)

            start = None

            for radius in range(0, 15):
                for dx in range(-radius, radius + 1):
                    for dy in range(-radius, radius + 1):
                        nx, ny = gx + dx, gy + dy
                        if 0 <= nx < width and 0 <= ny < height and safe[ny, nx]:
                            start = (nx, ny)
                            break
                    if start:
                        break
                if start:
                    break

            if start and not reachable[start[1], start[0]]:
                reachable[start[1], start[0]] = True
                queue.append(start)

        while queue:

            x, y = queue.popleft()

            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    nx, ny = x + dx, y + dy
                    if (
                        0 <= nx < width and 0 <= ny < height
                        and safe[ny, nx] and not reachable[ny, nx]
                    ):
                        reachable[ny, nx] = True
                        queue.append((nx, ny))

        # Reachable cells within 2 cells of a frontier cell.
        frontier = np.zeros_like(safe)
        for x, y in frontier_cells:
            frontier[y, x] = True

        near = np.zeros_like(safe)
        padded = np.pad(frontier, 2)
        for dy in range(-2, 3):
            for dx in range(-2, 3):
                near |= padded[2 + dy:2 + dy + height, 2 + dx:2 + dx + width]

        ys, xs = np.nonzero(reachable & near)

        # One goal per tile: the candidate nearest the tile's mean.
        tile = max(1, int(self.frontier_tile_size / resolution))
        tiles = {}

        for x, y in zip(xs.tolist(), ys.tolist()):
            tiles.setdefault((x // tile, y // tile), []).append((x, y))

        goals = []

        for cells in tiles.values():

            if len(cells) < 3:
                continue

            mx = sum(c[0] for c in cells) / len(cells)
            my = sum(c[1] for c in cells) / len(cells)
            x, y = min(cells, key=lambda c: (c[0] - mx) ** 2 + (c[1] - my) ** 2)

            goals.append((
                origin_x + (x + 0.5) * resolution,
                origin_y + (y + 0.5) * resolution,
            ))

        return goals

    # FIND FRONTIER CELLS

    def find_frontier_cells(self):

        width = (
            self.map_msg.info.width
        )

        height = (
            self.map_msg.info.height
        )

        data = self.map_msg.data

        frontier_cells = []

        for y in range(
            1,
            height - 1
        ):

            for x in range(
                1,
                width - 1
            ):

                index = (
                    y * width + x
                )

                # Must be FREE.

                if data[index] != 0:
                    continue

                neighbours = [
                    data[index - 1],
                    data[index + 1],
                    data[index - width],
                    data[index + width]
                ]

                # Free cell next to unknown cell.

                if -1 in neighbours:

                    frontier_cells.append(
                        (x, y)
                    )

        return frontier_cells

#Cluster the frontier cells based on their proximity to each other. Cells that are close together  are grouped into clusters. Each cluster represents a potential frontier area for exploration.

    def cluster_frontiers(
        self,
        cells
    ):

        clusters = []

        resolution = (
            self.map_msg.info.resolution
        )

        threshold_cells = max(
            1,
            int(
                self.cluster_distance
                / resolution
            )
        )

        unused = set(cells)

        while unused:

            seed = unused.pop()

            cluster = [
                seed
            ]

            queue = [
                seed
            ]

            while queue:

                current = queue.pop()

                cx, cy = current

                for dx in range(
                    -threshold_cells,
                    threshold_cells + 1
                ):

                    for dy in range(
                        -threshold_cells,
                        threshold_cells + 1
                    ):

                        if (
                            dx == 0
                            and dy == 0
                        ):

                            continue

                        neighbour = (
                            cx + dx,
                            cy + dy
                        )

                        if (
                            neighbour
                            in unused
                        ):

                            unused.remove(
                                neighbour
                            )

                            cluster.append(
                                neighbour
                            )

                            queue.append(
                                neighbour
                            )

            if len(cluster) >= 3:

                clusters.append(
                    cluster
                )

        return clusters

#Get the center of a cluster of frontier cells by calculating the average position of all cells in the cluster. The center is then converted from grid coordinates to world coordinates based on the map's resolution and origin.

    def cluster_center(
        self,
        cluster
    ):

        if not cluster:

            return None

        avg_x = (
            sum(
                cell[0]
                for cell in cluster
            )
            / len(cluster)
        )

        avg_y = (
            sum(
                cell[1]
                for cell in cluster
            )
            / len(cluster)
        )

        resolution = (
            self.map_msg.info.resolution
        )

        origin_x = (
            self.map_msg.info.origin.position.x
        )

        origin_y = (
            self.map_msg.info.origin.position.y
        )

        world_x = (
            origin_x
            + (avg_x + 0.5)
            * resolution
        )

        world_y = (
            origin_y
            + (avg_y + 0.5)
            * resolution
        )

        return (
            world_x,
            world_y
        )

    # Fraction of map cells that are known (occupied or free) rather than
    # unknown (-1). Used as the exploration-complete stop condition instead
    # of "zero frontier cells left", which live SLAM re-optimization makes
    # too strict to ever reliably hit exactly.
    def known_map_ratio(self):

        data = self.map_msg.data

        total = len(data)

        if total == 0:
            return 0.0

        known = sum(1 for cell in data if cell != -1)

        return known / total

#Failed frontiers are those that have been attempted multiple times without success. This function checks if a given frontier has failed based on the number of attempts recorded in the failed_frontiers dictionary. If the number of attempts exceeds the maximum allowed retries, the frontier is considered failed.

    def is_failed(
        self,
        frontier
    ):

        x, y = frontier

        for (
            failed_x,
            failed_y
        ), attempts in self.failed_frontiers.items():

            distance = self.distance(
                x,
                y,
                failed_x,
                failed_y
            )

            if (
                distance < 0.30
                and
                attempts >=
                self.max_frontier_retries
            ):

                return True

        return False

    # DISTANCE

    def distance(
        self,
        x1,
        y1,
        x2,
        y2
    ):

        return math.sqrt(
            (x2 - x1) ** 2
            +
            (y2 - y1) ** 2
        )

    # PUBLISH ROBOT 1 GOAL

    def publish_robot1_goals(
        self
    ):

        msg = PoseArray()

        msg.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        msg.header.frame_id = 'map'

        for x, y in self.robot1_goals:

            pose = Pose()

            pose.position.x = x
            pose.position.y = y

            pose.orientation.w = 1.0

            msg.poses.append(
                pose
            )

        self.robot1_goal_pub.publish(
            msg
        )

    # PUBLISH ROBOT 2 GOAL

    def publish_robot2_goals(
        self
    ):

        msg = PoseArray()

        msg.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        msg.header.frame_id = 'map'

        for x, y in self.robot2_goals:

            pose = Pose()

            pose.position.x = x
            pose.position.y = y

            pose.orientation.w = 1.0

            msg.poses.append(
                pose
            )

        self.robot2_goal_pub.publish(
            msg
        )


def main(args=None):

    rclpy.init(
        args=args
    )

    node = FrontierExplorer()

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == '__main__':

    main()