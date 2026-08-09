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

        # ============================================================
        # PARAMETERS
        # ============================================================

        self.declare_parameter(
            'robot_name',
            'robot1'
        )

        self.declare_parameter(
            'inflation_cells',
            3
        )

        self.declare_parameter(
            'occupied_threshold',
            50
        )

        self.robot_name = self.get_parameter(
            'robot_name'
        ).value

        self.inflation_cells = self.get_parameter(
            'inflation_cells'
        ).value

        self.occ_thresh = self.get_parameter(
            'occupied_threshold'
        ).value

        # ============================================================
        # FRAMES
        # ============================================================

        self.map_frame = "map"
        self.base_frame = (
            f"{self.robot_name}/base_link"
        )

        # ============================================================
        # STATE
        # ============================================================

        self.map = None
        self.last_goal = None

        # ============================================================
        # TF
        # ============================================================

        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        # ============================================================
        # SUBSCRIBERS
        # ============================================================

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            "/map",
            self.map_callback,
            1
        )

        self.goal_sub = self.create_subscription(
            PoseStamped,
            f"/{self.robot_name}/frontier_goal",
            self.goal_callback,
            10
        )

        # ============================================================
        # PUBLISHER
        # ============================================================

        self.path_pub = self.create_publisher(
            Path,
            f"/{self.robot_name}/planned_path",
            10
        )

        self.get_logger().info(
            "A* Cost-Gradient Planner ready."
        )

    # ================================================================
    # MAP CALLBACK
    # ================================================================

    def map_callback(self, msg):

        self.map = msg

    # ================================================================
    # GET ROBOT POSE
    # ================================================================

    def get_robot_pose(self):

        try:

            tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time()
            )

            return (
                tf.transform.translation.x,
                tf.transform.translation.y
            )

        except Exception:

            return None

    # ================================================================
    # WORLD -> GRID
    # ================================================================

    def world_to_grid(self, x, y):

        if self.map is None:
            return None

        info = self.map.info

        gx = int(
            math.floor(
                (x - info.origin.position.x)
                / info.resolution
            )
        )

        gy = int(
            math.floor(
                (y - info.origin.position.y)
                / info.resolution
            )
        )

        return gx, gy

    # ================================================================
    # GRID -> WORLD
    # ================================================================

    def grid_to_world(self, gx, gy):

        info = self.map.info

        x = (
            (gx + 0.5)
            * info.resolution
            + info.origin.position.x
        )

        y = (
            (gy + 0.5)
            * info.resolution
            + info.origin.position.y
        )

        return x, y

    # ================================================================
    # BUILD C-SPACE
    # ================================================================

    def build_cspace_and_costs(self, grid):

        h, w = grid.shape

        # ------------------------------------------------------------
        # REAL OBSTACLES
        # ------------------------------------------------------------

        obstacles = (
            grid >= self.occ_thresh
        )

        # ------------------------------------------------------------
        # UNKNOWN CELLS
        # ------------------------------------------------------------

        unknown = (
            grid == -1
        )

        # ------------------------------------------------------------
        # C-SPACE
        #
        # Unknown is NOT directly added to cspace here.
        # We want the robot to remain in known free space,
        # but we don't want unknown space to destroy connectivity
        # of the explored region.
        # ------------------------------------------------------------

        cspace = obstacles.copy()

        # ------------------------------------------------------------
        # COST GRADIENT
        #
        # Obstacles get a strong cost nearby.
        # Unknown gets a smaller penalty.
        # ------------------------------------------------------------

        cost_gradient = np.zeros(
            (h, w),
            dtype=np.float32
        )

        k = self.inflation_cells

        for dy in range(-k, k + 1):

            for dx in range(-k, k + 1):

                if dx == 0 and dy == 0:
                    continue

                distance = math.hypot(
                    dx,
                    dy
                )

                if distance == 0:
                    continue

                y_start = max(
                    0,
                    dy
                )

                y_end = min(
                    h,
                    h + dy
                )

                x_start = max(
                    0,
                    dx
                )

                x_end = min(
                    w,
                    w + dx
                )

                oy_start = max(
                    0,
                    -dy
                )

                oy_end = min(
                    h,
                    h - dy
                )

                ox_start = max(
                    0,
                    -dx
                )

                ox_end = min(
                    w,
                    w - dx
                )

                # ------------------------------------------------
                # Inflate REAL obstacles
                # ------------------------------------------------

                cspace[
                    y_start:y_end,
                    x_start:x_end
                ] |= obstacles[
                    oy_start:oy_end,
                    ox_start:ox_end
                ]

                # ------------------------------------------------
                # Obstacle cost
                # ------------------------------------------------

                obstacle_weight = (
                    12.0
                    / (distance + 0.5)
                )

                cost_gradient[
                    y_start:y_end,
                    x_start:x_end
                ] += (
                    obstacles[
                        oy_start:oy_end,
                        ox_start:ox_end
                    ]
                    * obstacle_weight
                )

                # ------------------------------------------------
                # Unknown cost
                #
                # Penalize unknown but don't make it an
                # impenetrable wall in the cost calculation.
                # ------------------------------------------------

                unknown_weight = (
                    2.0
                    / (distance + 0.5)
                )

                cost_gradient[
                    y_start:y_end,
                    x_start:x_end
                ] += (
                    unknown[
                        oy_start:oy_end,
                        ox_start:ox_end
                    ]
                    * unknown_weight
                )

        return cspace, cost_gradient

    # ================================================================
    # CHECK WALKABLE
    # ================================================================

    def is_walkable(
        self,
        cspace,
        x,
        y
    ):

        h, w = cspace.shape

        return (
            0 <= x < w
            and
            0 <= y < h
            and
            not cspace[y, x]
        )

    # ================================================================
    # FIND NEAREST FREE CELL
    # ================================================================

    def find_nearest_free(
        self,
        cell,
        cspace
    ):

        if cell is None:
            return None

        x, y = cell

        h, w = cspace.shape

        # ------------------------------------------------------------
        # Check bounds
        # ------------------------------------------------------------

        if not (
            0 <= x < w
            and
            0 <= y < h
        ):
            return None

        # ------------------------------------------------------------
        # Already free
        # ------------------------------------------------------------

        if self.is_walkable(
            cspace,
            x,
            y
        ):
            return x, y

        # ------------------------------------------------------------
        # Search nearby free cell
        # ------------------------------------------------------------

        queue = deque()

        queue.append(
            (x, y)
        )

        visited = set()

        visited.add(
            (x, y)
        )

        neighbours = [
            (-1, 0),
            (1, 0),
            (0, -1),
            (0, 1),
            (-1, -1),
            (-1, 1),
            (1, -1),
            (1, 1)
        ]

        while queue:

            cx, cy = queue.popleft()

            if self.is_walkable(
                cspace,
                cx,
                cy
            ):
                return cx, cy

            for dx, dy in neighbours:

                nx = cx + dx
                ny = cy + dy

                if not (
                    0 <= nx < w
                    and
                    0 <= ny < h
                ):
                    continue

                if (
                    nx,
                    ny
                ) in visited:
                    continue

                visited.add(
                    (nx, ny)
                )

                queue.append(
                    (nx, ny)
                )

        return None

    # ================================================================
    # HEURISTIC
    # ================================================================

    def heuristic(
        self,
        a,
        b
    ):

        return math.hypot(
            a[0] - b[0],
            a[1] - b[1]
        )

    # ================================================================
    # A* SEARCH
    # ================================================================

    def a_star(
        self,
        start,
        goal,
        cspace,
        cost_gradient
    ):

        neighbours = [
            (-1, 0),
            (1, 0),
            (0, -1),
            (0, 1),
            (-1, -1),
            (-1, 1),
            (1, -1),
            (1, 1)
        ]

        open_set = []

        heapq.heappush(
            open_set,
            (
                0.0,
                start
            )
        )

        came_from = {}

        g_score = {
            start: 0.0
        }

        closed = set()

        while open_set:

            _, current = heapq.heappop(
                open_set
            )

            if current in closed:
                continue

            closed.add(
                current
            )

            # --------------------------------------------------------
            # Goal reached
            # --------------------------------------------------------

            if current == goal:

                path = []

                while current in came_from:

                    path.append(
                        current
                    )

                    current = came_from[
                        current
                    ]

                path.append(
                    start
                )

                path.reverse()

                return path

            # --------------------------------------------------------
            # Explore neighbours
            # --------------------------------------------------------

            for dx, dy in neighbours:

                nx = current[0] + dx
                ny = current[1] + dy

                if not self.is_walkable(
                    cspace,
                    nx,
                    ny
                ):
                    continue

                # ----------------------------------------------------
                # Prevent diagonal corner cutting
                # ----------------------------------------------------

                if dx != 0 and dy != 0:

                    if not self.is_walkable(
                        cspace,
                        current[0] + dx,
                        current[1]
                    ):
                        continue

                    if not self.is_walkable(
                        cspace,
                        current[0],
                        current[1] + dy
                    ):
                        continue

                neighbour = (
                    nx,
                    ny
                )

                movement_cost = (
                    1.414
                    if dx != 0 and dy != 0
                    else 1.0
                )

                new_cost = (
                    g_score[current]
                    +
                    movement_cost
                    +
                    float(
                        cost_gradient[
                            ny,
                            nx
                        ]
                    )
                )

                if (
                    neighbour not in g_score
                    or
                    new_cost < g_score[
                        neighbour
                    ]
                ):

                    g_score[
                        neighbour
                    ] = new_cost

                    priority = (
                        new_cost
                        +
                        self.heuristic(
                            neighbour,
                            goal
                        )
                    )

                    heapq.heappush(
                        open_set,
                        (
                            priority,
                            neighbour
                        )
                    )

                    came_from[
                        neighbour
                    ] = current

        return None

    # ================================================================
    # GOAL CALLBACK
    # ================================================================

    def goal_callback(
        self,
        msg
    ):

        if self.map is None:

            self.get_logger().warn(
                "A* waiting for map..."
            )

            return

        # ------------------------------------------------------------
        # Robot pose
        # ------------------------------------------------------------

        pose = self.get_robot_pose()

        if pose is None:

            self.get_logger().warn(
                "A* waiting for TF pose..."
            )

            return

        # ------------------------------------------------------------
        # Goal
        # ------------------------------------------------------------

        goal_xy = (
            msg.pose.position.x,
            msg.pose.position.y
        )

        # ------------------------------------------------------------
        # Ignore exact duplicate goal
        # ------------------------------------------------------------

        if self.last_goal == goal_xy:
            return

        self.last_goal = goal_xy

        # ------------------------------------------------------------
        # Convert map
        # ------------------------------------------------------------

        info = self.map.info

        grid = np.array(
            self.map.data,
            dtype=np.int16
        ).reshape(
            info.height,
            info.width
        )

        # ------------------------------------------------------------
        # Build C-space
        # ------------------------------------------------------------

        cspace, cost_gradient = (
            self.build_cspace_and_costs(
                grid
            )
        )

        # ------------------------------------------------------------
        # Convert start and goal
        # ------------------------------------------------------------

        raw_start = self.world_to_grid(
            pose[0],
            pose[1]
        )

        raw_goal = self.world_to_grid(
            goal_xy[0],
            goal_xy[1]
        )

        # ------------------------------------------------------------
        # Debug occupancy information
        # ------------------------------------------------------------

        self.get_logger().info(
            f"Raw start: {raw_start}, "
            f"Raw goal: {raw_goal}"
        )

        if raw_start is None or raw_goal is None:

            self.get_logger().warn(
                "Start or goal outside map."
            )

            return

        sx, sy = raw_start
        gx, gy = raw_goal

        h, w = grid.shape

        if not (
            0 <= sx < w
            and
            0 <= sy < h
            and
            0 <= gx < w
            and
            0 <= gy < h
        ):

            self.get_logger().warn(
                "Start or goal outside grid bounds."
            )

            return

        self.get_logger().info(
            f"Raw occupancy: "
            f"start={grid[sy, sx]}, "
            f"goal={grid[gy, gx]}"
        )

        # ------------------------------------------------------------
        # Find usable cells
        # ------------------------------------------------------------

        start = self.find_nearest_free(
            raw_start,
            cspace
        )

        goal = self.find_nearest_free(
            raw_goal,
            cspace
        )

        if start is None:

            self.get_logger().error(
                "Could not find a free start cell."
            )

            return

        if goal is None:

            self.get_logger().error(
                "Could not find a free goal cell."
            )

            return

        self.get_logger().info(
            f"Resolved start: {start}"
        )

        self.get_logger().info(
            f"Resolved goal: {goal}"
        )

        # ------------------------------------------------------------
        # C-space values
        # ------------------------------------------------------------

        self.get_logger().info(
            f"C-space: "
            f"start_free={not cspace[start[1], start[0]]}, "
            f"goal_free={not cspace[goal[1], goal[0]]}"
        )

        # ============================================================
        # RUN A*
        # ============================================================

        self.get_logger().info(
            f"Planning Path: "
            f"Start {start} -> Goal {goal}"
        )

        path = self.a_star(
            start,
            goal,
            cspace,
            cost_gradient
        )

        # ------------------------------------------------------------
        # No path
        # ------------------------------------------------------------

        if path is None:

            self.get_logger().error(
                "A* could not find a path."
            )

            return

        # ============================================================
        # PATH SMOOTHING
        # ============================================================

        smoothed_path = []

        if len(path) > 2:

            smoothed_path.append(
                path[0]
            )

            for i in range(
                1,
                len(path) - 1
            ):

                dx1 = (
                    path[i][0]
                    -
                    path[i - 1][0]
                )

                dy1 = (
                    path[i][1]
                    -
                    path[i - 1][1]
                )

                dx2 = (
                    path[i + 1][0]
                    -
                    path[i][0]
                )

                dy2 = (
                    path[i + 1][1]
                    -
                    path[i][1]
                )

                if (
                    dx1 == dx2
                    and
                    dy1 == dy2
                ):
                    continue

                smoothed_path.append(
                    path[i]
                )

            smoothed_path.append(
                path[-1]
            )

        else:

            smoothed_path = path

        # ============================================================
        # CREATE ROS PATH
        # ============================================================

        path_msg = Path()

        path_msg.header.frame_id = (
            self.map_frame
        )

        path_msg.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        for gx, gy in smoothed_path:

            x, y = self.grid_to_world(
                gx,
                gy
            )

            p = PoseStamped()

            p.header.frame_id = (
                self.map_frame
            )

            p.header.stamp = (
                path_msg.header.stamp
            )

            p.pose.position.x = x
            p.pose.position.y = y

            p.pose.orientation.w = 1.0

            path_msg.poses.append(
                p
            )

        # ============================================================
        # PUBLISH PATH
        # ============================================================

        self.path_pub.publish(
            path_msg
        )

        self.get_logger().info(
            f"A* path generated successfully. "
            f"{len(path)} cells, "
            f"{len(smoothed_path)} after smoothing."
        )


# ====================================================================
# MAIN
# ====================================================================

def main(args=None):

    rclpy.init(args=args)

    node = AStarPlanner()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    node.destroy_node()

    if rclpy.ok():

        rclpy.shutdown()


if __name__ == '__main__':

    main()