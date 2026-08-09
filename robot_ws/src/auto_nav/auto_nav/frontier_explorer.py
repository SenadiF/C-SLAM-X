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

        # ============================================================
        # PARAMETERS
        # ============================================================

        self.declare_parameter('robot_name', 'robot1')
        self.declare_parameter('min_frontier_size', 8)
        self.declare_parameter('revisit_radius', 0.25)
        self.declare_parameter('poll_period', 2.0)
        self.declare_parameter('goal_timeout', 15.0)
        self.declare_parameter('stuck_distance', 0.03)

        self.robot_name = self.get_parameter(
            'robot_name'
        ).value

        self.min_frontier_size = self.get_parameter(
            'min_frontier_size'
        ).value

        self.revisit_radius = self.get_parameter(
            'revisit_radius'
        ).value

        poll_period = self.get_parameter(
            'poll_period'
        ).value

        self.goal_timeout = self.get_parameter(
            'goal_timeout'
        ).value

        self.stuck_distance = self.get_parameter(
            'stuck_distance'
        ).value

        # ============================================================
        # FRAMES
        # ============================================================

        self.map_frame = "map"
        self.base_frame = f"{self.robot_name}/base_link"

        # ============================================================
        # TF
        # ============================================================

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        # ============================================================
        # STATE
        # ============================================================

        self.map = None

        # Successfully visited/blacklisted frontier positions
        self.visited = []

        # Currently active frontier goal
        self.current_goal = None

        # Goal tracking
        self.goal_start_time = None
        self.last_robot_pose = None
        self.stuck_counter = 0

        # ============================================================
        # FAILED GOALS
        # ============================================================

        self.failed_attempts = {}

        self.max_retries = 3

        # ============================================================
        # SUBSCRIBERS
        # ============================================================

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            1
        )

        # ============================================================
        # PUBLISHERS
        # ============================================================

        self.goal_pub = self.create_publisher(
            PoseStamped,
            f'/{self.robot_name}/frontier_goal',
            10
        )

        self.done_pub = self.create_publisher(
            Bool,
            f'/{self.robot_name}/exploration_done',
            10
        )

        # ============================================================
        # TIMER
        # ============================================================

        self.create_timer(
            poll_period,
            self.explore
        )

        self.get_logger().info(
            "Frontier explorer ready."
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

            x = tf.transform.translation.x
            y = tf.transform.translation.y

            return x, y

        except Exception:

            return None

    # ================================================================
    # WORLD -> GRID
    # ================================================================

    def world_to_grid(self, x, y):

        if self.map is None:
            return 0, 0

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
    # FIND FRONTIERS
    # ================================================================

    def find_frontiers(self):

        if self.map is None:
            return []

        info = self.map.info

        width = info.width
        height = info.height

        grid = np.array(
            self.map.data,
            dtype=np.int16
        ).reshape(height, width)

        # ------------------------------------------------------------
        # Occupancy states
        # ------------------------------------------------------------

        free = grid == 0
        unknown = grid == -1

        # ------------------------------------------------------------
        # Find unknown cells next to free cells
        # ------------------------------------------------------------

        unknown_adjacent = np.zeros_like(
            unknown
        )

        unknown_adjacent[:-1, :] |= unknown[1:, :]
        unknown_adjacent[1:, :] |= unknown[:-1, :]
        unknown_adjacent[:, :-1] |= unknown[:, 1:]
        unknown_adjacent[:, 1:] |= unknown[:, :-1]

        # Frontier = FREE cell adjacent to UNKNOWN
        frontier_mask = (
            free
            & unknown_adjacent
        )

        if not frontier_mask.any():
            return []

        # ------------------------------------------------------------
        # Cluster frontiers
        # ------------------------------------------------------------

        visited_cells = np.zeros_like(
            frontier_mask
        )

        frontiers = []

        cells = np.argwhere(
            frontier_mask
        )

        for sy, sx in cells:

            if visited_cells[sy, sx]:
                continue

            queue = deque(
                [(int(sy), int(sx))]
            )

            visited_cells[sy, sx] = True

            cluster = []

            while queue:

                y, x = queue.popleft()

                cluster.append(
                    (y, x)
                )

                neighbours = [
                    (y - 1, x),
                    (y + 1, x),
                    (y, x - 1),
                    (y, x + 1)
                ]

                for ny, nx in neighbours:

                    if (
                        0 <= ny < height
                        and
                        0 <= nx < width
                        and
                        frontier_mask[ny, nx]
                        and
                        not visited_cells[ny, nx]
                    ):

                        visited_cells[
                            ny, nx
                        ] = True

                        queue.append(
                            (ny, nx)
                        )

            # --------------------------------------------------------
            # Ignore very small frontier clusters
            # --------------------------------------------------------

            if len(cluster) < self.min_frontier_size:
                continue

            # --------------------------------------------------------
            # Calculate cluster center
            # --------------------------------------------------------

            arr = np.array(
                cluster
            )

            cy, cx = arr.mean(
                axis=0
            )

            world_x, world_y = (
                self.grid_to_world(
                    int(cx),
                    int(cy)
                )
            )

            frontiers.append(
                (world_x, world_y)
            )

        return frontiers

    # ================================================================
    # CHECK WHETHER FRONTIER WAS VISITED
    # ================================================================

    def already_visited(self, x, y):

        for vx, vy in self.visited:

            distance = math.hypot(
                x - vx,
                y - vy
            )

            if distance < self.revisit_radius:
                return True

        return False

    # ================================================================
    # CHECK WHETHER CURRENT GOAL WAS REACHED
    # ================================================================

    def goal_reached(self, x, y):

        if self.current_goal is None:
            return False

        gx, gy = self.current_goal

        distance = math.hypot(
            gx - x,
            gy - y
        )

        return distance < 0.25

    # ================================================================
    # CHECK FOR TIMEOUT / STUCK ROBOT
    # ================================================================

    def goal_timeout_check(self, rx, ry):

        if self.current_goal is None:
            return False

        # ------------------------------------------------------------
        # First check
        # ------------------------------------------------------------

        if self.goal_start_time is None:

            self.goal_start_time = time.time()

            self.last_robot_pose = (
                rx,
                ry
            )

            return False

        # ------------------------------------------------------------
        # Calculate robot movement
        # ------------------------------------------------------------

        moved = math.hypot(
            rx - self.last_robot_pose[0],
            ry - self.last_robot_pose[1]
        )

        self.last_robot_pose = (
            rx,
            ry
        )

        # ------------------------------------------------------------
        # Stuck detection
        # ------------------------------------------------------------

        if moved < self.stuck_distance:

            self.stuck_counter += 1

        else:

            self.stuck_counter = 0

        # ------------------------------------------------------------
        # Time elapsed
        # ------------------------------------------------------------

        elapsed = (
            time.time()
            - self.goal_start_time
        )

        # ------------------------------------------------------------
        # Timeout
        # ------------------------------------------------------------

        if elapsed > self.goal_timeout:

            self.get_logger().warn(
                "Goal timeout reached."
            )

            return True

        # ------------------------------------------------------------
        # Physically stuck
        # ------------------------------------------------------------

        if self.stuck_counter > 4:

            self.get_logger().warn(
                "Robot appears to be physically stuck."
            )

            return True

        return False

    # ================================================================
    # HANDLE FAILED GOAL
    # ================================================================

    def handle_failed_goal(self):

        if self.current_goal is None:
            return

        # Round position so tiny coordinate differences
        # are treated as the same frontier.
        key = (
            round(self.current_goal[0], 1),
            round(self.current_goal[1], 1)
        )

        self.failed_attempts[key] = (
            self.failed_attempts.get(key, 0)
            + 1
        )

        attempts = self.failed_attempts[key]

        # ------------------------------------------------------------
        # Maximum retries reached
        # ------------------------------------------------------------

        if attempts >= self.max_retries:

            self.get_logger().warn(
                f"Frontier failed {attempts} times. "
                f"Blacklisting target."
            )

            self.visited.append(
                self.current_goal
            )

        # ------------------------------------------------------------
        # Retry later
        # ------------------------------------------------------------

        else:

            self.get_logger().warn(
                f"Frontier failed "
                f"({attempts}/{self.max_retries}). "
                f"Will retry later."
            )

    # ================================================================
    # RESET GOAL STATE
    # ================================================================

    def reset_goal_states(self):

        self.current_goal = None

        self.goal_start_time = None

        self.last_robot_pose = None

        self.stuck_counter = 0

    # ================================================================
    # MAIN EXPLORATION LOOP
    # ================================================================

    def explore(self):

        # ------------------------------------------------------------
        # Need map
        # ------------------------------------------------------------

        if self.map is None:
            return

        # ------------------------------------------------------------
        # Need robot pose
        # ------------------------------------------------------------

        pose = self.get_robot_pose()

        if pose is None:
            return

        rx, ry = pose

        # ------------------------------------------------------------
        # If we already have a goal, monitor it
        # ------------------------------------------------------------

        if self.current_goal is not None:

            # --------------------------------------------------------
            # Goal successfully reached
            # --------------------------------------------------------

            if self.goal_reached(
                rx,
                ry
            ):

                self.get_logger().info(
                    "Goal reached successfully."
                )

                # Mark this frontier as visited
                self.visited.append(
                    self.current_goal
                )

                self.reset_goal_states()

            # --------------------------------------------------------
            # Goal failed / timeout / stuck
            # --------------------------------------------------------

            elif self.goal_timeout_check(
                rx,
                ry
            ):

                self.get_logger().warn(
                    "Current frontier failed."
                )

                self.handle_failed_goal()

                self.reset_goal_states()

            # --------------------------------------------------------
            # Still travelling
            # --------------------------------------------------------

            else:

                return

        # ============================================================
        # FIND NEW FRONTIERS
        # ============================================================

        frontiers = self.find_frontiers()

        if not frontiers:

            self.get_logger().info(
                "No frontiers detected. "
                "Exploration complete."
            )

            done_msg = Bool()
            done_msg.data = True

            self.done_pub.publish(
                done_msg
            )

            return

        # ------------------------------------------------------------
        # Remove already visited / blacklisted frontiers
        # ------------------------------------------------------------

        valid_frontiers = [

            frontier

            for frontier in frontiers

            if not self.already_visited(
                frontier[0],
                frontier[1]
            )

        ]

        # ------------------------------------------------------------
        # No valid frontier remaining
        # ------------------------------------------------------------

        if not valid_frontiers:

            self.get_logger().info(
                "All frontiers already visited "
                "or blacklisted."
            )

            done_msg = Bool()
            done_msg.data = True

            self.done_pub.publish(
                done_msg
            )

            return

        # ============================================================
        # SELECT NEAREST FRONTIER
        # ============================================================

        valid_frontiers.sort(
            key=lambda f:
                math.hypot(
                    f[0] - rx,
                    f[1] - ry
                )
        )

        best_frontier = (
            valid_frontiers[0]
        )

        # ============================================================
        # SET CURRENT GOAL
        # ============================================================

        self.current_goal = (
            best_frontier[0],
            best_frontier[1]
        )

        self.goal_start_time = time.time()

        self.last_robot_pose = (
            rx,
            ry
        )

        self.stuck_counter = 0

        self.get_logger().info(
            f"Dispatched target frontier: "
            f"({best_frontier[0]:.2f}, "
            f"{best_frontier[1]:.2f})"
        )

        # ============================================================
        # PUBLISH GOAL
        # ============================================================

        goal_msg = PoseStamped()

        goal_msg.header.frame_id = (
            self.map_frame
        )

        goal_msg.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        goal_msg.pose.position.x = (
            best_frontier[0]
        )

        goal_msg.pose.position.y = (
            best_frontier[1]
        )

        goal_msg.pose.position.z = 0.0

        # No specific orientation required
        goal_msg.pose.orientation.x = 0.0
        goal_msg.pose.orientation.y = 0.0
        goal_msg.pose.orientation.z = 0.0
        goal_msg.pose.orientation.w = 1.0

        self.goal_pub.publish(
            goal_msg
        )


# ====================================================================
# MAIN
# ====================================================================

def main(args=None):

    rclpy.init(args=args)

    node = FrontierExplorer()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()