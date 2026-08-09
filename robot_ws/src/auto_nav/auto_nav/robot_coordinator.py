import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry

import math
import numpy as np


class RobotCoordinator(Node):

    def __init__(self):
        super().__init__('robot_coordinator')

        # ============================================================
        # PARAMETERS
        # ============================================================

        self.declare_parameter('robot_name', 'robot1')
        self.declare_parameter('frontier_min_size', 3)
        self.declare_parameter('frontier_cluster_distance', 0.5)
        self.declare_parameter('goal_reached_distance', 0.20)
        self.declare_parameter('goal_timeout', 30.0)

        self.robot_name = self.get_parameter(
            'robot_name'
        ).value

        self.frontier_min_size = self.get_parameter(
            'frontier_min_size'
        ).value

        self.cluster_distance = self.get_parameter(
            'frontier_cluster_distance'
        ).value

        self.goal_reached_distance = self.get_parameter(
            'goal_reached_distance'
        ).value

        self.goal_timeout = self.get_parameter(
            'goal_timeout'
        ).value

        # ============================================================
        # PUBLISHER
        # ============================================================

        self.goal_pub = self.create_publisher(
            PoseStamped,
            f'/{self.robot_name}/goal_pose',
            10
        )

        # ============================================================
        # SUBSCRIBERS
        # ============================================================

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            f'/{self.robot_name}/odometry/filtered',
            self.odom_callback,
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            f'/{self.robot_name}/scan',
            self.scan_callback,
            10
        )

        # ============================================================
        # STATE
        # ============================================================

        self.current_map = None
        self.current_pose = None
        self.current_scan = None

        self.frontiers = []

        self.current_goal = None
        self.goal_time = None

        # IMPORTANT:
        # prevents sending a new goal every 2 seconds
        self.goal_active = False

        # ============================================================
        # TIMER
        # ============================================================

        self.timer = self.create_timer(
            1.0,
            self.coordinate
        )

        self.get_logger().info(
            'Robot Coordinator ready.'
        )

    # ================================================================
    # CALLBACKS
    # ================================================================

    def map_callback(self, msg):

        self.current_map = msg

        self.frontiers = self.find_frontiers()

    def odom_callback(self, msg):

        self.current_pose = msg.pose.pose

    def scan_callback(self, msg):

        self.current_scan = msg

    # ================================================================
    # FRONTIER DETECTION
    # ================================================================

    def find_frontiers(self):

        if self.current_map is None:
            return []

        width = self.current_map.info.width
        height = self.current_map.info.height

        resolution = self.current_map.info.resolution

        origin_x = self.current_map.info.origin.position.x
        origin_y = self.current_map.info.origin.position.y

        data = np.array(
            self.current_map.data
        ).reshape(
            (height, width)
        )

        frontier_cells = []

        # ------------------------------------------------------------
        # A frontier cell is:
        #
        # UNKNOWN (-1)
        # next to FREE (0)
        # ------------------------------------------------------------

        for y in range(1, height - 1):

            for x in range(1, width - 1):

                if data[y, x] != -1:
                    continue

                neighbourhood = data[
                    y - 1:y + 2,
                    x - 1:x + 2
                ]

                if np.any(neighbourhood == 0):

                    world_x = (
                        origin_x
                        +
                        (x + 0.5) * resolution
                    )

                    world_y = (
                        origin_y
                        +
                        (y + 0.5) * resolution
                    )

                    frontier_cells.append(
                        (world_x, world_y)
                    )

        # ------------------------------------------------------------
        # Cluster frontier cells
        # ------------------------------------------------------------

        return self.cluster_frontiers(
            frontier_cells
        )

    # ================================================================
    # CLUSTER FRONTIERS
    # ================================================================

    def cluster_frontiers(self, cells):

        if not cells:
            return []

        clusters = []

        for point in cells:

            assigned = False

            for cluster in clusters:

                cx = cluster['center'][0]
                cy = cluster['center'][1]

                distance = math.hypot(
                    point[0] - cx,
                    point[1] - cy
                )

                if distance < self.cluster_distance:

                    cluster['points'].append(
                        point
                    )

                    # Recalculate center
                    xs = [
                        p[0]
                        for p in cluster['points']
                    ]

                    ys = [
                        p[1]
                        for p in cluster['points']
                    ]

                    cluster['center'] = (
                        sum(xs) / len(xs),
                        sum(ys) / len(ys)
                    )

                    assigned = True
                    break

            if not assigned:

                clusters.append({
                    'center': point,
                    'points': [point]
                })

        # ------------------------------------------------------------
        # Remove tiny clusters
        # ------------------------------------------------------------

        result = []

        for cluster in clusters:

            if len(cluster['points']) >= self.frontier_min_size:

                result.append(
                    cluster['center']
                )

        return result

    # ================================================================
    # MAIN COORDINATION
    # ================================================================

    def coordinate(self):

        if self.current_map is None:
            return

        if self.current_pose is None:
            return

        # ------------------------------------------------------------
        # If currently travelling to a goal
        # ------------------------------------------------------------

        if self.goal_active:

            if self.goal_reached():

                self.get_logger().info(
                    'Frontier reached. Looking for next frontier.'
                )

                self.goal_active = False
                self.current_goal = None

                return

            # --------------------------------------------------------
            # Safety timeout
            # --------------------------------------------------------

            if self.goal_time is not None:

                elapsed = (
                    self.get_clock().now()
                    -
                    self.goal_time
                ).nanoseconds / 1e9

                if elapsed > self.goal_timeout:

                    self.get_logger().warn(
                        'Goal timeout. Choosing another frontier.'
                    )

                    self.goal_active = False
                    self.current_goal = None

            return

        # ------------------------------------------------------------
        # No active goal
        # ------------------------------------------------------------

        if len(self.frontiers) == 0:

            self.get_logger().info(
                'No frontiers detected.'
            )

            return

        # ------------------------------------------------------------
        # Choose nearest frontier
        # ------------------------------------------------------------

        robot_x = self.current_pose.position.x
        robot_y = self.current_pose.position.y

        valid_frontiers = []

        for frontier in self.frontiers:

            fx = frontier[0]
            fy = frontier[1]

            distance = math.hypot(
                fx - robot_x,
                fy - robot_y
            )

            # Don't select a frontier almost on top of robot
            if distance > 0.30:

                valid_frontiers.append(
                    (distance, frontier)
                )

        if not valid_frontiers:

            self.get_logger().info(
                'No useful frontiers available.'
            )

            return

        # nearest first
        valid_frontiers.sort(
            key=lambda item: item[0]
        )

        distance, goal = valid_frontiers[0]

        # ------------------------------------------------------------
        # Send goal
        # ------------------------------------------------------------

        self.send_goal(goal)

    # ================================================================
    # SEND GOAL
    # ================================================================

    def send_goal(self, goal):

        msg = PoseStamped()

        msg.header.stamp = (
            self.get_clock().now().to_msg()
        )

        msg.header.frame_id = 'map'

        msg.pose.position.x = goal[0]
        msg.pose.position.y = goal[1]
        msg.pose.position.z = 0.0

        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0

        self.goal_pub.publish(msg)

        self.current_goal = goal

        self.goal_active = True

        self.goal_time = (
            self.get_clock().now()
        )

        self.get_logger().info(
            f'NEW FRONTIER GOAL: '
            f'({goal[0]:.2f}, {goal[1]:.2f})'
        )

    # ================================================================
    # GOAL REACHED
    # ================================================================

    def goal_reached(self):

        if self.current_goal is None:
            return True

        rx = self.current_pose.position.x
        ry = self.current_pose.position.y

        gx = self.current_goal[0]
        gy = self.current_goal[1]

        distance = math.hypot(
            gx - rx,
            gy - ry
        )

        return (
            distance
            <
            self.goal_reached_distance
        )


def main(args=None):

    rclpy.init(args=args)

    node = RobotCoordinator()

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