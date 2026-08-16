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
            10
        )
        self.robot2_goal_pub = self.create_publisher(
            PoseArray,
            '/robot2/frontier_goals',
            10
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

        self.robot1_x = (
            msg.pose.pose.position.x
        )

        self.robot1_y = (
            msg.pose.pose.position.y
        )

    def robot2_odom_callback(self, msg):

        self.robot2_x = (
            msg.pose.pose.position.x
        )

        self.robot2_y = (
            msg.pose.pose.position.y
        )

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

        if self.map_msg is None:

            self.get_logger().info(
                'Waiting for /map...'
            )

            return

        if (
            self.robot1_x is None
            or self.robot2_x is None
        ):

            self.get_logger().info(
                'Waiting for both robot positions...'
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
#Cluster 

        clusters = self.cluster_frontiers(
            frontier_cells
        )


        frontier_points = []

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

            distance_robot1 = self.distance(
                self.robot1_x,
                self.robot1_y,
                x,
                y
            )

            distance_robot2 = self.distance(
                self.robot2_x,
                self.robot2_y,
                x,
                y
            )

            # A frontier is useful if it is sufficiently far from
            # at least the robot that will use it.
            if (
                distance_robot1 >=
                self.minimum_frontier_distance
                or
                distance_robot2 >=
                self.minimum_frontier_distance
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
#Assign robot 1 frontiers 

        if not self.robot1_goals:

            robot1_frontier = (
                self.select_frontier_for_robot(
                    useful_frontiers,
                    self.robot1_x,
                    self.robot1_y,
                    reserved
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
        if not self.robot2_goals:

            robot2_frontier = (
                self.select_frontier_for_robot(
                    useful_frontiers,
                    self.robot2_x,
                    self.robot2_y,
                    reserved
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
        reserved
    ):

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
                        distance,
                        frontier
                    )
                )

        if not candidates:

            return None

        # Closest valid frontier first.

        candidates.sort(
            key=lambda item: item[0]
        )

        return candidates[0][1]
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

        def is_free(x, y):

            if (
                x < 0
                or x >= width
                or y < 0
                or y >= height
            ):

                return False

            return (
                data[y * width + x]
                == 0
            )

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