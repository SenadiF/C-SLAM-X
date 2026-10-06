
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from my_navigation.robot_pose import MapFramePose

from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry

from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseStamped

from std_msgs.msg import Bool, Float32

import math
import heapq
import time


class AStarPlanner(Node):

    def __init__(self):

        super().__init__('astar_planner')
        self.pose_tracker = MapFramePose(self)

        self.map_msg = None


        #15 cm around obstacles r avoided 
        self.declare_parameter(
            'obstacle_inflation_radius',
            0.15
        )

        self.obstacle_inflation_radius = self.get_parameter(
            'obstacle_inflation_radius'
        ).value

        # True: unknown cells inside the inflation disc also make a cell
        # unsafe. On a real LiDAR map the unknown gaps between rays then
        # reject almost every cell, so on hardware set this False - only
        # occupied cells are inflated, and the cell itself must still be
        # known free. Must match frontier_explorer's inflate_unknown.
        self.declare_parameter('inflate_unknown', True)
        self.inflate_unknown = self.get_parameter('inflate_unknown').value

        # If a goal is unreachable (blocked/inflated), find_nearest_free can
        # snap both the robot's start cell AND the goal cell to the same
        # small isolated safe pocket near the robot's current position.
        # a_star then trivially reports start==goal as "success" right where
        # the robot already is, even though it's nowhere near the real goal.
        # Reject that as a failure unless the robot genuinely was already
        # this close to the requested goal.
        self.declare_parameter(
            'same_cell_snap_tolerance',
            0.3
        )

        self.same_cell_snap_tolerance = self.get_parameter(
            'same_cell_snap_tolerance'
        ).value
#Robot 1 currect position 

        self.robot1_x = None
        self.robot1_y = None
#Robot2 current position 
        self.robot2_x = None
        self.robot2_y = None


        self.robot1_goals = []       
        self.robot2_goals = []

        self.robot1_current_goal = None
        self.robot2_current_goal = None

        # Once a goal has been successfully planned for, don't keep
        # re-publishing a path for it every cycle - pure_pursuit already
        # has what it needs and keeps executing its stored path on its own
        # timer. Re-publishing an identical/trivial path each cycle just
        # resets pure_pursuit's "reached" flag and makes it re-log forever.
        self.robot1_last_succeeded_goal = None
        self.robot2_last_succeeded_goal = None

        self.robot1_last_failed_goal = None
        self.robot2_last_failed_goal = None


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

        self.robot1_goal_sub = self.create_subscription(
            PoseArray,
            '/robot1/frontier_goals',
            self.robot1_goal_callback,
            10
        )

        self.robot2_goal_sub = self.create_subscription(
            PoseArray,
            '/robot2/frontier_goals',
            self.robot2_goal_callback,
            10
        )


        self.robot1_path_pub = self.create_publisher(
            Path,
            '/robot1/planned_path',
            10
        )

        # The inflated obstacles A* plans around, for RViz (Map display,
        # color scheme "costmap"): 100 = wall, 99..60 = inflation band,
        # 0 = free. Latched so RViz shows it as soon as it subscribes.
        self.inflated_map_pub = self.create_publisher(
            OccupancyGrid,
            '/inflated_map',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        )
        self.last_inflation_publish = 0.0

        self.robot2_path_pub = self.create_publisher(
            Path,
            '/robot2/planned_path',
            10
        )

        self.robot1_planning_failed_pub = self.create_publisher(
            Bool,
            '/robot1/planning_failed',
            10
        )

        self.robot2_planning_failed_pub = self.create_publisher(
            Bool,
            '/robot2/planning_failed',
            10
        )

        # For metrics_logger's mean-planning-time metric.
        self.robot1_planning_time_pub = self.create_publisher(
            Float32,
            '/robot1/planning_time_ms',
            10
        )

        self.robot2_planning_time_pub = self.create_publisher(
            Float32,
            '/robot2/planning_time_ms',
            10
        )


        self.timer = self.create_timer(
            0.5,
            self.plan_paths
        )

        self.get_logger().info(
            'A* Planner started.'
        )

        self.get_logger().info(
            f'Obstacle inflation radius: '
            f'{self.obstacle_inflation_radius:.2f} m'
        )




    def map_callback(self, msg):

        self.map_msg = msg

        # The merged map can arrive at 4 Hz; 1 Hz is plenty for display.
        if time.time() - self.last_inflation_publish >= 1.0:
            self.last_inflation_publish = time.time()
            self.publish_inflated_map(msg)

    def publish_inflated_map(self, msg):

        info = msg.info
        width, height = info.width, info.height

        grid = np.array(msg.data, dtype=np.int16).reshape(height, width)

        # Same sources as is_free.
        source = grid > 50

        if self.inflate_unknown:
            source |= grid == -1

        r = int(math.ceil(self.obstacle_inflation_radius / info.resolution))

        cost = np.zeros((height, width), dtype=np.int16)
        padded = np.pad(source, r)

        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):

                distance = math.sqrt(dx * dx + dy * dy)

                if distance > r:
                    continue

                # 99 next to the wall fading to 60 at the inflation edge.
                value = int(round(99 - 39 * distance / max(r, 1)))
                shifted = padded[r + dy:r + dy + height, r + dx:r + dx + width]
                np.maximum(cost, np.where(shifted, value, 0), out=cost)

        cost[grid > 50] = 100

        out = OccupancyGrid()
        out.header = msg.header
        out.info = info
        out.data = cost.ravel().astype(np.int8).tolist()

        self.inflated_map_pub.publish(out)

    def robot1_odom_callback(self, msg):

        self.robot1_x, self.robot1_y, _ = self.pose_tracker.get('robot1', msg)


    def robot2_odom_callback(self, msg):

        self.robot2_x, self.robot2_y, _ = self.pose_tracker.get('robot2', msg)


    def robot1_goal_callback(self, msg):

        self.robot1_goals = []

        for pose in msg.poses:

            x = pose.position.x
            y = pose.position.y

            self.robot1_goals.append(
                (x, y)
            )

        if len(self.robot1_goals) > 0:

            self.robot1_current_goal = (
                self.robot1_goals[0]
            )

            # New goal
            self.robot1_last_failed_goal = None
            self.robot1_last_succeeded_goal = None

    def robot2_goal_callback(self, msg):

        self.robot2_goals = []

        for pose in msg.poses:

            x = pose.position.x
            y = pose.position.y

            self.robot2_goals.append(
                (x, y)
            )

        if len(self.robot2_goals) > 0:

            self.robot2_current_goal = (
                self.robot2_goals[0]
            )

            # New goal
            self.robot2_last_failed_goal = None
            self.robot2_last_succeeded_goal = None


    def plan_paths(self):

        if self.map_msg is None:
            return


        if (
            self.robot1_x is not None
            and self.robot1_current_goal is not None
            and self.robot1_current_goal != self.robot1_last_succeeded_goal
        ):

            plan_start = time.time()

            path = self.create_path(
                self.robot1_x,
                self.robot1_y,
                self.robot1_current_goal
            )

            planning_time_msg = Float32()
            planning_time_msg.data = (time.time() - plan_start) * 1000.0
            self.robot1_planning_time_pub.publish(planning_time_msg)

            if path is not None:

                self.robot1_last_succeeded_goal = (
                    self.robot1_current_goal
                )

                self.robot1_path_pub.publish(
                    path
                )

            else:

                if (
                    self.robot1_last_failed_goal
                    != self.robot1_current_goal
                ):

                    self.robot1_last_failed_goal = (
                        self.robot1_current_goal
                    )

                    self.get_logger().warn(
                        f'No A* path found to '
                        f'({self.robot1_current_goal[0]:.2f}, '
                        f'{self.robot1_current_goal[1]:.2f})'
                    )

                    fail_msg = Bool()
                    fail_msg.data = True

                    self.robot1_planning_failed_pub.publish(
                        fail_msg
                    )

        if (
            self.robot2_x is not None
            and self.robot2_current_goal is not None
            and self.robot2_current_goal != self.robot2_last_succeeded_goal
        ):

            plan_start = time.time()

            path = self.create_path(
                self.robot2_x,
                self.robot2_y,
                self.robot2_current_goal
            )

            planning_time_msg = Float32()
            planning_time_msg.data = (time.time() - plan_start) * 1000.0
            self.robot2_planning_time_pub.publish(planning_time_msg)

            if path is not None:

                self.robot2_last_succeeded_goal = (
                    self.robot2_current_goal
                )

                self.robot2_path_pub.publish(
                    path
                )

            else:

                if (
                    self.robot2_last_failed_goal
                    != self.robot2_current_goal
                ):

                    self.robot2_last_failed_goal = (
                        self.robot2_current_goal
                    )

                    self.get_logger().warn(
                        f'No A* path found to '
                        f'({self.robot2_current_goal[0]:.2f}, '
                        f'{self.robot2_current_goal[1]:.2f})'
                    )

                    fail_msg = Bool()
                    fail_msg.data = True

                    self.robot2_planning_failed_pub.publish(
                        fail_msg
                    )

    def create_path(
        self,
        robot_x,
        robot_y,
        goal
    ):

        goal_x, goal_y = goal

        start = self.world_to_grid(
            robot_x,
            robot_y
        )

        goal_cell = self.world_to_grid(
            goal_x,
            goal_y
        )

        if start is None:

            self.get_logger().warn(
                'Robot position is outside map.'
            )

            return None

        if goal_cell is None:

            self.get_logger().warn(
                'Goal is outside map.'
            )

            return None
#Find nearest free cell to start and goal if they are not free
        start = self.find_nearest_free(
            start
        )


        goal_cell = self.find_nearest_free(
            goal_cell
        )

        if start is None:

            self.get_logger().warn(
                'No safe cell found near robot start.'
            )

            return None

        if goal_cell is None:

            self.get_logger().warn(
                'No safe cell found near goal.'
            )

            return None

        if start == goal_cell:

            robot_to_goal_distance = math.sqrt(
                (robot_x - goal_x) ** 2
                + (robot_y - goal_y) ** 2
            )

            if robot_to_goal_distance > self.same_cell_snap_tolerance:

                return None


        # RUN A*

        path_cells = self.a_star(
            start,
            goal_cell
        )

        if path_cells is None:

            return None

        return self.grid_path_to_ros_path(
            path_cells
        )

    # FIND NEAREST SAFE CELL

    def find_nearest_free(self, cell):

        if cell is None:
            return None

        x, y = cell

        if self.is_free(x, y):
            return cell

        # Search progressively farther away
        for radius in range(1, 15):

            for dx in range(
                -radius,
                radius + 1
            ):

                for dy in range(
                    -radius,
                    radius + 1
                ):

                    nx = x + dx
                    ny = y + dy

                    if self.is_free(
                        nx,
                        ny
                    ):

                        return (
                            nx,
                            ny
                        )

        return None

   
    # A* ALGORITHM
  

    def a_star(
        self,
        start,
        goal
    ):

        open_set = []

        heapq.heappush(
            open_set,
            (0, start)
        )

        came_from = {}

        g_score = {
            start: 0.0
        }

        while open_set:

            current_f, current = (
                heapq.heappop(open_set)
            )

          
            # GOAL REACHED
        

            if current == goal:

                return self.reconstruct_path(
                    came_from,
                    current
                )

          
            # GET NEIGHBOURS
    

            neighbours = self.get_neighbours(
                current
            )

            for neighbour in neighbours:

                nx, ny = neighbour
            

                if not self.is_free(
                    nx,
                    ny
                ):

                    continue

                dx = nx - current[0]
                dy = ny - current[1]

                # PREVENT DIAGONAL CORNER CUTTING
    

                if (
                    abs(dx) == 1
                    and abs(dy) == 1
                ):

                    # Cell beside the robot horizontally
                    if not self.is_free(
                        current[0] + dx,
                        current[1]
                    ):

                        continue

                    # Cell beside the robot vertically
                    if not self.is_free(
                        current[0],
                        current[1] + dy
                    ):

                        continue

                    movement_cost = math.sqrt(2)

                else:

                    movement_cost = 1.0

               
                # G SCORE
          

                tentative_g = (
                    g_score[current]
                    + movement_cost
                )

                if (
                    neighbour not in g_score
                    or tentative_g
                    < g_score[neighbour]
                ):

                    came_from[
                        neighbour
                    ] = current

                    g_score[
                        neighbour
                    ] = tentative_g

                    h = self.heuristic(
                        neighbour,
                        goal
                    )

                    f = tentative_g + h

                    heapq.heappush(
                        open_set,
                        (
                            f,
                            neighbour
                        )
                    )

        return None

 
    # HEURISTIC


    def heuristic(
        self,
        cell,
        goal
    ):

        dx = cell[0] - goal[0]
        dy = cell[1] - goal[1]

        return math.sqrt(
            dx * dx + dy * dy
        )

   
    # GET 8 NEIGHBOURS


    def get_neighbours(
        self,
        cell
    ):

        x, y = cell

        directions = [

            (-1, -1),
            (-1, 0),
            (-1, 1),

            (0, -1),
            (0, 1),

            (1, -1),
            (1, 0),
            (1, 1)

        ]

        neighbours = []

        for dx, dy in directions:

            neighbours.append(
                (
                    x + dx,
                    y + dy
                )
            )

        return neighbours

    
    # CHECK WHETHER CELL IS SAFE
    def is_free(
        self,
        x,
        y
    ):

        width = self.map_msg.info.width
        height = self.map_msg.info.height

        data = self.map_msg.data
#check wjtehr its within the map boundaries

        if x < 0 or x >= width:
            return False

        if y < 0 or y >= height:
            return False


        resolution = (
            self.map_msg.info.resolution
        )

        inflation_cells = int(
            math.ceil(
                self.obstacle_inflation_radius
                / resolution
            )
        )

        # The cell itself must be known free, even when unknown
        # neighbours are allowed.
        center = data[y * width + x]

        if center > 50 or center == -1:
            return False

        for dx in range(
            -inflation_cells,
            inflation_cells + 1
        ):

            for dy in range(
                -inflation_cells,
                inflation_cells + 1
            ):

                # Circular safety area
                distance = math.sqrt(
                    dx * dx + dy * dy
                )

                if (
                    distance
                    > inflation_cells
                ):

                    continue

                nx = x + dx
                ny = y + dy

                # Outside map = unsafe
                if (
                    nx < 0
                    or nx >= width
                    or ny < 0
                    or ny >= height
                ):

                    return False

                index = (
                    ny * width
                    + nx
                )

                value = data[index]

                
                
                
                # 0     = free
                # -1    = unknown
                # 1-100 = occupied
                #
                # Unknown is treated as unsafe unless inflate_unknown
                # is False.


                if value > 50 or (value == -1 and self.inflate_unknown):

                    return False

        return True


    def world_to_grid(
        self,
        world_x,
        world_y
    ):

        resolution = (
            self.map_msg.info.resolution
        )

        origin_x = (
            self.map_msg.info.origin.position.x
        )

        origin_y = (
            self.map_msg.info.origin.position.y
        )

        grid_x = int(
            (world_x - origin_x)
            / resolution
        )

        grid_y = int(
            (world_y - origin_y)
            / resolution
        )

        width = self.map_msg.info.width
        height = self.map_msg.info.height

        if (
            grid_x < 0
            or grid_x >= width
            or grid_y < 0
            or grid_y >= height
        ):

            return None

        return (
            grid_x,
            grid_y
        )


    def grid_to_world(
        self,
        grid_x,
        grid_y
    ):

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
            + (grid_x + 0.5)
            * resolution
        )

        world_y = (
            origin_y
            + (grid_y + 0.5)
            * resolution
        )

        return (
            world_x,
            world_y
        )

    # RECONSTRUCT PATH
    def reconstruct_path(
        self,
        came_from,
        current
    ):

        path = [
            current
        ]

        while current in came_from:

            current = came_from[
                current
            ]

            path.append(
                current
            )

        path.reverse()

        return path
#convert grid path to ROS Path message

    def grid_path_to_ros_path(
        self,
        grid_path
    ):

        path_msg = Path()

        path_msg.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        path_msg.header.frame_id = 'map'

        for cell in grid_path:

            grid_x, grid_y = cell

            world_x, world_y = (
                self.grid_to_world(
                    grid_x,
                    grid_y
                )
            )

            pose = PoseStamped()

            pose.header.stamp = (
                path_msg.header.stamp
            )

            pose.header.frame_id = 'map'

            pose.pose.position.x = (
                world_x
            )

            pose.pose.position.y = (
                world_y
            )

            pose.pose.orientation.w = 1.0

            path_msg.poses.append(
                pose
            )

        return path_msg


def main(args=None):

    rclpy.init(args=args)

    node = AStarPlanner()

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

