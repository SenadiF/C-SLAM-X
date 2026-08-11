import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseStamped

import math
import heapq # gives us a priority queue



class AStarPlanner(Node):

    def __init__(self):


        super().__init__('astar_planner')
        self.map_msg = None

        # Robot 1 current position.
        self.robot1_x = None
        self.robot1_y = None

        # Robot 2 current position.
        self.robot2_x = None
        self.robot2_y = None
        #Robot 1's queue.    
        self.robot1_goals = []

        # Robot 2's queue.
        self.robot2_goals = []

        self.robot1_current_goal = None
        self.robot2_current_goal = None

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

        self.robot2_path_pub = self.create_publisher(
            Path,
            '/robot2/planned_path',
            10
        )

       # plan paths every 0.5 seconds.

        self.timer = self.create_timer(
            0.5,
            self.plan_paths
        )


        self.get_logger().info(
            'A* Planner started.'
        )


    def map_callback(self, msg):

        # Save the newest map.
        self.map_msg = msg
    def robot1_odom_callback(self, msg):

        
        self.robot1_x = msg.pose.pose.position.x

        
        self.robot1_y = msg.pose.pose.position.y


    def robot2_odom_callback(self, msg):

       
        self.robot2_x = msg.pose.pose.position.x


        self.robot2_y = msg.pose.pose.position.y


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

        else:

            self.robot1_current_goal = None


    def robot2_goal_callback(self, msg):

        # Clear old queue.
        self.robot2_goals = []


        # Copy all new goals.
        for pose in msg.poses:

            x = pose.position.x
            y = pose.position.y

            self.robot2_goals.append(
                (x, y)
            )


        # Set the first goal as the current goal.
        if len(self.robot2_goals) > 0:

            self.robot2_current_goal = (
                self.robot2_goals[0]
            )

        else:

            self.robot2_current_goal = None



    def plan_paths(self):
        # First check whetehr the map is available.

        if self.map_msg is None:

            return


        if (
            self.robot1_x is not None
            and
            self.robot1_current_goal is not None
        ):

            path = self.create_path(

                self.robot1_x,
                self.robot1_y,

                self.robot1_current_goal
            )


            if path is not None:

                self.robot1_path_pub.publish(
                    path
                )


        if (
            self.robot2_x is not None
            and
            self.robot2_current_goal is not None
        ):

            path = self.create_path(

                self.robot2_x,
                self.robot2_y,

                self.robot2_current_goal
            )


            if path is not None:

                self.robot2_path_pub.publish(
                    path
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

        if not self.is_free(
            start[0],
            start[1]
        ):

            self.get_logger().warn(
                'Robot start cell is occupied.'
            )

            return None


        if not self.is_free(
            goal_cell[0],
            goal_cell[1]
        ):

            self.get_logger().warn(
                'Goal cell is occupied.'
            )

            return None
        path_cells = self.a_star(
            start,
            goal_cell
        )

        if path_cells is None:

            self.get_logger().warn(
                f'No A* path found to '
                f'({goal_x:.2f}, {goal_y:.2f})'
            )

            return None


        return self.grid_path_to_ros_path(
            path_cells
        )


    def a_star(
        self,
        start,
        goal
    ):
       #heapq is a priority queue that will store cells to explore.
        # Each entry is a tuple: (f_score, cell)

        open_set = []

        heapq.heappush(
            open_set,
            (
                0,
                start
            )
        )


        came_from = {}

        # stores the known cost from START to each cell.
       

        g_score = {

            start: 0.0

        }



        while open_set:

            # Get the cell with the smallest f score.
            current_f, current = heapq.heappop(
                open_set
            )


            if current == goal:

                return self.reconstruct_path(
                    came_from,
                    current
                )


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


                dx = abs(
                    nx - current[0]
                )

                dy = abs(
                    ny - current[1]
                )
                #path cost is 1 for horizontal and vertical moves, and sqrt(2) for diagonal moves.

                if dx == 1 and dy == 1:

                    movement_cost = math.sqrt(2)

                else:

                    movement_cost = 1.0


                tentative_g = (

                    g_score[current]
                    +
                    movement_cost

                )

                if (

                    neighbour not in g_score

                    or

                    tentative_g
                    <
                    g_score[neighbour]

                ):

                    came_from[neighbour] = current

                    g_score[neighbour] = (
                        tentative_g
                    )


                    h = self.heuristic(
                        neighbour,
                        goal
                    )
                    f = (
                        tentative_g
                        +
                        h
                    )


                    # Add neighbour to priority queue.
                    heapq.heappush(
                        open_set,
                        (
                            f,
                            neighbour
                        )
                    )



        return None



    def heuristic(
        self,
        cell,
        goal
    ):

        dx = cell[0] - goal[0]
        dy = cell[1] - goal[1]

        return math.sqrt(
            dx * dx +
            dy * dy
        )


    def get_neighbours(
        self,
        cell
    ):

        x, y = cell


        directions = [

            (-1, -1),
            (-1,  0),
            (-1,  1),

            ( 0, -1),
            ( 0,  1),

            ( 1, -1),
            ( 1,  0),
            ( 1,  1)

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
#Checks if a cell is free  in the occupancy grid map.

    def is_free(
        self,
        x,
        y
    ):

        #Get the map dimensions.

        width = self.map_msg.info.width
        height = self.map_msg.info.height
        # Check map boundaries.
       

        if x < 0 or x >= width:

            return False


        if y < 0 or y >= height:

            return False


        index = (
            y * width
            +
            x
        )


        value = self.map_msg.data[index]


        if value == 0:

            return True


        return False

#Converts world coordinates to grid coordinates based on the occupancy grid map's resolution and origin.

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
            /
            resolution
        )


        grid_y = int(
            (world_y - origin_y)
            /
            resolution
        )

        width = self.map_msg.info.width
        height = self.map_msg.info.height


        if (

            grid_x < 0
            or
            grid_x >= width
            or
            grid_y < 0
            or
            grid_y >= height

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
            +
            (grid_x + 0.5)
            *
            resolution

        )


        world_y = (

            origin_y
            +
            (grid_y + 0.5)
            *
            resolution

        )


        return (
            world_x,
            world_y
        )


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
        #Reverse the path so that it goes from start to goal.

        path.reverse()


        return path



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


        # The path is expressed in the global map frame.
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


            pose.pose.position.x = world_x
            pose.pose.position.y = world_y


            pose.pose.orientation.w = 1.0


            path_msg.poses.append(
                pose
            )


        return path_msg

def main(args=None):

    # Start ROS 2.
    rclpy.init(args=args)


    # Create our A* node.
    node = AStarPlanner()


    try:

        # Keep the node alive.
        rclpy.spin(node)


    except KeyboardInterrupt:

        pass


    finally:

        # Clean up.
        node.destroy_node()

        rclpy.shutdown()


# ============================================================
# START THE PROGRAM
# ============================================================

if __name__ == '__main__':

    main()
