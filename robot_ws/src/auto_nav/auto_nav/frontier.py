
import math
import rclpy

from rclpy.node import Node

import numpy as np

from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry

from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose

class FrontierExplorer(Node):

    def __init__(self):

        super().__init__('frontier_explorer')

        # Distance between frontier cells to be considered part of the same cluster.
        self.declare_parameter(
            'frontier_cluster_distance',
            0.30
        )

        

        self.declare_parameter(
            'minimum_frontier_distance',
            0.40
        )
        #frontier goal queue size
        self.declare_parameter(
            'queue_size',
            5
        )

        self.cluster_distance = self.get_parameter(
            'frontier_cluster_distance'
        ).value

        self.minimum_frontier_distance = self.get_parameter(
            'minimum_frontier_distance'
        ).value

        self.queue_size = self.get_parameter(
            'queue_size'
        ).value



        # Robot 1 position.
        self.robot1_x = None
        self.robot1_y = None

        # Robot 2 position.
        self.robot2_x = None
        self.robot2_y = None


      
        self.map_msg = None

        self.robot1_goals = []
        self.robot2_goals = []


        #Kepp track of frontiers that have failed too many times.
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


        # Robot 1 receives its frontier queue.
        self.robot1_goal_pub = self.create_publisher(
            PoseArray,
            '/robot1/frontier_goals',
            10
        )


        # Robot 2 receives its frontier queue.
        self.robot2_goal_pub = self.create_publisher(
            PoseArray,
            '/robot2/frontier_goals',
            10
        )

        #every 2 seconds, run the exploration function.

        self.timer = self.create_timer(
            2.0,
            self.explore
        )


        self.get_logger().info(
            'Frontier Explorer started.'
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



    def explore(self):


        if self.map_msg is None:

            self.get_logger().info(
                'Waiting for /map...'
            )

            return


        if self.robot1_x is None or self.robot2_x is None:

            self.get_logger().info(
                'Waiting for both robot positions...'
            )

            return

        frontier_cells = self.find_frontier_cells()


        if len(frontier_cells) == 0:

            self.get_logger().info(
                'No frontiers found.'
            )

            return

        clusters = self.cluster_frontiers(
            frontier_cells
        )

        #Get one point per cluster to use as a goal for the robots.

        frontier_points = []

        for cluster in clusters:

            point = self.cluster_center(
                cluster
            )

            if point is not None:

                frontier_points.append(
                    point
                )
        #Remove frontiers that are too close to either robot.

        useful_frontiers = []

        for frontier in frontier_points:

            x, y = frontier

            d1 = self.distance(
                self.robot1_x,
                self.robot1_y,
                x,
                y
            )

            d2 = self.distance(
                self.robot2_x,
                self.robot2_y,
                x,
                y
            )

            if (
                d1 >= self.minimum_frontier_distance
                or
                d2 >= self.minimum_frontier_distance
            ):

                useful_frontiers.append(
                    frontier
                )


    

        self.allocate_frontiers(
            useful_frontiers
        )


        self.publish_goal_queues()







    # OccupancyGrid values:
    #
    #       -1 = unknown
    #        0 = free
    #      100 = occupied
   
   

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

                # Check the 4 neighbouring cells.
               

                neighbours = [

                    data[index - 1],       # left

                    data[index + 1],       # right

                    data[index - width],   # down

                    data[index + width]    # up

                ]
# If any of the neighbouring cells are unknown (-1), then this cell is a frontier cell.

                if -1 in neighbours:

                    frontier_cells.append(
                        (x, y)
                    )


        return frontier_cells



    def cluster_frontiers(self, cells):

        clusters = []
    #Convert the cluster distance from meters to grid cells.

        resolution = self.map_msg.info.resolution

        threshold_cells = max(
            1,
            int(
                self.cluster_distance /
                resolution
            )
        )



        unused = set(cells)

      #Cluster the frontier cells using a flood fill algorithm.

        while unused:

            seed = unused.pop()

            cluster = [seed]

            queue = [seed]


            while queue:

                current = queue.pop()

                cx, cy = current


                nearby = []


                # Search nearby cells.
                for dx in range(
                    -threshold_cells,
                    threshold_cells + 1
                ):

                    for dy in range(
                        -threshold_cells,
                        threshold_cells + 1
                    ):

                        if dx == 0 and dy == 0:
                            continue

                        nearby.append(
                            (cx + dx, cy + dy)
                        )


                for neighbour in nearby:

                    if neighbour in unused:

                        unused.remove(
                            neighbour
                        )

                        cluster.append(
                            neighbour
                        )

                        queue.append(
                            neighbour
                        )


            
            # Ignore extremely tiny clusters.
          

            if len(cluster) >= 3:

                clusters.append(
                    cluster
                )


        return clusters
#Get the center of a cluster of frontier cells in world coordinates.

    def cluster_center(self, cluster):

        if len(cluster) == 0:

            return None

        # Calculate average grid position.
    
        avg_x = sum(
            cell[0]
            for cell in cluster
        ) / len(cluster)


        avg_y = sum(
            cell[1]
            for cell in cluster
        ) / len(cluster)


      
        # Convert MAP CELL into a world coordinate using the map's resolution and origin.
        

        resolution = self.map_msg.info.resolution

        origin_x = self.map_msg.info.origin.position.x
        origin_y = self.map_msg.info.origin.position.y


        world_x = (
            origin_x +
            (avg_x + 0.5) * resolution
        )

        world_y = (
            origin_y +
            (avg_y + 0.5) * resolution
        )


        return (
            world_x,
            world_y
        )

#Allocate frontiers to the two robots based on their distance to each frontier.

    def allocate_frontiers(
        self,
        frontiers
    ):


        new_robot1 = []
        new_robot2 = []

        scored_frontiers = []


        for frontier in frontiers:

            x, y = frontier


            d1 = self.distance(
                self.robot1_x,
                self.robot1_y,
                x,
                y
            )


            d2 = self.distance(
                self.robot2_x,
                self.robot2_y,
                x,
                y
            )



            if self.is_failed(frontier):

                continue


            scored_frontiers.append(
                (
                    frontier,
                    d1,
                    d2
                )
            )



        scored_frontiers.sort(
            key=lambda item:
            min(item[1], item[2])
        )



        for frontier, d1, d2 in scored_frontiers:
#based on the distance to each robot, assign the frontier to the closest robot's queue.

            if d1 <= d2:

                if len(new_robot1) < self.queue_size:

                    new_robot1.append(
                        frontier
                    )

                elif len(new_robot2) < self.queue_size:

                    new_robot2.append(
                        frontier
                    )



                
        
            else:

                if len(new_robot2) < self.queue_size:

                    new_robot2.append(
                        frontier
                    )

                elif len(new_robot1) < self.queue_size:

                    new_robot1.append(
                        frontier
                    )




        self.robot1_goals = new_robot1

        self.robot2_goals = new_robot2


        self.get_logger().info(
            f'Robot 1 goals: {len(self.robot1_goals)} | '
            f'Robot 2 goals: {len(self.robot2_goals)}'
        )






    def is_failed(self, frontier):


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


            if distance < 0.30:

                # If this frontier has failed 3 or more times,
                # don't keep sending the robot there.
                if attempts >= 3:

                    return True


        return False


    def distance(
        self,
        x1,
        y1,
        x2,
        y2
    ):

        return math.sqrt(
            (x2 - x1) ** 2 +
            (y2 - y1) ** 2
        )


    def publish_goal_queues(self):


        msg1 = PoseArray()

        msg1.header.stamp = (
            self.get_clock().now().to_msg()
        )

        msg1.header.frame_id = 'map'



        for x, y in self.robot1_goals:

            pose = Pose()

            pose.position.x = x
            pose.position.y = y

            
            pose.orientation.w = 1.0

            msg1.poses.append(
                pose
            )


        msg2 = PoseArray()

        msg2.header.stamp = (
            self.get_clock().now().to_msg()
        )

        msg2.header.frame_id = 'map'


        for x, y in self.robot2_goals:

            pose = Pose()

            pose.position.x = x
            pose.position.y = y

            pose.orientation.w = 1.0

            msg2.poses.append(
                pose
            )



        self.robot1_goal_pub.publish(
            msg1
        )

        self.robot2_goal_pub.publish(
            msg2
        )


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