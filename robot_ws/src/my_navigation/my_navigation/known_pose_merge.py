"""Merge each robot's SLAM map into one /map using the known map -> robotN/map TF.

Replaces multirobot_map_merge's known-initial-poses mode on hardware. That mode
applies the init poses but not each map's own origin, which slam_toolbox
moves as the map grows, so a robot's map landed metres away from the robot
(measured: robot2's grid copied in with a 2.3 m origin error) and the robot
appeared to stand in unknown space - no reachable frontiers.

Here every merged cell is mapped back into each robot's map through the TF
and that map's real origin, and sampled there (works for any yaw):
occupied if any robot says occupied, else free if any says free, else unknown.
"""

import math

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class KnownPoseMerge(Node):

    def __init__(self):
        super().__init__('known_pose_merge')

        self.declare_parameter('robots', ['robot1', 'robot2'])
        self.declare_parameter('world_frame', 'map')
        self.declare_parameter('merge_rate', 2.0)

        self.robots = list(self.get_parameter('robots').value)
        self.world_frame = self.get_parameter('world_frame').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.maps = {}
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for robot in self.robots:
            self.create_subscription(
                OccupancyGrid, f'/{robot}/map',
                lambda msg, robot=robot: self.maps.__setitem__(robot, msg), latched)

        self.merged_pub = self.create_publisher(OccupancyGrid, '/map', latched)
        self.create_timer(1.0 / self.get_parameter('merge_rate').value, self.merge)
        self.missing_logged = set()

        self.get_logger().info(f'Merging {self.robots} into /map using TF {self.world_frame} -> robotN/map.')

    def robot_transform(self, robot, frame):
        """(x, y, yaw) of the robot's map frame in the world frame, or None."""
        try:
            t = self.tf_buffer.lookup_transform(self.world_frame, frame, Time())
        except TransformException:
            return None
        return (t.transform.translation.x, t.transform.translation.y,
                yaw_from_quaternion(t.transform.rotation))

    def merge(self):
        layers = []
        for robot in self.robots:
            msg = self.maps.get(robot)
            if msg is None or msg.info.width == 0:
                continue
            pose = self.robot_transform(robot, msg.header.frame_id)
            if pose is None:
                if robot not in self.missing_logged:
                    self.missing_logged.add(robot)
                    self.get_logger().warn(
                        f'No TF {self.world_frame} -> {msg.header.frame_id}; {robot} left out of /map.')
                continue
            self.missing_logged.discard(robot)
            layers.append((msg, pose))

        if not layers:
            return

        resolution = layers[0][0].info.resolution

        # World-frame bounding box of all maps' corners.
        corners = []
        for msg, (tx, ty, yaw) in layers:
            i = msg.info
            ox, oy = i.origin.position.x, i.origin.position.y
            c, s = math.cos(yaw), math.sin(yaw)
            for mx in (ox, ox + i.width * i.resolution):
                for my in (oy, oy + i.height * i.resolution):
                    corners.append((tx + c * mx - s * my, ty + s * mx + c * my))
        xs, ys = zip(*corners)
        min_x, min_y = min(xs), min(ys)
        width = int(math.ceil((max(xs) - min_x) / resolution))
        height = int(math.ceil((max(ys) - min_y) / resolution))

        # World coordinates of every merged cell centre.
        gx, gy = np.meshgrid(np.arange(width), np.arange(height))
        wx = min_x + (gx + 0.5) * resolution
        wy = min_y + (gy + 0.5) * resolution

        any_free = np.zeros((height, width), dtype=bool)
        any_occupied = np.zeros((height, width), dtype=bool)

        for msg, (tx, ty, yaw) in layers:
            i = msg.info
            grid = np.array(msg.data, dtype=np.int16).reshape(i.height, i.width)
            # World -> robot map frame -> that map's cell indices.
            c, s = math.cos(yaw), math.sin(yaw)
            dx, dy = wx - tx, wy - ty
            mx = c * dx + s * dy
            my = -s * dx + c * dy
            cx = np.floor((mx - i.origin.position.x) / i.resolution).astype(np.int64)
            cy = np.floor((my - i.origin.position.y) / i.resolution).astype(np.int64)
            inside = (cx >= 0) & (cx < i.width) & (cy >= 0) & (cy < i.height)
            values = np.full((height, width), -1, dtype=np.int16)
            values[inside] = grid[cy[inside], cx[inside]]
            any_occupied |= values > 50
            any_free |= (values >= 0) & (values <= 50)

        merged = np.full((height, width), -1, dtype=np.int8)
        merged[any_free] = 0
        merged[any_occupied] = 100

        out = OccupancyGrid()
        out.header.frame_id = self.world_frame
        out.header.stamp = self.get_clock().now().to_msg()
        out.info.resolution = resolution
        out.info.width = width
        out.info.height = height
        out.info.origin.position.x = min_x
        out.info.origin.position.y = min_y
        out.info.origin.orientation.w = 1.0
        out.data = merged.ravel().tolist()
        self.merged_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = KnownPoseMerge()
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
