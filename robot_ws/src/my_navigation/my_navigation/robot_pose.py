"""Robot poses in the shared 'map' frame.

/robotN/odometry/filtered is in robotN/odom, which starts at (0, 0) wherever
that robot was switched on. Frontier goals, A* paths and the merged /map are
all in 'map'. Using odometry coordinates directly put robot2 off by its whole
start offset (and ignored SLAM's drift correction for both robots), so
planning and path following for robot2 started from the wrong place.

The pose is taken from TF in this order:
  1. map -> robotN/base_link  (map_merge + slam_toolbox + EKF, the real thing)
  2. robotN/map -> robotN/base_link, composed with the robot's start pose
     (when map_merge isn't publishing its TF yet)
  3. odometry, composed with the robot's start pose (when SLAM isn't up yet)

robotN_init_pose must match the init_pose values given to map_merge.
"""

import math

from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def compose(base, local):
    bx, by, byaw = base
    lx, ly, lyaw = local
    c, s = math.cos(byaw), math.sin(byaw)
    yaw = math.atan2(math.sin(byaw + lyaw), math.cos(byaw + lyaw))
    return bx + c * lx - s * ly, by + s * lx + c * ly, yaw


class MapFramePose:

    DEFAULT_INIT_POSES = {
        'robot1': [0.0, 0.0, 0.0],
        'robot2': [-2.0, -1.5, 0.0],
    }

    def __init__(self, node):
        self.node = node
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)
        self.last_source = {}
        for robot, default in self.DEFAULT_INIT_POSES.items():
            node.declare_parameter(f'{robot}_init_pose', default)

    def init_pose(self, robot):
        return [float(v) for v in self.node.get_parameter(f'{robot}_init_pose').value]

    def get(self, robot, odom_msg):
        """Return (x, y, yaw) of `robot` in the 'map' frame."""
        base = f'{robot}/base_link'
        for parent, needs_init in (('map', False), (f'{robot}/map', True)):
            try:
                t = self.tf_buffer.lookup_transform(parent, base, Time())
            except TransformException:
                continue
            pose = (t.transform.translation.x,
                    t.transform.translation.y,
                    yaw_from_quaternion(t.transform.rotation))
            if needs_init:
                pose = compose(self.init_pose(robot), pose)
            self._note_source(robot, f'TF {parent} -> {base}')
            return pose

        p = odom_msg.pose.pose
        pose = compose(self.init_pose(robot),
                       (p.position.x, p.position.y, yaw_from_quaternion(p.orientation)))
        self._note_source(robot, 'odometry + init pose (no SLAM TF yet)')
        return pose

    def _note_source(self, robot, source):
        if self.last_source.get(robot) != source:
            self.last_source[robot] = source
            self.node.get_logger().info(f'{robot} pose source: {source}')
