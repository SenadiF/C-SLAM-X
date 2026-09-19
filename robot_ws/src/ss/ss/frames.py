import math

# Gazebo's DiffDrive plugin publishes odometry relative to each robot's own
# spawn point, not the shared world frame - so raw /robotN/odom starts at
# local (0,0,0) no matter where the robot actually spawned. These are the
# spawn poses from baseline_sim.launch.py, used to convert local odom back
# into the shared world frame that robot_coordinator / multirobot_slam need.
SPAWN_POSES = {
    'robot1': (0.0, 0.0, 0.0),
    'robot2': (-2.0, -1.5, math.pi),
}


def to_world_frame(robot_name, local_x, local_y, local_yaw):
    spawn_x, spawn_y, spawn_yaw = SPAWN_POSES.get(robot_name, (0.0, 0.0, 0.0))
    cos_yaw = math.cos(spawn_yaw)
    sin_yaw = math.sin(spawn_yaw)
    world_x = spawn_x + local_x * cos_yaw - local_y * sin_yaw
    world_y = spawn_y + local_x * sin_yaw + local_y * cos_yaw
    world_yaw = math.atan2(math.sin(spawn_yaw + local_yaw), math.cos(spawn_yaw + local_yaw))
    return world_x, world_y, world_yaw


def yaw_from_quaternion(orientation):
    siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
    cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
    return math.atan2(siny_cosp, cosy_cosp)
