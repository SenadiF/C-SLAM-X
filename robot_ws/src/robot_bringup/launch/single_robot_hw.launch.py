"""
single_robot_hw.launch.py
==========================
One real robot exploring on its own with the proposed stack (frontier /
A* / pure pursuit), no map merging. Use it to validate the navigation
stack on one robot before running both.

Usage (micro-ROS agent must already be running, ESP32 connected):
  ros2 launch robot_bringup single_robot_hw.launch.py              # robot2
  ros2 launch robot_bringup single_robot_hw.launch.py robot:=robot1

How it stands in for the two-robot setup:
  - The nav nodes read the merged /map; here /map is remapped to the
    robot's own SLAM map (/robotN/map).
  - An identity static TF map -> robotN/map makes the robot's SLAM frame
    the global 'map' frame, so MapFramePose finds map -> robotN/base_link
    directly and the 'map'-stamped goals/paths display in RViz.
  - robotN_init_pose is 0,0,0 because the robot's own map is the world.
The other robot simply never gets a pose, and every nav node skips a
robot with no pose.
"""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    robot = LaunchConfiguration('robot').perform(context)
    number = robot[-1]
    loc_share = get_package_share_directory('robot_localization_config')
    slam_config = os.path.join(loc_share, 'config', f'slam{number}.yaml')

    # The robot is 22 x 15 cm, so its farthest corner is 0.133 m from the
    # rotation centre. Pure pursuit cuts corners by ~0.35 * lookahead
    # (0.09 m at 0.25), so the inflation must cover 0.133 + 0.09 plus a
    # margin; at 0.22 the corners clipped walls on turns. Narrowest passage
    # it will plan through: 2 * 0.26 = 0.52 m.
    inflation = 0.26
    nav_common = {
        f'{robot}_init_pose': [0.0, 0.0, 0.0],
        'use_sim_time': False,
    }
    # Real LiDAR maps have unknown gaps between rays; only inflate walls.
    planning = {
        'obstacle_inflation_radius': inflation,
        'inflate_unknown': False,
    }
    map_remap = [('/map', f'/{robot}/map')]

    # The URDF gives base_link -> lidar_link / imu_link (replacing the
    # static TFs from static_tf.launch.py) and the model shown in RViz.
    robot_description = xacro.process_file(
        os.path.join(get_package_share_directory('robot_bringup'), 'urdf', 'cslam_robot.urdf.xacro'),
        mappings={'robot_name': robot},
    ).toxml()

    front_end = [
        Node(
            package='robot_odometry',
            executable='wheel_odometry_node',
            name=f'wheel_odometry_{robot}',
            output='screen',
            parameters=[{'robot_name': robot}],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=robot,
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'map_to_{robot}_map',
            arguments=['0', '0', '0', '0', '0', '0', 'map', f'{robot}/map'],
        ),
        # Restamps scan_raw/imu_raw (firmware sends zero stamps).
        Node(
            package='robot_control',
            executable='time_node',
            output='screen',
        ),
    ]

    ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(loc_share, 'launch', 'localization.launch.py')),
        launch_arguments={'namespace': robot}.items(),
    )

    slam = [
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            namespace=robot,
            output='screen',
            parameters=[slam_config],
            remappings=[('/map', 'map'), ('/map_metadata', 'map_metadata')],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_slam',
            namespace=robot,
            output='screen',
            parameters=[{
                'autostart': True,
                'bond_timeout': 0.0,
                'node_names': ['slam_toolbox'],
            }],
        ),
    ]

    nav = [
        Node(
            package='my_navigation',
            executable='frontier_explorer_node',
            output='screen',
            parameters=[nav_common, planning, {'goal_selection': 'reachable'}],
            remappings=map_remap,
        ),
        Node(
            package='my_navigation',
            executable='astar_node',
            output='screen',
            parameters=[nav_common, planning],
            remappings=map_remap,
        ),
        Node(
            package='my_navigation',
            executable='pure_pursuit_node',
            output='screen',
            parameters=[nav_common, {
                'bootstrap_enabled': False,
                # Wider than 45 so obstacles diagonally ahead also slow/stop it.
                'front_cone_deg': 60.0,
                # Full power, like teleop: the firmware maps 0.6 m/s
                # (MAX_WHEEL_SPEED_MS) to PWM 255, and teleop's "speed 2.78"
                # is clamped to the same 255. At commands.txt's 0.25 / 1.0
                # the loaded robot barely moved under navigation.
                'linear_speed': 0.6,
                'max_angular_speed': 1.5,
                # Pure pursuit cuts corners by roughly lookahead * 0.35
                # (see inflation above).
                'lookahead_distance': 0.25,
                'avoidance_angular_speed': 1.5,
                'emergency_distance': 0.25,
                'safe_distance': 0.45,
            }],
        ),
    ]

    # Relative topics in the config resolve inside the robot's namespace.
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        namespace=robot,
        arguments=['-d', os.path.join(
            get_package_share_directory('robot_bringup'), 'rviz', 'single_robot.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    # Staggered like commands.txt, to avoid slam_toolbox lifecycle races.
    return [
        rviz,
        *front_end,
        TimerAction(period=3.0, actions=[ekf]),
        TimerAction(period=6.0, actions=slam),
        TimerAction(period=15.0, actions=nav),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot', default_value='robot2',
            description='Which robot explores alone: robot1 or robot2'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Open RViz with rviz/single_robot.rviz'),
        OpaqueFunction(function=launch_setup),
    ])
