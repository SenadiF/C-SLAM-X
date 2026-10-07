"""
multi_robot_hw.launch.py
=========================
Both real robots exploring together with the proposed stack: per-robot
odometry / EKF / SLAM, known_pose_merge into /map, and one
frontier / A* / pure pursuit set handling both robots.

Usage (micro-ROS agent must already be running, both ESP32s connected):
  ros2 launch robot_bringup multi_robot_hw.launch.py \
      robot1_pose:="0 0 0" robot2_pose:="-2.0 -1.5 0"

robotN_pose is "x y yaw" (metres, radians) of where robotN was switched on,
in a shared world frame - measure it before launching. The same values go to
  - a static TF map -> robotN/map, which known_pose_merge uses to place each
    robot's map in /map and the nav nodes use to locate the robots,
  - the nav nodes' robotN_init_pose (MapFramePose's fallback).
so they can no longer disagree.

Hardware settings (inflation, goal selection, pure pursuit speeds) match
single_robot_hw.launch.py - see the comments there.
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

ROBOTS = ('robot1', 'robot2')


def parse_pose(text, name):
    try:
        x, y, yaw = (float(v) for v in text.split())
    except ValueError:
        raise RuntimeError(f'{name}_pose must be "x y yaw", got {text!r}')
    return x, y, yaw


def launch_setup(context):
    poses = {
        robot: parse_pose(LaunchConfiguration(f'{robot}_pose').perform(context), robot)
        for robot in ROBOTS
    }

    bringup_share = get_package_share_directory('robot_bringup')
    loc_share = get_package_share_directory('robot_localization_config')

    inflation = 0.26
    nav_common = {'use_sim_time': False}
    for robot, (x, y, yaw) in poses.items():
        nav_common[f'{robot}_init_pose'] = [x, y, yaw]
    planning = {
        'obstacle_inflation_radius': inflation,
        'inflate_unknown': False,
    }

    front_end = [
        # Restamps scan_raw/imu_raw for both robots.
        Node(package='robot_control', executable='time_node', output='screen'),
    ]
    ekf = []
    slam = []

    for number, robot in enumerate(ROBOTS, start=1):
        x, y, yaw = poses[robot]
        robot_description = xacro.process_file(
            os.path.join(bringup_share, 'urdf', 'cslam_robot.urdf.xacro'),
            mappings={'robot_name': robot},
        ).toxml()

        front_end += [
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
                arguments=['--x', str(x), '--y', str(y), '--yaw', str(yaw),
                           '--frame-id', 'map', '--child-frame-id', f'{robot}/map'],
            ),
        ]

        ekf.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(loc_share, 'launch', 'localization.launch.py')),
            launch_arguments={'namespace': robot}.items(),
        ))

        slam += [
            Node(
                package='slam_toolbox',
                executable='sync_slam_toolbox_node',
                name='slam_toolbox',
                namespace=robot,
                output='screen',
                parameters=[os.path.join(loc_share, 'config', f'slam{number}.yaml')],
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

    # Not multirobot_map_merge: its known-poses mode ignores each map's own
    # origin (slam_toolbox moves it), which put robot2's map 2.3 m off. This
    # merges through the same map -> robotN/map TFs published above.
    map_merge = Node(
        package='my_navigation',
        executable='known_pose_merge',
        output='screen',
        parameters=[{'robots': list(ROBOTS), 'use_sim_time': False}],
    )

    nav = [
        Node(
            package='my_navigation',
            executable='frontier_explorer_node',
            output='screen',
            parameters=[nav_common, planning, {'goal_selection': 'reachable'}],
        ),
        Node(
            package='my_navigation',
            executable='astar_node',
            output='screen',
            parameters=[nav_common, planning],
        ),
        Node(
            package='my_navigation',
            executable='pure_pursuit_node',
            output='screen',
            parameters=[nav_common, {
                'bootstrap_enabled': False,
                'front_cone_deg': 60.0,
                'linear_speed': 0.6,
                'max_angular_speed': 1.5,
                'lookahead_distance': 0.25,
                'avoidance_angular_speed': 1.5,
                'emergency_distance': 0.25,
                'safe_distance': 0.45,
                # Real speeds are ~0.1-0.3 m/s, so allow long paths; give up
                # sooner when the robot is not moving at all (see
                # pure_pursuit_node.goal_given_up).
                'goal_pursuit_timeout': 60.0,
                'stuck_timeout': 10.0,
                # Hold still if a robot's scans stop arriving (bad Wi-Fi).
                'scan_timeout': 0.5,
            }],
        ),
    ]

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(bringup_share, 'rviz', 'multi_robot.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    # Staggered like commands.txt, to avoid slam_toolbox lifecycle races.
    # Navigation starts once the robots' SLAM maps are up and merged.
    return [
        rviz,
        *front_end,
        TimerAction(period=3.0, actions=ekf),
        TimerAction(period=6.0, actions=slam),
        TimerAction(period=10.0, actions=[map_merge]),
        TimerAction(period=25.0, actions=nav),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot1_pose', default_value='0 0 0',
            description='"x y yaw" where robot1 starts, in the shared map frame'),
        DeclareLaunchArgument(
            'robot2_pose', default_value='-2.0 -1.5 0',
            description='"x y yaw" where robot2 starts, in the shared map frame'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Open RViz with rviz/multi_robot.rviz'),
        OpaqueFunction(function=launch_setup),
    ])
