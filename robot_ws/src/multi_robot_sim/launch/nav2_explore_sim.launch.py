"""
nav2_explore_sim.launch.py
============================
Single-robot Gazebo simulation testing the FULL Nav2 + explore_lite
autonomous exploration pipeline: EKF -> SLAM -> Nav2 -> explore_lite.

This is deliberately single-robot to minimize risk while validating
this specific pipeline - your custom frontier/astar/pure_pursuit stack
is tested separately in your_stack_sim.launch.py.

REQUIRES: your existing nav2_navigation_no_dock.launch.py (built earlier
today to remove the docking_server crash) copied into this package's
launch/ folder alongside this file.

Usage:
  ros2 launch multi_robot_sim nav2_explore_sim.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_share = get_package_share_directory('multi_robot_sim')
    world_path = os.path.join(pkg_share, 'worlds', 'two_room_test.sdf')
    bridge_config = os.path.join(pkg_share, 'config', 'bridge.yaml')
    nav2_params = os.path.join(pkg_share, 'config', 'nav2_params_sim.yaml')

    robot1_x, robot1_y, robot1_yaw = '0.0', '0.0', '0.0'

    gazebo = ExecuteProcess(cmd=['gz', 'sim', '-r', world_path], output='screen')

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        arguments=['--ros-args', '-p', f'config_file:={bridge_config}']
    )

    spawn_robot1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'spawn_robot.launch.py')),
        launch_arguments={'robot_name': 'robot1', 'x': robot1_x, 'y': robot1_y, 'yaw': robot1_yaw}.items()
    )

    ekf1 = Node(
        package='robot_localization', executable='ekf_node', name='ekf_filter_node',
        namespace='robot1', output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'ekf1_sim.yaml')]
    )

    slam1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': os.path.join(pkg_share, 'config', 'slam1_sim.yaml'),
            'use_sim_time': 'true',
            'namespace': 'robot1',
        }.items()
    )

    # Uses your existing docking-removed Nav2 launch file - copy it into
    # this package's launch/ folder before running.
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'nav2_navigation_no_dock.launch.py')
        ),
        launch_arguments={
            'namespace': 'robot1',
            'params_file': nav2_params,
            'use_sim_time': 'true',
            'autostart': 'true',
        }.items()
    )

    explore_lite = Node(
        package='explore_lite',
        executable='explore',
        name='explore_node',
        namespace='robot1',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_base_frame': 'robot1/base_link',
            'costmap_topic': 'global_costmap/costmap',
            'costmap_updates_topic': 'global_costmap/costmap_updates',
            'visualize': True,
            'planner_frequency': 0.33,
            'progress_timeout': 30.0,
            'potential_scale': 3.0,
            'orientation_scale': 0.0,
            'gain_scale': 1.0,
            'transform_tolerance': 0.3,
            'min_frontier_size': 0.5,
        }]
    )

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        output='screen', parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        gazebo,
        TimerAction(period=3.0, actions=[bridge]),
        TimerAction(period=5.0, actions=[spawn_robot1]),
        TimerAction(period=8.0, actions=[ekf1]),
        TimerAction(period=10.0, actions=[slam1]),
        # Extra delay before Nav2 - this is the exact step that failed
        # last time due to TF not being live yet. Giving SLAM's map->odom
        # transform time to establish first.
        TimerAction(period=15.0, actions=[nav2]),
        TimerAction(period=20.0, actions=[explore_lite]),
        TimerAction(period=21.0, actions=[rviz]),
    ])
