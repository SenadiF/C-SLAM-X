"""
your_stack_sim.launch.py
==========================
Runs YOUR full stack in Gazebo simulation: EKF x2, SLAM Toolbox x2,
map_merge (switchable mode), frontier.py, astar.py, pure_pursuit_node.py.

Usage:
  ros2 launch multi_robot_sim your_stack_sim.launch.py merge_mode:=known
  ros2 launch multi_robot_sim your_stack_sim.launch.py merge_mode:=unknown

NOTE: verify the executable names below match what
`ros2 pkg executables my_navigation` actually shows before running —
these are best-guess names based on your filenames
(frontier.py, astar.py, pure_pursuit_node.py).
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_share = get_package_share_directory('multi_robot_sim')
    world_path = os.path.join(pkg_share, 'worlds', 'two_room_test.sdf')
    bridge_config = os.path.join(pkg_share, 'config', 'bridge.yaml')  # odom_sim topics, matches your stack's EKF input

    nav_pkg = 'my_navigation'   # TODO confirm this matches your actual package name

    merge_mode = LaunchConfiguration('merge_mode')

    robot1_x, robot1_y, robot1_yaw = '0.0', '0.0', '0.0'
    robot2_x, robot2_y, robot2_yaw = '-2.0', '-1.5', '0.0'

    declare_merge_mode = DeclareLaunchArgument(
        'merge_mode', default_value='known', description='known | unknown'
    )

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
    spawn_robot2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'spawn_robot.launch.py')),
        launch_arguments={'robot_name': 'robot2', 'x': robot2_x, 'y': robot2_y, 'yaw': robot2_yaw}.items()
    )

    ekf1 = Node(
        package='robot_localization', executable='ekf_node', name='ekf_filter_node',
        namespace='robot1', output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'ekf1_sim.yaml')]
    )
    ekf2 = Node(
        package='robot_localization', executable='ekf_node', name='ekf_filter_node',
        namespace='robot2', output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'ekf2_sim.yaml')]
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
    slam2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': os.path.join(pkg_share, 'config', 'slam2_sim.yaml'),
            'use_sim_time': 'true',
            'namespace': 'robot2',
        }.items()
    )

    map_merge_known = Node(
        package='multirobot_map_merge', executable='map_merge', name='map_merge',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'map_merge_known_sim.yaml')],
        condition=IfCondition(PythonExpression(["'", merge_mode, "' == 'known'"]))
    )
    map_merge_unknown = Node(
        package='multirobot_map_merge', executable='map_merge', name='map_merge',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'map_merge_unknown_sim.yaml')],
        condition=IfCondition(PythonExpression(["'", merge_mode, "' == 'unknown'"]))
    )

    # Confirmed executable names from my_navigation's setup.py
    frontier = Node(
        package=nav_pkg, executable='frontier_explorer_node', name='frontier_explorer',
        output='screen', parameters=[{'use_sim_time': True}]
    )
    astar = Node(
        package=nav_pkg, executable='astar_node', name='astar_planner',
        output='screen', parameters=[{'use_sim_time': True}]
    )
    pure_pursuit = Node(
        package=nav_pkg, executable='pure_pursuit_node', name='pure_pursuit',
        output='screen', parameters=[{'use_sim_time': True}]
    )
    # NOTE: cmd_vel_relay_node also exists in this package but its role/
    # topic wiring is unconfirmed - add it here once you clarify what it
    # does (likely relays/merges cmd_vel from multiple sources).
    # cmd_vel_relay = Node(
    #     package=nav_pkg, executable='cmd_vel_relay_node', name='cmd_vel_relay',
    #     output='screen', parameters=[{'use_sim_time': True}]
    # )

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        output='screen', parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        declare_merge_mode,
        gazebo,
        TimerAction(period=3.0, actions=[bridge]),
        TimerAction(period=5.0, actions=[spawn_robot1, spawn_robot2]),
        TimerAction(period=8.0, actions=[ekf1, ekf2]),
        TimerAction(period=10.0, actions=[slam1, slam2]),
        TimerAction(period=14.0, actions=[map_merge_known, map_merge_unknown]),
        TimerAction(period=16.0, actions=[frontier, astar, pure_pursuit]),
        TimerAction(period=17.0, actions=[rviz]),
    ])