"""
multi_robot_sim.launch.py
==========================
Full two-robot SLAM + navigation stack test bed, in Gazebo simulation.

Usage:
  # Known initial poses (fast, deterministic baseline)
  ros2 launch multi_robot_sim multi_robot_sim.launch.py merge_mode:=known

  # Unknown poses, multirobot_map_merge's built-in feature matching
  ros2 launch multi_robot_sim multi_robot_sim.launch.py merge_mode:=unknown

  # Unknown poses, your own RANSAC+ICP aligner (no multirobot_map_merge)
  ros2 launch multi_robot_sim multi_robot_sim.launch.py merge_mode:=ransac_icp

Assumes 'multirobot_map_merge', 'slam_toolbox', 'robot_localization',
'ros_gz_sim', 'ros_gz_bridge', and your own navigation package
(frontier_explorer / astar_planner / pure_pursuit / ransac_icp_map_aligner)
are already built in the same workspace.
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, TimerAction,
    ExecuteProcess,
)
from launch.actions import GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_share = get_package_share_directory('multi_robot_sim')
    world_path = os.path.join(pkg_share, 'worlds', 'two_room_test.sdf')
    bridge_config = os.path.join(pkg_share, 'config', 'bridge.yaml')

    # Adjust these to match your actual package names if different.
    ekf_pkg = 'robot_localization_config'
    nav_pkg = 'my_navigation'

    merge_mode = LaunchConfiguration('merge_mode')

    # ---- Robot spawn poses (robot2 offset so there's partial overlap
    #      near the doorway, matching config/map_merge_known_sim.yaml) ----
    robot1_x, robot1_y, robot1_yaw = '0.0', '0.0', '0.0'
    robot2_x, robot2_y, robot2_yaw = '-2.0', '-1.5', '0.0'

    declare_merge_mode = DeclareLaunchArgument(
        'merge_mode',
        default_value='known',
        description='known | unknown | ransac_icp'
    )

    declare_gui = DeclareLaunchArgument(
        'gui',
        default_value='false',
        description="Launch Gazebo's own GUI client too. Off by default: RViz "
                    "already visualizes everything, and Gazebo's GUI adds a "
                    "second Ogre2 rendering load on top of the sensors "
                    "system's - on systems where gz-sim falls back to "
                    "software rendering (see 'failed to create dri2 screen' "
                    "warnings), that's enough to make EKF/SLAM miss their "
                    "real-time deadlines and drop every scan."
    )
    gui = LaunchConfiguration('gui')

    declare_log_metrics = DeclareLaunchArgument(
        'log_metrics',
        default_value='false',
        description="Also start metrics_logger for both robots, timed to "
                    "start exactly when the nav stack does. Manually timing "
                    "an external metrics_logger against this launch's "
                    "staggered startup is unreliable - it's easy to attach "
                    "after exploration has already finished, silently "
                    "capturing nothing but steady-state data."
    )
    log_metrics = LaunchConfiguration('log_metrics')

    declare_strategy_name = DeclareLaunchArgument(
        'strategy_name',
        default_value='two_robot_known',
        description='Label recorded in the metrics CSV (only used if log_metrics:=true).'
    )
    strategy_name = LaunchConfiguration('strategy_name')

    declare_metrics_csv_dir = DeclareLaunchArgument(
        'metrics_csv_dir',
        default_value='.',
        description='Directory the trial CSVs are written to (only used if log_metrics:=true).'
    )
    metrics_csv_dir = LaunchConfiguration('metrics_csv_dir')

    # ---- Gazebo ----
    gazebo_headless = ExecuteProcess(
        cmd=['gz', 'sim', '-s', '-r', world_path],
        output='screen',
        condition=UnlessCondition(gui)
    )
    gazebo_gui = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_path],
        output='screen',
        condition=IfCondition(gui)
    )

    # ---- Bridge (scan/imu/odom/clock/cmd_vel for both robots) ----
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        arguments=['--ros-args', '-p', f'config_file:={bridge_config}']
    )

    # ---- Spawn both robots ----
    spawn_robot1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'spawn_robot.launch.py')
        ),
        launch_arguments={
            'robot_name': 'robot1',
            'x': robot1_x, 'y': robot1_y, 'yaw': robot1_yaw,
        }.items()
    )

    spawn_robot2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'spawn_robot.launch.py')
        ),
        launch_arguments={
            'robot_name': 'robot2',
            'x': robot2_x, 'y': robot2_y, 'yaw': robot2_yaw,
        }.items()
    )

    # ---- EKF x2 ----
    ekf1 = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        namespace='robot1',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'ekf1_sim.yaml')]
    )

    ekf2 = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        namespace='robot2',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'ekf2_sim.yaml')]
    )

    # ---- SLAM x2 ----
    # Mirrors robot_localization_config/launch/slam.launch.py (the proven
    # hardware pattern): slam_toolbox publishes its map on an absolute /map
    # topic internally regardless of node namespace, so without the explicit
    # remapping here both instances would collide on the same /map topic
    # even though the nodes themselves are properly namespaced. Remapping to
    # the relative 'map' topic moves each one onto /robot1/map, /robot2/map,
    # matching multirobot_map_merge's <namespace>/<robot_map_topic> discovery.
    slam1 = GroupAction([
        PushRosNamespace('robot1'),
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                os.path.join(pkg_share, 'config', 'slam1_sim.yaml'),
                {'use_sim_time': True},
            ],
            remappings=[
                ('/map', 'map'),
                ('/map_metadata', 'map_metadata'),
            ]
        ),
    ])

    slam2 = GroupAction([
        PushRosNamespace('robot2'),
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                os.path.join(pkg_share, 'config', 'slam2_sim.yaml'),
                {'use_sim_time': True},
            ],
            remappings=[
                ('/map', 'map'),
                ('/map_metadata', 'map_metadata'),
            ]
        ),
    ])

    # Started a couple seconds after the slam nodes above (see the
    # TimerAction spacing below) rather than in the same GroupAction:
    # slam_toolbox's own constructor (loading the Ceres solver plugin,
    # allocating its default map buffer) can take long enough that the
    # lifecycle_manager's configure() service call times out waiting for a
    # response before slam_toolbox is ready to send one - it then never
    # proceeds to activate, and the map silently stays empty forever.
    lifecycle1 = GroupAction([
        PushRosNamespace('robot1'),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_slam',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': ['slam_toolbox'],
            }]
        ),
    ])

    lifecycle2 = GroupAction([
        PushRosNamespace('robot2'),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_slam',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': ['slam_toolbox'],
            }]
        ),
    ])

    # ---- Map merge: known-pose OR feature-matching mode ----
    map_merge_known = Node(
        package='multirobot_map_merge',
        executable='map_merge',
        name='map_merge',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'map_merge_known_sim.yaml')],
        condition=IfCondition(PythonExpression(["'", merge_mode, "' == 'known'"]))
    )

    map_merge_unknown = Node(
        package='multirobot_map_merge',
        executable='map_merge',
        name='map_merge',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'map_merge_unknown_sim.yaml')],
        condition=IfCondition(PythonExpression(["'", merge_mode, "' == 'unknown'"]))
    )

    # ---- Your own RANSAC+ICP aligner, as the third comparison method ----
    ransac_icp_aligner = Node(
        package=nav_pkg,
        executable='ransac_icp_map_aligner',
        name='ransac_icp_map_aligner',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(PythonExpression(["'", merge_mode, "' == 'ransac_icp'"]))
    )

    # ---- Navigation stack: frontier explorer, A*, pure pursuit ----
    frontier_explorer = Node(
        package=nav_pkg,
        executable='frontier_explorer_node',
        name='frontier_explorer',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    astar_planner = Node(
        package=nav_pkg,
        executable='astar_node',
        name='astar_planner',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    pure_pursuit = Node(
        package=nav_pkg,
        executable='pure_pursuit_node',
        name='pure_pursuit',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # ---- RViz ----
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # ---- Metrics logging (optional) - started in the SAME TimerAction as
    # the nav stack below, so it's always synchronized to exploration
    # actually starting, with no manual timing guesswork required.
    metrics_logger_r1 = Node(
        package=nav_pkg,
        executable='metrics_logger',
        name='metrics_logger_robot1',
        output='screen',
        parameters=[{
            'robot_name': 'robot1',
            'strategy_name': strategy_name,
            'output_csv': PythonExpression(
                ["'", metrics_csv_dir, "/", strategy_name, "_robot1.csv'"]
            ),
        }],
        condition=IfCondition(log_metrics)
    )

    metrics_logger_r2 = Node(
        package=nav_pkg,
        executable='metrics_logger',
        name='metrics_logger_robot2',
        output='screen',
        parameters=[{
            'robot_name': 'robot2',
            'strategy_name': strategy_name,
            'output_csv': PythonExpression(
                ["'", metrics_csv_dir, "/", strategy_name, "_robot2.csv'"]
            ),
        }],
        condition=IfCondition(log_metrics)
    )

    return LaunchDescription([
        declare_merge_mode,
        declare_gui,
        declare_log_metrics,
        declare_strategy_name,
        declare_metrics_csv_dir,
        gazebo_headless,
        gazebo_gui,
        TimerAction(period=3.0, actions=[bridge]),
        TimerAction(period=5.0, actions=[spawn_robot1, spawn_robot2]),
        TimerAction(period=8.0, actions=[ekf1, ekf2]),
        TimerAction(period=10.0, actions=[slam1, slam2]),
        TimerAction(period=12.0, actions=[lifecycle1, lifecycle2]),
        TimerAction(period=14.0, actions=[
            map_merge_known, map_merge_unknown, ransac_icp_aligner
        ]),
        TimerAction(period=16.0, actions=[
            frontier_explorer, astar_planner, pure_pursuit,
            metrics_logger_r1, metrics_logger_r2,
        ]),
        TimerAction(period=17.0, actions=[rviz]),
    ])
