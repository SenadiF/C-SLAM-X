"""
single_robot_sim.launch.py
============================
Single-robot variant of multi_robot_sim.launch.py, in the SAME two-room
world with the SAME robot model/sensors, for a fair "1 robot vs 2 robots"
exploration-time comparison. No map_merge is needed - there's only one
map - and frontier_explorer/astar_node/pure_pursuit_node already handle a
missing robot2 gracefully (they only act on robot1 in that case), so this
reuses those nodes completely unchanged.

Usage:
  ros2 launch multi_robot_sim single_robot_sim.launch.py

  # Hand-drive a thorough mapping run instead (for a ground-truth map to
  # compare autonomous runs against) - leaves /robot1/cmd_vel free for
  # teleop_twist_keyboard instead of fighting the autonomous nav stack:
  ros2 launch multi_robot_sim single_robot_sim.launch.py manual_control:=true
  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/robot1/cmd_vel
  # ... drive slowly, cover both rooms fully, revisit earlier spots from a
  # different angle a few times so slam_toolbox's loop closure gets a
  # chance to correct drift, then:
  ros2 run nav2_map_server map_saver_cli -f ~/ground_truth_map
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, TimerAction,
    ExecuteProcess,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_share = get_package_share_directory('multi_robot_sim')
    world_path = os.path.join(pkg_share, 'worlds', 'two_room_test.sdf')
    bridge_config = os.path.join(pkg_share, 'config', 'bridge.yaml')

    nav_pkg = 'my_navigation'

    robot1_x, robot1_y, robot1_yaw = '0.0', '0.0', '0.0'

    declare_gui = DeclareLaunchArgument(
        'gui',
        default_value='false',
        description="Launch Gazebo's own GUI client too. Off by default - "
                    "see multi_robot_sim.launch.py for why."
    )
    gui = LaunchConfiguration('gui')

    declare_log_metrics = DeclareLaunchArgument(
        'log_metrics',
        default_value='false',
        description="Also start metrics_logger for robot1, timed to start "
                    "exactly when the nav stack does - see "
                    "multi_robot_sim.launch.py for why this matters."
    )
    log_metrics = LaunchConfiguration('log_metrics')

    declare_strategy_name = DeclareLaunchArgument(
        'strategy_name',
        default_value='single_robot',
        description='Label recorded in the metrics CSV (only used if log_metrics:=true).'
    )
    strategy_name = LaunchConfiguration('strategy_name')

    declare_metrics_csv_dir = DeclareLaunchArgument(
        'metrics_csv_dir',
        default_value='.',
        description='Directory the trial CSV is written to (only used if log_metrics:=true).'
    )
    metrics_csv_dir = LaunchConfiguration('metrics_csv_dir')

    declare_manual_control = DeclareLaunchArgument(
        'manual_control',
        default_value='false',
        description="Skip frontier_explorer/astar/pure_pursuit (and "
                    "metrics_logger) entirely, leaving /robot1/cmd_vel free "
                    "for teleop_twist_keyboard - e.g. to hand-drive a "
                    "thorough, careful mapping run to use as a ground-truth "
                    "map for comparing autonomous runs against. With the "
                    "autonomous stack also running, both would fight over "
                    "cmd_vel."
    )
    manual_control = LaunchConfiguration('manual_control')

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

    # ---- Bridge (only robot1's topics will ever carry data; robot2's
    #      side of the bridge just has no publisher and is harmless) ----
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        arguments=['--ros-args', '-p', f'config_file:={bridge_config}']
    )

    # ---- Spawn robot1 only ----
    spawn_robot1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'spawn_robot.launch.py')
        ),
        launch_arguments={
            'robot_name': 'robot1',
            'x': robot1_x, 'y': robot1_y, 'yaw': robot1_yaw,
        }.items()
    )

    # ---- EKF ----
    ekf1 = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        namespace='robot1',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'ekf1_sim.yaml')]
    )

    # ---- SLAM (unnamespaced, so its default relative "map" topic
    #      resolves straight to the global /map - no merge step needed
    #      or wanted for a single robot) ----
    slam = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            os.path.join(pkg_share, 'config', 'slam_single_sim.yaml'),
            {'use_sim_time': True},
        ],
    )

    lifecycle_manager_slam = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['slam_toolbox'],
        }]
    )

    # ---- Navigation stack: frontier explorer, A*, pure pursuit - same
    #      nodes as the two-robot launch, completely unchanged. They
    #      already skip robot2-specific logic when robot2 never reports
    #      a position. ----
    frontier_explorer = Node(
        package=nav_pkg,
        executable='frontier_explorer_node',
        name='frontier_explorer',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=UnlessCondition(manual_control)
    )

    astar_planner = Node(
        package=nav_pkg,
        executable='astar_node',
        name='astar_planner',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=UnlessCondition(manual_control)
    )

    pure_pursuit = Node(
        package=nav_pkg,
        executable='pure_pursuit_node',
        name='pure_pursuit',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=UnlessCondition(manual_control)
    )

    # ---- RViz ----
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # ---- Metrics logging (optional), synchronized to the nav stack -
    # see multi_robot_sim.launch.py for why this is done here rather than
    # left to manual timing.
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

    return LaunchDescription([
        declare_gui,
        declare_log_metrics,
        declare_strategy_name,
        declare_metrics_csv_dir,
        declare_manual_control,
        gazebo_headless,
        gazebo_gui,
        TimerAction(period=3.0, actions=[bridge]),
        TimerAction(period=5.0, actions=[spawn_robot1]),
        TimerAction(period=8.0, actions=[ekf1]),
        # slam_toolbox's own constructor (loading the Ceres solver plugin,
        # allocating its default map buffer) can take long enough that the
        # lifecycle_manager's configure() service call times out waiting
        # for a response before slam_toolbox is ready to send one - it then
        # never proceeds to activate, and the map silently stays empty
        # forever. Giving slam_toolbox a head start avoids the race.
        TimerAction(period=10.0, actions=[slam]),
        TimerAction(period=12.0, actions=[lifecycle_manager_slam]),
        TimerAction(period=16.0, actions=[
            frontier_explorer, astar_planner, pure_pursuit, metrics_logger_r1,
        ]),
        TimerAction(period=17.0, actions=[rviz]),
    ])
