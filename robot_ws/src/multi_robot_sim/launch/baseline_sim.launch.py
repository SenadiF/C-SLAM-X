
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_share = get_package_share_directory('multi_robot_sim')
    world_path = os.path.join(pkg_share, 'worlds', 'two_room_test.sdf')
    bridge_config = os.path.join(pkg_share, 'config', 'bridge_baseline.yaml')



    baseline_pkg = 'ss'

    robot1_x, robot1_y, robot1_yaw = '0.0', '0.0', '0.0'
    # robot_coordinator's own safety check (min_robot_distance: 1.5) treats
    # anything closer than 1.5m as an emergency and zeroes both robots'
    # cmd_vel - the previous (1.0, 0.0) spawn was only 1.0m from robot1,
    # closer than that threshold, causing intermittent stuck/deadlock
    # right from t=0. This spawn is ~2.5m away instead. If you change this,
    # ss/frames.py's SPAWN_POSES['robot2'] must be updated to match exactly
    # - it's a separate hardcoded copy of this same assumption, used to
    # convert robot2's local odometry into the shared world frame.
    robot2_x, robot2_y, robot2_yaw = '-2.0', '-1.5', '3.14159'

    declare_gui = DeclareLaunchArgument(
        'gui',
        default_value='false',
        description="Launch Gazebo's own GUI client too. Off by default: "
                    "RViz already visualizes everything, and on systems "
                    "where gz-sim falls back to software rendering (see "
                    "'failed to create dri2 screen' warnings), the extra "
                    "GUI rendering load can be enough to make real-time "
                    "nodes miss their deadlines."
    )
    gui = LaunchConfiguration('gui')

    declare_log_metrics = DeclareLaunchArgument(
        'log_metrics',
        default_value='false',
        description="Also start metrics_logger for both robots, timed to "
                    "start exactly when the nav stack does - see "
                    "multi_robot_sim.launch.py for why this matters. This "
                    "stack has no EKF, so odom_topic is hardcoded to 'odom' "
                    "here rather than the my_navigation-stack default of "
                    "'odometry/filtered'."
    )
    log_metrics = LaunchConfiguration('log_metrics')

    declare_strategy_name = DeclareLaunchArgument(
        'strategy_name',
        default_value='ss_reactive',
        description='Label recorded in the metrics CSV (only used if log_metrics:=true).'
    )
    strategy_name = LaunchConfiguration('strategy_name')

    declare_metrics_csv_dir = DeclareLaunchArgument(
        'metrics_csv_dir',
        default_value='.',
        description='Directory the trial CSVs are written to (only used if log_metrics:=true).'
    )
    metrics_csv_dir = LaunchConfiguration('metrics_csv_dir')

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

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        arguments=['--ros-args', '-p', f'config_file:={bridge_config}']
    )

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

    multirobot_slam = Node(
        package=baseline_pkg,
        executable='multirobot_slam_node',
        name='multi_robot_slam',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    robot_coordinator = Node(
        package=baseline_pkg,
        executable='robot_coordinator_node',
        name='robot_coordinator',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # robot_controller.py is a single reusable node parameterized by
    # 'robot_name' - launch one instance per robot, each subscribing to
    # its own /<robot_name>/scan, /<robot_name>/odom, /<robot_name>/goal_pose
    # and publishing /<robot_name>/cmd_vel.
    robot1_controller = Node(
        package=baseline_pkg,
        executable='robot_controller_node',
        name='robot_controller_robot1',
        output='screen',
        parameters=[{'use_sim_time': True, 'robot_name': 'robot1'}]
    )

    robot2_controller = Node(
        package=baseline_pkg,
        executable='robot_controller_node',
        name='robot_controller_robot2',
        output='screen',
        parameters=[{'use_sim_time': True, 'robot_name': 'robot2'}]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    metrics_logger_r1 = Node(
        package='my_navigation',
        executable='metrics_logger',
        name='metrics_logger_robot1',
        output='screen',
        parameters=[{
            'robot_name': 'robot1',
            'strategy_name': strategy_name,
            'odom_topic': 'odom',
            'output_csv': PythonExpression(
                ["'", metrics_csv_dir, "/", strategy_name, "_robot1.csv'"]
            ),
        }],
        condition=IfCondition(log_metrics)
    )

    metrics_logger_r2 = Node(
        package='my_navigation',
        executable='metrics_logger',
        name='metrics_logger_robot2',
        output='screen',
        parameters=[{
            'robot_name': 'robot2',
            'strategy_name': strategy_name,
            'odom_topic': 'odom',
            'output_csv': PythonExpression(
                ["'", metrics_csv_dir, "/", strategy_name, "_robot2.csv'"]
            ),
        }],
        condition=IfCondition(log_metrics)
    )

    return LaunchDescription([
        declare_gui,
        declare_log_metrics,
        declare_strategy_name,
        declare_metrics_csv_dir,
        gazebo_headless,
        gazebo_gui,
        TimerAction(period=3.0, actions=[bridge]),
        TimerAction(period=5.0, actions=[spawn_robot1, spawn_robot2]),
        TimerAction(period=8.0, actions=[multirobot_slam]),
        TimerAction(period=9.0, actions=[
            robot_coordinator, robot1_controller, robot2_controller,
            metrics_logger_r1, metrics_logger_r2,
        ]),
        TimerAction(period=11.0, actions=[rviz]),
    ])