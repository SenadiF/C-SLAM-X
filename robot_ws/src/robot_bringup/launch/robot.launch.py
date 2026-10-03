from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('robot_localization_config')

    #  micro-ROS Agent 
    
    microros_agent = ExecuteProcess(
        cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 'udp4', '--port', '8888'],
        output='screen'
    )

    bringup_dir = get_package_share_directory('robot_bringup')

    # Static transforms for both robots' sensor mounting offsets - this
    # file already existed with the real measured hardware values (0.12m
    # lidar height) but was never actually included in this launch file.
    static_transforms = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'static_tf.launch.py')
        )
    )

    #  Wheel Odometry Nodes
    wheel_odom_node = Node(
        package='robot_odometry',
        executable='wheel_odometry_node',
        name='wheel_odometry_node',
        namespace='robot1',
        output='screen',
        parameters=[{'robot_name': 'robot1'}],
    )
    wheel_odom_node2 = Node(
        package='robot_odometry',
        executable='wheel_odometry_node',
        name='wheel_odometry_node',
        namespace='robot2',
        output='screen',
        parameters=[{'robot_name': 'robot2'}],
    )

    #  Robot Localization
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        namespace='robot1',
        output='screen',
        parameters=[os.path.join(pkg_dir, 'config', 'ekf.yaml')]
    )
    ekf_node2 = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        namespace='robot2',
        output='screen',
        parameters=[os.path.join(pkg_dir, 'config', 'ekf.yaml')]
    )

    # SLAM Toolbox Nodes - remapped off the absolute /map and /map_metadata
    # topics slam_toolbox publishes internally regardless of node namespace;
    # without this both robots' SLAM instances collide on the same /map
    # topic (the exact bug this workspace already hit and fixed in sim).
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='robot1',
        output='screen',
        parameters=[{
            'odom_frame': 'robot1/odom',
            'base_frame': 'robot1/base_link',
            'map_frame': 'map',
            'scan_topic': 'scan_raw',
            'use_scan_matching': True,
            'minimum_travel_distance': 0.1,
            'minimum_travel_heading': 0.1,
        }],
        remappings=[('/map', 'map'), ('/map_metadata', 'map_metadata')],
    )
    slam_node2 = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='robot2',
        output='screen',
        parameters=[{
            'odom_frame': 'robot2/odom',
            'base_frame': 'robot2/base_link',
            'map_frame': 'map',
            'scan_topic': 'scan_raw',
            'use_scan_matching': True,
            'minimum_travel_distance': 0.1,
            'minimum_travel_heading': 0.1,
        }],
        remappings=[('/map', 'map'), ('/map_metadata', 'map_metadata')],
    )

    # Map merge - combines /robot1/map and /robot2/map into one shared /map
    # using the known relative pose set in map_merge_known.yaml. Update
    # that file's init_pose values to match wherever you actually place
    # the two robots before launching.
    map_merge_node = Node(
        package='multirobot_map_merge',
        executable='map_merge',
        name='map_merge',
        output='screen',
        parameters=[os.path.join(bringup_dir, 'config', 'map_merge_known.yaml')],
    )

    # Proposed navigation stack - frontier detection, A* planning, pure
    # pursuit following. These already use /map (merged) and
    # /robotN/odometry/filtered (the EKF's default output topic, no remap
    # needed), matching the sim setup exactly - except pure_pursuit_node
    # subscribes to '/robotN/scan', which needs the same scan_raw remap
    # as everything else here.
    frontier_explorer = Node(
        package='my_navigation',
        executable='frontier_explorer_node',
        name='frontier_explorer',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )
    astar_planner = Node(
        package='my_navigation',
        executable='astar_node',
        name='astar_planner',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )
    pure_pursuit = Node(
        package='my_navigation',
        executable='pure_pursuit_node',
        name='pure_pursuit',
        output='screen',
        parameters=[{'use_sim_time': False}],
        remappings=[
            ('/robot1/scan', '/robot1/scan_raw'),
            ('/robot2/scan', '/robot2/scan_raw'),
        ],
    )

    # rviz2 Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_dir, 'config', 'robot1_view.rviz')]
    )

    return LaunchDescription([
        microros_agent,
        TimerAction(period=1.0, actions=[static_transforms]),
        TimerAction(period=3.0, actions=[wheel_odom_node, wheel_odom_node2]),
        TimerAction(period=4.0, actions=[ekf_node, ekf_node2]),
        TimerAction(period=7.0, actions=[slam_node, slam_node2]),
        TimerAction(period=9.0, actions=[map_merge_node]),
        TimerAction(period=12.0, actions=[frontier_explorer, astar_planner, pure_pursuit]),
        TimerAction(period=15.0, actions=[rviz_node]),
    ])