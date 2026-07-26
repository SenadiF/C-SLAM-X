from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('robot_localization_config')  
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    #  micro-ROS Agent 
    
    microros_agent = ExecuteProcess(
        cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 'udp4', '--port', '8888'],
        output='screen'
    )

    #  Wheel Odometry Node 
    wheel_odom_node = Node(
        package='robot_odometry',            
        executable='wheel_odometry_node',
        name='wheel_odometry_node',
        namespace='robot1',
        output='screen'
    )

    #  Robot Localization
    ekf_node = Node(
        package='robot_localization_config',
        executable='ekf_node',
        name='ekf_node',
        namespace='robot1',
        output='screen',
        parameters=[os.path.join(pkg_dir, 'config', 'ekf.yaml')]
    )

    # SLAM Toolbox Node
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
            'scan_topic': 'scan',
            'use_scan_matching': True,
            'minimum_travel_distance': 0.1,
            'minimum_travel_heading': 0.1,
        }]
    )

    # Nav2
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'namespace': 'robot1',
            'params_file': os.path.join(pkg_dir, 'config', 'nav2_params.yaml'),
            'use_sim_time': 'false',
        }.items()
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
        TimerAction(period=3.0, actions=[wheel_odom_node]),
        TimerAction(period=4.0, actions=[ekf_node]),
        TimerAction(period=7.0, actions=[slam_node]),
        TimerAction(period=10.0, actions=[navigation]),
        TimerAction(period=13.0, actions=[rviz_node]),
    ])