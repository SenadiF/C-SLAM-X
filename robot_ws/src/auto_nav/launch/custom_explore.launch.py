from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_name', default_value='robot1'),

        Node(
            package='auto_nav',
            executable='frontier_explorer',
            name='frontier_explorer',
            output='screen',
            parameters=[{'robot_name': LaunchConfiguration('robot_name')}]
        ),
        Node(
            package='auto_nav',
            executable='astar_planner',
            name='astar_planner',
            output='screen',
            parameters=[{'robot_name': LaunchConfiguration('robot_name')}]
        ),
        Node(
            package='auto_nav',
            executable='pure_pursuit',
            name='pure_pursuit',
            output='screen',
            parameters=[{'robot_name': LaunchConfiguration('robot_name')}]
        ),
    ])