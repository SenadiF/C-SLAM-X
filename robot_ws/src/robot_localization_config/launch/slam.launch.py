import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    namespace = LaunchConfiguration('namespace')

    pkg_share = get_package_share_directory(
        'robot_localization_config'
    )

    slam_config = os.path.join(
        pkg_share,
        'config',
        'slam.yaml'
    )
    return LaunchDescription([

    DeclareLaunchArgument(
        'namespace',
        default_value='robot1',
        description='Robot namespace'
    ),

    Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        namespace=namespace,
        output='screen',
        parameters=[
            slam_config
        ]
    ),

    Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        namespace=namespace,
        output='screen',
        parameters=[
            {
                'autostart': True,
                'node_names': [
                    'slam_toolbox'
                ]
            }
        ]
    )

])