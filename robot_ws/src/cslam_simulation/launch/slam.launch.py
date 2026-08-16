import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():

    pkg_share = get_package_share_directory(
        'cslam_simulation'
    )

    slam1_config = os.path.join(
        pkg_share,
        'config',
        'slam_robot1.yaml'
    )

    slam2_config = os.path.join(
        pkg_share,
        'config',
        'slam_robot2.yaml'
    )

    robot1 = GroupAction(
        actions=[

            PushRosNamespace('robot1'),

            Node(
                package='slam_toolbox',
                executable='sync_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[
                    slam1_config
                ],
                remappings=[
                    ('/map', 'map'),
                    ('/map_metadata', 'map_metadata'),
                    ('/map_updates', 'map_updates'),
                ]
            ),

            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_slam',
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
        ]
    )

    robot2 = GroupAction(
        actions=[

            PushRosNamespace('robot2'),

            Node(
                package='slam_toolbox',
                executable='sync_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[
                    slam2_config
                ],
                remappings=[
                    ('/map', 'map'),
                    ('/map_metadata', 'map_metadata'),
                    ('/map_updates', 'map_updates'),
                ]
            ),

            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_slam',
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
        ]
    )

    return LaunchDescription([
        robot1,
        robot2
    ])