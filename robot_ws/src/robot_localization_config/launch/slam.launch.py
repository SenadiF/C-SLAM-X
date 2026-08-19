import os

from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import GroupAction
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_share = get_package_share_directory(
        'robot_localization_config'
    )

    slam_config_robot1 = os.path.join(
        pkg_share,
        'config',
        'slam1.yaml'
    )

    slam_config_robot2 = os.path.join(
        pkg_share,
        'config',
        'slam2.yaml'
    )

    return LaunchDescription([
        #Robot 1 

        GroupAction(
            actions=[

                PushRosNamespace('robot1'),

                Node(
                    package='slam_toolbox',
                    executable='sync_slam_toolbox_node',
                    name='slam_toolbox',
                    output='screen',

                    parameters=[
                        slam_config_robot1
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
        ),
        #Robot 2 

        GroupAction(
            actions=[

                PushRosNamespace('robot2'),

                Node(
                    package='slam_toolbox',
                    executable='sync_slam_toolbox_node',
                    name='slam_toolbox',
                    output='screen',

                    parameters=[
                        slam_config_robot2
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
    ])