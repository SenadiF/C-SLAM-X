import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    os.environ['TURTLEBOT3_MODEL'] = 'burger'

    turtlebot3_gazebo = get_package_share_directory(
        'turtlebot3_gazebo'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                turtlebot3_gazebo,
                'launch',
                'turtlebot3_world.launch.py'
            )
        )
    )

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',

        parameters=[
            {
                'use_sim_time': True,

                'odom_frame': 'odom',
                'map_frame': 'map',
                'base_frame': 'base_link',

                'scan_topic': '/robot1/scan',

                'mode': 'mapping'
            }
        ],

        remappings=[
            ('/scan', '/robot1/scan'),
            ('/odom', '/robot1/odom')
        ]
    )
    scan_relay = Node(
        package='topic_tools',
        executable='relay',
        name='scan_relay',
        output='screen',

        arguments=[
            '/scan',
            '/robot1/scan'
        ]
    )

    odom_relay = Node(
        package='topic_tools',
        executable='relay',
        name='odom_relay',
        output='screen',

        arguments=[
            '/odom',
            '/robot1/odometry/filtered'
        ]
    )
    frontier_explorer = Node(
    package='my_navigation',
    executable='frontier_explorer_node',
    name='frontier_explorer',
    output='screen',

    parameters=[
        {
            'use_sim_time': True,

            'frontier_cluster_distance': 0.15,

            'minimum_frontier_distance': 0.10,

            'queue_size': 1
        }
    ]
)






    astar = Node(
        package='my_navigation',
        executable='astar_node',
        name='astar_planner',
        output='screen',

        parameters=[
            {
                'use_sim_time': True
            }
        ]
    )

    pure_pursuit = Node(
        package='my_navigation',
        executable='pure_pursuit_node',
        name='pure_pursuit',
        output='screen',

        parameters=[
            {
                'use_sim_time': True,

                'lookahead_distance': 0.30,

                'linear_speed': 0.15,

                'max_angular_speed': 1.5,

                'goal_tolerance': 0.05,

                'minimum_linear_speed': 0.05,

                'safe_distance': 0.30
            }
        ]
    )

    cmd_vel_relay = Node(
        package='my_navigation',
        executable='cmd_vel_relay_node',
        name='cmd_vel_relay',
        output='screen'
    )
    return LaunchDescription([

        gazebo,

        scan_relay,

        odom_relay,

        slam_toolbox,

        frontier_explorer,

        astar,

        pure_pursuit,

        cmd_vel_relay

    ])