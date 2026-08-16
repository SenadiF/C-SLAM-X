
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ============================================================
    # TURTLEBOT3 MODEL
    # ============================================================

    os.environ['TURTLEBOT3_MODEL'] = 'burger'

    turtlebot3_gazebo = get_package_share_directory(
        'turtlebot3_gazebo'
    )

    # ============================================================
    # GAZEBO WORLD
    # ============================================================

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                turtlebot3_gazebo,
                'launch',
                'turtlebot3_world.launch.py'
            )
        )
    )

    # ============================================================
    # ROBOT 1
    # ============================================================

    robot1 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_robot1',

        arguments=[
            '-entity',
            'robot1',

            '-file',
            os.path.join(
                os.environ['TURTLEBOT3_MODEL'],
                'burger'
            )
        ],

        output='screen'
    )

    # ============================================================
    # ROBOT 2
    # ============================================================

    robot2 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_robot2',

        arguments=[
            '-entity',
            'robot2',

            '-file',
            os.path.join(
                os.environ['TURTLEBOT3_MODEL'],
                'burger'
            )
        ],

        output='screen'
    )

    return LaunchDescription([

        gazebo,

        robot1,

        robot2

    ])

