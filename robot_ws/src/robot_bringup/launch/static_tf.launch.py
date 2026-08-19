from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
#Robot 1

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='robot1_base_to_imu',
            arguments=[
                '0',
                '0',
                '0.05',
                '0',
                '0',
                '0',
                'robot1/base_link',
                'robot1/imu_link'
            ]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='robot1_base_to_lidar',
            arguments=[
                '0',
                '0',
                '0.12',
                '0',
                '0',
                '0',
                'robot1/base_link',
                'robot1/lidar_link'
            ]
        ),
#Robot 2

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='robot2_base_to_imu',
            arguments=[
                '0',
                '0',
                '0.05',
                '0',
                '0',
                '0',
                'robot2/base_link',
                'robot2/imu_link'
            ]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='robot2_base_to_lidar',
            arguments=[
                '0',
                '0',
                '0.12',
                '0',
                '0',
                '0',
                'robot2/base_link',
                'robot2/lidar_link'
            ]
        ),

    ])