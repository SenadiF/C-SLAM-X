import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_share = get_package_share_directory('multi_robot_sim')
    xacro_path = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')

    robot_name = LaunchConfiguration('robot_name')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    yaw = LaunchConfiguration('yaw')

    robot_description = Command([
        'xacro ', xacro_path, ' robot_name:=', robot_name
    ])

    return LaunchDescription([
        DeclareLaunchArgument('robot_name', default_value='robot1'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=robot_name,
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True,
                'frame_prefix': ''  # frame names already prefixed inside the xacro itself
            }]
        ),

        Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            arguments=[
                '-name', robot_name,
                '-topic', [robot_name, '/robot_description'],
                '-x', x,
                '-y', y,
                '-Y', yaw,
                '-z', '0.05',
            ]
        ),

        # gz-sim converts our fixed lidar/imu joints into SDF <frame> elements
        # (not real links), so its sensor plugins auto-generate a frame_id off
        # the merged base_link instead of the actual lidar_link/imu_link -
        # and since our link names already contain "<robot>/", gz-sim's own
        # "/"-joining doubles it into garbage like
        # "robot1/robot1/base_link/robot1_lidar", a TF frame that never
        # otherwise exists. These give that exact (deterministic) frame a
        # real place in the tree at the sensor's true offset from base_link,
        # so message filters (slam_toolbox, rviz) can resolve it instead of
        # waiting forever and dropping every scan.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_frame_fixup',
            namespace=robot_name,
            output='screen',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0.08',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', [robot_name, '/base_link'],
                '--child-frame-id', [robot_name, '/', robot_name, '/base_link/', robot_name, '_lidar'],
            ]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_frame_fixup',
            namespace=robot_name,
            output='screen',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0.05',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', [robot_name, '/base_link'],
                '--child-frame-id', [robot_name, '/', robot_name, '/base_link/', robot_name, '_imu'],
            ]
        ),
    ])
