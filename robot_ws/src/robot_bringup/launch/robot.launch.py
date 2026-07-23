from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    namespace = LaunchConfiguration('namespace')
    robot_name = LaunchConfiguration('robot_name')


    # Micro ROS agent
    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        arguments=[
            'udp4',
            '--port',
            '8888'
        ],
        output='screen'
    )


    # Wheel odometry
    wheel_odom_node = Node(
        package='robot_odometry',
        executable='wheel_odometry_node',
        name='wheel_odometry',
        namespace=namespace,
        output='screen',
        parameters=[
            {
                'robot_name': robot_name
            }
        ]
    )


    # EKF Localization
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        namespace=namespace,
        parameters=[
            '/home/senadi/Desktop/C-SLAM-X/robot_ws/src/robot_localization_config/config/ekf.yaml',
            {
                'tf_prefix': namespace
            }
        ],
        output='screen'
    )


    # Lidar
    lidar_node = Node(
        package='ldlidar_ros2',
        executable='ldlidar_ros2_node',
        namespace=namespace,
        parameters=[
            {
                'product_name':'LDLiDAR_LD19',

                'laser_scan_topic_name':'scan',

                'point_cloud_2d_topic_name':'pointcloud2d',

                'frame_id':'base_laser',

                'laser_scan_dir':True,

                'enable_angle_crop_func':False,

                'range_min':0.02,

                'range_max':12.0
            }
        ],
        output='screen'
    )


    # Lidar TF
    lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        namespace=namespace,
        arguments=[
            '0',
            '0',
            '0.18',
            '0',
            '0',
            '0',
            'base_link',
            'base_laser'
        ]
    )


    return LaunchDescription([

        DeclareLaunchArgument(
            'namespace',
            default_value='robot1'
        ),

        DeclareLaunchArgument(
            'robot_name',
            default_value='robot1'
        ),

        micro_ros_agent,
        wheel_odom_node,
        ekf_node,
        lidar_node,
        lidar_tf

    ])