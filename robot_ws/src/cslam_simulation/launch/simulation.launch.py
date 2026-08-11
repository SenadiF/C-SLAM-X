
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    turtlebot3_gazebo_dir = get_package_share_directory(
        'turtlebot3_gazebo'
    )

    cslam_simulation_dir = get_package_share_directory(
        'cslam_simulation'
    )

   
    robot1_model_path = os.path.join(
        cslam_simulation_dir,
        'models',
        'turtlebot3_robot1',
        'model.sdf'
    )

    
    robot2_model_path = os.path.join(
        cslam_simulation_dir,
        'models',
        'turtlebot3_robot2',
        'model.sdf'
    )


    world_path = os.path.join(
        turtlebot3_gazebo_dir,
        'worlds',
        'turtlebot3_world.world'
    )



    urdf_path = os.path.join(
        turtlebot3_gazebo_dir,
        'urdf',
        'turtlebot3_burger.urdf'
    )

    with open(urdf_path, 'r') as f:
        robot_description = f.read()


    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': world_path + ' -r'
        }.items()
    )


    robot1_spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'robot1',
            '-file', robot1_model_path,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.01'
        ],
        output='screen'
    )


    robot2_spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'robot2',
            '-file', robot2_model_path,
            '-x', '1.5',
            '-y', '0.0',
            '-z', '0.01'
        ],
        output='screen'
    )

    robot1_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',

        namespace='robot1',
        name='robot_state_publisher',

        output='screen',

        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
            'frame_prefix': 'robot1/'
        }]
    )
    robot2_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',

        namespace='robot2',
        name='robot_state_publisher',

        output='screen',

        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
            'frame_prefix': 'robot2/'
        }]
    )


    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',

        name='cslam_gz_bridge',

        output='screen',

        arguments=[

            # Odometry
            '/robot1/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',

            # Velocity command
            '/robot1/cmd_vel@geometry_msgs/msg/TwistStamped]gz.msgs.Twist',

            # IMU
            '/robot1/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',

            # Laser scan
            '/robot1/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',

            # Joint states
            '/robot1/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',


            # Odometry
            '/robot2/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',

            # Velocity command
            '/robot2/cmd_vel@geometry_msgs/msg/TwistStamped]gz.msgs.Twist',

            # IMU
            '/robot2/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',

            # Laser scan
            '/robot2/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',

            # Joint states
            '/robot2/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',



            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ]
    )

    return LaunchDescription([

        # Gazebo
        gazebo,

        # Robots
        robot1_spawn,
        robot2_spawn,

        # TF / robot descriptions
        robot1_state_publisher,
        robot2_state_publisher,

        # Gazebo <--> ROS bridge
        bridge
    ])

