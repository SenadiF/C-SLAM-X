from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    namespace = LaunchConfiguration('namespace')


    return LaunchDescription([

        DeclareLaunchArgument(
            'namespace',
            default_value='robot1'
        ),


        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            namespace=namespace,
            output='screen',
            parameters=[
                'config/ekf.yaml'
            ]
        )

    ])