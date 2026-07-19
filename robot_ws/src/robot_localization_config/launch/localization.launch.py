from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    namespace = LaunchConfiguration('namespace')

    package_share = get_package_share_directory(
        'robot_localization_config'
    )

    ekf_config = os.path.join(
        package_share,
        'config',
        'ekf.yaml'
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'namespace',
            default_value='robot1'
        ),
        Node(
    package='robot_localization',
    executable='ekf_node',
    namespace=namespace,
    parameters=[
        ekf_config,
        {
            'tf_prefix': namespace
        }
    ]
)  
        

    ])