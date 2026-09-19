import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_share = get_package_share_directory('robot_localization_config')
    ekf_config = os.path.join(package_share, 'config', 'ekf.yaml')

    namespace = LaunchConfiguration('namespace')

    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='robot1',
        description='Which robot to launch the EKF node for: robot1 or robot2. '
                    'ekf.yaml has parameter blocks for both under their own '
                    'namespace key, so run this once per robot.'
    )

    return LaunchDescription([
        declare_namespace,
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            namespace=namespace,
            output='screen',
            parameters=[ekf_config, {'use_sim_time': False}],
        ),
    ])