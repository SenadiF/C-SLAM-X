from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('robot_localization_config')
    params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'nav2_navigation_no_dock.launch.py')
            ),
            launch_arguments={
                'namespace': 'robot1',
                'params_file': params_file,
                'use_sim_time': 'false',
                'autostart': 'true',
                
            }.items()
        ),
    ])