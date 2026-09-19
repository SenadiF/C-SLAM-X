import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_share = get_package_share_directory('my_navigation')
    costmap_config = os.path.join(pkg_share, 'config', 'explore_lite_costmap.yaml')

    costmap_node = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='global_costmap',
        namespace='robot1',
        output='screen',
        parameters=[costmap_config]
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_costmap',
        namespace='robot1',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['global_costmap']
        }]
    )

    explore_lite = Node(
        package='explore_lite',
        executable='explore',
        name='explore_node',
        namespace='robot1',
        output='screen',
        parameters=[{
            'robot_base_frame': 'robot1/base_link',
            'costmap_topic': 'global_costmap/costmap',
            'costmap_updates_topic': 'global_costmap/costmap_updates',
            'visualize': True,
            'planner_frequency': 0.33,
            'progress_timeout': 30.0,
            'potential_scale': 3.0,
            'orientation_scale': 0.0,
            'gain_scale': 1.0,
            'transform_tolerance': 0.3,
            'min_frontier_size': 0.5,
        }]
    )

    return LaunchDescription([
        costmap_node,
        lifecycle_manager,
        explore_lite,
    ])

