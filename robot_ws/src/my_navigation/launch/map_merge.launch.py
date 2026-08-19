import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory("my_navigation"),
        "config",
        "map_merge_params.yaml"
    )

    namespace = LaunchConfiguration("namespace")
    known_init_poses = LaunchConfiguration("known_init_poses")

    declare_namespace_argument = DeclareLaunchArgument(
        "namespace",
        default_value="/",
        description="Namespace for the map merge node",
    )

    declare_known_init_poses_argument = DeclareLaunchArgument(
        "known_init_poses",
        default_value="false",
        description="Known initial poses of the robots",
    )

    
    remappings = [
        ("/tf", "tf"),
        ("/tf_static", "tf_static")
    ]

    node = Node(
        package="multirobot_map_merge",
        name="map_merge",
        namespace=namespace,
        executable="map_merge",
        parameters=[
            config,
            {
                "use_sim_time": False,
                "known_init_poses": known_init_poses,
            }
        ],
        output="screen",
        remappings=remappings,
    )

    ld.add_action(declare_known_init_poses_argument)
    ld.add_action(declare_namespace_argument)
    ld.add_action(node)

    return ld