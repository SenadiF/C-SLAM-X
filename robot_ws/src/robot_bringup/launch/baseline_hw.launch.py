"""
baseline_hw.launch.py
======================
Hardware version of multi_robot_sim/launch/baseline_sim.launch.py - runs the
'ss' reactive baseline stack (multirobot_slam, robot_coordinator,
robot_controller x2) against the two real ESP32 robots instead of Gazebo,
for the same baseline-vs-proposed comparison as the sim results, now on
hardware.

IMPORTANT: ss/frames.py's SPAWN_POSES dict is hardcoded to the sim's
baseline spawn layout (robot1 at origin, robot2 at (-2.0, -1.5, pi rad)).
Update SPAWN_POSES in that file to match wherever you actually place the
two robots before running this - it converts each robot's own local wheel
odometry into the shared world frame multirobot_slam/robot_coordinator
need, and a wrong value here will misplace robot2 in the merged map by
exactly the error in that guess.

The 'ss' stack has no EKF and no scan-matching SLAM of its own - it builds
its occupancy grid directly from raw wheel odometry + LiDAR, which is the
whole point of it being the "reactive baseline" being compared against.
So this launch deliberately does NOT include static_tf/EKF/slam_toolbox -
only bare wheel odometry, remapped to look like the 'odom'/'scan' topics
the ss nodes hardcode.

Usage:
  ros2 launch robot_bringup baseline_hw.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    baseline_pkg = 'ss'

    # ---- micro-ROS agent (both boards share one, see project notes on
    #      client-key collisions if you ever see cross-talk again) ----
    microros_agent = ExecuteProcess(
        cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 'udp4', '--port', '8888'],
        output='screen'
    )

    # ---- Wheel odometry (raw, no EKF - ss does its own world-frame
    #      conversion via frames.py, matching the sim's raw Gazebo odom) ----
    # robot_name defaults to 'robot1' and is used to build an ABSOLUTE
    # topic string (f'/{robot_name}/wheel_odom'), not a relative one - so
    # it must be set explicitly per instance (namespace= alone does not
    # affect it), and the remap must target that exact absolute name.
    wheel_odom_r1 = Node(
        package='robot_odometry',
        executable='wheel_odometry_node',
        name='wheel_odometry_node',
        namespace='robot1',
        output='screen',
        parameters=[{'robot_name': 'robot1'}],
        remappings=[('/robot1/wheel_odom', '/robot1/odom')],
    )
    wheel_odom_r2 = Node(
        package='robot_odometry',
        executable='wheel_odometry_node',
        name='wheel_odometry_node',
        namespace='robot2',
        output='screen',
        parameters=[{'robot_name': 'robot2'}],
        remappings=[('/robot2/wheel_odom', '/robot2/odom')],
    )

    # ---- ss stack - multirobot_slam and robot_coordinator hardcode
    #      absolute '/robot1/scan' and '/robot1/odom' (not 'scan_raw'),
    #      so remap those absolute names directly. ----
    multirobot_slam = Node(
        package=baseline_pkg,
        executable='multirobot_slam_node',
        name='multi_robot_slam',
        output='screen',
        parameters=[{'use_sim_time': False}],
        remappings=[
            ('/robot1/scan', '/robot1/scan_raw'),
            ('/robot2/scan', '/robot2/scan_raw'),
            ('/robot1/odom', '/robot1/wheel_odom'),
            ('/robot2/odom', '/robot2/wheel_odom'),
        ],
    )

    robot_coordinator = Node(
        package=baseline_pkg,
        executable='robot_coordinator_node',
        name='robot_coordinator',
        output='screen',
        parameters=[{'use_sim_time': False}],
        remappings=[
            ('/robot1/scan', '/robot1/scan_raw'),
            ('/robot2/scan', '/robot2/scan_raw'),
            ('/robot1/odom', '/robot1/wheel_odom'),
            ('/robot2/odom', '/robot2/wheel_odom'),
        ],
    )

    robot1_controller = Node(
        package=baseline_pkg,
        executable='robot_controller_node',
        name='robot_controller_robot1',
        output='screen',
        parameters=[{'use_sim_time': False, 'robot_name': 'robot1'}],
        remappings=[('/robot1/scan', '/robot1/scan_raw'), ('/robot1/odom', '/robot1/wheel_odom')],
    )
    robot2_controller = Node(
        package=baseline_pkg,
        executable='robot_controller_node',
        name='robot_controller_robot2',
        output='screen',
        parameters=[{'use_sim_time': False, 'robot_name': 'robot2'}],
        remappings=[('/robot2/scan', '/robot2/scan_raw'), ('/robot2/odom', '/robot2/wheel_odom')],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )

    return LaunchDescription([
        microros_agent,
        TimerAction(period=3.0, actions=[wheel_odom_r1, wheel_odom_r2]),
        TimerAction(period=6.0, actions=[multirobot_slam]),
        TimerAction(period=8.0, actions=[robot_coordinator, robot1_controller, robot2_controller]),
        TimerAction(period=10.0, actions=[rviz]),
    ])
