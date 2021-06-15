"""Launch Create3 in RViz."""

import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    pkg_create3_description = get_package_share_directory('irobot_create_description')

    urdf_dir = os.path.join(pkg_create3_description, 'urdf')
    xacro_file = os.path.join(urdf_dir, 'create3.urdf.xacro')

    # Gazebo server
    gzserver_exe = launch.actions.ExecuteProcess(
        cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # Gazebo client
    gzclient_exe = launch.actions.ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': Command(['xacro',' ',xacro_file])}
        ],
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'create3', '-topic', 'robot_description',
                   '-x', '0', '-y', '0', '-z', '0.0', '-Y', "0"],
        output='screen'
    )

    return LaunchDescription([robot_state_publisher, spawn_robot, gzserver_exe, gzclient_exe])
