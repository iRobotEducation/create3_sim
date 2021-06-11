"""Launch Gazebo with a world that has a Create3."""

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

    # Rviz
    rviz_config_dir = os.path.join(
        pkg_create3_description,
        'rviz',
        'model.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        output='screen')

    # Gazebo server
    gzserver_exe = launch.actions.ExecuteProcess(
        cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # Publish joint states
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

    return LaunchDescription([gzserver_exe, robot_state_publisher, spawn_robot, rviz_node])
