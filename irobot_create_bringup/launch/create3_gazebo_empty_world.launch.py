#!/usr/bin/env python3
# Copyright 2021 iRobot Corporation. All Rights Reserved.
# @author Rodrigo Jose Causarano Nunez (rcausaran@irobot.com)
#
# Launch Create(R) 3 in Gazebo and optionally also in RViz.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument('rviz', default_value='true',
                          choices=['true', 'false'],
                          description='Start rviz.'),
    DeclareLaunchArgument('gui', default_value='true',
                          choices=['true', 'false'],
                          description='Set "false" to run gazebo headless.'),
    DeclareLaunchArgument('dock', default_value='true',
                          choices=['true', 'false'],
                          description='Spawn the standard dock model.'),
    DeclareLaunchArgument('world_path', default_value='',
                          description='Set world path, by default is empty.world'),
]

for pose_element in ['x', 'y', 'z', 'yaw']:
    ARGUMENTS.append(DeclareLaunchArgument(pose_element, default_value='0.0',
                     description=f'{pose_element} component of the robot pose.'))


def generate_launch_description():
    # Directories
    pkg_create3_bringup = get_package_share_directory('irobot_create_bringup')
    pkg_create3_control = get_package_share_directory('irobot_create_control')
    pkg_create3_description = get_package_share_directory('irobot_create_description')

    # Paths
    create3_nodes_launch_file = PathJoinSubstitution(
        [pkg_create3_bringup, 'launch', 'create3_nodes.launch.py'])
    description_launch_file = PathJoinSubstitution(
        [pkg_create3_description, 'launch', 'include', 'rviz2.py'])
    dock_launch_file = PathJoinSubstitution(
        [pkg_create3_description, 'launch', 'include', 'dock.py'])

    gazebo_params_yaml_file = os.path.join(pkg_create3_bringup, 'config', 'gazebo_params.yaml')

    # Launch configurations
    x, y, z = LaunchConfiguration('x'), LaunchConfiguration('y'), LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')
    world_path = LaunchConfiguration('world_path')

    # Includes
    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([description_launch_file])
    )
    spawn_dock = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([dock_launch_file]),
        condition=IfCondition(LaunchConfiguration('dock')),
        # The robot starts docked
        launch_arguments={'x': x, 'y': y, 'z': z, 'yaw': yaw}.items(),
    )

    create3_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([create3_nodes_launch_file])
    )

    # Gazebo server
    gzserver = ExecuteProcess(
        cmd=['gzserver',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             world_path,
             'extra-gazebo-args', '--ros-args', '--params-file', gazebo_params_yaml_file],
        output='screen',
    )

    # Gazebo client
    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('gui')),
    )

    # Spawn robot
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_create3',
        arguments=['-entity',
                   'create3',
                   '-topic',
                   'robot_description',
                   '-x', x,
                   '-y', y,
                   '-z', z,
                   '-Y', yaw],
        output='screen',
    )

    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)
    # Include Create 3 nodes
    ld.add_action(create3_nodes)
    # Include robot description
    ld.add_action(robot_description)
    # Add nodes to LaunchDescription
    ld.add_action(gzserver)
    ld.add_action(gzclient)
    ld.add_action(spawn_robot)
    ld.add_action(spawn_dock)

    return ld
