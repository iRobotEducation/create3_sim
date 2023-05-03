#!/usr/bin/env python3
# Copyright 2021 iRobot Corporation. All Rights Reserved.
# @author Rodrigo Jose Causarano Nunez (rcausaran@irobot.com)
#
# Launch Create(R) 3 in Gazebo and optionally also in RViz.

import os

from ament_index_python.packages import get_package_share_directory

from irobot_create_common_bringup.namespace import GetNamespacedName
from irobot_create_common_bringup.offset import OffsetParser, RotationalOffsetX, RotationalOffsetY

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace


ARGUMENTS = [
    DeclareLaunchArgument('use_rviz', default_value='true',
                          choices=['true', 'false'],
                          description='Start rviz.'),
    DeclareLaunchArgument('spawn_dock', default_value='true',
                          choices=['true', 'false'],
                          description='Spawn the standard dock model.'),
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
]

for pose_element in ['x', 'y', 'z', 'yaw']:
    ARGUMENTS.append(DeclareLaunchArgument(pose_element, default_value='0.0',
                     description=f'{pose_element} component of the robot pose.'))


# Rviz requires US locale to correctly display the wheels
os.environ['LC_NUMERIC'] = 'en_US.UTF-8'


def generate_launch_description():
    # Directories
    pkg_create3_common_bringup = get_package_share_directory('irobot_create_common_bringup')

    # Paths
    create3_nodes_launch_file = PathJoinSubstitution(
        [pkg_create3_common_bringup, 'launch', 'create3_nodes.launch.py'])
    dock_description_launch_file = PathJoinSubstitution(
        [pkg_create3_common_bringup, 'launch', 'dock_description.launch.py'])
    robot_description_launch_file = PathJoinSubstitution(
        [pkg_create3_common_bringup, 'launch', 'robot_description.launch.py'])
    rviz2_launch_file = PathJoinSubstitution(
        [pkg_create3_common_bringup, 'launch', 'rviz2.launch.py'])

    # Launch configurations
    namespace = LaunchConfiguration('namespace')
    x, y, z = LaunchConfiguration('x'), LaunchConfiguration('y'), LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')
    spawn_dock = LaunchConfiguration('spawn_dock')
    use_rviz = LaunchConfiguration('use_rviz')

    robot_name = GetNamespacedName(namespace, 'create3')
    dock_name = GetNamespacedName(namespace, 'standard_dock')

    # Calculate dock offset due to yaw rotation
    dock_offset_x = RotationalOffsetX(0.157, yaw)
    dock_offset_y = RotationalOffsetY(0.157, yaw)
    # Spawn dock at robot position + rotational offset
    x_dock = OffsetParser(x, dock_offset_x)
    y_dock = OffsetParser(y, dock_offset_y)
    # Rotate dock towards robot
    yaw_dock = OffsetParser(yaw, 3.1416)

    spawn_robot_group_action = GroupAction([
        PushRosNamespace(namespace),

        # Dock description
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([dock_description_launch_file]),
            launch_arguments={'gazebo': 'classic'}.items(),
            condition=IfCondition(spawn_dock),
        ),

        # Dock spawn
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_standard_dock',
            arguments=['-entity',
                       dock_name,
                       '-topic',
                       'standard_dock_description',
                       '-x', x_dock,
                       '-y', y_dock,
                       '-z', z,
                       '-Y', yaw_dock],
            output='screen',
            condition=IfCondition(spawn_dock),
        ),

        # Create 3 robot model and description
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([robot_description_launch_file]),
            launch_arguments={'gazebo': 'classic'}.items(),
        ),

        # Create 3 spawn
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_create3',
            arguments=['-entity',
                       robot_name,
                       '-topic',
                       'robot_description',
                       '-x', x,
                       '-y', y,
                       '-z', z,
                       '-Y', yaw],
            output='screen',
        ),

        # Create 3 nodes
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([create3_nodes_launch_file]),
            launch_arguments=[
                ('namespace', namespace)
            ]
        ),

        # RVIZ2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([rviz2_launch_file]),
            condition=IfCondition(use_rviz),
        ),
    ])

    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(spawn_robot_group_action)
    return ld
