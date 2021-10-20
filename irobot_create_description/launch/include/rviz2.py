# Copyright 2021 iRobot Corporation. All Rights Reserved.
# @author Rodrigo Jose Causarano Nunez (rcausaran@irobot.com)
#
# Launch Create(R) 3 in RViz.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

# Rviz requires US locale to correctly display the wheels
os.environ['LC_NUMERIC'] = 'en_US.UTF-8'

ARGUMENTS = [
    DeclareLaunchArgument('rviz', default_value='true',
                          description='Start rviz')
]


def generate_launch_description():
    pkg_create3_description = get_package_share_directory('irobot_create_description')
    xacro_file = PathJoinSubstitution([pkg_create3_description, 'urdf', 'create3.urdf.xacro'])

    # Rviz
    rviz_config_dir = PathJoinSubstitution([pkg_create3_description, 'rviz', 'model.rviz'])
    rviz_logo_dir = PathJoinSubstitution([pkg_create3_description, 'rviz', 'irobot_logo.jpg'])

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '--display-config', rviz_config_dir,
            '--splash-screen', rviz_logo_dir,
        ],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': Command(['xacro', ' ', xacro_file])},
        ],
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
    )

    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)
    # Add nodes to LaunchDescription
    ld.add_action(joint_state_publisher)
    ld.add_action(robot_state_publisher)
    ld.add_action(rviz)

    return ld
