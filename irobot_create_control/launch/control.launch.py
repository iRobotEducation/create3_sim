#!/usr/bin/env python3
# Copyright 2021 iRobot Corporation. All Rights Reserved.
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# @author Rodrigo Jose Causarano Nunez (rcausaran@irobot.com)
#
# Launch Create3 with diffdrive controller in Gazebo and optionally also in RViz.

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument('rviz', default_value='false',
                          description='Start rviz'),
    DeclareLaunchArgument('gui', default_value='true',
                          description='Set "false" to run gazebo headless')
]


def generate_launch_description():

    pkg_create3_gazebo = get_package_share_directory('irobot_create_gazebo')
    pkg_create3_control = get_package_share_directory('irobot_create_control')
    gazebo_launch_file = PathJoinSubstitution(
        [pkg_create3_gazebo, 'launch', 'create3.launch.py'])
    control_params_file = PathJoinSubstitution(
        [pkg_create3_control, 'config', 'control.yaml'])

    diffdrive_controller_node = Node(
        package='controller_manager',
        executable='spawner.py',
        parameters=[control_params_file],
        arguments=['create_diffdrive_controller', '-c', '/controller_manager'],
        output='screen',
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner.py',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
        output='screen',
    )

    # Ensure diffdrive_controller_node starts after joint_state_broadcaster_spawner
    diffdrive_controller_callback = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diffdrive_controller_node],
        )
    )

    include_gazebo_launch_file = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([gazebo_launch_file])
        )

    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)

    ld.add_action(include_gazebo_launch_file)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(diffdrive_controller_callback)

    return ld
