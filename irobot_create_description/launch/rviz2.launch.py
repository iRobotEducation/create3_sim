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
# @author Rodrigo Jose Causarano Nuñez (rcausaran@irobot.com)
#
# Launch Create3 in RViz.

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

ARGUMENTS = [
  DeclareLaunchArgument('rviz', default_value='true',
                        description='Start rviz')
]


def generate_launch_description():
    pkg_create3_description = get_package_share_directory('irobot_create_description')

    xacro_file = PathJoinSubstitution([pkg_create3_description, 'urdf', 'create3.urdf.xacro'])

    # Rviz
    rviz_config_dir = PathJoinSubstitution([pkg_create3_description, 'rviz', 'model.rviz'])

    rviz = Node(package='rviz2', executable='rviz2', name='rviz2',
                arguments=['-d', rviz_config_dir],
                condition=IfCondition(LaunchConfiguration('rviz')), output='screen')

    robot_state_publisher = Node(package='robot_state_publisher',
                                 executable='robot_state_publisher', name='robot_state_publisher',
                                 output='screen',
                                 parameters=[{'use_sim_time': True},
                                             {'robot_description': Command(['xacro', ' ', xacro_file])}])

    joint_state_publisher = Node(package='joint_state_publisher_gui',
                                 executable='joint_state_publisher_gui',
                                 name='joint_state_publisher_gui',
                                 condition=IfCondition(LaunchConfiguration('rviz')),
                                 output='screen')

    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)
    # Add nodes to LaunchDescription
    ld.add_action(joint_state_publisher)
    ld.add_action(robot_state_publisher)
    ld.add_action(rviz)

    return ld
