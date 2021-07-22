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
# @author Emiliano Javier Borghi Orue (creativa_eborghi@irobot.com)
#
# Launch standard docking station in Gazebo.

from typing import Text

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription, SomeSubstitutionsType, Substitution
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

class OffsetParser(Substitution):
  def __init__(
      self,
      number: SomeSubstitutionsType,
      offset: float,
  ) -> None:
    self.__number = number
    self.__offset = offset

  def perform(
      self,
      context: LaunchContext = None,
  ) -> Text:
    number = float(self.__number.perform(context))
    return f"{number + self.__offset}"


ARGUMENTS = []
for pose_element in ['x', 'y', 'z', 'yaw']:
    ARGUMENTS.append(DeclareLaunchArgument(f'{pose_element}', default_value='0.0',
                     description=f'{pose_element} component of the dock pose.'))


def generate_launch_description():
    # Directory
    pkg_create3_description = get_package_share_directory('irobot_create_description')
    # Path
    dock_xacro_file = PathJoinSubstitution([pkg_create3_description, 'urdf', 'dock', 'standard.urdf.xacro'])

    # Launch Configurations
    x, y, z = LaunchConfiguration('x'), LaunchConfiguration('y'), LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')

    state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='dock_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': Command(['xacro', ' ', dock_xacro_file])}
        ],
        remappings=[
            ('robot_description', 'standard_dock_description')
        ]
    )

    spawn_model = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_standard_dock',
        arguments=['-entity',
                   'standard_dock',
                   '-topic',
                   'standard_dock_description',
                   '-x', OffsetParser(x, 0.157),
                   '-y', y,
                   '-z', z,
                   '-Y', OffsetParser(yaw, 3.1416)],
        output='screen',
    )

    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)
    # Add nodes to LaunchDescription
    ld.add_action(state_publisher)
    ld.add_action(spawn_model)

    return ld
