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
# Launch Create3 in Gazebo and optionally also in RViz.

from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument('rviz', default_value='true',
                          description='Start rviz'),
    DeclareLaunchArgument('gui', default_value='true',
                          description='Set "false" to run gazebo headless')
]


def generate_launch_description():
    pkg_create3_description = get_package_share_directory('irobot_create_description')
    description_launch_file = PathJoinSubstitution(
        [pkg_create3_description, 'launch', 'rviz2.launch.py'])

    # Gazebo server
    gzserver = launch.actions.ExecuteProcess(
        cmd=['gzserver', '--verbose',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # Gazebo client
    gzclient = launch.actions.ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('gui_required'))
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity',
                   'create3',
                   '-topic',
                   'robot_description',
                   '-x', '0',
                   '-y', '0',
                   '-z', '0',
                   '-Y', '0'],
        output='screen'
    )

    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)
    # Include robot description
    ld.add_action(IncludeLaunchDescription(
            PythonLaunchDescriptionSource([description_launch_file])
        ))
    # Add nodes to LaunchDescription
    ld.add_action(spawn_robot)
    ld.add_action(gzserver)
    ld.add_action(gzclient)

    return ld
