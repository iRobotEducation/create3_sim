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
# @author Luis Enrique Chico Capistrano (lchico@irobot.com)
#
# Launch Create3 in Gazebo and optionally also in RViz.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

ARGUMENTS = [
    # Set the robot and dock position close to the wall by default
    DeclareLaunchArgument('x', default_value='9.0',
                          description='Update "x" component of the robot pose.')
]


def generate_launch_description():
    # Directories
    pkg_aws_house_dir = get_package_share_directory('aws_robomaker_small_house_world')
    pkg_create3_gazebo = get_package_share_directory('irobot_create_gazebo')

    # Paths
    create3_launch_file = PathJoinSubstitution(
        [pkg_create3_gazebo, 'launch', 'create3.launch.py'])

    world_name_str = 'small_house.world'
    world_path = os.path.join(pkg_aws_house_dir, 'worlds', world_name_str)

    # Includes
    # Disable gui and gzserver because they launch on AWS Small world
    create3_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([create3_launch_file]),
        launch_arguments={'world_path': world_path}.items()
    )

    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)
    # Add nodes to LaunchDescription
    ld.add_action(create3_gazebo)
    # Add models AWS models to gazebo path
    os.environ['GAZEBO_MODEL_PATH'] += os.pathsep + os.path.join(pkg_aws_house_dir, 'models')

    return ld
