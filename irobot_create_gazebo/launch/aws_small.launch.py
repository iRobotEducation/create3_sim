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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, PathJoinSubstitution

import os

ARGUMENTS = []

# Set the robot and dock pose close to the wall by default
for pose_element, default_value in zip(['x', 'y', 'yaw'], ['9.0', '0.0', '0.0']):
    ARGUMENTS.append(DeclareLaunchArgument(pose_element, default_value=default_value,
                     description=f'{pose_element} component of the robot pose.'))


def generate_launch_description():
    # Directories
    aws_small_house_dir = get_package_share_directory('aws_robomaker_small_house_world')
    irobot_create_gazebo_dir = get_package_share_directory('irobot_create_gazebo')

    # Paths
    create3_launch_file = PathJoinSubstitution(
        [irobot_create_gazebo_dir, 'launch', 'create3.launch.py'])
    world_path = PathJoinSubstitution([aws_small_house_dir, 'worlds', 'small_house.world'])
    model_path = PathJoinSubstitution([aws_small_house_dir, 'models'])

    # Includes
    world_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([create3_launch_file]),
        launch_arguments={'world_path': world_path}.items())

    # Add AWS models to gazebo path
    gazebo_model_path = os.getenv('GAZEBO_MODEL_PATH')
    if gazebo_model_path is None:
        set_gazebo_model_path_env = SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=[model_path])
    else:
        set_gazebo_model_path_env = SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=[EnvironmentVariable('GAZEBO_MODEL_PATH'), model_path])

    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)
    # Add actions to LaunchDescription
    ld.add_action(set_gazebo_model_path_env)
    ld.add_action(world_spawn)

    return ld
