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
#
# @author Alexis Pojomovsky (apojomovsky@irobot.com)
#
# Launches a joystick teleop node.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    joy_config = LaunchConfiguration('joy_config')
    joy_dev = LaunchConfiguration('joy_dev')
    config_filepath = LaunchConfiguration('config_filepath')

    # Invokes a node that interfaces a generic joystick to ROS2.
    joy_node = Node(package='joy', executable='joy_node', name='joy_node',
                    parameters=[{
                        'dev': joy_dev,
                        'deadzone': 0.3,
                        'autorepeat_rate': 20.0,
                    }])

    # Publish unstamped Twist message from an attached USB Joystick.
    teleop_node = Node(package='teleop_twist_joy', executable='teleop_node',
                       name='teleop_twist_joy_node', parameters=[config_filepath])

    # Declare launchfile arguments
    ld_args = [DeclareLaunchArgument('joy_config', default_value='xbox'),
               DeclareLaunchArgument(
        'joy_dev', default_value='/dev/input/js0'),
        DeclareLaunchArgument('config_filepath', default_value=[
            TextSubstitution(text=os.path.join(
                get_package_share_directory('teleop_twist_joy'), 'config', '')),
            joy_config, TextSubstitution(text='.config.yaml')])]

    # Define LaunchDescription variable
    ld = LaunchDescription(ld_args)

    # Add nodes to LaunchDescription
    ld.add_action(joy_node)
    ld.add_action(teleop_node)

    return ld
