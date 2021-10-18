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
from typing import Text

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError

from launch import LaunchContext, LaunchDescription, SomeSubstitutionsType, Substitution
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


class JoystickConfigParser(Substitution):
    def __init__(
        self,
        joystick_package_name: Text,
        joystick_type: SomeSubstitutionsType
    ) -> None:
        self.__joystick_package_name = joystick_package_name
        self.__joystick_type = joystick_type

    def perform(
        self,
        context: LaunchContext = None,
    ) -> Text:
        joystick_type_str = self.__joystick_type.perform(context)
        joystick_package_str = self.__joystick_package_name
        joystick_share_dir = ''
        try:
            joystick_share_dir = get_package_share_directory(
                joystick_package_str)
        except PackageNotFoundError:
            raise PackageNotFoundError(joystick_package_str)
        else:
            config_filepath = [joystick_share_dir, 'config',
                               f'{joystick_type_str}.config.yaml']
            return os.path.join(*config_filepath)


def generate_launch_description():
    joy_config = LaunchConfiguration('joy_config')
    joy_dev = LaunchConfiguration('joy_dev')

    # Invokes a node that interfaces a generic joystick to ROS2.
    joy_node = Node(package='joy', executable='joy_node', name='joy_node',
                    parameters=[{
                        'dev': joy_dev,
                        'deadzone': 0.3,
                        'autorepeat_rate': 20.0,
                    }])

    # Retrieve the path to the correct configuration .yaml depending on
    # the joy_config argument
    config_filepath = JoystickConfigParser('teleop_twist_joy', joy_config)

    # Publish unstamped Twist message from an attached USB Joystick.
    teleop_node = Node(package='teleop_twist_joy', executable='teleop_node',
                       name='teleop_twist_joy_node', parameters=[config_filepath])

    # Declare launchfile arguments
    ld_args = []
    ld_args.append(DeclareLaunchArgument('joy_config',
                                         default_value='xbox',
                                         choices=['xbox', 'ps3', 'ps3-holonomic', 'atk3', 'xd3']))
    ld_args.append(DeclareLaunchArgument('joy_dev',
                                         default_value='/dev/input/js0'))

    # Define LaunchDescription variable
    ld = LaunchDescription(ld_args)

    # Add nodes to LaunchDescription
    ld.add_action(joy_node)
    ld.add_action(teleop_node)

    return ld
