#!/usr/bin/env python3
# Copyright 2021 iRobot Corporation. All Rights Reserved.
# @author Emiliano Javier Borghi Orue (creativa_eborghi@irobot.com)
#
# Launch a keyboard teleop node.

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # Publish unstamped Twist message with the keyboard.
    # Executes the node in another XTerm terminal to avoid issues
    # with ros2 launch.
    teleop_twist_keyboard = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop',
        output='screen',
        prefix='xterm -e',
    )

    # Define LaunchDescription variable
    ld = LaunchDescription()
    # Add nodes to LaunchDescription
    ld.add_action(teleop_twist_keyboard)

    return ld
