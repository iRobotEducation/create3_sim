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
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_create3_control = get_package_share_directory('irobot_create_control')

    control_params_file = PathJoinSubstitution(
        [pkg_create3_control, 'config', 'control.yaml'])

    diffdrive_controller_node = Node(
        package='controller_manager',
        executable='spawner',
        parameters=[control_params_file],
        arguments=['diffdrive_controller', '-c', '/controller_manager'],
        output='screen',
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
        output='screen',
    )

    cmd_vel_republish_node = Node(
        package='irobot_create_toolbox',
        name='republish_node',
        executable='topic_republisher.py',
        parameters=[
            {'current_topic': 'cmd_vel'},
            {'new_topic': 'diffdrive_controller/cmd_vel_unstamped'},
        ]
    )

    # Ensure diffdrive_controller_node starts after joint_state_broadcaster_spawner
    diffdrive_controller_callback = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diffdrive_controller_node],
        )
    )

    ld = LaunchDescription()

    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(diffdrive_controller_callback)
    ld.add_action(cmd_vel_republish_node)

    return ld
