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
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument('rviz', default_value='true',
                          choices=['true', 'false'],
                          description='Start rviz.'),
    DeclareLaunchArgument('gui', default_value='true',
                          choices=['true', 'false'],
                          description='Set "false" to run gazebo headless.'),
    DeclareLaunchArgument('dock', default_value='true',
                          choices=['true', 'false'],
                          description='Spawn the standard dock model.'),
]

for pose_element in ['x', 'y', 'z', 'yaw']:
    ARGUMENTS.append(DeclareLaunchArgument(pose_element, default_value='0.0',
                     description=f'{pose_element} component of the robot pose.'))


def generate_launch_description():
    # Directories
    pkg_create3_control = get_package_share_directory('irobot_create_control')
    pkg_create3_description = get_package_share_directory('irobot_create_description')

    # Paths
    control_launch_file = PathJoinSubstitution(
        [pkg_create3_control, 'launch', 'control.launch.py'])
    description_launch_file = PathJoinSubstitution(
        [pkg_create3_description, 'launch', 'rviz2.launch.py'])
    dock_launch_file = PathJoinSubstitution(
        [pkg_create3_description, 'launch', 'dock.launch.py'])

    # Launch configurations
    x, y, z = LaunchConfiguration('x'), LaunchConfiguration('y'), LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')

    # Includes
    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([description_launch_file])
    )
    diffdrive_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([control_launch_file])
    )
    spawn_dock = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([dock_launch_file]),
        condition=IfCondition(LaunchConfiguration('dock')),
        # The robot starts docked
        launch_arguments={'x': x, 'y': y, 'z': z, 'yaw': yaw}.items(),
    )

    # Gazebo server
    gzserver = ExecuteProcess(
        cmd=['gzserver',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so'],
        output='screen',
    )

    # Gazebo client
    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('gui')),
    )

    # Spawn robot
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_create3',
        arguments=['-entity',
                   'create3',
                   '-topic',
                   'robot_description',
                   '-x', x,
                   '-y', y,
                   '-z', z,
                   '-Y', yaw],
        output='screen',
    )

    # Publish hazards vector
    hazards_vector_node = Node(
        package='irobot_create_toolbox',
        name='hazards_vector_node',
        executable='hazards_vector_node',
    )

    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)
    # Include robot description
    ld.add_action(robot_description)
    ld.add_action(diffdrive_controller)
    # Add nodes to LaunchDescription
    ld.add_action(gzserver)
    ld.add_action(gzclient)
    ld.add_action(spawn_robot)
    ld.add_action(spawn_dock)
    ld.add_action(hazards_vector_node)

    return ld
