#!/usr/bin/env python3
# Copyright 2021 iRobot Corporation. All Rights Reserved.
# @author Rodrigo Jose Causarano Nunez (rcausaran@irobot.com)
#
# Launch Create(R) 3 in Gazebo and optionally also in RViz.

import os

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration


ARGUMENTS = [
    DeclareLaunchArgument('use_rviz', default_value='true',
                          choices=['true', 'false'],
                          description='Start rviz.'),
    DeclareLaunchArgument('use_gazebo_gui', default_value='true',
                          choices=['true', 'false'],
                          description='Set "false" to run gazebo headless.'),
    DeclareLaunchArgument('world_path', default_value='',
                          description='Set world path, by default is empty.world'),
]


# Rviz requires US locale to correctly display the wheels
os.environ['LC_NUMERIC'] = 'en_US.UTF-8'


def generate_launch_description():
    # Directories
    pkg_create3_gazebo_bringup = get_package_share_directory('irobot_create_gazebo_bringup')
    pkg_irobot_create_description = get_package_share_directory('irobot_create_description')

    # Set ignition resource path
    gz_resource_path = SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=[
                                                EnvironmentVariable('GAZEBO_MODEL_PATH',
                                                                    default_value=''),
                                                '/usr/share/gazebo-11/models/:',
                                                str(Path(pkg_irobot_create_description).
                                                    parent.resolve())])

    # Set GAZEBO_MODEL_URI to empty string to prevent Gazebo from downloading models
    gz_model_uri = SetEnvironmentVariable(name='GAZEBO_MODEL_URI', value=[''])

    gazebo_params_yaml_file = os.path.join(
        pkg_create3_gazebo_bringup, 'config', 'gazebo_params.yaml')

    # Launch configurations
    world_path = LaunchConfiguration('world_path')
    use_gazebo_gui = LaunchConfiguration('use_gazebo_gui')

    # Gazebo server
    gzserver = ExecuteProcess(
        cmd=['gzserver',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             world_path,
             'extra-gazebo-args', '--ros-args', '--params-file', gazebo_params_yaml_file],
        output='screen',
    )

    # Gazebo client
    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        condition=IfCondition(use_gazebo_gui),
    )

    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)
    # Gazebo processes
    ld.add_action(gz_resource_path)
    ld.add_action(gz_model_uri)
    ld.add_action(gzserver)
    ld.add_action(gzclient)

    return ld
