#!/usr/bin/env python3
# Copyright 2021 iRobot Corporation. All Rights Reserved.
# @author Luis Enrique Chico Capistrano (lchico@irobot.com)

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, PathJoinSubstitution


ARGUMENTS = []

# Set the robot and dock pose close to the wall by default
for pose_element, default_value in zip(['x', 'y', 'yaw'], ['9.0', '0.0', '0.0']):
    ARGUMENTS.append(DeclareLaunchArgument(pose_element, default_value=default_value,
                     description=f'{pose_element} component of the robot pose.'))


def generate_launch_description():
    # Directories
    aws_small_house_dir = get_package_share_directory('aws_robomaker_small_house_world')
    irobot_create_gazebo_bringup_dir = get_package_share_directory('irobot_create_gazebo_bringup')

    # Paths
    create3_launch_file = PathJoinSubstitution(
        [irobot_create_gazebo_bringup_dir, 'launch', 'create3_gazebo.launch.py'])
    world_path = PathJoinSubstitution([aws_small_house_dir, 'worlds', 'small_house.world'])
    aws_model_path = PathJoinSubstitution([aws_small_house_dir, 'models:'])

    # Includes
    world_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([create3_launch_file]),
        launch_arguments={'world_path': world_path}.items())

    # Add AWS models to gazebo path
    # This environment variable needs to be set, otherwise code fails
    set_gazebo_model_path_env = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[EnvironmentVariable('GAZEBO_MODEL_PATH', default_value=''), aws_model_path])

    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)
    # Add actions to LaunchDescription
    ld.add_action(set_gazebo_model_path_env)
    ld.add_action(world_spawn)

    return ld
