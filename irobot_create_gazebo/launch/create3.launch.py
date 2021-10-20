#!/usr/bin/env python3
# Copyright 2021 iRobot Corporation. All Rights Reserved.
# @author Rodrigo Jose Causarano Nunez (rcausaran@irobot.com)
#
# Launch Create(R) 3 in Gazebo and optionally also in RViz.

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
    DeclareLaunchArgument('world_path', default_value='',
                          description='Set world path, by default is empty.world'),
]

for pose_element in ['x', 'y', 'z', 'yaw']:
    ARGUMENTS.append(DeclareLaunchArgument(pose_element, default_value='0.0',
                     description=f'{pose_element} component of the robot pose.'))


def generate_launch_description():
    # Directories
    pkg_create3_control = get_package_share_directory('irobot_create_control')
    pkg_create3_description = get_package_share_directory('irobot_create_description')
    pkg_create3_gazebo = get_package_share_directory('irobot_create_gazebo')

    # Paths
    control_launch_file = PathJoinSubstitution(
        [pkg_create3_control, 'launch', 'include', 'control.py'])
    description_launch_file = PathJoinSubstitution(
        [pkg_create3_description, 'launch', 'include', 'rviz2.py'])
    dock_launch_file = PathJoinSubstitution(
        [pkg_create3_description, 'launch', 'include', 'dock.py'])
    hazards_params_yaml_file = PathJoinSubstitution(
        [pkg_create3_gazebo, 'config', 'hazard_vector_params.yaml'])
    ir_intensity_params_yaml_file = PathJoinSubstitution(
        [pkg_create3_gazebo, 'config', 'ir_intensity_vector_params.yaml'])
    wheel_status_params_yaml_file = PathJoinSubstitution(
        [pkg_create3_gazebo, 'config', 'wheel_status_params.yaml'])
    mock_params_yaml_file = PathJoinSubstitution(
        [pkg_create3_gazebo, 'config', 'mock_params.yaml'])

    # Launch configurations
    x, y, z = LaunchConfiguration('x'), LaunchConfiguration('y'), LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')
    world_path = LaunchConfiguration('world_path')

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
             '-s', 'libgazebo_ros_factory.so',
             world_path],
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
        executable='hazards_vector_publisher_node',
        parameters=[hazards_params_yaml_file,
                    {'use_sim_time': True}],
        output='screen',
    )

    # Publish IR intensity vector
    ir_intensity_vector_node = Node(
        package='irobot_create_toolbox',
        name='ir_intensity_vector_node',
        executable='ir_intensity_vector_publisher_node',
        parameters=[ir_intensity_params_yaml_file,
                    {'use_sim_time': True}],
        output='screen',
    )

    # Motion Control
    motion_control_node = Node(
        package='irobot_create_toolbox',
        name='motion_control',
        executable='motion_control',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # Publish wheel status
    wheel_status_node = Node(
        package='irobot_create_toolbox',
        name='wheel_status_publisher_node',
        executable='wheel_status_publisher_node',
        parameters=[wheel_status_params_yaml_file,
                    {'use_sim_time': True}],
        output='screen',
    )

    # Publish wheel status
    mock_topics_node = Node(
        package='irobot_create_toolbox',
        name='mock_publisher_node',
        executable='mock_publisher_node',
        parameters=[mock_params_yaml_file,
                    {'use_sim_time': True}],
        output='screen',
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
    ld.add_action(ir_intensity_vector_node)
    ld.add_action(motion_control_node)
    ld.add_action(wheel_status_node)
    ld.add_action(mock_topics_node)

    return ld
