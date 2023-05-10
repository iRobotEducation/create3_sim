#!/usr/bin/env python3
# Copyright 2021 iRobot Corporation. All Rights Reserved.
# @author Rodrigo Jose Causarano Nunez (rcausaran@irobot.com)
#
# Launch Create(R) 3 nodes

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument('gazebo', default_value='classic',
                          choices=['classic', 'ignition'],
                          description='Which gazebo simulator to use'),
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
]


def generate_launch_description():

    # Directories
    pkg_create3_common_bringup = get_package_share_directory('irobot_create_common_bringup')
    pkg_create3_control = get_package_share_directory('irobot_create_control')

    # Paths
    control_launch_file = PathJoinSubstitution(
        [pkg_create3_control, 'launch', 'include', 'control.py'])
    hazards_params_yaml_file = PathJoinSubstitution(
        [pkg_create3_common_bringup, 'config', 'hazard_vector_params.yaml'])
    ir_intensity_params_yaml_file = PathJoinSubstitution(
        [pkg_create3_common_bringup, 'config', 'ir_intensity_vector_params.yaml'])
    wheel_status_params_yaml_file = PathJoinSubstitution(
        [pkg_create3_common_bringup, 'config', 'wheel_status_params.yaml'])
    mock_params_yaml_file = PathJoinSubstitution(
        [pkg_create3_common_bringup, 'config', 'mock_params.yaml'])
    robot_state_yaml_file = PathJoinSubstitution(
        [pkg_create3_common_bringup, 'config', 'robot_state_params.yaml'])
    kidnap_estimator_yaml_file = PathJoinSubstitution(
        [pkg_create3_common_bringup, 'config', 'kidnap_estimator_params.yaml'])
    ui_mgr_params_yaml_file = PathJoinSubstitution(
        [pkg_create3_common_bringup, 'config', 'ui_mgr_params.yaml'])

    # Includes
    diffdrive_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([control_launch_file]),
        launch_arguments=[('namespace', LaunchConfiguration('namespace'))]
    )

    # Publish hazards vector
    hazards_vector_node = Node(
        package='irobot_create_nodes',
        name='hazards_vector_publisher',
        executable='hazards_vector_publisher',
        parameters=[hazards_params_yaml_file,
                    {'use_sim_time': True}],
        output='screen',
    )

    # Publish IR intensity vector
    ir_intensity_vector_node = Node(
        package='irobot_create_nodes',
        name='ir_intensity_vector_publisher',
        executable='ir_intensity_vector_publisher',
        parameters=[ir_intensity_params_yaml_file,
                    {'use_sim_time': True}],
        output='screen',
    )

    # Motion Control
    motion_control_node = Node(
        package='irobot_create_nodes',
        name='motion_control',
        executable='motion_control',
        parameters=[{'use_sim_time': True}],
        output='screen',
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    # Publish wheel status
    wheel_status_node = Node(
        package='irobot_create_nodes',
        name='wheel_status_publisher',
        executable='wheel_status_publisher',
        parameters=[wheel_status_params_yaml_file,
                    {'use_sim_time': True}],
        output='screen',
    )

    # Publish mock topics
    mock_topics_node = Node(
        package='irobot_create_nodes',
        name='mock_publisher',
        executable='mock_publisher',
        parameters=[mock_params_yaml_file,
                    {'use_sim_time': True}],
        output='screen',
    )

    # Publish robot state
    robot_state_node = Node(
        package='irobot_create_nodes',
        name='robot_state',
        executable='robot_state',
        parameters=[robot_state_yaml_file,
                    {'use_sim_time': True}],
        output='screen',
    )

    # Publish kidnap estimator
    kidnap_estimator_node = Node(
        package='irobot_create_nodes',
        name='kidnap_estimator_publisher',
        executable='kidnap_estimator_publisher',
        parameters=[kidnap_estimator_yaml_file,
                    {'use_sim_time': True}],
        output='screen',
    )

    # UI topics / actions
    ui_mgr_node = Node(
        package='irobot_create_nodes',
        name='ui_mgr',
        executable='ui_mgr',
        parameters=[ui_mgr_params_yaml_file,
                    {'use_sim_time': True},
                    {'gazebo': LaunchConfiguration('gazebo')}],
        output='screen',
    )

    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)
    # Include robot description
    ld.add_action(diffdrive_controller)
    # Add nodes to LaunchDescription
    ld.add_action(hazards_vector_node)
    ld.add_action(ir_intensity_vector_node)
    ld.add_action(motion_control_node)
    ld.add_action(wheel_status_node)
    ld.add_action(mock_topics_node)
    ld.add_action(robot_state_node)
    ld.add_action(kidnap_estimator_node)
    ld.add_action(ui_mgr_node)

    return ld
