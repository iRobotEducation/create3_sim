# Copyright 2021 iRobot Corporation. All Rights Reserved.
# @author Rodrigo Jose Causarano Nunez (rcausaran@irobot.com)
#
# Launch Create(R) 3 with diffdrive controller in Gazebo and optionally also in RViz.

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace')
]

def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    namespaced_node_name = [namespace, '/controller_manager']
    namespaced_diffdrive_controller = [namespace, '_diffdrive_controller']
    namespaced_joint_state_broadcaster = [namespace, '_joint_state_broadcaster']
    pkg_create3_control = get_package_share_directory('irobot_create_control')

    control_params_file = PathJoinSubstitution(
        [pkg_create3_control, 'config', 'control.yaml'])

    diffdrive_controller_node = Node(
        condition=LaunchConfigurationEquals('namespace', ''),
        package='controller_manager',
        executable='spawner',
        parameters=[control_params_file],
        arguments=['diffdrive_controller', '-c', namespaced_node_name],
        output='screen',
    )

    joint_state_broadcaster_spawner = Node(
        condition=LaunchConfigurationEquals('namespace', ''),
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', namespaced_node_name],
        output='screen',
    )

    # Ensure diffdrive_controller_node starts after joint_state_broadcaster_spawner
    diffdrive_controller_callback= RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diffdrive_controller_node],
        )
    )

    # Launch with namespace
    diffdrive_controller_node_namespaced = Node(
        condition=LaunchConfigurationNotEquals('namespace', ''),
        package='controller_manager',
        executable='spawner',
        namespace=namespace,
        parameters=[control_params_file],
        arguments=[namespaced_diffdrive_controller, '-c', namespaced_node_name],
        output='screen',
    )

    joint_state_broadcaster_spawner_namespaced = Node(
        condition=LaunchConfigurationNotEquals('namespace', ''),
        package='controller_manager',
        executable='spawner',
        namespace=namespace,
        arguments=[namespaced_joint_state_broadcaster, '-c', namespaced_node_name],
        output='screen',
    )

    # Ensure diffdrive_controller_node starts after joint_state_broadcaster_spawner
    diffdrive_controller_callback_namespaced = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner_namespaced,
            on_exit=[diffdrive_controller_node_namespaced],
        )
    )

    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(diffdrive_controller_callback)
    ld.add_action(joint_state_broadcaster_spawner_namespaced)
    ld.add_action(diffdrive_controller_callback_namespaced)

    return ld
