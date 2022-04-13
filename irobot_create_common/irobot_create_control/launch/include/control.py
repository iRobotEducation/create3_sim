# Copyright 2021 iRobot Corporation. All Rights Reserved.
# @author Rodrigo Jose Causarano Nunez (rcausaran@irobot.com)
#
# Launch Create(R) 3 with diffdrive controller in Gazebo and optionally also in RViz.

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
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
    pkg_create3_control = get_package_share_directory('irobot_create_control')

    control_params_file = PathJoinSubstitution(
        [pkg_create3_control, 'config', 'control.yaml'])

    diffdrive_controller_node = Node(
        package='controller_manager',
        executable='spawner',
        namespace=namespace,
        parameters=[control_params_file],
        arguments=['diffdrive_controller', '-c', namespaced_node_name],
        output='screen',
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace=namespace,
        arguments=['joint_state_broadcaster', '-c', namespaced_node_name],
        output='screen',
    )

    # Ensure diffdrive_controller_node starts after joint_state_broadcaster_spawner
    diffdrive_controller_callback = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diffdrive_controller_node],
        )
    )

    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(diffdrive_controller_callback)

    return ld
