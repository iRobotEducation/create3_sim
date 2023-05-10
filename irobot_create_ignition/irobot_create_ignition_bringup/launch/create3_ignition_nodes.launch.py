# Copyright 2021 Clearpath Robotics, Inc.
# @author Roni Kreinin (rkreinin@clearpathrobotics.com)

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument('robot_name', default_value='create3',
                          description='Robot name'),
    DeclareLaunchArgument('dock_name', default_value='standard_dock',
                          description='Dock name')
]


def generate_launch_description():

    # Directories
    pkg_create3_ignition_bringup = get_package_share_directory('irobot_create_ignition_bringup')

    pose_republisher_params_yaml_file = PathJoinSubstitution(
        [pkg_create3_ignition_bringup, 'config', 'pose_republisher_params.yaml'])
    sensors_params_yaml_file = PathJoinSubstitution(
        [pkg_create3_ignition_bringup, 'config', 'sensors_params.yaml'])

    # Pose republisher
    pose_republisher_node = Node(
        package='irobot_create_ignition_toolbox',
        name='pose_republisher_node',
        executable='pose_republisher_node',
        parameters=[pose_republisher_params_yaml_file,
                    {'robot_name': LaunchConfiguration('robot_name')},
                    {'dock_name': LaunchConfiguration('dock_name')},
                    {'use_sim_time': True}],
        output='screen',
    )

    # Sensors
    sensors_node = Node(
        package='irobot_create_ignition_toolbox',
        name='sensors_node',
        executable='sensors_node',
        parameters=[sensors_params_yaml_file,
                    {'use_sim_time': True}],
        output='screen',
    )

    # Interface buttons
    interface_buttons_node = Node(
        package='irobot_create_ignition_toolbox',
        name='interface_buttons_node',
        executable='interface_buttons_node',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(pose_republisher_node)
    ld.add_action(sensors_node)
    ld.add_action(interface_buttons_node)
    return ld
