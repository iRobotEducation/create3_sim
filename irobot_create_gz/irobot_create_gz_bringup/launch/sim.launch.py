# Copyright 2021 Clearpath Robotics, Inc.
# @author Roni Kreinin (rkreinin@clearpathrobotics.com)

import os

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='use_sim_time'),
    DeclareLaunchArgument('world', default_value='depot',
                          description='Ignition World'),
]


def generate_launch_description():

    # Directories
    pkg_irobot_create_gz_bringup = get_package_share_directory(
        'irobot_create_gz_bringup')
    pkg_irobot_create_gz_plugins = get_package_share_directory(
        'irobot_create_gz_plugins')
    pkg_irobot_create_description = get_package_share_directory(
        'irobot_create_description')
    pkg_ros_gz_sim = get_package_share_directory(
        'ros_gz_sim')

    # Set Ignition resource path
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=':'.join([
            os.path.join(pkg_irobot_create_gz_bringup, 'worlds'),
            str(Path(pkg_irobot_create_description).parent.resolve())
        ])
    )

    gz_gui_plugin_path = SetEnvironmentVariable(
        name='GZ_GUI_PLUGIN_PATH',
        value=':'.join([
            os.path.join(pkg_irobot_create_gz_plugins, 'lib')
        ])
    )

    # Paths
    gz_sim_launch = PathJoinSubstitution(
        [pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])

    # Ignition gazebo
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gz_sim_launch]),
        launch_arguments=[(
            'gz_args', [
                LaunchConfiguration('world'),
                '.sdf',
                ' -v 4',
                ' --gui-config ',
                PathJoinSubstitution(
                    [pkg_irobot_create_gz_bringup, 'gui', 'create3', 'gui.config']
                )
            ]
        )]
    )

    # clock bridge
    clock_bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
                        name='clock_bridge',
                        output='screen',
                        arguments=[
                            '/clock' + '@rosgraph_msgs/msg/Clock' + '[ignition.msgs.Clock'
                        ])

    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(gz_resource_path)
    ld.add_action(gz_gui_plugin_path)
    ld.add_action(gz_sim)
    ld.add_action(clock_bridge)
    return ld
