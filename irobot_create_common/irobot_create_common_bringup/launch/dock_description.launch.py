# Copyright 2021 iRobot Corporation. All Rights Reserved.
# @author Emiliano Javier Borghi Orue (creativa_eborghi@irobot.com)
#
# Launch standard docking station state publishers.

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument('gazebo', default_value='classic',
                          choices=['classic', 'ignition'],
                          description='Which gazebo simulation to use'),
    DeclareLaunchArgument('visualize_rays', default_value='true',
                          choices=['true', 'false'],
                          description='Enable/disable ray visualization'),
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
]


def generate_launch_description():
    # Directory
    pkg_create3_description = get_package_share_directory('irobot_create_description')
    # Path
    dock_xacro_file = PathJoinSubstitution(
        [pkg_create3_description, 'urdf', 'dock', 'standard_dock.urdf.xacro'])

    # Launch Configurations
    visualize_rays = LaunchConfiguration('visualize_rays')
    gazebo_simulator = LaunchConfiguration('gazebo')
    namespace = LaunchConfiguration('namespace')

    state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='dock_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'robot_description':
             Command(
                ['xacro', ' ', dock_xacro_file, ' ',
                 'gazebo:=', gazebo_simulator, ' ',
                 'namespace:=', namespace, ' ',
                 'visualize_rays:=', visualize_rays])},
        ],
        remappings=[
            ('robot_description', 'standard_dock_description'),
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ],
    )

    tf_odom_std_dock_link_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_odom_std_dock_link_publisher',
        arguments=['0.157', '0', '0',
                   # According to documentation (http://wiki.ros.org/tf2_ros):
                   # the order is yaw, pitch, roll
                   '3.141592', '0', '0',
                   'odom', 'std_dock_link'],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ],
        output='screen',
    )

    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)
    # Add nodes to LaunchDescription
    ld.add_action(state_publisher)
    ld.add_action(tf_odom_std_dock_link_publisher)

    return ld
