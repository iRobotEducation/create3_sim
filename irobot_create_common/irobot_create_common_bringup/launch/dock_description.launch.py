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
                          description='Which gazebo simulation to use')
]
for pose_element in ['x', 'y', 'z', 'yaw']:
    ARGUMENTS.append(DeclareLaunchArgument(f'{pose_element}', default_value='0.0',
                     description=f'{pose_element} component of the dock pose.'))

ARGUMENTS.append(DeclareLaunchArgument('visualize_rays', default_value='true',
                                       choices=['true', 'false'],
                                       description='Enable/disable ray visualization'))


def generate_launch_description():
    # Directory
    pkg_create3_description = get_package_share_directory('irobot_create_description')
    # Path
    dock_xacro_file = PathJoinSubstitution(
        [pkg_create3_description, 'urdf', 'dock', 'standard_dock.urdf.xacro'])

    # Launch Configurations
    x, y, z = LaunchConfiguration('x'), LaunchConfiguration('y'), LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')
    visualize_rays = LaunchConfiguration('visualize_rays')

    gazebo_simulator = LaunchConfiguration('gazebo')

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
                 'visualize_rays:=', visualize_rays])},
        ],
        remappings=[
            ('robot_description', 'standard_dock_description'),
        ],
    )

    tf_odom_std_dock_link_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_odom_std_dock_link_publisher',
        arguments=[x, y, z,
                   # According to documentation (http://wiki.ros.org/tf2_ros):
                   # the order is yaw, pitch, roll
                   yaw, '0', '0',
                   'odom', 'std_dock_link'],
        output='screen',
    )

    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)
    # Add nodes to LaunchDescription
    ld.add_action(state_publisher)
    ld.add_action(tf_odom_std_dock_link_publisher)

    return ld
