# Copyright 2021 iRobot Corporation. All Rights Reserved.
# @author Emiliano Javier Borghi Orue (creativa_eborghi@irobot.com)
#
# Launch standard docking station in Gazebo.

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription, SomeSubstitutionsType, Substitution
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


class OffsetParser(Substitution):
    def __init__(
            self,
            number: SomeSubstitutionsType,
            offset: float,
    ) -> None:
        self.__number = number
        self.__offset = offset

    def perform(
            self,
            context: LaunchContext = None,
    ) -> str:
        number = float(self.__number.perform(context))
        return f'{number + self.__offset}'


ARGUMENTS = []
for pose_element in ['x', 'y', 'z', 'yaw']:
    ARGUMENTS.append(DeclareLaunchArgument(f'{pose_element}', default_value='0.0',
                     description=f'{pose_element} component of the dock pose.'))


def generate_launch_description():
    # Directory
    pkg_create3_description = get_package_share_directory('irobot_create_description')
    # Path
    dock_xacro_file = PathJoinSubstitution(
        [pkg_create3_description, 'urdf', 'dock', 'standard.urdf.xacro'])

    # Launch Configurations
    x, y, z = LaunchConfiguration('x'), LaunchConfiguration('y'), LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')
    # Apply offsets
    x_offset = OffsetParser(x, 0.157)
    yaw_offset = OffsetParser(yaw, 3.1416)

    state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='dock_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': Command(['xacro', ' ', dock_xacro_file])},
        ],
        remappings=[
            ('robot_description', 'standard_dock_description'),
        ],
    )

    spawn_model = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_standard_dock',
        arguments=['-entity',
                   'standard_dock',
                   '-topic',
                   'standard_dock_description',
                   '-x', x_offset,
                   '-y', y,
                   '-z', z,
                   '-Y', yaw_offset],
        output='screen',
    )

    tf_odom_std_dock_link_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_odom_std_dock_link_publisher',
        arguments=[x_offset, y, z,
                   # According to documentation (http://wiki.ros.org/tf2_ros):
                   # the order is yaw, pitch, roll
                   yaw_offset, '0', '0',
                   'odom', 'std_dock_link'],
        output='screen',
    )

    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)
    # Add nodes to LaunchDescription
    ld.add_action(state_publisher)
    ld.add_action(spawn_model)
    ld.add_action(tf_odom_std_dock_link_publisher)

    return ld
