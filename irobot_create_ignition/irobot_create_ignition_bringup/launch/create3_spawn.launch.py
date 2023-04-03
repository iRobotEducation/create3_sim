# Copyright 2021 Clearpath Robotics, Inc.
# @author Roni Kreinin (rkreinin@clearpathrobotics.com)

from ament_index_python.packages import get_package_share_directory

from irobot_create_common_bringup.namespace import GetNamespacedName
from irobot_create_common_bringup.offset import OffsetParser, RotationalOffsetX, RotationalOffsetY

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace


ARGUMENTS = [
    DeclareLaunchArgument('bridge', default_value='true',
                          choices=['true', 'false'],
                          description='Use ros_ign_bridge'),
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='use_sim_time'),
    DeclareLaunchArgument('world', default_value='depot',
                          description='Ignition World'),
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
    DeclareLaunchArgument('use_rviz', default_value='true',
                          choices=['true', 'false'], description='Start rviz.'),
    DeclareLaunchArgument('spawn_dock', default_value='true',
                          choices=['true', 'false'],
                          description='Spawn the standard dock model.'),
]

for pose_element in ['x', 'y', 'z', 'yaw']:
    ARGUMENTS.append(DeclareLaunchArgument(pose_element, default_value='0.0',
                     description=f'{pose_element} component of the robot pose.'))


def generate_launch_description():

    # Directories
    pkg_irobot_create_common_bringup = get_package_share_directory(
        'irobot_create_common_bringup')
    pkg_irobot_create_ignition_bringup = get_package_share_directory(
        'irobot_create_ignition_bringup')

    # Paths
    ros_ign_bridge_launch = PathJoinSubstitution(
        [pkg_irobot_create_ignition_bringup, 'launch', 'create3_ros_ignition_bridge.launch.py'])
    create3_nodes_launch = PathJoinSubstitution(
        [pkg_irobot_create_common_bringup, 'launch', 'create3_nodes.launch.py'])
    create3_ignition_nodes_launch = PathJoinSubstitution(
        [pkg_irobot_create_ignition_bringup, 'launch', 'create3_ignition_nodes.launch.py'])
    robot_description_launch = PathJoinSubstitution(
        [pkg_irobot_create_common_bringup, 'launch', 'robot_description.launch.py'])
    dock_description_launch = PathJoinSubstitution(
        [pkg_irobot_create_common_bringup, 'launch', 'dock_description.launch.py'])
    rviz2_launch = PathJoinSubstitution(
        [pkg_irobot_create_common_bringup, 'launch', 'rviz2.launch.py'])

    # Launch configurations
    namespace = LaunchConfiguration('namespace')
    x, y, z = LaunchConfiguration('x'), LaunchConfiguration('y'), LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')

    robot_name = GetNamespacedName(namespace, 'create3')
    dock_name = GetNamespacedName(namespace, 'standard_dock')

    # Calculate dock offset due to yaw rotation
    dock_offset_x = RotationalOffsetX(0.157, yaw)
    dock_offset_y = RotationalOffsetY(0.157, yaw)
    # Spawn dock at robot position + rotational offset
    x_dock = OffsetParser(x, dock_offset_x)
    y_dock = OffsetParser(y, dock_offset_y)
    # Rotate dock towards robot
    yaw_dock = OffsetParser(yaw, 3.1416)

    spawn_robot_group_action = GroupAction([
        PushRosNamespace(namespace),

        # Dock description
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([dock_description_launch]),
            condition=IfCondition(LaunchConfiguration('spawn_dock')),
            # The robot starts docked
            launch_arguments={'gazebo': 'ignition'}.items(),
        ),

        # Robot description
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([robot_description_launch]),
            launch_arguments={'gazebo': 'ignition'}.items()
        ),

        # Spawn Create 3
        Node(
            package='ros_ign_gazebo',
            executable='create',
            arguments=['-name', robot_name,
                       '-x', x,
                       '-y', y,
                       '-z', z,
                       '-Y', yaw,
                       '-topic', 'robot_description'],
            output='screen',
        ),

        # Spawn dock
        Node(
            package='ros_ign_gazebo',
            executable='create',
            arguments=['-name', dock_name,
                       '-x', x_dock,
                       '-y', y_dock,
                       '-z', z,
                       '-Y', yaw_dock,
                       '-topic', 'standard_dock_description'],
            output='screen',
            condition=IfCondition(LaunchConfiguration('spawn_dock'))
        ),

        # ROS Ign Bridge
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ros_ign_bridge_launch]),
            launch_arguments=[
                ('world', LaunchConfiguration('world')),
                ('robot_name', robot_name),
                ('dock_name', dock_name),
            ]
        ),

        # Create 3 nodes
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([create3_nodes_launch]),
            launch_arguments=[
                ('namespace', namespace)
            ]
        ),

        # Create 3 Ignition nodes
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([create3_ignition_nodes_launch]),
            launch_arguments=[
                ('robot_name', robot_name),
                ('dock_name', dock_name),
            ]
        ),

        # Rviz
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([rviz2_launch]),
            condition=IfCondition(LaunchConfiguration('use_rviz')),
        )
    ])

    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(spawn_robot_group_action)
    return ld
