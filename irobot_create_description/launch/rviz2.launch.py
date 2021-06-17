"""Launch Create3 in RViz."""

import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    pkg_create3_description = get_package_share_directory('irobot_create_description')

    urdf_dir = os.path.join(pkg_create3_description, 'urdf')
    xacro_file = os.path.join(urdf_dir, 'create3.urdf.xacro')

    # Rviz
    rviz_config_dir = os.path.join(
        pkg_create3_description,
        'rviz',
        'model.rviz')

    run_rviz = DeclareLaunchArgument("rviz", default_value="true",
                                     description="Start rviz")

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        condition=IfCondition(LaunchConfiguration("rviz")),
        output='screen')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': Command(['xacro',' ',xacro_file])}
        ],
    )

    joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration("rviz")),
        output='screen'
    )

    return LaunchDescription([run_rviz, joint_state_publisher, robot_state_publisher, rviz])
