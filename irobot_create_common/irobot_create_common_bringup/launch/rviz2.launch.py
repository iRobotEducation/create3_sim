# Copyright 2021 iRobot Corporation. All Rights Reserved.
# @author Rodrigo Jose Causarano Nunez (rcausaran@irobot.com)
#
# Launch RViz.

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString

ARGUMENTS = [
    DeclareLaunchArgument('namespace', default_value='',
                          description='robot namespace')
]


def generate_launch_description():
    create_bringup = get_package_share_directory(
        'irobot_create_common_bringup')

    namespace = LaunchConfiguration('namespace')

    # Rviz
    rviz_config = PathJoinSubstitution(
        [create_bringup, 'rviz', 'irobot_create_view.rviz'])
    rviz_logo = PathJoinSubstitution(
        [create_bringup, 'rviz', 'irobot_logo.jpg'])

    namespaced_rviz_config_file = ReplaceString(
        source_file=rviz_config,
        replacements={'/robot_description': ('/', namespace, '/robot_description'),
                      '/standard_dock_description': ('/', namespace, '/standard_dock_description'),
                      '/initialpose': ('/', namespace, '/initialpose'),
                      '/clicked_point': ('/', namespace, '/clicked_point'),
                      '/goal_pose': ('/', namespace, '/goal_pose')}
    )

    rviz = Node(
        condition=LaunchConfigurationEquals('namespace', ''),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '--display-config', rviz_config,
            '--splash-screen', rviz_logo,
        ]
    )

    rviz_namespaced = Node(
        condition=LaunchConfigurationNotEquals('namespace', ''),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '--display-config', namespaced_rviz_config_file,
            '--splash-screen', rviz_logo,
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('/goal_pose', 'goal_pose'),
            ('/clicked_point', 'clicked_point'),
            ('/initialpose', 'initialpose')
        ]
    )

    # Define LaunchDescription variable
    ld = LaunchDescription()
    # Add nodes to LaunchDescription
    ld.add_action(rviz)
    ld.add_action(rviz_namespaced)

    return ld
