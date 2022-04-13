from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument('gazebo', default_value='classic',
                          choices=['classic', 'ignition'],
                          description='Which gazebo simulator to use'),
    DeclareLaunchArgument('visualize_rays', default_value='false',
                          choices=['true', 'false'],
                          description='Enable/disable ray visualization'),
    DeclareLaunchArgument('robot_name', default_value='create3',
                          description='Create3 robot name'),
    DeclareLaunchArgument('robot_description', default_value='robot_description',
                          description='robot description topic name'),
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace')
]

for pose_element in ['x', 'y', 'z', 'yaw']:
    ARGUMENTS.append(DeclareLaunchArgument(pose_element, default_value='0.0',
                     description=f'{pose_element} component of the robot pose.'))


def generate_launch_description():

    pkg_create3_common_bringup = get_package_share_directory(
        'irobot_create_common_bringup')
    pkg_irobot_create_common_bringup = get_package_share_directory(
        'irobot_create_common_bringup')
    pkg_irobot_create_ignition_bringup = get_package_share_directory(
        'irobot_create_ignition_bringup')

    robot_description_launch = PathJoinSubstitution(
        [pkg_create3_common_bringup, 'launch', 'robot_description.launch.py'])
    ros_ign_bridge_launch = PathJoinSubstitution(
        [pkg_irobot_create_ignition_bringup, 'launch', 'create3_ros_ignition_bridge.launch.py'])
    create3_nodes_launch = PathJoinSubstitution(
        [pkg_irobot_create_common_bringup, 'launch', 'create3_nodes.launch.py'])
    create3_ignition_nodes_launch = PathJoinSubstitution(
        [pkg_irobot_create_ignition_bringup, 'launch', 'create3_ignition_nodes.launch.py'])
    robot_description_launch = PathJoinSubstitution(
        [pkg_irobot_create_common_bringup, 'launch', 'robot_description.launch.py'])

    # Launch configurations
    x, y, z = LaunchConfiguration('x'), LaunchConfiguration(
        'y'), LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')
    robot_name = LaunchConfiguration('robot_name')
    robot_description = LaunchConfiguration('robot_description')
    namespace = LaunchConfiguration('namespace')

    # Spawn robot
    spawn_robot = Node(
            package='ros_ign_gazebo',
            executable='create',
            output='screen',
            arguments=[
                '-name', robot_name,
                '-topic', robot_description,
                '-Y', yaw,
                '-x', x,
                '-y', y,
                '-z', z])

    # Robot description
    robot_description_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([robot_description_launch]),
            launch_arguments={'gazebo': 'ignition', 'namespace': namespace}.items())

    # ROS Ign bridge
    ros_ign_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ros_ign_bridge_launch]),
        launch_arguments=[('world', LaunchConfiguration('world')),
                          ('robot_name', robot_name),
                          ('namespace', namespace)]

    )

    # Create3 nodes
    create3_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([create3_nodes_launch]),
        launch_arguments=[('namespace', namespace)]
                )

    create3_ignition_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([create3_ignition_nodes_launch]),
        launch_arguments=[('robot_name', LaunchConfiguration('robot_name')),
                          ('namespace', namespace)]
    )

    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(spawn_robot)
    ld.add_action(ros_ign_bridge)
    ld.add_action(robot_description_launch)
    ld.add_action(create3_nodes)
    ld.add_action(create3_ignition_nodes)
    return ld
