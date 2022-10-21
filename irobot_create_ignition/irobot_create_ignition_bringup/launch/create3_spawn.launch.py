from ament_index_python.packages import get_package_share_directory

from launch import LaunchContext, LaunchDescription, SomeSubstitutionsType, Substitution
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

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

ARGUMENTS = [
    DeclareLaunchArgument('gazebo', default_value='ignition',
                          choices=['classic', 'ignition'],
                          description='Which gazebo simulator to use'),
    DeclareLaunchArgument('visualize_rays', default_value='false',
                          choices=['true', 'false'],
                          description='Enable/disable ray visualization'),
    DeclareLaunchArgument('world', default_value='depot',
                          description='World name'),
    DeclareLaunchArgument('robot_name', default_value='create3',
                          description='Create3 robot name'),
    DeclareLaunchArgument('robot_description', default_value='robot_description',
                          description='robot description topic name'),
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
    DeclareLaunchArgument('use_rviz', default_value='false',
                          choices=['true', 'false'], description='Start rviz.'),
    DeclareLaunchArgument('spawn_dock', default_value='true',
                          choices=['true', 'false'],
                          description='Spawn the standard dock model.'),
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
    rviz2_launch = PathJoinSubstitution(
        [pkg_irobot_create_common_bringup, 'launch', 'rviz2.launch.py'])
    dock_description_launch = PathJoinSubstitution(
        [pkg_irobot_create_common_bringup, 'launch', 'dock_description.launch.py'])

    # Launch configurations
    x, y, z = LaunchConfiguration('x'), LaunchConfiguration('y'), LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')
    robot_name = LaunchConfiguration('robot_name')
    namespace = LaunchConfiguration('namespace')
    namespaced_robot_description = [namespace, '/robot_description']
    namespaced_dock_description = [namespace, '/standard_dock_description']

    # Robot description
    robot_description_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([robot_description_launch]),
            launch_arguments={'gazebo': 'ignition', 'namespace': namespace}.items())

    # Dock description     
    x_dock = OffsetParser(x, 0.157)
    yaw_dock = OffsetParser(yaw, 3.1416)
    dock_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([dock_description_launch]),
        condition=IfCondition(LaunchConfiguration('spawn_dock')),
        # The robot starts docked
        launch_arguments={'x': x_dock, 'y': y, 'z': z, 'yaw': yaw_dock,
                          'namespace': namespace,
                          'gazebo': 'ignition'}.items()
    )

    rviz2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rviz2_launch]),
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        launch_arguments={'namespace': namespace}.items()
    )    

    # Spawn robot
    spawn_robot = Node(
            package='ros_ign_gazebo',
            executable='create',
            output='screen',
            arguments=[
                '-name', robot_name,
                '-topic', namespaced_robot_description,
                '-Y', yaw,
                '-x', x,
                '-y', y,
                '-z', z])

    # Dock
    spawn_dock = Node(package='ros_ign_gazebo',
                      executable='create',
                      output='screen',
                      arguments=['-name', (robot_name,'_standard_dock'),
                                 '-x', x_dock,
                                 '-y', y,
                                 '-z', z,
                                 '-Y', yaw_dock,
                                 '-topic', namespaced_dock_description],
                      )

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
    ld.add_action(spawn_dock)
    ld.add_action(dock_description)
    ld.add_action(ros_ign_bridge)
    ld.add_action(rviz2)
    ld.add_action(robot_description_launch)
    ld.add_action(create3_nodes)
    ld.add_action(create3_ignition_nodes)
    return ld
