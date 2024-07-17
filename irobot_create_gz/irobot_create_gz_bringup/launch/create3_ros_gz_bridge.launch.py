# Copyright 2021 Clearpath Robotics, Inc.
# @author Roni Kreinin (rkreinin@clearpathrobotics.com)

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='Use sim time'),
    DeclareLaunchArgument('robot_name', default_value='create3',
                          description='Ignition model name'),
    DeclareLaunchArgument('dock_name', default_value='standard_dock',
                          description='Ignition model name'),
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
    DeclareLaunchArgument('world', default_value='depot',
                          description='World name')
]


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_name = LaunchConfiguration('robot_name')
    dock_name = LaunchConfiguration('dock_name')
    namespace = LaunchConfiguration('namespace')
    world = LaunchConfiguration('world')

    cliff_sensors = [
        'cliff_front_left',
        'cliff_front_right',
        'cliff_side_left',
        'cliff_side_right',
    ]

    ir_intensity_sensors = [
        'ir_intensity_front_center_left',
        'ir_intensity_front_center_right',
        'ir_intensity_front_left',
        'ir_intensity_front_right',
        'ir_intensity_left',
        'ir_intensity_right',
        'ir_intensity_side_left',
    ]

    # cmd_vel bridge
    cmd_vel_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='cmd_vel_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        arguments=[
            [namespace,
             '/cmd_vel' + '@geometry_msgs/msg/Twist' + '[ignition.msgs.Twist'],
            ['/model/', robot_name, '/cmd_vel' +
             '@geometry_msgs/msg/Twist' +
             ']ignition.msgs.Twist']
        ],
        remappings=[
            ([namespace, '/cmd_vel'], 'cmd_vel'),
            (['/model/', robot_name, '/cmd_vel'],
             'diffdrive_controller/cmd_vel_unstamped')
        ])

    # Pose bridge
    pose_bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
                       name='pose_bridge',
                       output='screen',
                       parameters=[{
                            'use_sim_time': use_sim_time
                       }],
                       arguments=[
                           ['/model/', robot_name, '/pose' +
                            '@tf2_msgs/msg/TFMessage' +
                            '[ignition.msgs.Pose_V'],
                           ['/model/', dock_name, '/pose' +
                            '@tf2_msgs/msg/TFMessage' +
                            '[ignition.msgs.Pose_V']
                       ],
                       remappings=[
                           (['/model/', robot_name, '/pose'],
                            '_internal/sim_ground_truth_pose'),
                           (['/model/', dock_name, '/pose'],
                            '_internal/sim_ground_truth_dock_pose')
                       ])

    # odom to base_link transform bridge
    odom_base_tf_bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
                               name='odom_base_tf_bridge',
                               output='screen',
                               parameters=[{
                                   'use_sim_time': use_sim_time
                               }],
                               arguments=[
                                   ['/model/', robot_name, '/tf' +
                                    '@tf2_msgs/msg/TFMessage' +
                                    '[ignition.msgs.Pose_V']
                               ],
                               remappings=[
                                   (['/model/', robot_name, '/tf'], 'tf')
                               ])

    # Bumper contact sensor bridge
    bumper_contact_bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
                                 name='bumper_contact_bridge',
                                 output='screen',
                                 parameters=[{
                                     'use_sim_time': use_sim_time
                                 }],
                                 arguments=[
                                     [namespace,
                                      '/bumper_contact' +
                                      '@ros_gz_interfaces/msg/Contacts' +
                                      '[ignition.msgs.Contacts']
                                 ],
                                 remappings=[
                                     ([namespace,
                                      '/bumper_contact'],
                                      'bumper_contact')
                                 ])

    # Cliff bridge
    cliff_bridges = GroupAction([
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             name=cliff + '_bridge',
             output='screen',
             parameters=[{
                 'use_sim_time': use_sim_time,
                 'lazy': True
             }],
             arguments=[
                 ['/world/', world,
                  '/model/', robot_name,
                  '/link/base_link/sensor/' + cliff + '/scan' +
                  '@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan']
             ],
             remappings=[
                 (['/world/', world,
                     '/model/', robot_name,
                     '/link/base_link/sensor/' + cliff + '/scan'],
                     '_internal/' + cliff + '/scan')
             ]) for cliff in cliff_sensors
    ])

    # IR intensity bridges
    ir_bridges = GroupAction([
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             name=ir + '_bridge',
             output='screen',
             parameters=[{
                 'use_sim_time': use_sim_time,
                 'lazy': True
             }],
             arguments=[
                 ['/world/', world,
                  '/model/', robot_name,
                  '/link/' + ir + '/sensor/' + ir + '/scan' +
                  '@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan']
             ],
             remappings=[
                 (['/world/', world,
                     '/model/', robot_name,
                     '/link/' + ir + '/sensor/' + ir + '/scan'],
                  '_internal/' + ir + '/scan')
             ]) for ir in ir_intensity_sensors
    ])

    # Buttons message bridge
    buttons_msg_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='buttons_msg_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        arguments=[
            [namespace, '/create3_buttons' +
             '@std_msgs/msg/Int32' +
             '[ignition.msgs.Int32'],
        ],
        remappings=[
            ([namespace, '/create3_buttons'], '_internal/create3_buttons'),
        ]
    )

    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(cmd_vel_bridge)
    ld.add_action(pose_bridge)
    ld.add_action(odom_base_tf_bridge)
    ld.add_action(bumper_contact_bridge)
    ld.add_action(cliff_bridges)
    ld.add_action(ir_bridges)
    ld.add_action(buttons_msg_bridge)
    return ld
