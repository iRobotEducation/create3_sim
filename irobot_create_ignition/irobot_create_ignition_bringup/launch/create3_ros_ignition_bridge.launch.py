# Copyright 2021 Clearpath Robotics, Inc.
# @author Roni Kreinin (rkreinin@clearpathrobotics.com)

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='Use sim time'),
    DeclareLaunchArgument('robot_name', default_value='create3',
                          description='Ignition model name'),
    DeclareLaunchArgument('world', default_value='depot',
                          description='World name')
]


def generate_launch_description():
    namespace = LaunchConfiguration('robot_name')
    use_sim_time = LaunchConfiguration('use_sim_time')

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

    # clock bridge
    clock_bridge = Node(package='ros_ign_bridge', executable='parameter_bridge',
                        namespace=namespace,
                        name='clock_bridge',
                        output='screen',
                        arguments=[
                            '/clock' + '@rosgraph_msgs/msg/Clock' + '[ignition.msgs.Clock'
                        ],
                        condition=IfCondition(use_sim_time))

    # cmd_vel bridge
    cmd_vel_bridge = Node(package='ros_ign_bridge', executable='parameter_bridge',
                          name='cmd_vel_bridge',
                          output='screen',
                          parameters=[{
                              'use_sim_time': use_sim_time
                          }],
                          arguments=[
                              '/cmd_vel' + '@geometry_msgs/msg/Twist' + '[ignition.msgs.Twist',
                              ['/model/', LaunchConfiguration('robot_name'), '/cmd_vel' +
                               '@geometry_msgs/msg/Twist' +
                               ']ignition.msgs.Twist']
                          ],
                          remappings=[
                              (['/model/', LaunchConfiguration('robot_name'), '/cmd_vel'],
                               'diffdrive_controller/cmd_vel_unstamped')
                          ])

    # Pose bridge
    pose_bridge = Node(package='ros_ign_bridge', executable='parameter_bridge',
                       namespace=namespace,
                       name='pose_bridge',
                       output='screen',
                       parameters=[{
                            'use_sim_time': use_sim_time
                       }],
                       arguments=[
                           ['/model/', LaunchConfiguration('robot_name'), '/pose' +
                            '@tf2_msgs/msg/TFMessage' +
                            '[ignition.msgs.Pose_V'],
                           '/model/standard_dock/pose' +
                           '@tf2_msgs/msg/TFMessage' +
                           '[ignition.msgs.Pose_V'
                       ],
                       remappings=[
                           (['/model/', LaunchConfiguration('robot_name'), '/pose'],
                            '/_internal/sim_ground_truth_pose'),
                           ('/model/standard_dock/pose',
                            '/_internal/sim_ground_truth_dock_pose')
                       ])

    # odom to base_link transform bridge
    odom_base_tf_bridge = Node(package='ros_ign_bridge', executable='parameter_bridge',
                               namespace=namespace,
                               name='odom_base_tf_bridge',
                               output='screen',
                               parameters=[{
                                   'use_sim_time': use_sim_time
                               }],
                               arguments=[
                                   ['/model/', LaunchConfiguration('robot_name'), '/tf' +
                                    '@tf2_msgs/msg/TFMessage' +
                                    '[ignition.msgs.Pose_V']
                               ],
                               remappings=[
                                   (['/model/', LaunchConfiguration('robot_name'), '/tf'], '/tf')
                               ])

    # Bumper contact sensor bridge
    bumper_contact_bridge = Node(package='ros_ign_bridge', executable='parameter_bridge',
                                 namespace=namespace,
                                 name='bumper_contact_bridge',
                                 output='screen',
                                 parameters=[{
                                     'use_sim_time': use_sim_time
                                 }],
                                 arguments=[
                                     ['/model/', LaunchConfiguration('robot_name'),
                                      '/bumper_contact' +
                                      '@ros_ign_interfaces/msg/Contacts' +
                                      '[ignition.msgs.Contacts']
                                 ],
                                 remappings=[
                                     (['/model/', LaunchConfiguration('robot_name'),
                                      '/bumper_contact'],
                                      '/bumper_contact')
                                 ])

    # Cliff bridge
    cliff_bridge = Node(package='ros_ign_bridge', executable='parameter_bridge',
                        namespace=namespace,
                        name='cliff_bridge',
                        output='screen',
                        parameters=[{
                            'use_sim_time': use_sim_time
                        }],
                        arguments=[
                            ['/world/', LaunchConfiguration('world'),
                             '/model/', LaunchConfiguration('robot_name'),
                             '/link/base_link/sensor/' + cliff + '/scan' +
                             '@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan']
                            for cliff in cliff_sensors
                        ],
                        remappings=[
                            (['/world/', LaunchConfiguration('world'),
                              '/model/', LaunchConfiguration('robot_name'),
                              '/link/base_link/sensor/' + cliff + '/scan'],
                             '/_internal/' + cliff + '/scan')
                            for cliff in cliff_sensors
                        ])
    # IR intensity bridge
    ir_intensity_bridge = Node(package='ros_ign_bridge', executable='parameter_bridge',
                               namespace=namespace,
                               name='ir_intensity_bridge',
                               output='screen',
                               parameters=[{
                                   'use_sim_time': use_sim_time
                               }],
                               arguments=[
                                   ['/world/', LaunchConfiguration('world'),
                                    '/model/', LaunchConfiguration('robot_name'),
                                    '/link/' + ir + '/sensor/' + ir + '/scan' +
                                    '@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan']
                                   for ir in ir_intensity_sensors
                               ],
                               remappings=[
                                   (['/world/', LaunchConfiguration('world'),
                                     '/model/', LaunchConfiguration('robot_name'),
                                     '/link/' + ir + '/sensor/' + ir + '/scan'],
                                    '/_internal/' + ir + '/scan') for ir in ir_intensity_sensors
                               ])

    # Buttons message bridge
    buttons_msg_bridge = Node(package='ros_ign_bridge', executable='parameter_bridge',
                              namespace=namespace,
                              name='buttons_msg_bridge',
                              output='screen',
                              parameters=[{
                                   'use_sim_time': use_sim_time
                              }],
                              arguments=[
                                  ['/create3/buttons' +
                                   '@std_msgs/msg/Int32' +
                                   '[ignition.msgs.Int32']
                              ])

    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(clock_bridge)
    ld.add_action(cmd_vel_bridge)
    ld.add_action(pose_bridge)
    ld.add_action(odom_base_tf_bridge)
    ld.add_action(bumper_contact_bridge)
    ld.add_action(cliff_bridge)
    ld.add_action(ir_intensity_bridge)
    ld.add_action(buttons_msg_bridge)
    return ld
