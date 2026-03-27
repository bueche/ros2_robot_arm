#!/usr/bin/env python3
"""
Launch file for koch_v11 robot with REAL Dynamixel hardware.

Startup sequence:
  t=0s   servo_config_node  — writes PID gains + motion profiles to all servos
  t=3s   controller_manager — acquires /dev/ttyOpenRB
  t=5s   joint_state_broadcaster spawner
  t=7s   koch_v11_controller spawner
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():

    # Get paths
    pkg_path = get_package_share_directory('writing_robot_description')
    urdf_file        = os.path.join(pkg_path, 'urdf',   'koch_v11_arm_real_for_balance.urdf')
    controllers_file = os.path.join(pkg_path, 'config', 'koch_v11_controllers_real.yaml')
    #servo_config_node = os.path.join(pkg_path, 'scripts', 'servo_config_node.py')

    # Read URDF
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    #  1. Servo configurator — runs immediately, exits when done 
    # Opens /dev/ttyOpenRB directly, writes gains, closes port, exits.
    # Controller manager is delayed 3 s to ensure this completes first.
    servo_configurator = Node(
        package='writing_robot_description',
        executable='servo_config_node',
        output='screen',
    )

    #  2. Robot State Publisher 
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
        }]
    )

    #  3. Controller Manager — delayed to let servo_config finish 
    controller_manager = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='controller_manager',
                executable='ros2_control_node',
                parameters=[
                    {
                        'robot_description': robot_description,
                        'update_rate': 100,
                        'joint_state_broadcaster': {
                            'type': 'joint_state_broadcaster/JointStateBroadcaster',
                        },
                        'koch_v11_controller': {
                            'type': 'joint_trajectory_controller/JointTrajectoryController',
                        },
                    },
                ],
                output='screen',
            )
        ]
    )

    #  4. Joint State Broadcaster Spawner 
    joint_state_broadcaster_spawner = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'joint_state_broadcaster',
                    '-p', controllers_file,
                ],
                output='screen',
            )
        ]
    )

    #  5. Koch Controller Spawner 
    koch_v11_controller_spawner = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'koch_v11_controller',
                    '-p', controllers_file,
                ],
                output='screen',
            )
        ]
    )

    return LaunchDescription([
        servo_configurator,
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        koch_v11_controller_spawner,
    ])
