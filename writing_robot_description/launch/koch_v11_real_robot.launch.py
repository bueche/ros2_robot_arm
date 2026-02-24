#!/usr/bin/env python3
"""
Launch file for koch_v11 robot with REAL Dynamixel hardware
Workaround: Set controller types directly in controller_manager parameters
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    
    # Get paths
    pkg_path = get_package_share_directory('writing_robot_description')
    urdf_file = os.path.join(pkg_path, 'urdf', 'koch_v11_arm_real.urdf')
    controllers_file = os.path.join(pkg_path, 'config', 'koch_v11_controllers_real.yaml')
    
    # Read URDF
    with open(urdf_file, 'r') as file:
        robot_description = file.read()
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False
        }]
    )
    
    # Controller Manager - explicitly set controller types
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {
                'robot_description': robot_description,
                'update_rate': 100,
                # Explicitly define controller types here
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
    
    # Joint State Broadcaster Spawner - with params file for controller-specific config
    joint_state_broadcaster_spawner = TimerAction(
        period=2.0,
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
    
    # Koch Controller Spawner
    koch_v11_controller_spawner = TimerAction(
        period=4.0,
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
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        koch_v11_controller_spawner,
    ])
