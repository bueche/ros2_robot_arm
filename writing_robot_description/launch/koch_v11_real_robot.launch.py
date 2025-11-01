#!/usr/bin/env python3
"""
Launch file for koch_v11 robot with REAL Dynamixel hardware
Runs on RPi5 - RViz should be launched separately on Nvidia Nano
Follows the working pattern from koch_v11_simulation.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    
    # Get paths (SAME PATTERN as simulation)
    pkg_path = get_package_share_directory('writing_robot_description')
    urdf_file = os.path.join(pkg_path, 'urdf', 'koch_v11_arm_real.urdf')
    rviz_config_file = os.path.join(pkg_path, 'rviz', 'koch_v11_rviz_config.rviz')
    controllers_file = os.path.join(pkg_path, 'config', 'koch_v11_controllers_real.yaml')
    
    # Read URDF (EXACT pattern from working simulation)
    with open(urdf_file, 'r') as file:
        robot_description = file.read()
    
    # Robot State Publisher (SAME PATTERN as simulation)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False
        }]
    )
    
    # Controller Manager (SAME PATTERN as simulation)
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            controllers_file
        ],
        output='screen',
    )
    
    # Joint State Broadcaster Spawner
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )
    
    # Koch v11 Controller Spawner (delayed - waits for joint_state_broadcaster spawner to exit)
    koch_v11_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['koch_v11_controller'],
                    output='screen',
                )
            ],
        )
    )
    
    return LaunchDescription([
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        koch_v11_controller_spawner,
    ])
