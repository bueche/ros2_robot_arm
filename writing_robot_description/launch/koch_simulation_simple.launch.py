#!/usr/bin/env python3
"""
Simplified launch file for Koch v1.1 Writing Arm Simulation

This is a simpler version that's easier to debug.
Use this if the main koch_simulation.launch.py has issues.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('writing_robot_description')
    
    # Paths
    urdf_file = os.path.join(pkg_dir, 'urdf', 'koch_writing_arm.urdf')
    controllers_file = os.path.join(pkg_dir, 'config', 'koch_controllers.yaml')
    
    # Read URDF file
    with open(urdf_file, 'r') as f:
        robot_description = f.read()
    
    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # robot_state_publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # controller_manager (ros2_control_node)
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            controllers_file,
            {'use_sim_time': use_sim_time}
        ],
        output='both',
        remappings=[
            ('/controller_manager/robot_description', '/robot_description')
        ]
    )
    
    # Spawn joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    
    # Spawn joint_trajectory_controller (after joint_state_broadcaster)
    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    
    # Delay spawning controllers until controller_manager is ready
    delay_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=controller_manager_node,
            on_exit=[joint_state_broadcaster_spawner]
        )
    )
    
    delay_joint_trajectory_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[joint_trajectory_controller_spawner]
        )
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock'
        ),
        robot_state_publisher_node,
        controller_manager_node,
        delay_joint_state_broadcaster,
        delay_joint_trajectory_controller,
    ])
