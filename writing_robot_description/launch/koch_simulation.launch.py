#!/usr/bin/env python3
"""
Launch file for Koch v1.1 Writing Arm Simulation

This launch file starts:
1. robot_state_publisher (publishes robot description and TF)
2. ros2_control controller_manager (manages hardware interface)
3. Joint trajectory controller (for motion control)
4. Joint state broadcaster (publishes joint states)
5. Drawing visualizer (shows pen trail and surface)

Note: RViz is not launched here for remote display compatibility
Run RViz separately on remote machine if desired
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock'
        )
    )
    
    # Get URDF path
    urdf_file_path = PathJoinSubstitution([
        FindPackageShare('writing_robot_description'),
        'urdf',
        'koch_writing_arm.urdf'
    ])
    
    # Read URDF content and wrap as ParameterValue with type string
    robot_description_content = ParameterValue(
        Command(['cat ', urdf_file_path]),
        value_type=str
    )
    
    robot_description = {'robot_description': robot_description_content}
    
    # Get controller configuration
    robot_controllers = PathJoinSubstitution([
        FindPackageShare('writing_robot_description'),
        'config',
        'koch_controllers.yaml'  # New Koch controllers
    ])
    
    # Node: robot_state_publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[
            robot_description,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    # Node: controller_manager
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            robot_description,
            robot_controllers,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='both'
    )
    
    # Spawn joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen'
    )
    
    # Spawn joint_trajectory_controller
    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_trajectory_controller',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen'
    )
    
    # Delay controller spawners until controller_manager is ready
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
    
    # Node: Drawing visualizer (updated for Koch joints)
    drawing_visualizer_node = Node(
        package='writing_robot_description',
        executable='drawing_visualizer',
        name='drawing_visualizer',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            # Koch arm parameters (update after measuring your robot!)
            'shoulder_height': 0.04,
            'upper_arm_length': 0.20,
            'forearm_length': 0.16,
            'wrist_to_pen': 0.15,
            # Writing surface parameters
            'surface_x': 0.355,
            'surface_y': 0.0,
            'surface_z': 0.20,
            'surface_rotation_axis': 'z_axis',
            'surface_angle': 1.57,  # 90 degrees
            # Visualization parameters
            'pen_down_threshold': 0.02,  # Distance to consider pen "down"
            'trail_lifetime': 0.0,  # 0 = permanent trail
        }]
    )
    
    nodes = [
        robot_state_publisher_node,
        controller_manager_node,
        delay_joint_state_broadcaster,
        delay_joint_trajectory_controller,
        drawing_visualizer_node,
    ]
    
    return LaunchDescription(declared_arguments + nodes)
