#!/usr/bin/env python3
"""
Koch v1.1 Simulation Launch
Following the exact pattern of the working 4-DOF simulation
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue  # CRITICAL IMPORT


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        )
    )
    
    # Get URDF via cat command (same as working 4-DOF)
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='cat')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('writing_robot_description'),
            'urdf',
            'koch_v11_arm.urdf'
        ])
    ])
    
    # Wrap in ParameterValue with value_type=str (CRITICAL!)
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }
    
    # Get controllers configuration
    robot_controllers = PathJoinSubstitution([
        FindPackageShare('writing_robot_description'),
        'config',
        'koch_v11_controllers.yaml'
    ])
    
    # Node 1: robot_state_publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[
            robot_description,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    # Node 2: controller_manager
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
    
    # Node 3: Spawn joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )
    
    # Node 4: Spawn koch_v11_controller
    koch_v11_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['koch_v11_controller'],
        output='screen'
    )
    
    # Delay controller spawning (same pattern as working 4-DOF)
    delay_joint_state_broadcaster_after_controller_manager = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=controller_manager_node,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )
    
    delay_koch_v11_controller_after_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[koch_v11_controller_spawner],
        )
    )
    
    # Optional: drawing visualizer
    drawing_visualizer_node = Node(
        package='writing_robot_description',
        executable='drawing_visualizer',
        name='drawing_visualizer',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    nodes_to_start = [
        robot_state_publisher_node,
        controller_manager_node,
        delay_joint_state_broadcaster_after_controller_manager,
        delay_koch_v11_controller_after_joint_state_broadcaster,
        drawing_visualizer_node,
    ]
    
    return LaunchDescription(declared_arguments + nodes_to_start)
