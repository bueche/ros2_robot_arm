from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():
    
    # Get paths
    pkg_path = get_package_share_directory('writing_robot_description')
    urdf_file = os.path.join(pkg_path, 'urdf', 'robot_arm_ros2control.urdf')
    rviz_config_file = os.path.join(pkg_path, 'rviz', 'view_robot.rviz')
    controllers_file = os.path.join(pkg_path, 'config', 'controllers.yaml')
    
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
    
    # Controller Manager
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
    
    # Arm Controller Spawner (delayed)
    arm_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['arm_controller'],
                    output='screen',
                )
            ],
        )
    )
    
    # Drawing Visualizer - shows pen trail and writing surface
    drawing_visualizer = Node(
        package='writing_robot_description',
        executable='drawing_visualizer',
        output='screen',
        parameters=[{
            'surface_x': 0.355,       # Pen draws at X = 0.225-0.275, center = 0.25
            'surface_y': 0.005, # -0.01,      # Surface BEHIND the pen (pen is at Y=0)
            'surface_z': 0.155,       # Pen Z = 0.125-0.175, center = 0.15
            'surface_width': 0.28,   # Width in X: 0.275-0.225 = 0.05, add margin = 0.06
            'surface_height': 0.28,  # Height in Z: 0.175-0.125 = 0.05, add margin = 0.06
            'surface_angle': 1.57,    #  0.0 = No rotation, 1.57 = 90 degrees - vertical surface
            'surface_rotation_axis': 'z_axis',
        }]
    )
    
    # RViz (delayed start)
    rviz = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
                output='screen',
            )
        ]
    )
    
    return LaunchDescription([
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        drawing_visualizer,
        rviz,
    ])
