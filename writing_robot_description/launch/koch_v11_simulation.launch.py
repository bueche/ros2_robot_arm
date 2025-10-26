from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    # Get paths
    pkg_path = get_package_share_directory('writing_robot_description')
    urdf_file = os.path.join(pkg_path, 'urdf', 'koch_v11_arm.urdf')
    rviz_config_file = os.path.join(pkg_path, 'rviz', 'koch_v11_rviz_config.rviz')
    controllers_file = os.path.join(pkg_path, 'config', 'koch_v11_controllers.yaml')
    
    # Read URDF (EXACT pattern from working 4-DOF)
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
    
    # Drawing Visualizer - shows pen trail and writing surface
    drawing_visualizer = Node(
        package='writing_robot_description',
        executable='drawing_visualizer',
        output='screen',
        parameters=[{
            'surface_x': 0.30,        # Moved forward to match pen drawing position
            'surface_y': 0.0,         # Centered
            'surface_z': 0.20,        # Match square center height
            'surface_width': 0.12,    # Wider to see full square
            'surface_height': 0.12,   # Taller to see full square
            'surface_angle': 1.57,    # 90 degrees - vertical surface
            'surface_rotation_axis': 'z_axis',
        }]
    )
    
    # RViz (optional - uncomment to enable)
    '''
    rviz = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
                output='screen',
            )
        ]
    )
    '''
    
    return LaunchDescription([
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        koch_v11_controller_spawner,
        drawing_visualizer,
        # rviz,
    ])
