"""
RViz display of an arm with joint_state_publisher_gui sliders (no hardware).

    ros2 launch robot_arm_description display_balance_launch.py
    ros2 launch robot_arm_description display_balance_launch.py arm:=so_arm101 \
        urdf:=urdf/so_arm101_for_balance.urdf

The URDF is the arm profile's launch.urdf (config/arm_profiles/<arm>.yaml),
or the 'urdf' argument (relative to the robot_arm_description share dir).
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PACKAGE = 'robot_arm_description'


def _launch_setup(context):
    arm = LaunchConfiguration('arm').perform(context)
    urdf_override = LaunchConfiguration('urdf').perform(context)
    share = get_package_share_directory(PACKAGE)

    if urdf_override:
        urdf_rel = urdf_override
    else:
        profile_path = arm if os.path.isfile(arm) else os.path.join(
            share, 'config', 'arm_profiles', f'{arm}.yaml')
        if not os.path.isfile(profile_path):
            raise RuntimeError(f"display_balance_launch.py: no arm profile '{arm}' "
                               f"(looked for {profile_path})")
        with open(profile_path, 'r') as f:
            profile = yaml.safe_load(f) or {}
        try:
            urdf_rel = profile['launch']['urdf']
        except (KeyError, TypeError):
            raise RuntimeError(f"display_balance_launch.py: {profile_path} has no launch.urdf")
    urdf_file = os.path.join(share, urdf_rel)

    # Read the URDF file
    with open(urdf_file, 'r') as file:
        robot_description = file.read()

    # Robot State Publisher - publishes TF transforms
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'publish_frequency': 30.0
        }]
    )
    
    # Joint State Publisher GUI - lets you move joints with sliders
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
    )
    
    # RViz - visualizes the robot
    rviz_config_file = os.path.join(
        get_package_share_directory('robot_arm_description'),
        'rviz',
        'view_robot.rviz'
    )
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else []
    )
    
    return [robot_state_publisher, joint_state_publisher_gui, rviz]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'arm', default_value='koch_v1_1',
            description='Arm profile name (config/arm_profiles/<arm>.yaml) or path to a profile file'),
        DeclareLaunchArgument(
            'urdf', default_value='',
            description="URDF path relative to the robot_arm_description share dir; "
                        "empty = the profile's launch.urdf"),
        OpaqueFunction(function=_launch_setup),
    ])
