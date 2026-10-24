#!/usr/bin/env python3
"""
Launch a real robot arm with ros2_control, selected by arm profile.

    ros2 launch robot_arm_description robot_arm.launch.py arm:=koch_v1_1
    ros2 launch robot_arm_description robot_arm.launch.py arm:=so_arm101

Everything arm-specific comes from the profile's 'launch' section
(config/arm_profiles/<arm>.yaml): URDF, controllers file, nodes that must run
before the controller_manager, and how long to wait for them. The controller
to spawn is the profile's 'controller'.

Startup sequence:
  t=0                 robot_state_publisher, pre_launch nodes (e.g. servo_config_node)
  t=startup_delay_s   ros2_control_node, then one spawner that activates
                      joint_state_broadcaster and the arm controller in order

URDF text may use $(find <package>), which is replaced with that package's
share directory (the URDF is read as plain text, not processed by xacro).
"""

import os
import re

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PACKAGE = 'robot_arm_description'
PROFILE_SUBDIR = os.path.join('config', 'arm_profiles')


def _expand_find(text):
    return re.sub(r'\$\(find ([A-Za-z0-9_]+)\)', lambda m: get_package_share_directory(m.group(1)), text)


def _launch_setup(context):
    arm = LaunchConfiguration('arm').perform(context)
    share = get_package_share_directory(PACKAGE)

    profile_path = arm if os.path.isfile(arm) else os.path.join(share, PROFILE_SUBDIR, f'{arm}.yaml')
    if not os.path.isfile(profile_path):
        raise RuntimeError(f"robot_arm.launch.py: no arm profile '{arm}' (looked for {profile_path})")
    with open(profile_path, 'r') as f:
        profile = yaml.safe_load(f) or {}

    try:
        controller = profile['controller']
        cfg = profile['launch']
        urdf_file = os.path.join(share, cfg['urdf'])
        controllers_file = os.path.join(share, cfg['controllers'])
    except KeyError as e:
        raise RuntimeError(f"robot_arm.launch.py: {profile_path} is missing {e} (needs 'controller' and "
                           f"'launch.urdf' / 'launch.controllers')")
    pre_launch = cfg.get('pre_launch') or []
    startup_delay_s = float(cfg.get('startup_delay_s', 0.0))

    with open(urdf_file, 'r') as f:
        robot_description = _expand_find(f.read())

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': False}],
    )

    pre_launch_nodes = [
        Node(package=n['package'], executable=n['executable'], output='screen')
        for n in pre_launch
    ]

    # controller_manager reads robot_description from robot_state_publisher's topic,
    # and passes controllers_file on to every controller it loads.
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controllers_file],
        output='screen',
    )

    # One spawner, controllers activated in order.
    spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', controller, '--controller-manager', '/controller_manager'],
        output='screen',
    )

    control = [ros2_control_node, spawner]
    if startup_delay_s > 0.0:
        control = [TimerAction(period=startup_delay_s, actions=control)]

    return [robot_state_publisher, *pre_launch_nodes, *control]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'arm', default_value='koch_v1_1',
            description='Arm profile name (config/arm_profiles/<arm>.yaml) or path to a profile file'),
        OpaqueFunction(function=_launch_setup),
    ])
