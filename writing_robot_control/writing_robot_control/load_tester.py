#!/usr/bin/env python3
"""
Unified Load Testing Tool for Koch v1.1 - v6

Features:
- Basic load testing (torque predictions)
- Trajectory analysis (dynamic torque including acceleration, friction, back-EMF)
- Full pose-based analysis with forward kinematics (ACCURATE moment arms)
- YAML pose sequence analysis (test entire sequences from pose_test.py)
- What-if analysis (test different servos)
- Servo telemetry (temp/current/voltage - if dynamixel_sdk available)
- Comparison matrix (compare all servo options)
- XL330-M077-T support (more available than M288)

New in v6:
- Forward kinematics for accurate torque calculations
- Full arm configuration considered (not just single joint)
- Analyze complete pose transitions
- Read and analyze YAML pose sequences directly

Usage:
  # Basic tests (v5 compatibility)
  ros2 run writing_robot_control load_tester --full_analysis
  ros2 run writing_robot_control load_tester --joint shoulder_lift --static
  
  # Trajectory analysis
  ros2 run writing_robot_control load_tester --joint shoulder_lift --static \\
    --previous-position 0.0 --movement-time 2.0
  
  # Full pose transition (NEW!)
  ros2 run writing_robot_control load_tester --analyze-transition \\
    --pose-from "0.0,1.57,0.0,0.0,0.0,0.0" \\
    --pose-to "0.0,1.57,1.57,0.0,0.0,0.0" \\
    --movement-time 2.0
  
  # YAML sequence analysis (NEW!)
  ros2 run writing_robot_control load_tester --analyze-sequence poses.yaml \\
    --movement-time 2.0
  
  # What-if scenarios
  ros2 run writing_robot_control load_tester --joint shoulder_lift --what-if XL330-M077
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time
import argparse
import sys
import numpy as np
import yaml
import csv
from datetime import datetime
import os

# Optional: Try to import Dynamixel SDK for telemetry
DYNAMIXEL_AVAILABLE = False
try:
    from dynamixel_sdk import *
    DYNAMIXEL_AVAILABLE = True
except ImportError:
    pass


class LoadTester(Node):
    """Unified load testing with what-if analysis and optional telemetry."""
    
    def __init__(self):
        super().__init__('load_tester')
        
        # Joint configuration
        self.joint_names = [
            'shoulder_pan',
            'shoulder_lift', 
            'elbow_flex',
            'wrist_flex',
            'wrist_roll',
            'pen_holder'
        ]
        
        # Expanded servo specs database
        self.servo_specs = {
            'XL330-M077': {
                'stall_torque': 0.26,  # Nm at 5.0V
                'no_load_speed': 70,   # rpm (faster but weaker)
                'mass': 0.018,  # kg (lighter!)
                'voltage': [3.0, 5.5],  # V (min, max)
                'stall_current': 0.9,  # A
                'cost': 20,  # USD (cheaper and more available)
                'gear_ratio': 77  # For reference
            },
            'XL330-M288': {
                'stall_torque': 0.39,  # Nm at 5.0V
                'no_load_speed': 46,   # rpm
                'mass': 0.029,  # kg
                'voltage': [3.0, 5.5],  # V (min, max)
                'stall_current': 1.2,  # A
                'cost': 24,  # USD
                'gear_ratio': 288
            },
            'XL430-W210': {
                'stall_torque': 1.0,  # Nm at 11.1V
                'no_load_speed': 57,  # rpm
                'mass': 0.059,  # kg
                'voltage': [6.5, 12.0],
                'stall_current': 1.4,
                'cost': 48
            },
            'XL430-W250': {
                'stall_torque': 1.4,  # Nm at 11.1V
                'no_load_speed': 47,  # rpm
                'mass': 0.059,  # kg
                'voltage': [6.5, 12.0],
                'stall_current': 1.4,
                'cost': 50
            },
            'XM430-W210': {
                'stall_torque': 2.7,  # Nm at 12V
                'no_load_speed': 70,  # rpm
                'mass': 0.082,  # kg
                'voltage': [10.0, 14.8],
                'stall_current': 2.5,
                'cost': 200
            },
            'XM430-W350': {
                'stall_torque': 4.1,  # Nm at 12V
                'no_load_speed': 46,  # rpm
                'mass': 0.082,  # kg
                'voltage': [10.0, 14.8],
                'stall_current': 2.5,
                'cost': 200
            }
        }
        
        # Current robot configuration
        self.current_servos = {
            'shoulder_pan': 'XL430-W250',
            'shoulder_lift': 'XL430-W250',
            'elbow_flex': 'XL330-M288',
            'wrist_flex': 'XL330-M288',
            'wrist_roll': 'XL330-M288',
            'pen_holder': 'XL330-M288'
        }
        
        # Servo ID mapping (for telemetry)
        self.servo_ids = {
            'shoulder_pan': 1,
            'shoulder_lift': 2,
            'elbow_flex': 3,
            'wrist_flex': 4,
            'wrist_roll': 5,
            'pen_holder': 6
        }
        
        # Link masses (from URDF inertial data - koch_v11_arm_real.urdf)
        # Extracted from actual STL files via stl_inertia_from_urdf.py
        self.link_masses = {
            'base_link': 0.090,      # 89.9g (base + XL430 servo)
            'shoulder_link': 0.062,  # 61.6g (bracket + XL430 servo)
            'upper_arm_link': 0.072, # 71.6g (arm segment + mounting)
            'forearm_link': 0.088,   # 88.5g (forearm + servo)
            'wrist_link': 0.035,     # 34.6g (wrist assembly)
            'hand_link': 0.046,      # 45.6g (hand bracket)
            'pen_link': 0.016        # 15.9g (pen holder + servo)
        }
        
        # Link COM distances from joint (for v5 backward compatibility)
        # NOTE: v6 calculate_torque_with_fk() uses forward kinematics instead!
        # This dictionary is ONLY used by the old calculate_static_torque() method.
        # 
        # Values below are approximate - v6 FK calculates exact positions.
        # Updated with actual URDF inertial data from koch_v11_arm_real.urdf
        self.link_com_distances = {
            'shoulder_pan': {
                'shoulder_link': 0.038,   # From URDF: 0.038m
                'upper_arm_link': 0.093,  # Approx (joint offset + upper arm)
                'forearm_link': 0.144,    # Approx
                'wrist_link': 0.245,      # Approx
                'hand_link': 0.288,       # Approx
                'pen_link': 0.303         # Approx
            },
            'shoulder_lift': {
                'upper_arm_link': 0.141,  # From URDF: COM at 0.141m from link origin
                'forearm_link': 0.210,    # Approx (upper arm length + forearm COM)
                'wrist_link': 0.287,      # Approx
                'hand_link': 0.323,       # Approx
                'pen_link': 0.340         # Approx
            },
            'elbow_flex': {
                'forearm_link': 0.101,    # From URDF: COM at 0.101m from link origin
                'wrist_link': 0.177,      # Approx (forearm length + wrist COM)
                'hand_link': 0.220,       # Approx
                'pen_link': 0.237         # Approx
            },
            'wrist_flex': {
                # Wrist joints have small masses, but include for completeness
                'wrist_link': 0.077,      # From URDF: COM at 0.077m from link origin
                'hand_link': 0.120,       # Approx (wrist length + hand COM)
                'pen_link': 0.137         # Approx
            },
            'wrist_roll': {
                'hand_link': 0.036,       # From URDF: COM at 0.036m from link origin
                'pen_link': 0.053         # Approx
            },
            'pen_holder': {
                'pen_link': 0.017         # From URDF: Distance to pen COM
            }
        }
        
        # Approximate link rotational inertias (for acceleration torque)
        # I = m * r^2 for point mass approximation
        # Updated with actual URDF masses and distances
        self.link_inertias = {
            'shoulder_lift': {
                'upper_arm_link': 0.072 * (0.141**2),  # 72g at 141mm
                'forearm_link': 0.088 * (0.210**2),    # 88g at ~210mm
                'wrist_link': 0.035 * (0.287**2),      # 35g at ~287mm
                'hand_link': 0.046 * (0.323**2),       # 46g at ~323mm
                'pen_link': 0.016 * (0.340**2)         # 16g at ~340mm
            },
            'elbow_flex': {
                'forearm_link': 0.088 * (0.101**2),    # 88g at 101mm
                'wrist_link': 0.035 * (0.177**2),      # 35g at ~177mm
                'hand_link': 0.046 * (0.220**2),       # 46g at ~220mm
                'pen_link': 0.016 * (0.237**2)         # 16g at ~237mm
            },
            'wrist_flex': {
                'wrist_link': 0.035 * (0.077**2),      # 35g at 77mm
                'hand_link': 0.046 * (0.120**2),       # 46g at ~120mm
                'pen_link': 0.016 * (0.137**2)         # 16g at ~137mm
            },
            'wrist_roll': {
                'hand_link': 0.046 * (0.036**2),       # 46g at 36mm
                'pen_link': 0.016 * (0.053**2)         # 16g at ~53mm
            },
            'pen_holder': {
                'pen_link': 0.016 * (0.017**2)         # 16g at 17mm
            }
        }
        
        # Subscribe to joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.current_positions = {}
        self.current_velocities = {}
        self.current_efforts = {}
        self.joint_states_received = False
        
        # Dynamixel SDK setup (optional)
        self.dxl_port = None
        self.dxl_packet = None
        
        if DYNAMIXEL_AVAILABLE:
            try:
                self.dxl_port = PortHandler('/dev/ttyOpenRB')
                self.dxl_packet = PacketHandler(2.0)
                if self.dxl_port.openPort() and self.dxl_port.setBaudRate(57600):
                    self.get_logger().info('✓ Dynamixel SDK connected (telemetry enabled)')
                else:
                    self.dxl_port = None
                    self.get_logger().warn('⚠️  Could not open Dynamixel port (telemetry disabled)')
            except:
                self.dxl_port = None
                self.get_logger().warn('⚠️  Dynamixel SDK error (telemetry disabled)')
        
        self.get_logger().info('Load Tester initialized')
    
    def joint_state_callback(self, msg):
        """Store current joint states."""
        for i, name in enumerate(msg.name):
            if name in self.joint_names:
                self.current_positions[name] = msg.position[i]
                if len(msg.velocity) > i:
                    self.current_velocities[name] = msg.velocity[i]
                if len(msg.effort) > i:
                    self.current_efforts[name] = msg.effort[i]
        self.joint_states_received = True
    
    def wait_for_joint_states(self, timeout=5.0):
        """Wait for joint states."""
        start = time.time()
        while not self.joint_states_received and (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.joint_states_received
    
    def calculate_static_torque(self, joint_name, joint_position, payload_mass=0.0):
        """Calculate required torque to hold position against gravity.
        
        NOTE: This is the simplified v5-compatible method.
        For accurate analysis, use calculate_torque_with_fk() which uses forward kinematics!
        """
        g = 9.81
        total_torque = 0.0
        
        if joint_name == 'shoulder_pan':
            return 0.0  # Vertical axis, no gravity load
        
        elif joint_name == 'shoulder_lift':
            links = ['upper_arm_link', 'forearm_link', 'wrist_link', 'hand_link', 'pen_link']
            
            for link in links:
                mass = self.link_masses.get(link, 0.0)
                distance = self.link_com_distances['shoulder_lift'].get(link, 0.0)
                angle_from_vertical = joint_position - np.pi/2
                torque = mass * g * distance * np.abs(np.sin(angle_from_vertical))
                total_torque += torque
            
            if payload_mass > 0:
                payload_distance = self.link_com_distances['shoulder_lift']['pen_link']
                total_torque += payload_mass * g * payload_distance * np.abs(np.sin(angle_from_vertical))
        
        elif joint_name == 'elbow_flex':
            links = ['forearm_link', 'wrist_link', 'hand_link', 'pen_link']
            
            for link in links:
                mass = self.link_masses.get(link, 0.0)
                distance = self.link_com_distances['elbow_flex'].get(link, 0.0)
                torque = mass * g * distance * 0.5  # Approximate
                total_torque += torque
            
            if payload_mass > 0:
                payload_distance = self.link_com_distances['elbow_flex']['pen_link']
                total_torque += payload_mass * g * payload_distance * 0.5
        
        elif joint_name == 'wrist_flex':
            # Wrist joints have small masses, but calculate for completeness
            links = ['wrist_link', 'hand_link', 'pen_link']
            
            for link in links:
                mass = self.link_masses.get(link, 0.0)
                distance = self.link_com_distances['wrist_flex'].get(link, 0.0)
                torque = mass * g * distance * 0.3  # Approximate (small effect)
                total_torque += torque
            
            if payload_mass > 0:
                payload_distance = self.link_com_distances['wrist_flex']['pen_link']
                total_torque += payload_mass * g * payload_distance * 0.3
        
        elif joint_name == 'wrist_roll':
            # Roll axis - minimal gravity torque, but include for completeness
            links = ['hand_link', 'pen_link']
            
            for link in links:
                mass = self.link_masses.get(link, 0.0)
                distance = self.link_com_distances['wrist_roll'].get(link, 0.0)
                torque = mass * g * distance * 0.2  # Very approximate (roll axis)
                total_torque += torque
            
            if payload_mass > 0:
                payload_distance = self.link_com_distances['wrist_roll']['pen_link']
                total_torque += payload_mass * g * payload_distance * 0.2
        
        elif joint_name == 'pen_holder':
            # Pen holder joint - only pen_link downstream
            mass = self.link_masses.get('pen_link', 0.0)
            distance = self.link_com_distances['pen_holder'].get('pen_link', 0.0)
            torque = mass * g * distance * 0.2  # Very small effect
            total_torque += torque
            
            if payload_mass > 0:
                payload_distance = self.link_com_distances['pen_holder']['pen_link']
                total_torque += payload_mass * g * payload_distance * 0.2
        
        return total_torque
    
    def calculate_angular_distance(self, joint_name, pos_from, pos_to):
        """Calculate angular distance between two positions.
        
        Returns: (distance in radians, direction +/-)
        """
        distance = pos_to - pos_from
        # Normalize to [-pi, pi]
        while distance > np.pi:
            distance -= 2*np.pi
        while distance < -np.pi:
            distance += 2*np.pi
        return distance
    
    def calculate_acceleration_torque(self, joint_name, angular_distance, movement_time):
        """Calculate torque needed for acceleration/deceleration.
        
        This is τ = I × α where:
        - I = rotational inertia (kg⋅m²)
        - α = angular acceleration (rad/s²)
        
        Assumes trapezoidal velocity profile:
        - Accelerate for first 1/3 of time
        - Constant velocity for middle 1/3
        - Decelerate for final 1/3
        """
        if movement_time <= 0:
            return 0.0
        
        # Calculate peak angular velocity
        # For trapezoidal profile: total distance = 0.5 * (accel_time + const_time + decel_time) * peak_vel
        # With equal accel/const/decel phases: distance = (2/3) * movement_time * peak_vel
        peak_velocity = (3.0 / 2.0) * abs(angular_distance) / movement_time  # rad/s
        
        # Calculate angular acceleration
        # α = peak_vel / accel_time, where accel_time = movement_time / 3
        accel_time = movement_time / 3.0
        angular_accel = peak_velocity / accel_time  # rad/s²
        
        # Calculate total rotational inertia about this joint
        total_inertia = 0.0
        
        if joint_name in self.link_inertias:
            for link, inertia in self.link_inertias[joint_name].items():
                total_inertia += inertia
        
        # Torque = I × α
        accel_torque = total_inertia * angular_accel
        
        return abs(accel_torque)
    
    def calculate_dynamic_friction(self, joint_name, velocity):
        """Estimate dynamic friction torque.
        
        Dynamic friction occurs during motion and depends on:
        - Bearing friction (viscous: proportional to velocity)
        - Gear friction (roughly constant once moving)
        - Cable/wire friction
        
        This is approximate - real friction is complex!
        """
        # Approximate friction coefficients (Nm⋅s/rad for viscous, Nm for coulomb)
        friction_params = {
            'shoulder_pan': {'viscous': 0.01, 'coulomb': 0.02},
            'shoulder_lift': {'viscous': 0.015, 'coulomb': 0.03},
            'elbow_flex': {'viscous': 0.008, 'coulomb': 0.015},
            'wrist_flex': {'viscous': 0.005, 'coulomb': 0.01},
            'wrist_roll': {'viscous': 0.003, 'coulomb': 0.008},
            'pen_holder': {'viscous': 0.002, 'coulomb': 0.005}
        }
        
        params = friction_params.get(joint_name, {'viscous': 0.01, 'coulomb': 0.02})
        
        # Total friction = viscous + coulomb
        viscous_friction = params['viscous'] * abs(velocity)
        coulomb_friction = params['coulomb'] if abs(velocity) > 0.01 else 0
        
        return viscous_friction + coulomb_friction
    
    def calculate_back_emf_losses(self, joint_name, velocity, servo_type):
        """Estimate torque loss due to back-EMF.
        
        Back-EMF (Electromotive Force):
        When a motor spins, it acts like a generator and produces voltage
        that opposes the applied voltage. This reduces available torque.
        
        Relationship:
        - Torque available = Stall torque × (1 - speed/max_speed)
        - At 0 speed: Full stall torque available
        - At max speed: 0 torque available
        
        This is why servos are stronger at low speeds!
        """
        specs = self.servo_specs[servo_type]
        
        # Convert no-load speed to rad/s
        max_speed_rpm = specs['no_load_speed']
        max_speed_rad_s = max_speed_rpm * (2 * np.pi / 60.0)
        
        # Speed ratio (0 to 1)
        speed_ratio = abs(velocity) / max_speed_rad_s
        speed_ratio = min(speed_ratio, 1.0)  # Cap at 1.0
        
        # Torque reduction factor
        # At 0 speed: factor = 1.0 (full torque)
        # At max speed: factor = 0.0 (no torque)
        torque_factor = 1.0 - speed_ratio
        
        # Effective torque loss
        stall_torque = specs['stall_torque']
        torque_loss = stall_torque * speed_ratio
        
        return torque_loss, torque_factor
    
    def calculate_total_dynamic_torque(self, joint_name, pos_from, pos_to, movement_time, 
                                      payload_mass=0.0, servo_type=None):
        """Calculate total torque requirement including all dynamic effects.
        
        Total torque = Static (gravity) 
                     + Acceleration (τ = I × α)
                     + Dynamic friction
                     - Back-EMF losses (reduces available torque)
        
        This gives a much more realistic prediction than static analysis alone.
        """
        if servo_type is None:
            servo_type = self.current_servos[joint_name]
        
        # 1. Static torque (gravity) at destination position
        static_torque = self.calculate_static_torque(joint_name, pos_to, payload_mass)
        
        # 2. Acceleration torque
        angular_distance = self.calculate_angular_distance(joint_name, pos_from, pos_to)
        accel_torque = self.calculate_acceleration_torque(joint_name, angular_distance, movement_time)
        
        # 3. Average velocity during movement
        avg_velocity = abs(angular_distance) / movement_time if movement_time > 0 else 0
        
        # 4. Dynamic friction
        friction_torque = self.calculate_dynamic_friction(joint_name, avg_velocity)
        
        # 5. Back-EMF losses
        back_emf_loss, torque_factor = self.calculate_back_emf_losses(joint_name, avg_velocity, servo_type)
        
        # Total required torque (before back-EMF)
        required_torque = static_torque + accel_torque + friction_torque
        
        # Available torque after back-EMF
        available_torque = self.servo_specs[servo_type]['stall_torque'] * torque_factor
        
        return {
            'static': static_torque,
            'acceleration': accel_torque,
            'friction': friction_torque,
            'back_emf_loss': back_emf_loss,
            'total_required': required_torque,
            'available_torque': available_torque,
            'utilization': (required_torque / available_torque * 100) if available_torque > 0 else float('inf'),
            'velocity': avg_velocity,
            'angular_distance': angular_distance
        }
    
    def calculate_forward_kinematics(self, pose):
        """Calculate actual 3D positions of link COMs using forward kinematics.
        
        This considers the full arm configuration, not just individual joint angles.
        Critical for accurate torque calculations!
        
        Uses actual link lengths and COM positions from URDF (koch_v11_arm_real.urdf).
        
        Args:
            pose: dict of joint angles {joint_name: angle_rad}
        
        Returns:
            dict of {link_name: {'x': x, 'z': z, 'dist': distance_from_base}}
        """
        # Joint angles
        pan = pose.get('shoulder_pan', 0.0)
        lift = pose.get('shoulder_lift', 0.0)
        elbow = pose.get('elbow_flex', 0.0)
        wrist_flex = pose.get('wrist_flex', 0.0)
        wrist_roll = pose.get('wrist_roll', 0.0)
        
        # Link lengths from URDF joint origins
        L_shoulder_to_lift = 0.038    # shoulder_lift joint offset
        L_upper_arm = 0.110           # upper_arm link length (to elbow)
        L_forearm = 0.101             # forearm link length (to wrist)
        L_wrist = 0.043               # wrist length (to hand)
        L_hand = 0.017                # hand length (to pen)
        
        # COM offsets along each link (from URDF inertial origins)
        # These are distances from link origin to COM along the link
        upper_arm_com_offset = 0.141  # COM is 141mm from upper_arm origin
        forearm_com_offset = 0.101    # COM is 101mm from forearm origin
        wrist_com_offset = 0.077      # COM is 77mm from wrist origin
        hand_com_offset = 0.036       # COM is 36mm from hand origin
        pen_com_offset = 0.310        # COM is 310mm from pen origin (long holder!)
        
        positions = {}
        
        # --- UPPER ARM LINK ---
        # Upper arm COM position (simplified - using lift angle only for 2D analysis)
        # In reality, upper_arm COM is offset in x,y,z but for torque we care about z-height
        upper_x = upper_arm_com_offset * np.cos(lift)
        upper_z = upper_arm_com_offset * np.sin(lift)
        positions['upper_arm_link'] = {
            'x': upper_x,
            'z': upper_z,
            'dist': upper_arm_com_offset
        }
        
        # Position of elbow joint
        elbow_x = L_upper_arm * np.cos(lift)
        elbow_z = L_upper_arm * np.sin(lift)
        
        # --- FOREARM LINK ---
        # Forearm angle is lift + elbow
        forearm_angle = lift + elbow
        forearm_com_x = elbow_x + forearm_com_offset * np.cos(forearm_angle)
        forearm_com_z = elbow_z + forearm_com_offset * np.sin(forearm_angle)
        forearm_dist = np.sqrt(forearm_com_x**2 + forearm_com_z**2)
        positions['forearm_link'] = {
            'x': forearm_com_x,
            'z': forearm_com_z,
            'dist': forearm_dist
        }
        
        # Position of wrist joint
        wrist_x = elbow_x + L_forearm * np.cos(forearm_angle)
        wrist_z = elbow_z + L_forearm * np.sin(forearm_angle)
        
        # --- WRIST LINK ---
        # Wrist angle is lift + elbow + wrist_flex
        wrist_angle = lift + elbow + wrist_flex
        wrist_com_x = wrist_x + wrist_com_offset * np.cos(wrist_angle)
        wrist_com_z = wrist_z + wrist_com_offset * np.sin(wrist_angle)
        wrist_dist = np.sqrt(wrist_com_x**2 + wrist_com_z**2)
        positions['wrist_link'] = {
            'x': wrist_com_x,
            'z': wrist_com_z,
            'dist': wrist_dist
        }
        
        # Position of hand start
        hand_start_x = wrist_x + L_wrist * np.cos(wrist_angle)
        hand_start_z = wrist_z + L_wrist * np.sin(wrist_angle)
        
        # --- HAND LINK ---
        hand_com_x = hand_start_x + hand_com_offset * np.cos(wrist_angle)
        hand_com_z = hand_start_z + hand_com_offset * np.sin(wrist_angle)
        hand_dist = np.sqrt(hand_com_x**2 + hand_com_z**2)
        positions['hand_link'] = {
            'x': hand_com_x,
            'z': hand_com_z,
            'dist': hand_dist
        }
        
        # --- PEN LINK ---
        # Pen starts at end of hand
        pen_start_x = hand_start_x + L_hand * np.cos(wrist_angle)
        pen_start_z = hand_start_z + L_hand * np.sin(wrist_angle)
        
        # Pen COM (note: pen has unusual COM location due to holder geometry)
        pen_com_x = pen_start_x + pen_com_offset * np.cos(wrist_angle)
        pen_com_z = pen_start_z + pen_com_offset * np.sin(wrist_angle)
        pen_dist = np.sqrt(pen_com_x**2 + pen_com_z**2)
        positions['pen_link'] = {
            'x': pen_com_x,
            'z': pen_com_z,
            'dist': pen_dist
        }
        
        return positions
    
    def calculate_torque_with_fk(self, joint_name, pose, payload_mass=0.0):
        """Calculate torque using forward kinematics for accurate moment arms.
        
        This is the CORRECT way to calculate torque - considers full arm configuration!
        
        Args:
            joint_name: Joint to calculate torque for
            pose: Full arm configuration dict
            payload_mass: Additional payload (kg)
        
        Returns:
            Static torque (Nm) with correct moment arms
        """
        g = 9.81
        
        # Get actual COM positions using FK
        com_positions = self.calculate_forward_kinematics(pose)
        
        # Get joint angle
        joint_angle = pose.get(joint_name, 0.0)
        
        if joint_name == 'shoulder_pan':
            # Vertical axis - no gravity torque
            return 0.0
        
        elif joint_name == 'shoulder_lift':
            # All links downstream of shoulder
            links = ['upper_arm_link', 'forearm_link', 'wrist_link', 'hand_link', 'pen_link']
            
            total_torque = 0.0
            for link in links:
                mass = self.link_masses.get(link, 0.0)
                # Use actual distance from FK (accounts for elbow/wrist bending!)
                distance = com_positions[link]['dist']
                z_height = com_positions[link]['z']
                
                # Torque = m * g * horizontal_distance
                # horizontal_distance = distance * sin(angle_from_vertical)
                # But we can use z_height directly: torque = m * g * z_height
                torque = mass * g * abs(z_height)
                total_torque += torque
            
            # Payload at pen tip
            if payload_mass > 0:
                pen_z = com_positions['pen_link']['z']
                total_torque += payload_mass * g * abs(pen_z)
            
            return total_torque
        
        elif joint_name == 'elbow_flex':
            # Links downstream of elbow
            links = ['forearm_link', 'wrist_link', 'hand_link', 'pen_link']
            
            # Need to calculate torque relative to elbow joint
            lift = pose.get('shoulder_lift', 0.0)
            L_upper_arm = 0.093
            elbow_x = L_upper_arm * np.cos(lift)
            elbow_z = L_upper_arm * np.sin(lift)
            
            total_torque = 0.0
            for link in links:
                mass = self.link_masses.get(link, 0.0)
                # Distance from elbow joint
                link_x = com_positions[link]['x']
                link_z = com_positions[link]['z']
                dx = link_x - elbow_x
                dz = link_z - elbow_z
                distance_from_elbow = np.sqrt(dx**2 + dz**2)
                
                # Torque about elbow
                torque = mass * g * abs(dz)
                total_torque += torque
            
            # Payload
            if payload_mass > 0:
                pen_z = com_positions['pen_link']['z']
                dz = pen_z - elbow_z
                total_torque += payload_mass * g * abs(dz)
            
            return total_torque
        
        else:
            # Wrist joints - use simplified model (small effect)
            return self.calculate_static_torque(joint_name, joint_angle, payload_mass)
    
    def analyze_pose_transition(self, pose_from, pose_to, movement_time=2.0, csv_output=None, yaml_file=None, transition_name=None):
        """Analyze complete arm transition between two poses.
        
        This is the full analysis using forward kinematics!
        
        Args:
            pose_from: dict of {joint_name: angle_rad}
            pose_to: dict of {joint_name: angle_rad}
            movement_time: seconds for transition
            csv_output: Optional path to CSV file for appending results
            yaml_file: Optional source YAML filename for CSV metadata
            transition_name: Optional name for this transition (e.g., "pose1 → pose2")
        
        Returns:
            dict of {joint_name: dynamics_analysis}
        """
        results = {}
        
        self.get_logger().info(f"\n{'='*60}")
        self.get_logger().info("FULL POSE TRANSITION ANALYSIS")
        self.get_logger().info(f"{'='*60}")
        self.get_logger().info(f"Movement time: {movement_time:.1f}s")
        
        # Show pose configurations
        self.get_logger().info(f"\nFrom pose:")
        for joint in self.joint_names:
            angle = pose_from.get(joint, 0.0)
            self.get_logger().info(f"  {joint:15s}: {angle:6.3f} rad ({np.degrees(angle):6.1f}°)")
        
        self.get_logger().info(f"\nTo pose:")
        for joint in self.joint_names:
            angle = pose_to.get(joint, 0.0)
            self.get_logger().info(f"  {joint:15s}: {angle:6.3f} rad ({np.degrees(angle):6.1f}°)")
        
        # Analyze each joint
        for joint_name in self.joint_names:
            servo_type = self.current_servos[joint_name]
            
            # Get joint angles
            angle_from = pose_from.get(joint_name, 0.0)
            angle_to = pose_to.get(joint_name, 0.0)
            
            # Calculate torques with FK (correct!)
            static_torque_from = self.calculate_torque_with_fk(joint_name, pose_from)
            static_torque_to = self.calculate_torque_with_fk(joint_name, pose_to)
            
            # Calculate torque with simplified model (for comparison)
            simple_torque_to = self.calculate_static_torque(joint_name, angle_to)
            
            # Calculate dynamics
            angular_distance = self.calculate_angular_distance(joint_name, angle_from, angle_to)
            accel_torque = self.calculate_acceleration_torque(joint_name, angular_distance, movement_time)
            
            avg_velocity = abs(angular_distance) / movement_time if movement_time > 0 else 0
            friction_torque = self.calculate_dynamic_friction(joint_name, avg_velocity)
            back_emf_loss, torque_factor = self.calculate_back_emf_losses(joint_name, avg_velocity, servo_type)
            
            # Total required
            total_required = static_torque_to + accel_torque + friction_torque
            
            # Available after back-EMF
            stall_torque = self.servo_specs[servo_type]['stall_torque']
            available_torque = stall_torque * torque_factor
            
            # Utilization
            utilization = (total_required / available_torque * 100) if available_torque > 0 else float('inf')
            
            # FK vs simplified error
            fk_error = abs(static_torque_to - simple_torque_to)
            fk_error_pct = (fk_error / static_torque_to * 100) if static_torque_to > 0 else 0
            
            results[joint_name] = {
                'angle_from': angle_from,
                'angle_to': angle_to,
                'angular_distance': angular_distance,
                'static_fk': static_torque_to,
                'static_simple': simple_torque_to,
                'fk_error': fk_error,
                'fk_error_pct': fk_error_pct,
                'acceleration': accel_torque,
                'friction': friction_torque,
                'back_emf_loss': back_emf_loss,
                'total_required': total_required,
                'available_torque': available_torque,
                'utilization': utilization,
                'velocity': avg_velocity,
                'servo_type': servo_type
            }
        
        # Display results
        self._display_pose_transition_results(results)
        
        # Write to CSV if requested
        if csv_output:
            # Create transition name if not provided
            if not transition_name:
                from_angles = [f"{pose_from.get(j, 0.0):.2f}" for j in self.joint_names[:3]]
                to_angles = [f"{pose_to.get(j, 0.0):.2f}" for j in self.joint_names[:3]]
                transition_name = f"[{','.join(from_angles)}] → [{','.join(to_angles)}]"
            
            # Use provided yaml_file or default
            source_file = yaml_file if yaml_file else "manual_transition"
            
            self.write_csv_results(csv_output, source_file, movement_time, transition_name, results)
        
        return results
    
    def _display_pose_transition_results(self, results):
        """Display pose transition analysis results."""
        
        self.get_logger().info(f"\n{'='*60}")
        self.get_logger().info("JOINT-BY-JOINT ANALYSIS")
        self.get_logger().info(f"{'='*60}")
        
        for joint_name, data in results.items():
            if abs(data['angular_distance']) < 0.001:
                continue  # Skip joints that don't move
            
            self.get_logger().info(f"\n{joint_name.upper()}")
            self.get_logger().info(f"  Servo: {data['servo_type']}")
            self.get_logger().info(f"  Movement: {np.degrees(abs(data['angular_distance'])):.1f}° in {data['velocity']:.2f} rad/s avg")
            
            self.get_logger().info(f"\n  Torque breakdown:")
            self.get_logger().info(f"    Static (FK):      {data['static_fk']:.3f} Nm")
            self.get_logger().info(f"    Static (simple):  {data['static_simple']:.3f} Nm")
            self.get_logger().info(f"    FK error:         {data['fk_error']:.3f} Nm ({data['fk_error_pct']:+.1f}%)")
            self.get_logger().info(f"    Acceleration:     {data['acceleration']:.3f} Nm")
            self.get_logger().info(f"    Friction:         {data['friction']:.3f} Nm")
            self.get_logger().info(f"    ───────────────────────")
            self.get_logger().info(f"    Total required:   {data['total_required']:.3f} Nm")
            
            self.get_logger().info(f"\n  Servo capacity:")
            self.get_logger().info(f"    Back-EMF loss:    {data['back_emf_loss']:.3f} Nm")
            self.get_logger().info(f"    Available:        {data['available_torque']:.3f} Nm")
            self.get_logger().info(f"    Utilization:      {data['utilization']:.1f}%")
            
            if data['utilization'] > 100:
                self.get_logger().error(f"    ❌ OVERLOADED")
            elif data['utilization'] > 80:
                self.get_logger().warn(f"    ⚠️  HIGH LOAD")
            elif data['utilization'] > 50:
                self.get_logger().warn(f"    ⚠️  MODERATE")
            else:
                self.get_logger().info(f"    ✓ SAFE")
        
        # Summary
        self.get_logger().info(f"\n{'='*60}")
        self.get_logger().info("SUMMARY")
        self.get_logger().info(f"{'='*60}")
        
        worst_joint = max(results.items(), key=lambda x: x[1]['utilization'])
        worst_name, worst_data = worst_joint
        
        self.get_logger().info(f"Worst case: {worst_name} at {worst_data['utilization']:.1f}%")
        
        if worst_data['utilization'] > 100:
            self.get_logger().error("❌ UNSAFE - Servo may stall or be damaged")
        elif worst_data['utilization'] > 80:
            self.get_logger().warn("⚠️  RISKY - May stall, overheat, or wear quickly")
        elif worst_data['utilization'] > 50:
            self.get_logger().warn("⚠️  MODERATE - Monitor temperature")
        else:
            self.get_logger().info("✓ SAFE - All joints within limits")
        
        # FK error summary
        max_fk_error = max(results.items(), key=lambda x: abs(x[1]['fk_error_pct']))
        if abs(max_fk_error[1]['fk_error_pct']) > 20:
            self.get_logger().warn(f"\n⚠️  FK correction significant for {max_fk_error[0]}: {max_fk_error[1]['fk_error_pct']:+.1f}%")
            self.get_logger().warn("   Simplified single-joint model would be inaccurate!")
    
    def analyze_pose_sequence_from_yaml(self, yaml_file, movement_time=2.0, csv_output=None):
        """Analyze complete pose sequence from YAML file.
        
        Compatible with pose_test.py YAML format.
        
        Args:
            yaml_file: Path to YAML file
            movement_time: Default movement time between poses
            csv_output: Optional path to CSV file for appending results
        """
        self.get_logger().info(f"\n{'='*60}")
        self.get_logger().info(f"YAML POSE SEQUENCE ANALYSIS")
        self.get_logger().info(f"{'='*60}")
        self.get_logger().info(f"File: {yaml_file}")
        
        # Extract filename for CSV metadata
        yaml_basename = os.path.basename(yaml_file)
        
        # Load YAML
        try:
            with open(yaml_file, 'r') as f:
                data = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f"Failed to load YAML: {e}")
            return
        
        poses = data.get('poses', [])
        
        if not poses:
            self.get_logger().error("No poses found in YAML file!")
            return
        
        self.get_logger().info(f"Found {len(poses)} poses")
        
        if csv_output:
            self.get_logger().info(f"CSV output: {csv_output}")
        
        # Analyze each transition
        all_results = []
        
        for i in range(len(poses) - 1):
            pose_from_data = poses[i]
            pose_to_data = poses[i + 1]
            
            # Extract pose dictionaries
            pose_from = pose_from_data.get('positions', {})
            pose_to = pose_to_data.get('positions', {})
            
            # Fill in missing joints with zeros
            for joint in self.joint_names:
                if joint not in pose_from:
                    pose_from[joint] = 0.0
                if joint not in pose_to:
                    pose_to[joint] = 0.0
            
            self.get_logger().info(f"\n{'='*60}")
            self.get_logger().info(f"TRANSITION {i+1}/{len(poses)-1}: {pose_from_data.get('name', f'pose_{i}')} → {pose_to_data.get('name', f'pose_{i+1}')}")
            self.get_logger().info(f"{'='*60}")
            
            # Create transition name for CSV
            transition_name = f"{pose_from_data.get('name', f'pose_{i}')} → {pose_to_data.get('name', f'pose_{i+1}')}"
            
            # Analyze with CSV output
            results = self.analyze_pose_transition(
                pose_from, pose_to, movement_time, 
                csv_output=csv_output, 
                yaml_file=yaml_basename,
                transition_name=transition_name
            )
            
            all_results.append({
                'from': pose_from_data.get('name', f'pose_{i}'),
                'to': pose_to_data.get('name', f'pose_{i+1}'),
                'results': results
            })
        
        # Overall summary
        self.get_logger().info(f"\n{'='*80}")
        self.get_logger().info("SEQUENCE SUMMARY")
        self.get_logger().info(f"{'='*80}")
        
        for i, transition in enumerate(all_results, 1):
            worst = max(transition['results'].items(), key=lambda x: x[1]['utilization'])
            worst_name, worst_data = worst
            
            status = "✓" if worst_data['utilization'] < 50 else "⚠️" if worst_data['utilization'] < 80 else "❌"
            
            self.get_logger().info(
                f"{i:2d}. {transition['from']:15s} → {transition['to']:15s}  "
                f"{status} {worst_name:15s} {worst_data['utilization']:5.1f}%"
            )
    
    def write_csv_results(self, csv_file, yaml_file, movement_time, transition_name, results):
        """Append analysis results to CSV file for Excel graphing.
        
        Args:
            csv_file: Path to CSV output file
            yaml_file: Source YAML file name (or "manual_transition")
            movement_time: Movement time in seconds
            transition_name: "pose_from → pose_to"
            results: Dictionary of joint results from analyze_pose_transition
        """
        # Check if file exists to determine if we need headers
        file_exists = os.path.isfile(csv_file)
        
        # CSV columns
        fieldnames = [
            'timestamp',
            'yaml_file',
            'movement_time_sec',
            'transition',
            'joint_name',
            'servo_type',
            'angle_from_rad',
            'angle_to_rad',
            'angular_distance_rad',
            'angular_distance_deg',
            'static_torque_fk_nm',
            'static_torque_simple_nm',
            'fk_error_pct',
            'acceleration_torque_nm',
            'friction_torque_nm',
            'total_required_nm',
            'back_emf_loss_nm',
            'available_torque_nm',
            'utilization_pct',
            'avg_velocity_rad_s'
        ]
        
        try:
            with open(csv_file, 'a', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=fieldnames)
                
                # Write header if new file
                if not file_exists:
                    writer.writeheader()
                
                # Write one row per joint
                timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                
                for joint_name, data in results.items():
                    # Skip joints that didn't move
                    if abs(data['angular_distance']) < 0.001:
                        continue
                    
                    row = {
                        'timestamp': timestamp,
                        'yaml_file': yaml_file,
                        'movement_time_sec': movement_time,
                        'transition': transition_name,
                        'joint_name': joint_name,
                        'servo_type': data['servo_type'],
                        'angle_from_rad': f"{data['angle_from']:.4f}",
                        'angle_to_rad': f"{data['angle_to']:.4f}",
                        'angular_distance_rad': f"{data['angular_distance']:.4f}",
                        'angular_distance_deg': f"{np.degrees(abs(data['angular_distance'])):.2f}",
                        'static_torque_fk_nm': f"{data['static_fk']:.6f}",
                        'static_torque_simple_nm': f"{data['static_simple']:.6f}",
                        'fk_error_pct': f"{data['fk_error_pct']:.2f}",
                        'acceleration_torque_nm': f"{data['acceleration']:.6f}",
                        'friction_torque_nm': f"{data['friction']:.6f}",
                        'total_required_nm': f"{data['total_required']:.6f}",
                        'back_emf_loss_nm': f"{data['back_emf_loss']:.6f}",
                        'available_torque_nm': f"{data['available_torque']:.6f}",
                        'utilization_pct': f"{data['utilization']:.2f}",
                        'avg_velocity_rad_s': f"{data['velocity']:.4f}"
                    }
                    
                    writer.writerow(row)
            
            self.get_logger().info(f"✓ Results appended to {csv_file}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to write CSV: {e}")
    
    def read_servo_telemetry(self, joint_name):
        """Read temperature, current, voltage from servo (if available)."""
        if not self.dxl_port:
            return None
        
        servo_id = self.servo_ids.get(joint_name)
        if not servo_id:
            return None
        
        try:
            # Read temperature
            temp, _, _ = self.dxl_packet.read1ByteTxRx(self.dxl_port, servo_id, 146)
            
            # Read current
            current_raw, _, _ = self.dxl_packet.read2ByteTxRx(self.dxl_port, servo_id, 126)
            current_mA = current_raw * 2.69
            
            # Read voltage
            voltage_raw, _, _ = self.dxl_packet.read2ByteTxRx(self.dxl_port, servo_id, 144)
            voltage = voltage_raw * 0.1
            
            return {
                'temperature': temp,
                'current_mA': current_mA,
                'voltage': voltage
            }
        except:
            return None
    
    def test_static_hold(self, joint_name, previous_position=None, movement_time=2.0):
        """Test ability to hold position under gravity with optional trajectory analysis."""
        self.get_logger().info(f"\n{'='*60}")
        self.get_logger().info(f"STATIC HOLD TEST: {joint_name}")
        self.get_logger().info(f"{'='*60}")
        
        if not self.wait_for_joint_states():
            self.get_logger().error("No joint states available!")
            return
        
        current_pos = self.current_positions.get(joint_name, 0.0)
        servo_type = self.current_servos[joint_name]
        servo_specs = self.servo_specs[servo_type]
        
        # Calculate predicted torque
        predicted_torque = self.calculate_static_torque(joint_name, current_pos)
        servo_max = servo_specs['stall_torque']
        
        self.get_logger().info(f"\nServo: {servo_type}")
        self.get_logger().info(f"Position: {current_pos:.3f} rad ({np.degrees(current_pos):.1f}°)")
        
        self.get_logger().info(f"\n📊 STATIC ANALYSIS")
        self.get_logger().info(f"Predicted torque: {predicted_torque:.3f} Nm")
        self.get_logger().info(f"Servo max: {servo_max:.3f} Nm")
        
        utilization = (predicted_torque / servo_max) * 100
        self.get_logger().info(f"Utilization: {utilization:.1f}%")
        
        if utilization < 50:
            self.get_logger().info("✓ Safe - Low load")
        elif utilization < 80:
            self.get_logger().warn("⚠️  Moderate load")
        else:
            self.get_logger().error("❌ High load - risk of damage!")
        
        # Trajectory analysis if previous position provided
        if previous_position is not None:
            self.get_logger().info(f"\n📊 TRAJECTORY ANALYSIS")
            self.get_logger().info(f"From: {previous_position:.3f} rad ({np.degrees(previous_position):.1f}°)")
            self.get_logger().info(f"To:   {current_pos:.3f} rad ({np.degrees(current_pos):.1f}°)")
            self.get_logger().info(f"Movement time: {movement_time:.1f}s")
            
            dynamics = self.calculate_total_dynamic_torque(
                joint_name, previous_position, current_pos, movement_time, servo_type=servo_type
            )
            
            self.get_logger().info(f"\nTorque Breakdown:")
            self.get_logger().info(f"  Static (gravity):   {dynamics['static']:.3f} Nm")
            self.get_logger().info(f"  Acceleration:       {dynamics['acceleration']:.3f} Nm")
            self.get_logger().info(f"  Dynamic friction:   {dynamics['friction']:.3f} Nm")
            self.get_logger().info(f"  ─────────────────────────────")
            self.get_logger().info(f"  Total required:     {dynamics['total_required']:.3f} Nm")
            
            self.get_logger().info(f"\nMotion Profile:")
            self.get_logger().info(f"  Angular distance:   {abs(dynamics['angular_distance']):.3f} rad ({np.degrees(abs(dynamics['angular_distance'])):.1f}°)")
            self.get_logger().info(f"  Average velocity:   {dynamics['velocity']:.3f} rad/s")
            
            self.get_logger().info(f"\nBack-EMF Analysis:")
            self.get_logger().info(f"  Torque loss:        {dynamics['back_emf_loss']:.3f} Nm")
            self.get_logger().info(f"  Available torque:   {dynamics['available_torque']:.3f} Nm")
            self.get_logger().info(f"  Dynamic util:       {dynamics['utilization']:.1f}%")
            
            # Error analysis
            error = dynamics['total_required'] - predicted_torque
            error_pct = (error / predicted_torque * 100) if predicted_torque > 0 else 0
            
            self.get_logger().info(f"\n⚠️  SIMPLIFICATION ERROR:")
            self.get_logger().info(f"  Static model:       {predicted_torque:.3f} Nm")
            self.get_logger().info(f"  Dynamic model:      {dynamics['total_required']:.3f} Nm")
            self.get_logger().info(f"  Error:              {error:.3f} Nm ({error_pct:+.1f}%)")
            
            if abs(error_pct) > 50:
                self.get_logger().error(f"  ❌ LARGE ERROR - Dynamic effects dominate!")
            elif abs(error_pct) > 20:
                self.get_logger().warn(f"  ⚠️  Significant error - Consider dynamic analysis")
            else:
                self.get_logger().info(f"  ✓ Acceptable error - Static model OK")
            
            # Safety verdict
            if dynamics['utilization'] > 100:
                self.get_logger().error(f"\n❌ INSUFFICIENT TORQUE FOR MOVEMENT")
                self.get_logger().error(f"   May stall or require slower movement")
            elif dynamics['utilization'] > 80:
                self.get_logger().warn(f"\n⚠️  HIGH UTILIZATION - May stall or overheat")
            elif dynamics['utilization'] > 50:
                self.get_logger().warn(f"\n⚠️  MODERATE - Monitor temperature")
            else:
                self.get_logger().info(f"\n✓ SAFE FOR MOVEMENT")
        
        # Test different positions
        self.get_logger().info(f"\n📊 Static torque at different positions:")
        for angle_deg in [0, 30, 45, 60, 90]:
            angle_rad = np.radians(angle_deg)
            torque = self.calculate_static_torque(joint_name, angle_rad)
            util = (torque / servo_max) * 100
            self.get_logger().info(f"  {angle_deg:3d}°: {torque:.3f} Nm ({util:5.1f}%)")
        
        return predicted_torque
    
    def test_payload_capacity(self, joint_name):
        """Test maximum payload capacity."""
        self.get_logger().info(f"\n{'='*60}")
        self.get_logger().info(f"PAYLOAD CAPACITY TEST: {joint_name}")
        self.get_logger().info(f"{'='*60}")
        
        if not self.wait_for_joint_states():
            self.get_logger().error("No joint states available!")
            return
        
        current_pos = self.current_positions.get(joint_name, 0.0)
        servo_type = self.current_servos[joint_name]
        servo_max = self.servo_specs[servo_type]['stall_torque']
        
        # Calculate torque from arm
        arm_torque = self.calculate_static_torque(joint_name, current_pos, 0.0)
        available_torque = servo_max - arm_torque
        
        # Calculate max payload
        if joint_name == 'shoulder_lift':
            payload_distance = self.link_com_distances['shoulder_lift']['pen_link']
            angle_from_vertical = current_pos - np.pi/2
            sin_angle = np.abs(np.sin(angle_from_vertical))
            if sin_angle < 0.1:
                sin_angle = 0.1
            max_payload = available_torque / (9.81 * payload_distance * sin_angle)
        else:
            max_payload = available_torque / (9.81 * 0.15)
        
        self.get_logger().info(f"\nPosition: {current_pos:.3f} rad ({np.degrees(current_pos):.1f}°)")
        self.get_logger().info(f"Servo max: {servo_max:.3f} Nm")
        self.get_logger().info(f"Arm torque: {arm_torque:.3f} Nm")
        self.get_logger().info(f"Available: {available_torque:.3f} Nm")
        self.get_logger().info(f"\n📊 MAX PAYLOAD: {max_payload*1000:.0f}g")
        
        # Test different payloads
        self.get_logger().info(f"\nPayload test:")
        for payload_g in [0, 50, 100, 150, 200, 250]:
            payload_kg = payload_g / 1000.0
            total_torque = self.calculate_static_torque(joint_name, current_pos, payload_kg)
            util = (total_torque / servo_max) * 100
            status = "✓" if util < 80 else "⚠️" if util < 100 else "❌"
            self.get_logger().info(f"  {payload_g:3d}g: {total_torque:.3f} Nm ({util:5.1f}%) {status}")
    
    def test_what_if(self, joint_name, new_servo_type, position=None):
        """What-if analysis with different servo."""
        self.get_logger().info(f"\n{'='*60}")
        self.get_logger().info(f"WHAT-IF ANALYSIS: {joint_name}")
        self.get_logger().info(f"{'='*60}")
        
        if new_servo_type not in self.servo_specs:
            self.get_logger().error(f"Unknown servo: {new_servo_type}")
            self.get_logger().info(f"Available: {', '.join(self.servo_specs.keys())}")
            return
        
        if not self.wait_for_joint_states():
            self.get_logger().error("No joint states available!")
            return
        
        current_pos = self.current_positions.get(joint_name, 0.0) if position is None else position
        
        # Current vs proposed
        current_servo = self.current_servos[joint_name]
        current_specs = self.servo_specs[current_servo]
        proposed_specs = self.servo_specs[new_servo_type]
        
        required = self.calculate_static_torque(joint_name, current_pos)
        
        self.get_logger().info(f"\nCurrent: {current_servo}")
        self.get_logger().info(f"  Torque: {current_specs['stall_torque']:.2f} Nm")
        self.get_logger().info(f"  Mass: {current_specs['mass']*1000:.0f}g")
        self.get_logger().info(f"  Cost: ${current_specs['cost']}")
        
        self.get_logger().info(f"\nProposed: {new_servo_type}")
        self.get_logger().info(f"  Torque: {proposed_specs['stall_torque']:.2f} Nm")
        self.get_logger().info(f"  Mass: {proposed_specs['mass']*1000:.0f}g")
        self.get_logger().info(f"  Cost: ${proposed_specs['cost']}")
        
        self.get_logger().info(f"\nAt {np.degrees(current_pos):.1f}°:")
        self.get_logger().info(f"  Required: {required:.3f} Nm")
        
        current_util = (required / current_specs['stall_torque']) * 100
        proposed_util = (required / proposed_specs['stall_torque']) * 100
        
        self.get_logger().info(f"  Current util: {current_util:.1f}%")
        self.get_logger().info(f"  Proposed util: {proposed_util:.1f}%")
        
        # Analysis
        self.get_logger().info(f"\n{'='*60}")
        self.get_logger().info("VERDICT")
        self.get_logger().info(f"{'='*60}")
        
        if proposed_util > 100:
            self.get_logger().error("❌ INSUFFICIENT TORQUE")
            self.get_logger().error(f"   Deficit: {required - proposed_specs['stall_torque']:.3f} Nm")
        elif proposed_util > 80:
            self.get_logger().warn("⚠️  HIGH UTILIZATION")
        elif proposed_util > 50:
            self.get_logger().info("⚠️  MODERATE - Monitor temp")
        else:
            self.get_logger().info("✓ SAFE")
        
        # Differences
        mass_diff = proposed_specs['mass'] - current_specs['mass']
        cost_diff = proposed_specs['cost'] - current_specs['cost']
        
        if abs(mass_diff) > 0.001:
            self.get_logger().info(f"\nMass: {mass_diff*1000:+.0f}g")
        self.get_logger().info(f"Cost: ${cost_diff:+.0f}")
        
        if current_specs['voltage'] != proposed_specs['voltage']:
            self.get_logger().warn(f"\n⚠️  Voltage mismatch!")
            self.get_logger().warn(f"   Current: {current_specs['voltage'][0]}-{current_specs['voltage'][1]}V")
            self.get_logger().warn(f"   Proposed: {proposed_specs['voltage'][0]}-{proposed_specs['voltage'][1]}V")
    
    def diagnostics(self, joint_name):
        """Full diagnostics including telemetry."""
        self.get_logger().info(f"\n{'='*60}")
        self.get_logger().info(f"DIAGNOSTICS: {joint_name}")
        self.get_logger().info(f"{'='*60}")
        
        if not self.wait_for_joint_states():
            self.get_logger().error("No joint states available!")
            return
        
        position = self.current_positions.get(joint_name, 0.0)
        servo_type = self.current_servos[joint_name]
        specs = self.servo_specs[servo_type]
        
        self.get_logger().info(f"\nServo: {servo_type}")
        self.get_logger().info(f"Max Torque: {specs['stall_torque']:.2f} Nm")
        self.get_logger().info(f"Max Speed: {specs['no_load_speed']} rpm")
        
        self.get_logger().info(f"\nCurrent State:")
        self.get_logger().info(f"  Position: {position:.3f} rad ({np.degrees(position):.1f}°)")
        
        # Predicted load
        predicted = self.calculate_static_torque(joint_name, position)
        self.get_logger().info(f"\nPredicted torque: {predicted:.3f} Nm")
        self.get_logger().info(f"Utilization: {(predicted/specs['stall_torque'])*100:.1f}%")
        
        # Telemetry if available
        telemetry = self.read_servo_telemetry(joint_name)
        if telemetry:
            self.get_logger().info(f"\n📊 TELEMETRY:")
            self.get_logger().info(f"  Temperature: {telemetry['temperature']}°C")
            self.get_logger().info(f"  Current: {telemetry['current_mA']:.0f}mA")
            self.get_logger().info(f"  Voltage: {telemetry['voltage']:.1f}V")
            
            # Compare predicted vs actual
            current_util = (telemetry['current_mA'] / (specs['stall_current'] * 1000)) * 100
            self.get_logger().info(f"  Current util: {current_util:.1f}%")
            
            torque_util = (predicted / specs['stall_torque']) * 100
            error = abs(current_util - torque_util)
            
            if error < 10:
                self.get_logger().info(f"  ✓ Predicted matches actual ({error:.1f}% error)")
            elif error < 25:
                self.get_logger().info(f"  ✓ Good match ({error:.1f}% error)")
            else:
                self.get_logger().warn(f"  ⚠️  Discrepancy ({error:.1f}% error)")
        else:
            self.get_logger().warn("\n⚠️  Telemetry not available")
            self.get_logger().info("    Install dynamixel_sdk: pip install dynamixel-sdk")
    
    def compare_servos(self):
        """Compare all servo options."""
        self.get_logger().info(f"\n{'='*60}")
        self.get_logger().info("SERVO COMPARISON")
        self.get_logger().info(f"{'='*60}")
        
        if not self.wait_for_joint_states():
            self.get_logger().error("No joint states available!")
            return
        
        for joint in ['shoulder_lift', 'elbow_flex']:
            pos = self.current_positions.get(joint, 0.0)
            req = self.calculate_static_torque(joint, pos)
            
            self.get_logger().info(f"\n{joint.upper()}")
            self.get_logger().info(f"Position: {np.degrees(pos):.1f}°, Required: {req:.3f} Nm\n")
            self.get_logger().info(f"{'Servo':<20} {'Torque':<10} {'Util%':<10} {'Status':<15} {'Cost'}")
            self.get_logger().info(f"{'-'*70}")
            
            for name, specs in sorted(self.servo_specs.items()):
                util = (req / specs['stall_torque']) * 100
                
                if util > 100:
                    status = "❌ Insufficient"
                elif util > 80:
                    status = "⚠️  High"
                elif util > 50:
                    status = "⚠️  Moderate"
                else:
                    status = "✓ Safe"
                
                marker = " ←" if name == self.current_servos[joint] else ""
                
                self.get_logger().info(
                    f"{name:<20} {specs['stall_torque']:<10.2f} {util:<10.1f} {status:<15} ${specs['cost']}{marker}"
                )
    
    def full_analysis(self):
        """Complete analysis."""
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("FULL ROBOT ANALYSIS")
        self.get_logger().info("="*60)
        
        for joint in ['shoulder_lift', 'elbow_flex']:
            self.test_static_hold(joint)
            time.sleep(0.5)
            self.test_payload_capacity(joint)
            time.sleep(0.5)
        
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("RECOMMENDATIONS")
        self.get_logger().info("="*60)
        self.get_logger().info("\nSafe payload: <100g")
        self.get_logger().info("Max payload: ~150-200g")
        self.get_logger().info("\nUse --what-if to test servo upgrades")
        self.get_logger().info("Use --diagnostics for live telemetry")
    
    def __del__(self):
        """Cleanup Dynamixel connection."""
        if self.dxl_port:
            self.dxl_port.closePort()


def main():
    parser = argparse.ArgumentParser(
        description='Unified Load Testing Tool',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Basic tests
  ros2 run writing_robot_control load_tester --full_analysis
  ros2 run writing_robot_control load_tester --joint shoulder_lift --static
  ros2 run writing_robot_control load_tester --joint shoulder_lift --payload
  
  # Trajectory analysis (single joint)
  ros2 run writing_robot_control load_tester --joint shoulder_lift --static \\
    --previous-position 0.0 --movement-time 2.0
  
  # Full pose transition analysis
  ros2 run writing_robot_control load_tester --analyze-transition \\
    --pose-from "0.0,1.57,0.0,0.0,0.0,0.0" \\
    --pose-to "0.0,1.57,1.57,0.0,0.0,0.0" \\
    --movement-time 2.0
  
  # YAML sequence analysis
  ros2 run writing_robot_control load_tester --analyze-sequence poses.yaml \\
    --movement-time 2.0
  
  # CSV output for Excel graphing (NEW!)
  ros2 run writing_robot_control load_tester --analyze-sequence poses.yaml \\
    --movement-time 2.0 --csv-output results.csv
  
  # Compare different movement times (appends to same CSV)
  ros2 run writing_robot_control load_tester --analyze-sequence poses.yaml \\
    --movement-time 2.0 --csv-output comparison.csv
  ros2 run writing_robot_control load_tester --analyze-sequence poses.yaml \\
    --movement-time 1.0 --csv-output comparison.csv
  ros2 run writing_robot_control load_tester --analyze-sequence poses.yaml \\
    --movement-time 0.5 --csv-output comparison.csv
  
  # What-if scenarios
  ros2 run writing_robot_control load_tester --joint shoulder_lift --what-if XL330-M077
  ros2 run writing_robot_control load_tester --joint shoulder_lift --what-if XL330-M288
  ros2 run writing_robot_control load_tester --joint shoulder_lift --what-if XM430-W350
  
  # Diagnostics with telemetry
  ros2 run writing_robot_control load_tester --joint shoulder_lift --diagnostics
  
  # Compare all options
  ros2 run writing_robot_control load_tester --compare-servos
        """
    )
    
    parser.add_argument('--joint', type=str, help='Joint to test')
    parser.add_argument('--static', action='store_true', help='Static hold test')
    parser.add_argument('--payload', action='store_true', help='Payload capacity test')
    parser.add_argument('--diagnostics', action='store_true', help='Full diagnostics with telemetry')
    parser.add_argument('--what-if', type=str, metavar='SERVO',
                       help='Test with different servo (XL330-M077, XL330-M288, XM430-W350, etc)')
    parser.add_argument('--compare-servos', action='store_true',
                       help='Compare all servo options')
    parser.add_argument('--full_analysis', action='store_true',
                       help='Complete analysis')
    parser.add_argument('--position', type=float,
                       help='Test at specific position (radians)')
    parser.add_argument('--previous-position', type=float, metavar='RAD',
                       help='Previous position for trajectory analysis (radians)')
    parser.add_argument('--movement-time', type=float, default=2.0, metavar='SEC',
                       help='Movement time for trajectory analysis (seconds, default: 2.0)')
    
    # NEW: Full pose analysis
    parser.add_argument('--analyze-transition', action='store_true',
                       help='Analyze full pose transition (use with --pose-from and --pose-to)')
    parser.add_argument('--pose-from', type=str, metavar='ANGLES',
                       help='Starting pose as comma-separated angles (e.g., "0.0,1.57,0.0,0.0,0.0,0.0")')
    parser.add_argument('--pose-to', type=str, metavar='ANGLES',
                       help='Ending pose as comma-separated angles')
    parser.add_argument('--analyze-sequence', type=str, metavar='YAML_FILE',
                       help='Analyze complete pose sequence from YAML file')
    parser.add_argument('--csv-output', type=str, metavar='CSV_FILE',
                       help='Append analysis results to CSV file for Excel graphing')
    
    args, unknown = parser.parse_known_args()
    
    rclpy.init()
    tester = LoadTester()
    
    try:
        if args.full_analysis:
            tester.full_analysis()
        
        elif args.compare_servos:
            tester.compare_servos()
        
        elif args.analyze_sequence:
            # NEW: Analyze YAML pose sequence
            tester.analyze_pose_sequence_from_yaml(args.analyze_sequence, args.movement_time, args.csv_output)
        
        elif args.analyze_transition:
            # NEW: Analyze pose transition
            if not args.pose_from or not args.pose_to:
                print("Error: --analyze-transition requires --pose-from and --pose-to")
                parser.print_help()
            else:
                # Parse pose arrays
                try:
                    from_angles = [float(x.strip()) for x in args.pose_from.split(',')]
                    to_angles = [float(x.strip()) for x in args.pose_to.split(',')]
                    
                    if len(from_angles) != 6 or len(to_angles) != 6:
                        print("Error: Poses must have 6 angles (shoulder_pan,shoulder_lift,elbow_flex,wrist_flex,wrist_roll,pen_holder)")
                    else:
                        # Convert to dict
                        pose_from = dict(zip(tester.joint_names, from_angles))
                        pose_to = dict(zip(tester.joint_names, to_angles))
                        
                        tester.analyze_pose_transition(pose_from, pose_to, args.movement_time, args.csv_output)
                except ValueError as e:
                    print(f"Error parsing poses: {e}")
        
        elif args.joint:
            if args.what_if:
                tester.test_what_if(args.joint, args.what_if, args.position)
            elif args.diagnostics:
                tester.diagnostics(args.joint)
            elif args.static:
                tester.test_static_hold(args.joint, args.previous_position, args.movement_time)
            elif args.payload:
                tester.test_payload_capacity(args.joint)
            else:
                # Default to diagnostics
                tester.diagnostics(args.joint)
        
        else:
            parser.print_help()
    
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
