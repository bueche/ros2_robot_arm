#!/usr/bin/env python3
"""
ROS2 Pose Testing Node with Validation and Servo Testing

Features:
- ROS2 node publishing JointTrajectory messages
- Reads joint limits from URDF parameter
- Servo test mode (safe position + test min/max/middle per joint)
- Full pose sequence testing
- Telemetry from /joint_states topic (no direct servo access)
- Validation with URDF limits

Usage:
  ros2 run writing_robot_control pose_test --ros-args -p poses_file:=poses.yaml
  ros2 run writing_robot_control pose_test --ros-args -p servo_test:=true
  ros2 run writing_robot_control pose_test --ros-args -p servo_test:=true -p telemetry:=true
  ros2 run writing_robot_control pose_test --ros-args -p servo_test_joint:=shoulder_lift
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
import yaml
import time
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np


class URDFLimitsParser:
    """Parse joint limits from URDF."""
    
    def __init__(self, urdf_content):
        self.joint_limits = {}
        self.velocity_limits = {}
        self.parse_urdf(urdf_content)
    
    def parse_urdf(self, urdf_content):
        """Parse URDF XML string."""
        try:
            root = ET.fromstring(urdf_content)
            
            for joint in root.findall('joint'):
                joint_name = joint.get('name')
                joint_type = joint.get('type')
                
                if joint_type not in ['revolute', 'continuous']:
                    continue
                
                limit = joint.find('limit')
                if limit is not None:
                    lower = float(limit.get('lower', -3.14))
                    upper = float(limit.get('upper', 3.14))
                    velocity = float(limit.get('velocity', 2.0))
                    
                    # Store as 'min' and 'max' for actual range
                    # (lower/upper are directional, not necessarily min/max)
                    self.joint_limits[joint_name] = {
                        'min': min(lower, upper),
                        'max': max(lower, upper),
                        'lower': lower,  # Keep original for reference
                        'upper': upper
                    }
                    self.velocity_limits[joint_name] = velocity
            
        except Exception as e:
            print(f"Error parsing URDF: {e}")
    
    def get_joint_limits(self):
        return self.joint_limits
    
    def get_velocity_limits(self):
        return self.velocity_limits


class JointStateTelemetry:
    """Read telemetry from ROS2 joint_states topic (no direct servo access)."""
    
    def __init__(self, node, joint_names):
        self.node = node
        self.joint_names = joint_names
        self.latest_states = None
        self.enabled = False
        
        # Subscribe to joint_states
        self.subscription = self.node.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            10
        )
        
        self.node.get_logger().info("✓ Telemetry using /joint_states topic")
        
        # Wait a bit for first message (spin to allow callback)
        start_time = time.time()
        while (time.time() - start_time) < 0.5:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.latest_states is not None:
                break
            time.sleep(0.01)
        
        if self.latest_states is not None:
            self.enabled = True
            self.node.get_logger().info("✓ Receiving joint states")
        else:
            self.node.get_logger().warn("⚠️  No joint_states received yet")
            self.node.get_logger().warn("   Telemetry will be available once robot publishes states")
    
    def _joint_state_callback(self, msg):
        """Store latest joint states."""
        self.latest_states = msg
        if not self.enabled:
            self.enabled = True
    
    def get_joint_telemetry(self, joint_name):
        """Get telemetry for a specific joint from joint_states."""
        if not self.enabled or self.latest_states is None:
            return None
        
        try:
            # Find joint index
            if joint_name not in self.latest_states.name:
                return None
            
            idx = self.latest_states.name.index(joint_name)
            
            # Extract data
            result = {'joint': joint_name}
            
            # Position
            if idx < len(self.latest_states.position):
                result['position'] = self.latest_states.position[idx]
            
            # Velocity
            if idx < len(self.latest_states.velocity):
                result['velocity'] = self.latest_states.velocity[idx]
            
            # Effort (torque/current related)
            if idx < len(self.latest_states.effort):
                effort = self.latest_states.effort[idx]
                result['effort'] = effort
                
                # Rough current estimate (effort is usually in Nm, but for display purposes)
                # This is approximate - actual current would need servo-specific conversion
                result['current_estimate_mA'] = abs(effort * 1000)  # Rough scaling
            
            return result
            
        except Exception as e:
            self.node.get_logger().error(f"Error reading telemetry: {e}")
            return None
    
    def get_all_joints_telemetry(self):
        """Get telemetry for all joints."""
        if not self.enabled or self.latest_states is None:
            return {}
        
        results = {}
        for joint_name in self.joint_names:
            data = self.get_joint_telemetry(joint_name)
            if data:
                results[joint_name] = data
        
        return results
    
    def close(self):
        """Cleanup (nothing to do for topic-based telemetry)."""
        pass


class PoseTestNode(Node):
    """ROS2 node for pose testing with validation."""
    
    def __init__(self):
        super().__init__('pose_test_node')
        
        # Declare parameters
        self.declare_parameter('poses_file', '')
        self.declare_parameter('program', '')  # calibration, sweep, stress
        self.declare_parameter('servo_test', False)
        self.declare_parameter('servo_test_joint', '')  # Specific joint to test
        self.declare_parameter('safe_position_first', True)
        self.declare_parameter('validate', True)
        self.declare_parameter('telemetry', False)
        self.declare_parameter('delay', 2.0)
        self.declare_parameter('movement_time', 2.0)
        self.declare_parameter('urdf_file', '')  # NEW: Load URDF from file
        
        # Get parameters
        self.poses_file = self.get_parameter('poses_file').value
        self.program = self.get_parameter('program').value
        self.servo_test_mode = self.get_parameter('servo_test').value
        self.servo_test_joint = self.get_parameter('servo_test_joint').value
        self.safe_position_first = self.get_parameter('safe_position_first').value
        self.validate = self.get_parameter('validate').value
        self.enable_telemetry = self.get_parameter('telemetry').value
        self.delay = self.get_parameter('delay').value
        self.movement_time = self.get_parameter('movement_time').value
        self.urdf_file = self.get_parameter('urdf_file').value
        
        # Publisher
        self.publisher = self.create_publisher(
            JointTrajectory,
            '/koch_v11_controller/joint_trajectory',
            10
        )
        
        # Subscriber for joint states (optional feedback)
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        self.current_joint_states = None
        
        # Joint names (6-DOF Koch arm)
        self.joint_names = [
            'shoulder_pan',
            'shoulder_lift',
            'elbow_flex',
            'wrist_flex',
            'wrist_roll',
            'pen_holder'
        ]
        
        # Safe position for servo testing
        self.safe_position = {
            'shoulder_pan': 1.292,
            'shoulder_lift': 2.688,
            'elbow_flex': 1.063,
            'wrist_flex': 2.499,
            'wrist_roll': -0.014,
            'pen_holder': 0.895
        }
        
        # Load URDF limits
        self.joint_limits = {}
        self.velocity_limits = {}
        self.load_urdf_limits()
        
        # Telemetry (uses /joint_states topic, not direct servo access)
        self.telemetry = None
        if self.enable_telemetry:
            self.telemetry = JointStateTelemetry(self, self.joint_names)
        
        # Statistics
        self.stats = {
            'total_poses': 0,
            'validation_errors': 0,
            'validation_warnings': 0,
        }
        
        self.get_logger().info('Pose Test Node initialized')
    
    def load_urdf_limits(self):
        """Load joint limits from robot_description parameter or file."""
        urdf_content = None
        
        # Option 1: Load from urdf_file parameter (file path)
        if self.urdf_file:
            try:
                self.get_logger().info(f'Loading URDF from file: {self.urdf_file}')
                with open(self.urdf_file, 'r') as f:
                    urdf_content = f.read()
                self.get_logger().info('✓ URDF loaded from file')
            except FileNotFoundError:
                self.get_logger().error(f'URDF file not found: {self.urdf_file}')
            except Exception as e:
                self.get_logger().error(f'Error reading URDF file: {e}')
        
        # Option 2: Get from robot_description parameter (parameter server)
        if not urdf_content:
            try:
                urdf_content = self.get_parameter_or(
                    'robot_description',
                    rclpy.Parameter('robot_description', 
                                   rclpy.Parameter.Type.STRING, 
                                   '')
                ).value
                
                if urdf_content:
                    self.get_logger().info('✓ URDF loaded from robot_description parameter')
                else:
                    self.get_logger().warn('robot_description parameter is empty')
            except Exception as e:
                self.get_logger().warn(f'Could not read robot_description parameter: {e}')
        
        # Parse URDF if we got content
        if urdf_content:
            parser = URDFLimitsParser(urdf_content)
            self.joint_limits = parser.get_joint_limits()
            self.velocity_limits = parser.get_velocity_limits()
            
            if self.joint_limits:
                self.get_logger().info(f'✓ Loaded limits from URDF: {len(self.joint_limits)} joints')
                self.print_limits()
            else:
                self.get_logger().warn('No joint limits found in URDF')
                self.use_default_limits()
        else:
            self.get_logger().warn('No URDF content available')
            self.use_default_limits()
    
    def use_default_limits(self):
        """Fallback to default limits."""
        self.get_logger().warn('⚠️  Using default limits (not from URDF!)')
        
        # Default limits with proper min/max calculation
        defaults = {
            'shoulder_pan': {'lower': -3.14, 'upper': 3.14},
            'shoulder_lift': {'lower': -1.57, 'upper': 3.14},
            'elbow_flex': {'lower': -3.14, 'upper': 3.14},
            'wrist_flex': {'lower': -3.14, 'upper': 3.14},
            'wrist_roll': {'lower': -3.14, 'upper': 3.14},
            'pen_holder': {'lower': 0.0, 'upper': 3.0},
        }
        
        for joint_name, vals in defaults.items():
            self.joint_limits[joint_name] = {
                'min': min(vals['lower'], vals['upper']),
                'max': max(vals['lower'], vals['upper']),
                'lower': vals['lower'],
                'upper': vals['upper']
            }
        
        self.velocity_limits = {
            'shoulder_pan': 2.0, 'shoulder_lift': 2.0, 'elbow_flex': 2.5,
            'wrist_flex': 2.5, 'wrist_roll': 2.5, 'pen_holder': 3.0,
        }
    
    def print_limits(self):
        """Print loaded joint limits."""
        self.get_logger().info('='*60)
        self.get_logger().info('JOINT LIMITS (from URDF)')
        self.get_logger().info('='*60)
        
        for joint_name in self.joint_names:
            if joint_name in self.joint_limits:
                limits = self.joint_limits[joint_name]
                
                # Show actual range (min to max)
                actual_min = limits['min']
                actual_max = limits['max']
                min_deg = np.degrees(actual_min)
                max_deg = np.degrees(actual_max)
                
                # Show URDF lower/upper values
                lower_deg = np.degrees(limits['lower'])
                upper_deg = np.degrees(limits['upper'])
                
                self.get_logger().info(
                    f"{joint_name:<15} Range: {actual_min:>6.3f} to {actual_max:>6.3f} rad "
                    f"({min_deg:>6.1f}° to {max_deg:>6.1f}°)"
                )
                self.get_logger().info(
                    f"{'':15} URDF: lower={limits['lower']:>6.3f} ({lower_deg:>6.1f}°), "
                    f"upper={limits['upper']:>6.3f} ({upper_deg:>6.1f}°)"
                )
    
    def joint_state_callback(self, msg):
        """Store current joint states."""
        self.current_joint_states = msg
    
    def validate_position(self, joint_name, position):
        """Validate single joint position."""
        if joint_name not in self.joint_limits:
            return True, f"Unknown joint (not in URDF)"
        
        limits = self.joint_limits[joint_name]
        
        # CRITICAL: lower/upper in URDF are directional, not min/max!
        # The actual range is [min(lower, upper), max(lower, upper)]
        actual_min = min(limits['min'], limits['max'])
        actual_max = max(limits['min'], limits['max'])
        
        if position < actual_min:
            return False, f"Outside range: {position:.3f} < {actual_min:.3f}"
        if position > actual_max:
            return False, f"Outside range: {position:.3f} > {actual_max:.3f}"
        
        # Near limit warning (5% margin from either end)
        total_range = actual_max - actual_min
        if total_range > 0:
            dist_from_min = position - actual_min
            dist_from_max = actual_max - position
            
            if dist_from_min < 0.05 * total_range or dist_from_max < 0.05 * total_range:
                return True, f"Near limit (within 5% of boundary)"
        
        return True, "OK"
    
    def validate_pose(self, pose_name, positions):
        """Validate complete pose."""
        errors = []
        warnings = []
        
        for joint_name, position in positions.items():
            valid, message = self.validate_position(joint_name, position)
            
            if not valid:
                errors.append(f"{joint_name}: {message}")
                self.stats['validation_errors'] += 1
            elif message != "OK":
                warnings.append(f"{joint_name}: {message}")
                self.stats['validation_warnings'] += 1
        
        if errors or warnings:
            self.get_logger().info(f"\n📋 Validating: {pose_name}")
            
        if errors:
            self.get_logger().error(f"  ❌ ERRORS ({len(errors)}):")
            for error in errors:
                self.get_logger().error(f"     {error}")
        
        if warnings:
            self.get_logger().warn(f"  ⚠️  WARNINGS ({len(warnings)}):")
            for warning in warnings:
                self.get_logger().warn(f"     {warning}")
        
        if not errors and not warnings:
            self.get_logger().info(f"  ✓ {pose_name} valid")
        
        return len(errors) == 0
    
    def create_trajectory_msg(self, positions, duration_sec):
        """Create JointTrajectory message."""
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        
        # Build position array in correct order
        position_array = []
        for joint_name in self.joint_names:
            position_array.append(positions.get(joint_name, 0.0))
        
        point = JointTrajectoryPoint()
        point.positions = position_array
        point.time_from_start = Duration(sec=int(duration_sec), 
                                        nanosec=int((duration_sec % 1) * 1e9))
        
        msg.points = [point]
        return msg
    
    def send_pose(self, pose_name, positions, duration):
        """Send pose to robot."""
        msg = self.create_trajectory_msg(positions, duration)
        self.publisher.publish(msg)
        self.get_logger().info(f"  ⏳ Moving to {pose_name}")
    
    def spin_for_duration(self, duration):
        """Spin the node for a duration to allow callbacks to execute."""
        start_time = time.time()
        while (time.time() - start_time) < duration:
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.01)  # Small sleep to prevent CPU spinning
    
    def move_to_safe_position(self):
        """Move to safe position for servo testing."""
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('MOVING TO SAFE POSITION')
        self.get_logger().info('='*60)
        self.get_logger().info('Safe position: Shoulder up 90°, all else neutral')
        self.get_logger().info('This keeps arm away from base for safe testing')
        
        if self.validate:
            valid = self.validate_pose('safe_position', self.safe_position)
            if not valid:
                self.get_logger().error('Safe position validation failed!')
                return False
        
        self.send_pose('safe_position', self.safe_position, self.movement_time)
        self.spin_for_duration(self.movement_time + 1.0)
        
        self.get_logger().info('✓ Safe position reached')
        return True
    
    def servo_test_single_joint(self, joint_name):
        """Test a single joint through its range."""
        if joint_name not in self.joint_names:
            self.get_logger().error(f'Unknown joint: {joint_name}')
            return False
        
        if joint_name not in self.joint_limits:
            self.get_logger().error(f'No limits found for {joint_name}')
            return False
        
        limits = self.joint_limits[joint_name]
        min_pos = limits['min']
        max_pos = limits['max']
        mid_pos = (min_pos + max_pos) / 2.0
        
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info(f'SERVO TEST: {joint_name}')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Range: {min_pos:.3f} to {max_pos:.3f} rad')
        self.get_logger().info(f'       {np.degrees(min_pos):.1f}° to {np.degrees(max_pos):.1f}°')
        self.get_logger().info(f'Middle: {mid_pos:.3f} rad ({np.degrees(mid_pos):.1f}°)')
        self.get_logger().info('='*60)
        
        # Test sequence: middle -> min -> middle -> max -> middle
        test_sequence = [
            ('middle', mid_pos),
            ('minimum', min_pos),
            ('middle', mid_pos),
            ('maximum', max_pos),
            ('middle', mid_pos),
        ]
        
        for test_name, position in test_sequence:
            # Create pose with safe position + test joint
            test_pose = self.safe_position.copy()
            test_pose[joint_name] = position
            
            self.get_logger().info(f'\n[{joint_name}] → {test_name}: {position:.3f} rad ({np.degrees(position):.1f}°)')
            
            if self.validate:
                valid = self.validate_pose(f'{joint_name}_{test_name}', test_pose)
                if not valid:
                    self.get_logger().error('Validation failed - skipping')
                    continue
            
            self.send_pose(f'{joint_name}_{test_name}', test_pose, self.movement_time)
            
            # Wait for movement + delay (spin to allow callbacks)
            self.spin_for_duration(self.movement_time + self.delay)
            
            # Read telemetry if available
            if self.telemetry and self.telemetry.enabled:
                data = self.telemetry.get_joint_telemetry(joint_name)
                if data:
                    telemetry_str = f"  📊 {joint_name}: "
                    if 'position' in data:
                        telemetry_str += f"Pos: {data['position']:.3f}rad "
                    if 'effort' in data:
                        telemetry_str += f"Effort: {data['effort']:.3f}Nm "
                    if 'current_estimate_mA' in data:
                        telemetry_str += f"Current: ~{data['current_estimate_mA']:.0f}mA"
                    self.get_logger().info(telemetry_str)
        
        self.get_logger().info(f'\n✓ Servo test complete for {joint_name}')
        return True
    
    def servo_test_all_joints(self):
        """Test all joints sequentially."""
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('SERVO TEST: ALL JOINTS')
        self.get_logger().info('='*60)
        
        for joint_name in self.joint_names:
            if not self.servo_test_single_joint(joint_name):
                self.get_logger().error(f'Failed testing {joint_name}')
                return False
        
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('✓ All servo tests complete')
        self.get_logger().info('='*60)
        return True
    
    def load_poses_from_file(self, filename):
        """Load poses from YAML file."""
        try:
            with open(filename, 'r') as f:
                return yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f'Error loading {filename}: {e}')
            return None
    
    def get_programmatic_poses(self, program_name):
        """Get programmatic pose sequence."""
        if program_name == 'calibration':
            return [
                {'name': 'home', 'positions': {j: 0.0 for j in self.joint_names}},
                {'name': 'shoulder_up', 'positions': {'shoulder_lift': 1.57}},
                {'name': 'elbow_bent', 'positions': {'shoulder_lift': 1.57, 'elbow_flex': -1.57}},
                {'name': 'return_home', 'positions': {j: 0.0 for j in self.joint_names}},
            ]
        elif program_name == 'sweep':
            poses = []
            for i, angle in enumerate(np.linspace(-1.57, 1.57, 8)):
                poses.append({'name': f'pan_{i}', 'positions': {'shoulder_pan': angle}})
            return poses
        elif program_name == 'stress':
            return [
                {'name': 'horizontal', 'positions': {'shoulder_lift': 1.57}},
                {'name': 'extended', 'positions': {'shoulder_lift': 1.57, 'elbow_flex': 1.57}},
                {'name': 'overhead', 'positions': {'shoulder_lift': 3.14}},
            ]
        elif program_name == 'default':
            return [
                {'name': 'pose 0', 'positions': {'shoulder_pan': 1.1679, 'shoulder_lift': 2.60, 'elbow_flex': 1.193, 'wrist_flex': 2.6873, 'wrist_roll': 1.01906, 'pen_holder':  1.10}},
                {'name': 'pose 1', 'positions': {'shoulder_pan': 1.1679, 'shoulder_lift': 2.50, 'elbow_flex': 1.193, 'wrist_flex': 2.4556, 'wrist_roll': 0.92544, 'pen_holder':  1.58}},
                {'name': 'pose 2', 'positions': {'shoulder_pan': 1.9874, 'shoulder_lift': 2.50, 'elbow_flex': 1.000, 'wrist_flex': 1.5908, 'wrist_roll': -1.4083, 'pen_holder':  0.21}},
                {'name': 'pose 3', 'positions': {'shoulder_pan': 1.4533, 'shoulder_lift': 2.56, 'elbow_flex': 1.105, 'wrist_flex': 1.2992, 'wrist_roll': -0.3728, 'pen_holder':  1.59}},
                {'name': 'pose 4', 'positions': {'shoulder_pan': 1.8125, 'shoulder_lift': 2.56, 'elbow_flex': 1.632, 'wrist_flex': 2.4402, 'wrist_roll': 1.1915, 'pen_holder':  0.5}},
                {'name': 'pose 5', 'positions': {'shoulder_pan': 1.8125, 'shoulder_lift': 2.56, 'elbow_flex': 1.632, 'wrist_flex': 2.4402, 'wrist_roll': 1.1915, 'pen_holder':  1.5}},
            ]
        else:
            self.get_logger().error(f'Unknown program: {program_name}')
            return None
    
    def run_pose_sequence(self, poses):
        """Run sequence of poses."""
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('POSE SEQUENCE TEST')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Total poses: {len(poses)}')
        self.get_logger().info(f'Validation: {self.validate}')
        self.get_logger().info('='*60)
        
        for idx, pose in enumerate(poses, 1):
            pose_name = pose.get('name', f'pose_{idx}')
            positions_dict = pose.get('positions', {})
            
            # Fill in missing joints with safe position defaults
            positions = self.safe_position.copy()
            positions.update(positions_dict)
            
            self.get_logger().info(f'\n[{idx}/{len(poses)}] {pose_name}')
            
            if self.validate:
                valid = self.validate_pose(pose_name, positions)
                if not valid:
                    self.get_logger().error('Validation failed - aborting sequence')
                    return False
            
            self.send_pose(pose_name, positions, self.movement_time)
            
            # Wait for movement + delay (spin to allow callbacks)
            self.spin_for_duration(self.movement_time + self.delay)
            
            # Read telemetry if available (BUG FIX: was missing!)
            if self.telemetry and self.telemetry.enabled:
                all_data = self.telemetry.get_all_joints_telemetry()
                if all_data:
                    self.get_logger().info("  📊 Joint States:")
                    for joint_name, data in all_data.items():
                        if joint_name in positions_dict:  # Only show joints that moved
                            telemetry_str = f"    {joint_name:<15} "
                            if 'position' in data:
                                telemetry_str += f"Pos: {data['position']:>6.3f}rad "
                            if 'effort' in data:
                                telemetry_str += f"Effort: {data['effort']:>6.3f}Nm"
                            self.get_logger().info(telemetry_str)
            
            self.stats['total_poses'] += 1
        
        self.print_summary()
        return True
    
    def print_summary(self):
        """Print test summary."""
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('SUMMARY')
        self.get_logger().info('='*60)
        self.get_logger().info(f"Poses tested: {self.stats['total_poses']}")
        if self.validate:
            self.get_logger().info(f"Validation errors: {self.stats['validation_errors']}")
            self.get_logger().info(f"Validation warnings: {self.stats['validation_warnings']}")
        self.get_logger().info('='*60)
    
    def run(self):
        """Main execution."""
        # Wait for joint trajectory controller
        self.get_logger().info('Waiting for joint trajectory controller...')
        self.spin_for_duration(2.0)
        
        # Servo test mode
        if self.servo_test_mode:
            # Move to safe position first
            if self.safe_position_first:
                if not self.move_to_safe_position():
                    return
            
            # Test specific joint or all joints
            if self.servo_test_joint:
                self.servo_test_single_joint(self.servo_test_joint)
            else:
                self.servo_test_all_joints()
        
        # Pose sequence mode
        elif self.poses_file or self.program:
            # Load poses
            poses = None
            if self.poses_file:
                poses = self.load_poses_from_file(self.poses_file)
                if poses:
                    self.get_logger().info(f'✓ Loaded {len(poses)} poses from {self.poses_file}')
            elif self.program:
                poses = self.get_programmatic_poses(self.program)
                if poses:
                    self.get_logger().info(f'✓ Using program: {self.program}')
            
            if poses:
                self.run_pose_sequence(poses)
            else:
                self.get_logger().error('No poses to execute')
        
        else:
            self.get_logger().error('Must specify poses_file, program, or servo_test')
            self.get_logger().info('Examples:')
            self.get_logger().info('  ros2 run PKG pose_test --ros-args -p poses_file:=poses.yaml')
            self.get_logger().info('  ros2 run PKG pose_test --ros-args -p program:=calibration')
            self.get_logger().info('  ros2 run PKG pose_test --ros-args -p servo_test:=true')
            self.get_logger().info('  ros2 run PKG pose_test --ros-args -p servo_test:=true -p servo_test_joint:=shoulder_lift')


def main(args=None):
    rclpy.init(args=args)
    
    node = PoseTestNode()
    
    try:
        node.run()
        time.sleep(1.0)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    finally:
        if node.telemetry:
            node.telemetry.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
