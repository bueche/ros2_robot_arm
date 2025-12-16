#!/usr/bin/env python3
"""
ROS2 node for Koch v1.1 robot arm servo testing and pose sequences.

Usage:
  ros2 run writing_robot_control pose_test --zero_pose
  ros2 run writing_robot_control pose_test --setup_servo_test 3
  ros2 run writing_robot_control pose_test --servo_test 3
  ros2 run writing_robot_control pose_test --sequence_pose default 3 --delay 2.0
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
import time
import argparse
import sys
import threading
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory
import os


class PoseTestNode(Node):
    def __init__(self, urdf_path=None):
        super().__init__('pose_test_node')
        
        # Create publisher for joint trajectory
        self.publisher = self.create_publisher(
            JointTrajectory,
            '/koch_v11_controller/joint_trajectory',
            10
        )
        
        # Subscribe to joint states for position verification
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Store latest joint states
        self.current_joint_positions = {}
        self.joint_state_lock = threading.Lock()
        self.joint_states_received = False
        
        # Position tolerance (can be overridden)
        self.position_tolerance = 1.0
        
        # Hold points configuration (can be overridden)
        self.use_hold_points = True  # Default to using hold points
        self.hold_duration = 0.5  # Additional hold time after reaching position
        
        # Define joint names (6 joints total)
        self.joint_names = [
            'shoulder_pan',
            'shoulder_lift', 
            'elbow_flex',
            'wrist_flex',
            'wrist_roll',
            'pen_holder'
        ]
        
        # Parse URDF to get joint limits dynamically
        self.joint_limits = self.parse_urdf_limits(urdf_path)
        
        if not self.joint_limits:
            self.get_logger().warn("Could not parse URDF limits, using fallback hardcoded limits")
            self.joint_limits = self.get_fallback_limits()
        else:
            self.get_logger().info(f"Loaded joint limits from URDF: {urdf_path if urdf_path else 'default'}")
        
        # Display loaded limits
        self.get_logger().info("Joint limits loaded:")
        for joint_name in self.joint_names:
            limits = self.joint_limits.get(joint_name, {})
            self.get_logger().info(
                f"  {joint_name}: [{limits.get('min', 'N/A'):.3f}, {limits.get('max', 'N/A'):.3f}] rad"
            )
        
        # Safe raised position for servo testing
        # This keeps the arm elevated to prevent collisions during single-joint tests
        self.safe_test_position = [
            1.292,   # shoulder_pan: centered
            2.688,   # shoulder_lift: raised up (near max)
            -1.063,  # elbow_flex: centered
            2.499,   # wrist_flex: centered
            -0.014,  # wrist_roll: centered
            0.895    # pen_holder: half-open
        ]
        
        # Define pose sequences
        self.sequences = {
            'default': [
                {
                    'name': 'Pose 0',
                    'positions': [1.1679296563996093, 2.607506552198339, -1.1937930235710798,
                                  2.6873125208353694, 1.0190608302882267, 1.10],
                    'duration': 2
                },
                {
                    'name': 'Pose 1',
                    'positions': [1.1617907357352224, 2.50, -1.1937930235710798, 
                                  2.455568265754763, 0.9254422901563264, 1.58],
                    'duration': 2
                },
                {
                    'name': 'Pose 2',
                    'positions': [1.9874755650952616, 2.50, -1.00,
                                  1.5908711139472399, -1.408349834228516, 0.21],
                    'duration': 2
                },
                {
                    'name': 'Pose 3',
                    'positions': [1.4533894672936005, 2.5660688377137277, -1.105168763849536,
                                  1.2992723823888618, -0.3727573310302735, 1.59],
                    'duration': 2
                },
                {
                    'name': 'Pose 4',
                    'positions': [1.8125163261602346, 2.564534107547631, -1.6329528967269173,
                                  2.440220964093796, 1.191594677186126, 0.5],
                    'duration': 2
                },
                {
                    'name': 'Pose 5',
                    'positions': [1.8125163261602346, 2.564534107547631, -1.6329528967269173,
                                  2.440220964093796, 1.191594677186126, 1.5],
                    'duration': 2
                }
            ]
        }
        
        self.get_logger().info('Pose Test Node initialized')
    
    def parse_urdf_limits(self, urdf_path=None):
        """Parse joint limits from URDF file.
        
        Reads limits from ros2_control section (software limits) which are
        typically more conservative than hardware limits.
        """
        try:
            # If no path provided, try to find the URDF in the package
            if urdf_path is None:
                try:
                    pkg_share = get_package_share_directory('writing_robot_description')
                    urdf_path = os.path.join(pkg_share, 'urdf', 'koch_v11_arm_real.urdf')
                except:
                    self.get_logger().warn("Could not locate URDF package, trying uploaded file path")
                    urdf_path = '/mnt/user-data/uploads/koch_v11_arm_real.urdf'
            
            if not os.path.exists(urdf_path):
                self.get_logger().warn(f"URDF file not found: {urdf_path}")
                return None
            
            # Parse the URDF XML
            tree = ET.parse(urdf_path)
            root = tree.getroot()
            
            # Find the ros2_control section
            ros2_control = root.find('.//ros2_control')
            if ros2_control is None:
                self.get_logger().warn("No ros2_control section found in URDF")
                return None
            
            # Extract limits for each joint
            joint_limits = {}
            servo_id = 1  # Assume sequential IDs
            
            for joint in ros2_control.findall('.//joint'):
                joint_name = joint.get('name')
                if joint_name not in self.joint_names:
                    continue
                
                # Find position command interface with min/max params
                cmd_interface = joint.find(".//command_interface[@name='position']")
                if cmd_interface is not None:
                    min_param = cmd_interface.find(".//param[@name='min']")
                    max_param = cmd_interface.find(".//param[@name='max']")
                    
                    if min_param is not None and max_param is not None:
                        joint_limits[joint_name] = {
                            'min': float(min_param.text),
                            'max': float(max_param.text),
                            'id': servo_id
                        }
                        servo_id += 1
            
            # Verify we got all joints
            if len(joint_limits) != len(self.joint_names):
                self.get_logger().warn(
                    f"Only found {len(joint_limits)}/{len(self.joint_names)} joint limits in URDF"
                )
                return None
            
            return joint_limits
            
        except Exception as e:
            self.get_logger().error(f"Error parsing URDF: {e}")
            return None
    
    def get_fallback_limits(self):
        """Fallback hardcoded limits if URDF parsing fails.
        
        These are the limits from your koch_v11_arm_real.urdf ros2_control section.
        """
        return {
            'shoulder_pan': {'min': 0.540, 'max': 2.044, 'id': 1},
            'shoulder_lift': {'min': 2.486, 'max': 2.8857, 'id': 2},
            'elbow_flex': {'min': -1.200, 'max': -0.926, 'id': 3},
            'wrist_flex': {'min': 0.297, 'max': 2.70, 'id': 4},
            'wrist_roll': {'min': -1.475, 'max': 1.448, 'id': 5},
            'pen_holder': {'min': 0.19, 'max': 1.60, 'id': 6}
        }
    
    def joint_state_callback(self, msg):
        """Callback to store current joint positions."""
        with self.joint_state_lock:
            for i, name in enumerate(msg.name):
                if name in self.joint_names:
                    self.current_joint_positions[name] = msg.position[i]
            self.joint_states_received = True
    
    def get_current_positions(self):
        """Get current positions for all joints."""
        with self.joint_state_lock:
            positions = []
            for joint_name in self.joint_names:
                positions.append(self.current_joint_positions.get(joint_name, 0.0))
            return positions
    
    def wait_for_joint_states(self, timeout=5.0):
        """Wait for joint states to be received."""
        start_time = time.time()
        while not self.joint_states_received and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if not self.joint_states_received:
            self.get_logger().warn("No joint states received - position verification disabled")
            return False
        return True
    
    def validate_positions(self, positions, description=""):
        """Validate that positions are within joint limits.
        
        Handles cases where min > max numerically (e.g., min=2.0, max=0.5)
        which can occur depending on joint axis direction and servo mounting.
        """
        violations = []
        warnings = []
        
        for i, joint_name in enumerate(self.joint_names):
            limits = self.joint_limits[joint_name]
            pos = positions[i]
            
            # Get the actual numeric range (handle min > max case)
            # "min" and "max" are semantic (physical extremes)
            # but numerically they might be reversed
            lower_bound = min(limits['min'], limits['max'])
            upper_bound = max(limits['min'], limits['max'])
            
            # Check if position is outside the valid range
            if pos < lower_bound or pos > upper_bound:
                violations.append(
                    f"  ❌ {joint_name}: {pos:.3f} rad is OUTSIDE valid range "
                    f"[{lower_bound:.3f}, {upper_bound:.3f}] "
                    f"(URDF: min={limits['min']:.3f}, max={limits['max']:.3f})"
                )
            # Check if position is close to limits (within 5%)
            else:
                range_size = abs(limits['max'] - limits['min'])
                margin = range_size * 0.05
                
                if pos < lower_bound + margin or pos > upper_bound - margin:
                    warnings.append(
                        f"  ⚠️  {joint_name}: {pos:.3f} rad is near limit "
                        f"[{lower_bound:.3f}, {upper_bound:.3f}]"
                    )
        
        if violations:
            self.get_logger().error(f"POSITION VALIDATION FAILED for: {description}")
            for v in violations:
                self.get_logger().error(v)
            return False
        
        if warnings:
            self.get_logger().warn(f"Position warnings for: {description}")
            for w in warnings:
                self.get_logger().warn(w)
        
        return True
    
    def create_trajectory_msg(self, positions, duration_sec, hold_duration=None):
        """Create a JointTrajectory message, optionally with a hold point.
        
        Args:
            positions: Target joint positions
            duration_sec: Time to reach target position
            hold_duration: If provided, adds a hold point at same position
        """
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        
        # First point: Move to target position
        point1 = JointTrajectoryPoint()
        point1.positions = positions
        point1.time_from_start = Duration(sec=int(duration_sec), 
                                        nanosec=int((duration_sec % 1) * 1e9))
        
        msg.points = [point1]
        
        # Second point: Hold at same position (prevents droop)
        if hold_duration is not None and hold_duration > 0:
            point2 = JointTrajectoryPoint()
            point2.positions = positions  # Same position!
            total_time = duration_sec + hold_duration
            point2.time_from_start = Duration(sec=int(total_time),
                                            nanosec=int((total_time % 1) * 1e9))
            msg.points.append(point2)
        
        return msg
    
    def publish_and_wait(self, positions, duration_sec, description=""):
        """Publish trajectory, wait for completion, and verify positions."""
        try:
            # Validate positions before sending (unless skip_validation is set)
            if not getattr(self, 'skip_validation', False):
                if not self.validate_positions(positions, description):
                    self.get_logger().error(f"SKIPPING {description} - position(s) outside safe limits")
                    self.get_logger().error("Use --skip_validation to override (DANGEROUS!)")
                    return False
            
            self.get_logger().info(f"Moving to: {description}")
            self.get_logger().info(f"  Target positions: {[f'{p:.3f}' for p in positions]}")
            
            # Use hold points if enabled
            hold_dur = self.hold_duration if self.use_hold_points else None
            
            if self.use_hold_points:
                self.get_logger().info(f"  Using hold point (move: {duration_sec}s, hold: {hold_dur}s)")
            
            msg = self.create_trajectory_msg(positions, duration_sec, hold_dur)
            self.publisher.publish(msg)
            
            # Total trajectory time
            total_time = duration_sec + (hold_dur if hold_dur else 0)
            self.get_logger().info(f"  Total duration: {total_time}s")
            
            # Wait for movement with periodic position checks
            check_interval = 0.5
            elapsed = 0.0
            while elapsed < total_time:
                time.sleep(min(check_interval, total_time - elapsed))
                rclpy.spin_once(self, timeout_sec=0.0)
                elapsed += check_interval
            
            # Additional settling time
            time.sleep(0.5)
            rclpy.spin_once(self, timeout_sec=0.1)
            
            # Verify final positions if joint states are available
            if self.joint_states_received:
                actual_positions = self.get_current_positions()
                max_error = 0.0
                max_error_joint = None
                errors_found = False
                
                for i, joint_name in enumerate(self.joint_names):
                    error = abs(positions[i] - actual_positions[i])
                    if error > max_error:
                        max_error = error
                        max_error_joint = joint_name
                    
                    if error > self.position_tolerance:
                        self.get_logger().error(
                            f"  ⚠️  {joint_name}: Position error {error:.3f} rad "
                            f"(target: {positions[i]:.3f}, actual: {actual_positions[i]:.3f}, "
                            f"tolerance: {self.position_tolerance:.3f})"
                        )
                        errors_found = True
                
                if errors_found:
                    self.get_logger().error(
                        f"  ❌ TOLERANCE VIOLATION - Movement may have failed!"
                    )
                    self.get_logger().error(
                        f"  Largest error: {max_error_joint} = {max_error:.3f} rad"
                    )
                    return False
                else:
                    self.get_logger().info(
                        f"  ✓ Position verified (max error: {max_error:.4f} rad on {max_error_joint})"
                    )
            else:
                self.get_logger().warn("  ⚠️  Position verification skipped (no joint states)")
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"ERROR in {description}: {str(e)}")
            return False
    
    def zero_pose(self):
        """Move all joints to zero position (rpy = 0,0,0)."""
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("ZERO POSE COMMAND")
        self.get_logger().info("="*60)
        
        # Wait for joint states
        self.get_logger().info("Waiting for joint states...")
        self.wait_for_joint_states()
        
        if self.joint_states_received:
            current = self.get_current_positions()
            self.get_logger().info(f"Current positions: {[f'{p:.3f}' for p in current]}")
        
        zero_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Check if zero position is within limits for each joint
        warnings = []
        for i, joint_name in enumerate(self.joint_names):
            limits = self.joint_limits[joint_name]
            if zero_positions[i] < limits['min'] or zero_positions[i] > limits['max']:
                warnings.append(f"  WARNING: {joint_name} zero position (0.0) is outside "
                              f"safe limits [{limits['min']:.3f}, {limits['max']:.3f}]")
        
        if warnings:
            self.get_logger().warn("Zero pose may exceed safe joint limits:")
            for warning in warnings:
                self.get_logger().warn(warning)
            self.get_logger().warn("Consider using a different 'home' position instead.")
        
        success = self.publish_and_wait(zero_positions, 3.0, "Zero Pose")
        
        if success:
            self.get_logger().info("Zero pose command completed")
        else:
            self.get_logger().error("Zero pose command FAILED")
        
        return success
    
    def setup_servo_test(self, servo_id):
        """Move to safe position for testing a specific servo."""
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info(f"SETUP SERVO TEST - Servo ID {servo_id}")
        self.get_logger().info("="*60)
        
        # Wait for joint states
        self.get_logger().info("Waiting for joint states...")
        self.wait_for_joint_states()
        
        if self.joint_states_received:
            current = self.get_current_positions()
            self.get_logger().info(f"Current positions: {[f'{p:.3f}' for p in current]}")
        
        # Find joint name for this servo ID
        joint_name = None
        for name, limits in self.joint_limits.items():
            if limits['id'] == servo_id:
                joint_name = name
                break
        
        if joint_name is None:
            self.get_logger().error(f"ERROR: Invalid servo ID {servo_id}. Must be 1-6")
            return False
        
        self.get_logger().info(f"Preparing to test: {joint_name} (Servo ID {servo_id})")
        self.get_logger().info("Moving arm to safe raised position...")
        
        success = self.publish_and_wait(
            self.safe_test_position, 
            4.0, 
            f"Safe position for testing {joint_name}"
        )
        
        if success:
            self.get_logger().info(f"Ready to test {joint_name} (Servo ID {servo_id})")
            self.get_logger().info(f"Joint limits: [{self.joint_limits[joint_name]['min']:.3f}, "
                                 f"{self.joint_limits[joint_name]['max']:.3f}]")
        else:
            self.get_logger().error("Setup FAILED")
        
        return success
    
    def servo_test(self, servo_id):
        """Test a specific servo through its full range of motion."""
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info(f"SERVO TEST - Servo ID {servo_id}")
        self.get_logger().info("="*60)
        
        # Wait for joint states
        self.get_logger().info("Waiting for joint states...")
        self.wait_for_joint_states()
        
        if self.joint_states_received:
            current = self.get_current_positions()
            self.get_logger().info(f"Current positions: {[f'{p:.3f}' for p in current]}")
        
        # Find joint name and index
        joint_name = None
        joint_index = None
        for i, name in enumerate(self.joint_names):
            if self.joint_limits[name]['id'] == servo_id:
                joint_name = name
                joint_index = i
                break
        
        if joint_name is None:
            self.get_logger().error(f"ERROR: Invalid servo ID {servo_id}. Must be 1-6")
            return False
        
        limits = self.joint_limits[joint_name]
        self.get_logger().info(f"Testing: {joint_name} (Servo ID {servo_id})")
        self.get_logger().info(f"Range: [{limits['min']:.3f}, {limits['max']:.3f}]")
        
        # Calculate test positions
        min_pos = limits['min']
        max_pos = limits['max']
        mid_pos = (min_pos + max_pos) / 2.0
        
        # Test sequence: center -> min -> center -> max -> center
        test_sequence = [
            ('Center', mid_pos),
            ('Minimum', min_pos),
            ('Center', mid_pos),
            ('Maximum', max_pos),
            ('Center', mid_pos)
        ]
        
        self.get_logger().info("\nTest sequence:")
        for step_name, position in test_sequence:
            self.get_logger().info(f"  {step_name}: {position:.3f}")
        
        # Execute test sequence
        all_success = True
        for step_num, (step_name, target_position) in enumerate(test_sequence, 1):
            # Create position array with all joints at safe position except the test joint
            positions = self.safe_test_position.copy()
            positions[joint_index] = target_position
            
            self.get_logger().info(f"\nStep {step_num}/{len(test_sequence)}: {step_name}")
            success = self.publish_and_wait(
                positions,
                2.0,
                f"{joint_name} to {step_name} ({target_position:.3f})"
            )
            
            if not success:
                self.get_logger().error(f"FAILED at step {step_num}: {step_name}")
                all_success = False
                break
            
            # Pause between movements
            time.sleep(1.0)
        
        if all_success:
            self.get_logger().info("\n" + "="*60)
            self.get_logger().info(f"Servo test for ID {servo_id} COMPLETED successfully")
            self.get_logger().info("="*60)
        else:
            self.get_logger().error("\n" + "="*60)
            self.get_logger().error(f"Servo test for ID {servo_id} FAILED")
            self.get_logger().error("="*60)
        
        return all_success
    
    def sequence_pose(self, sequence_name, up_to, delay):
        """Execute a pose sequence up to a specified number of poses."""
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info(f"SEQUENCE: {sequence_name} (up to pose {up_to})")
        self.get_logger().info("="*60)
        
        # Wait for joint states
        self.get_logger().info("Waiting for joint states...")
        self.wait_for_joint_states()
        
        if self.joint_states_received:
            current = self.get_current_positions()
            self.get_logger().info(f"Starting positions: {[f'{p:.3f}' for p in current]}")
        
        if sequence_name not in self.sequences:
            self.get_logger().error(f"ERROR: Unknown sequence '{sequence_name}'")
            self.get_logger().info(f"Available sequences: {list(self.sequences.keys())}")
            return False
        
        sequence = self.sequences[sequence_name]
        num_poses = min(up_to, len(sequence))
        
        self.get_logger().info(f"Executing {num_poses} pose(s) from '{sequence_name}' sequence")
        self.get_logger().info(f"Delay between poses: {delay}s")
        
        all_success = True
        for i in range(num_poses):
            pose = sequence[i]
            
            self.get_logger().info(f"\n{'='*50}")
            self.get_logger().info(f"Pose {i+1}/{num_poses}: {pose['name']}")
            
            success = self.publish_and_wait(
                pose['positions'],
                pose['duration'],
                pose['name']
            )
            
            if not success:
                self.get_logger().error(f"FAILED at pose {i+1}: {pose['name']}")
                all_success = False
                break
            
            # Wait for additional delay if not the last pose
            if i < num_poses - 1:
                self.get_logger().info(f"Pausing for {delay}s...")
                time.sleep(delay)
        
        if all_success:
            self.get_logger().info("\n" + "="*60)
            self.get_logger().info("Sequence COMPLETED successfully")
            self.get_logger().info("="*60)
        else:
            self.get_logger().error("\n" + "="*60)
            self.get_logger().error("Sequence FAILED")
            self.get_logger().error("="*60)
        
        return all_success
    
    def validate_sequence(self, sequence_name):
        """Validate all poses in a sequence without executing."""
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info(f"VALIDATING SEQUENCE: {sequence_name}")
        self.get_logger().info("="*60)
        
        if sequence_name not in self.sequences:
            self.get_logger().error(f"ERROR: Unknown sequence '{sequence_name}'")
            self.get_logger().info(f"Available sequences: {list(self.sequences.keys())}")
            return False
        
        sequence = self.sequences[sequence_name]
        self.get_logger().info(f"Checking {len(sequence)} poses...")
        
        all_valid = True
        for i, pose in enumerate(sequence):
            self.get_logger().info(f"\n{'='*50}")
            self.get_logger().info(f"Pose {i+1}/{len(sequence)}: {pose['name']}")
            
            if not self.validate_positions(pose['positions'], pose['name']):
                all_valid = False
                self.get_logger().error(f"  ❌ {pose['name']} has INVALID positions")
            else:
                self.get_logger().info(f"  ✓ {pose['name']} is valid")
        
        self.get_logger().info(f"\n{'='*60}")
        if all_valid:
            self.get_logger().info(f"VALIDATION PASSED: All {len(sequence)} poses are within limits")
        else:
            self.get_logger().error("VALIDATION FAILED: Some poses have positions outside safe limits")
            self.get_logger().error("These poses need to be corrected before execution!")
        self.get_logger().info("="*60)
        
        return all_valid


def main(args=None):
    # Parse command line arguments
    parser = argparse.ArgumentParser(
        description='Koch v1.1 Robot Arm Pose Testing Utility',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Validate sequence before running
  ros2 run writing_robot_control pose_test --validate_sequence default
  
  # Move to zero pose
  ros2 run writing_robot_control pose_test --zero_pose
  
  # Setup for testing servo 3 (elbow_flex)
  ros2 run writing_robot_control pose_test --setup_servo_test 3
  
  # Test servo 3 through its full range
  ros2 run writing_robot_control pose_test --servo_test 3
  
  # Run first 3 poses of default sequence (with hold points)
  ros2 run writing_robot_control pose_test --sequence_pose default 3
  
  # Run all poses with 3 second delay and 1 second hold
  ros2 run writing_robot_control pose_test --sequence_pose default 100 --delay 3.0 --hold_duration 1.0
  
  # Disable hold points (single-point trajectories only)
  ros2 run writing_robot_control pose_test --sequence_pose default 5 --no_hold_points
  
  # Custom hold duration for preventing droop on extended arm
  ros2 run writing_robot_control pose_test --servo_test 4 --hold_duration 2.0
        """
    )
    
    parser.add_argument('--zero_pose', action='store_true',
                       help='Move all servos to zero position (rpy = 0,0,0)')
    
    parser.add_argument('--setup_servo_test', type=int, metavar='SERVO_ID',
                       help='Setup for testing specific servo (1-6). Moves arm to safe position.')
    
    parser.add_argument('--servo_test', type=int, metavar='SERVO_ID',
                       help='Test specific servo (1-6) through its full range')
    
    parser.add_argument('--sequence_pose', nargs=2, metavar=('SEQUENCE_NAME', 'UP_TO'),
                       help='Run pose sequence up to specified number')
    
    parser.add_argument('--validate_sequence', type=str, metavar='SEQUENCE_NAME',
                       help='Validate all poses in sequence without executing')
    
    parser.add_argument('--delay', type=float, default=2.0,
                       help='Delay in seconds between poses (default: 2.0)')
    
    parser.add_argument('--tolerance', type=float, default=1.0,
                       help='Position error tolerance in radians (default: 1.0)')
    
    parser.add_argument('--no_hold_points', action='store_true',
                       help='Disable hold points (use single-point trajectories)')
    
    parser.add_argument('--hold_duration', type=float, default=0.5,
                       help='Hold duration in seconds after reaching position (default: 0.5)')
    
    parser.add_argument('--skip_validation', action='store_true',
                       help='Skip position validation (DANGEROUS - may damage servos!)')
    
    parser.add_argument('--urdf', type=str, default=None,
                       help='Path to URDF file (default: auto-detect from package)')
    
    # Parse arguments (skip ROS args)
    parsed_args, unknown = parser.parse_known_args()
    
    # Check that exactly one command was specified
    commands = [parsed_args.zero_pose, 
                parsed_args.setup_servo_test is not None,
                parsed_args.servo_test is not None,
                parsed_args.sequence_pose is not None,
                parsed_args.validate_sequence is not None]
    
    if sum(commands) != 1:
        parser.print_help()
        print("\nERROR: Must specify exactly one command")
        return 1
    
    # Initialize ROS2
    rclpy.init(args=args)
    node = PoseTestNode(urdf_path=parsed_args.urdf)
    
    # Set tolerance if specified
    node.position_tolerance = parsed_args.tolerance
    
    # Set hold point configuration
    node.use_hold_points = not parsed_args.no_hold_points
    node.hold_duration = parsed_args.hold_duration
    
    # Set validation setting
    node.skip_validation = parsed_args.skip_validation
    
    if node.use_hold_points:
        node.get_logger().info(f"Hold points ENABLED (duration: {node.hold_duration}s)")
    else:
        node.get_logger().info("Hold points DISABLED (single-point trajectories)")
    
    if node.skip_validation:
        node.get_logger().warn("Position validation DISABLED - servos may be damaged!")
    
    try:
        success = False
        
        if parsed_args.zero_pose:
            success = node.zero_pose()
        
        elif parsed_args.setup_servo_test is not None:
            servo_id = parsed_args.setup_servo_test
            if servo_id < 1 or servo_id > 6:
                node.get_logger().error("ERROR: Servo ID must be between 1 and 6")
                return 1
            success = node.setup_servo_test(servo_id)
        
        elif parsed_args.servo_test is not None:
            servo_id = parsed_args.servo_test
            if servo_id < 1 or servo_id > 6:
                node.get_logger().error("ERROR: Servo ID must be between 1 and 6")
                return 1
            success = node.servo_test(servo_id)
        
        elif parsed_args.sequence_pose is not None:
            sequence_name = parsed_args.sequence_pose[0]
            try:
                up_to = int(parsed_args.sequence_pose[1])
            except ValueError:
                node.get_logger().error("ERROR: UP_TO must be an integer")
                return 1
            
            success = node.sequence_pose(sequence_name, up_to, parsed_args.delay)
        
        elif parsed_args.validate_sequence is not None:
            sequence_name = parsed_args.validate_sequence
            success = node.validate_sequence(sequence_name)
        
        # Keep node alive briefly to ensure last message is sent
        time.sleep(1.0)
        
        return 0 if success else 1
        
    except KeyboardInterrupt:
        node.get_logger().info('\nInterrupted by user')
        return 1
    except Exception as e:
        node.get_logger().error(f'Unexpected error: {str(e)}')
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
