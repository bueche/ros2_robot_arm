#!/usr/bin/env python3
"""
ROS2 Pose Testing Node with Validation and Servo Testing
Version 17 - Updated with optional power monitoring

Features:
- ROS2 node publishing JointTrajectory messages
- Reads joint limits from URDF parameter
- Servo test mode (safe position + test min/max/middle per joint)
- Full pose sequence testing
- Dual-topic telemetry (joint_states + dxl_state)
- Temperature, current, voltage monitoring
- Thermal and overload detection
- Validation with URDF limits
- **NEW**: Optional power monitoring with INA219 sensors (v17)

Usage:
  ros2 run writing_robot_control pose_test --ros-args -p poses_file:=poses.yaml
  ros2 run writing_robot_control pose_test --ros-args -p servo_test:=true
  ros2 run writing_robot_control pose_test --ros-args -p servo_test:=true -p telemetry:=true
  ros2 run writing_robot_control pose_test --ros-args -p servo_test_joint:=shoulder_lift
  ros2 run writing_robot_control pose_test --ros-args -p program:=default -p power_monitoring:=true
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from dynamixel_interfaces.msg import DynamixelState
from builtin_interfaces.msg import Duration
import yaml
import time
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np

# Optional power monitoring imports
try:
    from koch_v1_1_msgs.msg import PowerTelemetry, PoseEvent
    from std_msgs.msg import Header
    POWER_MONITORING_AVAILABLE = True
except ImportError:
    POWER_MONITORING_AVAILABLE = False

print(f"POWER_MONITORING_AVAILABLEF: {POWER_MONITORING_AVAILABLE}")

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


class PowerMonitor:
    """Optional power monitoring integration."""
    
    def __init__(self, node):
        self.node = node
        self.enabled = False
        self.power_samples = []
        self.final_samples = []  # Last few samples for holding current comparison
        self.tracking_active = False
        
        if not POWER_MONITORING_AVAILABLE:
            self.node.get_logger().info('Power monitoring messages not available')
            return
        
        try:
            # Publisher for pose events
            self.pose_event_pub = self.node.create_publisher(
                PoseEvent, 'pose_events', 10
            )
            
            # Subscriber for power telemetry
            self.power_sub = self.node.create_subscription(
                PowerTelemetry, 'power_telemetry',
                self._power_callback, 10
            )
            
            self.enabled = True
            self.node.get_logger().info('✓ Power monitoring enabled')
            
        except Exception as e:
            self.node.get_logger().warn(f'Power monitoring disabled: {e}')
    
    def _power_callback(self, msg):
        """Collect power samples during pose transitions."""
        if self.tracking_active:
            self.power_samples.append({
                'timestamp': time.time(),
                'bus_12v': msg.bus_12v_voltage,
                'current_12v': msg.current_12v,
                'power_12v': msg.power_12v,
                'bus_5v': msg.bus_5v_voltage,
                'current_5v': msg.current_5v,
                'power_5v': msg.power_5v,
                'total_power': msg.total_power
            })
    
    def start_tracking(self, pose_name, pose_number):
        """Start power tracking and publish pose start event."""
        if not self.enabled:
            return
        
        self.power_samples = []
        self.tracking_active = True
        
        # Publish pose start event
        msg = PoseEvent()
        msg.header = Header()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose_name = pose_name
        msg.pose_number = pose_number
        msg.event_type = 'start'
        self.pose_event_pub.publish(msg)
    
    def stop_tracking(self, pose_name, pose_number):
        """Stop power tracking and publish pose end event."""
        if not self.enabled:
            return
        
        # Capture last 4 samples for holding current comparison
        if len(self.power_samples) >= 4:
            self.final_samples = self.power_samples[-4:]
        else:
            self.final_samples = self.power_samples.copy()
        
        self.tracking_active = False
        
        # Publish pose end event
        msg = PoseEvent()
        msg.header = Header()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose_name = pose_name
        msg.pose_number = pose_number
        msg.event_type = 'end'
        self.pose_event_pub.publish(msg)
    
    def analyze_and_report(self, pose_name, servo_telemetry, peak_telemetry=None):
        """Analyze power draw and compare with servo telemetry."""
        if not self.enabled or len(self.power_samples) == 0:
            return
        
        # Find peak values
        peak_5v_current = max(s['current_5v'] for s in self.power_samples)
        peak_12v_current = max(s['current_12v'] for s in self.power_samples)
        peak_5v_voltage = max(s['bus_5v'] for s in self.power_samples)
        min_5v_voltage = min(s['bus_5v'] for s in self.power_samples)
        peak_total_power = max(s['total_power'] for s in self.power_samples)
        
        # Sum up peak currents from XL330s (5V servos) if we have peak data
        xl330_peak_sum_A = 0
        if peak_telemetry:
            for joint_name, data in peak_telemetry.items():
                if data.get('servo_type') == 'XL330' and 'current_ma' in data:
                    xl330_peak_sum_A += abs(data['current_ma']) / 1000.0
        
        # Sum up holding currents from XL330s (5V servos) if we have servo data
        xl330_holding_sum_A = 0
        if servo_telemetry:
            for joint_name, data in servo_telemetry.items():
                if data.get('servo_type') == 'XL330' and 'current_ma' in data:
                    xl330_holding_sum_A += abs(data['current_ma']) / 1000.0
        
        # Report peak comparison
        self.node.get_logger().info(f'  ⚡ Power Analysis for "{pose_name}":')
        self.node.get_logger().info(f'     Peak 5V current (INA219):    {peak_5v_current:.3f}A')
        if xl330_peak_sum_A > 0:
            self.node.get_logger().info(f'     Peak motor sum (XL330s):     {xl330_peak_sum_A:.3f}A')
            benefit = xl330_peak_sum_A - peak_5v_current
            if benefit > 0.05:  # Controller saving >50mA
                percent = (benefit / xl330_peak_sum_A) * 100
                self.node.get_logger().info(f'     Controller benefit:          {benefit:.3f}A ({percent:.0f}% reduction)')
            elif benefit < -0.05:  # Supply using >50mA more than expected
                self.node.get_logger().warn(f'     ⚠ Supply higher than motor sum by {abs(benefit):.3f}A!')
        self.node.get_logger().info(f'     5V voltage: {min_5v_voltage:.2f}V (min) / {peak_5v_voltage:.2f}V (max)')
        self.node.get_logger().info(f'     Peak 12V current:            {peak_12v_current:.3f}A')
        self.node.get_logger().info(f'     Peak total power:            {peak_total_power:.2f}W')
        
        # Compare final holding currents (INA219 vs Servo)
        if len(self.final_samples) > 0 and servo_telemetry:
            # Average the last few INA219 samples
            avg_final_5v = sum(s['current_5v'] for s in self.final_samples) / len(self.final_samples)
            avg_final_12v = sum(s['current_12v'] for s in self.final_samples) / len(self.final_samples)
            
            if xl330_holding_sum_A > 0:
                self.node.get_logger().info(f'     Holding current comparison:')
                self.node.get_logger().info(f'       Supply current (INA219):     {avg_final_5v:.3f}A')
                self.node.get_logger().info(f'       Motor current sum (XL330s):  {xl330_holding_sum_A:.3f}A')
                benefit = xl330_holding_sum_A - avg_final_5v
                if benefit > 0.05:  # Controller saving >50mA
                    percent = (benefit / xl330_holding_sum_A) * 100
                    self.node.get_logger().info(f'       Controller benefit:          {benefit:.3f}A ({percent:.0f}% efficiency)')
                elif benefit < -0.05:  # Supply using >50mA more than expected
                    self.node.get_logger().warn(f'       ⚠ Supply higher than motor sum by {abs(benefit):.3f}A!')
                else:
                    self.node.get_logger().info(f'       ✓ Supply matches motor sum')



class JointStateTelemetry:
    """
    Dual-topic telemetry reader for Dynamixel servos.
    
    Handles XL430 vs XL330 differences:
    - XL430 (IDs 1, 2): Has Load (%), NO Current
    - XL330 (IDs 3-6): Has Current (mA)
    
    Subscribes to:
    - /joint_states for position, velocity, effort
    - /dynamixel_hardware_interface/dxl_state for temperature, current, voltage
    """
    
    # Servo types by ID
    XL430_IDS = [1, 2]  # shoulder_pan, shoulder_lift
    XL330_IDS = [3, 4, 5, 6]  # elbow, wrists, gripper
    
    def __init__(self, node, joint_names, servo_id_map):
        """
        Initialize telemetry reader.
        
        Args:
            node: ROS2 node
            joint_names: List of joint names
            servo_id_map: Dict mapping joint name -> servo ID
                         {'shoulder_pan': 1, 'shoulder_lift': 2, ...}
        """
        self.node = node
        self.joint_names = joint_names
        self.servo_id_map = servo_id_map
        self.enabled = False
        
        self.latest_joint_states = None
        self.latest_dxl_states = None
        
        # Peak tracking during poses
        self.track_peaks = False
        self.peak_telemetry = {}
        
        # Subscribe to both topics
        self.joint_state_sub = self.node.create_subscription(
            JointState, '/joint_states',
            self._joint_state_callback, 10
        )
        
        self.dxl_state_sub = self.node.create_subscription(
            DynamixelState, '/dynamixel_hardware_interface/dxl_state',
            self._dxl_state_callback, 10
        )
        
        self.node.get_logger().info("✓ Telemetry initialized (dual-topic mode)")
        self.node.get_logger().info("  - Subscribing to /joint_states")
        self.node.get_logger().info("  - Subscribing to /dxl_state")
        self.node.get_logger().info("  - XL430 servos (IDs 1,2): Load only")
        self.node.get_logger().info("  - XL330 servos (IDs 3-6): Current + Load")
        
        self._wait_for_data()
    
    def _wait_for_data(self, timeout=2.0):
        """Wait for first data."""
        start = time.time()
        while (time.time() - start) < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.latest_joint_states is not None:
                self.enabled = True
                self.node.get_logger().info("✓ Telemetry active")
                return
            time.sleep(0.01)
        
        if self.latest_joint_states:
            self.enabled = True
        else:
            self.node.get_logger().warn("⚠️  No telemetry data yet")
    
    def _joint_state_callback(self, msg):
        self.latest_joint_states = msg
        if not self.enabled:
            self.enabled = True
    
    def _dxl_state_callback(self, msg):
        self.latest_dxl_states = msg
        
        # Track peaks if enabled
        if self.track_peaks:
            self._update_peaks(msg)
    
    def _update_peaks(self, dxl_msg):
        """Update peak values from dynamixel state."""
        for servo_id in dxl_msg.id:
            idx = list(dxl_msg.id).index(servo_id)
            
            # Find joint name
            joint_name = None
            for name, sid in self.servo_id_map.items():
                if sid == servo_id:
                    joint_name = name
                    break
            
            if not joint_name:
                continue
            
            # Initialize peak tracking for this joint
            if joint_name not in self.peak_telemetry:
                self.peak_telemetry[joint_name] = {
                    'servo_id': servo_id,
                    'servo_type': 'XL430' if servo_id in self.XL430_IDS else 'XL330'
                }
            
            # Track peak current (XL330 only)
            if servo_id in self.XL330_IDS:
                if hasattr(dxl_msg, 'present_current') and idx < len(dxl_msg.present_current):
                    current = dxl_msg.present_current[idx]
                    if 'current_ma' not in self.peak_telemetry[joint_name]:
                        self.peak_telemetry[joint_name]['current_ma'] = current
                    else:
                        # Track highest absolute value
                        if abs(current) > abs(self.peak_telemetry[joint_name]['current_ma']):
                            self.peak_telemetry[joint_name]['current_ma'] = current
            
            # Track peak load (XL430 only)
            if servo_id in self.XL430_IDS:
                if hasattr(dxl_msg, 'present_load') and idx < len(dxl_msg.present_load):
                    load = dxl_msg.present_load[idx] / 10.0  # Convert to %
                    if 'load_percent' not in self.peak_telemetry[joint_name]:
                        self.peak_telemetry[joint_name]['load_percent'] = load
                    else:
                        # Track highest absolute value
                        if abs(load) > abs(self.peak_telemetry[joint_name]['load_percent']):
                            self.peak_telemetry[joint_name]['load_percent'] = load
    
    def start_peak_tracking(self):
        """Start tracking peak values."""
        self.track_peaks = True
        self.peak_telemetry = {}
    
    def get_peak_telemetry(self):
        """Get peak telemetry collected during tracking."""
        return self.peak_telemetry.copy()
    
    def get_joint_telemetry(self, joint_name):
        """
        Get telemetry for a joint.
        
        Returns dict with:
        - For XL430 servos (IDs 1, 2):
            position, velocity, effort, load_percent, temperature, voltage
        - For XL330 servos (IDs 3-6):
            position, velocity, effort, current_ma, load_percent, temperature, voltage
        """
        if not self.enabled or not self.latest_joint_states:
            return None
        
        result = {'joint': joint_name}
        servo_id = self.servo_id_map.get(joint_name)
        
        if servo_id:
            result['servo_id'] = servo_id
            result['servo_type'] = 'XL430' if servo_id in self.XL430_IDS else 'XL330'
        
        # Get standard joint state data
        try:
            if joint_name in self.latest_joint_states.name:
                idx = self.latest_joint_states.name.index(joint_name)
                
                if idx < len(self.latest_joint_states.position):
                    result['position'] = self.latest_joint_states.position[idx]
                
                if idx < len(self.latest_joint_states.velocity):
                    result['velocity'] = self.latest_joint_states.velocity[idx]
                
                # Keep effort for reference but don't use it as load
                if idx < len(self.latest_joint_states.effort):
                    effort = self.latest_joint_states.effort[idx]
                    result['effort'] = effort  # Torque in Nm, NOT percentage
                    
        except Exception as e:
            self.node.get_logger().error(f"Error reading joint_states: {e}")
        
        # Get Dynamixel-specific data
        if servo_id and self.latest_dxl_states:
            try:
                if servo_id in self.latest_dxl_states.id:
                    idx = list(self.latest_dxl_states.id).index(servo_id)
                    
                    # Temperature (both servo types) - already in °C
                    if hasattr(self.latest_dxl_states, 'temperature'):
                        if idx < len(self.latest_dxl_states.temperature):
                            result['temperature'] = \
                                self.latest_dxl_states.temperature[idx]
                    
                    # Voltage (both servo types) - convert from 0.1V units to V
                    if hasattr(self.latest_dxl_states, 'voltage'):
                        if idx < len(self.latest_dxl_states.voltage):
                            result['voltage'] = \
                                self.latest_dxl_states.voltage[idx] / 10.0
                    
                    # Load (XL430 only) - convert from 0.1% units to %
                    if servo_id in self.XL430_IDS:
                        if hasattr(self.latest_dxl_states, 'present_load'):
                            if idx < len(self.latest_dxl_states.present_load):
                                result['load_percent'] = \
                                    self.latest_dxl_states.present_load[idx] / 10.0
                    
                    # Current (XL330 only) - already in mA
                    if servo_id in self.XL330_IDS:
                        if hasattr(self.latest_dxl_states, 'present_current'):
                            if idx < len(self.latest_dxl_states.present_current):
                                result['current_ma'] = \
                                    self.latest_dxl_states.present_current[idx]
                    
                    # Hardware error
                    if idx < len(self.latest_dxl_states.dxl_hw_state):
                        result['hw_error'] = \
                            self.latest_dxl_states.dxl_hw_state[idx]
                    
            except Exception as e:
                self.node.get_logger().error(f"Error reading dxl_state: {e}")
        
        return result if len(result) > 1 else None
    
    def get_all_joints_telemetry(self):
        """Get telemetry for all joints."""
        results = {}
        for joint_name in self.joint_names:
            data = self.get_joint_telemetry(joint_name)
            if data:
                results[joint_name] = data
        return results
    
    def print_telemetry(self, joint_name=None):
        """Pretty print telemetry."""
        if joint_name:
            data = self.get_joint_telemetry(joint_name)
            if data:
                self._print_single(data)
        else:
            all_data = self.get_all_joints_telemetry()
            if all_data:
                self.node.get_logger().info("="*75)
                self.node.get_logger().info("📊 SERVO TELEMETRY")
                self.node.get_logger().info("="*75)
                for _, data in all_data.items():
                    self._print_single(data)
                self.node.get_logger().info("="*75)
    
    def _print_single(self, data):
        """Format single joint telemetry."""
        line = f"{data['joint']:<15}"
        
        if 'servo_id' in data:
            line += f" ID:{data['servo_id']}"
        
        if 'servo_type' in data:
            line += f"({data['servo_type']:<5})"
        
        if 'position' in data:
            line += f" Pos:{data['position']:>7.3f}rad"
        
        if 'load_percent' in data:
            line += f" Load:{data['load_percent']:>5.1f}%"
        
        if 'current_ma' in data:
            line += f" Curr:{data['current_ma']:>5.0f}mA"
        
        if 'voltage' in data:
            line += f" Volt:{data['voltage']:>4.1f}V"
        
        if 'temperature' in data:
            temp = data['temperature']
            temp_str = f"Temp:{temp:>5.1f}°C"
            if temp > 60:
                temp_str += " 🔥"
            elif temp > 50:
                temp_str += " ⚠️"
            line += f" {temp_str}"
        
        self.node.get_logger().info(line)
    
    def get_max_temperature(self):
        """Get max temperature across all servos."""
        all_data = self.get_all_joints_telemetry()
        temps = [d['temperature'] for d in all_data.values() 
                 if 'temperature' in d]
        return max(temps) if temps else None
    
    def get_total_current(self):
        """
        Get total current draw (XL330 servos only).
        XL430 servos don't measure current!
        """
        all_data = self.get_all_joints_telemetry()
        currents = [d['current_ma'] for d in all_data.values() 
                    if 'current_ma' in d]
        return sum(currents) if currents else None
    
    def check_thermal_limits(self, warning_temp=55.0, critical_temp=65.0):
        """Check thermal status."""
        all_data = self.get_all_joints_telemetry()
        
        max_temp = 0.0
        hot_joints = []
        status = 'ok'
        
        for joint_name, data in all_data.items():
            if 'temperature' in data:
                temp = data['temperature']
                max_temp = max(max_temp, temp)
                
                if temp >= critical_temp:
                    status = 'critical'
                    hot_joints.append((joint_name, temp))
                elif temp >= warning_temp:
                    if status != 'critical':
                        status = 'warning'
                    hot_joints.append((joint_name, temp))
        
        return {
            'status': status,
            'max_temp': max_temp,
            'hot_joints': hot_joints
        }
    
    def check_overload(self, xl430_warning=80.0, xl330_current_warning=400.0):
        """
        Check for overload conditions.
        
        Args:
            xl430_warning: Load percentage threshold for XL430 servos
            xl330_current_warning: Current (mA) threshold for XL330 servos
        """
        all_data = self.get_all_joints_telemetry()
        overloaded = []
        
        for joint_name, data in all_data.items():
            servo_id = data.get('servo_id')
            
            # Check XL430 load
            if servo_id in self.XL430_IDS:
                if 'load_percent' in data and abs(data['load_percent']) >= xl430_warning:
                    overloaded.append((joint_name, 'Load', data['load_percent']))
            
            # Check XL330 current
            elif servo_id in self.XL330_IDS:
                if 'current_ma' in data and abs(data['current_ma']) >= xl330_current_warning:
                    overloaded.append((joint_name, 'Current', data['current_ma']))
        
        return overloaded
    
    def print_summary(self):
        """Print comprehensive summary."""
        all_data = self.get_all_joints_telemetry()
        
        if not all_data:
            self.node.get_logger().warn("No telemetry data available")
            return
        
        self.node.get_logger().info("\n" + "="*75)
        self.node.get_logger().info("📊 TELEMETRY SUMMARY")
        self.node.get_logger().info("="*75)
        
        # XL430 Summary
        xl430_loads = [d['load_percent'] for d in all_data.values()
                       if d.get('servo_id') in self.XL430_IDS and 'load_percent' in d]
        if xl430_loads:
            self.node.get_logger().info(
                f"XL430 (IDs 1-2): Avg Load: {sum(xl430_loads)/len(xl430_loads):.1f}%"
            )
        
        # XL330 Summary
        xl330_currents = [d['current_ma'] for d in all_data.values()
                          if d.get('servo_id') in self.XL330_IDS and 'current_ma' in d]
        if xl330_currents:
            total = sum(xl330_currents)
            avg = total / len(xl330_currents)
            self.node.get_logger().info(
                f"XL330 (IDs 3-6): Total Current: {total:.0f}mA, "
                f"Avg: {avg:.0f}mA ({total*12/1000:.1f}W @ 12V)"
            )
        
        # Temperature
        max_temp = self.get_max_temperature()
        if max_temp:
            self.node.get_logger().info(f"Max Temperature: {max_temp:.1f}°C")
        
        # Voltage
        voltages = [d['voltage'] for d in all_data.values() if 'voltage' in d]
        if voltages:
            min_volt = min(voltages)
            max_volt = max(voltages)
            avg_volt = sum(voltages) / len(voltages)
            self.node.get_logger().info(
                f"Voltage: {min_volt:.2f}V - {max_volt:.2f}V (avg {avg_volt:.2f}V)"
            )
            if min_volt < 4.5:
                self.node.get_logger().warn(f"⚠️  LOW VOLTAGE: {min_volt:.2f}V")
        
        # Thermal check
        thermal = self.check_thermal_limits()
        if thermal['status'] != 'ok':
            self.node.get_logger().warn(f"⚠️  Thermal Status: {thermal['status'].upper()}")
            for joint, temp in thermal['hot_joints']:
                self.node.get_logger().warn(f"   {joint}: {temp:.1f}°C")
        
        # Overload check
        overloaded = self.check_overload()
        if overloaded:
            self.node.get_logger().warn("⚠️  Overloaded servos:")
            for joint, metric, value in overloaded:
                self.node.get_logger().warn(f"   {joint}: {metric} = {value:.1f}")
        
        self.node.get_logger().info("="*75)
    
    def close(self):
        """Cleanup."""
        pass


class PoseTestNode(Node):
    """ROS2 node for pose testing with validation."""
    
    def __init__(self):
        super().__init__('pose_test_node')
        
        # Declare parameters
        self.declare_parameter('poses_file', '')
        self.declare_parameter('program', '')  # calibration, sweep, stress, default
        self.declare_parameter('servo_test', False)
        self.declare_parameter('servo_test_joint', '')  # Specific joint to test
        self.declare_parameter('safe_position_first', True)
        self.declare_parameter('validate', True)
        self.declare_parameter('telemetry', False)
        self.declare_parameter('delay', 2.0)
        self.declare_parameter('movement_time', 2.0)
        self.declare_parameter('urdf_file', '')  # NEW: Load URDF from file
        self.declare_parameter('power_monitoring', False)  # NEW: Power monitoring
        
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
        urdf_file = self.get_parameter('urdf_file').value
        self.power_monitoring_requested = self.get_parameter('power_monitoring').value
        
        # Joint names (Koch v1.1 - 6 DOF)
        self.joint_names = [
            'shoulder_pan',
            'shoulder_lift', 
            'elbow_flex',
            'wrist_flex',
            'wrist_roll',
            'pen_holder'
        ]
        
        # Servo ID mapping for telemetry
        self.servo_id_map = {
            'shoulder_pan': 1,   # XL430-W250
            'shoulder_lift': 2,  # XL430-W250
            'elbow_flex': 3,     # XL330-M288
            'wrist_flex': 4,     # XL330-M288
            'wrist_roll': 5,     # XL330-M288
            'pen_holder': 6      # XL330-M077
        }
        
        # Create publisher
        self.publisher = self.create_publisher(
            JointTrajectory,
            '/koch_v11_controller/joint_trajectory',
            10
        )
        
        # Safe starting position
        self.safe_position = {
            'shoulder_pan': 1.292,
            'shoulder_lift': 2.688,
            'elbow_flex': 1.063,
            'wrist_flex': 2.499,
            'wrist_roll': -0.014,
            'pen_holder': 0.895
        }
        self.safe_position = {
            'shoulder_pan': 1.55,
            'shoulder_lift': 2.7,    # Near vertical (safe)
            'elbow_flex': 1.2,       # Moderately bent
            'wrist_flex': 1.5,       # Neutral
            'wrist_roll': 0.0,       # Centered
            'pen_holder': 0.9        # Slightly open
        }
        
        # Load URDF limits
        self.get_logger().info('Loading URDF limits...')
        urdf_content = self.get_urdf_content(urdf_file)
        if urdf_content:
            self.urdf_parser = URDFLimitsParser(urdf_content)
            self.joint_limits = self.urdf_parser.get_joint_limits()
            self.velocity_limits = self.urdf_parser.get_velocity_limits()
            self.get_logger().info(f'✓ Loaded limits for {len(self.joint_limits)} joints')
        else:
            self.get_logger().error('Failed to load URDF - validation disabled')
            self.joint_limits = {}
            self.velocity_limits = {}
            self.validate = False
        
        # Initialize telemetry with servo ID mapping
        self.telemetry = None
        if self.enable_telemetry:
            self.get_logger().info('Initializing telemetry...')
            try:
                self.telemetry = JointStateTelemetry(
                    self, 
                    self.joint_names,
                    self.servo_id_map  # Pass servo ID mapping
                )
            except Exception as e:
                self.get_logger().error(f'Telemetry init failed: {e}')
                self.telemetry = None
        
        # Power monitoring (optional)
        self.power_monitor = None
        if self.power_monitoring_requested:
            self.power_monitor = PowerMonitor(self)
        
        # Statistics
        self.stats = {
            'total_poses': 0,
            'validation_errors': 0,
            'validation_warnings': 0
        }
        
        self.get_logger().info('='*60)
        self.get_logger().info('POSE TEST NODE INITIALIZED')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Joints: {len(self.joint_names)}')
        self.get_logger().info(f'Validation: {self.validate}')
        self.get_logger().info(f'Telemetry: {self.enable_telemetry}')
        if self.power_monitor and self.power_monitor.enabled:
            self.get_logger().info(f'Power Monitoring: Enabled')
        self.get_logger().info(f'Movement time: {self.movement_time}s')
        self.get_logger().info(f'Delay between poses: {self.delay}s')
        self.get_logger().info('='*60)
    
    def get_urdf_content(self, urdf_file=''):
        """Get URDF content from parameter server or file."""
        # Try from file first
        if urdf_file:
            try:
                with open(urdf_file, 'r') as f:
                    return f.read()
            except Exception as e:
                self.get_logger().error(f'Error reading URDF file {urdf_file}: {e}')
        
        # Try from parameter server
        try:
            urdf_param = self.get_parameter('robot_description')
            return urdf_param.value
        except Exception:
            self.get_logger().warn('robot_description parameter not found')
        
        # Try to declare and get it
        try:
            self.declare_parameter('robot_description', '')
            urdf_param = self.get_parameter('robot_description')
            if urdf_param.value:
                return urdf_param.value
        except Exception as e:
            self.get_logger().error(f'Could not get URDF: {e}')
        
        return None
    
    def spin_for_duration(self, duration):
        """Spin node for specified duration (allows callbacks to process)."""
        start = time.time()
        while (time.time() - start) < duration:
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.01)
    
    def validate_pose(self, pose_name, positions):
        """Validate pose against URDF limits."""
        if not self.validate or not self.joint_limits:
            return True
        
        valid = True
        
        for joint_name, position in positions.items():
            if joint_name not in self.joint_limits:
                self.get_logger().warn(
                    f'  ⚠️  {joint_name}: No limits found in URDF'
                )
                self.stats['validation_warnings'] += 1
                continue
            
            limits = self.joint_limits[joint_name]
            min_pos = limits['min']
            max_pos = limits['max']
            
            if position < min_pos or position > max_pos:
                self.get_logger().error(
                    f'  ❌ {joint_name}: {position:.3f} rad OUT OF RANGE '
                    f'[{min_pos:.3f}, {max_pos:.3f}]'
                )
                self.stats['validation_errors'] += 1
                valid = False
            else:
                self.get_logger().info(
                    f'  ✓ {joint_name}: {position:.3f} rad '
                    f'(within [{min_pos:.3f}, {max_pos:.3f}])'
                )
        
        return valid
    
    def send_pose(self, pose_name, positions, duration=2.0):
        """Send pose to joint trajectory controller."""
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = [positions.get(j, 0.0) for j in self.joint_names]
        point.time_from_start = Duration(sec=int(duration), nanosec=int((duration % 1) * 1e9))
        
        msg.points = [point]
        
        self.publisher.publish(msg)
        self.get_logger().info(f'→ Sent: {pose_name}')
    
    def move_to_safe_position(self):
        """Move to safe starting position."""
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('MOVING TO SAFE POSITION')
        self.get_logger().info('='*60)
        
        if self.validate:
            valid = self.validate_pose('safe_position', self.safe_position)
            if not valid:
                self.get_logger().error('Safe position validation failed!')
                return False
        
        self.send_pose('safe_position', self.safe_position, self.movement_time)
        self.spin_for_duration(self.movement_time + 1.0)
        
        self.get_logger().info('✓ At safe position\n')
        return True
    
    def servo_test_single_joint(self, joint_name):
        """Test a single joint through its range."""
        if joint_name not in self.joint_names:
            self.get_logger().error(f'Unknown joint: {joint_name}')
            return False
        
        if joint_name not in self.joint_limits:
            self.get_logger().error(f'No limits for {joint_name}')
            return False
        
        limits = self.joint_limits[joint_name]
        min_pos = limits['min']
        max_pos = limits['max']
        mid_pos = (min_pos + max_pos) / 2.0
        
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info(f'SERVO TEST: {joint_name.upper()}')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Range: [{min_pos:.3f}, {max_pos:.3f}] rad')
        self.get_logger().info(f'Middle: {mid_pos:.3f} rad')
        self.get_logger().info('='*60)
        
        # Test positions: min, middle, max
        test_positions = [
            ('min', min_pos),
            ('middle', mid_pos),
            ('max', max_pos),
        ]
        
        for test_name, test_value in test_positions:
            # Create pose with safe position + test joint
            test_pose = self.safe_position.copy()
            test_pose[joint_name] = test_value
            
            self.get_logger().info(f'\nTesting {test_name}: {test_value:.3f} rad')
            
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
                    self._print_joint_telemetry(data)
        
        self.get_logger().info(f'\n✓ Servo test complete for {joint_name}')
        return True
    
    def _print_joint_telemetry(self, data):
        """Print telemetry for a single joint (helper method)."""
        telemetry_str = f"  📊 {data['joint']:<15} "
        
        if 'servo_type' in data:
            telemetry_str += f"[{data['servo_type']}] "
        
        if 'position' in data:
            telemetry_str += f"Pos:{data['position']:>7.3f}rad "
        
        if 'load_percent' in data:
            telemetry_str += f"Load:{data['load_percent']:>6.2f}% "
        
        if 'current_ma' in data:
            telemetry_str += f"Curr:{data['current_ma']:>5.0f}mA "
        
        if 'voltage' in data:
            telemetry_str += f"Volt:{data['voltage']:>4.1f}V "
        
        if 'temperature' in data:
            temp = data['temperature']
            telemetry_str += f"Temp:{temp:>5.1f}°C"
            if temp > 60:
                telemetry_str += " 🔥"
            elif temp > 50:
                telemetry_str += " ⚠️"
        
        self.get_logger().info(telemetry_str)
    
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
        
        # Print telemetry summary if enabled
        if self.telemetry and self.telemetry.enabled:
            self.telemetry.print_summary()
        
        return True
    
    def load_poses_from_file(self, filename):
        """Load poses from YAML file."""
        try:
            with open(filename, 'r') as f:
                data = yaml.safe_load(f)
                # Extract the 'poses' list from the YAML structure
                if isinstance(data, dict) and 'poses' in data:
                    return data['poses']
                elif isinstance(data, list):
                    # Already a list of poses (old format)
                    return data
                else:
                    self.get_logger().error(f'Invalid YAML structure in {filename}')
                    return None
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
        self.get_logger().info(f'Telemetry: {self.enable_telemetry}')
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
            
            # Start power tracking
            if self.power_monitor:
                self.power_monitor.start_tracking(pose_name, idx)
            
            # Start peak servo tracking
            if self.telemetry and self.telemetry.enabled:
                self.telemetry.start_peak_tracking()
            
            self.send_pose(pose_name, positions, self.movement_time)
            
            # Wait for movement + delay (spin to allow callbacks)
            self.spin_for_duration(self.movement_time + self.delay)
            
            # Stop power tracking
            if self.power_monitor:
                self.power_monitor.stop_tracking(pose_name, idx)
            
            # Read telemetry if available
            all_data = None
            peak_data = None
            if self.telemetry and self.telemetry.enabled:
                all_data = self.telemetry.get_all_joints_telemetry()
                peak_data = self.telemetry.get_peak_telemetry()
                if all_data:
                    self.get_logger().info("  📊 Joint States:")
                    for joint_name, data in all_data.items():
                        if joint_name in positions_dict:  # Only show joints that moved
                            self._print_joint_telemetry(data)
                    
                    # Check thermal status
                    thermal = self.telemetry.check_thermal_limits()
                    if thermal['status'] == 'critical':
                        self.get_logger().error(
                            f"  🔥 CRITICAL TEMPERATURE: {thermal['max_temp']:.1f}°C"
                        )
                        for joint, temp in thermal['hot_joints']:
                            self.get_logger().error(f"     {joint}: {temp:.1f}°C")
                        self.get_logger().error("  ABORTING FOR SAFETY!")
                        return False
                    elif thermal['status'] == 'warning':
                        self.get_logger().warn(
                            f"  ⚠️  High temperature: {thermal['max_temp']:.1f}°C"
                        )
            
            # Analyze power draw
            if self.power_monitor:
                self.power_monitor.analyze_and_report(pose_name, all_data, peak_data)
            
            self.stats['total_poses'] += 1
        
        self.print_summary()
        
        # Print final telemetry summary if enabled
        if self.telemetry and self.telemetry.enabled:
            self.telemetry.print_summary()
        
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
