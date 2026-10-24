#!/usr/bin/env python3
"""
ROS2 Pose Testing Node with Validation and Servo Testing
Version 17 - Updated with optional power monitoring

Features:
- ROS2 node publishing JointTrajectory messages
- Reads joint limits from URDF parameter
- Servo test mode (safe position + test min/max/middle per joint)
- Full pose sequence testing
- Dual-topic telemetry (joint_states + servo state: DynamixelState or FeetechState)
- Arm selected by arm profile (koch_v1_1, so_arm101): joints, servo IDs,
  controller, safe position, thresholds and pose programs come from the profile
- Temperature, current, voltage monitoring
- Thermal and overload detection
- Validation with URDF limits
- **NEW**: Optional power monitoring with INA226 sensors (primary), INA219 fallback

Usage (add -p arm_profile:=so_arm101 for the SO-ARM101; default koch_v1_1):
  ros2 run robot_arm_control pose_test --ros-args -p poses_file:=poses.yaml
  ros2 run robot_arm_control pose_test --ros-args -p servo_test:=true
  ros2 run robot_arm_control pose_test --ros-args -p servo_test:=true -p telemetry:=true
  ros2 run robot_arm_control pose_test --ros-args -p servo_test_joint:=shoulder_lift
  ros2 run robot_arm_control pose_test --ros-args -p program:=default -p power_monitoring:=true
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
from std_msgs.msg import String as StringMsg
import yaml
import time
import xml.etree.ElementTree as ET
from pathlib import Path

from robot_arm_control.arm_profile import load_arm_profile
from robot_arm_control.servo_telemetry import ServoTelemetry

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
            # Use INA226 fields as primary; fall back to INA219 when INA226 fields are zero
            ina226_12v_active = msg.bus_12v_voltage_226 > 0.0
            ina226_5v_active  = msg.bus_5v_voltage_226  > 0.0
            self.power_samples.append({
                'timestamp':   time.time(),
                'bus_12v':     msg.bus_12v_voltage_226  if ina226_12v_active else msg.bus_12v_voltage,
                'current_12v': msg.current_12v_226      if ina226_12v_active else msg.current_12v,
                'power_12v':   msg.power_12v_226        if ina226_12v_active else msg.power_12v,
                'bus_5v':      msg.bus_5v_voltage_226   if ina226_5v_active  else msg.bus_5v_voltage,
                'current_5v':  msg.current_5v_226       if ina226_5v_active  else msg.current_5v,
                'power_5v':    msg.power_5v_226         if ina226_5v_active  else msg.power_5v,
                'total_power': msg.total_power,
                'source_12v':  'INA226' if ina226_12v_active else 'INA219',
                'source_5v':   'INA226' if ina226_5v_active  else 'INA219',
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
    
    # Profile rail name -> INA sensor channel (sample key suffix)
    RAIL_CHANNELS = {'5v': '5v', '12v': '12v'}

    @staticmethod
    def _rail_current_sum_A(telemetry, rail):
        """Sum of |current| (A) of the servos on 'rail' that report current."""
        if not telemetry:
            return 0.0
        return sum(abs(d['current_ma']) / 1000.0 for d in telemetry.values()
                   if d.get('rail') == rail and 'current_ma' in d)

    def _compare(self, label, supply_A, motor_A, indent, what):
        """Log supply current vs summed servo current for one rail."""
        log = self.node.get_logger()
        benefit = motor_A - supply_A
        if benefit > 0.05:  # Controller saving >50mA
            percent = (benefit / motor_A) * 100
            log.info(f'{indent}Controller benefit:          {benefit:.3f}A ({percent:.0f}% {what})')
        elif benefit < -0.05:  # Supply using >50mA more than expected
            log.warn(f'{indent}⚠ {label} supply higher than motor sum by {abs(benefit):.3f}A!')
        elif what == 'efficiency':
            log.info(f'{indent}✓ Supply within 50mA of  motor sum')

    def analyze_and_report(self, pose_name, servo_telemetry, peak_telemetry=None):
        """Analyze power draw and compare with servo telemetry, per supply rail."""
        if not self.enabled or len(self.power_samples) == 0:
            return

        log = self.node.get_logger()
        # Rails that have current-reporting servos and a matching INA channel
        rails = [r for r in dict.fromkeys(j.rail for j in self.node.profile.joints_reporting('present_current'))
                 if r in self.RAIL_CHANNELS]

        peak_5v_voltage = max(s['bus_5v'] for s in self.power_samples)
        min_5v_voltage = min(s['bus_5v'] for s in self.power_samples)
        peak_total_power = max(s['total_power'] for s in self.power_samples)
        source = {ch: self.power_samples[-1][f'source_{ch}'] for ch in ('5v', '12v')}

        log.info(f'  ⚡ Power Analysis for "{pose_name}":')
        for ch in ('5v', '12v'):
            peak_supply = max(s[f'current_{ch}'] for s in self.power_samples)
            log.info(f'     Peak {ch.upper()} current ({source[ch]}): {peak_supply:.3f}A')
            if ch in rails:
                motor_peak = self._rail_current_sum_A(peak_telemetry, ch)
                if motor_peak > 0:
                    log.info(f'     Peak motor sum ({ch} servos):  {motor_peak:.3f}A')
                    self._compare(ch.upper(), peak_supply, motor_peak, '     ', 'reduction')
        log.info(f'     5V voltage: {min_5v_voltage:.2f}V (min) / {peak_5v_voltage:.2f}V (max)')
        log.info(f'     Peak total power:            {peak_total_power:.2f}W')

        # Compare final holding currents (power sensor vs servo-reported)
        if len(self.final_samples) > 0 and servo_telemetry:
            for ch in rails:
                motor_hold = self._rail_current_sum_A(servo_telemetry, ch)
                if motor_hold <= 0:
                    continue
                # Average the last few power sensor samples (INA226 or INA219)
                avg_supply = sum(s[f'current_{ch}'] for s in self.final_samples) / len(self.final_samples)
                log.info(f'     Holding current comparison ({ch} rail):')
                log.info(f'       Supply current ({source[ch]}):  {avg_supply:.3f}A')
                log.info(f'       Motor current sum:           {motor_hold:.3f}A')
                self._compare(ch.upper(), avg_supply, motor_hold, '       ', 'efficiency')



class JointStateTelemetry:
    """
    Dual-topic telemetry reader, vendor-neutral.

    Subscribes to:
    - /joint_states for position, velocity, effort
    - the arm profile's servo telemetry topic (DynamixelState or FeetechState,
      via ServoTelemetry) for temperature, input voltage, current, load

    Which of current/load a joint has comes from the profile ('reports'); e.g.
    Koch XL430s report load only, XL330s current only, STS3215s both.
    Telemetry dicts use display units: current_ma (mA), load_percent (%),
    temperature (degC), voltage (V).
    """

    def __init__(self, node, profile):
        """
        Initialize telemetry reader.

        Args:
            node: ROS2 node
            profile: ArmProfile (joint names, servo ids, telemetry backend, thresholds)
        """
        self.node = node
        self.profile = profile
        self.joint_names = profile.joint_names
        self.joint_specs = profile.joint_by_name
        self.enabled = False

        self.latest_joint_states = None

        # Peak tracking during poses
        self.track_peaks = False
        self.peak_telemetry = {}

        self.joint_state_sub = self.node.create_subscription(
            JointState, '/joint_states',
            self._joint_state_callback, 10
        )
        self.servo_telemetry = ServoTelemetry(self.node, profile, on_update=self._servo_update_callback)

        self.node.get_logger().info("✓ Telemetry initialized (dual-topic mode)")
        self.node.get_logger().info("  - Subscribing to /joint_states")
        self.node.get_logger().info(f"  - Subscribing to {profile.telemetry_topic}")
        for spec in profile.joints:
            extras = [f for f in ('present_current', 'present_load') if f in spec.reports]
            self.node.get_logger().info(
                f"  - {spec.name} (ID {spec.id}, {spec.model}): {', '.join(extras) or 'no current/load'}")

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

    def _servo_update_callback(self, readings):
        if self.track_peaks:
            self._update_peaks(readings)

    @staticmethod
    def _servo_values(reading):
        """ServoReading (message units) -> display-unit dict entries."""
        values = {}
        if reading.present_temperature is not None:
            values['temperature'] = reading.present_temperature          # degC
        if reading.present_input_voltage is not None:
            values['voltage'] = reading.present_input_voltage / 10.0     # 0.1 V -> V
        if reading.present_current is not None:
            values['current_ma'] = reading.present_current               # mA
        if reading.present_load is not None:
            values['load_percent'] = reading.present_load / 10.0         # 0.1 % -> %
        values['hw_error'] = reading.hw_state
        return values

    def _update_peaks(self, readings):
        """Update peak current/load per joint (highest absolute value)."""
        for joint_name, reading in readings.items():
            spec = self.joint_specs[joint_name]
            peak = self.peak_telemetry.setdefault(
                joint_name, {'servo_id': spec.id, 'servo_type': spec.model, 'rail': spec.rail})
            values = self._servo_values(reading)
            for key in ('current_ma', 'load_percent'):
                if key in values and (key not in peak or abs(values[key]) > abs(peak[key])):
                    peak[key] = values[key]

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

        Returns dict with joint, servo_id, servo_type (model), rail, position,
        velocity, effort, temperature, voltage, hw_error, and current_ma and/or
        load_percent depending on what the servo reports.
        """
        if not self.enabled or not self.latest_joint_states:
            return None

        result = {'joint': joint_name}
        spec = self.joint_specs.get(joint_name)
        if spec:
            result['servo_id'] = spec.id
            result['servo_type'] = spec.model
            result['rail'] = spec.rail

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
                    result['effort'] = self.latest_joint_states.effort[idx]

        except Exception as e:
            self.node.get_logger().error(f"Error reading joint_states: {e}")

        reading = self.servo_telemetry.readings.get(joint_name)
        if reading is not None:
            result.update(self._servo_values(reading))

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
            line += f"({data['servo_type']:<10})"

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
        """Get total current draw of the servos that report current."""
        all_data = self.get_all_joints_telemetry()
        currents = [d['current_ma'] for d in all_data.values()
                    if 'current_ma' in d]
        return sum(currents) if currents else None

    def check_thermal_limits(self, warning_temp=None, critical_temp=None):
        """Check thermal status. Limits default to the profile's thresholds."""
        t = self.profile.thresholds
        warning_temp = t.temperature_warning_c if warning_temp is None else warning_temp
        critical_temp = t.temperature_critical_c if critical_temp is None else critical_temp
        all_data = self.get_all_joints_telemetry()

        max_temp = 0.0
        hot_joints = []
        status = 'ok'

        for joint_name, data in all_data.items():
            if 'temperature' in data:
                temp = data['temperature']
                max_temp = max(max_temp, temp)

                if critical_temp is not None and temp >= critical_temp:
                    status = 'critical'
                    hot_joints.append((joint_name, temp))
                elif warning_temp is not None and temp >= warning_temp:
                    if status != 'critical':
                        status = 'warning'
                    hot_joints.append((joint_name, temp))

        return {
            'status': status,
            'max_temp': max_temp,
            'hot_joints': hot_joints
        }

    def check_overload(self, load_warning=None, current_warning=None):
        """
        Check for overload conditions. Limits default to the profile's
        thresholds (load_warning_pct, current_warning_ma); a None limit is skipped.
        """
        t = self.profile.thresholds
        load_warning = t.load_warning_pct if load_warning is None else load_warning
        current_warning = t.current_warning_ma if current_warning is None else current_warning
        all_data = self.get_all_joints_telemetry()
        overloaded = []

        for joint_name, data in all_data.items():
            if load_warning is not None and 'load_percent' in data and abs(data['load_percent']) >= load_warning:
                overloaded.append((joint_name, 'Load', data['load_percent']))
            if current_warning is not None and 'current_ma' in data and abs(data['current_ma']) >= current_warning:
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

        # Load summary (joints whose servo reports load)
        loads = {j: d['load_percent'] for j, d in all_data.items() if 'load_percent' in d}
        if loads:
            self.node.get_logger().info(
                f"Load ({', '.join(loads)}): Avg Load: {sum(abs(v) for v in loads.values())/len(loads):.1f}%"
            )

        # Current summary per supply rail (joints whose servo reports current)
        rails = {}
        for j, d in all_data.items():
            if 'current_ma' in d:
                rails.setdefault(d.get('rail', '?'), []).append(d)
        for rail, items in rails.items():
            total = sum(abs(d['current_ma']) for d in items)
            # Power from each servo's own measured voltage
            watts = sum(abs(d['current_ma']) / 1000.0 * d.get('voltage', 0.0) for d in items)
            self.node.get_logger().info(
                f"Current ({rail} rail, {len(items)} servos): Total: {total:.0f}mA, "
                f"Avg: {total/len(items):.0f}mA ({watts:.1f}W)"
            )

        # Temperature
        max_temp = self.get_max_temperature()
        if max_temp:
            self.node.get_logger().info(f"Max Temperature: {max_temp:.1f}°C")

        # Voltage, with per-rail low-voltage check from the profile
        voltages = [d['voltage'] for d in all_data.values() if 'voltage' in d]
        if voltages:
            min_volt = min(voltages)
            max_volt = max(voltages)
            avg_volt = sum(voltages) / len(voltages)
            self.node.get_logger().info(
                f"Voltage: {min_volt:.2f}V - {max_volt:.2f}V (avg {avg_volt:.2f}V)"
            )
        for rail, min_allowed in self.profile.rail_min_voltage.items():
            if min_allowed is None:
                continue
            rail_volts = [d['voltage'] for d in all_data.values() if d.get('rail') == rail and 'voltage' in d]
            if rail_volts and min(rail_volts) < min_allowed:
                self.node.get_logger().warn(
                    f"⚠️  LOW VOLTAGE on {rail} rail: {min(rail_volts):.2f}V (< {min_allowed:.2f}V)")

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

        # Arm profile: joints, servo IDs, controller, telemetry, safe position,
        # thresholds and pose programs for the selected arm.
        self.declare_parameter('arm_profile', 'koch_v1_1')
        self.profile = load_arm_profile(self.get_parameter('arm_profile').value)

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
        self.declare_parameter('urdf_file', '')
        self.declare_parameter('power_monitoring', False)
        self.declare_parameter('wait_for_stable', False)   # wait for /ball/is_centered before next pose
        self.declare_parameter('stable_timeout',  10.0)    # max seconds to wait for ball centering
        
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
        self.wait_for_stable = self.get_parameter('wait_for_stable').value
        self.stable_timeout  = self.get_parameter('stable_timeout').value
        
        self.joint_names = self.profile.joint_names
        self.servo_id_map = {j.name: j.id for j in self.profile.joints}

        # Create publisher
        self.publisher = self.create_publisher(
            JointTrajectory,
            self.profile.trajectory_topic,
            10
        )

        # Arm state publisher — signals imu_balance_node to enable/disable PID.
        # Publishes 'MOVING' before each pose command, 'SETTLED' after movement
        # + delay completes.  imu_balance_node subscribes to gate its PID loop.
        self._arm_state_pub = self.create_publisher(StringMsg, '/arm_state', 10)

        # Track stability signal from imu_balance_node (used when wait_for_stable=True).
        # Gracefully absent if imu_balance_node is not running.
        from std_msgs.msg import Bool as BoolMsg
        self._is_stable = False
        self.create_subscription(
            BoolMsg, '/ball/is_centered',
            lambda msg: setattr(self, '_is_stable', msg.data),
            10
        )
        
        # Safe starting position (rad, this arm's joint frame)
        self.safe_position = dict(self.profile.safe_position)

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
                self.telemetry = JointStateTelemetry(self, self.profile)
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
        self.get_logger().info(f'Arm profile: {self.profile.name} ({self.profile.path})')
        self.get_logger().info(f'Trajectory topic: {self.profile.trajectory_topic}')
        self.get_logger().info(f'Joints: {len(self.joint_names)}')
        self.get_logger().info(f'Validation: {self.validate}')
        self.get_logger().info(f'Telemetry: {self.enable_telemetry}')
        if self.power_monitor and self.power_monitor.enabled:
            self.get_logger().info(f'Power Monitoring: Enabled')
        self.get_logger().info(f'Movement time: {self.movement_time}s')
        self.get_logger().info(f'Delay between poses: {self.delay}s')
        if self.wait_for_stable:
            self.get_logger().info(
                f'Wait for stable: YES (timeout: {self.stable_timeout:.1f}s)'
            )
        self.get_logger().info('='*60)
    
    def get_urdf_content(self, urdf_file=''):
        """Get URDF content: urdf_file parameter, else robot_description
        parameter, else the arm profile's URDF (launch.urdf)."""
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
            pass  # not set on this node; declared below
        
        # Try to declare and get it
        try:
            self.declare_parameter('robot_description', '')
            urdf_param = self.get_parameter('robot_description')
            if urdf_param.value:
                return urdf_param.value
        except Exception as e:
            self.get_logger().error(f'Could not get URDF: {e}')

        # Fall back to the URDF the launch file uses for this arm
        if self.profile.urdf_path:
            try:
                with open(self.profile.urdf_path, 'r') as f:
                    self.get_logger().info(f'Using arm profile URDF: {self.profile.urdf_path}')
                    return f.read()
            except Exception as e:
                self.get_logger().error(f'Error reading profile URDF {self.profile.urdf_path}: {e}')

        return None
    
    def spin_for_duration(self, duration):
        """Spin node for specified duration (allows callbacks to process)."""
        start = time.time()
        while (time.time() - start) < duration:
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.01)

    def _publish_arm_state(self, state: str):
        """
        Publish arm state to /arm_state for imu_balance_node.
        state: 'MOVING' or 'SETTLED'
        """
        msg = StringMsg()
        msg.data = state + '\x00'
        self._arm_state_pub.publish(msg)

    def _wait_for_stable(self):
        """
        Block until /ball/is_centered is True or stable_timeout expires.

        /ball/is_centered is published True by ball_balance_node only when
        the ball has been continuously within stable_thresh for
        centered_hold_time seconds (default 2s). A single frame at center
        is not enough — the ball must genuinely settle.

        Returns immediately when True is received. On timeout, logs a warning
        and proceeds so the test sequence is not blocked indefinitely.
        Returns True if stable was achieved, False if timed out.
        """
        if not self.wait_for_stable:
            return True

        self.get_logger().info(
            f'  ⏳ Waiting for ball to center (timeout: {self.stable_timeout:.1f}s)...'
        )
        start = time.time()
        self._is_stable = False   # reset before waiting

        while (time.time() - start) < self.stable_timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._is_stable:
                elapsed = time.time() - start
                self.get_logger().info(f'  ✓ Cup stable after {elapsed:.2f}s')
                return True

        self.get_logger().warn(
            f'  ⚠️  Stability timeout after {self.stable_timeout:.1f}s — '
            f'proceeding anyway (cup may not be balanced)'
        )
        return False
    
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
        self._publish_arm_state('MOVING')
        self.spin_for_duration(self.movement_time + 1.0)
        self._publish_arm_state('SETTLED')
        
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
        """Get a built-in pose program from the arm profile ('programs')."""
        poses = self.profile.programs.get(program_name)
        if poses is None:
            available = ', '.join(sorted(self.profile.programs)) or 'none'
            self.get_logger().error(
                f"Unknown program '{program_name}' for arm profile '{self.profile.name}' (available: {available})")
            return None
        return [{'name': p['name'], 'positions': dict(p['positions'])} for p in poses]

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
            
            # Signal imu_balance_node that arm is about to move
            self._publish_arm_state('MOVING')

            # Start power tracking
            if self.power_monitor:
                self.power_monitor.start_tracking(pose_name, idx)
            
            # Start peak servo tracking
            if self.telemetry and self.telemetry.enabled:
                self.telemetry.start_peak_tracking()
            
            self.send_pose(pose_name, positions, self.movement_time)
            
            # Wait for movement + delay (spin to allow callbacks)
            self.spin_for_duration(self.movement_time + self.delay)

            # Signal imu_balance_node that arm has settled — PID may activate
            self._publish_arm_state('SETTLED')

            # Optionally wait for cup to stabilise before moving to next pose
            self._wait_for_stable()
            
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
