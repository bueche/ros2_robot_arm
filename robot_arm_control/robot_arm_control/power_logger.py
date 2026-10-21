#!/usr/bin/env python3
"""
Enhanced Power Logger with Servo Current Tracking

Logs INA226 power data (primary) and Dynamixel servo current readings.
Falls back to INA219 fields when INA226 fields are zero (legacy firmware).
Tracks peak currents during each pose period.
"""

import rclpy
from rclpy.node import Node
from koch_v1_1_msgs.msg import PowerTelemetry, PoseEvent
from sensor_msgs.msg import JointState
from dynamixel_interfaces.msg import DynamixelState
import csv
from datetime import datetime
import time


class PowerLoggerEnhanced(Node):
    def __init__(self):
        super().__init__('power_logger_enhanced')
        
        # Create filename with timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = f'power_log_{timestamp}.csv'
        
        # Open CSV file
        self.csvfile = open(self.filename, 'w', newline='')
        self.writer = csv.writer(self.csvfile)
        
        # Write header with servo current columns
        self.writer.writerow([
            'timestamp', 
            'bus_12v', 'current_12v', 'power_12v',
            'bus_5v', 'current_5v', 'power_5v',
            'total_power',
            'pose_event', 'pose_name', 'pose_number',
            # Servo currents (XL330s only report current)
            'elbow_current_ma', 'wrist_flex_current_ma', 
            'wrist_roll_current_ma', 'gripper_current_ma',
            'shoulder_pan_load', 'shoulder_lift_load',
            'shoulder_pan_temp', 'shoulder_lift_temp',
            'elbow_temp', 'wrist_flex_temp',
            'wrist_roll_temp', 'gripper_temp',
            # Per-servo input voltages (all 6, in V) — for sag tracking
            'shoulder_pan_voltage', 'shoulder_lift_voltage',
            'elbow_voltage', 'wrist_flex_voltage',
            'wrist_roll_voltage', 'gripper_voltage',
            # Sum of absolute XL330 currents (mA) — excludes XL430s which report load not current
            'total_xl330_current_ma',
        ])
        
        # Current pose tracking
        self.current_pose = ''
        self.current_pose_num = 0
        self.pose_active = False
        
        # Latest servo states
        self.latest_joint_state = None
        self.latest_dxl_state = None
        
        # Peak tracking during pose
        self.reset_peak_tracking()
        
        # Servo ID mapping for XL330s (IDs 3-6 have current)
        self.servo_id_map = {
            'shoulder_pan': 1,
            'shoulder_lift': 2,
            'elbow_flex': 3,
            'wrist_flex': 4,
            'wrist_roll': 5,
            'pen_holder': 6
        }
        
        # Subscribe to power telemetry
        self.power_sub = self.create_subscription(
            PowerTelemetry,
            'power_telemetry',
            self.telemetry_callback,
            10
        )
        
        # Subscribe to pose events
        self.pose_event_sub = self.create_subscription(
            PoseEvent,
            'pose_events',
            self.pose_event_callback,
            10
        )
        
        # Subscribe to servo telemetry
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.dxl_state_sub = self.create_subscription(
            DynamixelState,
            '/dynamixel_hardware_interface/dxl_state',
            self.dxl_state_callback,
            10
        )
        
        self.get_logger().info(f'Logging to: {self.filename}')
        self.get_logger().info('Tracking: INA226 power (primary) + Servo currents')
    
    def reset_peak_tracking(self):
        """Reset peak tracking for new pose."""
        self.peak_servo_currents = {
            'elbow_flex': 0,
            'wrist_flex': 0,
            'wrist_roll': 0,
            'pen_holder': 0
        }
        self.peak_servo_temperatures = {
            'shoulder_pan': 0,
            'shoulder_lift': 0,
            'elbow_flex': 0,
            'wrist_flex': 0,
            'wrist_roll': 0,
            'pen_holder': 0
        }
        self.peak_servo_loads = {
            'shoulder_pan': 0,
            'shoulder_lift': 0
        }
    
    def pose_event_callback(self, msg):
        """Track current pose and peak currents."""
        if msg.event_type == 'start':
            self.current_pose = msg.pose_name
            self.current_pose_num = msg.pose_number
            self.pose_active = True
            self.reset_peak_tracking()
            self.get_logger().info(f'→ Pose started: {msg.pose_name}')
        elif msg.event_type == 'end':
            self.pose_active = False
            # Report peak servo currents for this pose
            self.get_logger().info(f'← Pose ended: {msg.pose_name}')
            self.get_logger().info(f'   Peak servo currents (mA):')
            for joint, peak in self.peak_servo_currents.items():
                self.get_logger().info(f'     {joint:12s}: {abs(peak):6.1f}')
    
    def joint_state_callback(self, msg):
        """Store latest joint state."""
        self.latest_joint_state = msg
    
    def dxl_state_callback(self, msg):
        """Store latest dynamixel state."""
        self.latest_dxl_state = msg
    
    def get_servo_currents(self):
        """Extract current values from servo telemetry. Returned only for XL330s """
        currents = {
            'elbow_flex': 0,
            'wrist_flex': 0,
            'wrist_roll': 0,
            'pen_holder': 0
        }
        
        if not self.latest_dxl_state:
            return currents

        # Parse DynamixelState message
        for i, servo_id in enumerate(self.latest_dxl_state.id):
            # Find joint name for this servo ID
            joint_name = None
            for name, sid in self.servo_id_map.items():
                if sid == servo_id and name in currents.keys():
                    joint_name = name
                    break
            if joint_name is None:
                continue
            
            if joint_name and hasattr(self.latest_dxl_state, 'present_current'):
                if i < len(self.latest_dxl_state.present_current):
                    # DynamixelState.present_current is in mA
                    currents[joint_name] = self.latest_dxl_state.present_current[i]
                    
                    # Track peak during active pose
                    if self.pose_active:
                        if abs(currents[joint_name]) > abs(self.peak_servo_currents[joint_name]):
                            self.peak_servo_currents[joint_name] = currents[joint_name]
        
        return currents
    
    def get_servo_temperatures(self):
        """Extract temperature values from servo telemetry."""
        temperatures = {
            'shoulder_pan': 0,
            'shoulder_lift': 0,
            'elbow_flex': 0,
            'wrist_flex': 0,
            'wrist_roll': 0,
            'pen_holder': 0
        }
        
        if not self.latest_dxl_state:
            return temperatures
        
        # Parse DynamixelState message
        for i, servo_id in enumerate(self.latest_dxl_state.id):
            # Find joint name for this servo ID
            joint_name = None
            for name, sid in self.servo_id_map.items():
                if sid == servo_id:
                    joint_name = name
                    break
            
            if joint_name and hasattr(self.latest_dxl_state, 'present_temperature'):
                if i < len(self.latest_dxl_state.present_temperature):
                    # DynamixelState.temperature is in degrees centigrade
                    temperatures[joint_name] = self.latest_dxl_state.present_temperature[i]
                    
                    # Track peak during active pose
                    if self.pose_active:
                        if abs(temperatures[joint_name]) > abs(self.peak_servo_temperatures[joint_name]):
                            self.peak_servo_temperatures[joint_name] = temperatures[joint_name]
        
        return temperatures

    def get_servo_loads(self):
        """Extract load values from servo telemetry. This is returned for XL430's instead of current"""
        loads = {
            'shoulder_pan': 0,
            'shoulder_lift': 0,
        }
        
        if not self.latest_dxl_state:
            return loads
        
        # Parse DynamixelState message
        for i, servo_id in enumerate(self.latest_dxl_state.id):
            # Find joint name for this servo ID
            joint_name = None
            for name, sid in self.servo_id_map.items():
                if sid == servo_id and name in loads.keys():
                    joint_name = name
                    break
            if joint_name is None:
                    continue
            
            if joint_name and hasattr(self.latest_dxl_state, 'present_load'):
                if i < len(self.latest_dxl_state.present_load):
                    # DynamixelState.present_load is in percentage
                    loads[joint_name] = self.latest_dxl_state.present_load[i] / 10.0 # Convert to %
                    
                    # Track peak during active pose
                    if self.pose_active:
                        if abs(loads[joint_name]) > abs(self.peak_servo_loads[joint_name]):
                            self.peak_servo_loads[joint_name] = loads[joint_name]
        
        return loads

    def get_servo_voltages(self):
        """Extract input voltage for all 6 servos (in V). Used for sag tracking."""
        voltages = {
            'shoulder_pan':  0.0,
            'shoulder_lift': 0.0,
            'elbow_flex':    0.0,
            'wrist_flex':    0.0,
            'wrist_roll':    0.0,
            'pen_holder':    0.0,
        }

        if not self.latest_dxl_state:
            return voltages

        for i, servo_id in enumerate(self.latest_dxl_state.id):
            joint_name = None
            for name, sid in self.servo_id_map.items():
                if sid == servo_id:
                    joint_name = name
                    break
            if joint_name is None:
                continue

            if hasattr(self.latest_dxl_state, 'present_input_voltage'):
                if i < len(self.latest_dxl_state.present_input_voltage):
                    # present_input_voltage is in 0.1V units — convert to V
                    voltages[joint_name] = self.latest_dxl_state.present_input_voltage[i] / 10.0

        return voltages

    def telemetry_callback(self, msg):
        """Log power data with servo currents."""
        # Get timestamp in seconds
        ts = time.time()
        
        # Get current servo readings
        servo_currents = self.get_servo_currents()
        servo_temperatures = self.get_servo_temperatures()
        servo_loads = self.get_servo_loads()
        servo_voltages = self.get_servo_voltages()
        total_xl330_current_ma = sum(
            abs(servo_currents[j]) for j in servo_currents
        )
        
        self.writer.writerow([
            ts,
            # INA226 primary; fall back to INA219 if INA226 fields are zero
            msg.bus_12v_voltage_226 if msg.bus_12v_voltage_226 > 0.0 else msg.bus_12v_voltage,
            msg.current_12v_226     if msg.bus_12v_voltage_226 > 0.0 else msg.current_12v,
            msg.power_12v_226       if msg.bus_12v_voltage_226 > 0.0 else msg.power_12v,
            msg.bus_5v_voltage_226  if msg.bus_5v_voltage_226  > 0.0 else msg.bus_5v_voltage,
            msg.current_5v_226      if msg.bus_5v_voltage_226  > 0.0 else msg.current_5v,
            msg.power_5v_226        if msg.bus_5v_voltage_226  > 0.0 else msg.power_5v,
            msg.total_power,
            'active' if self.pose_active else '',
            self.current_pose,
            self.current_pose_num,
            servo_currents['elbow_flex'],
            servo_currents['wrist_flex'],
            servo_currents['wrist_roll'],
            servo_currents['pen_holder'],
            servo_loads['shoulder_pan'],
            servo_loads['shoulder_lift'],
            servo_temperatures['shoulder_pan'],
            servo_temperatures['shoulder_lift'],
            servo_temperatures['elbow_flex'],
            servo_temperatures['wrist_flex'],
            servo_temperatures['wrist_roll'],
            servo_temperatures['pen_holder'],
            servo_voltages['shoulder_pan'],
            servo_voltages['shoulder_lift'],
            servo_voltages['elbow_flex'],
            servo_voltages['wrist_flex'],
            servo_voltages['wrist_roll'],
            servo_voltages['pen_holder'],
            total_xl330_current_ma,
        ])
        
        # Flush periodically
        self.csvfile.flush()
    
    def destroy_node(self):
        """Cleanup."""
        self.csvfile.close()
        self.get_logger().info(f'Log file closed: {self.filename}')
        super().destroy_node()


def main():
    rclpy.init()
    logger = PowerLoggerEnhanced()
    
    try:
        rclpy.spin(logger)
    except KeyboardInterrupt:
        pass
    finally:
        logger.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
