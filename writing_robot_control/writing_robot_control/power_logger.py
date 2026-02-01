#!/usr/bin/env python3
"""
Enhanced Power Logger with Servo Current Tracking

Logs both INA219 power data and Dynamixel servo current readings.
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
            'bus_5v_226', 'current_5v_226', 'power_5v_226',
            'total_power',
            'pose_event', 'pose_name', 'pose_number',
            # Servo currents (XL330s only report current)
            'elbow_current_ma', 'wrist_flex_current_ma', 
            'wrist_roll_current_ma', 'gripper_current_ma'
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
        self.get_logger().info('Tracking: INA219 power + Servo currents')
    
    def reset_peak_tracking(self):
        """Reset peak tracking for new pose."""
        self.peak_servo_currents = {
            'elbow_flex': 0,
            'wrist_flex': 0,
            'wrist_roll': 0,
            'pen_holder': 0
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
        """Extract current values from servo telemetry."""
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
                if sid == servo_id:
                    joint_name = name
                    break
            
            if joint_name and hasattr(self.latest_dxl_state, 'present_current'):
                if i < len(self.latest_dxl_state.present_current):
                    # DynamixelState.present_current is in mA
                    currents[joint_name] = self.latest_dxl_state.present_current[i]
                    
                    # Track peak during active pose
                    if self.pose_active:
                        if abs(currents[joint_name]) > abs(self.peak_servo_currents[joint_name]):
                            self.peak_servo_currents[joint_name] = currents[joint_name]
        
        return currents
    
    def telemetry_callback(self, msg):
        """Log power data with servo currents."""
        # Get timestamp in seconds
        ts = time.time()
        
        # Get current servo readings
        servo_currents = self.get_servo_currents()
        
        self.writer.writerow([
            ts,
            msg.bus_12v_voltage,
            msg.current_12v,
            msg.power_12v,
            msg.bus_5v_voltage,
            msg.current_5v,
            msg.power_5v,
            msg.bus_5v_voltage_226, # from INA226 sensors if present
            msg.current_5v_226, # from INA226 sensors if present
            msg.power_5v_226, # from INA226 sensors if present
            msg.total_power,
            'active' if self.pose_active else '',
            self.current_pose,
            self.current_pose_num,
            servo_currents['elbow_flex'],
            servo_currents['wrist_flex'],
            servo_currents['wrist_roll'],
            servo_currents['pen_holder']
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
