#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from koch_v1_1_msgs.msg import PowerTelemetry, PoseEvent
import csv
from datetime import datetime

class PowerLogger(Node):
    def __init__(self):
        super().__init__('power_logger')
        
        # Create filename with timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = f'power_log_{timestamp}.csv'
        
        # Open CSV file
        self.csvfile = open(self.filename, 'w', newline='')
        self.writer = csv.writer(self.csvfile)
        
        # Write header
        self.writer.writerow([
            'timestamp', 
            'bus_12v', 'current_12v', 'power_12v',
            'bus_5v', 'current_5v', 'power_5v',
            'total_power',
            'pose_event', 'pose_name', 'pose_number'
        ])
        
        # Current pose tracking
        self.current_pose = ''
        self.current_pose_num = 0
        
        # Subscribe to telemetry
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
        
        self.get_logger().info(f'Logging to: {self.filename}')
    
    def pose_event_callback(self, msg):
        """Track current pose from events"""
        if msg.event_type == 'start':
            self.current_pose = msg.pose_name
            self.current_pose_num = msg.pose_number
            self.get_logger().info(f'→ Pose started: {msg.pose_name}')
        elif msg.event_type == 'end':
            self.get_logger().info(f'← Pose ended: {msg.pose_name}')
            # Keep pose name for a bit after end
    
    def telemetry_callback(self, msg):
        # Get timestamp in seconds
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        self.writer.writerow([
            ts,
            msg.bus_12v_voltage,
            msg.current_12v,
            msg.power_12v,
            msg.bus_5v_voltage,
            msg.current_5v,
            msg.power_5v,
            msg.total_power,
            'active' if self.current_pose else '',
            self.current_pose,
            self.current_pose_num
        ])
        
        # Flush periodically
        self.csvfile.flush()
    
    def destroy_node(self):
        self.csvfile.close()
        self.get_logger().info(f'Log file closed: {self.filename}')
        super().destroy_node()

def main():
    rclpy.init()
    logger = PowerLogger()
    
    try:
        rclpy.spin(logger)
    except KeyboardInterrupt:
        pass
    finally:
        logger.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()