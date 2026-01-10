#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from koch_v1_1_msgs.msg import PowerTelemetry
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
            'total_power'
        ])
        
        # Subscribe to telemetry
        self.sub = self.create_subscription(
            PowerTelemetry,
            'power_telemetry',
            self.telemetry_callback,
            10
        )
        
        self.get_logger().info(f'Logging to: {self.filename}')
    
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
            msg.total_power
        ])
        
        # Flush periodically
        self.csvfile.flush()
    
    def destroy_node(self):
        self.csvfile.close()
        self.get_logger().info('Log file closed')
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