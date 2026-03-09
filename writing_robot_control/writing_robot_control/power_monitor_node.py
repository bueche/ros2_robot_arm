#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from koch_v1_1_msgs.msg import PowerTelemetry
import serial
import re
from std_msgs.msg import Header

class PowerMonitorNode(Node):
    def __init__(self):
        super().__init__('power_monitor_node')
        
        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyIna219') # assumed to be soft link to usb device
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('publish_rate', 20.0)  # Hz
        
        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value
        
        # Publisher
        self.power_pub = self.create_publisher(
            PowerTelemetry,
            'power_telemetry',
            10
        )
        
        # Open serial connection
        try:
            self.serial = serial.Serial(port, baud, timeout=1.0)
            self.get_logger().info(f'Connected to Arduino on {port} at {baud} baud')
            
            # Clear any startup messages
            import time
            time.sleep(2)
            self.serial.reset_input_buffer()
            
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            raise
        
        # Timer for reading serial data
        rate = self.get_parameter('publish_rate').value
        self.timer = self.create_timer(1.0 / rate, self.read_and_publish)
        
        self.get_logger().info('Power monitor node started')
    
    def read_and_publish(self):
        """Read CSV line from Arduino and publish telemetry"""
        try:
            if self.serial.in_waiting > 0:
                line = self.serial.readline().decode('utf-8').strip()
                
                # Skip comment lines
                if line.startswith('#'):
                    return
                
                # Parse CSV: t_us,bus12_v,shunt12_mv,current12_A,power12_W,bus5_v,shunt5_mv,current5_A,power5_W
                parts = line.split(',')
                if len(parts) < 8:
                    return
                
                try:
                    msg = PowerTelemetry()
                    msg.header = Header()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = 'power_monitor'
                    
                    # Parse values (skip timestamp)
                    msg.bus_12v_voltage = float(parts[1])
                    msg.shunt_12v_voltage = float(parts[2])
                    msg.current_12v = float(parts[3])
                    msg.power_12v = float(parts[4])
                    
                    msg.bus_5v_voltage = float(parts[5])
                    msg.shunt_5v_voltage = float(parts[6])
                    msg.current_5v = float(parts[7])
                    msg.power_5v = float(parts[8])
                    
                    if len(parts) >= 12:
                        msg.bus_5v_voltage_226 = float(parts[9])
                        msg.shunt_5v_voltage_226 = float(parts[10])
                        msg.current_5v_226 = float(parts[11])
                        msg.power_5v_226 = float(parts[12])
                    else:
                        msg.bus_5v_voltage_226 = float(0)
                        msg.shunt_5v_voltage_226 = float(0)
                        msg.current_5v_226 = float(0)
                        msg.power_5v_226 = float(0)
                    
                    msg.total_power = msg.power_12v + msg.power_5v
                    msg.total_power = msg.power_12v + msg.power_5v
                    
                    self.power_pub.publish(msg)
                    
                except ValueError as e:
                    self.get_logger().warn(f'Failed to parse line: {line}')
                    
        except Exception as e:
            self.get_logger().error(f'Error reading serial: {e}')
    
    def destroy_node(self):
        if hasattr(self, 'serial') and self.serial.is_open:
            self.serial.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PowerMonitorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
