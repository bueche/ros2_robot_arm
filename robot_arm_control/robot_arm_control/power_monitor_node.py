#!/usr/bin/env python3
"""
Power Monitor Node

Reads CSV lines from ESP32 over serial and publishes PowerTelemetry.

Expected CSV column layout (matches read_ina219_ina226.ino):
  col  0: t_us
  col  1- 4: INA219 12V  (bus_v, shunt_mv, current_A, power_W)
  col  5- 8: INA219 5V   (bus_v, shunt_mv, current_A, power_W)
  col  9-12: INA226 12V  (bus_v, shunt_mv, current_A, power_W)
  col 13-16: INA226 5V   (bus_v, shunt_mv, current_A, power_W)
  col 17:    current_5v_diff_mA   (INA219 - INA226)
  col 18:    current_5v_avg_mA    ((INA219 + INA226) / 2)

Backward-compatible: if only 9 cols present (no INA226 data), 226 fields are zeroed.
"""

import rclpy
from rclpy.node import Node
from koch_v1_1_msgs.msg import PowerTelemetry
import serial
from std_msgs.msg import Header


# Minimum cols required for a valid legacy (3-sensor) line
_MIN_COLS_LEGACY = 9   # t_us + INA219_12V(4) + INA219_5V(4)
# Minimum cols required for a full 4-sensor line
_MIN_COLS_FULL   = 19  # + INA226_12V(4) + INA226_5V(4) + diff + avg


class PowerMonitorNode(Node):
    def __init__(self):
        super().__init__('power_monitor_node')

        self.declare_parameter('serial_port', '/dev/ttyIna219')  # symlink to USB device
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('publish_rate', 20.0)  # Hz

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value

        self.power_pub = self.create_publisher(PowerTelemetry, 'power_telemetry', 10)

        try:
            self.serial = serial.Serial(port, baud, timeout=1.0)
            self.get_logger().info(f'Connected to ESP32 on {port} at {baud} baud')
            import time
            time.sleep(2)
            self.serial.reset_input_buffer()
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            raise

        rate = self.get_parameter('publish_rate').value
        self.timer = self.create_timer(1.0 / rate, self.read_and_publish)
        self.get_logger().info('Power monitor node started')

    # ------------------------------------------------------------------

    def read_and_publish(self):
        try:
            if self.serial.in_waiting == 0:
                return

            line = self.serial.readline().decode('utf-8').strip()

            # Log errors from sketch comment lines; skip all comment lines
            if line.startswith('#'):
                low = line.lower()
                if 'fail' in low or 'error' in low:
                    self.get_logger().warn(f'ESP32: {line}')
                return

            parts = line.split(',')
            if len(parts) < _MIN_COLS_LEGACY:
                return

            try:
                msg = PowerTelemetry()
                msg.header = Header()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'power_monitor'

                # --- INA219 12V (cols 1-4) ---
                msg.bus_12v_voltage   = float(parts[1])
                msg.shunt_12v_voltage = float(parts[2])
                msg.current_12v       = float(parts[3])
                msg.power_12v         = float(parts[4])

                # --- INA219 5V (cols 5-8) ---
                msg.bus_5v_voltage   = float(parts[5])
                msg.shunt_5v_voltage = float(parts[6])
                msg.current_5v       = float(parts[7])
                msg.power_5v         = float(parts[8])

                if len(parts) >= _MIN_COLS_FULL:
                    # --- INA226 12V (cols 9-12) ---
                    msg.bus_12v_voltage_226   = float(parts[9])
                    msg.shunt_12v_voltage_226 = float(parts[10])
                    msg.current_12v_226       = float(parts[11])
                    msg.power_12v_226         = float(parts[12])

                    # --- INA226 5V (cols 13-16) ---
                    msg.bus_5v_voltage_226   = float(parts[13])
                    msg.shunt_5v_voltage_226 = float(parts[14])
                    msg.current_5v_226       = float(parts[15])
                    msg.power_5v_226         = float(parts[16])

                    # --- 5V cross-check (cols 17-18) ---
                    msg.current_5v_diff_ma = float(parts[17])
                    msg.current_5v_avg_ma  = float(parts[18])

                    # --- Totals ---
                    msg.total_power     = msg.power_12v_226 + msg.power_5v_226
                    msg.total_power_219 = msg.power_12v     + msg.power_5v

                else:
                    # Legacy 3-sensor firmware — zero out INA226 fields
                    msg.bus_12v_voltage_226   = 0.0
                    msg.shunt_12v_voltage_226 = 0.0
                    msg.current_12v_226       = 0.0
                    msg.power_12v_226         = 0.0
                    msg.bus_5v_voltage_226    = 0.0
                    msg.shunt_5v_voltage_226  = 0.0
                    msg.current_5v_226        = 0.0
                    msg.power_5v_226          = 0.0
                    msg.current_5v_diff_ma    = 0.0
                    msg.current_5v_avg_ma     = 0.0

                    # Fall back to INA219 totals
                    msg.total_power     = msg.power_12v + msg.power_5v
                    msg.total_power_219 = msg.total_power

                # total_servo_current_5v is filled by power_logger, not here
                msg.total_servo_current_5v = 0.0

                self.power_pub.publish(msg)

            except (ValueError, IndexError) as e:
                self.get_logger().warn(f'Failed to parse line: {line!r} ({e})')

        except Exception as e:
            self.get_logger().error(f'Error reading serial: {e}')

    # ------------------------------------------------------------------

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
