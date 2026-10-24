#!/usr/bin/env python3
"""
Enhanced Power Logger with Servo Current Tracking

Logs INA226 power data (primary) and servo telemetry (current, load,
temperature, voltage) for whichever arm the 'arm_profile' parameter selects.
Falls back to INA219 fields when INA226 fields are zero (legacy firmware).
Tracks peak currents/loads during each pose period.

Usage:
  ros2 run robot_arm_control power_logger --ros-args -p arm_profile:=koch_v1_1
  ros2 run robot_arm_control power_logger --ros-args -p arm_profile:=so_arm101
"""

import rclpy
from rclpy.node import Node
from koch_v1_1_msgs.msg import PowerTelemetry, PoseEvent
import csv
from datetime import datetime
import time

from robot_arm_control.arm_profile import load_arm_profile
from robot_arm_control.servo_telemetry import ServoTelemetry


class PowerLoggerEnhanced(Node):
    def __init__(self):
        super().__init__('power_logger_enhanced')

        self.declare_parameter('arm_profile', 'koch_v1_1')
        self.profile = load_arm_profile(self.get_parameter('arm_profile').value)

        # CSV servo columns come from the profile: one column per joint whose servo
        # provides that telemetry field, plus one summed-current column per supply
        # rail that has servos providing present_current.
        def reporting(field):
            return [j.name for j in self.profile.joints_reporting(field)]
        self.current_joints = reporting('present_current')
        self.load_joints = reporting('present_load')
        self.temperature_joints = reporting('present_temperature')
        self.voltage_joints = reporting('present_input_voltage')
        self.current_rails = {}  # rail -> joints on it that provide present_current
        for j in self.profile.joints_reporting('present_current'):
            self.current_rails.setdefault(j.rail, []).append(j.name)

        # Create filename with timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = f'power_log_{timestamp}.csv'

        # Open CSV file
        self.csvfile = open(self.filename, 'w', newline='')
        self.writer = csv.writer(self.csvfile)

        self.writer.writerow(
            [
                'timestamp',
                'bus_12v', 'current_12v', 'power_12v',
                'bus_5v', 'current_5v', 'power_5v',
                'total_power',
                'pose_event', 'pose_name', 'pose_number',
            ]
            + [f'{j}_current_ma' for j in self.current_joints]
            + [f'{j}_load' for j in self.load_joints]
            + [f'{j}_temp' for j in self.temperature_joints]
            # Per-servo input voltages (V) -- for sag tracking
            + [f'{j}_voltage' for j in self.voltage_joints]
            # Sum of absolute servo currents (mA) per supply rail
            + [f'total_{rail}_current_ma' for rail in self.current_rails]
        )

        # Current pose tracking
        self.current_pose = ''
        self.current_pose_num = 0
        self.pose_active = False

        # Peak tracking during pose
        self.reset_peak_tracking()

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

        # Servo telemetry (DynamixelState or FeetechState, per profile)
        self.servo_telemetry = ServoTelemetry(self, self.profile, on_update=self.servo_update_callback)

        self.get_logger().info(f'Logging to: {self.filename}')
        self.get_logger().info(f'Tracking: INA226 power (primary) + servo telemetry ({self.profile.name})')

    def reset_peak_tracking(self):
        """Reset peak tracking for new pose."""
        self.peak_servo_currents = {j: 0.0 for j in self.current_joints}
        self.peak_servo_loads = {j: 0.0 for j in self.load_joints}
        self.peak_servo_temperatures = {j: 0.0 for j in self.temperature_joints}

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
            self.get_logger().info(f'← Pose ended: {msg.pose_name}')
            if self.peak_servo_currents:
                self.get_logger().info('   Peak servo currents (mA):')
                for joint, peak in self.peak_servo_currents.items():
                    self.get_logger().info(f'     {joint:13s}: {abs(peak):6.1f}')
            if self.peak_servo_loads:
                self.get_logger().info('   Peak servo loads (%):')
                for joint, peak in self.peak_servo_loads.items():
                    self.get_logger().info(f'     {joint:13s}: {abs(peak):6.1f}')

    def servo_update_callback(self, readings):
        """Update peaks on every servo telemetry message while a pose is active."""
        if not self.pose_active:
            return
        for joint, r in readings.items():
            if r.present_current is not None and abs(r.present_current) > abs(self.peak_servo_currents[joint]):
                self.peak_servo_currents[joint] = r.present_current          # mA
            if r.present_load is not None and abs(r.present_load / 10.0) > abs(self.peak_servo_loads[joint]):
                self.peak_servo_loads[joint] = r.present_load / 10.0         # 0.1 % -> %
            if r.present_temperature is not None and r.present_temperature > self.peak_servo_temperatures[joint]:
                self.peak_servo_temperatures[joint] = r.present_temperature  # degC

    def telemetry_callback(self, msg):
        """Log power data with servo telemetry."""
        # Get timestamp in seconds
        ts = time.time()

        readings = self.servo_telemetry.readings

        def value(joint, field, divisor=1):
            r = readings.get(joint)
            v = getattr(r, field) if r is not None else None
            return v / divisor if v is not None else 0

        # CSV units as before: current mA, load %, temperature degC, voltage V
        servo_currents = [value(j, 'present_current') for j in self.current_joints]
        servo_loads = [value(j, 'present_load', 10.0) for j in self.load_joints]
        servo_temperatures = [value(j, 'present_temperature') for j in self.temperature_joints]
        servo_voltages = [value(j, 'present_input_voltage', 10.0) for j in self.voltage_joints]
        rail_totals = [sum(abs(value(j, 'present_current')) for j in joints)
                       for joints in self.current_rails.values()]

        self.writer.writerow(
            [
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
            ]
            + servo_currents
            + servo_loads
            + servo_temperatures
            + servo_voltages
            + rail_totals
        )

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
