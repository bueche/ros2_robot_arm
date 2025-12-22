#!/usr/bin/env python3
"""
Dynamixel Continuous Servo Monitor - CORRECTED
Monitors servos periodically to catch intermittent failures

FIXED: Now correctly handles XL430 vs XL330 differences
- XL430 (IDs 1, 2): Reads Present Load (%), NOT Current
- XL330 (IDs 3-6): Reads Present Current (mA)

Usage:
  python3 servo_monitor.py --interval 5 --duration 300
  python3 servo_monitor.py --interval 10 --csv monitor.csv
  
Useful for:
  - Catching thermal issues (temperature rising over time)
  - Voltage sag detection (voltage dropping under load)
  - Intermittent communication failures
  - Degradation patterns
"""

from dynamixel_sdk import *
import time
import argparse
import csv
from datetime import datetime
import signal
import sys

# Servo configuration with types
SERVOS = {
    1: {'name': 'shoulder_pan', 'model': 'XL430-W250', 'type': 'XL430'},
    2: {'name': 'shoulder_lift', 'model': 'XL430-W250', 'type': 'XL430'},
    3: {'name': 'elbow_flex', 'model': 'XL330-M288', 'type': 'XL330'},
    4: {'name': 'wrist_flex', 'model': 'XL330-M288', 'type': 'XL330'},
    5: {'name': 'wrist_roll', 'model': 'XL330-M288', 'type': 'XL330'},
    6: {'name': 'pen_holder', 'model': 'XL330-M077', 'type': 'XL330'}
}

# Protocol 2.0
PROTOCOL_VERSION = 2.0

# Control table addresses (Protocol 2.0)
# Common addresses
ADDR_TORQUE_ENABLE = 64
ADDR_PRESENT_POSITION = 132
ADDR_PRESENT_TEMPERATURE = 146
ADDR_PRESENT_INPUT_VOLTAGE = 144
ADDR_HARDWARE_ERROR_STATUS = 70

# Address 126 is DIFFERENT for each servo type:
# XL430: Present Load (percentage)
# XL330: Present Current (mA)
ADDR_126_XL430_LOAD = 126      # Present Load for XL430
ADDR_126_XL330_CURRENT = 126   # Present Current for XL330

# Torque values
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

# Port
DEVICENAME = '/dev/ttyUSB0'
BAUDRATE = 57600

# Global for clean shutdown
running = True

def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully."""
    global running
    print('\n\nShutdown requested...')
    running = False

def enable_torque_all(port_handler, packet_handler, servo_ids):
    """Enable torque on all servos."""
    print("\nEnabling torque on all servos...")
    
    success_count = 0
    for servo_id in servo_ids:
        result, error = packet_handler.write1ByteTxRx(
            port_handler, servo_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE
        )
        
        if result == COMM_SUCCESS and error == 0:
            print(f"  ✓ Servo #{servo_id} ({SERVOS[servo_id]['name']}): Torque enabled")
            success_count += 1
        else:
            print(f"  ❌ Servo #{servo_id} ({SERVOS[servo_id]['name']}): Failed to enable torque")
            if result != COMM_SUCCESS:
                print(f"     Error: {packet_handler.getTxRxResult(result)}")
            if error != 0:
                print(f"     Packet Error: {packet_handler.getRxPacketError(error)}")
    
    print(f"\nTorque enabled on {success_count}/{len(servo_ids)} servos")
    return success_count == len(servo_ids)

def disable_torque_all(port_handler, packet_handler, servo_ids):
    """Disable torque on all servos (for safety at shutdown)."""
    print("\nDisabling torque on all servos...")
    
    for servo_id in servo_ids:
        result, error = packet_handler.write1ByteTxRx(
            port_handler, servo_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE
        )
        
        if result == COMM_SUCCESS and error == 0:
            print(f"  ✓ Servo #{servo_id} ({SERVOS[servo_id]['name']}): Torque disabled")
        else:
            print(f"  ⚠️ Servo #{servo_id} ({SERVOS[servo_id]['name']}): Failed to disable torque")

def read_servo_telemetry(port_handler, packet_handler, servo_id):
    """Read all telemetry from a single servo (handles XL430 vs XL330 differences)."""
    
    servo_type = SERVOS[servo_id]['type']
    
    telemetry = {
        'servo_id': servo_id,
        'name': SERVOS[servo_id]['name'],
        'model': SERVOS[servo_id]['model'],
        'type': servo_type,
        'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
        'position': None,
        'position_success': False,
        'position_time_ms': 0,
        'temperature_c': None,
        'temperature_success': False,
        'temperature_time_ms': 0,
        'voltage_v': None,
        'voltage_success': False,
        'voltage_time_ms': 0,
        'hardware_error': None,
        'hardware_error_success': False,
        'total_time_ms': 0,
        'all_success': False
    }
    
    # XL430 has Load, XL330 has Current
    if servo_type == 'XL430':
        telemetry['load_percent'] = None
        telemetry['load_success'] = False
        telemetry['load_time_ms'] = 0
    else:  # XL330
        telemetry['current_ma'] = None
        telemetry['current_success'] = False
        telemetry['current_time_ms'] = 0
    
    start_total = time.time()
    
    # Read position (common to all)
    start = time.time()
    position, result, error = packet_handler.read4ByteTxRx(
        port_handler, servo_id, ADDR_PRESENT_POSITION
    )
    telemetry['position_time_ms'] = (time.time() - start) * 1000
    
    if result == COMM_SUCCESS and error == 0:
        telemetry['position'] = position
        telemetry['position_success'] = True
    
    # Read address 126 (Load for XL430, Current for XL330)
    start = time.time()
    value_126, result, error = packet_handler.read2ByteTxRx(
        port_handler, servo_id, 126
    )
    
    if servo_type == 'XL430':
        telemetry['load_time_ms'] = (time.time() - start) * 1000
        if result == COMM_SUCCESS and error == 0:
            # Present Load: signed percentage value
            # Range: -100 to +100 (%)
            if value_126 < 32768:
                load = value_126 * 0.1  # Units to percentage
            else:
                load = (value_126 - 65536) * 0.1
            telemetry['load_percent'] = load
            telemetry['load_success'] = True
    else:  # XL330
        telemetry['current_time_ms'] = (time.time() - start) * 1000
        if result == COMM_SUCCESS and error == 0:
            # Present Current: signed value in mA
            # Conversion: 2.69mA per unit
            if value_126 < 32768:
                current = value_126 * 2.69
            else:
                current = (value_126 - 65536) * 2.69
            telemetry['current_ma'] = current
            telemetry['current_success'] = True
    
    # Read temperature (common to all)
    start = time.time()
    temp, result, error = packet_handler.read1ByteTxRx(
        port_handler, servo_id, ADDR_PRESENT_TEMPERATURE
    )
    telemetry['temperature_time_ms'] = (time.time() - start) * 1000
    
    if result == COMM_SUCCESS and error == 0:
        telemetry['temperature_c'] = temp
        telemetry['temperature_success'] = True
    
    # Read voltage (common to all)
    start = time.time()
    voltage, result, error = packet_handler.read2ByteTxRx(
        port_handler, servo_id, ADDR_PRESENT_INPUT_VOLTAGE
    )
    telemetry['voltage_time_ms'] = (time.time() - start) * 1000
    
    if result == COMM_SUCCESS and error == 0:
        # Convert to volts (0.1V per unit)
        telemetry['voltage_v'] = voltage * 0.1
        telemetry['voltage_success'] = True
    
    # Read hardware errors (common to all)
    error_status, result, error = packet_handler.read1ByteTxRx(
        port_handler, servo_id, ADDR_HARDWARE_ERROR_STATUS
    )
    
    if result == COMM_SUCCESS and error == 0:
        telemetry['hardware_error'] = error_status
        telemetry['hardware_error_success'] = True
    
    telemetry['total_time_ms'] = (time.time() - start_total) * 1000
    
    # Check all_success based on servo type
    if servo_type == 'XL430':
        telemetry['all_success'] = (
            telemetry['position_success'] and
            telemetry['load_success'] and
            telemetry['temperature_success'] and
            telemetry['voltage_success'] and
            telemetry['hardware_error_success']
        )
    else:  # XL330
        telemetry['all_success'] = (
            telemetry['position_success'] and
            telemetry['current_success'] and
            telemetry['temperature_success'] and
            telemetry['voltage_success'] and
            telemetry['hardware_error_success']
        )
    
    return telemetry

def print_telemetry(iteration, telemetry_data):
    """Print telemetry in readable format."""
    
    print(f"\n{'='*80}")
    print(f"ITERATION {iteration} - {datetime.now().strftime('%H:%M:%S')}")
    print(f"{'='*80}")
    
    for data in telemetry_data:
        status = "✓" if data['all_success'] else "❌"
        
        # Temperature warning
        temp_warn = ""
        if data['temperature_c'] is not None:
            if data['temperature_c'] > 60:
                temp_warn = " ⚠️ HOT!"
            elif data['temperature_c'] > 50:
                temp_warn = " ⚠️"
        
        # Voltage warning
        voltage_warn = ""
        if data['voltage_v'] is not None:
            if data['voltage_v'] < 4.5:
                voltage_warn = " ⚠️ LOW!"
            elif data['voltage_v'] < 4.8:
                voltage_warn = " ⚠️"
        
        # Response time warning
        time_warn = ""
        if data['total_time_ms'] > 100:
            time_warn = " ⚠️ SLOW!"
        elif data['total_time_ms'] > 50:
            time_warn = " ⚠️"
        
        # Build output line
        line = f"{status} Servo #{data['servo_id']} ({data['name']:15s}) [{data['type']}]: "
        
        if data['temperature_c'] is not None:
            line += f"Temp:{data['temperature_c']:3d}°C{temp_warn:8s} "
        else:
            line += f"Temp:  --        "
        
        if data['voltage_v'] is not None:
            line += f"Volt:{data['voltage_v']:4.2f}V{voltage_warn:8s} "
        else:
            line += f"Volt: --        "
        
        # Different output for XL430 vs XL330
        if data['type'] == 'XL430':
            if data.get('load_percent') is not None:
                line += f"Load:{data['load_percent']:5.1f}% "
            else:
                line += f"Load:  --- % "
        else:  # XL330
            if data.get('current_ma') is not None:
                line += f"Curr:{data['current_ma']:5.0f}mA "
            else:
                line += f"Curr:  --- mA "
        
        line += f"Time:{data['total_time_ms']:5.1f}ms{time_warn}"
        
        print(line)

def write_csv_header(filename):
    """Write CSV header."""
    with open(filename, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=[
            'iteration', 'timestamp', 'servo_id', 'servo_name', 'servo_model', 'servo_type',
            'position', 'current_ma', 'load_percent', 'temperature_c', 'voltage_v',
            'hardware_error', 'position_time_ms', 'current_or_load_time_ms',
            'temperature_time_ms', 'voltage_time_ms', 'total_time_ms', 'all_success'
        ])
        writer.writeheader()

def append_csv_data(filename, iteration, telemetry_data):
    """Append telemetry data to CSV."""
    with open(filename, 'a', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=[
            'iteration', 'timestamp', 'servo_id', 'servo_name', 'servo_model', 'servo_type',
            'position', 'current_ma', 'load_percent', 'temperature_c', 'voltage_v',
            'hardware_error', 'position_time_ms', 'current_or_load_time_ms',
            'temperature_time_ms', 'voltage_time_ms', 'total_time_ms', 'all_success'
        ])
        
        for data in telemetry_data:
            # Determine current_or_load_time based on servo type
            if data['type'] == 'XL430':
                current_or_load_time = f"{data['load_time_ms']:.1f}"
            else:
                current_or_load_time = f"{data['current_time_ms']:.1f}"
            
            row = {
                'iteration': iteration,
                'timestamp': data['timestamp'],
                'servo_id': data['servo_id'],
                'servo_name': data['name'],
                'servo_model': data['model'],
                'servo_type': data['type'],
                'position': data['position'] if data['position'] is not None else '',
                'current_ma': f"{data.get('current_ma', ''):.1f}" if data.get('current_ma') is not None else '',
                'load_percent': f"{data.get('load_percent', ''):.1f}" if data.get('load_percent') is not None else '',
                'temperature_c': data['temperature_c'] if data['temperature_c'] is not None else '',
                'voltage_v': f"{data['voltage_v']:.2f}" if data['voltage_v'] is not None else '',
                'hardware_error': f"0x{data['hardware_error']:02X}" if data['hardware_error'] is not None else '',
                'position_time_ms': f"{data['position_time_ms']:.1f}",
                'current_or_load_time_ms': current_or_load_time,
                'temperature_time_ms': f"{data['temperature_time_ms']:.1f}",
                'voltage_time_ms': f"{data['voltage_time_ms']:.1f}",
                'total_time_ms': f"{data['total_time_ms']:.1f}",
                'all_success': data['all_success']
            }
            writer.writerow(row)

def print_summary(all_telemetry, start_time):
    """Print summary statistics."""
    
    print(f"\n{'='*80}")
    print("MONITORING SUMMARY")
    print(f"{'='*80}")
    
    duration = time.time() - start_time
    iterations = len(all_telemetry)
    
    print(f"Duration: {duration:.0f}s ({duration/60:.1f}min)")
    print(f"Iterations: {iterations}")
    print(f"Samples per servo: {iterations}")
    
    # Calculate statistics per servo
    for servo_id in sorted(SERVOS.keys()):
        servo_name = SERVOS[servo_id]['name']
        servo_type = SERVOS[servo_id]['type']
        
        temps = []
        voltages = []
        currents = []
        loads = []
        times = []
        failures = 0
        
        for iteration_data in all_telemetry:
            for data in iteration_data:
                if data['servo_id'] == servo_id:
                    if data['temperature_c'] is not None:
                        temps.append(data['temperature_c'])
                    if data['voltage_v'] is not None:
                        voltages.append(data['voltage_v'])
                    if data.get('current_ma') is not None:
                        currents.append(data['current_ma'])
                    if data.get('load_percent') is not None:
                        loads.append(data['load_percent'])
                    if data['total_time_ms'] is not None:
                        times.append(data['total_time_ms'])
                    if not data['all_success']:
                        failures += 1
        
        print(f"\nServo #{servo_id} ({servo_name}) [{servo_type}]:")
        
        if temps:
            print(f"  Temperature: {min(temps):3d}°C → {max(temps):3d}°C (avg {sum(temps)/len(temps):.1f}°C)")
        if voltages:
            print(f"  Voltage:     {min(voltages):4.2f}V → {max(voltages):4.2f}V (avg {sum(voltages)/len(voltages):.2f}V)")
        
        if servo_type == 'XL430':
            if loads:
                print(f"  Load:        {min(loads):4.1f}% → {max(loads):4.1f}% (avg {sum(loads)/len(loads):.1f}%)")
        else:  # XL330
            if currents:
                print(f"  Current:     {min(currents):4.0f}mA → {max(currents):4.0f}mA (avg {sum(currents)/len(currents):.0f}mA)")
        
        if times:
            print(f"  Resp time:   {min(times):5.1f}ms → {max(times):5.1f}ms (avg {sum(times)/len(times):.1f}ms)")
        
        if failures > 0:
            failure_rate = (failures / iterations) * 100
            print(f"  ⚠️ Failures:  {failures}/{iterations} ({failure_rate:.1f}%)")
        else:
            print(f"  ✓ Failures:  0/{iterations} (0%)")

def main():
    global running
    
    parser = argparse.ArgumentParser(
        description='Continuous Dynamixel Servo Monitor (XL430/XL330 aware)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Monitor every 5 seconds for 5 minutes (torque enabled, servos hold position)
  python3 servo_monitor.py --interval 5 --duration 300
  
  # Monitor every 10 seconds, save to CSV
  python3 servo_monitor.py --interval 10 --csv monitor.csv
  
  # Monitor indefinitely (Ctrl+C to stop)
  python3 servo_monitor.py --interval 5
  
  # Quick check every 2 seconds for 1 minute
  python3 servo_monitor.py --interval 2 --duration 60
  
  # Test communication only (no torque, servos limp)
  python3 servo_monitor.py --interval 5 --no-torque

Note: This version correctly handles XL430 vs XL330 differences:
  - XL430 (IDs 1, 2): Reports Load (%)
  - XL330 (IDs 3-6): Reports Current (mA)
        """
    )
    
    parser.add_argument('--interval', type=int, default=5,
                       help='Seconds between measurements (default: 5)')
    parser.add_argument('--duration', type=int, default=None,
                       help='Total duration in seconds (default: run until Ctrl+C)')
    parser.add_argument('--csv', type=str, default=None,
                       help='Save results to CSV file')
    parser.add_argument('--no-torque', action='store_true',
                       help='Skip torque enable (servos will be limp)')
    
    args = parser.parse_args()
    
    print("="*80)
    print("DYNAMIXEL SERVO MONITOR - XL430/XL330 AWARE")
    print("="*80)
    print(f"Port: {DEVICENAME}")
    print(f"Baudrate: {BAUDRATE}")
    print(f"Interval: {args.interval}s")
    if args.duration:
        print(f"Duration: {args.duration}s ({args.duration/60:.1f}min)")
    else:
        print(f"Duration: Indefinite (Ctrl+C to stop)")
    if args.csv:
        print(f"CSV output: {args.csv}")
    print()
    print("Servo Configuration:")
    for servo_id, info in SERVOS.items():
        print(f"  ID {servo_id}: {info['name']:<15} [{info['type']}] {info['model']}")
    print()
    print("⚠️  WARNING: Close ROS2 control before running!")
    print("   (This will conflict with ros2_control_node)")
    print()
    print("Press Ctrl+C to stop monitoring")
    print("="*80)
    
    # Setup signal handler
    signal.signal(signal.SIGINT, signal_handler)
    
    # Initialize PortHandler and PacketHandler
    port_handler = PortHandler(DEVICENAME)
    packet_handler = PacketHandler(PROTOCOL_VERSION)
    
    # Open port
    if not port_handler.openPort():
        print(f"❌ Failed to open port {DEVICENAME}")
        return 1
    
    print(f"✓ Port opened")
    
    # Set baudrate
    if not port_handler.setBaudRate(BAUDRATE):
        print(f"❌ Failed to set baudrate to {BAUDRATE}")
        return 1
    
    print(f"✓ Baudrate set to {BAUDRATE}")
    
    # Enable torque on all servos (unless --no-torque flag)
    servo_ids = sorted(SERVOS.keys())
    
    if not args.no_torque:
        if not enable_torque_all(port_handler, packet_handler, servo_ids):
            print("\n⚠️ WARNING: Not all servos responded to torque enable")
            print("   Continuing anyway, but some servos may not hold position")
    else:
        print("\n⚠️ Torque enable skipped (--no-torque flag)")
        print("   Servos will be limp")
    
    # Create CSV if requested
    if args.csv:
        write_csv_header(args.csv)
        print(f"✓ CSV file created: {args.csv}")
    
    print("\nStarting monitoring...\n")
    
    # Main monitoring loop
    iteration = 0
    start_time = time.time()
    all_telemetry = []
    
    try:
        while running:
            iteration += 1
            
            # Check duration limit
            if args.duration and (time.time() - start_time) >= args.duration:
                print(f"\n✓ Duration limit reached ({args.duration}s)")
                break
            
            # Read telemetry from all servos
            telemetry_data = []
            for servo_id in sorted(SERVOS.keys()):
                data = read_servo_telemetry(port_handler, packet_handler, servo_id)
                telemetry_data.append(data)
            
            # Store for summary
            all_telemetry.append(telemetry_data)
            
            # Print to console
            print_telemetry(iteration, telemetry_data)
            
            # Write to CSV
            if args.csv:
                append_csv_data(args.csv, iteration, telemetry_data)
            
            # Wait for next interval
            if running:  # Don't sleep if shutting down
                time.sleep(args.interval)
    
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        # Disable torque for safety
        if not args.no_torque:
            disable_torque_all(port_handler, packet_handler, servo_ids)
        
        # Print summary
        if all_telemetry:
            print_summary(all_telemetry, start_time)
        
        # Close port
        port_handler.closePort()
        print(f"\n✓ Monitoring stopped")
        
        if args.csv:
            print(f"✓ Results saved to {args.csv}")
    
    return 0


if __name__ == '__main__':
    sys.exit(main())
