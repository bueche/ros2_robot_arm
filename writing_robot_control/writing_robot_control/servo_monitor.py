#!/usr/bin/env python3
"""
Dynamixel Continuous Servo Monitor
Monitors servos periodically to catch intermittent failures

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

# Servo configuration
SERVOS = {
    1: {'name': 'shoulder_pan', 'model': 'XL430-W250'},
    2: {'name': 'shoulder_lift', 'model': 'XL430-W250'},
    3: {'name': 'elbow_flex', 'model': 'XL330-M288'},
    4: {'name': 'wrist_flex', 'model': 'XL330-M288'},
    5: {'name': 'wrist_roll', 'model': 'XL330-M288'},
    6: {'name': 'pen_holder', 'model': 'XL330-M288'}
}

# Protocol 2.0
PROTOCOL_VERSION = 2.0

# Control table addresses (Protocol 2.0)
ADDR_TORQUE_ENABLE = 64
ADDR_PRESENT_POSITION = 132
ADDR_PRESENT_CURRENT = 126
ADDR_PRESENT_TEMPERATURE = 146
ADDR_PRESENT_INPUT_VOLTAGE = 144  # NEW: Monitor voltage!
ADDR_HARDWARE_ERROR_STATUS = 70

# Torque values
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

# Port
DEVICENAME = '/dev/ttyUSB0'
BAUDRATE = 1000000  # 1Mbps - faster and more reliable than 57600

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
    """Read all telemetry from a single servo."""
    
    telemetry = {
        'servo_id': servo_id,
        'name': SERVOS[servo_id]['name'],
        'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
        'position': None,
        'position_success': False,
        'position_time_ms': 0,
        'current_ma': None,
        'current_success': False,
        'current_time_ms': 0,
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
    
    start_total = time.time()
    
    # Read position
    start = time.time()
    position, result, error = packet_handler.read4ByteTxRx(
        port_handler, servo_id, ADDR_PRESENT_POSITION
    )
    telemetry['position_time_ms'] = (time.time() - start) * 1000
    
    if result == COMM_SUCCESS and error == 0:
        telemetry['position'] = position
        telemetry['position_success'] = True
    
    # Read current
    start = time.time()
    current, result, error = packet_handler.read2ByteTxRx(
        port_handler, servo_id, ADDR_PRESENT_CURRENT
    )
    telemetry['current_time_ms'] = (time.time() - start) * 1000
    
    if result == COMM_SUCCESS and error == 0:
        # Convert to mA (2.69mA per unit)
        telemetry['current_ma'] = current * 2.69 if current < 32768 else (current - 65536) * 2.69
        telemetry['current_success'] = True
    
    # Read temperature
    start = time.time()
    temp, result, error = packet_handler.read1ByteTxRx(
        port_handler, servo_id, ADDR_PRESENT_TEMPERATURE
    )
    telemetry['temperature_time_ms'] = (time.time() - start) * 1000
    
    if result == COMM_SUCCESS and error == 0:
        telemetry['temperature_c'] = temp
        telemetry['temperature_success'] = True
    
    # Read voltage
    start = time.time()
    voltage, result, error = packet_handler.read2ByteTxRx(
        port_handler, servo_id, ADDR_PRESENT_INPUT_VOLTAGE
    )
    telemetry['voltage_time_ms'] = (time.time() - start) * 1000
    
    if result == COMM_SUCCESS and error == 0:
        # Convert to volts (0.1V per unit)
        telemetry['voltage_v'] = voltage * 0.1
        telemetry['voltage_success'] = True
    
    # Read hardware errors
    error_status, result, error = packet_handler.read1ByteTxRx(
        port_handler, servo_id, ADDR_HARDWARE_ERROR_STATUS
    )
    
    if result == COMM_SUCCESS and error == 0:
        telemetry['hardware_error'] = error_status
        telemetry['hardware_error_success'] = True
    
    telemetry['total_time_ms'] = (time.time() - start_total) * 1000
    
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
        
        print(f"{status} Servo #{data['servo_id']} ({data['name']:15s}): ", end="")
        
        if data['temperature_c'] is not None:
            print(f"Temp:{data['temperature_c']:3d}°C{temp_warn:8s} ", end="")
        else:
            print(f"Temp:  --        ", end="")
        
        if data['voltage_v'] is not None:
            print(f"Volt:{data['voltage_v']:4.2f}V{voltage_warn:8s} ", end="")
        else:
            print(f"Volt: --        ", end="")
        
        if data['current_ma'] is not None:
            print(f"Curr:{data['current_ma']:5.0f}mA ", end="")
        else:
            print(f"Curr:  --- mA ", end="")
        
        print(f"Time:{data['total_time_ms']:5.1f}ms{time_warn}", end="")
        
        if data['hardware_error'] is not None and data['hardware_error'] != 0:
            print(f" ⚠️ HW_ERR:0x{data['hardware_error']:02X}", end="")
        
        print()

def write_csv_header(csv_file):
    """Write CSV header."""
    fieldnames = [
        'iteration',
        'timestamp',
        'servo_id',
        'servo_name',
        'position',
        'current_ma',
        'temperature_c',
        'voltage_v',
        'hardware_error',
        'position_time_ms',
        'current_time_ms',
        'temperature_time_ms',
        'voltage_time_ms',
        'total_time_ms',
        'all_success'
    ]
    
    with open(csv_file, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()

def append_csv_data(csv_file, iteration, telemetry_data):
    """Append telemetry data to CSV."""
    fieldnames = [
        'iteration',
        'timestamp',
        'servo_id',
        'servo_name',
        'position',
        'current_ma',
        'temperature_c',
        'voltage_v',
        'hardware_error',
        'position_time_ms',
        'current_time_ms',
        'temperature_time_ms',
        'voltage_time_ms',
        'total_time_ms',
        'all_success'
    ]
    
    with open(csv_file, 'a', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        
        for data in telemetry_data:
            row = {
                'iteration': iteration,
                'timestamp': data['timestamp'],
                'servo_id': data['servo_id'],
                'servo_name': data['name'],
                'position': data['position'] if data['position'] is not None else '',
                'current_ma': f"{data['current_ma']:.1f}" if data['current_ma'] is not None else '',
                'temperature_c': data['temperature_c'] if data['temperature_c'] is not None else '',
                'voltage_v': f"{data['voltage_v']:.2f}" if data['voltage_v'] is not None else '',
                'hardware_error': f"0x{data['hardware_error']:02X}" if data['hardware_error'] is not None else '',
                'position_time_ms': f"{data['position_time_ms']:.1f}",
                'current_time_ms': f"{data['current_time_ms']:.1f}",
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
        
        temps = []
        voltages = []
        currents = []
        times = []
        failures = 0
        
        for iteration_data in all_telemetry:
            for data in iteration_data:
                if data['servo_id'] == servo_id:
                    if data['temperature_c'] is not None:
                        temps.append(data['temperature_c'])
                    if data['voltage_v'] is not None:
                        voltages.append(data['voltage_v'])
                    if data['current_ma'] is not None:
                        currents.append(data['current_ma'])
                    if data['total_time_ms'] is not None:
                        times.append(data['total_time_ms'])
                    if not data['all_success']:
                        failures += 1
        
        print(f"\nServo #{servo_id} ({servo_name}):")
        
        if temps:
            print(f"  Temperature: {min(temps):3d}°C → {max(temps):3d}°C (avg {sum(temps)/len(temps):.1f}°C)")
        if voltages:
            print(f"  Voltage:     {min(voltages):4.2f}V → {max(voltages):4.2f}V (avg {sum(voltages)/len(voltages):.2f}V)")
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
        description='Continuous Dynamixel Servo Monitor',
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
        """
    )
    
    parser.add_argument('--interval', type=int, default=5,
                       help='Seconds between measurements (default: 5)')
    parser.add_argument('--duration', type=int, default=None,
                       help='Total duration in seconds (default: run until Ctrl+C)')
    parser.add_argument('--csv', type=str, default=None,
                       help='Save results to CSV file')
    parser.add_argument('--no-torque', action='store_true',
                       help='Skip torque enable (servos will be limp, useful for testing communication only)')
    
    args = parser.parse_args()
    
    print("="*80)
    print("DYNAMIXEL CONTINUOUS SERVO MONITOR")
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
        print("   Servos will be limp and show 0mA current")
    
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
