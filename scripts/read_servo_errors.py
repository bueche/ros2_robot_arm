#!/usr/bin/env python3
"""
read_servo_errors.py

Reads hardware error status from all Koch v1.1 servos.
Run this AFTER a fault, BEFORE power cycling, to capture the error.

Usage:
    python3 read_servo_errors.py
"""

import sys
from dynamixel_sdk import PortHandler, PacketHandler

USB_PORT  = '/dev/ttyOpenRB'
BAUD_RATE = 1_000_000
PROTOCOL  = 2.0

ADDR_HARDWARE_ERROR_STATUS = 70   # 1 byte, read only
ADDR_ERROR_CODE            = 518  # 2 bytes, Pro series only - skip for XL
ADDR_PRESENT_POSITION      = 132  # 4 bytes
ADDR_PRESENT_CURRENT       = 126  # 2 bytes, signed
ADDR_PRESENT_VOLTAGE       = 144  # 2 bytes
ADDR_PRESENT_TEMPERATURE   = 146  # 1 byte
ADDR_TORQUE_ENABLE         = 64   # 1 byte

HARDWARE_ERROR_BITS = {
    0: 'Input Voltage Error',
    2: 'Overheating Error',
    3: 'Motor Encoder Error',
    4: 'Electrical Shock Error',
    5: 'Overload Error',
}

SERVO_MAP = {
    1: 'shoulder_pan   [XL430]',
    2: 'shoulder_lift  [XL430]',
    3: 'elbow_flex     [XL330]',
    4: 'wrist_flex     [XL330]',
    5: 'wrist_roll     [XL330]',
    6: 'pen_holder     [XL330]',
}


def read1(ph, port, sid, addr, label):
    val, result, error = ph.read1ByteTxRx(port, sid, addr)
    if result != 0 or error != 0:
        print(f'    [read error] {label}: comm={result} err={error}')
        return None
    return val


def read2(ph, port, sid, addr, label):
    val, result, error = ph.read2ByteTxRx(port, sid, addr)
    if result != 0 or error != 0:
        print(f'    [read error] {label}: comm={result} err={error}')
        return None
    return val


def read4(ph, port, sid, addr, label):
    val, result, error = ph.read4ByteTxRx(port, sid, addr)
    if result != 0 or error != 0:
        print(f'    [read error] {label}: comm={result} err={error}')
        return None
    return val


def decode_hardware_error(value):
    if value == 0:
        return 'No error'
    errors = []
    for bit, name in HARDWARE_ERROR_BITS.items():
        if value & (1 << bit):
            errors.append(name)
    return ', '.join(errors)


def signed16(val):
    if val is None:
        return None
    if val > 32767:
        val -= 65536
    return val


def main():
    print('=== Dynamixel Hardware Error Reader ===')
    print(f'Port: {USB_PORT}  Baud: {BAUD_RATE}')
    print()

    port = PortHandler(USB_PORT)
    ph   = PacketHandler(PROTOCOL)

    if not port.openPort():
        print(f'[FATAL] Cannot open port {USB_PORT}')
        sys.exit(1)

    if not port.setBaudRate(BAUD_RATE):
        print(f'[FATAL] Cannot set baud rate')
        port.closePort()
        sys.exit(1)

    any_error = False

    for sid, name in SERVO_MAP.items():
        print(f'ID {sid}  {name}')

        hw_error = read1(ph, port, sid, ADDR_HARDWARE_ERROR_STATUS, 'hw error')
        torque   = read1(ph, port, sid, ADDR_TORQUE_ENABLE,         'torque')
        position = read4(ph, port, sid, ADDR_PRESENT_POSITION,      'position')
        current  = signed16(read2(ph, port, sid, ADDR_PRESENT_CURRENT,     'current'))
        voltage  = read2(ph, port, sid, ADDR_PRESENT_VOLTAGE,       'voltage')
        temp     = read1(ph, port, sid, ADDR_PRESENT_TEMPERATURE,   'temperature')

        if hw_error is None:
            print('  Could not communicate with servo')
            print()
            continue

        error_str = decode_hardware_error(hw_error)
        torque_str = 'enabled' if torque else 'disabled'

        pos_deg  = (position / 4096.0 * 360.0) if position is not None else None
        volt_v   = (voltage  / 10.0)            if voltage  is not None else None

        print(f'  Hardware Error : 0x{hw_error:02X} ({hw_error})  ->  {error_str}')
        print(f'  Torque         : {torque_str}')
        if position is not None:
            print(f'  Position       : {position} counts  ({pos_deg:.2f} deg)')
        if current is not None:
            print(f'  Current        : {current} mA')
        if volt_v is not None:
            print(f'  Voltage        : {volt_v:.1f} V')
        if temp is not None:
            print(f'  Temperature    : {temp} C')

        if hw_error != 0:
            any_error = True
            print(f'  *** FAULT DETECTED ***')

        print()

    port.closePort()

    if any_error:
        print('=== One or more servos reported hardware errors ===')
    else:
        print('=== All servos clear ===')


if __name__ == '__main__':
    main()
