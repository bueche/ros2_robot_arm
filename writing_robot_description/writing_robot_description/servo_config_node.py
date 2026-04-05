#!/usr/bin/env python3
"""
servo_config_node.py

Runs once at startup BEFORE the controller manager acquires the port.
Opens /dev/ttyOpenRB directly via Dynamixel SDK, writes PID gains and
motion profile parameters to all servos, then exits cleanly.

Servo map:
  ID 1  XL430-W250  shoulder_pan
  ID 2  XL430-W250  shoulder_lift
  ID 3  XL330-M288  elbow_flex
  ID 4  XL330-M288  wrist_flex
  ID 5  XL330-M288  wrist_roll
  ID 6  XL330-M077  pen_holder
"""

import sys
import time
from dynamixel_sdk import PortHandler, PacketHandler

USB_PORT  = '/dev/ttyOpenRB'
BAUD_RATE = 1_000_000
PROTOCOL  = 2.0

# Control table addresses - verified against XL330-M288 documentation
ADDR_OPERATING_MODE       = 11   # 1 byte, EEPROM - must write with torque off
ADDR_TORQUE_ENABLE        = 64   # 1 byte
ADDR_POSITION_D_GAIN      = 80   # 2 bytes
ADDR_POSITION_I_GAIN      = 82   # 2 bytes
ADDR_POSITION_P_GAIN      = 84   # 2 bytes
ADDR_FEEDFORWARD_2ND_GAIN = 88   # 2 bytes
ADDR_FEEDFORWARD_1ST_GAIN = 90   # 2 bytes
ADDR_GOAL_CURRENT         = 102  # 2 bytes
ADDR_PROFILE_ACCELERATION = 108  # 4 bytes
ADDR_PROFILE_VELOCITY     = 112  # 4 bytes

TORQUE_DISABLE = 0
TORQUE_ENABLE  = 1

OP_MODE_POSITION         = 3  # standard position control (default)
OP_MODE_CURRENT_POSITION = 5  # current-based position control

# Per-servo tuning parameters. Keys are Dynamixel IDs.
#
# operating_mode:
#   3 = position control (default)
#   5 = current-based position control - servo drives toward goal position
#       but caps torque at goal_current. Use for joints fighting gravity.
#
# goal_current:
#   0       = use servo default (uncapped in position mode)
#   non-zero = cap in mA. Required when operating_mode=5.
#              XL330 stall current at 5V is 1470mA.
#              elbow_flex was hitting ~940mA before faulting so 600mA
#              gives a safe ceiling while still holding position.
#
# Profile Velocity 0 = unlimited. 200 rev/min is a smooth default.
# Profile Acceleration 0 = unlimited. 50 gives a gentle ramp.
# I gain is 0 on all servos - no hardware anti-windup on XL330.
# D gain damps oscillation if servo hunts after P increase.

SERVO_CONFIG = {
    #1: dict(name='shoulder_pan   [XL430]',
    #        operating_mode=OP_MODE_POSITION,
    #        P=800,  I=0, D=0, FF1=0, FF2=0,
    #        prof_vel=200, prof_acc=50,
    #        goal_current=0),
#
    #2: dict(name='shoulder_lift  [XL430]',
    #        operating_mode=OP_MODE_POSITION,
    #        P=800,  I=0, D=0, FF1=0, FF2=0,
    #        prof_vel=200, prof_acc=50,
    #        goal_current=0),
#
    3: dict(name='elbow_flex     [XL330]',
            operating_mode=OP_MODE_CURRENT_POSITION,
            P=2000, I=0, D=0, FF1=0, FF2=0,
            prof_vel=200, prof_acc=50,
            goal_current=600),

    4: dict(name='wrist_flex     [XL330]',
            operating_mode=OP_MODE_POSITION,
            P=2000, I=0, D=0, FF1=0, FF2=0,
            prof_vel=200, prof_acc=50,
            goal_current=0),

    5: dict(name='wrist_roll     [XL330]',
            operating_mode=OP_MODE_POSITION,
            P=2000, I=0, D=0, FF1=0, FF2=0,
            prof_vel=200, prof_acc=50,
            goal_current=0),

    6: dict(name='pen_holder     [XL330]',
            operating_mode=OP_MODE_POSITION,
            P=2000, I=0, D=0, FF1=0, FF2=0,
            prof_vel=200, prof_acc=50,
            goal_current=200),
}


def check(dxl_comm_result, dxl_error, ph, label):
    if dxl_comm_result != 0:
        print(f'  [ERROR] {label}: {ph.getTxRxResult(dxl_comm_result)}')
        return False
    if dxl_error != 0:
        print(f'  [ERROR] {label}: {ph.getRxPacketError(dxl_error)}')
        return False
    return True


def write1(ph, port, sid, addr, value, label):
    r, e = ph.write1ByteTxRx(port, sid, addr, value)
    return check(r, e, ph, label)


def write2(ph, port, sid, addr, value, label):
    r, e = ph.write2ByteTxRx(port, sid, addr, value)
    return check(r, e, ph, label)


def write4(ph, port, sid, addr, value, label):
    r, e = ph.write4ByteTxRx(port, sid, addr, value)
    return check(r, e, ph, label)


def main():
    print('=== servo_config_node: configuring Dynamixel servos ===')

    port = PortHandler(USB_PORT)
    ph   = PacketHandler(PROTOCOL)

    if not port.openPort():
        print(f'[FATAL] Cannot open port {USB_PORT}')
        sys.exit(1)
    print(f'Opened {USB_PORT}')

    if not port.setBaudRate(BAUD_RATE):
        print(f'[FATAL] Cannot set baud rate {BAUD_RATE}')
        port.closePort()
        sys.exit(1)
    print(f'Baud rate set to {BAUD_RATE}')

    all_ok = True

    for sid, cfg in SERVO_CONFIG.items():
        print(f'\n  Configuring ID {sid}  {cfg["name"]}')

        ok = write1(ph, port, sid, ADDR_TORQUE_ENABLE, TORQUE_DISABLE,
                    f'ID{sid} torque disable')
        if not ok:
            print(f'  [WARN] Could not disable torque on ID {sid} — skipping')
            all_ok = False
            continue

        time.sleep(0.01)

        # Operating mode is EEPROM - must be written first while torque is off.
        # When mode changes the servo resets profile and goal current registers,
        # so all other writes must come after this.
        op_mode = cfg.get('operating_mode', OP_MODE_POSITION)
        ok = write1(ph, port, sid, ADDR_OPERATING_MODE, op_mode,
                    f'ID{sid} operating mode')
        if not ok:
            print(f'  [WARN] Could not set operating mode on ID {sid} — skipping')
            all_ok = False
            continue

        time.sleep(0.01)

        ok  = write2(ph, port, sid, ADDR_POSITION_P_GAIN,      cfg['P'],
                     f'ID{sid} P gain')
        ok &= write2(ph, port, sid, ADDR_POSITION_I_GAIN,      cfg['I'],
                     f'ID{sid} I gain')
        ok &= write2(ph, port, sid, ADDR_POSITION_D_GAIN,      cfg['D'],
                     f'ID{sid} D gain')
        ok &= write2(ph, port, sid, ADDR_FEEDFORWARD_1ST_GAIN, cfg['FF1'],
                     f'ID{sid} FF1 gain')
        ok &= write2(ph, port, sid, ADDR_FEEDFORWARD_2ND_GAIN, cfg['FF2'],
                     f'ID{sid} FF2 gain')
        ok &= write4(ph, port, sid, ADDR_PROFILE_VELOCITY,     cfg['prof_vel'],
                     f'ID{sid} profile velocity')
        ok &= write4(ph, port, sid, ADDR_PROFILE_ACCELERATION, cfg['prof_acc'],
                     f'ID{sid} profile acceleration')

        if cfg['goal_current'] > 0:
            ok &= write2(ph, port, sid, ADDR_GOAL_CURRENT, cfg['goal_current'],
                         f'ID{sid} goal current')

        if ok:
            mode_str = 'current-pos' if op_mode == OP_MODE_CURRENT_POSITION else 'position'
            print(f'  ok mode={mode_str} P={cfg["P"]} I={cfg["I"]} '
                  f'D={cfg["D"]} FF1={cfg["FF1"]} FF2={cfg["FF2"]} '
                  f'vel={cfg["prof_vel"]} acc={cfg["prof_acc"]} '
                  f'goal_curr={cfg["goal_current"]}')
        else:
            all_ok = False

        time.sleep(0.01)

    port.closePort()
    print('\n=== servo_config_node: done ===')

    if not all_ok:
        print('[WARN] One or more servos had errors — check output above')
        sys.exit(1)

    sys.exit(0)


if __name__ == '__main__':
    main()
