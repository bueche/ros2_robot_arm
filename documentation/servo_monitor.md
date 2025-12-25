# Running the Servo Monitor utility

The servo monitor software should not be run with the ros2 controlled robot. This is a stand alone piece of software that brings up the robot and takes measurements in it current pose.

## Table of Contents
- [Parameters](#parameters)
- [Example run](#example)

## Parameters
TBD

## Example 

```bash
ubuntu@bueche-rpi5:~/robot_ws$ python3 src/writing_robot_control/writing_robot_control/servo_monitor.py --interval 10 --duration 120

================================================================================
DYNAMIXEL SERVO MONITOR - XL430/XL330 AWARE
================================================================================
Port: /dev/ttyUSB0
Baudrate: 1000000
Interval: 10s
Duration: 120s (2.0min)

Servo Configuration:
  ID 1: shoulder_pan    [XL430] XL430-W250
  ID 2: shoulder_lift   [XL430] XL430-W250
  ID 3: elbow_flex      [XL330] XL330-M288
  ID 4: wrist_flex      [XL330] XL330-M288
  ID 5: wrist_roll      [XL330] XL330-M288
  ID 6: pen_holder      [XL330] XL330-M077

⚠️  WARNING: Close ROS2 control before running!
   (This will conflict with ros2_control_node)

Press Ctrl+C to stop monitoring
================================================================================
✓ Port opened
✓ Baudrate set to 1000000

Enabling torque on all servos...
  ✓ Servo #1 (shoulder_pan): Torque enabled
  ✓ Servo #2 (shoulder_lift): Torque enabled
  ✓ Servo #3 (elbow_flex): Torque enabled
  ✓ Servo #4 (wrist_flex): Torque enabled
  ✓ Servo #5 (wrist_roll): Torque enabled
  ✓ Servo #6 (pen_holder): Torque enabled

Torque enabled on 6/6 servos

Starting monitoring...


================================================================================
ITERATION 1 - 23:22:41
================================================================================
✓ Servo #1 (shoulder_pan   ) [XL430]: Temp: 32°C         Volt:12.20V         Load:  0.0% Time: 14.9ms
✓ Servo #2 (shoulder_lift  ) [XL430]: Temp: 39°C         Volt:12.20V         Load:  0.0% Time: 14.0ms
✓ Servo #3 (elbow_flex     ) [XL330]: Temp: 23°C         Volt:5.20V         Curr:    0mA Time:  7.0ms
✓ Servo #4 (wrist_flex     ) [XL330]: Temp: 20°C         Volt:5.20V         Curr:    0mA Time:  8.0ms
✓ Servo #5 (wrist_roll     ) [XL330]: Temp: 20°C         Volt:5.20V         Curr:    0mA Time:  8.0ms
✓ Servo #6 (pen_holder     ) [XL330]: Temp: 22°C         Volt:5.20V         Curr:    0mA Time:  7.0ms

================================================================================
ITERATION 2 - 23:22:51
================================================================================
✓ Servo #1 (shoulder_pan   ) [XL430]: Temp: 32°C         Volt:12.20V         Load:  0.0% Time: 14.3ms
✓ Servo #2 (shoulder_lift  ) [XL430]: Temp: 39°C         Volt:12.20V         Load:  3.3% Time: 12.0ms
✓ Servo #3 (elbow_flex     ) [XL330]: Temp: 23°C         Volt:5.20V         Curr:  -54mA Time:  7.0ms
✓ Servo #4 (wrist_flex     ) [XL330]: Temp: 20°C         Volt:5.20V         Curr:    0mA Time:  7.0ms
✓ Servo #5 (wrist_roll     ) [XL330]: Temp: 20°C         Volt:5.20V         Curr:    0mA Time:  8.0ms
✓ Servo #6 (pen_holder     ) [XL330]: Temp: 22°C         Volt:5.20V         Curr:    0mA Time:  8.0ms

================================================================================
ITERATION 3 - 23:23:01
================================================================================
✓ Servo #1 (shoulder_pan   ) [XL430]: Temp: 32°C         Volt:12.20V         Load:  0.0% Time: 14.3ms
✓ Servo #2 (shoulder_lift  ) [XL430]: Temp: 39°C         Volt:12.20V         Load:  3.3% Time: 12.0ms
✓ Servo #3 (elbow_flex     ) [XL330]: Temp: 23°C         Volt:5.20V         Curr:  -54mA Time:  8.9ms
✓ Servo #4 (wrist_flex     ) [XL330]: Temp: 20°C         Volt:5.20V         Curr:    0mA Time: 10.0ms
✓ Servo #5 (wrist_roll     ) [XL330]: Temp: 20°C         Volt:5.20V         Curr:    0mA Time: 10.0ms
✓ Servo #6 (pen_holder     ) [XL330]: Temp: 22°C         Volt:5.20V         Curr:    0mA Time: 10.0ms

================================================================================
ITERATION 4 - 23:23:11
================================================================================
✓ Servo #1 (shoulder_pan   ) [XL430]: Temp: 32°C         Volt:12.20V         Load:  0.0% Time: 14.2ms
✓ Servo #2 (shoulder_lift  ) [XL430]: Temp: 39°C         Volt:12.20V         Load:  3.4% Time: 13.0ms
✓ Servo #3 (elbow_flex     ) [XL330]: Temp: 23°C         Volt:5.20V         Curr:  -54mA Time:  7.0ms
✓ Servo #4 (wrist_flex     ) [XL330]: Temp: 20°C         Volt:5.20V         Curr:    0mA Time:  6.0ms
✓ Servo #5 (wrist_roll     ) [XL330]: Temp: 20°C         Volt:5.20V         Curr:    0mA Time:  7.0ms
✓ Servo #6 (pen_holder     ) [XL330]: Temp: 22°C         Volt:5.20V         Curr:    0mA Time:  9.0ms
^C

Shutdown requested...

Disabling torque on all servos...
  ✓ Servo #1 (shoulder_pan): Torque disabled
  ✓ Servo #2 (shoulder_lift): Torque disabled
  ✓ Servo #3 (elbow_flex): Torque disabled
  ✓ Servo #4 (wrist_flex): Torque disabled
  ✓ Servo #5 (wrist_roll): Torque disabled
  ✓ Servo #6 (pen_holder): Torque disabled

================================================================================
MONITORING SUMMARY
================================================================================
Duration: 40s (0.7min)
Iterations: 4
Samples per servo: 4

Servo #1 (shoulder_pan) [XL430]:
  Temperature:  32°C →  32°C (avg 32.0°C)
  Voltage:     12.20V → 12.20V (avg 12.20V)
  Load:         0.0% →  0.0% (avg 0.0%)
  Resp time:    14.2ms →  14.9ms (avg 14.4ms)
  ✓ Failures:  0/4 (0%)

Servo #2 (shoulder_lift) [XL430]:
  Temperature:  39°C →  39°C (avg 39.0°C)
  Voltage:     12.20V → 12.20V (avg 12.20V)
  Load:         0.0% →  3.4% (avg 2.5%)
  Resp time:    12.0ms →  14.0ms (avg 12.7ms)
  ✓ Failures:  0/4 (0%)

Servo #3 (elbow_flex) [XL330]:
  Temperature:  23°C →  23°C (avg 23.0°C)
  Voltage:     5.20V → 5.20V (avg 5.20V)
  Current:      -54mA →    0mA (avg -40mA)
  Resp time:     7.0ms →   8.9ms (avg 7.5ms)
  ✓ Failures:  0/4 (0%)

Servo #4 (wrist_flex) [XL330]:
  Temperature:  20°C →  20°C (avg 20.0°C)
  Voltage:     5.20V → 5.20V (avg 5.20V)
  Current:        0mA →    0mA (avg 0mA)
  Resp time:     6.0ms →  10.0ms (avg 7.7ms)
  ✓ Failures:  0/4 (0%)

Servo #5 (wrist_roll) [XL330]:
  Temperature:  20°C →  20°C (avg 20.0°C)
  Voltage:     5.20V → 5.20V (avg 5.20V)
  Current:        0mA →    0mA (avg 0mA)
  Resp time:     7.0ms →  10.0ms (avg 8.2ms)
  ✓ Failures:  0/4 (0%)

Servo #6 (pen_holder) [XL330]:
  Temperature:  22°C →  22°C (avg 22.0°C)
  Voltage:     5.20V → 5.20V (avg 5.20V)
  Current:        0mA →    0mA (avg 0mA)
  Resp time:     7.0ms →  10.0ms (avg 8.5ms)
  ✓ Failures:  0/4 (0%)

✓ Monitoring stopped
```
