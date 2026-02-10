# Running the Pose Test utility

This ros2 utility does a number of tests that include:
1. running through a sequence of pre-defined poses
2. perform range of motion tests of defined joints in the robot system
3. measure various properties of the robot in the current pose (servo temperature, current, voltage, etc)
4. validate poses wrt to the bounds defined in the urdf file

## Table of contents
- [Parameters](#parameters)
- [Pose Sequence Example](#pose-sequence-example)
- [Servo Range of motion test example](#servo-range-of-motion-test-example)
- [Example of a validation error](#example-of-a-validation-error)
- [FAQ](#faq)


## Parameters

The parameters are:
| parameter name | default | description |
|----------------|---------|-------------|
|`poses_file`| |The argument should be the file path to a yaml formatted set of poses. See [example pose sequence yaml file](https://github.com/bueche/ros2_robot_arm/blob/main/writing_robot_description/config/delivery_poses.yaml) for more detail.|
| `program`|| This can have an argument value of calibration, sweep, stress, or default. Each represents a sequence of poses.|
| `servo_test`| False| if true then a servo range-of-motion test is run. Must be accompanied by the `servo_test_joint` parameter (see next).|
| `servo_test_joint`|| Specific joint to test and can have a value of shoulder_pan, shoulder_lift, elbow_flex, elbow_flex, wrist_roll, and pen_holder.|
| `safe_position_first`| `True`| should the arm position itself first in the safe pose?|
| `validate`| `True`| should the program check to ensure that the next pose is safe or not (within the bounds specified in the urdf)|
| `telemetry`| `False`| should the servo stats be sampled after each pose? the stats include temperature, current, voltage, etc.|
| `delay`|`2.0`| The time in seconds to delay between each pose.|
| `movement_time`| `2.0`| how much time should the arm have to complete the transition from the current pose to the next pose.|
| `urdf_file`| |Load URDF from file. (TODO: should have a default file to load)|
| `power_monitoring`| |Determines whether or not the power monitoring will take place during poses. See [power_monitor_node](./writing_robot_control/writing_robot_comtrol/power_monitor.py).
  

## Pose Sequence Example
An example run through a sequence of poses is as follows. In this test the robot will go through a short simulated package handling scenario. Each pose transition is to occur within one second and there will be a one second pause between each pose. The software will also validate the that each pose is allowed by the URDF definition and stop if it does not. 

The particular example illustrated is defined by [this pose sequence](../src/writing_robot_description/config/delivery_poses.yaml) and is illustrated in the story board below (which was an rviz2 reproduction of the actual physical robot movements). 

<p align="center">
  <img src="../images/story_board_for_example_pose_test.jpg" alt="story board for example pose test" width="900">
</p>

Ouptut from the Pose Test utility is given below.

```bash
ubuntu@bueche-rpi5:~/robot_ws$ ros2 run writing_robot_control pose_test --ros-args -p poses_file:=src/writing_robot_description/config/delivery_poses.yaml -p validate:=true -p telemetry:=true -p urdf_file:=src/writing_robot_description/urdf/koch_v11_arm_real.urdf -p power_monitoring:=true
POWER_MONITORING_AVAILABLEF: True
[INFO] [1768185812.878629696] [pose_test_node]: Loading URDF limits...
[INFO] [1768185812.880739114] [pose_test_node]: ✓ Loaded limits for 6 joints
[INFO] [1768185812.881362043] [pose_test_node]: Initializing telemetry...
[INFO] [1768185812.893764124] [pose_test_node]: ✓ Telemetry initialized (dual-topic mode)
[INFO] [1768185812.894397923] [pose_test_node]:   - Subscribing to /joint_states
[INFO] [1768185812.894981260] [pose_test_node]:   - Subscribing to /dxl_state
[INFO] [1768185812.895531633] [pose_test_node]:   - XL430 servos (IDs 1,2): Load only
[INFO] [1768185812.896093525] [pose_test_node]:   - XL330 servos (IDs 3-6): Current + Load
[INFO] [1768185812.902823040] [pose_test_node]: ✓ Telemetry active
[INFO] [1768185812.907117099] [pose_test_node]: ✓ Power monitoring enabled
[INFO] [1768185812.907755750] [pose_test_node]: ============================================================
[INFO] [1768185812.908277160] [pose_test_node]: POSE TEST NODE INITIALIZED
[INFO] [1768185812.908843811] [pose_test_node]: ============================================================
[INFO] [1768185812.909380295] [pose_test_node]: Joints: 6
[INFO] [1768185812.909956280] [pose_test_node]: Validation: True
[INFO] [1768185812.910545023] [pose_test_node]: Telemetry: True
[INFO] [1768185812.911091285] [pose_test_node]: Power Monitoring: Enabled
[INFO] [1768185812.911670066] [pose_test_node]: Movement time: 2.0s
[INFO] [1768185812.912324958] [pose_test_node]: Delay between poses: 2.0s
[INFO] [1768185812.912926406] [pose_test_node]: ============================================================
[INFO] [1768185812.913602280] [pose_test_node]: Waiting for joint trajectory controller...
[INFO] [1768185814.936714057] [pose_test_node]: ✓ Loaded 11 poses from src/writing_robot_description/config/delivery_poses.yaml
[INFO] [1768185814.937424301] [pose_test_node]: 
============================================================
[INFO] [1768185814.938000378] [pose_test_node]: POSE SEQUENCE TEST
[INFO] [1768185814.938578362] [pose_test_node]: ============================================================
[INFO] [1768185814.939086754] [pose_test_node]: Total poses: 11
[INFO] [1768185814.939596238] [pose_test_node]: Validation: True
[INFO] [1768185814.940142722] [pose_test_node]: Telemetry: True
[INFO] [1768185814.940672021] [pose_test_node]: ============================================================
[INFO] [1768185814.941236746] [pose_test_node]: 
[1/11] pose 1 - poised to work
[INFO] [1768185814.941892416] [pose_test_node]:   ✓ shoulder_pan: 1.558 rad (within [0.540, 2.044])
[INFO] [1768185814.942582401] [pose_test_node]:   ✓ shoulder_lift: 2.879 rad (within [2.200, 2.886])
[INFO] [1768185814.943108145] [pose_test_node]:   ✓ elbow_flex: 2.124 rad (within [0.726, 2.250])
[INFO] [1768185814.943600629] [pose_test_node]:   ✓ wrist_flex: 2.279 rad (within [0.297, 2.700])
[INFO] [1768185814.944069594] [pose_test_node]:   ✓ wrist_roll: 1.182 rad (within [-1.448, 1.900])
[INFO] [1768185814.944546022] [pose_test_node]:   ✓ pen_holder: 1.590 rad (within [0.190, 1.600])
[INFO] [1768185814.945650231] [pose_test_node]: → Sent: pose 1 - poised to work
[INFO] [1768185818.949158015] [pose_test_node]:   📊 Joint States:
[INFO] [1768185818.950222131] [pose_test_node]:   📊 shoulder_pan    [XL430] Pos:  1.564rad Load: -2.40% Volt:12.2V Temp: 33.0°C
[INFO] [1768185818.951038598] [pose_test_node]:   📊 shoulder_lift   [XL430] Pos:  2.835rad Load: 16.30% Volt:12.1V Temp: 41.0°C
[INFO] [1768185818.951867454] [pose_test_node]:   📊 elbow_flex      [XL330] Pos:  2.202rad Curr:  -80mA Volt: 5.6V Temp: 20.0°C
[INFO] [1768185818.952823311] [pose_test_node]:   📊 wrist_flex      [XL330] Pos:  2.262rad Curr:   43mA Volt: 5.5V Temp: 19.0°C
[INFO] [1768185818.953637000] [pose_test_node]:   📊 wrist_roll      [XL330] Pos:  1.188rad Curr:  -32mA Volt: 5.6V Temp: 19.0°C
[INFO] [1768185818.954303504] [pose_test_node]:   📊 pen_holder      [XL330] Pos:  1.552rad Curr:   22mA Volt: 5.5V Temp: 22.0°C
[INFO] [1768185818.955140045] [pose_test_node]:   ⚡ Power Analysis for "pose 1 - poised to work":
[INFO] [1768185818.955875938] [pose_test_node]:      Peak 5V current (INA219):    0.128A
[INFO] [1768185818.956547089] [pose_test_node]:      Motor current sum (XL330s):  0.177A (at end)
[INFO] [1768185818.957165870] [pose_test_node]:      5V voltage: 5.68V (min) / 5.73V (max)
[INFO] [1768185818.957809225] [pose_test_node]:      Peak 12V current:            0.153A
[INFO] [1768185818.958449025] [pose_test_node]:      Peak total power:            2.57W
[INFO] [1768185818.959102195] [pose_test_node]:      Holding current comparison:
[INFO] [1768185818.959756772] [pose_test_node]:        Supply current (INA219):     0.127A
[INFO] [1768185818.960390312] [pose_test_node]:        Motor current sum (XL330s):  0.177A
[INFO] [1768185818.961032167] [pose_test_node]:        Controller benefit:          0.050A (28% efficiency)
[INFO] [1768185818.961708689] [pose_test_node]: 
[2/11] pose 2 - reach to grab
[INFO] [1768185818.962500823] [pose_test_node]:   ✓ shoulder_pan: 1.283 rad (within [0.540, 2.044])
[INFO] [1768185818.963182789] [pose_test_node]:   ✓ shoulder_lift: 2.314 rad (within [2.200, 2.886])
[INFO] [1768185818.963775255] [pose_test_node]:   ✓ elbow_flex: 1.320 rad (within [0.726, 2.250])
[INFO] [1768185818.964259221] [pose_test_node]:   ✓ wrist_flex: 2.279 rad (within [0.297, 2.700])
[INFO] [1768185818.964750890] [pose_test_node]:   ✓ wrist_roll: 1.871 rad (within [-1.448, 1.900])
[INFO] [1768185818.965226892] [pose_test_node]:   ✓ pen_holder: 0.982 rad (within [0.190, 1.600])
[INFO] [1768185818.966257471] [pose_test_node]: → Sent: pose 2 - reach to grab
[INFO] [1768185822.969187733] [pose_test_node]:   📊 Joint States:
[INFO] [1768185822.970164405] [pose_test_node]:   📊 shoulder_pan    [XL430] Pos:  1.291rad Load: -2.80% Volt:12.2V Temp: 33.0°C
[INFO] [1768185822.970998094] [pose_test_node]:   📊 shoulder_lift   [XL430] Pos:  2.293rad Load:  7.30% Volt:12.1V Temp: 41.0°C
[INFO] [1768185822.971942025] [pose_test_node]:   📊 elbow_flex      [XL330] Pos:  1.524rad Curr: -344mA Volt: 5.4V Temp: 20.0°C
[INFO] [1768185822.972769362] [pose_test_node]:   📊 wrist_flex      [XL330] Pos:  2.261rad Curr:   45mA Volt: 5.3V Temp: 19.0°C
[INFO] [1768185822.973525014] [pose_test_node]:   📊 wrist_roll      [XL330] Pos:  1.866rad Curr:   32mA Volt: 5.3V Temp: 19.0°C
[INFO] [1768185822.974186777] [pose_test_node]:   📊 pen_holder      [XL330] Pos:  1.042rad Curr:  -31mA Volt: 5.3V Temp: 22.0°C
[INFO] [1768185822.974997337] [pose_test_node]:   ⚡ Power Analysis for "pose 2 - reach to grab":
[INFO] [1768185822.975637895] [pose_test_node]:      Peak 5V current (INA219):    0.253A
[INFO] [1768185822.976251491] [pose_test_node]:      Motor current sum (XL330s):  0.452A (at end)
[INFO] [1768185822.976881346] [pose_test_node]:      5V voltage: 5.50V (min) / 5.72V (max)
[INFO] [1768185822.977519683] [pose_test_node]:      Peak 12V current:            0.148A
[INFO] [1768185822.978136482] [pose_test_node]:      Peak total power:            2.83W
[INFO] [1768185822.978799319] [pose_test_node]:      Holding current comparison:
[INFO] [1768185822.979444581] [pose_test_node]:        Supply current (INA219):     0.229A
[INFO] [1768185822.980061436] [pose_test_node]:        Motor current sum (XL330s):  0.452A
[INFO] [1768185822.980713791] [pose_test_node]:        Controller benefit:          0.223A (49% efficiency)
[INFO] [1768185822.981482832] [pose_test_node]: 
[3/11] pose 3 - grab
[INFO] [1768185822.982268299] [pose_test_node]:   ✓ shoulder_pan: 1.329 rad (within [0.540, 2.044])
[INFO] [1768185822.982941025] [pose_test_node]:   ✓ shoulder_lift: 2.322 rad (within [2.200, 2.886])
[INFO] [1768185822.983579732] [pose_test_node]:   ✓ elbow_flex: 1.005 rad (within [0.726, 2.250])
[INFO] [1768185822.984071734] [pose_test_node]:   ✓ wrist_flex: 2.279 rad (within [0.297, 2.700])
[INFO] [1768185822.984567570] [pose_test_node]:   ✓ wrist_roll: 1.766 rad (within [-1.448, 1.900])
[INFO] [1768185822.985055850] [pose_test_node]:   ✓ pen_holder: 1.501 rad (within [0.190, 1.600])
[INFO] [1768185822.986153578] [pose_test_node]: → Sent: pose 3 - grab
[INFO] [1768185826.995639873] [pose_test_node]:   📊 Joint States:
[INFO] [1768185826.996537285] [pose_test_node]:   📊 shoulder_pan    [XL430] Pos:  1.329rad Load:  0.00% Volt:12.1V Temp: 33.0°C
[INFO] [1768185826.997365345] [pose_test_node]:   📊 shoulder_lift   [XL430] Pos:  2.282rad Load: 14.60% Volt:12.1V Temp: 41.0°C
[INFO] [1768185826.998173145] [pose_test_node]:   📊 elbow_flex      [XL330] Pos:  1.274rad Curr: -527mA Volt: 5.3V Temp: 20.0°C
[INFO] [1768185826.998947001] [pose_test_node]:   📊 wrist_flex      [XL330] Pos:  2.261rad Curr:   43mA Volt: 5.2V Temp: 19.0°C
[INFO] [1768185826.999710060] [pose_test_node]:   📊 wrist_roll      [XL330] Pos:  1.766rad Curr:    0mA Volt: 5.2V Temp: 19.0°C
[INFO] [1768185827.001078104] [pose_test_node]:   📊 pen_holder      [XL330] Pos:  1.467rad Curr:   19mA Volt: 5.2V Temp: 22.0°C
[INFO] [1768185827.002191239] [pose_test_node]:   ⚡ Power Analysis for "pose 3 - grab":
[INFO] [1768185827.003331467] [pose_test_node]:      Peak 5V current (INA219):    0.349A
[INFO] [1768185827.004149583] [pose_test_node]:      Motor current sum (XL330s):  0.589A (at end)
[INFO] [1768185827.004898198] [pose_test_node]:      5V voltage: 5.36V (min) / 5.73V (max)
[INFO] [1768185827.005622979] [pose_test_node]:      Peak 12V current:            0.138A
[INFO] [1768185827.006307371] [pose_test_node]:      Peak total power:            3.41W
[INFO] [1768185827.007010856] [pose_test_node]:      Holding current comparison:
[INFO] [1768185827.007686378] [pose_test_node]:        Supply current (INA219):     0.281A
[INFO] [1768185827.008221418] [pose_test_node]:        Motor current sum (XL330s):  0.589A
[INFO] [1768185827.008759902] [pose_test_node]:        Controller benefit:          0.308A (52% efficiency)
[INFO] [1768185827.009291701] [pose_test_node]: 
[4/11] pose 4 - lift and center
[INFO] [1768185827.009828000] [pose_test_node]:   ✓ shoulder_pan: 1.576 rad (within [0.540, 2.044])
[INFO] [1768185827.010348781] [pose_test_node]:   ✓ shoulder_lift: 2.575 rad (within [2.200, 2.886])
[INFO] [1768185827.010901728] [pose_test_node]:   ✓ elbow_flex: 1.898 rad (within [0.726, 2.250])
[INFO] [1768185827.011548287] [pose_test_node]:   ✓ wrist_flex: 2.279 rad (within [0.297, 2.700])
[INFO] [1768185827.012223105] [pose_test_node]:   ✓ wrist_roll: 1.766 rad (within [-1.448, 1.900])
[INFO] [1768185827.012791256] [pose_test_node]:   ✓ pen_holder: 1.521 rad (within [0.190, 1.600])
[INFO] [1768185827.013907465] [pose_test_node]: → Sent: pose 4 - lift and center
[INFO] [1768185831.024413432] [pose_test_node]:   📊 Joint States:
[INFO] [1768185831.025349215] [pose_test_node]:   📊 shoulder_pan    [XL430] Pos:  1.576rad Load:  0.00% Volt:12.1V Temp: 33.0°C
[INFO] [1768185831.026158700] [pose_test_node]:   📊 shoulder_lift   [XL430] Pos:  2.522rad Load: 19.10% Volt:12.1V Temp: 41.0°C
[INFO] [1768185831.026930130] [pose_test_node]:   📊 elbow_flex      [XL330] Pos:  1.931rad Curr:  -32mA Volt: 5.6V Temp: 20.0°C
[INFO] [1768185831.027708671] [pose_test_node]:   📊 wrist_flex      [XL330] Pos:  2.261rad Curr:   45mA Volt: 5.6V Temp: 19.0°C
[INFO] [1768185831.028499582] [pose_test_node]:   📊 wrist_roll      [XL330] Pos:  1.766rad Curr:    0mA Volt: 5.6V Temp: 19.0°C
[INFO] [1768185831.029225104] [pose_test_node]:   📊 pen_holder      [XL330] Pos:  1.470rad Curr:   26mA Volt: 5.5V Temp: 22.0°C
[INFO] [1768185831.030122942] [pose_test_node]:   ⚡ Power Analysis for "pose 4 - lift and center":
[INFO] [1768185831.030851261] [pose_test_node]:      Peak 5V current (INA219):    0.282A
[INFO] [1768185831.031636246] [pose_test_node]:      Motor current sum (XL330s):  0.103A (at end)
[INFO] [1768185831.032460065] [pose_test_node]:      5V voltage: 5.46V (min) / 5.75V (max)
[INFO] [1768185831.033073550] [pose_test_node]:      Peak 12V current:            0.336A
[INFO] [1768185831.033639182] [pose_test_node]:      Peak total power:            4.71W
[INFO] [1768185831.034192722] [pose_test_node]:      Holding current comparison:
[INFO] [1768185831.034810670] [pose_test_node]:        Supply current (INA219):     0.109A
[INFO] [1768185831.035366358] [pose_test_node]:        Motor current sum (XL330s):  0.103A
[INFO] [1768185831.035903194] [pose_test_node]:        ✓ Supply matches motor sum
[INFO] [1768185831.036451141] [pose_test_node]: 
[5/11] pose 5 - raise and get ready to place
[INFO] [1768185831.036984273] [pose_test_node]:   ✓ shoulder_pan: 1.596 rad (within [0.540, 2.044])
[INFO] [1768185831.037559424] [pose_test_node]:   ✓ shoulder_lift: 2.795 rad (within [2.200, 2.886])
[INFO] [1768185831.038137409] [pose_test_node]:   ✓ elbow_flex: 0.970 rad (within [0.726, 2.250])
[INFO] [1768185831.038691726] [pose_test_node]:   ✓ wrist_flex: 1.473 rad (within [0.297, 2.700])
[INFO] [1768185831.039245951] [pose_test_node]:   ✓ wrist_roll: 1.762 rad (within [-1.448, 1.900])
[INFO] [1768185831.039794436] [pose_test_node]:   ✓ pen_holder: 1.521 rad (within [0.190, 1.600])
[INFO] [1768185831.040941515] [pose_test_node]: → Sent: pose 5 - raise and get ready to place
[INFO] [1768185835.051445982] [pose_test_node]:   📊 Joint States:
[INFO] [1768185835.052346598] [pose_test_node]:   📊 shoulder_pan    [XL430] Pos:  1.587rad Load:  3.90% Volt:12.1V Temp: 33.0°C
[INFO] [1768185835.053624345] [pose_test_node]:   📊 shoulder_lift   [XL430] Pos:  2.738rad Load: 20.20% Volt:12.1V Temp: 41.0°C
[INFO] [1768185835.054680739] [pose_test_node]:   📊 elbow_flex      [XL330] Pos:  1.231rad Curr: -481mA Volt: 5.3V Temp: 20.0°C
[INFO] [1768185835.055674226] [pose_test_node]:   📊 wrist_flex      [XL330] Pos:  1.466rad Curr:   37mA Volt: 5.2V Temp: 19.0°C
[INFO] [1768185835.056732416] [pose_test_node]:   📊 wrist_roll      [XL330] Pos:  1.763rad Curr:  -29mA Volt: 5.3V Temp: 19.0°C
[INFO] [1768185835.057764662] [pose_test_node]:   📊 pen_holder      [XL330] Pos:  1.470rad Curr:   25mA Volt: 5.2V Temp: 22.0°C
[INFO] [1768185835.058709722] [pose_test_node]:   ⚡ Power Analysis for "pose 5 - raise and get ready to place":
[INFO] [1768185835.059443819] [pose_test_node]:      Peak 5V current (INA219):    0.356A
[INFO] [1768185835.060121341] [pose_test_node]:      Motor current sum (XL330s):  0.572A (at end)
[INFO] [1768185835.060795844] [pose_test_node]:      5V voltage: 5.34V (min) / 5.75V (max)
[INFO] [1768185835.061460755] [pose_test_node]:      Peak 12V current:            0.296A
[INFO] [1768185835.062151351] [pose_test_node]:      Peak total power:            4.88W
[INFO] [1768185835.062935151] [pose_test_node]:      Holding current comparison:
[INFO] [1768185835.063786322] [pose_test_node]:        Supply current (INA219):     0.271A
[INFO] [1768185835.064482733] [pose_test_node]:        Motor current sum (XL330s):  0.572A
[INFO] [1768185835.065131514] [pose_test_node]:        Controller benefit:          0.301A (53% efficiency)
[INFO] [1768185835.065662961] [pose_test_node]: 
[6/11] pose 6 - place high
[INFO] [1768185835.066134982] [pose_test_node]:   ✓ shoulder_pan: 1.598 rad (within [0.540, 2.044])
[INFO] [1768185835.066599503] [pose_test_node]:   ✓ shoulder_lift: 2.655 rad (within [2.200, 2.886])
[INFO] [1768185835.067084209] [pose_test_node]:   ✓ elbow_flex: 0.806 rad (within [0.726, 2.250])
[INFO] [1768185835.067589508] [pose_test_node]:   ✓ wrist_flex: 1.473 rad (within [0.297, 2.700])
[INFO] [1768185835.068075251] [pose_test_node]:   ✓ wrist_roll: 1.762 rad (within [-1.448, 1.900])
[INFO] [1768185835.068611069] [pose_test_node]:   ✓ pen_holder: 0.655 rad (within [0.190, 1.600])
[INFO] [1768185835.069646648] [pose_test_node]: → Sent: pose 6 - place high
[INFO] [1768185839.080669673] [pose_test_node]:   📊 Joint States:
[INFO] [1768185839.081595363] [pose_test_node]:   📊 shoulder_pan    [XL430] Pos:  1.588rad Load:  3.90% Volt:12.1V Temp: 33.0°C
[INFO] [1768185839.082762961] [pose_test_node]:   📊 shoulder_lift   [XL430] Pos:  2.650rad Load:  1.30% Volt:12.1V Temp: 41.0°C
[INFO] [1768185839.083626021] [pose_test_node]:   📊 elbow_flex      [XL330] Pos:  1.105rad Curr: -613mA Volt: 5.2V Temp: 21.0°C
[INFO] [1768185839.084490211] [pose_test_node]:   📊 wrist_flex      [XL330] Pos:  1.464rad Curr:   40mA Volt: 5.1V Temp: 19.0°C
[INFO] [1768185839.085244881] [pose_test_node]:   📊 wrist_roll      [XL330] Pos:  1.762rad Curr:    5mA Volt: 5.2V Temp: 19.0°C
[INFO] [1768185839.085982848] [pose_test_node]:   📊 pen_holder      [XL330] Pos:  0.723rad Curr:  -31mA Volt: 5.1V Temp: 22.0°C
[INFO] [1768185839.086868389] [pose_test_node]:   ⚡ Power Analysis for "pose 6 - place high":
[INFO] [1768185839.087633449] [pose_test_node]:      Peak 5V current (INA219):    0.450A
[INFO] [1768185839.088314156] [pose_test_node]:      Motor current sum (XL330s):  0.689A (at end)
[INFO] [1768185839.088943567] [pose_test_node]:      5V voltage: 5.20V (min) / 5.72V (max)
[INFO] [1768185839.089555718] [pose_test_node]:      Peak 12V current:            0.166A
[INFO] [1768185839.090145239] [pose_test_node]:      Peak total power:            3.66W
[INFO] [1768185839.090749261] [pose_test_node]:      Holding current comparison:
[INFO] [1768185839.091308838] [pose_test_node]:        Supply current (INA219):     0.323A
[INFO] [1768185839.091901618] [pose_test_node]:        Motor current sum (XL330s):  0.689A
[INFO] [1768185839.092665419] [pose_test_node]:        Controller benefit:          0.366A (53% efficiency)
[INFO] [1768185839.093291737] [pose_test_node]: 
[7/11] pose 7 - reach to grab again
[INFO] [1768185839.093891054] [pose_test_node]:   ✓ shoulder_pan: 1.283 rad (within [0.540, 2.044])
[INFO] [1768185839.094493483] [pose_test_node]:   ✓ shoulder_lift: 2.314 rad (within [2.200, 2.886])
[INFO] [1768185839.095074375] [pose_test_node]:   ✓ elbow_flex: 1.320 rad (within [0.726, 2.250])
[INFO] [1768185839.095679212] [pose_test_node]:   ✓ wrist_flex: 2.327 rad (within [0.297, 2.700])
[INFO] [1768185839.096293474] [pose_test_node]:   ✓ wrist_roll: 1.871 rad (within [-1.448, 1.900])
[INFO] [1768185839.096899477] [pose_test_node]:   ✓ pen_holder: 0.982 rad (within [0.190, 1.600])
[INFO] [1768185839.098191521] [pose_test_node]: → Sent: pose 7 - reach to grab again
[INFO] [1768185843.100491520] [pose_test_node]:   📊 Joint States:
[INFO] [1768185843.101416080] [pose_test_node]:   📊 shoulder_pan    [XL430] Pos:  1.291rad Load: -2.80% Volt:12.2V Temp: 33.0°C
[INFO] [1768185843.102925162] [pose_test_node]:   📊 shoulder_lift   [XL430] Pos:  2.293rad Load:  7.30% Volt:12.1V Temp: 41.0°C
[INFO] [1768185843.103826481] [pose_test_node]:   📊 elbow_flex      [XL330] Pos:  1.381rad Curr:  -59mA Volt: 5.6V Temp: 21.0°C
[INFO] [1768185843.104513300] [pose_test_node]:   📊 wrist_flex      [XL330] Pos:  2.261rad Curr:   86mA Volt: 5.5V Temp: 19.0°C
[INFO] [1768185843.105155099] [pose_test_node]:   📊 wrist_roll      [XL330] Pos:  1.865rad Curr:   32mA Volt: 5.6V Temp: 19.0°C
[INFO] [1768185843.105814825] [pose_test_node]:   📊 pen_holder      [XL330] Pos:  0.944rad Curr:   22mA Volt: 5.5V Temp: 22.0°C
[INFO] [1768185843.106584810] [pose_test_node]:   ⚡ Power Analysis for "pose 7 - reach to grab again":
[INFO] [1768185843.107181850] [pose_test_node]:      Peak 5V current (INA219):    0.322A
[INFO] [1768185843.107830557] [pose_test_node]:      Motor current sum (XL330s):  0.199A (at end)
[INFO] [1768185843.108457208] [pose_test_node]:      5V voltage: 5.40V (min) / 5.72V (max)
[INFO] [1768185843.109046378] [pose_test_node]:      Peak 12V current:            0.124A
[INFO] [1768185843.109638140] [pose_test_node]:      Peak total power:            3.16W
[INFO] [1768185843.110252273] [pose_test_node]:      Holding current comparison:
[INFO] [1768185843.110880276] [pose_test_node]:        Supply current (INA219):     0.136A
[INFO] [1768185843.111633687] [pose_test_node]:        Motor current sum (XL330s):  0.199A
[INFO] [1768185843.112281117] [pose_test_node]:        Controller benefit:          0.063A (32% efficiency)
[INFO] [1768185843.112904194] [pose_test_node]: 
[8/11] pose 8 - grab
[INFO] [1768185843.113505697] [pose_test_node]:   ✓ shoulder_pan: 1.329 rad (within [0.540, 2.044])
[INFO] [1768185843.113985310] [pose_test_node]:   ✓ shoulder_lift: 2.314 rad (within [2.200, 2.886])
[INFO] [1768185843.114530313] [pose_test_node]:   ✓ elbow_flex: 1.005 rad (within [0.726, 2.250])
[INFO] [1768185843.115021741] [pose_test_node]:   ✓ wrist_flex: 2.037 rad (within [0.297, 2.700])
[INFO] [1768185843.115498781] [pose_test_node]:   ✓ wrist_roll: 1.766 rad (within [-1.448, 1.900])
[INFO] [1768185843.115984246] [pose_test_node]:   ✓ pen_holder: 1.521 rad (within [0.190, 1.600])
[INFO] [1768185843.117041641] [pose_test_node]: → Sent: pose 8 - grab
[INFO] [1768185847.119906698] [pose_test_node]:   📊 Joint States:
[INFO] [1768185847.120991926] [pose_test_node]:   📊 shoulder_pan    [XL430] Pos:  1.329rad Load:  0.00% Volt:12.1V Temp: 33.0°C
[INFO] [1768185847.121821282] [pose_test_node]:   📊 shoulder_lift   [XL430] Pos:  2.282rad Load: 11.20% Volt:12.1V Temp: 41.0°C
[INFO] [1768185847.122747713] [pose_test_node]:   📊 elbow_flex      [XL330] Pos:  1.429rad Curr:-1043mA Volt: 4.8V Temp: 21.0°C
[INFO] [1768185847.123627143] [pose_test_node]:   📊 wrist_flex      [XL330] Pos:  2.034rad Curr:   37mA Volt: 4.9V Temp: 19.0°C
[INFO] [1768185847.124374906] [pose_test_node]:   📊 wrist_roll      [XL330] Pos:  1.766rad Curr:    0mA Volt: 4.8V Temp: 19.0°C
[INFO] [1768185847.125102799] [pose_test_node]:   📊 pen_holder      [XL330] Pos:  1.472rad Curr:   23mA Volt: 4.8V Temp: 22.0°C
[INFO] [1768185847.125904043] [pose_test_node]:   ⚡ Power Analysis for "pose 8 - grab":
[INFO] [1768185847.126542732] [pose_test_node]:      Peak 5V current (INA219):    0.477A
[INFO] [1768185847.127159828] [pose_test_node]:      Motor current sum (XL330s):  1.103A (at end)
[INFO] [1768185847.127804849] [pose_test_node]:      5V voltage: 5.17V (min) / 5.73V (max)
[INFO] [1768185847.128483464] [pose_test_node]:      Peak 12V current:            0.132A
[INFO] [1768185847.129125837] [pose_test_node]:      Peak total power:            4.03W
[INFO] [1768185847.129760267] [pose_test_node]:      Holding current comparison:
[INFO] [1768185847.130366621] [pose_test_node]:        Supply current (INA219):     0.468A
[INFO] [1768185847.130981810] [pose_test_node]:        Motor current sum (XL330s):  1.103A
[INFO] [1768185847.131630665] [pose_test_node]:        Controller benefit:          0.635A (58% efficiency)
[INFO] [1768185847.132311483] [pose_test_node]: 
[9/11] pose 9 - place low
[INFO] [1768185847.133149710] [pose_test_node]:   ✓ shoulder_pan: 1.925 rad (within [0.540, 2.044])
[INFO] [1768185847.133864065] [pose_test_node]:   ✓ shoulder_lift: 2.403 rad (within [2.200, 2.886])
[INFO] [1768185847.134521217] [pose_test_node]:   ✓ elbow_flex: 1.352 rad (within [0.726, 2.250])
[INFO] [1768185847.135037071] [pose_test_node]:   ✓ wrist_flex: 2.327 rad (within [0.297, 2.700])
[INFO] [1768185847.135545129] [pose_test_node]:   ✓ wrist_roll: 1.737 rad (within [-1.448, 1.900])
[INFO] [1768185847.136017502] [pose_test_node]:   ✓ pen_holder: 1.521 rad (within [0.190, 1.600])
[INFO] [1768185847.137066433] [pose_test_node]: → Sent: pose 9 - place low
[INFO] [1768185851.148527071] [pose_test_node]:   📊 Joint States:
[INFO] [1768185851.149433039] [pose_test_node]:   📊 shoulder_pan    [XL430] Pos:  1.918rad Load:  1.90% Volt:12.1V Temp: 33.0°C
[INFO] [1768185851.150255932] [pose_test_node]:   📊 shoulder_lift   [XL430] Pos:  2.342rad Load: 22.50% Volt:12.2V Temp: 41.0°C
[INFO] [1768185851.151069695] [pose_test_node]:   📊 elbow_flex      [XL330] Pos:  1.473rad Curr: -145mA Volt: 5.5V Temp: 21.0°C
[INFO] [1768185851.151882755] [pose_test_node]:   📊 wrist_flex      [XL330] Pos:  2.261rad Curr:   86mA Volt: 5.4V Temp: 19.0°C
[INFO] [1768185851.152755648] [pose_test_node]:   📊 wrist_roll      [XL330] Pos:  1.742rad Curr:  -32mA Volt: 5.5V Temp: 19.0°C
[INFO] [1768185851.153749172] [pose_test_node]:   📊 pen_holder      [XL330] Pos:  1.472rad Curr:   25mA Volt: 5.4V Temp: 22.0°C
[INFO] [1768185851.154694899] [pose_test_node]:   ⚡ Power Analysis for "pose 9 - place low":
[INFO] [1768185851.155369995] [pose_test_node]:      Peak 5V current (INA219):    0.468A
[INFO] [1768185851.156112073] [pose_test_node]:      Motor current sum (XL330s):  0.288A (at end)
[INFO] [1768185851.156782428] [pose_test_node]:      5V voltage: 5.18V (min) / 5.72V (max)
[INFO] [1768185851.157524098] [pose_test_node]:      Peak 12V current:            0.233A
[INFO] [1768185851.158189213] [pose_test_node]:      Peak total power:            3.86W
[INFO] [1768185851.158878698] [pose_test_node]:      Holding current comparison:
[INFO] [1768185851.159542053] [pose_test_node]:        Supply current (INA219):     0.168A
[INFO] [1768185851.160119482] [pose_test_node]:        Motor current sum (XL330s):  0.288A
[INFO] [1768185851.160623040] [pose_test_node]:        Controller benefit:          0.120A (42% efficiency)
[INFO] [1768185851.161096875] [pose_test_node]: 
[10/11] pose 10 - release
[INFO] [1768185851.161649971] [pose_test_node]:   ✓ shoulder_pan: 1.872 rad (within [0.540, 2.044])
[INFO] [1768185851.162277363] [pose_test_node]:   ✓ shoulder_lift: 2.403 rad (within [2.200, 2.886])
[INFO] [1768185851.162828403] [pose_test_node]:   ✓ elbow_flex: 1.266 rad (within [0.726, 2.250])
[INFO] [1768185851.163306461] [pose_test_node]:   ✓ wrist_flex: 2.327 rad (within [0.297, 2.700])
[INFO] [1768185851.163777130] [pose_test_node]:   ✓ wrist_roll: 1.737 rad (within [-1.448, 1.900])
[INFO] [1768185851.164260614] [pose_test_node]:   ✓ pen_holder: 0.582 rad (within [0.190, 1.600])
[INFO] [1768185851.165265119] [pose_test_node]: → Sent: pose 10 - release
[INFO] [1768185855.168191658] [pose_test_node]:   📊 Joint States:
[INFO] [1768185855.169125126] [pose_test_node]:   📊 shoulder_pan    [XL430] Pos:  1.880rad Load: -2.80% Volt:12.1V Temp: 33.0°C
[INFO] [1768185855.169992593] [pose_test_node]:   📊 shoulder_lift   [XL430] Pos:  2.342rad Load: 22.50% Volt:12.1V Temp: 41.0°C
[INFO] [1768185855.170834820] [pose_test_node]:   📊 elbow_flex      [XL330] Pos:  1.521rad Curr: -468mA Volt: 5.2V Temp: 21.0°C
[INFO] [1768185855.171677657] [pose_test_node]:   📊 wrist_flex      [XL330] Pos:  2.259rad Curr:   86mA Volt: 5.2V Temp: 19.0°C
[INFO] [1768185855.172666903] [pose_test_node]:   📊 wrist_roll      [XL330] Pos:  1.737rad Curr:    0mA Volt: 5.2V Temp: 19.0°C
[INFO] [1768185855.173549482] [pose_test_node]:   📊 pen_holder      [XL330] Pos:  0.652rad Curr:  -34mA Volt: 5.2V Temp: 22.0°C
[INFO] [1768185855.174455505] [pose_test_node]:   ⚡ Power Analysis for "pose 10 - release":
[INFO] [1768185855.175068915] [pose_test_node]:      Peak 5V current (INA219):    0.289A
[INFO] [1768185855.175713511] [pose_test_node]:      Motor current sum (XL330s):  0.588A (at end)
[INFO] [1768185855.176312922] [pose_test_node]:      5V voltage: 5.44V (min) / 5.72V (max)
[INFO] [1768185855.176896461] [pose_test_node]:      Peak 12V current:            0.189A
[INFO] [1768185855.177486057] [pose_test_node]:      Peak total power:            3.81W
[INFO] [1768185855.178090764] [pose_test_node]:      Holding current comparison:
[INFO] [1768185855.178714397] [pose_test_node]:        Supply current (INA219):     0.288A
[INFO] [1768185855.179281622] [pose_test_node]:        Motor current sum (XL330s):  0.588A
[INFO] [1768185855.179883236] [pose_test_node]:        Controller benefit:          0.300A (51% efficiency)
[INFO] [1768185855.180545017] [pose_test_node]: 
[11/11] pose 11 - poised to work
[INFO] [1768185855.181158464] [pose_test_node]:   ✓ shoulder_pan: 1.558 rad (within [0.540, 2.044])
[INFO] [1768185855.181807227] [pose_test_node]:   ✓ shoulder_lift: 2.879 rad (within [2.200, 2.886])
[INFO] [1768185855.182576527] [pose_test_node]:   ✓ elbow_flex: 2.124 rad (within [0.726, 2.250])
[INFO] [1768185855.183245938] [pose_test_node]:   ✓ wrist_flex: 2.279 rad (within [0.297, 2.700])
[INFO] [1768185855.183869700] [pose_test_node]:   ✓ wrist_roll: 1.182 rad (within [-1.448, 1.900])
[INFO] [1768185855.184390351] [pose_test_node]:   ✓ pen_holder: 1.590 rad (within [0.190, 1.600])
[INFO] [1768185855.185452597] [pose_test_node]: → Sent: pose 11 - poised to work
[INFO] [1768185859.186740851] [pose_test_node]:   📊 Joint States:
[INFO] [1768185859.188096284] [pose_test_node]:   📊 shoulder_pan    [XL430] Pos:  1.565rad Load: -2.80% Volt:12.2V Temp: 33.0°C
[INFO] [1768185859.188928955] [pose_test_node]:   📊 shoulder_lift   [XL430] Pos:  2.836rad Load: 15.80% Volt:12.1V Temp: 41.0°C
[INFO] [1768185859.189703958] [pose_test_node]:   📊 elbow_flex      [XL330] Pos:  2.169rad Curr:  -40mA Volt: 5.6V Temp: 21.0°C
[INFO] [1768185859.190443314] [pose_test_node]:   📊 wrist_flex      [XL330] Pos:  2.261rad Curr:   45mA Volt: 5.5V Temp: 19.0°C
[INFO] [1768185859.191165336] [pose_test_node]:   📊 wrist_roll      [XL330] Pos:  1.188rad Curr:  -32mA Volt: 5.6V Temp: 19.0°C
[INFO] [1768185859.191908562] [pose_test_node]:   📊 pen_holder      [XL330] Pos:  1.552rad Curr:   24mA Volt: 5.6V Temp: 22.0°C
[INFO] [1768185859.192971919] [pose_test_node]:   ⚡ Power Analysis for "pose 11 - poised to work":
[INFO] [1768185859.193744275] [pose_test_node]:      Peak 5V current (INA219):    0.287A
[INFO] [1768185859.194361593] [pose_test_node]:      Motor current sum (XL330s):  0.141A (at end)
[INFO] [1768185859.194938651] [pose_test_node]:      5V voltage: 5.44V (min) / 5.72V (max)
[INFO] [1768185859.195507932] [pose_test_node]:      Peak 12V current:            0.208A
[INFO] [1768185859.196076194] [pose_test_node]:      Peak total power:            3.70W
[INFO] [1768185859.196703623] [pose_test_node]:      Holding current comparison:
[INFO] [1768185859.197389701] [pose_test_node]:        Supply current (INA219):     0.113A
[INFO] [1768185859.198084686] [pose_test_node]:        Motor current sum (XL330s):  0.141A
[INFO] [1768185859.198693559] [pose_test_node]:        ✓ Supply matches motor sum
[INFO] [1768185859.199275303] [pose_test_node]: 
============================================================
[INFO] [1768185859.199863436] [pose_test_node]: SUMMARY
[INFO] [1768185859.200463402] [pose_test_node]: ============================================================
[INFO] [1768185859.201072256] [pose_test_node]: Poses tested: 11
[INFO] [1768185859.201765390] [pose_test_node]: Validation errors: 0
[INFO] [1768185859.202496004] [pose_test_node]: Validation warnings: 0
[INFO] [1768185859.203206934] [pose_test_node]: ============================================================
[INFO] [1768185859.203895252] [pose_test_node]: 
===========================================================================
[INFO] [1768185859.204397810] [pose_test_node]: 📊 TELEMETRY SUMMARY
[INFO] [1768185859.204916776] [pose_test_node]: ===========================================================================
[INFO] [1768185859.205429353] [pose_test_node]: XL430 (IDs 1-2): Avg Load: 6.5%
[INFO] [1768185859.205921077] [pose_test_node]: XL330 (IDs 3-6): Total Current: -3mA, Avg: -1mA (-0.0W @ 12V)
[INFO] [1768185859.206499784] [pose_test_node]: Max Temperature: 41.0°C
[INFO] [1768185859.206973786] [pose_test_node]: Voltage: 5.50V - 12.20V (avg 7.77V)
[INFO] [1768185859.207570715] [pose_test_node]: ===========================================================================
```

## Servo Range of motion test example
The following is a servo range of motion test example.
```bash
-rpi5:~/robot_ws$ ros2 run writing_robot_control pose_test --ros-args -p servo_test:=true -p servo_test_joint:=wrist_flex  -p validate:=true -p telemetry:=true -p urdf_file:=src/writing_robot_description/urdf/koch_v11_arm_real.urdf

[INFO] [1766531325.081504237] [pose_test_node]: Loading URDF limits...
[INFO] [1766531325.083479988] [pose_test_node]: ✓ Loaded limits for 6 joints
[INFO] [1766531325.084081010] [pose_test_node]: Initializing telemetry...
[INFO] [1766531325.090024356] [pose_test_node]: ✓ Telemetry initialized (dual-topic mode)
[INFO] [1766531325.090631452] [pose_test_node]:   - Subscribing to /joint_states
[INFO] [1766531325.091240066] [pose_test_node]:   - Subscribing to /dxl_state
[INFO] [1766531325.091755143] [pose_test_node]:   - XL430 servos (IDs 1,2): Load only
[INFO] [1766531325.092275071] [pose_test_node]:   - XL330 servos (IDs 3-6): Current + Load
[INFO] [1766531325.105151491] [pose_test_node]: ✓ Telemetry active
[INFO] [1766531325.105910902] [pose_test_node]: ============================================================
[INFO] [1766531325.106616813] [pose_test_node]: POSE TEST NODE INITIALIZED
[INFO] [1766531325.107315298] [pose_test_node]: ============================================================
[INFO] [1766531325.108024598] [pose_test_node]: Joints: 6
[INFO] [1766531325.108750046] [pose_test_node]: Validation: True
[INFO] [1766531325.109473532] [pose_test_node]: Telemetry: True
[INFO] [1766531325.110177591] [pose_test_node]: Movement time: 2.0s
[INFO] [1766531325.110860243] [pose_test_node]: Delay between poses: 2.0s
[INFO] [1766531325.111544598] [pose_test_node]: ============================================================
[INFO] [1766531325.112241694] [pose_test_node]: Waiting for joint trajectory controller...
[INFO] [1766531327.123563036] [pose_test_node]: 
============================================================
[INFO] [1766531327.124607134] [pose_test_node]: MOVING TO SAFE POSITION
[INFO] [1766531327.125423008] [pose_test_node]: ============================================================
[INFO] [1766531327.126191994] [pose_test_node]:   ✓ shoulder_pan: 1.550 rad (within [0.540, 2.044])
[INFO] [1766531327.126972331] [pose_test_node]:   ✓ shoulder_lift: 2.700 rad (within [2.200, 2.886])
[INFO] [1766531327.127706465] [pose_test_node]:   ✓ elbow_flex: 1.200 rad (within [0.726, 2.250])
[INFO] [1766531327.128444117] [pose_test_node]:   ✓ wrist_flex: 1.500 rad (within [0.297, 2.700])
[INFO] [1766531327.129200602] [pose_test_node]:   ✓ wrist_roll: 0.000 rad (within [-1.448, 1.900])
[INFO] [1766531327.130002199] [pose_test_node]:   ✓ pen_holder: 0.900 rad (within [0.190, 1.600])
[INFO] [1766531327.131207668] [pose_test_node]: → Sent: safe_position
[INFO] [1766531330.132835276] [pose_test_node]: ✓ At safe position

[INFO] [1766531330.133694188] [pose_test_node]: 
============================================================
[INFO] [1766531330.134473488] [pose_test_node]: SERVO TEST: WRIST_FLEX
[INFO] [1766531330.135232844] [pose_test_node]: ============================================================
[INFO] [1766531330.135980792] [pose_test_node]: Range: [0.297, 2.700] rad
[INFO] [1766531330.136705111] [pose_test_node]: Middle: 1.499 rad
[INFO] [1766531330.137446967] [pose_test_node]: ============================================================
[INFO] [1766531330.138216249] [pose_test_node]: 
Testing min: 0.297 rad
[INFO] [1766531330.139037327] [pose_test_node]:   ✓ shoulder_pan: 1.550 rad (within [0.540, 2.044])
[INFO] [1766531330.139855109] [pose_test_node]:   ✓ shoulder_lift: 2.700 rad (within [2.200, 2.886])
[INFO] [1766531330.140645280] [pose_test_node]:   ✓ elbow_flex: 1.200 rad (within [0.726, 2.250])
[INFO] [1766531330.141326765] [pose_test_node]:   ✓ wrist_flex: 0.297 rad (within [0.297, 2.700])
[INFO] [1766531330.141962824] [pose_test_node]:   ✓ wrist_roll: 0.000 rad (within [-1.448, 1.900])
[INFO] [1766531330.142758143] [pose_test_node]:   ✓ pen_holder: 0.900 rad (within [0.190, 1.600])
[INFO] [1766531330.143763870] [pose_test_node]: → Sent: wrist_flex_min
[INFO] [1766531334.154675695] [pose_test_node]:   📊 wrist_flex      [XL330] Pos:  0.330rad Curr:  -48mA Volt: 5.0V Temp: 21.0°C
[INFO] [1766531334.155527422] [pose_test_node]: 
Testing middle: 1.499 rad
[INFO] [1766531334.156312389] [pose_test_node]:   ✓ shoulder_pan: 1.550 rad (within [0.540, 2.044])
[INFO] [1766531334.157141208] [pose_test_node]:   ✓ shoulder_lift: 2.700 rad (within [2.200, 2.886])
[INFO] [1766531334.157887601] [pose_test_node]:   ✓ elbow_flex: 1.200 rad (within [0.726, 2.250])
[INFO] [1766531334.158647290] [pose_test_node]:   ✓ wrist_flex: 1.499 rad (within [0.297, 2.700])
[INFO] [1766531334.159401924] [pose_test_node]:   ✓ wrist_roll: 0.000 rad (within [-1.448, 1.900])
[INFO] [1766531334.160153317] [pose_test_node]:   ✓ pen_holder: 0.900 rad (within [0.190, 1.600])
[INFO] [1766531334.161309452] [pose_test_node]: → Sent: wrist_flex_middle
[INFO] [1766531338.165583169] [pose_test_node]:   📊 wrist_flex      [XL330] Pos:  1.446rad Curr:   67mA Volt: 5.0V Temp: 21.0°C
[INFO] [1766531338.166378673] [pose_test_node]: 
Testing max: 2.700 rad
[INFO] [1766531338.167133306] [pose_test_node]:   ✓ shoulder_pan: 1.550 rad (within [0.540, 2.044])
[INFO] [1766531338.167864699] [pose_test_node]:   ✓ shoulder_lift: 2.700 rad (within [2.200, 2.886])
[INFO] [1766531338.168604759] [pose_test_node]:   ✓ elbow_flex: 1.200 rad (within [0.726, 2.250])
[INFO] [1766531338.169381763] [pose_test_node]:   ✓ wrist_flex: 2.700 rad (within [0.297, 2.700])
[INFO] [1766531338.170127007] [pose_test_node]:   ✓ wrist_roll: 0.000 rad (within [-1.448, 1.900])
[INFO] [1766531338.170851530] [pose_test_node]:   ✓ pen_holder: 0.900 rad (within [0.190, 1.600])
[INFO] [1766531338.172027221] [pose_test_node]: → Sent: wrist_flex_max
[INFO] [1766531342.179639436] [pose_test_node]:   📊 wrist_flex      [XL330] Pos:  2.612rad Curr:   86mA Volt: 4.9V Temp: 21.0°C
```
## Example of a validation error
The following is an example of a validation failure.
```bash
-rpi5:~/robot_ws$ ros2 run writing_robot_control pose_test --ros-args -p servo_test:=true -p servo_test_joint:=wrist_flex  -p validate:=true -p telemetry:=true -p urdf_file:=src/writing_robot_description/urdf/koch_v11_arm_real.urdf

[INFO] [1766531037.347260479] [pose_test_node]: Loading URDF limits...
[INFO] [1766531037.349198675] [pose_test_node]: ✓ Loaded limits for 6 joints
[INFO] [1766531037.349771233] [pose_test_node]: Initializing telemetry...
[INFO] [1766531037.355808116] [pose_test_node]: ✓ Telemetry initialized (dual-topic mode)
[INFO] [1766531037.356440601] [pose_test_node]:   - Subscribing to /joint_states
[INFO] [1766531037.357091586] [pose_test_node]:   - Subscribing to /dxl_state
[INFO] [1766531037.357585626] [pose_test_node]:   - XL430 servos (IDs 1,2): Load only
[INFO] [1766531037.358076906] [pose_test_node]:   - XL330 servos (IDs 3-6): Current + Load
[INFO] [1766531037.359465599] [pose_test_node]: ✓ Telemetry active
[INFO] [1766531037.360016731] [pose_test_node]: ============================================================
[INFO] [1766531037.360535678] [pose_test_node]: POSE TEST NODE INITIALIZED
[INFO] [1766531037.361041459] [pose_test_node]: ============================================================
[INFO] [1766531037.361554943] [pose_test_node]: Joints: 6
[INFO] [1766531037.362056982] [pose_test_node]: Validation: True
[INFO] [1766531037.362611115] [pose_test_node]: Telemetry: True
[INFO] [1766531037.363494194] [pose_test_node]: Movement time: 2.0s
[INFO] [1766531037.364100401] [pose_test_node]: Delay between poses: 2.0s
[INFO] [1766531037.364603588] [pose_test_node]: ============================================================
[INFO] [1766531037.365098665] [pose_test_node]: Waiting for joint trajectory controller...
[INFO] [1766531039.370914793] [pose_test_node]: 
============================================================
[INFO] [1766531039.371708519] [pose_test_node]: MOVING TO SAFE POSITION
[INFO] [1766531039.372430208] [pose_test_node]: ============================================================
[ERROR] [1766531039.373292064] [pose_test_node]:   ❌ shoulder_pan: 0.000 rad OUT OF RANGE [0.540, 2.044]
[INFO] [1766531039.374141939] [pose_test_node]:   ✓ shoulder_lift: 2.700 rad (within [2.200, 2.886])
[INFO] [1766531039.374865295] [pose_test_node]:   ✓ elbow_flex: 1.200 rad (within [0.726, 2.250])
[INFO] [1766531039.375566687] [pose_test_node]:   ✓ wrist_flex: 1.500 rad (within [0.297, 2.700])
[INFO] [1766531039.376257506] [pose_test_node]:   ✓ wrist_roll: 0.000 rad (within [-1.448, 1.900])
[INFO] [1766531039.376932694] [pose_test_node]:   ✓ pen_holder: 0.900 rad (within [0.190, 1.600])
[ERROR] [1766531039.377608124] [pose_test_node]: Safe position validation failed!
```

## FAQ
TBD
