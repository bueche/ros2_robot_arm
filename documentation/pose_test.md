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
```bash
rpi5:~/robot_ws$ ros2 run writing_robot_control pose_test --ros-args -p poses_file:=src/writing_robot_description/config/delivery_poses.yaml -p validate:=true -p telemetry:=true -p urdf_file:=src/writing_robot_description/urdf/koch_v11_arm_real.urdf -p power_monitoring:=true
POWER_MONITORING_AVAILABLEF: True
[INFO] [1768066260.099537922] [pose_test_node]: Loading URDF limits...
[INFO] [1768066260.101946582] [pose_test_node]: ✓ Loaded limits for 6 joints
[INFO] [1768066260.102590788] [pose_test_node]: Initializing telemetry...
[INFO] [1768066260.108656371] [pose_test_node]: ✓ Telemetry initialized (dual-topic mode)
[INFO] [1768066260.109449986] [pose_test_node]:   - Subscribing to /joint_states
[INFO] [1768066260.110017118] [pose_test_node]:   - Subscribing to /dxl_state
[INFO] [1768066260.110568936] [pose_test_node]:   - XL430 servos (IDs 1,2): Load only
[INFO] [1768066260.111120901] [pose_test_node]:   - XL330 servos (IDs 3-6): Current + Load
[INFO] [1768066260.112549093] [pose_test_node]: ✓ Telemetry active
[INFO] [1768066260.116461111] [pose_test_node]: ✓ Power monitoring enabled
[INFO] [1768066260.117041224] [pose_test_node]: ============================================================
[INFO] [1768066260.117588393] [pose_test_node]: POSE TEST NODE INITIALIZED
[INFO] [1768066260.118107025] [pose_test_node]: ============================================================
[INFO] [1768066260.118648065] [pose_test_node]: Joints: 6
[INFO] [1768066260.119245401] [pose_test_node]: Validation: True
[INFO] [1768066260.119948182] [pose_test_node]: Telemetry: True
[INFO] [1768066260.120534166] [pose_test_node]: Power Monitoring: Enabled
[INFO] [1768066260.121085558] [pose_test_node]: Movement time: 2.0s
[INFO] [1768066260.121608819] [pose_test_node]: Delay between poses: 2.0s
[INFO] [1768066260.122099488] [pose_test_node]: ============================================================
[INFO] [1768066260.122604546] [pose_test_node]: Waiting for joint trajectory controller...
[INFO] [1768066262.142233017] [pose_test_node]: ✓ Loaded 11 poses from src/writing_robot_description/config/delivery_poses.yaml
[INFO] [1768066262.142854520] [pose_test_node]: 
============================================================
[INFO] [1768066262.143396245] [pose_test_node]: POSE SEQUENCE TEST
[INFO] [1768066262.143915951] [pose_test_node]: ============================================================
[INFO] [1768066262.144439138] [pose_test_node]: Total poses: 11
[INFO] [1768066262.144961159] [pose_test_node]: Validation: True
[INFO] [1768066262.145520606] [pose_test_node]: Telemetry: True
[INFO] [1768066262.146037312] [pose_test_node]: ============================================================
[INFO] [1768066262.146572926] [pose_test_node]: 
[1/11] pose 1 - poised to work
[INFO] [1768066262.147100743] [pose_test_node]:   ✓ shoulder_pan: 1.558 rad (within [0.540, 2.044])
[INFO] [1768066262.147636801] [pose_test_node]:   ✓ shoulder_lift: 2.879 rad (within [2.200, 2.886])
[INFO] [1768066262.148151878] [pose_test_node]:   ✓ elbow_flex: 2.124 rad (within [0.726, 2.250])
[INFO] [1768066262.148678343] [pose_test_node]:   ✓ wrist_flex: 2.279 rad (within [0.297, 2.700])
[INFO] [1768066262.149188142] [pose_test_node]:   ✓ wrist_roll: 1.182 rad (within [-1.448, 1.900])
[INFO] [1768066262.149769663] [pose_test_node]:   ✓ pen_holder: 1.590 rad (within [0.190, 1.600])
[INFO] [1768066262.151134965] [pose_test_node]: → Sent: pose 1 - poised to work
[INFO] [1768066266.155382156] [pose_test_node]:   📊 Joint States:
[INFO] [1768066266.156301771] [pose_test_node]:   📊 shoulder_pan    [XL430] Pos:  1.558rad Load:  0.00% Volt:12.2V Temp: 26.0°C
[INFO] [1768066266.157138627] [pose_test_node]:   📊 shoulder_lift   [XL430] Pos:  2.841rad Load: 14.00% Volt:12.2V Temp: 28.0°C
[INFO] [1768066266.157930760] [pose_test_node]:   📊 elbow_flex      [XL330] Pos:  2.166rad Curr:  -40mA Volt: 5.6V Temp: 15.0°C
[INFO] [1768066266.158712467] [pose_test_node]:   📊 wrist_flex      [XL330] Pos:  2.224rad Curr:   78mA Volt: 5.6V Temp: 15.0°C
[INFO] [1768066266.159683268] [pose_test_node]:   📊 wrist_roll      [XL330] Pos:  1.186rad Curr:  -40mA Volt: 5.6V Temp: 15.0°C
[INFO] [1768066266.160568679] [pose_test_node]:   📊 pen_holder      [XL330] Pos:  1.567rad Curr:   18mA Volt: 5.5V Temp: 17.0°C
[INFO] [1768066266.161564128] [pose_test_node]:   ⚡ Power Analysis for "pose 1 - poised to work":
[INFO] [1768066266.162370206] [pose_test_node]:      Peak 5V current (INA219):    0.129A
[INFO] [1768066266.163125987] [pose_test_node]:      Servo current sum (XL330s):  0.176A
[INFO] [1768066266.163871398] [pose_test_node]:      Difference:                  0.047A
[INFO] [1768066266.164623531] [pose_test_node]:      5V voltage: 5.69V (min) / 5.75V (max)
[INFO] [1768066266.165389387] [pose_test_node]:      Peak 12V current:            0.195A
[INFO] [1768066266.166138686] [pose_test_node]:      Peak total power:            3.10W
[INFO] [1768066266.166794430] [pose_test_node]: 
[2/11] pose 2 - reach to grab
[INFO] [1768066266.167322766] [pose_test_node]:   ✓ shoulder_pan: 1.283 rad (within [0.540, 2.044])
[INFO] [1768066266.167847935] [pose_test_node]:   ✓ shoulder_lift: 2.314 rad (within [2.200, 2.886])
[INFO] [1768066266.168365400] [pose_test_node]:   ✓ elbow_flex: 1.320 rad (within [0.726, 2.250])
[INFO] [1768066266.168903847] [pose_test_node]:   ✓ wrist_flex: 2.279 rad (within [0.297, 2.700])
[INFO] [1768066266.169638980] [pose_test_node]:   ✓ wrist_roll: 1.871 rad (within [-1.448, 1.900])
[INFO] [1768066266.170228501] [pose_test_node]:   ✓ pen_holder: 0.982 rad (within [0.190, 1.600])
[INFO] [1768066266.171357432] [pose_test_node]: → Sent: pose 2 - reach to grab
[INFO] [1768066270.175864513] [pose_test_node]:   📊 Joint States:
[INFO] [1768066270.176791851] [pose_test_node]:   📊 shoulder_pan    [XL430] Pos:  1.289rad Load: -2.30% Volt:12.1V Temp: 26.0°C
[INFO] [1768066270.177625243] [pose_test_node]:   📊 shoulder_lift   [XL430] Pos:  2.296rad Load:  6.20% Volt:12.2V Temp: 28.0°C
[INFO] [1768066270.178409580] [pose_test_node]:   📊 elbow_flex      [XL330] Pos:  1.509rad Curr: -306mA Volt: 5.4V Temp: 15.0°C
[INFO] [1768066270.179189584] [pose_test_node]:   📊 wrist_flex      [XL330] Pos:  2.224rad Curr:   75mA Volt: 5.3V Temp: 15.0°C
[INFO] [1768066270.180166570] [pose_test_node]:   📊 wrist_roll      [XL330] Pos:  1.866rad Curr:   40mA Volt: 5.3V Temp: 15.0°C
[INFO] [1768066270.181023240] [pose_test_node]:   📊 pen_holder      [XL330] Pos:  1.044rad Curr:  -32mA Volt: 5.4V Temp: 17.0°C
[INFO] [1768066270.181984800] [pose_test_node]:   ⚡ Power Analysis for "pose 2 - reach to grab":
[INFO] [1768066270.182648655] [pose_test_node]:      Peak 5V current (INA219):    0.241A
[INFO] [1768066270.183247491] [pose_test_node]:      Servo current sum (XL330s):  0.453A
[INFO] [1768066270.183856901] [pose_test_node]:      Difference:                  0.212A
[WARN] [1768066270.184569867] [pose_test_node]:      ⚠ Significant current discrepancy!
[INFO] [1768066270.185190981] [pose_test_node]:      5V voltage: 5.54V (min) / 5.73V (max)
[INFO] [1768066270.185812743] [pose_test_node]:      Peak 12V current:            0.141A
[INFO] [1768066270.186426543] [pose_test_node]:      Peak total power:            2.78W
[INFO] [1768066270.187029675] [pose_test_node]: 
[3/11] pose 3 - grab
[INFO] [1768066270.187638085] [pose_test_node]:   ✓ shoulder_pan: 1.329 rad (within [0.540, 2.044])
[INFO] [1768066270.188233329] [pose_test_node]:   ✓ shoulder_lift: 2.322 rad (within [2.200, 2.886])
[INFO] [1768066270.188833850] [pose_test_node]:   ✓ elbow_flex: 1.005 rad (within [0.726, 2.250])
[INFO] [1768066270.189530464] [pose_test_node]:   ✓ wrist_flex: 2.279 rad (within [0.297, 2.700])
[INFO] [1768066270.190302023] [pose_test_node]:   ✓ wrist_roll: 1.766 rad (within [-1.448, 1.900])
[INFO] [1768066270.190985156] [pose_test_node]:   ✓ pen_holder: 1.501 rad (within [0.190, 1.600])
[INFO] [1768066270.192197958] [pose_test_node]: → Sent: pose 3 - grab
[INFO] [1768066274.195121846] [pose_test_node]:   📊 Joint States:
[INFO] [1768066274.196112517] [pose_test_node]:   📊 shoulder_pan    [XL430] Pos:  1.329rad Load:  0.00% Volt:12.2V Temp: 26.0°C
[INFO] [1768066274.197034873] [pose_test_node]:   📊 shoulder_lift   [XL430] Pos:  2.284rad Load: 14.00% Volt:12.1V Temp: 28.0°C
[INFO] [1768066274.198043007] [pose_test_node]:   📊 elbow_flex      [XL330] Pos:  1.211rad Curr: -355mA Volt: 5.3V Temp: 15.0°C
[INFO] [1768066274.199211309] [pose_test_node]:   📊 wrist_flex      [XL330] Pos:  2.224rad Curr:   75mA Volt: 5.3V Temp: 15.0°C
[INFO] [1768066274.200181813] [pose_test_node]:   📊 wrist_roll      [XL330] Pos:  1.766rad Curr:    0mA Volt: 5.4V Temp: 15.0°C
[INFO] [1768066274.201060429] [pose_test_node]:   📊 pen_holder      [XL330] Pos:  1.469rad Curr:   20mA Volt: 5.3V Temp: 17.0°C
[INFO] [1768066274.202119674] [pose_test_node]:   ⚡ Power Analysis for "pose 3 - grab":
[INFO] [1768066274.202917770] [pose_test_node]:      Peak 5V current (INA219):    0.314A
[INFO] [1768066274.203695792] [pose_test_node]:      Servo current sum (XL330s):  0.450A
[INFO] [1768066274.204472833] [pose_test_node]:      Difference:                  0.136A
[WARN] [1768066274.205408263] [pose_test_node]:      ⚠ Significant current discrepancy!
[INFO] [1768066274.206324156] [pose_test_node]:      5V voltage: 5.44V (min) / 5.72V (max)
[INFO] [1768066274.206926029] [pose_test_node]:      Peak 12V current:            0.140A
[INFO] [1768066274.207431828] [pose_test_node]:      Peak total power:            6.28W
[INFO] [1768066274.207977608] [pose_test_node]: 
[4/11] pose 4 - lift and center
[INFO] [1768066274.208581981] [pose_test_node]:   ✓ shoulder_pan: 1.576 rad (within [0.540, 2.044])
[INFO] [1768066274.209279873] [pose_test_node]:   ✓ shoulder_lift: 2.575 rad (within [2.200, 2.886])
[INFO] [1768066274.209825654] [pose_test_node]:   ✓ elbow_flex: 1.898 rad (within [0.726, 2.250])
[INFO] [1768066274.210330989] [pose_test_node]:   ✓ wrist_flex: 2.279 rad (within [0.297, 2.700])
[INFO] [1768066274.210829103] [pose_test_node]:   ✓ wrist_roll: 1.766 rad (within [-1.448, 1.900])
[INFO] [1768066274.211318772] [pose_test_node]:   ✓ pen_holder: 1.521 rad (within [0.190, 1.600])
[INFO] [1768066274.212344795] [pose_test_node]: → Sent: pose 4 - lift and center
[INFO] [1768066278.216164484] [pose_test_node]:   📊 Joint States:
[INFO] [1768066278.217216859] [pose_test_node]:   📊 shoulder_pan    [XL430] Pos:  1.576rad Load:  0.00% Volt:12.2V Temp: 26.0°C
[INFO] [1768066278.218041733] [pose_test_node]:   📊 shoulder_lift   [XL430] Pos:  2.529rad Load: 16.30% Volt:12.1V Temp: 28.0°C
[INFO] [1768066278.218856422] [pose_test_node]:   📊 elbow_flex      [XL330] Pos:  1.935rad Curr:  -37mA Volt: 5.6V Temp: 15.0°C
[INFO] [1768066278.219830482] [pose_test_node]:   📊 wrist_flex      [XL330] Pos:  2.224rad Curr:   78mA Volt: 5.6V Temp: 15.0°C
[INFO] [1768066278.220660560] [pose_test_node]:   📊 wrist_roll      [XL330] Pos:  1.766rad Curr:    0mA Volt: 5.5V Temp: 15.0°C
[INFO] [1768066278.221437267] [pose_test_node]:   📊 pen_holder      [XL330] Pos:  1.470rad Curr:   26mA Volt: 5.6V Temp: 17.0°C
[INFO] [1768066278.222341512] [pose_test_node]:   ⚡ Power Analysis for "pose 4 - lift and center":
[INFO] [1768066278.223081571] [pose_test_node]:      Peak 5V current (INA219):    0.229A
[INFO] [1768066278.223820055] [pose_test_node]:      Servo current sum (XL330s):  0.141A
[INFO] [1768066278.224558633] [pose_test_node]:      Difference:                  0.088A
[WARN] [1768066278.225442118] [pose_test_node]:      ⚠ Significant current discrepancy!
[INFO] [1768066278.226179548] [pose_test_node]:      5V voltage: 5.54V (min) / 5.74V (max)
[INFO] [1768066278.226869310] [pose_test_node]:      Peak 12V current:            0.334A
[INFO] [1768066278.227380423] [pose_test_node]:      Peak total power:            4.65W
[INFO] [1768066278.227916815] [pose_test_node]: 
[5/11] pose 5 - raise and get ready to place
[INFO] [1768066278.228467540] [pose_test_node]:   ✓ shoulder_pan: 1.596 rad (within [0.540, 2.044])
[INFO] [1768066278.229117468] [pose_test_node]:   ✓ shoulder_lift: 2.795 rad (within [2.200, 2.886])
[INFO] [1768066278.229730323] [pose_test_node]:   ✓ elbow_flex: 0.970 rad (within [0.726, 2.250])
[INFO] [1768066278.230263159] [pose_test_node]:   ✓ wrist_flex: 1.473 rad (within [0.297, 2.700])
[INFO] [1768066278.230795180] [pose_test_node]:   ✓ wrist_roll: 1.762 rad (within [-1.448, 1.900])
[INFO] [1768066278.231365127] [pose_test_node]:   ✓ pen_holder: 1.521 rad (within [0.190, 1.600])
[INFO] [1768066278.232473373] [pose_test_node]: → Sent: pose 5 - raise and get ready to place
[INFO] [1768066282.234917388] [pose_test_node]:   📊 Joint States:
[INFO] [1768066282.235846041] [pose_test_node]:   📊 shoulder_pan    [XL430] Pos:  1.587rad Load:  3.90% Volt:12.1V Temp: 26.0°C
[INFO] [1768066282.236700434] [pose_test_node]:   📊 shoulder_lift   [XL430] Pos:  2.741rad Load: 19.10% Volt:12.1V Temp: 29.0°C
[INFO] [1768066282.237483178] [pose_test_node]:   📊 elbow_flex      [XL330] Pos:  1.202rad Curr: -419mA Volt: 5.3V Temp: 16.0°C
[INFO] [1768066282.238234181] [pose_test_node]:   📊 wrist_flex      [XL330] Pos:  1.466rad Curr:   40mA Volt: 5.3V Temp: 15.0°C
[INFO] [1768066282.239058259] [pose_test_node]:   📊 wrist_roll      [XL330] Pos:  1.763rad Curr:  -40mA Volt: 5.4V Temp: 15.0°C
[INFO] [1768066282.240079504] [pose_test_node]:   📊 pen_holder      [XL330] Pos:  1.470rad Curr:   26mA Volt: 5.2V Temp: 17.0°C
[INFO] [1768066282.241088879] [pose_test_node]:   ⚡ Power Analysis for "pose 5 - raise and get ready to place":
[INFO] [1768066282.241845642] [pose_test_node]:      Peak 5V current (INA219):    0.304A
[INFO] [1768066282.242515571] [pose_test_node]:      Servo current sum (XL330s):  0.525A
[INFO] [1768066282.243111907] [pose_test_node]:      Difference:                  0.221A
[WARN] [1768066282.243833077] [pose_test_node]:      ⚠ Significant current discrepancy!
[INFO] [1768066282.244441061] [pose_test_node]:      5V voltage: 5.44V (min) / 5.74V (max)
[INFO] [1768066282.245050823] [pose_test_node]:      Peak 12V current:            0.198A
[INFO] [1768066282.245669604] [pose_test_node]:      Peak total power:            3.84W
[INFO] [1768066282.246279773] [pose_test_node]: 
[6/11] pose 6 - place high
[INFO] [1768066282.246890035] [pose_test_node]:   ✓ shoulder_pan: 1.598 rad (within [0.540, 2.044])
[INFO] [1768066282.247491797] [pose_test_node]:   ✓ shoulder_lift: 2.655 rad (within [2.200, 2.886])
[INFO] [1768066282.248079356] [pose_test_node]:   ✓ elbow_flex: 0.806 rad (within [0.726, 2.250])
[INFO] [1768066282.248691210] [pose_test_node]:   ✓ wrist_flex: 1.473 rad (within [0.297, 2.700])
[INFO] [1768066282.249475603] [pose_test_node]:   ✓ wrist_roll: 1.762 rad (within [-1.448, 1.900])
[INFO] [1768066282.250130384] [pose_test_node]:   ✓ pen_holder: 0.655 rad (within [0.190, 1.600])
[INFO] [1768066282.251319556] [pose_test_node]: → Sent: pose 6 - place high
[INFO] [1768066286.263065077] [pose_test_node]:   📊 Joint States:
[INFO] [1768066286.263944914] [pose_test_node]:   📊 shoulder_pan    [XL430] Pos:  1.587rad Load:  4.40% Volt:12.2V Temp: 26.0°C
[INFO] [1768066286.264749658] [pose_test_node]:   📊 shoulder_lift   [XL430] Pos:  2.650rad Load:  1.60% Volt:12.1V Temp: 29.0°C
[INFO] [1768066286.265518125] [pose_test_node]:   📊 elbow_flex      [XL330] Pos:  1.028rad Curr: -497mA Volt: 5.3V Temp: 16.0°C
[INFO] [1768066286.266247517] [pose_test_node]:   📊 wrist_flex      [XL330] Pos:  1.464rad Curr:   40mA Volt: 5.3V Temp: 15.0°C
[INFO] [1768066286.266966835] [pose_test_node]:   📊 wrist_roll      [XL330] Pos:  1.762rad Curr:   37mA Volt: 5.3V Temp: 15.0°C
[INFO] [1768066286.267691524] [pose_test_node]:   📊 pen_holder      [XL330] Pos:  0.734rad Curr:  -38mA Volt: 5.2V Temp: 16.0°C
[INFO] [1768066286.268552861] [pose_test_node]:   ⚡ Power Analysis for "pose 6 - place high":
[INFO] [1768066286.269315790] [pose_test_node]:      Peak 5V current (INA219):    0.541A
[INFO] [1768066286.270181239] [pose_test_node]:      Servo current sum (XL330s):  0.612A
[INFO] [1768066286.270937131] [pose_test_node]:      Difference:                  0.071A
[WARN] [1768066286.271743246] [pose_test_node]:      ⚠ Significant current discrepancy!
[INFO] [1768066286.272279582] [pose_test_node]:      5V voltage: 5.10V (min) / 5.74V (max)
[INFO] [1768066286.272822677] [pose_test_node]:      Peak 12V current:            0.163A
[INFO] [1768066286.273347679] [pose_test_node]:      Peak total power:            4.16W
[INFO] [1768066286.273945904] [pose_test_node]: 
[7/11] pose 7 - reach to grab again
[INFO] [1768066286.274487425] [pose_test_node]:   ✓ shoulder_pan: 1.283 rad (within [0.540, 2.044])
[INFO] [1768066286.275049002] [pose_test_node]:   ✓ shoulder_lift: 2.314 rad (within [2.200, 2.886])
[INFO] [1768066286.275653449] [pose_test_node]:   ✓ elbow_flex: 1.320 rad (within [0.726, 2.250])
[INFO] [1768066286.276225007] [pose_test_node]:   ✓ wrist_flex: 2.327 rad (within [0.297, 2.700])
[INFO] [1768066286.276790121] [pose_test_node]:   ✓ wrist_roll: 1.871 rad (within [-1.448, 1.900])
[INFO] [1768066286.277333568] [pose_test_node]:   ✓ pen_holder: 0.982 rad (within [0.190, 1.600])
[INFO] [1768066286.278491425] [pose_test_node]: → Sent: pose 7 - reach to grab again
[INFO] [1768066290.289811666] [pose_test_node]:   📊 Joint States:
[INFO] [1768066290.290741078] [pose_test_node]:   📊 shoulder_pan    [XL430] Pos:  1.289rad Load: -2.30% Volt:12.2V Temp: 26.0°C
[INFO] [1768066290.291938676] [pose_test_node]:   📊 shoulder_lift   [XL430] Pos:  2.296rad Load:  6.20% Volt:12.2V Temp: 29.0°C
[INFO] [1768066290.292728346] [pose_test_node]:   📊 elbow_flex      [XL330] Pos:  1.378rad Curr:  -56mA Volt: 5.6V Temp: 16.0°C
[INFO] [1768066290.293451720] [pose_test_node]:   📊 wrist_flex      [XL330] Pos:  2.261rad Curr:   86mA Volt: 5.5V Temp: 15.0°C
[INFO] [1768066290.294146038] [pose_test_node]:   📊 wrist_roll      [XL330] Pos:  1.865rad Curr:   40mA Volt: 5.6V Temp: 15.0°C
[INFO] [1768066290.294873708] [pose_test_node]:   📊 pen_holder      [XL330] Pos:  0.944rad Curr:   23mA Volt: 5.5V Temp: 17.0°C
[INFO] [1768066290.295744878] [pose_test_node]:   ⚡ Power Analysis for "pose 7 - reach to grab again":
[INFO] [1768066290.296498882] [pose_test_node]:      Peak 5V current (INA219):    0.288A
[INFO] [1768066290.297348441] [pose_test_node]:      Servo current sum (XL330s):  0.205A
[INFO] [1768066290.298170130] [pose_test_node]:      Difference:                  0.083A
[WARN] [1768066290.299037115] [pose_test_node]:      ⚠ Significant current discrepancy!
[INFO] [1768066290.299902971] [pose_test_node]:      5V voltage: 5.46V (min) / 5.74V (max)
[INFO] [1768066290.300786864] [pose_test_node]:      Peak 12V current:            0.155A
[INFO] [1768066290.301553312] [pose_test_node]:      Peak total power:            3.00W
[INFO] [1768066290.302265278] [pose_test_node]: 
[8/11] pose 8 - grab
[INFO] [1768066290.302930337] [pose_test_node]:   ✓ shoulder_pan: 1.329 rad (within [0.540, 2.044])
[INFO] [1768066290.303438358] [pose_test_node]:   ✓ shoulder_lift: 2.314 rad (within [2.200, 2.886])
[INFO] [1768066290.303925990] [pose_test_node]:   ✓ elbow_flex: 1.005 rad (within [0.726, 2.250])
[INFO] [1768066290.304426751] [pose_test_node]:   ✓ wrist_flex: 2.037 rad (within [0.297, 2.700])
[INFO] [1768066290.304928272] [pose_test_node]:   ✓ wrist_roll: 1.766 rad (within [-1.448, 1.900])
[INFO] [1768066290.305447293] [pose_test_node]:   ✓ pen_holder: 1.521 rad (within [0.190, 1.600])
[INFO] [1768066290.306499131] [pose_test_node]: → Sent: pose 8 - grab
[INFO] [1768066294.310268079] [pose_test_node]:   📊 Joint States:
[INFO] [1768066294.311199898] [pose_test_node]:   📊 shoulder_pan    [XL430] Pos:  1.329rad Load:  0.00% Volt:12.2V Temp: 26.0°C
[INFO] [1768066294.311991735] [pose_test_node]:   📊 shoulder_lift   [XL430] Pos:  2.284rad Load: 10.70% Volt:12.2V Temp: 29.0°C
[INFO] [1768066294.312733053] [pose_test_node]:   📊 elbow_flex      [XL330] Pos:  1.212rad Curr: -349mA Volt: 5.4V Temp: 16.0°C
[INFO] [1768066294.313447352] [pose_test_node]:   📊 wrist_flex      [XL330] Pos:  2.035rad Curr:   37mA Volt: 5.4V Temp: 15.0°C
[INFO] [1768066294.314214467] [pose_test_node]:   📊 wrist_roll      [XL330] Pos:  1.766rad Curr:    0mA Volt: 5.4V Temp: 15.0°C
[INFO] [1768066294.315023526] [pose_test_node]:   📊 pen_holder      [XL330] Pos:  1.484rad Curr:   21mA Volt: 5.4V Temp: 17.0°C
[INFO] [1768066294.315912253] [pose_test_node]:   ⚡ Power Analysis for "pose 8 - grab":
[INFO] [1768066294.316702867] [pose_test_node]:      Peak 5V current (INA219):    0.385A
[INFO] [1768066294.317460037] [pose_test_node]:      Servo current sum (XL330s):  0.407A
[INFO] [1768066294.318288708] [pose_test_node]:      Difference:                  0.022A
[INFO] [1768066294.319192675] [pose_test_node]:      5V voltage: 5.32V (min) / 5.72V (max)
[INFO] [1768066294.320024956] [pose_test_node]:      Peak 12V current:            0.131A
[INFO] [1768066294.320664959] [pose_test_node]:      Peak total power:            3.59W
[INFO] [1768066294.321199480] [pose_test_node]: 
[9/11] pose 9 - place low
[INFO] [1768066294.321728409] [pose_test_node]:   ✓ shoulder_pan: 1.925 rad (within [0.540, 2.044])
[INFO] [1768066294.322230133] [pose_test_node]:   ✓ shoulder_lift: 2.403 rad (within [2.200, 2.886])
[INFO] [1768066294.322738487] [pose_test_node]:   ✓ elbow_flex: 1.352 rad (within [0.726, 2.250])
[INFO] [1768066294.323236749] [pose_test_node]:   ✓ wrist_flex: 2.327 rad (within [0.297, 2.700])
[INFO] [1768066294.323746547] [pose_test_node]:   ✓ wrist_roll: 1.737 rad (within [-1.448, 1.900])
[INFO] [1768066294.324269198] [pose_test_node]:   ✓ pen_holder: 1.521 rad (within [0.190, 1.600])
[INFO] [1768066294.325372277] [pose_test_node]: → Sent: pose 9 - place low
[INFO] [1768066298.336740222] [pose_test_node]:   📊 Joint States:
[INFO] [1768066298.337753671] [pose_test_node]:   📊 shoulder_pan    [XL430] Pos:  1.918rad Load:  2.20% Volt:12.1V Temp: 26.0°C
[INFO] [1768066298.338613120] [pose_test_node]:   📊 shoulder_lift   [XL430] Pos:  2.348rad Load: 20.30% Volt:12.1V Temp: 29.0°C
[INFO] [1768066298.339566494] [pose_test_node]:   📊 elbow_flex      [XL330] Pos:  1.404rad Curr:  -51mA Volt: 5.6V Temp: 16.0°C
[INFO] [1768066298.340374387] [pose_test_node]:   📊 wrist_flex      [XL330] Pos:  2.250rad Curr:  104mA Volt: 5.5V Temp: 15.0°C
[INFO] [1768066298.341159335] [pose_test_node]:   📊 wrist_roll      [XL330] Pos:  1.742rad Curr:  -40mA Volt: 5.6V Temp: 15.0°C
[INFO] [1768066298.341863116] [pose_test_node]:   📊 pen_holder      [XL330] Pos:  1.484rad Curr:   22mA Volt: 5.5V Temp: 17.0°C
[INFO] [1768066298.342678546] [pose_test_node]:   ⚡ Power Analysis for "pose 9 - place low":
[INFO] [1768066298.343322419] [pose_test_node]:      Peak 5V current (INA219):    0.215A
[INFO] [1768066298.343966996] [pose_test_node]:      Servo current sum (XL330s):  0.217A
[INFO] [1768066298.344643555] [pose_test_node]:      Difference:                  0.002A
[INFO] [1768066298.345320298] [pose_test_node]:      5V voltage: 5.57V (min) / 5.74V (max)
[INFO] [1768066298.345989653] [pose_test_node]:      Peak 12V current:            0.230A
[INFO] [1768066298.346641027] [pose_test_node]:      Peak total power:            3.51W
[INFO] [1768066298.347282067] [pose_test_node]: 
[10/11] pose 10 - release
[INFO] [1768066298.347936384] [pose_test_node]:   ✓ shoulder_pan: 1.872 rad (within [0.540, 2.044])
[INFO] [1768066298.348600369] [pose_test_node]:   ✓ shoulder_lift: 2.403 rad (within [2.200, 2.886])
[INFO] [1768066298.349391076] [pose_test_node]:   ✓ elbow_flex: 1.266 rad (within [0.726, 2.250])
[INFO] [1768066298.350099505] [pose_test_node]:   ✓ wrist_flex: 2.327 rad (within [0.297, 2.700])
[INFO] [1768066298.350768323] [pose_test_node]:   ✓ wrist_roll: 1.737 rad (within [-1.448, 1.900])
[INFO] [1768066298.351361011] [pose_test_node]:   ✓ pen_holder: 0.582 rad (within [0.190, 1.600])
[INFO] [1768066298.352384794] [pose_test_node]: → Sent: pose 10 - release
[INFO] [1768066302.359557775] [pose_test_node]:   📊 Joint States:
[INFO] [1768066302.361167597] [pose_test_node]:   📊 shoulder_pan    [XL430] Pos:  1.874rad Load: -0.90% Volt:12.1V Temp: 26.0°C
[INFO] [1768066302.362396177] [pose_test_node]:   📊 shoulder_lift   [XL430] Pos:  2.344rad Load: 21.90% Volt:12.1V Temp: 29.0°C
[INFO] [1768066302.363350848] [pose_test_node]:   📊 elbow_flex      [XL330] Pos:  1.463rad Curr: -320mA Volt: 5.3V Temp: 16.0°C
[INFO] [1768066302.364196371] [pose_test_node]:   📊 wrist_flex      [XL330] Pos:  2.250rad Curr:   99mA Volt: 5.3V Temp: 15.0°C
[INFO] [1768066302.365049023] [pose_test_node]:   📊 wrist_roll      [XL330] Pos:  1.737rad Curr:    0mA Volt: 5.4V Temp: 15.0°C
[INFO] [1768066302.365840508] [pose_test_node]:   📊 pen_holder      [XL330] Pos:  0.652rad Curr:  -36mA Volt: 5.3V Temp: 17.0°C
[INFO] [1768066302.366781697] [pose_test_node]:   ⚡ Power Analysis for "pose 10 - release":
[INFO] [1768066302.367525256] [pose_test_node]:      Peak 5V current (INA219):    0.240A
[INFO] [1768066302.368246537] [pose_test_node]:      Servo current sum (XL330s):  0.455A
[INFO] [1768066302.368990263] [pose_test_node]:      Difference:                  0.215A
[WARN] [1768066302.369857081] [pose_test_node]:      ⚠ Significant current discrepancy!
[INFO] [1768066302.370768030] [pose_test_node]:      5V voltage: 5.53V (min) / 5.73V (max)
[INFO] [1768066302.371371218] [pose_test_node]:      Peak 12V current:            0.190A
[INFO] [1768066302.371900202] [pose_test_node]:      Peak total power:            3.63W
[INFO] [1768066302.372408834] [pose_test_node]: 
[11/11] pose 11 - poised to work
[INFO] [1768066302.372883040] [pose_test_node]:   ✓ shoulder_pan: 1.558 rad (within [0.540, 2.044])
[INFO] [1768066302.373355523] [pose_test_node]:   ✓ shoulder_lift: 2.879 rad (within [2.200, 2.886])
[INFO] [1768066302.373842970] [pose_test_node]:   ✓ elbow_flex: 2.124 rad (within [0.726, 2.250])
[INFO] [1768066302.374321157] [pose_test_node]:   ✓ wrist_flex: 2.279 rad (within [0.297, 2.700])
[INFO] [1768066302.374824882] [pose_test_node]:   ✓ wrist_roll: 1.182 rad (within [-1.448, 1.900])
[INFO] [1768066302.375349051] [pose_test_node]:   ✓ pen_holder: 1.590 rad (within [0.190, 1.600])
[INFO] [1768066302.376435723] [pose_test_node]: → Sent: pose 11 - poised to work
[INFO] [1768066306.377787104] [pose_test_node]:   📊 Joint States:
[INFO] [1768066306.378798738] [pose_test_node]:   📊 shoulder_pan    [XL430] Pos:  1.564rad Load: -2.30% Volt:12.2V Temp: 27.0°C
[INFO] [1768066306.379988966] [pose_test_node]:   📊 shoulder_lift   [XL430] Pos:  2.838rad Load: 15.20% Volt:12.2V Temp: 29.0°C
[INFO] [1768066306.380850488] [pose_test_node]:   📊 elbow_flex      [XL330] Pos:  2.167rad Curr:  -40mA Volt: 5.6V Temp: 16.0°C
[INFO] [1768066306.381617658] [pose_test_node]:   📊 wrist_flex      [XL330] Pos:  2.248rad Curr:   53mA Volt: 5.6V Temp: 15.0°C
[INFO] [1768066306.382369606] [pose_test_node]:   📊 wrist_roll      [XL330] Pos:  1.188rad Curr:  -40mA Volt: 5.6V Temp: 15.0°C
[INFO] [1768066306.383124943] [pose_test_node]:   📊 pen_holder      [XL330] Pos:  1.552rad Curr:   23mA Volt: 5.5V Temp: 17.0°C
[INFO] [1768066306.384021373] [pose_test_node]:   ⚡ Power Analysis for "pose 11 - poised to work":
[INFO] [1768066306.384775617] [pose_test_node]:      Peak 5V current (INA219):    0.241A
[INFO] [1768066306.385581676] [pose_test_node]:      Servo current sum (XL330s):  0.156A
[INFO] [1768066306.386313420] [pose_test_node]:      Difference:                  0.085A
[WARN] [1768066306.387166591] [pose_test_node]:      ⚠ Significant current discrepancy!
[INFO] [1768066306.387898187] [pose_test_node]:      5V voltage: 5.53V (min) / 5.74V (max)
[INFO] [1768066306.388509134] [pose_test_node]:      Peak 12V current:            0.288A
[INFO] [1768066306.389066748] [pose_test_node]:      Peak total power:            4.17W
[INFO] [1768066306.389834214] [pose_test_node]: 
============================================================
[INFO] [1768066306.390441309] [pose_test_node]: SUMMARY
[INFO] [1768066306.390975571] [pose_test_node]: ============================================================
[INFO] [1768066306.391514925] [pose_test_node]: Poses tested: 11
[INFO] [1768066306.392036798] [pose_test_node]: Validation errors: 0
[INFO] [1768066306.392568986] [pose_test_node]: Validation warnings: 0
[INFO] [1768066306.393085007] [pose_test_node]: ============================================================
[INFO] [1768066306.393716639] [pose_test_node]: 
===========================================================================
[INFO] [1768066306.394288512] [pose_test_node]: 📊 TELEMETRY SUMMARY
[INFO] [1768066306.394824144] [pose_test_node]: ===========================================================================
[INFO] [1768066306.395448869] [pose_test_node]: XL430 (IDs 1-2): Avg Load: 6.4%
[INFO] [1768066306.396050650] [pose_test_node]: XL330 (IDs 3-6): Total Current: -4mA, Avg: -1mA (-0.0W @ 12V)
[INFO] [1768066306.396711560] [pose_test_node]: Max Temperature: 29.0°C
[INFO] [1768066306.397340137] [pose_test_node]: Voltage: 5.50V - 12.20V (avg 7.78V)
[INFO] [1768066306.398075752] [pose_test_node]: ===========================================================================
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
