# Running the pose load test analysis utility

The load tester analysis tool will examine a sequence of transitions and make torque projections as to the load safety of that move based on the physical characteristics of the robot, its speed of movement, and starting/ending poses. 

- Basic load testing (torque predictions)
- Trajectory analysis (dynamic torque including acceleration, friction, back-EMF)
- Full pose-based analysis with forward kinematics (ACCURATE moment arms)
- YAML pose sequence analysis (test entire sequences from [pose_test.py](./documentation/pose_test.md)
- What-if analysis (test different servos and pose transition timings)
- Servo telemetry (temp/current/voltage - if dynamixel_sdk available)
- Comparison matrix (compare all servo options)

These are projections and the supporting analysis are a work in progress (that still need a bit of validation). 🚧 

## Table of contents
- [Parameters](#parameters)
- [Some example invocations](#some-example-invocations)
- [Example run with shorter pose duration](#example-run-with-shorter-pose-duration)
- [Example servo motor what-if](#example-servo-motor-what-if)
- [FAQ](#faq)


## Parameters:
- `--joint`: Joint to analyze
- `--static`: Static hold test
- `--payload`: Payload capacity test
- `--diagnostics`: Full diagnostics with telemetry
- `--what-if`, SERVO-TYPE to swap out in model for existing one in specified joint (e.g., XL330-M077, XL330-M288, XM430-W350)
- `--compare-servos`: (default: True) compare all servo options
- `--full_analysis`: Complete analysis
- `--position`: Test at specific position (radians)
- `--previous-position`,  Previous position for trajectory analysis (radians)
- `--movement-time`: (default: 2.0) Movement time for trajectory analysis
- `--analyze-transition`: Analyze full pose transition (use with --pose-from and --pose-to)')
- `--pose-from`: Starting pose as comma-separated angles in radians (e.g., "0.0,1.57,0.0,0.0,0.0,0.0")
- `--pose-to`: Ending pose as comma-separated angles
- `--analyze-sequence`: Analyze complete pose sequence from YAML file. See the following example yaml file: [delivery_poses.yaml](https://github.com/bueche/ros2_robot_arm/blob/main/writing_robot_description/config/delivery_poses.yaml). That yaml file can be used as input to the pose_test tool.
- `--csv-output`: Append analysis results to CSV file for Excel graphing')
    
## Some example invocations
```
# Basic tests
  ros2 run writing_robot_control load_tester --full_analysis
  ros2 run writing_robot_control load_tester --joint shoulder_lift --static
  ros2 run writing_robot_control load_tester --joint shoulder_lift --payload
  
  # Trajectory analysis (single joint)
  ros2 run writing_robot_control load_tester --joint shoulder_lift --static \\
    --previous-position 0.0 --movement-time 2.0
  
  # Full pose transition analysis
  ros2 run writing_robot_control load_tester --analyze-transition \\
    --pose-from "0.0,1.57,0.0,0.0,0.0,0.0" \\
    --pose-to "0.0,1.57,1.57,0.0,0.0,0.0" \\
    --movement-time 2.0
  
  # YAML sequence analysis
  ros2 run writing_robot_control load_tester --analyze-sequence poses.yaml \\
    --movement-time 2.0
  
  # CSV output for Excel graphing. 
  ros2 run writing_robot_control load_tester --analyze-sequence poses.yaml \\
    --movement-time 2.0 --csv-output results.csv
  
  # Compare different movement times (appends to same CSV)
  ros2 run writing_robot_control load_tester --analyze-sequence poses.yaml \\
    --movement-time 2.0 --csv-output comparison.csv
  ros2 run writing_robot_control load_tester --analyze-sequence poses.yaml \\
    --movement-time 1.0 --csv-output comparison.csv
  ros2 run writing_robot_control load_tester --analyze-sequence poses.yaml \\
    --movement-time 0.5 --csv-output comparison.csv
  
  # What-if scenarios
  ros2 run writing_robot_control load_tester --joint shoulder_lift --what-if XL330-M077
  ros2 run writing_robot_control load_tester --joint shoulder_lift --what-if XL330-M288
  ros2 run writing_robot_control load_tester --joint shoulder_lift --what-if XM430-W350
  
  # Diagnostics with telemetry
  ros2 run writing_robot_control load_tester --joint shoulder_lift --diagnostics
  
  # Compare all options
  ros2 run writing_robot_control load_tester --compare-servos
        """
```
## Example run with shorter pose duration
Example run in which the speed of movement is increased, and hence putting more potential strain on the servos. Again, any projection (at the moment) has not been verified with samples.
```bash
-rpi5:~/robot_ws$ ros2 run writing_robot_control load_tester --analyze-sequence src/writing_robot_description/config/default_poses.yaml    --movement-time 0.5 --csv-output movement.tst.csv

[INFO] [1766198123.975833871] [load_tester]: ✓ Dynamixel SDK connected (telemetry enabled)
[INFO] [1766198123.976458355] [load_tester]: Load Tester initialized
[INFO] [1766198123.977010210] [load_tester]: 
============================================================
[INFO] [1766198123.977552861] [load_tester]: YAML POSE SEQUENCE ANALYSIS
[INFO] [1766198123.978092271] [load_tester]: ============================================================
[INFO] [1766198123.978611773] [load_tester]: File: src/writing_robot_description/config/default_poses.yaml
[INFO] [1766198123.985224768] [load_tester]: Found 6 poses
[INFO] [1766198123.985798715] [load_tester]: CSV output: movement.tst.csv
[INFO] [1766198123.986313014] [load_tester]: 
============================================================
[INFO] [1766198123.986829016] [load_tester]: TRANSITION 1/5: pose 0 → pose 1
[INFO] [1766198123.987322352] [load_tester]: ============================================================
[INFO] [1766198123.987837744] [load_tester]: 
============================================================
[INFO] [1766198123.988327616] [load_tester]: FULL POSE TRANSITION ANALYSIS
[INFO] [1766198123.988837859] [load_tester]: ============================================================
[INFO] [1766198123.989321121] [load_tester]: Movement time: 0.5s
[INFO] [1766198123.989892513] [load_tester]: 
From pose:
[INFO] [1766198123.990440497] [load_tester]:   shoulder_pan   :  1.168 rad (  66.9°)
[INFO] [1766198123.990983036] [load_tester]:   shoulder_lift  :  2.600 rad ( 149.0°)
[INFO] [1766198123.991516243] [load_tester]:   elbow_flex     :  1.193 rad (  68.4°)
[INFO] [1766198123.992038838] [load_tester]:   wrist_flex     :  2.687 rad ( 154.0°)
[INFO] [1766198123.992561563] [load_tester]:   wrist_roll     :  1.019 rad (  58.4°)
[INFO] [1766198123.993080676] [load_tester]:   pen_holder     :  1.100 rad (  63.0°)
[INFO] [1766198123.993579493] [load_tester]: 
To pose:
[INFO] [1766198123.994105959] [load_tester]:   shoulder_pan   :  1.168 rad (  66.9°)
[INFO] [1766198123.994779814] [load_tester]:   shoulder_lift  :  2.500 rad ( 143.2°)
[INFO] [1766198123.995489743] [load_tester]:   elbow_flex     :  1.193 rad (  68.4°)
[INFO] [1766198123.996104691] [load_tester]:   wrist_flex     :  2.456 rad ( 140.7°)
[INFO] [1766198123.996645990] [load_tester]:   wrist_roll     :  0.925 rad (  53.0°)
[INFO] [1766198123.997153492] [load_tester]:   pen_holder     :  1.580 rad (  90.5°)
[INFO] [1766198123.998293127] [load_tester]: 
============================================================
[INFO] [1766198123.998794185] [load_tester]: JOINT-BY-JOINT ANALYSIS
[INFO] [1766198123.999275354] [load_tester]: ============================================================
[INFO] [1766198123.999773320] [load_tester]: 
SHOULDER_LIFT
[INFO] [1766198124.000277600] [load_tester]:   Servo: XL430-W250
[INFO] [1766198124.000799806] [load_tester]:   Movement: 5.7° in 0.20 rad/s avg
[INFO] [1766198124.001295364] [load_tester]: 
  Torque breakdown:
[INFO] [1766198124.001775533] [load_tester]:     Static (FK):      0.078 Nm
[INFO] [1766198124.002253128] [load_tester]:     Static (simple):  0.463 Nm
[INFO] [1766198124.002742908] [load_tester]:     FK error:         0.385 Nm (+490.8%)
[INFO] [1766198124.003222966] [load_tester]:     Acceleration:     0.027 Nm
[INFO] [1766198124.003696802] [load_tester]:     Friction:         0.033 Nm
[INFO] [1766198124.004178693] [load_tester]:     ───────────────────────
[INFO] [1766198124.004657991] [load_tester]:     Total required:   0.138 Nm
[INFO] [1766198124.005136605] [load_tester]: 
  Servo capacity:
[INFO] [1766198124.005616255] [load_tester]:     Back-EMF loss:    0.057 Nm
[INFO] [1766198124.006120813] [load_tester]:     Available:        1.343 Nm
[INFO] [1766198124.006844687] [load_tester]:     Utilization:      10.3%
[INFO] [1766198124.007486005] [load_tester]:     ✓ SAFE
[INFO] [1766198124.008046360] [load_tester]: 
WRIST_FLEX
[INFO] [1766198124.008573955] [load_tester]:   Servo: XL330-M288
[INFO] [1766198124.009104846] [load_tester]:   Movement: 13.3° in 0.46 rad/s avg
[INFO] [1766198124.009668979] [load_tester]: 
  Torque breakdown:
[INFO] [1766198124.010180629] [load_tester]:     Static (FK):      0.031 Nm
[INFO] [1766198124.010677113] [load_tester]:     Static (simple):  0.031 Nm
[INFO] [1766198124.011185023] [load_tester]:     FK error:         0.000 Nm (+0.0%)
[INFO] [1766198124.011695099] [load_tester]:     Acceleration:     0.005 Nm
[INFO] [1766198124.012219639] [load_tester]:     Friction:         0.012 Nm
[INFO] [1766198124.012714401] [load_tester]:     ───────────────────────
[INFO] [1766198124.013228107] [load_tester]:     Total required:   0.048 Nm
[INFO] [1766198124.013718924] [load_tester]: 
  Servo capacity:
[INFO] [1766198124.014234575] [load_tester]:     Back-EMF loss:    0.038 Nm
[INFO] [1766198124.014744892] [load_tester]:     Available:        0.352 Nm
[INFO] [1766198124.015238765] [load_tester]:     Utilization:      13.6%
[INFO] [1766198124.015725008] [load_tester]:     ✓ SAFE
[INFO] [1766198124.016226418] [load_tester]: 
WRIST_ROLL
[INFO] [1766198124.016700568] [load_tester]:   Servo: XL330-M288
[INFO] [1766198124.017240645] [load_tester]:   Movement: 5.4° in 0.19 rad/s avg
[INFO] [1766198124.017738406] [load_tester]: 
  Torque breakdown:
[INFO] [1766198124.018420484] [load_tester]:     Static (FK):      0.005 Nm
[INFO] [1766198124.019106024] [load_tester]:     Static (simple):  0.005 Nm
[INFO] [1766198124.019747731] [load_tester]:     FK error:         0.000 Nm (+0.0%)
[INFO] [1766198124.020267604] [load_tester]:     Acceleration:     0.000 Nm
[INFO] [1766198124.020786403] [load_tester]:     Friction:         0.009 Nm
[INFO] [1766198124.021281701] [load_tester]:     ───────────────────────
[INFO] [1766198124.021775870] [load_tester]:     Total required:   0.014 Nm
[INFO] [1766198124.022268188] [load_tester]: 
  Servo capacity:
[INFO] [1766198124.022780764] [load_tester]:     Back-EMF loss:    0.015 Nm
[INFO] [1766198124.023281526] [load_tester]:     Available:        0.375 Nm
[INFO] [1766198124.023816788] [load_tester]:     Utilization:      3.6%
[INFO] [1766198124.024326012] [load_tester]:     ✓ SAFE
[INFO] [1766198124.024833978] [load_tester]: 
PEN_HOLDER
[INFO] [1766198124.025317925] [load_tester]:   Servo: XL330-M288
[INFO] [1766198124.025822316] [load_tester]:   Movement: 27.5° in 0.96 rad/s avg
[INFO] [1766198124.026304077] [load_tester]: 
  Torque breakdown:
[INFO] [1766198124.026795691] [load_tester]:     Static (FK):      0.001 Nm
[INFO] [1766198124.027277360] [load_tester]:     Static (simple):  0.001 Nm
[INFO] [1766198124.027763307] [load_tester]:     FK error:         0.000 Nm (+0.0%)
[INFO] [1766198124.028240161] [load_tester]:     Acceleration:     0.000 Nm
[INFO] [1766198124.028719682] [load_tester]:     Friction:         0.007 Nm
[INFO] [1766198124.029208517] [load_tester]:     ───────────────────────
[INFO] [1766198124.029694112] [load_tester]:     Total required:   0.007 Nm
[INFO] [1766198124.030175541] [load_tester]: 
  Servo capacity:
[INFO] [1766198124.030784469] [load_tester]:     Back-EMF loss:    0.078 Nm
[INFO] [1766198124.031483251] [load_tester]:     Available:        0.312 Nm
[INFO] [1766198124.032154698] [load_tester]:     Utilization:      2.4%
[INFO] [1766198124.032683923] [load_tester]:     ✓ SAFE
[INFO] [1766198124.033194351] [load_tester]: 
============================================================
[INFO] [1766198124.033682039] [load_tester]: SUMMARY
[INFO] [1766198124.034179319] [load_tester]: ============================================================
[INFO] [1766198124.034677470] [load_tester]: Worst case: elbow_flex at 24.0%
[INFO] [1766198124.035174528] [load_tester]: ✓ SAFE - All joints within limits
[WARN] [1766198124.035814660] [load_tester]: 
⚠️  FK correction significant for shoulder_lift: +490.8%
[WARN] [1766198124.036398811] [load_tester]:    Simplified single-joint model would be inaccurate!
[INFO] [1766198124.037239390] [load_tester]: ✓ Results appended to movement.tst.csv
[INFO] [1766198124.037786855] [load_tester]: 
============================================================
[INFO] [1766198124.038272765] [load_tester]: TRANSITION 2/5: pose 1 → pose 2
[INFO] [1766198124.038753452] [load_tester]: ============================================================
[INFO] [1766198124.039219121] [load_tester]: 
============================================================
[INFO] [1766198124.039694050] [load_tester]: FULL POSE TRANSITION ANALYSIS
[INFO] [1766198124.040178274] [load_tester]: ============================================================
[INFO] [1766198124.040656184] [load_tester]: Movement time: 0.5s
[INFO] [1766198124.041147649] [load_tester]: 
From pose:
[INFO] [1766198124.041768671] [load_tester]:   shoulder_pan   :  1.168 rad (  66.9°)
[INFO] [1766198124.042365637] [load_tester]:   shoulder_lift  :  2.500 rad ( 143.2°)
[INFO] [1766198124.042929065] [load_tester]:   elbow_flex     :  1.193 rad (  68.4°)
[INFO] [1766198124.043685328] [load_tester]:   wrist_flex     :  2.456 rad ( 140.7°)
[INFO] [1766198124.044449869] [load_tester]:   wrist_roll     :  0.925 rad (  53.0°)
[INFO] [1766198124.045076927] [load_tester]:   pen_holder     :  1.580 rad (  90.5°)
[INFO] [1766198124.045590856] [load_tester]: 
To pose:
[INFO] [1766198124.046131525] [load_tester]:   shoulder_pan   :  1.987 rad ( 113.9°)
[INFO] [1766198124.046653528] [load_tester]:   shoulder_lift  :  2.500 rad ( 143.2°)
[INFO] [1766198124.047200734] [load_tester]:   elbow_flex     :  1.000 rad (  57.3°)
[INFO] [1766198124.047741866] [load_tester]:   wrist_flex     :  1.591 rad (  91.1°)
[INFO] [1766198124.048271554] [load_tester]:   wrist_roll     : -1.408 rad ( -80.7°)
[INFO] [1766198124.048802686] [load_tester]:   pen_holder     :  0.210 rad (  12.0°)
[INFO] [1766198124.049935340] [load_tester]: 
============================================================
[INFO] [1766198124.050452676] [load_tester]: JOINT-BY-JOINT ANALYSIS
[INFO] [1766198124.050973863] [load_tester]: ============================================================
[INFO] [1766198124.051475718] [load_tester]: 
SHOULDER_PAN
[INFO] [1766198124.051987183] [load_tester]:   Servo: XL430-W250
[INFO] [1766198124.052504167] [load_tester]:   Movement: 47.0° in 1.64 rad/s avg
[INFO] [1766198124.053020058] [load_tester]: 
  Torque breakdown:
[INFO] [1766198124.053541413] [load_tester]:     Static (FK):      0.000 Nm
[INFO] [1766198124.054053415] [load_tester]:     Static (simple):  0.000 Nm
[INFO] [1766198124.054565844] [load_tester]:     FK error:         0.000 Nm (+0.0%)
[INFO] [1766198124.055081494] [load_tester]:     Acceleration:     0.000 Nm
[INFO] [1766198124.055598126] [load_tester]:     Friction:         0.036 Nm
[INFO] [1766198124.056117499] [load_tester]:     ───────────────────────
[INFO] [1766198124.056617446] [load_tester]:     Total required:   0.036 Nm
[INFO] [1766198124.057244153] [load_tester]: 
  Servo capacity:
[INFO] [1766198124.057954638] [load_tester]:     Back-EMF loss:    0.466 Nm
[INFO] [1766198124.058560215] [load_tester]:     Available:        0.934 Nm
[INFO] [1766198124.059111625] [load_tester]:     Utilization:      3.9%
[INFO] [1766198124.059625238] [load_tester]:     ✓ SAFE
[INFO] [1766198124.060144667] [load_tester]: 
ELBOW_FLEX
[INFO] [1766198124.060716096] [load_tester]:   Servo: XL330-M288
[INFO] [1766198124.061453321] [load_tester]:   Movement: 11.1° in 0.39 rad/s avg
[INFO] [1766198124.062014805] [load_tester]: 
  Torque breakdown:
[INFO] [1766198124.062550808] [load_tester]:     Static (FK):      0.157 Nm
[INFO] [1766198124.063104755] [load_tester]:     Static (simple):  0.142 Nm
[INFO] [1766198124.063673739] [load_tester]:     FK error:         0.015 Nm (+9.7%)
[INFO] [1766198124.064232779] [load_tester]:     Acceleration:     0.018 Nm
[INFO] [1766198124.064818541] [load_tester]:     Friction:         0.018 Nm
[INFO] [1766198124.065453952] [load_tester]:     ───────────────────────
[INFO] [1766198124.066149825] [load_tester]:     Total required:   0.193 Nm
[INFO] [1766198124.066679772] [load_tester]: 
  Servo capacity:
[INFO] [1766198124.067193016] [load_tester]:     Back-EMF loss:    0.031 Nm
[INFO] [1766198124.067693888] [load_tester]:     Available:        0.359 Nm
[INFO] [1766198124.068234798] [load_tester]:     Utilization:      53.9%
[WARN] [1766198124.068968117] [load_tester]:     ⚠️  MODERATE
[INFO] [1766198124.069509471] [load_tester]: 
WRIST_FLEX
[INFO] [1766198124.070030103] [load_tester]:   Servo: XL330-M288
[INFO] [1766198124.070560328] [load_tester]:   Movement: 49.5° in 1.73 rad/s avg
[INFO] [1766198124.071275072] [load_tester]: 
  Torque breakdown:
[INFO] [1766198124.071993965] [load_tester]:     Static (FK):      0.031 Nm
[INFO] [1766198124.072604931] [load_tester]:     Static (simple):  0.031 Nm
[INFO] [1766198124.073176193] [load_tester]:     FK error:         0.000 Nm (+0.0%)
[INFO] [1766198124.073692492] [load_tester]:     Acceleration:     0.018 Nm
[INFO] [1766198124.074232124] [load_tester]:     Friction:         0.019 Nm
[INFO] [1766198124.074774163] [load_tester]:     ───────────────────────
[INFO] [1766198124.075286944] [load_tester]:     Total required:   0.067 Nm
[INFO] [1766198124.075808668] [load_tester]: 
  Servo capacity:
[INFO] [1766198124.076312041] [load_tester]:     Back-EMF loss:    0.140 Nm
[INFO] [1766198124.076849710] [load_tester]:     Available:        0.250 Nm
[INFO] [1766198124.077408046] [load_tester]:     Utilization:      27.0%
[INFO] [1766198124.077983957] [load_tester]:     ✓ SAFE
[INFO] [1766198124.078547496] [load_tester]: 
WRIST_ROLL
[INFO] [1766198124.079087184] [load_tester]:   Servo: XL330-M288
[INFO] [1766198124.079647687] [load_tester]:   Movement: 133.7° in 4.67 rad/s avg
[INFO] [1766198124.080159708] [load_tester]: 
  Torque breakdown:
[INFO] [1766198124.080659562] [load_tester]:     Static (FK):      0.005 Nm
[INFO] [1766198124.081151713] [load_tester]:     Static (simple):  0.005 Nm
[INFO] [1766198124.081641789] [load_tester]:     FK error:         0.000 Nm (+0.0%)
[INFO] [1766198124.082165310] [load_tester]:     Acceleration:     0.004 Nm
[INFO] [1766198124.082739220] [load_tester]:     Friction:         0.022 Nm
[INFO] [1766198124.083270075] [load_tester]:     ───────────────────────
[INFO] [1766198124.083779040] [load_tester]:     Total required:   0.031 Nm
[INFO] [1766198124.084261524] [load_tester]: 
  Servo capacity:
[INFO] [1766198124.084750008] [load_tester]:     Back-EMF loss:    0.378 Nm
[INFO] [1766198124.085256214] [load_tester]:     Available:        0.012 Nm
[INFO] [1766198124.085913199] [load_tester]:     Utilization:      258.4%
[ERROR] [1766198124.086578313] [load_tester]:     ❌ OVERLOADED
[INFO] [1766198124.087130279] [load_tester]: 
PEN_HOLDER
[INFO] [1766198124.087631651] [load_tester]:   Servo: XL330-M288
[INFO] [1766198124.088210636] [load_tester]:   Movement: 78.5° in 2.74 rad/s avg
[INFO] [1766198124.088716786] [load_tester]: 
  Torque breakdown:
[INFO] [1766198124.089231474] [load_tester]:     Static (FK):      0.001 Nm
[INFO] [1766198124.089760791] [load_tester]:     Static (simple):  0.001 Nm
[INFO] [1766198124.090280423] [load_tester]:     FK error:         0.000 Nm (+0.0%)
[INFO] [1766198124.090787426] [load_tester]:     Acceleration:     0.000 Nm
[INFO] [1766198124.091300540] [load_tester]:     Friction:         0.010 Nm
[INFO] [1766198124.091823616] [load_tester]:     ───────────────────────
[INFO] [1766198124.092326711] [load_tester]:     Total required:   0.011 Nm
[INFO] [1766198124.092836584] [load_tester]: 
  Servo capacity:
[INFO] [1766198124.093362661] [load_tester]:     Back-EMF loss:    0.222 Nm
[INFO] [1766198124.093889256] [load_tester]:     Available:        0.168 Nm
[INFO] [1766198124.094386203] [load_tester]:     Utilization:      6.6%
[INFO] [1766198124.094886798] [load_tester]:     ✓ SAFE
[INFO] [1766198124.095380689] [load_tester]: 
============================================================
[INFO] [1766198124.095928451] [load_tester]: SUMMARY
[INFO] [1766198124.096408916] [load_tester]: ============================================================
[INFO] [1766198124.096960122] [load_tester]: Worst case: wrist_roll at 258.4%
[ERROR] [1766198124.097496181] [load_tester]: ❌ UNSAFE - Servo may stall or be damaged
[WARN] [1766198124.098123369] [load_tester]: 
⚠️  FK correction significant for shoulder_lift: +174.9%
[WARN] [1766198124.098706538] [load_tester]:    Simplified single-joint model would be inaccurate!
[INFO] [1766198124.099556265] [load_tester]: ✓ Results appended to movement.tst.csv
[INFO] [1766198124.100085749] [load_tester]: 
============================================================
[INFO] [1766198124.100603788] [load_tester]: TRANSITION 3/5: pose 2 → pose 3
[INFO] [1766198124.101261865] [load_tester]: ============================================================
[INFO] [1766198124.101948813] [load_tester]: 
============================================================
[INFO] [1766198124.102616557] [load_tester]: FULL POSE TRANSITION ANALYSIS
[INFO] [1766198124.103212504] [load_tester]: ============================================================
[INFO] [1766198124.103738007] [load_tester]: Movement time: 0.5s
[INFO] [1766198124.104214361] [load_tester]: 
From pose:
[INFO] [1766198124.104753975] [load_tester]:   shoulder_pan   :  1.987 rad ( 113.9°)
[INFO] [1766198124.105259996] [load_tester]:   shoulder_lift  :  2.500 rad ( 143.2°)
[INFO] [1766198124.105780924] [load_tester]:   elbow_flex     :  1.000 rad (  57.3°)
[INFO] [1766198124.106287464] [load_tester]:   wrist_flex     :  1.591 rad (  91.1°)
[INFO] [1766198124.106851189] [load_tester]:   wrist_roll     : -1.408 rad ( -80.7°)
[INFO] [1766198124.107375895] [load_tester]:   pen_holder     :  0.210 rad (  12.0°)
[INFO] [1766198124.107880027] [load_tester]: 
To pose:
[INFO] [1766198124.108385548] [load_tester]:   shoulder_pan   :  1.453 rad (  83.3°)
[INFO] [1766198124.108903939] [load_tester]:   shoulder_lift  :  2.560 rad ( 146.7°)
[INFO] [1766198124.109405849] [load_tester]:   elbow_flex     :  1.105 rad (  63.3°)
[INFO] [1766198124.109917629] [load_tester]:   wrist_flex     :  1.299 rad (  74.4°)
[INFO] [1766198124.110422947] [load_tester]:   wrist_roll     : -0.373 rad ( -21.4°)
[INFO] [1766198124.110936375] [load_tester]:   pen_holder     :  1.590 rad (  91.1°)
[INFO] [1766198124.112021214] [load_tester]: 
============================================================
[INFO] [1766198124.112508216] [load_tester]: JOINT-BY-JOINT ANALYSIS
[INFO] [1766198124.113005607] [load_tester]: ============================================================
[INFO] [1766198124.113496424] [load_tester]: 
SHOULDER_PAN
[INFO] [1766198124.114003038] [load_tester]:   Servo: XL430-W250
[INFO] [1766198124.114513281] [load_tester]:   Movement: 30.6° in 1.07 rad/s avg
[INFO] [1766198124.115010117] [load_tester]: 
  Torque breakdown:
[INFO] [1766198124.115496416] [load_tester]:     Static (FK):      0.000 Nm
[INFO] [1766198124.115975362] [load_tester]:     Static (simple):  0.000 Nm
[INFO] [1766198124.116467087] [load_tester]:     FK error:         0.000 Nm (+0.0%)
[INFO] [1766198124.117036534] [load_tester]:     Acceleration:     0.000 Nm
[INFO] [1766198124.117667963] [load_tester]:     Friction:         0.031 Nm
[INFO] [1766198124.118328448] [load_tester]:     ───────────────────────
[INFO] [1766198124.118915747] [load_tester]:     Total required:   0.031 Nm
[INFO] [1766198124.119421157] [load_tester]: 
  Servo capacity:
[INFO] [1766198124.119935048] [load_tester]:     Back-EMF loss:    0.304 Nm
[INFO] [1766198124.120429032] [load_tester]:     Available:        1.096 Nm
[INFO] [1766198124.120939683] [load_tester]:     Utilization:      2.8%
[INFO] [1766198124.121442296] [load_tester]:     ✓ SAFE
[INFO] [1766198124.121963928] [load_tester]: 
SHOULDER_LIFT
[INFO] [1766198124.122456986] [load_tester]:   Servo: XL430-W250
[INFO] [1766198124.123052656] [load_tester]:   Movement: 3.4° in 0.12 rad/s avg
[INFO] [1766198124.123554269] [load_tester]: 
  Torque breakdown:
[INFO] [1766198124.124065883] [load_tester]:     Static (FK):      0.170 Nm
[INFO] [1766198124.124600923] [load_tester]:     Static (simple):  0.483 Nm
[INFO] [1766198124.125116610] [load_tester]:     FK error:         0.313 Nm (+184.0%)
[INFO] [1766198124.125630391] [load_tester]:     Acceleration:     0.016 Nm
[INFO] [1766198124.126126837] [load_tester]:     Friction:         0.032 Nm
[INFO] [1766198124.126632914] [load_tester]:     ───────────────────────
[INFO] [1766198124.127149639] [load_tester]:     Total required:   0.218 Nm
[INFO] [1766198124.127647474] [load_tester]: 
  Servo capacity:
[INFO] [1766198124.128156162] [load_tester]:     Back-EMF loss:    0.034 Nm
[INFO] [1766198124.128661313] [load_tester]:     Available:        1.366 Nm
[INFO] [1766198124.129172204] [load_tester]:     Utilization:      16.0%
[INFO] [1766198124.129675188] [load_tester]:     ✓ SAFE
[INFO] [1766198124.130203894] [load_tester]: 
ELBOW_FLEX
[INFO] [1766198124.130695748] [load_tester]:   Servo: XL330-M288
[INFO] [1766198124.131228325] [load_tester]:   Movement: 6.0° in 0.21 rad/s avg
[INFO] [1766198124.131717124] [load_tester]: 
  Torque breakdown:
[INFO] [1766198124.132228774] [load_tester]:     Static (FK):      0.191 Nm
[INFO] [1766198124.132720721] [load_tester]:     Static (simple):  0.142 Nm
[INFO] [1766198124.133309465] [load_tester]:     FK error:         0.049 Nm (+25.6%)
[INFO] [1766198124.133926005] [load_tester]:     Acceleration:     0.010 Nm
[INFO] [1766198124.134591878] [load_tester]:     Friction:         0.017 Nm
[INFO] [1766198124.135145807] [load_tester]:     ───────────────────────
[INFO] [1766198124.135658920] [load_tester]:     Total required:   0.217 Nm
[INFO] [1766198124.136164793] [load_tester]: 
  Servo capacity:
[INFO] [1766198124.136644184] [load_tester]:     Back-EMF loss:    0.017 Nm
[INFO] [1766198124.137158446] [load_tester]:     Available:        0.373 Nm
[INFO] [1766198124.137653337] [load_tester]:     Utilization:      58.3%
[WARN] [1766198124.138232655] [load_tester]:     ⚠️  MODERATE
[INFO] [1766198124.138745509] [load_tester]: 
WRIST_FLEX
[INFO] [1766198124.139227382] [load_tester]:   Servo: XL330-M288
[INFO] [1766198124.139761625] [load_tester]:   Movement: 16.7° in 0.58 rad/s avg
[INFO] [1766198124.140251017] [load_tester]: 
  Torque breakdown:
[INFO] [1766198124.140755352] [load_tester]:     Static (FK):      0.031 Nm
[INFO] [1766198124.141233299] [load_tester]:     Static (simple):  0.031 Nm
[INFO] [1766198124.141705116] [load_tester]:     FK error:         0.000 Nm (+0.0%)
[INFO] [1766198124.142203119] [load_tester]:     Acceleration:     0.006 Nm
[INFO] [1766198124.142690195] [load_tester]:     Friction:         0.013 Nm
[INFO] [1766198124.143209383] [load_tester]:     ───────────────────────
[INFO] [1766198124.143701126] [load_tester]:     Total required:   0.050 Nm
[INFO] [1766198124.144196702] [load_tester]: 
  Servo capacity:
[INFO] [1766198124.144684057] [load_tester]:     Back-EMF loss:    0.047 Nm
[INFO] [1766198124.145192004] [load_tester]:     Available:        0.343 Nm
[INFO] [1766198124.145682950] [load_tester]:     Utilization:      14.5%
[INFO] [1766198124.146163453] [load_tester]:     ✓ SAFE
[INFO] [1766198124.146660066] [load_tester]: 
WRIST_ROLL
[INFO] [1766198124.147162087] [load_tester]:   Servo: XL330-M288
[INFO] [1766198124.147690330] [load_tester]:   Movement: 59.3° in 2.07 rad/s avg
[INFO] [1766198124.148180314] [load_tester]: 
  Torque breakdown:
[INFO] [1766198124.148679298] [load_tester]:     Static (FK):      0.005 Nm
[INFO] [1766198124.149194060] [load_tester]:     Static (simple):  0.005 Nm
[INFO] [1766198124.149687340] [load_tester]:     FK error:         0.000 Nm (+0.0%)
[INFO] [1766198124.150250102] [load_tester]:     Acceleration:     0.002 Nm
[INFO] [1766198124.150960420] [load_tester]:     Friction:         0.014 Nm
[INFO] [1766198124.151673072] [load_tester]:     ───────────────────────
[INFO] [1766198124.152251686] [load_tester]:     Total required:   0.021 Nm
[INFO] [1766198124.152766577] [load_tester]: 
  Servo capacity:
[INFO] [1766198124.153282080] [load_tester]:     Back-EMF loss:    0.168 Nm
[INFO] [1766198124.153840638] [load_tester]:     Available:        0.222 Nm
[INFO] [1766198124.154361233] [load_tester]:     Utilization:      9.5%
[INFO] [1766198124.154870606] [load_tester]:     ✓ SAFE
[INFO] [1766198124.155362108] [load_tester]: 
PEN_HOLDER
[INFO] [1766198124.155866296] [load_tester]:   Servo: XL330-M288
[INFO] [1766198124.156408169] [load_tester]:   Movement: 79.1° in 2.76 rad/s avg
[INFO] [1766198124.156918375] [load_tester]: 
  Torque breakdown:
[INFO] [1766198124.157412100] [load_tester]:     Static (FK):      0.001 Nm
[INFO] [1766198124.157896028] [load_tester]:     Static (simple):  0.001 Nm
[INFO] [1766198124.158364493] [load_tester]:     FK error:         0.000 Nm (+0.0%)
[INFO] [1766198124.158860662] [load_tester]:     Acceleration:     0.000 Nm
[INFO] [1766198124.159351516] [load_tester]:     Friction:         0.011 Nm
[INFO] [1766198124.159865834] [load_tester]:     ───────────────────────
[INFO] [1766198124.160354040] [load_tester]:     Total required:   0.011 Nm
[INFO] [1766198124.160862209] [load_tester]: 
  Servo capacity:
[INFO] [1766198124.161360396] [load_tester]:     Back-EMF loss:    0.223 Nm
[INFO] [1766198124.161865417] [load_tester]:     Available:        0.167 Nm
[INFO] [1766198124.162356327] [load_tester]:     Utilization:      6.7%
[INFO] [1766198124.162859737] [load_tester]:     ✓ SAFE
[INFO] [1766198124.163350313] [load_tester]: 
============================================================
[INFO] [1766198124.163889668] [load_tester]: SUMMARY
[INFO] [1766198124.164382300] [load_tester]: ============================================================
[INFO] [1766198124.164903636] [load_tester]: Worst case: elbow_flex at 58.3%
[WARN] [1766198124.165476231] [load_tester]: ⚠️  MODERATE - Monitor temperature
[WARN] [1766198124.166071845] [load_tester]: 
⚠️  FK correction significant for shoulder_lift: +184.0%
[WARN] [1766198124.166648663] [load_tester]:    Simplified single-joint model would be inaccurate!
[INFO] [1766198124.167703075] [load_tester]: ✓ Results appended to movement.tst.csv
[INFO] [1766198124.168401523] [load_tester]: 
============================================================
[INFO] [1766198124.169022193] [load_tester]: TRANSITION 4/5: pose 3 → pose 4
[INFO] [1766198124.169548677] [load_tester]: ============================================================
[INFO] [1766198124.170074976] [load_tester]: 
============================================================
[INFO] [1766198124.170576756] [load_tester]: FULL POSE TRANSITION ANALYSIS
[INFO] [1766198124.171087443] [load_tester]: ============================================================
[INFO] [1766198124.171593075] [load_tester]: Movement time: 0.5s
[INFO] [1766198124.172100393] [load_tester]: 
From pose:
[INFO] [1766198124.172635988] [load_tester]:   shoulder_pan   :  1.453 rad (  83.3°)
[INFO] [1766198124.173170528] [load_tester]:   shoulder_lift  :  2.560 rad ( 146.7°)
[INFO] [1766198124.173688845] [load_tester]:   elbow_flex     :  1.105 rad (  63.3°)
[INFO] [1766198124.174201570] [load_tester]:   wrist_flex     :  1.299 rad (  74.4°)
[INFO] [1766198124.174704035] [load_tester]:   wrist_roll     : -0.373 rad ( -21.4°)
[INFO] [1766198124.175225278] [load_tester]:   pen_holder     :  1.590 rad (  91.1°)
[INFO] [1766198124.175712595] [load_tester]: 
To pose:
[INFO] [1766198124.176234820] [load_tester]:   shoulder_pan   :  1.812 rad ( 103.8°)
[INFO] [1766198124.176749545] [load_tester]:   shoulder_lift  :  2.560 rad ( 146.7°)
[INFO] [1766198124.177242436] [load_tester]:   elbow_flex     :  1.632 rad (  93.5°)
[INFO] [1766198124.177737920] [load_tester]:   wrist_flex     :  2.440 rad ( 139.8°)
[INFO] [1766198124.178288552] [load_tester]:   wrist_roll     :  1.192 rad (  68.3°)
[INFO] [1766198124.178806407] [load_tester]:   pen_holder     :  0.500 rad (  28.6°)
[INFO] [1766198124.179918097] [load_tester]: 
============================================================
[INFO] [1766198124.180410118] [load_tester]: JOINT-BY-JOINT ANALYSIS
[INFO] [1766198124.180917139] [load_tester]: ============================================================
[INFO] [1766198124.181400845] [load_tester]: 
SHOULDER_PAN
[INFO] [1766198124.181890644] [load_tester]:   Servo: XL430-W250
[INFO] [1766198124.182386442] [load_tester]:   Movement: 20.6° in 0.72 rad/s avg
[INFO] [1766198124.182867241] [load_tester]: 
  Torque breakdown:
[INFO] [1766198124.183336503] [load_tester]:     Static (FK):      0.000 Nm
[INFO] [1766198124.183819005] [load_tester]:     Static (simple):  0.000 Nm
[INFO] [1766198124.184298044] [load_tester]:     FK error:         0.000 Nm (+0.0%)
[INFO] [1766198124.184884769] [load_tester]:     Acceleration:     0.000 Nm
[INFO] [1766198124.185475643] [load_tester]:     Friction:         0.027 Nm
[INFO] [1766198124.186041404] [load_tester]:     ───────────────────────
[INFO] [1766198124.186563759] [load_tester]:     Total required:   0.027 Nm
[INFO] [1766198124.187062984] [load_tester]: 
  Servo capacity:
[INFO] [1766198124.187549912] [load_tester]:     Back-EMF loss:    0.204 Nm
[INFO] [1766198124.188058248] [load_tester]:     Available:        1.196 Nm
[INFO] [1766198124.188527787] [load_tester]:     Utilization:      2.3%
[INFO] [1766198124.189006789] [load_tester]:     ✓ SAFE
[INFO] [1766198124.189478329] [load_tester]: 
ELBOW_FLEX
[INFO] [1766198124.189964349] [load_tester]:   Servo: XL330-M288
[INFO] [1766198124.190497778] [load_tester]:   Movement: 30.2° in 1.05 rad/s avg
[INFO] [1766198124.190993002] [load_tester]: 
  Torque breakdown:
[INFO] [1766198124.191470523] [load_tester]:     Static (FK):      0.116 Nm
[INFO] [1766198124.191954415] [load_tester]:     Static (simple):  0.142 Nm
[INFO] [1766198124.192439972] [load_tester]:     FK error:         0.026 Nm (+22.5%)
[INFO] [1766198124.192930086] [load_tester]:     Acceleration:     0.049 Nm
[INFO] [1766198124.193419718] [load_tester]:     Friction:         0.023 Nm
[INFO] [1766198124.193952832] [load_tester]:     ───────────────────────
[INFO] [1766198124.194460260] [load_tester]:     Total required:   0.188 Nm
[INFO] [1766198124.194946799] [load_tester]: 
  Servo capacity:
[INFO] [1766198124.195436802] [load_tester]:     Back-EMF loss:    0.085 Nm
[INFO] [1766198124.195942638] [load_tester]:     Available:        0.305 Nm
[INFO] [1766198124.196434473] [load_tester]:     Utilization:      61.7%
[WARN] [1766198124.197040661] [load_tester]:     ⚠️  MODERATE
[INFO] [1766198124.197532460] [load_tester]: 
WRIST_FLEX
[INFO] [1766198124.198047314] [load_tester]:   Servo: XL330-M288
[INFO] [1766198124.198572539] [load_tester]:   Movement: 65.4° in 2.28 rad/s avg
[INFO] [1766198124.199071338] [load_tester]: 
  Torque breakdown:
[INFO] [1766198124.199562174] [load_tester]:     Static (FK):      0.031 Nm
[INFO] [1766198124.200060583] [load_tester]:     Static (simple):  0.031 Nm
[INFO] [1766198124.200555641] [load_tester]:     FK error:         0.000 Nm (+0.0%)
[INFO] [1766198124.201060977] [load_tester]:     Acceleration:     0.024 Nm
[INFO] [1766198124.201554757] [load_tester]:     Friction:         0.021 Nm
[INFO] [1766198124.202064463] [load_tester]:     ───────────────────────
[INFO] [1766198124.202617577] [load_tester]:     Total required:   0.076 Nm
[INFO] [1766198124.203295951] [load_tester]: 
  Servo capacity:
[INFO] [1766198124.203952621] [load_tester]:     Back-EMF loss:    0.185 Nm
[INFO] [1766198124.204496716] [load_tester]:     Available:        0.205 Nm
[INFO] [1766198124.205018829] [load_tester]:     Utilization:      37.1%
[INFO] [1766198124.205516850] [load_tester]:     ✓ SAFE
[INFO] [1766198124.206116705] [load_tester]: 
WRIST_ROLL
[INFO] [1766198124.206607615] [load_tester]:   Servo: XL330-M288
[INFO] [1766198124.207134080] [load_tester]:   Movement: 89.6° in 3.13 rad/s avg
[INFO] [1766198124.207627231] [load_tester]: 
  Torque breakdown:
[INFO] [1766198124.208126585] [load_tester]:     Static (FK):      0.005 Nm
[INFO] [1766198124.208615884] [load_tester]:     Static (simple):  0.005 Nm
[INFO] [1766198124.209126553] [load_tester]:     FK error:         0.000 Nm (+0.0%)
[INFO] [1766198124.209633630] [load_tester]:     Acceleration:     0.003 Nm
[INFO] [1766198124.210148169] [load_tester]:     Friction:         0.017 Nm
[INFO] [1766198124.210650986] [load_tester]:     ───────────────────────
[INFO] [1766198124.211159341] [load_tester]:     Total required:   0.025 Nm
[INFO] [1766198124.211648639] [load_tester]: 
  Servo capacity:
[INFO] [1766198124.212131438] [load_tester]:     Back-EMF loss:    0.253 Nm
[INFO] [1766198124.212646477] [load_tester]:     Available:        0.137 Nm
[INFO] [1766198124.213138869] [load_tester]:     Utilization:      18.5%
[INFO] [1766198124.213628223] [load_tester]:     ✓ SAFE
[INFO] [1766198124.214125096] [load_tester]: 
PEN_HOLDER
[INFO] [1766198124.214626487] [load_tester]:   Servo: XL330-M288
[INFO] [1766198124.215150693] [load_tester]:   Movement: 62.5° in 2.18 rad/s avg
[INFO] [1766198124.215637029] [load_tester]: 
  Torque breakdown:
[INFO] [1766198124.216137883] [load_tester]:     Static (FK):      0.001 Nm
[INFO] [1766198124.216624923] [load_tester]:     Static (simple):  0.001 Nm
[INFO] [1766198124.217127147] [load_tester]:     FK error:         0.000 Nm (+0.0%)
[INFO] [1766198124.217617076] [load_tester]:     Acceleration:     0.000 Nm
[INFO] [1766198124.218123004] [load_tester]:     Friction:         0.009 Nm
[INFO] [1766198124.218617914] [load_tester]:     ───────────────────────
[INFO] [1766198124.219123527] [load_tester]:     Total required:   0.010 Nm
[INFO] [1766198124.219609530] [load_tester]: 
  Servo capacity:
[INFO] [1766198124.220116884] [load_tester]:     Back-EMF loss:    0.176 Nm
[INFO] [1766198124.220607053] [load_tester]:     Available:        0.214 Nm
[INFO] [1766198124.221211611] [load_tester]:     Utilization:      4.7%
[INFO] [1766198124.221908133] [load_tester]:     ✓ SAFE
[INFO] [1766198124.222522377] [load_tester]: 
============================================================
[INFO] [1766198124.223138898] [load_tester]: SUMMARY
[INFO] [1766198124.223663808] [load_tester]: ============================================================
[INFO] [1766198124.224213441] [load_tester]: Worst case: elbow_flex at 61.7%
[WARN] [1766198124.224810110] [load_tester]: ⚠️  MODERATE - Monitor temperature
[WARN] [1766198124.225400372] [load_tester]: 
⚠️  FK correction significant for shoulder_lift: +413.4%
[WARN] [1766198124.225982820] [load_tester]:    Simplified single-joint model would be inaccurate!
[INFO] [1766198124.226841435] [load_tester]: ✓ Results appended to movement.tst.csv
[INFO] [1766198124.227364252] [load_tester]: 
============================================================
[INFO] [1766198124.227880755] [load_tester]: TRANSITION 5/5: pose 4 → pose 5
[INFO] [1766198124.228379294] [load_tester]: ============================================================
[INFO] [1766198124.228887982] [load_tester]: 
============================================================
[INFO] [1766198124.229380484] [load_tester]: FULL POSE TRANSITION ANALYSIS
[INFO] [1766198124.229879413] [load_tester]: ============================================================
[INFO] [1766198124.230367582] [load_tester]: Movement time: 0.5s
[INFO] [1766198124.230869510] [load_tester]: 
From pose:
[INFO] [1766198124.231401235] [load_tester]:   shoulder_pan   :  1.812 rad ( 103.8°)
[INFO] [1766198124.231939015] [load_tester]:   shoulder_lift  :  2.560 rad ( 146.7°)
[INFO] [1766198124.232449758] [load_tester]:   elbow_flex     :  1.632 rad (  93.5°)
[INFO] [1766198124.232968131] [load_tester]:   wrist_flex     :  2.440 rad ( 139.8°)
[INFO] [1766198124.233517837] [load_tester]:   wrist_roll     :  1.192 rad (  68.3°)
[INFO] [1766198124.234033914] [load_tester]:   pen_holder     :  0.500 rad (  28.6°)
[INFO] [1766198124.234519657] [load_tester]: 
To pose:
[INFO] [1766198124.235047752] [load_tester]:   shoulder_pan   :  1.812 rad ( 103.8°)
[INFO] [1766198124.235563310] [load_tester]:   shoulder_lift  :  2.560 rad ( 146.7°)
[INFO] [1766198124.236078813] [load_tester]:   elbow_flex     :  1.632 rad (  93.5°)
[INFO] [1766198124.236582593] [load_tester]:   wrist_flex     :  2.440 rad ( 139.8°)
[INFO] [1766198124.237100651] [load_tester]:   wrist_roll     :  1.192 rad (  68.3°)
[INFO] [1766198124.237604357] [load_tester]:   pen_holder     :  1.500 rad (  85.9°)
[INFO] [1766198124.238700474] [load_tester]: 
============================================================
[INFO] [1766198124.239196458] [load_tester]: JOINT-BY-JOINT ANALYSIS
[INFO] [1766198124.239743553] [load_tester]: ============================================================
[INFO] [1766198124.240323074] [load_tester]: 
PEN_HOLDER
[INFO] [1766198124.240832169] [load_tester]:   Servo: XL330-M288
[INFO] [1766198124.241461931] [load_tester]:   Movement: 57.3° in 2.00 rad/s avg
[INFO] [1766198124.241984360] [load_tester]: 
  Torque breakdown:
[INFO] [1766198124.242496529] [load_tester]:     Static (FK):      0.001 Nm
[INFO] [1766198124.243051476] [load_tester]:     Static (simple):  0.001 Nm
[INFO] [1766198124.243555293] [load_tester]:     FK error:         0.000 Nm (+0.0%)
[INFO] [1766198124.244057907] [load_tester]:     Acceleration:     0.000 Nm
[INFO] [1766198124.244544465] [load_tester]:     Friction:         0.009 Nm
[INFO] [1766198124.245048393] [load_tester]:     ───────────────────────
[INFO] [1766198124.245565599] [load_tester]:     Total required:   0.010 Nm
[INFO] [1766198124.246067768] [load_tester]: 
  Servo capacity:
[INFO] [1766198124.246577271] [load_tester]:     Back-EMF loss:    0.162 Nm
[INFO] [1766198124.247086977] [load_tester]:     Available:        0.228 Nm
[INFO] [1766198124.247574091] [load_tester]:     Utilization:      4.2%
[INFO] [1766198124.248073815] [load_tester]:     ✓ SAFE
[INFO] [1766198124.248561447] [load_tester]: 
============================================================
[INFO] [1766198124.249063487] [load_tester]: SUMMARY
[INFO] [1766198124.249552137] [load_tester]: ============================================================
[INFO] [1766198124.250085362] [load_tester]: Worst case: elbow_flex at 29.8%
[INFO] [1766198124.250584605] [load_tester]: ✓ SAFE - All joints within limits
[WARN] [1766198124.251185867] [load_tester]: 
⚠️  FK correction significant for shoulder_lift: +413.4%
[WARN] [1766198124.251762611] [load_tester]:    Simplified single-joint model would be inaccurate!
[INFO] [1766198124.252515466] [load_tester]: ✓ Results appended to movement.tst.csv
[INFO] [1766198124.253042728] [load_tester]: 
================================================================================
[INFO] [1766198124.253559768] [load_tester]: SEQUENCE SUMMARY
[INFO] [1766198124.254095900] [load_tester]: ================================================================================
[INFO] [1766198124.254631958] [load_tester]:  1. pose 0          → pose 1           ✓ elbow_flex       24.0%
[INFO] [1766198124.255164886] [load_tester]:  2. pose 1          → pose 2           ❌ wrist_roll      258.4%
[INFO] [1766198124.255692352] [load_tester]:  3. pose 2          → pose 3           ⚠️ elbow_flex       58.3%
[INFO] [1766198124.256201577] [load_tester]:  4. pose 3          → pose 4           ⚠️ elbow_flex       61.7%
[INFO] [1766198124.256705690] [load_tester]:  5. pose 4          → pose 5           ✓ elbow_flex       29.8%
```

## Example servo motor what-if
TBD

## FAQ

