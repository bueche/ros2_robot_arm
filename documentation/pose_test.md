# Running the Pose Test utility

This utility does a number of tests that include:
1. running through a sequence of pre-defined poses
2. perform range of motion tests of defined joints in the robot system
3. measure various properties of the robot in the current pose (servo temperature, current, voltage, etc)
4. validate poses wrt to the bounds defined in the urdf file


The parameters are:
TBD

An example run through a sequence of poses is as follows:
```bash
rpi5:~/robot_ws$ ros2 run writing_robot_control pose_test --ros-args -p poses_file:=src/writing_robot_description/config/delivery_poses.yaml -p validate:=true -p telemetry:=true -p urdf_file:=src/writing_robot_description/urdf/koch_v11_arm_real.urdf -p delay:=1.0 -p movement_time:=1.0

[INFO] [1766531805.174681953] [pose_test_node]: Loading URDF limits...
[INFO] [1766531805.176593426] [pose_test_node]: ✓ Loaded limits for 6 joints
[INFO] [1766531805.177169207] [pose_test_node]: Initializing telemetry...
[INFO] [1766531805.184463782] [pose_test_node]: ✓ Telemetry initialized (dual-topic mode)
[INFO] [1766531805.185056767] [pose_test_node]:   - Subscribing to /joint_states
[INFO] [1766531805.185615288] [pose_test_node]:   - Subscribing to /dxl_state
[INFO] [1766531805.186236180] [pose_test_node]:   - XL430 servos (IDs 1,2): Load only
[INFO] [1766531805.186825480] [pose_test_node]:   - XL330 servos (IDs 3-6): Current + Load
[INFO] [1766531805.188316376] [pose_test_node]: ✓ Telemetry active
[INFO] [1766531805.188898935] [pose_test_node]: ============================================================
[INFO] [1766531805.189463308] [pose_test_node]: POSE TEST NODE INITIALIZED
[INFO] [1766531805.189989367] [pose_test_node]: ============================================================
[INFO] [1766531805.190477240] [pose_test_node]: Joints: 6
[INFO] [1766531805.190989539] [pose_test_node]: Validation: True
[INFO] [1766531805.191531467] [pose_test_node]: Telemetry: True
[INFO] [1766531805.192066840] [pose_test_node]: Movement time: 1.0s
[INFO] [1766531805.192652844] [pose_test_node]: Delay between poses: 1.0s
[INFO] [1766531805.193310273] [pose_test_node]: ============================================================
[INFO] [1766531805.193959554] [pose_test_node]: Waiting for joint trajectory controller...
[INFO] [1766531807.212278932] [pose_test_node]: ✓ Loaded 11 poses from src/writing_robot_description/config/delivery_poses.yaml
[INFO] [1766531807.213124177] [pose_test_node]: 
============================================================
[INFO] [1766531807.213938515] [pose_test_node]: POSE SEQUENCE TEST
[INFO] [1766531807.214586648] [pose_test_node]: ============================================================
[INFO] [1766531807.215203373] [pose_test_node]: Total poses: 11
[INFO] [1766531807.215733024] [pose_test_node]: Validation: True
[INFO] [1766531807.216232768] [pose_test_node]: Telemetry: True
[INFO] [1766531807.216718585] [pose_test_node]: ============================================================
[INFO] [1766531807.217218217] [pose_test_node]: 
[1/11] pose 1 - poised to work
[INFO] [1766531807.217710812] [pose_test_node]:   ✓ shoulder_pan: 1.558 rad (within [0.540, 2.044])
[INFO] [1766531807.218233445] [pose_test_node]:   ✓ shoulder_lift: 2.879 rad (within [2.200, 2.886])
[INFO] [1766531807.218795040] [pose_test_node]:   ✓ elbow_flex: 2.124 rad (within [0.726, 2.250])
[INFO] [1766531807.219311117] [pose_test_node]:   ✓ wrist_flex: 2.279 rad (within [0.297, 2.700])
[INFO] [1766531807.219804786] [pose_test_node]:   ✓ wrist_roll: 1.182 rad (within [-1.448, 1.900])
[INFO] [1766531807.220301511] [pose_test_node]:   ✓ pen_holder: 1.590 rad (within [0.190, 1.600])
[INFO] [1766531807.221236683] [pose_test_node]: → Sent: pose 1 - poised to work
[INFO] [1766531809.229608472] [pose_test_node]:   📊 Joint States:
[INFO] [1766531809.230426605] [pose_test_node]:   📊 shoulder_pan    [XL430] Pos:  1.558rad Load:  0.00% Volt:12.2V Temp: 33.0°C
[INFO] [1766531809.231160794] [pose_test_node]:   📊 shoulder_lift   [XL430] Pos:  2.835rad Load: 16.30% Volt:12.2V Temp: 41.0°C
[INFO] [1766531809.231872983] [pose_test_node]:   📊 elbow_flex      [XL330] Pos:  2.208rad Curr:  -80mA Volt: 5.2V Temp: 25.0°C
[INFO] [1766531809.232755581] [pose_test_node]:   📊 wrist_flex      [XL330] Pos:  2.255rad Curr:   45mA Volt: 5.2V Temp: 21.0°C
[INFO] [1766531809.233530270] [pose_test_node]:   📊 wrist_roll      [XL330] Pos:  1.192rad Curr:  -32mA Volt: 5.2V Temp: 20.0°C
[INFO] [1766531809.234306403] [pose_test_node]:   📊 pen_holder      [XL330] Pos:  1.545rad Curr:   23mA Volt: 5.1V Temp: 23.0°C
[INFO] [1766531809.235130908] [pose_test_node]: 
[2/11] pose 2 - reach to grab
[INFO] [1766531809.235810356] [pose_test_node]:   ✓ shoulder_pan: 1.283 rad (within [0.540, 2.044])
[INFO] [1766531809.236465600] [pose_test_node]:   ✓ shoulder_lift: 2.314 rad (within [2.200, 2.886])
[INFO] [1766531809.237179659] [pose_test_node]:   ✓ elbow_flex: 1.320 rad (within [0.726, 2.250])
[INFO] [1766531809.237815792] [pose_test_node]:   ✓ wrist_flex: 2.279 rad (within [0.297, 2.700])
[INFO] [1766531809.238436425] [pose_test_node]:   ✓ wrist_roll: 1.871 rad (within [-1.448, 1.900])
[INFO] [1766531809.239075021] [pose_test_node]:   ✓ pen_holder: 0.982 rad (within [0.190, 1.600])
[INFO] [1766531809.240080193] [pose_test_node]: → Sent: pose 2 - reach to grab
[INFO] [1766531811.243569308] [pose_test_node]:   📊 Joint States:
[INFO] [1766531811.244493702] [pose_test_node]:   📊 shoulder_pan    [XL430] Pos:  1.289rad Load: -2.30% Volt:12.3V Temp: 33.0°C
[INFO] [1766531811.245330336] [pose_test_node]:   📊 shoulder_lift   [XL430] Pos:  2.293rad Load:  7.30% Volt:12.2V Temp: 41.0°C
[INFO] [1766531811.246099544] [pose_test_node]:   📊 elbow_flex      [XL330] Pos:  1.527rad Curr: -320mA Volt: 5.1V Temp: 25.0°C
[INFO] [1766531811.246824196] [pose_test_node]:   📊 wrist_flex      [XL330] Pos:  2.255rad Curr:   48mA Volt: 5.1V Temp: 21.0°C
[INFO] [1766531811.247696682] [pose_test_node]:   📊 wrist_roll      [XL330] Pos:  1.865rad Curr:   32mA Volt: 5.1V Temp: 20.0°C
[INFO] [1766531811.248480538] [pose_test_node]:   📊 pen_holder      [XL330] Pos:  1.044rad Curr:  -30mA Volt: 5.0V Temp: 23.0°C
[INFO] [1766531811.249401172] [pose_test_node]: 
[3/11] pose 3 - grab
[INFO] [1766531811.250156528] [pose_test_node]:   ✓ shoulder_pan: 1.329 rad (within [0.540, 2.044])
[INFO] [1766531811.250815328] [pose_test_node]:   ✓ shoulder_lift: 2.322 rad (within [2.200, 2.886])
[INFO] [1766531811.251436164] [pose_test_node]:   ✓ elbow_flex: 1.005 rad (within [0.726, 2.250])
[INFO] [1766531811.252128964] [pose_test_node]:   ✓ wrist_flex: 2.279 rad (within [0.297, 2.700])
[INFO] [1766531811.252940061] [pose_test_node]:   ✓ wrist_roll: 1.766 rad (within [-1.448, 1.900])
[INFO] [1766531811.253620231] [pose_test_node]:   ✓ pen_holder: 1.501 rad (within [0.190, 1.600])
[INFO] [1766531811.254670181] [pose_test_node]: → Sent: pose 3 - grab
[INFO] [1766531813.264756183] [pose_test_node]:   📊 Joint States:
[INFO] [1766531813.265607724] [pose_test_node]:   📊 shoulder_pan    [XL430] Pos:  1.329rad Load:  0.10% Volt:12.2V Temp: 33.0°C
[INFO] [1766531813.266375784] [pose_test_node]:   📊 shoulder_lift   [XL430] Pos:  2.282rad Load: 14.70% Volt:12.2V Temp: 41.0°C
[INFO] [1766531813.267115862] [pose_test_node]:   📊 elbow_flex      [XL330] Pos:  1.252rad Curr: -416mA Volt: 5.1V Temp: 25.0°C
[INFO] [1766531813.267851329] [pose_test_node]:   📊 wrist_flex      [XL330] Pos:  2.255rad Curr:   45mA Volt: 5.1V Temp: 21.0°C
[INFO] [1766531813.268610573] [pose_test_node]:   📊 wrist_roll      [XL330] Pos:  1.766rad Curr:   -5mA Volt: 5.1V Temp: 20.0°C
[INFO] [1766531813.269370151] [pose_test_node]:   📊 pen_holder      [XL330] Pos:  1.456rad Curr:   20mA Volt: 5.0V Temp: 23.0°C
[INFO] [1766531813.270220063] [pose_test_node]: 
[4/11] pose 4 - lift and center
[INFO] [1766531813.270940715] [pose_test_node]:   ✓ shoulder_pan: 1.576 rad (within [0.540, 2.044])
[INFO] [1766531813.271666367] [pose_test_node]:   ✓ shoulder_lift: 2.575 rad (within [2.200, 2.886])
[INFO] [1766531813.272426908] [pose_test_node]:   ✓ elbow_flex: 1.898 rad (within [0.726, 2.250])
[INFO] [1766531813.273391043] [pose_test_node]:   ✓ wrist_flex: 2.279 rad (within [0.297, 2.700])
[INFO] [1766531813.274181065] [pose_test_node]:   ✓ wrist_roll: 1.766 rad (within [-1.448, 1.900])
[INFO] [1766531813.274757142] [pose_test_node]:   ✓ pen_holder: 1.521 rad (within [0.190, 1.600])
[INFO] [1766531813.275652277] [pose_test_node]: → Sent: pose 4 - lift and center
[INFO] [1766531815.282991690] [pose_test_node]:   📊 Joint States:
[INFO] [1766531815.283846454] [pose_test_node]:   📊 shoulder_pan    [XL430] Pos:  1.573rad Load:  1.10% Volt:12.2V Temp: 33.0°C
[INFO] [1766531815.284596569] [pose_test_node]:   📊 shoulder_lift   [XL430] Pos:  2.523rad Load: 18.60% Volt:12.2V Temp: 41.0°C
[INFO] [1766531815.285446721] [pose_test_node]:   📊 elbow_flex      [XL330] Pos:  1.935rad Curr:  -32mA Volt: 5.2V Temp: 25.0°C
[INFO] [1766531815.286199633] [pose_test_node]:   📊 wrist_flex      [XL330] Pos:  2.255rad Curr:   45mA Volt: 5.2V Temp: 21.0°C
[INFO] [1766531815.286929433] [pose_test_node]:   📊 wrist_roll      [XL330] Pos:  1.766rad Curr:    0mA Volt: 5.2V Temp: 20.0°C
[INFO] [1766531815.287680881] [pose_test_node]:   📊 pen_holder      [XL330] Pos:  1.470rad Curr:   25mA Volt: 5.2V Temp: 23.0°C
[INFO] [1766531815.288554312] [pose_test_node]: 
[5/11] pose 5 - raise and get ready to place
[INFO] [1766531815.289293297] [pose_test_node]:   ✓ shoulder_pan: 1.596 rad (within [0.540, 2.044])
[INFO] [1766531815.290025912] [pose_test_node]:   ✓ shoulder_lift: 2.795 rad (within [2.200, 2.886])
[INFO] [1766531815.290732916] [pose_test_node]:   ✓ elbow_flex: 0.970 rad (within [0.726, 2.250])
[INFO] [1766531815.291469068] [pose_test_node]:   ✓ wrist_flex: 1.473 rad (within [0.297, 2.700])
[INFO] [1766531815.292226997] [pose_test_node]:   ✓ wrist_roll: 1.762 rad (within [-1.448, 1.900])
[INFO] [1766531815.293125613] [pose_test_node]:   ✓ pen_holder: 1.521 rad (within [0.190, 1.600])
[INFO] [1766531815.294142341] [pose_test_node]: → Sent: pose 5 - raise and get ready to place
[INFO] [1766531817.298481164] [pose_test_node]:   📊 Joint States:
[INFO] [1766531817.299380762] [pose_test_node]:   📊 shoulder_pan    [XL430] Pos:  1.585rad Load:  4.40% Volt:12.2V Temp: 33.0°C
[INFO] [1766531817.300167729] [pose_test_node]:   📊 shoulder_lift   [XL430] Pos:  2.738rad Load: 20.30% Volt:12.2V Temp: 41.0°C
[INFO] [1766531817.300902418] [pose_test_node]:   📊 elbow_flex      [XL330] Pos:  1.222rad Curr: -441mA Volt: 5.0V Temp: 25.0°C
[INFO] [1766531817.301631514] [pose_test_node]:   📊 wrist_flex      [XL330] Pos:  1.463rad Curr:   37mA Volt: 5.0V Temp: 21.0°C
[INFO] [1766531817.302372888] [pose_test_node]:   📊 wrist_roll      [XL330] Pos:  1.765rad Curr:  -32mA Volt: 5.1V Temp: 20.0°C
[INFO] [1766531817.303134559] [pose_test_node]:   📊 pen_holder      [XL330] Pos:  1.470rad Curr:   25mA Volt: 5.0V Temp: 23.0°C
[INFO] [1766531817.304155675] [pose_test_node]: 
[6/11] pose 6 - place high
[INFO] [1766531817.304888846] [pose_test_node]:   ✓ shoulder_pan: 1.598 rad (within [0.540, 2.044])
[INFO] [1766531817.305535294] [pose_test_node]:   ✓ shoulder_lift: 2.655 rad (within [2.200, 2.886])
[INFO] [1766531817.306159908] [pose_test_node]:   ✓ elbow_flex: 0.806 rad (within [0.726, 2.250])
[INFO] [1766531817.306766819] [pose_test_node]:   ✓ wrist_flex: 1.473 rad (within [0.297, 2.700])
[INFO] [1766531817.307377007] [pose_test_node]:   ✓ wrist_roll: 1.762 rad (within [-1.448, 1.900])
[INFO] [1766531817.307987621] [pose_test_node]:   ✓ pen_holder: 0.655 rad (within [0.190, 1.600])
[INFO] [1766531817.308975256] [pose_test_node]: → Sent: pose 6 - place high
[INFO] [1766531819.313801564] [pose_test_node]:   📊 Joint States:
[INFO] [1766531819.314721531] [pose_test_node]:   📊 shoulder_pan    [XL430] Pos:  1.587rad Load:  4.50% Volt:12.2V Temp: 33.0°C
[INFO] [1766531819.315595758] [pose_test_node]:   📊 shoulder_lift   [XL430] Pos:  2.635rad Load:  7.30% Volt:12.2V Temp: 41.0°C
[INFO] [1766531819.316389948] [pose_test_node]:   📊 elbow_flex      [XL330] Pos:  1.071rad Curr: -476mA Volt: 5.1V Temp: 26.0°C
[INFO] [1766531819.317152414] [pose_test_node]:   📊 wrist_flex      [XL330] Pos:  1.463rad Curr:   37mA Volt: 5.0V Temp: 21.0°C
[INFO] [1766531819.317900622] [pose_test_node]:   📊 wrist_roll      [XL330] Pos:  1.762rad Curr:    0mA Volt: 5.1V Temp: 20.0°C
[INFO] [1766531819.318683311] [pose_test_node]:   📊 pen_holder      [XL330] Pos:  0.734rad Curr:  -33mA Volt: 5.0V Temp: 23.0°C
[INFO] [1766531819.319615057] [pose_test_node]: 
[7/11] pose 7 - reach to grab again
[INFO] [1766531819.320373950] [pose_test_node]:   ✓ shoulder_pan: 1.283 rad (within [0.540, 2.044])
[INFO] [1766531819.321166102] [pose_test_node]:   ✓ shoulder_lift: 2.314 rad (within [2.200, 2.886])
[INFO] [1766531819.321970458] [pose_test_node]:   ✓ elbow_flex: 1.320 rad (within [0.726, 2.250])
[INFO] [1766531819.322765147] [pose_test_node]:   ✓ wrist_flex: 2.327 rad (within [0.297, 2.700])
[INFO] [1766531819.323671874] [pose_test_node]:   ✓ wrist_roll: 1.871 rad (within [-1.448, 1.900])
[INFO] [1766531819.324519064] [pose_test_node]:   ✓ pen_holder: 0.982 rad (within [0.190, 1.600])
[INFO] [1766531819.325593551] [pose_test_node]: → Sent: pose 7 - reach to grab again
[INFO] [1766531821.334916678] [pose_test_node]:   📊 Joint States:
[INFO] [1766531821.335782294] [pose_test_node]:   📊 shoulder_pan    [XL430] Pos:  1.289rad Load: -2.30% Volt:12.2V Temp: 33.0°C
[INFO] [1766531821.336554613] [pose_test_node]:   📊 shoulder_lift   [XL430] Pos:  2.293rad Load:  7.30% Volt:12.2V Temp: 41.0°C
[INFO] [1766531821.337303654] [pose_test_node]:   📊 elbow_flex      [XL330] Pos:  1.398rad Curr:  -75mA Volt: 5.2V Temp: 26.0°C
[INFO] [1766531821.338056121] [pose_test_node]:   📊 wrist_flex      [XL330] Pos:  2.250rad Curr:   94mA Volt: 5.1V Temp: 21.0°C
[INFO] [1766531821.338803106] [pose_test_node]:   📊 wrist_roll      [XL330] Pos:  1.865rad Curr:   29mA Volt: 5.2V Temp: 20.0°C
[INFO] [1766531821.339546851] [pose_test_node]:   📊 pen_holder      [XL330] Pos:  0.942rad Curr:   22mA Volt: 5.1V Temp: 23.0°C
[INFO] [1766531821.340400837] [pose_test_node]: 
[8/11] pose 8 - grab
[INFO] [1766531821.341143674] [pose_test_node]:   ✓ shoulder_pan: 1.329 rad (within [0.540, 2.044])
[INFO] [1766531821.341864067] [pose_test_node]:   ✓ shoulder_lift: 2.314 rad (within [2.200, 2.886])
[INFO] [1766531821.342599459] [pose_test_node]:   ✓ elbow_flex: 1.005 rad (within [0.726, 2.250])
[INFO] [1766531821.343510612] [pose_test_node]:   ✓ wrist_flex: 2.037 rad (within [0.297, 2.700])
[INFO] [1766531821.344303376] [pose_test_node]:   ✓ wrist_roll: 1.766 rad (within [-1.448, 1.900])
[INFO] [1766531821.345041972] [pose_test_node]:   ✓ pen_holder: 1.521 rad (within [0.190, 1.600])
[INFO] [1766531821.346014385] [pose_test_node]: → Sent: pose 8 - grab
[INFO] [1766531823.349588945] [pose_test_node]:   📊 Joint States:
[INFO] [1766531823.350469098] [pose_test_node]:   📊 shoulder_pan    [XL430] Pos:  1.329rad Load:  0.10% Volt:12.2V Temp: 33.0°C
[INFO] [1766531823.351367621] [pose_test_node]:   📊 shoulder_lift   [XL430] Pos:  2.281rad Load: 11.80% Volt:12.2V Temp: 41.0°C
[INFO] [1766531823.352259274] [pose_test_node]:   📊 elbow_flex      [XL330] Pos:  1.211rad Curr: -306mA Volt: 5.1V Temp: 26.0°C
[INFO] [1766531823.353135056] [pose_test_node]:   📊 wrist_flex      [XL330] Pos:  2.030rad Curr:   37mA Volt: 5.1V Temp: 21.0°C
[INFO] [1766531823.353986060] [pose_test_node]:   📊 wrist_roll      [XL330] Pos:  1.766rad Curr:    0mA Volt: 5.1V Temp: 20.0°C
[INFO] [1766531823.354931436] [pose_test_node]:   📊 pen_holder      [XL330] Pos:  1.483rad Curr:   20mA Volt: 5.1V Temp: 23.0°C
[INFO] [1766531823.355819644] [pose_test_node]: 
[9/11] pose 9 - place low
[INFO] [1766531823.356561629] [pose_test_node]:   ✓ shoulder_pan: 1.925 rad (within [0.540, 2.044])
[INFO] [1766531823.357164966] [pose_test_node]:   ✓ shoulder_lift: 2.403 rad (within [2.200, 2.886])
[INFO] [1766531823.357741339] [pose_test_node]:   ✓ elbow_flex: 1.352 rad (within [0.726, 2.250])
[INFO] [1766531823.358326342] [pose_test_node]:   ✓ wrist_flex: 2.327 rad (within [0.297, 2.700])
[INFO] [1766531823.358906623] [pose_test_node]:   ✓ wrist_roll: 1.737 rad (within [-1.448, 1.900])
[INFO] [1766531823.359497978] [pose_test_node]:   ✓ pen_holder: 1.521 rad (within [0.190, 1.600])
[INFO] [1766531823.360451798] [pose_test_node]: → Sent: pose 9 - place low
[INFO] [1766531825.367743952] [pose_test_node]:   📊 Joint States:
[INFO] [1766531825.368681086] [pose_test_node]:   📊 shoulder_pan    [XL430] Pos:  1.909rad Load:  5.00% Volt:12.2V Temp: 33.0°C
[INFO] [1766531825.369571943] [pose_test_node]:   📊 shoulder_lift   [XL430] Pos:  2.344rad Load: 22.00% Volt:12.2V Temp: 41.0°C
[INFO] [1766531825.370376762] [pose_test_node]:   📊 elbow_flex      [XL330] Pos:  1.412rad Curr:  -53mA Volt: 5.2V Temp: 26.0°C
[INFO] [1766531825.371231044] [pose_test_node]:   📊 wrist_flex      [XL330] Pos:  2.259rad Curr:   83mA Volt: 5.2V Temp: 21.0°C
[INFO] [1766531825.372050900] [pose_test_node]:   📊 wrist_roll      [XL330] Pos:  1.742rad Curr:  -29mA Volt: 5.2V Temp: 20.0°C
[INFO] [1766531825.372870571] [pose_test_node]:   📊 pen_holder      [XL330] Pos:  1.483rad Curr:   21mA Volt: 5.1V Temp: 23.0°C
[INFO] [1766531825.373903206] [pose_test_node]: 
[10/11] pose 10 - release
[INFO] [1766531825.374689395] [pose_test_node]:   ✓ shoulder_pan: 1.872 rad (within [0.540, 2.044])
[INFO] [1766531825.375370991] [pose_test_node]:   ✓ shoulder_lift: 2.403 rad (within [2.200, 2.886])
[INFO] [1766531825.376021847] [pose_test_node]:   ✓ elbow_flex: 1.266 rad (within [0.726, 2.250])
[INFO] [1766531825.376634979] [pose_test_node]:   ✓ wrist_flex: 2.327 rad (within [0.297, 2.700])
[INFO] [1766531825.377324983] [pose_test_node]:   ✓ wrist_roll: 1.737 rad (within [-1.448, 1.900])
[INFO] [1766531825.377952709] [pose_test_node]:   ✓ pen_holder: 0.582 rad (within [0.190, 1.600])
[INFO] [1766531825.378966973] [pose_test_node]: → Sent: pose 10 - release
[INFO] [1766531827.383531001] [pose_test_node]:   📊 Joint States:
[INFO] [1766531827.384436969] [pose_test_node]:   📊 shoulder_pan    [XL430] Pos:  1.872rad Load:  0.00% Volt:12.2V Temp: 33.0°C
[INFO] [1766531827.385242399] [pose_test_node]:   📊 shoulder_lift   [XL430] Pos:  2.340rad Load: 23.10% Volt:12.2V Temp: 41.0°C
[INFO] [1766531827.386004292] [pose_test_node]:   📊 elbow_flex      [XL330] Pos:  1.470rad Curr: -303mA Volt: 5.1V Temp: 26.0°C
[INFO] [1766531827.386786296] [pose_test_node]:   📊 wrist_flex      [XL330] Pos:  2.259rad Curr:   83mA Volt: 5.1V Temp: 21.0°C
[INFO] [1766531827.387549485] [pose_test_node]:   📊 wrist_roll      [XL330] Pos:  1.737rad Curr:    0mA Volt: 5.1V Temp: 20.0°C
[INFO] [1766531827.388318823] [pose_test_node]:   📊 pen_holder      [XL330] Pos:  0.654rad Curr:  -33mA Volt: 5.0V Temp: 23.0°C
[INFO] [1766531827.389218661] [pose_test_node]: 
[11/11] pose 11 - poised to work
[INFO] [1766531827.389918498] [pose_test_node]:   ✓ shoulder_pan: 1.558 rad (within [0.540, 2.044])
[INFO] [1766531827.390603316] [pose_test_node]:   ✓ shoulder_lift: 2.879 rad (within [2.200, 2.886])
[INFO] [1766531827.391255505] [pose_test_node]:   ✓ elbow_flex: 2.124 rad (within [0.726, 2.250])
[INFO] [1766531827.391957286] [pose_test_node]:   ✓ wrist_flex: 2.279 rad (within [0.297, 2.700])
[INFO] [1766531827.392724512] [pose_test_node]:   ✓ wrist_roll: 1.182 rad (within [-1.448, 1.900])
[INFO] [1766531827.393415053] [pose_test_node]:   ✓ pen_holder: 1.590 rad (within [0.190, 1.600])
[INFO] [1766531827.394469614] [pose_test_node]: → Sent: pose 11 - poised to work
[INFO] [1766531829.401782398] [pose_test_node]:   📊 Joint States:
[INFO] [1766531829.402845514] [pose_test_node]:   📊 shoulder_pan    [XL430] Pos:  1.562rad Load: -1.70% Volt:12.2V Temp: 33.0°C
[INFO] [1766531829.403743297] [pose_test_node]:   📊 shoulder_lift   [XL430] Pos:  2.835rad Load: 16.40% Volt:12.2V Temp: 41.0°C
[INFO] [1766531829.404509430] [pose_test_node]:   📊 elbow_flex      [XL330] Pos:  2.176rad Curr:  -43mA Volt: 5.2V Temp: 26.0°C
[INFO] [1766531829.405302916] [pose_test_node]:   📊 wrist_flex      [XL330] Pos:  2.258rad Curr:   45mA Volt: 5.2V Temp: 21.0°C
[INFO] [1766531829.406053698] [pose_test_node]:   📊 wrist_roll      [XL330] Pos:  1.192rad Curr:  -32mA Volt: 5.2V Temp: 20.0°C
[INFO] [1766531829.406788609] [pose_test_node]:   📊 pen_holder      [XL330] Pos:  1.545rad Curr:   24mA Volt: 5.2V Temp: 23.0°C
[INFO] [1766531829.407636687] [pose_test_node]: 
============================================================
[INFO] [1766531829.408384340] [pose_test_node]: SUMMARY
[INFO] [1766531829.409170788] [pose_test_node]: ============================================================
[INFO] [1766531829.409901847] [pose_test_node]: Poses tested: 11
[INFO] [1766531829.410742148] [pose_test_node]: Validation errors: 0
[INFO] [1766531829.411484448] [pose_test_node]: Validation warnings: 0
[INFO] [1766531829.412093081] [pose_test_node]: ============================================================
[INFO] [1766531829.412974438] [pose_test_node]: 
===========================================================================
[INFO] [1766531829.413565570] [pose_test_node]: 📊 TELEMETRY SUMMARY
[INFO] [1766531829.414125999] [pose_test_node]: ===========================================================================
[INFO] [1766531829.414683002] [pose_test_node]: XL430 (IDs 1-2): Avg Load: 7.3%
[INFO] [1766531829.415257653] [pose_test_node]: XL330 (IDs 3-6): Total Current: -6mA, Avg: -2mA (-0.1W @ 12V)
[INFO] [1766531829.415881323] [pose_test_node]: Max Temperature: 41.0°C
[INFO] [1766531829.416432826] [pose_test_node]: Voltage: 5.20V - 12.20V (avg 7.53V)
[INFO] [1766531829.417080626] [pose_test_node]: ===========================================================================
```

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
