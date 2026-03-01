# Launching the Physical Robot and its digital twin in Rviz2

Once the software and hardware envionment has been setup with the one-time setup items, then a few additional commands are needed to bring up the physical robot and its digital twin in Rviz2.

## Table of Contents
- [Ensure that the USB port can be accessed](./documentation/launching_physical.md#ensure_that_the_uSB_port_can_be_accessed)
- [Launch the robot via ros2](./documentation/launching_physical.md#launch_the_robot_via_ros2)

  
## Ensure that the USB port can be accessed
The code assumes the port is `/dev/ttyUSB0`. On a Raspberry Pi5 on Ubuntu the device is either `/dev/ttyACM0` or `/dev/ttyACM0`.
If the there is no ttyUSB0 then create a soft link to it.
On ubuntu:

```bash
$ ls -lt /dev/tty* | head -5
crw-rw-rw- 1 root   tty       5,   0 Dec 24 10:44 /dev/tty
crw-rw---- 1 root   dialout 166,   0 Dec 24 10:43 /dev/ttyACM0   
crw--w---- 1 bueche tty       4,   2 Nov 10 14:28 /dev/tty2
crw------- 1 root   root      3, 167 Nov 10 14:27 /dev/ttyz7
rpi5:~$
rpi5:~$ sudo ln -sf /dev/ttyACM0 /dev/ttyUSB0
rpi5:~$
rpi5:~$ ls -lt /dev/tty* | head -5
crw-rw-rw- 1 root   tty       5,   0 Dec 24 10:44 /dev/tty
crw-rw---- 1 root   dialout 166,   0 Dec 24 10:43 /dev/ttyACM0   
lrwxrwxrwx 1 root   root          12 Dec 21 18:35 /dev/ttyUSB0 -> /dev/ttyACM0 # SOFT LINK CREATED
crw--w---- 1 bueche tty       4,   2 Nov 10 14:28 /dev/tty2
crw------- 1 root   root      3, 167 Nov 10 14:27 /dev/ttyz7
```
But it still needs to be readable and writable by other.

On ubuntu:
```bash
rpi5:~$ ls -lt /dev/tty* | head -5
crw-rw-rw- 1 root   tty       5,   0 Dec 24 10:44 /dev/tty
crw-rw---- 1 root   dialout 166,   0 Dec 24 10:43 /dev/ttyACM0   # NOTE: IS NOT rw BY other
lrwxrwxrwx 1 root   root          12 Dec 21 18:35 /dev/ttyUSB0 -> /dev/ttyACM0
crw--w---- 1 bueche tty       4,   2 Nov 10 14:28 /dev/tty2
crw------- 1 root   root      3, 167 Nov 10 14:27 /dev/ttyz7

rpi5:~$ sudo chmod ugo+rw /dev/ttyACM0

rpi5:~$ ls -lt /dev/tty* | head -5
crw-rw-rw- 1 root   tty       5,   0 Dec 24 11:44 /dev/tty
crw-rw-rw- 1 root   dialout 166,   0 Dec 24 10:47 /dev/ttyACM0 # NOTES: NOW rw BY other
lrwxrwxrwx 1 root   root          12 Dec 21 18:35 /dev/ttyUSB0 -> /dev/ttyACM0
crw--w---- 1 bueche tty       4,   2 Nov 10 14:28 /dev/tty2
crw------- 1 root   root      3, 167 Nov 10 14:27 /dev/ttyz7

```

## Launch the robot via ros2
```bash
rpi5:~/$ 
rpi5:~/$ pwd
/home/ubuntu/
rpi5:~/$ cd robot_ws
rpi5:~/robot_ws$ source ./install/setup.bash
rpi5:~/robot_ws$ ros2 launch writing_robot_description koch_v11_real_robot.launch.py
```
This will launch the ros2 robot software. The full example output is shown below. We comment on some of parts below.

```
[INFO] [launch]: All log files can be found below /home/ubuntu/.ros/log/2025-12-24-18-45-33-765368-bueche-rpi5-5351
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [robot_state_publisher-1]: process started with pid [5352]
[INFO] [ros2_control_node-2]: process started with pid [5354]
[INFO] [spawner-3]: process started with pid [5356]
[ros2_control_node-2] [WARN] [1766601934.056054063] [controller_manager]: [Deprecated] Passing the robot description parameter directly to the control_manager node is deprecated. Use '~/robot_description' topic from 'robot_state_publisher' instead.
[ros2_control_node-2] [INFO] [1766601934.056561232] [resource_manager]: Loading hardware 'koch_v11_system' 
[ros2_control_node-2] [INFO] [1766601934.066173128] [resource_manager]: Initialize hardware 'koch_v11_system' 
[ros2_control_node-2] transmission_to_joint_matrix_ 
[ros2_control_node-2] [0][0] 1.000000, [0][1] 0.000000, [0][2] 0.000000, [0][3] 0.000000, [0][4] 0.000000, [0][5] 0.000000, 
[ros2_control_node-2] [1][0] 0.000000, [1][1] 1.000000, [1][2] 0.000000, [1][3] 0.000000, [1][4] 0.000000, [1][5] 0.000000, 
[ros2_control_node-2] [2][0] 0.000000, [2][1] 0.000000, [2][2] 1.000000, [2][3] 0.000000, [2][4] 0.000000, [2][5] 0.000000, 
[ros2_control_node-2] [3][0] 0.000000, [3][1] 0.000000, [3][2] 0.000000, [3][3] 1.000000, [3][4] 0.000000, [3][5] 0.000000, 
[ros2_control_node-2] [4][0] 0.000000, [4][1] 0.000000, [4][2] 0.000000, [4][3] 0.000000, [4][4] 1.000000, [4][5] 0.000000, 
[ros2_control_node-2] [5][0] 0.000000, [5][1] 0.000000, [5][2] 0.000000, [5][3] 0.000000, [5][4] 0.000000, [5][5] 1.000000, 
[ros2_control_node-2] joint_to_transmission_matrix_ 
[ros2_control_node-2] [0][0] 1.000000, [0][1] 0.000000, [0][2] 0.000000, [0][3] 0.000000, [0][4] 0.000000, [0][5] 0.000000, 
[ros2_control_node-2] [1][0] 0.000000, [1][1] 1.000000, [1][2] 0.000000, [1][3] 0.000000, [1][4] 0.000000, [1][5] 0.000000, 
[ros2_control_node-2] [2][0] 0.000000, [2][1] 0.000000, [2][2] 1.000000, [2][3] 0.000000, [2][4] 0.000000, [2][5] 0.000000, 
[ros2_control_node-2] [3][0] 0.000000, [3][1] 0.000000, [3][2] 0.000000, [3][3] 1.000000, [3][4] 0.000000, [3][5] 0.000000, 
[ros2_control_node-2] [4][0] 0.000000, [4][1] 0.000000, [4][2] 0.000000, [4][3] 0.000000, [4][4] 1.000000, [4][5] 0.000000, 
[ros2_control_node-2] [5][0] 0.000000, [5][1] 0.000000, [5][2] 0.000000, [5][3] 0.000000, [5][4] 0.000000, [5][5] 1.000000, 
[ros2_control_node-2] [INFO] [1766601934.067227355] [dynamixel_hardware_interface]: error_timeout_ms parameter not found, using default value of 500ms
[ros2_control_node-2] [INFO] [1766601934.067293152] [dynamixel_hardware_interface]: If there is a torque enabled Dynamixel, the program will be terminated. Set 'disable_torque_at_init' parameter to 'true' to disable torque at initialization.
[ros2_control_node-2] [INFO] [1766601934.067330449] [dynamixel_hardware_interface]: port_name /dev/ttyUSB0 / baudrate 1000000
[ros2_control_node-2] Dynamixel Information File List.
[ros2_control_node-2] num: 30, name: mx_28.model
[ros2_control_node-2] num: 220, name: omy_hat.model
[ros2_control_node-2] num: 230, name: omy_end.model
[ros2_control_node-2] num: 231, name: omy_end_rh_p12_rn.model
[ros2_control_node-2] num: 311, name: mx_64.model
[ros2_control_node-2] num: 321, name: mx_106.model
[ros2_control_node-2] num: 350, name: xl320.model
[ros2_control_node-2] num: 536, name: sensorxel_joy.model
[ros2_control_node-2] num: 537, name: ffw_g40_imu.model
[ros2_control_node-2] num: 600, name: sensorxel_joy.model
[ros2_control_node-2] num: 601, name: ffw_g10_led.model
[ros2_control_node-2] num: 602, name: ffw_g10_rcu.model
[ros2_control_node-2] num: 620, name: ffw_sg2_steer_1.model
[ros2_control_node-2] num: 621, name: ffw_sg2_steer_2.model
[ros2_control_node-2] num: 622, name: ffw_sg2_steer_3.model
[ros2_control_node-2] num: 623, name: ffw_sg2_drive_1.model
[ros2_control_node-2] num: 624, name: ffw_sg2_drive_2.model
[ros2_control_node-2] num: 625, name: ffw_sg2_drive_3.model
[ros2_control_node-2] num: 1000, name: xh430_w350.model
[ros2_control_node-2] num: 1001, name: xd430_t350.model
[ros2_control_node-2] num: 1010, name: xh430_w210.model
[ros2_control_node-2] num: 1011, name: xd430_t210.model
[ros2_control_node-2] num: 1020, name: xm430_w350.model
[ros2_control_node-2] num: 1030, name: xm430_w210.model
[ros2_control_node-2] num: 1040, name: xh430_v350.model
[ros2_control_node-2] num: 1050, name: xh430_v210.model
[ros2_control_node-2] num: 1060, name: xl430_w250.model
[ros2_control_node-2] num: 1070, name: xc430_w150.model
[ros2_control_node-2] num: 1070, name: xc430_w150.model
[ros2_control_node-2] num: 1080, name: xc430_w240.model
[ros2_control_node-2] num: 1080, name: xc430_w240.model
[ros2_control_node-2] num: 1090, name: 2xl430_w250.model
[ros2_control_node-2] num: 1100, name: xh540_w270.model
[ros2_control_node-2] num: 1101, name: xd540_t270.model
[ros2_control_node-2] num: 1110, name: xh540_w150.model
[ros2_control_node-2] num: 1111, name: xd540_t150.model
[ros2_control_node-2] num: 1120, name: xm540_w270.model
[ros2_control_node-2] num: 1130, name: xm540_w150.model
[ros2_control_node-2] num: 1140, name: xh540_v270.model
[ros2_control_node-2] num: 1150, name: xh540_v150.model
[ros2_control_node-2] num: 1160, name: 2xc430_w250.model
[ros2_control_node-2] num: 1170, name: xw540_t260.model
[ros2_control_node-2] num: 1180, name: xw540_t140.model
[ros2_control_node-2] num: 1190, name: xl330_m077.model
[ros2_control_node-2] num: 1200, name: xl330_m288.model
[ros2_control_node-2] num: 1210, name: xc330_t181.model
[ros2_control_node-2] num: 1220, name: xc330_t288.model
[ros2_control_node-2] num: 1230, name: xc330_m181.model
[ros2_control_node-2] num: 1240, name: xc330_m288.model
[ros2_control_node-2] num: 1270, name: xw430_t333.model
[ros2_control_node-2] num: 1310, name: xw540_h260.model
[ros2_control_node-2] num: 2000, name: ph42_020_s300.model
[ros2_control_node-2] num: 2010, name: ph54_100_s500.model
[ros2_control_node-2] num: 2020, name: ph54_200_s500.model
[ros2_control_node-2] num: 2100, name: pm42_010_s260.model
[ros2_control_node-2] num: 2110, name: pm54_040_s250.model
[ros2_control_node-2] num: 2120, name: pm54_060_s250.model
[ros2_control_node-2] num: 4000, name: ym070_210_m001.model
[ros2_control_node-2] num: 4010, name: ym070_210_b001.model
[ros2_control_node-2] num: 4020, name: ym070_210_r051.model
[ros2_control_node-2] num: 4030, name: ym070_210_r099.model
[ros2_control_node-2] num: 4040, name: ym070_210_a051.model
[ros2_control_node-2] num: 4050, name: ym070_210_a099.model
[ros2_control_node-2] num: 4120, name: ym080_230_m001.model
[ros2_control_node-2] num: 4130, name: ym080_230_b001.model
[ros2_control_node-2] num: 4140, name: ym080_230_r051.model
[ros2_control_node-2] num: 4150, name: ym080_230_r099.model
[ros2_control_node-2] num: 4160, name: ym080_230_a051.model
[ros2_control_node-2] num: 4170, name: ym080_230_a099.model
[ros2_control_node-2] num: 35074, name: rh_p12_rn.model
[ros2_control_node-2] num: 43289, name: m42_10_s260_ra.model
[ros2_control_node-2] num: 46097, name: m54_40_s250_ra.model
[ros2_control_node-2] num: 46353, name: m54_60_s250_ra.model
[ros2_control_node-2] num: 51201, name: h42_20_s300_ra.model
[ros2_control_node-2] num: 53769, name: h54_100_s500_ra.model
[ros2_control_node-2] num: 54025, name: h54_200_s500_ra.model
[ros2_control_node-2] [INFO] [1766601934.068302916] [dynamixel_hardware_interface]: $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
[ros2_control_node-2] [INFO] [1766601934.068350027] [dynamixel_hardware_interface]: $$$$$ Init Dxl Comm Port
[ros2_control_node-2] [INFO] [1766601934.068572177] [dynamixel_hardware_interface]: $$$$$ Init Items for type: controller
[ros2_control_node-2] [INFO] [1766601934.068613880] [dynamixel_hardware_interface]: $$$$$ Init Items for type: virtual_dxl
[ros2_control_node-2] Succeeded to open the port!
[ros2_control_node-2] Succeeded to change the [1000000] baudrate!
[ros2_control_node-2] [ID:001] Request ping	 - Ping succeeded. Dynamixel model number : 1060 (xl430_w250.model)
[robot_state_publisher-1] [WARN] [1766601934.088562399] [kdl_parser]: The root link base_link has an inertia specified in the URDF, but KDL does not support a root link with an inertia.  As a workaround, you can add an extra dummy link to your URDF.
[robot_state_publisher-1] [INFO] [1766601934.088692233] [robot_state_publisher]: got segment base_link
[robot_state_publisher-1] [INFO] [1766601934.088791438] [robot_state_publisher]: got segment forearm_link
[robot_state_publisher-1] [INFO] [1766601934.088807790] [robot_state_publisher]: got segment hand_link
[robot_state_publisher-1] [INFO] [1766601934.088821197] [robot_state_publisher]: got segment pen_link
[robot_state_publisher-1] [INFO] [1766601934.088831919] [robot_state_publisher]: got segment shoulder_link
[robot_state_publisher-1] [INFO] [1766601934.088843364] [robot_state_publisher]: got segment upper_arm_link
[robot_state_publisher-1] [INFO] [1766601934.088854716] [robot_state_publisher]: got segment wrist_link
[ros2_control_node-2] [ID:002] Request ping	 - Ping succeeded. Dynamixel model number : 1060 (xl430_w250.model)
[ros2_control_node-2] [ID:003] Request ping	 - Ping succeeded. Dynamixel model number : 1200 (xl330_m288.model)
[ros2_control_node-2] [ID:004] Request ping	 - Ping succeeded. Dynamixel model number : 1200 (xl330_m288.model)
[ros2_control_node-2] [ID:005] Request ping	 - Ping succeeded. Dynamixel model number : 1200 (xl330_m288.model)
[ros2_control_node-2] [ID:006] Request ping	 - Ping succeeded. Dynamixel model number : 1190 (xl330_m077.model)
[ros2_control_node-2] [INFO] [1766601934.116113102] [dynamixel_hardware_interface]: Trying to connect to the communication port...
[ros2_control_node-2] [InitTorqueStates][ID:001] Current torque state: OFF
[ros2_control_node-2] [InitTorqueStates][ID:002] Current torque state: OFF
[ros2_control_node-2] [InitTorqueStates][ID:003] Current torque state: OFF
[ros2_control_node-2] [InitTorqueStates][ID:004] Current torque state: OFF
[ros2_control_node-2] [InitTorqueStates][ID:005] Current torque state: OFF
[ros2_control_node-2] [InitTorqueStates][ID:006] Current torque state: OFF
[ros2_control_node-2] [INFO] [1766601934.129620961] [dynamixel_hardware_interface]: $$$$$ Init Items for type: dxl
[ros2_control_node-2] [INFO] [1766601934.129738888] [dynamixel_hardware_interface]: $$$$$ Init Items for type: sensor
[ros2_control_node-2] [INFO] [1766601934.129772221] [dynamixel_hardware_interface]: $$$$$ Init Dxl Read Items
[ros2_control_node-2] Dynamixel Read Type : bulk read
[ros2_control_node-2] ID : 1	Read items : 	Present Position	Present Velocity	Present Load	Present Temperature	Present Input Voltage
[ros2_control_node-2] ID : 2	Read items : 	Present Position	Present Velocity	Present Load	Present Temperature	Present Input Voltage
[ros2_control_node-2] ID : 3	Read items : 	Present Position	Present Velocity	Present Current	Present Temperature	Present Input Voltage
[ros2_control_node-2] ID : 4	Read items : 	Present Position	Present Velocity	Present Current	Present Temperature	Present Input Voltage
[ros2_control_node-2] ID : 5	Read items : 	Present Position	Present Velocity	Present Current	Present Temperature	Present Input Voltage
[ros2_control_node-2] ID : 6	Read items : 	Present Position	Present Velocity	Present Current	Present Temperature	Present Input Voltage
[ros2_control_node-2] [ID:001] Add Indirect Address Read Item : [Present Position]
[ros2_control_node-2] [ID:001] Add Indirect Address Read Item : [Present Velocity]
[ros2_control_node-2] [ID:001] Add Indirect Address Read Item : [Present Load]
[ros2_control_node-2] [ID:001] Add Indirect Address Read Item : [Present Temperature]
[ros2_control_node-2] [ID:001] Add Indirect Address Read Item : [Present Input Voltage]
[ros2_control_node-2] [ID:002] Add Indirect Address Read Item : [Present Position]
[ros2_control_node-2] [ID:002] Add Indirect Address Read Item : [Present Velocity]
[ros2_control_node-2] [ID:002] Add Indirect Address Read Item : [Present Load]
[ros2_control_node-2] [ID:002] Add Indirect Address Read Item : [Present Temperature]
[ros2_control_node-2] [ID:002] Add Indirect Address Read Item : [Present Input Voltage]
[ros2_control_node-2] [ID:003] Add Indirect Address Read Item : [Present Position]
[ros2_control_node-2] [ID:003] Add Indirect Address Read Item : [Present Velocity]
[ros2_control_node-2] [ID:003] Add Indirect Address Read Item : [Present Current]
[ros2_control_node-2] [ID:003] Add Indirect Address Read Item : [Present Temperature]
[ros2_control_node-2] [ID:003] Add Indirect Address Read Item : [Present Input Voltage]
[ros2_control_node-2] [ID:004] Add Indirect Address Read Item : [Present Position]
[ros2_control_node-2] [ID:004] Add Indirect Address Read Item : [Present Velocity]
[ros2_control_node-2] [ID:004] Add Indirect Address Read Item : [Present Current]
[ros2_control_node-2] [ID:004] Add Indirect Address Read Item : [Present Temperature]
[ros2_control_node-2] [ID:004] Add Indirect Address Read Item : [Present Input Voltage]
[ros2_control_node-2] [ID:005] Add Indirect Address Read Item : [Present Position]
[ros2_control_node-2] [ID:005] Add Indirect Address Read Item : [Present Velocity]
[ros2_control_node-2] [ID:005] Add Indirect Address Read Item : [Present Current]
[ros2_control_node-2] [ID:005] Add Indirect Address Read Item : [Present Temperature]
[ros2_control_node-2] [ID:005] Add Indirect Address Read Item : [Present Input Voltage]
[ros2_control_node-2] [ID:006] Add Indirect Address Read Item : [Present Position]
[ros2_control_node-2] [ID:006] Add Indirect Address Read Item : [Present Velocity]
[ros2_control_node-2] [ID:006] Add Indirect Address Read Item : [Present Current]
[ros2_control_node-2] [ID:006] Add Indirect Address Read Item : [Present Temperature]
[ros2_control_node-2] [ID:006] Add Indirect Address Read Item : [Present Input Voltage]
[ros2_control_node-2] set fast bulk read (indirect addr) : addr 634, size 13
[ros2_control_node-2] set fast bulk read (indirect addr) : addr 634, size 13
[ros2_control_node-2] set fast bulk read (indirect addr) : addr 230, size 13
[ros2_control_node-2] set fast bulk read (indirect addr) : addr 230, size 13
[ros2_control_node-2] set fast bulk read (indirect addr) : addr 230, size 13
[ros2_control_node-2] set fast bulk read (indirect addr) : addr 230, size 13
[ros2_control_node-2] [ID:001] Add BulkRead item : [Indirect Item Data][634][13]
[ros2_control_node-2] [ID:002] Add BulkRead item : [Indirect Item Data][634][13]
[ros2_control_node-2] [ID:003] Add BulkRead item : [Indirect Item Data][230][13]
[ros2_control_node-2] [ID:004] Add BulkRead item : [Indirect Item Data][230][13]
[ros2_control_node-2] [ID:005] Add BulkRead item : [Indirect Item Data][230][13]
[ros2_control_node-2] [ID:006] Add BulkRead item : [Indirect Item Data][230][13]
[ros2_control_node-2] FastBulkRead handler set up successfully.
[ros2_control_node-2] Success to set BulkRead handler using indirect address
[ros2_control_node-2] [INFO] [1766601934.272907834] [dynamixel_hardware_interface]: $$$$$ Init Dxl Write Items
[ros2_control_node-2] Dynamixel Write Type : sync write
[ros2_control_node-2] ID : 1, 2, 3, 4, 5, 6, 
[ros2_control_node-2] Write items : 	Goal Position
[ros2_control_node-2] set sync write (indirect addr) : addr 224, size 4
[ros2_control_node-2] Success to set SyncWrite handler using indirect address
[ros2_control_node-2] [INFO] [1766601934.321550802] [resource_manager]: Successful initialization of hardware 'koch_v11_system'
[ros2_control_node-2] [INFO] [1766601934.322016859] [resource_manager]: 'configure' hardware 'koch_v11_system' 
[ros2_control_node-2] [INFO] [1766601934.322043323] [resource_manager]: Successful 'configure' of hardware 'koch_v11_system'
[ros2_control_node-2] [INFO] [1766601934.322062434] [resource_manager]: 'activate' hardware 'koch_v11_system' 
[ros2_control_node-2] [INFO] [1766601934.327125013] [dynamixel_hardware_interface]: Sync joint state to command (joint: shoulder_pan, position, 1.56082 <- position, 1.56082
[ros2_control_node-2] [INFO] [1766601934.327199828] [dynamixel_hardware_interface]: Sync joint state to command (joint: shoulder_lift, position, 2.70266 <- position, 2.70266
[ros2_control_node-2] [INFO] [1766601934.327222884] [dynamixel_hardware_interface]: Sync joint state to command (joint: elbow_flex, position, 2.00896 <- position, 2.00896
[ros2_control_node-2] [INFO] [1766601934.327244828] [dynamixel_hardware_interface]: Sync joint state to command (joint: wrist_flex, position, 2.40492 <- position, 2.40492
[ros2_control_node-2] [INFO] [1766601934.327265532] [dynamixel_hardware_interface]: Sync joint state to command (joint: wrist_roll, position, 0.408238 <- position, 0.408238
[ros2_control_node-2] [INFO] [1766601934.327287106] [dynamixel_hardware_interface]: Sync joint state to command (joint: pen_holder, position, 1.06203 <- position, 1.06203
[ros2_control_node-2] [INFO] [1766601934.327448774] [dynamixel_hardware_interface]: Enabling torque for Dynamixels
[ros2_control_node-2] [ID:001] Torque ON
[ros2_control_node-2] [ID:002] Torque ON
[ros2_control_node-2] [ID:003] Torque ON
[ros2_control_node-2] [ID:004] Torque ON
[ros2_control_node-2] [ID:005] Torque ON
[ros2_control_node-2] [ID:006] Torque ON
[ros2_control_node-2] [INFO] [1766601934.340637595] [dynamixel_hardware_interface]: Dynamixel Hardware Start!
[ros2_control_node-2] [INFO] [1766601934.340722854] [resource_manager]: Successful 'activate' of hardware 'koch_v11_system'
[ros2_control_node-2] [INFO] [1766601934.349695915] [controller_manager]: update rate is 100 Hz
[ros2_control_node-2] [INFO] [1766601934.349752656] [controller_manager]: Spawning controller_manager RT thread with scheduler priority: 50
[ros2_control_node-2] [WARN] [1766601934.350023416] [controller_manager]: No real-time kernel detected on this system. See [https://control.ros.org/master/doc/ros2_control/controller_manager/doc/userdoc.html] for details on how to enable realtime scheduling.
[ros2_control_node-2] FastBulkRead Rx Fail [Dxl Size : 6] [Error code : -3001]
[ros2_control_node-2] [ERROR] [1766601934.350200991] [dynamixel_hardware_interface]: Communication Fail --> BULK_READ_FAIL
[ros2_control_node-2] [ERROR] [1766601934.350262769] [dynamixel_hardware_interface]: Dynamixel Read Fail (Duration: 0.001815ms/500ms)
[ros2_control_node-2] [ERROR] [1766601934.350302158] [dynamixel_hardware_interface]: Dynamixel Write Fail (Duration: 0.001815ms/500ms)
[ros2_control_node-2] [INFO] [1766601934.468844823] [controller_manager]: Loading controller 'joint_state_broadcaster'
[spawner-3] [INFO] [1766601934.508757083] [spawner_joint_state_broadcaster]: Loaded joint_state_broadcaster
[ros2_control_node-2] [INFO] [1766601934.510310053] [controller_manager]: Configuring controller 'joint_state_broadcaster'
[ros2_control_node-2] [INFO] [1766601934.510467369] [joint_state_broadcaster]: 'joints' or 'interfaces' parameter is empty. All available state interfaces will be published
[spawner-3] [INFO] [1766601934.546571500] [spawner_joint_state_broadcaster]: Configured and activated joint_state_broadcaster
[INFO] [spawner-3]: process has finished cleanly [pid 5356]
[INFO] [spawner-4]: process started with pid [5392]
[ros2_control_node-2] [INFO] [1766601935.197657224] [controller_manager]: Loading controller 'koch_v11_controller'
[ros2_control_node-2] [WARN] [1766601935.212151625] [koch_v11_controller]: [Deprecated]: "allow_nonzero_velocity_at_trajectory_end" is set to true. The default behavior will change to false.
[spawner-4] [INFO] [1766601935.239325419] [spawner_koch_v11_controller]: Loaded koch_v11_controller
[ros2_control_node-2] [INFO] [1766601935.240988260] [controller_manager]: Configuring controller 'koch_v11_controller'
[ros2_control_node-2] [INFO] [1766601935.241231168] [koch_v11_controller]: No specific joint names are used for command interfaces. Using 'joints' parameter.
[ros2_control_node-2] [INFO] [1766601935.241275076] [koch_v11_controller]: Command interfaces are [position] and state interfaces are [position velocity].
[ros2_control_node-2] [INFO] [1766601935.241308872] [koch_v11_controller]: Using 'splines' interpolation method.
[ros2_control_node-2] [INFO] [1766601935.242696286] [koch_v11_controller]: Controller state will be published at 50.00 Hz.
[ros2_control_node-2] [INFO] [1766601935.245407762] [koch_v11_controller]: Action status changes will be monitored at 20.00 Hz.
[spawner-4] [INFO] [1766601935.275855015] [spawner_koch_v11_controller]: Configured and activated koch_v11_controller
```
