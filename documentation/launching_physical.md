# Launching the Physical Robot 

Once the software and hardware envionment has been setup with the one-time setup items, then a few additional commands are needed to bring up the physical robot and its digital twin in Rviz2.

## Table of Contents
- [Ensure that the USB port can be accessed](#ensure-that-the-usb-port-can-be-accessed)
- [Launching the basic robot](#launch-the-robot-via-ros2)
- [Launching the robot for balancing activity](#launcing-the-robot-for-balancing-activity)

  
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
$ ros2 launch writing_robot_description koch_v11_real_robot.launch.py
[INFO] [launch]: All log files can be found below /home/ubuntu/.ros/log/2026-03-01-23-27-25-361593-bueche-rpi5-6584
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [robot_state_publisher-1]: process started with pid [6587]
[INFO] [ros2_control_node-2]: process started with pid [6588]
[ros2_control_node-2] [INFO] [1772407645.679930506] [controller_manager]: Using Steady (Monotonic) clock for triggering controller manager cycles.
[ros2_control_node-2] [INFO] [1772407645.686064407] [controller_manager]: Subscribing to '/robot_description' topic for robot description.
[ros2_control_node-2] [INFO] [1772407645.690637597] [controller_manager]: update rate is 100 Hz
[ros2_control_node-2] [INFO] [1772407645.690681912] [controller_manager]: Overruns handling is : enabled
[ros2_control_node-2] [INFO] [1772407645.690695190] [controller_manager]: Spawning controller_manager RT thread with scheduler priority: 50
[ros2_control_node-2] [WARN] [1772407645.690820061] [controller_manager]: Could not enable FIFO RT scheduling policy: with error number <1>(Operation not permitted). See [https://control.ros.org/master/doc/ros2_control/controller_manager/doc/userdoc.html] for details on how to enable realtime scheduling.
[robot_state_publisher-1] [WARN] [1772407645.709807044] [kdl_parser]: The root link base_link has an inertia specified in the URDF, but KDL does not support a root link with an inertia.  As a workaround, you can add an extra dummy link to your URDF.
[robot_state_publisher-1] [INFO] [1772407645.710002990] [robot_state_publisher]: Robot initialized
[ros2_control_node-2] [INFO] [1772407645.781235642] [controller_manager]: Received robot description from topic.
[ros2_control_node-2] [INFO] [1772407645.787500822] [controller_manager]: Loading hardware 'koch_v11_system' 
[ros2_control_node-2] [INFO] [1772407645.802065987] [controller_manager]: Loaded hardware 'koch_v11_system' from plugin 'dynamixel_hardware_interface/DynamixelHardware'
[ros2_control_node-2] [INFO] [1772407645.803658458] [controller_manager]: Initialize hardware 'koch_v11_system' 
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
[ros2_control_node-2] [INFO] [1772407645.818822442] [dynamixel_hardware_interface]: error_timeout_ms parameter not found, using default value of 500ms
[ros2_control_node-2] [INFO] [1772407645.818887646] [dynamixel_hardware_interface]: If there is a torque enabled Dynamixel, the program will be terminated. Set 'disable_torque_at_init' parameter to 'true' to disable torque at initialization.
[ros2_control_node-2] [INFO] [1772407645.818910035] [dynamixel_hardware_interface]: port_name /dev/ttyUSB0 / baudrate 1000000
[ros2_control_node-2] Dynamixel Information File List.
[ros2_control_node-2] num: 30, name: mx_28.model
[ros2_control_node-2] num: 220, name: omy_hat.model
[ros2_control_node-2] num: 230, name: omy_end.model
[ros2_control_node-2] num: 231, name: omy_end_rh_p12_rn.model
[ros2_control_node-2] num: 260, name: hx5_d20_rr.model
[ros2_control_node-2] num: 261, name: hx5_d20_rl.model
[ros2_control_node-2] num: 6001, name: hx5_d20_r_synctable_1_device_1.model
[ros2_control_node-2] num: 6002, name: hx5_d20_r_synctable_1_device_2.model
[ros2_control_node-2] num: 6003, name: hx5_d20_r_synctable_1_device_3.model
[ros2_control_node-2] num: 6004, name: hx5_d20_r_synctable_1_device_4.model
[ros2_control_node-2] num: 6005, name: hx5_d20_r_synctable_1_device_5.model
[ros2_control_node-2] num: 6006, name: hx5_d20_r_synctable_2_device_1.model
[ros2_control_node-2] num: 6007, name: hx5_d20_r_synctable_2_device_2.model
[ros2_control_node-2] num: 6008, name: hx5_d20_r_synctable_2_device_3.model
[ros2_control_node-2] num: 6009, name: hx5_d20_r_synctable_2_device_4.model
[ros2_control_node-2] num: 6010, name: hx5_d20_r_synctable_2_device_5.model
[ros2_control_node-2] num: 6011, name: hx5_d20_r_synctable_3_device_1.model
[ros2_control_node-2] num: 6012, name: hx5_d20_r_synctable_3_device_2.model
[ros2_control_node-2] num: 6013, name: hx5_d20_r_synctable_3_device_3.model
[ros2_control_node-2] num: 6014, name: hx5_d20_r_synctable_3_device_4.model
[ros2_control_node-2] num: 6015, name: hx5_d20_r_synctable_3_device_5.model
[ros2_control_node-2] num: 6016, name: hx5_d20_r_synctable_4_device_1.model
[ros2_control_node-2] num: 6017, name: hx5_d20_r_synctable_4_device_2.model
[ros2_control_node-2] num: 6018, name: hx5_d20_r_synctable_4_device_3.model
[ros2_control_node-2] num: 6019, name: hx5_d20_r_synctable_4_device_4.model
[ros2_control_node-2] num: 6020, name: hx5_d20_r_synctable_4_device_5.model
[ros2_control_node-2] num: 6021, name: hx5_d20_r_synctable_5_device_1.model
[ros2_control_node-2] num: 6022, name: hx5_d20_r_synctable_5_device_2.model
[ros2_control_node-2] num: 6023, name: hx5_d20_r_synctable_5_device_3.model
[ros2_control_node-2] num: 6024, name: hx5_d20_r_synctable_5_device_4.model
[ros2_control_node-2] num: 6025, name: hx5_d20_r_synctable_5_device_5.model
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
[ros2_control_node-2] num: 1700, name: xc335_t332.model
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
[ros2_control_node-2] [INFO] [1772407645.819989522] [dynamixel_hardware_interface]: $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
[ros2_control_node-2] [INFO] [1772407645.820040651] [dynamixel_hardware_interface]: $$$$$ Init Dxl Comm Port
[ros2_control_node-2] Succeeded to open the port [/dev/ttyUSB0]! (attempt 1/5)
[ros2_control_node-2] Succeeded to change the baudrate [1000000]!
[ros2_control_node-2] [comm_id:001][ID:001] Request ping	 - Ping succeeded. Dynamixel model number : 1060 (xl430_w250.model)
[ros2_control_node-2] [InitTorqueStates][comm_id:001][ID:001] Current torque state: OFF
[ros2_control_node-2] [comm_id:002][ID:002] Request ping	 - Ping succeeded. Dynamixel model number : 1060 (xl430_w250.model)
[ros2_control_node-2] [InitTorqueStates][comm_id:002][ID:002] Current torque state: OFF
[ros2_control_node-2] [comm_id:003][ID:003] Request ping	 - Ping succeeded. Dynamixel model number : 1200 (xl330_m288.model)
[ros2_control_node-2] [InitTorqueStates][comm_id:003][ID:003] Current torque state: OFF
[ros2_control_node-2] [comm_id:004][ID:004] Request ping	 - Ping succeeded. Dynamixel model number : 1200 (xl330_m288.model)
[ros2_control_node-2] [InitTorqueStates][comm_id:004][ID:004] Current torque state: OFF
[ros2_control_node-2] [comm_id:005][ID:005] Request ping	 - Ping succeeded. Dynamixel model number : 1200 (xl330_m288.model)
[ros2_control_node-2] [InitTorqueStates][comm_id:005][ID:005] Current torque state: OFF
[ros2_control_node-2] [comm_id:006][ID:006] Request ping	 - Ping succeeded. Dynamixel model number : 1190 (xl330_m077.model)
[ros2_control_node-2] [InitTorqueStates][comm_id:006][ID:006] Current torque state: OFF
[ros2_control_node-2] [INFO] [1772407645.889421591] [dynamixel_hardware_interface]: $$$$$ Init Dxl Read Items
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
[ros2_control_node-2] [INFO] [1772407646.032963679] [dynamixel_hardware_interface]: $$$$$ Init Dxl Write Items
[ros2_control_node-2] Dynamixel Write Type : sync write
[ros2_control_node-2] ID : 1, 2, 3, 4, 5, 6, 
[ros2_control_node-2] Write items : 	Goal Position
[ros2_control_node-2] set sync write (indirect addr) : addr 224, size 4
[ros2_control_node-2] Success to set SyncWrite handler using indirect address
[ros2_control_node-2] [INFO] [1772407646.088268549] [controller_manager]: Successful initialization of hardware 'koch_v11_system'
[ros2_control_node-2] [INFO] [1772407646.089759389] [resource_manager]: 'configure' hardware 'koch_v11_system' 
[ros2_control_node-2] [INFO] [1772407646.089791723] [resource_manager]: Successful 'configure' of hardware 'koch_v11_system'
[ros2_control_node-2] [INFO] [1772407646.089814186] [resource_manager]: 'activate' hardware 'koch_v11_system' 
[ros2_control_node-2] [INFO] [1772407646.094021670] [dynamixel_hardware_interface]: Sync joint state to command (joint: shoulder_pan, position, 1.56313 <- position, 1.56313
[ros2_control_node-2] [INFO] [1772407646.094076893] [dynamixel_hardware_interface]: Sync joint state to command (joint: shoulder_lift, position, 2.71515 <- position, 2.71515
[ros2_control_node-2] [INFO] [1772407646.094095541] [dynamixel_hardware_interface]: Sync joint state to command (joint: elbow_flex, position, 1.77482 <- position, 1.77482
[ros2_control_node-2] [INFO] [1772407646.094112671] [dynamixel_hardware_interface]: Sync joint state to command (joint: wrist_flex, position, 0.895845 <- position, 0.895845
[ros2_control_node-2] [INFO] [1772407646.094128745] [dynamixel_hardware_interface]: Sync joint state to command (joint: wrist_roll, position, 1.56159 <- position, 1.56159
[ros2_control_node-2] [INFO] [1772407646.094144930] [dynamixel_hardware_interface]: Sync joint state to command (joint: pen_holder, position, 1.62142 <- position, 1.62142
[ros2_control_node-2] [INFO] [1772407646.094361209] [dynamixel_hardware_interface]: Enabling torque for Dynamixels
[ros2_control_node-2] [comm_id:001][ID:001] Torque ON
[ros2_control_node-2] [comm_id:002][ID:002] Torque ON
[ros2_control_node-2] [comm_id:003][ID:003] Torque ON
[ros2_control_node-2] [comm_id:004][ID:004] Torque ON
[ros2_control_node-2] [comm_id:005][ID:005] Torque ON
[ros2_control_node-2] [comm_id:006][ID:006] Torque ON
[ros2_control_node-2] [INFO] [1772407646.108393835] [dynamixel_hardware_interface]: Dynamixel Hardware Start!
[ros2_control_node-2] [INFO] [1772407646.108474039] [resource_manager]: Successful 'activate' of hardware 'koch_v11_system'
[ros2_control_node-2] [INFO] [1772407646.108612113] [controller_manager]: Registering statistics for : koch_v11_system
[ros2_control_node-2] [INFO] [1772407646.108785466] [controller_manager]: Resource Manager has been successfully initialized. Starting Controller Manager services...
[INFO] [spawner-3]: process started with pid [6624]
[spawner-3] [INFO] [1772407648.214204665] [spawner_joint_state_broadcaster]: waiting for service /controller_manager/list_controllers to become available...
[ros2_control_node-2] [INFO] [1772407648.222263297] [controller_manager]: Loading controller : 'joint_state_broadcaster' of type 'joint_state_broadcaster/JointStateBroadcaster'
[ros2_control_node-2] [INFO] [1772407648.222330113] [controller_manager]: Loading controller 'joint_state_broadcaster'
[ros2_control_node-2] [INFO] [1772407648.238686880] [controller_manager]: Controller 'joint_state_broadcaster' node arguments: --ros-args --params-file /tmp/launch_params_yroe8zdt 
[spawner-3] [INFO] [1772407648.267153504] [spawner_joint_state_broadcaster]: Loaded joint_state_broadcaster
[ros2_control_node-2] [INFO] [1772407648.269032328] [controller_manager]: Configuring controller: 'joint_state_broadcaster'
[ros2_control_node-2] [INFO] [1772407648.269240977] [joint_state_broadcaster]: 'joints' or 'interfaces' parameter is empty. All available state interfaces will be published
[ros2_control_node-2] [INFO] [1772407648.287854144] [controller_manager]: Activating controllers: [ joint_state_broadcaster ]
[ros2_control_node-2] [INFO] [1772407648.295436664] [controller_manager]: Successfully switched controllers!
[spawner-3] [INFO] [1772407648.307639576] [spawner_joint_state_broadcaster]: Configured and activated joint_state_broadcaster
[INFO] [spawner-3]: process has finished cleanly [pid 6624]
[INFO] [spawner-4]: process started with pid [6641]
[spawner-4] [INFO] [1772407650.225252761] [spawner_koch_v11_controller]: waiting for service /controller_manager/list_controllers to become available...
[spawner-4] [INFO] [1772407650.492474802] [spawner_koch_v11_controller]: Setting controller param "params_file" to "['/home/ubuntu/robot_ws/install/writing_robot_description/share/writing_robot_description/config/koch_v11_controllers_real.yaml']" for koch_v11_controller
[ros2_control_node-2] [INFO] [1772407650.502698982] [controller_manager]: Loading controller : 'koch_v11_controller' of type 'joint_trajectory_controller/JointTrajectoryController'
[ros2_control_node-2] [INFO] [1772407650.502751779] [controller_manager]: Loading controller 'koch_v11_controller'
[ros2_control_node-2] [INFO] [1772407650.513313313] [controller_manager]: Controller 'koch_v11_controller' node arguments: --ros-args --params-file /tmp/launch_params_yroe8zdt --params-file /home/ubuntu/robot_ws/install/writing_robot_description/share/writing_robot_description/config/koch_v11_controllers_real.yaml 
[spawner-4] [INFO] [1772407650.547436799] [spawner_koch_v11_controller]: Loaded koch_v11_controller
[ros2_control_node-2] [INFO] [1772407650.549255364] [controller_manager]: Configuring controller: 'koch_v11_controller'
[ros2_control_node-2] [INFO] [1772407650.549608347] [koch_v11_controller]: No specific joint names are used for command interfaces. Using 'joints' parameter.
[ros2_control_node-2] [INFO] [1772407650.549662384] [koch_v11_controller]: Command interfaces are [position] and state interfaces are [position velocity].
[ros2_control_node-2] [INFO] [1772407650.549714199] [koch_v11_controller]: Using 'splines' interpolation method.
[ros2_control_node-2] [INFO] [1772407650.553734645] [koch_v11_controller]: Action status changes will be monitored at 20.00 Hz.
[ros2_control_node-2] [INFO] [1772407650.567711919] [controller_manager]: Activating controllers: [ koch_v11_controller ]
[ros2_control_node-2] [INFO] [1772407650.575641718] [controller_manager]: Successfully switched controllers!
[spawner-4] [INFO] [1772407650.587938187] [spawner_koch_v11_controller]: Configured and activated koch_v11_controller
[INFO] [spawner-4]: process has finished cleanly [pid 6641]

```
## Launcing the robot for balancing activity
In this section we cover how to launch all of the ROS2 nodes for the balancing activity. This currently involves a number of terminal windows and we will at some point simplify it.

First, in the current setup some of the nodes run on the RPI5 and some on the Nvidia Orin Nano. 

### Step 1: Start up ball detector node on Nvidia Nano
This particular node and its parameters has been documented[here](./ball_detector.md). This is useful to start first as camera placement and lighting can greatly impact the detection process. An example invocation is shown below.
```
ubuntu@bueche-nvidia-nano:~/robot_ws/src$ ros2 run balance ball_detector_nvidia   --ros-args   -p engine_path:=/home/ubuntu/robot_ws/src/balance/models/ball_detector_v4.engine   -p input_size:=640   -p conf_threshold:=0.30   -p rgb_fps:=35
[INFO] [1777504126.790845165] [ball_detector_nvidia]: Loading TensorRT engine: /home/ubuntu/robot_ws/src/balance/models/ball_detector_v4.engine
[INFO] [1777504126.794908312] [ball_detector_nvidia]: TensorRT engine loaded — running warmup inferences
Loading /home/ubuntu/robot_ws/src/balance/models/ball_detector_v4.engine for TensorRT inference...
[04/29/2026-23:08:46] [TRT] [I] Loaded engine size: 8 MiB
[04/29/2026-23:08:47] [TRT] [I] [MemUsageChange] TensorRT-managed allocation in engine deserialization: CPU +0, GPU +5, now: CPU 0, GPU 5 (MiB)
[04/29/2026-23:08:47] [TRT] [I] [MemUsageChange] TensorRT-managed allocation in IExecutionContext creation: CPU +0, GPU +9, now: CPU 0, GPU 14 (MiB)
[INFO] [1777504127.372610392] [ball_detector_nvidia]: TRT warmup 1/3: 576.2ms
[INFO] [1777504127.384929564] [ball_detector_nvidia]: TRT warmup 2/3: 10.9ms
[INFO] [1777504127.396973207] [ball_detector_nvidia]: TRT warmup 3/3: 10.8ms
[19443010E156077E00] [1.2.2] [1.067] [ColorCamera(0)] [warning] Unsupported resolution set for detected camera IMX378/214, needs 1080_P / 4_K / 12_MP. Defaulting to 1080_P
[INFO] [1777504129.988719848] [ball_detector_nvidia]: OAK-D connected: OAK-D-LITE  USB HIGH  (camera-only mode, MJPEG q=85)
[INFO] [1777504129.990624073] [ball_detector_nvidia]: ball_detector_nvidia ready  engine=ball_detector_v4.engine  input=640px  conf=0.3  iou=0.45  rgb_fps=35  mjpeg_q=85  debug_fps=5.0
[INFO] [1777504130.121832326] [ball_detector_nvidia]: Cup  bbox: 223x233px  conf=0.897
[INFO] [1777504130.123852747] [ball_detector_nvidia]: Ball bbox: 34x53px  conf=0.532
[INFO] [1777504131.135371740] [ball_detector_nvidia]: Cup  bbox: 223x229px  conf=0.903
[INFO] [1777504131.136420192] [ball_detector_nvidia]: Ball bbox: 32x50px  conf=0.736

```

### Step 2: Start rviz2 on the Nvidia Nano
Even if the initial camera works well upon boot (high confidence recognition) its no guarantee that the one of the tipping poses will cause the ball bearing to fall out of the camera view. Rviz2 and the debug picture topic sent by the ball detector node are useful to tune camera placement and optimal lighting. Rviz2 really can't be run on the RPI5 due to capacity limitations.

```
ubuntu@bueche-nvidia-nano:~/robot_ws/src$ ros2 run rviz2 rviz2

```

### Step 3: Start ball marker node on Nvidia Nano

```
ubuntu@bueche-nvidia-nano:~/robot_ws$ ros2 run balance ball_marker_node
[INFO] [1783545409.262960482] [ball_marker_node]: ball_marker_node ready
```

### Step 4: Launch the robot
Very similar to the non-balance one, but there is a special launch file (`koch_v11_real_robot_for_balance.launch.py`) to capture the URDF specific to this balance sub-project: this URDF adds a cup and ball.
```
ubuntu@bueche-rpi5:~/robot_ws$ ros2 launch writing_robot_description koch_v11_real_robot_for_balance.launch.py
[INFO] [launch]: All log files can be found below /home/ubuntu/.ros/log/2026-07-08-21-38-31-032445-bueche-rpi5-608
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [servo_config_node-1]: process started with pid [611]
[INFO] [robot_state_publisher-2]: process started with pid [612]
[robot_state_publisher-2] [WARN] [1783546711.317092766] [kdl_parser]: The root link base_link has an inertia specified in the URDF, but KDL does not support a root link with an inertia.  As a workaround, you can add an extra dummy link to your URDF.
[robot_state_publisher-2] [INFO] [1783546711.317259934] [robot_state_publisher]: Robot initialized
[servo_config_node-1] === servo_config_node: configuring Dynamixel servos ===
[servo_config_node-1] Opened /dev/ttyOpenRB
[servo_config_node-1] Baud rate set to 1000000
[servo_config_node-1] 
[servo_config_node-1]   Configuring ID 2  shoulder_lift  [XL430]
[servo_config_node-1]   ok mode=position P=640 I=0 D=9000 FF1=0 FF2=0 vel=0 acc=0 goal_curr=0
[servo_config_node-1] 
[servo_config_node-1]   Configuring ID 3  elbow_flex     [XL330]
[servo_config_node-1]   ok mode=current-pos P=2000 I=0 D=0 FF1=0 FF2=0 vel=200 acc=50 goal_curr=600
[servo_config_node-1] 
[servo_config_node-1]   Configuring ID 4  wrist_flex     [XL330]
[servo_config_node-1]   ok mode=position P=2000 I=0 D=0 FF1=0 FF2=0 vel=200 acc=50 goal_curr=0
[servo_config_node-1] 
[servo_config_node-1]   Configuring ID 5  wrist_roll     [XL330]
[servo_config_node-1]   ok mode=position P=2000 I=0 D=0 FF1=0 FF2=0 vel=200 acc=50 goal_curr=0
[servo_config_node-1] 
[servo_config_node-1]   Configuring ID 6  pen_holder     [XL330]
[servo_config_node-1]   ok mode=position P=2000 I=0 D=0 FF1=0 FF2=0 vel=200 acc=50 goal_curr=200
[servo_config_node-1] 
[servo_config_node-1] === servo_config_node: done ===
[INFO] [servo_config_node-1]: process has finished cleanly [pid 611]
[INFO] [ros2_control_node-3]: process started with pid [624]
[ros2_control_node-3] [INFO] [1783546
:
:
```

### Step 5: Launch the ball balance node and wrist balance controller
These are also discussed in more detail in the [documentation for the balance node](./ball_balance_node.md) and that for the [wrist balance controller](./wrist_balance_controller.md).

### Step 6: Launch the power monitor and imu node
These take care of the external sensors (INA226 current sensors and IMU sensor).

```
ubuntu@bueche-rpi5:~/robot_ws$ ros2 run writing_robot_control power_monitor_node

[INFO] [1783546835.463551482] [power_monitor_node]: Connected to ESP32 on /dev/ttyIna219 at 115200 baud
[INFO] [1783546837.464657527] [power_monitor_node]: Power monitor node started
```

and 

```
ubuntu@bueche-rpi5:~/robot_ws$ ros2 run writing_robot_control imu_balance_node
[INFO] [1783546819.931613581] [imu_balance_node]: Opening serial port /dev/ttyIMU...
[INFO] [1783546819.932385679] [imu_balance_node]: IMU balance node started — /dev/ttyIMU @ 115200 baud, 50Hz publish rate
[INFO] [1783546819.934893583] [imu_balance_node]: Topics: /imu/raw  /imu/balance_error  /imu/balance_cmd  /imu/is_stable
[INFO] [1783546819.937843046] [imu_balance_node]: Calibrate IMU: ros2 service call /imu/calibrate std_srvs/srv/Trigger "{}"
[INFO] [1783546821.032117054] [imu_balance_node]: stty configured CP210x
[INFO] [1783546821.037916645] [imu_balance_node]: CP210x opened O_RDONLY — RTS not asserted, data will flow
[INFO] [1783546821.138958916] [imu_balance_node]: Serial port open — buffer flushed.
[INFO] [1783546821.170528333] [imu_balance_node]: Raw read test (20 bytes): b'# RAW ax=-1556 ay=-1'
[INFO] [1783547017.366142264] [imu_balance_node]: Arm SETTLED — IMU publishing continues.
[INFO] [1783547102.440154454] [imu_balance_node]: Arm MOVING — IMU publishing continues.
[
```
### Step 7: Launch the pid and current loggers 
These nodes store detailed topic information on the movement of the joints and the electrical behavior of the robot respectively. They log their infomration to a file on the file system so should only be running during the actual test.


```
ubuntu@bueche-rpi5:~/robot_ws$ ros2 run writing_robot_control pid_logger
[INFO] [1783547094.259052777] [pid_logger]: pid_logger started — writing to ./pid_log_20260708_214454.csv
[INFO] [1783547094.260191044] [pid_logger]: Logging at /joint_states rate (100Hz) — ball/IMU held at their native rates
[INFO] [1783547102.442729822] [pid_logger]: Pose started: pose 1 - initial (#1)

```
and

```
ubuntu@bueche-rpi5:~/robot_ws$ ros2 run writing_robot_control power_logger
[INFO] [1783546872.491511213] [power_logger_enhanced]: Logging to: power_log_20260708_214112.csv
[INFO] [1783546872.492295236] [power_logger_enhanced]: Tracking: INA226 power (primary) + Servo currents
[INFO] [1783547010.356605090] [power_logger_enhanced]: → Pose started: pose 1 - initial
[INFO] [1783547102.439814915] [power_logger_enhanced]: → Pose started: pose 1 - initial
[INFO] [1783547116.922453863] [power_logger_enhanced]: ← Pose ended: pose 1 - initial
[INFO] [1783547116.923098256] [power_logger_enhanced]:    Peak servo currents (mA):
[INFO] [1783547116.925730365] [power_logger_enhanced]:      elbow_flex  :  339.0
[INFO] [1783547116.927332245] [power_logger_enhanced]:      wrist_flex  :   71.0
[INFO] [1783547116.928105527] [power_logger_enhanced]:      wrist_roll  :   18.0
[INFO] [1783547116.929350294] [power_logger_enhanced]:      pen_holder  :   16.0
:
:
```
### Step 7: turn on the system watch dog node
This monitors the latency of topics and across hosts.
```
ubuntu@bueche-rpi5:~/robot_ws$ ros2 run writing_robot_control system_watchdog
[INFO] [1783546947.102875785] [system_watchdog]: System watchdog started — monitoring 8 topics, reporting at 0.2Hz
[INFO] [1783546952.077402223] [system_watchdog]: === System Watchdog ===
  /ball/position: NO DATA
  /ball/cup_detected: NO DATA
  ✅ /imu/balance_error: 50.3Hz  max_gap=21ms  spikes=0  msgs=251
  /imu/balance_cmd: NO DATA
  ✅ /imu/is_stable: 50.3Hz  max_gap=21ms  spikes=0  msgs=251
  /balance_enabled: NO DATA
  ✅ /joint_states: 100.0Hz  max_gap=13ms  spikes=0  msgs=497
  /ball/image: NO DATA
  Pipeline latency: calibrating...
  Overall: ⚠️  ISSUES DETECTED
[INFO] [1783546957.077359467] [system_watchdog]: === System Watchdog ===
  /ball/position: NO DATA
  /ball/cup_detected: NO DATA
  ✅ /imu/balance_error: 50.0Hz  max_gap=21ms  spikes=0  msgs=501
  /imu/balance_cmd: NO DATA
  ✅ /imu/is_stable: 50.0Hz  max_gap=21ms  spikes=0  msgs=501
  /balance_enabled: NO DATA
  ✅ /joint_states: 100.0Hz  max_gap=13ms  spikes=0  msgs=997
  /ball/image: NO DATA
  Pipeline latency: calibrating...
  Overall: ⚠️  ISSUES DETECTED
[INFO] [1783546962.077551730] [system_watchdog]: === System Watchdog ===
```
### Step 8: Turn on the video capture of rviz animation if desired
We use peek on the nvidia orin nano.

### Step 9: Start the pose and balance sequence
We use the pose test utility for this purpose.

```
ubuntu@bueche-rpi5:~/robot_ws$ ros2 run writing_robot_control pose_test --ros-args -p poses_file:=src/writing_robot_description/config/balance_v1.yaml -p validate:=true -p telemetry:=true -p urdf_file:=src/writing_robot_description/urdf/koch_v11_arm_real_for_balance.urdf -p power_monitoring:=true -p delay:=5.0 -p wait_for_stable:=true -p stable_timeout:=20.0
POWER_MONITORING_AVAILABLEF: True
[INFO] [1783547100.375273498] [pose_test_node]: Loading URDF limits...
[INFO] [1783547100.378236201] [pose_test_node]: ✓ Loaded limits for 6 joints
[INFO] [1783547100.379325597] [pose_test_node]: Initializing telemetry...
[INFO] [1783547100.387277553] [pose_test_node]: ✓ Telemetry initialized (dual-topic mode)
[INFO] [1783547100.387969835] [pose_test_node]:   - Subscribing to /joint_states
[INFO] [1783547100.388619580] [pose_test_node]:   - Subscribing to /dxl_state
[INFO] [1783547100.389486993] [pose_test_node]:   - XL430 servos (IDs 1,2): Load only
[INFO] [1783547100.390356979] [pose_test_node]:   - XL330 servos (IDs 3-6): Current + Load
[INFO] [1783547100.392174954] [pose_test_node]: ✓ Telemetry active
[INFO] [1783547100.397957545] [pose_test_node]: ✓ Power monitoring enabled
[INFO] [1783547100.399250090] [pose_test_node]: ============================================================
[INFO] [1783547100.400570635] [pose_test_node]: POSE TEST NODE INITIALIZED
[INFO] [1783547100.401849458] [pose_test_node]: ============================================================
[INFO] [1783547100.402987613] [pose_test_node]: Joints: 6
[INFO] [1783547100.404043064] [pose_test_node]: Validation: True
[INFO] [1783547100.405095089] [pose_test_node]: Telemetry: True
[INFO] [1783547100.406757932] [pose_test_node]: Power Monitoring: Enabled
[INFO] [1783547100.408066329] [pose_test_node]: Movement time: 2.0s
[INFO] [1783547100.409367541] [pose_test_node]: Delay between poses: 5.0s
[INFO] [1783547100.410620438] [pose_test_node]: Wait for stable: YES (timeout: 20.0s)
[INFO] [1783547100.412071632] [pose_test_node]: ============================================================
[INFO] [1783547100.412887155] [pose_test_node]: Waiting for joint trajectory controller...
[INFO] [1783547102.426382814] [pose_test_node]: ✓ Loaded 11 poses from src/writing_robot_description/config/balance_v1.yaml
[INFO] [1783547102.426991077] [pose_test_node]: 
============================================================
[INFO] [1783547102.427521265] [pose_test_node]: POSE SEQUENCE TEST
[INFO] [1783547102.428018361] [pose_test_node]: ============================================================
[INFO] [1783547102.428657680] [pose_test_node]: Total poses: 11
[INFO] [1783547102.429442314] [pose_test_node]: Validation: True
[INFO] [1783547102.430010114] [pose_test_node]: Telemetry: True
[INFO] [1783547102.430557691] [pose_test_node]: ============================================================
[INFO] [1783547102.431205807] [pose_test_node]: 
[1/11] pose 1 - initial
[INFO] [1783547102.432973484] [pose_test_node]:   ✓ shoulder_pan: 1.500 rad (within [0.540, 2.044])
[INFO] [1783547102.434090713] [pose_test_node]:   ✓ shoulder_lift: 2.563 rad (within [2.200, 2.886])
[INFO] [1783547102.434860885] [pose_test_node]:   ✓ elbow_flex: 1.470 rad (within [0.726, 2.250])
[INFO] [1783547102.435565371] [pose_test_node]:   ✓ wrist_flex: 2.700 rad (within [0.297, 2.700])
[INFO] [1783547102.436182411] [pose_test_node]:   ✓ wrist_roll: 1.600 rad (within [-1.448, 1.900])
[INFO] [1783547102.436800100] [pose_test_node]:   ✓ pen_holder: 0.977 rad (within [0.190, 1.600])
[INFO] [1783547102.441800631] [pose_test_node]: → Sent: pose 1 - initial
[INFO] [1783547109.444336614] [pose_test_node]:   ⏳ Waiting for ball to center (timeout: 20.0s)...
[INFO] [1783547116.919794995] [pose_test_node]:   ✓ Cup stable after 7.47s
[INFO] [1783547116.923481795] [pose_test_node]:   📊 Joint States:
[INFO] [1783547116.927325485] [pose_test_node]:   📊 shoulder_pan    [XL430] Pos:  1.497rad Load:  0.50% Volt:11.5V Temp: 36.0°C
[INFO] [1783547116.928125361] [pose_test_node]:   📊 shoulder_lift   [XL430] Pos:  2.494rad Load: 24.70% Volt:11.5V Temp: 39.0°C
[INFO] [1783547116.930858359] [pose_test_node]:   📊 elbow_flex      [XL330] Pos:  1.534rad Curr: -330mA Volt: 4.8V Temp: 29.0°C
[INFO] [1783547116.931532919] [pose_test_node]:   📊 wrist_flex      [XL330] Pos:  2.467rad Curr:   17mA Volt: 4.8V Temp: 23.0°C
[INFO] [1783547116.932140700] [pose_test_node]:   📊 wrist_roll      [XL330] Pos:  1.420rad Curr:    0mA Volt: 4.8V Temp: 23.0°C
[INFO] [1783547116.933323078] [pose_test_node]:   📊 pen_holder      [XL330] Pos:  0.974rad Curr:   15mA Volt: 4.7V Temp: 24.0°C
[INFO] [1783547116.934215695] [pose_test_node]:   ⚡ Power Analysis for "pose 1 - initial":
[INFO] [1783547116.935191719] [pose_test_node]:      Peak 5V current (INA226):  0.409A
[INFO] [1783547116.936076076] [pose_test_node]:      Peak motor sum (XL330s):     0.458A
[INFO] [1783547116.937022934] [pose_test_node]:      5V voltage: 5.42V (min) / 5.97V (max)
[INFO] [1783547116.938087830] [pose_test_node]:      Peak 12V current (INA226): 0.228A
[INFO] [1783547116.939169818] [pose_test_node]:      Peak total power:            4.74W
[INFO] [1783547116.940125324] [pose_test_node]:      Holding current comparison:
[INFO] [1783547116.941109867] [pose_test_node]:        Supply current (INA226):  0.388A
[INFO] [1783547116.941983743] [pose_test_node]:        Motor current sum (XL330s):  0.362A
[INFO] [1783547116.943054249] [pose_test_node]:        ✓ Supply within 50mA of  motor sum
[INFO] [1783547116.943754402] [pose_test_node]: 
[2/11] pose 2 -tilt cup right
[INFO] [1783547116.944361053] [pose_test_node]:   ✓ shoulder_pan: 1.500 rad (within [0.540, 2.044])
[INFO] [1783547116.944949613] [pose_test_node]:   ✓ shoulder_lift: 2.563 rad (within [2.200, 2.886])
[INFO
```

### Step 10: Save data and analyze
After each run we collect and analyze data. 
1. add the output of the `pose_test`, `ball_balance_node`, `wrist_balance_controller`, `ball_detector node`, and `system watch dog node` to a single file (called: `test.results.MMDDYYY.tN.cv3.txt` where N = number of tests in the day MMDDYYY).
2. do the overall analysis. Example: `%python3 balance_analysis.v10.py test.results.070826.t1.cv3.txt  > test.analysis.070826.t1.cv3.txt`
3. do the correction analysis. Example: `% python3 corr_analysis.v3.py test.results.070826.t1.cv3.txt  --max-steps 100 > test.corr.070826.t1.cv3.txt`
4. upload all of the data into the google drive.