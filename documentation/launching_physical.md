# Launching the Physical Robot 

Once the software and hardware envionment has been setup with the one-time setup items, then a few additional commands are needed to bring up the physical robot and its digital twin in Rviz2.

## Table of Contents
- [Ensure that the USB port can be accessed](../documentation/launching_physical.md#ensure_that_the_uSB_port_can_be_accessed)
- [Launch the robot via ros2](../documentation/launching_physical.md#launch_the_robot_via_ros2)

  
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
ubuntu@bueche-rpi5:~/robot_ws$ ros2 launch writing_robot_description koch_v11_real_robot.launch.py
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
