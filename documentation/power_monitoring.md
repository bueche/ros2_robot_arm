# Power Monitoring of Robotic Arm

This project also includes utilities to measure and correlate the power draw of servos with the sequence of poses as they enfold. Two INA219 current sensors have been added to the setup (see hardware) and the utilities and software enhanced to report and analyze  this in a ros2 environment. The code also supports having INA226 sensors in addition to the INA219 ones (both sensors at the same time). The results between the two sensors are close enough so in general this is not really needed.

The particular example illustrated is defined by [this pose sequence](../src/writing_robot_description/config/delivery_poses.yaml) and is illustrated in the story board below (which was an rviz2 reproduction of the actual physical robot movements). 

<p align="center">
  <img src="../images/story_board_for_example_pose_test.jpg" alt="story board for example pose test" width="900">
</p>

and the electrical wiring of the physical robot (including current sensors) is detailed [here](https://github.com/bueche/ros2_robot_arm/blob/main/documentation/hardware_setup.md#electrical-notes).

<p align="center">
  <img src="../images/current_v_pose.001.jpeg" alt="current sensor data" width="900">
</p>

This setup includes:
- Two different arduino sketches to be run on the ESP32 dev boarding that is monitoring current:
   - [read_ina219.ino](../src/power_monitor/sketch/read_ina219.ino): ESP32dev sketch to collect data from the two INA219 sensors.
   - [read_ina219_ina226.ino](../src/power_monitor/sketch/read_ina219_ina226.ino): ESP32dev sketch to collect data from the two INA219 sensors and an optional ina226 sensor on the 5v rail.
- [power_monitor_node.py](../src/writing_robot_control/writing_robot_control/power_monitor_node.py): A ros2 node that interfaces with the ESP32 board that is communicating with the INA219 sensors. An arduino sketch is running on the ESP32 board which samples the sensor data and passes the inforamtion back via USB. This node takes that infomration and publishes it in ROS2.
- [power_logger.py](../src/writing_robot_control/writing_robot_control/power_logger.py): A ros2 client which pulls the power data (from the power_monitoring_node) and the pose data (from pose_test) and writes this to a CSV file for further analysis. Each timestamp is 100ms from the last by default.

 
## Observations and Investigations To Come

- **The Jump in current from pose 5 to pose 6**: Pose 5 represents a movement to the "place high" position and Pose 6 just involves the opening of the pen-link (or thumb) while in that high position. This raises the total current draw on the 5v servos significantly and unexpectedly (see graph and story board above). This might imply opportunities for movement optimization.
- **The difference in servo reported current vs. external sensor reported current**: That is, if we sum the Dynamixel reported "Present Input current" with the input current measured by the sensors the difference is in total ~50 mA (total for all four XL330 servo's combined). This is pretty consistent for the moving robot, but the difference is even higher for the "at rest" robot (~65 mA). This could be due to calibration on the wrt to the sensors, but they match closely and are far closer to a multi-meter reading (see diagram above).

