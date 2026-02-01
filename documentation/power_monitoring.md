# Power Monitoring of Robotic Arm

This project also includes utilities to measure and correlate the power draw of servos with the sequence of poses as they enfold. Two INA219 current sensors have been added to the setup (see hardware) and the utilities and software enhanced to report and analyze  this in a ros2 environment. The code also supports having INA226 sensors in addition to the INA219 ones (both sensors at the same time). The results between the two sensors are close enough so in general this is not really needed.

<p align="center">
  <img src="../images/current_v_pose.001.jpeg" alt="current sensor data" width="900">
</p>

This setup includes:
- Two different arduino sketches to be run on the ESP32 dev boarding that is monitoring current:
   - [read_ina219.ino](../src/power_monitor/sketch/read_ina219.ino): ESP32dev sketch to collect data from the two INA219 sensors.
   - [read_ina219_ina226.ino](../src/power_monitor/sketch/read_ina219_ina226.ino): ESP32dev sketch to collect data from the two INA219 sensors and an optional ina226 sensor on the 5v rail.
- [power_monitor_node.py](../src/writing_robot_control/writing_robot_control/power_monitor_node.py): A ros2 node that interfaces with the ESP32 board that is communicating with the INA219 sensors. An arduino sketch is running on the ESP32 board which samples the sensor data and passes the inforamtion back via USB. This node takes that infomration and publishes it in ROS2.
- [power_logger.py](../src/writing_robot_control/writing_robot_control/power_logger.py): A ros2 client which pulls the power data (from the power_monitoring_node) and the pose data (from pose_test) and writes this to a CSV file for further analysis. Each timestamp is 100ms from the last by default.

 
