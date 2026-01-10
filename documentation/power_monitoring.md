# Power Monitoring of Robotic Arm

This project also includes utilities to measure and correlate the power draw of servos with the sequence of poses as they enfold. Two INA219 current sensors have been added to the setup (see hardware) and the utilities and software enhanced to report and analyze  this in a ros2 environment. 

<p align="center">
  <img src="../images/current_v_pose.001.jpeg" alt="current sensor data" width="900">
</p>

This setup includes:
- [power_monitor_node.py](../src/writing_robot_control/writing_robot_control/power_monitor_node.py): A ros2 node that interfaces with the ESP32 board that is communicating with the INA219 sensors. An arduino sketch is running on the ESP32 board which samples the sensor data and passes the inforamtion back via USB. This node takes that infomration and publishes it in ROS2.
- [power_logger.py](../src/writing_robot_control/writing_robot_control/power_logger.py): A ros2 client which pulls the power data (from the power_monitoring_node) and the pose data (from pose_test) and writes this to a CSV file for further analysis. Each timestamp is 100ms from the last by default.


 
