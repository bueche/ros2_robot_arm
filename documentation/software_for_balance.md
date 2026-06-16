# Software
- draw software architecture and highlight the new ros2 components needed for this project
- outline each of the new components and their purpose
- describe the hybrid jazzy / humble approach
- Describe the software/hardware approaches for ball inference

## 1. System Architecture and Major components

The ball balancing system consists of five nodes working in a pipeline:
<p align="center">
  <img src="../images/balance_topic_flow.jpg" alt="ball_orientation " width="800">
</p> 

1. The camera interfaces either with `ball_detector_oak.py` or `ball_detector_nvidia.py` depending on whether cup and ball inferencing is happening on the camera or the nvidia orin nano. These nodes publish the same topics, however, which consist of the ball coordinates within the image and whether a ball was detected or not (along with some debug information). For more details on these nodes see [here](./ball_detector.md).

2. The `ball_balance_node.py` then uses this information to work out the ball's coordinates within the cup. Then it formulates the Proportional adjustment portion of the PID command and publishes this as the topic `/imu/balance_cmd`. Note this is labeled as the node `/imu` due to originally all I had was the imu and not the camera. Need to clean this up. Also, note that the ball balance node doesn't know the position of the servos. So the pid command still needs to be translated into servo radian positions by the `wrist_balance_controller`. See [here](./ball_balance_node.md) for more details on the `ball_balance_node`.

3. The `imu_balance_node.py` interfaces with the IMU (BNO085/MPU-6050 on ESP32, mounted on wrist) and publishes `/imu/balance_error` and `/imu/raw` for monitoring and optional Derivative or D-term feedforward, but in this test run 
D-term gains were zero so it contributed no corrections. See [here](./imu_balance_node.md) for more information on the `imu_balance_node` and its supporting imu interfacing sketch running on the ESP32.

4. The `wrist_balance_controller` takes in the input from the camera (now formulated as a PID command), the servo positions, and the IMU and does the last bit of PID operation by translating this into servo radian targets. It publishes these to make this happen. See [here](./wrist_balance_controller.md) for more details on the `wrist_balance_controller`.



## 2. 