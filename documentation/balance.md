# Balancing a cup with the robot arm

In this section we describe the project to balance a cup using the koch v1.1 robotic arm. This is useful to explore the ulility of [PID controllers](https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller) and vision machine learning in robotics. 
We use it also to apply the Robotis Dynamixel servos on a more challenging task with this Koch v1.1 robotic arm.

This high-level ROS2-based architecture is shown below. We highlight some of the differentiating nodes, clients, and controllers.
We have enhanced the koch v1.1 follower robot with an [IMU sensor](https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller) and a [Luxonis OAK-D lite 3D inferencing camera](https://www.luxonis.com/). A shallow cup has been 3D printed and 
attached to the arm end-effector along with a ball bearing that will roll into the center of the cup 
when it is balanced. The image and IMU sensor provide input to a special purpose PID controller that makes fine motor adjustments for two of the six Dynamixel servos in the arm (the wrist_flex and wrist_roll joints). The PID input kicks in after the current pose has been set (using the the [pose test driver](./pose_test.md)). The poses, defined in the [balance_v1.yaml](../writing_robot_description/config/balance_v1.yaml) file as input to the [pose_test utility](./pose_test.md), essentially tip the cup and ball to different sides. Once a step is completed then the PID controller (in this case [wrist_balance_controller](./balance_pid_controller.md)) will make fine-level adjustments to try to bring the cup level and the ball into its center. The PID adjustments are based on the ball position and on the IMU feedback. The PID controller times out  after 5+ seconds and ceases tryng to achieve balance (in order to prevent hardware burnout).

<p align="center">
  <img src="../images/balance_high_level_architecture.jpg" alt="high level balance architecture" width="700">
  
</p>

The key camera interface is the [ball_detector_oak.py](../balance/balance/ball_detector_oak.py) or [ball_detector_nvidia.py](../balance/balance/ball_detector_nvidia.py). The former node pulls the ball and cup coordinates infered by the 640x640 YOLOv8n ML model running on the camera as well as a lower resolution debug image stream. The latter is used to have the Nvidia Orin Nano do the inferencing instead. Both are driven by a  model that was trained on ~1500 pictures of the cup+ball (and cup-only and no-cup or ball). The debug image stream is annotated by these nodes to draw the inferred bounding boxes and this is displayed in rviz.

This sub-project builds upon the earlier work to accurately measure the robot's input current and voltage. The importance of gathering 
those measurements becomes clearer as fine adjustments and error correction could lead to undesiable current spikes (and voltage drops).  

The video clip below shows the rviz output for a portion of the test (as of 04/24/2026). Currrently the ball never makes it to the center but tuning is underway.

<p align="center">
  <img src="../images/balance_video_camera_inference.gif" alt="using the camera's inferencing ability to detect cup and ball" width="700">
  
</p>

The rest of this page provides more detail or links to more detail on this sub-project.

## Sections
- [Hardware](./hardware_for_balance.md): This section focuses on the hardware added to the base robot hardware to support the cup balancing project, with specific emphasis on the 3D Inferencing camera and IMU. [Learn more...](./hardware_for_balance.md)
- [Software](software_for_balance.md): A number of software components were added to support the balance project. This section outlines them. [Learn more...](./software_for_balance.md)
- [Vision Model training and building](./vision_model_training.md): A key capability for the cup balancing project was the Machine Learning model used to detect the position of the ball bearing within the cup. We explored two different approaches: ML inferencing on the camera and inferencing on the Nvidia Orin Nano's GPUs. This section describes how the two models were built. [Learn more...](./vision_model_training.md)
- [Ball detection tuning](./ball_detection_tuning.md): This section describes some of the tuning steps wrt ball detection. It also compares camera inferencing vs. inferencing on the Nvidia Orin Nano. [Learn more ...](./ball_detection_tuning.md)
- [Dynamixel PID controller tuning](./dynamixel_pid_controller_tuning.md): There was actually multilple levels of PID-related control. Indeed each Dynamixel servo has a PID controller. We did need to make some adjustments to that configuration for this application. [Learn more...](./dynamixel_pid_controller_tuning.md)
- [Balacing PID controller](./ball_balance_pid_documentation.md): The above described ball detection fed into an overall PID controller whose focus was to balance the ball bearing. How do you adjust a servo based on knowing its coordinates? [Learn more..](./ball_balance_pid_documentation.md)
- [Balancing Metrics](./balance_metrics.md): As the robot works to balance the cup alot is happening dynamically. We derived a number of metrics that are helpful to determine if a run has improved over the last one. This sections discusses the metrics in detail and outlines their evolution. [Learn more..](./balance_metrics.md)
- [Balancing Test Results](./balance_test_results.md): Getting to a point in which the robot can balance the cup effectively will be the result of many tests and attention to the metrics. This section covers the latest test results in more detail. [Learn more...](./balance_test_results.md).
- [Current status and next steps](#current-status-and-next-steps) 
- [FAQ](#faq)



## Current Status and Next Steps
- 05/2026 - Entire architecture is more or less working, but the task is not complete (ball not balanced).
  1. The imu readings are not calibrated and hence although collected  unused to impact the servo movements.
  2. More tuning of the PID controller is required to complete the balance task.
  3. We are likely to build the version 3 cup so that the surface is a bit more concave.
  


## FAQ
### Why balance a cup (or ball within a cup)?
As was noted in the introduction the task of balancing a ball within a cup is a great way to get started with vision machine learning and PID controllers. These are foundational components of more sophisticated systems. In addition, we spent a bit of time and effort ensuring that we had accurate electrical information reported by the robot. Those measurements become foundational and important as we have this low-cost robotic arm do more interesting work.

### What have been some of the learnings so far?
TBD ... 
need to discuss some of the following:
1. capacity limitations on the camera (there were a number ... perhaps unsurprising for OAK-D lite)
2. nvidia orin nano USB ports ... mis-advertised as USB-3?
3. Nvidia Orin Nano (Unbuntu 22.04) ... can't get Jazzy working, stuck with humble (so learning how to Jazzy & humble nodes can co-exist)
4. the trade-off between the faster inferencing with lower resolution images vs. the inaccuracies of those faster inferences
5. Why INA226 current sensors are better than INA219 ones.
6. Undocumented behavior when using CP210x USB-UART
7. 

