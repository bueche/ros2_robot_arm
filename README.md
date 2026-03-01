# Ros2 Robot Arm Study Using Koch v1.1 Follower

## Summary
The following repository has the source, resources, and instructions for building a robotic arm and its digital twin. The arm is based on a modified Koch V1.1 "follower arm" physical design (see [Robotis Koch v1.1 follower arm](https://robotis.us/koch-v1-1-low-cost-robot-arm-follower/)) and its powered by The Robot Operating System v2 [ROS2](https://github.com/ros2).  This repository supports both [Ros2 Jazzy](https://docs.ros.org/en/jazzy/index.html) and [Ros2 Humble](https://docs.ros.org/en/humble/index.html) (see respective  branches in this repository). 

This effort is meant to learn and explore both the software and hardware used in robotic arm development. See FAQ at the bottom for more information.

This branch supports ros2 humble.

This includes:
 - Fabrication and/or purchase, modification, and assembly of robotic parts (brackets, Dynamixel servos, and related circuit boards and sensors)
 - Testing and tuning of the physical robot
 - Creating, calibrating, and tuning of the robot arm digital twin
 - Refinement of ROS2-based software to support various actions of the robotic arm and the gathering of physical statistics (current, voltage, temperature, etc)

<p align="center">
  <img src="./images/real_robot.jpg" alt="Real Robot Arm" width="400">
  <img src="./images/digital_twin.jpg" alt="The digital twin" width="400">
  <img src="./images/freecad.robot.representation.jpg" alt="The freecad digital twin" width="400">
</p>

The documentation for this repository covers the following:
- [Software Setup](./documentation/software_setup.md): how to create the docker envrionment and any additional setup commands. 
- [Hardware Setup](./documentation/hardware_setup.md): Although the setup of the koch_v11 physical robot (leader and follower arms) is partially documented ([Arm assembly by Jess Moss](https://www.youtube.com/watch?v=8nQIg9BwwTk)), those instructions lacked details on some of the required electrical setup. In addition, we are using different controller boards (e.g., Dynamixel OpenRB 150).
- [Firmware Setup and Configuration](./documentation/firmware.md): Notes on the firmware setup and configuration for the OpenRB 150, the servo motors, the current sensors, and the ESP32.
- [Notes on creating the digital twin](./documentation/digital_twin.md): A series of notes that cover the problems and solutions for creating the digital twin.
- [Launching the Physical Robot](./documentation/launching_physical.md) Steps involved in bringing up the physical robot.
- [Running the display only version of the Digital twin](./documentation/display_launch.md): This describes how to bring up the digital twin - either along side the physical robot or by itself as an rviz display-only flavor of the robot.
- [Pose Sequence Tools and Analysis](./documentation/utilities.md): Several tools are provided that drive the robot through a series of poses and analyze physical characteristics (position, temp, current, voltage, etc). This section also provides some analysis case studies that are of interest.

## Medium articles:
- TBD 🚧 

## FAQ
### How can this repository be useful?
This repository can be of use to someone building robotics in several ways:
1. *Users of the Koch V1.1 follower arm*: Although some of the design and operation of the Koch v1.1 robot arm is open source, a user of it may encounter a number of gaps or issues when working with it. This repository should complement information and software from ROBOTIS and other sources. For example, we provide a  [koch v1.1 URDF](https://github.com/bueche/ros2_robot_arm/blob/main/writing_robot_description/urdf/koch_v11_arm_real.urdf) file that is configured and calibrated to provide a digital twin that matches the physical robot in appearance. In addition, we provide [complete and accurate wiring diagrams](https://github.com/bueche/ros2_robot_arm/blob/main/documentation/hardware_setup.md#electrical-notes) that detail how to co-exist the 12V and 5V servos in the same environment.
   
2. *Users who are adapting robotic arms to ROS2 or not starting from a well defined kit*: ROBOTIS provides several full kits for someone getting started with robot arms that provide a good head start (including this [Koch v1.1.](https://robotis.us/koch-v1-1-low-cost-robot-arm-follower/), [OMX AI manipulator](https://robotis.us/omx-ai-us/), [OMY AI manipulator](https://robotis.us/omy-ai3m/), and more. This project provides information for someone who is departing from an easy-to-assemble kit.
 
3. *Users wanting to pull more telemetry from Dynamixel servos in a ROS2 environment*: The ros2 interface for the Dynamixel servos is quite limited relative to the internal data it can pull (as compared with the Dynamixel Wizard tool). For example, one can't get a reading on the voltage seen by a servo, even though extra strain on the system could lead to an under-volt situation for the servos in series. We provide information on how to improve this for ros2 and how to set this up ([in code](https://github.com/bueche/ros2_robot_arm/blob/main/documentation/software_setup.md#dynamixel-software-changes)) and retrieve the values at runtime via ros2.
   
### What is a leader-follower robotic arm?
A leader-follower robotic arm system consists of two robotic manipulators where one arm (the leader) is manually controlled by a human operator, and the second arm (the follower) replicates those movements in real-time. It's essentially a form of bilateral teleoperation. In this project we are primarily focused on the follower arm.
### Why is this project focused on the follower?
In general leader arms are meant more for human manipulation as they define the poses that the follower will replicate. That limited operational scope for the leader means that its likely that the arm is under powered and/or has a design that is for human manipulation vs. automation. In this project we use the follower as the leader (to predefine poses and collect the measurements) albiet in a less automatic fashion. We also want to study the follower's load and physical characeteristics (e.g., temperature) as it repeatedly performs the activity.
