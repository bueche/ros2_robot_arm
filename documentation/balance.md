# Balancing a cup with the robot arm
**DRAFT ... UNDER CONSTRUCTION**

In this section we describe the project to balance a cup using the koch v1.1 robotic arm. 
We have enhanced the robot with an [IMU sensor](https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller) and a [Luxonis 3D camera](https://www.luxonis.com/). A shallow cup
is attached to the arm end-effector along with a ball bearing that will roll into the center of the cup 
when it is balanced. The image and IMU sensor provide input to a special purpose [PID controller](https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller) that makes fine motor adjustments for two of the six Dynamixel servos in the arm. 

This project builds upon the earlier work to accurately measure the input current and voltage. The importance of gathering 
those measurements becomes clearer as fine adjustments and error correction could lead to undesiable current spikes (and voltage drops).  

<p align="center">
  <img src="../images/balance_video_camera_inference.gif" alt="using the camera's inferencing ability to detect cup and ball" width="400">
  
</p>

## Sections
- [Hardware](#hardware)
- [Software](#software)
- [Vision Model training and building](#vision-model-training-and-building)
- [Ball detection tuning](#ball-detection-tuning)
- [Dynamixel PID controller tuning](#dynamixel-pid-controller-tuning)
- [Balacing PID controller tuning](#balancing-pid-controller-tuning) 
- [Current status and next steps](#current-status-and-next-steps) 

## Hardware 
The additional hardware for this cup balancing portion of the project is identified in the [hardware section](./hardware_setup.md#balance-related-hardware). In this section we describe some key aspects of the two main additional components: the Luxonis Oak D camera and the IMU. 

### Camera 
In this section we discuss some points related to camera's and this application. 
- Introduction: As noted earlier the basic function of the camera is to track the movement and coordiates of the ball within the cup. The goal is to get the ball to the middle of the cup and the ball is typically on the perimeter of the cup. Its current coordinates need to determined accurately and with low latency. 

- Inferencing camera vs. basic USB camera: 
  - *Why use an inferencing camera?* The basic flow requires that images are fed to a model which detects the cup and ball and provides the coordinates of the ball within the cup. The theoretically benefit of the inferencing camera is its ability to avoid having the transport those images in a high frame rate to the inferencing software running on a different host / processor. Instead the inferencing camera can do the detection locally and only report coordinates (which require significantly less bandwidth). We are motivated to reduce the image transport bandwidth partly because of Nvidia Orin Nano's USB-2 limitation. 
  - *Why does USB-2 have insufficient bandwidth for streaming video for this application?* The raw image bandwidth formula can be put as follows:
  ```
  bandwidth = image_width × image_height × channels × bytes_per_channel × frames_per_sec x jpeg_compression_factor
  ```
  - where:
    - image_width x image_height define the pixels/frame.
    - Ideally, we could process at least 30 frames per sec to ensure smooth/fluid movement as perceived by our eyes, but this rate has a signficant impact on bandwidth.
    - there are 3 channels of  ....
    - the compression factor is 1/15
  - USB bandwidth:
    | Standard | Theoretical| Practical (60-70%) | 
    |----------|------------|--------------------|
    | USB 2.0 High Speed | 480 Mbps | ~300 Mbps | 
    | USB 3.0 SuperSpeed | 5000 Mbps | ~3000 Mbps |

- Camera inferencing vs. Nvidia inferencing
  - https://docs.luxonis.com/hardware/platform/rvc/rvc2/ (capacity of RVC2)
  - 
- 3D camera vs. 2D camera

Some discussion points on IMU:
- IMU level vs. ball level
- IMU calibration challenges
- IMU wiring impact on servo accuracy and mobility 
- also reference the Dynamixel servo tuning section

Discussion on the Koch v1.1 and:
- its ability to hold a cup steady
- its ability to hold a cup + ball + IMU + wires for IMU

## Software
- draw software architecture and highlight the new ros2 components needed for this project
- outline each of the new components and their purpose
- describe the hybrid jazzy / humble approach
- Describe the software/hardware approaches for ball inference

## Vision Model training and building
This section should outline the steps and services used in training and building the cup and ball detection and tracking model. 

## Ball Detection Tuning
A number of steps were needed to get the ball detection node working properly. This section discusses some of them and includes:
- coordinates 
- training issues: resolution vs. size
- camera exposure time
- FPS and impact on rviz and PID effectiveness
- USB bandwidth bottleneck on Nvidia orin nano
- Camera inference timing

## Dynamixel PID controller tuning
In this section we should write up the findings on the needed PID tuning of the dynamixel servos and the accellerated 
current draw and the INA219 current bottleneck (and the work around)

## Balancing PID controller tuning
- TBD

## Current Status and Next Steps
- TBD

