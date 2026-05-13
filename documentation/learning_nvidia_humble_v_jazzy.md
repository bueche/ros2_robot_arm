# LEARNING: NVIDIA Jetson Orian Nano doesn't do Ros2 Jazzy well.. and what to do about that..

The tech goal for this project was to, for as much as possible, work with the most modern version of ros2 as possible (LTS versions). The major learning was that the Nvidia Orin Nano does Jazzy poorly. The Orin Nano supports a version of Ubuntu 22.04 and [Ros2 Jazzy(https://docs.ros.org/en/jazzy/) supports only Ubuntu 24.04 ([here](https://docs.ros.org/en/jazzy/Installation.html)), so perhaps this may seem obvious. But its possible to build  and run 24.04 based Docker containers on the Orin Nano, but after much effort it was not possible to get Jazzy Ros2 software to access the GPU related software needed for ML inference. This only seems possible using [Ros2 Humble](https://docs.ros.org/en/humble/). Luckily, humble ros2 nodes are interoperable with Jazzy ones (not perfectly but enough at least).

## Why do we need to access the Orian Nano's GPU for inference?
- reference the learnings on the camera
- basic answer: inference constrained to 12 Frames per sec
- possibility of 60+ FPS


## ROS2 Jazzy not on Ubuntu 22.04? 
ROS 2 Jazzy Jalisco does not officially support Ubuntu 22.04 (Jammy) because it is designed to align with the newer Ubuntu 24.04 LTS (Noble Numbat). Jazzy requires newer system libraries, particularly Python 3.12 and updated middleware, which are native to 24.04, whereas 22.04 relies on older packages that cause conflicts.


## What are the Docker approach options? and why didn't they work?
Now there were several options tried in order to get 

## The Ros2 Humble option..

## ROS Humble to Jazzy  works but has issues..

