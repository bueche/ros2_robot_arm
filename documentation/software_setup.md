# Sofware Setup for Robot Arm

This section describes how to setup the software to run the koch_v11 follower on ros2. The ros2 environment configured and run on a docker image.

## Sections 
- [Underlying hardware and OS variations](#underlying_hardware_and_os_variations): This section briefly outlines some of the hardware/OS variations that this has been tested on.
- [Docker setup on host OS](#docker_setup_on_host_os): installing and setting up docker on the host os.
- [Docker image and container setup](#docker_image_and_container_setup): This section describes how to create the docker images needed for this project and how to startup and finish setting up the containers.
- [Dynamixel software changes](#dynamixel_software_changes): Outlines the changes made to the Dynamixel software to pull additional telemetry data from the servos on Ros2.
- [TODO list](#todo_list): some notes on changes to come.

## Underlying hardware and OS variations
- The robot software setup and operation has been tested on a Raspberry Pi5 running Ubuntu 22.04 (Jammy). 
- The robot software setup and rviz2 operation has been tested on an Nvidia Orin Nano dev kit also running Ubunto 22.04. Rviz is run on the Nano as the Raspberry pi is underpowered for simulation. In this setup rviz2 runs on the Nvidia and the ros2 robot interfacing software runs on the Pi. 

## Docker setup on host OS

1. After installing the OS on your Nvidia or Pi then install git.
```
$ sudo apt update
$ sudo apt install git -y
```

2. Configure git
```
$ git config --global user.name "Your Name"
$ git config --global user.email "youremail@example.com"
```
3. make your project directory and clone this project's source 

```
$ mkdir -p ~/robot_ws
$ git clone https://github.com/bueche/ros2_robot_arm.git src
```
The act of cloning it to a directory named `src` is intentional. Also, this `robot_ws` will be mirrored into the docker container.

### Docker Installation

See https://docs.docker.com/engine/install/ubuntu/ for more details.

On Nvidia do some additional docker setup steps to enable the Nvidia container runtime

```
$ sudo apt-get install nvidia-container-toolkit
$ sudo nvidia-ctk runtime configure --runtime=docker
$ sudo systemctl restart docker
$ sudo docker info | grep -i runtime
 Runtimes: runc io.containerd.runc.v2 nvidia
 Default Runtime: nvidia
```

## Docker image and container setup
1. Build the docker image
In the example below the user id that cloned this environment has a UID and GID of 1002. The default is 1000, so if different then pass in the alternative value (in this case 1002 ... see below).

```
$ cd ~/robot_ws/src
$ # create base ros 
$ sudo docker build --platform linux/arm64 --build-arg UID=1002 --build-arg GID=1002 -t ros-jazzy-arm64 --load  --no-cache -f docker/Dockerfile.ros-jazzy-arm64 .
$ # create image upon base
$ sudo docker build --platform linux/arm64 --build-arg UID=1002 --build-arg GID=1002 -t robot-jazzy-arm64 --load  --no-cache -f docker/Dockerfile.jazzy.arm64  .
```
This will run for a while and create the docker image.
```
$ docker image list
REPOSITORY           TAG       IMAGE ID       CREATED         SIZE
robot-jazzy-arm64   latest    7ad21e00ddc5   44 hours ago    5.52GB
ros-jazzy-arm64     latest    3756d0e83149   9 months ago    3.98GB
```
2. Start the container
```
$ cd ~/robot_ws/src
$ docker/run.sh 
```
Note: there may be some errors that print related to key setup failure. These can be ignored.

3. Run the post-container start script
There is a post-container script to run that will download some additional software and setup the environment.
```
$ cd robot_ws/src
$ls -lt additional_env_setup.sh 
-rwxrwxr-x 1 bueche bueche 3337 Feb 22 14:53 additional_env_setup.sh
$ bash  additional_env_setup.sh
```


### Dynamixel software changes
   - This ros2-based software requires some small enhancements to improve telemetry for the dynamixel servos. This involved modifying the [dynamixel_hardware_interface](https://github.com/ROBOTIS-GIT/dynamixel_hardware_interface) and [dynamixel_interface](https://github.com/ROBOTIS-GIT/dynamixel_interfaces) packages of Robotis. The changes are small. See the following PRs for more details.
      - [dynamixel_interface PR-9](https://github.com/ROBOTIS-GIT/dynamixel_interfaces/pull/9)
      - [dynamixel_hardware_interface PR-106](https://github.com/ROBOTIS-GIT/dynamixel_hardware_interface/pull/106)
    
These PRs are applied automatically to the Dynamixel software downloads done by the ` additional_env_setup.sh` noted above.

## TODO list
- Get this working on ros2 jazzy
- get amd64 containers tested on windows.
- clean up some noise in the scripts 

