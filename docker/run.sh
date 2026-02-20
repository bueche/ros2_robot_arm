#!/bin/bash

set +e

# Usage information
function show_help() {
  echo -e "\nUsage: run_locomotive.sh [OPTIONS]\n
  Options:
  \t-i --image_name\t\t Name of the image to be run (default locomotive_air_cleaner_capstone-arm64).
  \t-c --container_name\t Name of the container (default locomotive_container).
  \t--use_nvidia\t\t Use NVIDIA runtime.
  Examples:
  \trun_locomotive.sh
  \trun_locomotive.sh --image_name custom_image_name --container_name custom_container_name\n"
}

# Parse arguments
while [[ "$#" -gt 0 ]]; do
    case $1 in
        -i|--image_name) IMAGE_NAME="${2}"; shift ;;
        -c|--container_name) CONTAINER_NAME="${2}"; shift ;;
        -h|--help) show_help ; exit 1 ;;
        --use_nvidia) NVIDIA_FLAGS="--gpus=all -e NVIDIA_DRIVER_CAPABILITIES=all" ;;
        *) echo "Unknown parameter passed: $1"; exit 1 ;;
    esac
    shift
done

# Set default values if not provided
IMAGE_NAME=${IMAGE_NAME:-locomotive_air_cleaner_capstone-arm64}
CONTAINER_NAME=${CONTAINER_NAME:-locomotive_container}

USER=ubuntu
SSH_PATH=/home/$USER/.ssh
WORKSPACE_SRC_CONTAINER=/home/$USER/ws/src
WORKSPACE_ROOT_CONTAINER=/home/$USER/ws
SSH_AUTH_SOCK_USER=$SSH_AUTH_SOCK

# Ensure cache folders exist
mkdir -p .build
mkdir -p .install

# Check if container name is already in use
if sudo docker container ls -a | grep "${CONTAINER_NAME}$" -c &> /dev/null; then
   echo -e "Error: Docker container named $CONTAINER_NAME is already running.\n"
   echo -e "Try removing it with: \n\tdocker rm $CONTAINER_NAME"
   echo -e "Or use a different container name."
   exit 1
fi

# Start the container
xhost +
sudo docker run --privileged --net=host -it $NVIDIA_FLAGS \
       -e DISPLAY=$DISPLAY \
       -e SSH_AUTH_SOCK=$SSH_AUTH_SOCK_USER \
       -v $(dirname $SSH_AUTH_SOCK_USER):$(dirname $SSH_AUTH_SOCK_USER) \
       -v /tmp/.X11-unix:/tmp/.X11-unix \
       -v $(pwd):$WORKSPACE_SRC_CONTAINER \
       -v $(pwd)/.build:$WORKSPACE_ROOT_CONTAINER/build:rw \
       -v $(pwd)/.install:$WORKSPACE_ROOT_CONTAINER/install:rw \
       -v $SSH_PATH:$SSH_PATH \
       -v /dev:/dev \
       -u 1000:1000 \
       --name $CONTAINER_NAME $IMAGE_NAME
xhost -

# Cleanup function to commit changes if needed
function onexit() {
  while true; do
    read -p "Do you want to overwrite the image '$IMAGE_NAME' with current changes? [y/n]: " answer
    if [[ "${answer:0:1}" =~ y|Y ]]; then
      echo "Overwriting Docker image..."
      sudo docker commit $CONTAINER_NAME $IMAGE_NAME
      break
    elif [[ "${answer:0:1}" =~ n|N ]]; then
      break
    fi
  done
  docker stop $CONTAINER_NAME > /dev/null
  docker rm $CONTAINER_NAME > /dev/null
}

trap onexit EXIT

