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
  \trun.sh
  \trun.sh --image_name custom_image_name --container_name custom_container_name\n"
}

# Parse arguments
while [[ "$#" -gt 0 ]]; do
    case $1 in
        -i|--image_name) IMAGE_NAME="${2}"; shift ;;
        -c|--container_name) CONTAINER_NAME="${2}"; shift ;;
        -h|--help) show_help ; exit 1 ;;
        *) echo "Unknown parameter passed: $1"; exit 1 ;;
    esac
    shift
done

# auto detect nvidia
if [ -f /etc/nv_tegra_release ]; then
    echo "starting container on Nvidia Nano"
    NVIDIA_FLAGS="--runtime nvidia \
                  -e NVIDIA_VISIBLE_DEVICES=all \
                  -e NVIDIA_DRIVER_CAPABILITIES=all \
                  --group-add video "
fi

HOST_UID=$(id -u)
HOST_GID=$(id -g)

# Set default values if not provided
IMAGE_NAME=${IMAGE_NAME:-robot-humble-arm64}
CONTAINER_NAME=${CONTAINER_NAME:-robot_container}

CONTAINER_USER=ubuntu
HOST_USER=$USER

SSH_PATH=/home/$HOST_USER/.ssh
if [ -z "$SSH_AUTH_SOCK" ]; then
    echo "Warning: SSH_AUTH_SOCK not set, starting ssh-agent..."
    eval $(ssh-agent -s)
    ssh-add $SSH_PATH/id_ed25519  # or whatever your key is named
fi

WORKSPACE_HOST=/home/$HOST_USER/robot_ws
WORKSPACE_SRC_CONTAINER=/home/$CONTAINER_USER/robot_ws/src
WORKSPACE_ROOT_CONTAINER=/home/$CONTAINER_USER/robot_ws
SSH_AUTH_SOCK_CONTAINER_USER=$SSH_AUTH_SOCK


# Ensure cache folders exist
mkdir -p $WORKSPACE_HOST/.build
mkdir -p $WORKSPACE_HOST/.install

# Check if container name is already in use
if docker container ls -a | grep "${CONTAINER_NAME}$" -c &> /dev/null; then
   echo -e "Error: Docker container named $CONTAINER_NAME is already running.\n"
   echo -e "Try removing it with: \n\tdocker rm $CONTAINER_NAME"
   echo -e "Or use a different container name."
   exit 1
fi

# Start the container
xhost +
docker run --privileged --net=host -it $NVIDIA_FLAGS \
       -e DISPLAY=$DISPLAY \
       -e SSH_AUTH_SOCK=$SSH_AUTH_SOCK_CONTAINER_USER \
       -v $(dirname $SSH_AUTH_SOCK_CONTAINER_USER):$(dirname $SSH_AUTH_SOCK_CONTAINER_USER) \
       -v $SSH_PATH:/home/$CONTAINER_USER/.ssh:ro \
       -v /tmp/.X11-unix:/tmp/.X11-unix \
       -v $WORKSPACE_HOST:$WORKSPACE_ROOT_CONTAINER:rw \
       -v $WORKSPACE_HOST/.build:$WORKSPACE_ROOT_CONTAINER/build:rw \
       -v $WORKSPACE_HOST/.install:$WORKSPACE_ROOT_CONTAINER/install:rw \
       -v /dev:/dev \
       -u $HOST_UID:$HOST_GID \
       --name $CONTAINER_NAME $IMAGE_NAME
xhost -

# Cleanup function to commit changes if needed
function onexit() {
  while true; do
    read -p "Do you want to overwrite the image '$IMAGE_NAME' with current changes? [y/n]: " answer
    if [[ "${answer:0:1}" =~ y|Y ]]; then
      echo "Overwriting Docker image..."
      docker commit $CONTAINER_NAME $IMAGE_NAME
      break
    elif [[ "${answer:0:1}" =~ n|N ]]; then
      break
    fi
  done
  docker stop $CONTAINER_NAME > /dev/null
  docker rm $CONTAINER_NAME > /dev/null
}

trap onexit EXIT

