#!/bin/bash

# Configurable variables
DOCKER_IMAGE=599631481fe0

WEB_PORT=10334
SSH_PORT=7901
CONTAINER_NAME=discower_GUI
VNC_PASSWORD=password
# NOETIC_BASE_DIR=${HOME}/workspace/ros_noetic

# Check if a container with the same name already exists
if [ $(docker ps -aq -f name=^${CONTAINER_NAME}$) ]; then
    echo "A container with the name $CONTAINER_NAME already exists."
    # Choose what to do here: stop/remove the existing container, or exit
    # docker stop $CONTAINER_NAME
    # docker rm $CONTAINER_NAME
    # or
    exit 1
fi

# Run the Docker container
docker run -itd \
      --shm-size 32g \
      --runtime=nvidia \
      --gpus all\
      --privileged \
      -p $WEB_PORT:6901 \
      -p $SSH_PORT:22 \
      --security-opt seccomp=unconfined \
      -e VNC_PW=$VNC_PASSWORD \
      -e NVIDIA_DRIVER_CAPABILITIES=all \
      --name $CONTAINER_NAME \
      $DOCKER_IMAGE
      #--volume="${NOETIC_BASE_DIR}:/home/kasm-user/workspace:Z" \

# Check if Docker run was successful
if [ $? -eq 0 ]; then
    echo "Container $CONTAINER_NAME started successfully."
else
    echo "Failed to start the container."
    exit 1
fi
