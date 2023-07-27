#!/usr/bin/env bash

# Specify docke user name
DOCKER_USR=bwingo47
# Specify name of container
CONTAINER_NAME=mac-pinocchio-container
# Specify name of image (repository)
IMAGE_NAME=$DOCKER_USR/$CONTAINER_NAME
# Specify name of docker file to build and run
DOCKERFILE_NAME=mac_vnc_pinocchio.dockerfile
# Specify build stage name for multi-target dockerfile
DOCKERFILE_BUILD_STAGE=pinocchio3_dev
# Specify docker workspace folder name to be mounted 
DOCKER_WS=docker_pinocchio3_ws
# Specify github login name
GIT_LOGIN_EMAIL=wingobruce47@gmail.com
# Specify the number of CPU cores to run cmake
NUM_MAKE_CORES=4
# Specify NVIDIA runtime usage. Set to false if NVIDIA runtime not installed
USE_NVIDIA_RUNTIME=true
# Specify localhost name
LOCALHOSTNAME=localhost
# Startup folder name 
STARTUP_FOLDER=startup
# GDB port
GDB_SSH_PORT=7780
# X11 websocket port
X11_WEBSOCKET_PORT=8084
# Display
DISPLAY=:0.0
# Display width, height, depth
DISPLAY_WIDTH=1335
DISPLAY_HEIGHT=735
DISPLAY_DEPTH=24

# If the container is running stop it
if [ "$( docker container inspect -f '{{.State.Running}}' $CONTAINER_NAME )" == "true" ]; then
  docker container stop $CONTAINER_NAME
fi


CUSTOM_USER=remote_usr

# echo "Building docker image"
# docker build -f $DOCKERFILE_NAME \
#   --build-arg NVIDIA_DRIVER_VERSION="$NVIDIA_DRIVER_VERSION" \
#   --build-arg SSH_PRV_KEY="$(cat $HOME/.ssh/id_ed25519)" \
#   --build-arg SSH_PUB_KEY="$(cat $HOME/.ssh/id_ed25519.pub)" \
#   --build-arg GDB_SSH_PORT=$GDB_SSH_PORT \
#   --build-arg CUSTOM_USER=$CUSTOM_USER \
#   --build-arg GIT_LOGIN_EMAIL=$GIT_LOGIN_EMAIL \
#   --build-arg NUM_MAKE_CORES=$NUM_MAKE_CORES \
#   --build-arg DOCKER_WS=$DOCKER_WS \
#   --build-arg STARTUP_FOLDER=$STARTUP_FOLDER \
#   --build-arg DISPLAY=$DISPLAY \
#   --build-arg DISPLAY_WIDTH=$DISPLAY_WIDTH \
#   --build-arg DISPLAY_DEPTH=$DISPLAY_DEPTH \
#   --build-arg DISPLAY_HEIGHT=$DISPLAY_HEIGHT \
#   --build-arg X11_WEBSOCKET_PORT=$X11_WEBSOCKET_PORT \
#   --target $DOCKERFILE_BUILD_STAGE \
#   -t $IMAGE_NAME . || { echo "Build docker failed"; exit 1; }


echo "Running docker"

# Set up XQuartz https://gist.github.com/cschiewek/246a244ba23da8b9f0e7b11a68bf3285
# xhost +
# xhost + ${hostname}
# export HOSTNAME=`hostname`
# defaults write org.macosforge.xquartz.X11 enable_iglx -bool true
# defaults write org.xquartz.X11 enable_iglx -bool true


echo "Container spinning up"

docker run \
      -d \
      --name $CONTAINER_NAME \
      --hostname $LOCALHOSTNAME \
      --ipc=host \
      -e GDB_SSH_PORT=$GDB_SSH_PORT \
      -e X11_WEBSOCKET_PORT=$X11_WEBSOCKET_PORT \
      -e DISPLAY=$DISPLAY \
      -e DISPLAY_WIDTH=$DISPLAY_WIDTH \
      -e DISPLAY_HEIGHT=$DISPLAY_HEIGHT \
      -e DISPLAY_DEPTH=$DISPLAY_DEPTH \
      -p $GDB_SSH_PORT:$GDB_SSH_PORT \
      -p $X11_WEBSOCKET_PORT:$X11_WEBSOCKET_PORT \
      -it \
      --rm \
      -v $PWD/$STARTUP_FOLDER:/root/$STARTUP_FOLDER \
      -v $HOME/$DOCKER_WS:/root/$DOCKER_WS \
      -v $HOME/.ssh:/root/.ssh \
      -v $HOME/.drake_gdb:/root/.drake_gdb \
      --privileged \
      --security-opt seccomp=unconfined \
      $IMAGE_NAME 

if [ "$( docker container inspect -f '{{.State.Status}}' $CONTAINER_NAME )" == "running" ]; then
  echo "Container is up";
  sleep 2;
fi

/bin/bash $PWD/run_ssh.sh -p $GDB_SSH_PORT

# open http://localhost:8080/vnc.html
