#!/usr/bin/env bash

# Specify docke user name
DOCKER_USR=bwingo47
# Specify name of container
CONTAINER_NAME=mac-amd64-container
# Specify name of image (repository)
IMAGE_NAME=$DOCKER_USR/$CONTAINER_NAME
# Specify name of docker file to build and run
DOCKERFILE_NAME=mac_amd64.dockerfile
# Specify build stage name for multi-target dockerfile
DOCKERFILE_BUILD_STAGE=remote_ros_ocs2
# Specify docker workspace folder name to be mounted 
DOCKER_WS=docker_ws
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

# If the container is running stop it
if [ "$( docker container inspect -f '{{.State.Running}}' $CONTAINER_NAME )" == "true" ]; then
  docker container stop $CONTAINER_NAME
fi


CUSTOM_USER=remote_usr

echo "Building docker image"
docker buildx build -f $DOCKERFILE_NAME \
  --platform linux/amd64 \
  --build-arg NVIDIA_DRIVER_VERSION="$NVIDIA_DRIVER_VERSION" \
  --build-arg SSH_PRV_KEY="$(cat $HOME/.ssh/id_ed25519)" \
  --build-arg SSH_PUB_KEY="$(cat $HOME/.ssh/id_ed25519.pub)" \
  --build-arg CUSTOM_USER=$CUSTOM_USER \
  --build-arg GIT_LOGIN_EMAIL=$GIT_LOGIN_EMAIL \
  --build-arg NUM_MAKE_CORES=$NUM_MAKE_CORES \
  --build-arg DOCKER_WS=$DOCKER_WS \
  --build-arg STARTUP_FOLDER=$STARTUP_FOLDER \
  --target $DOCKERFILE_BUILD_STAGE \
  -t $IMAGE_NAME . || { echo "Build docker failed"; exit 1; }


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
      --platform linux/amd64 \
      --name $CONTAINER_NAME \
      --hostname $LOCALHOSTNAME \
      --ipc=host \
      -p 7777:7777 \
      -p 8080:8080 \
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
  echo "Container is up"
  sleep 4;
fi

/bin/bash $PWD/run_ssh.sh 

# open http://localhost:8080/vnc.html

# ssh-keygen -f "$HOME/.ssh/known_hosts" -R "[$LOCALHOSTNAME]:7777"

# ssh root@localhost -p 7777