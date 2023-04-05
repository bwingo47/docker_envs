#!/usr/bin/env bash

# Specify docke user name
DOCKER_USR=bwingo47
# Specify name of container
CONTAINER_NAME=ocs2-container
# Specify name of image (repository)
IMAGE_NAME=$DOCKER_USR/$CONTAINER_NAME
# Specify docker workspace folder name to be mounted 
DOCKER_WS=docker_ws
DOCKER_WS_1=docker_ws_digit_sim
# Specify NVIDIA runtime usage. Set to false if NVIDIA runtime not installed
USE_NVIDIA_RUNTIME=true


# If the container is running stop it
if [ "$( docker container inspect -f '{{.State.Running}}' $CONTAINER_NAME )" == "true" ]; then
  docker container stop $CONTAINER_NAME
fi

# check if nvidia driver is installed, and surpress output
nvidia-smi 2> /dev/null
# check exit status should return 127 if command not found
NVIDIA_DRIVER_INSTALLED_EXIT_STATUS=$?

# echo $NVIDIA_DRIVER_INSTALLED_EXIT_STATUS

if [ $NVIDIA_DRIVER_INSTALLED_EXIT_STATUS == "0" ]
then
  echo "NVIDIA driver installed, proceed to get driver version"

  TMP_LINE="$(nvidia-smi | grep NVIDIA-SMI)"
  REGEX="Driver Version: ([0-9]{3})\."
  if [[ "$TMP_LINE" =~ $REGEX ]]
  then
    NVIDIA_DRIVER_VERSION="${BASH_REMATCH[1]}"
    echo "Nvidia driver version: ${NVIDIA_DRIVER_VERSION}"
  else
    echo "Error: Cannot find Nvidia driver version"
    exit 1
  fi
else
  echo "NVIDIA driver not installed, driver version not defined"
fi

## for graphics passthrough [not necessary]
# Connect the existing X11 socket on the local machine to the one in the docker
XSOCK=/tmp/.X11-unix
# Create a new socket and mounting it to the docker
XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]
then
  xauth_list=$(xauth nlist $DISPLAY | sed -e 's/^..../ffff/')
  if [ ! -z "$xauth_list" ]
  then
    echo $xauth_list | xauth -f $XAUTH nmerge -
  else
    touch $XAUTH
  fi
  chmod a+r $XAUTH
fi



if [ $NVIDIA_DRIVER_INSTALLED_EXIT_STATUS == "0" ]
then 
  echo "nvidia driver installed"

  if [ $USE_NVIDIA_RUNTIME = true ]
  then 
    echo "running containter with nvidia runtime"

    docker run \
          -d \
          --name $CONTAINER_NAME \
          --hostname localhost \
          --ipc=host \
          --net=host \
          -it \
          --rm \
          -e DISPLAY=$DISPLAY \
          -e XAUTHORITY=$XAUTH \
          -e QT_X11_NO_MITSHM=1 \
          -v $XSOCK:$XSOCK:rw \
          -v $XAUTH:$XAUTH \
          -v $HOME/$DOCKER_WS:/root/$DOCKER_WS \
          -v $HOME/$DOCKER_WS_1:/root/$DOCKER_WS_1 \
          -v $HOME/.ssh:/root/.ssh \
          -v $HOME/.drake_gdb:/root/.drake_gdb \
          --gpus all \
          --privileged \
          --runtime=nvidia \
          --security-opt seccomp=unconfined \
          $IMAGE_NAME
  else 
    echo "nvidia runtime not available"

    docker run \
          -d \
          --name $CONTAINER_NAME \
          --hostname localhost \
          --ipc=host \
          --net=host \
          -it \
          --rm \
          -e DISPLAY=$DISPLAY \
          -e XAUTHORITY=$XAUTH \
          -e QT_X11_NO_MITSHM=1 \
          -v $XSOCK:$XSOCK:rw \
          -v $XAUTH:$XAUTH \
          -v $HOME/$DOCKER_WS:/root/$DOCKER_WS \
          -v $HOME/$DOCKER_WS_1:/root/$DOCKER_WS_1 \
          -v $HOME/.ssh:/root/.ssh \
          -v $HOME/.drake_gdb:/root/.drake_gdb \
          --privileged \
          --security-opt seccomp=unconfined \
          $IMAGE_NAME
  fi

else 
  echo "nvidia driver not available"

  docker run \
        -d \
        --name $CONTAINER_NAME \
        --hostname localhost \
        --ipc=host \
        --net=host \
        -it \
        --rm \
        -e DISPLAY=$DISPLAY \
        -e XAUTHORITY=$XAUTH \
        -e QT_X11_NO_MITSHM=1 \
        -v $XSOCK:$XSOCK:rw \
        -v $XAUTH:$XAUTH \
        -v $HOME/$DOCKER_WS:/root/$DOCKER_WS \
        -v $HOME/$DOCKER_WS_1:/root/$DOCKER_WS_1 \
        -v $HOME/.ssh:/root/.ssh \
        -v $HOME/.drake_gdb:/root/.drake_gdb \
        --privileged \
        --security-opt seccomp=unconfined \
        $IMAGE_NAME
fi

ssh-keygen -f "$HOME/.ssh/known_hosts" -R "[localhost]:7777"

ssh root@localhost -X -p 7777