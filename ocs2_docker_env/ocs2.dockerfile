ARG BASE_IMAGE_NAME=ubuntu:20.04
FROM $BASE_IMAGE_NAME AS BASE
LABEL maintainer="Bruce Wingo" \
      description="base image setup" \
	  version="0.0.1"

SHELL ["/bin/bash", "-c"]

ENV TZ=America/New_York
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

ENV DEBIAN_FRONTEND noninteractive

RUN apt update --fix-missing \
  && apt-get install -y --no-install-recommends \
    apt-utils \
  && apt-get install -y --no-install-recommends \
    keyboard-configuration \
    tzdata

## Install APT packages
RUN apt update --fix-missing
RUN apt upgrade -y
RUN apt-get install -y --no-install-recommends \
    build-essential \
	cmake \
    g++-9 \
    gcc-9 \
    g++-10 \
    gcc-10 \
    clang-format-10 \
    clang-tidy-10 \
    clang-format-12 \
    clang-tidy-12 \
	curl \
	doxygen \
    doxygen-latex \
	libeigen3-dev \
	gdb \
    gdbserver \
	git \
    git-lfs \
    gnupg \
    lcov \
    libboost-all-dev \
	libglu1-mesa-dev \
    libopencv-dev \
    libtbb-dev \
    lsb-core \
	python3 \
    python3-tk \
    python3-apt \
    python3-distutils \
    python3-opencv \
    python3-pip \
    software-properties-common \
	tmux \
    wget \
    xorg-dev
# Install what you want (remember the ' \')

## Setup alternatives python 
RUN update-alternatives --install /usr/bin/python python /usr/bin/python3 3
# RUN update-alternatives --install /usr/bin/python python /usr/bin/python2 2

## Setup alternatives C++
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 9
RUN update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-9 9
RUN update-alternatives --install /usr/bin/clang-format clang-format /usr/bin/clang-format-10 10
RUN update-alternatives --install /usr/bin/clang-tidy clang-tidy /usr/bin/clang-tidy-10 10

# C++ 20
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-10 10
RUN update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-10 10
RUN update-alternatives --install /usr/bin/clang-format clang-format /usr/bin/clang-format-12 12
RUN update-alternatives --install /usr/bin/clang-tidy clang-tidy /usr/bin/clang-tidy-12 12

## Install pip packages
RUN pip3 install \
    Cheetah3 \
    gcovr \
    gitpython \
    jupyter \
    keras \
    matplotlib \
    networkx \
    numpy==1.20 \
    pandas \
    pybind11 \
    scipy \
    tensorflow \
    torch \
    torchaudio \
    torchvision
# Install what you want (remember the ' \')

## Clean up
RUN apt autoclean
RUN apt autoremove
RUN apt clean
RUN rm -rf /var/lib/apt/lists/*

## Fix eigen
RUN ln -s /usr/include/eigen3/Eigen /usr/include/Eigen

## Install Google Test

## Install Google Benchmark

## Install nlohmann json 

## Install {fmt}

## Install spdlog

## Configure system-wide git user 
ARG GIT_LOGIN_EMAIL
RUN git config --system user.email "$GIT_LOGIN_EMAIL"
# RUN echo "git login email set to: $GIT_LOGIN_EMAIL"

## Configure default file creation permissions 
#  (RUN umask in dockerfile doesn't work)
# RUN umask u=rwx,g=rwx,o=rwx
RUN echo "umask u=rwx,g=rwx,o=rwx" >> /root/.profile

## Bring up bash 
CMD ["bash"]


#### REMOTE ####
## can only ssh into REMOTE_USR for now, 
## ssh into root required additional configuration
FROM BASE AS REMOTE
LABEL maintainer="Bruce Wingo" \
      description="Added remote access functionality to BASE. \ 
                   can only ssh into REMOTE_USR for now, \
                   ssh into root required additional configuration" \
      version="0.0.1"

ARG REMOTE_USR=remote_usr
ARG home=/home/$REMOTE_USR

RUN apt update --fix-missing \
  && apt-get install -y --no-install-recommends \
     dbus-x11 \
     openssh-server \
     gosu \
     mesa-utils \
     rsync \
     vim \
     xauth
# Install what you want (remember the ' \')     

## Check that gosu works for the entrypoint
RUN gosu nobody true

## Clean up
RUN apt autoclean
RUN apt autoremove
RUN apt clean
RUN rm -rf /var/lib/apt/lists/*

## Config sshd
RUN mkdir /var/run/sshd
# change password for root to "pwd"
RUN echo 'root:pwd' | chpasswd
# set login permissions 
RUN sed -i 's/PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config
# enable rootlogin
RUN sed -i 's/#PermitRootLogin yes/PermitRootLogin yes/' /etc/ssh/sshd_config
RUN sed -i 's/#AddressFamily any/AddressFamily inet/' /etc/ssh/sshd_config
# ssh login fix. Otherwise user is kicked off after login
RUN sed 's@session\s*required\s*pam_loginuid.so@session optional pam_loginuid.so@g' -i /etc/pam.d/sshd
# allow environment vars to be passed to dockerized sshd service, 
# see https://stackoverflow.com/questions/36292317/why-set-visible-now-in-etc-profile
ENV NOTVISIBLE "in users profile"
RUN echo "export VISIBLE=now" >> /etc/profile
# expose port 22 for ssh server, and 7777 for gdb server
RUN sed -i 's/Port 22/Port 7777/' /etc/ssh/sshd_config

# create container user $REMOTE_USR and set default shell
RUN useradd -ms /bin/bash $REMOTE_USR
# change password for $REMOTE_USR to "pwd"
RUN echo $REMOTE_USR:pwd | chpasswd

USER $REMOTE_USR
ENV HOME $home

USER root
# run sshd in detached mode "-D", and open port 7777
# !!!! to run the container on the same network as the local machine,
# use '--net=host' option for docker run,
# also set '--hostname localhost' and '--ipc=host'
# if both container and local machine runs on the same network, 
# then the follong CMD instuction will open the port 7777 on localhost of the local machine
# and there is no need to map ports using the -p flag of docker run.
# i.e. 'ssh remote_usr@localhost -p 7777' will ssh into the container
# otherwise '-p 127.0.0.1:[localhost_port_number]:7777' is needed when docker run to map ports
# for this, use 'ssh remote_usr@localhost -p [localhost_port_number]' to remote into container
CMD ["/usr/sbin/sshd", "-D", "-p", "7777"]


#### REMOTE with ROS ####
FROM REMOTE AS REMOTE_ROS
LABEL maintainer="Bruce Wingo" \
      description="Upgrades the remote environment to have full ROS desktop" \
      version="0.0.1"

## Install ROS
ARG REMOTE_USR=remote_usr
ARG home=/home/$REMOTE_USR
ARG ROS_DISTRO=noetic
RUN echo "deb [trusted=yes] http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt update --fix-missing
RUN apt-get install -y --no-install-recommends \
	ros-${ROS_DISTRO}-desktop-full
# Install what you want (remember the '\')

RUN apt-get install -y --no-install-recommends \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool

RUN rosdep init

# clean up
RUN apt autoclean
RUN apt autoremove
RUN apt clean
RUN rm -rf /var/lib/apt/lists/*


# USER $REMOTE_USR
ENV HOME $home
RUN rosdep update
ENV ROS_DISTRO $ROS_DISTRO

# update path for $REMOTE_USR
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> $home/.bashrc
# RUN mkdir -p ~/catkin_ws/src

# update path for root
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc

ARG path=/opt/ros/$ROS_DISTRO/bin:$PATH
ARG pkg_config_path=/opt/ros/$ROS_DISTRO/lib/pkgconfig:$PKG_CONFIG_PATH
ARG ld_library_path=/opt/ros/$ROS_DISTRO/lib:$LD_LIBRARY_PATH
ARG pythonpath=/opt/ros/$ROS_DISTRO/lib/python3/dist-packages:$PYTHONPATH
ARG cmake_prefix_path=/opt/ros/$ROS_DISTRO:$CMAKE_PREFIX_PATH

ENV PATH=$path
ENV PKG_CONFIG_PATH=$pkg_config_path
ENV LD_LIBRARY_PATH=$ld_library_path
ENV PYTHONPATH=$pythonpath
ENV CMAKE_PREFIX_PATH=$cmake_prefix_path

# RUN source /opt/ros/${ROS_DISTRO}/setup.bash &&\
#     cd ~/catkin_ws &&\
#     catkin_make &&\
#     source devel/setup.bash
# RUN echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc


# ARG ld_library_path=$home/catkin_ws/devel/lib:$LD_LIBRARY_PATH
# ARG cmake_prefix_path=$home/catkin_ws/devel:$CMAKE_PREFIX_PATH

# ENV LD_LIBRARY_PATH=$ld_library_path
# ENV CMAKE_PREFIX_PATH=$cmake_prefix_path

# RUN roscore &

# ENV environment-variable-name environment-variable-value

USER root
CMD ["/usr/sbin/sshd", "-D", "-p", "7777"]


#### REMOTE with ROS and OCS2 and full dependencies ####
FROM REMOTE_ROS AS REMOTE_ROS_GEPETTO
LABEL maintainer="Bruce Wingo" \
      description="Upgrades the remote ROS environment to have Gepetto team packages" \
      version="0.0.1"


ARG REMOTE_USR=remote_usr
ARG home=/home/$REMOTE_USR

## Install catkin-tools and pybind11-catkin
RUN sh \
    -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" \
        > /etc/apt/sources.list.d/ros-latest.list'
RUN wget http://packages.ros.org/ros.key -O - | apt-key add -
RUN apt-get update \
  && apt-get install -y --no-install-recommends \
     python3-catkin-tools \
     ros-$ROS_DISTRO-pybind11-catkin

## Install optional dependencies
RUN apt update --fix-missing \
  && apt-get install -y --no-install-recommends \
     libglpk-dev \
     liburdfdom-dev \
     liboctomap-dev \
     libassimp-dev \
     ros-$ROS_DISTRO-rqt-multiplot \
     ros-$ROS_DISTRO-grid-map-msgs

## Install robotpkg binaries 
# install binaries from robotpkg
# RUN sh -c "echo 'deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub \
#  $(lsb_release -cs) robotpkg' >> /etc/apt/sources.list.d/robotpkg.list"
# RUN curl http://robotpkg.openrobots.org/packages/debian/robotpkg.key | apt-key add -
# RUN apt-get update \
#   && apt-get install -y --no-install-recommends \
#      robotpkg-py38-eigenpy
#     #  robotpkg-hpp-fcl

## Install eigenpy from source 
RUN cd /root &&\
    git clone --recursive https://github.com/bwingo47/eigenpy.git
    # git clone --recursive https://github.com/stack-of-tasks/eigenpy.git

RUN cd /root/eigenpy &&\
    # git checkout f53d37e0f732361f338769b730a129e2da9b6a46 &&\
    git checkout master &&\
    mkdir build &&\
    cd build &&\
    cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local &&\
    make -j12 &&\
    make install

ARG path=/usr/local/bin:$PATH
ARG pkg_config_path=/usr/local/lib/pkgconfig:$PKG_CONFIG_PATH
ARG ld_library_path=/usr/local/lib:$LD_LIBRARY_PATH
ARG pythonpath=/usr/local/lib/python3.8/dist-packages:$PYTHONPATH
ARG cmake_prefix_path=/usr/local:$CMAKE_PREFIX_PATH

ENV PATH=$path
ENV PKG_CONFIG_PATH=$pkg_config_path
ENV LD_LIBRARY_PATH=$ld_library_path
ENV PYTHONPATH=$pythonpath
ENV CMAKE_PREFIX_PATH=$cmake_prefix_path

## Install hpp-fcl from source
RUN cd /root &&\
    git clone --recursive https://github.com/bwingo47/hpp-fcl.git
    # git clone --recursive https://github.com/humanoid-path-planner/hpp-fcl.git 

RUN cd /root/hpp-fcl &&\
    # git checkout c8373f933ec0fe9fbded6f8d1235f6fc08845ada &&\
    git checkout devel &&\
    mkdir build &&\
    cd build &&\
    cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local &&\
    make -j12 &&\
    make install


## Install pinocchio from source
RUN cd /root &&\ 
    git clone --recursive https://github.com/bwingo47/pinocchio.git
    # git clone --recursive https://github.com/stack-of-tasks/pinocchio

# must using turn on BUILD_WITH_COLLISION_SUPPORT option during cmake .. 
# otherwise ocs2_pinocchio (specifically ocs2_self_collision) won't build
RUN cd /root/pinocchio &&\
    # git checkout 5c93f4e043886ec43659a10a79701263a1e8fa18 &&\
    git checkout master &&\
    mkdir build &&\
    cd build &&\
    cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local -D BUILD_WITH_COLLISION_SUPPORT=ON &&\
    make -j12 &&\
    make install




USER root
CMD ["/usr/sbin/sshd", "-D", "-p", "7777"]


#### REMOTE with ROS and OCS2 and full dependencies ####
FROM REMOTE_ROS_GEPETTO AS REMOTE_ROS_OCS2
LABEL maintainer="Bruce Wingo" \
      description="Upgrades the remote ROS Gepetto environment to have full OCS2" \
      version="0.0.1"


ARG REMOTE_USR=remote_usr
ARG home=/home/$REMOTE_USR
# RUN echo "cmake path is: $CMAKE_PREFIX_PATH"

## Install gnome-terminal, ocs2 roslaunch files use this to input commands 
RUN apt update --fix-missing \
  && apt-get install -y --no-install-recommends \
    gnome-terminal

## Make and initialize catkin_ws
RUN mkdir -p /root/catkin_ws/src

RUN cd /root/catkin_ws &&\
    catkin init &&\
    catkin config --extend /opt/ros/$ROS_DISTRO &&\
    catkin config -DCMAKE_BUILD_TYPE=RelWithDebInfo &&\
    catkin build

# export catkin_ws path to $REMOTE_USR .bashrc
RUN echo "source /root/catkin_ws/devel/setup.bash" >> $home/.bashrc

# export catkin_ws path to root .bashrc
RUN echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc

# update catkin_ws path variables
ARG pkg_config_path=/root/catkin_ws/devel/lib/pkgconfig:$PKG_CONFIG_PATH
ARG ld_library_path=/root/catkin_ws/devel/lib:$LD_LIBRARY_PATH
ARG pythonpath=/usr/local/lib/python3/dist-packages:$PYTHONPATH
ARG cmake_prefix_path=/root/catkin_ws/devel:$CMAKE_PREFIX_PATH

ENV PKG_CONFIG_PATH=$pkg_config_path
ENV LD_LIBRARY_PATH=$ld_library_path
ENV PYTHONPATH=$pythonpath
ENV CMAKE_PREFIX_PATH=$cmake_prefix_path

# RUN echo $PATH
# RUN echo $PKG_CONFIG_PATH
# RUN echo $LD_LIBRARY_PATH
# RUN echo $PYTHONPATH
# RUN echo $CMAKE_PREFIX_PATH

## Install Raisim


## Install OCS2
# RUN cd /root/catkin_ws/src &&\
#     git clone https://github.com/bwingo47/ocs2.git &&\
#     git clone https://github.com/bwingo47/ocs2_robotic_assets.git &&\
#     cd .. &&\
#     catkin build ocs2_robotic_assets &&\
#     catkin build ocs2_legged_robot_ros
#     # catkin build ocs2

## Grant $REMOTE_USR root access
# RUN usermod -aG root $REMOTE_USR



# update docker_ws path variables
ARG DOCKER_WS
ARG pkg_config_path=/root/$DOCKER_WS/devel/lib/pkgconfig:$PKG_CONFIG_PATH
ARG ld_library_path=/root/$DOCKER_WS/devel/lib:$LD_LIBRARY_PATH
ARG cmake_prefix_path=/root/$DOCKER_WS/devel:$CMAKE_PREFIX_PATH

ENV PKG_CONFIG_PATH=$pkg_config_path
ENV LD_LIBRARY_PATH=$ld_library_path
ENV CMAKE_PREFIX_PATH=$cmake_prefix_path

## Add all modified path to .profile so CLion can access these path variables
#  This only works on session restart, need to find better solution
#  Not sure if this actually works as CLion still can't see the ENV variables
RUN echo "PATH=$PATH" >> /root/.profile
RUN echo "PKG_CONFIG_PATH=$PKG_CONFIG_PATH" >> /root/.profile
RUN echo "LD_LIBRARY_PATH=$LD_LIBRARY_PATH" >> /root/.profile
RUN echo "PYTHONPATH=$PYTHONPATH" >> /root/.profile
RUN echo "CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH" >> /root/.profile

RUN touch /root/clion_env_vars.txt
RUN echo "PATH=$PATH" >> /root/clion_env_vars.txt
RUN echo "PKG_CONFIG_PATH=$PKG_CONFIG_PATH" >> /root/clion_env_vars.txt
RUN echo "LD_LIBRARY_PATH=$LD_LIBRARY_PATH" >> /root/clion_env_vars.txt
RUN echo "PYTHONPATH=$PYTHONPATH" >> /root/clion_env_vars.txt
RUN echo "CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH" >> /root/clion_env_vars.txt

# Add path variables to /etc/environment for all shells 
RUN echo "PATH=$PATH" >> /etc/environment
RUN echo "PKG_CONFIG_PATH=$PKG_CONFIG_PATH" >> /etc/environment
RUN echo "LD_LIBRARY_PATH=$LD_LIBRARY_PATH" >> /etc/environment
RUN echo "PYTHONPATH=$PYTHONPATH" >> /etc/environment
RUN echo "CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH" >> /etc/environment



## Create an entry point script for this
# RUN touch /root/start.sh
# RUN chmod +x /root/start.sh
# RUN echo "#!/usr/bin/env bash" >> /root/start.sh
# RUN echo "/usr/sbin/sshd -D -p 7777" >> /root/start.sh





USER root
# ENTRYPOINT ["/root/start.sh"]
CMD ["/usr/sbin/sshd", "-D", "-p", "7777"]



#### REMOTE with NVIDIA Driver ####
## fist, need to install nvidia-container-runtime on local host
## follow https://github.com/nvidia/nvidia-container-runtime#docker-engine-setup for your OS
## then follow the 'Docker Engine setup' section to setup nvidia runtime for docker
FROM REMOTE AS REMOTE_NVIDIA
LABEL maintainer="Bruce Wingo" \
      description="Upgrades the remote environment to have nvidia driver" \
      version="0.0.1"

ARG REMOTE_USR=remote_usr
ARG NVIDIA_DRIVER_VERSION

## Install Nvidia Drivers
RUN add-apt-repository ppa:graphics-drivers
RUN apt update --fix-missing
RUN apt install -y --no-install-recommends nvidia-driver-$NVIDIA_DRIVER_VERSION

# test gui passthrough using firefox
# RUN apt-get install -y --no-install-recommends firefox

RUN apt autoclean
RUN apt autoremove
RUN apt clean
RUN rm -rf /var/lib/apt/lists/*

USER root
CMD ["/usr/sbin/sshd", "-D", "-p", "7777"]



#### REMOTE with NVIDIA Driver and ROS ####
FROM REMOTE_NVIDIA AS REMOTE_NVIDIA_ROS
LABEL maintainer="Bruce Wingo" \
      description="Upgrades the remote environment to have nvidia driver and full ROS desktop" \
      version="0.0.1"

## Install ROS
ARG REMOTE_USR=remote_usr
ARG home=/home/$REMOTE_USR
ARG ROS_DISTRO=noetic
RUN echo "deb [trusted=yes] http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt update --fix-missing
RUN apt-get install -y --no-install-recommends \
	ros-${ROS_DISTRO}-desktop-full
# Install what you want (remember the '\')

RUN apt-get install -y --no-install-recommends \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool

RUN rosdep init

# clean up
RUN apt autoclean
RUN apt autoremove
RUN apt clean
RUN rm -rf /var/lib/apt/lists/*


USER $REMOTE_USR
ENV HOME $home
RUN rosdep update
ENV ROS_DISTRO $ROS_DISTRO
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN mkdir -p ~/catkin_ws/src

RUN source /opt/ros/${ROS_DISTRO}/setup.bash &&\
    cd ~/catkin_ws &&\
    catkin_make &&\
    source devel/setup.bash
RUN echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

# RUN roscore &

# ENV environment-variable-name environment-variable-value

USER root
CMD ["/usr/sbin/sshd", "-D", "-p", "7777"]


# Install clion somehow


