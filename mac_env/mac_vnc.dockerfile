ARG BASE_IMAGE_NAME=ubuntu:20.04
FROM $BASE_IMAGE_NAME AS base
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
FROM base AS remote
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
# RUN sed -i 's/Port 22/Port 7777/' /etc/ssh/sshd_config

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


#### REMOTE with VNC ####
## can only ssh into REMOTE_USR for now, 
## ssh into root required additional configuration
FROM remote AS remote_vnc
LABEL maintainer="Bruce Wingo" \
      description="Added vnc x11 rendering to REMOTE." \
      version="0.0.1"

ARG STARTUP_FOLDER

RUN apt update \
    && apt install -y --no-install-recommends --allow-unauthenticated \
        xvfb \
        x11vnc \
        firefox \ 
        ttf-ubuntu-font-family \
        ttf-wqy-zenhei \
        fluxbox \
        net-tools \
        novnc \
        supervisor \
        xterm \
    && apt autoclean -y \
    && apt autoremove -y \
    && rm -rf /var/lib/apt/lists/*

# Setup visual environment variables
ENV LC_ALL=C.UTF-8 \
    DISPLAY=:0.0 \
    DISPLAY_WIDTH=1024 \
    DISPLAY_HEIGHT=768 \
    DISPLAY_DEPTh=32

RUN echo "export DISPLAY=$DISPLAY" >> /root/.profile
RUN echo "export DISPLAY_WIDTH=$DISPLAY_WIDTH" >> /root/.profile
RUN echo "export DISPLAY_HEIGHT=$DISPLAY_HEIGHT" >> /root/.profile
RUN echo "export DISPLAY_DEPTh=$DISPLAY_DEPTh" >> /root/.profile

USER root
CMD ["/root/startup/entrypoint.sh"]
# EXPOSE 8080

#### REMOTE with ROS ####
FROM remote_vnc AS remote_ros
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

# source ros setup in $REMOTE_USR .bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> $home/.bashrc
# RUN mkdir -p ~/catkin_ws/src

# source ros setup in root .bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc

# source ros setup in root .profile
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.profile

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
ENTRYPOINT ["/root/startup/entrypoint.sh"]


#### REMOTE with ROS, and Optimization Solvers ####
FROM remote_ros AS remote_ros_solvers
LABEL maintainer="Bruce Wingo" \
      description="Upgrades the remote ROS environment to have solver dependencies" \
      version="0.0.1"

## Install APT dependencies packages
RUN apt update --fix-missing
RUN apt upgrade -y
# RUN apt-get install -y --no-install-recommends \
#     gfortran \
# 	liblapack-dev \
#     liblapack64-dev \
#     libopenblas-dev \
#     swig \
#     patch \
#     libmetis-dev

RUN apt-get install -y --no-install-recommends \
    gfortran \
    libblas3 \
    libblas-dev \
	liblapack-dev \
    liblapack3 \
    swig \
    patch \
    libmetis-dev

## Install IPOPT from source follow instructions here: https://coin-or.github.io/Ipopt/INSTALL.html

# Install HSL. coinhsl is a private repo due to licenscing, please request hsl access 
# from official website: https://www.hsl.rl.ac.uk/ipopt/ and create ur own private repo
RUN cd /root &&\ 
    git clone --recursive https://github.com/bwingo47/ThirdParty-HSL.git


ARG SSH_PRV_KEY
ARG SSH_PUB_KEY
ARG NUM_MAKE_CORES
ARG num_cores=$NUM_MAKE_CORES
RUN echo "number of cmake cores to use: $num_cores"

RUN mkdir -p /root/.ssh &&\
    echo "$SSH_PRV_KEY" > /root/.ssh/id_ed25519 &&\
    echo "$SSH_PUB_KEY" > /root/.ssh/id_ed25519.pub &&\
    chmod 600 /root/.ssh/id_ed25519 &&\
    chmod 600 /root/.ssh/id_ed25519.pub &&\
    ssh-keyscan github.com > /root/.ssh/known_hosts


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


RUN cd /root/ThirdParty-HSL &&\
    git clone --recursive git@github.com:bwingo47/coinhsl.git &&\
    ./configure &&\
    make -j$NUM_MAKE_CORES &&\
    make install

# create symbolic link from coinhsl to hsl
RUN ln -s /usr/local/lib/libcoinhsl.so /usr/local/lib/libhsl.so

RUN rm -r /root/.ssh

# install IPOPT
RUN cd /root &&\
    git clone --recursive https://github.com/bwingo47/Ipopt.git &&\
    cd Ipopt &&\
    mkdir build && cd build &&\
    ./../configure ADD_FFLAGS=-fPIC ADD_CFLAGS=-fPIC ADD_CXXFLAGS=-fPIC &&\
    make -j$NUM_MAKE_CORES &&\
    make test &&\
    make install

# compiling parametric sensitivity support 
RUN cd /root/Ipopt/build/contrib/sIPOPT &&\
    make -j$NUM_MAKE_CORES &&\
    make install

# RUN apt-get install -y --no-install-recommends coinor-libipopt-dev


## Install CASADI from source (follow https://github.com/casadi/casadi/wiki/InstallationLinux)
RUN cd /root &&\
    git clone --recursive https://github.com/bwingo47/casadi.git &&\
    cd casadi && git checkout master &&\
    mkdir build && cd build &&\
    cmake -DWITH_PYTHON=ON -DWITH_COMMON=ON -DWITH_OPENMP=ON -DWITH_PYTHON3=ON -DWITH_HSL=ON -DWITH_THREAD=ON -DWITH_OSQP=ON .. &&\
    make -j$NUM_MAKE_CORES &&\
    make install
    # make doc


## Install CppAD from source (https://coin-or.github.io/CppAD/doc/cmake.htm)
RUN cd /root &&\
    git clone --recursive https://github.com/bwingo47/CppAD.git &&\
    cd CppAD && git checkout master &&\
    mkdir build && cd build &&\
    cmake -D include_ipopt=ON -D include_eigen=ON .. &&\
    make -j$NUM_MAKE_CORES &&\
    make install


## Install CppADCodeGen from source (https://github.com/joaoleal/CppADCodeGen)
RUN cd /root &&\
    git clone --recursive https://github.com/bwingo47/CppADCodeGen.git &&\
    cd CppADCodeGen && git checkout master &&\
    mkdir build && cd build &&\
    cmake .. &&\
    make -j$NUM_MAKE_CORES &&\
    make install

# ## Add all modified path to .profile so CLion can access these path variables
# #  This only works on session restart, need to find better solution
# #  Not sure if this actually works as CLion still can't see the ENV variables
# RUN echo "PATH=$PATH" >> /root/.profile
# RUN echo "PKG_CONFIG_PATH=$PKG_CONFIG_PATH" >> /root/.profile
# RUN echo "LD_LIBRARY_PATH=$LD_LIBRARY_PATH" >> /root/.profile
# RUN echo "PYTHONPATH=$PYTHONPATH" >> /root/.profile
# RUN echo "CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH" >> /root/.profile

# RUN touch /root/clion_env_vars.txt
# RUN echo "PATH=$PATH" >> /root/clion_env_vars.txt
# RUN echo "PKG_CONFIG_PATH=$PKG_CONFIG_PATH" >> /root/clion_env_vars.txt
# RUN echo "LD_LIBRARY_PATH=$LD_LIBRARY_PATH" >> /root/clion_env_vars.txt
# RUN echo "PYTHONPATH=$PYTHONPATH" >> /root/clion_env_vars.txt
# RUN echo "CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH" >> /root/clion_env_vars.txt

# # Add path variables to /etc/environment for all shells 
# RUN echo "PATH=$PATH" >> /etc/environment
# RUN echo "PKG_CONFIG_PATH=$PKG_CONFIG_PATH" >> /etc/environment
# RUN echo "LD_LIBRARY_PATH=$LD_LIBRARY_PATH" >> /etc/environment
# RUN echo "PYTHONPATH=$PYTHONPATH" >> /etc/environment
# RUN echo "CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH" >> /etc/environment



USER root
CMD ["/usr/sbin/sshd", "-D", "-p", "7777"]


#### REMOTE with ROS and OCS2 and full dependencies ####
FROM remote_ros_solvers AS remote_ros_gepetto
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
    make &&\
    make install

# ARG path=/usr/local/bin:$PATH
# ARG pkg_config_path=/usr/local/lib/pkgconfig:$PKG_CONFIG_PATH
# ARG ld_library_path=/usr/local/lib:$LD_LIBRARY_PATH
# ARG pythonpath=/usr/local/lib/python3.8/dist-packages:$PYTHONPATH
# ARG cmake_prefix_path=/usr/local:$CMAKE_PREFIX_PATH

# ENV PATH=$path
# ENV PKG_CONFIG_PATH=$pkg_config_path
# ENV LD_LIBRARY_PATH=$ld_library_path
# ENV PYTHONPATH=$pythonpath
# ENV CMAKE_PREFIX_PATH=$cmake_prefix_path

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
    make &&\
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
    cmake .. -D CMAKE_BUILD_TYPE=Release \
             -D CMAKE_INSTALL_PREFIX=/usr/local \
             -D BUILD_WITH_COLLISION_SUPPORT=ON \
             -D BUILD_WITH_CASADI_SUPPORT=ON \
             -D BUILD_WITH_AUTODIFF_SUPPORT=ON \
             -D BUILD_WITH_CODEGEN_SUPPORT=ON \
             -D BUILD_WITH_OPENMP_SUPPORT=ON &&\
    make &&\
    make install

# RUN mv /usr/lib/aarch64-linux-gnu/libOpenGL.so.0.0.0 /usr/lib/aarch64-linux-gnu/libOpenGL.so.0.0.0.orig &&\
#     ln -s /usr/lib/aarch64-linux-gnu/libGL.so.1 /usr/lib/aarch64-linux-gnu/libOpenGL.so.0.0.0

# RUN apt-get update \
#   && apt-get install -y --no-install-recommends \
#      libgl1-mesa-glx \
#      firefox

# # 
# RUN echo "LIBGL_ALWAYS_INDIRECT=1" >> $home/.bashrc
# RUN echo "LIBGL_ALWAYS_INDIRECT=1" >> /root/.bashrc
# RUN echo "LIBGL_ALWAYS_INDIRECT=1" >> /root/.profile


USER root
CMD ["/usr/sbin/sshd", "-D", "-p", "7777"]



#### REMOTE with ROS and OCS2 and full dependencies ####
FROM remote_ros_gepetto AS remote_ros_ocs2
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

# source catkin_ws setup in $REMOTE_USR .bashrc
RUN echo "source /root/catkin_ws/devel/setup.bash" >> $home/.bashrc

# source catkin_ws setup in root .bashrc
RUN echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc

# source catkin_ws setup in root .profile
RUN echo "source /root/catkin_ws/devel/setup.bash" >> /root/.profile

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


## Test install OCS2
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

# source docker_ws setup in $REMOTE_USR .bashrc
RUN echo "source /root/docker_ws/devel/setup.bash" >> $home/.bashrc

# source docker_ws setup in root .bashrc
RUN echo "source /root/docker_ws/devel/setup.bash" >> /root/.bashrc

# source docker_ws setup in root .profile
RUN echo "source /root/docker_ws/devel/setup.bash" >> /root/.profile

# configure gpd eigen pretty printers 
RUN touch /root/.gdbinit
RUN echo "python" >> /root/.gdbinit
RUN echo "import sys" >> /root/.gdbinit
RUN echo "sys.path.insert(0, \"/root/.drake_gdb\")" >> /root/.gdbinit
RUN echo "import drake_gdb" >> /root/.gdbinit
RUN echo "drake_gdb.register_printers()" >> /root/.gdbinit
RUN echo "end" >> /root/.gdbinit





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
FROM remote AS remote_nvidia
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
FROM remote_nvidia AS remote_nvidia_ros
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


