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

# configure gpd eigen pretty printers 
RUN touch /root/.gdbinit
RUN echo "python" >> /root/.gdbinit
RUN echo "import sys" >> /root/.gdbinit
RUN echo "sys.path.insert(0, \"/root/.drake_gdb\")" >> /root/.gdbinit
RUN echo "import drake_gdb" >> /root/.gdbinit
RUN echo "drake_gdb.register_printers()" >> /root/.gdbinit
RUN echo "end" >> /root/.gdbinit

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

ARG EXPOSED_PORT
ENV EXPOSED_PORT $EXPOSED_PORT
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
# expose port 22 for ssh server, and $EXPOSED_PORT for gdb server
RUN sed -i 's/Port 22/Port $EXPOSED_PORT/' /etc/ssh/sshd_config

# create container user $REMOTE_USR and set default shell
RUN useradd -ms /bin/bash $REMOTE_USR
# change password for $REMOTE_USR to "pwd"
RUN echo $REMOTE_USR:pwd | chpasswd

USER $REMOTE_USR
ENV HOME $home

USER root
# run sshd in detached mode "-D", and open port $EXPOSED_PORT
# !!!! to run the container on the same network as the local machine,
# use '--net=host' option for docker run,
# also set '--hostname localhost' and '--ipc=host'
# if both container and local machine runs on the same network, 
# then the follong CMD instuction will open the port $EXPOSED_PORT on localhost of the local machine
# and there is no need to map ports using the -p flag of docker run.
# i.e. 'ssh remote_usr@localhost -p $EXPOSED_PORT' will ssh into the container
# otherwise '-p 127.0.0.1:[localhost_port_number]:$EXPOSED_PORT' is needed when docker run to map ports
# for this, use 'ssh remote_usr@localhost -p [localhost_port_number]' to remote into container
CMD ["/usr/sbin/sshd", "-D", "-p", "7779"]


#### REMOTE with steamCMD ####
FROM REMOTE AS REMOTE_STEAMCMD
LABEL maintainer="Bruce Wingo" \
      description="Upgrades the remote environment to have full ROS desktop" \
      version="0.0.1"

## Install steamCMD
RUN apt update --fix-missing \
  && apt-get install -y --no-install-recommends 

RUN echo steam steam/question select "I AGREE" | debconf-set-selections \
 && echo steam steam/license note '' | debconf-set-selections

# Update the repository and install SteamCMD
ARG DEBIAN_FRONTEND=noninteractive
RUN dpkg --add-architecture i386 \
 && apt-get update -y \
 && apt-get install -y --no-install-recommends ca-certificates locales steamcmd \
 && rm -rf /var/lib/apt/lists/*

# Add unicode support
RUN locale-gen en_US.UTF-8
ENV LANG 'en_US.UTF-8'
ENV LANGUAGE 'en_US:en'

# Create symlink for executable
RUN ln -s /usr/games/steamcmd /usr/bin/steamcmd

# Update SteamCMD and verify latest version
RUN steamcmd +quit

# RUN echo -e "\n \n \n \n \n \n 2 \n" | apt install lib32gcc-s1 steamcmd -y


USER root
CMD ["/usr/sbin/sshd", "-D", "-p", "7779"]







