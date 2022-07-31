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

## Install pip packages for dl env 
RUN pip3 install \
    Cython \
    Jinja2 \
    MarkupSafe \
    Pillow \
    Pygments \
    appnope \
    argparse \
    backports-abc \
    backports.ssl-match-hostname \
    certifi \
    cycler \
    decorator \
    future \
    gnureadline \
    h5py \
    ipykernel \
    ipython \
    ipython-genutils \
    ipywidgets \
    jsonschema \
    jupyter \
    jupyter-client \
    jupyter-console \
    jupyter-core \
    matplotlib \
    mistune \
    nbconvert \
    nbformat \
    nltk \
    notebook \
    numpy \
    path.py \
    pexpect \
    pickleshare \
    ptyprocess \
    pyparsing \
    python-dateutil \
    pytz \
    pyzmq \
    qtconsole \
    scipy \
    simplegeneric \
    singledispatch \
    six \
    terminado \
    tornado \
    traitlets

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





