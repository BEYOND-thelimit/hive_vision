# CUDA Version 11.3.1

# Container image Copyright (c) 2016-2023, NVIDIA CORPORATION & AFFILIATES. All rights reserved.

# This container image and its contents are governed by the NVIDIA Deep Learning Container License.
# By pulling and using the container, you accept the terms and conditions of this license:
# https://developer.nvidia.com/ngc/nvidia-deep-learning-container-license

# A copy of this license is made available in this container at /NGC-DL-CONTAINER-LICENSE for your convenience.
FROM nvidia/cuda:11.3.1-cudnn8-devel-ubuntu20.04
LABEL maintainer="Ryu Taehun, <xogns2079@gmail.com>"

###############################
## Set environment variables ##
###############################
SHELL ["/bin/bash", "-c"]
ARG DEBIAN_FRONTEND=noninteractive
# you should set HOST_USER using --build-arg option like HOST_USER=$USER
ARG HOST_USER
ARG UNAME=${HOST_USER}
ARG UID=1000
ARG GID=1000
ARG HOME=/home/${UNAME}
# switch to root user
USER root

####################################
## Install the required packages. ##
####################################
# Reset apt package list
RUN apt-get autoremove --purge --yes \
    && rm -rf /var/lib/apt/lists/*

# Install basic packages
RUN apt-get update \
    && apt-get install -y locales lsb-release git vim wget curl gnupg2 dirmngr unzip build-essential sudo
RUN dpkg-reconfigure locales

# Set locale
RUN locale-gen en_US.UTF-8
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US:en
ENV LC_ALL en_US.UTF-8

# Install ROS2 Foxy
RUN apt-get update && apt-get install -y \
    curl \
    && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
    && sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros2-latest.list' \
    && apt-get update \
    && apt-get install -y ros-foxy-desktop python3-argcomplete


#######################
## Set non-root user ##
#######################
# Grant sudo permissions to ${UNAME} user only without password
RUN mkdir -p /etc/sudoers.d && \
    echo "${UNAME} ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/${UNAME} && \
    chmod 0440 /etc/sudoers.d/${UNAME}


# Create a group
RUN groupadd -g ${GID} usergroup
# Create a user
RUN useradd --no-log-init -m -u ${UID} -g ${GID} -s /bin/bash ${UNAME}
# Set user
USER ${UNAME}
# The WORKDIR instruction sets the working directory for any RUN, CMD, ENTRYPOINT, COPY and ADD instructions.
WORKDIR $HOME