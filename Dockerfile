# CUDA Version 11.3.1

# Container image Copyright (c) 2016-2023, NVIDIA CORPORATION & AFFILIATES. All rights reserved.

# This container image and its contents are governed by the NVIDIA Deep Learning Container License.
# By pulling and using the container, you accept the terms and conditions of this license:
# https://developer.nvidia.com/ngc/nvidia-deep-learning-container-license

# A copy of this license is made available in this container at /NGC-DL-CONTAINER-LICENSE for your convenience.
FROM nvidia/cuda:11.3.1-cudnn8-devel-ubuntu20.04 as nvidia_base
LABEL maintainer="Ryu Taehun, <xogns2079@gmail.com>"

############################################

FROM nvidia_base as base
SHELL ["/bin/bash", "-c"]
ARG DEBIAN_FRONTEND=noninteractive
# switch to root user
USER root
# Install basic packages
RUN apt-get update \
    && apt-get install -y locales lsb-release git vim wget curl gnupg2 dirmngr unzip build-essential \
     sudo python3-pip software-properties-common libgl1-mesa-glx libgl1 libglib2.0-0 libpython3-dev \
     gnupg g++ libusb-1.0-0
# Set locale
RUN dpkg-reconfigure locales
RUN locale-gen en_US.UTF-8
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US:en
ENV LC_ALL en_US.UTF-8

############################################

FROM base as overlay
# Install ROS2 Foxy
RUN apt-get update && apt-get install -y \
    curl \
    && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
    && sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros2-latest.list' \
    && apt-get update \
    && apt-get install -y ros-foxy-desktop python3-argcomplete
# YOLO-v8-segmentation setting
RUN pip3 install ultralytics

############################################

FROM overlay as develop
RUN mkdir yolov8 && chmod -R +x yolov8 && cd yolov8 && \
  yolo segment predict model=yolov8n-seg.pt source='https://ultralytics.com/images/bus.jpg' show=True
RUN mkdir -p ros2_ws/src && chmod -R +x ./ros2_ws
WORKDIR /ros2_ws
RUN cd src && git clone https://github.com/mgonzs13/yolov8_ros.git \
  && pip3 install -r yolov8_ros/requirements.txt
RUN pip3 install -U pytest==7.2 && pip3 install -U colcon-common-extensions

COPY .. /ros2_ws/src/hive_vision
RUN ls --recursive /ros2_ws/src/hive_vision/
