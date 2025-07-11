# Base image
FROM ubuntu:22.04

# Avoid interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

# Locale and system dependencies
RUN apt-get update && apt-get install -y \
    locales \
    curl \
    gnupg2 \
    lsb-release \
    sudo \
    git \
    bash-completion \
    wget \
    software-properties-common \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# ROS 2 Humble keys and sources
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list

# Install ROS 2 and tools
RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-argcomplete \
    python3-pip \
    && apt-get clean

# Initialize rosdep
RUN rosdep init || true

# Set up environment
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc
ENV ROS_DISTRO=humble

# Set up non-root user for VSCode
ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && usermod -aG sudo $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER $USERNAME
WORKDIR /workspace/ros2_ws
