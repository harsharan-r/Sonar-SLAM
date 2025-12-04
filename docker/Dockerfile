ARG BASE_IMAGE=ubuntu:22.04

################################ Source ################################
FROM ${BASE_IMAGE} AS source

# Avoid timezone prompts during install
ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies for Ros 2 and Python
RUN apt-get update && apt-get install -y \
    locales \
    curl \
    gnupg2 \
    lsb-release \
    python3 \
    python3-pip \
    python3-rpi.gpio \
    && rm -rf /var/lib/apt/lists/*

# Set locale (required by ROS 2)
RUN locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Set locale (required by ROS 2)
RUN locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Add ROS 2 apt repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
    && sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list'

# Install ROS 2 base
RUN apt-get update && apt-get install -y \
    ros-humble-ros-base \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Source ROS 2 environment automatically
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc

# Install Python packages
COPY requirements.txt .
RUN pip install -r requirements.txt

# Install ROS 2 packages
RUN apt-get update && apt-get install -y \
    ros-humble-foxglove-bridge

# Install custom fonts
RUN mkdir -p /root/.fonts
COPY figures/fonts/source_code_pro/*.ttf /root/.fonts/
RUN fc-cache -fv

# Set working directory
WORKDIR /src

# Default to bash shell
CMD ["bash"]
