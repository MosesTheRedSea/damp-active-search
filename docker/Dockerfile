FROM docker.io/library/ros:foxy

ENV DEBIAN_FRONTEND=noninteractive

# Basic development tools
RUN apt update && apt install -y \
    build-essential \
    git \
    vim \
    curl \
    wget \
    python3-pip \
    python3-colcon-common-extensions \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# ROS dependencies commonly needed
RUN apt update && apt install -y \
    ros-foxy-tf2 \
    ros-foxy-tf2-ros \
    ros-foxy-nav-msgs \
    ros-foxy-geometry-msgs \
    ros-foxy-sensor-msgs \
    ros-foxy-std-msgs \
    ros-foxy-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# Set workspace directory
WORKDIR /foxy_ws

# Default shell when container starts
CMD ["bash"]