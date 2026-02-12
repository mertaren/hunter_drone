FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
ENV SHELL /bin/bash

# System Dependencies
RUN apt-get update && apt-get install -y \
    git wget curl nano build-essential cmake g++ \
    python3-pip python3-dev \
    # ROS 2 & Gazebo Bridges
    ros-humble-gazebo-ros-pkgs \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-cv-bridge \
    ros-humble-vision-msgs \
    # GStreamer
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav gstreamer1.0-tools \
    libopencv-dev protobuf-compiler \
    && apt-get remove -y python3-sympy \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir --upgrade pip

# Install Core & AI Libs
RUN pip3 install --no-cache-dir \
    "numpy<2" \
    empy==3.3.4 \
    pyros-genmsg \
    setuptools==59.6.0 \
    future kconfiglib jsonschema jinja2 packaging toml lxml pyserial scipy \
    torch torchvision \
    ultralytics

# Micro-XRCE-DDS-Agent
WORKDIR /tmp
RUN git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git && \
    cd Micro-XRCE-DDS-Agent && \
    mkdir build && cd build && \
    cmake .. && make && make install && \
    ldconfig /usr/local/lib/ && \
    cd /tmp && rm -rf Micro-XRCE-DDS-Agent

# Git Safety Config
RUN git config --global --add safe.directory '*'

# Environment Setup
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /root/hunter_ws/install/setup.bash" >> /root/.bashrc && \
    echo "export RCUTILS_COLORIZED_OUTPUT=1" >> /root/.bashrc && \
    echo "alias gzfps='gz stats'" >> /root/.bashrc && \
    echo "alias cb='colcon build --symlink-install'" >> /root/.bashrc && \
    echo "alias ssetup='source install/setup.bash'" >> /root/.bashrc

# Set workspace
WORKDIR /root/hunter_ws
CMD ["/bin/bash"]