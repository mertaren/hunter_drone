
# ROS2 Humble and tools
FROM osrf/ros:humble-desktop-full

# Non-Interactive  mode
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# Utils
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    git \
    wget \
    curl \
    nano \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# MICRO-XRCE-DDS AGENT
RUN git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /tmp/Micro-XRCE-DDS-Agent \
    && cd /tmp/Micro-XRCE-DDS-Agent \
    && mkdir build && cd build \
    && cmake .. \
    && make \
    && make install \
    && ldconfig /usr/local/lib/ \
    && rm -rf /tmp/Micro-XRCE-DDS-Agent


# CV Stack
RUN pip3 install --no-cache-dir \
    opencv-python-headless \
    ultralytics \
    torch \
    torchvision

# WS
WORKDIR /root/hunter_ws

# Bash setup
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

CMD ["bash"]
