FROM osrf/ros:humble-desktop-full

RUN apt-get update && apt-get install -y \
    git \
    wget \
    curl \
    nano \
    build-essential \
    cmake \
    g++ \
    python3-pip \
    python3-dev \
    && rm -rf /var/lib/apt/lists/*

# PX4 and Gazebo
RUN apt-get update && apt-get install -y \
    gazebo \
    libgazebo-dev \
    libopencv-dev \
    protobuf-compiler \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer1.0-dev \
    python3-opencv \
    python3-numpy \
    python3-toml \
    python3-packaging \
    python3-jinja2 \
    python3-jsonschema \
    python3-yaml \
    python3-lxml \
    python3-kconfiglib \
    && rm -rf /var/lib/apt/lists/*

# Python libs
RUN pip3 install --no-cache-dir \
    empy==3.3.4 \
    pyros-genmsg \
    setuptools==59.6.0 \
    future \
    kconfiglib \
    jsonschema \
    jinja2 \
    packaging \
    toml \
    numpy \
    lxml

# Micro-XRCE-DDS-Agent
WORKDIR /tmp
RUN git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git && \
    cd Micro-XRCE-DDS-Agent && \
    mkdir build && cd build && \
    cmake .. && \
    make && \
    make install && \
    ldconfig /usr/local/lib/ && \
    cd /tmp && rm -rf Micro-XRCE-DDS-Agent

# ROS 2 vision packages
RUN apt-get update && apt-get install -y \
    ros-humble-cv-bridge \
    ros-humble-vision-msgs \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    && rm -rf /var/lib/apt/lists/*

# Install YOLOv8 and additional Python vision tools
RUN pip3 install --no-cache-dir \
    ultralytics \
    pyserial \
    scipy

# Bash settings
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "export RCUTILS_COLORIZED_OUTPUT=1" >> /root/.bashrc && \
    echo "alias gzfps='gz stats'" >> /root/.bashrc && \
    echo "alias cb='colcon build --symlink-install'" >> /root/.bashrc && \
    echo "alias ssetup='source install/setup.bash'" >> /root/.bashrc

WORKDIR /root/hunter_ws

CMD ["tail", "-f", "/dev/null"]