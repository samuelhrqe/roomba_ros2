# Ubuntu 22.04 x86_64 (use: docker build --platform=linux/amd64 ...)
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=humble

SHELL ["/bin/bash", "-c"]

# ---------------------------------------------------------------------
# Base do sistema, universe e dependências gerais
# ---------------------------------------------------------------------
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y --no-install-recommends \
        curl git wget cmake build-essential \
        libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev python3-dev \
        libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev at \
        software-properties-common \
        python3-pip \
        locales \
        ca-certificates && \
    add-apt-repository universe && \
    rm -rf /var/lib/apt/lists/*

# Locale
RUN locale-gen en_US.UTF-8 && \
    update-locale LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8

# ---------------------------------------------------------------------
# librealsense v2.54.2 com bindings Python
# ---------------------------------------------------------------------
RUN set -e; \
    cd /opt && \
    git clone --depth 1 https://github.com/IntelRealSense/librealsense.git -b v2.54.2 && \
    cd librealsense && \
    mkdir build && cd build && \
    cmake ../ \
      -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_EXAMPLES=true \
      -DBUILD_GRAPHICAL_EXAMPLES=false \
      -DBUILD_TOOLS=false \
      -DBUILD_PYTHON_BINDINGS=bool:true \
      -DPYTHON_EXECUTABLE=$(which python3) -L && \
    make -j"$(($(nproc)-1))" && \
    make install && \
    PYDIR=$(python3 -c 'import sysconfig; print(sysconfig.get_paths()["platlib"])') && \
    if ls /usr/local/OFF/*.so >/dev/null 2>&1; then \
      cp /usr/local/OFF/*.so "$PYDIR"/; \
    fi && \
    python3 -c "import pyrealsense2 as rs; print('OK:', rs.__version__)" && \
    if command -v rs-enumerate-devices >/dev/null 2>&1; then \
      rs-enumerate-devices || true; \
    fi && \
    cd / && \
    rm -rf /opt/librealsense

# ---------------------------------------------------------------------
# ROS 2 Humble (via ros-apt-source)
# ---------------------------------------------------------------------
RUN set -e; \
    . /etc/os-release; \
    export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}'); \
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${UBUNTU_CODENAME:-${VERSION_CODENAME}}_all.deb"; \
    dpkg -i /tmp/ros2-apt-source.deb; \
    apt-get update && apt-get upgrade -y && \
    apt-get install -y --no-install-recommends \
        ros-humble-ros-base \
        ros-dev-tools \
        ros-humble-rmw-cyclonedds-cpp \
        python3-rosdep && \
    rm -rf /var/lib/apt/lists/* /tmp/ros2-apt-source.deb

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# ---------------------------------------------------------------------
# Workspace ROS2, realsense-ros e roomba_ros2
# ---------------------------------------------------------------------
RUN set -e; \
    source /opt/ros/humble/setup.bash; \
    mkdir -p /opt/ros2_ws/src && \
    cd /opt/ros2_ws/src && \
    git clone --depth 1 https://github.com/IntelRealSense/realsense-ros.git -b 4.54.1 && \
    git clone --depth 1 https://github.com/samuelhrqe/roomba_ros2.git && \
    cd /opt/ros2_ws && \
    apt-get update && \
    rosdep init || echo "rosdep already initialized"; \
    for i in 1 2 3; do \
      rosdep update && break || (echo "rosdep update failed, trying again in 30 seconds..." && sleep 30); \
    done; \
    rosdep install --from-paths src --rosdistro "$ROS_DISTRO" \
        --ignore-src --skip-keys=librealsense2 --skip-keys=libfreenect -r -y && \
    colcon build && \
    echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /opt/ros2_ws/install/setup.bash" >> /root/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /root/.bashrc && \
    rm -rf /var/lib/apt/lists/* /opt/ros2_ws/build /opt/ros2_ws/log

# ---------------------------------------------------------------------
# PyTorch, Ultralytics e demais dependências Python
# ---------------------------------------------------------------------
RUN pip3 install --no-cache-dir \
      torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu && \
    pip3 install --no-cache-dir \
      ultralytics --no-deps && \
    pip3 install --no-cache-dir \
      polars requests torchvision ultralytics-thop opencv-python "numpy==1.26.4"

WORKDIR /opt/ros2_ws

# Set config directory to YOLO
RUN mkdir -p /tmp/Ultralytics && chmod 777 /tmp/Ultralytics
ENV YOLO_CONFIG_DIR=/tmp/Ultralytics

CMD ["bash", "-lc", "source /opt/ros/humble/setup.bash && source /opt/ros2_ws/install/setup.bash && ros2 launch roomba_ros2 roomba_camera_launch.py"]
