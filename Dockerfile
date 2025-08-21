# =============================
# Build librealsense
# =============================
ARG ROS_DISTRO="jazzy"

FROM ros:${ROS_DISTRO} AS realsense-build
ARG RSVER="2.57.2"

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
    build-essential cmake git pkg-config \
    libssl-dev libusb-1.0-0-dev \
    libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev \
    curl python3 python3-dev ca-certificates \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
RUN curl -L https://codeload.github.com/IntelRealSense/librealsense/tar.gz/refs/tags/v$RSVER -o librealsense.tar.gz
RUN tar -xzf librealsense.tar.gz && rm librealsense.tar.gz
RUN ln -s /ws/librealsense-$RSVER /ws/librs

WORKDIR /ws/librealsense-$RSVER/build
RUN cmake .. \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_PYTHON_BINDINGS=ON \
    -DBUILD_GLSL_EXTENSIONS=ON \
    -DCHECK_FOR_UPDATES=OFF \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/opt/librs
RUN make -j$(nproc) all
RUN make install

# =============================
# Base runtime image
# =============================
FROM ros:${ROS_DISTRO} AS robot-base

# Copy built librealsense libs + rules
COPY --from=realsense-build /opt/librs /usr/local/
COPY --from=realsense-build /ws/librs/config/99-realsense-libusb.rules /etc/udev/rules.d/
COPY --from=realsense-build /ws/librs/config/99-realsense-d4xx-mipi-dfu.rules /etc/udev/rules.d/

RUN apt-get update && apt-get upgrade -y \
    && apt-get install -y --no-install-recommends \
    libusb-1.0-0 udev \
    apt-transport-https ca-certificates \
    curl iputils-ping usbutils \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# ROS packages needed at runtime
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-realsense2-camera \
    ros-${ROS_DISTRO}-rtabmap-ros \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir robotpy --break-system-packages

WORKDIR /robot_ws
COPY . .

# =============================
# Development environment
# =============================
FROM robot-base AS robot-dev
# Dev tools & debugging utilities
RUN apt-get update && apt-get install -y --no-install-recommends \
    git vim nano less tmux \
    python3-pip python3-colcon-common-extensions \
    python3-rosdep python3-vcstool \
    gdb lldb valgrind \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init || true && rosdep update

WORKDIR /robot_ws

FROM robot-base AS robot-sys
WORKDIR /robot_ws
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build --symlink-install
RUN . install/local_setup.sh
CMD ["rs-enumerate-devices"]
#CMD ["ros2 launch robot vision_launch.py"]