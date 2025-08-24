# =============================
# Build librealsense
# =============================
ARG ROS_DISTRO="jazzy"

FROM ros:${ROS_DISTRO} AS realsense-build
ARG RSVER="2.57.2"

# librealsense dependencies + build tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git pkg-config \
    libssl-dev libusb-1.0-0-dev \
    libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev \
    curl python3 python3-dev ca-certificates \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
RUN curl -L https://codeload.github.com/IntelRealSense/librealsense/tar.gz/refs/tags/v$RSVER -o librealsense.tar.gz \
    && tar -xzf librealsense.tar.gz \
    && rm librealsense.tar.gz
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

# TODO: build realsense-ros package (arm64 requires it)

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
    libusb-1.0-0 udev python3-pip \
    apt-transport-https ca-certificates \
    curl software-properties-common \
    ros-${ROS_DISTRO}-rtabmap-ros \
    && rm -rf /var/lib/apt/lists/*

# TODO: rosdep install dependencies
RUN pip install --no-cache-dir --break-system-packages robotpy

WORKDIR /robot_ws
COPY ./src/ ./src/
RUN rosdep update && rosdep install --from-paths src -y --ignore-src
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build --symlink-install
RUN . install/local_setup.sh
CMD ["rs-enumerate-devices"]
#CMD ["ros2 launch robot vision_launch.py"]