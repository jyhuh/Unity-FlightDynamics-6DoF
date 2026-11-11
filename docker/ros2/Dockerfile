# Licensed under the Apache License, Version 2.0.
# Container 2: ROS 2 Humble + eProsima Micro-XRCE-DDS-Agent + px4_msgs Autonomy Node
FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# Include libasio-dev, libtinyxml2-dev, libssl-dev required by eProsima Fast-DDS / Micro-XRCE-DDS-Agent
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    libasio-dev \
    libtinyxml2-dev \
    libssl-dev \
    libspdlog-dev \
    pkg-config \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    iproute2 \
    iputils-ping \
    && rm -rf /var/lib/apt/lists/*

# 1. Build eProsima Micro-XRCE-DDS-Agent (Apache 2.0) into isolated /opt/uxrce prefix
#    so its bundled Fast-DDS 2.12 never collides with ROS 2 Humble's /opt/ros/humble/lib/libfastrtps.so
RUN git clone --depth 1 --branch v2.4.2 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /tmp/Micro-XRCE-DDS-Agent && \
    sed -i 's/2\.12\.x/v2.12.2/g' /tmp/Micro-XRCE-DDS-Agent/CMakeLists.txt && \
    cmake -S /tmp/Micro-XRCE-DDS-Agent -B /tmp/Micro-XRCE-DDS-Agent/build \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/opt/uxrce \
        -DCMAKE_INSTALL_RPATH=/opt/uxrce/lib && \
    cmake --build /tmp/Micro-XRCE-DDS-Agent/build -j"$(nproc)" && \
    cmake --install /tmp/Micro-XRCE-DDS-Agent/build && \
    ln -sf /opt/uxrce/bin/MicroXRCEAgent /usr/local/bin/MicroXRCEAgent && \
    rm -rf /tmp/Micro-XRCE-DDS-Agent

# 2. Build px4_msgs (release/1.15, BSD-3-Clause) inside /ros2_ws
WORKDIR /ros2_ws
RUN mkdir -p /ros2_ws/src && \
    git clone --depth 1 --branch release/1.15 https://github.com/PX4/px4_msgs.git /ros2_ws/src/px4_msgs && \
    bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"

COPY unity_px4_offboard_node.py /ros2_ws/unity_px4_offboard_node.py
COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /ros2_ws/unity_px4_offboard_node.py /usr/local/bin/entrypoint.sh && \
    echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["agent"]
