# Use the official OSRF ROS 2 Humble full desktop image
# This image was recently updated and should have correct GPG keys.
FROM osrf/ros:humble-desktop-full

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV SHELL=/bin/bash

# STEP 1: Install ONLY your specific system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-opencv \
    libopencv-dev \
    python3-numpy \
    && rm -rf /var/lib/apt/lists/*

# STEP 2: Set up ROS workspace, copy code, install ROS deps for your package, and build it.
WORKDIR /ros2_ws 
COPY . ./src/sim_cam_pkg/

RUN ["/bin/bash", "-c", ". /opt/ros/${ROS_DISTRO}/setup.bash && \
    apt-get update && \
    rosdep init || true && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y --rosdistro ${ROS_DISTRO} && \
    colcon build --symlink-install --packages-select sim_cam_pkg && \
    rm -rf /var/lib/apt/lists/* /root/.cache/rosdep"]

# STEP 3: Entrypoint setup
COPY ./docker-entrypoint.sh /docker-entrypoint.sh
RUN chmod +x /docker-entrypoint.sh

ENTRYPOINT [ "/docker-entrypoint.sh" ]
CMD ["ros2", "launch", "sim_cam_pkg", "object_tracking_pipeline.launch.py"]
