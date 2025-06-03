# Use an official ROS 2 Humble base image
FROM osrf/ros:humble-desktop-full

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV SHELL=/bin/bash

# STEP 1: IMMEDIATELY fix/update the ROS GPG key and sources.
RUN apt-get update -o Acquire::AllowInsecureRepositories=true -o Acquire::AllowDowngradeToInsecureRepositories=true && \
    apt-get install -y --no-install-recommends curl gnupg lsb-release && \
    rm -f /etc/apt/sources.list.d/ros2*.list && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && \
    rm -rf /var/lib/apt/lists/*

# STEP 2: Install specific system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-opencv \
    libopencv-dev \
    python3-numpy \
    && rm -rf /var/lib/apt/lists/*

# STEP 3: Set up ROS workspace and build the package
WORKDIR /ros2_ws
COPY . ./src/sim_cam_pkg/

RUN ["/bin/bash", "-c", ". /opt/ros/${ROS_DISTRO}/setup.bash && \
    apt-get update && \
    rosdep init || true && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y --rosdistro ${ROS_DISTRO} && \
    colcon build  && \
    rm -rf /var/lib/apt/lists/* /root/.cache/rosdep"]

# STEP 4: Entrypoint setup
COPY ./docker-entrypoint.sh /docker-entrypoint.sh
RUN chmod +x /docker-entrypoint.sh

ENTRYPOINT [ "/docker-entrypoint.sh" ]
# CMD ["bash"]
# New default  - Launch file
CMD ["ros2", "launch", "sim_cam_pkg", "object_tracking_pipeline.launch.py"] 

