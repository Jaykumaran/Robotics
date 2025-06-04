#!/bin/bash
set -e # Exit immediately if a command exists with non-zero status

# Source the ROS 2 installation setup file
if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
    echo "Sourced ROS 2 ${ROS_DISTRO} setup."
else
    echo "Error: ROS 2 ${ROS_DISTRO} setup.bash not found."
fi 

# Source the workspace's setup file from /ros2_ws (which is the WORKDIR)
WS_SETUP_FILE="/ros2_ws/install/setup.bash" 
if [ -f "${WS_SETUP_FILE}" ]; then
   source "${WS_SETUP_FILE}"
   echo "Sourced custom workspace (${WS_SETUP_FILE})."
else
   echo "Error: Custom workspace setup file (${WS_SETUP_FILE}) not found."
fi

# Execute the command passed into 'docker run' (or the CMD in Dockerfile)
exec "$@"

# ** Important:  Before building the package, I've to do this to make the docker file executable**
# chmod +x docker-entrypoint.sh