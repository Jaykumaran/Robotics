#!/bin/bash
set -e # Exit immediately if a command exists with non-zero status

# Source the ROS 2 installation setup file
if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
    echo "Sourced ROS 2 ${ROS_DISTRO} setup."
else
    echo "ROS 2 ${ROS_DISTRO} setup.bash not found."
fi 

# Source the workspace's setup file
WS_SETUP_FILE="/ros2_ws/install/setup.bash"
if [ -f "${WS_SETUP_FILE}" ]; then
   source "${WS_SETUP_FILE}"
   echo "Sourced custom workspace (${WS_SETUP_FILE})."
else
   echo "Custom workspace setup file (${WS_SETUP_FILE}) not found."
fi # fi is just if spelled backward, just indicates the end of the if statement

# Execute the command passed into 'docker run' (or the CMD in Dockerfile)
exec "$@"


# ** Important:  Before building the package, I've to do this to make the docker file executable**
# chmod +x docker-entrypoint.sh