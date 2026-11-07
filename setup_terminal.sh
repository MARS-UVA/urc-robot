#!/bin/bash
# Fast DDS is included with the ROS Jazzy base image.
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/jazzy/setup.bash
WORKSPACE_SETUP="$(dirname -- "${BASH_SOURCE[0]}")/install/setup.bash"
if [ -f "${WORKSPACE_SETUP}" ]; then
  source "${WORKSPACE_SETUP}"
fi
