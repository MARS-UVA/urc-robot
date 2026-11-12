#!/bin/bash
set -eo pipefail

cd "$(dirname -- "${BASH_SOURCE[0]}")"

# Initialize the ROS underlay before building, including in fresh terminals.
source /opt/ros/jazzy/setup.bash
# Configure the unmodified Point-LIO package for Jazzy and external debug logs.
colcon build --symlink-install --packages-select point_lio --cmake-args \
    -DCMAKE_CXX_STANDARD=17 \
    '-DCMAKE_CXX_FLAGS=-UROOT_DIR -DROOT_DIR=\"/tmp/urc_point_lio/\"'
colcon build --symlink-install
source ./setup_terminal.sh
