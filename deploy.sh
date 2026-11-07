#!/bin/bash
set -eo pipefail

cd "$(dirname -- "${BASH_SOURCE[0]}")"

# Initialize the ROS underlay before building, including in fresh terminals.
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source ./setup_terminal.sh
