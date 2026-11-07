#!/usr/bin/env bash
set -euo pipefail
source /opt/ros/jazzy/setup.bash
# The base image normally initializes rosdep. Keep this idempotent.
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update --rosdistro jazzy
sudo apt-get update
# Mongo warehouse support is optional; it is not available on all architectures.
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y \
    --skip-keys 'warehouse_ros_mongo phoenix6'
printf '\nReady. Run: colcon build && source install/setup.bash\n'
