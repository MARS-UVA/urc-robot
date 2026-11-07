#!/bin/bash
set -eo pipefail

cd "$(dirname -- "${BASH_SOURCE[0]}")"

# Build and initialize the workspace before starting the simulation.
source ./deploy.sh

# Keep Gazebo discovery on the container's local interface unless overridden.
export GZ_IP="${GZ_IP:-127.0.0.1}"
exec ros2 launch startup sim_launch.py enable_rviz:=true "$@"
