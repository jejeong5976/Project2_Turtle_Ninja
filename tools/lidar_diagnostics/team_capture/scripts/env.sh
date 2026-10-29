#!/usr/bin/env bash
# Source only: user-run helpers attach to the existing Jetson mapping session.
set +u
source /opt/ros/lyrical/setup.bash
set -u
export ROS_DOMAIN_ID=182
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_STATIC_PEERS=
capture_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
