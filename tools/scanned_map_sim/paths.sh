#!/usr/bin/env bash
# Shared paths; source this after defining sim_tools. No hardware setup.
sim_repo=$(cd -- "$sim_tools/../.." && pwd)
sim_ros_setup=${SCANNED_SIM_ROS_SETUP:-/opt/ros/lyrical/setup.bash}
sim_build_root=${SCANNED_SIM_BUILD_ROOT:-$sim_repo/.scanned-map-sim}
sim_install=${SCANNED_SIM_INSTALL:-$sim_build_root/install}
sim_helpers=${SCANNED_SIM_HELPERS:-$sim_build_root/helpers}
sim_results=${SCANNED_SIM_RESULTS:-$sim_repo/output/scanned_map_controller_sim/runs}
