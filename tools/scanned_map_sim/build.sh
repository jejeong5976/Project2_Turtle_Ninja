#!/usr/bin/env bash
set -eo pipefail
sim_tools=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
source "$sim_tools/paths.sh"
mkdir -p "$sim_build_root"
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
source "$sim_ros_setup"
colcon --log-base "$sim_build_root/log" build \
  --base-paths "$sim_repo/ros2_ws/src" \
  --packages-up-to body_route_controller bringup \
  --build-base "$sim_build_root/build" \
  --install-base "$sim_install" \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON
source "$sim_install/setup.bash"
cmake -S "$sim_tools" -B "$sim_helpers" \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBODY_ROUTE_PREFIX="$(ros2 pkg prefix body_route_controller)"
cmake --build "$sim_helpers" -j 3
