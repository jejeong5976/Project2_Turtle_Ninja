#!/usr/bin/env bash
# Passive recorder; never starts drivers, changes parameters or publishes commands.
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
[[ $# == 0 ]] || { echo 'Usage: ./scripts/record_bag.sh' >&2; exit 2; }
mkdir -p -- "$capture_root/runs"
capture_bag="$capture_root/runs/bag_$(date +%Y%m%dT%H%M%S_%N)"
[[ ! -e "$capture_bag" ]] || { echo 'Output already exists; refusing overwrite.' >&2; exit 1; }
echo "Recording to: $capture_bag"
echo 'Wait for topic subscriptions before moving. Save the map before Ctrl+C here.'
echo 'Recorder keyboard pause controls are disabled; Ctrl+C still closes the bag.'
exec ros2 bag record --storage sqlite3 --output "$capture_bag" \
  --node-name team_mapping_recorder --disable-keyboard-controls \
  --qos-profile-overrides-path "$capture_root/record_qos.yaml" --topics \
  /lidar_front/scan_mapping /lidar_front/mapping_status /odom /tf /tf_static \
  /joint_states /cmd_vel /robot_description /map \
  /slam_toolbox/pose_graph /slam_toolbox/new_node_event /slam_toolbox/loop_closure_event
