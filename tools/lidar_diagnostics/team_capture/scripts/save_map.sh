#!/usr/bin/env bash
# User runs while physically stopped; keeps the current mapper and recorder alive.
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
[[ $# == 0 ]] || { echo 'Usage: ./scripts/save_map.sh' >&2; exit 2; }
mkdir -p -- "$capture_root/runs"
capture_map=$(mktemp -d "$capture_root/runs/map_$(date +%Y%m%dT%H%M%S).XXXXXX")
echo "Checkpoint directory (retained even on failure): $capture_map"
ros2 run nav2_map_server map_saver_cli -f "$capture_map/map" \
  --ros-args -p save_map_timeout:=10.0 -p map_subscribe_transient_local:=true \
  2>&1 | tee "$capture_map/map_save.log"
# Escape apostrophes for a YAML single-quoted scalar; no eval or shell re-parsing.
capture_graph_yaml=${capture_map//\'/\'\'}
timeout 30 ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '$capture_graph_yaml/map_graph'}" 2>&1 | tee "$capture_map/serialize.log"
grep -Eq 'result[=:][[:space:]]*0([,) }]|$)' "$capture_map/serialize.log" || {
  echo 'Serializer did not report result=0. Keep this directory and inspect the log.' >&2; exit 1;
}
for capture_file in map.pgm map.yaml map_graph.posegraph map_graph.data; do
  [[ -s "$capture_map/$capture_file" ]] || { echo "Missing/empty $capture_file" >&2; exit 1; }
done
(cd -- "$capture_map" && sha256sum map.pgm map.yaml map_graph.posegraph map_graph.data > SHA256SUMS)
echo "CHECKPOINT SAVED: $capture_map"
echo 'Now finish the console at READY with q, then Ctrl+C in the recorder terminal.'
