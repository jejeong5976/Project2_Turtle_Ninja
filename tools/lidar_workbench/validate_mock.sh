#!/usr/bin/env bash
# Test entry point: fixed mock mode, no device options accepted.
set -euo pipefail
[[ $# == 1 || ( $# == 2 && $2 == --sensor-loss ) ]] || { echo "Usage: $0 WORKSPACE [--sensor-loss]" >&2; exit 2; }
script_dir="$(cd -- "$(dirname -- "$0")" && pwd)"
workbench_workspace="$(realpath -- "$1")"
result_dir="$(mktemp -d /tmp/lidar-workbench-validation-XXXXXX)"
printf 'Validation artifacts: %s\n' "$result_dir"
if [[ $# == 1 ]]; then
  "$script_dir/run.sh" "$workbench_workspace" --hardware mock --script "$script_dir/mock_sequence.txt" \
    --output "$result_dir/session" --domain 219 --filter-neighbors 1
else
  printf 'forward 0.30\nquit\n' > "$result_dir/fault_sequence.txt"
  "$script_dir/run.sh" "$workbench_workspace" --hardware mock --script "$result_dir/fault_sequence.txt" \
    --output "$result_dir/session" --domain 219 --filter-neighbors 1 > "$result_dir/console.log" 2>&1 &
  supervisor_pid=$!
  trap 'kill -INT "$supervisor_pid" 2>/dev/null || true; wait "$supervisor_pid" 2>/dev/null || true' EXIT
  deadline=$((SECONDS+90))
  until [[ -f "$result_dir/session/events.csv" ]] && rg -q 'motion_forward' "$result_dir/session/events.csv"; do
    kill -0 "$supervisor_pid" || { cat "$result_dir/console.log"; exit 1; }
    [[ $SECONDS -lt $deadline ]] || { echo 'Mock motion startup timed out' >&2; exit 1; }
    sleep 0.05
  done
  sleep 1
  fault_pid=""
  while IFS=, read -r label pid; do
    if [[ $label == front_sensor ]]; then fault_pid=$pid; fi
  done < "$result_dir/session/child_pids.csv"
  [[ $fault_pid =~ ^[0-9]+$ ]] || { echo 'Missing owned mock sensor PID' >&2; exit 1; }
  fault_time_ns="$(date +%s%N)"
  printf '%s\n' "$fault_time_ns" > "$result_dir/fault_time_ns.txt"
  kill -STOP -- "-$fault_pid"
  set +e
  wait "$supervisor_pid"
  result=$?
  set -e
  trap - EXIT
  cat "$result_dir/console.log"
  [[ $result == 1 ]] || { echo 'Fault trial did not fail closed' >&2; exit 1; }
  rg -q 'Integrated health lost' "$result_dir/console.log"
  if kill -0 -- "-$fault_pid" 2>/dev/null; then echo 'Mock sensor process group survived cleanup' >&2; exit 1; fi
fi
# Reading a saved bag needs no live ROS graph.
set +u
source /opt/ros/lyrical/setup.bash
source "$workbench_workspace/install/local_setup.bash"
if [[ $# == 1 ]]; then
  "$workbench_workspace/install/lidar_workbench/lib/lidar_workbench/verify_mock_recording" "$result_dir/session"
else
  "$workbench_workspace/install/lidar_workbench/lib/lidar_workbench/verify_mock_recording" "$result_dir/session" "$fault_time_ns"
fi
