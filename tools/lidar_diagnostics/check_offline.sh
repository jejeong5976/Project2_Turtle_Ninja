#!/usr/bin/env bash
# Hardware-free C++ checks. Never opens a physical serial port or starts a ROS node.
set -euo pipefail
if [[ $# -lt 1 || $# -gt 3 ]]; then
  echo "Usage: $0 UPSTREAM_SOURCE [BUILT_TIMING_WORKSPACE [CAPTURE.jsonl]]" >&2
  exit 2
fi
script_dir="$(cd -- "$(dirname -- "$0")" && pwd)"
upstream="$(realpath -- "$1")"
(cd "$upstream" && sha256sum --check "$script_dir/source.sha256")
review_dir="$(mktemp -d /tmp/lidar-offline-check-XXXXXX)"
echo "Check artifacts: $review_dir"
flags=(-std=c++17 -Wall -Wextra -Werror -pedantic -fsanitize=undefined -fno-sanitize-recover=undefined)
for test in test_recorder test_timing_recorder test_sample_time_model test_host_time_anchor; do
  g++ "${flags[@]}" -pthread -I"$upstream/include" \
    "$script_dir/$test.cpp" -o "$review_dir/$test"
  mkdir "$review_dir/$test-output"
  "$review_dir/$test" "$review_dir/$test-output"
done
g++ "${flags[@]}" "$script_dir/replay_timing.cpp" -lboost_json -o "$review_dir/replay_timing"
if [[ $# -ge 2 ]]; then
  timing_workspace="$(realpath -- "$2")"
  "$timing_workspace/build/coin_d4_timing_driver/test_timing_parser" "$review_dir/parser.jsonl"
fi
if [[ $# == 3 ]]; then
  capture="$(realpath -- "$3")"
  "$review_dir/replay_timing" "$capture" "$review_dir/replay.csv"
  original_hash="$(sha256sum "$review_dir/replay.csv")"
  if "$review_dir/replay_timing" "$capture" "$review_dir/replay.csv"; then
    echo "FAIL: replay overwrote existing output" >&2; exit 1
  fi
  [[ "$(sha256sum "$review_dir/replay.csv")" == "$original_hash" ]]
  head -n -1 "$capture" > "$review_dir/truncated.jsonl"
  # INT64_MIN used to overflow end-start before malformed timestamps were rejected.
  sed -E 's/"start_ns":[0-9]+/"start_ns":-9223372036854775808/g' \
    "$capture" > "$review_dir/negative-start.jsonl"
  for invalid in truncated negative-start; do
    if "$review_dir/replay_timing" "$review_dir/$invalid.jsonl" "$review_dir/$invalid.csv" \
        2> "$review_dir/$invalid.log"; then
      echo "FAIL: replay accepted $invalid capture" >&2; exit 1
    fi
    [[ ! -e "$review_dir/$invalid.csv" ]]
  done
  [[ "$(cat "$review_dir/negative-start.log")" == *"Invalid ring boundary/count"* ]]
  [[ "$(cat "$review_dir/truncated.log")" == *"Missing metadata/footer"* ]]
fi
echo "PASS: standalone checks and replay compilation; artifacts retained at $review_dir"
