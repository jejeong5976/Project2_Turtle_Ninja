#!/usr/bin/env bash
# SLAM 지도 저장 — run_robot.sh slam 이 떠 있는 동안, 로봇을 세운 채로.
#   bash tools/jetson/save_map.sh arena          → ~/maps/arena.{pgm,yaml} (보기 · Nav2 용) + ~/maps/arena.{posegraph,data} (locate 용)
# 같은 이름이 있으면 거부 (덮어쓰지 않음). 저장 뒤 locate: bash tools/jetson/run_robot.sh locate opencr ~/maps/arena
set -uo pipefail
NAME=${1:-}; [ -n "$NAME" ] || { sed -n 2,4p "$0"; exit 2; }
DIR=${NT_MAP_DIR:-$HOME/maps}; mkdir -p "$DIR"; OUT="$DIR/$NAME"
for e in pgm yaml posegraph data; do [ -e "$OUT.$e" ] && { echo "✗ $OUT.$e 이미 있음 — 다른 이름으로"; exit 2; }; done
set +u; source /opt/ros/${ROS_DISTRO_WANT:-lyrical}/setup.bash; set -u
# run_robot.sh slam 을 띄운 터미널과 같은 ROS_DOMAIN_ID · ROS_AUTOMATIC_DISCOVERY_RANGE 여야 서비스가 보임
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0} · discovery=${ROS_AUTOMATIC_DISCOVERY_RANGE:-기본} (run_robot.sh 터미널과 같아야 함)"
# 서비스가 없으면 ros2 service call 은 끝없이 기다림 → 20 s 제한 (posegraph 직렬화는 그래프가 크면 몇 초)
timeout 20 ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '$OUT'}}" || echo "  ✗ save_map 응답 없음"
timeout 20 ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '$OUT'}" || echo "  ✗ serialize_map 응답 없음"
ok=1; for e in pgm yaml posegraph data; do [ -s "$OUT.$e" ] && echo "  ✓ $OUT.$e" || { echo "  ✗ $OUT.$e 없음"; ok=0; }; done
[ $ok = 1 ] && echo "저장 끝 — locate: bash tools/jetson/run_robot.sh locate opencr $OUT" || { echo "✗ 일부 저장 실패 (slam_toolbox 가 떠 있는지 · ROS_DOMAIN_ID)"; exit 1; }
