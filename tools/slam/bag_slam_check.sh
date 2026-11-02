#!/bin/bash
# 기록(bag) 으로 SLAM 설정 확인 (2026-10-31, 노트북 · 로봇 없이). 10-27 젯슨 매핑 bag 을 새 slam_toolbox(config/slam_mapping.yaml) 에 넣어
# 지도를 처음부터 다시 만들고 저장 → 그때 지도와 비교. 실물 · 가짜 시험과 겹치지 않게 ROS_DOMAIN_ID 91.
#   bash tools/slam/bag_slam_check.sh <bag 폴더> <출력 폴더> [속도 2]
set -euo pipefail
BAG=$1; OUT=$2; RATE=${3:-2}
ROOT=$(cd "$(dirname "$0")/../.." && pwd)
mkdir -p "$OUT"
export ROS_DOMAIN_ID=91 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
set +u; source /opt/ros/lyrical/setup.bash; source "$ROOT/ros2_ws/install/setup.bash"; set -u
setsid ros2 launch bringup slam.launch.py robot:=false lidar:=false use_sim_time:=true slam_mode:=mapping > "$OUT/slam.log" 2>&1 < /dev/null &
SL=$!
setsid python3 "$ROOT/tools/slam/tf_drop_map.py" > "$OUT/tf_drop.log" 2>&1 < /dev/null &
TF=$!
trap 'kill -INT -- -$SL -$TF 2>/dev/null; sleep 3; kill -TERM -- -$SL -$TF 2>/dev/null || true' EXIT
sleep 6
ros2 bag play "$BAG" --clock --rate "$RATE" --remap /tf:=/tf_bag /lidar_front/scan_mapping:=/lidar_front/scan \
  --topics /lidar_front/scan_mapping /odom /tf /tf_static > "$OUT/play.log" 2>&1
sleep 4
timeout 20 ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '$OUT/map'}}" > "$OUT/save.log" 2>&1 || true
timeout 20 ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '$OUT/map'}" >> "$OUT/save.log" 2>&1 || true
ls -la "$OUT"
