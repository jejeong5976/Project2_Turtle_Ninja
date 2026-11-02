#!/usr/bin/env bash
# 젯슨 빌드 — 저장소 맨 위에서: bash tools/jetson/build_jetson.sh  [--test]
#   메모리 8 GB 젯슨이라 작업자 1개 · make -j2. 남은 메모리 1.5 GB 아래면 멈춤 (swap 이 없으면 빌드 중 멈출 수 있음)
#   --test : 단위 시험까지 (swerve_controller · hardware · manipulation · navigation + tools/drive pytest). 하나라도 실패하면 종료 코드 1
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/../.." && pwd)
DISTRO=${ROS_DISTRO_WANT:-lyrical}
[ -f /opt/ros/$DISTRO/setup.bash ] || { echo "✗ /opt/ros/$DISTRO 없음 — tools/jetson/jetson_check.sh 결과를 확인"; exit 2; }
set +u; source /opt/ros/$DISTRO/setup.bash; set -u
avail=$(LC_ALL=C free -m | awk '/Mem/{print $7}')
[ "$avail" -lt 1500 ] && { echo "✗ 남은 메모리 ${avail} MB — 다른 프로그램(SLAM · YOLO · 브라우저)을 끄고 다시"; exit 3; }
cd "$ROOT/ros2_ws"
TEST=OFF; [ "${1:-}" = "--test" ] && TEST=ON
echo "빌드: $ROOT/ros2_ws · ROS $DISTRO · $(uname -m) · 남은 메모리 ${avail} MB · 시험 $TEST"
MAKEFLAGS=-j2 colcon build --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=$TEST 2>&1 | tee build_jetson.log
if [ "$TEST" = ON ]; then
  fail=0
  colcon test --packages-select swerve_controller hardware manipulation navigation --executor sequential > test_jetson.log 2>&1 || fail=1
  colcon test-result --all | tail -3 || fail=1
  python3 -m pytest -q "$ROOT/tools/drive" || fail=1
  [ $fail = 0 ] || { echo "✗ 시험 실패 있음 — ros2_ws/test_jetson.log"; exit 1; }
fi
echo "끝. 다음: bash tools/jetson/run_robot.sh drive mock   (로봇 없이 확인)"
