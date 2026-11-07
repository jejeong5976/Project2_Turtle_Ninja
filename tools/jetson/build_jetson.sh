#!/usr/bin/env bash
# 젯슨 빌드 — 저장소 맨 위에서: bash tools/jetson/build_jetson.sh  [--test] [패키지 ...]
#   메모리 8 GB 젯슨이라 작업자 1개 · make -j2. 남은 메모리 1.5 GB 아래면 멈춤 (swap 이 없으면 빌드 중 멈출 수 있음)
#   --test : 단위 시험까지 (swerve_controller · hardware · manipulation · navigation + tools/drive pytest). 하나라도 실패하면 종료 코드 1
#   패키지를 주면 그것만 (예: bash tools/jetson/build_jetson.sh manipulation bringup) — 설정 (config/) 만 바뀌면 bringup 하나
#   ccache 가 있으면 씀 (sudo apt install ccache 한 번): 안 바뀐 파일은 다시 컴파일 안 함
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/../.." && pwd)
DISTRO=${ROS_DISTRO_WANT:-lyrical}
[ -f /opt/ros/$DISTRO/setup.bash ] || { echo "✗ /opt/ros/$DISTRO 없음 — tools/jetson/jetson_check.sh 결과를 확인"; exit 2; }
set +u; source /opt/ros/$DISTRO/setup.bash; set -u
avail=$(LC_ALL=C free -m | awk '/Mem/{print $7}')
[ "$avail" -lt 1500 ] && { echo "✗ 남은 메모리 ${avail} MB — 다른 프로그램(SLAM · YOLO · 브라우저)을 끄고 다시"; exit 3; }
cd "$ROOT/ros2_ws"
TEST=OFF; [ "${1:-}" = "--test" ] && { TEST=ON; shift; }
PKGS=("$@")
SEL=(); [ ${#PKGS[@]} -gt 0 ] && SEL=(--packages-select "${PKGS[@]}")
CC=(); command -v ccache >/dev/null && CC=(-DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache)
echo "빌드: $ROOT/ros2_ws · ROS $DISTRO · $(uname -m) · 남은 메모리 ${avail} MB · 시험 $TEST · 패키지 ${PKGS[*]:-전부} · ccache $([ ${#CC[@]} -gt 0 ] && echo 씀 || echo 없음)"
MAKEFLAGS=-j2 colcon build --executor sequential "${SEL[@]}" --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=$TEST "${CC[@]}" 2>&1 | tee build_jetson.log
if [ "$TEST" = ON ]; then
  fail=0
  # 시험 노드가 켜져 있는 로봇(도메인 17)과 섞이지 않게 시험 전용 도메인 · 이 기기 안에서만 (README.md 시험과 같은 방식)
  ROS_DOMAIN_ID=181 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST colcon test --packages-select swerve_controller hardware manipulation navigation --executor sequential > test_jetson.log 2>&1 || fail=1
  colcon test-result --all | tail -3 || fail=1
  python3 -m pytest -q "$ROOT/tools/drive" || fail=1
  [ $fail = 0 ] || { echo "✗ 시험 실패 있음 — ros2_ws/test_jetson.log"; exit 1; }
fi
echo "끝. 다음: bash tools/jetson/run_robot.sh drive mock   (로봇 없이 확인)"
