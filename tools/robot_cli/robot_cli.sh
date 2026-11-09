#!/bin/bash
# robot_cli.sh — 로봇 상태 화면 · 명령 (C++ 패키지 ros2_ws/src/robot_cli 실행기). install.sh 뒤에는 어디서든 nt
#   nt                 상태 화면 + 입력줄 (? = 도움말)
#   nt status [쪽]     상태 한 번 (--watch = 1 s 마다)
#   nt check           시작 전 점검 (끝 코드 0 통과 · 1 실패)
#   nt h · nt g B1     팔 HOME · 접고 이동 (짧은 별칭 = nt help)
# ROS_DOMAIN_ID: 안 정했으면 팀 로봇 17 (tools/jetson/run_robot.sh 와 같음), export 한 값이 우선. 설명: ros2_ws/src/robot_cli/README.md
# FASTDDS_BUILTIN_TRANSPORTS: 이 창 값을 그대로 씀 (안 정했으면 기본). 로봇 노드와 같아야 보임 — 젯슨: CLI 만 LARGE_DATA 면
#   run_robot.sh (기본) 노드가 안 보였음. bringup.launch.py (LARGE_DATA) 와 같이 쓸 때는 모든 창에서 같은 값을 export
set -eo pipefail
ROOT="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/../.." && pwd)"   # 심볼릭 링크 (~/.local/bin/nt) 로 불러도 레포를 찾음
DISTRO=${ROS_DISTRO_WANT:-lyrical}
set +u; source /opt/ros/$DISTRO/setup.bash   # ROS setup 은 없는 변수를 씀
source "$ROOT/ros2_ws/install/setup.bash"
ros2 pkg prefix robot_cli >/dev/null 2>&1 || { echo "robot_cli 빌드 안 됨 — cd $ROOT/ros2_ws && colcon build --packages-select robot_cli"; exit 4; }
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-17}
exec ros2 run robot_cli robot_cli "$@"
