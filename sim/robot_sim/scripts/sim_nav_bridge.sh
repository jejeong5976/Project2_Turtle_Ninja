#!/usr/bin/env bash
# Isaac Lab 시뮬 + ROS 2 Lyrical 브리지 실행 (scripts/sim_nav_bridge.py). 인자는 그대로 넘김.
#   예) scripts/sim_nav_bridge.sh                 # 창 없이 (Nav2 · RViz 는 따로)
#       scripts/sim_nav_bridge.sh --viz kit --camera --real
# Isaac 파이썬(3.12)에서 시스템 Lyrical rclpy 를 씀: ROS 경로 + 순수 파이썬 모듈 em · lark (python3-empy · python3-lark, Isaac venv 에 없음) 만 덧붙임.
# 다른 시스템 패키지는 넣지 않음 (Isaac venv 패키지와 충돌 방지).
# 물리 기본 = CPU: env 1개에서 GPU PhysX 는 스텝당 88 ms (실시간 0.23배), CPU 는 11 ms (1.8배) (2026-10-28 측정). --device cuda:0 로 바꿀 수 있음
set -e
HERE="$(cd "$(dirname "$0")/.." && pwd)"
source /opt/ros/lyrical/setup.bash
SHIM="$(mktemp -d "${TMPDIR:-/tmp}/ros_shim.XXXX")"
trap 'rm -rf "$SHIM"' EXIT
ln -s /usr/lib/python3/dist-packages/em.py "$SHIM/em.py"
ln -s /usr/lib/python3/dist-packages/lark "$SHIM/lark"
PY="${ISAACLAB_PY:-$HOME/pa-course/IsaacLab-6.2/.venv/bin/python}"
cd "$HERE"
env -u PYTHONPATH PYTHONPATH="/opt/ros/lyrical/lib/python3.12/site-packages:$SHIM" OMNI_KIT_ACCEPT_EULA=YES \
    "$PY" scripts/sim_nav_bridge.py --device cpu "$@"
