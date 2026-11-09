#!/usr/bin/env bash
# Isaac 검증 (두 단계): 주행 정책 (navigate_to_pose) + 정밀 접근 정책 (dock_to_pose, --box_track · --goal_frame odom · 영역 끄기)
#   으로 dice 10 곳, planning Executor 와 같은 넘김 (Nav2 목표 = 면 2 cm, 정차점 0.40 m 안 취소 → 정밀 접근). 경기장 전체 녹화.
#   브리지 → Nav2 일부 (rl_nav.launch.py) → 주행 노드 · 접근 노드 → run_dock_goals.py → 녹화 끝 → 정리
# 사용: run_isaac_dock.sh <주행 policy.(pt|npz)> <주행 cfg.json> <접근 policy> <접근 cfg.json> <이름> [녹화 초 (기본 330)]
#   npz 정책은 시스템 python3 로 (Jetson 과 같음), pt 는 Isaac 파이썬
# 환경변수 추가: DOCK_TOL (접근 성공 거리, 기본 0.02), NAV_BACK · HANDOFF (run_dock_goals.py, 기본 0.025 · 0.40),
#   BRIDGE_EXTRA (브리지 인자, 예: '--viz kit' = Isaac 창 띄워 직접 보기), NAV_EXTRA · DOCK_EXTRA (노드 인자 더하기, 예: DOCK_EXTRA='--no_settle --min_cmd_lin 0 --min_cmd_ang 0')
# 환경변수 (기본값):
#   MOVEIT_SIM  팀 로봇 시뮬 폴더 (Project2_moveit/sim/robot_sim, TN_ROBOT=team · sim_nav_bridge --real_speed 가 있는 곳)
#   NAV_WS      navigation 패키지 (cmd_vel_hold) 를 빌드한 install 폴더 — 필수
#   OUT         결과 폴더 (영상 · 로그 · res_<이름>.json), 기본 rl/data/local_planner/isaac
#   ROS_DOMAIN_ID 기본 23 (다른 시험과 안 겹치게), 항상 LOCALHOST
#   PLANNER_ID  전역 경로 플러그인 (기본 "" = GridBased NavFn). Smac2D · Smac2DFar 면 EXTRA_PARAMS 에 rl/tasks/local_planner_nav2/smac2d_planners.yaml
#   EXTRA_PARAMS config/nav2.yaml 뒤에 읽을 yaml (쉼표, 기본 없음)
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
SIM="$(cd "$HERE/../../../.." && pwd)"                      # sim/robot_sim
REPO="$(cd "$SIM/../.." && pwd)"
MOVEIT_SIM="${MOVEIT_SIM:-$REPO/../Project2_moveit/sim/robot_sim}"
NAV_WS="${NAV_WS:?navigation 패키지를 빌드한 install 폴더를 NAV_WS 로 줄 것}"
OUT="${OUT:-$SIM/rl/data/local_planner/isaac}"
POL=$(realpath "$1"); CFG=$(realpath "$2"); DPOL=$(realpath "$3"); DCFG=$(realpath "$4"); TAG=$5; SECS=${6:-330}
mkdir -p "$OUT"
set +u; source /opt/ros/lyrical/setup.bash; source "$NAV_WS/setup.bash"; set -u   # ROS setup 은 없는 변수를 씀
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-23} ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST TN_ROBOT=team
SHIM="$(mktemp -d "${TMPDIR:-/tmp}/ros_shim.XXXX")"            # Isaac 파이썬에 시스템 rclpy 를 붙일 때 필요한 em · lark (sim_nav_bridge.sh 와 같음)
ln -s /usr/lib/python3/dist-packages/em.py "$SHIM/em.py"; ln -s /usr/lib/python3/dist-packages/lark "$SHIM/lark"
BR= NP= NODE= NODE2=
RUN="rl_${TAG}_$$"                     # 브리지 표식 (환경변수, 자식이 물려받음). 예전 kill -$BR 은 setsid 로 새 그룹이 된 브리지를 못 끔 (2026-11-05)
bridge_pids() {
  for p in $(pgrep -u "$USER" .); do
    [ "$p" = "$$" ] && continue
    tr '\0' '\n' < /proc/$p/environ 2>/dev/null | grep -qx "NT_RUN_MARK=$RUN.bridge" && echo $p
  done; true
}
cleanup() {
  [ -n "$NODE2" ] && kill -INT $NODE2 2>/dev/null; [ -n "$NODE" ] && kill -INT $NODE 2>/dev/null; [ -n "$NP" ] && kill -INT -$NP 2>/dev/null; sleep 4
  [ -n "$NP" ] && kill -9 -$NP 2>/dev/null; [ -n "$NODE" ] && kill -9 $NODE 2>/dev/null; [ -n "$NODE2" ] && kill -9 $NODE2 2>/dev/null
  local b; b=$(bridge_pids); [ -n "$b" ] && kill -INT $b 2>/dev/null; sleep 8; b=$(bridge_pids); [ -n "$b" ] && kill -9 $b 2>/dev/null
  rm -rf "$SHIM"
}
trap cleanup EXIT
# 1. 브리지 (실물 속도 한계, 경기장 전체를 남쪽 위에서 비스듬히 녹화)
(export NT_RUN_MARK="$RUN.bridge"; cd "$MOVEIT_SIM" && setsid scripts/sim_nav_bridge.sh --signal_delay -1 --real_speed --record "$OUT/$TAG.mp4" --record_secs $SECS \
   --record_view=0.0,-6.2,6.8,0.0,0.1,0.0 ${BRIDGE_EXTRA:-} > "$OUT/bridge_$TAG.log" 2>&1) & BR=$!
timeout 400 bash -c "until grep -qE '^\[bridge\] 경기장|Traceback' '$OUT/bridge_$TAG.log'; do sleep 2; done" || { echo BRIDGE_FAIL; exit 1; }
# 2. Nav2 일부 (활성화가 가끔 시간 초과로 실패 → 두 관리자 다 활성까지 최대 3 번)
for try in 1 2 3; do
  setsid ros2 launch "$HERE/rl_nav.launch.py" params:="$REPO/config/nav2.yaml" map:="$MOVEIT_SIM/maps/maze_dice_map.yaml" \
     urdf:="$MOVEIT_SIM/robot_description/team/robot.urdf" initial_pose:=-1.468,-2.202,180 ${EXTRA_PARAMS:+extra_params:="$EXTRA_PARAMS"} > "$OUT/nav_$TAG.log" 2>&1 & NP=$!
  if timeout 60 bash -c "until [ \$(grep -c 'Managed nodes are active' '$OUT/nav_$TAG.log') -ge 2 ]; do sleep 1; done"; then break; fi
  echo "NAV2 retry $try"; kill -INT -$NP; sleep 5; kill -9 -$NP 2>/dev/null; NP=
done
[ -z "$NP" ] && { echo NAV2_FAIL; exit 1; }
# 3. 정책 노드 둘 (npz = 시스템 python3, pt = Isaac 파이썬 + 시스템 Lyrical rclpy). exec → $! = 파이썬 자체 (정리 때 꺼짐)
pyrun() { case "$1" in *.npz) shift; exec python3 "$@";; *) shift; exec env -u PYTHONPATH PYTHONPATH="/opt/ros/lyrical/lib/python3.12/site-packages:$SHIM" "${ISAACLAB_PY:-$HOME/pa-course/IsaacLab-6.2/.venv/bin/python}" "$@";; esac; }
pyrun "$POL" "$HERE/local_planner_node.py" --policy "$POL" --cfg "$CFG" --sim_time --planner_id "${PLANNER_ID:-}" \
   --action_name navigate_to_pose --tol_pos 0.05 --tol_yaw_deg 5 --near_disable_polygons PolygonStop,FootprintApproach --near_ok_s 0 ${NAV_EXTRA:-} > "$OUT/node_$TAG.log" 2>&1 & NODE=$!
pyrun "$DPOL" "$HERE/local_planner_node.py" --policy "$DPOL" --cfg "$DCFG" --sim_time --planner_id "${PLANNER_ID:-}" \
   --action_name dock_to_pose --tol_pos "${DOCK_TOL:-0.02}" --tol_yaw_deg 2 --disable_polygons PolygonStop,FootprintApproach \
   --goal_frame odom --box_track ${DOCK_EXTRA:-} > "$OUT/dock_$TAG.log" 2>&1 & NODE2=$!
for f in node dock; do
  timeout 60 bash -c "until grep -q '정책' '$OUT/${f}_$TAG.log'; do sleep 1; done" || { echo NODE_FAIL $f; tail "$OUT/${f}_$TAG.log"; exit 1; }
done
sleep 2
# 4. 10 곳 두 단계 주행 (정답 자세로 잼)
timeout 1500 python3 "$HERE/run_dock_goals.py" "$OUT/res_$TAG.json" 2>&1 | tee "$OUT/goals_$TAG.log"
# 5. 녹화 끝 기다림
timeout 600 bash -c "until grep -q '녹화 끝' '$OUT/bridge_$TAG.log'; do sleep 3; done"
grep '녹화 끝' "$OUT/bridge_$TAG.log"
echo DONE $TAG
