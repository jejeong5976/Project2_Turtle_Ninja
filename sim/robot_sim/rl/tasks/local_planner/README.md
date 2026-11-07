# local_planner — MPPI 대체 지역 계획기 RL

Nav2 전역 경로 (NavFn) · 목표 자세 · 라이다 · 바퀴 상태 → 몸 속도 (vx, vy, wz) → 스워브 컨트롤러. MPPI · bt_navigator 자리에 들어감.
시작 이유 (2026-11-04): MPPI 는 목표 5 cm 앞 경계에서 멈춰 정답보다 6–10 cm 짧게 서고, 출발 → B1 처럼 벽 끝을 도는 구간에서 먼 쪽 (+270°) 으로 돎 (critic 조정으로 못 고침).

```
planner_server (/plan) ─┐
목표 (navigate_to_pose) ─┼→ [정책] → cmd_vel_nav → collision_monitor → /cmd_vel → 스워브 컨트롤러
라이다 · odom · 조향 · 바퀴 ┘
```

## 구조

| 파일 | 내용 |
|---|---|
| `cfg.py` | 환경 · DR · 보상 · PPO. 한계 = 실물 swerve_controller (0.2 m/s · 0.5 rad/s, 0.5 · 2.0) |
| `env.py` | rsl_rl VecEnv. 플랜트 · 경로 묶음 = `swerve_follow`, 컨트롤러 = `sim_core/swerve_torch.py` (팀 swerve.yaml) |
| `world.py` | 점유 격자 거리장 + 사람 원기둥 → 외곽 충돌 · 여유 · 라이다 (구 추적) |
| `check_env.py` | 무작위 행동 · 기준 추종기를 정책 자리에 넣고 결과 비율 |
| `play.py` | dice 10 구간 평가 (출발 → B1 → L1 → B2 → B6 → L3 → B3 → B5 → B4 → L2 → 출발), `--plot` 궤적 |
| `export.py` | 체크포인트 → TorchScript (관측 정규화 포함) `<실행>/exported/policy_<번호>.pt` + numpy 판 `policy_<번호>.npz` |
| `deploy/local_planner_node.py` | ROS 노드: `navigate_to_pose` 서버, 1 s 마다 `compute_path_to_pose`, 20 Hz (ROS 시계) `cmd_vel_nav`. `--policy` = `.pt` (torch) 또는 `.npz` (numpy). `--shadow` = 그림자 모드 (실물 시험: MPPI 가 주행, 정책은 `/plan` 목표로 `cmd_vel_rl_shadow` 에만 냄, `--log_csv` 로 정책 · MPPI 명령 기록). 라이다 = `/scan` (scan_merger, 2026-11-04 옛 앞 · 뒤 원본 + 반경 0.17 원 필터 → 몸 옆 · 모서리 점이 장애물로 들어가던 것 고침), 0.5 s 넘게 안 오면 정지 |
| `deploy/numpy_policy.py` | `export.py` 의 `policy_N.npz` 를 numpy 로 (torch 없음 = Jetson). TorchScript 대비 차이 3e-7 |
| `deploy/check_bag.py` | 실물 bag 오프라인 확인: 몸 상자 밖 0.30 m 안 점 (안 걸러진 몸 점) · 가림표 ↔ 실물 몸 점 · 관측 (라이다 · 바퀴 상태) 이 학습 정규화 범위 밖인지 |
| `deploy/rl_nav.launch.py` | 시험용 Nav2 일부 (지도 · AMCL · planner · collision_monitor · cmd_vel_hold · scan_merger + RSP), `extra_params` |
| `deploy/run_goals.py` | Isaac 측정: dice 정차 자세를 차례로 `navigate_to_pose` → 정답 자세로 시간 · 도착 오차 (MPPI 와 같은 스크립트로 비교) |
| `deploy/run_isaac.sh` | Isaac 검증 한 번에: 브리지 (`--real_speed`, 녹화) → Nav2 일부 → 정책 노드 → `run_goals.py` → 정리. 결과 `rl/data/local_planner/isaac/` |
| `make_dice_legs.py` | dice 정차점끼리 90 구간 `rl/data/local_planner/dice_legs.npz` (레포 `maps/dice_map.yaml` 정차점, `tools/make_path_bank.py` 의 NavFn 흉내 함수 사용) |

## 관측 · 행동 · 보상

| 항목 | 내용 |
|---|---|
| 관측 (정책 161) | 경로 앞 16 점 (0.1 m 간격, 몸 좌표) 32 · 목표 (dx/2, dy/2, sin · cos dyaw, 남은 길이/3) 5 · 라이다 36 방향 (2 m 자름) 36 · 이력 4 프레임 × (조향 sin · cos · 범위 위치, 바퀴 속도, odom, 직전 명령) 88 |
| critic 추가 | 참 몸 속도 · 경로 오차 · 목표 오차 · 여유 · 사람 거리 · DR 값 · 위치 추정 오차 · 미끄럼 · 단계 |
| 행동 | (vx, vy, wz) ∈ [−1, 1]³ × 한계, 선속도는 노름으로 자름 |
| 보상 | 경로 진행 5 × Δs · 목표 퍼텐셜 −(2 · min(거리, 0.5) + 0.5 · \|dyaw\|) 변화 · 여유 < 4 cm 벌점 (목표 0.3 m 안 제외) · 경로 이탈 · 행동 변화 · 조향 이동 0.02 / rad · 시간 0.01 |
| 끝 | 성공 +20 (추정 자세 2 cm · 2° 안 · 멈춤 0.3 s, 커리큘럼 5 → 3 → 2 cm) / 충돌 −20 (참 자세, 외곽 ±0.15) / 경로 0.6 m 이탈 −10 / 시간 초과 |
| 에피소드 | 지도 경로 (path_bank 지도 경로, 시작 yaw 무작위) 60 % · dice 구간 40 %, 사람 0–2 (단계 1 · 2) |

## 결과

| 실행 | 내용 | dice 10 구간 (간이 플랜트, env 64) |
|---|---|---|
| 기준 추종기 | `swerve_follow.follower` (장애물 안 피함) | 성공 84.5 % · 충돌 15.5 % · 150.4 s · 조향 이동 271 rad |
| lp1 400 회 | 옛 dice 지도 (중앙 받침대), 조향 벌점 0.002 | 성공 96.7 % · 충돌 3.3 % · 146.0 s · 조향 이동 397 rad (멈춤: 지도 바뀜) |
| lp2 | L2 = 205 박스 지도, 조향 벌점 0.02 | 학습 중 |

참고: Isaac MPPI (E 설정, 실물 한계) 같은 10 구간 150.5 s (2026-11-04).

## 실행

```bash
cd sim/robot_sim
$PY rl/tasks/local_planner/check_env.py --level 2
$PY rl/train.py --task local_planner --run_name 이름                     # 8192 env, 약 2.6 s/반복
$PY rl/tasks/local_planner/play.py --checkpoint rl/logs/local_planner/<실행>/model_<번호>.pt --plot out.png
$PY rl/tasks/local_planner/export.py rl/logs/local_planner/<실행>/model_<번호>.pt
python3 rl/tasks/local_planner/make_dice_legs.py                            # dice 지도 · 정차점 바뀌면
# Isaac 검증 (팀 로봇 = Project2_moveit 시뮬, navigation 패키지 빌드 필요)
NAV_WS=<ws>/install rl/tasks/local_planner/deploy/run_isaac.sh <exported/policy_N.pt> <실행>/cfg.json <이름>
```

## 실물 시험 (2026-11-04 준비)

실물 한계 = 0.2 m/s · 연속 조향 (control 확인) = 학습과 같음. 레포 `config/controllers.yaml` 의 `steer_then_drive: true` · 0.17 은 실물 실제 값이 아직 안 올라온 것으로 봄 (control 확인 필요).

```bash
# 1. bag (넓은 곳 정지 + 조향 0 · 45 · 90° · X 자 + 수동 주행)
ros2 bag record /lidar_front/scan /lidar_rear/scan /scan /joint_states /odom /tf /tf_static /cmd_vel_nav
python3 rl/tasks/local_planner/deploy/check_bag.py <bag> --policy <run>/exported/policy_N.npz --cfg <run>/cfg.json
# 2. 그림자 모드 (MPPI 주행 중, 로봇은 정책으로 안 움직임)
python3 rl/tasks/local_planner/deploy/local_planner_node.py --policy <run>/exported/policy_N.npz --cfg <run>/cfg.json --shadow --log_csv rl_shadow.csv
# 3 이후 (저속 · dice 구간): --shadow 빼고 nav2 대신 rl_nav 계열로 (collision_monitor · cmd_vel_hold 그대로)
```

- Jetson: npz 정책이면 torch 불필요 (numpy · yaml · rclpy 만). cfg.json 의 `swerve_yaml` 경로가 없으면 `--swerve_yaml` 로 줌 (`rl/data/team_swerve.yaml` 복사)
- 그림자 모드 정책 `last_cmd` = 자기 명령 (실제 주행은 MPPI) → 열린 고리 비교. 방향 · 크기 차이가 큰 구간을 보는 용도
