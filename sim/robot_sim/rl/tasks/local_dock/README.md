# local_dock — 정밀 접근 정책 (박스 · 장소 면 앞 2 cm · 2°)

- 목적: 주행 (MPPI 또는 nf 정책) 뒤 마지막 단계. MPPI 는 Isaac 에서 4–10 cm 짧게 서고 (목표 허용 5 cm), 0.5 m 앞 yaw 오차가 최대 130° → 박스 2 cm 정차가 안 됨
- 원칙: **방향은 먼 곳 (0.3 m 밖) 에서 먼저 맞추고, 마지막 0.3 m 는 직선 접근** (박스 앞 제자리 회전 = 모서리가 박스에 닿음)
- env · 관측 161 · 실물형 라이다 · 플랜트 · DR = `local_planner_nav2` 그대로 (`local_planner/env.py`)

## 바뀐 것 (`cfg.py`)

| 항목 | 값 | 이유 |
|---|---|---|
| 구간 | `make_dock_legs.py`: 정차 면 22 곳 × 300 = 6600, 출발 → 정차점 직선 | |
| 출발 거리 | 50 % 0.03–0.15 m (MPPI 가 서는 자리) · 50 % 0.15–0.8 m | |
| 출발 옆 · yaw | 옆 N(0, 0.05) ±0.15 m · yaw 50 % ±10° · 50 % ±60° | MPPI 0.5 m 앞 yaw 오차 0–130° |
| 속도 상한 | 0.08 m/s · 0.3 rad/s | 박스 바로 앞, 지연 0.24 s 면 2 cm |
| 방향 보상 | 늘 켬, 가중 1.0 (주행 정책은 2.0–0.8 m 만 · 0.5) | 방향 먼저 |
| 목표 0.3 m 안 회전 벌점 `rot_near` | −0.05 × \|wz\| / 0.3 | 마지막 0.3 m 직선 접근 |
| 멈춤 벌점 · 회전 비용 | 끔 | 0.3 m 밖 제자리 방향 맞추기 허용 |
| 진행 보상 | 10 (5 → 10) | 구간이 짧음 (중앙 0.15 m) |
| 사람 | 마지막 단계 10 % | 박스 바로 앞이라 드묾 |
| 학습 | 1500 iter (약 1.5 시간) | |

## 실행

```bash
cd sim/robot_sim
PY="env -u PYTHONPATH $HOME/pa-course/IsaacLab-6.2/.venv/bin/python"
python3 rl/tasks/local_dock/make_dock_legs.py                      # 구간 (이미 만듦, 정차 면 바뀌면 다시)
$PY rl/train.py --task local_dock --run_name dock1                  # 학습
$PY rl/tasks/local_dock/eval_dock.py rl/logs/local_dock/<run>/model_1500.pt   # 평가 (출발 거리 · yaw 별)
$PY rl/tasks/local_planner/export.py rl/logs/local_dock/<run>/model_1500.pt   # .pt · .npz
```

- `eval_dock.py`: 성공 · 충돌 · 시간 · 도착 오차 · **마지막 0.3 m 안 회전량** (작을수록 직선 접근), 출발 0.03–0.15 m (MPPI 자리) · yaw > 10° 묶음 따로
- 배포: `local_planner/deploy/local_planner_node.py --policy <dock>.npz --cfg <dock run>/cfg.json --action_name dock_to_pose --tol_pos 0.02 --tol_yaw_deg 2 --disable_polygons PolygonStop,FootprintApproach --goal_frame odom --box_track` (주행의 `navigate_to_pose` 와 안 겹침)
  - planning (2026-11-06 구현): 박스 · 장소 이동 단계 = Nav2 목표 = 정차점 뒤 2.5 cm (nf 학습 구간 끝과 같음), 정차점 0.40 m 안에서 Nav2 취소 → `dock_to_pose` 로 정차점 (`config/planning.yaml` `dock_action` · `dock_nav_back_m` · `dock_handoff_m`)
  - 서지 않고 넘김 → dock2 는 vmax 0.2 + 움직이며 출발 50 % (`start_speed_prob`), pre 모드 출발 = 정차점 뒤 0.30–0.45 m (넘김 자리)
  - `--box_track` (2026-11-06): 스캔마다 박스 앞면 직선 맞춤 → 목표 = 잰 박스 중심에서 0.2575 m 정면 (`deploy/box_tracker.py`). SLAM 지도와 시연 박스 자리가 달라도 원하는 틈에 섬. 목표 0.10 m 안은 고정 (면이 몸 상자 · range_min 안). 가짜 스캔 시험 300 회 (박스 ±6 cm · ±10°, 잡음 5–15 mm, 옆 벽 반): 목표 오차 중앙 1.6 · 90 % 5.3 mm, 틈 0.77–1.42 cm (목표 1 cm). **실물 확인 필요: 라이다 스캔면 60 mm ↔ 박스 높이 70 mm (1 cm 차) → /scan 에 박스가 잡히는지**
  - `--goal_frame odom`: 목표를 받는 순간 map → odom 으로 한 번 바꿔 고정 → 접근 중 slam_toolbox 가 map → odom 을 고쳐도 목표가 안 튐. 경로 · 판정도 odom 기준

## dock2 (2026-11-06): 박스 밀림 · 짧은 구간 · 틈 2–4 cm

planning 쪽 요청 반영. 구간 `dock_legs_v2.npz` (dock1 은 `dock_legs.npz` 그대로 → dock1 평가 영향 없음)

| 요청 | 반영 |
|---|---|
| 1. 박스 밀림 + 같이 움직인 목표 | `env.box_dynamic`: 리셋 때 70 % 확률로 박스 ±6 cm · ±10° 밈 (벽 · 출발과 겹치면 안 밂 → 실제 약 64 %). 목표는 박스에 붙여 같이 옮김 (상대 자세 오차 0 mm 확인). 충돌 · 여유 · 라이다 = 밀린 박스 (`World.enable_boxes`: 그 박스 칸 지운 거리장 + 회전 사각형 sdf), 지도 · 출발 = 원래 자리, 경로 = 출발 → 옮긴 목표 직선 |
| 2. 짧은 구간 | 출발 모드 pre 35 % (0.25–0.35 m 앞, correct 모드) · fix 35 % (−1–4 cm, ±3 cm · ±5° = 측정 뒤 몇 cm 고치기, 박스 밀림과 겹쳐 ±6 cm · ±10°) · broad 30 % (옛 분포). 구간 길이 중앙 0.10 m |
| 3. 경로 계획 실패 | 배포 노드 `--straight_max 0.6`: planner 실패 + 목표 0.6 m 안 → 지금 자리 → 목표 직선 경로 (다시 계획할 때도 실패면 직선 유지) |
| 4. 목표마다 도착 기준 | 이미 나눔: 노드 두 개 — 주행 `--action_name navigate_to_pose --tol_pos 0.05 --tol_yaw_deg 5`, 접근 `--action_name dock_to_pose --tol_pos 0.02 --tol_yaw_deg 2` |
| 5. 틈 2–4 cm | 구간마다 gap U(2, 4) cm → 정차 거리 0.2825–0.3025 m (planning 0.2935 포함) |

- `eval_dock.py` 에 모드별 (pre · fix · broad) · 박스 밀림 여부 묶음 추가
- 학습: `$PY rl/train.py --task local_dock --run_name dock2` (nf3 의 목표 근처 속도 상한 · 조향 벌점 · 명령 크기 벌점도 물려받음)
