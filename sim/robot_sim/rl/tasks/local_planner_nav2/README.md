# local_planner_nav2 — 실제 Nav2 경로 · 네 면 정차로 지역 계획기 학습

`local_planner` 와 같은 env · 보상 · 관측 (`local_planner/env.py`). 바뀐 것은 셋.

1. **경로 = 실제 planner_server**: `local_planner` 경로 은행은 NavFn 파이썬 흉내 (`tools/make_path_bank.py`). 여기선 같은 시작 · 목표 쌍을 실제 `compute_path_to_pose` 에 planner 별로 요청 → 학습 입력 = 배포 때 입력.
2. **dice 정차 = 박스 · 장소 네 면**: 출발 + B1–B6 · L1–L3 의 N · E · S · W 앞 자리 중 설 수 있는 것끼리 모든 구간. 정차 거리 = 박스 면 ↔ Nav2 외곽 2 cm.
3. **행동 std 상한** (튜터 PPO 글 8.5): `std_type: log`, `std_range [0.05, 0.8]`.
4. **라이다 = 실물 · 배포와 같은 처리** (`lidar_model: real`): 앞 · 뒤 LDS-03 원점에서 빔 (2° 간격) → 자기 몸 가림 (팀 URDF 단면, Isaac 브리지 `lidar_self.py` 와 같은 모델, 조향각별) → scan_merger 몸 상자 ±0.17 × ±0.19 안 점 버림 → 몸 중심 10° 칸 최솟값. 배포 노드는 `/scan` (scan_merger) 을 받음

## 파일

| 파일 | 내용 |
|---|---|
| `stops.py` | 정차 후보 (네 면, `--gap` 기본 0.02 → 박스 중심에서 0.2825 m), 벽에 닿는 면 뺌, 별칭 `B1 = B1.S` (= `maps/dice_map.yaml` 지금 면) |
| `smac2d_planners.yaml` | planner_server 에 `Smac2D` (cost_travel_multiplier 2.0) · `Smac2DFar` (5.0) 추가. `GridBased` (NavFn) 는 `config/nav2.yaml` 그대로 |
| `paths_only.yaml` | 경로 만들기 전용: 실제 시계, global_costmap = static + inflation (라이다 층 뺌) |
| `paths.launch.py` | map_server · planner_server · 정적 TF map → base_link |
| `make_nav2_paths.py` | 지도마다 `load_map` → 쌍마다 planner 별 경로 + 흉내 경로 (같은 쌍) → 은행 · dice 구간 |
| `run_make_paths.sh` | 위 둘 한 번에 (시스템 Lyrical 파이썬, `ROS_DOMAIN_ID=17` + LOCALHOST) |
| `compare_paths.py` | 경로 모양 비교 (길이 · 벽 여유 · 꺾임 · 흉내 ↔ 실제 NavFn 거리), `--plot` |
| `cfg.py` | `local_planner` Cfg + `paths` (navfn · smac2d · smac2d_far · imitation) + std 상한 |
| `eval_legs.py` | dice 구간 전부 주행 시간표 (정책 · 기준 추종기) → `leg_times_*.json` |
| `make_lidar_occlusion.py` | 라이다 자기 몸 가림표 (라이다 2 × 모듈 4 × 조향각 72 칸 × 빔 360) `lidar_occlusion.npz` + 사각 통계 |
| `best_faces.py` | 시간표로 방문 순서마다 면 조합 최적 (DP) → 지금 면 대비 줄일 수 있는 이동 시간 |

데이터 `rl/data/local_planner_nav2/` (gitignore): `path_bank_<이름>.npz` · `dice_legs_<이름>.npz` · `stats.json` · `compare.json` · `leg_times_*.json`.

## 실행 순서

```bash
cd sim/robot_sim
PY="env -u PYTHONPATH $HOME/pa-course/IsaacLab-6.2/.venv/bin/python"
bash rl/tasks/local_planner_nav2/run_make_paths.sh                                   # 1. 경로 (약 6 분)
python3 rl/tasks/local_planner_nav2/make_lidar_occlusion.py                          #    라이다 가림표 (2 s, 팀 URDF 는 Project2_moveit)
python3 rl/tasks/local_planner_nav2/compare_paths.py --plot rl/data/local_planner_nav2/dice_paths.png   # 2. 경로 비교
$PY rl/tasks/local_planner_nav2/eval_legs.py --paths navfn                           # 3. 기준 추종기 시간표
$PY rl/tasks/local_planner_nav2/eval_legs.py --paths navfn --checkpoint rl/logs/local_planner/<lp2>/model_<N>.pt   # lp2 를 실제 NavFn 경로로
$PY rl/train.py --task local_planner_nav2 --paths navfn --run_name nf1               # 4. 학습 (smac2d 면 --paths smac2d)
$PY rl/tasks/local_planner_nav2/eval_legs.py --paths navfn --checkpoint rl/logs/local_planner_nav2/<nf1>/model_3000.pt
python3 rl/tasks/local_planner_nav2/best_faces.py rl/data/local_planner_nav2/leg_times_<nf1>_navfn.json --permute 2,3   # 5. 면 고르기
$PY rl/tasks/local_planner/play.py --checkpoint <...> --dice_legs rl/data/local_planner_nav2/dice_legs_navfn.npz \
    --path_bank rl/data/local_planner_nav2/path_bank_navfn.npz                       # 옛 10 구간 경로 (별칭으로 지금 면)
```

Isaac 검증 (Smac2D 로): `PLANNER_ID=Smac2D EXTRA_PARAMS=<레포>/sim/robot_sim/rl/tasks/local_planner_nav2/smac2d_planners.yaml rl/tasks/local_planner/deploy/run_isaac.sh ...` (기본 = NavFn, 그대로).

## NavFn vs Smac2D

경로 모양 (같은 쌍, `compare_paths.py`, 결과는 아래 "결과" 표). 판단 기준:

| 항목 | 좋은 쪽 | 이유 |
|---|---|---|
| 벽 근처 비율 · 평균 여유 | 큰 여유 | 회전할 자리, 충돌 · collision_monitor 정지 |
| 총 꺾임 · 급한 모서리 수 | 적음 | 조향 다시 맞추기 = 멈춤 (튜터 '회전하다 멈춤') |
| 길이 | 짧음 | 시간 |
| 계획 실패 · 시간 | 적음 | 1 s 마다 다시 계획 |

경로 모양만으로 정하지 않음 → 같은 정책 구조로 `--paths navfn` · `--paths smac2d` 각각 학습 → `eval_legs.py` 시간 · 성공 · 조향 이동으로 정함. 차이가 작으면 NavFn 유지 (배포 설정 그대로, 바꿀 것 없음).

## 결과

### 경로 모양 (2026-11-04, `compare_paths.py`, 같은 쌍 중앙값)

| 묶음 | 항목 | 흉내 | NavFn | Smac2D | Smac2DFar |
|---|---|---|---|---|---|
| 학습 지도 4800 | 평균 여유 m | 0.324 | 0.328 | 0.314 | 0.314 |
| | 벽 근처 비율 p90 | 0.000 | 0.014 | 0.094 | 0.094 |
| | 총 꺾임 rad (p90) | 3.43 (7.08) | 4.88 (9.73) | 4.67 (8.39) | 4.68 (8.42) |
| dice 505 구간 | 길이 m | 3.95 | 4.10 | 4.05 | 4.05 |
| | 평균 여유 m | 0.341 | 0.340 | 0.323 | 0.321 |
| | 벽 근처 비율 p90 | 0.125 | 0.129 | 0.247 | 0.224 |
| | 최대 꺾임 ° (p90) | 84 (94) | 78 (91) | 65 (82) | 84 (106) |
| 계획 시간 | 중앙 (p95) ms | — | 0.4 (0.9) | 1.3 (4.7) | 1.3 (3.3) |

- Smac2D = 직선 구간으로 펴서 꺾임 p90 · 최대 꺾임은 작지만 내접 영역 (0.17) 경계를 따라 벽에 더 붙음 (벽 근처 비율 dice p90 0.13 → 0.25). `cost_travel_multiplier` 5.0 도 거의 같음 (팽창 cost_scaling 10 → 0.3 m 밖 비용 거의 0)
- **흉내 ↔ 실제 NavFn**: 평균 거리 중앙 3–5 cm, p90 7–12 cm, 최대 74–95 cm (다른 쪽으로 돈 경로). 실제 NavFn 총 꺾임이 흉내의 1.3–1.4 배 → lp1 · lp2 는 실제보다 매끈한 경로로 학습함
- 판단: 경로 모양으로는 우열 없음 (Smac2D = 꺾임 ↓ · 벽 여유 ↓). 정하는 건 `--paths navfn` / `--paths smac2d` 학습 뒤 `eval_legs.py` 시간 · 성공 · 조향 이동. 차이 작으면 NavFn (배포 설정 그대로)

### 정차 후보 (gap 2 cm)

- 27 개 중 벽에 닿는 면 10 개 (B1.N · B1.W · B2.N · B2.W · B3.W · B4.E · B5.E · B6.S · L1.N · L3.W), 출발점에서 경로 없는 면 4 개 (B3.E · B5.S · B6.E · B6.N, 들어가는 길이 0.34 m 보다 좁음) → **23 개 (출발 + 면 22), 구간 505**
- 남은 면: B1 S·E / B2 S·E / B3 S·N / B4 S·N·W / B5 W·N / B6 W / L1 W·S·E / L2 N·E·S·W / L3 S·N·E

### 라이다 (2026-11-04, `make_lidar_occlusion.py`, 팀 URDF)

- 라이다 원점 앞 (+0.115, 0) · 뒤 (−0.115, 0), 고정 부분 (차체 · 기둥 · 반대쪽 라이다) 에 가린 빔 각 4 %
- 합친 스캔 10° 칸 36 개 중 두 라이다 다 못 보는 칸 (반경 R 원 위):

| 조향 | R 0.3 m | 0.5 m | 1.0 m | 2.0 m |
|---|---|---|---|---|
| 0° | 2 | 0 | 0 | 0 |
| 90° | 4 | 0 | 0 | 0 |
| 45° | 4 | 0 | 0 | 0 |
| 제자리 회전 (X 자) | 4 | 2 | 0 | 0 |
| 무작위 100 평균 | 2.6 | 0.3 | 0 | 0 |

- 두 라이다가 서로 가린 곳을 채워 사각은 0.5 m 안에만 생김 → 무작위 사각 DR 대신 가림표 그대로 씀
- 학습 env 8192 개 비교: ideal 과 다른 칸 = 몸 상자 안 1.4 % · 몸 가림 2.1 % (둘 다 0.5 m 안), 몸 상자 밖인데 빈 칸 0 %. 칸 최솟값이라 28 % 칸이 ideal 보다 5 cm 넘게 가까움 (배포와 같음)
- 속도: 8192 env 한 번 22.5 ms (compile, ideal 8.5 ms) → iteration 약 3.6 s (lp2 2.56 s), 3000 회 약 3 시간

### 시간표 흐름 시험 (기준 추종기, 실제 수치 아님)

`eval_legs.py --paths navfn` (기준 추종기는 장애물 안 피함 → 성공 65 %) → `best_faces.py --min_success 0.5`: 주어진 순서 면만 바꾸면 −9.6 s (L2.N → L2.E), 배달 · 재검증 순서까지 −61.6 s, 수집 순서까지 25920 가지 −63.4 s. 정책 시간표로 다시 볼 것

## 튜터 PPO 글 (velog 'R12 PPO 및 미션 제어 구조') 과 비교

| 글 | 우리 (local_planner) | 반영 |
|---|---|---|
| 행동 클리핑 포화 → 포화율 기록, 필요하면 bounded 분포 | lp2 std 0.24 → 16 (3000 iter 중 2500), 샘플 행동 대부분 ±1 로 잘림, 성공 86 → 82 % | std 상한 (이 과제), `/act/sat_*` 기록 (env 공용) |
| 보상 항별 · 종료 원인별 기록 | 종료 원인 · 에피소드 값만 | `/rew/*` 항별 평균 (env 공용) |
| 학습 = 샘플, 평가 = 평균 행동 | `get_inference_policy` = 평균 | 그대로 |
| timeout 은 bootstrap (실패 종료와 구분) | `extras['time_outs']` = 시간 초과 | 그대로 |
| 정답 상태는 보상 · 평가 · critic 만, actor 관측 금지 | actor = 추정 자세 (지도 오차 DR), critic 만 참 자세 | 그대로 |
| 기준 제어와 같은 조건 비교 | `play.py` · `eval_legs.py` 기준 추종기 | 그대로 |
| 조향 오차 한 바퀴로 전체 정지 → 바퀴별 cos 감속 | 학습 = 연속 조향 (π) + cos 감속. 실물 `config/controllers.yaml` = `steer_then_drive: true` · 0.17 (전체 정지) | control 확인 필요 (아래) |

## 주의 · 남은 것

- **collision_monitor `PolygonStop` ±0.18 m**: 박스 면 2 cm 정차면 박스가 로봇 중심에서 0.18 m → 정차 때 비상 정지에 걸림. 2 cm 정차를 쓰려면 ±0.17 이하로 줄여야 함 (`config/nav2.yaml`, 안 바꿈)
- **내접 반경 0.17** (외곽 0.16 + padding 0.01): 2 cm 정차 칸 일부가 2.5 cm 지도 칸 오차로 내접 영역에 듦 → planner `tolerance` 0.1 로 근처 칸까지 경로, 끝 몇 cm 는 지역 계획기가 감. 1 cm 정차면 padding 0 필요
- **박스 E · W 면**: 퍽 2 개가 박스 X ±40 mm 라 E · W 에서 보면 앞뒤 한 줄 → 먼 퍽 팔 도달 control 확인 필요. 장소는 주사위 방향 무관 (튜터 2026-11-04)
- **planning 은 아직 한 면만 씀** (`dice_map.yaml` stop 하나 · `dice_travel_times.yaml` 10 × 10). 면을 고르려면 노드 × 면 시간표 · 정차 선택이 planning 에 필요 (best_faces 로 이득 확인 뒤)
- 실물 스워브 `steer_then_drive` 0.17 이면 학습 (연속 조향) 과 다름 → control 담당 확인
- 라이다 가림표는 Isaac 검증 로봇 (Project2_moveit 팀 URDF) 기준. 실물 장착이 다르면 다시 만들 것. 위치 추정 DR (`map_err_*`) 은 아직 실물 slam_toolbox 로 안 맞춤 (bag 필요)
- 몸 상자 ±0.19 안 옆 장애물은 실물 · 학습 둘 다 안 보임 (scan_merger 가 버림)

## nf2 → nf3 (2026-11-06): 도착 직전 흔들림

- nf2 B2 → B6 마지막 3.5 s: 20 cm 앞에서 이동 방향을 옆 → 앞 → 옆 으로 바꿔 몸 4–20 cm 가는 동안 바퀴 165° 쓸림, 도착 (1–2 cm) 뒤 최고 속도 명령 (0.14–0.19 m/s) 을 반대 방향으로 번갈아 내 몸은 서 있고 바퀴만 80° 돎
- 원인: 도착 근처 명령 크기를 줄일 이유가 없음 (성공 +20 대비 행동 변화 0.01 · 조향 0.02/rad 벌점이 작음)

| 추가 (`local_planner/cfg.py`, 기본 끔 → 이 과제에서 켬) | 값 |
|---|---|
| `approach_limit` (고정 규칙, 배포 노드 `command()` 같은 식) | 병진 ≤ 0.02 + 0.5 × 목표 거리 (2 cm 0.03 · 10 cm 0.07 · 36 cm 0.2 m/s) |
| `steer_near_mult` · `steer_near_dist` | 목표 0.3 m 안 조향 이동 벌점 × 10 |
| `hold_cmd` · `hold_dist` | 목표 3 cm 안 −0.05 × ((병진 / vmax)² + (wz / wmax)²) |

- 로그 `/act/approach_capped` · `/rew/hold_cmd`. `local_dock` 도 이 설정을 물려받음 (dock2 부터)
- 평가 스크립트 (eval_legs · eval_dock) 는 저장된 cfg.json 에 없는 항목을 꺼진 기본값으로 → nf2 · dock1 은 그 학습 조건 그대로 평가
