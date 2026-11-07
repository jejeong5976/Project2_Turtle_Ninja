# 강화학습 (스워브 주행)

`sim_core/` (스워브 컨트롤러 `swerve_torch.py` 등) 위에서 학습하는 코드. 학습은 torch 간이 플랜트 (Isaac 안 띄움), 검증은 Isaac 브리지.

| 과제 | 내용 | 상태 |
|---|---|---|
| [`swerve_follow`](tasks/swerve_follow/README.md) | Nav2 경로 → 몸 속도 잔차 (기준 추종기 + 잔차) | 본학습 1 개선 없음 (2026-10-30) |
| [`swerve_track`](tasks/swerve_track/README.md) | MPPI `/cmd_vel` → 몸 속도 잔차 | 개선 1–3 % (2026-11-02) |
| [`local_planner`](tasks/local_planner/README.md) | MPPI 대체: 전역 경로 · 목표 · 라이다 → 몸 속도 직접 | 학습 중 (2026-11-04) |
| [`local_dock`](tasks/local_dock/README.md) | 정밀 접근: 박스 · 장소 면 앞 0.03–0.8 m → 2 cm · 2°, 방향 먼저 · 마지막 0.3 m 직선 (MPPI · nf 뒤에 붙임) | 코드 (2026-11-05) |
| [`local_planner_nav2`](tasks/local_planner_nav2/README.md) | `local_planner` 와 같은 env, 경로 = 실제 Nav2 planner_server (NavFn · Smac2D), dice 박스 네 면 정차 · 면 고르기 | 준비 (2026-11-04) |

## 구조

```
rl/
├── train.py                  # 공통 학습 (--task <과제>)
├── tasks/
│   └── <과제>/               # 과제 하나 = 폴더 하나
│       ├── cfg.py            # 환경 · DR · 보상 · PPO
│       ├── env.py            # rsl_rl VecEnv
│       ├── play.py           # 평가 (기준선 vs 정책)
│       ├── check_env.py      # 환경 확인 (있으면)
│       └── README.md         # 설계 · 결과
├── data/                     # (gitignore) 만든 데이터 · 평가 결과
│   ├── maps/                 #   공용: 미로 점유 지도 (tools/layout_to_occupancy.py)
│   ├── path_bank.npz         #   공용: 경로 은행 (tools/make_path_bank.py) — swerve_follow · local_planner
│   ├── team_swerve.yaml      #   공용: 팀 로봇 swerve.yaml 복사본 — local_planner
│   └── <과제>/               #   과제 전용 (명령 은행 · dice 구간 · 평가 json · 그림 · 학습 로그)
└── logs/<과제>/<시각>_<이름>/ # (gitignore) 체크포인트 · TensorBoard · cfg.json
```

- 과제끼리 공유하는 코드: `swerve_follow/plant.py` (간이 플랜트) · `paths.py` (경로 묶음) · `follower.py` (기준 추종기) 를 `swerve_track` · `local_planner` 가 가져다 씀
- 실행 스크립트는 맨 위에서 `sim/robot_sim` 과 `rl/tasks` 를 `sys.path` 에 넣음 → `from sim_core.swerve_torch import ...`, `from swerve_follow.paths import ...`

## 데이터 만들기 (새로 받은 뒤)

```bash
cd sim/robot_sim
PY="env -u PYTHONPATH $HOME/pa-course/IsaacLab-6.2/.venv/bin/python"
python3 tools/layout_to_occupancy.py --maze dice --out_dir rl/data/maps      # 지도 (real · rand_00..07 도 같은 식, make_path_bank 가 없으면 만듦)
$PY tools/make_path_bank.py                                                   # 공용 경로 은행
$PY tools/make_cmd_bank.py                                                    # swerve_track
python3 rl/tasks/local_planner/make_dice_legs.py                              # local_planner (레포 maps/dice_map.yaml 정차점)
bash rl/tasks/local_planner_nav2/run_make_paths.sh                            # local_planner_nav2 (실제 planner_server 경로 · 네 면 구간, 시스템 Lyrical)
cp <Project2_moveit>/sim/robot_sim/robot_description/team/config/swerve.yaml rl/data/team_swerve.yaml   # local_planner
```

## 실행

```bash
$PY rl/train.py --task <과제> [--num_envs N] [--max_iterations N] [--run_name 이름]
$PY rl/tasks/<과제>/play.py --checkpoint rl/logs/<과제>/<실행>/model_<번호>.pt
```
