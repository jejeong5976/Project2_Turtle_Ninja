# 강화학습 (스워브 주행)

`sim_core/` (스워브 컨트롤러 `swerve_torch.py` 등) 위에서 학습하는 코드. 학습은 torch 간이 플랜트 (Isaac 안 띄움), 검증은 Isaac 브리지.

학습 과제 = 배포 정책 넷 (설명 = 각 cfg.py 머리):

| 과제 | 내용 | 배포 |
|---|---|---|
| `local_planner_nf5` | 주행: 전역 경로 · 목표 · 라이다 → 몸 속도, 목표 앞 정지 막기 · PolygonStop 구역 | `config/rl/nav_nf5` |
| `local_planner_nf7` | nf5 + 헛회전 · 회전 반전 벌점 + 실물 차이 랜덤화 (arena5 구간 · 박스 어긋남 · yaw 흐름 · 큰 튐) | `config/rl/nav_nf7` |
| [`local_dock3`](tasks/local_dock3/README.md) | 정밀 접근: 박스 · 장소 면 앞 0.03–0.8 m → 정차점, 성공 1 cm | `config/rl/dock3` |
| `local_dock5` | dock3 + 회전하며 넘겨받기 + 박스 크게 어긋남 · 측정기 흉내 · 앞쪽 라이다 | `config/rl/dock5` |

바탕 (학습 과제 아님, 위 넷이 상속 · 데이터 도구):

| 폴더 | 내용 |
|---|---|
| [`local_planner`](tasks/local_planner/README.md) | env · 보상 · DR · 플랜트 연결, 평가 `play.py`, 배포 · Isaac 검증 `deploy/` |
| [`local_planner_nav2`](tasks/local_planner_nav2/README.md) | 실제 Nav2 planner_server 경로 · dice 박스 네 면 정차 · 실물형 라이다 가림표 |
| [`local_dock`](tasks/local_dock/README.md) | 정밀 접근 구간 (`make_dock_legs.py`) · 평가 `eval_dock.py` |
| `swerve_follow` | 간이 플랜트 · 경로 묶음 · 기준 추종기 |

## 구조

```
rl/
├── train.py                  # 공통 학습 (--task <과제>)
├── tasks/
│   ├── swerve_follow/        # 공용 라이브러리: plant.py (간이 플랜트) · paths.py (경로 묶음) · follower.py (기준 추종기)
│   └── <과제>/               # 과제 하나 = 폴더 하나 (cfg.py 는 부모 과제 cfg 를 상속)
├── data/                     # (gitignore) 학습 입력
│   ├── maps/                 #   미로 점유 지도 (tools/layout_to_occupancy.py)
│   ├── path_bank.npz         #   경로 은행 (tools/make_path_bank.py)
│   ├── team_swerve.yaml      #   팀 로봇 swerve.yaml 복사본
│   └── <과제>/               #   과제 전용 (dice 구간 · 경로 은행 · 가림표 · arena5 구간)
└── logs/<과제>/<시각>_<이름>/ # (gitignore) 배포 정책 run 만 둠 (체크포인트 · TensorBoard · cfg.json · exported)
```

- 실행 스크립트는 맨 위에서 `sim/robot_sim` 과 `rl/tasks` 를 `sys.path` 에 넣음 → `from sim_core.swerve_torch import ...`, `from swerve_follow.paths import ...`

## 데이터 만들기 (새로 받은 뒤)

```bash
cd sim/robot_sim
PY="env -u PYTHONPATH $HOME/pa-course/IsaacLab-6.2/.venv/bin/python"
python3 tools/layout_to_occupancy.py --maze dice --out_dir rl/data/maps      # 지도 (real · rand_00..07 도 같은 식, make_path_bank 가 없으면 만듦)
$PY tools/make_path_bank.py                                                   # 공용 경로 은행
python3 rl/tasks/local_planner/make_dice_legs.py                              # local_planner (레포 maps/dice_map.yaml 정차점)
bash rl/tasks/local_planner_nav2/run_make_paths.sh                            # local_planner_nav2 (실제 planner_server 경로 · 네 면 구간, 시스템 Lyrical)
```

## 실행

```bash
$PY rl/train.py --task <과제> [--num_envs N] [--max_iterations N] [--run_name 이름]
$PY rl/tasks/local_planner/play.py --checkpoint rl/logs/<과제>/<실행>/model_<번호>.pt
```
