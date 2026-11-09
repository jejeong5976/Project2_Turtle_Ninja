# local_dock3 — 정밀 접근 dock2 + 성공 1 cm

dock2 (`../local_dock`) 와 env · 관측 · 구간 (`dock_legs_v2.npz`) · DR 같음. 바뀐 것은 `cfg.py` 머리 설명.

| | dock2 | dock3 |
|---|---|---|
| 마지막 단계 성공 | 2 cm · 2° | 1 cm · 2° |
| 명령 크기 벌점 범위 (hold_dist) | 3 cm | 1 cm |
| 반복 | 1500 | 2000 |
| 속도 상한 · 움직이며 출발 | 0.2 · 0.05–0.2 m/s | 0.255 · 0.05–0.255 m/s |
| 정차 거리 (구간) | 0.2525–0.2675 (틈 0.5–2 cm, `dock_legs_v2.npz`) | 0.2525–0.2875 (틈 0.5–4 cm, `dock_legs_v3.npz`) |

dock2 시험 (2026-11-06): 성공 99.8 % · 충돌 0.19 %, 박스 면 틈 중앙 2.46 cm (목표 1 cm) = 1.4 cm 짧게 섬.

```bash
cd sim/robot_sim
PY="env -u PYTHONPATH $HOME/pa-course/IsaacLab-6.2/.venv/bin/python"
$PY rl/train.py --task local_dock3 --run_name dock3
$PY rl/tasks/local_dock/eval_dock.py rl/logs/local_dock3/<run>/model_2000.pt        # 평가 (dock 평가 그대로)
$PY rl/tasks/local_planner/export.py rl/logs/local_dock3/<run>/model_2000.pt        # .pt · .npz
```
배포 노드: dock2 와 같은 인자에 `--tol_pos 0.01`.
