# config/rl — RL 지역 계획기 정책 (배포용)

`ros2 launch navigation nav2.launch.py local_planner:=rl` (기본) 이 읽음. bringup 이 `share/bringup/config/rl/` 로 설치.

| 파일 | 내용 | 학습 원본 (sim/robot_sim/rl/logs, gitignore) |
|---|---|---|
| `nav_nf5.npz` · `nav_nf5.json` | 주행 정책 nf5 (`navigate_to_pose`, vmax 0.255, PolygonStop 구역 학습) | `local_planner_nf5/20261107_152247_nf5/model_1500.pt` |
| `dock3.npz` · `dock3.json` | 정밀 접근 정책 dock3 (`dock_to_pose`, 성공 1 cm 학습, 정차 0.2525–0.2875) | `local_dock3/20261106_191109_dock3_v0255/model_2000.pt` |
| `team_swerve.yaml` | 스워브 기하 · 조향 범위 (노드 관측 · 관절 이름) | `sim/robot_sim/rl/data/team_swerve.yaml` |

- `*.npz` = `rl/tasks/local_planner/export.py` 출력 (numpy 만으로 돎, torch 없음 = Jetson), `*.json` = 학습 `cfg.json` (관측 · 속도 한계 · 근접 속도 제한을 노드가 읽음)
- Isaac 시험: nf5 + dock3, 박스 · 장소 9 곳 131 s, 박스 면 틈 1.19–2.11 cm
- 바꿀 때: 새 정책을 export → 여기 덮어쓰고 이 표 고침
