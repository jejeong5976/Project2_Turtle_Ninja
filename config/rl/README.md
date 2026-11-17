# config/rl — RL 지역 계획기 정책 (배포용)

`ros2 launch navigation nav2.launch.py local_planner:=rl` (기본) 이 읽음. 주행 정책은 `nav_policy:=<이름>` (기본 `nav_nf5`, 다른 판 `nav_nf7`), 정밀 접근은 `dock_policy:=<이름>` (기본 `dock3`, 다른 판 `dock5`). bringup 이 `share/bringup/config/rl/` 로 설치.

| 파일 | 내용 | 학습 원본 (sim/robot_sim/rl/logs, gitignore) |
|---|---|---|
| `nav_nf7.npz` · `nav_nf7.json` | 주행 정책 nf7 (nf6 헛회전 · 반전 벌점 + arena5 구간 · 지도 ↔ 박스 어긋남 · yaw 흐름 · 위치 튐 랜덤화) | `local_planner_nf7/20261111_105107_nf7/model_1500.pt` |
| `nav_nf5.npz` · `nav_nf5.json` | 주행 정책 nf5 (기본, `navigate_to_pose`, vmax 0.255, PolygonStop 구역 학습) | `local_planner_nf5/20261107_152247_nf5/model_1500.pt` |
| `dock5.npz` · `dock5.json` | 정밀 접근 정책 dock5 (`dock_policy:=dock5`): dock4 (회전하며 넘겨받기) + 박스 지도 대비 반지름 18 cm · ±15° 밀림 + 박스 측정기 흉내 (측정 전 = 지도 정차점) + 관측 측정 표시 · 앞쪽 라이다 41 칸 (입력 203) | `local_dock5/20261112_001452_dock5/model_2500.pt` |
| `dock3.npz` · `dock3.json` | 정밀 접근 정책 dock3 (기본, `dock_to_pose`, 성공 1 cm 학습, 정차 0.2525–0.2875) | `local_dock3/20261106_191109_dock3_v0255/model_2000.pt` |
| `nudge_n1b.npz` · `nudge_n1b.json` · `nudge_n1b.md` | 팔 정책: 칸 안 삐뚤어진 퍽 툭 건드리기 (팀 feat/rl-nudge b7e6825, 관측 86 · 행동 6 · 30 Hz). control_master `nudge_enable` (기본 꺼짐) · robot_cli `nudge <칸>` 이 읽음 (`nudge_policy_path` "" = 설치 share 의 이 파일). 설명 `nudge_n1b.md` | `training2/logs/20261111_210322_n1b/model_199.pt` |
| `team_swerve.yaml` | 스워브 기하 · 조향 범위 (노드 관측 · 관절 이름) | `sim/robot_sim/rl/data/team_swerve.yaml` |

- `*.npz` = `rl/tasks/local_planner/export.py` 출력 (numpy 만으로 돎, torch 없음 = Jetson), `*.json` = 학습 `cfg.json` (관측 · 속도 한계 · 근접 속도 제한을 노드가 읽음)
- `dock_tol` (nav2.launch) 기본 = 비움 → dock5 0.01 · dock3 · simple 0.015
- 바꿀 때: 새 정책을 export → 여기 덮어쓰고 이 표 고침
