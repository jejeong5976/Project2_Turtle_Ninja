# 배포판 URDF (control 도구용 임시)

시뮬은 팀 URDF (`tools/convert_robot.py` → `robot/`, `usd/robot/`) 를 씀. 이 폴더는 아래 도구가 아직 배포판 관절 이름 (`arm_joint1..5`) · 틱 변환으로 읽어서 남겨 둔 것뿐임.

| 파일 | 읽는 도구 |
|---|---|
| `robot.urdf` | `tools/arm_real/place_spots/place_ik.py`, `tools/arm_real/arm/camera_angle_ref.py`, `tools/wrist_d435/handeye.py` (순기구학 `sim_core/arm_poses.py`) |
| `config/joint_map.json` | `tools/wrist_d435/handeye.py` (모터 틱 → 배포판 관절 rad) |

도구를 팀 URDF (`joint1..5`, `config/hardware_map.yaml` 틱 기준) 로 옮긴 뒤 이 폴더를 지움.
