# moveit_config — 닌자터틀 팔 MoveIt 설정

| 파일 | 내용 |
|---|---|
| `config/ninja_turtle.srdf` | 팔 그룹 `arm` (`link1` → `end_effector_link`, `joint1` ~ `joint5`), 그리퍼 그룹 `gripper` (손가락 + 연결 막대), 끝점 `hand`, 이름 붙인 자세(`home` 받침 · `drive` 주행 · `pick_above` · `pick_down` · `pick_grasp` = 실물 grasp_pick 값, `open` · `close`), 수동 관절(그리퍼 연결 막대 3 · 바퀴 · 조향), 충돌 검사 끄는 쌍 |
| `config/kinematics.yaml` | IK = `pick_ik` (5 축이라 방향 일부만 맞춤) |
| `config/joint_limits.yaml` | 계획 속도 · 가속 한계, 기본 배율 0.3 |
| `config/moveit_controllers.yaml` | `arm_controller` · `gripper_controller` (FollowJointTrajectory) |
| `launch/move_group.launch.py` | 계획 서버 + rviz. 기본은 계획만 (`execute:=false`) |

## 실행
```bash
ros2 launch bringup robot.launch.py hardware:=mock          # 로봇 (또는 실물)
ros2 launch moveit_config move_group.launch.py use_rviz:=true
```
필요한 apt: `ros-lyrical-moveit`, `ros-lyrical-pick-ik`.

## 실행(execute) 을 막아 둔 이유
바퀴 · 팔 차례는 `control_master` 가 관리한다 (바퀴를 세우고 정지를 확인한 뒤 팔). MoveIt 이 `arm_controller` 에 직접 보내면
이 차례를 건너뛰어 바퀴가 움직이는 중에 팔이 움직일 수 있다. 실행은 `control_master` 가 MoveIt 계획을 받아 자기 차례 아래서 하도록 연결한다.

## 검사 결과 (11-02, GPT URDF 최종본)
`config/arm_paths.yaml` 의 30 경로 (넣기 · 빼기 S1~S12 실물 재생, LOOK, 펼치기 · 접기) 의 모든 점 (약 7,800) 을 `/check_state_validity` 로 검사 → 충돌 0.
그리퍼 연결 막대는 손가락 위치로만 움직여 MoveIt 이 피할 수 없음. 닫힘(−1.6 mm)에서 충돌용 단순 메시끼리만 겹치는
크랭크 ↔ 손가락 · 막대 ↔ 막대 쌍은 끔 (실제 메시는 막대 사이 0.46 mm). 연결 막대 각도는 `manipulation/arm_state_publisher` 가 `/joint_states_corrected` 로 냄.
적재함 칸 안 퍽 · 쥔 퍽은 MoveIt 장면에 없음 (칸 퍽과의 거리는 pose_safety 검사).

## URDF 를 바꾸면 충돌 쌍 다시 계산
```bash
xacro $(ros2 pkg prefix description)/share/description/urdf/robot.urdf.xacro hardware_map:=<hardware_map.yaml> > /tmp/robot.urdf
ros2 run moveit_setup_assistant collisions_updater --urdf /tmp/robot.urdf --srdf config/ninja_turtle.srdf \
  --output config/ninja_turtle.srdf --default --always --trials 10000 --keep
```
