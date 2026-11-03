# 로봇 쪽 인터페이스 (주행 · 조향 · IMU · 라이다 SLAM · 팔)

planning · perception 이 붙을 때 보는 표. 이름은 팀 시뮬 브리지(`sim/robot_sim/scripts/sim_nav_bridge.py`)
· 인지 노드(`perception`, `config/perception.yaml`) 와 맞춤. 장착값은 실측 반영 `description/urdf/robot.urdf.xacro` 기준.

## 토픽 · 액션

| 이름 | 타입 | 보내는 곳 | 프레임 · 주기 | 비고 |
|---|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/TwistStamped` | Nav2 · planning → swerve_controller | base_footprint 기준 몸 속도 | 0.3 s 끊기면 정지. 최고 0.10 m/s · 0.5 rad/s. header.stamp 가 젯슨 시각과 0.3 s 넘게 다르면 버림 (젯슨 안에서 보내거나 시각 동기화 필요, stamp 0 은 받은 시각 기준) |
| `/odom` | `nav_msgs/Odometry` | swerve_controller | `odom` → `base_footprint`, 50 Hz | 회전은 OpenCR 자이로 (`/imu_sensor_broadcaster/imu`) |
| TF `odom → base_footprint` | | swerve_controller | 50 Hz | `base_footprint → base_link` 는 URDF 고정 TF (z=0.033 m) |
| `/lidar_front/scan` | `sensor_msgs/LaserScan` | `coin_d4_driver/multi_coin_d4_node` | `lidar_front_link` (= `base_scan_front` = `base_scan`) | `slam.launch.py` 기본 입력. SensorDataQoS |
| `/lidar_rear/scan` | `sensor_msgs/LaserScan` | `coin_d4_driver/multi_coin_d4_node` | `lidar_rear_link` (= `base_scan_rear`) | 실물 운용 = 앞뒤 둘 다 (`run_robot.sh` 기본). 앞만 쓰려면 `NT_LIDAR_REAR=none` |
| `/scan` | `sensor_msgs/LaserScan` | `navigation/scan_merger` | `base_footprint`, 10 Hz | 앞 · 뒤 스캔을 고정 TF 로 합침 (0.5° 칸, 몸 상자 안 점 버림). 한쪽 라이다가 0.15 s 넘게 끊기면 안 냄. slam_toolbox · 앞 물체 정지 입력 |
| `/map` · TF `map → odom` | | slam_toolbox (`navigation/slam.launch.py`) | | 지도 만들기 = online_async, 위치 찾기 = localization (저장한 posegraph) |
| `/localization_health` | `diagnostic_msgs/DiagnosticStatus` | `navigation/pose_guard` | 5 Hz | level 0 정상 · 1 경고(느리게) · 2 정지. 지도 맞음 · SLAM 보정 · 라이다 · IMU · TF 끊김을 봄 |
| `/imu_motion_bias` | `geometry_msgs/Vector3Stamped` | `navigation/pose_guard` → swerve_controller | 1 Hz | z = 움직일 때 자이로에 더할 회전 [rad/s]. 정상 판정일 때만 보냄, 5 s 안 오면 컨트롤러는 0 |
| `/imu_sensor_broadcaster/imu` | `sensor_msgs/Imu` | imu_sensor_broadcaster (실물만) | `imu_link` | 주행 OpenCR 의 MPU-9250 칩 기준. 장착 yaw=+90° |
| `/joint_states` | `sensor_msgs/JointState` | joint_state_broadcaster | | 바퀴 · 조향 `drive_*_joint` · `steer_*_joint`, 팔 `joint1~5` · `gripper_left_joint` (오른손은 URDF mimic) |
| `/joint_states_corrected` | `sensor_msgs/JointState` | `manipulation/arm_state_publisher` → robot_state_publisher | `/joint_states` 와 같은 시각 | TF 용. 팔 `joint2~4` 에 처짐 보정을 더하고 그리퍼 연결 막대 3 관절을 붙임. 팔 관절이 하나라도 빠진 순간은 팔 관절을 안 보냄 (TF 가 멈춘 걸로 드러남). control_master 는 `/joint_states` 그대로 |
| `/joint_states_moveit` | `sensor_msgs/JointState` | `manipulation/arm_state_publisher` → move_group | 같은 시각 | MoveIt 용. 엔코더 값 그대로 + 그리퍼 연결 막대 3 관절 |
| `/arm_task` | `interfaces/action/ArmTask` | control_master (서버) | target `PoseStamped` (보통 base_link) | 7 가지 작업 모두 동작 (아래 '팔 작업') |
| `/control/motion_mode` | `interfaces/msg/MotionMode` | control_master → swerve_controller | 10 Hz | 바퀴 · 팔 차례 (팔 움직이는 동안 바퀴 잠금) |

손목 RealSense 는 기본 이름으로 띄운다: `ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true publish_tf:=false` → `/camera/camera/*`.
인지 노드(`config/perception.yaml`)와 팔 적재 코드가 이 토픽을 받는다. 광학 TF 는 URDF 가 보내므로 드라이버 TF 는 끈다(`publish_tf:=false`).
드라이버 header 의 frame_id(`camera_color_optical_frame`)는 URDF 에 없으므로, 받는 쪽이 URDF 이름으로 바꿔 쓴다 (인지 노드는 `wrist.frame_id` = `d435_color_optical_frame`).

| 토픽 | URDF 프레임 |
|---|---|
| `/camera/camera/color/image_raw` · `/camera/camera/color/camera_info` | `wrist_cam_color_optical_frame` (= `d435_color_optical_frame`) |
| `/camera/camera/aligned_depth_to_color/image_raw` (인지 노드) | `wrist_cam_color_optical_frame` |
| `/camera/camera/depth/image_rect_raw` (팔 적재 코드) | `wrist_cam_depth_optical_frame` |

토픽 이름에는 `/` 를 쓰고 TF 프레임 이름에는 붙이지 않는다. 광각 카메라 드라이버의 frame_id 는 `birdeye_camera_optical_frame` 으로 맞춘다.

## 좌표 · 장착 TF

새 앞쪽은 +X, 왼쪽은 +Y. `base_footprint` 는 바닥, 팀 `base_link` 는 바닥에서 0.033 m 위이다.
팔 검증 모델의 `base_link` 좌표를 팀 `base_link` 로 옮길 때는 **xyz 에 (0, 0, +0.039) m 를 더하고 회전은 더하지 않는다**.
이는 팔 검증 모델의 바닥 높이 0.072 m 와 팀 바닥 높이 0.033 m 의 차이이다.

| 부모 → 자식 프레임 | xyz (m, 부모 기준) | rpy (rad) · 의미 |
|---|---|---|
| `base_footprint → base_link` | `(0, 0, 0.033)` | `(0, 0, 0)` |
| `base_footprint → lidar_front_link` | `(+0.1145, 0, 0.060)` | `(0, 0, π)` · 앞 스캔면 |
| `base_footprint → lidar_rear_link` | `(-0.1145, 0, 0.060)` | `(0, 0, 0)` · 뒤 스캔면 |
| `base_link → imu_link` | `(0.035, 0.069, 0.0948)` | `(0, 0, +π/2)` · 주행 OpenCR 보드 중심, 위에서 찍은 사진으로 맞춤 (약 ±5 mm) |
| `base_link → link1` | `(0.0225499941511, 0, 0.166512411)` | `(0, 0, -1.208484306590865)` · 팔 받침 |
| `wrist_link → wrist_cam_color_optical_frame` | `(0.086934, -0.004296, -0.013517)` | `(-2.82003587790852, -0.00491770751513343, -2.01872902587038)` · T_5c |
| `wrist_link → wrist_cam_depth_optical_frame` | `(0.080595, -0.017995, -0.013696)` | `(-2.81348510600011, -0.016591029457623, -2.01201857941374)` · T_5d |

라이다 높이 0.060 m 는 **바닥 기준**이므로 `base_link` 기준으로는 0.027 m 이다. IMU 의 0.0948 m 는 **base_link 기준**이므로 바닥 높이는 0.1278 m 이다.
`robot.launch.py` · `slam.launch.py` 의 라이다 기본값은 위 xacro 값과 같다. 사용자가 launch 인자로 덮으면 그 값이 적용된다.
`base_scan_front` · `base_scan` 은 앞 스캔면과, `base_scan_rear` 는 뒤 스캔면과 위치·방향이 같은 별칭이다. 라이다 본체 메시 원점은 스캔면에서 z=-0.031 m 이다.

손눈 보정 JSON 의 `link5` 는 관절5 뒤 링크, 즉 팀 **`wrist_link`** 를 뜻한다. 팀 `link5` (관절5 이전) 와 구분한다.
T_5c · T_5d 는 광학축 회전까지 포함하므로 optical 회전을 다시 더하지 않는다.
`camera_link` 는 CAD 전면 프레임이고, `camera_optical_frame` · `d435_color_optical_frame` (인지 노드 · 시뮬 이름) 은 `wrist_cam_color_optical_frame` 과 같은 위치·방향의 별칭이다.
손목 · 광각 카메라와 프로파일은 `with_accessories:=true` 일 때 들어간다.

적재함 시각 프레임은 `rack_cell_S1_link` ~ `rack_cell_S12_link` 이다. 번호는 팔 J1 축에서 먼 칸부터이다. 채우는 순서는 `config/planning.yaml` `cargo_slots` (실물 시험 순서 S12, S7, S9, …), 칸끼리 제약은 `config/rack_cells.yaml` `rack_constraints`.
옛 이름 (`arm_tests/rack_cells.json`) 대응: S1 = L8, S2 = L7, S3 = L6, S4 = B, S5 = L2, S6 = L4, S7 = D, S8 = L5, S9 = A, S10 = L1, S11 = L3, S12 = C.
`base_link → rack_cell_<칸>_link` 고정 관절은 `rack_cell_<칸>_fixed` 이며, 원점은 각 칸 바닥 중심이다.
`arm_tests/rack_cells.json` 의 `cells.*.xy` 를 그대로 쓰고, z 는 `floor_z + 0.039 = 0.1325` m, yaw 는 칸별 `yaw_deg` (없으면 전체값 1°) 를 쓴다.
컵 테두리 윗면은 팀 base_link 기준 `rim_z + 0.039 = 0.1605` m (칸 원점에서 +0.028 m) 이다.
적재함도 `with_accessories:=true` 조건의 링크이다. 칸마다 바닥 + 벽 4 장 충돌 상자가 있어 MoveIt 이 팔과의 충돌을 본다 (관성은 없음). 집기 보정용 `place_offset` 은 컵 프레임 위치에 더하지 않는다.
칸 위치 그림: `config/rack_cells.yaml` (S1~S12 의 xy · yaw · 옛 이름).

스워브 관절 이름의 fl · fr · rl · rr 은 옛 이름을 유지한다. 새 좌표계에서 같은 순서의 조향축 `(x,y)` 는
`(-0.070,+0.112)` · `(+0.060,+0.112)` · `(-0.070,-0.112)` · `(+0.060,-0.112)` m (실측: 앞뒤 조향축 130 mm, 중심이 몸체 가운데보다 5 mm 뒤),
바퀴 offset 은 `(+0.0269,+0.0269,-0.0269,-0.0269)` m 로 `config/controllers.yaml` 과 같다.

## 팔 작업 (`/arm_task`, control_master)

팔은 늘 한 작업씩. 받으면 바퀴를 세우고 정지를 확인한 뒤 움직인다. **주행은 받침에 접혀 끝나는 작업 뒤에만** 허용된다 (`/control/motion_mode`).
STORE · RETRIEVE 경로는 11-01 실물 적재함 12 칸 시험에서 성공한 실행 기록을 그대로 재생한 것 (`config/arm_paths.yaml`, 칸 근처 15 °/s · 빈 공간 40 °/s).

| 작업 | 시작 자세 | 하는 일 | 끝 자세 | 다음 | 가짜 로봇 시간 |
|---|---|---|---|---|---|
| HOME | 아무 자세 | 접기 (LOOK 자세면 정해진 복귀 경로) | 받침 (주행 가능) | | 5 s |
| PICK | 받침 또는 LOOK 자세 (LOOK 이면 검사된 복귀 경로로 받침을 거쳐 감) | target 퍽 위 40 mm → 10 → −30 mm 에서 쥠 → 들기 | **박스 위 기준점** (퍽 쥠, 바퀴 잠금 유지) | STORE 또는 PLACE | 16 s (LOOK 에서 24 s) |
| STORE | 박스 위 기준점 (PICK 뒤) | slot 칸에 놓기 | 받침 (주행 가능) | | 19~31 s |
| RETRIEVE | 받침 또는 LOOK 자세 (LOOK 이면 복귀 경로로 받침을 거쳐 감) | slot 칸에서 꺼내기 | **박스 위 기준점** (퍽 쥠, 바퀴 잠금 유지) | PLACE | 19~32 s |
| PLACE | 박스 위 기준점 (PICK · RETRIEVE 뒤) | target 위 40 → 10 → −23 mm 에서 놓기 → 들기 | 받침 (주행 가능) | | 17 s |
| LOOK · LOOK_BARCODE | 받침 | 손목 카메라 관찰 자세 | 그 자세 유지 | HOME | |

- **target** (PICK · PLACE): `docs/planning/planning_interfaces.md` 6 절 약속 그대로 — `base_link` 기준 **그리퍼 중심이 갈 점** = 퍽 중심 + 쥘 끝(위) 방향 × `grasp_offset_m` (0.015), 방향 z 축 = 쥘 끝 · x 축 = 네모 퍽 면. 서 있는 퍽만 (누운 퍽은 실물 시험 전이라 거절). `base_link` 가 아닌 프레임 · NaN 은 거절. `z = 0` 이면 박스 위 기본 높이 (손 시험용). 팔이 닿지 않는 자리면 거절(이유가 result.message).
- **slot** (STORE · RETRIEVE): `S1` ~ `S12`. 시작 자세가 맞지 않으면 거절 (예: PICK 없이 STORE).
- 실패 · 취소 · 시간 초과 뒤에는 STOP 잠금, HOME 이 성공해야 다시 주행.
- HOME 이 검사된 복귀 경로 없이 받침으로 곧장 가는 건 받침 근처이거나 (켤 때 등) 팔이 경로 중간이 아닐 때만. **퍽을 쥔 채 · 경로 작업이 도중에 멈춘 뒤에는 HOME 거절** (칸 · 박스 안에서 직선으로 쓸고 나올 수 있음) → STORE · PLACE 로 끝내거나 사람이 팔을 받침 근처로 옮긴 뒤 HOME.
- 퍽을 쥔 채 주행은 없음 (PICK 뒤에는 STORE 또는 PLACE).
- 사람이 팔 주변을 보고 안전하다고 판단했을 때만 `{task: 0, slot: force}` 로 위 HOME 거절을 건너뜀 (시험 중 도중 정지 복구용).
- **닿는 거리**: 수직 손은 박스 위 퍽을 차체 앞 약 24 cm 까지. 더 멀면 손을 10° 씩 바깥으로 기울임 (`pick_max_tilt_deg`, 기본 30° → 약 34~36 cm, 10-27 실물 34 cm 집기와 같은 범위).
  5 축 팔이라 기울이는 방향은 J1 → 퍽 방향뿐 → 네모 퍽은 면이 그 방향에서 10° 안일 때만 기울임 (손가락이 닫히며 맞춤). 기울인 손은 실물 확인 전.
- **이웃 퍽 피해 집기 · 놓기**: control_master 가 `/perception/wrist/detections` 를 받아 (odom 기준 저장, 최근 10 s) 목표 둘레 12 cm 안 · 같은 높이의 퍽을 이웃으로 봄.
  손가락 끝 실측 치수 (안쪽 면 9.1 mm + 손가락 위치, 두께 18 mm, 폭 ±16.4 mm) 로 닫힘 방향 (네모 = 면 방향 2 개, 원기둥 = 15° 간격) 과 벌림 (다 열기 → 넣기 폭) 을 고름.
  이웃과 2 mm · 목표 퍽과 3 mm 여유가 안 되면 거절 (이유가 result.message). 놓을 자리 25 mm 안에 이미 퍽이 보이면 PLACE 거절. 이웃이 없으면 예전과 같은 동작.
- 박스 위 기준점 = 실물 빼기 시험 때 내려놓던 자리 (base_link 기준 x 210.2, y 38.0 mm, −4°) 위 40 mm.

## 지도 파일

`tools/jetson/save_map.sh <이름>` → `~/maps/<이름>.{pgm,yaml}` (map_server · Nav2 용) + `.{posegraph,data}` (slam_toolbox 위치 추정용).

## 아직 안 맞는 것

- **팔 작업 실물 확인**: `/arm_task` 7 작업은 가짜 로봇에서 12 칸 전부 (49 동작) 통과. C++ control_master 로 실물 팔을 돌리는 확인은 남아 있다 (PICK 1 회, 칸 몇 개 저속).
- **손목 카메라 TF 처짐 보정**: 실물 비교 전. 이상하면 `robot.launch.py sag_correction:=false` 로 끄고 비교.
- **실물 카메라 연결**: 위 frame_id · `publish_tf:=false` 를 실제 RealSense 실행 설정과 대조하는 시험은 남아 있다.
