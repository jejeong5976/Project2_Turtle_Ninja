# planning 인터페이스 (mission_manager)

- 대상 노드: `mission_manager` (패키지 `mission`)
- 기준 코드: main `b55aeb4`(PR #2 `feat/base-drive` 머지) + 브랜치 `feat/planning-core`
- 작성 2026-10-20, 갱신 2026-10-21 (코드 구현 기준으로 다시 씀), 2026-10-22 (상태 2층 구조 · 출발 신호 · 준비 확인 · 실행 중 감시), 2026-11-01 (planning / mission 패키지 분리, 옛 move 흐름 · 신호등 구역 규칙 삭제)
- 패키지 구조 (2026-11-01)

| 패키지 | 내용 |
|---|---|
| `planning` (실행 층, 미션 내용 모름) | 라이브러리 `planning_core` (types · puck_registry · cargo · health · grasp, ROS 없음) + `planning_exec` (`Executor`: Nav2 · `/arm_task` 호출, 재시도, HOME 끼워 넣기, DRIVE 대기, 감시, 퍽 기록부, 재고, `/mission/cargo` · `/planning/markers`). 실행 파일 없음 (2026-11-02 `record_point` · `mission_map` 삭제) |
| `mission` (발제 v5 미션) | 계산 파일 dice · dice_map · dice_steps · dice_verify · dice_record · transfer_* · signal_recorder · task_parser + 노드 `mission_manager` (`mission_manager.cpp`), `launch/mission.launch.py` |

- 의존 방향: mission → planning (반대 없음). 노드 이름 `mission_manager` (2026-11-02 옛 `planning_master` 에서 바꿈, 토픽 · 서비스 · 파라미터 이름은 그대로, `config/planning.yaml` 최상위 키만 바뀜)
- 표시: **[구현]** 코드에 있음, **[계획]** 합의 · 구현 전
- 확정 사항
  - task는 벽에 붙은 1D 바코드로 읽음. 내용 형식 미정 → perception은 원문만 넘기고 해석은 planning
  - task는 퍽을 AprilTag ID로 지정. 태그는 퍽에 잘 보이게 부착
  - 퍽 색: 빨강 · 초록 · 베이지 (파랑 제외)
  - 신호등 1개, 색 이름 red / yellow / green
  - 퍽은 긴축(6 cm) 한쪽 끝 3 cm 구간을 쥠 (적재함 설계 기준). 누운 퍽도 집음

---

## 1. planning이 맡는 일

| 맡음 | 맡지 않음 |
|---|---|
| 바코드 원문 해석 → task | 영상 처리, 검출, 신호등 판정 · 안정화 (perception) |
| 퍽 기록부 (ID → 지도 위치) | 팔 궤적 · IK, 바퀴 제어, 바퀴/팔 차례 (control) |
| 적재함 · 그리퍼 재고 기록 | 지도 위 경로 계획 (Nav2) |
| task → 단계 목록 → 순서대로 실행 | 모터 명령 직접 발행 (`/cmd_vel` 안 보냄) |
| 팔 작업 목표점(target) 계산 | |
| 신호등 점멸 기록, 시간 관리, 출발점 복귀 | |

원칙

- 한 번에 하나만 움직임: Nav2 이동 → 팔 작업. 앞 단계 결과를 받은 뒤 다음 단계 시작
- 검출 해석(좌표 변환 · 기록)은 planning이 함
- 재고 · 퍽 상태는 팔 작업 **성공 결과**를 받았을 때만 바꿈

---

## 2. 전체 연결

```
          /perception/wrist/detections ──┐  (손목 퍽: 색 · 모양 · AprilTag ID · 3D 자세)
perception ──────/perception/traffic_light┤  (신호등 최종 상태)
                 /perception/task_text ───┤  (바코드 원문)
                                          ▼
/control/motion_mode ──────────────▶ mission_manager ──/mission/status──▶ 대시보드 · rosbag
/odom, /tf ────────────────────────▶       │          ──/mission/hold───▶ cmd_vel_hold (navigation)
                                           │          ──/mission/cargo──▶ 대시보드
                                           │          ──/planning/markers▶ RViz
                                           ├─ navigate_to_pose (액션) ──▶ Nav2
                                           ├─ /arm_task        (액션) ──▶ control_master
                                           └─ /approach        (액션) ──▶ [계획] 정밀 정렬
```

---

## 3. 받는 토픽 [구현]

| 토픽 | 타입 | 보내는 쪽 | QoS | planning이 하는 일 |
|---|---|---|---|---|
| `/perception/wrist/detections` (`detection_topics`) | `interfaces/DetectionArray` | perception 손목 | reliable, depth 10 | 촬영 시각 TF로 map(기록부) · odom(집기용) 변환. 2026-11-02 카메라별 토픽으로 분리 (광각 `/perception/wide/detections` 는 위치 NaN 이라 안 받음) |
| `/perception/task_text` | `interfaces/TaskText` | perception | reliable, depth 10 | 바코드 원문 → `parse_task` |
| `/perception/traffic_light` | `interfaces/TrafficLight` | perception | reliable, depth 10 | 출발 구역에서 점멸 순서 기록 (9.1절). 구역 정지 규칙(9.2절)은 기본 꺼짐 |
| `/control/motion_mode` | `interfaces/MotionMode` | control_master (10 Hz) | reliable, transient_local, depth 1 | 주행 = DRIVE일 때만, 팔 작업 = `ready`일 때만. 끊기면 FAULT |
| `/odom` | `nav_msgs/Odometry` | swerve_controller (50 Hz) | reliable, depth 10 | 준비 확인, RUNNING 중 끊기면 FAULT |
| `/lidar_front/scan` · `/lidar_rear/scan` (`scan_topics`) | `sensor_msgs/LaserScan` | 라이다 드라이버 2 개 | SensorData | 라이다마다 생존 확인만 (준비 확인, 하나라도 끊기면 이동 차단) |
| `/localization_health` | `diagnostic_msgs/DiagnosticStatus` | navigation `pose_guard` (slam.launch.py) | 기본, 5 Hz | `localization_source: health` (기본, 실물 slam_toolbox). level 0 정상 · 1 경고 · 2 정지, 2 이상 · 끊김 = 상실 |
| `/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | AMCL | reliable, transient_local, depth 1 | `localization_source: amcl` (시뮬) 일 때만. 위치 표준편차 (마지막 유효 값 유지) |
| `/tf`, `/tf_static` | TF | slam_toolbox 또는 AMCL · swerve · robot_state_publisher | 기본 | 카메라 → map / odom / base_link, map → odom 신선도(위치 추정 생존) |

받는 쪽 규칙

| 규칙 | 기본값 | 설정 이름 |
|---|---|---|
| `header.stamp`가 오래된 검출 버림 | 0.5 s | `detection_max_age_s` |
| 신뢰도 낮은 검출 버림 | 0.5 | `min_confidence` |
| `pose.position`이 NaN · `frame_id` 빈 것 · TF 변환 실패 → 버림 | - | - |
| ID 못 읽은 후보를 오래 못 보면 삭제 | 30 s | `candidate_max_age_s` |
| `/perception/traffic_light` 끊김 → UNKNOWN + 못 찾음으로 봄 | 1.0 s | `signal_timeout_s` |
| `/odom` · `/control/motion_mode` 끊김 → FAULT (RUNNING) | 1.0 s | `control_timeout_s` |
| 검출 출력 생존은 카메라 프레임별 (`DetectionArray.header.frame_id`, 빈 배열도 출력) | d435_color_optical_frame | `detection_frames` |
| MotionMode 따르기 (control 없이 시험할 땐 false) | true | `require_motion_mode` |

---

## 4. 보내는 토픽 [구현]

| 토픽 | 타입 | 받는 쪽 | 주기 | 내용 |
|---|---|---|---|---|
| `/mission/status` | `interfaces/MissionStatus` | 대시보드 · rosbag | 2 Hz | 운영 상태, 미션 단계, 대기 원인, ready, 결과, 팔 phase, task · 퍽 개수, 시간, hold |
| `/planning/signal_record` | `interfaces/SignalRecord` | 대시보드 · 이후 task 계산 | 바뀔 때 + 1 Hz, transient_local | 신호등 점멸 순서 · R/Y/G 횟수 · 상태 · 문제 (9.1절) |
| `/mission/hold` | `std_msgs/Bool` | `cmd_vel_hold` (navigation 패키지) | 바뀔 때 + 1 Hz | true = 실제로 주행을 막을 때: STANDBY · FAULT · RUNNING 중 이동 차단 원인 |
| `/mission/cargo` | `interfaces/CargoState` | 대시보드 · rosbag | 바뀔 때 + 1 Hz | 적재함 칸별 재고, 그리퍼 |
| `/planning/markers` | `visualization_msgs/MarkerArray` | RViz | 2 Hz | 퍽 기록부(색 원기둥 + ID), 후보(구), Nav2 목표(화살표) |
| `/planning/perception_request` | `PerceptionRequest` | perception | - | [계획, 선택] 지금 찾는 대상 |

- `/mission/hold`: twist_mux 대신 `cmd_vel_hold` 가 받음 (2026-10-28). Nav2 `cmd_vel_nav` → collision_monitor → `cmd_vel_safe` → `cmd_vel_hold` → `/cmd_vel` → swerve_controller. true 면 Nav2 명령을 버리고 0 을 보냄. teleop 이 `/cmd_vel` 로 직접 보내면 안 거침. 2026-11-01: `nav2.launch.py mission_hold:=true` (기본) 면 hold 를 받기 전에도 막고 (`start_hold`), hold 가 2.5 s 넘게 안 오면 막음 (mission_manager 가 죽으면 멈춤). Nav2 단독 시험은 `mission_hold:=false`. 운영 규칙: Nav2 를 띄운 동안 teleop 금지, 매핑 · 수동 시험은 Nav2 없이

---

## 5. 서비스 · 부르는 액션

제공하는 서비스 [구현] (모두 `std_srvs/srv/Trigger`, 결과 `success` + `message`)

| 이름 | 하는 일 |
|---|---|
| `/mission/start` | STANDBY + 준비 완료일 때만 RUNNING. 경기 시간 시작(첫 start). 안 되면 거절 + 준비 안 된 이유. 사람이 RViz로 위치 확인한 뒤 호출 |
| `/mission/resume` | FAULT → STANDBY. 취소한 팔 · Nav2 목표의 최종 결과를 받기 전이면 거절. 다시 start 해야 움직임 |
| `/mission/clear_gripper` | 그리퍼 재고 기록 지움 (사람이 퍽을 빼고 확인한 뒤). 기록이 남아 있으면 start 거절 |

부르는 서비스 [구현]: Nav2 `lifecycle_manager`의 `/<매니저>/is_active` (Trigger, 1 Hz, `lifecycle_services`)

부르는 액션

| 이름 | 타입 | 서버 | 상태 |
|---|---|---|---|
| `navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | Nav2 | [구현] 목표 frame `map` |
| `/arm_task` | `interfaces/action/ArmTask` | control_master | [구현] HOME · PICK · PLACE · STORE · RETRIEVE · LOOK · LOOK_BARCODE (control 은 HOME · PICK · PLACE 만 구현) |
| `/approach` | `Approach.action` | 미정 | [계획] 정밀 정렬 |
| `/perception/check_slot` | `CheckSlot.srv` | perception | [계획, 선택] 적재함 칸 확인 |

액션 규칙 [구현]

| 규칙 | 기본값 | 설정 이름 |
|---|---|---|
| 서버가 안 뜸 → 그 단계 실패 (팔은 `ready` 대기 포함) | 30 s | `server_wait_s` |
| 팔 작업 거절 → 재전송 간격 · 최대 시간 | 1 s / 15 s | `action_retry_s`, `action_retry_max_s` |
| 결과 대기 최대 시간 | 60 s / 60 s | `nav_timeout_s`, `arm_timeout_s` |
| FAULT · 복귀 시작 → 진행 중 Nav2 목표 취소 | 즉시 | - |
| FAULT → 진행 중 팔 작업 취소 | 즉시 | - |
| 취소한 목표(팔 · Nav2)의 최종 결과 전에는 새 목표 안 보냄 (2026-11-01) | 넘으면 단계 실패 (HOME 이면 FAULT) | `server_wait_s` |
| 팔 시간 초과 → 취소 → 최종 결과까지 기다려 판정. 결국 성공이면 단계 성공 + 재고 반영, 아니면 실패 + HOME 필요 | - | `arm_timeout_s` |
| FAULT 등으로 취소한 팔 작업이 결국 성공 → 재고 · 기록부에 반영 (그리퍼 기록 → start 전 `/mission/clear_gripper` 확인) | - | - |

control_master 동작에 맞춘 규칙 [구현]

| control 동작 (main) | planning 규칙 |
|---|---|
| 켤 때 STOP, 첫 HOME 성공 뒤 DRIVE | 출발 전에는 HOME 안 함. start 승인 뒤 HOME → DRIVE 대기 (`drive_wait_s` 넘으면 FAULT) |
| 팔 작업 실패 · 취소 · 시간 초과 → STOP 잠금, HOME 성공해야 DRIVE | 팔이 움직이다 성공으로 안 끝나면 다음 단계 앞에 `arm:HOME(복구)` 끼워 넣음 |
| HOME 실패 → 바퀴 못 풂 | HOME 실패 = FAULT |
| `ready=false`면 ArmTask 거절 | `ready=true`일 때만 보냄 |
| 바퀴는 DRIVE일 때만 움직임 | Navigate는 DRIVE일 때만 진행 (대기 원인 DRIVE_LOST). `drive_wait_s`(10 s) 넘으면 HOME 끼워 넣음. DRIVE 받기 전 두 번째 HOME이면 FAULT |
| 팔 진행 feedback `phase` | `/mission/status.arm_phase`에 표시 |

---

## 6. ArmTask.target 약속 [planning 구현, control IK 합의 대기]

- control_master는 지금 target을 안 쓰고 고정 자세로 동작함 (시험용). IK로 target을 쓰도록 바뀔 예정 (2026-10-21 control 담당 확인)

| 항목 | 값 |
|---|---|
| frame | `base_link`, 보내는 순간 기준 |
| `position` | **그리퍼 중심이 갈 점** = 퍽 중심 + 쥘 끝 방향 × `grasp_offset_m`(0.015) → 쥘 쪽 3 cm 구간의 가운데 |
| `orientation` | z 축 = **쥘 끝** 방향. x 축 = 아래 규칙 (`grasp_orientation`, 2026-10-27). perception 이 방향을 안 줬으면 (0,0,0,0) |
| HOME | position (0,0,0), orientation (0,0,0,0) |

쥘 끝 고르기 (`choose_grasp_axis`, 퍽은 위아래 대칭 → 양 끝 모두 가능)

| 퍽 상태 | 쥘 끝 |
|---|---|
| 방향 모름 | 똑바로 섰다고 봄 → 위쪽 끝 |
| 긴축이 수직에서 45° 안 | 위쪽 끝 (뒤집혀 있어도) |
| 더 누움 | 로봇 쪽 끝 (팔을 덜 뻗음). 축이 로봇 방향과 직각이면 자세 z 쪽 |

target x 축 규칙 (`grasp_orientation`) — 항상 같은 규칙으로 만들어서 control 이 x 축을 믿을 수 있게 함

| 퍽 상태 | x 축 |
|---|---|
| 선 사각기둥 · 면 방향 앎 (`Detection.yaw_known`) | 면 방향 θ (사각기둥 면 법선, 90° 주기) → 손가락이 마주 보는 두 면을 잡게 할 기준 |
| 선 퍽 · 면 방향 모름 (원기둥 · 면 방향 못 구한 사각기둥) | 로봇 → 퍽 수평 방향 (기본값) |
| 누운 퍽 | 위쪽 (0, 0, 1) 을 z 에 수직으로 눌러 놓은 방향 (누운 사각기둥은 perception 도 x 축 = 위쪽 + `yaw_known` true 로 보냄 → 같은 방향) |

PICK 때 base_link 기준 **x, y, z, θ + 선/누움** 을 로그로 남김 (`planar_from_pose`). 퍽은 수평면 위라 이 4 값 + 선/누움이면 집기 자세가 다 정해짐
- θ: 선 사각기둥 = 면 방향 [-45°, 45°), 누운 퍽 = 긴축 수평 방향 [-90°, 90°), 선 원기둥 = 필요 없음

위치 출처

| 작업 | 출처 |
|---|---|
| PICK | ① odom 기준 최근 검출(ID 퍽, `pick_fresh_s` 2 s 안) → base_link. map을 거치지 않아 AMCL 보정에 안 흔들림 ② 없으면 퍽 기록부(map) 위치 + 경고 |
| PLACE | `dice_map.yaml` 장소 칸 점 (x, y), z = 장소 `top_z` + `puck_half_height_m`(0.03) + `grasp_offset_m` → 세워서 놓을 때 그리퍼 중심 (mission 노드가 계산) |

예 (가짜 환경 확인값, 퍽 중심 몸 앞 0.5 m)

| 경우 | target position | target orientation (x, y, z, w) |
|---|---|---|
| 서 있는 퍽, 중심 z 0.03 | (0.500, 0.000, 0.045) | z 위 · x 면 방향(앎) 또는 로봇 → 퍽. 방향 안 줬으면 (0,0,0,0) |
| 앞뒤로 누운 퍽, 중심 z 0.015 | (0.485, 0.000, 0.015) | z 축이 로봇 쪽 (−x), x 축 위쪽 |
| PLACE, 선반 0.08 | (0.180, 0.000, 0.125) | (0,0,0,0) |

---

## 7. 메시지 · 액션 정의

### 7.1 planning이 추가한 것 [구현] (`interfaces/msg`)

```
# Detection.msg — 검출 1개 (perception → planning)
std_msgs/Header header        # stamp = 촬영 시각, frame_id = 카메라 광학 프레임
string kind                   # "puck"
string label                  # "red" | "green" | "beige"
string shape                  # "cylinder" | "box", 모르면 ""
string id                     # AprilTag ID 문자열, 못 읽으면 ""
float32 confidence            # 0.0 ~ 1.0
geometry_msgs/Pose pose       # 퍽 중심 [m] · 퍽 자세 (z 축 = 긴축 6 cm 방향)
                              #   위치 모르면 position 전부 NaN
                              #   방향 모르면 orientation (0,0,0,0) — msg 기본값이 w=1 이라 w=0 을 직접 넣어야 함
bool yaw_known                # [신규 2026-10-27] 퍽 x 축(사각기둥 면 방향)을 믿어도 되는지
                              #   true = x 축이 사각기둥 면 법선 (선 = 윗면 윤곽, 누움 = 위쪽 · 2026-10-29), false = z 축(긴축)만 믿음
float32[4] bbox               # [x_min, y_min, x_max, y_max] 원본 영상 픽셀
```

```
# DetectionArray.msg — 영상 1장의 검출 전부 (못 찾아도 빈 배열로 발행)
std_msgs/Header header
Detection[] detections
```

```
# TaskText.msg — 벽 1D 바코드 원문 (해석 안 함, 중복 억제 안 함)
std_msgs/Header header        # stamp = 촬영 시각, frame_id = 카메라
string raw_text
geometry_msgs/Pose pose       # 바코드 위치, 모르면 NaN
```

```
# TrafficLight.msg — 신호등 최종 상태 (판정 · 안정화 perception, 약 5 Hz 계속 발행)
uint8 UNKNOWN=0
uint8 RED=1
uint8 YELLOW=2
uint8 GREEN=3
uint8 OFF=4                   # 2026-10-28 추가: 보이고 불이 모두 꺼짐 (점멸 기록은 소등으로 점멸을 구별)
std_msgs/Header header
uint8 state
bool visible                  # OFF + true = 소등, UNKNOWN + true = 애매, UNKNOWN + false = 못 찾음
```

```
# SignalRecord.msg — 신호등 점멸 기록 결과 (planning → 대시보드 · 이후 task 계산), 2026-10-28
uint8 WAITING=0 / RECORDING=1 / DONE=2 / INCOMPLETE=3
std_msgs/Header header
uint8 status
string sequence               # 예 "RGRRGRGGYGYG", 색 불명은 '?'
uint8 count_red / count_yellow / count_green / count_unknown
string[] issues               # 관측 끊김 · 색 불명 · 개수 · 횟수 규칙
```

```
# CargoState.msg / CargoSlot.msg — 적재함 · 그리퍼 재고 (planning → 대시보드)
std_msgs/Header header
CargoSlot[] slots             # 팔이 닿는 칸만
CargoSlot gripper             # slot = "gripper"

string slot                   # "R1" 등, 그리퍼면 "gripper"
bool occupied
string puck_id
string color
string shape
string task_id
bool verified                 # 카메라로 확인했는가 (지금은 기록만 → false)
```

### 7.2 MissionStatus [구현, 2026-10-21 확장 · 10-22 상태 구조 반영]

- main 필드 5개는 유지 (state 값만 운영 상태로 바뀜), 아래에 필드 추가 (main에서 이 msg를 쓰는 다른 노드 없음)

```
std_msgs/Header header
string state                  # 로봇 운영 상태: STANDBY · RUNNING · DONE · FAULT
string detail                 # 마지막 실패 이유 등 한 줄
uint8 pucks_done              # 옮긴 퍽 수 (PLACE 성공)
uint8 pucks_total             # 옮겨야 할 퍽 수 (지금까지 읽은 task 의 퍽 합)
bool fault
# ---- 추가
string phase                  # 미션 단계 (RUNNING 일 때): HOME · READ_BARCODE · OBSERVE_SIGNAL · COLLECT · DELIVER · VERIFY · RETURN_HOME
string[] wait_reasons         # STANDBY: 준비 안 된 이유 / RUNNING: 대기 원인 / FAULT: 원인 코드
bool ready                    # STANDBY 에서 /mission/start 받을 수 있나
string result                 # DONE 일 때: COMPLETED · RETURN_FAILED
string task_id                # 실행 중인 task
string step                   # 지금 단계 이름 ("nav:puck:7", "arm:PICK:7" …)
string arm_phase              # ArmTask feedback phase
uint8 tasks_done
uint8 tasks_failed
uint8 tasks_total
float32 elapsed_s             # 첫 /mission/start 후 경과 (FAULT 중에도 흐름, DONE 에서 멈춤)
float32 time_left_s           # 제한 시간 없으면 -1
bool hold                     # /mission/hold 와 같은 값
```

### 7.3 control 소유 (main 정의 그대로)

```
# MotionMode.msg
uint8 STOP=0
uint8 DRIVE=1
uint8 ARM=2
std_msgs/Header header
uint8 mode
uint32 request_id
string reason
bool ready                    # control_master 가 지금 새 팔 작업을 받을 수 있나 (하드웨어 정상 + JTC 뜸)
```

```
# ArmTask.action (지금)
uint8 HOME=0
uint8 PICK=1
uint8 PLACE=2
uint8 task
geometry_msgs/PoseStamped target   # 6절 약속
---
bool success
string message
---
string phase                  # "approach" | "grasp" | "lift" | "release" | "retreat"
```

ArmTask 확장 요청 [계획, control 합의 필요]

| 추가 | 쓰는 곳 |
|---|---|
| `STORE=3`, `RETRIEVE=4` + `string slot` | 적재함 넣기 · 꺼내기 (planning 재고 전환은 구현됨) |
| `PUSH=5` | 퍽 치기 (밀어 넘어뜨리기) |
| `LOOK=6` | 바코드 · 퍽 관찰 자세 |
| 결과 `uint8 error` (UNREACHABLE · NO_OBJECT · HW_FAULT …) | 실패 이유별 처리 (다시 서기 / 포기 / FAULT) |
| 결과 `bool holding` | PICK 성공 확인 |

### 7.4 [계획] 미구현 정의

- `Approach.action`(정밀 정렬), `CheckSlot.srv`(적재함 칸 확인), `PerceptionRequest.msg`(찾는 대상) — 담당 · 필요 여부 미정

---

## 8. 상태 · 단계 [구현, 2026-10-22 2층 구조]

- 위층 = 로봇 운영 상태 4개, 아래층 = 미션 단계 4개 (RUNNING 안). 신호등 · 센서 대기는 상태가 아니라 **대기 원인 목록**
- 설계 검토: Codex 와 토의 후 단순화 (PREPARING + WAIT_START → STANDBY, PAUSED + SUSPENDED → 대기 원인)

```
STANDBY ──/mission/start──▶ RUNNING ──미션 끝──▶ DONE (COMPLETED · RETURN_FAILED)
   ▲                           │ 이상
   └──/mission/resume── FAULT ◀┘
RUNNING 안:  HOME → READ_BARCODE → OBSERVE_SIGNAL → COLLECT → DELIVER → VERIFY → RETURN_HOME   (발제 v5 순서: 바코드 → 12회 점멸 관측 → 이동, 2026-11-01)
```

- `OBSERVE_SIGNAL` (2026-10-28, 순서 2026-11-01): 첫 바코드를 읽은 뒤 출발 구역에 선 채 신호등 점멸 기록 끝까지 기다림 (9.1절). 기록은 STANDBY · HOME · 첫 READ_BARCODE 중에도 하므로 운영자가 언제 시작해도 놓치지 않음. 끝나거나 `signal_wait_s` 넘으면 COLLECT, 이미 끝났으면 건너뜀. 기록이 불완전하면 움직이지 않고 복귀 처리 (`DONE`)
- `READ_BARCODE` 동안만 perception 바코드 판독을 켬: `barcode_enable_service` (기본 `/perception/barcode/enable`, std_srvs/SetBool) 에 true, 다른 단계 · FAULT · DONE 이면 false. 응답 받을 때까지 1 s 마다 다시 보냄

| 위층 | 하는 일 | 다음 |
|---|---|---|
| `STANDBY` | 무동작, hold true. 준비 확인 (아래 표) | start 승인 → `RUNNING` / 준비 안 됐으면 거절 + 이유 |
| `RUNNING` | 미션 단계 실행 | 미션 끝 → `DONE` / 이상 → `FAULT` |
| `DONE` | 정지, 결과 기록 | - |
| `FAULT` | 모든 목표 취소, hold true, 원인 코드 기록 | `/mission/resume` (사람 확인) → `STANDBY` |

STANDBY 준비 확인 (`wait_reasons` 표시 이름, 움직이지 않는 확인만)

| 이름 | 조건 | 끄는 설정 |
|---|---|---|
| `ODOM` | `/odom` 최근 수신 | - |
| `CONTROL` / `CONTROL_NOT_READY` | MotionMode 최근 수신 / `ready=true` (DRIVE 는 안 봄) | `require_motion_mode` |
| `ARM_SERVER` · `NAV2_SERVER` | 액션 서버 연결 | - |
| `NAV2_LIFECYCLE:<서비스>` | is_active 응답 success (3 s 안) | `lifecycle_services: []` |
| `LOCALIZATION_TF` | map → odom 이 `loc_timeout_s`(2 s) 안에 갱신 (미래 시각 정상) | `require_localization` |
| `LOCALIZATION_HEALTH:<level>` · `:LOST` | health: `/localization_health` level 0 (`LOST` = 못 받음 · 2 s 끊김) | `require_localization` |
| `LOCALIZATION_COV` | amcl: AMCL 표준편차 x · y ≤ 0.10 m, yaw ≤ 10° | `require_localization` |
| `SCAN:<토픽>` | `scan_topics` 각 라이다 최근 수신 (앞 · 뒤 따로) | `require_scan` |
| `DETECTIONS:<프레임>` | 해당 카메라 검출 출력 최근 수신 | `require_detections` |
| `GRIPPER_RECORD` | 그리퍼 재고 기록 비어 있음 | `/mission/clear_gripper` |

| 아래층 (RUNNING) | 하는 일 | 다음 |
|---|---|---|
| `HOME` | ArmTask HOME → DRIVE 대기 | 시간 부족 → `RETURN_HOME` / 복귀 중 FAULT 였음 → `RETURN_HOME` / 아니면 `READ_BARCODE` |
| `READ_BARCODE` | 바코드 지점 이동 → LOOK_BARCODE → 읽기 → HOME (`barcode_look`, 이 단계 동안만 perception 바코드 판독 켬) | 해석 성공 → `OBSERVE_SIGNAL` / 미등록 → FAULT `BARCODE_UNREGISTERED` / 안 읽힘 → `RETURN_HOME` |
| `OBSERVE_SIGNAL` | 출발 구역에서 점멸 기록 끝까지 대기 | 완전 → `COLLECT` / 불완전 · 시간 초과 → `RETURN_HOME` |
| `COLLECT` | 박스마다 이동 → LOOK → Plan(CollectBox) = 본 퍽 [STORE] PICK 끼워 넣기, 마지막 박스 뒤 Plan(Delivery) = 할당 · 운반 계획 | `DELIVER` |
| `DELIVER` | 운반 계획대로 [RETRIEVE] PLACE (목표 = 장소 중심 + 주사위 눈 오프셋) | `VERIFY` |
| `VERIFY` | 장소마다 이동 → LOOK → Plan(Verify) 판정 · 기록 (`/planning/verify_record`, verify.yaml) | `RETURN_HOME` |
| `RETURN_HOME` | `start_pose`로 이동 | 도착 → `DONE`(COMPLETED) / 실패 → `DONE`(RETURN_FAILED) |

대기 원인 (RUNNING, 동시에 여러 개 가능, 차단 범위는 코드 한 곳 `nav_blocked` · `pick_blocked`)

| 원인 | 막는 것 | 해제 · FAULT |
|---|---|---|
| `DRIVE_LOST` | 모든 이동 | DRIVE 오면 해제, `drive_wait_s` 넘으면 HOME 끼워 넣음 |
| `SCAN_LOST` | 모든 이동 (진행 중 팔 작업은 끝까지) | 회복 2 s 유지 시 해제, 자동 해제 최대 2회(start 마다 초기화), 10 s 미회복 → FAULT |
| `DETECTION_LOST` | PICK 시작 (주행 허용) | 위와 같음 |
| `LOCALIZATION_LOST` | 모든 이동 + PICK | map → odom 오래됨 또는 표준편차 x · y > 0.30 m · yaw > 30° → 1 s 안에 안 돌아오면 FAULT |

- 시간 부족(`time_limit_s − return_margin_s`, 경기 시간은 첫 start 부터, 대기 원인 있어도 확인, 팔 작업 · HOME 중에는 끝나고 결과를 재고에 반영한 뒤 = `arm_busy()` false) → `RETURN_HOME`
- FAULT 원인 코드: 실행기 `ODOM_LOST` · `CONTROL_LOST` · `HOME_FAILED` · `DRIVE_NOT_RESTORED` · `SCAN_LOST` · `DETECTION_LOST` · `LOCALIZATION_LOST`, 미션 `MAP_INVALID` · `BARCODE_UNREGISTERED` · `DICE_UNSOLVABLE`
- 실행기는 사건(단계 성공 · 실패 · DRIVE 받음 · FAULT)을 돌려주기만 함. FAULT 결정 · 실패 뒤 처리는 미션이 함. 미등록 바코드 · 계획 불가 FAULT 는 한 주기 처리가 끝난 뒤 적용
- FAULT 뒤 재진입 (resume → start, 항상 HOME 부터): 바코드를 이미 처리했으면 복귀, RETURN_HOME 중이었으면 복귀 다시. 이전 팔 단계는 다시 실행 안 함
- 취소 추적: 취소한 팔 · Nav2 목표는 최종 결과(또는 거절)를 받을 때까지 기다림 → resume 조건

단계 종류 (`planning/types.hpp`, 미션을 모르는 형태)

| 단계 | 실행 | 성공 조건 |
|---|---|---|
| `Navigate` | `navigate_to_pose` (목표: `Fixed` 자세 / `Puck` 퍽 앞 / `Custom` 미션이 준 점) | Nav2 SUCCEEDED |
| `Arm` | `/arm_task` (PICK 목표 = 퍽 집기 점, PLACE `Custom` 목표 = 미션이 준 map 점 → base_link) | 결과 success |
| `Custom` | 미션 함수가 실행 (`kind` · `ref` 칸은 실행기가 해석 안 함) | 미션 함수 결과 |

dice 의 `Custom` 뜻 (`mission/common.hpp`)

| kind | 뜻 |
|---|---|
| `kReadBarcode` | `/perception/task_text` 대기 + 해석 |
| `kPlanCollectBox` | 이 박스에서 본 퍽 → [STORE] PICK 끼워 넣기 |
| `kPlanDelivery` | 할당 · 운반 계획 → 배달 · 재검증 단계 |
| `kPlanVerify` | 장소 LOOK 뒤 새 관측으로 칸 · 그룹 판정 |
| `kTargetDiceCell` (목표) | PLACE 목표 = 장소 `ref[1]` 의 칸 `ref[2]` |

재시도 [구현]

| 단계 | 실패 시 | 설정 이름 |
|---|---|---|
| Navigate | 1회 다시 → 실패 사건 | `nav_retries` = 1 |
| Arm PICK | 퍽 앞 이동부터 다시, 최대 2회 → 실패 사건 | `pick_retries` = 2 |
| Arm PLACE | 1회 다시 → 실패 사건 | `place_retries` = 1 |
| Arm HOME | 재시도 없이 FAULT | - |
| 팔이 움직이다 실패 | 위 규칙 전에 HOME 끼워 넣음 (DRIVE 복구 셈에 안 넣음) | - |
| 실패 사건 (미션) | 그 퍽 · 장소 빼고 재계획 (`dice_recover`, 최대 `max_replans`), 못 하면 복귀 | `max_replans` = 6 |

---

## 9. 신호등 [구현]

### 9.1 점멸 기록 (발제 v5, 2026-10-28) — `signal_recorder.hpp`

- 과제: 출발 구역에서 12 번 점멸 (1 s 켜짐 + 1 s 소등, 같은 색 연속도 소등으로 구별), 12 번째 뒤 3 s 소등 = 끝. R · Y · G 각 1–6 회, 합 12. 불명 색은 임의 추정 금지
- 관측: RED · YELLOW · GREEN, OFF + visible = 소등, UNKNOWN + visible = 애매, UNKNOWN + not visible 또는 토픽 끊김 = 못 봄

| 판단 | 기준 (설정) |
|---|---|
| 점멸 1 개 | 색 관측 시작 ~ 소등 `signal_min_off_s` 0.25 s 이상 |
| 깜빡임 버림 | 켜짐 < `signal_min_on_s` 0.3 s |
| 점멸 색 | 다수결 ≥ `signal_majority` 0.8 이고 애매 ≤ `signal_max_ambiguous` 0.3, 아니면 '?' |
| 끝 | 12 개 뒤 소등 `signal_end_off_s` 2.5 s. 그 전에 긴 소등 = 개수 부족 |
| 불완전 (INCOMPLETE) | '?' 있음 · 기록 중 못 봄 ≥ `signal_max_gap_s` 0.8 s · 개수 ≠ 12 · 횟수 규칙 위반 · 소등 없이 색 바뀜 |

- 못 본 구간은 켜짐 · 소등 어느 쪽으로도 세지 않음 → 소등을 못 봤으면 같은 색 두 점멸을 하나로 남김 (추정해 나누지 않음)
- 결과: `/planning/signal_record` + 로그. 다시 기록: `/mission/reset_signal`
- 단독 확인: `mission_manager` 를 띄우면 STANDBY 에서도 기록 → `/planning/signal_record` 확인 (2026-11-02 `signal_watch` 삭제). 시뮬: `sim/robot_sim/scripts/fake_traffic_light.py` (2026-10-28 확인: 잡음 애매 5 % · 못 봄 2 % 에서 RGRRGRGGYGYG DONE)

### 9.2 구역 정지 규칙 — 삭제 (2026-11-01)

- 옛 `light_gate` (신호등 구역 RED · YELLOW 정지) 는 발제 v5 에 없어 패키지 분리 때 지움. 필요하면 분리 전 원본 참고

---

## 10. 퍽 기록부 · 재고 [구현]

| 규칙 | 내용 |
|---|---|
| ID 있는 검출 | map 좌표로 기록 (같은 ID는 최신 값으로 덮어씀, 상태 유지). 같은 자리 후보 삭제 |
| ID 없는 검출 | 후보로 기록, 6 cm 안이면 합침. ID 퍽 근처면 그 퍽 위치만 갱신 |
| 상태 | `on_field` / `in_gripper` / `in_cargo:<칸>` / `delivered` — 팔 작업 성공 결과로만 변경 |
| 재고 | PICK → 그리퍼, STORE → 칸, RETRIEVE → 그리퍼, PLACE → 비움. 맞지 않는 전환은 거부 + 경고 (미션은 계속) |
| 재고 파일 | `cargo_file` 지정 시 바뀔 때마다 저장, `cargo_restore`로 재시작 때 복원 |
| [계획] 오래된 위치 | 기록 위치에 갔는데 안 보이면 기록 삭제 후 다시 탐색 |

---

## 11. 설정 파일

### 11.1 `config/planning.yaml` (레포 최상위, bringup이 설치. 없으면 코드 기본값)

| 이름 | 기본값 | 내용 |
|---|---|---|
| `dice_map` / `travel_table` | "" / "" | 비우면 레포 `maps/dice_map.yaml` / `maps/dice_travel_times.yaml` |
| `collect_order` | [1, 2, 6, 3, 4, 5] | 시작 박스 방문 순서 |
| `look_dwell_s` / `dice_pick_fresh_s` | 1.0 / 90.0 | LOOK 뒤 검출 모으기 / 정차 중 PICK 위치 유효 시간 |
| `transfer_max_remaining` | 4 | 박스 남은 퍽이 이보다 많으면 대체 해 |
| `verify_radius_m` / `verify_cell_tol_m` | 0.18 / 0.03 | 재검증 판정 |
| `record_dir` / `max_replans` | ~/planning_logs / 6 | 기록 파일 폴더 / 재계획 최대 횟수 |
| `barcode_look` / `barcode_enable_service` | true / /perception/barcode/enable | |
| `signal_record` / `signal_wait_s` / `signal_*` | true / 90.0 / 9.1절 | 점멸 기록 |
| `map_frame` / `base_frame` / `odom_frame` | map / base_link / odom | |
| `tick_hz` | 10.0 | 판단 주기 |
| `time_limit_s` | 0.0 | 0 = 제한 없음 |
| `return_margin_s` | 60.0 | 남은 시간이 이보다 적으면 복귀 |
| `detection_max_age_s` | 0.5 | |
| `min_confidence` | 0.5 | |
| `candidate_max_age_s` | 30.0 | |
| `signal_timeout_s` | 1.0 | |
| `control_timeout_s` | 1.0 | `/odom` · `/control/motion_mode` |
| `require_motion_mode` | true | |
| `drive_wait_s` | 10.0 | |
| `auto_start` | false | true 면 준비되는 즉시 시작 (시험용) |
| `lifecycle_services` | navigation is_active | 빈 목록이면 안 봄. `localization_source: amcl` 이면 localization is_active 를 코드가 더함 |
| `localization_source` | health | health = `/localization_health` (실물 slam) · amcl = `/amcl_pose` (시뮬). launch `localization:=slam\|amcl` |
| `require_scan` / `scan_timeout_s` / `scan_topics` | true / 1.0 / [/lidar_front/scan, /lidar_rear/scan] | 하나라도 끊기면 SCAN_LOST |
| `require_detections` / `detection_frames` / `detection_timeout_s` | true / ["d435_color_optical_frame"] / 1.0 | |
| `require_localization` / `loc_timeout_s` | true / 2.0 | |
| `loc_ready_xy_std_m` / `loc_ready_yaw_std_deg` | 0.10 / 10.0 | amcl 만: start 허용 기준 (시험용 초기값) |
| `loc_lost_xy_std_m` / `loc_lost_yaw_std_deg` / `loc_fault_s` | 0.30 / 30.0 / 1.0 | 실행 중 상실 기준 (표준편차는 amcl 만) |
| `monitor_recover_s` / `monitor_max_recoveries` / `monitor_fault_s` | 2.0 / 2 / 10.0 | scan · 검출 끊김 |
| `action_retry_s` / `action_retry_max_s` | 1.0 / 15.0 | |
| `server_wait_s` | 30.0 | |
| `nav_timeout_s` / `arm_timeout_s` | 60.0 / 60.0 | |
| `read_timeout_s` | 5.0 | |
| `cargo_slots` | S1 … S12 | 12 칸 = URDF `rack_cell_S*`, 팔에서 먼 칸부터 = 채우는 순서 (dice 는 11 칸 이상 필요) |
| `cargo_file` / `cargo_restore` | "" / false | |
| `puck_standoff_m` | 0.30 | 퍽 앞에 설 거리 (임시, 팔 닿는 범위 받으면 조정) |
| `pick_fresh_s` | 2.0 | 이보다 오래된 위치로 안 집음 |
| `puck_half_height_m` | 0.03 | |
| `grasp_offset_m` | 0.015 | |
| `nav_retries` / `pick_retries` / `place_retries` | 1 / 2 / 1 | |

launch

```bash
ros2 launch mission mission.launch.py
ros2 launch mission mission.launch.py use_sim_time:=true require_motion_mode:=false
ros2 launch mission mission.launch.py dice_map:=<다른 파일> params:=<다른 yaml>
```

- 설정 yaml 순서: `params` 인자 → 레포 `config/planning.yaml`(bringup share) → 코드 기본값
- 지도 폴더는 빌드 때 정해짐: mission `-DMISSION_MAPS_DIR=<경로>` (예전 이름 `PLANNING_MAPS_DIR` 도 받음). 기본 = 레포 `maps/`

---

## 12. 바코드 해석 [구현, 발제 v5 형식 (2026-11-01)]

- 출발 구역 벽 Code128 하나. `task_parser` → `dice.hpp parse_dice_barcode`

| 원문 | 뜻 |
|---|---|
| `P2-01` · `P2-03` | 오름차순: 신호 횟수 적은 그룹부터 L1 → L2 → L3 |
| `P2-02` · `P2-04` | 내림차순: 많은 그룹부터 |
| 그 밖 | 미등록 바코드 → **바로 FAULT** (`BARCODE_UNREGISTERED`, 발제: 오류 정지) |

| 규칙 | 내용 |
|---|---|
| Task | action `dice`, `dice_order`, `task_id` = 원문 |
| 미등록 바코드 | 바로 FAULT, detail 에 원문. resume → start 하면 HOME 부터 다시 읽음 |
| 안 읽힘 | `read_timeout_s` 안에 아무것도 안 오면 단계 실패 → 복귀 |
| 단계 생성 | `dice_steps` (COLLECT · DELIVER · VERIFY), 운반 계획 `transfer_planner` |
| 옛 시험 형식 | `move` 흐름은 2026-11-01 삭제 |

---

## 13. 다른 파트와 합의할 것

| 항목 | 상대 | 내용 | 상태 |
|---|---|---|---|
| target으로 IK | control | 6절 약속대로 target(그리퍼 중심 · 쥘 끝 방향) 사용 | 바꿀 예정 확인, 세부 대기 |
| 팔 닿는 범위 | control | 몸 중심 기준 최소 · 최대 거리, 높이, 좌우 각도 → `puck_standoff_m` | 대기 |
| 누운 퍽 세우기 | control | joint5 ±90° 제한으로 못 세우는 자세 | 대기 |
| ArmTask 확장 | control | STORE · RETRIEVE · PUSH · LOOK, slot, error, holding | 대기 |
| 퍽 쥔 채 주행 (CARRY) | control | dice 운반 계획이 그리퍼에 퍽 하나 든 채 주행할 수 있음 | 대기 |
| 퍽 자세 약속 | perception | 자세 z 축 = 긴축, 모르면 w=0 직접 | 전달 필요 |
| 검출 frame_id | perception | URDF 이름 `d435_color_optical_frame`(D435 컬러 렌즈, 케이스 가운데 `d435_cad_optical_proxy_frame` 에서 광학 −x 32.5 mm, 2026-10-29 추가), `birdeye_camera_optical_frame`과 맞추기 | 전달 필요 |
| 인터페이스 전체 | perception | `Detection.id/shape`, `TaskText`, `TrafficLight` | 전달함, 답 대기 |
| `/mission/hold` → `cmd_vel_hold` | 주행 | Nav2 → collision_monitor → `cmd_vel_hold` → `/cmd_vel` (TwistStamped), twist_mux 안 씀. `/cmd_vel` 을 이 경로 하나만 내는지 확인 | 전달 필요 |
| Nav2 · AMCL base frame | 주행 | `base_footprint` (swerve가 `odom → base_footprint` 발행) | 전달 필요 |
| 정밀 정렬(approach) 필요 여부 · 담당 | control / 주행 | IK 범위 보고 결정 | 미정 |

## 14. 미정 (대회 정보 필요)

- 바코드 내용 형식 · 크기 · 부착 높이 · 개수, 바코드 하나에 task 몇 개
- 도착지 표시 방식 (지도 좌표 / 태그 / 색 구역)
- 제한 시간, 채점 기준, 출발점 복귀 필요 여부
- 적재함 칸 수 (팔이 닿는 칸 기준)
- 퍽 AprilTag 계열(tag36h11 등) · 크기
- 시연 때 퍽 배치가 매핑 때와 같은지
