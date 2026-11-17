# Isaac Lab 시뮬레이션

Isaac Lab 3.0 (Isaac Sim 6.2) 에서 로봇·실제 미로·퍽/바코드/신호등을 띄운 환경을 통해 시뮬레이션

## 1. 준비

**Isaac Lab 3.0** 이 설치되어 있어야 한다. 아래 `ISAACLAB_PY` 는 각자 설치한 Isaac Lab 파이썬 경로로 바꿀 것

```bash
export ISAACLAB_PY=~/IsaacLab/.venv/bin/python      # 각자 Isaac Lab 파이썬 경로
export OMNI_KIT_ACCEPT_EULA=YES
```

**대용량 파일 받기** (GitHub 에 없음): [Google Drive](https://drive.google.com/drive/folders/1l8WlhxTULpfTjzqxAcxq03mRZiVnJxpc?usp=drive_link) 에서 `turtle_ninja_sim_large_files.zip` 을 받아 **레포 루트에서** 푼다. 경로 수정은 필요 없다.

```bash
cd Project2_Turtle_Ninja
unzip turtle_ninja_sim_large_files.zip
```

| 풀리는 위치 | 내용 |
|---|---|
| `sim/robot_sim/real_photos/`, `sim/robot_sim/source/maze_parts_*` | 실제 사진, 미로 부품 원본 |

**로봇** = 팀 URDF (`ros2_ws/src/description`, 실물). 시뮬용 파일은 GitHub 에 없고 한 번 만들어 씀 (URDF 가 바뀌면 다시):

```bash
source /opt/ros/lyrical/setup.bash && source <ws>/install/setup.bash   # description · bringup 패키지를 빌드한 ws (xacro 가 찾음)
python3 tools/convert_robot.py urdf                                   # -> robot/robot.urdf · robot/swerve.yaml
OMNI_KIT_ACCEPT_EULA=YES $ISAACLAB_PY tools/convert_robot.py usd     # -> usd/robot/robot/robot.usda
```

- 바퀴 관절 `steer_<fl|fr|rl|rr>_joint` · `drive_<..>_joint` (팀 `swerve_controller` 이름), 팔 `joint1~5`, 그리퍼 `gripper_left/right_joint` (직선). 그리퍼 연결기구 (크랭크 · 막대) 는 변환 때 고정
- 루트 = `base_footprint` (바퀴 바닥), `base_link` 는 그 0.033 m 위. 센서는 USD 에서 `base_footprint` 몸체에 붙음 (변환 때 base_link 가 합쳐짐)
- 라이다 스캔면 바닥 위 60 mm, 앞 라이다 yaw 180° · 뒤 0° (실물 장착 방향)
- 주행 = `sim_core/nt_swerve.py` (팀 `swerve_controller` 규칙을 옮긴 것) 를 100 Hz 로: 같은 선 위 조향 선택, 출발 때 바퀴 바깥쪽, 범위 끝 1° 여유, 동시 조향 · 구동. 값은 `robot/swerve.yaml` (= `sim_core/swerve_base.yaml` + 모듈 위치 `config/controllers.yaml` · 조향 범위 URDF). 학습용 한계 0.261 m/s · 1.675 rad/s, 실물 값은 `use_real_speed_limits` (브리지 `--real_speed`)
- 팔 자세 (실물 설정 그대로): 시작 · 주행 = `config/control.yaml` poses.home, 준비 P0 · 관찰 LOOK = `config/arm_paths.yaml` look 경로. 손가락 열림 58.5 mm (q 0.0165) · 닫힘 URDF 하한
- 관절 이득: 조향 20 / 0.3, 바퀴 감쇠 1.0, 팔 500 / 10, 그리퍼 1000 / 2 · 20 N. 팔 힘 · 속도 한계 = URDF
- 자기 충돌은 끔 (팔을 고정하고 주행 · 촬영만 하므로)
- 손목 D435 를 쓰면 `hide_wrist_camera_body` 로 D435 케이스 메시를 숨김 (URDF 광학 프레임이 케이스보다 뒤라 그대로면 화면을 가림)
- `robot_description/` = 배포판 URDF 남은 것 (control 도구용 임시, 그 폴더 README)

로봇 부품 색은 항상 실물 색 (D435 은색, 받침대 크림색, 팔 · 손가락 · 모터 · 조향 모듈 · 라이다 판 검정), 선반 = 골판지 박스색

| 옵션 | 뜻 |
|---|---|
| `--real` | 실제 강의실 배경(파노라마) + 실제 카펫 바닥 + 천장 조명 6개 + 벽판 유광 반사 (2-6 데이터와 같은 겉모습, 빼면 기본 회색 배경) |
| `--lidar` | 앞 · 뒤 라이다(레이캐스트, 360° · 1°, 최대 8 m 가정) 를 켜고 맞은 점을 창에 표시. 스캔 높이 바닥 위 6.0 cm (`base_scan_front/rear`) = 선반(7 cm) 은 잡히고, 바닥에 선 퍽(6 cm) 은 윗면과 같은 높이라 거의 안 잡힘. 로봇 자기 몸체에 가려지는 각도는 없음 |
| `--render pt` | PathTracing (2-6 데이터 기본). 움직이는 동안 노이즈, 멈추면 깨끗함. 기본 `rt` |
| `--dome_yaw 90` | 배경 사진을 90° 돌림 |
| `--arm_extended` | 팔을 접지 않고 편 자세로 |
| `--num_envs N` | 미로 N 개 동시에 |
| `--device cpu` | 물리를 CPU 로 (env 1개일 때 GPU 보다 약 8배 빠름: 스텝당 11 ms vs 88 ms. 렌더링은 GPU 그대로) |

### 2-6. YOLO 학습 데이터 만들기

```bash
$ISAACLAB_PY scripts/make_yolo_dataset.py --scenes 400 --views 2 --num_envs 1 --maze rand_00 --seed 0     # 1 × 400 × 2 = 800장 (미로마다 seed 다르게)
```

- 결과: `Project2_Turtle_Ninja/yolo/wide/datasets/sim/` (이미지·라벨·`data.yaml`)
- `preview/` 폴더에서 라벨 상자를 그린 그림을 먼저 확인할 것
- 클래스: 빨간 원기둥 · 초록 원기둥 · 빨간 직육면체 · 초록 직육면체 · 바코드 · 신호등 (적게 나온 클래스를 우선 찍어 골고루 맞춤)
- 헷갈리기 쉬운 경우도 섞음: 넘어진·바닥에 떨어진 퍽, 불 켜진 신호등, 가짜 종이(QR·글자·체크무늬. 바코드 닮은 줄무늬는 진짜와 구분 안 돼 뺌), 다른 모양의 색 물체 (`--p_fallen`, `--p_floor`, `--no_distractors`)
- `--num_envs` 는 1 (기본 PathTracing 렌더는 2 부터 GPU 메모리 부족으로 디노이저가 빠짐)
- 조명 · 반사 (장면마다 무작위, 실제 영상과 비교해 맞춘 값):
  - 배경 파노라마 돔 0.25–1.1배 + 천장 사각 조명 6개 (세기 10000 × 0.7–1.3, `--ceil_int 0` 이면 끔)
  - 신호등: 꺼짐 · 빨 · 노 · 초 무작위, 켜진 불 세기 = 실제 영상에 맞춘 값(`TL_LAMP`)의 0.7–1.5배 (`--tl_glow_min/max`)
  - 벽판 유광 반사: 굴절률 2.2–3.0 · 거칠기 0.02–0.15 (`--panel_ior`, `--panel_rough`). 퍽 · 박스 · 로봇이 벽에 비침 (비친 상은 라벨 없음 = 오탐 대비)
  - 선반 절반은 벽에서 0.05–0.35 m 떼고 절반은 모서리 쪽에 붙임 (`--p_shelf_off`, `--shelf_off`, `--p_shelf_corner`), 시점 25 % 는 벽을 비스듬히 봄 (`--p_graze`)
- 미로: `--maze real`(기본, 팀 A 경기장) 또는 `--maze rand_03` (`tools/random_maze.py` 무작위 미로). 실행 한 번에 미로 하나 -> 미로마다 나눠 같은 폴더에 쌓음
- YOLO 학습 코드는 `yolo/wide/` (광각) · `yolo/wrist/` (손목)

### 2-7. 손목 D435 학습 데이터 만들기

```bash
$ISAACLAB_PY scripts/make_wrist_dataset.py --scenes 20 --views 2 --check
```

- 선반 앞에서 팔을 관찰 자세로 두고 손목 D435 로 찍음: 컬러 · 깊이 · 퍽 마스크 · 퍽 정답 자세(카메라 좌표)
- 결과: `Project2_Turtle_Ninja/yolo/wrist/datasets/sim_wrist/`. 학습은 `yolo/wrist/README.md`
- 조명 · 벽판 반사 범위는 2-6 과 같음

### 2-8. 렌더 그림 저장

```bash
$ISAACLAB_PY scripts/snapshot_props.py --real --out results/snapshots
```

미로 전체·신호등·선반 근접 그림을 `results/snapshots/` 에 저장

### 2-9. 주사위 기억 미션 경기장 (발제 v5)

발제 맵 그대로 (4464 × 6364 mm, 벽 · 시작 박스 6 · L1–L3 · 중앙 받침대 · 장소 태그 · 바코드 · 신호등, 퍽 12개). 랜덤 미로와 따로 둠

```bash
$ISAACLAB_PY tools/make_dice_map.py --problem mazes/dice_problem_q1.json   # 처음 한 번 · 문제 바꿀 때
$ISAACLAB_PY scripts/view_dice.py --viz kit --keyboard --real               # 출발점 (734, 950) 에서 −X 방향으로 시작
```

- 신호등은 시작 5초 뒤 문제 파일 순서대로 점멸 (1 s 켜짐 + 1 s 꺼짐, 12번). `--signal RGRRGRGGYGYG` 로 바꿈, `--signal_delay -1` 이면 점멸 안 함
- 신호등 겉모습 = 실제 영상(`yolo/wide/datasets/real_frames/wide_traffic_light_0`)에 맞춘 값 (`sim_core/turtleninja_maze_env.py` 의 `TL_LAMP` · `TrafficLightLook`): 꺼진 렌즈 색, 켜진 칸 가운데 흰색 포화, 주변 바닥 · 벽 빛 번짐 (등마다 작은 구형 조명). 2-6 데이터 생성기 · 2-10 Nav2 브리지도 같은 값
- `--camera` 광각 · 손목 카메라, `--lidar` 라이다. `--snap results/dice` 면 창 없이 광각 · 손목 영상 4장 저장 + 퍽 상태 확인 후 끝
- 문제 파일 (`mazes/dice_problem_*.json`) = 신호 · 바코드 · 퍽 ID별 색 · 모양 · 시작 박스 자리. Q1 파일의 퍽 배정은 가정 (placement.csv 받으면 바꿈)
- 좌표: `mazes/dice_layout.json` 에 경기장 mm (발제 좌표) 와 시뮬 m (경기장 중심 원점 = 발제 − (2202, 3152)) 둘 다 있음
- 발제에 없는 값 (신호등 크기, 바코드 받침, 태그 면, 출발 구역 크기, 3030/R3 구분) 은 `tools/make_dice_map.py` 머리말의 가정

### 2-10. Nav2 주행 시험 (ROS 2 연동)

시뮬 로봇에 Nav2 를 붙여 목표 주행을 시험함. 경기장 = 2-9 주사위 경기장, 출발점에서 시작. ROS 2 Lyrical + Nav2 설치 필요

```bash
# 터미널 1: 시뮬 + ROS 브리지 (물리 CPU, 실제 시간에 맞춤). 창 보려면 --viz kit --camera --real
ROS_DOMAIN_ID=18 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST scripts/sim_nav_bridge.sh

# 터미널 2: navigation 패키지 빌드 후 robot_state_publisher + Nav2 + RViz
cd ../../ros2_ws && colcon build --packages-select navigation && source install/setup.bash && cd ../sim/robot_sim
ROS_DOMAIN_ID=18 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ros2 launch scripts/sim_nav2.launch.py
```

- RViz 의 `Nav2 Goal` 로 목표를 찍으면 감. AMCL 은 출발 자세로 시작 (`initial_pose:=x,y,yaw°`, 기본 = `dice_layout.json` 출발점)
- 브리지 (`scripts/sim_nav_bridge.py`, 실행은 `.sh`): 내보냄 `/clock` · `/odom` · TF `odom -> base_footprint` · `/joint_states` · `/lidar_front/scan` · `/lidar_rear/scan` (레이캐스트, 360 · 1°, 60 mm) · `/sim/ground_truth`, 받음 `/cmd_vel` (TwistStamped) -> 스워브 컨트롤러 (`--real_speed` = 실물 속도 한계). `/odom` 은 정답 위치 (잡음 없음)
- 라이다 자기 몸 가림: 바퀴 · 조향 모듈 · 차체 기둥 · 반대쪽 라이다에 가린 빔은 돌아오지 않음(inf) 처리 (`sim_core/lidar_self.py`, URDF 메시를 스캔 높이에서 자른 단면, 바퀴는 조향 각도를 따라 돎). 앞 라이다 기준 360 빔 중 약 100 개가 가림. `view_maze.py --lidar` 표시는 가림 없음
- 비상 정지 (Nav2 `collision_monitor`): 진행 방향 1 s 앞 충돌 예상이면 감속 · 정지 (`FootprintApproach`), 차체 + 4 cm 안에 점이 들어오면 즉시 정지 (`PolygonStop`). 시험용 사람 = `/sim/person` (geometry_msgs/PointStamped, map xy, z < 0 이면 치움). 확인: 0.21 m/s 주행 중 앞면 11 cm 앞에 세우면 0.38 s 에 정지 (앞면 사이 8.4 cm), 치우면 다시 가서 도착
- 실물 주의: 실물 라이다는 자기 바퀴 · 기둥을 실제로 맞힘 (외곽에서 0.08–0.15 m) -> 그대로 두면 비상 정지 구역에 걸려 계속 멈춤. 실물 스캔은 몸에 맞은 점을 걸러야 함 (`laser_filters` 상자 필터 등, Lyrical 에 미설치)
- Isaac 파이썬에서 시스템 Lyrical `rclpy` 를 씀 (`.sh` 가 ROS 경로와 순수 파이썬 모듈 `em` · `lark` 만 덧붙임, 시스템 패키지 `python3-empy` · `python3-lark` 필요)
- 지도 = `maps/maze_dice_map.yaml`: 경기장 배치에서 라이다 높이(60 mm) 단면을 그린 것 (`python3 tools/layout_to_occupancy.py --maze dice`). 실제 경기는 slam_toolbox 지도로 바꿈 (`map:=`)
- Nav2 설정 = 레포 `config/nav2.yaml` (스워브: AMCL `OmniMotionModel`, MPPI `omni`), 실행 = `ros2_ws/src/navigation`
- `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` 필수: 같은 망에서 같은 도메인의 실물 로봇이 켜져 있으면 시뮬 `/cmd_vel` 이 실물로 갈 수 있음. 도메인도 실물(17)과 다르게 (예 18)
- 확인: 출발점 -> L1 접근 지점 (3230, 950) mm, 시작 박스 1 을 돌아 21 s, 위치 오차 12 mm. 홀로노믹이라 진행 방향과 관계없이 몸 방향을 둔 채 이동하고 도착 때 돌아섬

## 3. 폴더

| 폴더 | 내용 |
|---|---|
| `scripts/` | 실행 스크립트 (위 실습), Nav2 시험 브리지 · launch (`sim_nav_bridge.*`, `sim_nav2.launch.py`) |
| `sim_core/` | 환경 정의: 로봇·스워브 제어·미로·소품·카메라 (`turtleninja_maze_env.py`), 배치 고정 (`fixed_layout.py`), 팔 무작위 · 관찰 자세 (`arm_poses.py`), 주사위 미션 경기장 · 신호등 점멸 (`dice_mission.py`), 스워브 컨트롤러 (`nt_swerve.py` · `swerve_base.yaml`) |
| `usd/` | 시뮬레이션용 3D 파일: 로봇 (`robot/`), 미로 (`maze_real.usd`, 무작위 미로 `mazes/`), 카펫 바닥, 소품 (`props/`: 퍽·태그·바코드·신호등), 배경 텍스처 (`textures/`) |
| `robot/` | 시뮬용 팀 URDF · 스워브 설정 (`tools/convert_robot.py` 가 만듦, GitHub 에 안 올림) |
| `robot_description/` | 배포판 URDF 남은 것 (control 도구용 임시) |
| `maps/` | Nav2 시험용 지도 (`tools/layout_to_occupancy.py`) |
| `source/` | 미로 배치 JSON, 미로 부품 원본 |
| `tools/` | 3D 파일 다시 만들기 (아래) |
| `rl/` | 강화학습 코드 자리 (`rl/README.md`) |

## 4. 로봇·미로·소품이 바뀌었을 때 (3D 파일 다시 만들기)

평소에는 필요 없다. 아래 원본이 바뀌었을 때만 실행한다.

| 바뀐 것 | 명령 | 필요한 것 |
|---|---|---|
| 팀 URDF (`ros2_ws/src/description`) | 위 '로봇' 의 `tools/convert_robot.py urdf` · `usd` -> `robot/`, `usd/robot/` | ROS (xacro), Isaac Lab |
| 미로 배치 (`source/*.json`) | `python3 tools/layout_to_maze_usd.py` | `pip install usd-core` |
| 무작위 미로 (YOLO 데이터용) | `python3 tools/random_maze.py --count 8 --seed 0` -> `mazes/`, `usd/mazes/` | usd-core (또는 `$ISAACLAB_PY`) |
| 태그·바코드·소품 | `python3 tools/make_tag_barcode_images.py` 후 `python3 tools/make_props_usd.py` | OpenCV, usd-core |
| 주사위 미션 경기장 · 문제 | `$ISAACLAB_PY tools/make_dice_map.py --problem mazes/<문제>.json` -> `mazes/dice_layout.json`, `usd/mazes/dice.usd`, `usd/props/dice/` | usd-core (`make_props_usd.py` 함수 사용) |
| 실제 사진 (`real_photos/`) | `python3 tools/photos_to_textures.py` 후 `python3 tools/layout_to_maze_usd.py` | OpenCV, usd-core |
| Nav2 시험 지도 | `python3 tools/layout_to_occupancy.py --maze dice` (또는 `real`, `rand_03`) -> `maps/maze_<이름>_map.{pgm,yaml}` | numpy |
