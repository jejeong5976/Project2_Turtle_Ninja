# 프로젝트 2 — 주사위 기억 로봇 (Turtle Ninja)

- 신호등 12 점멸과 바코드로 작업 조건을 읽음
- 퍽 12 개를 모아 L1–L3 에 주사위 눈 모양으로 놓음
- 새 카메라 관측으로 배치를 확인하고 출발 구역으로 돌아옴

- 결과 (1 회차)
	- 12/12 배치, 재검증 3/3, 출발 구역 복귀
	- 사람 개입 5 번
	- 상세: `report.md` 2 절
- 시험 결과: [`results/test-results.csv`](results/test-results.csv), 실패 원인 · 수정 전후 비교: [`report.md`](report.md) 3 절, 한계: 4 절

## 1. 폴더

| 경로 | 내용 |
|---|---|
| `report.md` | 시스템 구성, 1 회차 경기 결과 · 기록, 수정 전후, 한계 |
| `docs/` | `architecture.md` · `contributions.md` |
| `results/` | `test-results.csv` · 로봇이 남긴 `signal.json` · `allocation.json` · `verify.yaml` · `config/` (경기 설정 사본) |
| `results/figures/` | 1 회차 그림 7 개 (경로 · 타임라인 · 신호 · 바코드 · 재검증) + 경기 영상에서 자른 GIF 13 개 (신호 · 수집 · 정밀 접근 · 배치 · 재검증 · 복귀 · 자동 회복 · 사람 개입 5) |
| `results/logs/` | 1 회차 로그 (mission_manager · navigation · 인지 처리율 · 바코드 판독 · 실행 스크립트) |
| 코드 | 저장소 `ros2_ws/src/` (perception · mission · planning · navigation · manipulation · swerve_controller · hardware · description · interfaces · bringup), 설정 `config/`, 지도 `maps/`, 실행 도구 `tools/jetson/` |

## 2. 장비 · 버전

| 항목 | 값 |
|---|---|
| 로봇 | 상세 구성 = `docs/architecture.md` 1 절 |
| 컴퓨터 | Jetson Orin Nano 8 GB (JetPack 7.2.1, Ubuntu 24.04), OpenCR × 2 |
| ROS | ROS 2 Lyrical, slam_toolbox, Nav2, ros2_control, `ROS_DOMAIN_ID=17` |
| 추론 | TensorRT (`.engine`, Jetson 에서 `trtexec --fp16` 변환) |
| 라이브러리 | OpenCV, zxing-cpp (바코드), librealsense, yaml-cpp, coin_d4_driver (LDS-03) |

- 설치 목록: 저장소 `docs/setup/jetson_install_list.md`, Jetson 기본 설치 `docs/setup/jetson_orin_nano_ssd_setup.md`

## 3. 설치 · 빌드 (Jetson)

```bash
git clone <저장소> ~/p2 && cd ~/p2 && git checkout project2-submit
bash tools/jetson/jetson_check.sh     # 장치 · 패키지 확인 (읽기만)
bash tools/jetson/build_jetson.sh     # colcon 빌드
```

- 경기 설정 사본을 Jetson 홈에 둠

```bash
cp results/config/planning_full.yaml ~/planning_full.yaml
cp results/config/dice_map_arena7b.yaml ~/maps/dice_map_arena7b.yaml
cp results/config/arena7.yaml results/config/arena7.pgm ~/maps/   # 경기 지도 (AMCL 용)
```

- 모델 엔진: `~/models/best_wide.engine` · `~/models/best_wrist.engine` + 같은 이름 `.names.txt` (launch 기본 경로, 없으면 실행 거부)
  - TensorRT 엔진은 장치마다 따로 만듦 → Jetson 에서 변환

```bash
python yolo/wide/scripts/export_onnx.py --weights <광각 best.pt>     # → ONNX + .names.txt (544×960)
python yolo/wrist/scripts/export_onnx.py --weights <손목 best.pt>    # → ONNX + .names.txt (480×640)
trtexec --onnx=<모델>.onnx --saveEngine=$HOME/models/best_wide.engine --fp16    # 손목도 같은 방법 → best_wrist.engine
```

- `.names.txt` 도 엔진 이름에 맞춰 `~/models/best_wide.names.txt` · `best_wrist.names.txt` 로 둠
- 저장소: https://github.com/jejeong5976/Project2_Turtle_Ninja , 제출 버전 = `project2-submit` 태그

## 4. 실행 순서

1. 경기 초기화
	- 퍽 12 개를 시작 박스에 놓음 (박스마다 2 개, 태그가 위를 보게)
	- L1–L3 박스를 비움, 신호등은 꺼진 상태
	- 로봇을 출발 구역 표시 자리 (지도 딸 때 자세) 에 놓음
2. 실행 (Jetson 터미널)

```bash
cd ~/p2
bash tools/jetson/run_robot.sh mission opencr ~/maps/arena7 dice_map:=$HOME/maps/dice_map_arena7b.yaml planning_params:=$HOME/planning_full.yaml dds_transport:=DEFAULT dock:=rl rack_tags:=true localization:=amcl initial_pose:=-0.357,-0.005,-1.1 mission_loc:=amcl nav_policy:=nav_nf5 dock_policy:=dock3 dock_tol:=0.015 dock_search:=0.30
```

3. `STANDBY 준비 완료` 가 뜨면 신호 시작과 함께 다른 터미널에서

```bash
export ROS_DOMAIN_ID=17
ros2 service call /mission/start std_srvs/srv/Trigger '{}'
```

4. 상태 보기 `ros2 topic echo /mission/status`, 결과 = `~/planning_logs/run_<시각>/` (signal · allocation · verify)
	- 로봇 없이 연결만 확인: `bash tools/jetson/run_robot.sh mission mock
	- 포트가 기본 udev 이름과 다르면 `DRIVE_PORT=… ARM_PORT=…` 를 앞에 붙임

## 5. 정지 · 초기화

| 상황 | 방법 |
|---|---|
| 끝내기 | 실행 터미널 Ctrl+C (`/cmd_vel` 이 0.3 s 끊기면 바퀴 스스로 정지) |
| 오류 뒤 | FAULT 면 로봇이 멈춰 있음, `/mission/resume` 전까지 움직이지 않음 (경기에선 같은 회차 재개 안 함) |
| 신호 다시 기록 | `ros2 service call /mission/reset_signal std_srvs/srv/Trigger '{}'` |
| 다음 회차 | Ctrl+C → 퍽 · 로봇 초기 배치 → 다시 실행 |

## 6. 재현 시 주의

- 지도 딸 때와 같은 출발 자세에 놓아야 위치 추정이 맞음 (`initial_pose`)
- 박스 실제 자리가 지도와 다르면 로봇이 라이다로 다시 재서 정차점을 고침
- 다른 터미널도 `ROS_DOMAIN_ID=17` 을 먼저 설정함
- 실행 중 새 ros2 명령을 자주 띄우면 DDS 탐색으로 잠깐 멈칫할 수 있음
- 배터리 전압이 낮으면 바퀴 · 팔 모터 오류 가능 → 충전 상태로 시작
- 비밀번호 · 토큰은 저장소에 없음
