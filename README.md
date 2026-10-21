# Project2_Turtle_Ninja

미로 맵에서 벽에 붙은 QR 문제를 읽고, 색 퍽을 구분해 밀어 넘어뜨리거나 옮기는 로봇 (Physical AI 과제 2)

## 미션

- 주행 중 벽의 QR(JSON)을 찾아 판독하고 해당 task 수행
- 색 퍽을 구분해 밀어서 넘어뜨리거나 지정 위치로 이동
- 인식 모델 Yolo 이용
- SLAM 기반 자율주행. 맵은 시연 1주 전 공개, 출발 지점 고정

## 일정

| 날짜 | 내용 |
|---|---|
| 11/4 (수) | 맵 공개 |
| 11/11 (수) | 시연 |

## 하드웨어

| 구성 | 사용 부품 |
|---|---|
| 구동 | 4륜 swerve drive (조향 XM430-W210 ×4, 구동 XM430-W350 ×4) |
| 온보드 컴퓨터 | Jetson Orin Nano |
| 제어 보드 | OpenCR 1.0 ×2 (바퀴용, 팔용) |
| 라이다 | LDS-03 |
| 매니퓰레이터 | OpenManipulator-X + 손목 회전축 1개 |
| 카메라 | RealSense D435 (손목 장착), Arducam UC-684 (상단 광각) |

## 개발 환경

- ROS 2 Lyrical
- 시뮬레이션: Isaac Sim 6.2

## 디렉토리 구조

```
Project2_Turtle_Ninja/
├── config/                  # 팀 공용 설정 예약 (현재 노드 설정은 각 패키지 config/)
├── firmware/                # OpenCR 펌웨어
├── ros2_ws/                 # ROS 2 워크스페이스
│   └── src/
│       ├── bringup/             # 전체 실행 launch, 실기/시뮬 전환
│       ├── description/         # URDF/xacro, 메쉬 (RViz·Isaac Sim 공용)
│       ├── interfaces/          # 공용 메시지·액션 정의
│       ├── hardware/            # OpenCR 2대(바퀴/팔) Dynamixel 통신
│       ├── swerve_controller/   # cmd_vel → 바퀴별 조향각/속도, 휠 오도메트리
│       ├── manipulation/        # 팔 제어, 밀기·집기·적재함 동작
│       ├── navigation/          # SLAM, Nav2 설정, 맵
│       ├── perception/          # YOLO 추론, QR 디코딩, 퍽 색, 신호등
│       └── mission/             # QR JSON → task 변환, 미션 순서 관리
├── yolo/
│   ├── dataset_gen/         # Isaac Sim 합성 데이터 생성 코드
│   ├── datasets/            # 생성·수집한 데이터셋 (git 제외)
│   ├── train/               # 학습·평가·.onnx 변환
│   └── models/              # .pt / .onnx / .engine (git 제외)
├── sim/
│   └── robot_sim/           # Isaac Sim 로봇 주행 시뮬레이션 (ROS 2 연동)
└── docs/                    # 설치 기록, 설계 문서, 가이드
```

## 빌드

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## 패키지와 설정

ROS 패키지 이름은 `src` 디렉터리 이름과 같다. `navigation`은 아직 예약 폴더다.
`swerve_controller`는 바퀴 계산·제어 플러그인, `manipulation`은 팔 계산과
`control_master`, `mission`은 `planning_master` 미션 상태머신을 제공한다.
실행 노드 이름과 토픽·액션 주소는 유지한다.

설정은 각 패키지의 `config/`에 유지하며 설치 후 `share/<package>/config/`에서 읽는다.
`firmware/nt_opencr`와 통신 프로토콜은 그대로 유지한다.

패키지 이름이 바뀐 checkout은 기존 설치본을 재사용하지 말고 별도로 빌드한다.
공용 타입은 `interfaces/msg/*`, `interfaces/action/ArmTask`이므로 외부 소비자도
이 타입 이름으로 함께 빌드해야 한다. 주행·팔 실물 실행은 별도 하드웨어 검증이 필요하다.

하드웨어 없는 시험:

```bash
source /opt/ros/lyrical/setup.bash
cd ros2_ws
CMAKE_BUILD_PARALLEL_LEVEL=1 colcon build --executor sequential
source install/setup.bash
ROS_DOMAIN_ID=181 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST colcon test --executor sequential \
  --packages-select hardware manipulation mission swerve_controller
colcon test-result --verbose
```

`perception`에는 아직 자동 시험이 없다. 위 명령은 기존 시험이 있는 패키지만 실행한다.

## Git 규칙

### 브랜치

| 브랜치 | 용도 | 머지 |
|---|---|---|
| `main` | 팀 전체 통합 코드 (기본 브랜치) | 승인 1개 후 관리자만 |
| `feat/...` 등 | 각자 작업하는 곳 | - |

- 흐름: `feat/...` → `main`
- `main`에는 직접 push 불가. PR로만 반영
- 실기 테스트를 통과한 시점은 `main`에 태그로 표시 (예: `tested-1020`, `v1.0-demo`)

### 브랜치 이름

`타입/내용` 형식, 소문자와 하이픈만 사용

| 타입 | 용도 | 예시 |
|---|---|---|
| `feat/` | 새 기능 | `feat/swerve-ik` |
| `fix/` | 버그 수정 | `fix/qr-decode` |
| `docs/` | 문서 | `docs/jetson-setup` |
| `chore/` | 폴더 구조, 설정, 빌드 스크립트 | `chore/init-structure` |

### 작업 방법

```bash
# 1. main 최신화 후 브랜치 생성
git checkout main
git pull
git checkout -b feat/swerve-ik

# 2. 작업 후 커밋, push
git status
git add <파일>
git commit -m "feat: swerve 역기구학 추가"
git push -u origin feat/swerve-ik
```

3. GitHub에서 PR 생성 (대상: `main`)
4. 승인 1개 받은 뒤 관리자가 **Squash and merge**

### 커밋 금지

- 빌드 산출물, 데이터셋, 모델 가중치, rosbag
- 비밀번호, 토큰, SSH 키
