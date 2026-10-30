# Project2_Turtle_Ninja

미로 벽의 바코드로 task를 읽고, AprilTag가 붙은 퍽을 찾아 옮기거나 밀어 넘어뜨리는 로봇 (Physical AI 과제 2)

## 미션

- 주행 중 벽의 1D 바코드를 찾아 판독하고 해당 task 수행
- task가 지정한 퍽을 AprilTag ID로 찾아 지정 위치로 옮기는 등의 mission을 수행
- 퍽 색: 빨강 · 초록 · 베이지
- 신호등 인식 후 정지 · 진행 판단
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
├── config/                  # 노드 파라미터, 카메라 보정값 yaml
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
│       ├── perception/          # YOLO 추론, 바코드 판독, 퍽 AprilTag · 색, 신호등
│       └── mission/             # 바코드 → task 변환, 미션 순서 관리
├── yolo/
│   ├── wide/                # 광각 상자 YOLO (퍽·바코드·신호등) 학습·평가·변환
│   └── wrist/               # 손목 D435 퍽 seg + 깊이 자세 추정
│                            #   (각 폴더의 datasets/ · runs/ · 가중치는 git 제외)
├── sim/
│   └── robot_sim/           # Isaac Sim 로봇 주행 시뮬레이션 (ROS 2 연동)
├── tools/                   # ROS 밖에서 단독 실행하는 개발 도구
│   ├── camera_calib/        # 카메라 캘리브레이션 사진 수집 (OpenCV)
│   ├── d435_viewer/         # 손목 D435 실시간 퍽 자세 보기 · 사진 저장
│   └── wrist_calib/         # 손목 D435 장착 자세 (핸드-아이) 캘리브레이션
└── docs/                    # 설치 기록, 설계 문서, 가이드 (코드 X, 문서·이미지만)
```

## 빌드

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## 제어 패키지 검증

`swerve_controller`는 바퀴 제어 플러그인, `manipulation`은 팔 계산과 `control_master`를 제공합니다.
이 PR에는 `mission`과 `perception` 구현을 포함하지 않습니다. 두 폴더는 담당자용 자리로 유지합니다.
공용 타입은 `interfaces/msg/MotionMode`, `interfaces/msg/MissionStatus`, `interfaces/action/ArmTask`입니다.
패키지 이름이 바뀐 checkout은 이전 설치본을 재사용하지 말고 새 build/install 경로에서 빌드하세요.
실물 주행·팔 실행은 별도 하드웨어 검증이 필요합니다.
아래 검증은 6개 패키지 전체 설치 기준입니다.
`description`만 단독 설치한 환경의 기본 표시 실행은 지원하지 않습니다.

하드웨어 없는 단위시험:

```bash
source /opt/ros/lyrical/setup.bash
cd ros2_ws
CMAKE_BUILD_PARALLEL_LEVEL=1 colcon build --executor sequential \
  --build-base build/pr2-review --install-base install/pr2-review
source install/pr2-review/setup.bash
ROS_DOMAIN_ID=181 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST colcon test --executor sequential \
  --build-base build/pr2-review --install-base install/pr2-review \
  --packages-select hardware manipulation swerve_controller
colcon test-result --test-result-base build/pr2-review --verbose
```

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
| `fix/` | 버그 수정 | `fix/barcode-decode` |
| `docs/` | 문서 | `docs/jetson-setup` |
| `chore/` | 폴더 구조, 설정, 빌드 스크립트 | `chore/init-structure` |

### 커밋 메시지

형식: `type: what you did`

- 영어로 통일, 소문자 이용
- 동사 원형으로 시작 (`add`, `fix`, `update`, `remove`), 끝에 마침표 없음

| type | 용도 | 예시 |
|---|---|---|
| `feat` | 새 기능 | `feat: add swerve inverse kinematics` |
| `fix` | 버그 수정 | `fix: correct steering zero offset sign` |
| `docs` | 문서 | `docs: add jetson setup notes` |
| `chore` | 폴더 구조, 설정, 스크립트 | `chore: add mcap to gitignore` |
| `build` | 빌드 설정 (CMake, package.xml) | `build: add yaml-cpp dependency` |


### 작업 방법

```bash
# 1. main 최신화 후 브랜치 생성
git checkout main
git pull
git checkout -b feat/swerve-ik

# 2. 작업 후 커밋, push
git status
git add <파일>
git commit -m "feat: add swerve inverse kinematics"
git push -u origin feat/swerve-ik
```

3. GitHub에서 PR 생성 (대상: `main`)
4. 승인 1개 받은 뒤 관리자가 **Create a merge commit**으로 머지

### 커밋 금지

- 빌드 산출물, 데이터셋, 모델 가중치, rosbag
- 비밀번호, 토큰, SSH 키
