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
├── config/                  # 노드 파라미터 yaml (perception, swerve_controller 등)
├── firmware/                # OpenCR 펌웨어
├── ros2_ws/                 # ROS 2 워크스페이스
│   └── src/
│       ├── bringup/             # 전체 실행 launch, 실기/시뮬 전환
│       ├── description/         # URDF/xacro, 메쉬 (RViz·Isaac Sim 공용)
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
├── tools/                   # ROS 밖에서 단독 실행하는 개발 도구
│   └── camera_calib/        # 카메라 캘리브레이션 사진 수집 (OpenCV)
└── docs/                    # 설치 기록, 설계 문서, 가이드 (코드 X, 문서·이미지만)
```

## 빌드

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
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
4. 승인 1개 받은 뒤 관리자가 **Create a merge commit**으로 머지

### 커밋 금지

- 빌드 산출물, 데이터셋, 모델 가중치, rosbag
- 비밀번호, 토큰, SSH 키
