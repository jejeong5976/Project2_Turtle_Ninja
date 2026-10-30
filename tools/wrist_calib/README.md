# tools/wrist_calib — 손목 D435 장착 자세 캘리브레이션 (핸드-아이)

손목 링크(`arm_link5_wrist_roll`)에 D435 가 어떤 위치 · 각도로 붙어 있는지 잼. 결과는 URDF 의 `d435_cad_front_fixed` origin 에 넣음 → TF `base_link ← d435_color_optical_frame` 이 맞아야 손목 카메라로 찾은 퍽 위치를 로봇 기준으로 바꿀 수 있음.

- 브래킷이 경첩을 조여 고정하는 방식이라 각도는 설계값이 없음. 지금 URDF 값(경첩 −12°)은 사진으로 추정한 값
- 관찰 거리 0.29 m 에서 각도 1° 틀리면 퍽 위치 약 5 mm 틀림
- 실물 로봇에서 한 번, 브래킷을 다시 조이면 또 한 번

## 파일

```
tools/wrist_calib/
├── handeye.py          # capture (촬영 + 관절값) · solve (계산) · selftest (카메라 없이 확인)
├── make_gt_sheet.py    # ArUco 정답 종이 만들기 → gt_sheet.pdf (인쇄용) · gt_sheet.json
├── gt_sheet.json       # 종이 위 마커 위치 (handeye.py 가 읽음)
└── datasets/           # 촬영 저장, 커밋 안 함
```

- `gt_sheet.pdf` 는 커밋 안 함 → `make_gt_sheet.py` 로 만듦 (같은 종이가 나옴)
- 실행 환경: Jetson (또는 D435 · 팔 OpenCR 이 같이 연결된 PC). 패키지는 레포 최상위 `requirements.txt`, 화면이 있어야 함 (모니터 또는 `ssh -X`). 화면 창은 `tools/d435_viewer/d435_puck_viewer.py` 의 `Viewer` 를 씀, 순기구학은 `sim/robot_sim/sim_core/arm_poses.py` + 배포판 URDF

## 순서

```bash
cd tools/wrist_calib
PY=python3                     # requirements.txt 설치한 venv 의 python

# 0) 정답 종이: PDF 를 100 % 로 인쇄 → 종이의 100 mm 눈금으로 축척 확인 → 로봇 앞 바닥에 테이프로 고정
$PY make_gt_sheet.py

# 1) 촬영: 차체는 움직이지 않고 팔만 10–20 자세 (J1 좌우 · J4 숙임 · J5 비틀기를 크게 다양하게), 자세마다 멈추고 s 키
$PY handeye.py capture --joints ticks

# 2) 계산 → 손목 기준 카메라 자세, URDF 와 차이, URDF 에 넣을 origin, 자세별 재투영 오차
$PY handeye.py solve datasets/<세션>
```

## 관절값 넣는 법 (`--joints`)

| 방법 | 넣는 법 |
|---|---|
| `ticks` (기본) | s 키 → 터미널에 팔 모터 Present Position 틱 5 개 (ID 11 → 12 → 13 → 14 → 16, 예: `4088 2154 1624 1897 2018`). 팔 OpenCR 쪽 모터 값 (Dynamixel Wizard · control 로그). `sim/robot_sim/robot_description/config/joint_map.json` 으로 URDF 각도 변환 |
| `ros` | `/joint_states` 자동 구독 (`joint1..5`). control 관절 0 점 · 부호가 URDF 와 다르면 `--ros_offsets '0점:부호,…'` |
| `urdf` | URDF 관절각 5 개 (rad) 직접 입력 |

- D435 와 팔 OpenCR 이 같은 컴퓨터에 연결돼 있어야 함
- 자세마다 팔이 멈춘 상태에서 저장 (영상과 관절값이 같은 순간)
