# meshes — 로봇 모양 파일 출처

> ⚠ 지금 메시 · 배치는 임시 모델. 하드웨어(라이다 · 팔 받침 · 층)가 확정되면 CAD 를 다시 받아 3절 순서로 다시 뽑는다.

1. 출처 표 · 아직 없는 부품  2. omx_6axis (손목)  3. swerve_cad (몸통 · 스워브)

## 1. 출처

| 폴더 | 출처 | 버전 | 라이선스 |
|---|---|---|---|
| `swerve_cad/` | 팀 Swerve CAD (Onshape → `Assembly_1.step`) 를 링크별로 나눠 변환 | — | 팀 자료. 안의 와플·다이나믹셀 부품은 ROBOTIS 도면 |
| `open_manipulator_x/` (`link1~5`, `gripper_*_palm`) | ROBOTIS-GIT/open_manipulator `open_manipulator_description/meshes/open_manipulator_x` (main) | 커밋 `d31000d` | Apache-2.0 (같은 폴더 LICENSE) |
| `omx_6axis/` | 위 `link5.stl` 을 부품별로 나눈 것 + `wrist_plate_v2.step` | — | 아래 2절 |
| `turtlebot3/lds.stl` | ROBOTIS-GIT/turtlebot3 `turtlebot3_description/meshes` (jazzy) | 커밋 `0c0be84` | Apache-2.0 (같은 폴더 LICENSE) |


### 아직 메시가 없는 부품 (상자로 대신 그림) — TODO

CAD(`Assembly_1.step`)에 없는 것들이라 위치도 가정이다.
- 팔 받침 위치 (`arm_mount_x/y` 인자, 기본 0,0 = 맨 위 판 가운데로 추정 — 미확정)
- 라이다 LDS 위치 (맨 위 판, 팔 뒤로 가정) — 메시는 터틀봇3 것. ⚠ 지금 가정 위치면 HOME 팔과 겹침 → 위치 다시 정해야 함
- 2020 프로파일 45 cm, D435, bird-eye 카메라

팀 CAD 에 이것들이 들어가면 STEP 을 다시 받아 아래 3절 순서로 다시 뽑는다.

## 2. omx_6axis — 6축 손목 메시

공식 OpenManipulator-X `link5.stl` 은 파란 프레임과 그리퍼가 한 덩어리라서, 손목 관절을 넣으면 화면에서 제대로 돌지 않는다.
그래서 부품별로 나눴다.

| 파일 | 어디에 붙나 | 내용 | 출처 |
|---|---|---|---|
| `frame.stl` | link5 (안 돎) | 파란 프레임 FR12-H104 | 공식 link5.stl 에서 분리 |
| `xm430.stl` | link5 (안 돎) → 손목 모터 / wrist_link (돎, −20 mm) → 그리퍼 모터 | XM430 한 개. 같은 모터라 파일 하나를 두 번 씀 | 공식 link5.stl 에서 분리 |
| `gripper_frame.stl` | wrist_link (돎, −20 mm) | 그리퍼 레일 브래킷 · 봉 · 나사 | 공식 link5.stl 에서 분리 |
| `wrist_plate_v2.stl` | wrist_link (돎) | 손목 판 v2 | `wrist_plate_v2.step` 그대로 (부피 12,488 mm³) |

좌표: 앞의 세 파일은 **link5 좌표 그대로** (4번 관절 축 = 원점, 단위 mm, 팔 방향 = +x).
판은 wrist_link 좌표 (손목 모터 혼 면 = 원점). STEP 원본은 턱이 아래(−z)로 그려져 있어 x 축으로 180° 돌려 넣었다
→ 턱 윗면이 z = 40.85 mm = 레일 브래킷 윗면 37.8 + 턱 3 mm 로 딱 맞음.

숫자 근거 (메시에서 잰 값이 손목 설계값과 맞는지 확인함)
- 원래 그리퍼 모터: 뒷면 x = 28 mm, 혼 면 x ≈ 64 mm → 설계값 "손목 관절 = 4번 관절에서 +0.064 m" 와 같음
- 레일 브래킷 윗면 z = 37.8 mm = 모터 윗면 35.2 + 2.6 → 설계값 "턱이 얹히는 면, 모터 윗면에서 2.6 mm 위" 와 같음
- 그리퍼 쪽 전체를 +44 mm 옮김 → wrist_link 기준 44 − 64 = **−20 mm** (`robot.urdf.xacro` 의 `grip_shift`)

참고: `Assembly_2.step`(6축 팔 전체)의 `Part 1` 은 부피 8,421 mm³ 로 판 v2 와 다름 → 예전 판(v1)으로 보고 쓰지 않음.

할 일
- 무게 · 무게중심(inertial): 손목 모터 82 g, 판 15.5 g. 지금 URDF 의 관성은 상자 모양 추정값 → 실측 · CAD 값으로 바꾸기

## 3. swerve_cad — 팀 Swerve CAD 를 링크별로

원본: `Assembly_1.step` (Onshape, 루트 `2026-10-02-10-23-01-023`, 부품 974개)
— 와플파이 3층(`Chassis_WafflePi`) + 스워브 모듈 4개 (FL·FR·RL·RR 마다 조향 모터 · 나사 · FR12-S102K 브래킷 · 주행 모터 · 바퀴). 팔은 없음.

### 어느 부품이 어느 링크로

| 파일 | 링크 | 원본 부품 | 손을 돌리면 |
|---|---|---|---|
| `chassis.stl` | base_link | `Chassis_WafflePi` 전체 (판 3층, 지지대, OpenCR …) | 고정 |
| `steer_motors.stl` | base_link | `*_Steer_XM430_W350` ×4 + `*_SteerMount_Screws` | 고정 (몸통에 붙은 조향 모터 몸체) |
| `module_xx.stl` | steer_xx_link | `xx_Frame_FR12_S102K` + `xx_Drive_XM430_W210` | 조향하면 같이 돎 |
| `wheel_xx.stl` | wheel_xx_link | `xx_Wheel` | 바퀴 굴림 |

### 좌표 (CAD → ROS)

- CAD: x 앞, **y 위**, z 오른쪽 (mm) → ROS: x 앞, y 왼쪽, z 위
- base_link 원점 = **조향축 4개의 한가운데, 바퀴축 높이** (CAD 점 x = −126.8, y = 8.0, z = 29.7)
- module 메시는 자기 조향축이 원점, wheel 메시는 바퀴 중심이 원점

### CAD 에서 잰 값 (URDF · controllers.yaml 에 들어감)

| 항목 | 값 | 어떻게 잼 |
|---|---|---|
| 조향축 위치 | x ±0.112, y ±0.064 m | 조향 모터 혼의 원통면 축 |
| 바퀴 반지름 | 0.033 m (폭 20 mm) | 바퀴 가장 바깥 원통면 |
| 바퀴 중심 ↔ 조향축 | 바퀴축 방향으로 **바깥쪽 26.9 mm** | 바퀴 중심 − 조향축 |
| 바닥 → 1층 판 아랫면 | 72 mm | |
| 바닥 → 맨 위 판 윗면 | 164 mm | |
| 몸통 앞 · 뒤 끝 | +0.141 / −0.133 m | |

### 가볍게 만들기

원본 삼각형 약 93만 개(46 MB) → 몸통 8만, 모듈·바퀴 6천 개로 줄임 (`fast_simplification`, 모양 차이 최대 0.6 mm).
마이크로 USB 소켓 2개는 변환 도구가 멈춰서 뺐다 (OpenCR · USB2LDS 보드 위, 크기 5 mm).

### 다시 뽑을 때

CAD 가 바뀌면 STEP 을 다시 받아 같은 순서로: 부품 하나씩 STL → 위 표대로 묶기 → 좌표 바꾸기 → 줄이기.
스크립트: `tools/cad_to_urdf.py` (쓰는 법은 그 파일 맨 위)
