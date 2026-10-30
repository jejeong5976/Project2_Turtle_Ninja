# 닌자터틀 배포 URDF — 배치 수정판 (2026-10-27, `nt_urdf_release_20261027_layout`)

**이 판 = `nt_urdf_release_20261027_swerve`(URDF 6e1a…) + 높이·보드 배치 수정 + Isaac 시작 자세(parked, 그리퍼 16 mm).** 이전 배포본(…_20261026, …_swerve)은 정리하며 지움 — 이 판이 최신·유일.
URDF sha256 `026e4929…` (2026-10-27 저녁: 라이다 스캔 방향 앞 180° · 뒤 0° — 아래 '라이다 방향'). 관절 이름·영점·한계·질량·관성·스워브 값은 바뀌지 않음.

**팀원 실행 준비:** zip 을 아무 곳에 풀면 됨(경로는 모두 이 폴더 기준 상대 경로, 무결성 확인은 폴더 안에서 `sha256sum -c MANIFEST_SHA256.txt`).
- Isaac Lab 3.0 / Isaac Sim 6.2 의 python 으로, `OMNI_KIT_ACCEPT_EULA=YES` 붙여 실행. 학습 묶음은 `config/nt_train_kit.py` 하나(아래 '학습에 쓰는 법'), 먼저 `tools/check_train_kit.py`(화면 없이, 1~2 분)로 통과 확인.
- 미로가 필요한 도구(`demo_keyboard.py`, `record_train_kit.py`)는 `NT_TEAM_SIM=<Project2_Turtle_Ninja/sim/robot_sim>` 지정(없으면 데모는 빈 바닥, 촬영은 안내 후 종료). `trimesh` 는 없어도 됨.
- RViz 보기는 `./view.sh`(ROS lyrical). `_ament/share/nt_release_description` 은 이 폴더를 가리키는 링크 — zip 을 Linux 에서 풀 것.
- `tools/` 의 make_*·build_*·render_* 등은 원본 기록 폴더가 있어야 다시 만드는 제작 기록용(실행 안 해도 됨).

### 바뀐 것 (값·근거 `config/layout_20261027.yaml`, 도구 `tools/apply_layout.py`)
| 항목 | 이전 | 이 판 | 근거 |
|---|---|---|---|
| 라이다판 밑면(바닥 기준) | 11.6 mm | **20.0 mm** (판·앞뒤 라이다 +8.4 mm) | 실측 20 mm. 판 위 기둥 끝(−43 mm)과도 일치 |
| 라이다 스캔면 `base_scan_front/rear` | 48.6 mm (라이다 +28) | **60.0 mm** (라이다 +31) | 실측 60 mm. 뒤 라이다는 같은 판에 바로 고정(사진)이라 같은 값 [추정] |
| 배터리판 윗면 / 밑면 | 71 / 62 mm | **80** / 71 mm (+9 mm, 배터리 +9 mm) | 실측 윗면 80·밑면 75. 모델 판 두께 9 mm(실물 5)라 밑면 4 mm 차 |
| OpenCR판·팔판 | 윗면 118.5 / 163.5 | 그대로 | 실측 윗면 120 / 165 (차 1.5 mm), 밑면 110 / 159 |
| OpenCR② (주행) | 오른쪽, 판을 4.2 mm 파고듦 | **왼쪽 앞** PCB 중심 (+1, +75) mm | 사용자 확정 배치 + 사진(±10 mm) |
| OpenCR① (팔, 추정) | 가운데 | **왼쪽 뒤** (−92, +35) mm | 사진(±10 mm) |
| `imu_link` | ① 위치, yaw 0 | **② 중심 (1, 75, 55.8) mm, yaw +90°** | OpenCR REV.H BOM·XY(MPU-9250 PCB 중앙), 실크 축 |
| Jetson (충돌 상자 포함) | 오른쪽 뒤, yaw 0 | **오른쪽** 캐리어 중심 (+13, −58) mm, yaw −90° (I/O −y) | 사진(±10 mm) |
| 라이다↔Jetson 연결 보드 | 없음 | 표시용 상자만 (0.110, 0.049) m, 질량·충돌 없음 | 사진 |

사진으로 확정 못 한 값: 보드별 스탠드오프 높이(두 OpenCR PCB 윗면은 55.28 mm로 둠), 보드별 미세 yaw, ±10 mm보다 정확한 xy. IMU 칩이 ICM-20648 이면 축이 다를 수 있음.
`FL0_LiDAR_Spec.md`(CAD 사양서, 원점=FL0 판 윗면, 스캔 +26.5, yaw 180/0 잠정)는 기준이 달라 적용하지 않음. 라이다 x·y 그대로. yaw 는 아래 '라이다 방향' 참고(앞 180°, 뒤 0°).

### 라이다 방향 (2026-10-27 저녁)
- 실물: 앞·뒤 라이다 모두 구멍이 안 맞아 **앞뒤로 180° 돌려 장착**(위아래는 그대로, 사용자 확인). 이 상태로 Jetson 매핑(NJWQJ7, `/lidar_front/scan_mapping`, base_footprint→lidar_front_link yaw π)에서 지도가 정상으로 나옴(벽 사이 1.45 × 3.20 m, 미로 칸 크기와 5 cm 안).
- URDF: `lidar_front_mount` yaw 90° → **180°**, `lidar_rear_mount` 270° → **0°** (앞뒤가 같은 판에 서로 180° 반대인 관계 유지, `base_scan_*` 도 같이). 라이다 겉모양·충돌 상자는 차체 기준 그대로(링크 안에서 −90° 보정, 차이 1e-8 m) → USD 는 바뀔 것 없음(라이다 프레임이 USD 에 없음).
- 그대로 둔 것: 뒤 라이다는 매핑 기록이 없어 지도로는 미확인(앞과 같은 장착이라는 사용자 확인 기준). 앞 라이다 x 0.1145 · y −0.006 m · 스캔 높이 60 mm(실측). 매핑 설정은 x 0.110 · y +0.005 · 높이 54.5 mm(추정) — y 부호 차이는 중심에서 벗어난 광학 중심을 180° 돌린 결과일 수 있음 [추정, 미실측].
- 주의: 이 180° 는 매핑에 쓴 `/lidar_front/scan_mapping` 드라이버 기준. 다른 드라이버(예: 팀원 run_scan.sh 의 `/scan`, 기본 π/2)는 0° 기준이 다를 수 있어 따로 확인.

### Isaac 시작 자세 (주행 기본자세 parked, 그리퍼 16 mm)
- `tools/apply_initial_pose.py`: USD 에 팔 5 관절 + **손가락 a·b 둘 다**의 초기 위치(JointStateAPI)와 구동 목표를 parked 로 적음, 루트 `Physics` 변형 기본 = `physx`. 관절은 고정하지 않음(팔·그리퍼 계속 제어 가능).
- 손가락 값 −0.0212536 m = (16 − 58.507 mm)/2000. 관절 0 이면 58.5 mm 로 열린 상태.
- **Isaac Lab 은 USD 초기값을 읽지 않음** → `config/isaac_lab_init_state.py` 의 `parked_init_state()` 를 `ArticulationCfg.init_state` 로 쓰고, 리셋 때 팔·손가락 위치 목표도 같이 넣을 것.
- 손가락 충돌 CoACD 교체 + 손가락끼리 충돌 제외 + 받침을 그리퍼 중심에 맞춤 → 시뮬 그리퍼 **16.0 mm** 유지, 팔 토크 OFF 에서도 받침에 얹혀 안정, 꺼냈다 넣기 통과(CHANGES 참고). P0 시작 권장 문구는 이 판에서 철회.

### 다시 만들기 (순서 중요)
```bash
python3 tools/build_release.py          # 준비본 URDF e8bd… (README 옛 절차는 여기서 멈춰 스워브 한계가 빠졌음)
python3 tools/apply_swerve.py           # → 6e1a… 스워브 조향 한계
/home/pa25/IsaacLab-course/.venv/bin/python tools/make_finger_collision.py   # 손가락 충돌 조각(CoACD) — 이미 있으면 생략 가능
python3 tools/apply_layout.py           # → 02d5… 높이·보드 배치 + 배터리판 이동 메시 + 손가락 충돌 교체 + 그리퍼 범위 + 받침 충돌
/home/pa25/env_isaacsim62/bin/python tools/isaac_import.py
/home/pa25/env_isaacsim62/bin/python tools/apply_initial_pose.py
/home/pa25/env_isaacsim62/bin/python tools/apply_collision_filters.py   # 손가락 a↔b 충돌 제외
```
`isaac_render.py` 를 돌리면 usd/ 가 차체 고정본으로 덮어써지므로 마지막 두 줄을 다시 실행. 주행 학습용 팔 고정 USD(`swerve_rl_20261027/prepare_asset.py`)는 6e1a… 를 입력으로 고정하고 있어 이 판으로 갱신하려면 그 스크립트 해시도 바꿔야 함.


### 학습에 쓰는 법 — `config/nt_train_kit.py` (Isaac Lab 3.0)
```python
import sys; sys.path.insert(0, "<이 폴더>")
from config import nt_train_kit as kit
robot = kit.robot_cfg("{ENV_REGEX_NS}/Robot")      # USD + 검증 액추에이터 + parked·그리퍼 16 mm 시작 + 자기 충돌 켬(솔버 32/4)
# scene 만든 뒤 sim.reset() 전에 한 번:
kit.apply_collision_filters(stage)                 # 모든 env: 차체 (c) 필터 + 손가락 a↔b 충돌 제외
drive, nt = kit.swerve()                           # env 당 하나: 몸 속도 → 조향 각·바퀴 속도 (최고 0.26 m/s · 1.675 rad/s)
rp = kit.Replay()                                  # 실물 성공 실행 팔 궤적(기본 A9F5 최단 022813Z, 14개 중 선택)
a, b = rp.segment("out")                           # "out" 받침→P0, "back" P0→받침, "full" 미션 전체 (1배 = 실물 속도)
q_arm, q_finger = rp.at(t)                         # 시각 t 의 팔 5 관절·손가락 목표
q_arm, q_finger = rp.pose("grasp")                 # "parked" / "p0" 대기자세 / "grasp" 퍽 집기 자세
crank = kit.CrankVisuals(stage)                    # (카메라·영상 쓸 때) sim.reset() 전에. 렌더 전마다 crank.update(env별 손가락 q) → 크랭크·노란 막대가 손가락 따라 움직임(보이는 모양만)
```
**리셋 때:** `write_joint_position_to_sim(default_joint_pos)` 와 함께 팔·손가락 **위치 목표도 default 로** 넣을 것(안 넣으면 PD 가 0 으로 끌고 감).
**받침 안에서는 실측 경로(out/back)만 쓰기 권장:** 받침 안에서 다른 자세로 직선 보간하면 받침 벽에 걸렸다 말았다 함(시연에서 확인).

팀 sim(`robot_sim` TURTLENINJA_CFG) 설정으로 이 로봇을 학습하면 안 되는 점(팀 코드는 바꾸지 않음, 쓸 때 이 묶음 값으로):
| 항목 | 팀 sim 현재 | 이 묶음 | 이유 |
|---|---|---|---|
| 자기 충돌 | 끔 | **켬** | 받침이 base_link 에 합쳐져 있어 끄면 손가락이 받침을 통과 → 꺼내기·넣기 학습 불가 |
| 그리퍼 | 시작 0(58.5 mm 열림), 1 N · 강성 200 | 16 mm 시작, 두 손가락 1000/2 · 20 N · 0.15 m/s | 16 mm 유지·실물처럼 빠른 개폐 |
| 시작 자세 | P0 | parked(받침) — P0 는 `rp.pose("p0")` | 주행 기본자세 = 받침, 그리퍼 받침 접촉 안정 확인 |
| 충돌 필터 | 없음 | (c) 차체 + 손가락 a↔b | 볼록 덮개의 가짜 바퀴 접촉(90° 전환 6 s 대기 원인), 손가락끼리 겹침 |
| USD | URDF 를 다시 변환 | 이 배포본 `usd/` 그대로 | 변환본에는 손가락 충돌 CoACD·초기값·필터가 없음 |

**확인(`tools/check_train_kit.py`, GPU, env 8개, replicate_physics=False):** 8개 모두 parked 그리퍼 16.0 mm · 팔 0.18° · base 72 mm, 꺼내기 끝 0.11°, 복귀 끝 0.60~0.62° · 16 mm, 스워브 직진 0.260 m/s, 값 발산 없음 → `config/check_train_kit.json` pass.
replicate_physics=True(복제 가속)에서 필터가 전부 복제되는지는 아직 확인 안 함 — 처음엔 False 로.

### 키보드 시연 (Isaac Sim GUI) — `tools/demo_keyboard.py`
```bash
cd <이 폴더>
NT_TEAM_SIM=<Project2_Turtle_Ninja/sim/robot_sim> OMNI_KIT_ACCEPT_EULA=YES <Isaac Lab 3.0 python> tools/demo_keyboard.py
```
3D 화면을 한 번 클릭한 뒤 사용. **한글 입력이면 키가 안 먹음 → 영문으로.**

| 키 | 동작 |
|---|---|
| ↑ ↓ / ← → / Z X / L | 앞뒤 / 옆으로 / 제자리 회전 / 정지 — 스워브 `swerve/nt_candidate.py`(팀 swerve.cpp + 목표각 일관성 + 진입 여유 5°), 최고 0.26 m/s · 1.675 rad/s(실물 하드웨어 끝, 시뮬 실측 0.260 m/s) |
| 2 / 1 / M | 받침→P0 꺼내기 / P0→받침 복귀 / 미션 전체 — **실물 성공 실행의 측정 궤적**. 받침 안에서는 항상 실측 경로만 씀(받침에 있는데 1 → 꺼내기 먼저, 밖에서 2·M → 복귀 먼저) |
| P / O | 대기자세(P0) / 퍽 집기 자세(그 실행에서 그리퍼가 닫히기 시작한 순간, 그리퍼 ≈50 mm) |
| G | 그리퍼 16 mm ↔ 79 mm |
| 9 / 0 | 궤적으로 쓸 성공 실행 고르기(14개: A9 5 · A9F~F5 7 · A10 2, 기본 A9F5 최단 022813Z) |
| [ / ] | 재생 배속 0.5~4배(기본 2배, 1배 = 실물 속도) |
| 3~7 + W/S | 팔 자유 조작: J1~J5 고르고 W(+)/S(−) 누르는 동안 0.8 rad/s, 관절 범위 안 |
| C | 카메라 창 끄기/켜기(끄면 화면이 부드러워짐) |
| R (Backspace·Home) | 처음 자리로(재생도 멈춤) |

- 환경: 팀 sim 의 실제 경기장 미로 `maze_real.usd` + 카펫 + 강의실 파노라마 + 천장 조명 6개 + 퍽 3개(팀 sim 파일은 읽기만). 미로가 없으면 빈 바닥(`NT_WORLD=empty` 와 같음).
- 카메라 창 2개(실시간): **birdeye** = 팀 실측 K(1280×720) + OpenCV 어안 D(팀 sim 과 같은 값), **wrist_d435** = D435 공장 K(640×480).
- 그리퍼 크랭크(회색)·링크 막대(노랑)가 손가락과 같이 움직임(보이는 모양만, 물리 무관). 손가락 위치 → 실측 틱표(1045 열림 · 2270 29 mm · 2658 16 mm) → 모터 각도. 막대 끝 어긋남 79 mm 0.7 / 29 mm 1.7 / 16 mm 5.7 mm(CAD 막대 52 mm, 실측 55 mm).
- 물리 CPU 200 Hz, 화면은 따라잡을 때만 그림 → 카메라 창 켜면 약 12 fps 지만 움직임은 실제 시간(로그 1.00배).
- 받침대 모델은 실물과 1~2 cm 차이(원인 미확정, 이전 기록). 받침 근처를 W/S 로 직접 움직이면 걸릴 수 있음.

### 최종 점검 (2026-10-27)
| 항목 | 결과 |
|---|---|
| ROS `check_urdf` | 통과 (링크 42 · 관절 41, 루트 base_link) |
| 구조·값 검사(이름 중복, 부모 2개, 메시 700개 존재, 관절 한계·힘·속도, 축, 관성 양의 정부호·삼각 부등식, 저장 자세 7개 범위) | 문제 0 · 총질량 3.81 kg |
| parked 유지(`verify_isaaclab.py`) | 그리퍼 16.0 mm · 팔 오차 0.18° · base 72 mm · 스캔면 60 mm |
| 꺼냈다 넣기(`verify_insert.py`) | 꺼내기 0.43° · 넣기 2.0° · 끝 그리퍼 16.9 mm · 발산 없음 |
| 실측 복귀 배속(`verify_return_speed.py`, 022813Z) | 1 · 1.5 · 2배 모두 끝 자세 오차 0.6°, 그리퍼 16 mm (복귀 중 J2 최대 4.9 / 7.2 / 6.8°) |
| 키보드 시연 시작(미로·카메라·크랭크·궤적) | 오류 없음 |
참고: URDF 안 로봇 이름은 `nt_ninja_turtle_release_20261026` 그대로. `body.obj` 두 개(프로파일 받침·퍽 홀더)가 파일 이름이 같아 MuJoCo 로 읽을 때만 고유 이름 필요.

### 시연·검증 파일
- `tools/demo_keyboard.py` 키보드 시연 · `swerve/nt_candidate.py` 시연용 스워브 계산(`collision_c_sweep_20261027` 검증본 복사)
- `tools/make_arm_replay.py`, `tools/make_all_replays.sh` → `config/arm_replays/<실행>.json` (실물 03_session.jsonl Present 틱 → 관절값, 원본 실행 폴더가 있을 때만 다시 만들 수 있음). `config/arm_replay_A9F5.json` = 022813Z, `_023249Z.json` = 이전 판
- `tools/make_crank_linkage.py` → `config/crank_linkage.json`, `meshes/generated/crank_anim_*.obj`
- `tools/verify_return_speed.py` → `config/verify_return_speed.json`
- `tools/check_train_kit.py` → `config/check_train_kit.json` (학습 묶음 GPU env 8개 확인)
- `tools/record_train_kit.py` → 학습 묶음 동작 영상(팀 미로 안 env 8개, 스크립트 명령 재생 — 정책 학습 아님: 미션 재생·그리퍼 개폐·직진/회전·옆으로·대각·S자·무작위 몸 속도·무작위 팔 목표, 벽 막기 포함, 크랭크 표시). 영상·이미지는 이 묶음과 따로 배포

---

# 닌자터틀 배포 URDF + 스워브 주행 (2026-10-27)

**이 판 = 2026-10-26 배포본 + J1 축 위치 수정 + 고정 예시 퍽 제거(Codex 준비본 `e8bd…`) + 바퀴 관절 한계 최대값 + 스워브 계산(Python).**
URDF 에서 바뀐 것은 바퀴 관절 8개의 `<limit>` 뿐입니다(`tools/apply_swerve.py`). 팔·센서·홀더·받침·메시는 준비본 그대로.

## 학습에서 바퀴 움직이는 법 (스워브)

`swerve/nt_swerve.py` — 팀 실물 컨트롤러(`nt_control/swerve.cpp`, 2026-10-27 로컬 판)를 줄 단위로 옮긴 것. **C++ 원본과 같은 입력에서 같은 출력**(아래 검사).

```python
import sys; sys.path.insert(0, "<이 폴더>/swerve")
from nt_swerve import from_config, ModuleState, SwerveDrive
kin, cfg, J = from_config("<이 폴더>/config/swerve.yaml")
drive = SwerveDrive(kin, cfg)
# 매 제어 주기(권장 100 Hz):
drive.set_command(vx, vy, wz, t)       # 정책 출력 = 몸 속도 [m/s, m/s, rad/s]
meas = [ModuleState(steer_q[i], wheel_qd[i], steer_qd[i]) for i in range(4)]   # J["steer"], J["drive"] 순서
out = drive.update(t, dt, meas)        # out[i].steer_angle → 조향 관절 위치 목표, out[i].wheel_speed → 바퀴 관절 속도 목표
```

- 정책은 '몸 속도'만 내면 됩니다. 조향 각도 선택(같은 선 위 정렬·바퀴 바깥쪽·범위 끝 처리)·조향 먼저/동시 조향·바퀴 최고 속도 정규화는 실물과 똑같이 이 계산이 합니다.
- 상태 없는 변환만 필요하면 `kin.inverse(Twist(vx, vy, wz), meas, ...)` (반환: 모듈 명령, 가능 여부).

## 넣은 값 (`config/swerve.yaml`, 근거 표시)

| 항목 | 값 | 근거 |
|---|---|---|
| 바퀴 최고 | 7.914 rad/s → 몸 0.261 m/s, 제자리 회전 1.675 rad/s | 모터 Velocity Limit 330 (10-26 레지스터 읽기) |
| 조향 최고 | 4.796 rad/s (≈275°/s) | 모터 Velocity Limit 200 |
| 힘 | 바퀴 3.0 N·m · 조향 4.1 N·m | XM430 데이터시트 12 V |
| 조향 범위 | 엔코더 창 전체(끝 여유 0) + 끝 1° 여유 처리 | 선 때문에 360° 넘게 못 돎(사용자 확인) |
| 학습 가속 | 1.0 m/s² · 6.0 rad/s² | **추정**(바퀴 힘·접지는 훨씬 여유). 실물은 0.2 m/s²까지만 확인 |
| 시험용 제한 | 없음 (0.05 m/s 도구 상한·20틱 여유·명령 제한기 모두 뺌) | 학습이 막히지 않게 |

실물에서 확인한 것은 0.13 m/s(50%)·0.2 m/s²·조향 140°/s 까지입니다(10-26). 그 위는 시뮬 결과이며 실물 확인 전입니다.

## 이번 배포본에서 실제로 한 검사

| 검사 | 결과 | 파일 |
|---|---|---|
| Python ↔ C++ 원본 일치 (역기구학 6,000건 + 주행 순서 7종 4,290 주기) | 불일치 0 (차이 ≤ 5e-11) | `config/swerve_port_check.json`, `tools/check_swerve_port.py` |
| MuJoCo 바닥 주행(최고 속도, 100 Hz, 바퀴 마찰 1.0) | 전·후·옆·대각 속도 99.5 %, 회전 99.5 %, S자 전환 0.91~0.93 | `config/sim_drive_check.json`, `tools/sim_drive_check.py` |
| URDF 문법·colcon 설치·메시 경로·launch(받침/P0) | 통과 | (아래 '실행') |
| 실제 실행 기록 31만 표본의 팔 관절 범위 | 위반 0 | 10-23~26 기록 |
| 실제 실행 기록 자세 5만 7천 개 팔↔몸체 충돌(MuJoCo) | 퍽·상판·기둥 겹침 없어짐. **손가락↔받침 겹침 남음** | Claude 검사(10-27) |

**하지 않은 것:** 실물 0.26 m/s 주행, Isaac 물리 주행(USD 는 만들고 내용만 검사), 파지·학습, Isaac 보기 이미지 재생성.

## 그림·영상 (`images/`, MuJoCo 렌더 — `tools/render_media.py`)
- `release_p0.png`, `release_parked.png`: 준비 자세 P0 / 받침(주행) 자세
- `release_views.png`: 받침 자세 6방향(정면·후면·왼쪽·오른쪽·위·비스듬히), MuJoCo 렌더 (`tools/render_media.py views`)
- `drive_swerve_sim.mp4`: 스워브 계산으로 MuJoCo 바닥을 실제로 굴린 주행(최고 속도 S자 → 정면 유지 전방향 → 제자리 회전 → 회전 후 전진). 시뮬이며 실물 아님
- `arm_replay_A9F5.mp4`: 실물 실행 기록(ARM_20261026T023249Z, 퍽 잡아 적재함, 27.3 s)의 관절 값을 이 모델에 그대로 넣은 재생(물리 아님)
- MuJoCo 주의 ①: URDF 의 손가락 mimic(손가락 b = a)을 MuJoCo 가 읽지 않음 → 이 폴더 도구는 equality 로 직접 연결. Isaac 도 mimic 설정(PhysX mimic joint) 확인 필요.
- 주행 시뮬의 팔: 실물처럼 **팔·그리퍼 토크 OFF**(액추에이터 없음, 감속기 마찰·감쇠만 [추정]). 받침에 끼운 손목·손가락은 **받침에 용접**(받침에 끼면 못 움직이는 것을 그대로 표현).
  접촉 대신 용접을 쓰는 이유: 받침 자세에서 손가락 a 가 받침 벽에 1.2 mm 걸침(실제 메시 기준, 받침 관통 수정 때 확인한 1~3 mm 안). MuJoCo 는 손가락 충돌 메시를 볼록 껍질로 계산해 이것이 최대 3.9 mm 로 커지고, 시작하자마자 밀어내 팔이 접힘 → **MuJoCo 계산 방식 문제이고 모델 문제 아님**.
  결과: 모든 주행 시나리오에서 팔 관절 변화 ≤ 0.53°, 손가락 ≤ 0.03 mm (`config/sim_drive_check.json` 의 `arm_max_dev`).
  덧: MuJoCo 에서 컴파일 뒤 geom 충돌 비트를 바꾸면 몸체 비트(`body_contype/conaffinity`)도 같이 바꿔야 접촉이 생김(안 바꾸면 조용히 무시됨, 확인함).
- MuJoCo 주의 ②: URDF 를 MuJoCo 로 읽으면 같은 파일 이름 메시(`profile_holder/body.obj`, `puck_holder/body.obj`)가 하나로 합쳐져 프로파일 받침이 퍽 홀더 모양으로 보임. 이 폴더 도구들은 경로별 고유 이름으로 읽게 고쳐 둠(URDF 는 정상, RViz·Isaac 은 영향 없음).

## 알아 둘 것 (시뮬 결과)
- 정면 유지 전방향 주행처럼 방향을 한쪽으로 계속 돌리면, 조향 범위 끝 때문에 바퀴 하나가 반대로 크게 돌아야 하는 구간이 생깁니다(실물과 같음). 그 동안 0.5~1 s 구동 대기.
- 회전+전진을 동시에 최대로 주면 바퀴 최고 속도를 넘어서 네 바퀴가 같은 비율로 줄어듭니다(곡선 시험 64 %).
- 시뮬 관절 구동: 조향 = 위치 서보(목표를 4.796 rad/s 로 따라가게), 바퀴 = 속도 서보, 감속기 관성(armature) 0.005~0.012 kg·m²(추정) — 이것 없이 작은 바퀴 관성에 큰 서보 이득을 주면 MuJoCo 가 발산합니다(확인함).
- 받침 자세에서 손가락이 받침 앞·옆벽과 1~3 mm 급 겹침. (이전 "시뮬 시작은 P0 권장"은 철회 — 주행 기본자세는 parked, 위 "Isaac 시작 자세" 참고)

---

# 닌자터틀 배포용 URDF (2026-10-26)

`urdf/robot.urdf` — 팀 학습·시뮬용. **Codex 최신 모델(2026-10-25, 퍽 홀더 6칸 재배치 + 손가락 충돌 정밀화, sha256 7d482f…)** 에 오늘 실측값만 넣은 별도 판입니다. 원본 파일은 그대로 두었고, 이번 준비본은 J1 하우징 방향과 arm_mount_fixed X를 CAD·사용자 확인 40mm 면간 간격으로 수정합니다. 팔 전체는 기존 release 대비 +26.800350mm 전진하며 관절 영점/틱 지도는 보존합니다. release 적용은 승인 검토에서 차단된 상태입니다.

원본: `task-2/audit_work/holders6_20261024/inside_plate_proposal/side_columns_adjustment/full_review_h4_followup/candidate/simulation/robot_sim_candidate.urdf`

## 바뀐 것 (근거는 `config/release_changes.json`)

| 항목 | 원본 | 배포판 | 근거 |
|---|---|---|---|
| arm_joint1 범위 | −180° ~ 180° (격리 가정) | **−290.03° ~ 149.87°** (받침 기준 오른쪽 245° ~ 왼쪽 195°) | 팔(J3)을 든 자세 손 측정 중 가장 넓은 범위 + 실행 기록(사용자 확인). 팔을 접은 낮은 자세에서는 몸체에 닿아 좁음 → 충돌 형상이 막음 |
| arm_joint2 | −99.3° ~ 80.7° | **−126.93° ~ 88.68°** | 손 측정(프로파일 장착) + 실행 |
| arm_joint3 | −126.1° ~ 53.9° | **−125.04° ~ 82.04°** | 같음 |
| arm_joint4 | −163.7° ~ 16.3° | **−163.97° ~ 51.80°** | 같음, 위치 모드 엔코더 창 32~4063 |
| arm_joint5 | −90° ~ 90° | **−179.70° ~ 113.16°** | 같음, 같은 엔코더 창 |
| 팔 effort / velocity | 1.0 N·m / 1.0 rad/s (자리값) | **4.1 N·m / 4.817 rad/s** | XM430-W350 데이터시트 12 V. 실물 운용 값은 `config/operating_limits.yaml` |
| 그리퍼 열림 | 71.5 mm | **79 mm** | 실측(고무패드 안쪽). 닫힘 표시 하한16mm 반영; 접촉 정합은 별도 미완료 |
| 받침대 | 50×50×30 mm, x 0.086 | **50×53×33 mm, x 0.1003** | 사진 실측 |
| 바퀴 관절 이름 | `fl_steer_joint` / `fl_wheel_joint` … | **`steer_fl_joint` / `drive_fl_joint` …** | 팀 `swerve_controller` 가 찾는 이름 규칙(`steer_<fl|fr|rl|rr>_joint`, `drive_<..>_joint`). 위치·축·범위는 그대로 |
| 총질량 | 예시 퍽 포함 3.9 kg | **빈 로봇 모델 3.81 kg** | 기존 포함 질량에서 예시 퍽90g 제거. 새 빈 로봇 실측값이 아니며 base_link 관성은 유지 |

## 실물 ↔ 모델 관절 값 (`config/joint_map.json`)
`q[rad] = radians(q_deg_at_ref) + sign × (모터 Present Position − ref_tick) × 2π/4096`
- J2~J5: 줄자 높이 실측으로 확인(측정 자세 5개 + 받침 자세, 3 mm 안).
- J1: 받침 자세 4088 = 정면(−94.65°), 틱 증가 = 로봇 왼쪽 — 사진·실행 기록과 맞지만 줄자 확정은 아님.
- J1·그리퍼는 확장 위치 모드라 **받침 자세에서 전원을 켤 때** 이 지도가 맞습니다(지금까지의 실물 절차).
- 그리퍼: 틱 → 간격 표 보간, 손가락 q = (간격 − 58.507 mm)/2000. 그리퍼는 크랭크(2680 닫힘 ↔ 약 1045 열림 ↔ −593 반대쪽 닫힘).

## 꼭 알아야 할 것
1. **시작 자세:** 보기용 기본은 parked, 작업 대기 P0는 `pose:=p0`로 선택합니다. 토크 OFF 명령을 실행하지 않습니다. 물리 초기 접촉 안정성은 검증되지 않았습니다.
2. **그리퍼 닫힘:** 사용자 실측16mm를 표시합니다. 손가락/받침 교차와 충돌 근사 정합은 미완료이며 충돌 쌍을 제외해 통과시키지 않습니다.
3. **inertial 없는 고정 링크 25개**(센서·홀더 등)는 빈 로봇 URDF 질량 합3.81 kg 에 이미 포함된 것으로 봅니다. MuJoCo 는 이런 링크에 형상으로 질량을 붙이므로(`inertiafromgeom`) 총질량이 커집니다 → MuJoCo 에서는 `inertiafromgeom="false"` 또는 질량 0 처리. Isaac 은 고정 링크를 합치면 영향 없음.
4. **관절 속도:** URDF 는 하드웨어 한계입니다. 실물에서 써 본 최대는 약 0.84 rad/s(`operating_limits.yaml`). 학습 환경 상한은 이 파일을 보고 정하세요.
5. 질량 분포·관성은 원본 CAD 가정값입니다(총질량만 실측). 카메라 외부 파라미터(D435·버드아이)는 원본처럼 미확정입니다.

## 실행
- 보기(빌드 없이): `bash view.sh` (ROS 2 lyrical). colcon 패키지로도 빌드 가능(`nt_release_description`).
- 검사: 맨 위 '이번 배포본에서 실제로 한 검사' 표 참고(예전 `validate_release.py`/`validation.json` 은 3.9 kg·옛 모델 기준이라 넣지 않음).
- **Isaac USD: `usd/robot/robot.usda`** (10-27, 이 배포 URDF 로 새로 만듦, `tools/isaac_import.py`, 설정 `config/isaac_import_swerve.json` = 차체 고정 안 함).
  - 열 때 **Physics 변형을 `physx` 로 선택**해야 관절·질량이 생김(기본 선택 없음 = 물리 없음). Isaac Lab 예: `UsdFileCfg(usd_path=..., variants={"Physics": "physx"})`.
  - 확인함: articulation root = base_link, 바닥 고정 관절 없음, 움직이는 관절 15, 질량 3.810 kg, 조향 범위 = 엔코더 창 전체, 힘 조향 4.1·바퀴 3.0 N·m,
    속도 한계 `newton:velocityLimit` 조향 274.8°/s(4.7962 rad/s)·바퀴 453.4°/s(7.9137 rad/s), 손가락 mimic = `NewtonMimicAPI` (Isaac 6.2 는 PhysX 에서도 이 Newton 스키마를 읽음, `physxJoint:maxJointVelocity`·PhysxMimicJointAPI 는 안 씀 — 가져오기 도구 변경 기록).
  - 하지 않은 것: Isaac 안에서 물리로 굴려 보기(주행·mimic 실제 동작). 관절 이득(조향 500 N·m/rad·10, 팔 500·10, 손가락 1000·2)은 가져오기 설정 값이고 실물 맞춤 아님.
- 보기용 Isaac 이미지·관절 범위 그림: 이 판에는 넣지 않음(옛 판 것은 옛 모델). 다시 만들기 `tools/isaac_render.py all <실행 폴더>` (차체 고정 설정 `config/isaac_import_fixed.json` 을 써서 usd/ 를 덮어쓰므로, 그 뒤 `tools/isaac_import.py` 다시 실행).
- 다시 만들기: 맨 위 "다시 만들기 (순서 중요)" 참고 (`build_release.py` 만 돌리면 스워브 한계·배치 수정이 빠짐).

## 시뮬에서 주행하기
- 바퀴 위치(x ±64 mm, y ±112 mm, 바퀴 오프셋 26.9 mm), 조향 범위, 관절 이름이 실물 주행 설정(팀 `drive_only.yaml`)과 같습니다.
- 차처럼 "앞으로/옆으로/회전" 명령으로 움직이려면 팀 `swerve_controller`(ROS 2 control, `/cmd_vel`)를 붙이거나 같은 계산을 쓰면 됩니다. 계산 본체는 ROS 없는 C++(`swerve.hpp/.cpp` 의 `SwerveKinematics`)입니다.
- 2026-10-26 실물 확인: 좌우 주행 때 네 바퀴가 바깥쪽에 서도록 바뀐 규칙(fl·rr +90°, fr·rl −90°)은 로컬 팀 작업본(미커밋)에만 있습니다.
- 팔 관절 이름은 Codex 이름(`arm_joint1~5`, `gripper_finger_a_joint`)입니다. 팀 팔 컨트롤러 이름(`joint1~5`, `gripper_left_joint`)과 다르니, 팀 팔 컨트롤러를 그대로 붙일 때는 이름 매핑이 필요합니다.

## 이번 준비본 상태
J1 축 X: −4.250356→22.549994mm; 모터 출력축이 몸체 중심보다12mm 앞(+X). 모터 앞면→받침 뒷면40mm. 받침은 이동하지 않았습니다. 기존 USD/validation/images는 새 입력 검증 증거가 아니며 재생성 전 사용하지 마십시오. 구동 코드, joint_map, operating_limits, 실험 경로는 변경하지 않았습니다.

## 라이다 외형 및 Jetson 높이 준비 수정
라이다 위치/scan TF/프레임명은 보존하고 사진의 좌우 체결귀 방향에 맞춰 visual·collision만 함께90° 회전했습니다. 실제 scan0°는 확인한 것으로 승격하지 않습니다. Jetson은 전체링크를7.617mm 내려 CAD바닥z54.117→46.5mm(OpenCR판 윗면)에 맞췄습니다. 실제 스탠드오프/전체체결/접촉 검증은 미완료입니다. 위 팔판 아래면82.5mm 대비 최대높이81.266mm로 수직여유1.234mm. 개별관성이 없는 Jetson에 임의질량을 만들지 않았습니다.

## 준비본 표시 자세 선택 갱신 — A9F5 OFF 기록
기본 parked는 `ARM_20261026T023249Z/03_session.jsonl:10`의 torque OFF Present(11~16:4088/1097/2671/1966/2660/2058)를 기존 보정식으로 변환한 값입니다. 예전 10/25 torque ON 종료 기반 초기값은 `previous_default_positions`와 `poses/받침_최신`에 보존합니다. P0 선택은 유지합니다. 개도16mm 표시 조건은 유지합니다.
이는 실제 기록에 기반한 **표시 자세 선택 수정**이며 사진의 같은 촬영 순간 또는 영점 확정을 뜻하지 않습니다. J2 쪽 몸체와 그리퍼 뒤쪽의 틈, 받침 원본 메시 교차가 완전히 해결된 수정본은 아닙니다. 지지 접촉 자체는 의도된 상태입니다. 현재 자세의 새로운 물리 시험은 미실행이고 기존 USD/이미지의 기본 자세는 최신 증거로 사용하지 마십시오. URDF 장착·센서 위치·틱 매핑·구동 코드·실물은 바꾸지 않았습니다.

## 고정 예시 퍽 제거 — 2026-10-27 준비본
로봇은 빈 홀더6개로 시작합니다. test_puck_H1~H6의 link·fixed joint·visual·collision·inertial을 생성 단계에서 제거했습니다. 홀더6개와 그리퍼 주행 받침은 그대로입니다. 맵/학습환경의 독립 동적 퍽은 별도 생성 대상이며 이번 작업에서 생성하지 않았습니다.
기존3.9kg는 예시 퍽 포함 측정 조건입니다. 원기둥3×13g+네모3×17g=90g을 로봇에서 빼므로 authored mass 합은3.81kg입니다. 빈 로봇을 다시 잰 값은 아니며 base_link나 다른 링크 질량·관성으로90g을 재분배하지 않았습니다. 원본 퍽 메시 파일은 보존합니다.
기존 validation.json/이미지/USD/고정퍽 접촉 결과는 이전 모델 결과입니다. 새 변경검사는 task-2/audit_work/circled_gap_20261026/parked_off_update/full_record_review/no_fixed_pucks/CHECKS.json에 기록합니다. H3 고정 예시 퍽 충돌은 현재 로봇 실패로 계수하지 않습니다. 비퍽81개 노드가 동일하므로 모터/그리퍼·받침·상판의 원본 형상 교차 근거는 남습니다. 실제 release·구동·틱 매핑은 변경하지 않았습니다.
