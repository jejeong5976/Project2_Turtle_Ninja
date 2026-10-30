# 배치 수정판 변경·검증 기록 (2026-10-27, Claude Code)

## 바꾼 파일 (이 폴더만. 이전 배포본 nt_urdf_release_20261027_swerve 는 작업 당시 그대로였고, 10-27 저녁 정리하며 지움)
- 새로: `tools/apply_layout.py`, `config/layout_20261027.yaml`, `meshes/generated/base_without_arm_top_plate_layout.obj`(배터리판 조각만 +9 mm),
  `tools/apply_initial_pose.py`, `config/isaac_lab_init_state.py`, `tools/verify_isaaclab.py`, `tools/view_isaaclab.py`, `tools/verify_isaac.py`(순수 Isaac Sim 경로 — PhysX 텐서 연결 실패로 미완)
- 바뀜: `urdf/robot.urdf`(6e1a… → 78e74a29…), `usd/`(새 URDF 로 재생성 + parked 초기값), `README_KO.md`(맨 위 절·옛 P0 권장/재생성 문구), `config/operating_limits.yaml`(19.6 mm → 16 mm)

## 실제 적용값 (바닥 기준, base_link 는 바닥 위 72 mm)
| 항목 | 값 |
|---|---|
| 라이다판 | +8.4 mm → 밑면 20.0 mm (실측 20) |
| 앞·뒤 라이다 | +8.4 mm, 스캔면 오프셋 28 → 31 mm → 스캔면 60.0 mm (실측 60) |
| 배터리판·배터리 | +9 mm → 판 윗면 80.0 mm (실측 80), 밑면 71 mm (실측 75, 모델 판 두께 9 mm) |
| OpenCR판·팔판 | 그대로 (윗면 118.5/163.5 vs 실측 120/165) |
| OpenCR② 주행 | PCB 중심 (+1, +75, 55.3) mm |
| OpenCR① 팔(추정) | PCB 중심 (−92, +35, 55.3) mm |
| imu_link | (1, 75, 55.8) mm, yaw +90° |
| Jetson | 마운트 (−5.4, −47.5, −7.6) mm yaw −90°, 충돌 상자 x −38.5~64.5 y −99.4~−8.5 z 46.5~81.3 |
| 라이다↔Jetson 연결 보드 | 표시 상자 (110, 49, 47.5) mm, 질량·충돌 없음 |
| Isaac 시작 자세 | 팔 parked(A9F5), 손가락 a·b −0.0212536 m(16 mm), Physics 변형 기본 physx |

## 검증 (Isaac Lab, PhysX, `config/verify_isaaclab.json`)
- 정착 후 base 바닥 위 72 mm, 바퀴 중심 33 mm, 기울기 없음 → 스캔면 60 mm.
- 팔 5 관절 parked 대비 최대 0.63°.
- **그리퍼는 16 mm 명령에 21.3 mm 에서 멈춤.** 원인(`tools/check_park_stand.py`, 표면 표본 검사): 받침이 아니라 **손가락 a·b 충돌 근사끼리** 16 mm 에서 4.8 mm 겹침, 21.3 mm 에서 0.06 mm → 정확히 거기서 멈춤.
  손가락–받침 겹침은 개도와 무관하게 약 1.5 mm (임시 상자·실제 받침 모델 mani_home 동일) → 받침 충돌 모양 교체로는 해결 안 됨.
- 손가락 a↔b 충돌 제외 시도: FilteredPairsAPI(`tools/apply_collision_filters.py`), CollisionGroup, 두 손가락 모두 구동(1000/2) — **모두 PhysX NaN 폭주**.
  손가락이 한계(−0.021254)까지 닫히면 mimic 인 손가락 b 가 시작부터 −0.022022 로 한계를 넘음 → mimic 구속 + 관절 한계 문제로 추정. 16.5 mm 목표도 같음.
  → **USD 는 충돌 제외 없는 안정 상태로 되돌림**(physics.usda 에 filteredPairs/그룹 없음). apply_collision_filters.py 는 재생성 절차에 넣지 않음.
- 팔 토크 OFF 흉내(팔 관절 힘 0, 3 s, `config/verify_isaaclab_arm_off.json`): 팔이 받침에 얹혀 버팀(관절 최대 6° 움직임, 손가락 2 mm 위로 — 받침 1.5 mm 겹침이 밀어 올림). 뚫고 떨어지지 않음.
- **최종 조치(URDF 7858f243…)**: 손가락 a·b 충돌을 보이는 모양에서 CoACD(threshold 0.02, 각 48 조각)로 다시 만듦(`tools/make_finger_collision.py` → `config/finger_collision.json`, apply_layout 이 교체). 충돌 제외는 쓰지 않음.
  결과(Isaac Lab): 팔 잡고 있을 때 그리퍼 19.4 mm(이전 21.3), 팔 토크 OFF 3 s 팔 0.6° 이내로 받침에 얹힘(이전 6°), 폭주 없음.
- 남은 차(19.4 vs 16 mm): 보이는 손가락 모양 자체가 16 mm 표시값에서 서로 2.5 mm, 받침 벽과 1.4 mm 겹침 → 손가락 형상 두께나 관절값↔간격 대응(gap0 58.507)이 실물과 다름.
  operating_limits 의 원래 "모델 하한 19.6 mm(손가락 충돌 형상 한계)" 설명이 맞았음 → 문구 복원·보완. 고치려면 실측 필요(아래).
- 필요한 실측(1~2개): 그리퍼를 16 mm 로 닫았을 때 두 손가락 **바깥면 사이 전체 폭**, 가능하면 손가락 한 개 두께(패드 면~바깥면, 받침에 들어가는 부분).
- 받침 벽 얇게 하기 시도(park_stand_collision)는 원인이 아니어서 설정에서 뺌(도구 코드만 남음).
- 같은 설정으로 이전 배포본 USD: 팔 최대 5.4°, 그리퍼 35.5 mm (`config/verify_isaaclab_ref.json`).
- 팔 링크는 모두 base 위 127 mm 이상 → 옮긴 Jetson 상자(위끝 81 mm)와 겹침 없음(링크 원점 기준). 받침·접는 동작의 정밀 경로 검사(MuJoCo review_paths 등)는 다시 돌리지 않음.
- USD 에서 고정 관절이 합쳐져 base_scan·imu_link 같은 형상 없는 기준점은 prim 으로 남지 않음(이전 판도 같음). 시뮬 센서는 URDF 에서 위치를 읽어야 함.

## 남은 것
- 보드 스탠드오프 높이·미세 yaw·±10 mm 보다 정확한 xy, 뒤 라이다 높이 실측(같은 값으로 추정 적용).
- 배터리판 두께 차(밑면 4 mm), 판을 가로지르는 기둥 길이 미조정(보기만 영향).
- 주행 학습용 팔 고정 USD(swerve_rl_20261027/prepare_asset.py)는 아직 이전 URDF(6e1a…) 기준 → 위치 수정 전후 비교 결과(collision_c_sweep 등)는 **이전 모델** 결과임.

## 팀원 시뮬(~/sim/robot_sim) 확인
- URDF 는 이전 배포본과 내용 동일(메시 경로만 상대 경로).
- 시작 자세가 `C1_P0` + `'gripper_.*': 0.0` (sim_core/turtleninja_maze_env.py:75,139) → **P0 자세에 그리퍼 58.5 mm 열림**. 보고된 증상의 원인.
  고치려면 `config/isaac_lab_init_state.py` 의 parked 값(팔 positions + 손가락 a·b −0.0212536)을 init_state 로 쓰고 hold 목표도 같게.
- 라이다: 코드(add_lidars)는 URDF base_scan 위치를 쓰므로 이전 URDF 기준 스캔면 **48.6 mm**, README 의 "바닥 위 7.2 cm" 설명과 다름. 새 URDF 를 쓰면 60 mm.
  선반(7 cm)·퍽(6 cm) 미검출을 높이만으로 설명할 수 없음(48.6 mm 면 둘 다 스캔면보다 높음) → 레이캐스트 대상(prim 표현식, 추적 여부) 등 다른 원인 확인 필요. 실제 장면 실행은 하지 않음.
- 변환 설정: self_collision false, Convex Hull, merge_fixed_joints true.


## 그리퍼 최종 수정 (URDF 02d589cb…)
원인(최종 확인): 실측 16 mm 는 고무패드 안쪽(CAD 에 패드 포함, 모델 패드끼리 15.97 mm 로 일치). 실제 손가락 표면끼리는 닫힘에서 0.2 mm 이내로 거의 맞닿지만 겹치지 않음(광선 판정 0).
볼록 충돌 근사가 이 틈과 받침 벽 모서리를 메워 시뮬에서만 닿았음. 받침은 그리퍼 중심에서 좌우 1.15 mm 치우쳐 있었음.
- 손가락 충돌: CoACD(threshold 0.01, 각 96 조각) — `tools/make_finger_collision.py 0.01 96`
- 손가락 a↔b 충돌 제외(`tools/apply_collision_filters.py`, USD): 실물은 평행 닫힘·16 mm 기구 멈춤으로 서로 안 닿음. 관절 한계가 멈춤 역할.
- 받침(임시 상자): 위치를 그리퍼 중심에 맞춤(+0.5 mm 앞, +1.15 mm 왼), 충돌 벽 윗부분 4.5 mm 낮춤(33 → 28.5, 볼록 근사 모서리 메움 제거). 보이는 모양·바깥 크기 그대로.
- 손가락 관절 한계: 바깥쪽 1e-6 m 내림(−0.021254 / 0.010247) — 16 mm 초기값이 반올림으로 한계 밖이 되던 문제.
- 이전 시도 철회: 패드 1.5 mm 가정(CAD 에 패드 있음), 받침 벽 얇게 하기.
검증(Isaac Lab): 팔 잡고 있을 때 그리퍼 **16.0 mm**, 팔 0.18°. 팔 토크 OFF 3 s 그리퍼 16.0 mm, 팔 2.0°(받침이 받침), 폭주 없음.
꺼냈다 넣기(`tools/verify_insert.py`, parked→P0 2 s→parked 2 s, 관절 직선 보간): 꺼낼 때 최대 0.43°, 넣을 때 2.0°(받침 가장자리 스침), 끝 팔 2.0°·그리퍼 16.9 mm, 폭주 없음.
재생성 순서에 `apply_collision_filters.py` 추가(isaac_import → apply_initial_pose → apply_collision_filters).

## 라이다 참고 CAD (COIN_D4_OPTICAL_ORIGIN_REFERENCE.f3d, 튜터)
- F3D 는 zstd 압축 zip. 원점 = 광학 원점(회전부 원통 면 중앙), 라이다 위쪽 = CAD +Z, 장착면 평면 이름 BOTTOM_MOUNT_Z_MINUS_26p5_MM. BREP(.smb) 치수는 직접 읽지 못함.
- 실측 스캔면 60 mm − 26.5 = 장착면 33.5 mm → 모델 라이다판 윗면 29 mm(실물 판 5 mm 면 약 25)보다 4.5~8.5 mm 위 → 스페이서 또는 기준 차이 확인 필요.
- 모델 스캔면은 이미 바닥 60 mm(+28 재가산 아님). 외형 교체는 Fusion 에서 원점 유지 STL/OBJ 내보내기 필요.

## 키보드 데모 팔 궤적 = 실물 최단 실행
- `config/arm_replay_A9F5.json` 원본을 A9F5 실물 3/3 중 가장 짧은 **ARM_20261026T022813Z**(재생 26.36 s; 023047Z 26.42, 023249Z 26.42)로 바꿈. 이전 파일은 `config/arm_replay_A9F5_023249Z.json` 로 보존. `tools/make_arm_replay.py [ARM_실행이름]`.
- 크랭크·링크 막대 움직임: `tools/make_crank_linkage.py` → `config/crank_linkage.json`, `meshes/generated/crank_anim_*.obj`. 보이는 모양만(물리 무관), 실물 틱 각도 기준, 막대 끝 어긋남 79 mm 0.7 / 29 mm 1.7 / 16 mm 5.7 mm.
- 팔 재생 배속 기본 2배([ ] 0.5~4배, 1배 = 실물 022813Z 속도). 받침→P0 3.0→1.5 s, P0→받침 5.2→2.6 s.
- 데모 화면 50 Hz(물리 200 Hz) + 실제 시간 맞춤, 리셋(R/Backspace/Home)이 재생도 멈춤.
- 데모 물리 CPU 기본(NT_SIM_DEVICE=cuda:0 로 되돌림): GPU 11 fps(실시간의 약 0.2배, 끊김) → CPU 39~45 fps(약 0.9배).
- 1번 키(P0→받침 넣기 단독) 뺌(사용자 요청). 미션 전체(M)는 그대로 넣기까지 포함.

## 2026-10-27 배포 정리 (최종)
- 키보드 시연: 팀 sim 미로·카펫·파노라마·천장 조명·퍽 3개(NT_TEAM_SIM), 카메라 창 2개(birdeye 팀 K + 어안 D, wrist D435 공장 K, C 로 끄기), 팔 자유 조작(3~7 + W/S), 대기자세 P · 퍽 집기 자세 O, 성공 실행 14개 궤적 선택(9/0), 안내 창(Arm replay).
- 받침 안에서는 실측 경로만: 받침에 있는데 1 → 꺼내기 먼저, 밖에서 2·M → 복귀 먼저, P·O 도 받침 안이면 꺼내기 먼저. 시작 자세 맞추기 = 관절 최대 1.2 rad/s(0.3 s 고정이면 받침 안에서 튐). 1 연타 때 걸렸다 말았다 하던 원인.
- 복귀 배속 시험(verify_return_speed): 1·1.5·2배 모두 끝 0.6° — 걸림 원인은 배속 아님.
- 물리·화면 분리: 물리는 실제 시간, 화면은 따라잡을 때만(카메라 창 켜면 약 12 fps, 1.00배).
- 시연이 배포 폴더만으로 돌도록 `swerve/nt_candidate.py` 포함, 팀 sim 경로는 환경 변수(없으면 빈 바닥).
- 최종 점검: README '최종 점검' 표.
- 학습 묶음 `config/nt_train_kit.py`(robot_cfg · apply_collision_filters · swerve · Replay) + `tools/check_train_kit.py`: GPU env 8개 모두 통과(parked 16.0 mm, 꺼내기 0.11°, 복귀 0.60~0.62°, 직진 0.260 m/s). 팀 sim 설정과 다른 점은 README '학습에 쓰는 법' 표.
- 크랭크 표시를 학습 묶음에 추가: `nt_train_kit.CrankVisuals(stage)` (sim.reset() 전 생성, 렌더 전 `update(env별 손가락 q)`). GPU 물리에서 확인: 손가락 이동 방향·거리가 `crank_linkage.json` 축과 0.01 mm 안으로 같음(49·29·16 mm), q=0 에서 CAD 막대와 같은 자리.
- `tools/record_train_kit.py` 새로 작성: 팀 미로(maze_real.usd·카펫·파노라마·천장 조명) 칸 중심에 env 8개, env 별 다른 스크립트 동작, 벽 막기(0.45 s 뒤 로봇 외곽 — parked 에서 팔·그리퍼가 오른쪽 위로 y −0.306 m 까지 나옴 — 과 벽판·기둥 여유가 3 cm 아래로 줄 명령이면 이동·회전 0; 바닥 받침은 차체 폭 ±0.149 m 로 따로). 영상·이미지는 배포 zip 과 따로.
- 배포 zip `../nt_urdf_release_20261027_layout.zip` (영상·이미지 `images/train_kit_*`, `config/record_train_kit.json`, `__pycache__` 제외). 이전 `../nt_urdf_release_20261027_layout.tar.gz` 는 CrankVisuals 이전 판.
- **앞 라이다 스캔 방향 90° → 180° (URDF 02d589cb… → 7476204d…).** 실물이 앞뒤 180° 돌려 장착(사용자 확인), Jetson 매핑(wheelbase-mapping-data-20261027-v1, NJWQJ7 yaw π)에서 지도 정상.
  `lidar_front_mount` rpy z 1.57079633 → 3.14159265, 같은 링크의 visual 5개·collision 1개 rpy z → 0 (차체 기준 겉모양 그대로, FK 차이 1e-8). 뒤 라이다·x·y·높이·USD 는 그대로. tools/build_release.py·apply_layout.py 로 다시 만들면 예전 90° 로 돌아가므로 이 값을 다시 넣을 것.
- **뒤 라이다도 270° → 0° (URDF 7476204d… → 026e4929…).** 사용자 확인: 뒤도 앞과 똑같이 앞뒤 180° 돌려 장착. 앞뒤 서로 180° 반대 관계 유지. visual 5개·collision 1개 rpy z → 0 (차체 기준 겉모양 그대로, FK 차이 1e-8). 뒤 라이다는 매핑 데이터 없음.
