# sysid — 주행 응답 측정 (실물 · Isaac · 학습 플랜트 비교)

지역 계획기 학습 플랜트의 도메인 랜덤화 (지연 · 시간 상수 · 미끄럼 · 최소 속도 · 조향 전환) 를 실측으로 맞추려고 씀.
같은 계단 명령 (`steps.py`, 28 개 · 약 2 분) 을 실물 · Isaac · 학습 플랜트에 넣고 `analyze.py` 로 같은 표를 만듦.

| 파일 | 내용 |
|---|---|
| `steps.py` | 계단 순서 (앞뒤 0.05 · 0.1 · 0.2, 옆 0.1 · 0.2, 대각, 회전 0.25 · 0.5, 느린 0.01–0.03, 가면서 돌기, 멈추지 않는 90° 전환). 반대 계단이 바로 뒤라 출발점 둘레 약 0.4 m 안 |
| `record.py` | `/cmd_vel` (TwistStamped, 50 Hz, 0.2 m/s · 0.5 rad/s 로 자름) 을 보내며 `/odom` · `/joint_states` · IMU · TF (map · odom → base_footprint) 저장 → `data.npz` · `segments.json` |
| `analyze.py` | 계단마다 지연 · 시간 상수 (순수 지연 + 1 차 지연 맞춤) · 정상 비율 · 멈춤 시간 · 거리 · 미끄럼 (map / odom), 최소 속도, 90° 전환 → DR 제안. `--plot` 명령 대 odom 그림 |
| `plant_response.py` | 학습 플랜트 (팀 swerve.yaml) 에 같은 계단 → 같은 형식. `--dr fast · mid · slow` = DR 범위 끝 · 가운데 |

## 실물 측정 절차

1. 바닥: 시연 경기장과 같은 바닥, 로봇 둘레 앞뒤 · 옆 1 m 비움. 배터리 충전 상태 적어 둠
2. 로봇: `swerve_controller` + control_master (DRIVE 확인), **Nav2 · teleop 끔** (`/cmd_vel` 발행자가 있으면 `record.py` 가 시작 안 함)
3. 미끄럼까지 보려면 slam_toolbox localization 도 띄움 (`slam.launch.py slam_mode:=localization map:=…`) → TF map → base_footprint. 없으면 odom 기준 값만 나옴
4. 기록 (사람이 비상 정지 옆에서):
   ```bash
   python3 record.py --out ~/sysid/floor1 --dry_run      # 순서 확인
   python3 record.py --out ~/sysid/floor1 --scale 0.5    # 처음엔 절반 속도로
   python3 record.py --out ~/sysid/floor1                # 본 측정 (Ctrl+C = 0 을 1 s 보내고 저장)
   ```
5. 분석: `python3 analyze.py ~/sysid/floor1 --plot`
6. 2–3 번 반복 (배터리 상태 달리) → 범위로 씀

## 지금까지 결과 (2026-11-04)

| 항목 | 학습 플랜트 fast · mid · slow | Isaac (팀 로봇, 실물 한계) | 실물 |
|---|---|---|---|
| 명령 → 움직임 지연 (중앙) | 0.035 · 0.064 · 0.091 s | **0.24 s** (조향 크게 돌 때 0.4–0.7 s) | 측정 전 |
| 시간 상수 (중앙) | 0.13 · 0.15 · 0.22 s | 0.16 s | 측정 전 |
| 90° 전환 (새 축 63 %) | 0.24 · 0.31 · 0.46 s | 0.44 s | 측정 전 |
| 최소 속도 | 0.01 m/s 도 움직임 | 0.01 m/s 도 움직임 | 측정 전 |

- Isaac 지연이 학습 DR 끝 (slow) 보다 2.5 배 김. 0.01 m/s 처럼 조향이 거의 안 바뀌는 계단도 0.2 s → 조향이 아닌 고정 지연 (Isaac 브리지 · 스워브 액션 · 물리 경로). 실물 측정 전에는 어느 쪽이 실물에 가까운지 모름
- 학습 플랜트 시간 상수 0.2 m/s 계단 0.22 s 는 가속 한계 0.5 m/s² 때문 (0.2 m/s 까지 0.4 s)
- 지연 정밀도는 odom 주기 (50 Hz) 한 칸 ±20 ms (가짜 데이터로 확인: 넣은 0.08 s → 0.064 s, 시간 상수 0.15 → 0.156 s, 미끄럼 · 정상 비율 · 최소 속도는 그대로 나옴)
