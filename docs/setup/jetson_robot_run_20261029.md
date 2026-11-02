# 젯슨에서 로봇 실행 (2026-10-29, 새 구조 bringup)

노트북에서 만들고 모의로 확인한 판. 젯슨 설치 · 실행은 사람이 직접 한다.

## 0. 이미 아는 것 — 10-27 젯슨 매핑 기록 (`~/Downloads/wheelbase-mapping-data-20261027-v1.tar.gz`)

| 항목 | 값 |
|---|---|
| 젯슨 | Orin Nano · JetPack 7.2.1 · Ubuntu 24.04 · 사용자 `turtle` · NVMe 233 GB (190 GB 남음) |
| ROS | **Lyrical** (`/opt/ros/lyrical`) · slam_toolbox 2.10.0 · rosbag2 |
| 주행 | 새 구조 패키지(`swerve_controller` · `hardware`)를 젯슨에서 빌드해 실물 OpenCR 로 돌림 (주행 보드 1장, 팔 없음) |
| 포트 | USB 자리로 지정: 주행 OpenCR `/dev/serial/by-path/platform-3610000.usb-usb-0:2.4.3:1.0`, 앞 라이다 `…usb-0:2.4.4:1.0-port0` (허브) |
| 라이다 | Coin D4 (`coin_d4_mapping_driver`), `/lidar_front/scan_mapping` (2026-10-31 부터 slam.launch.py 가 `/lidar_front/scan` 으로 바꿔 붙임), TF `base_footprint → lidar_front_link` = (0.110, 0.005, 0.0545) yaw π |
| 결과 | 759 s · 48 구간 · 직선 5.76 m 매핑, 지도 정상(벽 사이 1.45 × 3.20 m). 라이다 방향 실물 확인(상자 +X)은 **아직** |
| 그때 주행 설정 | 10-27 판: 가속 0.2 / 0.8 · 조향 프로파일 40/10 · 자이로 안 씀(`use_imu: false`) · 정면 유지 없음 |

→ 젯슨에서 Lyrical · ros2_control · 우리 하드웨어 플러그인 빌드 · OpenCR 통신은 **이미 된다**. 이번 판은 10-28 실물로 정한 주행 설정을 얹은 것.

## 0-1. 이 노트북과 젯슨 — 10-18 기록
- 노트북에서 `scp …/orin_mock_sources_20261018.tar.gz turtle@10.2.12.123:~/` · `ssh turtle@10.2.12.123` 한 기록 (bash 기록).
- 그 묶음(개인 작업 폴더의 orin_preflight_20261018)은 옛 구조 7 패키지 · 가짜 하드웨어 전용, 노트북에서 빌드 · 단위 시험 111 · mock 확인. 젯슨 ARM64 결과는 그 문서에 기록 없음.
- 그 문서 메모: 공식 지원표상 Lyrical 은 Ubuntu 26.04 가 1순위, 24.04 는 소스 빌드 대상(Tier 3). 그러나 10-27 기록에서 젯슨 `/opt/ros/lyrical` 로 실제 주행 · SLAM 이 돌았으므로 젯슨에는 이미 Lyrical 이 있음.

## 1. 노트북 → 젯슨 복사 (커밋 · 푸시 없이)

```bash
# 노트북에서. 젯슨 주소는 노트북 접속 기록(10-18 scp · ssh) 기준 turtle@10.2.12.123 — 네트워크가 바뀌면 다를 수 있음
rsync -a --delete --exclude build/ --exclude install/ --exclude log/ --exclude runs/ \
  <노트북의 저장소 경로>/ turtle@10.2.12.123:~/p2_main_1029/
```
10-27 매핑 작업 폴더(`~/ros2_front_mapping_v1_ws` 등)는 그대로 둔다. 같은 시간에 두 판을 같이 띄우지 않는다.

## 2. 젯슨에서 — 확인 → 빌드 → 모의

```bash
cd ~/p2_main_1029
bash tools/jetson/jetson_check.sh        # 읽기만. 결과는 노트북에서 확인
bash tools/jetson/build_jetson.sh        # 작업자 1개 · -j2 (단위 시험까지: --test)
bash tools/jetson/run_robot.sh drive mock     # 로봇 없이: swerve_controller 활성 · 범위 줄 확인 → Ctrl+C
bash tools/jetson/run_robot.sh full  mock     # 팔까지 로봇 없이: 컨트롤러 4개
```

## 3. 포트 이름 고정 (허브에 여러 장치)

```bash
python3 tools/jetson/make_udev_rules.py   # OpenCR 을 모두 뺀 상태에서 시작 → 주행 보드 꽂고 Enter → 팔 보드 꽂고 Enter
# 출력의 sudo cp … && sudo udevadm control --reload-rules && sudo udevadm trigger 를 직접
ls -l /dev/opencr_drive /dev/opencr_arm
```
시리얼번호가 보드마다 다르면 시리얼번호 기준(꽂는 구멍이 바뀌어도 같음), 같으면 USB 자리 기준(구멍 고정 · 라벨).
규칙 없이 바로 쓰려면 10-27 처럼 `NT_DRIVE_PORT=/dev/serial/by-path/…` (구멍을 바꾸면 안 됨).
조향 ID 11~14 와 팔 J1~J4 ID 11~14 가 같아도 보드(USB)가 따로라 문제없다.

## 4. 실물

```bash
bash tools/jetson/run_robot.sh drive opencr   # 바퀴만 (팔 보드 없이 또는 꽂혀 있어도 주행 보드만 씀). 처음엔 바퀴 띄우고
bash tools/jetson/run_robot.sh full  opencr   # 시연 모드: 켤 때 바퀴 잠금 → 팔 HOME 성공 뒤 주행
#   팔 HOME: ros2 action send_goal /arm_task interfaces/action/ArmTask "{task: 0}"
```
- 거부되는 경우: 다른 ros2_control_node 가 떠 있음 · dialout 권한 없음 · 포트 없음 · 주행/팔 포트가 같은 장치.
- 라이다 · SLAM 은 팀원 실행 그대로 따로 띄운다. 우리 컨트롤러가 `odom → base_footprint` TF 를 내므로 SLAM 은 `map → odom` 만.
- 노트북 RViz 로 보려면 두 기기 `ROS_DOMAIN_ID` 를 같게 (기본 0).

## 5. 이번 판에서 10-27 젯슨 판과 달라진 주행 (모두 10-28 실물 비교로 정함)
- 가속 0.5 m/s² · 2.0 rad/s² (예전 0.2 / 0.8), 조향 프로파일 200/60 · 바퀴 가속 40 (예전 40/10 · 20)
- 정면 유지(자이로) kp 2 · 이동 속도 비례 상한, 자세 고르기 가중치 2, coherent
- rl 선 한계 170°, 모듈별 조향 범위를 **팔 있는 실행에서도** 적용 (예전엔 바퀴만 실행에서만)
- 시연용 전체 지도(`config/hardware_map.yaml`) 바퀴 8개 ID · 영점 · 방향을 실측값으로 (예전 ID 1~8 가정)

## 6. 제어 주기 지연 줄이기 (2026-10-29 측정 · 권장)
- 주행 컨트롤러 계산은 한 주기(10 ms)의 아주 작은 일부: 노트북에서 모든 기능 켜고 최악 조건 상위 0.1 % 가 최적화 없음 35 µs / 최적화(-O2) 8.6 µs.
  → `swerve_controller` · `hardware` CMake 기본을 Release 로 바꿈(빌드 유형을 따로 주면 그걸 따름). `build_jetson.sh` 도 Release.
- 실제 지연은 계산이 아니라 **스케줄링**: 실행 로그의 `Could not enable FIFO RT scheduling policy` = controller_manager 가 실시간 우선순위를 못 받음.
  라이다 SLAM · YOLO 와 같이 돌면 100 Hz 주기가 밀릴 수 있음 (10-20 모의에서 최대 45 ms 밀림 기록).
- 켜는 법 (시스템 설정 — 사용자가 직접, ros2_control 공식 안내 방식):
  ```bash
  sudo groupadd -f realtime && sudo usermod -aG realtime $USER
  printf '@realtime soft rtprio 99\n@realtime soft priority 99\n@realtime soft memlock 102400\n@realtime hard rtprio 99\n@realtime hard priority 99\n@realtime hard memlock 102400\n' | sudo tee /etc/security/limits.d/realtime.conf
  # 다시 로그인 후: ulimit -r 이 99 면 됨. 실행 로그에서 위 경고가 사라졌는지 확인
  ```
- 켠 뒤 확인: `run_robot.sh` 실행 로그에서 `overrun` · `missed` 경고 수를 라이다 · YOLO 를 켠 상태로 비교.
