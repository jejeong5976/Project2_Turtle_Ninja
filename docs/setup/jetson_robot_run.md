# 젯슨에서 로봇 실행

젯슨 설치 · 실행은 사람이 직접 한다. 젯슨 기본 설치(JetPack · SSD)는 `jetson_orin_nano_ssd_setup.md`.

## 0. 환경
| 항목 | 값 |
|---|---|
| 젯슨 | Orin Nano · JetPack 7 · Ubuntu 24.04 · 사용자 `turtle` |
| ROS | Lyrical (`/opt/ros/lyrical`), slam_toolbox, `ros-lyrical-coin-d4-driver` (LDS-03) |
| 포트 | 주행 OpenCR `/dev/opencr_drive`, 팔 OpenCR `/dev/opencr_arm` (3 절 udev 규칙). 앞 · 뒤 라이다는 USB 자리(`/dev/serial/by-path/…`)로 — 기본값은 `run_robot.sh` 의 `LF` · `LR` |
| 라이다 | 앞 · 뒤 LDS-03, 스캔면 바닥에서 60 mm, 앞뒤 ±0.1145 m (URDF `lidar_front_link` · `lidar_rear_link`) |

## 1. 노트북 → 젯슨 복사

```bash
rsync -a --delete --exclude build/ --exclude install/ --exclude log/ --exclude runs/ \
  <노트북의 저장소 경로>/ turtle@<젯슨 주소>:~/p2_main/
```

## 2. 젯슨에서 — 확인 → 빌드 → 모의

```bash
cd ~/p2_main
bash tools/jetson/jetson_check.sh        # 읽기만
bash tools/jetson/build_jetson.sh        # 작업자 1개 · -j2. --test 면 단위 시험까지 (실패 시 종료 코드 1)
bash tools/jetson/run_robot.sh drive mock     # 로봇 없이: swerve_controller 활성 · 조향 범위 줄 확인 → Ctrl+C
bash tools/jetson/run_robot.sh full  mock     # 팔까지 로봇 없이
```

## 3. 포트 이름 고정 (허브에 여러 장치)

```bash
python3 tools/jetson/make_udev_rules.py   # OpenCR 을 모두 뺀 상태에서 시작 → 주행 보드 꽂고 Enter → 팔 보드 꽂고 Enter
# 출력의 sudo cp … && sudo udevadm control --reload-rules && sudo udevadm trigger 를 직접
ls -l /dev/opencr_drive /dev/opencr_arm
```
규칙 없이 쓰려면 `NT_DRIVE_PORT=/dev/serial/by-path/…` (꽂는 구멍을 바꾸면 안 됨).
조향 ID 11~14 와 팔 J1~J4 ID 11~14 가 같아도 보드(USB)가 따로라 문제없다.

## 4. 실물 실행

```bash
bash tools/jetson/run_robot.sh drive opencr   # 바퀴만 (팔 OpenCR 은 젯슨에 안 붙임). 처음엔 바퀴를 띄우고
bash tools/jetson/run_robot.sh full  opencr   # 팔까지: 켤 때 바퀴 잠금 → 팔 HOME 성공 뒤 주행
#   팔 HOME: ros2 action send_goal /arm_task interfaces/action/ArmTask "{task: 0}"
bash tools/jetson/run_robot.sh slam  opencr   # 바퀴 + 앞 · 뒤 라이다 + 지도 만들기 + 위치 감시
#   라이다를 다른 USB 자리에 꽂았으면: NT_LIDAR_FRONT=/dev/serial/by-path/… NT_LIDAR_REAR=/dev/serial/by-path/… bash tools/jetson/run_robot.sh slam opencr
#   뒤 라이다 없이: NT_LIDAR_REAR=none (뒤 반원이 비어 후진 · 옆 주행에서 앞 물체 정지가 뒤를 못 봄)
bash tools/jetson/save_map.sh arena           # 지도 저장 (slam 이 떠 있는 동안, 로봇을 세우고) → ~/maps/arena.*
bash tools/jetson/run_robot.sh remap  opencr ~/maps/arena   # 시연 기본: 저장한 지도 불러와 이어 그리기 + 위치 감시 (지도 딸 때 출발 자세에 놓고 시작)
bash tools/jetson/run_robot.sh locate opencr ~/maps/arena   # 저장한 지도에서 위치 찾기만 (지도 고정) + 위치 감시
```
- `slam` · `remap` · `locate` 는 `navigation/slam.launch.py` 를 부르고, 위치 감시(`pose_guard`)도 같이 뜬다.
  `/localization_health` (정상 · 경고 · 정지)와 `/imu_motion_bias` (움직일 때 자이로 보정)를 낸다.
- 시연은 `remap` (2026-11-05 결정): 시연 중 벽판이 밀릴 수 있어 지도를 고정하지 않고 이어 그린다. 시작 자세 = 저장 지도의 첫 노드 (`map_start_at_dock`) → 지도 딸 때와 같은 출발 자세에 놓는다 (바닥 표시).
- locate 뒤 rviz 의 2D Pose Estimate 로 시작 위치를 찍는다 (`slam_rviz:=true` 또는 노트북 rviz `navigation/rviz/slam_view.rviz`).
- 실행 중 화면에는 경고 · 오류와 위치 감시 상태(2 s 마다 정상 · 경고 · 정지)만 보이고, 전체 기록은 `runs/robot_<모드>_…/launch.log`.
- 거부되는 경우: 다른 ros2_control_node 가 떠 있음 · 권한 없음 · 포트 없음 · 주행/팔 포트 또는 앞/뒤 라이다가 같은 장치 · 검사하는 인자를 덧붙임.
- 노트북 rviz 로 보려면 두 기기 `ROS_DOMAIN_ID` 를 같게. 와이파이 멀티캐스트가 불안정하면 양쪽에
  `export ROS_STATIC_PEERS=<상대 주소>` (젯슨 창마다 노트북 주소, 노트북에는 젯슨 주소).
- 젯슨 명령은 젯슨 터미널(`turtle@turtle`)에서. 노트북 창은 rviz · scp 만.

## 5. 경로 주행 · 점검 도구 (`tools/`)

```bash
python3 tools/drive/path_follow.py --frame map --abs --guard --waypoints="x,y;x,y"   # 지도 기준 경유점 주행 (경유점이 − 로 시작하면 = 를 붙임)
python3 tools/diagnostics/sensor_check.py                                          # 가만히 → 제자리 360° → 직진 왕복으로 IMU · odom · 라이다 대조
```

## 6. 제어 주기 지연
- 컨트롤러 계산은 한 주기(10 ms)의 아주 작은 일부: 최악 조건 상위 0.1 % 가 최적화(-O2) 8.6 µs. 라이다 합치기는 메시지당 약 0.03 ms (노트북).
- 실제 지연은 **스케줄링**: 실행 로그의 `Could not enable FIFO RT scheduling policy` = controller_manager 가 실시간 우선순위를 못 받음.
  라이다 SLAM · YOLO 와 같이 돌면 100 Hz 주기가 밀릴 수 있음.
- 켜는 법 (시스템 설정 — 사용자가 직접, ros2_control 공식 안내 방식):
  ```bash
  sudo groupadd -f realtime && sudo usermod -aG realtime $USER
  printf '@realtime soft rtprio 99\n@realtime soft priority 99\n@realtime soft memlock 102400\n@realtime hard rtprio 99\n@realtime hard priority 99\n@realtime hard memlock 102400\n' | sudo tee /etc/security/limits.d/realtime.conf
  # 다시 로그인 후: ulimit -r 이 99 면 됨. 실행 로그에서 위 경고가 사라졌는지 확인
  ```
- 확인: SLAM · 인지를 같이 켠 상태에서 `top` (CPU), `ros2 topic hz /scan /odom`, 실행 로그의 `overrun` · `missed` 경고 수.
