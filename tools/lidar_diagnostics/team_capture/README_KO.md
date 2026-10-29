# 휠베이스 팀 전달 자료 / 현재 매핑 세션 기록 방법

2026-10-27. 팀원 요청의 목적은 (1) 로봇 없이 scan/odom/TF/휠 피드백을
같이 분석할 수 있는 bag 확보, (2) 지도와 그래프 저장, (3) 라이다와 차체
좌표계의 방향을 실제로 확인하여 URDF 기본값에 반영하는 것입니다.
현재 NJWQJ7 연속 매핑과 기존 휠/라이다 설정을 유지하면서 이 목적을 따릅니다.

## 기존 실행과의 차이

팀원 원본 `run_scan.sh`는 다른 bringup, `base_scan` 프레임, `/scan` 토픽과
기본 yaw=pi/2를 사용합니다. 현재는 별도 wheel launch와
`front_mapping_trial.launch.xml`이 실행 중이고, 후자가 전면 라이다 입력과
SLAM을 함께 실행합니다. **지금 원본 run_scan.sh나 별도 라이다 드라이버를
추가 실행하면 안 됩니다.** 현재 A/B 역할은 이미 실행 중인 프로세스가 담당합니다.
같은 시리얼 포트, TF 또는 `/cmd_vel`의 소유자를 추가하지 않습니다.

휠 보정은 맞다고 가정하고, plate x/y/yaw=0 및 기존 LiDAR USB/마운트/마스크/
timing 가정을 유지합니다. 주행은 기존 제한된 구간 콘솔을 사용합니다.
새 콘솔 v2는 주행/정지 확인 중 일반 입력을 버리며, READY에서 잘못된 명령은
안내만 표시합니다. Ctrl+C, 연결 끊김 및 상태 이상 정지는 유지됩니다.

## 다음 기록 — 사용자가 Jetson에서 실행

콘솔 v2를 먼저 별도 디렉터리에 빌드합니다. 기본 실행파일 경로는
`~/jetson-mapping-console-v2/build/run_mapping_console`입니다.
아래 명령의 작업 디렉터리는 이 README와 `scripts/`가 들어 있는 디렉터리입니다.

1. 기존 wheel / 전면 입력 / SLAM 터미널은 그대로 유지합니다.
2. 터미널 C: `./scripts/record_bag.sh`. `runs/bag_<시각>/`가 생성됩니다.
   주요 scan, odom, tf, tf_static, map, joint_states 토픽의 구독 로그를 확인한 뒤
   움직입니다. 기록은 콘솔과 독립적이며 키보드 pause 기능은 꺼져 있습니다.
3. 터미널 D: `./scripts/teleop.sh --supervised`. READY에서 실제 장애물과
   회전 범위를 확인하고 한 구간씩 입력합니다. `q`는 READY에서 종료,
   Ctrl+C는 즉시 중단 요청입니다. 구간 중 다른 입력은 다음 명령으로 예약되지 않습니다.
4. 정지한 상태에서 다른 터미널: `./scripts/save_map.sh`.
   `CHECKPOINT SAVED`를 확인합니다. PGM/YAML과 posegraph/data를 새 디렉터리에
   보관하고 serializer의 result=0 및 파일 존재를 확인합니다.
5. 콘솔은 READY에서 `q`, 터미널 C는 Ctrl+C로 종료합니다. bag의 metadata.yaml과
   모든 db3를 함께 보관합니다. 콘솔의 자체 bag/CSV도 함께 보관합니다.
6. **현재 NJWQJ7 연속 매핑은 계속 유지합니다.** 팀원 원본 절차의 마지막 A 종료는
   이번 연속 매핑 유지 요청에는 적용하지 않습니다. 콘솔 종료는 SLAM 종료가 아닙니다.

독립 recorder는 상태 감시/주행 허가 도구가 아닙니다. 상태와 명령 소유권 검사는
콘솔이 수행합니다. v2는 `/team_mapping_recorder`라는 이름의 수동 구독자 1개를
추가 허용하지만 휠 컨트롤러 구독자 1개는 반드시 필요합니다. 추가 명령 발행자나
예상하지 않은 구독자는 계속 차단합니다. 이 recorder를 기존 v1 콘솔과 함께
실행하면 v1의 구독자 수 검사에서 멈추므로 반드시 v2를 사용합니다.
선택 토픽인 pose_graph/loop_closure_event의 무수신만으로
오류나 loop closure 부재를 단정하지 않습니다. 배터리/여유 공간과 실제 경로 확인은
실행 시점의 조건으로 판단합니다. 원본 continuous key teleop의 속도 제한을 가져오지 않습니다.

## 이번 전달 자료

데이터 압축파일의 `runs/bag_20261027T184444_G0I8Np/`에 완료된 48구간 bag이
있습니다. `sessions/G0I8Np/`에는 원래 result/events/segments/odom CSV가 있습니다.
`earlier_attempts/`는 직전 7회, 총 8개 시작 구간이며 **미완료 기록**입니다.
최종 지도는 이 이전 이동도 포함합니다. G0I8Np 시작 시점에 이미 지도가 있으므로
이 bag 하나를 빈 SLAM에 넣으면 최종 지도가 동일해진다고 주장할 수 없습니다.

`maps/checkpoints/`는 원본 before/after 지도 및 그래프, `maps/graph_resolution/`은
같은 최종 그래프를 추가 최적화 없이 5 cm / 2.5 cm로 렌더링한 결과입니다.
5 cm 픽셀은 저장 지도와 일치합니다. 2.5 cm는 셀 크기만 더 작으며 측정 정확도를
높였다는 뜻이 아닙니다. 현재 live map은 5 cm 그대로입니다.

G0I8Np: 759.32초, scan 7,610개, odom 37,967개, 완료 48구간,
직선 이동 합 5.757 m. 사용자는 모든 구간의 자동 정지와 이전 통로 재방문을
확인했습니다. 전체 공간 커버리지와 loop closure 성공은 아직 확정하지 않았습니다.
이 기록 중에는 물체 변경이나 수동 재배치가 없었지만, 그 이전에는 물체 하나를
옮긴 이력이 있습니다. before 지도와 최종 지도가 완전히 같은 장면이라고 가정하지 않습니다.
map->odom의 최대 변화 0.545 m / 21.696도는 **실제 오차 측정값이 아닙니다**.
외부 기준 거리/각도 또는 ground truth가 있어야 절대 오차를 계산할 수 있습니다.
bag만으로는 입력 연속성, scan 정합, odom 대비 상대 변화 등을 분석할 수 있습니다.

기록된 scan은 원시 패킷이 아니라 `/lidar_front/scan_mapping` 보정 출력입니다.
rear scan, IMU, 전체 구간 raw packet 및 robot_description 토픽은 기존 bag에 없습니다.
실행 중 URDF/파라미터 스냅샷을 `config/runtime_launch/`에 별도로 제공합니다.
`/map_metadata`는 별도 기록하지 않았으며 OccupancyGrid의 info에 해상도/크기/원점이 있습니다.
timing은 nominal, assumed_delay_ns=0, acquisition_time_valid=0의 잠정 가정입니다.
이를 실제로 측정된 latency=0으로 해석하면 안 됩니다.

## 오프라인 열기

압축을 새 디렉터리에 푼 후 `sha256sum --check SHA256SUMS`로 확인합니다.
ROS Lyrical / rosbag2 SQLite metadata v9와 slam_toolbox 2.10.0을 사용했습니다.
구형 ROS 배포판에서 바로 호환된다고 보장하지 않습니다. 데이터 원본은 수정하지 않습니다.

아래는 **로봇을 연결하지 않은 PC에서 저장된 결과를 보는 예시**입니다.
새 SLAM/하드웨어 노드를 함께 실행하지 않습니다. RViz도 같은 환경에서 띄우고
Fixed Frame `map`, LaserScan `/lidar_front/scan_mapping`, Map `/map`을 선택합니다.

```bash
source /opt/ros/lyrical/setup.bash
export ROS_DOMAIN_ID=193
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_STATIC_PEERS=
ros2 bag info runs/bag_20261027T184444_G0I8Np
ros2 bag play runs/bag_20261027T184444_G0I8Np --clock --topics \
  /lidar_front/scan_mapping /lidar_front/mapping_status /odom /tf /tf_static /map /joint_states
```

RViz의 `use_sim_time`을 true로 설정하고 transient-local `/tf_static`은
RViz를 먼저 시작하여 수신합니다. 위 명령은 `/cmd_vel`을 재생하지 않습니다.
새 SLAM으로 재계산하려면 기존 map->odom TF를 제거한 별도 입력과 시작 그래프/
초기화 전략이 필요합니다. 기록된 `/tf`에는 odom->base와 map->odom이 함께 있어,
그대로 새 SLAM과 함께 재생하면 TF 소유권이 충돌합니다.

## URDF로 넘길 방향 결과

[ORIENTATION.md](ORIENTATION.md)와 [frame_contract.yaml](frame_contract.yaml)을
함께 전달합니다. **사용자가 base 프레임 RViz 검사를 아직 하지 않았다고 확인했습니다.**
현재 절대 yaw는 pi이며, 동일한 드라이버 축 규약일 때 NT_SCAN_YAW의 대응 값은
`3.141592653589793`입니다. 아직 실물 검증 완료값으로 고정하지 말아 주세요.
전면 상자가 +X에, 왼쪽 대상이 +Y에 보이는지 확인한 결과를 추가 전달해야 합니다.
현재 TF를 바꾸거나 SLAM을 재시작할 필요 없이 정지 상태에서 검사할 수 있습니다.
