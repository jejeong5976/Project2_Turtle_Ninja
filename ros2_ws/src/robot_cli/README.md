# robot_cli — 로봇 상태 화면 + 명령 (C++)

`nt` 한 단어로 터미널에 로봇 상태를 띄우고, 화면 입력줄에서 팔 · 주행 명령을 보낸다. `nt <명령>` 은 한 줄 명령 (스크립트용).
참고한 도구: Lv2_bee_Assignment `simulator/bee.py` · `manual_window.py`. ROS 2 (Lyrical) 에 바로 붙고 새 인터페이스는 만들지 않는다.

## 켜기

```bash
cd ros2_ws && colcon build --packages-select robot_cli
tools/robot_cli/install.sh        # 한 번만: ~/.local/bin/nt → tools/robot_cli/robot_cli.sh (sudo 없음, 젯슨 · PC 같음)
nt                                # 상태 화면 (? = 도움말, q = 끝)
nt status · nt status 2           # 상태를 한 번 글로 (쪽 번호, --watch = 1 s 마다)
nt check                          # 시작 전 점검 (실패 있으면 끝 코드 1)
nt h · nt s5 · nt g B1            # 짧은 명령 (아래 표)
```

install.sh 는 같은 이름 `nt` 가 이미 있으면 알려 주고 덮어쓸지 묻는다. `~/.local/bin` 이 PATH 에 없으면 넣는 줄만 알려 준다.
실행기는 심볼릭 링크로 불려도 레포를 찾는다 (`readlink -f`). `ROS_DOMAIN_ID` 는 안 정했으면 팀 로봇 17.
`FASTDDS_BUILTIN_TRANSPORTS` 는 안 정했으면 `LARGE_DATA` (bringup.launch.py 카메라 · 인지와 같음, 큰 영상은 TCP). 같은 기계 안에서는
공유 메모리라 달라도 보이지만, PC 에서 Wi-Fi 로 볼 때는 로봇 쪽 노드와 같은 값이어야 보인다 (화면 맨 위에 지금 값). `''` 로 export 하면 기본 UDP.
가짜 로봇으로 해 볼 때: `ROS_DOMAIN_ID=96 ros2 launch bringup robot.launch.py hardware:=mock` → `ROS_DOMAIN_ID=96 nt`

## 짧은 명령

| 짧게 | 같은 긴 명령 | | 짧게 | 같은 긴 명령 |
|---|---|---|---|---|
| `h` | `home` | | `g B1` | `go B1` — 접고 이동 (아래) |
| `l` | `look` | | `x` | `cancel all` |
| `scan` | `look rack` — 빈 손 적재함 스캔 (칸 태그 재기) | | | |
| `b` | `barcode` | | `!` | `estop` (화면 어디서나 F9) |
| `s5` · `s 5` | `store S5` (S 생략 가능) | | `c` | `check` |
| `r5` | `retrieve S5` | | `m 이름` | `mark 이름` |
| `p x y` | `pick x y` | | `n 메모` | `note 메모` |
| `p det` · `p tag 3` | 손목 인지가 본 퍽 집기 (퍽이 하나뿐일 때 · 여러 개면 퍽 태그) | | | |
| `pl x y` | `place x y` | | `?` · `q` | `help` · `quit` |

화면 입력줄: `Tab` = 명령 · 칸 · 정차 이름 자동 완성 (후보가 여러 개면 기록 칸에), `↑ ↓` = 앞 명령.

**접고 이동** `g B1`: 팔이 받침에 접혀 있고 차례가 DRIVE 면 바로 nav. 아니면 (lazy_fold 로 P0 등) `home` 을 먼저 보내고 성공하면 nav.
화면 제목에 `[접고 이동]`. 퍽을 쥐었으면 (그리퍼 반쯤 닫힘 · 기준점 자세) · 팔 작업 중 · E-STOP 이면 거절. `nav B1` 은 접기 없이 바로.
접고 이동이 HOME 을 기다리는 동안 다른 팔 · 주행 명령은 거절 (HOME 뒤 옛 목적지가 다시 나가지 않게) — 바꾸려면 `x` 로 멈춘 뒤.

## 화면

맨 위 메뉴 `0 전체 요약 · 1 위치·SLAM · 2 팔 · 3 주행·Nav2 · 4 인지 · 5 미션 · 6 진단 · 7 실험`.

- **0 쪽에서 번호 + Enter** = 그 쪽. `Esc` 또는 `0` = 전체 요약, `←/→` = 옆 쪽, `?` = 도움말 쪽, `c` = 점검 쪽.
- **2 · 3 · 5 · 6 · 7 쪽에서 번호** = 그 쪽 동작 (쪽 맨 아래에 목록). 필요한 것만 짧게 묻는다 (`3 넣기` → "넣을 칸?" → `5`). 묻는 중 `Esc` = 그만.
  2 팔: 1 HOME · 2 LOOK · 3 넣기 · 4 빼기 · 5 멈춤 / 3 주행: 정차 번호 = 그리로 접고 이동, 마지막 번호 = 멈춤 /
  5 미션: 1 시작 (y 확인) · 2 이어서 · 3 그리퍼 비움 표시 (y 확인) · 4 신호 다시 / 6 진단: 1 점검 / 7 실험: mark · cycle · 메모 · bag · 점검.
  1 · 4 쪽은 동작이 없어 번호 = 그 쪽.
- 명령 입력줄 · 기록 칸 (6 줄, 명령 결과 위주 — 보냄 · phase 줄은 숨김) 은 모든 쪽에서 같다.

**0 전체 요약** = 덩어리마다 한 줄 + 배지 (자세한 숫자는 1~7 쪽):

```
바퀴      [잠금]      팔 작업 뒤 (P0) — 주행 전 home 필요
팔        [P0]        받침 바로 위 (lazy_fold 끝 자리) · 마지막 STORE S12 성공 19.4 s · 그리퍼 15 mm
위치      [꺼짐]      SLAM · AMCL 안 켜짐 (odom 만)
Nav2      [꺼짐]      nav 명령 안 됨
인지      [꺼짐]      신호등 · 손목 · 광각 없음
미션      [꺼짐]      mission_manager 안 켜짐
하드웨어  [정상]      보드 2 · 컨트롤러 4 · 라이다 없음
배터리    [모름]      OpenCR 로그에 전압 없음 (mock · 켜는 중)
```

배지 색: 초록 = 정상 (주행 가능 · 받침 · 작업 중 …), 노랑 = 주의 (잠금 · 주의), 빨강 = 문제 (문제 · E-STOP), 회색 = 꺼짐 · 없음 · 모름.
관절이 한계에 가까우면 `관절 [주의]` 줄이, 수동 주행 · E-STOP · 시계 문제가 있으면 `수동` 줄이 생긴다.

| 쪽 | 보이는 것 |
|---|---|
| 1 위치·SLAM | odom · map (TF, 나이) · amcl 위치 표, 위치 추정 종류 · 건강 · 노드 · lifecycle, 라이다 앞 · 뒤 · 합친 scan · IMU 나이, 시계 차이, 다시 띄울 launch 명령 |
| 2 팔 | 관절 표: 눈금 · rad · ° · 전류 A · 보정 rad · ° · 눈금 · 처짐 · 받침과 차이, 그리퍼 간격 · 전류 · 퍽 쥠, 이름 자세 · 접힘 · lazy_fold · URDF 한계, 차례, 최근 팔 작업 8 개 (걸린 시간 · phase 마다) |
| 3 주행·Nav2 | Nav2 goal · 남은 거리 · ETA, hold, drive 가 되는지 (안 되면 이유), 정차 11 곳 (번호 · 좌표 · 거리 · 방향 차이) |
| 4 인지 | 신호등, 손목 · 광각 검출 하나하나 (태그 · 위치), 적재함 칸 태그 · 칸 안 퍽 (칸마다 마지막으로 본 위치 · 언제), 바코드 |
| 5 미션 | 상태 · 퍽 수 · 고장, 적재 칸 표 |
| 6 진단 | 노드, 보드 · 컨트롤러 표, 센서 나이, `/diagnostics` 전부, 배터리 |
| 7 실험 | mark · cycle · 세션 기록 · 메모 · bag (아래 '실험') |

팔 자세 이름 · 접힘 기준은 control_master 와 같은 설정 (bringup share `config/control.yaml` 의 `poses.home` · `fold_tolerance` · `lazy_fold` ·
`gripper_open/close`, `config/arm_paths.yaml` 의 `fold` 경로 첫 점 = P0 · `staging_pose` = 기준점 · `look` 끝 점). 이름 자세 = 관절 5 개 차이가 모두 0.15 rad 안.
접힘 판정은 이 CLI 계산이고, 바퀴를 푸는 판단은 control_master 가 실측으로 따로 한다 (차례 DRIVE).
**lazy_fold**: STORE · PLACE 는 P0 (받침 바로 위) 에서 끝나고 바퀴 잠금 유지 → 다음 팔 작업은 바로, 주행 전 `home` (fold 경로). `g` 는 이걸 알아서 함.

경고 출처:

- **모터 전류**: `/joint_states` effort 칸 = 전류 [A] (OpenCR, mock 은 0). 그리퍼 반쯤 닫힘 (control_master 와 같은 식) = 퍽 쥠.
- **배터리**: OpenCR 하드웨어가 전압을 토픽이 아니라 로그로만 냄 → `/rosout` 의 "… 배터리 12.3 V (최저 12.1)" (10 s 마다) · "배터리 낮음" 줄을 읽음.
  `hardware_map.yaml` 의 `battery_warn_v` (10.8) 아래 주의, `battery_stop_v` (10.2) 아래 문제. 로그가 없으면 (mock) 모름.
- **관절 한계**: `/robot_description` (URDF) 의 limit. 한계까지 0.03 rad (1.7°) 안이면 주의, 넘으면 문제. 예 joint4 위쪽 한계 0.6735 rad ≈ 눈금 2487 (실물 멈춤 약 2485).
- **TF 위치 나이**: map → base_footprint 변환 시각이 1 s 넘게 오래되면 "지도 위치가 … 전 것 (TF 끊김?)".

## 점검 `check`

`nt check` (끝 코드 0 통과 · 1 실패) 또는 화면 `c` (점검 쪽, 계속 다시 봄). 항목: ROS_DOMAIN_ID, 필수 노드 (controller_manager · control_master ·
robot_state_publisher · arm_state_publisher), 보드 2 개 active, 컨트롤러 4 개 active, 관절 값 신선 (0.5 s), 받침 접힘, 라이다 앞 `/lidar_front/scan` ·
뒤 `/lidar_rear/scan`, IMU `/imu_sensor_broadcaster/imu`, 배터리, 로봇 ↔ PC 시계 차이, 차례. 주의 (노랑) 는 통과로 친다. mock 은 라이다 · IMU 가 없어 실패.

## 명령 (긴 이름)

```
팔   home [force] · look [rack] · barcode · store <칸> · retrieve <칸>   (칸 = S5 또는 5, look rack = 빈 손 적재함 스캔)
     pick x y [z] [yaw°] · place x y [z] [yaw°]   (base_link m, z 0 = 박스 위 기본 높이) · arm <TASK> …
     pick det · pick tag <ID>   (LOOK 자세에서 손목 인지 검출로: planning 과 같은 집기 점 · 방향, 2 s 넘은 검출은 안 씀)
주행 go <정차|x y [yaw°]> (접고 이동) · nav <정차|x y [yaw°]> (바로) · stops
     drive vx vy wz [초] [force] · stop · teleop [force] (화면만: w/s a/d q/e · +/- · space · Esc)
멈춤 cancel [arm|nav|all] · estop [on|off]
점검 check
실험 mark <이름> · markdir <폴더> · cycle <칸> <횟수> [store] · cycle stop · note <메모> · bag start|stop · teach
화면 status [쪽] · snap [파일] · help · quit
기타 call <서비스> [on|off] · mission start|resume|clear_gripper|reset_signal · loc
```

스크립트 예:

```bash
nt h && nt s12 --timeout 60
nt g B1 --no-wait
nt drive 0.05 0 0 2
nt check || echo "점검 실패"
```

끝 코드 (bee.py 와 같음): 0 성공 · 1 응답 없음 · 시간 초과 · Ctrl-C · 2 거절 · 실패 · 문법 오류.
`--no-wait` · `--timeout` 초과로 끝낸 goal 은 그대로 진행 → 멈추려면 `nt x`.

## 실험 (7 쪽)

기록 폴더 기본 `~/robot_cli_marks` (레포 안에 쓰지 않게). 바꾸기: 화면 `markdir <폴더>`, 한 줄 명령 `--mark-dir <폴더>`. 모두 덧붙여 쓴다.

| 파일 | 내용 |
|---|---|
| `poses.csv` | `mark` 하나: 이름 · 시각 · 측정 나이 · 차례 · 가까운 자세 · 접힘, 관절마다 `_tick _rad _deg` (`/joint_states`) · `_corr_rad _corr_deg` (`/joint_states_corrected`), 그리퍼, odom · map 위치, 손목 검출 수 · 태그 |
| `poses.yaml` | 같은 내용 + 손목 검출 전부 (`/perception/wrist/detections`: 태그 · 종류 · 위치 · 방향 · 카메라 프레임) |
| `detections.csv` | 검출 하나 + 그때 관절 rad (엔코더 · 보정) — 적재함 칸 바닥 AprilTag 20~31 로 실제 손목 카메라 자세를 잴 때 |
| `cycle_<칸>.csv` | `cycle` 작업 하나: 회 · 작업 · 시작 시각 · 걸린 s · 결과 · phase 마다 s · control_master 글 |
| `notes.csv` | `note` 하나: 시각 · 세션 파일 · 메모 |
| `sessions/session_NNN.log` | 화면을 켤 때마다 한 파일 (이름은 O_EXCL 로 차지 — 화면 두 개가 같은 파일을 안 씀): 기록 칸에 나온 모든 줄 |
| `bags/bag_NNN/` | `bag start` ~ `bag stop`: `ros2 bag record` (joint_states · joint_states_corrected · cmd_vel · odom · scan · 라이다 앞뒤 · tf · tf_static · arm_task 상태 · feedback · motion_mode · ack · diagnostics) |

- **mark** — 팔은 움직이지 않는다 (읽기만). 팔 관절 (joint1~5) 측정이 1 s 넘게 끊겼으면 기록하지 않는다. 처짐 보정 자료: 태그가 보이는 자세에서 mark.
  보정 값은 `/joint_states_corrected` 도 1 s 안일 때만 적고 아니면 빈 칸 · null (나이는 `corr_age_s` · `corrected_age_s`).
- **cycle** `cycle 5 3` — ArmTask 만 보낸다 (팔 순서 · 바퀴 잠금은 control_master, arm_controller 에 궤적을 직접 보내지 않음). RETRIEVE → STORE 를 횟수만큼,
  `store` 를 붙이면 STORE → RETRIEVE. 실패 · 거절이면 멈춤, `cycle stop` = 지금 작업 뒤 멈춤, `cancel` · estop · F9 · quit = 바로.
- **bag** — `ros2 bag record` 를 자기 프로세스 그룹으로 띄움. `bag stop` · quit · Ctrl-C 때 SIGINT 로 정상 종료 (10 s 안 끝나면 SIGTERM).
  `bag stop` 은 기다리지 않음 — 끝나는 동안에도 F9 · 명령이 됨. 혼자 끝나면 화면에 알림 (`bag_NNN_record.log`).
- **snap** — `snap` 만 치면 기록 폴더의 `snap_NNN.txt` (레포 안에 안 씀), `snap 파일.ansi` 면 색까지.

## 손으로 가르치기 (토크 끄기) — 없음, 읽기 전용만

팔을 손으로 움직여 값을 읽는 것 (토크 끄기) 은 **지금 인터페이스로는 안전하게 할 수 없어서 넣지 않았다.** 이 CLI 는 값 보기 (2 · 7 쪽) 와 `mark` 만 한다.
(`teach` 명령이 같은 안내를 보여 줌.)

확인한 것 (레포 코드):

1. **토크 끄기는 launch 고정값.** `description/urdf/ros2_control.xacro` 가 `torque_off_on_deactivate` 를 `false` 로 박아 둠 (launch 인자 아님).
   그래서 `bus_arm` 을 inactive 로 해도 토크는 그대로다 (팔이 안 떨어지게 한 팀 결정). 토크가 꺼지는 길은 cleanup → configure 의 CONFIG 악수뿐인데,
   이건 고장 복구 절차라 팔 6 축이 한 번에 떨어지고 포트를 다시 연다 (최대 8 s). 실행 중에 "지금만 끄기" 를 부를 인터페이스가 없다.
2. **다시 켤 때 튈 수 있음.** OpenCR 은 켤 때 "지금 위치 = 목표" 로 켜지만 (on_activate), arm · gripper JTC 는
   `set_last_command_interface_value_as_state_on_activation` 이 기본 true (controllers.yaml 에서 안 바꿈) 이고 하드웨어가 명령 칸을 비우지 않아
   (opencr on_activate 는 자기 targets_ 만 비움) 가르치기 **전** 명령 자세를 붙잡을 수 있다 → 사람이 잡고 있는 팔이 옛 자세로 확 움직임.
   "명령 = 지금 위치" 로 맞추려면 arm_controller 에 궤적을 보내야 하는데, 그건 control_master 의 바퀴 · 팔 차례를 건너뛰어서 하지 않는다.
   mock 하드웨어는 OpenCR 과 달라서 이 부분을 시험으로 확인할 수도 없다 (실물 시험은 이 작업 범위 밖).
3. **복구가 HOME 에 걸림.** control_master 는 보드가 active 가 아니면 하드웨어 이상 → STOP (바퀴 잠금 — 이건 좋음). 그런데 다시 주행하려면 HOME 이
   필요하고, 손으로 둔 자세에서는 검사된 복귀 경로가 없다. `home` 이 거절되는 것은 퍽을 쥐었거나 (그리퍼 반쯤 닫힘) 경로 작업이 도중에 멈춘 뒤뿐이고,
   그 밖에는 control_master 가 검사 안 된 직선으로 home 한 점에 곧장 간다 (적재함 · 박스 안이면 쓸고 나옴). 손으로 둔 뒤에는 주변을 먼저 보고,
   팔을 받침 근처 (또는 검사된 경로 출발 자세: P0 · LOOK) 로 옮긴 뒤 `home`.

필요한 지원 (만든다면 control_master · hardware 쪽에):

- **hardware**: 보드 · 관절마다 토크를 실행 중에 켜고 끄는 명령 칸 (ros2_control GPIO command interface, 예 `bus_arm/torque_enable`) — 생명 주기 전환 없이.
  켤 때는 지금처럼 펌웨어가 지금 위치를 목표로 잡음. 할 수 있으면 관절별 (joint1 · 그리퍼는 켜 둔 채 등), 전류 제한 순응 모드면 더 좋음.
- **control_master**: `teach` 작업 (ArmTask 새 번호나 서비스) — 차례가 바퀴 잠금 (ack 받은 STOP) · 몸통 정지일 때만 들어가고, 들어가면 arm · gripper JTC 를 끄고
  토크를 끔. 끝낼 때는 팔이 받침 근처인지 실측으로 확인한 뒤 토크를 켜고, JTC 명령 칸을 실측값으로 채운 뒤 (또는 controllers.yaml 에서
  `set_last_command_interface_value_as_state_on_activation: false`) JTC 를 켬 → 접힘 실측 확인을 다시 거쳐야 DRIVE.
- **CLI**: 그때는 이 CLI 가 그 작업을 부르고, 입력으로 "사람이 팔을 잡았음" 확인 글을 받게 하면 됨 (바퀴 잠금 · 확인 글은 control_master 도 한 번 더).

## 지켜지는 규칙

- **drive · teleop** 은 `/cmd_vel` 에 바로 보냄 (swerve_controller, 20 Hz, 끝나면 0). 한계 = `controllers.yaml` swerve_controller `max_linear` · `max_angular`.
  시작할 때 그리고 보내는 틱마다 허용 조건을 다시 본다: `/odom` 이 1 s 안에 옴 (시계 차이를 재려고), 시계 차이 0.25 s 안
  (swerve_controller 는 stamp 가 0.3 s 넘게 다른 `/cmd_vel` 을 버림), 노드 목록에 로봇이 보임 · Nav2 (`/bt_navigator`) 없음
  (`nav2.launch.py` 규칙 "Nav2 중 teleop 금지"), 차례 DRIVE (0.5 s 안). 잃으면 바로 멈추고 0 ("수동 주행 멈춤 (바퀴 0): 이유").
  `force` 는 노드 목록 · Nav2 · 차례만 넘김 (`arm:=false` 처럼 차례가 없을 때), odom · 시계 · E-STOP 은 못 넘김.
  속도는 어느 경로로 와도 설정 한계 안으로 줄임 (teleop 시작 속도 · +/- 도). 한계 값이 이상하면 (음수 · NaN · 1 m/s · 3 rad/s 넘음) 기본 0.20 · 0.5 + 경고. 한 줄 `drive` 끝 코드: 0 시간 다 됨 · 1 Ctrl-C · 2 조건을 잃어 멈춤.
- **estop** = 소프트웨어 정지 (F9 · `!`): 팔 · 주행 서버의 모든 goal 취소 + `/cmd_vel` 0. 화면에서는 `estop off` 까지 잠금. 속도 고르기 · 보내기와 같은 락이라
  estop 의 0 뒤에 옛 속도가 나가지 않는다. E-STOP 중에는 `call` · `mission` 서비스도 거절 (mission start · resume 이 새 동작을 시작할 수 있어서).
  **모터 전원 차단이 아니다.** 취소 뒤 control_master 는 STOP 잠금 → `home` (경로 중간이면 사람이 보고 `home force`).
- **cancel** 은 보낸 쪽과 상관없이 서버의 모든 goal 을 취소 (미션 goal 포함). 그 전에 이 CLI 가 보냈는데 받음 답이 아직 없던 goal 은 받아지는 대로 바로 취소.
  접고 이동이 HOME 단계일 때 멈춤 (3 쪽 마지막 번호 · `cancel nav`) 은 그 HOME 도 취소.
- **보드 · 컨트롤러 상태** 는 controller_manager 에 1 Hz 로 묻는다. 답이 3 s 넘게 끊기면 옛 active 목록을 믿지 않음 (요약 [주의], 점검 실패).
- **Ctrl-C** (한 줄 명령 · 화면): 보내기 전 (발견 · 서버 · 서비스를 기다리는 중) 이면 아무것도 안 보냄 (`call` · `mission start` 도). 보낸 뒤면 이 CLI 의 goal 취소,
  받음 답이 1 s 안에 안 오면 그 서버의 모든 goal 취소. `--no-wait` 에서 받음과 Ctrl-C 가 겹쳐도 취소가 먼저. 화면 quit 도 같음 + bag 정상 종료.
- 팔 작업이 거절되면 이유는 control_master 로그에 있다 (ArmTask 거절에는 이유 필드가 없음).
- **SLAM ↔ AMCL · 조향 정책** 을 실행 중에 바꾸는 인터페이스는 없다 → 1 쪽 · `loc` 이 다시 띄울 launch 명령을 보여 줌.
- Nav2 가 `mission_hold:=true` (기본) 로 떠 있으면 `/mission/hold` 가 true 인 동안 주행이 막힌다 (Nav2 줄 [주의]).
- 화면 숫자는 지역 설정과 상관없이 소수점 `.` (LC_NUMERIC = C) — 기록 파일이 쉼표 소수점으로 깨지지 않게.

## 파일

- `src/model.cpp` 설정 읽기 · 눈금 변환 · 팔 자세 · 접힘 · 배터리 로그 읽기 (dice_map 변환은 mission `dice_map.cpp` 와 같은 식) · `src/command.cpp` 명령 문법 · 짧은 별칭 · Tab 후보
- `src/status_view.cpp` 0 쪽 배지 줄 · 공용 표 · 점검 · 쪽 동작 · `src/pages.cpp` 1~7 쪽 · `src/teach.cpp` mark · cycle · `src/bag.cpp` bag 기록
- `src/robot_link.cpp` ROS 연결 · `src/tui.cpp` ncurses 화면 · 메뉴 · `src/main.cpp` 한 줄 명령
- 설정: `hardware_map.yaml` · `rack_cells.yaml` · `control.yaml` · `arm_paths.yaml` · `controllers.yaml` = bringup share/config, `dice_map.yaml` = 레포 `maps/`
  (빌드 때 경로 `ros2_ws/src/robot_cli/../../../maps`, 레포 밖에서 빌드하면 `--cmake-args -DROBOT_CLI_MAPS_DIR=<레포>/maps`, 실행 때 `--dice-map`)
- `tools/robot_cli/robot_cli.sh` 실행기 · `tools/robot_cli/install.sh` `nt` 설치
- 시험: `colcon test --packages-select robot_cli` (명령 · 별칭 · Tab · 눈금 · 정차 · 팔 자세 · 배터리 로그 · 주행 한계 · 배지 · 쪽 · 점검 · 한계 · mark 파일 · cycle 순서)
