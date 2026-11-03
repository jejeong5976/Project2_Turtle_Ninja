# 팔 경로 · 기구학 생성 도구 (실물 시험 10-28 ~ 11-01)

로봇에서 도는 코드는 전부 C++ `control_master` 입니다. 이 폴더는 그 C++ 이 읽는 설정을 **만드는** 파이썬 도구입니다.
- `config/arm_paths.yaml` — STORE · RETRIEVE 12 칸, LOOK · LOOK_BARCODE, 접기 · 펴기 (11-01 실물 성공 궤적 재생)
- `config/arm_kinematics.yaml` · `ros2_ws/src/manipulation/test/arm_kin_data.hpp` — C++ IK 의 기구학 상수 · 시험 정답값

실물 시험 때 쓴 코드 가운데 이 두 가지를 다시 만드는 데 필요한 것만 넣었습니다 (실물 실행기 · 키보드 맞추기 · 칸 스캔 · 손눈 보정 풀이 같은 1 회용 실험 도구는 뺌, 원본은 실험 PC `arm_tests/`).
바꾼 것은 개인 PC 절대 경로 → 환경변수뿐입니다.

**확인 (11-03)**: 이 폴더로 `export_yaml.py` 를 돌리면 지금 `config/arm_paths.yaml` 과 같은 파일, `replay_paths.py store S12` 는 실물 기록에서 같은 252 점 경로.

## 파일
| 파일 | 하는 일 |
|---|---|
| `arm/replay_paths.py` | 칸마다 실물 성공 실행 기록(joint_states.csv) → 16 틱 간격 · 처짐 앞먹임 · 안전 다시 검사 → `out/<경로>.checked.json`. '그때 찬 칸' 은 `rack_cells.json.bak_state_1101_*` 로 되살림 |
| `arm/retrieve_tail.py` | 빼기 경로 끝(실물 들기) → 박스 위 기준점 (PICK 끝과 같은 자세) |
| `arm/fullcheck.py` | 최악 검사: 대상 칸 말고 11 칸이 모두 찬 상태로 다시 검사 (경고만) |
| `arm/gen_look.py` | LOOK · LOOK_BARCODE 와 그 복귀 경로 |
| `arm/gen_paths.py` · `rack_load.py` · `unload_cell.py` · `unload_l78.py` | 계획기 경로 (재생 경로의 기준 · 칸 이름 짝 · L7/L8 사람 시연 재생, `teach_20261101T162442/`) |
| `arm/map_runs.py` | 실물 실행 기록 → 칸 짝짓기 |
| `arm/export_yaml.py` | `out/*.checked.json` → `config/arm_paths.yaml` (칸 근처 15 °/s 표시 포함) |
| `arm/export_kin.py` · `export_kin_cpp.py` | 기구학 상수 · 정답값 → `config/arm_kinematics.yaml` · `arm_kin_data.hpp` |
| `arm/pose_safety.py` | 자세 · 경로 안전 검사 (URDF 메시 충돌, 관절 범위, 쥔 퍽 · 칸 안 퍽, 처짐 ±2° 가설) |
| `arm/sag_model.py` · `arm_pose_fk.py` · `grasp_test.py` · `arm_map.py` | 처짐 모델 · 순기구학 · 집기 계산 · 하드웨어 지도 (검사기 · 생성기가 씀) |
| `arm/handeye_floor.py` · `camera_angle_ref.py` | 손목 카메라 장착 (`pose_safety.py --calib · --kin` 때만) |
| `reach/reach_model.py` · `reach_lib.py` | URDF 메시 충돌 · IK 모델 |
| `place_spots/place_ik.py` · `place_poses.json` | 놓기 자세 계산 (`pose_safety.py --calib · --kin` 때만, json 은 참고 자료) |
| `arm/rack_cells.json` (+ `.bak_state_1101_*` 12 개) | 칸 측정값 (옛 이름 A · B · C · D · L1~L8) · 11-01 칸 상태 기록 |
| `arm/camera_calib_20261030.json` · `ros/hardware_map_arm_only.yaml` | 손목 카메라 보정 · 팔 관절 영점 (생성기가 읽음) |
| `arm/grasp_poses*.json` | 실물 집기 기준 자세 (참고 자료, `grasp_test.py` 를 인자 없이 돌리면 덮어씀) |

## 다시 만들기
```bash
cd tools/arm_real/arm
rm -rf out/                      # 빈 out/ 에서 시작 (retrieve_tail 이 out/*.pre_tail.json 이 있으면 그걸 다시 씀)
export NT_PY=python3 NT_URDF_RELEASE=<10-27 배포판 폴더> NT_RUNS_DIR=<실물 runs 폴더>
$NT_PY gen_paths.py all          # 계획기 경로 (out/). 다른 모드: store · retrieve · store_real (map_runs 결과) · l78
$NT_PY replay_paths.py store     # 실물 성공 궤적 재생 (out/*.checked.json, out/replay_summary_store.json)
$NT_PY replay_paths.py retrieve  # (retrieve_tail · export_yaml 이 replay_summary_store/retrieve.json 을 읽음 — 'all' 로 한 번에 돌리면 요약이 _all 로 써져 안 맞음)
$NT_PY retrieve_tail.py
$NT_PY fullcheck.py              # 나머지 11 칸 다 참 (경고만)
$NT_PY gen_look.py
$NT_PY export_yaml.py ../../../config/arm_paths.yaml
$NT_PY export_kin.py && $NT_PY export_kin_cpp.py ../../..   # 기구학 → config/arm_kinematics.yaml · test/arm_kin_data.hpp
```
`export_yaml.py` 는 `out/<이름>_rot.checked.json` 이 있으면 손목 회전 후보를 넣음 — **만들지 말 것** (J5 만 돌린 후보는 그리퍼 중심이 J5 축에서 약 12 mm 떨어져 있어 퍽이 13~17 mm 옮겨짐, 11-03 시뮬 · FK 로 확인해 뺐음).

## 필요한 자료 (저장소에 없음)
| 환경변수 | 기본값 (이 폴더 기준) | 내용 |
|---|---|---|
| `NT_URDF_RELEASE` | `nt_urdf_release_20261027_layout/` | 10-27 팀 배포판 폴더 통째로 (335 MB: urdf · 메시 · config/joint_map.json · config/nt_train_kit.py · config/arm_replays). 충돌 검사 · 기구학 기준 |
| `NT_RUNS_DIR` | `arm/runs/` | 실물 실행 기록 (`runner_<시각>/`, 483 MB). 재생 경로는 `config/arm_paths.yaml` 주석의 runner 42 개 |
| `NT_POSE_MEASURE` | `arm_workspace_measure_20261024/` | 손으로 읽은 팔 자세 `runs/POSE_*` (11-01 L7 · L8 시연). `gen_paths.py l78` · `unload_l78.py` 만 씀 — 위 명령에는 필요 없음 |
| `NT_PY` | `python3` | numpy scipy trimesh shapely opencv-python open3d matplotlib Pillow |
| `NT_ARM_DIR` · `NT_REACH_DIR` · `NT_PLACE_SPOTS` | 이 폴더의 `arm/` · `reach/` · `place_spots/` | 보통 안 바꿈 (`NT_ROBOT_SIM` 은 `place_ik.py` 만) |

그 밖의 조절값: `NT_SAFETY_THREADS` (검사 스레드 수), `NT_POSE_CACHE` (검사 캐시 `arm/.pose_cache/`), `NT_PLACED_PUCK_MARGIN` (칸 안 퍽 여유, 기본 2 mm).
**설정하지 말 것** (결과가 말없이 바뀜): `NT_ARM_PONLY` (처짐 앞먹임 계산을 바꿔 `arm_paths.yaml` 관절값 전부가 달라짐), `NT_RACK_HOVER_STOP` (칸 위 멈춤 표시).

`gen_paths.py` 는 도는 동안 `arm/rack_cells.json` 을 잠깐 바꿨다 되돌림 — 도중에 끊겼으면 `git diff arm/rack_cells.json` 확인.
모두 계산만 합니다 (포트 · 모터 안 씀). 처짐 모델은 칸 근처(팔이 뒤로 젖힌 자세)에서 실물과 1~2 cm 차이가 날 수 있어, 새 자세를 계산했으면 실물로 확인할 것.
