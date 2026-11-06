# transfer_planner 설계 (A안: 돌면서 다 싣기)

- 작성 2026-10-30, 대상 발제 v5 '주사위 기억 로봇'
- 상태 (2026-10-30): `transfer_types.hpp` · `transfer_solver` · `transfer_planner` 구현, gtest 13개 통과 (`test/test_transfer.cpp`). 기존 `dice.hpp` 위에 얹음
- 상태 (2026-11-01): **노드 연결 (9장)** — `dice_map` (경기장 좌표 `maps/dice_map.yaml`) · `dice_steps` (단계 만들기) 추가, `mission_manager` COLLECT · DELIVER · VERIFY, ArmTask STORE 3 · RETRIEVE 4 · LOOK 5 + `slot`. gtest `test_transfer` 18개. **재계획 (8장) 은 아직 없음** → dice 단계 실패면 복귀
- 상태 (2026-11-01 저녁): 재계획 구현 (9장 '실패 · 재계획'). **planning / mission 패키지 분리**: 이 문서의 계산 파일 · 노드는 `mission` 패키지로 옮김 (`ros2_ws/src/mission/`), 노드 파일 `planning_node.cpp` → `mission_node.cpp` → `planning_master.cpp` → `mission_manager.cpp` (2026-11-02), 단계 실행은 `planning` 패키지 `Executor`. 옛 move 흐름 (`task_planner` 의 Find · Destination) 과 `light_gate` 는 삭제. (본문 표기도 `mission_manager.cpp` 로 바꿈)
- 비용표: `maps/dice_travel_times.yaml` (Isaac 측정 이동 시간 + 정차 · 팔 가정값). Q1 예상 1659 s (LOOK 5 s 포함)

## 1. 전략 요약

| 단계 | 내용 |
|---|---|
| 수집 | 고정 순서 S → B1 → B2 → B6 → B3 → B4 → B5. 박스마다 정차 → 손목 LOOK → 퍽 2개 집기 (그리퍼에 있던 퍽은 먼저 적재함에 넣음) |
| 계획 | 마지막 박스(B5) LOOK 직후 12개 정보가 다 모임 → 할당 · 장소 순서 · B5 집는 순서를 한 번에 계산 |
| 배달 | 장소 3곳을 한 번씩 방문. 장소마다 태그 LOOK → 먼 칸부터 꺼내기 · 놓기 |
| 검증 · 복귀 | 세 장소 재검증 순회 → 출발 구역 |

- 근거 (Isaac 측정 이동 시간, 문제 100개): 현재안(1개씩) 2119 s → A안 1604 s. 정차 32 → 12회. 이상적 하한(처음부터 다 앎) 1530 s
- 넣기+빼기 실측이 퍽당 약 45 s 를 넘으면 C안 (B3 · B6 은 배달 때 집기) 로 전환 → 설정 `defer_boxes` 하나로 바꿈 (6장)

## 2. 파일 구성

| 파일 | 하는 일 | ROS |
|---|---|---|
| `transfer_types.hpp` | 노드 번호 · 비용표 · 정차(Stop) · 팔 동작 자료형 | 없음 |
| `transfer_solver.hpp/.cpp` | 현재 실제 상태 → 남은 일 전체의 최소 시간 동작 순서 (정확해, Dijkstra) | 없음 |
| `transfer_planner.hpp/.cpp` | 수집 정차 목록 생성, 계획 시점 호출, 해 → 정차 목록 → `Step` 목록 변환, 실패 뒤 재계획 | 없음 |
| `mission_manager.cpp` (옛 `planning_node.cpp` · `mission_node.cpp`) | Phase 를 COLLECT · DELIVER · VERIFY 로 바꾸고 위 함수 호출 | 있음 |

- 정확해 하나로 A안 · C안 · 실패 복구를 모두 처리함 (A안은 "수집 순서가 고정된 특수한 경우")
- `task_planner` 의 move 흐름, `light_gate` 는 2026-11-01 패키지 분리 때 삭제. `task_parser` 는 dice 바코드 해석만 남음

## 3. 자료형 (`transfer_types.hpp`)

```cpp
// 노드: 0 = 출발점, 1–6 = 시작 박스 정차점, 7–9 = L1–L3 정차점
enum class NodeKind : uint8_t { Start, Box, Place };
struct Node { NodeKind kind; int index; Pose2D stop_pose; };

// 비용 [s]. 이동은 노드 쌍 시간표 (Isaac 측정 또는 Nav2 경로 길이 / 속도)
struct TransferCosts
{
  std::array<std::array<double, 10>, 10> travel_s;   // travel_s[a][b], 대각 0
  double stop_s = 20.0;       // 정차 1회 (감속 · 도킹 · 대각선 고정 · 해제)
  double look_s = 5.0;        // 손목 LOOK 1회
  double pick_s = 30.0;
  double place_s = 30.0;
  double store_s = 15.0;      // 그리퍼 → 칸
  double retrieve_s = 15.0;   // 칸 → 그리퍼
};

// 팔 동작 1개 (한 정차 안에서 연속 실행, 끝에만 HOME)
enum class ArmOp : uint8_t { Look, Pick, Store, Retrieve, Place };
struct ArmAction
{
  ArmOp op;
  std::string puck_id;        // Pick · Store · Retrieve · Place
  std::string slot;           // Store · Retrieve (적재함 칸 이름)
  int place = -1;             // Place: 0–2 (L1–L3)
  int cell = -1;              // Place: 그 장소 눈 목록 번호 (dice pip_offsets 순서)
};

// 정차 1회 = 이동 + 팔 세션
struct Stop
{
  int node = 0;
  std::vector<ArmAction> actions;
  std::string name;           // 상태 표시용 ("B3 수집", "L2 배달" …)
};
```

## 4. 수집 단계

### 4.1 정차 목록 (계획 전, 정보 없이 정함)

- `collect_order` (설정, 기본 `[1, 2, 6, 4, 5, 3]`, 2026-11-04 L2 박스 시간표 기준 수집 → 배달 → 재검증 → 복귀 이동 합 720 가지 중 1 등 293.7 s, 옛 `[1, 2, 6, 3, 4, 5]` 은 298.9 s 8 등) 를 그대로 씀. 옛 순서는 720 가지 순서 비교에서 1 등 (실물 확인 · 하드웨어 끝 속도, 넣기+빼기 30 · 60 s 네 조건 모두)
- 박스 b 의 정차 = `Look` → 퍽마다 [`Store`(그리퍼 퍽 → 빈 칸, 그리퍼가 차 있을 때만)] → `Pick`
- **마지막 박스는 Look 만 미리 넣음** → Look 결과로 계획한 뒤 집는 순서를 정함 (4.3)
- `defer_boxes` 에 든 박스는 Look 만 하고 집지 않음 (C안)

### 4.2 박스 Look 결과 처리

| 상황 | 처리 |
|---|---|
| 퍽 2개, ID · 색 · 모양 모두 읽음 | 기록부 갱신 → 집기 진행 |
| 퍽 2개, 일부 속성 못 읽음 | 손목 자세 · 차체 옆 이동으로 다시 Look (최대 `relook_max` 2회) → 그래도 못 읽으면 집은 뒤 그리퍼 안에서 다시 봄 |
| 퍽 수 ≠ 2 | 다시 Look 2회 → 그대로면 본 대로 기록하고 진행 (최종 판정은 4.3) |
| 집은 퍽의 ID 가 Look 결과와 다름 | 집은 결과(손목 근접 관측) 를 믿고 기록부 수정 |

### 4.3 계획 시점 (마지막 박스 Look 직후)

1. 12개 ID 검사: 0–11 이 하나씩 있어야 함. 중복 · 누락 → `MISSION_ERROR(ID_DUP_MISSING)` (발제 규칙, 추정 금지)
2. `dice_layout()` 으로 장소별 그룹 · 눈 수 확인 (할당 가능 여부는 3 의 solver 가 판정, 해 없음 → `MISSION_ERROR(UNSOLVABLE)`) (2026-11-02 `plan_dice` 삭제)
3. `TransferSolver` 호출 (5장) → 마지막 박스에서 집는 순서 · 장소 순서 · 장소별 꺼내기 순서 · 할당 (Y 모양, Y 로 보낼 퍽) 결정
4. 결과를 정차 목록으로 펼쳐 마지막 박스 정차 뒤에 이어 붙임

## 5. TransferSolver (정확해)

### 5.1 상태 · 동작

| 상태 항목 | 값 |
|---|---|
| 남은 퍽 | 아직 박스에 있는 퍽 (12 bit) |
| 위치 | 노드 0–9 |
| 그리퍼 | 비었음 또는 퍽 종류 1개 |
| 적재함 | 종류별 개수 (빨원 · 빨사 · 초원 · 초사) |
| Y 로 보낸 수 | 빨강 · 초록 각각 |
| Y 모양 s | 바깥 반복 (가능한 모양마다 따로 풀고 작은 쪽) |

| 동작 | 조건 | 비용 |
|---|---|---|
| 이동 a → b | 박스: 남은 퍽 있고 실을 칸 있음, 장소: 실은 퍽 있음 | `travel_s[a][b] + stop_s` |
| Pick | 박스에서, 그리퍼 빔 | `pick_s` |
| Store | 그리퍼 참, 빈 칸 있음 | `store_s` |
| Retrieve | 그리퍼 빔, 칸에 퍽 있음 | `retrieve_s` |
| Place | 장소에서, 그리퍼 퍽이 그 장소 그룹 조건에 맞음 (R = 빨강, G = 초록, Y = 모양 s, Y 개수 한도) | `place_s` |
| 끝 | 남은 퍽 · 그리퍼 · 칸 모두 비고 Y 개수 = 필요 수 | 재검증 순회 (장소 3곳 순서 최소) + 출발점 복귀 |

- 같은 종류 퍽은 서로 바꿔도 비용이 같음 → 퍽 ID 대신 종류로 상태를 세서 크기를 줄임. 해를 펼칠 때 실제 ID 를 배정 (Y 로 보낼 퍽 = 같은 종류 중 어느 것이든)
- 적재함 칸은 서로 같다고 봄 (칸별 넣기 · 빼기 시간이 다르면 `slot_cost_s` 로 확장, 7장)

### 5.2 계산 시간 (개발 PC, 평가기 `eval3` 기준)

| 시작 상태 | 1회 계산 |
|---|---|
| A안 계획 시점 (11개 실음, 1개 남음) | 약 3 ms |
| C안 (4개 남음) | 수십 ms |
| 12개 모두 남음 (최악, 수집 전 실패 복구) | 약 9 s |

- Jetson 은 몇 배 느릴 수 있음 → 남은 퍽이 `max_remaining` (기본 4) 보다 많으면 대체 해: 남은 퍽을 가까운 박스 순으로 다 싣고 배달만 정확해 (A안과 같은 구조)
- 구현 측정 (개발 PC): 모두 실림 25 ms, 남은 퍽 1개 60 ms, 남은 퍽 6개 2.1 s, 12개 대체 해 24 ms

## 6. 배달 단계 정차

- 장소 정차 = `Look`(장소 태그) → 칸마다 [`Retrieve`(그리퍼가 비었을 때)] → `Place(place, cell)`
- 눈 방향: **배치대 앞 관찰 원에서 봤을 때 주사위 모양** (2026-10-30 사용자 결정). `apply_view_directions()` 가 그림 좌표 → 지도 좌표로 바꾸고 먼 칸부터 정렬
  - 보는 방향 (관찰 원 → 장소 중심, 발제 그림): L1 (1, 0) 서 → 동, L2 (0, −1) 북 → 남, L3 (0, 1) 남 → 북
  - 발제 p.4 문구 "지도 위쪽이 +Y, 모든 장소 같은 방향" 과 L1 에서 90° 다름 → 운영자 확인 필요. 지도 기준으로 바꾸려면 보는 방향을 전부 (0, 1)
- 칸 순서: `order_far_first()` (보는 사람 = 로봇 쪽에서 먼 칸부터)
- 꺼내기(RETRIEVE) 는 장소 정차 안에서 바로 놓을 퍽만 → 다음 장소 퍽을 그리퍼에 쥐고 주행하지 않음
- **퍽 쥔 채 주행 안 함 (2026-11-04)**: 이동은 그리퍼가 빌 때만, 박스에서 집은 퍽은 전부 STORE (마지막 박스 퍽도). control 이 쥔 채 DRIVE · HOME 을 안 함
- Place 목표 계산은 F9 (태그 기준 칸 좌표 → base_link) 담당. transfer_planner 는 (장소, 칸 번호) 만 넘김
- 장소마다 마지막 Place 뒤 `Look` 1회 (그 장소 실측 기록, 재검증과 별개 증거)

## 7. 설정 (`config/planning.yaml` 의 `transfer:`)

```yaml
transfer:
  collect_order: [1, 2, 6, 4, 5, 3]   # 시작 박스 방문 순서
  defer_boxes: []                     # C안이면 [3, 6] (Look 만 하고 배달 때 집음)
  cargo_slots: [S1, S2, S3, S4, S5, S6, S7, S8, S9, S10, S11, S12]   # URDF rack_cell_S* 이름, 먼 칸부터 채움
  travel_table: maps/dice_travel_times.yaml   # 노드 쌍 이동 시간 [s]
  stop_s: 20.0
  look_s: 5.0
  pick_s: 30.0
  place_s: 30.0
  store_s: 15.0
  retrieve_s: 15.0
  relook_max: 2
  pick_retry_max: 2
  solver_max_remaining: 6
```

- 시간 값은 지금 가정값 → control 실측 받으면 갱신. 이동 시간표는 `sim/robot_sim/results/dice_route_times_confirmed_v2.json` 에서 만듦 (실물 Nav2 주행 시간으로 나중에 교체)

## 8. 실패 · 재계획

- 원칙: 팔 작업 결과가 계획과 다르면 남은 정차를 버리고 **실제 상태(기록부 + Cargo)** 에서 `TransferSolver` 를 다시 부름. 이미 놓은 퍽은 고정

| 실패 | 처리 |
|---|---|
| Pick 실패 | `pick_retry_max` 회 재시도 → 그래도 실패면 그 퍽은 박스에 남은 것으로 두고 진행 (다음 계획에서 다시 집으러 감) |
| Store · Retrieve 실패 (퍽 떨어뜨림) | 그 퍽 = 잃음. 할당 다시 계산 → 12개 조건 못 맞추면 그 퍽 그룹 장소를 "불완전" 으로 기록하고 나머지 장소는 계속 |
| Place 실패 · 놓은 뒤 넘어짐 | 그 장소는 건드리지 않음 (빈 칸 오염 위험). 기록만 남기고 다음 칸 |
| 재계획 결과 해 없음 | 남은 장소 중 완성 가능한 곳만 채우고 VERIFY · RETURN 로 감 (점수 보존) |

## 9. 노드 변경 (`mission_manager.cpp`, 옛 `planning_node.cpp`) — 2026-11-01 구현

| 항목 | 구현 |
|---|---|
| 단계 | HOME → READ_BARCODE → OBSERVE_SIGNAL → COLLECT → DELIVER → VERIFY → RETURN_HOME (발제 순서: 바코드 → 점멸 → 이동) |
| 시작 조건 | 신호 기록 DONE + 횟수 규칙 (각 1–6, 합 12). 아니면 task 실패로 복귀 (색 추정 안 함) |
| COLLECT | `collect_steps`: 박스마다 Navigate(정차) → Arm LOOK → Plan. Plan(CollectBox) 이 `look_dwell_s` 뒤 박스 반경 안 on_field 퍽으로 `collect_box_actions` → 바로 뒤에 끼워 넣음 |
| 계획 | 마지막 박스 Plan(Delivery): 실음 + 그리퍼 + 박스 퍽 → `dice_layout` (그룹 · 눈 수 · 눈 위치) → `apply_view_directions` → `plan_transfer`. 실패 = FAULT `DICE_UNSOLVABLE` |
| DELIVER | `transfer_steps`: move 정차는 Navigate, 팔 동작은 Arm (HOME 안 넣음). PLACE 목표 `Target::Custom` + kind `kTargetDiceCell` (옛 `Target::DiceCell`) = 장소 중심 + `pip_offsets[cell]` (map) → base_link, 높이 top_z + 퍽 반높이 + grasp_offset |
| PICK | 정차 중이라 LOOK 때 본 odom 위치를 `dice_pick_fresh_s` (90 s) 까지 씀 (STORE 뒤에도) |
| VERIFY | `verify_steps`: 재검증 순서로 장소마다 Navigate → LOOK → Plan(Verify). `dice_verify`: LOOK 뒤 새 관측 (반경 `verify_radius_m` 0.18) → 칸 (`verify_cell_tol_m` 0.03) · 그룹 · 개수 판정, 세 곳 다 보면 ID 중복 · 누락 → `/planning/verify_record` (VerifyRecord, transient_local) + `~/planning_logs/verify_<시각>.yaml`. 판정이 나빠도 미션은 계속 (기록이 목적) |
| 순서 | 배치 순서 · 재검증 순서 모두 정확해가 고름 (끝 비용 = 재검증 순회 + 복귀). Q1 (B5 출발) 6 가지 비교: L2→L3→L1 + 재검증 L1→L3→L2 가 이동 174.6 s 로 최소 (2026-11-01 확인, 다음 L3→L2→L1 179.1 s) |
| 실패 · 재계획 (2026-11-01) | 재시도 뒤에도 실패하면 `dice_recover`: PICK 실패 = 그 퍽 박스에 남김 (skipped), STORE · RETRIEVE · PLACE 실패 = 그 퍽 잃음 (lost), PLACE 실패 장소는 버림. COLLECT 는 그 박스 남은 단계를 지우고 같은 박스 Plan 다시 (이동 실패면 박스 건너뜀), DELIVER 는 지금 상태에서 `plan_from_state`, VERIFY 는 그 장소만 건너뜀. `max_replans` (6) 넘으면 포기 · 복귀 |
| 부분 계획 | `plan_from_state`: 기록부 박스 위 퍽 + 적재함 + 장소별 놓은 수로, **끝까지 채울 수 있는 장소가 가장 많은 조합** (같은 수면 시간 짧은 쪽) 을 고름. 정확해 `allow_leftover` (남는 퍽 허용, Y 모양 가능 여부는 R/G 가 모양 s 아닌 퍽부터 쓴다고 보고 판정). 처음 계획도 같은 함수 (12 개면 세 곳) |
| 가짜 세계 시험 | PICK 3 회 실패 (초록 3) → L1 Y2 · L2 R4 채움, L3 G6 못 채움 (6/12) / PLACE 실패 (8, L2) → L2 버림, L1 · L3 채움 (8/12) / STORE 실패 (초록 1 떨어뜨림) → L1 · L2 (6/12). 정상 · PICK 1 회 실패 12/12 PASS 그대로 |

설계 당시 표 (참고)


| 지금 | 바꿀 것 |
|---|---|
| Phase: HOME → OBSERVE_SIGNAL → READ_BARCODE ⇄ EXECUTE → RETURN_HOME | HOME → OBSERVE_SIGNAL → READ_BARCODE → **COLLECT → DELIVER → VERIFY** → RETURN_HOME |
| EXECUTE = task 1개의 Step 목록 | COLLECT · DELIVER = `Stop` 목록을 Step 목록으로 펼쳐 기존 실행기에 넣음 (Navigate + Arm 연속) |
| 팔 작업마다 HOME 으로 끝남 | 한 정차 안 팔 동작은 이어서 하고 마지막에만 HOME (control 세션 약속 필요) |
| READ_BARCODE 뒤 task 해석 | 바코드 원문 → `parse_dice_barcode()` + 신호 횟수 → DicePlan 준비 (퍽 정보는 COLLECT 끝에) |

- Step 확장: `StepType::Arm` 에 Look(LOOK_BOX · LOOK_TAG) 과 Store · Retrieve 사용, `Step.slot` · `Step.puck_id` 이미 있음, Place 용 `place` · `cell` 필드 추가

## 10. 시험 (gtest)

| 시험 | 확인 |
|---|---|
| Q1 (RGRRGRGGYGYG, P2-01) + 고정 배치 | 정차 목록 = 박스 6 + 장소 3, Place 12 개, 장소별 퍽이 그룹 조건 만족 |
| 장소 순서 6 가지 전부 비교 | 해가 6 가지 중 최소 비용과 같음 |
| 그리퍼 퍽 | 마지막에 집은 퍽이 첫 장소 퍽이면 Store 없이 바로 Place |
| ID 중복 · 누락 | MISSION_ERROR |
| Pick 실패 1회 → 재계획 | 남은 퍽 1개를 집으러 가는 정차가 생김 |
| Store 실패 (퍽 잃음) | 그 그룹 장소만 불완전, 나머지는 완성 |
| 정확해 vs 무차별 대입 | 퍽 4개짜리 작은 문제에서 비용 같음 |
| defer_boxes [3, 6] | B3 · B6 는 Look 만, 배달 중 집음 |
| 계산 시간 | A안 계획 시점 < 50 ms (개발 PC) |

## 11. 다른 파트 의존

| 대상 | 필요한 것 |
|---|---|
| control | 적재함 12칸 이름 · 도달 여부, STORE · RETRIEVE · LOOK 동작, 정차 세션 (마지막에만 HOME), 퍽 쥔 채 주행, 각 동작 시간 · 성공률 |
| perception | 손목 Look 결과 = 퍽 ID · 색 · 모양 (박스 위 2개), 장소 태그 ID 12–14 자세 |
| Nav2 | 노드 10개 정차 자세 도달, 실제 이동 시간 (시간표 갱신) |
| 운영자 | 장소 완성 즉시 재검증 허용 여부 (허용되면 VERIFY 순회 제거, 약 −100 s) |
