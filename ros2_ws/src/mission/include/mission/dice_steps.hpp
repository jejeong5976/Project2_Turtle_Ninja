// dice_steps.hpp — 발제 v5 dice 흐름의 단계 Step 만들기
//
// 역할: 단계별로 만드는 Step
//   - READ_BARCODE: 이동 → 팔 LOOK_BARCODE → ReadBarcode → 팔 HOME. look = false 면 이동 → 읽기
//   - COLLECT: 박스마다 이동 → LOOK → Plan CollectBox, 마지막 박스는 이동 → LOOK → Plan Delivery
//       Plan CollectBox 가 LOOK 결과로 [STORE] PICK … 을 바로 뒤에 끼워 넣음 = collect_box_steps
//       박스 퍽이 side_look_expected 보다 적게 보이면 Plan 이 먼저 옆 보기 → 같은 Plan 다시 = side_look_steps
//       relook_before_pick: 손목이 그 퍽을 안 보는 자세에서 시작하는 PICK 앞에 LOOK 다시 = with_pick_relooks
//   - DELIVER: Plan Delivery 가 12 개 정보로 plan_transfer → 정차 목록 → transfer_steps
//       첫 정차는 마지막 박스에서 이어서, 이동 없이 집기부터
//   - VERIFY: 재검증 순서대로 장소마다 이동 → LOOK → Plan Verify
//   - RETURN_HOME: 출발점으로 이동
// 입력
//   - DiceMap, 박스 순서, 정차 목록 TransferPlan, 재검증 순서
// 출력
//   - planning::Step 목록 → mission_manager 가 실행기에 넘김
// 실행
//   - mission_core 라이브러리, mission_manager 가 부름. 시험 test_mission_core · test_transfer
// note
//   - 설계 docs/planning/transfer_planner_design.md, A안 = 돌면서 12 개 다 싣기
//   - 한 정차 안 팔 동작은 이어서 보내고 HOME 은 안 넣음, 마지막 접기는 control 이 판단
//   - rack_look: STORE 마다 바로 앞에 LOOK_RACK (같은 칸 · 같은 퍽) → 실행기가 칸 측정을 STORE target 에 넣음
//   - PLACE 목표 = Target::Custom + kTargetDiceCell → 노드가 dice_cell_point 로 지도 점을 구해 base_link 로 바꿈
//   - Step.kind · ref 약속은 common.hpp

#pragma once

#include <array>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "mission/dice.hpp"
#include "mission/dice_map.hpp"
#include "mission/transfer_planner.hpp"
#include "mission/common.hpp"

namespace mission
{

// COLLECT 단계 목록. order = 박스 방문 순서 (1–6, 중복 없이). 잘못되면 빈 목록 + error
std::vector<Step> collect_steps(const DiceMap & arena, const std::vector<int> & order, std::string * error = nullptr);

// 팔 동작 목록 → Arm 단계 (Look · Pick · Store · Retrieve · Place). rack_look 이면 STORE 앞에 LOOK_RACK (같은 칸)
//   box_wait: 장소 LOOK 뒤 Custom kPlaceBox (박스 측정으로 칸 기준 확정)
std::vector<Step> arm_steps(
  const std::vector<ArmAction> & actions, bool rack_look = false, bool box_wait = false);

// 박스 옆 보기 (박스 LOOK 에 퍽이 모자랄 때): control_master LOOK slot
//   "right" · "left" = LOOK 자세에서 J1 ±look_side_deg, "near" = J3 +look_near_tilt_deg, "far" = J4 −look_far_tilt_deg
//   (노드 기본 side_look_order 는 left · near, 여기 기본값은 예전 그대로)
struct SideLookConfig
{
  bool enabled = true;                                 // side_look_missing
  int expected = 2;                                    // side_look_expected: 이보다 적게 보이면 옆 보기
  std::vector<std::string> order{"right", "left"};     // side_look_order: 볼 순서 (right · left · near · far 만 씀)
};

bool is_side_slot(const std::string & slot);   // "right" · "left" · "near" · "far"
bool is_side_look(const Step & s);             // 팔 LOOK + 옆 보기 slot
bool is_pick_relook(const Step & s);           // 팔 LOOK + kind kPickRelook + puck_id 없음 (with_pick_relooks 가 끼운 PICK 앞 다시 보기)
// 새 정차의 박스 앞 LOOK (옆 보기 아님 · puck_id 없음 · PICK 앞 다시 보기 아님) = 미션 관측 구간 look_started_ 와 옆 보기 상태를 새로 잡는 LOOK
//   다시 보기 LOOK (실행기 insert_relook · box_relook_step = puck_id 있음, with_pick_relooks = kPickRelook) 은 구간을 그대로 둠
//   → 옆 보기로만 본 퍽도 다시 계획에 남음 (다시 본 퍽은 기록부가 위치 · last_seen 을 새로 씀)
bool starts_box_look(const Step & s);

// 박스 Plan 단계 plan 이 쓸 퍽 seen 개를 봤을 때 바로 뒤에 끼울 옆 보기 단계
//   [LOOK slot1 → 기다리기 → LOOK slot2 …] → 같은 Plan 다시 (마지막 LOOK 뒤 기다리기는 Plan 의 look_dwell_s)
//   빈 목록 = 옆 보기 안 함: 꺼짐 · seen ≥ expected · 박스 Plan 아님 · 쓸 slot 없음
std::vector<Step> side_look_steps(const Step & plan, std::size_t seen, const SideLookConfig & cfg);

// 박스 정차 하나의 옆 보기 상태. 앞 LOOK 이 Unchecked 로 되돌림 (다시 보기 LOOK 은 안 되돌림)
//   Unchecked = 이 정차 첫 Plan 전 · Pending = 옆 보기 끼움, 합친 Plan 전 · Done = 끝 (이 정차에서 다시 안 함)
enum class SideLookState : uint8_t { Unchecked, Pending, Done };

// Plan 한 번 실행 때: Unchecked 면 판단 (끼울 단계 있으면 Pending, 없으면 Done), Pending 이면 Done (합친 계획), Done 이면 그대로
//   반환 = 지금 끼울 옆 보기 단계 (빈 목록 = 바로 계획)
std::vector<Step> side_look_on_plan(SideLookState & st, const Step & plan, std::size_t seen, const SideLookConfig & cfg);

// PICK 빈손 (EMPTY_GRASP) 뒤 같은 박스 Plan 다시 전에 끼울 앞 LOOK (손가락이 옆 퍽을 밀었을 수 있음 → 새 검출로 계획)
//   steps[at] 앞에서 뒤로 이동 단계 전까지, 그 박스 앞 LOOK (옆 보기 아님) 을 복사. puck_id 를 달아 다시 보기 LOOK
//   (PICK 앞 다시 보기 LOOK 을 복사해도 kind 는 지움 = 실패하면 예전처럼 재계획)
//   (미션이 옆 보기 상태 · 관측 시작 look_started_ 를 안 바꿈 = starts_box_look 거짓 → 옆 보기로만 본 퍽도 Plan 다시에 남음)
//   없으면 nullopt (예전처럼 Plan 만)
std::optional<Step> box_relook_step(const std::vector<Step> & steps, std::size_t at, int box, const std::string & puck_id);

// ---- PICK 앞 다시 보기 (relook_before_pick)
//   PICK 은 손목 카메라가 그 퍽을 보는 LOOK 자세에서 시작해야 control 이 방금 검출에 맞춤 (11-10 밤: 맞춤 11/11,
//   STORE 뒤 P0 · 그 퍽이 안 보이는 옆 보기 자세에서 LOOK 때 위치로 집은 것은 2/10 빈손)
// 박스 정차 LOOK 자세 하나에 서 있던 구간 (LOOK 성공 ~ 다음 팔 작업 보냄). slot "" = 앞 LOOK, to < 0 = 아직 그 자세
struct LookWindow
{
  std::string slot;
  double from = 0.0;
  double to = -1.0;
};
struct SeenAt                                    // 퍽 관측 하나 (촬영 시각 [s])
{
  std::string id;
  double stamp = 0.0;
};
using PuckViews = std::map<std::string, std::vector<std::string>>;   // 퍽 ID → 그 퍽이 찍힌 LOOK slot (구간 순서, 중복 없음)

// 구간마다 그 안에서 찍힌 퍽 → 그 slot. ID 없는 관측 · 구간 밖 (팔이 움직이는 중) 관측은 안 씀
PuckViews puck_views(const std::vector<LookWindow> & windows, const std::vector<SeenAt> & seen);
// 다시 볼 slot: 앞 LOOK 에서 봤거나 기록이 없으면 "" (앞), 옆 보기로만 봤으면 처음 본 slot
std::string relook_slot(const PuckViews & views, const std::string & id);
// 박스 PICK 마다 바로 앞 팔 단계가 같은 박스 LOOK 이고 그 slot 에서 그 퍽을 봤으면 그대로,
//   아니면 PICK 앞에 [LOOK (relook_slot, kind kPickRelook, puck_id 없음) → 기다리기 kRelookDwell (puck_id)] 를 끼움
//   held = steps 바로 앞의 마지막 팔 단계 (같은 정차, 이동 뒤 · 없으면 nullopt). 이동 단계는 자세를 지움
std::vector<Step> with_pick_relooks(
  const std::vector<Step> & steps, const std::optional<Step> & held, const PuckViews & views);

// 운반 계획 → DELIVER 단계 (move 인 정차는 이동 먼저)
std::vector<Step> transfer_steps(
  const TransferPlan & plan, const DiceMap & arena, bool rack_look = false, bool box_wait = false);

// 재검증 단계 (verify_order 순서로 장소마다 이동 → LOOK → Plan(Verify))
std::vector<Step> verify_steps(const std::array<int, kNumPlaces> & verify_order, const DiceMap & arena);

// 바코드 지점 1곳 읽기
std::vector<Step> build_read_steps(const Pose2D & barcode_point, double read_timeout_s, bool look = false);

// 출발점 복귀
std::vector<Step> build_return_steps(const Pose2D & start_pose);

// 장소 place 의 cell 번째 칸 지도 점 [m] (장소 중심 + pip_offsets[cell]). plan 은 apply_view_directions 를 거친 것
std::optional<std::array<double, 2>> dice_cell_point(
  const DicePlan & plan, const DiceMap & arena, int place, int cell);

// 보는 방향 3 개 (apply_view_directions 입력)
std::array<std::array<double, 2>, 3> place_view_dirs(const DiceMap & arena);

// 장소 PLACE 보정 [앞, 왼쪽] mm (정차점에서 장소를 보는 로봇 기준, planning place_offset_L*_mm) → map 이동 [m]
//   앞 = view_dir, 왼쪽 = 그 90° 왼쪽. 놓는 점에만 더함 (재검증 · 기록은 눈 점 그대로)
std::array<double, 2> place_offset_map(const std::array<double, 2> & view_dir, const std::array<double, 2> & fwd_left_mm);

}  // namespace mission
