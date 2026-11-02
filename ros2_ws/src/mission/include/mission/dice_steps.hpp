// dice_steps.hpp — 발제 v5 dice 흐름의 단계 Step 만들기
//
// 역할: 단계별로 만드는 Step
//   - READ_BARCODE: 이동 → 팔 LOOK_BARCODE → ReadBarcode → 팔 HOME. look = false 면 이동 → 읽기
//   - COLLECT: 박스마다 이동 → LOOK → Plan CollectBox, 마지막 박스는 이동 → LOOK → Plan Delivery
//       Plan CollectBox 가 LOOK 결과로 [STORE] PICK … 을 바로 뒤에 끼워 넣음 = collect_box_steps
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
//   - PLACE 목표 = Target::Custom + kTargetDiceCell → 노드가 dice_cell_point 로 지도 점을 구해 base_link 로 바꿈
//   - Step.kind · ref 약속은 common.hpp

#pragma once

#include <array>
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

// 팔 동작 목록 → Arm 단계 (Look · Pick · Store · Retrieve · Place)
std::vector<Step> arm_steps(const std::vector<ArmAction> & actions);

// 운반 계획 → DELIVER 단계 (move 인 정차는 이동 먼저)
std::vector<Step> transfer_steps(const TransferPlan & plan, const DiceMap & arena);

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

}  // namespace mission
