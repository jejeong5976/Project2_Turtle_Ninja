// transfer_planner.hpp — 퍽 수집 · 배달 정차 목록, A안 = 돌면서 다 싣기
//
// 역할
//   1. 수집 정차의 팔 동작: 박스 LOOK 결과 → PICK → STORE …, 받은 순서 = 가까운 퍽부터, 마지막 퍽도 넣음 (퍽 쥔 채 주행 안 함)
//   2. 계획: transfer_solver 해 = 종류 단위 → 실제 퍽 ID · 적재함 칸 이름 · 놓을 칸 번호를 붙인 정차 목록
//   3. 비용표 yaml 읽기: 이동 시간 · 정차 · 팔 동작 시간
// 입력
//   - 기록부 · Cargo 로 만든 SolverInput, 장소별 이미 놓은 수
//   - 비용표 maps/dice_travel_times.yaml
// 출력
//   - TransferPlan: 정차 목록, 노드가 Step 으로 펼쳐 실행
// 실행
//   - mission_core 라이브러리, mission_manager 가 부름. 시험 test_transfer
// note
//   - 칸 이름은 Cargo 기록을 그대로 따라 계산함 = 어느 칸에 몇 번 퍽이 있는지
//   - 넣기 = 목록 앞 빈 칸 (S1 = 팔에서 가장 먼 칸), 꺼내기 = 그 종류가 든 칸 중 목록 뒤 (가까운 칸) 먼저
//   - 박스에 남은 퍽이 max_remaining 기본 4 개보다 많으면 정확해 대신 대체 해: 가까운 박스 순 수집 → 배달만 정확해

#pragma once

#include "mission/common.hpp"

#include <array>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "planning/cargo.hpp"
#include "mission/transfer_solver.hpp"
#include "mission/transfer_types.hpp"

namespace mission
{

struct TransferPlan
{
  std::vector<Stop> stops;                         // 순서대로 실행 (첫 정차가 move=false 면 지금 자리에서 이어서)
  double cost_s = 0.0;                             // 예상 총 시간 (재검증 · 복귀 포함)
  std::string yellow_shape;
  std::array<int, kNumPlaces> verify_order{{0, 1, 2}};
  std::map<std::string, int> place_of;             // 퍽 ID → 장소 0–2 (할당 결과)
  std::size_t expanded = 0;
};

// Cargo 기록 (그리퍼 · 칸) → SolverInput.gripper · in_cargo. 색 · 모양을 모르는 퍽이 있으면 false + error
bool fill_carried(const Cargo & cargo, SolverInput & in, std::string * error = nullptr);

// 계획 + 펼치기
//   cargo         : 지금 적재함 기록 (칸 이름 · 내용). in.gripper · in.in_cargo 와 같아야 함 (fill_carried 로 만들 것)
//   cells_done    : 장소별 이미 놓은 수 → 이번 계획의 첫 칸 번호
//   max_remaining : 박스에 남은 퍽이 이보다 많으면 정확해 대신 "가까운 박스 순으로 다 싣고 → 배달만 정확해"
//                   (12개 다 남은 정확해는 상태 수백만 개 · 수십 초. Jetson 시간 보호)
std::optional<TransferPlan> plan_transfer(
  const SolverInput & in, const TransferCosts & costs, const Cargo & cargo,
  const std::array<int, kNumPlaces> & cells_done, std::string * error = nullptr,
  int max_remaining = 4);

// 수집 정차 1곳의 팔 동작 (LOOK 은 정차 시작 때 노드가 이미 함)
//   at_box : 이 박스에서 본 퍽 (보통 2개, 집을 순서대로)
//   퍽마다 PICK → STORE (그리퍼가 이미 차 있으면 그것부터 STORE). 끝나면 그리퍼 빔 = 바로 주행 가능
//   빈 칸이 모자라면 빈 목록 + error
std::vector<ArmAction> collect_box_actions(
  const std::vector<TransferPuck> & at_box, const Cargo & cargo, std::string * error = nullptr);

// 비용표 yaml
//   nodes: [S, B1, B2, B3, B4, B5, B6, L1, L2, L3]   # 이 순서여야 함
//   travel_s: [[…10개…], …10줄…]                   # [s], travel_s[a][b] = a → b
//   stop_s · look_s · pick_s · place_s · store_s · retrieve_s · verify_tour (없으면 기본값)
bool load_transfer_costs(const std::string & path, TransferCosts & out, std::string * error = nullptr);

}  // namespace mission
