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
//   - 넣기 = 목록 앞 빈 칸 (cargo_slots 순서 = 뒷줄부터), 꺼내기 = 그 장소 그룹에 맞는 퍽이 든 칸 중 목록 뒤 (나중에 넣은 칸) 먼저
//   - 칸 제약 (넣기 · 빼기 전에 비어야 할 칸) 을 지키게 퍽 · 칸 · 장소 안 순서를 DFS 로 고름. 안 되면 장소 방문 순서 · Y 모양을
//     바꾼 다른 해, 그래도 안 되면 막는 퍽을 빈 칸으로 옮기기 (1 → 2 번), 끝까지 없으면 예비 펼치기 (막지 않음)
//   - 쥔 퍽은 출발 전에 지금 자리에서 넣음 (재계획 때 그리퍼에 퍽이 남은 경우)
//   - 박스에 남은 퍽이 max_remaining 기본 4 개보다 많으면 정확해 대신 대체 해: 가까운 박스 순 수집 → 배달만 정확해

#pragma once

#include "mission/common.hpp"

#include <array>
#include <map>
#include <optional>
#include <set>
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

// 배달 계획 고르기 (mission_manager plan_from_state): 채울 장소 조합마다 plan_transfer 를 풀어 하나 고름
//   1. 채우는 장소 수가 많은 쪽 (퍽이 모자라면 일부 장소만)
//   2. prefer_more_pucks (노드 alloc_prefer_more_pucks, 기본 false) 면 같은 장소 수 중 놓는 퍽이 많은 쪽 (예 R6+G2 = 8 > G2+Y4 = 6)
//   3. 예상 시간 짧은 쪽
//   base   : 장소 그룹 · 위치 · 퍽 (slots_left 는 여기서 조합마다 채움)
//   need   : 장소별 더 놓을 수. 0 = 끝났거나 버린 장소 → 그 장소가 든 조합은 뺌
//   avail  : 남은 퍽 수 (박스 + 적재함 + 그리퍼). 조합의 놓을 수 합이 이보다 많으면 뺌
//   error  : 하나도 못 풀면 첫 풀이 실패 이유 (비어 있을 때만 씀)
//   partial: 부분 채우기 (노드 fill_partial_places). 장소마다 0 ~ need 개를 놓을 수 있음 → 놓는 퍽 수 최대 → 다 채운 장소 수 →
//            예상 시간 순. 색 · 모양 개수로 안 되는 조합은 먼저 거름. 덜 채운 장소는 앞 (먼) 칸부터 채우고 뒤 칸이 빔.
//            false = 위 1–3 (끝까지 채울 수 있는 장소만)
struct DeliveryChoice
{
  std::optional<TransferPlan> plan;                // 없으면 채울 수 있는 장소 없음
  int n = -1;                                      // 채우는 장소 수
  int mask = 0;                                    // 채우는 장소 비트 (1 = L1, 2 = L2, 4 = L3)
  int pucks = 0;                                   // 놓을 퍽 수
  std::array<int, kNumPlaces> count{{0, 0, 0}};     // 장소별 놓을 수 (partial 이면 need 보다 작을 수 있음)
  int time_mask = 0;                               // 같은 장소 수 중 시간만 본 선택 (prefer_more_pucks 가 바꿨는지 기록용)
  int time_pucks = 0;
  double time_cost_s = 0.0;
};
DeliveryChoice choose_delivery(
  const SolverInput & base, const std::array<int, kNumPlaces> & need, int avail,
  const TransferCosts & costs, const Cargo & cargo, const std::array<int, kNumPlaces> & cells_done,
  int max_remaining, bool prefer_more_pucks, std::string * error = nullptr, bool partial = false);

// 수집 정차 1곳의 팔 동작 (LOOK 은 정차 시작 때 노드가 이미 함)
//   at_box : 이 박스에서 본 퍽 (보통 2개, 집을 순서대로)
//   그리퍼가 이미 차 있으면 그것부터 STORE (박스에 남은 퍽이 없어도) → 퍽마다 PICK → STORE. 끝나면 그리퍼 빔 = 바로 주행 가능
//   빈 칸이 모자라면 빈 목록 + error
std::vector<ArmAction> collect_box_actions(
  const std::vector<TransferPuck> & at_box, const Cargo & cargo, std::string * error = nullptr);

// 쥔 퍽을 지금 자리에서 비우는 방법 (STORE 가 실패한 뒤 다시 계획할 때)
//   failed    : 이 퍽 STORE 가 실패한 칸 (거절 · 보내기 전 실패). 비면 Store = free_slot_for
//   alt_cells : 실패한 칸이 이 수보다 많으면 칸을 더 안 바꾸고 박스에 되돌림. 0 = 실패한 칸 그대로 다시
//   putback_ok: 박스에 되돌려 놓을 수 있음 (집은 정차 그대로 · 집은 자리 앎 · 아직 되돌림 실패 안 함)
//   Store  = 실패 안 한 빈 칸 중 store_ok (free_slot_for 가 고른 칸이 되면 그것, 아니면 목록 앞 칸). 제약 없는 첫 빈 칸 (free_slot 끝) 은 안 씀
//   PutBack= 그런 칸이 없거나 alt_cells 를 다 씀 → 집은 자리에 PLACE (kTargetPutback)
//   None   = 그리퍼 빔, 또는 칸도 되돌림도 못 함 (부르는 쪽이 보통 계획)
struct HeldChoice
{
  enum class Kind : uint8_t { None, Store, PutBack };
  Kind kind = Kind::None;
  std::string slot;   // Store 일 때
};
HeldChoice held_puck_choice(const Cargo & cargo, const std::set<std::string> & failed, int alt_cells, bool putback_ok);

// 적재함 여유 칸 (kRackSpareCells = 1 고정): 박스 위 퍽 id 를 빼고 (= 안 싣고) 계획해도 채울 장소 수 · 놓는 퍽 수가
//   그대로면 true. 수집 Plan 이 마지막 빈 칸을 쓰려 할 때 그 퍽이 필요 없으면 박스에 두어 한 칸을 남김 (DELIVER 막는 퍽 옮기기 · 다른 칸 STORE 용).
//   필요하면 (점수가 줄면) false → 마지막 칸도 씀 (거절 아님)
constexpr int kRackSpareCells = 1;
bool puck_not_needed(
  const SolverInput & base, const std::string & id, const std::array<int, kNumPlaces> & need,
  const TransferCosts & costs, const Cargo & cargo, const std::array<int, kNumPlaces> & cells_done,
  int max_remaining, std::string * why = nullptr, bool partial = false);

// 비용표 yaml
//   nodes: [S, B1, B2, B3, B4, B5, B6, L1, L2, L3]   # 이 순서여야 함
//   travel_s: [[…10개…], …10줄…]                   # [s], travel_s[a][b] = a → b
//   stop_s · look_s · pick_s · place_s · store_s · retrieve_s · verify_tour (없으면 기본값)
bool load_transfer_costs(const std::string & path, TransferCosts & out, std::string * error = nullptr);

}  // namespace mission
