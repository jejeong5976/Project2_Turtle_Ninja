// transfer_solver.hpp — 퍽 운반 정확해
//
// 역할
//   - 지금 실제 상태에서 남은 퍽을 모두 놓고 재검증 · 복귀까지의 최소 시간 동작 순서를 Dijkstra 로 구함
//       상태 = 위치 · 박스에 남은 퍽 · 그리퍼 · 적재함 · 장소별 남은 칸
//   - 할당도 같이 정함: R 장소 = 빨강, G 장소 = 초록, Y 장소 = 모양 s 한 가지, s 도 고름
//   - 같은 종류 퍽은 바꿔도 비용이 같음 → 상태는 종류별 개수로 셈
// 입력
//   - SolverInput: 노드가 기록부 · Cargo 로 만듦
//   - TransferCosts: 이동 시간 · 정차 · 팔 동작 시간
// 출력
//   - SolverResult: 동작 목록 · 총 시간 · Y 모양 · 재검증 순서
// 실행
//   - mission_core 라이브러리, transfer_planner 가 부름. 시험 test_transfer
// note: 동작 규칙
//   - Move: 다른 노드로 이동 + 정차 + LOOK. 그리퍼가 빌 때만 (퍽 쥔 채 주행 안 함)
//       박스는 남은 퍽이 있고 칸이 남을 때만, 장소는 그 장소에 놓을 퍽을 적재함에 실었을 때만
//   - Pick: 박스 정차 중 + 그리퍼 빔 → 그 박스에 남은 퍽 중 가장 가까운 것 = on_boxes 순서 첫 번째를 그리퍼로
//   - Store: 그리퍼 → 빈 칸, 어디서든 (정차 전이어도 = 출발 · 재계획 때 쥔 퍽)
//   - Retrieve: 칸 → 빈 그리퍼, 장소 정차 중 그 장소에 놓을 퍽만 → 다음 장소 퍽을 쥐고 주행 안 함
//   - Place: 장소 정차 중 + 그리퍼 퍽이 그 장소 그룹 조건에 맞고 남은 칸이 있음
//   - 끝: 박스 · 그리퍼 · 칸 모두 비고 세 장소 남은 칸 0 → 재검증 순회 + 출발점 복귀 비용
//       allow_leftover 면 세 장소 남은 칸 0 만 보면 됨
//   - 실제 ID 는 transfer_planner 가 붙임

#pragma once

#include "mission/common.hpp"

#include <array>
#include <optional>
#include <string>
#include <vector>

#include "mission/transfer_types.hpp"

namespace mission
{

struct SolverInput
{
  int position = kNodeStart;               // 지금 노드
  bool stopped = false;                    // 이미 그 노드에 정차 · LOOK 끝 (true 면 이동 없이 팔 작업 가능)
  std::vector<TransferPuck> on_boxes;      // 아직 박스 위 (box 1–6), 최대 12. 같은 박스 안 순서 = 집는 순서 (가까운 퍽부터)
  std::optional<TransferPuck> gripper;     // 그리퍼에 쥔 퍽
  std::vector<TransferPuck> in_cargo;      // 적재함에 실린 퍽
  int cargo_capacity = 12;                 // 적재함 칸 수
  std::array<char, kNumPlaces> place_group{{'?', '?', '?'}};   // L1–L3 그룹 'R' · 'Y' · 'G'
  std::array<int, kNumPlaces> slots_left{{0, 0, 0}};           // 장소별 아직 놓을 칸 수
  std::string yellow_shape;                // "" = 아직 안 정함. Y 장소에 하나라도 놓았으면 그 모양 ("cylinder" | "box")
  // true = 퍽이 칸보다 많아도 됨 (재계획: 일부 장소만 채울 때). 남는 퍽은 그대로 두거나 실은 채로 끝남
  bool allow_leftover = false;
};

enum class SolverAct : uint8_t { Move, Pick, Store, Retrieve, Place };

struct SolverStep
{
  SolverAct act = SolverAct::Move;
  int node = -1;                           // Move: 도착 노드, 나머지: 동작한 노드
  PuckKind kind = PuckKind::RedCylinder;   // Pick · Store · Retrieve · Place 의 퍽 종류
  int place = -1;                          // Place: 0–2
};

struct SolverResult
{
  double cost_s = 0.0;                     // 지금부터 복귀까지 총 시간 (재검증 포함)
  std::vector<SolverStep> steps;
  std::string yellow_shape;                // 정한 Y 모양
  std::array<int, kNumPlaces> verify_order{{0, 1, 2}};   // 재검증 방문 순서 (verify_tour 일 때)
  std::size_t expanded = 0;                // 펼친 상태 수 (계산량 확인용)
};

// 풀이. 입력이 잘못됐거나 (퍽 수 ≠ 남은 칸 합, 그룹 글자 · 칸 수 오류) 해가 없으면 nullopt + error
// max_states: 펼친 상태가 이보다 많으면 포기 (Jetson 시간 보호, 노드는 탐욕 해로 대신함)
std::optional<SolverResult> solve_transfer(
  const SolverInput & in, const TransferCosts & costs,
  std::string * error = nullptr, std::size_t max_states = 3000000);

}  // namespace mission
