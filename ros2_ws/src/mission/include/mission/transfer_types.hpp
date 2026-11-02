// transfer_types.hpp — 퍽 운반 계획 공통 자료형
//
// 역할
//   - 운반 계획에 쓰는 노드 번호 · 퍽 종류 · 비용 · 입력 · 결과 자료형
// 입력
//   - 없음, 자료형 정의만
// 출력
//   - 없음
// 실행
//   - mission_core 라이브러리, transfer_solver · transfer_planner 가 씀. 시험 test_transfer
// note
//   - 노드 번호: 0 = 출발점, 1–6 = 시작 박스 정차점 박스 번호 그대로, 7–9 = L1–L3 정차점
//   - 퍽 종류 4 가지 = 빨강 · 초록 × 원기둥 · 사각기둥. 계획은 종류 단위로 풀고 실제 ID · 칸은 펼칠 때 붙임
//   - 비용 단위는 전부 초
//   - 설계 docs/planning/transfer_planner_design.md

#pragma once

#include "mission/common.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace mission
{

// ---------------------------------------------------------------- 노드
constexpr int kNodeStart = 0;
constexpr int kNumBoxes = 6;
constexpr int kNumPlaces = 3;
constexpr int kNumNodes = 1 + kNumBoxes + kNumPlaces;

inline int place_node(int place) {return 1 + kNumBoxes + place;}     // 0–2 → 7–9
inline bool is_box_node(int n) {return n >= 1 && n <= kNumBoxes;}
inline bool is_place_node(int n) {return n > kNumBoxes && n < kNumNodes;}
inline int node_place(int n) {return n - 1 - kNumBoxes;}            // 7–9 → 0–2
std::string node_name(int n);                                       // "S" · "B3" · "L2"

// ---------------------------------------------------------------- 퍽 종류
enum class PuckKind : uint8_t { RedCylinder = 0, RedBox = 1, GreenCylinder = 2, GreenBox = 3 };
constexpr int kNumKinds = 4;

// 색 "red" | "green", 모양 "cylinder" | "box" (dice.hpp 와 같은 문자열). 그 밖이면 nullopt
std::optional<PuckKind> puck_kind(const std::string & color, const std::string & shape);
inline bool kind_is_red(PuckKind k) {return static_cast<int>(k) < 2;}
inline bool kind_is_box(PuckKind k) {return static_cast<int>(k) % 2 == 1;}
const char * kind_color(PuckKind k);
const char * kind_shape(PuckKind k);

// 계획에 넣는 퍽 1개
struct TransferPuck
{
  std::string id;               // AprilTag ID
  PuckKind kind = PuckKind::RedCylinder;
  int box = 0;                  // 박스 위면 1–6, 실려 있으면 0
};

// ---------------------------------------------------------------- 비용
struct TransferCosts
{
  std::array<std::array<double, kNumNodes>, kNumNodes> travel_s{};   // travel_s[a][b] 이동 시간
  double stop_s = 20.0;         // 정차 1회 (감속 · 도킹 · 바퀴 대각선 고정 · 해제)
  double look_s = 5.0;          // 정차마다 손목 LOOK 1회
  double pick_s = 30.0;
  double place_s = 30.0;
  double store_s = 15.0;        // 그리퍼 → 칸
  double retrieve_s = 15.0;     // 칸 → 그리퍼
  bool verify_tour = true;      // 마지막에 세 장소 재검증 순회 (false = 장소 완성 때 바로 검증, 순회 없음)
};

// ---------------------------------------------------------------- 정차 · 팔 동작
// 한 정차 안의 팔 동작은 이어서 하고 마지막에만 HOME (control 세션 약속)
enum class ArmOp : uint8_t { Look, Pick, Store, Retrieve, Place };

struct ArmAction
{
  ArmOp op = ArmOp::Look;
  std::string puck_id;          // Pick · Store · Retrieve · Place
  std::string color;            // 기록 · 로그용 (Pick · Store · Retrieve · Place)
  std::string shape;
  std::string slot;             // Store · Retrieve: 적재함 칸 이름
  int box = -1;                 // Look(박스) · Pick: 1–6
  int place = -1;               // Look(장소 태그) · Place: 0–2
  int cell = -1;                // Place: 그 장소에서 몇 번째로 놓는 칸 (0 부터, 먼 칸부터 순서 목록의 번호)
};

struct Stop
{
  int node = kNodeStart;
  bool move = true;             // false = 이동 없이 지금 정차 중인 자리에서 이어서 함
  std::vector<ArmAction> actions;
  std::string name;             // 상태 표시용
};

}  // namespace mission
