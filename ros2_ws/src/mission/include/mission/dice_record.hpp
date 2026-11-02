// dice_record.hpp — 발제 v5 제출 기록 파일 signal.json · allocation.json 만들기
//
// 역할
//   - signal.json: 신호등 관측 결과 = 순서 · 횟수 · 점멸 시각 · 상태 · 문제. 기록이 끝났을 때 씀
//   - allocation.json: 전체 할당 = 장소 그룹 · 눈 수 · Y 모양 · 칸마다 놓을 퍽 ID · 위치, 퍽 12 개 출처. 첫 PLACE 전에 씀
//       재계획하면 revision 을 올려 다시 씀, history 에 이전 판 요약, 판마다 allocation_rev<N>.json 사본
// 입력
//   - SignalRecorder 결과, DicePlan · 퍽 기록부 · 적재함 정보, DiceMap
// 출력
//   - json 문자열 → 노드가 <record_dir>/run_<시각>/ 에 저장, record_dir 기본 ~/planning_logs
// 실행
//   - mission_core 라이브러리, mission_manager 가 부름. 시험 test_transfer
// note
//   - 기록 형식은 발제에 없어서 우리가 정함, 발제 p.5 '기록: signal.json, allocation.json'
//   - 퍽 정보 = 손목 LOOK 관측으로 기록부 · 적재함에 들어간 것. 좌표 = map + 경기장 mm

#pragma once

#include <array>
#include <string>
#include <vector>

#include "mission/dice.hpp"
#include "mission/dice_map.hpp"
#include "mission/signal_recorder.hpp"
#include "mission/common.hpp"

namespace mission
{

// 할당에 쓴 퍽 1 개 (어디서 봤고 지금 어디 있나)
struct AllocPuck
{
  std::string id;
  std::string color;
  std::string shape;
  std::string where;            // "box:B3" · "cargo:S4" · "gripper" · "skipped:B2" · "lost" · "placed:L1"
};

// 칸 하나에 놓을 퍽
struct AllocCell
{
  int place = -1;
  int cell = -1;                // 놓는 순서 번호 (먼 칸부터)
  double x = 0.0, y = 0.0;      // map [m]
  std::string puck_id;          // "" = 이번 계획에서 안 채움
};

struct AllocationRecord
{
  std::string barcode;
  DiceOrder order = DiceOrder::Ascending;
  std::string signal_sequence;
  SignalCounts counts;
  int revision = 1;
  std::string reason;           // "initial" · "replan: …"
  double stamp_s = 0.0;         // 경기 시작 뒤 [s]
  std::string yellow_shape;
  std::array<bool, 3> filled{{false, false, false}};   // 이번 계획으로 끝까지 채우는 장소
  std::array<int, 3> placed{{0, 0, 0}};                // 이미 놓은 수
  std::vector<AllocPuck> pucks;
  std::vector<AllocCell> cells;                        // 계획된 PLACE (장소 · 칸 · 퍽)
  std::array<int, 3> verify_order{{0, 1, 2}};
  double cost_s = 0.0;          // 예상 남은 시간
  std::vector<std::string> history;                    // 이전 판 한 줄 요약
};

// DELIVER 단계 목록의 PLACE → 칸 목록 (plan 은 apply_view_directions 를 거친 것)
std::vector<AllocCell> planned_cells(const std::vector<Step> & steps, const DicePlan & plan, const DiceMap & arena);

std::string allocation_to_json(const AllocationRecord & r, const DicePlan & plan, const DiceMap & arena);

std::string signal_to_json(const SignalRecorder & rec, double start_s, bool timed_out);

}  // namespace mission
