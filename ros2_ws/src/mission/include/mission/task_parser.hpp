// task_parser.hpp — 바코드 원문 → Task
//
// 역할
//   - 발제 v5 바코드 해석, Code128 · 출발 구역 벽에 하나. 뜻은 바코드 표 (maps/dice_barcodes.json)
//       ASC = 오름차순, 적은 횟수부터 L1 → L2 → L3 (P2-01 · P2-03)
//       DESC = 내림차순, 많은 횟수부터 (P2-02 · P2-04)
//       표에 없음 = 미등록 바코드 → std::nullopt + error
// 입력
//   - /perception/task_text 의 raw_text, 바코드 표 (load_barcode_table)
// 출력
//   - Task, dice_order 에 정렬 방향
// 실행
//   - mission_core 라이브러리, mission_manager 가 부름. 시험 test_mission_core
// note
//   - 해석 규칙은 dice.hpp parse_dice_barcode 하나에만 둠, 여기서는 Task 로 감쌈
//   - 앞뒤 공백 · 줄바꿈은 무시
//   - 정렬 대상은 R · Y · G 그룹의 신호 횟수, 퍽 ID · 장소 번호 아님
//   - 발제상 미등록 바코드는 오류 → 노드가 FAULT BARCODE_UNREGISTERED

#pragma once

#include <optional>
#include <string>

#include "mission/common.hpp"
#include "mission/dice.hpp"

namespace mission
{

// 바코드 1개를 해석한 task (발제 v5: 바코드 하나 = dice 정렬 방향 하나)
struct Task
{
  std::string task_id;                 // 중복 판단용 (바코드 원문 그대로, 예: "P2-01")
  DiceOrder dice_order = DiceOrder::Ascending;   // 신호 횟수 정렬 방향 (L1 → L2 → L3)
  std::string raw_text;                // 바코드 원문
};

// → Task (dice_order, task_id = raw_text = 바코드 원문)
std::optional<Task> parse_task(const std::string & raw_text, const BarcodeTable & table, std::string * error = nullptr);

}  // namespace mission
