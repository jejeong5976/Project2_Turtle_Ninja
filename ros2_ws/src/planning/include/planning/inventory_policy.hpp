// inventory_policy.hpp — 팔 작업 결과로 재고가 불확실해졌는지 판정 · 사람 확인으로 정합
//
// 역할
//   - 재고를 바꾸거나 퍽을 쥔 채 움직이는 팔 작업 = PICK · STORE · RETRIEVE · PLACE · LOOK_RACK
//   - 이 작업이 수락된 뒤 성공으로 안 끝남 (실패 · 취소 · 시간 초과 · 결과 모름) → 재고 불확실
//       퍽이 그리퍼 · 칸 · 바닥 어디 있는지 모름 → 자동 재시도 · 재계획 금지, 실행기가 FAULT 사건을 냄
//   - 보내기 전 실패 · 서버의 목표 거절 = 팔이 안 움직임 → 재고 확실, 기존 재시도 · 재계획 그대로
//   - 보내기 전 확인 arm_preflight_ok: 단계의 퍽 · 칸이 그리퍼 · 칸 기록과 맞을 때만 팔 명령 (안 맞으면 FAULT)
//   - 최종 성공 반영 apply_arm_success: 전환 + 같은 단계 불확실 표시 해소를 한 번에, 기록과 안 맞으면 아무것도 안 바꿈
//       → 실행기가 불확실 표시 + FAULT (최종 성공이어도 기록이 모순이면 추가 조작 금지)
//   - 사람 확인 정합 resolve_doubt (세 가지 중 실물에 맞는 것을 사람이 고름)
//       Before  = 실물이 그 단계 전과 같음 → 기록 그대로
//       Done    = 실물이 그 단계 성공과 같음 → 성공 전환을 한 번 반영 (LOOK_RACK 은 전환이 없어 거절)
//       Removed = 사람이 그 퍽을 로봇 (그리퍼 · 그 칸) 에서 뺌 → 기록에서 지움, 퍽 상태 removed
// 입력
//   - 팔 작업 번호, 끝난 모양 ArmEnd, 재고 기록, 사람이 고른 정합
// 출력
//   - 불확실 여부, 불확실 표시, 정합 뒤 재고 · 퍽 기록부 상태
// 실행
//   - planning_core 라이브러리, Executor 가 씀. 시험 test_planning_core
// note
//   - ROS 를 모르는 순수 C++
//   - 실물 증거 없이 lost · delivered 로 정하지 않음: delivered 는 control 최종 성공 또는 사람 Done 확인일 때만
//   - HOME 성공은 팔 자세만 확인 → 불확실 표시를 안 지움

#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include "planning/cargo.hpp"

namespace planning
{

// 팔 작업 한 번이 끝난 모양 (실행기가 정함)
enum class ArmEnd
{
  NotSent,     // 보내기 전 실패 (목표 계산 · ready · 서버 · 취소 결과 대기)
  Rejected,    // 서버가 목표를 거절 (팔 안 움직임)
  Succeeded,   // 최종 성공
  Failed,      // 수락 뒤 최종 결과가 성공 아님 (실패 · 취소 · 중단, 어디까지 했는지 모름)
  Unknown,     // 보냈는데 최종 결과를 못 받음 (수락 응답 전 취소 · 결과 없음 · FAULT 로 취소)
  Mismatch,    // 최종 성공인데 재고 기록과 안 맞아 반영 못 함 (실물 · 기록 모순)
};

// control 최종 성공 하나를 재고에 반영한 결과
struct ArmSuccessApplied
{
  bool changed = false;                   // 재고 · 표시가 바뀜 (저장 필요)
  bool doubt_cleared = false;             // 이 단계가 불확실 원인이었고 지움
  std::string puck_state;                 // 퍽 기록부에 넣을 상태 ("" = 안 바꿈)
  std::optional<CargoItem> placed;        // PLACE: 놓은 퍽
};

// 사람이 실물을 보고 고르는 정합
enum class DoubtFix { Before, Done, Removed };

const char * doubt_fix_name(DoubtFix f);

// 재고를 바꾸거나 퍽을 쥔 채 움직이는 팔 작업인가
bool arm_task_risks_inventory(uint8_t task);

// 이 끝난 모양이면 재고가 불확실한가 (= 자동 재시도 · 재계획 금지)
bool inventory_doubt_after(uint8_t task, ArmEnd end);

// control 이 PICK 빈손을 그리퍼 실측으로 확인했다는 결과 글 표시 (보통 맨 앞, ArmRunner 가 "팔 작업 중단: " 을 앞에 붙여도 찾음)
//   control 약속: 퍽이 박스에 남는 닫기의 빈손에서만 붙임 = 박스 첫 쥐기 · 박스 안 옮겨 놓고 다시 쥐기 (re_grasp 등)
//   받침 세우기 다시 쥐기 (park_ 닫기, park_re_down) 빈손은 토큰 금지: 퍽이 로봇 위 받침에 있음 → 토큰 없이 예전대로
//   FAULT INVENTORY_UNCERTAIN (사람 확인). 실행기는 어느 닫기였는지 모름 → 이 약속을 control 이 지켜야 함
inline constexpr const char * kEmptyGraspToken = "EMPTY_GRASP";

// PICK 이 수락 뒤 빈손 확인으로 끝남 (결과 글에 EMPTY_GRASP) → 퍽은 박스에 그대로 · 그리퍼 빔 = 재고 확실
//   실행기 empty_grasp_skip 이 켜졌을 때만 씀 → 재고 불확실 대신 그 퍽 건너뜀. 그 밖 작업 · 끝난 모양 · 표시 없는 실패는 false
//   (받침 다시 쥐기 빈손은 control 이 토큰을 안 붙임 → 여기 false → 불확실 FAULT, 위 약속)
bool empty_grasp_confirmed(uint8_t task, ArmEnd end, const std::string & message);

// control 이 STORE 넣기 걸림 뒤 쥔 퍽을 박스에 되돌려 놓았다는 결과 글 표시 (store_recover, 11-11):
//   "STORE_RETURNED_TO_BOX x y" — 내려놓은 자리 base_link [m]. 손은 빔 (control 이 쥔 상태를 지움), 퍽은 박스 그 자리
//   control 약속: 퍽을 박스에 내려놓고 (열고) 다시 쥐기 전에 끝난 STORE 에서만 붙임 (칸 안 · 쥔 채 실패는 토큰 없음 → 예전대로 불확실 FAULT)
inline constexpr const char * kStoreReturnedToken = "STORE_RETURNED_TO_BOX";
// STORE 가 수락 뒤 실패로 끝났고 결과 글에 토큰 + 숫자 두 개가 있으면 true (x · y = base_link 자리). 그 밖은 false
bool store_returned_confirmed(uint8_t task, ArmEnd end, const std::string & message, double * x = nullptr, double * y = nullptr);
// 그 퍽을 그리퍼 기록에서 지움 (박스로 되돌아감). 그리퍼 기록 = 그 퍽일 때만 (아니면 false + error, 재고 그대로)
//   puck_state = 퍽 기록부에 넣을 상태 (on_field). 불확실 표시는 안 건드림
bool apply_store_returned(Cargo & cargo, const std::string & puck_id, std::string * puck_state, std::string * error = nullptr);

// 불확실 표시 만들기. item = 그때 기록상 그 퍽 (PICK 이면 pick_item, STORE · PLACE · LOOK_RACK = 그리퍼, RETRIEVE = 그 칸)
CargoDoubt make_doubt(const Cargo & cargo, uint8_t task, const std::string & puck_id, const std::string & slot,
  const CargoItem & pick_item, const std::string & step_name, const std::string & reason);

// 늦게 온 최종 성공이 불확실 원인과 같은 단계인가
bool doubt_matches(const CargoDoubt & d, uint8_t task, const std::string & puck_id, const std::string & slot);

// 팔 작업 보내기 전 재고 확인 (단계 퍽 ID 가 비어도 실패). 안 맞으면 false + error → 팔 명령 없이 FAULT
//   PICK      : 그리퍼 빔, 그 퍽이 칸 기록에 없음 (같은 ID 두 번 적재 방지)
//   STORE     : 그리퍼 = 그 퍽, 칸이 있고 빔
//   LOOK_RACK : 그리퍼 = 그 퍽 (STORE 직전, 퍽 쥔 채 움직임)
//   RETRIEVE  : 그리퍼 빔, 칸 = 그 퍽
//   PLACE     : 그리퍼 = 그 퍽
//   그 밖 (HOME · LOOK 류) 은 확인 없음 → true
bool arm_preflight_ok(const Cargo & cargo, uint8_t task, const std::string & puck_id, const std::string & slot,
  std::string * error = nullptr);

// control 최종 성공 → 재고 전환 + 같은 단계 불확실 표시 해소 (정상 · 시간 초과 뒤 · FAULT 뒤 늦은 성공 · stop 선반영 공통)
//   기록이 보내기 전 상태 (arm_preflight_ok) 가 아니거나 전환이 안 맞으면 재고 · 표시 그대로 false + error
//   다른 단계의 불확실 표시는 안 건드림. 전환 없는 작업 (LOOK_RACK 등) 은 표시 해소만
bool apply_arm_success(Cargo & cargo, uint8_t task, const std::string & puck_id, const std::string & slot,
  const CargoItem & pick_item, ArmSuccessApplied * out, std::string * error = nullptr);

// 사람 확인 정합 → 재고에 반영하고 불확실 표시를 지움
//   실패 (표시 없음 · 기록과 안 맞는 전환) 면 재고 · 표시 그대로 + error
//   puck_state = 퍽 기록부에 넣을 상태 ("" = 안 바꿈), puck_id = 그 퍽
bool resolve_doubt(Cargo & cargo, DoubtFix fix, std::string * puck_id, std::string * puck_state,
  std::string * error = nullptr);

}  // namespace planning
