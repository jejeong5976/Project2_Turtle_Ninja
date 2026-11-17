// common.hpp — mission 공통: planning 자료형 가져오기 · dice 단계 번호 약속
//
// 역할
//   - planning::Step 의 kind · ref 에 dice 용 뜻을 정함, 실행기는 이 칸을 해석 안 함
//       Custom 단계 kind = kReadBarcode · kPlanCollectBox · kPlanDelivery · kPlanVerify · kPlaceBox · kLookDwell · kRelookDwell · kBoxRelocate
//         · kReconcileCheck · kLyingBackoff
//       Custom 목표 kind = kTargetDiceCell, PLACE 목표 = 장소 place 의 cell 번째 칸
//       팔 LOOK 표시 kind = kPickRelook (PICK 앞 다시 보기, 실행기는 해석 안 함)
//       ref[0] = 박스 1–6, ref[1] = 장소 0–2 = L1–L3, ref[2] = 칸 번호, 먼 칸부터
//   - box_of · place_of · cell_of · is_custom · is_dice_cell 도우미
// 입력
//   - 없음, 약속 정의만
// 출력
//   - 없음
// 실행
//   - 헤더 전용, mission 계산 · 노드가 include
// note
//   - 의존 방향은 mission → planning 하나뿐

#pragma once

#include "planning/types.hpp"

namespace mission
{

using namespace planning;   // Step · Pose2D · Vec3 · set_error · kArm* 등

// Custom 단계 · Custom 목표 종류 (Step.kind)
enum StepKind : int
{
  kNoKind = 0,
  kReadBarcode = 1,       // /perception/task_text 대기 + 해석
  kPlanCollectBox = 2,    // 이 박스에서 본 퍽 → [STORE] PICK … 끼워 넣기
  kPlanDelivery = 3,      // 할당 · 운반 계획 → 배달 · 재검증 단계 (ref[0] < 1 이면 지금 자리에서 재계획)
  kPlanVerify = 4,        // 재검증: 장소 LOOK 뒤 새 관측으로 칸 · 그룹 판정
  kPlaceBox = 5,          // 장소 LOOK 뒤 박스 측정 기다려 칸 기준 확정 (place_box.hpp), 못 받으면 지도 값
  kLookDwell = 6,         // 옆 보기 LOOK 뒤 look_dwell_s 동안 그 자세로 검출 모음 (side_look_steps)
  kRelookDwell = 7,       // PICK 앞 다시 보기 LOOK 뒤 그 퍽 새 검출 기다림, 최대 relook_dwell_s (with_pick_relooks)
  kPickRelook = 8,        // 팔 LOOK 단계 표시: PICK 앞 다시 보기 (관측 구간 · 옆 보기 상태를 안 바꿈, starts_box_look 거짓)
  kBoxRelocate = 9,       // 박스 정밀 접근 실패 뒤 물러나 LOOK → 본 퍽으로 박스 중심 · 정차점 다시 잡고 그 박스 이동 다시 (box_relook)
  kTargetDiceCell = 10,   // PLACE 목표 = 장소 중심 + 주사위 눈 오프셋 (Target::Custom)
  // 11 · 12 = barcode_search kStepBcPlan · kStepBcDwell
  kTargetPutback = 21,    // PLACE 목표 = 쥔 퍽을 집은 박스 자리 (STORE 못 함 → 박스에 되돌려 놓기)
  kReconcileCheck = 22,   // 재고 자동 정합: 확인 LOOK (적재함 · 장소) 뒤 본 것으로 before · done · removed 고르고 재계획, 안 맞으면 FAULT
  kLyingBackoff = 23,     // 누운 퍽이 차체에 너무 가까움 → 곧게 뒤로 (odom, cmd_vel_nav) 뒤 같은 박스 앞 LOOK · Plan 다시 (lying_backoff.hpp)
};

// Step.ref 칸 이름
constexpr int kRefBox = 0;
constexpr int kRefPlace = 1;
constexpr int kRefCell = 2;

inline int & box_of(Step & s) {return s.ref[kRefBox];}
inline int box_of(const Step & s) {return s.ref[kRefBox];}
inline int & place_of(Step & s) {return s.ref[kRefPlace];}
inline int place_of(const Step & s) {return s.ref[kRefPlace];}
inline int & cell_of(Step & s) {return s.ref[kRefCell];}
inline int cell_of(const Step & s) {return s.ref[kRefCell];}

inline bool is_custom(const Step & s, StepKind k) {return s.type == StepType::Custom && s.kind == k;}
inline bool is_dice_cell(const Step & s) {return s.target == Target::Custom && s.kind == kTargetDiceCell;}
inline bool is_putback(const Step & s)
{
  return s.type == StepType::Arm && s.arm_task == kArmPlace && s.target == Target::Custom && s.kind == kTargetPutback;
}

}  // namespace mission
