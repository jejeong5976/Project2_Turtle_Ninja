// common.hpp — mission 공통: planning 자료형 가져오기 · dice 단계 번호 약속
//
// 역할
//   - planning::Step 의 kind · ref 에 dice 용 뜻을 정함, 실행기는 이 칸을 해석 안 함
//       Custom 단계 kind = kReadBarcode · kPlanCollectBox · kPlanDelivery · kPlanVerify
//       Custom 목표 kind = kTargetDiceCell, PLACE 목표 = 장소 place 의 cell 번째 칸
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
  kTargetDiceCell = 10,   // PLACE 목표 = 장소 중심 + 주사위 눈 오프셋 (Target::Custom)
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

}  // namespace mission
