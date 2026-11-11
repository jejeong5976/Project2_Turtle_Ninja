// look_variant.hpp — 정면 LOOK 기준 보기 변형 · 복귀 연결 (순수 계산)
#pragma once
#include "manipulation/arm_kinematics.hpp"
#include "manipulation/arm_path.hpp"

namespace manipulation
{
struct LookOptions
{
  double side_deg{18.0};
  double side_tilt_deg{0.0};
  double far_tilt_deg{0.0};
  double near_tilt_deg{0.0};
};
// side/far 양수 = J4 감소 (먼 곳), near 양수 = J3 증가 (가까운 곳).
// J4 는 정면 LOOK 에서 이미 모델 상한이라 near 는 J3 를 씀. 새 기울임 실패는 기존 보기로 되돌림.
// 0 은 계산/반올림 없이 기존 점을 그대로 보존. side_deg 검사는 기존 left/right 와 같음.
bool make_look_variant(const PathNode & front, const std::string & slot, const LookOptions & options,
  const ArmModel * model, PathNode & out, std::string & fallback);
// 현재 파라미터와 이전에 계획한 점 중 실제 관절에 가장 가까운 보기 변형. 정면보다 가까울 때만 선택.
bool match_look_variant(const PathNode & front, const std::vector<PathNode> & variants,
  const std::vector<double> & now, double tol, PathNode & out);
// 변형 → 정확한 정면 LOOK (멈춤) → 기존 home_from_look. 기존 경유점 순서/그리퍼를 보존.
ArmPath look_home_bridge(const PathNode & variant, const PathNode & front, const ArmPath & front_home);
}  // namespace manipulation
