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
// slot: left · right (J1 ± side_deg) · far · near, 그 밖이면 front 그대로. 기울임 [°]: side/far 양수 = J4 감소 (먼 곳), near 양수 = J3 증가 (가까운 곳).
// J4 는 정면 LOOK 에서 이미 모델 상한이라 near 는 J3 를 씀. 기울임이 안 되면 기울이지 않은 보기 + fallback 에 이유 (true).
// 기울임 0 이면 계산 · 반올림 없이 점 그대로. false = front 관절 5 개 아님 · side_deg 가 0 ~ 90° 밖 · 옆 보기가 관절 범위 밖.
bool make_look_variant(const PathNode & front, const std::string & slot, const LookOptions & options,
  const ArmModel * model, PathNode & out, std::string & fallback);
// variants 중 관절별 오차 ≤ tol [rad] 이고 지금 관절 now 에 (제곱합으로) 정면보다 가까운 것 → out, true.
bool match_look_variant(const PathNode & front, const std::vector<PathNode> & variants,
  const std::vector<double> & now, double tol, PathNode & out);
// 변형 → 정확한 정면 LOOK (멈춤) → front_home (home_from_look). front_home 의 경유점 순서 · 그리퍼는 그대로.
ArmPath look_home_bridge(const PathNode & variant, const PathNode & front, const ArmPath & front_home);
}  // namespace manipulation
