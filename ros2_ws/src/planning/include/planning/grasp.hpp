// grasp.hpp — 퍽 앞 자세 · 집기 축 · ArmTask 목표 방향 계산
//
// 역할
//   - approach_pose: 퍽 앞 정차 자세
//   - choose_grasp_axis: 퍽 긴축 어느 끝을 쥘지, 수직 45° 안이면 위쪽 끝, 더 누우면 로봇 쪽 끝
//   - grasp_point · grasp_orientation: 그리퍼 중심이 갈 점과 방향
//   - planar_from_pose: 퍽 자세 → 선/누움 · 바닥 방향
// 입력
//   - 로봇 · 퍽 위치, 퍽 자세 쿼터니언, yaw_known · 사각기둥 여부, 집기 오프셋
// 출력
//   - Pose2D 정차 자세, GraspAxis, 집기 점 Vec3, 방향 Quat
// 실행
//   - planning_core 라이브러리, Executor 가 PICK target 을 만들 때 씀. 시험 test_planning_core
// note
//   - 미션과 무관한 기하 계산
//   - 기준 프레임 base_link: 원점 = 로봇 몸 중심, z 위. 퍽 긴축 = 퍽 자세의 z 축

#pragma once

#include "planning/types.hpp"

namespace planning
{

// 퍽 앞에 설 자세: 로봇 → 퍽 방향으로, 퍽에서 standoff 만큼 떨어진 곳
Pose2D approach_pose(double robot_x, double robot_y, double puck_x, double puck_y, double standoff);

// 퍽 긴축 중 어느 끝을 쥘지 (퍽은 위아래 대칭 → 양 끝 모두 가능, 적재함엔 쥔 쪽 3 cm 기준으로 들어감)
//   - 기준 프레임 = base_link (원점 = 로봇 몸 중심, z 위)
//   - 방향 모름 (has_q = false): 똑바로 서 있다고 봄 → 위쪽 끝 (0, 0, 1)
//   - 긴축이 수직에서 45° 안 (|axis.z| ≥ cos 45°): 위쪽 끝
//   - 그보다 누움: 로봇 쪽 끝 (팔을 덜 뻗음). 축이 로봇 방향과 직각이면 그대로
//   - flipped = 퍽 자세의 z 축과 반대쪽 끝을 고름 → 보낼 자세를 x 축 180° 돌려야 z 축이 쥘 끝을 가리킴
struct GraspAxis
{
  Vec3 axis;            // 퍽 중심 → 쥘 끝 방향 단위 벡터
  bool flipped = false;
};
GraspAxis choose_grasp_axis(bool has_q, const Quat & q, const Vec3 & center);

// 그리퍼 중심이 갈 점 = 퍽 중심 + 쥘 끝 방향으로 grasp_offset (쥘 쪽 절반의 가운데)
Vec3 grasp_point(const Vec3 & center, const Vec3 & axis, double grasp_offset);

// 퍽 집기 자세 요약 (기준 프레임 = base_link, z 위)
//   - 퍽은 수평면(선반 · 바닥) 위 → 기울기는 0° 또는 90° 둘 중 하나 → x, y, z, θ + 선/누움이면 다 정해짐
//   - θ [rad]
//       선 사각기둥: 면 방향 (퍽 x 축의 수평 방향), [-π/4, π/4) (90° 주기)
//       누운 퍽    : 긴축의 수평 방향, [-π/2, π/2) (180° 주기)
//       선 원기둥  : 필요 없음 → 0, theta_known = false
//   - theta_known = false: 선 원기둥 · 면 방향 모르는 선 사각기둥 (yaw_known false) · 방향 자체를 모름 (has_q false)
struct PuckPlanar
{
  Vec3 center;
  double theta = 0.0;
  bool upright = true;
  bool theta_known = false;
};
// has_q = false 면 똑바로 섰다고 봄 (choose_grasp_axis 와 같은 약속). 선/누움 기준도 같음 (수직에서 45°)
PuckPlanar planar_from_pose(bool has_q, const Quat & q, const Vec3 & center, bool yaw_known, bool is_box);

// ArmTask target 방향 (항상 같은 규칙으로 만듦 → control 이 x 축을 믿을 수 있음)
//   z = 쥘 끝 방향 (choose_grasp_axis 결과)
//   x = 선 퍽 · 면 방향 앎: θ 방향 (사각기둥 면 법선)
//       선 퍽 · 면 방향 모름: 로봇(원점) → 퍽 수평 방향 (기본값, 원기둥은 어느 쪽이든 같음)
//       누운 퍽: 위쪽 (0, 0, 1) 을 z 에 수직으로 눌러 놓은 방향
//   y = z × x
Quat grasp_orientation(const PuckPlanar & p, const GraspAxis & ga);

// 회전 행렬 열 (x, z 축, 서로 수직 단위 벡터) → 쿼터니언. y = z × x
Quat quat_from_xz(const Vec3 & x, const Vec3 & z);

}  // namespace planning
