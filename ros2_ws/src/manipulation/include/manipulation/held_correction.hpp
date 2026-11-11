// held_correction.hpp — STORE 직전 손 안 퍽 측정의 신선도·깊이 차이·보정 부호 (ROS 없음)
// 입력: wrist_link 실측과 기준/쥔 깊이, 출력: 쓰는 경우에만 기준 경로에 더할 base XY
#pragma once
#include <string>
#include <vector>
#include <Eigen/Core>

namespace manipulation
{
struct HeldSample
{
  std::string id, frame;
  double stamp{0.0};
  Eigen::Vector3d p{Eigen::Vector3d::Zero()}, n{Eigen::Vector3d::UnitZ()};
};
struct HeldEstimate
{
  bool ok{false};
  std::string id, why;
  Eigen::Vector3d p{Eigen::Vector3d::Zero()}, n{Eigen::Vector3d::UnitZ()};
  int count{0};
};
// 최근 0.8초의 서로 다른 촬영 3장, 지정 ID 또는 유일한 ID만 씀. 불명/흔들림은 기존 동작으로 넘김.
HeldEstimate fresh_held(const std::vector<HeldSample> & samples, const std::string & expected_id,
  double after, double now, const Eigen::Vector3d & wrist_up);
struct HeldReference
{
  Eigen::Vector3d p{Eigen::Vector3d::Zero()}, n{Eigen::Vector3d::UnitZ()};
  double depth_mm{0.0};
};
struct HeldCorrection
{
  bool ok{false};
  Eigen::Vector2d xy{Eigen::Vector2d::Zero()};
  Eigen::Vector3d residual{Eigen::Vector3d::Zero()};
  std::string why;
};
// nominal depth를 빼야 50→30 mm 파지 차이 20 mm를 미끄러짐으로 세지 않음.
// correction = -R_release * ((p_cur-depth_cur*n_cur) - (p_ref-depth_ref*n_ref)). XY만 적용함.
HeldCorrection held_correction(const HeldEstimate & current, const HeldReference & reference,
  double current_depth_mm, const Eigen::Vector3d & wrist_up, const Eigen::Matrix3d & release_rotation,
  double max_mm);
}  // namespace manipulation
