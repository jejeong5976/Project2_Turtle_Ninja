// arm_state.hpp — 표시 · TF 용 팔 상태 계산 (순수 C++, test/test_arm_state.cpp)
// =====================================================================
// ① 처짐: 엔코더는 팔이 중력으로 내려간 만큼을 못 봄 → 실제 관절 = 엔코더 + c · τ(실제) (J2 · J3 · J4, arm_kinematics 와 같은 모델)
//    손목 카메라 외부 보정(URDF camera_joint)은 처짐을 보정한 손목 기준 → TF 가 실제 자세여야 카메라 위치가 맞음
// ② 그리퍼 연결 막대: 손가락 위치 하나로 크랭크 · 양쪽 막대 관절각이 정해짐 (크랭크 반경 · 막대 길이로 닫힘 조건)
//    config/gripper_linkage.yaml (URDF 메시 구멍축에서 뽑은 값)

#pragma once

#include <array>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "manipulation/arm_kinematics.hpp"

namespace manipulation
{

// 팀 명령 rad (관절 1..5, 엔코더) → 처짐을 보정한 실제 rad. 측정이 NaN 이면 그대로 돌려줌
//   weight: 0 = 보정 없음 (받침에 얹힘 — 받침대가 무게를 받침), 1 = 전부
std::array<double, 5> sag_corrected(const ArmModel & m, const std::array<double, 5> & team_rad, double held_kg = 0.0,
  double weight = 1.0);

// 받침(접은 자세)에서 떨어진 정도 → 처짐 보정 비율: J2 · J3 · J4 중 가장 먼 관절이 near 안이면 0, far 밖이면 1, 사이는 직선
double sag_weight(const std::array<double, 5> & team_rad, const std::array<double, 5> & rest, double near, double far);

struct GripperLinkage
{
  Eigen::Vector3d axis, center, slide, e, t;           // 힌지축 · 크랭크 중심 · 손가락 미끄럼축 · 힌지 평면 기저 (손목 좌표)
  std::array<Eigen::Vector3d, 2> finger_pin_q0;       // 손가락 핀 (q = 0), [왼, 오른]
  std::array<Eigen::Vector3d, 2> crank_pin_cad, finger_pin_cad;
  double crank_radius{0.015}, rod_length{0.040};
  double cad_pin_alpha{0.0}, theta0{0.0};
  std::array<double, 2> phi0{};
  double q_min{-0.003}, q_max{0.0175};
};

// 손가락 위치 q [m] → 크랭크 · 왼쪽 막대 · 오른쪽 막대 관절각 [rad]. 범위 밖 · 닫힘 불가면 false
bool solve_linkage(const GripperLinkage & g, double q, std::array<double, 3> & out);

}  // namespace manipulation
