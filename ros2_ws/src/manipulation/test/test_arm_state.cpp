// test_arm_state.cpp — 표시 · TF 용 팔 상태 (arm_state.hpp): 처짐 보정 · 그리퍼 연결 막대

#include <gtest/gtest.h>

#include <array>
#include <cmath>

#include "manipulation/arm_state.hpp"
#include "arm_kin_data.hpp"

using manipulation::ArmModel;

// 1) 처짐 보정 = 처짐 앞먹임(실물 계산기와 같다고 확인한 encoder_goal_q)의 역: 실제 → 엔코더 → 다시 실제
TEST(ArmState, SagCorrectionInvertsFeedforward)
{
  const ArmModel m = kin_data::model();
  for (const auto & c : kin_data::fk_cases) {
    const auto q_true = manipulation::ticks_to_q(m, c.ticks);
    const auto q_enc = manipulation::encoder_goal_q(m, q_true, 0.0);
    std::array<double, 5> team_enc, team_true;
    for (int j = 0; j < 5; ++j) {
      const double t_enc = m.jm[j].ref_tick + (q_enc[j] * 180.0 / M_PI - m.jm[j].q_deg_at_ref) / (m.jm[j].sign * 360.0 / 4096.0);
      const double t_true = m.jm[j].ref_tick + (q_true[j] * 180.0 / M_PI - m.jm[j].q_deg_at_ref) / (m.jm[j].sign * 360.0 / 4096.0);
      team_enc[j] = (t_enc - m.zero_tick[j]) * 2.0 * M_PI / 4096.0;
      team_true[j] = (t_true - m.zero_tick[j]) * 2.0 * M_PI / 4096.0;
    }
    const auto got = manipulation::sag_corrected(m, team_enc);
    for (int j = 0; j < 5; ++j) {EXPECT_NEAR(got[j], team_true[j], 2e-4) << "관절 " << j + 1;}   // 0.01° 안
    EXPECT_DOUBLE_EQ(got[0], team_enc[0]);   // J1 · J5 는 처짐 없음
    EXPECT_DOUBLE_EQ(got[4], team_enc[4]);
  }
}

// 2) NaN 이 있으면 손대지 않음
TEST(ArmState, SagCorrectionKeepsNan)
{
  const ArmModel m = kin_data::model();
  const std::array<double, 5> q{{0.0, std::nan(""), 0.1, 0.2, 0.0}};
  const auto got = manipulation::sag_corrected(m, q);
  EXPECT_TRUE(std::isnan(got[1]));
  EXPECT_DOUBLE_EQ(got[2], 0.1);
}

namespace
{
manipulation::GripperLinkage linkage()
{
  // config/gripper_linkage.yaml 과 같은 값
  manipulation::GripperLinkage g;
  g.axis = Eigen::Vector3d(0.6175023401994564, -0.2929746130211112, -0.7299703665035485).normalized();
  g.center = Eigen::Vector3d(0.0192560308655604, -0.009136042293253334, -0.04032044210465657);
  g.slide = Eigen::Vector3d(0.4286280631237271, 0.9034810362607972, -2.4898156407187062e-05);
  g.e = Eigen::Vector3d(0.4286280631237271, 0.9034810362607972, -2.4898156407187062e-05);
  g.t = Eigen::Vector3d(0.6595216776960382, -0.31287040966228513, 0.6834787951405346);
  g.finger_pin_q0 = {Eigen::Vector3d(0.0363703506666601, 0.0141787026983163, -0.0461597144674409),
    Eigen::Vector3d(0.0120217498033399, -0.0371383839983163, -0.0461606931325592)};
  g.crank_pin_cad = {Eigen::Vector3d(0.0358971613001141, -0.006273058317781398, -0.03835165620518844),
    Eigen::Vector3d(0.012494938285645588, -0.01668662131566264, -0.053968753023012815)};
  g.finger_pin_cad = {Eigen::Vector3d(0.044794333638568634, 0.03193514650562598, -0.04616020507754582),
    Eigen::Vector3d(0.0035977656070485357, -0.05489482491486852, -0.04616020492972646)};
  g.crank_radius = 0.015000000576719416;
  g.rod_length = 0.04000000033674506;
  g.cad_pin_alpha = 0.865888198610095;
  g.theta0 = 1.435359904426754;
  g.phi0 = {-1.4287932333071525, -1.4287933500825245};
  return g;
}
}  // namespace

// 3) 연결 막대: URDF 메시에서 구한 정답 (gripper_linkage 검증 예시)과 같음, q = 0 이면 모두 0
TEST(ArmState, LinkageMatchesGeometry)
{
  const auto g = linkage();
  struct Case {double q, crank, left, right;};
  const Case cases[] = {
    {-0.003, 0.5487568509631227, -0.3736772320719872, -0.3736772518533231},
    {0.0, 0.0, 0.0, 0.0},
    {0.005, -0.46851268092253706, 0.38097127779086337, 0.38097129658253004},
    {0.01, -0.8169540586365813, 0.7171644677896555, 0.7171645001627497},
    {0.0175, -1.2907442938272107, 1.250575261287316, 1.2505753091277954}};
  for (const auto & c : cases) {
    std::array<double, 3> out;
    ASSERT_TRUE(manipulation::solve_linkage(g, c.q, out)) << c.q;
    EXPECT_NEAR(out[0], c.crank, 1e-6) << c.q;
    EXPECT_NEAR(out[1], c.left, 1e-6) << c.q;
    EXPECT_NEAR(out[2], c.right, 1e-6) << c.q;
  }
}

// 4) 범위 밖 · NaN 은 계산 안 함
TEST(ArmState, LinkageRange)
{
  const auto g = linkage();
  std::array<double, 3> out;
  EXPECT_FALSE(manipulation::solve_linkage(g, 0.03, out));
  EXPECT_FALSE(manipulation::solve_linkage(g, std::nan(""), out));
}

// 5) 받침 근처는 처짐 보정 0, 멀어지면 1
TEST(ArmState, SagWeightAtRest)
{
  const std::array<double, 5> rest{{-0.0061, -1.4511, 0.9511, -0.1181, 0.0230}};
  EXPECT_DOUBLE_EQ(manipulation::sag_weight(rest, rest, 0.05, 0.15), 0.0);
  auto q = rest;
  q[1] += 0.10;
  EXPECT_NEAR(manipulation::sag_weight(q, rest, 0.05, 0.15), 0.5, 1e-9);
  q[1] += 1.0;
  EXPECT_DOUBLE_EQ(manipulation::sag_weight(q, rest, 0.05, 0.15), 1.0);
  const ArmModel m = kin_data::model();
  const auto c = manipulation::sag_corrected(m, rest, 0.0, 0.0);
  for (int j = 0; j < 5; ++j) {EXPECT_NEAR(c[j], rest[j], 1e-12);}
}
