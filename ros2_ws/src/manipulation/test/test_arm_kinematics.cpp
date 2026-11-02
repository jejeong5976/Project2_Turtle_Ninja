// test_arm_kinematics.cpp — C++ 팔 기구학이 실물 집기 계산기(grasp_test · sag_model)와 같은 답을 내나
// 정답 = test/arm_kin_data.hpp (실물 계산기 결과)

#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include "manipulation/arm_kinematics.hpp"
#include "arm_kin_data.hpp"

using manipulation::Ticks;

namespace
{
int max_diff(const Ticks & a, const Ticks & b)
{
  int d = 0;
  for (int j = 0; j < 5; ++j) {d = std::max(d, std::abs(a[j] - b[j]));}
  return d;
}
}  // namespace

// 1) 틱 → 관절 → 틱 왕복
TEST(ArmKinematics, TickRoundTrip)
{
  const auto m = kin_data::model();
  const Ticks t{{4084, 1102, 2668, 1971, 2063}};
  EXPECT_EQ(manipulation::q_to_ticks(m, manipulation::ticks_to_q(m, t)), t);
}

// 2) FK 손끝 · 접근축 · 닫힘축, 중력 토크, 처짐 앞먹임 명령이 실물 계산기와 같음
TEST(ArmKinematics, ForwardAndSagMatchRealCalculator)
{
  const auto m = kin_data::model();
  for (const auto & c : kin_data::fk_cases) {
    const auto q = manipulation::ticks_to_q(m, c.ticks);
    const auto g = manipulation::arm_grip(m, q);
    for (int i = 0; i < 3; ++i) {
      EXPECT_NEAR(g.tip(i), c.tip[i], 1e-9);
      EXPECT_NEAR(g.app(i), c.app[i], 1e-9);
      EXPECT_NEAR(g.close(i), c.close[i], 1e-9);
    }
    const auto tau = manipulation::sag_torques(m, q, 0.017);
    for (int i = 0; i < 3; ++i) {EXPECT_NEAR(tau[i], c.tau[i], 1e-9);}
    EXPECT_EQ(manipulation::q_to_ticks(m, manipulation::encoder_goal_q(m, q, 0.017)), c.enc_held);
  }
}

// 3) 퍽 집기 자세 (위 40 · 10 · 쥐기 −30 mm): 실물 계산기와 ±2 틱. 실물 계산기가 못 푸는 자리는 C++ 도 못 풂
TEST(ArmKinematics, PickPosesMatchRealCalculator)
{
  const auto m = kin_data::model();
  const double top = 0.070 - 0.072 + 0.060;   // 박스 윗면 + 퍽 높이 (모델 base 기준, grasp_test 와 같음)
  for (const auto & c : kin_data::pick_cases) {
    std::string why;
    const auto P = manipulation::pick_poses(m, c.x, c.y, top, c.yaw, {40, 10, -30}, why);
    if (!c.ok) {
      EXPECT_TRUE(P.empty()) << c.x << ", " << c.y;
      continue;
    }
    ASSERT_EQ(P.size(), 3u) << why;
    for (int k = 0; k < 3; ++k) {
      EXPECT_LE(max_diff(P[k].ticks, c.ticks[k]), 2) << c.x << ", " << c.y << " " << P[k].name;
      EXPECT_LE(max_diff(P[k].enc, c.enc[k]), 2) << c.x << ", " << c.y << " " << P[k].name;
    }
  }
}

// 4) 팀 명령 rad = (틱 − 영점) · 2π/4096
TEST(ArmKinematics, TeamCommand)
{
  const auto m = kin_data::model();
  const auto c = manipulation::ticks_to_command(m, Ticks{{4084, 1102, 2668, 1971, 2063}});
  EXPECT_NEAR(c[0], -0.0061, 1e-4);
  EXPECT_NEAR(c[1], -1.4511, 1e-4);
  EXPECT_NEAR(c[2], 0.9511, 1e-4);
}

// 닫힘 방향을 직접 주면 손가락 방향(180° 주기)을 정확히 맞춤: 기본 방향 = 예전과 같은 해, 90° 돌리면 J5 가 약 90° 돎
TEST(ArmKinematics, ExplicitCloseAxisTurnsWrist)
{
  const auto m = kin_data::model();
  std::string why;
  const double def = manipulation::default_close_rad(m, 0.2102, 0.038, -4.0) * 180.0 / M_PI;
  const auto a = manipulation::pick_poses(m, 0.2102, 0.038, 0.058, -4.0, {40, 10, -30}, why);
  const auto b = manipulation::pick_poses(m, 0.2102, 0.038, 0.058, -4.0, {40, 10, -30}, why, def);
  const auto c = manipulation::pick_poses(m, 0.2102, 0.038, 0.058, -4.0, {40, 10, -30}, why, def - 90.0);
  ASSERT_EQ(a.size(), 3u);
  ASSERT_EQ(b.size(), 3u);
  ASSERT_EQ(c.size(), 3u) << why;
  for (int j = 0; j < 5; ++j) {EXPECT_EQ(a[2].ticks[j], b[2].ticks[j]);}
  EXPECT_NEAR(std::abs(c[2].ticks[4] - a[2].ticks[4]), 1024, 80);   // 90° = 1024 틱 (손끝 위치를 맞추느라 조금 다름)
}
