// test_lying_backoff.cpp — 누운 퍽 뒤로 빼기 계산 (lying_backoff.hpp)

#include <gtest/gtest.h>

#include <cmath>

#include "mission/lying_backoff.hpp"

using mission::LyingBackoffConfig;
using mission::LyingCandidate;
using mission::StraightBackConfig;
using mission::lying_backoff_choice;
using mission::straight_back_cmd;

TEST(LyingBackoff, RealCaseNearLyingBacksToTableStart)
{
  // 누운 퍽 6 x 0.168 · 누운 8 x 0.172 → 가까운 6 기준 0.25 − 0.168 = 8.2 cm
  const auto c = lying_backoff_choice({{"6", 0.168, true}, {"8", 0.172, true}}, LyingBackoffConfig{});
  ASSERT_TRUE(c.has_value());
  EXPECT_EQ(c->id, "6");
  EXPECT_NEAR(c->back_m, 0.082, 1e-9);
}

TEST(LyingBackoff, UprightOrFarDoesNothing)
{
  const LyingBackoffConfig cfg;
  EXPECT_FALSE(lying_backoff_choice({{"1", 0.15, false}}, cfg).has_value());          // 선 퍽은 이 규칙 아님
  EXPECT_FALSE(lying_backoff_choice({{"2", 0.20, true}}, cfg).has_value());           // 경계 = 안 감
  EXPECT_FALSE(lying_backoff_choice({{"3", 0.27, true}, {"4", 0.12, false}}, cfg).has_value());
  EXPECT_FALSE(lying_backoff_choice({}, cfg).has_value());
  EXPECT_FALSE(lying_backoff_choice({{"5", NAN, true}}, cfg).has_value());
}

TEST(LyingBackoff, UprightUsesOwnLimit)
{
  LyingBackoffConfig cfg;
  cfg.up_near_x_m = 0.19;
  const auto c = lying_backoff_choice({{"1", 0.175, false}, {"2", 0.30, true}}, cfg);
  ASSERT_TRUE(c.has_value());
  EXPECT_EQ(c->id, "1");
  EXPECT_FALSE(c->lying);
  EXPECT_NEAR(c->back_m, 0.075, 1e-9);
  EXPECT_FALSE(lying_backoff_choice({{"3", 0.195, false}}, cfg).has_value());   // 선 퍽 한계 0.19 밖 (누운 한계 0.20 안이어도)
  // 누운 · 선 둘 다 걸리면 더 가까운 퍽 기준
  const auto d = lying_backoff_choice({{"4", 0.18, false}, {"5", 0.17, true}}, cfg);
  ASSERT_TRUE(d.has_value());
  EXPECT_EQ(d->id, "5");
  EXPECT_TRUE(d->lying);
  cfg.near_x_m = 0.0;   // 누운 쪽만 끔 → 선 퍽 규칙은 그대로
  EXPECT_TRUE(lying_backoff_choice({{"1", 0.175, false}}, cfg).has_value());
  EXPECT_FALSE(lying_backoff_choice({{"5", 0.17, true}}, cfg).has_value());
}

TEST(LyingBackoff, ClampedToMaxAndOffWhenZero)
{
  LyingBackoffConfig cfg;
  const auto c = lying_backoff_choice({{"7", 0.09, true}}, cfg);
  ASSERT_TRUE(c.has_value());
  EXPECT_NEAR(c->back_m, 0.12, 1e-9);   // 0.16 → 최대 0.12
  cfg.near_x_m = 0.0;
  EXPECT_FALSE(lying_backoff_choice({{"7", 0.09, true}}, cfg).has_value());
  cfg.near_x_m = 0.20;
  cfg.target_x_m = 0.195;               // 목표가 가까우면 (거리 < min) 안 감
  EXPECT_FALSE(lying_backoff_choice({{"7", 0.19, true}}, cfg).has_value());
}

TEST(LyingBackoff, StraightBackCommand)
{
  const StraightBackConfig cfg;
  const double yaw0 = 0.7;
  // 시작: 뒤로 (몸 −x), 최대 속도
  auto c = straight_back_cmd(1.0, 2.0, yaw0, 0.08, 1.0, 2.0, yaw0, cfg);
  EXPECT_FALSE(c.done);
  EXPECT_NEAR(c.vx, -0.10, 1e-9);
  EXPECT_NEAR(c.vy, 0.0, 1e-9);
  EXPECT_NEAR(c.wz, 0.0, 1e-9);
  // 5 cm 뒤로 감 · 왼쪽 1 cm 어긋남 · 2° 돎 → 남은 3 cm 느리게, 오른쪽으로, 되돌려 돎
  const double bx = 1.0 - 0.05 * std::cos(yaw0) - 0.01 * std::sin(yaw0);
  const double by = 2.0 - 0.05 * std::sin(yaw0) + 0.01 * std::cos(yaw0);
  c = straight_back_cmd(1.0, 2.0, yaw0, 0.08, bx, by, yaw0 + 2.0 * M_PI / 180.0, cfg);
  EXPECT_NEAR(c.traveled, 0.05, 1e-9);
  EXPECT_NEAR(c.lateral, 0.01, 1e-9);
  EXPECT_NEAR(c.remaining, 0.03, 1e-9);
  EXPECT_LT(c.vx, -0.03 + 1e-6);
  EXPECT_GT(c.vx, -0.05);
  EXPECT_LT(c.vy, 0.0);
  EXPECT_LT(c.wz, 0.0);
  // 도착 (남은 ≤ 4 mm) · 넘어감 → done, 명령 0
  c = straight_back_cmd(0.0, 0.0, 0.0, 0.08, -0.077, 0.0, 0.0, cfg);
  EXPECT_TRUE(c.done);
  EXPECT_EQ(c.vx, 0.0);
  c = straight_back_cmd(0.0, 0.0, 0.0, 0.08, -0.10, 0.0, 0.0, cfg);
  EXPECT_TRUE(c.done);
}

TEST(LyingBackoff, SimulatedMoveReachesGoal)
{
  // 몸 속도를 그대로 따르는 차체 (20 Hz) → 목표 ±5 mm, 옆 · 방향 어긋남은 줄어듦
  const StraightBackConfig cfg;
  double x = 0.0, y = 0.0, th = 0.3;
  const double x0 = x, y0 = y, th0 = th;
  th += 0.03;   // 처음부터 1.7° 돌아 있음
  y += 0.005;
  int n = 0;
  for (; n < 200; ++n) {
    const auto c = straight_back_cmd(x0, y0, th0, 0.08, x, y, th, cfg);
    if (c.done) {break;}
    x += 0.05 * (std::cos(th) * c.vx - std::sin(th) * c.vy);
    y += 0.05 * (std::sin(th) * c.vx + std::cos(th) * c.vy);
    th += 0.05 * c.wz;
  }
  const auto c = straight_back_cmd(x0, y0, th0, 0.08, x, y, th, cfg);
  EXPECT_TRUE(c.done);
  EXPECT_NEAR(c.traveled, 0.08, 0.006);
  EXPECT_LT(std::fabs(c.dyaw), 0.03);
  EXPECT_LT(n, 40);   // 2 s 안
}

// 측정을 못 믿는 선 퍽은 누운 퍽 기준 (near_x_m) 으로 뒤로 (예: 선 퍽 x 0.187, 높이 2 mm 로 읽힘)
TEST(LyingBackoff, BadUprightUsesLyingLimit)
{
  LyingBackoffConfig cfg;
  cfg.up_near_x_m = 0.10;
  EXPECT_FALSE(lying_backoff_choice({{"1", 0.187, false, false}}, cfg).has_value());   // 믿을 만한 선 퍽은 up_near_x_m 기준 그대로
  const auto c = lying_backoff_choice({{"1", 0.187, false, true}, {"0", 0.26, false, true}}, cfg);
  ASSERT_TRUE(c.has_value());
  EXPECT_EQ(c->id, "1");
  EXPECT_TRUE(c->bad);
  EXPECT_FALSE(c->lying);
  EXPECT_NEAR(c->back_m, 0.063, 1e-9);
  EXPECT_FALSE(lying_backoff_choice({{"1", 0.20, false, true}}, cfg).has_value());     // 경계 = 안 감
  const auto l = lying_backoff_choice({{"8", 0.15, true, true}}, cfg);                 // 누운 퍽은 bad 와 무관하게 near_x_m
  ASSERT_TRUE(l.has_value());
  EXPECT_FALSE(l->bad);
}
