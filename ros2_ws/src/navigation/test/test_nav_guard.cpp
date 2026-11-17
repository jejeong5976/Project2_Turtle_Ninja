// test_nav_guard.cpp — 주행 노드 덧붙임 시험 (ROS 없음): 경로 앞 막힘 · 막힘 기다리기 · 제자리 회전 반전 제한
//   실행: colcon build → ./build/navigation/test_nav_guard

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <vector>

#include "navigation/nav_guard.hpp"

using navigation::guard::ahead_blocked;
using navigation::guard::BlockWait;
using navigation::guard::RevLimit;
using navigation::rl::RlPath;
using navigation::rl::Vec2;

namespace
{
const RlPath kLine({{0.0, 0.0}, {2.0, 0.0}});
constexpr double kAhead = 0.5, kHalf = 0.13, kRmin = 0.20;

// 사람 다리 하나 (반지름 5 cm 원 앞쪽 반원 점)
std::vector<Vec2> leg(Vec2 c, double r = 0.05)
{
  std::vector<Vec2> pts;
  for (int i = -4; i <= 4; ++i) {
    const double a = M_PI + i * M_PI / 8;
    pts.push_back({c.x + r * std::cos(a), c.y + r * std::sin(a)});
  }
  return pts;
}
}  // namespace

TEST(Ahead, PersonOnPath)
{
  const auto a = ahead_blocked(kLine, 0.0, leg({0.45, 0.0}), kAhead, kHalf, kRmin);
  EXPECT_GE(a.n, 2);
  EXPECT_NEAR(a.at, 0.40, 0.01);                        // 다리 앞면 (0.45 − 0.05)
}

TEST(Ahead, IgnoresSideWallBehindNearAndFar)
{
  std::vector<Vec2> wall;                               // 경로 옆 15 cm 벽 (NavFn 은 중심을 내접 0.17 m 밖으로 냄, 지도 오차 2 cm)
  for (double x = -1.0; x <= 2.0; x += 0.02) {wall.push_back({x, 0.15});}
  EXPECT_EQ(ahead_blocked(kLine, 0.0, wall, kAhead, kHalf, kRmin).n, 0);
  EXPECT_EQ(ahead_blocked(kLine, 0.5, leg({0.2, 0.0}), kAhead, kHalf, kRmin).n, 0);    // 로봇 뒤
  EXPECT_EQ(ahead_blocked(kLine, 0.0, {{0.19, 0.0}, {0.19, 0.02}}, kAhead, kHalf, kRmin).n, 0);   // r_min 안 (PolygonStop 몫)
  EXPECT_EQ(ahead_blocked(kLine, 0.0, leg({0.9, 0.0}), kAhead, kHalf, kRmin).n, 0);    // ahead 밖
  EXPECT_EQ(ahead_blocked(kLine, 0.0, leg({0.45, 0.0}), 0.0, kHalf, kRmin).n, 0);      // ahead 0 = 끔
}

TEST(Ahead, BoxFaceAtDeparture)
{
  // 박스 면 1 cm (몸 중심 +0.17 m) 에서 뒤 · 옆으로 출발하는 경로: 박스 면은 안 걸림
  std::vector<Vec2> face;
  for (double y = -0.1; y <= 0.1; y += 0.01) {face.push_back({0.17, y});}
  const RlPath back({{0.0, 0.0}, {-1.0, 0.0}});
  const RlPath side({{0.0, 0.0}, {0.0, 1.0}});
  EXPECT_EQ(ahead_blocked(back, 0.0, face, kAhead, kHalf, kRmin).n, 0);
  EXPECT_EQ(ahead_blocked(side, 0.0, face, kAhead, kHalf, kRmin).n, 0);
}

TEST(Ahead, CurvedPathEndClipped)
{
  const RlPath turn({{0.0, 0.0}, {0.3, 0.0}, {0.3, 0.6}});
  EXPECT_GE(ahead_blocked(turn, 0.0, leg({0.3, 0.2}), kAhead, kHalf, kRmin).n, 2);    // 꺾인 뒤 경로 위
  EXPECT_EQ(ahead_blocked(turn, 0.0, leg({0.6, 0.0}), kAhead, kHalf, kRmin).n, 0);    // 꺾기 전 직진 연장선 (경로 아님)
  const RlPath shortp({{0.0, 0.0}, {0.3, 0.0}});
  EXPECT_EQ(ahead_blocked(shortp, 0.0, leg({0.55, 0.0}), kAhead, kHalf, kRmin).n, 0);  // 경로 끝 너머
}

TEST(BlockWait, OffNeverHolds)
{
  BlockWait w(0.0);
  for (int i = 0; i < 100; ++i) {EXPECT_FALSE(w.step(i * 0.05, true).hold);}
}

TEST(BlockWait, ClearsOnRecheck)
{
  BlockWait w(3.0, 0.5);
  auto o = w.step(0.0, true);
  EXPECT_TRUE(o.hold);
  EXPECT_EQ(o.ev, BlockWait::Ev::Start);
  o = w.step(0.25, false);                              // 다시 보기 전 → 계속 기다림
  EXPECT_TRUE(o.hold);
  EXPECT_EQ(o.ev, BlockWait::Ev::None);
  o = w.step(0.5, true);
  EXPECT_TRUE(o.hold);
  o = w.step(1.0, false);
  EXPECT_FALSE(o.hold);
  EXPECT_EQ(o.ev, BlockWait::Ev::Clear);
  EXPECT_NEAR(o.waited, 1.0, 1e-9);
  EXPECT_FALSE(w.step(1.05, false).hold);
}

TEST(BlockWait, GivesUpThenWaitsAgainAfterClear)
{
  BlockWait w(3.0, 0.5);
  double t = 0.0;
  int holds = 0;
  BlockWait::Out o;
  for (; t < 10.0; t += 0.05) {
    o = w.step(t, true);
    if (o.ev == BlockWait::Ev::GiveUp) {break;}
    holds += o.hold;
  }
  EXPECT_EQ(o.ev, BlockWait::Ev::GiveUp);
  EXPECT_FALSE(o.hold);
  EXPECT_NEAR(o.waited, 3.0, 0.06);
  EXPECT_NEAR(holds * 0.05, 3.0, 0.1);
  for (int i = 0; i < 40; ++i) {EXPECT_FALSE(w.step(t += 0.05, true).hold);}   // 같은 막힘 → 원래 동작
  // 한 주기만 풀렸다 다시 막힘 (점 수 · 상태 깜빡임) → 다시 안 기다림
  EXPECT_FALSE(w.step(t += 0.05, false).hold);
  EXPECT_FALSE(w.step(t += 0.05, true).hold);
  // recheck_s (0.5 s) 넘게 계속 풀림 → 다음 막힘은 다시 기다림
  for (int i = 0; i < 11; ++i) {EXPECT_FALSE(w.step(t += 0.05, false).hold);}
  EXPECT_EQ(w.step(t += 0.05, true).ev, BlockWait::Ev::Start);
}

namespace
{
// 제자리로 ±wz 를 번갈아 swing_s 씩 (odom = 명령). 반환 = 보낸 명령들
std::vector<std::array<double, 3>> wiggle(RevLimit & r, double & t, double secs, double wz, double swing_s, double vx = 0.0,
  bool free = false, std::vector<RevLimit::Ev> * evs = nullptr)
{
  const double dt = 0.05;
  std::vector<std::array<double, 3>> out;
  for (double e = 0; e < secs - 1e-9; e += dt, t += dt) {
    const double sg = std::fmod(e, 2 * swing_s) < swing_s ? 1.0 : -1.0;
    RevLimit::Ev ev;
    out.push_back(r.filter(t, {vx, 0.0, sg * wz}, sg * wz, dt, free, &ev));
    if (evs && ev != RevLimit::Ev::None) {evs->push_back(ev);}
  }
  return out;
}
}  // namespace

TEST(RevLimit, OffPassesThrough)
{
  RevLimit r;
  double t = 0;
  const auto out = wiggle(r, t, 20.0, 0.4, 1.0);
  for (const auto & c : out) {EXPECT_NEAR(std::abs(c[2]), 0.4, 1e-12);}
}

TEST(RevLimit, HoldThenOneDirectionThenFree)
{
  RevLimit::Cfg c;
  c.max_rev = 3;
  RevLimit r(c);
  double t = 0;
  std::vector<RevLimit::Ev> evs;
  // 1 s 마다 반전 (23°): 4 번째 반전 (t ≈ 4 s) 에 멈춤
  auto out = wiggle(r, t, 4.5, 0.4, 1.0, 0.0, false, &evs);
  ASSERT_EQ(evs.size(), 1u);
  EXPECT_EQ(evs[0], RevLimit::Ev::Hold);
  EXPECT_NEAR(out.back()[2], 0.0, 1e-12);
  // 멈춤 1 s 동안 0 → 고정 5 s: 처음 도는 쪽만
  evs.clear();
  out = wiggle(r, t, 5.4, 0.4, 1.0, 0.0, false, &evs);   // t 4.5 → 9.9 (멈춤 ~5.0 · 고정 ~10.0)
  int first = 0, opposite = 0, zero_after_lock = 0;
  for (const auto & cmd : out) {
    if (std::abs(cmd[2]) < 1e-12) {++zero_after_lock; continue;}
    const int sg = cmd[2] > 0 ? 1 : -1;
    if (first == 0) {first = sg;}
    if (sg != first) {++opposite;}
  }
  EXPECT_NE(first, 0);
  EXPECT_LE(opposite, 0);                               // 고정 동안 반대로 안 돎
  EXPECT_GT(zero_after_lock, 0);
  ASSERT_EQ(evs.size(), 1u);
  EXPECT_EQ(evs[0], RevLimit::Ev::Lock);
  // 고정 끝 → 다시 그대로 (Unlock)
  evs.clear();
  out = wiggle(r, t, 1.0, 0.4, 1.0, 0.0, false, &evs);
  ASSERT_FALSE(evs.empty());
  EXPECT_EQ(evs[0], RevLimit::Ev::Unlock);
  EXPECT_NEAR(std::abs(out.back()[2]), 0.4, 1e-12);
}

TEST(RevLimit, TranslationKeptInLock)
{
  RevLimit::Cfg c;
  c.max_rev = 1;
  c.lin_eps = 0.5;                                      // 병진 0.1 도 제자리로 봄 (이 시험만)
  RevLimit r(c);
  double t = 0;
  wiggle(r, t, 2.1, 0.4, 1.0, 0.1);                     // 2 번 반전 → 멈춤
  wiggle(r, t, 1.0, 0.4, 1.0, 0.1);                     // 멈춤 1 s
  const auto out = wiggle(r, t, 2.0, 0.4, 1.0, 0.1);    // 고정: 반대 wz 만 0, 병진 그대로
  for (const auto & cmd : out) {EXPECT_NEAR(cmd[0], 0.1, 1e-12);}
}

TEST(RevLimit, IgnoresSmallSwingsMovingAndNearGoal)
{
  RevLimit::Cfg c;
  c.max_rev = 3;
  {
    RevLimit r(c);                                      // 0.1 s 마다 반전 = 0.6° (도착 직전 미세 조정) → 안 셈
    double t = 0;
    std::vector<RevLimit::Ev> evs;
    wiggle(r, t, 10.0, 0.1, 0.1, 0.0, false, &evs);
    EXPECT_TRUE(evs.empty());
  }
  {
    RevLimit r(c);                                      // 달리며 (0.2 m/s) 도는 반전 → 제자리 아님
    double t = 0;
    std::vector<RevLimit::Ev> evs;
    wiggle(r, t, 10.0, 0.4, 1.0, 0.2, false, &evs);
    EXPECT_TRUE(evs.empty());
  }
  {
    RevLimit r(c);                                      // 목표 근처 (free) → 안 멈춤, 나온 뒤 묵은 셈으로 바로 멈추지 않음
    double t = 0;
    std::vector<RevLimit::Ev> evs;
    wiggle(r, t, 10.0, 0.4, 1.0, 0.0, true, &evs);
    EXPECT_TRUE(evs.empty());
    wiggle(r, t, 2.5, 0.4, 1.0, 0.0, false, &evs);      // 밖에서 반전 2–3 번 (≤ 3) → 그대로
    EXPECT_TRUE(evs.empty());
  }
  {
    RevLimit r(c);                                      // 반전 3 번까지는 그대로 (넘어야 멈춤), 창 10 s 지나면 잊음
    double t = 0;
    std::vector<RevLimit::Ev> evs;
    wiggle(r, t, 3.5, 0.4, 1.0, 0.0, false, &evs);      // 반전 3 번
    wiggle(r, t, 11.0, 0.0, 1.0, 0.0, false, &evs);     // 쉼 11 s
    wiggle(r, t, 3.5, 0.4, 1.0, 0.0, false, &evs);      // 다시 3 번 (창 밖 3 번은 잊음)
    EXPECT_TRUE(evs.empty());
  }
}
