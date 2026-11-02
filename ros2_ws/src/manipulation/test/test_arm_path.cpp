// test_arm_path.cpp — 실물 경로 → 시간표 · 동작 목록 시험 (arm_path.hpp)
// colcon test --packages-select manipulation

#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include "manipulation/arm_path.hpp"

using manipulation::ArmPath;
using manipulation::PathNode;
using manipulation::PathTiming;

namespace
{
constexpr double kDeg = M_PI / 180.0;

PathNode node(const std::string & name, std::vector<double> deg, bool stand = false, const std::string & grip = "",
  bool stop = false, double speed = 0.0)
{
  for (auto & d : deg) {d *= kDeg;}
  return {name, deg, stand, speed, grip, stop};
}

double max_speed_dps(const std::vector<double> & start, const std::vector<manipulation::TrajPoint> & pts)
{
  double vmax = 0.0, t0 = 0.0;
  std::vector<double> q0 = start;
  for (const auto & p : pts) {
    double d = 0.0;
    for (std::size_t j = 0; j < q0.size(); ++j) {d = std::max(d, std::fabs(p.q[j] - q0[j]));}
    vmax = std::max(vmax, d / kDeg / (p.t - t0));
    q0 = p.q;
    t0 = p.t;
  }
  return vmax;
}
}  // namespace

// 1) 한 구간: 끝점에 정확히 도착, 시각은 늘어나기만, 속도는 상한 안
TEST(TimePath, SingleSegment)
{
  const std::vector<double> s(5, 0.0);
  const auto pts = manipulation::time_path(s, {node("a", {90, 0, 0, 0, 0})}, PathTiming{});
  ASSERT_FALSE(pts.empty());
  EXPECT_NEAR(pts.back().q[0], 90 * kDeg, 1e-9);
  for (std::size_t i = 1; i < pts.size(); ++i) {EXPECT_GT(pts[i].t, pts[i - 1].t);}
  EXPECT_LE(max_speed_dps(s, pts), 40.0 * 1.05);
  // 사다리꼴: 90° · 40 °/s · 150 °/s² → 가감속 각 0.27 s + 등속 약 1.98 s ≈ 2.52 s
  EXPECT_NEAR(pts.back().t, 90.0 / 40.0 + 40.0 / 150.0, 0.05);
}

// 2) 받침 구간은 stand_speed, 경유점 speed 는 더 작으면 그 값
TEST(TimePath, StandAndPointSpeed)
{
  const std::vector<double> s(5, 0.0);
  EXPECT_LE(max_speed_dps(s, manipulation::time_path(s, {node("a", {0, 60, 0, 0, 0}, true)}, PathTiming{})), 20.0 * 1.05);
  EXPECT_LE(max_speed_dps(s, manipulation::time_path(s, {node("a", {0, 60, 0, 0, 0}, false, "", false, 15.0)}, PathTiming{})),
    15.0 * 1.05);
}

// 3) 되돌아가는 꺾임(180°)은 그 점에서 멈춤 — 그 근처 점 속도 ≈ 0
TEST(TimePath, ReversalStopsAtCorner)
{
  const std::vector<double> s(5, 0.0);
  const auto pts = manipulation::time_path(s, {node("a", {30, 0, 0, 0, 0}), node("b", {0, 0, 0, 0, 0})}, PathTiming{});
  double far = 0.0;
  for (const auto & p : pts) {far = std::max(far, p.q[0]);}
  EXPECT_NEAR(far, 30 * kDeg, 0.2 * kDeg);   // 모서리를 깎지 않고 끝까지 감
  EXPECT_NEAR(pts.back().q[0], 0.0, 1e-9);
}

// 4) 움직임이 없으면 마지막 점 하나 (0.3 s)
TEST(TimePath, ZeroLength)
{
  const std::vector<double> s(5, 0.1);
  const auto pts = manipulation::time_path(s, {node("a", {0.1 / kDeg, 0.1 / kDeg, 0.1 / kDeg, 0.1 / kDeg, 0.1 / kDeg})}, PathTiming{});
  ASSERT_EQ(pts.size(), 1u);
  EXPECT_DOUBLE_EQ(pts[0].t, 0.3);
}

// 5) 그리퍼 · 멈춤에서 끊고, 그리퍼 동작은 팔 동작 뒤 따로. 받침으로 끝나면 마지막 pose = home
TEST(PathActions, SplitAtGripAndStop)
{
  ArmPath p;
  p.name = "store_S9";
  p.ends_home = true;
  p.nodes = {node("lift", {10, 0, 0, 0, 0}), node("over", {20, 0, 0, 0, 0}), node("in", {20, 10, 0, 0, 0}, false, "release", true),
    node("out", {20, 0, 0, 0, 0}, false, "close"), node("P0", {0, 0, 0, 0, 0}, true), node("rest", {0, -20, 0, 0, 0}, true)};
  const auto acts = manipulation::path_actions(p, PathTiming{}, manipulation::GripperValues{});
  ASSERT_EQ(acts.size(), 5u);   // 팔(lift→in) · 놓기 · 팔(→out) · 닫기 · 팔(→받침)
  EXPECT_FALSE(acts[0].is_gripper);
  EXPECT_TRUE(acts[1].is_gripper);
  EXPECT_NEAR(acts[1].gripper, 0.0093, 1e-9);
  EXPECT_EQ(acts[1].phase, "release");
  EXPECT_TRUE(acts[3].is_gripper);
  EXPECT_NEAR(acts[3].gripper, -0.0016, 1e-9);
  EXPECT_FALSE(acts[4].is_gripper);
  EXPECT_EQ(acts[4].pose, "home");
  EXPECT_GT(acts[4].duration_s, 0.0);
  EXPECT_NEAR(acts[4].points.back().q[1], -20 * kDeg, 1e-9);
}

// 6) 받침으로 안 끝나는 경로(꺼내기 → 들기)는 마지막 pose 가 home 이 아님 → 주행 허용 안 됨
TEST(PathActions, NotHomeEnding)
{
  ArmPath p;
  p.name = "retrieve_S9";
  p.nodes = {node("rest", {0, -20, 0, 0, 0}, true), node("P0", {0, 0, 0, 0, 0}, true), node("lift", {10, 0, 0, 0, 0})};
  const auto acts = manipulation::path_actions(p, PathTiming{}, manipulation::GripperValues{});
  ASSERT_EQ(acts.size(), 1u);
  EXPECT_NE(acts.back().pose, "home");
}

// 7) 출발 자세 확인: 관절마다 허용 안 · NaN 은 아님
TEST(PathActions, StartCheck)
{
  ArmPath p;
  p.name = "x";
  p.nodes = {node("a", {10, 0, 0, 0, 0}), node("b", {20, 0, 0, 0, 0})};
  std::string why;
  EXPECT_TRUE(manipulation::path_start_ok(p, {10 * kDeg + 0.05, 0, 0, 0, 0}, 0.1, why));
  EXPECT_FALSE(manipulation::path_start_ok(p, {10 * kDeg + 0.2, 0, 0, 0, 0}, 0.1, why));
  EXPECT_FALSE(manipulation::path_start_ok(p, {std::nan(""), 0, 0, 0, 0}, 0.1, why));
  EXPECT_FALSE(manipulation::path_start_ok(p, {10 * kDeg, 0, 0, 0}, 0.1, why));
}
