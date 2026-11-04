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

// 6-1) hold_s: 멈추는 점에서 그 자세로 기다림 → 마지막 점 하나 더 (같은 자세, hold 만큼 뒤) · 동작 시간도 그만큼
TEST(PathActions, HoldAddsWaitAtStop)
{
  ArmPath p;
  p.name = "rack_scan";
  p.nodes = {node("P0", {0, 0, 0, 0, 0}, true), node("scan1", {10, 0, 0, 0, 0}), node("scan2", {20, 0, 0, 0, 0})};
  p.nodes[1].stop = true;
  p.nodes[1].hold_s = 2.0;
  const auto plain = manipulation::path_actions(p, PathTiming{}, manipulation::GripperValues{});
  ASSERT_EQ(plain.size(), 2u);
  const auto & a = plain[0];
  ASSERT_GE(a.points.size(), 2u);
  const auto & last = a.points.back();
  const auto & before = a.points[a.points.size() - 2];
  EXPECT_NEAR(last.t - before.t, 2.0, 1e-9);
  EXPECT_EQ(last.q, before.q);
  EXPECT_NEAR(last.q[0], 10 * kDeg, 1e-9);
  EXPECT_NEAR(a.duration_s, last.t, 1e-12);
  p.nodes[1].hold_s = 0.0;
  const auto no_hold = manipulation::path_actions(p, PathTiming{}, manipulation::GripperValues{});
  EXPECT_NEAR(a.duration_s - no_hold[0].duration_s, 2.0, 1e-9);
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

// 8) 0.1° 이하 미세 이동도 짧게 (예전: 한 칸이라 평균 속도 0 → 50 ~ 100 s)
TEST(TimePath, TinyMoveIsShort)
{
  const std::vector<double> s{0, 0, 0, 0, 0};
  for (double d : {0.01, 0.05, 0.09, 0.1, 0.11}) {
    const auto pts = manipulation::time_path(s, {node("a", {d, 0, 0, 0, 0})}, PathTiming{});
    ASSERT_FALSE(pts.empty());
    EXPECT_LT(pts.back().t, 0.5) << d << "°";
    EXPECT_NEAR(pts.back().q[0], d * kDeg, 1e-12);
  }
}

// 9) 모든 경유점이 출력 점에 그대로 있음 (JTC 가 점 사이를 직선으로 이어도 꺾인 곳을 깎지 않음)
TEST(TimePath, KeepsEveryWaypoint)
{
  const std::vector<double> s{0, 0, 0, 0, 0};
  const std::vector<PathNode> nodes{node("a", {10, 3, 0, 0, 0}), node("b", {12, -4, 1, 0, 0}), node("c", {25, 2, 0, 0, 0}),
    node("d", {26.3, 2.5, 0, 0, 0})};
  const auto pts = manipulation::time_path(s, nodes, PathTiming{});
  for (const auto & n : nodes) {
    bool found = false;
    for (const auto & p : pts) {
      double d = 0.0;
      for (std::size_t j = 0; j < 5; ++j) {d = std::max(d, std::fabs(p.q[j] - n.q[j]));}
      found = found || d < 1e-9;
    }
    EXPECT_TRUE(found) << n.name;
  }
  for (std::size_t i = 1; i < pts.size(); ++i) {EXPECT_GT(pts[i].t, pts[i - 1].t);}
  EXPECT_LE(max_speed_dps(s, pts), 40.0 * 1.05);
}

// 9b) 끝 바로 앞 경유점 (마지막 구간이 아주 짧음): 이웃 시각이 1 ms 넘게 떨어짐 — JTC 는 나노초로 바꿔 같은 시각이면 거절
//     (11-04 실물: "Time between points 19 and 20 is not strictly increasing, 0.999407 and 0.999407")
TEST(TimePath, NoNearDuplicateTimes)
{
  const std::vector<double> s{0, 0, 0, 0, 0};
  for (double tiny : {1e-9, 1e-6, 1e-3, 0.02}) {
    const std::vector<PathNode> nodes{node("a", {12, -4, 1, 0, 0}), node("b", {12 + tiny, -4, 1, 0, 0})};
    const auto pts = manipulation::time_path(s, nodes, PathTiming{});
    ASSERT_FALSE(pts.empty());
    for (std::size_t i = 1; i < pts.size(); ++i) {EXPECT_GT(pts[i].t - pts[i - 1].t, 1e-3) << "tiny " << tiny << " at " << i;}
    for (std::size_t j = 0; j < 5; ++j) {EXPECT_NEAR(pts.back().q[j], nodes.back().q[j], 1e-12);}   // 끝 = 마지막 경유점
  }
}

// 10) 실측 출발: 허용 오차만큼 어긋난 자세에서도 첫 구간이 속도 한계 안 (예전: 첫 0.05 s 에 몰아서 100 °/s 넘음)
TEST(PathActions, StartsFromMeasuredPose)
{
  ArmPath p{"stand", {node("받침", {0, 0, 0, 0, 0}, true), node("up", {0, 10, 0, 0, 0}, true)}, false};
  const std::vector<double> q_now{0.09, 0, 0, 0, 0};   // J1 0.09 rad (5.2°) 어긋남
  const auto acts = manipulation::path_actions(p, PathTiming{}, manipulation::GripperValues{}, q_now);
  ASSERT_EQ(acts.size(), 1u);
  EXPECT_LE(max_speed_dps(q_now, acts[0].points), 20.0 * 1.05);
  EXPECT_NEAR(acts[0].points.back().q[0], 0.0, 1e-12);
  const auto old = manipulation::path_actions(p, PathTiming{}, manipulation::GripperValues{});
  EXPECT_GT(max_speed_dps(q_now, old[0].points), 50.0);   // 실측을 안 주면 (예전 방식) 첫 점까지 급하게
}

// 나중에 접기: STORE 끝 = fold 전체 → P0 에서 끝 (P0 한 번), RETRIEVE 앞 = unfold 전체 → P0 에서 시작, 다른 경로는 그대로
TEST(LazyFold, CutTailSkipHeadAndGuard)
{
  ArmPath fold, unfold, store, retrieve, other;
  fold.nodes = {node("P0", {0, -79, 36, -1, 1}), node("up", {0, -40, 36, -1, 1}), node("rest", {0, -83, 54, -7, 1})};
  fold.ends_home = true;
  unfold.nodes = {node("rest", {0, -83, 54, -7, 1}), node("up", {0, -40, 36, -1, 1}), node("P0", {0, -79, 36, -1, 1})};
  store.nodes = {node("staging", {10, 24, -38, 11, 12}), node("cell", {120, 10, -50, 29, -48})};
  store.nodes.insert(store.nodes.end(), fold.nodes.begin(), fold.nodes.end());
  store.ends_home = true;
  ASSERT_TRUE(manipulation::cut_fold_tail(store, fold));
  ASSERT_EQ(store.nodes.size(), 3u);
  EXPECT_EQ(store.nodes.back().name, "P0");
  EXPECT_FALSE(store.ends_home);
  EXPECT_FALSE(manipulation::cut_fold_tail(store, fold));   // 이미 잘림 → 그대로
  ArmPath f2 = fold;
  EXPECT_FALSE(manipulation::cut_fold_tail(f2, fold));      // HOME 의 fold 자체는 안 자름
  EXPECT_EQ(f2.nodes.size(), 3u);

  retrieve.nodes = unfold.nodes;
  retrieve.nodes.push_back(node("cell", {120, 10, -50, 29, -48}));
  ASSERT_TRUE(manipulation::skip_unfold(retrieve, unfold));
  ASSERT_EQ(retrieve.nodes.size(), 2u);
  EXPECT_EQ(retrieve.nodes.front().name, "P0");
  EXPECT_FALSE(manipulation::skip_unfold(retrieve, unfold));

  other.nodes = {node("rest", {0, -83, 54, -7, 1}), node("x", {5, -40, 36, -1, 1}), node("P0", {0, -79, 36, -1, 1}), node("y", {9, 0, 0, 0, 0})};
  EXPECT_FALSE(manipulation::skip_unfold(other, unfold));   // 가운데 점이 다르면 안 자름

  // P0 와 받침이 출발 허용(0.15 rad) 보다 멀어야 켬: 여기 J3 차 18° = 0.31 rad
  EXPECT_TRUE(manipulation::p0_distinct(fold, unfold, 0.15));
  EXPECT_FALSE(manipulation::p0_distinct(fold, unfold, 0.35));
  EXPECT_FALSE(manipulation::p0_distinct(fold, ArmPath{}, 0.15));
}

// 그리퍼 동작 · 멈춤이 있는 점은 잘라내지 않음 (잘라내면 동작이 사라짐)
TEST(LazyFold, KeepsGripOrStopNodes)
{
  ArmPath fold, unfold, retrieve, store;
  fold.nodes = {node("P0", {0, -79, 36, -1, 1}), node("rest", {0, -83, 54, -7, 1})};
  unfold.nodes = {node("rest", {0, -83, 54, -7, 1}), node("P0", {0, -79, 36, -1, 1})};
  retrieve.nodes = {node("rest", {0, -83, 54, -7, 1}), node("P0", {0, -79, 36, -1, 1}, false, "open"), node("cell", {120, 10, -50, 29, -48})};
  EXPECT_FALSE(manipulation::skip_unfold(retrieve, unfold));   // P0 의 열기가 출발점이 되면 실행 안 됨
  store.nodes = {node("cell", {120, 10, -50, 29, -48}), node("P0", {0, -79, 36, -1, 1}), node("rest", {0, -83, 54, -7, 1}, false, "close")};
  store.ends_home = true;
  EXPECT_FALSE(manipulation::cut_fold_tail(store, fold));      // 받침에서 닫기가 사라짐
  EXPECT_TRUE(store.ends_home);
}
