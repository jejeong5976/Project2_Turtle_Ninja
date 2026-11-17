// test_arm_path.cpp — 실물 경로 → 시간표 · 동작 목록 시험 (arm_path.hpp)
// colcon test --packages-select manipulation

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <set>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "manipulation/arm_path.hpp"
#include "manipulation/cell_shift.hpp"
#include "manipulation/held_image.hpp"
#include "manipulation/standup_table.hpp"
#include "arm_kin_data.hpp"

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
// 세우기 표: 모든 경유점에 dt_s 가 있으면 속도 · 가속 한계 없이 그 간격 그대로 (녹화 재생)
TEST(TimePath, TimedNodesKeepRecordedSpacing)
{
  const std::vector<double> s(5, 0.0);
  std::vector<PathNode> ns{node("a", {2, 0, 0, 0, 0}), node("b", {4, 1, 0, 0, 0}), node("c", {90, 0, 0, 0, 0})};
  for (auto & n : ns) {n.dt_s = 0.1;}
  const auto pts = manipulation::time_path(s, ns, PathTiming{});
  ASSERT_EQ(pts.size(), 3u);
  EXPECT_NEAR(pts[0].t, 0.1, 1e-12);
  EXPECT_NEAR(pts[2].t, 0.3, 1e-12);   // 86° 를 0.1 s — 속도 상한 (40 °/s) 무시
  EXPECT_NEAR(pts[2].q[0], 90 * kDeg, 1e-12);
  ns[1].dt_s = 0.0;   // 하나라도 없으면 속도 기준 시간표
  EXPECT_GT(manipulation::time_path(s, ns, PathTiming{}).back().t, 2.0);
}

// 세우기: 시간 맞춘 점의 grip_on_way 는 팔 동작을 끊지 않고 도착 시각에 그리퍼만
TEST(PathActions, GripOnWayKeepsArmMoving)
{
  ArmPath p;
  p.nodes = {node("s", {0, 0, 0, 0, 0}), node("a", {1, 0, 0, 0, 0}), node("b", {2, 0, 0, 0, 0}, false, "open"), node("c", {3, 0, 0, 0, 0}),
    node("d", {4, 0, 0, 0, 0})};
  for (std::size_t i = 1; i < p.nodes.size(); ++i) {p.nodes[i].dt_s = 0.1;}
  p.nodes[2].grip_on_way = true;
  p.nodes[3].stop = true;
  const auto acts = manipulation::path_actions(p, PathTiming{}, manipulation::GripperValues{});
  ASSERT_EQ(acts.size(), 2u);                  // a·b·c 한 동작 (c 에서 멈춤) + d
  EXPECT_FALSE(acts[0].is_gripper);
  EXPECT_EQ(acts[0].points.size(), 3u);
  EXPECT_NEAR(acts[0].side_grip_t, 0.2, 1e-12);   // b 도착 시각
  EXPECT_NEAR(acts[0].side_grip, manipulation::GripperValues{}.open, 1e-12);
  EXPECT_LT(acts[1].side_grip_t, 0.0);
}

// PICK: 퍽 위로 출발할 때 미리 열기 (grip_early) — 속도 시간표 구간에서도 안 끊고 시각 0, 쥐기 (close) 는 그대로 끊음
TEST(PathActions, GripEarlyOpensWhileMoving)
{
  ArmPath p;
  p.nodes = {node("s", {0, 0, 0, 0, 0}), node("above", {20, 0, 0, 0, 0}, false, "open"), node("down", {25, 0, 0, 0, 0}),
    node("grasp", {30, 0, 0, 0, 0}, false, "close"), node("lift", {20, 0, 0, 0, 0})};
  p.nodes[1].grip_early = true;
  const auto acts = manipulation::path_actions(p, PathTiming{}, manipulation::GripperValues{});
  ASSERT_EQ(acts.size(), 3u);   // s → above → down → grasp (열기는 처음에) · [닫기] · lift
  EXPECT_FALSE(acts[0].is_gripper);
  EXPECT_NEAR(acts[0].side_grip_t, 0.0, 1e-12);
  EXPECT_NEAR(acts[0].side_grip, manipulation::GripperValues{}.open, 1e-12);
  EXPECT_TRUE(acts[1].is_gripper);
  // 도착 시각 (grip_on_way) 은 그 점의 시각을 시간표에서 찾음
  p.nodes[1].grip_early = false;
  p.nodes[1].grip_on_way = true;
  const auto b2 = manipulation::path_actions(p, PathTiming{}, manipulation::GripperValues{});
  ASSERT_EQ(b2.size(), 3u);
  EXPECT_GT(b2[0].side_grip_t, 0.0);
  EXPECT_LT(b2[0].side_grip_t, b2[0].duration_s);
}

// PICK 퍽 위 멈춤 (pick_settle_s → above 에 stop + hold_s): 0 이면 멈춤 없음, > 0 이면 above 에서 끊고 그 자세로 기다림,
//   미리 열기 (grip_early) 는 그대로 출발 때 · 그리퍼 동작은 안 생김 · 쥐기 (close) 순서 그대로
TEST(PathActions, SettleAtAboveKeepsEarlyOpen)
{
  ArmPath p;
  p.nodes = {node("s", {0, 0, 0, 0, 0}), node("above", {20, 0, 0, 0, 0}, false, "open"), node("down", {25, 0, 0, 0, 0}),
    node("grasp", {30, 0, 0, 0, 0}, false, "close"), node("lift", {20, 0, 0, 0, 0})};
  p.nodes[1].grip_early = true;
  const auto plain = manipulation::path_actions(p, PathTiming{}, manipulation::GripperValues{});
  ASSERT_EQ(plain.size(), 3u);   // 0: 멈춤 없음
  EXPECT_NEAR(plain[0].side_grip_t, 0.0, 1e-12);
  p.nodes[1].stop = true;
  p.nodes[1].hold_s = 0.4;
  const auto acts = manipulation::path_actions(p, PathTiming{}, manipulation::GripperValues{});
  ASSERT_EQ(acts.size(), 4u);   // s → above (출발 때 열기, 0.4 s 기다림) · above → down → grasp · [닫기] · lift
  EXPECT_FALSE(acts[0].is_gripper);
  EXPECT_NEAR(acts[0].side_grip_t, 0.0, 1e-12);
  EXPECT_NEAR(acts[0].side_grip, manipulation::GripperValues{}.open, 1e-12);
  ASSERT_GE(acts[0].points.size(), 2u);
  const auto & last = acts[0].points.back();
  const auto & before = acts[0].points[acts[0].points.size() - 2];
  EXPECT_NEAR(last.t - before.t, 0.4, 1e-9);
  EXPECT_EQ(last.q, before.q);
  EXPECT_NEAR(last.q[0], 20 * kDeg, 1e-9);
  EXPECT_FALSE(acts[1].is_gripper);
  EXPECT_LT(acts[1].side_grip_t, 0.0);   // 두 번째 동작엔 그리퍼 없음 (열기가 두 번 안 됨)
  EXPECT_NEAR(acts[1].points.back().q[0], 30 * kDeg, 1e-9);
  EXPECT_TRUE(acts[2].is_gripper);
  EXPECT_NEAR(acts[2].gripper, manipulation::GripperValues{}.close, 1e-9);
  // 도착해서 여는 (grip_on_way) 점도 같음: 도착 시각에 열고 그 점에서 멈춤
  p.nodes[1].grip_early = false;
  p.nodes[1].grip_on_way = true;
  const auto b2 = manipulation::path_actions(p, PathTiming{}, manipulation::GripperValues{});
  ASSERT_EQ(b2.size(), 4u);
  EXPECT_GT(b2[0].side_grip_t, 0.0);
  EXPECT_FALSE(b2[1].is_gripper);
}

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

// 8) 0.1° 이하 미세 이동도 짧게 (한 칸 이동을 평균 속도 0 으로 보면 50 ~ 100 s)
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
//     (실물: "Time between points 19 and 20 is not strictly increasing, 0.999407 and 0.999407")
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

// 10) 실측 출발: 허용 오차만큼 어긋난 자세에서도 첫 구간이 속도 한계 안 (실측을 안 쓰면 첫 0.05 s 에 몰아서 100 °/s 넘음)
TEST(PathActions, StartsFromMeasuredPose)
{
  ArmPath p{"stand", {node("받침", {0, 0, 0, 0, 0}, true), node("up", {0, 10, 0, 0, 0}, true)}, false};
  const std::vector<double> q_now{0.09, 0, 0, 0, 0};   // J1 0.09 rad (5.2°) 어긋남
  const auto acts = manipulation::path_actions(p, PathTiming{}, manipulation::GripperValues{}, q_now);
  ASSERT_EQ(acts.size(), 1u);
  EXPECT_LE(max_speed_dps(q_now, acts[0].points), 20.0 * 1.05);
  EXPECT_NEAR(acts[0].points.back().q[0], 0.0, 1e-12);
  const auto old = manipulation::path_actions(p, PathTiming{}, manipulation::GripperValues{});
  EXPECT_GT(max_speed_dps(q_now, old[0].points), 50.0);   // 실측을 안 주면 첫 점까지 급하게
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

// 실제 config/arm_paths.yaml: 받침에서 시작하는 빼기 · LOOK 경로는 앞이 unfold 와 같아야 P0 에서 바로 시작 (lazy_fold).
//   빼기 경로에서 upJ3a 를 지우면 P0 잇기가 조용히 안 됨 → 경로를 고칠 때 깨지지 않게
#ifdef ARM_PATHS_YAML
namespace
{
std::vector<std::string> list_items(const std::string & line)
{
  std::vector<std::string> out;
  const auto a = line.find('['), b = line.rfind(']');
  if (a == std::string::npos || b == std::string::npos || b <= a) {return out;}
  std::stringstream ss(line.substr(a + 1, b - a - 1));
  std::string it;
  while (std::getline(ss, it, ',')) {
    const auto l = it.find_first_not_of(" \""), r = it.find_last_not_of(" \"");
    out.push_back(l == std::string::npos ? "" : it.substr(l, r - l + 1));
  }
  return out;
}

bool load_path(const std::string & want, ArmPath & p, const std::string & file = ARM_PATHS_YAML)
{
  std::ifstream f(file);
  std::string line;
  bool in = false;
  std::vector<std::string> nodes, q, grip, stop;
  while (std::getline(f, line)) {
    if (line == "      " + want + ":") {in = true; continue;}
    if (!in) {continue;}
    if (line.rfind("        ", 0) != 0) {break;}
    const auto key = line.substr(8, line.find(':') - 8);
    if (key == "nodes") {nodes = list_items(line);}
    if (key == "q") {q = list_items(line);}
    if (key == "grip") {grip = list_items(line);}
    if (key == "stop") {stop = list_items(line);}
  }
  if (nodes.empty() || q.size() != nodes.size() * 5 || grip.size() != nodes.size() || stop.size() != nodes.size()) {return false;}
  p = ArmPath{want, {}, false};
  for (std::size_t i = 0; i < nodes.size(); ++i) {
    PathNode n;
    n.name = nodes[i];
    for (int j = 0; j < 5; ++j) {n.q.push_back(std::stod(q[i * 5 + j]));}
    n.grip = grip[i] == "-" ? "" : grip[i];
    n.stop = stop[i] == "true";
    p.nodes.push_back(n);
  }
  return true;
}
}  // namespace

// 넣은 뒤 보기 (control_master store_seat_check): P0 (fold 첫 점) → P0_up (rack_scan) → 칸 관찰 자세 (look_rack_<칸> 끝 rack_look) 관절 직선을
//   1° 씩 따라가며 손끝 · 손목이 다 찬 적재함 퍽 윗면 (칸 바닥 0.1325 + 60 mm) 보다 충분히 위 (칸 30 mm 안을 지날 때 손끝 20 · 손목 40 mm).
//   빈손 스캔 경로 (rack_scan, 다 찬 적재함 메쉬 검사) 가 지나는 점끼리라 높이만 확인 (MoveIt 검사는 아님)
TEST(RealPaths, SeatLookStaysAboveFullRack)
{
  const auto m = kin_data::model();
  std::string rl = ARM_PATHS_YAML;
  rl.replace(rl.rfind("arm_paths.yaml"), std::string("arm_paths.yaml").size(), "rack_look_paths.yaml");
  ArmPath fold, scan;
  ASSERT_TRUE(load_path("fold", fold));
  ASSERT_TRUE(load_path("rack_scan", scan));
  const auto up = std::find_if(scan.nodes.begin(), scan.nodes.end(), [](const PathNode & n) {return n.name == "P0_up";});
  ASSERT_NE(up, scan.nodes.end());
  const std::vector<Eigen::Vector2d> cells{{-0.102, 0.1131}, {-0.102, 0.0681}, {-0.042, 0.1131}, {0.078, -0.1169}, {0.078, 0.1131}, {0.018, 0.1131},
    {0.018, -0.1169}, {-0.042, 0.0681}, {0.078, -0.0719}, {0.078, 0.0681}, {0.018, 0.0681}, {0.018, -0.0719}};   // config/rack_cells.yaml
  const double top = 0.1325 + 0.060, dz = m.base_z_floor - m.team_base_z_floor;   // 모델 z + dz = base_link z
  auto pose = [&](const std::vector<double> & c, Eigen::Vector3d & tip, Eigen::Vector3d & wrist) {
      manipulation::Ticks t;
      for (int j = 0; j < 5; ++j) {t[j] = static_cast<int>(std::lround(m.zero_tick[j] + c[j] * 4096.0 / (2.0 * M_PI)));}
      const auto q = manipulation::ticks_to_q(m, t);
      tip = manipulation::arm_grip(m, q).tip + Eigen::Vector3d(0, 0, dz);
      wrist = manipulation::arm_frames(m, q)[4].block<3, 1>(0, 3) + Eigen::Vector3d(0, 0, dz);
    };
  auto over = [&](const Eigen::Vector3d & p) {
      for (const auto & c : cells) {if ((p.head<2>() - c).norm() < 0.030) {return true;}}
      return false;
    };
  for (int c = 1; c <= 12; ++c) {
    ArmPath lk;
    ASSERT_TRUE(load_path("look_rack_S" + std::to_string(c), lk, rl)) << c;
    const std::vector<std::vector<double>> way{fold.nodes.front().q, up->q, lk.nodes.back().q};
    double tip_min = 1e9, wrist_min = 1e9;
    for (std::size_t k = 0; k + 1 < way.size(); ++k) {
      double span = 0.0;
      for (int j = 0; j < 5; ++j) {span = std::max(span, std::fabs(way[k + 1][j] - way[k][j]) * 180.0 / M_PI);}
      const int n = std::max(1, static_cast<int>(std::ceil(span)));
      for (int i = 0; i <= n; ++i) {
        std::vector<double> q(5);
        for (int j = 0; j < 5; ++j) {q[j] = way[k][j] + (way[k + 1][j] - way[k][j]) * i / n;}
        Eigen::Vector3d tip, wrist;
        pose(q, tip, wrist);
        if (over(tip)) {tip_min = std::min(tip_min, tip.z() - top);}
        if (over(wrist)) {wrist_min = std::min(wrist_min, wrist.z() - top);}
      }
    }
    EXPECT_GT(tip_min, 0.020) << "S" << c << " 손끝이 퍽 윗면 위 " << tip_min * 1000.0 << " mm";
    EXPECT_GT(wrist_min, 0.040) << "S" << c << " 손목이 퍽 윗면 위 " << wrist_min * 1000.0 << " mm";
    std::printf("S%d 넣은 뒤 보기 길: 칸 위 손끝 ≥ %.0f mm · 손목 ≥ %.0f mm (퍽 윗면 위)\n", c, tip_min < 1e8 ? tip_min * 1000.0 : -1.0,
      wrist_min < 1e8 ? wrist_min * 1000.0 : -1.0);
  }
}

TEST(RealPaths, RetrieveAndLookStartWithUnfold)
{
  ArmPath unfold;
  ASSERT_TRUE(load_path("unfold", unfold));
  std::vector<std::string> names{"look", "look_barcode"};
  for (int c = 1; c <= 12; ++c) {names.push_back("retrieve_S" + std::to_string(c));}
  for (const auto & nm : names) {
    ArmPath p;
    ASSERT_TRUE(load_path(nm, p)) << nm;
    const std::size_t n0 = p.nodes.size();
    EXPECT_TRUE(manipulation::skip_unfold(p, unfold)) << nm << ": 앞이 unfold 와 달라 P0 에서 바로 시작 못 함";
    EXPECT_EQ(p.nodes.front().name, "P0") << nm;
    EXPECT_EQ(p.nodes.size(), n0 - (unfold.nodes.size() - 1)) << nm;
  }
}

// 빈손 되돌아가기 (control_master empty_retreat_path) 의 끝: P0 에서 바로 간 PICK 은 P0 (= fold 첫 점) 로 되돌아와 fold 로 받침
TEST(RealPaths, EmptyRetreatEndsAtP0ThenFold)
{
  ArmPath unfold, fold;
  ASSERT_TRUE(load_path("unfold", unfold));
  ASSERT_TRUE(load_path("fold", fold));
  fold.ends_home = true;
  ArmPath pick = unfold;
  const auto & p0 = unfold.nodes.back().q;
  auto at = [&](double dj2, double dj3) {std::vector<double> q = p0; q[1] += dj2; q[2] += dj3; return q;};
  pick.nodes.push_back({"above", at(0.30, -0.20), false, 0.0, "open", false});
  pick.nodes.push_back({"down", at(0.35, -0.20), false, 15.0, "", false});
  pick.nodes.push_back({"grasp", at(0.38, -0.20), false, 15.0, "close", false});
  pick.nodes.push_back({"up", at(0.35, -0.20), false, 15.0, "", false});
  pick.nodes.push_back({"lift", at(0.30, -0.20), false, 15.0, "", false});
  ASSERT_TRUE(manipulation::skip_unfold(pick, unfold));
  const std::size_t up = pick.nodes.size() - 2;
  std::string dummy;
  const std::vector<std::string> free_names{"above", "lift"};
  const auto r = manipulation::retrace_nodes(pick.nodes, up, free_names,
    [&](const PathNode & n) {return manipulation::path_start_ok(fold, n.q, 0.15, dummy);});
  ASSERT_EQ(r.size(), 3u);
  EXPECT_EQ(r.back().name, "P0");
  ArmPath back{"empty_retreat", r, true};
  back.nodes.insert(back.nodes.end(), fold.nodes.begin() + 1, fold.nodes.end());
  const auto acts = manipulation::path_actions(back, PathTiming{}, manipulation::GripperValues{}, r.front().q);
  ASSERT_FALSE(acts.empty());
  EXPECT_EQ(acts.back().pose, "home");
}

// 실제 store_S1 … S12: 놓은 뒤 닫기 늦추기 (store_close_rise_mm 50 · store_close_side_mm 40) 는 조건을 맞춘 점 (끝 점 아님) 으로,
//   접기 (P0_복귀) 앞 · 가며 닫기 그대로 (동작 수 · 시간 같음). 0 / 0 은 그대로
TEST(RealPaths, StoreCloseLaterStaysBeforeFold)
{
  const auto m = kin_data::model();
  for (int c = 1; c <= 12; ++c) {
    const std::string nm = "store_S" + std::to_string(c);
    ArmPath p;
    ASSERT_TRUE(load_path(nm, p)) << nm;
    p.ends_home = true;
    bool released = false;
    for (auto & n : p.nodes) {   // control_master gripper_on_the_way 와 같이
      if (n.grip == "release") {released = true;} else if (released && n.grip == "close") {n.grip_on_way = true;}
    }
    const ArmPath before = p;
    manipulation::StoreCloseMove mv;
    ArmPath z = p;
    EXPECT_FALSE(manipulation::store_delay_close(m, z, 0.0, 0.0, mv)) << nm;
    ASSERT_GT(mv.from, mv.release) << nm;
    EXPECT_EQ(z.nodes[mv.from].grip, "close") << nm;
    manipulation::store_delay_close(m, p, 0.050, 0.040, mv);
    ASSERT_GE(mv.to, mv.from) << nm;
    EXPECT_FALSE(mv.capped) << nm;
    EXPECT_TRUE(mv.to_dz_mm >= 50.0 || mv.to_dxy_mm >= 40.0) << nm << " dz " << mv.to_dz_mm << " dxy " << mv.to_dxy_mm;
    int fold = -1;
    for (std::size_t i = 0; i < p.nodes.size(); ++i) {if (p.nodes[i].name == "P0_복귀") {fold = static_cast<int>(i);}}
    ASSERT_GT(fold, 0) << nm;
    EXPECT_LT(mv.to, fold) << nm;
    EXPECT_EQ(p.nodes[mv.to].grip, "close") << nm;
    EXPECT_TRUE(p.nodes[mv.to].grip_on_way) << nm;
    const auto a0 = manipulation::path_actions(before, PathTiming{}, manipulation::GripperValues{});
    const auto a1 = manipulation::path_actions(p, PathTiming{}, manipulation::GripperValues{});
    ASSERT_EQ(a0.size(), a1.size()) << nm;
    double t0 = 0.0, t1 = 0.0;
    for (std::size_t i = 0; i < a0.size(); ++i) {t0 += a0[i].duration_s; t1 += a1[i].duration_s;}
    EXPECT_NEAR(t0, t1, 1e-9) << nm;
    EXPECT_EQ(a1.back().pose, "home") << nm;
  }
}
#endif

// ============================================================================ 빈손 쥐기 뒤 온 길로 되돌아가기 (retrace_nodes)
namespace
{
using manipulation::retrace_nodes;
const std::vector<std::string> kFree{"above", "lift", "reloc_above", "reloc_lift", "reloc_put_above", "reloc_put_clear",
  "re_above", "re_lift", "re_place_above", "re_place_clear"};

bool same_q(const std::vector<double> & a, const std::vector<double> & b)
{
  if (a.size() != b.size()) {return false;}
  for (std::size_t j = 0; j < a.size(); ++j) {if (std::fabs(a[j] - b[j]) > 1e-9) {return false;}}
  return true;
}

// 결과의 이웃 두 점 (a → b) 이 모두 지나온 구간 (b → a) 을 거꾸로 간 것인가
bool only_reversed_segments(const std::vector<PathNode> & path, const std::vector<PathNode> & r)
{
  for (std::size_t k = 1; k < r.size(); ++k) {
    bool found = false;
    for (std::size_t i = 0; i + 1 < path.size() && !found; ++i) {
      found = same_q(path[i].q, r[k].q) && same_q(path[i + 1].q, r[k - 1].q);
    }
    if (!found) {return false;}
  }
  return true;
}

// PICK 경로 모양 (pick_path): 출발 → 퍽 위 (above) → 내림 (down) → 쥐기 (닫기) → 올림 (up = down 자세) → 들기 (lift = above 자세)
std::vector<PathNode> pick_nodes(const PathNode & start)
{
  return {start, node("above", {10, 20, 30, 40, 0}, false, "open"), node("down", {10, 25, 30, 40, 0}, false, "", false, 15.0),
    node("grasp", {10, 28, 30, 40, 0}, false, "close", false, 15.0), node("up", {10, 25, 30, 40, 0}, false, "", false, 15.0),
    node("lift", {10, 20, 30, 40, 0}, false, "", false, 15.0), node("staging", {0, 5, 5, 5, 0})};
}
}  // namespace

TEST(EmptyRetreat, PickFromP0GoesUpThenBackToP0)
{
  const PathNode p0 = node("P0", {0, 0, 0, 0, 0});
  const auto path = pick_nodes(p0);
  auto exit = [&](const PathNode & n) {return same_q(n.q, p0.q);};
  const auto r = retrace_nodes(path, 4, kFree, exit);   // 4 = up (빈손으로 멈춘 점)
  ASSERT_EQ(r.size(), 3u);
  EXPECT_EQ(r[0].name, "up");
  EXPECT_EQ(r[1].name, "above") << "내려가 쥐었다 올라온 고리 (grasp · down) 는 뺌";
  EXPECT_EQ(r[2].name, "P0");
  EXPECT_TRUE(only_reversed_segments(path, r));
  EXPECT_DOUBLE_EQ(r[1].speed_dps, 15.0) << "up → above = 지나온 above → down 구간 속도 (퍽 근처 느리게)";
  EXPECT_DOUBLE_EQ(r[2].speed_dps, 0.0) << "above → P0 = 지나온 P0 → above 구간 속도 (기본)";
  for (const auto & n : r) {EXPECT_TRUE(n.grip.empty()) << n.name << ": 되돌아갈 때 그리퍼 안 움직임";}
}

// 실제 로봇 사례 (누운 퍽 옮겨 놓은 뒤 받침 세우기 PICK, 두 번째 쥐기에서 빈손): 옮겨 놓기 · 첫 쥐기 고리를 빼고 높은 점만 거꾸로 → P0
TEST(EmptyRetreat, RelocatePickSkipsDescentsAndUsesOnlyTraversedSegments)
{
  const PathNode p0 = node("P0", {0, 0, 0, 0, 0});
  std::vector<PathNode> path{p0,
    node("reloc_above", {20, 20, 30, 40, 0}, false, "open"), node("reloc_down", {20, 25, 30, 40, 0}, false, "", false, 15.0),
    node("reloc_grasp", {20, 28, 30, 40, 0}, false, "close", false, 15.0), node("reloc_up", {20, 25, 30, 40, 0}, false, "", false, 15.0),
    node("reloc_lift", {20, 20, 30, 40, 0}, false, "", false, 15.0),
    node("reloc_put_above", {15, 22, 30, 40, 0}, false, "", true, 15.0), node("reloc_put_down", {15, 26, 30, 40, 0}, false, "", false, 15.0),
    node("reloc_put", {15, 28, 30, 40, 0}, false, "open", false, 15.0), node("reloc_put_up", {15, 26, 30, 40, 0}, false, "", false, 15.0),
    node("reloc_put_clear", {15, 22, 30, 40, 0}, false, "", false, 15.0),
    node("above", {16, 21, 31, 40, 0}, false, "open"), node("down", {16, 25, 31, 40, 0}, false, "", false, 15.0),
    node("grasp", {16, 28, 31, 40, 0}, false, "close", false, 15.0), node("up", {16, 25, 31, 40, 0}, false, "", false, 15.0),
    node("lift", {16, 21, 31, 40, 0}, false, "", false, 15.0), node("park_pull1", {30, 30, 30, 30, 0})};
  auto exit = [&](const PathNode & n) {return same_q(n.q, p0.q);};
  const auto r = retrace_nodes(path, 14, kFree, exit);   // 14 = up
  ASSERT_EQ(r.size(), 5u);
  EXPECT_EQ(r[1].name, "above");
  EXPECT_EQ(r[2].name, "reloc_put_clear");
  EXPECT_EQ(r[3].name, "reloc_lift");
  EXPECT_EQ(r[4].name, "P0");
  EXPECT_TRUE(only_reversed_segments(path, r)) << "모든 구간이 지나온 구간 (거꾸로)";
  for (const auto & n : r) {
    EXPECT_FALSE(n.stop) << n.name;
    EXPECT_DOUBLE_EQ(n.hold_s, 0.0) << n.name;
    EXPECT_EQ(n.name.find("down"), std::string::npos) << "내림 점을 다시 지나지 않음";
    EXPECT_EQ(n.name.find("grasp"), std::string::npos) << "쥐는 점으로 다시 내려가지 않음";
  }
  // 첫 쥐기에서 빈손이면: reloc_up → reloc_above → P0
  const auto r1 = retrace_nodes(path, 4, kFree, exit);
  ASSERT_EQ(r1.size(), 3u);
  EXPECT_EQ(r1[1].name, "reloc_above");
  EXPECT_EQ(r1[2].name, "P0");
}

TEST(EmptyRetreat, LookStartAndUnknownSegments)
{
  const PathNode look = node("look", {0, -10, 40, 60, 0});
  const auto path = pick_nodes(look);
  // LOOK 에서 바로 퍽 위로 간 PICK → 퍽 위 → LOOK (home_from_look 은 control_master 가 붙임)
  const auto r = retrace_nodes(path, 4, kFree, [&](const PathNode & n) {return same_q(n.q, look.q);});
  ASSERT_EQ(r.size(), 3u);
  EXPECT_EQ(r.back().name, "look");
  // 끝점 (검사된 복귀 경로가 있는 자세) 이 없으면 빈 목록
  EXPECT_TRUE(retrace_nodes(path, 4, kFree, [](const PathNode &) {return false;}).empty());
  // 모르는 점 (받침 세우기 등) 을 지나야 하면 빈 목록 → control_master 는 지금 HOME 규칙대로
  std::vector<PathNode> park = path;
  park.insert(park.begin() + 1, node("park_press1", {5, 5, 5, 5, 0}));
  EXPECT_TRUE(retrace_nodes(park, 5, kFree, [&](const PathNode & n) {return same_q(n.q, look.q);}).empty());
  // 고리로 안 빠지는 쥐는 점 (up ≠ down) → 쥐는 점을 지나야 함 → 빈 목록 (다시 내려가지 않음)
  std::vector<PathNode> odd = path;
  odd[4].q[1] += 0.01;
  EXPECT_TRUE(retrace_nodes(odd, 4, kFree, [&](const PathNode & n) {return same_q(n.q, look.q);}).empty());
  EXPECT_TRUE(retrace_nodes(path, 99, kFree, [](const PathNode &) {return true;}).empty());
}

TEST(EmptyRetreat, ActionsEndHomeWithoutGripper)
{
  const PathNode p0 = node("P0", {0, 0, 0, 0, 0});
  const auto path = pick_nodes(p0);
  const auto r = retrace_nodes(path, 4, kFree, [&](const PathNode & n) {return same_q(n.q, p0.q);});
  ArmPath back{"empty_retreat", r, true};
  back.nodes.push_back(node("home", {0, -60, 60, 50, 0}, true));   // fold 의 나머지 (P0 다음부터)
  const auto acts = manipulation::path_actions(back, PathTiming{}, manipulation::GripperValues{}, r.front().q);
  ASSERT_FALSE(acts.empty());
  for (const auto & a : acts) {EXPECT_FALSE(a.is_gripper);}
  EXPECT_EQ(acts.back().pose, "home") << "끝 = 받침 → 접힘 실측 확인 뒤 주행 허용";
}

// FAULT [HOME_FAILED] 사례: 먼 퍽 표 (farpick_plan) 를 관찰 자세에서 재생 → far_grasp 빈손 → 위 점에서 멈춤 →
//   되돌아가기 경로 없음 (표 점 이름 모름) → HOME 거절. 표 점 (far<i> · far_grasp) 을 주면 위 점 → far_grasp → 내려온 표 점 거꾸로 → look
TEST(EmptyRetreat, FarpickTableBackToLookAtRecordedSpeed)
{
  const PathNode look = node("look", {0, -10, 40, 60, 0});
  std::vector<PathNode> path{look};
  PathNode n1 = node("far1", {6, 3, 30, 50, 0}, false, "open");   // LOOK → 위 (기본 속도, 시간 안 맞춤)
  path.push_back(n1);
  for (int i = 2; i <= 6; ++i) {   // 표 점 0.1 s 간격 (i = 5 가 닫는 점)
    PathNode n = node(i == 5 ? "far_grasp" : "far" + std::to_string(i), {5.0 + i, 3.0 * i, 30, 50, 0}, false, i == 5 ? "close" : "");
    n.dt_s = 0.1;
    path.push_back(n);
  }
  PathNode lift = node("far_lift", {12, 10, 30, 50, 0}, false, "", true);
  lift.dt_s = 0.1;
  path.push_back(lift);
  std::vector<std::string> free;
  for (const auto & n : path) {if (manipulation::farpick_free_node(n.name)) {free.push_back(n.name);}}
  auto at_look = [&](const PathNode & n) {return same_q(n.q, look.q);};
  // 위 점 = 닫는 점 다음 (far6, 번호 6)
  const auto r = retrace_nodes(path, 6, free, at_look);
  ASSERT_EQ(r.size(), 7u);
  EXPECT_EQ(r.front().name, "far6");
  EXPECT_EQ(r[1].name, "far_grasp");
  EXPECT_EQ(r[2].name, "far4");
  EXPECT_EQ(r.back().name, "look");
  EXPECT_TRUE(only_reversed_segments(path, r));
  for (const auto & n : r) {
    EXPECT_TRUE(n.grip.empty()) << n.name;
    EXPECT_DOUBLE_EQ(n.dt_s, 0.0) << n.name << ": 시간 맞춤 없이 (속도 시간표)";
    EXPECT_FALSE(n.stop) << n.name;
  }
  // 시간 맞춘 구간: 지나올 때 평균 속도 (각도 / 0.1 s), 처음 구간 (look → far1) 은 기본 속도 (0)
  EXPECT_NEAR(r[1].speed_dps, 30.0, 1e-9) << "far_grasp ← far6: 지나올 때 J2 3° / 0.1 s";
  EXPECT_NEAR(r[5].speed_dps, 30.0, 1e-9) << "far1 ← far2";
  EXPECT_DOUBLE_EQ(r[6].speed_dps, 0.0) << "look ← far1: 지나올 때 기본 속도";
  // 작업 안 되돌아가기 (control_master start_empty_retreat): 그리퍼 없는 팔 동작 하나, 끝 = look, 표 구간은 지나올 때 속도 (30 °/s) 넘지 않음
  const auto acts = manipulation::path_actions(manipulation::ArmPath{"empty_retreat_exit", r, false}, PathTiming{},
    manipulation::GripperValues{}, r.front().q);
  ASSERT_EQ(acts.size(), 1u);
  EXPECT_FALSE(acts[0].is_gripper);
  EXPECT_EQ(acts[0].pose, "look");
  ASSERT_FALSE(acts[0].points.empty());
  EXPECT_TRUE(same_q(acts[0].points.back().q, look.q));
  EXPECT_GT(acts[0].duration_s, 0.5) << "표 구간 5 개 (0.1 s 씩) + look 까지";
  // 표 구간 (far6 → … → far1) 은 지나올 때 속도 안: far1 에 닿기 전 점들
  std::vector<double> prev = r.front().q;
  double t_prev = 0.0, vmax = 0.0;
  for (const auto & p : acts[0].points) {
    if (same_q(prev, path[1].q)) {break;}
    double d = 0.0;
    for (std::size_t j = 0; j < p.q.size(); ++j) {d = std::max(d, std::fabs(p.q[j] - prev[j]) / kDeg);}
    if (p.t > t_prev) {vmax = std::max(vmax, d / (p.t - t_prev));}
    prev = p.q;
    t_prev = p.t;
    if (std::fabs(p.q[1] - path[1].q[1]) < 1e-9) {break;}
  }
  EXPECT_LE(vmax, 30.0 * 1.05);
  // 표 점 이름을 안 주면 빈 목록
  EXPECT_TRUE(retrace_nodes(path, 6, kFree, at_look).empty());
  // 들어 올린 끝 (far_lift) 은 되돌아가기 점 아님 (다시 쥐기 뒤 빈손이 먼 퍽 자리로 다시 내려가지 않게)
  EXPECT_FALSE(manipulation::farpick_free_node("far_lift"));
}

// 표 재생 점 이름 (farpick_free_node)
TEST(EmptyRetreat, FarpickFreeNodeNames)
{
  using manipulation::farpick_free_node;
  EXPECT_TRUE(farpick_free_node("far1"));
  EXPECT_TRUE(farpick_free_node("far63"));
  EXPECT_TRUE(farpick_free_node("far_grasp"));
  EXPECT_FALSE(farpick_free_node("far"));
  EXPECT_FALSE(farpick_free_node("far_lift"));
  EXPECT_FALSE(farpick_free_node("far_re_grasp"));
  EXPECT_FALSE(farpick_free_node("far_re_down"));
  EXPECT_FALSE(farpick_free_node("farx1"));
  EXPECT_FALSE(farpick_free_node("lp_in3"));
  EXPECT_FALSE(farpick_free_node("staging"));
}

// 실제 로봇 사례: 먼 퍽 표 → 세워 내려놓고 (re_place …) 다시 보기 (re_observe) → 다시 쥐기 (re_grasp) 빈손 → 지나온 길에 re_observe ·
//   far_lift (먼 퍽 자리로 다시 내려가지 않게 뺀 점) → retrace 빈 목록 → "빈손 되돌아가기 경로 없음 (모르는 구간)" → HOME 거절 → 사람이 home force.
//   regrasp_retreat_nodes: re_up → re_above → 관찰 자세 (control_master 가 home_from_look 을 붙임)
TEST(EmptyRetreat, RegraspEmptyGoesUpToAboveThenLook)
{
  const PathNode look = node("look", {0, -10, 40, 60, 0});
  std::vector<PathNode> path{look, node("far1", {6, 3, 30, 50, 0}, false, "open"), node("far_grasp", {7, 6, 30, 50, 0}, false, "close"),
    node("far3", {8, 9, 30, 50, 0}), node("far_lift", {12, 10, 30, 50, 0}, false, "", true),
    node("re_place_above", {20, 20, 30, 40, 0}, false, "", false, 15.0), node("re_place_down", {20, 25, 30, 40, 0}, false, "", false, 15.0),
    node("re_place", {20, 28, 30, 40, 0}, false, "release", false, 15.0), node("re_place_up", {20, 25, 30, 40, 0}, false, "", false, 15.0),
    node("re_place_clear", {20, 20, 30, 40, 0}, false, "", false, 15.0), node("re_observe", {20, 15, 30, 40, 0}, false, "", true),
    node("re_above", {21, 20, 31, 40, 0}, false, "open", true, 15.0), node("re_down", {21, 25, 31, 40, 0}, false, "", false, 15.0),
    node("re_grasp", {21, 28, 31, 40, 0}, false, "close", false, 15.0), node("re_up", {21, 25, 31, 40, 0}, false, "", false, 15.0),
    node("re_lift", {21, 20, 31, 40, 0}, false, "", false, 15.0)};
  path[11].hold_s = 0.3;
  std::vector<std::string> free = kFree;
  free.push_back("re_above");
  free.push_back("re_place_clear");
  for (const auto & n : path) {if (manipulation::farpick_free_node(n.name)) {free.push_back(n.name);}}
  // retrace 만으로는: re_observe 를 모름 → 빈 목록
  EXPECT_TRUE(retrace_nodes(path, 14, free, [&](const PathNode & n) {return same_q(n.q, look.q);}).empty());
  const PathNode p0 = node("look", {0, -10, 40, 60, 0}, false, "", true);
  const auto r = manipulation::regrasp_retreat_nodes(path, 14, p0);
  ASSERT_EQ(r.size(), 3u);
  EXPECT_EQ(r[0].name, "re_up");
  EXPECT_EQ(r[1].name, "re_above");
  EXPECT_EQ(r[2].name, "look");
  EXPECT_TRUE(same_q(r[1].q, path[11].q));
  EXPECT_EQ(r[2].q, p0.q);
  EXPECT_DOUBLE_EQ(r[1].speed_dps, 15.0) << "re_up → re_above = 지나온 re_above → re_down 속도 (퍽 근처 느리게)";
  for (const auto & n : r) {
    EXPECT_TRUE(n.grip.empty()) << n.name;
    EXPECT_FALSE(n.stop) << n.name;
    EXPECT_DOUBLE_EQ(n.hold_s, 0.0) << n.name;
    EXPECT_DOUBLE_EQ(n.dt_s, 0.0) << n.name;
  }
  const auto acts = manipulation::path_actions(manipulation::ArmPath{"empty_retreat_exit", r, false}, PathTiming{},
    manipulation::GripperValues{}, r.front().q);
  ASSERT_FALSE(acts.empty());
  for (const auto & a : acts) {EXPECT_FALSE(a.is_gripper);}
  EXPECT_EQ(acts.back().pose, "look");
  // 다시 쥐기 모양이 아니면 (re_up 아님 · re_down ≠ re_up · 앞 점 이름 다름 · 기준점 없음 · 범위 밖) 빈 목록
  EXPECT_TRUE(manipulation::regrasp_retreat_nodes(path, 13, p0).empty());
  EXPECT_TRUE(manipulation::regrasp_retreat_nodes(path, 15, p0).empty());
  EXPECT_TRUE(manipulation::regrasp_retreat_nodes(path, 14, PathNode{}).empty());
  EXPECT_TRUE(manipulation::regrasp_retreat_nodes(path, 99, p0).empty());
  EXPECT_TRUE(manipulation::regrasp_retreat_nodes(path, 2, p0).empty());
  auto odd = path;
  odd[12].q[1] += 0.01;   // re_down ≠ re_up
  EXPECT_TRUE(manipulation::regrasp_retreat_nodes(odd, 14, p0).empty());
  const auto pick = pick_nodes(look);   // 보통 PICK (up · grasp · down · above): 이름이 달라 안 씀 (retrace 로 됨)
  EXPECT_TRUE(manipulation::regrasp_retreat_nodes(pick, 4, p0).empty());
}

// 쥔 폭 잘못 (grasp_width_release): 먼 퍽 표 닫는 점 (far_grasp) 에서 바로 (위 점 안 거침) 표 점 거꾸로 → look
TEST(EmptyRetreat, FarpickCloseItselfBackToLook)
{
  const PathNode look = node("look", {0, -10, 40, 60, 0});
  std::vector<PathNode> path{look, node("far1", {6, 3, 30, 50, 0}, false, "open")};
  for (int i = 2; i <= 6; ++i) {
    PathNode n = node(i == 5 ? "far_grasp" : "far" + std::to_string(i), {5.0 + i, 3.0 * i, 30, 50, 0}, false, i == 5 ? "close" : "");
    n.dt_s = 0.1;
    path.push_back(n);
  }
  path.push_back(node("far_lift", {12, 10, 30, 50, 0}, false, "", true));
  std::vector<std::string> free = kFree;
  for (const auto & n : path) {if (manipulation::farpick_free_node(n.name)) {free.push_back(n.name);}}
  const auto r = retrace_nodes(path, 5, free, [&](const PathNode & n) {return same_q(n.q, look.q);});   // 5 = far_grasp
  ASSERT_EQ(r.size(), 6u);
  EXPECT_EQ(r.front().name, "far_grasp");
  EXPECT_EQ(r[1].name, "far4");
  EXPECT_EQ(r.back().name, "look");
  EXPECT_TRUE(only_reversed_segments(path, r));
  for (const auto & n : r) {EXPECT_TRUE(n.grip.empty()) << n.name;}
  // 보통 PICK 쥐는 점 (grasp ← down) 은 down 을 모름 → 빈 목록 (그 닫기는 그 자리 다시 쥐기가 됨)
  const auto pick = pick_nodes(look);
  EXPECT_TRUE(retrace_nodes(pick, 3, kFree, [&](const PathNode & n) {return same_q(n.q, look.q);}).empty());
}

// 실제 로봇 사례: 먼 누운 퍽을 표로 쥐어 (lp_in* · reloc_grasp · lp*) 가까이 옮겨 놓고 (reloc_put …) 관찰 점 (reloc_observe) 뒤 받침 세우기 쥐기 (grasp) 빈손 →
//   지나온 길에 표 점 (lp*) 이 있어 retrace 가 모름 → "빈손 되돌아가기 경로 없음 (모르는 구간)" 으로 경로 도중 멈춤 → 사람이 home force.
//   park_retreat_nodes (받침 세우기 경로에서만): up → above → 관찰 자세 (보통 PICK 의 LOOK → 퍽 위 를 거꾸로)
TEST(EmptyRetreat, ParkGraspEmptyGoesUpToAboveThenLook)
{
  const PathNode look = node("look", {0, -10, 40, 60, 0});
  std::vector<PathNode> path{look, node("lp_in1", {5, 0, 35, 55, 0}, false, "open"), node("lp_in_hover", {7, 4, 32, 52, 0}, false, "", true),
    node("reloc_grasp", {7, 6, 32, 52, 0}, false, "close"), node("lp9", {7, 4, 32, 52, 0}), node("reloc_lift", {6, 0, 33, 53, 0}, false, "", true),
    node("reloc_put_above", {20, 15, 30, 40, 0}, false, "", true, 15.0), node("reloc_put_down", {20, 22, 30, 40, 0}, false, "", false, 15.0),
    node("reloc_put", {20, 25, 30, 40, 0}, false, "open", false, 15.0), node("reloc_put_up", {20, 22, 30, 40, 0}, false, "", false, 15.0),
    node("reloc_put_clear", {20, 15, 30, 40, 0}, false, "", false, 15.0), node("reloc_observe", {20, 8, 30, 45, 0}, false, "", true),
    node("above", {22, 16, 30, 40, 0}, false, "open", true), node("down", {22, 22, 30, 40, 0}, false, "", false, 15.0),
    node("grasp", {22, 26, 30, 40, 0}, false, "close", false, 15.0), node("up", {22, 22, 30, 40, 0}, false, "", false, 15.0),
    node("lift", {22, 16, 30, 40, 0}, false, "", false, 15.0), node("park_over", {0, -20, 60, 30, 0}), node("park_re_down", {0, -25, 60, 30, 0}, false, "close")};
  std::vector<std::string> free = kFree;
  free.push_back("reloc_observe");
  for (const auto & n : path) {if (manipulation::lyingpick_free_node(n.name) || manipulation::farpick_free_node(n.name)) {free.push_back(n.name);}}
  // 지금 길: lp9 (표로 쥔 채 든 점) 를 모름 → 빈 목록
  EXPECT_TRUE(retrace_nodes(path, 15, free, [&](const PathNode & n) {return same_q(n.q, look.q);}).empty());
  const PathNode exit = node("look", {0, -10, 40, 60, 0}, false, "", true);
  EXPECT_TRUE(manipulation::regrasp_retreat_nodes(path, 15, exit).empty());   // 다시 쥐기 (re_*) 모양 아님
  const auto r = manipulation::park_retreat_nodes(path, 15, exit);
  ASSERT_EQ(r.size(), 3u);
  EXPECT_EQ(r[0].name, "up");
  EXPECT_EQ(r[1].name, "above");
  EXPECT_EQ(r[2].name, "look");
  EXPECT_TRUE(same_q(r[1].q, path[12].q));
  EXPECT_DOUBLE_EQ(r[1].speed_dps, 15.0) << "up → above = 지나온 above → down 속도 (퍽 근처 느리게)";
  for (const auto & n : r) {
    EXPECT_TRUE(n.grip.empty()) << n.name;
    EXPECT_FALSE(n.stop) << n.name;
    EXPECT_DOUBLE_EQ(n.hold_s, 0.0) << n.name;
  }
  const auto acts = manipulation::path_actions(manipulation::ArmPath{"empty_retreat_exit", r, false}, PathTiming{}, manipulation::GripperValues{},
    r.front().q);
  ASSERT_FALSE(acts.empty());
  for (const auto & a : acts) {EXPECT_FALSE(a.is_gripper);}
  EXPECT_EQ(acts.back().pose, "look");
  // 받침 세우기 경로가 아니면 (park_ 점 없음) 안 씀 — 보통 PICK 은 retrace 로 됨
  auto plain = path;
  plain.resize(17);
  EXPECT_TRUE(manipulation::park_retreat_nodes(plain, 15, exit).empty());
  EXPECT_TRUE(manipulation::park_retreat_nodes(pick_nodes(look), 4, exit).empty());
  // up 이 아닌 점 · up ≠ down · 기준점 없음 → 빈 목록
  EXPECT_TRUE(manipulation::park_retreat_nodes(path, 14, exit).empty());
  auto odd = path;
  odd[13].q[1] += 0.01;
  EXPECT_TRUE(manipulation::park_retreat_nodes(odd, 15, exit).empty());
  EXPECT_TRUE(manipulation::park_retreat_nodes(path, 15, PathNode{}).empty());
}

// 다시 본 퍽으로 다시 계획한 받침 세우기 (relook_start 에서, 멀면 옮기기 쥐기 reloc_*) 의 빈손: 첫 점 (지금 관찰 자세) 은 복귀 경로가 없어 retrace 가
//   끝점을 못 찾음 → 쥐기 grasp 는 up → above → look, 옮기기 쥐기 reloc_grasp 는 reloc_up → reloc_above → look
TEST(EmptyRetreat, ReplannedParkTailRetreatsToLook)
{
  const PathNode start = node("relook_start", {20, 8, 30, 45, 0});
  const PathNode exit = node("look", {0, -10, 40, 60, 0}, false, "", true);
  std::vector<PathNode> tail{start, node("reloc_above", {24, 14, 30, 40, 0}, false, "open", true), node("reloc_down", {24, 20, 30, 40, 0}, false, "", false, 15.0),
    node("reloc_grasp", {24, 24, 30, 40, 0}, false, "close", false, 15.0), node("reloc_up", {24, 20, 30, 40, 0}, false, "", false, 15.0),
    node("reloc_lift", {24, 14, 30, 40, 0}, false, "", false, 15.0), node("reloc_put_clear", {18, 15, 30, 40, 0}, false, "", false, 15.0),
    node("above", {18, 16, 30, 40, 0}, false, "open", true), node("down", {18, 22, 30, 40, 0}, false, "", false, 15.0),
    node("grasp", {18, 26, 30, 40, 0}, false, "close", false, 15.0), node("up", {18, 22, 30, 40, 0}, false, "", false, 15.0),
    node("lift", {18, 16, 30, 40, 0}, false, "", false, 15.0), node("park_over", {0, -20, 60, 30, 0})};
  std::vector<std::string> free = kFree;
  free.push_back("reloc_observe");
  EXPECT_TRUE(retrace_nodes(tail, 4, free, [&](const PathNode & n) {return same_q(n.q, exit.q);}).empty());
  const auto a = manipulation::park_retreat_nodes(tail, 4, exit);
  ASSERT_EQ(a.size(), 3u);
  EXPECT_EQ(a[0].name, "reloc_up");
  EXPECT_EQ(a[1].name, "reloc_above");
  EXPECT_EQ(a[2].name, "look");
  const auto b = manipulation::park_retreat_nodes(tail, 10, exit);
  ASSERT_EQ(b.size(), 3u);
  EXPECT_EQ(b[0].name, "up");
  EXPECT_EQ(b[1].name, "above");
}

// 옮기기 점 속도 (regrasp_transit_dps): 다시 쥐기 (regrasp_nodes) · 옮겨 놓기 · 먼 퍽 다시 쥐기의 위 · 비킨 점만 빠르게, 닿는 · 놓는 · 쥐는 점과
//   닿은 바로 뒤 올리기는 그대로. 시간 맞춘 점 · 받침 점도 그대로. 자세 · 그리퍼 · 끝 시각만 바뀜
TEST(TransitSpeed, OnlyAboveAndClearNodes)
{
  const double slow = 30.0, fast = 112.0;
  std::vector<PathNode> nodes;
  for (const char * n : {"re_place_above", "re_place_down", "re_place", "re_place_up", "re_place_clear", "re_observe", "re_above", "re_down", "re_grasp",
      "re_up", "re_lift", "reloc_put_above", "reloc_put_down", "reloc_put", "reloc_put_up", "reloc_put_clear", "reloc_above", "reloc_down", "reloc_grasp",
      "reloc_up", "reloc_lift", "far_put_above", "far_put_down", "far_put", "far_put_up", "far_put_clear", "far_re_above", "far_re_down", "far_re_grasp",
      "far_re_up", "far_re_lift", "above", "down", "grasp", "up", "lift", "drop", "clear"})
  {
    PathNode p;
    p.name = n;
    p.q = {0, 0, 0, 0, 0};
    p.speed_dps = std::string(n).find("observe") != std::string::npos || std::string(n) == "reloc_above" ? 0.0 : slow;
    nodes.push_back(p);
  }
  PathNode timed = nodes.front();
  timed.name = "reloc_lift";
  timed.dt_s = 0.1;   // 누운 퍽 표 재생 끝 (시간 맞춤) — 그대로
  nodes.push_back(timed);
  PathNode stand = nodes.front();
  stand.name = "re_lift";
  stand.stand = true;
  nodes.push_back(stand);
  const std::set<std::string> want{"re_place_above", "re_place_clear", "re_above", "re_lift", "reloc_put_above", "reloc_put_clear", "reloc_lift",
    "far_put_above", "far_put_clear", "far_re_above", "far_re_lift"};
  EXPECT_EQ(manipulation::set_transit_speed(nodes, fast), 11);
  for (std::size_t i = 0; i + 2 < nodes.size(); ++i) {
    const auto & n = nodes[i];
    EXPECT_EQ(manipulation::transit_node(n.name), want.count(n.name) == 1) << n.name;
    const double expect = want.count(n.name) ? fast : (n.name.find("observe") != std::string::npos || n.name == "reloc_above" ? 0.0 : slow);
    EXPECT_DOUBLE_EQ(n.speed_dps, expect) << n.name;
  }
  EXPECT_DOUBLE_EQ(nodes[nodes.size() - 2].speed_dps, slow) << "시간 맞춘 점";
  EXPECT_DOUBLE_EQ(nodes.back().speed_dps, slow) << "받침 점";
  EXPECT_EQ(manipulation::set_transit_speed(nodes, 0.0), 0) << "0 = 안 바꿈";
}

// 다시 쥐기 모양 경로: 옮기기 점만 빠르게 하면 놓는 점 · 다시 쥐는 점 자세는 같고 (마지막 점 · 그리퍼 동작 그대로) 시간만 줄어듦
TEST(TransitSpeed, SameReleaseAndGraspPosesShorterTime)
{
  const double slow = 30.0;
  auto nd = [](const char * n, std::vector<double> qd, double sp, const char * g = "") {
      PathNode p;
      p.name = n;
      for (double v : qd) {p.q.push_back(v * M_PI / 180.0);}
      p.speed_dps = sp;
      p.grip = g;
      return p;
    };
  ArmPath path{"regrasp", {nd("lift", {10, 20, -30, 40, 0}, 0.0), nd("re_place_above", {30, 25, -35, 45, 0}, slow),
    nd("re_place_down", {30, 30, -35, 45, 0}, slow), nd("re_place", {30, 34, -35, 45, 0}, slow, "release"), nd("re_place_up", {30, 30, -35, 45, 0}, slow),
    nd("re_place_clear", {30, 25, -35, 45, 0}, slow), nd("re_above", {38, 24, -34, 44, 10}, slow, "open"), nd("re_down", {38, 30, -34, 44, 10}, slow),
    nd("re_grasp", {38, 34, -34, 44, 10}, slow, "close"), nd("re_up", {38, 30, -34, 44, 10}, slow), nd("re_lift", {38, 24, -34, 44, 10}, slow)}, false};
  path.nodes[1].stop = true;   // 내려놓을 자리 위 멈춤 (control_master settle_at place_settle_s)
  path.nodes[1].hold_s = 0.2;
  PathTiming t;
  t.speed_dps = 112.0;
  t.accel_dps2 = 280.0;
  const auto before = manipulation::path_actions(path, t, manipulation::GripperValues{});
  ArmPath fast = path;
  EXPECT_EQ(manipulation::set_transit_speed(fast.nodes, 112.0), 4);
  const auto after = manipulation::path_actions(fast, t, manipulation::GripperValues{});
  ASSERT_EQ(before.size(), after.size());
  for (std::size_t i = 0; i < before.size(); ++i) {
    EXPECT_EQ(before[i].is_gripper, after[i].is_gripper);
    EXPECT_EQ(before[i].pose, after[i].pose);
    if (before[i].is_gripper) {EXPECT_DOUBLE_EQ(before[i].gripper, after[i].gripper); continue;}
    for (int j = 0; j < 5; ++j) {EXPECT_DOUBLE_EQ(before[i].points.back().q[j], after[i].points.back().q[j]) << before[i].pose;}
    const bool contact = before[i].pose == "re_place" || before[i].pose == "re_grasp" || before[i].pose == "re_place_up" || before[i].pose == "re_up" ||
      before[i].pose == "re_place_down" || before[i].pose == "re_down";
    if (contact) {EXPECT_NEAR(before[i].duration_s, after[i].duration_s, 1e-9) << before[i].pose << " 닿는 구간은 그대로";}
  }
  const double tb = manipulation::arm_motion_time(before), ta = manipulation::arm_motion_time(after);
  EXPECT_LT(ta, tb - 0.3) << "옮기기 점 4 개 (각 5–8°) 를 30 → 112 °/s";
  std::printf("다시 쥐기 모양: 팔 동작 %.2f s → %.2f s\n", tb, ta);
}

// 쥔 그대로 보정 (held_image_compensate): 실제 store_<칸> 첫 놓는 점 자세에서 앞으로 쥠 4.2 mm (실제 로봇 S5) 를 넣는 점 옮김으로.
//   control_master release_rot 와 같은 식 (팀 명령 rad → 틱 → 모델). 옮김 = 놓는 자세 손 x 방향으로 +4.2 mm (퍽이 손끝보다 손 −x 에 있으므로),
//   옮긴 경로의 놓는 점 손끝 + 손 안 퍽 = 칸 쪽 원래 손끝 (xy). 칸마다 옮김 · 관절 변화 출력
TEST(RealPaths, HeldImageCompShiftOnStorePaths)
{
  const auto m = kin_data::model();
  const std::vector<Eigen::Vector2d> cells{{-0.102, 0.1131}, {-0.102, 0.0681}, {-0.042, 0.1131}, {0.078, -0.1169}, {0.078, 0.1131}, {0.018, 0.1131},
    {0.018, -0.1169}, {-0.042, 0.0681}, {0.078, -0.0719}, {0.078, 0.0681}, {0.018, 0.0681}, {0.018, -0.0719}};   // config/rack_cells.yaml
  manipulation::HeldImageVerdict v;
  v.judged = v.bad = true;
  v.front_mm = 4.2;
  const auto c = manipulation::held_image_comp(v, 8.5, 10.0);
  ASSERT_TRUE(c.ok);
  auto ticks_of = [&](const std::vector<double> & cq) {
      manipulation::Ticks t;
      for (int j = 0; j < 5; ++j) {t[j] = static_cast<int>(std::lround(m.zero_tick[j] + cq[j] * 4096.0 / (2.0 * M_PI)));}
      return manipulation::ticks_to_q(m, t);
    };
  for (int k = 1; k <= 12; ++k) {
    ArmPath p;
    ASSERT_TRUE(load_path("store_S" + std::to_string(k), p)) << k;
    const auto rel = std::find_if(p.nodes.begin(), p.nodes.end(), [](const PathNode & n) {return n.grip == "release";});
    ASSERT_NE(rel, p.nodes.end()) << k;
    const auto q = ticks_of(rel->q);
    const Eigen::Matrix3d R = manipulation::arm_frames(m, q)[4].block<3, 3>(0, 0);
    const Eigen::Vector2d s = manipulation::held_image_comp_shift(m, R, c.hand_m);
    const Eigen::Vector3d hx = R * manipulation::tcp_hand_to_link5(m, Eigen::Vector3d::UnitX());
    const Eigen::Vector2d radial = (cells[k - 1] - m.j1_xy).normalized();
    // 옮김 = 손 x (수평 성분) × 4.2 mm, 퍽 중심이 원래 손끝 자리로
    EXPECT_NEAR((s - hx.head<2>() * 0.0042).norm(), 0.0, 1e-9) << k;
    const Eigen::Vector2d tip0 = manipulation::arm_grip(m, q).tip.head<2>();
    const Eigen::Vector2d puck_after = tip0 + s + (R * manipulation::tcp_hand_to_link5(m, c.hand_m)).head<2>();
    EXPECT_NEAR((puck_after - tip0).norm(), 0.0, 1e-9) << k;
    ArmPath p2 = p;
    manipulation::CellShiftReport rep;
    std::string why;
    const bool ok = manipulation::shift_cell_path(m, p2, true, cells[k - 1], s, rep, why);
    const auto rel2 = std::find_if(p2.nodes.begin(), p2.nodes.end(), [](const PathNode & n) {return n.grip == "release";});
    const Eigen::Vector2d tip2 = manipulation::arm_grip(m, ticks_of(rel2->q)).tip.head<2>();
    std::printf("S%d: 놓는 손 x (%+.2f, %+.2f, %+.2f) · 반지름 바깥과 %.0f° → 앞으로 쥠 4.2 mm 옮김 (%+.1f, %+.1f) mm · 칸 근처 옮김 %s (놓는 점 손끝 %+.1f, %+.1f mm · 관절 최대 %.2f°)\n",
      k, hx.x(), hx.y(), hx.z(), std::acos(std::clamp(hx.head<2>().normalized().dot(radial), -1.0, 1.0)) * 180.0 / M_PI, s.x() * 1000.0,
      s.y() * 1000.0, ok ? "됨" : why.c_str(), (tip2 - tip0).x() * 1000.0, (tip2 - tip0).y() * 1000.0, rep.max_joint_deg);
    if (ok) {EXPECT_NEAR((tip2 - tip0 - s).norm(), 0.0, 0.0005) << k;}
  }
}

// ============================================================================ LOOK_RACK 관찰 자세 → 칸 경로 잇기
namespace
{
ArmPath load_look_rack(const std::string & want)
{
  std::string rl = ARM_PATHS_YAML;
  rl.replace(rl.rfind("arm_paths.yaml"), std::string("arm_paths.yaml").size(), "rack_look_paths.yaml");
  ArmPath p;
  load_path(want, p, rl);
  return p;
}
bool same_lr(const std::vector<double> & a, const std::vector<double> & b, double tol = 1e-12)
{
  if (a.size() != b.size()) {return false;}
  for (std::size_t j = 0; j < a.size(); ++j) {if (std::fabs(a[j] - b[j]) > tol) {return false;}}
  return true;
}
}  // namespace

// 12 칸 실제 경로: d30 · store_<칸> 모두 관찰 자세 (look_rack 끝, 실측 0.05 rad 어긋남) 에서 지나온 관찰 경로를 거꾸로 → 칸 경로 그대로.
//   S10: look_rack 이 칸 경로 lift_real 을 그대로 지남 → lift_real 에서 이음. 나머지 11 칸: 관찰 경로 중간 높이를 올려 둬서
//   칸 경로 점과 겹치지 않음 → look_rack 첫 점 lift (= 칸 경로 첫 점, 박스 위 기준점) 까지 거꾸로 (S3 은 r8 … lift_real 도 거꾸로) → 칸 경로 처음부터.
//   look_rack 을 기준점에서 갔든 집은 자리에서 갔든 (첫 점 pick_end) 같음. 새 자세 없음 (지금 자세 · 관찰 경로 점 · 칸 경로 점뿐)
TEST(LookRackJoin, RealPathsAllCellsJoinThroughRecordedPoints)
{
  for (int k = 1; k <= 12; ++k) {
    const std::string s = "S" + std::to_string(k);
    const ArmPath look = load_look_rack("look_rack_" + s);
    ASSERT_GE(look.nodes.size(), 3u) << s;
    ArmPath look_pick = look;   // join_from_pick_end 로 간 look_rack
    look_pick.nodes.front().q = {0.3, 0.2, -0.5, 0.4, 0.1};
    look_pick.nodes.front().name = "pick_end";
    for (const std::string & name : {"store_" + s + "_d30", "store_" + s}) {
      ArmPath store;
      ASSERT_TRUE(load_path(name, store)) << name;
      std::string why;
      // 잇지 않으면: 관찰 자세에서 출발 검사 거절 (S2 관절 1 2.172 rad)
      EXPECT_FALSE(manipulation::path_start_ok(store, look.nodes.back().q, 0.15, why)) << name;
      for (const ArmPath * lp : std::vector<const ArmPath *>{&look, &look_pick}) {
        std::vector<double> q_now = lp->nodes.back().q;
        q_now[0] += 0.05;
        q_now[3] -= 0.05;
        std::size_t j = 0;
        const auto out = manipulation::look_rack_back_join(store, *lp, look.nodes.front(), q_now, 0.15, why, 1e-3, &j);
        ASSERT_FALSE(out.empty()) << name << ": " << why;
        const bool at_lift_real = k == 10;
        EXPECT_EQ(j, at_lift_real ? 2u : 1u) << name;   // out 끝 (store.size − j) 점 = store.nodes[j..] (store_recover 는 앞을 store.nodes[0..j) 로 되돌림)
        // store_recover 가 쓰는 경로: 이은 앞부분 → 원래 칸 경로 앞 (박스 위 기준점) = 원래 칸 경로와 같음
        std::vector<PathNode> rec(store.nodes.begin(), store.nodes.begin() + static_cast<std::ptrdiff_t>(j));
        rec.insert(rec.end(), out.end() - static_cast<std::ptrdiff_t>(store.nodes.size() - j), out.end());
        ASSERT_EQ(rec.size(), store.nodes.size()) << name;
        for (std::size_t i = 0; i < rec.size(); ++i) {EXPECT_TRUE(same_lr(rec[i].q, store.nodes[i].q)) << name << " " << i;}
        EXPECT_TRUE(same_lr(out.front().q, q_now)) << name;
        EXPECT_TRUE(manipulation::path_start_ok(ArmPath{name, out, true}, q_now, 0.15, why)) << name;
        // S10: 첫 점 (lift) 만 관찰 자세로 바뀜. 나머지: [관찰 자세, look_rack n-2 … 1 거꾸로, 칸 경로 전체] (S1 209 점 → 211 점)
        ASSERT_EQ(out.size(), store.nodes.size() + (at_lift_real ? 0 : look.nodes.size() - 1)) << name;
        EXPECT_TRUE(same_lr(out[1].q, look.nodes[look.nodes.size() - 2].q)) << name;   // look_rack 끝 바로 앞 점 (지나온 점)
        const std::size_t nb = out.size() - (store.nodes.size() - j + 1);   // 칸 경로 store.nodes[j − 1] 이 나오는 자리
        for (std::size_t i = 1; i < nb; ++i) {
          EXPECT_TRUE(same_lr(out[i].q, look.nodes[look.nodes.size() - 1 - i].q)) << name << " " << i;   // 지나온 관찰 경로 거꾸로
        }
        for (std::size_t i = nb; i < out.size(); ++i) {
          const PathNode & want = store.nodes[j - 1 + i - nb];
          EXPECT_TRUE(same_lr(out[i].q, want.q)) << name << " " << i;   // 칸 경로 (놓는 점 포함) 그대로
          EXPECT_EQ(out[i].grip, want.grip) << name << " " << i;
          EXPECT_EQ(out[i].stop, want.stop) << name << " " << i;
        }
      }
      // 관찰 자세가 아니면 (기준점 · 집은 자리 · 받침) 안 이음 → 칸 경로 그대로
      for (const auto & q : {store.nodes.front().q, look_pick.nodes.front().q, look.nodes[1].q}) {
        EXPECT_TRUE(manipulation::look_rack_back_join(store, look, look.nodes.front(), q, 0.15, why).empty()) << name;
      }
    }
  }
}

// lift_real 이 다르면 (칸 보정 · 다른 칸): look_rack 둘째 점까지 거꾸로 → look_rack 첫 점 (= 칸 경로 첫 점) → 칸 경로. 첫 점이 다르면 안 이음
TEST(LookRackJoin, FallbackThroughLookStartOrNothing)
{
  const ArmPath look{"look_rack_X", {node("lift", {10, 20, -30, 10, 10}), node("lift_real", {15, -20, -15, -10, 15}, false, "", false, 25.0),
    node("rack_look", {130, -30, -5, 10, -2}, false, "", true, 40.0)}, false};
  ArmPath store{"store_X_d30", {node("lift", {10, 20, -30, 10, 10}), node("lift_real", {18, -22, -15, -10, 15}),
    node("r2", {20, -25, -14, -9, 16}), node("in", {22, -30, -12, -8, 16}, false, "release")}, true};
  std::string why;
  const auto out = manipulation::look_rack_back_join(store, look, look.nodes.front(), look.nodes.back().q, 0.15, why);
  ASSERT_EQ(out.size(), 6u) << why;
  EXPECT_EQ(out[0].name, "look_rack_end");
  EXPECT_TRUE(same_lr(out[1].q, look.nodes[1].q));
  EXPECT_DOUBLE_EQ(out[1].speed_dps, 40.0);   // rack_look → lift_real = 지나온 lift_real → rack_look 구간 속도
  EXPECT_TRUE(same_lr(out[2].q, store.nodes[0].q));
  EXPECT_DOUBLE_EQ(out[2].speed_dps, 25.0);   // lift_real → lift = 지나온 lift → lift_real 구간 속도
  EXPECT_TRUE(same_lr(out[3].q, store.nodes[1].q));
  EXPECT_EQ(out[5].grip, "release");
  EXPECT_FALSE(out[1].stop);
  // look_rack 첫 점 ≠ 칸 경로 첫 점 → 새 직선이 필요 → 안 이음
  ArmPath store2 = store;
  store2.nodes.front().q[0] += 0.2;
  EXPECT_TRUE(manipulation::look_rack_back_join(store2, look, look.nodes.front(), look.nodes.back().q, 0.15, why).empty());
  // NaN · 관절 수 다름
  auto bad = look.nodes.back().q;
  bad[2] = std::nan("");
  EXPECT_TRUE(manipulation::look_rack_back_join(store, look, look.nodes.front(), bad, 0.15, why).empty());
  EXPECT_TRUE(manipulation::look_rack_back_join(store, look, look.nodes.front(), {0.1, 0.2}, 0.15, why).empty());
}

// STORE 경로 고르기: 출발 상태마다 (storeheld_paths true · storeheld_over_look true 가 최종 설정)
TEST(LookRackJoin, StorePickPerStartState)
{
  using manipulation::StorePick;
  using manipulation::store_path_pick;
  // 집은 자리 (PICK 끝, join_from_pick_end) · 기준점 · 다른 작업 끝 (관찰 자세 아님): 새 칸 경로
  EXPECT_EQ(store_path_pick(true, true, false, true), StorePick::kHeld);
  // 관찰 자세 아님 · 이을 수 없음 (받침 등): 새 칸 경로 (path_plan 의 출발 검사가 거절)
  EXPECT_EQ(store_path_pick(true, true, false, false), StorePick::kHeld);
  // LOOK_RACK 관찰 자세 · 거꾸로 이음 됨: 새 칸 경로 (실물 91 거절이 여기)
  EXPECT_EQ(store_path_pick(true, true, true, true), StorePick::kHeld);
  // LOOK_RACK 관찰 자세 · 못 이음: store_look_<칸> (관찰 흐름, store_match_grip_depth 포함) — 거절 대신
  EXPECT_EQ(store_path_pick(true, true, true, false), StorePick::kLook);
  // storeheld_over_look false: 관찰 자세면 store_look_<칸>
  EXPECT_EQ(store_path_pick(true, false, true, true), StorePick::kLook);
  EXPECT_EQ(store_path_pick(true, false, false, true), StorePick::kHeld);
  // 새 칸 경로 없음 (storeheld_paths false · 이름 없음): 기록 경로 흐름
  EXPECT_EQ(store_path_pick(false, false, true, false), StorePick::kLook);
  EXPECT_EQ(store_path_pick(false, false, false, false), StorePick::kPlain);
}

// 실제 경로로 출발 상태마다 control_master 와 같은 차례 (출발 검사 → 집은 자리 → 관찰 자세 거꾸로) 로 고르고 그 경로 첫 점이 출발 검사 통과
TEST(LookRackJoin, RealPathsStartStates)
{
  using manipulation::StorePick;
  ArmPath fold;
  ASSERT_TRUE(load_path("fold", fold));
  for (int k = 1; k <= 12; ++k) {
    const std::string s = "S" + std::to_string(k);
    const ArmPath look = load_look_rack("look_rack_" + s);
    ArmPath d30, store_look;
    ASSERT_TRUE(load_path("store_" + s + "_d30", d30));
    store_look = load_look_rack("store_look_" + s);
    ASSERT_FALSE(store_look.nodes.empty()) << s;
    struct State {const char * what; std::vector<double> q; bool from_pick; bool look_known; StorePick want;};
    const std::vector<State> states{
      {"집은 자리 (PICK 끝)", {0.3, 0.2, -0.5, 0.4, 0.1}, true, false, StorePick::kHeld},
      {"기준점 (staging)", d30.nodes.front().q, false, false, StorePick::kHeld},
      {"LOOK_RACK 끝", look.nodes.back().q, false, true, StorePick::kHeld},
      {"LOOK_RACK 끝 · 간 경로 모름", look.nodes.back().q, false, false, StorePick::kLook},
      {"P0 (다른 작업 끝)", fold.nodes.front().q, false, false, StorePick::kHeld},
    };
    for (const auto & st : states) {
      std::string why;
      const bool look_ok = manipulation::path_start_ok(store_look, st.q, 0.15, why);
      bool joinable = manipulation::path_start_ok(d30, st.q, 0.15, why) || st.from_pick;
      std::vector<PathNode> joined;
      if (!joinable && st.look_known) {
        joined = manipulation::look_rack_back_join(d30, look, look.nodes.front(), st.q, 0.15, why);
        joinable = !joined.empty();
      }
      const auto pick = manipulation::store_path_pick(true, true, look_ok, joinable);
      EXPECT_EQ(pick, st.want) << s << " " << st.what;
      if (pick == StorePick::kLook) {EXPECT_TRUE(look_ok) << s << " " << st.what;}
      if (!joined.empty()) {EXPECT_TRUE(manipulation::path_start_ok(ArmPath{"j", joined, true}, st.q, 0.15, why)) << s;}
    }
  }
}

// LOOK_RACK 관찰 자세에서 PUTBACK PLACE · 다시 보낸 LOOK_RACK. PLACE 경로 = [기준점 (staging_pose = 칸 경로 첫 점), 놓을 자리 위, …]
//   → 관찰 경로 거꾸로 → lift_real (S3 r8 … lift_real) → 기준점 → 놓을 자리 위. LOOK_RACK 다시 = 같은 칸 [관찰 자세, 바로 앞 점, 관찰 자세] · 다른 칸 = 지나온 점 · 그 칸 경로 점만
TEST(LookRackJoin, PlaceAndLookRackFromLookEnd)
{
  ArmPath s1;
  ASSERT_TRUE(load_path("store_S1_d30", s1));
  const PathNode staging = s1.nodes.front();
  for (int k = 1; k <= 12; ++k) {
    const std::string s = "S" + std::to_string(k);
    const ArmPath look = load_look_rack("look_rack_" + s);
    ASSERT_GE(look.nodes.size(), 3u) << s;
    std::vector<double> q_now = look.nodes.back().q;
    q_now[1] += 0.04;
    std::string why;
    // PUTBACK PLACE (박스 위 놓을 자리, 관절값은 아무 박스 위 자세)
    const ArmPath place{"place", {staging, node("above", {10, 5, -40, 30, 0}, false, "", false, 20.0)}, true};
    EXPECT_FALSE(manipulation::path_start_ok(place, q_now, 0.15, why)) << s;   // 잇지 않으면 거절 (최대 2.174 rad)
    const auto out = manipulation::look_rack_back_join(place, look, look.nodes.front(), q_now, 0.15, why);
    ASSERT_EQ(out.size(), look.nodes.size() + 1) << s << ": " << why;   // [지금, look n-2 … 1, 기준점, 위]
    EXPECT_TRUE(same_lr(out.front().q, q_now)) << s;
    for (std::size_t i = 1; i + 2 < out.size(); ++i) {EXPECT_TRUE(same_lr(out[i].q, look.nodes[look.nodes.size() - 1 - i].q)) << s << " " << i;}
    EXPECT_TRUE(same_lr(out[out.size() - 2].q, staging.q)) << s;
    EXPECT_EQ(out.back().name, "above");
    // 같은 칸 LOOK_RACK 다시: 바로 앞 점으로 올라갔다 다시 관찰 자세
    const auto again = manipulation::look_rack_back_join(look, look, look.nodes.front(), q_now, 0.15, why);
    ASSERT_EQ(again.size(), 3u) << s << ": " << why;
    EXPECT_TRUE(same_lr(again[1].q, look.nodes[look.nodes.size() - 2].q)) << s;
    EXPECT_TRUE(same_lr(again[2].q, look.nodes.back().q)) << s;
    // 다른 칸 LOOK_RACK: 결과 점은 지나온 관찰 경로 점 · 기준점 · 새 칸 관찰 경로 점뿐 (새 자세 없음)
    for (int m = 1; m <= 12; ++m) {
      if (m == k) {continue;}
      const ArmPath other = load_look_rack("look_rack_S" + std::to_string(m));
      const auto o = manipulation::look_rack_back_join(other, look, look.nodes.front(), q_now, 0.15, why);
      ASSERT_FALSE(o.empty()) << s << "→S" << m << ": " << why;
      EXPECT_TRUE(same_lr(o.back().q, other.nodes.back().q)) << s << "→S" << m;
      for (std::size_t i = 1; i < o.size(); ++i) {
        bool known = false;
        for (const auto & n : look.nodes) {known = known || same_lr(o[i].q, n.q);}
        for (const auto & n : other.nodes) {known = known || same_lr(o[i].q, n.q);}
        EXPECT_TRUE(known) << s << "→S" << m << " " << i;
      }
    }
  }
}

// 같은 정차에서 HOME 없이 다음 팔 작업 — 실제 경로로 끝 자세 × 다음 작업 출발 (control_master 와 같은 차례:
//   출발 검사 → P0 잇기 (skip_unfold) → 집은 자리 잇기 → LOOK_RACK 관찰 자세 잇기 → (from_look 작업) 보기 자세 복귀 경로로 P0 거쳐 (join_via_return))
//   이은 경로의 이웃 점은 모두 기록 경로 (unfold · fold · look · look_barcode · home_from_* · 칸 · look_rack) 의 점 또는 지금 자세
TEST(ChainMatrix, NoHomeBetweenTasksAtSameStop)
{
  std::map<std::string, ArmPath> P;
  for (const char * n : {"unfold", "fold", "look", "look_barcode", "home_from_look", "home_from_look_barcode", "rack_scan", "retrieve_S1", "store_S2_d30"}) {
    ASSERT_TRUE(load_path(n, P[n])) << n;
  }
  P["look_rack_S2"] = load_look_rack("look_rack_S2");
  const auto & unfold = P["unfold"];
  const auto & fold = P["fold"];
  const std::vector<double> home = unfold.nodes.front().q, p0 = fold.nodes.front().q, look_q = P["look"].nodes.back().q,
    lb = P["look_barcode"].nodes.back().q, staging = P["store_S2_d30"].nodes.front().q, rack_end = P["look_rack_S2"].nodes.back().q;
  std::vector<double> la = lb;   // LOOK_AT 자세 (look_barcode 끝에서 J1 0.4 · J4 0.3 rad)
  la[0] += 0.4;
  la[3] += 0.3;
  const std::vector<double> pick_end{0.30, 0.20, -0.50, 0.40, 0.10};
  const double tol = 0.15;
  // LOOK_AT 복귀 (control_master look_at_return): [지금, home_from_look_barcode …]
  auto pn = [](const char * name, const std::vector<double> & q) {PathNode n; n.name = name; n.q = q; return n;};
  auto look_at_return = [&](const std::vector<double> & q, ArmPath & out) {
      std::string w;
      ArmPath one{"la", {pn("la", la)}, false};
      if (!manipulation::path_start_ok(one, q, tol, w)) {return false;}
      out = ArmPath{"home_from_look_at", {pn("look_at_now", q)}, true};
      out.nodes.insert(out.nodes.end(), P["home_from_look_barcode"].nodes.begin(), P["home_from_look_barcode"].nodes.end());
      return true;
    };
  struct State {std::vector<double> q; bool from_pick; bool at_rack;};
  auto starts = [&](ArmPath path, const State & st, bool from_look, bool start_at_look) {
      std::string w;
      if (manipulation::path_start_ok(path, st.q, tol, w)) {return true;}
      if (start_at_look) {   // PICK: 관찰 자세에서 바로 퍽 위로 (start_at_look)
        ArmPath p = path;
        if (manipulation::skip_unfold(p, unfold)) {
          p.nodes.front() = P["look"].nodes.back();
          if (manipulation::path_start_ok(p, st.q, tol, w)) {return true;}
        }
      }
      ArmPath p = path;
      if (manipulation::skip_unfold(p, unfold) && manipulation::path_start_ok(p, st.q, tol, w)) {return true;}   // join_from_p0
      if (st.from_pick && path.nodes.size() >= 2) {return true;}                                                 // join_from_pick_end
      if (st.at_rack) {                                                                                            // join_from_look_rack_end
        const auto j = manipulation::look_rack_back_join(path, P["look_rack_S2"], P["look_rack_S2"].nodes.front(), st.q, tol, w);
        if (!j.empty()) {return manipulation::path_start_ok(ArmPath{"j", j, false}, st.q, tol, w);}
      }
      if (!from_look) {return false;}
      std::vector<ArmPath> rets{P["home_from_look"], P["home_from_look_barcode"]};
      ArmPath lr;
      if (look_at_return(st.q, lr)) {rets.push_back(lr);}
      for (const auto & r : rets) {
        if (!manipulation::path_start_ok(r, st.q, tol, w)) {continue;}
        ArmPath j = path;
        EXPECT_TRUE(manipulation::join_via_return(j, r, fold, unfold, true));   // P0 에서 이음 (받침까지 안 접음)
        return manipulation::path_start_ok(j, st.q, tol, w);
      }
      return false;
    };
  // 다음 작업 경로 (control_master 와 같은 시작): PICK = 받침 → P0 → 퍽 위 (모델), PLACE = 기준점 → 놓을 자리 위
  ArmPath pick = unfold;
  pick.nodes.push_back(node("above", {15, 10, -50, 40, 0}));
  const ArmPath place{"place", {pn("staging", staging), node("above", {10, 5, -40, 30, 0})}, true};
  const State s_p0{p0, false, false}, s_home{home, false, false}, s_look{look_q, false, false}, s_lb{lb, false, false}, s_la{la, false, false},
    s_pick{pick_end, true, false}, s_rack{rack_end, false, true}, s_staging{staging, false, false};
  struct Chain {const char * what; const State * st; ArmPath path; bool from_look; bool at_look;};
  const std::vector<Chain> chains{
    {"STORE·PLACE 끝 (P0) → PICK", &s_p0, pick, true, true},
    {"STORE·PLACE 끝 (P0) → LOOK", &s_p0, P["look"], true, false},
    {"STORE·PLACE 끝 (P0) → LOOK_BARCODE", &s_p0, P["look_barcode"], true, false},
    {"PLACE 끝 (P0) → RETRIEVE", &s_p0, P["retrieve_S1"], true, false},
    {"P0 → 적재함 스캔", &s_p0, P["rack_scan"], true, false},
    {"받침 → LOOK", &s_home, P["look"], true, false},
    {"LOOK 끝 → PICK", &s_look, pick, true, true},
    {"LOOK 끝 → RETRIEVE", &s_look, P["retrieve_S1"], true, false},
    {"LOOK 끝 → LOOK_BARCODE", &s_look, P["look_barcode"], true, false},
    {"LOOK 끝 → 적재함 스캔", &s_look, P["rack_scan"], true, false},
    {"LOOK_BARCODE 끝 → LOOK", &s_lb, P["look"], true, false},
    {"LOOK_BARCODE 끝 → PICK", &s_lb, pick, true, true},
    {"LOOK_AT 자세 → LOOK", &s_la, P["look"], true, false},
    {"LOOK_AT 자세 → PICK", &s_la, pick, true, true},
    {"LOOK_AT 자세 → RETRIEVE", &s_la, P["retrieve_S1"], true, false},
    {"PICK 끝 → STORE", &s_pick, P["store_S2_d30"], false, false},
    {"PICK 끝 → LOOK_RACK", &s_pick, P["look_rack_S2"], false, false},
    {"PICK 끝 → PLACE", &s_pick, place, false, false},
    {"LOOK_RACK 끝 → STORE", &s_rack, P["store_S2_d30"], false, false},
    {"LOOK_RACK 끝 → PUTBACK PLACE", &s_rack, place, false, false},
    {"LOOK_RACK 끝 → LOOK_RACK", &s_rack, P["look_rack_S2"], false, false},
    {"RETRIEVE 끝 (기준점) → PLACE", &s_staging, place, false, false},
    {"RETRIEVE 끝 (기준점) → LOOK_RACK", &s_staging, P["look_rack_S2"], false, false},
  };
  for (const auto & c : chains) {EXPECT_TRUE(starts(c.path, *c.st, c.from_look, c.at_look)) << c.what;}
  // LOOK_AT → HOME (검사된 복귀)
  ArmPath r;
  EXPECT_TRUE(look_at_return(la, r));
  EXPECT_FALSE(starts(P["look"], s_lb, false, false));
  EXPECT_FALSE(starts(P["look_barcode"], s_look, false, false));
}

// 받침에서 다시 쥔 퍽 (30 mm) 의 LOOK_RACK → STORE store_<칸>_d30 이 관찰 경로를 되짚어 lift_real → 칸 경로 전체를 감 (적재함을 두 번 오감).
//   새 칸 경로를 지금 자세에서 바로 이을 수 있으면 LOOK_RACK 을 건너뜀. 50 mm 박스 퍽 · 새 경로 없음 · 못 이음 · storeheld_over_look false 는 그대로
TEST(LookRackSkip, HeldPathsSkipBox50Keeps)
{
  using manipulation::look_rack_skip_for_store;
  using manipulation::storeheld_store_name;
  const std::set<std::string> have{"store_S1_d30", "store_S1_d30_low", "store_S1_d50", "store_S1_d40_tilt10", "store_S7_d30"};
  auto has = [&](const std::string & n) {return have.count(n) > 0;};
  // 받침에서 다시 쥔 퍽 30 mm (park_regrasp_depth_mm 30) → _d30, 지금 자세 (PICK 끝) 에서 이음 → 건너뜀 (실물 6 칸이 이 경우)
  const std::string d30 = storeheld_store_name("S1", 30.0, 30.0, 0.0, false, has);
  EXPECT_EQ(d30, "store_S1_d30");
  EXPECT_TRUE(look_rack_skip_for_store(d30, true, true));
  EXPECT_TRUE(look_rack_skip_for_store(storeheld_store_name("S1", 32.0, 30.0, 0.0, true, has), true, true));   // _d30_low 도
  // 기울여 쥔 퍽 (10° 까지는 LOOK_RACK 이 받음) → _d40_tilt10 도 관찰 결과 안 씀 → 건너뜀
  EXPECT_TRUE(look_rack_skip_for_store(storeheld_store_name("S1", 40.0, 30.0, 10.0, false, has), true, true));
  // 박스 퍽 50 mm: _d50 이 있어도 지금 흐름 그대로 (관찰 → store_look_<칸>)
  const std::string d50 = storeheld_store_name("S1", 50.0, 30.0, 0.0, false, has);
  EXPECT_EQ(d50, "store_S1_d50");
  EXPECT_FALSE(look_rack_skip_for_store(d50, true, true));
  // 새 경로 없음 (_d50 없는 칸 · storeheld_paths false = "") → 그대로
  EXPECT_FALSE(look_rack_skip_for_store(storeheld_store_name("S7", 50.0, 30.0, 0.0, false, has), true, true));
  EXPECT_FALSE(look_rack_skip_for_store("", true, true));
  // 지금 자세에서 못 이음 (받침 · 다른 작업 끝) → 관찰 자세를 거쳐 되짚어 잇는 지금 흐름 그대로
  EXPECT_FALSE(look_rack_skip_for_store(d30, true, false));
  // storeheld_over_look false: 관찰 자세면 store_look_<칸> 을 씀 (관찰 결과 씀) → 그대로
  EXPECT_FALSE(look_rack_skip_for_store(d30, false, true));
  // 건너뛴 뒤 STORE 고르기 = 집은 자리에서 새 칸 경로 (store_path_pick: 관찰 자세 아님 · 이음)
  EXPECT_EQ(manipulation::store_path_pick(true, true, false, true), manipulation::StorePick::kHeld);
}

// 받침에서 다시 쥔 퍽 STORE: 박스 앞 점 (lift_real …) 을 건너뛰고 표의 점으로 바로
TEST(ParkStoreJoin, SkipsBoxFrontNodes)
{
  using manipulation::park_store_join;
  ArmPath p{"store_S1_d30", {node("pick_end", {0, -20, -11, 31, -2}), node("lift_real", {7, 10, -40, 9, 0}), node("r2", {8, 8, -39, 9, 0}),
    node("r40", {30, -40, -33, 4, 0}), node("in_cell", {130, 4, -34, 21, 43}, false, "release"), node("out", {131, 10, -40, 25, 43}, false, "close")}, false};
  std::string to;
  std::size_t skipped = 0;
  ArmPath a = p;
  ASSERT_TRUE(park_store_join(a, "store_S1_d30", {"store_S2_d30=r9", "store_S1_d30=r40"}, to, skipped));
  EXPECT_EQ(to, "r40");
  EXPECT_EQ(skipped, 2u);
  ASSERT_EQ(a.nodes.size(), 4u);
  EXPECT_EQ(a.nodes[0].name, "pick_end");
  EXPECT_EQ(a.nodes[1].name, "r40");
  EXPECT_EQ(a.nodes[2].grip, "release");
  // 표에 없음 · 점 이름 없음 · 그리퍼 점 · 앞 점 (k < 2) → 그대로
  for (const std::vector<std::string> & t : std::vector<std::vector<std::string>>{{}, {"store_S1_d30=r99"}, {"store_S1_d30=in_cell"},
      {"store_S1_d30=lift_real"}, {"store_S1_d30_low=r40"}})
  {
    ArmPath b = p;
    EXPECT_FALSE(park_store_join(b, "store_S1_d30", t, to, skipped));
    EXPECT_EQ(b.nodes.size(), p.nodes.size());
  }
  // 건너뛸 점에 그리퍼 동작이 있으면 안 함
  ArmPath c = p;
  c.nodes[2].grip = "open";
  EXPECT_FALSE(park_store_join(c, "store_S1_d30", {"store_S1_d30=r40"}, to, skipped));
}

// 실제 젯슨 경로 store_<칸>_d30: 표의 점이 다 있고 (그리퍼 점 앞), 받침 위 (park_re_lift) 에서 시간이 줄어듦 (control.yaml 속도 · 비율)
TEST(ParkStoreJoin, RealD30PathsShorter)
{
  const std::string file = std::string(ARM_PATHS_YAML);
  const double t2r = 2.0 * M_PI / 4096.0;
  const std::vector<double> park{(4098 - 4088) * t2r, (1812 - 2048) * t2r, (1920 - 2048) * t2r, (2398 - 2048) * t2r, (2019 - 2048) * t2r};
  PathTiming tm;
  tm.speed_dps = 112.0;
  tm.stand_speed_dps = 62.5;
  tm.accel_dps2 = 280.0;
  const manipulation::GripperValues grip;
  const std::map<std::string, double> scale{{"S1", 0.7}, {"S2", 0.7}, {"S3", 0.9}, {"S8", 0.9}};
  double sum_old = 0.0, sum_new = 0.0;
  for (int k = 1; k <= 12; ++k) {
    const std::string s = "S" + std::to_string(k), name = "store_" + s + "_d30";
    ArmPath p;
    ASSERT_TRUE(load_path(name, p, file)) << name;
    p.nodes.front().q = park;
    p.nodes.front().name = "pick_end";
    for (auto & n : p.nodes) {   // control_master path_plan 과 같은 속도 (path_slow_min_dps 62.5 · path_speed_scale)
      if (n.speed_dps > 0.0 && n.speed_dps < 62.5) {n.speed_dps = 62.5;}
      const auto it = scale.find(s);
      if (it != scale.end()) {const double v0 = n.stand ? tm.stand_speed_dps : tm.speed_dps; n.speed_dps = it->second * (n.speed_dps > 0.0 ? std::min(v0, n.speed_dps) : v0);}
    }
    ArmPath j = p;
    std::string to;
    std::size_t skipped = 0;
    ASSERT_TRUE(manipulation::park_store_join(j, name, manipulation::kParkStoreJoinDefault, to, skipped)) << name;
    auto until_release = [&](const ArmPath & a) {
        double t = 0.0;
        for (const auto & act : manipulation::path_actions(a, tm, grip, park)) {
          if (act.is_gripper) {break;}   // 첫 그리퍼 (놓기) 앞까지
          t += act.duration_s;
        }
        return t;
      };
    const double t_old = until_release(p), t_new = until_release(j);
    std::printf("%s: 받침 → 놓기 %.2f s → %.2f s (%s, 점 %zu 개 건너뜀, %.2f s 줄어듦)\n", name.c_str(), t_old, t_new, to.c_str(), skipped, t_old - t_new);
    EXPECT_LT(t_new, t_old) << name;
    sum_old += t_old;
    sum_new += t_new;
  }
  std::printf("12 칸 평균: %.2f s → %.2f s (%.2f s 줄어듦)\n", sum_old / 12.0, sum_new / 12.0, (sum_old - sum_new) / 12.0);
}
