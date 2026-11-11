// test_arm_path.cpp — 실물 경로 → 시간표 · 동작 목록 시험 (arm_path.hpp)
// colcon test --packages-select manipulation

#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "manipulation/arm_path.hpp"
#include "manipulation/cell_shift.hpp"
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
  ns[1].dt_s = 0.0;   // 하나라도 없으면 예전 시간표
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

// PICK 퍽 위 멈춤 (pick_settle_s → above 에 stop + hold_s): 0 이면 예전 그대로, > 0 이면 above 에서 끊고 그 자세로 기다림,
//   미리 열기 (grip_early) 는 그대로 출발 때 · 그리퍼 동작은 안 생김 · 쥐기 (close) 순서 그대로
TEST(PathActions, SettleAtAboveKeepsEarlyOpen)
{
  ArmPath p;
  p.nodes = {node("s", {0, 0, 0, 0, 0}), node("above", {20, 0, 0, 0, 0}, false, "open"), node("down", {25, 0, 0, 0, 0}),
    node("grasp", {30, 0, 0, 0, 0}, false, "close"), node("lift", {20, 0, 0, 0, 0})};
  p.nodes[1].grip_early = true;
  const auto plain = manipulation::path_actions(p, PathTiming{}, manipulation::GripperValues{});
  ASSERT_EQ(plain.size(), 3u);   // 0: 예전과 같음
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

// 실제 config/arm_paths.yaml: 받침에서 시작하는 빼기 · LOOK 경로는 앞이 unfold 와 같아야 P0 에서 바로 시작 (lazy_fold).
//   빼기 경로에서 upJ3a 를 지우자 P0 잇기가 조용히 안 됨 (Fable 검수) → 경로를 고칠 때 깨지지 않게
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

bool load_path(const std::string & want, ArmPath & p)
{
  std::ifstream f(ARM_PATHS_YAML);
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

// 11-10 22:0x 실물 (누운 퍽 옮겨 놓은 뒤 받침 세우기 PICK, 두 번째 쥐기에서 빈손): 옮겨 놓기 · 첫 쥐기 고리를 빼고 높은 점만 거꾸로 → P0
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
