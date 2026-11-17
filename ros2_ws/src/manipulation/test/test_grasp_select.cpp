// test_grasp_select.cpp — 이웃 퍽을 보고 닫힘 방향 · 벌림 폭 고르기 (grasp_select.hpp)

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>
#include <vector>

#include "manipulation/arm_kinematics.hpp"
#include "manipulation/grasp_select.hpp"
#include "arm_kin_data.hpp"

using manipulation::NeighborPuck;
using manipulation::TargetPuck;

namespace
{
constexpr double kOpen = 0.0164, kRelease = 0.0093;
NeighborPuck cyl(double x, double y) {NeighborPuck n; n.c = {x, y}; return n;}
const TargetPuck kCyl{false, false, 0.0};
// 닫힘축 두 방향이 같은가 (180° 주기)
double axis_diff_deg(double a, double b) {return std::fabs(std::remainder(a - b, M_PI)) * 180.0 / M_PI;}
manipulation::GraspChoice pick(const TargetPuck & t, const std::vector<double> & closes, const std::vector<NeighborPuck> & nb, double def)
{
  return manipulation::choose_grasp({0.25, 0.0}, t, closes, {kOpen, kRelease}, nb, def);
}
}  // namespace

// 1) 이웃 없으면 기본 방향 · 다 열기
TEST(GraspSelect, NoNeighbors)
{
  const auto ch = pick(kCyl, manipulation::close_candidates(false, 0.0, 0.3), {}, 0.3);
  ASSERT_TRUE(ch.ok);
  EXPECT_NEAR(ch.close_rad, 0.3, 1e-9);
  EXPECT_DOUBLE_EQ(ch.grip, kOpen);
}

// 2) 손가락 치수: 다 열면 바깥 면 43.7 mm, 넣기 폭이면 36.6 mm, 폭 ±16.4 mm
TEST(GraspSelect, FingerReach)
{
  const Eigen::Vector2d c(0, 0);
  const std::vector<NeighborPuck> nb{cyl(0.055, 0.0)};   // 닫힘축 x 쪽, 퍽 사이 간격 25 mm
  EXPECT_NEAR(manipulation::finger_clearance(c, 0.0, kOpen, nb), 0.055 - 0.015 - 0.002 - (0.0273 + kOpen), 1e-9);
  EXPECT_GT(manipulation::finger_clearance(c, 0.0, kRelease, nb), 0.0);
  EXPECT_LT(manipulation::finger_clearance(c, 0.0, kOpen, nb), 0.0);
  // 폭 방향 (닫힘축 y): 손가락 사각 모서리 (x 16.4 mm, y 안쪽 면 9.1 + 16.4 mm) 에서 이웃 원까지
  EXPECT_NEAR(manipulation::finger_clearance(c, M_PI / 2, kOpen, nb), std::hypot(0.055 - 0.0164, 0.0091 + kOpen) - 0.017, 1e-9);
}

// 3) 한쪽 (x) 에 2 cm 간격 이웃 → 닫힘축을 그 수직 (y) 으로, 다 열기
TEST(GraspSelect, TurnsAwayFromCloseNeighbor)
{
  const auto ch = pick(kCyl, manipulation::close_candidates(false, 0.0, 0.0), {cyl(0.30, 0.0)}, 0.0);
  ASSERT_TRUE(ch.ok);
  EXPECT_NEAR(axis_diff_deg(ch.close_rad, M_PI / 2), 0.0, 1e-6);
  EXPECT_DOUBLE_EQ(ch.grip, kOpen);
}

// 4) 네모 퍽 (면 방향 앎): 면 방향 두 개 중 이웃을 피하는 쪽
TEST(GraspSelect, SquareUsesFaceDirections)
{
  const double yaw = 10.0 * M_PI / 180.0;
  const auto closes = manipulation::close_candidates(true, yaw, yaw);
  ASSERT_EQ(closes.size(), 2u);
  const auto ch = pick({true, true, yaw}, closes, {cyl(0.25 + 0.050 * std::cos(yaw), 0.050 * std::sin(yaw))}, yaw);
  ASSERT_TRUE(ch.ok);
  EXPECT_NEAR(axis_diff_deg(ch.close_rad, yaw + M_PI / 2), 0.0, 1e-6);
}

// 5) 사방 2 cm 간격 → 어느 방향 · 폭도 안 됨 → 거절 (이유 있음)
TEST(GraspSelect, BoxedInIsRejected)
{
  std::vector<NeighborPuck> nb;
  for (int k = 0; k < 4; ++k) {nb.push_back(cyl(0.25 + 0.05 * std::cos(k * M_PI / 2), 0.05 * std::sin(k * M_PI / 2)));}
  const auto ch = pick(kCyl, manipulation::close_candidates(false, 0.0, 0.0), nb, 0.0);
  EXPECT_FALSE(ch.ok);
  EXPECT_FALSE(ch.why.empty());
}

// 6) 닫힘축 x 양쪽 2.6 cm · y 양쪽 2 cm: 다 열기는 안 되고 넣기 폭(원기둥 목표와 3.4 mm)은 됨
TEST(GraspSelect, NarrowerOpeningWhenNeeded)
{
  const std::vector<NeighborPuck> nb{cyl(0.306, 0.0), cyl(0.194, 0.0), cyl(0.25, 0.050), cyl(0.25, -0.050)};
  const auto ch = pick(kCyl, manipulation::close_candidates(false, 0.0, 0.0), nb, 0.0);
  ASSERT_TRUE(ch.ok);
  EXPECT_DOUBLE_EQ(ch.grip, kRelease);
  EXPECT_NEAR(axis_diff_deg(ch.close_rad, 0.0), 0.0, 1e-6);
}

// 7) 면 방향 모르는 네모 목표 (외접 24.6 mm) 는 넣기 폭(안쪽 면 18.4 mm)으로 못 내려감 → 6 과 같은 배치면 거절
TEST(GraspSelect, UnknownSquareTargetNeedsWideOpening)
{
  const std::vector<NeighborPuck> nb{cyl(0.306, 0.0), cyl(0.194, 0.0), cyl(0.25, 0.050), cyl(0.25, -0.050)};
  EXPECT_FALSE(pick({true, false, 0.0}, manipulation::close_candidates(false, 0.0, 0.0), nb, 0.0).ok);
  EXPECT_NEAR(manipulation::target_half_along({true, false, 0.0}, 0.3), 0.0174 * std::sqrt(2.0), 1e-12);
  EXPECT_NEAR(manipulation::target_half_along({true, true, 0.2}, 0.2 + M_PI / 2), 0.0174, 1e-12);
}

// 8) 기본 방향 여유가 충분하면 (≥ 5 mm) 더 넓은 방향이 있어도 손목을 안 돌림
TEST(GraspSelect, KeepsDefaultWhenRoomy)
{
  // 닫힘축 x 기준, 이웃이 x 쪽 중심 75 mm (다 열기 여유 ≈ 14 mm)
  const auto ch = pick(kCyl, manipulation::close_candidates(false, 0.0, 0.0), {cyl(0.325, 0.0)}, 0.0);
  ASSERT_TRUE(ch.ok);
  EXPECT_NEAR(axis_diff_deg(ch.close_rad, 0.0), 0.0, 1e-6);
}

// 9) 면 방향 아는 네모 이웃이 손가락과 모서리끼리 가로지르는 배치도 겹침으로 (분리축)
TEST(GraspSelect, CrossingSquaresOverlap)
{
  NeighborPuck n;
  n.c = {0.25 + 0.0091 + kOpen + 0.009, 0.0};   // 손가락 중심에 퍽 중심, 45° 돌림
  n.square = true;
  n.yaw_known = true;
  n.yaw = M_PI / 4;
  EXPECT_LE(manipulation::finger_clearance({0.25, 0.0}, 0.0, kOpen, {n}), 0.0);
}

// 10) 목표 검출 위치 차이만큼 여유를 뺌 · PLACE (gate 끔) 은 목표 검사 안 함
TEST(GraspSelect, TargetOffsetAndPlaceGate)
{
  const std::vector<NeighborPuck> nb{cyl(0.306, 0.0), cyl(0.194, 0.0), cyl(0.25, 0.050), cyl(0.25, -0.050)};
  const auto closes = manipulation::close_candidates(false, 0.0, 0.0);
  TargetPuck t{false, false, 0.0, 0.0, true};
  EXPECT_TRUE(pick(t, closes, nb, 0.0).ok);          // 원기둥 목표: 넣기 폭 여유 3.4 mm
  t.offset = 0.001;
  EXPECT_FALSE(pick(t, closes, nb, 0.0).ok);         // 1 mm 어긋남 → 2.4 mm < 3 mm
  const TargetPuck unknown_place{true, false, 0.0, 0.0, false};
  EXPECT_TRUE(pick(unknown_place, closes, nb, 0.0).ok);   // PLACE: 퍽이 손 안 → 목표 검사 없이 넣기 폭
}

// 11) 누운 퍽 발자국 (긴축 ±30 · 옆 ±17.4 mm): 긴축 끝 쪽 이웃은 손가락 (폭 ±16.4 mm) 밖이라 손가락 여유로는 안 보임
TEST(GraspSelect, LyingFootprintClearance)
{
  const Eigen::Vector2d c(0.25, 0.0), half(0.030, 0.0174);
  const std::vector<NeighborPuck> end_nb{cyl(0.25 + 0.040, 0.0)};   // 긴축 (x) 끝에서 원기둥 중심까지 10 mm → 겹침
  EXPECT_LT(manipulation::footprint_clearance(c, 0.0, half, end_nb), 0.0);
  EXPECT_GT(manipulation::finger_clearance(c, M_PI / 2.0, kOpen, end_nb), 0.002);   // 닫힘축 = y: 손가락은 안 닿음
  const std::vector<NeighborPuck> far_nb{cyl(0.25 + 0.052, 0.0)};   // 여유 = 52 − 긴축 반 30 − 반지름 15 − slop 2 = 5 mm
  EXPECT_NEAR(manipulation::footprint_clearance(c, 0.0, half, far_nb), 0.005, 1e-9);
  EXPECT_GT(manipulation::footprint_clearance(c, 0.0, half, {}), 1.0);   // 이웃 없음
}

// 12) 면 정렬 · 붙은 (간격 0) 네모 둘: 연결선 (x) 방향 닫힘은 겹침, 수직 (y) 닫힘은 여유 25.5 − 19.4 = 6.1 mm → 집힘
TEST(GraspSelect, TouchingSquaresGraspPerpendicular)
{
  NeighborPuck n;
  n.c = {0.25 + 0.0348, 0.0};   // 한 변 34.8 mm 만큼 떨어짐 = 면이 붙음
  n.square = true;
  n.yaw_known = true;
  n.yaw = 0.0;
  EXPECT_LE(manipulation::finger_clearance({0.25, 0.0}, 0.0, kOpen, {n}), 0.0);
  EXPECT_NEAR(manipulation::finger_clearance({0.25, 0.0}, M_PI / 2, kOpen, {n}), 0.0255 - 0.0194, 1e-9);
  const auto ch = pick({true, true, 0.0}, manipulation::close_candidates(true, 0.0, 0.0), {n}, 0.0);
  ASSERT_TRUE(ch.ok);
  EXPECT_NEAR(axis_diff_deg(ch.close_rad, M_PI / 2), 0.0, 1e-6);
  EXPECT_DOUBLE_EQ(ch.grip, kOpen);
}

// ---- 다시 쥘 자리 (regrasp_spot_gap_mm · regrasp_edge_margin_mm) ----
namespace
{
using manipulation::RegraspSpot;
constexpr double kMinX = 0.212, kMaxY = 0.075, kGap = 0.030, kMargin = 0.015, kSq = 0.0174;
NeighborPuck sq(double x, double y, double yaw = 0.0)
{
  NeighborPuck n;
  n.c = {x, y};
  n.square = true;
  n.yaw_known = true;
  n.yaw = yaw;
  return n;
}
// 사례 2139 (실제 로봇): 목표 검출 (0.213, -0.003) · 같은 색 네모 이웃 -86° 쪽 면 사이 -5.4 mm (중심 29.4 mm) → 이웃 검출 중심
NeighborPuck nb_2139() {return sq(0.213 + 0.0294 * std::cos(-86.0 * M_PI / 180.0), -0.003 + 0.0294 * std::sin(-86.0 * M_PI / 180.0), -8.8 * M_PI / 180.0);}

struct SpotRun
{
  bool ok{false};
  RegraspSpot spot;
  int tier{0};
  size_t n{0}, index{0};
  int looked{0}, skipped{0};
  double ms{0.0}, best_gap{-1.0};
};
// control_master regrasp_nodes 와 같은 순서 · 같은 검사로 자리를 고름 (실물 팔 모델 IK): 빈 자리 (25 mm 안 검출 없음) · 손가락 여유
//   (grasp_for PLACE) → 수직 다시 쥐기 (regrasp_offset_mm −7) → 내려놓기 (put_tilt ± put_dev). 씨앗 재사용 · 기본 씨앗 한 번 더 · 먼 자리 넘김 그대로
SpotRun run_spots(const Eigen::Vector2d & p0, const std::vector<NeighborPuck> & seen, bool origin_first, double put_tilt = 0.0,
  double put_dev = 0.5, double depth = 50.0)
{
  const auto m = kin_data::model();
  const double top = 0.068;   // 박스 위 선 퍽 윗면 (모델) + pick_top_bias_mm 10 — test_visual_regrasp RealSpot 과 같음
  const auto t0 = std::chrono::steady_clock::now();
  const auto nb = manipulation::unbias_neighbors(p0, kSq, seen);
  std::vector<Eigen::Vector2d> known{p0};
  for (const auto & n : nb) {known.push_back(n.c);}
  std::vector<RegraspSpot> spots;
  for (const auto & q : manipulation::regrasp_spot_points(m.j1_xy, p0, kMinX, kMaxY, origin_first)) {
    spots.push_back({q, manipulation::spot_gap(q, kSq, nb), manipulation::spot_edge_short(q, kMinX, kMaxY, kMargin, known),
      static_cast<int>(spots.size()), (q - m.j1_xy).norm()});
  }
  spots = manipulation::order_regrasp_spots(spots, kGap, origin_first ? &p0 : nullptr);
  SpotRun out;
  out.n = spots.size();
  std::vector<manipulation::Q5> seed_r, seed_p;
  double far_re = 1e9, far_put = 1e9;
  const bool put_vertical = put_tilt + put_dev < 10.0 - 1e-6;
  double reach40 = -1.0;   // 수직 손 위 점 (윗면 + 40) 이 닿는 가장 먼 J1 거리 — 처음 안 닿을 때 한 번 (control_master reach_above 와 같음)
  auto reach_above = [&]() {
      if (reach40 > 0.0) {return reach40;}
      std::string w;
      auto at = [&](double d, const std::vector<manipulation::Q5> & seeds, manipulation::Q5 & q) {
          const auto P = manipulation::pick_poses(m, m.j1_xy.x() + d, m.j1_xy.y(), top, 90.0, {40.0}, w, 90.0, 0.0, 0.0, 0.5, seeds);
          if (P.size() == 1) {q = manipulation::ticks_to_q(m, P[0].ticks);}
          return P.size() == 1;
        };
      double lo = 0.17, hi = 0.32;
      manipulation::Q5 q{};
      reach40 = 1e9;
      if (at(lo, {}, q)) {
        while (hi - lo > 0.001) {
          const double mid = 0.5 * (lo + hi);
          manipulation::Q5 q2{};
          if (at(mid, {q}, q2) || at(mid, {}, q2)) {lo = mid; q = q2;} else {hi = mid;}   // 씨앗으로 안 되면 기본 씨앗
        }
        reach40 = lo;
      }
      return reach40;
    };
  for (size_t k = 0; k < spots.size(); ++k) {
    const auto & s = spots[k];
    const Eigen::Vector2d r(s.p.x() - 0.007, s.p.y());
    const double d_re = (r - m.j1_xy).norm(), d_put = (s.p - m.j1_xy).norm();
    if (d_re > far_re + 0.002 || (put_vertical && d_put > far_put + 0.002)) {++out.skipped; continue;}
    ++out.looked;
    bool taken = false;   // grasp_for (PLACE): 25 mm 안 퍽 · 손가락 들어갈 방향 (원본 seen, 이웃 반경 120 mm)
    std::vector<NeighborPuck> near;
    for (const auto & n : seen) {
      if ((n.c - s.p).norm() < 0.025) {taken = true;}
      if ((n.c - s.p).norm() <= 0.12) {near.push_back(n);}
    }
    if (taken) {continue;}
    std::string why;
    const double def = manipulation::default_close_rad(m, s.p.x(), s.p.y(), std::nan(""));
    if (!near.empty() && manipulation::rank_grasps(s.p, {true, false, 0.0, 0.0, false}, manipulation::close_candidates(false, 0.0, def),
      {kOpen, kRelease}, near, def, why).empty()) {continue;}
    const double c = (std::atan2(s.p.y() - m.j1_xy.y(), s.p.x() - m.j1_xy.x()) + M_PI / 2.0) * 180.0 / M_PI;
    auto Pr = manipulation::pick_poses(m, r.x(), r.y(), top, c, {40.0, 10.0, -50.0}, why, c, 40.0, 0.0, 0.5, seed_r);
    bool re_far = false;
    if (Pr.size() != 3 && !seed_r.empty()) {
      Pr = manipulation::pick_poses(m, r.x(), r.y(), top, c, {40.0, 10.0, -50.0}, why, c, 40.0, 0.0, 0.5);
      re_far = Pr.size() != 3;
    } else if (Pr.size() != 3) {
      re_far = true;
    }
    if (re_far) {const double R = reach_above(); far_re = std::min(far_re, R < 1.0 ? R : d_re);}
    if (Pr.size() != 3) {continue;}
    seed_r = {manipulation::ticks_to_q(m, Pr[0].ticks)};
    const std::vector<double> hp{40.0, 10.0, 8.0 - depth};
    auto Pp = manipulation::pick_poses(m, s.p.x(), s.p.y(), top, c, hp, why, c, 40.0, put_tilt, put_dev, seed_p);
    if (Pp.size() != 3 && put_vertical) {const double R = reach_above(); far_put = std::min(far_put, R < 1.0 ? R : d_put);}
    if (Pp.size() != 3) {continue;}
    out.ok = true;
    out.spot = s;
    out.tier = manipulation::spot_tier(s, kGap);
    out.index = k;
    break;
  }
  out.ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  for (const auto & s : spots) {out.best_gap = std::max(out.best_gap, s.gap);}
  return out;
}
std::string say(const SpotRun & r)
{
  char b[200];
  std::snprintf(b, sizeof(b), "자리 (%.4f, %.4f) 사이 %.1f mm 가장자리 %.1f mm 층 %d · %zu 곳 중 %zu 째 · 검사 %d · 넘김 %d · %.0f ms", r.spot.p.x(), r.spot.p.y(),
    r.spot.gap * 1000.0, r.spot.edge_short * 1000.0, r.tier, r.n, r.index + 1, r.looked, r.skipped, r.ms);
  return b;
}
}  // namespace

// 놓을 퍽 중심에서 이웃 몸통까지 − 반폭: 면을 마주한 네모 (면 사이 그대로) · 모서리 쪽 · 원기둥 · 면 모름 (외접원) · 누운 (30 mm 원) · 겹침 음수
TEST(RegraspSpot, GapToNeighborBody)
{
  const Eigen::Vector2d s(0.22, 0.0);
  EXPECT_NEAR(manipulation::spot_gap(s, kSq, {sq(0.22, 0.0348 + 0.030)}), 0.030, 1e-12);   // 면 사이 30 mm
  EXPECT_NEAR(manipulation::spot_gap(s, kSq, {sq(0.22 + 0.050, 0.050)}),
    std::hypot(0.050 - kSq, 0.050 - kSq) - kSq, 1e-12);                                         // 대각선: 이웃 모서리까지
  EXPECT_NEAR(manipulation::spot_gap(s, kSq, {cyl(0.22, 0.070)}), 0.070 - 0.015 - kSq, 1e-12);
  NeighborPuck unk = sq(0.22, 0.070);
  unk.yaw_known = false;
  EXPECT_NEAR(manipulation::spot_gap(s, kSq, {unk}), 0.070 - kSq * std::sqrt(2.0) - kSq, 1e-12);
  NeighborPuck lying = cyl(0.22, 0.070);
  lying.radius = 0.03;
  EXPECT_NEAR(manipulation::spot_gap(s, kSq, {lying}), 0.070 - 0.030 - kSq, 1e-12);
  EXPECT_LT(manipulation::spot_gap(s, kSq, {sq(0.22, 0.030)}), 0.0);   // 겹침
  EXPECT_GT(manipulation::spot_gap(s, kSq, {}), 1.0);                   // 이웃 없음
  // 사이 30 mm 이웃: 다 연 손가락 (바깥 면 43.7 mm, 모서리 46.7 mm) 이 어느 방향으로 내려와도 안 닿음 (위치 오차 slop 없이),
  //   닫힘축이 이웃 쪽 (앞줄 옆 이웃 = 접선 닫힘의 흔한 경우) 이면 slop 2 mm 를 빼도 1.7 mm. 넣기 폭으로 내려놓는 손가락은 7 mm 넘게 남음
  for (double a = 0.0; a < M_PI; a += M_PI / 36.0) {
    for (double b = 0.0; b < 2.0 * M_PI; b += M_PI / 18.0) {
      const NeighborPuck n = cyl(0.22 + (0.030 + kSq + 0.015) * std::cos(b), (0.030 + kSq + 0.015) * std::sin(b));
      ASSERT_NEAR(manipulation::spot_gap(s, kSq, {n}), 0.030, 1e-9);
      EXPECT_GT(manipulation::finger_clearance(s, a, kOpen, {n}, manipulation::FingerTip{}, 0.015, 0.0), 0.0) << a << " " << b;
      EXPECT_GT(manipulation::finger_clearance(s, a, kRelease, {n}, manipulation::FingerTip{}, 0.015, 0.0), 0.007) << a << " " << b;
    }
    const NeighborPuck on_axis = cyl(0.22 + (0.030 + kSq + 0.015) * std::cos(a), (0.030 + kSq + 0.015) * std::sin(a));
    EXPECT_NEAR(manipulation::finger_clearance(s, a, kOpen, {on_axis}), 0.030 + kSq - 0.0437 - 0.002, 1e-9) << a;
  }
}

// 같은 색 겹침 (면 사이 < 0) 이웃은 목표 반대쪽으로 겹친 만큼 (5 mm 까지) 밂, 떨어진 이웃은 그대로
TEST(RegraspSpot, UnbiasPushesOverlappingNeighborAway)
{
  const Eigen::Vector2d p0(0.212, -0.001);
  const auto nb = manipulation::unbias_neighbors(p0, kSq, {nb_2139(), sq(0.30, 0.05), sq(0.212, -0.001 - 0.020)});
  ASSERT_EQ(nb.size(), 3u);
  EXPECT_NEAR((nb[0].c - p0).norm(), 2.0 * kSq, 1e-9);   // 사례 2139 이웃: 3.4 mm 밀어 면이 붙음
  EXPECT_GT((nb[0].c - nb_2139().c).norm(), 0.003);
  EXPECT_EQ(nb[1].c, sq(0.30, 0.05).c);
  EXPECT_NEAR((nb[2].c - p0).norm(), 0.025, 1e-9);        // 크게 겹침 → 5 mm 까지만
}

// 고정 격자 후보 (옆 0 · ±4 · ±8 cm, 퍽 1 cm 앞까지, 앞쪽 끝 · origin_first) 를 그 점 그대로 다 품음 → 고정 격자보다 자리를 못 찾는 일 없음
TEST(RegraspSpot, CandidatesContainOldOnes)
{
  const Eigen::Vector2d j1(0.0225499941511, 0.0);
  for (const auto & p0 : {Eigen::Vector2d(0.212, -0.001), Eigen::Vector2d(0.2095, -0.039), Eigen::Vector2d(0.29, 0.06),
      Eigen::Vector2d(0.25, -0.074), Eigen::Vector2d(0.22, 0.0), Eigen::Vector2d(0.34, -0.094)})
  {
    for (bool of : {false, true}) {
      const auto now = manipulation::regrasp_spot_points(j1, p0, kMinX, kMaxY, of);
      std::vector<Eigen::Vector2d> old;
      const double r0 = (p0 - j1).norm();
      const Eigen::Vector2d dir = (p0 - j1) / r0, side(-dir.y(), dir.x());
      if (of) {old.push_back(p0);}
      const double r_near = dir.x() > 0.1 ? (kMinX + 0.002 - j1.x()) / dir.x() : r0 - 0.01;
      for (double r = std::min(r_near, r0 - 0.01); r <= r0 - 0.01 + 1e-9; r += 0.01) {
        for (double lat : {0.0, 0.04, -0.04, 0.08, -0.08}) {old.push_back(j1 + dir * r + side * lat);}
      }
      for (double dx = 0.002; dx <= 0.0221; dx += 0.01) {
        const double x = kMinX + dx, y_line = std::fabs(dir.x()) > 0.1 ? j1.y() + dir.y() * (x - j1.x()) / dir.x() : p0.y();
        for (double y = y_line; std::fabs(y) > 1e-9 && y * y_line > 0.0; y -= std::copysign(0.01, y_line)) {old.push_back({x, y});}
      }
      int n_old = 0;
      for (const auto & q : old) {
        if (q.x() < kMinX || std::fabs(q.y()) > kMaxY) {continue;}
        ++n_old;
        double best = 1e9;
        for (const auto & c : now) {best = std::min(best, (c - q).norm());}
        EXPECT_LT(best, 1e-9) << p0.transpose() << " " << of << " 예전 후보 " << q.transpose();   // 고정 격자 후보는 그 점 그대로 남음
      }
      EXPECT_GE(now.size(), static_cast<size_t>(n_old));
      if (of && p0.x() >= kMinX && std::fabs(p0.y()) <= kMaxY) {ASSERT_FALSE(now.empty()); EXPECT_EQ(now.front(), p0);}   // 박스 밖 원래 자리는 고정 격자에서도 뺌
    }
  }
}

// 순서: 사이 ≥ 30 · 안쪽 → pref, 사이 ≥ 30 · 가장자리 → 모자람 작은 순, 나머지 → (사이 − 모자람) 큰 순
TEST(RegraspSpot, OrderTiers)
{
  std::vector<RegraspSpot> s{{{0.21, 0.0}, 0.010, 0.0, 0}, {{0.22, 0.0}, 0.040, 0.010, 1}, {{0.23, 0.0}, 0.035, 0.0, 2},
    {{0.24, 0.0}, 0.050, 0.004, 3}, {{0.25, 0.0}, 0.025, 0.012, 4}, {{0.26, 0.0}, 0.020, 0.0, 5}, {{0.27, 0.0}, 1e9, 0.0, 6}};
  const auto o = manipulation::order_regrasp_spots(s, kGap);
  std::vector<int> pref;
  for (const auto & x : o) {pref.push_back(x.pref);}
  EXPECT_EQ(pref, (std::vector<int>{2, 6, 3, 1, 5, 4, 0}));   // 3 층: 20 · (25 − 12 = 13) · 10 mm
  EXPECT_EQ(manipulation::spot_tier(o[0], kGap), 1);
  EXPECT_EQ(manipulation::spot_tier(o[2], kGap), 2);
  EXPECT_EQ(manipulation::spot_tier(o[4], kGap), 3);
}

// 박스가 돌아 놓였을 때 (예: 오른쪽이 3 cm 앞): 앞 · 옆 끝 15 mm 안은 가장자리 (순서만 뒤로). 퍽이 있던 자리보다 안쪽이면 안쪽
TEST(RegraspSpot, EdgeMarginAndKnownPucks)
{
  EXPECT_NEAR(manipulation::spot_edge_short({0.2125, -0.07}, kMinX, kMaxY, kMargin, {}), 0.0145 + 0.010, 1e-9);   // 앞오른쪽 구석
  EXPECT_DOUBLE_EQ(manipulation::spot_edge_short({0.230, 0.0}, kMinX, kMaxY, kMargin, {}), 0.0);
  EXPECT_DOUBLE_EQ(manipulation::spot_edge_short({0.2125, -0.07}, kMinX, kMaxY, 0.0, {}), 0.0);   // 0 = 안 봄
  // 퍽이 (0.212, -0.072) 에 있었음 → 그 자리 · 그보다 안쪽은 박스 안이 확실
  EXPECT_DOUBLE_EQ(manipulation::spot_edge_short({0.2125, -0.07}, kMinX, kMaxY, kMargin, {{0.212, -0.072}}), 0.0);
  EXPECT_NEAR(manipulation::spot_edge_short({0.2125, 0.07}, kMinX, kMaxY, kMargin, {{0.212, -0.072}}), 0.0145 + 0.010, 1e-9);   // 반대쪽은 모름
  EXPECT_NEAR(manipulation::spot_edge_short({0.2125, -0.03}, kMinX, kMaxY, kMargin, {{0.212, -0.072}}), 0.0145, 1e-9);           // y 차 band 밖
}

// 사례 2139 (실제 로봇): 붙은 네모 (이웃 -86°, 겹쳐 잰 5.4 mm) 를 먼 퍽 표로 쥔 뒤 (기울임 0) 원래 자리 (0.214, -0.001) 에 내려놓으면 사이 < 0 → 이웃을 누름.
//   사이 30 mm 넘는 자리 (이웃 반대쪽) 를 고르고 팔 모델로 수직 다시 쥐기 · 내려놓기가 풀림
TEST(RegraspSpot, Case2139TouchingPairAtFrontEdge)
{
  const Eigen::Vector2d p0(0.212, -0.001);
  const auto nb = manipulation::unbias_neighbors(p0, kSq, {nb_2139()});
  EXPECT_LT(manipulation::spot_gap({0.214, -0.001}, kSq, nb), 0.0);   // 원래 자리
  const auto r = run_spots(p0, {nb_2139()}, false);
  ASSERT_TRUE(r.ok) << say(r);
  EXPECT_GE(r.spot.gap, kGap) << say(r);
  EXPECT_LE(r.tier, 2) << say(r);
  EXPECT_GT(r.spot.p.y(), 0.02) << say(r);   // 이웃 (-y) 반대쪽
  EXPECT_LT(r.ms, 800.0) << say(r);
  std::printf("[ 21:39 ] %s\n", say(r).c_str());
}

// 사례 2131 (실제 로봇): 목표 (0.210, -0.036) · 같은 색 이웃 +y 쪽 면 사이 -5.9 mm, 먼 퍽 표 기울임 6° (내려놓는 손 10°, farpick_regrasp_tilt_dev 5).
//   고정 격자는 자리를 못 찾음 (놓을 자리에 이미 퍽이 보임) → 먼 퍽 표를 못 씀. 사이 30 mm 넘는 자리를 찾아야 함
TEST(RegraspSpot, Case2131NeighborOnInnerSide)
{
  const Eigen::Vector2d det(0.210, -0.036), p0(0.2095, -0.039);   // PICK 이 이웃 반대쪽으로 3 mm 민 점
  const NeighborPuck n = sq(0.210, -0.036 + 0.0289);
  const auto r = run_spots(p0, {n}, false, 6.0, 5.0);
  ASSERT_TRUE(r.ok) << say(r);
  EXPECT_GE(r.spot.gap, kGap) << say(r);
  EXPECT_LT(r.ms, 800.0) << say(r);
  std::printf("[ 21:31 ] %s\n", say(r).c_str());
  (void)det;
}

// store_recover (origin_first): 원래 자리에 붙은 이웃이 없으면 원래 자리, 붙어 있으면 (면 사이 0) 원래 자리는 뒤로 · 사이 30 mm 자리
TEST(RegraspSpot, StoreRecoverOriginFirstWithTouchingNeighbor)
{
  const Eigen::Vector2d p0(0.218, 0.010);
  const auto alone = run_spots(p0, {sq(0.30, -0.06)}, true);
  ASSERT_TRUE(alone.ok) << say(alone);
  EXPECT_EQ(alone.spot.p, p0) << say(alone);
  EXPECT_EQ(alone.index, 0u);
  const auto touch = run_spots(p0, {sq(0.218, 0.010 - 0.0348)}, true);
  ASSERT_TRUE(touch.ok) << say(touch);
  EXPECT_NE(touch.spot.p, p0) << say(touch);
  EXPECT_GE(touch.spot.gap, kGap) << say(touch);
  EXPECT_LT(touch.ms, 800.0) << say(touch);
  std::printf("[ store_recover ] %s\n", say(touch).c_str());
}

// store_recover · 쥔 퍽 영상 다시 쥐기 (origin_first) 의 원래 자리는 이웃과 사이 ≥ 30 (층 1 · 2) 이면 맨 앞 그대로.
//   J1 거리 칸 순서로 박스 가운데 퍽의 원래 자리가 6–19 째로 밀리면, LOOK 이 오래돼 이웃을 못 본 박스에서 못 본 퍽 위에 놓을 수 있음
TEST(RegraspSpot, OriginFirstKeepsPickedSpotFirst)
{
  const auto m = kin_data::model();
  auto spots_for = [&](const Eigen::Vector2d & p0, const std::vector<NeighborPuck> & seen) {
      const auto nb = manipulation::unbias_neighbors(p0, kSq, seen);
      std::vector<Eigen::Vector2d> known{p0};
      for (const auto & n : nb) {known.push_back(n.c);}
      std::vector<RegraspSpot> spots;
      for (const auto & q : manipulation::regrasp_spot_points(m.j1_xy, p0, kMinX, kMaxY, true)) {
        spots.push_back({q, manipulation::spot_gap(q, kSq, nb), manipulation::spot_edge_short(q, kMinX, kMaxY, kMargin, known),
          static_cast<int>(spots.size()), (q - m.j1_xy).norm()});
      }
      return spots;
    };
  for (const Eigen::Vector2d & p0 : {Eigen::Vector2d(0.26, 0.0), Eigen::Vector2d(0.25, 0.03), Eigen::Vector2d(0.24, -0.02)}) {
    const auto spots = spots_for(p0, {});
    const auto o = manipulation::order_regrasp_spots(spots, kGap, &p0);
    ASSERT_FALSE(o.empty());
    EXPECT_EQ(o.front().p, p0) << p0.transpose();
    EXPECT_EQ(o.size(), spots.size());
  }
  {   // (0.26, 0) 이웃 없음 → J1 거리 순서로는 20 째지만 맨 앞이어야 함
    const Eigen::Vector2d p0(0.26, 0.0);
    const auto spots = spots_for(p0, {});
    const auto old = manipulation::order_regrasp_spots(spots, kGap);
    EXPECT_NE(old.front().p, p0);
    EXPECT_EQ(manipulation::order_regrasp_spots(spots, kGap, &p0).front().p, p0);
  }
  {   // 원래 자리에 이웃이 붙음 (층 3) → 맨 앞으로 안 올림 (사이를 채우는 자리 먼저)
    const Eigen::Vector2d p0(0.26, 0.0);
    const auto spots = spots_for(p0, {sq(0.26, 0.0348)});
    const auto o = manipulation::order_regrasp_spots(spots, kGap, &p0);
    ASSERT_FALSE(o.empty());
    EXPECT_NE(o.front().p, p0);
    EXPECT_GE(o.front().gap, kGap);
  }
}

// 층 2 (사이 ≥ 30 · 가장자리 쪽) 는 모자란 가장자리 5 mm 칸 (0–5 · 5–10 …) 작은 순 → 같은 칸이면 J1 거리 칸 (5 mm) 가까운 순.
//   J1 거리 먼저면 사례 2131 모양이 앞오른쪽 구석 (모자람 29 mm) — 박스 안 여유가 먼저
TEST(RegraspSpot, Tier2EdgeBucketThenNearest)
{
  std::vector<RegraspSpot> s{{{0.25, 0.07}, 0.05, 0.001, 0, 0.235}, {{0.22, 0.07}, 0.05, 0.012, 1, 0.205}, {{0.221, 0.06}, 0.05, 0.004, 2, 0.2065}};
  std::vector<int> pref;
  for (const auto & x : manipulation::order_regrasp_spots(s, kGap)) {pref.push_back(x.pref);}
  EXPECT_EQ(pref, (std::vector<int>{2, 0, 1}));   // 모자람 0–5 mm 칸 (1 · 4 mm) 안에서 가까운 (206.5 < 235 mm), 그다음 10–15 칸 (12 mm)
  for (const auto & x : s) {EXPECT_EQ(manipulation::spot_tier(x, kGap), 2);}
}

// 박스가 붐빔 (앞줄 양옆 · 뒤에 네모): 사이 30 mm 자리가 없음 → 거절하지 않고 닿는 자리 중 가장 넓은 자리 (대체)
TEST(RegraspSpot, CrowdedBoxFallsBackToWidestReachable)
{
  const Eigen::Vector2d p0(0.214, -0.010);
  const std::vector<NeighborPuck> nb{sq(0.216, -0.068), sq(0.216, 0.048), sq(0.280, -0.010), sq(0.262, 0.075), sq(0.262, -0.080)};
  const auto r = run_spots(p0, nb, false);
  ASSERT_TRUE(r.ok) << say(r);
  EXPECT_EQ(r.tier, 3) << say(r);
  EXPECT_LT(r.best_gap, kGap);
  EXPECT_GT(r.spot.gap, 0.0) << say(r);   // 가장 넓은 쪽 (이웃과 겹치지 않음)
  EXPECT_LT(r.ms, 800.0) << say(r);
  std::printf("[ 꽉 참 ] %s (가장 넓은 후보 %.1f mm)\n", say(r).c_str(), r.best_gap * 1000.0);
}
