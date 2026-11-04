// test_grasp_select.cpp — 이웃 퍽을 보고 닫힘 방향 · 벌림 폭 고르기 (grasp_select.hpp)

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "manipulation/grasp_select.hpp"

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

// 7) 면 방향 모르는 네모 목표 (외접 21.2 mm) 는 넣기 폭(안쪽 면 18.4 mm)으로 못 내려감 → 6 과 같은 배치면 거절
TEST(GraspSelect, UnknownSquareTargetNeedsWideOpening)
{
  const std::vector<NeighborPuck> nb{cyl(0.306, 0.0), cyl(0.194, 0.0), cyl(0.25, 0.050), cyl(0.25, -0.050)};
  EXPECT_FALSE(pick({true, false, 0.0}, manipulation::close_candidates(false, 0.0, 0.0), nb, 0.0).ok);
  EXPECT_NEAR(manipulation::target_half_along({true, false, 0.0}, 0.3), 0.015 * std::sqrt(2.0), 1e-12);
  EXPECT_NEAR(manipulation::target_half_along({true, true, 0.2}, 0.2 + M_PI / 2), 0.015, 1e-12);
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
