#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "navigation/pose_guard_logic.hpp"

using navigation::BiasLearner;
using navigation::GuardMetrics;

namespace
{
GuardMetrics healthy()
{
  GuardMetrics m;
  m.scan_age = 0.05;
  m.scan_hz = 10.0;
  m.scan_points = 260;
  m.match = 0.99;
  m.map_odom_age = 0.05;
  m.imu_age = 0.01;
  return m;
}
}  // namespace

TEST(PoseGuard, HealthyIsLevelZero)
{
  EXPECT_EQ(navigation::evaluate(healthy()).level, 0);
}

TEST(PoseGuard, StaleMapOdomStops)
{
  auto m = healthy();
  m.map_odom_age = 1.5;   // SLAM 이 멈춰 지난 TF 만 남음 → 지도 맞음이 높아도 정지
  EXPECT_EQ(navigation::evaluate(m).level, 2);
}

TEST(PoseGuard, ScanLossStops)
{
  auto m = healthy();
  m.scan_age = 0.8;
  EXPECT_EQ(navigation::evaluate(m).level, 2);
}

TEST(PoseGuard, UnknownMatchWarnsThenStops)
{
  auto m = healthy();
  m.match.reset();
  m.match_unknown_s = 1.0;
  EXPECT_EQ(navigation::evaluate(m).level, 1);
  m.match_unknown_s = 2.5;
  EXPECT_EQ(navigation::evaluate(m).level, 2);
}

TEST(PoseGuard, BigJumpAcceptedOnlyWithGoodMatch)
{
  auto m = healthy();
  m.jump_m = 0.30;
  EXPECT_EQ(navigation::evaluate(m).level, 1);
  m.match = 0.70;
  EXPECT_EQ(navigation::evaluate(m).level, 2);
}

// 5 개 중앙값엔 고치기 전 낮은 값이 남음 → 큰 보정은 고친 뒤 잰 맞음으로 판정
TEST(PoseGuard, BigJumpJudgedByMatchAfterJump)
{
  auto m = healthy();
  m.jump_m = 0.30;
  m.match = 0.70;                 // 중앙값 = 고치기 전 값들
  m.match_since_jump = 0.97;      // 고친 뒤 = 지도와 맞음
  EXPECT_EQ(navigation::evaluate(m).level, 1);
  m.match_since_jump = 0.70;      // 고친 뒤에도 안 맞음
  EXPECT_EQ(navigation::evaluate(m).level, 2);
}

TEST(BiasLearner, LearnsOncePerWindowWhenMovingAndHealthy)
{
  BiasLearner b(0.0, 0.3, 0.005, 5.0);
  // 5 s 동안 움직이며 map→odom 방향이 0.0016 rad/s 로 바뀜 (자이로가 덜 느낀 만큼)
  for (int k = 0; k <= 25; ++k) {b.add(0.2 * k, 0.0016 * 0.2 * k, true, 0);}
  EXPECT_NEAR(b.estimate(), 0.3 * 0.0016, 1e-6);
}

TEST(BiasLearner, IgnoresWindowWithWarningOrStanding)
{
  BiasLearner b(0.001, 0.3, 0.005, 5.0);
  for (int k = 0; k <= 25; ++k) {b.add(0.2 * k, 0.0016 * 0.2 * k, true, k == 10 ? 1 : 0);}
  EXPECT_DOUBLE_EQ(b.estimate(), 0.001);
  for (int k = 0; k <= 25; ++k) {b.add(10.0 + 0.2 * k, 0.0016 * 0.2 * k, false, 0);}   // 멈춰 있음
  EXPECT_DOUBLE_EQ(b.estimate(), 0.001);
}

TEST(BiasLearner, ClampsToMax)
{
  BiasLearner b(0.0049, 1.0, 0.005, 5.0);
  for (int k = 0; k <= 25; ++k) {b.add(0.2 * k, 0.01 * 0.2 * k, true, 0);}
  EXPECT_DOUBLE_EQ(b.estimate(), 0.005);
}

TEST(WallDistance, CapsAndCountsCells)
{
  std::vector<std::uint8_t> occ(7 * 1, 0);
  occ[0] = 1;
  const auto d = navigation::wall_distance(occ, 7, 1, 3);
  EXPECT_EQ(d[0], 0);
  EXPECT_EQ(d[1], 1);
  EXPECT_EQ(d[2], 2);
  EXPECT_EQ(d[6], 3);   // cap
}

TEST(MapCell, RotatedOriginUsesSameOccupiedCell)
{
  // 같은 벽 칸 중심을 origin 자세로 map 에 놓아도 셀은 변하지 않는다.
  for (const double yaw : {0.0, 0.7, -1.3, M_PI / 2}) {
    const double x = 1.2 + std::cos(yaw) * 0.275 - std::sin(yaw) * 0.425;
    const double y = -0.8 + std::sin(yaw) * 0.275 + std::cos(yaw) * 0.425;
    const auto cell = navigation::map_cell(x, y, 1.2, -0.8, yaw, 0.05);
    EXPECT_EQ(cell.first, 5);
    EXPECT_EQ(cell.second, 8);
  }
}

TEST(MapCell, NegativeCoordinatesStayOutside)
{
  const auto cell = navigation::map_cell(0.001, -0.001, 0.0, 0.0, M_PI / 2, 0.025);
  EXPECT_EQ(cell.first, -1);
  EXPECT_EQ(cell.second, -1);
}
