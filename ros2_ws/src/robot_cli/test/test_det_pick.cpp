// test_det_pick.cpp — 손목 검출 → PICK 목표 (det_pick.hpp)

#include <gtest/gtest.h>

#include <cmath>

#include "robot_cli/det_pick.hpp"

using robot_cli::DetView;
using robot_cli::Xform;

namespace
{
// 카메라가 (0.2, 0, 0.3) 에서 아래를 봄: 광학 z = base −z (x 축 둘레 180°)
Xform down_cam() {return Xform{0.2, 0.0, 0.3, 1.0, 0.0, 0.0, 0.0};}

DetView puck(double x, double y, double z, const std::string & id = "3")
{
  DetView d;
  d.kind = "puck";
  d.label = "red";
  d.shape = "cylinder";
  d.id = id;
  d.x = x;
  d.y = y;
  d.z = z;
  d.qx = 1.0;   // 퍽 긴축 (z) = 카메라 −z = base 위
  d.qw = 0.0;
  return d;
}
}  // namespace

// 1) 선 퍽: 퍽 중심 base (0.21, 0.04, 0.10) → 집기 점은 위로 grasp_offset, 목표 z 축 = 위
TEST(DetPick, UprightPuckToGraspPoint)
{
  robot_cli::DetPick p;
  std::string why;
  ASSERT_TRUE(robot_cli::det_pick(puck(0.01, -0.04, 0.20), down_cam(), 0.015, p, why)) << why;
  EXPECT_NEAR(p.cx, 0.21, 1e-9);
  EXPECT_NEAR(p.cy, 0.04, 1e-9);
  EXPECT_NEAR(p.cz, 0.10, 1e-9);
  EXPECT_NEAR(p.x, 0.21, 1e-9);
  EXPECT_NEAR(p.y, 0.04, 1e-9);
  EXPECT_NEAR(p.z, 0.115, 1e-9);
  EXPECT_TRUE(p.upright);
  const double up = 1.0 - 2.0 * (p.qx * p.qx + p.qy * p.qy);   // 목표 z 축의 위쪽 성분
  EXPECT_NEAR(up, 1.0, 1e-9);
}

// 2) 방향 모름 (0,0,0,0): 섰다고 보고 위로 grasp_offset, 목표 방향도 0,0,0,0
TEST(DetPick, UnknownOrientation)
{
  DetView d = puck(0.0, 0.0, 0.2);
  d.qx = d.qy = d.qz = d.qw = 0.0;
  robot_cli::DetPick p;
  std::string why;
  ASSERT_TRUE(robot_cli::det_pick(d, down_cam(), 0.015, p, why)) << why;
  EXPECT_NEAR(p.z, 0.115, 1e-9);
  EXPECT_DOUBLE_EQ(p.qw, 0.0);
  EXPECT_DOUBLE_EQ(p.qx, 0.0);
}

// 3) 누운 퍽 · 위치 모름은 거절
TEST(DetPick, RejectLyingAndUnknownPosition)
{
  DetView lying = puck(0.0, 0.0, 0.2);
  lying.qx = std::sqrt(0.5);   // 카메라 x 축 둘레 90° → 긴축이 수평
  lying.qw = std::sqrt(0.5);
  robot_cli::DetPick p;
  std::string why;
  EXPECT_FALSE(robot_cli::det_pick(lying, down_cam(), 0.015, p, why));
  DetView nan = puck(NAN, 0.0, 0.2);
  EXPECT_FALSE(robot_cli::det_pick(nan, down_cam(), 0.015, p, why));
}

// 4) 고르기: 태그 · 하나뿐일 때
TEST(DetPick, ChooseDetection)
{
  std::string why;
  std::vector<DetView> one{puck(0, 0, 0.2, "3")};
  EXPECT_EQ(robot_cli::choose_det(one, "", why), 0);
  std::vector<DetView> two{puck(0, 0, 0.2, "3"), puck(0.05, 0, 0.2, "7")};
  EXPECT_EQ(robot_cli::choose_det(two, "", why), -1);   // 둘이면 태그로 골라야 함
  EXPECT_EQ(robot_cli::choose_det(two, "7", why), 1);
  EXPECT_EQ(robot_cli::choose_det(two, "9", why), -1);
  EXPECT_EQ(robot_cli::choose_det({}, "", why), -1);
}
