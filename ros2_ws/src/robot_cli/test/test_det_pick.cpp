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

// 3) 누운 원기둥 · 모양 모르는 누운 퍽 · 위치 모름은 거절
TEST(DetPick, RejectLyingCylinderAndUnknownPosition)
{
  DetView lying = puck(0.0, 0.0, 0.2);
  lying.qx = std::sqrt(0.5);   // 카메라 x 축 둘레 90° → 긴축이 수평
  lying.qw = std::sqrt(0.5);
  robot_cli::DetPick p;
  std::string why;
  EXPECT_FALSE(robot_cli::det_pick(lying, down_cam(), 0.015, p, why));
  EXPECT_NE(why.find("원기둥"), std::string::npos) << why;
  lying.shape = "";
  EXPECT_FALSE(robot_cli::det_pick(lying, down_cam(), 0.015, p, why));
  DetView nan = puck(NAN, 0.0, 0.2);
  EXPECT_FALSE(robot_cli::det_pick(nan, down_cam(), 0.015, p, why));
}

// 3b) 누운 네모: 퍽 중심 base (0.20, 0, 0.10), 긴축 = base +y → 집기 점 = 중심 + 쥘 끝 (로봇 쪽 끝, 여기선 축이 로봇 방향과 직각이라 그대로)
//     × grasp_offset (수평), 목표 z = 긴축 (수평) · x = 위 (planning grasp_orientation)
TEST(DetPick, LyingBoxToGraspPoint)
{
  DetView d = puck(0.0, 0.0, 0.2);
  d.shape = "box";
  d.yaw_known = true;
  d.qx = std::sqrt(0.5);   // 카메라 x 축 둘레 90° → 퍽 긴축 = 카메라 −y = base +y (down_cam 은 base x 둘레 180°)
  d.qw = std::sqrt(0.5);
  robot_cli::DetPick p;
  std::string why;
  ASSERT_TRUE(robot_cli::det_pick(d, down_cam(), 0.015, p, why)) << why;
  EXPECT_FALSE(p.upright);
  EXPECT_NEAR(p.cx, 0.20, 1e-9);
  EXPECT_NEAR(p.cy, 0.0, 1e-9);
  EXPECT_NEAR(p.cz, 0.10, 1e-9);
  EXPECT_NEAR(p.z, 0.10, 1e-9);                                  // 수평으로 옮김 → 높이 그대로
  EXPECT_NEAR(std::hypot(p.x - p.cx, p.y - p.cy), 0.015, 1e-9);  // 긴축 방향으로 grasp_offset
  EXPECT_NEAR(p.x, 0.20, 1e-9);
  const double zz = 1.0 - 2.0 * (p.qx * p.qx + p.qy * p.qy);           // 목표 z 축 위쪽 성분 = 0 (수평)
  const double xz = 2.0 * (p.qx * p.qz - p.qw * p.qy);                 // 목표 x 축 위쪽 성분 = 1 (위)
  EXPECT_NEAR(zz, 0.0, 1e-9);
  EXPECT_NEAR(xz, 1.0, 1e-9);
  const double zy = 2.0 * (p.qy * p.qz - p.qw * p.qx);                 // 목표 z 축 = 집기 점 쪽 (중심 → 집기 점)
  EXPECT_NEAR(zy * 0.015, p.y - p.cy, 1e-9);
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
