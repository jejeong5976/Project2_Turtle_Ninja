// test_pick_far.cpp — 먼 퍽 첫 쥐기 보정 (pick_far.hpp, pick_offset_far_mm · pick_far_r_m)
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "manipulation/pick_far.hpp"

using manipulation::far_pick_offset;

namespace
{
const Eigen::Vector2d kJ1(0.0225499941511, 0.0);   // config/arm_kinematics.yaml j1_xy (base_link)
}

// 실제 로봇의 먼 퍽 첫 쥐기 두 점 (반지름 0.232 · 0.225) 과 x 0.30 넘는 퍽은 보정, 가까운 퍽은 그대로
TEST(PickFar, AppliesOnlyBeyondRadiusFromJ1)
{
  const std::vector<double> off{0.0, -8.0};
  for (const Eigen::Vector2d & p : {Eigen::Vector2d(0.243, 0.071), Eigen::Vector2d(0.247, 0.006), Eigen::Vector2d(0.305, -0.02)}) {
    const auto o = far_pick_offset(p, kJ1, 0.22, off);
    EXPECT_TRUE(o.valid);
    EXPECT_TRUE(o.applied) << p.transpose() << " r " << o.r_m;
    EXPECT_NEAR(o.d.x(), 0.0, 1e-12);
    EXPECT_NEAR(o.d.y(), -0.008, 1e-12);
  }
  EXPECT_NEAR(far_pick_offset({0.243, 0.071}, kJ1, 0.22, off).r_m, 0.2316, 5e-4);
  EXPECT_NEAR(far_pick_offset({0.247, 0.006}, kJ1, 0.22, off).r_m, 0.2246, 5e-4);
  for (const Eigen::Vector2d & p : {Eigen::Vector2d(0.20, 0.0), Eigen::Vector2d(0.21, 0.08), Eigen::Vector2d(0.24, 0.0)}) {
    const auto o = far_pick_offset(p, kJ1, 0.22, off);
    EXPECT_FALSE(o.applied) << p.transpose() << " r " << o.r_m;   // 정면 x 0.24 = 반지름 0.2175
    EXPECT_DOUBLE_EQ(o.d.norm(), 0.0);
  }
  // 기준은 J1 축 반지름: 옆으로 먼 퍽도 (x 가 작아도) 보정
  EXPECT_TRUE(far_pick_offset({0.12, 0.20}, kJ1, 0.22, off).applied);
}

// 기본 [0, 0] = 보정 없음 · 값이 이상하면 안 더함 (거절 없음)
TEST(PickFar, DefaultAndBadValues)
{
  EXPECT_FALSE(far_pick_offset({0.30, 0.0}, kJ1, 0.22, {0.0, 0.0}).applied);
  auto o = far_pick_offset({0.30, 0.0}, kJ1, 0.22, {-8.0});
  EXPECT_FALSE(o.valid);
  EXPECT_FALSE(o.applied);
  o = far_pick_offset({0.30, 0.0}, kJ1, 0.22, {0.0, std::nan("")});
  EXPECT_FALSE(o.valid);
  EXPECT_FALSE(far_pick_offset({0.30, 0.0}, kJ1, std::nan(""), {0.0, -8.0}).applied);
  EXPECT_FALSE(far_pick_offset({std::nan(""), 0.0}, kJ1, 0.22, {0.0, -8.0}).applied);
  const auto x = far_pick_offset({0.30, 0.0}, kJ1, 0.22, {3.0, -8.0});
  EXPECT_TRUE(x.applied);
  EXPECT_NEAR(x.d.x(), 0.003, 1e-12);
}
