// test_camera_drift.cpp — 홈 사진 화소 쌍으로 손목 카메라 회전 되찾기 · 보정 방향 (검출점 R · p 가 맞는 점)

#include <gtest/gtest.h>

#include <random>
#include <vector>

#include "manipulation/camera_drift.hpp"

using manipulation::CameraDrift;

namespace
{
const double fx = 606.4, fy = 606.2, cx = 325.2, cy = 256.8;
Eigen::Vector2d px(const Eigen::Vector3d & p) {return {fx * p.x() / p.z() + cx, fy * p.y() / p.z() + cy};}
}  // namespace

TEST(CameraDrift, RecoversRotationAndCorrectsDetections)
{
  // 실제 카메라 = 명목 · R (경첩 x 축 3.4° + 옆 1.5° + 조금 비틀림)
  const Eigen::Matrix3d R = (Eigen::AngleAxisd(0.059, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(-0.026, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(0.007, Eigen::Vector3d::UnitZ())).toRotationMatrix();
  std::mt19937 g(3);
  std::uniform_real_distribution<double> ux(-0.12, 0.12), uz(0.08, 0.25);
  std::vector<Eigen::Vector2d> a, b;
  for (int i = 0; i < 150; ++i) {
    const Eigen::Vector3d B(ux(g), ux(g) * 0.8 + 0.05, uz(g));   // 몸통 점 (명목 카메라 좌표)
    a.push_back(px(B));
    b.push_back(px(R.transpose() * B));                          // 돌아간 카메라로 본 같은 점
  }
  CameraDrift d;
  ASSERT_TRUE(manipulation::camera_drift(a, b, fx, fy, cx, cy, d));
  EXPECT_LT((d.R - R).norm(), 1e-9);
  EXPECT_NEAR(d.deg, Eigen::AngleAxisd(R).angle() * 180.0 / M_PI, 1e-9);
  EXPECT_LT(d.resid_px, 1e-6);
  // 퍽 (명목 카메라 좌표 P) 을 돌아간 카메라가 보면 Rᵀ P 로 나옴 → R · (Rᵀ P) = P
  const Eigen::Vector3d P(0.03, 0.10, 0.30);
  EXPECT_LT((d.R * (R.transpose() * P) - P).norm(), 1e-9);
}

TEST(CameraDrift, NoMoveIsIdentityAndTooFewPointsFail)
{
  std::vector<Eigen::Vector2d> a{{100, 100}, {400, 300}, {250, 420}, {600, 200}};
  CameraDrift d;
  ASSERT_TRUE(manipulation::camera_drift(a, a, fx, fy, cx, cy, d));
  EXPECT_LT(d.deg, 1e-6);
  EXPECT_FALSE(manipulation::camera_drift({a[0], a[1]}, {a[0], a[1]}, fx, fy, cx, cy, d));
}
