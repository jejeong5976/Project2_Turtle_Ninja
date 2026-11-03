// test_rack_tags.cpp — 칸 바닥 AprilTag 평면 교점 · 적재함 이동 맞춤 (rack_tags.hpp)

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <vector>

#include "manipulation/rack_tags.hpp"

using manipulation::TagPlaneConfig;

namespace
{

// 칸 위 높이 h 에서 아래를 보는 카메라 (광학 z = 아래, 기울기 tilt [rad] 를 x 축으로)
Eigen::Isometry3d camera_above(const Eigen::Vector3d & pos, double tilt, double yaw = 0.0)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(M_PI + tilt, Eigen::Vector3d::UnitX()))
    .toRotationMatrix();
  T.translation() = pos;
  return T;
}

// base_link 의 점들 → 카메라 정규 좌표
std::array<Eigen::Vector2d, 4> project(const Eigen::Isometry3d & T, const std::array<Eigen::Vector3d, 4> & pts)
{
  std::array<Eigen::Vector2d, 4> out;
  for (int i = 0; i < 4; ++i) {
    const Eigen::Vector3d c = T.inverse() * pts[i];
    out[i] = Eigen::Vector2d(c.x() / c.z(), c.y() / c.z());
  }
  return out;
}

std::array<Eigen::Vector3d, 4> square(const Eigen::Vector2d & center, double z, double side, double yaw)
{
  const Eigen::Rotation2Dd R(yaw);
  const double h = side / 2.0;
  const std::array<Eigen::Vector2d, 4> local{Eigen::Vector2d(-h, h), Eigen::Vector2d(h, h), Eigen::Vector2d(h, -h),
    Eigen::Vector2d(-h, -h)};
  std::array<Eigen::Vector3d, 4> out;
  for (int i = 0; i < 4; ++i) {
    const Eigen::Vector2d p = center + R * local[i];
    out[i] = Eigen::Vector3d(p.x(), p.y(), z);
  }
  return out;
}

}  // namespace

// 1) 기울어진 카메라에서도 태그 중심 · 방향 · 크기를 그대로 되찾음
TEST(RackTags, PlaneHitRecoversCenterYawSize)
{
  TagPlaneConfig c;
  const Eigen::Vector2d center(0.0812, -0.0721);
  for (double tilt : {0.0, 0.3, -0.5}) {
    for (double yaw : {0.0, 0.7, -2.0}) {
      const auto T = camera_above(Eigen::Vector3d(0.07, -0.05, 0.28), tilt, 0.4);
      const auto hit = manipulation::tag_on_plane(project(T, square(center, c.plane_z, c.tag_size_m, yaw)), T, c);
      ASSERT_TRUE(hit.ok) << hit.why;
      EXPECT_NEAR(hit.center.x(), center.x(), 1e-9);
      EXPECT_NEAR(hit.center.y(), center.y(), 1e-9);
      EXPECT_NEAR(std::remainder(hit.yaw - yaw, 2.0 * M_PI), 0.0, 1e-9);
      EXPECT_NEAR(hit.side_m, c.tag_size_m, 1e-9);
    }
  }
}

// 2) 태그가 다른 높이에 있으면 (컵 테두리 등) 크기가 틀어져 버림
TEST(RackTags, WrongHeightRejectedBySize)
{
  TagPlaneConfig c;
  const auto T = camera_above(Eigen::Vector3d(0.02, 0.10, 0.30), 0.2);
  const auto hit = manipulation::tag_on_plane(project(T, square({0.02, 0.11}, 0.1605, c.tag_size_m, 0.0)), T, c);
  EXPECT_FALSE(hit.ok);
}

// 3) 너무 비스듬한 광선 · 위를 보는 카메라 · NaN 은 버림
TEST(RackTags, BadViewRejected)
{
  TagPlaneConfig c;
  const auto pts = square({0.30, 0.0}, c.plane_z, c.tag_size_m, 0.0);
  const auto Tlow = camera_above(Eigen::Vector3d(0.0, 0.0, 0.17), -1.4);   // 거의 수평으로 멀리 봄
  EXPECT_FALSE(manipulation::tag_on_plane(project(Tlow, pts), Tlow, c).ok);
  std::array<Eigen::Vector2d, 4> nan;
  nan.fill(Eigen::Vector2d(std::nan(""), 0.0));
  EXPECT_FALSE(manipulation::tag_on_plane(nan, camera_above(Eigen::Vector3d(0.0, 0.0, 0.3), 0.0), c).ok);
  Eigen::Isometry3d up = Eigen::Isometry3d::Identity();   // 광학 z = 위
  up.translation() = Eigen::Vector3d(0.0, 0.0, 0.05);
  const std::array<Eigen::Vector2d, 4> sq{Eigen::Vector2d(-0.1, 0.1), Eigen::Vector2d(0.1, 0.1), Eigen::Vector2d(0.1, -0.1),
    Eigen::Vector2d(-0.1, -0.1)};
  EXPECT_FALSE(manipulation::tag_on_plane(sq, up, c).ok);
}

// 4) 적재함을 옮기고 돌린 만큼 맞춤 → 안 잰 칸 옮길 양도 맞음
TEST(RackTags, FitRecoversRackMove)
{
  const std::vector<Eigen::Vector2d> cells{{-0.110, 0.107}, {0.084, 0.107}, {0.0812, -0.0721}, {0.0158, -0.0681},
    {-0.0434, 0.111}};
  Eigen::Vector2d pivot = Eigen::Vector2d::Zero();
  for (const auto & p : cells) {pivot += p;}
  pivot /= static_cast<double>(cells.size());
  manipulation::RackFit truth;
  truth.dx = 0.003;
  truth.dy = -0.004;
  truth.dyaw = 1.5 * M_PI / 180.0;
  std::vector<Eigen::Vector2d> exp, meas;
  for (size_t i = 0; i < 3; ++i) {   // 세 칸만 잼
    exp.push_back(cells[i]);
    meas.push_back(cells[i] + manipulation::rack_shift(truth, cells[i], pivot));
  }
  const auto f = manipulation::fit_rack(exp, meas, pivot);
  ASSERT_TRUE(f.ok);
  EXPECT_TRUE(f.rotation);
  EXPECT_NEAR(f.dx, truth.dx, 1e-9);
  EXPECT_NEAR(f.dy, truth.dy, 1e-9);
  EXPECT_NEAR(f.dyaw, truth.dyaw, 1e-9);
  EXPECT_NEAR(f.rms, 0.0, 1e-9);
  const auto s = manipulation::rack_shift(f, cells[4], pivot);   // 안 잰 칸
  const auto t = manipulation::rack_shift(truth, cells[4], pivot);
  EXPECT_NEAR((s - t).norm(), 0.0, 1e-9);
}

// 5) 한 칸 · 가까운 두 칸은 평행 이동만, 빈 입력은 실패
TEST(RackTags, FitTranslationOnlyWhenSpanShort)
{
  const Eigen::Vector2d pivot(0.0, 0.0);
  const std::vector<Eigen::Vector2d> one{{0.08, -0.07}}, one_m{{0.082, -0.075}};
  const auto f1 = manipulation::fit_rack(one, one_m, pivot);
  ASSERT_TRUE(f1.ok);
  EXPECT_FALSE(f1.rotation);
  EXPECT_NEAR(f1.dx, 0.002, 1e-12);
  EXPECT_NEAR(f1.dy, -0.005, 1e-12);
  const std::vector<Eigen::Vector2d> two{{0.0812, -0.0721}, {0.0795, -0.114}}, two_m{{0.0822, -0.0711}, {0.0815, -0.112}};
  const auto f2 = manipulation::fit_rack(two, two_m, pivot);   // 42 mm 떨어진 두 칸
  ASSERT_TRUE(f2.ok);
  EXPECT_FALSE(f2.rotation);
  EXPECT_DOUBLE_EQ(f2.dyaw, 0.0);
  EXPECT_NEAR(f2.dx, 0.0015, 1e-12);
  EXPECT_NEAR(f2.dy, 0.0015, 1e-12);
  EXPECT_FALSE(manipulation::fit_rack({}, {}, pivot).ok);
}

TEST(RackTags, Median)
{
  EXPECT_DOUBLE_EQ(manipulation::median({3.0, 1.0, 2.0}), 2.0);
  EXPECT_DOUBLE_EQ(manipulation::median({4.0, 1.0, 3.0, 2.0}), 2.5);
  EXPECT_TRUE(std::isnan(manipulation::median({})));
}
