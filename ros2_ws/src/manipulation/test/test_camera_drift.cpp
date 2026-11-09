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

TEST(CameraDrift, NewReferenceKeepsEarlierDrift)
{
  // 실물: 기준 사진을 새로 찍을 때 카메라는 이미 명목에서 B (약 1.45°) 돌아 있음.
  //   새 기준 대비 측정 M 만으로는 명목 대비가 아님 → chain_drift(B, M) 로 이어야 퍽이 맞는 자리
  const Eigen::Matrix3d B = (Eigen::AngleAxisd(-0.0222, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(-0.0070, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(-0.0101, Eigen::Vector3d::UnitZ())).toRotationMatrix();
  const Eigen::Matrix3d M = (Eigen::AngleAxisd(0.010, Eigen::Vector3d::UnitX()) *
    Eigen::AngleAxisd(-0.004, Eigen::Vector3d::UnitY())).toRotationMatrix();   // 새 기준 뒤 더 돈 만큼
  std::mt19937 g(7);
  std::uniform_real_distribution<double> ux(-0.12, 0.12), uz(0.08, 0.25);
  std::vector<Eigen::Vector2d> a, b;
  for (int i = 0; i < 150; ++i) {
    const Eigen::Vector3d body(ux(g), ux(g) * 0.8 + 0.05, uz(g));   // 몸통 점 (명목 카메라 좌표)
    a.push_back(px(B.transpose() * body));                          // 새 기준 사진 = 명목 · B 카메라
    b.push_back(px((B * M).transpose() * body));                    // 지금 카메라 = 명목 · B · M
  }
  CameraDrift d;
  ASSERT_TRUE(manipulation::camera_drift(a, b, fx, fy, cx, cy, d));
  EXPECT_LT((d.R - M).norm(), 1e-9);   // 새 기준 대비 측정은 M 만
  const Eigen::Quaterniond q = manipulation::chain_drift(Eigen::Quaterniond(B), Eigen::Quaterniond(d.R));
  const Eigen::Vector3d P(0.025, -0.044, 0.334);         //  LOOK 에서 본 퍽 (카메라 좌표)
  const Eigen::Vector3d seen = (B * M).transpose() * P;  // 지금 카메라가 본 퍽
  EXPECT_LT((q * seen - P).norm(), 1e-9);                // 이어 쓰면 맞는 자리
  EXPECT_GT((d.R * seen - P).norm(), 0.006);             // 0 에서 다시 (예전) 면 B 만큼 틀림 — LOOK 거리에서 6 mm 넘게
  // base 가 단위면 측정 그대로 (기준 사진만 있고 ref_base.txt 가 없는 예전 상태)
  EXPECT_TRUE(manipulation::chain_drift(Eigen::Quaterniond::Identity(), Eigen::Quaterniond(d.R)).isApprox(Eigen::Quaterniond(d.R), 1e-12));
}

TEST(CameraDrift, RefBaseTagMatchesOnlyTheSameImage)
{
  // 검수: ref.png 만 바뀌고 ref_base.txt 가 남으면 옛 보정을 잘못 이어 씀 → 크기 · 수정 시각 짝이 같을 때만
  const std::string now = manipulation::ref_base_tag(307200, 1731079620);
  EXPECT_EQ(now, "ref 307200 1731079620");
  EXPECT_TRUE(manipulation::ref_base_tag_matches(now, now));
  EXPECT_TRUE(manipulation::ref_base_tag_matches("", now)) << "옛 형식 (사원수만) 은 그대로 받아 씀";
  EXPECT_TRUE(manipulation::ref_base_tag_matches("  \n", now));
  EXPECT_FALSE(manipulation::ref_base_tag_matches(now, manipulation::ref_base_tag(307200, 1731080000))) << "같은 크기라도 다시 찍었으면 안 씀";
  EXPECT_FALSE(manipulation::ref_base_tag_matches(now, manipulation::ref_base_tag(307201, 1731079620)));
  EXPECT_FALSE(manipulation::ref_base_tag_matches(now, "")) << "지금 ref.png 를 못 봄";
  EXPECT_FALSE(manipulation::ref_base_tag_matches("ref 307200", now)) << "줄이 깨짐";
}

TEST(CameraDrift, TotalDegIsOfTheChainedRotation)
{
  // 로그의 "이어 쓴 총 각" = 기준 때 보정 B 와 기준 대비 측정 M 을 이은 회전의 크기 (같은 축이면 합: 1.45° + 0.56°)
  const Eigen::Quaterniond B(Eigen::AngleAxisd(0.0253, Eigen::Vector3d::UnitX()));
  const Eigen::Quaterniond M(Eigen::AngleAxisd(0.0098, Eigen::Vector3d::UnitX()));
  EXPECT_NEAR(manipulation::drift_deg(M), 0.0098 * 180.0 / M_PI, 1e-9);
  EXPECT_NEAR(manipulation::drift_deg(manipulation::chain_drift(B, M)), 0.0351 * 180.0 / M_PI, 1e-9);
  EXPECT_NEAR(manipulation::drift_deg(Eigen::Quaterniond::Identity()), 0.0, 1e-12);
}
