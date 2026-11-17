// test_placed_memory.cpp — 놓은 퍽 기억 (remember_placed) · HOME J3 · J4 옮김
//   실제 로봇에서 넘어뜨린 순서: 먼 목표 (0.312, 0.061) 를 S (0.299, −0.023) 에 내려놓고 다시 쥐어 놓음 → 먼 목표 (0.317, −0.058) 의 S 로
//   (0.303, 0.026) 을 골라 (카메라 장 10 s 넘음 → 이웃 없음) 방금 놓은 퍽 3.6 cm 옆에 내려놓아 넘어뜨림. 가까운 칸은 그 뒤
//   (0.251, 0.061) · (0.257, −0.059) · (0.191, 0.059) · (0.197, −0.060).

#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <limits>
#include <vector>

#include "manipulation/arm_kinematics.hpp"
#include "manipulation/arm_path.hpp"
#include "manipulation/grasp_select.hpp"
#include "manipulation/placed_memory.hpp"
#include "arm_kin_data.hpp"

using namespace manipulation;

namespace
{
const double kMinX = 0.212, kMaxY = 0.075;   // regrasp_min_x · regrasp_max_abs_y (control_master 기본 · control.yaml)
const double kHalf = 0.0174;                 // 네모 반폭 (puck_lying_height / 2)
const double kMargin = 0.020;                // remember_placed_margin_mm 기본
const double kNeed = 0.030;                  // regrasp_spot_gap_mm 기본

// 정지 자리 (odom): base_link 원점 (1.2, −0.4), yaw 0.7 rad
struct Pose2
{
  Eigen::Matrix2d R;   // odom → base
  Eigen::Vector2d T;
  double dyaw;
  Eigen::Vector2d robot;
};

Pose2 stop_pose(double ox, double oy, double yaw)
{
  Pose2 p;
  // base → odom: o = Rb · b + t  →  odom → base: b = Rb^T (o − t) = R · o + T
  Eigen::Matrix2d Rb;
  Rb << std::cos(yaw), -std::sin(yaw), std::sin(yaw), std::cos(yaw);
  p.R = Rb.transpose();
  p.T = -p.R * Eigen::Vector2d(ox, oy);
  p.dyaw = -yaw;
  p.robot = Eigen::Vector2d(ox, oy);
  return p;
}

PlacedPuck placed_at(const Pose2 & s, double bx, double by, double close_base, double t)
{
  PlacedPuck e;
  e.p = s.R.transpose() * (Eigen::Vector2d(bx, by) - s.T);
  e.robot = s.robot;
  e.t = t;
  e.yaw = close_base - s.dyaw;
  e.yaw_known = true;
  e.square = true;
  return e;
}

// place_far_plan 과 같은 자리 순서 (이웃 = 카메라 (없음) + 기억 여유 원) → 겹치지 않는 첫 자리. 없으면 NaN
Eigen::Vector2d first_putdown(const Eigen::Vector2d & j1, const Eigen::Vector2d & p0, const std::vector<NeighborPuck> & placed_inflated,
  int * skipped = nullptr)
{
  std::vector<Eigen::Vector2d> known{p0};
  for (const auto & n : placed_inflated) {known.push_back(n.c);}
  std::vector<RegraspSpot> spots;
  for (const auto & q : far_putdown_points(j1, p0, kMinX, kMaxY)) {
    spots.push_back({q, spot_gap(q, kHalf, placed_inflated, 0.015, kHalf), spot_edge_short(q, kMinX, kMaxY, 0.015, known),
      static_cast<int>(spots.size()), 0.0});
  }
  spots = order_regrasp_spots(std::move(spots), kNeed);
  int skip = 0;
  for (const auto & s : spots) {
    if (!putdown_clear(s.p, kHalf, placed_inflated)) {++skip; continue;}
    if (skipped) {*skipped = skip;}
    return s.p;
  }
  if (skipped) {*skipped = skip;}
  return Eigen::Vector2d::Constant(std::numeric_limits<double>::quiet_NaN());
}

// grasp_for (place) 와 같은 손가락 여유 고르기: 목표 방향 모름 (12 후보) · 열기 / 놓기 폭
std::vector<GraspChoice> place_grasps(const ArmModel & m, const Eigen::Vector2d & c, const std::vector<NeighborPuck> & nb, std::string & why)
{
  const double def = default_close_rad(m, c.x(), c.y(), std::nan(""));
  GraspLimits lim;
  lim.min_clear = 0.002;
  lim.target_clear = 0.003;
  return rank_grasps(c, TargetPuck{true, false, 0.0, 0.0, false}, close_candidates(false, 0.0, def), {0.019, 0.0093}, nb, def, why, lim);
}
}  // namespace

// 기억 → 다른 정지 자세의 base_link 로 옮겨도 같은 점 (odom 왕복)
TEST(PlacedMemory, OdomRoundTrip)
{
  const auto s = stop_pose(1.2, -0.4, 0.7);
  const auto e = placed_at(s, 0.312, 0.061, 1.3, 0.0);
  const auto nb = placed_neighbors({e}, s.R, s.T, s.dyaw, Eigen::Vector2d(0.0225, 0.0), 0.5);
  ASSERT_EQ(nb.size(), 1u);
  EXPECT_NEAR(nb[0].c.x(), 0.312, 1e-12);
  EXPECT_NEAR(nb[0].c.y(), 0.061, 1e-12);
  EXPECT_NEAR(std::remainder(nb[0].yaw - 1.3, 2.0 * M_PI), 0.0, 1e-12);
  EXPECT_TRUE(nb[0].yaw_known);
  // 차체가 5 cm 앞으로 → 퍽은 base 에서 5 cm 뒤로 보임
  const auto s2 = stop_pose(1.2 + 0.05 * std::cos(0.7), -0.4 + 0.05 * std::sin(0.7), 0.7);
  const auto nb2 = placed_neighbors({e}, s2.R, s2.T, s2.dyaw, Eigen::Vector2d(0.0225, 0.0), 0.5);
  ASSERT_EQ(nb2.size(), 1u);
  EXPECT_NEAR(nb2[0].c.x(), 0.262, 1e-9);
  EXPECT_NEAR(nb2[0].c.y(), 0.061, 1e-9);
  // 팔에서 0.5 m 밖은 안 씀
  const auto s3 = stop_pose(1.2 + 1.0 * std::cos(0.7), -0.4 + 1.0 * std::sin(0.7), 0.7);
  EXPECT_TRUE(placed_neighbors({e}, s3.R, s3.T, s3.dyaw, Eigen::Vector2d(0.0225, 0.0), 0.5).empty());
}

// 실물 순서: 기억 없으면 두 번째 먼 PLACE 의 첫 자리 = 실물이 고른 (0.303, 0.026). 기억 (+20 mm) 이 있으면 그 자리는 안 고르고
//   고른 자리는 놓은 퍽과 겹치지 않음. 그다음 가까운 칸 넷은 기억 이웃 (여유 없음) 으로도 손가락 자리가 있음
TEST(PlacedMemory, RealSequence)
{
  const auto m = kin_data::model();
  const Eigen::Vector2d j1 = m.j1_xy;
  const auto s = stop_pose(1.2, -0.4, 0.7);
  std::vector<PlacedPuck> mem;

  // ① 먼 목표 1 (0.312, 0.061): 기억 없음 → 첫 자리 (실물 S (0.299, −0.023) 은 같은 줄)
  const Eigen::Vector2d f1(0.312, 0.061), f2(0.317, -0.058);
  const auto s1 = first_putdown(j1, f1, {});
  ASSERT_TRUE(s1.allFinite());
  EXPECT_LT(s1.y(), 0.0);
  const double c1 = default_close_rad(m, f1.x(), f1.y(), std::nan(""));
  mem.push_back(placed_at(s, f1.x(), f1.y(), c1, 10.0));   // PLACE 성공 → 기억

  // ② 먼 목표 2 (0.317, −0.058): 기억 없이 = 실물 (0.303, 0.026)
  const auto blind = first_putdown(j1, f2, {});
  ASSERT_TRUE(blind.allFinite());
  EXPECT_NEAR(blind.x(), 0.303, 0.002);
  EXPECT_NEAR(blind.y(), 0.026, 0.002);
  EXPECT_LT((blind - f1).norm(), 0.037);   // 놓은 퍽과 중심 3.6 cm = 몸통 사이 약 1 mm → 넘어뜨림

  const auto placed_now = placed_neighbors(mem, s.R, s.T, s.dyaw, j1, 0.5);
  ASSERT_EQ(placed_now.size(), 1u);
  const auto infl = inflate_neighbors(placed_now, kMargin, kHalf);
  EXPECT_FALSE(putdown_clear(blind, kHalf, infl));
  int skipped = 0;
  const auto s2 = first_putdown(j1, f2, infl, &skipped);
  ASSERT_TRUE(s2.allFinite()) << "기억이 있으면 내려놓을 자리가 없음";
  int overlap = 0;   // 기억과 겹쳐 아예 안 고르는 자리 (순서로 뒤에 간 뒤 IK 가 앞 자리를 다 거절해도)
  for (const auto & q : far_putdown_points(j1, f2, kMinX, kMaxY)) {overlap += putdown_clear(q, kHalf, infl) ? 0 : 1;}
  EXPECT_GT(overlap, 0);
  skipped = overlap;
  EXPECT_GT((s2 - blind).norm(), 0.005);
  EXPECT_GE(spot_gap(s2, kHalf, infl), 0.0);
  EXPECT_GE((s2 - f1).norm(), kHalf + kMargin + kHalf - 1e-9);
  std::printf("먼 목표 2: 기억 없이 S (%.3f, %.3f) → 기억 있으면 S (%.3f, %.3f) (겹쳐 안 고르는 자리 %d 곳, 놓은 퍽과 %.1f mm)\n", blind.x(), blind.y(),
    s2.x(), s2.y(), skipped, (s2 - f1).norm() * 1000.0);
  const double c2 = default_close_rad(m, f2.x(), f2.y(), std::nan(""));
  mem.push_back(placed_at(s, f2.x(), f2.y(), c2, 20.0));

  // ③ 가까운 칸 넷: 기억 (여유 없음, 카메라 이웃과 같은 꼴) 으로 손가락 자리가 있고 25 mm 안 기억 없음
  const std::vector<Eigen::Vector2d> near{{0.251, 0.061}, {0.257, -0.059}, {0.191, 0.059}, {0.197, -0.060}};
  double t = 30.0;
  for (const auto & c : near) {
    const auto nb = placed_neighbors(mem, s.R, s.T, s.dyaw, j1, 0.5);
    for (const auto & n : nb) {EXPECT_GE((n.c - c).norm(), 0.025);}
    std::string why;
    const auto g = place_grasps(m, c, nb, why);
    ASSERT_FALSE(g.empty()) << "(" << c.x() << ", " << c.y() << "): " << why;
    std::string why2;
    const auto gi = place_grasps(m, c, inflate_neighbors(nb, kMargin, kHalf), why2);
    std::printf("가까운 칸 (%.3f, %.3f): 기억 이웃 %zu 개 → 손가락 여유 %.1f mm (여유 원 넣으면 %s)\n", c.x(), c.y(), nb.size(),
      g.front().clearance * 1000.0, gi.empty() ? "자리 없음" : "있음");
    mem.push_back(placed_at(s, c.x(), c.y(), default_close_rad(m, c.x(), c.y(), std::nan("")), t));
    t += 10.0;
  }
  EXPECT_EQ(mem.size(), 6u);
}

// 지우기: 0.6 m 넘게 움직임 · 시간 · 그 자리 PICK
TEST(PlacedMemory, Expire)
{
  const auto s = stop_pose(0.0, 0.0, 0.0);
  std::vector<PlacedPuck> v{placed_at(s, 0.30, 0.0, 0.0, 0.0), placed_at(s, 0.25, 0.06, 0.0, 100.0)};
  EXPECT_EQ(expire_placed(v, Eigen::Vector2d(0.5, 0.0), 120.0, 0.6, 600.0), 0);
  EXPECT_EQ(v.size(), 2u);
  EXPECT_EQ(expire_placed(v, Eigen::Vector2d(0.61, 0.0), 120.0, 0.6, 600.0), 2);
  v = {placed_at(s, 0.30, 0.0, 0.0, 0.0), placed_at(s, 0.25, 0.06, 0.0, 100.0)};
  EXPECT_EQ(expire_placed(v, Eigen::Vector2d(0.0, 0.0), 650.0, 0.6, 600.0), 1);   // 0 s 것만
  ASSERT_EQ(v.size(), 1u);
  EXPECT_EQ(expire_placed(v, Eigen::Vector2d(0.0, 0.0), 1e6, 0.6, 0.0), 0);       // 0 = 시간으로 안 지움
  EXPECT_EQ(forget_placed_near(v, Eigen::Vector2d(0.27, 0.06), 0.03), 1);
  EXPECT_TRUE(v.empty());
}

// 먼 PLACE 자리 후보 = place_far_plan 이 쓰던 줄과 같은 순서
TEST(PlacedMemory, FarPutdownPointsSameAsBefore)
{
  const Eigen::Vector2d j1(0.0225, 0.0), p0(0.317, -0.058);
  std::vector<Eigen::Vector2d> old;
  const double r0 = (p0 - j1).norm();
  const Eigen::Vector2d dir = (p0 - j1) / r0, side(-dir.y(), dir.x());
  for (double r = r0 - 0.03; r > 0.0 && j1.x() + dir.x() * r >= kMinX - 0.05; r -= 0.01) {
    for (double lat : {0.0, 0.04, -0.04, 0.08, -0.08}) {
      const Eigen::Vector2d q = j1 + dir * r + side * lat;
      if (q.x() < kMinX || std::fabs(q.y()) > kMaxY || (q - p0).norm() < 0.06) {continue;}
      old.push_back(q);
    }
  }
  const auto now = far_putdown_points(j1, p0, kMinX, kMaxY);
  ASSERT_EQ(now.size(), old.size());
  for (size_t i = 0; i < now.size(); ++i) {EXPECT_EQ(std::memcmp(now[i].data(), old[i].data(), sizeof(double) * 2), 0) << i;}
  EXPECT_TRUE(putdown_clear(now.front(), kHalf, {}));   // 기억 없음 = 거르지 않음
}

// HOME 옮김: 0 = 비트까지 같음. 제안 값의 손끝 (HOME, J1 +17 틱 = 지금 control.yaml)
TEST(HomeOffset, ZeroIsIdenticalAndSuggestedValues)
{
  std::vector<double> h{-0.0061, -1.4511, 0.9511, -0.1181, 0.0230};
  const auto h0 = h;
  add_joint_offsets(h, {{0.0, 0.0, 0.0, 0.0, 0.0}});
  EXPECT_EQ(std::memcmp(h.data(), h0.data(), sizeof(double) * 5), 0);
  add_joint_offsets(h, {{0.0, 0.0, -0.068, 0.138, 0.0}});
  EXPECT_DOUBLE_EQ(h[2], 0.9511 - 0.068);
  EXPECT_DOUBLE_EQ(h[3], -0.1181 + 0.138);
  EXPECT_EQ(std::memcmp(&h[0], &h0[0], sizeof(double)), 0);
  std::vector<double> short_q{1.0, 2.0};
  add_joint_offsets(short_q, {{1.0, 1.0, 1.0, 1.0, 1.0}});
  EXPECT_EQ(short_q, (std::vector<double>{1.0, 2.0}));

  const auto m = kin_data::model();
  const Q5 q0 = ticks_to_q(m, Ticks{{4084 + 17, 1102, 2668, 1971, 2063}});
  auto tip_after = [&](double d3, double d4) {   // 팀 명령 rad 옮김 → 모델 q (jm sign)
      Q5 q = q0;
      q[2] += m.jm[2].sign * d3;
      q[3] += m.jm[3].sign * d4;
      return Eigen::Vector3d(arm_grip(m, q).tip - arm_grip(m, q0).tip);
    };
  const Eigen::Vector3d a = tip_after(0.0, 0.095), b = tip_after(-0.068, 0.138);
  std::printf("HOME 손끝: J4 +0.095 → (%+.1f, %+.1f, %+.1f) mm · J3 −0.068 + J4 +0.138 → (%+.1f, %+.1f, %+.1f) mm\n", a.x() * 1e3, a.y() * 1e3,
    a.z() * 1e3, b.x() * 1e3, b.y() * 1e3, b.z() * 1e3);
  EXPECT_NEAR(a.x(), -0.015, 0.0015);
  EXPECT_LT(a.z(), -0.005);   // J4 만 = 손끝이 내려감
  EXPECT_NEAR(b.x(), -0.015, 0.0015);
  EXPECT_LT(std::fabs(b.z()), 0.0015);
  EXPECT_LT(std::fabs(b.y()), 0.0015);
}
