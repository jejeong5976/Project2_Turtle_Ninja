// test_cell_shift.cpp — 칸 경로 옮기기: 칸 근처만 옮기고 양 끝 자세는 그대로인가

#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include "manipulation/cell_shift.hpp"
#include "arm_kin_data.hpp"

using manipulation::ArmPath;
using manipulation::PathNode;

namespace
{
// config/arm_paths.yaml store_S9 의 점 (팀 명령 rad)
const std::vector<double> kLift{-0.07210, 0.60746, -1.22258, 0.41417, -0.13346};
const std::vector<double> kInCell{-0.94493, -0.38503, 0.02301, 0.26998, 0.56144};
const std::vector<double> kP0{-0.00614, -1.37751, 0.62740, -0.02148, 0.02301};
const std::vector<double> kRest{-0.00614, -1.45115, 0.94340, -0.11658, 0.02301};
const Eigen::Vector2d kCellS9(0.0812, -0.0721);

std::vector<double> lerp(const std::vector<double> & a, const std::vector<double> & b, double t)
{
  std::vector<double> q(5);
  for (int j = 0; j < 5; ++j) {q[j] = a[j] + (b[j] - a[j]) * t;}
  return q;
}

PathNode node(const std::string & name, const std::vector<double> & q, bool stand = false, const std::string & grip = "")
{
  PathNode n;
  n.name = name;
  n.q = q;
  n.stand = stand;
  n.grip = grip;
  return n;
}

// 들기 → (30 점) → 칸 안 → (30 점) → P0_복귀 → 받침_복귀
ArmPath store_path()
{
  ArmPath p{"store_S9", {}, true};
  p.nodes.push_back(node("lift_real", kLift));
  for (int i = 1; i < 30; ++i) {p.nodes.push_back(node("r" + std::to_string(i), lerp(kLift, kInCell, i / 30.0)));}
  p.nodes.push_back(node("in_cell", kInCell, false, "release"));
  for (int i = 1; i < 30; ++i) {p.nodes.push_back(node("b" + std::to_string(i), lerp(kInCell, kP0, i / 30.0)));}
  p.nodes.push_back(node("P0_복귀", kP0, true));
  p.nodes.push_back(node("받침_복귀", kRest, true));
  return p;
}

Eigen::Vector3d tip(const manipulation::ArmModel & m, const std::vector<double> & c)
{
  manipulation::Q5 q;
  for (int j = 0; j < 5; ++j) {
    const double tick = m.zero_tick[j] + c[j] * 4096.0 / (2.0 * M_PI);
    q[j] = (m.jm[j].q_deg_at_ref + m.jm[j].sign * (tick - m.jm[j].ref_tick) * 360.0 / 4096.0) * M_PI / 180.0;
  }
  return manipulation::arm_grip(m, q).tip;
}
}  // namespace

TEST(CellShift, MovesOnlyNearTheCell)
{
  const auto m = kin_data::model();
  ArmPath p = store_path();
  const ArmPath before = p;
  const Eigen::Vector2d d(0.003, -0.002);
  manipulation::CellShiftReport rep;
  std::string why;
  ASSERT_TRUE(manipulation::shift_cell_path(m, p, true, kCellS9, d, rep, why)) << why;
  EXPECT_GT(rep.moved, 0);
  EXPECT_LT(rep.worst_pos_mm, 0.2);
  EXPECT_LT(rep.worst_app_deg, 1.0);
  // 칸 안 점: 손끝이 d 만큼, 높이는 그대로
  const size_t k = 30;
  ASSERT_EQ(p.nodes[k].name, "in_cell");
  const Eigen::Vector3d moved = (tip(m, p.nodes[k].q) - tip(m, before.nodes[k].q)) * 1000.0;
  EXPECT_NEAR(moved.x(), 3.0, 0.2);
  EXPECT_NEAR(moved.y(), -2.0, 0.2);
  EXPECT_NEAR(moved.z(), 0.0, 0.2);
  EXPECT_EQ(p.nodes[k].grip, "release");
  // 첫 점 (PICK 끝) · P0_복귀 · 받침 그대로
  for (size_t i : {size_t{0}, p.nodes.size() - 2, p.nodes.size() - 1}) {
    for (int j = 0; j < 5; ++j) {EXPECT_DOUBLE_EQ(p.nodes[i].q[j], before.nodes[i].q[j]) << p.nodes[i].name;}
  }
}

TEST(CellShift, ZeroShiftKeepsThePath)
{
  const auto m = kin_data::model();
  ArmPath p = store_path();
  const ArmPath before = p;
  manipulation::CellShiftReport rep;
  std::string why;
  ASSERT_TRUE(manipulation::shift_cell_path(m, p, true, kCellS9, Eigen::Vector2d::Zero(), rep, why)) << why;
  for (size_t i = 0; i < p.nodes.size(); ++i) {
    for (int j = 0; j < 5; ++j) {EXPECT_NEAR(p.nodes[i].q[j], before.nodes[i].q[j], 1e-9);}
  }
}

TEST(CellShift, RejectsPathWithoutP0)
{
  const auto m = kin_data::model();
  ArmPath p = store_path();
  p.nodes[p.nodes.size() - 2].name = "x";
  const ArmPath before = p;
  manipulation::CellShiftReport rep;
  std::string why;
  EXPECT_FALSE(manipulation::shift_cell_path(m, p, true, kCellS9, Eigen::Vector2d(0.003, 0.0), rep, why));
  EXPECT_NE(why.find("P0_복귀"), std::string::npos);
  EXPECT_EQ(p.nodes[30].q, before.nodes[30].q);
  // RETRIEVE 는 P0 를 찾음
  EXPECT_FALSE(manipulation::shift_cell_path(m, p, false, kCellS9, Eigen::Vector2d(0.003, 0.0), rep, why));
  EXPECT_NE(why.find("P0"), std::string::npos);
}

TEST(CellShift, RejectsNonFinite)
{
  const auto m = kin_data::model();
  ArmPath p = store_path();
  manipulation::CellShiftReport rep;
  std::string why;
  EXPECT_FALSE(manipulation::shift_cell_path(m, p, true, kCellS9, Eigen::Vector2d(std::nan(""), 0.0), rep, why));
}

// 칸 경로를 옮긴 점은 관절 범위를 넘어도 됨: 보통 15 틱, J4 위쪽은 40 틱 (2470 = 실물 재생 j4_hi, 멈춤 ~2485)
TEST(CellShift, ShiftedPointMayPassJ4RangeBy40Ticks)
{
  const auto m = kin_data::model();
  const ArmPath p0 = store_path();
  const Eigen::Vector2d d(0.003, -0.002);
  ArmPath p = p0;
  manipulation::CellShiftReport rep;
  std::string why;
  ASSERT_TRUE(manipulation::shift_cell_path(m, p, true, kCellS9, d, rep, why)) << why;
  double top = -1e9;   // 옮긴 점의 J4 틱 최대
  for (size_t i = 0; i < p.nodes.size(); ++i) {
    if (p.nodes[i].q == p0.nodes[i].q) {continue;}
    top = std::max(top, m.zero_tick[3] + p.nodes[i].q[3] * 4096.0 / (2.0 * M_PI));
  }
  ASSERT_GT(top, -1e9);
  auto tight = m;
  tight.tick_range[3][1] = top - 35.0;   // 35 틱 넘음 → 됨
  ArmPath q = p0;
  EXPECT_TRUE(manipulation::shift_cell_path(tight, q, true, kCellS9, d, rep, why)) << why;
  tight.tick_range[3][1] = top - 45.0;   // 45 틱 넘음 → 거절
  q = p0;
  EXPECT_FALSE(manipulation::shift_cell_path(tight, q, true, kCellS9, d, rep, why));
  EXPECT_NE(why.find("관절 4 범위 밖"), std::string::npos) << why;
}

// 놓는 점이 둘째 점에서 12 점보다 가까워도 (store_look_S1: 9 점째) 놓는 점은 다 옮김
TEST(CellShift, ReleaseNearTheStartGetsTheFullShift)
{
  const auto m = kin_data::model();
  ArmPath p{"store_look_S9", {}, true};
  p.nodes.push_back(node("rack_look", kLift));
  for (int i = 1; i < 10; ++i) {p.nodes.push_back(node("r" + std::to_string(i), lerp(kLift, kInCell, i / 10.0)));}
  p.nodes.push_back(node("in_cell", kInCell, false, "release"));
  for (int i = 1; i < 30; ++i) {p.nodes.push_back(node("b" + std::to_string(i), lerp(kInCell, kP0, i / 30.0)));}
  p.nodes.push_back(node("P0_복귀", kP0, true));
  const ArmPath before = p;
  manipulation::CellShiftReport rep;
  std::string why;
  ASSERT_TRUE(manipulation::shift_cell_path(m, p, true, kCellS9, Eigen::Vector2d(0.003, -0.002), rep, why, -0.004)) << why;
  const size_t k = 10;
  ASSERT_EQ(p.nodes[k].grip, "release");
  const Eigen::Vector3d moved = (tip(m, p.nodes[k].q) - tip(m, before.nodes[k].q)) * 1000.0;
  EXPECT_NEAR(moved.x(), 3.0, 0.2);
  EXPECT_NEAR(moved.y(), -2.0, 0.2);
  EXPECT_NEAR(moved.z(), -4.0, 0.2);
  for (int j = 0; j < 5; ++j) {EXPECT_DOUBLE_EQ(p.nodes[0].q[j], before.nodes[0].q[j]);}
}
