// test_cell_shift.cpp — 칸 경로 옮기기: 칸 근처만 옮기고 양 끝 자세는 그대로인가

#include <gtest/gtest.h>

#include <cmath>
#include <set>
#include <string>
#include <vector>

#include "manipulation/cell_shift.hpp"
#include "manipulation/grasp_guard.hpp"
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

// 놓는 점 (kInCell) 에서 J2 또는 J3 만 dq 돌려 손끝이 가장 높아지는 점 (팔 모델로 고름) — 칸 위 점 대신
std::vector<double> raised(const manipulation::ArmModel & m, double dq)
{
  std::vector<double> best = kInCell;
  double z_best = tip(m, kInCell).z();
  for (int j : {1, 2}) {
    for (double s : {-dq, dq}) {
      std::vector<double> q = kInCell;
      q[j] += s;
      const double z = tip(m, q).z();
      if (z > z_best) {z_best = z; best = q;}
    }
  }
  return best;
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

// 칸 경로를 옮긴 점은 관절 범위를 넘어도 됨: 보통 15 틱, J4 위쪽은 45 틱 (2475 = 기계 멈춤 2485 − 10, 새 칸 경로가 2455–2475)
TEST(CellShift, ShiftedPointMayPassJ4RangeBy50Ticks)
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
  tight.tick_range[3][1] = top - 45.0;   // 45 틱 넘음 → 됨 (11-10: 한도 50)
  ArmPath q = p0;
  EXPECT_TRUE(manipulation::shift_cell_path(tight, q, true, kCellS9, d, rep, why)) << why;
  tight.tick_range[3][1] = top - 55.0;   // 55 틱 넘음 → 거절
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

// 칸 위 멈춤 (store_settle_index): 놓는 점 앞에서 손끝이 놓는 점보다 up_m 이상 높은 마지막 점. release 없음 · 높은 점 없음이면 −1
TEST(CellShift, StoreSettleIndexPicksLastPointHighEnough)
{
  const auto m = kin_data::model();
  const std::vector<double> kHigh = raised(m, 0.5);
  const double dz = tip(m, kHigh).z() - tip(m, kInCell).z();
  ASSERT_GT(dz, 0.025) << "놓는 점 위로 25 mm 넘게 올린 점을 못 만듦";
  // 들기 → 높음 → 낮음 (놓는 점과 같은 높이) → 높음 → 낮음 → 놓는 점 → P0_복귀
  ArmPath p{"store_S9", {}, true};
  p.nodes.push_back(node("lift_real", kLift));
  p.nodes.push_back(node("r1", kHigh));
  p.nodes.push_back(node("r2", kInCell));
  p.nodes.push_back(node("r3", kHigh));
  p.nodes.push_back(node("r4", kInCell));
  p.nodes.push_back(node("in_cell", kInCell, false, "release"));
  p.nodes.push_back(node("P0_복귀", kP0, true));
  EXPECT_EQ(manipulation::store_settle_index(m, p, 0.020), 3);          // 20 mm 넘게 높은 마지막 점 (r4 · r2 는 놓는 점과 같은 높이)
  EXPECT_EQ(manipulation::store_settle_index(m, p, dz + 0.010), -1);   // 그만큼 높은 점 없음 (첫 점은 출발 자세라 안 봄)
  p.nodes[5].grip = "";   // 놓는 점 없음
  EXPECT_EQ(manipulation::store_settle_index(m, p, 0.020), -1);
}

// 새 칸 경로 (storeheld) 이름 → 칸: store_ / retrieve_ 다음의 S<n> (store_S5_d30 이 "d30" 이 되지 않게) · 새 경로 판별
TEST(CellShift, PathCellReadsTheCellAfterStoreOrRetrieve)
{
  EXPECT_EQ(manipulation::path_cell("store_S7"), "S7");
  EXPECT_EQ(manipulation::path_cell("retrieve_S12"), "S12");
  EXPECT_EQ(manipulation::path_cell("store_look_S7"), "S7");
  EXPECT_EQ(manipulation::path_cell("store_S5_d30"), "S5");
  EXPECT_EQ(manipulation::path_cell("store_S5_d30_low"), "S5");
  EXPECT_EQ(manipulation::path_cell("store_S11_d40_tilt20"), "S11");
  EXPECT_EQ(manipulation::path_cell("retrieve_S3_full"), "S3");
  EXPECT_EQ(manipulation::path_cell("rack_scan"), "");
  EXPECT_TRUE(manipulation::storeheld_path("store_S5_d30"));
  EXPECT_TRUE(manipulation::storeheld_path("store_S5_d30_low"));
  EXPECT_TRUE(manipulation::storeheld_path("store_S5_d50"));
  EXPECT_TRUE(manipulation::storeheld_path("store_S5_d40_tilt10"));
  EXPECT_TRUE(manipulation::storeheld_path("retrieve_S5_full"));
  EXPECT_FALSE(manipulation::storeheld_path("store_S5"));
  EXPECT_FALSE(manipulation::storeheld_path("store_look_S5"));
  EXPECT_FALSE(manipulation::storeheld_path("retrieve_S5"));
}

// STORE · RETRIEVE 새 경로 고르기: 쥔 깊이 (받침 다시 쥐기 30 ± 5 → _d30, 그 밖 → _d50) · 기울임 (가까운 _d40_tilt<n>, 없으면 "" = 지금 흐름)
TEST(CellShift, StoreheldNamePicksByDepthAndTilt)
{
  const std::set<std::string> have{"store_S5_d50", "store_S5_d30", "store_S5_d30_low", "store_S5_d40_tilt10", "store_S5_d40_tilt20",
    "store_S7_d50", "store_S7_d30", "retrieve_S5_full"};
  const auto has = [&](const std::string & n) {return have.count(n) > 0;};
  EXPECT_EQ(manipulation::storeheld_store_name("S5", 50.0, 30.0, 0.0, false, has), "store_S5_d50");
  EXPECT_EQ(manipulation::storeheld_store_name("S5", 0.0, 30.0, 0.0, false, has), "store_S5_d50");    // 깊이 모름 = 박스 퍽
  EXPECT_EQ(manipulation::storeheld_store_name("S5", 30.0, 30.0, 0.0, false, has), "store_S5_d30");
  EXPECT_EQ(manipulation::storeheld_store_name("S5", 34.0, 30.0, 0.0, true, has), "store_S5_d30_low");
  EXPECT_EQ(manipulation::storeheld_store_name("S7", 30.0, 30.0, 0.0, true, has), "store_S7_d30");    // _low 없음 (S7 네모) → _d30
  EXPECT_EQ(manipulation::storeheld_store_name("S5", 36.0, 30.0, 0.0, false, has), "store_S5_d50");   // 5 mm 넘게 다름 = 박스 퍽
  EXPECT_EQ(manipulation::storeheld_store_name("S5", 50.0, 30.0, 10.0, false, has), "store_S5_d40_tilt10");
  EXPECT_EQ(manipulation::storeheld_store_name("S5", 40.0, 30.0, 23.0, false, has), "store_S5_d40_tilt20");
  EXPECT_EQ(manipulation::storeheld_store_name("S5", 40.0, 30.0, 30.0, false, has), "");   // tilt30 없음 → 지금 흐름 (거절 · 다시 쥐기)
  EXPECT_EQ(manipulation::storeheld_store_name("S9", 50.0, 30.0, 0.0, false, has), "");    // 새 경로 없는 칸 → store_S9
  EXPECT_EQ(manipulation::storeheld_retrieve_name("S5", has), "retrieve_S5_full");
  EXPECT_EQ(manipulation::storeheld_retrieve_name("S7", has), "");
}

// STORE 놓은 뒤 곧게 올리기 (store_lift_clear): clear 0 이면 그대로 (끔)
TEST(CellShift, StoreLiftClearOffWhenZero)
{
  const auto m = kin_data::model();
  ArmPath p = store_path();
  const ArmPath before = p;
  int raised = -1;
  std::string why;
  ASSERT_TRUE(manipulation::store_lift_clear(m, p, kCellS9, 0.0, raised, why)) << why;
  EXPECT_EQ(raised, 0);
  ASSERT_EQ(p.nodes.size(), before.nodes.size());
  for (size_t i = 0; i < p.nodes.size(); ++i) {
    EXPECT_EQ(p.nodes[i].name, before.nodes[i].name);
    EXPECT_EQ(p.nodes[i].q, before.nodes[i].q);
    EXPECT_EQ(p.nodes[i].grip, before.nodes[i].grip);
  }
}

// 놓는 점 (release) 바로 뒤에 손끝을 곧게 clear 올린 점 (store_lift_clear, grip 없음 · 다음 점 속도) → 놓기는 원래 점 그대로, 그 뒤 칸 근처 낮은 점은 그 높이 이상
TEST(CellShift, StoreLiftClearInsertsAfterRelease)
{
  const auto m = kin_data::model();
  ArmPath p = store_path();
  p.nodes[31].speed_dps = 15.0;   // 놓는 점 다음 점 (b1)
  p.nodes[31].grip = "close";     // 실물 STORE 처럼 놓은 뒤 닫기 — 올려도 grip 은 그대로
  const ArmPath before = p;
  const double clear = 0.020;
  int raised = -1;
  std::string why;
  ASSERT_TRUE(manipulation::store_lift_clear(m, p, kCellS9, clear, raised, why)) << why;
  ASSERT_EQ(p.nodes.size(), before.nodes.size() + 1);
  const size_t k = 30;
  ASSERT_EQ(p.nodes[k].name, "in_cell");
  EXPECT_EQ(p.nodes[k].grip, "release");
  EXPECT_EQ(p.nodes[k].q, before.nodes[k].q);
  const auto & up = p.nodes[k + 1];
  EXPECT_EQ(up.name, "store_lift_clear");
  EXPECT_TRUE(up.grip.empty());
  EXPECT_FALSE(up.grip_on_way);
  EXPECT_FALSE(up.stop);
  EXPECT_DOUBLE_EQ(up.speed_dps, 15.0);
  const Eigen::Vector3d dp = (tip(m, up.q) - tip(m, before.nodes[k].q)) * 1000.0;
  EXPECT_NEAR(dp.x(), 0.0, 0.5);
  EXPECT_NEAR(dp.y(), 0.0, 0.5);
  EXPECT_NEAR(dp.z(), clear * 1000.0, 0.5);
  EXPECT_EQ(p.nodes[k + 2].name, "b1");
  EXPECT_EQ(p.nodes[k + 2].grip, "close");
  // 놓는 점 앞은 그대로 · 그 뒤는 칸 근처면 손끝이 놓는 점 + clear 이상 (0.5 mm 여유), 이름 · 순서 그대로
  for (size_t i = 0; i <= k; ++i) {EXPECT_EQ(p.nodes[i].q, before.nodes[i].q) << p.nodes[i].name;}
  const double z_need = tip(m, before.nodes[k].q).z() + clear;
  int higher = 0;
  for (size_t i = k + 1; i < before.nodes.size(); ++i) {
    const auto & a = p.nodes[i + 1];
    const auto & b = before.nodes[i];
    EXPECT_EQ(a.name, b.name);
    const Eigen::Vector3d t = tip(m, a.q);
    if (a.q != b.q) {
      ++higher;
      EXPECT_GT(t.z(), z_need - 0.0005) << a.name;
    }
  }
  EXPECT_EQ(higher, raised);
  for (size_t i : {p.nodes.size() - 2, p.nodes.size() - 1}) {EXPECT_EQ(p.nodes[i].q, before.nodes[i - 1].q) << p.nodes[i].name;}
}

// 놓는 점이 없으면 실패 + path 그대로 (control_master 는 거절하지 않고 기록 경로 그대로)
TEST(CellShift, StoreLiftClearFailsWithoutRelease)
{
  const auto m = kin_data::model();
  ArmPath p = store_path();
  p.nodes[30].grip = "";
  const ArmPath before = p;
  int raised = 0;
  std::string why;
  EXPECT_FALSE(manipulation::store_lift_clear(m, p, kCellS9, 0.020, raised, why));
  EXPECT_NE(why.find("놓는 점 (release) 없음"), std::string::npos) << why;
  ASSERT_EQ(p.nodes.size(), before.nodes.size());
  for (size_t i = 0; i < p.nodes.size(); ++i) {EXPECT_EQ(p.nodes[i].q, before.nodes[i].q);}
}

// RETRIEVE (retrieve_lift_clear) 는 그대로: 쥐는 점 (close) 뒤에 "lift_clear", 놓는 점은 안 봄 · 쥐는 점 없으면 같은 문구로 실패
TEST(CellShift, RetrieveLiftClearStillKeysOnClose)
{
  const auto m = kin_data::model();
  ArmPath r{"retrieve_S9", {}, false};
  r.nodes.push_back(node("P0", kP0, true));
  for (int i = 1; i < 30; ++i) {r.nodes.push_back(node("a" + std::to_string(i), lerp(kP0, kInCell, i / 30.0)));}
  r.nodes.push_back(node("in_cell", kInCell, false, "close"));
  for (int i = 1; i < 30; ++i) {r.nodes.push_back(node("b" + std::to_string(i), lerp(kInCell, kLift, i / 30.0)));}
  r.nodes.push_back(node("lift_real", kLift));
  r.nodes[31].speed_dps = 12.0;
  const ArmPath before = r;
  int raised = 0;
  std::string why;
  ASSERT_TRUE(manipulation::retrieve_lift_clear(m, r, kCellS9, 0.020, raised, why)) << why;
  ASSERT_EQ(r.nodes.size(), before.nodes.size() + 1);
  EXPECT_EQ(r.nodes[30].grip, "close");
  EXPECT_EQ(r.nodes[31].name, "lift_clear");
  EXPECT_TRUE(r.nodes[31].grip.empty());
  EXPECT_DOUBLE_EQ(r.nodes[31].speed_dps, 12.0);
  EXPECT_NEAR((tip(m, r.nodes[31].q) - tip(m, kInCell)).z() * 1000.0, 20.0, 0.5);
  // store_lift_clear 는 쥐는 점 (close) 을 안 봄 → 놓는 점 없음으로 실패
  ArmPath r2 = before;
  EXPECT_FALSE(manipulation::store_lift_clear(m, r2, kCellS9, 0.020, raised, why));
  // 놓은 뒤 닫는 STORE 경로: retrieve_lift_clear 는 닫는 점 뒤, store_lift_clear 는 놓는 점 뒤
  ArmPath s = store_path();
  s.nodes[40].grip = "close";
  ArmPath s_r = s, s_s = s;
  ASSERT_TRUE(manipulation::retrieve_lift_clear(m, s_r, kCellS9, 0.020, raised, why)) << why;
  EXPECT_EQ(s_r.nodes[41].name, "lift_clear");
  ASSERT_TRUE(manipulation::store_lift_clear(m, s_s, kCellS9, 0.020, raised, why)) << why;
  EXPECT_EQ(s_s.nodes[31].name, "store_lift_clear");
  ArmPath s0 = store_path();
  EXPECT_FALSE(manipulation::retrieve_lift_clear(m, s0, kCellS9, 0.020, raised, why));
  EXPECT_NE(why.find("쥐는 점 (close) 없음"), std::string::npos) << why;
}

// ---- STORE 놓은 뒤 닫기 늦추기 (store_delay_close) ----
namespace
{
// 놓는 점 (kInCell) 손끝을 칸 → 팔 J1 쪽 수평 side_mm · 위 up_mm 옮긴 자세 (손끝 위치 우선 감쇠 최소제곱, 접근축은 덜 맞춤).
//   손끝 계산만 보는 시험이라 관절 범위는 안 봄 (kInCell 은 J4 범위 끝이라 tip_offset_command 는 40 mm 넘게 못 올림). 0.1 mm 안 못 맞추면 빈 목록
std::vector<double> at_mm(const manipulation::ArmModel & m, double side_mm, double up_mm)
{
  using V5 = Eigen::Matrix<double, 5, 1>;
  using F6 = Eigen::Matrix<double, 6, 1>;
  auto feat = [&](const V5 & c) {
      const manipulation::Grip g = manipulation::arm_grip(m, manipulation::team_to_model_q(m, std::vector<double>(c.data(), c.data() + 5)));
      F6 f;
      f << g.tip * 1000.0 * 10.0, g.app * 10.0;
      return f;
    };
  const Eigen::Vector2d to_j1 = (m.j1_xy - tip(m, kInCell).head<2>()).normalized();   // 칸 → 팔 J1 쪽 (빠져나오는 쪽)
  const V5 c0 = Eigen::Map<const V5>(kInCell.data());
  F6 goal = feat(c0);
  goal.head<3>() += Eigen::Vector3d(to_j1.x() * side_mm, to_j1.y() * side_mm, up_mm) * 10.0;
  V5 c = c0;
  for (int it = 0; it < 50; ++it) {
    const F6 f = feat(c);
    Eigen::Matrix<double, 6, 5> J;
    for (int j = 0; j < 5; ++j) {
      V5 e = c;
      e[j] += 1e-6;
      J.col(j) = (feat(e) - f) / 1e-6;
    }
    c += (J.transpose() * J + 1e-3 * Eigen::Matrix<double, 5, 5>::Identity()).ldlt().solve(J.transpose() * (goal - f));
  }
  if ((feat(c) - goal).head<3>().norm() > 1.0) {return {};}   // 손끝 0.1 mm
  return std::vector<double>(c.data(), c.data() + 5);
}

// 기록 경로 모양: 들기 → 칸 안 놓기 → 곧게 10 mm (store_lift_clear) → 15 mm 닫기 (out_cell) → 30 · 45 mm → 옆 20 · 45 mm (높이 25)
//   → 그 위 60 mm → (10 점) → P0_복귀 → 받침_복귀
ArmPath store_close_path(const manipulation::ArmModel & m)
{
  ArmPath p{"store_S9", {}, true};
  p.nodes.push_back(node("lift_real", kLift));                    // 0
  p.nodes.push_back(node("in_cell", kInCell, false, "release"));  // 1
  p.nodes.push_back(node("store_lift_clear", at_mm(m, 0, 10)));   // 2
  p.nodes.push_back(node("out_cell", at_mm(m, 0, 15), false, "close"));   // 3
  p.nodes.push_back(node("u30", at_mm(m, 0, 30)));                // 4
  p.nodes.push_back(node("u45", at_mm(m, 0, 45)));                // 5
  p.nodes.push_back(node("s20", at_mm(m, 20, 25)));               // 6
  p.nodes.push_back(node("s45", at_mm(m, 45, 25)));               // 7
  p.nodes.push_back(node("u60", at_mm(m, 45, 60)));               // 8
  const auto top = p.nodes.back().q.size() == 5 ? p.nodes.back().q : kP0;   // 못 풀었으면 아래 all5 가 잡음
  for (int i = 1; i <= 10; ++i) {p.nodes.push_back(node("b" + std::to_string(i), lerp(top, kP0, i / 11.0)));}   // 9 .. 18
  p.nodes.push_back(node("P0_복귀", kP0, true));                  // 19
  p.nodes.push_back(node("받침_복귀", kRest, true));              // 20
  return p;
}

bool all5(const ArmPath & p)
{
  for (const auto & n : p.nodes) {
    if (n.q.size() != 5) {return false;}
  }
  return true;
}

int close_at(const ArmPath & p)
{
  int k = -1;
  for (size_t i = 0; i < p.nodes.size(); ++i) {
    if (p.nodes[i].grip == "close") {
      EXPECT_EQ(k, -1) << "닫기가 둘";
      k = static_cast<int>(i);
    }
  }
  return k;
}
}  // namespace

// 0 / 0 = 지금 그대로 (닫기 out_cell · 점 · 그리퍼 값 하나도 안 바뀜), 결과는 false
TEST(StoreDelayClose, ZeroZeroKeepsPath)
{
  const auto m = kin_data::model();
  ArmPath p = store_close_path(m);
  ASSERT_TRUE(all5(p)) << "손끝 옮긴 자세를 못 풂";
  p.nodes[3].grip_on_way = true;
  const ArmPath before = p;
  manipulation::StoreCloseMove mv;
  EXPECT_FALSE(manipulation::store_delay_close(m, p, 0.0, 0.0, mv));
  EXPECT_EQ(mv.release, 1);
  EXPECT_EQ(mv.from, 3);
  EXPECT_EQ(mv.to, 3);
  EXPECT_NEAR(mv.from_dz_mm, 15.0, 0.5);
  ASSERT_EQ(p.nodes.size(), before.nodes.size());
  for (size_t i = 0; i < p.nodes.size(); ++i) {
    EXPECT_EQ(p.nodes[i].q, before.nodes[i].q);
    EXPECT_EQ(p.nodes[i].grip, before.nodes[i].grip);
    EXPECT_EQ(p.nodes[i].grip_on_way, before.nodes[i].grip_on_way);
  }
  EXPECT_FALSE(manipulation::store_delay_close(m, p, -1.0, std::nan(""), mv));   // 0 이하 · NaN 도 안 봄
  EXPECT_EQ(close_at(p), 3);
}

// 50 / 40: 10 mm 위 (store_lift_clear) · 15 mm 닫기 · 30 · 45 mm 는 안 되고 옆 45 mm (s45) 첫 점으로 — 그리퍼 값만 옮기고 점은 그대로
TEST(StoreDelayClose, MovesToFirstPointHighOrAside)
{
  const auto m = kin_data::model();
  ArmPath p = store_close_path(m);
  ASSERT_TRUE(all5(p));
  p.nodes[3].grip_on_way = true;
  const ArmPath before = p;
  manipulation::StoreCloseMove mv;
  ASSERT_TRUE(manipulation::store_delay_close(m, p, 0.050, 0.040, mv));
  EXPECT_EQ(mv.from, 3);
  EXPECT_EQ(mv.to, 7);
  EXPECT_FALSE(mv.capped);
  EXPECT_NEAR(mv.to_dxy_mm, 45.0, 0.5);
  EXPECT_NEAR(mv.to_dz_mm, 25.0, 0.5);
  EXPECT_EQ(close_at(p), 7);
  EXPECT_TRUE(p.nodes[7].grip_on_way);
  EXPECT_TRUE(p.nodes[3].grip.empty());
  EXPECT_FALSE(p.nodes[3].grip_on_way);
  for (size_t i = 0; i < p.nodes.size(); ++i) {
    EXPECT_EQ(p.nodes[i].q, before.nodes[i].q) << p.nodes[i].name;
    EXPECT_EQ(p.nodes[i].stop, before.nodes[i].stop) << p.nodes[i].name;
  }
  EXPECT_EQ(p.nodes[1].grip, "release");
  // 높이만 (옆 0 = 안 봄): 60 mm 위 (u60) · 옆만 (높이 0): s45
  ArmPath h = before;
  ASSERT_TRUE(manipulation::store_delay_close(m, h, 0.050, 0.0, mv));
  EXPECT_EQ(close_at(h), 8);
  EXPECT_NEAR(mv.to_dz_mm, 60.0, 0.5);
  ArmPath s = before;
  ASSERT_TRUE(manipulation::store_delay_close(m, s, 0.0, 0.040, mv));
  EXPECT_EQ(close_at(s), 7);
  // 원래 닫는 점이 이미 조건을 맞추면 그대로 (앞으로 당기지 않음)
  ArmPath k = before;
  EXPECT_FALSE(manipulation::store_delay_close(m, k, 0.010, 0.0, mv));
  EXPECT_EQ(close_at(k), 3);
}

// 조건을 맞추는 점이 없으면 접기 (P0_복귀) 바로 앞 점에서 닫음 → lazy_fold 자르기 (cut_fold_tail) 그대로 되고 끝 점이 그리퍼가 아님.
//   그 사이 다른 그리퍼 점이 있으면 그 앞까지
TEST(StoreDelayClose, CapsBeforeFoldAndNextGrip)
{
  const auto m = kin_data::model();
  const ArmPath base = store_close_path(m);
  ASSERT_TRUE(all5(base));
  ArmPath p = base;
  manipulation::StoreCloseMove mv;
  ASSERT_TRUE(manipulation::store_delay_close(m, p, 1.0, 0.0, mv));   // 1 m 위는 없음
  EXPECT_TRUE(mv.capped);
  EXPECT_EQ(close_at(p), 18);
  EXPECT_EQ(p.nodes[19].name, "P0_복귀");
  const ArmPath fold{"fold", {base.nodes[19], base.nodes[20]}, true};
  ArmPath cut = p;
  EXPECT_TRUE(manipulation::cut_fold_tail(cut, fold));
  EXPECT_EQ(cut.nodes.back().name, "P0_복귀");
  // P0_복귀 가 없는 경로: 끝 앞 점
  ArmPath nf = base;
  nf.nodes[19].name = "p0";
  ASSERT_TRUE(manipulation::store_delay_close(m, nf, 1.0, 0.0, mv));
  EXPECT_EQ(mv.to, 19);
  // 다음 그리퍼 점 (두 번째 닫기) 앞까지만
  ArmPath g = base;
  g.nodes[6].grip = "close";
  ASSERT_TRUE(manipulation::store_delay_close(m, g, 0.050, 0.040, mv));
  EXPECT_TRUE(mv.capped);
  EXPECT_EQ(mv.to, 5);
  EXPECT_EQ(g.nodes[5].grip, "close");
  EXPECT_EQ(g.nodes[6].grip, "close");
  // 놓기 · 닫기 없음 → 그대로
  ArmPath nr = base;
  nr.nodes[1].grip.clear();
  EXPECT_FALSE(manipulation::store_delay_close(m, nr, 0.050, 0.040, mv));
  EXPECT_EQ(mv.from, -1);
  EXPECT_EQ(close_at(nr), 3);
  ArmPath nc = base;
  nc.nodes[3].grip.clear();
  EXPECT_FALSE(manipulation::store_delay_close(m, nc, 0.050, 0.040, mv));
  EXPECT_EQ(mv.from, -1);
}

// 가며 닫기 (grip_on_way, control_master gripper_on_the_way): 늦춰도 동작 수 · 시간 그대로, 닫는 시각만 늦어짐 · 받침 경로 마지막은 팔 동작 (home)
TEST(StoreDelayClose, OnTheWayCloseKeepsTiming)
{
  const auto m = kin_data::model();
  ArmPath p = store_close_path(m);
  ASSERT_TRUE(all5(p));
  p.nodes[3].grip_on_way = true;
  const manipulation::PathTiming t;
  const manipulation::GripperValues g;
  const auto a0 = manipulation::path_actions(p, t, g);
  manipulation::StoreCloseMove mv;
  ASSERT_TRUE(manipulation::store_delay_close(m, p, 0.050, 0.040, mv));
  const auto a1 = manipulation::path_actions(p, t, g);
  ASSERT_EQ(a0.size(), a1.size());
  ASSERT_EQ(a1.size(), 3u);   // 놓는 점까지 · 놓기 (그리퍼) · 받침까지 (가며 닫기)
  for (size_t i = 0; i < a0.size(); ++i) {
    EXPECT_EQ(a0[i].is_gripper, a1[i].is_gripper);
    EXPECT_NEAR(a0[i].duration_s, a1[i].duration_s, 1e-9);
  }
  EXPECT_GT(a0[2].side_grip_t, 0.0);
  EXPECT_GT(a1[2].side_grip_t, a0[2].side_grip_t);
  EXPECT_LT(a1[2].side_grip_t, a1[2].duration_s);
  EXPECT_NEAR(a1[2].side_grip, g.close, 1e-12);
  EXPECT_EQ(a1.back().pose, "home");
  EXPECT_FALSE(a1.back().is_gripper);
  // 끝 점까지 밀려도 (접기 앞 점) 가며 닫기 그대로
  ArmPath c = store_close_path(m);
  c.nodes[3].grip_on_way = true;
  ASSERT_TRUE(manipulation::store_delay_close(m, c, 1.0, 0.0, mv));
  const auto a2 = manipulation::path_actions(c, t, g);
  ASSERT_EQ(a2.size(), 3u);
  EXPECT_NEAR(a2[2].duration_s, a0[2].duration_s, 1e-9);
  EXPECT_GT(a2[2].side_grip_t, a1[2].side_grip_t);
  EXPECT_LT(a2[2].side_grip_t, a2[2].duration_s);
}
