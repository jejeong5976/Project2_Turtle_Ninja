// test_standup_table.cpp — 세우기 경로 표: 읽기 · 틀린 줄 거절 · 가장 가까운 경로 (180° 대칭 · 크기)

#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include "manipulation/standup_table.hpp"
#include "arm_kin_data.hpp"

using manipulation::StandupPath;

namespace
{
std::string write_tmp(const std::string & body)
{
  const std::string f = ::testing::TempDir() + "standup_paths_test.txt";
  std::ofstream(f) << body;
  return f;
}

// side x y axis rate n release end_x end_y end_face above[5] grasp[5] q[n × 5]
std::string row(double side, double x, double y, double axis, int n = 2, int release = 1)
{
  std::string s = std::to_string(side) + " " + std::to_string(x) + " " + std::to_string(y) + " " + std::to_string(axis) +
    " 0.9 " + std::to_string(n) + " " + std::to_string(release) + " 0.23 0.01 12.5 1 2 3 4 5 6 7 8 9 10";
  for (int i = 0; i < n * 5; ++i) {s += " " + std::to_string(2000 + i);}
  return s + "\n";
}
}  // namespace

TEST(StandupTable, LoadsRowsAndSkipsComments)
{
  const auto f = write_tmp("# 머리말\n\n" + row(0.0348, 0.23, 0.0, 15.0, 3, 1) + row(0.030, 0.245, -0.01, -30.0));
  std::vector<StandupPath> t;
  std::string why;
  ASSERT_TRUE(manipulation::load_standup_table(f, t, why)) << why;
  ASSERT_EQ(t.size(), 2u);
  EXPECT_DOUBLE_EQ(t[0].x, 0.23);
  EXPECT_EQ(t[0].release, 1);
  ASSERT_EQ(t[0].q.size(), 3u);
  EXPECT_EQ(t[0].q[2][4], 2014);
  EXPECT_EQ(t[0].grasp[0], 6);
  EXPECT_DOUBLE_EQ(t[0].end_face_deg, 12.5);
}

TEST(StandupTable, RejectsWrongPointCount)
{
  std::string bad = row(0.0348, 0.23, 0.0, 15.0, 2, 1);
  bad.insert(bad.size() - 1, " 7");   // 점 하나에 칸 하나 더
  const auto f = write_tmp(bad);
  std::vector<StandupPath> t;
  std::string why;
  EXPECT_FALSE(manipulation::load_standup_table(f, t, why));
  EXPECT_NE(why.find("칸 수"), std::string::npos) << why;
}

TEST(StandupTable, RejectsReleaseOutOfRange)
{
  const auto f = write_tmp(row(0.0348, 0.23, 0.0, 15.0, 2, 2));
  std::vector<StandupPath> t;
  std::string why;
  EXPECT_FALSE(manipulation::load_standup_table(f, t, why));
}

TEST(StandupTable, NearestUsesSizeAndHalfTurnSymmetry)
{
  std::vector<StandupPath> t(4);
  t[0].side = 0.0348; t[0].x = 0.23; t[0].y = 0.0; t[0].axis_deg = 80.0;
  t[1].side = 0.0348; t[1].x = 0.23; t[1].y = 0.0; t[1].axis_deg = 0.0;
  t[2].side = 0.030; t[2].x = 0.231; t[2].y = 0.001; t[2].axis_deg = -85.0;
  t[3].side = 0.0348; t[3].x = 0.26; t[3].y = 0.05; t[3].axis_deg = -85.0;
  double dx = 0, dy = 0, da = 0;
  // −85° 는 95° 와 같아서 80° (차 15°) 가 0° (차 85°) 보다 가까움. 크기가 다른 t[2] 는 안 고름
  EXPECT_EQ(manipulation::nearest_standup(t, 0.232, 0.0, -85.0, 0.0348, dx, dy, da), 0);
  EXPECT_NEAR(dx, 0.002, 1e-9);
  EXPECT_NEAR(da, 15.0, 1e-9);   // −85 − 80 = −165 → 180° 대칭으로 +15
  // 30 mm 퍽은 30 mm 경로만
  EXPECT_EQ(manipulation::nearest_standup(t, 0.26, 0.05, -85.0, 0.030, dx, dy, da), 2);
  EXPECT_EQ(manipulation::nearest_standup({}, 0.26, 0.05, -85.0, 0.030, dx, dy, da), -1);
}

TEST(StandupTable, FarpickTableNameRoundsNeighborDirection)
{
  EXPECT_EQ(manipulation::farpick_table_name(false, 30.0), "farpick_paths_ok75_nbr_none.txt");
  EXPECT_EQ(manipulation::farpick_table_name(true, 0.0), "farpick_paths_ok75_nbr_000.txt");
  EXPECT_EQ(manipulation::farpick_table_name(true, 50.0), "farpick_paths_ok75_nbr_045.txt");
  EXPECT_EQ(manipulation::farpick_table_name(true, -20.0), "farpick_paths_ok75_nbr_000.txt");
  EXPECT_EQ(manipulation::farpick_table_name(true, -90.0), "farpick_paths_ok75_nbr_270.txt");
  EXPECT_EQ(manipulation::farpick_table_name(true, 175.0), "farpick_paths_ok75_nbr_180.txt");   // 180 = −180
  EXPECT_EQ(manipulation::farpick_table_name(true, -170.0), "farpick_paths_ok75_nbr_180.txt");
}

TEST(StandupTable, FarpickNearestUsesQuarterTurnAndIgnoresAxisForCylinder)
{
  std::vector<StandupPath> t(3);
  t[0].side = 0.0348; t[0].x = 0.30; t[0].y = 0.0; t[0].axis_deg = 80.0;
  t[1].side = 0.0348; t[1].x = 0.30; t[1].y = 0.0; t[1].axis_deg = 0.0;
  t[2].side = 0.030; t[2].x = 0.30; t[2].y = 0.0; t[2].axis_deg = 45.0;
  double dx = 0, dy = 0, da = 0;
  // 네모 90° 주기: −85° 는 5° 와 같음 → 0° (차 5°) 가 80° (차 15°) 보다 가까움
  EXPECT_EQ(manipulation::nearest_farpick(t, 0.30, 0.0, -85.0, 0.0348, dx, dy, da, 90.0), 1);
  EXPECT_NEAR(da, 5.0, 1e-9);
  // 원기둥: 축 무시 (차 0), 크기 0.030 표만
  EXPECT_EQ(manipulation::nearest_farpick(t, 0.302, 0.0, -85.0, 0.030, dx, dy, da, 0.0), 2);
  EXPECT_NEAR(da, 0.0, 1e-9);
  EXPECT_NEAR(dx, 0.002, 1e-9);
}

// ---------------------------------------------------------------- 먼 누운 퍽 가운데 쥐기 표 (lyingpick)

namespace
{
// export_farpick_table.py --side 0.060 과 같은 머리말 · 칸 (release = 닫는 점)
std::string lp_table(const std::vector<std::string> & rows)
{
  std::string s = "# 먼 선 퍽 집기 경로 표 (farpick) — robot_sim_rl/rl/farpick(_res)_openloop.py record (Isaac, 팀 URDF, 강화학습) → export_farpick_table.py\n"
    "# 정책 test · 경로 n 개 · 점 간격 0.10 s · 칸 배치 = standup_paths.txt (release = 닫는 점)\n"
    "# side(0.030 원기둥/0.0348 네모) x y axis rate n close_idx end_x end_y end_face  first[5] close_pose[5] q[n x 5] (팀 명령 틱)\n";
  for (const auto & r : rows) {s += r;}
  return s;
}

std::string lp_row(double side, double x, double y, double axis, const std::vector<manipulation::Ticks> & q, int release, double rate = 0.88)
{
  char h[200];
  std::snprintf(h, sizeof(h), "%.4f %.4f %.4f %.1f %.2f %zu %d %.4f %.4f %.1f", side, x, y, axis, rate, q.size(), release, x, y, axis);
  std::string s = h;
  for (int v : q.front()) {s += " " + std::to_string(v);}
  for (int v : q[static_cast<size_t>(release)]) {s += " " + std::to_string(v);}
  for (const auto & t : q) {for (int v : t) {s += " " + std::to_string(v);}}
  return s + "\n";
}

std::vector<manipulation::Ticks> dummy_q(int n)
{
  std::vector<manipulation::Ticks> q(static_cast<size_t>(n));
  for (int i = 0; i < n; ++i) {q[static_cast<size_t>(i)] = {{3829, 2600 + i, 1400, 2000, 1765}};}
  return q;
}
}  // namespace

TEST(LyingPickTable, PathFollowsFarpickDirRule)
{
  using manipulation::lyingpick_table_path;
  EXPECT_EQ(lyingpick_table_path("/x/my.txt", "/s/share/bringup/config/standup_paths.txt"), "/x/my.txt");
  EXPECT_EQ(lyingpick_table_path("", "/s/share/bringup/config/standup_paths.txt"), "/s/share/bringup/config/lyingpick/lyingpick_paths_ok75.txt");
  EXPECT_EQ(lyingpick_table_path("", ""), "");
}

TEST(LyingPickTable, LoadsExportFormatAndChoosesHalfTurn)
{
  const auto q = dummy_q(5);
  const auto f = ::testing::TempDir() + "lyingpick_paths_test.txt";
  std::ofstream(f) << lp_table({lp_row(0.060, 0.31, 0.09, 0.0, q, 2), lp_row(0.060, 0.31, 0.09, 60.0, q, 2), lp_row(0.060, 0.31, 0.09, 120.0, q, 2),
    lp_row(0.060, 0.31, 0.09, 165.0, q, 3, 0.75), lp_row(0.0348, 0.31, 0.09, 30.0, q, 2)});
  std::vector<StandupPath> t;
  std::string why;
  ASSERT_TRUE(manipulation::load_standup_table(f, t, why)) << why;
  ASSERT_EQ(t.size(), 5u);
  EXPECT_DOUBLE_EQ(t[3].side, 0.060);
  EXPECT_DOUBLE_EQ(t[3].axis_deg, 165.0);
  EXPECT_DOUBLE_EQ(t[3].rate, 0.75);
  EXPECT_EQ(t[3].release, 3);   // 닫는 점
  EXPECT_EQ(t[3].grasp, q[3]);
  ASSERT_EQ(t[3].q.size(), 5u);
  EXPECT_EQ(t[3].q[4][1], 2604);
  // 178° = −2° → 0° 줄 (차 −2°, 180° 주기)
  auto c = manipulation::choose_lyingpick(t, 0.312, 0.087, 178.0, 12.0, 8.0);
  EXPECT_EQ(c.row, 0);
  EXPECT_NEAR(c.dyaw_deg, -2.0, 1e-9);
  EXPECT_NEAR(c.dx, 0.002, 1e-9);
  EXPECT_NEAR(c.dist_mm, std::hypot(2.0, 3.0), 1e-6);
  // −12° = 168° → 165° 줄 (차 +3°). 0° 줄은 차 −12°
  c = manipulation::choose_lyingpick(t, 0.31, 0.09, -12.0, 12.0, 8.0);
  EXPECT_EQ(c.row, 3);
  EXPECT_NEAR(c.dyaw_deg, 3.0, 1e-9);
  // 30° 는 0.0348 (선 네모) 줄에 딱 맞지만 누운 퍽 줄 (0 · 60°, 차 30°) 만 봄 → 한도 밖
  c = manipulation::choose_lyingpick(t, 0.31, 0.09, 30.0, 12.0, 8.0);
  EXPECT_EQ(c.row, -1);
  EXPECT_TRUE(c.nearest == 0 || c.nearest == 1);
  EXPECT_NEAR(std::fabs(c.dyaw_deg), 30.0, 1e-9);
  // 거리 한도 (13 mm > 12)
  c = manipulation::choose_lyingpick(t, 0.323, 0.09, 0.0, 12.0, 8.0);
  EXPECT_EQ(c.row, -1);
  EXPECT_EQ(c.nearest, 0);
  EXPECT_NEAR(c.dist_mm, 13.0, 1e-6);
}

TEST(LyingPickTable, ChoosesClosestRowInsideLimits)
{
  std::vector<StandupPath> t(2);
  t[0].side = 0.060; t[0].x = 0.31; t[0].y = 0.09; t[0].axis_deg = 9.0;    // 자리 같고 긴축 9° (한도 8° 밖) — 가장 가까움
  t[1].side = 0.060; t[1].x = 0.32; t[1].y = 0.09; t[1].axis_deg = 0.0;    // 10 mm 옆, 긴축 같음
  const auto c = manipulation::choose_lyingpick(t, 0.31, 0.09, 0.0, 12.0, 8.0);
  EXPECT_EQ(c.nearest, 0);
  EXPECT_EQ(c.row, 1);
  EXPECT_NEAR(c.dx, -0.01, 1e-9);
  EXPECT_NEAR(c.dyaw_deg, 0.0, 1e-9);
}

TEST(LyingPickTable, MissingOrEmptyFileKeepsCurrentGrasp)
{
  std::vector<StandupPath> t;
  std::string why;
  EXPECT_FALSE(manipulation::load_standup_table(::testing::TempDir() + "no_such_dir/lyingpick_paths_ok75.txt", t, why));
  EXPECT_TRUE(t.empty());
  const auto f = ::testing::TempDir() + "lyingpick_empty.txt";
  std::ofstream(f) << lp_table({});
  EXPECT_FALSE(manipulation::load_standup_table(f, t, why));
  EXPECT_NE(why.find("비어"), std::string::npos) << why;
  EXPECT_TRUE(t.empty());
  // 표가 비면 고를 줄이 없음 → control_master 는 지금 쥐기 (poses_for) 그대로
  const auto c = manipulation::choose_lyingpick(t, 0.315, 0.094, 30.0, 12.0, 8.0);
  EXPECT_EQ(c.row, -1);
  EXPECT_EQ(c.nearest, -1);
}

TEST(LyingPickTable, HeldCenterFollowsHandAndDropsCloseAxis)
{
  using manipulation::Grip;
  const double t = 20.0 * M_PI / 180.0;
  Grip g1;
  g1.tip = Eigen::Vector3d(0.30, 0.10, 0.02);
  g1.app = Eigen::Vector3d(std::sin(t), 0.0, -std::cos(t));
  g1.close = Eigen::Vector3d(0.0, 1.0, 0.0);
  const Eigen::Vector3d b1 = g1.app.cross(g1.close);
  const Eigen::Vector3d c1 = g1.tip + 0.005 * g1.app + 0.010 * b1 + 0.004 * g1.close;   // 닫힘축 4 mm 는 손가락이 맞춤
  const Eigen::Vector2d h = manipulation::held_center_in_hand(g1, c1);
  EXPECT_NEAR(h.x(), 0.005, 1e-12);
  EXPECT_NEAR(h.y(), 0.010, 1e-12);
  Grip g2;   // 다른 자리 · J1 90° 돈 손 (닫힘축 반대 부호)
  g2.tip = Eigen::Vector3d(0.22, -0.03, 0.04);
  g2.app = Eigen::Vector3d(0.0, std::sin(t), -std::cos(t));
  g2.close = Eigen::Vector3d(1.0, 0.0, 0.0);
  const Eigen::Vector3d at = manipulation::held_center_at(g2, h);
  const Eigen::Vector3d want = g2.tip + 0.005 * g2.app + 0.010 * g2.app.cross(g2.close);
  EXPECT_NEAR((at - want).norm(), 0.0, 1e-12);
}

// 실제 팔 모델로 만든 작은 표 한 줄 (정해진 쥐기: 위 120 → 40 mm → 쥐기 −27 mm → 들기 +33 mm, 닫는 동안 멈춤) → 실제 퍽으로 옮긴 경로 점
TEST(LyingPickTable, PathRetargetsGeneratedRowToMeasuredPuck)
{
  using namespace manipulation;
  const auto m = kin_data::model();
  const double kTop = 0.070 + 0.0348 + 0.0024 - 0.072;   // 박스 위 누운 퍽 윗면 (모델, test_park_stand 와 같음)
  const double x0 = 0.30, y0 = 0.07, ax0 = 30.0;          // 표 칸 (먼 쪽 누운 퍽)
  std::string why;
  const double cdeg = lying_close_rad(m, x0, y0, ax0) * 180.0 / M_PI;
  const auto P = pick_poses(m, x0, y0, kTop, ax0, {120.0, 40.0, -27.0, 33.0}, why, cdeg, 30.0);
  ASSERT_EQ(P.size(), 4u) << why;
  std::vector<Ticks> q;
  auto lerp = [&](const Ticks & a, const Ticks & b, int steps) {
      for (int k = 1; k <= steps; ++k) {
        Ticks t;
        for (int j = 0; j < 5; ++j) {t[j] = static_cast<int>(std::lround(a[j] + (b[j] - a[j]) * static_cast<double>(k) / steps));}
        q.push_back(t);
      }
    };
  q.push_back(P[0].ticks);
  lerp(P[0].ticks, P[1].ticks, 4);
  lerp(P[1].ticks, P[1].ticks, 3);   // 위에서 멈춤
  lerp(P[1].ticks, P[2].ticks, 5);
  const int release = static_cast<int>(q.size()) - 1;   // 쥐기 도착 = 닫는 점
  lerp(P[2].ticks, P[2].ticks, 6);   // 닫는 동안 멈춤
  lerp(P[2].ticks, P[3].ticks, 5);
  lerp(P[3].ticks, P[3].ticks, 2);
  const auto f = ::testing::TempDir() + "lyingpick_gen.txt";
  std::ofstream(f) << lp_table({lp_row(kLyingPickSide, x0, y0, ax0, q, release)});
  std::vector<StandupPath> t;
  ASSERT_TRUE(load_standup_table(f, t, why)) << why;
  // 잰 퍽: 칸에서 (+4, −3) mm · 긴축 +5° (+180° 로 와도 같은 줄)
  const double x = x0 + 0.004, y = y0 - 0.003, ax = ax0 + 5.0 + 180.0;
  const auto ch = choose_lyingpick(t, x, y, ax, 12.0, 8.0);
  ASSERT_EQ(ch.row, 0);
  EXPECT_NEAR(ch.dyaw_deg, 5.0, 1e-9);
  StandupRetarget rt;
  rt.from = Eigen::Vector2d(x0, y0);
  rt.to = Eigen::Vector2d(x, y);
  rt.da = ch.dyaw_deg * M_PI / 180.0;
  rt.tip_min_z = kTop - 0.0348 + 0.003;
  LyingPickPath lp;
  ASSERT_TRUE(lyingpick_path(m, t[0], rt, 0.017, 0.1, true, lp, why)) << why;
  ASSERT_EQ(lp.nodes.size(), q.size() - 1 - static_cast<size_t>(lp.skipped));
  EXPECT_EQ(lp.skipped, 0);
  EXPECT_EQ(lp.nodes.front().grip, "open");
  EXPECT_TRUE(lp.nodes.front().grip_early);
  EXPECT_DOUBLE_EQ(lp.nodes.front().dt_s, 0.0);
  int closes = 0;
  size_t ci = 0;
  for (size_t i = 0; i < lp.nodes.size(); ++i) {
    const auto & n = lp.nodes[i];
    if (i > 0) {EXPECT_NEAR(n.dt_s, 0.1, 1e-12) << n.name;}
    if (n.grip == "close") {++closes; ci = i;}
  }
  ASSERT_EQ(closes, 1);
  EXPECT_EQ(lp.nodes[ci].name, "reloc_grasp");
  EXPECT_EQ(ci, static_cast<size_t>(release - 1));   // 첫 점 (LOOK) 은 안 넣음
  for (size_t i = 0; i < ci; ++i) {EXPECT_TRUE(lyingpick_free_node(lp.nodes[i].name)) << lp.nodes[i].name;}
  for (size_t i = ci + 1; i < lp.nodes.size(); ++i) {EXPECT_FALSE(lyingpick_free_node(lp.nodes[i].name)) << lp.nodes[i].name;}
  EXPECT_EQ(lp.nodes.back().name, "reloc_lift");
  EXPECT_TRUE(lp.nodes.back().stop);
  // 닫는 점 손끝: 잰 퍽에 대해 표 칸 퍽에 대한 것과 같음 (긴축 차만큼 돌림)
  const Eigen::Vector3d tip_tab = arm_grip(m, ticks_to_q(m, P[2].ticks)).tip;
  const Eigen::Vector3d tip_now = arm_grip(m, lp.q_close).tip;
  const Eigen::Vector2d want = Eigen::Vector2d(x, y) + Eigen::Rotation2Dd(rt.da) * (tip_tab.head<2>() - Eigen::Vector2d(x0, y0));
  EXPECT_LT((tip_now.head<2>() - want).norm(), 0.001);
  EXPECT_NEAR(tip_now.z(), tip_tab.z(), 0.001);
  // 닫는 점부터 쥔 무게 처짐 앞먹임
  EXPECT_EQ(lp.nodes[ci].q, ticks_to_command(m, q_to_ticks(m, encoder_goal_q(m, lp.q_close, 0.017))));
  // 빈손이면 닫는 점 다음 점 (멈춤 = 같은 자세) 에서 내려온 길 (lp_in*) 을 거꾸로 LOOK 까지
  std::vector<PathNode> nodes{PathNode{"look", ticks_to_command(m, q.front()), false, 0.0, "", false}};
  nodes.insert(nodes.end(), lp.nodes.begin(), lp.nodes.end());
  std::vector<std::string> free;
  for (const auto & n : nodes) {if (lyingpick_free_node(n.name)) {free.push_back(n.name);}}
  const auto back = retrace_nodes(nodes, ci + 2, free, [](const PathNode & n) {return n.name == "look";});
  ASSERT_GE(back.size(), 2u);
  EXPECT_EQ(back.back().name, "look");
  for (size_t i = 1; i + 1 < back.size(); ++i) {EXPECT_TRUE(lyingpick_free_node(back[i].name)) << back[i].name;}
  EXPECT_TRUE(retrace_nodes(nodes, ci + 2, {}, [](const PathNode & n) {return n.name == "look";}).empty());   // 이름을 안 주면 (farpick) 없음
}

// ---------------------------------------------------------------- 실물 맞춤 (lyingpick_dz_mm · lyingpick_dxy_mm)

namespace
{
// 실제 팔 모델 정해진 쥐기 한 줄: 위 120 → 40 mm (멈춤) → 쥐기 −27 mm (닫는 동안 멈춤) → +33 mm 들기. 수직으로 내려감 (손끝이 퍽 중심 위)
struct GenRow
{
  StandupPath e;
  manipulation::StandupRetarget rt;
};
GenRow gen_row(const manipulation::ArmModel & m, double top)
{
  using namespace manipulation;
  const double x0 = 0.30, y0 = 0.07, ax0 = 30.0;
  std::string why;
  const auto P = pick_poses(m, x0, y0, top, ax0, {120.0, 40.0, -27.0, 33.0}, why, lying_close_rad(m, x0, y0, ax0) * 180.0 / M_PI, 30.0);
  EXPECT_EQ(P.size(), 4u) << why;
  GenRow g;
  if (P.size() != 4) {return g;}
  auto& q = g.e.q;
  auto lerp = [&](const Ticks & a, const Ticks & b, int steps) {
      for (int k = 1; k <= steps; ++k) {
        std::array<int, 5> t;
        for (int j = 0; j < 5; ++j) {t[j] = static_cast<int>(std::lround(a[j] + (b[j] - a[j]) * static_cast<double>(k) / steps));}
        q.push_back(t);
      }
    };
  q.push_back(P[0].ticks);
  lerp(P[0].ticks, P[1].ticks, 4);
  lerp(P[1].ticks, P[1].ticks, 3);
  lerp(P[1].ticks, P[2].ticks, 5);
  g.e.release = static_cast<int>(q.size()) - 1;
  lerp(P[2].ticks, P[2].ticks, 6);
  lerp(P[2].ticks, P[3].ticks, 5);
  lerp(P[3].ticks, P[3].ticks, 2);
  g.e.side = kLyingPickSide;
  g.e.x = x0;
  g.e.y = y0;
  g.e.axis_deg = ax0;
  g.rt.from = Eigen::Vector2d(x0, y0);
  g.rt.to = Eigen::Vector2d(x0 + 0.003, y0 - 0.002);
  g.rt.tip_min_z = top - 0.0348 + 0.003;
  return g;
}

Eigen::Vector3d tip_of(const manipulation::ArmModel & m, const manipulation::Q5 & q) {return manipulation::arm_grip(m, q).tip;}
}  // namespace

TEST(LyingPickTable, DzShiftsNearPuckSegmentOnly)
{
  using namespace manipulation;
  const auto m = kin_data::model();
  const double kTop = 0.070 + 0.0348 + 0.0024 - 0.072;
  auto g = gen_row(m, kTop);
  ASSERT_FALSE(g.e.q.empty());
  std::string why;
  LyingPickPath a, b;
  ASSERT_TRUE(lyingpick_path(m, g.e, g.rt, 0.017, 0.1, true, a, why)) << why;
  auto rt = g.rt;
  rt.tip_min_z -= 0.006;   // control_master: dz < 0 이면 손끝 최저도 그만큼 내림
  ASSERT_TRUE(lyingpick_path(m, g.e, rt, 0.017, 0.1, true, b, why, Eigen::Vector3d(0.0, 0.0, -0.006))) << why;
  ASSERT_EQ(a.nodes.size(), b.nodes.size());
  ASSERT_EQ(a.q.size(), a.nodes.size());
  for (size_t i = 0; i < a.nodes.size(); ++i) {EXPECT_EQ(a.nodes[i].name, b.nodes[i].name);}
  // 위 점 쪽 (표 손끝이 닫는 점에서 80 mm 밖) 은 그대로
  EXPECT_EQ(a.nodes.front().q, b.nodes.front().q);
  // 닫는 점 · 들기 끝: 손끝만 6 mm 아래
  const Eigen::Vector3d dc = tip_of(m, b.q_close) - tip_of(m, a.q_close);
  EXPECT_NEAR(dc.z(), -0.006, 0.001);
  EXPECT_LT(dc.head<2>().norm(), 0.001);
  EXPECT_NEAR(arm_grip(m, b.q_close).app.dot(arm_grip(m, a.q_close).app), 1.0, 1e-3);   // 자세 방향 그대로
  const Eigen::Vector3d dl = tip_of(m, b.q.back()) - tip_of(m, a.q.back());
  EXPECT_NEAR(dl.z(), -0.006, 0.001);
  // 위 40 mm 점 (닫는 점에서 67 mm) 은 들어가는 중: (80 − 67) / 30 만큼
  const size_t above = 6;   // q[7] (위 멈춤 마지막) → 첫 점 (q[0]) 은 안 넣어 nodes 번호는 하나 앞
  ASSERT_EQ(a.nodes[above].name, "lp_in7");
  const double d_above = (tip_of(m, a.q[above]) - tip_of(m, a.q_close)).norm();
  ASSERT_GT(d_above, kLyingShiftIn);
  ASSERT_LT(d_above, kLyingShiftOut);
  const double w = (kLyingShiftOut - d_above) / (kLyingShiftOut - kLyingShiftIn);
  EXPECT_NEAR((tip_of(m, b.q[above]) - tip_of(m, a.q[above])).z(), -0.006 * w, 0.001);
}

TEST(LyingPickTable, RecenterOnlyAlongClosingAxis)
{
  using namespace manipulation;
  Grip g;
  g.tip = Eigen::Vector3d::Zero();
  g.app = Eigen::Vector3d(0.0, 0.0, -1.0);
  const double a = 44.0 * M_PI / 180.0;   // 닫힘축 44° (11-11 실물 줄 1443)
  g.close = Eigen::Vector3d(std::cos(a), std::sin(a), 0.0);
  const Eigen::Vector2d off(0.0018, 0.0050);
  const Eigen::Vector2d d = lying_recenter_shift(g, off);
  const Eigen::Vector2d c(std::cos(a), std::sin(a));
  EXPECT_NEAR((off + d).dot(c), 0.0, 1e-9);                                   // 닫힘축 어긋남 0
  EXPECT_NEAR((off + d).dot(Eigen::Vector2d(-c.y(), c.x())), off.dot(Eigen::Vector2d(-c.y(), c.x())), 1e-9);   // 긴축 방향 그대로
  g.close = Eigen::Vector3d(0.1, 0.0, 0.995);                                  // 닫힘축이 거의 수직
  EXPECT_LT(lying_recenter_shift(g, off).norm(), 1e-12);
}

TEST(LyingPickTable, HoverAboveCloseThenStraightDown)
{
  using namespace manipulation;
  const auto m = kin_data::model();
  const double kTop = 0.070 + 0.0348 + 0.0024 - 0.072;
  auto g = gen_row(m, kTop);
  ASSERT_FALSE(g.e.q.empty());
  std::string why;
  LyingPickPath a, b;
  ASSERT_TRUE(lyingpick_path(m, g.e, g.rt, 0.017, 0.1, true, a, why)) << why;
  LyingHover hv;
  hv.hover_m = 0.004;
  hv.hover_s = 0.3;
  hv.descend_s = 0.5;
  ASSERT_TRUE(lyingpick_path(m, g.e, g.rt, 0.017, 0.1, true, b, why, Eigen::Vector3d::Zero(), hv)) << why;
  ASSERT_EQ(b.q.size(), b.nodes.size());
  size_t kc = 0, kh = 0;
  for (size_t i = 0; i < b.nodes.size(); ++i) {
    if (b.nodes[i].name == "reloc_grasp") {kc = i;}
    if (b.nodes[i].name == "lp_in_hover") {kh = i;}
  }
  ASSERT_GT(kc, 0u);
  ASSERT_EQ(kh + 1, kc);   // 멈춤 점 바로 다음이 닫는 점
  EXPECT_TRUE(b.nodes[kh].stop);
  EXPECT_NEAR(b.nodes[kh].hold_s, 0.3, 1e-9);
  EXPECT_NEAR(b.nodes[kc].dt_s, 0.5, 1e-9);
  EXPECT_EQ(b.nodes[kc].grip, "close");
  EXPECT_TRUE(lyingpick_free_node(b.nodes[kh].name));   // 빈손 되돌아가기 길에 들어감
  // 멈춤 손끝 = 닫는 점 손끝 바로 위 4 mm, 같은 손 방향
  const Eigen::Vector3d d = tip_of(m, b.q[kh]) - tip_of(m, b.q_close);
  EXPECT_NEAR(d.z(), 0.004, 0.0005);
  EXPECT_LT(d.head<2>().norm(), 0.0005);
  EXPECT_NEAR(arm_grip(m, b.q[kh]).app.dot(arm_grip(m, b.q_close).app), 1.0, 1e-4);
  // 닫는 점 자체 · 들기 끝은 표 그대로
  EXPECT_LT((tip_of(m, b.q_close) - tip_of(m, a.q_close)).norm(), 1e-6);
  EXPECT_LT((tip_of(m, b.q.back()) - tip_of(m, a.q.back())).norm(), 1e-6);
  // 멈춤 점 앞 표 점은 닫는 점 손끝에서 7 mm (4 + 3) 밖
  for (size_t i = 0; i < kh; ++i) {EXPECT_GE((tip_of(m, b.q[i]) - tip_of(m, b.q_close)).norm(), 0.007 - 1e-9) << b.nodes[i].name;}
  // 0 이면 표 그대로
  LyingPickPath c;
  ASSERT_TRUE(lyingpick_path(m, g.e, g.rt, 0.017, 0.1, true, c, why, Eigen::Vector3d::Zero(), LyingHover{})) << why;
  ASSERT_EQ(c.nodes.size(), a.nodes.size());
  for (size_t i = 0; i < a.nodes.size(); ++i) {EXPECT_EQ(c.nodes[i].q, a.nodes[i].q);}
}

TEST(LyingPickTable, DxyShiftsNearPuckSegmentInBaseFrame)
{
  using namespace manipulation;
  const auto m = kin_data::model();
  const double kTop = 0.070 + 0.0348 + 0.0024 - 0.072;
  auto g = gen_row(m, kTop);
  ASSERT_FALSE(g.e.q.empty());
  std::string why;
  LyingPickPath a, b;
  ASSERT_TRUE(lyingpick_path(m, g.e, g.rt, 0.017, 0.1, true, a, why)) << why;
  ASSERT_TRUE(lyingpick_path(m, g.e, g.rt, 0.017, 0.1, true, b, why, Eigen::Vector3d(0.004, -0.003, 0.0))) << why;
  ASSERT_EQ(a.nodes.size(), b.nodes.size());
  EXPECT_EQ(a.nodes.front().q, b.nodes.front().q);
  for (const auto & [qa, qb] : {std::make_pair(a.q_close, b.q_close), std::make_pair(a.q.back(), b.q.back())}) {
    const Eigen::Vector3d d = tip_of(m, qb) - tip_of(m, qa);
    EXPECT_NEAR(d.x(), 0.004, 0.001);
    EXPECT_NEAR(d.y(), -0.003, 0.001);
    EXPECT_NEAR(d.z(), 0.0, 0.001);
  }
  // 닫는 동안 멈춘 점은 닫는 점과 같은 자세 그대로 (빈손 되돌아가기 고리 빼기)
  size_t ci = 0;
  for (size_t i = 0; i < b.nodes.size(); ++i) {if (b.nodes[i].grip == "close") {ci = i;}}
  EXPECT_EQ(b.nodes[ci].q, b.nodes[ci + 1].q);
}
