// park_stand: 누운 퍽 쥐기 (기울인 손 · 굴림) · 받침 넣기 자세
#include <cctype>
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <optional>
#include <string>
#include <vector>

#include "manipulation/park_stand.hpp"
#include "manipulation/look_at.hpp"
#include "arm_kin_data.hpp"

using namespace manipulation;

namespace
{
const double kTop = 0.070 + 0.0348 + 0.0024 - 0.072;   // 박스 위 누운 퍽 윗면 (모델)
const std::vector<double> kHeights{40.0, 10.0, -45.0};

// 쥔 퍽을 손 좌표로 옮겨 받침 자세 q 에서의 긴 쪽 끝 방향 · 끝면 가장 낮은 모서리
void puck_at(const ArmModel & m, const LyingGrasp & g, const Q5 & q, double side, double length, Eigen::Vector3d & a_s, double & low_z)
{
  auto frame = [](const Eigen::Vector3d & app, const Eigen::Vector3d & close) {
      Eigen::Matrix3d h;
      h.col(0) = close;
      h.col(1) = app.cross(close);
      h.col(2) = app;
      return h;
    };
  const Eigen::Matrix3d Hg = frame(g.app, g.close);
  const Grip s = arm_grip(m, q);
  const Eigen::Matrix3d Hs = frame(s.app.normalized(), s.close.normalized());
  a_s = Hs * Hg.transpose() * g.a_end;
  const Eigen::Vector3d c = s.tip + Hs * Hg.transpose() * (g.c - g.tip);
  const Eigen::Vector3d k = Hs * Hg.transpose() * g.close, n = Hs * Hg.transpose() * g.n;
  const Eigen::Vector3d f = c + (length / 2.0) * a_s;
  low_z = f.z() - side / 2.0 * (std::fabs(k.z()) + std::fabs(n.z()));
}

const ParkNode * find(const std::vector<ParkNode> & v, const std::string & name)
{
  for (const auto & n : v) {if (n.name == name) {return &n;}}
  return nullptr;
}
}  // namespace

// 수직 손 · 반지름 방향 누운 퍽: 바깥 끝을 쥠 (J1 쪽 끝을 쥐면 받침에서 긴 쪽이 J1 쪽으로 가려면 손목이 180° 돌아야 함 = 범위 밖,
//   시뮬 run_fixture 도 바깥 끝). 넣기 자세 = park_plan 식 (손끝 tr · tz)
TEST(ParkStand, VerticalGraspMatchesOldFormula)
{
  const auto m = kin_data::model();
  LyingGraspCfg gc;
  ParkCfg pk;
  std::vector<ParkNode> pn;
  double elev = 0.0;
  auto accept = [&](const LyingGrasp & c, std::string & w) {return park_nodes(m, pk, c, ticks_to_q(m, c.poses[0].ticks), pn, elev, w);};
  LyingGrasp g;
  std::string why;
  ASSERT_TRUE(lying_grasp(m, 0.205, 0.0, kTop, 0.0, kHeights, gc, g, why, 0.0, accept)) << why;
  EXPECT_DOUBLE_EQ(g.tilt_deg, 0.0);
  EXPECT_NEAR(g.tip.x(), 0.230, 1e-6);   // 바깥 끝으로 25 mm
  EXPECT_NEAR(g.tip.z(), kTop - 0.045, 1e-9);
  EXPECT_DOUBLE_EQ(elev, 45.0);
  const ParkNode * ins = find(pn, "park_tilt8");
  ASSERT_NE(ins, nullptr);
  EXPECT_EQ(ins->grip, "open");
  const double t = 45.0 * M_PI / 180.0, hc = 0.045 - 0.0348 / 2.0, reach = 0.025 + 0.030;
  const double tr = -pk.end_inset + hc * std::sin(t) + reach * std::cos(t);
  const double tz = pk.floor_z + pk.insert_clear - hc * std::cos(t) + reach * std::sin(t) + 0.0174 * std::cos(t);
  const Eigen::Vector2d rdir = (pk.center - m.j1_xy).normalized();
  const Eigen::Vector2d xy = pk.center + tr * rdir;
  const Grip s = arm_grip(m, ins->q);
  EXPECT_NEAR(s.tip.x(), xy.x(), 1e-3);
  EXPECT_NEAR(s.tip.y(), xy.y(), 1e-3);
  EXPECT_NEAR(s.tip.z(), tz, 1e-3);
}

// 당기며 놓기: 넣는 점은 loose, 같은 높이로 J1 쪽 release_pull 만큼 곧게, 끝점에서 open. 손 기울임 그대로
TEST(ParkStand, ReleasePullMovesTowardJ1)
{
  const auto m = kin_data::model();
  LyingGraspCfg gc;
  for (bool straight : {false, true}) {
    ParkCfg pk;
    pk.release_pull = 0.010;
    if (straight) {pk.insert_back = 0.015; pk.insert_extra = 0.003;}
    std::vector<ParkNode> pn;
    double elev = 0.0;
    auto accept = [&](const LyingGrasp & c, std::string & w) {return park_nodes(m, pk, c, ticks_to_q(m, c.poses[0].ticks), pn, elev, w);};
    LyingGrasp g;
    std::string why;
    ASSERT_TRUE(lying_grasp(m, 0.205, 0.0, kTop, 0.0, kHeights, gc, g, why, 0.0, accept)) << why;
    const ParkNode * ins = find(pn, straight ? "park_insert" : "park_tilt8");
    const ParkNode * p3 = find(pn, "park_pull3");
    ASSERT_NE(ins, nullptr);
    ASSERT_NE(p3, nullptr);
    EXPECT_EQ(ins->grip, "loose");
    EXPECT_EQ(find(pn, "park_pull1")->grip, "");
    EXPECT_EQ(p3->grip, "open");
    const Eigen::Vector2d rdir = (pk.center - m.j1_xy).normalized();
    const Grip a = arm_grip(m, ins->q), b = arm_grip(m, p3->q);
    EXPECT_NEAR((a.tip - b.tip).head<2>().dot(rdir), 0.010, 1e-3);
    EXPECT_NEAR(b.tip.z(), a.tip.z(), 1e-3);
    EXPECT_NEAR(a.app.normalized().dot(b.app.normalized()), 1.0, 1e-4);
    EXPECT_EQ(pn[static_cast<size_t>(p3 - pn.data()) + 1].name, "park_back");
  }
}

// 넣는 끝점을 손등 (손목 카메라) 쪽으로: release_up 없을 때보다 손 축에 수직 app × close 로 5 mm, 따로 멈추는 점 없음, 기울임 그대로
TEST(ParkStand, ReleaseUpShiftsInsertTowardCameraSide)
{
  const auto m = kin_data::model();
  LyingGraspCfg gc;
  auto plan = [&](bool straight, double up, std::vector<ParkNode> & pn) {
      ParkCfg pk;
      if (straight) {pk.insert_back = 0.015; pk.insert_extra = 0.003;}
      pk.release_up = up;
      double elev = 0.0;
      auto accept = [&](const LyingGrasp & c, std::string & w) {return park_nodes(m, pk, c, ticks_to_q(m, c.poses[0].ticks), pn, elev, w);};
      LyingGrasp g;
      std::string why;
      return lying_grasp(m, 0.205, 0.0, kTop, 0.0, kHeights, gc, g, why, 0.0, accept);
    };
  for (bool straight : {false, true}) {
    std::vector<ParkNode> p0, p5;
    ASSERT_TRUE(plan(straight, 0.0, p0));
    ASSERT_TRUE(plan(straight, 0.005, p5));
    EXPECT_EQ(p0.size(), p5.size());
    const std::string nm = straight ? "park_insert" : "park_tilt8";
    const ParkNode * a0 = find(p0, nm);
    const ParkNode * a5 = find(p5, nm);
    ASSERT_NE(a0, nullptr);
    ASSERT_NE(a5, nullptr);
    EXPECT_EQ(a5->grip, "open");
    const Grip g0 = arm_grip(m, a0->q), g5 = arm_grip(m, a5->q);
    const Eigen::Vector3d u = g0.app.cross(g0.close).normalized();
    EXPECT_NEAR((g5.tip - g0.tip).dot(u), 0.005, 3e-4);
    EXPECT_NEAR((g5.tip - g0.tip).norm(), 0.005, 3e-4);
    EXPECT_GT(u.z(), 0.3);   // 넣는 자세에서 손등 (카메라) 쪽 = 위 바깥
    EXPECT_NEAR(g0.app.normalized().dot(g5.app.normalized()), 1.0, 1e-4);
  }
}

// 밀기 끝점: push_extra 만큼 J1 쪽으로 더 (높이 그대로)
TEST(ParkStand, PushExtraGoesFurtherTowardJ1)
{
  const auto m = kin_data::model();
  LyingGraspCfg gc;
  auto push_end = [&](double extra) {
      ParkCfg pk;
      pk.push_extra = extra;
      std::vector<ParkNode> pn;
      double elev = 0.0;
      auto accept = [&](const LyingGrasp & c, std::string & w) {return park_nodes(m, pk, c, ticks_to_q(m, c.poses[0].ticks), pn, elev, w);};
      LyingGrasp g;
      std::string why;
      EXPECT_TRUE(lying_grasp(m, 0.205, 0.0, kTop, 0.0, kHeights, gc, g, why, 0.0, accept)) << why;
      const ParkNode * p4 = find(pn, "park_push4");
      return p4 ? arm_grip(m, p4->q).tip : Eigen::Vector3d(Eigen::Vector3d::Constant(1e9));
    };
  const Eigen::Vector3d a = push_end(0.0), b = push_end(0.004);
  const Eigen::Vector2d rdir = (ParkCfg{}.center - m.j1_xy).normalized();
  EXPECT_NEAR((a - b).head<2>().dot(rdir), 0.004, 3e-4);
  EXPECT_NEAR(a.z(), b.z(), 3e-4);
}

// 굴삭기처럼 밀기: 밀기 시작은 손끝이 J1 쪽으로 기울고 (app 의 반지름 성분 < 0) 끝은 수직, 끝점은 곧은 밀기와 같음
TEST(ParkStand, ScoopPushTiltsFingertipsTowardJ1ThenUpright)
{
  const auto m = kin_data::model();
  LyingGraspCfg gc;
  auto plan = [&](double scoop, std::vector<ParkNode> & pn) {
      ParkCfg pk;
      pk.push_scoop_deg = scoop;
      double elev = 0.0;
      auto accept = [&](const LyingGrasp & c, std::string & w) {return park_nodes(m, pk, c, ticks_to_q(m, c.poses[0].ticks), pn, elev, w);};
      LyingGrasp g;
      std::string why;
      return lying_grasp(m, 0.205, 0.0, kTop, 0.0, kHeights, gc, g, why, 0.0, accept);
    };
  std::vector<ParkNode> p0, p20;
  ASSERT_TRUE(plan(0.0, p0));
  ASSERT_TRUE(plan(20.0, p20));
  const Eigen::Vector2d rdir = (ParkCfg{}.center - m.j1_xy).normalized();
  const Grip s0 = arm_grip(m, find(p20, "park_push_down")->q);
  const Grip e0 = arm_grip(m, find(p0, "park_push4")->q), e20 = arm_grip(m, find(p20, "park_push4")->q);
  EXPECT_LT(s0.app.head<2>().normalized().dot(rdir) * s0.app.head<2>().norm(), -0.1);   // 손끝이 J1 쪽
  EXPECT_NEAR((e0.tip - e20.tip).norm(), 0.0, 1e-3);
  EXPECT_NEAR(e20.app.normalized().z(), -1.0, 1e-3);
}

// 밀기 J3 더하기: 밀기 점마다 J3 만 sin(π f) × push_dj3 더해짐 → 가운데 전부, 끝점 0 (다른 관절 · 다른 점은 그대로)
TEST(ParkStand, PushJ3OffsetRampsOverPush)
{
  const auto m = kin_data::model();
  LyingGraspCfg gc;
  auto plan = [&](double dj3, std::vector<ParkNode> & pn) {
      ParkCfg pk;
      pk.push_scoop_deg = 25.0;
      pk.push_dj3 = dj3;
      double elev = 0.0;
      auto accept = [&](const LyingGrasp & c, std::string & w) {return park_nodes(m, pk, c, ticks_to_q(m, c.poses[0].ticks), pn, elev, w);};
      LyingGrasp g;
      std::string why;
      return lying_grasp(m, 0.205, 0.0, kTop, 0.0, kHeights, gc, g, why, 0.0, accept);
    };
  std::vector<ParkNode> p0, p5;
  ASSERT_TRUE(plan(0.0, p0));
  ASSERT_TRUE(plan(-5.0 * M_PI / 180.0, p5));
  ASSERT_EQ(p0.size(), p5.size());
  for (size_t i = 0; i < p0.size(); ++i) {
    double want = 0.0;
    const std::string & nm = p0[i].name;
    if (nm.rfind("park_push", 0) == 0 && nm.size() == 10 && std::isdigit(static_cast<unsigned char>(nm[9]))) {
      want = -5.0 * M_PI / 180.0 * std::sin(M_PI * (nm[9] - '0') / ParkCfg{}.push_steps);
    }
    EXPECT_NEAR(p5[i].q[2] - p0[i].q[2], want, 1e-9) << nm;
    for (int j : {0, 1, 3, 4}) {EXPECT_NEAR(p5[i].q[j], p0[i].q[j], 1e-9) << nm << " J" << j + 1;}
  }
}

// 밀기 J2 더하기: + 면 밀기 가운데서 손끝이 올라가고 (J2 만 바뀜), 끝점은 그대로
TEST(ParkStand, PushJ2OffsetLiftsFingertipsMidPush)
{
  const auto m = kin_data::model();
  LyingGraspCfg gc;
  auto plan = [&](double dj2, std::vector<ParkNode> & pn) {
      ParkCfg pk;
      pk.push_scoop_deg = 25.0;
      pk.push_dj2 = dj2;
      double elev = 0.0;
      auto accept = [&](const LyingGrasp & c, std::string & w) {return park_nodes(m, pk, c, ticks_to_q(m, c.poses[0].ticks), pn, elev, w);};
      LyingGrasp g;
      std::string why;
      return lying_grasp(m, 0.205, 0.0, kTop, 0.0, kHeights, gc, g, why, 0.0, accept);
    };
  std::vector<ParkNode> p0, p3;
  ASSERT_TRUE(plan(0.0, p0));
  ASSERT_TRUE(plan(3.0 * M_PI / 180.0, p3));
  ASSERT_EQ(p0.size(), p3.size());
  EXPECT_GT(arm_grip(m, find(p3, "park_push2")->q).tip.z(), arm_grip(m, find(p0, "park_push2")->q).tip.z() + 0.003);
  EXPECT_NEAR((arm_grip(m, find(p3, "park_push4")->q).tip - arm_grip(m, find(p0, "park_push4")->q).tip).norm(), 0.0, 1e-6);
  for (size_t i = 0; i < p0.size(); ++i) {
    for (int j : {0, 2, 3, 4}) {EXPECT_NEAR(p3[i].q[j], p0[i].q[j], 1e-9) << p0[i].name << " J" << j + 1;}
  }
}

// 꽂아 누르기 (밀기 대신): 넣고 놓기까지 같음 → 놓은 손 (같은 기울임) 을 press_top 까지 곧게 올림 → 수직 · 닫음 → 받침 뒤 press_r 위 →
//   수직으로 곧게 바닥 + press_h 까지 → 위로. 밀기 점은 없음, 선 퍽 다시 쥐기는 그대로
TEST(ParkStand, PressModePlungesVerticallyAtBack)
{
  const auto m = kin_data::model();
  LyingGraspCfg gc;
  auto plan = [&](bool press, std::vector<ParkNode> & pn) {
      ParkCfg pk;
      pk.press_mode = press;
      double elev = 0.0;
      auto accept = [&](const LyingGrasp & c, std::string & w) {return park_nodes(m, pk, c, ticks_to_q(m, c.poses[0].ticks), pn, elev, w);};
      LyingGrasp g;
      std::string why;
      const bool ok = lying_grasp(m, 0.205, 0.0, kTop, 0.0, kHeights, gc, g, why, 0.0, accept);
      if (!ok) {std::printf("why: %s\n", why.c_str());}
      return ok;
    };
  std::vector<ParkNode> push, press;
  ASSERT_TRUE(plan(false, push));
  ASSERT_TRUE(plan(true, press));
  const ParkCfg pk;
  const Eigen::Vector2d r2 = (pk.center - m.j1_xy).normalized();
  for (const char * nm : {"park_over", "park_tilt8"}) {   // 넣기까지 같음
    ASSERT_NE(find(press, nm), nullptr) << nm;
    for (int j = 0; j < 5; ++j) {EXPECT_NEAR(find(press, nm)->q[j], find(push, nm)->q[j], 1e-9) << nm;}
  }
  EXPECT_EQ(find(press, "park_push1"), nullptr);
  const Grip gl = arm_grip(m, find(press, "park_press_lift")->q);
  EXPECT_NEAR(gl.tip.z(), pk.floor_z + pk.press_top, 1e-3);
  ASSERT_NE(find(press, "park_press_turn"), nullptr);
  EXPECT_EQ(find(press, "park_press_turn")->grip, "close");
  EXPECT_NEAR(arm_grip(m, find(press, "park_press_turn")->q).app.normalized().z(), -1.0, 1e-3) << "수직";
  double z_prev = 1e9;
  for (const std::string nm : {"park_press_over", "park_press1", "park_press2", "park_press3", "park_press4"}) {
    const ParkNode * n = find(press, nm);
    ASSERT_NE(n, nullptr) << nm;
    const Grip gp = arm_grip(m, n->q);
    EXPECT_NEAR((gp.tip.head<2>() - pk.center).dot(r2), pk.press_r, 1e-3) << nm << " 받침 뒤";
    EXPECT_NEAR(gp.app.normalized().z(), -1.0, 1e-3) << nm << " 수직";
    EXPECT_LT(gp.tip.z(), z_prev + 1e-6) << nm;
    z_prev = gp.tip.z();
  }
  EXPECT_NEAR(z_prev, pk.floor_z + pk.press_h, 1e-3) << "바닥 위 press_h 까지 꽂음";
  EXPECT_NE(find(press, "park_press_up"), nullptr);
  EXPECT_NE(find(press, "park_re_down"), nullptr) << "꽂은 뒤 다시 쥐기는 그대로";
}

// 먼 접선 방향 누운 퍽: 손을 기울이면 닫힘축이 팔 평면 안에서 기울어 퍽이 굴러 손가락에 맞음
TEST(ParkStand, FarTangentialPuckRollsIntoTiltedHand)
{
  const auto m = kin_data::model();
  LyingGraspCfg gc;
  LyingGrasp g;
  std::string why;
  ASSERT_TRUE(lying_grasp(m, 0.29, 0.0, kTop, 90.0, kHeights, gc, g, why, 30.0)) << why;
  EXPECT_GT(g.tilt_deg, 0.0);
  EXPECT_NEAR(g.roll_deg, g.tilt_deg, 1.0);
  EXPECT_LE(g.roll_deg, gc.max_roll_deg + 1e-9);
  EXPECT_GE(g.margin_mm, 2.5 - 1e-6);
  EXPECT_NEAR((g.c - g.tip).dot(g.close), 0.0, 1e-9);   // 손가락 가운데
  const double rho = g.roll_deg * M_PI / 180.0;
  EXPECT_NEAR(g.c.z(), kTop - 0.0348 + 0.0174 * (std::sin(rho) + std::cos(rho)), 1e-6);   // 아래 바닥 모서리로 섬
  const Eigen::Vector3d b = g.app.cross(g.close).normalized();
  const double low = g.tip.z() - gc.open_outer * std::fabs(g.close.z()) - gc.finger_half_w * std::fabs(b.z());
  EXPECT_GE(low, kTop - 0.045 - 1e-9);   // 열린 손가락이 수직 손 손끝보다 낮지 않음
  const Grip s = arm_grip(m, g.q_grasp);
  EXPECT_LT((s.tip - g.tip).norm(), 1e-3);
  EXPECT_GT(s.app.normalized().dot(g.app), std::cos(1.0 * M_PI / 180.0));
}

// 기울여 쥔 퍽 (굴림 · 손 안 기울기) 도 받침에서 긴 쪽 끝이 J1 쪽 아래로 elev, 가장 낮은 모서리 = 바닥 + insert_clear
TEST(ParkStand, TiltedGraspsInsertAtSameElevation)
{
  const auto m = kin_data::model();
  struct Case {double x, y, axis;};
  for (const Case & cs : {Case{0.29, 0.0, 90.0}, Case{0.25, 0.0, 0.0}, Case{0.21, 0.0, 90.0}, Case{0.27, 0.04, 45.0}, Case{0.27, 0.0, 90.0}}) {
    LyingGraspCfg gc;
    ParkCfg pk;
    std::vector<ParkNode> pn;
    double elev = 0.0;
    auto accept = [&](const LyingGrasp & c, std::string & w) {return park_nodes(m, pk, c, ticks_to_q(m, c.poses[0].ticks), pn, elev, w);};
    LyingGrasp g;
    std::string why;
    ASSERT_TRUE(lying_grasp(m, cs.x, cs.y, kTop, cs.axis, kHeights, gc, g, why, 30.0, accept)) << cs.x << " " << cs.axis << ": " << why;
    const ParkNode * ins = find(pn, "park_tilt8");
    ASSERT_NE(ins, nullptr);
    Eigen::Vector3d a_s;
    double low = 0.0;
    puck_at(m, g, ins->q, gc.side, gc.length, a_s, low);
    const Eigen::Vector2d rdir = (pk.center - m.j1_xy).normalized();
    EXPECT_NEAR(std::asin(-a_s.z()) * 180.0 / M_PI, elev, 1.0) << cs.x << " " << cs.axis;
    EXPECT_LT(a_s.head<2>().dot(rdir), 0.0);   // J1 쪽으로
    EXPECT_NEAR(low, pk.floor_z + pk.insert_clear, 1e-3) << cs.x << " " << cs.axis;
    EXPECT_NE(find(pn, "park_re_lift"), nullptr);
  }
}

// 20° 넘게 굴려야 하는 먼 접선 퍽 (0.31 m) 은 쥐지 않음 (시뮬: 들다 놓침)
TEST(ParkStand, RefusesLargeRoll)
{
  const auto m = kin_data::model();
  LyingGraspCfg gc;
  LyingGrasp g;
  std::string why;
  EXPECT_FALSE(lying_grasp(m, 0.31, 0.0, kTop, 90.0, kHeights, gc, g, why, 30.0));
  EXPECT_FALSE(why.empty());
}

// 옮겨 놓기 (park_relocate): 가까운 자리 = J1 에서 거리, 자리 후보는 J1 → 퍽 선 위 박스 앞쪽 끝부터 가까운 자리 · 박스 안만 (가까운 것이 첫째),
//   첫 후보에 긴축 = 반지름 방향으로 놓인 퍽은 받침 세우기가 됨
TEST(ParkStand, RelocateNearZoneAndSpots)
{
  const auto m = kin_data::model();
  const Eigen::Vector2d j1 = m.j1_xy;
  const Eigen::Vector2d ok(0.225, -0.050), far(0.296, -0.065);   // 실제 로봇에서 된 자리 · 안 된 자리
  const double near_r = (ok - j1).norm() + 0.005;
  EXPECT_TRUE(park_near(j1, ok, near_r));
  EXPECT_FALSE(park_near(j1, far, near_r));
  const auto spots = park_relocate_spots(j1, far, 0.212, 0.075, near_r);
  ASSERT_FALSE(spots.empty());
  for (const auto & s : spots) {
    EXPECT_GE(s.x(), 0.212 - 1e-9);
    EXPECT_LE(std::fabs(s.y()), 0.075 + 1e-9);
    EXPECT_TRUE(park_near(j1, s, near_r));
  }
  const Eigen::Vector2d d = far - j1, s0 = spots.front() - j1;
  EXPECT_NEAR(spots.front().x(), 0.214, 1e-6);              // 박스 앞쪽 끝 (regrasp_min_x + 2 mm)
  EXPECT_NEAR(s0.x() * d.y() - s0.y() * d.x(), 0.0, 1e-9);   // J1 → 퍽 선 위
  for (size_t i = 1; i < spots.size(); ++i) {EXPECT_GE((spots[i] - j1).norm(), s0.norm() - 1e-9);}
  EXPECT_TRUE(park_relocate_spots(j1, far, 0.212, 0.075, 0.1).empty());   // 가까운 자리가 박스 앞보다 안쪽이면 후보 없음
  const double axis = park_relocate_axis_deg(j1, spots.front());
  EXPECT_NEAR(axis, std::atan2(s0.y(), s0.x()) * 180.0 / M_PI, 1e-9);
  LyingGraspCfg gc;
  ParkCfg pk;
  std::vector<ParkNode> pn;
  double elev = 0.0;
  auto accept = [&](const LyingGrasp & c, std::string & w) {return park_nodes(m, pk, c, ticks_to_q(m, c.poses[0].ticks), pn, elev, w);};
  LyingGrasp g;
  std::string why;
  ASSERT_TRUE(lying_grasp(m, spots.front().x(), spots.front().y(), kTop, axis, kHeights, gc, g, why, 30.0, accept)) << why;
  EXPECT_NE(find(pn, "park_re_lift"), nullptr);
}

// 실제 로봇 사례 (park_relook): 먼 누운 퍽을 (0.213, 0.049) 긴축 14° 로 옮겨 놓았다고 보고 그 자리 끝 30 mm 를 쥠 → 빈손.
//   다시 보니 15 mm 옮겨지고 20° 돌아 놓였다면: 계획한 자리 쥐기 (보지 않고) 는 실제 퍽의 쥘 끝에서 10 mm 넘게 어긋남.
//   다시 본 자리 · 긴축으로 다시 푼 쥐기 (control_master lying_replan → park_plan 과 같은 lying_grasp + park_nodes) 는 그 퍽 끝 30 mm 를 쥐고 받침 자세까지 됨
TEST(ParkStand, RelookShiftedRotatedPuckReplansGraspAtRealEnd)
{
  const auto m = kin_data::model();
  LyingGraspCfg gc;
  gc.grip_offset = 0.030;   // 젯슨 park_grasp_offset_mm
  ParkCfg pk;
  const Eigen::Vector2d planned(0.213, 0.049);
  const double ax_plan = park_relocate_axis_deg(m.j1_xy, planned);
  const Eigen::Vector2d real = planned + 0.015 * Eigen::Vector2d(std::cos(0.6), std::sin(0.6));
  const double ax_real = ax_plan + 20.0;
  auto plan_at = [&](const Eigen::Vector2d & c, double ax, LyingGrasp & g, std::vector<ParkNode> & pn, std::string & why) {
      double elev = 0.0;
      auto accept = [&](const LyingGrasp & cand, std::string & w) {return park_nodes(m, pk, cand, ticks_to_q(m, cand.poses[0].ticks), pn, elev, w);};
      return lying_grasp(m, c.x(), c.y(), kTop, ax, kHeights, gc, g, why, 30.0, accept);
    };
  LyingGrasp blind, seen;
  std::vector<ParkNode> pn_blind, pn_seen;
  std::string why;
  ASSERT_TRUE(plan_at(planned, ax_plan, blind, pn_blind, why)) << why;
  ASSERT_TRUE(plan_at(real, ax_real, seen, pn_seen, why)) << why;
  // 다시 푼 쥐기: 실제 퍽 긴축 위 끝 쪽 (중심에서 30 mm + 기울인 손이면 몇 mm 더), 긴축에서 옆으로 안 벗어남
  const Eigen::Vector2d u(std::cos(ax_real * M_PI / 180.0), std::sin(ax_real * M_PI / 180.0));
  const Eigen::Vector2d off = seen.tip.head<2>() - real;
  EXPECT_GT(std::fabs(off.dot(u)), 0.028);
  EXPECT_LT(std::fabs(off.dot(u)), 0.036);
  EXPECT_LT(std::fabs(off.x() * u.y() - off.y() * u.x()), 0.002);
  // 보지 않고 계획한 자리 쥐기: 실제 퍽 기준으로 끝 쪽 13 mm · 옆으로 10 mm — 손가락이 퍽 끝을 반쯤 비켜 내려감
  const Eigen::Vector2d off_b = blind.tip.head<2>() - real;
  EXPECT_GT((blind.tip.head<2>() - seen.tip.head<2>()).norm(), 0.010);
  EXPECT_GT(std::fabs(off_b.x() * u.y() - off_b.y() * u.x()), 0.008);
  EXPECT_LT(std::fabs(off_b.dot(u)), 0.020);
  ASSERT_FALSE(pn_seen.empty());
  EXPECT_NE(find(pn_seen, "park_tilt8"), nullptr);   // 받침 넣기 자세까지 풀림
}

// ---------------------------------------------------------------- 비스듬한 가까운 누운 퍽 (park_relocate_diag_deg)
// 실제 로봇: 가까운 자리 끝 쥐기 — 곧은 퍽 (긴축 0 · 180 ± 10°, −92°) 6/6, 비스듬 (−136° · −153°) 2/2 손가락이 퍽 위로. 옮겨 놓은 퍽 4/4
namespace
{
const Eigen::Vector2d kNearA(0.225, -0.050), kNearB(0.215, 0.030);   // 가까운 자리 (J1 에서 0.24 m 안)
constexpr double kNearR = 0.24, kDiag = 20.0, kMinX = 0.212, kMaxY = 0.075;   // 기본값 (control.yaml · 코드)

// control_master park_relocate_plan 의 diag 자리 거르기와 같음: 놓는 긴축 (반지름 방향) 이 0° · 90° 에서 max_dev 안인 자리만
std::vector<Eigen::Vector2d> diag_spots(const Eigen::Vector2d & j1, const Eigen::Vector2d & p, double max_dev)
{
  std::vector<Eigen::Vector2d> out;
  for (const auto & q : park_relocate_spots(j1, p, kMinX, kMaxY, kNearR)) {
    if (park_axis_diag_deg(park_relocate_axis_deg(j1, q)) <= max_dev) {out.push_back(q);}
  }
  return out;
}

struct Calls {int reloc{0}, restore{0}, direct{0};};
}  // namespace

TEST(ParkDiag, AxisDeviationFromNearestRightAngle)
{
  EXPECT_NEAR(park_axis_diag_deg(0.0), 0.0, 1e-12);
  EXPECT_NEAR(park_axis_diag_deg(180.0), 0.0, 1e-12);
  EXPECT_NEAR(park_axis_diag_deg(-92.0), 2.0, 1e-9);
  EXPECT_NEAR(park_axis_diag_deg(-10.0), 10.0, 1e-9);
  EXPECT_NEAR(park_axis_diag_deg(172.0), 8.0, 1e-9);
  EXPECT_NEAR(park_axis_diag_deg(-136.0), 44.0, 1e-9);
  EXPECT_NEAR(park_axis_diag_deg(-153.0), 27.0, 1e-9);
  EXPECT_NEAR(park_axis_diag_deg(45.0), 45.0, 1e-9);
  EXPECT_NEAR(park_axis_diag_deg(405.0 + 20.0), 25.0, 1e-9);
  EXPECT_TRUE(std::isnan(park_axis_diag_deg(std::nan(""))));
}

// 비스듬한 가까운 퍽 → 옮겨 놓기 (먼 퍽과 같은 길). 놓을 자리 (긴축 = 반지름 방향, 0°/90° 에서 20° 안) 가 있고 거기서 받침 세우기가 됨
TEST(ParkDiag, DiagonalNearPuckRelocates)
{
  const auto m = kin_data::model();
  for (const auto & p : {kNearA, kNearB}) {
    ASSERT_TRUE(park_near(m.j1_xy, p, kNearR));
    for (double ax : {-136.0, -153.0, 44.0, 30.0, -60.0}) {
      SCOPED_TRACE(ax);
      EXPECT_TRUE(park_diag_relocate(ax, kDiag, true, false));
      Calls c;
      const std::vector<int> got = park_diag_or_direct<std::vector<int>>(park_diag_relocate(ax, kDiag, true, false),
        [&]() {++c.reloc; return std::vector<int>{2};}, [&]() {++c.restore;}, [&]() {++c.direct; return std::vector<int>{1};});
      EXPECT_EQ(got, std::vector<int>{2});
      EXPECT_EQ(c.reloc, 1);
      EXPECT_EQ(c.restore, 0);
      EXPECT_EQ(c.direct, 0);
    }
    const auto spots = diag_spots(m.j1_xy, p, kDiag);
    ASSERT_FALSE(spots.empty());
    const double axis = park_relocate_axis_deg(m.j1_xy, spots.front());
    EXPECT_LE(park_axis_diag_deg(axis), kDiag);
    LyingGraspCfg gc;
    gc.grip_offset = 0.030;   // 젯슨 park_grasp_offset_mm
    ParkCfg pk;
    std::vector<ParkNode> pn;
    double elev = 0.0;
    auto accept = [&](const LyingGrasp & cand, std::string & w) {return park_nodes(m, pk, cand, ticks_to_q(m, cand.poses[0].ticks), pn, elev, w);};
    LyingGrasp g;
    std::string why;
    ASSERT_TRUE(lying_grasp(m, spots.front().x(), spots.front().y(), kTop, axis, kHeights, gc, g, why, 30.0, accept)) << why;
    EXPECT_NE(find(pn, "park_re_lift"), nullptr);
  }
  // 거르기가 하는 일: 거르지 않은 후보에는 놓는 긴축이 20° 넘는 자리도 있음 (박스 끝 |y| 0.075 → 반지름 방향 약 21°)
  bool any_diag = false;
  for (const auto & q : park_relocate_spots(m.j1_xy, Eigen::Vector2d(0.214, 0.074), kMinX, kMaxY, kNearR)) {
    any_diag = any_diag || park_axis_diag_deg(park_relocate_axis_deg(m.j1_xy, q)) > kDiag;
  }
  EXPECT_TRUE(any_diag);
}

// 곧은 가까운 퍽 (실제 로봇에서 6/6 된 긴축) → 옮기지 않음, 계획 = 끝 쥐기 그대로 (direct 한 번, 옮기기 · 되돌림 안 부름)
TEST(ParkDiag, StraightNearPuckPlanUnchanged)
{
  for (double ax : {-10.0, -4.0, 0.0, 6.0, 10.0, 170.0, 180.0, -176.0, -92.0, 88.0, 20.0}) {
    SCOPED_TRACE(ax);
    EXPECT_FALSE(park_diag_relocate(ax, kDiag, true, false));
    Calls c;
    const std::vector<int> before{7, 8, 9};
    const auto got = park_diag_or_direct<std::vector<int>>(park_diag_relocate(ax, kDiag, true, false),
      [&]() {++c.reloc; return std::vector<int>{2};}, [&]() {++c.restore;}, [&]() {++c.direct; return before;});
    EXPECT_EQ(got, before);
    EXPECT_EQ(c.reloc, 0);
    EXPECT_EQ(c.restore, 0);
    EXPECT_EQ(c.direct, 1);
  }
}

// park_relocate_diag_deg ≤ 0 (끔) · 숫자 아님 · park_relocate false · 긴축 모름 → 옮기지 않고 끝 쥐기
TEST(ParkDiag, ParamZeroOrRelocateOffUnchanged)
{
  for (double ax : {-136.0, -153.0, 45.0}) {
    EXPECT_FALSE(park_diag_relocate(ax, 0.0, true, false));
    EXPECT_FALSE(park_diag_relocate(ax, -5.0, true, false));
    EXPECT_FALSE(park_diag_relocate(ax, std::nan(""), true, false));
    EXPECT_FALSE(park_diag_relocate(ax, kDiag, false, false));
  }
  EXPECT_FALSE(park_diag_relocate(std::nan(""), kDiag, true, false));
  Calls c;
  const auto got = park_diag_or_direct<std::vector<int>>(park_diag_relocate(-136.0, 0.0, true, false),
    [&]() {++c.reloc; return std::vector<int>{2};}, [&]() {++c.restore;}, [&]() {++c.direct; return std::vector<int>{1};});
  EXPECT_EQ(got, std::vector<int>{1});
  EXPECT_EQ(c.reloc + c.restore, 0);
}

// 옮겨 놓기가 안 됨 (자리 없음 · 계획 안 됨 = 빈 계획) → 계획 상태 되돌림 한 번 뒤 끝 쥐기 (direct)
TEST(ParkDiag, RelocationImpossibleFallsBackToDirect)
{
  const auto m = kin_data::model();
  // 자리 없음: 놓는 긴축 허용이 아주 좁으면 (0.01°) 거른 후보가 빔 → 그때 control_master 는 빈 계획
  EXPECT_TRUE(diag_spots(m.j1_xy, kNearA, 0.01).empty());
  Calls c;
  const std::vector<int> direct{4, 5};
  const auto got = park_diag_or_direct<std::vector<int>>(park_diag_relocate(-136.0, kDiag, true, false),
    [&]() {++c.reloc; return std::vector<int>{};}, [&]() {++c.restore;}, [&]() {++c.direct; return direct;});
  EXPECT_EQ(got, direct);
  EXPECT_EQ(c.reloc, 1);
  EXPECT_EQ(c.restore, 1);
  EXPECT_EQ(c.direct, 1);
  // 끝 쥐기도 안 되면 그 빈 계획 그대로 (부르는 쪽이 "지금 자리에서 세우기 계획 안 됨" 옮겨 놓기로)
  c = Calls{};
  EXPECT_TRUE((park_diag_or_direct<std::vector<int>>(true, [&]() {++c.reloc; return std::vector<int>{};}, [&]() {++c.restore;},
    [&]() {++c.direct; return std::vector<int>{};})).empty());
  EXPECT_EQ(c.direct, 1);
}

// 한 PICK 에 한 번: 옮겨 놓은 뒤 (넘어짐 · 다시 계획) 비스듬해도 다시 안 옮김. 놓는 긴축 자체도 비스듬하지 않아 플래그 없이도 안 걸림
TEST(ParkDiag, NoDoubleRelocation)
{
  const auto m = kin_data::model();
  bool relocated = false;   // control_master park_relocated_ (PICK 마다 지움)
  auto plan_once = [&](double ax, Calls & c) {
      return park_diag_or_direct<std::vector<int>>(park_diag_relocate(ax, kDiag, true, relocated),
        [&]() {++c.reloc; relocated = true; return std::vector<int>{2};}, [&]() {++c.restore;}, [&]() {++c.direct; return std::vector<int>{1};});
    };
  Calls c1, c2, c3;
  EXPECT_EQ(plan_once(-136.0, c1), std::vector<int>{2});
  EXPECT_TRUE(relocated);
  EXPECT_EQ(plan_once(-140.0, c2), std::vector<int>{1});   // 같은 PICK 다시 계획: 비스듬해도 끝 쥐기
  EXPECT_EQ(c2.reloc, 0);
  EXPECT_FALSE(park_diag_relocate(-136.0, kDiag, true, true));
  relocated = false;   // 새 PICK
  EXPECT_EQ(plan_once(-153.0, c3), std::vector<int>{2});
  for (const auto & p : {kNearA, kNearB}) {
    for (const auto & q : diag_spots(m.j1_xy, p, kDiag)) {
      EXPECT_FALSE(park_diag_relocate(park_relocate_axis_deg(m.j1_xy, q), kDiag, true, false));
    }
  }
}

// ---------------------------------------------------------------- 기울여 쥔 선 퍽 → 받침 (park_upright_nodes)
namespace
{
const double kTopUp = 0.037 + 0.060 - 0.039;   // 박스 위 선 퍽 윗면 (모델)

// control_master 선 퍽 PICK 과 같은 쥐기: 10° 까지 50 mm, 안 되면 20–40° (20 에 가까운 것부터) 40 mm
bool tilted_pick(const ArmModel & m, double x, double y, bool square, std::vector<PickPose> & P)
{
  const double tang = std::atan2(y - m.j1_xy.y(), x - m.j1_xy.x()) * 180.0 / M_PI + 90.0;
  const double yaw = square ? tang : std::nan("");
  std::string why;
  P = pick_poses(m, x, y, kTopUp, yaw, {40.0, 10.0, -50.0}, why, std::nan(""), 10.0);
  if (P.size() != 3) {P = pick_poses(m, x, y, kTopUp, yaw, {40.0, 10.0, -40.0}, why, std::nan(""), 40.0, 20.0, 20.0);}
  return P.size() == 3;
}

UprightHeld held_of(const ArmModel & m, const std::vector<PickPose> & P, double x, double y, bool square)
{
  UprightHeld h;
  h.q_grasp = ticks_to_q(m, P.back().ticks);
  h.puck_xy = Eigen::Vector2d(x, y);
  h.bottom_z = kTopUp - 0.060;
  h.square = square;
  h.width = square ? 0.0348 : 0.0335;
  return h;
}
}  // namespace

// 먼 줄 · 옆 끝 (손 10 · 20 · 30°): 받침 위로 같은 기울임 (퍽 수직) → 곧게 넣고 벽 위에서 놓음 → 연 손 위로 → 수직 손 30 mm 다시 쥐기
//   (40° 는 받침 안 씀 → TiltedPick40GoesToBoxFlow)
TEST(ParkUpright, TiltedPicksReachStandUpright)
{
  const auto m = kin_data::model();
  ParkCfg pk;
  pk.regrasp_r = 0.0;
  UprightCfg u;
  struct C {double x, y, tilt;};
  for (const C cs : {C{0.25, -0.03, 10.0}, C{0.24, 0.09, 10.0}, C{0.28, 0.09, 20.0}, C{0.33, -0.09, 30.0}, C{0.31, 0.0, 30.0}}) {
    for (bool square : {true, false}) {
      std::vector<PickPose> P;
      ASSERT_TRUE(tilted_pick(m, cs.x, cs.y, square, P)) << cs.x << " " << cs.y;
      ASSERT_NEAR(P.back().tilt_deg, cs.tilt, 1e-9) << cs.x << " " << cs.y;
      const UprightHeld h = held_of(m, P, cs.x, cs.y, square);
      std::vector<ParkNode> pn;
      UprightInfo info;
      std::string why;
      ASSERT_TRUE(park_upright_nodes(m, pk, u, h, ticks_to_q(m, P[0].ticks), pn, info, why)) << cs.x << " " << cs.y << ": " << why;
      EXPECT_NEAR(info.tilt_deg, cs.tilt, 1.0);
      EXPECT_NEAR(info.depth_mm, cs.tilt <= 10.0 ? 50.0 : 40.0, 1.0);
      const ParkNode * rel = find(pn, "pk_down4");
      ASSERT_NE(rel, nullptr);
      EXPECT_EQ(rel->grip, "open");
      EXPECT_EQ(find(pn, "park_re_down")->grip, "close");
      EXPECT_EQ(find(pn, "park_re_above")->grip, "");   // 이미 열림
      EXPECT_EQ(pn.back().name, "park_re_lift");
      EXPECT_GT(pn.back().held_kg, 0.0);
      // 놓는 점: 손 = 쥔 손을 수직축 둘레로만 돌림 → 퍽 축 수직, 퍽 축이 받침 중심, 바닥 = 받침 바닥 + b (벽 안)
      const Grip gp = arm_grip(m, h.q_grasp), gs = arm_grip(m, rel->q);
      EXPECT_NEAR(gs.app.normalized().z(), gp.app.normalized().z(), 1e-3);
      EXPECT_NEAR(gs.close.normalized().z(), gp.close.normalized().z(), 2e-2);
      EXPECT_NEAR(gs.tip.x(), pk.center.x(), 1e-3);
      EXPECT_NEAR(gs.tip.y(), pk.center.y(), 1e-3);
      EXPECT_NEAR(gs.tip.z() - (gp.tip.z() - h.bottom_z), pk.floor_z + info.b_mm / 1000.0, 1e-3);
      EXPECT_LT(info.b_mm, u.wall_h * 1000.0);   // 퍽 바닥이 벽 안에서 놓임 (벽 윗면 아래)
      // 쥔 손가락 가장 낮은 곳 (손끝 면, 폭 끝) 이 벽 윗면 + 여유 위
      const Eigen::Vector3d b = gs.app.normalized().cross(gs.close.normalized()).normalized();
      const double low = gs.tip.z() - u.finger_half_w * std::fabs(b.z());
      EXPECT_GE(low, pk.floor_z + u.wall_h + u.finger_clear - 2e-4);
      // 다시 쥐기 = 받침 중심 수직 손 30 mm (park_nodes 와 같은 자리)
      const Grip gd = arm_grip(m, find(pn, "park_re_down")->q);
      EXPECT_NEAR(gd.tip.x(), pk.center.x(), 1e-3);
      EXPECT_NEAR(gd.tip.z(), pk.floor_z + pk.length - pk.regrasp_depth, 1e-3);
      EXPECT_LT(gd.app.normalized().z(), -0.9998);
      for (const auto & n : pn) {
        EXPECT_TRUE(ticks_in_range(m, q_to_ticks(m, n.q))) << n.name;
        EXPECT_TRUE(ticks_in_range(m, q_to_ticks(m, encoder_goal_q(m, n.q, n.held_kg)))) << n.name;
      }
      // 곧게 넣기: 위 점 → 놓는 점 관절 한 점 차 10° 아래 (갈래 안 바뀜)
      const ParkNode * ov = find(pn, "pk_over");
      ASSERT_NE(ov, nullptr);
      for (size_t i = static_cast<size_t>(ov - pn.data()); i + 1 <= static_cast<size_t>(rel - pn.data()); ++i) {
        for (int j = 0; j < 5; ++j) {EXPECT_LT(std::fabs(pn[i + 1].q[j] - pn[i].q[j]) * 180.0 / M_PI, 10.0) << pn[i + 1].name;}
      }
      // 연 손가락이 선 퍽 윗면 위로 올라간 뒤 수직으로
      const Grip gu = arm_grip(m, find(pn, "pk_up")->q);
      EXPECT_GT(gu.tip.z(), gs.tip.z() + 0.02);
    }
  }
}

// 손 40° (실제 로봇: 먼 원기둥 (0.338, −0.081) 40° 받침 흐름 approach 실패, 놓는 자세 J2 가 tick_lo 끝) → false (박스 위 다시 쥐기 흐름).
//   같은 자리 · 같은 손 30° 는 그대로 받침 (한도 = 쥔 손 기울임만)
TEST(ParkUpright, TiltedPick40GoesToBoxFlow)
{
  const auto m = kin_data::model();
  ParkCfg pk;
  UprightCfg u;
  for (const Eigen::Vector2d & c : {Eigen::Vector2d(0.34, 0.06), Eigen::Vector2d(0.338, -0.081)}) {
    for (bool square : {true, false}) {
      std::vector<PickPose> P;
      ASSERT_TRUE(tilted_pick(m, c.x(), c.y(), square, P)) << c.transpose();
      ASSERT_NEAR(P.back().tilt_deg, 40.0, 1e-9) << c.transpose();
      const UprightHeld h = held_of(m, P, c.x(), c.y(), square);
      std::vector<ParkNode> pn;
      UprightInfo info;
      std::string why;
      EXPECT_FALSE(park_upright_nodes(m, pk, u, h, ticks_to_q(m, P[0].ticks), pn, info, why)) << c.transpose();
      EXPECT_TRUE(pn.empty());
      EXPECT_NEAR(info.tilt_deg, 40.0, 1.0);
      EXPECT_NE(why.find("30° 까지"), std::string::npos) << why;
      // 한도를 40 으로 올리면 같은 쥐기가 받침으로 감 (막는 것은 기울임 한도뿐)
      UprightCfg u40 = u;
      u40.max_tilt_deg = 40.0;
      EXPECT_TRUE(park_upright_nodes(m, pk, u40, h, ticks_to_q(m, P[0].ticks), pn, info, why)) << c.transpose() << ": " << why;
    }
  }
  // 30° 경계: 손 30° 쥐기는 받침 (0.33, −0.09)
  std::vector<PickPose> P;
  ASSERT_TRUE(tilted_pick(m, 0.33, -0.09, false, P));
  ASSERT_NEAR(P.back().tilt_deg, 30.0, 1e-9);
  std::vector<ParkNode> pn;
  UprightInfo info;
  std::string why;
  EXPECT_TRUE(park_upright_nodes(m, pk, u, held_of(m, P, 0.33, -0.09, false), ticks_to_q(m, P[0].ticks), pn, info, why)) << why;
}

// 네모 면이 받침 벽과 많이 틀어진 쥐기 (손 10° 는 닫힘축을 면에 맞춤): 받침에 안 들어감 → false (control_master 는 박스 위 다시 쥐기 흐름). 원기둥은 됨
TEST(ParkUpright, SquareFaceMisalignedFallsBack)
{
  const auto m = kin_data::model();
  ParkCfg pk;
  UprightCfg u;
  const double x = 0.25, y = -0.03;
  const double tang = std::atan2(y - m.j1_xy.y(), x - m.j1_xy.x()) * 180.0 / M_PI + 90.0;
  std::string why;
  auto P = pick_poses(m, x, y, kTopUp, tang + 25.0, {40.0, 10.0, -50.0}, why, std::nan(""), 10.0);
  ASSERT_EQ(P.size(), 3u) << why;
  ASSERT_NEAR(P.back().tilt_deg, 10.0, 1e-9);
  UprightHeld h = held_of(m, P, x, y, true);
  std::vector<ParkNode> pn;
  UprightInfo info;
  EXPECT_FALSE(park_upright_nodes(m, pk, u, h, ticks_to_q(m, P[0].ticks), pn, info, why));
  EXPECT_NEAR(info.face_deg, 25.0, 2.0);
  EXPECT_NE(why.find("틀어짐"), std::string::npos) << why;
  h.square = false;
  h.width = 0.0335;
  EXPECT_TRUE(park_upright_nodes(m, pk, u, h, ticks_to_q(m, P[0].ticks), pn, info, why)) << why;
}

// 손끝이 퍽 축에서 떨어진 쥐기 (표 쥐기): 닫힘 방향 어긋남은 손가락이 모아 0, 손가락 폭 방향만 남겨 받침 중심에 퍽을 놓음
TEST(ParkUpright, OffAxisGraspKeepsOnlyFingerWidthOffset)
{
  const auto m = kin_data::model();
  ParkCfg pk;
  UprightCfg u;
  std::vector<PickPose> P;
  ASSERT_TRUE(tilted_pick(m, 0.33, -0.09, false, P));
  const Grip g = arm_grip(m, ticks_to_q(m, P.back().ticks));
  const Eigen::Vector3d cl = Eigen::Vector3d(g.close.x(), g.close.y(), 0.0).normalized();
  const Eigen::Vector3d bh = Eigen::Vector3d::UnitZ().cross(cl);
  for (const Eigen::Vector3d & d : {Eigen::Vector3d(0.004 * cl), Eigen::Vector3d(0.004 * bh)}) {
    UprightHeld h = held_of(m, P, 0.33 - d.x(), -0.09 - d.y(), false);   // 퍽 중심 = 손끝 − d
    std::vector<ParkNode> pn;
    UprightInfo info;
    std::string why;
    ASSERT_TRUE(park_upright_nodes(m, pk, u, h, ticks_to_q(m, P[0].ticks), pn, info, why)) << why;
    const Grip gs = arm_grip(m, find(pn, "pk_down4")->q);
    // 닫힘 방향 어긋남 → 손끝이 받침 중심 (손가락이 퍽을 모음), 폭 방향 어긋남 → 손끝이 중심에서 그만큼 (퍽 중심이 받침 중심)
    const double want = std::fabs(d.dot(cl)) > 1e-6 ? 0.0 : 0.004;
    EXPECT_NEAR((gs.tip.head<2>() - pk.center).norm(), want, 1e-3) << d.transpose();
  }
}

// 받침 안 다시 쥐기 빈손 뒤 자세: park_re_lift (빈손으로 멈추는 위 점) 는 손끝이 받침 벽 위 35 mm 넘게 (받침 밖),
//   HOME (접은 자세, 틱 4084 1102 2668 1971 2063) 은 손끝이 받침 안 바닥 위 20 mm 아래 → 받침에 퍽이 남았으면 접는 손이 그 퍽에 닿음.
//   그래서 받침에 퍽이 없음이 확실할 때 (놓기 전 이미 빈손) 만 받침 위에서 접고, 아니면 접지 않음 (control_master)
TEST(ParkUpright, EmptyRegripPoseIsOutOfStandHomeIsInside)
{
  const auto m = kin_data::model();
  ParkCfg pk;
  UprightCfg u;
  std::vector<PickPose> P;
  ASSERT_TRUE(tilted_pick(m, 0.33, -0.09, false, P));
  std::vector<ParkNode> pn;
  UprightInfo info;
  std::string why;
  ASSERT_TRUE(park_upright_nodes(m, pk, u, held_of(m, P, 0.33, -0.09, false), ticks_to_q(m, P[0].ticks), pn, info, why)) << why;
  const Grip gl = arm_grip(m, pn.back().q);
  ASSERT_EQ(pn.back().name, "park_re_lift");
  EXPECT_GT(gl.tip.z() - (pk.floor_z + u.wall_h), 0.035);
  EXPECT_GT(gl.tip.z() - (pk.floor_z + u.puck_h), 0.005);   // 선 퍽 윗면 위
  const Grip gh = arm_grip(m, ticks_to_q(m, Ticks{4084, 1102, 2668, 1971, 2063}));
  EXPECT_LT(gh.tip.z() - pk.floor_z, 0.020);
  EXPECT_LT(std::fabs(gh.tip.x() - pk.center.x()), u.cup_in_r);
  EXPECT_LT(std::fabs(gh.tip.y() - pk.center.y()), u.cup_in_t);
}

// ---------------------------------------------------------------- 받침에 놓은 원기둥 다시 보기 (park_look_pose · park_regrasp_at)
namespace
{
Eigen::Isometry3d urdf_T5c_park()   // test_look_at urdf_T5c 와 같음 (URDF camera_joint)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = (Eigen::AngleAxisd(-1.995706, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(-0.048951, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(-2.969292, Eigen::Vector3d::UnitX())).toRotationMatrix();
  T.translation() = Eigen::Vector3d(0.085672, 0.001345, 0.004098);
  return T;
}
struct ParkCase {double x, y;};
// 실제 로봇 로그의 원기둥 받침 자리 + 시험 자리: 손 6–10° · 20° · 30°
const ParkCase kParkCases[] = {{0.248, 0.065}, {0.256, 0.052}, {0.259, 0.024}, {0.259, -0.023}, {0.25, -0.03}, {0.24, 0.09},
  {0.300, 0.050}, {0.28, 0.09}, {0.33, -0.09}, {0.31, 0.0}};
}  // namespace

// 로그 자리 전부: 관찰 자세가 풀리고 (손 기울임 · 깊이 · 화면 · 틱 · 몸 · 길), 박스 다시 보기 값 (윗면 위 110 mm · 손 20°) 은 받침에서 IK 안 됨
TEST(ParkLook, ObservePoseOverLoggedCylinderSpots)
{
  const auto m = kin_data::model();
  const auto T5c = urdf_T5c_park();
  ParkCfg pk;
  pk.regrasp_r = 0.0;   // 로봇 값
  UprightCfg u;
  const ParkLookCfg L;
  int n = 0;
  std::optional<Q5> hint;
  for (const auto & cs : kParkCases) {
    std::vector<PickPose> P;
    ASSERT_TRUE(tilted_pick(m, cs.x, cs.y, false, P)) << cs.x << " " << cs.y;
    std::vector<ParkNode> pn;
    UprightInfo info;
    std::string why;
    ASSERT_TRUE(park_upright_nodes(m, pk, u, held_of(m, P, cs.x, cs.y, false), ticks_to_q(m, P[0].ticks), pn, info, why)) << why;
    const ParkNode * up = find(pn, "pk_up"), * ra = find(pn, "park_re_above");
    ASSERT_NE(up, nullptr);
    ASSERT_NE(ra, nullptr);
    Q5 qo{};
    ParkLookInfo li;
    ASSERT_TRUE(park_look_pose(m, T5c, pk, u, L, up->q, ra->q, qo, li, why, hint ? &*hint : nullptr)) << cs.x << " " << cs.y << ": " << why;
    if (hint) {EXPECT_TRUE(li.from_hint);}
    hint = qo;
    ++n;
    const Ticks t = q_to_ticks(m, qo);
    EXPECT_TRUE(ticks_in_range(m, t));
    EXPECT_TRUE(ticks_in_range(m, q_to_ticks(m, encoder_goal_q(m, qo, 0.0))));
    EXPECT_LE(t[3], 2430);   // J4 CAD 한계 (pose_safety)
    EXPECT_GE(li.dmin, L.min_depth - 1e-9);
    EXPECT_GE(li.margin_px, L.min_margin_px - 1e-9);
    EXPECT_LT(li.u1, L.width);
    EXPECT_LT(li.v1, L.height);
    EXPECT_TRUE(li.u1 < L.finger_u || li.v1 < L.finger_v);
    EXPECT_GT(li.tip_h, 0.05);   // 손이 퍽 위로 멀리 (열린 손가락이 퍽에 안 닿음)
    EXPECT_TRUE(li.lifted);
    EXPECT_GE(li.clear_obs, L.path_clear - 1e-9);
    for (const auto & p : look_arm_points(m, T5c, qo)) {
      if (p.x() < L.rack_x && std::fabs(p.y()) < L.half_y) {EXPECT_GE(p.z(), L.rack_z);}   // 적재함 · 칸 퍽 위
      const double ox = std::max({L.mast_box[0] - p.x(), p.x() - L.mast_box[1], 0.0}), oy = std::max({L.mast_box[2] - p.y(), p.y() - L.mast_box[3], 0.0});
      EXPECT_GE(std::hypot(ox, oy), L.mast_pad);   // 광각 기둥 밖
    }
    std::printf("(%.3f, %+.3f) 손 %2.0f° b %.1f: 관찰 틱 [%d %d %d %d %d] 손 %.0f° 손끝 윗면 위 %.0f mm · 깊이 %.3f (가장 가까운 %.3f) · u %.0f–%.0f v %.0f–%.0f "
      "여유 %.0f px · 길 여유 %.1f/%.1f/%.1f mm%s\n", cs.x, cs.y, info.tilt_deg, info.b_mm, t[0], t[1], t[2], t[3], t[4], li.tilt_deg, li.tip_h * 1000.0, li.depth,
      li.dmin, li.u0, li.u1, li.v0, li.v1, li.margin_px, li.clear_up * 1000.0, li.clear_obs * 1000.0, li.clear_down * 1000.0, li.from_hint ? " (기억)" : "");
  }
  EXPECT_EQ(n, 10);
  // 박스 다시 보기 값을 받침에 그대로: 손끝 받침 가운데 위 110 mm · 손 20° (바깥 · 안쪽) · 수직 손 60 mm 넘게 → IK 없음
  const Eigen::Vector2d rd = (pk.center - m.j1_xy).normalized(), td(-rd.y(), rd.x());
  const double ptop = pk.floor_z + u.puck_h;
  int solved = 0;
  for (double tilt : {-20.0, 0.0, 20.0}) {
    for (double h : {0.110, 0.080}) {
      const double a = tilt * M_PI / 180.0;
      const Eigen::Vector3d app = std::sin(a) * Eigen::Vector3d(rd.x(), rd.y(), 0.0) - std::cos(a) * Eigen::Vector3d::UnitZ();
      for (const auto & s : arm_ik(m, Eigen::Vector3d(pk.center.x(), pk.center.y(), ptop + h), app, Eigen::Vector3d(td.x(), td.y(), 0.0), {}, 180.0)) {
        if (s.pos_mm < 1.0 && s.app_deg < 1.0 && s.roll_deg < 2.0 && ticks_in_range(m, q_to_ticks(m, s.q))) {++solved;}
      }
    }
  }
  EXPECT_EQ(solved, 0);
}

// 다시 쥘 수 있는 잰 중심 범위: 받침 가운데 ±7 mm (반지름 · 접선 · 대각) 모두 수직 손 30 mm (높이 그대로) 가 풀리고, 관찰 점 → 새 park_re_above 길도 비어 있음
TEST(ParkLook, RegraspAtMeasuredCenterWithin7mm)
{
  const auto m = kin_data::model();
  const auto T5c = urdf_T5c_park();
  ParkCfg pk;
  pk.regrasp_r = 0.0;
  UprightCfg u;
  const ParkLookCfg L;
  std::vector<PickPose> P;
  ASSERT_TRUE(tilted_pick(m, 0.256, 0.052, false, P));
  std::vector<ParkNode> pn;
  UprightInfo info;
  std::string why;
  ASSERT_TRUE(park_upright_nodes(m, pk, u, held_of(m, P, 0.256, 0.052, false), ticks_to_q(m, P[0].ticks), pn, info, why)) << why;
  const ParkNode * up = find(pn, "pk_up"), * ra = find(pn, "park_re_above"), * rd = find(pn, "park_re_down");
  Q5 qo{};
  ParkLookInfo li;
  ASSERT_TRUE(park_look_pose(m, T5c, pk, u, L, up->q, ra->q, qo, li, why)) << why;
  const Eigen::Vector3d close = arm_grip(m, rd->q).close;
  const Eigen::Vector2d R = (pk.center - m.j1_xy).normalized(), T(-R.y(), R.x());
  const double ptop = pk.floor_z + u.puck_h;
  int n = 0;
  double worst = 1e9;
  for (double a = -7.0; a <= 7.0 + 1e-9; a += 3.5) {
    for (double b = -7.0; b <= 7.0 + 1e-9; b += 3.5) {
      const Eigen::Vector2d xy = pk.center + (a * R + b * T) / 1000.0;
      Q5 qa{}, qd{};
      ASSERT_TRUE(park_regrasp_at(m, pk, u, xy, close, ra->q, qa, qd, why)) << a << " " << b << ": " << why;
      const Grip ga = arm_grip(m, qa), gd = arm_grip(m, qd);
      EXPECT_NEAR((gd.tip.head<2>() - xy).norm(), 0.0, 1e-3);
      EXPECT_NEAR(gd.tip.z(), ptop - pk.regrasp_depth, 1e-3);   // 깊이 그대로 (30 mm)
      EXPECT_NEAR(ga.tip.z(), ptop + 0.0075, 1e-3);
      EXPECT_LT(gd.app.normalized().z(), -0.9998);
      EXPECT_GT(gd.close.normalized().dot(close.normalized()), 0.999);   // 닫힘 그대로
      EXPECT_TRUE(ticks_in_range(m, q_to_ticks(m, encoder_goal_q(m, qa, pk.held_kg))));
      for (int j = 0; j < 5; ++j) {EXPECT_LT(std::fabs(qa[j] - ra->q[j]), 0.2) << j;}   // 원래 해 갈래 그대로
      worst = std::min(worst, park_look_path_clear(m, pk, u, L, qo, qa));
      ++n;
    }
  }
  EXPECT_EQ(n, 25);
  EXPECT_GT(worst, 0.002);   // 관찰 점 → 옮긴 park_re_above (끝점 손끝 윗면 위 7.5 mm, 퍽 기울기 여유 5 mm 뺀 2.5 mm)
  std::printf("잰 중심 ±7 mm 25 곳: 수직 30 mm 다시 쥐기 모두 풀림 · 관찰 점 → park_re_above 손가락 여유 최소 %.1f mm\n", worst * 1000.0);
}

// 카메라 TF 오차 (카메라 받침대: 약 18 mm 높고 3.35° 돎, 실제 로봇은 TF 로 다시 풂): 고른 자세를 TF 만 흔들어 보면 퍽이 화면 · 깊이 안에 남는가 (숫자 기록)
TEST(ParkLook, CameraTfErrorMarginReport)
{
  const auto m = kin_data::model();
  const auto T5c = urdf_T5c_park();
  ParkCfg pk;
  UprightCfg u;
  ParkLookCfg L;
  std::vector<PickPose> P;
  ASSERT_TRUE(tilted_pick(m, 0.259, 0.024, false, P));
  std::vector<ParkNode> pn;
  UprightInfo info;
  std::string why;
  ASSERT_TRUE(park_upright_nodes(m, pk, u, held_of(m, P, 0.259, 0.024, false), ticks_to_q(m, P[0].ticks), pn, info, why)) << why;
  Q5 qo{};
  ParkLookInfo li;
  ASSERT_TRUE(park_look_pose(m, T5c, pk, u, L, find(pn, "pk_up")->q, find(pn, "park_re_above")->q, qo, li, why)) << why;
  ParkLookCfg loose = L;
  loose.min_margin_px = 0.0;
  loose.min_depth = 0.0;
  int seen = 0, total = 0;
  for (int ax = 0; ax < 3; ++ax) {
    for (double deg : {-3.35, 3.35}) {
      Eigen::Isometry3d T = T5c;
      T.linear() = T5c.linear() * Eigen::AngleAxisd(deg * M_PI / 180.0, Eigen::Vector3d::Unit(ax)).toRotationMatrix();
      ParkLookInfo vi;
      park_look_view(m, T, pk, u, loose, qo, vi);
      ++total;
      const bool ok = vi.margin_px > 0.0 && vi.dmin > 0.175;
      seen += ok ? 1 : 0;
      std::printf("카메라 %c 축 %+.2f°: 깊이 %.3f (가장 가까운 %.3f) · 여유 %.0f px%s\n", "xyz"[ax], deg, vi.depth, vi.dmin, vi.margin_px, ok ? "" : " ← 안 보임");
    }
    for (double mm : {-18.0, 18.0}) {
      Eigen::Isometry3d T = T5c;
      T.translation() += T5c.linear() * (Eigen::Vector3d::Unit(ax) * mm / 1000.0);
      ParkLookInfo vi;
      park_look_view(m, T, pk, u, loose, qo, vi);
      ++total;
      const bool ok = vi.margin_px > 0.0 && vi.dmin > 0.175;
      seen += ok ? 1 : 0;
      std::printf("카메라 %c 축 %+.0f mm: 깊이 %.3f (가장 가까운 %.3f) · 여유 %.0f px%s\n", "xyz"[ax], mm, vi.depth, vi.dmin, vi.margin_px, ok ? "" : " ← 안 보임");
    }
  }
  std::printf("TF 흔들기 %d 중 %d 에서 퍽 윗면 (±7 mm 포함) 이 화면 안 · 깊이 0.175 넘음 (안 보이면 실물은 시간 초과 → 받침 가운데 쥐기)\n", total, seen);
  EXPECT_GE(seen, total / 2);
}

// 원기둥만 끼움: 네모 · 꺼짐 = 받침 경로 그대로, 원기둥 = pk_up 뒤 park_look_up · 관찰 점 (멈춤은 control_master 가 붙임)
// 올린 점에서 보기 (park_lift_look). 로그 자리 전부 올림이 되고 (40 mm), 길 여유가 park_look_pose 기준 이상, 받침 가운데가 화면 안,
//   끼우면 pk_up 바로 뒤에 관찰 점 하나 (park_look_up 없음) → 그 다음 park_re_above. 손 72° 격자 자세보다 관절 이동이 짧음
TEST(ParkLook, LiftLookOverLoggedCylinderSpots)
{
  const auto m = kin_data::model();
  const auto T5c = urdf_T5c_park();
  ParkCfg pk;
  pk.regrasp_r = 0.0;
  UprightCfg u;
  const ParkLookCfg L;
  int n = 0;
  for (const auto & cs : kParkCases) {
    std::vector<PickPose> P;
    ASSERT_TRUE(tilted_pick(m, cs.x, cs.y, false, P));
    std::vector<ParkNode> pn;
    UprightInfo info;
    std::string why;
    ASSERT_TRUE(park_upright_nodes(m, pk, u, held_of(m, P, cs.x, cs.y, false), ticks_to_q(m, P[0].ticks), pn, info, why)) << why;
    const ParkNode * up = find(pn, "pk_up"), * ra = find(pn, "park_re_above");
    Q5 qo{};
    ParkLookInfo li;
    ASSERT_TRUE(park_lift_look(m, T5c, pk, u, L, up->q, ra->q, qo, li, why)) << cs.x << " " << cs.y << ": " << why;
    EXPECT_TRUE(li.at_lift);
    EXPECT_FALSE(li.lifted);
    EXPECT_EQ(qo, li.q_lift);
    EXPECT_GE(li.lift_m, 0.010 - 1e-9);   // 손 30° 40 · 20° 30 · 10° 10–20 mm (J4 한계, park_look_pose 0 단계와 같음). 실물 창 3 (손 10°) 도 이 근처에서 태그 읽음
    const Grip g0 = arm_grip(m, up->q), g1 = arm_grip(m, qo);
    EXPECT_NEAR((g1.tip - g0.tip - li.lift_m * Eigen::Vector3d::UnitZ()).norm(), 0.0, 1e-3);   // 같은 손 자세로 곧게
    EXPECT_GT(g1.app.normalized().dot(g0.app.normalized()), 0.9998);
    EXPECT_TRUE(ticks_in_range(m, q_to_ticks(m, qo)));
    EXPECT_GE(li.clear_down, std::min(L.path_clear, park_look_path_clear(m, pk, u, L, ra->q, ra->q) - 0.0005) - 1e-9);
    Q5 qg{};
    ParkLookInfo lg;
    ASSERT_TRUE(park_look_pose(m, T5c, pk, u, L, up->q, ra->q, qg, lg, why)) << why;
    double d_lift = 0.0, d_grid = 0.0;
    for (int j = 0; j < 5; ++j) {d_lift = std::max(d_lift, std::fabs(ra->q[j] - qo[j])); d_grid = std::max(d_grid, std::fabs(qg[j] - lg.q_lift[j]) + std::fabs(ra->q[j] - qg[j]));}
    const auto t = q_to_ticks(m, qo);
    std::printf("(%.3f, %+.3f) 손 %2.0f° 올림 %.0f mm: 올린 점 틱 [%d %d %d %d %d] 손끝 윗면 위 %.0f mm · 깊이 %.3f (가장 가까운 %.3f) · u %.0f–%.0f v %.0f–%.0f · 길 여유 %.1f/%.1f mm · "
      "관절 최대 이동 %.0f° (격자 관찰 자세 %.0f°)\n", cs.x, cs.y, info.tilt_deg, li.lift_m * 1000.0, t[0], t[1], t[2], t[3], t[4], li.tip_h * 1000.0, li.depth, li.dmin, li.u0, li.u1, li.v0, li.v1,
      li.clear_up * 1000.0, li.clear_down * 1000.0, d_lift * 180.0 / M_PI, d_grid * 180.0 / M_PI);
    auto pl = pn;
    ASSERT_TRUE(park_insert_look(pl, false, true, qo, li, "re_observe"));
    ASSERT_EQ(pl.size(), pn.size() + 1);
    const auto it = std::find_if(pl.begin(), pl.end(), [](const ParkNode & x) {return x.name == "pk_up";});
    EXPECT_EQ((it + 1)->name, "re_observe");
    EXPECT_EQ((it + 1)->q, qo);
    EXPECT_EQ((it + 2)->name, "park_re_above");
    ++n;
  }
  EXPECT_EQ(n, 10);
}

TEST(ParkLook, InsertOnlyForCylinder)
{
  const auto m = kin_data::model();
  ParkCfg pk;
  UprightCfg u;
  std::vector<PickPose> P;
  ASSERT_TRUE(tilted_pick(m, 0.25, -0.03, false, P));
  std::vector<ParkNode> pn;
  UprightInfo info;
  std::string why;
  ASSERT_TRUE(park_upright_nodes(m, pk, u, held_of(m, P, 0.25, -0.03, false), ticks_to_q(m, P[0].ticks), pn, info, why)) << why;
  ParkLookInfo li;
  li.lifted = true;
  li.q_lift = find(pn, "pk_up")->q;
  const Q5 qo{-1.5, -1.6, -0.3, 0.79, 0.3};
  auto names = [](const std::vector<ParkNode> & v) {std::string s; for (const auto & n : v) {s += n.name + " ";} return s;};
  const std::string before = names(pn);
  auto sq = pn;
  EXPECT_FALSE(park_insert_look(sq, true, true, qo, li, "re_observe"));
  EXPECT_EQ(names(sq), before);
  auto off = pn;
  EXPECT_FALSE(park_insert_look(off, false, false, qo, li, "re_observe"));
  EXPECT_EQ(names(off), before);
  auto cyl = pn;
  EXPECT_TRUE(park_insert_look(cyl, false, true, qo, li, "re_observe"));
  ASSERT_EQ(cyl.size(), pn.size() + 2);
  const auto it = std::find_if(cyl.begin(), cyl.end(), [](const ParkNode & n) {return n.name == "pk_up";});
  ASSERT_NE(it, cyl.end());
  EXPECT_EQ((it + 1)->name, "park_look_up");
  EXPECT_EQ((it + 2)->name, "re_observe");
  EXPECT_EQ((it + 3)->name, "park_re_above");
  EXPECT_EQ((it + 2)->q, qo);
  EXPECT_EQ((it + 2)->held_kg, 0.0);
  EXPECT_EQ((it + 2)->grip, "");   // 손은 이미 열림 (pk_down4) — 그리퍼 동작 안 바꿈
  li.lifted = false;
  auto nl = pn;
  EXPECT_TRUE(park_insert_look(nl, false, true, qo, li, "re_observe"));
  EXPECT_EQ(nl.size(), pn.size() + 1);
}

// 받침 놓는 높이 b 를 2 mm 마다 먼저 보고 0.5 mm 로 좁혀도 0.5 mm 씩 70 번 다 보는 것과 같은 b · 같은 놓는 자세 (pk_down4 틱).
//   기대값 = 0.5 mm 씩 다 본 결과 (918 경우를 견주어 모두 같음)
TEST(ParkUpright, CoarseToFineKeepsLowestB)
{
  const auto m = kin_data::model();
  ParkCfg pk;
  pk.regrasp_r = 0.0;
  UprightCfg u;
  // 표 쥐기 흉내: 손끝 퍽 축 · 윗면 아래 30 mm · 바깥 t° · 닫힘 = 접선을 손 축 둘레로 rot° (첫 IK 해) → 70 mm 위 들기
  auto table_grasp = [&](double x, double y, double t, double rot, Q5 & qg, Q5 & qf) {
      const Eigen::Vector2d R = (Eigen::Vector2d(x, y) - m.j1_xy).normalized();
      const Eigen::Vector3d R3(R.x(), R.y(), 0.0), T3(-R.y(), R.x(), 0.0);
      const double a = t * M_PI / 180.0;
      const Eigen::Vector3d app = std::sin(a) * R3 - std::cos(a) * Eigen::Vector3d::UnitZ();
      const Eigen::Vector3d C = Eigen::AngleAxisd(rot * M_PI / 180.0, app) * T3;
      const Eigen::Vector3d tip(x, y, kTopUp - 0.030);
      bool g = false, f = false;
      for (const auto & s : arm_ik(m, tip, app, C, {}, 180.0)) {
        if (s.pos_mm < 1 && s.app_deg < 1 && s.roll_deg < 2 && ticks_in_range(m, q_to_ticks(m, s.q))) {qg = s.q; g = true; break;}
      }
      if (!g) {return false;}
      for (const auto & s : arm_ik(m, tip + Eigen::Vector3d(0, 0, 0.07), app, C, {qg}, 180.0)) {
        if (s.pos_mm < 1 && s.app_deg < 1 && s.roll_deg < 2 && ticks_in_range(m, q_to_ticks(m, s.q))) {qf = s.q; f = true; break;}
      }
      return f;
    };
  struct Case {double x, y, t, rot; bool table; double b; Ticks down4;};
  const Case cases[] = {
    {0.240, -0.090, 0.0, 0.0, false, 22.5, {4098, 1606, 2288, 2159, 2019}},
    {0.280, -0.100, 0.0, 0.0, false, 15.5, {4098, 1442, 2365, 2132, 2019}},
    {0.220, -0.060, 0.0, 0.0, false, 19.5, {4098, 1766, 2202, 2199, 2019}},
    {0.240, -0.090, 5.0, -45.0, true, 1.5, {4161, 1672, 2253, 2185, 1570}},
    {0.220, -0.100, 5.0, 0.0, true, 1.0, {4098, 1687, 2245, 2178, 2019}},
    {0.240, -0.090, 27.0, 0.0, true, 7.0, {4098, 1323, 2417, 2119, 2019}},
    {0.300, -0.090, 24.0, -78.0, true, -1.0, {}},   // 표 쥐기 (손 24°, 손목 −78°) 처럼 어느 b 도 안 됨
  };
  for (const auto & c : cases) {
    SCOPED_TRACE(std::to_string(c.x) + " " + std::to_string(c.y) + " " + std::to_string(c.t) + " " + std::to_string(c.rot));
    Q5 qg{}, qf{};
    if (c.table) {
      ASSERT_TRUE(table_grasp(c.x, c.y, c.t, c.rot, qg, qf));
    } else {
      std::vector<PickPose> P;
      ASSERT_TRUE(tilted_pick(m, c.x, c.y, false, P));
      qg = ticks_to_q(m, P.back().ticks);
      qf = ticks_to_q(m, P[0].ticks);
    }
    UprightHeld h;
    h.q_grasp = qg;
    h.puck_xy = Eigen::Vector2d(c.x, c.y);
    h.bottom_z = kTopUp - 0.060;
    h.square = false;
    h.width = 0.0335;
    std::vector<ParkNode> pn;
    UprightInfo info;
    std::string why;
    const bool ok = park_upright_nodes(m, pk, u, h, qf, pn, info, why);
    if (c.b < 0.0) {
      EXPECT_FALSE(ok);
      EXPECT_NE(why.find("받침에 놓는 자세 없음"), std::string::npos) << why;
      continue;
    }
    ASSERT_TRUE(ok) << why;
    EXPECT_NEAR(info.b_mm, c.b, 1e-6);
    EXPECT_EQ(q_to_ticks(m, find(pn, "pk_down4")->q), c.down4);
  }
}

// ---- 받침 다시 쥐기 고정 보정 (park_regrasp_xy_mm) 충돌 계산 ----
//   누운 퍽 세우기 (꽂아 누르기, 실물 값) · 기울여 쥔 원기둥 · 네모 → 받침 퍽을 놓은 뒤 (press_up · pk_up) 부터 park_re_lift 까지
//   관절 직선 2° 간격으로 손가락 점 (안 · 가운데 · 바깥 면 × 폭 5 점 × 손끝에서 30 mm 위까지) 을 받침 벽 (안 44 × 46.6 · 바깥 50 × 53, 높이 26.4) 과
//   받침 퍽 (원기둥 r 16.75 · 네모 반 17.4 ± 18.4° 돎, 바닥 ~ 60 mm) 에 대해 잼. 퍽 자리는 받침 안 놀이 (접선 · 반지름) 전체 격자에서 가장 나쁜 값과,
//   실물에서 잰 자리 (+2.4, +5.3 → 놀이 안으로 자름) 에서의 값을 따로. 손 상태: 열림 = 손가락 안쪽 면 finger_inner0 + grip_open (모델, 실물 58 mm 보다 좁게 = 보수적)
namespace
{
struct ClearOut {double wall{1e9}, puck_worst{1e9}, puck_real{1e9}; std::string wall_at, puck_at;};

void finger_pts(const ArmModel & m, const Q5 & q, double half_in, const UprightCfg & u, std::vector<Eigen::Vector3d> & out)
{
  const Grip g = arm_grip(m, q);
  const Eigen::Vector3d app = g.app.normalized(), cl = g.close.normalized(), b = app.cross(cl).normalized();
  const double th = u.finger_outer0 - u.finger_inner0;
  out.clear();
  for (int side : {-1, 1}) {
    for (double xc : {half_in, half_in + 0.5 * th, half_in + th}) {
      for (int k = -2; k <= 2; ++k) {
        for (double za = 0.0; za <= 0.030 + 1e-9; za += 0.005) {out.push_back(g.tip + side * xc * cl + (u.finger_half_w * k / 2.0) * b - za * app);}
      }
    }
  }
}

// 받침 벽: 바깥 상자 안 · 안쪽 상자 밖 · 벽 윗면 아래 = 닿음. 값 = 그런 발자국 점의 (z − 벽 윗면) 최소 (− = 박힘)
double wall_clear(const ParkCfg & c, const UprightCfg & u, const Eigen::Vector2d & rd, const std::vector<Eigen::Vector3d> & P)
{
  const Eigen::Vector2d td(-rd.y(), rd.x());
  double cl = 1e9;
  for (const auto & p : P) {
    const Eigen::Vector2d d = p.head<2>() - c.center;
    const double r = std::fabs(d.dot(rd)), t = std::fabs(d.dot(td));
    if (r > u.cup_out_r || t > u.cup_out_t) {continue;}
    if (r < u.cup_in_r && t < u.cup_in_t) {continue;}   // 안쪽 (퍽 자리)
    cl = std::min(cl, p.z() - (c.floor_z + u.wall_h));
  }
  return cl;
}

// 받침 퍽 (중심 pc, 돎 yaw): 옆 거리 (− = 안) · 윗면 위 거리 중 큰 쪽 (= 떨어진 정도), 점들 최소
double puck_clear(const ParkCfg & c, const UprightCfg & u, const Eigen::Vector2d & rd, bool square, const Eigen::Vector2d & pc, double yaw,
  const std::vector<Eigen::Vector3d> & P)
{
  const Eigen::Vector2d td(-rd.y(), rd.x());
  const double cy = std::cos(yaw), sy = std::sin(yaw);
  double cl = 1e9;
  for (const auto & p : P) {
    const Eigen::Vector2d d = p.head<2>() - pc;
    double lat;
    if (square) {
      const double a = d.dot(td), r = d.dot(rd);
      const double x = std::fabs(cy * a + sy * r) - 0.0174, y = std::fabs(-sy * a + cy * r) - 0.0174;
      lat = std::max(x, y) > 0.0 ? std::hypot(std::max(x, 0.0), std::max(y, 0.0)) : std::max(x, y);
    } else {
      lat = d.norm() - 0.01675;
    }
    const double above = p.z() - (c.floor_z + u.puck_h);
    cl = std::min(cl, std::max(lat, above));
  }
  return cl;
}

ClearOut stand_clear(const ArmModel & m, const ParkCfg & c, const UprightCfg & u, const std::vector<ParkNode> & pn, const std::string & from,
  bool square, bool closed_from, const Eigen::Vector2d & real_off)
{
  ClearOut o;
  const Eigen::Vector2d rd = (c.center - m.j1_xy).normalized(), td(-rd.y(), rd.x());
  auto it = std::find_if(pn.begin(), pn.end(), [&](const ParkNode & n) {return n.name == from;});
  if (it == pn.end()) {o.wall = o.puck_worst = o.puck_real = -1e9; o.wall_at = "no " + from; return o;}
  const double half_open = u.finger_inner0 + u.grip_open, half_closed = u.finger_inner0;
  bool open = !closed_from;
  const double pt = c.inner_half > 0 ? u.cup_in_t - (square ? 0.0174 : 0.01675) : 0.0, pr = u.cup_in_r - (square ? 0.0174 : 0.01675);
  const Eigen::Vector2d rl(std::clamp(real_off.dot(rd), -pr, pr), std::clamp(real_off.dot(td), -pt, pt));   // (반지름, 접선) 놀이 안으로
  std::vector<Eigen::Vector3d> P;
  for (auto a = it; a + 1 != pn.end(); ++a) {
    if (a->grip == "open") {open = true;}
    if (a->grip == "close") {break;}   // 닫은 뒤 (re_down 에서 쥠) 는 퍽이 손 안
    const auto b = a + 1;
    double dmax = 0.0;
    for (int j = 0; j < 5; ++j) {dmax = std::max(dmax, std::fabs(b->q[j] - a->q[j]));}
    const int n = std::max(1, static_cast<int>(std::ceil(dmax * 180.0 / M_PI / 2.0)));
    for (int i = 0; i <= n; ++i) {
      Q5 q;
      for (int j = 0; j < 5; ++j) {q[j] = a->q[j] + (b->q[j] - a->q[j]) * i / n;}
      const bool closing = b->grip == "close" && i == n;   // 닫기 직전 자세는 열린 손
      finger_pts(m, q, open || closing ? half_open : half_closed, u, P);
      const std::string at = a->name + "→" + b->name;
      const double w = wall_clear(c, u, rd, P);
      if (w < o.wall) {o.wall = w; o.wall_at = at;}
      for (double fr : {-1.0, -0.5, 0.0, 0.5, 1.0}) {
        for (double ft : {-1.0, -0.5, 0.0, 0.5, 1.0}) {
          for (double yd : square ? std::vector<double>{-18.4, -10.0, 0.0, 10.0, 18.4} : std::vector<double>{0.0}) {
            const double ys = yd * M_PI / 180.0;
            const double tmax = square ? u.cup_in_t - 0.0174 * (std::cos(std::fabs(ys)) + std::sin(std::fabs(ys))) : pt;
            const Eigen::Vector2d pc = c.center + fr * pr * rd + ft * std::max(0.0, tmax) * td;
            const double pv = puck_clear(c, u, rd, square, pc, ys, P);
            if (pv < o.puck_worst) {o.puck_worst = pv; o.puck_at = at;}
          }
        }
      }
      const Eigen::Vector2d pr0 = c.center + rl.x() * rd + rl.y() * td;
      o.puck_real = std::min(o.puck_real, puck_clear(c, u, rd, square, pr0, 0.0, P));
    }
  }
  return o;
}
}  // namespace

TEST(ParkFix1116, RegraspOffsetCollision)
{
  const auto m = kin_data::model();
  const UprightCfg u;
  const Eigen::Vector2d real_off(0.0024, 0.0053);   // 실물 잰 받침 퍽 자리 평균 (base_link x · y)
  std::printf("받침 다시 쥐기 충돌 계산 [mm] (+ = 떨어짐, − = 닿음/박힘)  열린 손 안쪽 반 %.1f mm (모델)\n", (u.finger_inner0 + u.grip_open) * 1000.0);
  for (const Eigen::Vector2d off : {Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(0.002, 0.0035)}) {
    double w_min = 1e9, pr_min = 1e9, pw_min = 1e9;
    int n = 0;
    // 누운 퍽 세우기 (꽂아 누르기, 실물 control.yaml 값)
    for (double x : {0.205, 0.215, 0.225, 0.235}) {
      for (double y : {-0.06, -0.03, 0.0, 0.03, 0.06}) {
        for (double ax : {0.0, 90.0, 180.0, 270.0}) {
          ParkCfg pk;
          pk.press_mode = true;
          pk.press_r = -0.028;
          pk.press_top = 0.066;
          pk.press_h = 0.010;
          pk.regrasp_r = 0.0;
          pk.regrasp_depth = 0.030;
          pk.regrasp_xy = off;
          LyingGraspCfg gc;
          std::vector<ParkNode> pn;
          double elev = 0.0;
          auto accept = [&](const LyingGrasp & cg, std::string & w) {return park_nodes(m, pk, cg, ticks_to_q(m, cg.poses[0].ticks), pn, elev, w);};
          LyingGrasp g;
          std::string why;
          if (!lying_grasp(m, x, y, kTop, ax, kHeights, gc, g, why, 0.0, accept)) {continue;}
          const auto o = stand_clear(m, pk, u, pn, "park_press_up", false, true, real_off);
          ++n;
          if (o.wall < w_min) {w_min = o.wall;}
          pw_min = std::min(pw_min, o.puck_worst);
          pr_min = std::min(pr_min, o.puck_real);
          EXPECT_GT(o.wall, 0.0) << "누운 (" << x << ", " << y << ") " << ax << "° 벽 " << o.wall_at;
          EXPECT_GT(o.puck_real, 0.0) << "누운 (" << x << ", " << y << ") " << ax << "° 퍽 " << o.puck_at;
          EXPECT_GT(o.puck_worst, -1e-4) << "놀이 전체 가장 나쁜 퍽 자리 " << o.puck_at;
        }
      }
    }
    std::printf("  보정 (%+.1f, %+.1f) 누운 퍽 세우기 %d 가지: 벽 %.1f · 퍽 (잰 자리) %.1f · 퍽 (놀이 전체 가장 나쁜) %.1f\n", off.x() * 1000, off.y() * 1000, n,
      w_min * 1000, pr_min * 1000, pw_min * 1000);
    for (bool square : {false, true}) {
      double w2 = 1e9, pr2 = 1e9, pw2 = 1e9;
      int n2 = 0;
      for (double x : {0.24, 0.25, 0.28, 0.31, 0.33}) {
        for (double y : {-0.09, -0.03, 0.0, 0.03, 0.09}) {
          ParkCfg pk;
          pk.regrasp_r = 0.0;
          pk.regrasp_xy = off;
          std::vector<PickPose> P;
          if (!tilted_pick(m, x, y, square, P)) {continue;}
          std::vector<ParkNode> pn;
          UprightInfo info;
          std::string why;
          if (!park_upright_nodes(m, pk, u, held_of(m, P, x, y, square), ticks_to_q(m, P[0].ticks), pn, info, why)) {continue;}
          const auto o = stand_clear(m, pk, u, pn, "pk_up", square, false, real_off);
          ++n2;
          w2 = std::min(w2, o.wall);
          pw2 = std::min(pw2, o.puck_worst);
          pr2 = std::min(pr2, o.puck_real);
          EXPECT_GT(o.wall, 0.0) << (square ? "네모" : "원기둥") << " (" << x << ", " << y << ") 벽 " << o.wall_at;
          EXPECT_GT(o.puck_real, 0.0) << (square ? "네모" : "원기둥") << " (" << x << ", " << y << ") 퍽 " << o.puck_at;
          EXPECT_GT(o.puck_worst, -1e-4) << "놀이 전체 가장 나쁜 퍽 자리 " << o.puck_at;
        }
      }
      std::printf("  보정 (%+.1f, %+.1f) 기울여 쥔 %s %d 가지: 벽 %.1f · 퍽 (잰 자리) %.1f · 퍽 (놀이 전체 가장 나쁜) %.1f\n", off.x() * 1000, off.y() * 1000,
        square ? "네모" : "원기둥", n2, w2 * 1000, pr2 * 1000, pw2 * 1000);
    }
  }
}
