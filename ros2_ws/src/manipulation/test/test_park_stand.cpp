// park_stand: 누운 퍽 쥐기 (기울인 손 · 굴림) · 받침 넣기 자세
#include <cctype>
#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include "manipulation/park_stand.hpp"
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
//   시뮬 run_fixture 도 바깥 끝). 넣기 자세 = 예전 park_plan 식 (손끝 tr · tz)
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

// 누르기 (밀기 대신): 넣고 놓기까지 같음 → 놓은 손끝에서 press_lift 위 (닫힘) → J1 쪽으로 press_back → 곧게 press_depth 아래 → 위로 빠짐.
//   손 기울임은 놓을 때와 같음, 밀기 점은 없음, 선 퍽 다시 쥐기는 그대로
TEST(ParkStand, PressModeLiftsBackAndPressesDown)
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
  const Eigen::Vector3d R(r2.x(), r2.y(), 0.0);
  for (const char * nm : {"park_over", "park_tilt8"}) {   // 넣기까지 같음
    ASSERT_NE(find(press, nm), nullptr) << nm;
    for (int j = 0; j < 5; ++j) {EXPECT_NEAR(find(press, nm)->q[j], find(push, nm)->q[j], 1e-9) << nm;}
  }
  EXPECT_EQ(find(press, "park_push1"), nullptr);
  EXPECT_EQ(find(press, "park_push_over"), nullptr);
  // 놓은 점 = 당기며 놓기 마지막 (없으면 마지막 기울임 점)
  const ParkNode * rel = find(press, "park_pull" + std::to_string(pk.release_pull_steps));
  if (rel == nullptr) {rel = find(press, "park_tilt" + std::to_string(pk.tilt_steps));}
  ASSERT_NE(rel, nullptr);
  const Grip g0 = arm_grip(m, rel->q);
  const ParkNode * lift = find(press, "park_press_lift");
  ASSERT_NE(lift, nullptr);
  EXPECT_EQ(lift->grip, "close");
  const Grip gl = arm_grip(m, lift->q);
  EXPECT_NEAR((gl.tip - g0.tip - pk.press_lift * Eigen::Vector3d::UnitZ()).norm(), 0.0, 1e-3);
  EXPECT_NEAR(gl.app.dot(g0.app), 1.0, 1e-4) << "손 기울임 그대로";
  const Grip gb = arm_grip(m, find(press, "park_press_back")->q);
  EXPECT_NEAR((gb.tip - gl.tip + pk.press_back * R).norm(), 0.0, 1e-3) << "J1 쪽으로";
  const Grip ge = arm_grip(m, find(press, "park_press" + std::to_string(pk.press_steps))->q);
  EXPECT_NEAR((ge.tip - gb.tip + pk.press_depth * Eigen::Vector3d::UnitZ()).norm(), 0.0, 1e-3) << "곧게 아래로";
  EXPECT_NE(find(press, "park_press_up"), nullptr);
  EXPECT_NE(find(press, "park_re_down"), nullptr) << "누른 뒤 다시 쥐기는 그대로";
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
