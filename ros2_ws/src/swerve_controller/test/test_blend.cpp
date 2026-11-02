// 동시 조향·구동(blend) 시험본 단위 시험. 기존 test_swerve.cpp 57 개는 그대로 통과해야 함.
#include <gtest/gtest.h>
#include <cmath>
#include "swerve_controller/swerve.hpp"
using namespace swerve_controller;

namespace {
const double PI = 3.14159265358979323846;
SwerveKinematics kin() {
  std::vector<ModuleGeometry> g = {{-0.064, 0.112, 0.0269}, {0.064, 0.112, 0.0269}, {-0.064, -0.112, -0.0269}, {0.064, -0.112, -0.0269}};
  return SwerveKinematics(g, 0.033, PI, {}, true);
}
std::vector<ModuleState> at(double a) {return std::vector<ModuleState>(4, ModuleState{a, 0.0, 0.0});}
// 전진으로 충분히 달린 뒤의 SwerveDrive (조향 0, 바퀴는 전진 속도라고 가정)
SwerveDrive running(SwerveDriveConfig c, double & t) {
  SwerveDrive d(kin(), c);
  t = 0.0;
  for (int k = 0; k < 200; ++k, t += 0.01) {d.set_command({0.05, 0, 0}, t); d.set_turn(true, 0, t); d.update(t, 0.01, at(0.0));}
  return d;
}
}  // namespace

TEST(Blend, StallUnsetKeepsLegacyGate) {   // blend 를 켜도 정체 시간이 없으면 매 주기 기존과 같아야 함
  SwerveDriveConfig a, b; b.blend_steer_drive = true; b.blend_stall_s = 0.0;
  double ta, tb; auto da = running(a, ta); auto db = running(b, tb);
  bool gated = false;
  for (int k = 0; k < 100; ++k, ta += 0.01, tb += 0.01) {   // 가속 제한 때문에 방향은 몇 주기에 걸쳐 바뀜
    da.set_command({0, 0.05, 0}, ta); db.set_command({0, 0.05, 0}, tb);
    da.set_turn(true, 0, ta); db.set_turn(true, 0, tb);
    const auto ma = da.update(ta, 0.01, at(0.0)), mb = db.update(tb, 0.01, at(0.0));
    for (int i = 0; i < 4; ++i) {EXPECT_DOUBLE_EQ(ma[i].steer_angle, mb[i].steer_angle); EXPECT_DOUBLE_EQ(ma[i].wheel_speed, mb[i].wheel_speed);}
    EXPECT_EQ(da.gate_blocked(), db.gate_blocked());
    gated = gated || da.gate_blocked();
  }
  EXPECT_TRUE(gated);   // 조향이 안 따라오면 기존 게이트가 걸림
}

TEST(Blend, WheelsFollowMeasuredWhileSteerGoesToFinalAngle) {   // 2026-10-26: 바퀴는 진행 비율 명령, 조향은 최종 각도로 바로
  SwerveDriveConfig c; c.blend_steer_drive = true; c.blend_stall_s = 0.5;
  double t; auto d = running(c, t);
  bool limited = false, led = false;
  for (int k = 0; k < 40; ++k, t += 0.01) {   // 0.4 s < blend_stall_s
    d.set_command({0, 0.05, 0}, t); d.set_turn(true, 0, t);
    const auto m = d.update(t, 0.01, at(0.0));
    EXPECT_FALSE(d.gate_blocked());
    if (d.blend_ratio() < 1.0) {
      limited = true;
      // 조향 목표 = 새 명령의 최종 각도 +90° (이동 중이라 바깥쪽 −90° 로 뒤집지 않음, 네 바퀴 같은 분기)
      for (const auto & s : m) {EXPECT_NEAR(s.steer_angle, PI / 2, 1e-9);}
      led = true;
      for (const auto & s : m) {EXPECT_GT(s.wheel_speed, 0.5);}   // 구동은 계속 (조향 0° 방향 = 실측과 맞는 몸 명령)
    }
  }
  EXPECT_TRUE(limited);
  EXPECT_TRUE(led);
}

TEST(Blend, MarginAboveOneIsClamped) {   // blend_margin 을 1 보다 크게 줘도 게이트를 넓히지 않음 (1.0 과 같은 출력)
  SwerveDriveConfig c1, c3; c1.blend_steer_drive = c3.blend_steer_drive = true; c1.blend_stall_s = c3.blend_stall_s = 0.5;
  c3.blend_margin = 3.0;
  double t1, t3; auto d1 = running(c1, t1); auto d3 = running(c3, t3);
  for (int k = 0; k < 40; ++k, t1 += 0.01, t3 += 0.01) {
    d1.set_command({0, 0.05, 0}, t1); d1.set_turn(true, 0, t1);
    d3.set_command({0, 0.05, 0}, t3); d3.set_turn(true, 0, t3);
    const auto m1 = d1.update(t1, 0.01, at(0.0)), m3 = d3.update(t3, 0.01, at(0.0));
    EXPECT_DOUBLE_EQ(d1.blend_ratio(), d3.blend_ratio());
    for (int i = 0; i < 4; ++i) {EXPECT_DOUBLE_EQ(m1[i].wheel_speed, m3[i].wheel_speed);}
  }
}

TEST(Blend, TurningSteerIsNotStall) {   // 조향이 돌고 있으면 큰 전환이라도 정체로 세우지 않음
  SwerveDriveConfig c; c.blend_steer_drive = true; c.blend_stall_s = 0.3;
  double t; auto d = running(c, t);
  auto meas = at(0.0);
  for (auto & m : meas) {m.steer_rate = 1.0;}
  for (int k = 0; k < 100; ++k, t += 0.01) {
    d.set_command({0, 0.05, 0}, t); d.set_turn(true, 0, t);
    d.update(t, 0.01, meas);
    EXPECT_FALSE(d.blend_stalled());
  }
}

TEST(Blend, OutwardOnlyFromStandstill) {   // 같은 왼쪽 명령: 멈춰서 출발 → fr·rl 바깥쪽(−90°), 이동 중 → 지금 분기(+90°)
  const auto k = kin();
  const BodyTwist left{0, 0.05, 0};
  const auto rest = k.inverse(left, at(0.0), false, nullptr, {}, true);
  EXPECT_NEAR(rest[1].steer_angle, -PI / 2, 1e-9);
  EXPECT_NEAR(rest[2].steer_angle, -PI / 2, 1e-9);
  const auto moving = k.inverse(left, at(0.0), false, nullptr, at(PI / 4), false);
  for (const auto & s : moving) {EXPECT_NEAR(s.steer_angle, PI / 2, 1e-9);}
}

TEST(Blend, StallStopsDrive) {   // 실측 조향이 안 움직이면 blend_stall_s 뒤 구동 0
  SwerveDriveConfig c; c.blend_steer_drive = true; c.blend_stall_s = 0.3;
  double t; auto d = running(c, t);
  bool stopped = false;
  for (int k = 0; k < 100; ++k, t += 0.01) {
    d.set_command({0, 0.05, 0}, t); d.set_turn(true, 0, t);
    d.update(t, 0.01, at(0.0));   // 조향이 0 에서 안 움직임 (멈춤 고장)
    if (d.blend_stalled()) {stopped = true; EXPECT_TRUE(d.gate_blocked()); break;}
  }
  EXPECT_TRUE(stopped);
  EXPECT_GE(t, 2.0 + 0.3 - 0.02);
}

TEST(Blend, StaleCommandDoesNotBlend) {   // 명령이 끊기면 blend 하지 않고 기존 감속·조향 유지
  SwerveDriveConfig c; c.blend_steer_drive = true; c.blend_stall_s = 0.5;
  double t; auto d = running(c, t);
  d.set_command({0, 0.05, 0}, t);
  t += 0.5;   // cmd_timeout 0.3 s 넘김
  const auto m = d.update(t, 0.01, at(0.0));
  EXPECT_DOUBLE_EQ(d.blend_ratio(), 1.0);
  for (const auto & s : m) {EXPECT_DOUBLE_EQ(s.steer_angle, 0.0);}   // 새 조향을 시작하지 않음
}

TEST(Blend, NanMeasurementSkipsBlend) {
  SwerveDriveConfig c; c.blend_steer_drive = true; c.blend_stall_s = 0.5;
  double t; auto d = running(c, t);
  d.set_command({0, 0.05, 0}, t);
  auto meas = at(0.0); meas[2].steer_angle = NAN;
  d.update(t, 0.01, meas);
  EXPECT_DOUBLE_EQ(d.blend_ratio(), 1.0);
}

TEST(Blend, PureRotationToTranslationKeepsLegacyGate) {   // 2026-10-26 실물: 회전→전진은 기존 "조향 먼저"
  SwerveDriveConfig c; c.blend_steer_drive = true; c.blend_stall_s = 1.0;
  SwerveDrive d(kin(), c);
  double t = 0.0;
  for (int k = 0; k < 200; ++k, t += 0.01) {d.set_command({0, 0, 0.3}, t); d.set_turn(true, 0, t); d.update(t, 0.01, at(0.0));}
  for (int k = 0; k < 50; ++k, t += 0.01) {
    d.set_command({0.05, 0, 0}, t); d.set_turn(true, 0, t);
    d.update(t, 0.01, at(0.0));
    EXPECT_DOUBLE_EQ(d.blend_ratio(), 1.0);
  }
}

TEST(MaxPerformance, ZeroBodyCapsPassCommandAndTimeoutStillStops) {
  SwerveDriveConfig c;
  c.max_linear = c.max_angular = c.max_linear_accel = c.max_angular_accel = 0.0;
  c.steer_then_drive = false; c.cosine_scale = false; c.roll_while_steering = false;
  SwerveDrive d(kin(), c);
  d.set_turn(true, 0, 0.0); d.set_command({0.26, 0.0, 0.0}, 0.0);
  auto m = d.update(0.0, 0.01, at(0.0));
  for (const auto & x : m) {EXPECT_NEAR(x.wheel_speed, .26/.033, 1e-9);}
  m = d.update(0.31, 0.01, at(0.0));
  for (const auto & x : m) {EXPECT_DOUBLE_EQ(x.wheel_speed, 0.0);}
}

TEST(MaxPerformance, WheelRegisterBudgetPreservesRatios) {
  SwerveDriveConfig c;
  c.max_linear = c.max_angular = c.max_linear_accel = c.max_angular_accel = 0.0;
  c.steer_then_drive = false; c.cosine_scale = false; c.roll_while_steering = false;
  auto unlimited = c; c.max_wheel_speed = 7.9;
  SwerveDrive a(kin(), c), b(kin(), unlimited);
  a.set_turn(true,0,0); b.set_turn(true,0,0);
  a.set_command({.26,.1,1.5},0); b.set_command({.26,.1,1.5},0);
  auto actual=a.update(0,.01,at(0)), raw=b.update(0,.01,at(0));
  double peak=0;for(const auto & m:raw){peak=std::max(peak,std::fabs(m.wheel_speed));}
  ASSERT_GT(peak,7.9);
  for(int i=0;i<4;i++) {
    EXPECT_NEAR(actual[i].wheel_speed,raw[i].wheel_speed*7.9/peak,1e-9);
    EXPECT_DOUBLE_EQ(actual[i].steer_angle,raw[i].steer_angle);
  }
}

TEST(MaxPerformance, FullSingleTurnRangeKeepsEquivalentDiagonalInsideEndpoints) {
  // The W350 position-mode endpoint cannot be extended merely by removing margins.
  const double tick=2*PI/4096;
  std::vector<SteeringRange> ranges;
  for(int zero:{3066,1552,2577,3589}){ranges.push_back({(zero-4095)*tick,zero*tick});}
  SwerveKinematics k({{-.064,.112,.0269},{.064,.112,.0269},{-.064,-.112,-.0269},{.064,-.112,-.0269}},.033,PI,ranges,true,0);
  bool feasible=false;
  auto m=k.inverse({.18,-.18,0},at(0),false,&feasible);
  ASSERT_TRUE(feasible);
  EXPECT_GT(m[3].steer_angle,0.0);
  EXPECT_NEAR(std::sin(m[3].steer_angle + PI/4),0.0,1e-9);
  for(int i=0;i<4;i++){EXPECT_GE(m[i].steer_angle,ranges[i].min);EXPECT_LE(m[i].steer_angle,ranges[i].max);}
}
