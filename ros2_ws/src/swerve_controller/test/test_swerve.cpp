// test_swerve.cpp — 바퀴 쪽 순수 계산 시험 (swerve.hpp: 기구학 · 운전석 · 자이로)
// colcon test --packages-select swerve_controller

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "swerve_controller/swerve.hpp"

// ============================================================================ ① 기구학
// 실행: colcon test --packages-select swerve_controller && colcon test-result --verbose
//
// 각 시험은 "답을 머리로 알 수 있는 상황" 을 골랐다. 공식이 틀리면 바로 드러나게.

using swerve_controller::BodyTwist;
using swerve_controller::ModuleGeometry;
using swerve_controller::ModuleState;
using swerve_controller::SwerveKinematics;

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kR = 0.033;   // 바퀴 반지름
constexpr double kTol = 1e-9;

// 순서: fl, fr, rl, rr (hardware_map · controllers.yaml 과 같은 순서)
SwerveKinematics make()
{
  return SwerveKinematics({{0.09, 0.09}, {0.09, -0.09}, {-0.09, 0.09}, {-0.09, -0.09}}, kR);
}
std::vector<ModuleState> facing(double angle)
{
  return std::vector<ModuleState>(4, ModuleState{angle, 0.0});
}
}  // namespace

// 1) 앞으로 0.1 m/s → 4개 모두 0°, 바퀴 0.1/0.033 = 3.03 rad/s
TEST(SwerveInverse, StraightForward)
{
  auto out = make().inverse({0.1, 0.0, 0.0}, facing(0.0));
  for (const auto & m : out) {
    EXPECT_NEAR(m.steer_angle, 0.0, kTol);
    EXPECT_NEAR(m.wheel_speed, 0.1 / kR, kTol);
  }
}

// 2) 왼쪽으로 게걸음 0.1 m/s → 4개 모두 +90°
//    지금 각도를 90° 로 주면 뒤집기 없이 그대로 +90°, 바퀴 +3.03
TEST(SwerveInverse, StrafeLeft)
{
  auto out = make().inverse({0.0, 0.1, 0.0}, facing(kPi / 2));
  for (const auto & m : out) {
    EXPECT_NEAR(m.steer_angle, kPi / 2, kTol);
    EXPECT_NEAR(m.wheel_speed, 0.1 / kR, kTol);
  }
}

// 3) 제자리 회전 → 각 모듈이 중심을 도는 원의 접선 방향 (fl 은 135°)
//    속도 = ω × 중심까지 거리 = 1.0 × √(0.09² + 0.09²)
TEST(SwerveInverse, SpinInPlace)
{
  auto out = make().inverse({0.0, 0.0, 1.0}, {});   // current 없음 → 뒤집기 안 함
  const double r = std::hypot(0.09, 0.09);
  EXPECT_NEAR(out[0].steer_angle, 3 * kPi / 4, 1e-9);   // fl
  EXPECT_NEAR(out[3].steer_angle, -kPi / 4, 1e-9);      // rr (fl 의 정반대)
  for (const auto & m : out) {
    EXPECT_NEAR(m.wheel_speed, r / kR, 1e-9);
  }
}

// 4) 뒤집기: 지금 0° 인데 "뒤로" 가라 → 180° 로 돌지 말고 0° 그대로 바퀴만 거꾸로
TEST(SwerveInverse, FlipInsteadOfTurning180)
{
  auto out = make().inverse({-0.1, 0.0, 0.0}, facing(0.0));
  for (const auto & m : out) {
    EXPECT_NEAR(m.steer_angle, 0.0, kTol);
    EXPECT_NEAR(m.wheel_speed, -0.1 / kR, kTol);
  }
}

// 5) 코사인 줄이기: 지금 0° 인데 왼쪽(90°)으로 가라 → 조향은 90° 로, 바퀴는 cos(90°)=0 → 일단 멈춰서 돌기
TEST(SwerveInverse, CosineScaleWaitsForSteer)
{
  auto out = make().inverse({0.0, 0.1, 0.0}, facing(0.0));
  for (const auto & m : out) {
    EXPECT_NEAR(std::fabs(m.steer_angle), kPi / 2, kTol);
    EXPECT_NEAR(m.wheel_speed, 0.0, 1e-9);
  }
}

// 6) 멈춤 명령 → 조향은 지금 자리 유지, 바퀴 0
TEST(SwerveInverse, StopKeepsSteerAngle)
{
  auto out = make().inverse({0.0, 0.0, 0.0}, facing(0.7));
  for (const auto & m : out) {
    EXPECT_NEAR(m.steer_angle, 0.7, kTol);
    EXPECT_NEAR(m.wheel_speed, 0.0, kTol);
  }
}

// 7) 왕복: 아무 속도나 역기구학 → 순기구학 하면 원래 속도로 돌아와야 함 (odom 이 맞다는 증거)
TEST(SwerveForward, RoundTrip)
{
  auto k = make();
  const BodyTwist cmds[] = {{0.1, 0.0, 0.0}, {0.0, -0.05, 0.0}, {0.0, 0.0, 0.8}, {0.07, 0.03, -0.4}};
  for (const auto & c : cmds) {
    auto modules = k.inverse(c, {});
    auto back = k.forward(modules);
    EXPECT_NEAR(back.vx, c.vx, 1e-9);
    EXPECT_NEAR(back.vy, c.vy, 1e-9);
    EXPECT_NEAR(back.wz, c.wz, 1e-9);
  }
}

TEST(WrapAngle, Basics)
{
  EXPECT_NEAR(swerve_controller::wrap_angle(3 * kPi / 2), -kPi / 2, kTol);
  EXPECT_NEAR(swerve_controller::wrap_angle(-3 * kPi / 2), kPi / 2, kTol);
  EXPECT_NEAR(swerve_controller::wrap_angle(0.3), 0.3, kTol);
}

namespace
{
SwerveKinematics cad_limited()
{
  return SwerveKinematics({{.112,.064,.0279},{.112,-.064,-.0279},
    {-.112,.064,.0279},{-.112,-.064,-.0279}}, kR, kPi,
    {{-kPi/2,0},{0,kPi/2},{0,kPi/2},{-kPi/2,0}});
}
}

TEST(SwerveRanges, SidewaysUsesDifferentAnglesAndReversesWheels)
{
  bool ok=false;
  auto k=cad_limited();
  auto out=k.inverse({0,.05,0},facing(0),false,&ok);
  ASSERT_TRUE(ok);
  const double sign[]={-1,1,1,-1};
  for (int i=0;i<4;++i) {
    EXPECT_NEAR(out[i].steer_angle,sign[i]*kPi/2,kTol);
    EXPECT_NEAR(out[i].wheel_speed,sign[i]*.05/kR,kTol);
  }
  auto v=k.forward(out);
  EXPECT_NEAR(v.vx,0,kTol); EXPECT_NEAR(v.vy,.05,kTol); EXPECT_NEAR(v.wz,0,kTol);
}

TEST(SwerveRanges, AllCardinalDirectionsAndSpinRoundTrip)
{
  auto k=cad_limited();
  for (const auto & cmd : std::vector<BodyTwist>{{.05,0,0},{-.05,0,0},{0,.05,0},{0,-.05,0},{0,0,.2},{0,0,-.2}}) {
    bool ok=false;
    auto out=k.inverse(cmd,facing(0),false,&ok);
    ASSERT_TRUE(ok);
    for (int i=0;i<4;++i) {
      const bool positive=i==1 || i==2;
      EXPECT_GE(out[i].steer_angle,positive ? 0 : -kPi/2);
      EXPECT_LE(out[i].steer_angle,positive ? kPi/2 : 0);
    }
    auto back=k.forward(out);
    EXPECT_NEAR(back.vx,cmd.vx,kTol); EXPECT_NEAR(back.vy,cmd.vy,kTol); EXPECT_NEAR(back.wz,cmd.wz,kTol);
  }
}

TEST(SwerveRanges, ImpossibleDiagonalStopsEveryModuleWithoutSteering)
{
  auto now=facing(0);
  now[0].steer_angle=-.2; now[1].steer_angle=.3;
  bool ok=true;
  auto out=cad_limited().inverse({.03,.03,0},now,false,&ok);
  EXPECT_FALSE(ok);
  for (int i=0;i<4;++i) {
    EXPECT_DOUBLE_EQ(out[i].wheel_speed,0);
    EXPECT_DOUBLE_EQ(out[i].steer_angle,now[i].steer_angle);
  }
}

TEST(SwerveRanges, NarrowLimitsAreNotSilentlyExpandedOrClamped)
{
  SwerveKinematics k({{0,0}},kR,kPi,{{-.1,.1}});
  bool ok=true;
  auto out=k.inverse({0,.05,0},{{0,0}},false,&ok);
  EXPECT_FALSE(ok); EXPECT_DOUBLE_EQ(out[0].wheel_speed,0);
  out=k.inverse({-.05,0,0},{{0,0}},false,&ok);
  EXPECT_TRUE(ok); EXPECT_NEAR(out[0].steer_angle,0,kTol);
  EXPECT_LT(out[0].wheel_speed,0);
}

TEST(SwerveRanges, ChoosesDirectBoundedPathRatherThanWrappingAcrossStop)
{
  SwerveKinematics k({{0,0}},kR,kPi,{{-kPi,kPi}});
  const double heading=-170*kPi/180;
  bool ok=false;
  auto out=k.inverse({.05*std::cos(heading),.05*std::sin(heading),0},
    {{170*kPi/180,0}},false,&ok);
  EXPECT_TRUE(ok);
  EXPECT_NEAR(out[0].steer_angle,10*kPi/180,kTol);
  EXPECT_LT(out[0].wheel_speed,0);
}

TEST(SwerveRanges, InvalidConfigurationRejected)
{
  EXPECT_THROW((SwerveKinematics({{0,0}},kR,kPi,{{0,1},{0,1}})),std::invalid_argument);
  EXPECT_THROW((SwerveKinematics({{0,0}},kR,kPi,{{1,0}})),std::invalid_argument);
  EXPECT_THROW((SwerveKinematics({{0,0}},kR,kPi,{{0,INFINITY}})),std::invalid_argument);
  EXPECT_THROW((SwerveKinematics({{0,0}},kR,kPi,{{NAN,1}})),std::invalid_argument);
  EXPECT_THROW((SwerveKinematics({{0,0}},kR,kPi,{{-4,4}})),std::invalid_argument);
}

TEST(SwerveRanges, DriveRejectsBeforeRampAndNeverReplacesStopWithRollCompensation)
{
  swerve_controller::SwerveDrive d(cad_limited(),{});
  auto now=facing(0);
  for (auto & m:now) {m.steer_rate=.2;}
  d.set_turn(true,1,0);
  d.set_command({.05,0,0},0);
  d.update(0,.01,now);  // prior feasible moving command
  d.set_command({.03,.03,0},.01);
  auto out=d.update(.01,.01,now);
  EXPECT_FALSE(d.command_feasible());
  EXPECT_FALSE(d.applied_drive());
  EXPECT_DOUBLE_EQ(d.output().vx,0);
  for (const auto & m:out) {EXPECT_DOUBLE_EQ(m.wheel_speed,0); EXPECT_DOUBLE_EQ(m.steer_angle,0);}
  d.set_command({0,.05,0},.02);
  out=d.update(.02,.01,now);
  EXPECT_TRUE(d.command_feasible());
  EXPECT_LT(out[0].steer_angle,0);
  EXPECT_LT(out[0].wheel_speed,0);  // compensation allowed again only for feasible steering
}

// ---------- 바퀴가 조향축 옆에 있는 경우 (CAD: 바깥쪽 26.9 mm) ----------
namespace
{
SwerveKinematics make_cad()
{
  return SwerveKinematics({{0.112, 0.064, 0.0269}, {0.112, -0.064, -0.0269},
                           {-0.112, 0.064, 0.0269}, {-0.112, -0.064, -0.0269}}, kR);
}
}  // namespace

// 9) 옆에 붙은 바퀴도 직진·게걸음은 그대로 (몸이 안 돌면 offset 이 상관없음)
TEST(SwerveOffset, TranslationUnchanged)
{
  auto a = make_cad().inverse({0.1, 0.05, 0.0}, facing(0.0), false);
  for (const auto & m : a) {
    EXPECT_NEAR(m.steer_angle, std::atan2(0.05, 0.1), 1e-9);
    EXPECT_NEAR(m.wheel_speed, std::hypot(0.1, 0.05) / kR, 1e-9);
  }
}

// 10) 제자리 회전: 바퀴가 실제로 도는 원의 반지름 = 조향축 거리 ± 26.9 mm
//     (어느 쪽으로 뒤집느냐에 따라 바퀴가 안쪽/바깥쪽에 옴)
TEST(SwerveOffset, SpinUsesWheelRadiusNotPivot)
{
  const double w = 1.0;
  const double rp = std::hypot(0.112, 0.064);
  auto out = make_cad().inverse({0.0, 0.0, w}, {}, false);   // 뒤집기 없이
  // fl: 조향 각도 = atan2(0.112, -0.064) (≈ 120°) → 바퀴(모듈 +y 쪽)가 회전 중심 쪽으로 옴 → 반지름 rp - 0.0269
  EXPECT_NEAR(out[0].wheel_speed * kR, w * (rp - 0.0269), 1e-9);
  // 같은 명령을 "뒤집은" 각도에서 보내면 바퀴가 바깥쪽 → 반지름 rp + 0.0269, 거꾸로 굴림
  auto flipped = make_cad().inverse({0.0, 0.0, w}, facing(std::atan2(0.112, -0.064) - kPi), false);
  EXPECT_NEAR(flipped[0].wheel_speed * kR, -w * (rp + 0.0269), 1e-9);
}

// 11) 왕복: inverse 로 만든 바퀴 명령을 forward 에 넣으면 원래 몸 속도가 나와야 함 (offset 포함)
TEST(SwerveOffset, ForwardInverseRoundTrip)
{
  const auto kin = make_cad();
  for (const BodyTwist cmd : {BodyTwist{0.1, 0.0, 0.0}, BodyTwist{0.0, 0.0, 1.2},
      BodyTwist{0.08, -0.05, 0.7}, BodyTwist{-0.1, 0.1, -0.4}})
  {
    auto states = kin.inverse(cmd, facing(0.3), false);
    auto back = kin.forward(states);
    EXPECT_NEAR(back.vx, cmd.vx, 1e-9);
    EXPECT_NEAR(back.vy, cmd.vy, 1e-9);
    EXPECT_NEAR(back.wz, cmd.wz, 1e-9);
  }
}

// 12) 회전 중심이 딱 조향축 위면 조향축은 안 움직이지만 바퀴는 offset 원을 돌아야 함
TEST(SwerveOffset, PivotAtRotationCenter)
{
  SwerveKinematics k({{0.0, 0.0, 0.0269}, {0.2, 0.0, 0.0}}, kR);
  auto out = k.inverse({0.0, 0.0, 1.0}, std::vector<ModuleState>{{0.0, 0.0}, {0.0, 0.0}}, false);
  EXPECT_NEAR(out[0].steer_angle, 0.0, 1e-12);                 // 각도는 그대로
  EXPECT_NEAR(out[0].wheel_speed * kR, -1.0 * 0.0269, 1e-9);    // ω × offset 만큼 굴림
}

// ---------- 느리게 · 한 무리씩 (D15 · D16) ----------

// 13) 가속 제한: 0 → 0.1 m/s 명령이어도 0.02 s 동안 0.2 m/s² 이면 0.004 m/s 만 올라감
TEST(SlowAndSequential, AccelLimit)
{
  using swerve_controller::limit_accel;
  auto out = limit_accel({0, 0, 0}, {0.1, 0.0, 0.5}, 0.2, 1.0, 0.02);
  EXPECT_NEAR(out.vx, 0.004, 1e-12);
  EXPECT_NEAR(out.vy, 0.0, 1e-12);
  EXPECT_NEAR(out.wz, 0.02, 1e-12);
  // 충분히 반복하면 목표에 도달하고 넘지 않음
  BodyTwist v{0, 0, 0};
  for (int i = 0; i < 100; ++i) {v = limit_accel(v, {0.1, 0.0, 0.5}, 0.2, 1.0, 0.02);}
  EXPECT_NEAR(v.vx, 0.1, 1e-12);
  EXPECT_NEAR(v.wz, 0.5, 1e-12);
}

// 14) 조향 정렬 판단: 하나라도 tol 넘게 남으면 false (바퀴는 아직 멈춰 있어야 함)
TEST(SlowAndSequential, SteeringAligned)
{
  using swerve_controller::steering_aligned;
  std::vector<ModuleState> cmd(4, ModuleState{1.0, 3.0});
  std::vector<ModuleState> meas(4, ModuleState{0.95, 0.0});
  EXPECT_TRUE(steering_aligned(cmd, meas, 0.1));
  meas[2].steer_angle = 0.5;
  EXPECT_FALSE(steering_aligned(cmd, meas, 0.1));
  // -179° 와 +179° 는 모터 입장에서 358° 차이 (위치 모드는 -180°~180° 사이에서만 움직임 → 빙 돌아야 함)
  EXPECT_FALSE(steering_aligned({{kPi - 0.01, 0}}, {{-kPi + 0.01, 0}}, 0.05));
}

// 15) 조향만 도는 동안 바퀴가 따라 구르는 속도, 그리고 그때 odom 이 "몸이 안 움직였다" 고 보는지
TEST(SlowAndSequential, RollWhileSteeringKeepsOdomStill)
{
  const auto kin = make_cad();
  const double rate = 1.0;   // 조향 1 rad/s
  std::vector<ModuleState> st(4);
  for (std::size_t i = 0; i < 4; ++i) {
    st[i] = {0.3, kin.roll_while_steering(i, rate), rate};
  }
  EXPECT_NEAR(st[0].wheel_speed * kR, -0.0269, 1e-12);   // fl: offset +0.0269 → 반대로 굴림
  const auto v = kin.forward(st);
  EXPECT_NEAR(v.vx, 0.0, 1e-9);
  EXPECT_NEAR(v.vy, 0.0, 1e-9);
  EXPECT_NEAR(v.wz, 0.0, 1e-9);
}

// 끝 넘김 막기 1) 지금 170° 에서 -170° 방향으로 → 접으면 -170° 지만 모터는 340° 를 빙 돌게 됨
//   → 10° 를 보고 바퀴를 거꾸로 (조향은 160° 만 돌고, 그동안 바퀴는 조향 먼저 규칙으로 멈춰 있음)
TEST(SteerLimit, AvoidsWrapAroundAtPlusPi)
{
  const double now = 170.0 * kPi / 180.0;
  const double dir = -170.0 * kPi / 180.0;
  // cosine_scale 끔: 켜 두면 "아직 170° 에 있는 동안" 은 앞으로 굴러도 -170° 와 20° 차이라 + 로 나옴 (그것도 맞는 값)
  auto out = make().inverse({0.1 * std::cos(dir), 0.1 * std::sin(dir), 0.0}, facing(now), false);
  for (const auto & m : out) {
    EXPECT_NEAR(m.steer_angle, 10.0 * kPi / 180.0, 1e-9);
    EXPECT_LE(std::fabs(m.steer_angle), kPi);
    EXPECT_LT(m.wheel_speed, 0.0);   // 10° 를 보고 뒤로 = -170° 로 앞으로
  }
}

// 끝 넘김 막기 2) 반대쪽도: 지금 -170° 에서 170° 방향 → -10° 를 보고 거꾸로
TEST(SteerLimit, AvoidsWrapAroundAtMinusPi)
{
  const double now = -170.0 * kPi / 180.0;
  const double dir = 170.0 * kPi / 180.0;
  auto out = make().inverse({0.1 * std::cos(dir), 0.1 * std::sin(dir), 0.0}, facing(now), false);
  for (const auto & m : out) {
    EXPECT_NEAR(m.steer_angle, -10.0 * kPi / 180.0, 1e-9);
    EXPECT_LT(m.wheel_speed, 0.0);
  }
}

// 끝 넘김 막기 3) 끝 근처라도 넘지 않으면 그대로 (지금 170° → 175° 방향은 175° 로, 앞으로)
TEST(SteerLimit, KeepsNearbyTargetInsideRange)
{
  const double now = 170.0 * kPi / 180.0;
  const double dir = 175.0 * kPi / 180.0;
  auto out = make().inverse({0.1 * std::cos(dir), 0.1 * std::sin(dir), 0.0}, facing(now), false);
  for (const auto & m : out) {
    EXPECT_NEAR(m.steer_angle, dir, 1e-9);
    EXPECT_NEAR(m.wheel_speed, 0.1 / kR, 1e-9);
  }
}

// 끝 넘김 막기 4) 어떤 지금 각도 · 어떤 방향이든 명령 각도는 언제나 -π ~ π 안 (모터가 갈 수 있는 곳)
//   그리고 (각도, 속도) 가 가리키는 바닥 속도는 원래 명령과 같음 (뒤집어도 같은 움직임)
TEST(SteerLimit, AlwaysInsideRangeAndSameMotion)
{
  for (int a = -180; a <= 180; a += 15) {
    for (int d = -180; d < 180; d += 20) {
      const double now = a * kPi / 180.0;
      const double dir = d * kPi / 180.0;
      auto out = make().inverse({0.1 * std::cos(dir), 0.1 * std::sin(dir), 0.0}, facing(now), false);
      for (const auto & m : out) {
        ASSERT_LE(std::fabs(m.steer_angle), kPi + 1e-9) << "now " << a << " dir " << d;
        const double v = m.wheel_speed * kR;
        EXPECT_NEAR(v * std::cos(m.steer_angle), 0.1 * std::cos(dir), 1e-9);
        EXPECT_NEAR(v * std::sin(m.steer_angle), 0.1 * std::sin(dir), 1e-9);
      }
    }
  }
}

// ============================================================================ ③ 운전석
// "명령을 보내던 노드가 멈춰도 바퀴가 계속 도는" 일이 없는지가 1)~3)

using swerve_controller::BodyTwist;
using swerve_controller::ModuleGeometry;
using swerve_controller::ModuleState;
using swerve_controller::SwerveDrive;
using swerve_controller::SwerveDriveConfig;
using swerve_controller::SwerveKinematics;

namespace
{
constexpr double kDt = 0.01;   // 컨트롤러 100 Hz

SwerveDrive make_drive(SwerveDriveConfig cfg = {})
{
  return SwerveDrive(
    SwerveKinematics({{0.112, 0.064, 0.0}, {0.112, -0.064, 0.0}, {-0.112, 0.064, 0.0}, {-0.112, -0.064, 0.0}}, kR),
    cfg);
}

// 바퀴가 명령을 그대로 따라간다고 치고(이상적인 모터) t0~t1 동안 돌림. 마지막 모듈 명령을 돌려줌
//   turn: 1 = 매 주기 DRIVE 차례가 옴(control_master 살아 있음), 0 = STOP 차례가 옴, -1 = 차례가 안 옴
std::vector<ModuleState> run(
  SwerveDrive & d, std::vector<ModuleState> & meas, double t0, double t1, const BodyTwist * cmd_every_step,
  int turn = 1)
{
  std::vector<ModuleState> out;
  for (double t = t0; t < t1 - 1e-9; t += kDt) {
    if (turn >= 0) {d.set_turn(turn == 1, 7, t);}
    if (cmd_every_step) {d.set_command(*cmd_every_step, t);}
    out = d.update(t, kDt, meas);
    for (std::size_t i = 0; i < meas.size(); ++i) {
      meas[i].steer_angle = out[i].steer_angle;
      meas[i].wheel_speed = out[i].wheel_speed;
    }
  }
  return out;
}
}  // namespace

// 1) 명령이 오면 가속 제한대로 올라가서 0.1 m/s = 바퀴 3.03 rad/s
TEST(SwerveDrive, FollowsFreshCommand)
{
  auto d = make_drive();
  std::vector<ModuleState> meas(4, ModuleState{0.0, 0.0});
  const BodyTwist fwd{0.1, 0.0, 0.0};
  auto out = run(d, meas, 0.0, 1.0, &fwd);   // 0.2 m/s² 로 0.5 s 면 도달
  for (const auto & m : out) {EXPECT_NEAR(m.wheel_speed, 0.1 / kR, 1e-6);}
}

// 2) ★ 명령이 끊기면 (Nav2 · Wi-Fi · 보내는 노드가 멈춤) 이 루프 안에서 세움
//    0.3 s(cmd_timeout) 뒤부터 가속 제한으로 0.5 s 동안 감속 → 끊긴 뒤 0.8 s 안에 0
TEST(SwerveDrive, StopsWhenCommandGoesStale)
{
  auto d = make_drive();
  std::vector<ModuleState> meas(4, ModuleState{0.0, 0.0});
  const BodyTwist fwd{0.1, 0.0, 0.0};
  run(d, meas, 0.0, 1.0, &fwd);                       // 1.0 s 에 마지막 명령
  auto out = run(d, meas, 1.0, 1.25, nullptr);        // 0.25 s: 아직 신선 → 그대로
  EXPECT_NEAR(out[0].wheel_speed, 0.1 / kR, 1e-6);
  out = run(d, meas, 1.25, 1.85, nullptr);            // 끊긴 뒤 0.85 s
  for (const auto & m : out) {EXPECT_NEAR(m.wheel_speed, 0.0, 1e-9);}
  EXPECT_FALSE(d.command_fresh(1.85));
}

// 3) 바퀴 차례가 아니면 (팔 차례 · 세우는 중 · 차례를 모름) 명령이 와도 세움
TEST(SwerveDrive, StopsWhenNotDriveTurn)
{
  auto d = make_drive();
  std::vector<ModuleState> meas(4, ModuleState{0.0, 0.0});
  const BodyTwist fwd{0.1, 0.0, 0.0};
  auto out = run(d, meas, 0.0, 1.0, &fwd, -1);   // 차례를 아직 못 받음 (기본 = 안 움직임)
  for (const auto & m : out) {EXPECT_NEAR(m.wheel_speed, 0.0, 1e-9);}

  run(d, meas, 1.0, 2.0, &fwd);
  out = run(d, meas, 2.0, 2.6, &fwd, 0);   // 팔 작업 들어옴 → control_master 가 STOP(STOPPING) 방송, 명령은 계속 오지만
  for (const auto & m : out) {EXPECT_NEAR(m.wheel_speed, 0.0, 1e-9);}
  EXPECT_EQ(d.applied_turn_id(), 7u);   // "번호 7 의 잠금을 반영했음" → control_master 가 이 답을 보고 팔을 시작
  EXPECT_FALSE(d.applied_drive());
}

// 3-2) ★ 차례 방송이 끊기면 (control_master 죽음 · 신호 끊김) 0.5 s 뒤 잠금 — 명령이 계속 와도
TEST(SwerveDrive, LocksWhenTurnGoesStale)
{
  auto d = make_drive();
  std::vector<ModuleState> meas(4, ModuleState{0.0, 0.0});
  const BodyTwist fwd{0.1, 0.0, 0.0};
  run(d, meas, 0.0, 1.0, &fwd);                       // DRIVE 차례 · 명령 둘 다 옴
  auto out = run(d, meas, 1.0, 1.4, &fwd, -1);        // 차례만 끊김, 0.4 s: 아직 신선 → 그대로
  EXPECT_NEAR(out[0].wheel_speed, 0.1 / kR, 1e-6);
  out = run(d, meas, 1.4, 2.1, &fwd, -1);             // 0.5 s 넘음 → 잠금 → 감속 0.5 s
  for (const auto & m : out) {EXPECT_NEAR(m.wheel_speed, 0.0, 1e-9);}
  EXPECT_FALSE(d.applied_drive());
}

// 4) NaN 이 오면 이전 명령까지 취소
TEST(SwerveDrive, NaNCancelsCommand)
{
  auto d = make_drive();
  EXPECT_TRUE(d.set_command({0.1, 0.0, 0.0}, 0.0));
  EXPECT_FALSE(d.set_command({NAN, 0.0, 0.0}, 0.05));
  EXPECT_FALSE(d.command_fresh(0.06));
}

// 5) 한계로 자르기 (방향은 유지): 1 m/s 대각선 → 0.1 m/s
TEST(SwerveDrive, ClampsToLimits)
{
  auto d = make_drive();
  std::vector<ModuleState> meas(4, ModuleState{M_PI / 4, 0.0});
  const BodyTwist diag{1.0, 1.0, 5.0};
  run(d, meas, 0.0, 3.0, &diag);
  EXPECT_NEAR(std::hypot(d.output().vx, d.output().vy), 0.10, 1e-6);
  EXPECT_NEAR(d.output().wz, 0.5, 1e-6);
}

// 6) 조향 먼저: 앞을 보고 있는데 옆으로 가라 → 조향 90°, 구동은 0 (조향이 다 돌 때까지)
TEST(SwerveDrive, SteersBeforeDriving)
{
  auto d = make_drive();
  std::vector<ModuleState> meas(4, ModuleState{0.0, 0.0});
  d.set_command({0.0, 0.1, 0.0}, 0.0);
  for (int k = 0; k < 30; ++k) {
    d.set_turn(true, 1, k * kDt);
    auto out = d.update(k * kDt, kDt, meas);   // 측정 조향은 0 에 머묾 (아직 덜 돎)
    for (const auto & m : out) {
      EXPECT_NEAR(std::fabs(m.steer_angle), M_PI / 2, 1e-9);
      EXPECT_NEAR(m.wheel_speed, 0.0, 1e-9);
    }
  }
}

// 7) odom: 1 s 동안 0.1 m/s 앞으로 → x = 0.1. 자이로가 있으면 회전은 자이로 값
TEST(SwerveDrive, OdomIntegratesAndUsesGyro)
{
  auto d = make_drive();
  std::vector<ModuleState> meas(4, ModuleState{0.0, 0.1 / kR});
  for (int k = 0; k < 100; ++k) {d.integrate_odom(meas, kDt, false, 0.0);}
  EXPECT_NEAR(d.pose().x, 0.1, 1e-9);
  EXPECT_NEAR(d.pose().y, 0.0, 1e-9);
  const auto v = d.integrate_odom(meas, kDt, true, 0.25);
  EXPECT_NEAR(v.wz, 0.25, 1e-12);
  EXPECT_NEAR(d.pose().yaw, 0.0025, 1e-9);
}

// ============================================================================ ② 자이로

using swerve_controller::GyroYaw;

// 1) 멈춰 있는 2 s 동안 평균(0.01)을 바이어스로 잡고, 그 뒤엔 빼서 돌려줌
TEST(GyroYaw, CalibratesBiasWhileStill)
{
  GyroYaw g(2.0, 0.2);
  for (int i = 0; i <= 200; ++i) {g.add_sample(i * 0.01, 0.01, true);}
  EXPECT_TRUE(g.calibrated());
  EXPECT_NEAR(g.bias(), 0.01, 1e-9);
  g.add_sample(2.01, 0.51, false);   // 실제 0.5 rad/s 로 도는 중
  EXPECT_TRUE(g.ready(2.05));
  EXPECT_NEAR(g.rate(), 0.5, 1e-9);
}

// 2) 바이어스를 잡기 전이나 IMU 가 끊기면 ready() = false → control_master 는 바퀴 ω 를 씀
TEST(GyroYaw, NotReadyBeforeCalibrationOrWhenStale)
{
  GyroYaw g(2.0, 0.2);
  g.add_sample(0.0, 0.0, true);
  EXPECT_FALSE(g.ready(0.0));
  for (int i = 0; i <= 200; ++i) {g.add_sample(i * 0.01, 0.0, true);}
  EXPECT_TRUE(g.ready(2.1));
  EXPECT_FALSE(g.ready(2.5));   // 마지막 샘플 2.0 s 뒤 0.5 s → 끊김
}

// 3) 바이어스 잡는 중에 움직이면 처음부터 다시
TEST(GyroYaw, MovementRestartsCalibration)
{
  GyroYaw g(1.0, 0.2);
  for (int i = 0; i < 80; ++i) {g.add_sample(i * 0.01, 0.0, true);}
  g.add_sample(0.80, 0.3, false);
  for (int i = 81; i < 150; ++i) {g.add_sample(i * 0.01, 0.0, true);}
  EXPECT_FALSE(g.calibrated());   // 0.81 부터 0.68 s 밖에 안 됨
}

// 4) OpenCR 플러그인이 IMU 고장 때 보내는 NaN 은 무시 → 끊긴 것과 같게 처리 (odom 이 NaN 으로 망가지지 않음)
TEST(GyroYaw, IgnoresNaNSamples)
{
  GyroYaw g(2.0, 0.2);
  for (int i = 0; i <= 200; ++i) {g.add_sample(i * 0.01, 0.0, true);}
  ASSERT_TRUE(g.ready(2.0));
  for (int i = 0; i < 50; ++i) {g.add_sample(2.0 + i * 0.01, NAN, true);}
  EXPECT_TRUE(std::isfinite(g.bias()));
  EXPECT_TRUE(std::isfinite(g.rate()));
  EXPECT_FALSE(g.ready(2.5));   // 마지막 진짜 샘플 2.0 s → 0.5 s 지남
}

// 5) 2026-11-01 멈춘 직후(still_hold_s 전)와 아직 도는 중(track_gate 밖)엔 바이어스를 안 따라감
TEST(GyroYaw, TracksBiasOnlyAfterHoldAndInsideGate)
{
  GyroYaw g(1.0, 0.2, 0.002, 1.0, 0.0087);
  for (int i = 0; i <= 100; ++i) {g.add_sample(i * 0.01, 0.0, true);}
  ASSERT_TRUE(g.calibrated());
  g.add_sample(1.01, 0.3, false);                                          // 움직임 → 멈춤 다시 셈
  for (int i = 0; i < 90; ++i) {g.add_sample(1.02 + i * 0.01, 0.005, true);}   // 멈춘 지 0.9 s: 아직 안 따라감
  EXPECT_NEAR(g.bias(), 0.0, 1e-12);
  for (int i = 0; i < 100; ++i) {g.add_sample(1.92 + i * 0.01, 0.02, true);}   // 1 s 넘었지만 0.02 rad/s = 아직 도는 중 (문 밖)
  EXPECT_NEAR(g.bias(), 0.0, 1e-12);
  for (int i = 0; i < 100; ++i) {g.add_sample(2.92 + i * 0.01, 0.005, true);}  // 문 안 → 천천히 따라감
  EXPECT_GT(g.bias(), 0.0005);
  EXPECT_LT(g.bias(), 0.005);
}

// 6) 2026-11-01 5 rad/s 넘는 순간 튐은 버림 (바이어스 · 회전 속도에 안 들어감), 계속 튀면 ready() = false
TEST(GyroYaw, DropsSpikes)
{
  GyroYaw g(1.0, 0.2);
  for (int i = 0; i <= 100; ++i) {g.add_sample(i * 0.01, 0.0, true);}
  g.add_sample(1.01, 0.1, false);
  g.add_sample(1.02, 40.0, false);
  EXPECT_NEAR(g.rate(), 0.1, 1e-12);
  EXPECT_TRUE(g.ready(1.05));
  for (int i = 0; i < 50; ++i) {g.add_sample(1.03 + i * 0.01, -40.0, false);}
  EXPECT_FALSE(g.ready(1.5));
}

// 7) 2026-11-02 GPT 검토 재현: 튐 샘플과 함께 온 '움직임' 은 버리지 않음
TEST(GyroYaw, InvalidSampleStillCountsMotion)
{
  GyroYaw g(1.0, 0.2, 0.002, 1.0, 0.0087);
  for (int i = 0; i <= 100; ++i) {g.add_sample(i * 0.01, 0.0, true);}
  ASSERT_TRUE(g.calibrated());
  g.add_sample(1.10, 40.0, false);   // 움직이는 중 튐
  g.add_sample(1.11, 0.005, true);   // 멈춘 지 0.01 s → 아직 따라가면 안 됨
  EXPECT_NEAR(g.bias(), 0.0, 1e-12);
}

// 8) 2026-11-02 GPT 검토 재현: 켤 때 재는 중 움직임(튐 값) · 긴 공백이 끼면 처음부터
TEST(GyroYaw, CalibrationRestartsOnInvalidMotionOrGap)
{
  GyroYaw g(5.0, 0.2);
  g.add_sample(0.0, 0.0, true);
  g.add_sample(4.9, 40.0, false);
  g.add_sample(5.0, 0.003, true);
  EXPECT_FALSE(g.calibrated());
  GyroYaw h(5.0, 0.2);
  h.add_sample(0.0, 0.0, true);
  h.add_sample(5.0, 0.003, true);   // 5 s 공백 (IMU 끊김)
  EXPECT_FALSE(h.calibrated());
}

// ---------- 실물 영점으로 만든 모듈별 범위 (hardware_map_drive_only.yaml · drive_only.yaml 과 같은 값) ----------
// 범위 = robot.launch.py steer_ranges() 와 같은 식: 관절 = scale × (눈금 − zero) × 2π/4096, 눈금 20 ~ 4075, 조향 scale −1.
// 영점 11 · 12 · 13 · 14 = 3066 · 1552 · 2577 · 3589 (새 전진 방향) → 범위 [deg] fl −88.7~+267.7 · fr −221.7~+134.6 ·
//   rl −131.7~+224.7 · rr −42.7~+313.7. 14 번(rr)은 시계 쪽이 약 −43° 까지만.
namespace
{
std::vector<swerve_controller::SteeringRange> ranges_from_zero(const int (&zero)[4])
{
  std::vector<swerve_controller::SteeringRange> r;
  const double k = 2.0 * kPi / 4096.0;
  for (int z : zero) {r.push_back({-(4075 - z) * k, (z - 20) * k});}
  return r;
}
// 모듈 위치 · offset: 새 전진 방향, 순서 fl · fr · rl · rr = 관절 이름 순서 (10-29 controllers.yaml 값. 11-01 실측 반영값은
//   x −0.070 / +0.060 — 아래 검사는 기하에 무관한 식이라 시험 고정값은 그대로 둠, 2026-11-02 검토)
const double kRealGeo[4][3] = {{-0.070, 0.112, 0.0269}, {0.060, 0.112, 0.0269},   // 11-01 실측 (앞 +0.060 · 뒤 −0.070)
                               {-0.070, -0.112, -0.0269}, {0.060, -0.112, -0.0269}};
const int kRealZero[4] = {3066, 1552, 2577, 3589};
std::vector<ModuleGeometry> real_modules()
{
  std::vector<ModuleGeometry> m;
  for (const auto & g : kRealGeo) {m.push_back({g[0], g[1], g[2]});}
  return m;
}
SwerveKinematics real_geometry(const int (&zero)[4])
{
  return SwerveKinematics(real_modules(), kR, kPi, ranges_from_zero(zero));
}
// 바퀴가 만든 바닥 속도가 명령의 모듈 속도와 같은지 (offset 보상 포함: 바퀴 선속도 = 모듈 속도 − ω·offset)
void expect_realizes(const SwerveKinematics & k, const BodyTwist & c, const std::vector<ModuleState> & out,
                     const std::vector<swerve_controller::SteeringRange> & r)
{
  const auto & xy = kRealGeo;
  for (int i = 0; i < 4; ++i) {
    EXPECT_GE(out[i].steer_angle, r[i].min - 1e-12) << "module " << i;
    EXPECT_LE(out[i].steer_angle, r[i].max + 1e-12) << "module " << i;
    const double lin = out[i].wheel_speed * kR + c.wz * xy[i][2];
    EXPECT_NEAR(lin * std::cos(out[i].steer_angle), c.vx - c.wz * xy[i][1], 1e-9) << "module " << i;
    EXPECT_NEAR(lin * std::sin(out[i].steer_angle), c.vy + c.wz * xy[i][0], 1e-9) << "module " << i;
  }
  (void)k;
}
}  // namespace

// 10) 전후진 · 좌우 · 대각선 4 · 제자리 회전 양쪽 · 이동+회전: 모두 범위 안 조향 + 정 · 역회전으로 만들 수 있음 (자르지 않음)
TEST(SwerveRanges, RealZeroAllMotionsFeasibleInsideRange)
{
  const std::vector<BodyTwist> motions = {
    {0.05, 0, 0}, {-0.05, 0, 0}, {0, 0.05, 0}, {0, -0.05, 0},
    {0.035, 0.035, 0}, {0.035, -0.035, 0}, {-0.035, 0.035, 0}, {-0.035, -0.035, 0},
    {0, 0, 0.3}, {0, 0, -0.3}, {0.05, 0, 0.3}, {0, 0.05, -0.3}};
  const auto k = real_geometry(kRealZero);
  const auto r = ranges_from_zero(kRealZero);
  auto now = facing(0.0);
  for (const auto & c : motions) {      // 앞 명령의 조향에서 이어서 (시험 순서 그대로)
    bool ok = false;
    auto out = k.inverse(c, now, false, &ok);
    ASSERT_TRUE(ok) << c.vx << "," << c.vy << "," << c.wz;
    expect_realizes(k, c, out, r);
    for (int i = 0; i < 4; ++i) {now[i].steer_angle = out[i].steer_angle;}
  }
}

// 11) 오른쪽(−y) 이동: −90° 가 범위 밖인 14 번(rr, −42.7° 까지) · 11 번(fl, −88.7° 까지) → +90° + 바퀴 역회전.
//     12 · 13 은 −90° 그대로. 범위 없는 예전 계산은 넷 다 −90° 를 줌
TEST(SwerveRanges, RightwardMotionUsesReverseWheelWhereMinus90IsOutOfRange)
{
  bool ok = false;
  auto out = real_geometry(kRealZero).inverse({0, -0.05, 0}, facing(0.0), false, &ok);
  ASSERT_TRUE(ok);
  for (int i : {0, 3}) {
    EXPECT_NEAR(out[i].steer_angle, kPi / 2, 1e-9) << i;
    EXPECT_LT(out[i].wheel_speed, 0.0) << i;
  }
  for (int i : {1, 2}) {
    EXPECT_NEAR(out[i].steer_angle, -kPi / 2, 1e-9) << i;
    EXPECT_GT(out[i].wheel_speed, 0.0) << i;
  }
  const auto unbounded = SwerveKinematics(real_modules(), kR).inverse({0, -0.05, 0}, facing(0.0), false);
  EXPECT_NEAR(unbounded[3].steer_angle, -kPi / 2, 1e-9);   // 이 각도는 14 번이 못 감 (−43° 까지) — 범위가 필요한 이유
}

// 12) 가속 제한은 선속도 · 회전을 같은 비율로: 출발 중에도 vx:ω 비율(= 모듈 조향각)이 그대로, 각 한계는 넘지 않음
TEST(SlowAndSequential, AccelLimitKeepsTwistDirection)
{
  using swerve_controller::limit_accel;
  const BodyTwist target{0.05, 0.0, 0.3};
  BodyTwist v{0, 0, 0};
  for (int i = 0; i < 60; ++i) {
    const BodyTwist prev = v;
    v = limit_accel(v, target, 0.2, 0.8, 0.01);
    EXPECT_LE(std::hypot(v.vx - prev.vx, v.vy - prev.vy), 0.2 * 0.01 + 1e-12);
    EXPECT_LE(std::fabs(v.wz - prev.wz), 0.8 * 0.01 + 1e-12);
    if (v.wz > 0) {EXPECT_NEAR(v.vx / v.wz, target.vx / target.wz, 1e-9);}   // 방향 유지
  }
  EXPECT_NEAR(v.vx, 0.05, 1e-12);
  EXPECT_NEAR(v.wz, 0.3, 1e-12);
}

// ---------- 정지 후 바깥쪽 자세(조향 0°) 복귀 park_on_idle (실물 영점 · 기하 · 범위) ----------
namespace
{
constexpr double kSlew = 0.96;   // [rad/s] 조향 모터 프로파일 ≈ 55°/s 흉내
SwerveDrive park_drive(bool on, std::vector<swerve_controller::SteeringRange> r = ranges_from_zero(kRealZero))
{
  SwerveDriveConfig cfg;
  cfg.park_on_idle = on;
  return SwerveDrive(SwerveKinematics(real_modules(), kR, kPi, r), cfg);
}
// 한 주기: 명령(nullptr = 안 보냄) → update → 조향은 kSlew 로 따라가고 실측 조향 속도를 채움, 바퀴는 명령 그대로 (+ extra)
std::vector<ModuleState> step(SwerveDrive & d, std::vector<ModuleState> & meas, double t, const BodyTwist * cmd,
                              int turn = 1, double extra_wheel = 0.0)
{
  if (turn >= 0) {d.set_turn(turn == 1, 7, t);}
  if (cmd) {d.set_command(*cmd, t);}
  auto out = d.update(t, kDt, meas);
  for (std::size_t i = 0; i < meas.size(); ++i) {
    const double dlt = std::clamp(out[i].steer_angle - meas[i].steer_angle, -kSlew * kDt, kSlew * kDt);
    meas[i].steer_angle += dlt;
    meas[i].steer_rate = dlt / kDt;
    meas[i].wheel_speed = out[i].wheel_speed + extra_wheel;
  }
  return out;
}
std::vector<ModuleState> inward_fr()   // 바닥 시험 끝 자세: 12 번(fr) −180° (안쪽), 나머지 0°
{
  auto m = facing(0.0);
  m[1].steer_angle = -kPi;
  return m;
}
const BodyTwist kZero{0.0, 0.0, 0.0};
}  // namespace

// 13) 기본은 끔: 0 명령이 계속 와도 조향은 그 자리 (−180° 그대로)
TEST(ParkOnIdle, OffByDefault)
{
  auto d = park_drive(false);
  auto meas = inward_fr();
  std::vector<ModuleState> out;
  for (double t = 0.0; t < 3.0; t += kDt) {out = step(d, meas, t, &kZero);}
  EXPECT_NEAR(out[1].steer_angle, -kPi, 1e-12);
  EXPECT_FALSE(d.parking());
}

// 14) 켜면: 신선한 0 + 바퀴 정지가 1 s 이어진 뒤에만 네 조향 0°. 도는 동안 바퀴는 offset 보상만 → 몸 odom 0
TEST(ParkOnIdle, ParksOnlyAfterOneSecondFreshZeroAndStill)
{
  auto d = park_drive(true);
  auto meas = inward_fr();
  std::vector<ModuleState> out;
  double t = 0.0;
  for (; t < 0.99; t += kDt) {out = step(d, meas, t, &kZero);}
  EXPECT_FALSE(d.parking());
  EXPECT_NEAR(out[1].steer_angle, -kPi, 1e-12);
  for (; t < 1.03; t += kDt) {out = step(d, meas, t, &kZero);}
  ASSERT_TRUE(d.parking());
  for (const auto & m : out) {EXPECT_NEAR(m.steer_angle, 0.0, 1e-12);}
  const auto k = real_geometry(kRealZero);
  for (; t < 5.0; t += kDt) {
    const double rate = meas[1].steer_rate;
    out = step(d, meas, t, &kZero);
    EXPECT_NEAR(out[1].wheel_speed, k.roll_while_steering(1, rate), 1e-12);   // 구동 0 고정 아님: 보상만
    d.integrate_odom(meas, kDt, false, 0.0);
  }
  // 몸은 그 자리 (조향 출발 · 도착 한 주기씩만 보상이 한 박자 늦음 → 1 mm 안)
  EXPECT_LT(std::hypot(d.pose().x, d.pose().y), 1e-3);
  EXPECT_LT(std::fabs(d.pose().yaw), 0.01);
  EXPECT_NEAR(meas[1].steer_angle, 0.0, 1e-9);   // 180° / 0.96 rad/s ≈ 3.3 s 에 도착
  EXPECT_NEAR(out[1].wheel_speed, 0.0, 1e-9);
}

// 15) 명령 끊김 (0 을 보내다 멈춤) · 바퀴 차례 아님 · NaN: 복귀 시작 안 함, 조향 그 자리
TEST(ParkOnIdle, NoParkOnStaleCommandTurnLossOrNaN)
{
  {
    auto d = park_drive(true);
    auto meas = inward_fr();
    double t = 0.0;
    for (; t < 0.2; t += kDt) {step(d, meas, t, &kZero);}
    std::vector<ModuleState> out;
    for (; t < 3.0; t += kDt) {out = step(d, meas, t, nullptr);}
    EXPECT_FALSE(d.parking());
    EXPECT_NEAR(out[1].steer_angle, -kPi, 1e-12);
  }
  {
    auto d = park_drive(true);
    auto meas = inward_fr();
    std::vector<ModuleState> out;
    for (double t = 0.0; t < 3.0; t += kDt) {out = step(d, meas, t, &kZero, 0);}
    EXPECT_FALSE(d.parking());
    EXPECT_NEAR(out[1].steer_angle, -kPi, 1e-12);
  }
  {
    auto d = park_drive(true);
    auto meas = inward_fr();
    const BodyTwist bad{NAN, 0.0, 0.0};
    std::vector<ModuleState> out;
    for (double t = 0.0; t < 3.0; t += kDt) {out = step(d, meas, t, &bad);}
    EXPECT_FALSE(d.parking());
    EXPECT_NEAR(out[1].steer_angle, -kPi, 1e-12);
  }
}

// 16) 실측 바퀴가 아직 돌면(밀림 · 감속 중) 기다림. 멈춘 때부터 다시 1 s
TEST(ParkOnIdle, WaitsForMeasuredWheelStop)
{
  auto d = park_drive(true);
  auto meas = inward_fr();
  double t = 0.0;
  for (; t < 2.0; t += kDt) {step(d, meas, t, &kZero, 1, 0.2);}
  EXPECT_FALSE(d.parking());
  const double still_from = t;
  for (; t < still_from + 0.95; t += kDt) {step(d, meas, t, &kZero);}
  EXPECT_FALSE(d.parking());
  for (; t < still_from + 1.05; t += kDt) {step(d, meas, t, &kZero);}
  EXPECT_TRUE(d.parking());
}

// 17) 복귀 도중 새 주행 명령: 복귀를 멈추고 지금 각도에서 가까운 쪽 선택 + 조향 먼저 (정렬 전 구동 없음)
TEST(ParkOnIdle, NewCommandDuringParkTakesOver)
{
  auto d = park_drive(true);
  auto meas = inward_fr();
  double t = 0.0;
  for (; t < 2.3; t += kDt) {step(d, meas, t, &kZero);}   // 1 s 뒤 시작 → 약 1.2 rad 돌아 −1.9 rad 근처
  ASSERT_TRUE(d.parking());
  ASSERT_GT(meas[1].steer_angle, -kPi + 0.5);
  ASSERT_LT(meas[1].steer_angle, -kPi / 2);   // 아직 0 보다 −180° 에 가까움 → 전진은 −180° + 역회전을 고름
  const BodyTwist fwd{0.05, 0.0, 0.0};
  const auto k = real_geometry(kRealZero);
  std::vector<double> rate(4);
  for (int i = 0; i < 4; ++i) {rate[i] = meas[i].steer_rate;}
  auto out = step(d, meas, t, &fwd);
  EXPECT_FALSE(d.parking());
  EXPECT_NEAR(out[1].steer_angle, -kPi, 1e-9);
  for (int i = 0; i < 4; ++i) {   // 정렬 전: 네 바퀴 모두 보상만 (구동 없음)
    EXPECT_NEAR(out[i].wheel_speed, k.roll_while_steering(i, rate[i]), 1e-12) << i;
  }
  for (t += kDt; t < 6.0; t += kDt) {out = step(d, meas, t, &fwd);}
  EXPECT_NEAR(meas[1].steer_angle, -kPi, 1e-9);
  EXPECT_LT(out[1].wheel_speed, 0.0);   // 안쪽 자세로 역회전 전진
  EXPECT_GT(out[0].wheel_speed, 0.0);
}

// 18) 상태를 못 읽음(stop_hold) · 컨트롤러 다시 켬(reset): 복귀 취소, 다시 1 s 부터
TEST(ParkOnIdle, StopHoldAndResetCancelPark)
{
  auto d = park_drive(true);
  auto meas = inward_fr();
  double t = 0.0;
  for (; t < 1.5; t += kDt) {step(d, meas, t, &kZero);}
  ASSERT_TRUE(d.parking());
  const auto held = d.stop_hold({0.0, meas[1].steer_angle, 0.0, 0.0});
  EXPECT_FALSE(d.parking());
  EXPECT_NEAR(held[1].steer_angle, meas[1].steer_angle, 1e-12);
  for (const auto & m : held) {EXPECT_EQ(m.wheel_speed, 0.0);}
  const double from = t;
  std::vector<ModuleState> out;
  for (; t < from + 0.9; t += kDt) {out = step(d, meas, t, &kZero);}
  EXPECT_FALSE(d.parking());
  EXPECT_NEAR(out[1].steer_angle, meas[1].steer_angle, 1e-12);
  d.reset();
  EXPECT_FALSE(d.parking());
}

// 19) 0° 가 범위 밖인 모듈이 있으면 복귀 안 함 (자르지 않음)
TEST(ParkOnIdle, NotStartedWhenZeroOutsideRange)
{
  auto r = ranges_from_zero(kRealZero);
  r[3] = {0.2, 3.0};
  auto d = park_drive(true, r);
  auto meas = facing(0.5);
  std::vector<ModuleState> out;
  for (double t = 0.0; t < 3.0; t += kDt) {out = step(d, meas, t, &kZero);}
  EXPECT_FALSE(d.parking());
  EXPECT_NEAR(out[1].steer_angle, 0.5, 1e-12);
}

namespace {
SwerveKinematics coherent_geometry() {
  return SwerveKinematics(real_modules(), kR, kPi, ranges_from_zero(kRealZero), true);
}
BodyTwist heading(double deg) {
  return {.05*std::cos(deg*kPi/180), .05*std::sin(deg*kPi/180), 0};
}
}

TEST(CoherentPose, MixedInitialForwardAndReverseConvergeToOutward) {
  auto mixed = facing(0); mixed[1].steer_angle = -kPi;
  const auto legacy = real_geometry(kRealZero).inverse({.05,0,0}, mixed, false);
  EXPECT_NEAR(legacy[1].steer_angle, -kPi, 1e-9);
  for (double v : {.05, -.05}) {
    const auto out = coherent_geometry().inverse({v,0,0}, mixed, false);
    for (const auto & m : out) {EXPECT_NEAR(m.steer_angle, 0, 1e-9);}
    expect_realizes(coherent_geometry(), {v,0,0}, out, ranges_from_zero(kRealZero));
  }
}

TEST(CoherentPose, CardinalAndFeasibleDiagonalSharePhysicalAngle) {
  auto mixed=facing(0); mixed[1].steer_angle=-kPi; mixed[3].steer_angle=4;
  for (double deg : {0.,45.,90.,180.,225.,270.}) {
    const auto out=coherent_geometry().inverse(heading(deg),mixed,false);
    // Same physical line for every module: equal angles or exactly pi apart (outboard branch).
    for (const auto & m:out) {
      const double d=std::fabs(std::remainder(m.steer_angle-out[0].steer_angle, kPi));
      EXPECT_NEAR(d,0,1e-9);
    }
    expect_realizes(coherent_geometry(),heading(deg),out,ranges_from_zero(kRealZero));
  }
  // -45 cannot be shared: rr cannot reach -45, fr cannot reach +135.
  const auto out=coherent_geometry().inverse(heading(-45),mixed,false);
  EXPECT_GT(std::fabs(out[1].steer_angle-out[3].steer_angle),3.0);
  expect_realizes(coherent_geometry(),heading(-45),out,ranges_from_zero(kRealZero));
}

TEST(CoherentPose, FullCircleAndCompoundSweepAlwaysRealizeTwistWithinRealLimits) {
  auto k=coherent_geometry(); auto previous=facing(0); auto measured=facing(0);
  measured[1].steer_angle=-kPi; measured[2].steer_angle=3;
  for (double wz : {0., .3, -.3}) {
    for (int d=-1800;d<=1800;++d) {
      auto c=heading(d*.1); c.wz=wz; bool feasible=false;
      const auto out=k.inverse(c,measured,false,&feasible,previous);
      ASSERT_TRUE(feasible); expect_realizes(k,c,out,ranges_from_zero(kRealZero));
      previous=out;
    }
  }
  for (double wz : {.3,-.3}) {
    auto out=k.inverse({0,0,wz},measured,false);
    expect_realizes(k,{0,0,wz},out,ranges_from_zero(kRealZero));
    EXPECT_GT(std::fabs(out[0].steer_angle-out[1].steer_angle),.1);
  }
}

TEST(CoherentPose, LimitNoiseDoesNotFlipBackAndForth) {
  auto k=coherent_geometry(); auto m=facing(0);
  auto prev=k.inverse(heading(-41),m,false);
  // Enter common pose from the interior, then leave at the hard limit once.
  prev=k.inverse(heading(-38),m,false,nullptr,prev);
  prev=k.inverse(heading(-43),m,false,nullptr,prev);
  const double branch=prev[3].steer_angle;
  for (int i=0;i<100;++i) {
    auto c=heading(i%2 ? -42.5 : -43.0);
    auto out=k.inverse(c,m,false,nullptr,prev);
    EXPECT_LT(std::fabs(out[3].steer_angle-branch),.02);
    expect_realizes(k,c,out,ranges_from_zero(kRealZero)); prev=out;
  }
  auto recovered=k.inverse(heading(-38),m,false,nullptr,prev);
  for (const auto & x:recovered) {EXPECT_NEAR(x.steer_angle,-38*kPi/180,1e-9);}
}

TEST(CoherentPose, DriveGatesUntilAllAlignedKeepsOffsetAndHandlesNewCommand) {
  swerve_controller::SwerveDriveConfig cfg; cfg.park_on_idle=true;
  swerve_controller::SwerveDrive drive(coherent_geometry(),cfg);
  auto m=facing(0); m[1].steer_angle=-kPi;
  for (auto & x:m) {x.steer_rate=.2;}
  drive.set_turn(true,1,0); drive.set_command({.05,0,0},0);
  auto out=drive.update(0,.01,m);
  for (int i=0;i<4;++i) {
    EXPECT_NEAR(out[i].steer_angle,0,1e-9);
    EXPECT_NEAR(out[i].wheel_speed,-.2*kRealGeo[i][2]/kR,1e-9);
  }
  drive.set_command({0,.05,0},.01); drive.set_turn(true,1,.01);
  out=drive.update(.01,.01,m);
  // Sideways: parallel wheels, fr/rl on the -90 branch so every wheel stays outboard (2026-10-26)
  const double side[4]={kPi/2,-kPi/2,-kPi/2,kPi/2};
  for (int i=0;i<4;++i) {EXPECT_NEAR(out[i].steer_angle,side[i],1e-9);}
  // A timeout during alignment holds measured pose, without starting park or rolling compensation.
  out=drive.update(1,.01,m);
  for (int i=0;i<4;++i) {
    EXPECT_NEAR(out[i].steer_angle,m[i].steer_angle,1e-9); EXPECT_DOUBLE_EQ(out[i].wheel_speed,0);
  }
  EXPECT_FALSE(drive.parking());
  drive.set_command({NAN,0,0},1.01); out=drive.update(1.01,.01,m);
  for (int i=0;i<4;++i) {EXPECT_DOUBLE_EQ(out[i].steer_angle,m[i].steer_angle);}
}

TEST(CoherentPose, ParkFreshZeroNewCommandAndStopHold) {
  swerve_controller::SwerveDriveConfig cfg; cfg.park_on_idle=true;
  swerve_controller::SwerveDrive drive(coherent_geometry(),cfg);
  auto m=facing(0); m[1].steer_angle=-kPi;
  for (int i=0;i<=110;++i) {
    double t=i*.01; drive.set_turn(true,1,t); drive.set_command({0,0,0},t);
    drive.update(t,.01,m);
  }
  ASSERT_TRUE(drive.parking());
  m[1].steer_angle=-2.5;
  drive.set_turn(true,1,1.11); drive.set_command({.05,0,0},1.11);
  auto out=drive.update(1.11,.01,m); EXPECT_FALSE(drive.parking());
  for (const auto & x:out) {EXPECT_NEAR(x.steer_angle,0,1e-9); EXPECT_DOUBLE_EQ(x.wheel_speed,0);}
  out=drive.stop_hold({0,-2.5,0,0});
  EXPECT_FALSE(drive.parking()); EXPECT_DOUBLE_EQ(out[1].steer_angle,-2.5);
  drive.reset(); out=drive.update(3,.01,m);
  EXPECT_DOUBLE_EQ(out[1].steer_angle,-2.5);
}

TEST(CoherentPose, TinyYawNoisePreservesTranslationBranches) {
  auto k=coherent_geometry(); auto m=facing(0);
  auto previous=k.inverse({0,-.05,0},m,false);
  for (double wz : {-1e-6,1e-6,-1e-5,1e-5,0.}) {
    const BodyTwist c{0,-.05,wz};
    auto out=k.inverse(c,m,false,nullptr,previous);
    for (int i=0;i<4;++i) {EXPECT_LT(std::fabs(out[i].steer_angle-previous[i].steer_angle),.001);}
    expect_realizes(k,c,out,ranges_from_zero(kRealZero));previous=out;
  }
}

TEST(CoherentPose, SameNewCommandIndependentOfEveryInitialPhysicalPose) {
  auto k=coherent_geometry(); const auto ranges=ranges_from_zero(kRealZero);
  std::vector<std::vector<ModuleState>> starts(5,facing(0));
  for (int i=0;i<4;++i) {
    starts[1][i].steer_angle=ranges[i].min+.001;
    starts[2][i].steer_angle=ranges[i].max-.001;
    starts[3][i].steer_angle=i%2 ? ranges[i].min+.001 : ranges[i].max-.001;
  }
  starts[4][1].steer_angle=-kPi;
  for (double wz : {0.,.3,-.3}) {
    for (int deg=-180;deg<=180;++deg) {
      auto cmd=heading(deg);cmd.wz=wz;
      const auto reference=k.inverse(cmd,starts[0],false);
      for (const auto & initial:starts) {
        const auto out=k.inverse(cmd,initial,false);
        for (int i=0;i<4;++i) {
          EXPECT_NEAR(out[i].steer_angle,reference[i].steer_angle,1e-9);
          EXPECT_NEAR(out[i].wheel_speed,reference[i].wheel_speed,1e-9);
        }
      }
    }
  }
}

TEST(CoherentPose, EveryPhysicalBoundaryRejectsRepeatedBranchChatter) {
  auto k=coherent_geometry(); const auto ranges=ranges_from_zero(kRealZero);
  for (const auto & r:ranges) {
    for (double edge:{r.min,r.max}) {
      const double deg=edge*180/kPi;
      auto previous=k.inverse(heading(deg),facing(0),false);
      int switches[4]={0,0,0,0};
      for (int sample=0;sample<100;++sample) {
        const auto cmd=heading(deg+(sample%2 ? -.1 : .1));
        auto out=k.inverse(cmd,facing(0),false,nullptr,previous);
        for (int i=0;i<4;++i) {
          if (std::fabs(out[i].steer_angle-previous[i].steer_angle)>kPi/2) {++switches[i];}
        }
        expect_realizes(k,cmd,out,ranges);previous=out;
      }
      for (int count:switches) {EXPECT_LE(count,1) << "boundary degrees " << deg;}
    }
  }
}


// 2026-10-26: sideways driving keeps every wheel outboard (no wheel tucked under the deck)
TEST(CoherentPose, SidewaysKeepsEveryWheelOutboard) {
  auto k=coherent_geometry(); const auto mods=real_modules();
  for (double deg : {90., -90., 0., 180., 45., -135.}) {
    const auto out=k.inverse(heading(deg),facing(0),false);
    expect_realizes(k,heading(deg),out,ranges_from_zero(kRealZero));
    for (std::size_t i=0;i<mods.size();++i) {
      const double a=out[i].steer_angle;
      const double o=-mods[i].offset*std::sin(a)*mods[i].x+mods[i].offset*std::cos(a)*mods[i].y;
      const double band=std::fabs(mods[i].offset)*std::hypot(mods[i].x,mods[i].y)*std::sin(10.0*kPi/180.0);
      EXPECT_GT(o,-band-1e-12) << "deg " << deg << " module " << i << " angle " << a;   // near-tie keeps the old choice
    }
  }
  // -45 / 135 line: rr cannot reach -45, so its +135 pose is the only feasible one (old documented exception)
  for (double deg : {-45., 135.}) {
    const auto out=k.inverse(heading(deg),facing(0),false);
    expect_realizes(k,heading(deg),out,ranges_from_zero(kRealZero));
    EXPECT_NEAR(out[3].steer_angle, 3*kPi/4, 1e-9);
  }
}

// 2026-10-26 실물 floor_max_050: 범위 끝 여유 0(최대 시험)에서 rr 이 끝에 붙은 바깥쪽 315° 를 골라 200° 넘게 돌았음
TEST(CoherentPose, OutwardIgnoredWhenItsBranchHugsRangeEnd) {
  const SwerveKinematics k(real_modules(), kR, kPi, ranges_from_zero(kRealZero), true, 0.0);
  for (double deg : {135., -45.}) {
    const auto out = k.inverse(heading(deg), facing(0), false);
    expect_realizes(k, heading(deg), out, ranges_from_zero(kRealZero));
    EXPECT_NEAR(out[3].steer_angle, 3*kPi/4, 1e-9) << "deg " << deg;
  }
}

// 2026-10-26: 멈춤(바깥쪽 선호)↔이동(분기 유지) 경계에서 분기가 오락가락해 출발 못 하던 것 (모의 실행에서 발견).
// 조향이 2.4 rad/s 로 목표를 따라가는 간단한 폐루프: 정지에서 어느 방향이든 1.5 s 안에 출발하고, 이어지는 전환도 멈추지 않음.
TEST(CoherentPose, ClosedLoopStartsAndChainsWithoutBranchChatter) {
  const SwerveKinematics k(real_modules(), kR, kPi, ranges_from_zero(kRealZero), true, 0.0);
  for (double deg : {0., 45., 90., 135., 180., -135., -90., -45.}) {
    swerve_controller::SwerveDriveConfig cfg; cfg.blend_steer_drive = true; cfg.blend_stall_s = 1.0; cfg.blend_margin = 0.7;
    swerve_controller::SwerveDrive d(k, cfg);
    auto m = facing(0);
    double t = 0.0;
    auto step_to = [&](const BodyTwist & c, double secs, const char * label) {
      int flips = 0; double moved = 0.0;
      std::vector<ModuleState> last;
      for (double e = 0.0; e < secs; e += .01, t += .01) {
        d.set_turn(true, 1, t); d.set_command(c, t);
        const auto out = d.update(t, .01, m);
        if (!last.empty()) {
          for (int i = 0; i < 4; ++i) {if (std::fabs(out[i].steer_angle - last[i].steer_angle) > kPi / 2) {++flips;}}
        }
        last = out;
        for (int i = 0; i < 4; ++i) {   // 조향 2.4 rad/s 로 따라감, 바퀴는 명령 그대로
          const double err = out[i].steer_angle - m[i].steer_angle;
          const double s = std::clamp(err, -2.4 * .01, 2.4 * .01);
          m[i].steer_angle += s; m[i].steer_rate = s / .01; m[i].wheel_speed = out[i].wheel_speed;
        }
        if (std::fabs(out[0].wheel_speed) > 1.0) {moved += .01;}
      }
      EXPECT_LE(flips, 1) << label << " deg " << deg;
      return moved;
    };
    EXPECT_GT(step_to(heading(deg), 1.5, "start"), 0.3) << "start deg " << deg;
    // 이동 중 45° 전환 두 번: 각 1.5 s 중 대부분 구동
    EXPECT_GT(step_to(heading(deg + 45), 1.5, "chain1"), 1.0) << "chain1 deg " << deg;
    // 135° / −45° 줄: 이 범위에서 fr 은 +135° (최대 134.6°), rr 은 −45° (최소 −42.7°) 를 못 감 → 한 모듈은 반드시 반대 분기로 돎
    const double d2 = std::remainder(deg + 90 - 135, 180.0);
    EXPECT_GT(step_to(heading(deg + 90), 1.5, "chain2"), std::fabs(d2) < 1e-9 ? 0.3 : 1.0) << "chain2 deg " << deg;
  }
}

// 2026-10-26 실물(최대 시험, 끝 여유 0): rr 범위 −44.5° 라 −45° 대각에서 135° 로 177° 를 돌며 전체 정지.
// steer_end_slack(기본 0)을 1° 로 켜면 끝 각도로 보내고 바퀴 속도 × cos(0.5°). 범위 밖 각도는 절대 안 보냄.
TEST(CoherentPose, EndSlackSendsNearlyReachableHeadingToRangeEnd) {
  const double d = kPi / 180;
  const std::vector<swerve_controller::SteeringRange> r = {{-90.4*d, 269.5*d}, {-223.5*d, 136.4*d}, {-133.4*d, 226.5*d}, {-44.5*d, 315.4*d}};
  const SwerveKinematics strict(real_modules(), kR, kPi, r, true, 0.0);
  const SwerveKinematics slack(real_modules(), kR, kPi, r, true, 0.0, 1.0 * d);
  const auto a = strict.inverse(heading(-45), facing(0), false);
  EXPECT_NEAR(a[3].steer_angle, 135 * d, 1e-9);
  const auto b = slack.inverse(heading(-45), facing(0), false);
  for (int i = 0; i < 3; ++i) {EXPECT_NEAR(b[i].steer_angle, -45 * d, 1e-9);}
  EXPECT_NEAR(b[3].steer_angle, -44.5 * d, 1e-9);
  EXPECT_NEAR(b[3].wheel_speed, .05 / kR * std::cos(.5 * d), 1e-9);
  // 2° 넘게 모자라면 그대로 반대 분기 (범위를 넓히지 않음)
  const auto c = slack.inverse(heading(-47), facing(0), false);
  EXPECT_NEAR(c[3].steer_angle, 133 * d, 1e-9);
  for (const auto & x : {a, b, c}) {for (int i = 0; i < 4; ++i) {
    EXPECT_GE(x[i].steer_angle, r[i].min - 1e-12); EXPECT_LE(x[i].steer_angle, r[i].max + 1e-12);}}
  EXPECT_THROW(SwerveKinematics(real_modules(), kR, kPi, r, true, 0.0, 3.0 * d), std::invalid_argument);
}

// 2026-10-28 돌아야 할 각도 비용 (steer_travel_weight): 정면 유지 전방향 "오른쪽 → 앞오른쪽" 에서
//   10-26 실물 · 10-28 조향 속도 모의 모두 fl 이 269.5° → −45° (314°) 를 돌아 그 구간을 거의 못 감 (진행률 0.0~0.07)
TEST(CoherentPose, TravelWeightAvoidsWrapAroundSwingAtRangeEnd) {
  const double d = kPi / 180;
  const std::vector<swerve_controller::SteeringRange> r = {{-90.439*d, 269.473*d}, {-223.506*d, 136.406*d}, {-133.418*d, 170.0*d}, {-44.473*d, 315.439*d}};
  std::vector<ModuleState> now = {{269.473*d, 0}, {90*d, 0}, {90*d, 0}, {270*d, 0}};   // 오른쪽으로 달리던 자세
  auto swing = [&](const std::vector<ModuleState> & out) {
      double m = 0.0;
      for (int i = 0; i < 4; ++i) {m = std::max(m, std::fabs(out[i].steer_angle - now[i].steer_angle));}
      return m;
    };
  SwerveKinematics old_rule(real_modules(), kR, kPi, r, true, 0.0, 1.0 * d);
  const auto a = old_rule.inverse(heading(-45), now, false, nullptr, now, false);
  EXPECT_NEAR(a[0].steer_angle, -45 * d, 1e-9);            // 예전: fl 314° 회전
  EXPECT_GT(swing(a), 300 * d);
  SwerveKinematics travel(real_modules(), kR, kPi, r, true, 0.0, 1.0 * d);
  travel.set_travel_weight(1.0);
  const auto b = travel.inverse(heading(-45), now, false, nullptr, now, false);
  EXPECT_LT(swing(b), 181 * d);                             // 어느 바퀴도 반 바퀴 넘게 안 돎
  expect_realizes(travel, heading(-45), b, r);              // 그래도 명령한 몸 속도는 그대로
  EXPECT_THROW(travel.set_travel_weight(-1.0), std::invalid_argument);
  EXPECT_THROW(travel.set_travel_weight(11.0), std::invalid_argument);
  // 가중치 0 = 예전과 모든 출력이 같음 (전방향 · 여러 시작 자세)
  SwerveKinematics zero(real_modules(), kR, kPi, r, true, 0.0, 1.0 * d);
  zero.set_travel_weight(0.0);
  for (int deg = -180; deg < 180; deg += 15) {
    for (const auto & start : {facing(0), now, facing(1.0)}) {
      const auto p = old_rule.inverse(heading(deg), start, false, nullptr, start, false);
      const auto q = zero.inverse(heading(deg), start, false, nullptr, start, false);
      for (int i = 0; i < 4; ++i) {
        EXPECT_EQ(p[i].steer_angle, q[i].steer_angle);
        EXPECT_EQ(p[i].wheel_speed, q[i].wheel_speed);
      }
    }
  }
}

// 2026-10-28 정면 유지 되먹임 (heading_hold_kp): 이동 중 차체를 한쪽으로 미는 외란(+0.05 rad/s, 자이로에 보임)
//   끄면 3 s 뒤 0.15 rad(8.6°) 틀어짐, 켜면 kp 2 → 정상 상태 오차 ≈ 외란/kp = 0.025 rad 근처에서 붙잡음
TEST(HeadingHold, HoldsYawAgainstDisturbanceOnlyWhileTranslatingWithoutTurnCommand) {
  auto run_hold = [](double kp, const BodyTwist & cmd, double secs) {
      SwerveDriveConfig cfg;
      cfg.heading_hold_kp = kp;
      auto d = make_drive(cfg);
      std::vector<ModuleState> meas(4, ModuleState{0.0, 0.0});
      double max_wz = 0.0;
      for (double t = 0.0; t < secs - 1e-9; t += kDt) {
        d.set_turn(true, 7, t);
        d.set_command(cmd, t);
        const auto out = d.update(t, kDt, meas);
        for (std::size_t i = 0; i < meas.size(); ++i) {meas[i] = out[i];}
        max_wz = std::max(max_wz, std::fabs(d.output().wz));
        d.integrate_odom(meas, kDt, true, d.output().wz + 0.05);   // 실제 회전 = 명령 + 외란
      }
      return std::make_pair(d.pose().yaw, max_wz);
    };
  const BodyTwist fwd{0.1, 0.0, 0.0};
  const auto off = run_hold(0.0, fwd, 3.0);
  EXPECT_NEAR(off.first, 0.15, 0.01);
  EXPECT_NEAR(off.second, 0.0, 1e-12);                  // 끄면 회전 명령을 전혀 안 섞음
  const auto on = run_hold(2.0, fwd, 3.0);
  EXPECT_LT(std::fabs(on.first), 0.04);                  // 붙잡음 (P 제어라 외란/kp 만큼 남음)
  EXPECT_LE(on.second, 0.15 + 0.8 * kDt + 1e-9);         // 최대 회전 0.15 rad/s (+ 가속 제한 한 주기)
  // 회전 명령이 있으면 손대지 않음 (사람이 돌리는 중)
  const BodyTwist turn{0.1, 0.0, 0.2};
  const auto turning = run_hold(2.0, turn, 1.0);
  EXPECT_GT(turning.first, 0.1);                         // 명령 0.2 + 외란 0.05 그대로 돎
  // 제자리 정지 명령에서는 돌리지 않음
  const auto still = run_hold(2.0, BodyTwist{0.0, 0.0, 0.0}, 1.0);
  EXPECT_NEAR(still.second, 0.0, 1e-12);
}

// 2026-10-28 정면 유지가 범위 끝 바퀴를 갈아타게 만들면 그 주기는 보정 안 함
//   (모의: 대각 왕복 뒤왼쪽 귀환에서 rr −44.5° 끝 + 보정 회전 → rr 이 반대 분기로 돌기 시작해 구동이 멈춤)
TEST(HeadingHold, NeverFlipsModuleSittingAtRangeEnd) {
  const double d = kPi / 180;
  const std::vector<swerve_controller::SteeringRange> r = {{-90.439*d, 269.473*d}, {-223.506*d, 136.406*d}, {-133.418*d, 170.0*d}, {-44.473*d, 315.439*d}};
  SwerveDriveConfig cfg;
  cfg.heading_hold_kp = 2.0;
  cfg.max_linear_accel = 0.0; cfg.max_angular_accel = 0.0;   // 최대 시험처럼 램프 없음
  SwerveKinematics k(real_modules(), kR, kPi, r, true, 0.0, 1.0 * d);
  SwerveDrive drv(k, cfg);
  std::vector<ModuleState> meas(4, ModuleState{-45 * d, 0.0});
  meas[3].steer_angle = -44.473 * d;
  const BodyTwist back_left{-0.092, 0.092, 0.0};
  double worst = 0.0;
  for (double t = 0.0; t < 2.0; t += kDt) {
    step(drv, meas, t, &back_left);
    worst = std::max(worst, std::fabs(meas[3].steer_angle - (-44.473 * d)));
    drv.integrate_odom(meas, kDt, true, drv.output().wz - 0.05);   // 차체를 반대로 미는 외란 → 보정이 rr 끝 쪽으로 밀려 함
  }
  EXPECT_LT(worst, 5 * d);   // rr 은 끝에 붙은 그대로 (갈아타기 없음)
}

// 2026-10-28 정면 유지 보정 상한을 이동 속도에 비례 (heading_hold_wz_per_speed): 느릴 때 조향이 흔들리지 않게
TEST(HeadingHold, CorrectionScalesWithTranslationSpeed) {
  auto max_wz_at = [](double v, double per_speed) {
      SwerveDriveConfig cfg;
      cfg.heading_hold_kp = 2.0;
      cfg.heading_hold_wz_per_speed = per_speed;
      auto d = make_drive(cfg);
      std::vector<ModuleState> meas(4, ModuleState{0.0, 0.0});
      const BodyTwist cmd{v, 0.0, 0.0};
      double m = 0.0;
      for (double t = 0.0; t < 3.0; t += kDt) {
        d.set_turn(true, 7, t);
        d.set_command(cmd, t);
        const auto out = d.update(t, kDt, meas);
        for (std::size_t i = 0; i < meas.size(); ++i) {meas[i] = out[i];}
        m = std::max(m, std::fabs(d.output().wz));
        d.integrate_odom(meas, kDt, true, d.output().wz + 0.05);   // 큰 외란 → 보정이 상한에 붙음
      }
      return m;
    };
  EXPECT_GT(max_wz_at(0.05, 0.0), 0.04);                 // 끄면 외란(0.05)만큼 크게 보정 (상한 0.15 안)
  EXPECT_LE(max_wz_at(0.05, 0.4), 0.02 + 1e-9);          // 0.05 m/s → 0.02 rad/s
  EXPECT_LE(max_wz_at(0.1, 0.4), 0.04 + 1e-9);           // 0.10 m/s → 0.04 rad/s
}

// 2026-10-29 가지 고르기 보강 — 10-28 실물 36 방향 왕복의 뒤집힘 (실측 범위, 끝 여유 1°)
namespace {
const double kDeg = kPi / 180;
const std::vector<swerve_controller::SteeringRange> kMeasured = {
  {-90.439*kDeg, 269.473*kDeg}, {-223.506*kDeg, 136.406*kDeg}, {-133.418*kDeg, 170.0*kDeg}, {-44.473*kDeg, 315.439*kDeg}};
double max_turn(const std::vector<ModuleState> & a, const std::vector<ModuleState> & b) {
  double m = 0.0;
  for (std::size_t i = 0; i < a.size(); ++i) {m = std::max(m, std::fabs(a[i].steer_angle - b[i].steer_angle));}
  return m;
}
}  // namespace

// 공통 자세(네 범위의 공통 구간 약 −41°~+133°)끼리 옮기면 바퀴가 170° 가까이 돎 → 한도를 넘으면 바퀴별로
TEST(BranchGuard, CoherentMaxTravelAvoidsLongCommonSwing) {
  const auto now = facing(-30 * kDeg);                       // 직전: 공통 자세 −30°
  SwerveKinematics old_rule(real_modules(), kR, kPi, kMeasured, true, 0.0, 1.0 * kDeg);
  old_rule.set_travel_weight(2.0);
  const auto a = old_rule.inverse(heading(120), now, false, nullptr, now, true);
  EXPECT_GT(max_turn(a, now), 140 * kDeg);                   // 예전: 공통 120° 로 150° 돎
  SwerveKinematics guard(real_modules(), kR, kPi, kMeasured, true, 0.0, 1.0 * kDeg);
  guard.set_travel_weight(2.0);
  guard.set_branch_options(100 * kDeg, 0.0);
  const auto b = guard.inverse(heading(120), now, false, nullptr, now, true);
  int long_turns = 0;
  for (int i = 0; i < 4; ++i) {long_turns += std::fabs(b[i].steer_angle - now[i].steer_angle) > 100 * kDeg;}
  EXPECT_LE(long_turns, 1);                                  // rr 만 (−60° 는 rr 범위 밖이라 어쩔 수 없음)
  expect_realizes(guard, heading(120), b, kMeasured);
  EXPECT_THROW(guard.set_branch_options(-0.1, 0.0), std::invalid_argument);
  EXPECT_THROW(guard.set_branch_options(0.0, 50 * kDeg), std::invalid_argument);
}

// 출발 전에 범위 끝에 붙는 가지를 피함 (320° 방향: rr −40° 는 끝 −44.5° 에서 4.5°) · 이동 중에는 벌점 없음
TEST(BranchGuard, EndGuardPicksRoomyBranchOnlyWhenStarting) {
  const auto now = facing(0.0);
  SwerveKinematics plain(real_modules(), kR, kPi, kMeasured, true, 0.0, 1.0 * kDeg);
  SwerveKinematics guard(real_modules(), kR, kPi, kMeasured, true, 0.0, 1.0 * kDeg);
  guard.set_branch_options(0.0, 20 * kDeg);
  const auto a = plain.inverse(heading(-40), now, false, nullptr, {}, true);
  const auto b = guard.inverse(heading(-40), now, false, nullptr, {}, true);
  EXPECT_NEAR(a[3].steer_angle, -40 * kDeg, 1e-9);           // 끔: rr 끝 근처
  EXPECT_NEAR(b[3].steer_angle, 140 * kDeg, 1e-9);           // 켬: 반대 가지(바퀴 거꾸로)
  expect_realizes(guard, heading(-40), b, kMeasured);
  // 이동 중(prefer_outward=false)에는 벌점을 안 씀 → 출력이 끈 것과 같음
  const auto moving_now = facing(-40 * kDeg);
  const auto c = plain.inverse(heading(-40), moving_now, false, nullptr, moving_now, false);
  const auto e = guard.inverse(heading(-40), moving_now, false, nullptr, moving_now, false);
  for (int i = 0; i < 4; ++i) {EXPECT_EQ(c[i].steer_angle, e[i].steer_angle);}
}

// 두 값 모두 0 이면 예전과 모든 출력이 같음 (전방향 · 여러 시작 자세 · 출발/이동)
TEST(BranchGuard, ZeroOptionsKeepOldOutputs) {
  SwerveKinematics old_rule(real_modules(), kR, kPi, kMeasured, true, 0.0, 1.0 * kDeg);
  SwerveKinematics zero(real_modules(), kR, kPi, kMeasured, true, 0.0, 1.0 * kDeg);
  old_rule.set_travel_weight(2.0);
  zero.set_travel_weight(2.0);
  zero.set_branch_options(0.0, 0.0);
  for (int deg = -180; deg < 180; deg += 10) {
    for (const auto & start : {facing(0), facing(-40 * kDeg), facing(120 * kDeg)}) {
      for (bool outward : {true, false}) {
        const auto p = old_rule.inverse(heading(deg), start, false, nullptr, start, outward);
        const auto q = zero.inverse(heading(deg), start, false, nullptr, start, outward);
        for (int i = 0; i < 4; ++i) {EXPECT_EQ(p[i].steer_angle, q[i].steer_angle);}
      }
    }
  }
}

// 10-28 실물 140° 왕복 귀환: 0.02 m/s · −40.5° 로 가는데 추종기 방향 보정 wz −0.015 → rr 목표 −45.6° (끝 + 여유 넘음)
//   끔: rr 이 반대 가지로 돌기 시작 / 켬(keep_branch_max_wz): 회전을 줄여 rr 이 −40° 근처 그대로
TEST(BranchGuard, KeepBranchTrimsSmallTurnInsteadOfFlipping) {
  auto run_case = [](double keep) {
      SwerveDriveConfig cfg;
      cfg.keep_branch_max_wz = keep;
      cfg.max_linear_accel = 0.0; cfg.max_angular_accel = 0.0;
      SwerveKinematics k(real_modules(), kR, kPi, kMeasured, true, 0.0, 1.0 * kDeg);
      SwerveDrive drv(k, cfg);
      std::vector<ModuleState> meas = facing(-40.5 * kDeg);
      const double v = 0.02, a = -40.5 * kDeg;
      const BodyTwist straight{v * std::cos(a), v * std::sin(a), 0.0};
      const BodyTwist with_turn{v * std::cos(a), v * std::sin(a), -0.015};
      double worst = 0.0, min_ratio = 1.0;
      for (double t = 0.0; t < 3.0; t += kDt) {
        step(drv, meas, t, t < 0.5 ? &straight : &with_turn);   // 먼저 직진으로 출발(병진 중) → 보정 섞임
        worst = std::max(worst, std::fabs(meas[3].steer_angle - (-40.5 * kDeg)));
        min_ratio = std::min(min_ratio, drv.wz_keep_ratio());
      }
      return std::make_pair(worst, min_ratio);
    };
  const auto off = run_case(0.0);
  EXPECT_GT(off.first, 30 * kDeg);                           // 끔: 갈아타기 시작
  EXPECT_DOUBLE_EQ(off.second, 1.0);
  const auto on = run_case(0.2);
  EXPECT_LT(on.first, 6 * kDeg);                             // 켬: 같은 가지 (끝까지 몇 도만)
  EXPECT_LT(on.second, 1.0);                                 // 회전을 줄였음
}

// 10-29 오후 모의 12 동작 회귀 (곡선 정렬 시간 초과): 일부러 준 느린 곡선(0.02 m/s + 0.15 rad/s)은 바퀴가 기하학적으로
//   30° 넘게 꺾여도 같은 가지 → 회전을 깎지 않고 명령 그대로 (예전 판정은 회전을 0 근처로 깎아 곡선이 안 됨)
TEST(BranchGuard, KeepBranchDoesNotTrimIntendedSlowArc) {
  SwerveDriveConfig cfg;
  cfg.keep_branch_max_wz = 0.2;
  cfg.max_linear_accel = 0.0; cfg.max_angular_accel = 0.0;
  SwerveKinematics k(real_modules(), kR, kPi, kMeasured, true, 0.0, 1.0 * kDeg);
  SwerveDrive drv(k, cfg);
  std::vector<ModuleState> meas = facing(0.0);
  const BodyTwist straight{0.02, 0.0, 0.0}, arc{0.02, 0.0, 0.15};
  double min_ratio = 1.0;
  for (double t = 0.0; t < 3.0; t += kDt) {
    step(drv, meas, t, t < 0.5 ? &straight : &arc);
    min_ratio = std::min(min_ratio, drv.wz_keep_ratio());
  }
  EXPECT_DOUBLE_EQ(min_ratio, 1.0);
  EXPECT_NEAR(drv.output().wz, 0.15, 1e-9);
  double max_turn = 0.0;
  const auto geo = real_modules();
  for (std::size_t i = 0; i < meas.size(); ++i) {   // 바퀴는 곡선 방향 (조향축 속도 방향)
    const auto & g = geo[i];
    const double want = std::atan2(arc.vy + arc.wz * g.x, arc.vx - arc.wz * g.y);
    EXPECT_NEAR(std::remainder(meas[i].steer_angle - want, kPi), 0.0, 1 * kDeg) << i << " 각 " << meas[i].steer_angle / kDeg << " 기대 " << want / kDeg;
    max_turn = std::max(max_turn, std::fabs(want));
  }
  EXPECT_GT(max_turn, 30 * kDeg);   // 예전 판정이면 깎였을 크기
}

// 2026-10-29 검토: 범위 끝 벌점은 nearest 정책에도 (예전 비용식은 ?: 우선순위로 coherent 에만 붙었음)
TEST(BranchGuard, EndGuardAlsoAppliesToNearestPolicy) {
  const auto now = facing(-30 * kDeg);
  SwerveKinematics plain(real_modules(), kR, kPi, kMeasured, false, 0.0, 1.0 * kDeg);
  SwerveKinematics guard(real_modules(), kR, kPi, kMeasured, false, 0.0, 1.0 * kDeg);
  guard.set_branch_options(0.0, 20 * kDeg);
  const auto a = plain.inverse(heading(-40), now, false, nullptr, {}, true);
  const auto b = guard.inverse(heading(-40), now, false, nullptr, {}, true);
  EXPECT_NEAR(a[3].steer_angle, -40 * kDeg, 1e-9);           // nearest: 가까운 −40° (끝 −44.5° 에서 4.5°)
  EXPECT_NEAR(b[3].steer_angle, 140 * kDeg, 1e-9);           // 벌점: 여유 있는 반대 가지
  expect_realizes(guard, heading(-40), b, kMeasured);
}

// 2026-10-29 검토: 생성자가 NaN · 범위 밖 값을 거름 (예전엔 루프 안 예외 또는 조용히 통과)
TEST(BranchGuard, ConstructorRejectsNonFiniteGeometry) {
  EXPECT_THROW(SwerveKinematics(real_modules(), NAN, kPi), std::invalid_argument);
  EXPECT_THROW(SwerveKinematics(real_modules(), kR, NAN), std::invalid_argument);
  EXPECT_THROW(SwerveKinematics(real_modules(), kR, 3.5), std::invalid_argument);   // ±3.5 rad 는 한 바퀴(2π) 넘음
  auto bad = real_modules();
  bad[0].x = NAN;
  EXPECT_THROW(SwerveKinematics(bad, kR, kPi), std::invalid_argument);
  EXPECT_NO_THROW(SwerveKinematics(real_modules(), kR, 3.14159));
}
