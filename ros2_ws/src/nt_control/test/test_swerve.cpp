// test_swerve.cpp — 바퀴 쪽 순수 계산 시험 (swerve.hpp: 기구학 · 운전석 · 자이로)
// colcon test --packages-select nt_control

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "nt_control/swerve.hpp"

// ============================================================================ ① 기구학
// 실행: colcon test --packages-select nt_control && colcon test-result --verbose
//
// 각 시험은 "답을 머리로 알 수 있는 상황" 을 골랐다. 공식이 틀리면 바로 드러나게.

using nt_control::BodyTwist;
using nt_control::ModuleState;
using nt_control::SwerveKinematics;

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
  EXPECT_NEAR(nt_control::wrap_angle(3 * kPi / 2), -kPi / 2, kTol);
  EXPECT_NEAR(nt_control::wrap_angle(-3 * kPi / 2), kPi / 2, kTol);
  EXPECT_NEAR(nt_control::wrap_angle(0.3), 0.3, kTol);
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
  nt_control::SwerveDrive d(cad_limited(),{});
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
  using nt_control::limit_accel;
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
  using nt_control::steering_aligned;
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

using nt_control::BodyTwist;
using nt_control::ModuleState;
using nt_control::SwerveDrive;
using nt_control::SwerveDriveConfig;
using nt_control::SwerveKinematics;

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

using nt_control::GyroYaw;

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
