// 저속 조향 유지 (steer_hold_speed) 단위 시험. 실물 설정 (controllers.yaml: coherent · 모듈별 범위 · blend 켬) + 간단 플랜트
#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <random>
#include "swerve_controller/swerve.hpp"
using namespace swerve_controller;

namespace {
const double PI = 3.14159265358979323846;

// old = blend 끔 설정 (travel_weight 2 · heading_hold 2 · coherent_max_travel 90° ·
//   end_guard 15° · keep_branch 0.2 · branch_margin 3° · end_slack 0 · blend 끔 · swing_gate 0)
SwerveKinematics real_kin(bool old = false) {
  std::vector<ModuleGeometry> g = {{-0.07, 0.112, 0.0269}, {0.06, 0.112, 0.0269}, {-0.07, -0.112, -0.0269}, {0.06, -0.112, -0.0269}};
  std::vector<SteeringRange> r;
  const double t = 2.0 * PI / 4096;
  for (int z : {2048, 2048, 2048, 2565}) {r.push_back({-(4075 - z) * t, (z - 20) * t});}
  SwerveKinematics k(g, 0.033, PI, r, true, old ? 0.0524 : 0.0, old ? 0.0 : 0.017453, 2.0 * PI);
  k.set_travel_weight(old ? 2.0 : 0.0);
  k.set_branch_options(old ? 1.5708 : 0.0, old ? 0.2618 : 0.0);
  return k;
}

SwerveDriveConfig real_cfg(double hold, bool old = false) {
  SwerveDriveConfig c;
  c.max_wheel_speed = 7.9; c.max_linear = 0.255; c.max_angular = 0.5; c.max_linear_accel = 0.5; c.max_angular_accel = 2.0;
  c.cosine_scale = true; c.steer_then_drive = false; c.steer_tolerance = 0.17; c.roll_while_steering = true;
  c.blend_steer_drive = true; c.blend_margin = 0.7; c.blend_tolerance = 2.1991; c.blend_stall_s = 1.0;
  c.steer_hold_speed = hold; c.steer_hold_release = hold > 0.0 ? 0.025 : 0.0;
  if (old) {
    c.blend_steer_drive = false; c.blend_swing_gate = 0.0; c.heading_hold_kp = 2.0; c.heading_hold_wz_per_speed = 0.4;
    c.keep_branch_max_wz = 0.2;
  }
  return c;
}

// 조향 플랜트: 느린 실물 쪽 (최대 2 rad/s, τ 0.15 s) · 바퀴 τ 0.04 s
struct Sim {
  SwerveDrive d;
  std::vector<ModuleState> m{4, ModuleState{0.0, 0.0, 0.0}};
  double t = 0.0;
  double travel = 0.0;   // 조향 실측 총 이동 [rad]
  explicit Sim(double hold, bool old = false) : d(real_kin(old), real_cfg(hold, old)) {}
  std::vector<ModuleState> step(BodyTwist cmd, bool fresh = true) {
    if (fresh) {d.set_command(cmd, t);}
    d.set_turn(true, 1, t);
    const auto out = d.update(t, 0.01, m);
    const auto v = d.kinematics().forward(m);   // 몸 자세 (정면 유지 되먹임용 odom)
    d.integrate_odom(m, 0.01, false, 0.0);
    (void)v;
    for (int i = 0; i < 4; ++i) {
      const double rate = std::clamp((out[i].steer_angle - m[i].steer_angle) / 0.15, -2.0, 2.0);
      m[i].steer_rate = rate;
      m[i].steer_angle += rate * 0.01;
      travel += std::fabs(rate * 0.01);
      m[i].wheel_speed += (1.0 - std::exp(-0.01 / 0.04)) * (out[i].wheel_speed - m[i].wheel_speed);
    }
    t += 0.01;
    return out;
  }
  void run(BodyTwist cmd, int n) {for (int k = 0; k < n; ++k) {step(cmd);}}
};

// 정밀 접근 끝 흉내: 0.008 m/s 안팎 + 방향 잡음 ±40° (20 Hz 로 바뀜)
// 돌려줌 = 조향 이동 [rad], fwd = 앞으로 간 거리 [m]
double noisy_tail(Sim & s, unsigned seed, double * min_speed_sum = nullptr) {
  std::mt19937 rng(seed);
  std::normal_distribution<double> n(0.0, 0.7);
  const double t0 = s.travel;
  double wheel_sum = 0.0;
  BodyTwist c{};
  for (int k = 0; k < 400; ++k) {
    if (k % 5 == 0) {const double a = 0.0 + std::clamp(n(rng), -0.7, 0.7); c = {0.008 * std::cos(a), 0.008 * std::sin(a), 0.0};}
    s.step(c);
    wheel_sum += s.d.kinematics().forward(s.m).vx * 0.01;   // 앞으로 간 거리 [m]
  }
  if (min_speed_sum) {*min_speed_sum = wheel_sum;}
  return s.travel - t0;
}
}  // namespace

TEST(SteerHold, OffIsBitIdenticalToDefault) {   // steer_hold_speed 0 = 기능이 없을 때와 같은 출력 (학습 플랜트와 같음)
  SwerveDrive a(real_kin(), real_cfg(0.0));
  SwerveDriveConfig cb = real_cfg(0.0); cb.steer_hold_release = 0.05; cb.steer_hold_max_angle = 0.3;
  SwerveDrive b(real_kin(), cb);
  std::mt19937 rng(3);
  std::uniform_real_distribution<double> u(-0.03, 0.03);
  std::vector<ModuleState> m(4, ModuleState{0.2, 0.0, 0.0});
  double t = 0.0;
  for (int k = 0; k < 300; ++k, t += 0.01) {
    const BodyTwist c{u(rng), u(rng), u(rng)};
    a.set_command(c, t); b.set_command(c, t); a.set_turn(true, 1, t); b.set_turn(true, 1, t);
    const auto oa = a.update(t, 0.01, m), ob = b.update(t, 0.01, m);
    for (int i = 0; i < 4; ++i) {
      EXPECT_DOUBLE_EQ(oa[i].steer_angle, ob[i].steer_angle);
      EXPECT_DOUBLE_EQ(oa[i].wheel_speed, ob[i].wheel_speed);
      m[i].steer_angle += std::clamp(oa[i].steer_angle - m[i].steer_angle, -0.02, 0.02);
    }
    EXPECT_EQ(b.steer_held_count(), 0);
  }
}

TEST(SteerHold, NoisyCreepKeepsSteerAndStillDrives) {   // 정밀 접근 끝: 조향 이동이 크게 줄고 바퀴는 계속 구름
  Sim off(0.0), on(0.015);
  off.run({0.05, 0.0, 0.0}, 100); on.run({0.05, 0.0, 0.0}, 100);   // 앞으로 달리다
  double w_off = 0.0, w_on = 0.0;
  const double tr_off = noisy_tail(off, 7, &w_off), tr_on = noisy_tail(on, 7, &w_on);
  EXPECT_GT(tr_off, 1.0);            // 끔: 잡음 방향을 쫓아 조향이 1 rad 넘게 돎
  EXPECT_LT(tr_on, 0.25 * tr_off);   // 켬: 4 배 넘게 줄어듦
  EXPECT_EQ(on.d.steer_held_count(), 4);
  EXPECT_GT(w_off, 0.02);            // 4 s · 0.008 m/s 안팎
  EXPECT_NEAR(w_on, w_off, 0.25 * w_off);   // 켬도 앞으로 거의 같은 거리 (투영 = 앞 성분 유지)
}

TEST(SteerHold, LowSpeedReverseRollsBackwardInsteadOfSwinging) {   // 목표 근처 vx 부호 바뀜 → 조향 그대로, 바퀴 거꾸로
  Sim s(0.015);
  s.run({0.01, 0.0, 0.0}, 200);
  const auto before = s.step({0.01, 0.0, 0.0});
  std::vector<ModuleState> out;
  for (int k = 0; k < 100; ++k) {out = s.step({-0.01, 0.0, 0.0});}
  for (int i = 0; i < 4; ++i) {
    EXPECT_NEAR(out[i].steer_angle, before[i].steer_angle, 1e-9);
    EXPECT_LT(out[i].wheel_speed * before[i].wheel_speed, 0.0);   // 반대 방향
    EXPECT_NEAR(std::fabs(out[i].wheel_speed), 0.01 / 0.033, 0.02);
  }
}

TEST(SteerHold, ReleasesAboveReleaseSpeedWithHysteresis) {
  Sim s(0.015);
  s.run({0.01, 0.0, 0.0}, 100);
  EXPECT_EQ(s.d.steer_held_count(), 4);
  s.run({0.0, 0.02, 0.0}, 5);   // 0.015 ~ 0.025 사이 · 90° → 각도로 풂 → 새 각도로 다시 묶임은 0.015 아래에서만
  EXPECT_EQ(s.d.steer_held_count(), 0);
  Sim h(0.015);
  h.run({0.01, 0.0, 0.0}, 100);
  h.run({0.02, 0.0, 0.0}, 50);   // 같은 방향 0.02: 풀림 속도 아래 → 계속 묶임
  EXPECT_EQ(h.d.steer_held_count(), 4);
  h.run({0.03, 0.0, 0.0}, 5);    // 0.025 넘음 → 풂
  EXPECT_EQ(h.d.steer_held_count(), 0);
  h.run({0.02, 0.0, 0.0}, 50);   // 빠른 데서 0.02 로: 묶음 속도 (0.015) 위라 안 묶임
  EXPECT_EQ(h.d.steer_held_count(), 0);
}

TEST(SteerHold, LargeDirectionChangeRetargetsThenHolds) {   // 느려도 방향이 60° 넘게 바뀌면 다시 조향 (옆으로 못 가 갇히지 않게)
  Sim s(0.015);
  s.run({0.01, 0.0, 0.0}, 100);
  std::vector<ModuleState> out;
  for (int k = 0; k < 300; ++k) {out = s.step({0.0, 0.01, 0.0});}
  for (int i = 0; i < 4; ++i) {EXPECT_NEAR(std::fabs(std::remainder(out[i].steer_angle, PI)), PI / 2, 0.05);}
  EXPECT_EQ(s.d.steer_held_count(), 4);   // 새 각도에서 다시 묶임
  for (const auto & m : s.m) {EXPECT_NEAR(std::fabs(std::remainder(m.steer_angle, PI)), PI / 2, 0.1);}
}

TEST(SteerHold, SmallDirectionChangeProjectsOntoHeldAngle) {
  Sim s(0.015);
  s.run({0.01, 0.0, 0.0}, 100);
  const double a = 30.0 * PI / 180.0;
  const auto before = s.step({0.01, 0.0, 0.0});
  const auto out = s.step({0.01 * std::cos(a), 0.01 * std::sin(a), 0.0});
  for (int i = 0; i < 4; ++i) {
    EXPECT_NEAR(out[i].steer_angle, before[i].steer_angle, 1e-9);
    EXPECT_NEAR(std::fabs(out[i].wheel_speed), 0.01 * std::cos(a) / 0.033 * std::cos(before[i].steer_angle - s.m[i].steer_angle), 0.02);
  }
}

TEST(SteerHold, ZeroAndStaleCommandsUnchanged) {   // 0 명령 · 끊김은 끔과 같음 (지금 각도 유지 · 정렬 확인)
  Sim off(0.0), on(0.015);
  off.run({0.01, 0.0, 0.0}, 100); on.run({0.01, 0.0, 0.0}, 100);
  for (int k = 0; k < 100; ++k) {
    const auto a = off.step({0.0, 0.0, 0.0}), b = on.step({0.0, 0.0, 0.0});
    if (k > 10) {for (int i = 0; i < 4; ++i) {EXPECT_NEAR(a[i].steer_angle, b[i].steer_angle, 1e-6); EXPECT_NEAR(b[i].wheel_speed, a[i].wheel_speed, 1e-6);}}
  }
  on.run({0.01, 0.0, 0.0}, 50);
  for (int k = 0; k < 60; ++k) {on.step({}, false);}   // 0.3 s 넘게 명령 없음
  EXPECT_EQ(on.d.steer_held_count(), 0);
  const auto o = on.step({}, false);
  for (int i = 0; i < 4; ++i) {EXPECT_DOUBLE_EQ(o[i].steer_angle, on.m[i].steer_angle);}
}

TEST(SteerHold, FastMotionUnchanged) {   // 빠른 주행 (모듈 0.025 m/s 넘음) 은 끔과 바이트까지 같음
  Sim off(0.0), on(0.015);
  for (int k = 0; k < 400; ++k) {
    const double a = 0.01 * k;
    const BodyTwist c{0.15 * std::cos(a), 0.15 * std::sin(a), 0.1};
    const auto x = off.step(c), y = on.step(c);
    for (int i = 0; i < 4; ++i) {EXPECT_DOUBLE_EQ(x[i].steer_angle, y[i].steer_angle); EXPECT_DOUBLE_EQ(x[i].wheel_speed, y[i].wheel_speed);}
  }
}

TEST(SteerHold, SlowRotationStillTurnsBody) {   // 묶인 채 느린 제자리 회전: 투영으로 몸이 같은 쪽으로 돎
  Sim s(0.015);
  s.run({0.01, 0.0, 0.0}, 100);
  BodyTwist v{};
  for (int k = 0; k < 100; ++k) {s.step({0.0, 0.0, 0.08}); v = s.d.kinematics().forward(s.m);}
  EXPECT_GT(v.wz, 0.03);
}

// ---- blend 끔 설정 (정면 유지 켬 · 가지 유지 켬) 에서도 같은 성질 ----
TEST(SteerHoldOldParams, OffIsBitIdentical) {
  Sim a(0.0, true);
  SwerveDriveConfig cb = real_cfg(0.0, true); cb.steer_hold_release = 0.05; cb.steer_hold_max_angle = 0.3;
  Sim b(0.0, true); b.d = SwerveDrive(real_kin(true), cb);
  std::mt19937 rng(5);
  std::uniform_real_distribution<double> u(-0.03, 0.03);
  for (int k = 0; k < 400; ++k) {
    const BodyTwist c{u(rng), u(rng), k % 3 ? 0.0 : u(rng)};
    const auto x = a.step(c), y = b.step(c);
    for (int i = 0; i < 4; ++i) {EXPECT_DOUBLE_EQ(x[i].steer_angle, y[i].steer_angle); EXPECT_DOUBLE_EQ(x[i].wheel_speed, y[i].wheel_speed);}
  }
}

TEST(SteerHoldOldParams, NoisyCreepKeepsSteerAndStillDrives) {
  Sim off(0.0, true), on(0.015, true);
  off.run({0.05, 0.0, 0.0}, 100); on.run({0.05, 0.0, 0.0}, 100);
  double w_off = 0.0, w_on = 0.0;
  const double tr_off = noisy_tail(off, 9, &w_off), tr_on = noisy_tail(on, 9, &w_on);
  EXPECT_GT(tr_off, 1.0);
  EXPECT_LT(tr_on, 0.25 * tr_off);
  EXPECT_GT(w_off, 0.02);
  EXPECT_NEAR(w_on, w_off, 0.25 * w_off);
}

TEST(SteerHoldOldParams, LowSpeedReverseRollsBackward) {
  Sim s(0.015, true);
  s.run({0.01, 0.0, 0.0}, 200);
  const auto before = s.step({0.01, 0.0, 0.0});
  std::vector<ModuleState> out;
  for (int k = 0; k < 100; ++k) {out = s.step({-0.01, 0.0, 0.0});}
  for (int i = 0; i < 4; ++i) {
    EXPECT_NEAR(out[i].steer_angle, before[i].steer_angle, 0.02);   // 정면 유지 보정 회전만큼만 다름
    EXPECT_LT(out[i].wheel_speed * before[i].wheel_speed, 0.0);
  }
}

TEST(SteerHoldOldParams, FastMotionUnchanged) {
  Sim off(0.0, true), on(0.015, true);
  for (int k = 0; k < 400; ++k) {
    const double a = 0.01 * k;
    const BodyTwist c{0.15 * std::cos(a), 0.15 * std::sin(a), k < 200 ? 0.0 : 0.1};
    const auto x = off.step(c), y = on.step(c);
    for (int i = 0; i < 4; ++i) {EXPECT_DOUBLE_EQ(x[i].steer_angle, y[i].steer_angle); EXPECT_DOUBLE_EQ(x[i].wheel_speed, y[i].wheel_speed);}
  }
}

TEST(SteerHoldOldParams, LargeDirectionChangeRetargets) {
  Sim s(0.015, true);
  s.run({0.01, 0.0, 0.0}, 100);
  for (int k = 0; k < 300; ++k) {s.step({0.0, 0.01, 0.0});}
  for (const auto & m : s.m) {EXPECT_NEAR(std::fabs(std::remainder(m.steer_angle, PI)), PI / 2, 0.15);}
}
