// 방향 전환 때 조향은 최종 명령 각도로 (steer_final_target) — 실물에서 본 rl_nav → rl_dock 넘김 재현
#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include "swerve_controller/swerve.hpp"
using namespace swerve_controller;

namespace {
const double PI = 3.14159265358979323846;
// 실물 범위 (hardware_map zero_tick 2048 · 2565, scale −1, 끝 20 눈금 여유): fl · fr · rl ±178.2°, rr −132.7° ~ 223.7°
SwerveKinematics kin(bool ours) {
  std::vector<ModuleGeometry> g = {{-0.07, 0.112, 0.0269}, {0.06, 0.112, 0.0269}, {-0.07, -0.112, -0.0269}, {0.06, -0.112, -0.0269}};
  std::vector<SteeringRange> r;
  const double t = 2.0 * PI / 4096;
  for (int z : {2048, 2048, 2048, 2565}) {r.push_back({-(4075 - z) * t, (z - 20) * t});}
  SwerveKinematics k(g, 0.033, PI, r, true, ours ? 0.0524 : 0.0, ours ? 0.0 : 0.017453, 2.0 * PI);
  k.set_travel_weight(ours ? 2.0 : 0.0);
  k.set_branch_options(ours ? 1.5708 : 0.0, ours ? 0.2618 : 0.0);
  return k;
}
// ours = blend 끔 설정 (정면 유지 · 가지 유지 켬) / 아니면 blend 켬 설정 (학습 플랜트 값)
SwerveDriveConfig cfg(bool ours, bool fin) {
  SwerveDriveConfig c;
  c.max_wheel_speed = 7.9; c.max_linear = 0.255; c.max_angular = 0.5; c.max_linear_accel = 0.5; c.max_angular_accel = 2.0;
  c.steer_then_drive = false; c.steer_tolerance = 0.17; c.roll_while_steering = true;
  if (ours) {
    c.blend_steer_drive = false; c.blend_swing_gate = 0.0; c.heading_hold_kp = 2.0; c.heading_hold_wz_per_speed = 0.4; c.keep_branch_max_wz = 0.2;
  } else {
    c.blend_steer_drive = true; c.blend_margin = 0.7; c.blend_tolerance = 2.1991; c.blend_stall_s = 1.0;
  }
  c.steer_final_target = fin;
  return c;
}
struct HandoffRun {
  double travel[4] = {0, 0, 0, 0};
  double min_q[4], max_q[4], start[4], end[4];
};
// 실물 조향: 사다리꼴 4.8 rad/s · 22.5 rad/s² (profile_vel 200 · profile_acc 60). 바퀴 τ 40 ms
HandoffRun handoff(bool ours, bool fin) {
  SwerveDrive d(kin(ours), cfg(ours, fin));
  std::vector<ModuleState> m = {{-104.0 * PI / 180, -5.0, 0}, {-101.0 * PI / 180, -6.0, 0}, {-120.0 * PI / 180, -6.0, 0}, {-116.0 * PI / 180, -6.0, 0}};
  double vel[4] = {0, 0, 0, 0};
  double t = 0.0;
  HandoffRun r;
  auto step = [&](BodyTwist c, bool rec) {
      d.set_command(c, t);
      d.set_turn(true, 1, t);
      const auto out = d.update(t, 0.01, m);
      d.integrate_odom(m, 0.01, false, 0.0);
      for (int i = 0; i < 4; ++i) {
        const double err = out[i].steer_angle - m[i].steer_angle;
        const double want = std::copysign(std::min(4.8, std::sqrt(2.0 * 22.5 * std::fabs(err))), err);
        vel[i] += std::clamp(want - vel[i], -0.225, 0.225);
        const double q = m[i].steer_angle + vel[i] * 0.01;
        if (rec) {r.travel[i] += std::fabs(q - m[i].steer_angle); r.min_q[i] = std::min(r.min_q[i], q); r.max_q[i] = std::max(r.max_q[i], q);}
        m[i].steer_angle = q;
        m[i].steer_rate = vel[i];
        m[i].wheel_speed += (1.0 - std::exp(-0.25)) * (out[i].wheel_speed - m[i].wheel_speed);
      }
      t += 0.01;
    };
  for (int k = 0; k < 100; ++k) {step({0.071, 0.186, 0.265}, false);}   // rl_nav: 역방향 가지로 이동 중
  for (int i = 0; i < 4; ++i) {r.start[i] = r.min_q[i] = r.max_q[i] = m[i].steer_angle;}
  for (int k = 0; k < 10; ++k) {step({0.0, 0.0, 0.0}, true);}           // 넘김 0 명령 0.1 s
  for (int k = 0; k < 6; ++k) {step({0.176, -0.088, -0.003}, true);}    // rl_dock 첫 명령들
  for (int k = 0; k < 10; ++k) {step({0.145, -0.128, -0.021}, true);}
  for (int k = 0; k < 200; ++k) {step({0.13, -0.06, 0.0}, true);}       // 약 −25°
  for (int i = 0; i < 4; ++i) {r.end[i] = m[i].steer_angle;}
  return r;
}
}  // namespace

TEST(FinalTarget, HandoffGoesStraightToForwardBranch) {   // 두 값 모두: 출발 → 끝 (≈ −25°) 사이만 지남 (먼저 반대로 · 범위 끝 쪽으로 안 감)
  for (bool ours : {true, false}) {
    const HandoffRun r = handoff(ours, true);
    int reverse = 0;
    for (int i = 0; i < 4; ++i) {
      const double lo = std::min(r.start[i], r.end[i]), hi = std::max(r.start[i], r.end[i]);
      reverse += r.start[i] < -90.0 * PI / 180 ? 1 : 0;
      // 끝 = −25° 방향 (정 · 역 가지 어느 쪽이든: blend 값은 가까운 역방향 155° 를 고르기도 함)
      EXPECT_LT(std::fabs(std::remainder(r.end[i] + 25.0 * PI / 180, PI)), 12.0 * PI / 180) << "ours " << ours << " 모듈 " << i;
      EXPECT_GT(r.min_q[i], lo - 5.0 * PI / 180) << "ours " << ours << " 모듈 " << i;
      EXPECT_LT(r.max_q[i], hi + 5.0 * PI / 180) << "ours " << ours << " 모듈 " << i;
      EXPECT_LE(r.travel[i], hi - lo + 20.0 * PI / 180) << "ours " << ours << " 모듈 " << i;   // 정면 유지 · 회전 보정 몫
      EXPECT_LE(r.travel[i], 110.0 * PI / 180) << "ours " << ours << " 모듈 " << i;   // 역방향 −120° → −25° 가 95°
    }
    EXPECT_GE(reverse, ours ? 4 : 1);   // 출발이 역방향 가지 (blend 끔 설정은 4 바퀴 = 실물과 같음, blend 켬 설정은 fr 만)
  }
}

TEST(FinalTarget, OldBehaviourSweptTowardRangeEnd) {   // 끄면 blend 끔 설정에서 가속 램프를 따라 범위 끝 쪽으로 갔다가 뒤집힘 = 실물과 같음
  const HandoffRun r = handoff(true, false);
  int swept = 0;
  for (int i = 0; i < 3; ++i) {   // fl · fr · rl (범위 ±178°)
    if (r.min_q[i] < r.start[i] - 20.0 * PI / 180 && r.travel[i] > 150.0 * PI / 180) {++swept;}
  }
  EXPECT_GE(swept, 2);
}

TEST(FinalTarget, BlendSetOldBehaviourAlsoSwept) {   // blend 켬 설정도 끄면 역방향 바퀴 (fr) 가 범위 끝 (−178°) 쪽으로 갔다 돌아옴
  const HandoffRun r = handoff(false, false);
  EXPECT_LT(r.min_q[1], r.start[1] - 40.0 * PI / 180);
  EXPECT_GT(r.travel[1], 200.0 * PI / 180);
}

TEST(FinalTarget, SteadyAndStraightRampsUnchanged) {   // 방향이 안 바뀌는 출발 · 정지 · 정상 주행은 끔과 같음
  SwerveDrive a(kin(true), cfg(true, true)), b(kin(true), cfg(true, false));
  std::vector<ModuleState> m(4, ModuleState{0.3, 0.0, 0.0});
  double t = 0.0;
  auto both = [&](BodyTwist c, int n) {
      for (int k = 0; k < n; ++k, t += 0.01) {
        a.set_command(c, t); b.set_command(c, t); a.set_turn(true, 1, t); b.set_turn(true, 1, t);
        const auto x = a.update(t, 0.01, m), y = b.update(t, 0.01, m);
        for (int i = 0; i < 4; ++i) {
          EXPECT_NEAR(x[i].steer_angle, y[i].steer_angle, 1e-9);
          EXPECT_NEAR(x[i].wheel_speed, y[i].wheel_speed, 1e-9);
          m[i].steer_angle = x[i].steer_angle;
          m[i].wheel_speed = x[i].wheel_speed;
        }
      }
    };
  both({0.15 * std::cos(0.3), 0.15 * std::sin(0.3), 0.0}, 100);   // 같은 방향 가속 → 정상
  both({0.0, 0.0, 0.0}, 100);                                     // 정지
}
