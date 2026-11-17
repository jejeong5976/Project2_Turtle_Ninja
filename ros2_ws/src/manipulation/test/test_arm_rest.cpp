// test_arm_rest.cpp — 쉬는 팔 (arm_rest.hpp): 끄기 · 켜기 요청, 끈 뒤 느슨한 접힘, 켜짐 기다림, 다시 앉히기 고르기
#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include "manipulation/arm.hpp"
#include "manipulation/arm_rest.hpp"

using manipulation::ArmRest;
using manipulation::ArmRestConfig;
using manipulation::ArmRestInput;
using manipulation::FoldCheck;
using manipulation::MotionArbiter;
using manipulation::Reseat;
using manipulation::reseat_choice;
using manipulation::slip_text;
using manipulation::Turn;

namespace
{
const std::vector<double> kHome{-0.0061, -1.4511, 0.9511, -0.1181, 0.0230};   // control.yaml poses.home

ArmRestConfig on_cfg()
{
  ArmRestConfig c;
  c.enabled = true;
  c.after_s = 1.0;
  c.wake_timeout_s = 1.0;
  c.state_alive_s = 1.0;
  return c;
}

ArmRestInput resting_input()
{
  ArmRestInput in;
  in.drive = true;
  in.job = false;
  in.holding = false;
  in.hw_ok = true;
  in.folded = true;
  return in;
}
}  // namespace

// 기본 꺼짐: 아무 조건이어도 끄기 요청 없음 · 처짐 · 기다림 없음 (토크를 늘 켜 두는 흐름과 같음)
TEST(ArmRest, DisabledNeverRequests)
{
  ArmRest r;
  ArmRestConfig c;   // enabled false
  std::string why;
  for (double t = 0.0; t < 10.0; t += 0.02) {
    r.on_state(t, true);
    EXPECT_FALSE(r.update(t, resting_input(), c, &why));
  }
  EXPECT_EQ(why, "arm_rest_torque_off 꺼짐");
  r.on_state(10.0, false);   // 요청 안 한 꺼짐 (켤 때 첫 주기 · 고장) → 쉬는 팔과 상관없음
  EXPECT_FALSE(r.slipped());
  r.wake(10.1);
  EXPECT_FALSE(r.waking());
  EXPECT_TRUE(r.arm_ok(10.1, c));
}

// 조건이 rest_after_s 동안 이어져야 끔. 하나라도 깨지면 처음부터
TEST(ArmRest, RequestsOnlyAfterQuietPeriodWithAllConditions)
{
  ArmRest r;
  const auto c = on_cfg();
  r.on_state(0.0, true);
  EXPECT_FALSE(r.update(0.0, resting_input(), c));
  r.on_state(0.5, true);
  EXPECT_FALSE(r.update(0.9, resting_input(), c));
  r.on_state(1.0, true);
  EXPECT_TRUE(r.update(1.0, resting_input(), c));
  EXPECT_TRUE(r.slipped()) << "보드가 살아 있으면 끄기 요청부터 처질 수 있음 → 느슨한 확인";

  const char * names[] = {"job", "not drive", "holding", "hw", "folded"};
  for (int k = 0; k < 5; ++k) {
    ArmRest q;
    q.on_state(0.0, true);
    q.update(0.0, resting_input(), c);
    auto in = resting_input();
    if (k == 0) {in.job = true;}
    if (k == 1) {in.drive = false;}
    if (k == 2) {in.holding = true;}
    if (k == 3) {in.hw_ok = false;}
    if (k == 4) {in.folded = false;}
    EXPECT_FALSE(q.update(0.5, in, c)) << names[k];
    q.on_state(1.0, true);
    EXPECT_FALSE(q.update(1.2, resting_input(), c)) << names[k] << ": 깨지면 처음부터 셈";
    EXPECT_FALSE(q.update(2.1, resting_input(), c)) << names[k] << ": 다시 이어진 1.2 s 부터 1 s";
    q.on_state(2.2, true);
    EXPECT_TRUE(q.update(2.2, resting_input(), c)) << names[k];
  }
}

// 끈 뒤 조건이 깨지면 바로 켜기 요청 + 켜짐 확인까지 팔 시작 안 함 (보드가 알리면 바로, 아니면 wake_timeout_s 뒤 경고 진행)
TEST(ArmRest, WakeWaitsForBoardConfirmationOrTimeout)
{
  ArmRest r;
  const auto c = on_cfg();
  r.on_state(0.0, true);
  r.update(0.0, resting_input(), c);
  ASSERT_TRUE(r.update(1.0, resting_input(), c));
  r.on_state(1.1, false);   // 보드: 꺼짐
  EXPECT_TRUE(r.slipped());
  auto in = resting_input();
  in.drive = false;          // 팔 작업 → STOPPING
  EXPECT_FALSE(r.update(5.0, in, c));
  EXPECT_TRUE(r.waking());
  EXPECT_FALSE(r.arm_ok(5.2, c));
  r.on_state(4.9, true);     // 켜기 요청 전에 보낸 상태는 확인이 아님
  EXPECT_TRUE(r.waking());
  r.on_state(5.15, true);    // 켜기 요청 뒤 "켜짐"
  EXPECT_FALSE(r.waking());
  EXPECT_TRUE(r.arm_ok(5.2, c));
  EXPECT_TRUE(r.slipped()) << "팔 궤적을 보내기 전까지는 처진 채";
  r.moved();
  EXPECT_FALSE(r.slipped());

  // 보드 상태가 끊김: wake_timeout_s 뒤 진행 (거절 안 함)
  ArmRest s;
  s.on_state(0.0, true);
  s.update(0.0, resting_input(), c);
  s.update(1.0, resting_input(), c);
  s.on_state(1.1, false);
  s.wake(3.0);   // on_arm_goal
  EXPECT_FALSE(s.requested());
  bool timed_out = false;
  EXPECT_FALSE(s.arm_ok(3.5, c, &timed_out));
  EXPECT_FALSE(timed_out);
  EXPECT_TRUE(s.arm_ok(4.0, c, &timed_out));
  EXPECT_TRUE(timed_out);
  EXPECT_TRUE(s.arm_ok(4.1, c, &timed_out));
  EXPECT_FALSE(timed_out) << "경고는 한 번";
}

// 보드가 쉬는 팔을 모름 (쉬는 팔 없는 플러그인 · 상태가 한 번도 안 옴): 끄기 요청은 나가도 처짐 · 기다림 없음 = 토크를 늘 켜 두는 흐름
TEST(ArmRest, OldBoardWithoutStateKeepsTodaysFlow)
{
  ArmRest r;
  const auto c = on_cfg();
  r.update(0.0, resting_input(), c);
  EXPECT_TRUE(r.update(1.0, resting_input(), c));
  EXPECT_FALSE(r.slipped());
  r.wake(2.0);
  EXPECT_FALSE(r.waking());
  EXPECT_TRUE(r.arm_ok(2.0, c));
}

// 팔 작업 (on_arm_goal) 은 계획 전에 바로 켜기 요청, rest_after_s 를 처음부터 (거절된 작업 뒤 바로 다시 끄지 않음)
TEST(ArmRest, JobWakesImmediatelyAndRestartsQuietPeriod)
{
  ArmRest r;
  const auto c = on_cfg();
  r.on_state(0.0, true);
  r.update(0.0, resting_input(), c);
  ASSERT_TRUE(r.update(1.0, resting_input(), c));
  r.wake(2.0);
  EXPECT_FALSE(r.requested());
  EXPECT_TRUE(r.waking());
  r.on_state(2.05, true);
  EXPECT_FALSE(r.update(2.1, resting_input(), c)) << "작업이 거절돼도 rest_after_s 동안은 켠 채";
  EXPECT_FALSE(r.update(2.9, resting_input(), c));
  EXPECT_TRUE(r.update(3.15, resting_input(), c));
}

// 끈 뒤 DRIVE 접힘 확인: 엄격 (0.05) 밖 · 느슨 (0.15) 안이면 계속 DRIVE, 느슨 밖이면 STOP (step 의 enforce_drive_guard 와 같은 식)
TEST(ArmRest, LooseFoldWhileSlippedStrictOtherwise)
{
  FoldCheck strict(kHome, {0.05}, 0.3, 0.2), loose(kHome, {0.15}, 0.3, 0.2);
  ArmRest r;
  const auto c = on_cfg();
  MotionArbiter arb(0.5, false);   // DRIVE 로 시작
  auto feed = [&](double t, double dj3) {
      auto q = kHome;
      q[2] += dj3;
      strict.observe(t, q);
      loose.observe(t, q);
    };
  double t = 0.0;
  for (; t < 1.0; t += 0.01) {feed(t, 0.0);}
  r.on_state(t, true);
  r.update(0.0, resting_input(), c);
  ASSERT_TRUE(r.update(t, resting_input(), c));
  r.on_state(t, false);
  // 흔들려 0.1 rad 처짐 (엄격 밖, 느슨 안)
  for (; t < 3.0; t += 0.01) {
    feed(t, 0.1 * std::min(1.0, (t - 1.0) / 0.5));
    const bool folded = strict.confirmed(t) || (r.slipped() && loose.confirmed(t));
    EXPECT_FALSE(arb.enforce_drive_guard(folded, true)) << t;
  }
  EXPECT_FALSE(strict.confirmed(t));
  // 0.2 rad (느슨 밖) → STOP
  bool stopped = false;
  for (; t < 4.0 && !stopped; t += 0.01) {
    feed(t, 0.2);
    const bool folded = strict.confirmed(t) || (r.slipped() && loose.confirmed(t));
    stopped = arb.enforce_drive_guard(folded, true);
  }
  EXPECT_TRUE(stopped);
  EXPECT_EQ(arb.turn(), Turn::STOP);
  // 처짐이 끝난 (다시 앉힌) 뒤에는 엄격만
  MotionArbiter arb2(0.5, false);
  r.moved();
  for (double u = 10.0; u < 11.0; u += 0.01) {feed(u, 0.1);}
  EXPECT_TRUE(arb2.enforce_drive_guard(strict.confirmed(11.0) || (r.slipped() && loose.confirmed(11.0)), true));
}

// 다시 앉히기 고르기: 작은 처짐 = home 한 점 짧게 (≥ rest_reseat_s, 빠르면 늘림) · 큰 처짐 = 검사된 경로 · 아주 작으면 없음
TEST(ArmRest, ReseatChoice)
{
  const double speed = 40.0 * M_PI / 180.0;   // path_speed_dps 40
  auto q = kHome;
  EXPECT_EQ(reseat_choice(q, kHome, 0.005, 0.30, 0.4, speed).kind, Reseat::Kind::NONE);
  q[2] += 0.004;
  EXPECT_EQ(reseat_choice(q, kHome, 0.005, 0.30, 0.4, speed).kind, Reseat::Kind::NONE);
  q[2] = kHome[2] + 0.08;
  q[1] = kHome[1] - 0.02;
  auto r = reseat_choice(q, kHome, 0.005, 0.30, 0.4, speed);
  EXPECT_EQ(r.kind, Reseat::Kind::SHORT);
  EXPECT_EQ(r.worst, 2);
  EXPECT_NEAR(r.slip_rad, 0.08, 1e-12);
  EXPECT_DOUBLE_EQ(r.time_s, 0.4);
  q[2] = kHome[2] + 0.29;
  r = reseat_choice(q, kHome, 0.005, 0.30, 0.4, speed);
  EXPECT_EQ(r.kind, Reseat::Kind::SHORT);
  EXPECT_NEAR(r.time_s, 0.29 / speed, 1e-9) << "40 °/s 보다 빠르면 늘림";
  q[2] = kHome[2] - 0.31;
  EXPECT_EQ(reseat_choice(q, kHome, 0.005, 0.30, 0.4, speed).kind, Reseat::Kind::CHECKED);
  q[0] = std::nan("");
  EXPECT_EQ(reseat_choice(q, kHome, 0.005, 0.30, 0.4, speed).kind, Reseat::Kind::NONE) << "측정 없음 → 지금 규칙";
  EXPECT_EQ(reseat_choice({0.0}, kHome, 0.005, 0.30, 0.4, speed).kind, Reseat::Kind::NONE);
}

TEST(ArmRest, SlipText)
{
  auto q = kHome;
  q[1] += 0.5 * M_PI / 180.0;
  q[2] -= 2.0 * M_PI / 180.0;
  q[4] = std::nan("");
  EXPECT_EQ(slip_text(q, kHome), "J1 +0.0° · J2 +0.5° · J3 -2.0° · J4 +0.0° · J5 ?");
}
