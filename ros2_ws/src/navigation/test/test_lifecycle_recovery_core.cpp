// test_lifecycle_recovery_core.cpp — test_lifecycle_recovery.py 를 옮김.
// ROS 없이 시작 순서·부분 ACTIVE·응답 지연·복구 횟수를 검증함.
#include <gtest/gtest.h>

#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "navigation/lifecycle_recovery_core.hpp"

using namespace navigation::lifecycle;   // NOLINT

namespace
{
struct SnapArgs
{
  std::pair<int, int> values{UNCONFIGURED, UNCONFIGURED};
  bool active = false;
  bool inputs = true;
  bool services = true;
  std::optional<double> requested;
};

Snapshot snapshot(double now, SnapArgs a = {})
{
  const double requested = a.requested ? *a.requested : now;
  Snapshot s;
  s.inputs_ready = a.inputs;
  s.services_ready = a.services;
  s.input_reason = "scan/TF 기다림";
  s.active = Observation{a.active ? 1 : 0, requested, now};
  s.states["map"] = Observation{a.values.first, requested, now};
  s.states["amcl"] = Observation{a.values.second, requested, now};
  return s;
}

Snapshot snapshot(double now, std::pair<int, int> values, bool active = false)
{
  SnapArgs a;
  a.values = values;
  a.active = active;
  return snapshot(now, a);
}

// Snapshot(True, True) (active · states 없음)
Snapshot bare()
{
  Snapshot s;
  s.inputs_ready = true;
  s.services_ready = true;
  return s;
}

SnapArgs req(double requested, std::pair<int, int> values = {UNCONFIGURED, UNCONFIGURED})
{
  SnapArgs a;
  a.values = values;
  a.requested = requested;
  return a;
}

RecoveryOptions opts()
{
  RecoveryOptions o;
  o.stable_s = 1;
  o.backoff_s = 0;
  o.command_timeout = 2;
  o.node_timeout = 2;
  o.drain_timeout = 3;
  o.verify_timeout = 4;
  o.cleanup_timeout = 8;
  return o;
}

LifecycleRecovery core(RecoveryOptions o = opts())
{
  return LifecycleRecovery("localization", {"map", "amcl"}, o);
}

const std::pair<int, int> kAA{ACTIVE, ACTIVE};
const std::pair<int, int> kAI{ACTIVE, INACTIVE};
const std::pair<int, int> kAU{ACTIVE, UNCONFIGURED};
const std::pair<int, int> kIU{INACTIVE, UNCONFIGURED};
}  // namespace

TEST(LifecycleRecovery, WaitsForScanTfServicesAndRealState)
{
  auto c = core();
  for (int t = 0; t < 10; ++t) {
    SnapArgs a;
    a.inputs = false;
    EXPECT_FALSE(c.tick(t, snapshot(t, a)));
  }
  SnapArgs ns;
  ns.services = false;
  EXPECT_FALSE(c.tick(10, snapshot(10, ns)));
  EXPECT_FALSE(c.tick(11, bare()));
  EXPECT_EQ(c.recoveries, 0);
  auto r = c.tick(12, snapshot(12));
  ASSERT_TRUE(r);
  EXPECT_EQ(r->command, STARTUP);
}

TEST(LifecycleRecovery, StartupIsNotReadyUntilManagerNodesAndBondStable)
{
  auto c = core();
  auto r = c.tick(0, snapshot(0));
  c.complete(r->token, true, .1);
  c.tick(.2, snapshot(.2, kAI, true));
  EXPECT_NE(c.phase, "ready");
  c.tick(.3, snapshot(.3, kAA, true));
  c.tick(1.29, snapshot(1.29, kAA, true));
  EXPECT_NE(c.phase, "ready");
  c.tick(1.3, snapshot(1.3, kAA, true));
  EXPECT_EQ(c.phase, "ready");
  // 시작 완료 뒤 운영 중 lifecycle을 새로 건드리지 않음.
  EXPECT_FALSE(c.tick(9, snapshot(9)));
  EXPECT_EQ(c.phase, "ready");
}

TEST(LifecycleRecovery, PartialActiveResetFalseDeactivatesAndCleansReverseOrder)
{
  auto c = core();
  auto r = c.tick(0, snapshot(0));
  c.complete(r->token, false, .1);
  r = c.tick(.2, snapshot(.2, kAI));
  ASSERT_TRUE(r);
  EXPECT_EQ(r->command, RESET);
  c.complete(r->token, false, .3);  // manager가 inactive amcl을 deactivate하다 실패한 장면임.
  c.tick(.4, snapshot(.4, req(.1, kAI)));
  EXPECT_FALSE(c.pending);  // reset 전 오래된 상태로 바로 전환하지 않음.
  r = c.tick(.5, snapshot(.5, kAI));
  ASSERT_TRUE(r);
  EXPECT_EQ(std::make_pair(r->target, r->command), std::make_pair(std::string("amcl"), CLEANUP));
  c.complete(r->token, true, .6);
  c.tick(.7, snapshot(.7, kAU));
  r = c.tick(.8, snapshot(.8, kAU));
  ASSERT_TRUE(r);
  EXPECT_EQ(std::make_pair(r->target, r->command), std::make_pair(std::string("map"), DEACTIVATE));
  c.complete(r->token, true, .9);
  r = c.tick(1.0, snapshot(1.0, kIU));
  ASSERT_TRUE(r);
  EXPECT_EQ(std::make_pair(r->target, r->command), std::make_pair(std::string("map"), CLEANUP));
  c.complete(r->token, true, 1.1);
  c.tick(1.2, snapshot(1.2));
  c.tick(1.3, snapshot(1.3));
  r = c.tick(1.4, snapshot(1.4));
  ASSERT_TRUE(r);
  EXPECT_EQ(r->command, STARTUP);
  EXPECT_EQ(c.recoveries, 1);
}

TEST(LifecycleRecovery, InitialPartialStateUsesResetNotInvalidConfigure)
{
  auto c = core();
  EXPECT_FALSE(c.tick(0, snapshot(0, kAU)));
  auto r = c.tick(.1, snapshot(.1, kAU));
  ASSERT_TRUE(r);
  EXPECT_EQ(r->command, RESET);
}

TEST(LifecycleRecovery, TimeoutKeepsPendingUntilRemoteCompletion)
{
  auto c = core();
  auto r = c.tick(0, snapshot(0));
  EXPECT_FALSE(c.tick(2.1, snapshot(2.1, req(.5))));
  EXPECT_EQ(c.pending->token, r->token);
  EXPECT_FALSE(c.tick(2.2, snapshot(2.2, req(1.9))));
  EXPECT_EQ(c.recoveries, 0);
  // 원래 응답이 늦게 성공하면 RESET을 겹쳐 보내지 않고 검증함.
  c.complete(r->token, true, 2.3);
  EXPECT_EQ(c.phase, "verifying");
}

TEST(LifecycleRecovery, TimeoutBarrierThenOldCallbackDoesNotOverrideReset)
{
  auto c = core();
  auto old = c.tick(0, snapshot(0));
  c.tick(2.1, snapshot(2.1, req(1.0)));
  c.tick(2.2, snapshot(2.2, req(2.15)));  // timeout 뒤 is_active 응답
  auto r = c.tick(2.3, snapshot(2.3));
  ASSERT_TRUE(r);
  EXPECT_EQ(r->command, RESET);
  EXPECT_FALSE(c.complete(old->token, true, 2.4));
  EXPECT_EQ(c.pending->token, r->token);
  EXPECT_EQ(c.phase, "resetting");
}

TEST(LifecycleRecovery, NoResponseDrainIsBoundedAndDoesNotIssueParallelReset)
{
  auto c = core();
  auto r = c.tick(0, snapshot(0));
  c.tick(2.1, snapshot(2.1, req(0)));
  EXPECT_FALSE(c.tick(5.2, bare()));
  EXPECT_EQ(c.phase, "exhausted");
  EXPECT_FALSE(c.tick(9, snapshot(9)));
  EXPECT_FALSE(c.complete(r->token, true, 10));
  EXPECT_EQ(c.phase, "exhausted");
}

TEST(LifecycleRecovery, ThreeRecoveryCyclesMaximum)
{
  auto c = core();
  double now = 0;
  int starts = 0, resets = 0;
  for (int i = 0; i < 200; ++i) {
    auto r = c.tick(now, snapshot(now));
    if (r) {
      if (r->command == STARTUP) {
        starts += 1;
        c.complete(r->token, false, now + .01);
      } else if (r->command == RESET) {
        resets += 1;
        c.complete(r->token, false, now + .01);
      } else {
        FAIL() << "전부 unconfigured라 직접 transition 불필요";
      }
    }
    now += .1;
    if (c.phase == "exhausted") {
      break;
    }
  }
  EXPECT_EQ(starts, 4);
  EXPECT_EQ(resets, 3);
  EXPECT_EQ(c.recoveries, 3);
  EXPECT_EQ(c.phase, "exhausted");
}

TEST(LifecycleRecovery, BondDropsDuringVerificationRetries)
{
  auto o = opts();
  o.drain_timeout = 30;
  auto c = core(o);
  auto r = c.tick(0, snapshot(0));
  c.complete(r->token, true, .1);
  c.tick(.2, snapshot(.2, kAA, true));
  c.tick(.4, snapshot(.4, kAA, false));
  EXPECT_EQ(c.phase, "waiting_native");
  c.tick(.5, snapshot(.5));
  EXPECT_FALSE(c.tick(11.3, snapshot(11.3)));
  EXPECT_FALSE(c.tick(11.5, snapshot(11.5, req(11.3))));
  EXPECT_EQ(c.phase, "waiting_native");
  c.tick(11.6, snapshot(11.6));
  r = c.tick(11.7, snapshot(11.7));
  ASSERT_TRUE(r);
  EXPECT_EQ(r->command, RESET);
}

TEST(LifecycleRecovery, FalseNodeResponseRechecksActualState)
{
  auto c = core();
  c.tick(0, snapshot(0, kAI));
  auto r = c.tick(.1, snapshot(.1));
  c.complete(r->token, false, .2);
  r = c.tick(.3, snapshot(.3, kAI));
  ASSERT_TRUE(r);
  EXPECT_EQ(r->command, CLEANUP);
  c.complete(r->token, false, .4);
  // 응답 false여도 실제 unconfigured이면 중복cleanup 없이 다음 노드로 감.
  c.tick(.5, snapshot(.5, kAU));
  r = c.tick(.6, snapshot(.6, kAU));
  ASSERT_TRUE(r);
  EXPECT_EQ(std::make_pair(r->target, r->command), std::make_pair(std::string("map"), DEACTIVATE));
}

TEST(LifecycleRecovery, NodeTransitionFailureIsBounded)
{
  auto c = core();
  double now = 0;
  int changes = 0;
  for (int i = 0; i < 400; ++i) {
    auto r = c.tick(now, snapshot(now, kAI));
    if (r) {
      changes += r->kind == "node" ? 1 : 0;
      c.complete(r->token, false, now + .01);
    }
    now += .1;
    if (c.phase == "exhausted") {
      break;
    }
  }
  EXPECT_EQ(c.phase, "exhausted");
  EXPECT_EQ(c.recoveries, 3);
  EXPECT_EQ(changes, 6);
}

TEST(LifecycleRecovery, SensorLossDoesNotResetAHealthyManager)
{
  auto c = core();
  auto r = c.tick(0, snapshot(0));
  c.complete(r->token, true, .1);
  c.tick(.2, snapshot(.2, kAA, true));
  for (double now : {1., 3., 8.}) {
    SnapArgs a;
    a.values = kAA;
    a.active = true;
    a.inputs = false;
    EXPECT_FALSE(c.tick(now, snapshot(now, a)));
  }
  EXPECT_EQ(c.phase, "verifying");
  EXPECT_EQ(c.recoveries, 0);
}

TEST(LifecycleRecovery, EarlierGroupCanRecoverBeforeGlobalStartupHandoff)
{
  auto o = opts();
  o.stable_s = 0;
  o.drain_timeout = 30;
  auto c = core(o);
  c.tick(0, snapshot(0, kAA, true));
  c.tick(.1, snapshot(.1, kAA, true));
  EXPECT_EQ(c.phase, "ready");
  c.recheck_before_handoff(.2, snapshot(.2, kAI, false));
  EXPECT_EQ(c.phase, "waiting_native");
  c.tick(.25, snapshot(.25, kAI));
  c.tick(11.3, snapshot(11.3, kAI));
  auto r = c.tick(11.4, snapshot(11.4, kAI));
  ASSERT_TRUE(r);
  EXPECT_EQ(r->command, RESET);
}

TEST(LifecycleRecovery, NativeReconnectSuccessNeedsNoWatchdogReset)
{
  auto c = core();
  auto r = c.tick(0, snapshot(0));
  c.complete(r->token, true, .1);
  c.tick(.2, snapshot(.2, kAA, true));
  c.tick(.4, snapshot(.4, kAA, false));
  EXPECT_EQ(c.phase, "waiting_native");
  EXPECT_FALSE(c.tick(1, snapshot(1)));
  EXPECT_FALSE(c.tick(2, snapshot(2, kAA, true)));
  c.tick(2.1, snapshot(2.1, kAA, true));
  c.tick(3.1, snapshot(3.1, kAA, true));
  EXPECT_EQ(c.phase, "ready");
  EXPECT_EQ(c.recoveries, 0);
}

TEST(LifecycleRecovery, NativeReconnectUnresponsiveManagerNeverOverlapsReset)
{
  auto c = core();
  auto r = c.tick(0, snapshot(0));
  c.complete(r->token, true, .1);
  c.tick(.2, snapshot(.2, kAA, true));
  c.tick(.4, snapshot(.4, kAA, false));
  EXPECT_FALSE(c.tick(15, bare()));
  EXPECT_EQ(c.phase, "exhausted");
  EXPECT_EQ(c.recoveries, 0);
}

TEST(LifecycleRecovery, SlowNativeHardResetStartsReconnectWindowAfterResponse)
{
  auto o = opts();
  o.drain_timeout = 120;
  auto c = core(o);
  auto r = c.tick(0, snapshot(0));
  c.complete(r->token, true, .1);
  c.tick(.2, snapshot(.2, kAA, true));
  c.tick(.4, bare());
  EXPECT_EQ(c.phase, "waiting_native");
  EXPECT_FALSE(c.tick(20, bare()));
  EXPECT_FALSE(c.tick(30, snapshot(30)));
  EXPECT_FALSE(c.tick(40, snapshot(40)));
  EXPECT_EQ(c.phase, "waiting_native");
  c.tick(41.1, snapshot(41.1));
  r = c.tick(41.2, snapshot(41.2));
  ASSERT_TRUE(r);
  EXPECT_EQ(r->command, RESET);
}

TEST(LifecycleRecovery, NativeRecoveredWithStaleScanIsNotReset)
{
  auto c = core();
  auto r = c.tick(0, snapshot(0));
  c.complete(r->token, true, .1);
  c.tick(.2, snapshot(.2, kAA, true));
  SnapArgs off;
  off.active = false;
  c.tick(.4, snapshot(.4, off));
  SnapArgs a;
  a.values = kAA;
  a.active = true;
  a.inputs = false;
  c.tick(1, snapshot(1, a));
  EXPECT_EQ(c.phase, "verifying");
  EXPECT_EQ(c.recoveries, 0);
}

TEST(LifecycleRecovery, FinalizedOrTransitioningNodeWaitsBoundedWithoutInvalidTransition)
{
  auto o = opts();
  o.cleanup_timeout = .5;
  auto c = core(o);
  const std::pair<int, int> odd{4, 10};
  c.tick(0, snapshot(0, odd));
  auto r = c.tick(.1, snapshot(.1, odd));
  c.complete(r->token, false, .2);
  EXPECT_FALSE(c.tick(.3, snapshot(.3, odd)));
  EXPECT_FALSE(c.tick(.8, snapshot(.8, odd)));
  EXPECT_EQ(c.phase, "waiting_reset");
}

// C++ 판 추가: 상태 JSON 이 파이썬 json.dumps(sort_keys=True, ensure_ascii=False) 와 같은 글자
TEST(LifecycleRecovery, WatchdogStatusJsonMatchesPython)
{
  auto loc = core();
  auto nav = LifecycleRecovery("/lifecycle_manager_navigation", {"planner_server", "collision_monitor"});
  loc.tick(0, snapshot(0));   // STARTUP 보냄 → pending
  WatchdogStatus st;
  st.ready = false;
  st.finished = false;
  st.localization_ready = false;
  st.navigation_ready = false;
  st.scan = true;
  st.odom_tf = true;
  st.map = false;
  st.map_odom_tf = false;
  st.groups = {{"localization", &loc}, {"navigation", &nav}};
  std::string out;
  write_watchdog_status(out, st);
  // 기대값 = 같은 상태로 python3 json.dumps(status, ensure_ascii=False, sort_keys=True)
  const std::string expected =
    "{\"groups\": {\"localization\": {\"last_error\": \"\", \"pending\": {\"command\": 0, \"kind\": \"manager\", "
    "\"target\": \"localization\", \"timed_out\": false, \"token\": 1}, \"phase\": \"starting\", \"ready\": false, "
    "\"reason\": \"STARTUP\", \"recovery_attempts\": 0}, \"navigation\": {\"last_error\": \"\", \"pending\": null, "
    "\"phase\": \"waiting_inputs\", \"ready\": false, \"reason\": \"시작 입력 기다림\", \"recovery_attempts\": 0}}, "
    "\"inputs\": {\"map\": false, \"map_odom_tf\": false, \"odom_tf\": true, \"scan\": true}, "
    "\"last_error\": {\"localization\": \"\", \"navigation\": \"\"}, \"localization_ready\": false, "
    "\"monitor_only_after_startup\": false, \"navigation_ready\": false, \"pending\": {\"localization\": "
    "{\"command\": 0, \"kind\": \"manager\", \"target\": \"localization\", \"timed_out\": false, \"token\": 1}, "
    "\"navigation\": null}, \"phase\": \"starting\", \"ready\": false, \"recovery_attempts\": {\"localization\": 0, "
    "\"navigation\": 0}, \"startup_completed\": false}";
  EXPECT_EQ(out, expected);
}
