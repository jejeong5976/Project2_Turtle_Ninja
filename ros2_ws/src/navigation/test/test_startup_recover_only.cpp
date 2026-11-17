// test_startup_recover_only.cpp — recover_only (autostart 먼저, 실패한 manager 만 복구) 계산 시험. ROS 없음.
//   autostart 성공 → 명령 0 · abort (is_active false) → RESET 1 번 · 멈춘 manager (무응답) → 프로세스 재시작 → autostart 다시
//   navigation 은 입력 (localization · 지도 · map→odom) 전엔 시간 안 셈 · ACTIVE 인데 map→odom 없음 → RESET
//   + /proc cmdline · environ 읽기, 구독 다시 만들기 (ResubscribeWatch) 규칙
#include <gtest/gtest.h>

#include <optional>
#include <string>
#include <utility>

#include "navigation/lifecycle_recovery_core.hpp"
#include "navigation/resubscribe_watch.hpp"

using namespace navigation::lifecycle;   // NOLINT
using navigation::ResubscribeWatch;

namespace
{
constexpr int kNone = -1;   // 관측 없음

RecoveryOptions ropts()
{
  RecoveryOptions o;
  o.recover_only = true;
  o.stable_s = 1;
  o.backoff_s = 0;
  o.command_timeout = 2;
  o.node_timeout = 2;
  o.drain_timeout = 30;
  o.verify_timeout = 4;
  o.cleanup_timeout = 8;
  o.autostart_grace_s = 10;
  o.abort_confirm_s = 3;
  o.hang_s = 4;
  o.max_restarts = 2;
  return o;
}

LifecycleRecovery rcore(RecoveryOptions o = ropts())
{
  return LifecycleRecovery("/lifecycle_manager_localization", {"map", "amcl"}, o);
}

struct S
{
  int active = kNone;                     // is_active 응답 (kNone = 무응답)
  std::pair<int, int> nodes{kNone, kNone};
  bool inputs = true;
  bool manager = true;
  bool output = true;
};

Snapshot snap(double now, S a = {})
{
  Snapshot s;
  s.inputs_ready = a.inputs;
  s.services_ready = a.manager;
  s.manager_ready = a.manager;
  s.output_ok = a.output;
  s.output_reason = "map→odom TF (AMCL)";
  s.input_reason = "입력 기다림";
  if (a.active != kNone) {
    s.active = Observation{a.active, now, now};
  }
  if (a.nodes.first != kNone) {
    s.states["map"] = Observation{a.nodes.first, now, now};
  }
  if (a.nodes.second != kNone) {
    s.states["amcl"] = Observation{a.nodes.second, now, now};
  }
  return s;
}

S active_all()
{
  S a;
  a.active = 1;
  a.nodes = {ACTIVE, ACTIVE};
  return a;
}

S aborted(std::pair<int, int> nodes = {ACTIVE, UNCONFIGURED})
{
  S a;
  a.active = 0;
  a.nodes = nodes;
  return a;
}

// t0..t1 를 0.5 s 마다 돌리고 처음 나온 요청을 돌려줌
std::optional<Request> run(LifecycleRecovery & c, double t0, double t1, const S & a)
{
  for (double t = t0; t <= t1 + 1e-9; t += 0.5) {
    auto r = c.tick(t, snap(t, a));
    if (r) {
      return r;
    }
  }
  return std::nullopt;
}

// RESET 뒤 정합 → STARTUP 까지 (map ACTIVE · amcl UNCONFIGURED 에서 시작)
void finish_reset(LifecycleRecovery & c, double & t, const Request & reset)
{
  EXPECT_EQ(reset.command, RESET);
  c.complete(reset.token, false, t);   // Nav2 RESET 은 섞인 상태면 false 가 흔함 → 노드별 정합으로 이어 감
  t += 0.5;
  auto d = c.tick(t, snap(t, aborted({ACTIVE, UNCONFIGURED})));   // 뒤 노드 (amcl) 는 이미 UNCONFIGURED
  if (!d) {
    t += 0.5;
    d = c.tick(t, snap(t, aborted({ACTIVE, UNCONFIGURED})));
  }
  ASSERT_TRUE(d);
  EXPECT_EQ(d->target, "map");
  EXPECT_EQ(d->command, DEACTIVATE);
  c.complete(d->token, true, t);
  t += 0.5;
  auto cl = c.tick(t, snap(t, aborted({INACTIVE, UNCONFIGURED})));
  ASSERT_TRUE(cl);
  EXPECT_EQ(cl->command, CLEANUP);
  c.complete(cl->token, true, t);
  std::optional<Request> st;
  for (int i = 0; i < 6 && !st; ++i) {
    t += 0.5;
    st = c.tick(t, snap(t, aborted({UNCONFIGURED, UNCONFIGURED})));
  }
  ASSERT_TRUE(st);
  EXPECT_EQ(st->command, STARTUP);
  EXPECT_EQ(st->kind, "manager");
  c.complete(st->token, true, t);
}
}  // namespace

TEST(RecoverOnly, AutostartSuccessSendsNothing)
{
  auto c = rcore();
  EXPECT_EQ(c.phase, "waiting_autostart");
  // autostart 중: manager 가 is_active 에 답 안 함 (같은 callback group) → 기다림
  EXPECT_FALSE(run(c, 0, 5, S{}));
  EXPECT_FALSE(run(c, 5.5, 8, active_all()));
  EXPECT_EQ(c.phase, "ready");
  EXPECT_EQ(c.recoveries, 0);
  EXPECT_EQ(c.restarts, 0);
  EXPECT_FALSE(run(c, 8.5, 60, S{}));   // 준비 뒤엔 손 안 댐
}

TEST(RecoverOnly, ManagerActiveIsTrustedWhenNodesUnobserved)
{
  auto c = rcore();
  S a;
  a.active = 1;   // 감시기가 노드 get_state 를 못 봐도 manager true 면 정상
  EXPECT_FALSE(run(c, 0, 3, a));
  EXPECT_EQ(c.phase, "ready");
  // 본 노드가 ACTIVE 아님이면 정상 아님
  auto d = rcore();
  a.nodes = {ACTIVE, INACTIVE};
  EXPECT_FALSE(d.healthy(snap(1, a), 1));
}

TEST(RecoverOnly, ShortFalseBeforeAutostartIsNotFailure)
{
  auto c = rcore();
  EXPECT_FALSE(run(c, 0, 2, aborted({UNCONFIGURED, UNCONFIGURED})));   // 3 s 안 됨
  EXPECT_FALSE(run(c, 2.5, 5, active_all()));
  EXPECT_EQ(c.phase, "ready");
  EXPECT_EQ(c.recoveries, 0);
}

TEST(RecoverOnly, AbortedAutostartResetsOnlyThatManager)
{
  auto c = rcore();
  EXPECT_FALSE(run(c, 0, 2, S{}));                 // autostart 중 (무응답)
  auto r = run(c, 2.5, 20, aborted());            // Aborting bringup → is_active false
  ASSERT_TRUE(r);
  EXPECT_EQ(r->kind, "manager");
  EXPECT_EQ(r->target, "/lifecycle_manager_localization");
  EXPECT_EQ(r->command, RESET);
  EXPECT_LE(r->sent, 2.5 + 3 + 0.6);               // false 3 s 이어지면 바로 (grace 10 s 안 기다림)
  EXPECT_EQ(c.recoveries, 1);
  double t = r->sent;
  finish_reset(c, t, *r);
  EXPECT_FALSE(run(c, t + 0.5, t + 3, active_all()));
  EXPECT_EQ(c.phase, "ready");
  EXPECT_EQ(c.recoveries, 1);
  EXPECT_EQ(c.restarts, 0);
}

TEST(RecoverOnly, AnsweringManagerWithInactiveNodesAfterGraceIsReset)
{
  auto c = rcore();
  S a;
  a.active = 1;
  a.nodes = {ACTIVE, INACTIVE};   // manager 는 답하는데 amcl 이 안 켜짐 (verifying 들어가지 않음)
  auto r = run(c, 0, 30, a);
  ASSERT_TRUE(r);
  EXPECT_EQ(r->command, RESET);
  EXPECT_GE(r->sent, 10.0);
}

TEST(RecoverOnly, HungManagerIsRestartedThenAutostartsAgain)
{
  auto c = rcore();
  auto r = run(c, 0, 30, S{});   // 서비스는 보이는데 is_active 무응답 (change_state 응답 유실로 멈춤)
  ASSERT_TRUE(r);
  EXPECT_EQ(r->kind, "process");
  EXPECT_EQ(r->purpose, "RESTART");
  EXPECT_EQ(r->target, "/lifecycle_manager_localization");
  EXPECT_GE(r->sent, 10.0);
  EXPECT_EQ(c.restarts, 1);
  EXPECT_EQ(c.recoveries, 0);
  c.complete(r->token, true, r->sent);
  EXPECT_EQ(c.phase, "waiting_autostart");
  // 다시 뜬 manager: 노드가 이미 켜져 있어 autostart 가 configure 에서 실패 → false → RESET 차례
  const double t0 = r->sent + 0.5;
  EXPECT_FALSE(run(c, t0, t0 + 2, S{}));
  auto reset = run(c, t0 + 2.5, t0 + 20, aborted());
  ASSERT_TRUE(reset);
  double t = reset->sent;
  finish_reset(c, t, *reset);
  EXPECT_FALSE(run(c, t + 0.5, t + 3, active_all()));
  EXPECT_EQ(c.phase, "ready");
  EXPECT_EQ(c.restarts, 1);
  EXPECT_EQ(c.recoveries, 1);
}

TEST(RecoverOnly, RestartIgnoresAnswersRequestedBeforeRestart)
{
  auto c = rcore();
  auto r = run(c, 0, 30, S{});
  ASSERT_TRUE(r);
  c.complete(r->token, true, 20);
  // 재시작 전에 요청한 false 응답 (이전 manager) 은 abort 로 세지 않음
  Snapshot old = snap(21, aborted());
  old.active->requested = 19;
  for (double t = 21; t < 26; t += 0.5) {
    old.active->received = t;
    EXPECT_FALSE(c.tick(t, old));
  }
  EXPECT_EQ(c.phase, "waiting_autostart");
}

TEST(RecoverOnly, RestartLimitThenReportOnly)
{
  auto c = rcore();
  double t = 0;
  for (int i = 0; i < 2; ++i) {
    auto r = run(c, t, t + 30, S{});
    ASSERT_TRUE(r);
    EXPECT_EQ(r->kind, "process");
    c.complete(r->token, true, r->sent);
    t = r->sent + 0.5;
  }
  EXPECT_FALSE(run(c, t, t + 60, S{}));
  EXPECT_EQ(c.phase, "exhausted");
  EXPECT_EQ(c.restarts, 2);
}

TEST(RecoverOnly, RestartFailureWhenProcessMissingIsReported)
{
  auto c = rcore();
  auto r = run(c, 0, 30, S{});
  ASSERT_TRUE(r);
  c.complete(r->token, false, r->sent, "프로세스 못 찾음");
  EXPECT_EQ(c.phase, "exhausted");
  EXPECT_EQ(c.last_error, "프로세스 못 찾음");
}

TEST(RecoverOnly, WatchdogCommandLostThenManagerHungIsRestarted)
{
  auto c = rcore();
  auto reset = run(c, 0, 20, aborted());
  ASSERT_TRUE(reset);
  // RESET 응답이 오지 않고 manager 가 is_active 에도 답 안 함 → 기한 (2 s) + hang_s (4 s) 뒤 프로세스 재시작
  auto r = run(c, reset->sent + 0.5, reset->sent + 20, S{});
  ASSERT_TRUE(r);
  EXPECT_EQ(r->kind, "process");
  EXPECT_NEAR(r->sent - reset->sent, 2 + 4, 1.01);
  EXPECT_FALSE(c.complete(reset->token, true, r->sent + 1));   // 늦게 온 이전 RESET 응답은 무시
}

TEST(RecoverOnly, NavigationClockStartsOnlyAfterInputs)
{
  auto o = ropts();
  auto c = LifecycleRecovery("/lifecycle_manager_navigation", {"planner_server", "collision_monitor"}, o);
  S wait;
  wait.inputs = false;   // localization 준비 · 지도 · map→odom 전: planner activate 가 TF 를 기다리는 게 정상
  EXPECT_FALSE(run(c, 0, 120, wait));
  S ab;
  ab.active = 0;
  ab.nodes = {UNCONFIGURED, UNCONFIGURED};
  ab.inputs = false;
  EXPECT_FALSE(run(c, 120.5, 200, ab));   // 이미 abort 했어도 입력 전엔 안 셈
  ab.inputs = true;
  auto r = run(c, 200.5, 220, ab);
  ASSERT_TRUE(r);
  EXPECT_EQ(r->command, RESET);
  EXPECT_GE(r->sent, 203.5);
}

TEST(RecoverOnly, LocalizationGateIgnoresInputsWhenConfigured)
{
  auto o = ropts();
  o.autostart_needs_inputs = false;
  auto c = rcore(o);
  S a = aborted();
  a.inputs = false;   // 감시기 /scan 구독이 늦어도 AMCL lifecycle 판단은 함
  auto r = run(c, 0, 20, a);
  ASSERT_TRUE(r);
  EXPECT_EQ(r->command, RESET);
}

TEST(RecoverOnly, ActiveLocalizationWithoutMapOdomIsReset)
{
  auto o = ropts();
  o.output_timeout_s = 5;
  auto c = rcore(o);
  S a = active_all();
  a.output = false;   // AMCL ACTIVE 인데 map→odom TF 없음 (AMCL /map · /scan 구독 유실)
  auto r = run(c, 0, 20, a);
  ASSERT_TRUE(r);
  EXPECT_EQ(r->command, RESET);
  EXPECT_GE(r->sent, 5.0);
  EXPECT_LE(r->sent, 6.5);
  EXPECT_NE(c.phase, "ready");
}

TEST(RecoverOnly, LateMapOdomWithinWindowIsFine)
{
  auto o = ropts();
  o.output_timeout_s = 5;
  auto c = rcore(o);
  S a = active_all();
  a.output = false;
  EXPECT_FALSE(run(c, 0, 4, a));
  EXPECT_FALSE(run(c, 4.5, 7, active_all()));
  EXPECT_EQ(c.phase, "ready");
  EXPECT_EQ(c.recoveries, 0);
}

TEST(RecoverOnly, StatusJsonCarriesRestarts)
{
  auto c = rcore();
  WatchdogStatus st;
  st.groups.emplace_back("localization", &c);
  std::string out;
  write_watchdog_status(out, st);
  EXPECT_NE(out.find("\"restarts\": 0"), std::string::npos);
  EXPECT_NE(out.find("\"phase\": \"waiting_autostart\""), std::string::npos);
}

TEST(ProcessMatch, CmdlineNeedsExactNodeRemap)
{
  using namespace std::string_literals;
  const std::string cmd =
    "/opt/ros/lyrical/lib/nav2_lifecycle_manager/lifecycle_manager\0--ros-args\0-r\0"
    "__node:=lifecycle_manager_localization\0--params-file\0/tmp/x.yaml\0"s;
  EXPECT_TRUE(cmdline_has_node(cmd, "lifecycle_manager_localization"));
  EXPECT_FALSE(cmdline_has_node(cmd, "lifecycle_manager_navigation"));
  EXPECT_FALSE(cmdline_has_node(cmd, "lifecycle_manager"));
  EXPECT_FALSE(cmdline_has_node(cmd, ""));
  // ros2 CLI · launch 자체는 __node 가 없음
  EXPECT_FALSE(cmdline_has_node("/usr/bin/python3\0/opt/ros/lyrical/bin/ros2\0launch\0bringup\0"s,
    "lifecycle_manager_localization"));
}

TEST(ProcessMatch, DomainFromEnviron)
{
  using namespace std::string_literals;
  EXPECT_EQ(environ_domain("HOME=/home/x\0ROS_DOMAIN_ID=17\0PATH=/bin\0"s), "17");
  EXPECT_EQ(environ_domain("HOME=/home/x\0PATH=/bin\0"s), "0");
  EXPECT_EQ(environ_domain("ROS_DOMAIN_ID=\0"s), "0");
  EXPECT_EQ(normalize_domain(nullptr), "0");
  EXPECT_EQ(normalize_domain(" 187 "), "187");
  EXPECT_NE(environ_domain("ROS_DOMAIN_ID=187\0"s), normalize_domain("17"));
}

TEST(ResubscribeWatchTest, QuietTopicIsResubscribedWithBackoff)
{
  ResubscribeWatch w(5.0, 30.0);
  w.start(0);
  EXPECT_FALSE(w.due(4.9));
  EXPECT_TRUE(w.due(5.0));      // 1 번째: 5 s
  EXPECT_FALSE(w.due(14.9));
  EXPECT_TRUE(w.due(15.0));     // 2 번째: 10 s 뒤
  EXPECT_FALSE(w.due(34.9));
  EXPECT_TRUE(w.due(35.0));     // 3 번째: 20 s 뒤
  EXPECT_TRUE(w.due(65.0));     // 최대 30 s
  EXPECT_EQ(w.count(), 4);
  EXPECT_FALSE(w.ever());
  w.seen(66);                   // 받으면 처음 5 s 로
  EXPECT_FALSE(w.due(70.9));
  EXPECT_TRUE(w.due(71.0));
  EXPECT_DOUBLE_EQ(w.since_msg(71.0), 5.0);
}

TEST(ResubscribeWatchTest, FlowingTopicIsNeverResubscribed)
{
  ResubscribeWatch w(5.0, 60.0);
  w.start(0);
  for (double t = 0.1; t < 100; t += 0.1) {
    w.seen(t);
    EXPECT_FALSE(w.due(t + 0.05));
  }
  EXPECT_EQ(w.count(), 0);
}

TEST(ResubscribeWatchTest, RestartingWatchKeepsBackoff)
{
  ResubscribeWatch w(5.0, 60.0);
  w.start(0);
  EXPECT_TRUE(w.due(5));
  w.start(5);                   // 구독을 다시 만들며 start → 기다림은 2 배 유지
  EXPECT_FALSE(w.due(14.9));
  EXPECT_TRUE(w.due(15));
  ResubscribeWatch off(0.0);
  EXPECT_FALSE(off.due(1e9));
}
