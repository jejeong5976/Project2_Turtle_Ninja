// test_arm.cpp — 팔 쪽 순수 계산 시험 (arm.hpp: 순서표 · 차례 · 팔 작업 진행)
// colcon test --packages-select manipulation

#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include "manipulation/arm.hpp"

// ============================================================================ ① 순서표

using manipulation::make_arm_plan;

// 1) 모든 작업은 "접기(home)" 로 끝남 → 끝나자마자 주행해도 안전
TEST(ArmPlan, EveryTaskEndsFolded)
{
  for (uint8_t task : {manipulation::kArmHome, manipulation::kArmPick, manipulation::kArmPlace}) {
    auto p = make_arm_plan(task, 0.019, -0.005);
    ASSERT_FALSE(p.empty());
    EXPECT_FALSE(p.back().is_gripper);
    EXPECT_EQ(p.back().pose, "home");
  }
}

// 2) 집기: 손을 벌린 다음에 내려가고, 내려간 다음에 쥠
TEST(ArmPlan, PickOrder)
{
  auto p = make_arm_plan(manipulation::kArmPick, 0.019, -0.005);
  int open = -1, down = -1, close = -1;
  for (int i = 0; i < static_cast<int>(p.size()); ++i) {
    if (p[i].is_gripper && p[i].gripper > 0.0 && open < 0) {open = i;}
    if (!p[i].is_gripper && p[i].pose == "pick_down") {down = i;}
    if (p[i].is_gripper && p[i].gripper < 0.0) {close = i;}
  }
  EXPECT_LT(open, down);
  EXPECT_LT(down, close);
}

// 3) 동작 하나는 팔 아니면 그리퍼 (동시에 둘 다 X — 한 번에 한 무리)
TEST(ArmPlan, OneThingPerAction)
{
  for (const auto & a : make_arm_plan(manipulation::kArmPlace, 0.019, -0.005)) {
    if (a.is_gripper) {EXPECT_TRUE(a.pose.empty());} else {EXPECT_FALSE(a.pose.empty());}
  }
}

// 4) 모르는 번호는 빈 목록 (control_master 가 거절)
TEST(ArmPlan, UnknownTask)
{
  EXPECT_TRUE(make_arm_plan(7, 0.019, -0.005).empty());
}

// ============================================================================ ② 차례
// 켤 때 잠금 · 요청 번호 · "바퀴가 잠갔다" 답이 있어야 ARM

using manipulation::MotionArbiter;
using manipulation::Turn;

// 1) 켤 때는 STOP (바퀴 · 팔 다 잠금). 팔 자세를 모르니까 — HOME 작업이 끝나야 DRIVE
TEST(MotionArbiter, StartsLocked)
{
  MotionArbiter a(0.0);
  EXPECT_EQ(a.turn(), Turn::STOP);
  EXPECT_FALSE(a.wheels_may_move());
  EXPECT_FALSE(a.arm_may_move());
  a.request_arm();                          // HOME
  a.update(0.0, true, a.request_id());      // 바퀴가 잠갔다고 답함
  EXPECT_EQ(a.turn(), Turn::ARM);
  a.arm_ended(true);
  EXPECT_EQ(a.turn(), Turn::DRIVE);

  MotionArbiter b(0.5, false);              // 시험용: 바로 DRIVE 로 시작
  EXPECT_TRUE(b.wheels_may_move());
}

// 2) 팔 작업 → STOPPING. "잠갔다" 답 + 0.5 s 정지가 둘 다 있어야 ARM
TEST(MotionArbiter, NeedsLockAckAndSettle)
{
  MotionArbiter a(0.5, false);
  a.request_arm();
  const uint32_t id = a.request_id();
  EXPECT_EQ(a.turn(), Turn::STOPPING);
  EXPECT_FALSE(a.wheels_may_move());

  a.update(10.0, true, id - 1);   // 멈춰 있지만 답이 없음(예전 번호) → 안 셈
  a.update(11.0, true, id - 1);
  EXPECT_EQ(a.turn(), Turn::STOPPING);

  a.update(12.0, true, id);       // 이제 답이 옴 → 여기서부터 0.5 s
  a.update(12.3, true, id);
  EXPECT_EQ(a.turn(), Turn::STOPPING);
  a.update(12.5, true, id);
  EXPECT_EQ(a.turn(), Turn::ARM);
  EXPECT_TRUE(a.arm_may_move());
  EXPECT_FALSE(a.wheels_may_move());
}

// 3) ★ 로봇이 원래 멈춰 있어도, 바퀴 쪽이 잠금을 못 받았으면(답 없음) 팔을 시작하지 않음
TEST(MotionArbiter, StillButNotLockedNeverStartsArm)
{
  MotionArbiter a(0.5, false);
  const uint32_t drive_id = a.request_id();   // 바퀴 쪽이 마지막으로 받은 건 DRIVE
  a.request_arm();
  for (int k = 0; k < 100; ++k) {a.update(k * 0.1, true, drive_id);}
  EXPECT_EQ(a.turn(), Turn::STOPPING);
}

// 4) 멈추는 중에 다시 움직이면 처음부터 다시 셈
TEST(MotionArbiter, MovementResetsSettleTimer)
{
  MotionArbiter a(0.5, false);
  a.request_arm();
  const uint32_t id = a.request_id();
  a.update(0.0, true, id);
  a.update(0.4, false, id);   // 흔들림
  a.update(0.6, true, id);
  a.update(1.0, true, id);    // 0.6 부터 0.4 s
  EXPECT_EQ(a.turn(), Turn::STOPPING);
  a.update(1.1, true, id);
  EXPECT_EQ(a.turn(), Turn::ARM);
}

// 5) 끝나면 바퀴 차례로, 실패하면 STOP → 다음 팔 작업(HOME)으로만 풀림. 차례가 바뀔 때마다 번호가 바뀜
TEST(MotionArbiter, FinishAndFail)
{
  MotionArbiter a(0.0, false);
  a.request_arm();
  a.update(0.0, true, a.request_id());
  const uint32_t arm_id = a.request_id();
  a.arm_ended(true);
  EXPECT_EQ(a.turn(), Turn::DRIVE);
  EXPECT_NE(a.request_id(), arm_id);

  a.request_arm();
  a.update(1.0, true, a.request_id());
  a.arm_ended(false);
  EXPECT_EQ(a.turn(), Turn::STOP);
  a.update(2.0, true, a.request_id());
  EXPECT_EQ(a.turn(), Turn::STOP);   // 저절로 안 풀림
  a.request_arm();
  EXPECT_EQ(a.turn(), Turn::STOPPING);
}

// 6) ★ STOP(켤 때 · 재시작) 에서 받은 작업이 ARM 전에 취소돼도 STOP 그대로
TEST(MotionArbiter, CancelBeforeArmKeepsLock)
{
  MotionArbiter a(0.5);                     // 켤 때 (또는 control_master 재시작 직후)
  a.request_arm();                          // 복구 HOME 받음 → STOPPING
  EXPECT_EQ(a.turn(), Turn::STOPPING);
  a.arm_ended(false);                       // 시작 전에 취소
  EXPECT_EQ(a.turn(), Turn::STOP);
  EXPECT_FALSE(a.wheels_may_move());
}

// ============================================================================ ③ 팔 작업 진행
// 핵심: "HOME 시작 전 취소" 같은 경계에서도 잠금이 풀리지 않는지 (아래 ArmRunnerLock).
//   control_master.cpp 는 ROS 입출력만 하고 판단은 ArmRunner + MotionArbiter 가 하므로,
//   아래 FakeMaster 가 control_master 의 run_arm / step 과 같은 순서로 두 개를 부른다.

using manipulation::ArmCommand;
using manipulation::ArmEnd;
using manipulation::HardwareWatch;
using manipulation::ArmRunner;
using manipulation::FoldCheck;
using manipulation::LateGoal;
using manipulation::MotionArbiter;
using manipulation::Turn;

namespace
{

// control_master 의 판단 부분만 흉내 (설정값은 config/control.yaml 기본값과 같게)
const std::vector<double> kHome{0.0, -1.45, 1.35, 0.9, 0.0};   // control.yaml poses.home

struct FakeMaster
{
  MotionArbiter arbiter{0.5};                // start_locked = true (켤 때 · 재시작)
  ArmRunner runner{2.0, 4.0 + 3.0, 2.0, 2.0};   // accept 2 s, move 4 + 여유 3 s, cancel 2 s, 접힘 확인 2 s
  // 접힘 실측 확인 (control.yaml 기본값과 같게: 허용 0.05 rad, 0.3 s 유지, 측정 0.2 s 안)
  FoldCheck fold{kHome, {0.05}, 0.3, 0.2};
  HardwareWatch hw{0.3, 0.1, 1.0};          // control.yaml 기본값과 같게 (재검증 R6 · 보드 2장)
  bool boards_ok{true};                     // controller_manager 가 알려 주는 보드 상태 (false = 어느 보드가 active 아님)
  bool hw_ok_before{false};
  bool goal_active{false};
  bool cancel_requested{false};
  std::vector<ArmCommand> sent, cancels, finished;

  // ---- 가짜 팔: 궤적이 성공하면 관절이 목표로 감. frozen 관절은 안 움직임, meas_ok=false 면 측정이 NaN
  std::vector<double> joints = std::vector<double>(5, 0.0);   // 켤 때 자세 모름 (0)
  int frozen = -1;
  bool meas_ok = true;
  bool meas_flowing = true;   // false = /joint_states 가 끊김 (측정 시각이 안 바뀜)

  // control_master on_arm_goal 처럼: 하드웨어 이상이면 거절
  bool try_accept(uint8_t task, double t)
  {
    if (!hw.healthy(t)) {return false;}
    accept(task);
    return true;
  }
  void accept(uint8_t task)
  {
    goal_active = true;
    cancel_requested = false;
    runner.start(manipulation::make_arm_plan(task, 0.019, -0.005));
    arbiter.request_arm();
  }
  // still: 몸통이 멈춤. 바퀴 컨트롤러는 STOP/STOPPING 번호를 바로 "잠갔음" 이라고 답한다고 가정
  void step(double t, bool still = true)
  {
    // control_master 와 같은 순서: 측정 → 하드웨어 이상 → 차례 → 팔
    if (meas_flowing) {
      std::vector<double> m = joints;
      if (!meas_ok) {m.assign(5, std::nan(""));}
      fold.observe(t, m);
      hw.observe(t, meas_ok);
    }
    hw.observe_components(t, boards_ok, boards_ok ? "" : "bus_drive=unconfigured");
    hw_ok_before = hw.healthy(t);
    arbiter.enforce_drive_guard(fold.confirmed(t), hw_ok_before);
    if (!hw_ok_before) {
      if (arbiter.turn() != Turn::STOP) {arbiter.force_stop();}
      runner.fail_now("하드웨어 이상");
    }
    const uint32_t ack = arbiter.turn() == Turn::DRIVE ? 0 : arbiter.request_id();
    arbiter.update(t, still, ack);
    if (!goal_active) {return;}
    if (cancel_requested) {runner.request_cancel();}
    for (const ArmCommand & c : runner.tick(t, arbiter.arm_may_move(), fold.confirmed(t))) {
      if (c.kind == ArmCommand::Kind::SEND) {sent.push_back(c);}
      if (c.kind == ArmCommand::Kind::CANCEL_ACTIVE) {cancels.push_back(c);}
      if (c.kind == ArmCommand::Kind::VERIFY_START) {fold.reset();}
      if (c.kind == ArmCommand::Kind::FINISH) {
        finished.push_back(c);
        arbiter.arm_ended(c.drive_ok && hw_ok_before);
        goal_active = false;
      }
    }
  }
  // 지금 보낸 궤적을 재생기가 받아서 끝냄 (팔 동작이면 관절이 목표로 감 — frozen 관절만 빼고)
  void complete_last(double t, bool ok = true)
  {
    ASSERT_FALSE(sent.empty());
    EXPECT_EQ(runner.on_goal_response(sent.back().gen, true, t), LateGoal::USE);
    const manipulation::ArmAction & a = sent.back().action;
    if (ok && !a.is_gripper) {
      const std::vector<double> target = pose(a.pose);
      for (int j = 0; j < 5; ++j) {
        if (j != frozen) {joints[j] = target[j];}
      }
    }
    runner.on_result(sent.back().gen, ok);
  }
  // 작업 하나를 끝까지 돌림 (성공이든 실패든)
  void run_to_end(double & t)
  {
    for (int k = 0; k < 2000 && goal_active; ++k) {
      const std::size_t n = sent.size();
      step(t);
      if (sent.size() > n) {complete_last(t);}
      t += 0.02;
    }
  }
  static std::vector<double> pose(const std::string & name)
  {
    if (name == "home") {return kHome;}
    return {0.0, 0.04, -0.2, 1.7, 0.0};   // 집기 · 놓기 자세 (시험용 아무 값)
  }
};

}  // namespace

// ───────────── 장면 A: 처음 켠 뒤 HOME 을 시작 전에 취소 → STOP 그대로
TEST(ArmRunnerLock, InitialHomeCancelledBeforeStartKeepsLock)
{
  FakeMaster m;
  EXPECT_EQ(m.arbiter.turn(), Turn::STOP);
  m.accept(manipulation::kArmHome);
  m.step(0.0, false);                 // 아직 몸통이 안 멈춤 → STOPPING, 궤적 안 보냄
  EXPECT_EQ(m.arbiter.turn(), Turn::STOPPING);
  EXPECT_TRUE(m.sent.empty());

  m.cancel_requested = true;          // HOME 시작 전 취소
  m.step(0.02, false);
  ASSERT_EQ(m.finished.size(), 1u);
  EXPECT_EQ(m.finished[0].end, ArmEnd::CANCELED);
  EXPECT_FALSE(m.finished[0].drive_ok);
  EXPECT_EQ(m.arbiter.turn(), Turn::STOP);
  EXPECT_FALSE(m.arbiter.wheels_may_move());
  EXPECT_TRUE(m.cancels.empty());     // 보낸 궤적이 없으니 취소할 것도 없음
}

// ───────────── 장면 B: 팔 동작 중 control_master 재시작 → 새 노드는 STOP → 복구 HOME 즉시 취소 → STOP 그대로
TEST(ArmRunnerLock, RestartRecoveryHomeCancelledKeepsLock)
{
  // 재시작 전: HOME 성공 → DRIVE → PICK 진행 중 (팔이 움직이는 중에 노드가 죽음)
  {
    FakeMaster before;
    double t = 0.0;
    before.accept(manipulation::kArmHome);
    before.run_to_end(t);
    ASSERT_EQ(before.arbiter.turn(), Turn::DRIVE);
    before.accept(manipulation::kArmPick);
    for (int k = 0; k < 100; ++k) {before.step(t); t += 0.02;}
    ASSERT_EQ(before.arbiter.turn(), Turn::ARM);
    ASSERT_FALSE(before.sent.empty());   // 팔 궤적을 보냄 → 이 상태에서 노드가 죽었다고 침
  }
  // 재시작한 새 control_master: 이전 기억 없음, STOP 으로 시작
  FakeMaster m;
  EXPECT_EQ(m.arbiter.turn(), Turn::STOP);
  m.accept(manipulation::kArmHome);     // planning 이 복구 HOME 을 보냄
  m.cancel_requested = true;          // 바로 취소
  m.step(0.0);
  ASSERT_EQ(m.finished.size(), 1u);
  EXPECT_EQ(m.finished[0].end, ArmEnd::CANCELED);
  EXPECT_EQ(m.arbiter.turn(), Turn::STOP);
  // 그 뒤 아무리 시간이 지나도 저절로 풀리지 않음
  for (int k = 1; k < 200; ++k) {m.step(k * 0.02);}
  EXPECT_EQ(m.arbiter.turn(), Turn::STOP);
  EXPECT_TRUE(m.sent.empty());
}

// DRIVE 중에 받은 작업도 ARM 전에 취소되면 STOP (단순 규칙: 취소는 언제나 STOP. 다음 HOME 이 풀어 줌)
TEST(ArmRunnerLock, CancelFromDriveAlsoLocks)
{
  FakeMaster m;
  double t = 0.0;
  m.accept(manipulation::kArmHome);
  m.run_to_end(t);
  ASSERT_EQ(m.arbiter.turn(), Turn::DRIVE);
  m.accept(manipulation::kArmPick);
  m.cancel_requested = true;
  m.step(t);
  EXPECT_EQ(m.arbiter.turn(), Turn::STOP);
  // 이어서 HOME 이 성공해야 DRIVE
  m.accept(manipulation::kArmHome);
  m.run_to_end(t);
  EXPECT_EQ(m.arbiter.turn(), Turn::DRIVE);
}

// 정상: HOME 성공 → DRIVE, PICK 전체 성공(마지막 = home) → DRIVE
TEST(ArmRunner, SuccessEndingFoldedReturnsToDrive)
{
  FakeMaster m;
  double t = 0.0;
  m.accept(manipulation::kArmHome);
  m.run_to_end(t);
  ASSERT_EQ(m.finished.size(), 1u);
  EXPECT_EQ(m.finished[0].end, ArmEnd::SUCCEEDED);
  EXPECT_TRUE(m.finished[0].drive_ok);
  EXPECT_EQ(m.arbiter.turn(), Turn::DRIVE);

  m.accept(manipulation::kArmPick);
  m.run_to_end(t);
  ASSERT_EQ(m.finished.size(), 2u);
  EXPECT_TRUE(m.finished[1].drive_ok);
  EXPECT_EQ(m.sent.size(), 1u + 6u);   // HOME 1개 + PICK 6개
  EXPECT_EQ(m.arbiter.turn(), Turn::DRIVE);
}

// 팔이 움직이는 중 취소 → 재생기에 취소 → "멈췄음"(결과) 을 받아야 canceled → STOP
TEST(ArmRunner, CancelDuringMotionWaitsForStop)
{
  FakeMaster m;
  double t = 0.0;
  m.accept(manipulation::kArmHome);
  for (int k = 0; k < 40 && m.sent.empty(); ++k) {m.step(t); t += 0.02;}
  ASSERT_EQ(m.sent.size(), 1u);
  const unsigned g = m.sent[0].gen;
  EXPECT_EQ(m.runner.on_goal_response(g, true, t), LateGoal::USE);

  m.cancel_requested = true;
  m.step(t);
  ASSERT_EQ(m.cancels.size(), 1u);
  EXPECT_EQ(m.cancels[0].gen, g);
  EXPECT_TRUE(m.finished.empty());    // 아직 멈췄다는 답 없음
  m.step(t + 0.5);
  EXPECT_TRUE(m.finished.empty());
  m.runner.on_result(g, false);       // JTC: 취소로 끝남
  m.step(t + 0.6);
  ASSERT_EQ(m.finished.size(), 1u);
  EXPECT_EQ(m.finished[0].end, ArmEnd::CANCELED);
  EXPECT_EQ(m.finished[0].message.find("정지 답 없음"), std::string::npos);
  EXPECT_EQ(m.arbiter.turn(), Turn::STOP);
}

// 취소했는데 재생기가 끝내 답이 없음 → cancel_timeout 뒤 canceled (메시지에 표시), STOP. 늦은 결과는 무시
TEST(ArmRunner, CancelTimeoutStillLocks)
{
  FakeMaster m;
  double t = 0.0;
  m.accept(manipulation::kArmHome);
  for (int k = 0; k < 40 && m.sent.empty(); ++k) {m.step(t); t += 0.02;}
  const unsigned g = m.sent[0].gen;
  m.runner.on_goal_response(g, true, t);
  m.cancel_requested = true;
  m.step(t);
  m.step(t + 1.9);
  EXPECT_TRUE(m.finished.empty());
  m.step(t + 2.1);
  ASSERT_EQ(m.finished.size(), 1u);
  EXPECT_EQ(m.finished[0].end, ArmEnd::CANCELED);
  EXPECT_NE(m.finished[0].message.find("정지 답 없음"), std::string::npos);
  EXPECT_EQ(m.arbiter.turn(), Turn::STOP);

  // 다음 작업이 시작된 뒤 예전 궤적의 결과가 늦게 와도 새 작업을 건드리지 않음
  m.accept(manipulation::kArmHome);
  m.runner.on_result(g, false);
  EXPECT_FALSE(m.runner.busy());
  double t2 = t + 3.0;
  m.run_to_end(t2);
  EXPECT_EQ(m.finished.back().end, ArmEnd::SUCCEEDED);
  EXPECT_EQ(m.arbiter.turn(), Turn::DRIVE);
}

// 받아 주기 전에 취소 → 전체 취소 + 늦게 받아들여지면 그 궤적만 다시 취소(CANCEL_IT)
TEST(ArmRunner, CancelBeforeAcceptAndLateAccept)
{
  FakeMaster m;
  double t = 0.0;
  m.accept(manipulation::kArmHome);
  for (int k = 0; k < 40 && m.sent.empty(); ++k) {m.step(t); t += 0.02;}
  const unsigned g = m.sent[0].gen;
  m.cancel_requested = true;
  m.step(t);
  ASSERT_EQ(m.cancels.size(), 1u);
  ASSERT_EQ(m.finished.size(), 1u);   // 결과가 올 일이 없으니 바로 끝
  EXPECT_EQ(m.arbiter.turn(), Turn::STOP);
  EXPECT_EQ(m.runner.on_goal_response(g, true, t + 0.1), LateGoal::CANCEL_IT);
  EXPECT_EQ(m.runner.on_goal_response(g, false, t + 0.1), LateGoal::IGNORE);
}

// 시간 제한: 받아 주지 않음 / 끝나지 않음 → 취소 요청 + 실패(abort) → STOP
TEST(ArmRunner, TimeoutsAbortAndLock)
{
  {
    FakeMaster m;
    double t = 0.0;
    m.accept(manipulation::kArmHome);
    for (int k = 0; k < 40 && m.sent.empty(); ++k) {m.step(t); t += 0.02;}
    m.step(t + 2.5);                  // 2 s 넘게 받았다는 답 없음
    ASSERT_EQ(m.finished.size(), 1u);
    EXPECT_EQ(m.finished[0].end, ArmEnd::ABORTED);
    EXPECT_EQ(m.cancels.size(), 1u);
    EXPECT_EQ(m.arbiter.turn(), Turn::STOP);
    EXPECT_EQ(m.runner.on_goal_response(m.sent[0].gen, true, t + 3.0), LateGoal::CANCEL_IT);
  }
  {
    FakeMaster m;
    double t = 0.0;
    m.accept(manipulation::kArmHome);
    for (int k = 0; k < 40 && m.sent.empty(); ++k) {m.step(t); t += 0.02;}
    m.runner.on_goal_response(m.sent[0].gen, true, t);
    m.step(t + 6.9);
    EXPECT_TRUE(m.finished.empty());
    m.step(t + 7.1);                  // move 4 + 여유 3 s 초과
    ASSERT_EQ(m.finished.size(), 1u);
    EXPECT_EQ(m.finished[0].end, ArmEnd::ABORTED);
    EXPECT_EQ(m.arbiter.turn(), Turn::STOP);
  }
}

// 재생기 거절 · 동작 실패 → abort → STOP (다음 동작을 보내지 않음)
TEST(ArmRunner, RejectOrFailAbortsAndLocks)
{
  FakeMaster m;
  double t = 0.0;
  m.accept(manipulation::kArmHome);
  m.run_to_end(t);
  m.accept(manipulation::kArmPick);
  for (int k = 0; k < 60 && m.sent.size() < 2; ++k) {m.step(t); t += 0.02;}
  ASSERT_EQ(m.sent.size(), 2u);
  m.runner.on_goal_response(m.sent.back().gen, true, t);
  m.runner.on_result(m.sent.back().gen, false);   // pick_above 실패
  m.step(t);
  EXPECT_EQ(m.finished.back().end, ArmEnd::ABORTED);
  EXPECT_EQ(m.arbiter.turn(), Turn::STOP);
  EXPECT_EQ(m.sent.size(), 2u);

  m.accept(manipulation::kArmHome);
  for (int k = 0; k < 60 && m.sent.size() < 3; ++k) {m.step(t); t += 0.02;}
  m.runner.on_goal_response(m.sent.back().gen, false, t);   // 거절
  m.step(t);
  EXPECT_EQ(m.finished.back().end, ArmEnd::ABORTED);
  EXPECT_EQ(m.arbiter.turn(), Turn::STOP);
}

// 마지막 동작이 접기가 아닌 계획은 성공해도 DRIVE 로 못 감 (안전장치 — 지금 계획들은 모두 home 으로 끝남)
TEST(ArmRunner, SuccessNotFoldedStaysLocked)
{
  ArmRunner r(2.0, 7.0, 2.0);
  r.start({{"approach", false, "pick_above", 0.0}});
  auto c = r.tick(0.0, true);
  ASSERT_EQ(c.size(), 1u);
  r.on_goal_response(c[0].gen, true, 0.0);
  r.on_result(c[0].gen, true);
  c = r.tick(0.1, true);
  ASSERT_EQ(c.size(), 1u);
  EXPECT_EQ(c[0].kind, ArmCommand::Kind::FINISH);
  EXPECT_EQ(c[0].end, ArmEnd::SUCCEEDED);
  EXPECT_FALSE(c[0].drive_ok);
}

// ============================================================================
// 2026-10-16 R1 — "JTC 가 성공이라고 해도 팔이 실제로 안 접혔으면 바퀴를 풀지 않는다"
// (Codex 재현: 가짜 보드에서 어깨(ID 12)를 고정 → joint2 = 0, home = −1.45 인데 HOME 성공 + DRIVE)
// ============================================================================

// 어깨가 고정돼 HOME 에 못 감 → JTC 는 성공을 돌려줘도 실측 확인 실패 → ABORTED, STOP 유지
TEST(ArmFoldCheck, FrozenShoulderHomeFailsAndKeepsLock)
{
  FakeMaster m;
  m.frozen = 1;   // joint2 (어깨)
  double t = 0.0;
  m.accept(manipulation::kArmHome);
  m.run_to_end(t);
  ASSERT_EQ(m.finished.size(), 1u);
  EXPECT_EQ(m.finished[0].end, ArmEnd::ABORTED);
  EXPECT_FALSE(m.finished[0].drive_ok);
  EXPECT_NE(m.finished[0].message.find("HOME 실측 확인 실패"), std::string::npos);
  EXPECT_EQ(m.arbiter.turn(), Turn::STOP);
  EXPECT_FALSE(m.arbiter.wheels_may_move());
}

// 같은 고장으로 PICK (마지막이 home) 도 실패 → STOP
TEST(ArmFoldCheck, FrozenShoulderPickFailsAndKeepsLock)
{
  FakeMaster m;
  double t = 0.0;
  m.accept(manipulation::kArmHome);
  m.run_to_end(t);
  ASSERT_EQ(m.arbiter.turn(), Turn::DRIVE);
  m.accept(manipulation::kArmPick);
  const std::size_t before = m.sent.size();
  for (int k = 0; k < 200 && m.sent.size() == before; ++k) {m.step(t); t += 0.02;}
  m.complete_last(t);             // 첫 동작(pick_above)으로 어깨가 앞으로 나감
  ASSERT_NEAR(m.joints[1], 0.04, 1e-9);
  m.frozen = 1;                   // 그 자리에서 어깨가 걸림 → 마지막 home 에 못 돌아옴
  m.run_to_end(t);
  ASSERT_FALSE(m.finished.empty()) << "작업이 끝나지 않음";
  EXPECT_EQ(m.finished.back().end, ArmEnd::ABORTED);
  EXPECT_EQ(m.arbiter.turn(), Turn::STOP);
}

// 측정이 NaN (모터를 못 읽음 — 하드웨어가 NaN 으로 표시) → 실패, STOP
TEST(ArmFoldCheck, InvalidMeasurementFails)
{
  FakeMaster m;
  m.meas_ok = false;
  double t = 0.0;
  m.accept(manipulation::kArmHome);
  m.run_to_end(t);
  ASSERT_FALSE(m.finished.empty()) << "작업이 끝나지 않음";
  EXPECT_EQ(m.finished.back().end, ArmEnd::ABORTED);
  EXPECT_EQ(m.arbiter.turn(), Turn::STOP);
}

// /joint_states 가 끊김 (측정 시각이 안 바뀜) → 옛날 측정으로는 확인 안 됨 → 실패, STOP
TEST(ArmFoldCheck, StoppedMeasurementFails)
{
  FakeMaster m;
  double t = 0.0;
  m.accept(manipulation::kArmHome);
  for (int k = 0; k < 200 && m.sent.empty(); ++k) {m.step(t); t += 0.02;}
  m.complete_last(t);           // 팔은 실제로 home 에 감
  m.meas_flowing = false;       // 그런데 측정이 끊김
  m.run_to_end(t);
  ASSERT_FALSE(m.finished.empty()) << "작업이 끝나지 않음";
  EXPECT_EQ(m.finished.back().end, ArmEnd::ABORTED);
  EXPECT_EQ(m.arbiter.turn(), Turn::STOP);
}

// 일부만 이동 (어깨가 절반에서 멈춤) → 실패
TEST(ArmFoldCheck, PartialMoveFails)
{
  FakeMaster m;
  double t = 0.0;
  m.accept(manipulation::kArmHome);
  for (int k = 0; k < 200 && m.sent.empty(); ++k) {m.step(t); t += 0.02;}
  m.frozen = 1;
  m.joints[1] = -0.70;          // 절반쯤 가다 멈춤
  m.complete_last(t);
  m.run_to_end(t);
  ASSERT_FALSE(m.finished.empty()) << "작업이 끝나지 않음";
  EXPECT_EQ(m.finished.back().end, ArmEnd::ABORTED);
  EXPECT_EQ(m.arbiter.turn(), Turn::STOP);
}

// 정상: 측정이 home 범위에 들어오고 0.3 s 유지돼야 성공 (그 전에는 DRIVE 아님)
TEST(ArmFoldCheck, NormalHomeNeedsHoldTime)
{
  FakeMaster m;
  double t = 0.0;
  m.accept(manipulation::kArmHome);
  for (int k = 0; k < 200 && m.sent.empty(); ++k) {m.step(t); t += 0.02;}
  m.complete_last(t);
  m.step(t); t += 0.02;          // VERIFY_START
  ASSERT_TRUE(m.runner.verifying());
  for (int k = 0; k < 10; ++k) {m.step(t); t += 0.02;}   // 0.2 s — 아직 유지 시간 부족
  EXPECT_TRUE(m.finished.empty());
  EXPECT_NE(m.arbiter.turn(), Turn::DRIVE);
  m.run_to_end(t);
  ASSERT_EQ(m.finished.size(), 1u);
  EXPECT_EQ(m.finished[0].end, ArmEnd::SUCCEEDED);
  EXPECT_TRUE(m.finished[0].drive_ok);
  EXPECT_EQ(m.arbiter.turn(), Turn::DRIVE);
}

// VERIFY 중 취소 → CANCELED, STOP
TEST(ArmFoldCheck, CancelDuringVerifyKeepsLock)
{
  FakeMaster m;
  m.frozen = 1;
  double t = 0.0;
  m.accept(manipulation::kArmHome);
  for (int k = 0; k < 200 && m.sent.empty(); ++k) {m.step(t); t += 0.02;}
  m.complete_last(t);
  m.step(t); t += 0.02;
  ASSERT_TRUE(m.runner.verifying());
  m.cancel_requested = true;
  m.step(t);
  EXPECT_EQ(m.finished.back().end, ArmEnd::CANCELED);
  EXPECT_EQ(m.arbiter.turn(), Turn::STOP);
}

// FoldCheck 단독: 같은 시각 반복은 무시, 유지 시간, 측정 나이, NaN, 관절 수
TEST(ArmFoldCheck, FoldCheckRules)
{
  FoldCheck f(kHome, {0.05}, 0.3, 0.2);
  EXPECT_FALSE(f.confirmed(0.0));
  f.observe(1.0, kHome);
  for (int k = 0; k < 20; ++k) {f.observe(1.0, kHome);}   // 같은 측정 시각이 계속 와도
  EXPECT_FALSE(f.confirmed(1.0));                         // 유지 시간이 안 쌓임
  for (double t = 1.05; t < 1.30; t += 0.05) {f.observe(t, kHome);}   // 측정이 촘촘히 계속 옴
  f.observe(1.31, kHome);
  EXPECT_TRUE(f.confirmed(1.31));
  EXPECT_FALSE(f.confirmed(1.6));                         // 측정이 0.2 s 넘게 오래됨
  auto off = kHome;
  off[2] += 0.06;                                         // 팔꿈치 0.06 rad 벗어남
  f.observe(1.4, off);
  EXPECT_FALSE(f.confirmed(1.4));
  f.observe(1.5, kHome);
  EXPECT_FALSE(f.confirmed(1.5));                         // 다시 들어와도 처음부터
  for (double t = 1.55; t < 1.8; t += 0.05) {f.observe(t, kHome);}
  f.observe(1.8, kHome);
  EXPECT_TRUE(f.confirmed(1.8));
  auto nan = kHome;
  nan[0] = std::nan("");
  f.observe(1.9, nan);
  EXPECT_FALSE(f.confirmed(1.9));
  f.observe(2.0, {0.0, -1.45});                           // 관절 수가 모자람
  EXPECT_FALSE(f.confirmed(2.3));
  f.reset();
  EXPECT_FALSE(f.confirmed(2.3));
}

// (Codex 10/16 재검증) 측정이 끊겼다가 다시 오면 끊긴 시간을 유지 시간으로 세지 않음
//   예전: 1.0 s 에 범위 안 → 측정 끊김 → 1.5 s 에 다시 범위 안 한 번 = "0.5 s 유지" 로 바로 확인됨
TEST(ArmFoldCheck, GapDoesNotCountAsHold)
{
  FoldCheck f(kHome, {0.05}, 0.3, 0.2);
  f.observe(1.00, kHome);
  f.observe(1.01, kHome);
  f.observe(1.50, kHome);          // 0.49 s 끊겼다가 다시 옴
  EXPECT_FALSE(f.confirmed(1.50));
  for (double t = 1.51; t < 1.79; t += 0.01) {f.observe(t, kHome);}
  EXPECT_FALSE(f.confirmed(1.79));  // 다시 온 뒤로 아직 0.3 s 안 됨
  f.observe(1.81, kHome);
  EXPECT_TRUE(f.confirmed(1.81));
}

// 같은 장면을 작업 전체로: 확인 중 측정이 0.4 s 끊겼다가 돌아와도 바로 DRIVE 가 되지 않음
TEST(ArmFoldCheck, MeasurementGapDuringVerifyRestartsHold)
{
  FakeMaster m;
  double t = 0.0;
  m.accept(manipulation::kArmHome);
  for (int k = 0; k < 200 && m.sent.empty(); ++k) {m.step(t); t += 0.02;}
  m.complete_last(t);
  m.step(t); t += 0.02;            // VERIFY_START
  m.step(t); t += 0.02;            // 범위 안 측정 1번
  // 0.26 s 끊김: 접힘 확인 기준(0.2 s)보다 길고 하드웨어 이상 기준(0.3 s)보다 짧음
  //   (0.3 s 넘게 끊기면 이제 하드웨어 이상으로 작업 자체가 실패 — HardwareWatch.StateLossDuringMotion…)
  m.meas_flowing = false;
  for (int k = 0; k < 13; ++k) {m.step(t); t += 0.02;}
  m.meas_flowing = true;
  m.step(t);                       // 다시 온 첫 측정
  EXPECT_TRUE(m.finished.empty());
  EXPECT_NE(m.arbiter.turn(), Turn::DRIVE);
  m.run_to_end(t);
  EXPECT_EQ(m.finished.back().end, ArmEnd::SUCCEEDED);   // 다시 0.3 s 머문 뒤에는 정상 성공
}

// ───────────── 2026-10-16 재검증 R6: 하드웨어 이상 → HOME 확인 무효, 복구 뒤 HOME 이 다시 성공해야 DRIVE
namespace
{
// HOME 을 끝까지 돌려 DRIVE 로 만든 FakeMaster
void home_to_drive(FakeMaster & m, double & t)
{
  for (int k = 0; k < 10; ++k) {m.step(t); t += 0.02;}   // 측정이 들어오기 시작
  ASSERT_TRUE(m.try_accept(manipulation::kArmHome, t));
  m.run_to_end(t);
  ASSERT_EQ(m.arbiter.turn(), Turn::DRIVE);
}
}  // namespace

TEST(HardwareWatch, Rules)
{
  HardwareWatch w(0.3, 0.1);
  EXPECT_FALSE(w.healthy(0.0)) << "측정 전에는 이상";
  w.observe(1.0, true);
  EXPECT_TRUE(w.healthy(1.1));
  EXPECT_FALSE(w.healthy(1.31)) << "0.3 s 넘게 안 오면 이상";
  w.observe(1.0, true);   // 같은 시각 반복은 살아 있다는 뜻이 아님
  EXPECT_FALSE(w.healthy(1.31));
  w.observe(1.4, false);
  EXPECT_TRUE(w.healthy(1.4)) << "NaN 한 번은 아직 (0.1 s 유예)";
  w.observe(1.45, false);
  EXPECT_TRUE(w.healthy(1.45));
  w.observe(1.51, false);
  EXPECT_FALSE(w.healthy(1.51)) << "NaN 이 0.1 s 넘게";
  w.observe(1.52, true);
  EXPECT_TRUE(w.healthy(1.52)) << "측정이 돌아오면 정상 (잠금 해제는 HOME 이 따로 함)";
}

TEST(HardwareWatch, ArbiterForceStopOnlyHomeReturnsToDrive)
{
  MotionArbiter a(0.5, false);
  ASSERT_EQ(a.turn(), Turn::DRIVE);
  const uint32_t id = a.request_id();
  a.force_stop();
  EXPECT_EQ(a.turn(), Turn::STOP);
  EXPECT_NE(a.request_id(), id) << "새 번호 — 예전 DRIVE 번호로는 안 풀림";
  a.update(10.0, true, a.request_id());
  EXPECT_EQ(a.turn(), Turn::STOP) << "STOP 은 시간이 지나도 그대로";
  a.arm_ended(false);
  EXPECT_EQ(a.turn(), Turn::STOP);
  a.arm_ended(true);
  EXPECT_EQ(a.turn(), Turn::DRIVE);
}

TEST(HardwareWatch, FaultInDriveLocksAndRecoveryNeedsHome)
{
  FakeMaster m;
  double t = 0.0;
  home_to_drive(m, t);
  m.meas_ok = false;   // 하드웨어 ERROR → 모든 칸 NaN
  for (int k = 0; k < 10; ++k) {m.step(t); t += 0.02;}
  EXPECT_EQ(m.arbiter.turn(), Turn::STOP) << "NaN 0.1 s 넘게 → STOP";
  EXPECT_FALSE(m.try_accept(manipulation::kArmHome, t)) << "이상 중 새 팔 작업은 거절";
  m.meas_ok = true;    // 사람이 재설정 → 측정 돌아옴
  for (int k = 0; k < 100; ++k) {m.step(t); t += 0.02;}
  EXPECT_EQ(m.arbiter.turn(), Turn::STOP) << "측정이 돌아왔다고 DRIVE 로 가면 안 됨";
  ASSERT_TRUE(m.try_accept(manipulation::kArmHome, t));
  m.run_to_end(t);
  EXPECT_EQ(m.arbiter.turn(), Turn::DRIVE) << "HOME + 실측 확인 뒤에만 DRIVE";
}

TEST(HardwareWatch, StateLossDuringMotionAbortsAndLateSuccessIgnored)
{
  FakeMaster m;
  double t = 0.0;
  home_to_drive(m, t);
  ASSERT_TRUE(m.try_accept(manipulation::kArmPick, t));
  const std::size_t before = m.sent.size();
  for (int k = 0; k < 200 && m.sent.size() == before; ++k) {m.step(t); t += 0.02;}
  ASSERT_GT(m.sent.size(), before);
  const unsigned gen = m.sent.back().gen;
  EXPECT_EQ(m.runner.on_goal_response(gen, true, t), LateGoal::USE);
  m.meas_flowing = false;   // /joint_states 끊김
  for (int k = 0; k < 25 && m.goal_active; ++k) {m.step(t); t += 0.02;}
  ASSERT_FALSE(m.goal_active) << "끊긴 지 0.3 s 넘으면 작업 실패";
  EXPECT_EQ(m.finished.back().end, ArmEnd::ABORTED);
  EXPECT_FALSE(m.finished.back().drive_ok);
  ASSERT_FALSE(m.cancels.empty());
  EXPECT_EQ(m.cancels.back().gen, gen) << "진행 중 궤적에 취소 요청";
  m.runner.on_result(gen, true);   // 옛 궤적의 늦은 성공
  m.meas_flowing = true;
  for (int k = 0; k < 50; ++k) {m.step(t); t += 0.02;}
  EXPECT_EQ(m.arbiter.turn(), Turn::STOP) << "늦은 성공으로 잠금이 풀리면 안 됨";
  EXPECT_FALSE(m.runner.active());
}

TEST(HardwareWatch, FaultDuringVerifyFails)
{
  FakeMaster m;
  double t = 0.0;
  for (int k = 0; k < 10; ++k) {m.step(t); t += 0.02;}
  ASSERT_TRUE(m.try_accept(manipulation::kArmHome, t));
  // home 궤적까지 보내고 끝냄 → VERIFY 들어가는 순간 측정이 NaN
  for (int k = 0; k < 2000 && m.goal_active && !m.runner.verifying(); ++k) {
    const std::size_t n = m.sent.size();
    m.step(t);
    if (m.sent.size() > n) {m.complete_last(t);}
    t += 0.02;
  }
  ASSERT_TRUE(m.runner.verifying());
  m.meas_ok = false;
  for (int k = 0; k < 50 && m.goal_active; ++k) {m.step(t); t += 0.02;}
  ASSERT_FALSE(m.goal_active);
  EXPECT_EQ(m.finished.back().end, ArmEnd::ABORTED);
  EXPECT_EQ(m.arbiter.turn(), Turn::STOP);
}

// ───────────── 2026-10-16 보드 2장: 보드 상태를 직접 봄 — "주행 보드만 죽고 팔 측정은 멀쩡" 도 STOP
TEST(HardwareWatch, ComponentRules)
{
  HardwareWatch w(0.3, 0.1, 1.0);
  w.observe(1.0, true);
  EXPECT_FALSE(w.healthy(1.0)) << "보드 상태를 아직 못 받았으면 이상";
  w.observe_components(1.0, true, "");
  EXPECT_TRUE(w.healthy(1.05));
  w.observe_components(1.1, false, "bus_drive=unconfigured");
  w.observe(1.1, true);
  EXPECT_FALSE(w.healthy(1.1)) << "관절 측정이 멀쩡해도 보드 하나가 active 가 아니면 이상";
  EXPECT_NE(w.status(1.1).find("bus_drive=unconfigured"), std::string::npos) << w.status(1.1);
  w.observe_components(1.2, true, "");
  w.observe(1.2, true);
  EXPECT_TRUE(w.healthy(1.2));
  w.observe(2.1, true);
  EXPECT_FALSE(w.healthy(2.25)) << "보드 상태 답이 1 s 넘게 없으면 모름 = 이상";
  HardwareWatch off(0.3, 0.1);   // 보드 상태를 안 보는 설정
  off.observe(1.0, true);
  EXPECT_TRUE(off.healthy(1.0));
}

TEST(HardwareWatch, DriveBoardDownWhileArmFineLocksAndNeedsHome)
{
  FakeMaster m;
  double t = 0.0;
  home_to_drive(m, t);
  m.boards_ok = false;   // 주행 보드만 ERROR → unconfigured. 팔 관절 측정은 그대로 정상
  m.step(t); t += 0.02;
  EXPECT_EQ(m.arbiter.turn(), Turn::STOP) << "다음 주기에 바로 STOP";
  EXPECT_FALSE(m.try_accept(manipulation::kArmHome, t)) << "고장 중 팔 작업 거절";
  m.boards_ok = true;    // 주행 보드만 재설정
  for (int k = 0; k < 50; ++k) {m.step(t); t += 0.02;}
  EXPECT_EQ(m.arbiter.turn(), Turn::STOP) << "보드가 돌아와도 HOME 전에는 STOP";
  ASSERT_TRUE(m.try_accept(manipulation::kArmHome, t));
  m.run_to_end(t);
  EXPECT_EQ(m.arbiter.turn(), Turn::DRIVE);
}

TEST(HardwareWatch, ArmTaskAbortedWhenOtherBoardFails)
{
  FakeMaster m;
  double t = 0.0;
  home_to_drive(m, t);
  ASSERT_TRUE(m.try_accept(manipulation::kArmPick, t));
  const std::size_t before = m.sent.size();
  for (int k = 0; k < 200 && m.sent.size() == before; ++k) {m.step(t); t += 0.02;}
  ASSERT_GT(m.sent.size(), before);
  EXPECT_EQ(m.runner.on_goal_response(m.sent.back().gen, true, t), LateGoal::USE);
  m.boards_ok = false;   // 팔이 움직이는 중 주행 보드 고장 (팔 측정은 정상)
  m.step(t); t += 0.02;
  ASSERT_FALSE(m.goal_active) << "팔 작업도 바로 실패";
  EXPECT_EQ(m.finished.back().end, ArmEnd::ABORTED);
  EXPECT_FALSE(m.cancels.empty()) << "진행 중 궤적 취소";
  EXPECT_EQ(m.arbiter.turn(), Turn::STOP);
}

TEST(DriveFoldGuard, DeviationLatchesUntilNewHome)
{
  FakeMaster m; double t = 0.; home_to_drive(m, t);
  m.joints[2] += .06;
  m.step(t); t += .02;
  EXPECT_EQ(m.arbiter.turn(), Turn::STOP);
  m.joints = kHome;
  for (int k=0;k<50;++k) {m.step(t);t+=.02;}
  EXPECT_EQ(m.arbiter.turn(), Turn::STOP);
  ASSERT_TRUE(m.try_accept(manipulation::kArmHome,t));m.run_to_end(t);
  EXPECT_EQ(m.arbiter.turn(), Turn::DRIVE);
}

TEST(DriveFoldGuard, StaleFoldStopsBeforeHardwareTimeout)
{
  FakeMaster m; double t = 0.; home_to_drive(m, t);
  m.meas_flowing = false;
  // Fold age limit .2s is stricter than HardwareWatch .3s.
  t += .21; m.step(t);
  ASSERT_TRUE(m.hw.healthy(t));
  EXPECT_EQ(m.arbiter.turn(), Turn::STOP);
}

TEST(DriveFoldGuard, InvalidArmMeasurementStopsImmediately)
{
  FakeMaster m; double t = 0.; home_to_drive(m,t);
  m.meas_ok=false;m.step(t);
  EXPECT_EQ(m.arbiter.turn(),Turn::STOP);
}

TEST(DriveFoldGuard, FutureAndNonfiniteTimestampsCannotPermitDrive)
{
  FoldCheck f(kHome,{.05},.3,.2);
  for(int k=0;k<=4;++k) {f.observe(10.+k*.1,kHome);}
  ASSERT_TRUE(f.confirmed(10.4));
  EXPECT_FALSE(f.confirmed(1.));
  HardwareWatch h(.3,.1);h.observe(10.,true);
  EXPECT_FALSE(h.healthy(1.));
  h.observe(std::nan(""),true);EXPECT_FALSE(h.healthy(10.));
  EXPECT_FALSE(f.confirmed(std::nan("")));
}
