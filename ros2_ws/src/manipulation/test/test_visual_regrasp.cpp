// test_visual_regrasp.cpp — 다시 보고 쥐기 (regrasp_visual): 관측 고르기 · 대기 끼우기 · 계획 교체 · 수직 IK · 원래 경로로 돌아가기
#include <gtest/gtest.h>
#include <chrono>
#include <limits>
#include "manipulation/visual_regrasp.hpp"
#include "arm_kin_data.hpp"
using namespace manipulation;

TEST(VisualRegrasp, UsesOnlyNewExactUprightSameId)
{
  const Eigen::Vector3d expected(0.214, -0.050, 0.1);
  RegraspObservation good{expected + Eigen::Vector3d(0.002, 0, 0), 10.2, "4", true, true};
  auto bad = good; bad.stamp = 9.9;
  EXPECT_FALSE(select_regrasp_observation({bad}, expected, "4", 10, 10.3, .5, .06));
  bad = good; bad.id = "5";
  EXPECT_FALSE(select_regrasp_observation({bad}, expected, "4", 10, 10.3, .5, .06));
  bad = good; bad.exact_tf = false;
  EXPECT_FALSE(select_regrasp_observation({bad}, expected, "4", 10, 10.3, .5, .06));
  bad = good; bad.upright = false;
  EXPECT_FALSE(select_regrasp_observation({bad}, expected, "4", 10, 10.3, .5, .06));
  auto p = select_regrasp_observation({bad, good}, expected, "4", 10, 10.3, .5, .06);
  ASSERT_TRUE(p); EXPECT_NEAR(p->center.x(), .216, 1e-12);
}

TEST(VisualRegrasp, RejectsFutureStaleNanAndRemoteObservation)
{
  const Eigen::Vector3d expected(.214, -.05, .1);
  RegraspObservation p{expected, 10.2, "", true, true};
  EXPECT_FALSE(select_regrasp_observation({p}, expected, "", 10, 10.1, .5, .06));
  EXPECT_FALSE(select_regrasp_observation({p}, expected, "", 10, 11, .5, .06));
  p.center.x() += .026;
  EXPECT_FALSE(select_regrasp_observation({p}, expected, "", 10, 10.3, .5, .06));
  p.center.x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(select_regrasp_observation({p}, expected, "", 10, 10.3, .5, .06));
}

// 요청 ID 가 있어도 ID 를 못 읽은 검출은 위치 (25 mm) 로 씀 (기존 PICK 과 같음), ID 맞는 검출이 있으면 그것. 둘 이상 = 모호 → 안 씀
TEST(VisualRegrasp, UntaggedByPositionAndAmbiguousRejected)
{
  const Eigen::Vector3d expected(.214, -.03, .067);
  RegraspObservation untagged{expected + Eigen::Vector3d(.003, 0, 0), 10.25, "", true, true};
  RegraspObservation tagged{expected + Eigen::Vector3d(-.002, .001, 0), 10.25, "7", true, true};
  auto p = select_regrasp_observation({untagged}, expected, "7", 10.2, 10.3, .5, .06);
  ASSERT_TRUE(p); EXPECT_NEAR(p->center.x(), expected.x() + .003, 1e-12);
  p = select_regrasp_observation({untagged, tagged}, expected, "7", 10.2, 10.3, .5, .06);
  ASSERT_TRUE(p); EXPECT_EQ(p->id, "7");
  std::string why;
  auto twin = untagged; twin.center.y() += .004;
  EXPECT_FALSE(select_regrasp_observation({untagged, twin}, expected, "", 10.2, 10.3, .5, .06, &why));
  EXPECT_NE(why.find("모호"), std::string::npos) << why;
  auto tagged2 = tagged; tagged2.center.x() += .01;
  EXPECT_FALSE(select_regrasp_observation({tagged, tagged2}, expected, "7", 10.2, 10.3, .5, .06, &why));
  EXPECT_NE(why.find("모호"), std::string::npos) << why;
  // 높이가 40 mm 넘게 다른 검출 (박스 밖 · 다른 면) 은 안 씀
  auto low = untagged; low.center.z() -= .045;
  EXPECT_FALSE(select_regrasp_observation({low}, expected, "", 10.2, 10.3, .5, .06));
}

TEST(VisualRegrasp, WaitAtObservePointPreservesFallbackGrips)
{
  auto node = [](const char * name, double q, const char * grip, bool stop = false) {
      return PathNode{name, std::vector<double>(5, q), false, 15, grip, stop};
    };
  ArmPath path{"pick", {node("start", 0, ""), node("re_place", .1, "release"), node("re_place_clear", 0, ""),
    node(kRegraspObserveNode, -.05, "", true), node("re_above", .02, "open"),
    node("re_grasp", .1, "close"), node("re_lift", .02, "")}, false};
  auto original = path_actions(path, PathTiming{}, GripperValues{}, std::vector<double>(5, 0));
  auto disabled = split_regrasp_observation(path, original, false, 2);
  ASSERT_EQ(disabled.plan.size(), original.size()); EXPECT_TRUE(disabled.tail.nodes.empty());
  for (size_t i = 0; i < original.size(); ++i) {
    EXPECT_EQ(disabled.plan[i].pose, original[i].pose);
    EXPECT_EQ(disabled.plan[i].points.size(), original[i].points.size());
    EXPECT_EQ(disabled.plan[i].gripper, original[i].gripper);
  }
  auto enabled = split_regrasp_observation(path, original, true, 2);
  ASSERT_EQ(enabled.plan.size(), original.size() + 1);
  ASSERT_EQ(enabled.tail.nodes.front().name, kRegraspObserveNode);
  size_t k = 0;
  for (const auto & a : enabled.plan) {
    if (a.pose == kRegraspObserve) {ASSERT_GT(k, 0u); EXPECT_EQ(original[k - 1].pose, kRegraspObserveNode); continue;}
    EXPECT_EQ(a.pose, original[k].pose); EXPECT_EQ(a.gripper, original[k].gripper); ++k;
  }
  // 멈춤이 없는 관찰 점에는 안 끼움 · 관찰 점이 없으면 (끄고 만든 경로, re_place_clear 만) 안 끼움
  path.nodes[3].stop = false;
  auto plain = path_actions(path, PathTiming{}, GripperValues{}, std::vector<double>(5, 0));
  auto none = split_regrasp_observation(path, plain, true, 2);
  EXPECT_EQ(none.plan.size(), plain.size());
  EXPECT_TRUE(none.tail.nodes.empty());
  path.nodes.erase(path.nodes.begin() + 3);
  path.nodes[2].stop = true;
  auto off = path_actions(path, PathTiming{}, GripperValues{}, std::vector<double>(5, 0));
  EXPECT_EQ(split_regrasp_observation(path, off, true, 2).plan.size(), off.size());
}

namespace
{
ArmAction observation_wait()
{
  ArmAction wait{"reobserve", false, kRegraspObserve, 0.0};
  wait.duration_s = 2.0;
  return wait;
}

ArmAction movement(const char * pose)
{
  return {"approach", false, pose, 0.0};
}
}  // namespace

TEST(VisualRegrasp, ReplacementContinuesAtNewTailWithoutReplayingPrefix)
{
  ArmRunner runner(1, 10, 1);
  runner.start({movement("re_place_clear"), observation_wait(), movement("nominal_grasp")});
  auto commands = runner.tick(0, true);
  ASSERT_EQ(commands.size(), 1u);
  EXPECT_EQ(commands[0].action.pose, "re_place_clear");
  runner.on_goal_response(commands[0].gen, true, 0);
  runner.on_result(commands[0].gen, true);
  commands = runner.tick(.1, true);
  ASSERT_EQ(commands.size(), 1u);
  ASSERT_EQ(commands[0].action.pose, kRegraspObserve);
  const auto wait_gen = commands[0].gen;
  runner.on_goal_response(wait_gen, true, .1);

  ASSERT_TRUE(runner.replace_after_wait(wait_gen, {movement("measured_grasp"), movement("staging")}));
  EXPECT_EQ(runner.next_index(), 2u);
  EXPECT_GT(runner.gen(), wait_gen);
  // 끝낸 논리 대기의 늦은 답은 새 동작을 끝내거나 실패시키지 못함
  EXPECT_EQ(runner.on_goal_response(wait_gen, true, .2), LateGoal::CANCEL_IT);
  runner.on_result(wait_gen, false);
  commands = runner.tick(.2, true);
  ASSERT_EQ(commands.size(), 1u);
  EXPECT_EQ(commands[0].kind, ArmCommand::Kind::SEND);
  EXPECT_EQ(commands[0].action.pose, "measured_grasp");
  const auto movement_gen = commands[0].gen;
  runner.on_goal_response(movement_gen, true, .2);
  runner.on_result(wait_gen, true);
  EXPECT_TRUE(runner.tick(.3, true).empty());
  runner.on_result(movement_gen, true);
  commands = runner.tick(.4, true);
  ASSERT_EQ(commands.size(), 1u);
  EXPECT_EQ(commands[0].action.pose, "staging");
  runner.on_goal_response(commands[0].gen, true, .4);
  runner.on_result(commands[0].gen, true);
  commands = runner.tick(.5, true);
  ASSERT_EQ(commands.size(), 1u);
  EXPECT_EQ(commands[0].kind, ArmCommand::Kind::FINISH);
  EXPECT_EQ(commands[0].end, ArmEnd::SUCCEEDED);
  EXPECT_FALSE(commands[0].drive_ok);  // PICK 은 HOME 확인으로 안 끝남 → 바퀴 잠금 그대로
}

// 못 봄 (시간 초과) · 모호 · 한계 넘음 · IK 실패 = control_master 가 대기만 끝냄 (on_result ok) → 원래 후반 그대로, 같은 성공
TEST(VisualRegrasp, FallbackFinishesWaitAndRunsOriginalTail)
{
  ArmRunner runner(1, 10, 1);
  runner.start({movement("re_place_clear"), observation_wait(), movement("re_above"),
    ArmAction{"grasp", true, "", 0.0}, movement("re_lift")});
  auto c = runner.tick(0, true);
  runner.on_goal_response(c[0].gen, true, 0);
  runner.on_result(c[0].gen, true);
  c = runner.tick(.1, true);
  ASSERT_EQ(c.size(), 1u);
  ASSERT_EQ(c[0].action.pose, kRegraspObserve);
  const auto wait_gen = c[0].gen;
  runner.on_goal_response(wait_gen, true, .1);
  EXPECT_TRUE(runner.tick(1.0, true).empty());   // 기다리는 동안은 아무것도 안 보냄 (시간 제한 10 + 2 s 안)
  runner.on_result(wait_gen, true);              // fallback
  std::vector<std::string> sent;
  double t = 2.2;
  for (int i = 0; i < 10; ++i, t += .1) {
    c = runner.tick(t, true);
    if (c.empty()) {continue;}
    if (c[0].kind == ArmCommand::Kind::FINISH) {
      EXPECT_EQ(c[0].end, ArmEnd::SUCCEEDED);
      break;
    }
    ASSERT_EQ(c[0].kind, ArmCommand::Kind::SEND);
    sent.push_back(c[0].action.is_gripper ? "[grip]" : c[0].action.pose);
    runner.on_goal_response(c[0].gen, true, t);
    runner.on_result(c[0].gen, true);
  }
  EXPECT_EQ(sent, (std::vector<std::string>{"re_above", "[grip]", "re_lift"}));
}

TEST(VisualRegrasp, CancellationAndFaultCannotBeClearedByReplacement)
{
  for (bool fault : {false, true}) {
    SCOPED_TRACE(fault ? "hardware fault" : "cancel request");
    ArmRunner runner(1, 10, 1);
    runner.start({observation_wait(), movement("nominal_grasp")});
    const auto commands = runner.tick(0, true);
    ASSERT_EQ(commands.size(), 1u);
    const auto gen = commands[0].gen;
    runner.on_goal_response(gen, true, 0);
    if (fault) {runner.fail_now("hardware lost");} else {runner.request_cancel();}
    EXPECT_FALSE(runner.replace_after_wait(gen, {movement("measured_grasp")}));
    // control_master 는 대기를 끝낸 뒤 tick 에 넘김
    runner.on_result(gen, true);
    const auto end = runner.tick(.1, true);
    ASSERT_EQ(end.size(), 1u);
    EXPECT_EQ(end[0].kind, ArmCommand::Kind::FINISH);
    EXPECT_EQ(end[0].end, fault ? ArmEnd::ABORTED : ArmEnd::CANCELED);
    EXPECT_FALSE(end[0].drive_ok);
    EXPECT_FALSE(runner.replace_after_wait(gen, {movement("late_worker_result")}));
    EXPECT_TRUE(runner.tick(.2, true).empty());
  }
}

TEST(VisualRegrasp, CancellationAndFaultAfterReplacementPreemptFirstMotorCommand)
{
  for (bool fault : {false, true}) {
    ArmRunner runner(1, 10, 1);
    runner.start({observation_wait(), movement("nominal_grasp")});
    const auto commands = runner.tick(0, true);
    ASSERT_EQ(commands.size(), 1u);
    const auto gen = commands[0].gen;
    runner.on_goal_response(gen, true, 0);
    ASSERT_TRUE(runner.replace_after_wait(gen, {movement("measured_grasp")}));
    if (fault) {runner.fail_now("hardware lost");} else {runner.request_cancel();}
    const auto end = runner.tick(.1, true);
    ASSERT_EQ(end.size(), 1u);
    EXPECT_EQ(end[0].kind, ArmCommand::Kind::FINISH);
    EXPECT_EQ(end[0].end, fault ? ArmEnd::ABORTED : ArmEnd::CANCELED);
    EXPECT_FALSE(end[0].drive_ok);
  }
}

TEST(VisualRegrasp, OldGenerationCannotReplaceAnotherTask)
{
  ArmRunner runner(1, 10, 1);
  runner.start({observation_wait(), movement("old_nominal")});
  auto commands = runner.tick(0, true);
  ASSERT_EQ(commands.size(), 1u);
  const auto old_gen = commands[0].gen;
  runner.on_goal_response(old_gen, true, 0);
  runner.request_cancel();
  runner.on_result(old_gen, true);
  commands = runner.tick(.1, true);
  ASSERT_EQ(commands.size(), 1u);
  ASSERT_EQ(commands[0].end, ArmEnd::CANCELED);

  runner.start({observation_wait(), movement("new_nominal")});
  commands = runner.tick(.2, true);
  ASSERT_EQ(commands.size(), 1u);
  const auto new_gen = commands[0].gen;
  runner.on_goal_response(new_gen, true, .2);
  ASSERT_NE(new_gen, old_gen);
  EXPECT_FALSE(runner.replace_after_wait(old_gen, {movement("old_result")}));
  runner.on_result(old_gen, true);
  EXPECT_TRUE(runner.busy());
  EXPECT_TRUE(runner.tick(.3, true).empty());
  runner.on_result(new_gen, true);  // 새 대기는 자기 원래 후반으로
  commands = runner.tick(.4, true);
  ASSERT_EQ(commands.size(), 1u);
  EXPECT_EQ(commands[0].action.pose, "new_nominal");
}

TEST(VisualRegrasp, ReplacementPreservesStepBoundaryAndQueuedStepPermission)
{
  for (bool step_already_requested : {false, true}) {
    SCOPED_TRACE(step_already_requested);
    ArmRunner runner(1, 10, 1);
    runner.start({observation_wait(), movement("nominal_grasp")});
    runner.set_step(true);
    auto commands = runner.tick(0, true);
    ASSERT_EQ(commands.size(), 1u);
    const auto gen = commands[0].gen;
    runner.on_goal_response(gen, true, 0);
    if (step_already_requested) {runner.step_go();}
    ASSERT_TRUE(runner.replace_after_wait(gen, {movement("measured_grasp"), movement("staging")}));
    if (!step_already_requested) {
      EXPECT_TRUE(runner.tick(.1, true).empty());
      EXPECT_TRUE(runner.waiting_step());
      EXPECT_EQ(runner.next_name(), "measured_grasp");
      runner.step_go();
    }
    commands = runner.tick(.2, true);
    ASSERT_EQ(commands.size(), 1u);
    EXPECT_EQ(commands[0].kind, ArmCommand::Kind::SEND);
    EXPECT_EQ(commands[0].action.pose, "measured_grasp");
    runner.on_goal_response(commands[0].gen, true, .2);
    runner.on_result(commands[0].gen, true);
    EXPECT_TRUE(runner.tick(.3, true).empty());
    EXPECT_TRUE(runner.waiting_step());
    EXPECT_EQ(runner.next_name(), "staging");  // 허가 하나로 동작 둘을 보내지 않음
  }
}

TEST(VisualRegrasp, InvalidReplacementKeepsFallbackAndFailedWaitCannotBeRevived)
{
  ArmRunner runner(1, 10, 1);
  runner.start({observation_wait(), movement("nominal_grasp")});
  auto commands = runner.tick(0, true);
  ASSERT_EQ(commands.size(), 1u);
  const auto gen = commands[0].gen;
  EXPECT_FALSE(runner.replace_after_wait(gen, {movement("before_acceptance")}));
  runner.on_goal_response(gen, true, 0);
  EXPECT_FALSE(runner.replace_after_wait(gen, {}));
  EXPECT_TRUE(runner.busy());
  runner.on_result(gen, true);
  commands = runner.tick(.1, true);
  ASSERT_EQ(commands.size(), 1u);
  EXPECT_EQ(commands[0].action.pose, "nominal_grasp");

  ArmRunner failed(1, 10, 1);
  failed.start({observation_wait(), movement("nominal_grasp")});
  commands = failed.tick(0, true);
  ASSERT_EQ(commands.size(), 1u);
  const auto failed_gen = commands[0].gen;
  failed.on_goal_response(failed_gen, true, 0);
  failed.on_result(failed_gen, false);
  EXPECT_FALSE(failed.replace_after_wait(failed_gen, {movement("late_result")}));
  commands = failed.tick(.1, true);
  ASSERT_EQ(commands.size(), 1u);
  EXPECT_EQ(commands[0].kind, ArmCommand::Kind::FINISH);
  EXPECT_EQ(commands[0].end, ArmEnd::ABORTED);
  EXPECT_FALSE(commands[0].drive_ok);
}

TEST(VisualRegrasp, MeasuredXYShiftKeepsOriginalHeightIncludingTopBias)
{
  const auto model = kin_data::model();
  const Eigen::Vector2d original(.2102, .038), measured(.2122, .038);
  const std::vector<double> heights{40, 10, -30};
  const std::vector<std::pair<double, std::string>> candidates{{std::nan(""), "open"}};
  for (double top : {.058, .068}) {  // 원래 높이 (pick_top_bias +10 mm 포함일 수 있음) 그대로
    SCOPED_TRACE(top);
    const auto nominal = solve_visual_regrasp(model, original, top, -4, heights, candidates, {});
    ASSERT_EQ(nominal.poses.size(), 3u) << nominal.why;
    const std::vector<Q5> seeds{ticks_to_q(model, nominal.poses.front().ticks)};
    const auto shifted = solve_visual_regrasp(model, measured, top, -4, heights, candidates, seeds);
    ASSERT_EQ(shifted.poses.size(), 3u) << shifted.why;
    EXPECT_EQ(shifted.grip, "open");
    for (size_t k = 0; k < shifted.poses.size(); ++k) {
      const auto before = arm_grip(model, ticks_to_q(model, nominal.poses[k].ticks));
      const auto after = arm_grip(model, ticks_to_q(model, shifted.poses[k].ticks));
      EXPECT_DOUBLE_EQ(shifted.poses[k].tilt_deg, 0.0);
      EXPECT_NEAR(after.app.x(), 0.0, .005);
      EXPECT_NEAR(after.app.y(), 0.0, .005);
      EXPECT_NEAR(after.app.z(), -1.0, .005);
      EXPECT_NEAR(after.tip.x(), measured.x(), .0015);
      EXPECT_NEAR(after.tip.y(), measured.y(), .0015);
      EXPECT_NEAR(after.tip.z(), top + heights[k] / 1000.0, .0015);
      EXPECT_NEAR(after.tip.z(), before.tip.z(), .0015);
      EXPECT_NEAR(after.tip.x() - before.tip.x(), .002, .0015);
    }
  }
}

TEST(VisualRegrasp, UnreachableVerticalTargetReturnsNoTiltedReplacement)
{
  const auto solution = solve_visual_regrasp(kin_data::model(), {.30, 0}, .058, -4,
    {40, 10, -30}, {{std::nan(""), "open"}}, {});
  EXPECT_TRUE(solution.poses.empty());
  EXPECT_TRUE(solution.grip.empty());
  EXPECT_FALSE(solution.why.empty());
}

namespace
{
// 실물 다시 쥐기 자리처럼: 박스 앞 (x 0.214) · 닫힘 = 접선 · 높이 = 박스 퍽 윗면 + pick_top_bias 10 mm · pick_heights_mm (Jetson)
struct RealSpot
{
  ArmModel m = kin_data::model();
  double x, y, top = 0.068, c_deg;
  std::vector<double> heights{40.0, 10.0, -50.0};
  std::vector<PickPose> blind;
  ArmPath tail;
  RealSpot(double x_, double y_) : x(x_), y(y_)
  {
    c_deg = (std::atan2(y - m.j1_xy.y(), x - m.j1_xy.x()) + M_PI / 2.0) * 180.0 / M_PI;
    std::string why;
    blind = pick_poses(m, x, y, top, c_deg, heights, why, c_deg, 40.0, 0.0, 0.5);   // control_master regrasp_nodes 와 같은 풀이
    auto node = [&](const char * name, const Ticks & enc, const char * grip = "", bool stop = false) {
        return PathNode{name, ticks_to_command(m, enc), false, 30.0, grip, stop};
      };
    if (blind.size() == 3) {
      tail = {"regrasp_visual", {node(kRegraspObserveNode, blind[0].enc, "", true), node("re_above", blind[0].enc, "open", true),
        node("re_down", blind[1].enc), node("re_grasp", blind[2].enc, "close"), node("re_up", blind[1].enc),
        node("re_lift", blind[0].enc), PathNode{"staging", std::vector<double>(5, 0.1), false, 0.0, "", false}}, false};
      tail.nodes[1].hold_s = 0.3;
    }
  }
  RegraspTarget target(double after) const
  {
    RegraspTarget t;
    t.expected = Eigen::Vector3d(x, y, 0.077);
    t.after = after;
    t.top = top;
    t.close_deg = c_deg;
    t.heights = heights;
    t.seed = ticks_to_q(m, blind[0].ticks);
    return t;
  }
};
}  // namespace

// 실물 자리에서 놓은 퍽이 1 – 5 mm (가운데 자리는 15 mm 까지, 팔 쪽 반원) 어긋났을 때: 원래 해를 씨앗으로 수직 해가 바로 나오고 (몇 ms),
//   손끝이 잰 중심 · 원래 높이 · 원래 닫힘 방향. 바뀌는 점은 다시 쥐기 다섯 점뿐 (관찰 점 · staging · 멈춤 · 그리퍼 그대로).
//   바깥쪽 수직 여유는 자리마다 6 (y 0.07) – 19 mm (y 0) → 그보다 밖이면 IK 안 됨 = 원래 경로 (DecisionFallsBackForEveryFailure)
TEST(VisualRegrasp, RealSpotShiftSolvesFastAndOnlyRetargetsRegrip)
{
  for (double y : {-0.06, -0.028, 0.0, 0.013, 0.07}) {
    RealSpot s(0.214, y);
    ASSERT_EQ(s.blind.size(), 3u) << y;
    const Eigen::Vector2d out = (Eigen::Vector2d(s.x, s.y) - s.m.j1_xy).normalized();
    for (double ang = 0.0; ang < 360.0; ang += 45.0) {
      for (double r : {0.001, 0.003, 0.005, 0.015}) {
        const Eigen::Vector2d u(std::cos(ang * M_PI / 180.0), std::sin(ang * M_PI / 180.0));
        if (r > 0.006 && u.dot(out) > -0.1) {continue;}   // 15 mm 는 팔 쪽 (안쪽) 만
        SCOPED_TRACE(std::to_string(y) + " " + std::to_string(ang) + " " + std::to_string(r));
        const Eigen::Vector2d xy(s.x + r * std::cos(ang * M_PI / 180.0), s.y + r * std::sin(ang * M_PI / 180.0));
        RegraspObservation o{Eigen::Vector3d(xy.x(), xy.y(), 0.067), 10.3, "3", true, true};
        auto tg = s.target(10.2);
        tg.id = "3";
        const auto t0 = std::chrono::steady_clock::now();
        const auto d = decide_visual_regrasp({o}, tg, 10.35, 12.0, s.m, s.tail);
        const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        ASSERT_EQ(d.decision, RegraspDecision::USE) << d.why;
        EXPECT_LT(ms, 50.0);
        ASSERT_EQ(d.tail.nodes.size(), s.tail.nodes.size());
        EXPECT_EQ(d.tail.nodes.front().q, s.tail.nodes.front().q);
        EXPECT_EQ(d.tail.nodes.back().q, s.tail.nodes.back().q);
        for (size_t i = 0; i < d.tail.nodes.size(); ++i) {
          EXPECT_EQ(d.tail.nodes[i].name, s.tail.nodes[i].name);
          EXPECT_EQ(d.tail.nodes[i].grip, s.tail.nodes[i].grip);
          EXPECT_EQ(d.tail.nodes[i].stop, s.tail.nodes[i].stop);
          EXPECT_EQ(d.tail.nodes[i].hold_s, s.tail.nodes[i].hold_s);
          EXPECT_EQ(d.tail.nodes[i].speed_dps, s.tail.nodes[i].speed_dps);
        }
        for (size_t k = 0; k < 3; ++k) {
          const auto g = arm_grip(s.m, ticks_to_q(s.m, d.poses[k].ticks));
          const auto g0 = arm_grip(s.m, ticks_to_q(s.m, s.blind[k].ticks));
          EXPECT_NEAR(g.tip.x(), xy.x(), .0015);
          EXPECT_NEAR(g.tip.y(), xy.y(), .0015);
          EXPECT_NEAR(g.tip.z(), g0.tip.z(), .0015);
          EXPECT_NEAR(g.app.z(), -1.0, .005);
          EXPECT_GT(std::fabs(g.close.dot(g0.close)), std::cos(3.0 * M_PI / 180.0));   // 닫힘 방향 그대로 (손목을 안 돌림)
          EXPECT_LE(std::abs(d.poses[k].ticks[4] - s.blind[k].ticks[4]), 80);   // J1 이 돈 만큼만 (15 mm ≈ 4°), 90° 돌려 쥐기 (1024 틱) 아님
        }
      }
    }
  }
}

// 11-11 실물: 놓은 퍽이 (0.221, -0.063) → (0.232, -0.067) 에 섬 → 수직 손 위 점 (윗면 + 40) 이 안 닿음 (손목을 돌려도 같음)
//   → 위 점만 윗면 + 20 mm 로 낮춰 수직 다시 쥐기 (쥐는 높이 · 닫힘 방향 그대로)
TEST(VisualRegrasp, LowerAboveWhenShiftedOutOfVerticalReach)
{
  RealSpot s(0.221, -0.063);
  ASSERT_EQ(s.blind.size(), 3u);
  const Eigen::Vector2d xy(0.232, -0.067);
  RegraspObservation o{Eigen::Vector3d(xy.x(), xy.y(), 0.067), 10.3, "", true, true};
  auto tg = s.target(10.2);
  tg.max_shift_m = 0.02;
  for (bool sq : {true, false}) {
    tg.square = sq;
    const auto d = decide_visual_regrasp({o}, tg, 10.35, 12.0, s.m, s.tail);
    ASSERT_EQ(d.decision, RegraspDecision::USE) << d.why;
    EXPECT_TRUE(d.low_above);
    const auto ga = arm_grip(s.m, ticks_to_q(s.m, d.poses[0].ticks));
    const auto gg = arm_grip(s.m, ticks_to_q(s.m, d.poses[2].ticks));
    EXPECT_NEAR(ga.tip.z(), s.top + 0.020, 0.0015);                     // 위 점 = 윗면 + 20 mm
    EXPECT_NEAR(gg.tip.z(), s.top + s.heights[2] / 1000.0, 0.0015);     // 쥐는 높이 그대로
    EXPECT_NEAR(gg.tip.x(), xy.x(), 0.0015);
    EXPECT_NEAR(gg.tip.y(), xy.y(), 0.0015);
    EXPECT_NEAR(gg.app.z(), -1.0, 0.005);
  }
  // 원래 자리 근처 (수직 여유 안) 는 위 점 그대로
  RegraspObservation n{Eigen::Vector3d(0.222, -0.063, 0.067), 10.3, "", true, true};
  const auto d0 = decide_visual_regrasp({n}, tg, 10.35, 12.0, s.m, s.tail);
  ASSERT_EQ(d0.decision, RegraspDecision::USE) << d0.why;
  EXPECT_FALSE(d0.low_above);
}

// fallback 이유마다 원래 경로 (tail 안 바꿈): 새 장 없음 (기한 전 WAIT → 기한 뒤 FALLBACK), 모호, 한계 넘음, 수직 IK 안 됨, 다시 쥐기 점 없음
TEST(VisualRegrasp, DecisionFallsBackForEveryFailure)
{
  RealSpot s(0.214, -0.028);
  ASSERT_EQ(s.blind.size(), 3u);
  const auto tg = s.target(10.2);
  RegraspObservation good{Eigen::Vector3d(s.x + .002, s.y, .067), 10.3, "", true, true};

  // 새 장 없음: 대기 시작 전 장만 → 기한 전 WAIT, 기한 뒤 FALLBACK
  auto old = good; old.stamp = 10.1;
  EXPECT_EQ(decide_visual_regrasp({old}, tg, 10.4, 12.0, s.m, s.tail).decision, RegraspDecision::WAIT);
  EXPECT_EQ(decide_visual_regrasp({}, tg, 11.0, 12.0, s.m, s.tail).decision, RegraspDecision::WAIT);
  auto d = decide_visual_regrasp({old}, tg, 12.0, 12.0, s.m, s.tail);
  EXPECT_EQ(d.decision, RegraspDecision::FALLBACK);
  EXPECT_NE(d.why.find("새 관측 없음"), std::string::npos) << d.why;
  // 오래된 장 (0.5 s 넘음), 촬영 시각 TF 없음 (최신 TF 로 옮긴 검출)
  EXPECT_EQ(decide_visual_regrasp({good}, tg, 10.9, 12.0, s.m, s.tail).decision, RegraspDecision::WAIT);
  auto latest_tf = good; latest_tf.exact_tf = false;
  EXPECT_EQ(decide_visual_regrasp({latest_tf}, tg, 10.35, 12.0, s.m, s.tail).decision, RegraspDecision::WAIT);
  d = decide_visual_regrasp({latest_tf}, tg, 10.35, 10.35, s.m, s.tail);
  EXPECT_EQ(d.decision, RegraspDecision::FALLBACK);
  EXPECT_NE(d.why.find("새 장에 놓은 퍽 없음"), std::string::npos) << d.why;
  // 모호: 기한 전에는 다음 장을 기다리고, 기한이면 원래 경로
  auto twin = good; twin.center.y() += .005;
  d = decide_visual_regrasp({good, twin}, tg, 10.35, 12.0, s.m, s.tail);
  EXPECT_EQ(d.decision, RegraspDecision::WAIT);
  d = decide_visual_regrasp({good, twin}, tg, 10.35, 10.35, s.m, s.tail);
  EXPECT_EQ(d.decision, RegraspDecision::FALLBACK);
  EXPECT_NE(d.why.find("모호"), std::string::npos) << d.why;
  // 한계 넘음 (요청 ID 로 60 mm 안이면 고르지만 15 mm 넘으면 안 씀) → 바로 FALLBACK (기한 전이라도)
  auto far = good; far.center.x() = s.x - .018; far.id = "5";
  auto tg5 = tg; tg5.id = "5";
  d = decide_visual_regrasp({far}, tg5, 10.35, 12.0, s.m, s.tail);
  EXPECT_EQ(d.decision, RegraspDecision::FALLBACK);
  EXPECT_NE(d.why.find("한계"), std::string::npos) << d.why;
  auto open = tg5; open.max_shift_m = std::nan("");   // NaN = 한계 없음
  EXPECT_EQ(decide_visual_regrasp({far}, open, 10.35, 12.0, s.m, s.tail).decision, RegraspDecision::USE);
  // 수직 IK 안 됨 (팔에서 너무 먼 중심, 한계 없음으로)
  auto unreachable = good; unreachable.center.x() = 0.30; unreachable.id = "5";
  open.expected.x() = 0.29;
  d = decide_visual_regrasp({unreachable}, open, 10.35, 12.0, s.m, s.tail);
  EXPECT_EQ(d.decision, RegraspDecision::FALLBACK);
  EXPECT_NE(d.why.find("IK"), std::string::npos) << d.why;
  // 후반 경로에 다시 쥐기 점이 없음
  ArmPath broken = s.tail;
  broken.nodes.erase(broken.nodes.begin() + 3);   // re_grasp
  d = decide_visual_regrasp({good}, tg, 10.35, 12.0, s.m, broken);
  EXPECT_EQ(d.decision, RegraspDecision::FALLBACK);
  EXPECT_TRUE(d.tail.nodes.empty());
  // 정상
  d = decide_visual_regrasp({good}, tg, 10.35, 12.0, s.m, s.tail);
  EXPECT_EQ(d.decision, RegraspDecision::USE) << d.why;
}

namespace
{
// control_master regrasp_nodes 와 같은 관찰 점: 손끝이 박스 위 선 퍽 윗면 (모델 z 0.058) + observe_mm, 손 기울임 20° 부터, 놓기 해를 씨앗으로
std::vector<PickPose> observe_pose(const ArmModel & m, double x, double y, double top, double c_deg, const Q5 & seed, double observe_mm = 110.0)
{
  const double true_top = 0.037 + 0.060 + m.team_base_z_floor - m.base_z_floor;
  const double h = (true_top - top) * 1000.0 + observe_mm;
  std::string why;
  auto P = pick_poses(m, x, y, top, c_deg, {h}, why, c_deg, 40.0, 20.0, 90.0, {seed});
  if (P.size() != 1) {P = pick_poses(m, x, y, top, c_deg, {h}, why, c_deg, 40.0, 20.0, 90.0);}
  return P;
}
}  // namespace

// 다시 쥐기 자리 전부 (x 0.212 – 0.232, |y| ≤ 0.075) · 윗면 보정 있음 / 없음: 관찰 점이 풀리고, 손끝이
//   퍽 축 위 · 실제 윗면 + 110 mm · 손 20° · 닫힘 방향 그대로 (URDF 카메라로 잰 값: 광축 깊이 0.207 m, 화면 안, 손가락 안 가림)
//   놓는 손 10 – 40° (40° 는 30 mm 로 쥔 퍽)
TEST(VisualRegrasp, ObservePointReachableOverRegraspSpots)
{
  const auto m = kin_data::model();
  int n = 0;
  for (double bias : {0.0, 0.010}) {
    const double top = 0.058 + bias;
    for (double x : {0.212, 0.219, 0.232}) {   // 계산 (scratch geom3/4) 은 x 5 · y 8 곳 — 시험은 시간 때문에 줄임
      for (double y : {-0.075, -0.028, 0.0, 0.04, 0.075}) {
        for (double tilt : {10.0, 20.0, 30.0, 40.0}) {   // 40° 는 30 mm 로 쥔 퍽 (11-10 실물 (0.219, -0.014) 40°)
          const double c = (std::atan2(y - m.j1_xy.y(), x - m.j1_xy.x()) + M_PI / 2.0) * 180.0 / M_PI;
          std::string why;
          const auto Pp = pick_poses(m, x, y, top, c, {40.0, 10.0, 8.0 - (tilt > 35.0 ? 30.0 : 50.0)}, why, c, 40.0, tilt, 0.5);
          if (Pp.size() != 3) {continue;}   // 이 자리 · 기울임은 놓기부터 안 됨 (regrasp_nodes 가 다른 자리를 고름)
          SCOPED_TRACE(std::to_string(x) + " " + std::to_string(y) + " " + std::to_string(tilt) + " " + std::to_string(bias));
          const auto Po = observe_pose(m, x, y, top, c, ticks_to_q(m, Pp[0].ticks));
          ASSERT_EQ(Po.size(), 1u);
          ++n;
          EXPECT_DOUBLE_EQ(Po[0].tilt_deg, 20.0);
          const auto g = arm_grip(m, ticks_to_q(m, Po[0].ticks));
          const auto g0 = arm_grip(m, ticks_to_q(m, Pp[0].ticks));
          EXPECT_NEAR(g.tip.x(), x, .0015);
          EXPECT_NEAR(g.tip.y(), y, .0015);
          EXPECT_NEAR(g.tip.z(), 0.058 + 0.110, .0015);
          EXPECT_NEAR(std::acos(-g.app.z()) * 180.0 / M_PI, 20.0, 1.0);
          EXPECT_GT(std::fabs(g.close.normalized().dot(g0.close.normalized())), std::cos(15.0 * M_PI / 180.0));   // 같은 손가락 방향 (손목 안 돌림)
        }
      }
    }
  }
  EXPECT_GT(n, 60);
}

// 관찰 점 (regrasp_visual 켬) 의 동작 시간: 손 뺀 점에서 곧게 올라가 멈춤 → 다시 쥐기 위로 (보는 시간 제외, Jetson 시간표)
TEST(VisualRegrasp, ObservePointMotionTime)
{
  const PathTiming jetson{150.0, 20.0, 375.0, 0.05, 0.3};   // config/control.yaml (Jetson 11-10): path_speed · accel · dt · grip_settle
  for (double y : {-0.07, -0.028, 0.0, 0.04, 0.07}) {
    for (double tilt : {10.0, 20.0, 30.0}) {
      RealSpot s(0.214, y);
      ASSERT_EQ(s.blind.size(), 3u);
      std::string why;
      const double c = s.c_deg;
      const auto Pp = pick_poses(s.m, s.x, s.y, s.top, c, {40.0, 10.0, 8.0 - 50.0}, why, c, 40.0, tilt, 0.5);
      if (Pp.size() != 3) {continue;}
      const auto Po = observe_pose(s.m, s.x, s.y, s.top, c, ticks_to_q(s.m, Pp[0].ticks));
      ASSERT_EQ(Po.size(), 1u);
      auto node = [&](const char * name, const Ticks & enc, const char * grip = "", double speed = 30.0) {
          return PathNode{name, ticks_to_command(s.m, enc), false, speed, grip, false};
        };
      ArmPath p{"pick", {node("far_lift", Pp[0].enc), node("re_place_above", Pp[0].enc), node("re_place_down", Pp[1].enc),
        node("re_place", Pp[2].enc, "release"), node("re_place_up", Pp[1].enc), node("re_place_clear", Pp[0].enc),
        node("re_above", s.blind[0].enc, "open"), node("re_down", s.blind[1].enc), node("re_grasp", s.blind[2].enc, "close"),
        node("re_up", s.blind[1].enc), node("re_lift", s.blind[0].enc)}, false};
      p.nodes[6].stop = true; p.nodes[6].hold_s = 0.3;   // pick_settle_s
      auto total = [](const std::vector<ArmAction> & plan) {
          double t = 0.0;
          for (const auto & a : plan) {t += a.is_gripper ? 0.7 : a.duration_s;}
          return t;
        };
      const auto q0 = p.nodes.front().q;
      const double t_blind = total(path_actions(p, jetson, GripperValues{}, q0));
      auto ob = node(kRegraspObserveNode, Po[0].enc, "", 0.0);
      ob.stop = true;
      p.nodes.insert(p.nodes.begin() + 6, ob);
      const double t_obs = total(path_actions(p, jetson, GripperValues{}, q0));
      // 관절 직선 보간 동안 손끝 높이: 손 뺀 점 → 관찰 점은 손 뺀 점보다 안 내려가고, 관찰 점 → re_above 는 re_above 보다 안 내려감
      auto min_tip = [&](const Ticks & a, const Ticks & b) {
          const Q5 qa = ticks_to_q(s.m, a), qb = ticks_to_q(s.m, b);
          double z = 1e9;
          for (int k = 0; k <= 50; ++k) {
            Q5 q;
            for (int j = 0; j < 5; ++j) {q[j] = qa[j] + (qb[j] - qa[j]) * k / 50.0;}
            z = std::min(z, arm_grip(s.m, q).tip.z());
          }
          return z;
        };
      const double z_clear = arm_grip(s.m, ticks_to_q(s.m, Pp[0].ticks)).tip.z(), z_above = arm_grip(s.m, ticks_to_q(s.m, s.blind[0].ticks)).tip.z();
      EXPECT_GT(min_tip(Pp[0].ticks, Po[0].ticks), z_clear - 0.002);
      EXPECT_GT(min_tip(Po[0].ticks, s.blind[0].ticks), z_above - 0.002);
      std::printf("y %+.3f 놓는 손 %2.0f°: 동작 %.2f s → %.2f s (%+.2f s, 보는 시간 · 동작 경계 제외) · 손끝 최저 (윗면 위) 원래 %.0f mm / 관찰 점 거쳐 %.0f mm\n",
        y, tilt, t_blind, t_obs, t_obs - t_blind, (min_tip(Pp[0].ticks, s.blind[0].ticks) - 0.058) * 1000.0,
        (std::min(min_tip(Pp[0].ticks, Po[0].ticks), min_tip(Po[0].ticks, s.blind[0].ticks)) - 0.058) * 1000.0);
      EXPECT_LT(std::fabs(t_obs - t_blind), 1.5);
    }
  }
}
