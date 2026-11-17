// test_visual_regrasp.cpp — 다시 보고 쥐기 (regrasp_visual): 관측 고르기 · 대기 끼우기 · 계획 교체 · 수직 IK · 원래 경로로 돌아가기
#include <gtest/gtest.h>
#include <chrono>
#include <limits>
#include "manipulation/visual_regrasp.hpp"
#include "manipulation/rack_tags.hpp"
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

// 다시 쥘 자리 (regrasp_spot_gap_mm 30): 이웃 몸통이 놓은 자리에서 30 mm 넘게 (중심 64.8 mm 넘게) 떨어짐 → 다시 보기는 방금 놓은 퍽만 잡음
//   (ID 없음: 위치 25 mm 안, ID 있음: 그 ID). 사이가 모자란 대체 자리도 grasp_for 가 25 mm 안 검출을 막아 이웃은 25 mm 밖 → 같음
TEST(VisualRegrasp, LocksOntoPutDownPuckNotNeighbor)
{
  const Eigen::Vector3d expected(.2222, .0389, .067);
  RegraspObservation put{expected + Eigen::Vector3d(.004, -.003, 0), 10.3, "", true, true};
  for (double d : {0.0648, 0.040, 0.026}) {   // 사이 30 mm · 대체 자리 (5.2 · −8.8 mm)
    RegraspObservation nb{expected + Eigen::Vector3d(-.006, -d, 0), 10.3, "", true, true};
    SCOPED_TRACE(d);
    auto p = select_regrasp_observation({nb, put}, expected, "", 10.2, 10.35, .5, .06);
    ASSERT_TRUE(p);
    EXPECT_EQ(p->center, put.center);
    auto put3 = put; put3.id = "3";
    auto nb5 = nb; nb5.id = "5";
    p = select_regrasp_observation({nb5, put3}, expected, "3", 10.2, 10.35, .5, .06);
    ASSERT_TRUE(p);
    EXPECT_EQ(p->id, "3");
    p = select_regrasp_observation({nb5, put}, expected, "3", 10.2, 10.35, .5, .06);   // 놓은 퍽 태그를 못 읽어도 위치로, 다른 ID 이웃은 안 씀
    ASSERT_TRUE(p);
    EXPECT_EQ(p->center, put.center);
  }
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

// 실제 로봇 사례: 놓은 퍽이 (0.221, -0.063) → (0.232, -0.067) 에 섬 → 수직 손 위 점 (윗면 + 40) 이 안 닿음 (손목을 돌려도 같음)
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
        for (double tilt : {10.0, 20.0, 30.0, 40.0}) {   // 40° 는 30 mm 로 쥔 퍽 (실제 로봇 (0.219, -0.014) 40°)
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
  const PathTiming jetson{150.0, 20.0, 375.0, 0.05, 0.3};   // 시험용 속도 값: path_speed · accel · dt · grip_settle
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

// ── regrasp_align_yaw: 다시 쥘 때 닫힘 방향을 잰 네모 면에 맞춤 ──
//   내려놓다 몇 도 돈 네모를 처음 닫힘 그대로 쥐면 손 안에서 대각선 (실제 로봇: −30°) → 그렇게 다시 쥔 퍽만 칸 (S1 · S2) 에 걸림

// 네모 90° 대칭 접기: 원래 닫힘에서 가장 가까운 면 법선까지 (−45 … 45)
TEST(VisualRegraspAlign, TurnFoldsSquareSymmetry)
{
  EXPECT_NEAR(regrasp_align_turn_deg(89.0, 59.0), -30.0, 1e-9);
  EXPECT_NEAR(regrasp_align_turn_deg(89.0, 149.0), -30.0, 1e-9);    // 면 법선 4 개 중 아무것이나 같음
  EXPECT_NEAR(regrasp_align_turn_deg(89.0, -121.0), -30.0, 1e-9);
  EXPECT_NEAR(regrasp_align_turn_deg(89.0, 99.0), 10.0, 1e-9);
  EXPECT_NEAR(regrasp_align_turn_deg(89.0, 89.0 + 44.0), 44.0, 1e-9);
  EXPECT_NEAR(regrasp_align_turn_deg(89.0, 89.0 + 46.0), -44.0, 1e-9);
  EXPECT_NEAR(regrasp_align_turn_deg(-80.0, 100.0), 0.0, 1e-9);     // 반대쪽 법선 = 그대로
  EXPECT_TRUE(std::isnan(regrasp_align_turn_deg(89.0, std::nan(""))));
  EXPECT_TRUE(std::isnan(regrasp_align_turn_deg(std::nan(""), 10.0)));
}

// 손목 돌림 보정 식 = PICK pick_offset_rot_mm 정의: 90° (실물 −90°) 돌려 쥐면 보정 = [앞, 왼쪽] 을 팔 방향으로 돌린 값 + 로봇 기준 값, 돌림 0 이면 0
TEST(VisualRegraspAlign, WristRotOffsetMatchesPickDefinition)
{
  const auto m = kin_data::model();
  const Eigen::Vector2d r(0.4, -7.6), rr(-2.0, -7.0);   // control.yaml pick_offset_rot_mm · pick_offset_rot_robot_mm 값
  for (double y : {-0.04, 0.0, 0.03}) {
    const double x = 0.214, az = std::atan2(y - m.j1_xy.y(), x - m.j1_xy.x()), tang = az * 180.0 / M_PI + 90.0;
    std::string why;
    const auto P0 = pick_poses(m, x, y, 0.068, tang, {40.0, 10.0, -50.0}, why, tang, 0.0, 0.0, 0.5);
    ASSERT_EQ(P0.size(), 3u) << why;
    double a0 = 99.0;
    const auto d0 = wrist_rot_offset(m, x, y, ticks_to_q(m, P0.back().ticks), r, rr, &a0);
    EXPECT_NEAR(a0, 0.0, 1.0);
    EXPECT_LT(d0.norm(), 0.0002);
    bool seen = false;
    for (double c : {tang - 90.0, tang + 90.0}) {
      const auto P = pick_poses(m, x, y, 0.068, c, {40.0, 10.0, -50.0}, why, c, 0.0, 0.0, 0.5);
      if (P.size() != 3) {continue;}
      double a = 0.0;
      const auto d = wrist_rot_offset(m, x, y, ticks_to_q(m, P.back().ticks), r, rr, &a);
      if (std::fabs(a + 90.0) > 1.0) {continue;}
      seen = true;
      Eigen::Matrix2d R;
      R << std::cos(az), -std::sin(az), std::sin(az), std::cos(az);
      const Eigen::Vector2d want = (R * r + rr) / 1000.0;
      EXPECT_NEAR(d.x(), want.x(), 0.0003);
      EXPECT_NEAR(d.y(), want.y(), 0.0003);
    }
    EXPECT_TRUE(seen) << y;
  }
}

namespace
{
double close_deg_of(const ArmModel & m, const PickPose & p)
{
  const auto c = arm_grip(m, ticks_to_q(m, p.ticks)).close;
  return std::atan2(c.y(), c.x()) * 180.0 / M_PI;
}
}  // namespace

// 내려놓다 θ 돈 네모 (면 방향 앎): 닫힘이 가장 가까운 면 법선 (원래 + θ) 으로, 손끝은 잰 중심 · 원래 높이, 수직 손 · 바뀌는 점은 다시 쥐기 다섯 점뿐
TEST(VisualRegraspAlign, ClosesAlongMeasuredSquareFace)
{
  int n = 0;
  for (double y : {-0.06, -0.028, 0.0, 0.03, 0.07}) {
    RealSpot s(0.214, y);
    ASSERT_EQ(s.blind.size(), 3u);
    const double c0 = close_deg_of(s.m, s.blind.back());
    for (double th : {-44.0, -30.0, -10.0, -3.0, 0.0, 5.0, 10.0, 20.0, 30.0, 44.0}) {
      for (int k : {0, 1, -1, 2}) {   // 면 방향 값은 90° 마다 같은 퍽
        SCOPED_TRACE(std::to_string(y) + " θ " + std::to_string(th) + " k " + std::to_string(k));
        const Eigen::Vector2d xy(s.x + 0.002, s.y - 0.001);
        RegraspObservation o{Eigen::Vector3d(xy.x(), xy.y(), 0.067), 10.3, "", true, true};
        o.yaw_known = true;
        o.yaw_rad = (s.c_deg + th + 90.0 * k) * M_PI / 180.0;
        auto tg = s.target(10.2);
        tg.align_yaw = true;
        const auto d = decide_visual_regrasp({o}, tg, 10.35, 12.0, s.m, s.tail);
        ASSERT_EQ(d.decision, RegraspDecision::USE) << d.why;
        EXPECT_TRUE(d.aligned) << d.align_note;
        EXPECT_NEAR(d.turn_deg, th, 1e-6);
        EXPECT_NEAR(std::remainder(d.close_deg - (s.c_deg + th), 90.0), 0.0, 1e-6);
        ASSERT_EQ(d.tail.nodes.size(), s.tail.nodes.size());
        EXPECT_EQ(d.tail.nodes.front().q, s.tail.nodes.front().q);
        EXPECT_EQ(d.tail.nodes.back().q, s.tail.nodes.back().q);
        for (size_t k2 = 0; k2 < 3; ++k2) {
          const auto g = arm_grip(s.m, ticks_to_q(s.m, d.poses[k2].ticks));
          const auto g0 = arm_grip(s.m, ticks_to_q(s.m, s.blind[k2].ticks));
          EXPECT_NEAR(g.tip.x(), xy.x(), .0015);
          EXPECT_NEAR(g.tip.y(), xy.y(), .0015);
          EXPECT_NEAR(g.tip.z(), g0.tip.z(), .0015);
          EXPECT_NEAR(g.app.z(), -1.0, .005);
          // 실제 닫힘축이 잰 면 법선과 (90° 주기) 2° 안
          const double cd = std::atan2(g.close.y(), g.close.x());
          EXPECT_LT(std::fabs(std::remainder(cd - o.yaw_rad, M_PI / 2.0)) * 180.0 / M_PI, 2.0);
        }
        // 손목은 θ 만큼 (가장 가까운 면 — 90° 돌려 쥐지 않음, 손가락 방향 그대로)
        const double turned = std::remainder(close_deg_of(s.m, d.poses.back()) - c0, 360.0);
        EXPECT_NEAR(turned, th, 2.5);
        ++n;
      }
    }
  }
  EXPECT_EQ(n, 5 * 10 * 4);
}

// 끄면 · 면 방향 모름 (원기둥 · 못 잼) · 한계 넘음 = 처음 닫힘 그대로 (닫힘 · 해 · 보정 안 바뀜)
TEST(VisualRegraspAlign, OffUnknownOrOverLimitKeepsOriginalClose)
{
  RealSpot s(0.214, -0.028);
  ASSERT_EQ(s.blind.size(), 3u);
  RegraspObservation o{Eigen::Vector3d(s.x + .002, s.y, .067), 10.3, "", true, true};
  o.yaw_known = true;
  o.yaw_rad = (s.c_deg - 30.0) * M_PI / 180.0;
  auto tg = s.target(10.2);
  const auto base = decide_visual_regrasp({o}, tg, 10.35, 12.0, s.m, s.tail);   // align_yaw 기본 false
  ASSERT_EQ(base.decision, RegraspDecision::USE) << base.why;
  EXPECT_FALSE(base.aligned);
  EXPECT_NEAR(base.close_deg, s.c_deg, 1e-9);
  EXPECT_LT(base.rot_d.norm(), 1e-9);
  tg.rot_mm = Eigen::Vector2d(0.4, -7.6);
  tg.rot_robot_mm = Eigen::Vector2d(-2.0, -7.0);
  auto same = [&](const RegraspPlan & d) {
      ASSERT_EQ(d.decision, RegraspDecision::USE) << d.why;
      EXPECT_FALSE(d.aligned);
      EXPECT_NEAR(d.close_deg, s.c_deg, 1e-9);
      ASSERT_EQ(d.poses.size(), 3u);
      for (size_t k = 0; k < 3; ++k) {EXPECT_EQ(d.poses[k].ticks, base.poses[k].ticks);}
      EXPECT_EQ(d.tail.nodes.size(), base.tail.nodes.size());
      for (size_t i = 0; i < d.tail.nodes.size(); ++i) {EXPECT_EQ(d.tail.nodes[i].q, base.tail.nodes[i].q);}
    };
  same(decide_visual_regrasp({o}, tg, 10.35, 12.0, s.m, s.tail));   // 끔 (보정 값이 있어도 안 씀)
  tg.align_yaw = true;
  auto unknown = o;
  unknown.yaw_known = false;
  auto d = decide_visual_regrasp({unknown}, tg, 10.35, 12.0, s.m, s.tail);
  same(d);
  EXPECT_NE(d.align_note.find("면 방향 없음"), std::string::npos) << d.align_note;
  EXPECT_LT(d.rot_d.norm(), 1e-9);   // 원래 자세 그대로 = 돌림 보정 0
  tg.align_max_deg = 20.0;
  d = decide_visual_regrasp({o}, tg, 10.35, 12.0, s.m, s.tail);
  same(d);
  EXPECT_NE(d.align_note.find("한계"), std::string::npos) << d.align_note;
  EXPECT_NEAR(d.turn_deg, -30.0, 1e-6);
}

// 켜고 손목 θ 돌리면 PICK 과 같은 돌림 보정 (원래 다시 쥐기 자세와의 차이) — 손끝이 잰 중심 + 보정. θ = 0 이면 보정 0 · 같은 해
TEST(VisualRegraspAlign, AddsPickRotOffsetForTurnOnly)
{
  for (double y : {-0.05, 0.0, 0.04}) {
    RealSpot s(0.214, y);
    ASSERT_EQ(s.blind.size(), 3u);
    const Eigen::Vector2d xy(s.x + 0.001, s.y + 0.001);
    auto tg = s.target(10.2);
    tg.align_yaw = true;
    tg.rot_mm = Eigen::Vector2d(0.4, -7.6);
    tg.rot_robot_mm = Eigen::Vector2d(-2.0, -7.0);
    const Eigen::Matrix2d I = Eigen::Matrix2d::Identity();
    const Eigen::Vector2d h = ((Eigen::Matrix2d() << 0.0, 1.0, -1.0, 0.0).finished() - I).inverse() * tg.rot_mm;   // R(−90°)
    for (double th : {-30.0, -10.0, 0.0, 15.0, 30.0}) {
      SCOPED_TRACE(std::to_string(y) + " θ " + std::to_string(th));
      RegraspObservation o{Eigen::Vector3d(xy.x(), xy.y(), 0.067), 10.3, "", true, true};
      o.yaw_known = true;
      o.yaw_rad = (s.c_deg + th) * M_PI / 180.0;
      const auto d = decide_visual_regrasp({o}, tg, 10.35, 12.0, s.m, s.tail);
      ASSERT_EQ(d.decision, RegraspDecision::USE) << d.why;
      EXPECT_TRUE(d.aligned);
      const double t = th * M_PI / 180.0;
      const double want = ((Eigen::Matrix2d() << std::cos(t), -std::sin(t), std::sin(t), std::cos(t)).finished() - I).operator*(h).norm();
      EXPECT_NEAR(d.rot_d.norm() * 1000.0, want, 0.25);   // |(R(θ) − I) h| (45° 안 = 로봇 기준 값 없음)
      if (th == 0.0) {EXPECT_LT(d.rot_d.norm(), 1e-4);}
      const auto g = arm_grip(s.m, ticks_to_q(s.m, d.poses.back().ticks));
      EXPECT_NEAR(g.tip.x(), xy.x() + d.rot_d.x(), .0015);
      EXPECT_NEAR(g.tip.y(), xy.y() + d.rot_d.y(), .0015);
      EXPECT_NEAR(g.app.z(), -1.0, .005);
      std::printf("y %+.3f θ %+3.0f°: 닫힘 %.1f° → %.1f° · 돌림 보정 (%+.2f, %+.2f) mm\n", y, th, s.c_deg, d.close_deg,
        d.rot_d.x() * 1000.0, d.rot_d.y() * 1000.0);
    }
  }
}

// 실제 로봇 사례: 먼 퍽 표로 쥔 퍽을 (0.221, −0.063) 근처에 내려놓았는데 −30° 돌아 섬 (손목 사진) + 바깥으로 밀려 위 점이 안 닿는 자리
//   → 위 점 낮춤과 면 맞춤이 같이 됨
TEST(VisualRegraspAlign, AlignWithLoweredAbove)
{
  RealSpot s(0.221, -0.063);
  ASSERT_EQ(s.blind.size(), 3u);
  const Eigen::Vector2d xy(0.232, -0.067);
  RegraspObservation o{Eigen::Vector3d(xy.x(), xy.y(), 0.067), 10.3, "", true, true};
  o.yaw_known = true;
  o.yaw_rad = (s.c_deg - 30.0) * M_PI / 180.0;
  auto tg = s.target(10.2);
  tg.max_shift_m = 0.02;
  tg.align_yaw = true;
  const auto d = decide_visual_regrasp({o}, tg, 10.35, 12.0, s.m, s.tail);
  ASSERT_EQ(d.decision, RegraspDecision::USE) << d.why;
  EXPECT_TRUE(d.low_above);
  EXPECT_TRUE(d.aligned) << d.align_note;
  const auto g = arm_grip(s.m, ticks_to_q(s.m, d.poses.back().ticks));
  EXPECT_LT(std::fabs(std::remainder(std::atan2(g.close.y(), g.close.x()) - o.yaw_rad, M_PI / 2.0)) * 180.0 / M_PI, 2.0);
  EXPECT_NEAR(g.tip.x(), xy.x(), 0.0015);
  EXPECT_NEAR(g.tip.y(), xy.y(), 0.0015);
}

// ── regrasp_rot_hand_mm: 45° 넘게 돌려 다시 쥘 때 손 지렛대 보정 ──
//   실제 로봇 사례: 네모 퍽 9 (0.240, 0.022) 닫힘 94° → 184° (로그 '손목 +91°', 실제 J5 −89°) + PICK 값 (−1.1, −14.5) mm → 앞으로 쥠 11.9 mm
namespace
{
Eigen::Matrix2d rot2d(double t)
{
  Eigen::Matrix2d R;
  R << std::cos(t), -std::sin(t), std::sin(t), std::cos(t);
  return R;
}
// 손 x 축 (base xy): J5 기본 (닫힘축 = +접선) 에서 반지름 바깥, 실제 닫힘축을 따라 돎
Eigen::Vector2d hand_x_of(const ArmModel & m, const Q5 & q)
{
  const Eigen::Vector3d c = arm_grip(m, q).close;
  return Eigen::Vector2d(c.y(), -c.x()).normalized();   // x = y × z (z = 위)
}
}  // namespace

// 식: −R(az) (R(a) − R(a_ref)) g. 같은 자세면 정확히 0, J5 +90° 와 −90° 는 닫힘 선은 같지만 보정은 거울 (g 를 손 x 에 대해 뒤집은 것)
TEST(RegraspRotHand, LeverOffsetDefinitionAndMirror)
{
  const auto m = kin_data::model();
  const Eigen::Vector2d g(-1.1, -2.2);
  for (double y : {-0.05, 0.0, 0.04}) {
    const double x = 0.222, az = std::atan2(y - m.j1_xy.y(), x - m.j1_xy.x()), tang = az * 180.0 / M_PI + 90.0;
    std::string why;
    const auto P0 = pick_poses(m, x, y, 0.068, tang, {40.0, 10.0, -50.0}, why, tang, 0.0, 0.0, 0.5);
    ASSERT_EQ(P0.size(), 3u) << why;
    const Q5 q0 = ticks_to_q(m, P0.back().ticks);
    double t0 = 99.0;
    EXPECT_EQ(wrist_lever_offset(m, x, y, q0, q0, g, &t0), Eigen::Vector2d::Zero());   // 비트까지 0
    EXPECT_NEAR(t0, 0.0, 1e-12);
    int seen = 0;
    for (double sgn : {-1.0, 1.0}) {
      // J5 를 실제로 ±90° 돌린 해 (닫힘 선은 같음 — 손가락 방향은 IK 가 J5 받침에 가까운 쪽으로 고름) 를 J5 만 바꿔 만듦
      Q5 q = q0;
      q[4] += sgn * M_PI / 2.0;
      double turn = 0.0;
      const Eigen::Vector2d d = wrist_lever_offset(m, x, y, q, q0, g, &turn) * 1000.0;
      const Eigen::Vector3d c0 = arm_grip(m, q0).close, c1 = arm_grip(m, q).close;
      const double want_turn = std::remainder(std::atan2(c1.y(), c1.x()) - std::atan2(c0.y(), c0.x()), 2.0 * M_PI) * 180.0 / M_PI;
      EXPECT_NEAR(turn, want_turn, 1e-9);
      EXPECT_GT(std::fabs(turn), 80.0);
      const double a = turn * M_PI / 180.0;
      const double a0 = 0.0;   // q0 은 접선 그대로 (아래에서 확인)
      (void)a0;
      double r0 = 0.0;
      wrist_rot_offset(m, x, y, q0, Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(), &r0);
      EXPECT_NEAR(r0, 0.0, 1.0);
      const Eigen::Vector2d want = -(rot2d(az) * ((rot2d(a + r0 * M_PI / 180.0) - rot2d(r0 * M_PI / 180.0)) * g));
      EXPECT_NEAR(d.x(), want.x(), 1e-6);
      EXPECT_NEAR(d.y(), want.y(), 1e-6);
      // 보정 뒤 손끝 어긋남 = 원래 자세와 같음: (R(a) g + d) 가 R(az) R(a0) g 와 같음 (손에 붙은 어긋남 g 가 돌아도 퍽 기준 그대로)
      const Eigen::Vector2d miss = rot2d(az) * rot2d(a + r0 * M_PI / 180.0) * g + d;
      const Eigen::Vector2d miss0 = rot2d(az) * rot2d(r0 * M_PI / 180.0) * g;
      EXPECT_NEAR((miss - miss0).norm(), 0.0, 1e-9);
      ++seen;
    }
    EXPECT_EQ(seen, 2);
    // 거울: J5 +90° 와 −90° 의 보정은 다름 (같은 닫힘 선이지만 손가락 가운데 어긋남이 반대쪽)
    Q5 qp = q0, qm = q0;
    qp[4] += M_PI / 2.0;
    qm[4] -= M_PI / 2.0;
    const Eigen::Vector2d dp = wrist_lever_offset(m, x, y, qp, q0, g), dm = wrist_lever_offset(m, x, y, qm, q0, g);
    EXPECT_GT((dp - dm).norm() * 1000.0, 3.0);
    // 손 지렛대 0 이면 보정 0
    EXPECT_LT(wrist_lever_offset(m, x, y, qp, q0, Eigen::Vector2d::Zero()).norm(), 1e-12);
  }
}

namespace
{
// 위 사례 모양: 놓은 자리 (0.229, 0.015) · regrasp_offset [−1.5, −8] → 보지 않고 쥘 점 (0.2275, 0.007), 퍽이 (0.240, 0.022) 에 섬, 잰 면 4.5°
struct Spot0118
{
  ArmModel m = kin_data::model();
  Eigen::Vector2d put{0.229, 0.015}, off{-0.0015, -0.008};
  double top = 0.068, c_deg = 0.0;
  std::vector<double> H{40.0, 10.0, -50.0};
  std::vector<PickPose> blind;
  ArmPath tail;
  Spot0118()
  {
    c_deg = (std::atan2(put.y() - m.j1_xy.y(), put.x() - m.j1_xy.x()) + M_PI / 2.0) * 180.0 / M_PI;
    const Eigen::Vector2d b = put + off;
    std::string why;
    blind = pick_poses(m, b.x(), b.y(), top, c_deg, H, why, c_deg, 0.0, 0.0, 0.5);
    auto node = [&](const char * nm, const Ticks & t) {return PathNode{nm, ticks_to_command(m, t), false, 30.0, "", false};};
    if (blind.size() == 3) {
      tail = {"regrasp_visual", {node(kRegraspObserveNode, blind[0].enc), node("re_above", blind[0].enc), node("re_down", blind[1].enc),
        node("re_grasp", blind[2].enc), node("re_up", blind[1].enc), node("re_lift", blind[0].enc), node("staging", blind[0].enc)}, false};
    }
  }
  RegraspTarget target() const
  {
    RegraspTarget t;
    t.expected = Eigen::Vector3d(put.x(), put.y(), 0.077);
    t.after = 0.0;
    t.max_age = 1.0;
    t.max_shift_m = 0.015;
    t.top = top;
    t.close_deg = c_deg;
    t.square = true;
    t.offset = off;
    t.heights = H;
    t.seed = ticks_to_q(m, blind[0].ticks);
    t.align_yaw = true;
    t.rot_mm = Eigen::Vector2d(0.4, -7.6);          // control.yaml pick_offset_rot_mm 값
    t.rot_robot_mm = Eigen::Vector2d(-2.0, -7.0);
    return t;
  }
  RegraspObservation obs(double face_deg, Eigen::Vector2d c = {0.240, 0.022}) const
  {
    RegraspObservation o{Eigen::Vector3d(c.x(), c.y(), 0.077), 0.5, "9", true, true};
    o.yaw_known = true;
    o.yaw_rad = face_deg * M_PI / 180.0;
    return o;
  }
};
}  // namespace

// 위 사례 재현: 면에 맞춘 94.5° 는 위 점 40 mm IK 가 안 돼 184.5° (= J5 −89°) 로 쥠. rot_hand_mm 없이 = PICK 값 (−1.1, −14.5) mm (로그 그대로),
//   새 = 손 지렛대 (+1.4, −3.1) mm 근처. 손끝은 잰 중심 + regrasp_offset + 보정, 실제 손목 돌림은 −89° (로그 글 +91° 가 아님)
TEST(RegraspRotHand, RealTurnUsesHandLever)
{
  Spot0118 s;
  ASSERT_EQ(s.blind.size(), 3u);
  auto tg = s.target();
  const auto o = s.obs(4.5);
  const auto old = decide_visual_regrasp({o}, tg, 1.0, 2.0, s.m, s.tail);   // rot_hand_mm NaN = PICK 값 보정
  ASSERT_EQ(old.decision, RegraspDecision::USE) << old.why;
  EXPECT_TRUE(old.aligned);
  EXPECT_NEAR(std::remainder(old.close_deg - (s.c_deg + 90.0 + 0.4), 180.0), 0.0, 0.5);
  EXPECT_NEAR(old.wrist_turn_deg, -89.0, 2.0);   // J5 −89° (닫힘 각 차는 +90°)
  EXPECT_FALSE(old.rot_lever);
  EXPECT_NEAR(old.rot_d.x() * 1000.0, -1.1, 0.3);   // 실물 로그 (−1.1, −14.5)
  EXPECT_NEAR(old.rot_d.y() * 1000.0, -14.5, 0.3);

  tg.rot_hand_mm = Eigen::Vector2d(-1.1, -2.2);
  const auto d = decide_visual_regrasp({o}, tg, 1.0, 2.0, s.m, s.tail);
  ASSERT_EQ(d.decision, RegraspDecision::USE) << d.why;
  EXPECT_TRUE(d.aligned);
  EXPECT_TRUE(d.rot_lever);
  EXPECT_NEAR(d.wrist_turn_deg, -89.0, 2.0);
  EXPECT_NEAR(std::remainder(d.close_deg - old.close_deg, 360.0), 0.0, 1e-9);   // 같은 닫힘 (손목 방향은 그대로, 보정만 다름)
  const Eigen::Vector2d xy = o.center.head<2>() + tg.offset;
  const Eigen::Vector2d want = wrist_lever_offset(s.m, xy.x(), xy.y(), ticks_to_q(s.m, old.poses.back().ticks), tg.seed, tg.rot_hand_mm);
  EXPECT_NEAR((d.rot_d - want).norm() * 1000.0, 0.0, 1e-6);
  EXPECT_NEAR(d.rot_d.x() * 1000.0, 1.4, 0.5);
  EXPECT_NEAR(d.rot_d.y() * 1000.0, -3.1, 0.5);
  const auto g = arm_grip(s.m, ticks_to_q(s.m, d.poses.back().ticks));
  EXPECT_NEAR(g.tip.x(), xy.x() + d.rot_d.x(), .0015);
  EXPECT_NEAR(g.tip.y(), xy.y() + d.rot_d.y(), .0015);
  EXPECT_NEAR(g.app.z(), -1.0, .005);
  // 손 x 축이 base −y 쪽 (J5 −90°): PICK 값 보정과 손 지렛대 보정의 차이가 손 x 로 약 −11 mm (앞으로 쥠 11.9 mm 를 지움)
  const Eigen::Vector2d hx = hand_x_of(s.m, ticks_to_q(s.m, d.poses.back().ticks));
  EXPECT_LT(hx.y(), -0.95);
  EXPECT_NEAR((d.rot_d - old.rot_d).dot(hx) * 1000.0, -11.0, 1.0);
  std::printf("01:18 재현: 닫힘 %.1f° → %.1f° · J5 %+.1f° · 예전 보정 (%+.1f, %+.1f) → 손 지렛대 (%+.1f, %+.1f) mm\n", s.c_deg, d.close_deg,
    d.wrist_turn_deg, old.rot_d.x() * 1000.0, old.rot_d.y() * 1000.0, d.rot_d.x() * 1000.0, d.rot_d.y() * 1000.0);
}

// 면이 반대쪽으로 2° 틀어지면 같은 자리에서 J5 가 +88° 로 돔 (닫힘 선은 같아도 손가락 어긋남이 거울) → 보정도 거울 쪽 (지렛대 식이 실제 해를 따름)
TEST(RegraspRotHand, PlusNinetyMirrorsCorrection)
{
  Spot0118 s;
  ASSERT_EQ(s.blind.size(), 3u);
  auto tg = s.target();
  tg.rot_hand_mm = Eigen::Vector2d(-1.1, -2.2);
  int plus = 0, minus = 0;
  for (double face : {4.5, 2.0, 0.0, -2.0, -3.0}) {
    SCOPED_TRACE(face);
    const auto o = s.obs(face);
    const auto d = decide_visual_regrasp({o}, tg, 1.0, 2.0, s.m, s.tail);
    ASSERT_EQ(d.decision, RegraspDecision::USE) << d.why;
    if (std::fabs(d.wrist_turn_deg) < 45.0) {EXPECT_FALSE(d.rot_lever); continue;}
    EXPECT_TRUE(d.rot_lever);
    const Eigen::Vector2d xy = o.center.head<2>() + tg.offset;
    const Q5 q = ticks_to_q(s.m, d.poses.back().ticks);
    // 보정 뒤 다시 푼 해의 돌림은 처음 해와 5° 안 → 처음 해 대신 이 해로 식을 확인 (보정은 xy 에서 계산)
    double t_chk = 0.0;
    wrist_lever_offset(s.m, xy.x(), xy.y(), q, tg.seed, tg.rot_hand_mm, &t_chk);
    EXPECT_NEAR(t_chk, d.wrist_turn_deg, 5.0);
    const Eigen::Vector2d hx = hand_x_of(s.m, q);
    // 손 x 성분 = −g_x (1 − cos a) + g_y sin a (지렛대가 손 x 로 옮기는 −g_x (1 − cos a) + g_y sin a 를 지움):
    //   −90° 면 −g_x − g_y = +3.3 mm, +90° 면 −g_x + g_y = −1.1 mm — 같은 닫힘 선인데 부호 · 크기가 다름 (거울)
    const double along = d.rot_d.dot(hx) * 1000.0, a = d.wrist_turn_deg * M_PI / 180.0;
    EXPECT_NEAR(along, 1.1 * (1.0 - std::cos(a)) - 2.2 * std::sin(a), 0.4);
    if (d.wrist_turn_deg < 0.0) {++minus; EXPECT_NEAR(along, 3.3, 0.5);} else {++plus; EXPECT_NEAR(along, -1.1, 0.5);}
    std::printf("면 %+.1f°: 닫힘 %.1f° · J5 %+.1f° · 보정 (%+.2f, %+.2f) mm · 손 x 성분 %+.2f mm\n", face, d.close_deg, d.wrist_turn_deg,
      d.rot_d.x() * 1000.0, d.rot_d.y() * 1000.0, along);
  }
  EXPECT_GT(minus, 0);
  EXPECT_GT(plus, 0);
}

// 45° 안 돌림 (면 맞춤 0 … 30°) · 돌림 없음은 rot_hand_mm 이 있어도 없는 것과 비트까지 같음 (실제 로봇 0 … 7° 다시 쥐기 −0.8 … 1.8 mm 그대로)
TEST(RegraspRotHand, SmallTurnsBitIdentical)
{
  for (double y : {-0.05, 0.0, 0.04}) {
    RealSpot s(0.214, y);
    ASSERT_EQ(s.blind.size(), 3u);
    const Eigen::Vector2d xy(s.x + 0.001, s.y + 0.001);
    for (double th : {-30.0, -10.0, -3.0, 0.0, 6.0, 15.0, 30.0, 44.0}) {
      SCOPED_TRACE(std::to_string(y) + " θ " + std::to_string(th));
      RegraspObservation o{Eigen::Vector3d(xy.x(), xy.y(), 0.067), 10.3, "", true, true};
      o.yaw_known = true;
      o.yaw_rad = (s.c_deg + th) * M_PI / 180.0;
      auto tg = s.target(10.2);
      tg.align_yaw = true;
      tg.rot_mm = Eigen::Vector2d(0.4, -7.6);
      tg.rot_robot_mm = Eigen::Vector2d(-2.0, -7.0);
      const auto a = decide_visual_regrasp({o}, tg, 10.35, 12.0, s.m, s.tail);
      tg.rot_hand_mm = Eigen::Vector2d(-1.1, -2.2);
      const auto b = decide_visual_regrasp({o}, tg, 10.35, 12.0, s.m, s.tail);
      ASSERT_EQ(a.decision, RegraspDecision::USE) << a.why;
      ASSERT_EQ(b.decision, RegraspDecision::USE) << b.why;
      ASSERT_LT(std::fabs(b.wrist_turn_deg), 45.0);
      EXPECT_FALSE(b.rot_lever);
      EXPECT_EQ(a.rot_d, b.rot_d);
      EXPECT_EQ(a.close_deg, b.close_deg);
      ASSERT_EQ(a.poses.size(), b.poses.size());
      for (size_t k = 0; k < a.poses.size(); ++k) {EXPECT_EQ(a.poses[k].ticks, b.poses[k].ticks); EXPECT_EQ(a.poses[k].enc, b.poses[k].enc);}
      ASSERT_EQ(a.tail.nodes.size(), b.tail.nodes.size());
      for (size_t i = 0; i < a.tail.nodes.size(); ++i) {EXPECT_EQ(a.tail.nodes[i].q, b.tail.nodes[i].q);}
      EXPECT_NEAR(b.wrist_turn_deg, th, 3.0);   // 실제 손목 돌림 = 면 맞춤 각
    }
  }
}

// 실제 로봇 기록 맞춤 (네모 · 쥔 퍽 영상): 앞으로 쥠 = s_x + d·x̂(a) + g_x (1 − cos a) − g_y sin a (손 지렛대 모형, d = 그때 더한 보정
//   팔 좌표). g = (−1.1, −2.2), s_x = 0.1 로 모든 기록이 2 mm 안 · 손 지렛대 보정이면 돌린 넷 모두 1 mm 안 (PICK 값 보정이면 −90° 에서 10 mm 넘음)
TEST(RegraspRotHand, RealRecordsFitHandLever)
{
  struct R {double x, y, a_deg, dx, dy, front;};
  const std::vector<R> rec = {
    {0.240, 0.022, -89.2, -1.1, -14.5, 11.9},   // 퍽 9
    {0.231, -0.055, -86.0, -3.5, -14.1, 10.7},
    {0.223, 0.065, -86.0, 0.6, -13.8, 9.7},     // 퍽 3
    {0.254, -0.006, 88.0, 0.0, 0.0, 1.1},       // 퍽 6 (보정 자리 IK 안 됨 → 보정 없이)
    {0.226, 0.057, 3.0, -0.3, 0.1, -0.6}, {0.215, 0.077, 7.0, -0.7, 0.2, -0.8}, {0.227, -0.026, 3.8, -0.2, 0.3, 1.8},
    {0.210, 0.083, 6.2, -0.6, 0.2, -0.7}, {0.223, 0.020, -0.7, 0.0, 0.0, -0.4}, {0.226, -0.002, 5.6, -0.4, 0.4, 0.3}};
  const auto m = kin_data::model();
  const Eigen::Vector2d g(-1.1, -2.2);
  const double sx = 0.1;
  for (const auto & r : rec) {
    const double az = std::atan2(r.y - m.j1_xy.y(), r.x - m.j1_xy.x()), a = r.a_deg * M_PI / 180.0;
    const Eigen::Vector2d xh(std::cos(a), std::sin(a));   // 팔 좌표 손 x
    auto front = [&](const Eigen::Vector2d & d_base) {
        return sx + (rot2d(-az) * d_base).dot(xh) + g.x() * (1.0 - std::cos(a)) - g.y() * std::sin(a);
      };
    EXPECT_NEAR(front(Eigen::Vector2d(r.dx, r.dy)), r.front, 2.0) << r.x << " " << r.y;
    if (std::fabs(r.a_deg) > 45.0) {
      const Eigen::Vector2d d_new = -(rot2d(az) * ((rot2d(a) - Eigen::Matrix2d::Identity()) * g));
      EXPECT_LT(std::fabs(front(d_new)), 1.0);
      // PICK 값 보정 (−90° 쪽) 은 10 mm 넘게 앞 · regrasp_offset 을 손과 같이 돌리면 반대로 10 mm 넘게 뒤
      const Eigen::Vector2d h = ((rot2d(-M_PI / 2.0) - Eigen::Matrix2d::Identity()).inverse()) * Eigen::Vector2d(0.4, -7.6);
      const Eigen::Vector2d d_old = rot2d(az) * ((rot2d(a) - Eigen::Matrix2d::Identity()) * h) + Eigen::Vector2d(-2.0, -7.0);
      const Eigen::Vector2d d_rot_off = (rot2d(a) - Eigen::Matrix2d::Identity()) * Eigen::Vector2d(-1.5, -8.0);
      if (r.a_deg < 0.0) {EXPECT_GT(front(d_old), 9.0); EXPECT_LT(front(d_rot_off), -9.0);}
      else {EXPECT_LT(front(d_old), -5.0); EXPECT_GT(front(d_rot_off), 6.0);}
    }
  }
}

// ---------------------------------------------------------------- 옮겨 놓은 누운 퍽 다시 보기 (park_relook) · 넘어진 퍽 (regrasp_topple_lying)
namespace
{
RegraspObservation lying_obs(const Eigen::Vector3d & c, double stamp, double axis_deg, const std::string & id = "")
{
  RegraspObservation o{c, stamp, id, false, true};
  o.lying = true;
  o.axis_rad = axis_deg * M_PI / 180.0;
  return o;
}
}  // namespace

// 실제 로봇 사례 A: 먼 누운 퍽을 (0.213, 0.049) 긴축 14° 로 옮겨 놓았는데 실제는 다르게 놓임 → 관찰 점에서 다시 봄.
//   놓은 자리에서 15 mm 옮겨지고 20° 돈 누운 퍽을 고름 (선 퍽 · 계획 때 이웃 자리 · 한계 밖 · 다른 ID · 오래된 장은 안 씀)
TEST(LyingRelook, PicksShiftedRotatedLyingPuckNearPutDown)
{
  const Eigen::Vector3d expected(0.213, 0.049, 0.054);
  const Eigen::Vector3d moved = expected + Eigen::Vector3d(0.015 * std::cos(0.6), 0.015 * std::sin(0.6), 0.001);   // 15 mm
  const auto got = lying_obs(moved, 10.25, 14.0 + 20.0);
  const std::vector<Eigen::Vector2d> nb{{0.29, -0.02}};   // 계획 때 본 이웃
  std::string why;
  auto p = select_lying_observation({got}, expected, "", 10.2, 10.3, .5, .06, .06, nb, &why);
  ASSERT_TRUE(p) << why;
  EXPECT_NEAR((p->center - expected).head<2>().norm(), 0.015, 1e-9);
  EXPECT_NEAR(p->axis_rad * 180.0 / M_PI, 34.0, 1e-9);
  // 선 퍽 (같은 자리) 은 안 씀 · 누운 퍽만
  RegraspObservation upright{moved, 10.25, "", true, true};
  EXPECT_FALSE(select_lying_observation({upright}, expected, "", 10.2, 10.3, .5, .06, .06, nb));
  p = select_lying_observation({upright, got}, expected, "", 10.2, 10.3, .5, .06, .06, nb);
  ASSERT_TRUE(p);
  EXPECT_TRUE(p->lying);
  // 계획 때 본 이웃 자리 (25 mm 안) 의 누운 퍽은 이웃 → 안 씀
  const auto at_nb = lying_obs(Eigen::Vector3d(0.29 + 0.01, -0.02, 0.054), 10.25, 0.0);
  EXPECT_FALSE(select_lying_observation({at_nb}, Eigen::Vector3d(0.27, -0.02, 0.054), "", 10.2, 10.3, .5, .06, .06, nb, &why));
  EXPECT_NE(why.find("이웃 자리"), std::string::npos) << why;
  // 한계 (relook_max_shift_mm 60) 밖 · 촬영 시각 TF 아님 · 대기 앞 장 · 다른 ID 는 안 씀
  EXPECT_FALSE(select_lying_observation({lying_obs(expected + Eigen::Vector3d(0.065, 0, 0), 10.25, 14.0)}, expected, "", 10.2, 10.3, .5, .06, .06, nb));
  auto inexact = got;
  inexact.exact_tf = false;
  EXPECT_FALSE(select_lying_observation({inexact}, expected, "", 10.2, 10.3, .5, .06, .06, nb));
  auto early = got;
  early.stamp = 10.1;
  EXPECT_FALSE(select_lying_observation({early}, expected, "", 10.2, 10.3, .5, .06, .06, nb));
  EXPECT_FALSE(select_lying_observation({lying_obs(moved, 10.25, 34.0, "9")}, expected, "4", 10.2, 10.3, .5, .06, .06, nb));
  // 같은 ID 면 그 ID (ID 못 읽은 검출보다 먼저), 둘 이상이면 모호
  p = select_lying_observation({got, lying_obs(moved + Eigen::Vector3d(0.004, 0, 0), 10.25, 30.0, "4")}, expected, "4", 10.2, 10.3, .5, .06, .06, nb);
  ASSERT_TRUE(p);
  EXPECT_EQ(p->id, "4");
  EXPECT_FALSE(select_lying_observation({got, lying_obs(moved + Eigen::Vector3d(0.0, 0.03, 0), 10.25, 30.0)}, expected, "", 10.2, 10.3, .5, .06, .06,
    nb, &why));
  EXPECT_NE(why.find("모호"), std::string::npos) << why;
}

// 다시 봐도 못 봄 (장 없음 · 놓은 자리 근처 누운 퍽 없음) → 없음 (계획한 자리 그대로)
TEST(LyingRelook, NothingSeenKeepsPlan)
{
  const Eigen::Vector3d expected(0.213, 0.049, 0.054);
  std::string why;
  EXPECT_FALSE(select_lying_observation({}, expected, "", 10.2, 10.3, .5, .06, .06, {}, &why));
  EXPECT_NE(why.find("새 장 없음"), std::string::npos) << why;
  RegraspObservation far_upright{Eigen::Vector3d(0.30, -0.05, 0.067), 10.25, "", true, true};
  EXPECT_FALSE(select_lying_observation({far_upright}, expected, "", 10.2, 10.3, .5, .06, .06, {}, &why));
  EXPECT_NE(why.find("근처 누운 퍽 없음"), std::string::npos) << why;
  EXPECT_FALSE(select_lying_observation({lying_obs(expected, 10.25, 14.0)}, expected, "", 10.2, 10.3, .5, .06, 0.0, {}, &why));   // 한계 0 = 입력 이상
}

// 같은 대기 (split_regrasp_observation) 를 누운 퍽 관찰 점 (reloc_observe) 뒤에 kLyingObserve 로 끼움 — 박스 다시 쥐기 (re_observe) 대기와 같은 계획에 둘 다
TEST(LyingRelook, SplitAddsLyingWaitBeforeBoxRegripWait)
{
  auto n = [](const char * name, double v, const char * grip = "", bool stop = false) {
      PathNode p{name, {v, 0.1, 0.2, 0.3, 0.4}, false, 0.0, grip, stop};
      return p;
    };
  ArmPath path{"pick", {n("look", 0.0), n("reloc_grasp", 0.1, "close"), n("reloc_lift", 0.2, "", true), n("reloc_put", 0.3, "open"),
    n("reloc_put_clear", 0.35), n(kLyingObserveNode, 0.4, "", true), n("above", 0.5, "open"), n("grasp", 0.6, "close"), n("up", 0.55),
    n("park_over", 0.7), n("re_place_clear", 0.8), n(kRegraspObserveNode, 0.85, "", true), n("re_above", 0.9, "open"), n("re_grasp", 0.95, "close"),
    n("re_up", 0.92), n("re_lift", 0.9)}, false};
  auto plan = path_actions(path, PathTiming{}, GripperValues{});
  const size_t n0 = plan.size();
  plan = split_regrasp_observation(path, std::move(plan), true, 1.0, kLyingObserveNode, kLyingObserve).plan;
  auto s = split_regrasp_observation(path, std::move(plan), true, 1.0);
  ASSERT_EQ(s.plan.size(), n0 + 2);
  ASSERT_EQ(s.tail.nodes.front().name, kRegraspObserveNode);
  int lying = -1, box = -1;
  for (size_t i = 0; i < s.plan.size(); ++i) {
    if (s.plan[i].pose == kLyingObserve) {lying = static_cast<int>(i);}
    if (s.plan[i].pose == kRegraspObserve) {box = static_cast<int>(i);}
  }
  ASSERT_GT(lying, 0);
  ASSERT_GT(box, lying);
  EXPECT_EQ(s.plan[lying - 1].pose, kLyingObserveNode);   // 관찰 점 동작 바로 뒤
  EXPECT_EQ(s.plan[box - 1].pose, kRegraspObserveNode);
  EXPECT_DOUBLE_EQ(s.plan[lying].duration_s, 1.0);
  EXPECT_EQ(s.plan[lying].phase, "reobserve");
  // 관찰 점이 멈춤이 아니면 안 끼움 (원래 계획 그대로)
  path.nodes[5].stop = false;
  auto plan2 = path_actions(path, PathTiming{}, GripperValues{});
  const size_t n2 = plan2.size();
  EXPECT_EQ(split_regrasp_observation(path, std::move(plan2), true, 1.0, kLyingObserveNode, kLyingObserve).plan.size(), n2);
}

// 실제 로봇 사례 B: 먼 퍽 표로 쥐어 세워 놓은 퍽을 다시 보기가 못 봄 (넘어졌을 수 있음). 선 퍽 결정은 '안 보임' (found false) 으로 끝나고,
//   놓은 자리 근처 누운 퍽 (넘어지면 중심이 약 47 mm 옮겨짐) 은 누운 퍽 고르기로 찾음 → 받침 세우기로. 다시 쥘 자리를 고를 때 본 이웃은 안 잡음
TEST(ToppleLying, StandingMissingButToppledLyingFound)
{
  const auto m = kin_data::model();
  const Eigen::Vector3d put(0.214, 0.0, 0.067);
  ArmPath tail{"t", {}, false};
  RegraspTarget tg;
  tg.expected = put;
  tg.after = 10.2;
  tg.max_age = 1.5;
  tg.top = 0.068;
  tg.heights = {40.0, 10.0, -50.0};
  const auto toppled = lying_obs(Eigen::Vector3d(0.214 + 0.047, 0.004, 0.054), 10.3, 2.0);
  const std::vector<Eigen::Vector2d> nb{{0.214, -0.065}};   // 다시 쥘 자리를 고를 때 본 이웃 (사이 30 mm)
  // 선 퍽 없음 → 마감 뒤 FALLBACK · found false (넘어짐 찾기로)
  auto d = decide_visual_regrasp({toppled}, tg, 11.3, 11.2, m, tail);
  EXPECT_EQ(d.decision, RegraspDecision::FALLBACK);
  EXPECT_FALSE(d.found);
  // 놓은 자리 근처 누운 퍽 (47 mm, relook_max_shift_mm 60 안)
  auto p = select_lying_observation({toppled}, put, "", 10.2, 11.3, 1.5, .06, .06, nb);
  ASSERT_TRUE(p);
  EXPECT_NEAR((p->center - put).head<2>().norm(), 0.047, 0.001);
  EXPECT_TRUE(std::fabs(p->center.z() - put.z()) < 0.04);   // 선 퍽 중심 높이 기준 ±40 mm 안 (누운 중심 13 mm 낮음)
  // 이웃 자리의 누운 퍽만 보이면 안 잡음 → 보지 않고 원래 자리에서 쥠
  EXPECT_FALSE(select_lying_observation({lying_obs(Eigen::Vector3d(0.214, -0.060, 0.054), 10.3, 0.0)}, put, "", 10.2, 11.3, 1.5, .06, .06, nb));
  // 선 퍽이 보이면 found true (넘어짐 찾기 안 함 — 지금 다시 보고 쥐기 그대로)
  RegraspObservation up{put + Eigen::Vector3d(0.003, 0, 0), 10.3, "", true, true};
  d = decide_visual_regrasp({up, toppled}, tg, 11.3, 11.2, m, tail);
  EXPECT_TRUE(d.found);
}

// 사례 B: 선 퍽도 누운 퍽도 안 보임 → 넘어짐 찾기 없음 (보지 않고 원래 자리에서 쥠, 빈손이면 온 길로 되돌아감)
TEST(ToppleLying, NothingSeenKeepsBlindRegrip)
{
  const auto m = kin_data::model();
  const Eigen::Vector3d put(0.214, 0.0, 0.067);
  RegraspTarget tg;
  tg.expected = put;
  tg.after = 10.2;
  const auto d = decide_visual_regrasp({}, tg, 11.3, 11.2, m, ArmPath{});
  EXPECT_EQ(d.decision, RegraspDecision::FALLBACK);
  EXPECT_FALSE(d.found);
  std::string why;
  EXPECT_FALSE(select_lying_observation({}, put, "", 10.2, 11.3, 1.5, .06, .06, {}, &why));
}

// ── 받침에 놓은 원기둥 다시 보기 (park_insert_look · split_regrasp_observation · decide_park_regrasp) ──
namespace
{
Eigen::Isometry3d urdf_T5c_vr()
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = (Eigen::AngleAxisd(-1.995706, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(-0.048951, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(-2.969292, Eigen::Vector3d::UnitX())).toRotationMatrix();
  T.translation() = Eigen::Vector3d(0.085672, 0.001345, 0.004098);
  return T;
}
// control_master park_upright_again 과 같은 받침 경로 (원기둥, 박스 (x, y) 를 손 기울여 쥔 뒤): look = 관찰 점 끼움
struct ParkFlow
{
  ArmModel m{kin_data::model()};
  ParkCfg pk;
  UprightCfg u;
  std::vector<ParkNode> pn;
  ParkLookInfo li;
  Q5 q_obs{}, q_ra{}, q_rd{};
  ArmPath path;
  bool ok{false};
  ParkFlow(double x, double y, bool look, bool lift = false)
  {
    pk.regrasp_r = 0.0;
    const double top = 0.037 + 0.060 - 0.039;
    std::string why;
    auto P = pick_poses(m, x, y, top, std::nan(""), {40.0, 10.0, -50.0}, why, std::nan(""), 10.0);
    if (P.size() != 3) {P = pick_poses(m, x, y, top, std::nan(""), {40.0, 10.0, -40.0}, why, std::nan(""), 40.0, 20.0, 20.0);}
    if (P.size() != 3) {return;}
    UprightHeld h;
    h.q_grasp = ticks_to_q(m, P.back().ticks);
    h.puck_xy = Eigen::Vector2d(x, y);
    h.bottom_z = top - 0.060;
    h.square = false;
    h.width = 0.0335;
    UprightInfo info;
    if (!park_upright_nodes(m, pk, u, h, ticks_to_q(m, P[0].ticks), pn, info, why)) {return;}
    for (const auto & n : pn) {
      if (n.name == "park_re_above") {q_ra = n.q;}
      if (n.name == "park_re_down") {q_rd = n.q;}
    }
    const auto up = std::find_if(pn.begin(), pn.end(), [](const ParkNode & n) {return n.name == "pk_up";});
    if (look && lift && !park_lift_look(m, urdf_T5c_vr(), pk, u, ParkLookCfg{}, up->q, q_ra, q_obs, li, why)) {return;}
    if (look && !lift && !park_look_pose(m, urdf_T5c_vr(), pk, u, ParkLookCfg{}, up->q, q_ra, q_obs, li, why)) {return;}
    if (look) {park_insert_look(pn, false, true, q_obs, li, kRegraspObserveNode);}
    path = {"pick", {{"lift", ticks_to_command(m, P[0].enc), false, 0.0, "", false}}, false};
    for (const auto & n : pn) {
      path.nodes.push_back({n.name, ticks_to_command(m, q_to_ticks(m, encoder_goal_q(m, n.q, n.held_kg))), false, n.slow ? 50.0 : 0.0, n.grip,
        n.name == kRegraspObserveNode});
    }
    ok = true;
  }
  ParkRegraspTarget target(double after) const
  {
    ParkRegraspTarget t;
    t.expected = Eigen::Vector3d(pk.center.x(), pk.center.y(), 0.1365 + 0.030);   // base_link (park_floor_z + 반 높이)
    t.after = after;
    t.park = pk;
    t.upright = u;
    t.close = arm_grip(m, q_rd).close;
    t.seed = q_ra;
    t.held_kg = pk.held_kg;
    return t;
  }
  ArmPath tail() const
  {
    const auto n = std::find_if(path.nodes.begin(), path.nodes.end(), [](const PathNode & p) {return p.name == kRegraspObserveNode;});
    return {"regrasp_visual", {n, path.nodes.end()}, false};
  }
};
const PathTiming kJetsonTiming{150.0, 20.0, 375.0, 0.05, 0.3};
}  // namespace

// 관찰 끼우기: 원기둥 받침 경로의 re_observe (멈춤) 동작 뒤에 대기 하나 (그리퍼 · 다른 동작 그대로), 후반 = re_observe · park_re_above · park_re_down · park_re_lift
TEST(ParkRelook, SplitAddsWaitAfterParkObservePoint)
{
  ParkFlow f(0.256, 0.052, true);
  ASSERT_TRUE(f.ok);
  const auto plan = path_actions(f.path, kJetsonTiming, GripperValues{}, f.path.nodes.front().q);
  const auto split = split_regrasp_observation(f.path, plan, true, 1.0);
  ASSERT_EQ(split.plan.size(), plan.size() + 1);
  ASSERT_EQ(split.tail.nodes.size(), 4u);
  EXPECT_EQ(split.tail.nodes[0].name, kRegraspObserveNode);
  EXPECT_EQ(split.tail.nodes[1].name, "park_re_above");
  EXPECT_EQ(split.tail.nodes[2].name, "park_re_down");
  EXPECT_EQ(split.tail.nodes[3].name, "park_re_lift");
  size_t k = 0;
  bool waited = false;
  for (const auto & a : split.plan) {
    if (a.pose == kRegraspObserve) {
      ASSERT_GT(k, 0u);
      EXPECT_EQ(plan[k - 1].pose, kRegraspObserveNode);
      EXPECT_DOUBLE_EQ(a.duration_s, 1.0);
      waited = true;
      continue;
    }
    EXPECT_EQ(a.pose, plan[k].pose);
    EXPECT_EQ(a.is_gripper, plan[k].is_gripper);
    EXPECT_EQ(a.gripper, plan[k].gripper);
    ++k;
  }
  EXPECT_TRUE(waited);
  // 원기둥 · 관찰 없이 (꺼짐 · 관찰 자세 없음) 만든 경로 = 대기 없음 (지금 동작)
  ParkFlow g(0.256, 0.052, false);
  ASSERT_TRUE(g.ok);
  const auto p2 = path_actions(g.path, kJetsonTiming, GripperValues{}, g.path.nodes.front().q);
  const auto s2 = split_regrasp_observation(g.path, p2, true, 1.0);
  EXPECT_EQ(s2.plan.size(), p2.size());
  EXPECT_TRUE(s2.tail.nodes.empty());
}

// 잰 중심 (치우침 뺀 뒤) 으로 park_re_* 만 바뀜: 손끝 = 잰 중심, 높이 · 깊이 · 닫힘 그대로, 관찰 점 그대로
TEST(ParkRelook, UsesMeasuredCenterMinusBias)
{
  ParkFlow f(0.248, 0.065, true);
  ASSERT_TRUE(f.ok);
  const ArmPath tail = f.tail();
  auto tg = f.target(10.2);
  tg.bias = Eigen::Vector2d(0.002, -0.001);   // 잰 값 − 실제
  const Eigen::Vector2d R = (f.pk.center - f.m.j1_xy).normalized(), T(-R.y(), R.x());
  const Eigen::Vector2d real = f.pk.center + 0.006 * R - 0.004 * T;   // 받침 안에서 반지름 6 mm · 접선 −4 mm 굴러간 퍽
  RegraspObservation o{Eigen::Vector3d(real.x() + tg.bias.x(), real.y() + tg.bias.y(), 0.1665), 10.3, "", true, true};
  const auto d = decide_park_regrasp({o}, tg, 10.35, 11.2, f.m, tail);
  ASSERT_EQ(d.decision, RegraspDecision::USE) << d.why;
  EXPECT_NEAR((d.xy - real).norm(), 0.0, 1e-9);
  EXPECT_NEAR(d.shift_mm, std::hypot(6.0, 4.0), 1e-6);
  ASSERT_EQ(d.tail.nodes.size(), tail.nodes.size());
  EXPECT_EQ(d.tail.nodes[0].q, tail.nodes[0].q);   // 관찰 점 그대로
  // 바뀐 명령 = 실제 퍽 자리로 푼 park_regrasp_at 해 (빈손 · 빈손 · held_kg 처짐 앞먹임), 그 해의 손끝 = 실제 퍽 · 높이 · 깊이 · 닫힘 그대로
  Q5 qa{}, qd{};
  std::string why;
  ASSERT_TRUE(park_regrasp_at(f.m, f.pk, f.u, real, tg.close, tg.seed, qa, qd, why)) << why;
  auto cmd = [&](const Q5 & q, double kg) {return ticks_to_command(f.m, q_to_ticks(f.m, encoder_goal_q(f.m, q, kg)));};
  const double ptop = f.pk.floor_z + f.u.puck_h;
  int seen = 0;
  for (const auto & n : d.tail.nodes) {
    if (n.name == "park_re_above") {EXPECT_EQ(n.q, cmd(qa, 0.0)); ++seen;}
    if (n.name == "park_re_down") {EXPECT_EQ(n.q, cmd(qd, 0.0)); ++seen;}
    if (n.name == "park_re_lift") {EXPECT_EQ(n.q, cmd(qa, f.pk.held_kg)); ++seen;}
  }
  EXPECT_EQ(seen, 3);
  const Grip ga = arm_grip(f.m, qa), gd = arm_grip(f.m, qd);
  EXPECT_NEAR((gd.tip.head<2>() - real).norm(), 0.0, 1e-3);
  EXPECT_NEAR(gd.tip.z(), ptop - 0.030, 1e-3);
  EXPECT_NEAR(ga.tip.z(), ptop + 0.0075, 1e-3);
  EXPECT_GT(gd.close.normalized().dot(tg.close.normalized()), 0.999);
  for (size_t i = 0; i < tail.nodes.size(); ++i) {
    EXPECT_EQ(d.tail.nodes[i].grip, tail.nodes[i].grip);
    EXPECT_EQ(d.tail.nodes[i].stop, tail.nodes[i].stop);
    EXPECT_EQ(d.tail.nodes[i].speed_dps, tail.nodes[i].speed_dps);
  }
  // 퍽이 정확히 가운데 + 치우침 0 이면 원래 경로와 같은 자리 (명령 틱 차 2 틱 안)
  auto t0 = f.target(10.2);
  RegraspObservation c0{Eigen::Vector3d(f.pk.center.x(), f.pk.center.y(), 0.1665), 10.3, "", true, true};
  const auto d0 = decide_park_regrasp({c0}, t0, 10.35, 11.2, f.m, tail);
  ASSERT_EQ(d0.decision, RegraspDecision::USE);
  for (size_t i = 0; i < tail.nodes.size(); ++i) {
    for (int j = 0; j < 5; ++j) {EXPECT_NEAR(d0.tail.nodes[i].q[j], tail.nodes[i].q[j], 2.0 * 2.0 * M_PI / 4096.0) << tail.nodes[i].name;}
  }
}

// 못 보면 그대로: 새 장 없음 (기한 전 WAIT · 뒤 FALLBACK) · 오래됨 · 촬영 TF 없음 · 누운 퍽 · 모호 · 너무 옮겨짐 · IK 안 됨 · 후반 점 모자람 → FALLBACK (tail 안 바꿈)
TEST(ParkRelook, FallbackKeepsBlindCenterRegrasp)
{
  ParkFlow f(0.259, 0.024, true);
  ASSERT_TRUE(f.ok);
  const ArmPath tail = f.tail();
  const auto tg = f.target(10.2);
  RegraspObservation good{Eigen::Vector3d(f.pk.center.x() + 0.003, f.pk.center.y(), 0.1665), 10.3, "", true, true};
  auto old = good;
  old.stamp = 10.1;
  EXPECT_EQ(decide_park_regrasp({old}, tg, 10.4, 11.2, f.m, tail).decision, RegraspDecision::WAIT);
  auto d = decide_park_regrasp({old}, tg, 11.2, 11.2, f.m, tail);
  EXPECT_EQ(d.decision, RegraspDecision::FALLBACK);
  EXPECT_FALSE(d.found);
  EXPECT_NE(d.why.find("새 관측 없음"), std::string::npos) << d.why;
  EXPECT_EQ(decide_park_regrasp({}, tg, 11.2, 11.2, f.m, tail).decision, RegraspDecision::FALLBACK);
  EXPECT_EQ(decide_park_regrasp({good}, tg, 10.9, 10.9, f.m, tail).decision, RegraspDecision::FALLBACK);   // 0.5 s 넘은 장
  auto latest = good;
  latest.exact_tf = false;
  EXPECT_EQ(decide_park_regrasp({latest}, tg, 10.35, 10.35, f.m, tail).decision, RegraspDecision::FALLBACK);
  auto lying = good;
  lying.upright = false;
  lying.lying = true;
  EXPECT_EQ(decide_park_regrasp({lying}, tg, 10.35, 10.35, f.m, tail).decision, RegraspDecision::FALLBACK);
  auto twin = good;
  twin.center.y() += 0.006;
  d = decide_park_regrasp({good, twin}, tg, 10.35, 10.35, f.m, tail);
  EXPECT_EQ(d.decision, RegraspDecision::FALLBACK);
  EXPECT_NE(d.why.find("모호"), std::string::npos) << d.why;
  // 너무 옮겨짐: ID 로 고른 12 mm (한계 10) → 기한 전이라도 바로 FALLBACK. 치우침을 빼면 한계 안 → USE
  auto far = good;
  far.center.x() = f.pk.center.x() + 0.012;
  far.id = "7";
  auto tg7 = tg;
  tg7.id = "7";
  d = decide_park_regrasp({far}, tg7, 10.35, 11.2, f.m, tail);
  EXPECT_EQ(d.decision, RegraspDecision::FALLBACK);
  EXPECT_TRUE(d.found);
  EXPECT_NE(d.why.find("park_look_max_shift_mm"), std::string::npos) << d.why;
  auto tgb = tg7;
  tgb.bias = Eigen::Vector2d(0.008, 0.0);
  EXPECT_EQ(decide_park_regrasp({far}, tgb, 10.35, 11.2, f.m, tail).decision, RegraspDecision::USE);
  // IK 안 됨 (한계 없음 + 팔에서 먼 중심)
  auto open = tg7;
  open.max_shift_m = std::nan("");
  open.id_radius = 1.0;
  auto unreach = far;
  unreach.center.x() = 0.60;
  d = decide_park_regrasp({unreach}, open, 10.35, 11.2, f.m, tail);
  EXPECT_EQ(d.decision, RegraspDecision::FALLBACK);
  EXPECT_NE(d.why.find("IK"), std::string::npos) << d.why;
  EXPECT_TRUE(d.tail.nodes.empty());
  // 후반에 park_re_down 없음
  ArmPath broken = tail;
  broken.nodes.erase(broken.nodes.begin() + 2);
  d = decide_park_regrasp({good}, tg, 10.35, 11.2, f.m, broken);
  EXPECT_EQ(d.decision, RegraspDecision::FALLBACK);
  EXPECT_TRUE(d.tail.nodes.empty());
  EXPECT_EQ(decide_park_regrasp({good}, tg, 10.35, 11.2, f.m, tail).decision, RegraspDecision::USE);
}

// 관찰 점이 더하는 동작 시간 (Jetson 시간표, 보는 시간 1 s 이하 제외): 손 10 · 20 · 30° 자리
TEST(ParkRelook, ObserveMotionTime)
{
  auto total = [](const std::vector<ArmAction> & plan) {
      double t = 0.0;
      for (const auto & a : plan) {t += a.is_gripper ? 0.7 : a.duration_s;}
      return t;
    };
  for (const auto & c : {std::pair{0.256, 0.052}, std::pair{0.300, 0.050}, std::pair{0.33, -0.09}}) {
    ParkFlow a(c.first, c.second, false), b(c.first, c.second, true);
    ASSERT_TRUE(a.ok && b.ok);
    const double ta = total(path_actions(a.path, kJetsonTiming, GripperValues{}, a.path.nodes.front().q));
    const double tb = total(path_actions(b.path, kJetsonTiming, GripperValues{}, b.path.nodes.front().q));
    ParkFlow l(c.first, c.second, true, true);   // 올린 점에서 보기
    ASSERT_TRUE(l.ok);
    const double tl = total(path_actions(l.path, kJetsonTiming, GripperValues{}, l.path.nodes.front().q));
    std::printf("(%.3f, %+.3f): 받침 흐름 %.2f s → 관찰 점 거쳐 %.2f s (%+.2f s) · 올린 점 (%.0f mm) 에서 보기 %.2f s (%+.2f s) (보는 대기 · 동작 경계 제외)\n",
      c.first, c.second, ta, tb, tb - ta, l.li.lift_m * 1000.0, tl, tl - ta);
    EXPECT_LT(tb - ta, 4.0);
    EXPECT_LT(tl, tb);
  }
}

// 다시 쥘 자리 찾기의 기울인 손 놓기 거름 (control_master regrasp_nodes put_min): x 축 위에서 잰 가장 가까운 거리보다 2 mm 넘게 가까운
//   자리는 어느 방위에서도 기울인 놓기 (손 t ± 0.5°, 기본 씨앗) 가 안 풀림 → IK 없이 넘겨도 고르는 자리가 같음. 손 40° · 깊이 50 (실제 로봇 퍽 6) 은 0.239 m 부터
TEST(RegraspSpeed, TiltedPutNearLimitIsDirectionFree)
{
  const auto m = kin_data::model();
  const double top = 0.037 + 0.060 + m.team_base_z_floor - m.base_z_floor + 0.010;   // 박스 위 선 퍽 윗면 + pick_top_bias 10
  auto put_ok = [&](double x, double y, double tilt, const std::vector<double> & hp) {
      const double c = (std::atan2(y - m.j1_xy.y(), x - m.j1_xy.x()) + M_PI / 2.0) * 180.0 / M_PI;
      std::string w;
      return pick_poses(m, x, y, top, c, hp, w, c, 40.0, tilt, 0.5).size() == hp.size();
    };
  auto put_min = [&](double tilt, const std::vector<double> & hp) {   // control_master put_min 과 같은 잼
      auto ok = [&](double r) {return put_ok(m.j1_xy.x() + r, m.j1_xy.y(), tilt, hp);};
      double lo = 0.17, hi = -1.0;
      if (ok(lo)) {return lo;}
      for (double pr : {0.22, 0.25, 0.28, 0.31, 0.34, 0.37}) {if (ok(pr)) {hi = pr; break;}}
      if (hi < 0.0) {return 1e9;}
      while (hi - lo > 0.0005) {const double mid = 0.5 * (lo + hi); if (ok(mid)) {hi = mid;} else {lo = mid;}}
      return hi;
    };
  const double r40 = put_min(40.0, {40.0, 10.0, 8.0 - 40.0}), r50 = put_min(40.0, {40.0, 10.0, 8.0 - 50.0});
  EXPECT_GT(r40, 0.20);
  EXPECT_LT(r40, 0.23);
  EXPECT_GT(r50, 0.2390);   // 퍽 6: 수직 다시 쥐기 (0.2106 m 까지) 와 겹치는 자리 없음
  int n = 0;
  for (double a = -40.0; a <= 40.0 + 1e-9; a += 10.0) {
    for (double dr : {0.0025, 0.005, 0.010, 0.020}) {
      const double r = r40 - dr;
      EXPECT_FALSE(put_ok(m.j1_xy.x() + r * std::cos(a * M_PI / 180.0), r * std::sin(a * M_PI / 180.0), 40.0, {40.0, 10.0, 8.0 - 40.0})) << a << " " << r;
      ++n;
    }
  }
  EXPECT_EQ(n, 36);
  std::printf("기울인 놓기 가장 가까운 거리: 손 40° 깊이 40 → %.4f m · 깊이 50 → %.4f m\n", r40, r50);
}

// ---------------------------------------------------------------- 실제 로봇 사례 재현 (받침 다시 보기 4 번 모두 '새 장에 놓은 퍽 없음')
namespace
{
// 손목 카메라 drift (~/.ros/camera_drift/last.txt, 3.30°) — control_master 가 손목 검출에 곱하는 값
const Eigen::Quaterniond kDrift1903(0.999585, -0.005038, -0.028124, -0.00357);
Eigen::Isometry3d iso(const std::array<double, 3> & t, const std::array<double, 4> & xyzw)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = Eigen::Quaterniond(xyzw[3], xyzw[0], xyzw[1], xyzw[2]).normalized().toRotationMatrix();
  T.translation() = Eigen::Vector3d(t[0], t[1], t[2]);
  return T;
}
// 곧게 올린 점 (손 30° · 10° · 30° · 30°) 을 지날 때 /perception/wrist/tags 퍽 태그 한 장: 왜곡 푼 모서리 · base_link ← d435_color_optical_frame (그 시각 /tf) · 관절
struct TagFrame1903 {int win, id; std::array<double, 8> cn; std::array<double, 3> t; std::array<double, 4> q; std::array<double, 5> j;};
const TagFrame1903 kLiftTags1903[] = {
  {1, 11, {0.173803, 0.101096, 0.172186, 0.000199, 0.273830, -0.003990, 0.275968, 0.097250}, {0.119841, 0.039573, 0.349723}, {0.722535, -0.690579, -0.000277, 0.032296}, {0.0169, -0.8145, -0.2454, 0.5123, -0.0460}},
  {2, 2, {0.203704, 0.016379, 0.290928, 0.070160, 0.237350, 0.155887, 0.149987, 0.101309}, {0.119948, 0.039575, 0.350104}, {0.722536, -0.690603, 0.000247, 0.031777}, {0.0169, -0.8130, -0.2454, 0.5139, -0.0460}},
  {3, 1, {0.169105, 0.061453, 0.090345, -0.000268, 0.151524, -0.081924, 0.230325, -0.018255}, {0.116017, 0.040990, 0.344477}, {0.736596, -0.673660, 0.013393, 0.058561}, {-0.0399, -0.8483, -0.2178, 0.5216, -0.1626}},
  {4, 5, {0.206975, -0.003902, 0.303898, 0.050417, 0.247490, 0.148053, 0.150845, 0.093568}, {0.120130, 0.039488, 0.333593}, {0.722093, -0.691110, 0.000646, 0.030811}, {0.0169, -0.9265, -0.0215, 0.3973, -0.0445}},
};
// 손 72° 관찰 자세 대기 중 /perception/wrist/detections (받침 퍽, ID 못 읽음 · 태그 0 장): 카메라 광학 프레임 위치 · 자세 · 그 시각 base ← 카메라
struct DetFrame1903 {int win; std::array<double, 3> p; std::array<double, 4> q; std::array<double, 3> t; std::array<double, 4> tq;};
const DetFrame1903 kObsDets1903[] = {
  {1, {-0.04041, 0.07292, 0.35903}, {-0.91466, -0.06202, 0.15390, 0.36859}, {-0.004836, 0.058978, 0.375840}, {0.668580, -0.637682, 0.365071, -0.114395}},
  {2, {-0.04885, 0.09290, 0.40878}, {0.92932, 0.05248, 0.14542, 0.33536}, {-0.004874, 0.058985, 0.375465}, {0.668833, -0.637808, 0.364508, -0.114006}},
  {3, {-0.04132, 0.07080, 0.35269}, {0.92778, 0.05335, 0.14589, 0.33926}, {-0.006198, 0.059253, 0.374989}, {0.666936, -0.637618, 0.367713, -0.115876}},
  {4, {-0.04278, 0.07697, 0.36047}, {0.92897, 0.05278, 0.14590, 0.33606}, {-0.005000, 0.059061, 0.375603}, {0.668067, -0.638183, 0.365210, -0.114150}},
};
}  // namespace

// 실물 재현: (1) 손 72° 관찰 자세의 깊이 검출은 깊이 0.35–0.41 m (뒤 바닥) → base z 가 받침 퍽 중심 (0.1665) 에서 100 mm 넘게 낮아
//   고르기에서 빠짐 = 실물 '새 장에 놓은 퍽 없음' 그대로. (2) 올린 점의 태그 장은 받침 퍽 윗면 높이 (0.1965) 에 내리면 한 변 14.5–16 mm (15.3 ± 6 % 안),
//   중심이 받침 가운데에서 10 mm 안팎 → 받침 다시 보기가 그 장을 고름 (창 3 은 11.7 mm 라 한계 10 mm 로 지금 동작 = 가운데에서 쥠)
//   칸 퍽 평면 (칸 바닥 0.1325 + 퍽 0.06) 에 내려도 받침과 4 mm 차라 크기는 맞음 — 실물 로그 '퍽 태그 11 안 씀 (16.4 mm)' 은 올라가는 중 다른 장
TEST(ParkRelook, ReplayLiftTagsAtParkHeight)
{
  const double top = 0.1365 + 0.060;
  const Eigen::Vector2d park(0.1008, 0.00115);
  TagPlaneConfig pc;
  pc.plane_z = top;
  pc.tag_size_m = 0.0153;
  pc.size_tol = 0.06;
  pc.min_view_cos = 0.94;
  ParkFlow f(0.259, 0.024, true, true);
  ASSERT_TRUE(f.ok);
  EXPECT_TRUE(f.li.at_lift);
  const ArmPath tail = f.tail();
  // (1) 72° 관찰 자세 깊이 검출 → 고르기에서 빠짐 (실물 실패 재현)
  for (const auto & d : kObsDets1903) {
    const Eigen::Isometry3d T = iso(d.t, d.tq);
    const Eigen::Vector3d c = T * (kDrift1903 * Eigen::Vector3d(d.p[0], d.p[1], d.p[2]));
    EXPECT_LT(c.z(), 0.1665 - 0.09) << "창 " << d.win;
    auto tg = f.target(10.0);
    RegraspObservation o{c, 10.3, "", true, true};
    const auto r = decide_park_regrasp({o}, tg, 10.4, 11.0, f.m, tail);
    EXPECT_EQ(r.decision, RegraspDecision::WAIT) << "창 " << d.win;
    EXPECT_NE(r.why.find("새 장에 놓은 퍽 없음"), std::string::npos) << r.why;
  }
  // (2) 올린 점 태그 장 → 받침 높이 평면 → 고름
  const double expect_mm[4][2] = {{4.4, 5.2}, {-1.1, 5.3}, {7.8, 8.7}, {3.5, 8.1}};   // bag 분석 (같은 계산, 파이썬)
  int use = 0;
  for (const auto & e : kLiftTags1903) {
    Eigen::Isometry3d T = iso(e.t, e.q);
    T.linear() = T.linear() * kDrift1903.toRotationMatrix();
    std::array<Eigen::Vector2d, 4> cn;
    for (int k = 0; k < 4; ++k) {cn[k] = Eigen::Vector2d(e.cn[2 * k], e.cn[2 * k + 1]);}
    const TagOnPlane hit = tag_on_plane(cn, T, pc);
    ASSERT_TRUE(hit.ok) << "창 " << e.win << ": " << hit.why << " " << hit.side_m;
    EXPECT_NEAR(hit.side_m, 0.0153, 0.0153 * 0.06);
    const Eigen::Vector2d dmm = (hit.center - park) * 1000.0;
    EXPECT_NEAR(dmm.x(), expect_mm[e.win - 1][0], 0.3) << "창 " << e.win;
    EXPECT_NEAR(dmm.y(), expect_mm[e.win - 1][1], 0.3) << "창 " << e.win;
    auto tg = f.target(10.0);
    tg.id = std::to_string(e.id);
    tg.tag_wait_s = 0.5;
    tg.max_shift_m = 0.010;
    RegraspObservation o{Eigen::Vector3d(hit.center.x(), hit.center.y(), top - 0.030), 10.3, tg.id, true, true};
    o.tagged = true;
    // 깊이 검출 (72° 자세에서처럼 높이가 틀린 것) 이 같이 와도 태그를 먼저 · 깊이만 있으면 0.5 s 기다림
    RegraspObservation bad{Eigen::Vector3d(park.x() + 0.06, park.y(), 0.05), 10.3, "", true, true};
    const auto r = decide_park_regrasp({bad}, {o}, tg, 10.35, 11.0, f.m, tail);
    ASSERT_TRUE(r.found) << r.why;
    EXPECT_TRUE(r.from_tag);
    std::printf("창 %d 태그 %d: 받침 높이 평면 중심 받침 가운데에서 (%+.1f, %+.1f) mm · 한 변 %.1f mm → %s %s\n", e.win, e.id, dmm.x(), dmm.y(),
      hit.side_m * 1000.0, r.decision == RegraspDecision::USE ? "다시 보고 쥠" : "가운데에서 쥠", r.why.c_str());
    if (dmm.norm() <= 10.0) {EXPECT_EQ(r.decision, RegraspDecision::USE) << r.why; ++use;}
    else {EXPECT_EQ(r.decision, RegraspDecision::FALLBACK); EXPECT_NE(r.why.find("park_look_max_shift_mm"), std::string::npos) << r.why;}
  }
  EXPECT_EQ(use, 3);
}

// 태그 먼저: 태그 장 없이 깊이 검출만 오면 after + tag_wait_s 까지 WAIT, 그 뒤 깊이로. 태그가 크기 검사로 빠져 없으면 같은 흐름 (지금처럼 깊이 · 가운데)
TEST(ParkRelook, TagFirstThenDepthAfterWait)
{
  ParkFlow f(0.259, 0.024, true, true);
  ASSERT_TRUE(f.ok);
  const ArmPath tail = f.tail();
  auto tg = f.target(10.0);
  tg.tag_wait_s = 0.5;
  RegraspObservation depth{Eigen::Vector3d(f.pk.center.x() + 0.002, f.pk.center.y() + 0.001, 0.1665), 10.3, "", true, true};
  const auto w = decide_park_regrasp({depth}, {}, tg, 10.35, 11.0, f.m, tail);
  EXPECT_EQ(w.decision, RegraspDecision::WAIT);
  EXPECT_NE(w.why.find("태그 기다림"), std::string::npos) << w.why;
  RegraspObservation depth2 = depth;
  depth2.stamp = 10.55;
  const auto u = decide_park_regrasp({depth2}, {}, tg, 10.6, 11.0, f.m, tail);
  ASSERT_EQ(u.decision, RegraspDecision::USE) << u.why;
  EXPECT_FALSE(u.from_tag);
  // 태그 없이 부르면 (tag_wait_s 0) 그대로 바로
  auto t0 = f.target(10.0);
  const auto old = decide_park_regrasp({depth}, t0, 10.35, 11.0, f.m, tail);
  EXPECT_EQ(old.decision, RegraspDecision::USE) << old.why;
  // 기한까지 아무것도 없으면 FALLBACK (지금 동작)
  const auto none = decide_park_regrasp({}, {}, tg, 11.0, 11.0, f.m, tail);
  EXPECT_EQ(none.decision, RegraspDecision::FALLBACK);
}

// ---- 원기둥 손 안 앞뒤 보정 (pick_offset_cyl_hand_mm) ----
// 실제 로봇: 박스 원기둥 쥔 퍽 영상 14 번 모두 '뒤로 쥠' (평균 −2.2 mm) → 손 앞 (손 x) 으로 옮겨 쥠.
//   수직 손 · J5 기본 (닫힘 = 접선) 이면 손 앞 = J1 에서 반지름 바깥, 크기 그대로
TEST(CylHandOffset, ShiftIsHandFrontRadialOutForVerticalHand)
{
  for (double y : {-0.07, -0.03, 0.0, 0.04, 0.08}) {
    RealSpot s(0.20, y);
    ASSERT_EQ(s.blind.size(), 3u) << y;
    const Q5 q = ticks_to_q(s.m, s.blind[2].ticks);
    const Eigen::Vector2d d = cyl_hand_shift(s.m, q, Eigen::Vector2d(2.5, 0.0));
    const Eigen::Vector2d out = (Eigen::Vector2d(s.x, s.y) - s.m.j1_xy).normalized();
    EXPECT_NEAR(d.norm(), 0.0025, 1e-4) << y;
    EXPECT_GT(d.normalized().dot(out), std::cos(5.0 * M_PI / 180.0)) << y;   // 반지름 바깥
    EXPECT_NEAR((d - 0.0025 * hand_x_of(s.m, q)).norm(), 0.0, 1e-4) << y;
    const Eigen::Vector2d l = cyl_hand_shift(s.m, q, Eigen::Vector2d(0.0, 1.0));   // 손 왼쪽 = 왼손가락 = 닫힘축 (close)
    const Eigen::Vector3d c = arm_grip(s.m, q).close;
    EXPECT_GT(l.normalized().dot(Eigen::Vector2d(c.x(), c.y()).normalized()), 0.99) << y;
    EXPECT_TRUE(same_hand_front(s.m, q, q, 1.0));
    Q5 qf = q;
    qf[4] += M_PI;   // 같은 닫힘축 · 손 앞 뒤집힘
    EXPECT_FALSE(same_hand_front(s.m, q, qf, 10.0));
    EXPECT_EQ(cyl_hand_shift(s.m, q, Eigen::Vector2d(std::nan(""), 0.0)), Eigen::Vector2d::Zero());
  }
}

// 박스에 내려놓고 다시 보고 쥐기: 원기둥 (square false) · 깊이 점 중심 관측만 손 앞으로 2.5 mm. 네모 · 태그 중심 · 0 은 보정 없는 결과와 비트까지 같음
TEST(CylHandOffset, VisualRegraspShiftsUntaggedCylinderOnly)
{
  for (double y : {-0.05, 0.0, 0.05}) {
    RealSpot s(0.214, y);
    ASSERT_EQ(s.blind.size(), 3u) << y;
    const Eigen::Vector2d xy(s.x + 0.002, s.y - 0.001);
    RegraspObservation o{Eigen::Vector3d(xy.x(), xy.y(), 0.067), 10.3, "", true, true};
    auto tg = s.target(10.2);
    tg.square = false;
    const auto d0 = decide_visual_regrasp({o}, tg, 10.35, 12.0, s.m, s.tail);   // 보정 0
    ASSERT_EQ(d0.decision, RegraspDecision::USE) << d0.why;
    EXPECT_LT(d0.cyl_d.norm(), 1e-9);
    tg.cyl_hand_mm = Eigen::Vector2d(2.5, 0.0);
    const auto d = decide_visual_regrasp({o}, tg, 10.35, 12.0, s.m, s.tail);
    ASSERT_EQ(d.decision, RegraspDecision::USE) << d.why;
    const Q5 qg = ticks_to_q(s.m, d.poses[2].ticks);
    const auto g = arm_grip(s.m, qg), g0 = arm_grip(s.m, ticks_to_q(s.m, d0.poses[2].ticks));
    const Eigen::Vector2d hx = hand_x_of(s.m, qg);
    EXPECT_NEAR((g.tip.head<2>() - g0.tip.head<2>()).dot(hx), 0.0025, 0.0004) << y;   // 손 앞으로 2.5 mm
    EXPECT_NEAR(d.cyl_d.norm(), 0.0025, 1e-4);
    EXPECT_NEAR(g.tip.z(), g0.tip.z(), 0.0015);   // 높이 그대로
    EXPECT_NEAR(g.app.z(), -1.0, 0.005);          // 수직 손
    EXPECT_GT(g.close.dot(g0.close), std::cos(3.0 * M_PI / 180.0));   // 같은 닫힘 · 같은 손가락 쪽
    // 네모 · 태그 중심 관측: 보정 없음 (보정 0 과 같은 해)
    for (int k = 0; k < 2; ++k) {
      auto t2 = tg;
      RegraspObservation o2 = o;
      if (k == 0) {t2.square = true;} else {o2.tagged = true;}
      const auto e = decide_visual_regrasp({o2}, t2, 10.35, 12.0, s.m, s.tail);
      auto t0 = t2;
      t0.cyl_hand_mm = Eigen::Vector2d::Zero();
      const auto e0 = decide_visual_regrasp({o2}, t0, 10.35, 12.0, s.m, s.tail);
      ASSERT_EQ(e.decision, RegraspDecision::USE) << e.why;
      EXPECT_LT(e.cyl_d.norm(), 1e-9);
      for (size_t i = 0; i < 3; ++i) {EXPECT_EQ(e.poses[i].ticks, e0.poses[i].ticks) << k;}
    }
  }
}
