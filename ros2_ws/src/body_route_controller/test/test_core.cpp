#include "body_route_controller/core.hpp"
#include <gtest/gtest.h>
#include <cmath>
#include <limits>

using namespace body_route_controller;
namespace
{
constexpr double pi = 3.14159265358979323846;
const std::vector<Point> straight{{0, 0}, {1, 0}};
const Pose goal{1, 0, 0};
}

TEST(BodyRoute, RejectsInvalidConfiguration)
{
  Settings s; s.align_exit = s.align_enter;
  EXPECT_THROW(Core{s}, std::invalid_argument);
  s = Settings{}; s.max_speed = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(Core{s}, std::invalid_argument);
}
TEST(BodyRoute, FacesBodyXAxisWithoutCameraOffset)
{
  Core c;
  auto r = c.step({0, 0, 0}, {}, straight, goal, .05, .025);
  EXPECT_EQ(r.mode, "aligned"); EXPECT_DOUBLE_EQ(r.heading, 0);
  r = c.step({0, 0, 0}, {}, straight, goal, .05, .025);
  EXPECT_GT(r.command.x, 0); EXPECT_DOUBLE_EQ(r.command.y, 0);
  EXPECT_DOUBLE_EQ(r.command.yaw, 0);
}
TEST(BodyRoute, StopsBeforeLargeTurnAndSettlesAfterward)
{
  Core c;
  auto r = c.step({0, 0, pi / 2}, {.02, 0, 0}, straight, goal, .05, .025);
  EXPECT_EQ(r.mode, "stop_before_turn"); EXPECT_DOUBLE_EQ(r.command.yaw, 0);
  r = c.step({0, 0, pi / 2}, {}, straight, goal, .05, .025);
  EXPECT_EQ(r.mode, "route_turn"); EXPECT_LT(r.command.yaw, 0);
  EXPECT_DOUBLE_EQ(r.command.x, 0); EXPECT_DOUBLE_EQ(r.command.y, 0);
  r = c.step({0, 0, .01}, {0, 0, -.03}, straight, goal, .05, .025);
  EXPECT_EQ(r.mode, "settle_turn"); EXPECT_DOUBLE_EQ(r.command.x, 0);
}
TEST(BodyRoute, SmallYawCorrectionCanAccompanyTranslation)
{
  Core c;
  c.step({0, 0, 0}, {}, straight, goal, .05, .025);
  auto r = c.step({0, 0, .06}, {}, straight, goal, .05, .025);
  EXPECT_EQ(r.mode, "forward"); EXPECT_LT(r.command.yaw, 0);
  EXPECT_LE(std::abs(r.command.yaw), .4 * std::abs(r.command.x) + 1e-12);
  r = c.step({0, 0, .2}, {.01, 0, 0}, straight, goal, .05, .025);
  EXPECT_EQ(r.mode, "stop_before_turn");
}
TEST(BodyRoute, SideCorrectionKeepsRouteHeading)
{
  Core c;
  c.step({.1, .06, 0}, {}, straight, goal, .05, .025);
  auto r = c.step({.1, .06, 0}, {}, straight, goal, .05, .025);
  EXPECT_EQ(r.mode, "right"); EXPECT_DOUBLE_EQ(r.heading, 0);
  EXPECT_DOUBLE_EQ(r.command.x, 0); EXPECT_LT(r.command.y, 0);
  r = c.step({.1, .005, 0}, {0, -.01, 0}, straight, goal, .05, .025);
  EXPECT_EQ(r.mode, "stop_before_axis_change");
}
TEST(BodyRoute, CorrectsSmallOvershootBackwardWithoutTurningAround)
{
  Core c;
  c.step({1.06, 0, 0}, {}, straight, goal, .05, .025);
  const auto r = c.step({1.06, 0, 0}, {}, straight, goal, .05, .025);
  EXPECT_EQ(r.mode, "backward"); EXPECT_LT(r.command.x, 0);
  EXPECT_DOUBLE_EQ(r.heading, 0);
}
TEST(BodyRoute, FinalGoalHeadingOverridesRouteHeading)
{
  Core c;
  const auto r = c.step({1, 0, 0}, {}, straight, {1, 0, pi / 2}, .05, .025);
  EXPECT_EQ(r.mode, "final_turn"); EXPECT_DOUBLE_EQ(r.command.x, 0);
  EXPECT_GT(r.command.yaw, 0);
  EXPECT_EQ(c.step({1, 0, pi / 2}, {}, straight, {1, 0, pi / 2}, .05, .025).mode, "goal_stop");
}
TEST(BodyRoute, SinglePointSupportsFinalTurnButCannotInventRouteHeading)
{
  Core c;
  EXPECT_EQ(c.step({1, 0, 0}, {}, {{1, 0}}, {1, 0, pi / 2}, .05, .025).mode, "final_turn");
  EXPECT_THROW(c.step({1.06, 0, 0}, {}, {{1, 0}}, goal, .05, .025), std::runtime_error);
}
TEST(BodyRoute, DoesNotTurnBeforeSharpCorner)
{
  const std::vector<Point> path{{0, 0}, {1, 0}, {1, 1}};
  Settings s;
  auto target = routeTarget(path, {.94, 0, 0}, s);
  EXPECT_DOUBLE_EQ(target.heading, 0); EXPECT_NEAR(target.point.x, 1, 1e-12);
  EXPECT_DOUBLE_EQ(target.point.y, 0);
  target = routeTarget(path, {1, 0, 0}, s);
  EXPECT_NEAR(target.heading, pi / 2, 1e-12);
}
TEST(BodyRoute, ClosedLoopCornerAndShelfArrival)
{
  Core c;
  const std::vector<Point> path{{0, 0}, {.35, 0}, {.35, .35}};
  const Pose finish{.35, .35, pi};
  Pose pose{0, -.04, .5}; Velocity measured;
  bool side = false, turn = false, complete = false;
  for (int i = 0; i < 5000; ++i) {
    const auto r = c.step(pose, measured, path, finish, .05, .025);
    ASSERT_TRUE(r.command.x == 0.0 || r.command.y == 0.0);
    if (r.mode == "route_turn" || r.mode == "final_turn") {
      EXPECT_EQ(r.command.x, 0); EXPECT_EQ(r.command.y, 0); turn = true;
    }
    side |= r.mode == "left" || r.mode == "right";
    if (r.mode == "goal_stop" && std::hypot(measured.x, measured.y) < .003 && std::abs(measured.yaw) < .01) {
      complete = true; break;
    }
    measured = r.command;  // Deliberately an ideal plant, not Isaac/physical validation.
    pose.x += .05 * (std::cos(pose.yaw) * measured.x - std::sin(pose.yaw) * measured.y);
    pose.y += .05 * (std::sin(pose.yaw) * measured.x + std::cos(pose.yaw) * measured.y);
    pose.yaw = wrap(pose.yaw + .05 * measured.yaw);
  }
  EXPECT_TRUE(complete); EXPECT_TRUE(side); EXPECT_TRUE(turn);
  EXPECT_LE(std::hypot(pose.x - finish.x, pose.y - finish.y), .03);
  EXPECT_LE(std::abs(wrap(pose.yaw - finish.yaw)), .05);
}
TEST(BodyRoute, SpeedLimitAndClockGap)
{
  Core c;
  c.step({}, {}, straight, goal, .05, .025);
  for (int i = 0; i < 10; ++i) {
    EXPECT_LE(c.step({}, {}, straight, goal, .05, .004).command.x, .004);
  }
  EXPECT_EQ(c.step({}, {}, straight, goal, .5, .025).mode, "clock_gap");
  EXPECT_EQ(c.step({}, {}, straight, goal, .05, 0).mode, "speed_limit_stop");
}
TEST(BodyRoute, RejectsBadAndDistantRoutes)
{
  Core c;
  EXPECT_THROW(c.step({}, {}, {}, goal, .05, .025), std::invalid_argument);
  EXPECT_THROW(c.step({}, {}, {{NAN, 0}}, goal, .05, .025), std::invalid_argument);
  EXPECT_THROW(c.step({0, .5, 0}, {}, straight, goal, .05, .025), std::runtime_error);
  EXPECT_THROW(c.step({}, {NAN, 0, 0}, straight, goal, .05, .025), std::invalid_argument);
}
TEST(BodyRoute, RepeatedPointsAndWrappedHeadings)
{
  const auto t = routeTarget({{0, 0}, {0, 0}, {-1, 0}}, {}, Settings{});
  EXPECT_NEAR(std::abs(t.heading), pi, 1e-12);
  EXPECT_NEAR(wrap(-pi - .01), pi - .01, 1e-12);
}
TEST(BodyRoute, CorrectionTimeout)
{
  Settings s; s.max_correction_time = .1; Core c(s);
  c.step({0, .05, 0}, {}, straight, goal, .05, .025);
  c.step({0, .05, 0}, {}, straight, goal, .05, .025);
  c.step({0, .05, 0}, {}, straight, goal, .05, .025);
  EXPECT_THROW(c.step({0, .05, 0}, {}, straight, goal, .05, .025), std::runtime_error);
}
TEST(BodyRoute, FreshnessChecksReceiptHeaderAndClockDirection)
{
  EXPECT_TRUE(fresh(10, 9.9, 9.9, .3));
  EXPECT_FALSE(fresh(10, 9, 9.9, .3));
  EXPECT_FALSE(fresh(10, 9.9, 9, .3));
  EXPECT_FALSE(fresh(10, 9.9, 10.1, .3));
  EXPECT_FALSE(fresh(10, 11, 10, .3));
  EXPECT_FALSE(fresh(10, NAN, 10, .3));
}

TEST(BodyRoute, OverlappingReturnLegDoesNotCausePrematureTurn)
{
  const std::vector<Point> path{{0,0},{1,0},{.3,0}};Settings cfg;
  EXPECT_NEAR(routeTarget(path,{.4,0,0},cfg).heading,0,1e-12);
  EXPECT_NEAR(std::abs(routeTarget(path,{1,0,0},cfg).heading),pi,1e-12);
}

TEST(BodyRoute, TightMazeTrackingBrakesBeforeCorrectingSmallLateralDrift)
{
  Settings cfg; cfg.max_speed = .08; cfg.cross_track_enter = .01; cfg.cross_track_exit = .004;
  Core c(cfg);
  EXPECT_EQ(c.step({}, {}, straight, goal, .05, .08).mode, "aligned");
  EXPECT_EQ(c.step({}, {}, straight, goal, .05, .08).mode, "forward");
  // An 18 mm offset is below the old 25 mm threshold, but consumes the
  // remaining clearance after local-map rasterization in the shifted L2 gap.
  auto r = c.step({.2, -.018, 0}, {.08, 0, 0}, straight, goal, .05, .08);
  EXPECT_EQ(r.mode, "stop_before_axis_change");
  EXPECT_DOUBLE_EQ(r.command.x, 0); EXPECT_DOUBLE_EQ(r.command.y, 0);
  r = c.step({.2, -.018, 0}, {}, straight, goal, .05, .08);
  EXPECT_EQ(r.mode, "left"); EXPECT_DOUBLE_EQ(r.command.x, 0); EXPECT_GT(r.command.y, 0);
  r = c.step({.2, -.003, 0}, {}, straight, goal, .05, .08);
  EXPECT_EQ(r.mode, "forward"); EXPECT_GT(r.command.x, 0); EXPECT_DOUBLE_EQ(r.command.y, 0);
}

TEST(BodyRoute, CornerCommitmentSurvivesLocalizationJitterButReleasesAway)
{
  const std::vector<Point> path{{0, 0}, {1, 0}, {1, 1}};
  Settings cfg; RouteProgress progress;
  EXPECT_NEAR(routeTarget(path, {.97, 0, 0}, cfg, &progress).heading, 0, 1e-12);
  EXPECT_FALSE(progress.committed);  // Do not use the exit tolerance to turn early.
  EXPECT_NEAR(routeTarget(path, {.985, 0, 0}, cfg, &progress).heading, pi / 2, 1e-12);
  EXPECT_TRUE(progress.committed);
  EXPECT_NEAR(routeTarget(path, {.978, .002, .4}, cfg, &progress).heading, pi / 2, 1e-12);
  EXPECT_NEAR(routeTarget(path, {.90, 0, .4}, cfg, &progress).heading, 0, 1e-12);
  EXPECT_FALSE(progress.committed);
}
TEST(BodyRoute, ChangedPathCannotReuseCommittedCorner)
{
  Settings cfg; RouteProgress progress;
  routeTarget({{0, 0}, {1, 0}, {1, 1}}, {.985, 0, 0}, cfg, &progress);
  const auto target = routeTarget({{0, 0}, {1, 0}, {1, -1}}, {.978, 0, .4}, cfg, &progress);
  EXPECT_NEAR(target.heading, 0, 1e-12); EXPECT_FALSE(progress.committed);
}
TEST(BodyRoute, CoreDoesNotReverseTurnAfterSmallPoseUpdateAndResetClearsCommitment)
{
  Core c; const std::vector<Point> path{{0, 0}, {1, 0}, {1, 1}};
  const Pose finish{1, 1, pi / 2};
  auto r = c.step({.985, 0, 0}, {}, path, finish, .05, .025);
  ASSERT_EQ(r.mode, "route_turn"); EXPECT_GT(r.command.yaw, 0);
  r = c.step({.978, 0, .4}, {}, path, finish, .05, .025);
  EXPECT_NEAR(r.heading, pi / 2, 1e-12); EXPECT_GT(r.command.yaw, 0);
  c.reset();
  r = c.step({.978, 0, .4}, {}, path, finish, .05, .025);
  EXPECT_NEAR(r.heading, 0, 1e-12); EXPECT_LT(r.command.yaw, 0);
}

TEST(BodyRoute, ArrivalTargetReservesMarginForSmallLocalizationHeadingBias)
{
  auto arrive = [](double yaw_tolerance) {
      Settings cfg; cfg.goal_xy = .02; cfg.goal_yaw = yaw_tolerance; Core c(cfg);
      Pose truth{1.01, 0, pi / 2 - .061}; Velocity measured;
      for (int i = 0; i < 500; ++i) {
        Pose estimated = truth; estimated.yaw += .012;
        auto r = c.step(estimated, measured, straight, {1, 0, pi / 2}, .05, .025);
        if (r.mode == "goal_stop" && std::abs(measured.yaw) < .01) {return std::abs(truth.yaw - pi / 2);}
        measured = r.command; truth.yaw = wrap(truth.yaw + .05 * measured.yaw);
      }
      return pi;
    };
  EXPECT_GT(arrive(.05), .055);  // Estimated arrival alone consumed the truth margin.
  EXPECT_LT(arrive(.03), .055);  // The acceptance threshold itself is unchanged.
}
