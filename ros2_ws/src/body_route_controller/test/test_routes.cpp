#include "body_route_controller/core.hpp"
#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <set>
using namespace body_route_controller;
namespace {
constexpr double pi = 3.14159265358979323846;
// Prune monotonically as a path handler does. A delayed first-order plant is
// deliberately only a numerical regression, not a calibrated robot model.
void follow(std::vector<Point> path, Pose pose, Pose goal, double seconds)
{
  Core core; Velocity measured, delayed; std::size_t first = 0;
  bool done = false; std::set<std::string> modes;
  for (int step = 0; step < seconds / .05; ++step) {
    double best = 1e9, searched = 0; std::size_t nearest = first;
    for (std::size_t i = first; i < path.size(); ++i) {
      const double d = std::hypot(pose.x - path[i].x, pose.y - path[i].y);
      if (d < best) {best = d; nearest = i;}
      if (i + 1 < path.size()) {searched += std::hypot(path[i+1].x-path[i].x,path[i+1].y-path[i].y);}
      if (searched > .5) {break;}
    }
    first = nearest > 0 ? nearest - 1 : 0;
    std::vector<Point> local(path.begin() + first, path.end());
    const auto r = core.step(pose, measured, local, goal, .05, .025);
    modes.insert(r.mode);
    ASSERT_TRUE(r.command.x == 0.0 || r.command.y == 0.0);
    ASSERT_LE(std::hypot(r.command.x,r.command.y),.025 + 1e-10);
    ASSERT_LE(std::abs(r.command.yaw),.1 + 1e-10);
    if (r.mode == "route_turn" || r.mode == "final_turn") {
      ASSERT_LE(std::hypot(measured.x, measured.y),.003);
      ASSERT_EQ(r.command.x,0); ASSERT_EQ(r.command.y,0);
    }
    if (r.mode == "goal_stop" && std::hypot(measured.x,measured.y)<.003 && std::abs(measured.yaw)<.01) {done = true; break;}
    measured.x += .25*(delayed.x-measured.x); measured.y += .25*(delayed.y-measured.y);
    measured.yaw += .25*(delayed.yaw-measured.yaw); delayed=r.command;
    pose.x += .05*(std::cos(pose.yaw)*measured.x-std::sin(pose.yaw)*measured.y);
    pose.y += .05*(std::sin(pose.yaw)*measured.x+std::cos(pose.yaw)*measured.y);
    pose.yaw=wrap(pose.yaw+.05*measured.yaw);
  }
  EXPECT_TRUE(done) << pose.x << ',' << pose.y << ',' << pose.yaw;
  EXPECT_LE(std::hypot(pose.x-goal.x,pose.y-goal.y),.03);
  EXPECT_LE(std::abs(wrap(pose.yaw-goal.yaw)),.05);
  EXPECT_TRUE(modes.count("forward")); EXPECT_TRUE(modes.count("route_turn"));
}
std::vector<Point> dense(const std::vector<Point>& corners)
{
  std::vector<Point> out;
  for (std::size_t i=0;i+1<corners.size();++i) {
    const Point a=corners[i],b=corners[i+1];
    const int n=std::max(1,int(std::ceil(std::hypot(b.x-a.x,b.y-a.y)/.005)));
    for(int j=0;j<n;++j) {double t=double(j)/n;out.push_back({a.x+t*(b.x-a.x),a.y+t*(b.y-a.y)});}
  }
  out.push_back(corners.back());return out;
}
}
TEST(Routes, RecordedL2WithDelayedMeasuredFeedback)
{
  std::ifstream file(BODY_ROUTE_TEST_DATA "/l2_recorded.path");
  ASSERT_TRUE(file); std::vector<Point> path; double x,y,a; Pose goal;
  while(file>>x>>y>>a) {path.push_back({x,y});goal={x,y,a};}
  ASSERT_EQ(path.size(),553u);
  follow(path,{path.front().x,path.front().y,1.573},goal,360);
}
TEST(Routes, MultiTurnMazeAndFinalShelfHeading)
{
  follow(dense({{0,0},{0,1},{.6,1},{.6,2},{-.2,2},{-.2,2.6}}),
    {0,-.01,pi/2},{-.2,2.6,pi},600);
}
TEST(Routes, ReturnJourneyAndHalfTurn)
{
  follow(dense({{.6,1},{.6,0},{0,0},{0,.8}}),{.6,1,pi/2},{0,.8,0},400);
}
TEST(Routes, NearbyGoalMustNotSkipRemainingLoop)
{
  Core core;
  auto path=dense({{0,0},{1,0},{1,1},{0,1},{0,.01}});
  EXPECT_NE(core.step({}, {}, path, {0,.01,0}, .05,.025).mode,"goal_stop");
}
TEST(Routes, CornerResidualBelowNormalCorrectionThresholdDoesNotStall)
{
  follow(dense({{0,0},{1,0},{1,1}}),{1,-.023,0},{1,1,pi/2},160);
}
