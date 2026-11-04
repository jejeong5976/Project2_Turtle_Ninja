#include "body_route_controller/supervision.hpp"
#include <gtest/gtest.h>
#include <cmath>
using namespace body_route_controller;
namespace {const std::vector<Point> path{{0,0},{0,1},{1,1}};}
TEST(Supervision, TurnsAreAllowedAtAnyInitialHeading)
{
  TrackingGuard g;
  for(double yaw=0;yaw<3.14;yaw+=.05) {EXPECT_TRUE(g.check({},{0,0,yaw},{0,0,.1},path,"route_turn",3.14).empty());}
  EXPECT_TRUE(g.check({0,1,0},{0,1,0},{.02,0,0},path,"forward",0).empty());
}
TEST(Supervision, TurnDriftAndTranslatingHeadingAreStillBounded)
{
  TrackingGuard g;
  EXPECT_TRUE(g.check({}, {}, {}, path,"route_turn",1.57).empty());
  EXPECT_EQ(g.check({}, {.03,0,0}, {}, path,"route_turn",1.57),"translation_during_turn");
  g.reset();
  EXPECT_EQ(g.check({}, {}, {.02,0,0}, path,"route_turn",1.57),"turn_before_stopped");
  EXPECT_EQ(g.check({}, {}, {.02,0,0}, path,"forward",1.57),"translation_heading_error");
}
TEST(Supervision, CorridorSpeedInvalidStateAndBlockedModeStop)
{
  TrackingGuard g;
  EXPECT_EQ(g.check({.2,0,0},{},{},path,"forward",0),"outside_route_corridor");
  EXPECT_EQ(g.check({}, {}, {.041,0,0},path,"forward",0),"measured_speed_limit");
  EXPECT_EQ(g.check({}, {}, {0,0,.151},path,"route_turn",0),"measured_speed_limit");
  EXPECT_EQ(g.check({}, {}, {},path,"blocked",0),"unexpected_controller_mode:blocked");
  EXPECT_EQ(g.check({}, {}, {},{},"forward",0),"missing_route");
  EXPECT_EQ(g.check({NAN,0,0}, {}, {},path,"forward",0),"non_finite_tracking_state");
}
