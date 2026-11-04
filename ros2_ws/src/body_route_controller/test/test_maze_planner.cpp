#include "body_route_controller/maze_planner.hpp"
#include <nav2_core/planner_exceptions.hpp>
#include <gtest/gtest.h>
#include <cmath>
using namespace body_route_controller;
namespace {
constexpr double pi=3.14159265358979323846;
Footprint body(){Footprint out;for(auto q:std::vector<Point>{{-.205,-.205},{.205,-.205},{.205,.205},{-.205,.205}}){geometry_msgs::msg::Point p;p.x=q.x;p.y=q.y;out.push_back(p);}return out;}
nav2_costmap_2d::Costmap2D gate(){nav2_costmap_2d::Costmap2D map(240,240,.01,-1.2,-1.2,0);
  for(unsigned y=0;y<240;++y){double x,w;map.mapToWorld(120,y,x,w);if(w<-.12||w>.32)for(unsigned c=119;c<=121;++c)map.setCost(c,y,254);}return map;}
}
TEST(MazePlanner, CrossingUsesOneCornerOutsideNarrowOpening)
{
  auto map=gate();auto f=body();auto path=planMaze(map,f,{-.7,-.7,pi/2},{.7,.1,0});
  ASSERT_GT(path.size(),2u);int corners=0;
  for(std::size_t i=0;i+1<path.size();++i){auto a=path[i],b=path[i+1];double heading=std::atan2(b.y-a.y,b.x-a.x),length=std::hypot(b.x-a.x,b.y-a.y);
    EXPECT_TRUE(std::abs(b.x-a.x)<1e-8||std::abs(b.y-a.y)<1e-8);
    EXPECT_TRUE(motionSafe(map,f,{a.x,a.y,heading},{.1,0,0},length/.1));
    if(i&&std::abs(wrap(heading-path[i-1].yaw))>.1){++corners;EXPECT_TRUE(turnSafe(map,f,{a.x,a.y,path[i-1].yaw},heading));}}
  EXPECT_EQ(corners,1);EXPECT_NEAR(path.back().x,.7,1e-12);EXPECT_NEAR(path.back().y,.1,1e-12);
}
TEST(MazePlanner, UnknownBarrierCannotBeCrossed)
{
  auto map=gate();for(unsigned y=0;y<240;++y)map.setCost(120,y,255);
  EXPECT_THROW(planMaze(map,body(),{-.7,-.7,pi/2},{.7,.1,0}),nav2_core::NoValidPathCouldBeFound);
}
TEST(MazePlanner, RejectsOccupiedGoalsBoundsAndCancellation)
{
  auto map=gate();
  EXPECT_THROW(planMaze(map,body(),{-.7,-.7,pi/2},{0,.8,0}),nav2_core::GoalOccupied);
  EXPECT_THROW(planMaze(map,body(),{-.7,-.7,pi/2},{3,0,0}),nav2_core::GoalOutsideMapBounds);
  EXPECT_THROW(planMaze(map,body(),{-.7,-.7,pi/2},{.7,.1,0},{},[]{return true;}),nav2_core::PlannerCancelled);
}
TEST(MazePlanner, CanDriveStraightThroughOpeningButCannotRotateThere)
{
  auto map=gate();auto f=body();
  EXPECT_FALSE(turnSafe(map,f,{0,.1,0},pi/2));
  auto path=planMaze(map,f,{0,.1,0},{.7,.1,0});
  for(auto p:path){EXPECT_NEAR(p.y,.1,1e-12);EXPECT_NEAR(p.yaw,0,1e-12);}
}

TEST(MazePlanner, RetainsOutAndBackTurnWhenGoalCannotRotateInPlace)
{
  nav2_costmap_2d::Costmap2D map(300,300,.01,-1.5,-1.5,0);
  for(unsigned x=120;x<=220;++x){map.setCost(x,125,254);map.setCost(x,175,254);}
  const auto f=body();EXPECT_FALSE(turnSafe(map,f,{.3,0,0},pi));
  auto path=planMaze(map,f,{0,0,0},{.3,0,pi});double furthest=0;
  for(auto p:path)furthest=std::max(furthest,p.x);
  EXPECT_GT(furthest,.71);  // Must pass the wall end before returning.
  EXPECT_NEAR(path.back().x,.3,1e-12);EXPECT_NEAR(path.back().yaw,pi,1e-12);
  for(std::size_t i=1;i+1<path.size();++i){auto a=path[i-1],b=path[i];if(std::abs(wrap(b.yaw-a.yaw))>.1){EXPECT_TRUE(turnSafe(map,f,{b.x,b.y,a.yaw},b.yaw));}}
}
