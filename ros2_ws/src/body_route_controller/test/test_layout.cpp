#include "layout_fixture.hpp"
#include <gtest/gtest.h>
using namespace layout_test;

TEST(Layout, ExtractsDualSensorSegmentsAndVisibleEdges) {
  const auto f=survey(nominal());
  for(const auto & scan:f[0].scans) {
    const auto segments=extract(scan);int opening_walls=0;
    for(const auto & s:segments) {if(std::abs(s.a.x-.8)<.01&&std::abs(s.b.x-.8)<.01) {
      ++opening_walls;EXPECT_TRUE(s.edge_a||s.edge_b);
    }}
    EXPECT_EQ(opening_walls,2); // Both sensor origins recover the same two wall faces.
  }
}
TEST(Layout, ShiftedOpeningWithIndependentOdometryError) {
  const auto prior=nominal();auto world=prior;
  for(int i=0;i<2;++i) {world[i].a.x+=.03;world[i].b.x+=.03;world[i].a.y+=.02;world[i].b.y+=.02;}
  const auto frames=survey(world,.001,true);const auto result=estimate(prior,frames,{-.5,0,0});
  ASSERT_TRUE(result.accepted)<<result.reason<<" residual="<<result.max_residual;
  EXPECT_LT(std::hypot(result.poses.back().pose.x+.38,result.poses.back().pose.y),.012);
  EXPECT_LT(std::abs(result.poses.back().pose.yaw),.012);
  ASSERT_TRUE(result.walls[0].verified);ASSERT_TRUE(result.walls[1].verified);
  EXPECT_NEAR(result.walls[0].wall.b.y,world[0].b.y,.012);
  EXPECT_NEAR(result.walls[0].wall.b.x,world[0].b.x,.008);
  const auto updated=updateMap(grid(prior),prior,frames,result);
  EXPECT_GT(updated.cleared,0u);EXPECT_GT(updated.hits,0u);
  const auto path=checkedCrossing(portal(),result,updated.map,footprint());
  EXPECT_TRUE(path.accepted)<<path.reason;EXPECT_GT(path.path.size(),100u);
}
TEST(Layout, CoherentOuterAndInnerWallTranslationDoesNotMoveStart) {
  auto world=nominal();for(auto & w:world) {w.a.x+=.035;w.b.x+=.035;w.a.y-=.025;w.b.y-=.025;}
  const auto result=estimate(nominal(),survey(world,.0005,true),{-.5,0,0});
  ASSERT_TRUE(result.accepted)<<result.reason<<" residual="<<result.max_residual;
  EXPECT_NEAR(result.poses.front().pose.x,-.5,.003);EXPECT_NEAR(result.poses.front().pose.y,0,.003);
  EXPECT_NEAR(result.poses.back().pose.x,-.38,.012);EXPECT_NEAR(result.poses.back().pose.y,0,.012);
  EXPECT_TRUE(result.walls[3].line_verified); // A wall face can be known without a visible end.
  const auto updated=updateMap(grid(nominal()),nominal(),survey(world,.0005,true),result);
  EXPECT_EQ(updated.map.cells[updated.map.index({2,0})],0); // Observed vacated east boundary.
}
TEST(Layout, ParallelWallsReportUnobservedDirection) {
  const std::vector<Wall> walls{{"west",{-1,-10},{-1,10}},{"east",{1,-10},{1,10}}};
  const auto result=estimate(walls,survey(walls),{-.5,0,0});
  EXPECT_FALSE(result.accepted);EXPECT_EQ(result.reason,"pose_geometry_unobservable_or_uncertain");
}
TEST(Layout, DoesNotInventEdgesAtMissingOrMaxRangeReturns) {
  auto scan=survey(nominal())[0].scans[0];
  for(auto & r:scan.ranges) {if(r>1) {r=std::numeric_limits<double>::infinity();}}
  for(const auto & s:extract(scan)) {EXPECT_FALSE(s.edge_a);EXPECT_FALSE(s.edge_b);}
}
TEST(Layout, RejectsMovingUnsynchronizedOrDuplicateSensorInput) {
  auto frames=survey(nominal());frames[1].measured.x=.01;
  EXPECT_THROW(estimate(nominal(),frames,{-.5,0,0}),std::invalid_argument);
  frames=survey(nominal());frames[1].scans[0].stamp+=.1;
  EXPECT_THROW(estimate(nominal(),frames,{-.5,0,0}),std::invalid_argument);
  frames=survey(nominal());frames[0].scans[1].sensor_id="front";
  EXPECT_THROW(estimate(nominal(),frames,{-.5,0,0}),std::invalid_argument);
}
TEST(Layout, RejectsLargeWallShiftAndDoesNotPatchRejectedEstimate) {
  auto world=nominal();for(int i=0;i<2;++i) {world[i].a.x+=.09;world[i].b.x+=.09;}
  const auto frames=survey(world);const auto e=estimate(nominal(),frames,{-.5,0,0});
  EXPECT_FALSE(e.accepted);EXPECT_THROW(updateMap(grid(nominal()),nominal(),frames,e),std::invalid_argument);
}
TEST(Layout, DuplicateNominalWallIsAmbiguous) {
  auto walls=nominal();walls.push_back(walls[0]);walls.back().id="duplicate";
  const auto result=estimate(walls,survey(nominal()),{-.5,0,0});
  EXPECT_FALSE(result.accepted);EXPECT_GT(result.ambiguous,0u);
}
TEST(Layout, SensorDisagreementDoesNotBecomeConfidentWallMovement) {
  auto frames=survey(nominal());for(auto & f:frames) {for(auto & r:f.scans[1].ranges) {if(std::isfinite(r)) {r+=.06;}}}
  const auto result=estimate(nominal(),frames,{-.5,0,0});EXPECT_FALSE(result.accepted);
}
TEST(Layout, RepeatedStationaryViewsDoNotAuthorizeOpening) {
  auto frames=survey(nominal());for(std::size_t i=1;i<frames.size();++i) {
    const double stamp=frames[i].stamp;frames[i]=frames[0];frames[i].stamp=stamp;
    for(auto & s:frames[i].scans) {s.stamp=stamp;}
  }
  const auto result=estimate(nominal(),frames,{-.5,0,0});ASSERT_TRUE(result.accepted)<<result.reason;
  EXPECT_FALSE(result.walls[0].verified);EXPECT_FALSE(crossing(portal(),result,.36).accepted);
}
TEST(Layout, UnknownApproachAndNewObstacleBlockPath) {
  const auto result=estimate(nominal(),survey(nominal()),{-.5,0,0});ASSERT_TRUE(result.accepted)<<result.reason;
  auto map=grid(nominal());const int cell=map.index({.3,0});map.cells[cell]=-1;
  EXPECT_FALSE(checkedCrossing(portal(),result,map,footprint()).accepted);
  map.cells[cell]=100;EXPECT_FALSE(checkedCrossing(portal(),result,map,footprint()).accepted);
}
TEST(Layout, NarrowGapRejectsFullFootprint) {
  auto walls=nominal();walls[0].b.y=-.16;walls[1].a.y=.16;
  const auto result=estimate(walls,survey(walls),{-.5,0,0});ASSERT_TRUE(result.accepted)<<result.reason;
  const auto path=checkedCrossing(portal(),result,grid(walls),footprint());
  EXPECT_FALSE(path.accepted);EXPECT_EQ(path.reason,"opening_too_narrow_or_uncertain");
}
TEST(Layout, InvalidNumbersAndGridAreRejected) {
  auto frames=survey(nominal());frames[0].odom.yaw=std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(estimate(nominal(),frames,{-.5,0,0}),std::invalid_argument);
  auto g=grid(nominal());g.cells.pop_back();EXPECT_THROW(g.validate(),std::invalid_argument);
  body_route_controller::layout::Settings cfg;cfg.max_wall_shift=.1;EXPECT_THROW(cfg.validate(),std::invalid_argument);
}
TEST(Layout, RotatedWallSegmentsWithinEndpointDisplacementBound) {
  auto world=nominal();for(int i=0;i<2;++i) {
    const Point center{(world[i].a.x+world[i].b.x)/2,(world[i].a.y+world[i].b.y)/2};
    for(Point *p:{&world[i].a,&world[i].b}) {
      const Point q=transform({center.x+.01,center.y,.06},{p->x-center.x,p->y-center.y});*p=q;
    }
  }
  const auto result=estimate(nominal(),survey(world,.001,true),{-.5,0,0});
  ASSERT_TRUE(result.accepted)<<result.reason;
  EXPECT_NEAR(result.walls[0].wall.b.x,world[0].b.x,.01);
}
TEST(Layout, UnrelatedOccupiedCellsAreNeverCleared) {
  auto world=nominal();for(int i=0;i<2;++i) {world[i].a.x+=.04;world[i].b.x+=.04;}
  const auto frames=survey(world);const auto result=estimate(nominal(),frames,{-.5,0,0});ASSERT_TRUE(result.accepted)<<result.reason;
  auto prior=grid(nominal());const int unrelated=prior.index({0,0});prior.cells[unrelated]=100;
  const auto update=updateMap(prior,nominal(),frames,result);
  EXPECT_EQ(update.map.cells[unrelated],100);EXPECT_GT(update.cleared,0u);
}
TEST(Layout, DisconnectedNewWallGroupDoesNotInheritOdometryConfidence) {
  auto walls=nominal();auto second=nominal();for(auto & w:second) {w.id+="_new";w.a.x+=7;w.b.x+=7;walls.push_back(w);}
  auto frames=survey(walls);for(std::size_t i=1;i<frames.size();++i) {
    frames[i].odom.x+=7;frames[i].scans={scan(walls,frames[i].odom,{.08,0,0},frames[i].stamp,0),scan(walls,frames[i].odom,{-.08,0,pi},frames[i].stamp,1)};
  }
  const auto result=estimate(walls,frames,{-.5,0,0});EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.reason,"pose_geometry_unobservable_or_uncertain");
}
TEST(Layout, RequiredFiveCentimetreOffsets) {
  for(Point shift:{Point{.05,0},Point{-.05,0},Point{0,.05},Point{0,-.05},Point{.03,.04}}) {
    auto world=nominal();for(int i=0;i<2;++i) {world[i].a.x+=shift.x;world[i].b.x+=shift.x;world[i].a.y+=shift.y;world[i].b.y+=shift.y;}
    const auto result=estimate(nominal(),survey(world),{-.5,0,0});
    EXPECT_TRUE(result.accepted)<<shift.x<<','<<shift.y<<' '<<result.reason;
  }
}
TEST(Layout, SharedAssemblyPreservesRelativeGeometry) {
  auto walls=nominal();walls[0].assembly="joined_pair";walls[1].assembly="joined_pair";auto world=walls;
  for(int i=0;i<2;++i) {world[i].a.x+=.03;world[i].b.x+=.03;world[i].a.y+=.02;world[i].b.y+=.02;}
  const auto result=estimate(walls,survey(world,.001,true),{-.5,0,0});ASSERT_TRUE(result.accepted)<<result.reason;
  EXPECT_EQ(result.variables,27);
  EXPECT_NEAR(std::hypot(result.walls[0].wall.b.x-result.walls[1].wall.a.x,result.walls[0].wall.b.y-result.walls[1].wall.a.y),.7,1e-9);
}
TEST(Layout, FullTurnSweepCanRejectAnOtherwiseClearApproach) {
  auto result=estimate(nominal(),survey(nominal()),{-.5,0,0});ASSERT_TRUE(result.accepted)<<result.reason;
  auto map=grid(nominal());auto & pose=result.poses.back().pose;pose.yaw=pi/2;
  map.cells[map.index({pose.x-.22,pose.y})]=100; // Behind the robot; a rotating corner reaches it.
  const auto path=checkedCrossing(portal(),result,map,footprint());
  EXPECT_FALSE(path.accepted);EXPECT_EQ(path.reason,"turn_sweep_blocked_or_unknown");
}
TEST(Layout, NewUnmatchedObstaclePrevailsOverEarlierFreeRays) {
  auto frames=survey(nominal());auto world=nominal();world.push_back({"new_obstacle",{.3,-.1},{.3,.1}});
  auto & last=frames.back();last.scans={scan(world,last.odom,{.08,0,0},last.stamp,0),scan(world,last.odom,{-.08,0,pi},last.stamp,1)};
  const auto result=estimate(nominal(),frames,{-.5,0,0});ASSERT_TRUE(result.accepted)<<result.reason;
  EXPECT_GT(result.unmatched,0u);const auto updated=updateMap(grid(nominal()),nominal(),frames,result);
  EXPECT_FALSE(checkedCrossing(portal(),result,updated.map,footprint()).accepted);
}
TEST(Layout, MissingRearScanCannotAuthorizeClearing) {
  auto frames=survey(nominal());for(auto & r:frames[2].scans[1].ranges) {r=std::numeric_limits<double>::quiet_NaN();}
  EXPECT_THROW(estimate(nominal(),frames,{-.5,0,0}),std::invalid_argument);
}
TEST(Layout, UncertainStartCannotBecomeConfidentThroughWallPriors) {
  body_route_controller::layout::Settings settings;settings.start_sigma=.04;settings.start_yaw_sigma=.06;
  const auto result=estimate(nominal(),survey(nominal()),{-.5,0,0},settings);
  EXPECT_FALSE(result.accepted);EXPECT_EQ(result.reason,"pose_geometry_unobservable_or_uncertain");
}
