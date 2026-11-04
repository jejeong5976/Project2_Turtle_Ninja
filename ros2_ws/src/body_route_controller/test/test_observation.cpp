#include "unknown_fixture.hpp"
#include <nav2_core/planner_exceptions.hpp>
#include <gtest/gtest.h>
using namespace unknown_fixture;
TEST(Unknown, PolicyAndPositivePenaltyValidation){
  EXPECT_EQ(unknownPolicy("blocked"),UnknownPolicy::Blocked);EXPECT_EQ(unknownPolicy("observe-before-enter"),UnknownPolicy::ObserveBeforeEnter);
  EXPECT_THROW(unknownPolicy("free"),std::invalid_argument);
  auto raw=world();ObservationSpace s;s.setMap(raw);auto map=s.rawMap();MazePlannerSettings cfg;cfg.unknown_penalty=0;
  EXPECT_THROW(planMaze(map,body(),{.6,1.4,0},{1,1.4,0},cfg),nav2_core::PlannerException);
}
TEST(Unknown, PlanningAndGoalRotationAreProvisionalIdentitySurvivesInflation){
  auto nav=partial(world());ObservationSpace s;s.setMap(nav);auto raw=s.rawMap(),cost=raw;
  EXPECT_THROW(planMaze(cost,body(),{.6,1.4,0},{2,1.4,pi/2}),nav2_core::GoalOccupied);
  // Simulate inflation's conversion, then prove raw identity still controls the evidence gate.
  for(unsigned i=0;i<cost.getSizeInCellsX()*cost.getSizeInCellsY();++i)if(cost.getCharMap()[i]==255)cost.getCharMap()[i]=253;
  MazePlannerSettings cfg;cfg.unknown_policy=UnknownPolicy::ObserveBeforeEnter;
  EXPECT_NO_THROW(planMaze(cost,body(),{.6,1.4,0},{2,1.4,pi/2},cfg,[]{return false;},&raw));
  EXPECT_EQ(s.identity(2,1.4),255);auto exec=s.executionMap(10,10);EXPECT_FALSE(footprintSafe(exec,body(),{2,1.4,pi/2}));
  raw.setCost(100,70,254);EXPECT_THROW(planMaze(cost,body(),{.6,1.4,0},{2,1.4,0},cfg,[]{return false;},&raw),nav2_core::GoalOccupied);
}
TEST(Unknown, KnownRoutePreferenceUsesRawIdentity){
  auto nav=world();obstacle(nav,1.8,1.1,2.1,1.7);
  for(unsigned y=88;y<120;++y)for(unsigned x=40;x<180;++x)nav.data[y*nav.info.width+x]=-1;
  ObservationSpace s;s.setMap(nav);auto raw=s.rawMap();MazePlannerSettings cfg;cfg.unknown_policy=UnknownPolicy::ObserveBeforeEnter;cfg.unknown_penalty=5;
  auto path=planMaze(raw,body(),{.6,1.4,0},{3.5,1.4,0},cfg,[]{return false;},&raw);
  for(auto p:path)EXPECT_LT(p.y,1.8);
}
TEST(Unknown, MissingStaleInvalidAndMissingTransformFailClosed){
  auto truth=world();ObservationSpace s;s.setMap(partial(truth));Pose p{.8,1.4,0};
  EXPECT_NE(s.health(10),"");observe(s,truth,p,10);EXPECT_EQ(s.health(10.01),"");
  EXPECT_NE(s.health(10.31),"");s.invalidate(0,"missing_scan_transform");EXPECT_NE(s.health(10.02).find("missing_scan_transform"),std::string::npos);
  auto rays=scan(truth,p,0,11);rays[1].range=NAN;EXPECT_FALSE(s.observe(0,rays,11,11.01));
  EXPECT_FALSE(s.observe(1,{},11,11.01));EXPECT_FALSE(s.observe(0,scan(truth,p,0,12),12,11.9));
}
TEST(Unknown, InitialFootprintIsNotBlanketCleared){
  auto truth=world();ObservationSpace s;s.setMap(partial(truth));
  EXPECT_TRUE(s.certify(body(),{.8,1.4,0},10));EXPECT_FALSE(s.certify(body(),{2,1.4,0},10.05));
  observe(s,truth,{.8,1.4,0},10.1);auto map=s.executionMap(10.11,10.11);EXPECT_EQ(s.identity(3,1.4),255);EXPECT_FALSE(footprintSafe(map,body(),{3,1.4,0}));
}
TEST(Unknown, RaysDoNotClearStaticObstaclesOrOcclusion){
  auto truth=world();obstacle(truth,1.7,.9,1.76,1.9);ObservationSpace s;s.setMap(partial(truth));observe(s,truth,{1,1.4,0},10,2);
  auto map=s.executionMap(10.01,10.01);unsigned x,y;map.worldToMap(1.72,1.4,x,y);EXPECT_EQ(map.getCost(x,y),254);
  map.worldToMap(2,1.4,x,y);EXPECT_EQ(map.getCost(x,y),255);
  auto nav=world();obstacle(nav,1.2,1.2,1.3,1.6);s.setMap(nav);observe(s,world(),{.8,1.4,0},11,2);
  map=s.executionMap(11.01,11.01);map.worldToMap(1.25,1.4,x,y);EXPECT_EQ(map.getCost(x,y),254);
}
TEST(Unknown, ExpiryAndBrakingHorizonIncludeSelectedSpeed){
  auto truth=world();ObservationSpace s;s.setMap(partial(truth));Pose p{1.02,1.4,0};observe(s,truth,p,10,.5);
  ASSERT_TRUE(s.certify(body(),p,10.01));
  EXPECT_EQ(s.check(body(),p,{}, {.025,0,0},10.01),"");
  EXPECT_NE(s.check(body(),p,{}, {.26,0,0},10.01),"");
  auto expired=s.executionMap(15,15);unsigned x,y;expired.worldToMap(1.4,1.4,x,y);EXPECT_EQ(expired.getCost(x,y),255);
}
TEST(Unknown, TurnSweepNeedsObservedCorners){
  auto truth=world();auto nav=partial(truth);ObservationSpace s;s.setMap(nav);Pose p{.98,1.4,0};
  observe(s,truth,p,10,.15);ASSERT_TRUE(s.certify(body(),p,10.01));
  EXPECT_NE(s.check(body(),p,{}, {0,0,.3},10.01),"");
  observe(s,truth,p,10.1,.7);EXPECT_EQ(s.check(body(),p,{}, {0,0,.3},10.11),"");
}
TEST(Unknown, ProgressiveClearCorridorDoesNotNeedWholeRouteObserved){
  auto truth=world();ObservationSpace s;s.setMap(partial(truth));Pose p{.7,1.4,0};
  auto raw=s.rawMap();MazePlannerSettings cfg;cfg.unknown_policy=UnknownPolicy::ObserveBeforeEnter;
  auto route=planMaze(raw,body(),p,{3.3,1.4,0},cfg,[]{return false;},&raw);ASSERT_GT(route.size(),2u);
  for(int step=0;step<1800&&p.x<3.2;++step){double t=10+step*.05;observe(s,truth,p,t,.65);ASSERT_TRUE(s.certify(body(),p,t+.001));
    const auto why=s.check(body(),p,{}, {.08,0,0},t+.001);ASSERT_EQ(why,"")<<p.x;
    p.x+=.004;
    ObservationSpace oracle;oracle.setMap(truth);auto world_map=oracle.rawMap();ASSERT_TRUE(footprintSafe(world_map,body(),p));
  }EXPECT_GT(p.x,3.19);EXPECT_EQ(s.identity(3.3,1.4),255);
}

TEST(Unknown, OlderSensorRayCannotClearNewerObstacleHit){
  auto truth=world();auto blocked=truth;obstacle(blocked,1.5,1.2,1.6,1.6);
  ObservationSpace s;s.setMap(partial(truth));Pose p{.8,1.4,0};
  ASSERT_TRUE(s.observe(0,scan(blocked,p,0,10.1,2),10.1,10.11));
  auto before=s.executionMap(10.11,10.11),raw=s.rawMap();std::vector<unsigned> hits;
  for(unsigned i=0;i<before.getSizeInCellsX()*before.getSizeInCellsY();++i)if(raw.getCharMap()[i]==255&&before.getCharMap()[i]==254){
    hits.push_back(i);EXPECT_GT(blocked.data[i],0); // No duplicate dilation into free cells.
  }
  ASSERT_FALSE(hits.empty());
  ASSERT_TRUE(s.observe(1,scan(truth,p,1,10,2),10,10.12));
  auto map=s.executionMap(10.13,10.13);for(auto i:hits)EXPECT_EQ(map.getCharMap()[i],254);
}
TEST(Unknown, MissingPoseContinuityDoesNotSeedUnknownFootprint){
  auto truth=world();ObservationSpace s;s.setMap(partial(truth));Pose p{1.5,1.4,0};
  observe(s,truth,{1,1.4,0},10,2);ASSERT_TRUE(s.certify(body(),p,10.01));
  // After observations expire and pose continuity is lost, previous body coverage
  // cannot be reused to initialize a new unknown footprint.
  EXPECT_FALSE(s.certify(body(),p,15));
}
TEST(Unknown, ContinuousPoseMaintainsOnlyCertifiedBodyAcrossHeldGoals){
  auto truth=world();ObservationSpace s;s.setMap(partial(truth));Pose p{1.5,1.4,0};
  observe(s,truth,{1,1.4,0},10,2);ASSERT_TRUE(s.certify(body(),p,10.01));
  for(int i=1;i<40;++i){const double t=10+i*.2;observe(s,truth,p,t,.65);ASSERT_TRUE(s.certify(body(),p,t+.001));}
  EXPECT_EQ(s.check(body(),p,{}, {.08,0,0},17.81),"");
  auto map=s.executionMap(17.81,17.81);EXPECT_FALSE(footprintSafe(map,body(),{3,1.4,0}));
}

TEST(Unknown, ObservedFreePreferenceDoesNotRewriteRawUnknownIdentity){
  auto nav=world();obstacle(nav,1.8,1.1,2.1,1.7);
  for(unsigned y=30;y<120;++y)for(unsigned x=40;x<180;++x)if(nav.data[y*nav.info.width+x]==0)nav.data[y*nav.info.width+x]=-1;
  ObservationSpace s;s.setMap(nav);auto raw=s.rawMap(),observed=raw;
  for(unsigned y=90;y<125;++y)for(unsigned x=35;x<185;++x)if(observed.getCost(x,y)==255)observed.setCost(x,y,0);
  MazePlannerSettings cfg;cfg.unknown_policy=UnknownPolicy::ObserveBeforeEnter;cfg.unknown_penalty=5;
  auto path=planMaze(raw,body(),{.6,1.4,0},{3.5,1.4,0},cfg,[]{return false;},&raw,&observed);
  double north=0;for(auto p:path)north=std::max(north,p.y);EXPECT_GT(north,1.9);
  unsigned x,y;ASSERT_TRUE(raw.worldToMap(2,2,x,y));EXPECT_EQ(raw.getCost(x,y),255);EXPECT_EQ(observed.getCost(x,y),0);
}

TEST(Unknown, MixedMotionEnvelopeIncludesUnequalBrakingAndSteeringChange){
  auto truth=world();ObservationSpace s;s.setMap(truth);Pose p{1,1.4,0};observe(s,truth,p,10,2);
  EXPECT_EQ(s.check(body(),p,{.08,0,.3},{.08,0,.3},10.01),"");
  // This cell lies outside the single constant-curvature stopping sweep, but
  // inside a possible trajectory when yaw brakes sooner than translation.
  obstacle(truth,1.35,1.19,1.39,1.23);s.setMap(truth);observe(s,truth,p,11,2);
  auto map=s.rawMap();ASSERT_TRUE(motionSafe(map,body(),p,{.08,0,.3},2.1));
  EXPECT_NE(s.check(body(),p,{.08,0,.3},{.08,0,.3},11.01),"");
  // A transition from forward to lateral motion must cover both directions.
  EXPECT_NE(s.check(body(),p,{.08,0,0},{0,-.08,0},11.01),"");
}

TEST(Unknown, CertifiedFootprintSurvivesSmallContinuousPoseJitter){
  auto truth=world();ObservationSpace s;s.setMap(partial(truth));Pose p{1.5,1.4,0};
  observe(s,truth,{1,1.4,0},10,2);ASSERT_TRUE(s.certify(body(),p,10.01));
  for(int i=1;i<120;++i){const double t=10+i*.1;
    p.x=1.5+(i%4<2?.001:-.001);p.y=1.4+(i%6<3?.001:-.001);p.yaw=i%2?.001:-.001;
    observe(s,truth,p,t,.65);ASSERT_TRUE(s.certify(body(),p,t+.001))<<"step "<<i;
  }
}

TEST(Unknown, PendingCommandsRemainInsideStoppingEnvelope){
  auto truth=world();obstacle(truth,1.32,1.2,1.36,1.6);ObservationSpace s;s.setMap(truth);Pose p{1,1.4,0};observe(s,truth,p,10,2);
  ASSERT_EQ(s.check(body(),p,{},{},10.01),"");
  EXPECT_NE(s.check(body(),p,{},{},10.01,{{.08,0,0}}),"");
  EXPECT_NE(s.check(body(),p,{}, {0,-.08,0},10.01,{{.08,0,.1}}),"");
}
