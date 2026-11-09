#include "body_route_controller/passage.hpp"
#include <gtest/gtest.h>
#include <cmath>
using namespace body_route_controller;
namespace {
constexpr double pi=3.14159265358979323846;
Settings settings() {Settings s;s.drive_style="continuous";s.max_speed=.08;s.lookahead=.24;return s;}
Footprint footprint() {Footprint f;for(auto xy:std::vector<Point>{{-.205,-.205},{.205,-.205},{.205,.205},{-.205,.205}}){geometry_msgs::msg::Point p;p.x=xy.x;p.y=xy.y;f.push_back(p);}return f;}
void obstacle(nav2_costmap_2d::Costmap2D & m,double x,double y,unsigned char cost=254) {unsigned a,b;ASSERT_TRUE(m.worldToMap(x,y,a,b));m.setCost(a,b,cost);}
std::vector<Point> route{{0,-1},{0,0},{1.2,0}};
Pose goal{1.2,0,0};
// A narrow eastbound passage after a generous turning area at the corner.
void walls(nav2_costmap_2d::Costmap2D & m) {for(double x=.45;x<.9;x+=.005){obstacle(m,x,.33);obstacle(m,x,-.33);}}
}
TEST(Passage, DisabledAndAxisModePreserveBaseline) {
  nav2_costmap_2d::Costmap2D m(600,600,.01,-3,-3,0);
  PreparedPassage disabled(settings());
  EXPECT_FALSE(disabled.step(m,footprint(),{0,-.4,pi/2},{},route,goal,.05,.08,{}));
  PassageSettings opt;opt.enabled=true;PreparedPassage axis(Settings{},opt);
  EXPECT_FALSE(axis.step(m,footprint(),{0,-.4,pi/2},{},route,goal,.05,.08,{}));
}
TEST(Passage, ChoosesBeforeCollisionAndKeepsHeadingAcrossJitterAndWait) {
  nav2_costmap_2d::Costmap2D m(600,600,.01,-3,-3,0);walls(m);
  PassageSettings opt;opt.enabled=true;PreparedPassage c(settings(),opt);
  auto r=c.step(m,footprint(),{0,-.5,pi/2},{.08,0,0},route,goal,.05,.08,{});
  ASSERT_TRUE(r);EXPECT_EQ(r->mode,"passage_approach");EXPECT_TRUE(c.active());
  EXPECT_DOUBLE_EQ(c.crossingHeading(),0);
  r=c.step(m,footprint(),{0,0,pi/2},{},route,goal,.05,.08,{});
  ASSERT_TRUE(r);EXPECT_EQ(r->mode,"route_turn");EXPECT_DOUBLE_EQ(r->heading,0);
  c.pause();auto jitter=route;for(auto &p:jitter){p.y+=.004;}
  r=c.step(m,footprint(),{.002,.001,.2},{},jitter,goal,.05,.08,{});
  ASSERT_TRUE(r);EXPECT_EQ(r->mode,"route_turn");EXPECT_DOUBLE_EQ(r->heading,0);
  EXPECT_DOUBLE_EQ(c.corner().y,0);
}
TEST(Passage, ChangedRouteAndNewUnknownInvalidateWithZero) {
  for(bool change_route:{false,true}) {
    nav2_costmap_2d::Costmap2D m(600,600,.01,-3,-3,0);walls(m);
    PassageSettings opt;opt.enabled=true;PreparedPassage c(settings(),opt);
    ASSERT_TRUE(c.step(m,footprint(),{0,-.5,pi/2},{},route,goal,.05,.08,{}));
    auto updated=route;
    if(change_route){updated.back().y=.2;}else{obstacle(m,.7,0,255);}
    auto r=c.step(m,footprint(),{0,-.4,pi/2},{},updated,goal,.05,.08,{});
    ASSERT_TRUE(r);EXPECT_EQ(r->mode,"passage_wait");EXPECT_DOUBLE_EQ(r->command.x,0);EXPECT_DOUBLE_EQ(r->command.yaw,0);EXPECT_FALSE(c.active());
  }
}
TEST(Passage, ChangedIncomingDetourInvalidatesEvenWithSameExit) {
  nav2_costmap_2d::Costmap2D m(600,600,.01,-3,-3,0);walls(m);
  PassageSettings opt;opt.enabled=true;PreparedPassage c(settings(),opt);
  ASSERT_TRUE(c.step(m,footprint(),{0,-.5,pi/2},{},route,goal,.05,.08,{}));
  auto detour=route;detour.insert(detour.begin()+1,{.15,-.25});
  auto r=c.step(m,footprint(),{0,-.4,pi/2},{},detour,goal,.05,.08,{});
  ASSERT_TRUE(r);EXPECT_EQ(r->mode,"passage_wait");EXPECT_FALSE(c.active());
  EXPECT_EQ(c.reason(),"passage_route_changed");
}
TEST(Passage, StableDiagonalCorrectionSuppressesCPreviewUntilClear) {
  auto cfg=settings();cfg.heading_policy="preview";
  nav2_costmap_2d::Costmap2D m(600,600,.01,-3,-3,0);walls(m);
  PassageSettings opt;opt.enabled=true;PreparedPassage c(cfg,opt);Pose final=goal;final.yaw=.4;
  ASSERT_TRUE(c.step(m,footprint(),{0,-.5,pi/2},{},route,final,.05,.08,{}));
  ASSERT_TRUE(c.step(m,footprint(),{0,0,0},{},route,final,.05,.08,{}));
  for(int i=0;i<10;++i){
    auto r=c.step(m,footprint(),{.55,.005,0},{.02,0,0},route,final,.05,.08,{});
    ASSERT_TRUE(r);EXPECT_EQ(r->mode,"passage_cross");EXPECT_DOUBLE_EQ(r->heading,0);
    EXPECT_GT(r->command.x,0);EXPECT_LT(r->command.y,0);EXPECT_DOUBLE_EQ(r->command.yaw,0);
  }
  EXPECT_FALSE(c.step(m,footprint(),{1.18,0,0},{.03,0,0},route,final,.05,.08,{}));
}
TEST(Passage, OpenCornerUsesContinuousCurve) {
  nav2_costmap_2d::Costmap2D m(600,600,.01,-3,-3,0);
  PassageSettings opt;opt.enabled=true;PreparedPassage c(settings(),opt);
  auto r=c.step(m,footprint(),{0,-.5,pi/2},{},route,goal,.05,.08,{});
  ASSERT_TRUE(r);EXPECT_EQ(r->mode,"continuous");EXPECT_EQ(c.reason(),"prepared_open_curve");
}
TEST(Passage, CannotTurnAtGoalInsideNarrowOpening) {
  nav2_costmap_2d::Costmap2D m(600,600,.01,-3,-3,0);walls(m);
  auto short_route=route;short_route.back().x=.7;Pose short_goal{.7,0,pi/2};
  PassageSettings opt;opt.enabled=true;PreparedPassage c(settings(),opt);
  ASSERT_TRUE(c.step(m,footprint(),{0,-.5,pi/2},{},short_route,short_goal,.05,.08,{}));
  ASSERT_TRUE(c.step(m,footprint(),{0,0,0},{},short_route,short_goal,.05,.08,{}));
  auto r=c.step(m,footprint(),{.699,0,0},{},short_route,short_goal,.05,.08,{});
  ASSERT_TRUE(r);EXPECT_EQ(r->mode,"passage_wait");EXPECT_DOUBLE_EQ(r->command.yaw,0);
}
TEST(Passage, UncertaintyInflationContainsDeclaredPoseErrors) {
  const auto f=footprint();auto outer=uncertaintyFootprint(f,{},.05,.0872664626);
  ASSERT_EQ(outer.size(),32u);
  // Support-plane containment checked over both yaw/translation extrema.
  for(double yaw:{-.0872664626,.0872664626}) for(int a=0;a<32;++a) for(auto p:f) {
    Point q{std::cos(yaw)*p.x-std::sin(yaw)*p.y+.05*std::cos(2*pi*a/32),
      std::sin(yaw)*p.x+std::cos(yaw)*p.y+.05*std::sin(2*pi*a/32)};
    for(std::size_t i=0;i<outer.size();++i) {
      const auto u=outer[i],v=outer[(i+1)%outer.size()];
      EXPECT_GE((v.x-u.x)*(q.y-u.y)-(v.y-u.y)*(q.x-u.x),-1e-9);
    }
  }
}

TEST(Passage, RollingGridAndNewlyBlockedInflationInvalidateRetainedGeometry) {
  for(bool roll:{false,true}) {
    nav2_costmap_2d::Costmap2D m(600,600,.01,-3,-3,0);walls(m);
    obstacle(m,.7,0,200); // inflation remains nonblocking in existing geometry policy
    PassageSettings opt;opt.enabled=true;PreparedPassage c(settings(),opt);
    ASSERT_TRUE(c.step(m,footprint(),{0,-.5,pi/2},{},route,goal,.05,.08,{}));
    if(roll)m.updateOrigin(-2.9,-3);
    obstacle(m,.7,0,255);
    auto r=c.step(m,footprint(),{0,-.4,pi/2},{},route,goal,.05,.08,{});
    ASSERT_TRUE(r);EXPECT_EQ(r->mode,"passage_wait");EXPECT_DOUBLE_EQ(r->command.x,0);
    EXPECT_FALSE(c.active());EXPECT_EQ(c.reason(),"passage_obstacle_or_unknown_changed");
  }
}
