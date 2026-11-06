#include "body_route_controller/start_localization.hpp"
#include <gtest/gtest.h>
#include <cmath>
#include <algorithm>
#include <limits>

namespace sl=body_route_controller::start_localization;
namespace {
sl::Config config() {return sl::readConfig(START_CONFIG);}
std::array<std::vector<sl::Point>,2> map() {
  std::array<std::vector<sl::Point>,2> m;
  for(int i=0;i<=200;++i) {m[0].push_back({-2.2,-.45+i*.01});m[1].push_back({-2.1+i*.01,-.60});}
  return m;
}
std::vector<sl::Sample> observed(const sl::Config & c) {
  const sl::Pose truth{-1.65,.27,1.59};
  std::vector<sl::Sample> points;std::array<std::size_t,2> index{0,0};
  for(const auto & wall:map()) for(const auto & p:wall) {
    const auto q=sl::inverse(p,truth);
    const int id=sl::select(q,{c.rear_mount[0],c.rear_mount[1]},c);
    if(id>=0) points.push_back({q,static_cast<std::size_t>(id),index[id]++});
  }
  return points;
}
}
TEST(StartLocalization, FixedWallsRecoverPositionAndHeading) {
  const auto c=config();const auto result=sl::fit(map(),observed(c),c);
  EXPECT_TRUE(result.accepted);
  EXPECT_NEAR(result.pose[0],-1.65,.01);EXPECT_NEAR(result.pose[1],.27,.01);
  EXPECT_NEAR(result.pose[2],1.59,.01);
  for(const auto & m:result.heldout) {EXPECT_GE(m.count,8u);EXPECT_GE(m.span,.25);EXPECT_LT(m.p90,.015);}
}
TEST(StartLocalization, MovingFrontWallCannotEnterFit) {
  const auto c=config();const auto origin=sl::Point{c.rear_mount[0],c.rear_mount[1]};
  // Move the front wall through multiple positions; both near and distant returns remain excluded.
  for(double forward:{.25,.50,1.0,1.5}) for(int i=-50;i<=50;++i)
    EXPECT_EQ(sl::select({forward,i*.02},origin,c),-1);
  EXPECT_EQ(sl::select({std::numeric_limits<double>::quiet_NaN(),0},origin,c),-1);
}
TEST(StartLocalization, OneWallCannotProduceAcceptedPose) {
  const auto c=config();auto p=observed(c);
  p.erase(std::remove_if(p.begin(),p.end(),[](const auto & q){return q.wall==1;}),p.end());
  EXPECT_THROW(sl::fit(map(),p,c),std::runtime_error);
}
TEST(StartLocalization, SmallClusterCannotConstrainHeading) {
  const auto c=config();std::vector<sl::Sample> p;
  for(std::size_t i=0;i<20;++i) for(std::size_t wall=0;wall<2;++wall)
    p.push_back({wall==0?sl::Point{-.2,.55}:sl::Point{-.9,.0},wall,i});
  EXPECT_FALSE(sl::fit(map(),p,c).accepted);
}
TEST(StartLocalization, WrongRegionAndNonFiniteSamplesFail) {
  const auto c=config();EXPECT_EQ(sl::select({-2.5,1.5},{-.1145,0},c),-1);
  auto p=observed(c);p.front().body[0]=std::numeric_limits<double>::infinity();
  EXPECT_THROW(sl::fit(map(),p,c),std::runtime_error);
}
