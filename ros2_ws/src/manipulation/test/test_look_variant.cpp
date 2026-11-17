#include <gtest/gtest.h>
#include <limits>
#include "manipulation/look_variant.hpp"
#include "arm_kin_data.hpp"
using namespace manipulation;
namespace
{
const PathNode front{"look", {-.32020, -.27540, -1.05500, .58640, -.15840}, false, 0.0, "", true};
class LookVariant : public ::testing::Test
{
protected:
  ArmModel model = kin_data::model();
  LookOptions options;
  PathNode node;
  std::string why;
  bool make(const std::string & slot) {return make_look_variant(front, slot, options, &model, node, why);}
};
}
TEST_F(LookVariant, DefaultFarNearAreExactFront)
{
  for (const auto slot : {"far", "near", ""}) {
    ASSERT_TRUE(make(slot)); EXPECT_TRUE(why.empty()); EXPECT_EQ(node.q, front.q);
    EXPECT_EQ(node.name, front.name); EXPECT_EQ(node.stop, front.stop);
  }
}
TEST_F(LookVariant, DefaultSideIsOriginalJ1Only)
{
  for (const auto slot : {"left", "right"}) {
    ASSERT_TRUE(make(slot));
    EXPECT_DOUBLE_EQ(node.q[0], front.q[0] + (std::string(slot) == "left" ? 1.0 : -1.0) * 18 * M_PI / 180);
    for (int j = 1; j < 5; ++j) {EXPECT_DOUBLE_EQ(node.q[j], front.q[j]);}
  }
}
TEST_F(LookVariant, PositiveSideAndFarLiftJ4)
{
  options.side_tilt_deg = options.far_tilt_deg = 8;
  for (const auto slot : {"left", "right", "far"}) {
    ASSERT_TRUE(make(slot)); EXPECT_TRUE(why.empty());
    EXPECT_DOUBLE_EQ(node.q[3], front.q[3] - 8 * M_PI / 180); EXPECT_DOUBLE_EQ(node.q[2], front.q[2]);
  }
}
TEST_F(LookVariant, NearUsesJ3AndDoesNotCrossJ4Stop)
{
  options.near_tilt_deg = 8;
  ASSERT_TRUE(make("near")); ASSERT_TRUE(why.empty());
  EXPECT_DOUBLE_EQ(node.q[2], front.q[2] + 8 * M_PI / 180);
  EXPECT_DOUBLE_EQ(node.q[3], front.q[3]);
  EXPECT_LT(2048 + node.q[3] * 4096 / (2 * M_PI), 2485);
}
TEST_F(LookVariant, InvalidNewAnglesFallbackWithoutRejection)
{
  for (double a : {31.0, std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()}) {
    options.side_tilt_deg = options.far_tilt_deg = options.near_tilt_deg = a;
    for (const auto slot : {"far", "near", "left", "right"}) {
      ASSERT_TRUE(make(slot)); EXPECT_FALSE(why.empty());
      EXPECT_DOUBLE_EQ(node.q[3], front.q[3]); EXPECT_DOUBLE_EQ(node.q[2], front.q[2]);
      if (std::string(slot) == "far" || std::string(slot) == "near") {EXPECT_EQ(node.q, front.q);}
    }
  }
}
TEST_F(LookVariant, OutOfModelRangeTiltFallsBackToOriginalSide)
{
  options.side_tilt_deg = -5; ASSERT_TRUE(make("right")); EXPECT_FALSE(why.empty());
  EXPECT_DOUBLE_EQ(node.q[3], front.q[3]);
  EXPECT_DOUBLE_EQ(node.q[0], front.q[0] - 18 * M_PI / 180);
}
TEST_F(LookVariant, MissingModelFallsBackOnlyNewTilt)
{
  options.far_tilt_deg = 5;
  ASSERT_TRUE(make_look_variant(front,"far",options,nullptr,node,why)); EXPECT_EQ(node.q,front.q); EXPECT_FALSE(why.empty());
  options.side_tilt_deg = 5;
  ASSERT_TRUE(make_look_variant(front,"left",options,nullptr,node,why)); EXPECT_NE(node.q[0],front.q[0]); EXPECT_DOUBLE_EQ(node.q[3],front.q[3]);
}
TEST_F(LookVariant, SmallTiltStillRecognizedInsideFrontTolerance)
{
  options.far_tilt_deg = 3; ASSERT_TRUE(make("far")); PathNode found;
  std::string w; ASSERT_TRUE(path_start_ok(ArmPath{"front",{front},false},node.q,.15,w));
  ASSERT_TRUE(match_look_variant(front,{node},node.q,.15,found)); EXPECT_EQ(found.q,node.q);
}
TEST_F(LookVariant, ParameterChangeDoesNotLosePreviousPose)
{
  options.side_tilt_deg = 8; ASSERT_TRUE(make("right")); const auto old = node;
  options.side_tilt_deg = 15; options.side_deg = 26; ASSERT_TRUE(make("right")); PathNode found;
  ASSERT_TRUE(match_look_variant(front,{old,node},old.q,.15,found)); EXPECT_EQ(found.q,old.q);
}
TEST_F(LookVariant, FrontNeverMistakenForNearbyVariant)
{
  options.near_tilt_deg = 3; ASSERT_TRUE(make("near")); PathNode found;
  EXPECT_FALSE(match_look_variant(front,{node},front.q,.15,found));
}
TEST_F(LookVariant, UnrelatedAndMissingStateDoNotMatch)
{
  ASSERT_TRUE(make("right")); PathNode found;
  auto wrong = node.q; wrong[1] += .5;
  EXPECT_FALSE(match_look_variant(front,{node},wrong,.15,found));
  wrong=node.q; wrong[2]=std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(match_look_variant(front,{node},wrong,.15,found));
  EXPECT_FALSE(match_look_variant(front,{node},{},.15,found));
}
TEST_F(LookVariant, AllVariantsReturnThroughSeparateExactFrontStop)
{
  options.side_tilt_deg=options.far_tilt_deg=options.near_tilt_deg=8;
  PathNode mid{"retreat",{-.32020,-.5,-.8,.3,-.15840},false,0,"",false};
  PathNode home{"home",{0,-1.1,.8,0,0},true,0,"",false};
  const ArmPath base{"home_from_look",{front,mid,home},true};
  for (const auto slot : {"left","right","far","near"}) {
    ASSERT_TRUE(make(slot)); const auto bridged = look_home_bridge(node,front,base);
    ASSERT_EQ(bridged.nodes.size(),4u); EXPECT_EQ(bridged.nodes[1].q,front.q); EXPECT_TRUE(bridged.nodes[1].stop);
    EXPECT_EQ(bridged.nodes[2].q,mid.q); EXPECT_EQ(bridged.nodes[3].q,home.q); EXPECT_TRUE(bridged.ends_home);
    const auto actions=path_actions(bridged,PathTiming{},GripperValues{},node.q);
    ASSERT_GE(actions.size(),2u); EXPECT_EQ(actions.front().points.back().q,front.q); EXPECT_EQ(actions.front().pose,front.name);
    EXPECT_EQ(actions.back().pose,"home"); EXPECT_EQ(actions.back().points.back().q,home.q);
  }
}
TEST_F(LookVariant, HomeBridgePreservesGripperAndIntermediatePath)
{
  ASSERT_TRUE(make("right"));
  PathNode mid{"open",{0,-.5,-.8,.3,0},true,12,"open",true,.2};
  PathNode home{"home",{0,-1.1,.8,0,0},true,0,"",false};
  const ArmPath base{"home_from_look",{front,mid,home},true};
  const auto bridge=look_home_bridge(node,front,base);
  EXPECT_EQ(bridge.nodes[2].grip,"open"); EXPECT_DOUBLE_EQ(bridge.nodes[2].hold_s,.2); EXPECT_DOUBLE_EQ(bridge.nodes[2].speed_dps,12);
  const auto actions=path_actions(bridge,PathTiming{},GripperValues{},node.q);
  int ng=0; for (const auto & a:actions) {if(a.is_gripper) {++ng; EXPECT_DOUBLE_EQ(a.gripper,GripperValues{}.open);}}
  EXPECT_EQ(ng,1);
}
TEST_F(LookVariant, BridgeUsesExactFrontEvenIfRecordedStartDiffers)
{
  ASSERT_TRUE(make("right"));
  PathNode recorded=front; recorded.q[3] -= .01;
  const auto bridge=look_home_bridge(node,front,ArmPath{"home_from_look",{recorded,front},true});
  ASSERT_EQ(bridge.nodes.size(),4u); EXPECT_EQ(bridge.nodes[1].q,front.q); EXPECT_TRUE(bridge.nodes[1].stop);
  EXPECT_EQ(bridge.nodes[2].q,recorded.q);
}

TEST_F(LookVariant, CommonTrackingErrorDoesNotHideSmallTilt)
{
  options.far_tilt_deg = options.near_tilt_deg = 3;
  for (const auto slot : {"far", "near"}) {
    ASSERT_TRUE(make(slot)); PathNode found;
    auto measured = node.q; measured[0] += .06; measured[1] -= .06;
    ASSERT_TRUE(match_look_variant(front,{node},measured,.15,found)); EXPECT_EQ(found.q,node.q);
    const auto bridge=look_home_bridge(found,front,ArmPath{"home_from_look",{front,node},true});
    const auto actions=path_actions(bridge,PathTiming{},GripperValues{},measured);
    ASSERT_GE(actions.size(),2u); EXPECT_EQ(actions.front().points.back().q,front.q);
  }
}
