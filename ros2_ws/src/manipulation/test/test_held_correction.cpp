// 쥔 퍽 보정: 시각/ID/좌표 부호/깊이 정규화의 ROS 없는 회귀 시험임.
#include <gtest/gtest.h>
#include <Eigen/Geometry>
#include <limits>
#include "manipulation/held_correction.hpp"
using namespace manipulation;
namespace {
const Eigen::Vector3d up=Eigen::Vector3d::UnitZ();
std::vector<HeldSample> samples() {return {{"3","wrist_link",9.6,{0,0,.03},up},{"3","wrist_link",9.8,{0,0,.03},up},{"3","wrist_link",10,{0,0,.03},up}};}
HeldEstimate current() {return fresh_held(samples(),"3",9.3,10,up);}
HeldReference ref50() {return {{0,0,.05},up,50};}
}
TEST(HeldCorrection, FreshUniqueFrames) {const auto e=current();EXPECT_TRUE(e.ok);EXPECT_EQ(e.count,3);EXPECT_EQ(e.id,"3");}
TEST(HeldCorrection, StaleAndBeforePickDoNotCount) {EXPECT_FALSE(fresh_held(samples(),"3",9.3,11,up).ok);EXPECT_FALSE(fresh_held(samples(),"3",9.9,10,up).ok);}
TEST(HeldCorrection, DuplicateFrameDoesNotFillWindow) {auto s=samples();s[1].stamp=s[0].stamp;EXPECT_FALSE(fresh_held(s,"3",9,10,up).ok);}
TEST(HeldCorrection, WrongIdOrFrameDoesNotCount) {auto s=samples();EXPECT_FALSE(fresh_held(s,"4",9,10,up).ok);s[0].frame="base_link";EXPECT_FALSE(fresh_held(s,"3",9,10,up).ok);}
TEST(HeldCorrection, FutureDoesNotCount) {auto s=samples();s[0].stamp=10.1;EXPECT_FALSE(fresh_held(s,"3",9,10,up).ok);}
TEST(HeldCorrection, UnknownIdMustBeUnique) {auto s=samples();EXPECT_TRUE(fresh_held(s,"",9,10,up).ok);s.push_back({"4","wrist_link",9.7,{0,0,.03},up});EXPECT_FALSE(fresh_held(s,"",9,10,up).ok);EXPECT_TRUE(fresh_held(s,"3",9,10,up).ok);}
TEST(HeldCorrection, PositionJitterFallsBack) {auto s=samples();s[0].p.x()=.0021;EXPECT_FALSE(fresh_held(s,"3",9,10,up).ok);}
TEST(HeldCorrection, NormalJitterFallsBack) {auto s=samples();s[0].n=Eigen::AngleAxisd(.10,Eigen::Vector3d::UnitX())*up;EXPECT_FALSE(fresh_held(s,"3",9,10,up).ok);}
TEST(HeldCorrection, NormalWindingIsCanonicalized) {auto s=samples();s[0].n=-up;EXPECT_TRUE(fresh_held(s,"3",9,10,up).ok);}
TEST(HeldCorrection, InvalidValuesDoNotCount) {auto s=samples();s[0].p.x()=std::numeric_limits<double>::quiet_NaN();EXPECT_FALSE(fresh_held(s,"3",9,10,up).ok);s=samples();s[0].n.setZero();EXPECT_FALSE(fresh_held(s,"3",9,10,up).ok);}
TEST(HeldCorrection, FiftyToThirtyIsNotTwentyMmSlip) {const auto c=held_correction(current(),ref50(),30,up,Eigen::Matrix3d::Identity(),8);ASSERT_TRUE(c.ok);EXPECT_NEAR(c.residual.norm(),0,1e-12);}
TEST(HeldCorrection, WorldOffsetHasOppositeSign) {auto e=current();e.p.x()=.002;const Eigen::Matrix3d R=Eigen::AngleAxisd(M_PI/2,up).toRotationMatrix();const auto c=held_correction(e,ref50(),30,up,R,8);ASSERT_TRUE(c.ok);EXPECT_NEAR(c.xy.x(),0,1e-12);EXPECT_NEAR(c.xy.y(),-.002,1e-12);}
TEST(HeldCorrection, MissingDepthNeverApplies) {auto r=ref50();r.depth_mm=0;EXPECT_FALSE(held_correction(current(),r,30,up,Eigen::Matrix3d::Identity(),8).ok);}
TEST(HeldCorrection, OverLimitLeavesZeroCorrection) {auto e=current();e.p.x()=.0081;const auto c=held_correction(e,ref50(),30,up,Eigen::Matrix3d::Identity(),8);EXPECT_FALSE(c.ok);EXPECT_DOUBLE_EQ(c.xy.norm(),0);}
TEST(HeldCorrection, InvalidCurrentCannotApply) {auto e=current();e.ok=false;EXPECT_FALSE(held_correction(e,ref50(),30,up,Eigen::Matrix3d::Identity(),8).ok);e=current();e.p.x()=std::numeric_limits<double>::quiet_NaN();EXPECT_FALSE(held_correction(e,ref50(),30,up,Eigen::Matrix3d::Identity(),8).ok);}
