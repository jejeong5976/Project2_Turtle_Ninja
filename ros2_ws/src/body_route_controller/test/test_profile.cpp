#include "body_route_controller/profile.hpp"
#include "body_route_controller/core.hpp"
#include <gtest/gtest.h>
#include <cmath>
using namespace body_route_controller;
namespace {
YAML::Node common() {return YAML::LoadFile(std::string(BODY_ROUTE_TEST_DATA)+"/../../config/nav2.yaml");}
YAML::Node original() {return YAML::Load(R"(
controller_server:
  ros__parameters:
    FollowPath:
      plugin: nav2_mppi_controller::MPPIController
      motion_model: omni
      vx_max: 0.26
      vy_max: 0.26
      wz_max: 1.6
      vx_std: 0.15
      vy_std: 0.15
      wz_std: 0.4
      critics: [CostCritic, PathFollowCritic]
)");}
}
TEST(Profile, BothModesShareMapPlannerFootprintStopAndDrivetrain)
{
  TrialSettings a;a.speed=.08;a.yaw_rate=.3;a.correction_speed=.048;
  auto b=a;b.mode="original";
  const auto custom=makeProfile(common(),original(),a),mppi=makeProfile(common(),original(),b);
  for(const auto & name:{"amcl","local_costmap","global_costmap","planner_server","collision_monitor","bt_navigator"}) {
    EXPECT_EQ(YAML::Dump(custom.navigation[name]),YAML::Dump(mppi.navigation[name]));
  }
  EXPECT_EQ(YAML::Dump(custom.drive),YAML::Dump(mppi.drive));
  for(const auto & name:{"general_goal_checker","progress_checker","PathHandler"}) {
    EXPECT_EQ(YAML::Dump(custom.navigation["controller_server"]["ros__parameters"][name]),
      YAML::Dump(mppi.navigation["controller_server"]["ros__parameters"][name]));
  }
  EXPECT_NE(custom.manifest["profile_id"].as<std::string>(),mppi.manifest["profile_id"].as<std::string>());
}
TEST(Profile, LimitsReachAllLayersWithoutChangingInputs)
{
  auto input=common();const auto before=YAML::Dump(input);TrialSettings s;s.speed=.1;s.yaw_rate=.4;s.correction_speed=.06;
  auto p=makeProfile(input,original(),s);auto f=p.navigation["controller_server"]["ros__parameters"]["FollowPath"];
  EXPECT_DOUBLE_EQ(f["max_speed"].as<double>(),.1);
  EXPECT_DOUBLE_EQ(f["correction_speed"].as<double>(),.06);
  EXPECT_DOUBLE_EQ(p.drive["max_linear"].as<double>(),.1);
  EXPECT_DOUBLE_EQ(p.trial["body_route_trial"]["ros__parameters"]["speed"].as<double>(),.1);
  EXPECT_DOUBLE_EQ(p.manifest["measured_speed_limit"].as<double>(),.12);
  EXPECT_EQ(before,YAML::Dump(input));
  s.mode="original";p=makeProfile(input,original(),s);f=p.navigation["controller_server"]["ros__parameters"]["FollowPath"];
  EXPECT_DOUBLE_EQ(f["vx_min"].as<double>(),-.1);
  EXPECT_DOUBLE_EQ(f["vy_max"].as<double>(),.1);
  EXPECT_DOUBLE_EQ(f["wz_max"].as<double>(),.4);
  EXPECT_NEAR(f["vx_std"].as<double>(),.15*.1/.26,1e-12);
  EXPECT_EQ(f["critics"][0].as<std::string>(),"CostCritic");
  EXPECT_FALSE(f["max_speed"]);
}
TEST(Profile, InvalidAndUnboundedInputsAreRejected)
{
  for(double speed:{0.,-.1,static_cast<double>(NAN),static_cast<double>(INFINITY),.261,.003}) {
    TrialSettings s;s.speed=speed;EXPECT_THROW(s.validate(),std::invalid_argument);
  }
  TrialSettings s;s.mode="fast";EXPECT_THROW(s.validate(),std::invalid_argument);
  s={};s.yaw_rate=0;EXPECT_THROW(s.validate(),std::invalid_argument);
  s={};s.linear_accel=0;EXPECT_THROW(s.validate(),std::invalid_argument);
  s={};s.correction_speed=.03;EXPECT_THROW(s.validate(),std::invalid_argument);
}
TEST(Profile, DefaultThresholdsAndDeterministicProvenance)
{
  TrialSettings s;EXPECT_DOUBLE_EQ(s.measuredSpeed(),.04);EXPECT_NEAR(s.measuredYawRate(),.15,1e-12);
  auto a=makeProfile(common(),original(),s),b=makeProfile(common(),original(),s);
  EXPECT_EQ(YAML::Dump(a.manifest),YAML::Dump(b.manifest));
  EXPECT_NE(fingerprint("0 0 0\n"),fingerprint("0 1 0\n"));
  EXPECT_FALSE(a.trial["body_route_trial"]["ros__parameters"]["execute"].as<bool>());
}

TEST(Profile, LookaheadReachesCliSpeedAndStillStopsForCorner)
{
  TrialSettings s;s.speed=.08;s.yaw_rate=.3;
  auto p=makeProfile(common(),original(),s);auto f=p.navigation["controller_server"]["ros__parameters"]["FollowPath"];
  Settings cfg;cfg.max_speed=s.speed;cfg.lookahead=f["lookahead"].as<double>();Core core(cfg);
  const std::vector<Point> path{{0,0},{1,0},{1,1}};
  core.step({}, {}, path,{1,1,1.57},.05,s.speed);
  Result result;for(int i=0;i<80;++i)result=core.step({}, {}, path,{1,1,1.57},.05,s.speed);
  EXPECT_NEAR(result.command.x,s.speed,1e-12);
  auto target=routeTarget(path,{.94,0,0},cfg);
  EXPECT_NEAR(target.point.x,1,1e-12);EXPECT_NEAR(target.point.y,0,1e-12);EXPECT_NEAR(target.heading,0,1e-12);
}

TEST(Profile, ObservationPolicyUsesIdenticalGateAndGoalCheckForBothModes)
{
  for(const auto &mode:{"custom","original"}){
    TrialSettings s;s.mode=mode;s.unknown_policy="observe-before-enter";s.unknown_penalty=3.5;
    auto p=makeProfile(common(),original(),s);auto controller=p.navigation["controller_server"]["ros__parameters"];
    EXPECT_EQ(controller["FollowPath"]["plugin"].as<std::string>(),"body_route_controller::ObservationController");
    EXPECT_EQ(controller["general_goal_checker"]["plugin"].as<std::string>(),"body_route_controller::ObservedGoalChecker");
    for(const auto &name:{"local_costmap","global_costmap"}){
      auto layer=p.navigation[name][name]["ros__parameters"]["observation_layer"];
      EXPECT_EQ(layer["unknown_policy"].as<std::string>(),s.unknown_policy);
      EXPECT_DOUBLE_EQ(layer["linear_decel"].as<double>(),s.linear_accel);
    }
    EXPECT_DOUBLE_EQ(p.navigation["planner_server"]["ros__parameters"]["GridBased"]["unknown_penalty"].as<double>(),3.5);
  }
  TrialSettings s;s.unknown_policy="free";EXPECT_THROW(s.validate(),std::invalid_argument);
  s={};s.unknown_penalty=0;EXPECT_THROW(s.validate(),std::invalid_argument);
}
