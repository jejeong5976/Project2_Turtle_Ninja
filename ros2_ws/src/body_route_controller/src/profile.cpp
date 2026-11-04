#include "body_route_controller/profile.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <fcntl.h>
#include <unistd.h>
#include <cerrno>
namespace body_route_controller
{
namespace {
// yaml-cpp emits an assigned double such as 0.0 as "0". ROS distinguishes
// integer and double parameters, including after bringup's PyYAML round trip.
YAML::Node real(double value)
{
  std::ostringstream out;out<<std::showpoint<<std::setprecision(17)<<value;
  return YAML::Load(out.str());
}
}
void TrialSettings::validate() const
{
  if(unknown_policy!="blocked"&&unknown_policy!="observe-before-enter")throw std::invalid_argument("unknown-policy must be blocked or observe-before-enter");
  if(!std::isfinite(unknown_penalty)||unknown_penalty<=0||!std::isfinite(observation_ttl)||observation_ttl<.8)
    throw std::invalid_argument("unknown penalty must be positive; observation TTL must be >= .8 seconds");
  if(mode!="custom"&&mode!="original") {throw std::invalid_argument("mode must be custom or original (MPPI)");}
  for(double n:{speed,yaw_rate,linear_accel,angular_accel,correction_speed}) {
    if(!std::isfinite(n)) {throw std::invalid_argument("limits must be finite");}
  }
  if(speed<.005||speed>.26||yaw_rate<.02||yaw_rate>1.6||
    linear_accel<=0||linear_accel>1||angular_accel<=0||angular_accel>6||
    correction_speed<=.003||correction_speed>speed) {
    throw std::invalid_argument("speed: .005.. .26 m/s; yaw-rate: .02..1.6 rad/s; accel: (0,1] m/s^2, (0,6] rad/s^2; correction-speed: (.003,speed]");
  }
}
double TrialSettings::measuredSpeed() const {return speed+std::max(.015,.2*speed);}
double TrialSettings::measuredYawRate() const {return yaw_rate+std::max(.05,.2*yaw_rate);}
std::string fingerprint(const std::string & bytes)
{
  std::uint64_t value=14695981039346656037ULL;
  for(unsigned char c:bytes) {value^=c;value*=1099511628211ULL;}
  std::ostringstream out;out<<std::hex<<std::setfill('0')<<std::setw(16)<<value;return "fnv1a64-"+out.str();
}
std::string readText(const std::string & path)
{
  std::ifstream in(path);if(!in) {throw std::runtime_error("cannot read "+path);}
  return {std::istreambuf_iterator<char>(in),std::istreambuf_iterator<char>()};
}
void writeNew(const std::string & path,const std::string & text)
{
  const int fd=::open(path.c_str(),O_WRONLY|O_CREAT|O_EXCL,0644);
  if(fd<0) {throw std::runtime_error("cannot create new file (will not overwrite): "+path);}
  std::size_t offset=0;
  while(offset<text.size()) {
    const auto n=::write(fd,text.data()+offset,text.size()-offset);
    if(n<0&&errno==EINTR) {continue;}
    if(n<=0) {::close(fd);throw std::runtime_error("write failed: "+path);}
    offset+=static_cast<std::size_t>(n);
  }
  if(::close(fd)!=0) {throw std::runtime_error("close failed: "+path);}
}
TrialProfile makeProfile(const YAML::Node & common,const YAML::Node & original,const TrialSettings & s)
{
  s.validate();TrialProfile p;p.navigation=YAML::Clone(common);
  auto controller=p.navigation["controller_server"]["ros__parameters"];
  if(controller["FollowPath"]["plugin"].as<std::string>()!="body_route_controller::BodyRouteController") {
    throw std::runtime_error("common profile must be the body-route comparison profile");
  }
  if(s.mode=="original") {
    const auto source=original["controller_server"]["ros__parameters"]["FollowPath"];
    if(source["plugin"].as<std::string>()!="nav2_mppi_controller::MPPIController"||source["motion_model"].as<std::string>()!="omni") {
      throw std::runtime_error("original profile must contain the existing omni MPPI controller");
    }
    controller["FollowPath"]=YAML::Clone(source);
    auto f=controller["FollowPath"];
    f["vx_max"]=real(s.speed);f["vx_min"]=real(-s.speed);f["vy_max"]=real(s.speed);f["wz_max"]=real(s.yaw_rate);
    f["ax_max"]=real(s.linear_accel);f["ax_min"]=real(-s.linear_accel);
    f["ay_max"]=real(s.linear_accel);f["ay_min"]=real(-s.linear_accel);f["az_max"]=real(s.angular_accel);
    // Preserve the original noise-to-limit ratios at the selected test speed.
    f["vx_std"]=real(source["vx_std"].as<double>()*s.speed/source["vx_max"].as<double>());
    f["vy_std"]=real(source["vy_std"].as<double>()*s.speed/source["vy_max"].as<double>());
    f["wz_std"]=real(source["wz_std"].as<double>()*s.yaw_rate/source["wz_max"].as<double>());
  } else {
    auto f=controller["FollowPath"];
    f["max_speed"]=real(s.speed);f["correction_speed"]=real(s.correction_speed);f["max_yaw_rate"]=real(s.yaw_rate);
    f["linear_accel"]=real(s.linear_accel);f["angular_accel"]=real(s.angular_accel);
    f["heading_hold_rate"]=real(std::min(.02,s.yaw_rate));
    // A fixed 12 cm target with kp=.5 caps speed at .06 m/s regardless of CLI.
    // Longer straight-leg lookahead reaches the requested cap; corners still
    // truncate the target so the core slows before the stationary turn.
    f["lookahead"]=real(std::max(f["lookahead"].as<double>(.12),1.5*s.speed/f["linear_kp"].as<double>(.5)));
  }
  auto planner=p.navigation["planner_server"]["ros__parameters"]["GridBased"];
  planner["unknown_policy"]=s.unknown_policy;planner["unknown_penalty"]=real(s.unknown_penalty);
  if(s.unknown_policy=="observe-before-enter"){
    auto f=controller["FollowPath"];
    f["delegate_plugin"]=f["plugin"].as<std::string>();f["plugin"]="body_route_controller::ObservationController";
    controller["general_goal_checker"]["plugin"]="body_route_controller::ObservedGoalChecker";
    for(const auto &name:{"local_costmap","global_costmap"}){
      auto cost=p.navigation[name][name]["ros__parameters"];
      cost["plugins"]=YAML::Load("[static_layer, obstacle_layer, observation_layer, inflation_layer]");
      auto layer=cost["observation_layer"];layer["plugin"]="body_route_controller::ObservationLayer";
      layer["unknown_policy"]=s.unknown_policy;layer["ttl"]=real(s.observation_ttl);
      layer["linear_decel"]=real(s.linear_accel);layer["angular_decel"]=real(s.angular_accel);
      layer["latency"]=real(.5);layer["scan_timeout"]=real(.3);
      cost["obstacle_layer"]["combination_method"]=1;
      cost["inflation_layer"]["inflate_around_unknown"]=false;
    }
  }
  p.drive["max_linear"]=real(s.speed);p.drive["max_angular"]=real(s.yaw_rate);
  p.drive["max_linear_accel"]=real(s.linear_accel);p.drive["max_angular_accel"]=real(s.angular_accel);
  p.drive["heading_hold_kp"]=real(0.0); // Same drivetrain, navigation owns yaw in both modes.
  auto t=p.trial["body_route_trial"]["ros__parameters"];
  t["unknown_policy"]=s.unknown_policy;t["unknown_penalty"]=real(s.unknown_penalty);t["observation_ttl"]=real(s.observation_ttl);
  t["controller_mode"]=s.mode;t["speed"]=real(s.speed);t["yaw_rate"]=real(s.yaw_rate);
  t["linear_accel"]=real(s.linear_accel);t["angular_accel"]=real(s.angular_accel);
  t["correction_speed"]=real(s.correction_speed);
  t["execute"]=false;t["require_live_view"]=true;
  const auto id=fingerprint(YAML::Dump(p.navigation)+YAML::Dump(p.drive)+YAML::Dump(p.trial));
  t["profile_id"]=id;p.manifest["profile_id"]=id;
  p.manifest["controller_mode"]=s.mode;p.manifest["unknown_policy"]=s.unknown_policy;p.manifest["unknown_penalty"]=real(s.unknown_penalty);
  p.manifest["measured_speed_limit"]=s.measuredSpeed();p.manifest["measured_yaw_limit"]=s.measuredYawRate();
  p.manifest["common_input_id"]=fingerprint(YAML::Dump(common));
  p.manifest["original_input_id"]=fingerprint(YAML::Dump(original));
  return p;
}
}
