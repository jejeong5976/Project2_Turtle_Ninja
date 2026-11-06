// Offline only. Never publishes a pose, TF, hold command or movement goal.
#include "body_route_controller/start_localization.hpp"
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>

namespace sl=body_route_controller::start_localization;
namespace {
YAML::Node metrics(const sl::Metrics & m) {
  YAML::Node n;
  n["count"]=m.count;n["span_m"]=m.span;n["median_m"]=m.median;n["p90_m"]=m.p90;
  return n;
}
YAML::Node analyze(const std::string & path,const sl::Config & c,
  const std::array<std::vector<sl::Point>,2> & map,sl::Result & result) {
  const auto snap=YAML::LoadFile(path);
  const auto seed=snap["pose"].as<sl::Pose>();
  for(double v:seed) if(!std::isfinite(v)) throw std::runtime_error("Nonfinite snapshot pose");
  const double speed=snap["speed"].as<double>(),yaw_rate=snap["wz"].as<double>();
  const double age=snap["scan_age"][1].as<double>();
  if(!snap["held"].as<bool>() || !snap["live"].as<bool>() ||
    !snap["interlock"]["fresh"].as<bool>() || !std::isfinite(speed) || speed<0 || speed>.001 ||
    !std::isfinite(yaw_rate) || std::abs(yaw_rate)>.01 || !(age>=0 && age<=.3))
    throw std::runtime_error("Require fresh, held and stationary rear-scan snapshot");
  if(snap["scans"].size()!=2) throw std::runtime_error("Snapshot must identify front then rear arrays");
  std::vector<sl::Sample> samples;
  std::array<std::size_t,2> indices{0,0};
  for(const auto & item:snap["scans"][1]) {
    const auto point=sl::inverse(item.as<sl::Point>(),seed);
    const int wall=sl::select(point,{c.rear_mount[0],c.rear_mount[1]},c);
    if(wall>=0) samples.push_back({point,static_cast<std::size_t>(wall),indices[wall]++});
  }
  result=sl::fit(map,samples,c);
  YAML::Node out;
  out["snapshot"]=path;out["snapshot_pose"]=seed;out["reference_pose"]=c.reference;
  out["candidate_pose"]=result.pose;out["accepted"]=result.accepted;
  out["rear_total_points"]=snap["scans"][1].size();out["selected_count"]=samples.size();
  out["sequence"]=snap["sequence"].as<int>();
  out["capture_ros_time"]=snap["interlock"]["ros_time"].as<double>();
  for(std::size_t i=0;i<2;++i) {
    out["walls"][c.walls[i].name]["map_occupied_cells"]=map[i].size();
    out["walls"][c.walls[i].name]["train"]=metrics(result.train[i]);
    out["walls"][c.walls[i].name]["heldout"]=metrics(result.heldout[i]);
  }
  for(const auto & p:samples) {
    YAML::Node point;point["wall"]=c.walls[p.wall].name;
    point["heldout"]=(p.index%2!=0);point["body"]=p.body;
    point["candidate_map"]=sl::transform(p.body,result.pose);
    out["selected_points"].push_back(point);
  }
  return out;
}
}  // namespace
int main(int argc,char ** argv) {
  try {
    if(argc!=6) throw std::runtime_error("fit_start_pose CONFIG_YAML MAP_YAML SNAPSHOT_1 SNAPSHOT_2 NEW_RESULT_YAML");
    if(std::filesystem::exists(argv[5])) throw std::runtime_error("Output must be new; preserve prior results");
    if(std::filesystem::equivalent(argv[3],argv[4])) throw std::runtime_error("Require independent snapshots");
    const auto config=sl::readConfig(argv[1]);const auto map=sl::readMap(argv[2],config);
    sl::Result first{},second{};
    YAML::Node out;
    out["first"]=analyze(argv[3],config,map,first);
    out["second"]=analyze(argv[4],config,map,second);
    const double delta_time=std::abs(out["first"]["capture_ros_time"].as<double>()-
      out["second"]["capture_ros_time"].as<double>());
    if(delta_time<.5) throw std::runtime_error("Snapshots must be separated by at least 0.5 seconds");
    const double d=std::hypot(first.pose[0]-second.pose[0],first.pose[1]-second.pose[1]);
    const double a=std::abs(std::remainder(first.pose[2]-second.pose[2],2*std::acos(-1)));
    const bool accepted=first.accepted && second.accepted && d<=config.max_snapshot_distance && a<=config.max_snapshot_yaw;
    out["accepted"]=accepted;out["snapshot_difference_m"]=d;
    out["snapshot_difference_deg"]=a*180/std::acos(-1);
    out["scope"]="Stationary rear lidar against two fixed map regions; front data excluded. Candidate only; no live pose is applied.";
    out["limitations"]="Map consistency, not surveyed accuracy or clearance. Review wall ROIs and actual placement. Retain 0.10 m / 5 degree initial uncertainty. Driving localization requires separate integration.";
    std::ofstream file(argv[5]);file<<out<<'\n';file.close();
    if(!file) throw std::runtime_error("Failed to write result");
    std::cout<<"accepted="<<accepted<<" difference_m="<<d<<" difference_deg="<<a*180/std::acos(-1)<<'\n';
    return accepted?0:2;
  } catch(const std::exception & e) {std::cerr<<e.what()<<'\n';return 1;}
}
