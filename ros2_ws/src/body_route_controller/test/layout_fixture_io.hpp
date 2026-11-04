#pragma once
#include "layout_fixture.hpp"
#include "body_route_controller/profile.hpp"
#include <yaml-cpp/yaml.h>
#include <filesystem>
namespace layout_test {
inline YAML::Node coordinates(double x,double y,double yaw) {YAML::Node n;n.push_back(x);n.push_back(y);n.push_back(yaw);return n;}
inline YAML::Node coordinates(Point p) {YAML::Node n;n.push_back(p.x);n.push_back(p.y);return n;}
inline void saveGrid(const Grid & g,const std::filesystem::path & directory,const std::string & name) {
  std::string pixels;for(int y=g.height-1;y>=0;--y) {for(int x=0;x<g.width;++x) {pixels.push_back(g.cells[y*g.width+x]==100?0:static_cast<char>(254));}}
  writeNew((directory/(name+".pgm")).string(),"P5\n"+std::to_string(g.width)+" "+std::to_string(g.height)+"\n255\n"+pixels);
  YAML::Node map;map["image"]=name+".pgm";map["resolution"]=g.resolution;map["origin"]=coordinates(g.origin.x,g.origin.y,0);
  map["negate"]=0;map["occupied_thresh"]=.65;map["free_thresh"]=.25;
  writeNew((directory/(name+".yaml")).string(),YAML::Dump(map)+"\n");
}
inline YAML::Node templateDocument(const std::filesystem::path & directory) {
  YAML::Node d;d["schema_version"]=1;d["map"]=(directory/"nominal.yaml").string();d["fixed_start"]=coordinates(-.5,0,0);
  d["start_uncertainty"]["position_sigma_m"]=.003;d["start_uncertainty"]["yaw_sigma_rad"]=.0087;
  d["description"]="SYNTHETIC integration fixture, not physical L2 geometry";
  for(const auto & w:nominal()) {YAML::Node n;n["id"]=w.id;n["a"]=coordinates(w.a);n["b"]=coordinates(w.b);d["walls"].push_back(n);}
  YAML::Node portal;portal["first_wall"]="lower";portal["first_endpoint"]="b";portal["second_wall"]="upper";portal["second_endpoint"]="a";
  portal["heading"]=0.;portal["approach"]=.35;portal["exit"]=.35;d["portals"].push_back(portal);return d;
}
inline YAML::Node surveyDocument(const std::filesystem::path & directory,const std::vector<Frame>& frames) {
  auto d=templateDocument(directory);
  for(const auto & f:frames) {
    YAML::Node n;n["stamp"]=f.stamp;n["settled_seconds"]=f.settled_seconds;n["odom"]=coordinates(f.odom.x,f.odom.y,f.odom.yaw);
    n["measured_velocity"]=coordinates(f.measured.x,f.measured.y,f.measured.yaw);
    for(const auto & s:f.scans) {
      YAML::Node scan;scan["sensor_id"]=s.sensor_id;scan["stamp"]=s.stamp;scan["sensor_pose"]=coordinates(s.sensor.x,s.sensor.y,s.sensor.yaw);
      scan["angle_min"]=s.angle_min;scan["angle_increment"]=s.angle_increment;scan["range_min"]=s.range_min;scan["range_max"]=s.range_max;
      for(double r:s.ranges) {scan["ranges"].push_back(r);}n["scans"].push_back(scan);
    }d["frames"].push_back(n);
  }return d;
}
inline void fixture(const std::filesystem::path & directory) {
  if(!std::filesystem::create_directory(directory)) {throw std::runtime_error("fixture directory must be new");}
  auto world=nominal();for(int i=0;i<2;++i) {world[i].a.x+=.03;world[i].b.x+=.03;world[i].a.y+=.02;world[i].b.y+=.02;}
  saveGrid(grid(nominal()),directory,"nominal");saveGrid(grid(world),directory,"truth");
  writeNew((directory/"survey.yaml").string(),YAML::Dump(surveyDocument(directory,survey(world,.001,true)))+"\n");
  writeNew((directory/"template.yaml").string(),YAML::Dump(templateDocument(directory))+"\n");
}
}
