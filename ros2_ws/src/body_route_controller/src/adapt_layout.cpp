#include "body_route_controller/layout_navigation.hpp"
#include "body_route_controller/profile.hpp"
#include <nav2_costmap_2d/footprint.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <cmath>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace fs=std::filesystem;
using namespace body_route_controller;
using namespace body_route_controller::layout;
namespace {
Point point(const YAML::Node & n) {
  if(!n.IsSequence()||n.size()!=2) {throw std::runtime_error("expected [x,y]");}
  return {n[0].as<double>(),n[1].as<double>()};
}
Pose pose(const YAML::Node & n) {
  if(!n.IsSequence()||n.size()!=3) {throw std::runtime_error("expected [x,y,yaw_radians]");}
  return {n[0].as<double>(),n[1].as<double>(),n[2].as<double>()};
}
YAML::Node node(Pose p) {YAML::Node n;n.push_back(p.x);n.push_back(p.y);n.push_back(p.yaw);return n;}
YAML::Node node(Point p) {YAML::Node n;n.push_back(p.x);n.push_back(p.y);return n;}
std::string token(std::istream & in) {
  while(in>>std::ws&&in.peek()=='#') {std::string line;std::getline(in,line);}
  std::string value;if(!(in>>value)) {throw std::runtime_error("truncated PGM header");}return value;
}
Grid readMap(const fs::path & file) {
  const auto map=YAML::LoadFile(file);const auto origin=pose(map["origin"]);
  if(std::abs(origin.yaw)>1e-12) {throw std::runtime_error("rotated occupancy origin unsupported");}
  auto image=fs::path(map["image"].as<std::string>());if(image.is_relative()) {image=file.parent_path()/image;}
  std::ifstream in(image,std::ios::binary);if(token(in)!="P5") {throw std::runtime_error("map must use binary P5 PGM");}
  Grid g;g.width=std::stoi(token(in));g.height=std::stoi(token(in));
  if(token(in)!="255") {throw std::runtime_error("map must use 8-bit PGM");}
  const char delimiter=in.get();if(delimiter=='\r'&&in.peek()=='\n') {in.get();}
  if(g.width<1||g.height<1||std::int64_t(g.width)*g.height>10000000) {throw std::runtime_error("invalid map dimensions");}
  g.origin={origin.x,origin.y};g.resolution=map["resolution"].as<double>();g.cells.resize(g.width*g.height);
  const bool negate=map["negate"].as<int>()!=0;const double occupied=map["occupied_thresh"].as<double>(),free=map["free_thresh"].as<double>();
  if(!std::isfinite(occupied)||!std::isfinite(free)||free<0||occupied>1||free>=occupied) {throw std::runtime_error("invalid map thresholds");}
  for(int y=g.height-1;y>=0;--y) {for(int x=0;x<g.width;++x) {
    const int pixel=in.get();if(pixel<0) {throw std::runtime_error("truncated map image");}
    const double p=negate?pixel/255.:(255-pixel)/255.;g.cells[y*g.width+x]=p>occupied?100:p<free?0:-1;
  }}
  g.validate();return g;
}
void writeMap(const Grid & g,const fs::path & directory) {
  std::string pixels;pixels.reserve(g.cells.size());
  for(int y=g.height-1;y>=0;--y) {for(int x=0;x<g.width;++x) {
    const int c=g.cells[y*g.width+x];pixels.push_back(static_cast<char>(c==100?0:c==0?254:205));
  }}
  writeNew((directory/"map.pgm").string(),"P5\n"+std::to_string(g.width)+" "+std::to_string(g.height)+"\n255\n"+pixels);
  YAML::Node map;map["image"]="map.pgm";map["resolution"]=g.resolution;map["origin"]=node(Pose{g.origin.x,g.origin.y,0});
  map["negate"]=0;map["occupied_thresh"]=.65;map["free_thresh"]=.25;map["mode"]="trinary";
  writeNew((directory/"map.yaml").string(),YAML::Dump(map)+"\n");
}
}
int main(int argc,char ** argv) {
  try {
    const auto began=std::chrono::steady_clock::now();
    fs::path input,output,params;
    for(int i=1;i<argc;++i) {
      const std::string arg=argv[i];
      if(arg=="--help") {std::cout<<"adapt_layout --input stationary_survey.yaml --params nav2.yaml --output NEW_DIRECTORY\nOffline only; no ROS node, TF, command or hardware access.\n";return 0;}
      if(i+1>=argc) {throw std::runtime_error("missing argument value");}
      if(arg=="--input") {input=argv[++i];}else if(arg=="--output") {output=argv[++i];}
      else if(arg=="--params") {params=argv[++i];}else {throw std::runtime_error("unknown argument: "+arg);}
    }
    if(input.empty()||output.empty()||params.empty()) {throw std::runtime_error("--input, --params and --output are required");}
    const std::string bytes=readText(input);const auto data=YAML::Load(bytes);
    if(data["schema_version"].as<int>()!=1) {throw std::runtime_error("unsupported survey schema");}
    std::vector<Wall> walls;std::vector<Frame> frames;
    for(const auto & w:data["walls"]) {walls.push_back({w["id"].as<std::string>(),point(w["a"]),point(w["b"]),w["assembly"]?w["assembly"].as<std::string>():""});}
    for(const auto & f:data["frames"]) {
      Frame frame;frame.stamp=f["stamp"].as<double>();frame.settled_seconds=f["settled_seconds"].as<double>();frame.odom=pose(f["odom"]);
      const auto velocity=pose(f["measured_velocity"]);frame.measured={velocity.x,velocity.y,velocity.yaw};
      for(const auto & s:f["scans"]) {
        Scan scan;scan.sensor_id=s["sensor_id"].as<std::string>();scan.stamp=s["stamp"].as<double>();scan.sensor=pose(s["sensor_pose"]);
        scan.angle_min=s["angle_min"].as<double>();scan.angle_increment=s["angle_increment"].as<double>();
        scan.range_min=s["range_min"].as<double>();scan.range_max=s["range_max"].as<double>();scan.ranges=s["ranges"].as<std::vector<double>>();
        frame.scans.push_back(scan);
      }frames.push_back(frame);
    }
    const auto config=YAML::LoadFile(params);const auto costmap=config["global_costmap"]["global_costmap"]["ros__parameters"];
    Footprint footprint;
    if(!nav2_costmap_2d::makeFootprintFromString(costmap["footprint"].as<std::string>(),footprint)) {throw std::runtime_error("invalid Nav2 polygon footprint");}
    const double padding=costmap["footprint_padding"].as<double>();
    if(!std::isfinite(padding)||padding<0||padding>.2) {throw std::runtime_error("invalid footprint padding");}
    nav2_costmap_2d::padFootprint(footprint,padding);
    const auto local=config["local_costmap"]["local_costmap"]["ros__parameters"];
    if(local["footprint"].as<std::string>()!=costmap["footprint"].as<std::string>()||local["footprint_padding"].as<double>()!=padding) {
      throw std::runtime_error("local/global footprints must match for a shared checked snapshot");
    }
    auto map_path=fs::path(data["map"].as<std::string>());if(map_path.is_relative()) {map_path=input.parent_path()/map_path;}
    body_route_controller::layout::Settings settings;
    settings.start_sigma=data["start_uncertainty"]["position_sigma_m"].as<double>();
    settings.start_yaw_sigma=data["start_uncertainty"]["yaw_sigma_rad"].as<double>();settings.validate();
    const auto prior=readMap(map_path);const auto result=estimate(walls,frames,pose(data["fixed_start"]),settings);
    if(!fs::create_directory(output)) {throw std::runtime_error("output directory must not already exist");}
    YAML::Node report;report["schema_version"]=1;report["offline_snapshot"]=true;report["accepted"]=result.accepted;report["reason"]=result.reason;
    report["input_id"]=fingerprint(bytes);report["params_id"]=fingerprint(readText(params));
    report["start_uncertainty"]=data["start_uncertainty"];
    report["prior_yaml_id"]=fingerprint(readText(map_path));report["geometric_rank"]=result.geometric_rank;report["variables"]=result.variables;
    report["prior_cells_id"]=fingerprint(std::string(reinterpret_cast<const char*>(prior.cells.data()),prior.cells.size()));
    report["matched"]=result.matched;report["ambiguous"]=result.ambiguous;report["unmatched"]=result.unmatched;
    report["line_rms_m"]=result.residual_rms;report["max_residual_m"]=result.max_residual;
    for(const auto & p:result.poses) {YAML::Node n;n["pose"]=node(p.pose);n["verified"]=p.verified;n["position_sigma"]=p.position_sigma;n["yaw_sigma"]=p.yaw_sigma;report["poses"].push_back(n);}
    for(const auto & w:result.walls) {YAML::Node n;n["id"]=w.wall.id;n["a"]=node(w.wall.a);n["b"]=node(w.wall.b);n["verified"]=w.verified;
      n["line_verified"]=w.line_verified;n["line_sigma"]=w.line_sigma;
      n["assembly"]=w.wall.assembly;
      n["edge_a"]=w.edge_a;n["edge_b"]=w.edge_b;n["views"]=w.views;n["max_displacement"]=w.max_displacement;n["position_sigma"]=w.position_sigma;report["walls"].push_back(n);}
    bool paths_ok=true;
    if(result.accepted) {
      const auto updated=updateMap(prior,walls,frames,result,settings);writeMap(updated.map,output);
      report["cleared_cells"]=updated.cleared;report["hit_cells"]=updated.hits;report["uncertain_cells"]=updated.uncertain_cells;
      int index=0;
      for(const auto & p:data["portals"]) {
        Portal portal;portal.first=p["first_wall"].as<std::string>();portal.second=p["second_wall"].as<std::string>();
        const auto first=p["first_endpoint"].as<std::string>(),second=p["second_endpoint"].as<std::string>();
        if((first!="a"&&first!="b")||(second!="a"&&second!="b")) {throw std::runtime_error("portal endpoints must be a or b");}
        portal.first_b=first=="b";portal.second_b=second=="b";portal.heading=p["heading"].as<double>();
        portal.approach=p["approach"].as<double>();portal.exit=p["exit"].as<double>();
        const auto path=checkedCrossing(portal,result,updated.map,footprint);YAML::Node n;n["accepted"]=path.accepted;n["reason"]=path.reason;n["width_m"]=path.width;
        if(path.accepted) {
          const auto filename="portal_"+std::to_string(index)+".path";n["path_file"]=filename;
          std::ostringstream text;text<<std::setprecision(17);for(const auto & pose:path.path) {text<<pose.x<<' '<<pose.y<<' '<<pose.yaw<<'\n';}
          writeNew((output/filename).string(),text.str());
        } else {paths_ok=false;}
        report["portals"].push_back(n);++index;
      }
      report["initial_pose_radians"]=node(result.poses.back().pose);
      const auto p=result.poses.back().pose;std::ostringstream initial;initial<<std::setprecision(17)<<p.x<<','<<p.y<<','<<p.yaw*180/3.14159265358979323846;
      report["initial_pose_launch"]=initial.str();
    }
    report["survey_elapsed_seconds"]=frames.back().stamp-frames.front().stamp;
    report["processing_seconds"]=std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count();
    writeNew((output/"report.yaml").string(),YAML::Dump(report)+"\n");
    std::cout<<result.reason<<"; matched="<<result.matched<<"; rank="<<result.geometric_rank<<'/'<<result.variables
      <<"; portals="<<(paths_ok?"accepted_or_not_requested":"rejected")<<"; report="<<(output/"report.yaml")<<'\n';
    return result.accepted&&paths_ok?0:3;
  } catch(const std::exception & e) {std::cerr<<"adapt_layout: "<<e.what()<<'\n';return 2;}
}
