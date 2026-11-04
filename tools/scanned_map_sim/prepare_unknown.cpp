#include "common.hpp"
#include <iostream>
using namespace maze_sim;
int main(int argc,char **argv){try{
  if(argc!=3)throw std::runtime_error("prepare_unknown NEW_MAP_DIRECTORY corridor|concealed|partial|turn");
  const fs::path out=argv[1];const std::string kind=argv[2];
  if(kind!="corridor"&&kind!="concealed"&&kind!="partial"&&kind!="turn")throw std::runtime_error("invalid unknown scenario");
  if(!fs::create_directory(out))throw std::runtime_error("maps directory exists");
  Grid truth;truth.resolution=.01;truth.ox=0;truth.oy=0;truth.pixels=cv::Mat(280,440,CV_8U,cv::Scalar(254));
  auto box=[&](Grid &g,double x0,double y0,double x1,double y1,int value){cv::rectangle(g.pixels,g.pixel(x0,y1),g.pixel(x1,y0),cv::Scalar(value),-1);};
  box(truth,0,0,4.39,.04,0);box(truth,0,2.75,4.39,2.79,0);box(truth,0,0,.04,2.79,0);box(truth,4.35,0,4.39,2.79,0);
  // Non-symmetric known localization anchors behind the fixed start.
  box(truth,.04,.04,.35,.45,0);box(truth,.04,2.35,.22,2.75,0);
  if(kind=="concealed")box(truth,1.85,1.18,2.05,1.62,0);
  if(kind=="turn")box(truth,2.15,.04,2.25,1.1,0);
  auto nav=truth.clone();box(nav,1.2,.05,4.34,2.74,205);
  truth.save(out,"truth");nav.save(out,"navigation");nav.save(out,"nominal");
  Pose start{.7,1.4,0},goal=kind=="turn"?Pose{1.8,2.1,pi/2}:Pose{2.7,1.4,0};
  YAML::Node cfg;cfg["schema_version"]=2;cfg["scenario"]=kind;cfg["start"]=poseNode(start);cfg["goals"].push_back(poseNode(goal));
  cfg["view_bounds"]=YAML::Load("[0.0, 0.0, 4.39, 2.79]");cfg["route"]="unknown-"+kind;cfg["map_mode"]="partial";
  cfg["matched_map_is_oracle"]=false;cfg["ground_truth_complete"]=true;cfg["hidden_truth_usage"]="sensor simulation and independent evaluation only";
  cfg["shift_bound_m"]=0.0;cfg["seed"]=0;cfg["sensor_range_m"]=3.0;
  // Partial-visibility interval is explicit sensor loss, never a hidden truth leak.
  cfg["partial_visibility"]=kind=="partial";
  saveYaml(out/"scenario.yaml",cfg);std::ofstream goals(out/"goals.path");goals<<goal.x<<' '<<goal.y<<' '<<goal.yaw<<'\n';
  Canvas a(nav,cfg["view_bounds"]),b(truth,cfg["view_bounds"]);a.text("Navigation input: purple unknown is provisional",22);b.text("Complete truth: sensor/evaluator ONLY",22);
  for(auto *c:{&a,&b}){c->body(start,{180,80,10});cv::circle(c->image,c->point(goal),5,{10,100,230},-1);}
  cv::Mat both;cv::hconcat(a.image,b.image,both);cv::imwrite((out/"layouts.png").string(),both);
  std::cout<<"Complete synthetic truth + separate partial navigation map: "<<kind<<'\n';return 0;
}catch(const std::exception &e){std::cerr<<e.what()<<'\n';return 2;}}
