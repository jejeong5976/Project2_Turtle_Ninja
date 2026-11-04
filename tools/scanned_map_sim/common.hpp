#pragma once
#include "body_route_controller/collision.hpp"
#include <yaml-cpp/yaml.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <cmath>
#include <cstdlib>
#include <vector>
namespace maze_sim {
namespace fs = std::filesystem;
using body_route_controller::Pose;
using body_route_controller::Footprint;
constexpr double pi=3.14159265358979323846;
inline Pose pose(const YAML::Node &n) {
  if(!n.IsSequence()||n.size()!=3) throw std::runtime_error("pose must be x y yaw radians");
  Pose p{n[0].as<double>(),n[1].as<double>(),n[2].as<double>()};
  if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.yaw)) throw std::runtime_error("non-finite pose");
  return p;
}
inline YAML::Node poseNode(Pose p) {YAML::Node n; n.push_back(p.x);n.push_back(p.y);n.push_back(p.yaw);return n;}
inline Pose compose(Pose a,Pose b) {return {a.x+std::cos(a.yaw)*b.x-std::sin(a.yaw)*b.y,a.y+std::sin(a.yaw)*b.x+std::cos(a.yaw)*b.y,body_route_controller::wrap(a.yaw+b.yaw)};}
inline fs::path runRoot() {const char *p=std::getenv("SCANNED_SIM_RUN");if(!p||!*p)throw std::runtime_error("SCANNED_SIM_RUN required");return p;}
inline void saveYaml(const fs::path &p,const YAML::Node &n) {if(fs::exists(p))throw std::runtime_error("refusing overwrite: "+p.string());std::ofstream f(p);f<<std::setprecision(17)<<YAML::Dump(n)<<'\n';if(!f)throw std::runtime_error("write failed");}
inline Footprint footprint() {Footprint f;for(auto q:std::vector<std::pair<double,double>>{{-.18,-.18},{.18,-.18},{.18,.18},{-.18,.18}}){geometry_msgs::msg::Point p;p.x=q.first;p.y=q.second;f.push_back(p);}return f;}
struct Grid {
  cv::Mat pixels; double resolution=0,ox=0,oy=0;
  static Grid read(const fs::path &path) {
    const auto n=YAML::LoadFile(path.string());Grid g;
    if(n["negate"].as<int>(0)!=0||std::abs(n["origin"][2].as<double>())>1e-12)throw std::runtime_error("requires non-negated unrotated scanned map");
    g.resolution=n["resolution"].as<double>();g.ox=n["origin"][0].as<double>();g.oy=n["origin"][1].as<double>();
    fs::path image=n["image"].as<std::string>();if(image.is_relative())image=path.parent_path()/image;
    g.pixels=cv::imread(image.string(),cv::IMREAD_GRAYSCALE);
    if(g.pixels.empty()||!std::isfinite(g.resolution)||!std::isfinite(g.ox)||!std::isfinite(g.oy)||g.resolution<.005||g.resolution>.05||g.pixels.total()>10000000)throw std::runtime_error("invalid map dimensions/resolution");
    for(int r=0;r<g.pixels.rows;++r)for(int c=0;c<g.pixels.cols;++c) {
      const auto v=g.pixels.at<unsigned char>(r,c);
      if(v!=0&&v!=205&&v!=254)throw std::runtime_error("requires scanned 0/205/254 occupancy encoding");
    }
    return g;
  }
  Grid clone() const {Grid g=*this;g.pixels=pixels.clone();return g;}
  cv::Point pixel(double x,double y) const {return {static_cast<int>(std::floor((x-ox)/resolution)),pixels.rows-1-static_cast<int>(std::floor((y-oy)/resolution))};}
  Pose center(int r,int c) const {return {ox+(c+.5)*resolution,oy+(pixels.rows-r-.5)*resolution,0};}
  bool inside(cv::Point p) const {return p.x>=0&&p.y>=0&&p.x<pixels.cols&&p.y<pixels.rows;}
  bool contains(double x,double y) const {return std::isfinite(x)&&std::isfinite(y)&&x>=ox&&y>=oy&&x<ox+pixels.cols*resolution&&y<oy+pixels.rows*resolution;}
  int at(double x,double y) const {if(!contains(x,y))return -1;auto p=pixel(x,y);return inside(p)?pixels.at<unsigned char>(p):-1;}
  void save(const fs::path &directory,const std::string &stem) const {
    if(fs::exists(directory/(stem+".pgm"))||fs::exists(directory/(stem+".yaml")))throw std::runtime_error("map output exists");
    if(!cv::imwrite((directory/(stem+".pgm")).string(),pixels))throw std::runtime_error("map write failed");
    YAML::Node n;n["image"]=stem+".pgm";n["resolution"]=resolution;n["origin"]=poseNode({ox,oy,0});
    n["negate"]=0;n["occupied_thresh"]=.65;n["free_thresh"]=.196;n["mode"]="trinary";saveYaml(directory/(stem+".yaml"),n);
  }
  nav2_costmap_2d::Costmap2D costmap() const {
    nav2_costmap_2d::Costmap2D m(pixels.cols,pixels.rows,resolution,ox,oy,255);
    for(int r=0;r<pixels.rows;++r)for(int c=0;c<pixels.cols;++c){auto v=pixels.at<unsigned char>(r,c);m.setCost(c,pixels.rows-1-r,v==0?254:v==254?0:255);}
    return m;
  }
  // Stop at unknown: it cannot produce a valid obstacle hit or free-space ray.
  double ray(double x,double y,double angle,double maximum=8) const {
    const double c=std::cos(angle),s=std::sin(angle);
    for(double d=.05;d<maximum;d+=resolution*.5){int v=at(x+d*c,y+d*s);if(v<0||v==205)return std::numeric_limits<double>::quiet_NaN();if(v==0)return d;}return maximum;
  }
};
struct Canvas {
  const Grid &map;cv::Rect crop;cv::Mat image;double scale=1.6;int top=90;
  Canvas(const Grid &g,const YAML::Node &bounds):map(g) {
    auto a=g.pixel(bounds[0].as<double>(),bounds[3].as<double>()),b=g.pixel(bounds[2].as<double>(),bounds[1].as<double>());
    crop=cv::Rect(a,b)&cv::Rect(0,0,g.pixels.cols,g.pixels.rows);
    scale=std::max(scale,900.0/crop.width);
    cv::Mat rgb;cv::cvtColor(g.pixels(crop),rgb,cv::COLOR_GRAY2BGR);for(int y=0;y<rgb.rows;++y)for(int x=0;x<rgb.cols;++x)if(g.pixels.at<unsigned char>(y+crop.y,x+crop.x)==205)
      rgb.at<cv::Vec3b>(y,x)=cv::Vec3b(240,215,240);
    cv::resize(rgb,rgb,{},scale,scale,cv::INTER_NEAREST);
    image=cv::Mat(rgb.rows+top+45,rgb.cols,CV_8UC3,cv::Scalar(255,255,255));rgb.copyTo(image(cv::Rect(0,top,rgb.cols,rgb.rows)));
  }
  cv::Point point(Pose p) const {auto q=map.pixel(p.x,p.y);return {static_cast<int>((q.x-crop.x)*scale),top+static_cast<int>((q.y-crop.y)*scale)};}
  void text(const std::string &s,int y,cv::Scalar color={20,20,20}) {cv::putText(image,s,{12,y},cv::FONT_HERSHEY_SIMPLEX,.48,color,1,cv::LINE_AA);}
  void path(const std::vector<Pose> &p,cv::Scalar color,int thickness=2) {for(std::size_t i=1;i<p.size();++i)cv::line(image,point(p[i-1]),point(p[i]),color,thickness,cv::LINE_AA);}
  void body(Pose p,cv::Scalar color) {
    auto f=footprint();std::vector<cv::Point> v;for(auto q:f)v.push_back(point(compose(p,{q.x,q.y,0})));
    cv::Mat overlay=image.clone();cv::fillConvexPoly(overlay,v,color,cv::LINE_AA);cv::polylines(overlay,v,true,color,2,cv::LINE_AA);
    cv::addWeighted(overlay,.4,image,.6,0,image);
  }
  void observations(const fs::path &root) {
    auto state=cv::imread((root/"observed-space.pgm").string(),cv::IMREAD_GRAYSCALE);
    if(state.empty()||state.size()!=map.pixels.size())return;
    for(int y=top;y<image.rows-45;++y)for(int x=0;x<image.cols;++x){const int v=state.at<unsigned char>(crop.y+int((y-top)/scale),crop.x+int(x/scale));
      if(v!=100&&v!=0)continue;
      const cv::Vec3b color=v==100?cv::Vec3b(210,210,20):cv::Vec3b(0,0,0);auto &p=image.at<cv::Vec3b>(y,x);
      for(int k=0;k<3;++k)p[k]=cv::saturate_cast<unsigned char>(.7*p[k]+.3*color[k]);
    }
  }
  void inflation(const fs::path &root) {
    const auto metadata=root/"global-costmap.yaml",raw=root/"global-costmap-raw.pgm";
    if(!fs::exists(metadata)||!fs::exists(raw))return;
    auto cfg=YAML::LoadFile(metadata.string());if(cfg["frame"].as<std::string>()!="map")return;
    auto costs=cv::imread(raw.string(),cv::IMREAD_GRAYSCALE);auto origin=pose(cfg["origin"]);double res=cfg["resolution"].as<double>();
    if(costs.empty()||!std::isfinite(res)||res<=0)return;
    for(int y=top;y<image.rows-45;++y)for(int x=0;x<image.cols;++x){auto world=map.center(crop.y+int((y-top)/scale),crop.x+int(x/scale));
      int c=int(std::floor((world.x-origin.x)/res)),r=costs.rows-1-int(std::floor((world.y-origin.y)/res));
      if(c<0||r<0||c>=costs.cols||r>=costs.rows)continue;
      unsigned char v=costs.at<unsigned char>(r,c);if(v==0||v==255)continue;
      cv::Vec3b color=v>=253?cv::Vec3b(40,40,230):cv::Vec3b(220,255-v,v);
      auto &pixel=image.at<cv::Vec3b>(y,x);for(int k=0;k<3;++k)pixel[k]=cv::saturate_cast<unsigned char>(.75*pixel[k]+.25*color[k]);
    }
  }
  void save(const fs::path &p) {if(!cv::imwrite(p.string(),image))throw std::runtime_error("plot write failed");}
};
inline std::vector<std::string> split(const std::string &s) {std::istringstream row(s);std::vector<std::string> v;std::string f;while(std::getline(row,f,','))v.push_back(f);return v;}
}
