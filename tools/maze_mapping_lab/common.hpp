#pragma once
#include <nlohmann/json.hpp>
#include <mujoco/mujoco.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <queue>
#include <random>
#include <set>
#include <sstream>
#include <stdexcept>
#include <vector>
namespace maze {
using Json=nlohmann::json;
namespace fs=std::filesystem;
constexpr double pi=3.14159265358979323846;
inline void check(bool b,const std::string&s){if(!b)throw std::runtime_error(s);}
inline Json read_json(const fs::path&p){std::ifstream f(p);check(bool(f),"Cannot read "+p.string());Json j;f>>j;return j;}
inline void write_json(const fs::path&p,const Json&j){std::ofstream f(p);f<<j.dump(2)<<'\n';check(bool(f),"Cannot write "+p.string());}
inline std::string read_text(const fs::path&p){std::ifstream f(p);check(bool(f),"Cannot read "+p.string());return {std::istreambuf_iterator<char>(f),{}};}
inline std::string numbers(std::initializer_list<double>v){std::ostringstream s;s<<std::setprecision(12);for(auto x:v)s<<x<<' ';return s.str();}
inline double wrap(double a){return std::remainder(a,2*pi);}
struct Pose{double x=0,y=0,yaw=0;};
inline Pose combine(Pose a,Pose b){return{a.x+cos(a.yaw)*b.x-sin(a.yaw)*b.y,a.y+sin(a.yaw)*b.x+cos(a.yaw)*b.y,wrap(a.yaw+b.yaw)};}
inline Pose relative(Pose a,Pose b){double x=b.x-a.x,y=b.y-a.y;return{cos(a.yaw)*x+sin(a.yaw)*y,-sin(a.yaw)*x+cos(a.yaw)*y,wrap(b.yaw-a.yaw)};}
inline Pose interp(Pose a,Pose b,double f){return{a.x+(b.x-a.x)*f,a.y+(b.y-a.y)*f,wrap(a.yaw+wrap(b.yaw-a.yaw)*f)};}
struct Obstacle{std::string name;double x,y,z,hx,hy,hz;bool cylinder=false;};
inline std::vector<Obstacle> obstacles(const Json&j){
  std::vector<Obstacle> o;std::set<std::pair<double,double>>posts;int id=0;
  double h=j.at("wall_height"),t=j.at("wall_thickness"),r=j.at("post_radius");
  for(const auto&w:j.at("wall_segments")){
    double x1=w[0],y1=w[1],x2=w[2],y2=w[3];
    check(x1==x2||y1==y2,"Only axis-aligned wall segments supported");
    o.push_back({"wall_"+std::to_string(id++),(x1+x2)/2,(y1+y2)/2,h/2,std::max(t/2,std::abs(x2-x1)/2),std::max(t/2,std::abs(y2-y1)/2),h/2});
    int n=std::max(1,int(std::round(std::hypot(x2-x1,y2-y1)/.734)));
    if(x1==x2&&std::abs(y2-y1)>6)n=16; // Drawing shows 17 posts on each long boundary.
    for(int k=0;k<=n;++k)posts.insert({x1+(x2-x1)*k/n,y1+(y2-y1)*k/n});
  }
  for(auto [x,y]:posts)o.push_back({"post_"+std::to_string(id++),x,y,h/2,r,r,h/2,true});
  for(const auto&b:j.at("boxes"))o.push_back({b["name"],b["center"][0],b["center"][1],double(b["size"][2])/2,double(b["size"][0])/2,double(b["size"][1])/2,double(b["size"][2])/2});
  return o;
}
inline double clearance(double x,double y,const std::vector<Obstacle>&o){double d=100;for(const auto&b:o){double dx=std::abs(x-b.x)-b.hx,dy=std::abs(y-b.y)-b.hy;double c=b.cylinder?std::hypot(x-b.x,y-b.y)-b.hx:std::hypot(std::max(0.,dx),std::max(0.,dy))+std::min(0.,std::max(dx,dy));d=std::min(d,c);}return d;}
// Native binary dataset is local to this build/machine. JSON manifest carries schema/version/counts.
struct Sample{double t;Pose truth,odom;};
struct ScanRecord{double t;std::array<float,800> ranges;};
template<class T> inline void write_binary(const fs::path&p,const std::vector<T>&v){std::ofstream f(p,std::ios::binary);f.write(reinterpret_cast<const char*>(v.data()),v.size()*sizeof(T));check(bool(f),"Cannot write binary");}
template<class T> inline std::vector<T> read_binary(const fs::path&p){auto n=fs::file_size(p);check(n%sizeof(T)==0,"Truncated binary "+p.string());std::vector<T>v(n/sizeof(T));std::ifstream f(p,std::ios::binary);f.read(reinterpret_cast<char*>(v.data()),n);check(bool(f),"Cannot read binary");return v;}
inline Pose sample_at(const std::vector<Sample>&v,double t,bool odom=false){check(!v.empty(),"Empty trajectory");auto it=std::lower_bound(v.begin(),v.end(),t,[](const auto&s,double q){return s.t<q;});if(it==v.begin())return odom?it->odom:it->truth;if(it==v.end())return odom?v.back().odom:v.back().truth;const auto&a=*(it-1);return interp(odom?a.odom:a.truth,odom?it->odom:it->truth,(t-a.t)/(it->t-a.t));}
struct MjDeleter{void operator()(mjModel*m)const{mj_deleteModel(m);}void operator()(mjData*d)const{mj_deleteData(d);}};
using Model=std::unique_ptr<mjModel,MjDeleter>;using Data=std::unique_ptr<mjData,MjDeleter>;
inline Model load_model(const fs::path&p){char error[2048]{};Model m(mj_loadXML(p.c_str(),nullptr,error,sizeof(error)));check(bool(m),"MuJoCo: "+std::string(error));return m;}
inline std::map<std::string,std::string> args(int argc,char**argv,int start=2){std::map<std::string,std::string>r;for(int i=start;i<argc;i+=2){check(i+1<argc,"Missing CLI value");check(r.emplace(argv[i],argv[i+1]).second,"Duplicate CLI option");}return r;}
inline double number(const std::map<std::string,std::string>&a,const std::string&k,double d){auto it=a.find(k);if(it==a.end())return d;size_t n=0;double x=std::stod(it->second,&n);check(n==it->second.size()&&std::isfinite(x),"Invalid number "+k);return x;}
inline std::string option(const std::map<std::string,std::string>&a,const std::string&k,const std::string&d=""){auto it=a.find(k);return it==a.end()?d:it->second;}
} // namespace maze
