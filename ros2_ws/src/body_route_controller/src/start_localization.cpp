#include "body_route_controller/start_localization.hpp"
#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <stdexcept>

namespace body_route_controller::start_localization {
namespace {
constexpr double pi = 3.14159265358979323846;
namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;
using BPoint = bg::model::point<double, 2, bg::cs::cartesian>;
using Tree = bgi::rtree<BPoint, bgi::quadratic<16>>;
bool inside(const Point & p, const std::array<double, 4> & box, double margin=0) {
  return p[0]>=box[0]-margin && p[0]<=box[2]+margin &&
    p[1]>=box[1]-margin && p[1]<=box[3]+margin;
}
std::string token(std::istream & in) {
  std::string s;
  while(in>>s) {if(s[0]=='#') {std::getline(in,s);continue;} return s;}
  throw std::runtime_error("Truncated PGM header");
}
std::vector<double> distances(const Tree & tree, const std::vector<Sample> & points,
  const Pose & pose) {
  std::vector<double> values;
  std::vector<BPoint> nearest;
  for(const auto & p:points) {
    const auto q=transform(p.body,pose);
    nearest.clear();
    tree.query(bgi::nearest(BPoint(q[0],q[1]),1),std::back_inserter(nearest));
    values.push_back(bg::distance(BPoint(q[0],q[1]),nearest.at(0)));
  }
  return values;
}
double loss(const std::vector<double> & d) {
  double sum=0;
  for(const double x:d) sum+=std::pow(std::min(x,.10),2);
  return sum/d.size();
}
Metrics metrics(const Tree & tree,const std::vector<Sample> & p,const Pose & pose) {
  if(p.empty()) return {0,0,1,1};
  auto d=distances(tree,p,pose);
  std::sort(d.begin(),d.end());
  double span=0;
  for(const auto & a:p) for(const auto & b:p)
    span=std::max(span,std::hypot(a.body[0]-b.body[0],a.body[1]-b.body[1]));
  return {p.size(),span,d[d.size()/2],d[static_cast<std::size_t>(.9*(d.size()-1))]};
}
}  // namespace

Point transform(const Point & p,const Pose & pose) {
  const double c=std::cos(pose[2]),s=std::sin(pose[2]);
  return {pose[0]+c*p[0]-s*p[1],pose[1]+s*p[0]+c*p[1]};
}
Point inverse(const Point & p,const Pose & pose) {
  const double c=std::cos(pose[2]),s=std::sin(pose[2]);
  return {c*(p[0]-pose[0])+s*(p[1]-pose[1]),-s*(p[0]-pose[0])+c*(p[1]-pose[1])};
}
Config readConfig(const std::string & path) {
  const auto y=YAML::LoadFile(path);
  Config c{};
  c.reference=y["reference_pose"].as<Pose>();
  c.rear_mount=y["rear_mount_xy_yaw"].as<Pose>();
  for(double v:c.reference) if(!std::isfinite(v)) throw std::runtime_error("Nonfinite reference");
  for(double v:c.rear_mount) if(!std::isfinite(v)) throw std::runtime_error("Nonfinite rear mount");
  if(y["walls"].size()!=2) throw std::runtime_error("Require two reviewed fixed walls");
  for(std::size_t i=0;i<2;++i) {
    const auto w=y["walls"][i];
    c.walls[i]={w["name"].as<std::string>(),w["map_roi"].as<std::array<double,4>>(),
      w["body_sector_deg"].as<std::array<double,2>>()};
    const auto & b=c.walls[i].roi; const auto & a=c.walls[i].sector_deg;
    for(double v:b) if(!std::isfinite(v)) throw std::runtime_error("Invalid wall ROI");
    if(!(b[0]<b[2] && b[1]<b[3] && a[0]>=90 && a[0]<a[1] && a[1]<=270))
      throw std::runtime_error("Invalid ROI or sector; startup rays must point behind the body");
  }
  if(c.walls[0].name==c.walls[1].name ||
    std::max(c.walls[0].sector_deg[0],c.walls[1].sector_deg[0])<=
    std::min(c.walls[0].sector_deg[1],c.walls[1].sector_deg[1]))
    throw std::runtime_error("Wall names and sectors must be distinct");
  c.range_min=y["range_min"].as<double>(); c.range_max=y["range_max"].as<double>();
  c.association_margin=y["association_margin"].as<double>();
  c.min_points=y["min_points_per_wall_subset"].as<std::size_t>();
  c.min_span=y["min_span_per_wall_m"].as<double>();
  c.max_p90=y["max_heldout_p90_m"].as<double>();
  c.max_snapshot_distance=y["max_snapshot_difference_m"].as<double>();
  c.max_snapshot_yaw=y["max_snapshot_difference_deg"].as<double>()*pi/180;
  if(!(c.range_min>0 && c.range_min<c.range_max && c.range_max<=3 &&
    c.association_margin>0 && c.association_margin<=.35 && c.min_points>=5 &&
    c.min_span>=.20 && c.min_span<=2 && c.max_p90>0 && c.max_p90<=.05 &&
    c.max_snapshot_distance>0 && c.max_snapshot_distance<=.03 &&
    c.max_snapshot_yaw>0 && c.max_snapshot_yaw<=2*pi/180))
    throw std::runtime_error("Invalid start-localization limits");
  return c;
}
int select(const Point & body,const Point & origin,const Config & c) {
  const double x=body[0]-origin[0],y=body[1]-origin[1],r=std::hypot(x,y);
  if(!std::isfinite(r) || r<c.range_min || r>c.range_max) return -1;
  double a=std::atan2(y,x)*180/pi; if(a<0) a+=360;
  const auto p=transform(body,c.reference);
  for(std::size_t i=0;i<2;++i)
    if(a>=c.walls[i].sector_deg[0] && a<=c.walls[i].sector_deg[1] &&
      inside(p,c.walls[i].roi,c.association_margin)) return static_cast<int>(i);
  return -1;
}
std::array<std::vector<Point>,2> readMap(const std::string & file,const Config & c) {
  const auto y=YAML::LoadFile(file);
  const auto origin=y["origin"].as<Pose>();
  const double res=y["resolution"].as<double>(),occ=y["occupied_thresh"].as<double>();
  if(origin[2]!=0 || y["negate"].as<int>()!=0 || !(res>0 && res<=.1 && occ>0 && occ<1))
    throw std::runtime_error("Unsupported map metadata");
  auto image=std::filesystem::path(y["image"].as<std::string>());
  if(image.is_relative()) image=std::filesystem::path(file).parent_path()/image;
  std::ifstream in(image,std::ios::binary);
  if(token(in)!="P5") throw std::runtime_error("Require P5 map");
  const int w=std::stoi(token(in)),h=std::stoi(token(in));
  if(w<=0 || h<=0 || w>10000 || h>10000 || token(in)!="255")
    throw std::runtime_error("Invalid PGM dimensions/type");
  in.get(); std::vector<unsigned char> pixels(static_cast<std::size_t>(w)*h);
  in.read(reinterpret_cast<char *>(pixels.data()),static_cast<std::streamsize>(pixels.size()));
  if(!in) throw std::runtime_error("Truncated PGM");
  std::array<std::vector<Point>,2> map;
  for(int row=0;row<h;++row) for(int col=0;col<w;++col) {
    if((255-pixels[static_cast<std::size_t>(row)*w+col])/255.0<=occ) continue;
    const Point p{origin[0]+(col+.5)*res,origin[1]+(h-1-row+.5)*res};
    for(std::size_t i=0;i<2;++i) if(inside(p,c.walls[i].roi)) map[i].push_back(p);
  }
  for(const auto & points:map) if(points.size()<20) throw std::runtime_error("Insufficient occupied fixed-wall cells");
  return map;
}
Result fit(const std::array<std::vector<Point>,2> & map,
  const std::vector<Sample> & samples,const Config & c) {
  std::array<Tree,2> trees;
  std::array<std::vector<Sample>,2> train,heldout;
  for(std::size_t i=0;i<2;++i) {
    if(map[i].empty()) throw std::runtime_error("Empty fixed-wall map");
    for(const auto & p:map[i]) trees[i].insert(BPoint(p[0],p[1]));
  }
  for(const auto & p:samples) {
    if(p.wall>=2 || !std::isfinite(p.body[0]) || !std::isfinite(p.body[1]))
      throw std::runtime_error("Invalid selected sample");
    (p.index%2==0?train:heldout)[p.wall].push_back(p);
  }
  for(std::size_t i=0;i<2;++i)
    if(train[i].size()<c.min_points || heldout[i].size()<c.min_points)
      throw std::runtime_error("Insufficient independent training/validation support on both fixed walls");
  const auto score=[&](const Pose & p) {
    // Equal wall weights: a densely sampled wall must not dominate the other.
    return .5*(loss(distances(trees[0],train[0],p))+loss(distances(trees[1],train[1],p)));
  };
  Pose best=c.reference;
  double best_score=score(best);
  const auto consider=[&](const Pose & p) {
    if(std::abs(p[0]-c.reference[0])>.30+1e-9 || std::abs(p[1]-c.reference[1])>.30+1e-9 ||
      std::abs(p[2]-c.reference[2])>6*pi/180+1e-9) return;
    const double s=score(p); if(s<best_score) {best_score=s;best=p;}
  };
  for(int a=-12;a<=12;++a) for(int x=-15;x<=15;++x) for(int y=-15;y<=15;++y)
    consider({c.reference[0]+x*.02,c.reference[1]+y*.02,c.reference[2]+a*.5*pi/180});
  const auto center=best;
  for(int a=-6;a<=6;++a) for(int x=-8;x<=8;++x) for(int y=-8;y<=8;++y)
    consider({center[0]+x*.0025,center[1]+y*.0025,center[2]+a*.1*pi/180});
  Result result{c.reference,best,{},{},true};
  // A boundary solution is untrusted; retain it as diagnostic evidence only.
  if(std::abs(best[0]-c.reference[0])>=.295 || std::abs(best[1]-c.reference[1])>=.295 ||
    std::abs(best[2]-c.reference[2])>=5.9*pi/180) result.accepted=false;
  for(std::size_t i=0;i<2;++i) {
    result.train[i]=metrics(trees[i],train[i],best);
    result.heldout[i]=metrics(trees[i],heldout[i],best);
    if(result.train[i].span<c.min_span || result.heldout[i].span<c.min_span ||
      result.train[i].p90>c.max_p90 || result.heldout[i].p90>c.max_p90) result.accepted=false;
  }
  return result;
}
}  // namespace body_route_controller::start_localization
