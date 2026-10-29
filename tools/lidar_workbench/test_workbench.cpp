#include "ros_helpers.hpp"
#include <cassert>
#include <iostream>

template<class F> void rejects(F f) {bool rejected=false;try{f();}catch(const std::exception &){rejected=true;}assert(rejected);}
int main() {
  using namespace workbench;
  Options o; o.validate();
  rejects([]{parse_options({"--speed","nan"});});
  rejects([]{parse_options({"--scan-bins","400.5"});});
  rejects([]{parse_options({"--speed","0.11","--max-linear","0.11"});});
  rejects([]{parse_options({"--hardware","opencr"});});
  rejects([]{parse_options({"--hardware","opencr","--execute","--drive-port","/a","--front-port","/b","--rear-port","/c","--script","a"});});
  rejects([&]{parse_segment("forward 0.1\nleft 0.1",o);});
  rejects([&]{parse_segment("forward -0.1",o);});
  for(const auto &name:{"forward","backward","left","right","cw","ccw"})assert(parse_segment(std::string(name)+" 0.1",o).speed>0);
  assert(parse_segment("left 0.1",o).y==1);
  assert(parse_segment("cw 10",o).yaw==-1);
  assert(body({.1376,.1625})&&!body({.1377,.1625}));
  // A sensor ray transforms its real origin as well as its endpoint.
  const Ray ray{{0,0},{1,0},0,10,-.01};
  const auto transformed=transform(ray,mount(0),-.02);
  assert(std::abs(transformed.origin.x-.110)<1e-12);
  assert(std::abs(transformed.endpoint.x+.890)<1e-12);
  assert(std::abs(transformed.acquisition_offset+.03)<1e-12);
  const auto roundtrip=decode_cloud(cloud({ray},1000000000,"lidar_front_link"),0);
  assert(roundtrip.size()==1&&roundtrip[0].input==10);
  rejects([&]{decode_cloud(cloud({ray},1000000000,"lidar_front_link"),1);});
  // Spatially isolated nearby returns remain; only distant isolated returns are flagged.
  const std::vector<Ray> points{{{0,0},{.1,0},0,0,0},{{0,0},{1,0},0,1,0},{{0,0},{1.02,0},1,1,0},{{0,0},{2,0},1,2,0}};
  assert(spatial_filter(points,o).kept.size()==4);
  o.neighbors=1;const auto selected=spatial_filter(points,o);
  assert(selected.kept.size()==3&&selected.rejected.size()==1&&selected.rejected[0].endpoint.x==2);
  // Analytic room geometry independently checks the synthetic sensor.
  assert(std::abs(scene_range({0,0},0)-4)<1e-12);
  assert(std::abs(scene_range({0,0},std::acos(-1.))+0-2)<1e-12);
  std::vector<lidar_motion::Odom> odom;
  for(int i=0;i<=10;++i)odom.push_back({1000000000+i*10000000,{.025*i*.01,0,0}});
  validate_motion(odom,o);
  auto bad=odom;bad[5].stamp_ns=bad[4].stamp_ns;rejects([&]{validate_motion(bad,o);});
  bad=odom;bad[5].pose.y=.1;rejects([&]{validate_motion(bad,o);});
  lidar_mapping::TimedFrame frame{1000000000,1100000000,{{{0,0,1,0},1000000000},{{1,1,1,.1},1100000000}}};
  const auto result=prepare(frame,odom,1,o);
  assert(result.corrected.size()==2&&result.raw.size()==2);
  assert(std::abs(result.corrected[0].endpoint.x-.9975)<1e-12);
  assert(std::abs(result.corrected[0].origin.x+.0025)<1e-12);
  assert(result.scan.bins.size()==1600);
  std::cout<<"Workbench geometry, filtering, cloud schema, motion bounds and CLI checks passed\n";
}
