#include "mapping_frame.hpp"
#include <functional>
#include <iostream>
using namespace lidar_mapping;
void check(bool v){if(!v)throw std::runtime_error("Mapping frame assertion failed");}
void rejects(const std::function<void()> &f){bool bad=false;try{f();}catch(const std::exception &){bad=true;}check(bad);}
int main(){
  Frame f;f.id=1;f.start=1000000000;f.end=1100000000;f.count=400;f.source_valid=true;f.clock={1100001000,11100001000,1100001100};
  f.beams={{0,0,1.,0.},{1,399,.1,0.}};const auto t=time_frame(f,"nominal",0);
  check(t.reference-t.first==99750000); // Masked last beam still owns reference.
  std::vector<lidar_motion::Odom> odom;
  for(int i=0;i<=6;++i)odom.push_back({t.first-10000000+i*20000000,{0,0,0}});
  auto s=project_frame(t,odom);check(s.bins[0].input_bin==0);size_t kept=0;for(const auto &b:s.bins)kept+=b.input_bin>=0;check(kept==1);
  auto bad=f;bad.source_valid=false;rejects([&]{time_frame(bad,"nominal",0);});
  bad=f;bad.beams[1].source=400;rejects([&]{time_frame(bad,"nominal",0);});
  rejects([&]{time_frame(f,"nominal",-1);});rejects([&]{time_frame(f,"uniform",0);});
  bad=f;bad.clock.before_ns=f.end-1;rejects([&]{time_frame(bad,"nominal",0);});
  auto missing=odom;missing.erase(missing.begin(),missing.begin()+2);rejects([&]{project_frame(t,missing);});
  auto gap=odom;gap.erase(gap.begin()+1,gap.begin()+3);rejects([&]{project_frame(t,gap);});
  auto fast=odom;fast[2].pose.x=.01;rejects([&]{project_frame(t,fast);});
  fast=odom;fast[2].pose.yaw=.01;rejects([&]{project_frame(t,fast);});
  std::cout<<"PASS live-frame reference, body exclusion, provenance, timing, coverage/gap and motion-envelope rejection\n";
}
