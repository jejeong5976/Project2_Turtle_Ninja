#include "mapping_scan.hpp"
#include <functional>
#include <iostream>
using namespace lidar_mapping;
void check(bool v){if(!v)throw std::runtime_error("Assertion failed");}
void rejects(const std::function<void()> &f){bool bad=false;try{f();}catch(const std::exception &){bad=true;}check(bad);}
int main(){
  Policy p;p.bins=400;
  auto s=resample({{0,{0,0},{1,0}}},p);check(s.bins[0].range==1&&s.bins[0].endpoint_error==0&&std::isnan(s.bins[1].range));
  s=resample({{1,{0,0},{2,0}},{0,{0,0},{1,0}}},p);check(s.collisions==1&&s.bins[0].range==1&&s.bins[0].input_bin==0);
  // Foreground rejected by origin gate must not reveal farther background.
  s=resample({{0,{.006,0},{1,0}},{1,{0,0},{2,0}}},p);check(std::isnan(s.bins[0].range)&&s.approximation_rejected==1);
  s=resample({{0,{0,0},{1,-.001}}},p);check(s.bins[0].input_bin==0); // Seam.
  s=resample({},p);for(const auto &b:s.bins)check(std::isnan(b.range));
  s=resample({{0,{0,0},{.01,0}},{1,{0,0},{13,0}}},p);check(s.range_rejected==2);
  rejects([&]{resample({{0,{0,0},{1,0}},{0,{0,0},{2,0}}},p);});
  rejects([&]{resample({{0,{NAN,0},{1,0}}},p);});
  const Ray r{0,{.002,-.001},{5,.021}};s=resample({r},p);
  const auto b=s.bins[0];check(b.input_bin==0);
  const auto q=endpoint(b.range,0,p.bins);
  for(int i=0;i<=100;++i){const double f=i/100.;const Point physical{(1-f)*r.origin.x+f*r.endpoint.x,(1-f)*r.origin.y+f*r.endpoint.y};check(separation(physical,{f*q.x,f*q.y})<=b.path_bound+1e-12);}
  p.bins=1600;s=resample({r},p);double fine_error=1;for(const auto &v:s.bins)if(v.input_bin==0)fine_error=v.endpoint_error;check(fine_error<b.endpoint_error);
  const double tie=2*std::acos(-1.)*202.5/1600;
  for(const auto offset:{-1e-12,0.,1e-12}){s=resample({{0,{0,0},{std::cos(tie+offset),std::sin(tie+offset)}}},p);check(s.bins[203].input_bin==0);}
  std::cout<<"PASS mapping seam, unknowns, collisions, rejected foreground, range/invalid input, ray-path bound, finer-grid error\n";
}
