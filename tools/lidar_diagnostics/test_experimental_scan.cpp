#include "experimental_scan.hpp"
#include <cassert>
#include <iostream>
using namespace lidar_diagnostic;
int main() {
  node_info raw[6]{};
  const unsigned distances[]{49,50,150,12000,12001,0};
  for (unsigned i=0;i<6;++i) {
    raw[i].angle_q6_checkbit = (i*45*64 << 1) | 1;
    raw[i].distance_q2 = distances[i];
    raw[i].sync_quality = 100;
  }
  LaserScan scan{};
  experimental_scan(scan,raw,6,false,0.1);
  assert(scan.points.size()==400);
  assert(std::isnan(scan.points[0].range));
  assert(scan.points[350].range==0.05f);
  assert(scan.points[300].range==0.15f);
  assert(scan.points[250].range==12.0f);
  assert(std::isnan(scan.points[200].range));
  assert(scan.config.min_range==0.05f && scan.config.max_range==12.0f);
  assert(scan.config.time_increment==0 && scan.config.scan_time==0.1f);
  assert(scan.config.max_angle==399*scan.config.angle_increment);
  experimental_scan(scan,raw,6,true,0.1);
  assert(scan.points[50].range==0.05f);
  raw[0].distance_q2=1000;
  experimental_scan(scan,raw,6,false,0.1);
  assert(scan.points[0].range==1.0f);
  raw[1].sync_quality=65;
  experimental_scan(scan,raw,6,false,0.1);
  assert(std::isnan(scan.points[350].range));
  std::cout << "PASS: SDK adapter, 0.05/12 m boundaries, index zero, sign, timing metadata\n";
}
