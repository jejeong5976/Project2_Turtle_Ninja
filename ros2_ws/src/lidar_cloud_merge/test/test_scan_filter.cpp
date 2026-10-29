#include "lidar_cloud_merge/scan_filter.hpp"
#include <iostream>
void check(bool value) {if(!value) throw std::runtime_error("Scan filter test failed");}
int main() {
  using lidar_cloud_merge::filter_front_scan;
  sensor_msgs::msg::LaserScan s;
  s.header.frame_id="lidar_front_link";s.header.stamp.sec=123;
  s.angle_min=0;s.angle_increment=.1f;s.scan_time=.1f;
  s.range_min=.05f;s.range_max=12;
  const float nan=std::numeric_limits<float>::quiet_NaN();
  s.ranges={.1f,1,nan,std::numeric_limits<float>::infinity(),0,13};
  s.angle_max=(s.ranges.size()-1)*s.angle_increment;
  s.intensities={10,20,30,40,50,60};
  auto out=filter_front_scan(s);
  check(std::isnan(out.ranges[0]) && out.ranges[1]==1 && out.intensities[1]==20);
  for(size_t i:{0,2,3,4,5}) check(std::isnan(out.ranges[i]) && out.intensities[i]==0);
  check(out.header==s.header && out.angle_min==s.angle_min && out.angle_max==s.angle_max &&
        out.angle_increment==s.angle_increment && out.scan_time==s.scan_time &&
        out.time_increment==s.time_increment && out.range_min==s.range_min &&
        out.range_max==s.range_max && out.ranges.size()==s.ranges.size());
  check(s.ranges[0]==.1f && s.intensities[0]==10); // Raw input never mutated.
  auto boundary=s;boundary.ranges={.2476f};boundary.intensities.clear();boundary.angle_max=0;
  check(std::isnan(filter_front_scan(boundary).ranges[0]));
  boundary.ranges={.2477f};check(std::isfinite(filter_front_scan(boundary).ranges[0]));
  for(int fault=0;fault<5;++fault) {
    auto bad=s;
    if(fault==0)bad.header.frame_id="lidar_rear_link";
    if(fault==1)bad.angle_increment=nan;
    if(fault==2)bad.intensities.pop_back();
    if(fault==3)bad.angle_max+=.01f;
    if(fault==4)bad.scan_time=0;
    bool rejected=false;try{filter_front_scan(bad);}catch(const std::runtime_error&){rejected=true;}
    check(rejected);
  }
  std::cout<<"PASS: body boundary, unknown rays, metadata, immutability, invalid inputs\n";
}
