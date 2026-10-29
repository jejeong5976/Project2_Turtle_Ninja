#pragma once
#include "mapping_scan.hpp"
#include <string>
namespace lidar_mapping {
struct Selected {int bin;int64_t source;double range,angle;};
struct Frame {
  int64_t id=0,start=0,end=0,count=0;
  lidar_timing::ClockPair clock{};
  bool source_valid=false;
  std::vector<Selected> beams;
};
struct TimedBeam {Selected beam;int64_t stamp;};
struct TimedFrame {int64_t first,reference;std::vector<TimedBeam> beams;};
inline TimedFrame time_frame(const Frame &frame,const std::string &model,int64_t delay) {
  if(!frame.source_valid||frame.id<0||frame.start<0||frame.end<=frame.start||frame.count<=0||frame.count>=800||
     frame.beams.empty()||frame.beams.size()>400||(model!="nominal"&&model!="scaled")||frame.end>frame.clock.before_ns)
    throw std::invalid_argument("Invalid mapping source frame");
  const auto anchor=lidar_timing::host_anchor(frame.start,frame.clock);
  const auto period=lidar_timing::checked_sub(frame.end,frame.start);
  if(period<50000000||period>200000000)throw std::invalid_argument("Mapping ring period outside bounds");
  TimedFrame out{INT64_MAX,0,{}};std::set<int> bins;std::set<int64_t> sources;
  for(const auto &b:frame.beams) {
    if(b.bin<0||b.bin>=400||!bins.insert(b.bin).second||b.source<0||b.source>=frame.count||!sources.insert(b.source).second||
       !std::isfinite(b.range)||b.range<.05||b.range>12||!std::isfinite(b.angle)||std::abs(b.angle)>2*std::acos(-1.)+1e-9)
      throw std::invalid_argument("Invalid selected mapping beam");
    const auto envelope=model=="nominal"?lidar_timing::shift_model(anchor,b.source*250000.):
      lidar_timing::shift_sample_fraction(anchor,b.source,period,frame.count);
    const auto stamp=lidar_motion::estimate_time(envelope,delay);
    out.first=std::min(out.first,stamp);out.reference=std::max(out.reference,stamp);out.beams.push_back({b,stamp});
  }
  return out;
}
inline Scan project_frame(const TimedFrame &frame,const std::vector<lidar_motion::Odom> &samples,const Policy &policy={}) {
  // Caller supplies a bracketed slice, including one sample on either side.
  for(size_t i=1;i<samples.size();++i) {
    const auto &a=samples[i-1],&b=samples[i];const double dt=lidar_timing::checked_sub(b.stamp_ns,a.stamp_ns)/1e9;
    if(dt<=0||std::hypot(b.pose.x-a.pose.x,b.pose.y-a.pose.y)>.04*dt||
       std::abs(lidar_motion::wrap(b.pose.yaw-a.pose.yaw))>.15*dt)
      throw std::invalid_argument("Motion exceeds provisional mapping input envelope");
  }
  const lidar_motion::Trajectory odom(samples,40000000);
  odom.at(frame.reference);std::vector<Ray> rays;
  for(const auto &b:frame.beams) {
    odom.at(b.stamp);const Point p{b.beam.range*std::cos(b.beam.angle),b.beam.range*std::sin(b.beam.angle)};
    if(p.x>=-.0276&&p.x<=.2476&&p.y>=-.1575&&p.y<=.1675)continue;
    const auto ray=lidar_motion::correct(odom,b.stamp,frame.reference,{.110,.005,std::acos(-1.)},b.beam.range,b.beam.angle);
    rays.push_back({b.beam.bin,ray.origin,ray.endpoint});
  }
  return resample(rays,policy);
}
} // namespace lidar_mapping
