#pragma once
#include "host_time_anchor.hpp"
#include <algorithm>
#include <cmath>
#include <vector>

namespace lidar_motion {
struct Pose {double x, y, yaw;};
struct Odom {int64_t stamp_ns; Pose pose;};
struct Point {double x, y;};
inline bool finite(Pose p) {
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.yaw);
}
inline double wrap(double a) {return std::remainder(a,2*std::acos(-1.));}
inline Point apply(Pose p, Point v) {
  return {p.x+std::cos(p.yaw)*v.x-std::sin(p.yaw)*v.y,
          p.y+std::sin(p.yaw)*v.x+std::cos(p.yaw)*v.y};
}
inline Point inverse(Pose p, Point v) {
  const double x=v.x-p.x,y=v.y-p.y;
  return {std::cos(p.yaw)*x+std::sin(p.yaw)*y,
         -std::sin(p.yaw)*x+std::cos(p.yaw)*y};
}
// Ordered odom in one continuous odom frame. Never extrapolate, reorder clock
// regressions, bridge a reset, or substitute the latest pose for the beam time.
class Trajectory {
  std::vector<Odom> samples_;
  int64_t max_gap_ns_;
public:
  explicit Trajectory(std::vector<Odom> samples,int64_t max_gap_ns=100000000)
      :samples_(std::move(samples)),max_gap_ns_(max_gap_ns) {
    if(samples_.size()<2 || max_gap_ns<=0) throw std::invalid_argument("Insufficient odometry");
    for(size_t i=0;i<samples_.size();++i) {
      const auto &s=samples_[i];
      if(s.stamp_ns<0 || !finite(s.pose)) throw std::invalid_argument("Invalid odometry");
      if(i && s.stamp_ns<=samples_[i-1].stamp_ns)
        throw std::invalid_argument("Repeated/regressing odometry timestamp");
      if(i) {
        const auto &p=samples_[i-1];
        const double dt=lidar_timing::checked_sub(s.stamp_ns,p.stamp_ns)/1e9;
        if(std::hypot(s.pose.x-p.pose.x,s.pose.y-p.pose.y)>0.5*dt ||
           std::abs(wrap(s.pose.yaw-p.pose.yaw))>2.0*dt)
          throw std::invalid_argument("Odometry discontinuity or motion beyond experiment limits");
      }
    }
  }
  Pose at(int64_t stamp) const {
    auto hi=std::lower_bound(samples_.begin(),samples_.end(),stamp,
      [](const Odom &s,int64_t t){return s.stamp_ns<t;});
    if(hi!=samples_.end() && hi->stamp_ns==stamp) return hi->pose;
    if(hi==samples_.begin() || hi==samples_.end()) throw std::invalid_argument("Odometry does not bracket beam/reference time");
    const auto &lo=*(hi-1);
    const auto gap=lidar_timing::checked_sub(hi->stamp_ns,lo.stamp_ns);
    if(gap>max_gap_ns_) throw std::invalid_argument("Odometry gap exceeds limit");
    const double dt=gap/1e9;
    // Generous relative to our 0.10 m/s, 0.5 rad/s profile; rejects gross resets.
    if(std::hypot(hi->pose.x-lo.pose.x,hi->pose.y-lo.pose.y)>0.5*dt ||
       std::abs(wrap(hi->pose.yaw-lo.pose.yaw))>2.0*dt)
      throw std::invalid_argument("Odometry discontinuity or motion beyond experiment limits");
    const double f=double(lidar_timing::checked_sub(stamp,lo.stamp_ns))/gap;
    return {lo.pose.x+f*(hi->pose.x-lo.pose.x),lo.pose.y+f*(hi->pose.y-lo.pose.y),
            wrap(lo.pose.yaw+f*wrap(hi->pose.yaw-lo.pose.yaw))};
  }
};
inline int64_t estimate_time(lidar_timing::HostEnvelope host,int64_t delay_ns) {
  if(delay_ns<0 || delay_ns>1000000000 || host.lower_ns<0 || host.upper_ns<host.lower_ns)
    throw std::invalid_argument("Invalid host envelope or assumed delay");
  const auto middle=lidar_timing::checked_add(host.lower_ns,
    lidar_timing::checked_sub(host.upper_ns,host.lower_ns)/2);
  const auto result=lidar_timing::checked_sub(middle,delay_ns);
  if(result<0) throw std::invalid_argument("Estimated acquisition time is negative");
  return result;
}
struct CorrectedRay {Point origin, endpoint;};
// Returns both ray ends in front sensor coordinates at reference time. During
// translation/rotation these origins generally differ: do not silently cast
// all corrected endpoints as rays from a synthetic common origin.
inline CorrectedRay correct(const Trajectory &odom,int64_t beam_ns,int64_t reference_ns,
                            Pose sensor_in_base,double range,double angle) {
  if(!finite(sensor_in_base) || !std::isfinite(range) || range<=0 || !std::isfinite(angle))
    throw std::invalid_argument("Invalid ray or mounting pose");
  const Pose at_beam=odom.at(beam_ns),at_reference=odom.at(reference_ns);
  const auto transform=[&](Point p) {
    return inverse(sensor_in_base,inverse(at_reference,apply(at_beam,apply(sensor_in_base,p))));
  };
  return {transform({0,0}),transform({range*std::cos(angle),range*std::sin(angle)})};
}
} // namespace lidar_motion
