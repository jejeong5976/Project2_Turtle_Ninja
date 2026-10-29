#pragma once
// Shared offline/live resampling core. No sensor, ROS, TF, or command ownership.
#include "motion_model.hpp"
#include <limits>
#include <set>

namespace lidar_mapping {
using lidar_motion::Point;
struct Ray {int input_bin;Point origin,endpoint;};
struct Policy {
  size_t bins=1600;
  double min_range=.05,max_range=12.,max_origin=.005,max_endpoint_error=.025;
};
struct Bin {
  double range=std::numeric_limits<double>::quiet_NaN();
  int input_bin=-1;
  double endpoint_error=0,origin_shift=0,path_bound=0;
};
struct Scan {
  std::vector<Bin> bins;
  size_t collisions=0,range_rejected=0,approximation_rejected=0;
};
inline bool finite(Point p) {return std::isfinite(p.x)&&std::isfinite(p.y);}
inline double separation(Point a,Point b) {return std::hypot(a.x-b.x,a.y-b.y);}
inline Point endpoint(double range,size_t bin,size_t bins) {
  const double angle=2*std::acos(-1.)*bin/bins;
  return {range*std::cos(angle),range*std::sin(angle)};
}
inline Scan resample(const std::vector<Ray> &rays,const Policy &policy) {
  if(policy.bins<400||policy.bins>6400||!std::isfinite(policy.min_range)||
      !std::isfinite(policy.max_range)||!std::isfinite(policy.max_origin)||
      !std::isfinite(policy.max_endpoint_error)||policy.min_range<=0||
      policy.max_range<=policy.min_range||policy.max_origin<=0||policy.max_endpoint_error<=0)
    throw std::invalid_argument("Invalid mapping policy");
  Scan result{std::vector<Bin>(policy.bins)};
  std::vector<const Ray*> selected(policy.bins,nullptr);std::set<int> seen;
  for(const auto &ray:rays) {
    if(ray.input_bin<0||ray.input_bin>=400||!seen.insert(ray.input_bin).second||
       !finite(ray.origin)||!finite(ray.endpoint))throw std::invalid_argument("Invalid/duplicate corrected ray");
    const auto range=std::hypot(ray.endpoint.x,ray.endpoint.y);
    // Out-of-range returns are unknown, never synthesized max-range rays.
    if(range<policy.min_range||range>policy.max_range){++result.range_rejected;continue;}
    double angle=std::atan2(ray.endpoint.y,ray.endpoint.x);
    if(angle<0)angle+=2*std::acos(-1.);
    double coordinate=angle*policy.bins/(2*std::acos(-1.))+.5;
    // Decoded angles can land exactly on half-bin boundaries. Stabilize ties
    // against CSV decimal roundoff (<1e-8 of a bin); ties choose the upper bin.
    const double nearest=std::round(coordinate);
    if(std::abs(coordinate-nearest)<1e-8)coordinate=nearest;
    const size_t index=size_t(std::floor(coordinate))%policy.bins;
    auto &bin=result.bins[index];
    if(selected[index])++result.collisions;
    // Select nearest BEFORE approximation rejection. A rejected foreground ray
    // must not expose a farther return sharing its output bin. Ties use input ID.
    if(!selected[index]||range<bin.range||(range==bin.range&&ray.input_bin<bin.input_bin)) {
      selected[index]=&ray;bin.range=range;bin.input_bin=ray.input_bin;
    }
  }
  for(size_t i=0;i<policy.bins;++i) {
    if(!selected[i])continue;
    auto &bin=result.bins[i];const auto &ray=*selected[i];
    bin.endpoint_error=separation(endpoint(bin.range,i,policy.bins),ray.endpoint);
    bin.origin_shift=std::hypot(ray.origin.x,ray.origin.y);
    // For equal fractional positions along both straight rays, the difference
    // is a convex combination of the two endpoint differences. It is bounded
    // by this maximum. This is geometry under the timing model, not free-space
    // visibility equivalence or an uncertainty bound on the real measurement.
    bin.path_bound=std::max(bin.endpoint_error,bin.origin_shift);
    if(bin.endpoint_error>policy.max_endpoint_error||bin.origin_shift>policy.max_origin) {
      ++result.approximation_rejected;bin.range=std::numeric_limits<double>::quiet_NaN();bin.input_bin=-1;
    }
  }
  return result;
}
} // namespace lidar_mapping
