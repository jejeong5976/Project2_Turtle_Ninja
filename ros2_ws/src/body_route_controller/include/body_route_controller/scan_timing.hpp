#pragma once
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
namespace body_route_controller {
struct ScanInterval {
  double oldest=0;
  std::string error;
};
// LaserScan stamps its first array element, which need not be the oldest ray.
// Never change a sensor timestamp here. Both acquisition directions must pass
// the same future-time and oldest-ray freshness checks before any TF lookup.
inline ScanInterval scanInterval(double first,double increment,std::size_t count,
  double received,double timeout) {
  if(count<2||count>10000||!std::isfinite(first)||!std::isfinite(increment)||
    !std::isfinite(received)||!std::isfinite(timeout)||timeout<=0||
    std::abs(increment)*count>.2)return {0,"invalid_scan_metadata"};
  const double last=first+(count-1)*increment;
  const double oldest=std::min(first,last),newest=std::max(first,last);
  if(oldest<=0||received-oldest>timeout)return {oldest,"invalid_or_stale_stamp"};
  if(newest>received)return {oldest,"future_ray_stamp"};
  return {oldest,{}};
}
} // namespace body_route_controller
