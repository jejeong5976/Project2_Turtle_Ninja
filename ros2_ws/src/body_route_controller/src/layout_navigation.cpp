#include "body_route_controller/layout_navigation.hpp"
#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_costmap_2d/footprint.hpp>
#include <algorithm>
#include <cmath>
#include <limits>

namespace body_route_controller::layout
{
Crossing checkedCrossing(const Portal & portal,const Estimate & estimate,const Grid & grid,const Footprint & footprint)
{
  grid.validate();Crossing rejected;
  if(footprint.size()<3||estimate.poses.empty()) {rejected.reason="missing_footprint_or_pose";return rejected;}
  double low=std::numeric_limits<double>::infinity(),high=-low,radius=0;
  for(const auto & p:footprint) {
    if(!std::isfinite(p.x)||!std::isfinite(p.y)) {rejected.reason="invalid_footprint";return rejected;}
    low=std::min(low,p.y);high=std::max(high,p.y);
    radius=std::max(radius,std::hypot(p.x,p.y));
  }
  const auto & last=estimate.poses.back();
  const double uncertainty=3*(last.position_sigma+radius*last.yaw_sigma);
  if(!last.verified||!std::isfinite(uncertainty)) {rejected.reason="unverified_start_pose";return rejected;}
  Footprint guarded=footprint;nav2_costmap_2d::padFootprint(guarded,uncertainty);
  auto result=crossing(portal,estimate,high-low+2*uncertainty);
  if(!result.accepted) {return result;}
  nav2_costmap_2d::Costmap2D map(grid.width,grid.height,grid.resolution,grid.origin.x,grid.origin.y,255);
  for(int y=0;y<grid.height;++y) {for(int x=0;x<grid.width;++x) {
    const int c=grid.cells[y*grid.width+x];map.setCost(x,y,c<0?255:c==100?254:0);
  }}
  const auto waypoints=result.path;Pose current=estimate.poses.back().pose;
  result.path={current};
  for(const auto & end:waypoints) {
    const double length=std::hypot(end.x-current.x,end.y-current.y);
    if(length>5) {result.accepted=false;result.reason="approach_requires_global_replan";result.path.clear();return result;}
    const double heading=length>1e-8?std::atan2(end.y-current.y,end.x-current.x):end.yaw;
    if(!turnSafe(map,guarded,current,heading)) {
      result.accepted=false;result.reason="turn_sweep_blocked_or_unknown";result.path.clear();return result;
    }
    const Pose begin=current;const int count=std::max(1,static_cast<int>(std::ceil(length/(grid.resolution*.5))));
    for(int i=0;i<=count;++i) {
      const double t=double(i)/count;current={begin.x+t*(end.x-begin.x),begin.y+t*(end.y-begin.y),heading};
      if(!footprintSafe(map,guarded,current)) {
        result.accepted=false;result.reason="translation_sweep_blocked_or_unknown";result.path.clear();return result;
      }
      result.path.push_back(current);
    }
  }
  if(!turnSafe(map,guarded,current,waypoints.back().yaw)) {
    result.accepted=false;result.reason="exit_turn_blocked_or_unknown";result.path.clear();return result;
  }
  result.path.back().yaw=waypoints.back().yaw;result.reason="checked_snapshot_path";return result;
}
}
