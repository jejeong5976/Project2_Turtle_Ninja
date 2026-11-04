#include "body_route_controller/supervision.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
namespace body_route_controller
{
std::string TrackingGuard::check(const Pose & map, const Pose & odom,
  const Velocity & v, const std::vector<Point> & path, const std::string & mode, double heading, bool custom_mode)
{
  for (double value : {map.x,map.y,map.yaw,odom.x,odom.y,odom.yaw,v.x,v.y,v.yaw,heading}) {
    if (!std::isfinite(value)) {return "non_finite_tracking_state";}
  }
  if (path.empty()) {return "missing_route";}
  const double speed=std::hypot(v.x,v.y);
  if (speed>limits_.measured_speed || std::abs(v.yaw)>limits_.measured_yaw_rate) {return "measured_speed_limit";}
  double nearest=path.size()==1 ? std::hypot(map.x-path[0].x,map.y-path[0].y) : std::numeric_limits<double>::infinity();
  if(!std::isfinite(path[0].x)||!std::isfinite(path[0].y)) {return "invalid_route";}
  for (std::size_t i=0;i+1<path.size();++i) {
    auto a=path[i],b=path[i+1];
    if (!std::isfinite(a.x)||!std::isfinite(a.y)||!std::isfinite(b.x)||!std::isfinite(b.y)) {return "invalid_route";}
    const double dx=b.x-a.x,dy=b.y-a.y,n=dx*dx+dy*dy;
    const double t=n>0 ? std::clamp(((map.x-a.x)*dx+(map.y-a.y)*dy)/n,0.0,1.0):0.0;
    nearest=std::min(nearest,std::hypot(map.x-a.x-t*dx,map.y-a.y-t*dy));
  }
  if (nearest>limits_.route_distance) {return "outside_route_corridor";}
  // MPPI legitimately combines translation and yaw; it has no discrete-mode status.
  if(!custom_mode) {reset();return {};}
  // Braking translation precedes the turn. Bound it by stopping distance;
  // start the strict turn-drift witness only after measured translation stops.
  if(mode=="stop_before_turn"||mode=="observation_wait") {
    if(!std::isfinite(limits_.braking_deceleration)||limits_.braking_deceleration<=0||
      !std::isfinite(limits_.feedback_delay)||limits_.feedback_delay<0) {return "invalid_braking_limits";}
    if(!braking_) {brake_start_={odom.x,odom.y};brake_distance_=speed*speed/(2*limits_.braking_deceleration)+speed*limits_.feedback_delay+limits_.turn_drift;}
    braking_=true;turning_=false;
    if(std::hypot(odom.x-brake_start_.x,odom.y-brake_start_.y)>brake_distance_) {return "braking_distance_exceeded";}
    return {};
  }
  braking_=false;
  const bool turn=mode=="route_turn"||mode=="final_turn"||mode=="settle_turn";
  if (turn) {
    if (!turning_) {turn_start_={odom.x,odom.y};}
    if (std::hypot(odom.x-turn_start_.x,odom.y-turn_start_.y)>limits_.turn_drift) {return "translation_during_turn";}
    if (speed>limits_.turn_speed) {return "turn_before_stopped";}
  } else if (mode=="forward"||mode=="backward"||mode=="left"||mode=="right") {
    if (speed>limits_.turn_speed && std::abs(wrap(heading-odom.yaw))>limits_.translating_heading) {
      return "translation_heading_error";
    }
  } else if (mode!="aligned"&&mode!="goal_stop"&&mode!="stop_before_axis_change"&&mode!="mission_hold") {
    return "unexpected_controller_mode:"+mode;
  }
  turning_=turn;
  return {};
}
}
