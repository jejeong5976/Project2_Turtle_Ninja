#include "body_route_controller/passage.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace body_route_controller
{
namespace
{
constexpr double pi = 3.14159265358979323846;

// A per-call prefix sum proves that a polygon's complete cell bounding box
// contains no lethal/unknown cell. Any nonempty/out-of-window box falls back to
// the existing exact footprint test. The caller holds the map lock throughout.
class FreeBoxIndex {
public:
  FreeBoxIndex(nav2_costmap_2d::Costmap2D & map,Point a,Point b,Point c,double margin):map_(map) {
    if(map.getSizeInCellsX()==0||map.getSizeInCellsY()==0)return;
    int x0,y0,x1,y1;
    map.worldToMapEnforceBounds(std::min({a.x,b.x,c.x})-margin,std::min({a.y,b.y,c.y})-margin,x0,y0);
    map.worldToMapEnforceBounds(std::max({a.x,b.x,c.x})+margin,std::max({a.y,b.y,c.y})+margin,x1,y1);
    x0_=x0;y0_=y0;width_=x1-x0+1;height_=y1-y0+1;
    if(double(width_)*height_>2000000){width_=height_=0;return;}
    sums_.assign((width_+1)*(height_+1),0);
    for(unsigned y=0;y<height_;++y){
      unsigned row=0;
      for(unsigned x=0;x<width_;++x){
        row+=map.getCost(x+x0_,y+y0_)>=254;
        sums_[(y+1)*(width_+1)+x+1]=sums_[y*(width_+1)+x+1]+row;
      }
    }
  }
  bool safe(const Footprint & f,Pose pose) const {
    if(f.size()<3||!std::isfinite(pose.x)||!std::isfinite(pose.y)||!std::isfinite(pose.yaw))return false;
    double xmin=INFINITY,ymin=INFINITY,xmax=-INFINITY,ymax=-INFINITY;
    const double c=std::cos(pose.yaw),s=std::sin(pose.yaw);
    for(auto p:f){
      if(!std::isfinite(p.x)||!std::isfinite(p.y))return false;
      const double x=pose.x+c*p.x-s*p.y,y=pose.y+s*p.x+c*p.y;
      xmin=std::min(xmin,x);xmax=std::max(xmax,x);ymin=std::min(ymin,y);ymax=std::max(ymax,y);
    }
    unsigned x0,y0,x1,y1;
    if(!map_.worldToMap(xmin,ymin,x0,y0)||!map_.worldToMap(xmax,ymax,x1,y1))return false;
    if(width_&&x0>=x0_&&y0>=y0_&&x1<x0_+width_&&y1<y0_+height_){
      x0-=x0_;x1=x1-x0_+1;y0-=y0_;y1=y1-y0_+1;
      const auto stride=width_+1;
      const auto count=sums_[y1*stride+x1]+sums_[y0*stride+x0]-sums_[y0*stride+x1]-sums_[y1*stride+x0];
      if(count==0)return true;
    }
    return footprintSafe(map_,f,pose);
  }
private:
  nav2_costmap_2d::Costmap2D & map_;
  unsigned x0_=0,y0_=0,width_=0,height_=0;
  std::vector<unsigned> sums_;
};

double distance(Point a, Point b) {return std::hypot(a.x-b.x,a.y-b.y);}
double extent(const Footprint & f) {
  double r=0; for(auto p:f) r=std::max(r,std::hypot(p.x,p.y)); return r;
}
double along(Point p, Point origin, double heading) {
  return (p.x-origin.x)*std::cos(heading)+(p.y-origin.y)*std::sin(heading);
}
double segmentDistance(Point p, Point a, Point b) {
  const double n=std::pow(distance(a,b),2);
  const double t=n>0?std::clamp(((p.x-a.x)*(b.x-a.x)+(p.y-a.y)*(b.y-a.y))/n,0.,1.):0.;
  return distance(p,{a.x+t*(b.x-a.x),a.y+t*(b.y-a.y)});
}
Footprint box(Point c,double r) {
  Footprint out; for(auto p:std::vector<Point>{{-r,-r},{r,-r},{r,r},{-r,r}}) {
    geometry_msgs::msg::Point q;q.x=c.x+p.x;q.y=c.y+p.y;out.push_back(q);
  } return out;
}
}

void PassageSettings::validate() const {
  if(!std::isfinite(position_allowance)||position_allowance<0||position_allowance>.2||
    !std::isfinite(yaw_allowance)||yaw_allowance<0||yaw_allowance>pi/6||
    !std::isfinite(response_delay)||response_delay<.5||response_delay>2.) {
    throw std::invalid_argument("Invalid passage uncertainty/response allowance");
  }
}

Footprint uncertaintyFootprint(const Footprint & f, Point center, double position, double yaw) {
  if(f.size()<3||!std::isfinite(center.x)||!std::isfinite(center.y)||
    !std::isfinite(position)||position<0||!std::isfinite(yaw)||yaw<0||yaw>pi) return {};
  double radius=0;
  for(auto p:f) {
    if(!std::isfinite(p.x)||!std::isfinite(p.y)) return {};
    radius=std::max(radius,distance({p.x,p.y},center));
  }
  const double margin=position+2*radius*std::sin(yaw/2);
  // Support half-planes enclose the convex footprint plus a disk. Thus this
  // includes any translation/yaw error within the declared allowance.
  constexpr int count=32;
  double support[count]; Point normal[count];
  for(int i=0;i<count;++i) {
    normal[i]={std::cos(2*pi*i/count),std::sin(2*pi*i/count)};
    support[i]=-INFINITY;
    for(auto p:f) support[i]=std::max(support[i],p.x*normal[i].x+p.y*normal[i].y);
    support[i]+=margin+1e-9;
  }
  Footprint out;
  for(int i=0;i<count;++i) {
    int j=(i+1)%count; const double d=normal[i].x*normal[j].y-normal[i].y*normal[j].x;
    geometry_msgs::msg::Point p;
    p.x=(support[i]*normal[j].y-normal[i].y*support[j])/d;
    p.y=(normal[i].x*support[j]-support[i]*normal[j].x)/d;
    out.push_back(p);
  }
  return out;
}

PreparedPassage::PreparedPassage(Settings cfg,PassageSettings limits)
:cfg_(cfg),limits_(limits),curve_core_(cfg) {cfg_.validate();limits_.validate();}
void PreparedPassage::pause() {
  previous_={};curve_core_.reset();paused_=true;observed_costs_.clear();
}
void PreparedPassage::reset() {
  pause();phase_=Phase::Idle;route_.clear();reason_.clear();
}

bool PreparedPassage::routeMatches(const std::vector<Point> & path,const Pose & goal) const {
  if(distance({goal.x,goal.y},{goal_.x,goal_.y})>cfg_.corner_tolerance||
    std::abs(wrap(goal.yaw-goal_.yaw))>cfg_.align_exit||path.size()<2) return false;
  const double tolerance=2*cfg_.path_simplify_tolerance;
  // An unchanged exit alone is insufficient: a new detour can replace the
  // approach while ending on the same crossing leg. Validate the retained
  // portion of the current polyline too, ignoring already-pruned history and
  // the route after the passage exit.
  const double approach_length=distance(start_,corner_);
  const Point before{corner_.x-(approach_length+.1)*std::cos(incoming_),
    corner_.y-(approach_length+.1)*std::sin(incoming_)};
  for(auto p:path) {
    if(along(p,corner_,outgoing_)>distance(corner_,exit_)+tolerance) break;
    if(along(p,corner_,incoming_)<-approach_length-.1) continue;
    if(std::min(segmentDistance(p,before,corner_),segmentDistance(p,corner_,exit_))>tolerance) return false;
  }
  // Match retained entry, tangent and exit against the CURRENT route. Pruning
  // may remove the incoming leg, but not the outgoing crossing geometry.
  bool found=false;
  for(std::size_t i=0;i+1<path.size();++i) {
    const double heading=std::atan2(path[i+1].y-path[i].y,path[i+1].x-path[i].x);
    if(std::abs(wrap(heading-outgoing_))>cfg_.align_exit) continue;
    if(segmentDistance(exit_,path[i],path[i+1])<=tolerance &&
      segmentDistance(corner_,path[i],path[i+1])<=tolerance) {found=true;break;}
    if((phase_==Phase::Cross||phase_==Phase::Curve) && segmentDistance(exit_,path[i],path[i+1])<=tolerance &&
      std::abs(along(path[i],corner_,outgoing_+pi/2))<=tolerance &&
      along(path[i],corner_,outgoing_)>=-tolerance) {found=true;break;}
  }
  return found;
}

bool PreparedPassage::changed(nav2_costmap_2d::Costmap2D & map,const Footprint & f) {
  const double margin=extent(f)+cfg_.max_path_distance+cfg_.max_speed*
    (limits_.response_delay+cfg_.max_speed/cfg_.linear_accel)+limits_.position_allowance+cfg_.lookahead+.1;
  const double xmin=std::min({start_.x,corner_.x,exit_.x})-margin;
  const double ymin=std::min({start_.y,corner_.y,exit_.y})-margin;
  const double xmax=std::max({start_.x,corner_.x,exit_.x})+margin;
  const double ymax=std::max({start_.y,corner_.y,exit_.y})+margin;
  std::vector<unsigned char> current;
  if(map.getSizeInCellsX()==0||map.getSizeInCellsY()==0) return true;
  int bx0,by0,bx1,by1;
  map.worldToMapEnforceBounds(xmin,ymin,bx0,by0);
  map.worldToMapEnforceBounds(xmax,ymax,bx1,by1);
  const unsigned x0=bx0,y0=by0,x1=bx1,y1=by1;
  // Outside this clipped window stays out-of-map/blocked. Origin, dimensions
  // and resolution are part of equality, so rolling/revealed cells revalidate.
  const std::vector<double> key{map.getOriginX(),map.getOriginY(),map.getResolution(),
    double(map.getSizeInCellsX()),double(map.getSizeInCellsY()),
    double(x0),double(y0),double(x1),double(y1)};
  if((double(x1)-x0+1)*(double(y1)-y0+1)>2000000) return true;
  current.reserve((x1-x0+1)*(y1-y0+1));
  for(unsigned y=y0;y<=y1;++y) for(unsigned x=x0;x<=x1;++x) {
    const auto cost=map.getCost(x,y);
    // Geometric checks block lethal/unknown, not inflation costs. Retain their
    // exact class; only nonblocking cost variations are irrelevant here.
    current.push_back(cost>=254?cost:0);
  }
  const bool different=key!=observed_grid_||current!=observed_costs_;
  observed_grid_=key;observed_costs_=std::move(current);return different;
}

bool PreparedPassage::wideCurveSafe(nav2_costmap_2d::Costmap2D & map,const Footprint & f) const {
  // Sufficient open-room check, deliberately conservative. The current path
  // corridor, lookahead, entire body and response/stopping distance fit inside
  // this region. Current motion/envelope guards remain authoritative each tick.
  const double lead=cfg_.lookahead+extent(f)+cfg_.max_speed*(limits_.response_delay+cfg_.max_speed/cfg_.linear_accel);
  // Include the component-wise velocity hull used by stoppingEnvelope, not
  // only a nominal constant-twist arc. Curve centers must stay inside lead.
  const double hull_speed=std::sqrt(2.)*cfg_.max_speed;
  const double travel=hull_speed*(limits_.response_delay+hull_speed/cfg_.linear_accel);
  const double r=lead+extent(f)+travel+cfg_.max_speed*cfg_.max_dt+
    limits_.position_allowance+2*(extent(f)+travel)*std::sin(limits_.yaw_allowance/2)+map.getResolution();
  return footprintSafe(map,box(corner_,r),{});
}

bool PreparedPassage::geometrySafe(nav2_costmap_2d::Costmap2D & map,const Footprint & f,double speed, bool check_body_and_turn) const {
  // Reserve a cell half-diagonal in addition to pose uncertainty. Independent
  // polygon-to-cell distance analysis must stay positive at raster boundaries.
  const auto padded=uncertaintyFootprint(f,{},limits_.position_allowance+map.getResolution()/std::sqrt(2.),limits_.yaw_allowance);
  FreeBoxIndex free_boxes(map,start_,corner_,exit_,extent(f)+limits_.position_allowance+
    speed*(limits_.response_delay+speed/cfg_.linear_accel)+.2);
  if(check_body_and_turn&&!turnSafe(map,padded,{corner_.x,corner_.y,incoming_},outgoing_)) return false;
  // Check the complete straight approach/crossing, including forward stopping
  // tails at the selected local cap. No L2 coordinates or removed scan returns.
  for(const auto & leg:std::vector<std::pair<Point,Point>>{{start_,corner_},{corner_,exit_}}) {
    const double heading=std::atan2(leg.second.y-leg.first.y,leg.second.x-leg.first.x);
    const int count=std::max(1,static_cast<int>(std::ceil(distance(leg.first,leg.second)/(.5*map.getResolution()))));
    if(count>2000) return false;
    double cached_cap=-1;
    Footprint uncertain;
    for(int i=0;i<=count;++i) {
      const double t=double(i)/count;
      Pose p{leg.first.x+t*(leg.second.x-leg.first.x),leg.first.y+t*(leg.second.y-leg.first.y),heading};
      double cap=speed;
      if(leg.first.x==start_.x&&leg.first.y==start_.y) cap=std::min(cap,cfg_.linear_kp*distance({p.x,p.y},corner_));
      // At a fixed heading/cap the envelope translates without changing shape.
      // Cache only within this leg and this locked-map call; changed pose/cap,
      // observations and pause still receive the same checks.
      if(cap!=cached_cap) {
        const auto env=stoppingEnvelope(f,{0,0,heading},{{cap,0,0}},limits_.response_delay,cfg_.linear_accel,cfg_.angular_accel);
        uncertain=uncertaintyFootprint(env.polygon,{},limits_.position_allowance+map.getResolution()/std::sqrt(2.),limits_.yaw_allowance);
        cached_cap=cap;
      }
      if((check_body_and_turn&&!free_boxes.safe(padded,p))||uncertain.empty()||
        !free_boxes.safe(uncertain,{p.x,p.y,0})) return false;
    }
  }
  return true;
}

Result PreparedPassage::translate(const Pose & pose,const Velocity & measured,Point endpoint,
  double heading,double dt,double limit,double hard_limit,bool stop_at_endpoint) {
  const Point here{pose.x,pose.y};
  const double forward=along(endpoint,here,heading),lateral=along(endpoint,here,heading+pi/2);
  const double error=wrap(heading-pose.yaw);
  if(std::abs(error)>cfg_.max_travel_angle||
    (std::hypot(measured.x,measured.y)>cfg_.stopped_linear&&
    std::abs(std::atan2(measured.y,measured.x))>cfg_.max_travel_angle)) {
    previous_={};return {{},"passage_heading_wait",heading,error};
  }
  double speed=std::min(limit,cfg_.max_speed);
  if(stop_at_endpoint) {
    // Continuous distance-based deceleration starts before the checked turn.
    const double d=std::max(0.,forward);
    speed=std::min({speed,cfg_.linear_kp*d,
      std::max(0.,std::sqrt(std::pow(cfg_.linear_accel*limits_.response_delay,2)+2*cfg_.linear_accel*d)-cfg_.linear_accel*limits_.response_delay)});
  }
  const double side=std::clamp(cfg_.linear_kp*lateral,-cfg_.correction_speed,cfg_.correction_speed);
  // Smooth diagonal correction; cone limiting must not invent sideways/reverse
  // motion or replace the fixed heading with a pursuit-bearing turn target.
  double wx=speed*std::cos(heading)-side*std::sin(heading);
  double wy=speed*std::sin(heading)+side*std::cos(heading);
  double bx=std::cos(pose.yaw)*wx+std::sin(pose.yaw)*wy;
  double by=-std::sin(pose.yaw)*wx+std::cos(pose.yaw)*wy;
  const double cone=cfg_.max_travel_angle-std::min(pi/12,cfg_.max_travel_angle/3);
  by=std::clamp(by,-std::max(0.,bx)*std::tan(cone),std::max(0.,bx)*std::tan(cone));
  bx=std::max(0.,bx);
  const double norm=std::hypot(bx,by);if(norm>limit&&norm>0){bx*=limit/norm;by*=limit/norm;}
  const double rotation=wrap(previous_yaw_-pose.yaw);
  const double oldx=std::cos(rotation)*previous_.x-std::sin(rotation)*previous_.y;
  const double oldy=std::sin(rotation)*previous_.x+std::cos(rotation)*previous_.y;
  const double delta=std::hypot(bx-oldx,by-oldy);
  const double fraction=delta>0?std::min(1.,cfg_.linear_accel*dt/delta):1.;
  previous_={oldx+fraction*(bx-oldx),oldy+fraction*(by-oldy),previous_.yaw};
  const double command_norm=std::hypot(previous_.x,previous_.y);
  const double hard_cap=std::min(cfg_.max_speed,hard_limit);
  if(command_norm>hard_cap){previous_.x*=hard_cap/command_norm;previous_.y*=hard_cap/command_norm;}
  const double yaw_limit=std::min(cfg_.heading_hold_rate,cfg_.heading_rate_per_speed*std::hypot(previous_.x,previous_.y));
  const double wanted=std::clamp(cfg_.heading_kp*error,-yaw_limit,yaw_limit);
  previous_.yaw+=std::clamp(wanted-previous_.yaw,-cfg_.angular_accel*dt,cfg_.angular_accel*dt);
  previous_.yaw=std::clamp(previous_.yaw,-yaw_limit,yaw_limit);previous_yaw_=pose.yaw;
  return {previous_,stop_at_endpoint?"passage_approach":"passage_cross",heading,error};
}

std::optional<Result> PreparedPassage::step(nav2_costmap_2d::Costmap2D & map,const Footprint & f,
  const Pose & pose,const Velocity & measured,const std::vector<Point> & raw,const Pose & goal,
  double dt,double speed_limit,const std::vector<Velocity> & pending) {
  if(!limits_.enabled||cfg_.drive_style!="continuous") return std::nullopt;
  auto stop=[&](std::string why) -> std::optional<Result> {
    reason_=std::move(why);previous_={};previous_yaw_=pose.yaw;
    const double heading=active()?outgoing_:pose.yaw;
    return Result{{},"passage_wait",heading,wrap(heading-pose.yaw)};
  };
  for(double v:{pose.x,pose.y,pose.yaw,measured.x,measured.y,measured.yaw,goal.x,goal.y,goal.yaw,dt,speed_limit}) {
    if(!std::isfinite(v)) {reset();throw std::invalid_argument("Invalid passage state");}
  }
  if(dt<=0||dt>cfg_.max_dt||speed_limit<=0){pause();return stop("passage_clock_or_speed_stop");}
  const auto path=routeCorners(raw,cfg_.path_simplify_tolerance);
  const Point here{pose.x,pose.y};
  if(active()&&!routeMatches(path,goal)){reset();return stop("passage_route_changed");}
  bool newly_validated=false;
  if(!active()) {
    double best=INFINITY;std::size_t nearest=0;double searched=0;
    for(std::size_t i=0;i+1<path.size();++i) {
      double d=segmentDistance(here,path[i],path[i+1]);
      if(d<best-1e-9){best=d;nearest=i;}
      searched+=distance(path[i],path[i+1]);if(searched>1.)break;
    }
    if(best>cfg_.max_path_distance) return std::nullopt;
    if(nearest+2>=path.size()) return std::nullopt;
    const double incoming=std::atan2(path[nearest+1].y-path[nearest].y,path[nearest+1].x-path[nearest].x);
    const double outgoing=std::atan2(path[nearest+2].y-path[nearest+1].y,path[nearest+2].x-path[nearest+1].x);
    // Smaller changes remain inside B's forward/diagonal cone; keep its normal
    // smooth tracking and live guards rather than adding a stop at every bend.
    if(std::abs(wrap(outgoing-incoming))<=cfg_.max_travel_angle) return std::nullopt;
    const double lead=cfg_.lookahead+cfg_.max_speed*(limits_.response_delay+cfg_.max_speed/cfg_.linear_accel)+extent(f);
    if(distance(here,path[nearest+1])>lead) return std::nullopt;
    start_=here;corner_=path[nearest+1];exit_=path[nearest+2];goal_=goal;
    incoming_=incoming;outgoing_=outgoing;route_=path;previous_=measured;previous_yaw_=pose.yaw;
    local_cap_=std::min(speed_limit,cfg_.max_speed);
    // Keep the outgoing reference until the whole footprint has room to turn.
    // If the route ends inside the opening, stop there without final yaw.
    phase_=Phase::Approach;
    if(wideCurveSafe(map,f)) {phase_=Phase::Curve;curve_core_.reset();reason_="prepared_open_curve";}
    else {
      if(!geometrySafe(map,f,0)) {reset();return stop("no_checked_turn_or_crossing");}
      if(!geometrySafe(map,f,local_cap_,false)) {
        double low=0,high=local_cap_;
        for(int i=0;i<8;++i){double mid=(low+high)/2;if(geometrySafe(map,f,mid,false))low=mid;else high=mid;}
        local_cap_=low;
      }
      if(local_cap_<cfg_.stopped_linear){reset();return stop("no_checked_crossing_speed");}
      // End the lock after the final rotation-restricted part of this leg,
      // leaving enough route for proactive preparation of the following turn.
      const double turning_radius=extent(f)+limits_.position_allowance+map.getResolution()/std::sqrt(2.)+2*extent(f)*std::sin(limits_.yaw_allowance/2);
      Footprint turn_disk;
      for(int i=0;i<32;++i) {
        geometry_msgs::msg::Point p;p.x=turning_radius*std::cos(2*pi*i/32)/std::cos(pi/32);
        p.y=turning_radius*std::sin(2*pi*i/32)/std::cos(pi/32);turn_disk.push_back(p);
      }
      const double length=distance(corner_,exit_);double last_narrow=0;
      const int count=std::max(1,static_cast<int>(std::ceil(length/map.getResolution())));
      if(count>2000){reset();return stop("passage_segment_too_long");}
      for(int i=0;i<=count;++i) {
        const double d=length*i/count;
        Pose probe{corner_.x+d*std::cos(outgoing_),corner_.y+d*std::sin(outgoing_),outgoing_};
        if(!footprintSafe(map,turn_disk,probe)) last_narrow=d;
      }
      const double release=std::min(length,std::max(last_narrow+2*map.getResolution(),extent(f)+cfg_.lookahead));
      exit_={corner_.x+release*std::cos(outgoing_),corner_.y+release*std::sin(outgoing_)};
      reason_="prepared_stop_align_cross";
    }
    observed_costs_.clear();paused_=false;newly_validated=true;
  }
  const bool map_changed=changed(map,f);
  if((map_changed||paused_)&&!newly_validated) {
    const bool valid=phase_==Phase::Curve?wideCurveSafe(map,f):geometrySafe(map,f,local_cap_);
    if(!valid){reset();return stop("passage_obstacle_or_unknown_changed");}
    paused_=false;
  }
  if(segmentDistance(here,start_,corner_)>cfg_.max_path_distance&&
    segmentDistance(here,corner_,exit_)>cfg_.max_path_distance) {
    reset();return stop("passage_pose_departed");
  }
  const auto padded=uncertaintyFootprint(f,{},limits_.position_allowance+map.getResolution()/std::sqrt(2.),limits_.yaw_allowance);
  const bool stopped=std::hypot(measured.x,measured.y)<=cfg_.stopped_linear;
  const bool settled=stopped&&std::abs(measured.yaw)<=cfg_.stopped_angular;
  Result result;
  if(phase_==Phase::Curve) {
    if(along(here,corner_,outgoing_)>cfg_.lookahead+extent(f)) {reset();return std::nullopt;}
    const double lead=cfg_.lookahead+extent(f)+cfg_.max_speed*(limits_.response_delay+cfg_.max_speed/cfg_.linear_accel);
    if(distance(here,corner_)>lead) {pause();return stop("prepared_curve_departed_checked_room");}
    result=curve_core_.step(pose,measured,route_,goal,dt,speed_limit);
  } else if(phase_==Phase::Approach) {
    // A tighter position checkpoint reduces sensitivity to residual stop error;
    // it never widens the route or relaxes the existing final-goal tolerance.
    if(distance(here,corner_)<=cfg_.corner_tolerance/4) {
      previous_={};if(!settled) return stop("passage_settle_at_turn");
      phase_=Phase::Align;
    } else {
      if(along(corner_,here,incoming_)<0){reset();return stop("passage_turn_overshot");}
      result=translate(pose,measured,corner_,incoming_,dt,std::min(local_cap_,speed_limit),speed_limit,true);
    }
  }
  if(phase_==Phase::Align) {
    if(!stopped) return stop("passage_stop_before_alignment");
    if(!turnSafe(map,padded,pose,outgoing_)){reset();return stop("passage_actual_turn_blocked");}
    const double error=wrap(outgoing_-pose.yaw);
    if(std::abs(error)>cfg_.align_exit/4) {
      const double wanted=std::clamp(cfg_.heading_kp*error,-cfg_.max_yaw_rate,cfg_.max_yaw_rate);
      previous_={0,0,measured.yaw+std::clamp(wanted-measured.yaw,-cfg_.angular_accel*dt,cfg_.angular_accel*dt)};
      previous_.yaw=std::clamp(previous_.yaw,-cfg_.max_yaw_rate,cfg_.max_yaw_rate);
      result={previous_,"route_turn",outgoing_,error};
    } else {
      previous_={};if(!settled) return stop("passage_settle_heading");
      phase_=Phase::Cross;previous_yaw_=pose.yaw;
      return stop("passage_aligned");
    }
  } else if(phase_==Phase::Cross) {
    const double remaining=along(exit_,here,outgoing_);
    if(remaining<=cfg_.goal_xy) {
      // A complete swept turn at the measured exit proves the full footprint
      // has cleared the restriction before B/C normal/final heading resumes.
      if(turnSafe(map,padded,pose,wrap(outgoing_+pi))) {reset();return std::nullopt;}
      if(remaining<=cfg_.corner_tolerance/4) return stop("passage_exit_not_clear_for_heading");
    }
    result=translate(pose,measured,exit_,outgoing_,dt,std::min(local_cap_,speed_limit),speed_limit,remaining<cfg_.goal_xy);
  }
  std::vector<Velocity> velocities=pending;velocities.push_back(measured);velocities.push_back(result.command);
  const auto envelope=stoppingEnvelope(f,pose,velocities,limits_.response_delay,cfg_.linear_accel,cfg_.angular_accel);
  const auto uncertain=uncertaintyFootprint(envelope.polygon,here,limits_.position_allowance+map.getResolution()/std::sqrt(2.),limits_.yaw_allowance);
  if(uncertain.empty()||!footprintSafe(map,uncertain,{})) {pause();return stop("passage_uncertain_stopping_blocked");}
  return result;
}
}
