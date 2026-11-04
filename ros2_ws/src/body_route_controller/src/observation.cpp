#include "body_route_controller/observation.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace body_route_controller {
UnknownPolicy unknownPolicy(const std::string & s) {
  if(s=="blocked")return UnknownPolicy::Blocked;
  if(s=="observe-before-enter")return UnknownPolicy::ObserveBeforeEnter;
  throw std::invalid_argument("unknown-policy must be blocked or observe-before-enter");
}
void ObservationSettings::validate() const {
  for(double v:{ttl,scan_timeout,latency,linear_decel,angular_decel,self_radius,endpoint_margin})
    if(!std::isfinite(v)||v<=0)throw std::invalid_argument("observation limits must be finite and positive");
  if(ttl<scan_timeout+latency||scan_timeout>.5||latency<scan_timeout+.2||self_radius>.1)
    throw std::invalid_argument("invalid evidence lifetime, scan timeout, latency or self mask");
}
void ObservationSpace::setMap(const nav_msgs::msg::OccupancyGrid & m) {
  const auto &q=m.info.origin.orientation;
  if(m.header.frame_id!="map"||m.info.resolution<=0||!std::isfinite(m.info.resolution)||
    m.info.width==0||m.info.height==0||m.data.size()!=std::size_t(m.info.width)*m.info.height||m.data.size()>10000000||
    !std::isfinite(m.info.origin.position.x)||!std::isfinite(m.info.origin.position.y)||
    std::abs(q.x)>1e-8||std::abs(q.y)>1e-8||std::abs(q.z)>1e-8||std::abs(q.w-1)>1e-8)
    throw std::invalid_argument("evidence requires a finite unrotated map-frame occupancy grid");
  map_=m;identity_.clear();identity_.reserve(m.data.size());
  for(auto c:m.data)identity_.push_back(c<0?255:c==0?0:254); // uncertain nonzero cells fail closed
  hit_.assign(m.data.size(),0);free_.assign(m.data.size(),-1);hit_stamp_.assign(m.data.size(),-1);carried_.assign(m.data.size(),0);carried_indices_.clear();resetContinuity();
  stamp_={-1,-1};received_={-1,-1};failure_={"missing_scan","missing_scan"};
}
int ObservationSpace::index(double x,double y) const {
  if(!ready()||!std::isfinite(x)||!std::isfinite(y))return -1;
  const double u=(x-map_.info.origin.position.x)/map_.info.resolution,v=(y-map_.info.origin.position.y)/map_.info.resolution;
  if(u<0||v<0||u>=map_.info.width||v>=map_.info.height)return -1;
  return int(v)*map_.info.width+int(u);
}
unsigned char ObservationSpace::identity(double x,double y) const {int i=index(x,y);return i<0?255:identity_[i];}
bool ObservationSpace::observedOccupied(double x,double y) const {const int i=index(x,y);return i>=0&&hit_[i];}
int ObservationSpace::observedCell(double x,double y) const {const int i=index(x,y);return i<0?-1:hit_[i]?254:identity_[i];}
bool ObservationSpace::observedFree(double x,double y,double now) const {const int i=index(x,y);return i>=0&&identity_[i]==255&&executionCell(i,now,now)==0;}
bool ObservationSpace::selfOccluded(const ObservedRay & r) const {
  const double dx=r.body.x-r.origin.x,dy=r.body.y-r.origin.y;
  const double t=std::clamp(dx*std::cos(r.angle)+dy*std::sin(r.angle),0.0,r.range);
  return std::hypot(dx-t*std::cos(r.angle),dy-t*std::sin(r.angle))<settings_.self_radius;
}
void ObservationSpace::ray(const ObservedRay & r,double range,bool mark_hit) {
  const double step=map_.info.resolution*.4,c=std::cos(r.angle),s=std::sin(r.angle);
  if(mark_hit){
    if(!r.hit)return;
    const double x=r.origin.x+r.range*c,y=r.origin.y+r.range*s;
    // Match Nav2's occupied-hit geometry. Footprint padding and the existing
    // inflation layer already supply occupied-obstacle margins; do not inflate
    // hits a second time. The free ray still ends short of the measured hit.
    const int i=index(x,y);
    if(i>=0){hit_[i]=1;hit_stamp_[i]=std::max(hit_stamp_[i],r.stamp);free_[i]=-1;carried_[i]=0;}
    return;
  }
  // Do not assert clearance inside the minimum sensor range or through a known wall.
  for(double d=r.min_range;d<range-settings_.endpoint_margin;d+=step){
    int i=index(r.origin.x+d*c,r.origin.y+d*s);if(i<0||identity_[i]==254)break;
    if(r.stamp>=free_[i]&&r.stamp>hit_stamp_[i]){free_[i]=r.stamp;hit_[i]=0;}
  }
}
bool ObservationSpace::observe(std::size_t sensor,const std::vector<ObservedRay> & rays,double stamp,double received) {
  if(sensor>=2)throw std::invalid_argument("invalid lidar index");
  if(!ready()){invalidate(sensor,"missing_raw_map");return false;}
  if(!std::isfinite(stamp)||!std::isfinite(received)||stamp<=0||stamp<=stamp_[sensor]||
    received-stamp<0||received-stamp>settings_.scan_timeout){invalidate(sensor,"invalid_or_stale_stamp");return false;}
  if(rays.size()<50){invalidate(sensor,"insufficient_valid_rays");return false;}
  for(auto r:rays)for(double v:{r.origin.x,r.origin.y,r.angle,r.range,r.stamp,r.body.x,r.body.y,r.body.yaw,r.min_range})
    if(!std::isfinite(v)||r.min_range<.05||r.range<r.min_range||r.stamp<stamp||r.stamp>received){invalidate(sensor,"invalid_ray");return false;}
  if(std::count_if(rays.begin(),rays.end(),[this](const auto &r){return !selfOccluded(r);})<50){invalidate(sensor,"self_occluded_scan");return false;}
  std::size_t valid=0;
  for(std::size_t j=0;j<rays.size();++j){auto r=rays[j];if(selfOccluded(r))continue;++valid;ray(r,r.range,false);
    // Fill only adjacent angular samples, truncated to the nearer return. Invalid
    // beams and occlusion edges cannot bridge a hole or clear behind an obstacle.
    if(!j)continue;
    const auto a=rays[j-1];const double delta=wrap(r.angle-a.angle);
    if(selfOccluded(a)||std::abs(delta)>.012||std::hypot(r.origin.x-a.origin.x,r.origin.y-a.origin.y)>.002)continue;
    const double range=std::min(r.range,a.range);const int n=std::max(1,int(std::ceil(std::abs(delta)*range/(map_.info.resolution*.4))));
    for(int k=1;k<n;++k){auto b=a;b.angle+=delta*k/n;b.stamp=std::min(a.stamp,r.stamp);b.min_range=std::max(a.min_range,r.min_range);ray(b,range,false);}
  }
  if(valid<50){invalidate(sensor,"self_occluded_scan");return false;}
  for(auto r:rays)if(!selfOccluded(r))ray(r,r.range,true);
  stamp_[sensor]=stamp;received_[sensor]=received;failure_[sensor].clear();return true;
}
void ObservationSpace::invalidate(std::size_t sensor,const std::string & why){failure_.at(sensor)=why;}
std::string ObservationSpace::health(double now) const {
  if(!ready())return "missing_raw_map";
  for(std::size_t i=0;i<2;++i){
    if(!failure_[i].empty())return failure_[i]+"_"+std::to_string(i);
    if(!std::isfinite(now)||now<received_[i]||now-stamp_[i]>settings_.scan_timeout||now-received_[i]>settings_.scan_timeout)
      return "expired_scan_"+std::to_string(i);
  }return {};
}
nav2_costmap_2d::Costmap2D ObservationSpace::rawMap() const {
  nav2_costmap_2d::Costmap2D m(map_.info.width,map_.info.height,map_.info.resolution,map_.info.origin.position.x,map_.info.origin.position.y,255);
  if(ready()){std::copy(identity_.begin(),identity_.end(),m.getCharMap());}
  return m;
}
unsigned char ObservationSpace::executionCell(std::size_t i,double now,double valid_until) const {
  if(identity_[i]==254||hit_[i])return 254;
  if(identity_[i]==255&&((free_[i]>=0&&free_[i]<=now&&valid_until-free_[i]<=settings_.ttl)||carried_[i]))return 0;
  return identity_[i];
}
nav2_costmap_2d::Costmap2D ObservationSpace::executionMap(double now,double valid_until) const {
  auto m=rawMap();for(std::size_t i=0;i<identity_.size();++i)m.getCharMap()[i]=executionCell(i,now,valid_until);
  return m;
}
nav2_costmap_2d::Costmap2D ObservationSpace::window(Pose p,double radius,double now,double valid_until) const {
  const auto &info=map_.info;const double r=info.resolution;
  if(!ready()||!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(radius)||radius<0||index(p.x,p.y)<0)
    return nav2_costmap_2d::Costmap2D(1,1,.01,0,0,255);
  const int x0=std::clamp(int(std::floor((p.x-radius-info.origin.position.x)/r)),0,int(info.width)-1);
  const int y0=std::clamp(int(std::floor((p.y-radius-info.origin.position.y)/r)),0,int(info.height)-1);
  const int x1=std::clamp(int(std::ceil((p.x+radius-info.origin.position.x)/r)),x0+1,int(info.width));
  const int y1=std::clamp(int(std::ceil((p.y+radius-info.origin.position.y)/r)),y0+1,int(info.height));
  nav2_costmap_2d::Costmap2D m(x1-x0,y1-y0,r,info.origin.position.x+x0*r,info.origin.position.y+y0*r,255);
  for(int y=y0;y<y1;++y)for(int x=x0;x<x1;++x)m.setCost(x-x0,y-y0,executionCell(y*info.width+x,now,valid_until));
  return m;
}
void ObservationSpace::resetContinuity(){for(auto i:carried_indices_)carried_[i]=0;carried_indices_.clear();previous_time_=-1;}
bool ObservationSpace::certify(const Footprint & f,Pose p,double now) {
  const double dt=now-previous_time_;
  if(previous_time_<0||dt<0||dt>settings_.scan_timeout||std::hypot(p.x-previous_pose_.x,p.y-previous_pose_.y)>.35*dt+.01||
    std::abs(wrap(p.yaw-previous_pose_.yaw))>1.7*dt+.02)resetContinuity();
  double radius=.05;for(auto q:f)radius=std::max(radius,std::hypot(q.x,q.y)+.05);
  auto m=window(p,radius,now,now);std::vector<std::size_t> next;
  const bool safe=visitFootprint(m,f,p,[&](unsigned x,unsigned y){double wx,wy;m.mapToWorld(x,y,wx,wy);const auto i=index(wx,wy);
    if(i<0||m.getCost(x,y)>=254){return false;}
    next.push_back(i);return true;});
  for(auto i:carried_indices_)carried_[i]=0;
  carried_indices_.clear();if(safe){carried_indices_=std::move(next);for(auto i:carried_indices_)carried_[i]=1;}
  previous_time_=now;previous_pose_=p;return safe;
}
std::string ObservationSpace::check(const Footprint & f,Pose p,Velocity measured,Velocity requested,double now,const std::vector<Velocity> & pending) const {
  auto why=health(now);if(!why.empty())return why;
  auto velocities=pending;velocities.push_back(measured);velocities.push_back(requested);velocities.push_back({});
  for(auto v:velocities)if(!std::isfinite(v.x)||!std::isfinite(v.y)||!std::isfinite(v.yaw))return "invalid_velocity";
  if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.yaw))return "invalid_pose";
  if(f.size()<3)return "invalid_footprint";
  for(auto q:f)if(!std::isfinite(q.x)||!std::isfinite(q.y)||std::hypot(q.x,q.y)>2)return "invalid_footprint";
  // Also enclose component-wise velocity limiting during a direction change.
  double vx0=0,vx1=0,vy0=0,vy1=0,w0=0,w1=0;
  for(auto v:velocities){vx0=std::min(vx0,v.x);vx1=std::max(vx1,v.x);vy0=std::min(vy0,v.y);vy1=std::max(vy1,v.y);w0=std::min(w0,v.yaw);w1=std::max(w1,v.yaw);}
  const double speed=std::hypot(std::max(-vx0,vx1),std::max(-vy0,vy1)),yaw_speed=std::max(-w0,w1);
  if(speed>1||yaw_speed>6)return "velocity_outside_model_bounds";
  const double horizon=settings_.latency+std::max(speed/settings_.linear_decel,yaw_speed/settings_.angular_decel);
  if(!std::isfinite(horizon))return "invalid_stopping_horizon";
  // Independent linear/angular braking does not necessarily follow a constant
  // twist arc. Bound every orientation and velocity between measured/requested
  // commands (including zero), then integrate their coordinate bounds. This
  // conservative rectangle contains the full footprint throughout latency and
  // braking, including a change of steering direction or unequal braking rates.
  const double lo=p.yaw+w0*horizon;
  const double hi=p.yaw+w1*horizon;
  constexpr double pi=3.14159265358979323846;
  auto range=[&](double x,double y){
    const double radius=std::hypot(x,y),angle=std::atan2(y,x);
    if(hi-lo>=2*pi)return std::pair<double,double>{-radius,radius};
    const double a=lo+angle,b=hi+angle;
    double low=std::min(radius*std::cos(a),radius*std::cos(b));
    double high=std::max(radius*std::cos(a),radius*std::cos(b));
    if(std::ceil(a/(2*pi))<=std::floor(b/(2*pi)))high=radius;
    if(std::ceil((a-pi)/(2*pi))<=std::floor((b-pi)/(2*pi)))low=-radius;
    return std::pair<double,double>{low,high};
  };
  double dx0=0,dx1=0,dy0=0,dy1=0;
  for(double vx:{vx0,vx1})for(double vy:{vy0,vy1}){auto x=range(vx,vy),y=range(vy,-vx);
    dx0=std::min(dx0,x.first*horizon);dx1=std::max(dx1,x.second*horizon);
    dy0=std::min(dy0,y.first*horizon);dy1=std::max(dy1,y.second*horizon);}
  double x0=INFINITY,x1=-INFINITY,y0=INFINITY,y1=-INFINITY;
  for(auto q:f){auto x=range(q.x,q.y),y=range(q.y,-q.x);
    x0=std::min(x0,x.first);x1=std::max(x1,x.second);y0=std::min(y0,y.first);y1=std::max(y1,y.second);}
  x0+=p.x+dx0;x1+=p.x+dx1;y0+=p.y+dy0;y1+=p.y+dy1;
  if(index(x0,y0)<0||index(x1,y1)<0)return "outside_map_stopping_sweep";
  Footprint envelope;
  for(auto q:{Point{x0,y0},Point{x1,y0},Point{x1,y1},Point{x0,y1}}){geometry_msgs::msg::Point v;v.x=q.x;v.y=q.y;envelope.push_back(v);}
  const double radius=std::max({std::abs(x0-p.x),std::abs(x1-p.x),std::abs(y0-p.y),std::abs(y1-p.y)})+.05;
  auto m=window(p,radius,now,now+horizon);CollisionWitness witness;
  if(!footprintSafe(m,envelope,{},&witness))return std::string(witness.cost==255?"unobserved_stopping_sweep":witness.cost==254?"occupied_stopping_sweep":"outside_map_stopping_sweep")+
    " cell_x="+std::to_string(witness.x)+" cell_y="+std::to_string(witness.y)+" horizon_s="+std::to_string(horizon);
  return {};
}
nav_msgs::msg::OccupancyGrid ObservationSpace::display(double now) const {
  auto out=map_;for(std::size_t i=0;i<identity_.size();++i){
    out.data[i]=identity_[i]==254||hit_[i]?100:identity_[i]==0?0:
      ((free_[i]>=0&&now>=free_[i]&&now-free_[i]<=settings_.ttl)||carried_[i])?25:-1;
  }return out;
}
}
