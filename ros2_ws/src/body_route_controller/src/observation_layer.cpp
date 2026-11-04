#include "body_route_controller/observation_layer.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
namespace body_route_controller {
namespace {
Pose planar(const geometry_msgs::msg::Transform & t){
  const auto q=t.rotation;const double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
  if(!std::isfinite(t.translation.x)||!std::isfinite(t.translation.y)||!std::isfinite(norm)||
    std::abs(norm-1)>.001||std::abs(q.x)>.005||std::abs(q.y)>.005)throw std::runtime_error("non_planar_or_invalid_transform");
  return {t.translation.x,t.translation.y,tf2::getYaw(q)};
}
Pose convert(Pose p,Pose t){return {t.x+std::cos(t.yaw)*p.x-std::sin(t.yaw)*p.y,
  t.y+std::sin(t.yaw)*p.x+std::cos(t.yaw)*p.y,wrap(p.yaw+t.yaw)};}
}
void ObservationLayer::onInitialize(){
  auto n=node_.lock();if(!n)throw std::runtime_error("missing observation layer node");
  observe_=unknownPolicy(n->declare_or_get_parameter(name_+".unknown_policy",std::string("blocked")))==UnknownPolicy::ObserveBeforeEnter;
  ObservationSettings s;
#define LOAD(field) s.field=n->declare_or_get_parameter(name_+"."+#field,s.field)
  LOAD(ttl);LOAD(scan_timeout);LOAD(latency);LOAD(linear_decel);LOAD(angular_decel);LOAD(self_radius);LOAD(endpoint_margin);
#undef LOAD
  space_=ObservationSpace(s);enabled_=true;current_=false;
  map_sub_=n->create_subscription<nav_msgs::msg::OccupancyGrid>("/map",[this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m){
    std::lock_guard<std::mutex> lock(mutex_);try{space_.setMap(*m);current_=true;}
    catch(const std::exception & e){current_=false;RCLCPP_ERROR(logger_,"Raw map rejected: %s",e.what());}
  },rclcpp::QoS(1).transient_local().reliable());
  if(observe_)for(std::size_t i=0;i<2;++i)scans_[i]=n->create_subscription<sensor_msgs::msg::LaserScan>(
    i==0?"/lidar_front/scan":"/lidar_rear/scan",[this,i](sensor_msgs::msg::LaserScan::ConstSharedPtr m){scan(i,*m);},rclcpp::SensorDataQoS());
  display_=n->create_publisher<nav_msgs::msg::OccupancyGrid>("~/observation_space",rclcpp::QoS(1).transient_local());display_->on_activate();
  free_display_=n->create_publisher<nav_msgs::msg::GridCells>("~/observed_free",rclcpp::QoS(1).transient_local());free_display_->on_activate();
}
void ObservationLayer::scan(std::size_t sensor,const sensor_msgs::msg::LaserScan & m){
  std::lock_guard<std::mutex> lock(mutex_);const double now=clock_->now().seconds(),stamp=rclcpp::Time(m.header.stamp).seconds();
  auto bad=[&](const std::string & why){space_.invalidate(sensor,why);};
  if(m.header.frame_id!=(sensor==0?"lidar_front_link":"lidar_rear_link")){bad("wrong_scan_frame");return;}
  if(m.ranges.size()<50||m.ranges.size()>10000||!std::isfinite(m.angle_min)||!std::isfinite(m.angle_increment)||
    m.angle_increment<=0||m.angle_increment>.02||!std::isfinite(m.range_min)||m.range_min<.05||
    !std::isfinite(m.range_max)||m.range_max<=m.range_min||!std::isfinite(m.time_increment)||m.time_increment<0||
    m.time_increment*m.ranges.size()>.2){bad("invalid_scan_metadata");return;}
  if(stamp<=0||now-stamp<0||now-stamp>space_.settings().scan_timeout){bad("invalid_or_stale_stamp");return;}
  std::vector<ObservedRay> rays;
  try{
    Pose sensor_pose,body;double previous=-1;
    for(std::size_t j=0;j<m.ranges.size();++j){const double r=m.ranges[j];if(!std::isfinite(r)||r<m.range_min||r>m.range_max)continue;
      const auto time=rclcpp::Time(m.header.stamp)+rclcpp::Duration::from_seconds(j*m.time_increment);
      if(time.seconds()>now){bad("future_ray_stamp");return;}
      if(time.seconds()!=previous){
        sensor_pose=planar(tf_->lookupTransform("map",m.header.frame_id,time,rclcpp::Duration::from_seconds(.05)).transform);
        body=planar(tf_->lookupTransform("map","base_footprint",time,rclcpp::Duration::from_seconds(.05)).transform);previous=time.seconds();
      }
      rays.push_back({{sensor_pose.x,sensor_pose.y},sensor_pose.yaw+m.angle_min+j*m.angle_increment,r,time.seconds(),r<m.range_max,body,m.range_min});
    }
    space_.observe(sensor,rays,stamp,now);
  }catch(const std::exception &){bad("missing_or_invalid_scan_transform");}
}
void ObservationLayer::updateBounds(double,double,double,double *x0,double *y0,double *x1,double *y1){
  // Keep the already-certified body region continuous during held pauses between
  // navigation goals. Only fresh map-to-body TF can do this; no surrounding cells
  // are initialized, and a pose/TF gap still invalidates continuity.
  if(observe_){
    try{
      auto t=tf_->lookupTransform("map","base_footprint",tf2::TimePointZero);
      std::lock_guard<std::mutex> lock(mutex_);const auto now=clock_->now();
      // Another gate call may advance continuity while TF/lock acquisition waits.
      // Capture time inside the same lock so callback ordering cannot look like
      // clock rollback and erase valid under-body evidence between goals.
      const double age=(now-rclcpp::Time(t.header.stamp)).seconds();
      if(age>=0&&age<=space_.settings().scan_timeout)space_.certify(getFootprint(),planar(t.transform),now.seconds());
    }catch(const std::exception &){} // The command gate reports missing transforms.
  }
  auto *m=layered_costmap_->getCostmap();*x0=std::min(*x0,m->getOriginX());*y0=std::min(*y0,m->getOriginY());
  *x1=std::max(*x1,m->getOriginX()+m->getSizeInCellsX()*m->getResolution());*y1=std::max(*y1,m->getOriginY()+m->getSizeInCellsY()*m->getResolution());
}
void ObservationLayer::updateCosts(nav2_costmap_2d::Costmap2D & m,int x0,int y0,int x1,int y1){
  std::unique_lock<std::mutex> lock(mutex_);if(!space_.ready()){current_=false;return;}
  Pose t;const bool global=layered_costmap_->getGlobalFrameID()=="map";
  try{if(!global)t=planar(tf_->lookupTransform("map",layered_costmap_->getGlobalFrameID(),tf2::TimePointZero).transform);}
  catch(const std::exception &){current_=false;return;}
  // Query only updated cells. Never copy the entire scanned map on the local
  // control path merely to read immutable identity.
  for(int y=y0;y<y1;++y)for(int x=x0;x<x1;++x){double wx,wy;m.mapToWorld(x,y,wx,wy);auto p=convert({wx,wy,0},t);
    const int cell=space_.observedCell(p.x,p.y);
    if(cell<0){m.setCost(x,y,255);continue;}
    if(cell==254)m.setCost(x,y,254);
    else if(cell==255){
      // Explicit optimistic planning representation, never execution clearance.
      // Raw identity is independent; live lethal hits and static obstacles win.
      if(observe_&&m.getCost(x,y)==255)m.setCost(x,y,0);
      else if(!observe_&&m.getCost(x,y)!=254)m.setCost(x,y,255);
    }
  }
  current_=true;const auto now=clock_->now();
  if(!observe_||now.seconds()-last_display_<(global?1.0:.5))return;
  last_display_=now.seconds();
  if(global){auto out=space_.display(now.seconds());out.header.stamp=now;lock.unlock();display_->publish(out);}
  else {
    nav_msgs::msg::GridCells cells;cells.header.frame_id="map";cells.header.stamp=now;
    cells.cell_width=cells.cell_height=m.getResolution()*2;
    for(unsigned y=0;y<m.getSizeInCellsY();y+=2)for(unsigned x=0;x<m.getSizeInCellsX();x+=2){double wx,wy;m.mapToWorld(x,y,wx,wy);auto p=convert({wx,wy,0},t);
      if(space_.observedFree(p.x,p.y,now.seconds())){geometry_msgs::msg::Point point;point.x=p.x;point.y=p.y;point.z=.02;cells.cells.push_back(point);}
    }
    lock.unlock();free_display_->publish(cells);
  }
}
void ObservationLayer::reset(){std::lock_guard<std::mutex> lock(mutex_);space_.resetContinuity();}
nav2_costmap_2d::Costmap2D ObservationLayer::identityMap(){std::lock_guard<std::mutex> lock(mutex_);return space_.rawMap();}
nav2_costmap_2d::Costmap2D ObservationLayer::observedMap(){std::lock_guard<std::mutex> lock(mutex_);const double now=clock_->now().seconds();return space_.executionMap(now,now);}
std::vector<Velocity> ObservationLayer::pending(double now){
  if(!admitted_.empty()&&now<admitted_.back().first){admitted_.clear();space_.resetContinuity();}
  while(!admitted_.empty()&&now-admitted_.front().first>space_.settings().latency)admitted_.pop_front();
  std::vector<Velocity> result;for(auto v:admitted_)result.push_back(v.second);return result;
}
std::string ObservationLayer::gate(const Footprint & f,Pose p,Velocity actual,Velocity request,double now){
  std::lock_guard<std::mutex> lock(mutex_);now=clock_->now().seconds();auto why=space_.health(now);
  try{if(layered_costmap_->getGlobalFrameID()!="map")p=convert(p,planar(tf_->lookupTransform("map",layered_costmap_->getGlobalFrameID(),rclcpp::Time(int64_t(now*1e9))).transform));}
  catch(const std::exception &){space_.resetContinuity();return "missing_execution_transform";}
  const bool footprint_clear=space_.certify(f,p,now);
  // Scan loss blocks motion, but fresh continuous pose feedback can retain only
  // the robot's already-certified footprint while it brakes/stays stationary.
  // Loss of pose continuity still discards the certificate in certify().
  if(!why.empty())return why;
  if(!footprint_clear)return "initial_or_current_footprint_unobserved";
  const auto queued=pending(now);
  if(admitted_.size()>=128)return "command_history_rate_limit";
  why=space_.check(f,p,actual,request,now,queued);
  if(why.empty())admitted_.push_back({now,request});
  return why;
}
std::string ObservationLayer::goalClear(const Footprint & f,Pose p,double now){
  std::lock_guard<std::mutex> lock(mutex_);now=clock_->now().seconds();auto why=space_.health(now);if(!why.empty())return why;
  try{if(layered_costmap_->getGlobalFrameID()!="map")p=convert(p,planar(tf_->lookupTransform("map",layered_costmap_->getGlobalFrameID(),rclcpp::Time(int64_t(now*1e9))).transform));}
  catch(const std::exception &){return "missing_goal_transform";}
  return space_.check(f,p,{}, {},now,pending(now));
}
std::shared_ptr<ObservationLayer> observationLayer(const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & m){
  for(auto &layer:*m->getLayeredCostmap()->getPlugins())if(auto p=std::dynamic_pointer_cast<ObservationLayer>(layer))return p;
  throw std::runtime_error("observe-before-enter requires observation_layer");
}
}
PLUGINLIB_EXPORT_CLASS(body_route_controller::ObservationLayer,nav2_costmap_2d::Layer)
