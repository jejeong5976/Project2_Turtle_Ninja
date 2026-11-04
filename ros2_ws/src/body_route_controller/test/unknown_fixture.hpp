#pragma once
#include "body_route_controller/observation.hpp"
#include "body_route_controller/maze_planner.hpp"
#include <cmath>
namespace unknown_fixture {
using namespace body_route_controller;
constexpr double pi=3.141592653589793;
inline Footprint body(){Footprint out;for(auto q:std::vector<Point>{{-.205,-.205},{.205,-.205},{.205,.205},{-.205,.205}}){geometry_msgs::msg::Point p;p.x=q.x;p.y=q.y;out.push_back(p);}return out;}
inline nav_msgs::msg::OccupancyGrid world(){
  nav_msgs::msg::OccupancyGrid m;m.header.frame_id="map";m.info.width=220;m.info.height=140;m.info.resolution=.02;m.info.origin.orientation.w=1;
  m.data.assign(m.info.width*m.info.height,0);
  for(unsigned y=0;y<m.info.height;++y)for(unsigned x=0;x<m.info.width;++x)if(x<2||y<2||x+2>=m.info.width||y+2>=m.info.height)m.data[y*m.info.width+x]=100;
  return m;
}
inline nav_msgs::msg::OccupancyGrid partial(const nav_msgs::msg::OccupancyGrid &truth){auto m=truth;
  // Mask entire region including its obstacles: navigation cannot inspect truth there.
  for(unsigned y=2;y+2<m.info.height;++y)for(unsigned x=60;x+2<m.info.width;++x)m.data[y*m.info.width+x]=-1;
  return m;
}
inline int at(const nav_msgs::msg::OccupancyGrid &m,double x,double y){const int u=std::floor(x/m.info.resolution),v=std::floor(y/m.info.resolution);
  if(u<0||v<0||u>=int(m.info.width)||v>=int(m.info.height)){return 100;}
  return m.data[v*m.info.width+u];}
inline std::vector<ObservedRay> scan(const nav_msgs::msg::OccupancyGrid &truth,Pose p,int sensor,double stamp,double max_range=.65,bool partial_view=false){
  const double offset=sensor==0?.1145:-.1145;Point origin{p.x+offset*std::cos(p.yaw),p.y+offset*std::sin(p.yaw)};std::vector<ObservedRay> out;
  for(int j=0;j<720;++j){double a=-pi+j*2*pi/720;
    if(partial_view&&std::abs(wrap(a-p.yaw))<.45)continue;
    double range=.05;for(;range<max_range;range+=truth.info.resolution*.25)if(at(truth,origin.x+range*std::cos(a),origin.y+range*std::sin(a))>0)break;
    out.push_back({origin,a,std::min(range,max_range),stamp,range<max_range,p});
  }return out;
}
inline void observe(ObservationSpace &s,const nav_msgs::msg::OccupancyGrid &truth,Pose p,double stamp,double range=.65,bool partial_view=false){
  for(int i=0;i<2;++i)s.observe(i,scan(truth,p,i,stamp,range,partial_view),stamp,stamp+.001);
}
inline void obstacle(nav_msgs::msg::OccupancyGrid &m,double x0,double y0,double x1,double y1){
  for(unsigned y=0;y<m.info.height;++y)for(unsigned x=0;x<m.info.width;++x)if(x*m.info.resolution>=x0&&x*m.info.resolution<=x1&&y*m.info.resolution>=y0&&y*m.info.resolution<=y1)m.data[y*m.info.width+x]=100;
}
}
