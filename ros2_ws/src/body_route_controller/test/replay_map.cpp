// Offline numerical replay against a trinary P5 occupancy map. No ROS node,
// clock, TF publisher, action client, serial device, or hardware driver.
#include "body_route_controller/core.hpp"
#include "body_route_controller/collision.hpp"
#include <nav2_costmap_2d/cost_values.hpp>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <iomanip>
using namespace body_route_controller;
int main(int argc,char **argv)
{
  if(argc!=8) {std::cerr<<"P5_MAP RESOLUTION ORIGIN_X ORIGIN_Y PATH INITIAL_YAW OUTPUT_CSV\n";return 2;}
  try {
    std::ifstream pgm(argv[1],std::ios::binary);std::string magic;unsigned width,height,maximum;
    pgm>>magic>>width>>height>>maximum;pgm.get();
    if(!pgm||magic!="P5"||maximum!=255||width>20000||height>20000) {throw std::runtime_error("unsupported map");}
    nav2_costmap_2d::Costmap2D map(width,height,std::stod(argv[2]),std::stod(argv[3]),std::stod(argv[4]),255);
    for(unsigned y=0;y<height;++y) {for(unsigned x=0;x<width;++x) {
      char pixel;if(!pgm.get(pixel)) {throw std::runtime_error("truncated map");}
      unsigned v=static_cast<unsigned char>(pixel);
      map.setCost(x,height-y-1,v<89?254:v>191?0:255);
    }}
    std::ifstream file(argv[5]);std::vector<Point> path;Pose goal;double x,y,a;
    while(file>>x>>y>>a) {path.push_back({x,y});goal={x,y,a};}
    if(path.size()<2) {throw std::runtime_error("missing path");}
    Footprint footprint;
    for(Point p:{Point{-.18,-.18},{.18,-.18},{.18,.18},{-.18,.18}}) {geometry_msgs::msg::Point q;q.x=p.x;q.y=p.y;footprint.push_back(q);}
    Pose pose{path.front().x,path.front().y,std::stod(argv[6])};Velocity measured,delay;Core core;
    std::ofstream csv(argv[7]);csv<<std::setprecision(12)<<"time,x,y,yaw,vx,vy,wz,mode\n";
    std::size_t first=0;double travel=0;int turns=0;std::string last;
    for(int step=0;step<10000;++step) {
      double best=1e9,searched=0;std::size_t nearest=first;
      for(std::size_t i=first;i<path.size();++i) {
        double d=std::hypot(pose.x-path[i].x,pose.y-path[i].y);
        if(d<best) {best=d;nearest=i;}
        if(i+1<path.size()) {searched+=std::hypot(path[i+1].x-path[i].x,path[i+1].y-path[i].y);}
        if(searched>.5) {break;}
      }
      first=nearest>0?nearest-1:0;std::vector<Point> local(path.begin()+first,path.end());
      auto r=core.step(pose,measured,local,goal,.05,.025);
      if(r.mode!=last&&(r.mode=="route_turn"||r.mode=="final_turn")) {++turns;}last=r.mode;
      csv<<step*.05<<','<<pose.x<<','<<pose.y<<','<<pose.yaw<<','<<measured.x<<','<<measured.y<<','<<measured.yaw<<','<<r.mode<<'\n';
      const double braking=std::max(std::hypot(measured.x,measured.y)/.05,std::abs(measured.yaw)/.2);
      if(!motionSafe(map,footprint,pose,measured,.3+braking)||!motionSafe(map,footprint,pose,r.command,2.0)||
        ((r.mode=="route_turn"||r.mode=="final_turn")&&!turnSafe(map,footprint,pose,r.heading))) {
        std::cerr<<"BLOCKED at "<<step*.05<<" s pose "<<pose.x<<' '<<pose.y<<' '<<pose.yaw<<" mode "<<r.mode<<'\n';return 1;
      }
      if(r.mode=="goal_stop"&&std::hypot(measured.x,measured.y)<.003&&std::abs(measured.yaw)<.01) {
        std::cout<<"PASS duration="<<step*.05<<" travel="<<travel<<" turn_phases="<<turns
          <<" goal_error="<<std::hypot(pose.x-goal.x,pose.y-goal.y)<<" yaw_error="<<std::abs(wrap(pose.yaw-goal.yaw))<<'\n';return 0;
      }
      measured.x+=.25*(delay.x-measured.x);measured.y+=.25*(delay.y-measured.y);measured.yaw+=.25*(delay.yaw-measured.yaw);delay=r.command;
      const double dx=.05*(std::cos(pose.yaw)*measured.x-std::sin(pose.yaw)*measured.y),dy=.05*(std::sin(pose.yaw)*measured.x+std::cos(pose.yaw)*measured.y);
      pose.x+=dx;pose.y+=dy;pose.yaw=wrap(pose.yaw+.05*measured.yaw);travel+=std::hypot(dx,dy);
    }
    throw std::runtime_error("replay timeout");
  }catch(const std::exception&e) {std::cerr<<e.what()<<'\n';return 1;}
}
