#pragma once
#include "body_route_controller/layout_navigation.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
using namespace body_route_controller;
using namespace body_route_controller::layout;
namespace layout_test {
constexpr double pi=3.14159265358979323846;
inline double cross(Point a,Point b) {return a.x*b.y-a.y*b.x;}
inline Point sub(Point a,Point b) {return {a.x-b.x,a.y-b.y};}
inline std::vector<Wall> nominal() {
  return {{"lower",{.3,-.9},{.3,-.35}},{"upper",{.3,.35},{.3,.9}},
    {"west",{-1,-1.2},{-1,1.2}},{"east",{2,-1.2},{2,1.2}},
    {"south",{-1,-1.2},{2,-1.2}},{"north",{-1,1.2},{2,1.2}}};
}
inline Scan scan(const std::vector<Wall> & world,Pose pose,Pose sensor,double stamp,int index,double noise=0) {
  Scan result;result.sensor_id=index==0?"front":"rear";result.sensor=sensor;result.stamp=stamp;
  result.angle_min=-pi;result.angle_increment=2*pi/1440;result.range_min=.1;result.range_max=4;
  const Point origin=transform(pose,{sensor.x,sensor.y});
  std::mt19937 rng(static_cast<unsigned>(stamp*100+index));std::normal_distribution<double> error(0,noise);
  for(int i=0;i<1440;++i) {
    const double a=pose.yaw+sensor.yaw+result.angle_min+i*result.angle_increment;const Point ray{std::cos(a),std::sin(a)};
    double hit=std::numeric_limits<double>::infinity();
    for(const auto & w:world) {
      const Point segment=sub(w.b,w.a),q=sub(w.a,origin);const double denominator=cross(ray,segment);
      if(std::abs(denominator)<1e-10) {continue;}
      const double distance=cross(q,segment)/denominator,t=cross(q,ray)/denominator;
      if(distance>=result.range_min&&distance<result.range_max&&t>=0&&t<=1) {hit=std::min(hit,distance);}
    }
    result.ranges.push_back(std::isfinite(hit)?hit+error(rng):hit);
  }
  return result;
}
inline std::vector<Frame> survey(const std::vector<Wall> & world,double noise=0,bool odom_error=false) {
  std::vector<Frame> frames;int index=0;
  for(Pose truth:{Pose{-.5,0,0},Pose{-.42,.08,.03},Pose{-.5,-.08,-.025},Pose{-.38,0,0}}) {
    Frame f;f.stamp=10+index*3;f.settled_seconds=1.2;f.odom=truth;
    if(odom_error) {f.odom.x+=index*.006;f.odom.y-=index*.004;f.odom.yaw+=index*.004;}
    f.scans={scan(world,truth,{.08,0,0},f.stamp,0,noise),scan(world,truth,{-.08,0,pi},f.stamp,1,noise)};
    frames.push_back(f);++index;
  }return frames;
}
inline Grid grid(const std::vector<Wall> & walls) {
  Grid g;g.width=320;g.height=260;g.resolution=.01;g.origin={-1.1,-1.3};g.cells.assign(g.width*g.height,0);
  for(const auto & w:walls) {
    const double length=std::hypot(w.b.x-w.a.x,w.b.y-w.a.y);const int count=std::ceil(length/.002);
    for(int i=0;i<=count;++i) {
      const double t=double(i)/count;const int at=g.index({w.a.x+t*(w.b.x-w.a.x),w.a.y+t*(w.b.y-w.a.y)});
      if(at>=0) {g.cells[at]=100;}
    }
  }return g;
}
inline Footprint footprint() {
  Footprint f;for(Point p:{Point{.18,.18},Point{.18,-.18},Point{-.18,-.18},Point{-.18,.18}}) {
    geometry_msgs::msg::Point q;q.x=p.x;q.y=p.y;f.push_back(q);
  }return f;
}
inline Portal portal() {return {"lower","upper",true,false,0,.35,.35};}
}
