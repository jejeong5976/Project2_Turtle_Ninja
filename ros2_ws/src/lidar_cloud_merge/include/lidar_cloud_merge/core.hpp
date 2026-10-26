#pragma once
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <array>
#include <algorithm>
#include <cmath>
#include <vector>
#include <stdexcept>
namespace lidar_cloud_merge {
using Point = std::array<float, 5>;
// Axis-aligned footprint in the output frame; all heights are excluded.
struct BodyMask {
  double center_x=0.110, center_y=0.005;
  double length=0.256, width=0.256, margin=0.010;
  void validate() const {
    if (!std::isfinite(center_x)||!std::isfinite(center_y)||
        !std::isfinite(length)||!std::isfinite(width)||!std::isfinite(margin)||
        length<=0||width<=0||margin<0||
        !std::isfinite(std::abs(center_x)+length/2+margin)||
        !std::isfinite(std::abs(center_y)+width/2+margin))
      throw std::runtime_error("Invalid body footprint or margin");
  }
  bool contains(const Point &p) const {
    // Match bounds to PointCloud2 float32 precision, including boundary points.
    return p[0]>=float(center_x-length/2-margin)&&p[0]<=float(center_x+length/2+margin)&&
           p[1]>=float(center_y-width/2-margin)&&p[1]<=float(center_y+width/2+margin);
  }
};
inline size_t exclude_body(std::vector<Point>& points,const BodyMask& mask) {
  mask.validate();
  const auto before=points.size();
  points.erase(std::remove_if(points.begin(),points.end(),
    [&mask](const Point&p){return mask.contains(p);}),points.end());
  return before-points.size();
}
inline void append(const sensor_msgs::msg::LaserScan &s,
                   const geometry_msgs::msg::Transform &t, float source,
                   std::vector<Point> &out) {
  if (!std::isfinite(s.angle_min) || !std::isfinite(s.angle_increment) ||
      !std::isfinite(s.range_min) || !std::isfinite(s.range_max) ||
      s.range_min < 0 || s.range_max <= s.range_min) throw std::runtime_error("Invalid scan metadata");
  tf2::Quaternion q(t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w);
  if (!std::isfinite(q.length2()) || q.length2()<1e-12 ||
      !std::isfinite(t.translation.x)||!std::isfinite(t.translation.y)||!std::isfinite(t.translation.z))
    throw std::runtime_error("Invalid transform");
  q.normalize();
  tf2::Transform tf(q, tf2::Vector3(t.translation.x,t.translation.y,t.translation.z));
  for (size_t i=0;i<s.ranges.size();++i) {
    double r=s.ranges[i], a=double(s.angle_min)+i*double(s.angle_increment);
    if (!std::isfinite(r)||r<s.range_min||r>s.range_max) continue;
    auto p=tf*tf2::Vector3(r*std::cos(a),r*std::sin(a),0);
    float intensity=i<s.intensities.size()&&std::isfinite(s.intensities[i])?s.intensities[i]:0;
    out.push_back({float(p.x()),float(p.y()),float(p.z()),intensity,source});
  }
}
inline sensor_msgs::msg::PointCloud2 cloud(const std::vector<Point>& points,
                                         const std_msgs::msg::Header& header) {
  sensor_msgs::msg::PointCloud2 msg;msg.header=header;
  sensor_msgs::PointCloud2Modifier mod(msg);
  mod.setPointCloud2Fields(5,"x",1,7,"y",1,7,"z",1,7,"intensity",1,7,"source_id",1,7);
  mod.resize(points.size());msg.is_dense=true;
  sensor_msgs::PointCloud2Iterator<float> x(msg,"x"),y(msg,"y"),z(msg,"z"),i(msg,"intensity"),s(msg,"source_id");
  for(auto&p:points){*x=p[0];*y=p[1];*z=p[2];*i=p[3];*s=p[4];++x;++y;++z;++i;++s;}
  return msg;
}
// Both input stamps must be recent, nonfuture and sufficiently close.
inline bool eligible(double now,double a,double b,double age,double skew) {
  return std::isfinite(now)&&std::isfinite(a)&&std::isfinite(b)&&
    now>=a&&now>=b&&now-a<=age&&now-b<=age&&std::abs(a-b)<=skew;
}
}
