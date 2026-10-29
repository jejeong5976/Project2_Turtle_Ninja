#pragma once
#include "scan_core.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace workbench {
inline Options scan_options(rclcpp::Node &node) {
  Options o;
  o.bins = node.declare_parameter<int>("scan_bins", o.bins);
  o.max_linear = node.declare_parameter<double>("max_linear", o.max_linear);
  o.max_angular = node.declare_parameter<double>("max_angular", o.max_angular);
  o.speed = std::min(o.speed, o.max_linear); o.turn_rate = std::min(o.turn_rate, o.max_angular);
  o.max_gap = node.declare_parameter<double>("max_odom_gap", o.max_gap);
  o.max_age = node.declare_parameter<double>("max_age", o.max_age);
  o.max_skew = node.declare_parameter<double>("max_skew", o.max_skew);
  o.endpoint_error = node.declare_parameter<double>("max_endpoint_error", o.endpoint_error);
  o.origin_shift = node.declare_parameter<double>("max_origin_shift", o.origin_shift);
  o.min_range = node.declare_parameter<double>("min_range", o.min_range);
  o.max_range = node.declare_parameter<double>("max_range", o.max_range);
  o.neighbors = node.declare_parameter<int>("filter_neighbors", o.neighbors);
  o.filter_radius = node.declare_parameter<double>("filter_radius", o.filter_radius);
  o.protect_radius = node.declare_parameter<double>("protect_radius", o.protect_radius);
  o.validate(); return o;
}
inline double planar_yaw(const geometry_msgs::msg::Quaternion &q) {
  const double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
  require(std::isfinite(norm) && std::abs(norm-1)<1e-3 && std::abs(q.x)<1e-3 && std::abs(q.y)<1e-3,
    "Invalid planar quaternion");
  return std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
}
inline lidar_motion::Odom decode_odom(const nav_msgs::msg::Odometry &m) {
  const auto &p=m.pose.pose.position;
  require(m.header.frame_id=="odom" && m.child_frame_id=="base_footprint" &&
    std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) && std::abs(p.z)<1e-3,
    "Invalid odometry frame/position");
  return {rclcpp::Time(m.header.stamp).nanoseconds(), {p.x,p.y,planar_yaw(m.pose.pose.orientation)}};
}
inline sensor_msgs::msg::PointCloud2 cloud(const std::vector<Ray> &rays,
    int64_t stamp, const std::string &frame, float z=0) {
  sensor_msgs::msg::PointCloud2 out; out.header.stamp=rclcpp::Time(stamp); out.header.frame_id=frame;
  sensor_msgs::PointCloud2Modifier modifier(out);
  using Field=sensor_msgs::msg::PointField;
  modifier.setPointCloud2Fields(9, "x",1,Field::FLOAT32,"y",1,Field::FLOAT32,"z",1,Field::FLOAT32,
    "origin_x",1,Field::FLOAT32,"origin_y",1,Field::FLOAT32,"origin_z",1,Field::FLOAT32,
    "source_id",1,Field::INT32,"input_bin",1,Field::INT32,"acquisition_offset",1,Field::FLOAT64);
  modifier.resize(rays.size()); out.is_dense=true;
  sensor_msgs::PointCloud2Iterator<float> x(out,"x"),y(out,"y"),zi(out,"z"),ox(out,"origin_x"),oy(out,"origin_y"),oz(out,"origin_z");
  sensor_msgs::PointCloud2Iterator<int32_t> source(out,"source_id"),input(out,"input_bin");
  sensor_msgs::PointCloud2Iterator<double> time(out,"acquisition_offset");
  for (const auto &r:rays) {
    *x=r.endpoint.x; *y=r.endpoint.y; *zi=z; *ox=r.origin.x; *oy=r.origin.y; *oz=z;
    *source=r.source; *input=r.input; *time=r.acquisition_offset;
    ++x;++y;++zi;++ox;++oy;++oz;++source;++input;++time;
  }
  return out;
}
inline std::vector<Ray> decode_cloud(const sensor_msgs::msg::PointCloud2 &m, int expected_side) {
  require(m.height==1 && m.width<=400 && !m.is_bigendian && m.point_step==40 &&
    m.row_step==m.width*m.point_step && m.data.size()==m.row_step, "Invalid ray cloud layout");
  const std::array<std::string,9> names{"x","y","z","origin_x","origin_y","origin_z","source_id","input_bin","acquisition_offset"};
  require(m.fields.size()==names.size(), "Invalid ray cloud fields");
  for(size_t i=0;i<names.size();++i) require(m.fields[i].name==names[i] && m.fields[i].offset==i*4 && m.fields[i].count==1 &&
    m.fields[i].datatype==(i<6?sensor_msgs::msg::PointField::FLOAT32:(i<8?sensor_msgs::msg::PointField::INT32:sensor_msgs::msg::PointField::FLOAT64)),
    "Unexpected ray cloud field");
  std::vector<Ray> out;
  sensor_msgs::PointCloud2ConstIterator<float> x(m,"x"),y(m,"y"),z(m,"z"),ox(m,"origin_x"),oy(m,"origin_y"),oz(m,"origin_z");
  sensor_msgs::PointCloud2ConstIterator<int32_t> source(m,"source_id"),input(m,"input_bin");
  sensor_msgs::PointCloud2ConstIterator<double> time(m,"acquisition_offset");
  for(size_t i=0;i<m.width;++i,++x,++y,++z,++ox,++oy,++oz,++source,++input,++time) {
    require(std::isfinite(*x)&&std::isfinite(*y)&&std::isfinite(*ox)&&std::isfinite(*oy)&&
      std::abs(*z)<1e-6&&std::abs(*oz)<1e-6&&*source==expected_side&&*input>=0&&*input<400&&
      std::isfinite(*time)&&*time<=0&&*time>=-.2,"Invalid ray cloud value");
    out.push_back({{*ox,*oy},{*x,*y},*source,*input,*time});
  }
  return out;
}
inline void status(const rclcpp::Publisher<std_msgs::msg::String>::SharedPtr &publisher, const std::string &s) {
  std_msgs::msg::String message; message.data=s; publisher->publish(message);
}
}  // namespace workbench
