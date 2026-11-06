// Localization-only mask. It never publishes to either safety scan topic.
#include "body_route_controller/start_localization.hpp"
#include "body_route_controller/scan_timing.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Matrix3x3.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <iostream>

namespace sl=body_route_controller::start_localization;
class StartScanFilter:public rclcpp::Node {
public:
  StartScanFilter():Node("start_scan_filter"),buffer_(get_clock()),listener_(buffer_) {
    if(!declare_parameter<bool>("reviewed_start",false))
      throw std::runtime_error("Explicit reviewed_start=true is required for this startup-only mask");
    cfg_=sl::readConfig(declare_parameter<std::string>("config",""));
    output_=create_publisher<sensor_msgs::msg::LaserScan>("/localization/start_scan",rclcpp::SensorDataQoS());
    status_=create_publisher<std_msgs::msg::String>("/localization/start_status",rclcpp::QoS(1).transient_local());
    odom_=create_subscription<nav_msgs::msg::Odometry>("/odom",rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg){onOdom(*msg);});
    hold_=create_subscription<std_msgs::msg::Bool>("/mission/hold",rclcpp::QoS(10),
      [this](std_msgs::msg::Bool::ConstSharedPtr msg){if(!msg->data) {disabled_=true;report("disabled: hold released; restart required");}});
    scan_=create_subscription<sensor_msgs::msg::LaserScan>("/lidar_rear/scan",rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg){onScan(*msg);});
    report("waiting: stationary odometry and rear scan; this component is not a motor interlock");
  }
private:
  void report(const std::string & text) {
    std_msgs::msg::String msg;msg.data=text;status_->publish(msg);
  }
  void onOdom(const nav_msgs::msg::Odometry & m) {
    const auto & p=m.pose.pose.position;const auto & q=m.pose.pose.orientation;
    const double yaw=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
    const double speed=std::hypot(m.twist.twist.linear.x,m.twist.twist.linear.y);
    const double rate=m.twist.twist.angular.z;
    const double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
    if(m.header.frame_id!="odom" || m.child_frame_id!="base_footprint" ||
      !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(yaw) ||
      !std::isfinite(speed) || !std::isfinite(rate) || !std::isfinite(norm) ||
      std::abs(norm-1)>.01) {have_odom_=false;return;}
    const sl::Pose pose{p.x,p.y,yaw};
    if(!have_anchor_) {anchor_=pose;have_anchor_=true;}
    if(speed>.001 || std::abs(rate)>.01 || std::hypot(p.x-anchor_[0],p.y-anchor_[1])>.01 ||
      std::abs(std::remainder(yaw-anchor_[2],2*std::acos(-1)))>.01) disabled_=true;
    odom_stamp_=rclcpp::Time(m.header.stamp).seconds();have_odom_=true;
  }
  void onScan(const sensor_msgs::msg::LaserScan & scan) {
    if(disabled_) {report("disabled: motion or hold release observed; restart required");return;}
    const double time=now().seconds();
    if(!have_odom_ || time-odom_stamp_<0 || time-odom_stamp_>.3) {report("waiting: fresh stationary odometry");return;}
    const auto interval=body_route_controller::scanInterval(rclcpp::Time(scan.header.stamp).seconds(),
      scan.time_increment,scan.ranges.size(),time,.3);
    if(scan.header.frame_id!="lidar_rear_link" || !interval.error.empty() ||
      !std::isfinite(scan.angle_min) || !std::isfinite(scan.angle_increment) || scan.angle_increment<=0 ||
      !std::isfinite(scan.angle_max) || !std::isfinite(scan.scan_time) || scan.scan_time<0 ||
      scan.angle_increment*(scan.ranges.size()-1)>2*std::acos(-1)+.01 ||
      std::abs(scan.angle_max-(scan.angle_min+(scan.ranges.size()-1)*scan.angle_increment))>.001 ||
      !std::isfinite(scan.range_min) || !std::isfinite(scan.range_max) ||
      !(scan.range_min>=0 && scan.range_max>scan.range_min) ||
      (!scan.intensities.empty() && scan.intensities.size()!=scan.ranges.size())) {
      report("rejected: rear scan frame, timing or metadata");return;
    }
    sl::Pose mount{};
    try {
      const auto t=buffer_.lookupTransform("base_footprint","lidar_rear_link",tf2::TimePointZero).transform;
      const auto & q=t.rotation;double roll,pitch,yaw;
      const double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
      if(!std::isfinite(norm) || std::abs(norm-1)>.01 || !std::isfinite(t.translation.z))
        throw std::runtime_error("Invalid rear TF quaternion/translation");
      tf2::Matrix3x3(tf2::Quaternion(q.x,q.y,q.z,q.w)).getRPY(roll,pitch,yaw);
      mount={t.translation.x,t.translation.y,yaw};
      if(!std::isfinite(mount[0]) || !std::isfinite(mount[1]) || !std::isfinite(yaw) ||
        !std::isfinite(roll) || !std::isfinite(pitch) || std::abs(roll)>.001 || std::abs(pitch)>.001 ||
        std::hypot(mount[0]-cfg_.rear_mount[0],mount[1]-cfg_.rear_mount[1])>.002 ||
        std::abs(std::remainder(yaw-cfg_.rear_mount[2],2*std::acos(-1)))>.002)
        throw std::runtime_error("Rear TF differs from reviewed mount");
    } catch(const std::exception &) {report("rejected: missing or mismatched rear mount TF");return;}
    auto filtered=scan;
    std::array<std::vector<sl::Point>,2> support;
    for(std::size_t i=0;i<scan.ranges.size();++i) {
      const double r=scan.ranges[i],a=scan.angle_min+i*scan.angle_increment;
      const auto p=sl::transform({r*std::cos(a),r*std::sin(a)},mount);
      const int wall=std::isfinite(r) && r>=scan.range_min && r<=scan.range_max?
        sl::select(p,{mount[0],mount[1]},cfg_):-1;
      if(wall<0) {
        filtered.ranges[i]=std::numeric_limits<float>::quiet_NaN();
        if(!filtered.intensities.empty()) filtered.intensities[i]=0;
      } else support[wall].push_back(p);
    }
    for(const auto & points:support) {
      double span=0;
      for(const auto & a:points) for(const auto & b:points)
        span=std::max(span,std::hypot(a[0]-b[0],a[1]-b[1]));
      if(points.size()<2*cfg_.min_points || span<cfg_.min_span) {
        report("rejected: insufficient support for both fixed walls");return;
      }
    }
    output_->publish(filtered);
    report("selected left="+std::to_string(support[0].size())+" rear="+std::to_string(support[1].size()));
  }
  sl::Config cfg_{};sl::Pose anchor_{};
  bool disabled_=false,have_odom_=false,have_anchor_=false;double odom_stamp_=0;
  tf2_ros::Buffer buffer_;tf2_ros::TransformListener listener_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr output_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr hold_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_;
};
int main(int argc,char ** argv) {
  rclcpp::init(argc,argv);
  try {rclcpp::spin(std::make_shared<StartScanFilter>());}
  catch(const std::exception & e) {std::cerr<<e.what()<<'\n';rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
