#include "body_route_controller/profile.hpp"
#include "body_route_controller/core.hpp"
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <yaml-cpp/yaml.h>
#include <array>
#include <chrono>
#include <cmath>
#include <deque>
#include <filesystem>

using namespace std::chrono_literals;
namespace fs=std::filesystem;
using body_route_controller::Pose;
using body_route_controller::wrap;
namespace {
double yaw(const geometry_msgs::msg::Quaternion & q) {
  const double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
  if(!std::isfinite(norm)||std::abs(norm-1)>.01||std::abs(q.x)>.01||std::abs(q.y)>.01) {throw std::runtime_error("nonplanar or invalid quaternion");}
  return std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
}
YAML::Node values(double a,double b,double c) {YAML::Node n;n.push_back(a);n.push_back(b);n.push_back(c);return n;}
struct Odom {double stamp;Pose pose;double vx,vy,wz;};
}
// Passive recorder: no publishers, action clients, services, TF broadcast or drivers.
class Capture : public rclcpp::Node {
public:
  Capture():Node("capture_layout"),tf_(get_clock()),listener_(tf_) {
    if(!declare_parameter<bool>("fixed_start_verified",false)) {
      throw std::runtime_error("first capture must be at the template's fixed start: fixed_start_verified is required");
    }
    const fs::path input=declare_parameter<std::string>("layout_file","");
    output_=declare_parameter<std::string>("output_directory","");
    maximum_=declare_parameter<int>("max_keyframes",8);
    if(input.empty()||output_.empty()||maximum_<2||maximum_>32) {throw std::runtime_error("layout_file, new output_directory and 2..32 keyframes required");}
    document_=YAML::LoadFile(input);
    if(document_["schema_version"].as<int>()!=1||!document_["fixed_start"].IsSequence()||
      document_["fixed_start"].size()!=3||!document_["walls"].IsSequence()||document_["walls"].size()==0||document_["frames"]) {
      throw std::runtime_error("layout template must define schema_version, fixed_start, walls, map, and no frames");
    }
    fs::path map=document_["map"].as<std::string>();if(map.is_relative()) {map=input.parent_path()/map;}
    if(!fs::is_regular_file(map)) {throw std::runtime_error("template map missing");}
    document_["map"]=fs::absolute(map).string();document_["frames"]=YAML::Node(YAML::NodeType::Sequence);
    for(const auto & key:{"position_sigma_m","yaw_sigma_rad"}) {
      const double sigma=document_["start_uncertainty"][key].as<double>();
      if(!std::isfinite(sigma)||sigma<=0) {throw std::runtime_error("explicit positive start uncertainty is required");}
    }
    if(!fs::create_directory(output_)) {throw std::runtime_error("output_directory must be new");}
    odom_=create_subscription<nav_msgs::msg::Odometry>("/odom",rclcpp::SensorDataQoS(),[this](nav_msgs::msg::Odometry::ConstSharedPtr m){
      try {
        if(m->header.frame_id!="odom"||m->child_frame_id!="base_footprint") {throw std::runtime_error("unexpected odometry frame");}
        const auto & p=m->pose.pose.position;const auto & t=m->twist.twist;
        Odom o{rclcpp::Time(m->header.stamp).seconds(),{p.x,p.y,yaw(m->pose.pose.orientation)},t.linear.x,t.linear.y,t.angular.z};
        if(!std::isfinite(o.stamp)||!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(o.vx)||!std::isfinite(o.vy)||!std::isfinite(o.wz)||
          std::hypot(o.vx,o.vy)>.003||std::abs(o.wz)>.01||(!history_.empty()&&o.stamp<=history_.back().stamp)) {history_.clear();return;}
        history_.push_back(o);while(!history_.empty()&&o.stamp-history_.front().stamp>2) {history_.pop_front();}
      } catch(const std::exception &) {history_.clear();}
    });
    for(int i=0;i<2;++i) {subscriptions_[i]=create_subscription<sensor_msgs::msg::LaserScan>(i==0?"/lidar_front/scan":"/lidar_rear/scan",
      rclcpp::SensorDataQoS(),[this,i](sensor_msgs::msg::LaserScan::ConstSharedPtr m){scans_[i]=m;});}
    timer_=create_wall_timer(100ms,[this](){capture();});
    RCLCPP_INFO(get_logger(),"Passive capture: waiting for synchronized dual scans and one second stationary. No motion commands.");
  }
  bool done() const {return count_>=maximum_;}
private:
  void capture() {
    if(done()||!scans_[0]||!scans_[1]||history_.size()<10) {return;}
    const double a=rclcpp::Time(scans_[0]->header.stamp).seconds(),b=rclcpp::Time(scans_[1]->header.stamp).seconds();
    const double stamp=(a+b)/2,current=now().seconds();
    if(std::abs(a-b)>.05||current-std::min(a,b)>.3||std::max(a,b)>current+.02||stamp<=last_stamp_) {return;}
    if(stamp-history_.front().stamp<1||std::abs(history_.back().stamp-stamp)>.1) {return;}
    const Odom *nearest=&history_.front();
    for(const auto & o:history_) {if(std::abs(o.stamp-stamp)<std::abs(nearest->stamp-stamp)) {nearest=&o;}}
    if(std::abs(nearest->stamp-stamp)>.05) {return;}
    for(const auto & o:history_) {
      if(std::hypot(o.pose.x-nearest->pose.x,o.pose.y-nearest->pose.y)>.003||std::abs(wrap(o.pose.yaw-nearest->pose.yaw))>.01) {return;}
    }
    if(count_&&std::hypot(last_.x-nearest->pose.x,last_.y-nearest->pose.y)<.05&&std::abs(wrap(last_.yaw-nearest->pose.yaw))<.08) {return;}
    try {
      YAML::Node frame;frame["stamp"]=stamp;frame["settled_seconds"]=stamp-history_.front().stamp;
      frame["odom"]=values(nearest->pose.x,nearest->pose.y,nearest->pose.yaw);frame["measured_velocity"]=values(nearest->vx,nearest->vy,nearest->wz);
      for(int i=0;i<2;++i) {
        const auto & m=*scans_[i];const auto t=tf_.lookupTransform("base_footprint",m.header.frame_id,rclcpp::Time(m.header.stamp));
        if(std::abs(t.transform.translation.z)>.5) {throw std::runtime_error("unexpected lidar height");}
        YAML::Node scan;scan["sensor_id"]=i==0?"front":"rear";scan["stamp"]=rclcpp::Time(m.header.stamp).seconds();
        scan["sensor_pose"]=values(t.transform.translation.x,t.transform.translation.y,yaw(t.transform.rotation));
        scan["angle_min"]=m.angle_min;scan["angle_increment"]=m.angle_increment;scan["range_min"]=m.range_min;scan["range_max"]=m.range_max;
        for(float r:m.ranges) {scan["ranges"].push_back(static_cast<double>(r));}frame["scans"].push_back(scan);
      }
      document_["frames"].push_back(frame);++count_;last_=nearest->pose;last_stamp_=stamp;
      const auto path=output_/("survey_"+std::to_string(count_)+".yaml");
      body_route_controller::writeNew(path.string(),YAML::Dump(document_)+"\n");
      RCLCPP_INFO(get_logger(),"Captured %d/%d stationary keyframes: %s",count_,maximum_,path.c_str());
    } catch(const tf2::TransformException &) {return;}
  }
  tf2_ros::Buffer tf_;tf2_ros::TransformListener listener_;fs::path output_;YAML::Node document_;
  int maximum_=0,count_=0;double last_stamp_=-1;Pose last_;std::deque<Odom> history_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_;
  std::array<sensor_msgs::msg::LaserScan::ConstSharedPtr,2> scans_;
  std::array<rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr,2> subscriptions_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char ** argv) {
  rclcpp::init(argc,argv);int code=0;
  try {auto node=std::make_shared<Capture>();rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(node);
    while(rclcpp::ok()&&!node->done()) {executor.spin_once(100ms);}
  }catch(const std::exception & e) {RCLCPP_ERROR(rclcpp::get_logger("capture_layout"),"%s",e.what());code=1;}
  rclcpp::shutdown();return code;
}
