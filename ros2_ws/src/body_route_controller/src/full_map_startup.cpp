// Explicit stationary capture / reviewed-pose initialization. Never sends motion or hold.
#include "body_route_controller/scan_timing.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <nlohmann/json.hpp>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <thread>

using Json = nlohmann::json;
using Pose = std::array<double, 3>;
using namespace std::chrono_literals;
namespace {
double yaw(const geometry_msgs::msg::Quaternion & q) {
  const double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
  if (!std::isfinite(norm) || std::abs(norm-1)>.01 || std::abs(q.x)>.001 || std::abs(q.y)>.001)
    throw std::runtime_error("Invalid/nonplanar orientation");
  return std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
}
bool close(const Pose & a,const Pose & b,double xy,double angle) {
  return std::hypot(a[0]-b[0],a[1]-b[1])<=xy &&
    std::abs(std::remainder(a[2]-b[2],2*std::acos(-1)))<=angle;
}
void save(const std::filesystem::path & p,const Json & j) {
  if (std::filesystem::exists(p)) throw std::runtime_error("Output must be new");
  std::ofstream out(p);out<<j.dump(2)<<'\n';out.close();
  if (!out) throw std::runtime_error("Output write failed");
}
Json read(const char * path) { Json j;std::ifstream(path)>>j;return j; }
}

class Stationary : public rclcpp::Node {
public:
  Stationary():Node("full_map_startup"),tf_(get_clock()),listener_(tf_) {
    odom_sub_=create_subscription<nav_msgs::msg::Odometry>("/odom",rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
        try {
          const auto & p=m->pose.pose.position;const auto & v=m->twist.twist;
          Pose pose{p.x,p.y,yaw(m->pose.pose.orientation)};
          if (m->header.frame_id!="odom" || m->child_frame_id!="base_footprint" ||
              !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(v.linear.x) ||
              !std::isfinite(v.linear.y) || !std::isfinite(v.angular.z)) throw std::runtime_error("Invalid odometry");
          if (odom_stamp_==0) anchor_=pose;
          if (std::hypot(v.linear.x,v.linear.y)>.001 || std::abs(v.angular.z)>.01 ||
              !close(anchor_,pose,.01,.01)) failed_=true;
          odom_=pose;odom_stamp_=rclcpp::Time(m->header.stamp).seconds();
        } catch (const std::exception &) {failed_=true;}
      });
    command_=create_subscription<geometry_msgs::msg::TwistStamped>("/cmd_vel",10,
      [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr m) {
        const auto & v=m->twist;
        if (!std::isfinite(v.linear.x)||!std::isfinite(v.linear.y)||!std::isfinite(v.angular.z)||
            std::abs(v.linear.x)+std::abs(v.linear.y)+std::abs(v.angular.z)>1e-10) failed_=true;
        command_stamp_=rclcpp::Time(m->header.stamp).seconds();
      });
    hold_=create_subscription<std_msgs::msg::Bool>("/mission/hold",10,
      [this](std_msgs::msg::Bool::ConstSharedPtr m){if(!m->data) failed_=true;});
    for (int i=0;i<2;++i) scans_sub_[i]=create_subscription<sensor_msgs::msg::LaserScan>(
      i==0?"/lidar_front/scan":"/lidar_rear/scan",rclcpp::SensorDataQoS(),
      [this,i](sensor_msgs::msg::LaserScan::ConstSharedPtr m){scans_[i]=m;});
  }
  void checkParameters() {
    auto hold=std::make_shared<rclcpp::SyncParametersClient>(shared_from_this(),"/cmd_vel_hold");
    auto amcl=std::make_shared<rclcpp::SyncParametersClient>(shared_from_this(),"/amcl");
    if (!hold->wait_for_service(3s) || !amcl->wait_for_service(3s) ||
        !hold->get_parameters({"start_hold"},3s).at(0).as_bool() ||
        amcl->get_parameters({"scan_topic"},3s).at(0).as_string()!="/lidar_front/scan")
      throw std::runtime_error("Require start_hold=true and front-lidar AMCL");
    const double timeout=hold->get_parameters({"hold_timeout_s"},3s).at(0).as_double();
    if(!std::isfinite(timeout) || timeout<=0 || timeout>3)
      throw std::runtime_error("Require a positive hold timeout no longer than 3 s");
  }
  bool ready() const {
    if (failed_) throw std::runtime_error("Motion, hold release or invalid input observed; start a new capture");
    const auto owners=get_publishers_info_by_topic("/cmd_vel");
    if(count_publishers("/mission/hold")!=0 || owners.size()!=1 ||
        owners[0].node_name()!="cmd_vel_hold" || owners[0].node_namespace()!="/")
      throw std::runtime_error("Require idle held startup with sole cmd_vel_hold command ownership");
    const double t=now().seconds();
    return odom_stamp_>0 && command_stamp_>0 && t>=odom_stamp_ && t-odom_stamp_<=.3 &&
      t>=command_stamp_ && t-command_stamp_<=.3;
  }
  Json snapshot(const Pose & seed) {
    if (!ready()) throw std::runtime_error("Missing fresh stationary odometry/zero command");
    Json result={{"pose",seed},{"odom",odom_},{"held",true},{"live",true},{"speed",0.0},{"wz",0.0},
      {"capture_ros_time",now().seconds()},{"scans",Json::array()}};
    for (int i=0;i<2;++i) {
      if (!scans_[i]) throw std::runtime_error("Missing dual scans");
      const auto & m=*scans_[i];const std::string frame=i==0?"lidar_front_link":"lidar_rear_link";
      const auto interval=body_route_controller::scanInterval(rclcpp::Time(m.header.stamp).seconds(),
        m.time_increment,m.ranges.size(),now().seconds(),.3);
      if (m.header.frame_id!=frame || !interval.error.empty() || !std::isfinite(m.angle_min) ||
          !std::isfinite(m.angle_increment) || m.angle_increment<=0 || !std::isfinite(m.angle_max) ||
          std::abs(m.angle_max-(m.angle_min+(m.ranges.size()-1)*m.angle_increment))>.001 ||
          !std::isfinite(m.range_min) || !std::isfinite(m.range_max) || m.range_min<0 || m.range_max<=m.range_min)
        throw std::runtime_error("Stale/invalid dual scan; corrected timing/metadata required");
      const auto t=tf_.lookupTransform("base_footprint",frame,rclcpp::Time(m.header.stamp)).transform;
      const double a=yaw(t.rotation);
      if (!std::isfinite(t.translation.x)||!std::isfinite(t.translation.y)||!std::isfinite(t.translation.z))
        throw std::runtime_error("Invalid lidar TF");
      Json points=Json::array();
      for (std::size_t k=0;k<m.ranges.size();++k) {
        const double r=m.ranges[k];if(!std::isfinite(r)||r<m.range_min||r>m.range_max)continue;
        const double angle=a+m.angle_min+k*m.angle_increment;
        const double x=t.translation.x+r*std::cos(angle),y=t.translation.y+r*std::sin(angle);
        points.push_back({seed[0]+std::cos(seed[2])*x-std::sin(seed[2])*y,
                          seed[1]+std::sin(seed[2])*x+std::cos(seed[2])*y});
      }
      result["scans"].push_back(points);result["scan_stamps"].push_back(rclcpp::Time(m.header.stamp).seconds());
    }
    if (std::abs(double(result["scan_stamps"][0])-double(result["scan_stamps"][1]))>.15)
      throw std::runtime_error("Dual scan pair too far apart");
    return result;
  }
  void initialize(const Json & first,const Json & second,rclcpp::executors::SingleThreadedExecutor & executor) {
    const Pose pose=first.at("best").get<Pose>(),other=second.at("best").get<Pose>();
    for(double v:pose)if(!std::isfinite(v))throw std::runtime_error("Nonfinite fit");
    for(double v:other)if(!std::isfinite(v))throw std::runtime_error("Nonfinite fit");
    if(!ready() || !close(pose,other,.02,std::acos(-1)/180))throw std::runtime_error("Fits not repeatable");
    if(first.at("map_metadata")!=second.at("map_metadata"))throw std::runtime_error("Different maps");
    const auto a=first.at("source_snapshot"),b=second.at("source_snapshot");
    if(double(b.at("capture_ros_time"))-double(a.at("capture_ros_time"))<.5)
      throw std::runtime_error("Require two independent captures at least 0.5 s apart");
    auto check_fresh=[&]() {
    for(const auto & j:{a,b}) {
      const double age=now().seconds()-double(j.at("capture_ros_time"));
      if(!std::isfinite(age) || age<0 || age>180 || !j.at("held").get<bool>() ||
          !close(odom_,j.at("odom").get<Pose>(),.01,.01))
        throw std::runtime_error("Stale fit or robot moved since capture; capture and fit again");
    }
    };
    check_fresh();
    geometry_msgs::msg::PoseWithCovarianceStamped m;m.header.frame_id="map";m.header.stamp=now();
    m.pose.pose.position.x=pose[0];m.pose.pose.position.y=pose[1];
    m.pose.pose.orientation.z=std::sin(pose[2]/2);m.pose.pose.orientation.w=std::cos(pose[2]/2);
    m.pose.covariance[0]=.01;m.pose.covariance[7]=.01;m.pose.covariance[35]=std::pow(5*std::acos(-1)/180,2);
    publisher_=create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose",10);
    const auto end=std::chrono::steady_clock::now()+3s;
    while(publisher_->get_subscription_count()==0 && std::chrono::steady_clock::now()<end) {
      executor.spin_some();std::this_thread::sleep_for(10ms);
    }
    if(!ready() || publisher_->get_subscription_count()==0)throw std::runtime_error("Initializer not ready");
    check_fresh();
    m.header.stamp=now();publisher_->publish(m);
    std::cout<<"Published reviewed full-map pose; xy sigma=0.10 m, yaw sigma=5 deg; AMCL=/lidar_front/scan\n";
  }
private:
  tf2_ros::Buffer tf_;tf2_ros::TransformListener listener_;
  Pose anchor_{},odom_{};double odom_stamp_=0,command_stamp_=0;bool failed_=false;
  std::array<sensor_msgs::msg::LaserScan::ConstSharedPtr,2> scans_;
  std::array<rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr,2> scans_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr command_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr hold_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;
};

int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  try {
    if(argc!=6 || (std::string(argv[1])!="capture" && std::string(argv[1])!="initialize"))
      throw std::runtime_error("full_map_startup capture NEW_DIR COARSE_X COARSE_Y YAW_DEGREES | initialize FIT1 FIT2 --reviewed front");
    const bool capture=std::string(argv[1])=="capture";
    if(!capture && (std::string(argv[4])!="--reviewed" || std::string(argv[5])!="front"))
      throw std::runtime_error("Explicit reviewed fit and front AMCL required");
    auto node=std::make_shared<Stationary>();node->checkParameters();
    rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(node);
    const auto end=std::chrono::steady_clock::now()+3200ms;
    while(std::chrono::steady_clock::now()<end) {executor.spin_some();std::this_thread::sleep_for(10ms);}
    if(capture) {
      Pose seed{std::stod(argv[3]),std::stod(argv[4]),std::stod(argv[5])*std::acos(-1)/180};
      for(double v:seed)if(!std::isfinite(v))throw std::runtime_error("Invalid current coarse placement");
      const auto first=node->snapshot(seed);
      const auto second_at=std::chrono::steady_clock::now()+700ms;
      while(std::chrono::steady_clock::now()<second_at) {executor.spin_some();std::this_thread::sleep_for(10ms);}
      const auto second=node->snapshot(seed);
      if(!std::filesystem::create_directory(argv[2]))throw std::runtime_error("Capture directory must be new");
      save(std::filesystem::path(argv[2])/"first.json",first);save(std::filesystem::path(argv[2])/"second.json",second);
      std::cout<<"Captured two fresh dual-lidar snapshots without AMCL/map TF or a historical fitted pose\n";
    } else {
      node->initialize(read(argv[2]),read(argv[3]),executor);
      const auto flush=std::chrono::steady_clock::now()+500ms;
      while(std::chrono::steady_clock::now()<flush) {executor.spin_some();std::this_thread::sleep_for(10ms);}
    }
    rclcpp::shutdown();return 0;
  } catch(const std::exception & e) {std::cerr<<e.what()<<'\n';rclcpp::shutdown();return 1;}
}
