#include "lidar_cloud_merge/core.hpp"
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <chrono>
#include <iostream>
class Merger : public rclcpp::Node {
  using Scan=sensor_msgs::msg::LaserScan;
  using Steady=std::chrono::steady_clock;
  Scan::ConstSharedPtr scans_[2];Steady::time_point arrivals_[2];
  rclcpp::Subscription<Scan>::SharedPtr subscriptions_[2];
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::unique_ptr<tf2_ros::TransformListener> listener_;
  std::string frame_;double max_age_,max_skew_;
  bool self_filter_;lidar_cloud_merge::BodyMask body_;
 public:
  Merger():Node("stationary_cloud_merge") {
    if(!declare_parameter("stationary_test",false))throw std::runtime_error("Requires stationary_test:=true; no motion compensation");
    frame_=declare_parameter<std::string>("target_frame","lidar_front_link");
    self_filter_=declare_parameter("self_filter_enabled",false);
    body_.center_x=declare_parameter("body_center_x",0.110);
    body_.center_y=declare_parameter("body_center_y",0.005);
    body_.length=declare_parameter("body_length",0.256);
    body_.width=declare_parameter("body_width",0.256);
    body_.margin=declare_parameter("body_margin",0.010);
    body_.validate();
    if(self_filter_ && frame_!="lidar_front_link")
      throw std::runtime_error("Body filter currently requires target_frame=lidar_front_link");
    if(self_filter_)RCLCPP_INFO(get_logger(),"Body filter: X [%.3f, %.3f], Y [%.3f, %.3f] m; all Z; frame %s",
      body_.center_x-body_.length/2-body_.margin,body_.center_x+body_.length/2+body_.margin,
      body_.center_y-body_.width/2-body_.margin,body_.center_y+body_.width/2+body_.margin,frame_.c_str());
    max_age_=declare_parameter("max_age",0.3);max_skew_=declare_parameter("max_skew",0.075);
    if(frame_.empty()||!std::isfinite(max_age_)||!std::isfinite(max_skew_)||max_age_<=0||max_skew_<0||max_skew_>max_age_)
      throw std::runtime_error("Invalid frame or freshness limits");
    buffer_=std::make_unique<tf2_ros::Buffer>(get_clock());
    listener_=std::make_unique<tf2_ros::TransformListener>(*buffer_);
    publisher_=create_publisher<sensor_msgs::msg::PointCloud2>("/lidar/points_merged",rclcpp::SensorDataQoS());
    for(int k=0;k<2;k++)subscriptions_[k]=create_subscription<Scan>(k?"/lidar_rear/scan":"/lidar_front/scan",rclcpp::SensorDataQoS(),[this,k](Scan::ConstSharedPtr s){scans_[k]=s;arrivals_[k]=Steady::now();publish_pair();});
    RCLCPP_WARN(get_logger(),"Stationary visualization only: provisional TF, no deskew; source_id 0=front, 1=rear");
  }
 private:
  void publish_pair(){
    if(!scans_[0]||!scans_[1])return;
    double stamps[2]={rclcpp::Time(scans_[0]->header.stamp).seconds(),rclcpp::Time(scans_[1]->header.stamp).seconds()};
    auto steady=Steady::now();
    if(!lidar_cloud_merge::eligible(now().seconds(),stamps[0],stamps[1],max_age_,max_skew_)||
       std::chrono::duration<double>(steady-arrivals_[0]).count()>max_age_||std::chrono::duration<double>(steady-arrivals_[1]).count()>max_age_){
      RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),5000,"Waiting for fresh scans within max_skew");
      scans_[stamps[0]<=stamps[1]?0:1].reset();return;
    }
    try {
      std::vector<lidar_cloud_merge::Point> points;
      for(int k=0;k<2;k++){
        if(scans_[k]->header.frame_id.empty())throw std::runtime_error("Empty scan frame");
        geometry_msgs::msg::Transform tf;
        if(scans_[k]->header.frame_id==frame_)tf.rotation.w=1;
        else tf=buffer_->lookupTransform(frame_,scans_[k]->header.frame_id,rclcpp::Time(scans_[k]->header.stamp)).transform;
        lidar_cloud_merge::append(*scans_[k],tf,k,points);
      }
      if(self_filter_){
        const auto removed=lidar_cloud_merge::exclude_body(points,body_);
        RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),5000,"Body filter: removed %zu, retained %zu points in latest pair",removed,points.size());
      }
      auto header=scans_[stamps[0]>=stamps[1]?0:1]->header;header.frame_id=frame_;
      publisher_->publish(lidar_cloud_merge::cloud(points,header));
    }catch(const std::exception&e){RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),5000,"Skipping cloud: %s",e.what());}
    scans_[0].reset();scans_[1].reset(); // Never reuse an old scan in a new cloud.
  }
};
int main(int argc,char**argv){rclcpp::init(argc,argv);try{rclcpp::spin(std::make_shared<Merger>());}catch(const std::exception&e){std::cerr<<e.what()<<std::endl;rclcpp::shutdown();return 1;}rclcpp::shutdown();return 0;}
