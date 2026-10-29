// Passive subscriber: no commands, serial access, TF publication or services.
#include "nt_timing_trace.hpp"
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
class Observer:public rclcpp::Node {
  nt_timing::Logger timing_{"observer"};
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
public:
  Observer():Node("timing_observer") {
    if(!timing_.enabled()||get_parameter("use_sim_time").as_bool())throw std::runtime_error("Require NT_TIMING_DIR and system time");
    subscriptions_.push_back(create_subscription<nav_msgs::msg::Odometry>("/odom",100,[this](nav_msgs::msg::Odometry::ConstSharedPtr m){
      const auto stamp=rclcpp::Time(m->header.stamp).nanoseconds();
      timing_.event(nt_timing::ObserverOdom,stamp,stamp,0,0,0,0,m->pose.pose.position.x,m->pose.pose.position.y,m->twist.twist.angular.z);
    }));
    subscriptions_.push_back(create_subscription<sensor_msgs::msg::LaserScan>("/lidar_front/scan_mapping",rclcpp::SensorDataQoS().keep_last(100),[this](sensor_msgs::msg::LaserScan::ConstSharedPtr m){
      const auto stamp=rclcpp::Time(m->header.stamp).nanoseconds();timing_.event(nt_timing::ScanReceived,stamp,stamp,m->ranges.size());
    }));
    subscriptions_.push_back(create_subscription<nav_msgs::msg::OccupancyGrid>("/map",rclcpp::QoS(1).reliable().transient_local(),[this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m){
      const auto stamp=rclcpp::Time(m->header.stamp).nanoseconds();timing_.event(nt_timing::MapReceived,stamp,stamp,m->info.width,m->info.height);
    }));
    subscriptions_.push_back(create_subscription<tf2_msgs::msg::TFMessage>("/tf",rclcpp::QoS(100).best_effort(),[this](tf2_msgs::msg::TFMessage::ConstSharedPtr m){
      for(const auto &t:m->transforms)if(t.header.frame_id=="map"&&t.child_frame_id=="odom"){
        const auto stamp=rclcpp::Time(t.header.stamp).nanoseconds();timing_.event(nt_timing::MapOdomReceived,stamp,stamp);
      }
    }));
  }
};
int main(int argc,char **argv){
 rclcpp::init(argc,argv);
 try{auto node=std::make_shared<Observer>();rclcpp::spin(node);node.reset();}
 catch(const std::exception &e){std::fprintf(stderr,"TIMING OBSERVER FAILED: %s\n",e.what());rclcpp::shutdown();return 1;}
 rclcpp::shutdown();return 0;
}
