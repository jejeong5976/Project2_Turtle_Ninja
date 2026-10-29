// Offline verification of our synthetic scene only; never opens hardware or ROS sockets.
#include "ros_helpers.hpp"
#include <rosbag2_cpp/reader.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>
#include <set>

template<class T> T decode(const std::shared_ptr<rosbag2_storage::SerializedBagMessage> &m) {
  rclcpp::SerializedMessage data(*m->serialized_data);T out;
  rclcpp::Serialization<T> parser;parser.deserialize_message(&data,&out);return out;
}
int main(int argc,char **argv) {
  using workbench::require;
  try {
    require(argc==2||argc==3,"Usage: verify_mock_recording SESSION [FAULT_TIME_NS]");
    const std::filesystem::path directory(argv[1]);
    require(YAML::LoadFile((directory/"manifest.yaml").string())["session"]["hardware"].as<std::string>()=="mock","Synthetic verification accepts only mock sessions");
    const int64_t failure=argc==3?std::stoll(argv[2]):0;
    rosbag2_cpp::Reader reader;reader.open((directory/"bag").string());
    std::map<std::string,size_t> counts;
    std::map<int64_t,std::array<int,3>> selections;
    std::vector<lidar_motion::Odom> odometry;
    std::vector<sensor_msgs::msg::PointCloud2> clouds;
    std::set<std::string> directions;
    int64_t last_nonzero=0,first_zero_after_fault=0;
    while(reader.has_next()) {
      const auto message=reader.read_next();const auto &topic=message->topic_name;++counts[topic];
      if(topic=="/odom")odometry.push_back(workbench::decode_odom(decode<nav_msgs::msg::Odometry>(message)));
      if(topic=="/cmd_vel") {
        const auto m=decode<geometry_msgs::msg::TwistStamped>(message);const auto stamp=rclcpp::Time(m.header.stamp).nanoseconds();const auto &v=m.twist;
        if(v.linear.x>0)directions.insert("forward");
        if(v.linear.x<0)directions.insert("backward");
        if(v.linear.y>0)directions.insert("left");
        if(v.linear.y<0)directions.insert("right");
        if(v.angular.z>0)directions.insert("ccw");
        if(v.angular.z<0)directions.insert("cw");
        if(v.linear.x!=0||v.linear.y!=0||v.angular.z!=0)last_nonzero=stamp;
        else if(failure>0&&stamp>=failure&&first_zero_after_fault==0)first_zero_after_fault=stamp;
      }
      if(topic=="/lidar_front/scan_mapping"||topic=="/lidar_rear/scan_mapping") {
        const auto scan=decode<sensor_msgs::msg::LaserScan>(message);
        require(scan.ranges.size()==1600&&scan.time_increment==0,"Unexpected mock scan layout");
        require(std::count_if(scan.ranges.begin(),scan.ranges.end(),[](float x){return std::isfinite(x);})>=30,"Scan lacks usable returns");
      }
      if(topic=="/lidar/points_merged"||topic=="/lidar/points_filtered"||topic=="/lidar/points_rejected") {
        const auto cloud=decode<sensor_msgs::msg::PointCloud2>(message);const auto stamp=rclcpp::Time(cloud.header.stamp).nanoseconds();
        auto inserted=selections.emplace(stamp,std::array<int,3>{{-1,-1,-1}});auto &entry=inserted.first->second;
        const int column=topic=="/lidar/points_merged"?0:topic=="/lidar/points_filtered"?1:2;
        require(entry[column]<0,"Repeated fusion stamp");entry[column]=static_cast<int>(cloud.width);
        if(column==0)clouds.push_back(cloud);
      }
    }
    for(const auto *topic:{"/odom","/map","/tf","/tf_static","/cmd_vel","/joint_states","/lidar_front/scan_mapping",
      "/lidar_rear/scan_mapping","/lidar_front/raw_rays","/lidar_rear/raw_rays","/lidar/points_merged","/lidar/points_filtered","/lidar/points_rejected"})
      require(counts[topic]>0,std::string("No recording for ")+topic);
    size_t complete=0;
    for(const auto &[stamp,s]:selections) {
      (void)stamp;
      if(s[0]>=0&&s[1]>=0&&s[2]>=0){require(s[0]==s[1]+s[2],"Filtering lost/duplicated returns");++complete;}
    }
    require(complete>10&&complete*100>=selections.size()*95,"Too few complete recorded fusion groups");
    std::sort(odometry.begin(),odometry.end(),[](const auto &a,const auto &b){return a.stamp_ns<b.stamp_ns;});
    const lidar_motion::Trajectory trajectory(odometry,40000000);
    const std::array<std::array<double,4>,6> walls{{{-2,-2,4,-2},{4,-2,4,3},{4,3,-2,3},{-2,3,-2,-2},{1.5,.8,2.3,.8},{2.3,.8,2.3,1.6}}};
    double worst=0;size_t points=0;
    for(const auto &cloud:clouds) {
      const auto stamp=rclcpp::Time(cloud.header.stamp).nanoseconds();
      if(stamp<odometry.front().stamp_ns||stamp>odometry.back().stamp_ns)continue;
      require(cloud.header.frame_id=="base_footprint","Wrong merged frame");const auto pose=trajectory.at(stamp);
      sensor_msgs::PointCloud2ConstIterator<float> x(cloud,"x"),y(cloud,"y"),z(cloud,"z");
      sensor_msgs::PointCloud2ConstIterator<int32_t> side(cloud,"source_id");std::array<size_t,2> sources{{0,0}};
      for(size_t i=0;i<cloud.width;++i,++x,++y,++z,++side) {
        require(*side==0||*side==1,"Unknown source");++sources[*side];require(std::abs(*z-.0545)<1e-6,"Wrong sensor height");
        const auto p=lidar_motion::apply(pose,{*x,*y});double distance=100;
        for(const auto &w:walls) {
          const double dx=w[2]-w[0],dy=w[3]-w[1];const double t=std::clamp(((p.x-w[0])*dx+(p.y-w[1])*dy)/(dx*dx+dy*dy),0.,1.);
          distance=std::min(distance,std::hypot(p.x-w[0]-t*dx,p.y-w[1]-t*dy));
        }
        worst=std::max(worst,distance);++points;
      }
      require(sources[0]>0&&sources[1]>0,"A merged cloud lost one sensor");
    }
    require(points>1000&&worst<.002,"Synthetic merged endpoints miss known walls by more than 2 mm");
    if(failure>0) {
      require(directions.count("forward")>0&&last_nonzero>0,"Fault trial never moved");
      require(last_nonzero<failure+600000000&&first_zero_after_fault>=failure&&first_zero_after_fault<failure+600000000,"No bounded zero-command response to simulated sensor loss");
      std::cout<<"Fault to first recorded zero command: "<<(first_zero_after_fault-failure)*1e-6<<" ms\n";
    } else {
      require(directions.size()==6,"Not all six motion directions recorded");
      for(const auto *suffix:{".yaml",".pgm",".posegraph",".data"})require(std::filesystem::file_size(directory/"maps"/(std::string("mock_room")+suffix))>0,"Missing saved map/graph");
    }
    std::cout<<"PASS: "<<clouds.size()<<" merged frames, "<<complete<<" complete filter groups, "<<points
      <<" synthetic endpoints; maximum known-wall error "<<worst*1000<<" mm. Physical accuracy remains unverified.\n";
  }catch(const std::exception &e){std::cerr<<"FAIL: "<<e.what()<<'\n';return 1;}
}
