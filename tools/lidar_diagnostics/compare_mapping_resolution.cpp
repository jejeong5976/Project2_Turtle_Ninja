// Offline grid comparison: identical corrected scans and frozen odometry poses.
// No ROS node, publisher, hardware or new pose optimization. Not a live-graph restore.
#include "motion_model.hpp"
#include "mapping_trial_core.hpp"
#include <karto_sdk/Karto.h>
#include <rosbag2_cpp/reader.hpp>
#include <rclcpp/serialization.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <chrono>
namespace fs=std::filesystem;
using mapping_trial::require;
using Scan=sensor_msgs::msg::LaserScan;
int64_t stamp(const builtin_interfaces::msg::Time &t){return int64_t(t.sec)*1000000000+t.nanosec;}
double yaw(const geometry_msgs::msg::Quaternion &q){
  require(std::abs(q.x)<.001&&std::abs(q.y)<.001&&std::abs(q.z*q.z+q.w*q.w-1)<.001,"Invalid planar quaternion");
  return std::atan2(2*q.w*q.z,1-2*q.z*q.z);
}
template<class M>M decode(const std::shared_ptr<rosbag2_storage::SerializedBagMessage> &record){
  rclcpp::SerializedMessage data(*record->serialized_data);M m;rclcpp::Serialization<M>().deserialize_message(&data,&m);return m;
}
int main(int argc,char **argv){try{
  require(argc==3,"Usage: compare_mapping_resolution BAG NEW_OUTPUT");
  const fs::path out=argv[2];require(fs::create_directory(out),"Output must be new");
  rosbag2_cpp::Reader reader;reader.open(argv[1]);std::vector<Scan> scans;std::vector<lidar_motion::Odom> odom;
  lidar_motion::Pose final_tf{};bool tf=false;size_t records=0;
  while(reader.has_next()){
    require(++records<=500000,"Bag record limit");auto r=reader.read_next();
    if(r->topic_name=="/lidar_front/scan_mapping")scans.push_back(decode<Scan>(r));
    if(r->topic_name=="/odom"){
      const auto m=decode<nav_msgs::msg::Odometry>(r);require(m.header.frame_id=="odom"&&m.child_frame_id=="base_footprint","Odom frames");
      odom.push_back({stamp(m.header.stamp),{m.pose.pose.position.x,m.pose.pose.position.y,yaw(m.pose.pose.orientation)}});
    }
    if(r->topic_name=="/tf")for(const auto &t:decode<tf2_msgs::msg::TFMessage>(r).transforms)
      if(t.header.frame_id=="map"&&t.child_frame_id=="odom"){
        final_tf={t.transform.translation.x,t.transform.translation.y,yaw(t.transform.rotation)};tf=true;
      }
  }
  require(tf&&!scans.empty()&&scans.size()<=20000,"Missing/excess data");
  const lidar_motion::Trajectory trajectory(odom,40000000);
  auto laser=std::unique_ptr<karto::LaserRangeFinder>(karto::LaserRangeFinder::CreateLaserRangeFinder(karto::LaserRangeFinder_Custom,karto::Name("offline_resolution_front")));
  const auto &first=scans.front();laser->SetMinimumRange(.05);laser->SetMaximumRange(12);laser->SetRangeThreshold(12);
  laser->SetMinimumAngle(first.angle_min);laser->SetMaximumAngle(first.angle_max);laser->SetAngularResolution(first.angle_increment);laser->SetIs360Laser(false);
  laser->SetOffsetPose({.110,.005,std::acos(-1.)});karto::SensorManager::GetInstance()->RegisterSensor(laser.get());
  std::vector<std::unique_ptr<karto::LocalizedRangeScan>> owned;karto::LocalizedRangeScanVector raw;size_t skipped=0;int64_t previous=-1;
  std::ofstream poses(out/"frozen_poses.csv");poses<<std::setprecision(17)<<"stamp_ns,x_m,y_m,yaw_rad\n";
  for(const auto &scan:scans){
    const auto time=stamp(scan.header.stamp);require(time>previous,"Regressing scan");previous=time;
    require(scan.header.frame_id=="lidar_front_link"&&scan.ranges.size()==1600&&scan.time_increment==0&&
      scan.angle_min==first.angle_min&&scan.angle_max==first.angle_max&&scan.angle_increment==first.angle_increment,"Mixed scan contract");
    if(time<odom.front().stamp_ns||time>odom.back().stamp_ns){++skipped;continue;}
    auto p=trajectory.at(time);const auto xy=lidar_motion::apply(final_tf,{p.x,p.y});
    const karto::Pose2 pose(xy.x,xy.y,lidar_motion::wrap(p.yaw+final_tf.yaw));
    karto::RangeReadingsVector ranges;for(float v:scan.ranges){require(!std::isinf(v),"Infinite ray");ranges.push_back(v);}
    auto localized=std::make_unique<karto::LocalizedRangeScan>(laser->GetName(),ranges);
    localized->SetOdometricPose(pose);localized->SetCorrectedPose(pose);require(laser->Validate(localized.get()),"Invalid Karto scan");
    poses<<time<<','<<pose.GetX()<<','<<pose.GetY()<<','<<pose.GetHeading()<<'\n';raw.push_back(localized.get());owned.push_back(std::move(localized));
  }
  require(raw.size()>=10,"Too few bracketed scans");
  std::ofstream report(out/"comparison.txt");report<<std::setprecision(17)
    <<"Frozen odometry trajectory with one final recorded map->odom transform.\n"
    <<"Same scans/poses/thresholds in both grids; no SLAM optimization or live pose-graph reconstruction.\n"
    <<"scans="<<raw.size()<<" skipped_no_odom_bracket="<<skipped<<" min_pass_through=2 occupancy_threshold=0.1\n";
  for(double resolution:{.05,.025}){
    const auto began=std::chrono::steady_clock::now();
    std::unique_ptr<karto::OccupancyGrid> grid(karto::OccupancyGrid::CreateFromScans(raw,resolution,2,.1));
    require(bool(grid),"Grid creation failed");const auto ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-began).count();
    const std::string name=resolution==.05?"grid_5cm":"grid_2p5cm";
    const auto origin=grid->GetCoordinateConverter()->GetOffset();size_t occupied=0,free=0,unknown=0;
    std::ofstream image(out/(name+".pgm"),std::ios::binary);image<<"P5\n"<<grid->GetWidth()<<' '<<grid->GetHeight()<<"\n255\n";
    for(int y=grid->GetHeight()-1;y>=0;--y)for(int x=0;x<grid->GetWidth();++x){
      const auto v=grid->GetValue({x,y});char pixel=char(205);
      if(v==karto::GridStates_Occupied){++occupied;pixel=0;}else if(v==karto::GridStates_Free){++free;pixel=char(254);}else ++unknown;
      image.put(pixel);
    }
    image.close();require(bool(image),"Grid image write failed");std::ofstream yaml(out/(name+".yaml"));
    yaml<<std::setprecision(17)<<"image: "<<name<<".pgm\nmode: trinary\nresolution: "<<resolution
      <<"\norigin: ["<<origin.GetX()<<", "<<origin.GetY()<<", 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n";
    yaml.close();require(bool(yaml),"YAML write failed");
    report<<name<<" width="<<grid->GetWidth()<<" height="<<grid->GetHeight()<<" cells="<<size_t(grid->GetWidth())*grid->GetHeight()
      <<" occupied="<<occupied<<" free="<<free<<" unknown="<<unknown<<" raster_ms="<<ms<<'\n';
  }
  poses.close();report.close();require(bool(poses)&&bool(report),"Report write failed");
  owned.clear();karto::SensorManager::GetInstance()->UnregisterSensor(laser.get());
  std::cout<<"Comparison written: "<<out<<" (frozen poses, not live optimized graph).\n";return 0;
}catch(const std::exception &e){std::cerr<<e.what()<<'\n';return 1;}}
