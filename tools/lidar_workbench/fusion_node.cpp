#include "ros_helpers.hpp"
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <array>
#include <chrono>

class WorkbenchFusion : public rclcpp::Node {
  using Cloud=sensor_msgs::msg::PointCloud2;
  workbench::Options options_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  std::array<Cloud::ConstSharedPtr,2> pending_;
  std::array<int64_t,2> previous_{{-1,-1}};
  std::array<rclcpp::Subscription<Cloud>::SharedPtr,2> subscriptions_;
  rclcpp::Publisher<Cloud>::SharedPtr merged_,filtered_,rejected_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::TimerBase::SharedPtr timer_;
  int64_t last_=0;
  std::string reason_="waiting for both sensors";
  bool fault_=false;

  void receive(int side, Cloud::ConstSharedPtr message) {
    try {
      const auto stamp=rclcpp::Time(message->header.stamp).nanoseconds();
      workbench::require(message->header.frame_id==(side==0?"lidar_front_link":"lidar_rear_link"),"Wrong cloud frame");
      workbench::require(stamp>previous_[side],"Ray timestamp regression");previous_[side]=stamp;
      workbench::decode_cloud(*message,side);
      pending_[side]=message;
    } catch(const std::exception &e) {fault_=true;reason_=e.what();}
  }
  void process() {
    if(fault_) {workbench::status(status_,"fault: "+reason_);return;}
    if(pending_[0]&&pending_[1]) {
      try {
        const std::array<int64_t,2> stamp{{rclcpp::Time(pending_[0]->header.stamp).nanoseconds(),rclcpp::Time(pending_[1]->header.stamp).nanoseconds()}};
        const auto reference=std::max(stamp[0],stamp[1]), current=now().nanoseconds();
        for(const auto s:stamp)workbench::require(s<=current+5000000 && current-s<options_.max_age*1e9,"Stale/future rays");
        if(std::abs(stamp[0]-stamp[1])>options_.max_skew*1e9) {
          pending_[stamp[0]<stamp[1]?0:1].reset();throw std::runtime_error("Sensor pair outside max_skew");
        }
        std::vector<workbench::Ray> combined;
        for(int i=0;i<2;++i) {
          const auto &frame=pending_[i]->header.frame_id;
          const auto fixed=buffer_.lookupTransform("base_footprint",frame,rclcpp::Time(stamp[i]));
          const auto mount=workbench::mount(i);const auto &p=fixed.transform.translation;
          workbench::require(std::abs(p.x-mount.x)<1e-6&&std::abs(p.y-mount.y)<1e-6&&std::abs(p.z-.0545)<1e-6&&
            std::abs(lidar_motion::wrap(workbench::planar_yaw(fixed.transform.rotation)-mount.yaw))<1e-6,"Unexpected LiDAR mounting transform");
          // Each cloud was deskewed to its own reference time. Move both endpoints AND origins
          // through odom to the newer time; a static TF alone would misalign moving pairs.
          const auto tf=buffer_.lookupTransform("base_footprint",rclcpp::Time(reference),frame,rclcpp::Time(stamp[i]),"odom");
          workbench::Pose pose{tf.transform.translation.x,tf.transform.translation.y,workbench::planar_yaw(tf.transform.rotation)};
          for(const auto &ray:workbench::decode_cloud(*pending_[i],i))
            combined.push_back(workbench::transform(ray,pose,(stamp[i]-reference)*1e-9));
        }
        workbench::require(now().nanoseconds()-reference<options_.max_age*1e9,"Fusion processing exceeded freshness bound");
        const auto selection=workbench::spatial_filter(combined,options_);
        merged_->publish(workbench::cloud(combined,reference,"base_footprint",.0545));
        filtered_->publish(workbench::cloud(selection.kept,reference,"base_footprint",.0545));
        rejected_->publish(workbench::cloud(selection.rejected,reference,"base_footprint",.0545));
        pending_[0].reset();pending_[1].reset();last_=now().nanoseconds();reason_="ready";
      } catch(const std::exception &e) {reason_=e.what();}
    }
    workbench::status(status_,last_>0 && now().nanoseconds()-last_<options_.max_age*1e9 ? reason_ :
      "waiting: "+(reason_=="ready"?std::string("no fresh paired observations"):reason_));
  }
public:
  WorkbenchFusion():Node("workbench_fusion"),options_(workbench::scan_options(*this)),buffer_(get_clock()),listener_(buffer_) {
    workbench::require(!get_parameter("use_sim_time").as_bool(),"System time required");
    for(int i=0;i<2;++i)subscriptions_[i]=create_subscription<Cloud>(i==0?"/lidar_front/rays":"/lidar_rear/rays",
      rclcpp::SensorDataQoS(),[this,i](Cloud::ConstSharedPtr m){receive(i,m);});
    merged_=create_publisher<Cloud>("/lidar/points_merged",rclcpp::SensorDataQoS());
    filtered_=create_publisher<Cloud>("/lidar/points_filtered",rclcpp::SensorDataQoS());
    rejected_=create_publisher<Cloud>("/lidar/points_rejected",rclcpp::SensorDataQoS());
    status_=create_publisher<std_msgs::msg::String>("/lidar/status",10);
    timer_=create_wall_timer(std::chrono::milliseconds(20),[this]{process();});
  }
};
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  try {rclcpp::spin(std::make_shared<WorkbenchFusion>());}
  catch(const std::exception &e){std::cerr<<e.what()<<'\n';rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
