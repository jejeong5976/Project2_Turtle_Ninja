// Experimental live front-only input. No motor command, TF, or mapper ownership.
#include "coin_d4_driver/coin_d4_node_handler.hpp"
#include "coin_d4_driver/mapping_frame.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <deque>
#include <mutex>
#include <sstream>
#include <filesystem>
#include <sys/wait.h>
#include <unistd.h>
class MappingNode:public rclcpp::Node {
  std::unique_ptr<robotis::coin_d4::CoinD4NodeHandler> handler_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr work_,status_timer_;
  std::mutex mutex_;std::deque<lidar_mapping::Frame> pending_;
  std::deque<lidar_motion::Odom> odom_;
  std::string model_;int64_t delay_,last_reference_=-1,last_scan_id_=-1,last_publish_=0;
  size_t published_=0,rejected_=0;bool fault_=false;std::string reason_="waiting for input";
  void reject(const std::string &why,bool latch=false) {
    ++rejected_;reason_=why;fault_|=latch;
    RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,"Mapping scan withheld: %s%s",why.c_str(),latch?"; restart required":"");
  }
  void odometry(nav_msgs::msg::Odometry::ConstSharedPtr m) {
    if(fault_)return;
    const auto &q=m->pose.pose.orientation;const auto &p=m->pose.pose.position;
    const auto stamp=rclcpp::Time(m->header.stamp).nanoseconds(),current=now().nanoseconds();
    const double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
    if(m->header.frame_id!="odom"||m->child_frame_id!="base_footprint"||
       !std::isfinite(norm)||std::abs(norm-1)>1e-3||std::abs(q.x)>1e-3||std::abs(q.y)>1e-3||
       !std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z)||std::abs(p.z)>1e-3||
       stamp<0||stamp>current+5000000||(!odom_.empty()&&stamp<=odom_.back().stamp_ns)) {
      reject("invalid frames/pose or regressing/future odometry",true);return;
    }
    if(current-stamp>100000000){reject("stale odometry");return;}
    const double yaw=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
    odom_.push_back({stamp,{p.x,p.y,yaw}});
    while(odom_.size()>200||(!odom_.empty()&&stamp-odom_.front().stamp_ns>3000000000LL))odom_.pop_front();
  }
  void process() {
    if(get_parameter("use_sim_time").as_bool()){reject("simulation clock enabled",true);return;}
    if(fault_)return;
    lidar_mapping::Frame frame;
    {std::lock_guard<std::mutex> guard(mutex_);if(pending_.empty())return;frame=std::move(pending_.front());pending_.pop_front();}
    try {
      const auto timed=lidar_mapping::time_frame(frame,model_,delay_);
      const auto before=lidar_timing::mono_ns(),current=now().nanoseconds(),after=lidar_timing::mono_ns();
      if(lidar_timing::bracket_gap(frame.clock,{before,current,after})>5000000){reject("ROS/monotonic clock discontinuity",true);return;}
      if(timed.reference>current+5000000||current-timed.reference>250000000){reject("scan reference is future/stale");return;}
      if(odom_.size()<2||current-odom_.back().stamp_ns>100000000){reject("missing/fresh odometry unavailable");return;}
      if(timed.first<odom_.front().stamp_ns){reject("odometry lacks early beam coverage");return;}
      if(timed.reference>odom_.back().stamp_ns) {
        std::lock_guard<std::mutex> guard(mutex_);pending_.push_front(std::move(frame));
        while(pending_.size()>4)pending_.pop_back();
        return;
      }
      if(frame.id<=last_scan_id_||timed.reference<=last_reference_){reject("scan/reference regression",true);return;}
      auto lo=std::lower_bound(odom_.begin(),odom_.end(),timed.first,[](const auto &s,int64_t t){return s.stamp_ns<t;});
      if(lo!=odom_.begin())--lo;
      const auto hi=std::lower_bound(odom_.begin(),odom_.end(),timed.reference,[](const auto &s,int64_t t){return s.stamp_ns<t;});
      const auto scan=lidar_mapping::project_frame(timed,{lo,std::next(hi)});
      size_t valid=0;for(const auto &bin:scan.bins)valid+=bin.input_bin>=0;
      if(valid<30){reject("fewer than 30 accepted returns");return;}
      const auto ready=now().nanoseconds();
      if(ready-timed.reference>250000000||ready-odom_.back().stamp_ns>100000000){reject("data became stale during correction");return;}
      sensor_msgs::msg::LaserScan out;out.header.stamp=rclcpp::Time(timed.reference);out.header.frame_id="lidar_front_link";
      out.angle_min=0;out.angle_increment=2*std::acos(-1.)/scan.bins.size();out.angle_max=(scan.bins.size()-1)*out.angle_increment;
      out.time_increment=0;out.scan_time=(frame.end-frame.start)/1e9;out.range_min=.05;out.range_max=12;
      for(const auto &bin:scan.bins)out.ranges.push_back(float(bin.range));
      scan_pub_->publish(out);last_scan_id_=frame.id;last_reference_=timed.reference;last_publish_=current;++published_;reason_="publishing";
    }catch(const std::exception &e){reject(e.what());}
  }
public:
  explicit MappingNode(bool sensor_enabled=true,const rclcpp::NodeOptions &options=rclcpp::NodeOptions()):Node("front_mapping_input",options) {
    if(!declare_parameter<bool>("mapping_experiment",false)||get_parameter("use_sim_time").as_bool())
      throw std::runtime_error("Explicit mapping_experiment=true and system time required");
    model_=declare_parameter<std::string>("timing_model","");delay_=declare_parameter<int64_t>("assumed_delay_ns",-1);
    if((model_!="nominal"&&model_!="scaled")||delay_<0||delay_>100000000)
      throw std::runtime_error("Explicit nominal|scaled and hypothetical delay 0..100000000 ns required");
    if(declare_parameter<std::string>("port","")!="/dev/serial/by-path/platform-3610000.usb-usb-0:2.4.4:1.0-port0"||
       declare_parameter<int>("version",4)!=4||declare_parameter<bool>("reverse",false)||
       declare_parameter<int>("baudrate",230400)!=230400||declare_parameter<std::string>("frame_id","lidar_front_link")!="lidar_front_link")
      throw std::runtime_error("Require preserved front socket, version, orientation, baud and frame");
    declare_parameter<int>("warmup_time",5);
    odom_sub_=create_subscription<nav_msgs::msg::Odometry>("/odom",100,[this](nav_msgs::msg::Odometry::ConstSharedPtr m){odometry(m);});
    scan_pub_=create_publisher<sensor_msgs::msg::LaserScan>("/lidar_front/scan_mapping",rclcpp::SensorDataQoS());
    status_pub_=create_publisher<std_msgs::msg::String>("/lidar_front/mapping_status",1);
    work_=create_wall_timer(std::chrono::milliseconds(20),[this]{process();});
    status_timer_=create_wall_timer(std::chrono::seconds(1),[this]{
      std::ostringstream text;text<<"model="<<model_<<" assumed_delay_ns="<<delay_<<" acquisition_time_valid=0 bins=1600"
        <<" published="<<published_<<" rejected="<<rejected_<<" fault="<<fault_
        <<" fresh="<<(last_publish_>0&&now().nanoseconds()-last_publish_<250000000&&!fault_)
        <<" reference_ns="<<last_reference_<<" reason="<<reason_;
      std_msgs::msg::String status;status.data=text.str();status_pub_->publish(status);
    });
    if(sensor_enabled) {
      const auto device=std::filesystem::canonical(get_parameter("port").as_string()).string();
      const auto pid=fork();if(pid<0)throw std::runtime_error("Cannot inspect serial ownership");
      if(pid==0){execlp("fuser","fuser",device.c_str(),static_cast<char*>(nullptr));_exit(127);}
      int status=0;
      if(waitpid(pid,&status,0)!=pid||!WIFEXITED(status)||WEXITSTATUS(status)!=1)
        throw std::runtime_error("Front serial port is owned or fuser inspection failed; stop only the original front driver");
      handler_=std::make_unique<robotis::coin_d4::CoinD4NodeHandler>("",this);
      handler_->set_mapping_callback([this](const lidar_mapping::Frame &frame){
        std::lock_guard<std::mutex> guard(mutex_);if(pending_.size()>=4)pending_.pop_front();pending_.push_back(frame);
      });
      handler_->activate_grab_thread();handler_->activate_publish_thread();
    }
  }
  ~MappingNode() override {if(handler_){handler_->deactivate_publish_thread();handler_->deactivate_grab_thread();handler_.reset();}}
#ifdef LIDAR_MAPPING_NODE_TEST
  void inject(const lidar_mapping::Frame &frame){std::lock_guard<std::mutex> guard(mutex_);pending_.push_back(frame);}
  bool faulted() const{return fault_;}
#endif
};
#ifndef LIDAR_MAPPING_NODE_TEST
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  try{rclcpp::spin(std::make_shared<MappingNode>());}
  catch(const std::exception &e){std::cerr<<"MAPPING INPUT FAILED: "<<e.what()<<'\n';rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
#endif
