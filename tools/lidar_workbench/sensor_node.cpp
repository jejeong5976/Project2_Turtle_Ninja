#include "ros_helpers.hpp"
#include "coin_d4_driver/coin_d4_node_handler.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <deque>
#include <mutex>
#include <sys/wait.h>
#include <sys/stat.h>
#include <unistd.h>

class WorkbenchSensor : public rclcpp::Node {
  workbench::Options options_;
  int side_;
  bool mock_, latched_=false;
  std::string frame_, reason_="waiting for wheel odometry";
  std::unique_ptr<robotis::coin_d4::CoinD4NodeHandler> handler_;
  std::deque<lidar_motion::Odom> odom_;
  std::deque<lidar_mapping::Frame> pending_;
  std::mutex mutex_;
  int64_t id_=0, last_id_=-1, last_reference_=-1, last_publish_=0;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr rays_pub_,raw_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr process_timer_, status_timer_, mock_timer_;

  void odometry(const nav_msgs::msg::Odometry &m) {
    try {
      const auto s=workbench::decode_odom(m); const auto current=now().nanoseconds();
      workbench::require(s.stamp_ns>=0 && s.stamp_ns<=current+5000000 &&
        (odom_.empty()||s.stamp_ns>odom_.back().stamp_ns),"Odometry clock/frame regression");
      workbench::require(current-s.stamp_ns<100000000,"Stale odometry");
      odom_.push_back(s); while(odom_.size()>200) odom_.pop_front();
    } catch(const std::exception &e) {latched_=true;reason_=e.what();}
  }
  void simulate() {
    if(odom_.size()<10||latched_) return;
    try {
      lidar_mapping::Frame f; f.id=++id_; f.count=400; f.source_valid=true;
      f.clock.before_ns=lidar_timing::mono_ns(); f.clock.ros_ns=now().nanoseconds(); f.clock.after_ns=lidar_timing::mono_ns();
      f.end=f.clock.before_ns-30000000; f.start=f.end-100000000;
      for(int i=0;i<400;++i) f.beams.push_back({i,i,1.,-2*std::acos(-1.)*i/400.});
      const auto timed=lidar_mapping::time_frame(f,"nominal",0);
      const lidar_motion::Trajectory trajectory({odom_.begin(),odom_.end()},40000000);
      for(size_t i=0;i<f.beams.size();++i) {
        const auto base=trajectory.at(timed.beams[i].stamp), sensor=workbench::mount(side_);
        const auto origin=lidar_motion::apply(base,{sensor.x,sensor.y});
        f.beams[i].range=workbench::scene_range(origin,base.yaw+sensor.yaw+f.beams[i].angle);
      }
      std::lock_guard<std::mutex> lock(mutex_); pending_.push_back(f);
    } catch(const std::exception &e) {reason_=e.what();}
  }
  void process() {
    if(latched_) return;
    lidar_mapping::Frame f;
    {std::lock_guard<std::mutex> lock(mutex_); if(pending_.empty()) return; f=std::move(pending_.front());pending_.pop_front();}
    try {
      const auto timed=lidar_mapping::time_frame(f,"nominal",0);
      const auto before=lidar_timing::mono_ns(), current=now().nanoseconds(), after=lidar_timing::mono_ns();
      if(lidar_timing::bracket_gap(f.clock,{before,current,after})>5000000) {latched_=true;throw std::runtime_error("ROS/monotonic clock jump");}
      workbench::require(current-timed.reference<options_.max_age*1e9 && timed.reference<=current+5000000,"Stale/future scan");
      workbench::require(odom_.size()>=2 && current-odom_.back().stamp_ns<100000000,"Missing fresh odometry");
      if(timed.reference>odom_.back().stamp_ns) {
        std::lock_guard<std::mutex> lock(mutex_);pending_.push_front(f);while(pending_.size()>4)pending_.pop_back();return;
      }
      workbench::require(timed.first>=odom_.front().stamp_ns,"Missing early-beam odometry");
      if(f.id<=last_id_||timed.reference<=last_reference_){latched_=true;throw std::runtime_error("Regressing scan ID/time");}
      auto lo=std::lower_bound(odom_.begin(),odom_.end(),timed.first,[](const auto &s,int64_t t){return s.stamp_ns<t;});
      if(lo!=odom_.begin()) --lo;
      auto hi=std::lower_bound(odom_.begin(),odom_.end(),timed.reference,[](const auto &s,int64_t t){return s.stamp_ns<t;});
      const auto prepared=workbench::prepare(timed,{lo,std::next(hi)},side_,options_);
      raw_pub_->publish(workbench::cloud(prepared.raw,timed.reference,frame_));
      size_t finite=0; for(const auto &b:prepared.scan.bins) finite+=b.input_bin>=0;
      workbench::require(finite>=30,"Fewer than 30 usable projected returns");
      workbench::require(now().nanoseconds()-timed.reference<options_.max_age*1e9,"Scan processing exceeded freshness bound");
      sensor_msgs::msg::LaserScan scan;scan.header.stamp=rclcpp::Time(timed.reference);scan.header.frame_id=frame_;
      scan.angle_min=0;scan.angle_increment=2*std::acos(-1.)/options_.bins;
      scan.angle_max=(options_.bins-1)*scan.angle_increment;scan.time_increment=0;
      scan.scan_time=(f.end-f.start)*1e-9;scan.range_min=options_.min_range;scan.range_max=options_.max_range;
      for(const auto &b:prepared.scan.bins)scan.ranges.push_back(b.range);
      scan_pub_->publish(scan);rays_pub_->publish(workbench::cloud(prepared.corrected,timed.reference,frame_));
      last_id_=f.id;last_reference_=timed.reference;last_publish_=now().nanoseconds();reason_="ready";
    } catch(const std::exception &e) {reason_=e.what();}
  }
public:
  WorkbenchSensor():Node("workbench_sensor"),options_(workbench::scan_options(*this)),
      side_(declare_parameter<int>("side",0)),mock_(declare_parameter<bool>("mock",true)) {
    workbench::mount(side_); frame_=side_==0?"lidar_front_link":"lidar_rear_link";
    workbench::require(!get_parameter("use_sim_time").as_bool(),"System time required");
    declare_parameter<std::string>("frame_id",frame_);
    odom_sub_=create_subscription<nav_msgs::msg::Odometry>("/odom",100,[this](nav_msgs::msg::Odometry::ConstSharedPtr m){odometry(*m);});
    scan_pub_=create_publisher<sensor_msgs::msg::LaserScan>("scan_mapping",rclcpp::SensorDataQoS());
    raw_pub_=create_publisher<sensor_msgs::msg::PointCloud2>("raw_rays",rclcpp::SensorDataQoS());
    rays_pub_=create_publisher<sensor_msgs::msg::PointCloud2>("rays",rclcpp::SensorDataQoS());
    status_pub_=create_publisher<std_msgs::msg::String>("status",10);
    process_timer_=create_wall_timer(std::chrono::milliseconds(10),[this]{process();});
    status_timer_=create_wall_timer(std::chrono::milliseconds(100),[this]{
      const bool fresh=last_publish_>0 && now().nanoseconds()-last_publish_<options_.max_age*1e9;
      workbench::status(status_pub_,latched_?"fault: "+reason_:(!fresh?"waiting: "+(reason_=="ready"?std::string("no fresh sensor output"):reason_):reason_));
    });
    if(mock_) mock_timer_=create_wall_timer(std::chrono::milliseconds(100),[this]{simulate();});
    else {
      const auto port=declare_parameter<std::string>("port","");
      workbench::require(!port.empty()&&std::filesystem::path(port).is_absolute(),"Explicit absolute LiDAR device path required");
      struct stat info{};workbench::require(::stat(port.c_str(),&info)==0 && S_ISCHR(info.st_mode),"LiDAR path is not a character device");
      const auto device=std::filesystem::canonical(port).string();
      const pid_t pid=fork();workbench::require(pid>=0,"Cannot inspect serial ownership");
      if(pid==0){execlp("fuser","fuser",device.c_str(),static_cast<char*>(nullptr));_exit(127);}
      int code=0;workbench::require(waitpid(pid,&code,0)==pid && WIFEXITED(code)&&WEXITSTATUS(code)==1,"LiDAR device owned or ownership check failed");
      declare_parameter<bool>("mapping_experiment",true);
      declare_parameter<int>("version",4);declare_parameter<int>("baudrate",230400);declare_parameter<bool>("reverse",false);
      handler_=std::make_unique<robotis::coin_d4::CoinD4NodeHandler>("",this);
      handler_->set_mapping_callback([this](const lidar_mapping::Frame &f){
        std::lock_guard<std::mutex> lock(mutex_);if(pending_.size()>=4)pending_.pop_front();pending_.push_back(f);
      });
      handler_->activate_grab_thread();handler_->activate_publish_thread();
    }
    RCLCPP_INFO(get_logger(),"%s side=%d; nominal timing, assumed_delay=0, acquisition_time_valid=0",mock_?"SYNTHETIC":"REAL",side_);
  }
  ~WorkbenchSensor() override {if(handler_){handler_->deactivate_publish_thread();handler_->deactivate_grab_thread();handler_.reset();}}
};
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  try {rclcpp::spin(std::make_shared<WorkbenchSensor>());}
  catch(const std::exception &e){std::cerr<<e.what()<<'\n';rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
