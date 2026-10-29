// User-run bounded reverse/return route (separate from the accepted v2 pulses). Records existing ROS streams; never opens serial
// ports, starts/stops drivers, publishes TF, changes configuration or retries motion.
#include "mapping_trial_core.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <algorithm>
#include <chrono>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <thread>
#include <unistd.h>
namespace fs=std::filesystem;
using mapping_trial::require;
using Steady=std::chrono::steady_clock;
inline double steady_seconds(){return std::chrono::duration<double>(Steady::now().time_since_epoch()).count();}
volatile sig_atomic_t trial_interrupted=0;
void trial_signal(int){trial_interrupted=1;}
class MappingRoute:public rclcpp::Node {
  using Odom=nav_msgs::msg::Odometry;using Scan=sensor_msgs::msg::LaserScan;
  using Map=nav_msgs::msg::OccupancyGrid;using TF=tf2_msgs::msg::TFMessage;
  using Text=std_msgs::msg::String;using Cmd=geometry_msgs::msg::TwistStamped;
  using State=lifecycle_msgs::srv::GetState;
  enum class Phase{waiting,baseline,pulse,zeros,stopping,post,done};
  Phase phase_=Phase::waiting;std::string prefix_;fs::path dir_;
  size_t leg_=0;
  struct Leg {size_t messages=0, zeros=0, scans=0; int64_t first=0, stop=0; double end_forward=0;};
  Leg legs_[2];
  mapping_trial::Pose hold_;
  rosbag2_cpp::Writer bag_;bool bag_open_=false,armed_=false,attempted_=false,closed_=false;
  std::ofstream odom_file_,events_;std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;
  rclcpp::Publisher<Cmd>::SharedPtr command_;
  rclcpp::Client<State>::SharedPtr state_client_;
  rclcpp::Client<State>::SharedFuture state_future_;bool state_pending_=false;
  double started_=steady_seconds(),phase_at_=started_,last_command_=0,state_request_=0,state_seen_=0;
  double odom_seen_=0,scan_seen_=0,status_seen_=0,map_seen_=0,map_tf_seen_=0;
  int64_t clock_anchor_=0,last_scan_=-1;
  size_t bytes_=0,records_=0,scans_=0,maps_=0,motion_messages_=0,zero_messages_=0;
  bool state_active_=false,mount_=false,odom_tf_=false,map_changed_=false;
  mapping_trial::Status status_{0,0,0};std::deque<mapping_trial::Pose> odom_;
  mapping_trial::Pose origin_;int64_t starting_rejections_=0;uint64_t map_hash_=0,baseline_map_hash_=0;
  std::string topic(const std::string &name)const{return prefix_+name;}
  void event(const std::string &name){events_<<now().nanoseconds()<<','<<name<<'\n';events_.flush();require(bool(events_),"Event log failed");}
  template<class M> void record(const M &m,const std::string &name){
    auto serialized=std::make_shared<rclcpp::SerializedMessage>();rclcpp::Serialization<M> serializer;
    serializer.serialize_message(&m,serialized.get());bytes_+=serialized->size();
    require(++records_<=25000&&bytes_<=128*1024*1024,"Recording size/count limit exceeded");
    bag_.write(serialized,topic(name),rosidl_generator_traits::name<M>(),now());
  }
  template<class M,class F> void subscribe(const std::string &name,const rclcpp::QoS &qos,F callback){
    rosbag2_storage::TopicMetadata meta;meta.name=topic(name);meta.type=rosidl_generator_traits::name<M>();
    meta.serialization_format="cdr";meta.offered_qos_profiles={qos};bag_.create_topic(meta);
    subs_.push_back(create_subscription<M>(topic(name),qos,[this,name,callback](typename M::ConstSharedPtr msg){
      record(*msg,name);callback(*msg);
    }));
  }
  static double yaw(const geometry_msgs::msg::Quaternion &q){
    const double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
    require(std::isfinite(norm)&&std::abs(norm-1)<1e-3&&std::abs(q.x)<1e-3&&std::abs(q.y)<1e-3,"Invalid planar quaternion");
    return std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
  }
  void odometry(const Odom &m){
    const auto &p=m.pose.pose.position;const auto stamp=rclcpp::Time(m.header.stamp).nanoseconds();
    require(m.header.frame_id=="odom"&&m.child_frame_id=="base_footprint"&&std::isfinite(p.x)&&
      std::isfinite(p.y)&&std::isfinite(p.z)&&std::abs(p.z)<1e-3,"Invalid odometry frames/position");
    const mapping_trial::Pose next{stamp,p.x,p.y,yaw(m.pose.pose.orientation)};
    if(!odom_.empty()){
      require(stamp>odom_.back().stamp,"Odometry time regressed");
      if(armed_)mapping_trial::advancing(next,odom_.back());
    }
    odom_.push_back(next);while(odom_.size()>150)odom_.pop_front();odom_seen_=steady_seconds();
    if(armed_)route_bounds(next);
    odom_file_<<stamp<<','<<std::setprecision(17)<<p.x<<','<<p.y<<','<<next.yaw<<'\n';odom_file_.flush();
    require(bool(odom_file_),"Odometry CSV write failed");
  }
  void scan(const Scan &m){
    const auto stamp=rclcpp::Time(m.header.stamp).nanoseconds();
    require(m.header.frame_id=="lidar_front_link"&&m.ranges.size()==1600&&m.time_increment==0&&
      std::abs(m.angle_min)<1e-8&&std::abs(m.angle_increment-2*std::acos(-1.)/1600)<1e-8&&
      std::abs(m.range_min-.05)<1e-6&&m.range_max==12&&stamp>last_scan_,"Invalid/regressing mapping scan");
    size_t finite=0;for(float r:m.ranges){require(!std::isinf(r),"Mapping scan contains infinity");
      if(std::isfinite(r)){require(r>=m.range_min&&r<=m.range_max,"Invalid scan range");++finite;}}
    require(finite>=30,"Too few mapping returns");last_scan_=stamp;scan_seen_=steady_seconds();++scans_;
    for(auto &leg:legs_) if(leg.first>0&&stamp>=leg.first&&(leg.stop==0||stamp<=leg.stop))++leg.scans;
  }
  void mapping_status(const Text &m){
    const auto next=mapping_trial::status(m.data);
    require(next.published>=status_.published&&next.rejected>=status_.rejected&&next.reference>=status_.reference,"Mapping counters regressed");
    if(armed_)require(next.rejected==starting_rejections_,"Mapping input rejected a frame during the trial");
    status_=next;status_seen_=steady_seconds();
  }
  void map(const Map &m){
    require(m.header.frame_id=="map"&&std::abs(m.info.resolution-.05)<1e-6&&m.info.width>0&&m.info.height>0&&
      uint64_t(m.info.width)*m.info.height==m.data.size()&&m.data.size()<=4000000,"Invalid occupancy map");
    uint64_t hash=1469598103934665603ULL;for(int8_t v:m.data){require(v>=-1&&v<=100,"Invalid occupancy value");hash=(hash^uint8_t(v))*1099511628211ULL;}
    map_hash_=hash;map_seen_=steady_seconds();++maps_;
    if(attempted_&&hash!=baseline_map_hash_)map_changed_=true;
  }
  void transforms(const TF &m,bool fixed){
    for(const auto &t:m.transforms){
      if(t.child_frame_id=="lidar_front_link"){
        const auto &p=t.transform.translation;
        require(fixed&&t.header.frame_id=="base_footprint"&&std::abs(p.x-.110)<1e-6&&std::abs(p.y-.005)<1e-6&&
          std::abs(p.z-.0545)<1e-6&&std::abs(std::abs(yaw(t.transform.rotation))-std::acos(-1.))<1e-6,"Unexpected front TF owner/geometry");mount_=true;
      }
      if(t.child_frame_id=="base_footprint"){
        require(!fixed&&t.header.frame_id=="odom","Unexpected base_footprint TF parent");odom_tf_=true;
      }
      if(t.child_frame_id=="odom"){
        require(!fixed&&t.header.frame_id=="map","Unexpected odom TF parent");
        const auto &p=t.transform.translation;require(std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z),"Invalid map TF");
        yaw(t.transform.rotation);map_tf_seen_=steady_seconds();
      }
    }
  }
  bool owner(const std::string &name,const std::string &node,bool publisher=true){
    const auto endpoints=publisher?get_publishers_info_by_topic(topic(name)):get_subscriptions_info_by_topic(topic(name));
    return endpoints.size()==1&&endpoints[0].node_name()==node&&endpoints[0].node_namespace()==(prefix_.empty()?"/":prefix_);
  }
  bool healthy(double t,std::string &why){
    auto check=[&](bool v,const char *s){if(!v)why=s;return v;};const auto stamp=now().nanoseconds();
    return check(odom_.size()>=25&&t-odom_seen_<.10&&stamp-odom_.back().stamp>=-5000000&&stamp-odom_.back().stamp<100000000,"fresh odometry")&&
      check(scans_>=10&&t-scan_seen_<.25&&stamp-last_scan_>=-5000000&&stamp-last_scan_<250000000,"fresh mapping scans")&&
      check(status_seen_>0&&t-status_seen_<1.5,"fresh healthy mapping status")&&
      check(state_active_&&t-state_seen_<2.,"active mapper lifecycle")&&
      check(map_seen_>0&&t-map_seen_<6.&&map_tf_seen_>0&&t-map_tf_seen_<1.&&mount_&&odom_tf_,"map and TF reception")&&
      check(owner("/cmd_vel","mapping_route_check")&&owner("/cmd_vel","swerve_controller",false)&&command_->get_subscription_count()==1,"sole matched command ownership")&&
      check(owner("/odom","swerve_controller")&&owner("/lidar_front/scan_mapping","front_mapping_input")&&
        owner("/lidar_front/mapping_status","front_mapping_input")&&owner("/map","slam_toolbox"),"source topic ownership");
  }
  void poll_state(double t){
    if(state_pending_&&state_future_.wait_for(std::chrono::seconds(0))==std::future_status::ready){
      state_active_=state_future_.get()->current_state.id==3;state_seen_=t;state_pending_=false;
      if(armed_)require(state_active_,"Mapper left active state");
    }
    if(!state_pending_&&t-state_request_>=1.&&state_client_->service_is_ready()){
      state_future_=state_client_->async_send_request(std::make_shared<State::Request>()).share();state_pending_=true;state_request_=t;
    }
    if(armed_&&state_pending_)require(t-state_request_<1.5,"Mapper lifecycle query stalled");
  }
  void publish(bool moving,bool emergency=false){
    Cmd m;m.header.stamp=now();m.header.frame_id="base_footprint";
    if(moving)m.twist.linear.x=leg_==0?-.025:.025;
    // Publication first: a disk error must never prevent an attempted stop.
    command_->publish(m);last_command_=steady_seconds();
    auto &leg=legs_[leg_];
    if(moving){attempted_=true;++motion_messages_;++leg.messages;if(leg.first==0)leg.first=rclcpp::Time(m.header.stamp).nanoseconds();}
    else {++zero_messages_;++leg.zeros;if(leg.stop==0)leg.stop=rclcpp::Time(m.header.stamp).nanoseconds();}
    try{record(m,"/cmd_vel");}catch(...){if(!emergency)throw;}
  }
  void transition(Phase phase,const std::string &name,double t){phase_=phase;phase_at_=t;event(name);}
public:
  explicit MappingRoute(const fs::path &dir,std::string prefix=""):
    Node("mapping_route_check",prefix.empty()?"/":prefix),prefix_(std::move(prefix)),dir_(dir){
    require(!get_parameter("use_sim_time").as_bool(),"System time required");
    require(fs::is_directory(dir_)&&fs::is_empty(dir_)&&fs::space(dir_).available>200*1024*1024,"Require empty output directory and 200 MiB free");
    rosbag2_storage::StorageOptions opts;opts.uri=(dir_/"bag").string();opts.storage_id="sqlite3";bag_.open(opts);bag_open_=true;
    odom_file_.open(dir_/"odom.csv");odom_file_<<"stamp_ns,x_m,y_m,yaw_rad\n";events_.open(dir_/"events.csv");events_<<"stamp_ns,event\n";
    require(bool(odom_file_)&&bool(events_),"Cannot create CSV logs");clock_anchor_=now().nanoseconds()-int64_t(steady_seconds()*1e9);
    command_=create_publisher<Cmd>(topic("/cmd_vel"),10);
    rosbag2_storage::TopicMetadata cmd;cmd.name=topic("/cmd_vel");cmd.type="geometry_msgs/msg/TwistStamped";cmd.serialization_format="cdr";
    cmd.offered_qos_profiles={rclcpp::QoS(10)};bag_.create_topic(cmd);
    subscribe<Odom>("/odom",rclcpp::QoS(100),[this](const Odom &m){odometry(m);});
    subscribe<Scan>("/lidar_front/scan_mapping",rclcpp::SensorDataQoS().keep_last(20),[this](const Scan &m){scan(m);});
    subscribe<Text>("/lidar_front/mapping_status",rclcpp::QoS(10),[this](const Text &m){mapping_status(m);});
    subscribe<Map>("/map",rclcpp::QoS(1).transient_local(),[this](const Map &m){map(m);});
    subscribe<TF>("/tf",rclcpp::QoS(100).best_effort(),[this](const TF &m){transforms(m,false);});
    subscribe<TF>("/tf_static",rclcpp::QoS(100).transient_local(),[this](const TF &m){transforms(m,true);});
    state_client_=create_client<State>(topic("/slam_toolbox/get_state"));event("waiting_for_preflight");
  }
  bool done()const{return phase_==Phase::done;}
  bool attempted()const{return attempted_;}
  void route_bounds(const mapping_trial::Pose &p) const {
    const auto d=mapping_trial::relative(p,origin_);
    require(d.forward>-.150&&d.forward<.020&&std::abs(d.lateral)<.020&&std::abs(d.yaw)<.050,
      "Route envelope: -150 to +20 mm forward, 20 mm lateral, 0.050 rad yaw");
  }
  void hold_position() const {
    const auto d=mapping_trial::relative(odom_.back(),hold_);
    require(d.distance<.0005&&std::abs(d.yaw)<.005&&mapping_trial::stationary(odom_),
      "Motion during stationary capture");
  }
  void tick(){
    const double t=steady_seconds();require(t-started_<45.,"Overall 45-second deadline exceeded");
    require(std::abs(now().nanoseconds()-int64_t(t*1e9)-clock_anchor_)<5000000,"ROS/monotonic clock discontinuity");
    poll_state(t);std::string why;const bool good=healthy(t,why);
    if(phase_==Phase::waiting){
      require(t-started_<10.,"Preflight timeout waiting for "+why);
      if(good&&mapping_trial::stationary(odom_)){
        origin_=hold_=odom_.back();armed_=true;starting_rejections_=status_.rejected;
        transition(Phase::baseline,"stationary_baseline",t);
        std::cout<<"Recording stationary baseline; no motion yet.\n"<<std::flush;
      }return;
    }
    require(good,"Lost "+why);route_bounds(odom_.back());
    if(phase_==Phase::baseline){
      hold_position();
      if(t-phase_at_>=2.){
        baseline_map_hash_=map_hash_;transition(Phase::pulse,"reverse_start",t);
        std::cout<<"Reverse leg: -0.025 m/s, at most 40 commands, then stop.\n"<<std::flush;
        publish(true);
      }return;
    }
    auto &leg=legs_[leg_];
    if(phase_==Phase::pulse||phase_==Phase::zeros){
      require(t-last_command_<.15,"Command schedule stalled; no catch-up/extension");
      if(phase_==Phase::pulse){
        require(t-phase_at_<4.3,"Leg deadline");
        const auto d=mapping_trial::relative(odom_.back(),origin_);
        // Stop early at the spatial target; the finite command count remains a hard cap.
        if((leg_==0&&d.forward<=-.100)||(leg_==1&&d.forward>=-.005)){
          transition(Phase::zeros,"target_zero_sequence",t);publish(false);return;
        }
      }
      if(t-last_command_>=.10){
        if(phase_==Phase::pulse&&leg.messages<40)publish(true);
        else {
          if(phase_==Phase::pulse)transition(Phase::zeros,"bounded_zero_sequence",t);
          publish(false);
          if(leg.zeros==5)transition(Phase::stopping,"waiting_for_odom_stop",t);
        }
      }return;
    }
    if(phase_==Phase::stopping){
      require(t-phase_at_<3.,"Odometry did not settle after explicit zeros");
      if(mapping_trial::stationary(odom_)){
        hold_=odom_.back();leg.end_forward=mapping_trial::relative(hold_,origin_).forward;
        transition(Phase::post,leg_==0?"reverse_stationary_capture":"revisit_stationary_capture",t);
      }return;
    }
    if(phase_==Phase::post){
      hold_position();
      if(t-phase_at_>=5.){
        require(leg.scans>=10,"Too few mapping scans during leg");
        if(leg_==0){
          require(leg.end_forward<=-.030,"Less than 30 mm reverse travel; no return/retry");
          ++leg_;transition(Phase::pulse,"return_start",t);
          std::cout<<"Return leg: +0.025 m/s, stop near initial pose or after 40 commands.\n"<<std::flush;
          publish(true);
        }else{
          const auto d=mapping_trial::relative(odom_.back(),origin_);
          require(d.distance<=.020&&std::abs(d.yaw)<=.030,"Revisit outside 20 mm / 0.030 rad acceptance; no correction/retry");
          transition(Phase::done,"software_checks_passed",t);
        }
      }
    }
  }
  void abort_zeros()noexcept{
    if(!attempted_||!rclcpp::ok())return;
    for(int i=0;i<5;++i){try{publish(false,true);}catch(...){}std::this_thread::sleep_for(std::chrono::milliseconds(100));}
    std::cerr<<"Attempted five abort zeros on the existing publisher; confirm physical stop.\n";
  }
  void close(bool complete){
    if(closed_)return;
    if(bag_open_){bag_.close();bag_open_=false;}
    odom_file_.close();events_.close();const auto delta=odom_.empty()?mapping_trial::Delta{}:mapping_trial::relative(odom_.back(),origin_);
    std::ofstream out(dir_/"result.json");out<<std::setprecision(17)<<"{\n  \"complete\": "<<(complete?"true":"false")
      <<",\n  \"motion\": \""<<"reverse_return"<<"\""
      <<",\n  \"motion_publications\": "<<motion_messages_<<",\n  \"zero_publications\": "<<zero_messages_
      <<",\n  \"forward_m\": "<<delta.forward<<",\n  \"lateral_m\": "<<delta.lateral<<",\n  \"yaw_rad\": "<<delta.yaw
      <<",\n  \"scans\": "<<scans_<<",\n  \"scans_in_command_interval\": "<<(legs_[0].scans+legs_[1].scans)<<",\n  \"maps\": "<<maps_
      <<",\n  \"map_cells_changed\": "<<(map_changed_?"true":"false")<<",\n  \"acquisition_time_valid\": 0,\n  \"physical_stop_confirmed\": false\n}\n";
    out.close();require(bool(out),"Result write failed");
    std::ofstream legs(dir_/"legs.csv");
    legs<<"leg,first_command_ns,first_zero_ns,motion_commands,zero_commands,scans,end_forward_m\n";
    for(size_t i=0;i<2;++i){const auto &l=legs_[i];legs<<i<<','<<l.first<<','<<l.stop<<','<<l.messages<<','<<l.zeros<<','<<l.scans<<','<<std::setprecision(17)<<l.end_forward<<'\n';}
    legs.close();require(bool(legs),"Leg result write failed");closed_=true;
  }
};
#ifndef MAPPING_ROUTE_TEST
int main(int argc,char **argv){
  std::shared_ptr<MappingRoute> trial;fs::path dir;
  try{
    require(argc==2&&std::string(argv[1])=="--execute-reverse-return",
      "Usage: run_mapping_route --execute-reverse-return (user-run hardware action)");
    require(std::getenv("ROS_DOMAIN_ID")&&std::string(std::getenv("ROS_DOMAIN_ID"))=="182","Require domain 182");
    require(std::getenv("ROS_AUTOMATIC_DISCOVERY_RANGE")&&std::string(std::getenv("ROS_AUTOMATIC_DISCOVERY_RANGE"))=="LOCALHOST","Require LOCALHOST discovery");
    require(std::getenv("HOME"),"HOME missing");std::string pattern=std::string(std::getenv("HOME"))+"/jetson-mapping-route.XXXXXX";
    char *made=mkdtemp(pattern.data());require(made,"Cannot create new capture directory");dir=made;
    std::cout<<"TRIAL DIRECTORY: "<<dir<<std::endl;
    rclcpp::init(1,argv,rclcpp::InitOptions{},rclcpp::SignalHandlerOptions::None);
    std::signal(SIGINT,trial_signal);std::signal(SIGTERM,trial_signal);
    trial=std::make_shared<MappingRoute>(dir);
    rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(trial);
    while(!trial->done()){require(!trial_interrupted&&rclcpp::ok(),"Interrupted");executor.spin_some();trial->tick();std::this_thread::sleep_for(std::chrono::milliseconds(5));}
    trial->close(true);std::cout<<"SOFTWARE CHECKS PASSED. Confirm physical stop and save a new map. Existing wheel/mapper launches remain running.\n";
    trial.reset();rclcpp::shutdown();return 0;
  }catch(const std::exception &e){
    std::cerr<<"FAILED: "<<e.what()<<"\nNo retry. Retained data: "<<dir<<'\n';
    if(trial){trial->abort_zeros();try{trial->close(false);}catch(const std::exception &error){std::cerr<<error.what()<<'\n';}}
    trial.reset();if(rclcpp::ok())rclcpp::shutdown();return 1;
  }
}
#endif
