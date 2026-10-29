// User-operated finite mapping segments with continuous recording. Never opens serial
// ports, starts/stops drivers, publishes TF, changes configuration or retries motion.
#include "mapping_trial_core.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
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
#include <termios.h>
#include <poll.h>
#include <cerrno>
#include <optional>
namespace fs=std::filesystem;
using mapping_trial::require;
using Steady=std::chrono::steady_clock;
inline double steady_seconds(){return std::chrono::duration<double>(Steady::now().time_since_epoch()).count();}
volatile sig_atomic_t trial_interrupted=0;
void trial_signal(int){trial_interrupted=1;}
class MappingConsole:public rclcpp::Node {
  using Odom=nav_msgs::msg::Odometry;using Scan=sensor_msgs::msg::LaserScan;
  using Map=nav_msgs::msg::OccupancyGrid;using TF=tf2_msgs::msg::TFMessage;
  using Text=std_msgs::msg::String;using Cmd=geometry_msgs::msg::TwistStamped;
  using State=lifecycle_msgs::srv::GetState;
  enum class Phase{waiting,baseline,idle,pulse,zeros,stopping,post,done};
  Phase phase_=Phase::waiting;std::string prefix_;fs::path dir_;
  struct Segment {char mode='f';double target=0;size_t limit=0,messages=0,zeros=0,scans=0;
    int64_t first=0,stop=0;mapping_trial::Delta end{};};
  Segment segment_;size_t segments_=0;double resolution_=0;
  std::ofstream segments_file_;
  mapping_trial::Pose hold_;
  rosbag2_cpp::Writer bag_;bool bag_open_=false,armed_=false,attempted_=false,closed_=false;
  std::ofstream odom_file_,events_;std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;
  rclcpp::Publisher<Cmd>::SharedPtr command_;
  rclcpp::Client<State>::SharedPtr state_client_;
  rclcpp::Client<State>::SharedFuture state_future_;bool state_pending_=false;
  double started_=steady_seconds(),phase_at_=started_,last_command_=0,state_request_=0,state_seen_=0;
  double odom_seen_=0,scan_seen_=0,status_seen_=0,map_seen_=0,map_tf_seen_=0;
  int64_t clock_anchor_=0,last_scan_=-1;
  size_t bytes_=0,records_=0,scans_=0,maps_=0,motion_messages_=0,zero_messages_=0,ignored_input_batches_=0;
  bool state_active_=false,mount_=false,odom_tf_=false,map_changed_=false;
  mapping_trial::Status status_{0,0,0};std::deque<mapping_trial::Pose> odom_;
  mapping_trial::Pose origin_;int64_t starting_rejections_=0;uint64_t map_hash_=0,baseline_map_hash_=0;
  std::string topic(const std::string &name)const{return prefix_+name;}
  void event(const std::string &name){events_<<now().nanoseconds()<<','<<name<<'\n';events_.flush();require(bool(events_),"Event log failed");}
  template<class M> void record(const M &m,const std::string &name){
    auto serialized=std::make_shared<rclcpp::SerializedMessage>();rclcpp::Serialization<M> serializer;
    serializer.serialize_message(&m,serialized.get());bytes_+=serialized->size();
    require(++records_<=500000&&bytes_<=1024ULL*1024*1024,"Recording size/count limit exceeded");
    bag_.write(serialized,topic(name),rosidl_generator_traits::name<M>(),now());
  }
  template<class M,class F> void subscribe(const std::string &name,const rclcpp::QoS &qos,F callback){
    rosbag2_storage::TopicMetadata meta;meta.name=topic(name);meta.type=rosidl_generator_traits::name<M>();
    meta.serialization_format="cdr";meta.offered_qos_profiles={qos};bag_.create_topic(meta);
    subs_.push_back(create_subscription<M>(topic(name),qos,[this,name,callback](typename M::ConstSharedPtr msg){
      record(*msg,name);callback(*msg);
    }));
  }
  void record_optional(const std::string &name,const std::string &type,const rclcpp::QoS &qos){
    rosbag2_storage::TopicMetadata meta;meta.name=topic(name);meta.type=type;meta.serialization_format="cdr";
    meta.offered_qos_profiles={qos};bag_.create_topic(meta);
    subs_.push_back(create_generic_subscription(topic(name),type,qos,
      [this,name,type](std::shared_ptr<rclcpp::SerializedMessage> m){
        bytes_+=m->size();require(++records_<=500000&&bytes_<=1024ULL*1024*1024,"Recording size/count limit exceeded");
        bag_.write(m,topic(name),type,now());
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
    if(segment_.first>0&&stamp>=segment_.first&&(segment_.stop==0||stamp<=segment_.stop))++segment_.scans;
  }
  void mapping_status(const Text &m){
    const auto next=mapping_trial::status(m.data);
    require(next.published>=status_.published&&next.rejected>=status_.rejected&&next.reference>=status_.reference,"Mapping counters regressed");
    if(armed_)require(next.rejected==starting_rejections_,"Mapping input rejected a frame during the trial");
    status_=next;status_seen_=steady_seconds();
  }
  void map(const Map &m){
    require(m.header.frame_id=="map"&&(std::abs(m.info.resolution-.05)<1e-6||std::abs(m.info.resolution-.025)<1e-6)&&m.info.width>0&&m.info.height>0&&
      uint64_t(m.info.width)*m.info.height==m.data.size()&&m.data.size()<=4000000,"Invalid occupancy map");
    if(resolution_==0)resolution_=m.info.resolution;
    require(std::abs(resolution_-m.info.resolution)<1e-8,"Map resolution changed within session");
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
  bool command_consumers(){
    // One actuator is mandatory. The independent, explicitly named bag recorder
    // may also listen; an arbitrary extra subscriber is still a preflight fault.
    const auto endpoints=get_subscriptions_info_by_topic(topic("/cmd_vel"));
    size_t wheels=0,recorders=0;
    for(const auto &endpoint:endpoints){
      if(endpoint.node_namespace()!=(prefix_.empty()?"/":prefix_))return false;
      if(endpoint.node_name()=="swerve_controller")++wheels;
      else if(endpoint.node_name()=="team_mapping_recorder")++recorders;
      else return false;
    }
    return wheels==1&&recorders<=1&&command_->get_subscription_count()==endpoints.size();
  }
  bool healthy(double t,std::string &why){
    auto check=[&](bool v,const char *s){if(!v)why=s;return v;};const auto stamp=now().nanoseconds();
    return check(odom_.size()>=25&&t-odom_seen_<.10&&stamp-odom_.back().stamp>=-5000000&&stamp-odom_.back().stamp<100000000,"fresh odometry")&&
      check(scans_>=10&&t-scan_seen_<.25&&stamp-last_scan_>=-5000000&&stamp-last_scan_<250000000,"fresh mapping scans")&&
      check(status_seen_>0&&t-status_seen_<1.5,"fresh healthy mapping status")&&
      check(state_active_&&t-state_seen_<2.,"active mapper lifecycle")&&
      check(map_seen_>0&&t-map_seen_<6.&&map_tf_seen_>0&&t-map_tf_seen_<1.&&mount_&&odom_tf_,"map and TF reception")&&
      check(owner("/cmd_vel","mapping_console")&&command_consumers(),"sole matched command ownership")&&
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
    if(moving){if(turning())m.twist.angular.z=sign()*.10;else m.twist.linear.x=sign()*.025;}
    // Publication first: a disk error must never prevent an attempted stop.
    command_->publish(m);last_command_=steady_seconds();
    auto &leg=segment_;
    if(moving){attempted_=true;++motion_messages_;++leg.messages;if(leg.first==0)leg.first=rclcpp::Time(m.header.stamp).nanoseconds();}
    else {++zero_messages_;++leg.zeros;if(leg.stop==0)leg.stop=rclcpp::Time(m.header.stamp).nanoseconds();}
    try{record(m,"/cmd_vel");}catch(...){if(!emergency)throw;}
  }
  void transition(Phase phase,const std::string &name,double t){phase_=phase;phase_at_=t;event(name);}
public:
  explicit MappingConsole(const fs::path &dir,std::string prefix=""):
    Node("mapping_console",prefix.empty()?"/":prefix),prefix_(std::move(prefix)),dir_(dir){
    require(!get_parameter("use_sim_time").as_bool(),"System time required");
    require(fs::is_directory(dir_)&&fs::is_empty(dir_)&&fs::space(dir_).available>3ULL*1024*1024*1024,"Require empty output directory and 3 GiB free");
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
    subscribe<sensor_msgs::msg::JointState>("/joint_states",rclcpp::SensorDataQoS(),[](const auto &){});
    record_optional("/slam_toolbox/pose_graph","slam_toolbox/msg/PoseGraph",rclcpp::QoS(1).transient_local());
    record_optional("/slam_toolbox/loop_closure_event","slam_toolbox/msg/LoopClosureEvent",rclcpp::QoS(100));
    record_optional("/slam_toolbox/new_node_event","slam_toolbox/msg/NewNodeEvent",rclcpp::QoS(100));
    segments_file_.open(dir_/"segments.csv");
    segments_file_<<"segment,mode,target,first_command_ns,first_zero_ns,motion_commands,zero_commands,scans,forward_m,lateral_m,yaw_rad\n";
    require(bool(segments_file_),"Cannot create segment log");
    state_client_=create_client<State>(topic("/slam_toolbox/get_state"));event("waiting_for_preflight");
  }
  bool done()const{return phase_==Phase::done;}
  bool attempted()const{return attempted_;}
  bool idle()const{return phase_==Phase::idle;}
  bool accepting_input()const{return idle();}
  bool turning()const{return segment_.mode=='l'||segment_.mode=='r';}
  double sign()const{return segment_.mode=='b'||segment_.mode=='r'?-1.:1.;}
  void route_bounds(const mapping_trial::Pose &p)const{
    if(phase_==Phase::baseline||phase_==Phase::idle)return;
    const auto d=mapping_trial::relative(p,origin_);
    if(turning())require(d.distance<.030&&sign()*d.yaw>-.020&&sign()*d.yaw<segment_.target+.050,
      "Turn envelope: 30 mm translation / wrong direction / angular overshoot");
    else require(sign()*d.forward>-.010&&sign()*d.forward<segment_.target+.030&&std::abs(d.lateral)<.020&&std::abs(d.yaw)<.050,
      "Straight envelope: wrong direction / target +30 mm / 20 mm lateral / 0.050 rad yaw");
  }
  void hold_position()const{
    const auto d=mapping_trial::relative(odom_.back(),hold_);
    require(d.distance<.0005&&std::abs(d.yaw)<.005&&mapping_trial::stationary(odom_),"Motion during stationary hold");
  }
  static Segment parse(const std::string &line){
    std::istringstream in(line);Segment s;double value=0;std::string extra;
    require(bool(in>>s.mode>>value)&&!(in>>extra)&&std::isfinite(value),"Use f/b METRES or l/r DEGREES");
    if(s.mode=='f'||s.mode=='b'){
      require(value>=.05&&value<=.30,"Straight target must be 0.05 to 0.30 m");s.target=value;
      s.limit=size_t(std::ceil(value/.025/.1));
    }else{
      require((s.mode=='l'||s.mode=='r')&&value>=10&&value<=30,"Turn target must be 10 to 30 degrees");
      s.target=value*std::acos(-1.)/180.;s.limit=size_t(std::ceil(s.target/.10/.1));
    }return s;
  }
  void submit(const std::string &line){
    require(idle(),"Motion input accepted only at READY prompt");
    if(line=="q"){transition(Phase::done,"operator_finished",steady_seconds());return;}
    Segment requested=parse(line);
    std::string why;require(healthy(steady_seconds(),why),"Not ready: "+why);hold_position();
    require(++segments_<=100,"Maximum 100 segments per session");segment_=requested;origin_=odom_.back();
    transition(Phase::pulse,std::string("segment_start_")+segment_.mode,steady_seconds());
    std::cout<<"Executing segment "<<segments_<<". Ctrl+C stops and exits. Other input is discarded until READY.\n"<<std::flush;
    publish(true);
  }
  void prompt(){std::cout<<"READY: f/b metres (0.05..0.30), l/r degrees (10..30), q finish; Ctrl+C stops. Confirm swept clearance before Enter.\n> "<<std::flush;}
  void ignored_input(){++ignored_input_batches_;event("ignored_input_not_ready");}
  void tick(){
    const double t=steady_seconds();require(t-started_<900.,"15-minute session limit reached; stop and preserve data");
    require(std::abs(now().nanoseconds()-int64_t(t*1e9)-clock_anchor_)<5000000,"ROS/monotonic clock discontinuity");
    poll_state(t);std::string why;const bool good=healthy(t,why);
    if(phase_==Phase::waiting){
      require(t-started_<10.,"Preflight timeout waiting for "+why);
      if(good&&mapping_trial::stationary(odom_)){
        origin_=hold_=odom_.back();armed_=true;starting_rejections_=status_.rejected;
        transition(Phase::baseline,"stationary_baseline",t);
        std::cout<<"Recording two-second stationary baseline. No movement is automatic.\n"<<std::flush;
      }return;
    }
    require(good,"Lost "+why);route_bounds(odom_.back());
    if(phase_==Phase::baseline||phase_==Phase::idle){
      hold_position();
      if(phase_==Phase::baseline&&t-phase_at_>=2.){
        baseline_map_hash_=map_hash_;transition(Phase::idle,"ready",t);prompt();
      }return;
    }
    if(phase_==Phase::pulse||phase_==Phase::zeros){
      require(t-last_command_<.15,"Command schedule stalled; no catch-up/extension");
      if(phase_==Phase::pulse){
        require(t-phase_at_<segment_.limit*.11+.1,"Segment deadline exceeded");
        const auto d=mapping_trial::relative(odom_.back(),origin_);
        if(sign()*(turning()?d.yaw:d.forward)>=segment_.target){
          transition(Phase::zeros,"target_zero_sequence",t);publish(false);return;
        }
      }
      if(t-last_command_>=.10){
        if(phase_==Phase::pulse&&segment_.messages<segment_.limit)publish(true);
        else{
          if(phase_==Phase::pulse)transition(Phase::zeros,"bounded_zero_sequence",t);
          publish(false);if(segment_.zeros==5)transition(Phase::stopping,"waiting_for_odom_stop",t);
        }
      }return;
    }
    if(phase_==Phase::stopping){
      require(t-phase_at_<3.,"Odometry did not settle after zeros");
      if(mapping_trial::stationary(odom_)){
        hold_=odom_.back();transition(Phase::post,"stationary_capture",t);
      }return;
    }
    if(phase_==Phase::post){
      hold_position();
      if(t-phase_at_>=2.){
        segment_.end=mapping_trial::relative(odom_.back(),origin_);
        require(segment_.scans>=5,"Too few scans during segment");
        require(sign()*(turning()?segment_.end.yaw:segment_.end.forward)>=(turning()?.030:.020),
          "Insufficient segment travel; stop without automatic retry");
        save_segment();
        std::cout<<"Stopped. Measured forward="<<segment_.end.forward<<" m, yaw="<<segment_.end.yaw*180/std::acos(-1.)<<" deg.\n";
        transition(Phase::idle,"ready",t);prompt();
      }
    }
  }
  void save_segment(){
    segments_file_<<segments_<<','<<segment_.mode<<','<<std::setprecision(17)<<segment_.target<<','<<segment_.first<<','<<segment_.stop<<','
      <<segment_.messages<<','<<segment_.zeros<<','<<segment_.scans<<','<<segment_.end.forward<<','<<segment_.end.lateral<<','<<segment_.end.yaw<<'\n';
    segments_file_.flush();require(bool(segments_file_),"Segment log failed");
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
    std::ofstream out(dir_/"result.json");out<<std::setprecision(17)
      <<"{\n  \"complete\": "<<(complete?"true":"false")<<",\n  \"segments_started\": "<<segments_
      <<",\n  \"motion_publications\": "<<motion_messages_<<",\n  \"zero_publications\": "<<zero_messages_
      <<",\n  \"map_resolution_m\": "<<resolution_<<",\n  \"scans\": "<<scans_<<",\n  \"maps\": "<<maps_
      <<",\n  \"last_segment_forward_m\": "<<delta.forward
      <<",\n  \"ignored_input_batches\": "<<ignored_input_batches_
      <<",\n  \"acquisition_time_valid\": 0,\n  \"physical_stop_confirmed\": false\n}\n";
    out.close();segments_file_.close();require(bool(out),"Result write failed");closed_=true;
  }
};
// Ctrl+C is the explicit terminal stop. Other keys never interrupt a segment.
// SSH disconnect/EOF still aborts; typed-ahead input is never queued as motion.
class Terminal {
  termios original_{};bool active_=false;
public:
  Terminal(){
    require(isatty(STDIN_FILENO),"Interactive terminal required; no piped command sequences");
    require(tcgetattr(STDIN_FILENO,&original_)==0,"Cannot read terminal settings");
    auto mode=original_;mode.c_lflag&=~(ICANON|ECHO);mode.c_lflag|=ISIG;
    mode.c_iflag&=~(IXON|IXOFF);mode.c_cc[VINTR]=3;
    mode.c_cc[VQUIT]=_POSIX_VDISABLE;mode.c_cc[VSUSP]=_POSIX_VDISABLE;
    mode.c_cc[VMIN]=0;mode.c_cc[VTIME]=0;
    require(tcsetattr(STDIN_FILENO,TCSANOW,&mode)==0,"Cannot configure terminal");active_=true;
  }
  ~Terminal(){if(active_)tcsetattr(STDIN_FILENO,TCSANOW,&original_);}
  void discard_pending(){require(tcflush(STDIN_FILENO,TCIFLUSH)==0,"Cannot discard queued terminal input");}
  std::string read_available(){
    pollfd fd{STDIN_FILENO,POLLIN,0};const int ready=poll(&fd,1,0);
    require(ready>=0||errno==EINTR,"Terminal poll failed");if(ready<=0)return {};
    require(!(fd.revents&(POLLHUP|POLLERR|POLLNVAL)),"Terminal disconnected");
    if(!(fd.revents&POLLIN))return {};
    char buf[128];const auto n=read(STDIN_FILENO,buf,sizeof(buf));require(n>0,"Terminal EOF/disconnected");
    return std::string(buf,size_t(n));
  }
};
// Assemble at most one deliberate READY command. Arrow/function-key escape
// sequences are ignored, including sequences split between terminal reads.
// An overlong line is discarded in full, rather than executing a truncated prefix.
class ConsoleInput {
  enum class Escape {none, start, csi, ss3};
  std::string line_;Escape escape_=Escape::none;bool overflow_=false;
public:
  std::optional<std::string> consume(const std::string &input,bool ready){
    if(!ready){line_.clear();escape_=Escape::none;overflow_=false;return std::nullopt;}
    for(const unsigned char c:input){
      if(escape_==Escape::start){
        escape_=c=='['?Escape::csi:(c=='O'?Escape::ss3:Escape::none);continue;
      }
      if(escape_!=Escape::none){if(c>=0x40&&c<=0x7e)escape_=Escape::none;continue;}
      if(c==27){escape_=Escape::start;continue;}
      if(c=='\r'||c=='\n'){
        std::cout<<'\n';
        if(overflow_){line_.clear();overflow_=false;std::cout<<"Input too long; discarded.\n> "<<std::flush;return std::nullopt;}
        if(line_.empty())continue;
        auto command=std::move(line_);line_.clear();return command;
      }
      if(overflow_)continue;
      if(c==127||c==8){if(!line_.empty()){line_.pop_back();std::cout<<"\b \b"<<std::flush;}continue;}
      if(c>=32&&c<=126){
        if(line_.size()==32){overflow_=true;continue;}
        line_+=char(c);std::cout<<char(c)<<std::flush;
      }
    }
    return std::nullopt;
  }
};
void console_input(MappingConsole &trial,ConsoleInput &editor,const std::string &input,bool ready_at_read){
  const bool accepting=ready_at_read&&trial.accepting_input();
  if(!accepting&&!input.empty())trial.ignored_input();
  const auto command=editor.consume(input,accepting);
  if(!command)return;
  // Malformed READY input is recoverable. Health errors during submit still stop.
  if(*command!="q"){
    try{MappingConsole::parse(*command);}
    catch(const std::exception &e){std::cout<<e.what()<<"\n> "<<std::flush;return;}
  }
  trial.submit(*command);
}
#ifndef MAPPING_CONSOLE_TEST
int main(int argc,char **argv){
  std::shared_ptr<MappingConsole> trial;fs::path dir;
  try{
    require(argc==2&&std::string(argv[1])=="--supervised","Usage: run_mapping_console --supervised (user-operated hardware action)");
    require(std::getenv("ROS_DOMAIN_ID")&&std::string(std::getenv("ROS_DOMAIN_ID"))=="182","Require domain 182");
    require(std::getenv("ROS_AUTOMATIC_DISCOVERY_RANGE")&&std::string(std::getenv("ROS_AUTOMATIC_DISCOVERY_RANGE"))=="LOCALHOST","Require LOCALHOST discovery");
    Terminal terminal;
    require(std::getenv("HOME"),"HOME missing");std::string pattern=std::string(std::getenv("HOME"))+"/jetson-mapping-session.XXXXXX";
    char *made=mkdtemp(pattern.data());require(made,"Cannot create capture directory");dir=made;
    std::cout<<"SESSION DIRECTORY: "<<dir<<std::endl;
    rclcpp::init(1,argv,rclcpp::InitOptions{},rclcpp::SignalHandlerOptions::None);
    for(int signal:{SIGINT,SIGTERM,SIGHUP})std::signal(signal,trial_signal);
    trial=std::make_shared<MappingConsole>(dir);rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(trial);
    ConsoleInput editor;
    while(!trial->done()){
      require(!trial_interrupted&&rclcpp::ok(),"Interrupted/disconnected");
      const bool ready_at_read=trial->accepting_input();
      const auto input=terminal.read_available(); // drain before a tick can enter READY
      require(!trial_interrupted&&rclcpp::ok(),"Interrupted/disconnected");
      executor.spin_some();trial->tick();
      require(!trial_interrupted&&rclcpp::ok(),"Interrupted/disconnected");
      // A long paste can exceed one read. Clear the remaining kernel queue before
      // a new READY interval, so its tail cannot become an automatic next command.
      if(!ready_at_read&&trial->accepting_input())terminal.discard_pending();
      console_input(*trial,editor,input,ready_at_read);
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    trial->close(true);std::cout<<"SESSION FINISHED. Confirm physical stop; save occupancy map and pose graph. Mapper stays running.\n";
    trial.reset();rclcpp::shutdown();return 0;
  }catch(const std::exception &e){
    std::cerr<<"STOPPED: "<<e.what()<<"\nNo automatic retry. Retained data: "<<dir<<'\n';
    if(trial){trial->abort_zeros();try{trial->close(false);}catch(const std::exception &error){std::cerr<<error.what()<<'\n';}}
    trial.reset();if(rclcpp::ok())rclcpp::shutdown();return 1;
  }
}
#endif
