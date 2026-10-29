#include "ros_helpers.hpp"
#include "processes.hpp"
#include <ament_index_cpp/get_package_share_path.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <slam_toolbox/srv/save_map.hpp>
#include <slam_toolbox/srv/serialize_pose_graph.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <poll.h>
#include <termios.h>
#include <sys/stat.h>

namespace fs=std::filesystem;
using workbench::require;
volatile sig_atomic_t interrupted=0;
void interrupt_handler(int){interrupted=1;}
struct CommandError:std::runtime_error {using std::runtime_error::runtime_error;};
template<class F> auto command_input(F f) {
  try{return f();}catch(const std::exception &e){throw CommandError(e.what());}
}

static void write_yaml(const fs::path &path,const YAML::Node &node) {
  std::ofstream file(path);file<<node<<'\n';require(bool(file),"Cannot save configuration: "+path.string());
}
static std::vector<workbench::ChildSpec> plan(const workbench::Options &o,const fs::path &dir,bool write) {
  const fs::path bringup=ament_index_cpp::get_package_share_path("bringup");
  const fs::path own=ament_index_cpp::get_package_share_path("lidar_workbench");
  YAML::Node common;auto p=common["/**"]["ros__parameters"];
  p["scan_bins"]=o.bins;p["max_linear"]=o.max_linear;p["max_angular"]=o.max_angular;
  p["max_age"]=o.max_age;p["max_skew"]=o.max_skew;p["max_odom_gap"]=o.max_gap;
  p["max_endpoint_error"]=o.endpoint_error;p["max_origin_shift"]=o.origin_shift;
  p["min_range"]=o.min_range;p["max_range"]=o.max_range;p["filter_neighbors"]=o.neighbors;
  p["filter_radius"]=o.filter_radius;p["protect_radius"]=o.protect_radius;
  // ROS infers parameter types from YAML scalars; 12 must be emitted as 12.0.
  for(const auto *key:{"max_linear","max_angular","max_age","max_skew","max_odom_gap","max_endpoint_error",
      "max_origin_shift","min_range","max_range","filter_radius","protect_radius"}) {
    std::ostringstream literal;literal<<std::showpoint<<std::setprecision(17)<<p[key].as<double>();p[key]=literal.str();
  }
  auto slam=YAML::LoadFile((bringup/"config/slam_front_experiment.yaml").string());
  slam["slam_toolbox"]["ros__parameters"]["resolution"]=o.map_resolution;
  auto manifest=YAML::Clone(common);auto m=manifest["session"];
  m["hardware"]=o.hardware;m["domain"]=o.domain;m["speed"]=o.speed;m["turn_rate"]=o.turn_rate;
  m["max_distance"]=o.max_distance;m["max_turn_deg"]=o.max_turn;m["settle_s"]=o.settle;m["hold_s"]=o.hold;
  m["startup_timeout_s"]=o.startup;m["session_seconds"]=o.session_seconds;
  m["map_resolution"]=o.map_resolution;m["timing_model"]="nominal";m["assumed_delay_ns"]=0;m["acquisition_time_valid"]=false;
  m["physical_accuracy_verified"]=false;m["front_port"]=o.front_port;m["rear_port"]=o.rear_port;m["drive_port"]=o.drive_port;
  m["script"]=o.script;m["rviz"]=o.rviz;m["execute"]=o.execute;
  m["mapping_source"]="front scan; merged clouds are observations, not clearing rays";
  m["wheel_profile"]=YAML::LoadFile((bringup/"config/controllers_drive_only.yaml").string());
  m["wheel_hardware"]=YAML::LoadFile((bringup/"config/hardware_map_drive_only.yaml").string());
  m["mounts"]=YAML::LoadFile((bringup/"config/lidar_mounts.yaml").string());
  const auto geometry=m["mounts"];
  for(const auto *key:{"plate_x","plate_y","plate_yaw"})require(geometry[key].as<double>()==0,"Plate offsets changed; workbench baseline must be reviewed");
  require(std::abs(geometry["front_x"].as<double>()-.110)<1e-9&&std::abs(geometry["rear_x"].as<double>()+.110)<1e-9&&
    std::abs(geometry["sensor_y"].as<double>()-.005)<1e-9&&std::abs(geometry["front_yaw"].as<double>()-std::acos(-1.))<1e-9&&
    geometry["rear_yaw"].as<double>()==0&&std::abs(geometry["mount_surface_above_ground"].as<double>()+geometry["h_scan"].as<double>()-.0545)<1e-9,
    "LiDAR geometry changed; workbench baseline must be reviewed");
  if(write){write_yaml(dir/"input.yaml",common);write_yaml(dir/"slam.yaml",slam);write_yaml(dir/"manifest.yaml",manifest);}
  std::vector<workbench::ChildSpec> children;
  children.push_back({"wheels",{"ros2","launch","bringup","drive_only.launch.py","hardware:="+o.hardware,"mode:=drive"}});
  if(o.hardware=="opencr")children.back().argv.push_back("port:="+o.drive_port);
  for(int side=0;side<2;++side) {
    const std::string label=side==0?"front":"rear";const auto mount=workbench::mount(side);
    children.push_back({label+"_tf",{"ros2","run","tf2_ros","static_transform_publisher","--x",workbench::decimal(mount.x),
      "--y","0.005","--z","0.0545","--yaw",workbench::decimal(mount.yaw),"--pitch","0","--roll","0",
      "--frame-id","base_footprint","--child-frame-id","lidar_"+label+"_link","--ros-args","-r","__node:=workbench_"+label+"_mount"}});
    children.push_back({label+"_sensor",{"ros2","run","lidar_workbench","workbench_sensor","--ros-args",
      "-r","__ns:=/lidar_"+label,"--params-file",(dir/"input.yaml").string(),"-p","side:="+std::to_string(side),
      "-p",std::string("mock:=")+(o.hardware=="mock"?"true":"false"),
      "-p","timing_jsonl:="+(dir/(label+"_timing.jsonl")).string(),"-p","timing_scans:=200"}});
    if(o.hardware=="opencr") {
      children.back().argv.push_back("-p");children.back().argv.push_back("port:="+(side==0?o.front_port:o.rear_port));
    }
  }
  children.push_back({"fusion",{"ros2","run","lidar_workbench","workbench_fusion","--ros-args","--params-file",(dir/"input.yaml").string()}});
  children.push_back({"slam",{"ros2","launch","slam_toolbox","online_async_launch.py","use_sim_time:=false",
    "autostart:=true","use_lifecycle_manager:=false","slam_params_file:="+(dir/"slam.yaml").string()}});
  children.push_back({"recorder",{"ros2","bag","record","--storage","sqlite3","--output",(dir/"bag").string(),
    "--qos-profile-overrides-path",(own/"record_qos.yaml").string(),"--topics","/odom","/cmd_vel","/joint_states","/tf","/tf_static","/map",
    "/lidar_front/scan_mapping","/lidar_rear/scan_mapping","/lidar_front/rays","/lidar_rear/rays","/lidar_front/raw_rays","/lidar_rear/raw_rays",
    "/lidar_front/status","/lidar_rear/status","/lidar/status","/lidar/points_merged","/lidar/points_filtered","/lidar/points_rejected"}});
  if(o.rviz)children.push_back({"rviz",{"rviz2","-d",(own/"workbench.rviz").string()},false});
  return children;
}

class Console : public rclcpp::Node {
  using Cmd=geometry_msgs::msg::TwistStamped;
  using Save=slam_toolbox::srv::SaveMap;
  using Graph=slam_toolbox::srv::SerializePoseGraph;
  enum class Phase{waiting,ready,moving,settling,saving,finished};
  Phase phase_=Phase::waiting;
  workbench::Options options_;
  fs::path dir_;
  rclcpp::Publisher<Cmd>::SharedPtr commands_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  rclcpp::Client<Save>::SharedPtr save_;
  rclcpp::Client<Graph>::SharedPtr graph_;
  rclcpp::Client<Save>::SharedFuture save_future_;
  rclcpp::Client<Graph>::SharedFuture graph_future_;
  double started_=workbench::steady_now(),last_tick_=started_,last_command_=0,phase_at_=started_,still_since_=0;
  double odom_seen_=0,map_seen_=0,map_tf_seen_=0;
  std::array<double,3> status_seen_{{0,0,0}};
  std::array<std::string,3> health_{{"waiting","waiting","waiting"}};
  std::array<bool,2> mounts_{{false,false}};
  bool have_odom_=false,stationary_=false,quit_after_stop_=false,ever_ready_=false,segment_pending_=false;
  lidar_motion::Odom odom_{};
  workbench::Pose start_{};
  workbench::Segment segment_;
  double angle_=0;
  size_t segments_=0;
  std::ofstream events_,odom_file_,segments_file_;
  std::string save_base_;

  void event(const std::string &message) {
    events_<<now().nanoseconds()<<','<<message<<'\n';events_.flush();require(bool(events_),"Session event log failed");
  }
  void odometry(const nav_msgs::msg::Odometry &m) {
    const auto next=workbench::decode_odom(m);
    require(!have_odom_||next.stamp_ns>odom_.stamp_ns,"Odometry timestamp regressed");
    if(have_odom_&&phase_==Phase::moving)angle_+=lidar_motion::wrap(next.pose.yaw-odom_.pose.yaw);
    const auto &v=m.twist.twist;
    require(std::isfinite(v.linear.x)&&std::isfinite(v.linear.y)&&std::isfinite(v.angular.z),"Invalid odometry velocity");
    stationary_=std::hypot(v.linear.x,v.linear.y)<.005 && std::abs(v.angular.z)<.02;
    odom_=next;have_odom_=true;odom_seen_=workbench::steady_now();
    odom_file_<<next.stamp_ns<<','<<std::setprecision(17)<<next.pose.x<<','<<next.pose.y<<','<<next.pose.yaw<<'\n';
    require(bool(odom_file_),"Odometry log failed");
  }
  void tf(const tf2_msgs::msg::TFMessage &m,bool fixed) {
    for(const auto &t:m.transforms) {
      if(t.child_frame_id=="odom") {
        require(!fixed&&t.header.frame_id=="map","Unexpected map/odom TF parent");
        const auto &p=t.transform.translation;
        require(std::isfinite(p.x)&&std::isfinite(p.y)&&std::abs(p.z)<1e-3,"Invalid map/odom translation");
        workbench::planar_yaw(t.transform.rotation);map_tf_seen_=workbench::steady_now();
      }
      if(t.child_frame_id=="base_footprint")require(!fixed&&t.header.frame_id=="odom","Unexpected odometry TF parent");
      for(int side=0;side<2;++side)if(t.child_frame_id==(side==0?"lidar_front_link":"lidar_rear_link")) {
        const auto expected=workbench::mount(side);const auto &p=t.transform.translation;
        require(fixed&&t.header.frame_id=="base_footprint"&&std::abs(p.x-expected.x)<1e-6&&std::abs(p.y-expected.y)<1e-6&&
          std::abs(p.z-.0545)<1e-6&&std::abs(lidar_motion::wrap(workbench::planar_yaw(t.transform.rotation)-expected.yaw))<1e-6,
          "Unexpected sensor TF geometry/parent");mounts_[side]=true;
      }
    }
  }
  bool healthy(std::string &why) {
    const auto t=workbench::steady_now();
    auto check=[&](bool v,const std::string &s){if(!v)why=s;return v;};
    if(!check(have_odom_&&t-odom_seen_<.10&&now().nanoseconds()-odom_.stamp_ns<100000000&&
      odom_.stamp_ns<=now().nanoseconds()+5000000,"fresh odometry"))return false;
    for(int i=0;i<3;++i)if(!check(t-status_seen_[i]<.30&&health_[i]=="ready",std::string(i==0?"front":i==1?"rear":"fusion")+": "+health_[i]))return false;
    if(!check(mounts_[0]&&mounts_[1]&&t-map_seen_<6.&&t-map_tf_seen_<1.,"map and complete TF"))return false;
    const auto pubs=get_publishers_info_by_topic("/cmd_vel"),subs=get_subscriptions_info_by_topic("/cmd_vel");
    size_t wheels=0,recorders=0;for(const auto &s:subs) {
      if(s.node_name()=="swerve_controller"&&s.node_namespace()=="/")++wheels;
      if(s.node_name()=="rosbag2_recorder"&&s.node_namespace()=="/")++recorders;
    }
    if(!check(pubs.size()==1&&pubs[0].node_name()=="lidar_workbench"&&wheels==1&&recorders==1,"sole command publisher, wheel controller and recorder"))return false;
    for(const auto &topic:{std::string("/odom"),std::string("/map"),std::string("/lidar_front/rays"),std::string("/lidar_rear/rays")})
      if(!check(get_publishers_info_by_topic(topic).size()==1,"single source owner: "+topic))return false;
    return true;
  }
  void publish(bool moving) {
    Cmd cmd;cmd.header.stamp=now();cmd.header.frame_id="base_footprint";
    if(moving){cmd.twist.linear.x=segment_.x*segment_.speed;cmd.twist.linear.y=segment_.y*segment_.speed;cmd.twist.angular.z=segment_.yaw*segment_.speed;}
    commands_->publish(cmd);last_command_=workbench::steady_now();
  }
  void stop(const std::string &why) {
    publish(false);event(why);phase_=Phase::settling;phase_at_=workbench::steady_now();still_since_=0;
  }
  void ready() {
    phase_=Phase::ready;ever_ready_=true;event("ready");
    if(isatty(STDIN_FILENO))tcflush(STDIN_FILENO,TCIFLUSH);
    std::cout<<"READY: forward/backward/left/right metres; ccw/cw degrees; status, stop, save NAME, quit\n> "<<std::flush;
  }
public:
  Console(workbench::Options options,const fs::path &dir):Node("lidar_workbench"),options_(std::move(options)),dir_(dir) {
    events_.open(dir/"events.csv");events_<<"stamp_ns,event\n";
    odom_file_.open(dir/"odom.csv");odom_file_<<"stamp_ns,x_m,y_m,yaw_rad\n";
    segments_file_.open(dir/"segments.csv");segments_file_<<"segment,dx_m,dy_m,yaw_rad,target,turning\n";
    require(events_&&odom_file_&&segments_file_,"Cannot create session logs");
    commands_=create_publisher<Cmd>("/cmd_vel",10);
    subscriptions_.push_back(create_subscription<nav_msgs::msg::Odometry>("/odom",100,[this](nav_msgs::msg::Odometry::ConstSharedPtr m){odometry(*m);}));
    const std::array<std::string,3> names{{"/lidar_front/status","/lidar_rear/status","/lidar/status"}};
    for(int i=0;i<3;++i)subscriptions_.push_back(create_subscription<std_msgs::msg::String>(names[i],10,[this,i](std_msgs::msg::String::ConstSharedPtr m){health_[i]=m->data;status_seen_[i]=workbench::steady_now();}));
    subscriptions_.push_back(create_subscription<nav_msgs::msg::OccupancyGrid>("/map",rclcpp::QoS(1).transient_local(),[this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m){
      require(m->header.frame_id=="map"&&std::abs(m->info.resolution-options_.map_resolution)<1e-6&&m->info.width>0&&m->info.height>0&&
        uint64_t(m->info.width)*m->info.height==m->data.size(),"Invalid map metadata");map_seen_=workbench::steady_now();}));
    subscriptions_.push_back(create_subscription<tf2_msgs::msg::TFMessage>("/tf",rclcpp::QoS(100).best_effort(),[this](tf2_msgs::msg::TFMessage::ConstSharedPtr m){tf(*m,false);}));
    subscriptions_.push_back(create_subscription<tf2_msgs::msg::TFMessage>("/tf_static",rclcpp::QoS(100).transient_local(),[this](tf2_msgs::msg::TFMessage::ConstSharedPtr m){tf(*m,true);}));
    save_=create_client<Save>("/slam_toolbox/save_map");graph_=create_client<Graph>("/slam_toolbox/serialize_map");
    event("waiting_for_integrated_health");
  }
  bool is_ready()const{return phase_==Phase::ready;}
  bool done()const{return phase_==Phase::finished;}
  void zeros() noexcept {try{for(int i=0;i<6;++i){publish(false);std::this_thread::sleep_for(std::chrono::milliseconds(50));}}catch(...) {}}
  void fault(const std::string &reason) noexcept {try{publish(false);event("fault: "+reason);}catch(...) {}}
  void submit(const std::string &line) {
    if(line=="status") {std::string why;std::cout<<(healthy(why)?"Healthy":"Waiting/fault: "+why)<<"; speed="<<options_.speed<<" turn-rate="<<options_.turn_rate<<'\n';return;}
    if(line=="quit"){quit_after_stop_=true;stop("operator_quit");return;}
    if(line=="stop"){stop("operator_stop");return;}
    if(!is_ready()){std::cout<<"Input discarded: not READY. No motion is queued.\n";return;}
    std::istringstream input(line);std::string command,value,extra;input>>command>>value;
    if(command=="save") {
      command_input([&]{require(!value.empty()&&value.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-")==std::string::npos&&!(input>>extra),"Use save NAME (letters, digits, underscore, hyphen)");});
      require(save_->service_is_ready()&&graph_->service_is_ready(),"Map-saving services unavailable");
      const auto base=(dir_/"maps"/value).string();
      command_input([&]{require(!fs::exists(base+".yaml")&&!fs::exists(base+".pgm")&&!fs::exists(base+".posegraph")&&!fs::exists(base+".data"),"Map name already exists");});
      auto s=std::make_shared<Save::Request>();s->name.data=base;
      auto g=std::make_shared<Graph::Request>();g->filename=base;
      save_base_=base;
      save_future_=save_->async_send_request(s).share();graph_future_=graph_->async_send_request(g).share();
      phase_=Phase::saving;phase_at_=workbench::steady_now();event("save_requested_"+value);return;
    }
    if(command=="speed"||command=="turn-rate") {
      auto changed=options_;
      command_input([&]{require(!value.empty()&&!(input>>extra),"Use speed M/S or turn-rate RAD/S");
        (command=="speed"?changed.speed:changed.turn_rate)=workbench::number(value);changed.validate();});
      options_=changed;
      event(command+"_"+value);std::cout<<"Updated "<<command<<" = "<<value<<"\n> "<<std::flush;return;
    }
    const auto next=command_input([&]{return workbench::parse_segment(line,options_);});
    std::string why;require(healthy(why)&&stationary_,"Cannot start motion: "+why);
    segment_=next;start_=odom_.pose;angle_=0;++segments_;segment_pending_=true;
    phase_=Phase::moving;phase_at_=workbench::steady_now();event("motion_"+line);publish(true);
  }
  void tick() {
    const double t=workbench::steady_now();
    if(phase_==Phase::moving)require(t-last_tick_<.15,"Control schedule stalled");
    last_tick_=t;
    require(t-started_<options_.session_seconds,"Session duration limit reached");
    std::string why;const bool good=healthy(why);
    if(phase_==Phase::waiting) {
      require(t-started_<options_.startup,"Startup timeout: "+why);
      if(good&&stationary_){if(still_since_==0)still_since_=t;if(t-still_since_>=options_.hold)ready();}
      else still_since_=0;
      return;
    }
    require(good||(!ever_ready_&&quit_after_stop_),"Integrated health lost: "+why);
    if(phase_==Phase::moving) {
      const auto delta=lidar_motion::inverse(start_,{odom_.pose.x,odom_.pose.y});
      const double along=delta.x*segment_.x+delta.y*segment_.y;
      const double cross=-delta.x*segment_.y+delta.y*segment_.x;
      if(segment_.turning)require(std::hypot(delta.x,delta.y)<.03&&angle_*segment_.yaw>-.02&&
        angle_*segment_.yaw<segment_.target+.05,"Rotation departed its bounded envelope");
      else require(along>-.01&&along<segment_.target+.03&&std::abs(cross)<.02&&
        std::abs(angle_)<.05,"Translation departed its bounded envelope");
      require(t-phase_at_<segment_.target/segment_.speed+3.,"Segment timed out; no extension or retry");
      if((segment_.turning?angle_*segment_.yaw:along)>=segment_.target){stop("target_reached");return;}
      if(t-last_command_>=.05)publish(true);
    } else if(phase_==Phase::settling) {
      if(t-last_command_>=.05)publish(false);
      require(t-phase_at_<5+options_.settle+options_.hold,"Wheel odometry did not settle");
      if(stationary_) {if(still_since_==0)still_since_=t;}
      else still_since_=0;
      if(still_since_>0&&t-still_since_>=options_.settle+options_.hold) {
        if(segment_pending_) {
          const auto delta=lidar_motion::inverse(start_,{odom_.pose.x,odom_.pose.y});
          segments_file_<<segments_<<','<<delta.x<<','<<delta.y<<','<<lidar_motion::wrap(odom_.pose.yaw-start_.yaw)<<','<<segment_.target<<','<<segment_.turning<<'\n';segments_file_.flush();
          require(bool(segments_file_),"Segment log failed");segment_pending_=false;
        }
        if(quit_after_stop_){phase_=Phase::finished;event("finished");}else ready();
      }
    } else if(phase_==Phase::saving) {
      require(stationary_,"Motion during map save");
      require(t-phase_at_<15,"Map-saving request timed out");
      if(save_future_.wait_for(std::chrono::seconds(0))==std::future_status::ready&&graph_future_.wait_for(std::chrono::seconds(0))==std::future_status::ready) {
        require(save_future_.get()->result==0&&graph_future_.get()->result==0,"Map or graph save failed");
        for(const auto *suffix:{".yaml",".pgm",".posegraph",".data"})
          require(fs::is_regular_file(save_base_+suffix)&&fs::file_size(save_base_+suffix)>0,"Map saving did not produce all expected files");
        event("map_and_graph_saved");std::cout<<"Saved occupancy map and pose graph in "<<dir_/"maps"<<'\n';ready();
      }
    } else if(phase_==Phase::ready) require(stationary_,"Unexpected motion while READY");
  }
};

static void device_check(const workbench::Options &o) {
  if(o.hardware!="opencr")return;
  std::vector<std::string> resolved;
  for(const auto &port:{o.drive_port,o.front_port,o.rear_port}) {
    const auto p=fs::canonical(port).string();
    require(std::find(resolved.begin(),resolved.end(),p)==resolved.end(),"Two device aliases resolve to the same port");resolved.push_back(p);
    struct stat info{};require(::stat(p.c_str(),&info)==0&&S_ISCHR(info.st_mode),"Expected character device: "+p);
    const auto pid=fork();require(pid>=0,"Cannot inspect device ownership");
    if(pid==0){execlp("fuser","fuser",p.c_str(),static_cast<char*>(nullptr));_exit(127);}
    int result=0;require(waitpid(pid,&result,0)==pid&&WIFEXITED(result)&&WEXITSTATUS(result)==1,"Device is owned or inspection failed: "+p);
  }
}

int main(int argc,char **argv) {
  std::shared_ptr<Console> console;
  workbench::Processes processes;
  fs::path directory;
  try {
    const auto options=workbench::parse_options({argv+1,argv+argc});
    if(options.help){std::cout<<workbench::help_text();return 0;}
    directory=options.output.empty()?fs::path("/tmp/lidar-workbench-"+std::to_string(getpid())):fs::absolute(options.output);
    const auto children=plan(options,directory,false);
    if(options.dry_run) {
      std::cout<<"DRY RUN: no ROS nodes, devices, or files opened for execution.\n";
      for(const auto &c:children){std::cout<<c.name<<":";for(const auto &a:c.argv)std::cout<<' '<<std::quoted(a);std::cout<<'\n';}
      return 0;
    }
    require(isatty(STDIN_FILENO)||!options.script.empty(),"Interactive terminal required (or mock-only --script)");
    device_check(options);
    require(!fs::exists(directory),"Output directory already exists; choose a new session path");
    fs::create_directories(directory/"maps");fs::create_directories(directory/"ros_home");fs::create_directories(directory/"ros_logs");
    setenv("ROS_DOMAIN_ID",std::to_string(options.domain).c_str(),1);
    setenv("ROS_AUTOMATIC_DISCOVERY_RANGE","LOCALHOST",1);unsetenv("ROS_STATIC_PEERS");
    setenv("ROS_HOME",(directory/"ros_home").c_str(),1);setenv("ROS_LOG_DIR",(directory/"ros_logs").c_str(),1);
    // Inspect graph isolation before starting any torque-enabled child.
    rclcpp::init(1,argv,rclcpp::InitOptions{},rclcpp::SignalHandlerOptions::None);
    {
      auto preflight=std::make_shared<rclcpp::Node>("workbench_preflight");
      rclcpp::executors::SingleThreadedExecutor inspection;inspection.add_node(preflight);
      const auto deadline=workbench::steady_now()+1.;
      while(workbench::steady_now()<deadline){inspection.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(20));}
      for(const auto &name:preflight->get_node_names())require(name=="/workbench_preflight"||name.find("/_ros2cli_daemon_")==0,"ROS domain already contains a node: "+name);
    }
    rclcpp::shutdown();
    for(int sig:{SIGINT,SIGTERM,SIGHUP})std::signal(sig,interrupt_handler);
    const auto prepared=plan(options,directory,true);
    std::ofstream child_log(directory/"commands.txt");
    for(const auto &c:prepared){for(const auto &a:c.argv)child_log<<std::quoted(a)<<' ';child_log<<'\n';processes.start(c,directory);}
    require(bool(child_log),"Cannot record child command manifest");child_log.close();
    rclcpp::init(1,argv,rclcpp::InitOptions{},rclcpp::SignalHandlerOptions::None);
    console=std::make_shared<Console>(options,directory);
    rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(console);
    std::ifstream script;if(!options.script.empty()){script.open(options.script);require(bool(script),"Cannot open mock script");}
    std::cout<<"SESSION "<<directory<<"\n"<<(options.hardware=="mock"?"MOCK: no devices opened":"REAL: wheel hardware active; commands require swept clearance")
      <<"\nWaiting for both LiDARs, odometry, fusion, and SLAM.\n"<<std::flush;
    std::string pending;
    while(!console->done()) {
      require(!interrupted&&rclcpp::ok(),"Interrupted or terminal disconnected");processes.check();
      const bool was_ready=console->is_ready();executor.spin_some();console->tick();
      if(script.is_open()) {
        if(console->is_ready()){std::string line;if(!std::getline(script,line))line="quit";if(!line.empty()&&line[0]!='#')console->submit(line);}
      } else {
        pollfd input{STDIN_FILENO,POLLIN,0};const int result=poll(&input,1,0);
        require(result>=0||errno==EINTR,"Terminal poll failed");
        if(result>0&&(input.revents&(POLLHUP|POLLERR|POLLNVAL)))throw std::runtime_error("Terminal disconnected");
        if(result>0&&(input.revents&POLLIN)) {
          char buffer[4096];const auto count=read(STDIN_FILENO,buffer,sizeof(buffer));require(count>0,"Terminal EOF");pending.append(buffer,static_cast<size_t>(count));
          require(pending.size()<=4096,"Terminal input too long");const auto end=pending.find('\n');
          if(end!=std::string::npos) {
            const auto line=pending.substr(0,end);pending.clear(); // discard pasted trailing commands
            if(was_ready||line=="stop"||line=="quit"||line=="status") {
              // Only operator syntax/configuration errors are recoverable; runtime faults stop the session.
              try{console->submit(line);}catch(const CommandError &e){std::cout<<"Command rejected: "<<e.what()<<"\n> "<<std::flush;}
            } else std::cout<<"Input discarded while not READY.\n";
          }
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    console->zeros();console.reset();rclcpp::shutdown();processes.stop();
    std::cout<<"Session finished. Retained data: "<<directory<<'\n';return 0;
  } catch(const std::exception &e) {
    std::cerr<<"STOPPED: "<<e.what()<<"\nRetained data: "<<directory<<"\nNo automatic retry.\n";
    if(console){console->fault(e.what());console->zeros();}
    console.reset();if(rclcpp::ok())rclcpp::shutdown();processes.stop();return 1;
  }
}
