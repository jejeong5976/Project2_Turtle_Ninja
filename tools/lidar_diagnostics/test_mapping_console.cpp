#define MAPPING_CONSOLE_TEST
#include "run_mapping_console.cpp"
#include <rosbag2_cpp/reader.hpp>
#include <functional>
#include <limits>
#include <set>
#include <fcntl.h>

namespace {
using Cmd = geometry_msgs::msg::TwistStamped;
using Odom = nav_msgs::msg::Odometry;
using Scan = sensor_msgs::msg::LaserScan;
using Map = nav_msgs::msg::OccupancyGrid;
using Text = std_msgs::msg::String;
using TF = tf2_msgs::msg::TFMessage;
using State = lifecycle_msgs::srv::GetState;
const std::string prefix = "/mapping_console_test";

struct Fixture {
  rclcpp::Node::SharedPtr wheel = std::make_shared<rclcpp::Node>("swerve_controller", prefix);
  rclcpp::Node::SharedPtr input = std::make_shared<rclcpp::Node>("front_mapping_input", prefix);
  rclcpp::Node::SharedPtr slam = std::make_shared<rclcpp::Node>("slam_toolbox", prefix);
  rclcpp::Publisher<Odom>::SharedPtr odom = wheel->create_publisher<Odom>(prefix + "/odom", 100);
  rclcpp::Publisher<Scan>::SharedPtr scan = input->create_publisher<Scan>(prefix + "/lidar_front/scan_mapping", rclcpp::SensorDataQoS());
  rclcpp::Publisher<Text>::SharedPtr status = input->create_publisher<Text>(prefix + "/lidar_front/mapping_status", 10);
  rclcpp::Publisher<Map>::SharedPtr map = slam->create_publisher<Map>(prefix + "/map", rclcpp::QoS(1).transient_local());
  rclcpp::Publisher<TF>::SharedPtr tf = slam->create_publisher<TF>(prefix + "/tf", 100);
  rclcpp::Publisher<TF>::SharedPtr fixed = input->create_publisher<TF>(prefix + "/tf_static", rclcpp::QoS(1).transient_local());
  rclcpp::Subscription<Cmd>::SharedPtr commands;
  rclcpp::Service<State>::SharedPtr state;
  rclcpp::Publisher<Cmd>::SharedPtr competitor;
  std::vector<Cmd> received;
  double x = 0, y=0, resolution=.05, heading = 0, velocity = 0, yaw_velocity = 0;
  double yaw_scale = 1, translation_drift = 0, last_command = 0, last_pump = steady_seconds();
  double odom_at = 0, scan_at = 0, status_at = 0, map_at = 0;
  int scans = 0;
  double reverse_scale=1, return_scale=1, delay=0, moving_since=0, creep=0;
  int direction=0;
  bool drop_odom = false, drop_scan = false, fault = false, active = true;

  Fixture() {
    commands = wheel->create_subscription<Cmd>(prefix + "/cmd_vel", 100,
      [this](Cmd::ConstSharedPtr m) {
        const int next=m->twist.linear.x<0?-1:(m->twist.linear.x>0?1:0);
        if(next!=direction){direction=next;moving_since=steady_seconds();}
        received.push_back(*m); velocity = m->twist.linear.x; yaw_velocity = m->twist.angular.z;
        last_command = steady_seconds();
      });
    state = slam->create_service<State>(prefix + "/slam_toolbox/get_state",
      [this](std::shared_ptr<State::Request>, std::shared_ptr<State::Response> r) {
        r->current_state.id = active ? 3 : 2;
      });
    TF f;
    geometry_msgs::msg::TransformStamped t;
    t.header.frame_id = "base_footprint"; t.child_frame_id = "lidar_front_link";
    t.transform.translation.x = .110; t.transform.translation.y = .005;
    t.transform.translation.z = .0545; t.transform.rotation.z = 1; t.transform.rotation.w = 0;
    f.transforms.push_back(t); fixed->publish(f);
  }
  size_t moving() const {
    return std::count_if(received.begin(), received.end(), [](const Cmd &m) {
      return m.twist.linear.x != 0 || m.twist.angular.z != 0;
    });
  }
  void pump() {
    const double t = steady_seconds();
    if (t - last_command < .3) {
      if(t-moving_since>=delay){
        const double distance=velocity*(velocity<0?reverse_scale:return_scale)*(t-last_pump);
        x+=std::cos(heading)*distance;y+=std::sin(heading)*distance;
        heading+=yaw_velocity*yaw_scale*(t-last_pump);
      }
    }
    x += creep*(t-last_pump);
    last_pump = t;
    const auto stamp = wheel->now();
    if (t - odom_at >= .020) {
      odom_at = t;
      if (!drop_odom) {
        Odom m; m.header.stamp = stamp; m.header.frame_id = "odom";
        m.child_frame_id = "base_footprint"; m.pose.pose.position.x = x;m.pose.pose.position.y=y;
        m.pose.pose.orientation.z = std::sin(heading / 2);
        m.pose.pose.orientation.w = std::cos(heading / 2); odom->publish(m);
      }
      TF f;
      geometry_msgs::msg::TransformStamped a;
      a.header.stamp = stamp; a.header.frame_id = "odom"; a.child_frame_id = "base_footprint";
      a.transform.translation.x = x;a.transform.translation.y=y; a.transform.rotation.z = std::sin(heading / 2);
      a.transform.rotation.w = std::cos(heading / 2); f.transforms.push_back(a);
      a.header.frame_id = "map"; a.child_frame_id = "odom"; a.transform.translation.x = 0;a.transform.translation.y=0;
      a.transform.rotation.z = 0; a.transform.rotation.w = 1;
      f.transforms.push_back(a); tf->publish(f);
    }
    if (!drop_scan && t - scan_at >= .1) {
      scan_at = t; ++scans;
      Scan m; m.header.stamp = stamp; m.header.frame_id = "lidar_front_link";
      m.angle_increment = 2 * std::acos(-1.) / 1600; m.angle_max = 1599 * m.angle_increment;
      m.range_min = .05; m.range_max = 12;
      m.ranges.assign(1600, std::numeric_limits<float>::quiet_NaN());
      for (int i = 0; i < 1600; i += 4) m.ranges[i] = 2;
      scan->publish(m);
    }
    if (t - status_at >= .2) {
      status_at = t;
      Text m; m.data = "model=nominal assumed_delay_ns=0 acquisition_time_valid=0 bins=1600 published=" +
        std::to_string(scans) + " rejected=3 fault=" + (fault ? "1" : "0") +
        " fresh=1 reference_ns=" + std::to_string(stamp.nanoseconds()) + " reason=publishing";
      status->publish(m);
    }
    if (t - map_at >= .5) {
      map_at = t;
      Map m; m.header.stamp = stamp; m.header.frame_id = "map";
      m.info.resolution = resolution; m.info.width = 10; m.info.height = 10;
      m.info.origin.orientation.w = 1; m.data.assign(100, -1); m.data[0] = x > .05 || heading > .1 ? 100 : 0;
      map->publish(m);
    }
  }
};


void scenario(const std::string &name,const fs::path &root){
  const auto dir=root/name;fs::create_directory(dir);Fixture f;
  if(name=="resolution_025")f.resolution=.025;
  if(name=="delay")f.delay=.85;
  if(name=="no_progress")f.return_scale=0;
  if(name=="wrong_turn")f.yaw_scale=-1;
  if(name=="preflight_competitor")f.competitor=f.wheel->create_publisher<Cmd>(prefix+"/cmd_vel",10);
  rclcpp::Node::SharedPtr observer;rclcpp::Subscription<Cmd>::SharedPtr observer_commands;size_t observed=0;
  if(name=="recorder"||name=="recorder_without_wheel"||name=="unexpected_subscriber"){
    observer=std::make_shared<rclcpp::Node>(name=="unexpected_subscriber"?"unexpected_consumer":"team_mapping_recorder",prefix);
    observer_commands=observer->create_subscription<Cmd>(prefix+"/cmd_vel",100,[&observed](Cmd::ConstSharedPtr){++observed;});
    if(name=="recorder_without_wheel")f.commands.reset();
  }
  auto trial=std::make_shared<MappingConsole>(dir,prefix);
  rclcpp::executors::SingleThreadedExecutor ex;ex.add_node(f.wheel);ex.add_node(f.input);ex.add_node(f.slam);ex.add_node(trial);
  if(observer)ex.add_node(observer);
  const std::vector<std::string> plan=name=="sequence"?std::vector<std::string>{"f 0.30","b 0.05","l 30","r 30","f 0.05"}:
    std::vector<std::string>{name=="wrong_turn"?"r 30":"f 0.10"};
  const bool success=name=="sequence"||name=="normal"||name=="resolution_025"||name=="delay"||name=="accidental_input"||name=="recorder";
  ConsoleInput editor;bool transition_input_checked=false;
  size_t submitted=0,at_fault=0;bool injected=false;double idle_at=0;std::string failure;
  try{
    while(!trial->done()){
      const bool was_ready=trial->accepting_input();
      f.pump();ex.spin_some();trial->tick();
      if(name=="accidental_input"&&!was_ready&&trial->idle()&&submitted==1){
        // Input collected before READY must not turn into an automatic next leg.
        console_input(*trial,editor,"f 0.30\nq\n",false);
        require(trial->idle()&&!trial->done(),"Transition input must be discarded");
        transition_input_checked=true;
      }
      if(trial->idle()){
        if(idle_at==0)idle_at=steady_seconds();
        if(steady_seconds()-idle_at>=.6){
          if(name=="accidental_input"){
            console_input(*trial,editor,std::string(60,'x')+"\n",true);
            console_input(*trial,editor,"f invalid\n",true);
            require(trial->idle()&&!trial->done(),"Malformed READY input must not halt or move");
            console_input(*trial,editor,(submitted<plan.size()?plan[submitted++]:"q")+"\n",true);
          }else trial->submit(submitted<plan.size()?plan[submitted++]:"q");
          idle_at=0;
        }
      }
      if(!injected&&f.moving()>=5){
        injected=true;
        if(name=="stale_odom")f.drop_odom=true;
        if(name=="stale_scan")f.drop_scan=true;
        if(name=="input_fault")f.fault=true;
        if(name=="inactive_mapper")f.active=false;
        if(name=="resolution_change")f.resolution=.025;
        if(name=="competing_publisher")f.competitor=f.wheel->create_publisher<Cmd>(prefix+"/cmd_vel",10);
        if(name=="operator_stop")throw std::runtime_error("Operator Ctrl+C");
        if(name=="accidental_input")console_input(*trial,editor,"\n\x1b[Aq\nf 0.30\n\x04",false);
        if(name=="schedule_stall"){
          const auto until=steady_seconds()+.18;
          while(steady_seconds()<until){f.pump();ex.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(2));}
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  }catch(const std::exception &e){failure=e.what();at_fault=f.moving();trial->abort_zeros();}
  ex.remove_node(trial);const auto until=steady_seconds()+.3;
  while(steady_seconds()<until){ex.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(2));}
  require(success?failure.empty():!failure.empty(),"Unexpected result "+name+": "+failure);
  const std::map<std::string,std::string> expected{{"stale_odom","fresh odometry"},{"stale_scan","fresh mapping scans"},
    {"input_fault","Mapping status fault"},{"inactive_mapper","Mapper left active"},
    {"competing_publisher","sole matched command ownership"},{"preflight_competitor","Preflight timeout"},
    {"recorder_without_wheel","Preflight timeout"},{"unexpected_subscriber","Preflight timeout"},
    {"resolution_change","Map resolution changed"},{"operator_stop","Operator Ctrl+C"},
    {"schedule_stall","Command schedule stalled"},{"no_progress","Insufficient segment travel"},{"wrong_turn","Turn envelope"}};
  if(!success)require(failure.find(expected.at(name))!=std::string::npos,"Wrong failure: "+failure);
  trial->close(success);
  rosbag2_cpp::Reader reader;reader.open((dir/"bag").string());std::vector<Cmd> commands;bool nan=false;
  while(reader.has_next()){
    auto record=reader.read_next();rclcpp::SerializedMessage data(*record->serialized_data);
    if(record->topic_name==prefix+"/cmd_vel"){Cmd m;rclcpp::Serialization<Cmd>().deserialize_message(&data,&m);commands.push_back(m);}
    if(record->topic_name==prefix+"/lidar_front/scan_mapping"){Scan m;rclcpp::Serialization<Scan>().deserialize_message(&data,&m);nan=nan||std::isnan(m.ranges[1]);}
  }
  require(nan&&commands.size()==f.received.size(),"Bag delivery/unknown rays");
  if(name=="recorder")require(observed==commands.size(),"Independent recorder must receive every command");
  for(size_t i=0;i<commands.size();++i){const auto &a=commands[i],&b=f.received[i];
    require(a.twist.linear.x==b.twist.linear.x&&a.twist.angular.z==b.twist.angular.z&&a.twist.linear.y==0,"Recorded command mismatch");}
  if(success){
    if(name=="accidental_input")require(transition_input_checked,"READY transition was not tested");
    size_t i=0;
    for(const auto &line:plan){
      const auto segment=MappingConsole::parse(line);const bool turn=segment.mode=='l'||segment.mode=='r';
      const double sign=segment.mode=='b'||segment.mode=='r'?-1:1;const size_t begin=i;
      while(i<commands.size()&&(commands[i].twist.linear.x!=0||commands[i].twist.angular.z!=0)){
        const auto &m=commands[i];require(m.twist.linear.x==(turn?0:sign*.025)&&m.twist.angular.z==(turn?sign*.1:0),"Segment direction/speed");++i;}
      require(i>begin&&i-begin<=segment.limit,"Finite segment count");
      require((rclcpp::Time(commands[i].header.stamp)-rclcpp::Time(commands[begin].header.stamp)).seconds()<segment.limit*.11+.1,"Segment deadline");
      for(size_t k=0;k<5;++k){require(i<commands.size()&&commands[i].twist.linear.x==0&&commands[i].twist.angular.z==0,"Five stop zeros");++i;}
      if(i<commands.size())require((rclcpp::Time(commands[i].header.stamp)-rclcpp::Time(commands[i-1].header.stamp)).seconds()>2.5,"No automatic next movement during hold");
    }require(i==commands.size(),"No extra commands");
  }else if(trial->attempted()){
    require(f.moving()==at_fault&&commands.size()>=5,"No moving command after fault");
    for(size_t i=commands.size()-5;i<commands.size();++i)require(commands[i].twist.linear.x==0&&commands[i].twist.angular.z==0,"Abort zeros");
  }else require(commands.empty(),"Preflight cannot command motion");
  std::cout<<"PASS "<<name<<" commands="<<commands.size()<<" "<<failure<<'\n'<<std::flush;
}
}
int main(int argc,char **argv){
  try{
    ConsoleInput editor;
    require(!editor.consume("f 0.30\nq\n",false),"Busy input must be discarded");
    require(!editor.consume("\x1b[",true),"Partial arrow sequence");
    require(editor.consume("Af 0.10\nq\n",true)==std::optional<std::string>("f 0.10"),"Arrow ignored; only first command accepted");
    require(!editor.consume("",true),"No pasted second command retained");
    require(!editor.consume(std::string(40,'x')+"\n",true),"Overlong line must be discarded");
    require(editor.consume("q\n",true)==std::optional<std::string>("q"),"Editor must recover after overflow");
    require(!editor.consume("f 0.",true),"Partial command");
    require(!editor.consume("q",false),"Busy transition clears partial command");
    require(!editor.consume("\x04\x1bOP\n",true),"Control/function keys must not halt");
    std::cout<<"PASS input editing, accidental input and pasted-command discard\n";
    // Exercise the actual terminal adapter with a local pseudoterminal only.
    const int saved=dup(STDIN_FILENO),master=posix_openpt(O_RDWR|O_NOCTTY);
    require(saved>=0&&master>=0&&grantpt(master)==0&&unlockpt(master)==0,"PTY creation");
    const int slave=open(ptsname(master),O_RDWR|O_NOCTTY);require(slave>=0&&dup2(slave,STDIN_FILENO)>=0,"PTY attach");
    {
      Terminal terminal;termios settings{};require(tcgetattr(STDIN_FILENO,&settings)==0,"PTY settings");
      require((settings.c_lflag&ISIG)&&settings.c_cc[VINTR]==3&&!(settings.c_iflag&IXON)&&
        settings.c_cc[VSUSP]==_POSIX_VDISABLE&&settings.c_cc[VQUIT]==_POSIX_VDISABLE,"Ctrl+C remains explicit stop; accidental flow/suspend keys disabled");
      const std::string input="f 0.10\n";require(write(master,input.data(),input.size())==ssize_t(input.size()),"PTY write");
      std::string received;for(int i=0;i<50&&received.empty();++i){received=terminal.read_available();std::this_thread::sleep_for(std::chrono::milliseconds(2));}
      require(received==input,"Terminal must deliver a complete command without canonical buffering");
      const std::string paste=std::string(512,'x')+"\nf 0.30\n";
      require(write(master,paste.data(),paste.size())==ssize_t(paste.size()),"PTY long paste");
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      require(terminal.read_available().size()==128,"Long paste must exceed a single read");
      terminal.discard_pending();require(terminal.read_available().empty(),"READY transition must discard remaining paste");
      close(master);bool disconnected=false;try{terminal.read_available();}catch(...){disconnected=true;}
      require(disconnected,"Terminal hangup must abort");
    }
    require(dup2(saved,STDIN_FILENO)>=0,"Restore test stdin");close(saved);close(slave);
    std::cout<<"PASS terminal input and disconnect\n";

    for(const auto *bad:{"f -1","f nan","f 1","b 0.01","l 31","r 0","f 0.1 extra","z 10","ff 0.1"}){
      bool failed=false;try{MappingConsole::parse(bad);}catch(...){failed=true;}require(failed,"Invalid command accepted");
    }
    require(std::getenv("ROS_DOMAIN_ID")&&std::string(std::getenv("ROS_DOMAIN_ID"))=="193","Tests require domain 193");
    require(std::getenv("ROS_AUTOMATIC_DISCOVERY_RANGE")&&std::string(std::getenv("ROS_AUTOMATIC_DISCOVERY_RANGE"))=="LOCALHOST","Tests require LOCALHOST");
    require(!std::getenv("ROS_STATIC_PEERS")||std::string(std::getenv("ROS_STATIC_PEERS")).empty(),"Test peers must be empty");
    rclcpp::init(argc,argv);std::string pattern="/tmp/mapping-console-test.XXXXXX";char *made=mkdtemp(pattern.data());require(made,"Test dir");fs::path root=made;
    std::cout<<"TEST DATA: "<<root<<'\n'<<std::flush;
    if(argc>1){for(int i=1;i<argc;++i)scenario(argv[i],root);}
    else for(const auto *name:{"sequence","accidental_input","resolution_025","delay","stale_odom","stale_scan","input_fault","inactive_mapper",
      "competing_publisher","preflight_competitor","resolution_change","operator_stop","schedule_stall","no_progress","wrong_turn",
      "recorder","recorder_without_wheel","unexpected_subscriber"})scenario(name,root);
    rclcpp::shutdown();return 0;
  }catch(const std::exception &e){std::cerr<<"TEST FAILED: "<<e.what()<<'\n';if(rclcpp::ok())rclcpp::shutdown();return 1;}
}
