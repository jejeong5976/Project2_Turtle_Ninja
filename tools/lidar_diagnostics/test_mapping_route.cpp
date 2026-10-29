#define MAPPING_ROUTE_TEST
#include "run_mapping_route.cpp"
#include <rosbag2_cpp/reader.hpp>
#include <functional>
#include <limits>
#include <set>

namespace {
using Cmd = geometry_msgs::msg::TwistStamped;
using Odom = nav_msgs::msg::Odometry;
using Scan = sensor_msgs::msg::LaserScan;
using Map = nav_msgs::msg::OccupancyGrid;
using Text = std_msgs::msg::String;
using TF = tf2_msgs::msg::TFMessage;
using State = lifecycle_msgs::srv::GetState;
const std::string prefix = "/mapping_route_test";

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
  double x = 0, heading = 0, velocity = 0, yaw_velocity = 0;
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
      if(t-moving_since>=delay)x += velocity * (velocity<0?reverse_scale:return_scale) * (t-last_pump);
      heading += yaw_velocity * yaw_scale * (t - last_pump);
    }
    x += creep*(t-last_pump);
    last_pump = t;
    const auto stamp = wheel->now();
    if (t - odom_at >= .020) {
      odom_at = t;
      if (!drop_odom) {
        Odom m; m.header.stamp = stamp; m.header.frame_id = "odom";
        m.child_frame_id = "base_footprint"; m.pose.pose.position.x = x;
        m.pose.pose.orientation.z = std::sin(heading / 2);
        m.pose.pose.orientation.w = std::cos(heading / 2); odom->publish(m);
      }
      TF f;
      geometry_msgs::msg::TransformStamped a;
      a.header.stamp = stamp; a.header.frame_id = "odom"; a.child_frame_id = "base_footprint";
      a.transform.translation.x = x; a.transform.rotation.z = std::sin(heading / 2);
      a.transform.rotation.w = std::cos(heading / 2); f.transforms.push_back(a);
      a.header.frame_id = "map"; a.child_frame_id = "odom"; a.transform.translation.x = 0;
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
      m.info.resolution = .05; m.info.width = 10; m.info.height = 10;
      m.info.origin.orientation.w = 1; m.data.assign(100, -1); m.data[0] = x > .05 || heading > .1 ? 100 : 0;
      map->publish(m);
    }
  }
};

void scenario(const std::string &name,const fs::path &root){
  const auto dir=root/name;fs::create_directory(dir);
  Fixture f;
  if(name=="steering_delay")f.delay=.85;
  if(name=="short_reverse")f.reverse_scale=.1;
  if(name=="short_return")f.return_scale=.1;
  if(name=="wrong_reverse")f.reverse_scale=-1;
  if(name=="preflight_competitor")f.competitor=f.wheel->create_publisher<Cmd>(prefix+"/cmd_vel",10);
  auto trial=std::make_shared<MappingRoute>(dir,prefix);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(f.wheel);executor.add_node(f.input);executor.add_node(f.slam);executor.add_node(trial);
  std::string failure;bool injected=false;size_t commands_at_fault=0;
  const auto positives=[&]{return std::count_if(f.received.begin(),f.received.end(),[](const Cmd &m){return m.twist.linear.x>0;});};
  try{
    while(!trial->done()){
      if(name=="hold_creep"&&!f.received.empty()&&f.moving()>0&&f.received.back().twist.linear.x==0)f.creep=.0002;
      f.pump();executor.spin_some();
      const bool trigger=name.rfind("return_",0)==0?positives()>=5:f.moving()>=5;
      if(!injected&&trigger){
        injected=true;
        if(name=="stale_odom"||name=="return_stale_odom")f.drop_odom=true;
        if(name=="stale_scan"||name=="return_stale_scan")f.drop_scan=true;
        if(name=="input_fault"||name=="return_input_fault")f.fault=true;
        if(name=="inactive_mapper")f.active=false;
        if(name=="competing_publisher")f.competitor=f.wheel->create_publisher<Cmd>(prefix+"/cmd_vel",10);
        if(name=="schedule_stall"){
          const auto until=steady_seconds()+.18;
          while(steady_seconds()<until){f.pump();executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(2));}
        }
      }
      trial->tick();std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  }catch(const std::exception &e){failure=e.what();commands_at_fault=f.moving();trial->abort_zeros();}
  executor.remove_node(trial);
  const auto until=steady_seconds()+.25;
  while(steady_seconds()<until){executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(2));}
  const bool complete=name=="normal"||name=="steering_delay";
  require(complete?failure.empty():!failure.empty(),"Unexpected result: "+name+": "+failure);
  const std::map<std::string,std::string> expected{
    {"stale_odom","fresh odometry"},{"return_stale_odom","fresh odometry"},
    {"stale_scan","fresh mapping scans"},{"return_stale_scan","fresh mapping scans"},
    {"input_fault","Mapping status fault"},{"return_input_fault","Mapping status fault"},
    {"inactive_mapper","Mapper left active"},{"competing_publisher","sole matched command ownership"},
    {"schedule_stall","Command schedule stalled"},{"preflight_competitor","Preflight timeout"},
    {"short_reverse","Less than 30 mm reverse"},{"short_return","Revisit outside"},
    {"wrong_reverse","Route envelope"},{"hold_creep","Motion during stationary capture"}};
  if(!complete)require(failure.find(expected.at(name))!=std::string::npos,"Wrong failure: "+failure);
  trial->close(complete);
  rosbag2_cpp::Reader reader;reader.open((dir/"bag").string());
  std::vector<Cmd> commands;std::set<std::string> topics;bool nan=false;
  while(reader.has_next()){
    auto record=reader.read_next();topics.insert(record->topic_name);
    rclcpp::SerializedMessage data(*record->serialized_data);
    if(record->topic_name==prefix+"/cmd_vel"){
      Cmd m;rclcpp::Serialization<Cmd>().deserialize_message(&data,&m);commands.push_back(m);
    }
    if(record->topic_name==prefix+"/lidar_front/scan_mapping"){
      Scan m;rclcpp::Serialization<Scan>().deserialize_message(&data,&m);nan=nan||std::isnan(m.ranges[1]);
    }
  }
  require(topics.size()==(commands.empty()?6:7)&&nan,"Recorded streams/NaNs");
  require(commands.size()==f.received.size(),"Bag/delivered command count");
  size_t reverse=0,forward=0,zeros=0;bool return_seen=false,final_zero=false;
  int64_t reverse_start=0,reverse_zero=0,return_start=0,return_zero=0;
  for(size_t i=0;i<commands.size();++i){
    const auto &m=commands[i];const auto stamp=rclcpp::Time(m.header.stamp).nanoseconds();
    require(m.twist.linear.x==f.received[i].twist.linear.x&&m.twist.linear.y==0&&m.twist.linear.z==0&&
      m.twist.angular.x==0&&m.twist.angular.y==0&&m.twist.angular.z==0,"Only bounded longitudinal commands");
    if(m.twist.linear.x<0){
      require(!return_seen&&zeros==0&&m.twist.linear.x==-.025,"Reverse order/speed");
      if(!reverse_start)reverse_start=stamp;
      ++reverse;
    }else if(m.twist.linear.x>0){
      require(!final_zero&&zeros==5&&m.twist.linear.x==.025,"Return only after five zeros");
      if(!return_start)return_start=stamp;
      ++forward;return_seen=true;
    }else{
      ++zeros;
      if(return_seen){final_zero=true;if(!return_zero)return_zero=stamp;}
      else if(!reverse_zero)reverse_zero=stamp;
    }
  }
  require(reverse<=40&&forward<=40,"Per-leg command caps");
  if(reverse_start)require(reverse_zero>reverse_start&&(reverse_zero-reverse_start)/1e9<4.3,"Reverse deadline");
  if(return_start)require(return_zero>return_start&&(return_zero-return_start)/1e9<4.3&&
    (return_start-reverse_zero)/1e9>=5.4,"Return deadline and stationary dwell");
  if(complete)require(reverse>=10&&forward>=10&&zeros==10&&std::abs(f.x)<=.020,"Completed revisit and explicit stops");
  else if(trial->attempted()){
    require(f.moving()==commands_at_fault,"No new moving commands after fault");
    require(commands.size()>=5,"Abort zeros recorded");
    for(size_t i=commands.size()-5;i<commands.size();++i)require(commands[i].twist.linear.x==0,"Five final abort zeros");
  }
  if(name=="preflight_competitor")require(commands.empty(),"No preflight commands");
  if(name=="short_reverse"||name=="hold_creep")require(forward==0,"Failed reverse cannot trigger return");
  std::ifstream result(dir/"result.json");std::string json((std::istreambuf_iterator<char>(result)),{});
  require(json.find(complete?"\"complete\": true":"\"complete\": false")!=std::string::npos,"Completion metadata");
  std::cout<<"PASS "<<name<<" reverse="<<reverse<<" forward="<<forward<<" zeros="<<zeros<<" final_x="<<f.x<<" "<<failure<<'\n'<<std::flush;
}
}
int main(int argc,char **argv){
  try{
    require(std::getenv("ROS_DOMAIN_ID")&&std::string(std::getenv("ROS_DOMAIN_ID"))=="193","Tests require domain 193");
    require(std::getenv("ROS_AUTOMATIC_DISCOVERY_RANGE")&&std::string(std::getenv("ROS_AUTOMATIC_DISCOVERY_RANGE"))=="LOCALHOST","Tests require LOCALHOST");
    require(!std::getenv("ROS_STATIC_PEERS")||std::string(std::getenv("ROS_STATIC_PEERS")).empty(),"Test peers must be empty");
    rclcpp::init(argc,argv);std::string pattern="/tmp/mapping-route-test.XXXXXX";
    char *made=mkdtemp(pattern.data());require(made,"Test directory");fs::path root=made;
    std::cout<<"TEST DATA: "<<root<<'\n'<<std::flush;
    if(argc>1){for(int i=1;i<argc;++i)scenario(argv[i],root);}
    else for(const auto *name:{"normal","steering_delay","hold_creep","stale_odom","stale_scan","input_fault","inactive_mapper",
        "competing_publisher","schedule_stall","preflight_competitor","short_reverse","short_return",
        "wrong_reverse","return_stale_odom","return_stale_scan","return_input_fault"})scenario(name,root);
    rclcpp::shutdown();return 0;
  }catch(const std::exception &e){std::cerr<<"TEST FAILED: "<<e.what()<<'\n';if(rclcpp::ok())rclcpp::shutdown();return 1;}
}
