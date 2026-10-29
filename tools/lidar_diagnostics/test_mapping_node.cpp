#define LIDAR_MAPPING_NODE_TEST
#include "mapping_node.cpp"
void check(bool v,const char *why){if(!v)throw std::runtime_error(why);}
int main(int argc,char **argv){
 try{
  check(std::getenv("ROS_DOMAIN_ID")&&std::string(std::getenv("ROS_DOMAIN_ID"))=="193","Test requires domain 193");
  check(std::getenv("ROS_AUTOMATIC_DISCOVERY_RANGE")&&std::string(std::getenv("ROS_AUTOMATIC_DISCOVERY_RANGE"))=="LOCALHOST","Test requires LOCALHOST");
  rclcpp::init(argc,argv);rclcpp::NodeOptions options;
  options.parameter_overrides({{"mapping_experiment",true},{"timing_model","nominal"},{"assumed_delay_ns",int64_t(0)},
    {"port","/dev/serial/by-path/platform-3610000.usb-usb-0:2.4.4:1.0-port0"}});
  auto mapping=std::make_shared<MappingNode>(false,options); // No sensor construction/open.
  auto fixture=std::make_shared<rclcpp::Node>("mapping_fixture");
  auto pub=fixture->create_publisher<nav_msgs::msg::Odometry>("/odom",100);
  size_t received=0;auto sub=fixture->create_subscription<sensor_msgs::msg::LaserScan>("/lidar_front/scan_mapping",rclcpp::SensorDataQoS(),[&](sensor_msgs::msg::LaserScan::ConstSharedPtr s){
    check(s->ranges.size()==1600&&s->time_increment==0&&s->header.frame_id=="lidar_front_link","Wrong output metadata");
    size_t finite=0;for(auto r:s->ranges){check(!std::isinf(r),"Infinity fabricated");finite+=std::isfinite(r);}check(finite==40,"Unexpected finite count");++received;
  });
  rclcpp::executors::SingleThreadedExecutor ex;ex.add_node(mapping);ex.add_node(fixture);
  int64_t last_stamp=0;
  auto spin=[&](double seconds,bool feed){const auto end=std::chrono::steady_clock::now()+std::chrono::duration<double>(seconds);
    while(std::chrono::steady_clock::now()<end){if(feed){nav_msgs::msg::Odometry o;o.header.frame_id="odom";o.child_frame_id="base_footprint";o.header.stamp=fixture->now();last_stamp=rclcpp::Time(o.header.stamp).nanoseconds();o.pose.pose.orientation.w=1;pub->publish(o);}ex.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(20));}
  };
  int64_t id=0;auto frame=[&]{lidar_mapping::Frame f;f.id=++id;f.count=400;f.source_valid=true;
    f.clock.before_ns=lidar_timing::mono_ns();f.clock.ros_ns=fixture->now().nanoseconds();f.clock.after_ns=lidar_timing::mono_ns();f.end=f.clock.before_ns-1000;f.start=f.end-100000000;
    for(int i=0;i<40;++i)f.beams.push_back({i*10,i*10,2.,-2*std::acos(-1.)*i/40});
    return f;};
  spin(3,true);mapping->inject(frame());spin(.3,true);check(received==1,"Fresh frame not published");
  auto stale=frame();stale.start-=1000000000;stale.end-=1000000000;mapping->inject(stale);spin(.3,true);check(received==1,"Stale frame published");
  spin(.15,false);mapping->inject(frame());spin(.2,false);check(received==1,"Stale odometry accepted");
  spin(.3,true);mapping->inject(frame());spin(.3,true);check(received==2,"Fresh data did not recover");
  nav_msgs::msg::Odometry regressed;regressed.header.frame_id="odom";regressed.child_frame_id="base_footprint";regressed.header.stamp=rclcpp::Time(last_stamp-20000000);regressed.pose.pose.orientation.w=1;pub->publish(regressed);
  spin(.05,false);check(mapping->faulted(),"Regression not latched");mapping->inject(frame());spin(.3,true);check(received==2,"Published after clock/odom fault");
  rclcpp::shutdown();std::cout<<"PASS hardware-free live node: fresh output, NaN preservation, stale scan/odom rejection, recovery, regression latch\n";return 0;
 }catch(const std::exception &e){std::cerr<<"FAIL: "<<e.what()<<'\n';if(rclcpp::ok())rclcpp::shutdown();return 1;}
}
