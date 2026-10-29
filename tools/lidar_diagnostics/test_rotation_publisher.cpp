// Desktop-only ROS CLI integration test. Never publishes on /cmd_vel or uses hardware.
#define main motion_check_entry
#include "run_motion_check.cpp"
#undef main
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
int main(int argc,char **argv) {
  try {
    require(std::getenv("ROS_DOMAIN_ID") && std::string(std::getenv("ROS_DOMAIN_ID"))=="193","Test requires isolated domain 193");
    require(std::getenv("ROS_AUTOMATIC_DISCOVERY_RANGE") && std::string(std::getenv("ROS_AUTOMATIC_DISCOVERY_RANGE"))=="LOCALHOST","Test requires LOCALHOST");
    rclcpp::init(argc,argv);
    auto node=std::make_shared<rclcpp::Node>("rotation_sequence_test");
    rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(node);
    char pattern[]="/tmp/rotation-publisher-test.XXXXXX";const auto made=mkdtemp(pattern);require(made,"mkdtemp");fs::path dir=made;
    write_exclusive(dir/"pulse.yaml",rotation_sequence());
    auto args=rotation_command(dir/"pulse.yaml");require(args.at(3)=="/cmd_vel","Command layout changed");
    args[3]="/lidar_diagnostics/test_rotation_sequence";
    Child publisher;publisher.start(args,dir/"publisher.log");
    std::vector<geometry_msgs::msg::TwistStamped> received;
    std::vector<Clock::time_point> times;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub;
    const auto start=Clock::now();
    wait_publisher(publisher,dir/"publisher.log",23,3.2,[&]{
      // Force discovery waiting before the test receiver exists.
      if(!sub && Clock::now()-start>=std::chrono::seconds(3))
        sub=node->create_subscription<geometry_msgs::msg::TwistStamped>(args[3],10,[&](geometry_msgs::msg::TwistStamped::ConstSharedPtr m){received.push_back(*m);times.push_back(Clock::now());});
      executor.spin_some();
    });
    executor.spin_some();
    require(received.size()==23,"Test receiver did not receive exactly 23 messages");
    int64_t stamp=0;
    for(size_t i=0;i<received.size();++i) {
      const auto &m=received[i];const auto &t=m.twist;
      const auto next=rclcpp::Time(m.header.stamp).nanoseconds();
      require(m.header.frame_id=="base_footprint" && next>stamp,"Missing/nonadvancing command stamp");stamp=next;
      require(t.linear.x==0 && t.linear.y==0 && t.linear.z==0 && t.angular.x==0 && t.angular.y==0 && t.angular.z==(i<20?.10:0.),"Wrong delivered motion/zero sequence");
    }
    const double span=std::chrono::duration<double>(times.back()-times.front()).count();
    require(span>=1.9 && span<3.2,"Unexpected received sequence duration");
    std::cout<<"PASS actual ROS CLI: delayed receiver, 20 yaw + 3 zero messages on test-only topic; span="<<span<<" s; logs="<<dir<<'\n';
    rclcpp::shutdown();return 0;
  } catch(const std::exception &e){std::cerr<<"FAIL: "<<e.what()<<'\n';if(rclcpp::ok())rclcpp::shutdown();return 1;}
}
