#include <gtest/gtest.h>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/static_transform_broadcaster.hpp"
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>
#include <csignal>
#include <chrono>
#include <cmath>
#include <thread>
extern char ** environ;
using namespace std::chrono_literals;
class StartMaskInterface:public ::testing::Test {
protected:
  void SetUp() override {
    if(!rclcpp::ok()) rclcpp::init(0,nullptr);
    executor=std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    node=std::make_shared<rclcpp::Node>("start_mask_fixture");
    executor->add_node(node);
    scans=node->create_publisher<sensor_msgs::msg::LaserScan>("/lidar_rear/scan",rclcpp::SensorDataQoS());
    odom=node->create_publisher<nav_msgs::msg::Odometry>("/odom",rclcpp::SensorDataQoS());
    hold=node->create_publisher<std_msgs::msg::Bool>("/mission/hold",10);
    output=node->create_subscription<sensor_msgs::msg::LaserScan>("/localization/start_scan",rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr s){received=*s;++count;});
    status=node->create_subscription<std_msgs::msg::String>("/localization/start_status",rclcpp::QoS(1).transient_local(),
      [this](std_msgs::msg::String::ConstSharedPtr s){last_status=s->data;});
    broadcaster=std::make_unique<tf2_ros::StaticTransformBroadcaster>(node);
    geometry_msgs::msg::TransformStamped tf;tf.header.stamp=node->now();
    tf.header.frame_id="base_footprint";tf.child_frame_id="lidar_rear_link";
    tf.transform.translation.x=-.1145;tf.transform.translation.z=.06;tf.transform.rotation.w=1;
    broadcaster->sendTransform(tf);
    std::vector<std::string> args={FILTER_EXECUTABLE,"--ros-args","-p","reviewed_start:=true","-p",std::string("config:=")+START_CONFIG};
    std::vector<char *> argv;for(auto & a:args) argv.push_back(a.data());argv.push_back(nullptr);
    ASSERT_EQ(posix_spawn(&pid,FILTER_EXECUTABLE,nullptr,nullptr,argv.data(),environ),0);
    const auto end=std::chrono::steady_clock::now()+5s;
    while((scans->get_subscription_count()==0 || odom->get_subscription_count()==0 ||
      output->get_publisher_count()==0 || hold->get_subscription_count()==0 || last_status.empty()) &&
      std::chrono::steady_clock::now()<end) {
      executor->spin_some();std::this_thread::sleep_for(20ms);
    }
    ASSERT_GT(scans->get_subscription_count(),0u);
    pump(300ms);  // Let transient static TF reach the child's independent buffer.
  }
  void TearDown() override {
    if(pid>0) {
      kill(pid,SIGTERM);int status=0;
      const auto end=std::chrono::steady_clock::now()+3s;
      while(waitpid(pid,&status,WNOHANG)==0 && std::chrono::steady_clock::now()<end) std::this_thread::sleep_for(20ms);
      if(waitpid(pid,&status,WNOHANG)==0) {kill(pid,SIGKILL);waitpid(pid,&status,0);}
    }
    if(executor && node) executor->remove_node(node);
    broadcaster.reset();status.reset();output.reset();scans.reset();odom.reset();hold.reset();node.reset();
  }
  sensor_msgs::msg::LaserScan makeScan(double increment) {
    sensor_msgs::msg::LaserScan s;s.header.frame_id="lidar_rear_link";
    // Positive and negative acquisition directions both end before current time.
    s.header.stamp=node->now()-rclcpp::Duration::from_seconds(increment>0?.15:.03);
    s.angle_min=0;s.angle_increment=2*std::acos(-1)/720;s.angle_max=719*s.angle_increment;
    s.time_increment=increment;s.scan_time=.1;s.range_min=.05;s.range_max=8;
    for(int i=0;i<720;++i) {
      const double a=i*s.angle_increment,dx=std::cos(a),dy=std::sin(a);
      double r=8;
      if(dx>1e-6) r=std::min(r,(1.0+.1145)/dx);
      if(dx<-1e-6) r=std::min(r,(-.927922771+.1145)/dx);
      if(dy>1e-6) r=std::min(r,.5334964/dy);
      if(dy<-1e-6) r=std::min(r,-.6/dy);
      s.ranges.push_back(r);s.intensities.push_back(42);
    }
    return s;
  }
  void send(sensor_msgs::msg::LaserScan s,bool with_odom=true,double speed=0) {
    if(with_odom) {
      nav_msgs::msg::Odometry o;o.header.frame_id="odom";o.child_frame_id="base_footprint";
      o.header.stamp=node->now();o.pose.pose.orientation.w=1;o.twist.twist.linear.x=speed;odom->publish(o);
      pump(50ms);
    }
    scans->publish(s);pump(100ms);
  }
  void pump(std::chrono::milliseconds duration) {
    const auto end=std::chrono::steady_clock::now()+duration;
    while(std::chrono::steady_clock::now()<end) {executor->spin_some();std::this_thread::sleep_for(5ms);}
  }
  std::shared_ptr<rclcpp::Node> node;pid_t pid=-1;std::size_t count=0;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  sensor_msgs::msg::LaserScan received;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scans;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr hold;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr output;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status;
  std::string last_status;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster;
};
TEST_F(StartMaskInterface, PreservesSignedTimingAndMasksFrontWithoutChangingRetainedRays) {
  send(makeScan(-.1/719),false);EXPECT_EQ(count,0u);
  for(double increment:{-.1/719,.1/719}) {
    const auto input=makeScan(increment);send(input);
    ASSERT_GT(count,0u)<<last_status;
    EXPECT_EQ(received.header,input.header);EXPECT_EQ(received.angle_min,input.angle_min);
    EXPECT_EQ(received.angle_max,input.angle_max);EXPECT_EQ(received.angle_increment,input.angle_increment);
    EXPECT_EQ(received.time_increment,input.time_increment);EXPECT_EQ(received.scan_time,input.scan_time);
    EXPECT_EQ(received.range_min,input.range_min);EXPECT_EQ(received.range_max,input.range_max);
    ASSERT_EQ(received.ranges.size(),input.ranges.size());
    std::size_t kept=0;
    for(std::size_t i=0;i<received.ranges.size();++i) {
      if(std::isfinite(received.ranges[i])) {
        ++kept;EXPECT_FLOAT_EQ(received.ranges[i],input.ranges[i]);EXPECT_FLOAT_EQ(received.intensities[i],42);
        EXPECT_GE(i,190u);EXPECT_LE(i,480u);
      } else {EXPECT_TRUE(std::isnan(received.ranges[i]));EXPECT_FLOAT_EQ(received.intensities[i],0);}
    }
    EXPECT_GT(kept,100u);
  }
}
TEST_F(StartMaskInterface, RejectsStaleFutureAndMissingWallThenLatchesHoldRelease) {
  send(makeScan(-.1/719));ASSERT_GT(count,0u)<<last_status;const auto before=count;
  auto stale=makeScan(-.1/719);stale.header.stamp=node->now()-rclcpp::Duration::from_seconds(1);send(stale);
  auto future=makeScan(.1/719);future.header.stamp=node->now()+rclcpp::Duration::from_seconds(1);send(future);
  auto malformed=makeScan(-.1/719);malformed.angle_max=std::numeric_limits<float>::quiet_NaN();send(malformed);
  auto absent=makeScan(-.1/719);
  for(int i=320;i<=480;++i) absent.ranges[i]=std::numeric_limits<float>::quiet_NaN();
  send(absent);EXPECT_EQ(count,before);
  std_msgs::msg::Bool h;h.data=false;hold->publish(h);pump(100ms);
  send(makeScan(-.1/719));h.data=true;hold->publish(h);pump(100ms);send(makeScan(-.1/719));
  EXPECT_EQ(count,before);
}
TEST_F(StartMaskInterface, MotionLatchesDisabledEvenAfterStopping) {
  send(makeScan(-.1/719));ASSERT_GT(count,0u)<<last_status;const auto before=count;
  send(makeScan(-.1/719),true,.01);send(makeScan(-.1/719),true,0);EXPECT_EQ(count,before);
}
