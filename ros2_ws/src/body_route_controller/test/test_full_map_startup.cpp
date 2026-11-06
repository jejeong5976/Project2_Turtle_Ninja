#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>
#include <nlohmann/json.hpp>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>
#include <csignal>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <thread>
extern char ** environ;
using namespace std::chrono_literals;
namespace fs=std::filesystem;
using Json=nlohmann::json;
class FullMapStartup:public ::testing::Test {
protected:
  void SetUp() override {
    if(!rclcpp::ok())rclcpp::init(0,nullptr);
    dir=fs::temp_directory_path()/("full-map-startup-"+std::to_string(getpid())+"-"+std::to_string(counter++));fs::create_directory(dir);
    node=std::make_shared<rclcpp::Node>("full_map_fixture");
    hold=std::make_shared<rclcpp::Node>("cmd_vel_hold");hold->declare_parameter("start_hold",true);hold->declare_parameter("hold_timeout_s",2.5);
    amcl=std::make_shared<rclcpp::Node>("amcl");amcl->declare_parameter("scan_topic","/lidar_front/scan");
    executor=std::make_unique<rclcpp::executors::SingleThreadedExecutor>();for(auto n:{node,hold,amcl})executor->add_node(n);
    odom=node->create_publisher<nav_msgs::msg::Odometry>("/odom",rclcpp::SensorDataQoS());
    cmd=hold->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel",10);
    for(int i=0;i<2;++i)scans[i]=node->create_publisher<sensor_msgs::msg::LaserScan>(i==0?"/lidar_front/scan":"/lidar_rear/scan",rclcpp::SensorDataQoS());
    initial=amcl->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose",10,
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr m){received=*m;++initial_count;});
    broadcaster=std::make_unique<tf2_ros::StaticTransformBroadcaster>(node);
    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    for(int i=0;i<2;++i){geometry_msgs::msg::TransformStamped t;t.header.frame_id="base_footprint";t.child_frame_id=i==0?"lidar_front_link":"lidar_rear_link";t.header.stamp=node->now();t.transform.translation.x=i==0?.2:-.1;t.transform.rotation.w=1;transforms.push_back(t);}
    broadcaster->sendTransform(transforms); // Deliberately no map -> odom or AMCL pose.
  }
  void TearDown() override {
    if(pid>0){kill(pid,SIGKILL);waitpid(pid,nullptr,0);pid=-1;}
    for(auto n:{node,hold,amcl})executor->remove_node(n);
    executor.reset();broadcaster.reset();initial.reset();scans={};odom.reset();cmd.reset();hold_pub.reset();node.reset();amcl.reset();hold.reset();fs::remove_all(dir);
  }
  int run(std::vector<std::string> args) {
    args.insert(args.begin(),STARTUP_EXECUTABLE);std::vector<char*> argv;for(auto &a:args)argv.push_back(a.data());argv.push_back(nullptr);
    if(posix_spawn(&pid,argv[0],nullptr,nullptr,argv.data(),environ)!=0)return -1;
    const auto end=std::chrono::steady_clock::now()+10s;int status=0;
    while(std::chrono::steady_clock::now()<end) {
      send();executor->spin_some();
      if(waitpid(pid,&status,WNOHANG)==pid){pid=-1;executor->spin_some();return WIFEXITED(status)?WEXITSTATUS(status):-1;}
      std::this_thread::sleep_for(20ms);
    }
    kill(pid,SIGKILL);waitpid(pid,&status,0);pid=-1;return -2;
  }
  void send() {
    nav_msgs::msg::Odometry o;o.header.frame_id="odom";o.child_frame_id="base_footprint";o.header.stamp=node->now();o.pose.pose.orientation.w=1;o.pose.pose.position.x=odom_x;o.twist.twist.linear.x=speed;odom->publish(o);
    geometry_msgs::msg::TwistStamped c;c.header.stamp=node->now();cmd->publish(c);
    if(hold_pub){std_msgs::msg::Bool b;b.data=true;hold_pub->publish(b);}
    for(int i=0;i<(rear?2:1);++i){sensor_msgs::msg::LaserScan s;s.header.frame_id=i==0?"lidar_front_link":"lidar_rear_link";
      s.header.stamp=node->now()-rclcpp::Duration::from_seconds(.02);s.angle_min=0;s.angle_increment=2*std::acos(-1)/400;s.angle_max=(bad_metadata?400:399)*s.angle_increment;
      s.time_increment=-.1/399;s.scan_time=.1;s.range_min=.05;s.range_max=8;s.ranges.assign(400,1.0);scans[i]->publish(s);}
  }
  Json read(const fs::path &p){Json j;std::ifstream(p)>>j;return j;}
  void fitFiles(double age=0,double odom_at_capture=0) {
    const double t=node->now().seconds()-age;
    for(int i=0;i<2;++i){Json j={{"best",{1.0,2.0,.2}},{"map_metadata",{{"resolution",.01}}},
      {"source_snapshot",{{"capture_ros_time",t-1+i*.7},{"held",true},{"odom",{odom_at_capture,0,0}}}}};std::ofstream(dir/(i==0?"fit1.json":"fit2.json"))<<j;}
  }
  std::vector<std::string> initializeArgs(){return {"initialize",(dir/"fit1.json").string(),(dir/"fit2.json").string(),"--reviewed","front"};}
  fs::path dir;inline static int counter=0;pid_t pid=-1;bool rear=true,bad_metadata=false;double speed=0,odom_x=0;
  std::shared_ptr<rclcpp::Node> node,hold,amcl;std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd;
  std::array<rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr,2> scans;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr hold_pub;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster;
  geometry_msgs::msg::PoseWithCovarianceStamped received;int initial_count=0;
};
TEST_F(FullMapStartup, CapturesBothWithoutAmclPoseAndPublishesReviewedUncertainty) {
  ASSERT_EQ(run({"capture",(dir/"capture").string(),"1","2","0"}),0);
  auto first=read(dir/"capture/first.json"),second=read(dir/"capture/second.json");
  ASSERT_EQ(first["scans"].size(),2u);EXPECT_EQ(first["scans"][0].size(),400u);EXPECT_EQ(first["scans"][1].size(),400u);
  EXPECT_NEAR(first["scans"][0][0][0].get<double>(),2.2,1e-6);EXPECT_NEAR(first["scans"][1][0][0].get<double>(),1.9,1e-6);
  EXPECT_GE(second["capture_ros_time"].get<double>()-first["capture_ros_time"].get<double>(),.5);
  for(int i=0;i<2;++i){Json fit={{"best",{1.0,2.0,.2}},{"map_metadata",{{"resolution",.01}}},{"source_snapshot",i==0?first:second}};std::ofstream(dir/(i==0?"fit1.json":"fit2.json"))<<fit;}
  ASSERT_EQ(run(initializeArgs()),0);ASSERT_EQ(initial_count,1);
  EXPECT_EQ(received.header.frame_id,"map");EXPECT_DOUBLE_EQ(received.pose.pose.position.x,1);
  EXPECT_DOUBLE_EQ(received.pose.covariance[0],.01);EXPECT_DOUBLE_EQ(received.pose.covariance[7],.01);
  EXPECT_DOUBLE_EQ(received.pose.covariance[35],std::pow(5*std::acos(-1)/180,2));
}
TEST_F(FullMapStartup, RejectsMissingRearAndMalformedMetadata) {
  rear=false;EXPECT_NE(run({"capture",(dir/"missing").string(),"1","2","0"}),0);EXPECT_FALSE(fs::exists(dir/"missing"));
  rear=true;bad_metadata=true;EXPECT_NE(run({"capture",(dir/"bad").string(),"1","2","0"}),0);EXPECT_EQ(initial_count,0);
}
TEST_F(FullMapStartup, RejectsMotionAndHoldOwner) {
  speed=.01;EXPECT_NE(run({"capture",(dir/"motion").string(),"1","2","0"}),0);
  speed=0;hold_pub=node->create_publisher<std_msgs::msg::Bool>("/mission/hold",10);
  EXPECT_NE(run({"capture",(dir/"owner").string(),"1","2","0"}),0);EXPECT_EQ(initial_count,0);
}
TEST_F(FullMapStartup, RejectsHistoricalMovedAndRearAmclInitialization) {
  fitFiles(200);EXPECT_NE(run(initializeArgs()),0);
  fitFiles(0,.05);EXPECT_NE(run(initializeArgs()),0);
  fitFiles();amcl->set_parameter(rclcpp::Parameter("scan_topic","/lidar_rear/scan"));EXPECT_NE(run(initializeArgs()),0);
  EXPECT_EQ(initial_count,0);
}
