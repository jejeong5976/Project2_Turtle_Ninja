#include "layout_fixture_io.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>
#include <gtest/gtest.h>
#include <sys/wait.h>
#include <unistd.h>
#include <thread>

namespace fs=std::filesystem;
using namespace layout_test;
using namespace std::chrono_literals;
TEST(LayoutCapture, RecordsOnlySettledDistinctViewsFromBothSensors) {
  char pattern[]="/tmp/layout-capture-XXXXXX";const char *name=mkdtemp(pattern);ASSERT_NE(name,nullptr);const fs::path root=name;
  fixture(root/"input");
  const pid_t child=fork();ASSERT_GE(child,0);
  if(child==0) {
    const std::string layout="layout_file:="+(root/"input/template.yaml").string(),output="output_directory:="+(root/"capture").string();
    execl(CAPTURE_EXECUTABLE,CAPTURE_EXECUTABLE,"--ros-args","-p","fixed_start_verified:=true","-p",layout.c_str(),"-p",output.c_str(),"-p","max_keyframes:=2",nullptr);_exit(127);
  }
  struct Child {pid_t pid;~Child(){if(pid>0) {kill(pid,SIGTERM);int status;waitpid(pid,&status,0);}}} cleanup{child};
  rclcpp::init(0,nullptr);auto node=std::make_shared<rclcpp::Node>("layout_capture_fixture");
  auto odom=node->create_publisher<nav_msgs::msg::Odometry>("/odom",rclcpp::SensorDataQoS());
  auto front=node->create_publisher<sensor_msgs::msg::LaserScan>("/lidar_front/scan",rclcpp::SensorDataQoS());
  auto rear=node->create_publisher<sensor_msgs::msg::LaserScan>("/lidar_rear/scan",rclcpp::SensorDataQoS());
  tf2_ros::StaticTransformBroadcaster tf(tf2_ros::StaticTransformBroadcaster::RequiredInterfaces(
    node->get_node_parameters_interface(),node->get_node_topics_interface()));
  rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(node);
  std::vector<geometry_msgs::msg::TransformStamped> transforms;
  for(int i=0;i<2;++i) {geometry_msgs::msg::TransformStamped t;t.header.stamp=node->now();t.header.frame_id="base_footprint";
    t.child_frame_id=i==0?"test_front":"test_rear";t.transform.translation.x=i==0?.08:-.08;
    t.transform.rotation.z=i==0?0:1;t.transform.rotation.w=i==0?1:0;transforms.push_back(t);
  }tf.sendTransform(transforms);
  const auto began=std::chrono::steady_clock::now();bool done=false;int status=0;
  while(std::chrono::steady_clock::now()-began<10s) {
    const double elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count();
    const Pose pose{elapsed<2.5?-.5:-.4,0,0};nav_msgs::msg::Odometry m;m.header.stamp=node->now();m.header.frame_id="odom";m.child_frame_id="base_footprint";
    m.pose.pose.position.x=pose.x;m.pose.pose.orientation.w=1;odom->publish(m);
    for(int i=0;i<2;++i) {
      const auto s=scan(nominal(),pose,{i==0?.08:-.08,0,i==0?0:pi},node->now().seconds(),i);
      sensor_msgs::msg::LaserScan msg;msg.header.stamp=m.header.stamp;msg.header.frame_id=i==0?"test_front":"test_rear";
      msg.angle_min=s.angle_min;msg.angle_increment=s.angle_increment;msg.angle_max=s.angle_min+(s.ranges.size()-1)*s.angle_increment;
      msg.range_min=s.range_min;msg.range_max=s.range_max;for(double r:s.ranges) {msg.ranges.push_back(r);}
      if(i==0) {front->publish(msg);}else {rear->publish(msg);}
    }
    executor.spin_some();if(waitpid(child,&status,WNOHANG)==child) {done=true;cleanup.pid=-1;break;}std::this_thread::sleep_for(50ms);
  }
  rclcpp::shutdown();EXPECT_TRUE(done);EXPECT_TRUE(WIFEXITED(status));EXPECT_EQ(WEXITSTATUS(status),0);
  ASSERT_TRUE(fs::exists(root/"capture/survey_2.yaml"));const auto d=YAML::LoadFile(root/"capture/survey_2.yaml");
  ASSERT_EQ(d["frames"].size(),2u);
  EXPECT_GE(d["frames"][0]["settled_seconds"].as<double>(),1.);
  EXPECT_NEAR(d["frames"][1]["odom"][0].as<double>()-d["frames"][0]["odom"][0].as<double>(),.1,.001);
  EXPECT_EQ(d["frames"][0]["scans"].size(),2u);
  EXPECT_NEAR(d["frames"][0]["scans"][1]["sensor_pose"][2].as<double>(),pi,1e-6);
}
