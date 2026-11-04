#include "body_route_controller/controller.hpp"
#include <nav2_costmap_2d/inflation_layer.hpp>
#include <nav2_core/controller_exceptions.hpp>
#include <gtest/gtest.h>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

// In-process ROS graph with an empty synthetic costmap. No robot, hardware
// driver or velocity publisher is created. CTest isolates this graph on domain 231.
TEST(Adapter, HoldFeedbackCommandCollisionAndStaleness)
{
  rclcpp::init(0, nullptr);
  auto parent = std::make_shared<nav2::LifecycleNode>("body_route_adapter_test");
  auto feed = std::make_shared<rclcpp::Node>("body_route_test_feedback");
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("plugins", std::vector<std::string>{}),
    rclcpp::Parameter("global_frame", "odom"),
    rclcpp::Parameter("robot_base_frame", "base_footprint"),
    rclcpp::Parameter("resolution", .01),
    rclcpp::Parameter("width", 4), rclcpp::Parameter("height", 4),
    rclcpp::Parameter("origin_x", -2.0), rclcpp::Parameter("origin_y", -2.0),
    rclcpp::Parameter("footprint", "[[-0.18,-0.18],[0.18,-0.18],[0.18,0.18],[-0.18,0.18]]"),
    rclcpp::Parameter("footprint_padding", 0.0)});
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>(options);
  ASSERT_EQ(costmap->on_configure(rclcpp_lifecycle::State()), nav2::CallbackReturn::SUCCESS);
  auto * grid = costmap->getCostmap();
  grid->resizeMap(400, 400, .01, -2, -2);
  grid->resetMap(0, 0, 400, 400);
  auto tf = std::make_shared<nav2::TransformBuffer>(parent->get_clock());
  body_route_controller::BodyRouteController controller;
  controller.configure(parent, "FollowPath", tf, costmap);
  controller.activate();
  auto odom_pub = feed->create_publisher<nav_msgs::msg::Odometry>("/odom", rclcpp::SensorDataQoS());
  auto hold_pub = feed->create_publisher<std_msgs::msg::Bool>("/mission/hold", 10);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(parent->get_node_base_interface()); executor.add_node(feed);
  geometry_msgs::msg::PoseStamped pose, goal;
  pose.header.frame_id = goal.header.frame_id = "odom";
  pose.pose.orientation.w = goal.pose.orientation.w = 1.0;
  goal.pose.position.x = 1.0;
  nav_msgs::msg::Path path; path.header.frame_id = "odom";
  path.poses = {pose, goal};
  auto compute = [&]() {
      pose.header.stamp = parent->now();
      return controller.computeVelocityCommands(
        pose, geometry_msgs::msg::Twist(), nullptr, path, goal);
    };
  EXPECT_DOUBLE_EQ(compute().twist.linear.x, 0.0);  // Held before any heartbeat.
  auto feedback = [&](bool hold) {
      for (int i = 0; i < 15; ++i) {
        nav_msgs::msg::Odometry odom; odom.header.stamp = feed->now();
        odom.header.frame_id = "odom"; odom.child_frame_id = "base_footprint";
        odom_pub->publish(odom);
        std_msgs::msg::Bool h; h.data = hold; hold_pub->publish(h);
        executor.spin_some(); std::this_thread::sleep_for(10ms);
      }
    };
  feedback(false);
  EXPECT_DOUBLE_EQ(compute().twist.linear.x, 0.0);  // Alignment transition.
  std::this_thread::sleep_for(50ms);
  EXPECT_GT(compute().twist.linear.x, 0.0);
  controller.setSpeedLimit(.001, false);
  std::this_thread::sleep_for(50ms);
  EXPECT_LE(compute().twist.linear.x, .001);
  controller.setSpeedLimit(0.0, false);  // Nav2 NO_SPEED_LIMIT.
  unsigned int x, y; ASSERT_TRUE(grid->worldToMap(.01, .01, x, y)); grid->setCost(x, y, 254);
  EXPECT_THROW(compute(), nav2_core::NoValidControl);
  grid->setCost(x, y, 0);
  feedback(true);
  EXPECT_DOUBLE_EQ(compute().twist.linear.x, 0.0);
  feedback(false);
  std::this_thread::sleep_for(350ms);
  EXPECT_THROW(compute(), nav2_core::NoValidControl);
  controller.deactivate(); controller.cleanup();
  executor.remove_node(parent->get_node_base_interface()); executor.remove_node(feed);
  costmap->on_cleanup(rclcpp_lifecycle::State());
  costmap.reset(); parent.reset(); feed.reset(); tf.reset();
  rclcpp::shutdown();
}

// Exercise the installed Nav2 inflation implementation: an unknown cell near
// a wall must remain collision-blocking, not become traversable cost 253.
TEST(Adapter, InflationKeepsUnknownCellsCollisionBlocking)
{
  rclcpp::init(0,nullptr);
  {
    auto node=std::make_shared<nav2::LifecycleNode>("unknown_inflation_test");
    for(bool conservative:{false,true}) {
      const std::string name=conservative?"conservative":"baseline";
      node->declare_parameter(name+".inflate_around_unknown",conservative);
      node->declare_parameter(name+".inflation_radius",.26);
      nav2_costmap_2d::LayeredCostmap layered("map",false,true);
      layered.resizeMap(100,100,.01,0,0);
      std::vector<geometry_msgs::msg::Point> footprint;
      for(auto q:std::vector<std::pair<double,double>>{{-.205,-.205},{.205,-.205},{.205,.205},{-.205,.205}}) {
        geometry_msgs::msg::Point p;p.x=q.first;p.y=q.second;footprint.push_back(p);
      }
      layered.setFootprint(footprint);
      nav2_costmap_2d::InflationLayer layer;
      layer.initialize(&layered,name,nullptr,node,node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive));
      auto *map=layered.getCostmap();std::fill_n(map->getCharMap(),10000,0);
      map->setCost(40,50,254);map->setCost(55,50,255);
      layer.updateCosts(*map,0,0,100,100);
      if(conservative) {EXPECT_GE(map->getCost(55,50),254);}
      else {EXPECT_EQ(map->getCost(55,50),253);}
    }
  }
  rclcpp::shutdown();
}

#include "body_route_controller/observation_layer.hpp"
#include "unknown_fixture.hpp"
#include <tf2_ros/buffer.hpp>
TEST(Adapter, RawIdentityAndTimestampedTransformedRaysGateExecution)
{
  rclcpp::init(0,nullptr);
  {
    auto node=std::make_shared<nav2::LifecycleNode>("observation_adapter_test");
    node->declare_parameter("observations.unknown_policy",std::string("observe-before-enter"));
    nav2::TransformBuffer tf(node->get_clock());
    auto transform=[&](const std::string &parent,const std::string &child,double x,double y){
      geometry_msgs::msg::TransformStamped t;t.header.frame_id=parent;t.child_frame_id=child;t.header.stamp=node->now();
      t.transform.translation.x=x;t.transform.translation.y=y;t.transform.rotation.w=1;tf.setTransform(t,"offline_test",true);
    };
    transform("map","base_footprint",.8,1.4);transform("base_footprint","lidar_front_link",.1145,0);transform("base_footprint","lidar_rear_link",-.1145,0);
    nav2_costmap_2d::LayeredCostmap layered("map",false,true);layered.resizeMap(220,140,.02,0,0);layered.setFootprint(unknown_fixture::body());
    body_route_controller::ObservationLayer layer;
    layer.initialize(&layered,"observations",&tf,node,node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive));
    auto publisher=node->create_publisher<nav_msgs::msg::OccupancyGrid>("/map",rclcpp::QoS(1).transient_local());publisher->on_activate();
    std::array<nav2::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr,2> scans;
    for(int i=0;i<2;++i){scans[i]=node->create_publisher<sensor_msgs::msg::LaserScan>(i==0?"/lidar_front/scan":"/lidar_rear/scan",rclcpp::SensorDataQoS());scans[i]->on_activate();}
    auto truth=unknown_fixture::world();publisher->publish(unknown_fixture::partial(truth));
    rclcpp::executors::SingleThreadedExecutor exec;exec.add_node(node->get_node_base_interface());
    auto pump=[&](){for(int i=0;i<20;++i){exec.spin_some();std::this_thread::sleep_for(5ms);}};pump();
    auto raw=layer.identityMap();unsigned x,y;ASSERT_TRUE(raw.worldToMap(1.5,1.4,x,y));EXPECT_EQ(raw.getCost(x,y),255);
    auto *master=layered.getCostmap();std::fill_n(master->getCharMap(),220*140,255);
    layer.updateCosts(*master,0,0,220,140);EXPECT_EQ(master->getCost(x,y),0);EXPECT_EQ(layer.identityMap().getCost(x,y),255);
    EXPECT_NE(layer.gate(unknown_fixture::body(),{.8,1.4,0},{},{.08,0,0},node->now().seconds()),"");
    auto send=[&](double age){for(int i=0;i<2;++i){sensor_msgs::msg::LaserScan m;m.header.frame_id=i==0?"lidar_front_link":"lidar_rear_link";
        m.header.stamp=node->now()-rclcpp::Duration::from_seconds(age);m.angle_min=-unknown_fixture::pi;m.angle_increment=2*unknown_fixture::pi/720;m.range_min=.05;m.range_max=.65;
        for(auto r:unknown_fixture::scan(truth,{.8,1.4,0},i,rclcpp::Time(m.header.stamp).seconds()))m.ranges.push_back(r.range);
        scans[i]->publish(m);
      }pump();};
    send(0);EXPECT_EQ(layer.goalClear(unknown_fixture::body(),{1.28,1.4,0},node->now().seconds()),"");
    send(1);EXPECT_NE(layer.gate(unknown_fixture::body(),{.8,1.4,0},{},{.08,0,0},node->now().seconds()),"");
    send(0);tf.clear();send(0);
    EXPECT_NE(layer.gate(unknown_fixture::body(),{.8,1.4,0},{},{.08,0,0},node->now().seconds()).find("transform"),std::string::npos);
    exec.remove_node(node->get_node_base_interface());
  }
  rclcpp::shutdown();
}
