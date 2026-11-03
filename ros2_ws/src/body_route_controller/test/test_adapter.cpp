#include "body_route_controller/controller.hpp"
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
