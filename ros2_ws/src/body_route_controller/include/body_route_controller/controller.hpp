#pragma once
#include "body_route_controller/core.hpp"
#include <nav2_core/controller.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <mutex>

namespace body_route_controller
{
class BodyRouteController : public nav2_core::Controller
{
public:
  void configure(const nav2::LifecycleNode::WeakPtr &, std::string,
    nav2::TransformBuffer::SharedPtr, std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void newPathReceived(const nav_msgs::msg::Path &) override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped &, const geometry_msgs::msg::Twist &,
    nav2_core::GoalChecker *, const nav_msgs::msg::Path &,
    const geometry_msgs::msg::PoseStamped &) override;
  void setSpeedLimit(const double &, const bool &) override;
  bool cancel() override;
  void reset() override;

private:
  std::mutex mutex_;
  nav2::LifecycleNode::WeakPtr node_;
  nav2::TransformBuffer::SharedPtr tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
  nav2::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  nav2::Subscription<std_msgs::msg::Bool>::SharedPtr hold_sub_;
  nav2::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  Core core_;
  double speed_limit_ = 0.025;
  double feedback_timeout_ = 0.3, hold_timeout_ = 2.5, horizon_ = 2.0;
  double odom_received_ = -1.0, odom_stamped_ = -1.0, hold_received_ = -1.0;
  double previous_time_ = -1.0;
  bool hold_ = true;
  bool require_hold_ = true;
  bool active_ = false;
  std::string name_;
  void status(const Result &, const std::string & reason = "");
};
}
