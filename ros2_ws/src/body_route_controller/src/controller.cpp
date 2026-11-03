#include "body_route_controller/controller.hpp"
#include "body_route_controller/collision.hpp"
#include <nav2_core/controller_exceptions.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.hpp>
#include <cmath>
#include <sstream>

namespace body_route_controller
{
namespace
{
Pose planar(const geometry_msgs::msg::Pose & p)
{
  const auto & q = p.orientation;
  const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
  if (!std::isfinite(norm) || std::abs(norm - 1.0) > 0.01) {
    throw nav2_core::InvalidPath("Invalid pose quaternion");
  }
  return {p.position.x, p.position.y, tf2::getYaw(q)};
}
}

void BodyRouteController::configure(const nav2::LifecycleNode::WeakPtr & parent,
  std::string name, nav2::TransformBuffer::SharedPtr tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap)
{
  auto node = parent.lock();
  if (!node || !tf || !costmap) {throw nav2_core::ControllerException("Missing controller dependencies");}
  node_ = parent; name_ = std::move(name); tf_ = std::move(tf); costmap_ = std::move(costmap);
  auto number = [&](const std::string & key, double value) {
      return node->declare_or_get_parameter<double>(name_ + "." + key, value);
    };
  Settings cfg;
#define LOAD_SETTING(field) cfg.field = number(#field, cfg.field)
  LOAD_SETTING(max_speed); LOAD_SETTING(correction_speed); LOAD_SETTING(max_yaw_rate);
  LOAD_SETTING(heading_hold_rate); LOAD_SETTING(heading_rate_per_speed); LOAD_SETTING(heading_kp); LOAD_SETTING(linear_kp);
  LOAD_SETTING(linear_accel); LOAD_SETTING(angular_accel); LOAD_SETTING(align_enter);
  LOAD_SETTING(align_exit); LOAD_SETTING(lookahead); LOAD_SETTING(corner_tolerance);
  LOAD_SETTING(cross_track_enter); LOAD_SETTING(cross_track_exit); LOAD_SETTING(max_path_distance);
  LOAD_SETTING(max_correction_distance); LOAD_SETTING(max_correction_time);
  LOAD_SETTING(stopped_linear); LOAD_SETTING(stopped_angular); LOAD_SETTING(goal_xy);
  LOAD_SETTING(goal_yaw); LOAD_SETTING(max_dt);
#undef LOAD_SETTING
  core_ = Core(cfg); speed_limit_ = cfg.max_speed;
  feedback_timeout_ = number("feedback_timeout", 0.3);
  hold_timeout_ = number("hold_timeout", 2.5);
  horizon_ = number("collision_horizon", 2.0);
  if (!std::isfinite(feedback_timeout_) || feedback_timeout_ <= 0 ||
    !std::isfinite(hold_timeout_) || hold_timeout_ <= 0 || !std::isfinite(horizon_) || horizon_ < 1.0)
  {throw nav2_core::ControllerException("Invalid feedback/hold/collision time limits");}
  require_hold_ = node->declare_or_get_parameter<bool>(name_ + ".require_hold_signal", true);
  const auto odom_topic = node->declare_or_get_parameter<std::string>(name_ + ".odom_topic", "/odom");
  const auto hold_topic = node->declare_or_get_parameter<std::string>(name_ + ".hold_topic", "/mission/hold");
  odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(odom_topic,
    [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
      auto n = node_.lock(); if (!n) {return;}
      std::lock_guard<std::mutex> lock(mutex_);
      const auto & v = m->twist.twist;
      if (!std::isfinite(v.linear.x) || !std::isfinite(v.linear.y) || !std::isfinite(v.angular.z)) {
        odom_received_ = -1; return;
      }
      odom_received_ = n->now().seconds(); odom_stamped_ = rclcpp::Time(m->header.stamp).seconds();
    }, rclcpp::SensorDataQoS());
  hold_sub_ = node->create_subscription<std_msgs::msg::Bool>(hold_topic,
    [this](std_msgs::msg::Bool::ConstSharedPtr m) {
      auto n = node_.lock(); if (!n) {return;}
      std::lock_guard<std::mutex> lock(mutex_);
      hold_ = m->data; hold_received_ = n->now().seconds();
      if (hold_) {core_.reset(); previous_time_ = -1;}
    });
  status_pub_ = node->create_publisher<std_msgs::msg::String>("~/body_route_status");
  RCLCPP_INFO(node->get_logger(), "%s: body +X route heading; %.3f m/s, %.3f rad/s; hold required=%d",
    name_.c_str(), cfg.max_speed, cfg.max_yaw_rate, require_hold_);
}

void BodyRouteController::activate()
{
  std::lock_guard<std::mutex> lock(mutex_);
  active_ = true; core_.reset(); previous_time_ = -1;
  odom_received_ = hold_received_ = -1; hold_ = true;
  status_pub_->on_activate();
}
void BodyRouteController::deactivate()
{
  std::lock_guard<std::mutex> lock(mutex_);
  active_ = false; core_.reset(); previous_time_ = -1;
  status_pub_->on_deactivate();
}
void BodyRouteController::cleanup()
{
  odom_sub_.reset(); hold_sub_.reset(); status_pub_.reset(); costmap_.reset(); tf_.reset();
}
void BodyRouteController::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  core_.reset(); previous_time_ = -1;
}
bool BodyRouteController::cancel() {reset(); return true;}
void BodyRouteController::newPathReceived(const nav_msgs::msg::Path & path)
{
  if (path.poses.empty()) {throw nav2_core::InvalidPath("Empty route");}
  // Replanning is periodic: preserve phase hysteresis. Nav2 resets at task end.
}
void BodyRouteController::setSpeedLimit(const double & value, const bool & percentage)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!std::isfinite(value) || value < 0.0) {speed_limit_ = 0.0; return;}
  // Nav2 reserves zero for NO_SPEED_LIMIT. A commanded stop uses hold/cancel.
  speed_limit_ = value == 0.0 ? core_.settings().max_speed :
    std::min(core_.settings().max_speed, percentage ? core_.settings().max_speed * value / 100.0 : value);
}
void BodyRouteController::status(const Result & result, const std::string & reason)
{
  if (!status_pub_) {return;}
  std::ostringstream out;
  out << "mode=" << result.mode << " heading=" << result.heading << " error=" << result.heading_error
      << " vx=" << result.command.x << " vy=" << result.command.y << " wz=" << result.command.yaw;
  if (!reason.empty()) {out << " reason=" << reason;}
  std_msgs::msg::String message; message.data = out.str(); status_pub_->publish(message);
}

geometry_msgs::msg::TwistStamped BodyRouteController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker *, const nav_msgs::msg::Path & plan,
  const geometry_msgs::msg::PoseStamped & goal)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto node = node_.lock();
  if (!node || !active_) {throw nav2_core::ControllerException("Body-route controller inactive");}
  const auto now = node->now();
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = now; cmd.header.frame_id = costmap_->getBaseFrameID();
  auto reject = [&](const std::string & reason) {
      core_.reset(); previous_time_ = -1;
      status({{}, "blocked", 0.0, 0.0}, reason);
      throw nav2_core::NoValidControl(reason);
    };
  if (require_hold_ && (hold_ || hold_received_ < 0.0 || now.seconds() < hold_received_ ||
    now.seconds() - hold_received_ > hold_timeout_))
  {
    core_.reset(); previous_time_ = -1; status({{}, "mission_hold", 0.0, 0.0}); return cmd;
  }
  if (!fresh(now.seconds(), odom_received_, odom_stamped_, feedback_timeout_)) {reject("stale_odometry");}
  if (!fresh(now.seconds(), now.seconds(), rclcpp::Time(pose.header.stamp).seconds(), feedback_timeout_)) {
    reject("stale_pose");
  }
  if (!costmap_->isCurrent()) {reject("stale_costmap");}
  const auto frame = costmap_->getGlobalFrameID();
  if (plan.header.frame_id != frame || pose.header.frame_id != frame || goal.header.frame_id != frame) {
    reject("path_pose_goal_must_be_in_costmap_frame");
  }
  std::vector<Point> points;
  for (const auto & p : plan.poses) {
    if (!p.header.frame_id.empty() && p.header.frame_id != frame) {reject("mixed_path_frames");}
    points.push_back({p.pose.position.x, p.pose.position.y});
  }
  const double dt = previous_time_ < 0.0 ? 0.05 : now.seconds() - previous_time_;
  previous_time_ = now.seconds();
  Result result;
  Pose robot;
  const Velocity measured{velocity.linear.x, velocity.linear.y, velocity.angular.z};
  try {
    robot = planar(pose.pose);
    result = core_.step(robot, measured, points, planar(goal.pose), dt, speed_limit_);
  } catch (const std::exception & e) {reject(e.what());}
  auto * map = costmap_->getCostmap();
  const auto footprint = costmap_->getRobotFootprint();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> map_lock(*map->getMutex());
  const auto & cfg = core_.settings();
  // Check actual motion and its braking tail as well as the requested command.
  // These bounds assume the matched slow actuator profile; collision_monitor
  // remains a separate final guard on the unchanged velocity chain.
  const double braking_time = std::max(std::hypot(measured.x, measured.y) / cfg.linear_accel,
      std::abs(measured.yaw) / cfg.angular_accel);
  if (!motionSafe(*map, footprint, robot, measured, feedback_timeout_ + braking_time) ||
    !motionSafe(*map, footprint, robot, result.command, horizon_)) {reject("footprint_or_stopping_sweep_blocked");}
  if ((result.mode == "route_turn" || result.mode == "final_turn") &&
    !turnSafe(*map, footprint, robot, result.heading)) {reject("full_rotation_sweep_blocked");}
  cmd.twist.linear.x = result.command.x;
  cmd.twist.linear.y = result.command.y;
  cmd.twist.angular.z = result.command.yaw;
  status(result);
  return cmd;
}
}
PLUGINLIB_EXPORT_CLASS(body_route_controller::BodyRouteController, nav2_core::Controller)
