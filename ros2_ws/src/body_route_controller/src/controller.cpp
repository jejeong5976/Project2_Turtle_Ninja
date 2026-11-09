#include "body_route_controller/controller.hpp"
#include "body_route_controller/collision.hpp"
#include <nav2_core/controller_exceptions.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.hpp>
#include <cmath>
#include <chrono>
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
  cfg.drive_style = node->declare_or_get_parameter<std::string>(name_ + ".drive_style", cfg.drive_style);
  cfg.heading_policy = node->declare_or_get_parameter<std::string>(name_ + ".heading_policy", cfg.heading_policy);
  cfg.heading_preview_time = number("heading_preview_time", cfg.heading_preview_time);
  cfg.max_travel_angle = number("max_travel_angle", cfg.max_travel_angle);
#define LOAD_SETTING(field) cfg.field = number(#field, cfg.field)
  LOAD_SETTING(max_speed); LOAD_SETTING(correction_speed); LOAD_SETTING(max_yaw_rate);
  LOAD_SETTING(heading_hold_rate); LOAD_SETTING(heading_rate_per_speed); LOAD_SETTING(heading_kp); LOAD_SETTING(linear_kp);
  LOAD_SETTING(linear_accel); LOAD_SETTING(angular_accel); LOAD_SETTING(align_enter);
  LOAD_SETTING(align_exit); LOAD_SETTING(lookahead); LOAD_SETTING(arrival_lookahead); LOAD_SETTING(corner_tolerance);
  LOAD_SETTING(path_simplify_tolerance);
  LOAD_SETTING(cross_track_enter); LOAD_SETTING(cross_track_exit); LOAD_SETTING(max_path_distance);
  LOAD_SETTING(max_correction_distance); LOAD_SETTING(max_correction_time);
  LOAD_SETTING(stopped_linear); LOAD_SETTING(stopped_angular); LOAD_SETTING(goal_xy);
  LOAD_SETTING(goal_yaw); LOAD_SETTING(max_dt);
#undef LOAD_SETTING
  Settings corridor = cfg; corridor.round_corners = false; corridor_core_ = Core(corridor);
  core_ = Core(cfg); speed_limit_ = cfg.max_speed;
  PassageSettings passage;
  passage.enabled = node->declare_or_get_parameter<bool>(name_ + ".prepare_passages", false);
  passage.position_allowance = number("passage_position_allowance", passage.position_allowance);
  passage.yaw_allowance = number("passage_yaw_allowance", passage.yaw_allowance);
  passage.response_delay = number("passage_response_delay", passage.response_delay);
  passage_ = PreparedPassage(cfg, passage);
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
      if (hold_) {resetMotion(false); previous_time_ = -1;}
    });
  status_pub_ = node->create_publisher<std_msgs::msg::String>("~/body_route_status");
  RCLCPP_INFO(node->get_logger(), "%s: %s; %.3f m/s, %.3f rad/s; travel cone %.1f deg; hold required=%d",
    name_.c_str(), cfg.drive_style.c_str(), cfg.max_speed, cfg.max_yaw_rate,
    cfg.max_travel_angle * 180 / std::acos(-1.0), require_hold_);
}

void BodyRouteController::activate()
{
  std::lock_guard<std::mutex> lock(mutex_);
  active_ = true; resetMotion(); previous_time_ = -1;
  odom_received_ = hold_received_ = -1; hold_ = true;
  status_pub_->on_activate();
}
void BodyRouteController::deactivate()
{
  std::lock_guard<std::mutex> lock(mutex_);
  active_ = false; resetMotion(); previous_time_ = -1;
  status_pub_->on_deactivate();
}
void BodyRouteController::cleanup()
{
  odom_sub_.reset(); hold_sub_.reset(); status_pub_.reset(); costmap_.reset(); tf_.reset();
}
void BodyRouteController::resetMotion(bool forget_passage)
{
  core_.reset(); corridor_core_.reset(); corridor_ = false;
  if (forget_passage) {passage_.reset();} else {passage_.pause();}
}
void BodyRouteController::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  resetMotion(); previous_time_ = -1;
}
void BodyRouteController::pauseMotion()
{
  std::lock_guard<std::mutex> lock(mutex_);
  resetMotion(false); previous_time_ = -1;
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
  const auto compute_started=std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(mutex_);
  auto node = node_.lock();
  if (!node || !active_) {throw nav2_core::ControllerException("Body-route controller inactive");}
  const auto now = node->now();
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = now; cmd.header.frame_id = costmap_->getBaseFrameID();
  auto reject = [&](const std::string & reason) {
      resetMotion(false); previous_time_ = -1;
      status({{}, "blocked", 0.0, 0.0}, reason);
      throw nav2_core::NoValidControl(reason);
    };
  if (require_hold_ && (hold_ || hold_received_ < 0.0 || now.seconds() < hold_received_ ||
    now.seconds() - hold_received_ > hold_timeout_))
  {
    resetMotion(false); previous_time_ = -1; status({{}, "mission_hold", 0.0, 0.0}); return cmd;
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
  Core before = corridor_ ? corridor_core_ : core_;
  Pose robot, destination;
  const Velocity measured{velocity.linear.x, velocity.linear.y, velocity.angular.z};
  try {
    robot = planar(pose.pose);
    destination = planar(goal.pose);
    if (corridor_ && std::hypot(robot.x-corridor_start_.x,robot.y-corridor_start_.y)>.4 &&
      std::abs(measured.yaw)<core_.settings().stopped_angular) {
      corridor_=false; core_.reset(); before=core_;
    }
    result = corridor_ ? corridor_core_.step(robot, measured, points, destination, dt, speed_limit_) :
      core_.step(robot, measured, points, destination, dt, speed_limit_);
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
  while (!pending_.empty() && (now.seconds() - pending_.front().first > .5 ||
    now.seconds() < pending_.front().first)) {pending_.pop_front();}
  auto unsafe = [&](const Result & candidate) -> std::string {
      if (!motionSafe(*map, footprint, robot, measured, feedback_timeout_ + braking_time) ||
        !motionSafe(*map, footprint, robot, candidate.command, horizon_)) {
        return "footprint_or_stopping_sweep_blocked";
      }
      if (cfg.drive_style == "continuous") {
        std::vector<Velocity> velocities{measured, candidate.command};
        for (const auto & item : pending_) {velocities.push_back(item.second);}
        const auto bounds = stoppingEnvelope(footprint, robot, velocities, .5, cfg.linear_accel, cfg.angular_accel);
        if (bounds.polygon.empty() || !footprintSafe(*map, bounds.polygon, {})) {
          return "continuous_stopping_envelope_blocked";
        }
      }
      if ((candidate.mode == "route_turn" || candidate.mode == "final_turn") &&
        !turnSafe(*map, footprint, robot, candidate.heading)) {return "full_rotation_sweep_blocked";}
      return {};
    };
  std::optional<Result> prepared;
  try {
    std::vector<Velocity> history;
    for (const auto & item : pending_) {history.push_back(item.second);}
    prepared = passage_.step(*map, footprint, robot, measured, points, destination,
      dt, speed_limit_, history);
    if (prepared) {result = *prepared;}
  } catch (const std::exception & e) {reject(e.what());}
  auto reason = unsafe(result);
  if (prepared && !reason.empty()) {
    passage_.pause(); core_.reset(); corridor_core_.reset();
    status({{}, "clearance_wait", robot.yaw, 0}, reason); return cmd;
  }
  if (!prepared && !reason.empty() && cfg.drive_style == "continuous") {
    if (!corridor_) {
      // Rounding can be impossible beside a real wall or an occupied scan
      // return. Follow the planned corner with stop/turn/settle phases, but
      // retain diagonal corrections and the travel cone during translation.
      corridor_=true; corridor_start_={robot.x,robot.y}; corridor_core_.reset();
      before=corridor_core_;
      result=corridor_core_.step(robot,measured,points,destination,dt,speed_limit_);
      reason=unsafe(result);
    }
    // Keep the same path target while reducing speed. Do not alternate pursuit
    // targets and steering poses on each tick in constrained clearance.
    if (!reason.empty()) for (double speed : {.5,.25,.125}) {
      Core candidate=before;
      const auto proposal=candidate.step(robot,measured,points,destination,dt,speed_limit_*speed);
      if (unsafe(proposal).empty()) {corridor_core_=candidate;result=proposal;reason.clear();break;}
    }
    if (!reason.empty()) {
      core_.reset(); corridor_core_.reset(); status({{}, "clearance_wait", robot.yaw, 0}, reason);
      return cmd; // Zero while braking; periodic replanning continues, progress timeout remains active.
    }
  }
  if (!reason.empty()) {reject(reason);}
  // The prepared layer must not publish a late proposal based on an old pose.
  // This is an additional 20 Hz deadline, never an extension of feedback/scan
  // freshness. Existing recent commands remain in the stopping history.
  const double compute_seconds=std::chrono::duration<double>(
    std::chrono::steady_clock::now()-compute_started).count();
  if (passage_.enabled() && compute_seconds > .05) {
    passage_.pause(); core_.reset(); corridor_core_.reset(); previous_time_=-1;
    status({{}, "passage_wait", robot.yaw, 0}, "prepared_compute_deadline_"+
      std::to_string(compute_seconds));
    return cmd;
  }
  if (cfg.drive_style == "continuous") {pending_.emplace_back(now.seconds(), result.command);}
  cmd.twist.linear.x = result.command.x;
  cmd.twist.linear.y = result.command.y;
  cmd.twist.angular.z = result.command.yaw;
  if (prepared) {core_.reset(); corridor_core_.reset(); corridor_ = false;}
  status(result, prepared ? passage_.reason() : corridor_ ? "following_exact_corner_for_clearance" : "");
  return cmd;
}
}
PLUGINLIB_EXPORT_CLASS(body_route_controller::BodyRouteController, nav2_core::Controller)
