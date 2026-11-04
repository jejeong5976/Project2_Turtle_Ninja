#include "body_route_controller/observation_layer.hpp"
#include <nav2_core/controller.hpp>
#include <nav2_core/controller_exceptions.hpp>
#include <nav2_controller/plugins/stopped_goal_checker.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <atomic>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
namespace body_route_controller {
class ObservationController : public nav2_core::Controller {
public:
  void configure(const nav2::LifecycleNode::WeakPtr &parent,std::string name,nav2::TransformBuffer::SharedPtr tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map) override {
    node_=parent;map_=map;layer_=observationLayer(map);auto n=parent.lock();
    const auto type=n->declare_or_get_parameter(name+".delegate_plugin",std::string("body_route_controller::BodyRouteController"));
    if(type!="body_route_controller::BodyRouteController"&&type!="nav2_mppi_controller::MPPIController")throw std::invalid_argument("unsupported observation delegate");
    loader_=std::make_unique<pluginlib::ClassLoader<nav2_core::Controller>>("nav2_core","nav2_core::Controller");
    delegate_=loader_->createSharedInstance(type);delegate_->configure(parent,name,tf,map);
    odom_=n->create_subscription<nav_msgs::msg::Odometry>("/odom",[this](nav_msgs::msg::Odometry::ConstSharedPtr m){
      auto n=node_.lock();if(!n)return;
      const auto &v=m->twist.twist;
      if(m->header.frame_id!="odom"||m->child_frame_id!="base_footprint"||!std::isfinite(v.linear.x)||!std::isfinite(v.linear.y)||!std::isfinite(v.angular.z)){
        odom_stamp_=-1;return;
      }
      odom_stamp_=rclcpp::Time(m->header.stamp).seconds();odom_received_=n->now().seconds();
    },rclcpp::SensorDataQoS());
    status_=n->create_publisher<std_msgs::msg::String>("~/observation_status");
  }
  void cleanup() override {delegate_->cleanup();delegate_.reset();loader_.reset();status_.reset();odom_.reset();layer_.reset();map_.reset();}
  void activate() override {delegate_->activate();status_->on_activate();}
  void deactivate() override {delegate_->deactivate();status_->on_deactivate();}
  void reset() override {delegate_->reset();}
  bool cancel() override {return delegate_->cancel();}
  void newPathReceived(const nav_msgs::msg::Path &p) override {delegate_->newPathReceived(p);}
  void setSpeedLimit(const double &s,const bool &p) override {delegate_->setSpeedLimit(s,p);}
  geometry_msgs::msg::TwistStamped computeVelocityCommands(const geometry_msgs::msg::PoseStamped &p,
    const geometry_msgs::msg::Twist &v,nav2_core::GoalChecker *g,const nav_msgs::msg::Path &path,
    const geometry_msgs::msg::PoseStamped &goal) override {
    auto n=node_.lock();auto now=n->now();geometry_msgs::msg::TwistStamped out;out.header.stamp=now;out.header.frame_id=map_->getBaseFrameID();
    std::string reason;
    const Pose pose{p.pose.position.x,p.pose.position.y,tf2::getYaw(p.pose.orientation)};
    const Velocity measured{v.linear.x,v.linear.y,v.angular.z};
    const double age=(now-rclcpp::Time(p.header.stamp)).seconds();
    if(odom_stamp_<0||now.seconds()<odom_stamp_||now.seconds()-odom_stamp_>.3||now.seconds()<odom_received_||now.seconds()-odom_received_>.3)reason="stale_odometry";
    else if(age<0||age>.3||p.header.frame_id!=map_->getGlobalFrameID()||!map_->isCurrent())reason="stale_pose_or_costmap";
    else {
      // Gate before the delegate as well: stale/missing data must not throw a
      // delegate failure or reset the entire navigation action while waiting.
      reason=layer_->gate(map_->getRobotFootprint(),pose,measured,{},now.seconds());
      if(reason.empty()){
        try{out=delegate_->computeVelocityCommands(p,v,g,path,goal);}
        catch(const nav2_core::NoValidControl &e){reason=std::string("route_obstacle_waiting_for_replan:")+e.what();}
        if(reason.empty())reason=layer_->gate(map_->getRobotFootprint(),pose,measured,
          {out.twist.linear.x,out.twist.linear.y,out.twist.angular.z},now.seconds());
      }
    }
    if(!reason.empty()){out.twist=geometry_msgs::msg::Twist{};delegate_->reset();}
    std_msgs::msg::String status;status.data=reason.empty()?"clear":"waiting reason="+reason;status_->publish(status);
    if(status.data!=last_){RCLCPP_INFO(n->get_logger(),"Observation gate: %s",status.data.c_str());last_=status.data;}
    return out;
  }
private:
  nav2::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_;std::atomic<double> odom_stamp_{-1},odom_received_{-1};
  nav2::LifecycleNode::WeakPtr node_;std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map_;std::shared_ptr<ObservationLayer> layer_;
  std::unique_ptr<pluginlib::ClassLoader<nav2_core::Controller>> loader_;nav2_core::Controller::Ptr delegate_;
  nav2::Publisher<std_msgs::msg::String>::SharedPtr status_;std::string last_;
};
// Nav2 can declare success without another controller invocation. Check the same
// evidence at the goal checker, so a provisional unknown goal cannot bypass it.
class ObservedGoalChecker : public nav2_controller::StoppedGoalChecker {
public:
  void initialize(const nav2::LifecycleNode::WeakPtr &p,const std::string &name,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> m) override {
    nav2_controller::StoppedGoalChecker::initialize(p,name,m);node_=p;map_=m;layer_=observationLayer(m);
  }
  bool isGoalReached(const geometry_msgs::msg::Pose &p,const geometry_msgs::msg::Pose &g,
    const geometry_msgs::msg::Twist &v,const nav_msgs::msg::Path &path) override {
    if(!nav2_controller::StoppedGoalChecker::isGoalReached(p,g,v,path))return false;
    const auto now=node_.lock()->now().seconds();
    return layer_->goalClear(map_->getRobotFootprint(),{p.position.x,p.position.y,tf2::getYaw(p.orientation)},now).empty()&&
      layer_->goalClear(map_->getRobotFootprint(),{g.position.x,g.position.y,tf2::getYaw(g.orientation)},now).empty();
  }
private:
  nav2::LifecycleNode::WeakPtr node_;std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map_;std::shared_ptr<ObservationLayer> layer_;
};
}
PLUGINLIB_EXPORT_CLASS(body_route_controller::ObservationController,nav2_core::Controller)
PLUGINLIB_EXPORT_CLASS(body_route_controller::ObservedGoalChecker,nav2_core::GoalChecker)
