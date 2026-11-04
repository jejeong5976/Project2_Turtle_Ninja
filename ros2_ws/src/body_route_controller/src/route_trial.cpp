#include "body_route_controller/supervision.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <thread>
#include <csignal>

using namespace std::chrono_literals;
using body_route_controller::Pose;
using body_route_controller::Point;
using body_route_controller::Velocity;
using body_route_controller::wrap;
using Plan = nav2_msgs::action::ComputePathToPose;
using Nav = nav2_msgs::action::NavigateToPose;
using Handle = rclcpp_action::ClientGoalHandle<Nav>;
using Clock = std::chrono::steady_clock;
namespace {
volatile std::sig_atomic_t interrupted=0;
void interrupt(int) {interrupted=1;}
double yaw(const geometry_msgs::msg::Quaternion & q)
{
  const double n=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
  if (!std::isfinite(n)||std::abs(n-1)>.01) {throw std::runtime_error("invalid quaternion");}
  return std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
}
double age(Clock::time_point t) {return std::chrono::duration<double>(Clock::now()-t).count();}
geometry_msgs::msg::PoseStamped stamped(const Pose & p, rclcpp::Time now)
{
  geometry_msgs::msg::PoseStamped m; m.header.frame_id="map"; m.header.stamp=now;
  m.pose.position.x=p.x; m.pose.position.y=p.y;
  m.pose.orientation.z=std::sin(p.yaw/2); m.pose.orientation.w=std::cos(p.yaw/2); return m;
}
std::vector<Pose> readGoals(const std::string & name)
{
  std::ifstream file(name); if(!file) {throw std::runtime_error("goals_file must name a readable map-frame x y yaw-radians file");}
  std::vector<Pose> goals; std::string line;
  while(std::getline(file,line)) {
    line=line.substr(0,line.find('#')); if(line.find_first_not_of(" \t\r")==std::string::npos) {continue;}
    std::istringstream row(line); Pose p; std::string extra;
    if(!(row>>p.x>>p.y>>p.yaw)||(row>>extra)||!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.yaw)) {
      throw std::runtime_error("invalid goal row: "+line);
    }
    goals.push_back(p);
  }
  if(goals.empty()||goals.size()>100) {throw std::runtime_error("provide 1 to 100 explicit goals");}
  return goals;
}
}

// An optional wheel-only trial client. Mission operation instead uses the
// existing mission_manager/Executor NavigateToPose + hold interface. Never run
// both owners. This client never publishes a velocity or starts a driver.
class Trial : public rclcpp::Node
{
public:
  Trial() : Node("body_route_trial"), tf_(get_clock()), listener_(tf_)
  {
    goals_=readGoals(declare_parameter<std::string>("goals_file",""));
    execute_=declare_parameter<bool>("execute",false);
    require_view_=declare_parameter<bool>("require_live_view",true);
    max_duration_=declare_parameter<double>("max_goal_duration",900.0);
    if(!std::isfinite(max_duration_)||max_duration_<30||max_duration_>3600) {throw std::runtime_error("max_goal_duration must be 30..3600 seconds");}
    status_=create_publisher<std_msgs::msg::String>("/trial/status",rclcpp::QoS(1).transient_local());
    odom_sub_=create_subscription<nav_msgs::msg::Odometry>("/odom",rclcpp::SensorDataQoS(),[this](nav_msgs::msg::Odometry::ConstSharedPtr m){
      try {
        if(m->header.frame_id!="odom"||m->child_frame_id!="base_footprint") {throw std::runtime_error("unexpected odometry frame");}
        const auto & p=m->pose.pose.position; const auto & v=m->twist.twist;
        Pose next{p.x,p.y,yaw(m->pose.pose.orientation)};
        if(moving_ && odom_received_!=Clock::time_point{}) {travel_+=std::hypot(next.x-odom_.x,next.y-odom_.y);}
        odom_=next; measured_={v.linear.x,v.linear.y,v.angular.z};
        odom_stamp_=rclcpp::Time(m->header.stamp).seconds(); odom_received_=Clock::now();
      } catch(const std::exception & e) {fault_=e.what();}
    });
    for(std::size_t i=0;i<2;++i) {
      scans_[i]=create_subscription<sensor_msgs::msg::LaserScan>(i==0?"/lidar_front/scan":"/lidar_rear/scan",rclcpp::SensorDataQoS(),
        [this,i](sensor_msgs::msg::LaserScan::ConstSharedPtr m){scan(i,*m);});
    }
    view_=create_subscription<std_msgs::msg::Bool>("/live_view/fresh",10,[this](std_msgs::msg::Bool::ConstSharedPtr m){view_fresh_=m->data;view_received_=Clock::now();});
    stop_=create_subscription<std_msgs::msg::Bool>("/live_view/stop",rclcpp::QoS(1).transient_local(),
      [this](std_msgs::msg::Bool::ConstSharedPtr m){if(m->data) {fault_="operator_stop";}});
    controller_=create_subscription<std_msgs::msg::String>("/controller_server/body_route_status",10,[this](std_msgs::msg::String::ConstSharedPtr m){
      std::istringstream row(m->data); std::string mode,heading;
      if(!(row>>mode>>heading)||mode.rfind("mode=",0)!=0||heading.rfind("heading=",0)!=0) {fault_="invalid_controller_status";return;}
      try {mode_=mode.substr(5); heading_=std::stod(heading.substr(8)); controller_received_=Clock::now();}
      catch(const std::exception &) {fault_="invalid_controller_heading";}
    });
    path_sub_=create_subscription<nav_msgs::msg::Path>("/plan",10,[this](nav_msgs::msg::Path::ConstSharedPtr m){
      if(!moving_||m->header.frame_id!="map"||m->poses.empty()) {return;}
      const auto & end=m->poses.back().pose.position;
      if(std::hypot(end.x-goal_.x,end.y-goal_.y)>.035) {fault_="unexpected_plan_destination";return;}
      std::vector<Point> path;
      for(const auto & p:m->poses) {
        if(!p.header.frame_id.empty()&&p.header.frame_id!="map") {fault_="mixed_plan_frames";return;}
        path.push_back({p.pose.position.x,p.pose.position.y});
      }
      path_=std::move(path);
    });
    nav_=rclcpp_action::create_client<Nav>(this,"/navigate_to_pose");
    planner_=rclcpp_action::create_client<Plan>(this,"/compute_path_to_pose");
  }
  void pump(double seconds=.02)
  {
    const auto end=Clock::now()+std::chrono::duration<double>(seconds);
    if(interrupted&&!closing_) {throw std::runtime_error("operator_interrupt");}
    do {executor_.spin_some();std::this_thread::sleep_for(5ms);} while(rclcpp::ok()&&Clock::now()<end);
  }
  void report(const std::string & s)
  {
    std_msgs::msg::String m; m.data=s; status_->publish(m); RCLCPP_INFO(get_logger(),"%s",s.c_str());
  }
  void holdNow()
  {
    moving_=false;
    if(hold_) {std_msgs::msg::Bool m; m.data=true;hold_->publish(m);}
  }
  void finish()
  {
    closing_=true; holdNow();
    if(handle_ && !terminal_) {try {(void)nav_->async_cancel_goal(handle_);} catch(const std::exception &) {}}
    if(rclcpp::ok()) {pump(2.0);}
  }
  bool run()
  {
    executor_.add_node(get_node_base_interface());
    pump(1);
    if(execute_) {
      if(count_publishers("/mission/hold")!=0) {throw std::runtime_error("another hold owner is present; do not run alongside mission_manager or another trial");}
      hold_=create_publisher<std_msgs::msg::Bool>("/mission/hold",10);
      timer_=create_wall_timer(50ms,[this](){
        if(count_publishers("/mission/hold")!=1) {fault_="competing_hold_owner";}
        if(!fault_.empty()) {moving_=false;}
        std_msgs::msg::Bool m;m.data=!moving_;hold_->publish(m);
      });
      holdNow(); pump(1);
    }
    for(std::size_t index=0;index<goals_.size();++index) {
      goal_=goals_[index];
      if(execute_) {
        auto start=Clock::now(),clear=Clock::now(); bool ready=false;
        while(rclcpp::ok()&&age(start)<30) {
          pump(); const auto reason=health(true);
          if(!fault_.empty()) {throw std::runtime_error(fault_);}
          if(!reason.empty()) {clear=Clock::now();}
          if(reason.empty()&&age(clear)>2) {ready=true;break;}
        }
        if(!ready) {throw std::runtime_error("stationary preflight: "+health(true));}
      }
      Plan::Goal request;request.goal=stamped(goal_,now());request.planner_id="GridBased";
      request.use_start=!execute_&&index>0;
      if(request.use_start) {request.start=stamped(goals_[index-1],now());}
      if(!planner_->wait_for_action_server(2s)) {throw std::runtime_error("planner unavailable");}
      auto pending=planner_->async_send_goal(request);
      if(!wait(pending,5)) {throw std::runtime_error("planner acceptance timeout");}
      auto ph=pending.get();if(!ph) {throw std::runtime_error("planner rejected goal");}
      auto result=planner_->async_get_result(ph);
      if(!wait(result,10)) {(void)planner_->async_cancel_goal(ph);throw std::runtime_error("planning timeout");}
      const auto planned=result.get();
      if(planned.code!=rclcpp_action::ResultCode::SUCCEEDED||planned.result->error_code!=0) {throw std::runtime_error("planning failed: "+planned.result->error_msg);}
      path_.clear(); double length=0;
      for(const auto & p:planned.result->path.poses) {
        Point q{p.pose.position.x,p.pose.position.y};
        if(!path_.empty()) {length+=std::hypot(q.x-path_.back().x,q.y-path_.back().y);}
        path_.push_back(q);
      }
      if(path_.empty()||!std::isfinite(length)) {throw std::runtime_error("invalid planned path");}
      report("goal "+std::to_string(index+1)+"/"+std::to_string(goals_.size())+" planned "+std::to_string(length)+" m"+(execute_?"; HELD":"; PREVIEW ONLY"));
      if(!execute_) {continue;}
      auto problem=health(true);if(!problem.empty()) {throw std::runtime_error(problem);}
      if(!nav_->wait_for_action_server(2s)) {throw std::runtime_error("navigator unavailable");}
      Nav::Goal target;target.pose=stamped(goal_,now()); terminal_=false;handle_.reset();
      rclcpp_action::Client<Nav>::SendGoalOptions options;
      options.goal_response_callback=[this](Handle::SharedPtr h){handle_=h;if(closing_&&h) {(void)nav_->async_cancel_goal(h);}};
      auto accepted=nav_->async_send_goal(target,options);
      if(!wait(accepted,5)) {throw std::runtime_error("navigator acceptance timeout");}
      handle_=accepted.get();if(!handle_) {throw std::runtime_error("navigator rejected goal");}
      auto done=nav_->async_get_result(handle_);
      guard_.reset();travel_=0;moving_=true;const auto started=Clock::now();
      report("goal "+std::to_string(index+1)+" ACTIVE; route-relative turns; deadline "+std::to_string(max_duration_)+" s");
      while(rclcpp::ok()&&done.wait_for(0s)!=std::future_status::ready) {
        pump(); problem=health(false);
        if(problem.empty()&&age(started)>1.0) {
          if(age(controller_received_)>.4) {problem="stale_controller_status";}
          else {problem=guard_.check(mapPose(),odom_,measured_,path_,mode_,heading_);}
        }
        if(problem.empty()&&travel_>2*length+.25) {problem="travel_budget_exceeded";}
        if(problem.empty()&&age(started)>max_duration_) {problem="goal_deadline";}
        if(!problem.empty()) {throw std::runtime_error(problem);}
      }
      holdNow();
      if(!rclcpp::ok()||done.wait_for(0s)!=std::future_status::ready) {throw std::runtime_error("interrupted");}
      terminal_=true; const auto answer=done.get();
      if(answer.code!=rclcpp_action::ResultCode::SUCCEEDED||answer.result->error_code!=0) {throw std::runtime_error("navigation failed: "+answer.result->error_msg);}
      pump(1); problem=health(true); if(!problem.empty()) {throw std::runtime_error("arrival: "+problem);}
      const auto pose=mapPose();
      if(std::hypot(pose.x-goal_.x,pose.y-goal_.y)>.035||std::abs(wrap(pose.yaw-goal_.yaw))>.055) {throw std::runtime_error("arrival pose outside goal tolerance");}
      report("goal "+std::to_string(index+1)+" complete and measured stopped; HELD");
    }
    report(execute_?"Route complete; HELD":"All goals planned; no hold release or navigation goal sent");return true;
  }
private:
  template<typename T> bool wait(T & future,double seconds)
  {
    const auto start=Clock::now();
    while(rclcpp::ok()&&future.wait_for(0s)!=std::future_status::ready&&age(start)<seconds) {pump();}
    return future.wait_for(0s)==std::future_status::ready;
  }
  Pose mapPose()
  {
    const auto t=tf_.lookupTransform("map","base_footprint",tf2::TimePointZero);
    const double dt=now().seconds()-rclcpp::Time(t.header.stamp).seconds();
    if(dt<-.05||dt>.35) {throw std::runtime_error("stale_map_pose");}
    if(!std::isfinite(t.transform.translation.x)||!std::isfinite(t.transform.translation.y)) {throw std::runtime_error("non_finite_map_pose");}
    return {t.transform.translation.x,t.transform.translation.y,yaw(t.transform.rotation)};
  }
  void scan(std::size_t i,const sensor_msgs::msg::LaserScan & scan)
  {
    valid_[i]=0;points_[i]=0;
    try {
      if(scan.header.frame_id!=(i==0?"lidar_front_link":"lidar_rear_link")) {return;}
      if(!std::isfinite(scan.angle_min)||!std::isfinite(scan.angle_increment)||scan.angle_increment<=0||
        !std::isfinite(scan.range_min)||!std::isfinite(scan.range_max)||scan.range_min<0||scan.range_max<=scan.range_min) {return;}
      const auto t=tf_.lookupTransform("base_footprint",scan.header.frame_id,tf2::TimePointZero);
      if(!std::isfinite(t.transform.translation.x)||!std::isfinite(t.transform.translation.y)) {return;}
      const double angle=yaw(t.transform.rotation);
      for(std::size_t j=0;j<scan.ranges.size();++j) {
        const double r=scan.ranges[j]; if(!std::isfinite(r)||r<scan.range_min||r>scan.range_max) {continue;}
        ++valid_[i];const double a=angle+scan.angle_min+j*scan.angle_increment;
        const double x=t.transform.translation.x+r*std::cos(a),y=t.transform.translation.y+r*std::sin(a);
        if(std::abs(x)<.19&&std::abs(y)<.19) {++points_[i];}
      }
      scan_stamp_[i]=rclcpp::Time(scan.header.stamp).seconds();scan_received_[i]=Clock::now();
    } catch(const tf2::TransformException &) {}
  }
  std::string health(bool stationary)
  {
    if(!fault_.empty()) {return fault_;}
    if(hold_&&count_publishers("/mission/hold")!=1) {return "competing_hold_owner";}
    auto current=[this](Clock::time_point received,double stamp) {return age(received)<=.35&&now().seconds()-stamp>=-.05&&now().seconds()-stamp<=.35;};
    if(!current(odom_received_,odom_stamp_)) {return "stale_odometry";}
    for(double n:{odom_.x,odom_.y,odom_.yaw,measured_.x,measured_.y,measured_.yaw}) {if(!std::isfinite(n)) {return "non_finite_odometry";}}
    for(std::size_t i=0;i<2;++i) {
      if(!current(scan_received_[i],scan_stamp_[i])) {return "stale_scan_"+std::to_string(i);}
      if(valid_[i]<50) {return "insufficient_scan_coverage";}
    }
    if(points_[0]+points_[1]>=2) {return "points_in_stop_polygon";}
    if(require_view_&&(!view_fresh_||age(view_received_)>.4)) {return "live_view_not_fresh";}
    if(std::hypot(measured_.x,measured_.y)>.04||std::abs(measured_.yaw)>.15) {return "measured_speed_limit";}
    if(stationary&&(std::hypot(measured_.x,measured_.y)>.003||std::abs(measured_.yaw)>.01)) {return "not_measured_stopped";}
    try {(void)mapPose();} catch(const std::exception & e) {return e.what();}
    return {};
  }
  std::vector<Pose> goals_;Pose goal_,odom_; Velocity measured_;
  std::vector<Point> path_;body_route_controller::TrackingGuard guard_;
  bool execute_=false,require_view_=true,moving_=false,closing_=false,terminal_=true,view_fresh_=false;
  double max_duration_=900,travel_=0,odom_stamp_=0,heading_=0;
  std::array<double,2> scan_stamp_{};std::array<int,2> valid_{},points_{};
  Clock::time_point odom_received_{},view_received_{},controller_received_{};
  std::array<Clock::time_point,2> scan_received_{};
  std::string fault_,mode_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  tf2_ros::Buffer tf_;tf2_ros::TransformListener listener_;
  Handle::SharedPtr handle_;rclcpp_action::Client<Nav>::SharedPtr nav_;rclcpp_action::Client<Plan>::SharedPtr planner_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr hold_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  std::array<rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr,2> scans_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr view_,stop_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr controller_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char ** argv)
{
  rclcpp::init(argc,argv,rclcpp::InitOptions(),rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT,interrupt);std::signal(SIGTERM,interrupt);std::shared_ptr<Trial> trial;int code=1;
  try {trial=std::make_shared<Trial>();code=trial->run()?0:1;}
  catch(const std::exception & e) {if(trial) {trial->holdNow();trial->report(std::string("STOPPED: ")+e.what());} else {RCLCPP_ERROR(rclcpp::get_logger("body_route_trial"),"%s",e.what());}}
  if(trial) {trial->finish();trial.reset();}
  rclcpp::shutdown();return code;
}
