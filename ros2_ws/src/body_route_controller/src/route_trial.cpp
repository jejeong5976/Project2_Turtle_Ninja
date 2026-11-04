#include "body_route_controller/supervision.hpp"
#include "body_route_controller/profile.hpp"
#include <rclcpp/parameter_client.hpp>
#include <nav2_msgs/action/follow_path.hpp>
#include <functional>
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
using Follow = nav2_msgs::action::FollowPath;
using body_route_controller::TrialSettings;
using body_route_controller::fingerprint;
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
std::vector<Pose> readGoals(const std::string & name, std::size_t maximum=100)
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
  if(goals.empty()||goals.size()>maximum) {throw std::runtime_error("empty or oversized route file");}
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
    const auto goals_file=declare_parameter<std::string>("goals_file","");
    const auto path_file=declare_parameter<std::string>("path_file","");
    save_path_=declare_parameter<std::string>("save_path_file","");
    if(goals_file.empty()==path_file.empty()) {throw std::runtime_error("provide exactly one of goals_file or path_file");}
    fixed_path_=!path_file.empty();
    if(fixed_path_) {
      fixed_poses_=readGoals(path_file,100000);
      if(fixed_poses_.size()<2) {throw std::runtime_error("path_file requires at least two poses");}
      goals_={fixed_poses_.back()};
    } else {goals_=readGoals(goals_file);}
    input_id_=fingerprint(body_route_controller::readText(fixed_path_?path_file:goals_file));
    settings_.mode=declare_parameter<std::string>("controller_mode","custom");
    settings_.speed=declare_parameter<double>("speed",.025);
    settings_.yaw_rate=declare_parameter<double>("yaw_rate",.1);
    settings_.linear_accel=declare_parameter<double>("linear_accel",.05);
    settings_.angular_accel=declare_parameter<double>("angular_accel",.2);
    settings_.correction_speed=declare_parameter<double>("correction_speed",.015);
    settings_.unknown_policy=declare_parameter<std::string>("unknown_policy","blocked");
    settings_.unknown_penalty=declare_parameter<double>("unknown_penalty",2.0);
    settings_.observation_ttl=declare_parameter<double>("observation_ttl",4.0);
    settings_.validate();
    if(settings_.unknown_policy=="observe-before-enter")observation_=create_subscription<std_msgs::msg::String>(
      "/controller_server/observation_status",10,[this](std_msgs::msg::String::ConstSharedPtr m){
        observation_wait_=m->data.rfind("waiting",0)==0;observation_received_=Clock::now();
      });
    body_route_controller::TrackingLimits limits;
    limits.measured_speed=settings_.measuredSpeed();limits.measured_yaw_rate=settings_.measuredYawRate();
    limits.braking_deceleration=settings_.linear_accel;
    guard_=body_route_controller::TrackingGuard(limits);
    profile_id_=declare_parameter<std::string>("profile_id","manual");
    execute_=declare_parameter<bool>("execute",false);
    if(!save_path_.empty()&&(execute_||fixed_path_||goals_.size()!=1)) {
      throw std::runtime_error("save_path_file requires preview of exactly one goals_file destination");
    }
    require_view_=declare_parameter<bool>("require_live_view",true);
    max_duration_=declare_parameter<double>("max_goal_duration",900.0);
    if(!std::isfinite(max_duration_)||max_duration_<30||max_duration_>3600) {throw std::runtime_error("max_goal_duration must be 30..3600 seconds");}
    status_=create_publisher<std_msgs::msg::String>("/trial/status",rclcpp::QoS(10).transient_local());
    odom_sub_=create_subscription<nav_msgs::msg::Odometry>("/odom",rclcpp::SensorDataQoS(),[this](nav_msgs::msg::Odometry::ConstSharedPtr m){
      try {
        if(m->header.frame_id!="odom"||m->child_frame_id!="base_footprint") {throw std::runtime_error("unexpected odometry frame");}
        const auto & p=m->pose.pose.position; const auto & v=m->twist.twist;
        Pose next{p.x,p.y,yaw(m->pose.pose.orientation)};
        if(leg_active_ && odom_received_!=Clock::time_point{}) {travel_+=std::hypot(next.x-odom_.x,next.y-odom_.y);}
        odom_=next; measured_={v.linear.x,v.linear.y,v.angular.z};
        if(leg_active_) {peak_speed_=std::max(peak_speed_,std::hypot(measured_.x,measured_.y));peak_yaw_=std::max(peak_yaw_,std::abs(measured_.yaw));}
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
      if(settings_.mode!="custom") {return;}
      std::istringstream row(m->data); std::string mode,heading;
      if(!(row>>mode>>heading)||mode.rfind("mode=",0)!=0||heading.rfind("heading=",0)!=0) {fault_="invalid_controller_status";return;}
      try {mode_=mode.substr(5); heading_=std::stod(heading.substr(8)); controller_received_=Clock::now();}
      catch(const std::exception &) {fault_="invalid_controller_heading";}
    });
    path_sub_=create_subscription<nav_msgs::msg::Path>("/plan",10,[this](nav_msgs::msg::Path::ConstSharedPtr m){
      if(fixed_path_||!moving_||m->header.frame_id!="map"||m->poses.empty()) {return;}
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
    follower_=rclcpp_action::create_client<Follow>(this,"/follow_path");
    const auto stamp=std::chrono::system_clock::now().time_since_epoch();
    results_file_=declare_parameter<std::string>("results_file","/tmp/route-trial-"+
      std::to_string(std::chrono::duration_cast<std::chrono::nanoseconds>(stamp).count())+".yaml");
    YAML::Node header;header["kind"]="run";header["mode"]=settings_.mode;header["profile_id"]=profile_id_;
    header["input_id"]=input_id_;header["route_type"]=fixed_path_?"fixed_path":"replanning_goals";
    header["speed_mps"]=settings_.speed;header["yaw_rate_radps"]=settings_.yaw_rate;
    header["linear_accel"]=settings_.linear_accel;header["angular_accel"]=settings_.angular_accel;
    header["execute"]=execute_;
    body_route_controller::writeNew(results_file_,"---\n"+YAML::Dump(header)+"\n");
    results_.open(results_file_,std::ios::app);
    results_.exceptions(std::ios::badbit|std::ios::failbit);
    report("Results: "+results_file_);

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
    if(cancel_ && !terminal_) {try {cancel_();} catch(const std::exception &) {}}
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
        std_msgs::msg::Bool m;m.data=!moving_;
        if(moving_&&released_at_==Clock::time_point{}) {
          released_at_=Clock::now();if(route_started_==Clock::time_point{}) {route_started_=released_at_;}
        }
        hold_->publish(m);
      });
      holdNow(); pump(1);
    }
    for(std::size_t index=0;index<goals_.size();++index) {
      current_index_=index;goal_=goals_[index];leg_written_=false;released_at_={};navigation_seconds_=0;
      if(execute_) {
        verifyProfile();
        auto start=Clock::now(),clear=Clock::now(); bool ready=false;
        while(rclcpp::ok()&&age(start)<30) {
          pump(); const auto reason=health(true);
          if(!fault_.empty()) {throw std::runtime_error(fault_);}
          if(!reason.empty()) {clear=Clock::now();}
          if(reason.empty()&&age(clear)>2) {ready=true;break;}
        }
        if(!ready) {throw std::runtime_error("stationary preflight: "+health(true));}
      }
      nav_msgs::msg::Path planned_path;
      if(fixed_path_) {
        planned_path.header.frame_id="map";planned_path.header.stamp=now();
        for(const auto & p:fixed_poses_) {planned_path.poses.push_back(stamped(p,now()));}
      } else {
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
        planned_path=planned.result->path;
      }
      if(planned_path.header.frame_id!="map") {throw std::runtime_error("planned path is not in map frame");}
      path_.clear(); double length=0;
      for(const auto & p:planned_path.poses) {
        if(!p.header.frame_id.empty()&&p.header.frame_id!="map") {throw std::runtime_error("mixed planned path frames");}
        (void)yaw(p.pose.orientation);
        Point q{p.pose.position.x,p.pose.position.y};
        if(!std::isfinite(q.x)||!std::isfinite(q.y)) {throw std::runtime_error("non_finite_path");}
        if(!path_.empty()) {length+=std::hypot(q.x-path_.back().x,q.y-path_.back().y);}
        path_.push_back(q);
      }
      if(path_.empty()||!std::isfinite(length)) {throw std::runtime_error("invalid planned path");}
      report("goal "+std::to_string(index+1)+"/"+std::to_string(goals_.size())+" planned "+std::to_string(length)+" m"+(execute_?"; HELD":"; PREVIEW ONLY"));
      planned_length_=length;
      std::ostringstream saved;saved<<std::setprecision(17);
      for(const auto & p:planned_path.poses) {saved<<p.pose.position.x<<' '<<p.pose.position.y<<' '<<yaw(p.pose.orientation)<<'\n';}
      path_id_=fingerprint(saved.str());
      if(!save_path_.empty()) {body_route_controller::writeNew(save_path_,saved.str());}
      if(!execute_) {YAML::Node preview;preview["kind"]="preview";preview["leg"]=index+1;preview["path_id"]=path_id_;preview["planned_m"]=length;record(preview);continue;}
      auto problem=health(true);if(!problem.empty()) {throw std::runtime_error(problem);}
      start_pose_=mapPose();
      if(fixed_path_&&std::hypot(start_pose_.x-path_.front().x,start_pose_.y-path_.front().y)>.035) {
        throw std::runtime_error("fixed_path_start_mismatch: reset robot to the saved path start");
      }
      guard_.reset();travel_=0;peak_speed_=0;peak_yaw_=0;leg_active_=true;
      if(fixed_path_) {
        Follow::Goal target;target.path=planned_path;target.controller_id="FollowPath";
        target.goal_checker_id="general_goal_checker";target.progress_checker_id="progress_checker";target.path_handler_id="PathHandler";
        drive<Follow>(follower_,target);
      } else {
        Nav::Goal target;target.pose=stamped(goal_,now());drive<Nav>(nav_,target);
      }
      pump(1); problem=health(true); if(!problem.empty()) {throw std::runtime_error("arrival: "+problem);}
      const auto pose=mapPose();
      if(std::hypot(pose.x-goal_.x,pose.y-goal_.y)>.035||std::abs(wrap(pose.yaw-goal_.yaw))>.055) {throw std::runtime_error("arrival pose outside goal tolerance");}
      writeLeg("success","");
      report("goal "+std::to_string(index+1)+" complete and measured stopped; HELD; navigation_s="+
        std::to_string(navigation_seconds_)+" completion_s="+std::to_string(last_completion_)+" distance_m="+std::to_string(travel_));
    }
    writeSummary(execute_?"success":"preview","");
    report(execute_?"Route complete; HELD":"All goals planned; no hold release or navigation goal sent");return true;
  }
  void failed(const std::string & reason)
  {
    holdNow();
    if(leg_active_&&!leg_written_) {
      if(!terminal_&&released_at_!=Clock::time_point{}) {navigation_seconds_=age(released_at_);}
      writeLeg("failure",reason);
    }
    writeSummary("failure",reason);
    report("STOPPED: "+reason);
  }
private:
  void record(const YAML::Node & node) {results_<<"---\n"<<YAML::Dump(node)<<'\n';results_.flush();}
  static YAML::Node poseNode(const Pose & p)
  {YAML::Node n;n.push_back(p.x);n.push_back(p.y);n.push_back(p.yaw);return n;}
  void writeLeg(const std::string & status,const std::string & reason)
  {
    leg_active_=false;leg_written_=true;
    last_completion_=released_at_==Clock::time_point{}?0:age(released_at_);
    total_travel_+=travel_;total_navigation_+=navigation_seconds_;
    YAML::Node n;n["kind"]="leg";n["leg"]=current_index_+1;n["status"]=status;n["reason"]=reason;
    n["path_id"]=path_id_;n["planned_m"]=planned_length_;n["distance_m"]=travel_;
    n["navigation_s"]=navigation_seconds_;n["completion_s"]=last_completion_;
    n["peak_measured_mps"]=peak_speed_;n["peak_measured_radps"]=peak_yaw_;
    n["start"]=poseNode(start_pose_);n["goal"]=poseNode(goal_);
    try {n["end"]=poseNode(mapPose());} catch(const std::exception &) {n["end_unavailable"]=true;}
    record(n);
  }
  void writeSummary(const std::string & status,const std::string & reason)
  {
    YAML::Node n;n["kind"]="summary";n["status"]=status;n["reason"]=reason;
    n["route_wall_s"]=route_started_==Clock::time_point{}?0:age(route_started_);
    n["navigation_s"]=total_navigation_;n["distance_m"]=total_travel_;record(n);
  }
  void verifyProfile()
  {
    using Parameter=rclcpp::Parameter;
    auto check=[this](const std::string & node,const std::vector<Parameter> & expected) {
      auto client=std::make_shared<rclcpp::AsyncParametersClient>(this,node);
      if(!client->wait_for_service(2s)) {throw std::runtime_error("profile verification unavailable: "+node);}
      std::vector<std::string> names;for(const auto & p:expected) {names.push_back(p.get_name());}
      auto future=client->get_parameters(names);
      if(!wait(future,3)) {throw std::runtime_error("profile verification timeout: "+node);}
      const auto values=future.get();
      if(values.size()!=expected.size()) {throw std::runtime_error("incomplete runtime profile: "+node);}
      for(std::size_t i=0;i<values.size();++i) {
        const auto & a=values[i];const auto & b=expected[i];
        bool same=a.get_type()==b.get_type();
        if(same&&a.get_type()==rclcpp::ParameterType::PARAMETER_DOUBLE) {same=std::isfinite(a.as_double())&&std::abs(a.as_double()-b.as_double())<1e-9;}
        else if(same) {same=a.value_to_string()==b.value_to_string();}
        if(!same) {throw std::runtime_error("runtime profile mismatch: "+node+"."+names[i]+"; restart with matching generated profiles");}
      }
    };
    std::vector<Parameter> nav{Parameter("FollowPath.plugin",settings_.mode=="custom"?
      "body_route_controller::BodyRouteController":"nav2_mppi_controller::MPPIController")};
    if(settings_.mode=="custom") {
      nav.emplace_back("FollowPath.max_speed",settings_.speed);nav.emplace_back("FollowPath.correction_speed",settings_.correction_speed);
      nav.emplace_back("FollowPath.max_yaw_rate",settings_.yaw_rate);nav.emplace_back("FollowPath.linear_accel",settings_.linear_accel);
      nav.emplace_back("FollowPath.angular_accel",settings_.angular_accel);
    } else {
      nav.emplace_back("FollowPath.vx_max",settings_.speed);nav.emplace_back("FollowPath.vx_min",-settings_.speed);
      nav.emplace_back("FollowPath.vy_max",settings_.speed);nav.emplace_back("FollowPath.wz_max",settings_.yaw_rate);
      nav.emplace_back("FollowPath.ax_max",settings_.linear_accel);nav.emplace_back("FollowPath.ax_min",-settings_.linear_accel);
      nav.emplace_back("FollowPath.ay_max",settings_.linear_accel);nav.emplace_back("FollowPath.ay_min",-settings_.linear_accel);
      nav.emplace_back("FollowPath.az_max",settings_.angular_accel);
    }
    if(settings_.unknown_policy=="observe-before-enter"){
      const auto delegate=nav.front().as_string();nav.front()=Parameter("FollowPath.plugin","body_route_controller::ObservationController");
      nav.emplace_back("FollowPath.delegate_plugin",delegate);
      check("/local_costmap/local_costmap",{Parameter("observation_layer.unknown_policy",settings_.unknown_policy),
        Parameter("observation_layer.ttl",settings_.observation_ttl),Parameter("observation_layer.linear_decel",settings_.linear_accel),
        Parameter("observation_layer.angular_decel",settings_.angular_accel)});
      check("/planner_server",{Parameter("GridBased.unknown_policy",settings_.unknown_policy),Parameter("GridBased.unknown_penalty",settings_.unknown_penalty)});
    }
    check("/controller_server",nav);
    check("/swerve_controller",{Parameter("max_linear",settings_.speed),Parameter("max_angular",settings_.yaw_rate),
      Parameter("max_linear_accel",settings_.linear_accel),Parameter("max_angular_accel",settings_.angular_accel),Parameter("heading_hold_kp",0.0)});
    report("Verified runtime controller and wheel limits: "+settings_.mode);
  }
  template<typename Action> void drive(const typename rclcpp_action::Client<Action>::SharedPtr & client,
    const typename Action::Goal & target)
  {
    if(!client->wait_for_action_server(2s)) {throw std::runtime_error("navigation action unavailable");}
    terminal_=false;cancel_={};controller_received_={};observation_received_={};
    typename rclcpp_action::Client<Action>::SendGoalOptions options;
    options.goal_response_callback=[this,client](typename rclcpp_action::ClientGoalHandle<Action>::SharedPtr h) {
      if(h) {cancel_=[client,h](){(void)client->async_cancel_goal(h);};if(closing_) {cancel_();}}
    };
    auto accepted=client->async_send_goal(target,options);
    if(!wait(accepted,5)) {throw std::runtime_error("navigation acceptance timeout");}
    auto handle=accepted.get();if(!handle) {throw std::runtime_error("navigation rejected goal");}
    auto done=client->async_get_result(handle);moving_=true;const auto started=Clock::now();
    report("goal "+std::to_string(current_index_+1)+" ACTIVE; mode="+settings_.mode+"; deadline "+std::to_string(max_duration_)+" s");
    while(rclcpp::ok()&&done.wait_for(0s)!=std::future_status::ready) {
      pump();auto problem=health(false);
      if(problem.empty()&&age(started)>1.0) {
        const bool observation_wait=settings_.unknown_policy=="observe-before-enter"&&observation_wait_&&age(observation_received_)<.4;
        const bool stationary_planning=settings_.unknown_policy=="observe-before-enter"&&
          controller_received_==Clock::time_point{}&&observation_received_==Clock::time_point{}&&age(started)<4.0&&
          std::hypot(measured_.x,measured_.y)<.003&&std::abs(measured_.yaw)<.01;
        if(stationary_planning) {continue;}
        if(settings_.mode=="custom"&&!observation_wait&&age(controller_received_)>.4) {problem="stale_controller_status";}
        else {problem=guard_.check(mapPose(),odom_,measured_,path_,observation_wait?"observation_wait":mode_,heading_,settings_.mode=="custom");}
      }
      if(problem.empty()&&travel_>2*planned_length_+.25) {problem="travel_budget_exceeded";}
      if(problem.empty()&&age(started)>max_duration_) {problem="goal_deadline";}
      if(!problem.empty()) {navigation_seconds_=released_at_==Clock::time_point{}?0:age(released_at_);throw std::runtime_error(problem);}
    }
    holdNow();navigation_seconds_=released_at_==Clock::time_point{}?0:age(released_at_);
    if(!rclcpp::ok()||done.wait_for(0s)!=std::future_status::ready) {throw std::runtime_error("interrupted");}
    terminal_=true;const auto answer=done.get();
    if(answer.code!=rclcpp_action::ResultCode::SUCCEEDED||answer.result->error_code!=0) {throw std::runtime_error("navigation failed: "+answer.result->error_msg);}
  }
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
    if(std::hypot(measured_.x,measured_.y)>settings_.measuredSpeed()||std::abs(measured_.yaw)>settings_.measuredYawRate()) {return "measured_speed_limit";}
    if(stationary&&(std::hypot(measured_.x,measured_.y)>.003||std::abs(measured_.yaw)>.01)) {return "not_measured_stopped";}
    try {(void)mapPose();} catch(const std::exception & e) {return e.what();}
    return {};
  }
  TrialSettings settings_;
  bool fixed_path_=false,leg_active_=false,leg_written_=false;
  std::vector<Pose> fixed_poses_;
  std::string save_path_,input_id_,profile_id_,path_id_,results_file_;
  std::ofstream results_;
  std::size_t current_index_=0;
  Pose start_pose_;
  double planned_length_=0,navigation_seconds_=0,last_completion_=0,total_navigation_=0,total_travel_=0,peak_speed_=0,peak_yaw_=0;
  Clock::time_point released_at_{},route_started_{};
  std::vector<Pose> goals_;Pose goal_,odom_; Velocity measured_;
  std::vector<Point> path_;body_route_controller::TrackingGuard guard_;
  bool execute_=false,require_view_=true,moving_=false,closing_=false,terminal_=true,view_fresh_=false;
  double max_duration_=900,travel_=0,odom_stamp_=0,heading_=0;
  std::array<double,2> scan_stamp_{};std::array<int,2> valid_{},points_{};
  Clock::time_point odom_received_{},view_received_{},controller_received_{};
  std::array<Clock::time_point,2> scan_received_{};
  bool observation_wait_=false;Clock::time_point observation_received_{};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr observation_;
  std::string fault_,mode_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  tf2_ros::Buffer tf_;tf2_ros::TransformListener listener_;
  std::function<void()> cancel_;rclcpp_action::Client<Follow>::SharedPtr follower_;
  rclcpp_action::Client<Nav>::SharedPtr nav_;rclcpp_action::Client<Plan>::SharedPtr planner_;
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
  catch(const std::exception & e) {if(trial) {trial->holdNow();try {trial->failed(e.what());} catch(const std::exception & log_error) {RCLCPP_ERROR(trial->get_logger(),"Result logging failed: %s; original fault: %s",log_error.what(),e.what());}} else {RCLCPP_ERROR(rclcpp::get_logger("body_route_trial"),"%s",e.what());}}
  if(trial) {trial->finish();trial.reset();}
  rclcpp::shutdown();return code;
}
