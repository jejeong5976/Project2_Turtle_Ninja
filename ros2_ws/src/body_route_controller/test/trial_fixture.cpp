// Small ROS action/feedback fixture for route_trial failure behavior. It does
// not emulate drivetrain motion; full-stack motion has a separate recorded run.
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/action/follow_path.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>
#include <sys/wait.h>
#include <unistd.h>
#include <csignal>
#include <iostream>
#include <thread>
using namespace std::chrono_literals;
using Plan=nav2_msgs::action::ComputePathToPose;
using Nav=nav2_msgs::action::NavigateToPose;
using Follow=nav2_msgs::action::FollowPath;
using Clock=std::chrono::steady_clock;
int main(int argc,char**argv) {
 if(argc!=4)return 2;
 const std::string binary=argv[1],scenario=argv[2];
 const auto output=std::filesystem::path(argv[3])/(scenario+"-"+std::to_string(getpid()));
 std::filesystem::create_directories(output);
 const bool original=scenario.find("original")!=std::string::npos;
 const bool fixed=scenario.find("fixed")!=std::string::npos;
 const bool success=scenario.find("success")!=std::string::npos;
 const double speed=scenario=="custom_fast_success"?.08:.025;
 const std::string result_file=(output/"result.yaml").string(),goal_file=(output/"goals.path").string();
 {std::ofstream f(goal_file);if(fixed)f<<"0 0 0\n";f<<"0.15 0 0\n";}

 rclcpp::init(argc,argv);auto node=std::make_shared<rclcpp::Node>("route_trial_fixture");
 rclcpp::executors::SingleThreadedExecutor ex;ex.add_node(node);
 auto controller=std::make_shared<rclcpp::Node>("controller_server");
 auto wheel=std::make_shared<rclcpp::Node>("swerve_controller");
 ex.add_node(controller);ex.add_node(wheel);
 controller->declare_parameter("FollowPath.plugin",original?"nav2_mppi_controller::MPPIController":"body_route_controller::BodyRouteController");
 if(original){
  for(const auto & key:{"vx_max","vy_max"})controller->declare_parameter(std::string("FollowPath.")+key,speed);
  controller->declare_parameter("FollowPath.vx_min",-speed);controller->declare_parameter("FollowPath.wz_max",.1);
  for(const auto & key:{"ax_max","ay_max"})controller->declare_parameter(std::string("FollowPath.")+key,.05);
  for(const auto & key:{"ax_min","ay_min"})controller->declare_parameter(std::string("FollowPath.")+key,-.05);
  controller->declare_parameter("FollowPath.az_max",.2);
 }else{
  controller->declare_parameter("FollowPath.prepare_passages",false);
  controller->declare_parameter("FollowPath.drive_style","axis-stop");
  controller->declare_parameter("FollowPath.heading_policy","reactive");
  controller->declare_parameter("FollowPath.heading_preview_time",5.0);
  controller->declare_parameter("FollowPath.max_travel_angle",std::acos(-1.0)/4);
  controller->declare_parameter("FollowPath.max_speed",speed);controller->declare_parameter("FollowPath.correction_speed",.015);
  controller->declare_parameter("FollowPath.max_yaw_rate",.1);controller->declare_parameter("FollowPath.linear_accel",.05);controller->declare_parameter("FollowPath.angular_accel",.2);
 }
 wheel->declare_parameter("blend_steer_drive",false);wheel->declare_parameter("blend_stall_s",1.0);wheel->declare_parameter("blend_margin",.7);
 wheel->declare_parameter("max_linear",scenario=="profile_mismatch"?.09:speed);
 wheel->declare_parameter("max_angular",.1);wheel->declare_parameter("max_linear_accel",.05);wheel->declare_parameter("max_angular_accel",.2);wheel->declare_parameter("heading_hold_kp",0.0);
 int plans=0,navs=0,cancels=0,holds=0;bool held=true,released=false,held_after=false,sent_interrupt=false;
 Clock::time_point release_time;std::string status;
 auto hp=node->create_publisher<std_msgs::msg::Bool>("/mission/hold",10);
 if(scenario!="owner_conflict")hp.reset();
 auto hs=node->create_subscription<std_msgs::msg::Bool>("/mission/hold",10,[&](std_msgs::msg::Bool::ConstSharedPtr m){
  ++holds;held=m->data;if(!held&&!released){released=true;release_time=Clock::now();}if(released&&held)held_after=true;});
 auto ss=node->create_subscription<std_msgs::msg::String>("/trial/status",rclcpp::QoS(1).transient_local(),[&](std_msgs::msg::String::ConstSharedPtr m){status=m->data;});
 auto odom=node->create_publisher<nav_msgs::msg::Odometry>("/odom",rclcpp::SensorDataQoS());
 auto front=node->create_publisher<sensor_msgs::msg::LaserScan>("/lidar_front/scan",rclcpp::SensorDataQoS());
 auto rear=node->create_publisher<sensor_msgs::msg::LaserScan>("/lidar_rear/scan",rclcpp::SensorDataQoS());
 auto view=node->create_publisher<std_msgs::msg::Bool>("/live_view/fresh",10);
 auto ctl=node->create_publisher<std_msgs::msg::String>("/controller_server/body_route_status",10);
 tf2_ros::TransformBroadcaster::RequiredInterfaces interfaces(node->get_node_parameters_interface(),node->get_node_topics_interface());
 tf2_ros::TransformBroadcaster tf(interfaces);tf2_ros::StaticTransformBroadcaster static_tf(interfaces);
 std::vector<geometry_msgs::msg::TransformStamped> statics;
 for(int i=0;i<2;++i){geometry_msgs::msg::TransformStamped t;t.header.frame_id="base_footprint";t.child_frame_id=i?"lidar_rear_link":"lidar_front_link";t.header.stamp=node->now();t.transform.translation.x=i?-.1145:.1145;t.transform.rotation.w=1;statics.push_back(t);}static_tf.sendTransform(statics);
 auto planner=rclcpp_action::create_server<Plan>(node,"compute_path_to_pose",
  [](const auto&,const auto&){return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},
  [](const auto&){return rclcpp_action::CancelResponse::ACCEPT;},
  [&](const std::shared_ptr<rclcpp_action::ServerGoalHandle<Plan>> h){++plans;auto r=std::make_shared<Plan::Result>();r->path.header.frame_id="map";r->path.header.stamp=node->now();geometry_msgs::msg::PoseStamped start;start.header=r->path.header;start.pose.orientation.w=1;r->path.poses={start,h->get_goal()->goal};h->succeed(r);});
 std::shared_ptr<rclcpp_action::ServerGoalHandle<Nav>> active;
 auto nav=rclcpp_action::create_server<Nav>(node,"navigate_to_pose",
  [&](const auto&,const auto&){++navs;return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},
  [&](const auto&){++cancels;return rclcpp_action::CancelResponse::ACCEPT;},
  [&](const std::shared_ptr<rclcpp_action::ServerGoalHandle<Nav>> h){active=h;});
 std::shared_ptr<rclcpp_action::ServerGoalHandle<Follow>> active_follow;
 auto follow=rclcpp_action::create_server<Follow>(node,"follow_path",
  [&](const auto&,const auto&){++navs;return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},
  [&](const auto&){++cancels;return rclcpp_action::CancelResponse::ACCEPT;},
  [&](const std::shared_ptr<rclcpp_action::ServerGoalHandle<Follow>> h){active_follow=h;});
 auto tick=node->create_wall_timer(50ms,[&](){
  const double elapsed=released?std::chrono::duration<double>(Clock::now()-release_time).count():0;
  const bool fault=released&&elapsed>1;
  const double position=success?std::min(.15,.075*elapsed):0;

  if(hp){std_msgs::msg::Bool h;h.data=true;hp->publish(h);}
  nav_msgs::msg::Odometry o;o.header.frame_id="odom";o.child_frame_id="base_footprint";o.header.stamp=node->now();o.pose.pose.orientation.w=1;o.pose.pose.position.x=position;
  if(success&&released&&elapsed<2){o.twist.twist.linear.x=.02;if(original){o.twist.twist.linear.y=.01;o.twist.twist.angular.z=.05;}}
  if(fault&&scenario=="original_overspeed")o.twist.twist.linear.x=.1;
  odom->publish(o);
  geometry_msgs::msg::TransformStamped t;t.header.frame_id="map";t.child_frame_id="odom";t.header.stamp=node->now();t.transform.rotation.w=1;tf.sendTransform(t);t.header.frame_id="odom";t.child_frame_id="base_footprint";t.transform.translation.x=position;tf.sendTransform(t);
  sensor_msgs::msg::LaserScan s;s.header.stamp=node->now();s.header.frame_id="lidar_front_link";s.angle_min=-3.14;s.angle_max=3.14;s.angle_increment=6.28/720;s.range_min=.05;s.range_max=8;s.ranges.assign(721,3.0);front->publish(s);s.header.frame_id="lidar_rear_link";
  if(fault&&scenario=="invalid_scan")s.angle_increment=std::numeric_limits<float>::quiet_NaN();
  if(!(fault&&(scenario=="rear_loss"||scenario=="original_rear_loss")))rear->publish(s);
  std_msgs::msg::Bool visible;visible.data=!(fault&&scenario=="view_loss");view->publish(visible);
  std_msgs::msg::String c;c.data=held?"mode=mission_hold heading=0":"mode=forward heading=0";if(!original)ctl->publish(c);
  if(success&&elapsed>2.4){
   if(active){active->succeed(std::make_shared<Nav::Result>());active.reset();}
   if(active_follow){active_follow->succeed(std::make_shared<Follow::Result>());active_follow.reset();}
  }
  if(fault&&scenario=="navigation_abort"&&active){auto result=std::make_shared<Nav::Result>();result->error_code=100;result->error_msg="fixture abort";active->abort(result);active.reset();}
  if(active_follow&&active_follow->is_canceling()){active_follow->canceled(std::make_shared<Follow::Result>());active_follow.reset();}
  if(active&&active->is_canceling()){active->canceled(std::make_shared<Nav::Result>());active.reset();}
 });
 pid_t child=fork();if(child<0)return 3;
 if(child==0){
  const auto input=std::string(fixed?"path_file:=":"goals_file:=")+goal_file;
  const auto result_arg="results_file:="+result_file;
  const auto mode_arg=std::string("controller_mode:=")+(original?"original":"custom");
  const auto speed_arg="speed:="+std::to_string(speed);
  execl(binary.c_str(),binary.c_str(),"--ros-args","-p",input.c_str(),"-p",result_arg.c_str(),"-p",mode_arg.c_str(),"-p",speed_arg.c_str(),"-p",scenario=="preview"?"execute:=false":"execute:=true",static_cast<char*>(nullptr));_exit(127);
 }
 const auto start=Clock::now();int result=0;bool exited=false;
 while(Clock::now()-start<45s){ex.spin_some();std::this_thread::sleep_for(5ms);
  if(scenario=="interrupt"&&released&&!sent_interrupt&&Clock::now()-release_time>1s){kill(child,SIGINT);sent_interrupt=true;}
  if(waitpid(child,&result,WNOHANG)==child){exited=true;break;}
 }
 if(!exited){kill(child,SIGTERM);waitpid(child,&result,0);}
 for(int i=0;i<20;++i){ex.spin_some();std::this_thread::sleep_for(5ms);}
 bool ok=exited&&WIFEXITED(result);
 if(scenario=="preview")ok=ok&&WEXITSTATUS(result)==0&&plans==1&&navs==0&&holds==0;
 else if(scenario=="owner_conflict")ok=ok&&WEXITSTATUS(result)==1&&navs==0&&!released&&status.find("another hold owner")!=std::string::npos;
 else if(scenario=="profile_mismatch")ok=ok&&WEXITSTATUS(result)==1&&navs==0&&!released&&status.find("runtime profile mismatch")!=std::string::npos;
 else if(success)ok=ok&&WEXITSTATUS(result)==0&&navs==1&&cancels==0&&released&&held_after&&status.find("Route complete")!=std::string::npos&&plans==(fixed?0:1);
 else if(scenario=="navigation_abort")ok=ok&&WEXITSTATUS(result)==1&&released&&held_after&&status.find("fixture abort")!=std::string::npos;
 else {std::string reason=scenario=="original_overspeed"?"measured_speed_limit":scenario=="original_rear_loss"?"stale_scan_1":scenario=="rear_loss"?"stale_scan_1":scenario=="view_loss"?"live_view_not_fresh":scenario=="interrupt"?"operator_interrupt":"insufficient_scan_coverage";
  ok=ok&&WEXITSTATUS(result)==1&&released&&held_after&&cancels==1&&status.find(reason)!=std::string::npos;}
 try {
  const auto docs=YAML::LoadAllFromFile(result_file);const auto summary=docs.back();
  ok=ok&&summary["kind"].as<std::string>()=="summary";
  ok=ok&&summary["status"].as<std::string>()==(success?"success":scenario=="preview"?"preview":"failure");
  if(success){const auto leg=docs[1];ok=ok&&leg["navigation_s"].as<double>()>2&&leg["completion_s"].as<double>()>leg["navigation_s"].as<double>()+.9&&leg["distance_m"].as<double>()>.14;}
 }catch(const std::exception& e){std::cerr<<e.what()<<std::endl;ok=false;}
 std::cout<<(ok?"PASS ":"FAIL ")<<scenario<<" navs="<<navs<<" cancels="<<cancels<<" released="<<released<<" held_after="<<held_after<<" status="<<status<<std::endl;
 rclcpp::shutdown();return ok?0:1;
}
