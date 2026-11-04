#include "common.hpp"
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav2_msgs/msg/costmap.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <iostream>
using namespace maze_sim;
using namespace std::chrono_literals;
class Feed : public rclcpp::Node {
public:
  Feed():Node("scanned_map_simulation"),root(runRoot()),config(YAML::LoadFile((root/"maps/scenario.yaml").string())),
    grid(Grid::read(root/"maps/truth.yaml")),cost(grid.costmap()),start(pose(config["start"])),tf(get_clock()),listener(tf),
    trace(root/"runtime.csv"),plans(root/"plans.csv"),arrivals(root/"arrivals.csv") {
    trace<<std::setprecision(17)<<"time,truth_x,truth_y,truth_yaw,estimated_x,estimated_y,estimated_yaw,estimate_valid,speed,wz,nav_x,nav_y,nav_wz,safe_x,safe_y,safe_wz,out_x,out_y,out_wz,held,truth_clear,mode\n";
    plans<<std::setprecision(17)<<"time,plan_id,x,y,yaw\n";arrivals<<std::setprecision(17)<<"leg,time,truth_x,truth_y,truth_yaw,position_error_m,yaw_error_rad\n";
    odom_sub=create_subscription<nav_msgs::msg::Odometry>("/odom",rclcpp::SensorDataQoS(),[this](nav_msgs::msg::Odometry::ConstSharedPtr m){
      if(m->header.frame_id!="odom"||m->child_frame_id!="base_footprint")throw std::runtime_error("unexpected odom frame");
      odom_stamp=m->header.stamp;odom={m->pose.pose.position.x,m->pose.pose.position.y,angle(m->pose.pose.orientation)};speed=std::hypot(m->twist.twist.linear.x,m->twist.twist.linear.y);wz=m->twist.twist.angular.z;received=true;
    });
    for(int i=0;i<3;++i)cmd_sub.push_back(create_subscription<geometry_msgs::msg::TwistStamped>(std::vector<std::string>{"/cmd_vel_nav","/cmd_vel_safe","/cmd_vel"}[i],10,[this,i](geometry_msgs::msg::TwistStamped::ConstSharedPtr m){commands[i]=m->twist;}));
    held_sub=create_subscription<std_msgs::msg::Bool>("/mission/hold",10,[this](std_msgs::msg::Bool::ConstSharedPtr m){held=m->data;});
    mode_sub=create_subscription<std_msgs::msg::String>("/controller_server/body_route_status",10,[this](std_msgs::msg::String::ConstSharedPtr m){mode=m->data;std::replace(mode.begin(),mode.end(),',',';');});
    trial_sub=create_subscription<std_msgs::msg::String>("/trial/status",rclcpp::QoS(10).transient_local(),[this](std_msgs::msg::String::ConstSharedPtr m){
      if(m->data.find(" complete and measured stopped;")==std::string::npos)return;
      std::istringstream text(m->data);std::string word;int leg=0;text>>word>>leg;if(leg<1||leg>static_cast<int>(config["goals"].size()))return;
      auto p=compose(start,odom),g=pose(config["goals"][leg-1]);arrivals<<leg<<','<<now().seconds()<<','<<p.x<<','<<p.y<<','<<p.yaw<<','<<std::hypot(p.x-g.x,p.y-g.y)<<','<<std::abs(body_route_controller::wrap(p.yaw-g.yaw))<<'\n';arrivals.flush();
    });
    plan_sub=create_subscription<nav_msgs::msg::Path>("/plan",10,[this](nav_msgs::msg::Path::ConstSharedPtr m){if(m->header.frame_id!="map")return;++plan_id;for(auto p:m->poses)plans<<now().seconds()<<','<<plan_id<<','<<p.pose.position.x<<','<<p.pose.position.y<<','<<angle(p.pose.orientation)<<'\n';plans.flush();});
    evidence_sub=create_subscription<nav_msgs::msg::OccupancyGrid>("/global_costmap/global_costmap/observation_space",rclcpp::QoS(1).transient_local(),[this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m){
      if(m->data.size()!=std::size_t(m->info.width)*m->info.height)return;
      cv::Mat state(m->info.height,m->info.width,CV_8U);
      for(unsigned y=0;y<m->info.height;++y)for(unsigned x=0;x<m->info.width;++x){const int v=m->data[y*m->info.width+x];state.at<unsigned char>(m->info.height-1-y,x)=v<0?205:v==100?0:v==25?100:254;}
      cv::imwrite((root/"observed-space.pgm").string(),state);
    });
    gate_sub=create_subscription<std_msgs::msg::String>("/controller_server/observation_status",10,[this](std_msgs::msg::String::ConstSharedPtr m){
      if(m->data!=last_gate){std::ofstream out(root/"observation-events.csv",std::ios::app);out<<now().seconds()<<','<<m->data<<'\n';last_gate=m->data;}
    });
    body_pub=create_publisher<geometry_msgs::msg::PolygonStamped>("/simulation/truth_footprint",10);
    for(const auto &name:std::vector<std::string>{"global","local"})cost_sub.push_back(create_subscription<nav2_msgs::msg::Costmap>("/"+name+"_costmap/costmap_raw",rclcpp::QoS(1).transient_local(),[this,name](nav2_msgs::msg::Costmap::ConstSharedPtr m){
      if(m->data.size()!=std::size_t(m->metadata.size_x)*m->metadata.size_y||m->data.empty())return;
      cv::Mat raw(m->metadata.size_y,m->metadata.size_x,CV_8U,const_cast<unsigned char *>(m->data.data())),flipped;cv::flip(raw,flipped,0);
      cv::imwrite((root/(name+"-costmap-raw.pgm")).string(),flipped);
      YAML::Node n;n["frame"]=m->header.frame_id;n["resolution"]=m->metadata.resolution;n["origin"]=poseNode({m->metadata.origin.position.x,m->metadata.origin.position.y,0});n["stamp"]=rclcpp::Time(m->header.stamp).seconds();
      std::ofstream meta(root/(name+"-costmap.yaml"));meta<<YAML::Dump(n)<<'\n';
    }));
    front=create_publisher<sensor_msgs::msg::LaserScan>("/lidar_front/scan",rclcpp::SensorDataQoS());rear=create_publisher<sensor_msgs::msg::LaserScan>("/lidar_rear/scan",rclcpp::SensorDataQoS());
    heartbeat=create_publisher<std_msgs::msg::Bool>("/live_view/fresh",10);stop=create_publisher<std_msgs::msg::Bool>("/live_view/stop",rclcpp::QoS(1).transient_local());
    truth_path=create_publisher<nav_msgs::msg::Path>("/simulation/truth_path",rclcpp::QoS(1).transient_local());estimated_path=create_publisher<nav_msgs::msg::Path>("/simulation/estimated_path",rclcpp::QoS(1).transient_local());
    scan_timer=create_wall_timer(100ms,[this](){if(received){scan(front,.1145,pi);scan(rear,-.1145,0);}std_msgs::msg::Bool m;m.data=true;heartbeat->publish(m);});
    record_timer=create_wall_timer(50ms,[this](){record();});
  }
private:
  static double angle(const geometry_msgs::msg::Quaternion &q){return std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));}
  geometry_msgs::msg::PoseStamped stamp(Pose p){geometry_msgs::msg::PoseStamped m;m.header.frame_id="map";m.header.stamp=now();m.pose.position.x=p.x;m.pose.position.y=p.y;m.pose.orientation.z=std::sin(p.yaw/2);m.pose.orientation.w=std::cos(p.yaw/2);return m;}
  void scan(rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher,double offset,double heading){
    if(first_scan<0)first_scan=now().seconds();
    const auto p=compose(start,odom),sensor=compose(p,{offset,0,heading});sensor_msgs::msg::LaserScan m;m.header.stamp=odom_stamp;m.header.frame_id=offset>0?"lidar_front_link":"lidar_rear_link";
    m.angle_min=-pi;m.angle_increment=2*pi/720;m.angle_max=m.angle_min+720*m.angle_increment;m.range_min=.05;m.range_max=config["sensor_range_m"].as<double>(8);m.scan_time=.1;m.time_increment=0;
    for(int i=0;i<=720;++i){const double angle=sensor.yaw+m.angle_min+i*m.angle_increment;
      // Identical central self-occlusion geometry to the evidence adapter, but
      // generated here from truth; blocked rays are invalid, never free.
      const double dx=p.x-sensor.x,dy=p.y-sensor.y,t=std::max(0.0,dx*std::cos(angle)+dy*std::sin(angle));
      const bool self=config["ground_truth_complete"].as<bool>(false)&&std::hypot(dx-t*std::cos(angle),dy-t*std::sin(angle))<.08;
      const bool hidden=config["partial_visibility"].as<bool>(false)&&now().seconds()-first_scan<30&&std::abs(body_route_controller::wrap(angle-p.yaw))<.7;
      m.ranges.push_back((self||hidden)?std::numeric_limits<float>::quiet_NaN():grid.ray(sensor.x,sensor.y,angle,m.range_max));
    }
    publisher->publish(m);
  }
  void record(){
    if(!received)return;
    const auto p=compose(start,odom);Pose estimated;bool valid=false;
    try{auto t=tf.lookupTransform("map","base_footprint",tf2::TimePointZero);if((now()-rclcpp::Time(t.header.stamp)).seconds()<.5){estimated={t.transform.translation.x,t.transform.translation.y,angle(t.transform.rotation)};valid=true;}}catch(const tf2::TransformException &){}
    geometry_msgs::msg::PolygonStamped poly;poly.header=stamp(p).header;
    for(auto q:footprint()){auto v=compose(p,{q.x,q.y,0});geometry_msgs::msg::Point32 point;point.x=v.x;point.y=v.y;poly.polygon.points.push_back(point);}body_pub->publish(poly);
    const bool clear=body_route_controller::footprintSafe(cost,footprint(),p);
    const bool failed_oracle=config["ground_truth_complete"].as<bool>(false)&&!clear;
    if(!clear&&!collision_latched){collision_latched=true;
      // Complete-world tests terminate as failed evaluations. Hidden truth never
      // sends a navigation command or an obstacle input that could guide a route.
      if(!failed_oracle){std_msgs::msg::Bool m;m.data=true;stop->publish(m);}
      RCLCPP_ERROR(get_logger(),"SIMULATION truth footprint overlap: failed independent evaluation");
    }
    trace<<now().seconds()<<','<<p.x<<','<<p.y<<','<<p.yaw<<','<<estimated.x<<','<<estimated.y<<','<<estimated.yaw<<','<<valid<<','<<speed<<','<<wz;
    for(auto c:commands)trace<<','<<c.linear.x<<','<<c.linear.y<<','<<c.angular.z;
    trace<<','<<held<<','<<clear<<','<<mode<<'\n';if(++samples%4==0){trace.flush();tpath.header=stamp(p).header;tpath.poses.push_back(stamp(p));truth_path->publish(tpath);if(valid){epath.header=stamp(estimated).header;epath.poses.push_back(stamp(estimated));estimated_path->publish(epath);}}
    if(failed_oracle){trace.flush();throw std::runtime_error("independent truth collision evaluation failed");}
  }
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr evidence_sub;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gate_sub;std::string last_gate;double first_scan=-1;builtin_interfaces::msg::Time odom_stamp;
  fs::path root;YAML::Node config;Grid grid;nav2_costmap_2d::Costmap2D cost;Pose start,odom;
  tf2_ros::Buffer tf;tf2_ros::TransformListener listener;std::ofstream trace,plans,arrivals;
  geometry_msgs::msg::Twist commands[3];nav_msgs::msg::Path tpath,epath;bool received=false,held=true,collision_latched=false;double speed=0,wz=0;int plan_id=0,samples=0;std::string mode;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr> cmd_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr held_sub;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub,trial_sub;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr front,rear;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr heartbeat,stop;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr truth_path,estimated_path;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr body_pub;
  std::vector<rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr> cost_sub;
  rclcpp::TimerBase::SharedPtr scan_timer,record_timer;
};
int main(int argc,char **argv){try{rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<Feed>());rclcpp::shutdown();return 0;}catch(const std::exception &e){std::cerr<<e.what()<<'\n';return 2;}}
