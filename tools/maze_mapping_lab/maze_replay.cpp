#include "common.hpp"
#include "ros_helpers.hpp"
#include "processes.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/int64.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>
#include <slam_toolbox/srv/serialize_pose_graph.hpp>
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <deque>
#include <thread>
#include <signal.h>
#include <sys/resource.h>
using namespace maze;
static volatile sig_atomic_t interrupted=0;
static void stop_signal(int){interrupted=1;}
constexpr double epoch=100.;
int64_t ns(double t){return int64_t(std::llround((epoch+t)*1e9));}
geometry_msgs::msg::Quaternion quat(double yaw){geometry_msgs::msg::Quaternion q;q.z=sin(yaw/2);q.w=cos(yaw/2);return q;}
geometry_msgs::msg::TransformStamped transform(double t,const std::string&parent,const std::string&child,Pose p,double z=0){geometry_msgs::msg::TransformStamped m;m.header.stamp=rclcpp::Time(ns(t));m.header.frame_id=parent;m.child_frame_id=child;m.transform.translation.x=p.x;m.transform.translation.y=p.y;m.transform.translation.z=z;m.transform.rotation=quat(p.yaw);return m;}
std::map<std::string,double> counters(const fs::path&p){std::ifstream f(p);std::map<std::string,double>r;std::string key;double val;while(f>>key>>val)r[key]=val;return r;}
fs::path cgroup_path(){std::ifstream f("/proc/self/cgroup");std::string l;std::getline(f,l);auto n=l.find("::");return n==std::string::npos?fs::path{}:fs::path("/sys/fs/cgroup")/l.substr(n+3);}
double scalar_file(const fs::path&p){std::ifstream f(p);double v=0;f>>v;return v;}
double rss_tree(pid_t pid){std::ifstream f("/proc/"+std::to_string(pid)+"/statm");double total=0,resident=0;f>>total>>resident;double bytes=resident*sysconf(_SC_PAGESIZE);std::ifstream c("/proc/"+std::to_string(pid)+"/task/"+std::to_string(pid)+"/children");pid_t child;while(c>>child)bytes+=rss_tree(child);return bytes;}
Json evaluate_map(const nav_msgs::msg::OccupancyGrid&map,const Json&arena,Pose initial,const fs::path&out){
  const double r=map.info.resolution;const auto o=obstacles(arena);std::vector<Pose>occupied;int occupied_count=0,correct=0;const double tolerance=1.5*r;
  auto origin=map.info.origin;double yaw=workbench::planar_yaw(origin.orientation);Pose map_origin{origin.position.x,origin.position.y,yaw};
  for(unsigned y=0;y<map.info.height;++y)for(unsigned x=0;x<map.info.width;++x)if(map.data[y*map.info.width+x]>=65){auto world=combine(initial,combine(map_origin,{(x+.5)*r,(y+.5)*r,0}));occupied.push_back(world);++occupied_count;if(clearance(world.x,world.y,o)<=tolerance)++correct;}
  int free_total=0,free_known=0,truth_edges=0,edges_found=0;
  auto cell=[&](Pose world){auto p=relative(map_origin,relative(initial,world));int x=floor(p.x/r),y=floor(p.y/r);return x<0||y<0||x>=int(map.info.width)||y>=int(map.info.height)?-1:int(map.data[y*map.info.width+x]);};
  // Full-arena free-space coverage, excluding solid obstacle interiors. This
  // denominator includes potentially occluded spaces and is not a task success flag.
  for(double y=.025;y<6.304;y+=.05)for(double x=.025;x<4.404;x+=.05)if(clearance(x,y,o)>.025){++free_total;int c=cell({x,y,0});if(c>=0&&c<25)++free_known;}
  for(const auto&b:o){int n=b.cylinder?36:std::max(8,int(ceil(4*(b.hx+b.hy)/.025)));for(int k=0;k<n;++k){double x,y;if(b.cylinder){x=b.x+b.hx*cos(2*pi*k/n);y=b.y+b.hy*sin(2*pi*k/n);}else{double s=double(k)/n*4;if(s<1){x=b.x-b.hx+2*b.hx*s;y=b.y-b.hy;}else if(s<2){x=b.x+b.hx;y=b.y-b.hy+2*b.hy*(s-1);}else if(s<3){x=b.x+b.hx-2*b.hx*(s-2);y=b.y+b.hy;}else{x=b.x-b.hx;y=b.y+b.hy-2*b.hy*(s-3);}}
    // Exclude surfaces hidden inside a touching/overlapping obstacle.
    double nx=0,ny=0;
    if(b.cylinder){nx=(x-b.x)/b.hx;ny=(y-b.y)/b.hx;}
    else {double s=double(k)/n*4;if(s<1)ny=-1;else if(s<2)nx=1;else if(s<3)ny=1;else nx=-1;}
    bool internal=false;for(const auto&other:o)if(&other!=&b){std::vector<Obstacle>one{other};if(clearance(x+nx*.0001,y+ny*.0001,one)<0){internal=true;break;}}if(internal)continue;++truth_edges;bool hit=false;for(const auto&p:occupied)if(hypot(p.x-x,p.y-y)<=tolerance){hit=true;break;}edges_found+=hit;}}
  std::ofstream pgm(out/"map.pgm",std::ios::binary);pgm<<"P5\n"<<map.info.width<<' '<<map.info.height<<"\n255\n";for(int y=int(map.info.height)-1;y>=0;--y)for(unsigned x=0;x<map.info.width;++x){int v=map.data[y*map.info.width+x];unsigned char p=v<0?205:(v>=65?0:(v<25?254:150));pgm.write(reinterpret_cast<char*>(&p),1);}
  std::ofstream yaml(out/"map.yaml");yaml<<"image: map.pgm\nmode: trinary\nresolution: "<<r<<"\norigin: ["<<origin.position.x<<", "<<origin.position.y<<", "<<yaw<<"]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n";
  std::ofstream svg(out/"map_comparison.svg");svg<<"<svg xmlns='http://www.w3.org/2000/svg' width='760' height='1000' viewBox='-.25 -.25 4.95 6.95'><rect x='-.25' y='-.25' width='4.95' height='6.95' fill='#f5f7fa'/><g transform='translate(0 6.304) scale(1 -1)'>";
  for(unsigned y=0;y<map.info.height;++y)for(unsigned x=0;x<map.info.width;++x){int v=map.data[y*map.info.width+x];if(v<0)continue;auto w=combine(initial,combine(map_origin,{x*r,y*r,0}));svg<<"<rect x='"<<w.x<<"' y='"<<w.y<<"' width='"<<r<<"' height='"<<r<<"' transform='rotate("<<(initial.yaw+yaw)*180/pi<<' '<<w.x<<' '<<w.y<<")' fill='"<<(v>=65?"#263f52":"#d9e5e6")<<"'/>";}
  for(const auto&b:o)svg<<"<rect x='"<<b.x-b.hx<<"' y='"<<b.y-b.hy<<"' width='"<<2*b.hx<<"' height='"<<2*b.hy<<"' fill='none' stroke='#f0803a' stroke-width='.009'/>";
  svg<<"</g></svg>\n";
  return {{"occupied_precision_within_1_5_cells",occupied_count?double(correct)/occupied_count:0},{"obstacle_surface_recall_within_1_5_cells",truth_edges?double(edges_found)/truth_edges:0},{"arena_free_space_coverage",free_total?double(free_known)/free_total:0},{"occupied_cells",occupied_count},{"map_width",map.info.width},{"map_height",map.info.height},{"tolerance_m",tolerance},{"alignment","Fixed initial-pose transform only; no post-hoc ICP alignment to ground truth"}};
}
int run(int argc,char**argv){
  auto a=args(argc,argv,1);const std::set<std::string>allowed{"--dataset","--output","--baseline","--rate","--delay-ms","--jitter-ms","--odom-delay-ms","--drop-every","--filter-neighbors","--scan-bins","--map-resolution","--limit-seconds","--profile","--realtime-tail","--loop-search-distance","--loop-closing","--lockstep","--domain"};for(const auto&kv:a)check(allowed.count(kv.first),"Unknown option "+kv.first);
  const bool lockstep=number(a,"--lockstep",0)!=0;int domain=int(number(a,"--domain",221));check(domain>=0&&domain<=232,"Invalid ROS domain");setenv("ROS_DOMAIN_ID",std::to_string(domain).c_str(),1);
  fs::path dataset=option(a,"--dataset"),out=option(a,"--output"),baseline=option(a,"--baseline");check(!out.empty()&&!fs::exists(out),"Replay output must be new");fs::create_directories(out);
  auto manifest=read_json(dataset/"manifest.json");check(manifest["schema"]==1&&manifest["sample_bytes"]==sizeof(Sample)&&manifest["scan_bytes"]==sizeof(ScanRecord),"Dataset schema mismatch");auto samples=read_binary<Sample>(dataset/"trajectory.bin");auto scans=read_binary<ScanRecord>(dataset/"scans.bin");check(samples.size()==manifest["sample_count"]&&scans.size()==manifest["scan_count"],"Dataset count mismatch");
  double rate=number(a,"--rate",10),delay=number(a,"--delay-ms",0)/1000,jitter=number(a,"--jitter-ms",0)/1000,od=number(a,"--odom-delay-ms",0)/1000,limit=number(a,"--limit-seconds",double(manifest["duration_s"])),resolution=number(a,"--map-resolution",.05);int drop=int(number(a,"--drop-every",0));
  check(rate>=.1&&rate<=30&&delay>=0&&delay<=1&&jitter>=0&&jitter<=.5&&od>=0&&od<=1&&limit>0&&resolution>=.01&&resolution<=.10&&drop>=0,"Invalid replay setting");limit=std::min(limit,double(manifest["duration_s"]));double tail=number(a,"--realtime-tail",0);check(tail>=0&&tail<=limit,"Invalid real-time tail");double tail_start=limit-tail;
  workbench::Options options;options.bins=int(number(a,"--scan-bins",1600));options.neighbors=int(number(a,"--filter-neighbors",0));options.max_linear=.10;options.max_angular=.5;options.validate();
  auto cfg=YAML::LoadFile(baseline.string());auto params=cfg["slam_toolbox"]["ros__parameters"];params["use_sim_time"]=true;params["resolution"]=resolution;
  if(a.count("--loop-search-distance")){double distance=number(a,"--loop-search-distance",3);check(distance>=.1&&distance<=5,"Invalid loop search distance");params["loop_search_maximum_distance"]=distance;}
  if(a.count("--loop-closing")){double enabled=number(a,"--loop-closing",1);check(enabled==0||enabled==1,"loop-closing must be 0 or 1");params["do_loop_closing"]=(enabled==1);}
  std::ofstream config(out/"slam.yaml");config<<cfg;config.close();
  Json settings={{"dataset",fs::absolute(dataset).string()},{"rate",rate},{"resource_claim",rate==1?"Real-time desktop constraint trial; not measured Jetson equivalence":"Accelerated geometry/SLAM trial; not a real-time resource-capacity claim"},{"communication_delay_s",delay},{"communication_jitter_s",jitter},{"odom_delay_s",od},{"drop_every",drop},{"profile",option(a,"--profile","unconstrained")},{"mapping_input","Front scan only; both lidars deskewed, merged and optionally filtered as diagnostic clouds"},{"filter_neighbors",options.neighbors},{"scan_bins",options.bins},{"map_resolution",resolution},{"limit_seconds",limit}};write_json(out/"settings.json",settings);
  auto cg=cgroup_path();Json budget={{"cgroup",cg.string()}};for(const std::string name:{"cpu.max","memory.max","memory.swap.max","cpuset.cpus.effective"})if(fs::exists(cg/name))budget[name]=read_text(cg/name);write_json(out/"resource_limits.json",budget);auto cpu_start=counters(cg/"cpu.stat");
  workbench::Processes children;
  if(lockstep)children.start({"slam",{(fs::absolute(argv[0]).parent_path()/"maze_mapper").string(),"--ros-args","-r","__node:=slam_toolbox","--params-file",fs::absolute(out/"slam.yaml").string()}},out);
  else children.start({"slam",{"ros2","launch","slam_toolbox","online_async_launch.py","use_sim_time:=true","slam_params_file:="+fs::absolute(out/"slam.yaml").string()}},out);
  int empty_argc=1;char*empty_argv[]={argv[0]};rclcpp::init(empty_argc,empty_argv);auto node=std::make_shared<rclcpp::Node>("maze_replay");rclcpp::executors::SingleThreadedExecutor exec;exec.add_node(node);
  auto clock=node->create_publisher<rosgraph_msgs::msg::Clock>("/clock",10);auto odom_pub=node->create_publisher<nav_msgs::msg::Odometry>("/odom",100);tf2_ros::TransformBroadcaster tf(*node);tf2_ros::StaticTransformBroadcaster static_tf(*node);
  static_tf.sendTransform(std::vector<geometry_msgs::msg::TransformStamped>{transform(0,"base_footprint","lidar_front_link",{.110,.005,pi},.0545),transform(0,"base_footprint","lidar_rear_link",{-.110,.005,0},.0545)});
  auto front=node->create_publisher<sensor_msgs::msg::LaserScan>("/lidar_front/scan_mapping",rclcpp::SensorDataQoS());auto rear=node->create_publisher<sensor_msgs::msg::LaserScan>("/lidar_rear/scan_mapping",rclcpp::SensorDataQoS());auto merged=node->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar/points_merged",rclcpp::SensorDataQoS());auto filtered=node->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar/points_filtered",rclcpp::SensorDataQoS());
  nav_msgs::msg::OccupancyGrid latest_map;Pose correction{};size_t pose_count=0,ack_count=0;int64_t last_ack=-1;double last_pose_stamp=-1,realtime_wall_origin=0;bool realtime_measurement=false;std::vector<double>pose_errors,processing_ms,tail_pose_latencies_ms;std::ofstream poses(out/"slam_poses.csv");poses<<"stamp_s,x,y,yaw,truth_x,truth_y,position_error_m\n";
  auto ack_sub=node->create_subscription<std_msgs::msg::Int64>("/maze/scan_callback_completed",100,[&](std_msgs::msg::Int64::ConstSharedPtr m){check(m->data>last_ack,"Regressing/duplicate mapper acknowledgement");last_ack=m->data;++ack_count;});
  Pose initial{manifest["initial_pose"][0],manifest["initial_pose"][1],manifest["initial_pose"][2]};
  auto maps=node->create_subscription<nav_msgs::msg::OccupancyGrid>("/map",rclcpp::QoS(1).transient_local(),[&](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m){latest_map=*m;});
  auto corrections=node->create_subscription<tf2_msgs::msg::TFMessage>("/tf",100,[&](tf2_msgs::msg::TFMessage::ConstSharedPtr msg){for(const auto&t:msg->transforms)if(t.header.frame_id=="map"&&t.child_frame_id=="odom")correction={t.transform.translation.x,t.transform.translation.y,workbench::planar_yaw(t.transform.rotation)};});
  auto pose_sub=node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/pose",100,[&](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr p){double t=rclcpp::Time(p->header.stamp).seconds()-epoch;if(t<=last_pose_stamp)return;last_pose_stamp=t;++pose_count;if(realtime_measurement&&t>=tail_start)tail_pose_latencies_ms.push_back(std::max(0.,workbench::steady_now()-realtime_wall_origin-(t-tail_start))*1000);Pose est{p->pose.pose.position.x,p->pose.pose.position.y,workbench::planar_yaw(p->pose.pose.orientation)};auto world=combine(initial,est),truth=sample_at(samples,t);double error=hypot(world.x-truth.x,world.y-truth.y);pose_errors.push_back(error);poses<<t<<','<<world.x<<','<<world.y<<','<<world.yaw<<','<<truth.x<<','<<truth.y<<','<<error<<'\n';});
  auto publish_clock=[&](double t){rosgraph_msgs::msg::Clock c;c.clock=rclcpp::Time(ns(t));clock->publish(c);};
  double startup=workbench::steady_now();while(front->get_subscription_count()==0){check(workbench::steady_now()-startup<40,"SLAM discovery timed out");check(!interrupted,"Interrupted");children.check();publish_clock(0);tf.sendTransform(transform(0,"odom","base_footprint",{}));exec.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(50));}
  // Settle simulated-clock activation before reasserting the static transforms.
  // A TF buffer can clear its initial cache when ROS time becomes active.
  for(int k=0;k<20;++k){publish_clock(0);tf.sendTransform(transform(0,"odom","base_footprint",{}));if(k==10)static_tf.sendTransform(std::vector<geometry_msgs::msg::TransformStamped>{transform(0,"base_footprint","lidar_front_link",{.110,.005,pi},.0545),transform(0,"base_footprint","lidar_rear_link",{-.110,.005,0},.0545)});exec.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(50));}
  size_t next_odom=0,next_scan=0,published=0,rejected=0,dropped=0,sparse=0,approx=0,kept=0,totalpoints=0;double maxage=0,max_lag=0,peakrss=0;bool stale_fault=false;std::deque<lidar_motion::Odom>history;std::mt19937 rng(123);std::uniform_real_distribution<double>uniform(-jitter,jitter);std::vector<double>deliveries(scans.size());for(size_t i=0;i<scans.size();++i)deliveries[i]=scans[i].t+.03+std::max(0.,delay+uniform(rng));
  std::ofstream metrics(out/"runtime.csv");metrics<<"wall_s,simulation_s,tree_rss_bytes,cgroup_memory_bytes,scheduler_lag_s,published_pairs,stale_rejections\n";
  check(!lockstep||tail==0,"Lockstep cannot be used for a real-time resource tail");
  settings["realtime_tail_seconds"]=tail;settings["lockstep"]=lockstep;settings["domain"]=domain;
  settings["base_delivery_delay_s"]=.03;settings["delivery_tick_s"]=.02;
  settings["communication_delay_is_additional"]=true;
  settings["delivery_model_status"]="Proposed transport sensitivity model, not measured serial/Jetson timing; acquisition timestamps are unchanged";
  if(lockstep)settings["resource_claim"]="Deterministic callback-completed geometry replay using the installed async SLAM algorithm; clock waits for every scan; not a throughput/capacity test";
  settings["loop_search_distance_m"]=params["loop_search_maximum_distance"].as<double>();settings["loop_closing"]=params["do_loop_closing"].as<bool>();
  write_json(out/"settings.json",settings);
  const double start=workbench::steady_now();double report=start,resource=start,t=0,tail_wall=0,tail_lag=0,tail_cpu=0,tail_throttle=0,tail_maxage=0,peak_memory=0;
  for(size_t tick=0;!interrupted;++tick){t=tick*.02;if(t>limit+.5)break;
    if(tail>0&&t>=tail_start&&tail_wall==0){tail_wall=workbench::steady_now();realtime_wall_origin=tail_wall;realtime_measurement=true;auto c=counters(cg/"cpu.stat");tail_cpu=c["usage_usec"];tail_throttle=c["throttled_usec"];}
    // Restart the real-time clock after accelerated priming. Priming overruns
    // must not become a catch-up burst in the measured real-time window.
    double deadline=tail_wall>0?tail_wall+(t-tail_start):start+t/rate;
    std::this_thread::sleep_until(std::chrono::steady_clock::time_point(std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(deadline))));double now=workbench::steady_now();max_lag=std::max(max_lag,now-deadline);if(tail_wall>0)tail_lag=std::max(tail_lag,now-deadline);publish_clock(t);
    while(next_odom<samples.size()&&samples[next_odom].t+od<=t+1e-8){const auto&s=samples[next_odom++];history.push_back({ns(s.t),{s.odom.x,s.odom.y,s.odom.yaw}});while(history.size()>100)history.pop_front();nav_msgs::msg::Odometry m;m.header.stamp=rclcpp::Time(ns(s.t));m.header.frame_id="odom";m.child_frame_id="base_footprint";m.pose.pose.position.x=s.odom.x;m.pose.pose.position.y=s.odom.y;m.pose.pose.orientation=quat(s.odom.yaw);odom_pub->publish(m);tf.sendTransform(transform(s.t,"odom","base_footprint",s.odom));}
    exec.spin_some();
    while(next_scan<scans.size()&&scans[next_scan].t<=limit&&deliveries[next_scan]<=t+1e-8){const auto&scan=scans[next_scan];double age=t-scan.t;maxage=std::max(maxage,age);if(tail_wall>0)tail_maxage=std::max(tail_maxage,age);if(age>options.max_age){++rejected;++next_scan;stale_fault=true;continue;}if(history.size()<2||history.back().stamp_ns<ns(scan.t))break;if(drop>0&&(next_scan+1)%drop==0){++dropped;++next_scan;continue;}
      auto before=workbench::steady_now();std::vector<workbench::Ray>rays;bool pair_ok=true;
      for(int side=0;side<2;++side){lidar_mapping::TimedFrame frame{ns(scan.t-.09975),ns(scan.t),{}};for(int k=0;k<400;++k){float range=scan.ranges[side*400+k];if(range>=.05&&range<=12)frame.beams.push_back({{k,k,range,-2*pi*k/400.},ns(scan.t-(399-k)*.00025)});}
        auto lo=std::lower_bound(history.begin(),history.end(),frame.first,[](const auto&s,int64_t n){return s.stamp_ns<n;});if(lo!=history.begin())--lo;auto hi=std::lower_bound(history.begin(),history.end(),frame.reference,[](const auto&s,int64_t n){return s.stamp_ns<n;});check(lo!=history.end()&&hi!=history.end(),"Missing odometry bracket");auto prepared=workbench::prepare(frame,{lo,std::next(hi)},side,options);approx+=prepared.scan.approximation_rejected;
        size_t valid=0;sensor_msgs::msg::LaserScan m;m.header.stamp=rclcpp::Time(frame.reference);m.header.frame_id=side==0?"lidar_front_link":"lidar_rear_link";m.angle_min=0;m.angle_increment=2*pi/options.bins;m.angle_max=(options.bins-1)*m.angle_increment;m.time_increment=0;m.scan_time=.1;m.range_min=.05;m.range_max=12;for(const auto&b:prepared.scan.bins){m.ranges.push_back(b.range);valid+=b.input_bin>=0;}if(valid<30){++sparse;pair_ok=false;}else(side==0?front:rear)->publish(m);
        for(const auto&r:prepared.corrected)rays.push_back(workbench::transform(r,workbench::mount(side),0));
      }
      if(pair_ok){auto f=workbench::spatial_filter(rays,options);merged->publish(workbench::cloud(rays,ns(scan.t),"base_footprint",.0545));filtered->publish(workbench::cloud(f.kept,ns(scan.t),"base_footprint",.0545));totalpoints+=rays.size();kept+=f.kept.size();++published;}
      processing_ms.push_back((workbench::steady_now()-before)*1000);
      if(lockstep&&pair_ok){double until=workbench::steady_now()+10;while(last_ack<ns(scan.t)){check(workbench::steady_now()<until,"Mapper acknowledgement timed out");check(!interrupted,"Interrupted");children.check();exec.spin_some();std::this_thread::sleep_for(std::chrono::microseconds(200));}}
      ++next_scan;exec.spin_some();
    }
    exec.spin_some();children.check();
    if(now-resource>=1){
      double rss=rss_tree(getpid()),memory=scalar_file(cg/"memory.current");
      peakrss=std::max(peakrss,rss);peak_memory=std::max(peak_memory,memory);
      metrics<<now-start<<','<<t<<','<<rss<<','<<memory<<','<<now-deadline<<','<<published<<','<<rejected<<'\n';resource=now;
    }
    if(now-report>=20){std::cout<<"replay "<<int(t)<<" / "<<int(limit)<<" simulation seconds; pairs="<<published<<" map="<<latest_map.info.width<<'x'<<latest_map.info.height<<" processed poses="<<pose_count<<std::endl;report=now;}
  }
  check(!interrupted,"Interrupted");double playback_wall=workbench::steady_now()-start;
  const double tail_window_wall=tail_wall>0?workbench::steady_now()-tail_wall:0;
  const auto tail_window_cpu=counters(cg/"cpu.stat");
  realtime_measurement=false;
  for(int k=0;k<100;++k){publish_clock(t+k*.1);tf.sendTransform(transform(t+k*.1,"odom","base_footprint",samples[std::min(next_odom,samples.size())-1].odom));exec.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(20));}
  if(latest_map.data.empty()){
    write_json(out/"results.json",{{"quality",nullptr},{"freshness_pass",!stale_fault},{"pairs_published",published},{"stale_rejections",rejected},{"max_simulated_scan_age_s",maxage},{"result","No occupancy map; failed/negative trial, not a usable map"}});
    children.stop();rclcpp::shutdown();return 3;
  }
  auto arena=read_json(fs::path(manifest["scene"])/"arena.json");auto quality=evaluate_map(latest_map,arena,initial,out);auto end_truth=sample_at(samples,std::min(t,limit)),end_odom=sample_at(samples,std::min(t,limit),true),end_est=combine(initial,combine(correction,end_odom));quality["endpoint_position_error_m"]=hypot(end_est.x-end_truth.x,end_est.y-end_truth.y);quality["endpoint_yaw_error_rad"]=std::abs(wrap(end_est.yaw-end_truth.yaw));double sum=0;for(double e:pose_errors)sum+=e*e;quality["sampled_pose_rmse_m"]=pose_errors.empty()?Json(nullptr):Json(sqrt(sum/pose_errors.size()));quality["pose_messages"]=pose_count;
  auto serialize=node->create_client<slam_toolbox::srv::SerializePoseGraph>("/slam_toolbox/serialize_map");check(serialize->wait_for_service(std::chrono::seconds(5)),"Missing graph serialization service");auto request=std::make_shared<slam_toolbox::srv::SerializePoseGraph::Request>();request->filename=fs::absolute(out/"map_graph").string();auto future=serialize->async_send_request(request);check(exec.spin_until_future_complete(future,std::chrono::seconds(20))==rclcpp::FutureReturnCode::SUCCESS&&future.get()->result==0,"Graph save failed");
  auto cpu_end=counters(cg/"cpu.stat");double cpu_seconds=(cpu_end["usage_usec"]-cpu_start["usage_usec"])/1e6;
  std::sort(processing_ms.begin(),processing_ms.end());
  Json results={{"quality",quality},{"pairs_published",published},{"scan_frames_available",next_scan},{"stale_rejections",rejected},{"intentional_drops",dropped},{"sparse_sensor_frames",sparse},{"projection_rejected_rays",approx},{"merged_points",totalpoints},{"filtered_points",kept},{"max_simulated_scan_age_s",maxage},{"freshness_pass",!stale_fault},{"playback_wall_s",playback_wall},{"simulation_s",limit},{"achieved_sim_seconds_per_wall_second",limit/playback_wall},{"max_wall_scheduler_lag_s",max_lag},{"peak_process_tree_rss_bytes",peakrss},{"cgroup_cpu_seconds_including_startup",cpu_seconds},{"cgroup_throttled_seconds",(cpu_end["throttled_usec"]-cpu_start["throttled_usec"])/1e6},{"processing_p95_ms",processing_ms.empty()?0:processing_ms[std::min(processing_ms.size()-1,size_t(.95*processing_ms.size()))]},{"resource_scope",budget},{"gpu_compute_usage","SLAM/deskew/fusion are CPU-only; simulator/rendering excluded. No CUDA/TensorRT workload or Jetson VRAM performance emulated."}};
  results["peak_cgroup_memory_bytes"]=std::max(peak_memory,scalar_file(cg/"memory.peak"));
  {std::istringstream status(read_text("/proc/self/status"));std::string line;while(std::getline(status,line))if(line.rfind("Cpus_allowed_list:",0)==0)results["cpu_affinity_evidence"]=line;}
  if(tail>0){results["realtime_tail"]={{"requested_seconds",tail},{"wall_seconds",tail_window_wall},{"maximum_scheduler_lag_seconds",tail_lag},{"cpu_seconds",(tail_window_cpu.at("usage_usec")-tail_cpu)/1e6},{"throttled_seconds",(tail_window_cpu.at("throttled_usec")-tail_throttle)/1e6},{"maximum_simulated_scan_age_s",tail_maxage},{"conservative_scan_age_bound_s",tail_maxage+tail_lag},{"scheduling_pass",tail_lag<.1&&tail_maxage+tail_lag<.25},{"scope","Real-time playback window only; excludes priming, final drain and graph serialization"}};}
  if(tail>0){std::sort(tail_pose_latencies_ms.begin(),tail_pose_latencies_ms.end());auto&measurement=results["realtime_tail"];measurement["accepted_pose_samples"]=tail_pose_latencies_ms.size();measurement["accepted_pose_latency_p95_ms"]=tail_pose_latencies_ms.empty()?Json(nullptr):Json(tail_pose_latencies_ms[std::min(tail_pose_latencies_ms.size()-1,size_t(.95*tail_pose_latencies_ms.size()))]);measurement["accepted_pose_latency_max_ms"]=tail_pose_latencies_ms.empty()?Json(nullptr):Json(tail_pose_latencies_ms.back());measurement["latency_scope"]="Scheduled synthetic scan reference to receipt of SLAM pose for accepted scans only; not occupancy-map publication latency or loss accounting";}
  results["realtime_scheduling_pass"]=rate==1&&!lockstep?Json(max_lag<.1&&maxage+max_lag<.25):Json(nullptr);
  results["mapper_callbacks_acknowledged"]=lockstep?Json(ack_count):Json(nullptr);
  write_json(out/"results.json",results);std::cout<<results.dump(2)<<'\n';children.stop();rclcpp::shutdown();return 0;
}
void rescore(const fs::path&out){
  auto results=read_json(out/"results.json");if(results["quality"].is_null())return;
  auto settings=read_json(out/"settings.json"),manifest=read_json(fs::path(settings["dataset"])/"manifest.json");
  auto y=YAML::LoadFile((out/"map.yaml").string());std::ifstream f(out/"map.pgm",std::ios::binary);std::string magic;unsigned width,height,maxval;f>>magic>>width>>height>>maxval;check(magic=="P5"&&maxval==255,"Unsupported saved map");f.get();std::vector<unsigned char>pixels(width*height);f.read(reinterpret_cast<char*>(pixels.data()),pixels.size());check(bool(f),"Truncated map");
  nav_msgs::msg::OccupancyGrid map;map.info.width=width;map.info.height=height;map.info.resolution=y["resolution"].as<float>();map.info.origin.position.x=y["origin"][0].as<double>();map.info.origin.position.y=y["origin"][1].as<double>();map.info.origin.orientation=quat(y["origin"][2].as<double>());map.data.resize(width*height);
  for(unsigned row=0;row<height;++row)for(unsigned col=0;col<width;++col){auto p=pixels[row*width+col];map.data[(height-1-row)*width+col]=p==0?100:(p==254?0:(p==205?-1:50));}
  Pose initial{manifest["initial_pose"][0],manifest["initial_pose"][1],manifest["initial_pose"][2]};auto quality=evaluate_map(map,read_json(fs::path(manifest["scene"])/"arena.json"),initial,out);
  for(auto it=quality.begin();it!=quality.end();++it)results["quality"][it.key()]=it.value();
  results["geometry_metric_version"]=2;results["geometry_metric_note"]="Only exposed union surfaces are sampled; shared central-box seams are excluded. Fixed initial-pose alignment; no fitting.";
  write_json(out/"results.json",results);
}
int main(int argc,char**argv){signal(SIGINT,stop_signal);signal(SIGTERM,stop_signal);try{if(argc==3&&std::string(argv[1])=="--rescore"){rescore(argv[2]);return 0;}return run(argc,argv);}catch(const std::exception&e){std::cerr<<"maze_replay: "<<e.what()<<'\n';return 1;}}
