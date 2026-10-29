// Offline review of existing corrected scans. No ROS node, publishing or hardware.
#include "motion_model.hpp"
#include "mapping_trial_core.hpp"
#include <rosbag2_cpp/reader.hpp>
#include <rclcpp/serialization.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <array>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>

namespace fs = std::filesystem;
using namespace lidar_motion;
using mapping_trial::require;
using Scan = sensor_msgs::msg::LaserScan;
using Map = nav_msgs::msg::OccupancyGrid;
using Cmd = geometry_msgs::msg::TwistStamped;
struct SavedScan { int64_t received; Scan scan; };
struct SavedTF { int64_t received, stamp; Pose pose; };
struct Segment { Point a, b; };

template<class M> M decode(const std::shared_ptr<rosbag2_storage::SerializedBagMessage> &record) {
  rclcpp::SerializedMessage bytes(*record->serialized_data);
  M m; rclcpp::Serialization<M>().deserialize_message(&bytes, &m); return m;
}
int64_t stamp(const builtin_interfaces::msg::Time &t) { return int64_t(t.sec) * 1000000000 + t.nanosec; }
double yaw(const geometry_msgs::msg::Quaternion &q) {
  require(std::abs(q.x) < .001 && std::abs(q.y) < .001 &&
    std::abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1) < .001, "Invalid planar quaternion");
  return std::atan2(2*q.w*q.z, 1-2*q.z*q.z);
}
double quantile(std::vector<double> v, double f) {
  require(!v.empty(), "Empty statistic"); std::sort(v.begin(), v.end());
  return v[std::min(v.size()-1, size_t(f*(v.size()-1)))];
}
double rms(const std::vector<double> &v) {
  require(!v.empty(), "Empty RMS"); double sum = 0; for (double x : v) sum += x*x;
  return std::sqrt(sum / v.size());
}
void stats(std::ostream &out, const std::string &name, const std::vector<double> &v) {
  out << name << " n=" << v.size();
  if (!v.empty()) out << " min=" << *std::min_element(v.begin(), v.end()) << " median=" << quantile(v,.5)
    << " p95=" << quantile(v,.95) << " max=" << *std::max_element(v.begin(),v.end()) << " rms=" << rms(v);
  out << '\n';
}
double distance(Point p, Segment s) {
  const double x=s.b.x-s.a.x, y=s.b.y-s.a.y, norm=x*x+y*y;
  const double f=norm>0 ? std::clamp(((p.x-s.a.x)*x+(p.y-s.a.y)*y)/norm,0.,1.) : 0.;
  return std::hypot(p.x-s.a.x-f*x,p.y-s.a.y-f*y);
}
void save_map(const fs::path &dir, size_t id, const Map &m) {
  require(m.header.frame_id=="map" && m.info.width*m.info.height==m.data.size() && m.info.resolution>0, "Invalid map");
  std::ofstream pgm(dir/("map_"+std::to_string(id)+".pgm"),std::ios::binary);
  pgm << "P5\n" << m.info.width << ' ' << m.info.height << "\n255\n";
  for (int y=int(m.info.height)-1;y>=0;--y) for (size_t x=0;x<m.info.width;++x) {
    const int v=m.data[size_t(y)*m.info.width+x]; require(v==-1 || v==0 || v==100,"Non-trinary map requires explicit threshold policy");
    pgm.put(char(v<0 ? 205 : (v==100 ? 0 : 254)));
  }
  pgm.close(); require(bool(pgm),"Map output failed");
}

int main(int argc,char **argv) {
  try {
    require(argc==3||(argc==4&&std::string(argv[3])=="--reverse-return"),
      "Usage: review_mapping_motion BAG_DIRECTORY NEW_OUTPUT_DIRECTORY [--reverse-return]");
    const bool route=argc==4;
    const fs::path out=argv[2]; require(fs::create_directory(out),"Output directory must be new");
    std::ofstream report(out/"review.txt"), tf_csv(out/"map_tf.csv"), cmd_csv(out/"commands.csv"),
      maps_csv(out/"maps.csv"), status_csv(out/"status.csv"), odom_csv(out/"odometry.csv");
    for (auto *f : {&report,&tf_csv,&cmd_csv,&maps_csv,&status_csv,&odom_csv}) *f << std::setprecision(17);
    tf_csv << "received_ns,header_ns,x_m,y_m,yaw_rad\n";
    cmd_csv << "received_ns,header_ns,forward_m_s,yaw_rad_s\n";
    maps_csv << "id,received_ns,header_ns,width,height,resolution,origin_x,origin_y,occupied,free,unknown\n";
    status_csv << "received_ns,status\n"; odom_csv << "stamp_ns,x_m,y_m,yaw_rad\n";
    std::vector<Odom> odom; std::vector<SavedScan> scans; std::vector<SavedTF> transforms;
    std::vector<Cmd> commands; std::vector<Map> maps; std::map<std::string,size_t> counts;
    rosbag2_cpp::Reader reader; reader.open(argv[1]);
    int64_t previous_status=-1, rejected=-1;
    while(reader.has_next()) {
      auto record=reader.read_next(); const auto time=record->recv_timestamp;
      require(++counts[record->topic_name]<=100000,"Record bound exceeded");
      if(record->topic_name=="/odom") {
        const auto m=decode<nav_msgs::msg::Odometry>(record);
        require(m.header.frame_id=="odom" && m.child_frame_id=="base_footprint","Odometry frames");
        odom.push_back({stamp(m.header.stamp),{m.pose.pose.position.x,m.pose.pose.position.y,yaw(m.pose.pose.orientation)}});
        const auto &p=odom.back(); odom_csv<<p.stamp_ns<<','<<p.pose.x<<','<<p.pose.y<<','<<p.pose.yaw<<'\n';
      } else if(record->topic_name=="/lidar_front/scan_mapping") scans.push_back({time,decode<Scan>(record)});
      else if(record->topic_name=="/cmd_vel") {
        auto m=decode<Cmd>(record); commands.push_back(m);
        cmd_csv<<time<<','<<stamp(m.header.stamp)<<','<<m.twist.linear.x<<','<<m.twist.angular.z<<'\n';
      } else if(record->topic_name=="/tf") {
        const auto m=decode<tf2_msgs::msg::TFMessage>(record);
        for(const auto &t:m.transforms) if(t.header.frame_id=="map" && t.child_frame_id=="odom") {
          const Pose p{t.transform.translation.x,t.transform.translation.y,yaw(t.transform.rotation)};
          require(finite(p),"Nonfinite map TF"); transforms.push_back({time,stamp(t.header.stamp),p});
          tf_csv<<time<<','<<stamp(t.header.stamp)<<','<<p.x<<','<<p.y<<','<<p.yaw<<'\n';
        }
      } else if(record->topic_name=="/map") {
        const auto m=decode<Map>(record); size_t occupied=0,free=0,unknown=0;
        for(int8_t v:m.data) { occupied+=v==100; free+=v==0; unknown+=v<0; }
        maps_csv<<maps.size()<<','<<time<<','<<stamp(m.header.stamp)<<','<<m.info.width<<','<<m.info.height<<','<<m.info.resolution
          <<','<<m.info.origin.position.x<<','<<m.info.origin.position.y<<','<<occupied<<','<<free<<','<<unknown<<'\n';
        save_map(out,maps.size(),m); maps.push_back(m);
      } else if(record->topic_name=="/lidar_front/mapping_status") {
        const auto m=decode<std_msgs::msg::String>(record); const auto s=mapping_trial::status(m.data);
        require(s.published>=previous_status,"Regressing status");
        if(rejected>=0) require(rejected==s.rejected,"Rejections changed during capture");
        previous_status=s.published; rejected=s.rejected; status_csv<<time<<','<<m.data<<'\n';
      }
    }
    require(counts.size()==7 && !scans.empty() && !transforms.empty() && maps.size()>=2,"Missing streams");
    const Trajectory trajectory(odom,40000000); const Pose mount{.110,.005,std::acos(-1.)};
    for(const auto &c:counts) report<<"topic "<<c.first<<" count="<<c.second<<'\n';
    require(!commands.empty(),"No commands");
    struct Interval {size_t begin,zero,end;};
    std::vector<Interval> intervals;
    if(route){
      size_t i=0;
      for(double velocity:{-.025,.025}){
        const size_t begin=i;
        while(i<commands.size()&&commands[i].twist.linear.x==velocity)++i;
        const size_t zero=i;
        require(zero-begin>=10&&zero-begin<=40,"Route leg must have 10 to 40 commands");
        for(size_t j=0;j<5;++j){
          require(i<commands.size()&&commands[i].twist.linear.x==0,"Route requires five zeros per leg");++i;
        }
        intervals.push_back({begin,zero,i});
      }
      require(i==commands.size(),"Extra route commands or retry");
      require((stamp(commands[intervals[1].begin].header.stamp)-stamp(commands[intervals[0].zero].header.stamp))/1e9>=5.4,
        "Missing stationary dwell between route legs");
      report<<"motion=reverse_return\n";
    }else{
      const auto motion=commands.front().twist.angular.z==.10?mapping_trial::Motion::rotation:mapping_trial::Motion::forward;
      const auto pulse=mapping_trial::pulse(motion);
      require(commands.size()==pulse.messages+5,"Expected one complete pulse and five zeros");
      for(size_t i=0;i<commands.size();++i)
        require(commands[i].twist.linear.x==(i<pulse.messages?pulse.forward:0)&&
          commands[i].twist.angular.z==(i<pulse.messages?pulse.yaw:0),"Command sequence mismatch");
      intervals.push_back({0,pulse.messages,commands.size()});report<<"motion="<<pulse.name<<'\n';
    }
    for(size_t i=0;i<commands.size();++i){
      const auto &c=commands[i];require(c.header.frame_id=="base_footprint"&&c.twist.linear.y==0&&c.twist.linear.z==0&&
        c.twist.angular.x==0&&c.twist.angular.y==0&&(!route||c.twist.angular.z==0),"Command components/frames mismatch");
      if(i)require(stamp(c.header.stamp)>stamp(commands[i-1].header.stamp),"Command stamps regressed");
    }
    for(size_t leg=0;leg<intervals.size();++leg){
      const auto interval=intervals[leg];std::vector<double> command_gaps;
      for(size_t i=interval.begin+1;i<interval.end;++i){
        const double gap=(stamp(commands[i].header.stamp)-stamp(commands[i-1].header.stamp))/1e6;
        if(route)require(gap<150,"Route command gap exceeded 150 ms");
        command_gaps.push_back(gap);
      }
      const double duration=(stamp(commands[interval.zero].header.stamp)-stamp(commands[interval.begin].header.stamp))/1e9;
      if(route)require(duration<4.3,"Route leg deadline exceeded");
      stats(report,"leg_"+std::to_string(leg)+"_command_gap_ms",command_gaps);
      report<<"leg="<<leg<<" commands="<<interval.zero-interval.begin<<" until_first_zero_s="<<duration<<'\n';
    }
    const int64_t first_command=stamp(commands.front().header.stamp);
    const int64_t reverse_zero=stamp(commands[intervals.front().zero].header.stamp);
    const int64_t return_start=stamp(commands[intervals.back().begin].header.stamp);
    const int64_t last_zero=stamp(commands[intervals.back().zero].header.stamp);
    int64_t midpoint_capture=0,revisit_capture=0;
    if(route){
      std::ifstream events(fs::path(argv[1]).parent_path()/"events.csv");
      require(bool(events),"Route review requires sibling events.csv");
      std::string line;std::getline(events,line);require(line=="stamp_ns,event","Unexpected event header");
      int64_t previous=0,finished=0;
      while(std::getline(events,line)){
        const auto comma=line.find(',');require(comma!=std::string::npos,"Invalid event row");
        const auto t=std::stoll(line.substr(0,comma));const auto name=line.substr(comma+1);
        require(t>previous,"Regressing events");previous=t;
        if(name=="reverse_stationary_capture"){require(!midpoint_capture,"Duplicate midpoint event");midpoint_capture=t;}
        if(name=="revisit_stationary_capture"){require(!revisit_capture,"Duplicate revisit event");revisit_capture=t;}
        if(name=="software_checks_passed")finished=t;
      }
      require(midpoint_capture>reverse_zero&&return_start-midpoint_capture>=5000000000LL&&
        revisit_capture>last_zero&&finished-revisit_capture>=5000000000LL,"Missing route stationary captures");
      report<<"midpoint_capture_ns="<<midpoint_capture<<" revisit_capture_ns="<<revisit_capture<<'\n';
    }
    std::vector<double> gaps,speeds,yaw_rates; int64_t move_start=-1,move_end=-1;
    double max_lateral=0,max_yaw=0,max_distance=0;
    for(size_t i=1;i<odom.size();++i) {
      const auto &a=odom[i-1],&b=odom[i]; const double dt=(b.stamp_ns-a.stamp_ns)/1e9;
      const double d=std::hypot(b.pose.x-a.pose.x,b.pose.y-a.pose.y), angle=std::abs(wrap(b.pose.yaw-a.pose.yaw));
      gaps.push_back(dt*1000); speeds.push_back(d/dt); yaw_rates.push_back(angle/dt);
      if(d>1e-7 || angle>1e-7) { if(move_start<0)move_start=a.stamp_ns; move_end=b.stamp_ns; }
      const Point relative=inverse(odom.front().pose,{b.pose.x,b.pose.y});
      max_lateral=std::max(max_lateral,std::abs(relative.y)); max_distance=std::max(max_distance,std::hypot(relative.x,relative.y));
      max_yaw=std::max(max_yaw,std::abs(wrap(b.pose.yaw-odom.front().pose.yaw)));
    }
    require(move_start>=0,"No motion observed");
    stats(report,"odom_gap_ms",gaps); stats(report,"pose_speed_m_s",speeds); stats(report,"pose_yaw_rate_rad_s",yaw_rates);
    report<<"motion_start_ns="<<move_start<<" motion_end_ns="<<move_end
      <<" final_constant_pose_s="<<(odom.back().stamp_ns-move_end)/1e9<<" zero_to_last_pose_change_s="<<(move_end-last_zero)/1e9<<'\n';
    report<<"max_displacement_m="<<max_distance<<" max_lateral_m="<<max_lateral<<" max_yaw_rad="<<max_yaw<<'\n';
    const Pose first_tf=transforms.front().pose,last_tf=transforms.back().pose;
    std::vector<double> tf_steps,tf_yaw_steps; size_t tf_changes=0;
    for(size_t i=1;i<transforms.size();++i) {
      const auto &a=transforms[i-1].pose,&b=transforms[i].pose;
      const double d=std::hypot(b.x-a.x,b.y-a.y), angle=std::abs(wrap(b.yaw-a.yaw));
      if(d>1e-9 || angle>1e-9) { ++tf_changes; tf_steps.push_back(d); tf_yaw_steps.push_back(angle); }
    }
    report<<"map_tf_changes="<<tf_changes<<" dx="<<last_tf.x-first_tf.x<<" dy="<<last_tf.y-first_tf.y
      <<" dyaw="<<wrap(last_tf.yaw-first_tf.yaw)<<'\n';
    stats(report,"map_tf_changed_step_m",tf_steps); stats(report,"map_tf_changed_step_rad",tf_yaw_steps);
    // Endpoint-only consistency under the existing estimated reference stamp.
    // No second deskew, sensor-delay fit, odometry fit or ICP is performed.
    std::vector<std::map<size_t,Point>> points(scans.size());
    std::vector<int> groups(scans.size()); std::vector<double> ages,scan_gaps,finite_counts;
    std::ofstream endpoints(out/"endpoints.csv"); endpoints<<std::setprecision(17)<<"scan_id,stamp_ns,phase,bin,odom_x,odom_y,final_map_x,final_map_y\n";
    size_t skipped=0,finite_total=0;
    for(size_t i=0;i<scans.size();++i) {
      const auto &m=scans[i].scan; const auto t=stamp(m.header.stamp);
      require(m.header.frame_id=="lidar_front_link" && m.ranges.size()==1600 && m.time_increment==0 &&
        std::abs(m.angle_increment-2*std::acos(-1.)/1600)<1e-8,"Scan contract mismatch");
      ages.push_back((scans[i].received-t)/1e6);
      if(i) { require(t>stamp(scans[i-1].scan.header.stamp),"Scan stamps regressed"); scan_gaps.push_back((t-stamp(scans[i-1].scan.header.stamp))/1e6); }
      groups[i]=route?(t<first_command-200000000?0:(t<midpoint_capture?1:
        (t<return_start-200000000?2:(t>=revisit_capture?4:3)))):
        (t<move_start-200000000?0:(t>move_end+300000000?2:1));
      if(t<odom.front().stamp_ns || t>odom.back().stamp_ns) {
        if(route)require(t<first_command-200000000||t>move_end+300000000,"Unbracketed route motion scan");
        ++skipped; continue;
      }
      const Pose pose=trajectory.at(t);
      for(size_t b=0;b<m.ranges.size();++b) {
        const double r=m.ranges[b]; require(!std::isinf(r),"Infinite scan range"); if(std::isnan(r))continue;
        require(r>=m.range_min && r<=m.range_max,"Out of range endpoint");
        const double a=double(m.angle_min)+b*double(m.angle_increment);
        const auto p=apply(pose,apply(mount,{r*std::cos(a),r*std::sin(a)})); points[i][b]=p;
        const auto final_map=apply(last_tf,p);
        endpoints<<i<<','<<t<<','<<groups[i]<<','<<b<<','<<p.x<<','<<p.y<<','<<final_map.x<<','<<final_map.y<<'\n';
      }
      finite_total+=points[i].size(); finite_counts.push_back(points[i].size());
    }
    stats(report,"scan_reference_gap_ms",scan_gaps); stats(report,"scan_reference_to_receipt_ms",ages);
    stats(report,"finite_returns_per_scan",finite_counts); report<<"finite_total="<<finite_total<<" scans_skipped_no_odom_bracket="<<skipped<<'\n';
    std::array<std::vector<double>,1600> xs,ys; size_t trained=0,last_train=0;
    for(size_t i=0;i<points.size() && trained<15;++i) {
      if(points[i].empty())continue;
      require(groups[i]==0,"Training overlaps movement");
      for(const auto &b:points[i]) { xs[b.first].push_back(b.second.x); ys[b.first].push_back(b.second.y); }
      ++trained; last_train=i;
    }
    require(trained==15,"Insufficient reference scans");
    std::map<size_t,Point> stable;
    for(size_t i=0;i<1600;++i) if(xs[i].size()>=12) {
      const Point p{quantile(xs[i],.5),quantile(ys[i],.5)}; std::vector<double> deviations;
      for(size_t j=0;j<xs[i].size();++j) deviations.push_back(std::hypot(xs[i][j]-p.x,ys[i][j]-p.y));
      if(quantile(deviations,.5)<.010)stable[i]=p;
    }
    std::vector<Segment> segments;
    for(auto a=stable.begin();a!=stable.end();++a) {
      const auto b=std::next(a); if(b==stable.end())break;
      const double d=std::hypot(b->second.x-a->second.x,b->second.y-a->second.y);
      if(b->first-a->first<=8 && d>.002 && d<.080)segments.push_back({a->second,b->second});
    }
    report<<"same_bin_reference_stable_bins="<<stable.size()<<" same_bin_segments="<<segments.size()<<'\n';
    // Sparse resampling can change the occupied grid bin between scans. Build
    // geometric correspondences to the first scan instead of requiring an
    // identical output-bin index in 12/15 scans. Thresholds are fixed, with no
    // pose fitting or selection based on the motion/post residuals.
    const size_t seed=size_t(std::find_if(points.begin(),points.end(),[](const auto &p){return !p.empty();})-points.begin());
    stable.clear(); segments.clear();
    for(const auto &entry:points[seed]) {
      std::vector<double> x,y;
      for(size_t i=seed;i<=last_train;++i) {
        double best=.020; Point match{}; bool found=false;
        for(const auto &candidate:points[i]) {
          const double d=std::hypot(candidate.second.x-entry.second.x,candidate.second.y-entry.second.y);
          if(d<best) { best=d; match=candidate.second; found=true; }
        }
        if(found) { x.push_back(match.x); y.push_back(match.y); }
      }
      if(x.size()<12)continue;
      const Point p{quantile(x,.5),quantile(y,.5)}; std::vector<double> deviations;
      for(size_t i=0;i<x.size();++i)deviations.push_back(std::hypot(x[i]-p.x,y[i]-p.y));
      if(quantile(deviations,.5)<.010)stable[entry.first]=p;
    }
    for(auto a=stable.begin();a!=stable.end();++a) {
      const auto b=std::next(a); if(b==stable.end())break;
      const double d=std::hypot(b->second.x-a->second.x,b->second.y-a->second.y);
      if(b->first-a->first<=8 && d>.002 && d<.080)segments.push_back({a->second,b->second});
    }
    require(!segments.empty(),"No geometric fixed-scene segments");
    std::array<std::vector<double>,5> residuals; std::array<size_t,5> totals{},phase_scans{};
    const int post_group=route?4:2;
    std::vector<double> post_slam;
    for(size_t i=last_train+1;i<points.size();++i) {
      const int group=groups[i]; ++phase_scans[group];
      for(const auto &entry:points[i]) {
        const Point p=entry.second; ++totals[group]; size_t best=0; double d=distance(p,segments[0]);
        for(size_t k=1;k<segments.size();++k)if(distance(p,segments[k])<d) { d=distance(p,segments[k]); best=k; }
        if(d>.05)continue;
        residuals[group].push_back(d*1000);
        // Final rigid map correction applied only to post-stop points, using the
        // same odometry-selected association; this is not a timed TF replay.
        if(group==post_group) post_slam.push_back(distance(inverse(first_tf,apply(last_tf,p)),segments[best])*1000);
      }
    }
    report<<"fixed_scene_training=15 stable_bins="<<stable.size()<<" segments="<<segments.size()
      <<" geometric_training_match_m=0.02 spread_limit_m=0.01 segment_limit_m=0.08 angular_gap_bins=8 association_gate_m=0.05\n";
    const std::array<std::string,5> names=route?
      std::array<std::string,5>{"pre_holdout","reverse_margin","midpoint","return_margin","revisit"}:
      std::array<std::string,5>{"pre_holdout","motion_margin","post","unused","unused"};
    for(size_t i=0;i<(route?5u:3u);++i) { report<<"phase="<<names[i]<<" scans="<<phase_scans[i]<<" candidates="<<totals[i]<<'\n'; stats(report,names[i]+"_residual_mm",residuals[i]); }
    stats(report,"post_final_slam_correction_residual_mm",post_slam);
    for(auto *f:{&report,&tf_csv,&cmd_csv,&maps_csv,&status_csv,&odom_csv,&endpoints}) { f->close(); require(bool(*f),"Review output failed"); }
    std::cout<<"Review written to "<<out<<". Acquisition timing remains uncalibrated; association-gated scene statistics are exploratory.\n";
    return 0;
  } catch(const std::exception &e) { std::cerr<<"FAIL: "<<e.what()<<'\n'; return 1; }
}
