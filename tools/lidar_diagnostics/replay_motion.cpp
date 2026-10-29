// Offline continuous-motion experiment. No device, ROS publisher, or map writes.
#include "motion_model.hpp"
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <unistd.h>
using namespace lidar_motion;
void require(bool ok,const char *why) {if(!ok)throw std::runtime_error(why);}
std::vector<std::string> split(const std::string &s) {
  std::vector<std::string> result;std::istringstream in(s);std::string field;
  while(std::getline(in,field,','))result.push_back(field);
  return result;
}
int64_t integer(const std::string &s) {
  size_t used=0;auto value=std::stoll(s,&used);require(used==s.size(),"Invalid integer field");return value;
}
double number(const std::string &s) {
  size_t used=0;auto value=std::stod(s,&used);
  require(used==s.size() && std::isfinite(value),"Invalid finite numeric field");return value;
}
struct Beam {int64_t bin,source,packet,packet_sample,stamp;double range,angle;};
int main(int argc,char **argv) {
  try {
    require(argc==6,"Usage: replay_motion TIMING.csv ODOM.csv NEW_OUTPUT.csv nominal|scaled ASSUMED_DELAY_NS (explicit, 0..1e9)");
    const std::string model=argv[4];require(model=="nominal" || model=="scaled","Unknown timing model");
    const auto delay=integer(argv[5]);require(delay>=0 && delay<=1000000000,"Invalid delay hypothesis");
    std::ifstream oi(argv[2]);require(bool(oi),"Cannot open odometry");std::string line;
    std::getline(oi,line);require(line=="stamp_ns,x_m,y_m,yaw_rad","Invalid odom CSV header");
    std::vector<Odom> samples;
    while(std::getline(oi,line)) {
      require(line.size()<1024 && samples.size()<1000000,"Odometry input exceeds limits");
      auto c=split(line);require(c.size()==4,"Invalid odom row");
      samples.push_back({integer(c[0]),{number(c[1]),number(c[2]),number(c[3])}});
    }
    require(!oi.bad(),"Odometry read failure");const Trajectory odom(std::move(samples));
    std::ifstream ti(argv[1]);require(bool(ti),"Cannot open timing CSV");std::getline(ti,line);
    require(line=="scan_id,bin_index,source_index,packet_id,packet_sample_index,nominal_relative_ns,period_scaled_relative_ns,host_nominal_lower_ns,host_nominal_upper_ns,host_scaled_lower_ns,host_scaled_upper_ns,acquisition_time_valid,range_m,raw_angle_rad","Invalid timing CSV header; regenerate with replay_timing");
    std::map<int64_t,std::vector<Beam>> scans;std::map<int64_t,std::set<int64_t>> bins;
    size_t rows=0;
    while(std::getline(ti,line)) {
      require(line.size()<2048 && ++rows<=400000,"Timing input exceeds limits");
      auto c=split(line);require(c.size()==14,"Invalid timing row");
      const auto id=integer(c[0]),bin=integer(c[1]),source=integer(c[2]);
      require(id>=0 && bin>=0 && bin<400 && source>=-1 && source<800 && c[11]=="0","Invalid provenance/validity");
      require(bins[id].insert(bin).second,"Duplicate scan bin");
      if(source<0)continue;
      const size_t lower=model=="nominal"?7:9;
      const auto stamp=estimate_time({integer(c[lower]),integer(c[lower+1])},delay);
      const auto range=number(c[12]),angle=number(c[13]);
      require(range>=.05 && range<=12 && std::abs(angle)<=2*std::acos(-1.)+1e-9,"Invalid ray geometry");
      scans[id].push_back({bin,source,integer(c[3]),integer(c[4]),stamp,range,angle});
    }
    require(!ti.bad() && !bins.empty(),"Empty or unreadable timing input");
    for(const auto &entry:bins)require(entry.second.size()==400,"Incomplete scan grid");
    // Confirmed horizontal mounting. No unmeasured vertical value is needed for SE(2).
    const Pose mount{.110,.005,std::acos(-1.)};
    std::ostringstream out;out<<std::setprecision(17);
    out<<"scan_id,bin_index,source_index,packet_id,packet_sample_index,estimated_beam_ns,estimated_reference_ns,origin_x_m,origin_y_m,endpoint_x_m,endpoint_y_m,model,assumed_delay_ns,acquisition_time_valid\n";
    size_t corrected=0,masked=0;
    for(const auto &entry:scans) {
      const auto &beams=entry.second;
      const auto reference=std::max_element(beams.begin(),beams.end(),
        [](const Beam &a,const Beam &b){return a.stamp<b.stamp;})->stamp;
      // Prevalidate all beam poses, including masked returns, before creating output.
      odom.at(reference);
      for(const auto &b:beams) {
        odom.at(b.stamp);
        const double x=b.range*std::cos(b.angle),y=b.range*std::sin(b.angle);
        if(x>=-.0276 && x<=.2476 && y>=-.1575 && y<=.1675) {++masked;continue;}
        const auto ray=correct(odom,b.stamp,reference,mount,b.range,b.angle);
        out<<entry.first<<','<<b.bin<<','<<b.source<<','<<b.packet<<','<<b.packet_sample<<','
           <<b.stamp<<','<<reference<<','<<ray.origin.x<<','<<ray.origin.y<<','
           <<ray.endpoint.x<<','<<ray.endpoint.y<<','<<model<<','<<delay<<",0\n";
        ++corrected;
      }
    }
    require(corrected>0,"No exterior rays remain");
    const auto result=out.str();
    const int fd=open(argv[3],O_WRONLY|O_CREAT|O_EXCL,0600);
    if(fd<0)throw std::runtime_error(std::string("Cannot create output: ")+std::strerror(errno));
    FILE *file=fdopen(fd,"w");if(!file){close(fd);throw std::runtime_error("fdopen failed");}
    const bool good=std::fwrite(result.data(),1,result.size(),file)==result.size();
    const int closed=std::fclose(file);require(good && closed==0,"Write failed: discard partial output");
    std::cout<<"PASS model="<<model<<" assumed_delay_ns="<<delay<<" rays="<<corrected<<" body_masked="<<masked
      <<" acquisition_time_valid=0; output is per-ray geometry, NOT a LaserScan/map\n";
    return 0;
  } catch(const std::exception &e) {std::cerr<<"FAIL: "<<e.what()<<'\n';return 1;}
}
