// Consumes complete replay_motion output. No ROS publisher or device access.
#define main replay_motion_entry
#include "replay_motion.cpp"
#undef main
#include "mapping_scan.hpp"
int main(int argc,char **argv) {
 try {
  require(argc==5,"Usage: replay_mapping VALIDATED_RAYS.csv NEW_SCAN.csv BINS EXPECTED_SCANS");
  lidar_mapping::Policy policy;policy.bins=integer(argv[3]);const auto expected=integer(argv[4]);
  require(expected>=1&&expected<=1000,"Invalid expected scan count");
  std::ifstream in(argv[1]);std::string line;std::getline(in,line);
  require(line=="scan_id,bin_index,source_index,packet_id,packet_sample_index,estimated_beam_ns,estimated_reference_ns,origin_x_m,origin_y_m,endpoint_x_m,endpoint_y_m,model,assumed_delay_ns,acquisition_time_valid","Invalid corrected ray header");
  struct Record {int64_t reference;std::string model;int64_t delay;std::vector<lidar_mapping::Ray> rays;};
  std::map<int64_t,Record> scans;size_t rows=0;std::string model;int64_t delay=-1;
  while(std::getline(in,line)) {
    require(line.size()<2048 && ++rows<=400000,"Ray input exceeds bounds");auto c=split(line);
    require(c.size()==14 && c[13]=="0","Invalid ray/validity");
    const auto id=integer(c[0]),ref=integer(c[6]),d=integer(c[12]);
    require(id>=0&&ref>0&&integer(c[5])>=0&&integer(c[5])<=ref&&d>=0&&d<=1000000000&&
      (c[11]=="nominal"||c[11]=="scaled"),"Invalid ray metadata");
    if(model.empty()){model=c[11];delay=d;}
    require(model==c[11]&&delay==d,"Mixed timing hypotheses");
    auto [it,created]=scans.emplace(id,Record{ref,c[11],d,{}});
    require(created||it->second.reference==ref,"Mixed reference times");
    const auto bin=integer(c[1]);require(bin>=0&&bin<400,"Invalid source bin");
    it->second.rays.push_back({int(bin),{number(c[7]),number(c[8])},{number(c[9]),number(c[10])}});
  }
  require(!in.bad()&&scans.size()==size_t(expected),"Incomplete scan set; empty-ray scans need upstream handling");
  std::ostringstream out;out<<std::setprecision(17);
  out<<"scan_id,reference_ns,bin_index,angle_rad,range_m,input_bin,endpoint_error_m,origin_shift_m,path_bound_m,model,assumed_delay_ns,acquisition_time_valid\n";
  size_t kept=0,collisions=0,rejected=0,range_rejected=0;double max_error=0,max_origin=0,max_bound=0,sum_squared=0;
  int64_t previous=-1;
  for(const auto &entry:scans) {
    const auto &record=entry.second;require(record.reference>previous,"Reference stamps did not advance");previous=record.reference;
    const auto scan=lidar_mapping::resample(record.rays,policy);
    collisions+=scan.collisions;rejected+=scan.approximation_rejected;range_rejected+=scan.range_rejected;
    for(size_t i=0;i<scan.bins.size();++i) {
      const auto &b=scan.bins[i];
      if(b.input_bin>=0){++kept;sum_squared+=b.endpoint_error*b.endpoint_error;max_error=std::max(max_error,b.endpoint_error);max_origin=std::max(max_origin,b.origin_shift);max_bound=std::max(max_bound,b.path_bound);}
      out<<entry.first<<','<<record.reference<<','<<i<<','<<2*std::acos(-1.)*i/policy.bins<<','<<b.range<<','<<b.input_bin<<','
         <<b.endpoint_error<<','<<b.origin_shift<<','<<b.path_bound<<','<<record.model<<','<<record.delay<<",0\n";
    }
  }
  require(kept>0,"No accepted mapping returns");
  const auto data=out.str();const int fd=open(argv[2],O_WRONLY|O_CREAT|O_EXCL,0600);require(fd>=0,"Cannot exclusively create scan output");
  FILE *file=fdopen(fd,"w");if(!file){close(fd);throw std::runtime_error("fdopen");}
  const bool good=std::fwrite(data.data(),1,data.size(),file)==data.size();const int closed=std::fclose(file);require(good&&closed==0,"Write failed; discard partial output");
  std::cout<<"scans="<<scans.size()<<" bins="<<policy.bins<<" input="<<rows<<" kept="<<kept<<" collisions="<<collisions
    <<" approximation_rejected="<<rejected<<" range_rejected="<<range_rejected<<" endpoint_rms_mm="<<std::sqrt(sum_squared/kept)*1000
    <<" endpoint_max_mm="<<max_error*1000<<" origin_max_mm="<<max_origin*1000<<" path_bound_max_mm="<<max_bound*1000
    <<" model="<<model<<" delay_ns="<<delay<<" acquisition_time_valid=0\n";
  return 0;
 }catch(const std::exception &e){std::cerr<<"FAIL: "<<e.what()<<'\n';return 1;}
}
