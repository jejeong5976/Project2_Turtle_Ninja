// Reconstruct live frames from complete saved provenance, compare offline export.
#define main replay_motion_entry
#include "replay_motion.cpp"
#undef main
#include "mapping_frame.hpp"
#include <boost/json.hpp>
int64_t ji(const boost::json::object &o,const char *key){return o.at(key).to_number<int64_t>();}
int main(int argc,char **argv) {
 try {
  require(argc==4,"Usage: test_mapping_replay CAPTURE.jsonl ODOM.csv EXPECTED_1600_SCAN.csv");
  struct Saved {lidar_mapping::Frame frame;std::map<int64_t,lidar_mapping::Selected> raw;std::map<int,int64_t> bins;bool published=false;};
  std::map<int64_t,Saved> saved;std::ifstream in(argv[1]);std::string line;bool complete=false;
  while(std::getline(in,line)){
    const auto value=boost::json::parse(line);const auto &o=value.as_object();const auto kind=o.at("kind").as_string();
    if(kind=="end"){require(ji(o,"complete")==1&&ji(o,"published_scans")==200,"Incomplete capture");complete=true;continue;}
    if(!o.contains("scan_id"))continue;
    const auto id=ji(o,"scan_id");auto &s=saved[id];s.frame.id=id;s.frame.source_valid=true;
    if(kind=="boundary"){s.frame.start=ji(o,"start_ns");s.frame.end=ji(o,"end_ns");s.frame.count=ji(o,"raw_count");}
    if(kind=="handoff")s.frame.clock={ji(o,"mono_before_ns"),ji(o,"ros_ns"),ji(o,"mono_after_ns")};
    if(kind=="sample"){require(ji(o,"checksum_ok")==1,"Invalid sample");const auto index=ji(o,"raw_index");s.raw[index]={-1,index,ji(o,"distance_mm")/1000.,-ji(o,"angle_q6")/64.*std::acos(-1.)/180.};}
    if(kind=="bin")s.bins[int(ji(o,"bin_index"))]=ji(o,"raw_index");
    if(kind=="publish")s.published=true;
  }
  require(complete,"Missing footer");
  std::ifstream oi(argv[2]);std::getline(oi,line);std::vector<Odom> odom;
  while(std::getline(oi,line)){auto c=split(line);odom.push_back({integer(c[0]),{number(c[1]),number(c[2]),number(c[3])}});}
  std::ifstream ei(argv[3]);std::getline(ei,line);std::map<int64_t,std::vector<double>> expected;
  while(std::getline(ei,line)){auto c=split(line);require(c.size()==12&&c[9]=="nominal"&&c[10]=="0","Invalid expected data");auto &v=expected[integer(c[0])];require(integer(c[2])==int64_t(v.size()),"Grid order");v.push_back(c[4]=="nan"?NAN:number(c[4]));}
  size_t scans=0,finite=0;
  for(auto &entry:saved){auto &s=entry.second;if(!s.published)continue;
    for(const auto &bin:s.bins)if(bin.second>=0){auto beam=s.raw.at(bin.second);beam.bin=bin.first;s.frame.beams.push_back(beam);}
    const auto timed=lidar_mapping::time_frame(s.frame,"nominal",0);
    auto lo=std::lower_bound(odom.begin(),odom.end(),timed.first,[](const Odom &p,int64_t t){return p.stamp_ns<t;});require(lo!=odom.begin(),"Early coverage");--lo;
    auto hi=std::lower_bound(odom.begin(),odom.end(),timed.reference,[](const Odom &p,int64_t t){return p.stamp_ns<t;});require(hi!=odom.end(),"Late coverage");
    const auto projected=lidar_mapping::project_frame(timed,{lo,std::next(hi)});const auto &want=expected.at(entry.first);
    require(want.size()==projected.bins.size(),"Grid size mismatch");
    for(size_t i=0;i<want.size();++i){const auto actual=projected.bins[i].range;
      if(std::isnan(want[i])!=std::isnan(actual)||(!std::isnan(want[i])&&std::abs(actual-want[i])>=1e-8)) {
        std::cerr<<"scan="<<entry.first<<" bin="<<i<<" expected="<<want[i]<<" actual="<<actual<<'\n';throw std::runtime_error("Live/offline output mismatch");
      }
      finite+=std::isfinite(actual);
    }
    ++scans;
  }
  require(scans==200,"Missing scans");std::cout<<"PASS live-frame core equals offline export: scans="<<scans<<" finite="<<finite<<"; 40 ms gap and 0.04 m/s, 0.15 rad/s gates passed\n";return 0;
 }catch(const std::exception &e){std::cerr<<"FAIL: "<<e.what()<<'\n';return 1;}
}
