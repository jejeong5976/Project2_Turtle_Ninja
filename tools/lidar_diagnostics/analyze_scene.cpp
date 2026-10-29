// Offline scene repeatability: frozen stationary reference, no pose/delay fitting.
// Reuse the replay's strict CSV helpers and motion types without its entry point.
#define main replay_motion_entry
#include "replay_motion.cpp"
#undef main
#include <array>
struct Segment {Point a,b;};
double distance(Point p,Segment s) {
  const double x=s.b.x-s.a.x,y=s.b.y-s.a.y,d=x*x+y*y;
  const double t=d>0?std::clamp(((p.x-s.a.x)*x+(p.y-s.a.y)*y)/d,0.,1.):0.;
  return std::hypot(p.x-s.a.x-t*x,p.y-s.a.y-t*y);
}
double median(std::vector<double> v) {
  require(!v.empty(),"Empty statistic");std::sort(v.begin(),v.end());
  return v[v.size()/2];
}
struct SceneBeam {int bin;int64_t nominal,scaled;Point point;};
struct Stats {
  size_t total=0;std::vector<double> raw,corrected;
  void print() const {
    std::cout<<" accepted="<<raw.size()<<"/"<<total;
    if(raw.empty()){std::cout<<'\n';return;}
    double r=0,c=0;std::vector<double> delta;
    for(size_t i=0;i<raw.size();++i){r+=raw[i]*raw[i];c+=corrected[i]*corrected[i];delta.push_back(corrected[i]-raw[i]);}
    std::cout<<" raw_median_mm="<<median(raw)*1000<<" corrected_median_mm="<<median(corrected)*1000
      <<" raw_rms_mm="<<std::sqrt(r/raw.size())*1000<<" corrected_rms_mm="<<std::sqrt(c/raw.size())*1000
      <<" paired_median_delta_mm="<<median(delta)*1000<<'\n';
  }
};
int scene_main(int argc,char **argv) {
 try {
  require(argc>=3 && argc<=6,"Usage: analyze_scene VALIDATED_TIMING.csv ODOM.csv [REFERENCE_SPREAD_M SEGMENT_LENGTH_M] [MAPPING_SCAN.csv]; run replay_timing first");
  const double spread_limit=argc>=5?number(argv[3]):.010,length_limit=argc>=5?number(argv[4]):.080;
  require(spread_limit>=.001 && spread_limit<=.100 && length_limit>=.010 && length_limit<=.500,"Reference thresholds outside diagnostic bounds");
  struct Mapped {int64_t reference;Point endpoint;};
  std::map<std::pair<int64_t,int64_t>,Mapped> mapped;std::string mapped_model;int64_t mapped_delay=-1;
  if(argc==4||argc==6) {
    std::ifstream mi(argv[argc-1]);std::string row;std::getline(mi,row);
    require(row=="scan_id,reference_ns,bin_index,angle_rad,range_m,input_bin,endpoint_error_m,origin_shift_m,path_bound_m,model,assumed_delay_ns,acquisition_time_valid","Invalid mapping scan header");
    while(std::getline(mi,row)) {
      auto c=split(row);require(c.size()==12&&c[11]=="0","Invalid mapping scan row");
      if(mapped_model.empty()){mapped_model=c[9];mapped_delay=integer(c[10]);}
      require(c[9]==mapped_model&&integer(c[10])==mapped_delay,"Mixed mapping hypotheses");
      if(integer(c[5])<0)continue;
      const auto range=number(c[4]),angle=number(c[3]);
      require(mapped.emplace(std::make_pair(integer(c[0]),integer(c[5])),Mapped{integer(c[1]),{range*std::cos(angle),range*std::sin(angle)}}).second,"Duplicate mapping source");
    }
    require(!mi.bad()&&!mapped.empty(),"Empty/unreadable mapping scan");
  }
  std::ifstream oi(argv[2]);std::string line;std::getline(oi,line);
  require(line=="stamp_ns,x_m,y_m,yaw_rad","Invalid odom header");
  std::vector<Odom> samples;
  while(std::getline(oi,line)){auto c=split(line);require(c.size()==4,"Invalid odom");samples.push_back({integer(c[0]),{number(c[1]),number(c[2]),number(c[3])}});}
  const Trajectory odom(samples);const Pose mount{.110,.005,std::acos(-1.)};
  int64_t start=-1,end=-1;
  for(size_t i=1;i<samples.size();++i) {
    const auto &a=samples[i-1],&b=samples[i];
    if(std::hypot(b.pose.x-a.pose.x,b.pose.y-a.pose.y)>1e-7 || std::abs(wrap(b.pose.yaw-a.pose.yaw))>1e-7){if(start<0)start=a.stamp_ns;end=b.stamp_ns;}
  }
  require(start>=0,"No motion found");
  std::ifstream ti(argv[1]);std::getline(ti,line);
  require(line=="scan_id,bin_index,source_index,packet_id,packet_sample_index,nominal_relative_ns,period_scaled_relative_ns,host_nominal_lower_ns,host_nominal_upper_ns,host_scaled_lower_ns,host_scaled_upper_ns,acquisition_time_valid,range_m,raw_angle_rad","Invalid timing header");
  std::map<int64_t,std::vector<SceneBeam>> scans;
  std::map<int64_t,std::array<int64_t,2>> references;
  while(std::getline(ti,line)) {
    auto c=split(line);require(c.size()==14 && c[11]=="0","Invalid timing row");
    if(integer(c[2])<0)continue;
    const auto id=integer(c[0]);const auto bin=integer(c[1]);require(bin>=0&&bin<400,"Invalid bin");
    const auto n=estimate_time({integer(c[7]),integer(c[8])},0),s=estimate_time({integer(c[9]),integer(c[10])},0);
    auto &ref=references[id];ref[0]=std::max(ref[0],n);ref[1]=std::max(ref[1],s);
    Point p{number(c[12])*std::cos(number(c[13])),number(c[12])*std::sin(number(c[13]))};
    if(p.x>=-.0276&&p.x<=.2476&&p.y>=-.1575&&p.y<=.1675)continue;
    scans[id].push_back({int(bin),n,s,p});
  }
  require(scans.size()>=30,"Too few scans");
  std::array<std::vector<double>,400> xs,ys;
  size_t train=0;int64_t train_last=-1;
  for(const auto &scan:scans) {
    if(train==15)break;
    require(references.at(scan.first)[0]<start-200000000,"Training scans overlap motion");
    for(const auto &b:scan.second){auto p=apply(odom.at(b.nominal),apply(mount,b.point));xs[b.bin].push_back(p.x);ys[b.bin].push_back(p.y);}
    train_last=scan.first;++train;
  }
  std::vector<Segment> segments;
  for(size_t i=0;i<399;++i) {
    if(xs[i].size()<12||xs[i+1].size()<12)continue;
    Point a{median(xs[i]),median(ys[i])},b{median(xs[i+1]),median(ys[i+1])};
    double spread=0;
    for(size_t k=i;k<=i+1;++k){std::vector<double> dev;for(size_t j=0;j<xs[k].size();++j)dev.push_back(std::hypot(xs[k][j]-median(xs[k]),ys[k][j]-median(ys[k])));spread=std::max(spread,median(dev));}
    const double length=std::hypot(b.x-a.x,b.y-a.y);
    if(length>.002&&length<length_limit&&spread<spread_limit)segments.push_back({a,b});
  }
  require(!segments.empty(),"No stable reference segments");
  std::cout<<std::setprecision(7)<<"reference_scans="<<train<<" stable_segments="<<segments.size()<<" reference_spread_m="<<spread_limit<<" segment_length_m="<<length_limit<<" motion_duration_s="<<double(end-start)/1e9<<'\n';
  // Same conservative phase membership for every model/delay: the moving group
  // includes the transition margin; held-out stationary groups cannot overlap it.
  for(int model=0;model<2;++model)for(int64_t delay:{0LL,10000000LL,25000000LL,50000000LL,100000000LL}) {
    std::array<Stats,3> stats,grid_stats;std::array<size_t,3> counts{};
    const bool compare_grid=mapped_model==(model==0?"nominal":"scaled")&&mapped_delay==delay;
    for(const auto &scan:scans) {
      if(scan.first<=train_last)continue;
      const auto &ref=references.at(scan.first);
      const int group=ref[0]<start-200000000?0:(ref[0]>end+300000000?2:1);++counts[group];
      const auto reference_pose=odom.at(ref[model]-delay);
      for(const auto &b:scan.second) {
        auto &st=stats[group];++st.total;
        const auto raw=apply(reference_pose,apply(mount,b.point));
        const auto corrected=apply(odom.at((model==0?b.nominal:b.scaled)-delay),apply(mount,b.point));
        size_t best=0;double d=distance(raw,segments[0]);
        for(size_t j=1;j<segments.size();++j){const auto candidate=distance(raw,segments[j]);if(candidate<d){d=candidate;best=j;}}
        if(d>.050)continue; // Frozen raw association, shared with corrected case.
        st.raw.push_back(d);st.corrected.push_back(distance(corrected,segments[best]));
        if(compare_grid) {
          auto &gs=grid_stats[group];++gs.total;const auto found=mapped.find({scan.first,b.bin});
          if(found!=mapped.end()) {
            require(found->second.reference==ref[model]-delay,"Mapping reference mismatch");
            const auto point=apply(reference_pose,apply(mount,found->second.endpoint));
            gs.raw.push_back(distance(corrected,segments[best]));gs.corrected.push_back(distance(point,segments[best]));
          }
        }
      }
    }
    for(int group=0;group<3;++group){std::cout<<(model==0?"nominal":"scaled")<<" delay_ms="<<delay/1000000<<" phase="<<std::array<const char*,3>{"pre_holdout","motion_margin","post"}[group]<<" scans="<<counts[group];stats[group].print();}
    if(compare_grid)for(int group=0;group<3;++group){std::cout<<"per_ray_to_resampled phase="<<std::array<const char*,3>{"pre_holdout","motion_margin","post"}[group];grid_stats[group].print();}
  }
  std::cout<<"Exploratory fixed-scene repeatability only; no optical timing calibration, registration, or fitted delay. Distances are unsigned and association-gated.\n";
  return 0;
 }catch(const std::exception &e){std::cerr<<"FAIL: "<<e.what()<<'\n';return 1;}
}
#ifndef LIDAR_SCENE_TEST
int main(int argc,char **argv) {return scene_main(argc,argv);}
#endif
