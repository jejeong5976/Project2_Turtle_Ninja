// Offline host-referenced hypotheses only; never publishes acquisition timestamps.
#include "sample_time_model.hpp"
#include "host_time_anchor.hpp"
#include <boost/json.hpp>
#include <algorithm>
#include <array>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <set>
#include <string>
#include <unistd.h>
namespace json=boost::json;
int64_t integer(const json::object& o,const char* key) {
  const auto& value=o.at(key);
  if (value.is_int64()) return value.as_int64();
  if (value.is_uint64() && value.as_uint64()<=INT64_MAX) return static_cast<int64_t>(value.as_uint64());
  throw std::runtime_error(std::string("Expected integer: ")+key);
}
void require(bool condition,const char* message) {if (!condition) throw std::runtime_error(message);}
struct Scan {
  bool boundary=false,published=false,handoff=false;
  int64_t count=0,start=0,end=0,closing_packet=0;
  lidar_timing::ClockPair handoff_clock{},publish_clock{};
  std::map<int64_t,std::pair<int64_t,int64_t>> samples; // raw index -> packet, packet index
  std::map<int64_t,int64_t> bins;
  std::map<int64_t,std::pair<double,double>> rays; // range metres, decoded angle radians
  bool converted=false,reverse=false;
};
void summary(const char* name,std::vector<double> values) {
  require(!values.empty(),"No statistics");std::sort(values.begin(),values.end());
  const auto n=values.size();const double median=(values[(n-1)/2]+values[n/2])/2;
  std::cout << name << " min=" << values.front() << " median=" << median
    << " max=" << values.back() << '\n';
}
int main(int argc,char** argv) {
  try {
    require(argc==3,"Usage: replay_timing INPUT.jsonl NEW_OUTPUT.csv");
    std::ifstream input(argv[1]);require(static_cast<bool>(input),"Cannot read input");
    std::map<int64_t,Scan> scans;
    std::map<int64_t,int64_t> packets;
    std::map<int64_t,int64_t> packet_ring_headers;
    std::set<int64_t> invalid_packets;
    std::string line;
    bool metadata=false,ended=false;
    int64_t expected=0,completed=0;
    std::size_t rows=0;
    while (std::getline(input,line)) {
      require(!ended,"Data after completion footer");
      require(++rows<=250001 && line.size()<8192,"Input exceeds recorder bounds");
      const auto value=json::parse(line);const auto& o=value.as_object();
      const auto kind=o.at("kind").as_string();
      if (kind=="metadata") {
        require(!metadata && rows==1,"Invalid metadata position");metadata=true;
        require(integer(o,"schema")==1,"Unsupported schema");expected=integer(o,"requested_scans");
      } else if (kind=="packet") {
        require(packets.emplace(integer(o,"packet_id"),integer(o,"sample_count")).second,"Duplicate packet");
        packet_ring_headers.emplace(integer(o,"packet_id"),integer(o,"ring_header_ns"));
        if (!integer(o,"checksum_ok")) invalid_packets.insert(integer(o,"packet_id"));
      } else if (kind=="boundary") {
        auto& s=scans[integer(o,"scan_id")];require(!s.boundary,"Duplicate boundary");s.boundary=true;
        s.count=integer(o,"raw_count");s.start=integer(o,"start_ns");s.end=integer(o,"end_ns");
        s.closing_packet=integer(o,"closing_packet_id");
      } else if (kind=="handoff") {
        auto& s=scans[integer(o,"scan_id")];require(!s.handoff,"Duplicate handoff");s.handoff=true;
        require(integer(o,"clock_bad")==0,"Invalid handoff clock");
        s.handoff_clock={integer(o,"mono_before_ns"),integer(o,"ros_ns"),integer(o,"mono_after_ns")};
      } else if (kind=="sample") {
        auto& s=scans[integer(o,"scan_id")];
        require(s.samples.emplace(integer(o,"raw_index"),std::make_pair(integer(o,"packet_id"),integer(o,"packet_sample_index"))).second,"Duplicate sample");
        s.rays.emplace(integer(o,"raw_index"),std::make_pair(integer(o,"distance_mm")/1000.0,
          integer(o,"angle_q6")/64.0*std::acos(-1.)/180.0));
      } else if (kind=="conversion") {
        auto& s=scans[integer(o,"scan_id")];require(!s.converted,"Duplicate conversion");
        const auto reverse=integer(o,"reverse");require(reverse==0 || reverse==1,"Invalid reverse flag");
        s.converted=true;s.reverse=reverse;
      } else if (kind=="bin") {
        auto& s=scans[integer(o,"scan_id")];
        require(s.bins.emplace(integer(o,"bin_index"),integer(o,"raw_index")).second,"Duplicate bin");
      } else if (kind=="publish") {
        auto& s=scans[integer(o,"scan_id")];require(!s.published,"Duplicate published scan");s.published=true;
        require(integer(o,"clock_bad")==0,"Invalid clock");
        s.publish_clock={integer(o,"mono_before_ns"),integer(o,"ros_ns"),integer(o,"mono_after_ns")};
      } else if (kind=="end") {
        ended=true;completed=integer(o,"published_scans");
        require(integer(o,"complete")==1 && integer(o,"overflow")==0 && integer(o,"clock_bad")==0,"Incomplete or invalid capture");
      }
    }
    require(!input.bad() && metadata && ended,"Missing metadata/footer or read failure");
    require(expected>=1 && expected<=1000 && completed==expected,"Unexpected completion count");
    std::vector<double> periods,rate,span_mismatch,max_difference,bracket_widths,clock_gaps,anchor_ages;
    std::size_t published=0,selected=0,negative=0,positive=0;
    struct Output {
      int64_t id;
      std::vector<lidar_timing::RelativeTime> times;
      std::vector<std::array<lidar_timing::HostEnvelope,2>> envelopes;
    };
    std::vector<Output> outputs;
    for (const auto& entry:scans) {
      const auto& s=entry.second;if (!s.published) continue;
      ++published;
      require(s.boundary && s.handoff && s.count>0 && s.count<800 && s.start>=0 && s.end>s.start,"Invalid ring boundary/count or missing handoff");
      require(s.samples.size()==static_cast<std::size_t>(s.count) && s.bins.size()==400,"Incomplete scan provenance");
      require(s.converted,"Missing scan conversion orientation");
      for (int64_t i=0;i<s.count;++i) {
        const auto source=s.samples.at(i);const auto n=packets.at(source.first);
        require(source.second>=0 && source.second<n,"Invalid packet sample index");
      }
      const auto first=s.samples.at(0);
      require(first.second==0 && packet_ring_headers.at(first.first)==s.start &&
        !invalid_packets.count(first.first),"Raw zero does not match validated opening ring packet");
      require(packet_ring_headers.at(s.closing_packet)==s.end && !invalid_packets.count(s.closing_packet),
        "Closing boundary does not match validated ring packet");
      require(s.end<=s.handoff_clock.before_ns && s.handoff_clock.after_ns<=s.publish_clock.before_ns,
        "Invalid handoff/publication clock order");
      const auto anchor=lidar_timing::host_anchor(s.start,s.handoff_clock);
      const auto gap=lidar_timing::bracket_gap(s.handoff_clock,s.publish_clock);
      require(gap==0,"Clock-offset brackets do not overlap; constant-offset hypothesis rejected");
      bracket_widths.push_back(lidar_timing::checked_sub(anchor.upper_ns,anchor.lower_ns));
      clock_gaps.push_back(gap);
      anchor_ages.push_back((s.handoff_clock.before_ns-s.start)/1e6);
      std::vector<int64_t> sources;
      for (int64_t b=0;b<400;++b) sources.push_back(s.bins.at(b));
      const double period=static_cast<double>(s.end-s.start);
      auto times=lidar_timing::model_times(s.count,period,sources);
      std::vector<std::array<lidar_timing::HostEnvelope,2>> envelopes;
      for (const auto& time:times) {
        if (time.source_index<0) envelopes.push_back({});
        else envelopes.push_back({lidar_timing::shift_model(anchor,time.nominal_ns),
          lidar_timing::shift_sample_fraction(anchor,time.source_index,s.end-s.start,s.count)});
      }
      double max_diff=0;int64_t previous=-1;
      for (const auto& time:times) {
        if (time.source_index<0) continue;
        require(!invalid_packets.count(s.samples.at(time.source_index).first),"Selected sample from corrupt packet");
        ++selected;
        if (previous>=0) {negative+=time.source_index<previous;positive+=time.source_index>previous;}
        previous=time.source_index;
        max_diff=std::max(max_diff,std::abs(time.nominal_ns-time.period_scaled_ns));
      }
      periods.push_back(period/1e6);rate.push_back(s.count*1e9/period);
      span_mismatch.push_back((s.count*250000.0-period)/1e6);
      max_difference.push_back(max_diff/1e6);outputs.push_back({entry.first,std::move(times),std::move(envelopes)});
    }
    require(published==static_cast<std::size_t>(completed),"Published count mismatch");
    // Create only after validating the entire input; never overwrite existing data.
    const int fd=open(argv[2],O_WRONLY|O_CREAT|O_EXCL,0600);
    if (fd<0) throw std::runtime_error(std::string("Cannot create output: ")+std::strerror(errno));
    FILE* out=fdopen(fd,"w");if (!out) {close(fd);throw std::runtime_error("fdopen failed");}
    bool good=std::fprintf(out,"scan_id,bin_index,source_index,packet_id,packet_sample_index,nominal_relative_ns,period_scaled_relative_ns,host_nominal_lower_ns,host_nominal_upper_ns,host_scaled_lower_ns,host_scaled_upper_ns,acquisition_time_valid,range_m,raw_angle_rad\n")>0;
    for (const auto& output:outputs) {
      for (std::size_t b=0;b<output.times.size();++b) {
        const auto& t=output.times[b];
        const auto provenance=t.source_index<0 ? std::make_pair(int64_t(-1),int64_t(-1)) : scans.at(output.id).samples.at(t.source_index);
        good &= std::fprintf(out,"%lld,%zu,%lld,%lld,%lld,%.3f,%.3f",
          static_cast<long long>(output.id),b,static_cast<long long>(t.source_index),
          static_cast<long long>(provenance.first),static_cast<long long>(provenance.second),t.nominal_ns,t.period_scaled_ns)>0;
        if (t.source_index<0) good &= std::fprintf(out,",,,,,0,nan,nan\n")>0;
        else {
          const auto& e=output.envelopes[b];
          const auto &scan=scans.at(output.id);const auto ray=scan.rays.at(t.source_index);
          good &= std::fprintf(out,",%lld,%lld,%lld,%lld,0,%.9f,%.12f\n",static_cast<long long>(e[0].lower_ns),
            static_cast<long long>(e[0].upper_ns),static_cast<long long>(e[1].lower_ns),
            static_cast<long long>(e[1].upper_ns),ray.first,scan.reverse?ray.second:-ray.second)>0;
        }
      }
    }
    const int close_result=std::fclose(out);require(good && close_result==0,"Output write failed; discard partial output");
    std::cout << std::fixed << std::setprecision(6) << "published=" << published << " selected=" << selected
      << " negative_source_steps=" << negative << " positive_source_steps=" << positive << '\n';
    summary("host_period_ms",periods);summary("inferred_samples_per_second",rate);
    summary("nominal_ring_span_minus_host_period_ms",span_mismatch);
    summary("maximum_selected_offset_disagreement_ms",max_difference);
    summary("clock_bracket_only_width_ns",bracket_widths);
    summary("handoff_publish_offset_gap_ns",clock_gaps);
    summary("anchor_mapping_lookback_ms",anchor_ages);
    std::cout << "Host envelopes assume constant clock offset. Transport delay is unknown; acquisition_time_valid=0.\n";
  } catch (const std::exception& e) {std::cerr << "FAIL: " << e.what() << '\n';return 1;}
}
