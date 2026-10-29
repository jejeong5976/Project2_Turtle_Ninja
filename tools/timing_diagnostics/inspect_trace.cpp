#include "nt_timing_trace.hpp"
#include <fstream>
#include <iostream>
#include <map>
#include <limits>
int main(int argc,char **argv){
 try {
  if(argc!=2)throw std::runtime_error("Usage: inspect_trace FILE.nttrace");
  std::ifstream f(argv[1],std::ios::binary);if(!f)throw std::runtime_error("Cannot open trace");
  nt_timing::Record r{},end{};std::map<uint64_t,uint64_t> counts;
  if(!f.read(reinterpret_cast<char*>(&r),sizeof r)||r.kind!=nt_timing::Header||r.id!=1||r.a!=sizeof r)throw std::runtime_error("Invalid trace header");
  uint64_t n=0;bool ended=false;int64_t low=INT64_MAX,high=INT64_MIN,max_bracket=0;
  while(f.read(reinterpret_cast<char*>(&r),sizeof r)) {
    if(ended)throw std::runtime_error("Data follows footer");
    if(r.kind==nt_timing::End){end=r;ended=true;continue;}
    if(r.kind<=nt_timing::End||r.mono_before<0||r.system<0||r.mono_after<r.mono_before)throw std::runtime_error("Invalid record/clock bracket");
    low=std::min(low,r.system-r.mono_after);high=std::max(high,r.system-r.mono_before);
    max_bracket=std::max(max_bracket,r.mono_after-r.mono_before);++n;++counts[r.kind];
  }
  if(f.gcount()!=0||!ended||end.id!=n||end.a!=static_cast<int64_t>(n)||end.b!=0||end.c!=0)throw std::runtime_error("Incomplete/lost/corrupt trace; not calibration input");
  std::cout<<"TRACE CONTAINER PASS records="<<n<<" max_emit_ns="<<end.d<<" max_clock_bracket_ns="<<max_bracket<<'\n';
  if(n)std::cout<<"system_minus_monotonic_envelope_span_ns="<<high-low<<" (not sensor latency)\n";
  for(auto [kind,count]:counts)std::cout<<"kind="<<kind<<" count="<<count<<'\n';
  std::cout<<"Container completeness only; validate event associations, clock stability, physical input and coverage separately.\n";
  return 0;
 }catch(const std::exception &e){std::cerr<<"TRACE INVALID: "<<e.what()<<'\n';return 1;}
}
