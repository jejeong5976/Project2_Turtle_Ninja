#include <cassert>
#include <fstream>
#include <iterator>
#include <limits>
#include "diagnostic_recorder.hpp"
using robotis::coin_d4::DiagnosticRecorder;
std::string read(const std::string& p) {
  std::ifstream f(p); return {std::istreambuf_iterator<char>(f), {}};
}
int main(int argc, char** argv) {
  assert(argc == 2);
  const std::string dir=argv[1];
  std::vector<node_info> raw(4);
  for (size_t i=0;i<raw.size();++i) {
    raw[i].angle_q6_checkbit=(i*90*64<<1)|1;
    raw[i].distance_q2=1000+i;
    raw[i].sync_quality=100;
  }
  raw[0].sync_flag=1;
  LaserScan scan{};
  scan.config.angle_increment=1.5f; scan.config.max_angle=6.0f;
  for (int i=3;i>0;--i) {
    LaserPoint p{}; p.range=static_cast<float>(1000+i)/1000;
    scan.points.push_back(p);
  }
  for (bool reverse : {false,true}) {
    std::string file=dir+(reverse ? "/reverse.csv" : "/normal.csv");
    DiagnosticRecorder r(file,1);
    r.capture(raw,10,20,30,scan,reverse);
    r.publication(40);
    assert(r.flush_if_complete()); assert(!r.flush_if_complete()); assert(!r.wants_scan());
    const auto data=read(file);
    // raw index zero is explicitly omitted; input 3 uses input 1's angle.
    assert(data.find("0,4,3,0,1,0,1000,100,0,-1,,-1,,,")!=std::string::npos);
    const std::string expected=reverse ? "0,4,3,3,0,270,1003,100,0,1,90,2,3," :
      "0,4,3,3,0,270,1003,100,0,1,90,0,0,";
    assert(data.find(expected)!=std::string::npos);
    assert(data.find(",10,20,30,40\n")!=std::string::npos);
    bool rejected=false;
    try { DiagnosticRecorder duplicate(file,1); } catch(const std::runtime_error&) { rejected=true; }
    assert(rejected);
  }
  {
    DiagnosticRecorder partial(dir+"/partial.csv",2);
    scan.points[0].range=std::numeric_limits<float>::quiet_NaN();
    partial.capture(raw,10,20,30,scan,false);
    partial.publication(40);
    assert(!partial.flush_if_complete());
  }
  assert(read(dir+"/partial.csv").find("nan")!=std::string::npos);
  bool rejected=false;
  try { DiagnosticRecorder bad(dir+"/invalid.csv",0); } catch(const std::invalid_argument&) { rejected=true; }
  assert(rejected);
  std::puts("PASS: both index directions, omitted sample, timestamps, bounded completion, exclusive output, partial capture and NaN preservation");
}
