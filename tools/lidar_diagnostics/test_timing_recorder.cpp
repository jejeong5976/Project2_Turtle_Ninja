#include "timing_recorder.hpp"
#include <cassert>
#include <fstream>
#include <iterator>
#include <thread>
std::string read(const std::string& path) {
  std::ifstream file(path); return {std::istreambuf_iterator<char>(file),{}};
}
int main(int argc,char** argv) {
  assert(argc==2); const std::string dir=argv[1];
  {
    lidar_timing::Recorder recorder(dir+"/complete.jsonl",2);
    std::thread producer([&]{for(int i=0;i<100;++i) recorder.event("read",{{"id",i}});});
    recorder.pair("handoff",0,100,1000,110);
    recorder.pair("publish",0,200,1100,210);
    producer.join();
    assert(!recorder.published()); assert(recorder.published());
    assert(!recorder.published() && !recorder.recording());
  }
  assert(read(dir+"/complete.jsonl").find("\"complete\":1")!=std::string::npos);
  bool refused=false;
  try { lidar_timing::Recorder duplicate(dir+"/complete.jsonl",1); }
  catch(const std::runtime_error&) {refused=true;} assert(refused);
  {lidar_timing::Recorder partial(dir+"/partial.jsonl",2);partial.published();}
  assert(read(dir+"/partial.jsonl").find("\"complete\":0")!=std::string::npos);
  {lidar_timing::Recorder capped(dir+"/cap.jsonl",2,1);assert(capped.published());}
  assert(read(dir+"/cap.jsonl").find("\"overflow\":1")!=std::string::npos);
  {
    lidar_timing::Recorder jump(dir+"/jump.jsonl",1);
    jump.pair("handoff",0,100,1000,110);
    jump.pair("publish",0,200,10000000,210);jump.published();
  }
  assert(read(dir+"/jump.jsonl").find("\"clock_bad\":1")!=std::string::npos);
  assert(read(dir+"/jump.jsonl").find("\"complete\":0")!=std::string::npos);
  std::puts("PASS: bounded capture, concurrent events, complete/partial footer, clock jump, no overwrite");
}
