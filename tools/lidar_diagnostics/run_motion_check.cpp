// User-run hardware experiment coordinator. No automatic deployment or Git actions.
#include <sys/wait.h>
#include <signal.h>
#include <unistd.h>
#include <fcntl.h>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
namespace fs=std::filesystem;
using Clock=std::chrono::steady_clock;
volatile sig_atomic_t interrupted=0;
void signal_handler(int) {interrupted=1;}
void require(bool ok,const std::string &why) {if(!ok)throw std::runtime_error(why);}
void poll_delay() {std::this_thread::sleep_for(std::chrono::milliseconds(100));}
std::string read(const fs::path &p) {std::ifstream f(p);return {std::istreambuf_iterator<char>(f),{}};}
bool contains(const std::string &s,const std::string &v) {return s.find(v)!=std::string::npos;}
int64_t now_ns() {return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();}
struct Child {
  pid_t pid=-1;int status=0;bool done=false;
  Child()=default;Child(const Child&)=delete;Child& operator=(const Child&)=delete;
  void start(std::vector<std::string> args,const fs::path &log) {
    require(pid<0,"Child already started");
    const int fd=open(log.c_str(),O_WRONLY|O_CREAT|O_EXCL,0600);require(fd>=0,"Cannot create log");
    pid=fork();if(pid<0){close(fd);throw std::runtime_error("fork failed");}
    if(pid==0) {
      setpgid(0,0);dup2(fd,STDOUT_FILENO);dup2(fd,STDERR_FILENO);close(fd);
      // Progress must be observable while ros2 topic pub is still running.
      if(setenv("PYTHONUNBUFFERED","1",1)!=0)_exit(127);
      std::vector<char*> av;for(auto &s:args)av.push_back(s.data());av.push_back(nullptr);
      execvp(av[0],av.data());_exit(127);
    }
    close(fd);setpgid(pid,pid);
  }
  bool running() {
    if(pid<0 || done)return false;
    if(waitpid(pid,&status,WNOHANG)==pid)done=true;
    return !done;
  }
  int finish(double seconds,bool heed_interrupt=true) {
    auto end=Clock::now()+std::chrono::duration<double>(seconds);
    while(running() && Clock::now()<end) {
      if(heed_interrupt && interrupted)throw std::runtime_error("Interrupted");
      poll_delay();
    }
    require(!running(),"Child command timed out");
    return WIFEXITED(status)?WEXITSTATUS(status):128+WTERMSIG(status);
  }
  void stop() noexcept {
    if(pid<0)return;
    if(done){pid=-1;return;}
    // Signal only the process group created by this coordinator, including ros2run.
    for(int sig:{SIGINT,SIGTERM,SIGKILL}) {
      kill(-pid,sig);
      const auto end=Clock::now()+std::chrono::seconds(sig==SIGINT?4:1);
      while(running() && Clock::now()<end)poll_delay();
      if(!running()){kill(-pid,sig);break;}
    }
    if(!done){waitpid(pid,&status,0);done=true;}
    pid=-1;
  }
  ~Child(){stop();}
};
int command(std::vector<std::string> args,const fs::path &log,double timeout=10) {
  Child c;c.start(std::move(args),log);return c.finish(timeout);
}
int publication_count(const std::string &log) {
  std::istringstream lines(log);std::string line;int count=0;
  while(std::getline(lines,line)) {
    if(line.rfind("publishing #",0)!=0)continue;
    const auto colon=line.find(':',12);
    if(colon==std::string::npos)continue; // A partial progress write is not evidence.
    const auto value=line.substr(12,colon-12);size_t used=0;
    const int n=std::stoi(value,&used);
    require(used==value.size() && n==count+1,"Publisher progress is not sequential");++count;
  }
  return count;
}
struct PublicationWatch {
  int expected;double pulse_budget;double started;double first=-1,last=-1;int count=0;
  void update(int next,double now) {
    require(next>=count && next<=expected,"Unexpected publisher count");
    if(first<0)require(now-started<8.,"Publisher discovery/startup exceeded 8 seconds; no timely publication observed");
    else if(last<0)require(now-first<pulse_budget,"Publisher motion/zero sequence exceeded its publication deadline");
    else require(now-last<2.,"Publisher did not exit within 2 seconds after final publication");
    if(next>0 && first<0)first=now;
    if(next==expected && last<0)last=now;
    count=next;
  }
};
// CLI progress is printed immediately BEFORE publish(): it is not a delivery ack.
// Require successful child exit as well, and confirm stopping separately in odom.
void wait_publisher(Child &child,const fs::path &log,int count,double pulse_budget,
                    const std::function<void()> &monitor,bool heed_interrupt=true) {
  const auto epoch=Clock::now();PublicationWatch watch{count,pulse_budget,0};
  while(true) {
    if(heed_interrupt)require(!interrupted,"Interrupted while waiting for command publisher");
    monitor();const bool running=child.running();
    const auto n=publication_count(read(log));
    watch.update(n,std::chrono::duration<double>(Clock::now()-epoch).count());
    if(!running){require(child.finish(0,false)==0 && n==count,"Command publisher exited without completing sequence");return;}
    poll_delay();
  }
}
std::vector<std::string> zero_command() {
  return {"ros2","topic","pub","/cmd_vel","geometry_msgs/msg/TwistStamped",
    "{header: {stamp: now, frame_id: base_footprint}, twist: {linear: {x: 0.0}, angular: {z: 0.0}}}",
    "--rate","10","--times","3","--max-wait-time-secs","5","--keep-alive","0.2"};
}
std::string rotation_sequence() {
  std::ostringstream out;
  for(int i=0;i<23;++i) {
    if(i)out<<"---\n";
    out<<"header: {stamp: now, frame_id: base_footprint}\ntwist: {linear: {x: 0.0}, angular: {z: "
       <<(i<20?"0.10":"0.0")<<"}}\n";
  }
  return out.str();
}
std::vector<std::string> rotation_command(const fs::path &yaml) {
  return {"ros2","topic","pub","/cmd_vel","geometry_msgs/msg/TwistStamped",
    "--yaml-file",yaml.string(),"--rate","10","--times","23",
    "--max-wait-time-secs","5","--keep-alive","0.2"};
}
void write_exclusive(const fs::path &path,const std::string &data) {
  const int fd=open(path.c_str(),O_WRONLY|O_CREAT|O_EXCL,0600);require(fd>=0,"Cannot create sequence file");
  size_t offset=0;
  while(offset<data.size()) {
    const auto n=write(fd,data.data()+offset,data.size()-offset);
    if(n<=0){close(fd);throw std::runtime_error("Cannot write sequence file");}offset+=size_t(n);
  }
  require(close(fd)==0,"Cannot close sequence file");
}
struct Pose {int64_t stamp;double x,y,yaw;};
std::vector<Pose> poses(const fs::path &file) {
  std::istringstream in(read(file));std::string line;std::getline(in,line);std::vector<Pose> out;
  while(std::getline(in,line)) {
    if(in.eof())break; // Ignore an incomplete final write.
    for(char &c:line)if(c==',')c=' ';
    Pose p{};std::istringstream row(line);
    require(bool(row>>p.stamp>>p.x>>p.y>>p.yaw),"Malformed odometry row");
    require(std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.yaw),"Nonfinite odometry");
    if(!out.empty())require(p.stamp>out.back().stamp,"Odometry time did not advance");
    out.push_back(p);
  }
  return out;
}
void fresh(const std::vector<Pose> &p) {
  require(p.size()>=25,"Insufficient odometry samples");
  auto age=now_ns()-p.back().stamp;require(age>=0 && age<250000000,"Odometry stale or future-dated");
}
bool stationary(const std::vector<Pose> &p) {
  if(p.size()<25)return false;
  const auto &a=p[p.size()-25],&b=p.back();
  if(b.stamp-a.stamp<400000000)return false;
  for(size_t i=p.size()-25;i<p.size();++i)
    if(std::hypot(p[i].x-a.x,p[i].y-a.y)>=.0005 || std::abs(std::remainder(p[i].yaw-a.yaw,2*std::acos(-1.)))>=.005)return false;
  return true;
}
// Diagnostic abort thresholds, not guaranteed physical travel limits.
void rotation_bounds(const std::vector<Pose> &p,const Pose &origin) {
  fresh(p);
  for(const auto &v:p)if(v.stamp>=origin.stamp) {
    require(std::hypot(v.x-origin.x,v.y-origin.y)<.030,"Rotation translation exceeded 30 mm");
    const auto yaw=std::remainder(v.yaw-origin.yaw,2*std::acos(-1.));
    require(yaw>-.030 && yaw<.300,"Rotation yaw exceeded [-0.03,+0.30] rad");
  }
}
int64_t field(const std::string &line,const std::string &key) {
  auto pos=line.find('"'+key+"\":");require(pos!=std::string::npos,"Missing JSON field "+key);
  return std::stoll(line.substr(pos+key.size()+3));
}
void verify(const fs::path &dir,bool rotation=false) {
  const auto p=poses(dir/"odom.csv");require(p.size()>=25,"Missing odometry");
  int64_t first=-1,last=-1;size_t n=0;bool complete=false;
  std::istringstream in(read(dir/"timing.jsonl"));std::string line;
  while(std::getline(in,line)) {
    if(contains(line,"\"kind\":\"publish\"")) {
      auto t=field(line,"ros_ns");if(first<0)first=t;
      require(last<0 || t>last,"Scan timestamps did not advance");last=t;++n;
    }
    if(contains(line,"\"kind\":\"end\""))
      complete=field(line,"complete")==1 && field(line,"overflow")==0 && field(line,"clock_bad")==0 && field(line,"published_scans")==200;
  }
  require(complete && n==200,"Timing footer/count invalid");
  require(p.front().stamp<first-200000000 && p.back().stamp>last,"Odometry does not bracket capture");
  std::vector<Pose> inside;for(const auto &v:p)if(v.stamp>=first && v.stamp<=last)inside.push_back(v);
  require(inside.size()>25,"No odometry inside scan interval");
  double max_distance=0,max_yaw=0,min_yaw=0;
  for(const auto &v:inside){max_distance=std::max(max_distance,std::hypot(v.x-inside.front().x,v.y-inside.front().y));
    const auto yaw=std::remainder(v.yaw-inside.front().yaw,2*std::acos(-1.));max_yaw=std::max(max_yaw,yaw);min_yaw=std::min(min_yaw,yaw);}
  if(rotation)require(max_yaw>=.050 && max_yaw<.300 && min_yaw>-.030 && max_distance<.030,
    "Rotation overlap/bounds failed: require >=0.05 and <0.30 rad positive yaw, >-0.03 rad reverse yaw, <30 mm translation");
  else require(max_distance>=.005,"No movement of at least 5 mm during scan recording");
  require(stationary(p),"Final odometry does not indicate a stop");
  std::cout<<"OVERLAP CHECK PASSED: 200 scans; movement during capture "<<max_distance*1000
           <<" mm; yaw "<<max_yaw<<" rad. This does not validate acquisition timing or physical stopping.\n";
}
int main(int argc,char **argv) {
  if(argc==3 && (std::string(argv[1])=="--verify-only" || std::string(argv[1])=="--verify-rotation")) {
    try{verify(argv[2],std::string(argv[1])=="--verify-rotation");return 0;}catch(const std::exception &e){std::cerr<<e.what()<<'\n';return 1;}
  }
  fs::path dir;Child odom,lidar,pulse;bool pulse_attempted=false;
  try {
    const bool rotation=argc==2 && std::string(argv[1])=="--execute-rotation-pulse";
    require(argc==2 && (rotation || std::string(argv[1])=="--execute-forward-pulse"),
      "Usage: run_motion_check --execute-forward-pulse | --execute-rotation-pulse; or --verify-only | --verify-rotation DIRECTORY");
    signal(SIGINT,signal_handler);signal(SIGTERM,signal_handler);
    require(std::getenv("ROS_DOMAIN_ID") && std::string(std::getenv("ROS_DOMAIN_ID"))=="182","Require ROS_DOMAIN_ID=182");
    require(std::getenv("ROS_AUTOMATIC_DISCOVERY_RANGE") && std::string(std::getenv("ROS_AUTOMATIC_DISCOVERY_RANGE"))=="LOCALHOST","Require LOCALHOST discovery");
    require(std::getenv("HOME"),"Missing HOME");
    std::string pattern=std::string(std::getenv("HOME"))+"/jetson-motion-auto.XXXXXX";
    char *made=mkdtemp(pattern.data());require(made,"Cannot create capture directory");dir=made;
    std::cout<<"CAPTURE DIRECTORY: "<<dir<<std::endl;
    const std::string port="/dev/serial/by-path/platform-3610000.usb-usb-0:2.4.4:1.0-port0";
    require(fs::exists(port),"Front LiDAR port absent");
    require(command({"fuser",fs::canonical(port).string()},dir/"port.log")==1,
      "Front port owned or fuser failed. Stop only the original front driver first.");
    require(command({"ros2","control","list_hardware_components"},dir/"hardware.log")==0,"Hardware inspection failed");
    auto hw=read(dir/"hardware.log");require(contains(hw,"hardware/OpenCRHardware") && contains(hw,"label=active"),"OpenCR hardware is not active");
    require(command({"ros2","control","list_controllers"},dir/"controllers.log")==0,"Controller inspection failed");
    {std::istringstream lines(read(dir/"controllers.log"));std::string l;bool active=false;
      while(std::getline(lines,l)){std::istringstream row(l);std::string name,type,state;row>>name>>type>>state;if(name=="swerve_controller" && state=="active")active=true;}
      require(active,"Swerve controller is not active");}
    require(command({"ros2","topic","info","/cmd_vel"},dir/"command_graph.log")==0,"Command inspection failed");
    auto graph=read(dir/"command_graph.log");require(contains(graph,"Publisher count: 0\n") && contains(graph,"Subscription count: 1\n"),"Unexpected command publisher/subscriber count");
    odom.start({"ros2","run","coin_d4_motion_capture","capture_motion_odom","--ros-args","-p","use_sim_time:=false","-p","duration_s:=60","-p","output_csv:="+(dir/"odom.csv").string()},dir/"odom.log");
    auto deadline=Clock::now()+std::chrono::seconds(10);
    while(poses(dir/"odom.csv").size()<25) {
      require(!interrupted && odom.running() && Clock::now()<deadline,"Odometry recorder failed/not ready");poll_delay();
    }
    auto before=poses(dir/"odom.csv");fresh(before);require(stationary(before),"Robot not stationary before capture");
    lidar.start({"ros2","run","coin_d4_motion_capture","motion_capture_node","--ros-args","-r","__ns:=/lidar_front_motion","-r","__node:=driver","-p","motion_diagnostic:=true","-p","use_sim_time:=false","-p","port:="+port,"-p","frame_id:=lidar_front_link","-p","baudrate:=230400","-p","version:=4","-p","reverse:=false","-p","warmup_time:=5","-p","timing_scans:=200","-p","timing_jsonl:="+(dir/"timing.jsonl").string()},dir/"timing.log");
    deadline=Clock::now()+std::chrono::seconds(15);
    while(!contains(read(dir/"timing.log"),"MOTION CAPTURE READY:")) {
      require(!interrupted && lidar.running() && odom.running() && Clock::now()<deadline,"LiDAR did not report first-scan readiness; no pulse sent");poll_delay();
    }
    // Two seconds of stationary scans, with both recorder processes monitored.
    deadline=Clock::now()+std::chrono::seconds(2);
    while(Clock::now()<deadline){require(!interrupted && lidar.running() && odom.running(),"Recorder stopped before pulse");poll_delay();}
    require(!contains(read(dir/"timing.log"),"TIMING CAPTURE SEALED"),"Capture sealed before pulse");
    before=poses(dir/"odom.csv");fresh(before);require(stationary(before),"Robot moved before scheduled pulse");
    const Pose origin=before.back();
    std::cout<<"Both recorders ready. Sending ONE "<<(rotation?"+0.10 rad/s rotation pulse (20 messages)":"forward pulse")<<" now.\n"<<std::flush;
    pulse_attempted=true;
    if(rotation) {
      // One matched publisher sends 20 yaw messages THEN 3 zeros: no rediscovery
      // between motion and normal stop. Never loop or extend the YAML sequence.
      write_exclusive(dir/"pulse.yaml",rotation_sequence());
      pulse.start(rotation_command(dir/"pulse.yaml"),dir/"pulse.log");
    } else {
      pulse.start({"ros2","topic","pub","/cmd_vel","geometry_msgs/msg/TwistStamped",
        "{header: {stamp: now, frame_id: base_footprint}, twist: {linear: {x: 0.025}}}",
        "--rate","10","--times","10","--max-wait-time-secs","5","--keep-alive","0.2"},dir/"pulse.log");
    }
    wait_publisher(pulse,dir/"pulse.log",rotation?23:10,rotation?3.2:1.8,[&]{
      require(lidar.running(),"LiDAR recorder exited during command sequence");
      require(odom.running(),"Odometry recorder exited during command sequence");
      const auto current=poses(dir/"odom.csv");fresh(current);if(rotation)rotation_bounds(current,origin);
    });
    if(rotation)std::cout<<"Publisher completed 20 yaw + 3 zero publications; checking odometry stop.\n";
    deadline=Clock::now()+std::chrono::seconds(3);
    while(Clock::now()<deadline){require(!interrupted && odom.running() && lidar.running(),"Recorder stopped after pulse");if(rotation)rotation_bounds(poses(dir/"odom.csv"),origin);poll_delay();}
    auto after=poses(dir/"odom.csv");fresh(after);require(stationary(after),"Odometry does not indicate a stop after pulse");
    deadline=Clock::now()+std::chrono::seconds(30);
    while(!contains(read(dir/"timing.log"),"TIMING CAPTURE SEALED:")) {
      require(!interrupted && lidar.running() && odom.running() && Clock::now()<deadline,"Timing capture failed to seal");poll_delay();
      if(rotation){const auto current=poses(dir/"odom.csv");rotation_bounds(current,origin);require(stationary(current),"Motion resumed after rotation stop");}
    }
    lidar.stop();
    deadline=Clock::now()+std::chrono::seconds(1);
    while(Clock::now()<deadline){require(!interrupted && odom.running(),"Odometry stopped early");poll_delay();}
    odom.stop();verify(dir,rotation);
    std::cout<<"Restore the original front LiDAR driver in domain 181. Saved data: "<<dir<<std::endl;
    return 0;
  } catch(const std::exception &e) {
    std::cerr<<"FAILED: "<<e.what()<<'\n';pulse.stop();
    if(pulse_attempted && !dir.empty()) {
      // Best effort only; hardware power cutoff remains the physical fallback.
      try {
        Child zero;zero.start(zero_command(),dir/"abort_zero.log");
        wait_publisher(zero,dir/"abort_zero.log",3,.8,[]{},false);
        std::cerr<<"ABORT ZERO: three publications completed; physical stopping still needs confirmation.\n";
      } catch(const std::exception &stop_error) {
        std::cerr<<"ABORT ZERO FAILED: "<<stop_error.what()<<". Use motor power cutoff if not stopped.\n";
      }
    }
    lidar.stop();odom.stop();
    std::cerr<<"No automatic retry. Data/logs: "<<dir<<". Restore original front driver after confirming this recorder stopped.\n";
    return 1;
  }
}
