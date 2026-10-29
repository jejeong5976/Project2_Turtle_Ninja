#define main motion_check_entry
#include "run_motion_check.cpp"
#undef main
#include <functional>
void rejects(const std::function<void()> &f){bool bad=false;try{f();}catch(const std::exception&){bad=true;}require(bad,"Expected rejection");}
int main() {
  char pattern[]="/tmp/motion-coordinator-test.XXXXXX";const auto made=mkdtemp(pattern);require(made,"mkdtemp");fs::path dir=made;
  auto write=[](fs::path p,const std::string &s){std::ofstream f(p);f<<s;};
  const int64_t epoch=1793000000000000000LL;
  std::ostringstream od;
  od<<"stamp_ns,x_m,y_m,yaw_rad\n";
  for(int i=0;i<1200;++i){double x=i<200?0:(i<250?(i-200)*.0006:.03);od<<epoch+int64_t(i)*20000000<<','<<x<<",0,0\n";}
  write(dir/"odom.csv",od.str());
  std::ostringstream timing;
  for(int i=0;i<200;++i)timing<<"{\"kind\":\"publish\",\"ros_ns\":"<<epoch+1000000000+int64_t(i)*100000000<<"}\n";
  const std::string footer="{\"kind\":\"end\",\"published_scans\":200,\"complete\":1,\"overflow\":0,\"clock_bad\":0}\n";
  write(dir/"timing.jsonl",timing.str()+footer);verify(dir);
  rejects([&]{verify(dir,true);}); // Translation cannot pass rotation validation.
  std::ostringstream turn;turn<<"stamp_ns,x_m,y_m,yaw_rad\n";
  for(int i=0;i<1200;++i){double yaw=i<200?0:(i<300?(i-200)*.002:.2);turn<<epoch+int64_t(i)*20000000<<",0,0,"<<yaw<<'\n';}
  write(dir/"odom.csv",turn.str());verify(dir,true);rejects([&]{verify(dir);});
  auto moving=poses(dir/"odom.csv");require(stationary(moving),"Rotation stop rejected");
  moving[moving.size()-10].yaw+=.01;require(!stationary(moving),"Intermediate yaw excursion missed");
  const auto now=now_ns();std::vector<Pose> bounded;
  for(int i=0;i<25;++i)bounded.push_back({now-(24-i)*20000000LL,0,0,i*.005});
  const auto origin=bounded.front();rotation_bounds(bounded,origin);
  bounded.back().x=.031;rejects([&]{rotation_bounds(bounded,origin);});bounded.back().x=0;
  bounded.back().yaw=.301;rejects([&]{rotation_bounds(bounded,origin);});
  bounded.back().yaw=-.031;rejects([&]{rotation_bounds(bounded,origin);});
  write(dir/"odom.csv",od.str());
  write(dir/"timing.jsonl",timing.str());rejects([&]{verify(dir);});
  write(dir/"timing.jsonl",timing.str()+footer);
  std::ostringstream stationary_odom;stationary_odom<<"stamp_ns,x_m,y_m,yaw_rad\n";
  for(int i=0;i<1200;++i)stationary_odom<<epoch+int64_t(i)*20000000<<",0,0,0\n";
  write(dir/"odom.csv",stationary_odom.str());rejects([&]{verify(dir);});
  write(dir/"odom.csv","stamp_ns,x_m,y_m,yaw_rad\n100,0,0,0\n99,0,0,0\n");rejects([&]{poses(dir/"odom.csv");});
  write(dir/"odom.csv","stamp_ns,x_m,y_m,yaw_rad\n100,0,0,0\n101,0,");require(poses(dir/"odom.csv").size()==1,"Partial write not ignored");
  Child ok;ok.start({"/bin/sh","-c","exit 7"},dir/"exit.log");require(ok.finish(2)==7,"Exit status lost");
  Child slow;slow.start({"/bin/sleep","10"},dir/"slow.log");rejects([&]{slow.finish(.1);});slow.stop();require(!slow.running(),"Child not stopped");
  Child sig;sig.start({"/bin/sleep","10"},dir/"sig.log");interrupted=1;rejects([&]{sig.finish(2);});interrupted=0;sig.stop();
  PublicationWatch delayed{23,3.2,0};
  delayed.update(0,3.);delayed.update(1,4.5);delayed.update(20,6.4);delayed.update(23,6.7);delayed.update(23,7.);
  require(delayed.count==23,"Discovery consumed pulse budget");
  rejects([&]{delayed.update(23,8.8);});
  PublicationWatch absent{23,3.2,0};rejects([&]{absent.update(0,8.1);});
  PublicationWatch late{23,3.2,0};rejects([&]{late.update(1,8.1);});
  PublicationWatch stalled{23,3.2,0};stalled.update(1,1.);rejects([&]{stalled.update(23,4.3);});
  PublicationWatch excess{23,3.2,0};rejects([&]{excess.update(24,1.);});
  require(publication_count("Waiting...\npublishing #1: foo\npublishing #2") == 1,"Partial progress counted");
  rejects([&]{publication_count("publishing #1: foo\npublishing #3: bar\n");});
  const auto sequence=rotation_sequence();size_t yaw_count=0,zero_count=0,pos=0;
  while((pos=sequence.find("angular: {z: ",pos))!=std::string::npos){pos+=13;if(sequence.compare(pos,4,"0.10")==0){require(zero_count==0,"Yaw after stop");++yaw_count;}else ++zero_count;}
  require(yaw_count==20 && zero_count==3,"Wrong motion/zero sequence");
  write_exclusive(dir/"pulse.yaml",sequence);rejects([&]{write_exclusive(dir/"pulse.yaml",sequence);});
  Child fake;fake.start({"/bin/sh","-c","sleep 0.2; printf 'publishing #1: a\npublishing #2: b\npublishing #3: c\n'; sleep 0.1"},dir/"fake.log");
  wait_publisher(fake,dir/"fake.log",3,.8,[]{});
  Child short_pub;short_pub.start({"/bin/sh","-c","printf 'publishing #1: a\n'"},dir/"short.log");
  rejects([&]{wait_publisher(short_pub,dir/"short.log",3,.8,[]{});});
  std::cout<<"PASS overlap gate, stationary/missing-footer/regressing/partial data, child exit/timeout/interrupt cleanup\n";
  std::cout<<"PASS discovery/pulse/exit budgets, partial/progress failures, ordered 20-yaw/3-zero sequence, publisher process completion\n";
}
