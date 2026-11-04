#include "common.hpp"
#include <iostream>
#include <set>
#include <map>
using namespace maze_sim;
int main(int argc,char **argv){try{
  if(argc!=2)throw std::runtime_error("evaluate RUN_DIRECTORY");
  const fs::path root=argv[1];auto cfg=YAML::LoadFile((root/"maps/scenario.yaml").string());
  auto grid=Grid::read(root/"maps/truth.yaml");auto cost=grid.costmap();auto f=footprint();auto navigation=Grid::read(root/"maps/navigation.yaml");Canvas canvas(navigation,cfg["view_bounds"]);
  canvas.inflation(root);
  if(fs::exists(root/"observed-space.pgm"))canvas.observations(root);
  std::map<std::string,double> durations;std::string last_mode;double previous_time=0,rotation=0;int turn_episodes=0;bool was_turn=false,phase_available=false;
  std::vector<Pose> truth,estimated;std::ifstream csv(root/"runtime.csv");std::string line;std::getline(csv,line);
  double error2=0,max_error=0,travel=0;int error_count=0,unsafe=0;Pose previous;bool have=false;
  while(std::getline(csv,line)){auto v=split(line);if(v.size()<21)continue;Pose p{std::stod(v[1]),std::stod(v[2]),std::stod(v[3])};
    if(v.size()>21){double now=std::stod(v[0]);std::string mode=v[21];mode=mode.substr(0,mode.find(' '));if(mode.rfind("mode=",0)==0)mode=mode.substr(5);
      phase_available=phase_available||!mode.empty();
      bool turning=mode=="route_turn"||mode=="final_turn"||mode=="settle_turn";
      if(turning&&!was_turn)++turn_episodes;
      if(previous_time>0){double dt=now-previous_time;if(dt>=0&&dt<1){durations[last_mode]+=dt;rotation+=std::abs(std::stod(v[9]))*dt;}}
      was_turn=turning;last_mode=mode;previous_time=now;
    }
    if(have){double d=std::hypot(p.x-previous.x,p.y-previous.y),a=body_route_controller::wrap(p.yaw-previous.yaw);travel+=d;int n=std::max(1,int(std::ceil((d+.255*std::abs(a))/(grid.resolution*.5))));for(int i=0;i<=n;++i){double t=double(i)/n;if(!body_route_controller::footprintSafe(cost,f,{previous.x+t*(p.x-previous.x),previous.y+t*(p.y-previous.y),previous.yaw+t*a}))++unsafe;}}
    else if(!body_route_controller::footprintSafe(cost,f,p))++unsafe;
    previous=p;have=true;truth.push_back(p);
    if(std::stoi(v[7])){Pose q{std::stod(v[4]),std::stod(v[5]),std::stod(v[6])};estimated.push_back(q);double e=std::hypot(p.x-q.x,p.y-q.y);error2+=e*e;max_error=std::max(max_error,e);++error_count;}
  }
  std::ifstream paths(root/"plans.csv");std::getline(paths,line);std::vector<Pose> plan;int id=-1;
  while(std::getline(paths,line)){auto v=split(line);if(v.size()!=5)continue;int next=std::stoi(v[1]);if(id!=next&&!plan.empty()){canvas.path(plan,{180,210,230},1);plan.clear();}id=next;plan.push_back({std::stod(v[2]),std::stod(v[3]),std::stod(v[4])});}
  canvas.path(plan,{100,170,220},1);canvas.path(estimated,{40,150,50},2);canvas.path(truth,{220,90,25},2);
  int k=0;for(const auto &n:cfg["goals"]){auto p=pose(n);cv::circle(canvas.image,canvas.point(p),5,{30,100,230},-1);cv::putText(canvas.image,std::to_string(++k),canvas.point(p)+cv::Point(7,-7),cv::FONT_HERSHEY_SIMPLEX,.5,{30,80,200},1,cv::LINE_AA);}
  if(have){canvas.body(truth.front(),{220,90,25});canvas.body(truth.back(),{220,90,25});}
  bool success=false;std::string reason="missing trial result";YAML::Node legs;std::set<int> completed;
  if(fs::exists(root/"result.yaml"))for(const auto &entry:YAML::LoadAllFromFile((root/"result.yaml").string())){
    if(entry["kind"].as<std::string>("")=="summary"){success=entry["status"].as<std::string>("")=="success";reason=entry["reason"].as<std::string>("");}
    if(entry["kind"].as<std::string>("")=="leg"){legs.push_back(entry);if(entry["status"].as<std::string>("")=="success")completed.insert(entry["leg"].as<int>());}
  }
  YAML::Node startup;
  if(fs::exists(root/"startup.yaml")){startup=YAML::LoadFile((root/"startup.yaml").string());
    if(!startup["startup_passed"].as<bool>(false)){success=false;reason="startup_failed: "+startup["phase"].as<std::string>("unknown")+": "+startup["reason"].as<std::string>("no reason");}}
  std::ifstream a(root/"arrivals.csv");std::getline(a,line);int arrivals=0,arrival_failures=0;std::set<int> witnessed;
  while(std::getline(a,line)){auto v=split(line);if(v.size()!=7)continue;int leg=std::stoi(v[0]);if(!completed.count(leg)||witnessed.count(leg))continue;witnessed.insert(leg);++arrivals;if(std::stod(v[5])>.035||std::stod(v[6])>.055)++arrival_failures;}
  const bool passed=success&&unsafe==0&&truth.size()>10&&arrivals==static_cast<int>(cfg["goals"].size())&&arrival_failures==0;
  YAML::Node report;report["trial_success"]=success;report["trial_reason"]=reason;report["simulation_passed"]=passed;report["samples"]=truth.size();report["unsafe_sweep_samples"]=unsafe;report["truth_arrivals"]=arrivals;report["truth_arrival_failures"]=arrival_failures;report["distance_m"]=travel;report["valid_estimate_samples"]=error_count;if(error_count){report["position_rmse_m"]=std::sqrt(error2/error_count);report["max_position_error_m"]=max_error;}report["legs"]=legs;
  if(startup)report["startup"]=startup;
  report["controller_phase_available"]=phase_available;if(phase_available)report["turn_episodes"]=turn_episodes;else report["turn_episodes"]=YAML::Node();report["absolute_rotation_rad"]=rotation;for(auto item:durations)report["mode_duration_s"][item.first]=item.second;
  saveYaml(root/"evaluation.yaml",report);
  canvas.text(std::string("Scanned-maze controller trajectory: ")+(passed?"PASS":"FAIL / incomplete"),22);
  canvas.text("Blue: simulated truth   Green: AMCL estimate   Tan: Nav2 plans",45);
  canvas.text("Purple: unknown; cyan: observed-free; black/red: occupied",83);
  canvas.text("Costmap opacity 25%; footprint opacity 40% (latest costmap)",64);
  canvas.text("Scanned geometry + synthetic wall shifts; mock dynamics, not calibrated physics",canvas.image.rows-16);
  canvas.save(root/"trajectory.png");
  std::cout<<YAML::Dump(report)<<'\n';return passed?0:1;
}catch(const std::exception &e){std::cerr<<"Evaluation failed: "<<e.what()<<'\n';return 2;}}
