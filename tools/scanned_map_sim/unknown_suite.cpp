#include "common.hpp"
#include "unknown_fixture.hpp"
#include <deque>
#include <iostream>
#include <set>
using namespace maze_sim;
using body_route_controller::Velocity;
using body_route_controller::ObservationSpace;
using body_route_controller::Core;
using body_route_controller::Point;
namespace {
Grid grid(const nav_msgs::msg::OccupancyGrid &m){Grid g;g.resolution=m.info.resolution;g.pixels=cv::Mat(m.info.height,m.info.width,CV_8U);
  for(unsigned y=0;y<m.info.height;++y)for(unsigned x=0;x<m.info.width;++x){int v=m.data[y*m.info.width+x];g.pixels.at<unsigned char>(m.info.height-1-y,x)=v<0?205:v==0?254:0;}
  return g;}
// This object has no ground-truth map. It receives only the partial map,
// timestamped synthetic rays and ideal odometry. The runtime adapters add TF,
// ROS timestamp and independent odometry checks around these same components.
struct Navigator {
  ObservationSpace evidence;nav2_costmap_2d::Costmap2D raw;Core core;
  std::vector<Pose> path;std::vector<Point> points;int replans=0;
  std::deque<std::pair<double,Velocity>> admitted;
  Navigator(const nav_msgs::msg::OccupancyGrid &map,double speed):core(settings(speed)){evidence.setMap(map);raw=evidence.rawMap();}
  static body_route_controller::Settings settings(double speed){body_route_controller::Settings c;c.max_speed=speed;c.max_yaw_rate=.3;c.correction_speed=.6*speed;c.lookahead=std::max(.12,3*speed);c.cross_track_enter=.01;c.cross_track_exit=.004;c.goal_xy=.02;c.goal_yaw=.03;return c;}
  void plan(Pose p,Pose goal,double now){auto known=evidence.executionMap(now,now);
    body_route_controller::MazePlannerSettings cfg;cfg.unknown_policy=body_route_controller::UnknownPolicy::ObserveBeforeEnter;
    path=body_route_controller::planMaze(known,unknown_fixture::body(),p,goal,cfg,[]{return false;},&raw,&known);
    points.clear();for(auto q:path)points.push_back({q.x,q.y});++replans;
  }
  bool valid(double now){auto known=evidence.executionMap(now,now);
    for(unsigned y=0;y<known.getSizeInCellsY();++y)for(unsigned x=0;x<known.getSizeInCellsX();++x)if(known.getCost(x,y)==255&&raw.getCost(x,y)==255)known.setCost(x,y,0);
    for(auto p:path)if(!body_route_controller::footprintSafe(known,unknown_fixture::body(),p))return false;
    return true;
  }
};
double towards(double value,double target,double step){return value+std::clamp(target-value,-step,step);}
}
int main(int argc,char **argv){try{
  if(argc<2||argc>3)throw std::runtime_error("unknown_suite NEW_OUTPUT_DIRECTORY [SPEED_MPS]");
  const fs::path root=argv[1];if(!fs::create_directory(root))throw std::runtime_error("output exists");
  const double speed=argc==3?std::stod(argv[2]):.08;if(!std::isfinite(speed)||speed<.005||speed>.26)throw std::runtime_error("speed .005.. .26 required");
  YAML::Node summary;bool all=true;
  for(const std::string kind:{"corridor","concealed","partial","stale","stale_unknown","missing","invalid","transform","turn"}){
    auto truth=unknown_fixture::world();if(kind=="concealed")unknown_fixture::obstacle(truth,1.85,1.15,2.05,1.65);
    auto navigation=unknown_fixture::partial(truth);Navigator nav(navigation,speed);ObservationSpace oracle;oracle.setMap(truth);auto ground=oracle.rawMap();
    Pose p{.7,1.4,0},goal=kind=="turn"?Pose{1.8,2.1,maze_sim::pi/2}:Pose{3.3,1.4,0};Velocity measured;std::deque<Velocity> queue(3);std::vector<Pose> trajectory;
    const auto out=root/kind;fs::create_directory(out);grid(truth).save(out,"truth");grid(navigation).save(out,"navigation");
    std::ofstream trace(out/"trace.csv");trace<<"time,x,y,yaw,vx,vy,wz,gate\n";std::set<std::string> reasons;int unsafe=0,waiting=0;bool complete=false;double finish=0,max_fault_speed=0;bool fault_stopped=false;
    for(int step=0;step<4000;++step){const double elapsed=step*.05,stamp=10+elapsed,now=stamp+.001;finish=elapsed;
      const double fault_start=kind=="stale_unknown"?13:3;
      const bool fault=elapsed>=fault_start&&elapsed<fault_start+3;
      if(step%2==0){
        if(kind=="missing"&&fault){} // No callbacks; expiry must detect the loss.
        else if(kind=="transform"&&fault)nav.evidence.invalidate(0,"missing_scan_transform");
        else if(kind=="invalid"&&fault)nav.evidence.observe(0,{},stamp,now);
        else if((kind=="stale"||kind=="stale_unknown")&&fault)nav.evidence.observe(0,unknown_fixture::scan(truth,p,0,stamp-1),stamp-1,now);
        else unknown_fixture::observe(nav.evidence,truth,p,stamp,.65,kind=="partial"&&elapsed<8);
      }
      if(nav.path.empty()||(step%20==0&&!nav.valid(now))){try{nav.plan(p,goal,now);}catch(const std::exception &e){reasons.insert(std::string("planner_wait:")+e.what());}}
      auto reason=nav.evidence.health(now);Velocity request;
      const bool footprint_clear=nav.evidence.certify(unknown_fixture::body(),p,now);
      if(reason.empty()){
        if(!footprint_clear)reason="initial_or_current_footprint_unobserved";
        else if(!nav.points.empty()){
          // Match Nav2's path-handler contract: Core receives a pruned path.
          auto pruned=nav.points;std::size_t nearest=0;double distance=1e9;
          for(std::size_t j=0;j<pruned.size();++j){const double d=std::hypot(pruned[j].x-p.x,pruned[j].y-p.y);if(d<distance){distance=d;nearest=j;}}
          if(nearest>3)pruned.erase(pruned.begin(),pruned.begin()+nearest-3);
          auto command=nav.core.step(p,measured,pruned,goal,.05,speed);request=command.command;
          while(!nav.admitted.empty()&&now-nav.admitted.front().first>nav.evidence.settings().latency)nav.admitted.pop_front();
          std::vector<Velocity> pending;for(auto v:nav.admitted)pending.push_back(v.second);
          reason=nav.evidence.check(unknown_fixture::body(),p,measured,request,now,pending);
          if(reason.empty())nav.admitted.push_back({now,request});
        }
      }
      if(!reason.empty()){request={};nav.core.reset();++waiting;reasons.insert(reason);}
      queue.push_back(request);auto applied=queue.front();queue.pop_front();
      measured.x=towards(measured.x,applied.x,.05*.05);measured.y=towards(measured.y,applied.y,.05*.05);measured.yaw=towards(measured.yaw,applied.yaw,.2*.05);
      const Pose previous=p;p=compose(p,{measured.x*.05,measured.y*.05,measured.yaw*.05});
      if(!body_route_controller::motionSafe(ground,unknown_fixture::body(),previous,measured,.05))++unsafe;
      if(fault&&elapsed>fault_start+2.5&&std::hypot(measured.x,measured.y)<.003)fault_stopped=true;
      if(fault)max_fault_speed=std::max(max_fault_speed,std::hypot(measured.x,measured.y));
      trajectory.push_back(p);trace<<elapsed<<','<<p.x<<','<<p.y<<','<<p.yaw<<','<<measured.x<<','<<measured.y<<','<<measured.yaw<<','<<(reason.empty()?"clear":reason)<<'\n';
      if(std::hypot(p.x-goal.x,p.y-goal.y)<.025&&std::abs(body_route_controller::wrap(p.yaw-goal.yaw))<.04&&std::hypot(measured.x,measured.y)<.003&&std::abs(measured.yaw)<.01){complete=true;break;}
    }
    auto nav_grid=grid(navigation);auto bounds=YAML::Load("[0.0,0.0,4.38,2.78]");Canvas canvas(nav_grid,bounds);
    auto observed=nav.evidence.display(10+finish+.001);
    for(int y=canvas.top;y<canvas.image.rows-45;++y)for(int x=0;x<canvas.image.cols;++x){auto w=nav_grid.center(canvas.crop.y+int((y-canvas.top)/canvas.scale),canvas.crop.x+int(x/canvas.scale));int u=w.x/nav_grid.resolution,v=w.y/nav_grid.resolution;
      if(u<0||v<0||u>=int(observed.info.width)||v>=int(observed.info.height))continue;
      const int cell=observed.data[v*observed.info.width+u];if(cell!=25&&cell!=100)continue;
      auto &pixel=canvas.image.at<cv::Vec3b>(y,x);cv::Vec3b color=cell==25?cv::Vec3b(220,200,20):cv::Vec3b(0,0,0);for(int k=0;k<3;++k)pixel[k]=cv::saturate_cast<unsigned char>(.7*pixel[k]+.3*color[k]);
    }
    canvas.path(nav.path,{170,30,170},1);canvas.path(trajectory,{210,80,20},2);canvas.body(trajectory.front(),{210,80,20});canvas.body(p,{210,80,20});
    const bool passed=complete&&unsafe==0&&(kind!="concealed"||nav.replans>1)&&((kind=="corridor"||kind=="turn"||kind=="concealed")||waiting>0)&&
      ((kind!="stale"&&kind!="stale_unknown"&&kind!="missing"&&kind!="invalid"&&kind!="transform")||fault_stopped);
    canvas.text(kind+": "+(passed?"PASS":"FAIL")+"; waits="+std::to_string(waiting)+" replans="+std::to_string(nav.replans),22);
    canvas.text("Purple unknown / cyan observed-free / black occupied",45);canvas.text("Blue trajectory + transparent footprint / magenta planned path",64);canvas.save(out/"trajectory.png");
    YAML::Node result;result["passed"]=passed;result["complete"]=complete;result["unsafe_sweep_samples"]=unsafe;result["waiting_steps"]=waiting;result["replans"]=nav.replans;result["simulated_seconds"]=finish;result["peak_speed_during_fault_mps"]=max_fault_speed;result["measured_stopped_during_fault"]=fault_stopped;
    for(const auto &reason:reasons)result["reasons"].push_back(reason);
    summary[kind]=result;saveYaml(out/"result.yaml",result);all=all&&passed;std::cout<<kind<<": "<<YAML::Dump(result)<<std::endl;
  }
  summary["all_passed"]=all;summary["model"]="ideal odometry; 150 ms command latency and bounded acceleration; production custom Core, planner and shared evidence gate; not MPPI or calibrated hardware";
  saveYaml(root/"summary.yaml",summary);return all?0:1;
}catch(const std::exception &e){std::cerr<<e.what()<<'\n';return 2;}}
