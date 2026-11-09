// Offline geometry hypothesis only. No ROS initialization, network, or hardware.
// Mirrors BodyRouteController's collision/fallback selection; NOT the observation
// wrapper, changing scans/costmaps, replanning, or the full actuator plant.
#include "body_route_controller/collision.hpp"
#include "body_route_controller/passage.hpp"
#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
using namespace body_route_controller;
using Json = nlohmann::json;
int main(int argc, char **argv) {
  if (argc != 12) return 2;
  std::ifstream path_in(argv[1]); std::vector<Point> path; Pose goal; double x,y,a;
  while(path_in>>x>>y>>a){path.push_back({x,y});goal={x,y,a};}
  if(path.size()<2)return 2;
  const auto doc=YAML::LoadFile(argv[4]);
  const auto cfg=doc["controller_server"]["ros__parameters"]["FollowPath"];
  Settings s;
  s.drive_style=cfg["drive_style"].as<std::string>();s.heading_policy=cfg["heading_policy"].as<std::string>();
#define LOAD(field) if(cfg[#field])s.field=cfg[#field].as<double>()
  LOAD(heading_preview_time);LOAD(max_travel_angle);LOAD(max_speed);LOAD(correction_speed);LOAD(max_yaw_rate);
  LOAD(heading_hold_rate);LOAD(heading_rate_per_speed);LOAD(heading_kp);LOAD(linear_kp);LOAD(linear_accel);LOAD(angular_accel);
  LOAD(align_enter);LOAD(align_exit);LOAD(lookahead);LOAD(arrival_lookahead);LOAD(corner_tolerance);LOAD(path_simplify_tolerance);
  LOAD(cross_track_enter);LOAD(cross_track_exit);LOAD(max_path_distance);LOAD(max_correction_distance);LOAD(max_correction_time);
  LOAD(stopped_linear);LOAD(stopped_angular);LOAD(goal_xy);LOAD(goal_yaw);LOAD(max_dt);
#undef LOAD
  s.validate();
  const double feedback_timeout=cfg["feedback_timeout"].as<double>();
  const double horizon=cfg["collision_horizon"].as<double>();
  const auto cm=doc["local_costmap"]["local_costmap"]["ros__parameters"];
  const auto vertices=YAML::Load(cm["footprint"].as<std::string>());
  const double padding=cm["footprint_padding"].as<double>(); Footprint footprint;
  for(const auto &v:vertices){geometry_msgs::msg::Point q;q.x=v[0].as<double>();q.y=v[1].as<double>();q.x+=std::copysign(padding,q.x);q.y+=std::copysign(padding,q.y);footprint.push_back(q);}
  Json meta; std::ifstream(argv[5])>>meta;
  nav2_costmap_2d::Costmap2D map(meta.at("width"),meta.at("height"),meta.at("resolution"),meta.at("origin")[0],meta.at("origin")[1],0);
  std::ifstream data(argv[6],std::ios::binary);
  const auto bytes=map.getSizeInCellsX()*map.getSizeInCellsY();
  data.read(reinterpret_cast<char*>(map.getCharMap()),bytes);if(data.gcount()!=bytes)return 3;
  const double gain=std::stod(argv[7]);if(gain<=0||gain>1)return 3;
  if(std::string(argv[8])!="baseline"&&std::string(argv[8])!="prepared")return 2;
  PassageSettings ps;ps.enabled=std::string(argv[8])=="prepared";
  ps.position_allowance=std::stod(argv[10]);ps.yaw_allowance=std::stod(argv[11]);
  PreparedPassage passage(s,ps);
  const int delay_ticks=std::stoi(argv[9]);if(delay_ticks<1||delay_ticks>10)return 3;
  std::deque<Velocity> commands(delay_ticks-1);
  Core core(s);s.round_corners=false;Core corridor_core(s);bool corridor=false;Point corridor_start;
  Pose p{path.front().x,path.front().y,std::stod(argv[2])};Velocity v,delay;
  std::deque<std::pair<double,Velocity>> pending;
  std::ofstream out(std::string(argv[3])+".csv"),events(std::string(argv[3])+".events.jsonl"),envelopes(std::string(argv[3])+".envelopes.jsonl");
  out<<std::setprecision(12)<<"t,x,y,yaw,vx,vy,wz,cmd_x,cmd_y,cmd_w,mode,corridor,speed_factor,reason,body_safe,envelope_safe\n";
  std::size_t first=0;int body_collisions=0,envelope_blocks=0,wait_ticks=0,fallback_entries=0;
  std::map<std::string,int> reasons;Pose progress=p;double last_progress=0;bool reached=false;
  std::vector<double> compute_ms;
  std::string outcome="duration_limit";double last_t=0;std::string previous_mode;
  for(int i=0;i<4800;++i){
    const double t=i*.05;last_t=t;
    double best=1e9,searched=0;std::size_t nearest=first;
    for(std::size_t k=first;k<path.size();++k){double d=std::hypot(p.x-path[k].x,p.y-path[k].y);if(d<best){best=d;nearest=k;}if(k+1<path.size())searched+=std::hypot(path[k+1].x-path[k].x,path[k+1].y-path[k].y);if(searched>.5)break;}
    first=nearest>0?nearest-1:0;std::vector<Point> local(path.begin()+first,path.end());
    const auto compute_start=std::chrono::steady_clock::now();
    Core before=corridor?corridor_core:core;
    if(corridor&&std::hypot(p.x-corridor_start.x,p.y-corridor_start.y)>.4&&std::abs(v.yaw)<core.settings().stopped_angular){corridor=false;core.reset();before=core;}
    Result r;
    try{r=corridor?corridor_core.step(p,v,local,goal,.05,s.max_speed):core.step(p,v,local,goal,.05,s.max_speed);}
    catch(const std::exception &e){outcome=std::string("core_exception:")+e.what();break;}
    const double braking_time=std::max(std::hypot(v.x,v.y)/s.linear_accel,std::abs(v.yaw)/s.angular_accel);
    while(!pending.empty()&&(t-pending.front().first>.5||t<pending.front().first))pending.pop_front();
    auto unsafe=[&](const Result &candidate)->std::string{
      if(!motionSafe(map,footprint,p,v,feedback_timeout+braking_time)||!motionSafe(map,footprint,p,candidate.command,horizon))return "footprint_or_stopping_sweep_blocked";
      std::vector<Velocity> velocities{v,candidate.command};for(const auto &item:pending)velocities.push_back(item.second);
      const auto bounds=stoppingEnvelope(footprint,p,velocities,.5,s.linear_accel,s.angular_accel);
      if(bounds.polygon.empty()||!footprintSafe(map,bounds.polygon,{}))return "continuous_stopping_envelope_blocked";
      if((candidate.mode=="route_turn"||candidate.mode=="final_turn")&&!turnSafe(map,footprint,p,candidate.heading))return "full_rotation_sweep_blocked";
      return {};
    };
    std::optional<Result> prepared;
    std::vector<Velocity> history;for(const auto &item:pending)history.push_back(item.second);
    try {prepared=passage.step(map,footprint,p,v,local,goal,.05,s.max_speed,history);}
    catch(const std::exception &e){outcome=std::string("passage_exception:")+e.what();break;}
    if(prepared)r=*prepared;
    auto reason=unsafe(r);const auto initial_reason=reason;double factor=1;
    if(prepared&&!reason.empty()){passage.pause();core.reset();corridor_core.reset();r={{},"clearance_wait",p.yaw,0};wait_ticks++;factor=0;}
    if(prepared&&reason.empty()){core.reset();corridor_core.reset();corridor=false;}
    if(prepared&&r.mode=="passage_wait")reasons[passage.reason()]++;
    if(!prepared&&!reason.empty()){
      reasons[reason]++;
      if(!corridor){corridor=true;corridor_start={p.x,p.y};corridor_core.reset();before=corridor_core;fallback_entries++;r=corridor_core.step(p,v,local,goal,.05,s.max_speed);reason=unsafe(r);events<<Json{{"t",t},{"event","enter_exact_corner"},{"pose",{p.x,p.y,p.yaw}},{"initial_reason",initial_reason},{"fallback_reason",reason}}.dump()<<'\n';}
      if(!reason.empty())for(double speed:{.5,.25,.125}){Core candidate=before;const auto proposal=candidate.step(p,v,local,goal,.05,s.max_speed*speed);if(unsafe(proposal).empty()){corridor_core=candidate;r=proposal;reason.clear();factor=speed;break;}}
      if(!reason.empty()){core.reset();corridor_core.reset();r={{},"clearance_wait",p.yaw,0};wait_ticks++;factor=0;}
    }
    if(reason.empty())pending.emplace_back(t,r.command);
    compute_ms.push_back(std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-compute_start).count());
    std::vector<Velocity> velocities{v,r.command};for(const auto &item:pending)velocities.push_back(item.second);
    const auto envelope=stoppingEnvelope(footprint,p,velocities,.5,s.linear_accel,s.angular_accel);
    const bool body_safe=footprintSafe(map,footprint,p),envelope_safe=!envelope.polygon.empty()&&footprintSafe(map,envelope.polygon,{});
    body_collisions+=!body_safe;envelope_blocks+=!envelope_safe;
    Json polygon=Json::array();for(const auto &q:envelope.polygon)polygon.push_back({q.x,q.y});
    envelopes<<Json{{"t",t},{"pose",{p.x,p.y,p.yaw}},{"polygon",polygon},{"horizon",envelope.horizon}}.dump()<<'\n';
    out<<t<<','<<p.x<<','<<p.y<<','<<p.yaw<<','<<v.x<<','<<v.y<<','<<v.yaw<<','<<r.command.x<<','<<r.command.y<<','<<r.command.yaw<<','<<r.mode<<','<<corridor<<','<<factor<<','<<reason<<','<<body_safe<<','<<envelope_safe<<'\n';
    if(r.mode!=previous_mode){events<<Json{{"t",t},{"event","mode"},{"mode",r.mode},{"pose",{p.x,p.y,p.yaw}},{"reason",reason}}.dump()<<'\n';previous_mode=r.mode;}
    if(!body_safe){outcome="modeled_body_overlap";break;}
    if(r.mode=="goal_stop"&&std::hypot(v.x,v.y)<s.stopped_linear&&std::abs(v.yaw)<s.stopped_angular){reached=true;outcome="modeled_goal";break;}
    // Same progress thresholds as the profile; no replanner or new goals in this model.
    if(std::hypot(p.x-progress.x,p.y-progress.y)>.02||std::abs(wrap(p.yaw-progress.yaw))>.05){progress=p;last_progress=t;}
    if(t-last_progress>20){outcome="modeled_progress_timeout";break;}
    v.x+=gain*(delay.x-v.x);v.y+=gain*(delay.y-v.y);v.yaw+=gain*(delay.yaw-v.yaw);commands.push_back(r.command);delay=commands.front();commands.pop_front();
    p.x+=.05*(std::cos(p.yaw)*v.x-std::sin(p.yaw)*v.y);p.y+=.05*(std::sin(p.yaw)*v.x+std::cos(p.yaw)*v.y);p.yaw=wrap(p.yaw+.05*v.yaw);
  }
  Json report{{"outcome",outcome},{"reached_goal",reached},{"time_s",last_t},{"pose",{p.x,p.y,p.yaw}},{"body_overlap_samples",body_collisions},{"envelope_blocked_samples",envelope_blocks},{"clearance_wait_ticks",wait_ticks},{"fallback_entries",fallback_entries},{"initial_gate_reasons",reasons},{"speed_cap",s.max_speed},{"gain",gain},{"delay_ticks",delay_ticks},{"prepared",ps.enabled},{"position_allowance",ps.position_allowance},{"yaw_allowance",ps.yaw_allowance},{"map_metadata",meta},{"limitations","Static geometry hypothesis. Future scan evidence, live unknown clearing, perception uncertainty, plant fidelity, path-handler transforms/replanning, feedback freshness and external stop gates are not modeled. No movement qualification."}};
  std::sort(compute_ms.begin(),compute_ms.end());
  if(!compute_ms.empty()){
    report["host_compute_ms_p50"]=compute_ms[compute_ms.size()/2];
    report["host_compute_ms_p95"]=compute_ms[std::min(compute_ms.size()-1,compute_ms.size()*95/100)];
    report["host_compute_ms_max"]=compute_ms.back();
  }
  std::ofstream(std::string(argv[3])+".json")<<report.dump(2)<<'\n';std::cout<<report.dump(2)<<'\n';return 0;
}
