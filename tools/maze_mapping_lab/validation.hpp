#pragma once
#include "common.hpp"
namespace maze {
inline void validate_scene(const std::map<std::string,std::string>&a){
  fs::path scene=option(a,"--scene");auto m=load_model(scene/"maze.xml");Data d(mj_makeData(m.get()));mj_forward(m.get(),d.get());mjtByte group[6]={1,0,0,0,0,0};
  auto north_ray=[&](double x,double y,double z){mjtNum p[3]={x,y,z},v[3]={0,1,0};return mj_ray(m.get(),d.get(),p,v,group,1,-1,nullptr,nullptr);};
  check(std::abs(north_ray(1.468,.5,.0545)-.3475)<1e-6,"Low scan did not hit specified 70 mm source-box side");
  check(std::abs(north_ray(1.468,.5,.09)-.814)<1e-6,"Raised ray did not clear low source box and hit wall");
  check(std::abs(north_ray(1.428,.5,.09)-.435)<1e-6,"Raised ray did not hit decorative puck at its true height");
  check(mj_name2id(m.get(),mjOBJ_JOINT,"arm_joint1")>=0&&mj_name2id(m.get(),mjOBJ_JOINT,"arm_joint5")>=0,"Full arm was not imported");
  auto audit=read_json(scene/"model_audit.json");check(double(audit["robot_radius_bound_m"])>=std::hypot(.1376,.1625),"Envelope omits preserved body mask");
  Json result={{"status","passed"},{"checks",{"54.5 mm ray hits 70 mm platform","90 mm ray clears platform","90 mm ray hits 70-130 mm puck","full arm joints imported","planning envelope contains retained body mask"}}};
  if(a.count("--dataset")){
    fs::path dir=option(a,"--dataset");auto manifest=read_json(dir/"manifest.json");auto samples=read_binary<Sample>(dir/"trajectory.bin");auto scans=read_binary<ScanRecord>(dir/"scans.bin");auto obs=obstacles(read_json(scene/"arena.json"));double min_clear=100,max_speed=0,max_turn=0;
    for(size_t k=0;k<samples.size();++k){const auto&s=samples[k];min_clear=std::min(min_clear,clearance(s.truth.x,s.truth.y,obs)-double(audit["robot_radius_bound_m"]));if(k){double dt=s.t-samples[k-1].t;check(dt>0&&dt<.020001,"Invalid odometry sampling");max_speed=std::max(max_speed,std::hypot(s.truth.x-samples[k-1].truth.x,s.truth.y-samples[k-1].truth.y)/dt);max_turn=std::max(max_turn,std::abs(wrap(s.truth.yaw-samples[k-1].truth.yaw))/dt);}}
    check(min_clear>=double(manifest["planning_margin_m"])-1e-6,"Route enters conservative obstacle envelope");check(max_speed<=double(manifest["speed_m_s"])+1e-6&&max_turn<=double(manifest["turn_rate_rad_s"])+1e-6,"Trajectory exceeds requested speed");for(size_t i=1;i<scans.size();++i)check(std::abs(scans[i].t-scans[i-1].t-.1)<1e-6,"Invalid scan cadence");
    result["dataset"]=dir.string();result["samples_checked"]=samples.size();result["scans_checked"]=scans.size();result["minimum_clearance_m"]=min_clear;result["maximum_speed_m_s"]=max_speed;result["maximum_turn_rate_rad_s"]=max_turn;
  }
  if(a.count("--output"))write_json(option(a,"--output"),result);
  std::cout<<result.dump(2)<<'\n';
}
inline void summarize(const std::map<std::string,std::string>&a){
  fs::path trials=option(a,"--trials"),out=option(a,"--output");fs::create_directories(out);Json rows=Json::array();
  for(const auto&e:fs::directory_iterator(trials))if(e.is_directory()&&fs::exists(e.path()/"results.json")){
    auto result=read_json(e.path()/"results.json"),settings=read_json(e.path()/"settings.json"),manifest=read_json(fs::path(settings["dataset"])/"manifest.json");
    rows.push_back({{"name",e.path().filename().string()},{"path",fs::absolute(e.path()).string()},{"settings",settings},{"dataset",manifest},{"result",result}});
  }
  std::sort(rows.begin(),rows.end(),[](const auto&a,const auto&b){return a["name"]<b["name"];});write_json(out/"comparison.json",rows);
  std::ofstream csv(out/"comparison.csv");csv<<"trial,speed_m_s,route,heading,physical_minutes,free_space_coverage,occupied_precision,surface_recall,endpoint_error_m,pose_rmse_m,projection_rejected_rays,peak_cgroup_MiB,processing_p95_ms,freshness_pass\n";
  for(const auto&r:rows){const auto&d=r["dataset"],&v=r["result"];csv<<std::string(r["name"])<<','<<d["speed_m_s"]<<','<<std::string(d["route"])<<','<<std::string(d["heading"])<<','<<double(d["duration_s"])/60<<',';if(v["quality"].is_null()){csv<<",,,,,,,,,false\n";continue;}const auto&q=v["quality"];csv<<q["arena_free_space_coverage"]<<','<<q["occupied_precision_within_1_5_cells"]<<','<<q["obstacle_surface_recall_within_1_5_cells"]<<','<<q["endpoint_position_error_m"]<<','<<q["sampled_pose_rmse_m"]<<','<<v["projection_rejected_rays"]<<','<<double(v.value("peak_cgroup_memory_bytes",0.))/1048576<<','<<v["processing_p95_ms"]<<','<<v["freshness_pass"]<<'\n';}
  csv.close();std::cout<<read_text(out/"comparison.csv");
}
}
