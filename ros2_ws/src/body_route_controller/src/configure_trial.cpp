#include "body_route_controller/profile.hpp"
#include <ament_index_cpp/get_package_share_path.hpp>
#include <filesystem>
#include <algorithm>
#include <iostream>
#include <map>
#include <set>
using namespace body_route_controller;
int main(int argc,char ** argv)
{
  try {
    std::map<std::string,std::string> args;
    const std::set<std::string> keys{"--prepare-passages","--passage-position-allowance","--passage-yaw-allowance","--passage-response-delay","--heading-policy","--heading-preview-time","--drive-style","--max-travel-angle-deg","--unknown-policy","--unknown-penalty","--observation-ttl","--mode","--speed","--yaw-rate","--linear-accel","--angular-accel","--correction-speed","--output","--common-config","--original-config"};
    for(int i=1;i<argc;++i) {
      const std::string key=argv[i];
      if(key=="--help") {
        std::cout<<"configure_trial --output NEW_DIRECTORY [--mode custom|original] [--speed 0.025] [--yaw-rate 0.1]\n"
          "  [--drive-style axis-stop|continuous] [--max-travel-angle-deg 45] (custom only)\n"
          "  [--heading-policy reactive|preview] [--heading-preview-time 5.0] (preview: custom continuous only)\n"
          "  [--prepare-passages true|false] [--passage-position-allowance .05] [--passage-yaw-allowance .08726646259971647] [--passage-response-delay .5]\n"
          "  [--linear-accel 0.05] [--angular-accel 0.2] [--correction-speed 0.6*speed]\n"
          "  [--unknown-policy blocked|observe-before-enter] [--unknown-penalty 2.0] [--observation-ttl 4.0]\n"
          "  [--common-config YAML] [--original-config YAML]\n"
          "Creates matching navigation, wheel and trial profiles; starts no ROS nodes or hardware.\n";return 0;
      }
      if(!keys.count(key)||i+1==argc||args.count(key)) {throw std::invalid_argument("unknown, duplicate or incomplete option: "+key);}
      args[key]=argv[++i];
    }
    if(!args.count("--output")||args["--output"].empty()) {throw std::invalid_argument("--output NEW_DIRECTORY is required");}
    auto number=[&](const std::string & name,double fallback) {
      if(!args.count(name)) {return fallback;}
      std::size_t end=0;const double n=std::stod(args[name],&end);
      if(end!=args[name].size()) {throw std::invalid_argument("invalid number for "+name);}
      return n;
    };
    TrialSettings s;
    if(args.count("--mode")) {s.mode=args["--mode"];}
    if(args.count("--unknown-policy"))s.unknown_policy=args["--unknown-policy"];
    s.unknown_penalty=number("--unknown-penalty",s.unknown_penalty);s.observation_ttl=number("--observation-ttl",s.observation_ttl);
    if(args.count("--drive-style"))s.drive_style=args["--drive-style"];
    if(args.count("--heading-policy"))s.heading_policy=args["--heading-policy"];
    if(args.count("--prepare-passages")) {
      const auto value=args["--prepare-passages"];
      if(value!="true"&&value!="false")throw std::invalid_argument("--prepare-passages requires true or false");
      s.prepare_passages=value=="true";
    }
    s.passage_position_allowance=number("--passage-position-allowance",s.passage_position_allowance);
    s.passage_yaw_allowance=number("--passage-yaw-allowance",s.passage_yaw_allowance);
    s.passage_response_delay=number("--passage-response-delay",s.passage_response_delay);
    s.heading_preview_time=number("--heading-preview-time",s.heading_preview_time);
    s.max_travel_angle_deg=number("--max-travel-angle-deg",s.max_travel_angle_deg);
    s.speed=number("--speed",s.speed);s.yaw_rate=number("--yaw-rate",s.yaw_rate);
    s.linear_accel=number("--linear-accel",s.linear_accel);s.angular_accel=number("--angular-accel",s.angular_accel);
    s.correction_speed=number("--correction-speed",std::min(s.speed,std::max(.004,.6*s.speed)));s.validate();
    const auto common=args.count("--common-config")?args["--common-config"]:
      ament_index_cpp::get_package_share_path("body_route_controller").string()+"/config/nav2.yaml";
    const auto original=args.count("--original-config")?args["--original-config"]:
      ament_index_cpp::get_package_share_path("bringup").string()+"/config/nav2.yaml";
    auto p=makeProfile(YAML::LoadFile(common),YAML::LoadFile(original),s);
    p.manifest["common_source"]=std::filesystem::absolute(common).string();
    p.manifest["original_source"]=std::filesystem::absolute(original).string();
    const auto out=std::filesystem::absolute(args["--output"]);
    if(!std::filesystem::create_directory(out)) {throw std::runtime_error("output must be a NEW directory; existing profiles are never overwritten");}
    writeNew((out/"nav2.yaml").string(),YAML::Dump(p.navigation)+"\n");
    writeNew((out/"drive_limits.yaml").string(),YAML::Dump(p.drive)+"\n");
    writeNew((out/"trial.yaml").string(),YAML::Dump(p.trial)+"\n");
    writeNew((out/"manifest.yaml").string(),YAML::Dump(p.manifest)+"\n");
    std::cout<<"Created "<<out<<" mode="<<s.mode<<" speed="<<s.speed<<" m/s yaw_rate="<<s.yaw_rate<<" rad/s\n"
      "Use drive_limits.yaml at wheel bringup, nav2.yaml at navigation launch, and trial.yaml with route_trial.\n"
      "Defaults remain slow; higher values require physical qualification. Restart while stopped to change profiles.\n";
  } catch(const std::exception & e) {std::cerr<<"configure_trial: "<<e.what()<<'\n';return 1;}
  return 0;
}
