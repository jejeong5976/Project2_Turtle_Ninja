// File-only localization overlay. Keep all non-AMCL parameters intact.
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
int main(int argc,char **argv) {
  try {
    if(argc!=3)throw std::runtime_error("front_localization_params INPUT_NAV2 NEW_OUTPUT_NAV2");
    if(std::filesystem::exists(argv[2]))throw std::runtime_error("Output must be new; preserve historical profiles");
    auto config=YAML::LoadFile(argv[1]);auto amcl=config["amcl"]["ros__parameters"];
    if(!amcl.IsMap() || amcl["laser_model_type"].as<std::string>()!="likelihood_field")
      throw std::runtime_error("Expected existing likelihood_field AMCL profile");
    amcl["scan_topic"]="/lidar_front/scan";
    amcl["set_initial_pose"]=false;
    amcl["always_reset_initial_pose"]=true;
    std::ofstream out(argv[2]);out<<config<<'\n';out.close();
    if(!out)throw std::runtime_error("Cannot write output");
    std::cout<<"Startup fit: both filtered lidars / full existing map\n"
      "Stationary AMCL: /lidar_front/scan\nDriving AMCL: /lidar_front/scan\n"
      "AMCL waits for reviewed fresh initialization; no startup scan mask or sensor transition\n";
    return 0;
  } catch(const std::exception &e){std::cerr<<e.what()<<'\n';return 1;}
}
