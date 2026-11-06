#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>
#include <fcntl.h>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <vector>
extern char ** environ;
namespace fs=std::filesystem;
using Json=nlohmann::json;
namespace {
int run(std::vector<std::string> args) {
  std::vector<char*> argv;for(auto &a:args)argv.push_back(a.data());argv.push_back(nullptr);
  posix_spawn_file_actions_t actions;posix_spawn_file_actions_init(&actions);
  posix_spawn_file_actions_addopen(&actions,STDOUT_FILENO,"/dev/null",O_WRONLY,0);
  pid_t pid=-1;const int rc=posix_spawn(&pid,argv[0],&actions,nullptr,argv.data(),environ);
  posix_spawn_file_actions_destroy(&actions);if(rc!=0)return -1;
  int status=0;if(waitpid(pid,&status,0)!=pid || !WIFEXITED(status))return -1;
  return WEXITSTATUS(status);
}
void save(const fs::path &p,const Json &j){std::ofstream(p)<<j.dump(2);}
Json read(const fs::path &p){Json j;std::ifstream(p)>>j;return j;}
class FullMapFiles:public ::testing::Test {
protected:
  void SetUp() override {dir=fs::temp_directory_path()/("full-map-files-"+std::to_string(getpid())+"-"+std::to_string(counter++));fs::create_directory(dir);}
  void TearDown() override {fs::remove_all(dir);}
  fs::path dir;inline static int counter=0;
};
TEST_F(FullMapFiles, RestoresOnlyThreeAmclFieldsAndRefusesOverwrite) {
  const auto input=fs::path(BODY_ROUTE_SOURCE)/"config/nav2.yaml";
  auto original=YAML::LoadFile(input.string());auto legacy=YAML::Clone(original);
  legacy["amcl"]["ros__parameters"]["scan_topic"]="/localization/start_scan";
  legacy["amcl"]["ros__parameters"]["set_initial_pose"]=true;
  legacy["amcl"]["ros__parameters"]["always_reset_initial_pose"]=false;
  std::ofstream(dir/"in.yaml")<<legacy;
  ASSERT_EQ(run({PARAMS_EXECUTABLE,(dir/"in.yaml").string(),(dir/"out.yaml").string()}),0);
  auto restored=YAML::LoadFile((dir/"out.yaml").string());
  EXPECT_EQ(YAML::Dump(original),YAML::Dump(restored));
  EXPECT_NE(run({PARAMS_EXECUTABLE,(dir/"in.yaml").string(),(dir/"out.yaml").string()}),0);
}
TEST_F(FullMapFiles, RecoversPoseFromBothLidarsAndIgnoresGrayMapCells) {
  constexpr int width=200,height=180;constexpr double resolution=.01;
  std::vector<unsigned char> pixels(width*height,205);std::vector<std::array<double,2>> walls;
  for(int y=10;y<170;++y)for(int x=10;x<190;++x)if(x==10||x==189||y==10||y==169) {
    pixels[y*width+x]=0;walls.push_back({(x+.5)*resolution,(height-1-y+.5)*resolution});
  }
  std::ofstream pgm(dir/"map.pgm",std::ios::binary);pgm<<"P5\n200 180\n255\n";
  pgm.write(reinterpret_cast<char*>(pixels.data()),pixels.size());pgm.close();
  save(dir/"map.json",{{"resolution",resolution},{"origin",{0,0,0}},{"negate",0},{"occupied_thresh",.65},{"free_thresh",.196}});
  const std::array<double,3> truth{.55,.75,.1},seed{.59,.71,.1+2*std::acos(-1)/180};
  Json j={{"pose",seed},{"held",true},{"speed",0},{"wz",0},{"scans",Json::array({Json::array(),Json::array()})}};
  for(std::size_t i=0;i<walls.size();i+=3) {
    const double dx=walls[i][0]-truth[0],dy=walls[i][1]-truth[1];
    const double x=std::cos(truth[2])*dx+std::sin(truth[2])*dy,y=-std::sin(truth[2])*dx+std::cos(truth[2])*dy;
    j["scans"][i%2].push_back({seed[0]+std::cos(seed[2])*x-std::sin(seed[2])*y,seed[1]+std::sin(seed[2])*x+std::cos(seed[2])*y});
  }
  save(dir/"scan.json",j);
  const std::vector<std::string> args={REGISTER_EXECUTABLE,(dir/"map.pgm").string(),(dir/"map.json").string(),(dir/"scan.json").string(),(dir/"fit.json").string()};
  ASSERT_EQ(run(args),0);const auto result=read(dir/"fit.json");
  EXPECT_EQ(result["map_occupied_cells"].get<std::size_t>(),walls.size());
  EXPECT_DOUBLE_EQ(result["train_range_m"].get<double>(),7.5);
  for(int k=0;k<3;++k)EXPECT_NEAR(result["best"][k].get<double>(),truth[k],1e-6);
  EXPECT_EQ(result["source_snapshot"],j);EXPECT_NE(run(args),0);
  fs::remove(dir/"fit.json");j["scans"][1]=Json::array();save(dir/"scan.json",j);EXPECT_NE(run(args),0);
  j["speed"]=.01;save(dir/"scan.json",j);EXPECT_NE(run(args),0);EXPECT_FALSE(fs::exists(dir/"fit.json"));
}
} // namespace
