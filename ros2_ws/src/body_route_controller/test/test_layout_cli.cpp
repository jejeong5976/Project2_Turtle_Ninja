#include "layout_fixture_io.hpp"
#include <gtest/gtest.h>
#include <sys/wait.h>
#include <unistd.h>
#include <fstream>
namespace fs=std::filesystem;
namespace {
int run(const std::vector<std::string> & args) {
  const pid_t child=fork();if(child<0) {throw std::runtime_error("fork failed");}
  if(child==0) {
    std::vector<char*> argv;for(const auto & s:args) {argv.push_back(const_cast<char*>(s.c_str()));}argv.push_back(nullptr);
    execv(argv[0],argv.data());_exit(127);
  }
  int status=0;waitpid(child,&status,0);return WIFEXITED(status)?WEXITSTATUS(status):128;
}
}
TEST(LayoutCli, ProducesCheckedMapAndPathWithoutOverwrite) {
  char pattern[]="/tmp/layout-cli-XXXXXX";const char *name=mkdtemp(pattern);ASSERT_NE(name,nullptr);const fs::path root=name;
  layout_test::fixture(root/"input");
  const std::vector<std::string> args{LAYOUT_EXECUTABLE,"--input",(root/"input/survey.yaml").string(),
    "--params",std::string(BODY_ROUTE_SOURCE)+"/config/nav2.yaml","--output",(root/"output").string()};
  ASSERT_EQ(run(args),0);const auto report=YAML::LoadFile(root/"output/report.yaml");
  EXPECT_TRUE(report["accepted"].as<bool>());EXPECT_TRUE(report["portals"][0]["accepted"].as<bool>());
  EXPECT_GT(report["cleared_cells"].as<int>(),0);EXPECT_TRUE(fs::exists(root/"output/map.pgm"));
  const auto path=readText((root/"output/portal_0.path").string());EXPECT_GT(path.size(),100u);
  EXPECT_EQ(run(args),2);EXPECT_EQ(path,readText((root/"output/portal_0.path").string()));
  auto bad=YAML::LoadFile(root/"input/survey.yaml");bad["frames"][0]["measured_velocity"][0]=.1;
  writeNew((root/"bad.yaml").string(),YAML::Dump(bad));auto rejected=args;rejected[2]=(root/"bad.yaml").string();rejected.back()=(root/"bad-output").string();
  EXPECT_EQ(run(rejected),2);EXPECT_FALSE(fs::exists(root/"bad-output"));
}
