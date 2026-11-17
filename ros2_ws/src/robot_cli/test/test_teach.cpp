// test_teach.cpp — mark 기록 (csv · yaml · 검출) · cycle 순서
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <unistd.h>

#include "interfaces/action/arm_task.hpp"
#include "robot_cli/bag.hpp"
#include "robot_cli/teach.hpp"

using namespace robot_cli;
using ArmTask = interfaces::action::ArmTask;

namespace
{
Config make_cfg()
{
  Config c;
  c.cal = {{"joint1", {4088, 1.0}}, {"joint2", {2048, 1.0}}, {"joint3", {2048, 1.0}}, {"joint4", {2048, 1.0}},
    {"joint5", {2048, 1.0}}, {"gripper_left_joint", {3549, 0.01149}}};
  c.arm.poses = {{"받침", {-0.0061, -1.4511, 0.9511, -0.1181, 0.0230}}};
  c.arm.fold_tol = {0.05};
  return c;
}

Snapshot make_snap()
{
  Snapshot s;
  s.now = 20.0;
  s.joints = {{"joint1", -0.0061}, {"joint2", -1.4511}, {"joint3", 0.9511}, {"joint4", -0.1181}, {"joint5", 0.0230},
    {"gripper_left_joint", -0.0016}};
  s.corrected = {{"joint2", -1.44}, {"joint3", 0.95}};
  s.corrected_t = 19.99;
  s.joints_t = 19.99;
  s.mode = 1;
  s.mode_t = 19.9;
  s.odom_t = 19.9;
  s.ox = 0.1;
  s.wrist_t = 19.8;
  s.wrist_frame = "wrist_camera_color_optical_frame";
  DetView d;
  d.kind = "tag";
  d.id = "21";
  d.x = 0.01;
  d.y = -0.02;
  d.z = 0.25;
  d.qw = 1.0;
  s.wrist_dets = {d};
  s.wrist_n = 1;
  return s;
}

std::size_t commas(const std::string & s) {return static_cast<std::size_t>(std::count(s.begin(), s.end(), ','));}

std::vector<std::string> read_lines(const std::string & path)
{
  std::ifstream f(path);
  std::vector<std::string> out;
  for (std::string l; std::getline(f, l);) {out.push_back(l);}
  return out;
}
}  // namespace

TEST(Teach, MarkRows)
{
  const Config cfg = make_cfg();
  const Snapshot s = make_snap();
  const std::string row = mark_csv_row("home_check", "2026-11-03T15:00:00.000", s, cfg);
  EXPECT_EQ(commas(row), commas(mark_csv_header()));   // 칸 수가 머리줄과 같음
  EXPECT_NE(row.find("home_check,2026-11-03T15:00:00.000,"), std::string::npos);
  EXPECT_NE(row.find(",4084,"), std::string::npos);   // joint1 눈금 (zero 4088 − 0.0061 rad)
  EXPECT_NE(row.find(",받침,"), std::string::npos);
  EXPECT_NE(row.find(",DRIVE,"), std::string::npos);
  const auto dets = det_csv_rows("home_check", "t", s);
  ASSERT_EQ(dets.size(), 1u);
  EXPECT_EQ(commas(dets[0]), commas(det_csv_header()));
  EXPECT_NE(dets[0].find(",21,tag,"), std::string::npos);
  // yaml 은 읽히고 값이 맞음
  const YAML::Node y = YAML::Load(mark_yaml_entry("home_check", "t", s, cfg));
  ASSERT_TRUE(y.IsSequence());
  EXPECT_EQ(y[0]["name"].as<std::string>(), "home_check");
  EXPECT_EQ(y[0]["joints"]["joint1"]["tick"].as<int>(), 4084);
  EXPECT_NEAR(y[0]["joints"]["joint2"]["corrected_rad"].as<double>(), -1.44, 1e-9);
  EXPECT_TRUE(y[0]["joints"]["joint1"]["corrected_rad"].IsNull());   // 보정 값 없음 → null
  EXPECT_TRUE(y[0]["pose"]["folded"].as<bool>());
  EXPECT_EQ(y[0]["wrist"]["detections"][0]["id"].as<std::string>(), "21");
  EXPECT_NEAR(y[0]["wrist"]["detections"][0]["position"][2].as<double>(), 0.25, 1e-9);
  EXPECT_TRUE(y[0]["map"].IsNull());
  // 처짐 보정 값이 끊긴 지 오래면 새 엔코더 값과 짝짓지 않음 (null)
  Snapshot old = s;
  old.corrected_t = 10.0;
  const YAML::Node yo = YAML::Load(mark_yaml_entry("o", "t", old, cfg));
  EXPECT_TRUE(yo[0]["joints"]["joint2"]["corrected_rad"].IsNull());
  EXPECT_NEAR(yo[0]["corrected_age_s"].as<double>(), 10.0, 1e-6);
  EXPECT_EQ(commas(mark_csv_row("o", "t", old, cfg)), commas(mark_csv_header()));

  // 인지가 준 글에 , " \ 줄바꿈이 있어도 칸 수 · yaml 이 깨지지 않음
  Snapshot odd = s;
  odd.wrist_dets[0].label = "re,d\"x\\\n";
  odd.wrist_frame = "cam,frame";
  const auto rows = det_csv_rows("n", "t", odd);
  ASSERT_EQ(rows.size(), 1u);
  EXPECT_EQ(commas(rows[0]), commas(det_csv_header()));
  EXPECT_EQ(commas(mark_csv_row("n", "t", odd, cfg)), commas(mark_csv_header()));
  const YAML::Node y2 = YAML::Load(mark_yaml_entry("n", "t", odd, cfg));
  EXPECT_EQ(y2[0]["wrist"]["detections"][0]["label"].as<std::string>(), "re;dx ");
}

TEST(Teach, WriteMark)
{
  const auto dir = std::filesystem::temp_directory_path() / ("robot_cli_test_" + std::to_string(::getpid()));
  std::filesystem::remove_all(dir);
  const Config cfg = make_cfg();
  Snapshot s = make_snap();
  std::string text;
  ASSERT_TRUE(write_mark(dir.string(), "a1", s, cfg, text)) << text;
  EXPECT_NE(text.find("a1"), std::string::npos);
  ASSERT_TRUE(write_mark(dir.string(), "a2", s, cfg, text)) << text;
  const auto csv = read_lines((dir / "poses.csv").string());
  ASSERT_EQ(csv.size(), 3u);   // 머리줄 하나 + 두 줄
  EXPECT_EQ(csv[0], mark_csv_header());
  EXPECT_EQ(read_lines((dir / "detections.csv").string()).size(), 3u);
  const YAML::Node y = YAML::LoadFile((dir / "poses.yaml").string());
  ASSERT_EQ(y.size(), 2u);
  EXPECT_EQ(y[1]["name"].as<std::string>(), "a2");
  s.joints_t = 10.0;   // 10 s 전 측정 → 기록 안 함
  EXPECT_FALSE(write_mark(dir.string(), "old", s, cfg, text));
  EXPECT_EQ(read_lines((dir / "poses.csv").string()).size(), 3u);
  s.joints_t = -1.0;
  EXPECT_FALSE(write_mark(dir.string(), "none", s, cfg, text));
  std::filesystem::remove_all(dir);
}

TEST(Teach, Cycle)
{
  CycleRun c;
  std::string err;
  Command cmd;
  EXPECT_FALSE(c.start("S5", 0, false, err));
  ASSERT_TRUE(c.start("S5", 2, false, err));
  EXPECT_FALSE(c.start("S5", 2, false, err));   // 이미 반복 중
  std::vector<uint8_t> sent;
  double t = 0.0;
  std::string row;
  while (c.next(cmd, t, "w")) {
    EXPECT_FALSE(c.next(cmd, t, "w"));   // 결과 전에는 다음을 안 보냄
    sent.push_back(cmd.task);
    EXPECT_EQ(cmd.slot, "S5");
    t += 10.0;
    EXPECT_FALSE(c.on_done(0, "ok", t, "grasp 1.0", &row).empty());
  }
  EXPECT_EQ(sent, (std::vector<uint8_t>{ArmTask::Goal::RETRIEVE, ArmTask::Goal::STORE, ArmTask::Goal::RETRIEVE,
    ArmTask::Goal::STORE}));
  EXPECT_FALSE(c.active());
  EXPECT_NE(c.status(t).find("4/4"), std::string::npos);
  EXPECT_NE(c.status(t).find("평균 10.0"), std::string::npos);
  EXPECT_EQ(commas(row), commas(CycleRun::csv_header()));

  // 실패하면 바로 멈춤 · STORE 먼저
  ASSERT_TRUE(c.start("S1", 3, true, err));
  ASSERT_TRUE(c.next(cmd, 0.0, "w"));
  EXPECT_EQ(cmd.task, ArmTask::Goal::STORE);
  c.on_done(2, "거절, 이유", 1.0, "phase 없음", &row);
  EXPECT_FALSE(c.active());
  EXPECT_NE(row.find("fail"), std::string::npos);
  EXPECT_EQ(row.find("거절, 이유"), std::string::npos);   // csv 칸을 깨는 쉼표는 바뀜

  // cycle stop: 지금 작업이 끝나면 멈춤
  ASSERT_TRUE(c.start("S2", 5, false, err));
  ASSERT_TRUE(c.next(cmd, 0.0, "w"));
  c.request_stop();
  EXPECT_TRUE(c.active());
  c.on_done(0, "ok", 5.0, "", nullptr);
  EXPECT_FALSE(c.active());
  EXPECT_FALSE(c.next(cmd, 6.0, "w"));
}

TEST(Teach, NextFree)
{
  const auto dir = std::filesystem::temp_directory_path() / ("robot_cli_nf_" + std::to_string(::getpid()));
  std::filesystem::create_directories(dir);
  EXPECT_EQ(next_free(dir.string(), "session", ".log"), (dir / "session_001").string());
  std::ofstream((dir / "session_001.log").string()) << "x";
  EXPECT_EQ(next_free(dir.string(), "session", ".log"), (dir / "session_002").string());
  EXPECT_EQ(csv_clean("a,b\"c\nd"), "a;bc d");
  // reserve_free: 이름을 차지하므로 연달아 불러도 다른 파일
  const std::string r1 = reserve_free(dir.string(), "session", ".log");
  const std::string r2 = reserve_free(dir.string(), "session", ".log");
  EXPECT_EQ(r1, (dir / "session_002.log").string());
  EXPECT_EQ(r2, (dir / "session_003.log").string());
  std::filesystem::remove_all(dir);
}
