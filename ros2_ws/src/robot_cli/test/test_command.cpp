// test_command.cpp — 명령 문법
#include <gtest/gtest.h>

#include "interfaces/action/arm_task.hpp"
#include "robot_cli/command.hpp"

using namespace robot_cli;
using ArmTask = interfaces::action::ArmTask;

namespace
{
const Names kNames{{"start", "barcode", "B1", "B2", "L1"}, {"S1", "S5", "S12"}, 0.20, 0.5};

bool parse(const std::string & line, Command & c, std::string * err_out = nullptr)
{
  std::string err;
  const bool ok = parse_command(split_words(line), kNames, c, err);
  if (err_out) {*err_out = err;}
  return ok;
}
}  // namespace

TEST(Command, Arm)
{
  Command c;
  ASSERT_TRUE(parse("home", c));
  EXPECT_EQ(c.cmd, Cmd::ARM);
  EXPECT_EQ(c.task, ArmTask::Goal::HOME);
  EXPECT_EQ(c.slot, "");
  ASSERT_TRUE(parse("HOME force", c));
  EXPECT_EQ(c.slot, "force");
  ASSERT_TRUE(parse("arm STORE s5", c));
  EXPECT_EQ(c.task, ArmTask::Goal::STORE);
  EXPECT_EQ(c.slot, "S5");
  ASSERT_TRUE(parse("retrieve S12", c));
  EXPECT_EQ(c.task, ArmTask::Goal::RETRIEVE);
  EXPECT_FALSE(parse("store S13", c));
  EXPECT_FALSE(parse("store", c));
  ASSERT_TRUE(parse("look", c));
  EXPECT_EQ(c.task, ArmTask::Goal::LOOK);
  ASSERT_TRUE(parse("barcode", c));
  EXPECT_EQ(c.task, ArmTask::Goal::LOOK_BARCODE);
  ASSERT_TRUE(parse("arm LOOK_BARCODE", c));
  EXPECT_EQ(c.task, ArmTask::Goal::LOOK_BARCODE);
  EXPECT_FALSE(parse("look 1", c));
  ASSERT_TRUE(parse("pick det", c));   // 손목 인지 검출로 집기
  EXPECT_EQ(c.task, ArmTask::Goal::PICK);
  EXPECT_TRUE(c.from_det);
  EXPECT_EQ(c.det_tag, "");
  EXPECT_FALSE(parse("p det 2", c));   // 번호로 고르기 없음 (검출 순서가 바뀔 수 있음)
  ASSERT_TRUE(parse("pick tag 3", c));
  EXPECT_EQ(c.det_tag, "3");
  EXPECT_FALSE(parse("pick det x", c));
  EXPECT_FALSE(parse("pick tag", c));
  ASSERT_TRUE(parse("pick 0.21 0.04", c));
  EXPECT_FALSE(c.from_det);
  ASSERT_TRUE(parse("look rack", c));   // 빈 손 적재함 스캔
  EXPECT_EQ(c.task, ArmTask::Goal::LOOK);
  EXPECT_EQ(c.slot, "rack");
  ASSERT_TRUE(parse("scan", c));
  EXPECT_EQ(c.task, ArmTask::Goal::LOOK);
  EXPECT_EQ(c.slot, "rack");
  ASSERT_TRUE(parse("look", c));
  EXPECT_EQ(c.slot, "");
  EXPECT_FALSE(parse("arm JUMP", c));
}

TEST(Command, PickPlace)
{
  Command c;
  ASSERT_TRUE(parse("pick 0.25 -0.02", c));
  EXPECT_EQ(c.task, ArmTask::Goal::PICK);
  EXPECT_DOUBLE_EQ(c.x, 0.25);
  EXPECT_DOUBLE_EQ(c.y, -0.02);
  EXPECT_DOUBLE_EQ(c.z, 0.0);
  EXPECT_TRUE(std::isnan(c.yaw_deg));
  ASSERT_TRUE(parse("place 0.21 0.04 0.08 30", c));
  EXPECT_EQ(c.task, ArmTask::Goal::PLACE);
  EXPECT_DOUBLE_EQ(c.z, 0.08);
  EXPECT_DOUBLE_EQ(c.yaw_deg, 30.0);
  EXPECT_FALSE(c.from_last);
  EXPECT_FALSE(parse("pick 0.25", c));
  EXPECT_FALSE(parse("pick a b", c));
  EXPECT_FALSE(parse("pick 1 2 3 4 5", c));
  EXPECT_FALSE(parse("pick nan 0", c));
  // 인자 없는 place = 마지막 성공 PICK 자리 (목표는 보낼 때 LastPick)
  for (const char * line : {"place", "pl", "PL", "arm PLACE"}) {
    ASSERT_TRUE(parse(line, c)) << line;
    EXPECT_EQ(c.task, ArmTask::Goal::PLACE) << line;
    EXPECT_TRUE(c.from_last) << line;
  }
  ASSERT_TRUE(parse("pl 0.2 0.04", c));
  EXPECT_FALSE(c.from_last);
  std::string err;
  EXPECT_FALSE(parse("pl 0.2", c, &err));
  EXPECT_NE(err.find("마지막 PICK"), std::string::npos);
  EXPECT_FALSE(parse("pick", c));   // pick 은 좌표 · det · tag 가 꼭 있음
}

TEST(LastPick, PlaceAtLastPick)
{
  LastPick lp;
  ArmTarget t;
  std::string why;
  EXPECT_FALSE(lp.place_target(t, why));   // 켜진 뒤 PICK 없음
  EXPECT_NE(why.find("PICK 없음"), std::string::npos);

  const ArmTarget a{0.235, -0.010, 0.052, 0.0, 0.0, 0.3826834, 0.9238795, 45.0};   // pick det 이 보낸 목표 (방향까지)
  lp.on_result(ArmTask::Goal::PICK, a, false);   // 실패한 PICK 은 안 셈
  EXPECT_FALSE(lp.place_target(t, why));
  lp.on_result(ArmTask::Goal::PICK, a, true);
  ASSERT_TRUE(lp.place_target(t, why));
  EXPECT_DOUBLE_EQ(t.x, a.x);
  EXPECT_DOUBLE_EQ(t.y, a.y);
  EXPECT_DOUBLE_EQ(t.z, a.z);
  EXPECT_DOUBLE_EQ(t.qz, a.qz);
  EXPECT_DOUBLE_EQ(t.qw, a.qw);
  EXPECT_DOUBLE_EQ(t.yaw_deg, 45.0);

  // 다른 작업 · 실패한 PLACE · STORE 는 그대로 (아직 쥐고 있음)
  lp.on_result(ArmTask::Goal::LOOK, ArmTarget{}, true);
  lp.on_result(ArmTask::Goal::PLACE, ArmTarget{}, false);
  lp.on_result(ArmTask::Goal::STORE, ArmTarget{}, false);
  ASSERT_TRUE(lp.place_target(t, why));
  EXPECT_DOUBLE_EQ(t.x, a.x);

  lp.on_result(ArmTask::Goal::PLACE, a, true);   // 놓음 → 다시 놓을 퍽 없음
  EXPECT_FALSE(lp.place_target(t, why));
  EXPECT_NE(why.find("PLACE"), std::string::npos);
  EXPECT_NE(why.find("0.235"), std::string::npos);

  // 새 PICK (좌표, 방향 모름 0,0,0,0) → 다시 됨, 그 뒤 STORE 성공 → 거절
  const ArmTarget b{0.200, 0.050, 0.0, 0.0, 0.0, 0.0, 0.0, NAN};
  lp.on_result(ArmTask::Goal::PICK, b, true);
  ASSERT_TRUE(lp.place_target(t, why));
  EXPECT_DOUBLE_EQ(t.qw, 0.0);
  EXPECT_TRUE(std::isnan(t.yaw_deg));
  lp.on_result(ArmTask::Goal::STORE, ArmTarget{}, true);
  EXPECT_FALSE(lp.place_target(t, why));
  EXPECT_NE(why.find("STORE"), std::string::npos);
  EXPECT_NE(why.find("0.200"), std::string::npos);
}

TEST(Command, Nav)
{
  Command c;
  ASSERT_TRUE(parse("nav b1", c));
  EXPECT_EQ(c.cmd, Cmd::NAV);
  EXPECT_EQ(c.stop, "B1");
  ASSERT_TRUE(parse("goto 1.0 -0.5 90", c));
  EXPECT_EQ(c.stop, "");
  EXPECT_DOUBLE_EQ(c.x, 1.0);
  EXPECT_DOUBLE_EQ(c.yaw_nav_deg, 90.0);
  ASSERT_TRUE(parse("nav 1 2", c));
  EXPECT_DOUBLE_EQ(c.yaw_nav_deg, 0.0);
  std::string err;
  EXPECT_FALSE(parse("nav X9", c, &err));
  EXPECT_NE(err.find("B1"), std::string::npos);   // 아는 이름을 알려줌
  EXPECT_FALSE(parse("nav", c));
}

TEST(Command, Drive)
{
  Command c;
  ASSERT_TRUE(parse("drive 0.05 0 0.2 2", c));
  EXPECT_EQ(c.cmd, Cmd::DRIVE);
  EXPECT_DOUBLE_EQ(c.vx, 0.05);
  EXPECT_DOUBLE_EQ(c.wz, 0.2);
  EXPECT_DOUBLE_EQ(c.secs, 2.0);
  EXPECT_FALSE(c.force);
  ASSERT_TRUE(parse("drive 0.14 0.14 0 force", c));   // hypot 0.198 (한계 0.20 = controllers.yaml max_linear)
  EXPECT_TRUE(c.force);
  EXPECT_DOUBLE_EQ(c.secs, 1.0);
  EXPECT_FALSE(parse("drive 0.15 0.15 0", c));       // hypot 0.212 > 0.20
  EXPECT_FALSE(parse("drive 0 0 0.6", c));
  EXPECT_FALSE(parse("drive 0.05 0 0 11", c));
  EXPECT_FALSE(parse("drive 0.05 0 0 0", c));
  EXPECT_FALSE(parse("drive 0.05 0", c));
  ASSERT_TRUE(parse("stop", c));
  EXPECT_EQ(c.cmd, Cmd::STOP);
  ASSERT_TRUE(parse("teleop force", c));
  EXPECT_EQ(c.cmd, Cmd::TELEOP);
  EXPECT_TRUE(c.force);
  EXPECT_FALSE(parse("teleop now", c));
}

TEST(Command, StopsAndServices)
{
  Command c;
  ASSERT_TRUE(parse("cancel", c));
  EXPECT_EQ(c.which, "all");
  ASSERT_TRUE(parse("cancel ARM", c));
  EXPECT_EQ(c.which, "arm");
  EXPECT_FALSE(parse("cancel foo", c));
  ASSERT_TRUE(parse("estop", c));
  EXPECT_TRUE(c.on);
  ASSERT_TRUE(parse("estop off", c));
  EXPECT_FALSE(c.on);
  EXPECT_FALSE(parse("estop maybe", c));
  ASSERT_TRUE(parse("call mission/start", c));
  EXPECT_EQ(c.service, "/mission/start");
  EXPECT_EQ(c.setbool, -1);
  ASSERT_TRUE(parse("call /perception/barcode_enable on", c));
  EXPECT_EQ(c.setbool, 1);
  ASSERT_TRUE(parse("call /x false", c));
  EXPECT_EQ(c.setbool, 0);
  EXPECT_FALSE(parse("call /x maybe", c));
  ASSERT_TRUE(parse("mission reset_signal", c));
  EXPECT_EQ(c.cmd, Cmd::CALL);
  EXPECT_EQ(c.service, "/mission/reset_signal");
  EXPECT_FALSE(parse("mission go", c));
  EXPECT_FALSE(parse("fly", c));
  EXPECT_FALSE(parse("", c));
  ASSERT_TRUE(parse("help", c));
  EXPECT_EQ(c.cmd, Cmd::HELP);
}

TEST(Command, PagesAndExperiment)
{
  Command c;
  ASSERT_TRUE(parse("2", c));
  EXPECT_EQ(c.cmd, Cmd::PAGE);
  EXPECT_EQ(c.page, 2);
  EXPECT_FALSE(parse("8", c));
  EXPECT_FALSE(parse("2 3", c));
  ASSERT_TRUE(parse("status 7", c));
  EXPECT_EQ(c.cmd, Cmd::STATUS);
  EXPECT_EQ(c.page, 7);
  ASSERT_TRUE(parse("status", c));
  EXPECT_EQ(c.page, 0);
  EXPECT_FALSE(parse("status 9", c));
  ASSERT_TRUE(parse("mark S5_tag21", c));
  EXPECT_EQ(c.cmd, Cmd::MARK);
  EXPECT_EQ(c.name, "S5_tag21");
  ASSERT_TRUE(parse("mark 받침위", c));
  EXPECT_FALSE(parse("mark", c));
  EXPECT_FALSE(parse("mark a,b", c));
  EXPECT_FALSE(parse("mark a:b", c));
  EXPECT_FALSE(parse("mark a b", c));
  ASSERT_TRUE(parse("markdir /tmp/x", c));
  EXPECT_EQ(c.cmd, Cmd::MARKDIR);
  EXPECT_EQ(c.name, "/tmp/x");
  ASSERT_TRUE(parse("cycle s5 3", c));
  EXPECT_EQ(c.cmd, Cmd::CYCLE);
  EXPECT_EQ(c.slot, "S5");
  EXPECT_EQ(c.count, 3);
  EXPECT_FALSE(c.store_first);
  ASSERT_TRUE(parse("cycle S12 1 store", c));
  EXPECT_TRUE(c.store_first);
  ASSERT_TRUE(parse("cycle stop", c));
  EXPECT_EQ(c.count, 0);
  EXPECT_FALSE(parse("cycle S5 0", c));
  EXPECT_FALSE(parse("cycle S5 2.5", c));
  EXPECT_FALSE(parse("cycle S13 2", c));
  EXPECT_FALSE(parse("cycle S5 2 fast", c));
  ASSERT_TRUE(parse("teach", c));
  EXPECT_EQ(c.cmd, Cmd::TEACH);
}

TEST(Command, ShortAliases)
{
  Command c;
  ASSERT_TRUE(parse("h", c));
  EXPECT_EQ(c.task, ArmTask::Goal::HOME);
  ASSERT_TRUE(parse("l", c));
  EXPECT_EQ(c.task, ArmTask::Goal::LOOK);
  ASSERT_TRUE(parse("b", c));
  EXPECT_EQ(c.task, ArmTask::Goal::LOOK_BARCODE);
  ASSERT_TRUE(parse("s5", c));
  EXPECT_EQ(c.task, ArmTask::Goal::STORE);
  EXPECT_EQ(c.slot, "S5");
  ASSERT_TRUE(parse("r12", c));
  EXPECT_EQ(c.task, ArmTask::Goal::RETRIEVE);
  EXPECT_EQ(c.slot, "S12");
  ASSERT_TRUE(parse("store 5", c));   // S 생략
  EXPECT_EQ(c.slot, "S5");
  ASSERT_TRUE(parse("s S1", c));
  EXPECT_EQ(c.slot, "S1");
  EXPECT_FALSE(parse("s7", c));       // 없는 칸
  ASSERT_TRUE(parse("p 0.25 0", c));
  EXPECT_EQ(c.task, ArmTask::Goal::PICK);
  ASSERT_TRUE(parse("pl 0.2 0.04", c));
  EXPECT_EQ(c.task, ArmTask::Goal::PLACE);
  ASSERT_TRUE(parse("g b1", c));
  EXPECT_EQ(c.cmd, Cmd::GO);
  EXPECT_EQ(c.stop, "B1");
  ASSERT_TRUE(parse("x", c));
  EXPECT_EQ(c.cmd, Cmd::CANCEL);
  EXPECT_EQ(c.which, "all");
  ASSERT_TRUE(parse("!", c));
  EXPECT_EQ(c.cmd, Cmd::ESTOP);
  EXPECT_TRUE(c.on);
  ASSERT_TRUE(parse("c", c));
  EXPECT_EQ(c.cmd, Cmd::CHECK);
  ASSERT_TRUE(parse("m tag21", c));
  EXPECT_EQ(c.cmd, Cmd::MARK);
  ASSERT_TRUE(parse("n J4 2485 에서 멈춤", c));
  EXPECT_EQ(c.cmd, Cmd::NOTE);
  EXPECT_EQ(c.name, "J4 2485 에서 멈춤");
  ASSERT_TRUE(parse("q", c));
  EXPECT_EQ(c.cmd, Cmd::QUIT);
  ASSERT_TRUE(parse("?", c));
  EXPECT_EQ(c.cmd, Cmd::HELP);
  ASSERT_TRUE(parse("bag start", c));
  EXPECT_TRUE(c.on);
  EXPECT_FALSE(parse("bag", c));
  ASSERT_TRUE(parse("stop", c));      // 별칭이 긴 명령을 가리지 않음
  EXPECT_EQ(c.cmd, Cmd::STOP);
  ASSERT_TRUE(parse("stops", c));
  EXPECT_EQ(c.cmd, Cmd::STOPS);
  ASSERT_TRUE(parse("cycle 5 2", c));
  EXPECT_EQ(c.slot, "S5");
}

TEST(Command, Complete)
{
  EXPECT_EQ(complete_candidates("ho", kNames), (std::vector<std::string>{"home"}));
  EXPECT_EQ(complete_candidates("g B", kNames), (std::vector<std::string>{"barcode", "B1", "B2"}));   // 대소문자 무시
  EXPECT_EQ(complete_candidates("g B1", kNames), (std::vector<std::string>{"B1"}));
  EXPECT_EQ(complete_candidates("store s1", kNames), (std::vector<std::string>{"S1", "S12"}));
  EXPECT_EQ(complete_candidates("mission st", kNames), (std::vector<std::string>{"start"}));
  EXPECT_TRUE(complete_candidates("drive 0", kNames).empty());
  EXPECT_GT(complete_candidates("", kNames).size(), 10u);
}
