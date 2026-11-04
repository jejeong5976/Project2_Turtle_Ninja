// test_model.cpp — 눈금 변환 · 레포 설정 파일 읽기 (hardware_map · dice_map · rack_cells)
#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>

#include "robot_cli/model.hpp"

using namespace robot_cli;

TEST(Model, TickJoint)
{
  const JointCal j1{4088, 1.0};
  EXPECT_DOUBLE_EQ(joint_to_tick(j1, 0.0), 4088.0);
  EXPECT_NEAR(joint_to_tick(j1, M_PI / 2.0), 4088.0 + 1024.0, 1e-9);
  EXPECT_NEAR(tick_to_joint(j1, joint_to_tick(j1, -1.2345)), -1.2345, 1e-12);
  // 그리퍼 (scale 0.01149 m/rad, zero 3549): control.yaml gripper_open 0.0164 ≈ 닫힘 3478 + 1000 틱, close −0.0016 ≈ 닫힘 − 20 틱
  const JointCal g{3549, 0.01149};
  EXPECT_NEAR(joint_to_tick(g, 0.0164), 4478.0, 3.0);
  EXPECT_NEAR(joint_to_tick(g, -0.0016), 3458.0, 3.0);
  EXPECT_NEAR(gripper_gap(0.0164) * 1000.0, 51.0, 1e-9);
}

TEST(Model, HardwareMap)
{
  std::map<std::string, JointCal> cal;
  std::string err;
  ASSERT_TRUE(load_hardware_map(TEST_CONFIG_DIR "/hardware_map.yaml", cal, err)) << err;
  EXPECT_EQ(cal.at("joint1").zero_tick, 4088);
  EXPECT_EQ(cal.at("joint2").zero_tick, 2048);
  EXPECT_DOUBLE_EQ(cal.at("gripper_left_joint").scale, 0.01149);
  EXPECT_DOUBLE_EQ(cal.at("steer_fl_joint").scale, -1.0);
  EXPECT_FALSE(load_hardware_map("/nonexistent.yaml", cal, err));
  EXPECT_FALSE(err.empty());
}

TEST(Model, Stops)
{
  std::vector<NamedStop> stops;
  std::string err;
  ASSERT_TRUE(load_stops(TEST_MAPS_DIR "/dice_map.yaml", stops, err)) << err;
  ASSERT_EQ(stops.size(), 11u);   // start · barcode · B1~B6 · L1~L3
  const NamedStop * s = find_stop(stops, "START");
  ASSERT_NE(s, nullptr);
  // map_from_arena: start → 출발 자세가 map 원점
  EXPECT_NEAR(s->pose.x, 0.0, 1e-9);
  EXPECT_NEAR(s->pose.y, 0.0, 1e-9);
  EXPECT_NEAR(s->pose.yaw, 0.0, 1e-9);
  // B1 stop 경기장 (1.468, 0.6565, +90°) → 출발 (0.734, 0.950, 180°) 기준: (−0.734, +0.2935, −90°)
  const NamedStop * b1 = find_stop(stops, "b1");
  ASSERT_NE(b1, nullptr);
  EXPECT_NEAR(b1->pose.x, -0.734, 1e-4);   // start yaw 3.1416 ≠ π 정확히
  EXPECT_NEAR(b1->pose.y, 0.2935, 1e-4);
  EXPECT_NEAR(b1->pose.yaw, -M_PI / 2.0, 1e-4);
  EXPECT_EQ(find_stop(stops, "B9"), nullptr);
  double d = 0.0;
  const NamedStop * n = nearest_stop(stops, -0.70, 0.30, &d);
  ASSERT_NE(n, nullptr);
  EXPECT_EQ(n->name, "B1");
  EXPECT_LT(d, 0.05);
}

TEST(Model, Slots)
{
  std::vector<std::string> slots;
  std::string err;
  ASSERT_TRUE(load_slots(TEST_CONFIG_DIR "/rack_cells.yaml", slots, err)) << err;
  EXPECT_EQ(slots.size(), 12u);
  EXPECT_EQ(slots.front(), "S1");
  EXPECT_EQ(slots.back(), "S12");
}

TEST(Model, ArmRef)
{
  ArmRef ref;
  std::string err;
  ASSERT_TRUE(load_arm_ref(TEST_CONFIG_DIR "/control.yaml", TEST_CONFIG_DIR "/arm_paths.yaml", ref, err)) << err;
  std::vector<std::string> names;
  for (const auto & p : ref.poses) {names.push_back(p.name);}
  EXPECT_EQ(names, (std::vector<std::string>{"받침", "P0", "기준점", "LOOK", "LOOK_BARCODE"}));
  ASSERT_EQ(ref.fold_tol.size(), 1u);
  EXPECT_DOUBLE_EQ(ref.fold_tol[0], 0.05);
  // 받침 = poses.home, P0 = fold 경로 첫 점 (받침 바로 위)
  std::map<std::string, double> j;
  for (std::size_t i = 0; i < 5; ++i) {j[kArmJoint5[i]] = ref.poses[0].q[i];}
  double d = 1.0;
  const NamedPose * n = nearest_pose(ref, j, &d);
  ASSERT_NE(n, nullptr);
  EXPECT_EQ(n->name, "받침");
  EXPECT_NEAR(d, 0.0, 1e-12);
  EXPECT_TRUE(folded(ref, j));
  for (std::size_t i = 0; i < 5; ++i) {j[kArmJoint5[i]] = ref.poses[1].q[i];}
  EXPECT_EQ(nearest_pose(ref, j, &d)->name, "P0");
  std::string worst;
  EXPECT_FALSE(folded(ref, j, &worst));
  EXPECT_FALSE(worst.empty());
  j.erase("joint3");
  EXPECT_EQ(nearest_pose(ref, j, &d), nullptr);
  EXPECT_FALSE(load_arm_ref("/nonexistent.yaml", TEST_CONFIG_DIR "/arm_paths.yaml", ref, err));
}

// OpenCR 하드웨어 모터 온도 /diagnostics 한 줄 (hardware test_opencr Temp.CheckAndReport 와 같은 모양)
TEST(Model, TempStatus)
{
  std::string board;
  std::map<std::string, int> t;
  int warn = 60, crit = 70;
  ASSERT_TRUE(parse_temp_status("bus_arm/motor_temperature",
    {{"joint1", "41"}, {"joint2", "63"}, {"gripper_left_joint", "-"}, {"warn_c", "55"}, {"crit_c", "65"}}, board, t, warn, crit));
  EXPECT_EQ(board, "bus_arm");
  EXPECT_EQ(t.size(), 3u);
  EXPECT_EQ(t["joint1"], 41);
  EXPECT_EQ(t["joint2"], 63);
  EXPECT_EQ(t["gripper_left_joint"], -1);
  EXPECT_EQ(warn, 55);
  EXPECT_EQ(crit, 65);
  // 기준이 없거나 글이 이상하면 기준은 그대로 · 온도는 모름
  ASSERT_TRUE(parse_temp_status("bus_drive/motor_temperature", {{"drive_fl_joint", "4x"}, {"warn_c", "?"}}, board, t, warn, crit));
  EXPECT_EQ(board, "bus_drive");
  EXPECT_EQ(t["drive_fl_joint"], -1);
  EXPECT_EQ(warn, 55);
  EXPECT_FALSE(parse_temp_status("localization", {{"joint1", "41"}}, board, t, warn, crit));
  EXPECT_FALSE(parse_temp_status("/motor_temperature", {}, board, t, warn, crit));
}

TEST(Model, BatteryLogAndLimits)
{
  std::string b;
  double mean = 0, min = 0;
  ASSERT_TRUE(parse_battery_log("bus_arm 보드 안 (10 s): 한 바퀴 980/1200 us (1000회, 실패 0) · 주기 다시 잡기 0 · 배터리 12.3 V (최저 12.1)",
    b, mean, min));
  EXPECT_EQ(b, "bus_arm");
  EXPECT_DOUBLE_EQ(mean, 12.3);
  EXPECT_DOUBLE_EQ(min, 12.1);
  // 온도가 뒤에 붙은 10 s 줄 (온도 펌웨어부터)
  ASSERT_TRUE(parse_battery_log("bus_arm 보드 안 (10 s): 한 바퀴 980/1200 us (1000회, 실패 0) · 배터리 12.0 V (최저 11.8) · "
    "온도 °C joint1 38, joint2 70, joint5 -", b, mean, min));
  EXPECT_DOUBLE_EQ(mean, 12.0);
  EXPECT_DOUBLE_EQ(min, 11.8);
  ASSERT_TRUE(parse_battery_log("bus_drive 배터리 낮음: 평균 10.7 V · 최저 10.5 V (경고 10.8 / 정지 10.2 V)", b, mean, min));
  EXPECT_EQ(b, "bus_drive");
  EXPECT_DOUBLE_EQ(min, 10.5);
  ASSERT_TRUE(parse_battery_log("bus_arm 배터리 10.1 V (정지 기준 10.2 V) → 전체 멈춤", b, mean, min));
  EXPECT_DOUBLE_EQ(mean, 10.1);
  EXPECT_FALSE(parse_battery_log("controller_manager 시작", b, mean, min));
  double warn = 0, stop = 0;
  load_battery_limits(TEST_CONFIG_DIR "/hardware_map.yaml", warn, stop);
  EXPECT_DOUBLE_EQ(warn, 10.8);
  EXPECT_DOUBLE_EQ(stop, 10.2);
  double lin = 0, ang = 0;
  ASSERT_TRUE(load_drive_limits(TEST_CONFIG_DIR "/controllers.yaml", lin, ang));
  EXPECT_GT(lin, 0.0);
  EXPECT_GT(ang, 0.0);
  // 잘못된 값 (음수 · NaN) 이면 둘 다 그대로
  const std::string bad = std::string(::testing::TempDir()) + "robot_cli_bad_controllers.yaml";
  std::ofstream(bad) << "swerve_controller:\n  ros__parameters:\n    max_linear: .nan\n    max_angular: -0.5\n";
  double l2 = 0.2, a2 = 0.5;
  EXPECT_FALSE(load_drive_limits(bad, l2, a2));
  EXPECT_DOUBLE_EQ(l2, 0.2);
  EXPECT_DOUBLE_EQ(a2, 0.5);
  std::remove(bad.c_str());
  ArmRef r;
  r.grip_open = 0.0164;
  r.grip_close = -0.0016;
  EXPECT_TRUE(gripper_holding(r, 0.006));
  EXPECT_FALSE(gripper_holding(r, 0.0164));
  EXPECT_FALSE(gripper_holding(r, -0.0016));
}
