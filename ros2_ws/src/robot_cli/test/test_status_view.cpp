// test_status_view.cpp — 0 쪽 배지 줄 · 1~7 쪽 표 · 주행 막기 · 점검 · 한계 · 쪽 동작
#include <gtest/gtest.h>

#include "robot_cli/status_view.hpp"

using namespace robot_cli;

namespace
{
const Line * find(const std::vector<Line> & lines, const std::string & part)
{
  for (const auto & l : lines) {
    if (l.text.find(part) != std::string::npos) {return &l;}
  }
  return nullptr;
}

// 이름 칸이 label 인 배지 줄
const Line * block_of(const std::vector<Line> & lines, const std::string & label)
{
  for (const auto & l : lines) {
    if (l.mark_len > 0 && l.text.rfind(label + " ", 0) == 0) {return &l;}
  }
  return nullptr;
}

std::string badge(const Line & l) {return l.text.substr(l.mark_from, l.mark_len);}

Config make_cfg()
{
  Config c;
  c.cal = {{"joint1", {4088, 1.0}}, {"joint2", {2048, 1.0}}, {"joint4", {2048, 1.0}}, {"gripper_left_joint", {3549, 0.01149}}};
  c.stops = {{"start", {0, 0, 0}}, {"B1", {-0.734, 0.2935, -1.57}}};
  c.arm.poses = {{"받침", {0.0, -1.45, 0.95, -0.12, 0.02}}, {"P0", {0.0, -1.38, 0.63, -0.02, 0.02}},
    {"기준점", {0.18, 0.41, -0.67, 0.19, 0.21}}};
  c.arm.fold_tol = {0.05};
  c.arm.lazy_fold = true;
  c.arm.grip_open = 0.0164;
  c.arm.grip_close = -0.0016;
  return c;
}
const Config kCfg = make_cfg();

void set_arm(Snapshot & s, const std::vector<double> & q)
{
  for (std::size_t i = 0; i < 5; ++i) {s.joints["joint" + std::to_string(i + 1)] = q[i];}
  s.joints_t = s.now;
}
}  // namespace

TEST(StatusView, SummaryBadges)
{
  Snapshot s;
  s.now = 100.0;
  const auto empty = status_lines(s, kCfg);
  const Line * w = block_of(empty, "바퀴");
  ASSERT_NE(w, nullptr);
  EXPECT_EQ(badge(*w), "[문제]");
  EXPECT_EQ(w->level, Level::BAD);
  const Line * loc = block_of(empty, "위치");
  ASSERT_NE(loc, nullptr);
  EXPECT_EQ(badge(*loc), "[꺼짐]");
  EXPECT_EQ(loc->level, Level::OFF);
  // 요약에는 눈금 · 토픽 이름 · 노드 목록이 없음
  for (const auto & l : empty) {
    EXPECT_EQ(l.text.find("/joint_states"), std::string::npos) << l.text;
    EXPECT_EQ(l.text.find("controller_manager control_master"), std::string::npos) << l.text;
    EXPECT_LE(text_width(l.text), kMaxWidth) << l.text;
  }

  // lazy_fold: STORE 성공 뒤 P0, 차례 STOP → 바퀴 [잠금] "주행 전 home", 팔 [P0] + 마지막 작업
  s.mode_t = 99.9;
  s.mode = 0;
  s.ready = true;
  set_arm(s, {0.0, -1.38, 0.64, -0.02, 0.02});
  s.joints["gripper_left_joint"] = -0.0016;
  TaskTiming h;
  h.id = "aaaa0001";
  h.what = "STORE S12";
  h.t0 = 70.0;
  h.t1 = 89.4;
  h.status = 4;
  s.arm_hist = {h};
  const auto lines = status_lines(s, kCfg);
  const Line * wl = block_of(lines, "바퀴");
  ASSERT_NE(wl, nullptr);
  EXPECT_EQ(badge(*wl), "[잠금]");
  EXPECT_NE(wl->text.find("주행 전 home"), std::string::npos);
  const Line * al = block_of(lines, "팔");
  ASSERT_NE(al, nullptr);
  EXPECT_EQ(badge(*al), "[P0]");
  EXPECT_NE(al->text.find("마지막 STORE S12 성공 19.4 s"), std::string::npos);
  EXPECT_NE(al->text.find("그리퍼 15 mm"), std::string::npos);

  // 접힘 + DRIVE → [주행 가능] · [받침]
  s.mode = 1;
  set_arm(s, {0.01, -1.46, 0.95, -0.12, 0.02});
  const auto d = status_lines(s, kCfg);
  EXPECT_EQ(badge(*block_of(d, "바퀴")), "[주행 가능]");
  EXPECT_EQ(badge(*block_of(d, "팔")), "[받침]");
  // 퍽 쥠 (반쯤 닫힘)
  s.joints["gripper_left_joint"] = 0.006;
  const auto g = status_lines(s, kCfg);
  EXPECT_NE(find(g, "그리퍼 쥠"), nullptr);
}

TEST(StatusView, BatteryAndLimits)
{
  Snapshot s;
  s.now = 50.0;
  const auto none = status_lines(s, kCfg);
  EXPECT_EQ(badge(*block_of(none, "배터리")), "[모름]");
  s.battery["bus_arm"] = {10.6, 10.5, 49.0};
  const auto warn = status_lines(s, kCfg);
  const Line * b = block_of(warn, "배터리");
  EXPECT_EQ(badge(*b), "[주의]");
  EXPECT_EQ(b->level, Level::WARN);
  s.battery["bus_drive"] = {10.0, 9.9, 49.0};
  EXPECT_EQ(block_of(status_lines(s, kCfg), "배터리")->level, Level::BAD);

  // joint4 위쪽 한계 0.6735 (≈ 눈금 2487, 실물 멈춤 2485) 가까이
  set_arm(s, {0.0, -1.0, 0.5, 0.66, 0.0});
  s.limits = {{"joint4", {-1.7, 0.6735}}};
  Level worst = Level::NORMAL;
  auto w = limit_warnings(s, kCfg, worst);
  ASSERT_EQ(w.size(), 1u);
  EXPECT_EQ(worst, Level::WARN);
  EXPECT_NE(w[0].find("joint4 위쪽"), std::string::npos);
  EXPECT_NE(w[0].find("한계 2487"), std::string::npos);
  EXPECT_NE(block_of(status_lines(s, kCfg), "관절"), nullptr);
  s.joints["joint4"] = 0.70;
  limit_warnings(s, kCfg, worst);
  EXPECT_EQ(worst, Level::BAD);
  s.joints["joint4"] = 0.0;
  EXPECT_TRUE(limit_warnings(s, kCfg, worst).empty());
  EXPECT_EQ(block_of(status_lines(s, kCfg), "관절"), nullptr);   // 멀면 줄이 없음
}

// 모터 온도: 2 쪽 관절 표 온도 칸 (경고 · 위험이면 그 칸만 색) · 주행 모터 표 · 경고 넘으면 0 · 2 쪽 배지 줄
TEST(StatusView, MotorTemperature)
{
  Snapshot s;
  s.now = 100.0;
  s.mode_t = 99.9;
  set_arm(s, {0.0, -1.0, 0.5, 0.0, 0.0});
  s.joints["gripper_left_joint"] = 0.0164;
  const auto none = page_lines(2, s, kCfg, UiState{});
  EXPECT_NE(find(none, "온도[°C]"), nullptr);
  const Line * j1 = find(none, "joint1  ");
  ASSERT_NE(j1, nullptr);
  EXPECT_EQ(j1->level, Level::OFF) << "온도를 못 받으면 그 칸만 회색";
  EXPECT_EQ(j1->text.substr(j1->mark_from, j1->mark_len), "        -");
  EXPECT_NE(find(none, "없음 — OpenCR 하드웨어"), nullptr);
  EXPECT_EQ(block_of(status_lines(s, kCfg), "온도"), nullptr);

  s.temps = {{"joint1", {41, 99.5}}, {"joint2", {63, 99.5}}, {"joint3", {72, 99.5}}, {"joint4", {-1, 99.5}},
    {"gripper_left_joint", {45, 99.5}}, {"drive_fl_joint", {38, 99.6}}, {"steer_rr_joint", {61, 99.6}}};
  const auto p = page_lines(2, s, kCfg, UiState{});
  for (const auto & l : p) {EXPECT_LE(text_width(l.text), kMaxWidth) << l.text;}
  auto cell = [](const Line & l) {return l.text.substr(l.mark_from, l.mark_len);};
  const Line * a1 = find(p, "joint1  ");
  ASSERT_NE(a1, nullptr);
  EXPECT_EQ(a1->level, Level::NORMAL);
  EXPECT_EQ(a1->mark_len, 0u) << "보통 온도는 색 없음";
  EXPECT_NE(a1->text.find("      41"), std::string::npos);
  const Line * a2 = find(p, "joint2  ");
  ASSERT_NE(a2, nullptr);
  EXPECT_EQ(a2->level, Level::WARN);
  EXPECT_EQ(cell(*a2), "       63");
  const Line * a3 = find(p, "joint3  ");
  ASSERT_NE(a3, nullptr);
  EXPECT_EQ(a3->level, Level::BAD);
  EXPECT_EQ(cell(*a3), "       72");
  EXPECT_EQ(find(p, "joint4  ")->level, Level::OFF) << "보드가 못 읽은 모터 = 회색 -";
  EXPECT_NE(find(p, "열림")->text.find("45"), std::string::npos);
  const Line * wheel = find(p, "바퀴       ");
  ASSERT_NE(wheel, nullptr);
  EXPECT_NE(wheel->text.find("38"), std::string::npos);
  EXPECT_EQ(wheel->level, Level::NORMAL);
  EXPECT_EQ(find(p, "조향       ")->level, Level::WARN);
  const auto sum = status_lines(s, kCfg);
  const Line * tb = block_of(sum, "온도");
  ASSERT_NE(tb, nullptr);
  EXPECT_EQ(badge(*tb), "[문제]");
  EXPECT_EQ(tb->level, Level::BAD);
  EXPECT_NE(tb->text.find("joint2 63 °C · joint3 72 °C"), std::string::npos) << tb->text;
  EXPECT_NE(find(sum, "조향 rr 61 °C"), nullptr);
  EXPECT_NE(block_of(p, "온도"), nullptr);

  // 하드웨어가 멈춰 온도가 끊김: 보통 값은 회색, 뜨거웠던 값은 색 유지 + 관절 측정이 없어도 온도는 보임
  s.now = 120.2;
  s.joints.erase("joint3");
  const auto old = page_lines(2, s, kCfg, UiState{});
  EXPECT_EQ(find(old, "joint1  ")->level, Level::WARN) << "관절 측정도 오래됨 → 줄 전체 주의";
  const Line * gone = find(old, "joint3  ");
  ASSERT_NE(gone, nullptr);
  EXPECT_NE(gone->text.find("측정 없음"), std::string::npos);
  EXPECT_EQ(gone->level, Level::BAD);
  EXPECT_EQ(cell(*gone), "       72");
  EXPECT_NE(find(status_lines(s, kCfg), "마지막 값 21 s 전"), nullptr);
  // 기준은 하드웨어가 보낸 값
  s.now = 100.0;
  s.temps = {{"joint1", {41, 99.5}}};
  s.temp_warn = 40;
  const auto low = status_lines(s, kCfg);
  ASSERT_NE(block_of(low, "온도"), nullptr);
  EXPECT_EQ(badge(*block_of(low, "온도")), "[주의]");
}

TEST(StatusView, DriveBlock)
{
  Snapshot s;
  s.now = 10.0;
  // /odom 이 없거나 끊기면 시계 차이를 잴 수 없음 → force 로도 안 됨
  EXPECT_NE(drive_block_reason(s, true).find("/odom 없음"), std::string::npos);
  s.odom_t = 8.5;
  EXPECT_NE(drive_block_reason(s, true).find("끊김"), std::string::npos);
  s.odom_t = 9.95;
  EXPECT_EQ(drive_block_reason(s, true), "");
  // 노드 목록을 아직 못 받았으면 Nav2 가 떠 있는지 모름
  EXPECT_NE(drive_block_reason(s, false).find("노드 목록"), std::string::npos);
  s.nodes.insert("/controller_manager");
  EXPECT_NE(drive_block_reason(s, false).find("motion_mode 없음"), std::string::npos);
  s.mode_t = 9.9;
  s.mode = 0;
  EXPECT_NE(drive_block_reason(s, false).find("바퀴 잠금 (차례 STOP)"), std::string::npos);
  s.mode = 1;
  EXPECT_EQ(drive_block_reason(s, false), "");
  s.now = 10.8;   // 차례 신호 0.9 s 끊김 → 틱마다 다시 보면 멈춤
  s.odom_t = 10.75;
  EXPECT_NE(drive_block_reason(s, false).find("끊김"), std::string::npos);
  s.now = 10.0;
  s.odom_t = 9.95;
  s.nodes.insert("/bt_navigator");
  EXPECT_NE(drive_block_reason(s, false).find("Nav2"), std::string::npos);
  EXPECT_EQ(drive_block_reason(s, true), "");
  s.clock_off = 0.4;
  EXPECT_NE(drive_block_reason(s, true).find("시계"), std::string::npos);   // force 로도 못 넘김
  s.estop = true;
  EXPECT_NE(drive_block_reason(s, true).find("E-STOP"), std::string::npos);
}

TEST(StatusView, StalePoll)
{
  Snapshot s;
  s.now = 100.0;
  s.hardware = {{"bus_arm", "active"}, {"bus_drive", "active"}};
  s.controllers = {{"arm_controller", "active"}, {"gripper_controller", "active"}, {"swerve_controller", "active"},
    {"joint_state_broadcaster", "active"}};
  s.hw_t = s.ctrl_t = 99.5;
  auto items = preflight(s, kCfg, "96");
  auto level_of = [&](const std::string & n) {
      for (const auto & i : items) {
        if (i.name == n) {return i.level;}
      }
      return Level::NORMAL;
    };
  EXPECT_EQ(level_of("보드"), Level::GOOD);
  EXPECT_EQ(level_of("컨트롤러"), Level::GOOD);
  EXPECT_EQ(block_of(status_lines(s, kCfg), "하드웨어")->level, Level::GOOD);
  s.hw_t = s.ctrl_t = 90.0;   // 답이 10 s 끊김 → 옛 active 목록을 믿지 않음
  items = preflight(s, kCfg, "96");
  EXPECT_EQ(level_of("보드"), Level::BAD);
  EXPECT_EQ(level_of("컨트롤러"), Level::BAD);
  const auto lines = status_lines(s, kCfg);
  EXPECT_EQ(badge(*block_of(lines, "하드웨어")), "[주의]");
}

TEST(StatusView, GoAndCheck)
{
  Snapshot s;
  s.now = 20.0;
  s.odom_t = 19.95;
  s.mode_t = 19.95;
  s.mode = 1;
  set_arm(s, {0.0, -1.45, 0.95, -0.12, 0.02});
  s.joints["gripper_left_joint"] = -0.0016;
  EXPECT_EQ(go_block_reason(s, kCfg), "");
  EXPECT_FALSE(go_needs_fold(s, kCfg));
  set_arm(s, {0.0, -1.38, 0.63, -0.02, 0.02});   // P0 → home 먼저
  EXPECT_TRUE(go_needs_fold(s, kCfg));
  s.joints["gripper_left_joint"] = 0.006;          // 쥠 → 거절
  EXPECT_NE(go_block_reason(s, kCfg).find("퍽"), std::string::npos);

  const auto items = preflight(Snapshot{}, kCfg, "");
  EXPECT_FALSE(preflight_ok(items));
  const auto cl = check_lines(items);
  EXPECT_NE(find(cl, "점검 실패"), nullptr);
  EXPECT_NE(find(cl, "라이다 앞"), nullptr);
  EXPECT_NE(find(cl, "배터리"), nullptr);
}

TEST(StatusView, Pages)
{
  Snapshot s;
  s.now = 100.0;
  s.have_map_pose = true;
  s.mx = -0.70;
  s.my = 0.30;
  const auto nav = page_lines(3, s, kCfg, UiState{});
  const Line * b1 = find(nav, "◀ 가까움");
  ASSERT_NE(b1, nullptr);
  EXPECT_NE(b1->text.find("2  B1"), std::string::npos);   // 번호 2 = B1
  // 팔 작업 시간 표
  TaskTiming h;
  h.id = "abcd1234";
  h.what = "STORE S5";
  h.t0 = 80.0;
  h.t1 = 95.0;
  h.status = 4;
  h.phases = {{"release", 81.0}, {"grasp", 90.0}, {"retreat", 92.5}};
  s.arm_hist = {h};
  const auto arm = page_lines(2, s, kCfg, UiState{});
  const Line * t = find(arm, "abcd1234");
  ASSERT_NE(t, nullptr);
  EXPECT_NE(t->text.find("15.0"), std::string::npos);
  EXPECT_NE(t->text.find("release 9.0 · grasp 2.5 · retreat 2.5"), std::string::npos);
  EXPECT_NE(find(arm, "1 HOME · 2 LOOK"), nullptr);
  // 인지 쪽: 칸 태그 (칸마다 마지막으로 본 것)
  DetView r;
  r.kind = "rack_tag";
  r.label = "S12";
  r.id = "28";
  r.conf = 1.0;
  r.x = 0.0140;
  r.y = -0.0713;
  r.z = 0.1325;
  s.rack_t = 99.5;
  s.rack_n = 1;
  s.rack_seen["c12"] = {r, 99.5};
  const auto per = page_lines(4, s, kCfg, UiState{});
  const Line * rk = find(per, "S12");
  ASSERT_NE(rk, nullptr);
  EXPECT_NE(rk->text.find("28"), std::string::npos);
  EXPECT_NE(rk->text.find("+14.0"), std::string::npos);
  EXPECT_NE(rk->text.find("-71.3"), std::string::npos);
  EXPECT_NE(rk->text.find("멈춤"), std::string::npos);
  // 같은 칸의 태그 · 퍽은 따로 셈, 칸 태그만 받다가 끊기면 요약이 주의
  DetView pk = r;
  pk.kind = "rack_puck";
  pk.id = "3";
  s.rack_seen["p12"] = {pk, 99.5};
  EXPECT_EQ(s.rack_count('c'), 1u);
  EXPECT_EQ(s.rack_count('p'), 1u);
  EXPECT_NE(find(page_lines(4, s, kCfg, UiState{}), "본 칸 태그 1 · 칸 안 퍽 1"), nullptr);
  Snapshot rs;
  rs.now = 100.0;
  rs.rack_t = 99.5;
  rs.rack_seen["c12"] = {r, 99.5};
  const Line * ok = find(page_lines(4, rs, kCfg, UiState{}), "칸 태그 1 칸");
  ASSERT_NE(ok, nullptr);
  EXPECT_EQ(ok->level, Level::GOOD);
  rs.now = 110.0;
  const Line * old = find(page_lines(4, rs, kCfg, UiState{}), "칸 태그 1 칸");
  ASSERT_NE(old, nullptr);
  EXPECT_EQ(old->level, Level::WARN);
  // 실험 쪽: mark 와 눈금 차이
  set_arm(s, {0.0, -1.0, 0.0, 0.0, 0.0});
  UiState ui;
  ui.mark_dir = "/tmp/m";
  ui.last_mark_q = s.joints;
  ui.last_mark_q["joint2"] = -1.0 - 10 * kTickRad;
  const auto ex = page_lines(7, s, kCfg, ui);
  const Line * j2 = find(ex, "joint2");
  ASSERT_NE(j2, nullptr);
  EXPECT_NE(j2->text.find("+10"), std::string::npos);
  EXPECT_NE(find(ex, "/tmp/m"), nullptr);
  for (int k = 0; k < kPages; ++k) {
    const auto p = page_lines(k, Snapshot{}, kCfg, ui);
    EXPECT_FALSE(p.empty()) << k;
    for (const auto & l : p) {
      if (l.level != Level::HEAD && l.text.find(" · ") != std::string::npos) {EXPECT_LE(text_width(l.text), kMaxWidth + 30) << l.text;}
    }
  }
  // 쪽 번호 동작
  EXPECT_EQ(page_actions(2, kCfg)[2].command, "store {}");
  EXPECT_EQ(page_actions(3, kCfg)[1].command, "g B1");
  EXPECT_TRUE(page_actions(5, kCfg)[0].confirm);
  EXPECT_TRUE(page_actions(1, kCfg).empty());
  EXPECT_EQ(text_width("받침 ok"), 7);
}
