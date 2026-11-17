// test_nudge.cpp — 칸 안 퍽 툭 건드리기 (nudge_n1b): npz · MLP 가 numpy 와 같은가 · 관측 86 순서 · 퍽 자세 · 안전 멈춤 · 받기 거절
//   기대값: test/nudge_ref.hpp (test/gen_nudge_ref.py, numpy_policy.py 와 같은 식)

#include <gtest/gtest.h>

#include <cstdio>
#include <filesystem>
#include <fstream>

#include "manipulation/arm_path.hpp"
#include "manipulation/npz_policy.hpp"
#include "manipulation/nudge.hpp"
#include "nudge_ref.hpp"

using namespace manipulation;

namespace
{
std::string write_tmp(const std::vector<uint8_t> & b, const char * name)
{
  const auto p = std::filesystem::temp_directory_path() / name;
  std::ofstream f(p, std::ios::binary);
  f.write(reinterpret_cast<const char *>(b.data()), static_cast<std::streamsize>(b.size()));
  return p.string();
}
}  // namespace

// 실제 정책 파일 (config/rl/nudge_n1b.npz) 이 numpy 와 1e-5 안 — 파일이 없으면 (config 묶음 전) 건너뜀
TEST(NudgePolicy, ParityWithNumpy)
{
  if (!std::filesystem::exists(NUDGE_NPZ)) {GTEST_SKIP() << "정책 없음: " << NUDGE_NPZ;}
  MlpPolicy p;
  std::string why;
  ASSERT_TRUE(p.load(NUDGE_NPZ, why)) << why;
  EXPECT_EQ(p.obs_dim(), 86u);
  EXPECT_EQ(p.act_dim(), 6u);
  for (std::size_t i = 0; i < nudge_ref::kObs.size(); ++i) {
    const auto a = p.act(nudge_ref::kObs[i]);
    ASSERT_EQ(a.size(), 6u);
    for (int k = 0; k < 6; ++k) {EXPECT_NEAR(a[k], nudge_ref::kAct[i][k], 1e-5) << "관측 " << i << " 행동 " << k;}
  }
  EXPECT_TRUE(p.act(std::vector<double>(85, 0.0)).empty());
}

TEST(NudgePolicy, SmallStoredNpz)
{
  MlpPolicy p;
  std::string why;
  ASSERT_TRUE(p.load(write_tmp(nudge_ref::kSmall_stored, "nudge_small_stored.npz"), why)) << why;
  EXPECT_EQ(p.obs_dim(), 4u);
  EXPECT_EQ(p.act_dim(), 3u);
  for (std::size_t i = 0; i < nudge_ref::kSmallObs.size(); ++i) {
    const auto a = p.act(nudge_ref::kSmallObs[i]);
    ASSERT_EQ(a.size(), 3u);
    for (int k = 0; k < 3; ++k) {EXPECT_NEAR(a[k], nudge_ref::kSmallAct[i][k], 1e-9);}
  }
}

TEST(NudgePolicy, SmallDeflateNpz)
{
  MlpPolicy p;
  std::string why;
  const bool ok = p.load(write_tmp(nudge_ref::kSmall_deflate, "nudge_small_deflate.npz"), why);
#ifdef MANIP_HAVE_ZLIB
  ASSERT_TRUE(ok) << why;
  const auto a = p.act(nudge_ref::kSmallObs[0]);
  ASSERT_EQ(a.size(), 3u);
  for (int k = 0; k < 3; ++k) {EXPECT_NEAR(a[k], nudge_ref::kSmallAct[0][k], 1e-9);}
#else
  EXPECT_FALSE(ok);
  EXPECT_NE(why.find("zlib"), std::string::npos) << why;
#endif
}

TEST(NudgePolicy, BadFilesRejected)
{
  MlpPolicy p;
  std::string why;
  EXPECT_FALSE(p.load("/nonexistent/nudge.npz", why));
  EXPECT_FALSE(why.empty());
  std::vector<uint8_t> junk(100, 7);
  EXPECT_FALSE(p.load(write_tmp(junk, "nudge_junk.npz"), why));
  EXPECT_FALSE(p.loaded());
}

// 관측 86 순서 (nudge_n1b.md 표)
TEST(NudgeObs, ShapeAndOrder)
{
  NudgeObsIn in;
  in.q = {1.0, 2.0, 3.0, 4.0, 5.0};
  in.home = {0.5, 0.5, 0.5, 0.5, 0.5};
  in.qd = {1.0, -2.0, 3.0, -4.0, 5.0};
  in.grip_open = 0.25;
  in.grip_force_n = 10.0;
  in.cell.xy = Eigen::Vector2d(0.078, 0.113);
  in.cell.top_z = 0.1925;
  in.puck_h = 0.060;
  in.tip = Eigen::Vector3d(0.08, 0.12, 0.2225);
  in.wrist_z = Eigen::Vector3d(0.0, 0.6, 0.8);
  in.wrist_y = Eigen::Vector3d(-0.9, -0.3, 0.2);
  in.puck = nudge_puck_manual(in.cell, in.puck_h, 0.015, 15.0, 20.0, 3.0, 0.0, std::nan(""), 4.0);
  in.occupied[4] = 1.0;
  in.occupied[9] = 1.0;
  in.task_index = 4;
  in.prev_action = {0.1, -0.2, 0.3, -0.4, 0.5, -0.6};
  in.step = 30;
  in.tip_z_offset = 0.033;
  const auto o = nudge_obs(in);
  ASSERT_EQ(o.size(), static_cast<std::size_t>(kNudgeObsDim));
  for (int j = 0; j < 5; ++j) {EXPECT_DOUBLE_EQ(o[j], in.q[j] - 0.5);}
  for (int j = 0; j < 5; ++j) {EXPECT_DOUBLE_EQ(o[5 + j], in.qd[j] * 0.1);}
  EXPECT_DOUBLE_EQ(o[10], 0.25);
  EXPECT_DOUBLE_EQ(o[11], 0.5);
  EXPECT_DOUBLE_EQ(o[12], 0.08);
  EXPECT_DOUBLE_EQ(o[13], 0.12);
  EXPECT_NEAR(o[14], 0.2225 + 0.033, 1e-12);
  EXPECT_DOUBLE_EQ(o[16], 0.6);
  EXPECT_DOUBLE_EQ(o[18], -0.9);
  const Eigen::Vector3d tgt(0.078, 0.113, 0.1625);
  const Eigen::Vector3d pc = in.puck.centre;
  EXPECT_NEAR(pc.x(), 0.081, 1e-12);
  EXPECT_NEAR(pc.z(), 0.1665, 1e-12);
  for (int k = 0; k < 3; ++k) {
    EXPECT_NEAR(o[21 + k], pc[k] - in.tip[k], 1e-12);
    EXPECT_NEAR(o[27 + k], tgt[k] - pc[k], 1e-12);
    EXPECT_NEAR(o[30 + k], tgt[k] - in.tip[k], 1e-12);
  }
  EXPECT_NEAR(o[26], std::cos(15.0 * M_PI / 180.0), 1e-12);   // 위쪽 축 z
  EXPECT_NEAR(o[24], std::sin(15.0 * M_PI / 180.0), 1e-12);   // dx 쪽으로 기욺
  EXPECT_DOUBLE_EQ(o[33], 1.0);
  EXPECT_DOUBLE_EQ(o[34], 0.0);
  for (int k = 0; k < 12; ++k) {
    EXPECT_DOUBLE_EQ(o[35 + k], (k == 4 || k == 9) ? 1.0 : 0.0);
    EXPECT_DOUBLE_EQ(o[47 + k], k == 4 ? 1.0 : 0.0);
    EXPECT_DOUBLE_EQ(o[59 + k], 0.0);
  }
  EXPECT_DOUBLE_EQ(o[71], 0.0);
  EXPECT_DOUBLE_EQ(o[72], 0.0);
  for (int k = 0; k < 6; ++k) {EXPECT_DOUBLE_EQ(o[73 + k], in.prev_action[k]);}
  EXPECT_NEAR(o[79], 0.75, 1e-12);
  EXPECT_NEAR(o[80], std::sin(4.0 * 20.0 * M_PI / 180.0), 1e-12);
  EXPECT_NEAR(o[81], std::cos(4.0 * 20.0 * M_PI / 180.0), 1e-12);
  EXPECT_NEAR(o[82], 0.4, 1e-9);
  EXPECT_NEAR(o[83], 15.0 / 20.0, 1e-9);
  EXPECT_NEAR(o[84], 0.3, 1e-9);
  EXPECT_NEAR(o[85], 0.0, 1e-9);
}

TEST(NudgePuckPose, ManualDefaultsAndSeat)
{
  SeatCellRef c;
  c.xy = Eigen::Vector2d(0.02, -0.07);
  c.yaw = 0.0;
  c.top_z = 0.1925;
  // 방향 없고 dx · dy 0 → 칸 x 축, dz 없음 → 바닥 모서리 기울기 꼴
  const auto p = nudge_puck_manual(c, 0.060, 0.015, 15.0, 0.0, 0.0, 0.0);
  EXPECT_NEAR(p.tilt_deg(), 15.0, 1e-9);
  EXPECT_GT(p.up.x(), 0.2);
  EXPECT_NEAR(p.up.y(), 0.0, 1e-12);
  const double dz = 0.015 * std::sin(15.0 * M_PI / 180.0) - 0.03 * (1.0 - std::cos(15.0 * M_PI / 180.0));
  EXPECT_NEAR(p.centre.z(), 0.1625 + dz, 1e-12);
  EXPECT_EQ(p.source, "manual");
  // 넣은 뒤 보기: 바로 앉음 (0, 0, 0, 단위 자세) → 바른 자리 그대로
  const auto s = nudge_puck_from_seat(c, 0.060, 0.0, 0.0, 0.0, Eigen::Matrix3d::Identity(), 0.0);
  EXPECT_NEAR((s.centre - nudge_target(c, 0.060)).norm(), 0.0, 1e-12);
  EXPECT_NEAR(s.tilt_deg(), 0.0, 1e-9);
  // 칸이 90° 돌면 칸 축 x 3 mm = base y 3 mm
  c.yaw = M_PI / 2.0;
  const Eigen::Matrix3d R = Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitY()).toRotationMatrix();
  const auto s2 = nudge_puck_from_seat(c, 0.060, 3.0, 0.0, 5.0, R, 0.0);
  EXPECT_NEAR(s2.tilt_deg(), 0.2 * 180.0 / M_PI, 1e-9);
  EXPECT_NEAR(s2.up.y(), std::sin(0.2), 1e-9);   // 칸 x 축 (= base y) 쪽으로 기욺
  EXPECT_NEAR(s2.centre.y() + s2.up.y() * 0.03, c.xy.y() + 0.003, 1e-9);   // 윗면 중심
}

TEST(NudgeManualArg, Parse)
{
  NudgeManual m;
  std::string why;
  EXPECT_TRUE(parse_nudge_manual("", m, why));
  EXPECT_FALSE(m.given);
  EXPECT_TRUE(parse_nudge_manual("12", m, why));   // 태그 없음 = 손 값 아님
  EXPECT_FALSE(m.given);
  ASSERT_TRUE(parse_nudge_manual("nudge:15,20,3,0", m, why)) << why;
  EXPECT_TRUE(m.given);
  EXPECT_DOUBLE_EQ(m.tilt_deg, 15.0);
  EXPECT_DOUBLE_EQ(m.face_deg, 20.0);
  EXPECT_DOUBLE_EQ(m.dx_mm, 3.0);
  EXPECT_DOUBLE_EQ(m.dy_mm, 0.0);
  EXPECT_TRUE(std::isnan(m.dir_deg));
  ASSERT_TRUE(parse_nudge_manual("nudge:10,0,-2.5,1,90,3", m, why));
  EXPECT_DOUBLE_EQ(m.dir_deg, 90.0);
  EXPECT_DOUBLE_EQ(m.dz_mm, 3.0);
  EXPECT_FALSE(parse_nudge_manual("nudge:15,20,3", m, why));
  EXPECT_FALSE(parse_nudge_manual("nudge:15,x,3,0", m, why));
  EXPECT_FALSE(parse_nudge_manual("nudge:75,0,0,0", m, why));
}

// nudge_enable false → 분명한 거절 글
TEST(NudgeGate, DisabledRejectsWithClearMessage)
{
  NudgeGateIn g;
  g.enabled = false;
  g.slot_known = g.model_ok = g.policy_ok = true;
  const std::string w = nudge_gate(g);
  EXPECT_NE(w.find("nudge_enable"), std::string::npos) << w;
  EXPECT_NE(w.find("꺼짐"), std::string::npos) << w;
  g.enabled = true;
  EXPECT_EQ(nudge_gate(g), "");
  g.carrying = true;
  EXPECT_NE(nudge_gate(g).find("빈 손"), std::string::npos);
  g.carrying = false;
  g.policy_ok = false;
  g.policy_why = "파일 못 엶";
  EXPECT_NE(nudge_gate(g).find("파일 못 엶"), std::string::npos);
}

namespace
{
// 가짜 FK: 칸 중심 (0, 0) · 윗면 0.19 위 30 mm 에서 J1 → x, J2 → y, J3 → z (1 rad = 0.1 m)
Eigen::Vector3d fake_tip(const std::array<double, 5> & q)
{
  return Eigen::Vector3d(0.1 * q[0], 0.1 * q[1], 0.22 + 0.1 * q[2]);
}
NudgeConfig fake_cfg()
{
  NudgeConfig c;
  c.speed_scale = 1.0;
  c.lo.fill(-2.0);
  c.hi.fill(2.0);
  c.cell_xy = Eigen::Vector2d::Zero();
  c.top_z = 0.19;
  c.max_steps = 120;
  return c;
}
NudgeSense sense(const std::array<double, 5> & q, double cur = 0.0)
{
  NudgeSense s;
  s.q = q;
  s.effort.fill(cur);
  return s;
}
}  // namespace

// 계속 +x 로 미는 가짜 정책 (손끝 윗면 + 10 mm, 낮음) → 손끝이 상자 (±40 mm) 를 넘기 전에 멈춤 (목표는 실측 그대로)
TEST(NudgeRunnerSafety, BoxViolationStops)
{
  NudgeRunner r;
  const std::array<double, 5> q0{};
  NudgeConfig cfg = fake_cfg();
  cfg.top_z = 0.21;
  r.start(cfg, q0, std::array<double, 5>{}, fake_tip);
  std::array<double, 5> q = q0;
  NudgeStepOut o;
  int n = 0;
  for (; n < 200; ++n) {
    o = r.step(sense(q), {1.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    if (o.end != NudgeEnd::NONE) {break;}
    EXPECT_LE(std::fabs(fake_tip(o.target).x()), 0.040 + 1e-9);
    q = o.target;   // 재생기가 목표를 그대로 따라감
  }
  ASSERT_EQ(o.end, NudgeEnd::SAFETY);
  EXPECT_NE(o.why.find("수평"), std::string::npos) << o.why;
  for (int j = 0; j < 5; ++j) {EXPECT_DOUBLE_EQ(o.target[j], q[j]);}   // 멈춤 = 실측
  // 0.70 rad/s · 1/30 s = 0.0233 rad/걸음 → 손끝 2.33 mm/걸음 → 0.4 rad 까지 17 걸음
  EXPECT_EQ(n, 17);
}

// 윗면 + 30 mm 에서 옆으로 나감 = 일 끝내고 떠남 (LEFT, 안전 멈춤 아님 · 지금 목표에서 멈춤)
TEST(NudgeRunnerSafety, LeavingAboveIsLeft)
{
  NudgeRunner r;
  r.start(fake_cfg(), std::array<double, 5>{}, std::array<double, 5>{}, fake_tip);
  std::array<double, 5> q{};
  NudgeStepOut o;
  for (int n = 0; n < 200 && o.end == NudgeEnd::NONE; ++n) {o = r.step(sense(q), {1.0, 0.0, 0.0, 0.0, 0.0, 0.0}); if (o.end == NudgeEnd::NONE) {q = o.target;}}
  ASSERT_EQ(o.end, NudgeEnd::LEFT) << o.why;
  for (int j = 0; j < 5; ++j) {EXPECT_DOUBLE_EQ(o.target[j], q[j]);}
  EXPECT_LE(fake_tip(o.target).x(), 0.040);
}

// 퍽이 손끝을 따라 내려감: 처음 10 mm 뜬 15° 퍽, 손끝이 윗면 아래로 → 높이 · 기울기 · 어긋남 같은 비율로 줄어듦
TEST(NudgePuckPose, FollowTip)
{
  SeatCellRef c;
  c.xy = Eigen::Vector2d(0.0, 0.0);
  c.top_z = 0.19;
  const NudgePuck p0 = nudge_puck_manual(c, 0.060, 0.015, 15.0, 0.0, 4.0, 0.0, std::nan(""), 10.0);
  NudgePuck p = p0;
  const double top0 = p0.centre.z() + 0.03 * p0.up.z();
  EXPECT_FALSE(nudge_puck_follow(p, p0, Eigen::Vector3d(0.004, 0.0, top0 + 0.001), c, 0.060, 0.02));   // 위 = 안 바뀜
  EXPECT_FALSE(nudge_puck_follow(p, p0, Eigen::Vector3d(0.05, 0.0, top0 - 0.005), c, 0.060, 0.02));    // 옆 = 안 바뀜
  ASSERT_TRUE(nudge_puck_follow(p, p0, Eigen::Vector3d(0.004, 0.0, top0 - 0.005), c, 0.060, 0.02));
  const double dz = p.centre.z() - nudge_target(c, 0.060).z();
  EXPECT_LT(dz, 0.010);
  EXPECT_GT(dz, 0.004);
  EXPECT_NEAR(p.tilt_deg(), 15.0 * dz / 0.010, 1e-6);
  EXPECT_NEAR(p.centre.x(), 0.004 * dz / 0.010, 1e-9);
  // 바른 자리 밑으론 안 감 · 다시 올라가지 않음
  ASSERT_TRUE(nudge_puck_follow(p, p0, Eigen::Vector3d(0.0, 0.0, 0.10), c, 0.060, 0.02));
  EXPECT_NEAR(p.centre.z(), nudge_target(c, 0.060).z(), 1e-12);
  EXPECT_NEAR(p.tilt_deg(), 0.0, 1e-9);
  EXPECT_FALSE(nudge_puck_follow(p, p0, Eigen::Vector3d(0.0, 0.0, 0.25), c, 0.060, 0.02));
  // 처음부터 바른 높이면 안 바꿈
  NudgePuck q0 = nudge_puck_manual(c, 0.060, 0.015, 15.0, 0.0, 0.0, 0.0, std::nan(""), 0.0);
  NudgePuck q = q0;
  EXPECT_FALSE(nudge_puck_follow(q, q0, Eigen::Vector3d(0.0, 0.0, 0.17), c, 0.060, 0.02));
}

// 아래로 미는 정책 → 윗면 −5 mm 전에 멈춤
TEST(NudgeRunnerSafety, BelowTopStops)
{
  NudgeRunner r;
  r.start(fake_cfg(), std::array<double, 5>{}, std::array<double, 5>{}, fake_tip);
  std::array<double, 5> q{};
  NudgeStepOut o;
  for (int n = 0; n < 200; ++n) {
    o = r.step(sense(q), {0.0, 0.0, -1.0, 0.0, 0.0, 0.0});
    if (o.end != NudgeEnd::NONE) {break;}
    EXPECT_GE(fake_tip(o.target).z(), 0.19 - 0.005 - 1e-9);
    q = o.target;
  }
  ASSERT_EQ(o.end, NudgeEnd::SAFETY);
  EXPECT_NE(o.why.find("윗면"), std::string::npos) << o.why;
}

// 전류 변화 (2 걸음 이어짐) · 따라가기 오차 · 관절 한계 → 멈춤
TEST(NudgeRunnerSafety, CurrentLagAndLimits)
{
  NudgeRunner r;
  r.start(fake_cfg(), std::array<double, 5>{}, std::array<double, 5>{}, fake_tip);
  EXPECT_EQ(r.step(sense({}, 0.35), {0, 0, 0, 0, 0, 0}).end, NudgeEnd::NONE);   // 한 걸음은 넘김
  const auto o = r.step(sense({}, 0.35), {0, 0, 0, 0, 0, 0});
  EXPECT_EQ(o.end, NudgeEnd::SAFETY);
  EXPECT_NE(o.why.find("전류"), std::string::npos) << o.why;

  NudgeRunner l;
  l.start(fake_cfg(), std::array<double, 5>{}, std::array<double, 5>{}, fake_tip);
  NudgeStepOut a;
  for (int n = 0; n < 10 && a.end == NudgeEnd::NONE; ++n) {a = l.step(sense({}), {0, 0, 0, 1.0, 0, 0});}   // 팔이 안 따라옴 (막힘)
  EXPECT_EQ(a.end, NudgeEnd::SAFETY);
  EXPECT_NE(a.why.find("막힘"), std::string::npos) << a.why;

  NudgeConfig c = fake_cfg();
  c.hi[4] = 0.03;
  NudgeRunner j;
  j.start(c, std::array<double, 5>{}, std::array<double, 5>{}, fake_tip);
  std::array<double, 5> q{};
  NudgeStepOut b;
  for (int n = 0; n < 10 && b.end == NudgeEnd::NONE; ++n) {b = j.step(sense(q), {0, 0, 0, 0, 1.0, 0}); q = b.target;}
  EXPECT_EQ(b.end, NudgeEnd::SAFETY);
  EXPECT_NE(b.why.find("J5"), std::string::npos) << b.why;
}

// 정책이 가만히 (손끝 윗면 + 15 mm 위) 10 걸음 → IDLE · 걸음 다 쓰면 MAX_STEPS · a[5] 안 씀 · 행동 [−1, 1] 자름
TEST(NudgeRunnerEnd, IdleAndMaxSteps)
{
  NudgeRunner r;
  r.start(fake_cfg(), std::array<double, 5>{}, std::array<double, 5>{}, fake_tip);
  NudgeStepOut o;
  int n = 0;
  for (; n < 50 && o.end == NudgeEnd::NONE; ++n) {o = r.step(sense(r.target()), {0.01, 0, 0, 0, 0, 0.9});}
  EXPECT_EQ(o.end, NudgeEnd::IDLE);
  EXPECT_EQ(n, 10);
  EXPECT_DOUBLE_EQ(r.prev_action()[5], 0.9);

  NudgeConfig c = fake_cfg();
  c.max_steps = 5;
  NudgeRunner m;
  m.start(c, std::array<double, 5>{}, std::array<double, 5>{}, fake_tip);
  o = NudgeStepOut{};
  for (n = 0; n < 50 && o.end == NudgeEnd::NONE; ++n) {o = m.step(sense(m.target()), {0.5, 0, 0, 0, 0, 0});}
  EXPECT_EQ(o.end, NudgeEnd::MAX_STEPS);
  EXPECT_EQ(n, 5);
  NudgeRunner k;
  k.start(fake_cfg(), std::array<double, 5>{}, std::array<double, 5>{}, fake_tip);
  o = k.step(sense({}), {5.0, 0, 0, 0, 0, 0});
  EXPECT_NEAR(o.target[0], 0.70 / 30.0, 1e-12);
  EXPECT_DOUBLE_EQ(k.prev_action()[0], 1.0);
  // 속도 비율 0.5 = 반
  NudgeConfig h = fake_cfg();
  h.speed_scale = 0.5;
  NudgeRunner s;
  s.start(h, std::array<double, 5>{}, std::array<double, 5>{}, fake_tip);
  EXPECT_NEAR(s.step(sense({}), {1.0, 0, 0, 0, 0, 0}).target[0], 0.35 / 30.0, 1e-12);
}

// 시작 실측이 관절 한계를 조금 넘어도 (기록 경로 끝 · 처짐) 첫 걸음에 멈추지 않음 — 더 밖으로는 안 감
TEST(NudgeRunnerSafety, StartBeyondLimitIsAllowedButNotFurther)
{
  NudgeConfig c = fake_cfg();
  c.hi[3] = 0.5;
  std::array<double, 5> q0{};
  q0[3] = 0.51;
  NudgeRunner r;
  r.start(c, q0, std::array<double, 5>{}, fake_tip);
  EXPECT_EQ(r.step(sense(q0), {0, 0, 0, 0, 0, 0}).end, NudgeEnd::NONE);
  EXPECT_EQ(r.step(sense(q0), {0, 0, 0, -1.0, 0, 0}).end, NudgeEnd::NONE);   // 안쪽으로는 감
  std::array<double, 5> q = r.target();
  const auto o = r.step(sense(q), {0, 0, 0, 1.0, 0, 0});
  EXPECT_EQ(o.end, NudgeEnd::NONE);   // 시작 자세까지는 다시 감 (0.51 − 0.0233 + 0.0233)
  q = o.target;
  EXPECT_EQ(r.step(sense(q), {0, 0, 0, 1.0, 0, 0}).end, NudgeEnd::SAFETY);   // 시작보다 밖 = 멈춤
}

// 취소 때 팔이 마지막 명령보다 0.06 rad 넘게 뒤처져 (명령 꺾은선에서도 벗어남) 있으면 HOME 거꾸로 가기가 거절 →
//   nudge_hold_trace 로 붙잡은 실측 자세를 더하면 그 자리에서 지나온 길로 출구 (P0) 까지
TEST(NudgeCancel, HoldPoseJoinsTraceSoHomeRetreatWorks)
{
  const std::vector<double> p0{0, 0, 0, 0, 0}, c1{0.3, 0, 0, 0, 0}, c2{0.3, 0.3, 0, 0, 0};
  std::vector<std::vector<double>> trace;
  for (const auto * q : {&p0, &c1, &c2}) {trace_append(trace, *q);}
  const std::array<double, 5> meas{0.2, 0.1, 0.0, 0.0, 0.0};   // 마지막 명령 c2 와 0.2 rad, 꺾은선에서 0.1 rad
  const std::vector<double> m(meas.begin(), meas.end());
  auto is_p0 = [&p0](const std::vector<double> & q) {
      for (std::size_t j = 0; j < q.size(); ++j) {if (std::fabs(q[j] - p0[j]) > 1e-3) {return false;}}
      return true;
    };
  std::string why;
  EXPECT_TRUE(retreat_from_trace(trace, m, 0.06, 0.01, is_p0, 20.0, why).empty());   // 고치기 전: 지나온 길에서 벗어남
  EXPECT_FALSE(why.empty());
  nudge_hold_trace(trace, meas, 0);
  why.clear();
  const auto nodes = retreat_from_trace(trace, m, 0.06, 0.01, is_p0, 20.0, why);
  ASSERT_GE(nodes.size(), 2u) << why;
  EXPECT_TRUE(is_p0(nodes.back().q));
  EXPECT_NEAR(nodes.front().q[0], 0.2, 1e-9);   // 지금 자세에서 출발
}
