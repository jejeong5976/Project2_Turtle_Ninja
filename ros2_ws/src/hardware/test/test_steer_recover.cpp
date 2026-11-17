// test_steer_recover.cpp — 조향 카운터 자동 복구 (steer_recover.hpp) 판단 + 가짜 펌웨어로 CONFIG 두 번 순서
//   실물에서 본 숫자: rr −523/−522 눈금 (+271°) · rr −369 (+258°). rr zero_tick 2565 · scale −1 · 한계 0–4095

#include <gtest/gtest.h>

#include <fcntl.h>
#include <pty.h>
#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <mutex>
#include <thread>
#include <vector>

#include "hardware/steer_recover.hpp"

using hardware::JointSpec;
using hardware::OpenCRLink;

namespace
{
const double K = 2.0 * M_PI / 4096.0;

JointSpec steer(const char * name, uint8_t id, int zero_tick)
{
  JointSpec j{name, id, pkt::MODE_POSITION, 200, 60, 0, -1.0};
  j.zero_rad = (zero_tick - 2048) * K;
  return j;
}

// 실물 bus_drive 순서: 바퀴 4 + 조향 4
std::vector<JointSpec> drive_board()
{
  std::vector<JointSpec> js = {
    {"drive_fl_joint", 1, pkt::MODE_VELOCITY, 0, 40, 0, 1.0},
    {"drive_fr_joint", 2, pkt::MODE_VELOCITY, 0, 40, 0, -1.0},
    {"drive_rl_joint", 41, pkt::MODE_VELOCITY, 0, 40, 0, -1.0},
    {"drive_rr_joint", 42, pkt::MODE_VELOCITY, 0, 40, 0, 1.0},
  };
  js.push_back(steer("steer_fl_joint", 11, 2048));
  js.push_back(steer("steer_fr_joint", 12, 2048));
  js.push_back(steer("steer_rl_joint", 13, 2048));
  js.push_back(steer("steer_rr_joint", 14, 2565));
  return js;
}

std::vector<std::pair<int32_t, int32_t>> drive_limits()
{
  std::vector<std::pair<int32_t, int32_t>> l(4, {pkt::TF_NO_LIMIT, pkt::TF_NO_LIMIT});
  for (int i = 0; i < 4; ++i) {l.push_back({0, 4095});}
  return l;
}

// 관절값 (펌웨어 눈금 → 플러그인과 같은 변환)
double q_of(const JointSpec & j, long tick) {return hardware::tick_to_joint(j, tick);}

std::vector<double> drive_q(long fl, long fr, long rl, long rr)
{
  const auto js = drive_board();
  return {0.0, 0.0, 0.0, 0.0, q_of(js[4], fl), q_of(js[5], fr), q_of(js[6], rl), q_of(js[7], rr)};
}
}  // namespace

// 평소 (전부 한계 안) → 빈 계획 = 동작 그대로
TEST(SteerRecover, AllInRangeGivesEmptyPlan)
{
  const auto js = drive_board();
  // 실물에서 본 다른 세 조향 (−73° · −81° · −81°) + rr 경계 바로 안 (0 · 4095)
  for (long rr : {0L, 4095L, 2565L, 3574L}) {
    const auto p = hardware::plan_steer_recover(js, drive_limits(), drive_q(2879, 2970, 2970, rr));
    EXPECT_EQ(p.mask, 0u) << rr;
    EXPECT_TRUE(p.items.empty()) << rr;
  }
}

// 실물 값: rr −522 눈금 (+271°) → 다시 세면 3574 (−89°), 한계 안 → rr (관절 7) 만
TEST(SteerRecover, RealRrCaseIsFixableByRecount)
{
  const auto js = drive_board();
  const auto p = hardware::plan_steer_recover(js, drive_limits(), drive_q(2879, 2970, 2970, -522));
  ASSERT_EQ(p.items.size(), 1u);
  EXPECT_EQ(p.mask, 1u << 7);
  const auto & it = p.items[0];
  EXPECT_EQ(it.idx, 7u);
  EXPECT_EQ(it.tick, -522);
  EXPECT_EQ(it.after_tick, 3574);
  EXPECT_TRUE(it.fixable);
  EXPECT_NEAR(hardware::tick_to_joint(js[7], it.tick) * 180.0 / M_PI, 271.3, 0.1);
  EXPECT_NEAR(hardware::tick_to_joint(js[7], it.after_tick) * 180.0 / M_PI, -88.7, 0.1);
  // 같은 실제 자리 = 정확히 한 바퀴 차이
  EXPECT_NEAR(hardware::tick_to_joint(js[7], it.tick) - hardware::tick_to_joint(js[7], it.after_tick), 2.0 * M_PI, 1e-9);
  const std::string t = hardware::steer_recover_text(js, drive_limits(), it);
  EXPECT_NE(t.find("steer_rr_joint"), std::string::npos);
  EXPECT_NE(t.find("-522"), std::string::npos);
  EXPECT_NE(t.find("3574"), std::string::npos);
}

// 실물 값 (−369 → 3727) · 위로 넘은 쪽 (4200 → 104) · 여러 바퀴 (−4618 → 3574)
TEST(SteerRecover, OtherOutsideValuesWrapIntoOneTurn)
{
  const auto js = drive_board();
  struct C {long tick, after;};
  for (const C c : {C{-369, 3727}, C{4200, 104}, C{-4618, 3574}, C{-1, 4095}, C{4096, 0}}) {
    const auto p = hardware::plan_steer_recover(js, drive_limits(), drive_q(2048, 2048, 2048, c.tick));
    ASSERT_EQ(p.items.size(), 1u) << c.tick;
    EXPECT_EQ(p.items[0].after_tick, c.after) << c.tick;
    EXPECT_TRUE(p.items[0].fixable);
    EXPECT_EQ(p.mask, 1u << 7);
  }
  // 두 조향이 동시에 밖
  const auto p = hardware::plan_steer_recover(js, drive_limits(), drive_q(-10, 2048, 4300, 2048));
  EXPECT_EQ(p.mask, (1u << 4) | (1u << 6));
}

// 한계 창이 한 바퀴보다 좁으면 다시 세도 밖일 수 있음 → 계획엔 남기되 mask 에서 뺌 (손으로 돌리라는 안내)
TEST(SteerRecover, NarrowWindowNotFixable)
{
  const auto js = drive_board();
  auto lim = drive_limits();
  lim[7] = {1024, 3072};
  const auto p = hardware::plan_steer_recover(js, lim, drive_q(2048, 2048, 2048, -100));   // → 3996, 창 밖
  ASSERT_EQ(p.items.size(), 1u);
  EXPECT_FALSE(p.items[0].fixable);
  EXPECT_EQ(p.mask, 0u);
  const auto p2 = hardware::plan_steer_recover(js, lim, drive_q(2048, 2048, 2048, -2048));   // → 2048, 창 안
  EXPECT_EQ(p2.mask, 1u << 7);
}

// 위치 모드가 아닌 관절 · 한계 모름 · 측정 없음 (NaN) 은 건드리지 않음
TEST(SteerRecover, SkipsNonPositionUnknownLimitAndNaN)
{
  auto js = drive_board();
  auto lim = drive_limits();
  auto q = drive_q(-500, -500, -500, -500);
  q[4] = std::nan("");           // fl 못 읽음
  lim[5] = {pkt::TF_NO_LIMIT, pkt::TF_NO_LIMIT};   // fr 한계 모름
  js[6].mode = pkt::MODE_CURRENT_POSITION;          // 확장 위치 (J1 같은) — 한계 없음
  q[0] = 1e6;                    // 바퀴 (속도) 는 값이 커도 무관
  const auto p = hardware::plan_steer_recover(js, lim, q);
  ASSERT_EQ(p.items.size(), 1u);
  EXPECT_EQ(p.items[0].idx, 7u);
  EXPECT_EQ(p.mask, 1u << 7);
}

TEST(SteerRecover, ErrorTicksWrap)
{
  EXPECT_EQ(hardware::steer_recover_error_ticks(3574, 3574), 0);
  EXPECT_EQ(hardware::steer_recover_error_ticks(3574, 3580), 6);
  EXPECT_EQ(hardware::steer_recover_error_ticks(4094, 2), 4);
  EXPECT_EQ(hardware::steer_recover_error_ticks(2, 4094), -4);
  EXPECT_EQ(hardware::single_turn_tick(-4096), 0);
}

// ============================================================================ 가짜 펌웨어로 순서 확인
namespace
{
// XM430 흉내: 위치 모드로 "바뀌는" CONFIG 에서만 지금 위치를 한 바퀴 안으로 다시 셈 (같은 모드 다시 쓰기는 그대로 — 실물 −523 유지).
//   토크는 CMD 가 와야 켬 → 이 순서에선 CMD 가 없어야 함 (cmds == 0). CONFIG 뒤 ACK → POS_LIMITS → STATE 5 ms 마다 (실물 10 ms)
class RecountFirmware
{
public:
  RecountFirmware(int fd, std::vector<long> raw) : fd_(fd), raw_(std::move(raw)), mode_(raw_.size(), pkt::MODE_POSITION)
  {
    th_ = std::thread([this] {run();});
  }
  ~RecountFirmware() {stop_ = true; th_.join();}
  std::atomic<int> configs{0};
  std::atomic<int> cmds{0};
  std::atomic<uint16_t> last_velocity_mask{0};
  std::atomic<bool> recount_on_mode_change{true};
  long raw(std::size_t i) {std::lock_guard<std::mutex> g(m_); return raw_[i];}

private:
  void run()
  {
    pkt::Parser p;
    uint8_t in[256], out[pkt::MAX_FRAME];
    auto last = std::chrono::steady_clock::now();
    while (!stop_) {
      const ssize_t n = ::read(fd_, in, sizeof(in));
      for (ssize_t k = 0; k < n; ++k) {
        if (!p.feed(in[k])) {continue;}
        if (p.type() == pkt::TYPE_CMD) {++cmds; continue;}
        if (p.type() != pkt::TYPE_CONFIG) {continue;}
        pkt::Config c;
        ASSERT_TRUE(pkt::decode_config(p.data(), p.len(), c));
        std::lock_guard<std::mutex> g(m_);
        cfg_ = c;
        uint16_t vm = 0;
        pkt::PosLimits l{};
        l.n = c.n;
        for (uint8_t i = 0; i < c.n && i < raw_.size(); ++i) {
          if (c.j[i].mode == pkt::MODE_VELOCITY) {vm = static_cast<uint16_t>(vm | (1u << i));}
          if (recount_on_mode_change && c.j[i].mode == pkt::MODE_POSITION && mode_[i] != pkt::MODE_POSITION) {
            raw_[i] = hardware::single_turn_tick(raw_[i]);
          }
          mode_[i] = c.j[i].mode;
          l.lo[i] = l.hi[i] = pkt::TF_NO_LIMIT;
          if (c.j[i].mode == pkt::MODE_POSITION) {l.lo[i] = 0; l.hi[i] = 4095;}
        }
        last_velocity_mask = vm;
        ++configs;
        pkt::ConfigAck a{c.n, static_cast<uint16_t>((1u << c.n) - 1)};
        write_all(out, pkt::encode_ack(a, out, sizeof(out)));
        write_all(out, pkt::encode_pos_limits(l, out, sizeof(out)));
        configured_ = true;
      }
      if (configured_ && std::chrono::steady_clock::now() - last > std::chrono::milliseconds(5)) {
        last = std::chrono::steady_clock::now();
        std::lock_guard<std::mutex> g(m_);
        pkt::State s{};
        s.status = pkt::ST_CONFIGURED | pkt::ST_IMU_OK;
        s.n = cfg_.n;
        for (uint8_t i = 0; i < s.n; ++i) {
          s.pos[i] = cfg_.j[i].mode == pkt::MODE_VELOCITY ? 0.0f : static_cast<float>((raw_[i] - 2048) * K);
        }
        write_all(out, pkt::encode_state(s, out, sizeof(out)));
      }
      std::this_thread::sleep_for(std::chrono::microseconds(300));
    }
  }
  void write_all(const uint8_t * b, uint16_t len)
  {
    uint16_t done = 0;
    while (done < len) {
      const ssize_t w = ::write(fd_, b + done, len - done);
      if (w > 0) {done = static_cast<uint16_t>(done + w);}
    }
  }
  int fd_;
  std::mutex m_;
  std::vector<long> raw_;
  std::vector<uint8_t> mode_;
  pkt::Config cfg_{};
  bool configured_ = false;
  std::atomic<bool> stop_{false};
  std::thread th_;
};

struct Pty
{
  int master = -1, slave = -1;
  Pty()
  {
    EXPECT_EQ(openpty(&master, &slave, nullptr, nullptr, nullptr), 0);
    termios t{};
    for (int fd : {master, slave}) {
      tcgetattr(fd, &t);
      cfmakeraw(&t);
      tcsetattr(fd, TCSANOW, &t);
      fcntl(fd, F_SETFL, O_NONBLOCK);
    }
  }
  ~Pty() {if (master >= 0) {::close(master);}}
};

// 플러그인 on_configure 와 같은 판단: 지금 STATE · POS_LIMITS → 계획
hardware::SteerRecoverPlan plan_now(OpenCRLink & link)
{
  std::vector<std::pair<int32_t, int32_t>> lim;
  EXPECT_TRUE(link.pos_limits(lim));
  std::vector<double> q;
  for (std::size_t i = 0; i < link.size(); ++i) {q.push_back(link.joint_measured(i, 1.0) ? link.state().joints[i].position : NAN);}
  return hardware::plan_steer_recover(link.joints(), lim, q);
}
}  // namespace

// 실물 순서: CONFIG → rr 밖 → CONFIG (rr 속도) → CONFIG (원래) → rr 3574 (−89°), 한계 안. 그 사이 CMD (토크) 없음
TEST(SteerRecoverLink, VelocityThenPositionConfigRecountsOnlyThatMotor)
{
  Pty pty;
  RecountFirmware fw(pty.master, {0, 0, 0, 0, 2879, 2970, 2970, -522});
  OpenCRLink link(drive_board(), 15);
  link.adopt_fd(pty.slave);
  ASSERT_EQ(link.configure(2.0), 0xFF);
  ASSERT_TRUE(link.wait_for_state(1.0));
  // 같은 모드 다시 쓰기는 다시 안 셈 (실물: CONFIG 뒤에도 −523)
  auto p = plan_now(link);
  ASSERT_EQ(p.mask, 1u << 7);
  EXPECT_EQ(fw.last_velocity_mask.load(), 0x0F);   // 바퀴만 속도

  ASSERT_EQ(link.configure(2.0, p.mask), 0xFF);
  EXPECT_EQ(fw.last_velocity_mask.load(), 0x0F | (1u << 7));   // rr 만 이번에 속도
  ASSERT_EQ(link.configure(2.0), 0xFF);
  EXPECT_EQ(fw.last_velocity_mask.load(), 0x0F);
  ASSERT_TRUE(link.wait_for_state(1.0));
  ASSERT_TRUE(link.wait_for_state(1.0));

  EXPECT_EQ(fw.raw(7), 3574);
  EXPECT_EQ(fw.raw(4), 2879) << "다른 조향은 그대로";
  const auto after = plan_now(link);
  EXPECT_TRUE(after.items.empty());
  const auto & js = link.joints();
  EXPECT_EQ(hardware::motor_tick(js[7], link.state().joints[7].position), 3574);
  EXPECT_NEAR(link.state().joints[7].position * 180.0 / M_PI, -88.7, 0.1);
  EXPECT_EQ(fw.cmds.load(), 0) << "복구 중 CMD (토크) 를 안 보냄 → 모터 안 움직임";
  EXPECT_EQ(fw.configs.load(), 3);
}

// 모터가 다시 세지 않으면 (가정이 틀림) 계획이 그대로 남음 → 플러그인은 손으로 돌리라는 안내 · 활성화 때 ERROR
TEST(SteerRecoverLink, NoRecountLeavesPlanSoOldPathRuns)
{
  Pty pty;
  RecountFirmware fw(pty.master, {0, 0, 0, 0, 2048, 2048, 2048, -522});
  fw.recount_on_mode_change = false;
  OpenCRLink link(drive_board(), 15);
  link.adopt_fd(pty.slave);
  ASSERT_EQ(link.configure(2.0), 0xFF);
  ASSERT_TRUE(link.wait_for_state(1.0));
  const auto p = plan_now(link);
  ASSERT_EQ(link.configure(2.0, p.mask), 0xFF);
  ASSERT_EQ(link.configure(2.0), 0xFF);
  ASSERT_TRUE(link.wait_for_state(1.0));
  ASSERT_TRUE(link.wait_for_state(1.0));
  EXPECT_EQ(plan_now(link).mask, 1u << 7);
}
