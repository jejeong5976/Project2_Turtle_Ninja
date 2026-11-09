// test_opencr.cpp — 로봇 없이 OpenCR 쪽 시험
//   ① 편지 규칙(nt_packet.h): 펌웨어와 플러그인이 같은 파일을 쓰니까, 여기서 통과하면 양쪽 규칙이 같다는 뜻
//   ② OpenCRLink: 가짜 OpenCR(pty) 를 붙여 악수 → 명령 → 상태, 단위 변환, 끊김

#include <gtest/gtest.h>

#include <atomic>
#include <cmath>
#include <thread>
#include <vector>
#include <fcntl.h>
#include <pty.h>
#include <termios.h>
#include <unistd.h>

#include "hardware/opencr_link.hpp"
#include "nt_packet.h"

// ============================================================================ ① 편지 규칙
// 펌웨어와 플러그인이 같은 파일을 쓰니까, 여기서 통과하면 양쪽 규칙이 같다는 뜻.

namespace
{
// 상자 한 개를 Parser 에 바이트 단위로 넣고, 완성된 상자 수를 셈
int feed_all(ntp::Parser & p, const uint8_t * b, std::size_t n)
{
  int done = 0;
  for (std::size_t i = 0; i < n; ++i) {
    if (p.feed(b[i])) {++done;}
  }
  return done;
}
}  // namespace

// 1) CONFIG: 모터 14개 설정이 그대로 오가나 (켤 때 한 번)
TEST(Packet, ConfigRoundTrip)
{
  ntp::Config c{};
  c.n = 14;
  c.bus_watchdog = 15;
  for (uint8_t i = 0; i < c.n; ++i) {
    c.j[i] = {static_cast<uint8_t>(i + 1), ntp::MODE_POSITION, static_cast<uint16_t>(40 + i), 10, 200};
  }
  uint8_t buf[ntp::MAX_FRAME];
  const uint16_t len = ntp::encode_config(c, buf, sizeof(buf));
  ASSERT_EQ(len, 5 + 2 + 8 * 14);
  EXPECT_EQ(buf[0], 0xAA);
  EXPECT_EQ(buf[1], 0x55);
  EXPECT_EQ(buf[2], ntp::TYPE_CONFIG);

  ntp::Parser p;
  ASSERT_EQ(feed_all(p, buf, len), 1);
  ntp::Config d{};
  ASSERT_TRUE(ntp::decode_config(p.data(), p.len(), d));
  EXPECT_EQ(d.n, 14);
  EXPECT_EQ(d.bus_watchdog, 15);
  EXPECT_EQ(d.j[13].id, 14);
  EXPECT_EQ(d.j[13].profile_vel, 53);
  EXPECT_EQ(d.j[5].goal_current, 200);
}

// 2) CMD: 목표 float 와 "유효" 비트가 그대로 오가나 (100 Hz)
TEST(Packet, CmdRoundTrip)
{
  ntp::Cmd c{};
  c.seq = 200;
  c.flags = ntp::FLAG_TORQUE;
  c.valid = 0x3FFE;   // 0 번만 목표 없음
  c.n = 14;
  for (uint8_t i = 0; i < c.n; ++i) {c.target[i] = -1.5f + 0.25f * i;}
  uint8_t buf[ntp::MAX_FRAME];
  const uint16_t len = ntp::encode_cmd(c, buf, sizeof(buf));
  ASSERT_EQ(len, 5 + 5 + 4 * 14);   // 상자 66 바이트 (모터 14개)

  ntp::Parser p;
  ASSERT_EQ(feed_all(p, buf, len), 1);
  ASSERT_EQ(p.type(), ntp::TYPE_CMD);
  ntp::Cmd d{};
  ASSERT_TRUE(ntp::decode_cmd(p.data(), p.len(), d));
  EXPECT_EQ(d.seq, 200);
  EXPECT_EQ(d.flags, ntp::FLAG_TORQUE);
  EXPECT_EQ(d.valid, 0x3FFE);
  for (uint8_t i = 0; i < c.n; ++i) {EXPECT_FLOAT_EQ(d.target[i], c.target[i]);}
}

// 3) STATE: 최대 16 모터 + IMU 가 255 바이트 한 칸 안에 들어가고 그대로 오가나
TEST(Packet, StateRoundTripAtMaxSize)
{
  ntp::State s{};
  s.ms = 123456789u;
  s.seq_echo = 7;
  s.status = ntp::ST_TORQUE | ntp::ST_IMU_OK | ntp::ST_CONFIGURED;
  s.comm_err = 0x8001;
  s.imu.quat[0] = 1.0f;
  s.imu.gyro[2] = 0.25f;
  s.imu.acc[2] = 9.81f;
  s.n = ntp::MAX_JOINTS;
  for (uint8_t i = 0; i < s.n; ++i) {
    s.pos[i] = 0.1f * i;
    s.vel[i] = -0.2f * i;
    s.cur[i] = 0.01f * i;
  }
  uint8_t buf[ntp::MAX_FRAME];
  const uint16_t len = ntp::encode_state(s, buf, sizeof(buf));
  ASSERT_EQ(len, 5 + 49 + 12 * 16);   // 데이터 241 ≤ 255

  ntp::Parser p;
  ASSERT_EQ(feed_all(p, buf, len), 1);
  ntp::State d{};
  ASSERT_TRUE(ntp::decode_state(p.data(), p.len(), d));
  EXPECT_EQ(d.ms, 123456789u);
  EXPECT_EQ(d.status, s.status);
  EXPECT_EQ(d.comm_err, 0x8001);
  EXPECT_FLOAT_EQ(d.imu.gyro[2], 0.25f);
  EXPECT_FLOAT_EQ(d.imu.acc[2], 9.81f);
  EXPECT_FLOAT_EQ(d.pos[15], 1.5f);
  EXPECT_FLOAT_EQ(d.vel[15], -3.0f);
}

// 4) 체크섬이 틀린 상자는 버리고 개수를 셈 (선 잡음)
TEST(Packet, BadChecksumIsDropped)
{
  ntp::ConfigAck a{14, 0x3FFF};
  uint8_t buf[ntp::MAX_FRAME];
  const uint16_t len = ntp::encode_ack(a, buf, sizeof(buf));
  buf[5] ^= 0x01;   // 데이터 한 비트 뒤집기
  ntp::Parser p;
  EXPECT_EQ(feed_all(p, buf, len), 0);
  EXPECT_EQ(p.bad_checksum_count(), 1u);
}

// 5) 앞에 쓰레기 · 잘린 상자가 있어도 다음 헤더부터 다시 찾음 + 나눠서 도착해도 OK
//    ("AA 55 를 찾아 잘라 내기")
TEST(Packet, ResyncAfterGarbageAndSplitDelivery)
{
  ntp::ConfigAck a{14, 0x1234};
  uint8_t good[16];
  const uint16_t len = ntp::encode_ack(a, good, sizeof(good));

  std::vector<uint8_t> stream = {0x00, 0xAA, 0x13, 0xAA, 0xAA};   // 쓰레기 + 헤더 같아 보이는 바이트
  stream.insert(stream.end(), good, good + len);
  stream.insert(stream.end(), good, good + len);                  // 같은 상자 하나 더

  ntp::Parser p;
  int done = 0;
  ntp::ConfigAck d{};
  for (uint8_t b : stream) {   // 한 바이트씩 = 가장 잘게 나눠 도착
    if (p.feed(b)) {
      ++done;
      ASSERT_TRUE(ntp::decode_ack(p.data(), p.len(), d));
    }
  }
  EXPECT_EQ(done, 2);
  EXPECT_EQ(d.ok_mask, 0x1234);
}

// 6) 데이터 안에 AA 55 가 들어 있어도 상자가 안 깨짐 (길이 칸 덕분)
TEST(Packet, HeaderBytesInsidePayload)
{
  ntp::Cmd c{};
  c.n = 1;
  c.valid = 1;
  const uint8_t raw[4] = {0xAA, 0x55, 0xAA, 0x55};
  memcpy(&c.target[0], raw, 4);
  uint8_t buf[ntp::MAX_FRAME];
  const uint16_t len = ntp::encode_cmd(c, buf, sizeof(buf));
  ntp::Parser p;
  ASSERT_EQ(feed_all(p, buf, len), 1);
  ntp::Cmd d{};
  ASSERT_TRUE(ntp::decode_cmd(p.data(), p.len(), d));
  EXPECT_EQ(memcmp(&d.target[0], raw, 4), 0);
}

// 7) 모양이 안 맞는 상자는 개봉 거부 (n 이 너무 큼 / 길이가 n 과 안 맞음)
TEST(Packet, DiagRoundTrip)
{
  ntp::Diag g{};
  g.ms = 123456;
  g.window_ms = 1002;
  g.loop = {100, 3, 4123, 11000};
  g.resync = 2;
  g.sync_read = {99, 1, 3001, 3555};
  g.sync_write = {98, 0, 120, 300};
  g.single_read = {198, 4, 310, 3200};
  g.single_write = {13, 0, 290, 400};
  g.fault_stop_us = 4200;
  g.fault_flags = 3;
  g.volt_min_dv = 104;
  g.volt_mean_dv = 117;
  g.usb_tx = {100, 7, 3, 12};
  g.config_ms = 812;
  uint8_t buf[ntp::MAX_FRAME];
  const uint16_t len = ntp::encode_diag(g, buf, sizeof(buf));
  ASSERT_EQ(len, ntp::DIAG_LEN + 5);
  ntp::Parser p;
  bool got = false;
  for (uint16_t i = 0; i < len; ++i) {got = p.feed(buf[i]);}
  ASSERT_TRUE(got);
  ASSERT_EQ(p.type(), ntp::TYPE_DIAG);
  ntp::Diag d{};
  ASSERT_TRUE(ntp::decode_diag(p.data(), p.len(), d));
  EXPECT_EQ(d.ms, 123456u);
  EXPECT_EQ(d.loop.fail, 3);
  EXPECT_EQ(d.resync, 2);
  EXPECT_EQ(d.sync_read.max_us, 3555);
  EXPECT_EQ(d.single_write.mean_us, 290);
  EXPECT_EQ(d.fault_stop_us, 4200);
  EXPECT_EQ(d.usb_tx.fail, 7);
  EXPECT_EQ(d.usb_tx.max_us, 12);
  EXPECT_EQ(d.config_ms, 812);
  EXPECT_EQ(d.fault_flags, 3);
  EXPECT_EQ(d.volt_min_dv, 104);
  EXPECT_EQ(d.volt_mean_dv, 117);
  EXPECT_FALSE(ntp::decode_diag(p.data(), static_cast<uint8_t>(p.len() - 1), d));
}

// TEMP: 모터 온도 [°C] 가 그대로 오가나 (1 초마다). 못 읽은 모터 = TEMP_UNKNOWN. 모양이 틀리면 거부
TEST(Packet, TempRoundTrip)
{
  ntp::Temp t{};
  t.n = ntp::MAX_JOINTS;
  for (uint8_t i = 0; i < t.n; ++i) {t.c[i] = static_cast<uint8_t>(30 + i);}
  t.c[3] = ntp::TEMP_UNKNOWN;
  uint8_t buf[ntp::MAX_FRAME];
  const uint16_t len = ntp::encode_temp(t, buf, sizeof(buf));
  ASSERT_EQ(len, 5 + 1 + ntp::MAX_JOINTS);
  ntp::Parser p;
  ASSERT_EQ(feed_all(p, buf, len), 1);
  ASSERT_EQ(p.type(), ntp::TYPE_TEMP);
  ntp::Temp d{};
  ASSERT_TRUE(ntp::decode_temp(p.data(), p.len(), d));
  EXPECT_EQ(d.n, ntp::MAX_JOINTS);
  EXPECT_EQ(d.c[0], 30);
  EXPECT_EQ(d.c[3], ntp::TEMP_UNKNOWN);
  EXPECT_EQ(d.c[15], 45);
  EXPECT_FALSE(ntp::decode_temp(p.data(), static_cast<uint8_t>(p.len() - 1), d));   // 길이가 n 과 안 맞음
  t.n = ntp::MAX_JOINTS + 1;
  EXPECT_EQ(ntp::encode_temp(t, buf, sizeof(buf)), 0);
  const uint8_t too_many[1] = {17};
  EXPECT_FALSE(ntp::decode_temp(too_many, 1, d));
}

// TORQUE_FAIL: 토크 켜기 실패 이유가 그대로 오가나. 길이 · 단계가 틀리면 거부
TEST(Packet, TorqueFailRoundTrip)
{
  ntp::TorqueFail f{};
  f.idx = 9;
  f.id = 12;
  f.step = ntp::TF_RANGE;
  f.dxl_err = 0;
  f.lib_err = 0;
  f.pos = 3120;
  f.min_lim = 1024;
  f.max_lim = 3072;
  uint8_t buf[ntp::MAX_FRAME];
  const uint16_t len = ntp::encode_torque_fail(f, buf, sizeof(buf));
  ASSERT_EQ(len, 5 + 17);
  ntp::Parser p;
  ASSERT_EQ(feed_all(p, buf, len), 1);
  ASSERT_EQ(p.type(), ntp::TYPE_TORQUE_FAIL);
  ntp::TorqueFail d{};
  ASSERT_TRUE(ntp::decode_torque_fail(p.data(), p.len(), d));
  EXPECT_EQ(d.idx, 9);
  EXPECT_EQ(d.id, 12);
  EXPECT_EQ(d.step, ntp::TF_RANGE);
  EXPECT_EQ(d.pos, 3120);
  EXPECT_EQ(d.min_lim, 1024);
  EXPECT_EQ(d.max_lim, 3072);
  EXPECT_FALSE(ntp::decode_torque_fail(p.data(), static_cast<uint8_t>(p.len() - 1), d));
  f.step = 9;
  ASSERT_EQ(ntp::encode_torque_fail(f, buf, sizeof(buf)), 5 + 17);
  ASSERT_EQ(feed_all(p, buf, 5 + 17), 1);
  EXPECT_FALSE(ntp::decode_torque_fail(p.data(), p.len(), d)) << "모르는 단계";
}

TEST(Packet, DecodeRejectsBadShape)
{
  ntp::Cmd c{};
  c.n = 17;   // MAX_JOINTS 초과
  uint8_t buf[ntp::MAX_FRAME];
  EXPECT_EQ(ntp::encode_cmd(c, buf, sizeof(buf)), 0);

  uint8_t d[5 + 4 * 2] = {0};
  d[4] = 3;   // n = 3 이라는데 목표는 2개 분량
  ntp::Cmd out{};
  EXPECT_FALSE(ntp::decode_cmd(d, sizeof(d), out));
  ntp::State s{};
  EXPECT_FALSE(ntp::decode_state(d, sizeof(d), s));   // 상태 상자는 최소 49 바이트
}

// ============================================================================ ② OpenCRLink
// =====================================================================
// 리눅스 "가짜 터미널(pty)" = 양 끝이 있는 가상 시리얼 선.
//   한쪽 끝 → OpenCRLink (진짜 /dev/ttyACM0 대신)
//   다른 끝 → FakeOpenCR (펌웨어 nt_opencr.ino 가 하는 대답을 흉내: CONFIG → ACK, CMD → STATE)
// 로봇 · 보드 없이 "켤 때 악수 → 명령 → 상태" 한 바퀴와 단위 변환, 끊김 감지를 확인한다.

using hardware::DiagSummary;
using hardware::JointSpec;
using hardware::OpenCRLink;

namespace
{

// 펌웨어 흉내. 위치 모터는 목표에 바로 도착했다고, 바퀴는 목표 속도로 돈다고 대답
class FakeOpenCR
{
public:
  explicit FakeOpenCR(int fd, uint16_t answer_mask = 0xFFFF, bool silent = false)
  : fd_(fd), answer_mask_(answer_mask), silent_(silent)
  {
    th_ = std::thread([this] {run();});
  }
  ~FakeOpenCR() {stop_ = true; th_.join();}

  std::atomic<int> configs{0};
  std::atomic<int> cmds{0};
  std::atomic<uint16_t> last_valid{0};
  std::atomic<uint8_t> last_flags{0};
  std::atomic<float> last_target_15{0.0f};   // 마지막 모터(그리퍼 자리) 목표 [모터 rad]
  std::atomic<bool> stop_sending{false};
  std::atomic<uint16_t> comm_err{0};
  std::atomic<bool> send_diag{false};   // 시험용: STATE 마다 DIAG 도 하나 (실제 펌웨어는 1 초마다)   // 시험용: 이 비트의 모터를 "못 읽음" 으로 (값은 옛날 그대로 = 펌웨어와 같게)
  std::atomic<bool> send_temp{false};   // 시험용: STATE 마다 TEMP 도 하나 (모터 i = 40 + i °C, 마지막 모터는 못 읽음)
  std::atomic<bool> temp_bad_n{false};  // 시험용: TEMP 의 모터 수를 CONFIG 와 다르게
  std::atomic<float> idle_pos_last{0.0f};   // 시험용: 마지막 모터에 목표가 없을 때 보낼 위치 [모터 rad] (기본 0 = 예전과 같음)

private:
  void run()
  {
    ntp::Parser p;
    uint8_t in[256];
    uint8_t out[ntp::MAX_FRAME];
    while (!stop_) {
      const ssize_t n = ::read(fd_, in, sizeof(in));
      for (ssize_t k = 0; k < n; ++k) {
        if (!p.feed(in[k]) || silent_) {continue;}
        if (p.type() == ntp::TYPE_CONFIG) {
          ntp::Config c;
          ASSERT_TRUE(ntp::decode_config(p.data(), p.len(), c));
          cfg_ = c;
          ++configs;
          ntp::ConfigAck a{c.n, static_cast<uint16_t>(answer_mask_ & ((1u << c.n) - 1))};
          write_all(out, ntp::encode_ack(a, out, sizeof(out)));
        } else if (p.type() == ntp::TYPE_CMD) {
          ntp::Cmd c;
          ASSERT_TRUE(ntp::decode_cmd(p.data(), p.len(), c));
          ++cmds;
          last_valid = c.valid;
          last_flags = c.flags;
          if (c.n > 0) {last_target_15 = c.target[c.n - 1];}
          if (stop_sending) {continue;}
          ntp::State s{};
          s.ms = 1000;
          s.seq_echo = c.seq;
          s.status = ntp::ST_CONFIGURED | ntp::ST_IMU_OK;
          s.comm_err = comm_err;
          s.imu.gyro[2] = 0.5f;
          s.n = cfg_.n;
          for (uint8_t i = 0; i < s.n; ++i) {
            const bool valid = c.valid & (1u << i);
            if (cfg_.j[i].mode == ntp::MODE_VELOCITY) {
              s.vel[i] = valid ? c.target[i] : 0.0f;
            } else if (valid) {
              s.pos[i] = c.target[i];
            } else if (i + 1 == s.n) {
              s.pos[i] = idle_pos_last;
            }
            s.cur[i] = 0.1f;
          }
          write_all(out, ntp::encode_state(s, out, sizeof(out)));
          if (send_diag) {
            ntp::Diag g{};
            g.window_ms = 1000;
            g.loop = {100, 1, 4000, 12000};
            g.sync_read = {100, 0, 3000, 3500};
            g.single_read = {200, 2, 300, 3100};
            g.fault_stop_us = 0;
            write_all(out, ntp::encode_diag(g, out, sizeof(out)));
          }
          if (send_temp) {
            ntp::Temp t{};
            t.n = static_cast<uint8_t>(cfg_.n + (temp_bad_n ? 1 : 0));
            for (uint8_t i = 0; i < t.n; ++i) {t.c[i] = static_cast<uint8_t>(40 + i);}
            if (cfg_.n > 0) {t.c[cfg_.n - 1] = ntp::TEMP_UNKNOWN;}
            write_all(out, ntp::encode_temp(t, out, sizeof(out)));
          }
        }
      }
      std::this_thread::sleep_for(std::chrono::microseconds(500));
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
  uint16_t answer_mask_;
  bool silent_;
  ntp::Config cfg_{};
  std::atomic<bool> stop_{false};
  std::thread th_;
};

// 가상 시리얼 선 한 개 (양 끝 raw, 기다리지 않음)
struct Pty
{
  int master = -1, slave = -1;
  Pty()
  {
    EXPECT_EQ(openpty(&master, &slave, nullptr, nullptr, nullptr), 0);
    termios t{};
    tcgetattr(slave, &t);
    cfmakeraw(&t);
    tcsetattr(slave, TCSANOW, &t);
    tcgetattr(master, &t);
    cfmakeraw(&t);
    tcsetattr(master, TCSANOW, &t);
    fcntl(master, F_SETFL, O_NONBLOCK);
    fcntl(slave, F_SETFL, O_NONBLOCK);
  }
  ~Pty() {if (master >= 0) {::close(master);}}   // slave 는 OpenCRLink 가 닫음
};

// 스워브 + 팔 축소판: 바퀴 1, 조향 1, 그리퍼 1
std::vector<JointSpec> small_robot()
{
  return {
    {"drive_fl_joint", 1, ntp::MODE_VELOCITY, 0, 20, 0, 1.0},
    {"steer_fl_joint", 5, ntp::MODE_POSITION, 40, 10, 0, 1.0},
    {"gripper_left_joint", 15, ntp::MODE_CURRENT_POSITION, 0, 0, 200, 0.01149},
  };
}

}  // namespace

// 1) 켤 때 악수: CONFIG 를 보내면 ACK 의 ok_mask 가 돌아옴 + 대답 안 한 모터 이름을 알려 줌
TEST(OpenCRLink, ConfigureHandshakeReportsMissingMotor)
{
  Pty pty;
  FakeOpenCR fw(pty.master, 0b101);   // steer(ID 5) 가 ping 에 대답 안 함
  OpenCRLink link(small_robot(), 15);
  link.adopt_fd(pty.slave);
  const int mask = link.configure(2.0);
  EXPECT_EQ(mask, 0b101);
  EXPECT_EQ(link.missing_joints(mask), "steer_fl_joint(ID 5)");
  EXPECT_EQ(fw.configs.load(), 1);
}

// 2) 보드가 대답을 안 하면 -1 (포트 이름이 틀렸거나 펌웨어가 안 올라감)
TEST(OpenCRLink, ConfigureTimesOutWhenSilent)
{
  Pty pty;
  FakeOpenCR fw(pty.master, 0xFFFF, true);
  OpenCRLink link(small_robot(), 15);
  link.adopt_fd(pty.slave);
  EXPECT_EQ(link.configure(0.3), -1);
}

// 3) 명령 → 상태 한 바퀴: NaN 은 "목표 없음" 비트로, 그리퍼 m 는 모터 rad 로 바뀌어 감
//    돌아온 상태는 다시 관절 단위(m)로 바뀌어 옴
TEST(OpenCRLink, CommandAndStateRoundTripWithUnits)
{
  Pty pty;
  FakeOpenCR fw(pty.master);
  OpenCRLink link(small_robot(), 15);
  link.adopt_fd(pty.slave);
  ASSERT_EQ(link.configure(2.0), 0b111);
  // 실물 순서: 첫 STATE 로 그리퍼 (여러 바퀴 모드) 바퀴 보정을 정한 뒤에야 목표가 나감 (11-09)
  ASSERT_TRUE(link.send_command({NAN, NAN, NAN}, false));
  ASSERT_TRUE(link.wait_for_state(1.0));

  // 바퀴 2 rad/s, 조향은 목표 없음(NaN), 그리퍼 0.010 m 벌리기
  ASSERT_TRUE(link.send_command({2.0, NAN, 0.010}, true));
  ASSERT_TRUE(link.wait_for_state(1.0));

  EXPECT_EQ(fw.last_valid.load(), 0b101);              // 조향 비트만 0
  EXPECT_EQ(fw.last_flags.load(), ntp::FLAG_TORQUE);
  EXPECT_NEAR(fw.last_target_15.load(), 0.010 / 0.01149, 1e-4);   // 0.87 rad (모터 축)

  const auto & s = link.state();
  ASSERT_TRUE(s.joints_valid);
  EXPECT_NEAR(s.joints[0].velocity, 2.0, 1e-5);
  EXPECT_NEAR(s.joints[2].position, 0.010, 1e-6);   // 다시 m 로
  EXPECT_NEAR(s.joints[2].current, 0.1, 1e-6);      // 전류는 바꾸지 않음
  EXPECT_NEAR(s.imu.gyro[2], 0.5f, 1e-6);
  EXPECT_TRUE(s.status & ntp::ST_IMU_OK);
  EXPECT_LT(link.seconds_since_state(), 0.5);
  EXPECT_EQ(link.bad_checksums(), 0u);
}

// 4) 보드가 상태를 안 보내기 시작하면 seconds_since_state() 가 커짐
//    → 플러그인이 0.5 s 넘으면 ERROR 로 멈춤 (USB 뽑힘 · 펌웨어 멈춤)
TEST(OpenCRLink, DetectsStaleState)
{
  Pty pty;
  FakeOpenCR fw(pty.master);
  OpenCRLink link(small_robot(), 15);
  link.adopt_fd(pty.slave);
  ASSERT_EQ(link.configure(2.0), 0b111);
  ASSERT_TRUE(link.send_command({0.0, 0.0, 0.0}, true));
  ASSERT_TRUE(link.wait_for_state(1.0));
  fw.stop_sending = true;
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  ASSERT_TRUE(link.send_command({0.0, 0.0, 0.0}, true));
  EXPECT_FALSE(link.wait_for_state(0.1));
  EXPECT_GT(link.seconds_since_state(), 0.2);
}

// 5) (R3) 못 읽은 모터 · 끊긴 상태는 "지금의 측정" 이 아님 → 플러그인이 NaN 으로 내보냄
//    (예전: comm_err 유예 0.2 s 동안 옛 값을 그대로 내보내 컨트롤러가 새 시각의 odom 으로 포장했음 — Codex 재현)
TEST(OpenCRLink, JointMeasuredFollowsCommErrAndAge)
{
  Pty pty;
  FakeOpenCR fw(pty.master);
  OpenCRLink link(small_robot(), 15);
  link.adopt_fd(pty.slave);
  ASSERT_EQ(link.configure(2.0), 0b111);
  ASSERT_TRUE(link.send_command({0.0, 0.0, 0.0}, true));
  ASSERT_TRUE(link.wait_for_state(1.0));
  for (std::size_t i = 0; i < 3; ++i) {EXPECT_TRUE(link.joint_measured(i, 0.1));}
  EXPECT_FALSE(link.joint_measured(3, 0.1));   // 없는 관절

  fw.comm_err = 0b010;   // 조향 모터(ID 5) 응답 없음
  ASSERT_TRUE(link.send_command({0.0, 0.0, 0.0}, true));
  ASSERT_TRUE(link.wait_for_state(1.0));
  EXPECT_TRUE(link.joint_measured(0, 0.1));
  EXPECT_FALSE(link.joint_measured(1, 0.1));
  EXPECT_TRUE(link.joint_measured(2, 0.1));

  fw.comm_err = 0;
  ASSERT_TRUE(link.send_command({0.0, 0.0, 0.0}, true));
  ASSERT_TRUE(link.wait_for_state(1.0));
  EXPECT_TRUE(link.joint_measured(1, 0.1));   // 다시 읽히면 바로 측정으로

  fw.stop_sending = true;   // STATE 가 끊김 → 시간이 지나면 모두 측정 아님
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  link.poll();
  for (std::size_t i = 0; i < 3; ++i) {EXPECT_FALSE(link.joint_measured(i, 0.1));}
}

// DIAG(보드 안 시간)를 받아 모으고, 꺼내면 0 부터 다시. STATE 처리에는 영향 없음
TEST(OpenCRLink, DiagIsCollectedAndDoesNotDisturbState)
{
  Pty pty;
  FakeOpenCR fake(pty.master);
  OpenCRLink link(small_robot(), 15);
  link.adopt_fd(pty.slave);
  ASSERT_GE(link.configure(2.0), 0);
  fake.send_diag = true;
  const std::vector<double> none(3, std::nan(""));
  int states = 0;
  for (int k = 0; k < 50 && states < 5; ++k) {
    link.send_command(none, false);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    states += std::max(0, link.poll());
  }
  ASSERT_GE(states, 5);
  const DiagSummary d = link.take_diag();
  ASSERT_GE(d.packets, 5u);
  EXPECT_EQ(d.loop.n, 100u * d.packets);
  EXPECT_EQ(d.loop.fail, d.packets);
  EXPECT_NEAR(d.loop.mean_us(), 4000.0, 1e-9);
  EXPECT_EQ(d.loop.max_us, 12000u);
  EXPECT_EQ(d.single_read.fail, 2u * d.packets);
  EXPECT_EQ(link.take_diag().packets, 0u) << "꺼낸 뒤에는 0 부터";
  EXPECT_TRUE(link.state().joints_valid);
}

// TEMP(모터 온도)를 받아 관절 순서 °C 로 둠 (못 읽음 = -1). 모터 수가 틀린 TEMP 는 버림, CONFIG 하면 다시 모름
TEST(OpenCRLink, TempIsReceivedAndResetByConfig)
{
  Pty pty;
  FakeOpenCR fake(pty.master);
  OpenCRLink link(small_robot(), 15);
  link.adopt_fd(pty.slave);
  ASSERT_GE(link.configure(2.0), 0);
  EXPECT_EQ(link.temp_count(), 0u);
  EXPECT_EQ(link.temperatures(), (std::vector<int>{-1, -1, -1}));
  const std::vector<double> none(3, std::nan(""));
  auto exchange = [&]() {
      for (int k = 0; k < 20; ++k) {
        link.send_command(none, false);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        link.poll();
      }
    };
  fake.temp_bad_n = true;
  fake.send_temp = true;
  exchange();
  EXPECT_EQ(link.temp_count(), 0u) << "모터 수가 다른 TEMP 는 버림";
  fake.temp_bad_n = false;
  exchange();
  ASSERT_GT(link.temp_count(), 0u);
  EXPECT_EQ(link.temperatures(), (std::vector<int>{40, 41, -1}));
  EXPECT_TRUE(link.state().joints_valid) << "TEMP 가 STATE 처리를 방해하지 않음";
  fake.send_temp = false;
  ASSERT_GE(link.configure(2.0), 0);
  EXPECT_EQ(link.temperatures(), (std::vector<int>{-1, -1, -1})) << "CONFIG 전 온도를 새 설정 값처럼 안 보임";
}

// 온도 판정 · /diagnostics 내용 — ★ robot_cli (model.cpp parse_temp_status) 가 이 모양을 읽음
TEST(Temp, CheckAndReport)
{
  const auto j = small_robot();
  EXPECT_EQ(hardware::check_temps({-1, -1, -1}, 60, 70).level, 3);
  const auto ok = hardware::temp_report(j, {41, 52, -1}, 60, 70);
  EXPECT_EQ(ok.level, 0);
  EXPECT_EQ(ok.message, "최고 steer_fl_joint 52 °C · 못 읽음 1");
  ASSERT_EQ(ok.values.size(), 5u);
  EXPECT_EQ(ok.values[0], (std::pair<std::string, std::string>{"drive_fl_joint", "41"}));
  EXPECT_EQ(ok.values[2], (std::pair<std::string, std::string>{"gripper_left_joint", "-"}));
  EXPECT_EQ(ok.values[3], (std::pair<std::string, std::string>{"warn_c", "60"}));
  EXPECT_EQ(ok.values[4], (std::pair<std::string, std::string>{"crit_c", "70"}));
  const auto warn = hardware::temp_report(j, {60, 59, 61}, 60, 70);
  EXPECT_EQ(warn.level, 1) << "경고 기준 그 값부터 경고";
  EXPECT_EQ(warn.message, "경고 drive_fl_joint 60 °C · gripper_left_joint 61 °C (경고 60 · 위험 70 °C)");
  const auto crit = hardware::temp_report(j, {45, 70, 63}, 60, 70);
  EXPECT_EQ(crit.level, 2);
  EXPECT_EQ(crit.message, "위험 steer_fl_joint 70 °C · gripper_left_joint 63 °C (경고 60 · 위험 70 °C)");
  const auto none = hardware::temp_report(j, {}, 60, 70);
  EXPECT_EQ(none.level, 3);
  EXPECT_EQ(none.message, "온도 못 읽음");
  EXPECT_EQ(none.values[1].second, "-");
  EXPECT_STREQ(hardware::TEMP_DIAG_SUFFIX, "/motor_temperature");
}

// 첫 판 DIAG(53 B)도 읽음 — 새 칸은 0. 길이가 둘 다 아니면 거부
TEST(Packet, DiagV1StillDecodes)
{
  ntp::Diag g{};
  g.loop = {100, 4, 10953, 65535};
  g.usb_tx = {9, 9, 9, 9};
  g.config_ms = 9;
  uint8_t buf[ntp::MAX_FRAME];
  ASSERT_GT(ntp::encode_diag(g, buf, sizeof(buf)), 0);
  ntp::Diag d{};
  d.usb_tx = {1, 1, 1, 1};
  ASSERT_TRUE(ntp::decode_diag(buf + 4, ntp::DIAG_LEN_V1, d));   // 앞 53 B 만 = 첫 판 모양
  EXPECT_EQ(d.loop.max_us, 65535);
  EXPECT_EQ(d.usb_tx.n, 0);
  EXPECT_EQ(d.config_ms, 0);
  EXPECT_FALSE(ntp::decode_diag(buf + 4, ntp::DIAG_LEN_V1 + 1, d));
}

// CONFIG 전 구간은 측정 합계에 안 섞고 따로 셈 (숨기지 않음)
TEST(OpenCRLink, DiagBeforeConfigKeptSeparate)
{
  hardware::DiagSummary s;
  ntp::Diag before{};
  before.fault_flags = ntp::DIAG_FLAG_BEFORE_CONFIG;
  before.loop = {50, 5, 10953, 65535};
  before.usb_tx = {54, 54, 0, 1};
  ntp::Diag after{};
  after.loop = {100, 0, 5, 15};
  after.usb_tx = {101, 0, 3, 10};
  after.config_ms = 40;
  s.add(before);
  s.add(after);
  EXPECT_EQ(s.packets, 1u);
  EXPECT_EQ(s.loop.fail, 0u);
  EXPECT_EQ(s.loop.max_us, 15u);
  EXPECT_EQ(s.usb_tx.n, 101u);
  EXPECT_EQ(s.config_ms, 40);
  EXPECT_EQ(s.before_config_packets, 1u);
  EXPECT_EQ(s.before_loop.fail, 5u);
  EXPECT_EQ(s.before_loop.max_us, 65535u);
  EXPECT_EQ(s.before_usb_tx.fail, 54u);
}

// 조향 영점 · 방향: 모터 + = 위에서 시계, URDF 조향축 +z (+ = 반시계) → scale −1, zero_tick = 직진 눈금
TEST(JointConvert, SteerZeroAndSignFollowUrdf)
{
  const double k = 2.0 * M_PI / 4096.0;
  const auto tick_to_motor = [k](int tick) {return (tick - 2048) * k;};   // 펌웨어 위치 모드와 같은 식
  const int zeros[4] = {2054, 2583, 3596, 2589};                          // 임시 영점 ID 11 · 12 · 13 · 14
  for (int z : zeros) {
    hardware::JointSpec j;
    j.scale = -1.0;
    j.zero_rad = (z - 2048) * k;
    EXPECT_NEAR(hardware::motor_to_joint(j, tick_to_motor(z)), 0.0, 1e-12);          // 직진 = 관절 0
    // 눈금이 늘면(모터 +, 위에서 시계) 관절은 − (URDF +z 기준 시계 = 음수)
    EXPECT_NEAR(hardware::motor_to_joint(j, tick_to_motor(z + 57)), -57 * k, 1e-12);
    // 관절 +5° (반시계) 명령 → 모터 눈금은 줄어듦
    const double m = hardware::joint_to_motor(j, 5.0 * M_PI / 180.0);
    EXPECT_EQ(static_cast<int>(std::lround(m / k)) + 2048, z - 57);
    // 왕복
    for (double q : {-1.0, -0.3, 0.0, 0.4, 1.2}) {
      EXPECT_NEAR(hardware::motor_to_joint(j, hardware::joint_to_motor(j, q)), q, 1e-12);
    }
  }
}

// 바퀴: 모터 + = 전진, URDF 바퀴축 +y 에서 + 회전 = 전진 → scale +1, 영점 없음 (기본값 그대로)
TEST(JointConvert, DriveDefaultIsIdentity)
{
  hardware::JointSpec j;   // scale 1, zero_rad 0
  EXPECT_DOUBLE_EQ(hardware::motor_to_joint(j, 2.5), 2.5);
  EXPECT_DOUBLE_EQ(hardware::joint_to_motor(j, -0.3), -0.3);
}


// Several STATE frames received together must each contribute a board-time sample.
TEST(OpenCRPositionRate, BatchedFramesScalingAndFailedJoint)
{
  Pty p;
  auto specs = small_robot();
  specs[1].scale = -1.0;
  specs[1].zero_rad = 0.75;
  OpenCRLink link(specs, 15);
  link.adopt_fd(p.slave);
  auto send_state = [&](uint32_t ms, uint16_t err) {
    ntp::State s{};
    s.ms = ms;
    s.n = 3;
    s.comm_err = err;
    s.pos[1] = static_cast<float>(0.75 + ms * 0.0002);
    s.vel[1] = 0.16f;  // reported is deliberately 80% of position-derived rate
    uint8_t bytes[ntp::MAX_FRAME];
    const auto len = ntp::encode_state(s, bytes, sizeof(bytes));
    ASSERT_EQ(::write(p.master, bytes, len), len);
  };
  for (uint32_t ms = 0; ms <= 60; ms += 10) {send_state(ms, 0);}
  ASSERT_TRUE(link.wait_for_state(0.2));
  EXPECT_NEAR(link.state().joints[1].position_velocity, -0.2, 1e-5);
  EXPECT_NEAR(link.state().joints[1].velocity, -0.16, 1e-6);
  // Poll with no new data must neither re-feed a duplicate timestamp nor reset.
  EXPECT_EQ(link.poll(), 0);
  EXPECT_NEAR(link.state().joints[1].position_velocity, -0.2, 1e-5);
  send_state(70, 2);
  ASSERT_TRUE(link.wait_for_state(0.2));
  EXPECT_TRUE(std::isnan(link.state().joints[1].position_velocity));
  for (uint32_t ms = 80; ms <= 100; ms += 10) {send_state(ms, 0);}
  ASSERT_TRUE(link.wait_for_state(0.2));
  EXPECT_TRUE(std::isnan(link.state().joints[1].position_velocity));
  send_state(110, 0);
  ASSERT_TRUE(link.wait_for_state(0.2));
  EXPECT_NEAR(link.state().joints[1].position_velocity, -0.2, 1e-5);
  send_state(110, 0);  // a genuinely repeated board STATE is a clock fault
  ASSERT_TRUE(link.wait_for_state(0.2));
  EXPECT_TRUE(std::isnan(link.state().joints[1].position_velocity));
  for (uint32_t ms = 120; ms <= 180; ms += 10) {send_state(ms, 0);}
  ASSERT_TRUE(link.wait_for_state(0.2));
  EXPECT_TRUE(std::isnan(link.state().joints[1].position_velocity));
  // Existing reported velocity remains available to the default controller.
  EXPECT_NEAR(link.state().joints[1].velocity, -0.16, 1e-6);
}

// 토크 켜기 실패 → 사람 말: 관절 이름 · 모터 ID · 관절 각 (영점 · 방향 반영) · 위치 한계 · 모터 오류 이름
TEST(TorqueFail, TextNamesJointRangeAndError)
{
  const double k = 2.0 * M_PI / 4096.0;
  std::vector<JointSpec> js = small_robot();
  js[1].scale = -1.0;
  js[1].zero_rad = (2054 - 2048) * k;    // 조향 영점 2054 (ID 11 임시값과 같은 모양)
  ntp::TorqueFail f{};
  f.idx = 1;
  f.id = 5;
  f.step = ntp::TF_RANGE;
  f.pos = 2054 - 1100;                   // 관절 +96.7°
  f.min_lim = 1024;
  f.max_lim = 3072;
  const std::string t = hardware::torque_fail_text(js, f);
  EXPECT_NE(t.find("steer_fl_joint (모터 5)"), std::string::npos) << t;
  EXPECT_NE(t.find("954 눈금 (관절 +97°)"), std::string::npos) << t;
  EXPECT_NE(t.find("위치 한계 1024–3072"), std::string::npos) << t;
  EXPECT_NE(t.find("손으로"), std::string::npos) << t;
  f.step = ntp::TF_GOAL;
  f.dxl_err = 0x06;
  EXPECT_NE(hardware::torque_fail_text(js, f).find("0x06 (Data Limit)"), std::string::npos);
  f.step = ntp::TF_READ;
  f.dxl_err = 0;
  f.lib_err = 3;
  EXPECT_NE(hardware::torque_fail_text(js, f).find("읽기 실패 · 통신 오류 3"), std::string::npos);
  // 켜 둔 토크가 꺼짐: pos = Hardware Error Status (0x20 과부하), 오류 칸 Alert 비트만
  f.step = ntp::TF_LOST;
  f.lib_err = 0;
  f.dxl_err = 0x80;
  f.pos = 0x20;
  const std::string lost = hardware::torque_fail_text(js, f);
  EXPECT_NE(lost.find("토크가 꺼짐"), std::string::npos) << lost;
  EXPECT_NE(lost.find("0x20 = 과부하"), std::string::npos) << lost;
  EXPECT_NE(lost.find("모터 Alert (하드웨어 오류)"), std::string::npos) << lost;
  EXPECT_EQ(lost.find("( · Alert)"), std::string::npos) << lost;
}

// 새 이유 (TF_LOST · TF_BUDGET) 도 상자로 오가고, 모르는 단계는 버림
TEST(TorqueFail, NewStepsRoundTrip)
{
  for (uint8_t st : {ntp::TF_LOST, ntp::TF_BUDGET}) {
    ntp::TorqueFail sent{};
    sent.idx = 2;
    sent.id = 13;
    sent.step = st;
    sent.pos = 0x24;
    sent.min_lim = sent.max_lim = ntp::TF_NO_LIMIT;
    uint8_t buf[ntp::MAX_FRAME];
    const uint16_t len = ntp::encode_torque_fail(sent, buf, sizeof(buf));
    ASSERT_GT(len, 0);
    ntp::Parser ps;
    ASSERT_EQ(feed_all(ps, buf, len), 1);
    ntp::TorqueFail f{};
    ASSERT_TRUE(ntp::decode_torque_fail(ps.data(), ps.len(), f));
    EXPECT_EQ(f.step, st);
    EXPECT_EQ(f.pos, 0x24);
  }
}

// 링크가 TORQUE_FAIL 을 받아 두고 (STATE 처리는 그대로), 받은 뒤 지난 시간을 알려 줌
TEST(OpenCRLink, TorqueFailIsReceived)
{
  Pty pty;
  FakeOpenCR fake(pty.master);
  OpenCRLink link(small_robot(), 15);
  link.adopt_fd(pty.slave);
  ASSERT_GE(link.configure(2.0), 0);
  ntp::TorqueFail f{};
  double age = 0.0;
  EXPECT_FALSE(link.last_torque_fail(f, age));
  ntp::TorqueFail sent{};
  sent.idx = 1;
  sent.id = 5;
  sent.step = ntp::TF_TORQUE;
  sent.lib_err = 3;
  sent.min_lim = sent.max_lim = ntp::TF_NO_LIMIT;
  uint8_t buf[ntp::MAX_FRAME];
  const uint16_t len = ntp::encode_torque_fail(sent, buf, sizeof(buf));
  ASSERT_EQ(write(pty.master, buf, len), static_cast<ssize_t>(len));
  for (int k = 0; k < 20 && link.torque_fail_count() == 0; ++k) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    link.poll();
  }
  ASSERT_TRUE(link.last_torque_fail(f, age));
  EXPECT_EQ(f.id, 5);
  EXPECT_EQ(f.step, ntp::TF_TORQUE);
  EXPECT_LT(age, 1.0);
}

// 위치 한계 한 줄 · 한계 밖 알림: 조향 영점 · 방향 반영, 위치 모드가 아닌 관절 (바퀴 · 그리퍼) 은 빼고
TEST(PosLimits, TextAndOutOfRange)
{
  const double k = 2.0 * M_PI / 4096.0;
  std::vector<JointSpec> js = small_robot();
  js[1].scale = -1.0;
  js[1].zero_rad = (2054 - 2048) * k;
  const std::vector<std::pair<int32_t, int32_t>> lim = {{ntp::TF_NO_LIMIT, ntp::TF_NO_LIMIT}, {1024, 3072}, {ntp::TF_NO_LIMIT, ntp::TF_NO_LIMIT}};
  std::string bad;
  const std::string line = hardware::pos_limits_text(js, lim, {0.0, 3.0 * M_PI / 180.0, 0.01}, bad);
  EXPECT_EQ(line, "steer_fl_joint -89°~+91° (지금 +3°)") << line;   // 영점 2054 → (1024 · 3072 − 2054) × −1 눈금
  EXPECT_TRUE(bad.empty()) << bad;
  hardware::pos_limits_text(js, lim, {0.0, 97.0 * M_PI / 180.0, 0.01}, bad);
  EXPECT_NE(bad.find("steer_fl_joint 지금 +97°"), std::string::npos) << bad;
  EXPECT_NE(bad.find("(1024–3072) 밖"), std::string::npos) << bad;
}

// POS_LIMITS 를 받아 두고, 다음 CONFIG 에서 지움 (새 설정의 한계를 다시 받을 때까지 옛 것을 안 씀)
TEST(OpenCRLink, PosLimitsReceivedAndResetByConfig)
{
  Pty pty;
  FakeOpenCR fake(pty.master);
  OpenCRLink link(small_robot(), 15);
  link.adopt_fd(pty.slave);
  ASSERT_GE(link.configure(2.0), 0);
  std::vector<std::pair<int32_t, int32_t>> got;
  EXPECT_FALSE(link.pos_limits(got));
  ntp::PosLimits l{};
  l.n = 3;
  for (int i = 0; i < 3; ++i) {l.lo[i] = l.hi[i] = ntp::TF_NO_LIMIT;}
  l.lo[1] = 1024;
  l.hi[1] = 3072;
  uint8_t buf[ntp::MAX_FRAME];
  const uint16_t len = ntp::encode_pos_limits(l, buf, sizeof(buf));
  ASSERT_EQ(write(pty.master, buf, len), static_cast<ssize_t>(len));
  for (int k = 0; k < 20 && !link.pos_limits(got); ++k) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    link.poll();
  }
  ASSERT_TRUE(link.pos_limits(got));
  EXPECT_EQ(got[1].first, 1024);
  EXPECT_EQ(got[1].second, 3072);
  EXPECT_EQ(got[0].first, ntp::TF_NO_LIMIT);
  ASSERT_GE(link.configure(2.0), 0);
  EXPECT_FALSE(link.pos_limits(got));
}


// 여러 바퀴 모드 (J1 · 그리퍼): CONFIG 때 모터가 한 바퀴 안에서 다시 세어도 관절값은 실제 자리 (11-08 실물)
TEST(JointConvert, MultiTurnUnwrapKeepsTheJointWithinHalfATurn)
{
  const double k = 2.0 * M_PI / 4096.0;
  const auto m = [k](double tick) {return (tick - 2048.0) * k;};   // 펌웨어 모터 rad
  hardware::JointSpec j1;
  j1.mode = ntp::MODE_CURRENT_POSITION;
  j1.zero_rad = (4088 - 2048) * k;   // hardware_map joint1 zero_tick 4088 = 정면
  EXPECT_TRUE(hardware::is_multi_turn(j1));
  // 11-08: 몸통 4101 (정면 왼쪽 1°) 에서 T1 을 다시 켜자 5 눈금으로 읽힘 → +1 바퀴
  j1.turn_rad = hardware::unwrap_turn_rad(m(5), j1.zero_rad);
  EXPECT_NEAR(j1.turn_rad, 2.0 * M_PI, 1e-12);
  EXPECT_NEAR(hardware::motor_to_joint(j1, m(5)), 13 * k, 1e-9);                        // 예전: −6.26 rad
  EXPECT_NEAR(hardware::joint_to_motor(j1, 0.0) / k + 2048.0, -8.0, 1e-6);            // 정면 명령 = 모터 −8 눈금 (실제 4088)
  for (double q : {-1.2, -0.3, 0.0, 0.46, 1.5}) {
    EXPECT_NEAR(hardware::motor_to_joint(j1, hardware::joint_to_motor(j1, q)), q, 1e-12);
  }
  // 이어 센 값 (CONFIG 가 다시 안 셈) −8 눈금도 같은 자리 (정면)
  EXPECT_NEAR(hardware::unwrap_turn_rad(m(-8), j1.zero_rad), 2.0 * M_PI, 1e-12);
  // 바로 읽힌 값은 그대로: HOME 4085 · 왼쪽 적재함 4388
  EXPECT_DOUBLE_EQ(hardware::unwrap_turn_rad(m(4085), j1.zero_rad), 0.0);
  EXPECT_DOUBLE_EQ(hardware::unwrap_turn_rad(m(4388), j1.zero_rad), 0.0);
  // 왼쪽 적재함에서 켜서 292 로 읽힘 → +1 바퀴 = 4388
  EXPECT_NEAR(hardware::unwrap_turn_rad(m(292), j1.zero_rad), 2.0 * M_PI, 1e-12);
  // 그리퍼 (zero 3549, m/rad 0.01149): 1000 눈금 열린 4549 를 453 으로 읽어도 열린 값
  hardware::JointSpec g;
  g.mode = ntp::MODE_CURRENT_POSITION;
  g.scale = 0.01149;
  g.zero_rad = (3549 - 2048) * k;
  g.turn_rad = hardware::unwrap_turn_rad(m(453), g.zero_rad);
  EXPECT_NEAR(hardware::motor_to_joint(g, m(453)), 1000 * k * 0.01149, 1e-9);
  EXPECT_DOUBLE_EQ(hardware::unwrap_turn_rad(m(3478), g.zero_rad), 0.0);              // 닫힘
  // 0 ~ 4095 모드 (J2 ~ J5 · 조향) 는 해당 없음
  hardware::JointSpec p;
  p.mode = ntp::MODE_POSITION;
  EXPECT_FALSE(hardware::is_multi_turn(p));
  EXPECT_DOUBLE_EQ(hardware::unwrap_turn_rad(NAN, j1.zero_rad), 0.0);
}

// 11-08 저녁: 받침대에 앉힌 J1 이 16 눈금 크게 읽혀 영점 4088 → 4104. 플러그인은 zero_tick 0~4095 만 받으므로 hardware_map 은
//   한 바퀴 뺀 8. HOME (−0.0061 rad) = 모터 4. 켤 때 경계 아래 (4090 대) 로 읽혀도 바퀴 보정으로 같은 자리, 명령도 모터 자기 셈으로
TEST(JointConvert, MultiTurnUnwrapWithJ1ZeroPastTheWrap)
{
  const double k = 2.0 * M_PI / 4096.0;
  const auto m = [k](double tick) {return (tick - 2048.0) * k;};
  hardware::JointSpec j1;
  j1.mode = ntp::MODE_CURRENT_POSITION;
  j1.zero_rad = (8 - 2048) * k;
  const double home = -0.0061;
  // HOME 에서 켬: 4 로 읽힘 → 보정 없음 → HOME. HOME 명령 = 모터 4
  j1.turn_rad = hardware::unwrap_turn_rad(m(4), j1.zero_rad);
  EXPECT_DOUBLE_EQ(j1.turn_rad, 0.0);
  EXPECT_NEAR(hardware::motor_to_joint(j1, m(4)), -4 * k, 1e-9);
  EXPECT_NEAR(hardware::motor_to_joint(j1, m(4)), home, 0.5 * k);
  EXPECT_NEAR(hardware::joint_to_motor(j1, home) / k + 2048.0, 4.0, 0.5);
  // 받침대 좌우 여유 안 (켜면 1 ~ 10) 은 그대로
  for (double t : {1.0, 10.0}) {EXPECT_DOUBLE_EQ(hardware::unwrap_turn_rad(m(t), j1.zero_rad), 0.0);}
  // 경계 아래 (몸통이 HOME 보다 오른쪽, 4093 으로 읽힘) → −1 바퀴 = 관절 −11 눈금 (HOME 오른쪽 7 눈금, 보정 없으면 +6.27 rad)
  j1.turn_rad = hardware::unwrap_turn_rad(m(4093), j1.zero_rad);
  EXPECT_NEAR(j1.turn_rad, -2.0 * M_PI, 1e-12);
  EXPECT_NEAR(hardware::motor_to_joint(j1, m(4093)), -11 * k, 1e-9);
  // 왼쪽 적재함 (예전 4388 → 새 4404 = 모터 308) 에서 켬 → 보정 없음, 관절 +300 눈금
  j1.turn_rad = hardware::unwrap_turn_rad(m(308), j1.zero_rad);
  EXPECT_NEAR(hardware::motor_to_joint(j1, m(308)), 300 * k, 1e-9);
  // J1 범위 끝 (팔 모델 tick_lo · hi = 영점 ± 2025 눈금) 도 답이 하나
  EXPECT_DOUBLE_EQ(hardware::unwrap_turn_rad(m(8 + 2025), j1.zero_rad), 0.0);
  EXPECT_NEAR(hardware::unwrap_turn_rad(m(8 - 2025 + 4096), j1.zero_rad), -2.0 * M_PI, 1e-12);
  // 8 과 4104 는 같은 영점: 켤 때 어느 값으로 읽혀도 관절값이 같음
  hardware::JointSpec j2 = j1;
  j2.zero_rad = (4104 - 2048) * k;
  for (int t = 0; t < 4096; t += 7) {
    j1.turn_rad = hardware::unwrap_turn_rad(m(t), j1.zero_rad);
    j2.turn_rad = hardware::unwrap_turn_rad(m(t), j2.zero_rad);
    EXPECT_NEAR(hardware::motor_to_joint(j1, m(t)), hardware::motor_to_joint(j2, m(t)), 1e-9) << t;
    EXPECT_NEAR(hardware::joint_to_motor(j1, 0.3), hardware::joint_to_motor(j2, 0.3), 1e-9) << t;
  }
}

TEST(OpenCRLink, MultiTurnJointIsUnwrappedAfterEachConfig)
{
  const double k = 2.0 * M_PI / 4096.0;
  const auto m = [k](double tick) {return static_cast<float>((tick - 2048.0) * k);};
  Pty pty;
  FakeOpenCR fw(pty.master);
  std::vector<JointSpec> js(2);
  js[0] = {"joint2", 12, ntp::MODE_POSITION, 0, 0, 0, 1.0};
  js[1].name = "joint1";
  js[1].id = 11;
  js[1].mode = ntp::MODE_CURRENT_POSITION;   // goal_current 0 = 펌웨어 확장 위치
  js[1].zero_rad = (4088 - 2048) * k;
  OpenCRLink link(js, 15);
  link.adopt_fd(pty.slave);

  // ① 11-08 그대로: CONFIG 뒤 첫 측정이 5 눈금
  fw.idle_pos_last = m(5);
  ASSERT_EQ(link.configure(2.0), 0b11);
  ASSERT_TRUE(link.send_command({NAN, NAN}, true));
  ASSERT_TRUE(link.wait_for_state(1.0));
  EXPECT_EQ(link.turns(1), 1);
  EXPECT_EQ(link.turns(0), 0);
  EXPECT_NEAR(link.state().joints[1].position, 13 * k, 1e-5);   // 정면 왼쪽 1° (예전 −6.26 rad)
  const std::string note = link.take_turn_note();
  EXPECT_NE(note.find("joint1"), std::string::npos);
  EXPECT_TRUE(link.take_turn_note().empty());                    // 한 번만
  // 정면 (관절 0) 명령 → 모터 −8 눈금 = 실제 4088 (한 바퀴 돌지 않음)
  ASSERT_TRUE(link.send_command({NAN, 0.0}, true));
  ASSERT_TRUE(link.wait_for_state(1.0));
  EXPECT_NEAR(fw.last_target_15.load() / k + 2048.0, -8.0, 1e-3);
  EXPECT_NEAR(link.state().joints[1].position, 0.0, 1e-5);

  // ② 다시 CONFIG, 모터가 이어 셈 (−8) → 보정을 새로 정해도 같은 자리
  fw.idle_pos_last = m(-8);
  ASSERT_EQ(link.configure(2.0), 0b11);
  ASSERT_TRUE(link.send_command({NAN, NAN}, true));
  ASSERT_TRUE(link.wait_for_state(1.0));
  EXPECT_EQ(link.turns(1), 1);
  EXPECT_NEAR(link.state().joints[1].position, 0.0, 1e-5);
  link.take_turn_note();

  // ③ 보통 (HOME 4085 를 바로 읽음) → 보정 없음 · 알림 없음
  fw.idle_pos_last = m(4085);
  ASSERT_EQ(link.configure(2.0), 0b11);
  ASSERT_TRUE(link.send_command({NAN, NAN}, true));
  ASSERT_TRUE(link.wait_for_state(1.0));
  EXPECT_EQ(link.turns(1), 0);
  EXPECT_NEAR(link.state().joints[1].position, -3 * k, 1e-5);
  EXPECT_TRUE(link.take_turn_note().empty());
}

// 통신 끊김 (T1 켠 채 팔 전원이 0.2 s 안에 나갔다 돌아옴, ERROR 안 남) 뒤엔 모터가 0 ~ 4095 로 다시 셈 → 바퀴 보정도 다시 정함 (11-08 밤 검수)
//   예전엔 CONFIG 에서만 지워 J1 이 한 바퀴 (+2π) 튀거나 그리퍼가 반 바퀴 돌려 했음
TEST(OpenCRLink, MultiTurnJointIsUnwrappedAgainAfterCommErr)
{
  const double k = 2.0 * M_PI / 4096.0;
  const auto m = [k](double tick) {return static_cast<float>((tick - 2048.0) * k);};
  Pty pty;
  FakeOpenCR fw(pty.master);
  std::vector<JointSpec> js(2);
  js[0] = {"joint2", 12, ntp::MODE_POSITION, 0, 0, 0, 1.0};
  js[1].name = "joint1";
  js[1].id = 11;
  js[1].mode = ntp::MODE_CURRENT_POSITION;
  js[1].zero_rad = (4088 - 2048) * k;
  OpenCRLink link(js, 15);
  link.adopt_fd(pty.slave);

  // ① 켤 때 모터가 이어 센 −8 (정면) → +1 바퀴 보정, 관절 0
  fw.idle_pos_last = m(-8);
  ASSERT_EQ(link.configure(2.0), 0b11);
  ASSERT_TRUE(link.send_command({NAN, NAN}, true));
  ASSERT_TRUE(link.wait_for_state(1.0));
  EXPECT_EQ(link.turns(1), 1);
  EXPECT_NEAR(link.state().joints[1].position, 0.0, 1e-5);
  link.take_turn_note();

  // ② 팔 전원이 잠깐 나감: joint1 못 읽음 (값은 옛 그대로)
  fw.comm_err = 0b10;
  ASSERT_TRUE(link.send_command({NAN, NAN}, true));
  ASSERT_TRUE(link.wait_for_state(1.0));
  EXPECT_FALSE(link.joint_measured(1, 0.1));

  // ③ 전원이 돌아와 모터가 0 ~ 4095 로 다시 셈: 같은 자리 (정면) 가 4088 → 보정을 다시 정해 관절 0 (지우지 않으면 +2π 로 튐)
  fw.idle_pos_last = m(4088);
  fw.comm_err = 0;
  ASSERT_TRUE(link.send_command({NAN, NAN}, true));
  ASSERT_TRUE(link.wait_for_state(1.0));
  EXPECT_TRUE(link.joint_measured(1, 0.1));
  EXPECT_EQ(link.turns(1), 0);
  EXPECT_NEAR(link.state().joints[1].position, 0.0, 1e-5);
  const std::string note = link.take_turn_note();
  EXPECT_NE(note.find("joint1"), std::string::npos) << note;
  EXPECT_NE(note.find("다시 센 값"), std::string::npos) << note;
  // 정면 (관절 0) 명령 → 모터 4088 (한 바퀴 돌지 않음)
  ASSERT_TRUE(link.send_command({NAN, 0.0}, true));
  ASSERT_TRUE(link.wait_for_state(1.0));
  EXPECT_NEAR(fw.last_target_15.load() / k + 2048.0, 4088.0, 1e-3);
  EXPECT_NEAR(link.state().joints[1].position, 0.0, 1e-5);

  // ④ 잠깐 못 읽었을 뿐 값이 이어지면 보정도 그대로 · 알림 없음
  fw.comm_err = 0b10;
  ASSERT_TRUE(link.send_command({NAN, NAN}, true));
  ASSERT_TRUE(link.wait_for_state(1.0));
  fw.comm_err = 0;
  ASSERT_TRUE(link.send_command({NAN, NAN}, true));
  ASSERT_TRUE(link.wait_for_state(1.0));
  EXPECT_EQ(link.turns(1), 0);
  EXPECT_NEAR(link.state().joints[1].position, 0.0, 1e-5);
  EXPECT_TRUE(link.take_turn_note().empty());
}

// 11-09 검수 1: comm_err 로 바퀴 보정을 지운 뒤 다시 정하기 전엔 그 관절 목표를 안 보냄 (valid 비트 끔 → 펌웨어가 마지막 목표에서 버팀)
//   옛 turn_rad 로 보내면 모터가 이미 다시 센 뒤라 한 바퀴 돌 수 있음. 켤 때 (첫 STATE 전) 도 같음
TEST(OpenCRLink, MultiTurnTargetIsHeldUntilTurnIsSet)
{
  const double k = 2.0 * M_PI / 4096.0;
  const auto m = [k](double tick) {return static_cast<float>((tick - 2048.0) * k);};
  Pty pty;
  FakeOpenCR fw(pty.master);
  std::vector<JointSpec> js(2);
  js[0] = {"joint2", 12, ntp::MODE_POSITION, 0, 0, 0, 1.0};
  js[1].name = "joint1";
  js[1].id = 11;
  js[1].mode = ntp::MODE_CURRENT_POSITION;
  js[1].zero_rad = (4088 - 2048) * k;
  OpenCRLink link(js, 15);
  link.adopt_fd(pty.slave);

  // ① 켤 때: 첫 STATE 전엔 joint1 목표를 안 보냄 (joint2 는 NaN) → 첫 측정 (−8) 으로 +1 바퀴 정함
  fw.idle_pos_last = m(-8);
  ASSERT_EQ(link.configure(2.0), 0b11);
  ASSERT_TRUE(link.send_command({NAN, 0.0}, true));
  ASSERT_TRUE(link.wait_for_state(1.0));
  EXPECT_EQ(fw.last_valid.load(), 0b00);
  EXPECT_EQ(link.turns(1), 1);
  link.take_turn_note();
  // 정한 뒤엔 보냄: 정면 (관절 0) = 모터 −8
  ASSERT_TRUE(link.send_command({NAN, 0.0}, true));
  ASSERT_TRUE(link.wait_for_state(1.0));
  EXPECT_EQ(fw.last_valid.load(), 0b10);
  EXPECT_NEAR(fw.last_target_15.load() / k + 2048.0, -8.0, 1e-3);

  // ② joint1 못 읽음 → 보정 지움 → 다음 명령부터 joint1 valid 비트 끔
  fw.comm_err = 0b10;
  ASSERT_TRUE(link.send_command({NAN, 0.0}, true));
  ASSERT_TRUE(link.wait_for_state(1.0));
  EXPECT_FALSE(link.joint_measured(1, 0.1));
  fw.idle_pos_last = m(4088);   // 전원이 돌아오면 모터가 0 ~ 4095 로 다시 셈 (정면 = 4088)
  ASSERT_TRUE(link.send_command({NAN, 0.0}, true));
  ASSERT_TRUE(link.wait_for_state(1.0));
  EXPECT_EQ(fw.last_valid.load(), 0b00) << "다시 정하기 전엔 옛 turn_rad 로 보내지 않음";

  // ③ 다시 읽힘: 이 명령은 아직 안 보냄 (보낼 때는 못 정한 상태) → 그 STATE 로 다시 정함 → 그다음부터 보냄 (모터 4088, 한 바퀴 돌지 않음)
  fw.comm_err = 0;
  ASSERT_TRUE(link.send_command({NAN, 0.0}, true));
  ASSERT_TRUE(link.wait_for_state(1.0));
  EXPECT_EQ(fw.last_valid.load(), 0b00);
  EXPECT_EQ(link.turns(1), 0);
  EXPECT_NEAR(link.state().joints[1].position, 0.0, 1e-5);
  ASSERT_TRUE(link.send_command({NAN, 0.0}, true));
  ASSERT_TRUE(link.wait_for_state(1.0));
  EXPECT_EQ(fw.last_valid.load(), 0b10);
  EXPECT_NEAR(fw.last_target_15.load() / k + 2048.0, 4088.0, 1e-3);
  EXPECT_NEAR(link.state().joints[1].position, 0.0, 1e-5);
}
