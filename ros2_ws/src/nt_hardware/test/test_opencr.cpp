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

#include "nt_hardware/opencr_link.hpp"
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

using nt_hardware::DiagSummary;
using nt_hardware::JointSpec;
using nt_hardware::OpenCRLink;

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

// 5) (2026-10-16 R3) 못 읽은 모터 · 끊긴 상태는 "지금의 측정" 이 아님 → 플러그인이 NaN 으로 내보냄
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

// (2026-10-16 v2) 첫 판 DIAG(53 B)도 읽음 — 새 칸은 0. 길이가 둘 다 아니면 거부
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
  nt_hardware::DiagSummary s;
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

// (2026-10-16) 조향 영점 · 방향: 모터 + = 위에서 시계, URDF 조향축 +z (+ = 반시계) → scale −1, zero_tick = 직진 눈금
TEST(JointConvert, SteerZeroAndSignFollowUrdf)
{
  const double k = 2.0 * M_PI / 4096.0;
  const auto tick_to_motor = [k](int tick) {return (tick - 2048) * k;};   // 펌웨어 위치 모드와 같은 식
  const int zeros[4] = {2054, 2583, 3596, 2589};                          // 임시 영점 ID 11 · 12 · 13 · 14
  for (int z : zeros) {
    nt_hardware::JointSpec j;
    j.scale = -1.0;
    j.zero_rad = (z - 2048) * k;
    EXPECT_NEAR(nt_hardware::motor_to_joint(j, tick_to_motor(z)), 0.0, 1e-12);          // 직진 = 관절 0
    // 눈금이 늘면(모터 +, 위에서 시계) 관절은 − (URDF +z 기준 시계 = 음수)
    EXPECT_NEAR(nt_hardware::motor_to_joint(j, tick_to_motor(z + 57)), -57 * k, 1e-12);
    // 관절 +5° (반시계) 명령 → 모터 눈금은 줄어듦
    const double m = nt_hardware::joint_to_motor(j, 5.0 * M_PI / 180.0);
    EXPECT_EQ(static_cast<int>(std::lround(m / k)) + 2048, z - 57);
    // 왕복
    for (double q : {-1.0, -0.3, 0.0, 0.4, 1.2}) {
      EXPECT_NEAR(nt_hardware::motor_to_joint(j, nt_hardware::joint_to_motor(j, q)), q, 1e-12);
    }
  }
}

// 바퀴: 모터 + = 전진, URDF 바퀴축 +y 에서 + 회전 = 전진 → scale +1, 영점 없음 (기본값 그대로)
TEST(JointConvert, DriveDefaultIsIdentity)
{
  nt_hardware::JointSpec j;   // scale 1, zero_rad 0
  EXPECT_DOUBLE_EQ(nt_hardware::motor_to_joint(j, 2.5), 2.5);
  EXPECT_DOUBLE_EQ(nt_hardware::joint_to_motor(j, -0.3), -0.3);
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
