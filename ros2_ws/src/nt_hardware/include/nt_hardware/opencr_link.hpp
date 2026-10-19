// opencr_link.hpp — Jetson 쪽 "OpenCR 와 편지 주고받기" (ROS 없는 순수 C++)
// =====================================================================
// 시리얼로 OpenCR 와 주고받는 일을 ROS 와 떼어서 클래스 하나로.
//   ROS 쪽 껍데기(opencr_hardware.cpp)는 이걸 불러서 ros2_control 에 값을 넘겨주기만 함
//   → 이 클래스는 로봇 없이 가짜 OpenCR(test/test_opencr.cpp)로 끝까지 시험할 수 있음
//
// 이 파일 = SerialPort(포트 열기 · 바이트 읽기 · 쓰기) + OpenCRLink(편지 규칙 · 단위 변환)
//
// 하는 일
//   configure() : 켤 때 한 번. CONFIG 상자(모터 목록) 보내고 CONFIG_ACK(누가 대답했나) 기다림
//   send_command(): 100 Hz. 관절 단위 목표 → CMD 상자 (NaN = "목표 없음")
//   poll()      : 100 Hz. 들어온 바이트에서 STATE 상자 꺼내서 관절 단위로 바꿔 둠
//
// 단위: 펌웨어는 모터 축 기준 rad · rad/s · A 로 말함 (nt_packet.h).
//       관절이 rad 가 아닌 것(그리퍼 = 손가락 이동 m)만 scale 로 바꿈:  관절값 = 모터값 × scale

#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "nt_packet.h"   // firmware/nt_opencr/nt_packet.h — 펌웨어와 같은 파일 (CMakeLists 참고)
#include "nt_hardware/position_rate.hpp"

namespace nt_hardware
{

// ============================================================================
// SerialPort — 리눅스 시리얼 포트 (USB 로 꽂힌 OpenCR = /dev/ttyACM0 같은 것)
// "포트 열기 · 바이트 읽기 · 쓰기" 만.
// ROS 와 상관없는 순수 C++ (POSIX) → 테스트에서는 가짜 포트(pty)를 붙여서 씀.
//
// 읽기는 "기다리지 않음(non-blocking)": 지금 와 있는 바이트만 가져오고 바로 돌아옴.
//   ros2_control 의 read() 는 100 Hz 로 불리는데, 거기서 기다리면 제어 주기가 밀리기 때문.
// ============================================================================

class SerialPort
{
public:
  SerialPort() = default;
  ~SerialPort();
  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;

  // 포트 열기 (raw 모드, 8N1). 실패하면 false + err 에 이유
  bool open(const std::string & path, int baud, std::string & err);
  // 이미 열린 파일 번호를 넘겨받기 (테스트용 가짜 포트)
  void adopt(int fd);
  void close();
  bool is_open() const {return fd_ >= 0;}

  // 지금 와 있는 바이트를 최대 cap 개까지. 돌려주는 값 = 읽은 개수 (0 = 없음, -1 = 포트 고장 · 뽑힘)
  long read_some(uint8_t * buf, std::size_t cap);
  // 전부 쓸 때까지 (짧게) 기다림. 실패하면 false
  bool write_all(const uint8_t * data, std::size_t len);
  // 쌓여 있던 받은 바이트 버리기 (켤 때 옛날 상자 지우기)
  void flush_input();

private:
  int fd_ = -1;
};

// ============================================================================
// DurationStats — 주기 함수가 시간을 얼마나 쓰나 (2026-10-16 메모리 · 시간 검토)
//   100 Hz = 한 주기 10 ms. read/write 가 그 안에서 얼마를 쓰는지, 한도를 넘은 횟수를 셈
//   할당 없음 (숫자 몇 개만) → 제어 주기 안에서 불러도 됨. 로그는 부르는 쪽이 가끔 (예: 10 s 마다) 찍고 reset
// ============================================================================
struct DurationStats
{
  double budget_s = 0.005;   // 이보다 오래 걸리면 over 로 셈
  uint32_t n = 0, over = 0;
  double sum_s = 0.0, max_s = 0.0;
  void add(double s)
  {
    ++n;
    sum_s += s;
    if (s > max_s) {max_s = s;}
    if (s > budget_s) {++over;}
  }
  double mean_s() const {return n ? sum_s / n : 0.0;}
  void reset() {n = over = 0; sum_s = max_s = 0.0;}
};

// ============================================================================
// OpenCRLink — 편지 주고받기 (위 설명)
// ============================================================================

struct JointSpec
{
  std::string name;
  uint8_t id = 0;
  uint8_t mode = ntp::MODE_POSITION;   // ntp::MODE_VELOCITY / MODE_POSITION / MODE_CURRENT_POSITION
  uint16_t profile_vel = 0;
  uint16_t profile_acc = 0;
  uint16_t goal_current = 0;
  double scale = 1.0;                  // 관절값 = (모터값 − zero_rad) × scale. 그리퍼 = 0.01149 m/rad, 조향 = −1 (방향이 URDF 와 반대일 때)
  double zero_rad = 0.0;               // 관절 0 일 때의 모터값 [모터 rad] = (zero_tick − 2048) × 2π/4096. 위치 모드만 (바퀴는 0)
};

// 모터값 ↔ 관절값 (2026-10-16 조향 영점 · 방향). 모터값 = 펌웨어 rad (위치 모드: (눈금 − 2048) × 2π/4096)
//   관절 = (모터 − zero_rad) × scale,  모터 = 관절 / scale + zero_rad.  속도는 scale 만 (영점은 상관없음)
//   예) 조향 영점 3596 눈금 · 모터 + = 위에서 시계 / URDF 축 +z (+ = 반시계) → scale = −1, zero_tick = 3596
inline double motor_to_joint(const JointSpec & j, double m) {return (m - j.zero_rad) * j.scale;}
inline double joint_to_motor(const JointSpec & j, double q) {return q / j.scale + j.zero_rad;}

struct JointReading
{
  double position = 0.0;   // rad (그리퍼 m)
  double velocity = 0.0;   // rad/s (그리퍼 m/s)
  double current = 0.0;    // A (모터 전류 — 토크가 아님)
  double position_velocity = std::numeric_limits<double>::quiet_NaN();  // board-time estimate
};

struct LinkState
{
  uint32_t ms = 0;            // OpenCR 시계
  uint8_t seq_echo = 0;
  uint8_t status = 0;         // ntp::ST_* 비트
  uint16_t comm_err = 0;      // 비트 i = i 번째 모터 못 읽음
  ntp::Imu imu{};
  std::vector<JointReading> joints;
  bool joints_valid = false;  // 펌웨어가 설정한 모터 수만큼 보내 줬나 (CONFIG 전에는 0 개)
};

// 보드 안 시간 측정 (DIAG, 2026-10-16) 모으기 — 펌웨어가 1 초마다 보내는 것을 합쳐 둠 (플러그인이 10 s 마다 꺼내 로그)
struct DiagSummary
{
  struct Stat
  {
    uint32_t n = 0, fail = 0;
    uint64_t sum_us = 0;   // 평균 × 횟수 합 (가중 평균용)
    uint32_t max_us = 0;
    void add(const ntp::TimeStat & t)
    {
      n += t.n;
      fail += t.fail;
      sum_us += static_cast<uint64_t>(t.mean_us) * t.n;
      if (t.max_us > max_us) {max_us = t.max_us;}
    }
    double mean_us() const {return n ? static_cast<double>(sum_us) / n : 0.0;}
  };
  uint32_t packets = 0;
  Stat loop, sync_read, sync_write, single_read, single_write;
  Stat usb_tx;                  // 상태 · DIAG USB 보내기 (fail = 송신 버퍼가 차서 버림, v2 펌웨어부터)
  uint16_t config_ms = 0;       // 가장 최근 CONFIG 처리 시간 [ms] (0 = 이번에 없음)
  // CONFIG 전 구간 (DIAG_FLAG_BEFORE_CONFIG) — 포트를 열기 전 · 설정 전 숫자. 측정에 섞지 않고 따로 셈 (숨기지 않음)
  uint32_t before_config_packets = 0;
  Stat before_loop, before_usb_tx;
  uint32_t resync = 0;
  uint16_t fault_stop_us = 0;   // 가장 최근 값 (0 = 고장 없음)
  uint8_t fault_flags = 0;      // 가장 최근 값 (비트 0 잠금, 1 침묵)
  uint8_t volt_min_dv = 0;      // 배터리 전압 최저 [0.1 V] (모은 구간 전체, 0 = 못 읽음)
  uint8_t volt_mean_dv = 0;     // 가장 최근 구간 평균 [0.1 V]
  void add(const ntp::Diag & g)
  {
    if (g.fault_flags & ntp::DIAG_FLAG_BEFORE_CONFIG) {
      ++before_config_packets;
      before_loop.add(g.loop);
      before_usb_tx.add(g.usb_tx);
      return;
    }
    ++packets;
    usb_tx.add(g.usb_tx);
    if (g.config_ms) {config_ms = g.config_ms;}
    loop.add(g.loop);
    sync_read.add(g.sync_read);
    sync_write.add(g.sync_write);
    single_read.add(g.single_read);
    single_write.add(g.single_write);
    resync += g.resync;
    fault_stop_us = g.fault_stop_us;
    fault_flags = g.fault_flags;
    if (g.volt_min_dv && (volt_min_dv == 0 || g.volt_min_dv < volt_min_dv)) {volt_min_dv = g.volt_min_dv;}
    if (g.volt_mean_dv) {volt_mean_dv = g.volt_mean_dv;}
  }
};

class OpenCRLink
{
public:
  using Clock = std::chrono::steady_clock;

  OpenCRLink(std::vector<JointSpec> joints, uint8_t bus_watchdog);

  bool open(const std::string & port, int baud, std::string & err);
  void adopt_fd(int fd);       // 테스트용: 가짜 포트 붙이기
  void close();

  // CONFIG 보내고 ACK 기다림 (timeout_s 동안, 1 s 마다 다시 보냄).
  // 돌려줌: ok_mask (비트 i = i 번째 모터 대답) / -1 = ACK 를 못 받음
  int configure(double timeout_s);

  // 목표 보내기. targets 는 관절 순서 · 관절 단위, NaN(또는 inf) = 목표 없음
  //   → 펌웨어가 바퀴는 0, 위치 모터는 마지막 목표에서 버팀
  bool send_command(const std::vector<double> & targets, bool torque);

  // 들어온 바이트 처리. 돌려줌: 새로 받은 STATE 개수 (-1 = 포트 고장)
  int poll();
  // STATE 가 하나라도 올 때까지 최대 timeout_s 기다림
  bool wait_for_state(double timeout_s);

  const LinkState & state() const {return state_;}
  bool has_state() const {return got_state_;}
  double seconds_since_state() const;     // 마지막 STATE 뒤 지난 시간 (없으면 아주 큰 값)
  // i 번째 모터 값이 "지금의 측정" 인가 (2026-10-16 R3)
  //   = 마지막 STATE 가 max_age_s 안 + 펌웨어가 모터 수만큼 보냄 + 이번에 그 모터를 읽었음(comm_err 비트 0)
  //   펌웨어는 못 읽은 모터의 옛날 값을 그대로 남겨 보냄 → 이걸로 걸러야 옛 값이 새 시각을 달고 나가지 않음
  bool joint_measured(std::size_t i, double max_age_s) const;
  uint32_t bad_checksums() const {return parser_.bad_checksum_count();}
  // 지금까지 모은 DIAG 를 꺼내고 0 부터 다시 (packets == 0 이면 DIAG 를 안 보내는 옛 펌웨어거나 아직 1 초 전)
  DiagSummary take_diag() {DiagSummary d = diag_; diag_ = DiagSummary{}; return d;}
  // 가장 최근 DIAG 하나와 받은 개수 (개수가 바뀌면 새 것 — 배터리 판단은 1 초마다 바로)
  const ntp::Diag & last_diag() const {return last_diag_;}
  uint32_t diag_count() const {return diag_count_;}
  uint32_t frames_received() const {return frames_;}
  std::size_t size() const {return joints_.size();}
  const std::vector<JointSpec> & joints() const {return joints_;}

  // ok_mask 에서 빠진 모터 이름들 ("joint3(ID 13), ...") — 로그용
  std::string missing_joints(int ok_mask) const;

private:
  void handle_frame();

  std::vector<JointSpec> joints_;
  uint8_t bus_watchdog_;
  SerialPort port_;
  ntp::Parser parser_;
  LinkState state_;
  std::vector<PositionRate> position_rates_;
  bool got_state_ = false;
  Clock::time_point last_state_time_{};
  uint8_t seq_ = 0;
  uint32_t frames_ = 0;
  // configure() 가 ACK 를 기다리는 동안
  bool waiting_ack_ = false;
  int ack_mask_ = -1;
  DiagSummary diag_;
  ntp::Diag last_diag_{};
  uint32_t diag_count_ = 0;
};

}  // namespace nt_hardware
