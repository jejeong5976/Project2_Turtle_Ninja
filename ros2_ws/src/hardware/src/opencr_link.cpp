// opencr_link.cpp — 설명은 opencr_link.hpp

#include "hardware/opencr_link.hpp"

#include <cerrno>
#include <cmath>
#include <cstring>
#include <thread>
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

namespace hardware
{

// ============================================================================ SerialPort
namespace
{
speed_t to_speed(int baud)
{
  switch (baud) {
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 921600: return B921600;
    case 1000000: return B1000000;
    case 2000000: return B2000000;
    default: return B1000000;
  }
}
}  // namespace

SerialPort::~SerialPort() {close();}

bool SerialPort::open(const std::string & path, int baud, std::string & err)
{
  close();
  const int fd = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) {
    err = path + " 를 못 엶: " + std::strerror(errno) +
      " (OpenCR 가 꽂혀 있나? dialout 그룹인가? udev 이름이 맞나?)";
    return false;
  }
  termios tio{};
  if (tcgetattr(fd, &tio) != 0) {
    err = std::string("tcgetattr 실패: ") + std::strerror(errno);
    ::close(fd);
    return false;
  }
  cfmakeraw(&tio);              // 글자 변환 · 줄바꿈 처리 · 에코 전부 끔 (바이트 그대로)
  tio.c_cflag |= CLOCAL | CREAD;
  tio.c_cflag &= ~CSTOPB;       // 정지 비트 1
  tio.c_cflag &= ~CRTSCTS;      // 흐름 제어 없음
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;
  cfsetispeed(&tio, to_speed(baud));   // USB 가상 시리얼이라 실제 속도엔 영향 없음
  cfsetospeed(&tio, to_speed(baud));
  if (tcsetattr(fd, TCSANOW, &tio) != 0) {
    err = std::string("tcsetattr 실패: ") + std::strerror(errno);
    ::close(fd);
    return false;
  }
  fd_ = fd;
  return true;
}

void SerialPort::adopt(int fd)
{
  close();
  fd_ = fd;
}

void SerialPort::close()
{
  if (fd_ >= 0) {::close(fd_);}
  fd_ = -1;
}

long SerialPort::read_some(uint8_t * buf, std::size_t cap)
{
  if (fd_ < 0) {return -1;}
  const ssize_t n = ::read(fd_, buf, cap);
  if (n > 0) {return static_cast<long>(n);}
  if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {return 0;}
  if (n == 0) {return 0;}   // pty 등에서 지금 받을 게 없음
  return -1;                // 진짜 고장 (USB 뽑힘 등)
}

bool SerialPort::write_all(const uint8_t * data, std::size_t len)
{
  if (fd_ < 0) {return false;}
  std::size_t done = 0;
  int spins = 0;
  while (done < len) {
    const ssize_t n = ::write(fd_, data + done, len - done);
    if (n > 0) {done += static_cast<std::size_t>(n); continue;}
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
      if (++spins > 20) {return false;}   // 최대 ~20 ms 기다리고 포기 (제어 주기를 오래 막지 않게)
      pollfd p{fd_, POLLOUT, 0};
      ::poll(&p, 1, 1);
      continue;
    }
    return false;
  }
  return true;
}

void SerialPort::flush_input()
{
  if (fd_ >= 0) {tcflush(fd_, TCIFLUSH);}
}

// ============================================================================ OpenCRLink
OpenCRLink::OpenCRLink(std::vector<JointSpec> joints, uint8_t bus_watchdog)
: joints_(std::move(joints)), bus_watchdog_(bus_watchdog)
{
  state_.joints.resize(joints_.size());
  position_rates_.resize(joints_.size());
}

bool OpenCRLink::open(const std::string & port, int baud, std::string & err)
{
  if (joints_.size() > ntp::MAX_JOINTS) {
    err = "모터가 " + std::to_string(joints_.size()) + " 개 — 보드 1장에 최대 " +
      std::to_string(ntp::MAX_JOINTS) + " 개 (hardware_map.yaml 에서 보드를 나눌 것)";
    return false;
  }
  if (!port_.open(port, baud, err)) {return false;}
  port_.flush_input();
  return true;
}

void OpenCRLink::adopt_fd(int fd) {port_.adopt(fd);}

void OpenCRLink::close() {port_.close();}

int OpenCRLink::configure(double timeout_s)
{
  ntp::Config c{};
  c.n = static_cast<uint8_t>(joints_.size());
  c.bus_watchdog = bus_watchdog_;
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    c.j[i].id = joints_[i].id;
    c.j[i].mode = joints_[i].mode;
    c.j[i].profile_vel = joints_[i].profile_vel;
    c.j[i].profile_acc = joints_[i].profile_acc;
    c.j[i].goal_current = joints_[i].goal_current;
  }
  uint8_t buf[ntp::MAX_FRAME];
  const uint16_t len = ntp::encode_config(c, buf, sizeof(buf));
  if (len == 0) {return -1;}

  waiting_ack_ = true;
  ack_mask_ = -1;
  const auto start = Clock::now();
  auto last_send = start - std::chrono::seconds(10);
  while (std::chrono::duration<double>(Clock::now() - start).count() < timeout_s) {
    // 1 s 마다 다시 보냄: OpenCR 가 막 켜져서 IMU 영점 잡는 중(최대 5 s)이면 처음 상자를 못 볼 수 있음
    if (Clock::now() - last_send > std::chrono::seconds(1)) {
      if (!port_.write_all(buf, len)) {break;}
      last_send = Clock::now();
    }
    if (poll() < 0) {break;}
    if (ack_mask_ >= 0) {break;}
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  waiting_ack_ = false;
  // Do not carry a previous session's samples (or clock fault) across CONFIG.
  position_rates_.assign(joints_.size(), PositionRate{});
  for (auto & j : state_.joints) {j.position_velocity = NAN;}
  return ack_mask_;
}

bool OpenCRLink::send_command(const std::vector<double> & targets, bool torque)
{
  ntp::Cmd c{};
  c.seq = ++seq_;
  c.flags = torque ? ntp::FLAG_TORQUE : 0;
  c.n = static_cast<uint8_t>(joints_.size());
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    const double t = i < targets.size() ? targets[i] : NAN;
    if (std::isfinite(t)) {
      c.valid = static_cast<uint16_t>(c.valid | (1u << i));
      c.target[i] = static_cast<float>(joint_to_motor(joints_[i], t));   // 관절 단위 → 모터 rad (영점 · 방향 포함)
    } else {
      c.target[i] = 0.0f;
    }
  }
  uint8_t buf[ntp::MAX_FRAME];
  const uint16_t len = ntp::encode_cmd(c, buf, sizeof(buf));
  return len > 0 && port_.write_all(buf, len);
}

int OpenCRLink::poll()
{
  int new_states = 0;
  uint8_t buf[512];
  for (int guard = 0; guard < 64; ++guard) {   // 한 번에 너무 오래 붙잡지 않게 최대 32 kB
    const long n = port_.read_some(buf, sizeof(buf));
    if (n < 0) {return -1;}
    if (n == 0) {break;}
    for (long k = 0; k < n; ++k) {
      if (!parser_.feed(buf[k])) {continue;}
      ++frames_;
      if (parser_.type() == ntp::TYPE_STATE) {
        ntp::State s;
        if (!ntp::decode_state(parser_.data(), parser_.len(), s)) {continue;}
        state_.ms = s.ms;
        state_.seq_echo = s.seq_echo;
        state_.status = s.status;
        state_.comm_err = s.comm_err;
        state_.imu = s.imu;
        state_.joints_valid = (s.n == joints_.size());
        if (state_.joints_valid) {
          for (std::size_t i = 0; i < joints_.size(); ++i) {
            const double k_scale = joints_[i].scale;
            state_.joints[i].position = motor_to_joint(joints_[i], s.pos[i]);
            state_.joints[i].velocity = s.vel[i] * k_scale;
            state_.joints[i].current = s.cur[i];
            state_.joints[i].position_velocity = position_rates_[i].feed(
              s.ms, (s.comm_err & (1u << i)) ? NAN : state_.joints[i].position);
          }
        } else {
          for (std::size_t i = 0; i < joints_.size(); ++i) {
            position_rates_[i].feed(s.ms, NAN);
            state_.joints[i].position_velocity = NAN;
          }
        }
        got_state_ = true;
        last_state_time_ = Clock::now();
        ++new_states;
      } else if (parser_.type() == ntp::TYPE_DIAG) {
        ntp::Diag g;
        if (ntp::decode_diag(parser_.data(), parser_.len(), g)) {
          diag_.add(g);
          last_diag_ = g;
          ++diag_count_;
        }
      } else if (parser_.type() == ntp::TYPE_CONFIG_ACK && waiting_ack_) {
        ntp::ConfigAck a;
        if (ntp::decode_ack(parser_.data(), parser_.len(), a) && a.n == joints_.size()) {
          ack_mask_ = a.ok_mask;
        }
      }
    }
  }
  return new_states;
}

bool OpenCRLink::wait_for_state(double timeout_s)
{
  const auto start = Clock::now();
  const bool had = got_state_;
  const auto before = last_state_time_;
  while (std::chrono::duration<double>(Clock::now() - start).count() < timeout_s) {
    if (poll() < 0) {return false;}
    if (got_state_ && (!had || last_state_time_ != before)) {return true;}
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return false;
}

double OpenCRLink::seconds_since_state() const
{
  if (!got_state_) {return 1e9;}
  return std::chrono::duration<double>(Clock::now() - last_state_time_).count();
}

bool OpenCRLink::joint_measured(std::size_t i, double max_age_s) const
{
  return got_state_ && state_.joints_valid && i < joints_.size() && i < 16 &&
         !(state_.comm_err & (1u << i)) && seconds_since_state() <= max_age_s;
}

std::string OpenCRLink::missing_joints(int ok_mask) const
{
  std::string out;
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    if (ok_mask >= 0 && (ok_mask & (1 << i))) {continue;}
    if (!out.empty()) {out += ", ";}
    out += joints_[i].name + "(ID " + std::to_string(joints_[i].id) + ")";
  }
  return out;
}

}  // namespace hardware
