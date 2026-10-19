// motor_bus.cpp — 다이나믹셀 Sync Read / Sync Write (설명은 motor_bus.h)
// Sync 구조체 쓰는 법은 ROBOTIS Dynamixel2Arduino 예제 advanced/sync_read_write_velocity 와 같음

#include "motor_bus.h"

#include <Dynamixel2Arduino.h>

namespace
{
// OpenCR 의 다이나믹셀 포트 (예제 그대로)
#define DXL_SERIAL Serial3
const int DXL_DIR_PIN = 84;

const float TWO_PI_F = 6.28318530718f;
const float POS_RAD_PER_TICK = TWO_PI_F / 4096.0f;
const float VEL_RADS_PER_TICK = 0.229f * TWO_PI_F / 60.0f;   // ≈ 0.02398
const float CUR_A_PER_TICK = 0.00269f;

// XM430 컨트롤 테이블 주소 (Sync 용. 나머지는 라이브러리의 이름표 ControlTableItem 으로)
const uint16_t ADDR_GOAL_VELOCITY = 104;
const uint16_t ADDR_GOAL_POSITION = 116;
const uint16_t ADDR_PRESENT_CURRENT = 126;   // 126 전류(2) · 128 속도(4) · 132 위치(4) 가 붙어 있음 → 한 번에 10 바이트

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
using namespace ControlTableItem;

// 지금 설정 (CONFIG 로 받은 것)
uint8_t n_ = 0;
uint8_t ids_[ntp::MAX_JOINTS];
uint8_t modes_[ntp::MAX_JOINTS];
uint16_t ok_ = 0;          // 비트 i = i 번째 모터가 ping 에 대답함 → 이 모터들만 읽고 씀
bool torque_ = false;
// 운전 중 되읽기 (R5)
uint8_t health_idx_ = 0;
uint16_t torque_lost_ = 0;   // 토크를 켜 둔 상태인데 Torque Enable 이 0 으로 읽힌 모터
uint16_t hw_err_ = 0;        // Hardware Error Status ≠ 0 인 모터
// ---- 고장 잠금 · 시간 예산 (2026-10-16 재검증 R6 · R7) ----------------------------------------------
// 순서: Sync Write 실패 관측 → 고장 확정 → 바퀴에 하나씩 속도 0 (예산 안에서) → 실패면 버스 침묵 → 모터 Bus Watchdog
//   고장 확정 = 연속 WRITE_FAIL_LIMIT 번, 또는 첫 실패에서 WRITE_FAIL_CONFIRM_MS 가 지난 뒤 또 실패 (루프가 밀려도 늦어지지 않게)
//   ★ 확정되면 **잠금(latched_)**: 통신이 돌아와도 다시 목표를 쓰지 않음 (예전: 다음 Sync Write 성공 때 원래 속도로 재출발)
//     잠금을 푸는 길은 CONFIG(bus_configure) 하나 — 사람이 원인 확인 후 하드웨어를 다시 설정할 때만
//   시간: 운전 중 모든 개별 읽기 · 쓰기에 IO_TIMEOUT_MS 를 직접 넘김 (라이브러리 기본은 100 ms → 바퀴 4개면 0.4 s)
//   ⚠ 아래 숫자는 설계 목표. 실제 버스 · 모터에서 지키는지는 tools/hw_bench.py 로 따로 재야 함
const uint8_t WRITE_FAIL_LIMIT = 3;           // 연속 실패 횟수 (임시 판정값 — 정지 시간 보장이 아님)
const uint32_t WRITE_FAIL_CONFIRM_MS = 20;    // 첫 실패 뒤 이만큼 지나 또 실패하면 횟수와 상관없이 확정
const uint32_t IO_TIMEOUT_MS = 3;             // 운전 중 개별 읽기 · 쓰기 응답 대기 (1 Mbps 응답은 0.2 ms 남짓)
const uint32_t SYNC_READ_TIMEOUT_MS = 3;      // Sync Read 모터 하나당 응답 대기 (라이브러리 기본 10 ms)
const uint32_t STOP_BUDGET_MS = 20;           // 고장 확정 → 개별 정지 시도 전체 (바퀴 4 + 조향 4 + 팔 5 = 13 번 합)
                                              //   보통 응답이면 13 번 × 약 0.3~0.5 ms. 응답이 느려 못 끝내면 침묵으로 (안전한 쪽)
const uint32_t TORQUE_BUDGET_MS = 30;         // 토크 켜기/끄기 한 번 시도 전체 (넘으면 다음 바퀴에 다시)
uint8_t write_fail_streak_ = 0;
uint32_t first_fail_ms_ = 0;
bool latched_ = false;       // 고장 잠금: 목표 · 토크를 더는 안 바꿈 (CONFIG 로만 풀림)
bool silent_ = false;        // 잠금 + 개별 정지도 확인 못 함 → 버스에 아무 패킷도 안 보냄 → 모터 Bus Watchdog 이 스스로 세움
uint32_t stop_attempt_ms_ = 0;   // 마지막 개별 정지 시도에 걸린 시간 (시험 · 진단용)
uint16_t read_ok_ = 0;           // 직전 Sync Read 에서 읽은 모터 (고장 때 위치 모터의 "지금 위치" 로 씀)
// 보드 안 시간 측정 (DIAG) — micros() 로. PC · USB 시간이 섞이지 않은 버스 · 펌웨어 시간
TimeAcc t_sync_read_, t_sync_write_, t_read_, t_write_;
// 배터리 전압 (되읽기 때 1 모터씩 Present Input Voltage) [0.1 V]
uint32_t volt_sum_dv_ = 0, volt_n_ = 0;
uint8_t volt_min_dv_ = 0;   // 0 = 이번 구간에 못 읽음
uint32_t stop_attempt_us_ = 0;

bool last_ok()   // 바로 앞 단일 읽기 · 쓰기가 통신까지 성공했나
{
  return static_cast<int>(dxl.getLastLibErrCode()) == 0;   // DXL_LIB_OK
}

// 운전 중 개별 읽기 · 쓰기는 전부 이 둘로 — 응답 대기 IO_TIMEOUT_MS + 왕복 시간 기록 (DIAG single_read/write)
int32_t timed_read(uint8_t item, uint8_t id)
{
  const uint32_t t = micros();
  const int32_t v = dxl.readControlTableItem(item, id, IO_TIMEOUT_MS);
  t_read_.add(micros() - t, last_ok());
  return v;
}

bool timed_write(uint8_t item, uint8_t id, int32_t value)
{
  const uint32_t t = micros();
  const bool ok = dxl.writeControlTableItem(item, id, value, IO_TIMEOUT_MS);
  t_write_.add(micros() - t, ok);
  return ok;
}

// Sync Read: 전류 · 속도 · 위치
typedef struct
{
  int16_t current;
  int32_t velocity;
  int32_t position;
} __attribute__((packed)) sr_data_t;
sr_data_t sr_data_[ntp::MAX_JOINTS];
uint8_t sr_buf_[256];
DYNAMIXEL::InfoSyncReadInst_t sr_;
DYNAMIXEL::XELInfoSyncRead_t sr_xels_[ntp::MAX_JOINTS];

// Sync Write: 바퀴 속도 / 위치 모터 위치 — 두 묶음
int32_t sw_vel_[ntp::MAX_JOINTS];
int32_t sw_pos_[ntp::MAX_JOINTS];
DYNAMIXEL::InfoSyncWriteInst_t sw_vel_info_, sw_pos_info_;
DYNAMIXEL::XELInfoSyncWrite_t sw_vel_xels_[ntp::MAX_JOINTS], sw_pos_xels_[ntp::MAX_JOINTS];

int32_t rad_to_pos_tick(float rad)
{
  int32_t t = static_cast<int32_t>(lroundf(rad / POS_RAD_PER_TICK)) + 2048;
  if (t < 0) {t = 0;}
  if (t > 4095) {t = 4095;}
  return t;
}

int32_t rads_to_vel_tick(float rads)
{
  int32_t t = static_cast<int32_t>(lroundf(rads / VEL_RADS_PER_TICK));
  // 모터 안의 Velocity Limit 이 한 번 더 자름. 여기서는 터무니없는 값만 막음
  if (t > 1023) {t = 1023;}
  if (t < -1023) {t = -1023;}
  return t;
}

void setup_sync_read()
{
  sr_.packet.p_buf = sr_buf_;
  sr_.packet.buf_capacity = sizeof(sr_buf_);
  sr_.packet.is_completed = false;
  sr_.addr = ADDR_PRESENT_CURRENT;
  sr_.addr_length = 10;
  sr_.p_xels = sr_xels_;
  sr_.xel_count = 0;
  // ⚠ 라이브러리는 대답이 없는 모터를 만나면 거기서 읽기를 멈춤 (그 뒤 모터도 못 읽음)
  //   → ping 에 대답한 모터만 목록에 넣음. 돌다가 하나가 끊기면 그 뒤도 comm_err 로 보임 (선 · 전원 점검 신호)
  for (uint8_t i = 0; i < n_; ++i) {
    if (!(ok_ & (1u << i))) {continue;}
    DYNAMIXEL::XELInfoSyncRead_t &x = sr_xels_[sr_.xel_count++];
    x.id = ids_[i];
    x.p_recv_buf = reinterpret_cast<uint8_t *>(&sr_data_[i]);   // 받는 칸은 모터 번호 i 자리
  }
  sr_.is_info_changed = true;
}

void setup_sync_write(DYNAMIXEL::InfoSyncWriteInst_t &info, DYNAMIXEL::XELInfoSyncWrite_t *xels, uint16_t addr)
{
  info.packet.p_buf = nullptr;      // 라이브러리 안쪽 버퍼 사용 (예제와 같음)
  info.packet.is_completed = false;
  info.addr = addr;
  info.addr_length = 4;
  info.p_xels = xels;
  info.xel_count = 0;
  info.is_info_changed = true;
}
}  // namespace

bool bus_begin(uint32_t baud)
{
  dxl.begin(baud);
  dxl.setPortProtocolVersion(2.0);
  return true;
}

uint16_t bus_configure(const ntp::Config &c)
{
  n_ = c.n;
  uint16_t ok = 0;
  for (uint8_t i = 0; i < n_; ++i) {
    ids_[i] = c.j[i].id;
    modes_[i] = c.j[i].mode;
    if (!dxl.ping(ids_[i])) {continue;}
    // ping 만이 아니라 아래 설정이 **전부** 써져야 ok. 하나라도 실패하면 그 모터는 안 쓰는 것으로
    bool good = dxl.torqueOff(ids_[i]);                                       // 모드는 토크가 꺼져 있어야 바뀜
    good = dxl.writeControlTableItem(RETURN_DELAY_TIME, ids_[i], 0) && good;   // 응답 지연 0 (기본 500 µs → 14개면 7 ms 손해)
    if (modes_[i] == ntp::MODE_VELOCITY) {
      good = dxl.setOperatingMode(ids_[i], OP_VELOCITY) && good;
    } else if (modes_[i] == ntp::MODE_CURRENT_POSITION) {
      good = dxl.setOperatingMode(ids_[i], OP_CURRENT_BASED_POSITION) && good;
      good = dxl.writeControlTableItem(GOAL_CURRENT, ids_[i], c.j[i].goal_current) && good;
    } else {
      good = dxl.setOperatingMode(ids_[i], OP_POSITION) && good;
    }
    good = dxl.writeControlTableItem(PROFILE_VELOCITY, ids_[i], c.j[i].profile_vel) && good;
    good = dxl.writeControlTableItem(PROFILE_ACCELERATION, ids_[i], c.j[i].profile_acc) && good;
    // Bus Watchdog: OpenCR 가 멈춰 명령이 끊기면 모터가 스스로 멈춤 (값이 0 이 아니면 켜짐, 단위 20 ms)
    good = dxl.writeControlTableItem(BUS_WATCHDOG, ids_[i], 0) && good;        // 걸려 있던 것 풀기
    good = dxl.writeControlTableItem(BUS_WATCHDOG, ids_[i], c.bus_watchdog) && good;
    if (good) {ok |= static_cast<uint16_t>(1u << i);}
  }
  ok_ = ok;
  torque_ = false;
  torque_lost_ = 0;
  hw_err_ = 0;
  write_fail_streak_ = 0;
  // 고장 잠금을 푸는 유일한 길 = CONFIG (사람이 원인 확인 후 하드웨어를 다시 설정할 때). 자동으로 다시 보내는 곳은 없음
  //   ⚠ 위에서 모든 모터 토크를 끔 → 팔이 들려 있으면 처짐. 복구 전에 팔을 받치거나 안정 자세에 둘 것 (docs/hardware.md)
  latched_ = false;
  silent_ = false;
  stop_attempt_ms_ = 0;
  stop_attempt_us_ = 0;
  read_ok_ = 0;
  setup_sync_read();
  setup_sync_write(sw_vel_info_, sw_vel_xels_, ADDR_GOAL_VELOCITY);
  setup_sync_write(sw_pos_info_, sw_pos_xels_, ADDR_GOAL_POSITION);
  return ok;
}

void bus_set_torque(bool on)
{
  // 고장 잠금 중에는 토크를 바꾸지 않음 (켜서 다시 움직이지도, 꺼서 팔을 떨어뜨리지도 않게) — CONFIG 로 풀린 뒤에
  if (latched_ || on == torque_) {return;}
  // 전부 성공했을 때만 "켜짐/꺼짐" 으로 기록. 실패하면 다음 바퀴(10 ms 뒤)에 다시 시도하고,
  //          STATE 의 ST_TORQUE 비트는 실제 결과대로 → 플러그인이 "토크를 못 켬" 을 알아챔
  // (R7) 모든 읽기 · 쓰기에 IO_TIMEOUT_MS, 한 번 시도 전체는 TORQUE_BUDGET_MS — 버스가 고장 나도 한 바퀴가 길어지지 않게
  const uint32_t t0 = millis();
  bool all = true;
  for (uint8_t i = 0; i < n_; ++i) {
    if (!(ok_ & (1u << i))) {continue;}
    if (millis() - t0 > TORQUE_BUDGET_MS) {all = false; break;}   // 남은 모터는 다음 바퀴에
    if (on && modes_[i] != ntp::MODE_VELOCITY) {
      // 목표 = 지금 위치 → 토크가 켜지는 순간 제자리
      const int32_t now_pos = static_cast<int32_t>(timed_read(PRESENT_POSITION, ids_[i]));
      if (!last_ok()) {
        // (R5) 읽기 실패면 돌려받은 값(0 등)이 "지금 위치" 가 아님 → 그걸 목표로 쓰면 켜는 순간 팔이 튐
        //      → 이 모터는 이번에 토크를 켜지 않음. all = false 라 10 ms 뒤 다시 시도, 계속 실패면 플러그인이 0.5 s 뒤 ERROR
        all = false;
        continue;
      }
      all = timed_write(GOAL_POSITION, ids_[i], now_pos) && all;
    }
    if (on && modes_[i] == ntp::MODE_VELOCITY) {all = timed_write(GOAL_VELOCITY, ids_[i], 0) && all;}
    all = timed_write(TORQUE_ENABLE, ids_[i], on ? 1 : 0) && all;
  }
  if (all) {torque_ = on;}
}

bool bus_torque_is_on()
{
  return torque_ && torque_lost_ == 0;
}

uint16_t bus_hw_error_mask()
{
  return hw_err_;
}

void bus_health_step()
{
  if (silent_ || n_ == 0 || ok_ == 0) {return;}
  for (uint8_t k = 0; k < n_; ++k) {   // 다음 "쓰는 모터"
    health_idx_ = static_cast<uint8_t>((health_idx_ + 1) % n_);
    if (ok_ & (1u << health_idx_)) {break;}
  }
  const uint8_t i = health_idx_;
  const uint16_t bit = static_cast<uint16_t>(1u << i);
  const int32_t he = timed_read(HARDWARE_ERROR_STATUS, ids_[i]);
  if (!last_ok()) {
    // 대답 없는 모터: 나머지 되읽기(전압 · 토크)도 또 응답 대기만 하게 되므로 이번 바퀴는 건너뜀
    //   (그대로 두면 대기 3 번 ≈ 10 ms 로 한 바퀴가 주기를 넘김 — test_motor_bus 가 잡음). 통신 실패는 comm_err 가 이미 알림
    return;
  }
  if (he != 0) {hw_err_ |= bit;} else {hw_err_ &= static_cast<uint16_t>(~bit);}
  // 배터리 전압 (2026-10-16): 모터 입력 전압 = 그 보드 배터리 전압 (선 손실만큼 낮게). 출발 순간 처짐도 잡히게 매 바퀴
  const int32_t dv = timed_read(PRESENT_INPUT_VOLTAGE, ids_[i]);
  if (last_ok() && dv > 0 && dv < 256) {
    volt_sum_dv_ += static_cast<uint32_t>(dv);
    ++volt_n_;
    if (volt_min_dv_ == 0 || dv < volt_min_dv_) {volt_min_dv_ = static_cast<uint8_t>(dv);}
  }
  if (torque_) {
    const int32_t te = timed_read(TORQUE_ENABLE, ids_[i]);
    if (last_ok()) {
      if (te == 0) {torque_lost_ |= bit;} else {torque_lost_ &= static_cast<uint16_t>(~bit);}
    }
  } else {
    torque_lost_ = 0;
  }
}

// 고장 확정: 잠그고, 모터마다 하나씩 "지금 동작 멈춤" 을 씀 (하나씩 쓰면 모터가 대답 → 정말 받았는지 앎)
//   ① 바퀴(속도 모드): 속도 0 — 가장 위험한 것부터
//   ② 조향 · 팔(위치 모드): 목표 = 마지막으로 읽은 현재 위치 → 하던 동작(이전 목표로 가던 것)을 그 자리에서 멈춤
//      (2026-10-16 재검증 4-A: 예전에는 바퀴만 세워서, 관측 읽기가 계속되면 Watchdog 도 안 걸려 팔이 이전 목표까지 계속 갔음)
//      위치는 직전 Sync Read 값(최대 한 바퀴 10 ms 전) — 그 사이 움직인 만큼 아주 조금 되돌아갈 수 있음
//      직전 Sync Read 에서 못 읽은 위치 모터는 "지금 위치" 를 모름 → 확인 실패로 침
//   ③ 그리퍼(전류 기반 위치)는 건드리지 않음: 목표를 지금 위치로 바꾸면 쥐는 힘이 빠져 퍽을 놓칠 수 있음
//      (전류 한계 goal_current 로 누르는 중이라 계속 둬도 힘이 커지지 않음)
//   전체를 STOP_BUDGET_MS 안에: 끝낼 수 없는 쓰기는 시작하지 않음
//   하나라도 확인 못 하면 버스 침묵 → 모터 Bus Watchdog(15 × 20 ms = 0.3 s, 마지막 패킷 기준)이 모든 모터를 세움
//   ⚠ 침묵이면 읽기 · 되읽기까지 멈춤: Bus Watchdog 은 "아무 명령 패킷" 이 오면 다시 세기 때문 (읽기도 명령 패킷)
//   ⚠ 토크는 끄지 않음 — 팔이 떨어지지 않게. 다만 전원 상실 · 과부하 때 자세 유지까지 보장하는 것은 아님
static bool stop_one(uint8_t i, uint32_t t0)
{
  // 라이브러리는 "지난 시간 >= timeout" 이면 포기 → 최악 timeout + 1 ms. 그만큼 남아 있을 때만 시도
  if (millis() - t0 + IO_TIMEOUT_MS + 1 > STOP_BUDGET_MS) {return false;}
  if (modes_[i] == ntp::MODE_VELOCITY) {
    return timed_write(GOAL_VELOCITY, ids_[i], 0);
  }
  if (!(read_ok_ & (1u << i))) {return false;}   // 지금 위치를 모름
  int32_t here = sr_data_[i].position;
  if (here < 0) {here = 0;}
  if (here > 4095) {here = 4095;}   // 위치 모드 목표 범위
  return timed_write(GOAL_POSITION, ids_[i], here);
}

static void enter_fault()
{
  latched_ = true;
  const uint32_t t0 = millis();
  const uint32_t t0_us = micros();
  bool all = true;
  // pass 0 = 바퀴: 하나가 실패해도 나머지 바퀴는 계속 시도 (가장 위험한 것이라 하나라도 더 세움)
  // pass 1 = 조향 · 팔: 바퀴가 모두 확인됐을 때만. 하나라도 실패하면 거기서 멈추고 침묵 (어차피 Watchdog 이 전부 세움)
  for (int pass = 0; pass < 2 && all; ++pass) {
    for (uint8_t i = 0; i < n_; ++i) {
      if (!(ok_ & (1u << i)) || modes_[i] == ntp::MODE_CURRENT_POSITION) {continue;}
      if ((modes_[i] == ntp::MODE_VELOCITY) != (pass == 0)) {continue;}
      all = stop_one(i, t0) && all;
      if (!all && pass == 1) {break;}
    }
  }
  stop_attempt_ms_ = millis() - t0;
  stop_attempt_us_ = micros() - t0_us;   // 확정 → 개별 정지 시도 끝 (침묵이면 이 시각에 침묵 시작)
  if (!all) {silent_ = true;}
}

bool bus_fault_latched()
{
  return latched_;
}

uint32_t bus_stop_attempt_ms()
{
  return stop_attempt_ms_;
}

bool bus_is_silent()
{
  return silent_;
}

bool bus_write_targets(const ntp::Cmd &c, bool host_ok)
{
  // 잠금 중: 아무 목표도 안 씀 (바퀴는 개별 정지로 0 이거나, 침묵으로 Watchdog 이 세움). 계속 "실패" 로 알림
  if (latched_) {return false;}
  if (!torque_) {
    write_fail_streak_ = 0;   // 쓰지 않은 동안은 "연속 실패" 가 아님 (옛 첫 실패 시각으로 확정되지 않게)
    return true;
  }
  sw_vel_info_.xel_count = 0;
  sw_pos_info_.xel_count = 0;
  for (uint8_t i = 0; i < n_ && i < c.n; ++i) {
    if (!(ok_ & (1u << i))) {continue;}
    const bool valid = host_ok && (c.valid & (1u << i));
    if (modes_[i] == ntp::MODE_VELOCITY) {
      // 바퀴: 끊겼거나 목표가 없으면 0 (멈춤)
      sw_vel_[i] = valid ? rads_to_vel_tick(c.target[i]) : 0;
      sw_vel_xels_[sw_vel_info_.xel_count].id = ids_[i];
      sw_vel_xels_[sw_vel_info_.xel_count].p_data = reinterpret_cast<uint8_t *>(&sw_vel_[i]);
      sw_vel_info_.xel_count++;
    } else if (valid) {
      // 위치 모터: 목표가 있을 때만 씀. 없으면 안 써서 마지막 목표에서 버팀
      sw_pos_[i] = rad_to_pos_tick(c.target[i]);
      sw_pos_xels_[sw_pos_info_.xel_count].id = ids_[i];
      sw_pos_xels_[sw_pos_info_.xel_count].p_data = reinterpret_cast<uint8_t *>(&sw_pos_[i]);
      sw_pos_info_.xel_count++;
    }
  }
  // (R5) 예전에는 반환값을 버렸음 → 읽기만 성공하면 comm_err 0 이라 목표 쓰기 실패를 아무도 몰랐음
  bool ok = true;
  const uint32_t tw = micros();
  if (sw_vel_info_.xel_count) {sw_vel_info_.is_info_changed = true; ok = dxl.syncWrite(&sw_vel_info_) && ok;}
  if (sw_pos_info_.xel_count) {sw_pos_info_.is_info_changed = true; ok = dxl.syncWrite(&sw_pos_info_) && ok;}
  if (sw_vel_info_.xel_count || sw_pos_info_.xel_count) {t_sync_write_.add(micros() - tw, ok);}
  if (ok) {
    write_fail_streak_ = 0;
    return true;
  }
  const uint32_t now = millis();
  if (write_fail_streak_ == 0) {first_fail_ms_ = now;}
  if (write_fail_streak_ < 255) {++write_fail_streak_;}
  if (write_fail_streak_ >= WRITE_FAIL_LIMIT || now - first_fail_ms_ >= WRITE_FAIL_CONFIRM_MS) {enter_fault();}
  return false;
}

uint16_t bus_read_states(ntp::State &s)
{
  s.n = n_;
  // 일단 "쓰는 모터"(켤 때 설정까지 성공한 것) 전부를 실패로 두고, 대답한 것만 지움
  // 켤 때부터 빠진 모터(require_all_motors=false 로 책상 시험)는 실패로 치지 않음 → 플러그인이 멈추지 않게
  uint16_t err = ok_;
  if (silent_) {read_ok_ = 0; return err;}   // 조용히 하는 중: 읽지도 않음 (모터 경비견이 돌게) → 전부 "못 읽음" 으로 알림
  const uint32_t tr = micros();
  const uint8_t got = dxl.syncRead(&sr_, SYNC_READ_TIMEOUT_MS);
  const uint32_t tr_us = micros() - tr;
  for (uint8_t k = 0; k < got; ++k) {
    // 대답 순서가 요청 순서와 다를 수 있어서 ID 로 찾음
    for (uint8_t i = 0; i < n_; ++i) {
      if (sr_.p_xels[k].id != ids_[i]) {continue;}
      err &= static_cast<uint16_t>(~(1u << i));
      const sr_data_t &d = sr_data_[i];
      s.pos[i] = (modes_[i] == ntp::MODE_VELOCITY)
        ? d.position * POS_RAD_PER_TICK                      // 바퀴: 여러 바퀴 누적 각도 그대로
        : (d.position - 2048) * POS_RAD_PER_TICK;            // 위치 모터: 2048 = 0 rad
      s.vel[i] = d.velocity * VEL_RADS_PER_TICK;
      s.cur[i] = d.current * CUR_A_PER_TICK;
    }
  }
  read_ok_ = static_cast<uint16_t>(ok_ & ~err);
  t_sync_read_.add(tr_us, err == 0);   // 실패 = 이번에 못 읽은 모터가 있었음
  return err;
}

void bus_timing_take(ntp::Diag &g)
{
  t_sync_read_.take(g.sync_read);
  t_sync_write_.take(g.sync_write);
  t_read_.take(g.single_read);
  t_write_.take(g.single_write);
  g.fault_stop_us = static_cast<uint16_t>(stop_attempt_us_ > 65535 ? 65535 : stop_attempt_us_);
  g.fault_flags = static_cast<uint8_t>((latched_ ? 1 : 0) | (silent_ ? 2 : 0));
  g.volt_min_dv = volt_min_dv_;
  g.volt_mean_dv = static_cast<uint8_t>(volt_n_ ? volt_sum_dv_ / volt_n_ : 0);
  volt_sum_dv_ = volt_n_ = 0;
  volt_min_dv_ = 0;
}
