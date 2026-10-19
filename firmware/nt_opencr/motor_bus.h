// motor_bus.h — 다이나믹셀 14개를 한 번에 읽고 쓰기
// =====================================================================
// 모터 n 개를 Sync Read / Sync Write 로 한 번에 (하나씩 읽고 쓰면 14개에서 10 ms 주기를 못 맞춤).
// 어떤 ID 가 어떤 모드인지는 펌웨어에 적어 두지 않고, 켤 때 Jetson 이 CONFIG 상자로 알려 준다
//   → hardware_map.yaml 하나만 고치면 펌웨어를 다시 올릴 필요 없음.
//
// 단위 변환 (XM430 e-Manual)
//   위치 : 눈금 0~4095 = 한 바퀴, 2048 = 0 rad            → rad = (눈금 − 2048) × 2π/4096
//   속도 : 1 눈금 = 0.229 rpm                            → rad/s = 눈금 × 0.229 × 2π/60
//   전류 : 1 눈금 = 2.69 mA                              → A = 눈금 × 0.00269

#pragma once

#include <Arduino.h>

#include "nt_packet.h"

bool bus_begin(uint32_t baud);

// CONFIG 적용: 토크 끄고 → 모드 · 속도/가속 한계 · 그리퍼 전류 · 응답 지연 0 · Bus Watchdog 쓰기.
// 돌려주는 값 = ping 에 대답하고 **설정 쓰기까지 전부 성공한** 모터 비트 (CONFIG_ACK 로 보냄)
// 고장 잠금(아래)도 여기서만 풀림. ⚠ 모든 모터 토크를 먼저 끄므로 팔이 들려 있으면 처짐 → 팔을 받친 뒤 보낼 것
uint16_t bus_configure(const ntp::Config &c);

// 토크 켜기/끄기. 켤 때는 위치 모터의 목표를 "지금 위치" 로 먼저 맞춰서 팔이 휙 움직이지 않게
void bus_set_torque(bool on);
// 실제로 토크가 켜졌나 → STATE 의 ST_TORQUE 비트
//   = 위의 쓰기가 전부 성공했고 + 운전 중 되읽은 Torque Enable 이 꺼진 모터가 없음 (2026-10-16 R5)
//   예전에는 "마지막 쓰기 성공" 만 기억해서, 운전 중 모터가 스스로 토크를 끄거나(과부하 shutdown) 재부팅돼도 몰랐음
bool bus_torque_is_on();
// 모터 상태 되읽기 한 칸 (매 주기 1 모터씩 돌아가며: Hardware Error Status · 입력 전압(배터리) · Torque Enable). 14 개면 0.14 s 에 한 바퀴
//   한 번에 모두 읽으면 버스 시간이 늘어서 하나씩. 통신 실패는 bus_read_states 의 comm_err 가 이미 잡음
void bus_health_step();
// Hardware Error Status 가 0 이 아닌 모터 비트 → ST_HW_ERROR
uint16_t bus_hw_error_mask();

// 목표 쓰기 (Sync Write 2번: 바퀴 속도, 위치 모터 위치).
// host_ok = false (Jetson 끊김) 이면 바퀴만 0, 위치 모터는 마지막 목표 그대로 = 그 자리에서 버팀 (팔이 떨어지지 않게)
// 돌려주는 값: Sync Write 전송이 전부 성공했나 (false → ST_WRITE_FAIL). ⚠ 전송 성공 ≠ 각 모터가 목표를 받아들임
//   (Sync Write 는 모터가 대답하지 않음) → 실제 반영은 상태 읽기(위치가 따라오나)와 위 되읽기로 봄
//
// 고장 잠금 (2026-10-16 재검증 R6 · R7) — 자세한 숫자는 motor_bus.cpp 맨 위
//   Sync Write 가 연속 3 번 실패하거나 첫 실패 뒤 20 ms 지나 또 실패 → 고장 확정 → **잠금**
//     ① 바퀴에 하나씩 속도 0, 이어서 조향 · 팔은 목표 = 지금 위치 (그리퍼는 그대로 — 쥔 힘 유지)
//        전체 20 ms 예산, 각 3 ms 응답 대기
//     ② 하나라도 확인 못 하면 버스 침묵 (bus_is_silent) → 마지막 패킷부터 Bus Watchdog(0.3 s) 이 모든 모터를 세움
//   잠금 중에는 통신이 돌아와도 목표 · 토크를 바꾸지 않음 (예전: 회복되면 원래 속도로 재출발했음)
//   잠금은 CONFIG(bus_configure) 로만 풀림 — 펌웨어가 스스로 풀지 않음
//   ⚠ "3 번" 은 판정 횟수일 뿐 정지 시간 보장이 아님. 정지 시간은 실제 버스에서 따로 잼 (docs/hardware.md)
bool bus_write_targets(const ntp::Cmd &c, bool host_ok);
bool bus_fault_latched();      // → STATE 의 ST_FAULT_LATCHED
bool bus_is_silent();
uint32_t bus_stop_attempt_ms();   // 마지막 개별 정지 시도에 걸린 시간 [ms] (시험용)

// 보드 안 시간 측정 (DIAG, 2026-10-16) — 지금까지 모은 Sync Read · Sync Write · 개별 읽기/쓰기 시간과
//   마지막 고장 정지 시간을 g 에 채우고 0 부터 다시 모음 (g.loop · resync · ms · window_ms 는 .ino 가 채움)
void bus_timing_take(ntp::Diag &g);

// 시간 모으기 한 칸 (횟수 · 실패 · 평균 · 최대, 단위 µs)
struct TimeAcc
{
  uint32_t n = 0, fail = 0, sum = 0, max = 0;
  void add(uint32_t us, bool ok)
  {
    ++n;
    if (!ok) {++fail;}
    sum += us;
    if (us > max) {max = us;}
  }
  void take(ntp::TimeStat &t)
  {
    t.n = static_cast<uint16_t>(n > 65535 ? 65535 : n);
    t.fail = static_cast<uint16_t>(fail > 65535 ? 65535 : fail);
    const uint32_t mean = n ? sum / n : 0;
    t.mean_us = static_cast<uint16_t>(mean > 65535 ? 65535 : mean);
    t.max_us = static_cast<uint16_t>(max > 65535 ? 65535 : max);
    n = fail = sum = max = 0;
  }
};

// 상태 읽기 (Sync Read 1번: 전류 · 속도 · 위치, 주소 126 부터 10 바이트). 돌려주는 값 = 읽기 실패한 모터 비트
uint16_t bus_read_states(ntp::State &s);
