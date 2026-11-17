// steer_recover.hpp — 조향 모터 위치 카운터 자동 복구 (켤 때, 모터를 움직이지 않음) — ROS 없는 순수 C++
//   토크 OFF 동안 조향이 눈금 경계를 넘어 돌면 XM430 은 여러 바퀴로 이어 셈 (−522 같은 0–4095 밖 값)
//   → 펌웨어가 EEPROM 한계 밖이라 토크를 안 켬 (TF_RANGE) → 스워브 컨트롤러가 안 켜짐.
//   XM430 은 운전 모드가 위치 모드로 "바뀌면" 한 바퀴 안 값으로 다시 셈 (같은 모드를 다시 쓰면 안 셈)
//   → 그 모터만 CONFIG 에서 속도 모드로 한 번 보냈다가 원래 CONFIG 를 다시 보냄.
// 여기 = 판단 (어느 관절 · 다시 세면 몇 눈금 · 한계 안인지). 실제 순서는 opencr_hardware.cpp on_configure.

#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <string>
#include <utility>
#include <vector>

#include "hardware/opencr_link.hpp"

namespace hardware
{

constexpr double STEER_TICK_RAD = 2.0 * M_PI / 4096.0;

// 관절값 → 펌웨어 눈금 (위치 모드: 모터 rad / k + 2048). 여러 바퀴 값이면 0–4095 밖
inline long motor_tick(const JointSpec & j, double q) {return std::lround(joint_to_motor(j, q) / STEER_TICK_RAD) + 2048;}
inline double tick_to_joint(const JointSpec & j, long tick) {return motor_to_joint(j, (tick - 2048) * STEER_TICK_RAD);}
// 모터가 다시 센 뒤 보고할 한 바퀴 안 값 (같은 실제 자리)
inline long single_turn_tick(long tick)
{
  const long m = tick % 4096;
  return m < 0 ? m + 4096 : m;
}

struct SteerRecoverItem
{
  std::size_t idx = 0;
  long tick = 0;          // 지금 눈금 (한계 밖)
  long after_tick = 0;    // 다시 세면 될 눈금 (예상)
  bool fixable = false;   // after_tick 이 한계 안 → 카운터 다시 세기로 고침. false = 한 바퀴 안에서도 한계 밖 (손으로)
};

struct SteerRecoverPlan
{
  uint16_t mask = 0;      // 다시 셀 관절 (비트 i) — fixable 만
  std::vector<SteerRecoverItem> items;   // 한계 밖 관절 전부
};

// 위치 모드 · 한계를 아는 · 지금 측정이 있는 관절 중 EEPROM 한계 밖인 것. q = 관절값 (NaN = 모름 → 건너뜀)
//   한계 안이면 아무것도 안 넣음 → 평소엔 빈 계획 (동작 그대로)
inline SteerRecoverPlan plan_steer_recover(
  const std::vector<JointSpec> & joints, const std::vector<std::pair<int32_t, int32_t>> & lim,
  const std::vector<double> & q)
{
  SteerRecoverPlan p;
  for (std::size_t i = 0; i < joints.size() && i < lim.size() && i < q.size() && i < 16; ++i) {
    if (joints[i].mode != pkt::MODE_POSITION || lim[i].first == pkt::TF_NO_LIMIT || !std::isfinite(q[i])) {continue;}
    const long lo = lim[i].first, hi = lim[i].second;
    SteerRecoverItem it;
    it.idx = i;
    it.tick = motor_tick(joints[i], q[i]);
    if (it.tick >= lo && it.tick <= hi) {continue;}
    it.after_tick = single_turn_tick(it.tick);
    it.fixable = it.after_tick >= lo && it.after_tick <= hi;
    if (it.fixable) {p.mask = static_cast<uint16_t>(p.mask | (1u << i));}
    p.items.push_back(it);
  }
  return p;
}

// 로그 한 줄: "steer_rr_joint 지금 -522 눈금 (+271°) → 다시 세면 3574 눈금 (-89°), 한계 0–4095 안"
inline std::string steer_recover_text(
  const std::vector<JointSpec> & joints, const std::vector<std::pair<int32_t, int32_t>> & lim, const SteerRecoverItem & it)
{
  char b[256];
  const auto & j = joints[it.idx];
  std::snprintf(b, sizeof(b), "%s (모터 %d) 지금 %ld 눈금 (%+.0f°) → 한 바퀴 안으로 다시 세면 %ld 눈금 (%+.0f°), 한계 %d–%d %s",
    j.name.c_str(), j.id, it.tick, tick_to_joint(j, it.tick) * 180.0 / M_PI, it.after_tick,
    tick_to_joint(j, it.after_tick) * 180.0 / M_PI, lim[it.idx].first, lim[it.idx].second, it.fixable ? "안" : "밖 (다시 세도 못 고침)");
  return b;
}

// 다시 센 뒤 확인: 예상 눈금과 얼마나 다른가 (토크가 꺼져 있어 거의 0 이어야 함. 크면 Homing Offset · 손 · 다시 안 셈)
inline long steer_recover_error_ticks(long expected_tick, long measured_tick)
{
  long d = (measured_tick - expected_tick) % 4096;
  if (d > 2048) {d -= 4096;}
  if (d < -2048) {d += 4096;}
  return d;
}

}  // namespace hardware
