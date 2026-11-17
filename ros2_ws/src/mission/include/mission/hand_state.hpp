// hand_state.hpp — 그리퍼 실측 (/joint_states gripper_left_joint) 으로 손 안 퍽 판단 + 재고 불확실 스스로 정합 고르기
//
// 역할
//   - control_master grasp_guard 와 같은 식으로 그리퍼 q [m] → 손가락 사이 [mm] = 2 · (inner0 + q) · 1000 + gap_bias
//     (control.yaml grasp_gap_bias_mm 7.4, FingerTip inner0 0.0091, 쥔 네모 34.8 mm ↔ q 약 0.0046)
//   - 판단: q ≤ empty_q (빈손 닫힘, control grasp_empty_m 0.0003) 또는 퍽 폭 + tol 보다 훨씬 벌어짐 (open_extra_mm) = 빈손 (EMPTY)
//           퍽 폭 [원기둥, 네모] ± tol 안 = 쥠 (HOLDING), 그 사이 = 모름 (UNKNOWN → 스스로 정합 안 함)
//   - 표본이 stable_s 동안 모두 같은 판단 · 마지막 표본이 fresh_s 안 일 때만 판단 (아니면 UNKNOWN)
//   - 재고 불확실 (원인 팔 작업) × 손 상태 → planning DoubtFix (Before · Done · Removed) 하나, 못 정하면 nullopt
// 실행
//   - 헤더 전용 (mission_manager · test_transfer)
#pragma once

#include <cmath>
#include <cstdio>
#include <deque>
#include <optional>
#include <string>

#include "planning/inventory_policy.hpp"
#include "planning/types.hpp"

namespace mission
{

enum class HandState { Unknown, Empty, Holding };

inline const char * hand_state_name(HandState s)
{
  switch (s) {
    case HandState::Empty: return "빈손";
    case HandState::Holding: return "쥠";
    default: return "모름";
  }
}

struct HandConfig
{
  double inner0_m = 0.0091;        // q = 0 일 때 손가락 안쪽 면 반 간격 (control FingerTip inner0)
  double gap_bias_mm = 7.4;        // control grasp_gap_bias_mm
  double empty_q_m = 0.0003;       // 이하 = 빈손 닫힘 (control.yaml grasp_empty_m)
  double cylinder_mm = 33.5;       // control.yaml grasp_width_cylinder_mm
  double square_mm = 34.8;         // control grasp_width_square_mm
  double tol_mm = 5.0;             // control grasp_width_tol_mm
  double open_extra_mm = 5.0;      // 퍽 폭 + tol 보다 이만큼 더 벌어지면 = 열림 (빈손)
  double stable_s = 0.5;           // 이 시간 동안 같은 판단이어야 함
  double fresh_s = 0.3;            // 마지막 표본 나이 한도
};

struct HandSample
{
  double t = 0.0;   // 받은 시각 [s]
  double q = NAN;   // 그리퍼 [m]
};

inline double hand_gap_mm(double q, const HandConfig & c)
{
  if (!std::isfinite(q)) {return NAN;}
  return 2.0 * (c.inner0_m + q) * 1000.0 + c.gap_bias_mm;
}

inline HandState classify_hand(double q, const HandConfig & c)
{
  if (!std::isfinite(q)) {return HandState::Unknown;}
  if (q <= c.empty_q_m) {return HandState::Empty;}
  const double g = hand_gap_mm(q, c);
  const double lo = std::fmin(c.cylinder_mm, c.square_mm) - c.tol_mm, hi = std::fmax(c.cylinder_mm, c.square_mm) + c.tol_mm;
  if (g >= lo && g <= hi) {return HandState::Holding;}
  if (g > hi + c.open_extra_mm) {return HandState::Empty;}
  return HandState::Unknown;
}

struct HandReading
{
  HandState state = HandState::Unknown;
  double q = NAN;        // 마지막 표본
  double gap_mm = NAN;
  double age_s = NAN;    // 마지막 표본 나이
  double span_s = 0.0;   // 같은 판단으로 이어진 시간
  int n = 0;             // 그 표본 수
  std::string text;      // 로그 근거
};

// samples: 받은 순서 (오래된 것 먼저). 마지막 표본부터 거꾸로 같은 판단이 stable_s 이상 이어지고 마지막이 fresh_s 안이면 그 판단
inline HandReading read_hand(const std::deque<HandSample> & samples, double now, const HandConfig & c)
{
  HandReading r;
  char buf[200];
  if (samples.empty()) {
    r.text = "그리퍼 실측 없음";
    return r;
  }
  const HandSample & last = samples.back();
  r.q = last.q;
  r.gap_mm = hand_gap_mm(last.q, c);
  r.age_s = now - last.t;
  const HandState s = classify_hand(last.q, c);
  double first_t = last.t;
  for (auto it = samples.rbegin(); it != samples.rend(); ++it) {
    if (classify_hand(it->q, c) != s) {break;}
    first_t = it->t;
    ++r.n;
  }
  r.span_s = last.t - first_t;
  const bool fresh = r.age_s >= -0.05 && r.age_s <= c.fresh_s;
  const bool stable = r.span_s + 1e-9 >= c.stable_s;
  r.state = (fresh && stable) ? s : HandState::Unknown;
  std::snprintf(buf, sizeof(buf), "그리퍼 %.4f m → 손가락 사이 %.1f mm = %s (같은 판단 %d 장 %.2f s / 필요 %.2f, 마지막 %.2f s 전 / 한도 %.2f)%s",
    last.q, r.gap_mm, hand_state_name(s), r.n, r.span_s, c.stable_s, r.age_s, c.fresh_s,
    r.state == HandState::Unknown && s != HandState::Unknown ? " → 덜 안정 · 오래됨 = 모름" : "");
  r.text = buf;
  return r;
}

// 재고 불확실 원인 팔 작업 · 손 상태 · 놓기 (release) 단계를 봤나 → 정합. why 에 이유
//   PICK · RETRIEVE : 쥠 = Done (그리퍼에), 빈손 = Before (박스 · 칸에 그대로)
//   STORE           : 쥠 = Before (손에 그대로), 빈손 + 놓기 봄 = Done (그 칸), 빈손 + 놓기 못 봄 = Removed (어디 떨어졌는지 모름)
//   PLACE           : 쥠 = Before, 빈손 = Done (놓았다고 봄, 재검증이 다시 봄)
//   LOOK_RACK       : 쥠 = Before (전환 없음), 빈손 = Removed
//   모름 = nullopt (사람)
inline std::optional<planning::DoubtFix> choose_doubt_fix(uint8_t arm_task, HandState hand, bool release_seen, std::string * why)
{
  using planning::DoubtFix;
  auto say = [why](const char * w) {if (why) {*why = w;}};
  if (hand == HandState::Unknown) {
    say("손 상태 모름 → 스스로 정합 안 함");
    return std::nullopt;
  }
  const bool held = hand == HandState::Holding;
  switch (arm_task) {
    case planning::kArmPick:
      say(held ? "PICK 뒤 쥠 → 퍽은 그리퍼에 (done)" : "PICK 뒤 빈손 → 퍽은 박스 자리에 그대로 (before)");
      return held ? DoubtFix::Done : DoubtFix::Before;
    case planning::kArmRetrieve:
      say(held ? "RETRIEVE 뒤 쥠 → 퍽은 그리퍼에 (done)" : "RETRIEVE 뒤 빈손 → 퍽은 칸에 그대로 (before)");
      return held ? DoubtFix::Done : DoubtFix::Before;
    case planning::kArmStore:
      if (held) {say("STORE 뒤 쥠 → 퍽은 손에 그대로 (before)"); return DoubtFix::Before;}
      if (release_seen) {say("STORE 뒤 빈손 · 놓기 단계 봄 → 퍽은 그 칸에 (done)"); return DoubtFix::Done;}
      say("STORE 뒤 빈손 · 놓기 단계 못 봄 → 퍽 자리 모름 (removed, 다음 LOOK 이 다시 찾음)");
      return DoubtFix::Removed;
    case planning::kArmPlace:
      say(held ? "PLACE 뒤 쥠 → 퍽은 손에 그대로 (before)" : "PLACE 뒤 빈손 → 놓았다고 봄 (done, 재검증이 다시 봄)");
      return held ? DoubtFix::Before : DoubtFix::Done;
    case planning::kArmLookRack:
      say(held ? "LOOK_RACK 뒤 쥠 → 그대로 (before)" : "LOOK_RACK 뒤 빈손 → 퍽 자리 모름 (removed)");
      return held ? DoubtFix::Before : DoubtFix::Removed;
    default:
      say("재고 작업 아님 → 스스로 정합 안 함");
      return std::nullopt;
  }
}

}  // namespace mission
