// health.cpp — 실행 조건 감시 구현, 설명은 health.hpp

#include "planning/health.hpp"

#include <cmath>

namespace planning
{

Health ConditionMonitor::update(bool bad, double now)
{
  if (faulted_) {return Health::Fault;}

  if (bad) {
    good_since_ = -1.0;
    if (!active_) {
      // 자동 해제를 다 쓴 뒤 또 나빠짐 → 반복되는 문제로 봄
      if (max_recoveries_ >= 0 && recoveries_ >= max_recoveries_) {
        faulted_ = true;
        return Health::Fault;
      }
      active_ = true;
      bad_since_ = now;
    }
  } else {
    if (!active_) {return Health::Ok;}
    if (good_since_ < 0.0) {good_since_ = now;}
    if (now - good_since_ >= recover_s_) {
      active_ = false;
      good_since_ = -1.0;
      ++recoveries_;
      return Health::Ok;
    }
  }
  // 켜진 채 (회복 확인 중 포함) 제한 시간 넘음
  if (now - bad_since_ > fault_after_s_) {
    faulted_ = true;
    return Health::Fault;
  }
  return Health::Waiting;
}

void ConditionMonitor::reset()
{
  active_ = false;
  faulted_ = false;
  good_since_ = -1.0;
  recoveries_ = 0;
}

int DetectionGap::update(bool bad, double now)
{
  if (bad) {
    good_since_ = -1.0;
    if (active_) {return 0;}
    active_ = true;
    start_ = now;
    ++count_;
    return 1;
  }
  if (!active_) {return 0;}
  if (good_since_ < 0.0) {good_since_ = now;}
  if (now - good_since_ < recover_s_) {return 0;}
  active_ = false;
  end_ = good_since_;
  good_since_ = -1.0;
  return -1;
}

void DetectionGap::reset()
{
  active_ = false;
  start_ = end_ = good_since_ = -1.0;
  count_ = 0;
}

GapAction gap_action(const DetectionGap & g, double look_since, int redo_used, int redo_max, bool redo_allowed)
{
  if (look_since < 0.0 || !g.overlaps(look_since)) {return GapAction::Proceed;}
  if (g.active()) {return GapAction::Wait;}
  return redo_allowed && redo_used < redo_max ? GapAction::Redo : GapAction::Proceed;
}

std::optional<PoseStd> covariance_std(const double * c)
{
  const double vx = c[0];     // (x, x)
  const double vy = c[7];     // (y, y)
  const double vyaw = c[35];  // (yaw, yaw)
  for (const double v : {vx, vy, vyaw}) {
    if (!std::isfinite(v) || v < 0.0) {return std::nullopt;}
  }
  return PoseStd{std::sqrt(vx), std::sqrt(vy), std::sqrt(vyaw)};
}

namespace
{
bool loc_health_fresh(const LocHealth & h, double now, double timeout_s)
{
  return h.level >= 0 && h.received_s >= 0.0 && now - h.received_s <= timeout_s;
}
}  // namespace

bool loc_health_lost(const LocHealth & h, double now, double timeout_s)
{
  return !loc_health_fresh(h, now, timeout_s) || h.level >= 2;
}

bool loc_health_ready(const LocHealth & h, double now, double timeout_s)
{
  return loc_health_fresh(h, now, timeout_s) && h.level == 0;
}

bool arm_goal_uses_map(uint8_t task, Target target)
{
  if (task == kArmHome || is_look_task(task) || task == kArmLookBarcode || task == kArmLookRack || task == kArmStore ||
    task == kArmRetrieve)
  {
    return false;
  }
  return target == Target::Puck || target == Target::Custom;
}

bool arm_start_blocked(uint8_t task, Target target, bool detection_lost, bool localization_lost)
{
  if (task == kArmPick && (detection_lost || localization_lost)) {return true;}
  return localization_lost && arm_goal_uses_map(task, target);
}

const char * loc_mode_name(LocMode m)
{
  switch (m) {
    case LocMode::Ok: return "정상";
    case LocMode::Wait: return "대기";
    case LocMode::Backoff: return "물러나기";
    case LocMode::Degraded: return "조건부 계속";
  }
  return "?";
}

bool LocRecovery::update(bool lost, bool stable, bool can_backoff, double now)
{
  if (mode_ == LocMode::Ok) {
    if (!lost) {return false;}
    mode_ = LocMode::Wait;
    lost_since_ = now;
    wait_since_ = now;
    good_since_ = -1.0;
    backoffs_ = 0;
    return false;
  }
  // 상실 중 (Wait · Backoff · Degraded): 정상이 ok_hold_s 이어지면 Ok
  if (!lost) {
    if (good_since_ < 0.0) {good_since_ = now;}
    if (now - good_since_ >= c_.ok_hold_s) {
      mode_ = LocMode::Ok;
      good_since_ = -1.0;
      return false;
    }
  } else {
    good_since_ = -1.0;
  }
  if (mode_ != LocMode::Wait) {return false;}   // Backoff 는 backoff_done 까지, Degraded 는 정상 회복까지 유지
  const double lost_for = now - lost_since_;
  if (lost_for >= c_.force_s ||
    (lost_for >= c_.degrade_s && (stable || backoffs_ >= c_.max_backoff)))
  {
    mode_ = LocMode::Degraded;
    return false;
  }
  if (lost && can_backoff && backoffs_ < c_.max_backoff && now - wait_since_ >= c_.wait_s) {
    mode_ = LocMode::Backoff;
    ++backoffs_;
    return true;
  }
  return false;
}

void LocRecovery::backoff_done(double now)
{
  if (mode_ != LocMode::Backoff) {return;}
  mode_ = LocMode::Wait;
  wait_since_ = now;
}

void LocRecovery::reset()
{
  mode_ = LocMode::Ok;
  good_since_ = -1.0;
  backoffs_ = 0;
}

}  // namespace planning
