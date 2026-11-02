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

}  // namespace planning
