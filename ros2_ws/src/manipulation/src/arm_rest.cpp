// arm_rest.cpp — 쉬는 팔 판단 (설명은 include/manipulation/arm_rest.hpp)

#include "manipulation/arm_rest.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace manipulation
{

bool ArmRest::update(double now_s, const ArmRestInput & in, const ArmRestConfig & c, std::string * why)
{
  const char * no = nullptr;
  if (!c.enabled) {no = "arm_rest_torque_off 꺼짐";}
  else if (in.job) {no = "팔 작업";}
  else if (!in.drive) {no = "바퀴 차례 아님";}
  else if (in.holding) {no = "퍽을 쥐었을 수 있음 (carrying · 그리퍼 실측)";}
  else if (!in.hw_ok) {no = "하드웨어 이상";}
  else if (!in.folded) {no = "접힘 확인 안 됨";}
  if (no) {
    quiet_since_ = -1.0;
  } else if (quiet_since_ < 0.0) {
    quiet_since_ = now_s;
  }
  const bool want = !no && now_s - quiet_since_ >= std::max(0.0, c.after_s);
  if (why) {
    if (no) {
      *why = no;
    } else if (!want) {
      char b[64];
      std::snprintf(b, sizeof(b), "조건 이어짐 %.1f / %.1f s", now_s - quiet_since_, c.after_s);
      *why = b;
    } else {
      why->clear();
    }
  }
  if (want && !want_) {
    waking_ = false;
    // 보드가 쉬는 팔을 알면 (상태가 옴) 곧 꺼짐 → 지금부터 처질 수 있음. 모르면 (보드 상태 없음) 끈 적 없음으로 둠
    if (board_alive(now_s, c)) {slipped_ = true;}
  } else if (!want && want_) {
    start_wake(now_s);
  }
  want_ = want;
  return want_;
}

void ArmRest::wake(double now_s)
{
  quiet_since_ = -1.0;
  if (want_) {
    want_ = false;
    start_wake(now_s);
  }
}

void ArmRest::start_wake(double now_s)
{
  // 꺼졌을 수 있을 때만 켜짐을 기다림 (살아 있는 보드에 끄기를 요청했거나, 요청 중 보드가 꺼짐을 알림 = slipped_)
  if (slipped_) {
    waking_ = true;
    wake_t_ = now_s;
  }
}

void ArmRest::on_state(double now_s, bool torque_on)
{
  have_state_ = true;
  state_on_ = torque_on;
  state_t_ = now_s;
  // 요청해서 실제로 꺼짐 → 켜질 때까지 처질 수 있음. 요청 안 한 꺼짐 (켤 때 첫 주기 · 고장) 은 쉬는 팔과 상관없음 → slipped_ 안 바꿈
  if (!torque_on && want_) {slipped_ = true;}
  if (torque_on && waking_ && now_s >= wake_t_) {waking_ = false;}
}

bool ArmRest::arm_ok(double now_s, const ArmRestConfig & c, bool * timed_out)
{
  if (timed_out) {*timed_out = false;}
  if (!waking_) {return true;}
  if (now_s - wake_t_ >= c.wake_timeout_s) {
    waking_ = false;
    if (timed_out) {*timed_out = true;}
    return true;
  }
  return false;
}

Reseat reseat_choice(const std::vector<double> & q, const std::vector<double> & home, double min_rad, double max_rad,
  double min_s, double speed_rad_s)
{
  Reseat r;
  if (q.empty() || q.size() != home.size()) {return r;}
  for (std::size_t j = 0; j < q.size(); ++j) {
    const double d = std::fabs(q[j] - home[j]);
    if (!std::isfinite(d)) {return Reseat{};}
    if (d > r.slip_rad) {r.slip_rad = d; r.worst = static_cast<int>(j);}
  }
  if (!(r.slip_rad > min_rad)) {return r;}
  if (r.slip_rad <= max_rad) {
    r.kind = Reseat::Kind::SHORT;
    r.time_s = std::max(min_s, speed_rad_s > 0.0 ? r.slip_rad / speed_rad_s : 0.0);
  } else {
    r.kind = Reseat::Kind::CHECKED;
  }
  return r;
}

std::string slip_text(const std::vector<double> & q, const std::vector<double> & home)
{
  std::string out;
  for (std::size_t j = 0; j < q.size() && j < home.size(); ++j) {
    char b[32];
    const double d = (q[j] - home[j]) * 180.0 / M_PI;
    if (std::isfinite(d)) {std::snprintf(b, sizeof(b), "J%zu %+.1f°", j + 1, d);}
    else {std::snprintf(b, sizeof(b), "J%zu ?", j + 1);}
    out += (out.empty() ? "" : " · ") + std::string(b);
  }
  return out;
}

}  // namespace manipulation
