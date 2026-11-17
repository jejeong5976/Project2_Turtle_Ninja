// nav_guard.hpp — 주행 노드 (rl_nav) 덧붙임: 막히면 서서 기다리기 · 제자리 회전 반전 제한 (ROS 없음, 시험 대상)
//
// 역할
//   - ahead_blocked: 경로 앞 ahead m 안 (지금 호 길이 s 부터) 경로 중심선에서 half 안에 든 스캔 점
//   - BlockWait: 막힘 (경로 없음 · PolygonStop · 앞 점) 이면 0 속도로 최대 wait_s 기다림, recheck_s 마다 다시 봄
//                풀리면 계속, 안 풀리면 포기 → 그 막힘이 recheck_s 넘게 계속 풀릴 때까지 다시 안 기다림
//   - RevLimit: 제자리 회전 (병진 명령 거의 0) 중 wz 명령 부호가 window_s 안에서 max_rev 번 넘게 바뀌면
//               hold_s 동안 0 → lock_s 동안 그 뒤 처음 도는 방향만 허용 (반대 wz = 0, 병진은 그대로)
// note
//   - 둘 다 기본 꺼짐 (wait_s 0 · max_rev 0 이면 명령을 안 바꿈). 사람이 막으면 제자리 회전 반전이 쌓여
//     odom 회전 오차로 지도 맞음이 떨어질 수 있음 (LOCALIZATION_LOST)
//   - 반전 = |wz| ≥ kWzMin (노드 '명령 반전' 기록과 같은 문턱) 인 명령의 부호가 바뀜. 직전 한 방향 회전 (odom) 이
//     min_swing 보다 작으면 안 셈 (도착 직전 yaw 미세 조정)

#pragma once

#include <array>
#include <cmath>
#include <deque>
#include <string>
#include <vector>

#include "navigation/rl_core.hpp"

namespace navigation::guard
{

using rl::RlPath;
using rl::Vec2;

constexpr double kWzMin = 0.05;                         // 회전 명령으로 보는 |wz| [rad/s] (노드 반전 기록과 같음)

struct Ahead
{
  int n = 0;                                            // 걸린 점 수
  double at = 0.0;                                      // 걸린 점 중 로봇 중심에서 가장 가까운 거리 [m]
};

// 경로 앞 막힘: s + step, s + 2 step, … , min(s + ahead, L) 경로 점에서 half 안 · 로봇 중심 (경로 s 점) 보다 앞
//   · 로봇 중심에서 r_min 보다 먼 점. 점 = 경로와 같은 좌표계. r_min 안은 collision_monitor (PolygonStop) 몫
inline Ahead ahead_blocked(const RlPath & path, double s, const std::vector<Vec2> & pts, double ahead, double half,
  double r_min, double step = 0.05)
{
  Ahead out;
  if (ahead <= 0 || half <= 0 || pts.empty()) {return out;}
  const Vec2 p0 = path.point_at(s);
  const double s_end = std::min(s + ahead, path.length());
  std::vector<char> hit(pts.size(), 0);
  for (double sk = s + step; sk <= s_end + 1e-9; sk += step) {
    const Vec2 pk = path.point_at(sk);
    const double fx = pk.x - p0.x, fy = pk.y - p0.y;
    for (size_t i = 0; i < pts.size(); ++i) {
      if (hit[i]) {continue;}
      const double rx = pts[i].x - p0.x, ry = pts[i].y - p0.y;
      if (std::hypot(rx, ry) <= r_min || rx * fx + ry * fy <= 0) {continue;}
      if (std::hypot(pts[i].x - pk.x, pts[i].y - pk.y) < half) {
        hit[i] = 1;
        const double r = std::hypot(rx, ry);
        out.at = out.n++ == 0 ? r : std::min(out.at, r);
      }
    }
  }
  return out;
}

// 막힘 기다리기. step 을 제어 주기마다 부름 (blocked = 이번 주기 막힘)
class BlockWait
{
public:
  enum class Ev {None, Start, Clear, GiveUp};
  struct Out
  {
    bool hold = false;                                  // true = 0 속도
    Ev ev = Ev::None;
    double waited = 0.0;                                // Clear · GiveUp 때 기다린 시간 [s]
  };

  explicit BlockWait(double wait_s = 0.0, double recheck_s = 0.5)
  : wait_s_(wait_s), recheck_s_(recheck_s) {}

  bool enabled() const {return wait_s_ > 0;}
  bool waiting() const {return waiting_;}

  Out step(double t, bool blocked)
  {
    Out o;
    if (!enabled()) {return o;}
    if (!waiting_) {
      if (!blocked) {                                   // 포기 뒤엔 recheck_s 넘게 계속 풀려야 다시 기다림 (점 수 · 상태 깜빡임)
        if (spent_ && clear_t_ < 0) {clear_t_ = t;}
        if (spent_ && t - clear_t_ + 1e-9 >= recheck_s_) {spent_ = false;}
        return o;
      }
      clear_t_ = -1.0;
      if (spent_) {return o;}                           // 이번 막힘은 이미 기다렸음 → 그대로 진행
      waiting_ = true;
      t0_ = t;
      t_check_ = t + recheck_s_;
      o.hold = true;
      o.ev = Ev::Start;
      return o;
    }
    o.hold = true;
    if (t + 1e-9 < t_check_) {return o;}                // recheck_s 마다만 다시 봄
    o.waited = t - t0_;
    if (!blocked) {
      waiting_ = false;
      o.hold = false;
      o.ev = Ev::Clear;
    } else if (t - t0_ + 1e-9 >= wait_s_) {
      waiting_ = false;
      spent_ = true;
      clear_t_ = -1.0;
      o.hold = false;
      o.ev = Ev::GiveUp;
    } else {
      t_check_ += recheck_s_;
    }
    return o;
  }

private:
  double wait_s_, recheck_s_;
  bool waiting_ = false, spent_ = false;
  double t0_ = 0.0, t_check_ = 0.0, clear_t_ = -1.0;
};

// 제자리 회전 반전 제한. filter 를 제어 주기마다 부름 (보낼 명령을 돌려줌)
class RevLimit
{
public:
  enum class Ev {None, Hold, Lock, Unlock};
  struct Cfg
  {
    int max_rev = 0;                                    // 0 = 끔
    double window_s = 10.0, hold_s = 1.0, lock_s = 5.0;
    double lin_eps = 0.05;                              // 제자리 = 병진 명령 평균 < lin_eps [m/s]
    double min_swing = 5.0 * M_PI / 180.0;              // 이만큼 돈 뒤 반전만 셈 [rad]
  };

  RevLimit() = default;
  explicit RevLimit(const Cfg & c) : c_(c) {}

  bool enabled() const {return c_.max_rev > 0;}
  int sign() const {return lock_sign_;}
  double swing_deg() const {return last_swing_ * 180.0 / M_PI;}
  int count() const {return static_cast<int>(revs_.size());}

  // cmd = 보낼 명령 (정책 · 한계 뒤), odom_wz = 측정 각속도, free = 제한 안 함 · 안 셈 (목표 근처: 마지막 yaw 맞춤은 정책 몫)
  std::array<double, 3> filter(double t, std::array<double, 3> cmd, double odom_wz, double dt, bool free, Ev * ev)
  {
    *ev = Ev::None;
    if (!enabled()) {return cmd;}
    if (mode_ == Mode::Hold) {
      if (t + 1e-9 < until_) {return {0.0, 0.0, 0.0};}
      mode_ = Mode::Lock;
      lock_sign_ = 0;
      until_ = t + c_.lock_s;
      *ev = Ev::Lock;
    }
    if (mode_ == Mode::Lock) {
      if (free || t + 1e-9 >= until_) {
        mode_ = Mode::Free;
        lock_sign_ = 0;
        reset_swing();
        revs_.clear();
        *ev = Ev::Unlock;
      } else {
        if (lock_sign_ == 0 && std::abs(cmd[2]) >= kWzMin) {lock_sign_ = cmd[2] > 0 ? 1 : -1;}
        if (lock_sign_ != 0 && cmd[2] * lock_sign_ < 0) {cmd[2] = 0.0;}   // 반대로 안 돎 (병진 그대로)
        return cmd;
      }
    }
    // Free: 반전 셈 (free 동안은 안 셈 → 밖으로 나와 묵은 셈으로 바로 멈추지 않게)
    if (free) {revs_.clear();}
    swing_rot_ += std::abs(odom_wz) * dt;
    swing_lin_ += std::hypot(cmd[0], cmd[1]) * dt;
    swing_t_ += dt;
    while (!revs_.empty() && t - revs_.front() > c_.window_s) {revs_.pop_front();}
    if (std::abs(cmd[2]) < kWzMin) {return cmd;}
    const int sg = cmd[2] > 0 ? 1 : -1;
    if (last_sign_ != 0 && t - last_rot_t_ > c_.window_s) {   // window_s 넘게 안 돌다 다시 돎 → 새로 셈
      last_sign_ = 0;
      reset_swing();
    }
    last_rot_t_ = t;
    if (last_sign_ != 0 && sg != last_sign_) {
      const bool in_place = swing_t_ <= 0 || swing_lin_ / swing_t_ < c_.lin_eps;
      last_swing_ = swing_rot_;
      if (in_place && swing_rot_ >= c_.min_swing) {revs_.push_back(t);}
      reset_swing();
      if (!free && static_cast<int>(revs_.size()) > c_.max_rev) {
        mode_ = Mode::Hold;
        until_ = t + c_.hold_s;
        revs_.clear();
        last_sign_ = 0;
        *ev = Ev::Hold;
        return {0.0, 0.0, 0.0};
      }
    }
    last_sign_ = sg;
    return cmd;
  }

private:
  enum class Mode {Free, Hold, Lock};
  void reset_swing() {swing_rot_ = swing_lin_ = swing_t_ = 0.0;}

  Cfg c_;
  Mode mode_ = Mode::Free;
  std::deque<double> revs_;
  int last_sign_ = 0, lock_sign_ = 0;
  double until_ = 0.0, last_rot_t_ = 0.0, swing_rot_ = 0.0, swing_lin_ = 0.0, swing_t_ = 0.0, last_swing_ = 0.0;
};

}  // namespace navigation::guard
