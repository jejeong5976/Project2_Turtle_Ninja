// rest_torque.hpp — 쉬는 팔: 주행 중 팔 보드 토크 끄기 · 다시 켜기 (ROS 없는 순수 C++, test/test_rest_torque.cpp)
//   왜: 주행 내내 접은 자세를 토크로 버티면 팔 배터리가 처져 HOME 복귀 중 과부하 (Hardware Error) 가 남.
//
// 끄기: control_master 가 /<보드>/rest_torque (std_msgs/Bool) 를 0.1 s 마다 true 로 보냄 → CMD 토크 비트 (보드 전체) 끔. 읽기는 그대로
//   다시 켬: 요청 false 또는 request_timeout_s 넘게 안 옴 (control_master 죽음 · 끊김)
//   ⚠ 보드가 깨끗할 때만 끔 (읽기 · 쓰기 실패 · 토크 켜기 기다림 · Hardware Error · 펌웨어 잠금 없음)
//   ⚠ 바퀴 (속도 관절) 가 있는 보드는 안 씀 (rest_eligible)
//
// 다시 켤 때 튀지 않게 (RestGate):
//   펌웨어는 켤 때 목표 = 지금 위치 → 켜는 순간은 안 튐. 그러나 JTC 는 끄기 전 자세를 계속 명령 중 (profile_vel 0 = 최고 속도)
//   → 관절마다 명령이 지금 측정 tol 안으로 올 때까지 NaN (= 지금 위치에서 버팀) 을 보냄
//   ⚠ JTC interpolate_from_desired_state false 를 가정: 새 궤적이 측정 자세에서 시작해야 그 관절이 풀림.
//     옛 명령만 계속 오면 그 관절은 버팀 (튀지 않음, 작업이 목표 오차로 실패할 뿐)
//
// URDF 한계 (near_limit): JTC 는 측정 위치가 URDF 한계 밖이면 예외 → 컨트롤러가 꺼짐. HOME J2 는 한계에서 0.049 rad 뿐
//   → 끈 동안 한계 margin 안으로 들어오면 바로 다시 켜고 이번 요청이 끝날 때까지 켠 채 (래치). 이미 margin 안이면 안 끔

#pragma once

#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

namespace hardware
{

// 이번 주기 토크를 꺼 둘까: 요청이 true · 신선 (age ≤ timeout) · 보드 깨끗
inline bool rest_wanted(bool request, double request_age_s, double timeout_s, bool board_clean)
{
  return request && std::isfinite(request_age_s) && request_age_s >= 0.0 && request_age_s <= timeout_s && board_clean;
}

// URDF 한계 가까운 관절: q[i] 가 [lo[i] + margin[i], hi[i] − margin[i]] 밖이면 가장 앞 번호, 없으면 −1 (NaN · 한계 없음 (±inf) 은 안 봄)
inline int near_limit(const std::vector<double> & q, const std::vector<double> & lo, const std::vector<double> & hi,
  const std::vector<double> & margin)
{
  for (std::size_t i = 0; i < q.size() && i < lo.size() && i < hi.size(); ++i) {
    const double m = i < margin.size() ? margin[i] : 0.0;
    if (!std::isfinite(q[i])) {continue;}
    if ((std::isfinite(lo[i]) && q[i] < lo[i] + m) || (std::isfinite(hi[i]) && q[i] > hi[i] - m)) {return static_cast<int>(i);}
  }
  return -1;
}

// 다시 켠 뒤 옛 명령 막기 (위 설명)
class RestGate
{
public:
  // 다시 켤 때: gated[i] = 막을 관절 (위치 · 그리퍼 모드. 바퀴는 false)
  void arm(const std::vector<bool> & gated) {hold_ = gated;}
  void clear() {hold_.clear();}
  bool holding() const
  {
    for (bool h : hold_) {
      if (h) {return true;}
    }
    return false;
  }
  bool holding(std::size_t i) const {return i < hold_.size() && hold_[i];}
  // targets: 컨트롤러 명령 (막힌 관절은 NaN 으로 바꿈), meas: 지금 측정 (NaN = 모름 → 계속 막음), tol: 관절 단위 허용
  //   돌려줌 = 이번에 풀린 관절 수
  int filter(std::vector<double> & targets, const std::vector<double> & meas, const std::vector<double> & tol)
  {
    int released = 0;
    for (std::size_t i = 0; i < hold_.size() && i < targets.size(); ++i) {
      if (!hold_[i]) {continue;}
      const double c = targets[i];
      const double m = i < meas.size() ? meas[i] : std::numeric_limits<double>::quiet_NaN();
      const double t = i < tol.size() ? tol[i] : 0.0;
      if (std::isfinite(c) && std::isfinite(m) && std::fabs(c - m) <= t) {
        hold_[i] = false;   // 명령이 지금 자세에 옴 → 이제부터 그대로 보냄
        ++released;
        continue;
      }
      targets[i] = std::numeric_limits<double>::quiet_NaN();
    }
    return released;
  }

private:
  std::vector<bool> hold_;
};

// 플러그인 한 보드의 쉼 상태 (write() 가 주기마다 step). 로그 · 토크 비트는 부르는 쪽이
class RestTorque
{
public:
  enum class Change { NONE, OFF, ON };
  // want: rest_wanted(...) 결과. gated: 다시 켤 때 막을 관절 (RestGate::arm)
  //   near: near_limit(지금 측정) (−1 = 없음), request: 요청 자체 (false 가 되면 한계 래치를 풂 — 다음 쉼부터 다시 끔)
  //   OFF = 지금 토크를 끔 (쉼 시작), ON = 지금 다시 켬 (막기 시작). 한계 때문이면 limit_joint() ≥ 0
  Change step(bool want, const std::vector<bool> & gated, int near = -1, bool request = true)
  {
    if (!request) {limit_joint_ = -1;}
    if (request && near >= 0 && limit_joint_ < 0 && (want || resting_)) {limit_joint_ = near;}
    want = want && limit_joint_ < 0;
    if (want == resting_) {return Change::NONE;}
    resting_ = want;
    if (want) {
      gate_.clear();   // 끈 동안은 펌웨어가 목표를 안 씀 — 막기는 다시 켤 때부터
      return Change::OFF;
    }
    gate_.arm(gated);
    return Change::ON;
  }
  // 하드웨어 다시 켜기 (on_activate) · 처음: 쉼 · 막기 · 한계 래치 없음 (켜기는 원래대로 목표 없음 → 지금 위치)
  void reset() {resting_ = false; gate_.clear(); limit_joint_ = -1;}
  bool resting() const {return resting_;}
  int limit_joint() const {return limit_joint_;}   // URDF 한계 가까워 켠 채 두는 관절 (−1 = 없음)
  RestGate & gate() {return gate_;}
  const RestGate & gate() const {return gate_;}

private:
  bool resting_{false};
  int limit_joint_{-1};
  RestGate gate_;
};

}  // namespace hardware
