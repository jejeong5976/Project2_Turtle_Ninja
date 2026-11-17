// arm_rest.hpp — 쉬는 팔: 주행 중 팔 보드 토크를 끌지 · 다시 켤지 · 다시 앉힐지 (ROS 없는 순수 C++, test/test_arm_rest.cpp)
// 이유: 팔 토크가 주행 내내 켜져 있으면 팔 배터리가 처져 HOME 복귀 중 관절 과부하 (Hardware Error 0x20) 가 남.
//
// 판단 (control_master 가 주기마다 update, 기본 꺼짐 = 토크 늘 켜짐):
//   ① 끄기 요청: 켜짐 (arm_rest_torque_off) + 팔 작업 없음 + DRIVE 차례 + 빈손 + 하드웨어 정상
//      + 접힘 확인이 rest_after_s 동안 이어짐 → /<보드>/rest_torque true (0.1 s 마다, hardware/rest_torque.hpp)
//   ② 켜기 요청: 조건 하나라도 깨지면 바로 (팔 작업이 오면 계획 전에 wake). 보드는 요청이 1 s 끊겨도 켬
//   ③ 끈 뒤 (slipped): DRIVE 접힘 확인은 느슨한 허용 (rest_fold_tol_rad), 밖이면 STOP. 팔 궤적을 보내면 엄격한 확인으로
//   ④ ARM 차례에서 움직이기 전: 끈 적이 있으면 "토크 켜짐" 을 기다림 (wake_timeout_s 넘으면 경고 뒤 진행 — 거절 안 함)
//   ⑤ 다시 앉히기 (reseat_choice): 처짐 ≤ reseat_min_rad → 없음 · ≤ slip_max_rad → home 한 점 (SHORT) · 넘으면 검사된 복귀 경로 (CHECKED)
// 보드 상태가 한 번도 안 오면 끈 적이 없는 것으로 봄 → 기다림 · 느슨한 확인 · 다시 앉히기 없음

#pragma once

#include <string>
#include <vector>

namespace manipulation
{

struct ArmRestConfig
{
  bool enabled{false};          // arm_rest_torque_off
  double after_s{1.0};          // rest_after_s
  double wake_timeout_s{1.0};   // rest_wake_timeout_s
  double state_alive_s{1.0};    // 보드 토크 상태가 이 안에 왔으면 보드가 쉬는 팔을 앎
};

struct ArmRestInput
{
  bool drive{false};     // 차례 = DRIVE
  bool job{false};       // 팔 작업 (goal · 진행 중 계획 · 자동 접기)
  bool holding{false};   // 퍽을 쥐었을 수 있음 (carrying_ 또는 그리퍼 실측이 grasp_empty_m 과 열림 사이)
  bool hw_ok{false};     // 하드웨어 정상
  bool folded{false};    // 접힘 확인 (부르는 쪽: slipped() 면 느슨, 아니면 엄격)
};

class ArmRest
{
public:
  // 매 주기. 돌려줌 = 보드에 보낼 요청 (true = 토크 꺼도 됨). why: 끄지 않는 첫 이유 · 기다리는 중이면 남은 시간 (로그)
  bool update(double now_s, const ArmRestInput & in, const ArmRestConfig & c, std::string * why = nullptr);
  // 팔 작업이 옴 (on_arm_goal, 계획 전) → 바로 켜기 요청 + rest_after_s 를 처음부터
  void wake(double now_s);
  // 보드 토크 상태 (/<보드>/torque_state = 실제 토크 비트, 받은 시각)
  void on_state(double now_s, bool torque_on);
  // 팔을 움직여도 되나 (ARM 차례에서 첫 궤적 전). 켜기를 기다리는 중이면 켜짐 확인 또는 wake_timeout_s 지남 (timed_out = true)
  bool arm_ok(double now_s, const ArmRestConfig & c, bool * timed_out = nullptr);
  // 팔 궤적을 보냄 (다시 앉히기 · 작업 첫 동작) → 처짐 끝 (엄격한 접힘 확인으로)
  void moved() {slipped_ = false;}

  bool requested() const {return want_;}
  bool slipped() const {return slipped_;}
  bool waking() const {return waking_;}
  double wake_t() const {return wake_t_;}
  bool board_alive(double now_s, const ArmRestConfig & c) const {return have_state_ && now_s - state_t_ <= c.state_alive_s;}
  bool torque_on() const {return state_on_;}   // 마지막으로 받은 보드 상태 (없으면 true)

private:
  void start_wake(double now_s);
  bool want_{false};
  double quiet_since_{-1.0};   // 조건이 이어지기 시작한 시각 (음수 = 지금 안 맞음)
  bool slipped_{false};        // 끈 적이 있고 아직 팔 궤적을 안 보냄
  bool waking_{false};         // 켜기 요청 뒤 켜짐 확인을 기다림
  double wake_t_{0.0};
  bool have_state_{false}, state_on_{true};
  double state_t_{-1e9};
};

// 다시 앉히기 (⑤). q · home = 팔 관절 [rad] 같은 순서
struct Reseat
{
  enum class Kind { NONE, SHORT, CHECKED };
  Kind kind{Kind::NONE};
  double slip_rad{0.0};    // 가장 큰 관절 처짐 |q − home|
  int worst{-1};           // 그 관절 번호 (0 부터, −1 = 모름)
  double time_s{0.0};      // SHORT: home 한 점 시간 = max(min_s, slip / speed)
};
// 측정이 없거나 (NaN · 길이 다름) 처짐 ≤ min_rad → NONE. ≤ max_rad → SHORT. 넘으면 CHECKED
Reseat reseat_choice(const std::vector<double> & q, const std::vector<double> & home, double min_rad, double max_rad,
  double min_s, double speed_rad_s);
// "J1 +0.3° · J2 −1.2° · …" (NaN 은 "?") — 로그용
std::string slip_text(const std::vector<double> & q, const std::vector<double> & home);

}  // namespace manipulation
