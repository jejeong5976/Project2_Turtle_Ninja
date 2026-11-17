// mock_dynamics.cpp — 속도 · 가속 한계가 있는 가짜 모터
// =============================================================================
// mock_components/GenericSystem 은 위치 명령을 받은 그 순간 "도착" 으로 돌려줘 조향에 걸리는 시간이 0
// (실물은 S자 · ㄹ자에서 조향이 도는 동안 몸이 안 움직임). 이 플러그인은 다이나믹셀 프로파일처럼:
//   위치 관절(조향): 사다리꼴 — 최고 속도 profile_vel, 가속 profile_acc. 목표가 바뀌면 지금 속도에서 다시 계산
//   속도 관절(바퀴): 목표 속도까지 가속 profile_acc 로
// 값의 단위는 다이나믹셀 그대로 (hardware_map.yaml): profile_vel × 0.229 rpm, profile_acc × 214.577 rev/min²
//   0 = 제한 없음(다이나믹셀과 같음) → 그때는 모터 자체 한계 max_vel_rad_s · max_acc_rad_s2 (관절 param, 없으면 아래 기본)
//   조향 기본 최고 4.71 rad/s (= 270°/s, 공중 실측 265~273°/s) · 가속 기본 37 rad/s²
//     (공중 실측: profile 100 에서 135° 1.04~1.06 s · 90° 0.72 s → 역산 약 37)
//   바퀴 기본 최고 7.91 rad/s (W210 Velocity Limit 330 × 0.229 rpm) · 가속 기본 30 rad/s² (가정)
// 마찰 · 미끄럼 · 선 당김은 없음. effort 는 0.
// 쓰는 곳: ros2_control.xacro — hardware:=mock 이고 mock_dynamics:=true 일 때
//
// 쉬는 팔 (rest_torque.hpp) 흉내: 실물 플러그인과 같은 토픽 /<보드>/rest_torque · /<보드>/torque_state (바퀴 없는 보드만)
//   토크 끈 동안: 명령을 안 따름 + 팔 관절이 하드웨어 노드 파라미터 mock_rest_drift_rad_s 만큼 처짐 (0.5 rad 까지, 그리퍼는 안 처짐)
//     방향 = URDF 한계에서 먼 쪽. mock_rest_drift_toward_limit true = 가까운 한계 쪽 (한계 지킴 시험: J2 HOME 은 한계에서 0.049 rad)
//     예) ros2 param set /bus_arm mock_rest_drift_rad_s 0.02   (기본 0 = 안 처짐)
//   다시 켤 때: 펌웨어처럼 목표 = 지금 위치 + 옛 명령 막기 (RestGate) — 실물과 같은 판단

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

#include "hardware/rest_torque.hpp"

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace hardware
{
using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kVelUnit = 0.229 * 2.0 * kPi / 60.0;            // [rad/s] per profile_vel 눈금
constexpr double kAccUnit = 214.577 * 2.0 * kPi / 3600.0;        // [rad/s²] per profile_acc 눈금

double param(const std::unordered_map<std::string, std::string> & p, const std::string & k, double fallback)
{
  const auto it = p.find(k);
  return it == p.end() || it->second.empty() ? fallback : std::stod(it->second);
}
}  // namespace

class MockDynamics : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams & params) override
  {
    if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
      return CallbackReturn::ERROR;
    }
    for (const auto & j : get_hardware_info().joints) {
      Joint s;
      s.name = j.name;
      if (j.command_interfaces.size() != 1) {
        RCLCPP_ERROR(get_logger(), "%s: 명령 칸이 1개가 아님", j.name.c_str());
        return CallbackReturn::ERROR;
      }
      s.velocity_mode = j.command_interfaces[0].name == hardware_interface::HW_IF_VELOCITY;
      const auto & p = j.parameters;
      const double pv = param(p, "profile_vel", 0.0), pa = param(p, "profile_acc", 0.0);
      const double vmax = param(p, "max_vel_rad_s", s.velocity_mode ? 7.91 : 4.71);
      const double amax = param(p, "max_acc_rad_s2", s.velocity_mode ? 30.0 : 37.0);
      s.vmax = pv > 0 ? std::min(pv * kVelUnit, vmax) : vmax;
      s.amax = pa > 0 ? std::min(pa * kAccUnit, amax) : amax;
      for (const auto & si : j.state_interfaces) {
        if (si.name == hardware_interface::HW_IF_POSITION && !si.initial_value.empty()) {
          s.pos = std::stod(si.initial_value);
        }
      }
      joints_.push_back(s);
      RCLCPP_INFO(get_logger(), "%s: %s · 최고 %.2f rad/s (%.0f°/s) · 가속 %.1f rad/s²", s.name.c_str(),
        s.velocity_mode ? "속도" : "위치", s.vmax, s.vmax * 180.0 / kPi, s.amax);
    }
    bool has_wheel = false;
    for (const auto & s : joints_) {
      has_wheel = has_wheel || s.velocity_mode;
      gated_.push_back(!s.velocity_mode);
      const double sc = s.name.rfind("gripper", 0) == 0 ? 0.01149 : 1.0;   // 실물 hardware_map scale (그리퍼 m)
      gate_tol_.push_back(0.02 * sc);   // 실물과 같은 rest_gate_tol_rad × |scale|
      margin_.push_back(0.02 * sc);     // 실물과 같은 rest_limit_margin_rad × |scale|
      const auto lim = get_hardware_info().limits.find(s.name);
      const bool has = lim != get_hardware_info().limits.end() && lim->second.has_position_limits;
      lo_.push_back(has ? lim->second.min_position : -std::numeric_limits<double>::infinity());
      hi_.push_back(has ? lim->second.max_position : std::numeric_limits<double>::infinity());
    }
    meas_.assign(joints_.size(), 0.0);
    // 쉬는 팔 흉내 (실물 opencr_hardware 와 같은 토픽 · 판단)
    const auto node = get_node();
    if (node && !has_wheel && !joints_.empty()) {
      const std::string base = "/" + get_hardware_info().name;
      node->declare_parameter<double>("mock_rest_drift_rad_s", 0.0);
      node->declare_parameter<bool>("mock_rest_drift_toward_limit", false);
      rest_sub_ = node->create_subscription<std_msgs::msg::Bool>(base + "/rest_torque", rclcpp::QoS(1).reliable(),
        [this](std_msgs::msg::Bool::ConstSharedPtr m) {
          req_.store(m->data);
          req_ns_.store(steady_ns());
        });
      torque_pub_ = node->create_publisher<std_msgs::msg::Bool>(base + "/torque_state", rclcpp::QoS(1).reliable());
      timer_ = node->create_wall_timer(std::chrono::milliseconds(100), [this]() {
          if (const auto n = get_node()) {
            drift_.store(n->get_parameter("mock_rest_drift_rad_s").as_double());
            toward_.store(n->get_parameter("mock_rest_drift_toward_limit").as_bool());
          }
          std_msgs::msg::Bool b;
          b.data = !resting_.load();
          torque_pub_->publish(b);
        });
      eligible_ = true;
      RCLCPP_INFO(get_logger(), "쉬는 팔 흉내: %s/rest_torque → 토크 끔 (처짐 mock_rest_drift_rad_s), 상태 %s/torque_state", base.c_str(), base.c_str());
    }
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    for (auto & s : joints_) {s.goal = s.velocity_mode ? 0.0 : s.pos; s.vel = 0.0;}
    rest_.reset();
    resting_.store(false);
    publish();
    return CallbackReturn::SUCCESS;
  }

  return_type read(const rclcpp::Time &, const rclcpp::Duration & period) override
  {
    const double dt = std::clamp(period.seconds(), 0.0, 0.05);
    if (rest_.resting()) {   // 토크 끔: 명령을 안 따르고, 위치 관절이 번갈아 + − 로 처짐 (0.5 rad 까지)
      const double d = drift_.load();
      for (std::size_t i = 0; i < joints_.size(); ++i) {
        auto & s = joints_[i];
        s.vel = 0.0;
        if (s.velocity_mode || !(d > 0.0) || s.name.rfind("gripper", 0) == 0) {continue;}   // 그리퍼 (m) 는 안 처짐
        const bool up = (hi_[i] - s.rest_from > s.rest_from - lo_[i]) != toward_.load();   // 먼 한계 쪽 (toward = 가까운 쪽)
        const double off = s.pos + (up ? 1.0 : -1.0) * d * dt - s.rest_from;
        s.pos = s.rest_from + std::clamp(off, -0.5, 0.5);
      }
      publish();
      return return_type::OK;
    }
    for (auto & s : joints_) {
      if (s.velocity_mode) {
        const double target = std::isfinite(s.goal) ? std::clamp(s.goal, -s.vmax, s.vmax) : 0.0;
        s.vel += std::clamp(target - s.vel, -s.amax * dt, s.amax * dt);
        s.pos += s.vel * dt;
        continue;
      }
      if (!std::isfinite(s.goal)) {s.vel = 0.0; continue;}
      // 사다리꼴: 남은 거리에서 멈출 수 있는 속도 이하로, 가속 한계 안에서 바꿈
      const double err = s.goal - s.pos;
      const double want = std::copysign(std::min(s.vmax, std::sqrt(2.0 * s.amax * std::fabs(err))), err);
      s.vel += std::clamp(want - s.vel, -s.amax * dt, s.amax * dt);
      const double step = s.vel * dt;
      if (std::fabs(step) >= std::fabs(err) && step * err >= 0.0) {s.pos = s.goal; s.vel = 0.0;}
      else {s.pos += step;}
    }
    publish();
    return return_type::OK;
  }

  return_type write(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    std::vector<double> cmds(joints_.size());
    for (std::size_t i = 0; i < joints_.size(); ++i) {
      const auto & s = joints_[i];
      const std::string cmd = s.name + "/" + (s.velocity_mode ? hardware_interface::HW_IF_VELOCITY :
        hardware_interface::HW_IF_POSITION);
      cmds[i] = get_command<double>(cmd);
    }
    if (eligible_) {
      const int64_t rx = req_ns_.load();
      const double age = rx == 0 ? std::numeric_limits<double>::quiet_NaN() : (steady_ns() - rx) * 1e-9;
      for (std::size_t i = 0; i < joints_.size(); ++i) {meas_[i] = joints_[i].pos;}
      const bool req = req_.load();
      const int limit_before = rest_.limit_joint();
      const auto change = rest_.step(rest_wanted(req, age, 1.0, true), gated_, near_limit(meas_, lo_, hi_, margin_), req);
      if (rest_.limit_joint() >= 0 && limit_before < 0) {
        const auto j = static_cast<std::size_t>(rest_.limit_joint());
        RCLCPP_WARN(get_logger(), "쉬는 팔 흉내: %s %.4f 이 URDF 한계 [%.4f, %.4f] 에 %.3f 안 → 토크 켬 (요청이 끝날 때까지)", joints_[j].name.c_str(),
          meas_[j], lo_[j], hi_[j], margin_[j]);
      }
      switch (change) {
        case RestTorque::Change::OFF:
          for (auto & s : joints_) {s.rest_from = s.pos;}
          resting_.store(true);
          RCLCPP_INFO(get_logger(), "쉬는 팔 흉내: 토크 끔");
          break;
        case RestTorque::Change::ON:
          for (auto & s : joints_) {if (!s.velocity_mode) {s.goal = s.pos;}}   // 펌웨어: 목표 = 지금 위치로 켬
          resting_.store(false);
          RCLCPP_INFO(get_logger(), "쉬는 팔 흉내: 토크 다시 켬 (목표 = 지금 위치, 옛 명령 막기)");
          break;
        case RestTorque::Change::NONE:
          break;
      }
      if (rest_.resting()) {return return_type::OK;}   // 끈 동안 목표를 안 씀 (펌웨어 bus_write_targets 와 같음)
      if (rest_.gate().holding()) {
        rest_.gate().filter(cmds, meas_, gate_tol_);
        if (!rest_.gate().holding()) {RCLCPP_INFO(get_logger(), "쉬는 팔 흉내: 옛 명령 막기 끝");}
      }
    }
    for (std::size_t i = 0; i < joints_.size(); ++i) {
      if (std::isfinite(cmds[i])) {joints_[i].goal = cmds[i];}
    }
    return return_type::OK;
  }

private:
  struct Joint
  {
    std::string name;
    bool velocity_mode = false;
    double vmax = 0.0, amax = 0.0, pos = 0.0, vel = 0.0, goal = 0.0;
    double rest_from = 0.0;   // 토크 끈 순간 위치 (처짐 한계 기준)
  };

  static int64_t steady_ns()
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  // 쉬는 팔 흉내 (구독 · 타이머 = 하드웨어 노드 스레드 → atomic 만)
  bool eligible_ = false;
  RestTorque rest_;
  std::vector<bool> gated_;
  std::vector<double> gate_tol_, meas_, lo_, hi_, margin_;
  std::atomic<bool> req_{false}, resting_{false}, toward_{false};
  std::atomic<int64_t> req_ns_{0};
  std::atomic<double> drift_{0.0};
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr rest_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr torque_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void publish()
  {
    for (const auto & s : joints_) {
      set_state(s.name + "/" + hardware_interface::HW_IF_POSITION, s.pos);
      set_state(s.name + "/" + hardware_interface::HW_IF_VELOCITY, s.vel);
      set_state(s.name + "/" + hardware_interface::HW_IF_EFFORT, 0.0);
    }
  }

  std::vector<Joint> joints_;
};

}  // namespace hardware

PLUGINLIB_EXPORT_CLASS(hardware::MockDynamics, hardware_interface::SystemInterface)
