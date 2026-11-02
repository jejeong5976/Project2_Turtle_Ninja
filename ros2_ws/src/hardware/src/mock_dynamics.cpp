// mock_dynamics.cpp — 속도 · 가속 한계가 있는 가짜 모터
// =============================================================================
// mock_components/GenericSystem 은 위치 명령을 받은 그 순간 "도착" 으로 돌려줘서, 시뮬에서 조향에 걸리는 시간이 0 이었음
// (S자 · ㄹ자에서 실물은 조향이 도는 동안 몸이 안 움직였는데 시뮬은 매끄럽게 통과). 이 플러그인은 다이나믹셀 프로파일처럼:
//   위치 관절(조향): 사다리꼴 — 최고 속도 profile_vel, 가속 profile_acc. 목표가 바뀌면 지금 속도에서 다시 계산
//   속도 관절(바퀴): 목표 속도까지 가속 profile_acc 로
// 값의 단위는 다이나믹셀 그대로 (hardware_map.yaml): profile_vel × 0.229 rpm, profile_acc × 214.577 rev/min²
//   0 = 제한 없음(다이나믹셀과 같음) → 그때는 모터 자체 한계 max_vel_rad_s · max_acc_rad_s2 (관절 param, 없으면 아래 기본)
//   조향 기본 최고 4.71 rad/s (= 270°/s, 공중 실측 265~273°/s) · 가속 기본 37 rad/s²
//     (공중 실측: profile 100 에서 135° 1.04~1.06 s · 90° 0.72 s → 역산 약 37)
//   바퀴 기본 최고 7.91 rad/s (W210 Velocity Limit 330 × 0.229 rpm) · 가속 기본 30 rad/s² (가정)
// 마찰 · 미끄럼 · 선 당김은 없음. effort 는 0.
// 쓰는 곳: ros2_control.xacro — hardware:=mock 이고 mock_dynamics:=true 일 때

#include <algorithm>
#include <cmath>
#include <string>
#include <unordered_map>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

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
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    for (auto & s : joints_) {s.goal = s.velocity_mode ? 0.0 : s.pos; s.vel = 0.0;}
    publish();
    return CallbackReturn::SUCCESS;
  }

  return_type read(const rclcpp::Time &, const rclcpp::Duration & period) override
  {
    const double dt = std::clamp(period.seconds(), 0.0, 0.05);
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
    for (auto & s : joints_) {
      const std::string cmd = s.name + "/" + (s.velocity_mode ? hardware_interface::HW_IF_VELOCITY :
        hardware_interface::HW_IF_POSITION);
      const double v = get_command<double>(cmd);
      if (std::isfinite(v)) {s.goal = v;}
    }
    return return_type::OK;
  }

private:
  struct Joint
  {
    std::string name;
    bool velocity_mode = false;
    double vmax = 0.0, amax = 0.0, pos = 0.0, vel = 0.0, goal = 0.0;
  };

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
