// opencr_hardware.cpp — ros2_control 하드웨어 플러그인 "hardware/OpenCRHardware"
// =====================================================================
// D3 A안 · D12: OpenCR 1장 + 펌웨어(firmware/nt_opencr) + 이 플러그인 (OpenCR 내장 IMU 까지 한 줄로 받으려고)
//
//   컨트롤러들 ──명령(rad, rad/s, m)──▶ [이 플러그인] ──CMD 100 Hz──▶ OpenCR ──▶ 모터 14개
//   joint_state_broadcaster ◀──상태──    write()/read()  ◀──STATE──      ◀── IMU
//   imu_sensor_broadcaster  ◀──IMU──
//
// 왜 "노드" 가 아니라 ros2_control "플러그인" 인가
//   컨트롤러가 쓴 값을 controller_manager 가 100 Hz 로 바로 넘겨줌 → 토픽을 한 번 덜 거치고,
//   컨트롤러(JTC 등) · MoveIt · mock 과 hardware:= 인자 하나로 바꿔 끼울 수 있음
//
// 생명 주기 (ros2_control 이 부름)
//   on_init      : URDF(ros2_control.xacro) 의 param 읽기 — 모터 ID · 모드 · 한계 · 포트
//   on_configure : 포트 열기 → CONFIG/ACK 악수 (누가 대답했나) → 첫 상태 받기
//   on_activate  : 토크 켜기 (펌웨어가 "지금 위치 = 목표" 로 해서 켜는 순간 안 튐)
//   read/write   : 100 Hz — 상태 받기 / 명령 보내기
//   on_deactivate: 바퀴 0. 토크는 그대로 (끄면 팔이 떨어짐 — torque_off_on_deactivate 로 바꿀 수 있음)
//   on_error     : read/write 가 ERROR 일 때 → 바로 바퀴 0
//
// 멈추는 조건 (read() 가 ERROR → controller_manager 가 하드웨어 · 컨트롤러를 멈춤)
//   - OpenCR 상태가 state_timeout_s(0.5 s) 동안 안 옴 (USB 뽑힘 · 펌웨어 멈춤)
//   - 어느 모터든 motor_fail_timeout_s(0.2 s) 넘게 못 읽음 (D20 전체 멈춤 — 스워브에서 모터 하나만 빼고 계속 달리면 위험)
//   - OpenCR 가 리셋돼서 모터 목록을 잃음
//   - 토크를 켜라고 했는데 0.5 s 안에 펌웨어가 "켜짐" 을 안 알려 옴 (모터 쓰기 실패)
//   - 펌웨어 고장 잠금(ST_FAULT_LATCHED: 목표 쓰기 실패 → 펌웨어가 바퀴를 세우고 잠금) — 바로
//   - 배터리 전압 (2026-10-16, DIAG 1 초마다): 구간 평균이 battery_stop_v 아래로 2 번 연속 → ERROR (경고는 battery_warn_v)
//
// 복구 (2026-10-16 재검증 R6): ERROR 뒤에는 자동으로 다시 켜지 않음. 사람이 원인을 확인하고
//   팔을 받친 뒤(CONFIG 가 모든 모터 토크를 끔 → 팔 처짐) 하드웨어를 다시 configure · activate → CONFIG 가 펌웨어 잠금을 풂
//   → control_master 는 그동안 STOP 이고, HOME + 접힘 실측이 다시 성공해야 DRIVE
//
// 실제 일(바이트 주고받기 · 단위 변환)은 opencr_link.hpp (ROS 없는 클래스, 가짜 OpenCR 로 시험됨).
// 여기는 ros2_control 과 이어 붙이는 껍데기만.

#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware/opencr_link.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace hardware
{

class OpenCRHardware : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  struct JointIfaces
  {
    std::string cmd;   // "drive_fl_joint/velocity" 처럼 명령 칸 이름
    std::string pos, vel, eff, position_vel;
  };

  void publish_states();
  void invalidate_states();   // 모든 상태 칸 NaN (ERROR 뒤 옛 값이 남지 않게)
  bool send(bool torque, bool with_commands);

  std::unique_ptr<OpenCRLink> link_;
  std::vector<JointIfaces> ifaces_;
  std::vector<double> targets_;
  std::string port_;
  int baud_ = 1000000;
  double config_timeout_s_ = 8.0;
  double state_timeout_s_ = 0.5;
  double motor_fail_timeout_s_ = 0.2;   // 모터 하나라도 이만큼 못 읽으면 ERROR (전체 멈춤)
  double measure_max_age_s_ = 0.05;     // STATE 가 이보다 오래되면 그 값은 "지금의 측정" 이 아님 → NaN (R3)
  bool comm_failing_ = false;
  std::chrono::steady_clock::time_point comm_fail_since_{};
  bool write_failing_ = false;           // 펌웨어가 목표 쓰기 실패(ST_WRITE_FAIL)를 알려 오는 중 (R5)
  // 배터리 (와플파이 3셀 리튬폴리머 11.1 V, 2026-10-16). ⚠ 잠정값 — 실측(부하 중 처짐)으로 정할 것
  //   셀당 3.6 V(10.8) 경고 · 3.4 V(10.2) 정지. XM430 입력 하한이 약 10 V 라 그 전에 멈추려는 값
  double battery_warn_v_ = 10.8;
  double battery_stop_v_ = 10.2;
  uint32_t seen_diag_ = 0;
  int battery_low_count_ = 0;
  std::chrono::steady_clock::time_point write_fail_since_{};
  bool torque_waiting_ = false;          // 토크 켜라고 한 뒤 펌웨어가 "켜짐" 이라고 하기를 기다리는 중
  std::chrono::steady_clock::time_point torque_wait_since_{};
  bool require_all_motors_ = true;
  bool torque_off_on_deactivate_ = false;
  bool torque_ = false;
  std::string imu_name_;   // URDF 의 <sensor name="imu"> (없으면 IMU 안 내보냄)
  std::vector<std::string> imu_ifaces_;   // "imu/orientation.x" … 10 칸 (매 주기 문자열을 새로 만들지 않게 미리)
  uint8_t last_status_ = 0;
  // 주기 시간 측정 (read · write 각각, 10 s 마다 로그). 예산: 둘이 합쳐 10 ms 중 일부만 써야 컨트롤러 몫이 남음
  DurationStats read_time_, write_time_;
  std::chrono::steady_clock::time_point stats_since_{std::chrono::steady_clock::now()};
  void report_timing();
};

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

namespace
{
const double NaN = std::numeric_limits<double>::quiet_NaN();

// <param name="...">값</param> 하나 꺼내기 (없으면 기본값)
std::string param(
  const std::unordered_map<std::string, std::string> & m, const std::string & key, const std::string & def)
{
  const auto it = m.find(key);
  return it == m.end() ? def : it->second;
}

bool has_iface(const std::vector<hardware_interface::InterfaceInfo> & list, const std::string & name)
{
  for (const auto & i : list) {
    if (i.name == name) {return true;}
  }
  return false;
}

// IMU 칸 이름 (imu_sensor_broadcaster 가 찾는 이름 그대로)
const char * const IMU_IFACES[10] = {
  "orientation.x", "orientation.y", "orientation.z", "orientation.w",
  "angular_velocity.x", "angular_velocity.y", "angular_velocity.z",
  "linear_acceleration.x", "linear_acceleration.y", "linear_acceleration.z"};
}  // namespace

// ---------------------------------------------------------------- on_init: URDF 읽기
CallbackReturn OpenCRHardware::on_init(const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  const auto & info = get_hardware_info();
  const auto & hw = info.hardware_parameters;
  std::vector<JointSpec> specs;
  uint8_t bus_watchdog = 15;
  try {
    port_ = param(hw, "port_name", "/dev/ttyACM0");
    baud_ = std::stoi(param(hw, "baud_rate", "1000000"));
    bus_watchdog = static_cast<uint8_t>(std::stoi(param(hw, "bus_watchdog", "15")));
    config_timeout_s_ = std::stod(param(hw, "config_timeout_s", "8.0"));
    state_timeout_s_ = std::stod(param(hw, "state_timeout_s", "0.5"));
    motor_fail_timeout_s_ = std::stod(param(hw, "motor_fail_timeout_s", "0.2"));
    measure_max_age_s_ = std::stod(param(hw, "measure_max_age_s", "0.05"));
    require_all_motors_ = param(hw, "require_all_motors", "true") == "true";
    torque_off_on_deactivate_ = param(hw, "torque_off_on_deactivate", "false") == "true";
    battery_warn_v_ = std::stod(param(hw, "battery_warn_v", "10.8"));
    battery_stop_v_ = std::stod(param(hw, "battery_stop_v", "10.2"));

    for (const auto & j : info.joints) {
      const auto & p = j.parameters;
      JointSpec s;
      s.name = j.name;
      s.id = static_cast<uint8_t>(std::stoi(param(p, "id", "0")));
      const std::string mode = param(p, "mode", "position");
      if (mode == "velocity") {
        s.mode = ntp::MODE_VELOCITY;
      } else if (mode == "gripper") {
        s.mode = ntp::MODE_CURRENT_POSITION;
      } else if (mode == "position") {
        s.mode = ntp::MODE_POSITION;
      } else {
        RCLCPP_ERROR(get_logger(), "%s: mode '%s' 는 모름 (velocity / position / gripper)", j.name.c_str(), mode.c_str());
        return CallbackReturn::ERROR;
      }
      s.profile_vel = static_cast<uint16_t>(std::stoi(param(p, "profile_vel", "0")));
      s.profile_acc = static_cast<uint16_t>(std::stoi(param(p, "profile_acc", "0")));
      s.goal_current = static_cast<uint16_t>(std::stoi(param(p, "goal_current", "0")));
      s.scale = std::stod(param(p, "scale", "1.0"));
      if (s.id == 0 || s.scale == 0.0) {
        RCLCPP_ERROR(get_logger(), "%s: id 또는 scale 이 비었음 (ros2_control.xacro · hardware_map.yaml 확인)", j.name.c_str());
        return CallbackReturn::ERROR;
      }
      // 영점 눈금 (2026-10-16): 관절 0 일 때 모터 눈금. 기본 2048 = 예전과 같음. 위치 모드 한 바퀴(0~4095) 안이어야 함.
      //   바퀴(속도 모드)는 위치가 여러 바퀴 누적이라 영점이 의미 없음 → 2048 이 아니면 거부
      const int zero_tick = std::stoi(param(p, "zero_tick", "2048"));
      if (zero_tick < 0 || zero_tick > 4095 || (s.mode == ntp::MODE_VELOCITY && zero_tick != 2048)) {
        RCLCPP_ERROR(get_logger(), "%s: zero_tick %d 은 위치 모드에서 0~4095 만 (속도 모드는 쓰지 않음)", j.name.c_str(), zero_tick);
        return CallbackReturn::ERROR;
      }
      s.zero_rad = (zero_tick - 2048) * (2.0 * M_PI / 4096.0);

      // 명령 칸은 1개: 바퀴 = velocity, 나머지 = position
      const std::string want = (s.mode == ntp::MODE_VELOCITY) ? hardware_interface::HW_IF_VELOCITY
                                                              : hardware_interface::HW_IF_POSITION;
      if (j.command_interfaces.size() != 1 || j.command_interfaces[0].name != want) {
        RCLCPP_ERROR(get_logger(), "%s: 명령 칸은 '%s' 하나여야 함", j.name.c_str(), want.c_str());
        return CallbackReturn::ERROR;
      }
      JointIfaces f;
      f.cmd = j.name + "/" + want;
      if (has_iface(j.state_interfaces, hardware_interface::HW_IF_POSITION)) {f.pos = j.name + "/position";}
      if (has_iface(j.state_interfaces, hardware_interface::HW_IF_VELOCITY)) {f.vel = j.name + "/velocity";}
      if (has_iface(j.state_interfaces, hardware_interface::HW_IF_EFFORT)) {f.eff = j.name + "/effort";}
      if (has_iface(j.state_interfaces, "position_velocity")) {f.position_vel = j.name + "/position_velocity";}
      ifaces_.push_back(f);
      specs.push_back(s);
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "param 숫자를 못 읽음: %s", e.what());
    return CallbackReturn::ERROR;
  }
  if (specs.size() > ntp::MAX_JOINTS) {
    RCLCPP_ERROR(get_logger(), "모터 %zu 개 — 보드 1장에 최대 %d 개", specs.size(), ntp::MAX_JOINTS);
    return CallbackReturn::ERROR;
  }

  // IMU: <sensor> 중 angular_velocity.z 칸이 있는 것 (보통 name="imu")
  for (const auto & s : info.sensors) {
    if (has_iface(s.state_interfaces, "angular_velocity.z")) {imu_name_ = s.name;}
  }
  if (!imu_name_.empty()) {
    for (const char * n : IMU_IFACES) {imu_ifaces_.push_back(imu_name_ + "/" + n);}
  }

  targets_.assign(specs.size(), NaN);
  link_ = std::make_unique<OpenCRLink>(specs, bus_watchdog);
  RCLCPP_INFO(get_logger(), "OpenCR 플러그인: 모터 %zu 개, 포트 %s, IMU %s",
    specs.size(), port_.c_str(), imu_name_.empty() ? "안 씀" : imu_name_.c_str());
  return CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------- on_configure: 포트 열기 + 악수
CallbackReturn OpenCRHardware::on_configure(const rclcpp_lifecycle::State &)
{
  std::string err;
  if (!link_->open(port_, baud_, err)) {
    RCLCPP_ERROR(get_logger(), "%s", err.c_str());
    return CallbackReturn::ERROR;
  }
  // ⚠ CONFIG 는 모든 모터 토크를 끄고 설정함 → 팔이 들려 있으면 처짐. 고장 뒤 복구라면 팔을 받친 뒤에
  RCLCPP_WARN(get_logger(), "CONFIG 보내는 중 — 모든 모터 토크가 잠시 꺼짐 (팔이 들려 있으면 처짐: 받친 뒤 진행). "
    "모터마다 ping · 모드 · 한계 설정, 몇 초 걸릴 수 있음…");
  const int mask = link_->configure(config_timeout_s_);
  if (mask < 0) {
    RCLCPP_ERROR(get_logger(),
      "OpenCR 가 %.0f s 동안 대답 없음 — 펌웨어(firmware/nt_opencr)가 올라가 있나? 포트가 맞나?", config_timeout_s_);
    link_->close();
    return CallbackReturn::ERROR;
  }
  const std::string missing = link_->missing_joints(mask);
  if (!missing.empty()) {
    if (require_all_motors_) {
      RCLCPP_ERROR(get_logger(), "대답 없거나 설정에 실패한 모터: %s — 전원 · 선 · ID 확인 (일부만 시험하려면 require_all_motors:=false)",
        missing.c_str());
      link_->close();
      return CallbackReturn::ERROR;
    }
    RCLCPP_WARN(get_logger(), "대답 없거나 설정 실패한 모터 (무시하고 진행): %s", missing.c_str());
  }
  if (!link_->wait_for_state(1.0)) {
    RCLCPP_ERROR(get_logger(), "악수는 됐는데 상태(STATE)가 안 옴");
    link_->close();
    return CallbackReturn::ERROR;
  }
  publish_states();
  RCLCPP_INFO(get_logger(), "OpenCR 준비 끝 (IMU %s)",
    (link_->state().status & ntp::ST_IMU_OK) ? "정상" : "없음 — odom 회전은 바퀴 계산으로");
  return CallbackReturn::SUCCESS;
}

CallbackReturn OpenCRHardware::on_cleanup(const rclcpp_lifecycle::State &)
{
  link_->close();
  return CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------- 켜기 / 끄기
CallbackReturn OpenCRHardware::on_activate(const rclcpp_lifecycle::State &)
{
  // 목표는 아직 전부 "없음" → 펌웨어가 지금 위치를 목표로 잡고 토크를 켬 (켜는 순간 팔이 안 튐)
  targets_.assign(targets_.size(), NaN);
  torque_ = true;
  if (!send(true, false)) {return CallbackReturn::ERROR;}
  RCLCPP_INFO(get_logger(), "토크 켬");
  return CallbackReturn::SUCCESS;
}

CallbackReturn OpenCRHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
  // 바퀴는 0 (목표 없음). 팔은 기본으로 버팀 — 끄면 무게 때문에 떨어짐
  torque_ = !torque_off_on_deactivate_;
  send(torque_, false);
  RCLCPP_INFO(get_logger(), "멈춤 (바퀴 0, 토크 %s)", torque_ ? "유지" : "끔");
  return CallbackReturn::SUCCESS;
}

// read()/write() 가 ERROR 를 돌려주면 ros2_control 이 부름 → 펌웨어 경비견(0.3 s)을 기다리지 말고 바로 바퀴 0
CallbackReturn OpenCRHardware::on_error(const rclcpp_lifecycle::State &)
{
  send(torque_, false);   // 목표 전부 "없음" = 바퀴 0, 팔 · 조향은 그 자리에서 버팀
  // (재검증 R6) 상태 칸을 전부 NaN 으로: ERROR 뒤에는 read() 가 안 불려 마지막 값이 그대로 남는데,
  //   joint_state_broadcaster 가 계속 내보내면 control_master 가 "옛 값 = 지금 측정" 으로 볼 수 있음
  //   → NaN 이면 control_master 는 하드웨어 이상으로 보고 STOP + HOME 무효화, swerve_controller 는 바퀴 0
  invalidate_states();
  RCLCPP_ERROR(get_logger(), "하드웨어 ERROR → 바퀴 0 보냄 (팔은 토크 유지). 자동으로 다시 켜지 않음 — "
    "원인 확인 → 팔 받치기 → 하드웨어 재설정(configure · activate) → HOME");
  return CallbackReturn::SUCCESS;
}

CallbackReturn OpenCRHardware::on_shutdown(const rclcpp_lifecycle::State &)
{
  send(torque_, false);
  link_->close();
  return CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------- 100 Hz
return_type OpenCRHardware::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  const auto t0 = std::chrono::steady_clock::now();
  struct Timer   // 어느 return 으로 나가든 걸린 시간을 기록
  {
    DurationStats & st; std::chrono::steady_clock::time_point t0;
    ~Timer() {st.add(std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count());}
  } timer{read_time_, t0};
  if (link_->poll() < 0) {
    RCLCPP_ERROR(get_logger(), "시리얼 포트 고장 (USB 가 빠졌나?)");
    return return_type::ERROR;
  }
  const double age = link_->seconds_since_state();
  if (age > state_timeout_s_) {
    // ERROR 를 돌려주면 controller_manager 가 이 하드웨어를 멈춤 → 컨트롤러도 멈춤
    RCLCPP_ERROR(get_logger(), "OpenCR 상태가 %.2f s 동안 안 옴 → 멈춤", age);
    return return_type::ERROR;
  }
  const auto & s = link_->state();
  // 펌웨어가 설정한 모터 수만큼 안 보냄 = OpenCR 가 리셋돼서 CONFIG 를 잃었음 → 멈춤 (다시 켜야 함)
  if (!s.joints_valid) {
    RCLCPP_ERROR(get_logger(), "OpenCR 가 모터 상태를 안 보냄 (보드가 리셋됐나?) → 멈춤");
    return return_type::ERROR;
  }
  // (재검증 R6) 펌웨어 고장 잠금: 목표 쓰기가 계속 실패해 펌웨어가 바퀴를 세우고 잠갔음 → 바로 ERROR
  //   (침묵 중이면 comm_err 가 전부 켜지는데, 아래 0.2 s 유예를 기다리지 않도록 그보다 먼저 봄)
  //   펌웨어는 통신이 돌아와도 스스로 풀지 않음 (CONFIG 로만). 여기서도 기다리지 않고 멈춤
  if (s.status & ntp::ST_FAULT_LATCHED) {
    RCLCPP_ERROR(get_logger(), "펌웨어 고장 잠금 (모터 목표 쓰기 실패 → 바퀴 정지%s) → 전체 멈춤. "
      "버스 · 배선 확인 → 팔 받치기 → 하드웨어 재설정 → HOME", s.comm_err ? ", 버스 침묵 중" : "");
    return return_type::ERROR;
  }
  // D20 전체 멈춤: 어느 모터든 motor_fail_timeout_s 넘게 못 읽으면 ERROR
  //   (펌웨어는 못 읽은 모터의 옛날 값을 남겨 둠 → 그걸 최신 값처럼 컨트롤러에 넘기면
  //    조향이 다 돌았다고 착각하거나, 도는 바퀴를 멈췄다고 착각할 수 있음)
  if (s.comm_err) {
    const auto now = std::chrono::steady_clock::now();
    if (!comm_failing_) {
      comm_failing_ = true;
      comm_fail_since_ = now;
    }
    const double failing_s = std::chrono::duration<double>(now - comm_fail_since_).count();
    const std::string who = link_->missing_joints(~s.comm_err & 0xFFFF);
    if (failing_s > motor_fail_timeout_s_) {
      RCLCPP_ERROR(get_logger(), "모터 응답 없음 %.2f s: %s → 전체 멈춤 (선 · 전원 · ID 확인)", failing_s, who.c_str());
      return return_type::ERROR;
    }
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "모터 읽기 실패: %s (%.2f s 넘으면 멈춤)",
      who.c_str(), motor_fail_timeout_s_);
  } else {
    comm_failing_ = false;
  }
  // 배터리 (DIAG 가 새로 왔을 때만, 1 초마다). 평균이 정지 기준 아래로 2 번 연속이면 ERROR — 출발 순간 처짐 한 번으로는 안 멈춤
  if (link_->diag_count() != seen_diag_) {
    seen_diag_ = link_->diag_count();
    const auto & g = link_->last_diag();
    if (g.volt_mean_dv) {
      const double vmean = g.volt_mean_dv * 0.1, vmin = g.volt_min_dv * 0.1;
      battery_low_count_ = vmean < battery_stop_v_ ? battery_low_count_ + 1 : 0;
      if (battery_low_count_ >= 2) {
        RCLCPP_ERROR(get_logger(), "%s 배터리 %.1f V (정지 기준 %.1f V) → 전체 멈춤. 충전 · 교체 뒤 재설정",
          get_hardware_info().name.c_str(), vmean, battery_stop_v_);
        return return_type::ERROR;
      }
      if (vmin < battery_warn_v_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s 배터리 낮음: 평균 %.1f V · 최저 %.1f V (경고 %.1f / 정지 %.1f V)",
          get_hardware_info().name.c_str(), vmean, vmin, battery_warn_v_, battery_stop_v_);
      }
    }
  }
  // (R5) 모터가 스스로 멈춤 (과부하 · 과열 · 전압 → Hardware Error, 토크 꺼짐) → 바로 ERROR (D20 전체 멈춤)
  if (s.status & ntp::ST_HW_ERROR) {
    RCLCPP_ERROR(get_logger(), "모터 Hardware Error (과부하 · 과열 · 전압) → 전체 멈춤. 모터 재부팅 · 원인 확인 후 다시 켤 것");
    return return_type::ERROR;
  }
  // (R5) 목표 쓰기(Sync Write) 전송이 motor_fail_timeout_s 넘게 계속 실패 → ERROR
  //   읽기만 성공하면 comm_err 가 0 이라 예전에는 몰랐음 (모터는 마지막 목표를 계속 따름)
  if (s.status & ntp::ST_WRITE_FAIL) {
    const auto now = std::chrono::steady_clock::now();
    if (!write_failing_) {
      write_failing_ = true;
      write_fail_since_ = now;
    }
    if (std::chrono::duration<double>(now - write_fail_since_).count() > motor_fail_timeout_s_) {
      RCLCPP_ERROR(get_logger(), "모터 목표 쓰기가 %.2f s 넘게 실패 → 전체 멈춤 (버스 · 배선 확인)", motor_fail_timeout_s_);
      return return_type::ERROR;
    }
  } else {
    write_failing_ = false;
  }
  // 토크를 켜라고 했는데 펌웨어가 0.5 s 넘게 "켜짐" 이라고 안 함 → 모터에 토크가 안 들어감
  //   (R5) "켜짐" 은 이제 되읽은 Torque Enable 까지 반영 → 운전 중 모터가 토크를 잃어도 0.5 s 안에 여기로 옴
  const bool torque_ok = !torque_ || (s.status & ntp::ST_TORQUE);
  if (!torque_ok) {
    const auto now = std::chrono::steady_clock::now();
    if (!torque_waiting_) {
      torque_waiting_ = true;
      torque_wait_since_ = now;
    }
    if (std::chrono::duration<double>(now - torque_wait_since_).count() > 0.5) {
      RCLCPP_ERROR(get_logger(), "토크를 못 켬 (펌웨어가 모터 쓰기 실패를 알려 옴) → 멈춤");
      return return_type::ERROR;
    }
  } else {
    torque_waiting_ = false;
  }
  publish_states();
  if ((s.status & ntp::ST_HOST_TIMEOUT) && !(last_status_ & ntp::ST_HOST_TIMEOUT)) {
    RCLCPP_WARN(get_logger(), "펌웨어 경비견: 명령이 끊겨 바퀴를 세웠음 (Jetson 이 바빴나?)");
  }
  last_status_ = s.status;
  return return_type::OK;
}

return_type OpenCRHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  const auto t0 = std::chrono::steady_clock::now();
  struct Timer
  {
    DurationStats & st; std::chrono::steady_clock::time_point t0;
    ~Timer() {st.add(std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count());}
  } timer{write_time_, t0};
  report_timing();
  for (std::size_t i = 0; i < ifaces_.size(); ++i) {
    targets_[i] = get_command<double>(ifaces_[i].cmd);   // 컨트롤러가 아직 안 썼으면 NaN → "목표 없음"
  }
  if (!send(torque_, true)) {
    RCLCPP_ERROR(get_logger(), "명령을 못 보냄 (포트)");
    return return_type::ERROR;
  }
  return return_type::OK;
}

// ---------------------------------------------------------------- 도우미
// 10 s 마다 read/write 시간 요약 (평균 · 최대 · 5 ms 넘은 횟수). 넘은 적이 있으면 WARN
//   write_all 은 포트가 막히면 최대 약 20 ms 기다릴 수 있음 → 한 주기(10 ms)를 넘기는지 여기서 보임
void OpenCRHardware::report_timing()
{
  const auto now = std::chrono::steady_clock::now();
  if (std::chrono::duration<double>(now - stats_since_).count() < 10.0) {return;}
  const bool bad = read_time_.over > 0 || write_time_.over > 0;
  if (bad) {
    RCLCPP_WARN(get_logger(), "주기 시간 (10 s): read 평균 %.2f ms 최대 %.2f ms 초과 %u회 / write 평균 %.2f ms 최대 %.2f ms 초과 %u회 (예산 %.0f ms)",
      read_time_.mean_s() * 1e3, read_time_.max_s * 1e3, read_time_.over,
      write_time_.mean_s() * 1e3, write_time_.max_s * 1e3, write_time_.over, read_time_.budget_s * 1e3);
  } else {
    RCLCPP_DEBUG(get_logger(), "주기 시간 (10 s): read 최대 %.2f ms / write 최대 %.2f ms",
      read_time_.max_s * 1e3, write_time_.max_s * 1e3);
  }
  read_time_.reset();
  write_time_.reset();
  stats_since_ = now;
  // 보드 안 시간 (DIAG, 2026-10-16) — 펌웨어가 micros() 로 잰 값. PC · USB 시간은 안 섞임
  const DiagSummary d = link_->take_diag();
  if (d.before_config_packets) {
    // CONFIG 전 구간: 측정과 섞지 않고 따로 한 줄 (포트를 열기 전 USB 버림 · 설정 전 루프)
    RCLCPP_INFO(get_logger(), "%s 보드 CONFIG 전 구간 %u 개 (측정에 안 합침): 한 바퀴 %.0f/%u us (%u회, 10 ms 초과 %u) · USB 버림 %u/%u",
      get_hardware_info().name.c_str(), d.before_config_packets, d.before_loop.mean_us(), d.before_loop.max_us,
      d.before_loop.n, d.before_loop.fail, d.before_usb_tx.fail, d.before_usb_tx.n);
  }
  if (d.packets) {
    const bool warn = d.loop.fail > 0 || d.resync > 0 || d.sync_read.fail > 0 || d.single_read.fail > 0 ||
      d.usb_tx.fail > 0;
    const auto line = [](const DiagSummary::Stat & st) {
        char b[64];
        std::snprintf(b, sizeof(b), "%.0f/%u us (%u회, 실패 %u)", st.mean_us(), st.max_us, st.n, st.fail);
        return std::string(b);
      };
    const std::string msg = get_hardware_info().name + " 보드 안 (" + std::to_string(d.packets) + " s): 한 바퀴 " + line(d.loop) +
      " · 주기 다시 잡기 " + std::to_string(d.resync) + " · Sync Read " + line(d.sync_read) +
      " · Sync Write " + line(d.sync_write) + " · 개별 읽기 " + line(d.single_read) +
      " · 개별 쓰기 " + line(d.single_write) +
      (d.usb_tx.n ? " · USB 보내기 " + line(d.usb_tx) : std::string()) +
      (d.config_ms ? " · CONFIG 처리 " + std::to_string(d.config_ms) + " ms" : std::string()) +
      (d.fault_stop_us ? " · 고장 정지 시도 " + std::to_string(d.fault_stop_us) + " us" : std::string()) +
      (d.volt_mean_dv ? " · 배터리 " + std::to_string(d.volt_mean_dv / 10) + "." + std::to_string(d.volt_mean_dv % 10) +
        " V (최저 " + std::to_string(d.volt_min_dv / 10) + "." + std::to_string(d.volt_min_dv % 10) + ")" : std::string());
    if (warn) {
      RCLCPP_WARN(get_logger(), "%s", msg.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "%s", msg.c_str());
    }
  }
}

void OpenCRHardware::publish_states()
{
  const auto & s = link_->state();
  // (2026-10-16 R3) "지금의 측정" 이 아닌 관절은 NaN 으로 내보냄
  //   - 이번 STATE 에서 그 모터를 못 읽음 (comm_err 비트) — 펌웨어는 옛 값을 그대로 실어 보냄
  //   - STATE 자체가 measure_max_age_s 넘게 안 옴
  //   예전에는 comm_err 유예(motor_fail_timeout_s, 0.2 s) 동안 옛 값을 그대로 내보냈고, swerve_controller 가 그걸
  //   지금 시각의 /odom 으로 포장했음 (Codex 재현: 응답 끊긴 뒤 0.21 s 동안 새 시각 odom 11개)
  //   받는 쪽 약속: swerve_controller 는 NaN 이면 바퀴 0 + odom 안 냄, control_master 접힘 확인은 NaN 이면 "모름"
  //   ⚠ JTC 는 NaN 상태를 "오차 없음" 처럼 넘길 수 있음 → 바퀴를 푸는 판단은 JTC 결과만 믿지 않고 control_master 가 실측으로
  for (std::size_t i = 0; i < ifaces_.size(); ++i) {
    const bool fresh = link_->joint_measured(i, measure_max_age_s_);
    const auto & r = s.joints_valid ? s.joints[i] : JointReading{};
    if (!ifaces_[i].pos.empty()) {set_state(ifaces_[i].pos, fresh ? r.position : NaN);}
    if (!ifaces_[i].vel.empty()) {set_state(ifaces_[i].vel, fresh ? r.velocity : NaN);}
    if (!ifaces_[i].position_vel.empty()) {
      // Position-derived compensation never accepts STATE older than 50 ms,
      // even when measure_max_age_s is configured more permissively.
      set_state(ifaces_[i].position_vel,
        fresh && link_->joint_measured(i, 0.05) ? r.position_velocity : NaN);
    }
    // effort 칸에는 "전류 [A]" 를 넣음 (토크 N·m 아님). 물건을 쥐었나 · 걸렸나 보는 용도
    if (!ifaces_[i].eff.empty()) {set_state(ifaces_[i].eff, fresh ? r.current : NaN);}
  }
  if (!imu_name_.empty()) {
    // IMU 가 없으면 NaN → swerve_controller 의 GyroYaw 가 무시하고 바퀴 계산으로
    const bool ok = s.status & ntp::ST_IMU_OK;
    const double v[10] = {
      s.imu.quat[1], s.imu.quat[2], s.imu.quat[3], s.imu.quat[0],   // 펌웨어는 w,x,y,z → ROS 는 x,y,z,w
      s.imu.gyro[0], s.imu.gyro[1], s.imu.gyro[2],
      s.imu.acc[0], s.imu.acc[1], s.imu.acc[2]};
    for (int k = 0; k < 10; ++k) {set_state(imu_ifaces_[k], ok ? v[k] : NaN);}
  }
}

void OpenCRHardware::invalidate_states()
{
  for (const auto & f : ifaces_) {
    for (const std::string * n : {&f.pos, &f.vel, &f.eff, &f.position_vel}) {
      if (!n->empty()) {set_state(*n, NaN);}
    }
  }
  for (const auto & n : imu_ifaces_) {set_state(n, NaN);}
}

bool OpenCRHardware::send(bool torque, bool with_commands)
{
  if (!with_commands) {return link_->send_command(std::vector<double>(targets_.size(), NaN), torque);}
  return link_->send_command(targets_, torque);
}

}  // namespace hardware


PLUGINLIB_EXPORT_CLASS(hardware::OpenCRHardware, hardware_interface::SystemInterface)
