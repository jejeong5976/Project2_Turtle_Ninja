// opencr_hardware.cpp — ros2_control 하드웨어 플러그인 "hardware/OpenCRHardware" (보드 1장 = 하드웨어 1개)
//   통신 · 단위 변환은 opencr_link.hpp. 여기는 ros2_control 껍데기만.
//
// 생명 주기
//   on_configure : 포트 열기 → CONFIG/ACK → 첫 상태. 조향이 EEPROM 한계 밖이면 카운터 다시 세기 (steer_recover.hpp)
//   on_activate  : 토크 켜기 (목표 없음 → 펌웨어가 지금 위치를 목표로, 안 튐)
//   on_deactivate: 바퀴 0, 토크 유지 (끄면 팔이 떨어짐). 쉬는 팔로 꺼 둔 중이면 꺼 둔 채
//   on_error     : 바로 바퀴 0 + 상태 칸 NaN
//
// 쉬는 팔 (rest_torque.hpp): /<보드>/rest_torque 요청 동안 팔 보드 토크 끔, 실제 토크는 /<보드>/torque_state (10 Hz).
//   구독 · 타이머는 controller_manager 실행기 스레드 → read/write 와는 atomic 으로만
//
// read() 가 ERROR (→ 하드웨어 · 컨트롤러 멈춤): 상태가 state_timeout_s 동안 안 옴 · 모터 하나라도 motor_fail_timeout_s 넘게
//   못 읽음 (스워브에서 모터 하나 빼고 달리면 위험) · 보드 리셋 · 토크가 0.5 s 안에 안 켜짐 · 펌웨어 고장 잠금 ·
//   Hardware Error · 목표 쓰기 계속 실패 · 배터리 평균이 battery_stop_v 아래 2 번 연속
// 모터 온도 (TEMP 1 초마다): /diagnostics "<보드>/motor_temperature" (robot_cli 가 읽음), 로그만 — 멈추지 않음
// 복구: ERROR 뒤 자동으로 다시 켜지 않음. 팔을 받친 뒤 (CONFIG 가 토크를 끔) configure · activate

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware/opencr_link.hpp"
#include "hardware/rest_torque.hpp"
#include "hardware/steer_recover.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/bool.hpp"

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
  bool after_config(int mask);   // CONFIG ACK 뒤 확인 (대답 없는 모터 · 첫 STATE). false = 이미 ERROR 로그 + 포트 닫음
  SteerRecoverPlan check_pos_limits(bool recover_next);   // EEPROM 한계 로그 + 한계 밖 관절 (조향 자동 복구 계획)

  // 조향 카운터 자동 복구 (steer_recover.hpp): 바퀴 (속도 관절) 가 있는 보드만. param steer_auto_recover (기본 true)
  bool has_wheel_ = false;
  bool steer_auto_recover_ = true;

  std::unique_ptr<OpenCRLink> link_;
  std::vector<JointIfaces> ifaces_;
  std::vector<double> targets_;
  std::string port_;
  int baud_ = 1000000;
  double config_timeout_s_ = 8.0;
  double state_timeout_s_ = 0.5;
  double motor_fail_timeout_s_ = 0.2;   // 모터 하나라도 이만큼 못 읽으면 ERROR (전체 멈춤)
  double measure_max_age_s_ = 0.05;     // STATE 가 이보다 오래되면 그 값은 "지금의 측정" 이 아님 → NaN
  bool comm_failing_ = false;
  std::chrono::steady_clock::time_point comm_fail_since_{};
  bool write_failing_ = false;           // 펌웨어가 목표 쓰기 실패(ST_WRITE_FAIL)를 알려 오는 중
  // 배터리 3S 리튬폴리머: 셀당 3.6 V 경고 · 3.4 V 정지 (XM430 입력 하한 약 10 V 전에 멈춤)
  double battery_warn_v_ = 10.8;
  double battery_stop_v_ = 10.2;
  uint32_t seen_diag_ = 0;
  int battery_low_count_ = 0;
  // 모터 온도 [°C] 경고 · 위험 (로그만, 멈추지 않음). XM430 Temperature Limit 기본 80 °C 보다 낮게
  int temp_warn_c_ = 60;
  int temp_crit_c_ = 70;
  uint32_t seen_temp_ = 0;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;   // 하드웨어 노드가 없으면 null (로그만)
  void report_temps();
  std::string temps_text() const;   // "joint1 41, joint2 63, …" (못 읽음 "-") — 로그용
  std::chrono::steady_clock::time_point write_fail_since_{};
  bool torque_waiting_ = false;          // 토크 켜라고 한 뒤 펌웨어가 "켜짐" 이라고 하기를 기다리는 중
  std::chrono::steady_clock::time_point torque_wait_since_{};
  bool require_all_motors_ = true;
  bool torque_off_on_deactivate_ = false;
  bool torque_ = false;
  std::string imu_name_;   // URDF 의 <sensor name="imu"> (없으면 IMU 안 내보냄)
  std::vector<std::string> imu_ifaces_;   // "imu/orientation.x" … 10 칸 (매 주기 문자열을 새로 만들지 않게 미리)
  uint8_t last_status_ = 0;
  // 주기 시간 측정 (read · write 각각, 10 s 마다 로그)
  DurationStats read_time_, write_time_;
  std::chrono::steady_clock::time_point stats_since_{std::chrono::steady_clock::now()};
  void report_timing();

  // ---- 쉬는 팔 (rest_torque.hpp). 구독 · 타이머는 하드웨어 노드 (controller_manager 실행기 스레드) → 아래 atomic 만 같이 씀
  bool rest_eligible_ = false;               // 바퀴 (속도 관절) 없는 보드 + 하드웨어 노드 있음
  double rest_timeout_s_ = 1.0;              // 요청이 이만큼 안 오면 다시 켬 (control_master 0.1 s 마다)
  std::vector<bool> rest_gated_;             // 다시 켤 때 막을 관절 (위치 모드 팔 관절, 그리퍼 빼고)
  std::vector<double> rest_gate_tol_;        // 관절 단위 허용 (rest_gate_tol_rad × |scale|)
  std::vector<double> rest_meas_;            // 막기 · 한계 판단용 측정 (주기마다 할당하지 않게)
  std::vector<double> rest_lo_, rest_hi_, rest_margin_;   // URDF 위치 한계 (없으면 ±inf) · 여유 (rest_limit_margin_rad × |scale|)
  RestTorque rest_;                          // read/write 스레드만
  std::chrono::steady_clock::time_point rest_on_t_{};
  bool rest_gate_noted_ = false;             // 오래 막는 관절 한 번 알림
  std::atomic<bool> rest_req_{false};        // 마지막 요청 (구독 콜백이 씀)
  std::atomic<int64_t> rest_req_ns_{0};      // 그 받은 시각 [steady ns, 0 = 없음]
  std::atomic<bool> torque_actual_{false};   // STATE ST_TORQUE (read 가 씀, 타이머가 냄)
  std::atomic<int64_t> read_ns_{0};          // 마지막 정상 read [steady ns] — 멈춘 보드의 옛 값을 내지 않게
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr rest_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr torque_pub_;
  rclcpp::TimerBase::SharedPtr torque_timer_;
  bool board_clean() const;
  void rest_step();                          // write() 에서: 끄기 · 켜기 바꿈 + 옛 명령 막기
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
    temp_warn_c_ = std::stoi(param(hw, "temp_warn_c", "60"));
    temp_crit_c_ = std::stoi(param(hw, "temp_crit_c", "70"));
    rest_timeout_s_ = std::stod(param(hw, "rest_request_timeout_s", "1.0"));   // 쉬는 팔 요청 끊김 → 다시 켬
    steer_auto_recover_ = param(hw, "steer_auto_recover", "true") == "true";
    if (temp_warn_c_ >= temp_crit_c_) {
      RCLCPP_ERROR(get_logger(), "temp_warn_c %d 는 temp_crit_c %d 보다 낮아야 함", temp_warn_c_, temp_crit_c_);
      return CallbackReturn::ERROR;
    }

    for (const auto & j : info.joints) {
      const auto & p = j.parameters;
      JointSpec s;
      s.name = j.name;
      s.id = static_cast<uint8_t>(std::stoi(param(p, "id", "0")));
      const std::string mode = param(p, "mode", "position");
      if (mode == "velocity") {
        s.mode = pkt::MODE_VELOCITY;
      } else if (mode == "gripper") {
        s.mode = pkt::MODE_CURRENT_POSITION;
      } else if (mode == "position") {
        s.mode = pkt::MODE_POSITION;
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
      // 영점 눈금: 관절 0 일 때 모터 눈금. 기본 2048. 위치 모드 한 바퀴(0~4095) 안이어야 함.
      //   바퀴(속도 모드)는 위치가 여러 바퀴 누적이라 영점이 의미 없음 → 2048 이 아니면 거부
      const int zero_tick = std::stoi(param(p, "zero_tick", "2048"));
      if (zero_tick < 0 || zero_tick > 4095 || (s.mode == pkt::MODE_VELOCITY && zero_tick != 2048)) {
        RCLCPP_ERROR(get_logger(), "%s: zero_tick %d 은 위치 모드에서 0~4095 만 (속도 모드는 쓰지 않음)", j.name.c_str(), zero_tick);
        return CallbackReturn::ERROR;
      }
      s.zero_rad = (zero_tick - 2048) * (2.0 * M_PI / 4096.0);

      // 명령 칸은 1개: 바퀴 = velocity, 나머지 = position
      const std::string want = (s.mode == pkt::MODE_VELOCITY) ? hardware_interface::HW_IF_VELOCITY
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
  if (specs.size() > pkt::MAX_JOINTS) {
    RCLCPP_ERROR(get_logger(), "모터 %zu 개 — 보드 1장에 최대 %d 개", specs.size(), pkt::MAX_JOINTS);
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
  // 쉬는 팔: 다시 켤 때 막을 관절 = 위치 모드 팔 관절 (그리퍼 · 바퀴 빼고) (허용은 관절 단위: 그리퍼 m = rad × scale). 바퀴가 있는 보드는 안 씀
  double gate_tol_rad = 0.02, limit_margin_rad = 0.02;
  try {
    gate_tol_rad = std::stod(param(hw, "rest_gate_tol_rad", "0.02"));
    limit_margin_rad = std::stod(param(hw, "rest_limit_margin_rad", "0.02"));
  } catch (const std::exception &) {
    RCLCPP_WARN(get_logger(), "rest_gate_tol_rad · rest_limit_margin_rad 를 못 읽음 → 0.02 · 0.02 rad");
  }
  bool has_wheel = false;
  for (const auto & s : specs) {
    has_wheel = has_wheel || s.mode == pkt::MODE_VELOCITY;
    // 그리퍼는 막지 않음: 막으면 열기 명령이 무시되어 닫힌 채 내려감. 쉼 = 빈손이라 옛 명령으로 가도 튈 것이 없음
    const bool is_gripper = s.mode == pkt::MODE_CURRENT_POSITION || s.name.find("gripper") != std::string::npos;
    rest_gated_.push_back(s.mode != pkt::MODE_VELOCITY && !is_gripper);
    rest_gate_tol_.push_back(gate_tol_rad * std::fabs(s.scale));
    // URDF 위치 한계 (컨트롤러가 측정 위치를 이 한계로 봄 — 밖이면 그 컨트롤러가 꺼짐)
    const auto lim = info.limits.find(s.name);
    const bool has = lim != info.limits.end() && lim->second.has_position_limits;
    rest_lo_.push_back(has ? lim->second.min_position : -std::numeric_limits<double>::infinity());
    rest_hi_.push_back(has ? lim->second.max_position : std::numeric_limits<double>::infinity());
    rest_margin_.push_back(limit_margin_rad * std::fabs(s.scale));
  }
  rest_meas_.assign(specs.size(), NaN);
  has_wheel_ = has_wheel;
  link_ = std::make_unique<OpenCRLink>(specs, bus_watchdog);
  // 온도 → /diagnostics: controller_manager 가 하드웨어마다 만들어 주는 노드로 (rqt_robot_monitor · robot_cli 가 보는 표준 토픽)
  if (const auto node = get_node()) {
    diag_pub_ = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", rclcpp::QoS(10));
    // 쉬는 팔 요청 · 실제 토크 알림 (같은 노드 = controller_manager 실행기. read/write 와는 atomic 으로만)
    if (!has_wheel && !specs.empty()) {
      const std::string base = "/" + info.name;
      rest_sub_ = node->create_subscription<std_msgs::msg::Bool>(base + "/rest_torque", rclcpp::QoS(1).reliable(),
        [this](std_msgs::msg::Bool::ConstSharedPtr m) {
          rest_req_.store(m->data);
          rest_req_ns_.store(std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
        });
      torque_pub_ = node->create_publisher<std_msgs::msg::Bool>(base + "/torque_state", rclcpp::QoS(1).reliable());
      torque_timer_ = node->create_wall_timer(std::chrono::milliseconds(100), [this]() {
          const int64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
          const int64_t r = read_ns_.load();
          if (r == 0 || now_ns - r > 500000000) {return;}   // read 가 0.5 s 넘게 없음 (꺼짐 · ERROR) → 옛 값 안 냄
          std_msgs::msg::Bool b;
          b.data = torque_actual_.load();
          torque_pub_->publish(b);
        });
      rest_eligible_ = true;
      RCLCPP_INFO(get_logger(), "쉬는 팔: %s/rest_torque 받으면 토크 끔 (끊김 %.1f s → 다시 켬), 실제 토크 → %s/torque_state",
        base.c_str(), rest_timeout_s_, base.c_str());
    }
  } else {
    RCLCPP_WARN(get_logger(), "하드웨어 노드가 없어 모터 온도를 /diagnostics 로 못 냄 (로그로만) · 쉬는 팔 안 씀");
  }
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
  if (!after_config(link_->configure(config_timeout_s_))) {return CallbackReturn::ERROR;}
  const bool may_recover = steer_auto_recover_ && has_wheel_;   // 팔 보드 (바퀴 없음) 는 안 함
  const SteerRecoverPlan plan = check_pos_limits(may_recover);
  if (!may_recover || plan.mask == 0) {return CallbackReturn::SUCCESS;}   // 평소 (전부 한계 안)

  // ---- 조향 카운터 자동 복구 (steer_recover.hpp). 토크는 CONFIG 때부터 꺼져 있고 그대로 → 모터는 안 움직임
  const auto & js = link_->joints();
  std::vector<std::pair<int32_t, int32_t>> lim;
  link_->pos_limits(lim);
  for (const auto & it : plan.items) {
    if (it.fixable) {RCLCPP_WARN(get_logger(), "조향 자동 복구 ① 대상: %s", steer_recover_text(js, lim, it).c_str());}
  }
  RCLCPP_WARN(get_logger(), "조향 자동 복구 ② CONFIG (대상만 속도 모드, 토크 꺼짐 그대로 — 모터 안 움직임)…");
  const int m1 = link_->configure(config_timeout_s_, plan.mask);
  if (m1 < 0) {
    RCLCPP_ERROR(get_logger(), "조향 자동 복구 ②: OpenCR 가 %.0f s 동안 대답 없음 → 멈춤", config_timeout_s_);
    link_->close();
    return CallbackReturn::ERROR;
  }
  if ((m1 & plan.mask) != plan.mask) {
    RCLCPP_WARN(get_logger(), "조향 자동 복구 ②: 모드 바꾸기 실패한 모터 %s (그래도 원래 CONFIG 로 돌아감)",
      link_->missing_joints(m1 | (~plan.mask & 0xFFFF)).c_str());
  }
  RCLCPP_WARN(get_logger(), "조향 자동 복구 ③ 원래 CONFIG (위치 모드로 바뀌는 순간 모터가 한 바퀴 안 값으로 다시 셈)…");
  if (!after_config(link_->configure(config_timeout_s_))) {return CallbackReturn::ERROR;}
  // ④ 확인: 새 STATE 의 눈금 = 예상? 한계 안?
  link_->pos_limits(lim);
  bool all_ok = true;
  for (const auto & it : plan.items) {
    if (!it.fixable) {continue;}
    const bool meas = link_->joint_measured(it.idx, 1.0);
    const double q = meas ? link_->state().joints[it.idx].position : NaN;
    const long t = std::isfinite(q) ? motor_tick(js[it.idx], q) : 0;
    const bool in = std::isfinite(q) && it.idx < lim.size() && t >= lim[it.idx].first && t <= lim[it.idx].second;
    all_ok = all_ok && in;
    if (!std::isfinite(q)) {
      RCLCPP_ERROR(get_logger(), "조향 자동 복구 ④ %s: 다시 센 값을 못 읽음", js[it.idx].name.c_str());
    } else if (in) {
      RCLCPP_WARN(get_logger(), "조향 자동 복구 ④ %s: %ld → %ld 눈금 (관절 %+.0f° → %+.0f°, 예상과 %+ld 눈금) 한계 안 → 평소대로 켬 "
        "(토크 켤 때 목표 = 지금 위치, 바퀴 안 돎)", js[it.idx].name.c_str(), it.tick, t, tick_to_joint(js[it.idx], it.tick) * 180.0 / M_PI,
        q * 180.0 / M_PI, steer_recover_error_ticks(it.after_tick, t));
    } else {
      RCLCPP_ERROR(get_logger(), "조향 자동 복구 ④ %s: 다시 센 값 %ld 눈금 (%+.0f°) 도 한계 밖 → 못 고침 (모터가 다시 안 셈?). "
        "끈 채로 손으로 한계 안 (조향이면 똑바로) 으로 돌려 놓을 것", js[it.idx].name.c_str(), t, q * 180.0 / M_PI);
    }
  }
  RCLCPP_WARN(get_logger(), "조향 자동 복구 끝: %s", all_ok ? "성공" : "실패 (켜면 예전처럼 토크를 못 켬 → ERROR)");
  check_pos_limits(false);   // 다시 센 뒤 한계 한 줄 (아직 한계 밖인 관절은 손으로 돌리라는 경고)
  return CallbackReturn::SUCCESS;
}

// CONFIG ACK 뒤: 대답 없는 모터 · 첫 STATE · 준비 끝 로그
bool OpenCRHardware::after_config(int mask)
{
  if (mask < 0) {
    RCLCPP_ERROR(get_logger(),
      "OpenCR 가 %.0f s 동안 대답 없음 — 펌웨어(firmware/opencr)가 올라가 있나? 포트가 맞나?", config_timeout_s_);
    link_->close();
    return false;
  }
  const std::string missing = link_->missing_joints(mask);
  if (!missing.empty()) {
    if (require_all_motors_) {
      RCLCPP_ERROR(get_logger(), "대답 없거나 설정에 실패한 모터: %s — 전원 · 선 · ID 확인 (일부만 시험하려면 require_all_motors:=false)",
        missing.c_str());
      link_->close();
      return false;
    }
    RCLCPP_WARN(get_logger(), "대답 없거나 설정 실패한 모터 (무시하고 진행): %s", missing.c_str());
  }
  if (!link_->wait_for_state(1.0)) {
    RCLCPP_ERROR(get_logger(), "악수는 됐는데 상태(STATE)가 안 옴");
    link_->close();
    return false;
  }
  publish_states();
  {
    const std::string tn = link_->take_turn_note();
    if (!tn.empty()) {RCLCPP_WARN(get_logger(), "여러 바퀴 관절 바로잡음 — %s", tn.c_str());}
  }
  RCLCPP_INFO(get_logger(), "OpenCR 준비 끝 (IMU %s)",
    (link_->state().status & pkt::ST_IMU_OK) ? "정상" : "없음 — odom 회전은 바퀴 계산으로");
  return true;
}

// 모터 EEPROM 위치 한계 (CONFIG 바로 뒤 옴) 로그 + 지금 위치가 밖이면 토크를 켜기 전에 알림
//   recover_next = 바로 뒤 조향 자동 복구를 할 수 있음 (경고 문구만 다름). 돌려줌 = 한계 밖 관절 (복구 계획)
SteerRecoverPlan OpenCRHardware::check_pos_limits(bool recover_next)
{
  SteerRecoverPlan plan;
  std::vector<std::pair<int32_t, int32_t>> lim;
  if (link_->pos_limits(lim)) {
    std::vector<double> q;   // 첫 STATE 에서 못 읽은 모터는 펌웨어 초깃값 0 → 모름 (NaN)
    for (std::size_t i = 0; i < link_->state().joints.size(); ++i) {
      q.push_back(link_->joint_measured(i, 1.0) ? link_->state().joints[i].position : NaN);
    }
    std::string bad;
    const std::string line = pos_limits_text(link_->joints(), lim, q, bad);
    if (!line.empty()) {RCLCPP_INFO(get_logger(), "모터 위치 한계 (EEPROM): %s", line.c_str());}
    plan = plan_steer_recover(link_->joints(), lim, q);
    if (!bad.empty()) {
      if (recover_next && plan.mask) {
        RCLCPP_WARN(get_logger(), "%s → 조향 자동 복구 (모터 카운터만 한 바퀴 안으로 다시 셈, 안 움직임) 시도", bad.c_str());
      } else {
        RCLCPP_WARN(get_logger(), "%s → 이대로 켜면 토크를 못 켬. 끈 채로 손으로 한계 안 (조향이면 똑바로) 으로 돌려 놓을 것", bad.c_str());
      }
    }
  } else {
    RCLCPP_INFO(get_logger(), "모터 위치 한계: 펌웨어가 안 보냄 (위치 한계를 안 보내는 옛 펌웨어)");
  }
  return plan;
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
  rest_.reset();   // 쉬는 팔: 켤 때는 늘 토크 켬 (요청이 아직 신선하면 다음 write 가 다시 끔)
  if (!send(true, false)) {return CallbackReturn::ERROR;}
  RCLCPP_INFO(get_logger(), "토크 켬");
  return CallbackReturn::SUCCESS;
}

CallbackReturn OpenCRHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
  // 바퀴는 0 (목표 없음). 팔은 기본으로 버팀 — 끄면 무게 때문에 떨어짐
  //   쉬는 팔로 꺼 둔 중이면 꺼 둔 채 (팔은 받침에 접혀 있음 — 지금 켜면 처진 자리에서 버티며 배터리만 씀)
  torque_ = rest_.resting() ? false : !torque_off_on_deactivate_;
  send(torque_, false);
  RCLCPP_INFO(get_logger(), "멈춤 (바퀴 0, 토크 %s)", torque_ ? "유지" : (rest_.resting() ? "끈 채 (쉬는 팔)" : "끔"));
  return CallbackReturn::SUCCESS;
}

// read()/write() 가 ERROR 를 돌려주면 ros2_control 이 부름 → 펌웨어 경비견(0.3 s)을 기다리지 말고 바로 바퀴 0
CallbackReturn OpenCRHardware::on_error(const rclcpp_lifecycle::State &)
{
  send(torque_, false);   // 목표 전부 "없음" = 바퀴 0, 팔 · 조향은 그 자리에서 버팀
  // 상태 칸 전부 NaN: ERROR 뒤 read() 가 안 불려 옛 값이 "지금 측정" 으로 나가지 않게
  //   (control_master 는 NaN → STOP + HOME 무효화, swerve_controller 는 바퀴 0)
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
  {   // 여러 바퀴 모드 (J1 · 그리퍼) 를 바로잡았으면 한 번 알림
    const std::string tn = link_->take_turn_note();
    if (!tn.empty()) {RCLCPP_WARN(get_logger(), "여러 바퀴 관절 바로잡음 — %s", tn.c_str());}
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
  // 펌웨어 고장 잠금 (목표 쓰기 계속 실패 → 바퀴 세우고 잠금, CONFIG 로만 풀림) → 바로 ERROR.
  //   아래 comm_err 유예보다 먼저 봄
  if (s.status & pkt::ST_FAULT_LATCHED) {
    RCLCPP_ERROR(get_logger(), "펌웨어 고장 잠금 (모터 목표 쓰기 실패 → 바퀴 정지%s) → 전체 멈춤. "
      "버스 · 배선 확인 → 팔 받치기 → 하드웨어 재설정 → HOME", s.comm_err ? ", 버스 침묵 중" : "");
    return return_type::ERROR;
  }
  // 어느 모터든 motor_fail_timeout_s 넘게 못 읽으면 전체 ERROR (펌웨어는 못 읽은 모터의 옛 값을 그대로 실어 보냄)
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
  report_temps();   // 모터 온도 (TEMP 가 새로 왔을 때만, 1 초마다)
  // 모터가 스스로 멈춤 (과부하 · 과열 · 전압 → Hardware Error, 토크 꺼짐) → 바로 전체 ERROR
  if (s.status & pkt::ST_HW_ERROR) {
    pkt::TorqueFail f;   // 펌웨어는 토크가 꺼진 모터 · Hardware Error 비트를 이 STATE 앞에 보냄 (TF_LOST)
    double age = 1e9;
    const bool lost = link_->last_torque_fail(f, age) && age < 1.5 && f.step == pkt::TF_LOST;
    RCLCPP_ERROR(get_logger(), "모터 Hardware Error (과부하 · 과열 · 전압) → 전체 멈춤. 모터 재부팅 · 원인 확인 후 다시 켤 것 "
      "(마지막 온도 °C: %s)%s%s", temps_text().c_str(), lost ? " · " : "", lost ? torque_fail_text(link_->joints(), f).c_str() : "");
    return return_type::ERROR;
  }
  // 목표 쓰기(Sync Write) 가 motor_fail_timeout_s 넘게 계속 실패 → ERROR (읽기만 되면 comm_err 가 0 이라 따로 봄)
  if (s.status & pkt::ST_WRITE_FAIL) {
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
  // 토크를 켜라고 했는데 펌웨어가 0.5 s 넘게 "켜짐" 이라고 안 함 (되읽은 Torque Enable 기준 → 운전 중 토크를 잃어도 여기로 옴)
  const bool torque_ok = !torque_ || (s.status & pkt::ST_TORQUE);
  if (!torque_ok) {
    const auto now = std::chrono::steady_clock::now();
    if (!torque_waiting_) {
      torque_waiting_ = true;
      torque_wait_since_ = now;
    }
    if (std::chrono::duration<double>(now - torque_wait_since_).count() > 0.5) {
      // 펌웨어는 실패 이유 (TORQUE_FAIL) 를 0.5 s 마다 보냄 → 기다린 동안 받은 것이면 같이 찍음
      pkt::TorqueFail f;
      double age = 1e9;
      const bool why = link_->last_torque_fail(f, age) && age < 1.5;
      std::vector<std::pair<int32_t, int32_t>> lim;
      const bool new_fw = link_->pos_limits(lim);   // 위치 한계 상자를 보내는 펌웨어 = 이유 상자도 보냄
      RCLCPP_ERROR(get_logger(), "토크를 못 켬: %s → 멈춤",
        why ? torque_fail_text(link_->joints(), f).c_str() :
        (new_fw ? "펌웨어가 모터 쓰기 실패를 알려 옴 (이유 상자를 못 받음 — USB 가 바쁨)" : "펌웨어가 모터 쓰기 실패를 알려 옴 (이유 없음 = 옛 펌웨어)"));
      return return_type::ERROR;
    }
  } else {
    torque_waiting_ = false;
  }
  publish_states();
  if ((s.status & pkt::ST_HOST_TIMEOUT) && !(last_status_ & pkt::ST_HOST_TIMEOUT)) {
    RCLCPP_WARN(get_logger(), "펌웨어 경비견: 명령이 끊겨 바퀴를 세웠음 (Jetson 이 바빴나?)");
  }
  last_status_ = s.status;
  // 쉬는 팔: 실제 토크 (펌웨어 되읽기 포함) → /<보드>/torque_state 타이머가 냄
  torque_actual_.store((s.status & pkt::ST_TORQUE) != 0);
  read_ns_.store(std::chrono::duration_cast<std::chrono::nanoseconds>(t0.time_since_epoch()).count());
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
  rest_step();   // 쉬는 팔: 토크 끄기 · 다시 켜기 + 다시 켠 뒤 옛 명령 막기 (막힌 관절 목표 → NaN = 지금 위치에서 버팀)
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
  // 보드 안 시간 (DIAG) — 펌웨어가 micros() 로 잰 값. PC · USB 시간은 안 섞임
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
        " V (최저 " + std::to_string(d.volt_min_dv / 10) + "." + std::to_string(d.volt_min_dv % 10) + ")" : std::string()) +
      (link_->temp_count() ? " · 온도 °C " + temps_text() : std::string(" · 온도 상자 없음 (옛 펌웨어 → 다시 올릴 것)"));
    if (warn) {
      RCLCPP_WARN(get_logger(), "%s", msg.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "%s", msg.c_str());
    }
  }
}

// 모터 온도: TEMP 가 새로 왔을 때 (1 초마다) /diagnostics 한 줄 + 기준을 넘으면 로그
void OpenCRHardware::report_temps()
{
  if (link_->temp_count() == seen_temp_) {return;}
  seen_temp_ = link_->temp_count();
  const TempReport r = temp_report(link_->joints(), link_->temperatures(), temp_warn_c_, temp_crit_c_);
  const std::string & board = get_hardware_info().name;
  if (r.level == 2) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "%s 모터 온도 %s — 부하를 줄이고 식힐 것 (팔은 받침에 접기). "
      "모터는 Temperature Limit(기본 80 °C)에서 스스로 토크를 끔", board.c_str(), r.message.c_str());
  } else if (r.level == 1) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "%s 모터 온도 %s", board.c_str(), r.message.c_str());
  }
  if (!diag_pub_) {return;}
  diagnostic_msgs::msg::DiagnosticArray a;
  a.header.stamp = get_clock()->now();
  diagnostic_msgs::msg::DiagnosticStatus st;
  st.level = static_cast<uint8_t>(r.level);
  st.name = board + TEMP_DIAG_SUFFIX;
  st.hardware_id = board;
  st.message = r.message;
  for (const auto & [k, v] : r.values) {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = k;
    kv.value = v;
    st.values.push_back(kv);
  }
  a.status.push_back(st);
  diag_pub_->publish(a);
}

std::string OpenCRHardware::temps_text() const
{
  std::string out;
  const auto & c = link_->temperatures();
  for (std::size_t i = 0; i < c.size(); ++i) {
    out += (out.empty() ? "" : ", ") + link_->joints()[i].name + " " + (c[i] < 0 ? std::string("-") : std::to_string(c[i]));
  }
  return out;
}

void OpenCRHardware::publish_states()
{
  const auto & s = link_->state();
  // "지금의 측정" 이 아닌 관절 (이번 STATE 에서 못 읽음 · STATE 가 measure_max_age_s 넘게 안 옴) 은 NaN
  //   — 옛 값이 새 시각의 /odom 이 되지 않게. 받는 쪽: swerve_controller 는 바퀴 0 + odom 안 냄, control_master 는 "모름"
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
    const bool ok = s.status & pkt::ST_IMU_OK;
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

// 쉬는 팔: 토크를 꺼도 되는 보드 상태 (모터 읽기 · 쓰기 실패 · 토크 켜기 기다림 · Hardware Error · 펌웨어 잠금 없음)
bool OpenCRHardware::board_clean() const
{
  return !comm_failing_ && !write_failing_ && !torque_waiting_ &&
         !(last_status_ & (pkt::ST_HW_ERROR | pkt::ST_FAULT_LATCHED | pkt::ST_WRITE_FAIL));
}

// 쉬는 팔 (write 안, targets_ 를 채운 뒤 · 보내기 전): 요청 → 토크 끄기 · 다시 켜기, 다시 켠 뒤 옛 명령 막기
void OpenCRHardware::rest_step()
{
  if (!rest_eligible_) {return;}
  const auto now = std::chrono::steady_clock::now();
  const int64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
  const int64_t rx_ns = rest_req_ns_.load();
  const double age = rx_ns == 0 ? NaN : (now_ns - rx_ns) * 1e-9;
  const bool req = rest_req_.load();
  // 끄는 것은 깨끗할 때만. 꺼 둔 동안의 짧은 읽기 실패로 껐다 켰다 하지 않음 (오래가면 read 가 ERROR → 이 함수가 안 불림)
  const bool clean = board_clean() || rest_.resting();
  const std::string & board = get_hardware_info().name;
  for (std::size_t i = 0; i < rest_meas_.size(); ++i) {
    rest_meas_[i] = link_->joint_measured(i, measure_max_age_s_) ? link_->state().joints[i].position : NaN;
  }
  const int near = near_limit(rest_meas_, rest_lo_, rest_hi_, rest_margin_);   // URDF 한계 가까운 관절 (끈 동안 · 끄기 전)
  const int limit_before = rest_.limit_joint();
  const auto change = rest_.step(rest_wanted(req, age, rest_timeout_s_, clean), rest_gated_, near, req);
  if (rest_.limit_joint() >= 0 && limit_before < 0) {
    const auto j = static_cast<std::size_t>(rest_.limit_joint());
    RCLCPP_WARN(get_logger(), "쉬는 팔: %s %s %.4f 이 URDF 한계 [%.4f, %.4f] 에 %.3f 안 → %s (밖이면 컨트롤러가 꺼짐). "
      "control_master 가 요청을 끝낼 때까지 켠 채", board.c_str(), link_->joints()[j].name.c_str(), rest_meas_[j], rest_lo_[j], rest_hi_[j],
      rest_margin_[j], change == RestTorque::Change::ON ? "토크 다시 켬 (그 자리에서 버팀)" : "토크 안 끔");
  }
  switch (change) {
    case RestTorque::Change::OFF:
      torque_ = false;
      RCLCPP_INFO(get_logger(), "쉬는 팔: %s 토크 끔 (control_master 요청 — 접힘 · 빈손 · 바퀴 차례). 관절 측정은 그대로 나감", board.c_str());
      break;
    case RestTorque::Change::ON: {
      torque_ = true;
      rest_on_t_ = now;
      rest_gate_noted_ = false;
      char why[64];
      if (rest_.limit_joint() >= 0) {std::snprintf(why, sizeof(why), "관절 한계 가까움");}
      else if (!req) {std::snprintf(why, sizeof(why), "요청 끝");}
      else if (!clean) {std::snprintf(why, sizeof(why), "보드 이상");}
      else {std::snprintf(why, sizeof(why), "요청 %.1f s 끊김", std::isfinite(age) ? age : -1.0);}
      RCLCPP_INFO(get_logger(), "쉬는 팔: %s 토크 다시 켬 (%s) — 펌웨어가 지금 위치를 목표로 켬 · 컨트롤러의 옛 명령은 지금 위치에 올 때까지 안 보냄",
        board.c_str(), why);
      break;
    }
    case RestTorque::Change::NONE:
      break;
  }
  if (!rest_.gate().holding()) {return;}
  rest_.gate().filter(targets_, rest_meas_, rest_gate_tol_);
  const double held_s = std::chrono::duration<double>(now - rest_on_t_).count();
  if (!rest_.gate().holding()) {
    RCLCPP_INFO(get_logger(), "쉬는 팔: %s 옛 명령 막기 끝 (켠 뒤 %.2f s) — 모든 관절이 새 명령을 따름", board.c_str(), held_s);
  } else if (!rest_gate_noted_ && held_s > 3.0) {
    // 아직 막힌 관절 = 새 궤적이 아직 안 옴 (그리퍼는 막지 않음)
    rest_gate_noted_ = true;
    std::string held;
    for (std::size_t i = 0; i < ifaces_.size(); ++i) {
      if (!rest_.gate().holding(i)) {continue;}
      char b[96];
      std::snprintf(b, sizeof(b), "%s%s 명령 %.4f · 측정 %.4f", held.empty() ? "" : ", ", link_->joints()[i].name.c_str(),
        get_command<double>(ifaces_[i].cmd), rest_meas_[i]);
      held += b;
    }
    RCLCPP_INFO(get_logger(), "쉬는 팔: %s 켠 뒤 %.0f s — 아직 지금 위치에서 버티는 관절 (새 명령을 기다림): %s", board.c_str(), held_s, held.c_str());
  }
}

bool OpenCRHardware::send(bool torque, bool with_commands)
{
  if (!with_commands) {return link_->send_command(std::vector<double>(targets_.size(), NaN), torque);}
  return link_->send_command(targets_, torque);
}

}  // namespace hardware


PLUGINLIB_EXPORT_CLASS(hardware::OpenCRHardware, hardware_interface::SystemInterface)
