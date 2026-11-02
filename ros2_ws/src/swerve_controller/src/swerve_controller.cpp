// swerve_controller.cpp — ros2_control 컨트롤러 "swerve_controller/SwerveController"
// =====================================================================
//   Nav2 ─/cmd_vel─▶ [이 컨트롤러, controller_manager 의 100 Hz 루프 안] ─▶ 하드웨어
//   왜 컨트롤러 안인가 (D8): 노드가 계산해서 토픽으로 넘기는 구조면 그 노드가 멈췄을 때 받는 쪽이 마지막 속도를
//   붙잡고 계속 돌 수 있음. 여기서는 /cmd_vel 이 끊기면 바로 이 루프에서 목표 0 — 중간에 "붙잡고 있는" 단계가 없음
//
// 하는 일 (계산은 전부 swerve.hpp — 여기는 ROS 와 이어 붙이기만)
//   받는 것 : /cmd_vel (TwistStamped, Nav2) · /control/motion_mode (control_master — 바퀴 차례인가, 10 Hz)
//             차례가 0.5 s 넘게 안 오면 잠금 (control_master 가 죽거나 신호가 끊기면 바퀴도 섬 — 모르면 안 움직임)
//             /imu_sensor_broadcaster/imu (자이로 → odom 회전)
//             모터 상태 칸: steer_xx_joint/position·velocity, drive_xx_joint/velocity
//   쓰는 것 : 모터 명령 칸: steer_xx_joint/position (rad), drive_xx_joint/velocity (rad/s)
//   보내는 것: /odom + TF odom → base_footprint (50 Hz, 제어 루프 밖 타이머에서. 모터 상태를 못 읽으면 안 보냄)
//             ~/commands (JointState, 50 Hz): 이 루프가 하드웨어에 쓴 명령 (조향 position [rad] · 구동 velocity [rad/s]) — 정지 시험 기록용
//             /control/motion_mode_ack (20 Hz): "이 요청 번호의 차례를 반영했음" → control_master 가 이걸 받아야 팔 시작
//
// 실시간 규칙: update() 안에서는 기다리지 않는다 — 토픽에서 온 값은 try_lock 으로만 꺼내고,
//             발행은 별도 타이머가 함 (update 는 최신 odom 을 칸에 넣어 두기만)

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "swerve_controller/swerve.hpp"
#include "interfaces/msg/motion_mode.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#if __has_include("tf2_ros/transform_broadcaster.hpp")
#include "tf2_ros/transform_broadcaster.hpp"
#else
#include "tf2_ros/transform_broadcaster.h"
#endif
#include "pluginlib/class_list_macros.hpp"

namespace swerve_controller
{

class SwerveController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  bool read_modules(std::vector<ModuleState> & out) const;
  void write_modules(const std::vector<ModuleState> & m);
  void publish_odom();

  // 설정
  std::vector<std::string> names_, steer_joints_, drive_joints_;
  std::unique_ptr<SwerveDrive> drive_;
  bool require_motion_mode_ = true;
  bool publish_tf_ = true;
  bool use_imu_ = true;
  bool position_rate_ = false;
  double still_linear_ = 0.01, still_angular_ = 0.05;
  // 바퀴별 속도 배율 (2026-10-28, 기본 1): 쓸 때 곱하고 읽을 때 나눔 → 실제 이동과 odom 이 같이 맞음.
  //   카펫에서 바퀴마다 파묻힘(1~2 mm)이 달라 실제 굴림 반지름이 다른 것 보정 (정면 유지 끄면 1 m 에 2.1° 왼쪽으로 돎)
  std::vector<double> wheel_gain_;
  double max_wheel_speed_ = 0.0;
  std::string odom_frame_, base_frame_;

  // 토픽 → 루프 (짧게 잠그고 복사)
  std::mutex cmd_mtx_;
  BodyTwist cmd_in_{0.0, 0.0, 0.0};
  double cmd_in_time_ = -1e9;
  bool cmd_in_new_ = false;
  bool cmd_in_bad_ = false;
  // 차례: 한 메시지의 (mode, 번호, 받은 시각) 을 **한 묶음으로** 주고받음
  //   칸마다 따로 atomic 이면 "옛 mode + 새 번호" 처럼 섞여 읽힐 수 있어서 (잠금 답이 거짓이 될 수 있음)
  struct ModeIn
  {
    int mode = -1;          // MotionMode.mode, -1 = 아직 모름
    uint32_t id = 0;
    double time = -1e9;     // 받은 시각 (보낸 쪽 시계와 상관없이 "끊겼나" 를 보려고)
  };
  std::mutex mode_mtx_;
  ModeIn mode_in_;          // 콜백이 씀 (mode_mtx_)
  ModeIn mode_snap_;        // update 만 씀: 못 잠근 주기는 지난 묶음 그대로
  // 제어 루프가 반영한 (요청 번호, DRIVE 여부) → ack 타이머. 한 64비트 값에 같이 넣어서 섞여 읽히지 않게
  //   비트 0 = DRIVE 반영, 비트 1~32 = 요청 번호
  std::atomic<uint64_t> applied_{0};
  std::mutex imu_mtx_;
  GyroYaw gyro_;
  std::atomic<bool> base_still_{false};
  // 2026-11-01 움직이는 동안 자이로가 덜 느끼는 회전 (실물: 0.09°/s, 5 분에 29° — SLAM 이 고치던 양).
  //   pose_guard 가 map→odom 방향 고침 속도로 재서 imu_motion_bias_topic 으로 보냄 → 바퀴가 움직일 때만 자이로에 더함
  std::atomic<double> motion_bias_{0.0};
  std::atomic<double> motion_bias_t_{-1e9};   // 마지막으로 받은 시각 [s] — motion_bias_timeout_s_ 넘으면 안 씀 (pose_guard 가 꺼져도 남지 않게)
  double motion_bias_max_ = 0.005, motion_bias_timeout_s_ = 5.0;
  double last_lin_speed_ = 0.0;

  // 루프 → 발행 타이머
  std::mutex odom_mtx_;
  nav_msgs::msg::Odometry odom_msg_;
  bool odom_ready_ = false;
  sensor_msgs::msg::JointState cmd_msg_;   // 마지막으로 쓴 명령 (odom_mtx_)
  bool cmd_msg_new_ = false;
  std::vector<ModuleState> meas_buf_;   // update() 의 측정값 (크기는 처음 한 번만 잡힘)

  std::vector<double> last_steer_;
  double last_time_s_ = -1.0;
  bool was_parking_ = false;

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_cmd_;
  rclcpp::Subscription<interfaces::msg::MotionMode>::SharedPtr sub_mode_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_motion_bias_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_cmd_;
  rclcpp::Publisher<interfaces::msg::MotionMode>::SharedPtr pub_ack_;
  rclcpp::TimerBase::SharedPtr ack_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  rclcpp::TimerBase::SharedPtr odom_timer_;
};


using controller_interface::CallbackReturn;
using controller_interface::interface_configuration_type;
using controller_interface::InterfaceConfiguration;
using controller_interface::return_type;
using MotionMode = interfaces::msg::MotionMode;

// ---------------------------------------------------------------- 설정값 이름 등록 (controllers.yaml 에서 덮어씀)
CallbackReturn SwerveController::on_init()
{
  try {
    auto_declare<std::vector<std::string>>("module_names", {"fl", "fr", "rl", "rr"});
    auto_declare<std::vector<double>>("module_x", {0.112, 0.112, -0.112, -0.112});
    auto_declare<std::vector<double>>("module_y", {0.064, -0.064, 0.064, -0.064});
    auto_declare<std::vector<double>>("module_offset", {0.0269, -0.0269, 0.0269, -0.0269});
    auto_declare<double>("wheel_radius", 0.033);
    auto_declare<double>("steer_limit", M_PI);
    auto_declare<std::vector<double>>("steer_min", {});
    auto_declare<std::vector<double>>("steer_max", {});
    auto_declare<double>("max_linear", 0.10);
    auto_declare<double>("max_wheel_speed", 0.0);
    auto_declare<std::vector<double>>("wheel_speed_gain", {});   // 2026-10-28 바퀴별 속도 배율 [fl, fr, rl, rr], 비우면 모두 1
    auto_declare<double>("steer_branch_margin", 3.0 * M_PI / 180.0);
    auto_declare<double>("steer_end_slack", 0.0);       // [rad] 0 ~ 2°. 범위 끝을 이만큼 넘는 방향은 끝 각도로 (기본 끔)
    auto_declare<double>("steer_travel_weight", 0.0);   // 2026-10-28: 자세 고를 때 돌아야 할 각도 비용 가중치 (0 = 예전과 같음, 0~10)
    auto_declare<double>("steer_coherent_max_travel", 0.0);   // 2026-10-29 [rad] 공통 자세 이동 한도 (0 = 없음)
    auto_declare<double>("steer_end_guard", 0.0);             // 2026-10-29 [rad] 멈춘 상태에서 고를 때 범위 끝 벌점 구간 (0 = 끔)
    auto_declare<double>("keep_branch_max_wz", 0.0);          // 2026-10-29 [rad/s] 이동 중 작은 회전으로 가지 바꾸지 않기 (0 = 끔)
    auto_declare<bool>("steer_multi_turn", false);      // 2026-10-28: 여러 바퀴 조향 범위 허용 (확장 위치 모드, 지금은 모의 전용)
    auto_declare<double>("heading_hold_kp", 0.0);        // 2026-10-28 정면 유지 되먹임 (0 = 끔)
    auto_declare<double>("heading_hold_max_wz", 0.15);   // [rad/s]
    auto_declare<double>("heading_hold_deadband", 0.0087);   // [rad]
    auto_declare<double>("heading_hold_wz_per_speed", 0.0);  // [rad/s per m/s] 0 = 속도 비례 끔
    auto_declare<double>("max_angular", 0.5);
    auto_declare<double>("max_linear_accel", 0.2);
    auto_declare<double>("max_angular_accel", 0.8);
    auto_declare<double>("cmd_timeout_s", 0.3);
    auto_declare<bool>("cosine_scale", true);
    auto_declare<bool>("steer_then_drive", true);
    auto_declare<double>("steer_tolerance", 0.17);
    auto_declare<bool>("roll_while_steering", true);
    auto_declare<std::string>("steering_policy", "nearest");
    auto_declare<bool>("park_on_idle", false);
    auto_declare<double>("park_idle_s", 1.0);
    auto_declare<double>("park_still_wheel", 0.05);
    auto_declare<bool>("blend_steer_drive", false);   // 동시 조향·구동 (2026-10-26, 기본 끔)
    auto_declare<double>("blend_margin", 1.0);
    auto_declare<double>("blend_stall_s", 0.0);
    auto_declare<std::string>("steer_rate_source", "reported");
    auto_declare<std::string>("cmd_vel_topic", "/cmd_vel");
    auto_declare<std::string>("motion_mode_topic", "/control/motion_mode");
    auto_declare<bool>("require_motion_mode", true);
    auto_declare<double>("turn_timeout_s", 0.5);
    auto_declare<std::string>("motion_mode_ack_topic", "/control/motion_mode_ack");
    auto_declare<std::string>("odom_frame", "odom");
    auto_declare<std::string>("base_frame", "base_footprint");
    auto_declare<bool>("publish_tf", true);
    auto_declare<double>("odom_rate_hz", 50.0);
    auto_declare<bool>("use_imu", true);
    auto_declare<std::string>("imu_topic", "/imu_sensor_broadcaster/imu");
    auto_declare<double>("imu_bias_time_s", 2.0);
    auto_declare<double>("imu_timeout_s", 0.2);
    auto_declare<double>("imu_still_hold_s", 1.0);      // 2026-11-01 멈춘 뒤 이만큼 지나야 바이어스 따라감
    auto_declare<double>("imu_track_gate", 0.0087);     // [rad/s] 바이어스 뺀 값이 이 안일 때만 따라감 (0.5°/s)
    auto_declare<std::string>("imu_motion_bias_topic", "/imu_motion_bias");   // 2026-11-01 z = 움직일 때 더할 회전 [rad/s], 빈 문자열 = 안 씀
    auto_declare<double>("imu_motion_bias_max", 0.005);  // [rad/s] 받은 값 자르기 (0.3°/s)
    auto_declare<double>("imu_motion_bias_timeout_s", 5.0);   // 이만큼 안 오면 0 으로 (11-02 Fable 검토)
    auto_declare<double>("still_linear", 0.01);
    auto_declare<double>("still_angular", 0.05);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "설정값 등록 실패: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------- 어떤 칸을 빌릴지
InterfaceConfiguration SwerveController::command_interface_configuration() const
{
  InterfaceConfiguration c{interface_configuration_type::INDIVIDUAL, {}};
  for (const auto & j : steer_joints_) {c.names.push_back(j + "/position");}
  for (const auto & j : drive_joints_) {c.names.push_back(j + "/velocity");}
  return c;
}

InterfaceConfiguration SwerveController::state_interface_configuration() const
{
  InterfaceConfiguration c{interface_configuration_type::INDIVIDUAL, {}};
  for (const auto & j : steer_joints_) {
    c.names.push_back(j + "/position");
    c.names.push_back(j + (position_rate_ ? "/position_velocity" : "/velocity"));
  }
  for (const auto & j : drive_joints_) {c.names.push_back(j + "/velocity");}
  return c;
}

// ---------------------------------------------------------------- on_configure: 계산기 만들고 토픽 연결
CallbackReturn SwerveController::on_configure(const rclcpp_lifecycle::State &)
{
  auto node = get_node();
  const auto rate_source = node->get_parameter("steer_rate_source").as_string();
  if (rate_source != "reported" && rate_source != "position") {
    RCLCPP_ERROR(node->get_logger(), "steer_rate_source must be reported or position");
    return CallbackReturn::ERROR;
  }
  position_rate_ = rate_source == "position";
  names_ = node->get_parameter("module_names").as_string_array();
  const auto mx = node->get_parameter("module_x").as_double_array();
  const auto my = node->get_parameter("module_y").as_double_array();
  const auto mo = node->get_parameter("module_offset").as_double_array();
  if (names_.empty() || names_.size() != mx.size() || names_.size() != my.size() || names_.size() != mo.size()) {
    RCLCPP_ERROR(node->get_logger(), "module_names / module_x / module_y / module_offset 개수가 다름");
    return CallbackReturn::ERROR;
  }
  steer_joints_.clear();
  drive_joints_.clear();
  std::vector<ModuleGeometry> geo;
  for (std::size_t i = 0; i < names_.size(); ++i) {
    geo.push_back({mx[i], my[i], mo[i]});
    steer_joints_.push_back("steer_" + names_[i] + "_joint");
    drive_joints_.push_back("drive_" + names_[i] + "_joint");
  }

  const auto steering_policy = node->get_parameter("steering_policy").as_string();
  if (steering_policy != "nearest" && steering_policy != "coherent") {
    RCLCPP_ERROR(node->get_logger(), "steering_policy must be nearest or coherent");
    return CallbackReturn::ERROR;
  }
  SwerveDriveConfig cfg;
  cfg.max_linear = node->get_parameter("max_linear").as_double();
  cfg.max_wheel_speed = node->get_parameter("max_wheel_speed").as_double();
  max_wheel_speed_ = cfg.max_wheel_speed;
  wheel_gain_ = node->get_parameter("wheel_speed_gain").as_double_array();
  if (wheel_gain_.empty()) {wheel_gain_.assign(names_.size(), 1.0);}
  if (wheel_gain_.size() != names_.size()) {
    RCLCPP_ERROR(node->get_logger(), "wheel_speed_gain 은 모듈 수(%zu)와 같아야 함", names_.size());
    return controller_interface::CallbackReturn::ERROR;
  }
  for (double g : wheel_gain_) {
    if (!(g >= 0.9 && g <= 1.1)) {
      RCLCPP_ERROR(node->get_logger(), "wheel_speed_gain 은 0.9 ~ 1.1 (받은 값 %.4f)", g);
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  if (std::any_of(wheel_gain_.begin(), wheel_gain_.end(), [](double g) {return g != 1.0;})) {
    RCLCPP_INFO(node->get_logger(), "바퀴별 속도 배율 [%.4f, %.4f, %.4f, %.4f] (fl fr rl rr)",
      wheel_gain_[0], wheel_gain_[1 % wheel_gain_.size()], wheel_gain_[2 % wheel_gain_.size()], wheel_gain_[3 % wheel_gain_.size()]);
  }
  cfg.max_angular = node->get_parameter("max_angular").as_double();
  cfg.max_linear_accel = node->get_parameter("max_linear_accel").as_double();
  cfg.max_angular_accel = node->get_parameter("max_angular_accel").as_double();
  cfg.cmd_timeout_s = node->get_parameter("cmd_timeout_s").as_double();
  cfg.turn_timeout_s = node->get_parameter("turn_timeout_s").as_double();
  cfg.cosine_scale = node->get_parameter("cosine_scale").as_bool();
  cfg.steer_then_drive = node->get_parameter("steer_then_drive").as_bool();
  cfg.steer_tolerance = node->get_parameter("steer_tolerance").as_double();
  cfg.roll_while_steering = node->get_parameter("roll_while_steering").as_bool();
  // 0 이하면 명령 · 차례가 영원히 "오래됨" 이거나 조향 게이트가 영원히 닫혀 오류 없이 안 움직임 (2026-10-29 검토)
  if (!(cfg.cmd_timeout_s > 0.0 && cfg.turn_timeout_s > 0.0 && cfg.steer_tolerance > 0.0 &&
        node->get_parameter("imu_timeout_s").as_double() >= 0.0)) {
    RCLCPP_ERROR(node->get_logger(), "cmd_timeout_s · turn_timeout_s · steer_tolerance > 0, imu_timeout_s >= 0 이어야 함");
    return CallbackReturn::ERROR;
  }
  cfg.park_on_idle = node->get_parameter("park_on_idle").as_bool();
  cfg.park_idle_s = node->get_parameter("park_idle_s").as_double();
  cfg.park_still_wheel = node->get_parameter("park_still_wheel").as_double();
  cfg.heading_hold_kp = node->get_parameter("heading_hold_kp").as_double();
  cfg.heading_hold_max_wz = node->get_parameter("heading_hold_max_wz").as_double();
  cfg.heading_hold_deadband = node->get_parameter("heading_hold_deadband").as_double();
  cfg.heading_hold_wz_per_speed = node->get_parameter("heading_hold_wz_per_speed").as_double();
  if (!(cfg.heading_hold_wz_per_speed >= 0.0 && cfg.heading_hold_wz_per_speed <= 10.0)) {
    RCLCPP_ERROR(node->get_logger(), "heading_hold_wz_per_speed 는 0 ~ 10 이어야 함");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!(cfg.heading_hold_kp >= 0.0 && cfg.heading_hold_kp <= 10.0 && cfg.heading_hold_max_wz >= 0.0 &&
        cfg.heading_hold_max_wz <= 1.0 && cfg.heading_hold_deadband >= 0.0 && cfg.heading_hold_deadband <= 0.1)) {
    RCLCPP_ERROR(node->get_logger(), "heading_hold: kp 0~10 · max_wz 0~1 rad/s · deadband 0~0.1 rad 이어야 함");
    return controller_interface::CallbackReturn::ERROR;
  }
  cfg.keep_branch_max_wz = node->get_parameter("keep_branch_max_wz").as_double();
  if (!(cfg.keep_branch_max_wz >= 0.0 && cfg.keep_branch_max_wz <= 1.0)) {
    RCLCPP_ERROR(node->get_logger(), "keep_branch_max_wz 는 0 ~ 1 rad/s 이어야 함");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (cfg.keep_branch_max_wz > 0.0) {
    RCLCPP_INFO(node->get_logger(), "이동 중 가지 유지 켬: |회전| ≤ %.2f rad/s 는 바퀴가 가지를 바꾸지 않는 데까지만", cfg.keep_branch_max_wz);
  }
  if (cfg.heading_hold_kp > 0.0) {
    RCLCPP_INFO(node->get_logger(), "정면 유지 되먹임 켬: kp %.2f · 최대 %.2f rad/s (이동 속도 × %.2f 이하) · 무시 %.1f°",
      cfg.heading_hold_kp, cfg.heading_hold_max_wz, cfg.heading_hold_wz_per_speed, cfg.heading_hold_deadband * 180.0 / M_PI);
  }
  const double branch_margin = node->get_parameter("steer_branch_margin").as_double();
  for (double value : {cfg.max_linear, cfg.max_angular, cfg.max_linear_accel,
                      cfg.max_angular_accel, cfg.max_wheel_speed, branch_margin}) {
    if (!std::isfinite(value) || value < 0.0) {
      RCLCPP_ERROR(node->get_logger(), "speed/acceleration/branch margin must be finite and nonnegative");
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  cfg.blend_steer_drive = node->get_parameter("blend_steer_drive").as_bool();
  cfg.blend_margin = node->get_parameter("blend_margin").as_double();
  cfg.blend_stall_s = node->get_parameter("blend_stall_s").as_double();
  if (cfg.blend_steer_drive && !(cfg.blend_margin > 0.0 && cfg.blend_margin <= 1.0 && cfg.blend_stall_s > 0.0 && cfg.blend_stall_s <= 5.0)) {
    RCLCPP_ERROR(node->get_logger(), "blend_steer_drive: blend_margin (0,1] · blend_stall_s (0,5] 이어야 함");
    return CallbackReturn::ERROR;
  }
  if (cfg.blend_steer_drive) {
    RCLCPP_INFO(node->get_logger(), "동시 조향·구동 켬: 조향 오차 ≤ %.2f rad 안에서 새 방향으로 진행, 진행 막힘 %.1f s 넘으면 구동 0",
      cfg.steer_tolerance * cfg.blend_margin, cfg.blend_stall_s);
  }
  if (cfg.park_on_idle && !(cfg.park_idle_s >= 0.5 && cfg.park_still_wheel > 0.0)) {
    RCLCPP_ERROR(node->get_logger(), "park_idle_s >= 0.5 · park_still_wheel > 0 이어야 함");
    return CallbackReturn::ERROR;
  }
  try {
    const auto lo = node->get_parameter("steer_min").as_double_array();
    const auto hi = node->get_parameter("steer_max").as_double_array();
    std::vector<SteeringRange> ranges;
    if (!lo.empty() || !hi.empty()) {
      if (lo.size() != names_.size() || hi.size() != names_.size()) {
        throw std::invalid_argument("steer_min / steer_max는 둘 다 모듈 수와 같아야 함");
      }
      for (std::size_t i=0; i<lo.size(); ++i) {ranges.push_back({lo[i],hi[i]});}
    }
    SwerveKinematics kin(geo, node->get_parameter("wheel_radius").as_double(),
      node->get_parameter("steer_limit").as_double(), ranges, steering_policy == "coherent", branch_margin,
      node->get_parameter("steer_end_slack").as_double(),
      node->get_parameter("steer_multi_turn").as_bool() ? 6.0 * M_PI : 2.0 * M_PI);
    kin.set_travel_weight(node->get_parameter("steer_travel_weight").as_double());
    if (kin.travel_weight() > 0.0) {
      RCLCPP_INFO(node->get_logger(), "자세 고르기: 돌아야 할 각도 가중치 %.2f", kin.travel_weight());
    }
    kin.set_branch_options(node->get_parameter("steer_coherent_max_travel").as_double(),
      node->get_parameter("steer_end_guard").as_double());
    if (kin.coherent_max_travel() > 0.0 || kin.end_guard() > 0.0) {
      RCLCPP_INFO(node->get_logger(), "가지 고르기 보강: 공통 자세 이동 한도 %.0f° · 출발 때 범위 끝 벌점 구간 %.0f° (0 = 끔)",
        kin.coherent_max_travel() * 180.0 / M_PI, kin.end_guard() * 180.0 / M_PI);
    }
    drive_ = std::make_unique<SwerveDrive>(kin, cfg);
    // 실패면 타이머를 만들기 전에 끝냄 (예전엔 맨 끝에서 검사해 실패해도 ack · odom 타이머가 남았음)
    if (cfg.park_on_idle && !drive_->kinematics().park_reachable()) {
      RCLCPP_ERROR(node->get_logger(), "park_on_idle: 조향 0° 가 모듈별 범위 밖 → 복귀 기능을 쓸 수 없음");
      return CallbackReturn::ERROR;
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "스워브 설정 오류: %s", e.what());
    return CallbackReturn::ERROR;
  }

  require_motion_mode_ = node->get_parameter("require_motion_mode").as_bool();
  publish_tf_ = node->get_parameter("publish_tf").as_bool();
  use_imu_ = node->get_parameter("use_imu").as_bool();
  still_linear_ = node->get_parameter("still_linear").as_double();
  still_angular_ = node->get_parameter("still_angular").as_double();
  odom_frame_ = node->get_parameter("odom_frame").as_string();
  base_frame_ = node->get_parameter("base_frame").as_string();
  gyro_ = GyroYaw(
    node->get_parameter("imu_bias_time_s").as_double(), node->get_parameter("imu_timeout_s").as_double(), 0.002,
    node->get_parameter("imu_still_hold_s").as_double(), node->get_parameter("imu_track_gate").as_double());

  // /cmd_vel: 받기만 하고 계산은 update() 에서. NaN 이면 "이전 명령까지 취소" 표시
  sub_cmd_ = node->create_subscription<geometry_msgs::msg::TwistStamped>(
    node->get_parameter("cmd_vel_topic").as_string(), 10,
    [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr m) {
      std::lock_guard<std::mutex> lk(cmd_mtx_);
      const auto & t = m->twist;
      cmd_in_ = {t.linear.x, t.linear.y, t.angular.z};
      cmd_in_time_ = get_node()->now().seconds();   // 받은 시각 (보낸 쪽 시계가 달라도 상관없게)
      cmd_in_new_ = true;
      cmd_in_bad_ = !std::isfinite(t.linear.x) || !std::isfinite(t.linear.y) || !std::isfinite(t.angular.z);
    });

  // 차례 (D15): control_master 가 10 Hz 로 계속 보냄. 받은 시각을 같이 적어 두고, 0.5 s 넘게 안 오면 잠금
  sub_mode_ = node->create_subscription<MotionMode>(
    node->get_parameter("motion_mode_topic").as_string(), rclcpp::QoS(1).reliable().transient_local(),
    [this](MotionMode::ConstSharedPtr m) {
      const double t = get_node()->now().seconds();
      std::lock_guard<std::mutex> lk(mode_mtx_);
      mode_in_ = {static_cast<int>(m->mode), m->request_id, t};
    });
  // "반영했음" 답: 제어 루프가 실제로 쓴 차례를 control_master 에 알려 줌
  pub_ack_ = node->create_publisher<MotionMode>(node->get_parameter("motion_mode_ack_topic").as_string(), 10);
  ack_timer_ = node->create_wall_timer(std::chrono::milliseconds(50), [this]() {
      const uint64_t v = applied_.load();   // 한 번에 읽음 → 번호와 DRIVE 여부가 같은 주기의 것
      const bool drive = (v & 1u) != 0;
      MotionMode a;
      a.header.stamp = get_node()->now();
      a.mode = drive ? MotionMode::DRIVE : MotionMode::STOP;
      a.request_id = static_cast<uint32_t>(v >> 1);
      a.reason = drive ? "applied DRIVE" : "locked";
      pub_ack_->publish(a);
    });

  if (use_imu_) {
    sub_imu_ = node->create_subscription<sensor_msgs::msg::Imu>(
      node->get_parameter("imu_topic").as_string(), rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Imu::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lk(imu_mtx_);
        gyro_.add_sample(get_node()->now().seconds(), m->angular_velocity.z, base_still_);
      });
  }
  motion_bias_max_ = node->get_parameter("imu_motion_bias_max").as_double();
  // 음수면 아래 clamp 의 (하한 > 상한) 이 정의되지 않은 동작 (2026-11-02 검토). 0 = 받은 값을 늘 0 으로 자름 (= 안 씀)
  if (!std::isfinite(motion_bias_max_) || motion_bias_max_ < 0.0 || motion_bias_max_ > 0.1) {
    RCLCPP_ERROR(node->get_logger(), "imu_motion_bias_max 는 0 ~ 0.1 rad/s 이어야 함 (받은 값 %.4f)", motion_bias_max_);
    return CallbackReturn::ERROR;
  }
  motion_bias_timeout_s_ = node->get_parameter("imu_motion_bias_timeout_s").as_double();
  const auto mb_topic = node->get_parameter("imu_motion_bias_topic").as_string();
  if (use_imu_ && !mb_topic.empty()) {
    sub_motion_bias_ = node->create_subscription<geometry_msgs::msg::Vector3Stamped>(
      mb_topic, 10, [this](geometry_msgs::msg::Vector3Stamped::ConstSharedPtr m) {
        if (std::isfinite(m->vector.z)) {
          motion_bias_.store(std::clamp(m->vector.z, -motion_bias_max_, motion_bias_max_));
          motion_bias_t_.store(get_node()->now().seconds());
        }
      });
  }

  // odom 불확실성: 0 으로 두면 "완벽한 측정" 으로 읽힘. 평면 로봇이라 z · roll · pitch 는 크게(안 씀)
  //   값은 잠정 — 실측 뒤 조정. 지금 쓰는 SLAM toolbox · AMCL 은 이 값을 쓰지 않지만 robot_localization 등을 붙일 때 필요
  {
    std::lock_guard<std::mutex> lk(odom_mtx_);
    odom_msg_.pose.covariance.fill(0.0);
    odom_msg_.twist.covariance.fill(0.0);
    const double pose_var[6] = {1e-3, 1e-3, 1e6, 1e6, 1e6, 1e-2};
    const double twist_var[6] = {1e-3, 1e-3, 1e6, 1e6, 1e6, 1e-2};
    for (int k = 0; k < 6; ++k) {
      odom_msg_.pose.covariance[k * 6 + k] = pose_var[k];
      odom_msg_.twist.covariance[k * 6 + k] = twist_var[k];
    }
  }

  // odom 발행은 제어 루프 밖 타이머에서 (update 안에서 발행하면 가끔 막혀서 100 Hz 가 밀릴 수 있음)
  pub_odom_ = node->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
  pub_cmd_ = node->create_publisher<sensor_msgs::msg::JointState>("~/commands", 10);
  {
    std::lock_guard<std::mutex> lk(odom_mtx_);
    cmd_msg_.name.clear();
    for (const auto & j : steer_joints_) {cmd_msg_.name.push_back(j);}
    for (const auto & j : drive_joints_) {cmd_msg_.name.push_back(j);}
    cmd_msg_.position.assign(cmd_msg_.name.size(), NAN);
    cmd_msg_.velocity.assign(cmd_msg_.name.size(), NAN);
  }
  tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(node);
  const double hz = std::max(1.0, node->get_parameter("odom_rate_hz").as_double());
  odom_timer_ = node->create_wall_timer(
    std::chrono::duration<double>(1.0 / hz), [this]() {publish_odom();});

  RCLCPP_INFO(node->get_logger(), "스워브 컨트롤러: 모듈 %zu개, 최고 %.2f m/s, cmd_vel 끊김 %.2f s → 정지, 차례 %s",
    names_.size(), cfg.max_linear, cfg.cmd_timeout_s, require_motion_mode_ ? "따름" : "무시(시험용)");
  if (cfg.park_on_idle) {
    RCLCPP_INFO(node->get_logger(), "정지 후 바깥쪽 자세 복귀 켬: 신선한 0 명령 + 바퀴 정지 %.1f s 뒤 조향 0°",
      cfg.park_idle_s);
  }
  return CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------- 켜기 / 끄기
CallbackReturn SwerveController::on_activate(const rclcpp_lifecycle::State &)
{
  last_lin_speed_ = 0.0;
  // 빌린 칸 순서 확인 (configuration 에 적은 순서와 같아야 read/write 가 맞음)
  const auto cmd_cfg = command_interface_configuration();
  const auto st_cfg = state_interface_configuration();
  if (command_interfaces_.size() != cmd_cfg.names.size() || state_interfaces_.size() != st_cfg.names.size()) {
    RCLCPP_ERROR(get_node()->get_logger(), "모터 칸 개수가 다름 (URDF 의 ros2_control 블록 확인)");
    return CallbackReturn::ERROR;
  }
  for (std::size_t k = 0; k < cmd_cfg.names.size(); ++k) {
    if (command_interfaces_[k].get_name() != cmd_cfg.names[k]) {
      RCLCPP_ERROR(get_node()->get_logger(), "명령 칸 순서가 다름: %s", command_interfaces_[k].get_name().c_str());
      return CallbackReturn::ERROR;
    }
  }
  for (std::size_t k = 0; k < st_cfg.names.size(); ++k) {
    if (state_interfaces_[k].get_name() != st_cfg.names[k]) {
      RCLCPP_ERROR(get_node()->get_logger(), "상태 칸 순서가 다름: %s", state_interfaces_[k].get_name().c_str());
      return CallbackReturn::ERROR;
    }
  }
  drive_->reset();
  was_parking_ = false;
  {
    std::lock_guard<std::mutex> lk(cmd_mtx_);
    cmd_in_new_ = false;   // 켜기 전에 온 명령은 버림
  }
  // 조향은 지금 자리, 바퀴는 0 으로 시작
  std::vector<ModuleState> now;
  last_steer_.assign(names_.size(), 0.0);
  // 지금 조향 자세를 모르면 켜지 않음 — 모르는 자세 대신 0° 를 "유지" 명령으로 보내면 조향이 0° 로 돎
  //   (예전엔 position 모드만 거부, reported 모드는 0° 를 썼음 — 2026-10-29 검토). 켜진 뒤 못 읽으면 update() 가 마지막 실측을 유지
  if (!read_modules(now)) {
    RCLCPP_ERROR(get_node()->get_logger(), "모터 상태를 못 읽음 (position_velocity 준비 전 포함) → 켜기 거부");
    return CallbackReturn::ERROR;
  }
  for (std::size_t i = 0; i < now.size(); ++i) {last_steer_[i] = now[i].steer_angle;}
  write_modules(drive_->stop_hold(last_steer_));
  last_time_s_ = -1.0;
  return CallbackReturn::SUCCESS;
}

CallbackReturn SwerveController::on_deactivate(const rclcpp_lifecycle::State &)
{
  std::vector<ModuleState> measured;
  if (read_modules(measured)) {
    for (std::size_t i = 0; i < measured.size(); ++i) {last_steer_[i] = measured[i].steer_angle;}
  }
  applied_.store(static_cast<uint64_t>(drive_->applied_turn_id()) << 1);   // 꺼짐 = DRIVE 아님 (번호는 그대로)
  drive_->reset();
  write_modules(drive_->stop_hold(last_steer_));   // 마지막 실측 자세 (덜 끝난 목표가 아님)
  std::lock_guard<std::mutex> lk(odom_mtx_);
  odom_ready_ = false;   // 꺼진 동안 옛날 odom 을 계속 내보내지 않게
  return CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------- 100 Hz (controller_manager 가 부름)
return_type SwerveController::update(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  const double now_s = time.seconds();
  const double dt = period.seconds();

  // ① 토픽에서 온 명령 꺼내기 (못 잠그면 이번 주기는 지난 값으로 — 기다리지 않음)
  {
    std::unique_lock<std::mutex> lk(cmd_mtx_, std::try_to_lock);
    if (lk.owns_lock() && cmd_in_new_) {
      cmd_in_new_ = false;
      if (cmd_in_bad_) {
        drive_->set_command({NAN, NAN, NAN}, now_s);   // 이전 명령까지 취소
        RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000, "cmd_vel 에 NaN/무한대 → 정지");
      } else {
        // 받은 지 오래된 명령(루프가 밀린 경우)도 받은 시각 그대로 넣어서 신선도 판단이 맞게
        drive_->set_command(cmd_in_, std::min(cmd_in_time_, now_s));
      }
    }
  }
  // ② 차례: 받은 시각까지 넘겨서 "0.5 s 넘게 안 옴 = 잠금" 을 계산기(SwerveDrive)가 판단
  if (require_motion_mode_) {
    {
      std::unique_lock<std::mutex> lk(mode_mtx_, std::try_to_lock);
      if (lk.owns_lock()) {mode_snap_ = mode_in_;}   // 한 묶음째로 복사
    }
    if (mode_snap_.mode >= 0) {
      drive_->set_turn(mode_snap_.mode == MotionMode::DRIVE, mode_snap_.id, mode_snap_.time);
    }
  } else {
    drive_->set_turn(true, 0, now_s);   // 시험용: 차례 무시
  }

  // ③ 모터 상태 읽기 → 못 읽으면 바퀴 0 · 조향 그대로
  std::vector<ModuleState> & meas = meas_buf_;   // 멤버 버퍼 재사용 (주기마다 새로 할당하지 않게 — 2026-10-16 검토)
  if (!read_modules(meas)) {
    drive_->reset();  // Any unreadable state cancels the previous command, including park.
    write_modules(drive_->stop_hold(last_steer_));
    applied_.store(static_cast<uint64_t>(drive_->applied_turn_id()) << 1);   // 번호 그대로, DRIVE 아님
    {
      // 상태를 모르는 동안 옛날 odom 을 "지금 멈춰 있음" 처럼 계속 내보내지 않게
      std::unique_lock<std::mutex> lk(odom_mtx_, std::try_to_lock);
      if (lk.owns_lock()) {odom_ready_ = false;}
    }
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000, "모터 상태를 못 읽음 → 바퀴 정지");
    return return_type::OK;
  }

  // ④ 계산 → 쓰기 → 반영한 차례 기록 (ack 타이머가 보냄)
  const auto cmds = drive_->update(now_s, dt, meas);
  if (!drive_->command_feasible()) {   // 모듈별 범위 안에서 만들 수 없는 방향 → 바퀴 0 · 조향 유지 (자르지 않음)
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "조향 범위 안에서 만들 수 없는 명령 → 정지 (cmd_vel 거부)");
  }
  if (drive_->parking() != was_parking_) {
    was_parking_ = drive_->parking();
    RCLCPP_INFO(get_node()->get_logger(), was_parking_ ? "정지 확인 → 바깥쪽 자세(조향 0°)로 복귀" : "복귀 해제 (새 명령 · 명령 끊김 · 상태 못 읽음)");
  }
  write_modules(cmds);
  applied_.store((static_cast<uint64_t>(drive_->applied_turn_id()) << 1) | (drive_->applied_drive() ? 1u : 0u));
  for (std::size_t i = 0; i < meas.size(); ++i) {last_steer_[i] = meas[i].steer_angle;}

  // ⑤ odom (자이로는 잠글 수 있을 때만 읽음)
  bool gyro_ok = false;
  double gz = 0.0;
  if (use_imu_) {
    std::unique_lock<std::mutex> lk(imu_mtx_, std::try_to_lock);
    if (lk.owns_lock()) {
      gyro_ok = gyro_.ready(now_s);
      gz = gyro_.rate();
    }
    if (last_lin_speed_ > 0.02 && now_s - motion_bias_t_.load() <= motion_bias_timeout_s_) {
      gz += motion_bias_.load();   // 바퀴로 움직이는 중일 때만 (가만히 · 제자리 회전은 정확함), 최근에 받은 값만
    }
  }
  const BodyTwist v = drive_->integrate_odom(meas, last_time_s_ < 0.0 ? 0.0 : dt, gyro_ok, gz);
  last_lin_speed_ = std::hypot(v.vx, v.vy);
  last_time_s_ = now_s;
  const bool out_zero = drive_->output().vx == 0.0 && drive_->output().vy == 0.0 && drive_->output().wz == 0.0;
  base_still_ = out_zero && std::hypot(v.vx, v.vy) < still_linear_ && std::fabs(v.wz) < still_angular_;

  {
    std::unique_lock<std::mutex> lk(odom_mtx_, std::try_to_lock);
    if (lk.owns_lock()) {
      const auto & p = drive_->pose();
      odom_msg_.header.stamp = time;
      odom_msg_.header.frame_id = odom_frame_;
      odom_msg_.child_frame_id = base_frame_;
      odom_msg_.pose.pose.position.x = p.x;
      odom_msg_.pose.pose.position.y = p.y;
      odom_msg_.pose.pose.orientation.z = std::sin(p.yaw / 2.0);
      odom_msg_.pose.pose.orientation.w = std::cos(p.yaw / 2.0);
      odom_msg_.twist.twist.linear.x = v.vx;
      odom_msg_.twist.twist.linear.y = v.vy;
      odom_msg_.twist.twist.angular.z = v.wz;
      odom_ready_ = true;
    }
  }
  return return_type::OK;
}

// ---------------------------------------------------------------- 도우미
bool SwerveController::read_modules(std::vector<ModuleState> & out) const
{
  const std::size_t n = names_.size();
  out.assign(n, ModuleState{0.0, 0.0});
  if (state_interfaces_.size() != 3 * n) {return false;}
  for (std::size_t i = 0; i < n; ++i) {
    const auto pos = state_interfaces_[2 * i].get_optional<double>();
    const auto rate = state_interfaces_[2 * i + 1].get_optional<double>();
    const auto wheel = state_interfaces_[2 * n + i].get_optional<double>();
    if (!pos || !rate || !wheel || !std::isfinite(*pos) || !std::isfinite(*wheel)) {return false;}
    if (position_rate_ && !std::isfinite(*rate)) {return false;}
    out[i].steer_angle = *pos;
    out[i].steer_rate = std::isfinite(*rate) ? *rate : 0.0;
    out[i].wheel_speed = *wheel / wheel_gain_[i];   // 배율로 나눔 → 계산 · odom 은 "보정된 땅 위 속도" 기준
  }
  return true;
}

void SwerveController::write_modules(const std::vector<ModuleState> & m)
{
  const std::size_t n = names_.size();
  if (command_interfaces_.size() != 2 * n || m.size() != n) {return;}
  for (std::size_t i = 0; i < n; ++i) {
    // set_value 는 다른 쪽이 잠깐 잡고 있으면 실패할 수 있음 → 다음 주기에 다시 씀 (10 ms 뒤)
    (void)command_interfaces_[i].set_value(m[i].steer_angle);
    // 배율을 곱한 뒤 모터 속도 한계 안으로 (다이나믹셀은 한계를 넘는 Goal Velocity 를 무시함)
    double w = m[i].wheel_speed * wheel_gain_[i];
    if (max_wheel_speed_ > 0.0) {w = std::clamp(w, -max_wheel_speed_, max_wheel_speed_);}
    (void)command_interfaces_[n + i].set_value(w);
  }
  std::unique_lock<std::mutex> lk(odom_mtx_, std::try_to_lock);   // 기록용 사본 (못 잠그면 이번 주기는 건너뜀)
  if (lk.owns_lock() && cmd_msg_.position.size() == 2 * n) {
    cmd_msg_.header.stamp = get_node()->now();
    for (std::size_t i = 0; i < n; ++i) {   // 실제로 쓴 값 (배율 · 모터 한계 뒤, 11-02 GPT 검토)
      double w = m[i].wheel_speed * wheel_gain_[i];
      if (max_wheel_speed_ > 0.0) {w = std::clamp(w, -max_wheel_speed_, max_wheel_speed_);}
      cmd_msg_.position[i] = m[i].steer_angle;
      cmd_msg_.velocity[n + i] = w;
    }
    cmd_msg_new_ = true;
  }
}

void SwerveController::publish_odom()
{
  nav_msgs::msg::Odometry msg;
  sensor_msgs::msg::JointState cmd;
  bool have_cmd = false;
  bool have_odom = false;
  {
    std::lock_guard<std::mutex> lk(odom_mtx_);
    if (cmd_msg_new_) {cmd = cmd_msg_; cmd_msg_new_ = false; have_cmd = true;}
    if (odom_ready_) {msg = odom_msg_; have_odom = true;}
  }
  if (have_cmd) {pub_cmd_->publish(cmd);}
  if (!have_odom) {return;}
  pub_odom_->publish(msg);
  if (publish_tf_) {
    geometry_msgs::msg::TransformStamped tf;
    tf.header = msg.header;
    tf.child_frame_id = base_frame_;
    tf.transform.translation.x = msg.pose.pose.position.x;
    tf.transform.translation.y = msg.pose.pose.position.y;
    tf.transform.rotation = msg.pose.pose.orientation;
    tf_->sendTransform(tf);
  }
}

}  // namespace swerve_controller

PLUGINLIB_EXPORT_CLASS(swerve_controller::SwerveController, controller_interface::ControllerInterface)
