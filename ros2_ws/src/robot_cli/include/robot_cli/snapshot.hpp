// snapshot.hpp — 화면 한 장에 쓸 로봇 상태 (RobotLink 가 채우고 status_view 가 글로 바꿈, ROS 타입 없음)
//   *_t = 받은 시각 (steady s), -1 = 한 번도 못 받음. now 와 빼서 나이 [s]
#pragma once

#include <cmath>
#include <cstdint>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace robot_cli
{

struct GoalView
{
  std::string id;          // UUID 앞 8 자리
  int8_t status{0};        // action_msgs/GoalStatus
};

// 팔 작업 하나의 시간 (누가 보냈든 — status · feedback 토픽으로 봄)
struct TaskTiming
{
  std::string id;
  std::string what;        // 이 CLI 가 보낸 것이면 "STORE S5" 처럼, 아니면 ""
  double t0{-1}, t1{-1};   // 처음 본 시각 (받음 · 진행 중) · 끝난 시각
  int8_t status{0};
  std::vector<std::pair<std::string, double>> phases;   // phase 이름 · 시작 시각
};

// 검출 하나 (interfaces/msg/Detection)
struct DetView
{
  std::string kind, label, shape, id;
  double conf{0};
  double x{NAN}, y{NAN}, z{NAN};             // 카메라 광학 프레임 [m], 모르면 NaN
  double qx{0}, qy{0}, qz{0}, qw{0};         // 모르면 0,0,0,0
  bool yaw_known{false};
  double bbox[4]{0, 0, 0, 0};
};

// 적재 칸 하나 (interfaces/msg/CargoSlot)
struct SlotView
{
  std::string slot, color, puck_id, shape;
  bool occupied{false}, verified{false};
};

// 보드 배터리 (OpenCR 하드웨어 로그 /rosout 에서 — 전압 토픽이 없음)
struct BatteryView
{
  double mean{0}, min{0};   // [V]
  double t{-1};
};

struct DiagView
{
  std::string name, message;
  int level{0};
};

struct Snapshot
{
  double now{0.0};
  // 팔 관절 — /joint_states (엔코더) · /joint_states_corrected (TF 용, joint2~4 처짐 보정)
  //   *_t = joint1~5 중 가장 늦게 받은 관절의 시각 (바퀴만 든 메시지로 팔 값이 새것처럼 보이지 않게), 하나라도 없으면 -1
  std::map<std::string, double> joints, corrected;
  double joints_t{-1}, corrected_t{-1};
  std::map<std::string, double> current;                       // effort 칸 = 모터 전류 [A] (OpenCR, mock 은 0)
  std::map<std::string, std::pair<double, double>> limits;     // /robot_description (URDF) 관절 한계 [rad]
  // 차례 — /control/motion_mode · _ack
  uint8_t mode{0}, ack_mode{0};
  uint32_t request_id{0}, ack_id{0};
  std::string reason;
  bool ready{false};
  double mode_t{-1}, ack_t{-1};
  // 팔 작업 /arm_task (누가 보냈든 — _action/status · feedback)
  std::vector<GoalView> arm_goals;
  std::string arm_phase;
  double arm_phase_t{-1};
  std::string my_arm;      // 이 CLI 가 보낸 마지막 팔 작업과 결과 (사람 글)
  std::string my_arm_id;   // 그 goal id (앞 8 자리, 받았을 때)
  std::vector<TaskTiming> arm_hist;   // 최근 팔 작업 (오래된 것 먼저, 8 개까지)
  // 주행 navigate_to_pose
  std::vector<GoalView> nav_goals;
  double dist_left{0}, eta_s{0}, nav_time_s{0};
  int recoveries{0};
  double nav_fb_t{-1};
  std::string my_nav;
  // 위치
  double ox{0}, oy{0}, oyaw{0}, ovx{0}, ovy{0}, owz{0};
  double odom_t{-1};
  double clock_off{0};     // 이 PC 시계 − /odom stamp [s]. swerve_controller 는 0.3 s 넘게 다른 /cmd_vel 을 버림
  bool have_map_pose{false};
  double mx{0}, my{0}, myaw{0};
  double map_age{0};       // TF map → base_footprint 변환 시각의 나이 [s] (로봇 시계 기준, 끊기면 커짐)
  // 위치 추정
  std::string loc_mode;    // "SLAM mapping" · "SLAM localization" · "AMCL" · "" (없음)
  int health_level{-1};
  std::string health_msg;
  double health_t{-1};
  double amcl_std_xy{0}, amcl_std_yaw{0};
  double ax{0}, ay{0}, ayaw{0};   // /amcl_pose 위치 (map)
  double amcl_t{-1};
  // Nav2 lifecycle_manager is_active: -1 = 서비스 없음 · 0 = 아님 · 1 = active
  int nav_active{-1}, loc_active{-1};
  // 인지
  uint8_t light{0};
  bool light_visible{false};
  double light_t{-1};
  int wrist_n{0};
  std::string wrist_summary;
  double wrist_t{-1};
  std::string wrist_frame;
  double wrist_lag{0};      // 받은 시각 − 촬영 stamp [s] (이 PC 시계 기준, 시계 차이 포함)
  double wrist_stamp{0};    // 마지막 손목 검출 촬영 시각 (header.stamp) [s]
  std::vector<DetView> wrist_dets;
  int wide_n{0};
  double wide_t{-1};
  std::vector<DetView> wide_dets;
  // 적재함 칸 태그 (/rack_tags/detections, manipulation rack_tag_node, base_link): 이번 장 + 칸마다 마지막으로 본 것
  int rack_n{0};
  double rack_t{-1};
  std::map<std::string, std::pair<DetView, double>> rack_seen;   // 키 "c03" (칸 태그) · "p03" (칸 안 퍽) → (검출, 받은 시각)
  // rack_seen 에서 칸 태그 ('c') · 칸 안 퍽 ('p') 를 본 칸 수 (같은 칸의 태그 · 퍽은 따로 셈)
  std::size_t rack_count(char kind) const
  {
    std::size_t n = 0;
    for (const auto & kv : rack_seen) {n += !kv.first.empty() && kv.first[0] == kind ? 1 : 0;}
    return n;
  }
  std::string task_text;
  double text_t{-1};
  // 미션
  std::string m_state, m_phase, m_detail, m_step;
  int pucks_done{0}, pucks_total{0};
  bool m_ready{false}, m_fault{false};
  double mission_t{-1};
  bool hold{false};
  double hold_t{-1};
  std::string cargo;
  double cargo_t{-1};
  std::vector<SlotView> cargo_slots;
  SlotView cargo_gripper;
  // 진단 — controller_manager 목록 · /diagnostics · /localization_health
  std::vector<std::pair<std::string, std::string>> controllers, hardware;
  double ctrl_t{-1}, hw_t{-1};
  std::vector<DiagView> diags;
  double diag_t{-1};
  double scan_t{-1};
  double scan_front_t{-1}, scan_rear_t{-1};   // /lidar_front/scan · /lidar_rear/scan
  double imu_t{-1};                           // /imu_sensor_broadcaster/imu (OpenCR 일 때만)
  std::map<std::string, BatteryView> battery; // 보드 이름 → 전압
  std::set<std::string> nodes;    // "/control_master" 같은 전체 이름
  // 이 CLI
  bool estop{false};
  bool driving{false}, teleop{false};
  double dvx{0}, dvy{0}, dwz{0};
};

}  // namespace robot_cli
