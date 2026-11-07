// control_master.cpp — 제어 마스터: 팔 동작 + "한 번에 한 무리" (차례)
// =====================================================================================
// 모터와 직접 말하지 않는다 (그건 ros2_control 이 함). 바퀴 계산도 하지 않는다 (swerve_controller 가 함).
//   왜 바퀴를 여기서 안 하나: 이 노드가 바퀴 명령을 토픽으로 내보내는 구조면, 노드가 멈췄을 때 받는 쪽이
//   마지막 속도를 붙잡고 계속 돌 수 있음. 바퀴는 ros2_control 의 100 Hz 루프 안에서 스스로 끊김을 봐야 안전 (D8)
//
//   mission_manager ─/arm_task────▶ ┌─────────────────────┐ ──arm_controller/follow_joint_trajectory──▶ ros2_control
//                     (ArmTask 액션) │   control_master     │ ──gripper_controller/follow_joint_trajectory▶
//   swerve_controller ──/odom─────▶ │  ① 팔 동작 순서      │
//                                   │  ② 차례 정하기       │ ──/control/motion_mode (10 Hz)──▶ swerve_controller · planning
//   swerve_controller ─/control/motion_mode_ack (잠갔음)─▶  └─────────────────────┘
//
// 차례 (D15, arm.cpp 의 MotionArbiter) — "잠금 → 확인 → 팔"
//   ① 켤 때는 STOP (팔 자세를 모름 → 바퀴 잠금). planning 의 첫 HOME 이 끝나야 DRIVE
//   ② 차례를 **10 Hz 로 계속** 방송 (요청 번호 포함). swerve_controller 는 0.5 s 안 오면 스스로 잠금
//      → 이 노드가 죽거나 신호가 끊겨도 바퀴가 멋대로 돌지 않음
//   ③ 팔 작업이 오면 STOPPING(새 번호) → swerve_controller 가 "이 번호로 잠갔음" 답(/control/motion_mode_ack)
//      → 그 답을 받은 뒤 /odom(측정 시각이 새로운 것)이 settle_s 동안 멈춤이면 ARM
//      예전 번호의 답 · 멈춰 있기만 한 것으로는 팔을 시작하지 않음 (멈춰 있어도 잠기지 않았으면 언제든 움직일 수 있으니까)
//   ④ 팔 작업이 끝까지 성공하고 접기(home)로 끝났을 때만 DRIVE. 실패 · 취소 · 시간 초과 → 언제나 STOP 유지
//      왜: "이번 작업에서 팔을 아직 안 움직였다" 는 "팔이 접혀 있다" 는 뜻이 아님 (켤 때 · 재시작 직후엔 팔 자세를 모름)
//   ⑤ (2026-10-16 R1) "접기로 끝났다" 는 JTC 의 성공 답이 아니라 **/joint_states 실측**으로 확인
//      마지막 home 동작 뒤 VERIFY: 측정 시각이 새로운 관절값 5개가 home ± fold_tolerance 안에 fold_hold_s 동안 → DRIVE
//      verify_timeout_s 안에 안 되면 실패 → STOP. 측정이 없거나 NaN(하드웨어가 못 읽은 모터) · 오래됨도 실패
//      (Codex 재현: 어깨를 고정해도 JTC 목표 오차 검사가 꺼져 있어서 HOME 성공 → DRIVE 였음.
//       JTC 쪽에도 goal 허용 오차를 켰지만(controllers.yaml), 바퀴를 푸는 판단은 이 노드가 실측으로 한 번 더)
//   ⑥ (2026-10-16 재검증 R6 · 보드 2장) 하드웨어 이상이면 HOME 확인 무효 — arm.cpp HardwareWatch
//      ① 보드 상태 직접: controller_manager 의 list_hardware_components 를 hw_component_hz 로 물어 하나라도 active 가 아니면
//         (주행 보드만 죽고 팔은 멀쩡해도 잡힘). 답이 hw_component_timeout_s 넘게 없어도 이상
//      ② 관절 측정: hw_watch_joints(기본 14 개 전부)가 hw_state_timeout_s 넘게 끊기거나 NaN · 빠짐이 hw_bad_hold_s 넘게
//      → 즉시 STOP, 진행 중 팔 작업은 실패(답을 기다리지 않음), 새 팔 작업은 거절. 복구는 사람이 하드웨어를 다시 설정한 뒤
//        HOME + 접힘 실측이 다시 성공해야 DRIVE. 하드웨어가 돌아왔다고 바로 DRIVE 로 가는 길은 없음
//      옛 작업의 늦은 성공 답: 작업이 이미 실패로 끝났고(세대 번호가 바뀜) DRIVE 는 FINISH 의 drive_ok 로만 → 잠금이 풀리지 않음
//   팔 작업 진행(보내기 · 시간 제한 · 취소 · 끝내기) 판단은 arm.cpp 의 ArmRunner (순수 C++, test_arm.cpp) — 이 파일은 ROS 입출력만
//
// 팔 안전
//   - 팔 · 그리퍼 재생기(JTC)가 아직 안 떴으면 ArmTask 를 거절 → planning 이 잠시 뒤 다시 보냄 (켜는 순서에 기대지 않으려고)
//   - 받아 주기 시간 제한(accept_timeout_s) 과 끝나기 시간 제한(move_time_s + motion_margin_s) 을 따로
//   - 취소: 진행 중 궤적에 취소를 보내고 **끝났다는 답(또는 cancel_timeout_s)** 을 기다린 뒤 "취소됨"
//     (취소 요청 ≠ 실제로 멈춤 이라서)
//   - 늦게 받아들여진 궤적(이미 포기한 것)은 그 궤적만 골라 취소 (안 그러면 팔이 혼자 움직임)
//
// 적재함이 움직였을 때 (칸 바닥 AprilTag, manipulation rack_tag_node)
//   - LOOK slot "rack" (빈 손 스캔) 동안 /rack_tags/detections 의 멈춘 장 칸 태그를 모아 칸마다 중앙값
//   - ~/save_rack_reference (std_srvs/Trigger): 적재함이 제자리일 때 (넣기 · 빼기가 되는 때) 마지막 스캔을 기준으로 파일에 저장
//   - 그 뒤 스캔마다 기준과 비교해 적재함 전체 움직임 (dx · dy · 회전) 을 맞춤 → STORE · RETRIEVE 경로의 칸 근처만 그 칸 만큼 옮김
//     (cell_shift: shift_cell_paths.py 와 같은 규칙). 같은 스캔 자세 · 같은 카메라끼리 빼서 카메라 치우침이 빠짐
//     rack_shift_min_m 보다 작으면 그대로, rack_shift_max_m 넘으면 거절 (적재함을 제자리로 · 다시 스캔)
//
// 누운 네모 퍽 세우기 (stand_lying, 경로 표 config/standup_paths.txt — Isaac 강화학습 정책을 퍽 자리마다 돌려 뽑은 관절 경로)
//   - 누운 퍽 PICK 때 켜져 있으면: 가장 가까운 표 경로를 실제 퍽으로 옮겨 (표 칸 → 실제 퍽 차이만큼 수평 이동 · 퍽 둘레 회전, 쥐기 포함)
//     위 → 쥐기 → 세우기 경로 (여는 점에서 그리퍼 엶) → 손 5 cm 위로 →
//     시뮬에서 퍽이 선 자리 (같이 옮김) 의 선 퍽을 수직 손으로 다시 쥠 → 박스 위 기준점 (선 퍽을 쥔 채, 바로 STORE 가능)
//   - 표 자리와 차이가 standup_max_gap_mm · deg 넘으면 거절 (표 바깥 자리)
//
// 쥔 퍽 손 안 위치 (손목 카메라, manipulation rack_tag_node /held_puck/tag)
//   - PICK 이 끝나 박스 위 기준점에 있을 때 손목 카메라 가까이 보이는 퍽 태그 (손목 좌표) 를 모아 중앙값
//   - ~/save_held_reference (std_srvs/Trigger): STORE 가 잘 들어간 PICK 의 값을 기준으로 저장 (~/.ros/held_reference.txt)
//   - STORE 때 기준과 차이를 로그로 남기고, held_correct 가 true 면 그 차이를 놓는 자세의 손 방향으로 돌려 칸 근처 경로를 옮김

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "controller_manager_msgs/srv/list_hardware_components.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "geometry_msgs/msg/quaternion_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "interfaces/msg/detection_array.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

#include "manipulation/arm.hpp"
#include "manipulation/arm_kinematics.hpp"
#include "manipulation/grasp_select.hpp"
#include "manipulation/arm_model_params.hpp"
#include "manipulation/arm_path.hpp"
#include "manipulation/cell_shift.hpp"
#include "manipulation/rack_tags.hpp"
#include "manipulation/standup_table.hpp"
#include "manipulation/park_stand.hpp"
#include "interfaces/action/arm_task.hpp"
#include "interfaces/msg/motion_mode.hpp"

namespace manipulation
{

using ArmTask = interfaces::action::ArmTask;
using ArmGoalHandle = rclcpp_action::ServerGoalHandle<ArmTask>;
using FJT = control_msgs::action::FollowJointTrajectory;
using FJTClient = rclcpp_action::Client<FJT>;
using ListHw = controller_manager_msgs::srv::ListHardwareComponents;

class ControlMaster : public rclcpp::Node
{
public:
  ControlMaster()
  : Node("control_master"), arbiter_(0.5)
  {
    // ================= 설정값 (config/control.yaml) =================
    const double rate_hz = declare_parameter<double>("rate_hz", 50.0);
    // ---- 팔 ----
    arm_joints_ = declare_parameter<std::vector<std::string>>(
      "arm_joints", {"joint1", "joint2", "joint3", "joint4", "joint5"});
    gripper_joint_ = declare_parameter<std::string>("gripper_joint", "gripper_left_joint");
    move_time_s_ = declare_parameter<double>("move_time_s", 4.0);
    accept_timeout_s_ = declare_parameter<double>("accept_timeout_s", 2.0);
    motion_margin_s_ = declare_parameter<double>("motion_margin_s", 3.0);
    gripper_open_ = declare_parameter<double>("gripper_open", 0.019);
    gripper_close_ = declare_parameter<double>("gripper_close", -0.005);
    grip_.open = gripper_open_;
    grip_.close = gripper_close_;
    grip_.release = declare_parameter<double>("gripper_release", 0.0093);
    // 받침에 넣을 때 손가락을 살짝 풀어 퍽이 돌며 들어가게 (11-07 사용자: 박스 세우기 때처럼 돌 수 있는 폭으로). 퍽에 닿은 자리 약 0.0046
    grip_.loose = declare_parameter<double>("park_loose_grip_m", 0.0060);
    gripper_time_s_ = declare_parameter<double>("gripper_time_s", 1.2);
    // ---- 실물 경로 (config/arm_paths.yaml) ----
    timing_.speed_dps = declare_parameter<double>("path_speed_dps", 40.0);
    timing_.stand_speed_dps = declare_parameter<double>("path_stand_speed_dps", 20.0);
    timing_.accel_dps2 = declare_parameter<double>("path_accel_dps2", 150.0);
    timing_.dt_s = declare_parameter<double>("path_dt_s", 0.05);
    // 그리퍼를 여닫기 전 팔이 멈춰 따라오게 기다림 [s]. 11-07 실물: 그리퍼 동작 때 2 번 관절이 아직 2.3° 남아 (손끝 약 1 cm) 놓는 자리가 반반
    timing_.grip_settle_s = declare_parameter<double>("grip_settle_s", 0.0);
    if (!(timing_.speed_dps > 0.0) || !(timing_.stand_speed_dps > 0.0) || !(timing_.accel_dps2 > 0.0) ||
      !(timing_.dt_s >= 0.005 && timing_.dt_s <= 0.5))
    {
      // 0 · 음수 · NaN 이면 시간표가 수십만 점 · 나누기 0 → 실물 기본값으로 (설정 실수가 팔 동작으로 이어지지 않게)
      RCLCPP_ERROR(get_logger(), "path_speed · stand · accel · dt 설정이 잘못됨 (%.3g, %.3g, %.3g, %.3g) → 기본값 40 · 20 · 150 · 0.05",
        timing_.speed_dps, timing_.stand_speed_dps, timing_.accel_dps2, timing_.dt_s);
      timing_ = PathTiming{};
    }
    path_start_tol_ = declare_parameter<double>("path_start_tolerance", 0.15);   // ≥ arm_controller 목표 허용(0.12): 앞 작업 성공 자세에서 다음 작업이 거절되지 않게
    lazy_fold_ = declare_parameter<bool>("lazy_fold", false);   // 나중에 접기 (아래 '작업 → 동작 목록')
    // P0 에서 끝난 뒤 이만큼 다음 팔 작업이 없으면 스스로 접음 (11-03 실물: P0 에서 약 15 s 버티면 팔 모터 과열 Hardware Error)
    auto_fold_s_ = declare_parameter<double>("lazy_fold_idle_s", 5.0);
    load_paths();
    {
      const auto fold = paths_.find("fold");
      const auto unfold = paths_.find("unfold");
      // P0 가 fold · unfold 에서 같고, 받침과 출발 허용 × 2 넘게 달라야 P0 에서 잇기 (lazy_fold · LOOK 뒤 PICK)
      bool same_p0 = fold != paths_.end() && unfold != paths_.end() && !fold->second.nodes.empty() && !unfold->second.nodes.empty() &&
        fold->second.nodes.front().q.size() == unfold->second.nodes.back().q.size();
      for (std::size_t j = 0; same_p0 && j < fold->second.nodes.front().q.size(); ++j) {
        same_p0 = std::fabs(fold->second.nodes.front().q[j] - unfold->second.nodes.back().q[j]) < 1e-9;
      }
      p0_join_ok_ = same_p0 && p0_distinct(fold->second, unfold->second, 2.0 * path_start_tol_);
    }
    if (lazy_fold_ && !p0_join_ok_) {
      RCLCPP_ERROR(get_logger(), "lazy_fold: fold · unfold 의 P0 가 다르거나 P0 와 받침이 출발 허용 × 2 안이라 구분 못 함 → 끔");
      lazy_fold_ = false;
    }
    // ---- 집기 · 놓기 (실물 집기 계산기와 같은 기구학, config/arm_kinematics.yaml) ----
    staging_ = declare_parameter<std::vector<double>>("staging_pose", std::vector<double>{});
    // 쥐기 −40 mm = 11-01 실물 넣기 시험 때 쥔 깊이. 칸 경로(arm_paths)가 이 깊이로 쥔 퍽 기준 — 얕게 쥐면 쥔 퍽이 그만큼 낮게 지나가
    //   옆 칸 퍽을 긁음 (−30 이면 10 mm 낮음: 11-03 물리 시뮬 넣기 41/56 → −40 에서 53/56, 실제 채우는 순서 24/24)
    pick_heights_ = declare_parameter<std::vector<double>>("pick_heights_mm", {40.0, 10.0, -40.0});
    place_heights_ = declare_parameter<std::vector<double>>("place_heights_mm", {40.0, 10.0, -23.0});   // 마지막 = 쥔 깊이 모를 때만
    place_clear_mm_ = declare_parameter<double>("place_bottom_clear_mm", 2.0);      // 놓을 때 퍽 바닥 ↔ 놓을 면
    retrieve_depth_mm_ = declare_parameter<double>("retrieve_grip_depth_mm", 25.0); // 칸에서 꺼낸 퍽을 쥔 깊이 (11-01 실물 빼기 때 쓴 깊이, unload_cell.py)
    tilted_depth_mm_ = declare_parameter<double>("pick_tilted_depth_mm", 30.0);     // 손을 10° 넘게 기울인 PICK 의 쥐는 깊이 (퍽 무게중심)
    surface_z_ = declare_parameter<double>("puck_surface_z", 0.037);
    puck_h_ = declare_parameter<double>("puck_height", 0.060);
    // 누운 네모 퍽: 높이 = 한 변 (34.8 mm). 수직 손 · 닫힘축 = 긴축에 수직 (손가락이 긴 옆면 두 개를 쥠), 윗면 위 40 · 10 · 쥐기 −20
    //   −20: 손끝이 놓인 면 위 34.8 − 20 = 14.8 mm (10 mm 넘게 띄움 — 윗면 깊이 오차 · 처짐 몇 mm 에도 바닥에 안 닿음),
    //   손가락이 옆면 위쪽 20 mm (14.8 ~ 34.8) 를 쥐어 무게중심 높이 17.4 mm 를 덮음. 인지는 누운 퍽 중심을 윗면 − 15 mm 로 내므로
    //   (wrist_pose PUCK_D 30 mm) 윗면을 2.4 mm 높게 봄 → 실제 손끝은 그만큼 더 높음 (안전 쪽)
    //   실행 중 바로 바꿈 (다음 PICK 부터): ros2 param set /control_master pick_lying_heights_mm "[40.0, 10.0, -24.0]"
    //   쥐기는 손끝이 놓인 면 위 10 mm 넘게 (계산상 −24.8 까지) — 실물에서 명령보다 얕게 쥐면 (선 퍽: 명령 −40 → 실제 약 30) 이 안에서 깊게
    lying_h_ = declare_parameter<double>("puck_lying_height", 0.0348);
    declare_parameter<std::vector<double>>("pick_lying_heights_mm", {40.0, 10.0, -20.0});
    grasp_offset_ = declare_parameter<double>("grasp_offset_m", 0.015);   // planning 과 같은 값 (docs/planning/planning_interfaces.md 6 절)
    home_near_tol_ = declare_parameter<double>("home_direct_near", 0.2);
    // 이웃 퍽 (손목 인지) → 집기 · 놓기 닫힘 방향 · 벌림 폭 (grasp_select)
    nb_age_s_ = declare_parameter<double>("neighbor_max_age_s", 10.0);   // 차체가 멈춰 있는 동안 base_link 위치는 그대로 (LOOK 뒤 인지를 꺼도 씀)
    nb_radius_ = declare_parameter<double>("neighbor_radius", 0.12);
    nb_min_clear_ = declare_parameter<double>("grasp_min_clear", 0.002);
    nb_target_clear_ = declare_parameter<double>("grasp_target_clear", 0.003);
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    // 수직 손이 안 닿는 먼 퍽 · 자리: 손을 10° 씩 바깥으로 기울여 봄 (20° ≈ 31 cm). 0 = 수직 손만
    //   기울여 집은 퍽은 손 안에서 그만큼 기울어 있음 → STORE 는 10° 까지, PLACE 는 쥘 때 기울임과 20° 안으로 놓음 (넘어지지 않게)
    max_tilt_deg_ = declare_parameter<double>("pick_max_tilt_deg", 20.0);
    // 손목 카메라로 잡은 퍽을 실물에서 늘 한쪽으로 비껴 쥐면 그만큼 PICK 쥐는 점을 옮김 [mm, base_link] (25 까지)
    //   윗면 태그가 보인 퍽 (검출 id 있음) 은 인지가 태그로 중심을 잡으므로 옮기지 않음
    //   실행 중 바로 바꿈: ros2 param set /control_master pick_offset_x_mm -5.0 (다음 PICK 부터)
    declare_parameter<double>("pick_offset_x_mm", 0.0);
    // 다시 잡기: 수직 손이 안 닿아 기울여 쥔 퍽은 손 안에서 그만큼 기울어 STORE 때 칸 테두리에 걸림 (11-04 실물, 10° 에서 매번)
    //   → 같은 PICK 안에서 박스 위 차체 쪽 넉넉한 빈 자리 (손목 인지로 확인, 집어 든 퍽 자리는 빈 것으로) 에 세워 내려놓고
    //   수직 손으로 다시 쥠 (regrasp_nodes). 없으면 PICK 거절
    //   regrasp_min_x: 내려놓는 퍽 중심 x 하한 [m] — 박스 앞면이 0.191 m (정차 3 cm) 여도 퍽 (반폭 17.4 mm) 이 박스 위
    regrasp_ = declare_parameter<bool>("regrasp_tilted", true);
    regrasp_min_x_ = declare_parameter<double>("regrasp_min_x", 0.212);
    declare_parameter<double>("pick_offset_y_mm", 0.0);
    // 윗면 태그로 잰 퍽 (인지 중심이 2 mm 안) 은 팔 쪽 치우침만: ros2 param set /control_master pick_offset_tagged_x_mm -3.0
    declare_parameter<double>("pick_offset_tagged_x_mm", 0.0);
    declare_parameter<double>("pick_offset_tagged_y_mm", 0.0);
    // 누운 퍽을 위 옆면 태그로 잰 중심: 아직 실물로 못 잼 → 처음엔 선 퍽 태그 값과 같게 (config). 받침 세우기는 긴축 ±2 mm 안이 필요
    declare_parameter<double>("pick_offset_lying_tagged_x_mm", 0.0);
    declare_parameter<double>("pick_offset_lying_tagged_y_mm", 0.0);
    // 태그가 안 보인 누운 퍽 (깊이 점 중심): 선 퍽 pick_offset_x/y 와 따로 (11-06 실물: 태그 보이면 맞는데 안 보이면 선 퍽 값이라 어긋남)
    declare_parameter<double>("pick_offset_lying_x_mm", 0.0);
    declare_parameter<double>("pick_offset_lying_y_mm", 0.0);
    // 손목을 돌려 쥘 때 보정 [mm]: 손가락 가운데가 J5 축에서 12 mm 떨어져 있어, 팔 모델의 그 거리가 조금 틀리면 쥐는 점 오차가 손목 방향에 따라 달라짐.
    //   값 = 손목을 기본 (닫힘축 ≈ 접선) 에서 90° 돌려 쥘 때 (실물 11-04: 닫힘축이 반지름 방향) 더할 보정 [앞 · 왼쪽], 팔 바로 앞 퍽 기준.
    //   다른 각도 · 자리는 같은 오차 (손 좌표의 손끝 어긋남) 로 계산: (R(돌린 각) − I) · h. 기본 방향 (돌림 0) 은 0 → 다른 보정 그대로
    //   실행 중: ros2 param set /control_master pick_offset_rot_mm "[3.0, 0.0]"
    declare_parameter<std::vector<double>>("pick_offset_rot_mm", std::vector<double>{0.0, 0.0});
    // 손목을 돌려 쥘 때 (기본에서 45° 넘게, 어느 쪽이든) 그대로 더하는 보정 [앞 · 왼쪽 mm, base_link = 로봇 기준]. 위 모델 보정 위에 더함
    //   (11-07 실물: 모델 값을 바꾸니 돌린 각 · 자리에 따라 엉뚱한 쪽으로 감 → 사용자 말 그대로 로봇 기준으로)
    declare_parameter<std::vector<double>>("pick_offset_rot_robot_mm", std::vector<double>{0.0, 0.0});
    // STORE 때 칸 근처 경로를 더 옮김 [mm, base_link, 넣는 점 맞추기와 합해 20 까지] — 쥔 퍽이 칸에서 늘 한쪽으로 걸리면. 적재함 태그 보정 · 칸별 store_offset.<칸> 과 더해짐
    //   실행 중 바로 바꿈: ros2 param set /control_master store_offset_y_mm 3.0 (다음 STORE 부터, x+ = 앞 · y+ = 왼쪽)
    declare_parameter<double>("store_offset_x_mm", 0.0);
    // STORE 넣는 점 맞추기: 경로의 놓는 점 (grip release) 손끝을 팔 모델로 계산해 칸 표 (칸 중심 + place_offset) 로 옮김.
    //   실물 기록 재생 경로는 놓는 점이 칸보다 팔 J1 축 반대쪽으로 7 ~ 13 mm 나가 있음 (실물: S7 +13 · S12 +5 mm 에서 들어감 = 이 계산과 2 mm 안)
    declare_parameter<bool>("store_to_cell", false);
    declare_parameter<double>("store_offset_y_mm", 0.0);
    tf_buf_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listen_ = std::make_unique<tf2_ros::TransformListener>(*tf_buf_);
    // 손목 카메라가 손목에서 돈 만큼 (camera_drift_node, 홈 사진) — 손목 검출을 카메라 좌표에서 되돌린 뒤 TF 로 바꿈
    drift_correct_ = declare_parameter<bool>("wrist_cam_drift_correct", true);
    sub_drift_ = create_subscription<geometry_msgs::msg::QuaternionStamped>(
      declare_parameter<std::string>("wrist_cam_drift_topic", "/wrist_cam/drift"), rclcpp::QoS(1).transient_local().reliable(),
      [this](geometry_msgs::msg::QuaternionStamped::ConstSharedPtr m) {
        const Eigen::Quaterniond q(m->quaternion.w, m->quaternion.x, m->quaternion.y, m->quaternion.z);
        if (!(std::abs(q.norm() - 1.0) < 0.01)) {return;}
        drift_frame_ = m->header.frame_id;
        drift_q_ = q.normalized();
        RCLCPP_INFO(get_logger(), "손목 카메라 보정 받음: %.2f° (%s)%s", Eigen::AngleAxisd(drift_q_).angle() * 180.0 / M_PI, drift_frame_.c_str(),
          drift_correct_ ? "" : " — wrist_cam_drift_correct false 라 안 씀");
      });
    sub_dets_ = create_subscription<interfaces::msg::DetectionArray>(
      declare_parameter<std::string>("neighbor_topic", "/perception/wrist/detections"), rclcpp::QoS(10),
      [this](interfaces::msg::DetectionArray::ConstSharedPtr m) {on_detections(*m);});
    // ---- 적재함 태그로 칸 경로 옮기기 (위 머리말) ----
    {
      rack_names_ = declare_parameter<std::vector<std::string>>("rack_cell_names", std::vector<std::string>{});
      const auto rx = declare_parameter<std::vector<double>>("rack_cell_x", std::vector<double>{});
      const auto ry = declare_parameter<std::vector<double>>("rack_cell_y", std::vector<double>{});
      if (rack_names_.empty() || rx.size() != rack_names_.size() || ry.size() != rack_names_.size()) {
        RCLCPP_WARN(get_logger(), "rack_cell_names · x · y 없음 (config/rack_cells.yaml) → 적재함 태그로 칸 경로를 안 옮김");
        rack_names_.clear();
      } else {
        for (size_t i = 0; i < rx.size(); ++i) {rack_xy_.emplace_back(rx[i], ry[i]);}
        const auto px = declare_parameter<std::vector<double>>("rack_cell_place_x", std::vector<double>{});
        const auto py = declare_parameter<std::vector<double>>("rack_cell_place_y", std::vector<double>{});
        for (size_t i = 0; i < rx.size(); ++i) {
          rack_place_.emplace_back(px.size() == rx.size() ? px[i] : 0.0, py.size() == rx.size() ? py[i] : 0.0);
        }
        for (const auto & c : rack_xy_) {rack_pivot_ += c / static_cast<double>(rack_xy_.size());}
        // 칸마다 STORE 보정 [x, y] mm (store_offset_x · y_mm 와 더함): ros2 param set /control_master store_offset.S7 "[0.0, 5.0]"
        for (const auto & n : rack_names_) {declare_parameter<std::vector<double>>("store_offset." + n, std::vector<double>{0.0, 0.0});}
        // 칸마다 RETRIEVE 보정 [x, y] 또는 [x, y, 높이] mm (11-07: STORE 보정은 빼기에 안 들어감 → 따로): ros2 param set /control_master retrieve_offset.S12 "[-10.0, -5.0]"
        for (const auto & n : rack_names_) {declare_parameter<std::vector<double>>("retrieve_offset." + n, std::vector<double>{0.0, 0.0});}
      }
    }
    rack_ref_file_ = declare_parameter<std::string>("rack_reference_file", "~/.ros/rack_reference.txt");
    if (rack_ref_file_.rfind("~/", 0) == 0 && std::getenv("HOME")) {rack_ref_file_ = std::getenv("HOME") + rack_ref_file_.substr(1);}
    rack_min_shift_m_ = declare_parameter<double>("rack_shift_min_m", 0.0015);
    rack_max_shift_m_ = declare_parameter<double>("rack_shift_max_m", 0.008);
    rack_max_rms_m_ = declare_parameter<double>("rack_fit_max_rms_m", 0.003);
    rack_min_samples_ = declare_parameter<int>("rack_min_samples", 5);
    load_rack_reference();
    sub_rack_ = create_subscription<interfaces::msg::DetectionArray>(
      declare_parameter<std::string>("rack_tag_topic", "/rack_tags/detections"), rclcpp::QoS(10),
      [this](interfaces::msg::DetectionArray::ConstSharedPtr m) {on_rack_tags(*m);});
    // 구분 동작 (park_step): 누운 퍽 세우기를 동작마다 멈춰 가며 — 다음으로: ros2 service call /control_master/step std_srvs/srv/Trigger
    declare_parameter<bool>("park_step", false);
    srv_step_ = create_service<std_srvs::srv::Trigger>("~/step",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        if (!runner_.waiting_step()) {res->success = false; res->message = "구분 동작으로 기다리는 중이 아님"; return;}
        res->message = "다음 동작: " + runner_.next_name();
        runner_.step_go();
        res->success = true;
      });
    srv_rack_ref_ = create_service<std_srvs::srv::Trigger>("~/save_rack_reference",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        res->success = save_rack_reference(res->message);
      });
    // 누운 퍽 쥐기 깊이 한도: 손끝이 놓인 면 위 이만큼 (모델). 실물은 명령보다 약 10 mm 얕게 쥠 (11-05 누운 퍽 · 11-04 선 퍽 −40 → 30) → config 0
    declare_parameter<double>("pick_lying_tip_clear_mm", 10.0);
    // 세우기 경로 전체를 위아래로 옮김 [mm, − = 아래]: 실물이 명령보다 얕게 쥐어 시뮬 (−20 mm 쥐기) 과 같게 하려면 −10
    declare_parameter<double>("standup_dz_mm", 0.0);
    declare_parameter<double>("standup_tip_clear_mm", 2.0);   // 세우기 경로 손끝 최저 = 박스 윗면 위 이만큼 (모델)
    declare_parameter<double>("standup_reach_scale", 1.0);    // 쥔 뒤 경로의 수평 이동 배율 (1 = 표 그대로, 0.5 = 절반만 뻗음)
    declare_parameter<double>("standup_time_scale", 1.0);     // 세우기 경로 점 사이 시간 = 녹화 0.1 s × 이 배 (1 = 시뮬과 같은 속도)
    declare_parameter<double>("standup_open_time_s", 0.5);   // 팔 안 멈추고 하는 그리퍼 동작 시간 [s] (세우기 여는 점 · PICK 미리 열기 · 놓은 뒤 닫기. 시뮬 정책: 스텝마다 2 mm → 약 0.5 s)
    declare_parameter<bool>("gripper_on_the_way", true);      // 빈손일 때 그리퍼를 팔 멈추지 않고 (PICK 퍽 위로 가며 열기 · STORE · PLACE 놓은 뒤 닫기)
    // 누운 네모 퍽을 PICK 하면서 세움 (false = 누운 채 쥐고 기준점으로, 예전대로). 실행 중: ros2 param set /control_master stand_lying true
    declare_parameter<bool>("stand_lying", false);
    // 세우는 방법: park = 그리퍼 받침에 기울여 넣고 손가락으로 밀어 세운 뒤 다시 쥠 (11-06 시뮬 12 중 10), table = 강화학습 표 재생 (박스 위)
    declare_parameter<std::string>("stand_lying_mode", "park");
    declare_parameter<std::vector<double>>("park_center", {0.1008, 0.00115});   // 그리퍼 받침 중심 [m, base_link] (URDF gripper_park_stand_mount)
    declare_parameter<double>("park_floor_z", 0.1365);          // 받침 바닥 윗면 [m, base_link] (받침 판 0.1305 + 바닥 6 mm)
    declare_parameter<double>("park_tilt_deg", 45.0);           // 넣을 때 손 기울임 (바깥 +). 안 되면 5° 씩 줄여 40 까지
    declare_parameter<double>("park_grasp_offset_mm", 25.0);    // 누운 퍽을 중심에서 한쪽 끝으로 옮겨 쥠 (긴 쪽이 받침 안으로 내려감)
    declare_parameter<double>("park_insert_clear_mm", 4.0);     // 넣을 때 아래 끝 가장 낮은 모서리 ↔ 받침 바닥
    declare_parameter<double>("park_end_inset_mm", 4.0);        // 넣을 때 아래 끝면 중심 = 받침 중심에서 J1 쪽으로 (기대는 각을 정함, park_stand.hpp)
    declare_parameter<double>("park_push_h_mm", 25.0);          // 미는 손끝 높이 = 받침 바닥 위 (시뮬 25 · 35 다 됨, 30 은 두 번 실패)
    declare_parameter<double>("park_regrasp_depth_mm", 30.0);   // 받침에 선 퍽 다시 쥐기: 윗면에서 손끝까지 (손끝이 받침 벽 위)
    declare_parameter<double>("park_regrasp_r_mm", -6.0);       // 선 퍽 다시 쥐는 자리 = 받침 중심에서 반지름 방향 (− = J1 쪽)
    declare_parameter<double>("park_insert_j3_deg", 0.0);      // 넣는 자세 J3 를 이만큼 더 [°, + = 손끝 위] (넣는 점에서 5° ≈ 위 22 · 바깥 9 mm)
    declare_parameter<int>("park_loose_step", -1);              // 넣으며 기울이는 점 (1–8) 중 이 번째부터 손가락을 park_loose_grip_m 으로 풂. −1 = 안 풂
    declare_parameter<double>("park_insert_back_mm", 0.0);     // > 0: 다 기울인 자세를 이만큼 위에서 만들고 그 자세로 곧게 넣음 (from_back 이 있으면 뒤 위에서 대각선)
    declare_parameter<double>("park_insert_extra_mm", 0.0);    // 넣는 끝점을 이만큼 더 아래로 (턱에 걸치면 밀어 넣기)
    declare_parameter<double>("park_insert_from_back_mm", 0.0);  // 곧게 넣기 출발점을 J1 쪽 (로봇 뒤) 으로 이만큼 → 뒤 위에서 앞 아래로 대각선으로 넣음
    declare_parameter<double>("park_release_pull_mm", 0.0);      // > 0: 넣는 점에서 손가락만 살짝 풀고 (park_loose_grip_m) 같은 높이로 J1 쪽 (뒤) 으로 이만큼 곧게 당기며 놓음. 0 = 그 자리에서 놓음
    declare_parameter<double>("park_release_pull_dz_mm", 0.0);   // 당기는 동안 위아래 (− = 내려가며)
    declare_parameter<double>("park_push_extra_mm", 0.0);        // 밀어 세우기 끝점을 몸 (J1) 쪽으로 이만큼 더 (선 퍽 바깥 면 + 4 mm 앞에서 멈추던 것)
    declare_parameter<double>("park_push_scoop_deg", 0.0);       // > 0: 굴삭기처럼 밀기 — 손끝을 몸 쪽으로 이만큼 기울여 낮게 시작, 밀면서 손을 세우며 올라감
    declare_parameter<double>("park_push_low_mm", 0.0);          // 굴삭기처럼 밀기 시작 높이를 park_push_h_mm 보다 이만큼 낮게 (받침 바깥 벽 22.5 mm 위를 지나야 함)
    declare_parameter<double>("park_push_j3_deg", 0.0);         // 밀어 세우는 동안 J3 를 이만큼 더 [°, + = 손끝 위], 밀기 가운데서 전부 · 시작 · 끝점 0 (11-07 사용자: J3 를 더 굽혀, 끝에선 펴기)
    declare_parameter<double>("park_push_j2_deg", 0.0);         // 밀어 세우는 동안 J2 를 이만큼 더 올림 [°, + = 손끝 위], J3 와 같은 모양 → 퍽 뒷면 아래 모서리를 긁어 올려 세움 (11-07 사용자)
    declare_parameter<bool>("park_press_mode", false);          // true = 밀기 대신 누르기: 놓은 자리에서 살짝 올리며 그리퍼 닫고 → 뒤 (J1 쪽) 로 → 눌러 내려 걸린 퍽을 세움 (11-07 사용자)
    declare_parameter<double>("park_press_lift_mm", 10.0);      // 누르기: 놓은 자리에서 올리는 높이
    declare_parameter<double>("park_press_back_mm", 15.0);      // 누르기: 올린 뒤 J1 쪽 (뒤) 으로
    declare_parameter<double>("park_press_depth_mm", 20.0);     // 누르기: 거기서 곧게 눌러 내리는 깊이 (실물 값이면 손끝이 받침 벽 위 3 mm 에서 멈춤)
    declare_parameter<double>("park_release_up_mm", 0.0);        // 넣는 끝점을 손등 (손목 카메라) 쪽으로 이만큼 (넣으면서 같이, 손 축에 수직)
    declare_parameter<int>("park_release_pull_steps", 3);       // 당기는 점 수
    declare_parameter<double>("park_speed_dps", 20.0);          // 받침에서 넣기 · 다시 쥐기 속도
    declare_parameter<double>("park_push_speed_dps", 20.0);     // 받침에서 밀어 세우는 속도 (빠르면 손끝이 퍽 면을 타고 올라 넘어짐)
    declare_parameter<bool>("store_match_grip_depth", true);   // STORE: 쥔 깊이가 박스 PICK 깊이와 다르면 (받침에서 다시 쥔 퍽) 칸 근처 높이를 그 차만큼
    declare_parameter<std::vector<double>>("look_rack_bias_mm", {0.0, 0.0});   // LOOK_RACK 칸 측정의 카메라 치우침 [x, y] mm (제자리 적재함에서 잰 값 − 칸 표)
    declare_parameter<bool>("pick_end_at_staging", false);       // PICK 끝에 박스 위 기준점으로 (false = 집은 자리에서 끝, STORE · PLACE 가 이어 감)
    declare_parameter<bool>("place_far_regrasp", true);         // 먼 자리 PLACE: 가까운 빈 자리에 내려놓고 손을 기울여 다시 쥔 뒤 놓기 (place_far_plan)
    declare_parameter<double>("park_max_tilt_deg", 30.0);       // 먼 누운 퍽: 쥘 때 손을 10° 씩 이만큼까지 기울여 봄 (0 = 수직 손만)
    declare_parameter<double>("park_max_roll_deg", 12.0);       // 기울인 손에 퍽이 굴러 맞는 각 최대 (시뮬: 20° 는 들다 놓침)
    declare_parameter<double>("park_roll_margin_mm", 2.5);      // 기울인 손에 퍽이 굴러 맞을 때 들어가는 열린 손가락 ↔ 퍽 최소 여유
    standup_speed_dps_ = declare_parameter<double>("standup_speed_dps", 32.0);   // 시뮬 정책은 40 °/s (스텝 0.05 s 마다 2°). 1.25 배 느리게 재생해도 시뮬 91.8 % (40 은 89.9 %)
    // 세운 뒤 바로 다시 쥠 (true) · 손을 들고 LOOK 자세에서 끝냄 (false, 기본: 퍽은 선 채 박스에, 다음 pick det 로 카메라로 다시 재서 집음).
    //   시뮬 재생에서 선 자리가 표와 90 % 가 12 mm 안 (손가락이 받는 폭 약 ±8 mm) → 바로 다시 쥐면 가끔 놓침
    declare_parameter<bool>("standup_regrasp", false);
    standup_max_gap_mm_ = declare_parameter<double>("standup_max_gap_mm", 6.0);
    // 쥐기 뒤 확인: 손끝이 쥐는 자리에서 이보다 멀면 (퍽 위에 얹힘) · 그리퍼가 이보다 닫히면 (빈손) 세우기를 안 하고 멈춤
    standup_reach_tol_mm_ = declare_parameter<double>("standup_reach_tol_mm", 6.0);
    standup_grip_min_m_ = declare_parameter<double>("standup_grip_min_m", 0.001);
    standup_max_gap_deg_ = declare_parameter<double>("standup_max_gap_deg", 6.0);
    {
      const auto f = declare_parameter<std::string>("standup_paths_file", "");
      std::string w;
      if (f.empty()) {
        RCLCPP_INFO(get_logger(), "세우기 경로 표 없음 (standup_paths_file) → 누운 퍽은 누운 채 PICK");
      } else if (!load_standup_table(f, standup_, w)) {
        RCLCPP_WARN(get_logger(), "%s → 누운 퍽은 누운 채 PICK", w.c_str());
        standup_.clear();
      } else {
        RCLCPP_INFO(get_logger(), "세우기 경로 표: %zu 개 (%s)", standup_.size(), f.c_str());
      }
    }
    declare_parameter<std::string>("held_reference_file", "~/.ros/held_reference.txt");
    // 손 안 위치로 STORE 경로 옮기기 (false = 재서 로그만). 실행 중: ros2 param set /control_master held_correct true
    declare_parameter<bool>("held_correct", false);
    held_max_mm_ = declare_parameter<double>("held_max_mm", 8.0);   // 기준과 이보다 다르면 잘못 잰 것으로 보고 안 옮김
    load_held_reference();
    sub_held_ = create_subscription<interfaces::msg::DetectionArray>(
      declare_parameter<std::string>("held_tag_topic", "/held_puck/tag"), rclcpp::QoS(10),
      [this](interfaces::msg::DetectionArray::ConstSharedPtr m) {on_held_tag(*m);});
    srv_held_ref_ = create_service<std_srvs::srv::Trigger>("~/save_held_reference",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        res->success = save_held_reference(res->message);
      });
    slow_dps_ = declare_parameter<double>("near_puck_speed_dps", 15.0);
    path_slow_min_dps_ = declare_parameter<double>("path_slow_min_dps", 0.0);   // 실물 경로 느린 구간의 최소 속도 (0 = 기록 그대로 15)
    // 경로별 속도 비율 "경로 이름=비율" (11-07 사용자: S2 로 갈 때 10 % 느리게 → "store_S2=0.9"). 실행 중 바꾸면 다음 작업부터
    declare_parameter<std::vector<std::string>>("path_speed_scale", std::vector<std::string>{});
    model_ok_ = load_model();
    for (const char * p : {"home", "pick_above", "pick_down", "place_above", "place_down"}) {
      poses_[p] = declare_parameter<std::vector<double>>(
        std::string("poses.") + p, std::vector<double>(arm_joints_.size(), 0.0));
    }
    cancel_timeout_s_ = declare_parameter<double>("cancel_timeout_s", 2.0);
    // ---- 접힘 실측 확인 (R1) ----
    const double verify_timeout_s = declare_parameter<double>("verify_timeout_s", 2.0);
    const auto fold_tol = declare_parameter<std::vector<double>>("fold_tolerance", {0.05});
    const double fold_hold_s = declare_parameter<double>("fold_hold_s", 0.3);
    const double fold_max_age_s = declare_parameter<double>("fold_max_age_s", 0.2);
    fold_ = std::make_unique<FoldCheck>(poses_.at("home"), fold_tol, fold_hold_s, fold_max_age_s);
    runner_ = ArmRunner(accept_timeout_s_, move_time_s_ + motion_margin_s_, cancel_timeout_s_, verify_timeout_s);
    // ---- 하드웨어 이상 감지 (재검증 R6) ----
    hw_ = std::make_unique<HardwareWatch>(
      declare_parameter<double>("hw_state_timeout_s", 0.3), declare_parameter<double>("hw_bad_hold_s", 0.1),
      declare_parameter<double>("hw_component_timeout_s", 1.0));
    hw_watch_joints_ = declare_parameter<std::vector<std::string>>("hw_watch_joints", {
      "drive_fl_joint", "drive_fr_joint", "drive_rl_joint", "drive_rr_joint",
      "steer_fl_joint", "steer_fr_joint", "steer_rl_joint", "steer_rr_joint",
      "joint1", "joint2", "joint3", "joint4", "joint5", "gripper_left_joint"});
    const double hw_component_hz = declare_parameter<double>("hw_component_hz", 5.0);
    // ---- 차례 ----
    arbiter_ = MotionArbiter(declare_parameter<double>("settle_s", 0.5), declare_parameter<bool>("start_locked", true));
    const double turn_hz = declare_parameter<double>("turn_publish_hz", 10.0);
    still_lin_ = declare_parameter<double>("still_linear", 0.01);
    still_ang_ = declare_parameter<double>("still_angular", 0.05);
    odom_timeout_s_ = declare_parameter<double>("odom_timeout_s", 0.5);

    // ================= 받는 것 =================
    // 몸통이 멈췄나 = swerve_controller 가 측정값으로 계산한 /odom 의 속도
    //   받은 시각이 아니라 **측정 시각(header.stamp)이 새로워졌을 때만** 신선한 것으로 침
    //   (같은 옛날 odom 이 반복해서 와도 "지금 멈춰 있음" 으로 착각하지 않게)
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
        const rclcpp::Time stamp(m->header.stamp);
        if (have_odom_ && stamp <= odom_stamp_) {return;}
        const auto & t = m->twist.twist;
        odom_still_ = std::hypot(t.linear.x, t.linear.y) < still_lin_ && std::fabs(t.angular.z) < still_ang_;
        odom_stamp_ = stamp;
        have_odom_ = true;
      });
    // 팔 관절 실측 (joint_state_broadcaster). arm_joints 순서로 골라 FoldCheck 에. 없는 관절은 NaN (= 모름)
    //   측정 시각 = header.stamp (하드웨어가 못 읽은 모터는 NaN 으로 내보냄 — opencr_hardware.cpp, R3)
    sub_joints_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::JointState::ConstSharedPtr m) {
        joint_buf_.assign(arm_joints_.size(), std::nan(""));
        for (std::size_t i = 0; i < m->name.size() && i < m->position.size(); ++i) {
          for (std::size_t k = 0; k < arm_joints_.size(); ++k) {
            if (m->name[i] == arm_joints_[k]) {joint_buf_[k] = m->position[i];}
          }
          if (m->name[i] == gripper_joint_) {grip_now_ = m->position[i];}
        }
        const double stamp = rclcpp::Time(m->header.stamp).seconds();
        fold_->observe(stamp, joint_buf_);
        // 감시할 관절이 전부 있고 NaN 이 아닌가 (주행 · 팔 보드 둘 다 — 빠진 관절 = 그 보드의 상태가 안 옴)
        bool all_measured = true;
        for (const auto & w : hw_watch_joints_) {
          bool found = false;
          for (std::size_t i = 0; i < m->name.size() && i < m->position.size(); ++i) {
            if (m->name[i] == w) {found = std::isfinite(m->position[i]); break;}
          }
          all_measured = all_measured && found;
        }
        hw_->observe(stamp, all_measured);
      });
    // "이 번호로 잠갔음" 답 (swerve_controller, 20 Hz). DRIVE 가 아닌 답만 잠금 확인으로 씀
    sub_ack_ = create_subscription<interfaces::msg::MotionMode>(
      "control/motion_mode_ack", 10, [this](interfaces::msg::MotionMode::ConstSharedPtr m) {
        if (m->mode != interfaces::msg::MotionMode::DRIVE) {lock_acked_id_ = m->request_id;}
      });

    arm_server_ = rclcpp_action::create_server<ArmTask>(
      this, "arm_task",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const ArmTask::Goal> goal) {
        return on_arm_goal(*goal);
      },
      [](const std::shared_ptr<ArmGoalHandle>) {return rclcpp_action::CancelResponse::ACCEPT;},
      [this](const std::shared_ptr<ArmGoalHandle> gh) {on_arm_accepted(gh);});

    // ================= 보내는 것 =================
    // 차례: swerve_controller 가 이걸 보고 바퀴를 움직임 (늦게 켜진 쪽도 마지막 값을 받도록 transient_local)
    pub_turn_ = create_publisher<interfaces::msg::MotionMode>(
      "control/motion_mode", rclcpp::QoS(1).reliable().transient_local());
    arm_client_ = rclcpp_action::create_client<FJT>(this, "arm_controller/follow_joint_trajectory");
    // 보드(하드웨어) 상태 직접 묻기 — 응답을 기다리는 요청은 하나만 (취소 누적 R8 과 같은 이유)
    hw_list_client_ = create_client<ListHw>("controller_manager/list_hardware_components");
    hw_timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1.0 / hw_component_hz), [this]() {ask_components();});
    grip_client_ = rclcpp_action::create_client<FJT>(this, "gripper_controller/follow_joint_trajectory");

    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1.0 / rate_hz), [this]() {step();});
    // 차례 심장박동: 바뀌지 않아도 계속 보냄 → swerve_controller 가 "끊김 = 잠금" 으로 판단할 수 있게
    turn_timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1.0 / turn_hz), [this]() {publish_turn();});

    publish_turn();
    RCLCPP_INFO(get_logger(), "control_master 시작: %s, 팔 관절 %zu개, 팔 한 동작 %.1f s",
      arbiter_.turn() == Turn::STOP ? "바퀴 잠금 (HOME 이 끝나면 풀림)" : "바퀴 차례", arm_joints_.size(), move_time_s_);
  }

private:
  // ======================================================================== 팔 작업 받기
  rclcpp_action::GoalResponse on_arm_goal(const ArmTask::Goal & goal)
  {
    if (arm_goal_) {
      RCLCPP_WARN(get_logger(), "팔 작업이 이미 진행 중 → 새 작업 거절");
      return rclcpp_action::GoalResponse::REJECT;
    }
    // 준비 조건 (방송의 ready 와 같은 함수 — 방송과 실제 받기 판단이 어긋나지 않게)
    std::string why;
    if (!task_ready(now().seconds(), why)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "%s → 팔 작업 거절", why.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    // 계획 (IK 여러 번, 다시 잡기면 젯슨에서 0.5 s 넘음) 동안 이 노드는 관절 측정을 못 받음 → 끝난 뒤 밀린 측정을 받을 때까지
    //   하드웨어 감시가 '측정 끊김' 으로 오판하지 않게 잠깐 이전 판정을 유지 (11-04 실물: 0.59 s 로 PICK 중단)
    const auto t_plan = std::chrono::steady_clock::now();
    pending_step_ = false;
    pending_plan_ = build_plan(goal, why);
    const double plan_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_plan).count();
    hw_grace_until_ = now().seconds() + 0.3;
    if (plan_s > 0.2) {RCLCPP_INFO(get_logger(), "팔 작업 %u 계획 %.2f s", static_cast<unsigned>(goal.task), plan_s);}
    if (pending_plan_.empty()) {
      RCLCPP_WARN(get_logger(), "팔 작업 %u%s%s 거절: %s", static_cast<unsigned>(goal.task),
        goal.slot.empty() ? "" : " 칸 ", goal.slot.c_str(), why.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  // 지금 새 팔 작업을 받을 수 있나 (/control/motion_mode 의 ready). 바퀴 허가와는 별개 — 바퀴는 차례(DRIVE)만
  //   ① (재검증 R6) 하드웨어: 보드 상태 · 관절 측정이 신선하고 정상. 이상이면 복구(재설정) 뒤 planning 이 다시 보냄
  //   ② 켤 때 팔 · 그리퍼 재생기가 아직 안 떴으면 받지 않음 (받고 나서 실패 → FAULT 가 되지 않게)
  //   진행 중인 작업 · 모르는 작업 번호는 "준비" 가 아니라 그 요청의 문제라서 여기 넣지 않음
  bool task_ready(double now_s, std::string & why) const
  {
    if (!hw_->healthy(now_s)) {
      why = "하드웨어 이상 (" + hw_->status(now_s) + ")";
      return false;
    }
    if (!arm_client_->action_server_is_ready() || !grip_client_->action_server_is_ready()) {
      why = "팔 · 그리퍼 재생기(JTC)가 아직 준비 안 됨 (planning 이 다시 보냄)";
      return false;
    }
    if (auto_folding_) {
      why = "P0 에서 쉬다 자동으로 접는 중 (planning 이 다시 보냄)";
      return false;
    }
    why.clear();
    return true;
  }

  void on_arm_accepted(const std::shared_ptr<ArmGoalHandle> gh)
  {
    arm_goal_ = gh;
    const auto plan = std::move(pending_plan_);
    pending_plan_.clear();
    task_ends_p0_ = plan_ends_p0_;
    p0_idle_since_ = -1.0;
    runner_.start(plan);
    runner_.set_step(pending_step_);
    if (pending_step_) {
      RCLCPP_WARN(get_logger(), "구분 동작: 첫 동작 뒤로는 동작마다 멈춤 → 다음으로: ros2 service call /control_master/step std_srvs/srv/Trigger "
        "(park_step 끄기: ros2 param set /control_master park_step false)");
    }
    pending_step_ = false;
    step_logged_ = false;
    scanning_ = gh->get_goal()->task == ArmTask::Goal::LOOK && gh->get_goal()->slot == "rack";
    scan_seen_.clear();
    arbiter_.request_arm();   // 바퀴부터 세우기 → 잠금 답 + 몸통이 멈추면 ARM
    publish_turn();
    RCLCPP_INFO(get_logger(), "팔 작업 %u 받음 (동작 %zu개) → 바퀴 세우는 중",
      static_cast<unsigned>(gh->get_goal()->task), plan.size());
    {   // 동작별 시간 (미션 시간 줄이기 — 어디서 시간이 드는지)
      std::string line;
      double total = 0.0;
      char buf[96];
      for (const auto & a : plan) {
        const double t = a.is_gripper ? gripper_time_s_ : (a.points.empty() ? move_time_s_ : a.duration_s);
        total += t;
        std::snprintf(buf, sizeof(buf), "%s%s %.1f", line.empty() ? "" : " · ", a.is_gripper ? "[그리퍼]" : a.pose.c_str(), t);
        line += buf;
      }
      RCLCPP_INFO(get_logger(), "팔 작업 %u 시간: 합 %.1f s = %s", static_cast<unsigned>(gh->get_goal()->task), total, line.c_str());
    }
  }

  // ======================================================================== 작업 → 동작 목록
  // STORE · RETRIEVE · LOOK 류 · 접기는 실물에서 검사 · 실행한 경로(config/arm_paths.yaml) 그대로. 팔이 경로 출발 자세에 있어야 함
  //   STORE <칸>   : 박스 위 들기(퍽 쥠) → 칸에 넣기 → 받침 (주행 허용). lazy_fold 면 P0 (아래)
  //     STORE · RETRIEVE 는 적재함이 기준 스캔에서 움직였으면 칸 근처를 옮긴 경로 (rack_adjust, 머리말)
  //   RETRIEVE <칸>: 받침 → 칸에서 꺼내기 → 박스 위 들기 (퍽 쥠, 바퀴 잠금 유지 — 이어서 PLACE). LOOK 자세면 복귀 경로로 받침을 거쳐 감
  //     (경로 앞이 unfold 그대로면 P0 에서 이음)
  //   LOOK · LOOK_BARCODE: 받침 → 관찰 자세 (그 자세 유지). 이미 그 자세면 그대로
  //   LOOK slot "rack": 빈 손 적재함 스캔 (경로 rack_scan) — 받침 → 칸 두 개씩 손목 카메라로 내려다보며 2 s 씩 → 받침 (주행 허용)
  //     칸 바닥 태그로 칸 위치를 잼 (manipulation rack_tag_node). 퍽 쥐면 거절. 칸에 퍽이 있어도 경로는 퍽 위로 지나감 (그 칸은 못 잼)
  //   HOME: 접혀 있으면 home 한 점, 아니면 지금 자세에서 출발하는 검사된 복귀 경로(home_from_*), 없으면 home 한 점
  //   PICK: 받침 → 펼치기 → 퍽 위 40 열기 → 10 → 쥐기 −40 닫기 → 10 → 40 → 박스 위 기준점 (퍽 쥠, 바퀴 잠금 유지 — 이어서 STORE)
  //     손을 10° 넘게 기울여야 하면 (적재함에 못 넣는 먼 퍽) pick_tilted_depth_mm (30) = 퍽 무게중심을 쥠 — 더 깊으면 원기둥이 손가락 사이에서 굴러 넘어짐 (11-03 시뮬)
  //     LOOK 자세에서 받으면 그 자세에서 바로 퍽 위로. LOOK_BARCODE 자세면 검사된 복귀 경로(home_from_*)로 P0 까지 와서 퍽으로
  //     누운 네모 퍽: 수직 손만, 닫힘축 = 긴축에 수직 (손가락이 긴 옆면 두 개), 누운 윗면 위 40 · 10 · 쥐기 −20 (pick_lying_heights_mm).
  //       손목 검출로 누운 네모 (윗면이 위) 를 확인해야 함 — 누운 원기둥 · 기울여야 닿는 자리는 거절. 쥔 뒤 held_lying (STORE 거절)
  //   PLACE: 기준점(퍽 쥠) → 놓을 자리 위 40 → 10 → 놓기 열기 → 10 → 40 닫기 → P0 → 접기 → 받침 (주행 허용). lazy_fold 면 P0 (아래)
  //     놓기 높이: 놓는 순간 퍽의 가장 낮은 모서리가 놓을 면 위 place_bottom_clear_mm (drop_height_mm) — 수직이면 2 − 쥔 깊이:
  //     RETRIEVE 뒤 (retrieve_grip_depth_mm 25) −23, PICK 뒤 (40) −38. 다른 기울임으로 풀리면 그 기울임으로 높이를 다시 계산해 한 번 더 풂
  //     누운 퍽을 쥐었으면 누운 채로 수직 손으로 놓음: 놓기 높이는 누운 높이 기준 (2 − 20 = −18), 퍽 발자국 (60 mm) 도 이웃과 여유
  //     목표 = ArmTask.target (base_link) = planning 약속 (planning_interfaces.md 6 절): 그리퍼 중심이 갈 점 = 퍽 중심 + 쥘 끝 방향 × grasp_offset_m,
  //       방향 z 축 = 쥘 끝, x 축 = 네모 퍽 면 (누운 퍽: z = 로봇 쪽 긴축 끝 · x = 위, target_in_model). z = 0 이면 박스 위 기본 높이 (손 시험용)
  //   나중에 접기 (lazy_fold): STORE · PLACE 는 P0 (받침 바로 위) 에서 끝나 바퀴 잠금 유지.
  //     다음 PICK · RETRIEVE · LOOK 은 P0 에서 바로 (모두 검사된 경로의 일부), HOME 은 fold 경로로 접은 뒤 주행 허용
  // 빈 목록 = 거절 (why 에 이유)
  std::vector<ArmAction> build_plan(const ArmTask::Goal & goal, std::string & why)
  {
    using G = ArmTask::Goal;
    plan_ends_p0_ = false;
    // 속도는 작업마다 다시 읽음 (실행 중 ros2 param set 이 다음 작업부터)
    slow_dps_ = get_parameter("near_puck_speed_dps").as_double();
    path_slow_min_dps_ = get_parameter("path_slow_min_dps").as_double();
    side_grip_ = get_parameter("gripper_on_the_way").as_bool();
    gripper_time_s_ = get_parameter("gripper_time_s").as_double();
    timing_.grip_settle_s = std::max(0.0, get_parameter("grip_settle_s").as_double());
    {
      const double v = get_parameter("path_speed_dps").as_double(), vs = get_parameter("path_stand_speed_dps").as_double();
      const double ac = get_parameter("path_accel_dps2").as_double();
      if (v > 0.0 && v <= 250.0 && vs > 0.0 && vs <= 250.0 && ac > 0.0 && ac <= 1000.0) {   // 관절 속도 한계 4.8 rad/s = 275 °/s (URDF)
        timing_.speed_dps = v;
        timing_.stand_speed_dps = vs;
        timing_.accel_dps2 = ac;
      } else {
        RCLCPP_WARN(get_logger(), "path_speed_dps · path_stand_speed_dps · path_accel_dps2 가 이상함 (%.3g, %.3g, %.3g) → 그대로 (%.0f, %.0f, %.0f)", v, vs,
          ac, timing_.speed_dps, timing_.stand_speed_dps, timing_.accel_dps2);
      }
    }
    switch (goal.task) {
      case G::STORE:
        if (held_lying_) {why = "누운 퍽은 적재함에 못 넣음 (세우기 필요) — PLACE 로 내려놓기"; return {};}
        if (held_tilt_ > kStoreMaxTilt + 1e-6) {   // 적재함 경로는 수직 퍽으로 검증됨
          char buf[160];
          std::snprintf(buf, sizeof(buf), "손을 %.0f° 기울여 집은 퍽은 적재함에 못 넣음 (10° 까지) — 가까운 자리에 놓고 다시 집기", held_tilt_);
          why = buf;
          return {};
        }
        {
          // LOOK_RACK (작업 8) 뒤 관찰 자세에 있으면 store_look_<칸> (관찰 → 컵 입구 위 → 넣기), target = 그때 잰 칸 바닥 중심 (base_link)
          measured_cell_.reset();
          const auto & tp = goal.target.pose.position;
          if (goal.target.header.frame_id == "base_link" && std::isfinite(tp.x) && std::isfinite(tp.y) && (tp.x != 0.0 || tp.y != 0.0)) {
            measured_cell_ = Eigen::Vector2d(tp.x, tp.y);
          }
          std::string name = "store_" + goal.slot;
          const auto lk = paths_.find("store_look_" + goal.slot);
          std::string dummy;
          if (look_rack_slot_ == goal.slot && lk != paths_.end() && path_start_ok(lk->second, joint_buf_, path_start_tol_, dummy)) {
            name = "store_look_" + goal.slot;
            RCLCPP_INFO(get_logger(), "STORE %s: 관찰 자세에서 바로 칸으로 (%s)", goal.slot.c_str(), measured_cell_ ? "잰 칸 위치로" : "칸 측정 없음 → 칸 표");
          } else if (measured_cell_) {
            RCLCPP_INFO(get_logger(), "STORE %s: 칸 측정은 받았지만 관찰 자세가 아님 → 기록 경로 (store_%s)", goal.slot.c_str(), goal.slot.c_str());
          }
          auto plan = path_plan(name, why);
          measured_cell_.reset();
          return plan;
        }
      case kArmLookRack:   // 퍽 쥔 채 그 칸을 손목 카메라로 내려다봄 (look_rack_<칸>, 끝나면 그 자세 유지 → STORE)
        if (!carrying_) {why = "LOOK_RACK 은 퍽을 쥔 채만 (빈손이면 LOOK slot rack)"; return {};}
        if (held_lying_) {why = "누운 퍽은 적재함에 못 넣음 — LOOK_RACK 안 함"; return {};}
        if (held_tilt_ > kStoreMaxTilt + 1e-6) {why = "손을 기울여 쥔 퍽은 적재함에 못 넣음 — LOOK_RACK 안 함"; return {};}
        if (!paths_.count("look_rack_" + goal.slot)) {why = "LOOK_RACK " + goal.slot + ": 경로 없음 (config/rack_look_paths.yaml)"; return {};}
        return path_plan("look_rack_" + goal.slot, why);
      case G::RETRIEVE:
        return path_plan("retrieve_" + goal.slot, why, true);
      case G::LOOK:
        if (goal.slot == "rack") {
          if (carrying_) {why = "퍽을 쥔 채 적재함 스캔 안 함 (빈 손으로만)"; return {};}
          return path_plan("rack_scan", why, false, true);   // lazy_fold 여도 받침까지 (P0 에서 오래 버티지 않게)
        }
        return look_plan("look", why);
      case G::LOOK_BARCODE:
        return look_plan("look_barcode", why);
      case G::HOME:
        return home_plan(why, goal.slot == "force");
      case G::PICK:
        return pick_plan(goal, why);
      case G::PLACE:
        return place_plan(goal, why);
      default: {
        auto plan = make_arm_plan(goal.task, gripper_open_, gripper_close_);
        if (plan.empty()) {why = "모르는 작업 번호";}
        return plan;
      }
    }
  }

  // from_look: 받침에서 시작하는 경로를 LOOK · LOOK_BARCODE 자세에서 받으면 검사된 복귀 경로로 받침을 거쳐 감 (RETRIEVE)
  // keep_fold: lazy_fold 여도 끝의 접기를 안 자름 (받침까지)
  std::vector<ArmAction> path_plan(const std::string & name, std::string & why, bool from_look = false, bool keep_fold = false)
  {
    const auto it = paths_.find(name);
    if (it == paths_.end()) {why = "경로 " + name + " 없음 (config/arm_paths.yaml)"; return {};}
    ArmPath path = it->second;
    if (!rack_adjust(name, path, why)) {return {};}
    // STORE: 놓은 뒤 닫기는 팔을 멈추지 않고 (빈손으로 받침 쪽 가며) — 그리퍼 기다림 하나를 뺌
    if (side_grip_ && name.rfind("store_", 0) == 0) {   // 놓기 뒤 닫기
      bool released = false;
      for (auto & n : path.nodes) {
        if (n.grip == "release" || n.grip == "open") {released = true;}
        else if (released && n.grip == "close") {n.grip_on_way = true;}
      }
    }
    // 실물 기록의 느린 구간 (칸 근처 15 °/s) 을 path_slow_min_dps 까지 올림 (11-05 사용자: 적재가 너무 느림, 적당히 빠르게)
    for (auto & n : path.nodes) {
      if (n.speed_dps > 0.0 && n.speed_dps < path_slow_min_dps_) {n.speed_dps = path_slow_min_dps_;}
    }
    const auto speed_scale = get_parameter("path_speed_scale").as_string_array();   // 복사 (임시 Parameter 안을 가리키지 않게)
    // LOOK_RACK 의 store_look_<칸> · look_rack_<칸> 도 store_<칸> 값을 씀 (같은 칸, 같은 흔들림)
    std::string scale_name = name;
    for (const char * pre : {"store_look_", "look_rack_"}) {
      const std::string ps(pre);
      if (name.rfind(ps, 0) == 0) {scale_name = "store_" + name.substr(ps.size());}
    }
    for (const auto & e : speed_scale) {
      const auto eq = e.find('=');
      if (eq == std::string::npos || e.compare(0, eq, scale_name) != 0 || eq != scale_name.size()) {continue;}
      const double sc = std::atof(e.c_str() + eq + 1);
      if (!(sc > 0.0)) {continue;}
      for (auto & n : path.nodes) {
        const double v0 = n.stand ? timing_.stand_speed_dps : timing_.speed_dps;
        n.speed_dps = sc * (n.speed_dps > 0.0 ? std::min(v0, n.speed_dps) : v0);
      }
      RCLCPP_INFO(get_logger(), "%s: 속도 %.0f %% (path_speed_scale)", name.c_str(), sc * 100.0);
    }
    // PICK 이 집은 자리에서 끝났으면 (박스 위 기준점 안 감) 칸 경로 첫 점 (공통 기준점) 대신 지금 자세에서 실제 기록 시작점 (lift_real) 으로
    if (name.rfind("store_", 0) == 0 || name.rfind("look_rack_", 0) == 0) {join_from_pick_end(path, name.c_str());}
    if (!keep_fold && cut_fold_tail(path)) {
      plan_ends_p0_ = true;
      RCLCPP_INFO(get_logger(), "%s: P0 에서 멈춤 (다음 작업이 %.0f s 안에 안 오면 접음, 주행 전 HOME 이 접음)", name.c_str(), auto_fold_s_);
    }
    if (!path_start_ok(path, joint_buf_, path_start_tol_, why)) {
      if (!join_from_p0(path, name.c_str()) && (!from_look || !join_from_look(path, name.c_str()))) {
        if (from_look) {why += " — 받침(HOME) 또는 LOOK 자세에서 시작해야 함";}
        return {};
      }
      why.clear();
    }
    return path_actions(path, timing_, grip_, joint_buf_);
  }

  // PICK 이 성공해 퍽을 쥔 채 집은 자리 (PICK 마지막 점) 에 있으면 경로 첫 점 (박스 위 기준점) 을 지금 자세로 바꿈 → 다음 점으로 바로
  //   (기준점은 모든 칸 경로 · PLACE 가 같이 쓰려고 붙인 점, 실제 기록은 다음 점부터). 이미 기준점이면 그대로
  void join_from_pick_end(ArmPath & path, const char * what)
  {
    std::string dummy;
    if (!held_from_pick_ || arm_off_path_ || path.nodes.size() < 2 || joint_buf_.size() != path.nodes.front().q.size() ||
      path_start_ok(path, joint_buf_, path_start_tol_, dummy))
    {
      return;
    }
    for (double v : joint_buf_) {if (!std::isfinite(v)) {return;}}
    path.nodes.front().q = joint_buf_;
    path.nodes.front().name = "pick_end";
    RCLCPP_INFO(get_logger(), "%s: 집은 자리에서 바로 (박스 위 기준점 안 거침) → %s", what, path.nodes[1].name.c_str());
  }

  // lazy_fold 면: 끝이 fold 경로면 P0 에서 끝냄 (주행 허용 없이 끝남)
  bool cut_fold_tail(ArmPath & path) const
  {
    const auto fold = paths_.find("fold");
    return lazy_fold_ && fold != paths_.end() && manipulation::cut_fold_tail(path, fold->second);
  }

  // lazy_fold 면: 앞이 unfold 경로면 P0 에서 시작
  bool skip_unfold(ArmPath & path) const
  {
    const auto unfold = paths_.find("unfold");
    return lazy_fold_ && unfold != paths_.end() && manipulation::skip_unfold(path, unfold->second);
  }

  // 지금 P0 면 받침 출발 경로를 P0 에서 시작
  bool join_from_p0(ArmPath & path, const char * what)
  {
    ArmPath p = path;
    std::string dummy;
    if (!skip_unfold(p) || !path_start_ok(p, joint_buf_, path_start_tol_, dummy)) {return false;}
    path = p;
    RCLCPP_INFO(get_logger(), "%s: P0 에서 바로 (나중에 접기, 펼치기 생략)", what);
    return true;
  }

  // 지금 LOOK 끝 자세 (박스를 내려다봄) 면 받침 출발 경로의 펼치기를 빼고 그 자세에서 바로 다음 점으로 (PICK: 관찰 자세 → 퍽 위 40 mm)
  //   관찰 자세 → 박스 위 퍽 자리 (박스가 차체에 붙음 · 3 cm 띄움 각각 앞뒤 3 줄 × 좌우 3 칸, 닿는 곳) 위 40 mm 는
  //   차체 · 적재함 · 박스 · 퍽과 22 mm 넘게 떨어짐 (pose_safety)
  bool start_at_look(ArmPath & path, const char * what)
  {
    const auto look = paths_.find("look");
    const auto unfold = paths_.find("unfold");
    if (look == paths_.end() || look->second.nodes.empty() || unfold == paths_.end()) {return false;}
    ArmPath p = path;
    if (!manipulation::skip_unfold(p, unfold->second)) {return false;}   // 첫 점 = P0
    p.nodes.front() = look->second.nodes.back();
    std::string dummy;
    if (!path_start_ok(p, joint_buf_, path_start_tol_, dummy)) {return false;}
    path = p;
    RCLCPP_INFO(get_logger(), "%s: 관찰 자세에서 바로 퍽 위로 (P0 · 받침 안 거침)", what);
    return true;
  }

  // 지금 LOOK · LOOK_BARCODE 끝 자세면 그 복귀 경로(home_from_*) 뒤에 path 를 이어 붙임 (path 첫 점 = 받침 = 복귀 경로 끝)
  //   복귀 경로 끝이 fold, path 앞이 unfold 면 받침까지 접었다 펴지 않고 P0 에서 이음 (lazy_fold 와 상관없이 — 바퀴는 계속 잠금)
  bool join_from_look(ArmPath & path, const char * what)
  {
    const auto fold = paths_.find("fold");
    const auto unfold = paths_.find("unfold");
    for (const char * name : {"home_from_look", "home_from_look_barcode"}) {
      const auto back = paths_.find(name);
      std::string dummy;
      if (back == paths_.end() || !path_start_ok(back->second, joint_buf_, path_start_tol_, dummy)) {continue;}
      ArmPath to_p0 = back->second;
      ArmPath rest = path;
      if (p0_join_ok_ && manipulation::cut_fold_tail(to_p0, fold->second) && manipulation::skip_unfold(rest, unfold->second)) {
        to_p0.nodes.insert(to_p0.nodes.end(), rest.nodes.begin() + 1, rest.nodes.end());
        path.nodes = to_p0.nodes;
        RCLCPP_INFO(get_logger(), "%s: 관찰 자세 → P0 에서 바로 (받침까지 접지 않음)", what);
        return true;
      }
      std::vector<PathNode> nodes = back->second.nodes;
      nodes.insert(nodes.end(), path.nodes.begin() + 1, path.nodes.end());
      path.nodes = nodes;
      RCLCPP_INFO(get_logger(), "%s: 관찰 자세 → %s 로 받침을 거쳐 감", what, name);
      return true;
    }
    return false;
  }

  std::vector<ArmAction> look_plan(const std::string & name, std::string & why)
  {
    const auto it = paths_.find(name);
    if (it == paths_.end()) {why = "경로 " + name + " 없음 (config/arm_paths.yaml)"; return {};}
    ArmPath at_end{name, {it->second.nodes.back()}, false};
    std::string dummy;
    if (path_start_ok(at_end, joint_buf_, path_start_tol_, dummy)) {   // 이미 그 자세 → 그 점으로 한 번 (작은 보정)
      ArmAction a{"approach", false, at_end.nodes[0].name, 0.0};
      a.points = {{at_end.nodes[0].q, 0.5}};
      a.duration_s = 0.5;
      return {a};
    }
    auto plan = path_plan(name, why);
    if (plan.empty()) {why += " — 받침(HOME)에서 시작해야 함";}
    return plan;
  }

  // HOME: 접혀 있으면 home 한 점. 아니면 ① 지금 자세에서 출발하는 검사된 복귀 경로(home_from_*) ② 받침 근처면 home 한 점
  //   ③ 그 밖에 home 한 점 직행 (켤 때 등). 단 퍽을 쥐었거나 · 경로 작업이 도중에 멈춘 뒤면 거절 (칸 · 박스 안에서 직선으로 쓸고 나올 수 있음 → 사람 복구)
  //   force (slot "force"): 사람이 팔 주변을 보고 안전하다고 판단했을 때만 — 위 거절을 건너뜀 (시험 중 도중 정지 복구용)
  std::vector<ArmAction> home_plan(std::string & why, bool force = false)
  {
    home_was_path_ = false;
    if (!fold_->confirmed(now().seconds())) {
      if (carrying_ && !force) {why = "퍽을 쥔 채 HOME 안 함 — STORE · PLACE 로 끝내거나 사람이 퍽을 빼고 HOME force"; return {};}
      for (const auto & [name, path] : paths_) {
        std::string dummy;
        const bool home_path = name.rfind("home_from_", 0) == 0 || name == "fold";   // fold = P0 → 받침
        if (home_path && path.ends_home && path_start_ok(path, joint_buf_, path_start_tol_, dummy)) {
          RCLCPP_INFO(get_logger(), "HOME: 경로 %s", name.c_str());
          home_was_path_ = true;
          return path_actions(path, timing_, grip_, joint_buf_);
        }
      }
      const auto fold = paths_.find("fold");
      std::string dummy;
      const bool near_home = fold != paths_.end() &&
        path_start_ok(ArmPath{"home", {fold->second.nodes.back()}, true}, joint_buf_, home_near_tol_, dummy);
      if (!near_home && force) {
        RCLCPP_WARN(get_logger(), "HOME force: 퍽 쥠 · 도중 정지 검사를 건너뛰고 home 한 점으로 곧장 (사람 판단)");
      } else if (!near_home) {
        const bool half_closed = std::isfinite(grip_now_) && grip_now_ > gripper_close_ + 0.003 && grip_now_ < gripper_open_ - 0.004;
        const bool holding = carrying_ || half_closed;
        if (holding) {why = "퍽을 쥔 채(그리퍼 반쯤 닫힘) 받침으로 곧장 갈 수 없음 — STORE · PLACE 로 끝내거나 사람이 퍽을 빼고 HOME"; return {};}
        if (arm_off_path_) {why = "경로 작업이 도중에 멈춘 자세 — 검사 안 된 직선 복귀 안 함. 팔을 받침 근처로 옮긴 뒤 HOME"; return {};}
        RCLCPP_WARN(get_logger(), "HOME: 지금 자세에서 출발하는 검사된 복귀 경로 없음 → home 한 점으로 곧장");
      }
    } else if (poses_.count("home")) {   // 이미 접힘 (자동 접기 뒤 주행 전 HOME 등) → 0.3 s 한 점 + 접힘 실측 확인만 (4 s 안 씀)
      auto plan = make_arm_plan(kArmHome, gripper_open_, gripper_close_);
      plan.front().points = {{poses_.at("home"), 0.3}};
      plan.front().duration_s = 0.3;
      return plan;
    }
    return make_arm_plan(kArmHome, gripper_open_, gripper_close_);
  }

  // 목표(그리퍼 중심이 갈 점, planning 약속) → 모델 좌표 (x, y, 퍽 윗면 z, yaw [°]) + 누운 목표인지
  //   서 있는 퍽 (z 축 = 쥘 끝이 수직에서 30° 안): 그리퍼 중심 = 퍽 중심 + grasp_offset (위) → 윗면 = 목표 z − grasp_offset + 퍽 반높이,
  //     yaw = 면 방향 (x 축)
  //   누운 퍽 (z 축이 수평에서 30° 안, planning: z = 로봇 쪽 긴축 끝 · x = 위): 퍽 중심 = 목표 − z · grasp_offset →
  //     (x, y) = 퍽 중심, 윗면 = 중심 + 누운 높이/2, yaw = 긴축 방향. 그 사이 (비스듬) 는 거절
  //   as_lying (누운 퍽을 쥔 PLACE): 서 있는 목표면 그 놓을 면 (목표 z − grasp_offset − 퍽 반높이) 위 누운 높이를 윗면으로, yaw 모름
  //   z = 0 이면 박스 위 기본 높이 (손 시험용)
  bool target_in_model(const ArmTask::Goal & goal, bool as_lying, double & x, double & y, double & top, double & yaw, bool & lying,
    std::string & why) const
  {
    const auto & p = goal.target.pose.position;
    const auto & o = goal.target.pose.orientation;
    const auto & frame = goal.target.header.frame_id;
    if (!frame.empty() && frame != "base_link") {why = "target 프레임 " + frame + " (base_link 만 받음)"; return false;}
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {why = "target 위치에 NaN"; return false;}
    const double n = o.x * o.x + o.y * o.y + o.z * o.z + o.w * o.w;
    Eigen::Vector3d z = Eigen::Vector3d::UnitZ();   // 쥘 끝 방향
    lying = false;
    if (n > 0.5) {
      z = Eigen::Quaterniond(o.w, o.x, o.y, o.z).normalized() * Eigen::Vector3d::UnitZ();
      lying = std::fabs(z.z()) <= std::sin(30.0 * M_PI / 180.0);
      if (!lying && z.z() < std::cos(30.0 * M_PI / 180.0)) {why = "비스듬한 퍽 (쥘 끝이 수직 · 수평에서 30° 넘게 벗어남) — 안 함"; return false;}
    }
    double top_team;
    if (lying) {
      const Eigen::Vector3d c = Eigen::Vector3d(p.x, p.y, p.z) - z * grasp_offset_;
      x = c.x();
      y = c.y();
      top_team = p.z != 0.0 ? c.z() + lying_h_ / 2.0 : surface_z_ + lying_h_;
      yaw = std::atan2(z.y(), z.x()) * 180.0 / M_PI;
    } else {
      x = p.x;
      y = p.y;
      const double surface = p.z != 0.0 ? p.z - grasp_offset_ - puck_h_ / 2.0 : surface_z_;
      top_team = surface + (as_lying ? lying_h_ : puck_h_);
      yaw = n > 0.5 && !as_lying ? std::atan2(2.0 * (o.w * o.z + o.x * o.y), 1.0 - 2.0 * (o.y * o.y + o.z * o.z)) * 180.0 / M_PI : std::nan("");
    }
    top = top_team + model_.team_base_z_floor - model_.base_z_floor;
    return true;
  }

  PathNode model_node(const std::string & name, const Ticks & enc, double speed, const std::string & grip = "") const
  {
    return {name, ticks_to_command(model_, enc), false, speed, grip, false};
  }

  // 손목 인지 결과 → odom 기준 퍽 목록 (촬영 시각 TF, 안 되면 최신 TF). 쓸 때 base_link 로 (그 사이 차체가 움직여도 맞게)
  // ---- 적재함 태그 → 칸 경로 옮기기 ----
  // 스캔 (LOOK rack) 중에만: 멈춘 장 (confidence 1) 칸 태그 중심 [base_link]
  void on_rack_tags(const interfaces::msg::DetectionArray & m)
  {
    if (!scanning_) {return;}
    for (const auto & d : m.detections) {
      const auto & p = d.pose.position;
      if (d.kind != "rack_tag" || d.confidence < 0.99f || !std::isfinite(p.x) || !std::isfinite(p.y)) {continue;}
      scan_seen_[d.label].emplace_back(p.x, p.y);
    }
  }

  // 스캔이 끝나면 칸마다 중앙값 → 기준이 있으면 적재함 전체 움직임을 맞춤
  void finish_rack_scan()
  {
    last_scan_.clear();
    for (const auto & [name, v] : scan_seen_) {
      if (static_cast<int>(v.size()) < rack_min_samples_) {continue;}
      std::vector<double> xs, ys;
      for (const auto & p : v) {xs.push_back(p.x()); ys.push_back(p.y());}
      last_scan_[name] = Eigen::Vector2d(median(xs), median(ys));
    }
    scan_seen_.clear();
    rack_fit_ok_ = false;
    if (rack_ref_.empty()) {
      RCLCPP_WARN(get_logger(), "적재함 스캔: 칸 %zu 개 잼. 기준 스캔 없음 → 칸 경로 그대로. 지금 적재함이 제자리면 "
        "ros2 service call /control_master/save_rack_reference std_srvs/srv/Trigger", last_scan_.size());
      return;
    }
    std::vector<Eigen::Vector2d> ref, cur;
    for (const auto & [name, p] : last_scan_) {
      const auto r = rack_ref_.find(name);
      if (r != rack_ref_.end()) {ref.push_back(r->second); cur.push_back(p);}
    }
    if (ref.size() < 3) {
      RCLCPP_WARN(get_logger(), "적재함 스캔: 기준과 같이 잰 칸 %zu 개 (3 개 넘어야) → 칸 경로 그대로", ref.size());
      return;
    }
    rack_fit_ = fit_rack(ref, cur, rack_pivot_);
    rack_fit_ok_ = rack_fit_.ok && rack_fit_.rotation && rack_fit_.rms <= rack_max_rms_m_;
    RCLCPP_INFO(get_logger(), "적재함 움직임 (기준 스캔 대비, 칸 %d 개): dx %+.1f · dy %+.1f mm · 회전 %+.2f° · 남는 오차 %.1f mm → %s",
      rack_fit_.n, rack_fit_.dx * 1000.0, rack_fit_.dy * 1000.0, rack_fit_.dyaw * 180.0 / M_PI, rack_fit_.rms * 1000.0,
      rack_fit_ok_ ? "STORE · RETRIEVE 칸 경로를 옮김" : "맞춤이 나빠 안 씀 (칸 경로 그대로, 다시 스캔)");
  }

  // 마지막 스캔을 기준으로 저장 (적재함이 제자리 = 지금 경로로 넣기 · 빼기가 되는 때)
  void on_held_tag(const interfaces::msg::DetectionArray & m)
  {
    const double t = rclcpp::Time(m.header.stamp, get_clock()->get_clock_type()).seconds();
    if (!carrying_ || t < held_since_) {return;}
    for (const auto & d : m.detections) {
      const auto & p = d.pose.position;
      const auto & o = d.pose.orientation;
      const Eigen::Vector3d n = Eigen::Quaterniond(o.w, o.x, o.y, o.z).normalized() * Eigen::Vector3d::UnitZ();
      held_buf_.push_back({d.id, Eigen::Vector3d(p.x, p.y, p.z), n});
    }
    while (held_buf_.size() > 100) {held_buf_.pop_front();}
  }

  // 이번에 쥔 퍽의 손 안 태그 (가장 많이 본 ID 의 중앙값, 손목 좌표 m). 3 장 안 되면 false
  bool held_now(std::string & id, Eigen::Vector3d & p, Eigen::Vector3d & n, int & count) const
  {
    std::map<std::string, int> cnt;
    for (const auto & h : held_buf_) {++cnt[h.id];}
    count = 0;
    for (const auto & [k, c] : cnt) {if (c > count) {count = c; id = k;}}
    if (count < 3) {return false;}
    std::array<std::vector<double>, 6> v;
    for (const auto & h : held_buf_) {
      if (h.id != id) {continue;}
      for (int i = 0; i < 3; ++i) {v[i].push_back(h.p[i]); v[3 + i].push_back(h.n[i]);}
    }
    auto med = [](std::vector<double> x) {std::nth_element(x.begin(), x.begin() + x.size() / 2, x.end()); return x[x.size() / 2];};
    p = Eigen::Vector3d(med(v[0]), med(v[1]), med(v[2]));
    n = Eigen::Vector3d(med(v[3]), med(v[4]), med(v[5])).normalized();
    return true;
  }

  std::string held_ref_file() const   // 실행 중 바꿔도 다음 저장부터
  {
    std::string f = get_parameter("held_reference_file").as_string();
    if (f.rfind("~/", 0) == 0 && std::getenv("HOME")) {f = std::getenv("HOME") + f.substr(1);}
    return f;
  }

  bool save_held_reference(std::string & msg)
  {
    const std::string held_ref_file_ = held_ref_file();
    std::string id;
    Eigen::Vector3d p, n;
    int count = 0;
    if (!held_now(id, p, n, count)) {
      msg = "쥔 퍽 태그를 3 장 넘게 못 봄 (PICK 뒤 기준점에서, rack_tag_node 켜짐?) — 지금 " + std::to_string(count) + " 장";
      return false;
    }
    std::error_code ec;
    std::filesystem::create_directories(std::filesystem::path(held_ref_file_).parent_path(), ec);
    std::ofstream f(held_ref_file_);
    f << "# 쥔 퍽 기준 (control_master ~/save_held_reference): 태그 중심 x y z · 면 방향 x y z [m, wrist_link]\n";
    f.precision(6);
    f << std::fixed << p.x() << " " << p.y() << " " << p.z() << " " << n.x() << " " << n.y() << " " << n.z() << "\n";
    f.close();
    if (!f) {msg = "기준 파일을 못 씀: " + held_ref_file_; return false;}
    held_ref_p_ = p;
    held_ref_n_ = n;
    char buf[200];
    std::snprintf(buf, sizeof(buf), "쥔 퍽 기준 저장 (태그 %s, %d 장): 손목 (%.1f, %.1f, %.1f) mm → %s", id.c_str(), count, p.x() * 1000.0,
      p.y() * 1000.0, p.z() * 1000.0, held_ref_file_.c_str());
    msg = buf;
    RCLCPP_INFO(get_logger(), "%s", msg.c_str());
    return true;
  }

  void load_held_reference()
  {
    const std::string held_ref_file_ = held_ref_file();
    std::ifstream f(held_ref_file_);
    std::string line;
    while (f && std::getline(f, line)) {
      if (line.empty() || line[0] == '#') {continue;}
      std::istringstream in(line);
      double v[6];
      if (in >> v[0] >> v[1] >> v[2] >> v[3] >> v[4] >> v[5]) {
        held_ref_p_ = Eigen::Vector3d(v[0], v[1], v[2]);
        held_ref_n_ = Eigen::Vector3d(v[3], v[4], v[5]).normalized();
      }
    }
    if (held_ref_p_) {
      RCLCPP_INFO(get_logger(), "쥔 퍽 기준: 손목 (%.1f, %.1f, %.1f) mm (%s)", held_ref_p_->x() * 1000.0, held_ref_p_->y() * 1000.0,
        held_ref_p_->z() * 1000.0, held_ref_file_.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "쥔 퍽 기준 없음 (%s) → STORE 때 손 안 위치는 로그만 (save_held_reference 로 저장)", held_ref_file_.c_str());
    }
  }

  bool save_rack_reference(std::string & msg)
  {
    if (last_scan_.size() < 6) {
      msg = "칸 6 개 넘게 잰 스캔이 없음 (빈 손으로 scan 먼저, 지금 " + std::to_string(last_scan_.size()) + " 칸)";
      return false;
    }
    std::error_code ec;
    std::filesystem::create_directories(std::filesystem::path(rack_ref_file_).parent_path(), ec);
    std::ofstream f(rack_ref_file_);
    f << "# 적재함 기준 스캔 (control_master ~/save_rack_reference): 칸 · 칸 바닥 태그 중심 x y [m, base_link]\n";
    f.precision(6);
    for (const auto & [name, p] : last_scan_) {f << name << " " << std::fixed << p.x() << " " << p.y() << "\n";}
    f.close();
    if (!f) {msg = "기준 파일을 못 씀: " + rack_ref_file_; return false;}
    rack_ref_ = last_scan_;
    rack_fit_ = RackFit{};
    rack_fit_.ok = rack_fit_.rotation = true;   // 기준 = 지금 → 움직임 0
    rack_fit_ok_ = true;
    msg = "기준 저장: 칸 " + std::to_string(rack_ref_.size()) + " 개 → " + rack_ref_file_;
    RCLCPP_INFO(get_logger(), "%s", msg.c_str());
    return true;
  }

  void load_rack_reference()
  {
    std::ifstream f(rack_ref_file_);
    if (!f) {
      RCLCPP_INFO(get_logger(), "적재함 기준 스캔 없음 (%s) → 스캔해도 칸 경로 그대로 (save_rack_reference 로 저장)", rack_ref_file_.c_str());
      return;
    }
    std::string line;
    while (std::getline(f, line)) {
      if (line.empty() || line[0] == '#') {continue;}
      std::istringstream in(line);
      std::string name;
      double x, y;
      if (in >> name >> x >> y && std::isfinite(x) && std::isfinite(y)) {rack_ref_[name] = Eigen::Vector2d(x, y);}
    }
    RCLCPP_INFO(get_logger(), "적재함 기준 스캔: 칸 %zu 개 (%s). 스캔하면 이것과 비교해 칸 경로를 옮김", rack_ref_.size(), rack_ref_file_.c_str());
  }

  // 쥐는 자세의 손목 돌림 [°] (닫힘축이 접선에서 얼마나 돌았나, 손가락 방향 포함 ±180) 과 그 돌림 보정 d [m, base_link]
  bool rot_offset(double x, double y, const Ticks & t, Eigen::Vector2d & d, double & rot_deg, std::string & why) const
  {
    d.setZero();
    rot_deg = 0.0;
    const auto r = get_parameter("pick_offset_rot_mm").as_double_array();
    if (r.size() != 2 || !std::isfinite(r[0]) || !std::isfinite(r[1])) {
      why = "pick_offset_rot_mm 는 [앞, 왼쪽] mm 두 개";
      return false;
    }
    const Eigen::Vector3d c = arm_grip(model_, ticks_to_q(model_, t)).close;
    const double az = std::atan2(y - model_.j1_xy.y(), x - model_.j1_xy.x());
    const double a = std::remainder(std::atan2(c.y(), c.x()) - az - M_PI / 2.0, 2.0 * M_PI);   // 기본 = 닫힘축 +접선
    rot_deg = a * 180.0 / M_PI;
    const auto rr = get_parameter("pick_offset_rot_robot_mm").as_double_array();
    if (rr.size() != 2 || !std::isfinite(rr[0]) || !std::isfinite(rr[1])) {
      why = "pick_offset_rot_robot_mm 는 [앞, 왼쪽] mm 두 개 (로봇 기준)";
      return false;
    }
    if (std::fabs(rot_deg) >= 45.0) {d = Eigen::Vector2d(rr[0], rr[1]) / 1000.0;}
    if (r[0] == 0.0 && r[1] == 0.0) {return true;}
    // 90° 돌렸을 때 보정 r (팔 방향 기준, 기준 돌림 = 실물에서 본 −90°) = (R(−90°) − I) h → h, 이 돌림의 보정 = (R(a) − I) h
    auto rot2 = [](double t) {Eigen::Matrix2d R; R << std::cos(t), -std::sin(t), std::sin(t), std::cos(t); return R;};
    const Eigen::Matrix2d I = Eigen::Matrix2d::Identity();
    const Eigen::Vector2d h = (rot2(-M_PI / 2.0) - I).inverse() * Eigen::Vector2d(r[0], r[1]);
    d += rot2(az) * ((rot2(a) - I) * h) / 1000.0;
    return true;
  }

  // 경로의 첫 놓는 점 (grip release) 의 손목 (wrist_link) 회전 [모델 base = base_link 와 방향 같음]
  bool release_rot(const std::string & path_name, Eigen::Matrix3d & R) const
  {
    const auto it = paths_.find(path_name);
    if (it == paths_.end() || !model_ok_) {return false;}
    for (const auto & n : it->second.nodes) {
      if (n.grip != "release" || n.q.size() != 5) {continue;}
      Ticks t;
      for (int j = 0; j < 5; ++j) {t[j] = static_cast<int>(std::lround(model_.zero_tick[j] + n.q[j] * 4096.0 / (2.0 * M_PI)));}
      R = arm_frames(model_, ticks_to_q(model_, t))[4].block<3, 3>(0, 0);
      return true;
    }
    return false;
  }

  // 경로의 첫 놓는 점 (grip release) 손끝 [m, 모델 base = base_link 와 x · y 같음]
  bool release_tip(const std::string & path_name, Eigen::Vector2d & tip) const
  {
    const auto it = paths_.find(path_name);
    if (it == paths_.end() || !model_ok_) {return false;}
    for (const auto & n : it->second.nodes) {
      if (n.grip != "release" || n.q.size() != 5) {continue;}
      Ticks t;
      for (int j = 0; j < 5; ++j) {t[j] = static_cast<int>(std::lround(model_.zero_tick[j] + n.q[j] * 4096.0 / (2.0 * M_PI)));}
      tip = arm_grip(model_, ticks_to_q(model_, t)).tip.head<2>();
      return true;
    }
    return false;
  }

  // STORE · RETRIEVE: 마지막 스캔이 기준에서 움직였으면 그 칸 경로의 칸 근처를 옮김 + STORE 는 store_offset_*_mm 도 더함. false = 거절 (why)
  bool rack_adjust(const std::string & name, ArmPath & path, std::string & why)
  {
    const bool store = name.rfind("store_", 0) == 0;
    if (!store && name.rfind("retrieve_", 0) != 0) {return true;}
    const std::string cell = name.substr(name.rfind('_') + 1);   // store_S7 · retrieve_S7 · store_look_S7
    size_t i = 0;
    while (i < rack_names_.size() && rack_names_[i] != cell) {++i;}
    const bool known = i < rack_names_.size();
    char buf[240];
    Eigen::Vector2d d_rack = Eigen::Vector2d::Zero();
    if (store && known && measured_cell_) {   // LOOK_RACK 으로 잰 칸 바닥 중심 (STORE target) − 칸 표 − 카메라 치우침 = 그 칸이 움직인 양 (빈손 스캔 맞춤 대신)
      const auto bias = get_parameter("look_rack_bias_mm").as_double_array();   // 적재함이 제자리일 때 잰 값 − 칸 표 (실물 첫 시험 로그로)
      const Eigen::Vector2d b = bias.size() == 2 ? Eigen::Vector2d(bias[0] / 1000.0, bias[1] / 1000.0) : Eigen::Vector2d(0.0, 0.0);
      d_rack = *measured_cell_ - rack_xy_[i] - b;
      if (d_rack.norm() > rack_max_shift_m_) {   // 다른 칸 태그 · 카메라 치우침 (look_rack_bias_mm 미설정) → 거절하지 않고 칸 표대로 (기록 경로는 실물에서 들어감)
        RCLCPP_WARN(get_logger(), "%s: LOOK_RACK 로 잰 칸이 칸 표에서 (%+.1f, %+.1f) mm (한도 %.0f mm) → 측정 안 쓰고 칸 표대로. 늘 이만큼이면 look_rack_bias_mm",
          name.c_str(), d_rack.x() * 1000.0, d_rack.y() * 1000.0, rack_max_shift_m_ * 1000.0);
        d_rack.setZero();
      }
      RCLCPP_INFO(get_logger(), "%s: LOOK_RACK 로 잰 칸 (%.1f, %.1f) mm · 칸 표 대비 (%+.1f, %+.1f) mm", name.c_str(), measured_cell_->x() * 1000.0,
        measured_cell_->y() * 1000.0, d_rack.x() * 1000.0, d_rack.y() * 1000.0);
      if (d_rack.norm() < rack_min_shift_m_) {d_rack.setZero();}
    } else if (rack_fit_ok_ && known) {
      d_rack = rack_shift(rack_fit_, rack_xy_[i], rack_pivot_);
      if (d_rack.norm() > rack_max_shift_m_) {
        std::snprintf(buf, sizeof(buf), "적재함이 칸 %s 에서 %.1f mm 움직임 (기준 스캔 대비, 한도 %.0f mm) — 적재함을 제자리로 놓고 다시 scan",
          cell.c_str(), d_rack.norm() * 1000.0, rack_max_shift_m_ * 1000.0);
        why = buf;
        return false;
      }
      if (d_rack.norm() < rack_min_shift_m_) {d_rack.setZero();}
    }
    Eigen::Vector2d d_set = Eigen::Vector2d::Zero();
    double dz = 0.0;   // 칸 근처 높이 [m] (store_offset.<칸> 세 번째 값, − = 더 내려 놓기)
    if (store) {
      double ox = get_parameter("store_offset_x_mm").as_double(), oy = get_parameter("store_offset_y_mm").as_double();
      if (known) {
        const auto c = get_parameter("store_offset." + cell).as_double_array();
        if (c.size() != 2 && c.size() != 3) {why = "store_offset." + cell + " 는 [x, y] 또는 [x, y, 높이] mm"; return false;}
        ox += c[0];
        oy += c[1];
        if (c.size() == 3 && std::isfinite(c[2])) {dz = c[2] / 1000.0;}
      }
      // 쥔 깊이가 박스 PICK (pick_heights_mm 마지막, 칸 보정을 맞춘 깊이) 과 다르면 칸 근처 높이를 그 차만큼 → 퍽 바닥이 같은 높이로
      //   (11-07 실물: 받침에서 세워 30 mm 로 다시 쥔 퍽이 박스 PICK 50 mm 보다 20 mm 아래 매달려 S2 에 긁혀 안 들어감)
      if (get_parameter("store_match_grip_depth").as_bool() && depth_from_pick_ && held_depth_mm_ > 0.0 && !pick_heights_.empty()) {
        const double ref = -pick_heights_.back();
        const double ddz = (ref - held_depth_mm_) / 1000.0;
        if (std::fabs(ddz) > 1e-4) {
          RCLCPP_INFO(get_logger(), "%s: 쥔 깊이 %.0f mm (박스 PICK %.0f mm) → 칸 근처 높이 %+.1f mm (store_match_grip_depth)", name.c_str(),
            held_depth_mm_, ref, ddz * 1000.0);
          dz += ddz;
        }
      }
      if (known && get_parameter("store_to_cell").as_bool()) {
        Eigen::Vector2d tip;
        if (!release_tip(name, tip)) {why = "넣는 점 맞추기: " + name + " 에 놓는 점 (release) 이 없거나 팔 모델 없음"; return false;}
        const Eigen::Vector2d a = (rack_xy_[i] + rack_place_[i] - tip) * 1000.0;
        RCLCPP_INFO(get_logger(), "%s: 넣는 점 맞추기 — 놓는 점 (%.1f, %.1f) → 칸 (%.1f, %.1f) mm = (%+.1f, %+.1f)", name.c_str(),
          tip.x() * 1000.0, tip.y() * 1000.0, (rack_xy_[i] + rack_place_[i]).x() * 1000.0, (rack_xy_[i] + rack_place_[i]).y() * 1000.0,
          a.x(), a.y());
        ox += a.x();
        oy += a.y();
      }
      {
        std::string hid;
        Eigen::Vector3d hp, hn;
        int hc = 0;
        if (!held_now(hid, hp, hn, hc)) {
          RCLCPP_INFO(get_logger(), "%s: 쥔 퍽 태그 못 봄 (%d 장) → 손 안 위치 보정 없음", name.c_str(), hc);
        } else if (!held_ref_p_) {
          RCLCPP_INFO(get_logger(), "%s: 쥔 퍽 태그 %s 손목 (%.1f, %.1f, %.1f) mm (%d 장) · 기준 없음", name.c_str(), hid.c_str(),
            hp.x() * 1000.0, hp.y() * 1000.0, hp.z() * 1000.0, hc);
        } else {
          const Eigen::Vector3d dw = hp - *held_ref_p_;   // 손 안에서 퍽이 기준보다 움직인 양 (손목 좌표)
          const double face = std::acos(std::clamp(hn.dot(*held_ref_n_), -1.0, 1.0)) * 180.0 / M_PI;
          Eigen::Matrix3d Rr;
          const bool rot_ok = release_rot(name, Rr);
          const Eigen::Vector3d dworld = rot_ok ? Eigen::Vector3d(Rr * dw) : Eigen::Vector3d::Zero();   // 놓는 자세에서 퍽이 비키는 양
          const bool use = get_parameter("held_correct").as_bool() && rot_ok && face < 20.0 && dw.norm() * 1000.0 <= held_max_mm_;
          RCLCPP_INFO(get_logger(), "%s: 쥔 퍽 태그 %s 기준과 차 손목 (%+.1f, %+.1f, %+.1f) mm · 면 %.0f° (%d 장) → 놓는 자리에서 (%+.1f, %+.1f) mm %s",
            name.c_str(), hid.c_str(), dw.x() * 1000.0, dw.y() * 1000.0, dw.z() * 1000.0, face, hc, dworld.x() * 1000.0,
            dworld.y() * 1000.0, use ? "→ 반대로 옮김" : (get_parameter("held_correct").as_bool() ? "(면이 다르거나 너무 커서 안 옮김)" : "(held_correct 꺼짐, 로그만)"));
          if (use) {
            ox -= dworld.x() * 1000.0;
            oy -= dworld.y() * 1000.0;
          }
        }
      }
      if (!std::isfinite(ox) || !std::isfinite(oy)) {
        why = "store_offset (넣는 점 맞추기 + 전체 + 칸 " + cell + ") 가 숫자가 아님";
        return false;
      }
      d_set = Eigen::Vector2d(ox, oy) / 1000.0;
    } else if (known) {
      const auto c = get_parameter("retrieve_offset." + cell).as_double_array();
      if (c.size() != 2 && c.size() != 3) {why = "retrieve_offset." + cell + " 는 [x, y] 또는 [x, y, 높이] mm"; return false;}
      if (!std::isfinite(c[0]) || !std::isfinite(c[1])) {why = "retrieve_offset." + cell + " 가 숫자가 아님"; return false;}
      d_set = Eigen::Vector2d(c[0], c[1]) / 1000.0;
      if (c.size() == 3 && std::isfinite(c[2])) {dz = c[2] / 1000.0;}
    }
    const Eigen::Vector2d d = d_rack + d_set;
    if (d.norm() < 1e-6 && std::fabs(dz) < 1e-6) {return true;}
    if (!known) {why = "칸 " + cell + " 이 칸 표 (rack_cell_names) 에 없어 칸 경로를 못 옮김"; return false;}
    if (!model_ok_) {why = "팔 모델 없음 → 칸 경로를 못 옮김"; return false;}
    CellShiftReport rep;
    std::string err;
    // 다 옮기면 관절 범위 밖이면 (11-06 mock: S11 넣는 점 맞추기 10.7 mm → J4 범위 밖) 75 · 50 · 25 % 로 줄여 되는 만큼만, 그래도 안 되면 기록 그대로
    //   (기록 경로는 실물에서 들어간 경로 — 거절하지 않음)
    double used = 0.0;
    for (double sc : {1.0, 0.75, 0.5, 0.25}) {
      ArmPath p2 = path;
      std::string e2;
      if (shift_cell_path(model_, p2, store, rack_xy_[i], d * sc, rep, e2, dz * sc)) {path = p2; used = sc; break;}
      if (err.empty()) {err = e2;}
    }
    if (used == 0.0) {
      RCLCPP_WARN(get_logger(), "%s: 칸 근처를 (%+.1f, %+.1f) mm 옮기면 관절 범위 밖 (%s) → 25 %% 도 안 됨, 기록 경로 그대로", name.c_str(),
        d.x() * 1000.0, d.y() * 1000.0, err.c_str());
      return true;
    }
    const Eigen::Vector2d du = d * used;
    RCLCPP_INFO(get_logger(), "%s: 칸 근처를 (%+.1f, %+.1f) mm · 높이 %+.1f mm 옮김%s = 적재함 움직임 (%+.1f, %+.1f) + 칸 보정 (%+.1f, %+.1f) "
      "(점 %d 개 · 관절 최대 %.2f° · 손끝 어긋남 %.2f mm)", name.c_str(), du.x() * 1000.0, du.y() * 1000.0, dz * used * 1000.0,
      used < 1.0 ? (" (다 옮기면 관절 범위 밖 → " + std::to_string(static_cast<int>(used * 100)) + " %)").c_str() : "",
      d_rack.x() * 1000.0, d_rack.y() * 1000.0, d_set.x() * 1000.0, d_set.y() * 1000.0, rep.moved, rep.max_joint_deg, rep.worst_pos_mm);
    return true;
  }

  void on_detections(const interfaces::msg::DetectionArray & m)
  {
    std::vector<SeenPuck> out;
    const double t = rclcpp::Time(m.header.stamp).seconds();
    for (const auto & d : m.detections) {
      const auto & pp = d.pose.position;
      if (!std::isfinite(pp.x) || !std::isfinite(pp.y) || !std::isfinite(pp.z)) {continue;}   // 위치 모름 (Detection.msg 약속)
      geometry_msgs::msg::PoseStamped in, o;
      in.header = d.header.frame_id.empty() ? m.header : d.header;
      in.pose = d.pose;
      const auto & q0 = d.pose.orientation;
      const bool q_known = q0.x * q0.x + q0.y * q0.y + q0.z * q0.z + q0.w * q0.w > 0.5;
      if (!q_known) {in.pose.orientation.w = 1.0;}
      const geometry_msgs::msg::PoseStamped in_nom = in;   // 되돌리기 전 (pick det · planning 목표는 이 명목 TF 로 계산됨)
      if (drift_correct_ && !drift_frame_.empty() && in.header.frame_id == drift_frame_) {   // 카메라가 돈 만큼 되돌림 (실제 = 명목 · R → p 를 R · p 로)
        const Eigen::Vector3d p = drift_q_ * Eigen::Vector3d(in.pose.position.x, in.pose.position.y, in.pose.position.z);
        in.pose.position.x = p.x();
        in.pose.position.y = p.y();
        in.pose.position.z = p.z();
        if (q_known) {
          const auto & o = in.pose.orientation;
          const Eigen::Quaterniond r = drift_q_ * Eigen::Quaterniond(o.w, o.x, o.y, o.z);
          in.pose.orientation.x = r.x();
          in.pose.orientation.y = r.y();
          in.pose.orientation.z = r.z();
          in.pose.orientation.w = r.w();
        }
      }
      bool latest = false;
      try {
        o = tf_buf_->transform(in, odom_frame_, tf2::durationFromSec(0.0));
      } catch (const tf2::TransformException &) {
        in.header.stamp = rclcpp::Time(0, 0, get_clock()->get_clock_type());
        latest = true;
        try {
          o = tf_buf_->transform(in, odom_frame_, tf2::durationFromSec(0.0));
        } catch (const tf2::TransformException & e) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "손목 인지 검출을 %s 로 못 바꿈 (%s) → 이웃으로 안 씀", odom_frame_.c_str(), e.what());
          continue;
        }
      }
      Eigen::Vector3d p_nom(o.pose.position.x, o.pose.position.y, o.pose.position.z);
      if (in_nom.pose.position != in.pose.position) {
        geometry_msgs::msg::PoseStamped a = in_nom, b;
        if (latest) {a.header.stamp = in.header.stamp;}
        try {
          b = tf_buf_->transform(a, odom_frame_, tf2::durationFromSec(0.0));
          p_nom = Eigen::Vector3d(b.pose.position.x, b.pose.position.y, b.pose.position.z);
        } catch (const tf2::TransformException &) {}
      }
      const auto & q = o.pose.orientation;
      const double zz = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);                 // 긴축(z)의 위쪽 성분
      const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
      const bool square = d.shape != "cylinder";                               // 모양 모름("") = 사각으로 (보수적)
      const double axis = std::atan2(2.0 * (q.y * q.z - q.w * q.x), 2.0 * (q.x * q.z + q.w * q.y));   // 긴축(z) 의 수평 방향
      out.push_back({Eigen::Vector3d(o.pose.position.x, o.pose.position.y, o.pose.position.z), square,
        square && d.yaw_known && q_known, yaw, !q_known || std::fabs(zz) < 0.7, t, d.center_from_tag,
        q_known && std::fabs(zz) < std::sqrt(0.5), axis, p_nom});
    }
    seen_ = out;
    seen_t_ = t;   // 빈 배열도 "카메라는 살아 있고 아무것도 없음"
  }

  // 목표 (x, y, 퍽 중심 높이 zc, base_link) 주변 이웃 퍽. 목표 자신 = 25 mm 안 (선 퍽 두 개는 30 mm 이상 떨어짐)
  //   같은 면 위 퍽만 (높이 ±40 mm). 목표로 보이는 검출이 있으면 그 모양 · 면 방향 · 누움 · 긴축 방향 (lying_axis, base_link)
  std::vector<NeighborPuck> neighbors_near(double x, double y, double zc, bool & found, TargetPuck & tgt, Eigen::Vector2d & tc,
    bool & stale, bool & tagged, bool & lying, double & lying_axis) const
  {
    std::vector<NeighborPuck> nb;
    found = false;
    stale = !std::isfinite(seen_t_) || now().seconds() - seen_t_ > nb_age_s_;
    if (stale || seen_.empty()) {return nb;}
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buf_->lookupTransform("base_link", odom_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s → base_link TF 없음 (%s) → 이웃 정보 없이", odom_frame_.c_str(), e.what());
      stale = true;
      return nb;
    }
    const auto & r = tf.transform.rotation;
    const Eigen::Quaterniond Q(r.w, r.x, r.y, r.z);
    const Eigen::Vector3d T(tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z);
    const double dyaw = std::atan2(2.0 * (r.w * r.z + r.x * r.y), 1.0 - 2.0 * (r.y * r.y + r.z * r.z));
    double best = 1e9;   // 목표로 맞춘 검출의 거리 — 둘 이상 25 mm 안이면 가장 가까운 것 (명목 위치 우선, 목표가 명목 TF 로 나옴)
    std::optional<NeighborPuck> held_back;   // 지금까지 목표였다가 더 가까운 것에 밀린 검출 → 이웃으로 (nb_radius 안일 때)
    for (const auto & p : seen_) {
      const Eigen::Vector3d b = Q * p.p + T, bn = Q * p.p_nom + T;
      if (exclude_ && std::hypot(b.x() - exclude_->x(), b.y() - exclude_->y()) < 0.025) {continue;}   // 다시 잡기: 집어 든 퍽
      if (std::fabs(b.z() - zc) > 0.04 && std::fabs(bn.z() - zc) > 0.04) {continue;}   // 다른 높이 (쥔 퍽 · 박스 밖 바닥 등)
      const double d = std::hypot(b.x() - x, b.y() - y);
      const double dn = std::hypot(bn.x() - x, bn.y() - y);
      NeighborPuck n;
      n.c = b.head<2>();
      n.square = p.square;
      n.yaw_known = p.yaw_known && !p.round_r30;
      n.yaw = p.yaw + dyaw;
      n.radius = p.round_r30 ? 0.03 : 0.0;   // 누운 · 방향 모름 = 긴축 반 (30 mm) 원
      // 목표 = 되돌린 위치 또는 되돌리기 전 위치가 25 mm 안 (pick det · planning 은 명목 TF 로 목표를 냄 — 카메라가 3° 돌면 둘이 ~18 mm 차이)
      const double score = std::min(dn, d + 0.001);
      if (std::min(d, dn) < 0.025 && score < best) {
        if (held_back) {nb.push_back(*held_back);}
        best = score;
        found = true;
        tgt = {p.square, p.yaw_known, p.yaw + dyaw, d, true};
        tc = b.head<2>();
        tagged = p.tagged;
        lying = p.lying;
        lying_axis = p.axis + dyaw;
        held_back = d <= nb_radius_ ? std::optional<NeighborPuck>(n) : std::nullopt;
        continue;
      }
      if (d > nb_radius_) {continue;}
      nb.push_back(n);
    }
    return nb;
  }

  // 이웃 퍽을 보고 닫힘 방향 [°] · 벌림 키워드 (open | release) 후보를 고르는 순서대로. 이웃 없으면 기본 하나 (NaN, open)
  //   place: 놓을 자리 25 mm 안에 이미 퍽이 보이면 거절
  //   PICK 에서 목표 검출이 맞춰지면 x · y 를 그 검출 중심으로 바꿈 (방금 본 위치, 목표 여유 계산의 어긋남 0)
  //   lying (누운 네모 퍽, yaw_deg = 긴축 방향 · NaN = 모름): 닫힘 방향은 긴축에 수직 하나 (lying_close_rad, 모르면 기본 방향)
  //     PICK: goal 에는 모양이 없음 → 손목 검출로 누운 네모 · 윗면이 위인지 확인 (누운 원기둥 · 모서리로 기댄 네모 · 검출 없음은 거절),
  //       긴축은 그 검출 방향. 다 열기만 (넣기 폭은 34.8 mm 퍽과 양쪽 1 mm 뿐)
  //     PLACE: 놓을 퍽 발자국 (긴축 ±30 mm, 손가락 폭 ±16.4 mm 밖까지) 도 이웃과 grasp_min_clear 넘게
  bool grasp_for(double & x, double & y, double zc, double yaw_deg, bool place,
    std::vector<std::pair<double, std::string>> & cands, std::string & why, const char * what, bool lying = false)
  {
    cands.clear();
    lying_axis_deg_ = std::numeric_limits<double>::quiet_NaN();
    lying_cylinder_ = false;
    bool found = false, stale = false, seen_lying = false;
    double seen_axis = 0.0;
    TargetPuck tgt;
    Eigen::Vector2d tc(x, y);
    target_tagged_ = false;
    const auto nb = neighbors_near(x, y, zc, found, tgt, tc, stale, target_tagged_, seen_lying, seen_axis);
    last_nb_ = nb;
    if (place && found) {why = std::string(what) + ": 놓을 자리에 이미 퍽이 보임"; return false;}
    if (found && !place && seen_lying != lying) {
      why = std::string(what) + (lying ? ": 목표는 누운 퍽인데 손목 검출은 선 퍽 → 다시 LOOK" :
        ": 목표는 선 퍽인데 손목 검출은 누운 퍽 → 검출 방향으로 다시 (pick det)");
      return false;
    }
    // 박스 안은 소프트웨어로 막지 않음 (11-05 사용자: "박스 안에는 다 풀어") — 모르는 것은 알리고 가장 그럴듯한 값으로 쥠. 못 닿는 자세만 IK 가 거절
    if (lying && !place) {
      if (found) {
        if (!tgt.square) {RCLCPP_INFO(get_logger(), "%s: 누운 원기둥 — 긴축에 수직으로 둥근 옆면을 쥠", what);}
        if (!tgt.yaw_known) {RCLCPP_WARN(get_logger(), "%s: 누운 퍽 윗면을 못 잡음 (기댔을 수 있음) — 검출 긴축 그대로 쥠", what);}
        yaw_deg = seen_axis * 180.0 / M_PI;
      } else if (std::isfinite(yaw_deg)) {
        RCLCPP_WARN(get_logger(), "%s: 누운 퍽 손목 검출 없음 (%s) — 목표 긴축 %.0f° 로 쥠", what, stale ? "오래됨" : "그 자리에 없음", yaw_deg);
      } else {
        why = std::string(what) + ": 누운 퍽인데 긴축 방향을 모름 (손목 검출 없음 · 목표에도 없음) — LOOK 뒤 다시";
        return false;
      }
      lying_axis_deg_ = yaw_deg;
      lying_cylinder_ = found && !tgt.square;
    }
    if (found && !place && tgt.square && !tgt.yaw_known) {   // 면 방향 모르는 네모: 대각선으로 쥘 수 있음 (11-04) — 막지 않고 알림
      RCLCPP_WARN(get_logger(), "%s: 네모 퍽 면 방향을 못 잡음 — 기본 방향으로 쥠 (대각선으로 쥘 수 있음, STORE 전에 확인)", what);
    }
    if (found && !place) {
      RCLCPP_INFO(get_logger(), "%s: 목표 검출 중심 (%.3f, %.3f) 로 맞춤 (목표점과 %.1f mm)", what, tc.x(), tc.y(), tgt.offset * 1000.0);
      x = tc.x();
      y = tc.y();
      tgt.offset = 0.0;
    }
    if (lying) {
      const double def = default_close_rad(model_, x, y, std::nan(""));
      const double c = std::isfinite(yaw_deg) ? lying_close_rad(model_, x, y, yaw_deg) : def;
      GraspLimits lim;
      lim.min_clear = nb_min_clear_;
      lim.target_clear = nb_target_clear_;
      std::string rwhy;
      const auto ranked = rank_grasps(Eigen::Vector2d(x, y), TargetPuck{true, true, c, 0.0, !place}, {c},
        place ? std::vector<double>{grip_.open, grip_.release} : std::vector<double>{grip_.open}, nb, def, rwhy, lim);
      if (ranked.empty()) {why = std::string(what) + ": 누운 퍽 — " + rwhy; return false;}
      const double clr = place ? footprint_clearance(Eigen::Vector2d(x, y), c + M_PI / 2.0, Eigen::Vector2d(puck_h_ / 2.0, lying_h_ / 2.0), nb) : 1.0;
      if (clr < nb_min_clear_) {
        char buf[160];
        std::snprintf(buf, sizeof(buf), "%s: 누운 퍽 (긴축 %.0f mm) 이 이웃 퍽과 여유 %.1f mm", what, puck_h_ * 1000.0, clr * 1000.0);
        why = buf;
        return false;
      }
      for (const auto & ch : ranked) {cands.push_back({c * 180.0 / M_PI, ch.grip >= grip_.open - 1e-6 ? "open" : "release"});}
      RCLCPP_INFO(get_logger(), "%s: 누운 퍽 긴축 %.0f° → 닫힘 방향 %.0f° (긴축에 수직) · 벌림 %s · 이웃 퍽 %zu 개 (손가락 여유 %.1f mm)", what,
        std::isfinite(yaw_deg) ? yaw_deg : c * 180.0 / M_PI - 90.0, c * 180.0 / M_PI, cands.front().second.c_str(), nb.size(),
        nb.empty() ? 0.0 : ranked.front().clearance * 1000.0);
      return true;
    }
    if (nb.empty()) {   // 방향 없이 → IK 가 네모 면 4 방향 중 J5 가 받침 자세에 가까운 쪽 (끝자리는 J5 를 90° 돌려 쥠 → pick_offset_rot_mm)
      RCLCPP_INFO(get_logger(), "%s: 이웃 퍽 %s → 기본 방향 · 다 열기", what, stale ? "정보 없음 (손목 인지 결과가 없거나 오래됨)" : "없음");
      cands.push_back({std::nan(""), "open"});
      return true;
    }
    // 좁게 열기(넣기 폭)는 목표 검출이 맞춰졌을 때만 (planning 은 원기둥 · 면 모르는 네모에도 x 축을 채워 보냄 → goal 방향만으로는 못 믿음)
    std::vector<double> grips{grip_.open, grip_.release};
    if (!found || place) {   // 후보 방향은 goal 방향으로 (방향 있으면 면 방향 2 개)
      tgt = {true, std::isfinite(yaw_deg), std::isfinite(yaw_deg) ? yaw_deg * M_PI / 180.0 : 0.0, 0.0, !place};
      if (!found && !place) {grips = {grip_.open};}
    }
    const bool sq_known = tgt.square && tgt.yaw_known;
    const double def = default_close_rad(model_, x, y, sq_known ? tgt.yaw * 180.0 / M_PI : std::nan(""));
    GraspLimits lim;
    lim.min_clear = nb_min_clear_;
    lim.target_clear = nb_target_clear_;
    std::string rwhy;
    const auto ranked = rank_grasps(Eigen::Vector2d(x, y), tgt, close_candidates(sq_known, tgt.yaw, def),
      grips, nb, def, rwhy, lim);
    if (ranked.empty()) {why = std::string(what) + ": " + rwhy; return false;}
    for (const auto & ch : ranked) {
      double c = ch.close_rad;
      if (std::cos(c - def) < 0.0) {c += M_PI;}   // 기본 방향 쪽 반 (같은 손가락 배치, 손목이 덜 돎)
      cands.push_back({c * 180.0 / M_PI, ch.grip >= grip_.open - 1e-6 ? "open" : "release"});
    }
    RCLCPP_INFO(get_logger(), "%s: 이웃 퍽 %zu 개 → 후보 %zu 개, 첫째 닫힘 방향 %.0f° (기본 %.0f°) · 벌림 %s · 여유 %.1f mm", what,
      nb.size(), ranked.size(), cands.front().first, def * 180.0 / M_PI, cands.front().second.c_str(), ranked.front().clearance * 1000.0);
    return true;
  }

  // 후보를 차례로 IK (안 되면 다음 후보). 수직이 안 닿으면 기울인 손 (pick_max_tilt_deg 까지)
  //   기울인 손은 쥐는 높이에서 손가락이 차체 쪽으로 쥔 깊이(이웃 퍽 윗면 아래로 들어간 손가락 길이) · tan(기울임) 더 차지 → 이웃이 있으면 손가락 폭을 그만큼 넓혀 다시 검사
  std::vector<PickPose> poses_for(double x, double y, double top, double yaw, const std::vector<double> & heights,
    const std::vector<std::pair<double, std::string>> & cands, std::string & kw, std::string & why,
    double prefer_tilt = 0.0, double max_dev = 1e9, const std::vector<Q5> & seeds = {}) const
  {
    // 기울인 손은 닫힘축이 늘 접선 (pick_poses) → 접선에 가까운 후보 (35° 안) 의 기울인 풀이는 벌림마다 한 번만 (같은 답, 후보 수십 개면 수 초)
    //   수직 손: 맨 위 높이에 수직 손이 안 닿으면 (닫힘 방향과 상관없는 거리 한계 — J5 범위 293° 라 어느 방향이든 한쪽 부호는 됨) 후보마다 안 풀어 봄
    const double tang = std::atan2(y - model_.j1_xy.y(), x - model_.j1_xy.x()) + M_PI / 2.0;
    std::set<std::string> tilted_done;
    bool vertical = true;
    if (cands.size() > 1 && !heights.empty()) {
      std::string w;
      vertical = !pick_poses(model_, x, y, top, yaw, {heights.front()}, w, std::nan(""), 0.0, 0.0, 1e9, seeds).empty();
    }
    for (const auto & [cd, k] : cands) {
      const bool tilt_able = std::isfinite(cd) && std::fabs(std::remainder(cd * M_PI / 180.0 - tang, M_PI)) <= 35.0 * M_PI / 180.0;
      const bool try_tilt = !tilt_able || tilted_done.insert(k).second;
      if (!vertical && !try_tilt) {continue;}
      auto P = pick_poses(model_, x, y, top, yaw, heights, why, cd, try_tilt ? max_tilt_deg_ : 0.0, prefer_tilt, max_dev, seeds, vertical);
      if (P.size() != heights.size()) {continue;}
      const double tilt = P.back().tilt_deg;
      if (tilt > 0.0 && !last_nb_.empty()) {
        FingerTip f;
        const double e = std::max(0.0, -heights.back() / 1000.0) * std::tan(tilt * M_PI / 180.0);
        f.w_lo -= e;
        f.w_hi += e;
        const double c = std::atan2(y - model_.j1_xy.y(), x - model_.j1_xy.x()) + M_PI / 2.0;   // 기울인 손의 닫힘축 = 접선 (pick_poses)
        const double clr = finger_clearance(Eigen::Vector2d(x, y), c, k == "open" ? grip_.open : grip_.release, last_nb_, f);
        if (clr < nb_min_clear_) {
          char buf[160];
          std::snprintf(buf, sizeof(buf), "손을 %.0f° 기울이면 이웃 퍽과 여유 %.1f mm", tilt, clr * 1000.0);
          why = buf;
          continue;
        }
      }
      if (tilt > 0.0) {RCLCPP_INFO(get_logger(), "퍽 (%.3f, %.3f): 수직 손이 안 닿아 손을 %.0f° 기울임", x, y, tilt);}
      kw = k;
      return P;
    }
    return {};
  }

  // 다시 잡기 경유점: (px, py) 에서 손을 tilt 기울여 depth 깊이로 쥔 퍽을 J1 쪽으로 당긴 빈 자리에 내려놓고 (쥘 때 기울임 그대로
  //   = 퍽은 수직, 퍽 바닥이 면 위 place_clear_mm) 수직 손으로 다시 쥠 (pick_heights_). 놓기 · 다시 쥐기 모두 같은 닫힘 방향 =
  //   그 자리의 접선 (기울인 손은 닫힘축이 접선뿐) → 퍽 면이 손가락과 맞은 채로 놓이고 그대로 다시 쥠
  //   자리 후보: 같은 방향 (J1 → 퍽) 에서 박스 위 가장 가까운 곳 (x = regrasp_min_x) 부터 1 cm 씩 바깥으로 퍽 1 cm 앞까지,
  //   각각 그대로 · 좌우 4 · 8 cm. 수직 손이 넉넉히 닿는 곳에 먼저 놓음 (끝자리에 놓으면 다시 쥘 때 덜 정확).
  //   그래도 없으면 박스 앞쪽 끝 (x = regrasp_min_x ~ +2 cm) 을 따라 가운데 (y = 0) 쪽으로 1 cm 씩 (구석 퍽)
  bool regrasp_nodes(double px, double py, double top, double tilt, double depth, std::vector<PathNode> & out, double & sx, double & sy,
    std::string & why)
  {
    const double zc = top - model_.team_base_z_floor + model_.base_z_floor - puck_h_ / 2.0;
    const Eigen::Vector2d j1 = model_.j1_xy, p0(px, py);
    const double r0 = (p0 - j1).norm();
    if (r0 < 1e-6) {why = "다시 잡기: 퍽이 J1 위"; return false;}
    const Eigen::Vector2d dir = (p0 - j1) / r0, side(-dir.y(), dir.x());
    exclude_ = p0;
    std::string last = "후보 없음";
    bool ok = false;
    auto try_spot = [&](double x, double y) {
        if (x < regrasp_min_x_) {
          if (last == "후보 없음") {last = "박스 앞쪽 끝 (regrasp_min_x)";}   // 다른 이유가 있으면 그것을 남김
          return false;
        }
        std::vector<std::pair<double, std::string>> cands;
        double gx = x, gy = y;
        std::string w;
        if (!grasp_for(gx, gy, zc, std::nan(""), true, cands, w, "다시 잡기 자리")) {last = w; return false;}   // 빈 자리 · 손가락 여유
        const double c_deg = (std::atan2(y - j1.y(), x - j1.x()) + M_PI / 2.0) * 180.0 / M_PI;
        std::string kw;
        const std::vector<double> hp{place_heights_[0], place_heights_[1], place_clear_mm_ - depth};
        const auto Pp = poses_for(x, y, top, c_deg, hp, {{c_deg, "release"}}, kw, w, tilt, 0.5);
        if (Pp.size() != 3) {last = "놓기: " + w; return false;}
        const auto Pr = poses_for(x, y, top, c_deg, pick_heights_, {{c_deg, "open"}}, kw, w, 0.0, 0.5);
        if (Pr.size() != 3 || Pr.back().tilt_deg > 0.5) {last = "수직으로 다시 쥐기: " + (Pr.size() == 3 ? std::string("수직 손이 안 닿음") : w); return false;}
        out = {model_node("re_place_above", Pp[0].enc, slow_dps_), model_node("re_place_down", Pp[1].enc, slow_dps_),
          model_node("re_place", Pp[2].enc, slow_dps_, "release"), model_node("re_place_up", Pp[1].enc, slow_dps_),
          model_node("re_place_clear", Pp[0].enc, slow_dps_), model_node("re_above", Pr[0].enc, slow_dps_, "open"),
          model_node("re_down", Pr[1].enc, slow_dps_), model_node("re_grasp", Pr[2].enc, slow_dps_, "close"),
          model_node("re_up", Pr[1].enc, slow_dps_), model_node("re_lift", Pr[0].enc, slow_dps_)};
        sx = x;
        sy = y;
        return true;
      };
    const double r_near = dir.x() > 0.1 ? (regrasp_min_x_ + 0.002 - j1.x()) / dir.x() : r0 - 0.01;   // x = regrasp_min_x 가 되는 반지름
    for (double r = std::min(r_near, r0 - 0.01); r <= r0 - 0.01 + 1e-9 && !ok; r += 0.01) {
      for (double lat : {0.0, 0.04, -0.04, 0.08, -0.08}) {
        const Eigen::Vector2d q = j1 + dir * r + side * lat;
        if ((ok = try_spot(q.x(), q.y()))) {break;}
      }
    }
    // 구석 퍽 (J1 에서 옆으로 많이 돈 방향): 그 방향은 박스 앞쪽 끝에서도 J1 에서 멀어 수직 손이 안 닿음 → 박스 앞쪽 끝을 따라 가운데 (y = 0) 쪽으로
    for (double dx = 0.002; dx <= 0.0221 && !ok; dx += 0.01) {
      const double x = regrasp_min_x_ + dx, y_line = std::fabs(dir.x()) > 0.1 ? j1.y() + dir.y() * (x - j1.x()) / dir.x() : py;
      for (double y = y_line; std::fabs(y) > 1e-9 && y * y_line > 0.0 && !ok; y -= std::copysign(0.01, y_line)) {ok = try_spot(x, y);}
    }
    exclude_.reset();
    if (!ok) {
      why = "손을 " + std::to_string(static_cast<int>(std::lround(tilt))) + "° 기울여 쥐어야 하는데 다시 잡을 빈 자리가 없음 (마지막: " + last +
        ") — 퍽을 차체 쪽으로 옮기기";
    }
    return ok;
  }

  std::vector<ArmAction> pick_plan(const ArmTask::Goal & goal, std::string & why)
  {
    const auto unfold = paths_.find("unfold");
    if (!model_ok_ || unfold == paths_.end() || staging_.size() != 5) {why = "팔 모델 · 펼치기 경로 · 기준점 설정 없음"; return {};}
    double x, y, top, yaw;
    bool lying = false;
    if (!target_in_model(goal, false, x, y, top, yaw, lying, why)) {return {};}
    std::vector<std::pair<double, std::string>> cands;
    std::string open_kw;
    const double zc = top - model_.team_base_z_floor + model_.base_z_floor - (lying ? lying_h_ : puck_h_) / 2.0;   // 퍽 중심 (base_link)
    if (!grasp_for(x, y, zc, yaw, false, cands, why, "PICK", lying)) {return {};}
    // 태그로 잰 중심 (perception center_from_tag): 선 퍽 = 윗면 태그, 누운 퍽 = 위를 향한 옆면 태그 (11-06) — 보정값이 따로
    const bool tag_centre = target_tagged_;
    {
      const char * px = tag_centre ? (lying ? "pick_offset_lying_tagged_x_mm" : "pick_offset_tagged_x_mm") :
        (lying ? "pick_offset_lying_x_mm" : "pick_offset_x_mm");
      const char * py = tag_centre ? (lying ? "pick_offset_lying_tagged_y_mm" : "pick_offset_tagged_y_mm") :
        (lying ? "pick_offset_lying_y_mm" : "pick_offset_y_mm");
      const double ox = get_parameter(px).as_double(), oy = get_parameter(py).as_double();
      if (!std::isfinite(ox) || !std::isfinite(oy)) {
        why = std::string(px) + " · " + py + " 가 숫자가 아님";
        return {};
      }
      if (tag_centre) {
        x += ox / 1000.0;
        y += oy / 1000.0;
        RCLCPP_INFO(get_logger(), "PICK: %s 태그로 잰 퍽 → 태그 퍽 보정 (%+.1f, %+.1f) mm (%s)", lying ? "누운 퍽 위 옆면" : "윗면", ox, oy,
          lying ? "pick_offset_lying_tagged_*" : "pick_offset_tagged_*");
      } else if (ox != 0.0 || oy != 0.0) {
        x += ox / 1000.0;
        y += oy / 1000.0;
        RCLCPP_INFO(get_logger(), "PICK: 쥐는 점을 (%+.1f, %+.1f) mm 옮김 (%s, %s)", ox, oy,
          lying ? "pick_offset_lying_x_mm · y_mm" : "pick_offset_x_mm · y_mm", lying ? "누운 퍽, 태그 안 보임 = 깊이 점 중심" : "태그 없는 퍽");
      }
    }
    // 손 기울임 10° 까지 = pick_heights_ (−40, 적재함 경로와 같은 깊이), 그보다 기울여야 하면 pick_tilted_depth_mm (퍽 무게중심)
    //   기울임 후보는 10° 간격 (pick_poses): 2 번째 풀이는 20° 부터 max_tilt 까지, 수직에 가까운 것부터
    //   누운 퍽: 수직 손만 (pick_lying_heights_mm). 기울여 쥐면 내려놓아도 누운 채라 다시 잡기로 못 세움 → 안 닿으면 거절
    pending_empty_ = false;
    if (lying && get_parameter("stand_lying").as_bool() && lying_cylinder_) {
      RCLCPP_INFO(get_logger(), "PICK: 누운 원기둥 — 세우기 표는 네모 퍽 전용이라 누운 채 쥠");
    }
    if (lying && get_parameter("stand_lying").as_bool() && !lying_cylinder_ && get_parameter("stand_lying_mode").as_string() == "park") {
      return park_plan(unfold->second, x, y, top, cands, open_kw, why);
    }
    if (lying && get_parameter("stand_lying").as_bool() && !lying_cylinder_) {
      if (standup_.empty()) {why = "stand_lying 인데 세우기 경로 표가 없음 (standup_paths_file)"; return {};}
      return standup_plan(unfold->second, x, y, top, why);
    }
    if (lying) {
      const auto h = get_parameter("pick_lying_heights_mm").as_double_array();
      const double clear = get_parameter("pick_lying_tip_clear_mm").as_double();
      const double floor_min = clear - lying_h_ * 1000.0;   // 손끝이 놓인 면 위 clear mm (모델)
      if (h.size() != 3 || !(h[0] > h[1] && h[1] > h[2] && h[2] >= floor_min)) {
        char buf[200];
        std::snprintf(buf, sizeof(buf), "pick_lying_heights_mm 는 위 > 내림 > 쥐기 3 개, 쥐기 %.1f 이상 (손끝이 놓인 면 위 %.0f mm, pick_lying_tip_clear_mm)",
          floor_min, clear);
        why = buf;
        return {};
      }
      // 수직 손이 먼저, 안 닿으면 손을 기울여 (pick_max_tilt_deg 까지). 기울인 손은 닫힘축이 접선뿐이라 긴축이 반지름 방향에 가까울 때만 됨
      const auto P = poses_for(x, y, top, yaw, h, cands, open_kw, why, 0.0, max_tilt_deg_);
      if (P.size() != 3) {why = "누운 퍽: 손이 안 닿음 (" + why + ")"; return {};}
      pending_tilt_ = P.back().tilt_deg;
      pending_depth_ = -h.back();
      pending_lying_ = true;
      return pick_path(unfold->second, P, open_kw, {}, why, x, y, yaw, true);
    }
    auto P = poses_for(x, y, top, yaw, pick_heights_, cands, open_kw, why, 0.0, kStoreMaxTilt);
    if (P.size() == 3 && P.back().tilt_deg < 0.5) {
      Eigen::Vector2d d;
      double rot = 0.0;
      if (!rot_offset(x, y, P.back().ticks, d, rot, why)) {return {};}
      if (std::fabs(rot) >= 20.0 || d.norm() > 1e-4) {   // 보정이 0 이어도 돌려 쥔 것은 알림 (pick_offset_rot_mm 을 맞출 때)
        RCLCPP_INFO(get_logger(), "PICK: 손목을 기본에서 %+.0f° 돌려 쥠 → 돌림 보정 (%+.1f, %+.1f) mm (pick_offset_rot_mm)", rot, d.x() * 1000.0,
          d.y() * 1000.0);
      }
      if (d.norm() > 1e-4) {
        x += d.x();
        y += d.y();
        P = poses_for(x, y, top, yaw, pick_heights_, cands, open_kw, why, 0.0, kStoreMaxTilt);
      }
    }
    double depth = -pick_heights_.back();
    if (P.size() != 3 && max_tilt_deg_ >= kStoreMaxTilt + 10.0 - 1e-6) {
      auto h = pick_heights_;
      h.back() = -tilted_depth_mm_;
      std::string why2;
      P = poses_for(x, y, top, yaw, h, cands, open_kw, why2, kStoreMaxTilt + 10.0, max_tilt_deg_ - kStoreMaxTilt - 10.0);
      depth = tilted_depth_mm_;
      if (P.size() != 3) {why = why + " · 기울인 손 (" + why2 + ")";}
    }
    if (P.size() != 3) {return {};}
    pending_tilt_ = P.back().tilt_deg;
    pending_depth_ = depth;
    pending_lying_ = false;
    std::vector<PathNode> again;
    if (regrasp_ && pending_tilt_ > 0.5) {
      double sx = 0.0, sy = 0.0;
      if (!regrasp_nodes(x, y, top, pending_tilt_, depth, again, sx, sy, why)) {return {};}
      RCLCPP_INFO(get_logger(), "PICK: 손을 %.0f° 기울여 쥐어야 함 → (%.3f, %.3f) 에 세워 내려놓고 수직 손으로 다시 쥠", pending_tilt_, sx, sy);
      pending_tilt_ = 0.0;
      pending_depth_ = -pick_heights_.back();
    }
    return pick_path(unfold->second, P, open_kw, again, why, x, y, yaw, false);
  }

  // 세우기: 쥐기 뒤 실측 확인 — 손끝이 쥐는 자리에 내려갔나 (11-05 실물: 손가락이 퍽 위에 얹혀 17 mm 높게 멈춘 채 세우기가 돌아감),
  //   그리퍼가 빈손으로 끝까지 닫히지 않았나. 아니면 false (bad)
  bool standup_grasp_ok(std::string & bad) const
  {
    char buf[200];
    if (joint_buf_.size() != 5) {bad = "관절 실측 없음"; return false;}
    Ticks t;
    for (int j = 0; j < 5; ++j) {
      if (!std::isfinite(joint_buf_[j])) {bad = "관절 실측 없음"; return false;}
      t[j] = static_cast<int>(std::lround(model_.zero_tick[j] + joint_buf_[j] * 4096.0 / (2.0 * M_PI)));
    }
    const Eigen::Vector3d tip = arm_grip(model_, ticks_to_q(model_, t)).tip;
    const double d_mm = (tip - standup_check_->tip).norm() * 1000.0;
    if (d_mm > standup_reach_tol_mm_) {
      std::snprintf(buf, sizeof(buf), "쥐는 자리에 못 내려감 — 손끝이 목표에서 %.1f mm (높이 %+.1f mm, 손가락이 퍽 위에 얹혔을 수 있음)", d_mm,
        (tip.z() - standup_check_->tip.z()) * 1000.0);
      bad = buf;
      return false;
    }
    if (std::isfinite(grip_now_) && grip_now_ < standup_grip_min_m_) {
      std::snprintf(buf, sizeof(buf), "그리퍼가 %.4f m 까지 닫힘 — 퍽을 못 쥠", grip_now_);
      bad = buf;
      return false;
    }
    return true;
  }

  // 누운 네모 퍽 PICK + 세우기 (stand_lying): 표 위 → 쥐기 → 세우기 경로 → 손 5 cm 위 → 선 퍽 다시 쥐기 → 기준점
  std::vector<ArmAction> standup_plan(const ArmPath & unfold, double x, double y, double top, std::string & why)
  {
    const double axis = lying_axis_deg_;   // 아래 다시 잡기 자리 찾기 (grasp_for) 가 지움
    if (!std::isfinite(axis)) {why = "세우기: 누운 퍽 긴축 방향을 모름 (손목 검출)"; return {};}
    double dx = 0.0, dy = 0.0, da = 0.0;
    standup_speed_dps_ = get_parameter("standup_speed_dps").as_double();   // 실행 중 바꾸면 다음 PICK 부터
    standup_max_gap_mm_ = get_parameter("standup_max_gap_mm").as_double();
    standup_max_gap_deg_ = get_parameter("standup_max_gap_deg").as_double();
    standup_reach_tol_mm_ = get_parameter("standup_reach_tol_mm").as_double();
    standup_grip_min_m_ = get_parameter("standup_grip_min_m").as_double();
    const int k = nearest_standup(standup_, x, y, axis, lying_h_, dx, dy, da);
    if (k < 0) {why = "세우기: 표가 비어 있음"; return {};}
    const auto & e = standup_[static_cast<size_t>(k)];
    if (std::hypot(dx, dy) * 1000.0 > standup_max_gap_mm_ || std::abs(da) > standup_max_gap_deg_) {
      char buf[220];
      std::snprintf(buf, sizeof(buf), "세우기: 표에 가까운 자리가 없음 (퍽 (%.3f, %.3f) 긴축 %.0f° ↔ 표 (%.3f, %.3f) %.0f°, 차 %.1f mm · %.1f°)",
        x, y, axis, e.x, e.y, e.axis_deg, std::hypot(dx, dy) * 1000.0, da);
      why = buf;
      return {};
    }
    const double kg = 0.017;   // 누운 퍽 (세울 때 쥔 무게)
    const double dz = get_parameter("standup_dz_mm").as_double() / 1000.0;
    // 경로 전체 (위 · 쥐기 포함) 를 표 칸 퍽에서 실제 퍽으로 옮김 — 넣기 폭이 퍽 양쪽 1 mm 뿐이라 칸 차이 그대로면 손가락이 퍽에 얹힘
    //   박스 윗면 (누운 퍽 아래면) + standup_tip_clear_mm 아래를 가리키는 점은 그 높이까지 올림 (11-05 실물: 표가 박스 윗면 아래 10 mm 를 가리켜
    //   시뮬은 팔이 물러 막히지만 실물 모터는 거기까지 가려고 눌러 긁음). 쥔 뒤 점은 쥐는 손끝에서 수평 거리를 standup_reach_scale 배로 (덜 뻗기)
    StandupRetarget rt;
    rt.from = Eigen::Vector2d(e.x, e.y);
    rt.to = Eigen::Vector2d(x, y);
    rt.da = da * M_PI / 180.0;
    rt.dz = dz;
    const double floor_z = top - lying_h_;   // top 은 모델 높이 (target_in_model)
    rt.tip_min_z = floor_z + get_parameter("standup_tip_clear_mm").as_double() / 1000.0;
    rt.reach_scale = get_parameter("standup_reach_scale").as_double();
    double lifted_max = 0.0, app_err_max = 0.0;
    int lifted_n = 0;
    Ticks enc;
    Q5 q_true{};
    Eigen::Vector3d tip_low(0.0, 0.0, std::numeric_limits<double>::infinity());   // 경로에서 가장 낮은 손끝 [m, 모델]
    std::string fail;
    auto enc_of = [&](const std::array<int, 5> & t, double held_kg, bool scale) {
        Ticks tt;
        for (int j = 0; j < 5; ++j) {tt[j] = t[j];}
        RetargetResult rr;
        if (!standup_retarget(model_, ticks_to_q(model_, tt), rt, scale, rr)) {fail = rr.fail; return false;}
        q_true = rr.q;
        enc = q_to_ticks(model_, encoder_goal_q(model_, rr.q, held_kg));
        app_err_max = std::max(app_err_max, rr.app_err_deg);
        if (rr.lifted_mm > 0.0) {++lifted_n; lifted_max = std::max(lifted_max, rr.lifted_mm);}
        const Eigen::Vector3d tp = arm_grip(model_, q_true).tip;
        if (tp.z() < tip_low.z()) {tip_low = tp;}
        return true;
      };
    auto refuse = [&](const char * nm) {
        char buf[200];
        std::snprintf(buf, sizeof(buf), "세우기: %s 를 실제 퍽으로 (%+.1f, %+.1f) mm %+.1f° · 위아래 %+.0f mm 옮긴 자세 (IK) 가 없음 — ", nm,
          dx * 1000.0, dy * 1000.0, da, dz * 1000.0);
        why = buf + fail;
        return std::vector<ArmAction>{};
      };
    ArmPath path = unfold;
    path.name = "pick_standup";
    path.ends_home = false;
    if (!enc_of(e.above, 0.0, false)) {return refuse("위");}
    path.nodes.push_back(model_node("above", enc, 0.0, "open"));
    if (!enc_of(e.grasp, 0.0, false)) {return refuse("쥐기");}
    path.nodes.push_back(model_node("grasp", enc, slow_dps_, "close"));
    rt.anchor = arm_grip(model_, q_true).tip.head<2>();
    {   // 쥐는 손끝이 실제 퍽 중심 위에 오는지 (옮기기 전 표 그대로면 얼마나 빗나갔을지)
      Ticks tg;
      for (int j = 0; j < 5; ++j) {tg[j] = e.grasp[j];}
      const Eigen::Vector2d tip0 = arm_grip(model_, ticks_to_q(model_, tg)).tip.head<2>();
      RCLCPP_INFO(get_logger(), "PICK 세우기: 쥐는 손끝 (%.3f, %.3f) — 퍽 중심과 %.1f mm (표 칸 그대로였으면 %.1f mm)", rt.anchor.x(), rt.anchor.y(),
        (rt.anchor - Eigen::Vector2d(x, y)).norm() * 1000.0, (tip0 - Eigen::Vector2d(x, y)).norm() * 1000.0);
    }
    // 확인은 보낸 명령 (처짐 앞먹임 포함) 의 손끝과 실측 (엔코더) 손끝을 비교 — 앞먹임이 손끝을 약 18 mm 올려 보이게 함
    standup_check_ = StandupCheck{"", arm_grip(model_, ticks_to_q(model_, enc)).tip};   // 구간 이름은 경로 점을 다 만든 뒤
    // 경로 점이 IK 로 안 풀리면 그 점만 건너뜀 (11-05 사용자: 막지 말 것). 여는 점이 빠지면 다음 점에서 엶
    //   점 사이 시간 = 표 녹화 간격 (0.1 s) × standup_time_scale — 속도 상한으로 다시 짜면 시뮬 재생 84 → 47 % (11-06, 15 °/s · 여는 점 멈춤)
    const double dt = kStandupDt * get_parameter("standup_time_scale").as_double();
    int skipped = 0, gap = 1;   // gap: 건너뛴 점만큼 다음 점까지 시간을 늘림
    bool opened = false, any = false;
    for (size_t i = 0; i < e.q.size(); ++i) {
      const bool held = static_cast<int>(i) < e.release;
      if (!enc_of(e.q[i], held ? kg : 0.0, true)) {
        ++skipped;
        ++gap;
        RCLCPP_WARN(get_logger(), "PICK 세우기: 경로 점 %zu/%zu 건너뜀 (%s)", i, e.q.size(), fail.c_str());
        continue;
      }
      const bool open_here = !opened && static_cast<int>(i) >= e.release;
      opened = opened || open_here;
      any = true;
      PathNode n = model_node("stand" + std::to_string(i), enc, standup_speed_dps_, open_here ? "open" : "");
      n.dt_s = dt * gap;
      n.grip_on_way = open_here;   // 팔을 멈추지 않고 열기 (여는 점에서 0.2 s 만 멈춰도 시뮬 재생 84 → 36 %, 11-06)
      gap = 1;
      path.nodes.push_back(n);
    }
    if (!any) {return refuse("세우기 경로 점 전부");}
    if (!opened) {path.nodes.back().grip = "open";}
    path.nodes.back().stop = true;   // 시간 맞춘 구간은 여기서 끝 (다음 '손 위로' 는 속도 시간표)
    // 확인은 쥔 뒤 첫 세우기 동작을 보내기 전 — 동작 이름 = 그 동작의 끝 점 (멈추는 점 · 끝 점)
    for (const auto & n : path.nodes) {
      if (n.name.rfind("stand", 0) == 0 && (n.stop || (!n.grip.empty() && !n.grip_on_way))) {standup_check_->pose = n.name; break;}
    }
    RCLCPP_INFO(get_logger(), "PICK 세우기: 경로 손끝 최저 = 박스 윗면 위 %.0f mm @ (%.3f, %.3f) (모델, 실물은 약 10 mm 높음 · standup_dz_mm %.0f) · "
      "윗면 아래를 가리켜 올린 점 %d 개 (최대 %.0f mm) · 덜 뻗기 %.2f 배 · 건너뛴 점 %d 개", (tip_low.z() - floor_z) * 1000.0, tip_low.x(),
      tip_low.y(), dz * 1000.0, lifted_n, lifted_max, rt.reach_scale, skipped);
    // 손을 위로 (옮긴 마지막 자세의 접근축 · 닫힘축 그대로): 5 cm, LOOK 으로 끝낼 때는 12 cm 도
    const Q5 q_last = q_true;
    const Grip g = arm_grip(model_, q_last);
    // 시뮬에서 선 자리 · 면도 같이 옮김 (덜 뻗기 포함)
    const Eigen::Vector2d end0 = rt.to + Eigen::Rotation2Dd(rt.da) * (Eigen::Vector2d(e.end_x, e.end_y) - rt.from);
    const Eigen::Vector2d end_xy = rt.anchor + rt.reach_scale * (end0 - rt.anchor);
    const double end_face = e.end_face_deg + da;
    auto lift_q = [&](double dz, Q5 & out) {
        const auto sols = arm_ik(model_, g.tip + Eigen::Vector3d(0.0, 0.0, dz), g.app, g.close, {q_last}, 180.0);
        double best = 1e9;
        for (const auto & sl : sols) {
          if (sl.pos_mm < 1.0 && sl.app_deg < 1.0 && sl.roll_deg < 2.0 && ticks_in_range(model_, q_to_ticks(model_, sl.q))) {
            double d = 0.0;
            for (int j = 0; j < 5; ++j) {d += std::abs(sl.q[j] - q_last[j]);}
            if (d < best) {best = d; out = sl.q;}
          }
        }
        return best < 1e8;
      };
    Q5 up5;
    if (!lift_q(0.05, up5)) {why = "세우기: 세운 뒤 손을 5 cm 들 자세 (IK) 가 없음"; return {};}
    path.nodes.push_back(model_node("stand_up", q_to_ticks(model_, encoder_goal_q(model_, up5, 0.0)), slow_dps_));
    if (!get_parameter("standup_regrasp").as_bool()) {   // 선 퍽은 박스에 두고 LOOK 자세에서 끝 → 다음 pick det
      const auto look = paths_.find("look");
      if (look == paths_.end() || look->second.nodes.empty()) {why = "세우기: LOOK 경로 없음"; return {};}
      Q5 up12;
      if (lift_q(0.12, up12)) {path.nodes.push_back(model_node("stand_high", q_to_ticks(model_, encoder_goal_q(model_, up12, 0.0)), 0.0));}
      PathNode lk = look->second.nodes.back();
      lk.name = "look";
      lk.grip = "";
      path.nodes.push_back(lk);
      std::string look_why;
      if (!path_start_ok(path, joint_buf_, path_start_tol_, look_why) && !start_at_look(path, "PICK") && !join_from_p0(path, "PICK") &&
        !join_from_look(path, "PICK")) {
        why = look_why + " — 받침(HOME) 또는 LOOK 자세에서 시작해야 함";
        return {};
      }
      pending_tilt_ = 0.0;
      pending_depth_ = 0.0;
      pending_lying_ = false;
      pending_empty_ = true;   // 성공해도 퍽을 안 쥠
      RCLCPP_INFO(get_logger(), "PICK 세우기: 누운 퍽 (%.3f, %.3f) 긴축 %.0f° → 표 %d (%.3f, %.3f) %.0f° · 한 변 %.1f mm · 차 (%+.1f, %+.1f) mm %+.1f° "
        "만큼 경로 전체를 옮김 (기울인 손 방향 어긋남 최대 %.1f°) · 경로 %zu 점 (%zu 째에서 엶) → 퍽은 시뮬 기준 (%.3f, %.3f) 쯤에 섬 · "
        "LOOK 자세에서 끝 (다음: pick det)", x, y, axis, k, e.x, e.y, e.axis_deg, e.side * 1000.0, dx * 1000.0, dy * 1000.0, da, app_err_max,
        e.q.size(), static_cast<size_t>(e.release), end_xy.x(), end_xy.y());
      return path_actions(path, timing_, grip_, joint_buf_);
    }
    // 선 퍽 다시 쥐기: 시뮬에서 선 자리를 실제 퍽으로 옮긴 곳
    const double sx = end_xy.x(), sy = end_xy.y();
    const double top_stand = top - lying_h_ + puck_h_;
    // 시뮬 정책은 퍽을 박스 왼쪽 앞 (x 0.25 ~ 0.38) 으로 옮겨 세움 → 수직 손이 안 닿으면 보통 PICK 처럼 기울여 쥐고 가까이 세워 다시 쥠
    std::string kw, w;
    const std::vector<std::pair<double, std::string>> cands{{std::nan(""), "open"}};
    auto P = poses_for(sx, sy, top_stand, end_face, pick_heights_, cands, kw, w, 0.0, kStoreMaxTilt);
    double depth = -pick_heights_.back();
    if (P.size() != 3 && max_tilt_deg_ >= kStoreMaxTilt + 10.0 - 1e-6) {
      auto h = pick_heights_;
      h.back() = -tilted_depth_mm_;
      std::string w2;
      P = poses_for(sx, sy, top_stand, end_face, h, cands, kw, w2, kStoreMaxTilt + 10.0, max_tilt_deg_ - kStoreMaxTilt - 10.0);
      depth = tilted_depth_mm_;
      if (P.size() != 3) {w += " · 기울인 손 (" + w2 + ")";}
    }
    if (P.size() != 3) {why = "세우기: 선 퍽을 다시 쥘 자세가 없음 (" + w + ")"; return {};}
    path.nodes.push_back(model_node("re_above", P[0].enc, slow_dps_, "open"));
    path.nodes.push_back(model_node("re_down", P[1].enc, slow_dps_));
    path.nodes.push_back(model_node("re_grasp", P[2].enc, slow_dps_, "close"));
    path.nodes.push_back(model_node("re_up", P[1].enc, slow_dps_));
    path.nodes.push_back(model_node("re_lift", P[0].enc, slow_dps_));
    const double re_tilt = P.back().tilt_deg;
    if (re_tilt > 0.5) {
      std::vector<PathNode> again;
      double gx = 0.0, gy = 0.0;
      if (!regrasp_nodes(sx, sy, top_stand, re_tilt, depth, again, gx, gy, w)) {why = "세우기: " + w; return {};}
      path.nodes.insert(path.nodes.end(), again.begin(), again.end());
      RCLCPP_INFO(get_logger(), "PICK 세우기: 선 퍽을 손 %.0f° 기울여 쥐어야 함 → (%.3f, %.3f) 에 세워 내려놓고 수직 손으로 다시 쥠", re_tilt, gx, gy);
    }
    path.nodes.push_back({"staging", staging_, false, 0.0, "", false});
    std::string look_why;
    if (!path_start_ok(path, joint_buf_, path_start_tol_, look_why) && !start_at_look(path, "PICK") && !join_from_p0(path, "PICK") &&
      !join_from_look(path, "PICK")) {
      why = look_why + " — 받침(HOME) 또는 LOOK 자세에서 시작해야 함";
      return {};
    }
    pending_tilt_ = 0.0;
    pending_depth_ = -pick_heights_.back();
    pending_lying_ = false;   // 세운 퍽을 쥐고 끝남
    pending_empty_ = false;
    RCLCPP_INFO(get_logger(), "PICK 세우기: 누운 퍽 (%.3f, %.3f) 긴축 %.0f° → 표 %d (%.3f, %.3f) %.0f° · 한 변 %.1f mm · 차 (%+.1f, %+.1f) mm %+.1f° · "
      "시뮬 성공률 %.2f · 경로 %zu 점 (%zu 째에서 엶) → 선 퍽 (%.3f, %.3f) 면 %.0f° 다시 쥠", x, y, axis, k, e.x, e.y, e.axis_deg,
      e.side * 1000.0, dx * 1000.0, dy * 1000.0, da, e.rate, e.q.size(), static_cast<size_t>(e.release), sx, sy, end_face);
    return path_actions(path, timing_, grip_, joint_buf_);
  }

  // 누운 네모 퍽 세우기 — 그리퍼 받침 (stand_lying_mode park). 자세 계산은 park_stand.cpp (시뮬 도구 park_cli 와 같은 코드)
  //   1) 누운 퍽을 중심에서 한쪽 끝으로 park_grasp_offset_mm 옮겨 쥠 (반지름 쪽 퍽 = 바깥 끝, 접선 쪽 = J1 쪽 끝. 긴 쪽 = 받침 안으로). 수직 손이 안 닿으면 손을
  //      10 · 20 · 30° (park_max_tilt_deg) 바깥으로 기울임 — 닫힘축 = 손 축 × 긴축. 긴축이 접선이면 퍽이 긴축 둘레로 굴러 손가락에 맞음
  //   2) 받침 바깥 위 수직 → 손을 기울이며 (팔 평면 안, 닫힘축 = 접선) 긴 쪽 끝이 J1 쪽 아래로 park_tilt_deg (안 되면 40) 가 되게
  //      받침 바닥 위로 → 놓기 → 퍽은 받침 벽에 기댐. 쥘 때 손을 기울였으면 그만큼 손 기울임을 바꿔 퍽 기울기를 같게
  //   3) 손 빼고 오므려 받침 바깥에서 받침 바닥 위 park_push_h_mm 높이로 손끝을 안쪽으로 밀어 퍽이 서는 자리 앞에서 멈춤 → 섬
  //   4) 수직 손으로 선 퍽 쥐기 (윗면 − park_regrasp_depth_mm, 손끝이 받침 벽 위) → 박스 위 기준점 (선 퍽을 쥔 채, 바로 STORE)
  //   자세는 모두 실제 자세 IK → 처짐 앞먹임 (쥔 동안 퍽 무게). 안 풀리면 거절 (팔이 못 가는 자세)
  std::vector<ArmAction> park_plan(const ArmPath & unfold, double x, double y, double top,
    const std::vector<std::pair<double, std::string>> & cands, std::string & open_kw, std::string & why)
  {
    const double axis = lying_axis_deg_;
    if (!std::isfinite(axis)) {why = "받침 세우기: 누운 퍽 긴축 방향을 모름"; return {};}
    const auto hl = get_parameter("pick_lying_heights_mm").as_double_array();
    if (hl.size() != 3) {why = "pick_lying_heights_mm 는 3 개"; return {};}
    const auto pc = get_parameter("park_center").as_double_array();
    if (pc.size() != 2) {why = "park_center 는 [x, y]"; return {};}
    const double v = get_parameter("park_speed_dps").as_double();
    const double v_push = get_parameter("park_push_speed_dps").as_double();
    LyingGraspCfg gc;
    gc.side = lying_h_;
    gc.length = puck_h_;
    gc.grip_offset = get_parameter("park_grasp_offset_mm").as_double() / 1000.0;
    gc.open_inner = FingerTip{}.inner0 + grip_.open;
    gc.open_outer = FingerTip{}.outer0 + grip_.open;
    gc.min_margin = get_parameter("park_roll_margin_mm").as_double() / 1000.0;
    gc.max_roll_deg = get_parameter("park_max_roll_deg").as_double();
    open_kw = cands.empty() ? "open" : cands.front().second;
    ParkCfg pk;
    pk.center = Eigen::Vector2d(pc[0], pc[1]);
    pk.floor_z = get_parameter("park_floor_z").as_double() + model_.team_base_z_floor - model_.base_z_floor;   // 모델 높이 (target_in_model 과 같은 변환)
    pk.elev_deg = get_parameter("park_tilt_deg").as_double();
    pk.min_elev_deg = std::min(40.0, pk.elev_deg);
    pk.insert_clear = get_parameter("park_insert_clear_mm").as_double() / 1000.0;
    pk.end_inset = get_parameter("park_end_inset_mm").as_double() / 1000.0;
    pk.push_h = get_parameter("park_push_h_mm").as_double() / 1000.0;
    pk.regrasp_depth = get_parameter("park_regrasp_depth_mm").as_double() / 1000.0;
    pk.regrasp_r = get_parameter("park_regrasp_r_mm").as_double() / 1000.0;
    pk.insert_dj3 = get_parameter("park_insert_j3_deg").as_double() * M_PI / 180.0;
    pk.loose_step = static_cast<int>(get_parameter("park_loose_step").as_int());
    pk.insert_back = get_parameter("park_insert_back_mm").as_double() / 1000.0;
    pk.insert_extra = get_parameter("park_insert_extra_mm").as_double() / 1000.0;
    pk.insert_from_back = get_parameter("park_insert_from_back_mm").as_double() / 1000.0;
    pk.release_pull = get_parameter("park_release_pull_mm").as_double() / 1000.0;
    pk.release_pull_steps = static_cast<int>(get_parameter("park_release_pull_steps").as_int());
    pk.release_pull_dz = get_parameter("park_release_pull_dz_mm").as_double() / 1000.0;
    pk.release_up = get_parameter("park_release_up_mm").as_double() / 1000.0;
    pk.push_extra = get_parameter("park_push_extra_mm").as_double() / 1000.0;
    pk.push_scoop_deg = get_parameter("park_push_scoop_deg").as_double();
    pk.push_dj3 = get_parameter("park_push_j3_deg").as_double() * M_PI / 180.0;
    pk.push_dj2 = get_parameter("park_push_j2_deg").as_double() * M_PI / 180.0;
    pk.press_mode = get_parameter("park_press_mode").as_bool();
    pk.press_lift = get_parameter("park_press_lift_mm").as_double() / 1000.0;
    pk.press_back = get_parameter("park_press_back_mm").as_double() / 1000.0;
    pk.press_depth = get_parameter("park_press_depth_mm").as_double() / 1000.0;
    pk.push_low = get_parameter("park_push_low_mm").as_double() / 1000.0;
    grip_.loose = get_parameter("park_loose_grip_m").as_double();
    pk.side = lying_h_;
    pk.length = puck_h_;
    std::vector<ParkNode> pn;
    double elev = 0.0;
    // 쥐기 후보마다: 이웃 퍽 ↔ 손가락 (위에서 본 발자국, 기울인 손은 손가락이 기운 만큼 넓게) · 그 쥐기로 받침 자세까지 되는지
    //   (11-06 실물: 첫 후보만 받침 자세를 풀어 park_over IK 안 됨 → 거절. 다른 끝 · 닫힘 부호 후보는 됨)
    auto fingers_clear = [&](const LyingGrasp & g, std::string & w) {
        if (last_nb_.empty() || g.tilt_deg <= 0.0) {return true;}
        FingerTip f;
        const double dz = std::max(0.0, -hl.back() / 1000.0);
        const double e = dz * std::tan(g.tilt_deg * M_PI / 180.0);
        f.w_lo -= e;
        f.w_hi += e;
        f.outer0 += dz * std::tan(g.roll_deg * M_PI / 180.0);
        const double ch = std::atan2(g.close.y(), g.close.x());
        const double clr = finger_clearance(g.tip.head<2>(), ch, open_kw == "open" ? grip_.open : grip_.release, last_nb_, f);
        if (clr >= nb_min_clear_) {return true;}
        char buf[160];
        std::snprintf(buf, sizeof(buf), "손 %.0f°: 이웃 퍽과 여유 %.1f mm", g.tilt_deg, clr * 1000.0);
        w = buf;
        return false;
      };
    auto accept = [&](const LyingGrasp & g, std::string & w) {
        return fingers_clear(g, w) && park_nodes(model_, pk, g, ticks_to_q(model_, g.poses[0].ticks), pn, elev, w);
      };
    LyingGrasp lg;
    std::string w;
    if (!lying_grasp(model_, x, y, top, axis, hl, gc, lg, w, get_parameter("park_max_tilt_deg").as_double(), accept)) {
      why = "받침 세우기: 쥐기 후보가 다 안 됨 (마지막: " + w + ")";
      return {};
    }
    std::vector<PathNode> nodes;
    for (const auto & n : pn) {
      // 미는 구간 (park_push_down · 1–4 · back) 만 park_push_speed_dps (11-06 시뮬: 50 °/s 면 기대는 각 49° 아래에서 넘어짐, 20 °/s 면 ±2–4 mm 오차에도 섬)
      const double sp = !n.slow ? 0.0 : (n.name.rfind("park_push", 0) == 0 || n.name.rfind("park_pull", 0) == 0 ? v_push : v);
      nodes.push_back(model_node(n.name, q_to_ticks(model_, encoder_goal_q(model_, n.q, n.held_kg)), sp, n.grip));
      if (n.grip == "loose") {nodes.back().grip_on_way = true;}   // 팔을 안 멈추고 손가락만 풂
    }
    pending_tilt_ = 0.0;
    pending_depth_ = pk.regrasp_depth * 1000.0;
    pending_lying_ = false;   // 세운 퍽을 쥐고 끝남
    pending_empty_ = false;
    RCLCPP_INFO(get_logger(), "PICK 받침 세우기: 누운 퍽 (%.3f, %.3f) 긴축 %.0f° → 끝 쪽 %.0f mm 를 손 %.0f° (퍽 굴림 %.0f° · 손 안 기울기 %.0f° · 쥐는 깊이 %.0f mm · "
      "손가락 여유 %.1f mm) 로 쥐고 받침에 %.0f° 로 넣어 놓고, 받침 바닥 위 %.0f mm 로 밀어 세운 뒤 다시 쥠 (윗면 − %.0f mm) → 기준점", x, y, axis,
      gc.grip_offset * 1000.0, lg.tilt_deg, lg.roll_deg, lg.pitch_deg, lg.depth_mm, lg.margin_mm, elev, pk.push_h * 1000.0, pk.regrasp_depth * 1000.0);
    pending_step_ = get_parameter("park_step").as_bool();
    return pick_path(unfold, lg.poses, open_kw, nodes, why, lg.tip.x(), lg.tip.y(), axis, true);
  }

  // PICK 경로: 펼치기 → 퍽 위 (열기) → 내림 → 쥐기 (닫기) → 올림 → 들기 (P 세 자세) → 다시 잡기 (again) → 박스 위 기준점.
  //   받침 자세가 아니면 LOOK 자세 · P0 · LOOK 복귀 경로에서 이음
  std::vector<ArmAction> pick_path(const ArmPath & unfold, const std::vector<PickPose> & P, const std::string & open_kw,
    const std::vector<PathNode> & again, std::string & why, double x, double y, double yaw, bool lying)
  {
    ArmPath path = unfold;
    path.name = "pick";
    path.ends_home = false;
    path.nodes.push_back(model_node("above", P[0].enc, 0.0, open_kw));
    path.nodes.back().grip_early = side_grip_;   // 퍽 위로 가며 미리 엶 (손이 비어 있음) — 그리퍼 기다림 하나를 뺌
    path.nodes.push_back(model_node("down", P[1].enc, slow_dps_));
    path.nodes.push_back(model_node("grasp", P[2].enc, slow_dps_, "close"));
    path.nodes.push_back(model_node("up", P[1].enc, slow_dps_));
    path.nodes.push_back(model_node("lift", P[0].enc, slow_dps_));
    path.nodes.insert(path.nodes.end(), again.begin(), again.end());
    // 박스 위 기준점: 11-06 사용자 — 집은 뒤 왼쪽 (기준점 y +38 mm) 으로 갈 이유 없음 → 기본은 안 감 (pick_end_at_staging false).
    //   STORE · PLACE 가 집은 자리에서 바로 이어 감 (STORE = 칸 경로의 실제 시작점 lift_real 로, store_join_from_pick)
    if (get_parameter("pick_end_at_staging").as_bool()) {
      path.nodes.push_back({"staging", staging_, false, 0.0, "", false});   // 퍽을 40 mm 띄운 뒤는 빈 공간 → 보통 속도 (STORE 경로도 쥔 채 그 속도)
    }
    std::string look_why;
    if (!path_start_ok(path, joint_buf_, path_start_tol_, look_why) && !start_at_look(path, "PICK") && !join_from_p0(path, "PICK") &&
        !join_from_look(path, "PICK")) {
      why = look_why + " — 받침(HOME) 또는 LOOK 자세에서 시작해야 함";
      return {};
    }
    RCLCPP_INFO(get_logger(), "PICK: %s (%.3f, %.3f) %s %.1f° · 손 기울임 %.0f° · 쥐는 깊이 %.0f mm → 쥐기 틱 [%d %d %d %d %d]",
      lying ? "누운 퍽" : "퍽", x, y, lying ? "긴축" : "돌림", yaw, pending_tilt_, pending_depth_,
      P[2].ticks[0], P[2].ticks[1], P[2].ticks[2], P[2].ticks[3], P[2].ticks[4]);
    return path_actions(path, timing_, grip_, joint_buf_);
  }

  std::vector<ArmAction> place_plan(const ArmTask::Goal & goal, std::string & why)
  {
    const auto fold = paths_.find("fold");
    if (!model_ok_ || fold == paths_.end() || staging_.size() != 5) {why = "팔 모델 · 접기 경로 · 기준점 설정 없음"; return {};}
    double x, y, top, yaw;
    bool lying = false;
    if (!target_in_model(goal, held_lying_, x, y, top, yaw, lying, why)) {return {};}
    if (lying && !held_lying_) {why = "누운 목표인데 쥔 퍽은 선 퍽 — 선 퍽은 세워서만 놓음"; return {};}
    std::vector<std::pair<double, std::string>> cands;
    std::string open_kw;
    const double zc = top - model_.team_base_z_floor + model_.base_z_floor - (held_lying_ ? lying_h_ : puck_h_) / 2.0;
    if (!grasp_for(x, y, zc, yaw, true, cands, why, "PLACE", held_lying_)) {return {};}
    auto heights = place_heights_;
    const bool depth_known = heights.size() == 3 && held_depth_mm_ > 0.0;
    if (depth_known) {heights[2] = drop_height_mm(held_tilt_);}   // 쥘 때 기울임 그대로 놓으면 퍽은 수직
    // 누운 퍽: 수직 손 그대로 (쥘 때 수직), 놓기 높이는 누운 높이 기준 (drop_height_mm) → 퍽 아랫면이 놓을 면 위 place_bottom_clear_mm
    auto P = poses_for(x, y, top, yaw, heights, cands, open_kw, why, held_tilt_, 20.0);   // 놓는 퍽 기울임 ≤ 20° (누운 퍽도, 쥘 때 기울임 근처)
    if (P.size() != 3 && held_lying_) {
      // 누운 퍽은 수직 손으로만 놓음 → 그 자리가 수직 손 끝자리면 (11-05 실물: (0.250, −0.028) 위 40 mm IK 안 됨, 같은 자리 PICK 은 됨)
      //   J1 쪽으로 5 mm 씩 25 mm 까지 당겨 닿는 가장 가까운 자리에 놓음 (빈 자리 · 손가락 여유는 grasp_for 로 다시)
      const Eigen::Vector2d j1 = model_.j1_xy, p0(x, y);
      const Eigen::Vector2d dir = (p0 - j1).normalized();
      std::string w = why;
      for (int mm = 5; mm <= 25 && P.size() != 3; mm += 5) {
        double nx = x - dir.x() * mm / 1000.0, ny = y - dir.y() * mm / 1000.0;
        std::vector<std::pair<double, std::string>> c2;
        std::string w2;
        if (!grasp_for(nx, ny, zc, yaw, true, c2, w2, "PLACE", held_lying_)) {w = w2; continue;}
        P = poses_for(nx, ny, top, yaw, heights, c2, open_kw, w2, held_tilt_, 20.0);
        if (P.size() == 3) {
          RCLCPP_INFO(get_logger(), "PLACE: 누운 퍽 — (%.3f, %.3f) 는 수직 손이 안 닿아 J1 쪽으로 %d mm 당긴 (%.3f, %.3f) 에 놓음", x, y, mm, nx, ny);
          x = nx;
          y = ny;
          cands = c2;
        } else {
          w = w2;
        }
      }
      if (P.size() != 3) {why = w;}
    }
    if (P.size() != 3) {
      if (held_lying_) {why = "누운 퍽 놓기: 수직 손이 안 닿음 (J1 쪽으로 25 mm 까지 당겨도, " + why + ")"; return {};}
      if (get_parameter("place_far_regrasp").as_bool()) {return place_far_plan(x, y, top, yaw, cands, why);}
      return {};
    }
    if (depth_known && std::abs(P.back().tilt_deg - held_tilt_) > 0.5) {   // 다른 손 기울임으로 풀림 → 퍽이 기울어 모서리가 낮아짐 → 그 기울임으로 다시
      const double t = P.back().tilt_deg;
      heights[2] = drop_height_mm(t);
      P = poses_for(x, y, top, yaw, heights, cands, open_kw, why, t, 0.0);
      if (P.size() != 3) {why = "놓는 손 기울임 " + std::to_string(static_cast<int>(t)) + "° 의 놓기 높이로 다시 풀면 IK 안 됨: " + why; return {};}
    }
    RCLCPP_INFO(get_logger(), "PLACE: %s(%.3f, %.3f) 쥔 깊이 %.0f mm · 쥘 때 기울임 %.0f° · 놓는 손 %.0f° → 놓기 높이 %.1f mm", held_lying_ ? "누운 퍽 " : "",
      x, y, held_depth_mm_, held_tilt_, P.back().tilt_deg, heights.back());
    ArmPath path;
    path.name = "place";
    path.ends_home = true;
    path.nodes.push_back({"staging", staging_, false, 0.0, "", false});
    path.nodes.push_back(model_node("above", P[0].enc, slow_dps_));
    join_from_pick_end(path, "PLACE");
    path.nodes.push_back(model_node("down", P[1].enc, slow_dps_));
    path.nodes.push_back(model_node("drop", P[2].enc, slow_dps_, open_kw));
    path.nodes.push_back(model_node("up", P[1].enc, slow_dps_));
    path.nodes.push_back(model_node("clear", P[0].enc, slow_dps_, "close"));
    path.nodes.back().grip_on_way = side_grip_;   // 놓은 뒤 빈손 → 멈추지 않고 닫으며 P0 로
    for (const auto & n : fold->second.nodes) {path.nodes.push_back(n);}
    if (cut_fold_tail(path)) {
      plan_ends_p0_ = true;
      RCLCPP_INFO(get_logger(), "PLACE: P0 에서 멈춤 (다음 작업이 %.0f s 안에 안 오면 접음, 주행 전 HOME 이 접음)", auto_fold_s_);
    }
    if (!path_start_ok(path, joint_buf_, path_start_tol_, why)) {why += " — 퍽을 쥐고 박스 위 기준점에 있어야 함 (PICK · RETRIEVE 뒤)"; return {};}
    return path_actions(path, timing_, grip_, joint_buf_);
  }

  // 먼 자리 PLACE (11-06 사용자: 먼 줄은 박스에 한 번 내려놨다가 쥐는 방식을 바꿔서, 멀리 있는 퍽 집듯이): 쥘 때 기울임 ± 20° 로 안 닿으면
  //   ① 목표 쪽 빈 자리 S 에 지금 쥔 그대로 내려놓음 (손 기울임은 쥘 때 ± 20° 안 = 놓기 규칙, 퍽이 기울면 놓기 높이도 그만큼)
  //   ② S 에서 손을 t° 기울여 pick_tilted_depth_mm 로 다시 쥠 (닫힘축 = S 의 접선 = 내려놓을 때 손 방향 → 퍽 면과 맞음, 퍽은 수직)
  //   ③ 목표에 t° 손 그대로 놓음 (퍽 수직, 바닥이 면 위 place_bottom_clear_mm) → P0 · 접기
  //   t = 목표에 닿는 가장 작은 기울임 (10° 씩, pick_max_tilt_deg 까지). 40° 손은 x 0.28 m 넘어야 닿아서 S 는 목표 3 cm 앞부터 차체 쪽으로
  //   1 cm 씩 · 좌우 0 · ±4 · ±8 cm (박스 앞쪽 끝 regrasp_min_x 까지), 빈 자리 (grasp_for) · 목표와 6 cm 넘게. 먼 칸부터 놓아야 함 (planning order_far_first)
  std::vector<ArmAction> place_far_plan(double x, double y, double top, double yaw, const std::vector<std::pair<double, std::string>> & cands,
    std::string & why)
  {
    const auto fold = paths_.find("fold");
    const double H = puck_h_ * 1000.0;
    // ③ 목표: t° 로 쥔 퍽 (세상에서 수직) → 놓기 높이 = 바닥 여유 − 퍽 높이 + (퍽 높이 − 쥔 깊이)
    const std::vector<double> ht{place_heights_[0], place_heights_[1], place_clear_mm_ - H + (H - tilted_depth_mm_)};
    const auto t0 = std::chrono::steady_clock::now();
    std::vector<PickPose> Pt;
    std::string kw_t, w = "기울임 후보 없음";
    double t = 0.0;
    // 기울인 손은 닫힘축이 늘 접선 (pick_poses) → 닫힘 방향 후보는 하나면 됨 (벌림은 첫 후보 것)
    const std::vector<std::pair<double, std::string>> c1{cands.empty() ? std::make_pair(std::nan(""), std::string("open")) : cands.front()};
    for (double tt = 10.0; tt <= max_tilt_deg_ + 1e-6 && Pt.size() != 3; tt += 10.0) {
      if (std::fabs(tt - held_tilt_) <= 20.0 + 1e-6) {continue;}   // 쥘 때 ± 20° 는 그냥 놓기에서 이미 안 됨
      Pt = poses_for(x, y, top, yaw, ht, c1, kw_t, w, tt, 0.5);
      t = tt;
    }
    const double t_target = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    if (Pt.size() != 3) {
      why = "먼 자리 놓기: 손을 " + std::to_string(static_cast<int>(max_tilt_deg_)) + "° 까지 기울여도 안 닿음 (" + w + ")";
      return {};
    }
    const double zc = top - model_.team_base_z_floor + model_.base_z_floor - puck_h_ / 2.0;
    const Eigen::Vector2d j1 = model_.j1_xy, p0(x, y);
    const double r0 = (p0 - j1).norm();
    const Eigen::Vector2d dir = (p0 - j1) / r0, side(-dir.y(), dir.x());
    std::string last = "후보 없음";
    std::vector<PathNode> mid;
    double sx = 0.0, sy = 0.0, put_tilt = 0.0;
    // IK 씨앗: 목표 자세에서 (가까운 자리라 빠름). 그걸로 못 찾으면 기본 씨앗으로 한 번 더 (느림, 계획 수 초)
    const Q5 q_target = ticks_to_q(model_, Pt[2].ticks);
    for (int pass = 0; pass < 2 && mid.empty(); ++pass) {
      const std::vector<Q5> seed_t = pass == 0 ? std::vector<Q5>{q_target} : std::vector<Q5>{};
      for (double r = r0 - 0.03; mid.empty() && j1.x() + dir.x() * r >= regrasp_min_x_ - 0.05; r -= 0.01) {
        for (double lat : {0.0, 0.04, -0.04, 0.08, -0.08}) {
          const Eigen::Vector2d q = j1 + dir * r + side * lat;
          if (q.x() < regrasp_min_x_ || (q - p0).norm() < 0.06) {continue;}
          std::vector<std::pair<double, std::string>> cs;
          double gx = q.x(), gy = q.y();
          if (!grasp_for(gx, gy, zc, std::nan(""), true, cs, w, "먼 자리 놓기: 내려놓을 자리")) {last = w; continue;}   // 빈 자리 · 손가락 여유
          const double c_deg = (std::atan2(q.y() - j1.y(), q.x() - j1.x()) + M_PI / 2.0) * 180.0 / M_PI;
          std::string kw;
          // ② t° 로 다시 쥐기 (먼저 — 40° 손은 닿는 곳이 좁아 여기서 많이 걸러짐)
          const auto Pr = poses_for(q.x(), q.y(), top, c_deg, {pick_heights_[0], pick_heights_[1], -tilted_depth_mm_}, {{c_deg, "open"}}, kw, w, t, 0.5,
            seed_t);
          if (Pr.size() != 3) {last = std::to_string(static_cast<int>(t)) + "° 로 다시 쥐기: " + w; continue;}
          // ① 내려놓기: 쥘 때 기울임 ± 20°, 다른 기울임이면 그 기울임으로 놓기 높이를 다시 (place_plan 과 같음)
          const std::vector<Q5> seed_p = pass == 0 ? std::vector<Q5>{ticks_to_q(model_, Pr[2].ticks)} : std::vector<Q5>{};
          std::vector<double> hp{place_heights_[0], place_heights_[1], drop_height_mm(held_tilt_)};
          auto Pp = poses_for(q.x(), q.y(), top, c_deg, hp, {{c_deg, "release"}}, kw, w, held_tilt_, 20.0, seed_p);
          if (Pp.size() == 3 && std::abs(Pp.back().tilt_deg - held_tilt_) > 0.5) {
            hp[2] = drop_height_mm(Pp.back().tilt_deg);
            Pp = poses_for(q.x(), q.y(), top, c_deg, hp, {{c_deg, "release"}}, kw, w, Pp.back().tilt_deg, 0.5, seed_p);
          }
          if (Pp.size() != 3) {last = "내려놓기: " + w; continue;}
          mid = {model_node("far_put_above", Pp[0].enc, slow_dps_), model_node("far_put_down", Pp[1].enc, slow_dps_),
            model_node("far_put", Pp[2].enc, slow_dps_, "release"), model_node("far_put_up", Pp[1].enc, slow_dps_),
            model_node("far_put_clear", Pp[0].enc, slow_dps_), model_node("far_re_above", Pr[0].enc, slow_dps_, "open"),
            model_node("far_re_down", Pr[1].enc, slow_dps_), model_node("far_re_grasp", Pr[2].enc, slow_dps_, "close"),
            model_node("far_re_up", Pr[1].enc, slow_dps_), model_node("far_re_lift", Pr[0].enc, slow_dps_)};
          sx = q.x();
          sy = q.y();
          put_tilt = Pp.back().tilt_deg;
          break;
        }
      }
    }
    if (mid.empty()) {why = "먼 자리 놓기: 손 " + std::to_string(static_cast<int>(t)) + "° 로 다시 쥘 빈 자리가 없음 (마지막: " + last + ")"; return {};}
    RCLCPP_INFO(get_logger(), "PLACE 먼 자리 (%.3f, %.3f): 손 %.0f° 로 놓아야 함 → (%.3f, %.3f) 에 손 %.0f° 로 내려놓고 %.0f° 로 다시 쥠 (깊이 %.0f mm) → 목표에 %.0f° 로 놓음 "
      "(계산 목표 %.2f s · 자리 %.2f s)", x, y, t, sx, sy, put_tilt, t, tilted_depth_mm_, t, t_target,
      std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() - t_target);
    ArmPath path;
    path.name = "place_far";
    path.ends_home = true;
    path.nodes.push_back({"staging", staging_, false, 0.0, "", false});
    path.nodes.insert(path.nodes.end(), mid.begin(), mid.end());
    join_from_pick_end(path, "PLACE");
    path.nodes.push_back(model_node("above", Pt[0].enc, slow_dps_));
    path.nodes.push_back(model_node("down", Pt[1].enc, slow_dps_));
    path.nodes.push_back(model_node("drop", Pt[2].enc, slow_dps_, kw_t));
    path.nodes.push_back(model_node("up", Pt[1].enc, slow_dps_));
    path.nodes.push_back(model_node("clear", Pt[0].enc, slow_dps_, "close"));
    path.nodes.back().grip_on_way = side_grip_;
    for (const auto & n : fold->second.nodes) {path.nodes.push_back(n);}
    if (cut_fold_tail(path)) {plan_ends_p0_ = true;}
    if (!path_start_ok(path, joint_buf_, path_start_tol_, why)) {why += " — 퍽을 쥐고 박스 위 기준점에 있어야 함 (PICK · RETRIEVE 뒤)"; return {};}
    why.clear();
    return path_actions(path, timing_, grip_, joint_buf_);
  }

  // 놓기 높이 [mm, 퍽 윗면 기준 손끝]: 손 기울임 place_tilt 로 놓으면 퍽이 세상에서 Δ = place_tilt − held_tilt_ 기울어짐.
  //   손끝은 퍽 축 위 → 가장 낮은 모서리 = 손끝 − (퍽 높이 − 쥔 깊이)·cosΔ − 반폭·sin|Δ| 가 놓을 면 위 place_clear_mm_ 가 되게 (반폭 17.4 mm)
  //   누운 퍽은 퍽 높이 = 누운 높이 (Δ = 0: 손끝 = 윗면 − 쥔 깊이 20 → place_clear − 20 = −18)
  double drop_height_mm(double place_tilt) const
  {
    const double d = std::abs(place_tilt - held_tilt_) * M_PI / 180.0;
    const double H = (held_lying_ ? lying_h_ : puck_h_) * 1000.0;
    return place_clear_mm_ - H + (H - held_depth_mm_) * std::cos(d) + 17.4 * std::sin(d);   // 반폭: 네모 퍽 면이 10° 틀어져도 (15·(cos10°+sin10°))
  }

  // 팔 기구학 읽기: arm_model.* (config/arm_kinematics.yaml)
  bool load_model()
  {
    if (load_arm_model(*this, model_)) {return true;}
    RCLCPP_WARN(get_logger(), "팔 기구학(arm_model) 설정 없음 · 크기 다름 → PICK · PLACE 거절");
    return false;
  }

  // 경로 묶음 읽기: arm_paths.names 마다 arm_paths.<이름>.{nodes, q(관절 5 개 × 점), stand, speed, grip, stop, ends_home}, hold (없어도 됨)
  void load_paths()
  {
    auto names = declare_parameter<std::vector<std::string>>("arm_paths.names", std::vector<std::string>{});
    {   // LOOK_RACK 경로 (config/rack_look_paths.yaml, tools/arm_real/rack_look/gen_rack_look.py)
      const auto extra = declare_parameter<std::vector<std::string>>("arm_paths.look_rack_names", std::vector<std::string>{});
      names.insert(names.end(), extra.begin(), extra.end());
    }
    const std::size_t nj = arm_joints_.size();
    for (const auto & n : names) {
      if (paths_.count(n)) {RCLCPP_ERROR(get_logger(), "경로 %s: 이름이 두 번 → 두 번째는 안 씀", n.c_str()); continue;}
      const std::string k = "arm_paths." + n + ".";
      const auto ids = declare_parameter<std::vector<std::string>>(k + "nodes", std::vector<std::string>{});
      const auto q = declare_parameter<std::vector<double>>(k + "q", std::vector<double>{});
      const auto stand = declare_parameter<std::vector<bool>>(k + "stand", std::vector<bool>{});
      const auto speed = declare_parameter<std::vector<double>>(k + "speed", std::vector<double>{});
      const auto grip = declare_parameter<std::vector<std::string>>(k + "grip", std::vector<std::string>{});
      const auto stop = declare_parameter<std::vector<bool>>(k + "stop", std::vector<bool>{});
      auto hold = declare_parameter<std::vector<double>>(k + "hold", std::vector<double>{});
      const std::size_t m = ids.size();
      if (hold.empty()) {hold.assign(m, 0.0);}
      if (m < 2 || q.size() != m * nj || stand.size() != m || speed.size() != m || grip.size() != m || stop.size() != m || hold.size() != m) {
        RCLCPP_ERROR(get_logger(), "경로 %s: 점 수가 맞지 않음 (nodes %zu, q %zu) → 안 씀", n.c_str(), m, q.size());
        continue;
      }
      bool finite = true;
      for (double v : q) {finite = finite && std::isfinite(v);}
      for (double v : speed) {finite = finite && std::isfinite(v) && v >= 0.0;}
      for (double v : hold) {finite = finite && std::isfinite(v) && v >= 0.0 && v <= 10.0;}
      bool grip_ok = true;
      for (const auto & g : grip) {grip_ok = grip_ok && (g == "-" || g == "open" || g == "release" || g == "close");}
      if (!finite || !grip_ok) {
        RCLCPP_ERROR(get_logger(), "경로 %s: q · speed · hold 에 NaN · 음수 (hold 는 10 s 까지) 또는 모르는 grip → 안 씀", n.c_str());
        continue;
      }
      ArmPath p;
      p.name = n;
      p.ends_home = declare_parameter<bool>(k + "ends_home", false);
      for (std::size_t i = 0; i < m; ++i) {
        const std::string g = grip[i] == "-" ? "" : grip[i];
        p.nodes.push_back({ids[i], std::vector<double>(q.begin() + i * nj, q.begin() + (i + 1) * nj), stand[i], speed[i], g, stop[i], hold[i]});
      }
      paths_[n] = p;
    }
    RCLCPP_INFO(get_logger(), "실물 경로 %zu 개", paths_.size());
  }

  // ======================================================================== 50 Hz 본체
  void step()
  {
    const rclcpp::Time t = now();
    // ⓪ 하드웨어 이상 → STOP + 진행 중 팔 작업 실패 (HOME 확인 무효). 정상으로 돌아와도 여기서 DRIVE 로 안 감
    const bool hw_ok = t.seconds() < hw_grace_until_ ? hw_ok_before_ : hw_->healthy(t.seconds());   // 계획 직후 잠깐은 이전 판정 (on_arm_goal)
    if (!hw_ok) {
      if (hw_ok_before_) {
        RCLCPP_ERROR(get_logger(), "하드웨어 이상: %s → STOP, HOME 확인 무효 (복구 후 HOME 필요)",
          hw_->status(t.seconds()).c_str());
      }
      if (arbiter_.turn() != Turn::STOP) {
        arbiter_.force_stop();
        publish_turn();
      }
      runner_.fail_now("하드웨어 이상 (" + hw_->status(t.seconds()) + ")");
    } else if (!hw_ok_before_ && have_hw_once_) {
      RCLCPP_WARN(get_logger(), "하드웨어 측정 돌아옴 — 바퀴는 잠금 유지, HOME 이 성공해야 DRIVE");
    }
    have_hw_once_ = have_hw_once_ || hw_ok;
    hw_ok_before_ = hw_ok;
    // HOME confirmation is a continuous drive permit. An out-of-range, missing
    // or stale arm measurement invalidates it even after a successful HOME.
    if (arbiter_.enforce_drive_guard(fold_->confirmed(t.seconds()), hw_ok)) {
      RCLCPP_ERROR(get_logger(), "주행 중 접힘 확인 상실: %s → STOP (HOME 재확인 필요)",
        fold_->status(t.seconds()).c_str());
      publish_turn();
    }
    // ① 차례: 측정 시각이 odom_timeout_s 안인 /odom 이 "멈춤" 이고, 바퀴 쪽이 이번 번호로 잠갔다고 답했을 때만
    const bool odom_fresh = have_odom_ && std::fabs((t - odom_stamp_).seconds()) < odom_timeout_s_;
    if (arbiter_.turn() == Turn::STOPPING) {
      if (!odom_fresh) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "/odom 이 안 옴(또는 옛날 것) → 몸통이 멈췄는지 몰라서 팔을 시작하지 않음");
      } else if (lock_acked_id_ != arbiter_.request_id()) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000, "바퀴 잠금 확인 기다리는 중 (요청 %u, 받은 답 %u)",
          arbiter_.request_id(), lock_acked_id_);
      }
    }
    const Turn before = arbiter_.turn();
    arbiter_.update(t.seconds(), odom_fresh && odom_still_, lock_acked_id_);
    // 준비 상태가 바뀌면 심장박동(10 Hz)을 기다리지 않고 바로 알림
    std::string why;
    const bool ready = task_ready(t.seconds(), why);
    if (arbiter_.turn() != before || ready != ready_sent_) {publish_turn();}

    // ② 팔
    run_arm(t);
  }

  // ======================================================================== 팔 동작 진행
  // lazy_fold: STORE · PLACE 가 P0 에서 끝난 뒤 lazy_fold_idle_s 동안 다음 팔 작업이 없으면 fold 경로로 스스로 접음
  //   (P0 에서 오래 버티면 팔 모터 과열). 접는 동안 ready 꺼짐 → planning 은 기다렸다 보냄. 접힌 뒤 HOME 은 home 한 점
  void maybe_auto_fold(double now_s)
  {
    if (p0_idle_since_ < 0.0) {return;}
    auto_fold_s_ = get_parameter("lazy_fold_idle_s").as_double();   // 실행 중 ros2 param set 도 바로
    if (now_s - p0_idle_since_ < auto_fold_s_) {return;}
    p0_idle_since_ = -1.0;
    std::string why;
    const auto fold = paths_.find("fold");
    if (!task_ready(now_s, why)) {
      RCLCPP_WARN(get_logger(), "자동 접기 안 함: %s", why.c_str());
      return;
    }
    if (fold == paths_.end() || !path_start_ok(fold->second, joint_buf_, path_start_tol_, why)) {
      RCLCPP_WARN(get_logger(), "자동 접기 안 함: 팔이 P0 가 아님 (%s)", why.c_str());
      return;
    }
    const auto plan = path_actions(fold->second, timing_, grip_, joint_buf_);
    if (plan.empty()) {return;}
    auto_folding_ = true;
    home_was_path_ = true;
    runner_.start(plan);
    arbiter_.request_arm();
    publish_turn();
    RCLCPP_INFO(get_logger(), "P0 에서 %.1f s 동안 다음 팔 작업 없음 → 스스로 접음 (동작 %zu개)", auto_fold_s_, plan.size());
  }

  // 판단은 ArmRunner(arm.cpp, test_arm.cpp)가 하고, 여기서는 그 명령을 ROS 로 옮기기만 함
  void run_arm(const rclcpp::Time & t)
  {
    if (!arm_goal_ && !auto_folding_) {
      maybe_auto_fold(t.seconds());
      return;
    }
    if (arm_goal_ && arm_goal_->is_canceling()) {runner_.request_cancel();}

    const double now_s = t.seconds();
    for (const ArmCommand & c : runner_.tick(now_s, arbiter_.arm_may_move(), fold_->confirmed(now_s))) {
      switch (c.kind) {
        case ArmCommand::Kind::SEND: {
          if (!c.action.is_gripper && standup_check_ && c.action.pose == standup_check_->pose) {
            std::string bad;
            const bool ok = standup_grasp_ok(bad);
            standup_check_.reset();
            if (!ok) {
              RCLCPP_ERROR(get_logger(), "세우기 멈춤: %s", bad.c_str());
              runner_.fail_now("세우기 멈춤: " + bad);
              break;   // 세우기 경로를 안 보냄
            }
          }
          if (arm_goal_) {
            auto fb = std::make_shared<ArmTask::Feedback>();
            fb->phase = c.action.phase;
            arm_goal_->publish_feedback(fb);
          }
          if (c.action.is_gripper) {
            send_trajectory(grip_client_, {gripper_joint_}, {{{c.action.gripper}, gripper_time_s_}}, c.gen);
          } else if (!c.action.points.empty()) {
            send_trajectory(arm_client_, arm_joints_, c.action.points, c.gen);
            if (c.action.side_grip_t >= 0.0) {   // 팔을 멈추지 않고 그 시각에 그리퍼만 (세우기 여는 점)
              const double g = c.action.side_grip;
              side_grip_timer_ = create_wall_timer(std::chrono::duration<double>(c.action.side_grip_t), [this, g]() {
                  side_grip_timer_->cancel();
                  send_side_gripper(g);
                });
            }
          } else {
            send_trajectory(arm_client_, arm_joints_, {{poses_.at(c.action.pose), move_time_s_}}, c.gen);
          }
          break;
        }
        case ArmCommand::Kind::CANCEL_ACTIVE:
          if (side_grip_timer_) {side_grip_timer_->cancel();}
          cancel_trajectory(c.gen);
          break;
        case ArmCommand::Kind::VERIFY_START:
          // 이 시점 이후의 측정만 인정 (궤적 도중 · 이전 작업의 측정으로 확인되지 않게)
          fold_->reset();
          RCLCPP_INFO(get_logger(), "home 동작 끝 → 관절 실측으로 접힘 확인 중");
          break;
        case ArmCommand::Kind::FINISH: {
          if (auto_folding_) {   // 스스로 접기 (goal 없음): 성공 + 접힘 확인이면 주행 허용, 아니면 잠금 유지 · 다음 HOME 이 복구
            auto_folding_ = false;
            arm_off_path_ = c.end != ArmEnd::SUCCEEDED;
            if (c.end != ArmEnd::SUCCEEDED) {RCLCPP_ERROR(get_logger(), "자동 접기 실패: %s (접힘 확인: %s)", c.message.c_str(), fold_->status(now_s).c_str());}
            arbiter_.arm_ended(c.drive_ok && hw_ok_before_);
            publish_turn();
            RCLCPP_INFO(get_logger(), "자동 접기 끝 (%s) → %s 차례", c.message.c_str(), turn_name(arbiter_.turn()));
            return;
          }
          auto result = std::make_shared<ArmTask::Result>();
          result->success = c.end == ArmEnd::SUCCEEDED;
          result->message = c.message;
          // 경로 작업 (HOME 의 fold · home_from_* 경로 포함) 이 도중에 끝나면 팔이 경로 중간일 수 있음 → HOME 직행 막음 (home_plan)
          const auto task = arm_goal_->get_goal()->task;
          const bool path_task = task != ArmTask::Goal::HOME || home_was_path_;
          arm_off_path_ = c.end != ArmEnd::SUCCEEDED && path_task
            ? true : (c.end == ArmEnd::SUCCEEDED ? false : arm_off_path_);
          if (c.end == ArmEnd::SUCCEEDED) {   // 퍽을 쥐고 끝나는 작업 / 놓고 끝나는 작업
            if (task == ArmTask::Goal::PICK || task == ArmTask::Goal::RETRIEVE) {
              carrying_ = !(task == ArmTask::Goal::PICK && pending_empty_);   // 세우기만 하고 LOOK 에서 끝난 PICK 은 빈 손
              held_since_ = now().seconds() + 0.3;   // 기준점에 멈춘 뒤 장만
              held_buf_.clear();
            }
            if (task == ArmTask::Goal::STORE || task == ArmTask::Goal::PLACE || task == ArmTask::Goal::HOME) {carrying_ = false;}
            held_from_pick_ = task == ArmTask::Goal::PICK && carrying_;   // 집은 자리에 쥔 채 멈춤 (박스 위 기준점 아님)
            look_rack_slot_ = task == kArmLookRack ? arm_goal_->get_goal()->slot : "";   // 관찰 자세에서 그 칸 STORE 를 기다림
            if (task != kArmLookRack) {held_tilt_ = task == ArmTask::Goal::PICK ? pending_tilt_ : 0.0;}   // LOOK_RACK 은 쥔 퍽 그대로
            if (task == ArmTask::Goal::PICK) {held_lying_ = pending_lying_;}
            else if (task == ArmTask::Goal::RETRIEVE || !carrying_) {held_lying_ = false;}
            if (task == ArmTask::Goal::PICK) {held_depth_mm_ = pending_depth_; depth_from_pick_ = true;}
            else if (task == ArmTask::Goal::RETRIEVE) {held_depth_mm_ = retrieve_depth_mm_; depth_from_pick_ = false;}
            else if (!carrying_) {held_depth_mm_ = 0.0; depth_from_pick_ = false;}
          }
          if (c.end != ArmEnd::SUCCEEDED) {held_from_pick_ = false; look_rack_slot_.clear();}   // 도중에 끝남 → 집은 자리에서 잇기 안 함 (arm_off_path_)
          if (scanning_) {
            scanning_ = false;
            if (c.end == ArmEnd::SUCCEEDED) {finish_rack_scan();} else {scan_seen_.clear();}
          }
          p0_idle_since_ = c.end == ArmEnd::SUCCEEDED && task_ends_p0_ ? now_s : -1.0;   // P0 에서 쉬기 시작
          if (c.end == ArmEnd::SUCCEEDED) {
            arm_goal_->succeed(result);
          } else if (c.end == ArmEnd::CANCELED) {
            arm_goal_->canceled(result);
          } else {
            RCLCPP_ERROR(get_logger(), "%s (접힘 확인: %s)", c.message.c_str(), fold_->status(now_s).c_str());
            arm_goal_->abort(result);
          }
          // ★ DRIVE 는 "끝까지 성공 + 접기로 끝남" 일 때만. 취소 · 실패는 언제나 STOP
          arbiter_.arm_ended(c.drive_ok && hw_ok_before_);   // 같은 주기에 하드웨어 이상이면 성공이어도 STOP
          arm_goal_.reset();
          active_handle_.reset();
          publish_turn();
          RCLCPP_INFO(get_logger(), "팔 작업 끝 (%s) → %s 차례", c.message.c_str(), turn_name(arbiter_.turn()));
          return;
        }
      }
    }
    if (runner_.waiting_step()) {
      if (!step_logged_) {
        RCLCPP_INFO(get_logger(), "구분 동작: 다음 '%s' 전에 멈춤 → ros2 service call /control_master/step std_srvs/srv/Trigger",
          runner_.next_name().c_str());
        step_logged_ = true;
      }
    } else {
      step_logged_ = false;
    }
  }

  // 시간표(점들, 시작부터 초)를 재생기에 보냄. 답은 콜백 → runner_ 로 전달
  // 차례 (ArmRunner) 와 상관없는 그리퍼 명령 — 팔 궤적이 도는 동안 (PICK 퍽 위로 가며 엶 · STORE 놓은 뒤 닫기 · 세우기 여는 점). 결과는 로그만
  void send_side_gripper(double g)
  {
    if (!grip_client_->action_server_is_ready()) {RCLCPP_ERROR(get_logger(), "그리퍼 재생기가 없음 → 팔 가는 중 그리퍼 못 움직임"); return;}
    FJT::Goal goal;
    goal.trajectory.joint_names = {gripper_joint_};
    trajectory_msgs::msg::JointTrajectoryPoint p;
    p.positions = {g};
    p.time_from_start = rclcpp::Duration::from_seconds(get_parameter("standup_open_time_s").as_double());
    goal.trajectory.points.push_back(p);
    FJTClient::SendGoalOptions opt;
    opt.result_callback = [this](const rclcpp_action::ClientGoalHandle<FJT>::WrappedResult & r) {
        if (r.code != rclcpp_action::ResultCode::SUCCEEDED) {RCLCPP_WARN(get_logger(), "팔 가는 중 보낸 그리퍼가 끝까지 못 감");}
      };
    grip_client_->async_send_goal(goal, opt);
    RCLCPP_INFO(get_logger(), "팔을 멈추지 않고 그리퍼 %s (%.4f m)", g > 0.5 * (gripper_open_ + gripper_close_) ? "엶" : "닫음", g);
  }

  void send_trajectory(
    const FJTClient::SharedPtr & client, const std::vector<std::string> & joints,
    const std::vector<TrajPoint> & points, unsigned gen)
  {
    active_client_ = client;
    active_handle_.reset();
    if (!client->action_server_is_ready()) {
      RCLCPP_ERROR(get_logger(), "재생기(follow_joint_trajectory)가 없음 → 컨트롤러가 떴는지 확인");
      runner_.on_goal_response(gen, false, now().seconds());
      return;
    }
    FJT::Goal goal;
    goal.trajectory.joint_names = joints;
    for (const auto & tp : points) {
      trajectory_msgs::msg::JointTrajectoryPoint p;
      p.positions = tp.q;
      p.time_from_start = rclcpp::Duration::from_seconds(tp.t);
      goal.trajectory.points.push_back(p);
    }

    FJTClient::SendGoalOptions opt;
    opt.goal_response_callback = [this, gen, client](const rclcpp_action::ClientGoalHandle<FJT>::SharedPtr h) {
        switch (runner_.on_goal_response(gen, static_cast<bool>(h), now().seconds())) {
          case LateGoal::USE:
            active_handle_ = h;
            active_gen_ = gen;
            break;
          case LateGoal::CANCEL_IT:
            // 이미 포기 · 취소한 동작이 늦게 받아들여짐 → 그 동작만 골라 취소 (안 그러면 팔이 혼자 움직임)
            RCLCPP_WARN(get_logger(), "포기한 팔 궤적(세대 %u)이 늦게 받아들여짐 → 그것만 취소", gen);
            client->async_cancel_goal(h);
            break;
          case LateGoal::IGNORE:
            break;
        }
      };
    opt.result_callback = [this, gen](const rclcpp_action::ClientGoalHandle<FJT>::WrappedResult & r) {
        const bool ok = r.code == rclcpp_action::ResultCode::SUCCEEDED && r.result &&
          r.result->error_code == FJT::Result::SUCCESSFUL;
        if (gen == active_gen_) {active_handle_.reset();}
        runner_.on_result(gen, ok);
      };
    client->async_send_goal(goal, opt);
  }

  // 궤적 취소: 받아들여진 그 궤적이면 그것만, 아직 답이 없으면 그 재생기의 전체 취소
  //   (전체 취소 뒤 늦게 받아들여지면 goal_response_callback 의 CANCEL_IT 가 또 취소함)
  void cancel_trajectory(unsigned gen)
  {
    if (!active_client_) {return;}
    if (active_handle_ && active_gen_ == gen) {
      active_client_->async_cancel_goal(active_handle_);
    } else {
      active_client_->async_cancel_all_goals();
    }
  }

  // ======================================================================== 보드 상태 묻기
  void ask_components()
  {
    const double t = now().seconds();
    if (hw_list_pending_) {
      // 답이 안 오는 요청은 버리고 다시 (rclcpp 는 답이 올 때까지 요청을 보관 → 쌓이지 않게). 그동안은 HardwareWatch 가 "모름" = 이상
      if (t - hw_list_sent_ < 2.0) {return;}
      hw_list_client_->prune_pending_requests();
      hw_list_pending_ = false;
    }
    if (!hw_list_client_->service_is_ready()) {return;}
    hw_list_pending_ = true;
    hw_list_sent_ = t;
    hw_list_client_->async_send_request(std::make_shared<ListHw::Request>(),
      [this](rclcpp::Client<ListHw>::SharedFuture f) {
        hw_list_pending_ = false;
        const auto r = f.get();
        bool ok = r && !r->component.empty();
        std::string detail;
        for (const auto & c : r ? r->component : std::vector<controller_manager_msgs::msg::HardwareComponentState>{}) {
          if (c.state.id != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
            ok = false;
            detail += (detail.empty() ? "" : ", ") + c.name + "=" + c.state.label;
          }
        }
        if (r && r->component.empty()) {detail = "하드웨어가 하나도 없음";}
        hw_->observe_components(now().seconds(), ok, detail);
      });
  }

  // 차례 방송 (10 Hz + 바뀔 때): swerve_controller 는 신선한 DRIVE 일 때만 바퀴를 움직임
  void publish_turn()
  {
    interfaces::msg::MotionMode m;
    m.header.stamp = now();
    const Turn t = arbiter_.turn();
    m.mode = t == Turn::STOPPING ? interfaces::msg::MotionMode::STOP : static_cast<uint8_t>(t);
    m.request_id = arbiter_.request_id();
    m.reason = turn_name(t);
    std::string why;
    m.ready = task_ready(rclcpp::Time(m.header.stamp).seconds(), why);
    if (m.ready != ready_sent_) {
      RCLCPP_INFO(get_logger(), "작업 준비 %s%s", m.ready ? "됨" : "안 됨: ", why.c_str());
    }
    ready_sent_ = m.ready;
    pub_turn_->publish(m);
  }

  // ---------------- 설정
  std::vector<std::string> arm_joints_;
  std::string gripper_joint_;
  double move_time_s_{4.0}, accept_timeout_s_{2.0}, motion_margin_s_{3.0}, cancel_timeout_s_{2.0};
  double gripper_open_{0.019}, gripper_close_{-0.005};
  double still_lin_{0.01}, still_ang_{0.05}, odom_timeout_s_{0.5};
  std::map<std::string, std::vector<double>> poses_;
  GripperValues grip_;                       // 열기 · 놓기 · 닫기 [m]
  double gripper_time_s_{1.2};
  PathTiming timing_;                        // 실물 실행기와 같은 속도 규칙
  double path_start_tol_{0.1};               // 경로 출발 자세 허용 [rad]
  bool lazy_fold_{false};                    // 나중에 접기
  double auto_fold_s_{5.0};                  // P0 에서 이만큼 쉬면 스스로 접음
  bool plan_ends_p0_{false};                 // 방금 만든 계획이 P0 에서 끝남 (lazy_fold)
  bool task_ends_p0_{false};                 // 진행 중 작업이 P0 에서 끝남
  double p0_idle_since_{-1.0};               // P0 에서 쉬기 시작한 시각 (아니면 −1)
  bool auto_folding_{false};                 // 스스로 접는 중 (goal 없음)
  bool held_from_pick_{false};
  std::string look_rack_slot_;               // LOOK_RACK 이 성공해 그 칸 관찰 자세에 있음 (다음 STORE 가 store_look_<칸>)
  std::optional<Eigen::Vector2d> measured_cell_;   // 지금 계획하는 STORE 의 잰 칸 바닥 중심 (base_link), rack_adjust 가 씀               // PICK 이 성공해 퍽을 쥔 채 집은 자리에 있음 (STORE · PLACE 가 거기서 바로)
  bool p0_join_ok_{false};                   // fold · unfold 의 P0 가 같아 P0 에서 이을 수 있음
  std::map<std::string, ArmPath> paths_;     // 실물 경로 (config/arm_paths.yaml)
  std::vector<ArmAction> pending_plan_;      // 받기 판단 때 만든 동작 목록 → 받은 뒤 그대로 시작
  ArmModel model_;                           // 실물 집기 계산기와 같은 팔 기구학
  bool model_ok_{false};
  std::vector<double> staging_;              // 박스 위 기준점 명령 [rad] (PICK 끝 = STORE · PLACE 시작)
  std::vector<double> pick_heights_, place_heights_;   // 퍽 윗면 위 손끝 높이 [mm]: 위 · 내림 · 쥐기(놓기)
  double surface_z_{0.037}, puck_h_{0.060}, grasp_offset_{0.015}, slow_dps_{15.0}, lying_h_{0.0348};
  double grip_now_{std::nan("")};       // 그리퍼 실측 [m]
  // 손목 인지 퍽 (base_link): 집기 · 놓기 때 이웃
  // p · yaw · axis = odom 기준, tagged = 윗면 태그로 잰 중심, lying = 긴축이 수평에서 45° 안 (planning 과 같은 기준), axis = 긴축 수평 방향
  struct SeenPuck {Eigen::Vector3d p; bool square; bool yaw_known; double yaw; bool round_r30; double t; bool tagged; bool lying; double axis;
    Eigen::Vector3d p_nom;};   // p = 카메라 돌아감 되돌린 위치, p_nom = 되돌리기 전 (목표 맞추기용)
  std::vector<SeenPuck> seen_;
  bool target_tagged_{false};     // 마지막 grasp_for 가 맞춘 목표 검출이 윗면 태그로 잰 퍽
  bool regrasp_{true};            // 기울여 쥔 퍽을 세워 내려놓고 수직으로 다시 쥠
  double regrasp_min_x_{0.212};   // 다시 잡기 자리 x 하한 [m]
  std::optional<Eigen::Vector2d> exclude_;   // 이웃에서 뺄 퍽 (다시 잡기: 집어 든 퍽) base_link
  double seen_t_{std::nan("")};   // 마지막 손목 인지 결과 시각
  double nb_age_s_{10.0}, nb_radius_{0.12}, nb_min_clear_{0.002}, nb_target_clear_{0.003};
  std::string odom_frame_{"odom"};
  double max_tilt_deg_{20.0};
  double held_tilt_{0.0};          // 쥔 퍽의 손 안 기울임 [°] (PICK 때 손 기울임, RETRIEVE = 0)
  double pending_tilt_{0.0};       // 받은 PICK 계획의 손 기울임 (성공하면 held_tilt_)
  bool depth_from_pick_{false};    // held_depth_mm_ 이 PICK 계획의 깊이 (RETRIEVE 의 25 mm 는 추정값이라 STORE 높이 맞춤에 안 씀)
  double held_depth_mm_{0.0};      // 쥔 퍽 깊이 = 퍽 윗면 ↔ 손끝 [mm] (PICK = 그 계획의 쥐는 깊이, RETRIEVE = retrieve_grip_depth_mm, 0 = 모름)
  double pending_depth_{0.0};      // 받은 PICK 계획의 쥐는 깊이 (성공하면 held_depth_mm_)
  bool held_lying_{false};         // 쥔 퍽이 누운 네모 퍽 (STORE 거절, PLACE 는 누운 채로)
  bool pending_empty_{false};      // 받은 PICK 계획이 세우기만 (퍽을 안 쥐고 LOOK 에서 끝남)
  bool pending_lying_{false};      // 받은 PICK 계획이 누운 퍽 (성공하면 held_lying_)
  double place_clear_mm_{2.0}, retrieve_depth_mm_{25.0}, tilted_depth_mm_{30.0};
  static constexpr uint8_t kArmLookRack = 8;      // ArmTask LOOK_RACK (팀 interfaces 8 번, 합치기 전 우리 ArmTask.action 엔 이름이 없어 번호로)
  static constexpr double kStoreMaxTilt = 10.0;   // 이보다 기울여 쥔 퍽은 STORE 거절 (적재함 경로는 수직 퍽으로 검증)
  static constexpr double kStandupDt = 0.1;       // 세우기 표 점 간격 [s] (export_standup_table.py --every 2 × 0.05 s)
  std::vector<NeighborPuck> last_nb_;   // grasp_for 가 본 이웃 (poses_for 의 기울인 손 다시 검사용)
  std::unique_ptr<tf2_ros::Buffer> tf_buf_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listen_;
  rclcpp::Subscription<interfaces::msg::DetectionArray>::SharedPtr sub_dets_;
  rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr sub_drift_;
  rclcpp::TimerBase::SharedPtr side_grip_timer_;   // 세우기: 팔 동작 도중 그리퍼 여는 시각
  bool drift_correct_{true};
  std::string drift_frame_;
  Eigen::Quaterniond drift_q_{Eigen::Quaterniond::Identity()};
  // 적재함 태그 → 칸 경로 옮기기
  std::vector<std::string> rack_names_;
  std::vector<Eigen::Vector2d> rack_xy_;     // 칸 중심 표 (config/rack_cells.yaml)
  std::vector<Eigen::Vector2d> rack_place_;  // 칸 넣는 점 보정 (같은 표 place_offset) [m]
  std::vector<StandupPath> standup_;   // 누운 퍽 세우기 경로 표
  struct StandupCheck {std::string pose; Eigen::Vector3d tip;};   // 세우기 첫 구간 이름 · 쥐는 자리 손끝 (모델)
  std::optional<StandupCheck> standup_check_;
  double standup_reach_tol_mm_{6.0}, standup_grip_min_m_{0.001};
  double path_slow_min_dps_{0.0};
  bool side_grip_{true};   // 빈손 그리퍼 동작을 팔을 멈추지 않고 (gripper_on_the_way)
  double standup_speed_dps_{32.0}, standup_max_gap_mm_{6.0}, standup_max_gap_deg_{6.0};
  bool lying_cylinder_{false};   // 마지막 grasp_for 의 누운 퍽이 원기둥 (세우기 안 함)
  double lying_axis_deg_{std::numeric_limits<double>::quiet_NaN()};   // 마지막 grasp_for 의 누운 퍽 긴축 [°]
  struct HeldTag {std::string id; Eigen::Vector3d p, n;};   // 쥔 퍽 태그 (손목 좌표)
  std::deque<HeldTag> held_buf_;
  double held_since_{1e18};
  double held_max_mm_{8.0};
  std::optional<Eigen::Vector3d> held_ref_p_, held_ref_n_;
  rclcpp::Subscription<interfaces::msg::DetectionArray>::SharedPtr sub_held_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_held_ref_;
  Eigen::Vector2d rack_pivot_{Eigen::Vector2d::Zero()};
  std::string rack_ref_file_;
  double rack_min_shift_m_{0.0015}, rack_max_shift_m_{0.008}, rack_max_rms_m_{0.003};
  int rack_min_samples_{5};
  bool scanning_{false};                     // LOOK rack 진행 중
  std::map<std::string, std::vector<Eigen::Vector2d>> scan_seen_;   // 이번 스캔 칸 태그 중심
  std::map<std::string, Eigen::Vector2d> last_scan_, rack_ref_;      // 마지막 스캔 · 기준 스캔 (칸별 중앙값)
  RackFit rack_fit_;                         // 기준 → 마지막 스캔 적재함 움직임
  bool rack_fit_ok_{false};
  rclcpp::Subscription<interfaces::msg::DetectionArray>::SharedPtr sub_rack_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_rack_ref_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_step_;
  bool pending_step_{false};   // 받은 계획을 구분 동작으로 (누운 퍽 세우기 · park_step)
  bool step_logged_{false};
  double home_near_tol_{0.2};           // [rad] 받침에서 이만큼 안이면 HOME 한 점 직행 허용
  bool arm_off_path_{false};            // 경로 작업이 도중에 멈춤 (성공한 작업 · HOME 뒤 지움)
  bool home_was_path_{false};           // 지금 HOME 이 검사된 경로 (fold · home_from_*) 로 감
  bool carrying_{false};                // PICK · RETRIEVE 성공 뒤 퍽을 쥐고 있음 (STORE · PLACE · HOME 성공 뒤 지움)
  std::unique_ptr<FoldCheck> fold_;          // 접힘 실측 확인 (R1)
  std::unique_ptr<HardwareWatch> hw_;        // 하드웨어 측정 살아 있나 (재검증 R6 · 보드 2장)
  std::vector<std::string> hw_watch_joints_;  // 측정을 볼 관절 (기본 14 개 = 주행 · 팔 보드 전부)
  rclcpp::Client<ListHw>::SharedPtr hw_list_client_;
  rclcpp::TimerBase::SharedPtr hw_timer_;
  bool hw_list_pending_{false};
  double hw_list_sent_{0.0};
  bool hw_ok_before_{false}, have_hw_once_{false};
  double hw_grace_until_{0.0};   // 이 시각까지 하드웨어 감시는 이전 판정 유지 (계획으로 노드가 잠깐 멈춘 뒤)
  bool ready_sent_{false};   // 마지막으로 방송한 ready (바뀔 때 바로 다시 보내려고)
  std::vector<double> joint_buf_;            // /joint_states → arm_joints 순서 (콜백마다 재사용)

  // ---------------- 상태
  MotionArbiter arbiter_;
  ArmRunner runner_{2.0, 7.0, 2.0};   // 생성자에서 설정값으로 다시 만듦
  bool have_odom_{false}, odom_still_{false};
  rclcpp::Time odom_stamp_{0, 0, RCL_ROS_TIME};   // 마지막 /odom 의 측정 시각
  uint32_t lock_acked_id_{0};                      // swerve_controller 가 "잠갔음" 이라고 답한 요청 번호
  std::shared_ptr<ArmGoalHandle> arm_goal_;
  rclcpp_action::ClientGoalHandle<FJT>::SharedPtr active_handle_;   // 지금 궤적 (받아들여진 것)
  unsigned active_gen_{0};                                          // 그 궤적의 세대 번호
  FJTClient::SharedPtr active_client_;

  // ---------------- ROS 연결
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<interfaces::msg::MotionMode>::SharedPtr sub_ack_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joints_;
  rclcpp::Publisher<interfaces::msg::MotionMode>::SharedPtr pub_turn_;
  rclcpp_action::Server<ArmTask>::SharedPtr arm_server_;
  FJTClient::SharedPtr arm_client_, grip_client_;
  rclcpp::TimerBase::SharedPtr timer_, turn_timer_;
};

}  // namespace manipulation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<manipulation::ControlMaster>());
  rclcpp::shutdown();
  return 0;
}
