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
//   ⑤ (R1) "접기로 끝났다" 는 JTC 의 성공 답이 아니라 **/joint_states 실측**으로 확인
//      마지막 home 동작 뒤 VERIFY: 측정 시각이 새로운 관절값 5개가 home ± fold_tolerance 안에 fold_hold_s 동안 → DRIVE
//      verify_timeout_s 안에 안 되면 실패 → STOP. 측정이 없거나 NaN(하드웨어가 못 읽은 모터) · 오래됨도 실패
//      (재현: 어깨를 고정해도 JTC 목표 오차 검사가 꺼져 있으면 HOME 성공 → DRIVE.
//       JTC 쪽에도 goal 허용 오차를 켰지만(controllers.yaml), 바퀴를 푸는 판단은 이 노드가 실측으로 한 번 더)
//   ⑥ (재검증 R6 · 보드 2장) 하드웨어 이상이면 HOME 확인 무효 — arm.cpp HardwareWatch
//      ① 보드 상태 직접: controller_manager 의 list_hardware_components 를 hw_component_hz 로 물어 하나라도 active 가 아니면
//         (주행 보드만 죽고 팔은 멀쩡해도 잡힘). 답이 hw_component_timeout_s 넘게 없어도 이상
//         단 hw_status_optional (기본 true): 한 번도 못 받았거나 서비스가 안 보이면 (DDS, 11-10 실물) ② 관절 측정 + 팔 · 그리퍼 재생기 준비로 판단.
//         받은 답이 이상 (active 아닌 보드) 이면 그대로 이상
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
//   - 실패 뒤 자동 접기 (fail_fold_s, 기본 3 s): 팔 작업이 실패로 끝나고 새 작업이 없으면 HOME 과 같은 복귀 (home_plan) 로 스스로 접음
//     (퍽을 쥔 채 · 하드웨어 이상 · 취소 · HOME 실패 뒤엔 안 함). 빈손 쥐기로 멈췄으면 온 길 (퍽 위 → P0 · LOOK) 로 되돌아감.
//     빈손으로 끝난 PICK 결과 글은 "EMPTY_GRASP: " 로 시작 (planning 이 기계로 읽을 표시) — 퍽이 박스에 남는 닫기만.
//     받침 안 다시 쥐기 (park_re_down) 빈손은 표시 없이 실패 · 자동 접기 안 함 (퍽이 로봇 위 받침에 있음 → 사람 확인)
//     끄기: ros2 param set /control_master fail_fold_s 0.0 (접는 도중이면 그 접기도 취소)
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
//   - PLACE 는 place_held_correct 가 true 일 때만 같은 보정 (바로 놓기만)
//
// PLACE 놓는 점 보정: place_offset_x/y_mm (base_link, 모든 PLACE) + planning 의 장소별 place_offset_L*_mm (목표에 이미 더해져 옴)

#include <algorithm>
#include <array>
#include <chrono>
#include <ctime>
#include <cmath>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "controller_manager_msgs/srv/list_hardware_components.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "geometry_msgs/msg/quaternion_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
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
#include "manipulation/grasp_guard.hpp"
#include "manipulation/arm_model_params.hpp"
#include "manipulation/arm_path.hpp"
#include "manipulation/held_correction.hpp"
#include "manipulation/held_image.hpp"
#include "manipulation/arm_state.hpp"
#include "manipulation/look_variant.hpp"
#include "manipulation/cell_shift.hpp"
#include "manipulation/rack_tags.hpp"
#include "manipulation/standup_table.hpp"
#include "manipulation/park_stand.hpp"
#include "manipulation/place_depth.hpp"
#include "manipulation/visual_regrasp.hpp"
#include "interfaces/action/arm_task.hpp"
#include "interfaces/msg/motion_mode.hpp"

namespace manipulation
{

using ArmTask = interfaces::action::ArmTask;
using ArmGoalHandle = rclcpp_action::ServerGoalHandle<ArmTask>;
using FJT = control_msgs::action::FollowJointTrajectory;
using FJTClient = rclcpp_action::Client<FJT>;
using ListHw = controller_manager_msgs::srv::ListHardwareComponents;

namespace
{
// 심장박동 재발행 판단 (순수): 마지막 발행 뒤 max_gap_s 넘게 지났으면 true (ControlMaster::heartbeat)
bool heartbeat_due(double last_pub_s, double now_s, double max_gap_s) {return now_s - last_pub_s > max_gap_s;}
double steady_s() {return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
}  // namespace

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
    // 받침에 넣을 때 손가락을 살짝 풀어 퍽이 돌며 들어가게 (박스 세우기 때처럼 돌 수 있는 폭으로). 퍽에 닿은 자리 약 0.0046
    grip_.loose = declare_parameter<double>("park_loose_grip_m", 0.0060);
    gripper_time_s_ = declare_parameter<double>("gripper_time_s", 1.2);
    // ---- 실물 경로 (config/arm_paths.yaml) ----
    timing_.speed_dps = declare_parameter<double>("path_speed_dps", 40.0);
    timing_.stand_speed_dps = declare_parameter<double>("path_stand_speed_dps", 20.0);
    timing_.accel_dps2 = declare_parameter<double>("path_accel_dps2", 150.0);
    timing_.dt_s = declare_parameter<double>("path_dt_s", 0.05);
    // 그리퍼를 여닫기 전 팔이 멈춰 따라오게 기다림 [s]. 실물: 그리퍼 동작 때 2 번 관절이 아직 2.3° 남아 (손끝 약 1 cm) 놓는 자리가 반반
    timing_.grip_settle_s = declare_parameter<double>("grip_settle_s", 0.0);
    // 퍽 위 (내려가기 직전) 점에서 이만큼 멈춤 [s]. 손이 흔들린 채 above 에 도착해 바로 내려가 쥐면 위치 오차 → 0 = 지금 동작 그대로
    //   실행 중: ros2 param set /control_master pick_settle_s 0.4 (다음 작업부터). PLACE 의 놓을 자리 위는 place_settle_s
    declare_parameter<double>("pick_settle_s", 0.0);
    declare_parameter<double>("place_settle_s", 0.0);
    // STORE 칸 위 (놓는 점 위 store_settle_up_mm 이상 높은 마지막 점 = 마지막 내려가기 직전) 에서 이만큼 멈춤 [s]
    //   실행 중: ros2 param set /control_master store_settle_s 0.4 (다음 STORE 부터). 0 = 지금 동작 그대로 (store_settle)
    declare_parameter<double>("store_settle_s", 0.0);
    declare_parameter<double>("store_settle_up_mm", 20.0);
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
    // P0 에서 끝난 뒤 이만큼 다음 팔 작업이 없으면 스스로 접음 (실물: P0 에서 약 15 s 버티면 팔 모터 과열 Hardware Error)
    auto_fold_s_ = declare_parameter<double>("lazy_fold_idle_s", 5.0);
    // 팔 작업이 실패 (ABORTED) 로 끝난 뒤 이만큼 다음 팔 작업이 없으면 HOME 과 같은 복귀로 스스로 접음 (0 이하 = 끔, 실행 중 바꿈)
    //   11-10 실물: PICK 이 빈손으로 멈춘 뒤 planning FAULT → 팔이 박스 위에 뻗은 채 약 65 s → 관절 2 과부하로 팔 보드 전체 멈춤
    //   퍽을 쥔 채 · 하드웨어 이상 · 취소 · HOME 실패 · 받침 안 다시 쥐기 빈손 뒤에는 안 함 (maybe_fail_fold)
    //   끄기: ros2 param set /control_master fail_fold_s 0.0 (정수 0 도 아래 콜백이 0.0 으로 받음). yaml 에 적을 땐 3.0 처럼 소수점
    //   이미 시작한 실패 뒤 접기 도중 0.0 으로 바꾸면 그 접기도 취소 (run_arm, 바퀴 잠금 · 다시 안 함 · 사람이 보고 HOME)
    declare_parameter<double>("fail_fold_s", 3.0);
    // double 파라미터라 ros2 param set 의 정수 (0 · 3) 는 원래 "Wrong parameter type" 로 거절 → fail_fold_s 만 실수로 바꿔 받음
    fail_fold_int_cb_ = add_pre_set_parameters_callback([](std::vector<rclcpp::Parameter> & ps) {
        for (auto & p : ps) {
          if (p.get_name() == "fail_fold_s" && p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
            p = rclcpp::Parameter("fail_fold_s", static_cast<double>(p.as_int()));
          }
        }
      });
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
    // 쥐기 −40 mm = 실물 넣기 시험 때 쥔 깊이. 칸 경로(arm_paths)가 이 깊이로 쥔 퍽 기준 — 얕게 쥐면 쥔 퍽이 그만큼 낮게 지나가
    //   옆 칸 퍽을 긁음 (−30 이면 10 mm 낮음: 물리 시뮬 넣기 41/56 → −40 에서 53/56, 실제 채우는 순서 24/24)
    pick_heights_ = declare_parameter<std::vector<double>>("pick_heights_mm", {40.0, 10.0, -40.0});
    place_heights_ = declare_parameter<std::vector<double>>("place_heights_mm", {40.0, 10.0, -23.0});   // 마지막 = 쥔 깊이 모를 때만
    place_clear_mm_ = declare_parameter<double>("place_bottom_clear_mm", 2.0);      // 놓을 때 퍽 바닥 ↔ 놓을 면
    retrieve_depth_mm_ = declare_parameter<double>("retrieve_grip_depth_mm", 25.0); // 칸에서 꺼낸 퍽을 쥔 깊이 (실물 빼기 때 쓴 깊이, unload_cell.py)
    tilted_depth_mm_ = declare_parameter<double>("pick_tilted_depth_mm", 30.0);     // 손을 10° 넘게 기울인 PICK 의 쥐는 깊이 (퍽 무게중심)
    // 선 퍽 PICK: 손목 인지가 낸 윗면을 이만큼 높게 봄 [mm] → 쥐는 손끝이 그만큼 얕아짐. pick_heights_mm (= STORE 칸 높이 맞춤 기준) 은 그대로
    declare_parameter<double>("pick_top_bias_mm", 0.0);
    surface_z_ = declare_parameter<double>("puck_surface_z", 0.037);
    puck_h_ = declare_parameter<double>("puck_height", 0.060);
    // PICK 목표 퍽 중심 z 가 박스 윗면 + 반높이 (누운 퍽 = 반폭) 에서 이만큼 [mm] 벗어나면 거절 (11-09 B4: 퍽 8 z 0.178 로 10 s 계획). 0 이하 = 안 봄
    declare_parameter<double>("pick_z_tol_mm", 30.0);
    declare_parameter<double>("lying_z_fix_mm", 15.0);   // 누운 퍽 검출 z 가 박스 기준에서 이보다 어긋나면 박스 기준 높이로 쥠
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
    id_max_offset_m_ = declare_parameter<double>("pick_id_max_offset_m", 0.06);   // goal.puck_id 퍽이 목표점에서 이보다 멀면 거절
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    // 수직 손이 안 닿는 먼 퍽 · 자리: 손을 10° 씩 바깥으로 기울여 봄 (20° ≈ 31 cm). 0 = 수직 손만
    //   기울여 집은 퍽은 손 안에서 그만큼 기울어 있음 → STORE 는 10° 까지, PLACE 는 쥘 때 기울임과 20° 안으로 놓음 (넘어지지 않게)
    max_tilt_deg_ = declare_parameter<double>("pick_max_tilt_deg", 20.0);
    // 손목 카메라로 잡은 퍽을 실물에서 늘 한쪽으로 비껴 쥐면 그만큼 PICK 쥐는 점을 옮김 [mm, base_link] (25 까지)
    //   윗면 태그가 보인 퍽 (검출 id 있음) 은 인지가 태그로 중심을 잡으므로 옮기지 않음
    //   실행 중 바로 바꿈: ros2 param set /control_master pick_offset_x_mm -5.0 (다음 PICK 부터)
    declare_parameter<double>("pick_offset_x_mm", 0.0);
    // 다시 잡기: 수직 손이 안 닿아 기울여 쥔 퍽은 손 안에서 그만큼 기울어 STORE 때 칸 테두리에 걸림 (실물, 10° 에서 매번)
    //   → 같은 PICK 안에서 박스 위 차체 쪽 넉넉한 빈 자리 (손목 인지로 확인, 집어 든 퍽 자리는 빈 것으로) 에 세워 내려놓고
    //   수직 손으로 다시 쥠 (regrasp_nodes). 없으면 PICK 거절
    //   regrasp_min_x: 내려놓는 퍽 중심 x 하한 [m] — 박스 앞면이 0.191 m (정차 3 cm) 여도 퍽 (반폭 17.4 mm) 이 박스 위
    regrasp_ = declare_parameter<bool>("regrasp_tilted", true);
    // 다시 보고 쥐기 (regrasp_visual, visual_regrasp.hpp): 세워 놓고 손을 뺀 뒤 (re_place_clear) 곧게 더 올린 관찰 점 (re_observe) 에서
    //   손목 카메라 새 장을 기다려 잰 중심 XY 로 수직 다시 쥐기 (다시 쥐기 다섯 점의 높이 · 닫힘 방향 · 벌림 · 속도는 그대로).
    //   못 보거나 · 모호 · 너무 옮겨짐 · IK 안 됨 = 지금처럼 보지 않고 쥠. 다음 작업부터 (ros2 param set /control_master regrasp_visual false = 지금 동작 그대로)
    declare_parameter<bool>("regrasp_visual", true);
    // 관찰 점: 손끝이 박스 위 선 퍽 윗면에서 이만큼 위 [mm], 손 기울임은 이 값에 가까운 것부터 [°]. 11-10 계산 (URDF 카메라 · 팔 모델):
    //   110 mm · 20° = 카메라 광축 깊이로 퍽 윗면 0.207 m (D435 848×480 깊이 최소 약 0.18 m 밖), 퍽 전체가 화면 안 (광축에서 ≤ 11°),
    //   손가락이 퍽을 안 가림. re_place_clear (윗면 위 40–50 mm) 는 0.14–0.15 m 라 윗면 깊이가 빠짐 (인지가 누운 퍽 · 12 mm 치우침으로 봄)
    declare_parameter<double>("regrasp_visual_observe_mm", 110.0);
    declare_parameter<double>("regrasp_visual_observe_tilt_deg", 20.0);
    // 관찰 점에서 새 관측 기다림 한계 [s] (넘으면 보지 않고 쥠). 보이면 약 0.3 – 0.4 s (멈춤 0.2 + 인지 지연 0.1 + 확정, mock 0.32 s).
    //   Codex 2.0 → 1.0: 못 볼 때 (가림 · 인지 꺼짐) 손해를 줄임
    declare_parameter<double>("regrasp_visual_timeout_s", 1.0);
    declare_parameter<double>("regrasp_visual_settle_s", 0.2);     // 멈춘 뒤 이 시간 뒤 촬영한 장만 [s] (흔들림)
    declare_parameter<double>("regrasp_visual_max_age_s", 0.5);    // 고를 때 촬영한 지 이 시간 안 [s]
    declare_parameter<double>("regrasp_visual_max_shift_mm", 15.0);   // 잰 중심이 놓은 자리에서 이보다 멀면 안 씀 (오검출) [mm]
    regrasp_min_x_ = declare_parameter<double>("regrasp_min_x", 0.212);
    // 내려놓을 자리 |y| 상한 [m] = 박스 반폭 0.1025 − 퍽 반폭 0.0174 − 여유 0.01 (11-09: lat ±8 cm 가 박스 밖)
    regrasp_max_abs_y_ = declare_parameter<double>("regrasp_max_abs_y", 0.075);
    // 다시 쥘 자리 탐색 시간 예산 [s]: 11-09 실물 19 곳 × IK 씨앗 63 개 = 10 s 계획 → motion_mode 끊겨 미션 FAULT. 넘으면 그 PICK 거절 (planning 이 건너뜀). 0 이하 = 예산 없음
    declare_parameter<double>("regrasp_search_s", 0.8);
    declare_parameter<double>("pick_offset_y_mm", 0.0);
    // 윗면 태그로 잰 퍽 (인지 중심이 2 mm 안) 은 팔 쪽 치우침만: ros2 param set /control_master pick_offset_tagged_x_mm -3.0
    declare_parameter<double>("pick_offset_tagged_x_mm", 0.0);
    declare_parameter<double>("pick_offset_tagged_y_mm", 0.0);
    // 누운 퍽을 위 옆면 태그로 잰 중심: 아직 실물로 못 잼 → 처음엔 선 퍽 태그 값과 같게 (config). 받침 세우기는 긴축 ±2 mm 안이 필요
    declare_parameter<double>("pick_offset_lying_tagged_x_mm", 0.0);
    declare_parameter<double>("pick_offset_lying_tagged_y_mm", 0.0);
    // 태그가 안 보인 누운 퍽 (깊이 점 중심): 선 퍽 pick_offset_x/y 와 따로 (실물: 태그 보이면 맞는데 안 보이면 선 퍽 값이라 어긋남)
    declare_parameter<double>("pick_offset_lying_x_mm", 0.0);
    declare_parameter<double>("pick_offset_lying_y_mm", 0.0);
    // 손목을 돌려 쥘 때 보정 [mm]: 손가락 가운데가 J5 축에서 12 mm 떨어져 있어, 팔 모델의 그 거리가 조금 틀리면 쥐는 점 오차가 손목 방향에 따라 달라짐.
    //   값 = 손목을 기본 (닫힘축 ≈ 접선) 에서 90° 돌려 쥘 때 (실물: 닫힘축이 반지름 방향) 더할 보정 [앞 · 왼쪽], 팔 바로 앞 퍽 기준.
    //   다른 각도 · 자리는 같은 오차 (손 좌표의 손끝 어긋남) 로 계산: (R(돌린 각) − I) · h. 기본 방향 (돌림 0) 은 0 → 다른 보정 그대로
    //   실행 중: ros2 param set /control_master pick_offset_rot_mm "[3.0, 0.0]"
    declare_parameter<std::vector<double>>("pick_offset_rot_mm", std::vector<double>{0.0, 0.0});
    // 손목을 돌려 쥘 때 (기본에서 45° 넘게, 어느 쪽이든) 그대로 더하는 보정 [앞 · 왼쪽 mm, base_link = 로봇 기준]. 위 모델 보정 위에 더함
    //   (실물: 모델 값을 바꾸니 돌린 각 · 자리에 따라 엉뚱한 쪽으로 감 → 로봇 기준으로)
    declare_parameter<std::vector<double>>("pick_offset_rot_robot_mm", std::vector<double>{0.0, 0.0});
    // 기울인 손 PICK 보정 [접선 (+ = J1 반시계, 앞 퍽이면 왼쪽), 반지름 (+ = 바깥)] mm — 기울인 손은 닫힘축이 접선 (11-09 실물: 먼 퍽에 오른쪽 손가락이 닿음)
    declare_parameter<std::vector<double>>("pick_offset_tilt_mm", std::vector<double>{0.0, 0.0});
    // STORE 때 칸 근처 경로를 더 옮김 [mm, base_link, 넣는 점 맞추기와 합해 20 까지] — 쥔 퍽이 칸에서 늘 한쪽으로 걸리면. 적재함 태그 보정 · 칸별 store_offset.<칸> 과 더해짐
    //   실행 중 바로 바꿈: ros2 param set /control_master store_offset_y_mm 3.0 (다음 STORE 부터, x+ = 앞 · y+ = 왼쪽)
    declare_parameter<double>("store_offset_x_mm", 0.0);
    // STORE 넣는 점 맞추기: 경로의 놓는 점 (grip release) 손끝을 팔 모델로 계산해 칸 표 (칸 중심 + place_offset) 로 옮김.
    //   실물 기록 재생 경로는 놓는 점이 칸보다 팔 J1 축 반대쪽으로 7 ~ 13 mm 나가 있음 (실물: S7 +13 · S12 +5 mm 에서 들어감 = 이 계산과 2 mm 안)
    declare_parameter<bool>("store_to_cell", false);
    declare_parameter<double>("store_offset_y_mm", 0.0);
    // PLACE 놓는 점 보정 [mm, base_link, x+ = 앞 · y+ = 왼쪽] — 모든 PLACE 목표에 더함 (먼 자리 다시 쥐기의 마지막 놓기도). 장소별은 planning place_offset_L*_mm
    //   실행 중 바로 바꿈: ros2 param set /control_master place_offset_x_mm -5.0 (다음 PLACE 부터)
    declare_parameter<double>("place_offset_x_mm", 0.0);
    declare_parameter<double>("place_offset_y_mm", 0.0);
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
        for (const auto & n : rack_names_) {declare_parameter<std::vector<double>>("storeheld_offset." + n, std::vector<double>{0.0, 0.0});}
        // 칸마다 RETRIEVE 보정 [x, y] 또는 [x, y, 높이] mm (STORE 보정은 빼기에 안 들어감 → 따로): ros2 param set /control_master retrieve_offset.S12 "[-10.0, -5.0]"
        for (const auto & n : rack_names_) {declare_parameter<std::vector<double>>("retrieve_offset." + n, std::vector<double>{0.0, 0.0});}
        // RETRIEVE 가정보다 얕게 쥔 양 [mm]임. PLACE 높이에만 사용함 (0 = 기존 동작).
        for (const auto & n : rack_names_) {declare_parameter<double>("retrieve_depth_fix." + n, 0.0);}
      }
    }
    rack_ref_file_ = declare_parameter<std::string>("rack_reference_file", "~/.ros/rack_reference.txt");
    if (rack_ref_file_.rfind("~/", 0) == 0 && std::getenv("HOME")) {rack_ref_file_ = std::getenv("HOME") + rack_ref_file_.substr(1);}
    rack_min_shift_m_ = declare_parameter<double>("rack_shift_min_m", 0.0015);
    rack_max_shift_m_ = declare_parameter<double>("rack_shift_max_m", 0.008);
    rack_max_rms_m_ = declare_parameter<double>("rack_fit_max_rms_m", 0.003);
    rack_min_samples_ = declare_parameter<int>("rack_min_samples", 5);
    load_rack_reference();
    // 마지막 빈손 스캔 맞춤을 파일로 기억 → 다시 켜도 scan 없이 칸 경로를 옮김 (새 scan 이 덮어씀, save_rack_reference 하면 지움)
    rack_fit_file_ = declare_parameter<std::string>("rack_fit_file", "~/.ros/rack_fit_last.txt");
    if (rack_fit_file_.rfind("~/", 0) == 0 && std::getenv("HOME")) {rack_fit_file_ = std::getenv("HOME") + rack_fit_file_.substr(1);}
    rack_fit_persist_ = declare_parameter<bool>("rack_fit_persist", true);
    if (rack_fit_persist_) {load_rack_fit();}
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
    // 누운 퍽 쥐기 깊이 한도: 손끝이 놓인 면 위 이만큼 (모델). 실물은 명령보다 약 10 mm 얕게 쥠 (누운 퍽 · 선 퍽 −40 → 30) → config 0
    declare_parameter<double>("pick_lying_tip_clear_mm", 10.0);
    // 세우기 경로 전체를 위아래로 옮김 [mm, − = 아래]: 실물이 명령보다 얕게 쥐어 시뮬 (−20 mm 쥐기) 과 같게 하려면 −10
    declare_parameter<double>("standup_dz_mm", 0.0);
    declare_parameter<double>("standup_tip_clear_mm", 2.0);   // 세우기 경로 손끝 최저 = 박스 윗면 위 이만큼 (모델)
    declare_parameter<double>("standup_reach_scale", 1.0);    // 쥔 뒤 경로의 수평 이동 배율 (1 = 표 그대로, 0.5 = 절반만 뻗음)
    declare_parameter<double>("standup_time_scale", 1.0);     // 세우기 경로 점 사이 시간 = 녹화 0.1 s × 이 배 (1 = 시뮬과 같은 속도)
    declare_parameter<double>("standup_open_time_s", 0.5);   // 팔 안 멈추고 하는 그리퍼 동작 시간 [s] (세우기 여는 점 · PICK 미리 열기 · 놓은 뒤 닫기. 시뮬 정책: 스텝마다 2 mm → 약 0.5 s)
    declare_parameter<bool>("gripper_on_the_way", true);      // 빈손일 때 그리퍼를 팔 멈추지 않고 (PICK 퍽 위로 가며 열기 · STORE · PLACE 놓은 뒤 닫기)
    // 누운 네모 퍽을 PICK 하면서 세움 (false = 누운 채 쥐고 기준점으로). 실행 중: ros2 param set /control_master stand_lying true
    declare_parameter<bool>("stand_lying", false);
    // 세우는 방법: park = 그리퍼 받침에 기울여 넣고 손가락으로 밀어 세운 뒤 다시 쥠 (시뮬 12 중 10), table = 강화학습 표 재생 (박스 위)
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
    declare_parameter<double>("park_push_j3_deg", 0.0);         // 밀어 세우는 동안 J3 를 이만큼 더 [°, + = 손끝 위], 밀기 가운데서 전부 · 시작 · 끝점 0 (J3 를 더 굽혀, 끝에선 펴기)
    declare_parameter<double>("park_push_j2_deg", 0.0);         // 밀어 세우는 동안 J2 를 이만큼 더 올림 [°, + = 손끝 위], J3 와 같은 모양 → 퍽 뒷면 아래 모서리를 긁어 올려 세움
    declare_parameter<bool>("park_press_mode", false);          // true = 밀기 대신 꽂아 누르기: 놓은 손을 올려 빼고 수직으로 · 닫고 받침 뒤 (J1 쪽) 에 곧게 내려 꽂아 걸린 퍽을 세움
    declare_parameter<double>("park_press_r_mm", -16.0);        // 꽂는 손끝 자리 = 받침 중심에서 반지름 방향 (− = 뒤 · J1 쪽, 뒤 벽 안쪽 면 −23.5)
    declare_parameter<double>("park_press_top_mm", 66.0);       // 올리는 · 넘어가는 손끝 높이 = 바닥 위 (수직 손이 닿는 끝 66)
    declare_parameter<double>("park_press_h_mm", 10.0);         // 꽂는 가장 낮은 손끝 높이 = 바닥 위
    declare_parameter<double>("park_press_speed_dps", 0.0);     // 꽂아 내리기 (park_press1–4 · up) 속도, 0 = park_speed_dps (실물에서 이 속도로 섬)
    declare_parameter<double>("park_release_up_mm", 0.0);        // 넣는 끝점을 손등 (손목 카메라) 쪽으로 이만큼 (넣으면서 같이, 손 축에 수직)
    declare_parameter<int>("park_release_pull_steps", 3);       // 당기는 점 수
    declare_parameter<double>("park_speed_dps", 20.0);          // 받침에서 넣기 · 다시 쥐기 속도
    declare_parameter<double>("park_push_speed_dps", 20.0);     // 받침에서 밀어 세우는 속도 (빠르면 손끝이 퍽 면을 타고 올라 넘어짐)
    declare_parameter<bool>("store_match_grip_depth", true);   // STORE: 쥔 깊이가 박스 PICK 깊이와 다르면 (받침에서 다시 쥔 퍽) 칸 근처 높이를 그 차만큼
    declare_parameter<double>("store_match_grip_extra_mm", 0.0);   // 위 맞춤이 들어갈 때 (받침에서 다시 쥔 세운 퍽 등) 만 칸 근처 높이에 더함 [mm, − = 더 깊게 넣음]. 실행 중 바꿈
    // 새 칸 경로 (11-10, config/arm_paths.yaml store_<칸>_d50 · _d30 · _d30_low · _d40_tilt10 · 20 · 30 · retrieve_<칸>_full — 칸 보정이 이미 들어 있음, cell_shift.hpp)
    //   false = 지금 이름 (store_<칸> · retrieve_<칸>) 그대로. 실행 중: ros2 param set /control_master storeheld_paths false
    declare_parameter<bool>("storeheld_paths", true);
    declare_parameter<bool>("storeheld_low", false);        // 세운 퍽 (30 mm) 을 _d30_low (퍽 바닥 = 테두리 +3 mm) 로
    declare_parameter<bool>("storeheld_over_look", true);   // LOOK_RACK 뒤 관찰 자세여도 store_look_<칸> 대신 새 경로
    // STORE 칸 경로를 LOOK_RACK 로 잰 칸 (planning 이 STORE target 으로 줌) 으로 옮길지. false = 빈손 스캔 맞춤 (기억한 것 포함) 으로 —
    //   LOOK_RACK 측정은 look_rack_bias_mm 를 실물로 맞추기 전이라 칸 표와의 고정 차가 그대로 들어감 (11-10 밤). 실행 중 바꿈: 작업마다 읽음
    declare_parameter<bool>("store_use_look_rack", false);
    declare_parameter<std::vector<double>>("look_rack_bias_mm", {0.0, 0.0});   // LOOK_RACK 칸 측정의 카메라 치우침 [x, y] mm (제자리 적재함에서 잰 값 − 칸 표)
    // LOOK slot left · right: LOOK 자세에서 J1 만 이만큼 돌려 박스 옆 끝 퍽을 봄 [°, left = +] (11-09: y ±0.07 넘는 퍽이 LOOK 화면 밖). 실행 중 바꾸면 다음 LOOK 부터
    declare_parameter<double>("look_side_deg", 18.0);
    declare_parameter<double>("look_side_tilt_deg", 0.0);   // 양수 = J4 감소, 먼 쪽을 봄
    declare_parameter<double>("look_far_tilt_deg", 0.0);
    declare_parameter<double>("look_near_tilt_deg", 0.0);   // 양수 = J3 증가 (J4 는 정면 LOOK 에서 모델 상한)
    declare_parameter<bool>("pick_end_at_staging", false);       // PICK 끝에 박스 위 기준점으로 (false = 집은 자리에서 끝, STORE · PLACE 가 이어 감)
    declare_parameter<bool>("place_far_regrasp", true);         // 먼 자리 PLACE: 가까운 빈 자리에 내려놓고 손을 기울여 다시 쥔 뒤 놓기 (place_far_plan)
    // 먼 자리 다시 쥐기의 열기 (far_re_above) 를 팔을 멈추지 않고 (내려놓은 뒤 빈손, PICK 퍽 위로 가며 열기와 같음) — 한 번에 약 1.1 s. 실물 확인 전이라 끔
    declare_parameter<bool>("place_far_open_on_way", false);
    declare_parameter<double>("park_max_tilt_deg", 30.0);       // 먼 누운 퍽: 쥘 때 손을 10° 씩 이만큼까지 기울여 봄 (0 = 수직 손만)
    declare_parameter<double>("park_max_roll_deg", 12.0);       // 기울인 손에 퍽이 굴러 맞는 각 최대 (시뮬: 20° 는 들다 놓침)
    declare_parameter<double>("park_roll_margin_mm", 2.5);      // 기울인 손에 퍽이 굴러 맞을 때 들어가는 열린 손가락 ↔ 퍽 최소 여유
    // 먼 · 옆 누운 퍽 (11-09 21:4x: 계획 거절 · 끝 쪽 쥐기 놓침): 가까운 자리 밖이면 퍽 가운데를 누운 채 쥐어 가까운 빈 자리에 옮겨 놓고 거기서 세움 (park_or_relocate_plan)
    declare_parameter<bool>("park_relocate", true);
    declare_parameter<bool>("park_box_regrip", true);
    declare_parameter<bool>("park_table_fallback", true);
    declare_parameter<bool>("farpick_regrasp", true);
    declare_parameter<bool>("pick_prefer_tangent", true);   // 이웃 없는 PICK: 손목을 덜 돌리는 기본 닫힘 방향 먼저 (손목 90° 돌림 보정 오차 피함)
    declare_parameter<double>("regrasp_put_clear_mm", 8.0);   // 다시 쥐기 전 내려놓을 때 퍽 바닥 ↔ 박스 윗면 [mm] (PLACE 는 place_bottom_clear_mm 그대로)
    declare_parameter<double>("farpick_regrasp_tilt_dev", 5.0);   // 표로 쥔 기울임과 놓는 손 기울임 차 허용 [°] (놓는 손은 10° 단계)   // 먼 퍽 표로 기울여 쥔 퍽은 가까이 세워 내려놓고 수직으로 다시 쥠 (박스 퍽처럼 50 mm)   // 받침 세우기 · 옮겨 놓기가 다 안 되는 먼 누운 퍽은 강화학습 세우기 표 (standup_plan) 로   // 세운 퍽을 박스 위에 내려놓고 50 mm 로 다시 쥠 (칸 넣기 높이 · 값을 박스 퍽과 같게)
    // 가까운 자리 = J1 에서 이 거리 안 [m]: 실물로 된 (0.225, −0.050) 은 안, 안 된 (0.272, −0.055) 부터는 밖이 되게
    declare_parameter<double>("park_near_max_r_m", 0.24);
    standup_speed_dps_ = declare_parameter<double>("standup_speed_dps", 32.0);   // 시뮬 정책은 40 °/s (스텝 0.05 s 마다 2°). 1.25 배 느리게 재생해도 시뮬 91.8 % (40 은 89.9 %)
    // 세운 뒤 바로 다시 쥠 (true) · 손을 들고 LOOK 자세에서 끝냄 (false, 기본: 퍽은 선 채 박스에, 다음 pick det 로 카메라로 다시 재서 집음).
    //   시뮬 재생에서 선 자리가 표와 90 % 가 12 mm 안 (손가락이 받는 폭 약 ±8 mm) → 바로 다시 쥐면 가끔 놓침
    declare_parameter<bool>("standup_regrasp", false);
    standup_max_gap_mm_ = declare_parameter<double>("standup_max_gap_mm", 6.0);
    // 쥐기 뒤 확인: 손끝이 쥐는 자리에서 이보다 멀면 (퍽 위에 얹힘) · 그리퍼가 이보다 닫히면 (빈손) 세우기를 안 하고 멈춤
    standup_reach_tol_mm_ = declare_parameter<double>("standup_reach_tol_mm", 6.0);
    standup_grip_min_m_ = declare_parameter<double>("standup_grip_min_m", 0.001);
    // 쥐기 (닫기) 뒤 빈손 문턱 [m] (pick_path · run_arm): 실측이 이보다 더 닫혔으면 퍽을 못 쥔 것. 기본 = gripper_close (−0.0016) 와 퍽에 닿은 자리
    //   (약 0.0046, park_loose_grip_m 주석) 의 가운데 = home_plan 의 반쯤 닫힘 기준 (gripper_close + 0.003). 작업마다 다시 읽음 (build_plan), −1 = 끔
    grasp_empty_m_ = declare_parameter<double>("grasp_empty_m", 0.0015);
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
      // 먼 · 옆 퍽 붙은 선 퍽 집기 표 (11-10 새벽 강화학습, 시뮬 88.9 %): config/farpick/ 의 이웃 방향별 9 개 (farpick_table_name) — 없으면 경고만 (기능 끔)
      std::string dir = declare_parameter<std::string>("farpick_dir", "");
      if (dir.empty() && !f.empty()) {dir = (std::filesystem::path(f).parent_path() / "farpick").string();}   // launch 가 standup_paths_file 을 install config 경로로 줌
      declare_parameter<std::string>("farpick_mode", "auto");
    declare_parameter<double>("retrieve_lift_clear_mm", 32.0);   // RETRIEVE 쥔 뒤 칸 안에서 곧게 올리는 높이 [mm] (0 = 끔, 실행 중 바꿈)
    declare_parameter<double>("store_lift_clear_mm", 0.0);   // STORE 놓은 뒤 칸 안에서 곧게 올리는 높이 [mm] (0 = 끔, 실행 중 바꿈) — 벌린 손가락이 놓은 퍽을 끌고 나오지 않게
    // STORE 놓은 뒤 첫 닫기를 손끝이 놓는 점보다 이만큼 높거나 · 수평 이만큼 벗어난 첫 점까지 늦춤 [mm] (0 = 그 조건 안 봄, 둘 다 0 = 지금 그대로,
    //   작업마다 다시 읽음 — ros2 param set 이 다음 STORE 부터). 11-10 실물: 덜 들어간 퍽을 10 mm 올라온 자리에서 닫아 다시 집어 올림
    declare_parameter<double>("store_close_rise_mm", 50.0);
    declare_parameter<double>("store_close_side_mm", 40.0);
    declare_parameter<std::vector<double>>("regrasp_offset_mm", std::vector<double>{0.0, 0.0});   // 세워 놓고 수직으로 다시 쥐는 점 [앞, 왼쪽] mm (실행 중 바꿈)
    declare_parameter<double>("farpick_face_tol_deg", 10.0);
      declare_parameter<double>("farpick_nbr_max_m", 0.06);
      declare_parameter<double>("farpick_max_gap_mm", 12.0);
      declare_parameter<double>("farpick_max_gap_deg", 8.0);
      declare_parameter<double>("farpick_touch_mm", 10.0);
      if (dir.empty()) {
        RCLCPP_WARN(get_logger(), "먼 퍽 표 폴더 없음 (farpick_dir · standup_paths_file) → 먼 퍽 표 안 씀");
      } else {
        for (int i = -1; i < 8; ++i) {
          const std::string name = farpick_table_name(i >= 0, i * 45.0);
          std::vector<StandupPath> t;
          if (load_standup_table(dir + "/" + name, t, w)) {farpick_[name] = std::move(t);}
          else {RCLCPP_WARN(get_logger(), "%s → 그 표 안 씀", w.c_str());}
        }
        RCLCPP_INFO(get_logger(), "먼 퍽 표: %zu/9 개 (%s)%s", farpick_.size(), dir.c_str(), farpick_.empty() ? " → 먼 퍽 표 안 씀" : "");
      }
      // 먼 누운 네모 퍽 가운데 쥐기 표 (11-10 밤 강화학습 lyingpick_res, 실물 220632 빈손): park_or_relocate_plan ① 옮기기 쥐기 · 들기를 표 재생으로.
      //   lyingpick_table "" = standup_paths_file 과 같은 폴더의 lyingpick/lyingpick_paths_ok75.txt (farpick_dir 과 같은 규칙). 없거나 비면 지금 쥐기 그대로
      declare_parameter<bool>("lyingpick_use", true);
      declare_parameter<double>("lyingpick_max_dist_mm", 12.0);   // 표 칸 ↔ 잰 퍽 거리 한도 (farpick_max_gap_mm 과 같은 값)
      declare_parameter<double>("lyingpick_max_gap_deg", 8.0);    // 긴축 차 한도 (180° 주기, farpick_max_gap_deg 와 같은 값 · 표 칸 15° 의 반 넘게)
      // 실물 맞춤 (작업마다 다시 읽음): 표 경로의 퍽 근처 구간 (내려가기 · 닫기 · 쥔 채 들기) 손끝을 더 옮김 [mm]. dz 음수 = 더 깊게, dxy = base_link [x, y]
      declare_parameter<double>("lyingpick_dz_mm", 0.0);
    declare_parameter<bool>("lyingpick_recenter", true);       // 표 쥐기 손끝을 닫힘축으로 퍽 가운데에 (긴축 방향은 표 그대로). false = 표 그대로 (실행 중 바꿈)
    declare_parameter<double>("lyingpick_hover_mm", 0.0);    // > 0: 표로 쥘 때 닫는 점 위 이만큼에서 멈췄다가 곧게 내려와 쥠 [mm] (기울인 손이 퍽 옆면을 스치지 않게, 실행 중 바꿈)
    declare_parameter<double>("lyingpick_hover_s", 0.3);     // 그 멈춤 시간 [s]
    declare_parameter<double>("lyingpick_descend_s", 0.5);   // 멈춘 점 → 닫는 점 내려오는 시간 [s]
      declare_parameter<std::vector<double>>("lyingpick_dxy_mm", std::vector<double>{0.0, 0.0});
      const std::string lp = lyingpick_table_path(declare_parameter<std::string>("lyingpick_table", ""), f);
      if (lp.empty()) {
        RCLCPP_WARN(get_logger(), "누운 퍽 가운데 쥐기 표 경로 없음 (lyingpick_table · standup_paths_file) → 옮기기 쥐기는 지금 그대로");
      } else if (!load_standup_table(lp, lyingpick_, w)) {
        lyingpick_.clear();
        RCLCPP_WARN(get_logger(), "누운 퍽 가운데 쥐기 표: %s → 옮기기 쥐기는 지금 그대로", w.c_str());
      } else {
        const auto n = std::count_if(lyingpick_.begin(), lyingpick_.end(), [](const StandupPath & p) {return std::abs(p.side - kLyingPickSide) < 1e-4;});
        RCLCPP_INFO(get_logger(), "누운 퍽 가운데 쥐기 표: %zu 줄 (누운 퍽 side 0.060 %ld 줄, %s)%s", lyingpick_.size(), static_cast<long>(n), lp.c_str(),
          n == 0 ? " → 쓸 줄 없음, 옮기기 쥐기는 지금 그대로" : "");
      }
    }
    declare_parameter<std::string>("held_reference_file", "~/.ros/held_reference.txt");
    // 손 안 위치로 STORE 경로 옮기기 (false = 재서 로그만). 실행 중: ros2 param set /control_master held_correct true
    declare_parameter<bool>("held_correct", false);
    declare_parameter<bool>("held_correct_fresh", false);   // 새 신선한 측정+쥔 깊이 차이 정규화, false = 기존 동작 그대로
    // 같은 보정을 PLACE 놓는 점에도 (바로 놓기만, 다시 쥐는 먼 자리는 안 함 — 다시 쥐면 손 안 위치가 바뀜). 실행 중: ros2 param set /control_master place_held_correct true
    declare_parameter<bool>("place_held_correct", false);
    held_max_mm_ = declare_parameter<double>("held_max_mm", 8.0);   // 기준과 이보다 다르면 잘못 잰 것으로 보고 안 옮김
    load_held_reference();
    sub_held_ = create_subscription<interfaces::msg::DetectionArray>(
      declare_parameter<std::string>("held_tag_topic", "/held_puck/tag"), rclcpp::QoS(10),
      [this](interfaces::msg::DetectionArray::ConstSharedPtr m) {on_held_tag(*m);});
    sub_held_fresh_ = create_subscription<interfaces::msg::DetectionArray>("/held_puck/tag_fresh", rclcpp::QoS(10),
      [this](interfaces::msg::DetectionArray::ConstSharedPtr m) {on_held_fresh(*m);});
    srv_held_ref_ = create_service<std_srvs::srv::Trigger>("~/save_held_reference",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        res->success = save_held_reference(res->message);
      });
    // 쥔 퍽 영상 검사 (11-11 오후 실물: 박스에서 앞으로 쥔 퍽이 손 안에서 대각선 → 넣기 전에 고치기). 기본 꺼짐
    //   PICK (선 퍽 수직 손, 다시 잡기 없는 것) 이 들기 (lift) 끝에서 멈춰 perception /held_puck/image (쥔 퍽 분할 마스크 픽셀, wrist.held) 를
    //   잘 쥔 기준 (~/save_held_image_ref, 모양별) 과 비교 → 앞으로 쥠 > held_image_tol_mm · 뒤로 쥠 > held_image_back_tol_mm ·
    //   기울기 > held_image_tilt_deg 면 쥐어 간 박스 자리에 내려놓고 다시 보고 (regrasp_visual) 다시 쥠 (store_recover 와 같은 조각,
    //   held_image_retry_max 번까지) → 다시 쥔 뒤 한 번 더 재서 로그 → PICK 끝 그대로 (그다음 STORE 등 그대로)
    //   perception 도 켜야 함: ros2 param set /perception wrist.held.enable true. 모든 값은 실행 중 ros2 param set 바로 (다음 PICK 부터)
    declare_parameter<bool>("held_image", false);
    declare_parameter<double>("held_image_tol_mm", 3.0);        // 앞으로 쥔 허용 [mm]
    declare_parameter<double>("held_image_back_tol_mm", 6.0);   // 뒤로 쥔 허용 [mm] (11-11: 5 mm 뒤로 쥔 퍽은 S12 에 잘 들어감). 0 = tol 과 같게
    declare_parameter<double>("held_image_tilt_deg", 5.0);      // 기울기 허용 [°] (왼쪽 = 손가락 쪽 모서리 각 차)
    declare_parameter<int>("held_image_retry_max", 1);          // 박스에 내려놓고 다시 쥐는 횟수 (0 = 재서 로그만)
    declare_parameter<bool>("held_image_force", false);         // 시험용: 첫 검사를 무조건 잘못 쥔 것으로 봄 (흐름 확인, 다시 쥔 뒤 검사는 실제)
    declare_parameter<double>("held_image_settle_s", 0.3);      // 들기 끝에서 멈춰 재는 시간 (멈춘 뒤 0.1 s 부터의 장, 2 장 모이면 판단)
    declare_parameter<double>("held_image_timeout_s", 1.0);     // 이만큼 기다려도 2 장이 안 오면 있는 장으로 (0 장이면 그대로 진행)
    declare_parameter<std::vector<double>>("held_image_px_per_mm", std::vector<double>{5.2, 14.7});   // 위 모서리 [앞, 뒤] px/mm (11-11 실물 ±5 mm)
    declare_parameter<std::string>("held_image_ref_file", "~/.ros/held_image_ref.txt");
    load_held_image_refs_now();
    sub_held_img_ = create_subscription<interfaces::msg::DetectionArray>(
      declare_parameter<std::string>("held_image_topic", "/held_puck/image"), rclcpp::QoS(10),
      [this](interfaces::msg::DetectionArray::ConstSharedPtr m) {on_held_image(*m);});
    srv_held_img_ref_ = create_service<std_srvs::srv::Trigger>("~/save_held_image_ref",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        res->success = save_held_image_ref(res->message);
      });
    gg_declare_params();   // grasp_guard: 쥔 폭 · 손 안 퍽 · 넣기 걸림 · 넣은 뒤
    slow_dps_ = declare_parameter<double>("near_puck_speed_dps", 15.0);
    path_slow_min_dps_ = declare_parameter<double>("path_slow_min_dps", 0.0);   // 실물 경로 느린 구간의 최소 속도 (0 = 기록 그대로 15)
    // 경로별 속도 비율 "경로 이름=비율" (S2 로 갈 때 10 % 느리게 → "store_S2=0.9"). 실행 중 바꾸면 다음 작업부터
    declare_parameter<std::vector<std::string>>("path_speed_scale", std::vector<std::string>{});
    model_ok_ = load_model();
    for (const char * p : {"home", "pick_above", "pick_down", "place_above", "place_down"}) {
      poses_[p] = declare_parameter<std::vector<double>>(
        std::string("poses.") + p, std::vector<double>(arm_joints_.size(), 0.0));
    }
    // HOME 만 J1 · J5 옮김 (home_j1_offset_rad · home_j5_offset_rad, 켤 때 한 번). 옮기는 점 = 이름으로 정한 받침 구간만
    //   (11-10 검수: J1 · J2 조건으로 고르면 retrieve_S2 r6 · r8 등 칸 경로 중간까지 옮겨짐)
    declare_parameter<double>("home_j1_offset_rad", 0.0);
    declare_parameter<double>("home_j5_offset_rad", 0.0);
    home_q0_ = poses_.at("home");
    for (auto & [name, path] : paths_) {
      for (std::size_t i = 0; i < path.nodes.size(); ++i) {
        const auto & n = path.nodes[i];
        if (n.q.size() >= 5 && home_q0_.size() >= 5 && is_stand_node(n.name) && std::fabs(n.q[0] - home_q0_[0]) < 0.06) {
          home_nodes_.push_back({name, i});
        }
      }
    }
    apply_home_offsets();
    cancel_timeout_s_ = declare_parameter<double>("cancel_timeout_s", 2.0);
    // ---- 접힘 실측 확인 (R1) ----
    const double verify_timeout_s = declare_parameter<double>("verify_timeout_s", 2.0);
    const auto fold_tol = declare_parameter<std::vector<double>>("fold_tolerance", {0.05});
    const double fold_hold_s = declare_parameter<double>("fold_hold_s", 0.3);
    const double fold_max_age_s = declare_parameter<double>("fold_max_age_s", 0.2);
    fold_ = std::make_unique<FoldCheck>(poses_.at("home"), fold_tol, fold_hold_s, fold_max_age_s);
    fold_tol_ = fold_tol;
    fold_hold_s_ = fold_hold_s;
    fold_max_age_s_ = fold_max_age_s;
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
    // 보드 상태 (list_hardware_components) 를 한 번도 못 받았거나 서비스가 안 보일 때 (DDS), 관절 측정이 신선 · 정상이고
    //   팔 · 그리퍼 재생기가 준비됐으면 하드웨어 정상으로 봄 (WARN 한 번). 받은 보드 상태가 이상이면 지금처럼 이상 (HardwareWatch)
    hw_->set_components_optional(declare_parameter<bool>("hw_status_optional", true));   // 실행 중 바꾸면 ask_components 가 다시 읽음
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
    //   별도 콜백 그룹: 팔 계획 (on_arm_goal, 동기 · 수 초) 이 기본 그룹을 잡고 있어도 /odom 을 계속 받음
    //   (같은 그룹이면 계획 동안 콜백이 못 돌아 odom_stamp_ 가 odom_timeout_s 를 넘고 → 계획 직후 "/odom 이 안 옴" 으로 팔 시작 거절)
    odom_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions odom_opt;
    odom_opt.callback_group = odom_group_;
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
        const rclcpp::Time stamp(m->header.stamp);
        const auto & t = m->twist.twist;
        const bool still = std::hypot(t.linear.x, t.linear.y) < still_lin_ && std::fabs(t.angular.z) < still_ang_;
        std::lock_guard<std::mutex> lk(odom_mtx_);   // step() 이 다른 스레드에서 읽음
        if (have_odom_ && stamp <= odom_stamp_) {return;}
        odom_still_ = still;
        odom_stamp_ = stamp;
        have_odom_ = true;
      }, odom_opt);
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
        gg_feed_joints(*m);   // grasp_guard 넣기 감시 (켜져 있을 때만)
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
    // 팔 계획 (on_arm_goal, 동기) 동안에도 마지막 차례 메시지를 재발행 (heartbeat): 별도 콜백 그룹 + main 의 MultiThreadedExecutor (스레드 2).
    //   /odom 구독도 별도 그룹 (odom_group_). 나머지 콜백은 모두 기본 그룹 (서로 동시 실행 안 됨) 그대로
    hb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    hb_timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(0.1), [this]() {heartbeat();}, hb_group_);

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
    //   하드웨어 감시가 '측정 끊김' 으로 오판하지 않게 잠깐 이전 판정을 유지 (실물: 0.59 s 로 PICK 중단)
    const auto t_plan = std::chrono::steady_clock::now();
    pending_step_ = false;
    pending_plan_ = build_plan(goal, why);
    const double plan_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_plan).count();
    hw_grace_until_ = now().seconds() + 0.3;
    if (plan_s > 0.2) {RCLCPP_INFO(get_logger(), "팔 작업 %u 계획 %.2f s", static_cast<unsigned>(goal.task), plan_s);}
    if (pending_plan_.empty()) {
      RCLCPP_WARN(get_logger(), "팔 작업 %u%s%s 거절: %s", static_cast<unsigned>(goal.task),
        goal.slot.empty() ? "" : " 칸 ", goal.slot.c_str(), why.c_str());
      // 거절 이유를 /control/motion_mode reason 에 잠깐 실음 (액션 거절 응답엔 글이 없음 → planning 이 IK 거절이면 재시도 안 하게)
      reject_why_ = why;
      reject_until_s_ = now().seconds() + kRejectReasonS;
      publish_turn();   // 거절 응답보다 먼저 닿게
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
    if (auto_folding_) {   // P0 쉬기 · 실패 뒤 자동 접기 둘 다: 새 작업은 거절 (ready 꺼짐) → planning 이 접힌 뒤 다시 보냄
      why = fail_folding_ ? "팔 작업 실패 뒤 자동으로 접는 중 (planning 이 다시 보냄)" : "P0 에서 쉬다 자동으로 접는 중 (planning 이 다시 보냄)";
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
    fail_since_ = -1.0;            // 실패 뒤 자동 접기 기다림 끝 (새 작업이 옴)
    empty_why_.clear();            // 빈손 확인 · 되돌아가기는 작업마다 (계획은 이미 만들어 둠)
    empty_retreat_ = ArmPath{};
    empty_retreat_note_.clear();
    park_empty_ = false;
    job_step_ = pending_step_;
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
        const bool observe = !a.is_gripper && (a.pose == kRegraspObserve || a.pose == kHeldImageCheck);   // 논리 대기 (모터 궤적 없음): 한계만 적고 합에 안 넣음
        const double t = a.is_gripper ? gripper_time_s_ : (a.points.empty() && !observe ? move_time_s_ : a.duration_s);
        if (!observe) {total += t;}
        std::snprintf(buf, sizeof(buf), "%s%s %.1f", line.empty() ? "" : " · ", a.is_gripper ? "[그리퍼]" : (observe ? (a.pose == kHeldImageCheck ? "[쥔 퍽 영상 ≤]" : "[다시 보기 ≤]") : a.pose.c_str()), t);
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
  //   LOOK slot left/right/far/near: 정면 LOOK 기준 J1 옆 보기 · J4 먼 보기 · J3 가까운 보기. 새 기울임 0 은 기존 점 그대로.
  //     slot "" 을 옆 자세에서 받으면 J1 을 되돌려 LOOK 자세로. 다른 작업 (PICK 등) 은 옆 자세를 시작 자세로 안 봄 (거절 → 미션이 LOOK 으로 되돌림)
  //   LOOK slot "rack": 빈 손 적재함 스캔 (경로 rack_scan) — 받침 → 칸 두 개씩 손목 카메라로 내려다보며 2 s 씩 → 받침 (주행 허용)
  //     칸 바닥 태그로 칸 위치를 잼 (manipulation rack_tag_node). 퍽 쥐면 거절. 칸에 퍽이 있어도 경로는 퍽 위로 지나감 (그 칸은 못 잼)
  //   HOME: 접혀 있으면 home 한 점, 아니면 지금 자세에서 출발하는 검사된 복귀 경로(home_from_*), 없으면 home 한 점
  //   PICK: 받침 → 펼치기 → 퍽 위 40 열기 → 10 → 쥐기 −40 닫기 → 10 → 40 → 박스 위 기준점 (퍽 쥠, 바퀴 잠금 유지 — 이어서 STORE)
  //     손을 10° 넘게 기울여야 하면 (적재함에 못 넣는 먼 퍽) pick_tilted_depth_mm (30) = 퍽 무게중심을 쥠 — 더 깊으면 원기둥이 손가락 사이에서 굴러 넘어짐 (시뮬)
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
    visual_enabled_ = get_parameter("regrasp_visual").as_bool();   // 다시 보고 쥐기: 이 작업 계획부터
    visual_waiting_ = false;
    visual_tail_ = {};
    visual_obs_q_.clear();
    visual_note_.clear();
    visual_suffix_.clear();
    visual_id_.clear();
    store_recover_reset();
    held_img_ = HeldImageRun{};
    // 속도는 작업마다 다시 읽음 (실행 중 ros2 param set 이 다음 작업부터)
    slow_dps_ = get_parameter("near_puck_speed_dps").as_double();
    path_slow_min_dps_ = get_parameter("path_slow_min_dps").as_double();
    side_grip_ = get_parameter("gripper_on_the_way").as_bool();
    gripper_time_s_ = get_parameter("gripper_time_s").as_double();
    grasp_empty_m_ = get_parameter("grasp_empty_m").as_double();
    grasp_checks_.clear();   // 빈손 확인 지점은 pick_path 가 다시 만듦 (다른 작업에 남지 않게)
    gg_regrip_.clear();      // grasp_guard 다시 쥐기 정보도 같이
    gg_last_close_.reset();  // standup 처럼 set_grasp_checks 를 안 부르는 PICK 에 앞 PICK 닫기가 남지 않게
    grasp_empty_why_.clear();
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
      case G::STORE: {
        auto plan = store_plan(goal, why);
        gg_held_before_rack(goal, plan, true);   // grasp_guard: 손 안 퍽 한 줄 (held_regrasp 면 다시 쥐기)
        return plan;
      }
      case kArmLookRack:   // 퍽 쥔 채 그 칸을 손목 카메라로 내려다봄 (look_rack_<칸>, 끝나면 그 자세 유지 → STORE)
        if (!carrying_) {why = "LOOK_RACK 은 퍽을 쥔 채만 (빈손이면 LOOK slot rack)"; return {};}
        if (held_lying_) {why = "누운 퍽은 적재함에 못 넣음 — LOOK_RACK 안 함"; return {};}
        if (held_tilt_ > kStoreMaxTilt + 1e-6) {why = "손을 기울여 쥔 퍽은 적재함에 못 넣음 — LOOK_RACK 안 함"; return {};}
        if (!paths_.count("look_rack_" + goal.slot)) {why = "LOOK_RACK " + goal.slot + ": 경로 없음 (config/rack_look_paths.yaml)"; return {};}
        {
          auto plan = path_plan("look_rack_" + goal.slot, why);
          gg_held_before_rack(goal, plan, false);   // grasp_guard: 손 안 퍽 한 줄 (held_regrasp 면 다시 쥐기)
          return plan;
        }
      case G::RETRIEVE: {
        std::string name = "retrieve_" + goal.slot;
        retrieve_full_ = false;
        if (get_parameter("storeheld_paths").as_bool()) {   // 다 찬 적재함에서 빼기 (retrieve_<칸>_full, 칸 보정 포함) 가 있으면 그것
          const std::string n = storeheld_retrieve_name(goal.slot, [this](const std::string & p) {return paths_.count(p) > 0;});
          if (!n.empty()) {
            name = n;
            retrieve_full_ = true;
            RCLCPP_INFO(get_logger(), "RETRIEVE %s: 새 칸 경로 %s (storeheld_paths)", goal.slot.c_str(), name.c_str());
          }
        }
        return path_plan(name, why, true);
      }
      case G::LOOK:
        if (goal.slot == "rack") {
          if (carrying_) {why = "퍽을 쥔 채 적재함 스캔 안 함 (빈 손으로만)"; return {};}
          return path_plan("rack_scan", why, false, true);   // lazy_fold 여도 받침까지 (P0 에서 오래 버티지 않게)
        }
        if (goal.slot == "left" || goal.slot == "right" || goal.slot == "far" || goal.slot == "near") {return look_variant_plan(goal.slot, why);}
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

  // STORE 칸 경로 계획 (build_plan · store_recover 다시 넣기가 같이 씀). 손 안 퍽 한 줄 (gg_held_before_rack) 은 부르는 쪽이
  std::vector<ArmAction> store_plan(const ArmTask::Goal & goal, std::string & why)
  {
        if (held_lying_) {why = "누운 퍽은 적재함에 못 넣음 (세우기 필요) — PLACE 로 내려놓기"; return {};}
        {
          // 새 칸 경로 (storeheld_paths, cell_shift.hpp): 쥔 깊이 · 기울임으로 store_<칸>_d50 · _d30 (_d30_low) · _d40_tilt<n>. "" = 없음 → 지금 이름 · 지금 흐름
          std::string held_name;
          if (get_parameter("storeheld_paths").as_bool()) {
            held_name = storeheld_store_name(goal.slot, held_depth_mm_, get_parameter("park_regrasp_depth_mm").as_double(), held_tilt_,
              get_parameter("storeheld_low").as_bool(), [this](const std::string & n) {return paths_.count(n) > 0;});
          }
          if (held_name.empty() && held_tilt_ > kStoreMaxTilt + 1e-6) {   // 적재함 경로는 수직 퍽으로 검증됨 (기울임 경로 _d40_tilt 가 있으면 건너뜀)
            char buf[160];
            std::snprintf(buf, sizeof(buf), "손을 %.0f° 기울여 집은 퍽은 적재함에 못 넣음 (10° 까지) — 가까운 자리에 놓고 다시 집기", held_tilt_);
            why = buf;
            return {};
          }
          // LOOK_RACK (작업 8) 뒤 관찰 자세에 있으면 store_look_<칸> (관찰 → 컵 입구 위 → 넣기), target = 그때 잰 칸 바닥 중심 (base_link)
          measured_cell_.reset();
          const auto & tp = goal.target.pose.position;
          if (goal.target.header.frame_id == "base_link" && std::isfinite(tp.x) && std::isfinite(tp.y) && (tp.x != 0.0 || tp.y != 0.0)) {
            if (get_parameter("store_use_look_rack").as_bool()) {
              measured_cell_ = Eigen::Vector2d(tp.x, tp.y);
            } else {
              RCLCPP_INFO(get_logger(), "STORE %s: LOOK_RACK 로 잰 칸 (%.1f, %.1f) mm 은 로그만 (store_use_look_rack false) → 빈손 스캔 맞춤으로 옮김",
                goal.slot.c_str(), tp.x * 1000.0, tp.y * 1000.0);
            }
          }
          std::string name = "store_" + goal.slot;
          const auto lk = paths_.find("store_look_" + goal.slot);
          std::string dummy;
          // 관찰 자세여도 새 경로가 있고 storeheld_over_look 이면 새 경로 (store_look_<칸> 흐름 자체는 그대로)
          const bool over_look = !held_name.empty() && get_parameter("storeheld_over_look").as_bool();
          if (!over_look && look_rack_slot_ == goal.slot && lk != paths_.end() && path_start_ok(lk->second, joint_buf_, path_start_tol_, dummy)) {
            name = "store_look_" + goal.slot;
            RCLCPP_INFO(get_logger(), "STORE %s: 관찰 자세에서 바로 칸으로 (%s)", goal.slot.c_str(), measured_cell_ ? "잰 칸 위치로" : "칸 측정 없음 → 칸 표");
          } else if (!held_name.empty()) {
            name = held_name;
            RCLCPP_INFO(get_logger(), "STORE %s: 새 칸 경로 %s (쥔 깊이 %.0f mm · 기울임 %.0f°, storeheld_paths)", goal.slot.c_str(), name.c_str(),
              held_depth_mm_, held_tilt_);
          } else if (measured_cell_) {
            RCLCPP_INFO(get_logger(), "STORE %s: 칸 측정은 받았지만 관찰 자세가 아님 → 기록 경로 (store_%s)", goal.slot.c_str(), goal.slot.c_str());
          }
          auto plan = path_plan(name, why);
          measured_cell_.reset();
          return plan;
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
    apply_held_fresh(name,path);
    // STORE: 놓은 뒤 첫 닫기를 손끝이 충분히 올라오거나 칸에서 비킨 점으로 늦춤 (store_close_rise_mm · store_close_side_mm). 칸 보정 · 곧게 올리기 뒤 점으로 잼
    if (name.rfind("store_", 0) == 0) {store_close_later(path, name.c_str());}
    // STORE: 놓은 뒤 닫기는 팔을 멈추지 않고 (빈손으로 받침 쪽 가며) — 그리퍼 기다림 하나를 뺌
    if (side_grip_ && name.rfind("store_", 0) == 0) {   // 놓기 뒤 닫기
      bool released = false;
      for (auto & n : path.nodes) {
        if (n.grip == "release" || n.grip == "open") {released = true;}
        else if (released && n.grip == "close") {n.grip_on_way = true;}
      }
    }
    // 칸 위 (마지막 내려가기 직전) 에서 멈췄다 넣기 (store_settle_s, 0 = 그대로). rack_adjust 로 옮긴 뒤 · 속도 조정 전 (stop + hold 만 더함)
    if (name.rfind("store_", 0) == 0) {store_settle(path, name.c_str());}
    // 실물 기록의 느린 구간 (칸 근처 15 °/s) 을 path_slow_min_dps 까지 올림 (적재가 너무 느림, 적당히 빠르게)
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
    if (name.rfind("store_", 0) == 0) {store_path_ = path;}   // store_recover: 걸리면 이 경로를 거꾸로 (칸 보정 · 곧게 올리기 뒤 그대로)
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
    const char * from = "관찰 자세";
    PathNode viewing;
    if (at_look_variant(viewing)) {p.nodes.front() = viewing; from = "보기 변형 자세";}
    else if (!path_start_ok(p, joint_buf_, path_start_tol_, dummy)) {return false;}
    path = p;
    RCLCPP_INFO(get_logger(), "%s: %s에서 바로 퍽 위로 (P0 · 받침 안 거침)", what, from);
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
      if (back == paths_.end()) {continue;}
      ArmPath returning = back->second;
      PathNode viewing;
      if (std::string(name) == "home_from_look" && at_look_variant(viewing)) {returning = manipulation::look_home_bridge(viewing, paths_.at("look").nodes.back(), returning);}
      if (!path_start_ok(returning, joint_buf_, path_start_tol_, dummy)) {continue;}
      ArmPath to_p0 = returning;
      ArmPath rest = path;
      if (p0_join_ok_ && manipulation::cut_fold_tail(to_p0, fold->second) && manipulation::skip_unfold(rest, unfold->second)) {
        to_p0.nodes.insert(to_p0.nodes.end(), rest.nodes.begin() + 1, rest.nodes.end());
        path.nodes = to_p0.nodes;
        RCLCPP_INFO(get_logger(), "%s: 관찰 자세 → P0 에서 바로 (받침까지 접지 않음)", what);
        return true;
      }
      std::vector<PathNode> nodes = returning.nodes;
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
    PathNode viewing;
    if (name == "look" && at_look_variant(viewing)) {
      RCLCPP_INFO(get_logger(), "LOOK: 보기 변형에서 정확한 정면 LOOK 으로 되돌림");
      return {look_move(at_end.nodes[0], joint_buf_)};
    }
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

  // J1 만 바뀌면 기존 옆 보기 시간과 같음. 기울임이 있으면 가장 많이 움직이는 관절로 시간을 정함.
  ArmAction look_move(const PathNode & to, const std::vector<double> & from) const
  {
    double dq = 0.0;
    if (from.size() == to.q.size()) {
      for (size_t j = 0; j < from.size(); ++j) {
        if (std::isfinite(from[j])) {dq = std::max(dq, std::fabs(to.q[j] - from[j]));}
      }
    }
    const double t = std::max(0.5, dq * 180.0 / M_PI / std::max(1.0, timing_.stand_speed_dps));
    ArmAction a{"approach", false, to.name, 0.0};
    a.points = {{to.q, t}};
    a.duration_s = t;
    return a;
  }

  bool look_variant_node(const std::string & slot, PathNode & n, std::string & why) const
  {
    const auto look = paths_.find("look");
    if (look == paths_.end() || look->second.nodes.empty()) {why = "경로 look 없음"; return false;}
    manipulation::LookOptions o;
    o.side_deg = get_parameter("look_side_deg").as_double();
    o.side_tilt_deg = get_parameter("look_side_tilt_deg").as_double();
    o.far_tilt_deg = get_parameter("look_far_tilt_deg").as_double();
    o.near_tilt_deg = get_parameter("look_near_tilt_deg").as_double();
    return manipulation::make_look_variant(look->second.nodes.back(), slot, o, model_ok_ ? &model_ : nullptr, n, why);
  }

  bool at_look_variant(PathNode & n) const {return at_look_variant(n, joint_buf_);}
  // q: 지금 관절값 대신 그 자세 (빈손 되돌아가기의 경로 점 — empty_retreat_path)
  bool at_look_variant(PathNode & n, const std::vector<double> & q) const
  {
    const auto look = paths_.find("look");
    if (look == paths_.end() || look->second.nodes.empty()) {return false;}
    // 실행 중 파라미터를 바꿔도 방금 간 보기 자세를 잊지 않음. 새 값만으로 현재 자세를 추측하지 않음.
    std::vector<PathNode> variants = look_history_;
    for (const char * slot : {"left", "right", "far", "near"}) {
      PathNode v; std::string why;
      if (look_variant_node(slot, v, why)) {variants.push_back(v);}
    }
    return manipulation::match_look_variant(look->second.nodes.back(), variants, q, path_start_tol_, n);
  }

  // 그 자세에서 HOME 이 쓰는 검사된 복귀 경로 (home_plan 과 같은 차례): 보기 변형 → 정면 LOOK → home_from_look,
  //   아니면 출발 자세가 path_start_tol 안인 home_from_* · fold (P0 → 받침). 없으면 false (out 은 그대로)
  bool checked_return_from(const std::vector<double> & q, ArmPath & out) const
  {
    PathNode viewing;
    const auto front_home = paths_.find("home_from_look");
    if (front_home != paths_.end() && !front_home->second.nodes.empty() && at_look_variant(viewing, q)) {
      out = manipulation::look_home_bridge(viewing, paths_.at("look").nodes.back(), front_home->second);
      return true;
    }
    for (const auto & [name, path] : paths_) {
      std::string dummy;
      const bool home_path = name.rfind("home_from_", 0) == 0 || name == "fold";
      if (home_path && path.ends_home && path.nodes.size() >= 2 && path_start_ok(path, q, path_start_tol_, dummy)) {
        out = path;
        return true;
      }
    }
    return false;
  }

  // 빈손 쥐기 뒤 온 길로 되돌아가는 경로 (set_grasp_checks, 계획 때 닫기마다): path.nodes[from] (쥐기 높이 위 점) 에서
  //   지나온 길을 거꾸로 (내려가 쥐었다 올라온 고리는 뺌, 퍽 위 · 들어 올린 점만) 검사된 복귀 경로가 있는 자세 (P0 · LOOK · 보기 변형) 까지
  //   → 그 복귀 경로 (fold · home_from_look) 로 받침. 모르는 점 (받침 세우기 · 표 재생 구간 등) 을 지나야 하면 빈 경로 (안 씀)
  ArmPath empty_retreat_path(const ArmPath & path, std::size_t from, std::string & note) const
  {
    static const std::vector<std::string> kFree{"above", "lift", "reloc_above", "reloc_lift", "reloc_put_above", "reloc_put_clear",
      "re_above", "re_lift", "re_place_above", "re_place_clear"};
    ArmPath ret;
    std::vector<std::string> free = kFree;   // + 누운 퍽 표 (lyingpick) 로 닫기 전 내려온 점 (빈손, 지금 옮기기 쥐기의 reloc_above 와 같은 몫)
    for (const auto & n : path.nodes) {if (lyingpick_free_node(n.name)) {free.push_back(n.name);}}
    const auto nodes = manipulation::retrace_nodes(path.nodes, from, free,
      [this, &ret](const PathNode & n) {return checked_return_from(n.q, ret);});
    if (nodes.empty() || ret.nodes.size() < 2) {return {};}
    ArmPath out{"empty_retreat", nodes, ret.ends_home};
    out.nodes.insert(out.nodes.end(), ret.nodes.begin() + 1, ret.nodes.end());
    note.clear();
    for (const auto & n : nodes) {note += (note.empty() ? "" : " → ") + n.name;}
    note += " → " + ret.name;
    return out;
  }

  std::vector<ArmAction> look_variant_plan(const std::string & slot, std::string & why)
  {
    PathNode target{};
    std::string fallback;
    if (!look_variant_node(slot, target, fallback)) {why = fallback; return {};}
    if (!fallback.empty()) {RCLCPP_WARN(get_logger(), "LOOK %s: %s", slot.c_str(), fallback.c_str());}
    const auto & front = paths_.at("look").nodes.back();
    if (target.q == front.q) {return look_plan("look", why);}   // far/near 0 또는 실패 = 기존 정면 LOOK 그대로
    ArmPath at_end{"look", {front}, false};
    std::string dummy;
    PathNode viewing;
    std::vector<ArmAction> plan;
    std::vector<double> from = joint_buf_;
    if (!path_start_ok(at_end, joint_buf_, path_start_tol_, dummy) && !at_look_variant(viewing)) {
      plan = look_plan("look", why);
      if (plan.empty()) {return {};}
      from = front.q;
    }
    plan.push_back(look_move(target, from));
    // 목표를 기록해 두되, 사용 때에는 항상 실제 관절값과 일치하는지 검사함 (실패/취소 뒤 추측 금지).
    look_history_.push_back(target);
    if (look_history_.size() > 32) {look_history_.erase(look_history_.begin());}
    RCLCPP_INFO(get_logger(), "LOOK %s: 정면 기준 J1 %+.1f°, J3 %+.1f°, J4 %+.1f°", slot.c_str(),
      (target.q[0]-front.q[0])*180.0/M_PI, (target.q[2]-front.q[2])*180.0/M_PI, (target.q[3]-front.q[3])*180.0/M_PI);
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
      // 빈손 쥐기로 멈춘 자리 (쥐기 높이 위 점) 면 온 길로 되돌아감 (빈손 확인 때 고른 경로: 퍽 위 → … → P0 · LOOK → 검사된 복귀 경로)
      //   예전: 경로 도중 정지 (arm_off_path_) 라 거절 → 팔이 박스 위에 뻗은 채 남음 (11-10 실물 과부하)
      if (!empty_retreat_.nodes.empty()) {
        std::string w;
        if (path_start_ok(empty_retreat_, joint_buf_, path_start_tol_, w)) {
          RCLCPP_INFO(get_logger(), "HOME: 빈손 쥐기 자리에서 온 길로 되돌아감 (%s)", empty_retreat_note_.c_str());
          home_was_path_ = true;
          return path_actions(empty_retreat_, timing_, grip_, joint_buf_);
        }
        RCLCPP_WARN(get_logger(), "HOME: 빈손 되돌아가기 안 씀 — %s", w.c_str());
      }
      PathNode viewing;
      const auto front_home = paths_.find("home_from_look");
      if (at_look_variant(viewing) && front_home != paths_.end() && !front_home->second.nodes.empty()) {
        const ArmPath path = manipulation::look_home_bridge(viewing, paths_.at("look").nodes.back(), front_home->second);
        RCLCPP_INFO(get_logger(), "HOME: 보기 변형 → 정확한 정면 LOOK → home_from_look");
        home_was_path_ = true;
        return path_actions(path, timing_, grip_, joint_buf_);
      }
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

  // 퍽 위 (내려가기 직전) 점에서 멈춰 흔들림을 가라앉힘 (pick_settle_s · place_settle_s): stop + hold_s. 0 = 그대로 (점 · 속도 · grip 그대로)
  //   가며 여는 그리퍼 (grip_early · grip_on_way) 는 path_actions 가 그 시점 그대로 열고 그 점에서 멈춤
  void settle_at(PathNode & n, const char * param) const
  {
    const double s = get_parameter(param).as_double();
    if (!std::isfinite(s) || s < 1e-3) {return;}   // 1 ms 미만은 없는 것으로 (11-09 검수 발견 4)
    n.stop = true;
    n.hold_s = std::max(n.hold_s, s);
  }

  // STORE 칸 위 멈춤 (store_settle_s): 첫 놓는 점 (release) 앞에서 손끝이 놓는 점보다 store_settle_up_mm 이상 높은 마지막 점 (= 마지막 내려가기 직전) 에
  //   settle_at 과 같은 방식 (stop + hold_s). 11-09 실물: 같은 칸이 보정을 바꿔도 매번 다른 쪽으로 어긋남 = 흔들린 채 칸 위에 와서 그대로 내려감
  //   0 = 그대로 (점 · 속도 · grip · stop 그대로). 그런 점 · 놓는 점 · 팔 모델이 없으면 그대로 + INFO 한 줄
  void store_settle(ArmPath & path, const char * what) const
  {
    const double s = get_parameter("store_settle_s").as_double();
    if (!std::isfinite(s) || s < 1e-3) {return;}   // settle_at 과 같은 규칙
    const double up_mm = get_parameter("store_settle_up_mm").as_double();
    const int k = model_ok_ && std::isfinite(up_mm) ? store_settle_index(model_, path, up_mm / 1000.0) : -1;
    if (k < 0) {
      RCLCPP_INFO(get_logger(), "%s: 칸 위 멈춤 (store_settle_s) 없이 그대로 — %s", what,
        !model_ok_ ? "팔 모델 없음" : "놓는 점 (release) 이 없거나 그보다 store_settle_up_mm 이상 높은 점이 없음");
      return;
    }
    int rel = k + 1;   // 첫 놓는 점 (로그 "몇 점 앞")
    while (rel < static_cast<int>(path.nodes.size()) && path.nodes[rel].grip != "release") {++rel;}
    settle_at(path.nodes[k], "store_settle_s");
    RCLCPP_INFO(get_logger(), "%s: 칸 위 (놓는 점 위 %.0f mm · %d 점 앞) 에서 %.1f s 멈춤 (store_settle_s)", what, up_mm, rel - k, s);
  }

  // STORE 놓은 뒤 닫기 늦추기 (store_close_rise_mm · store_close_side_mm, cell_shift.hpp store_delay_close). 둘 다 0 = 그대로
  //   닫는 점만 옮김 (경로 · 시간표 그대로, 가며 닫기 grip_on_way 는 뒤에서 그 점에 붙음). 팔 모델 · 닫기가 없으면 그대로 + 로그
  void store_close_later(ArmPath & path, const char * what) const
  {
    const double rise = get_parameter("store_close_rise_mm").as_double(), side = get_parameter("store_close_side_mm").as_double();
    if (!(rise > 0.0) && !(side > 0.0)) {return;}
    if (!model_ok_) {RCLCPP_WARN(get_logger(), "%s: 팔 모델 없음 → 놓은 뒤 닫기 그대로 (store_close_rise_mm)", what); return;}
    StoreCloseMove mv;
    const bool moved = store_delay_close(model_, path, rise / 1000.0, side / 1000.0, mv);
    if (mv.from < 0) {
      RCLCPP_INFO(get_logger(), "%s: 놓은 뒤 닫기 없음 → 닫기 늦추기 안 함", what);
    } else if (!moved) {
      RCLCPP_INFO(get_logger(), "%s: 놓은 뒤 닫기 그대로 (%s, 놓는 점 대비 손끝 높이 %+.0f · 수평 %.0f mm)", what, path.nodes[mv.from].name.c_str(),
        mv.from_dz_mm, mv.from_dxy_mm);
    } else {
      RCLCPP_INFO(get_logger(), "%s: 놓은 뒤 닫기를 %d 점 늦춤 %s (높이 %+.0f · 수평 %.0f mm) → %s (높이 %+.0f · 수평 %.0f mm)%s "
        "(store_close_rise_mm %.0f · store_close_side_mm %.0f)", what, mv.to - mv.from, path.nodes[mv.from].name.c_str(), mv.from_dz_mm,
        mv.from_dxy_mm, path.nodes[mv.to].name.c_str(), mv.to_dz_mm, mv.to_dxy_mm, mv.capped ? " — 조건 맞는 점이 없어 접기 앞 점" : "", rise, side);
    }
  }

  // 손목 인지 결과 → odom 기준 퍽 목록 (촬영 시각 TF, 안 되면 최신 TF). 쓸 때 base_link 로 (그 사이 차체가 움직여도 맞게)
  // ---- 적재함 태그 → 칸 경로 옮기기 ----
  // 스캔 (LOOK rack) 중에만: 멈춘 장 (confidence 1) 칸 태그 중심 [base_link]
  void on_rack_tags(const interfaces::msg::DetectionArray & m)
  {
    gg_seat_feed_rack(m);   // grasp_guard: 넣은 퍽 (칸 퍽 태그)
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
    if (rack_fit_ok_ && rack_fit_persist_) {save_rack_fit();}
  }

  // 스캔 맞춤 기억 (rack_fit_file): 다시 켜도 같은 맞춤으로 칸 경로를 옮김
  void save_rack_fit()
  {
    std::error_code ec;
    std::filesystem::create_directories(std::filesystem::path(rack_fit_file_).parent_path(), ec);
    std::ofstream f(rack_fit_file_);
    char when[32] = "";
    const std::time_t t = std::time(nullptr);
    std::strftime(when, sizeof(when), "%m-%d %H:%M:%S", std::localtime(&t));
    f << "# 마지막 빈손 적재함 스캔 맞춤 (control_master, 기준 " << rack_ref_file_ << " 대비), 저장 " << when << "\n";
    f << "# dx dy [m] · 회전 [rad] · 남는 오차 [m] · 칸 수\n";
    f.precision(6);
    f << std::fixed << rack_fit_.dx << " " << rack_fit_.dy << " " << rack_fit_.dyaw << " " << rack_fit_.rms << " " << rack_fit_.n << "\n";
    f.close();
    if (!f) {RCLCPP_WARN(get_logger(), "적재함 스캔 맞춤을 못 씀: %s (이번 실행에서만 씀)", rack_fit_file_.c_str()); return;}
    RCLCPP_INFO(get_logger(), "적재함 스캔 맞춤 기억 → %s (다시 켜도 씀, 새 scan 이 덮어씀)", rack_fit_file_.c_str());
  }

  void load_rack_fit()
  {
    if (rack_ref_.empty()) {return;}   // 기준이 없으면 맞춤도 뜻 없음
    std::ifstream f(rack_fit_file_);
    if (!f) {
      RCLCPP_INFO(get_logger(), "기억한 적재함 스캔 맞춤 없음 (%s) → scan 하기 전까지 칸 경로 그대로", rack_fit_file_.c_str());
      return;
    }
    std::string line, when;
    while (std::getline(f, line)) {
      if (line.empty()) {continue;}
      if (line[0] == '#') {
        const auto k = line.find("저장 ");
        if (k != std::string::npos) {when = line.substr(k + std::string("저장 ").size());}
        continue;
      }
      std::istringstream in(line);
      RackFit r;
      if (in >> r.dx >> r.dy >> r.dyaw >> r.rms >> r.n && std::isfinite(r.dx) && std::isfinite(r.dy) && std::isfinite(r.dyaw) &&
        std::isfinite(r.rms))
      {
        r.ok = r.rotation = true;
        rack_fit_ = r;
        rack_fit_ok_ = r.rms <= rack_max_rms_m_;
        RCLCPP_INFO(get_logger(), "기억한 적재함 스캔 맞춤 (%s, 칸 %d 개): dx %+.1f · dy %+.1f mm · 회전 %+.2f° · 남는 오차 %.1f mm → %s",
          when.c_str(), r.n, r.dx * 1000.0, r.dy * 1000.0, r.dyaw * 180.0 / M_PI, r.rms * 1000.0,
          rack_fit_ok_ ? "STORE · RETRIEVE 칸 경로를 옮김 (새 scan 이 바꿈)" : "남는 오차가 커서 안 씀 (다시 scan)");
      }
      break;
    }
  }

  // 마지막 스캔을 기준으로 저장 (적재함이 제자리 = 지금 경로로 넣기 · 빼기가 되는 때)
  void on_held_tag(const interfaces::msg::DetectionArray & m)
  {
    gg_seat_feed_held(m);   // grasp_guard: 놓은 뒤 퍽 윗면 (넣은 뒤 확인)
    const double t = rclcpp::Time(m.header.stamp, get_clock()->get_clock_type()).seconds();
    if (!carrying_ || t < held_since_) {return;}
    for (const auto & d : m.detections) {
      if (!gg_held_accept(d.id, d.header.frame_id.empty() ? m.header.frame_id : d.header.frame_id, t)) {continue;}   // 지난 PICK 목표 퍽 ID · 놓기 전만 (11-10 밤: 쥔 퍽으로 잰 7 번이 다 이웃 퍽)
      const auto & p = d.pose.position;
      const auto & o = d.pose.orientation;
      const Eigen::Vector3d n = Eigen::Quaterniond(o.w, o.x, o.y, o.z).normalized() * Eigen::Vector3d::UnitZ();
      held_buf_.push_back({d.id, Eigen::Vector3d(p.x, p.y, p.z), n});
    }
    while (held_buf_.size() > 100) {held_buf_.pop_front();}
  }

  void on_held_fresh(const interfaces::msg::DetectionArray & m)
  {
    if (!get_parameter("held_correct_fresh").as_bool() || !carrying_ || m.header.frame_id!="wrist_link") {return;}
    const double stamp=rclcpp::Time(m.header.stamp,get_clock()->get_clock_type()).seconds();
    if (stamp<held_since_ || stamp>now().seconds()) {return;}
    for (const auto & d:m.detections) {
      if (d.kind!="held_tag_fresh" || d.confidence<0.99 || d.header.frame_id!=m.header.frame_id || d.header.stamp!=m.header.stamp) {continue;}
      const auto & p=d.pose.position;const auto & o=d.pose.orientation;
      const Eigen::Quaterniond q(o.w,o.x,o.y,o.z);
      if (!q.coeffs().allFinite() || q.norm()<0.5) {continue;}
      held_fresh_buf_.push_back({d.id,m.header.frame_id,stamp,{p.x,p.y,p.z},q.normalized()*Eigen::Vector3d::UnitZ()});
    }
    while (held_fresh_buf_.size()>30) {held_fresh_buf_.erase(held_fresh_buf_.begin());}
  }

  // 기존 경로 보정/놓은 뒤 들기가 끝난 사본에만 손 안 잔차를 더함. 못 재거나 못 풀면 그 사본 그대로임.
  void apply_held_fresh(const std::string & name, ArmPath & path)
  {
    if (!get_parameter("held_correct_fresh").as_bool() || !carrying_ || name.rfind("store_",0)!=0) {return;}
    std::string why="팔 모델/기준 없음";
    if (!model_ok_ || !held_ref_p_ || !held_ref_n_) {
      RCLCPP_INFO(get_logger(),"%s: 신선한 손 안 보정 없음 (%s) → 기존 경로",name.c_str(),why.c_str());return;
    }
    const auto current=fresh_held(held_fresh_buf_,held_fresh_id_,held_since_,now().seconds(),-model_.app5);
    const auto release=std::find_if(path.nodes.begin(),path.nodes.end(),[](const auto & n){return n.grip=="release";});
    if (release==path.nodes.end() || release->q.size()!=5) {return;}
    std::array<double,5> command{};std::copy_n(release->q.begin(),5,command.begin());
    const auto actual=sag_corrected(model_,command,0.0,1.0);   // 카메라 TF와 같은 실제 손목 방향
    Ticks ticks{};
    for(int j=0;j<5;++j) {ticks[j]=static_cast<int>(std::lround(model_.zero_tick[j]+actual[j]*4096.0/(2.0*M_PI)));}
    const Eigen::Matrix3d R=arm_frames(model_,ticks_to_q(model_,ticks))[4].block<3,3>(0,0);
    const auto correction=held_correction(current,{*held_ref_p_,*held_ref_n_,held_ref_depth_mm_},held_depth_mm_,
      -model_.app5,R,get_parameter("held_max_mm").as_double());
    if (!correction.ok) {
      RCLCPP_INFO(get_logger(),"%s: 신선한 손 안 보정 없음 (%s) → 기존 경로",name.c_str(),correction.why.c_str());return;
    }
    const auto cell=path_cell(name);const auto k=std::find(rack_names_.begin(),rack_names_.end(),cell);
    if(k==rack_names_.end() || correction.xy.norm()<1e-6) {return;}
    ArmPath trial=path;CellShiftReport report;
    if(!shift_cell_path(model_,trial,true,rack_xy_[k-rack_names_.begin()],correction.xy,report,why)) {
      RCLCPP_INFO(get_logger(),"%s: 신선한 손 안 보정 경로 못 풂 (%s) → 기존 경로",name.c_str(),why.c_str());return;
    }
    path=std::move(trial);
    RCLCPP_INFO(get_logger(),"%s: 신선한 퍽 %s %d장 · 깊이 기준%.0f/현재%.0f mm 제거 후 XY (%+.1f,%+.1f) mm 보정",
      name.c_str(),current.id.c_str(),current.count,held_ref_depth_mm_,held_depth_mm_,correction.xy.x()*1000,correction.xy.y()*1000);
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
    const bool fresh=get_parameter("held_correct_fresh").as_bool();
    bool got=false;
    if (fresh) {
      const auto value=fresh_held(held_fresh_buf_,held_fresh_id_,held_since_,now().seconds(),-model_.app5);
      got=carrying_ && model_ok_ && value.ok && std::isfinite(held_depth_mm_) && held_depth_mm_>0.0 && held_depth_mm_<=60.0;
      id=value.id;p=value.p;n=value.n;count=value.count;
      if (!got) {msg="새 기준 저장 안 함: 현재 쥔 깊이/신선한 태그 없음 (기존 기준 유지)";return false;}
    } else {got=held_now(id,p,n,count);}
    if (!got) {
      msg = "쥔 퍽 태그를 3 장 넘게 못 봄 (PICK 뒤 기준점에서, rack_tag_node 켜짐?) — 지금 " + std::to_string(count) + " 장";
      return false;
    }
    std::error_code ec;
    std::filesystem::create_directories(std::filesystem::path(held_ref_file_).parent_path(), ec);
    std::ofstream f(held_ref_file_);
    f << "# 쥔 퍽 기준 (control_master ~/save_held_reference): 태그 중심 x y z · 면 방향 x y z [m, wrist_link]\n";
    f.precision(6);
    f << std::fixed << p.x() << " " << p.y() << " " << p.z() << " " << n.x() << " " << n.y() << " " << n.z();
    if (fresh) {f << " " << held_depth_mm_;}   // 일곱째 값은 nominal 쥔 깊이 mm; 옛 6값 파일은 새 보정에 안 씀
    f << "\n";
    f.close();
    if (!f) {msg = "기준 파일을 못 씀: " + held_ref_file_; return false;}
    held_ref_p_ = p;
    held_ref_n_ = n;
    held_ref_depth_mm_ = fresh ? held_depth_mm_ : 0.0;
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
        double depth=0.0;held_ref_depth_mm_=(in>>depth) && std::isfinite(depth) && depth>0.0 && depth<=60.0 ? depth : 0.0;
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
    std::filesystem::remove(rack_fit_file_, ec);   // 기억한 맞춤은 옛 기준 대비라 지움
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
    const std::string cell = path_cell(name);   // store_S7 · retrieve_S7 · store_look_S7 · store_S7_d30 · retrieve_S7_full → S7 (cell_shift.hpp)
    // 새 칸 경로 (storeheld): 칸 보정 (store_to_cell · store_offset · 세운 퍽 높이 · retrieve_offset · 쥔 퍽 태그) 이 이미 들어 있음 → 적재함 움직임 (d_rack) 만
    const bool held = storeheld_path(name);
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
    if (held) {
      // 새 칸 경로: 칸 보정 건너뜀 (위 held). 대신 칸마다 storeheld_offset.<칸> [x, y] 또는 [x, y, 높이] mm 만 더함
      //   (11-10 15:2x 실물: 세운 퍽 store_S7_d30 이 뒤로 3 mm — 새 경로에도 칸별 미세 보정이 필요)
      if (known) {
        const auto c = get_parameter("storeheld_offset." + cell).as_double_array();
        if (c.size() != 2 && c.size() != 3) {why = "storeheld_offset." + cell + " 는 [x, y] 또는 [x, y, 높이] mm"; return false;}
        if (std::isfinite(c[0]) && std::isfinite(c[1])) {d_set = Eigen::Vector2d(c[0], c[1]) / 1000.0;}
        if (c.size() == 3 && std::isfinite(c[2])) {dz = c[2] / 1000.0;}
      }
    } else if (store) {
      double ox = get_parameter("store_offset_x_mm").as_double(), oy = get_parameter("store_offset_y_mm").as_double();
      if (known) {
        const auto c = get_parameter("store_offset." + cell).as_double_array();
        if (c.size() != 2 && c.size() != 3) {why = "store_offset." + cell + " 는 [x, y] 또는 [x, y, 높이] mm"; return false;}
        ox += c[0];
        oy += c[1];
        if (c.size() == 3 && std::isfinite(c[2])) {dz = c[2] / 1000.0;}
      }
      // 쥔 깊이가 박스 PICK (pick_heights_mm 마지막, 칸 보정을 맞춘 깊이) 과 다르면 칸 근처 높이를 그 차만큼 → 퍽 바닥이 같은 높이로
      //   (실물: 받침에서 세워 30 mm 로 다시 쥔 퍽이 박스 PICK 50 mm 보다 20 mm 아래 매달려 S2 에 긁혀 안 들어감)
      if (get_parameter("store_match_grip_depth").as_bool() && depth_from_pick_ && held_depth_mm_ > 0.0 && !pick_heights_.empty()) {
        const double ref = -pick_heights_.back();
        const double ddz = (ref - held_depth_mm_) / 1000.0;
        if (std::fabs(ddz) > 1e-4) {
          const double ex = get_parameter("store_match_grip_extra_mm").as_double();
          const double ex_m = std::isfinite(ex) ? ex / 1000.0 : 0.0;
          RCLCPP_INFO(get_logger(), "%s: 쥔 깊이 %.0f mm (박스 PICK %.0f mm) → 칸 근처 높이 %+.1f mm (store_match_grip_depth%s)", name.c_str(),
            held_depth_mm_, ref, (ddz + ex_m) * 1000.0, ex_m != 0.0 ? (" + store_match_grip_extra_mm " + std::to_string(std::lround(ex))).c_str() : "");
          dz += ddz + ex_m;
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
      if (!get_parameter("held_correct_fresh").as_bool()) {
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
    if (d.norm() < 1e-6 && std::fabs(dz) < 1e-6) {
      if (known) {lift_clear(path, rack_xy_[i], name, store);}
      return true;
    }
    if (!known) {why = "칸 " + cell + " 이 칸 표 (rack_cell_names) 에 없어 칸 경로를 못 옮김"; return false;}
    if (!model_ok_) {why = "팔 모델 없음 → 칸 경로를 못 옮김"; return false;}
    CellShiftReport rep;
    std::string err;
    // 다 옮기면 관절 범위 밖이면 (mock: S11 넣는 점 맞추기 10.7 mm → J4 범위 밖) 줄여 되는 만큼만, 그래도 안 되면 기록 그대로
    //   (기록 경로는 실물에서 들어간 경로 — 거절하지 않음)
    //   수평 · 높이를 따로 줄임 — 수평은 다 두고 높이부터 100 → 5 % (5 % 씩) · 0, 그래도 안 되면 높이 0 에 수평 0.75 · 0.5 · 0.25
    //   (실물: 세운 퍽 +20 mm 가 수평까지 같이 줄여 S4 · S5 · S6 · S8 · S10 · S11 칸 보정이 통째로 빠짐)
    double used = 0.0, used_z = 0.0;
    std::vector<std::pair<double, double>> tries;
    const bool has_z = std::fabs(dz) > 1e-6, has_xy = d.norm() > 1e-6;
    if (has_z) {   // 높이는 5 % 씩 (25 % 씩이면 세운 퍽 높이가 S8 · S11 등에서 0 으로 떨어짐)
      for (int q = 20; q >= 1; --q) {tries.emplace_back(1.0, q / 20.0);}
    }
    if (has_xy) {
      for (double sc : {1.0, 0.75, 0.5, 0.25}) {tries.emplace_back(sc, 0.0);}   // 높이 0 (has_z 가 없으면 첫 1.0 이 그대로 다 옮김)
    }
    for (const auto & [sc, sz] : tries) {
      ArmPath p2 = path;
      std::string e2;
      if (shift_cell_path(model_, p2, store, rack_xy_[i], d * sc, rep, e2, dz * sz)) {path = p2; used = sc; used_z = sz; break;}
      if (err.empty()) {err = e2;}
    }
    if (used == 0.0) {
      RCLCPP_WARN(get_logger(), "%s: 칸 근처를 (%+.1f, %+.1f) mm · 높이 %+.1f mm 옮기면 관절 범위 밖 (%s) → 줄여도 안 됨, 기록 경로 그대로",
        name.c_str(), d.x() * 1000.0, d.y() * 1000.0, dz * 1000.0, err.c_str());
      lift_clear(path, rack_xy_[i], name, store);
      return true;
    }
    const Eigen::Vector2d du = d * used;
    std::string cut;
    if (used < 1.0 || (used_z < 1.0 && has_z)) {
      cut = " (다 옮기면 관절 범위 밖 → 수평 " + std::to_string(static_cast<int>(used * 100)) + " % · 높이 " +
        std::to_string(static_cast<int>(used_z * 100)) + " %)";
    }
    RCLCPP_INFO(get_logger(), "%s: 칸 근처를 (%+.1f, %+.1f) mm · 높이 %+.1f mm 옮김%s = 적재함 움직임 (%+.1f, %+.1f) + 칸 보정 (%+.1f, %+.1f) "
      "(점 %d 개 · 관절 최대 %.2f° · 손끝 어긋남 %.2f mm)", name.c_str(), du.x() * 1000.0, du.y() * 1000.0, dz * used_z * 1000.0, cut.c_str(),
      d_rack.x() * 1000.0, d_rack.y() * 1000.0, d_set.x() * 1000.0, d_set.y() * 1000.0, rep.moved, rep.max_joint_deg, rep.worst_pos_mm);
    lift_clear(path, rack_xy_[i], name, store);
    return true;
  }

  // RETRIEVE: 쥔 뒤 칸 안에서는 곧게만 올라옴 (retrieve_lift_clear_mm, 0 = 끔 · 퍽 바닥이 칸 테두리 28 mm 위로 나오게 32)
  // STORE: 놓은 뒤 칸 안에서 곧게만 올라옴 (store_lift_clear_mm, 0 = 끔). 못 올리면 거절하지 않고 기록 경로 그대로
  void lift_clear(ArmPath & path, const Eigen::Vector2d & cell_xy, const std::string & name, bool store) const
  {
    const double c = get_parameter(store ? "store_lift_clear_mm" : "retrieve_lift_clear_mm").as_double();
    if (!std::isfinite(c) || c <= 0.0) {return;}
    ArmPath p2 = path;
    int raised = 0;
    std::string why;
    if (store) {
      if (!model_ok_) {RCLCPP_WARN(get_logger(), "%s: 팔 모델 없음 → 놓은 뒤 곧게 올리기 없이 기록 경로 그대로", name.c_str()); return;}
      if (!store_lift_clear(model_, p2, cell_xy, c / 1000.0, raised, why)) {
        RCLCPP_WARN(get_logger(), "%s: 놓은 뒤 곧게 올리기 안 됨 (%s) → 기록 경로 그대로", name.c_str(), why.c_str());
        return;
      }
      path = p2;
      RCLCPP_INFO(get_logger(), "%s: 놓은 뒤 칸 안에서 곧게 %.0f mm 올림 (store_lift_clear_mm) · 칸 근처 낮은 점 %d 개 올림", name.c_str(), c, raised);
      return;
    }
    if (!retrieve_lift_clear(model_, p2, cell_xy, c / 1000.0, raised, why)) {
      RCLCPP_WARN(get_logger(), "%s: 쥔 뒤 곧게 올리기 안 됨 (%s) → 기록 경로 그대로", name.c_str(), why.c_str());
      return;
    }
    path = p2;
    RCLCPP_INFO(get_logger(), "%s: 쥔 뒤 칸 안에서 곧게 %.0f mm 올림 (lift_clear) · 칸 근처 낮은 점 %d 개 올림 (retrieve_lift_clear_mm)", name.c_str(), c, raised);
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
      const double capture_t = rclcpp::Time(in.header.stamp).seconds();   // 촬영 시각 (아래 TF 실패 때 0 으로 바뀌기 전)
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
        q_known && std::fabs(zz) < std::sqrt(0.5), axis, p_nom, d.id, !latest && capture_t > 0.0, capture_t});
    }
    seen_ = out;
    seen_t_ = t;   // 빈 배열도 "카메라는 살아 있고 아무것도 없음"
  }

  // 목표 (x, y, 퍽 중심 높이 zc, base_link) 주변 이웃 퍽. 목표 자신 = 25 mm 안 (선 퍽 두 개는 30 mm 이상 떨어짐)
  //   같은 면 위 퍽만 (높이 ±40 mm). 목표로 보이는 검출이 있으면 그 모양 · 면 방향 · 누움 · 긴축 방향 (lying_axis, base_link)
  //   want_id_ (goal.puck_id) 가 있으면: 같은 높이 검출에 그 ID 가 있으면 그 검출만 목표 (목표점에서 id_max_offset 안, 넘으면 id_why).
  //     그 ID 가 안 보이면 위치로 맞추되, 25 mm 안 검출이 다른 ID 로 읽혔으면 목표로 안 잡고 id_why (옆 퍽을 집지 않게). ID 못 읽은 검출은 위치로
  std::vector<NeighborPuck> neighbors_near(double x, double y, double zc, bool & found, TargetPuck & tgt, Eigen::Vector2d & tc,
    bool & stale, bool & tagged, bool & lying, double & lying_axis, std::string & id_why) const
  {
    std::vector<NeighborPuck> nb;
    found = false;
    id_why.clear();
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
    auto skip = [&](const Eigen::Vector3d & b, const Eigen::Vector3d & bn) {
        if (exclude_ && std::hypot(b.x() - exclude_->x(), b.y() - exclude_->y()) < 0.025) {return true;}   // 다시 잡기: 집어 든 퍽
        return std::fabs(b.z() - zc) > 0.04 && std::fabs(bn.z() - zc) > 0.04;   // 다른 높이 (쥔 퍽 · 박스 밖 바닥 등)
      };
    // 목표 ID 가 이번 장에 보이나 (보이면 그 검출만 목표 후보)
    bool id_seen = false;
    if (!want_id_.empty()) {
      for (const auto & p : seen_) {
        const Eigen::Vector3d b = Q * p.p + T, bn = Q * p.p_nom + T;
        if (p.id == want_id_ && !skip(b, bn)) {id_seen = true; break;}
      }
    }
    std::string other_id;   // 위치로는 맞지만 다른 ID 로 읽힌 검출 (ID 가 안 보일 때)
    for (const auto & p : seen_) {
      const Eigen::Vector3d b = Q * p.p + T, bn = Q * p.p_nom + T;
      if (skip(b, bn)) {continue;}
      const double d = std::hypot(b.x() - x, b.y() - y);
      const double dn = std::hypot(bn.x() - x, bn.y() - y);
      const bool is_id = !want_id_.empty() && p.id == want_id_;
      const bool wrong_id = !want_id_.empty() && !p.id.empty() && p.id != want_id_;
      NeighborPuck n;
      n.c = b.head<2>();
      n.square = p.square;
      n.yaw_known = p.yaw_known && !p.round_r30;
      n.yaw = p.yaw + dyaw;
      n.radius = p.round_r30 ? 0.03 : 0.0;   // 누운 · 방향 모름 = 긴축 반 (30 mm) 원
      // 목표 = 되돌린 위치 또는 되돌리기 전 위치가 25 mm 안 (pick det · planning 은 명목 TF 로 목표를 냄 — 카메라가 3° 돌면 둘이 ~18 mm 차이)
      const double score = std::min(dn, d + 0.001);
      // 목표 후보: ID 가 보이면 그 ID 검출만 (거리 id_max_offset 안), 안 보이면 25 mm 안 · 다른 ID 아님
      bool cand = id_seen ? is_id : (std::min(d, dn) < 0.025 && !wrong_id);
      if (id_seen && is_id && std::min(d, dn) > id_max_offset_m_) {
        id_why = ": 퍽 " + want_id_ + " 이 목표점에서 " + std::to_string(std::lround(std::min(d, dn) * 1000.0)) + " mm (한계 " +
          std::to_string(std::lround(id_max_offset_m_ * 1000.0)) + ") → 다시 LOOK";
        cand = false;
      }
      if (!id_seen && wrong_id && std::min(d, dn) < 0.025 && other_id.empty()) {other_id = p.id;}
      if (cand && score < best) {
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
    if (!found && id_why.empty() && !other_id.empty()) {
      id_why = ": 목표점 25 mm 안 퍽은 " + other_id + " (요청 " + want_id_ + ") → 옆 퍽을 집지 않음, 다시 LOOK";
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
    std::string id_why;
    const auto nb = neighbors_near(x, y, zc, found, tgt, tc, stale, target_tagged_, seen_lying, seen_axis, id_why);
    last_nb_ = nb;
    target_square_ = !found || tgt.square;   // 선 퍽 종류 (모양 모름 = 네모) — farpick_plan 의 표 side
    if (!place) {gg_shape_ = !found ? PuckShape::Unknown : (tgt.square ? PuckShape::Square : PuckShape::Cylinder);}   // grasp_guard 쥔 폭
    if (!place && !id_why.empty()) {why = std::string(what) + id_why; return false;}
    if (!place && found && !want_id_.empty()) {RCLCPP_INFO(get_logger(), "%s: 퍽 %s 검출로 맞춤", what, want_id_.c_str());}
    if (place && found) {why = std::string(what) + ": 놓을 자리에 이미 퍽이 보임"; return false;}
    if (found && !place && seen_lying != lying) {
      why = std::string(what) + (lying ? ": 목표는 누운 퍽인데 손목 검출은 선 퍽 → 다시 LOOK" :
        ": 목표는 선 퍽인데 손목 검출은 누운 퍽 → 검출 방향으로 다시 (pick det)");
      return false;
    }
    // 박스 안은 소프트웨어로 막지 않음 ("박스 안에는 다 풀어") — 모르는 것은 알리고 가장 그럴듯한 값으로 쥠. 못 닿는 자세만 IK 가 거절
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
    if (found && !place && tgt.square && !tgt.yaw_known) {   // 면 방향 모르는 네모: 대각선으로 쥘 수 있음 — 막지 않고 알림
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
      // 손목을 덜 돌리는 닫힘 (pick_prefer_tangent): 기본 방향 (접선, 네모는 가장 가까운 면) 을 먼저 — 11-10 11:05 실물: 혼자 있는 네모를
      //   IK 가 손목 +79° 로 골라 돌림 보정 (−8.8, −7.8) mm 이 들어가 옆으로 비켜 내려가 손가락 대각선이 퍽을 밀고 앞을 쥠. 안 되면 지금처럼 IK 가 고름
      if (!place && get_parameter("pick_prefer_tangent").as_bool()) {
        const bool sqk = found && tgt.square && tgt.yaw_known;
        const double def0 = default_close_rad(model_, x, y, sqk ? tgt.yaw * 180.0 / M_PI : std::nan(""));
        if (std::isfinite(def0)) {cands.push_back({def0 * 180.0 / M_PI, "open"});}
      }
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
        const Eigen::Vector3d cv = arm_grip(model_, ticks_to_q(model_, P.back().ticks)).close;   // 기울인 손의 실제 닫힘축 (pick_poses: 면에 맞춤 · 20° 넘으면 접선)
        const double c = std::atan2(cv.y(), cv.x());
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
  //   origin_first (store_recover): (px, py) 자체를 먼저 봄 — 방금 이 퍽을 쥐어 간 박스 자리 (지금 비어 있음이 확실, 손목 검출이 오래돼도)
  bool regrasp_nodes(double px, double py, double top, double tilt, double depth, std::vector<PathNode> & out, double & sx, double & sy,
    std::string & why, double tilt_dev = 0.5, bool origin_first = false)
  {
    const double zc = top - model_.team_base_z_floor + model_.base_z_floor - puck_h_ / 2.0;
    const Eigen::Vector2d j1 = model_.j1_xy, p0(px, py);
    const double r0 = (p0 - j1).norm();
    if (r0 < 1e-6) {why = "다시 잡기: 퍽이 J1 위"; return false;}
    const Eigen::Vector2d dir = (p0 - j1) / r0, side(-dir.y(), dir.x());
    exclude_ = p0;
    std::string last = "후보 없음";
    bool ok = false;
    // 자리 탐색 시간 예산 (regrasp_search_s, 0 이하 = 없음): 11-09 실물 19 곳 × poses_for 2 번 × 씨앗 63 개 = 10 s 동안 motion_mode 가 끊겨 미션 FAULT
    const double budget_s = get_parameter("regrasp_search_s").as_double();
    const auto t_search = std::chrono::steady_clock::now();
    int looked = 0;
    bool timed_out = false;
    // 앞 자리에서 푼 관절값 → 다음 자리 IK 씨앗 (재쥐기 · 놓기 따로). 첫 자리는 기본 씨앗 수십 개 그대로 (11-09 성공한 다시 쥐기와 같은 풀이)
    std::vector<Q5> seed_r, seed_p;
    // 예산 검사는 자리 시작 · poses_for 가 실패한 뒤마다 (성공해 끝난 자리는 예산을 넘겼어도 그대로 받음)
    auto over_budget = [&]() {
        if (budget_s <= 0.0 || std::chrono::duration<double>(std::chrono::steady_clock::now() - t_search).count() <= budget_s) {return false;}
        timed_out = true;   // 남은 자리는 안 봄 → 거절 (planning 이 이 퍽을 건너뜀)
        last = "다시 쥘 자리 찾기 시간 초과 (" + std::to_string(looked) + " 곳 봄)";
        return true;
      };
    auto try_spot = [&](double x, double y) {
        if (timed_out || over_budget()) {return false;}
        if (x < regrasp_min_x_) {
          if (last == "후보 없음") {last = "박스 앞쪽 끝 (regrasp_min_x)";}   // 다른 이유가 있으면 그것을 남김
          return false;
        }
        if (std::fabs(y) > regrasp_max_abs_y_) {   // 박스 옆 밖 (퍽이 박스 위에 안 놓임)
          if (last == "후보 없음") {last = "박스 옆 밖 (regrasp_max_abs_y)";}
          return false;
        }
        ++looked;
        std::vector<std::pair<double, std::string>> cands;
        double gx = x, gy = y;
        std::string w;
        if (!grasp_for(gx, gy, zc, std::nan(""), true, cands, w, "다시 잡기 자리")) {last = w; return false;}   // 빈 자리 · 손가락 여유
        const double c_deg = (std::atan2(y - j1.y(), x - j1.x()) + M_PI / 2.0) * 180.0 / M_PI;
        std::string kw;
        // 수직으로 다시 쥐기 먼저 (여기서 많이 걸러짐 — place_far_plan 과 같은 순서), 앞 자리 해를 씨앗으로
        //   다시 쥐는 점만 regrasp_offset_mm [앞, 왼쪽] 옮김 (11-10 14:3x 실물: 세워 놓은 퍽을 수직으로 다시 쥘 때 2 mm 앞을 쥠 = 놓은 퍽이 2 mm 뒤에 앉음)
        const auto ro = get_parameter("regrasp_offset_mm").as_double_array();
        const double rx = x + (ro.size() == 2 && std::isfinite(ro[0]) ? ro[0] / 1000.0 : 0.0);
        const double ry = y + (ro.size() == 2 && std::isfinite(ro[1]) ? ro[1] / 1000.0 : 0.0);
        const auto Pr = poses_for(rx, ry, top, c_deg, pick_heights_, {{c_deg, "open"}}, kw, w, 0.0, 0.5, seed_r);
        if (Pr.size() != 3 || Pr.back().tilt_deg > 0.5) {
          last = "수직으로 다시 쥐기: " + (Pr.size() == 3 ? std::string("수직 손이 안 닿음") : w);
          over_budget();   // 이 IK 로 예산을 넘겼으면 다음 자리는 안 봄
          return false;
        }
        seed_r = {ticks_to_q(model_, Pr[0].ticks)};
        // 다시 쥐기 전 내려놓기: 퍽 바닥을 regrasp_put_clear_mm 위에서 놓음 (11-10 10:4x 실물: 2 mm 면 퍽이 먼저 닿고 손이 더 내려가
        //   손가락 안쪽 대각선 플라스틱이 퍽을 밀어냄 — 쥔 깊이가 손 안에서 밀려 실제와 다름)
        const std::vector<double> hp{place_heights_[0], place_heights_[1], get_parameter("regrasp_put_clear_mm").as_double() - depth};
        const auto Pp = poses_for(x, y, top, c_deg, hp, {{c_deg, "release"}}, kw, w, tilt, tilt_dev, seed_p);   // 놓는 손 기울임 = 쥘 때 ± tilt_dev (보통 0.5 = 같은 10° 단계)
        if (Pp.size() != 3) {last = "놓기: " + w; over_budget(); return false;}
        seed_p = {ticks_to_q(model_, Pp[0].ticks)};
        out = {model_node("re_place_above", Pp[0].enc, slow_dps_), model_node("re_place_down", Pp[1].enc, slow_dps_),
          model_node("re_place", Pp[2].enc, slow_dps_, "release"), model_node("re_place_up", Pp[1].enc, slow_dps_),
          model_node("re_place_clear", Pp[0].enc, slow_dps_), model_node("re_above", Pr[0].enc, slow_dps_, "open"),
          model_node("re_down", Pr[1].enc, slow_dps_), model_node("re_grasp", Pr[2].enc, slow_dps_, "close"),
          model_node("re_up", Pr[1].enc, slow_dps_), model_node("re_lift", Pr[0].enc, slow_dps_)};
        settle_at(out[0], "place_settle_s");   // 내려놓을 자리 위에서 멈춤 (0 = 그대로)
        settle_at(out[5], "pick_settle_s");    // 다시 쥘 퍽 위에서 멈춤
        visual_obs_q_.clear();
        if (visual_enabled_) {   // 다시 보고 쥐기: 손 뺀 점 위 관찰 점 (멈춤) 을 끼우고, 수직 다시 쥐기에 쓴 값 (높이 · 닫힘 · 씨앗) 그대로 기억
          const double true_top = surface_z_ + puck_h_ + model_.team_base_z_floor - model_.base_z_floor;   // 박스 위 선 퍽 윗면 (모델 z)
          const double h_obs = (true_top - top) * 1000.0 + get_parameter("regrasp_visual_observe_mm").as_double();
          const double t_obs = get_parameter("regrasp_visual_observe_tilt_deg").as_double();
          std::string ow;
          auto Po = pick_poses(model_, x, y, top, c_deg, {h_obs}, ow, c_deg, max_tilt_deg_, t_obs, 90.0, {ticks_to_q(model_, Pp[0].ticks)});
          if (Po.size() != 1) {Po = pick_poses(model_, x, y, top, c_deg, {h_obs}, ow, c_deg, max_tilt_deg_, t_obs, 90.0);}
          if (Po.size() == 1) {
            PathNode ob = model_node(kRegraspObserveNode, Po[0].enc, 0.0);   // 빈손으로 곧게 올라감 → 보통 속도
            ob.stop = true;
            out.insert(out.begin() + 5, ob);   // re_place_clear 뒤, re_above 앞
            visual_expected_ = Eigen::Vector3d(x, y, zc);
            visual_blind_xy_ = Eigen::Vector2d(rx, ry);
            visual_seed_ = ticks_to_q(model_, Pr[0].ticks);
            visual_top_ = top;
            visual_close_deg_ = c_deg;
            visual_obs_q_ = ob.q;
          } else {
            RCLCPP_WARN(get_logger(), "다시 쥐기: 관찰 점 (윗면 위 %.0f mm) IK 안 됨 (%s) → 이 다시 쥐기는 보지 않고 (지금 경로)",
              h_obs - (true_top - top) * 1000.0, ow.c_str());
          }
        }
        sx = x;
        sy = y;
        return true;
      };
    if (origin_first) {ok = try_spot(px, py);}
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
    {   // 퍽 높이 이상: 중심 z 가 박스 윗면 + 반높이 (누운 퍽 = 반폭) 에서 pick_z_tol_mm 밖이면 거절 (11-09 B4 퍽 8 z 0.178, 다른 퍽 0.064–0.070)
      const double tol = get_parameter("pick_z_tol_mm").as_double();
      const double zc0 = top - model_.team_base_z_floor + model_.base_z_floor - (lying ? lying_h_ : puck_h_) / 2.0;   // base_link 퍽 중심
      const double expect = surface_z_ + (lying ? lying_h_ : puck_h_) / 2.0;
      if (std::isfinite(tol) && tol > 0.0 && std::fabs(zc0 - expect) * 1000.0 > tol) {
        char buf[120];
        std::snprintf(buf, sizeof(buf), "퍽 높이 이상 (z %.0f mm, 기대 %.0f)", zc0 * 1000.0, expect * 1000.0);
        why = buf;
        return {};
      }
    }
    // 누운 퍽 윗면: 검출 z 가 박스 윗면 기준 (surface_z_ + lying_h_/2) 에서 lying_z_fix_mm 넘게 어긋날 때만 박스 기준으로
    //   (11-09 실물: 빨강 누운 퍽 깊이 점 중심 z 30.5 mm ≠ 기대 54 → 손가락이 박스 윗면을 침). 그 안이면 검출 그대로 —
    //   받침 세우기 (park · standup) 는 검출 높이로 맞춘 값이라 늘 바꾸면 받침에 못 들어감 (11-09 21:2x, 45 → 54 mm)
    if (lying && goal.target.pose.position.z != 0.0) {
      const double zc_det = top - model_.team_base_z_floor + model_.base_z_floor - lying_h_ / 2.0;   // base_link 검출 중심
      const double zc_box = surface_z_ + lying_h_ / 2.0;
      const double fix_mm = get_parameter("lying_z_fix_mm").as_double();
      if (std::fabs(zc_det - zc_box) * 1000.0 > fix_mm) {
        top = surface_z_ + lying_h_ + model_.team_base_z_floor - model_.base_z_floor;
        RCLCPP_INFO(get_logger(), "누운 퍽 높이: 검출 z %.1f mm 가 박스 기준 %.1f 에서 %.0f mm 넘게 어긋남 → 박스 기준으로 쥠", zc_det * 1000.0, zc_box * 1000.0, fix_mm);
      } else {
        RCLCPP_INFO(get_logger(), "누운 퍽 높이: 검출 z %.1f mm 그대로 (박스 기준 %.1f, %.0f mm 안)", zc_det * 1000.0, zc_box * 1000.0, fix_mm);
      }
    }
    // 선 퍽 윗면 높이 보정 (pick_top_bias_mm). 실물: 명령 −50 이 실제로
    //   너무 깊게 쥠 → 윗면을 10 mm 높게 봐 실제 깊이 약 40 mm 로. 쥔 깊이 기록 (pending_depth_) 은 명령값 그대로라
    //   STORE 칸 높이 맞춤 (store_match_grip_depth: 박스 PICK 50 − 받침 다시 쥐기 30 = +20) 이 바뀌지 않음. 누운 퍽은 안 건드림
    double top_bias_mm = 0.0;
    if (!lying) {
      const double bias = get_parameter("pick_top_bias_mm").as_double();
      if (!std::isfinite(bias) || std::fabs(bias) > 30.0) {why = "pick_top_bias_mm 이 숫자가 아니거나 30 mm 넘음"; return {};}
      if (bias != 0.0) {
        top += bias / 1000.0;
        top_bias_mm = bias;
        RCLCPP_INFO(get_logger(), "PICK: 선 퍽 윗면을 %+.0f mm 높게 봄 (pick_top_bias_mm) → 쥐는 손끝도 그만큼 얕게", bias);
      }
    }
    std::vector<std::pair<double, std::string>> cands;
    std::string open_kw;
    const double zc = top - model_.team_base_z_floor + model_.base_z_floor - (lying ? lying_h_ : puck_h_) / 2.0;   // 퍽 중심 (base_link)
    want_id_ = goal.puck_id;   // 이 grasp_for 만 ID 로 맞춤 (다시 잡기 · PLACE 는 위치)
    const bool grasp_ok = grasp_for(x, y, zc, yaw, false, cands, why, "PICK", lying);
    want_id_.clear();
    if (!grasp_ok) {return {};}
    // 붙은 이웃 (면 사이 farpick_touch_mm 안): 보정값 중 이웃 방향 성분은 빼고 (11-10 10:09 실물: 붙은 빨강 둘 — 손목 돌림 보정 · 태그 보정이
    //   이웃 쪽으로 11 mm 밀어 두 퍽 가운데를 쥠), 같은 색이라 경계가 섞여 겹쳐 잰 만큼 (면 사이 < 0) 이웃 반대쪽으로 반 밀어냄
    Eigen::Vector2d nb_u = Eigen::Vector2d::Zero();
    double nb_gap_mm = std::numeric_limits<double>::infinity();
    for (const auto & n : last_nb_) {
      const double half_n = n.radius > 0.0 ? n.radius : (n.square ? lying_h_ : 0.030) / 2.0;
      const Eigen::Vector2d v = n.c - Eigen::Vector2d(x, y);
      const double g = (v.norm() - (target_square_ ? lying_h_ : 0.030) / 2.0 - half_n) * 1000.0;
      if (g < nb_gap_mm && v.norm() > 1e-6) {nb_gap_mm = g; nb_u = v.normalized();}
    }
    const bool nb_touch = !lying && nb_gap_mm <= get_parameter("farpick_touch_mm").as_double();
    auto drop_toward_nb = [&](Eigen::Vector2d & d_m, const char * what) {   // d_m [m]: 이웃 방향 성분 제거
        if (!nb_touch) {return;}
        const double c = d_m.dot(nb_u);
        if (std::fabs(c) < 1e-6) {return;}
        d_m -= c * nb_u;
        RCLCPP_INFO(get_logger(), "PICK: 붙은 이웃 (면 사이 %.1f mm) → %s 중 이웃 방향 %+.1f mm 뺌", nb_gap_mm, what, c * 1000.0);
      };
    if (nb_touch && nb_gap_mm < 0.0) {
      const double push = std::min(-nb_gap_mm / 2.0, 5.0) / 1000.0;
      x -= nb_u.x() * push;
      y -= nb_u.y() * push;
      RCLCPP_INFO(get_logger(), "PICK: 이웃과 %.1f mm 겹쳐 잼 (같은 색 경계) → 이웃 반대쪽으로 %.1f mm", -nb_gap_mm, push * 1000.0);
    }
    // 태그로 잰 중심 (perception center_from_tag): 선 퍽 = 윗면 태그, 누운 퍽 = 위를 향한 옆면 태그 — 보정값이 따로
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
      Eigen::Vector2d od(ox / 1000.0, oy / 1000.0);
      drop_toward_nb(od, tag_centre ? "태그 퍽 보정" : "쥐는 점 보정");
      if (tag_centre) {
        x += od.x();
        y += od.y();
        RCLCPP_INFO(get_logger(), "PICK: %s 태그로 잰 퍽 → 태그 퍽 보정 (%+.1f, %+.1f) mm (%s)", lying ? "누운 퍽 위 옆면" : "윗면", ox, oy,
          lying ? "pick_offset_lying_tagged_*" : "pick_offset_tagged_*");
      } else if (ox != 0.0 || oy != 0.0) {
        x += od.x();
        y += od.y();
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
      return park_or_relocate_plan(unfold->second, x, y, top, cands, open_kw, why);   // 가까운 자리면 park_plan 그대로, 멀면 옮겨 놓고 세움
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
    // 먼 · 옆 퍽 붙은 선 퍽 표 (farpick_plan, 11-10 강화학습): always = 표에 맞는 칸이 있으면 늘, auto = 이웃이 면 사이 farpick_touch_mm 안이거나
    //   아래 지금 계획이 거절될 때 먼저 시도 (안 되면 지금 흐름 그대로, 거절 이유 둘 다 why 에), off = 안 씀
    const std::string fp_mode = get_parameter("farpick_mode").as_string();
    std::string fp_why;
    bool fp_tried = false;
    auto try_farpick = [&](const std::string & reason) {
        fp_tried = true;
        auto plan = farpick_plan(x, y, top, yaw, target_square_, fp_why);
        if (plan.empty()) {RCLCPP_INFO(get_logger(), "PICK: %s — 먼 퍽 표 안 됨 (%s) → 지금 흐름", reason.c_str(), fp_why.c_str());}
        return plan;
      };
    if (fp_mode == "always") {
      auto plan = try_farpick("farpick_mode always");
      if (!plan.empty()) {return plan;}
    } else if (fp_mode == "auto") {
      double touch = std::numeric_limits<double>::infinity();   // 가장 가까운 이웃과 면 사이 [mm] = 중심 거리 − 반폭 둘
      for (const auto & n : last_nb_) {
        const double half_n = n.radius > 0.0 ? n.radius : (n.square ? lying_h_ : 0.030) / 2.0;
        touch = std::min(touch, ((n.c - Eigen::Vector2d(x, y)).norm() - (target_square_ ? lying_h_ : 0.030) / 2.0 - half_n) * 1000.0);
      }
      if (touch <= get_parameter("farpick_touch_mm").as_double()) {   // (b) 이웃이 붙음
        char buf[80];
        std::snprintf(buf, sizeof(buf), "이웃 퍽이 면 사이 %.1f mm", touch);
        auto plan = try_farpick(buf);
        if (!plan.empty()) {return plan;}
      }
    }
    // 위 · 내려가는 점은 실제 윗면 기준 그대로 (pick_top_bias_mm 은 쥐는 깊이만) — 11-10 11:2x 실물: 위 점이 20 mm 올라가 수직 손이
    //   안 닿자 손목을 78° 돌린 갈래로 풀림 (돌리면 손끝이 축에서 12 mm 떨어져 더 닿음)
    std::vector<double> ph = pick_heights_;
    for (size_t i = 0; i + 1 < ph.size(); ++i) {ph[i] -= top_bias_mm;}
    auto P = poses_for(x, y, top, yaw, ph, cands, open_kw, why, 0.0, kStoreMaxTilt);
    if (P.size() == 3 && P.back().tilt_deg < 0.5) {
      Eigen::Vector2d d;
      double rot = 0.0;
      if (!rot_offset(x, y, P.back().ticks, d, rot, why)) {return {};}
      if (std::fabs(rot) >= 20.0 || d.norm() > 1e-4) {   // 보정이 0 이어도 돌려 쥔 것은 알림 (pick_offset_rot_mm 을 맞출 때)
        RCLCPP_INFO(get_logger(), "PICK: 손목을 기본에서 %+.0f° 돌려 쥠 → 돌림 보정 (%+.1f, %+.1f) mm (pick_offset_rot_mm)", rot, d.x() * 1000.0,
          d.y() * 1000.0);
      }
      if (d.norm() > 1e-4) {
        const Eigen::Vector3d c0 = arm_grip(model_, ticks_to_q(model_, P.back().ticks)).close;
        const auto P0 = P;
        const std::string kw0 = open_kw;
        x += d.x();
        y += d.y();
        P = poses_for(x, y, top, yaw, ph, cands, open_kw, why, 0.0, kStoreMaxTilt);
        if (P.size() != 3 || P.back().tilt_deg > 0.5) {   // 옮긴 자리에서 수직 손이 안 되면 돌림 보정 (수직 · 돌린 손목용) 을 빼고 처음 자리로 (11-10 검수 B2)
          x -= d.x();
          y -= d.y();
          P = P0;
          open_kw = kw0;
          RCLCPP_WARN(get_logger(), "PICK: 돌림 보정 자리에서 수직 손이 안 됨 → 돌림 보정 없이 처음 자리");
        }
        // 옮긴 자리에서 손목 갈래가 바뀌면 (78° 돌린 보정을 하고 돌리지 않고 쥠 · 반대) 보정이 틀림 → 처음 갈래 (같은 닫힘 방향) 로 다시 풂
        //   11-10 11:2x 실물: 돌림 보정 (−8.7, −8.1) mm 로 옮긴 자리에서 −9° 갈래로 쥠 → 퍽을 12 mm 어긋나게 쥐어 S12 에 옆으로
        Eigen::Vector2d d2;
        double rot2 = 0.0;
        std::string w2;
        if (P.size() == 3 && P.back().tilt_deg < 0.5 && rot_offset(x, y, P.back().ticks, d2, rot2, w2) &&
          (std::fabs(rot2) >= 45.0) != (std::fabs(rot) >= 45.0))
        {
          std::string kw2 = kw0;
          const auto Pf = poses_for(x, y, top, yaw, ph, {{std::atan2(c0.y(), c0.x()) * 180.0 / M_PI, kw0}}, kw2, w2, 0.0, 0.5);
          if (Pf.size() == 3) {
            P = Pf;
            open_kw = kw2;
            RCLCPP_INFO(get_logger(), "PICK: 옮긴 자리에서도 손목 %+.0f° 갈래 그대로 (다른 갈래 %+.0f° 는 보정과 안 맞음)", rot, rot2);
          } else {
            x -= d.x();
            y -= d.y();
            P = P0;
            open_kw = kw0;
            RCLCPP_WARN(get_logger(), "PICK: 옮긴 자리에서 손목 %+.0f° 갈래가 안 됨 (%s) → 돌림 보정 없이 처음 자리", rot, w2.c_str());
          }
        }
      }
    }
    double depth = -ph.back();
    if (P.size() != 3 && max_tilt_deg_ >= kStoreMaxTilt + 10.0 - 1e-6) {
      auto h = ph;
      h.back() = -tilted_depth_mm_;
      std::string why2;
      P = poses_for(x, y, top, yaw, h, cands, open_kw, why2, kStoreMaxTilt + 10.0, max_tilt_deg_ - kStoreMaxTilt - 10.0);
      depth = tilted_depth_mm_;
      if (P.size() != 3) {why = why + " · 기울인 손 (" + why2 + ")";}
    }
    if (P.size() == 3 && P.back().tilt_deg > 0.5) {   // 기울인 손만 옆 보정 (수직 손 집기는 그대로)
      const auto o = get_parameter("pick_offset_tilt_mm").as_double_array();
      if (o.size() == 2 && std::isfinite(o[0]) && std::isfinite(o[1]) && (o[0] != 0.0 || o[1] != 0.0)) {
        const Eigen::Vector2d rd = (Eigen::Vector2d(x, y) - model_.j1_xy).normalized(), td(-rd.y(), rd.x());
        const Eigen::Vector2d d = (td * o[0] + rd * o[1]) / 1000.0;
        auto h = ph;
        h.back() = -depth;
        std::string kw2 = open_kw, w;
        const auto P2 = poses_for(x + d.x(), y + d.y(), top, yaw, h, cands, kw2, w, P.back().tilt_deg, 5.0);
        if (P2.size() == 3) {
          x += d.x();
          y += d.y();
          P = P2;
          open_kw = kw2;
          RCLCPP_INFO(get_logger(), "PICK: 손 %.0f° 기울여 쥠 → 기울인 손 보정 접선 %+.1f · 반지름 %+.1f mm (pick_offset_tilt_mm)", P.back().tilt_deg, o[0], o[1]);
        } else {
          RCLCPP_WARN(get_logger(), "PICK: 기울인 손 보정 (%+.1f, %+.1f) mm 자리는 손이 안 닿음 (%s) → 보정 없이", o[0], o[1], w.c_str());
        }
      }
    }
    // (c) 지금 계획 거절 → 먼 퍽 표. 이웃 없이 손만 기울이면 되는 퍽은 표 안 씀 — 기울여 쥐고 세워 다시 쥠
    //   (11-10 11:1x 실물: 표가 가까운 네모 퍽에 손목을 76° 확 돌림 → 사용자: 그냥 기울여 쥐고 가져와 정렬)
    if (fp_mode == "auto" && !fp_tried && P.size() != 3) {
      auto plan = try_farpick("지금 계획 거절 (" + why + ")");
      if (!plan.empty()) {return plan;}
    }
    // 기울인 손은 닫힘축이 접선뿐 → 네모 면이 접선과 farpick_face_tol_deg 넘게 틀어지면 내려가는 손가락이 모서리를 침
    //   (11-10 11:3x 실물: 18.8° 틀어진 네모 모서리에 부딪힘) → 먼 퍽 표 먼저 (표는 손목을 돌려 면에 맞춰 쥐고 세워 다시 쥠)
    if (fp_mode == "auto" && !fp_tried && P.size() == 3 && P.back().tilt_deg > 0.5 && target_square_ && std::isfinite(yaw)) {
      const double tang = std::atan2(y - model_.j1_xy.y(), x - model_.j1_xy.x()) * 180.0 / M_PI + 90.0;
      const double mis = std::fabs(std::remainder(yaw - tang, 90.0));
      if (mis > get_parameter("farpick_face_tol_deg").as_double()) {
        char buf[120];
        std::snprintf(buf, sizeof(buf), "손을 %.0f° 기울여야 하는데 네모 면이 닫힘축 (접선) 과 %.0f° 틀어짐", P.back().tilt_deg, mis);
        auto plan = try_farpick(buf);
        if (!plan.empty()) {return plan;}
      }
    }
    if (P.size() != 3) {if (fp_tried) {why += " · 먼 퍽 표 (" + fp_why + ")";} return {};}
    pending_tilt_ = P.back().tilt_deg;
    pending_depth_ = depth;
    pending_lying_ = false;
    std::vector<PathNode> again;
    if (regrasp_ && pending_tilt_ > 0.5) {
      double sx = 0.0, sy = 0.0;
      if (!regrasp_nodes(x, y, top, pending_tilt_, depth, again, sx, sy, why)) {
        // 기울여 쥘 수는 있는데 다시 쥘 자리가 없음 → 먼 퍽 표 (11-10 19:5x 실물: 모서리 (0.340, -0.094) 40° 는 자리 36 곳 시간 초과로 거절, 표 칸은 4 mm 옆)
        if (fp_mode == "auto" && !fp_tried) {
          auto plan = try_farpick("다시 쥘 자리 없음 (" + why + ")");
          if (!plan.empty()) {return plan;}
        }
        if (fp_tried) {why += " · 먼 퍽 표 (" + fp_why + ")";}
        return {};
      }
      RCLCPP_INFO(get_logger(), "PICK: 손을 %.0f° 기울여 쥐어야 함 → (%.3f, %.3f) 에 세워 내려놓고 수직 손으로 다시 쥠", pending_tilt_, sx, sy);
      pending_tilt_ = 0.0;
      pending_depth_ = -pick_heights_.back();
    }
    return pick_path(unfold->second, P, open_kw, again, why, x, y, yaw, false);
  }

  // 세우기: 쥐기 뒤 실측 확인 — 손끝이 쥐는 자리에 내려갔나 (실물: 손가락이 퍽 위에 얹혀 17 mm 높게 멈춘 채 세우기가 돌아감),
  //   그리퍼가 빈손으로 끝까지 닫히지 않았나. 아니면 false (bad)
  // empty: 빈손 (그리퍼가 끝까지 닫힘) 으로 걸렸으면 true (EMPTY_GRASP 결과 표시 · 쥔 상태 지우기)
  bool standup_grasp_ok(std::string & bad, bool * empty = nullptr) const
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
      if (empty) {*empty = true;}
      return false;
    }
    return true;
  }

  // 빈손 (grasp_empty): 쥐기 다음 동작의 시간표를 쥐기 높이 위 점 (up_q, 닫는 점 다음 점) 까지만 남김 → 팔은 그 자리에서 위로만. 그 점이 시간표에 없으면 false
  bool cut_at_up(ArmAction & a, const std::vector<double> & up_q) const
  {
    if (up_q.empty()) {return false;}
    for (std::size_t k = 0; k < a.points.size(); ++k) {
      const auto & q = a.points[k].q;
      bool same = q.size() == up_q.size();
      for (std::size_t j = 0; same && j < q.size(); ++j) {same = std::fabs(q[j] - up_q[j]) < 1e-4;}
      if (!same) {continue;}
      a.points.resize(k + 1);
      a.duration_s = a.points.back().t;
      a.side_grip_t = -1.0;   // 가는 길의 그리퍼 동작 (loose 등) 은 안 함
      return true;
    }
    return false;
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
    //   박스 윗면 (누운 퍽 아래면) + standup_tip_clear_mm 아래를 가리키는 점은 그 높이까지 올림 (실물: 표가 박스 윗면 아래 10 mm 를 가리켜
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
    // 경로 점이 IK 로 안 풀리면 그 점만 건너뜀 (막지 말 것). 여는 점이 빠지면 다음 점에서 엶
    //   점 사이 시간 = 표 녹화 간격 (0.1 s) × standup_time_scale — 속도 상한으로 다시 짜면 시뮬 재생 84 → 47 % (15 °/s · 여는 점 멈춤)
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
      n.grip_on_way = open_here;   // 팔을 멈추지 않고 열기 (여는 점에서 0.2 s 만 멈춰도 시뮬 재생 84 → 36 %)
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
    settle_at(path.nodes.back(), "pick_settle_s");   // 선 퍽 다시 쥐기 위에서 멈춤 (0 = 그대로)
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
    return visual_regrasp_actions(path, false);
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
    const double v_press = get_parameter("park_press_speed_dps").as_double();
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
    pk.press_r = get_parameter("park_press_r_mm").as_double() / 1000.0;
    pk.press_top = get_parameter("park_press_top_mm").as_double() / 1000.0;
    pk.press_h = get_parameter("park_press_h_mm").as_double() / 1000.0;
    pk.push_low = get_parameter("park_push_low_mm").as_double() / 1000.0;
    grip_.loose = get_parameter("park_loose_grip_m").as_double();
    pk.side = lying_h_;
    pk.length = puck_h_;
    std::vector<ParkNode> pn;
    double elev = 0.0;
    // 쥐기 후보마다: 이웃 퍽 ↔ 손가락 (위에서 본 발자국, 기울인 손은 손가락이 기운 만큼 넓게) · 그 쥐기로 받침 자세까지 되는지
    //   (실물: 첫 후보만 받침 자세를 풀어 park_over IK 안 됨 → 거절. 다른 끝 · 닫힘 부호 후보는 됨)
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
      // 미는 구간 (park_push_down · 1–4 · back) 만 park_push_speed_dps (시뮬: 50 °/s 면 기대는 각 49° 아래에서 넘어짐, 20 °/s 면 ±2–4 mm 오차에도 섬)
      //   꽂아 누르기 (park_press1–4) 는 park_press_speed_dps (0 = park_speed_dps, 실물 그대로)
      const double sp = !n.slow ? 0.0 : (n.name.rfind("park_push", 0) == 0 || n.name.rfind("park_pull", 0) == 0 ? v_push :
        (v_press > 0.0 && n.name.rfind("park_press", 0) == 0 && n.name != "park_press_lift" && n.name != "park_press_turn" &&
        n.name != "park_press_over" ? v_press : v));
      nodes.push_back(model_node(n.name, q_to_ticks(model_, encoder_goal_q(model_, n.q, n.held_kg)), sp, n.grip));
      if (n.grip == "loose") {nodes.back().grip_on_way = true;}   // 팔을 안 멈추고 손가락만 풂
    }
    pending_tilt_ = 0.0;
    pending_depth_ = pk.regrasp_depth * 1000.0;
    pending_lying_ = false;   // 세운 퍽을 쥐고 끝남
    pending_empty_ = false;
    // 세운 퍽을 박스 위에 한 번 내려놓고 박스 퍽처럼 pick_heights_ (50 mm) 로 다시 쥠 (park_box_regrip). 받침 다시 쥐기는 30 mm 라 칸 넣기 때
    //   +20 mm 가 필요한데 J4 범위로 3–10 mm 만 올라감 (11-09 계산 12 칸 · 실물 S4 35 % → 13 mm 낮게 넣어 걸림) → 칸 · PLACE 값을 박스 퍽 그대로.
    //   자리는 선 퍽 다시 쥐기 (regrasp_nodes) 와 같은 탐색 · 같은 놓기 · 쥐기 (손 기울임 0). 자리가 없으면 받침에서 쥔 채 (예전 그대로)
    if (get_parameter("park_box_regrip").as_bool()) {
      std::vector<PathNode> again;
      double sx = 0.0, sy = 0.0;
      std::string w;
      // 박스 위 선 퍽 윗면 (팀 기준) + pick_top_bias_mm: 박스 퍽 PICK 과 같은 실제 깊이로 쥠
      //   (11-11 실물: 보정 없이 50 mm 면 박스 PICK (실제 40 mm) 보다 10 mm 깊어 손가락 안쪽 대각선이 퍽을 긁어 밂)
      const double tb = get_parameter("pick_top_bias_mm").as_double();
      const double top_box = surface_z_ + puck_h_ + model_.team_base_z_floor - model_.base_z_floor +
        (std::isfinite(tb) && std::fabs(tb) <= 30.0 ? tb / 1000.0 : 0.0);
      if (regrasp_nodes(x, y, top_box, 0.0, pending_depth_, again, sx, sy, w)) {
        nodes.insert(nodes.end(), again.begin(), again.end());
        pending_depth_ = -pick_heights_.back();
        RCLCPP_INFO(get_logger(), "PICK 받침 세우기: 세운 퍽을 (%.3f, %.3f) 박스 위에 내려놓고 %.0f mm 로 다시 쥠 (park_box_regrip)", sx, sy, pending_depth_);
      } else {
        RCLCPP_WARN(get_logger(), "PICK 받침 세우기: 세운 퍽을 내려놓을 박스 자리 없음 (%s) → 받침에서 %.0f mm 로 쥔 채", w.c_str(), pending_depth_);
      }
    }
    RCLCPP_INFO(get_logger(), "PICK 받침 세우기: 누운 퍽 (%.3f, %.3f) 긴축 %.0f° → 끝 쪽 %.0f mm 를 손 %.0f° (퍽 굴림 %.0f° · 손 안 기울기 %.0f° · 쥐는 깊이 %.0f mm · "
      "손가락 여유 %.1f mm) 로 쥐고 받침에 %.0f° 로 넣어 놓고, %s %.0f mm 로 %s 세운 뒤 다시 쥠 (윗면 − %.0f mm) → 기준점", x, y, axis,
      gc.grip_offset * 1000.0, lg.tilt_deg, lg.roll_deg, lg.pitch_deg, lg.depth_mm, lg.margin_mm, elev,
      pk.press_mode ? "받침 뒤에 닫은 손을 수직으로 바닥 위" : "받침 바닥 위", (pk.press_mode ? pk.press_h : pk.push_h) * 1000.0,
      pk.press_mode ? "꽂아" : "밀어", pk.regrasp_depth * 1000.0);
    pending_step_ = get_parameter("park_step").as_bool();
    return pick_path(unfold, lg.poses, open_kw, nodes, why, lg.tip.x(), lg.tip.y(), axis, true);
  }

  // 먼 누운 네모 퍽 가운데 쥐기 표 (config/lyingpick/, 11-10 밤 강화학습 lyingpick_res: park_or_relocate_plan ① 쥐기 + 보정 6 개) — ① 쥐기 · 들기 대신.
  //   farpick_plan 과 같은 방식: 가장 가까운 칸 (긴축 180° 주기, lyingpick_max_dist_mm · lyingpick_max_gap_deg 안) → standup_retarget 로 실제 퍽으로
  //   (손끝 최저 = 박스 윗면 + 3 mm) → 처짐 앞먹임 (닫는 점부터 쥔 무게) → LOOK (보기 변형) 자세에서 표 그대로 재생. 닫는 점 = 일반 닫기
  //   (빈손 확인 grasp_checks_ · grasp_guard 가 같이 봄), 끝 = 들어 올린 자세 (멈춤). 표는 이웃 없이 학습 → farpick 처럼 목표 중심에서
  //   farpick_nbr_max_m 안 이웃이 있으면 (그 방향 표 없음) 안 씀. 안 되면 false (why) → 지금 쥐기 그대로
  struct LyingPickGrasp
  {
    ArmPath path;                      // LOOK (보기 변형) → 표 점 (lp_in*) → reloc_grasp (닫기) → lp* → reloc_lift (멈춤)
    double tilt_deg{0.0};              // 닫는 점 손 기울임 (접근축 ↔ 수직)
    double depth_mm{0.0};              // 닫는 점 손끝의 누운 윗면 아래 깊이 (모델)
    Eigen::Vector2d in_hand{Eigen::Vector2d::Zero()};   // 손 안 퍽 중심 (held_center_in_hand)
  };
  bool lyingpick_grasp(double x, double y, double top, double axis, LyingPickGrasp & out, std::string & why)
  {
    if (!get_parameter("lyingpick_use").as_bool()) {why = "lyingpick_use false"; return false;}
    if (lyingpick_.empty()) {why = "표 없음 (lyingpick_table)"; return false;}
    const auto look = paths_.find("look");
    if (look == paths_.end() || look->second.nodes.empty()) {why = "경로 look 없음"; return false;}
    char buf[240];
    const double nb_max = get_parameter("farpick_nbr_max_m").as_double();
    for (const auto & n : last_nb_) {
      const double d = (n.c - Eigen::Vector2d(x, y)).norm();
      if (d <= nb_max) {
        std::snprintf(buf, sizeof(buf), "이웃 퍽이 중심에서 %.0f mm (farpick_nbr_max_m %.0f 안) — 표는 이웃 없이 학습", d * 1000.0, nb_max * 1000.0);
        why = buf;
        return false;
      }
    }
    const double max_mm = get_parameter("lyingpick_max_dist_mm").as_double(), max_deg = get_parameter("lyingpick_max_gap_deg").as_double();
    const auto ch = choose_lyingpick(lyingpick_, x, y, axis, max_mm, max_deg);
    if (ch.row < 0) {
      if (ch.nearest < 0) {
        why = "표에 누운 퍽 (side 0.060) 줄 없음";
      } else {
        const auto & n = lyingpick_[static_cast<size_t>(ch.nearest)];
        std::snprintf(buf, sizeof(buf), "가까운 칸 없음 (가장 가까운 (%.3f, %.3f) %.0f°, 차 %.1f mm · %.1f° — lyingpick_max_dist_mm %.0f · lyingpick_max_gap_deg %.0f)",
          n.x, n.y, n.axis_deg, ch.dist_mm, ch.dyaw_deg, max_mm, max_deg);
        why = buf;
      }
      return false;
    }
    const auto & e = lyingpick_[static_cast<size_t>(ch.row)];
    // 시작: LOOK · 보기 변형 자세에서만 (표가 LOOK 근처에서 시작, farpick_plan 과 같음)
    out = LyingPickGrasp{};
    out.path.name = "pick_lying_table";
    out.path.ends_home = false;
    out.path.nodes.push_back(look->second.nodes.back());
    out.path.nodes.back().name = "look";
    out.path.nodes.back().grip = "";
    const char * from = "관찰 자세";
    PathNode viewing;
    std::string w;
    if (at_look_variant(viewing)) {
      out.path.nodes.front() = viewing;
      from = "보기 변형 자세";
    } else if (!path_start_ok(out.path, joint_buf_, path_start_tol_, w)) {
      why = "LOOK 자세에서만 (" + w + ")";
      return false;
    }
    // 실물 맞춤 (lyingpick_dxy_mm · lyingpick_dz_mm): 퍽 근처 구간만 손끝을 더 옮김 (lyingpick_path). 더 깊게 (dz < 0) 면 손끝 최저도 그만큼 내림 —
    //   단 누운 퍽 쥐기 한도 pick_lying_tip_clear_mm (놓인 면 기준, 모델) 아래로는 안 감
    const auto dxy = get_parameter("lyingpick_dxy_mm").as_double_array();
    const double dz_mm = get_parameter("lyingpick_dz_mm").as_double();
    if (dxy.size() != 2 || !std::isfinite(dxy[0]) || !std::isfinite(dxy[1]) || !std::isfinite(dz_mm)) {
      why = "lyingpick_dxy_mm 는 [x, y] · lyingpick_dz_mm 는 숫자여야 함";
      return false;
    }
    const Eigen::Vector3d shift(dxy[0] / 1000.0, dxy[1] / 1000.0, dz_mm / 1000.0);
    StandupRetarget rt;
    rt.from = Eigen::Vector2d(e.x, e.y);
    rt.to = Eigen::Vector2d(x, y);
    rt.da = ch.dyaw_deg * M_PI / 180.0;
    rt.tip_min_z = top - lying_h_ + 0.003;   // 박스 윗면 + 3 mm (top 은 모델 높이, farpick 과 같음)
    if (shift.z() < 0.0) {
      rt.tip_min_z = std::max(rt.tip_min_z + shift.z(), top - lying_h_ + get_parameter("pick_lying_tip_clear_mm").as_double() / 1000.0);
    }
    rt.reach_scale = 1.0;
    LyingPickPath lp;
    LyingHover hv;   // 닫는 점 위 멈춤 → 곧게 내려와 쥐기 (lyingpick_hover_mm, 0 = 표 그대로)
    hv.hover_m = std::max(0.0, get_parameter("lyingpick_hover_mm").as_double()) / 1000.0;
    hv.hover_s = get_parameter("lyingpick_hover_s").as_double();
    hv.descend_s = get_parameter("lyingpick_descend_s").as_double();
    if (!lyingpick_path(model_, e, rt, 0.017, kStandupDt, side_grip_, lp, w, shift, hv)) {why = "표 줄: " + w; return false;}
    char rc[160] = "";
    if (get_parameter("lyingpick_recenter").as_bool()) {   // 표 손끝이 닫힘축으로 어긋난 만큼 되돌려 두 손가락이 퍽 양 옆면에서 같은 거리로 내려오게
      const Grip g0 = arm_grip(model_, lp.q_close);
      const Eigen::Vector2d off0 = g0.tip.head<2>() - Eigen::Vector2d(x, y);
      const Eigen::Vector2d d = lying_recenter_shift(g0, off0);
      if (d.norm() > 0.0005) {
        LyingPickPath lp2;
        std::string w2;
        const Eigen::Vector3d shift2 = shift + Eigen::Vector3d(d.x(), d.y(), 0.0);
        if (lyingpick_path(model_, e, rt, 0.017, kStandupDt, side_grip_, lp2, w2, shift2, hv)) {
          lp = std::move(lp2);
          std::snprintf(rc, sizeof(rc), " · 손가락 사이 가운데로 (%+.1f, %+.1f) mm (lyingpick_recenter)", d.x() * 1000.0, d.y() * 1000.0);
        } else {
          std::snprintf(rc, sizeof(rc), " · 가운데 맞춤 IK 안 됨 → 표 그대로 (%s)", w2.substr(0, 60).c_str());
        }
      }
    }
    if (hv.hover_m > 0.0) {
      const bool has = std::any_of(lp.nodes.begin(), lp.nodes.end(), [](const PathNode & n) {return n.name == "lp_in_hover";});
      RCLCPP_INFO(get_logger(), "누운 퍽 표: 닫는 점 위 %.0f mm 에서 %.1f s 멈춘 뒤 %.1f s 동안 곧게 내려와 쥠%s (lyingpick_hover_mm)", hv.hover_m * 1000.0,
        hv.hover_s, hv.descend_s, has ? "" : " — 멈춤 자세 IK 안 됨 → 표 그대로");
    }
    out.path.nodes.insert(out.path.nodes.end(), lp.nodes.begin(), lp.nodes.end());
    // 쥔 손: 기울임 · 누운 윗면 아래 깊이 (② 내려놓기가 같은 손으로) · 손 안 퍽 중심 (③ 놓은 자리)
    const Grip g = arm_grip(model_, lp.q_close);
    out.tilt_deg = std::acos(std::clamp(std::fabs(g.app.z()), 0.0, 1.0)) * 180.0 / M_PI;
    out.depth_mm = (top - g.tip.z()) * 1000.0;
    out.in_hand = held_center_in_hand(g, Eigen::Vector3d(x, y, top - lying_h_ / 2.0));
    const Eigen::Vector2d off = g.tip.head<2>() - Eigen::Vector2d(x, y);
    char sh[96] = "";
    if (shift.squaredNorm() > 0.0) {
      std::snprintf(sh, sizeof(sh), " · 실물 맞춤 (%+.1f, %+.1f, %+.1f) mm (lyingpick_dxy_mm · dz_mm)", dxy[0], dxy[1], dz_mm);
    }
    RCLCPP_INFO(get_logger(), "누운 퍽 표: 퍽 (%.3f, %.3f) 긴축 %.0f° → 표 줄 %d (%.3f, %.3f) 긴축 %.0f° · 재생 성공 %.2f · 차 %.1f mm · %+.1f° · "
      "닫는 점 %d/%zu · 손 %.0f° · 깊이 %.0f mm · 손끝 퍽 중심에서 (%+.1f, %+.1f) mm%s%s (건너뜀 %d, %s에서)", x, y, axis, ch.row, e.x, e.y, e.axis_deg,
      e.rate, ch.dist_mm, ch.dyaw_deg, e.release, e.q.size(), out.tilt_deg, out.depth_mm, off.x() * 1000.0, off.y() * 1000.0, sh, rc, lp.skipped, from);
    return true;
  }

  // 먼 · 옆 누운 퍽 (11-09 21:4x, regrasp_nodes 와 같은 생각): 가까운 자리 (park_near_max_r_m, J1 에서 거리) 밖이거나 지금 자리에서 park_plan 이 거절되면
  //   ① 퍽 가운데를 누운 채 쥠 (일반 누운 퍽 PICK 쥐기: 닫힘 = 긴축에 수직, 수직 손 먼저 · 안 닿으면 기울인 손)
  //      — 누운 퍽 표 (lyingpick_grasp) 가 되면 그 표 재생으로 쥐고 듦. 표로 쥔 손으로 ② 자리가 없으면 지금 쥐기로 다시
  //   ② 가까운 빈 자리 (J1 → 퍽 선 위 박스 앞쪽 끝부터, regrasp_min_x · regrasp_max_abs_y · regrasp_search_s 그대로) 에 쥘 때 기울임 그대로 내려놓고 손을 위로.
  //      긴축 = 그 자리의 반지름 방향 (가까운 자리에서 실물 · 시험으로 되는 방향. lying_grasp 가 바깥 끝을 쥐고, 닫힘축 = 접선이라 기울인 손도 맞음)
  //   ③ 그 자리 · 긴축 (손이 놓은 그대로, 다시 보기 없음) 에서 지금의 park_plan 그대로. 가까운 자리에서 되면 park_plan 만 (경로 · 값 그대로). park_relocate false = 옮기지 않음
  std::vector<ArmAction> park_or_relocate_plan(const ArmPath & unfold, double x, double y, double top,
    const std::vector<std::pair<double, std::string>> & cands, std::string & open_kw, std::string & why)
  {
    const bool relocate = get_parameter("park_relocate").as_bool();
    const double near_r = get_parameter("park_near_max_r_m").as_double();
    const Eigen::Vector2d p0(x, y);
    char buf[200];
    std::string reason;
    if (!relocate || park_near(model_.j1_xy, p0, near_r)) {
      auto plan = park_plan(unfold, x, y, top, cands, open_kw, why);
      if (!plan.empty() || !relocate) {return plan;}
      reason = "지금 자리에서 세우기 계획 안 됨: " + why;
      why.clear();
    } else {
      std::snprintf(buf, sizeof(buf), "J1 에서 %.3f m, 가까운 자리 %.3f m 밖 (park_near_max_r_m)", (p0 - model_.j1_xy).norm(), near_r);
      reason = buf;
    }
    const double axis = lying_axis_deg_;   // 아래 자리 검사 (grasp_for) 가 지움
    if (!std::isfinite(axis)) {why = "받침 세우기 옮겨 놓기: 누운 퍽 긴축 방향을 모름"; return {};}
    const auto hl = get_parameter("pick_lying_heights_mm").as_double_array();
    const double clear = get_parameter("pick_lying_tip_clear_mm").as_double();
    if (hl.size() != 3 || !(hl[0] > hl[1] && hl[1] > hl[2] && hl[2] >= clear - lying_h_ * 1000.0)) {
      why = "pick_lying_heights_mm 는 위 > 내림 > 쥐기 3 개 (쥐기는 pick_lying_tip_clear_mm 안)";
      return {};
    }
    // ① 퍽 가운데 쥐기: 강화학습 표 (lyingpick_grasp — LOOK 에서 표 재생, 11-10 실물 220632 빈손) 먼저, 안 되면 지금 쥐기
    //    (누운 퍽 일반 PICK 과 같은 poses_for, 이웃 = 원래 자리의 grasp_for 결과)
    LyingPickGrasp lpg;
    std::string lp_why;
    bool table = lyingpick_grasp(x, y, top, axis, lpg, lp_why);
    if (!table) {RCLCPP_INFO(get_logger(), "누운 퍽 (%.3f, %.3f) 옮기기 쥐기: 표 안 씀 (%s) → 지금 쥐기", x, y, lp_why.c_str());}
    const auto nb_target = last_nb_;   // 원래 자리 이웃 (아래 자리 찾기 grasp_for 가 바꿈 → 표가 안 되어 지금 쥐기로 갈 때 되돌림)
    std::string kw, w;
    std::vector<PickPose> Pg;
    auto now_grasp = [&]() {
        Pg = poses_for(x, y, top, axis, hl, cands, kw, w, 0.0, max_tilt_deg_);
        if (Pg.size() == 3) {return true;}
        why = "받침 세우기 옮겨 놓기 (" + reason + "): 누운 퍽 가운데를 쥘 손이 안 닿음 (" + w + ")";
        return false;
      };
    // ② 가까운 빈 자리: 누운 채 쥘 때 기울임 그대로 내려놓음 (퍽은 누운 채 평평, 바닥이 면 위 place_clear_mm)
    //    표로 쥔 손은 기울임 · 깊이가 표 그대로 (10° 단계 아님) → 놓는 손 기울임 차는 farpick_regrasp_tilt_dev 안 (먼 퍽 표와 같음)
    const double zc = top - model_.team_base_z_floor + model_.base_z_floor - lying_h_ / 2.0;
    const double budget_s = get_parameter("regrasp_search_s").as_double();
    std::string last = "후보 없음";
    std::vector<PathNode> put;
    std::vector<std::pair<double, std::string>> cs;
    Eigen::Vector2d spot = p0;
    double saxis = axis, tilt = 0.0;
    Q5 put_q{};   // 내려놓는 자세 (표로 쥐었을 때 놓은 퍽 중심 계산)
    auto search = [&](double tilt_dev) {
        tilt = table ? lpg.tilt_deg : Pg.back().tilt_deg;
        const double depth = table ? lpg.depth_mm : -hl.back();
        const std::vector<double> hp{place_heights_[0], place_heights_[1], place_clear_mm_ - depth};
        const auto t_search = std::chrono::steady_clock::now();
        exclude_ = p0;   // 집어 드는 퍽 자리는 빈 것으로
        last = "후보 없음";
        put.clear();
        std::vector<Q5> seed_p;
        int looked = 0;
        for (const auto & s : park_relocate_spots(model_.j1_xy, p0, regrasp_min_x_, regrasp_max_abs_y_, near_r)) {
          if (budget_s > 0.0 && std::chrono::duration<double>(std::chrono::steady_clock::now() - t_search).count() > budget_s) {
            last = "옮겨 놓을 자리 찾기 시간 초과 (" + std::to_string(looked) + " 곳 봄)";   // 넘으면 거절 (planning 이 건너뜀)
            break;
          }
          ++looked;
          saxis = park_relocate_axis_deg(model_.j1_xy, s);
          double gx = s.x(), gy = s.y();
          // 빈 자리 · 누운 발자국 · 손가락 여유 (이웃 = 그 자리 → 뒤 park_plan 의 fingers_clear 도 이 이웃)
          if (!grasp_for(gx, gy, zc, saxis, true, cs, w, "옮겨 놓을 자리", true)) {last = w; continue;}
          const auto Pp = poses_for(s.x(), s.y(), top, saxis, hp, {{cs.front().first, "open"}}, kw, w, tilt, tilt_dev, seed_p);
          if (Pp.size() != 3) {last = "내려놓기: " + w; continue;}
          seed_p = {ticks_to_q(model_, Pp[0].ticks)};
          put = {model_node("reloc_put_above", Pp[0].enc, slow_dps_), model_node("reloc_put_down", Pp[1].enc, slow_dps_),
            model_node("reloc_put", Pp[2].enc, slow_dps_, "open"), model_node("reloc_put_up", Pp[1].enc, slow_dps_),
            model_node("reloc_put_clear", Pp[0].enc, slow_dps_)};
          settle_at(put[0], "place_settle_s");   // 내려놓을 자리 위에서 멈춤 (0 = 그대로)
          put_q = ticks_to_q(model_, Pp[2].ticks);
          spot = s;
          break;
        }
        exclude_.reset();
      };
    // ② 자리를 찾은 뒤: 펼치기 뒤에 쥐기 · 옮겨 놓기를 붙이고 그 뒤를 park_plan (pick_path) 이 그대로 이음 (앞이 unfold 그대로라 P0 · LOOK 잇기도 그대로)
    //   표로 쥐면 앞 = LOOK (보기 변형) 자세 → 표 점 (lyingpick_grasp), 지금 자세가 그 첫 점
    auto finish = [&]() {
        ArmPath pre = table ? lpg.path : unfold;
        if (!table) {
          pre.nodes.push_back(model_node("reloc_above", Pg[0].enc, 0.0, kw));
          pre.nodes.back().grip_early = side_grip_;   // 퍽 위로 가며 미리 엶 (빈손)
          settle_at(pre.nodes.back(), "pick_settle_s");
          pre.nodes.push_back(model_node("reloc_down", Pg[1].enc, slow_dps_));
          pre.nodes.push_back(model_node("reloc_grasp", Pg[2].enc, slow_dps_, "close"));
          pre.nodes.push_back(model_node("reloc_up", Pg[1].enc, slow_dps_));
          pre.nodes.push_back(model_node("reloc_lift", Pg[0].enc, slow_dps_));
        }
        pre.nodes.insert(pre.nodes.end(), put.begin(), put.end());
        // ③ 놓은 자리: 표로 쥐었으면 표 보정이 퍽 가운데에서 비켜 쥔 만큼 손 안에 남음 → 놓은 퍽 중심 = 내려놓은 손 + 손 안 자리 (닫힘축 쪽은 손가락이 맞춤)
        Eigen::Vector2d at = spot;
        char tb[160] = "";
        if (table) {
          at = held_center_at(arm_grip(model_, put_q), lpg.in_hand).head<2>();
          std::snprintf(tb, sizeof(tb), " · 표로 쥠: 놓은 퍽 중심 = 놓은 손끝 %+.1f, %+.1f mm", (at.x() - spot.x()) * 1000.0, (at.y() - spot.y()) * 1000.0);
        }
        RCLCPP_INFO(get_logger(), "누운 퍽 (%.3f, %.3f) 가 멀어 (%s) → (%.3f, %.3f) 에 누운 채 옮겨 놓고 거기서 세움 (쥘 때 손 %.0f° · 놓은 긴축 %.0f°)%s",
          x, y, reason.c_str(), at.x(), at.y(), tilt, saxis, tb);
        // ③ 놓은 자리 · 긴축 = 손이 놓은 그대로 → 지금의 받침 세우기 계획 그대로
        lying_axis_deg_ = saxis;
        const std::vector<std::pair<double, std::string>> c_spot{{cs.front().first, "open"}};
        auto plan = park_plan(pre, at.x(), at.y(), top, c_spot, open_kw, why);
        if (plan.empty()) {
          std::snprintf(buf, sizeof(buf), "옮겨 놓은 자리 (%.3f, %.3f) 에서 ", at.x(), at.y());
          why = buf + why;
        }
        return plan;
      };
    if (table) {   // 표로 쥔 손으로 내려놓을 자리 · 그 자리 세우기가 안 되면 지금 쥐기로 (거절 늘리지 않음)
      search(get_parameter("farpick_regrasp_tilt_dev").as_double());
      if (!put.empty()) {
        auto plan = finish();
        if (!plan.empty()) {return plan;}
        RCLCPP_WARN(get_logger(), "누운 퍽 (%.3f, %.3f) 옮기기: 표로 쥐면 계획 안 됨 (%s) → 지금 쥐기로", x, y, why.c_str());
      } else {
        RCLCPP_WARN(get_logger(), "누운 퍽 (%.3f, %.3f) 옮기기: 표로 쥔 손 (기울임 %.0f° · 깊이 %.0f mm) 으로 내려놓을 가까운 빈 자리 없음 (%s) → 지금 쥐기로",
          x, y, lpg.tilt_deg, lpg.depth_mm, last.c_str());
      }
      table = false;
      why.clear();
      last_nb_ = nb_target;
    }
    if (!now_grasp()) {return {};}
    search(0.5);
    if (put.empty()) {
      why = "받침 세우기 옮겨 놓기 (" + reason + "): 가까운 빈 자리가 없음 (마지막: " + last + ")";
      // 마지막 길: 강화학습 세우기 표 (standup_plan, 팔을 멀리 뻗는 표 — 받침 · 옮기기가 다 안 될 때만). 표에 맞는 칸이 없으면 그 이유로 거절
      if (get_parameter("park_table_fallback").as_bool()) {
        std::string w2;
        lying_axis_deg_ = axis;
        auto plan = standup_plan(unfold, x, y, top, w2);
        if (!plan.empty()) {
          RCLCPP_WARN(get_logger(), "누운 퍽 (%.3f, %.3f): 받침 세우기 · 옮겨 놓기 안 됨 (%s) → 강화학습 세우기 표로", x, y, why.c_str());
          return plan;
        }
        why += " · 세우기 표 (" + w2 + ")";
      }
      return {};
    }
    return finish();
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
    settle_at(path.nodes.back(), "pick_settle_s");   // 11-09 실물: 흔들린 채 도착해 바로 내려감 → 퍽 위에서 멈췄다 내려감 (가며 열기는 그대로, 0 = 그대로)
    path.nodes.push_back(model_node("down", P[1].enc, slow_dps_));
    path.nodes.push_back(model_node("grasp", P[2].enc, slow_dps_, "close"));
    path.nodes.push_back(model_node("up", P[1].enc, slow_dps_));
    path.nodes.push_back(model_node("lift", P[0].enc, slow_dps_));
    path.nodes.insert(path.nodes.end(), again.begin(), again.end());
    // 박스 위 기준점: 집은 뒤 왼쪽 (기준점 y +38 mm) 으로 갈 이유 없음 → 기본은 안 감 (pick_end_at_staging false).
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
    // 쥔 퍽 영상 검사 (held_image): 수직 손 선 퍽 · 다시 잡기 없는 PICK 만. 들기 (lift) 에서 멈추고 그 뒤에 논리 대기 하나
    const bool held_check = get_parameter("held_image").as_bool() && !lying && again.empty() && P.back().tilt_deg < 0.5;
    if (held_check) {
      for (auto & n : path.nodes) {
        if (n.name == "lift") {n.stop = true;}
      }
    }
    auto plan = visual_regrasp_actions(path, true);
    if (held_check && !held_image_insert(plan, "lift")) {
      RCLCPP_WARN(get_logger(), "쥔 퍽 영상: PICK 계획에 들기 (lift) 동작이 없음 → 이번 PICK 은 안 봄");
    } else if (get_parameter("held_image").as_bool() && !held_check) {
      RCLCPP_INFO(get_logger(), "쥔 퍽 영상: %s → 이번 PICK 은 안 봄", lying ? "누운 퍽" : (!again.empty() ? "기울여 쥐고 세워 다시 잡기" : "기울인 손"));
    }
    return plan;
  }

  // ------------------------------------------------------------------------ held_image (쥔 퍽 영상 검사, 11-11)
  // plan 의 pose 동작 바로 뒤에 검사 논리 대기를 끼우고 그 뒤 원래 동작을 held_img_.rest 로 기억. 없으면 false
  bool held_image_insert(std::vector<ArmAction> & plan, const std::string & pose)
  {
    const auto it = std::find_if(plan.begin(), plan.end(), [&](const ArmAction & a) {return !a.is_gripper && a.pose == pose;});
    if (it == plan.end()) {return false;}
    ArmAction mark{"lift", false, kHeldImageCheck, 0.0};
    mark.duration_s = std::max(0.5, get_parameter("held_image_timeout_s").as_double()) + 0.5;
    const auto at = plan.insert(it + 1, mark);
    held_img_.rest.assign(at + 1, plan.end());
    return true;
  }

  std::string held_image_shape() const {return gg_shape_ == PuckShape::Cylinder ? "cylinder" : "box";}   // 모름 = 네모 (grasp_guard 와 같음)

  std::string held_img_file() const
  {
    std::string f = get_parameter("held_image_ref_file").as_string();
    if (f.rfind("~/", 0) == 0 && std::getenv("HOME")) {f = std::getenv("HOME") + f.substr(1);}
    return f;
  }

  void load_held_image_refs_now()
  {
    std::string err;
    held_img_refs_ = load_held_image_refs(held_img_file(), err);
    if (held_img_refs_.empty()) {
      RCLCPP_INFO(get_logger(), "쥔 퍽 영상 기준 없음 (%s) → held_image 켜도 판단 안 함 (잘 쥔 뒤 ~/save_held_image_ref 로 저장)", held_img_file().c_str());
      return;
    }
    for (const auto & [k, r] : held_img_refs_) {
      RCLCPP_INFO(get_logger(), "쥔 퍽 영상 기준 %s: 위 모서리 %.1f px · %.1f° · 왼쪽 %.1f px · %.1f°%s (%d 장, %s)", k.c_str(), r.s.top_y,
        r.s.top_deg, r.s.left_x, r.s.left_deg, r.s.left_ok ? "" : " (왼쪽 못 잼)", r.n, r.when.c_str());
    }
  }

  void on_held_image(const interfaces::msg::DetectionArray & m)
  {
    held_img_last_rx_ = now().seconds();
    const double t = rclcpp::Time(m.header.stamp, get_clock()->get_clock_type()).seconds();
    for (const auto & d : m.detections) {
      if (d.kind != "held_image") {continue;}
      HeldImageSample s;
      s.stamp = t;
      s.top_y = d.pose.position.x;
      s.left_x = d.pose.position.y;
      s.bottom_y = d.pose.position.z;
      s.top_deg = d.pose.orientation.x;
      s.left_deg = d.pose.orientation.y;
      s.left_ok = d.yaw_known && std::isfinite(s.left_x) && std::isfinite(s.left_deg);
      s.conf = d.confidence;
      s.label = d.label;
      s.shape = d.shape;
      if (!std::isfinite(s.top_y) || !std::isfinite(s.top_deg)) {continue;}
      held_img_buf_.push_back(s);
    }
    while (!held_img_buf_.empty() && (held_img_buf_.size() > 200 || held_img_buf_.front().stamp < t - 3.0)) {held_img_buf_.pop_front();}
  }

  // 잘 쥔 상태 (PICK 끝 들기 자세에서 멈춤) 에서 사람이 부름: 최근 1 s 의 쥔 퍽 영상 3 장 이상 중앙값 → 이 모양 기준 · 파일
  bool save_held_image_ref(std::string & msg)
  {
    if (!carrying_) {msg = "퍽을 쥐고 있지 않음 (PICK 뒤 들기 자세에서 부를 것)"; return false;}
    // 모양 = 지난 PICK 검출 (grasp_guard gg_shape_). 검출 없이 집었으면 (robot_cli pick x y) 네모로 — 검사도 모름 = 네모로 비교
    const std::string shape = held_image_shape();
    const double t = now().seconds();
    const auto v = samples_between(held_img_buf_, t - 1.0, t + 1.0);
    if (v.size() < 3) {
      msg = "최근 1 s 쥔 퍽 영상 " + std::to_string(v.size()) + " 장 (3 장 필요) — perception wrist.held.enable 켰는지, 팔이 멈췄는지";
      return false;
    }
    HeldImageRef r;
    r.ok = true;
    r.s = median_sample(v);
    r.n = static_cast<int>(v.size());
    {
      const std::time_t tt = std::time(nullptr);
      char tb[32];
      std::strftime(tb, sizeof(tb), "%Y-%m-%dT%H:%M:%S", std::localtime(&tt));
      r.when = tb;
    }
    auto refs = held_img_refs_;
    refs[shape] = r;
    std::string err;
    if (!save_held_image_refs(held_img_file(), refs, err)) {msg = err; return false;}
    held_img_refs_ = refs;
    char buf[256];
    std::snprintf(buf, sizeof(buf), "쥔 퍽 영상 기준 저장 (%s%s, %d 장): 위 모서리 %.1f px · %.1f° · 왼쪽 %.1f px · %.1f°%s → %s", shape.c_str(),
      gg_shape_ == PuckShape::Unknown ? " — PICK 검출 없어 네모로" : "", r.n,
      r.s.top_y, r.s.top_deg, r.s.left_x, r.s.left_deg, r.s.left_ok ? "" : " (왼쪽 못 잼)", held_img_file().c_str());
    msg = buf;
    RCLCPP_INFO(get_logger(), "%s", msg.c_str());
    return true;
  }

  // 검사 대기 중 (50 Hz, runner_.tick 앞): 멈춘 뒤 0.1 s 부터의 장이 2 장 모이고 held_image_settle_s 지나면 (아니면 timeout 에 있는 장으로) 판단
  //   괜찮음 · 못 봄 · 횟수 다 함 · 다시 쥐기 계획 안 됨 → 대기만 끝내 원래 계획 그대로. 잘못 쥠 → 남은 계획을 내려놓기 · 다시 보기 · 다시 쥐기 + 검사 + 원래 나머지로
  void advance_held_image(double now_s)
  {
    if (!held_img_.waiting) {return;}
    if (runner_.gen() != held_img_.gen || !runner_.busy()) {held_img_.waiting = false; return;}
    if (!arm_goal_ || arm_goal_->is_canceling() || !hw_ok_before_ || !arbiter_.arm_may_move()) {
      held_img_.waiting = false;
      runner_.on_result(held_img_.gen, true);
      return;
    }
    const double settle = std::max(0.15, get_parameter("held_image_settle_s").as_double());
    const double timeout = std::max(settle, get_parameter("held_image_timeout_s").as_double());
    const auto v = samples_between(held_img_buf_, held_img_.t0 + 0.1, now_s + 10.0);
    if (!((now_s >= held_img_.t0 + settle && v.size() >= 2) || now_s >= held_img_.t0 + timeout)) {return;}
    held_img_.waiting = false;
    ++held_img_.checks;
    const int max = std::max(0, static_cast<int>(get_parameter("held_image_retry_max").as_int()));
    const bool force = held_img_.tries == 0 && get_parameter("held_image_force").as_bool();
    const std::string shape = held_image_shape();
    const auto ref = held_img_refs_.find(shape);
    HeldImageConfig cfg;
    cfg.tol_mm = get_parameter("held_image_tol_mm").as_double();
    cfg.back_tol_mm = get_parameter("held_image_back_tol_mm").as_double();
    cfg.tilt_deg = get_parameter("held_image_tilt_deg").as_double();
    {
      const auto px = get_parameter("held_image_px_per_mm").as_double_array();
      if (px.size() == 2) {cfg.px_per_mm_front = px[0]; cfg.px_per_mm_back = px[1];}
    }
    HeldImageVerdict verdict;
    std::string what;
    if (v.empty()) {
      char b[200];
      std::snprintf(b, sizeof(b), "못 봄 — %.1f s 동안 쥔 퍽 0 장 (%s)", timeout,
        held_img_last_rx_ < 0.0 || now_s - held_img_last_rx_ > 1.0 ? "/held_puck/image 안 옴: ros2 param set /perception wrist.held.enable true" :
        "영상은 오는데 쥔 퍽 마스크 없음");
      what = b;
    } else if (ref == held_img_refs_.end()) {
      const auto m = median_sample(v);
      char b[200];
      std::snprintf(b, sizeof(b), "%s 기준 없음 (~/save_held_image_ref) — 잰 값 위 모서리 %.1f px · %.1f° · 왼쪽 %.1f px · %.1f° (%zu 장)", shape.c_str(),
        m.top_y, m.top_deg, m.left_x, m.left_deg, v.size());
      what = b;
    } else {
      verdict = judge_held_image(median_sample(v), static_cast<int>(v.size()), ref->second, cfg);
      what = verdict.text;
    }
    const bool bad = force || (verdict.judged && verdict.bad);
    const std::string tag = held_img_.tries == 0 ? "PICK 들기 끝" : "박스에서 다시 쥔 뒤 " + std::to_string(held_img_.tries);
    if (!bad) {
      RCLCPP_INFO(get_logger(), "쥔 퍽 영상 (%s, %s): %s → 그대로", tag.c_str(), shape.c_str(), what.c_str());
      if (held_img_.tries > 0) {held_img_.note += verdict.judged ? " → 다시 쥔 뒤 맞음" : " → 다시 쥔 뒤 못 잼";}
      runner_.on_result(held_img_.gen, true);
      return;
    }
    if (held_img_.tries >= max) {
      RCLCPP_WARN(get_logger(), "쥔 퍽 영상 (%s, %s): %s%s → 다시 쥐기 다 함 (held_image_retry_max %d) → 그대로 진행", tag.c_str(), shape.c_str(),
        what.c_str(), force ? " [held_image_force 시험]" : "", max);
      if (held_img_.tries > 0) {held_img_.note += " → 다시 쥔 뒤에도 허용 밖";}
      runner_.on_result(held_img_.gen, true);
      return;
    }
    std::string why;
    if (!held_image_regrasp(why)) {
      RCLCPP_WARN(get_logger(), "쥔 퍽 영상 (%s, %s): %s%s → 다시 쥐기 못 함 (%s) → 그대로 진행", tag.c_str(), shape.c_str(), what.c_str(),
        force ? " [held_image_force 시험]" : "", why.c_str());
      runner_.on_result(held_img_.gen, true);
      return;
    }
    char b[96];
    std::snprintf(b, sizeof(b), "쥔 퍽 영상: 박스에서 다시 쥠 %d 번", held_img_.tries);
    held_img_.note = b;
    RCLCPP_WARN(get_logger(), "쥔 퍽 영상 (%s, %s): %s%s → 쥐어 간 박스 자리에 내려놓고 다시 보고 다시 쥠 (%d/%d)", tag.c_str(), shape.c_str(),
      what.c_str(), force ? " [held_image_force 시험: 잘못 쥔 것으로 봄]" : "", held_img_.tries, max);
  }

  // 잘못 쥠: 지금 (들기 끝) 에서 쥐어 간 박스 자리 (마지막 닫기 손끝) 에 내려놓고 열기 → 다시 보기 → 수직 다시 쥐기 → 검사 → 원래 나머지
  //   store_recover 와 같은 regrasp_nodes (origin_first) · visual_regrasp_actions · visual_suffix_. 못 하면 false (원래 계획 그대로)
  bool held_image_regrasp(std::string & why)
  {
    if (!model_ok_) {why = "팔 모델 없음"; return false;}
    if (!gg_last_close_) {why = "쥔 자리 (마지막 닫기) 모름"; return false;}
    const auto xy = command_tip_xy(gg_last_close_->q_close);
    if (!xy) {why = "쥔 자리 손끝 계산 안 됨"; return false;}
    if (joint_buf_.size() != 5) {why = "관절 실측 없음"; return false;}
    // 박스 위 선 퍽 윗면 (모델 z) + pick_top_bias_mm: 박스 PICK 과 같은 실제 깊이로 내려놓고 다시 쥠 (store_recover 와 같음)
    const double tb = get_parameter("pick_top_bias_mm").as_double();
    const double top = surface_z_ + puck_h_ + model_.team_base_z_floor - model_.base_z_floor +
      (std::isfinite(tb) && std::fabs(tb) <= 30.0 ? tb / 1000.0 : 0.0);
    const double depth = pending_depth_ > 0.0 ? pending_depth_ : -pick_heights_.back();
    std::vector<PathNode> box;
    double sx = 0.0, sy = 0.0;
    exclude_.reset();
    if (!regrasp_nodes(xy->x(), xy->y(), top, 0.0, depth, box, sx, sy, why, 0.5, true)) {why = "박스 자리: " + why; return false;}
    ArmPath path{"held_image_regrasp", {}, false};
    path.nodes.push_back({"lift", joint_buf_, false, 0.0, "", false});
    path.nodes.insert(path.nodes.end(), box.begin(), box.end());
    auto plan = visual_regrasp_actions(path, true);   // 다시 쥔 닫기 빈손 · 폭 확인 · 다시 보기 대기 (regrasp_visual)
    if (plan.empty()) {why = "다시 쥐기 동작이 비어 있음"; return false;}
    ArmAction mark{"lift", false, kHeldImageCheck, 0.0};
    mark.duration_s = std::max(0.5, get_parameter("held_image_timeout_s").as_double()) + 0.5;
    std::vector<ArmAction> suffix{mark};
    suffix.insert(suffix.end(), held_img_.rest.begin(), held_img_.rest.end());
    plan.insert(plan.end(), suffix.begin(), suffix.end());
    visual_suffix_ = suffix;   // 다시 보고 쥐기가 후반을 바꿔도 끝에 검사 · 나머지
    if (!runner_.replace_after_wait(held_img_.gen, plan)) {visual_suffix_.clear(); why = "작업 상태가 바뀜"; return false;}
    ++held_img_.tries;
    RCLCPP_INFO(get_logger(), "쥔 퍽 영상: 쥐어 간 자리 (%.3f, %.3f) → 박스 (%.3f, %.3f) 에 내려놓기 (동작 %zu 개)", xy->x(), xy->y(), sx, sy, plan.size());
    return true;
  }

  // 경로 → 동작 (+ 빈손 확인 지점). regrasp_visual 이면 이 경로의 관찰 점 (re_observe) 이 regrasp_nodes 가 기억한 점일 때만
  //   그 동작 뒤에 다시 보기 대기 하나를 끼움 (원래 동작 · 그리퍼 순서는 그대로, 대기에서 못 바꾸면 그대로 재생)
  std::vector<ArmAction> visual_regrasp_actions(const ArmPath & path, bool checks)
  {
    auto plan = path_actions(path, timing_, grip_, joint_buf_);
    if (checks) {set_grasp_checks(path, plan);}
    if (!visual_enabled_ || visual_obs_q_.empty()) {return plan;}
    const auto n = std::find_if(path.nodes.begin(), path.nodes.end(), [](const PathNode & p) {return p.name == kRegraspObserveNode;});
    if (n == path.nodes.end() || n->q != visual_obs_q_) {return plan;}
    auto split = split_regrasp_observation(path, std::move(plan), true, get_parameter("regrasp_visual_timeout_s").as_double());
    if (!split.tail.nodes.empty()) {visual_tail_ = std::move(split.tail);}
    return split.plan;
  }

  // 다시 보기 대기 중 (50 Hz, runner_.tick 앞): 새 관측 → 수직 IK → 남은 계획 교체. 안 되면 대기만 끝내 원래 계획 그대로 (fallback)
  //   취소 · 하드웨어 이상은 대기를 끝내고 ArmRunner 의 원래 처리로 (이 기능 때문에 새 실패 · 거절 없음)
  void advance_visual_regrasp(double now_s)
  {
    if (!visual_waiting_) {return;}
    if (runner_.gen() != visual_gen_ || !runner_.busy()) {visual_waiting_ = false; return;}   // runner 가 이미 대기를 놓음
    if (!arm_goal_ || arm_goal_->is_canceling() || !hw_ok_before_ || !arbiter_.arm_may_move()) {
      visual_waiting_ = false;
      runner_.on_result(visual_gen_, true);
      return;
    }
    auto fallback = [&](const std::string & why) {
        visual_waiting_ = false;
        visual_note_ = "다시 보기 안 씀: " + why;
        RCLCPP_WARN(get_logger(), "다시 쥐기: %s → 보지 않고 원래 자리에서 쥠", why.c_str());
        runner_.on_result(visual_gen_, true);   // 원래 후반 계획 · 빈손 확인 그대로
      };
    std::vector<RegraspObservation> obs;
    try {
      const auto tf = tf_buf_->lookupTransform("base_link", odom_frame_, tf2::TimePointZero);
      const auto & r = tf.transform.rotation;
      const Eigen::Quaterniond Q(r.w, r.x, r.y, r.z);
      const Eigen::Vector3d T(tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z);
      const double dyaw = std::atan2(2.0 * (r.w * r.z + r.x * r.y), 1.0 - 2.0 * (r.y * r.y + r.z * r.z));
      for (const auto & p : seen_) {
        obs.push_back({Q * p.p + T, p.capture_t, p.id, !p.round_r30 && !p.lying, p.exact_tf, p.yaw_known, p.yaw + dyaw});
      }
    } catch (const tf2::TransformException &) {}   // TF 가 오면 다음 주기에
    RegraspTarget tg;
    tg.expected = visual_expected_;
    tg.id = !visual_id_.empty() ? visual_id_ : arm_goal_->get_goal()->puck_id;   // STORE 보정: 쥔 퍽 ID (STORE goal 에는 퍽 ID 없음)
    tg.after = visual_after_;
    const double age = get_parameter("regrasp_visual_max_age_s").as_double();
    tg.max_age = std::isfinite(age) && age > 0.0 ? age : 0.5;
    tg.id_radius = id_max_offset_m_;
    tg.max_shift_m = get_parameter("regrasp_visual_max_shift_mm").as_double() / 1000.0;
    tg.top = visual_top_;
    tg.close_deg = visual_close_deg_;
    tg.square = target_square_;
    {   // 다시 쥐기 보정 [앞, 왼쪽] mm 은 보고 다시 쥘 때도 같게 (11-11 실물: 다시 보고 쥔 퍽을 1–2 mm 앞으로 쥠 → 주말에 값만 맞추게)
      const auto ro = get_parameter("regrasp_offset_mm").as_double_array();
      if (ro.size() == 2 && std::isfinite(ro[0]) && std::isfinite(ro[1])) {tg.offset = Eigen::Vector2d(ro[0], ro[1]) / 1000.0;}
    }
    tg.heights = pick_heights_;
    tg.seed = visual_seed_;
    if (visual_tail_.nodes.size() < 6) {fallback("후반 경로 없음"); return;}
    const auto t_ik = std::chrono::steady_clock::now();
    auto d = decide_visual_regrasp(obs, tg, now_s, visual_deadline_, model_, visual_tail_);   // 수직 IK 는 원래 해를 씨앗으로 몇 ms
    const double ik_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t_ik).count();
    if (d.decision == RegraspDecision::WAIT) {return;}
    if (d.decision == RegraspDecision::FALLBACK) {
      fallback(d.why + " · " + std::to_string(std::lround((now_s - visual_start_) * 1000.0)) + " ms 기다림");
      return;
    }
    ArmPath & tail = d.tail;
    std::string why;
    if (joint_buf_.size() != 5 || !path_start_ok(tail, joint_buf_, path_start_tol_, why)) {fallback("팔이 손 뺀 자세가 아님 (" + why + ")"); return;}
    auto plan = path_actions(tail, timing_, grip_, joint_buf_);
    if (plan.empty()) {fallback("새 후반 계획이 비어 있음"); return;}
    plan.insert(plan.end(), visual_suffix_.begin(), visual_suffix_.end());   // STORE 보정: 다시 쥔 뒤 칸 경로 다시 계획 (논리 대기)
    if (!runner_.replace_after_wait(visual_gen_, plan)) {fallback("작업 상태가 바뀜"); return;}
    if (store_recovering_) {store_keep_regrip(tail);}   // 다시 쥔 점 (다시 넣기 계획이 안 되면 그 자리에 되돌려 놓음)
    set_grasp_checks(tail, plan);   // 남은 닫기 (re_grasp) 의 빈손 확인을 새 re_up 기준으로
    visual_waiting_ = false;
    const Eigen::Vector2d xy = d.obs.center.head<2>(), dv = xy - visual_blind_xy_;
    char buf[96];
    std::snprintf(buf, sizeof(buf), "다시 보고 쥠 (%+.1f, %+.1f) mm%s", dv.x() * 1000.0, dv.y() * 1000.0, d.low_above ? " · 위 점 낮춤" : "");
    visual_note_ = buf;
    RCLCPP_INFO(get_logger(), "다시 쥐기: 새 관측 퍽 %s (%.3f, %.3f) · 원래 다시 쥘 점에서 (%+.1f, %+.1f) mm → 그 중심으로 수직 다시 쥠 (높이 · 닫힘 %.0f° 그대로) · "
      "촬영 %.0f ms 전 · 기다림 %.0f ms · IK %.1f ms", d.obs.id.empty() ? "(ID 못 읽음)" : d.obs.id.c_str(), xy.x(), xy.y(),
      dv.x() * 1000.0, dv.y() * 1000.0, visual_close_deg_, (now_s - d.obs.stamp) * 1000.0, (now_s - visual_start_) * 1000.0, ik_ms);
  }

  // 쥐기 (닫기) 뒤 빈손 확인 지점 (run_arm, standup_check_ 와 같은 방식, pick_path · farpick_plan): 닫기 다음 팔 동작을 보내기 전 그리퍼 실측을 봄.
  //   빈손이면 그 동작을 쥐기 높이 위 점 (닫는 점 다음 점: up · re_up) 까지만 보내고 끝 — 받침 세우기 · 다시 쥐기 (again) 의 닫기도 같이
  void set_grasp_checks(const ArmPath & path, const std::vector<ArmAction> & plan)
  {
    grasp_checks_.clear();
    std::vector<std::vector<double>> ups;   // 닫는 점 다음 점 (경로 순서)
    std::vector<bool> holds;                // 그 닫기가 퍽을 쥐는 것 — 받침 세우기의 꽂기 · 밀기 전 닫기 (park_press_turn · park_push_over) 는 빈손이 정상
    std::vector<std::size_t> up_idx;        // 닫는 점 다음 점 번호 (빈손 되돌아가기 출발점)
    std::vector<bool> in_box;               // 빈손이면 퍽이 박스에 남는 닫기 — park_ 닫기 (받침 안 다시 쥐기 park_re_down) 는 퍽이 로봇 위 받침에 남음
    for (std::size_t k = 0; k + 1 < path.nodes.size(); ++k) {
      if (path.nodes[k].grip == "close" && !path.nodes[k].grip_on_way) {
        ups.push_back(path.nodes[k + 1].q);
        holds.push_back(path.nodes[k].name != "park_press_turn" && path.nodes[k].name != "park_push_over");
        up_idx.push_back(k + 1);
        in_box.push_back(path.nodes[k].name.rfind("park_", 0) != 0);
      }
    }
    std::vector<GraspCheck> all;
    for (std::size_t i = 0; i < plan.size(); ++i) {
      if (!plan[i].is_gripper || std::fabs(plan[i].gripper - grip_.close) > 1e-9) {continue;}
      for (std::size_t j = i + 1; j < plan.size(); ++j) {
        if (!plan[j].is_gripper) {all.push_back({plan[j].pose, {}}); break;}
      }
    }
    // 닫기 수가 경로와 맞을 때만 (어느 닫기가 빈손 정상인지 알 때만) 검사 — 11-09 22:30 실물: 받침에 넣고 누르기 전 빈손 닫기를 빈손으로 보고 멈춤
    if (ups.size() == all.size()) {
      for (std::size_t i = 0; i < ups.size(); ++i) {
        if (!holds[i]) {continue;}
        all[i].up_q = ups[i];
        all[i].in_box = in_box[i];
        if (in_box[i]) {all[i].back = empty_retreat_path(path, up_idx[i], all[i].back_note);}   // 받침 안이면 되돌아가기 없음
        grasp_checks_.push_back(all[i]);
      }
    }
    gg_set_regrip(path);   // grasp_guard: 닫기마다 다시 쥐기 정보 (같은 순서)
  }

  // 먼 · 옆 퍽 붙은 선 퍽 집기 (11-10 새벽 강화학습 표 config/farpick/, 시뮬 88.9 % · 네모 80.5 %): 이웃 퍽 방향 (45°) 표에서 가장 가까운 칸을
  //   실제 퍽으로 옮겨 (standup_plan 과 같은 방식: nearest · standup_retarget · encoder_goal_q) LOOK (옆 보기) 자세에서 표 그대로 재생.
  //   시뮬 일정: LOOK → 위 (150 °/s) → 0.3 s 멈춤 → 중간 · 쥐기 30 °/s → 닫기 0.7 s (팔 멈춤) → 들기 30 °/s (표 점 간격 0.1 s).
  //   위로 가며 엶 (pick_path 의 grip_early), 닫는 점에서 일반 닫기 (빈손 검사 grasp_checks_ 도 같이), 들어 올린 자세 뒤는 PICK 과 같은 마무리
  std::vector<ArmAction> farpick_plan(double x, double y, double top, double yaw, bool square, std::string & why)
  {
    if (farpick_.empty()) {why = "먼 퍽 표 없음 (farpick_dir)"; return {};}
    const auto look = paths_.find("look");
    if (look == paths_.end() || look->second.nodes.empty()) {why = "먼 퍽 표: 경로 look 없음"; return {};}
    // 이웃: 목표 퍽 중심에서 farpick_nbr_max_m 안 가장 가까운 것의 방향 (base_link) → 45° 표, 없으면 nbr_none
    const double nb_max = get_parameter("farpick_nbr_max_m").as_double();
    bool has_nb = false;
    double nb_dir = 0.0, nb_d = 1e9;
    for (const auto & n : last_nb_) {
      const double d = (n.c - Eigen::Vector2d(x, y)).norm();
      if (d <= nb_max && d < nb_d) {nb_d = d; has_nb = true; nb_dir = std::atan2(n.c.y() - y, n.c.x() - x) * 180.0 / M_PI;}
    }
    const std::string tname = farpick_table_name(has_nb, nb_dir);
    const auto tb = farpick_.find(tname);
    if (tb == farpick_.end()) {why = "먼 퍽 표 " + tname + " 없음 (farpick_dir)"; return {};}
    // 표 칸: side = 퍽 종류 (네모 0.0348 · 원기둥 0.030), 가장 가까운 (x, y, axis) — 네모 90° 주기, 원기둥 (면 모르는 네모) 은 axis 무시
    const double side = square ? lying_h_ : 0.030;
    const bool use_axis = square && std::isfinite(yaw);
    double dx = 0.0, dy = 0.0, da = 0.0;
    const int k = nearest_farpick(tb->second, x, y, use_axis ? yaw : 0.0, side, dx, dy, da, use_axis ? 90.0 : 0.0);
    if (k < 0) {why = "먼 퍽 표 " + tname + " 가 비어 있음"; return {};}
    const auto & e = tb->second[static_cast<size_t>(k)];
    char buf[240];
    if (std::hypot(dx, dy) * 1000.0 > get_parameter("farpick_max_gap_mm").as_double() || std::fabs(da) > get_parameter("farpick_max_gap_deg").as_double()) {
      std::snprintf(buf, sizeof(buf), "먼 퍽 표 %s 에 가까운 칸이 없음 (퍽 (%.3f, %.3f) %.0f° ↔ 표 (%.3f, %.3f) %.0f°, 차 %.1f mm · %.1f°)", tname.c_str(),
        x, y, use_axis ? yaw : 0.0, e.x, e.y, e.axis_deg, std::hypot(dx, dy) * 1000.0, da);
      why = buf;
      return {};
    }
    if (e.release < 1 || e.release + 1 >= static_cast<int>(e.q.size())) {why = "먼 퍽 표 " + tname + ": 닫는 점 번호가 이상함 (위 점 없음)"; return {};}
    // 옮기기: 표 칸 → 잰 퍽 (수평 이동 + 방향 차 회전), 손끝 최저 = 박스 윗면 + 3 mm, 덜 뻗기 없음
    StandupRetarget rt;
    rt.from = Eigen::Vector2d(e.x, e.y);
    rt.to = Eigen::Vector2d(x, y);
    rt.da = da * M_PI / 180.0;
    rt.tip_min_z = top - puck_h_ + 0.003;   // top 은 모델 높이 (target_in_model)
    rt.reach_scale = 1.0;
    const double kg = 0.017;   // 닫는 점부터 쥔 무게 (처짐 앞먹임)
    ArmPath path;
    path.name = "pick_far";
    path.ends_home = false;
    path.nodes.push_back(look->second.nodes.back());   // 표 첫 점 = LOOK 자세
    path.nodes.back().name = "look";
    path.nodes.back().grip = "";
    Q5 q_close{};
    int skipped = 0, gap = 1;   // gap: 건너뛴 점만큼 다음 점까지 시간을 늘림 (standup_plan 과 같음)
    for (size_t i = 1; i < e.q.size(); ++i) {
      Ticks tt;
      for (int j = 0; j < 5; ++j) {tt[j] = e.q[i][j];}
      RetargetResult rr;
      const bool held = static_cast<int>(i) >= e.release;
      if (!standup_retarget(model_, ticks_to_q(model_, tt), rt, false, rr)) {
        if (static_cast<int>(i) == e.release) {why = "먼 퍽 표: 닫는 점을 실제 퍽으로 옮긴 자세 (IK) 가 없음 — " + rr.fail; return {};}
        ++skipped;
        ++gap;
        continue;
      }
      PathNode n = model_node("far" + std::to_string(i), q_to_ticks(model_, encoder_goal_q(model_, rr.q, held ? kg : 0.0)), 0.0, "");
      if (i == 1) {   // 위로 가며 엶 (pick_path 의 above 와 같은 grip_early). 속도 시간표 (옆 보기에서는 J1 도 같이 돌아 0.1 s 로는 빠름)
        n.grip = "open";
        n.grip_early = side_grip_;
      } else {
        n.dt_s = kStandupDt * gap;   // 표 점 간격 0.1 s 그대로 (시뮬 일정)
      }
      if (static_cast<int>(i) == e.release) {   // 닫는 점: 일반 닫기 (팔 멈춤), 위 점 = 다음 점 (grasp_checks_)
        n.name = "far_grasp";
        n.grip = "close";
        q_close = rr.q;
      }
      gap = 1;
      path.nodes.push_back(n);
    }
    if (path.nodes.back().grip == "close") {why = "먼 퍽 표: 닫는 점 뒤 점이 전부 IK 안 됨"; return {};}
    path.nodes.back().name = "far_lift";
    path.nodes.back().stop = true;   // 시간 맞춘 구간은 들어 올린 자세에서 끝
    if (get_parameter("pick_end_at_staging").as_bool()) {   // 끝은 pick_path 와 같게 (기본은 집은 자리에서 끝, STORE 가 이어 감)
      path.nodes.push_back({"staging", staging_, false, 0.0, "", false});
    }
    // 정면 · 옆 · 먼 · 가까운 보기에서 표 두 번째 점으로 바로 감.
    std::string dummy;
    const char * from = "관찰 자세";
    PathNode viewing;
    if (at_look_variant(viewing)) {path.nodes.front() = viewing; from = "보기 변형 자세";}
    else if (!path_start_ok(path, joint_buf_, path_start_tol_, dummy)) {why = "먼 퍽 표: LOOK 에서만 (지금 자세가 보기 자세 아님)"; return {};}
    // 쥔 상태 (STORE 가 storeheld 경로 store_<칸>_d40_tilt<N> 등을 고르게): 닫는 점 손끝의 윗면 아래 깊이 · 손 기울임 (접근축 ↔ 수직)
    const Grip g = arm_grip(model_, q_close);
    pending_tilt_ = std::acos(std::clamp(std::fabs(g.app.z()), 0.0, 1.0)) * 180.0 / M_PI;
    pending_depth_ = (top - g.tip.z()) * 1000.0;
    pending_lying_ = false;
    pending_empty_ = false;
    // 기울여 쥔 먼 퍽은 가까운 빈 자리에 세워 내려놓고 수직 손으로 다시 쥠 (regrasp_tilted, 지금 선 퍽 다시 쥐기와 같은 regrasp_nodes)
    //   — 11-10 10:05 실물: 표로 27° · 22 mm 로 쥔 퍽을 store_S7_d40_tilt30 (40 mm 가정) 로 대각 넣다 실패 → 박스 퍽처럼 50 mm 로 쥔 뒤 실물에서 된 칸 경로로
    std::string re_note;
    if (regrasp_ && pending_tilt_ > 0.5 && get_parameter("farpick_regrasp").as_bool()) {
      std::vector<PathNode> again;
      double sx = 0.0, sy = 0.0;
      std::string w;
      // 표로 쥔 기울임 (예: 27°) 은 10° 단계가 아님 → 놓는 손 기울임을 farpick_regrasp_tilt_dev (기본 5°) 안에서 (11-10 10:25 실물: 27° 에 맞는 놓기 없음 → 예전 흐름으로 감)
      if (!regrasp_nodes(x, y, top, pending_tilt_, pending_depth_, again, sx, sy, w, get_parameter("farpick_regrasp_tilt_dev").as_double())) {
        why = "먼 퍽 표: 기울여 쥔 퍽을 다시 쥘 자리가 없음 (" + w + ")";
        return {};
      }
      // 들어 올린 점 (far_lift, 멈춤) 뒤 · 기준점 (staging) 앞에 끼움
      auto at = path.nodes.end();
      if (!path.nodes.empty() && path.nodes.back().name == "staging") {at = path.nodes.end() - 1;}
      path.nodes.insert(at, again.begin(), again.end());
      char rb[120];
      std::snprintf(rb, sizeof(rb), " → (%.3f, %.3f) 에 세워 내려놓고 수직 손으로 다시 쥠", sx, sy);
      re_note = rb;
      pending_tilt_ = 0.0;
      pending_depth_ = -pick_heights_.back();
    }
    char nbs[32];
    if (has_nb) {std::snprintf(nbs, sizeof(nbs), "%.0f°", nb_dir);} else {std::snprintf(nbs, sizeof(nbs), "없음");}
    RCLCPP_INFO(get_logger(), "PICK 먼 퍽 표: 퍽 (%.3f, %.3f) %s 이웃 %s → 표 칸 (%.3f, %.3f, %.0f°) 차 %.1f mm · %.1f° · 닫는 점 %d · 쥔 깊이 %.0f mm · 기울임 %.0f° "
      "(%s, %zu 점 · 건너뜀 %d, %s에서 바로)", x, y, square ? "네모" : "원기둥", nbs, e.x, e.y, e.axis_deg, std::hypot(dx, dy) * 1000.0, da, e.release,
      pending_depth_, pending_tilt_, tname.c_str(), e.q.size(), skipped, from);
    if (!re_note.empty()) {RCLCPP_INFO(get_logger(), "PICK 먼 퍽 표:%s (쥔 깊이 %.0f mm 로)", re_note.c_str(), pending_depth_);}
    return visual_regrasp_actions(path, true);
  }

  std::vector<ArmAction> place_plan(const ArmTask::Goal & goal, std::string & why)
  {
    const std::string key = "retrieve_depth_fix." + held_retrieve_slot_;
    const double fix = !held_retrieve_slot_.empty() && has_parameter(key) ? get_parameter(key).as_double() : 0.0;
    if (fix == 0.0) {return place_plan_at_depth(goal, why);}
    const double depth = retrieve_place_depth_mm(held_depth_mm_, held_retrieve_slot_, fix);
    if (depth == held_depth_mm_) {return place_plan_at_depth(goal, why);}
    RCLCPP_INFO(get_logger(), "PLACE: %s 빼기 뒤 가정보다 얕게 %+.1f mm → 놓기용 깊이 %.1f mm (기존 %.1f mm)",
      held_retrieve_slot_.c_str(), fix, depth, held_depth_mm_);
    place_depth_override_mm_ = depth;
    auto plan = place_plan_at_depth(goal, why);
    place_depth_override_mm_.reset();
    if (plan.empty()) {
      RCLCPP_WARN(get_logger(), "PLACE: 칸별 깊이 보정 계획 안 됨 (%s) → 보정 없이 기존 높이로", why.c_str());
      why.clear();
      plan = place_plan_at_depth(goal, why);
    }
    return plan;
  }

  std::vector<ArmAction> place_plan_at_depth(const ArmTask::Goal & goal, std::string & why)
  {
    const auto fold = paths_.find("fold");
    if (!model_ok_ || fold == paths_.end() || staging_.size() != 5) {why = "팔 모델 · 접기 경로 · 기준점 설정 없음"; return {};}
    double x, y, top, yaw;
    bool lying = false;
    if (!target_in_model(goal, held_lying_, x, y, top, yaw, lying, why)) {return {};}
    if (lying && !held_lying_) {why = "누운 목표인데 쥔 퍽은 선 퍽 — 선 퍽은 세워서만 놓음"; return {};}
    // 놓는 점 보정 (place_offset_x/y_mm): 목표를 옮긴 뒤 빈 자리 검사 · IK — 먼 자리 다시 쥐기의 내려놓을 자리도 옮긴 목표 기준 (임의 빈 자리라 상관없음)
    place_off_mm_ = Eigen::Vector2d(get_parameter("place_offset_x_mm").as_double(), get_parameter("place_offset_y_mm").as_double());
    if (!place_off_mm_.allFinite()) {why = "place_offset_x_mm · y_mm 가 숫자가 아님"; return {};}
    x += place_off_mm_.x() / 1000.0;
    y += place_off_mm_.y() / 1000.0;
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
      // 누운 퍽은 수직 손으로만 놓음 → 그 자리가 수직 손 끝자리면 (실물: (0.250, −0.028) 위 40 mm IK 안 됨, 같은 자리 PICK 은 됨)
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
    // 쥔 퍽 손 안 위치 보정 (place_held_correct, STORE 의 held_correct 와 같음): 기준과 차이를 놓는 자세의 손목 방향으로 돌려 반대로 옮기고 같은 기울임으로 다시 풂
    if (get_parameter("place_held_correct").as_bool()) {
      std::string hid;
      Eigen::Vector3d hp, hn;
      int hc = 0;
      if (!held_now(hid, hp, hn, hc) || !held_ref_p_ || !held_ref_n_) {
        RCLCPP_INFO(get_logger(), "PLACE: 쥔 퍽 태그 %s (%d 장) → 손 안 위치 보정 없음", held_ref_p_ ? "못 봄" : "기준 없음", hc);
      } else {
        const Eigen::Vector3d dw = hp - *held_ref_p_;   // 손 안에서 퍽이 기준보다 움직인 양 (손목 좌표)
        const double face = std::acos(std::clamp(hn.dot(*held_ref_n_), -1.0, 1.0)) * 180.0 / M_PI;
        const Eigen::Vector3d dworld = arm_frames(model_, ticks_to_q(model_, P[2].ticks))[4].block<3, 3>(0, 0) * dw;   // 놓는 자세에서 퍽이 비키는 양
        const bool use = face < 20.0 && dw.norm() * 1000.0 <= held_max_mm_;
        RCLCPP_INFO(get_logger(), "PLACE: 쥔 퍽 태그 %s 기준과 차 손목 (%+.1f, %+.1f, %+.1f) mm · 면 %.0f° (%d 장) → 놓는 자리에서 (%+.1f, %+.1f) mm %s",
          hid.c_str(), dw.x() * 1000.0, dw.y() * 1000.0, dw.z() * 1000.0, face, hc, dworld.x() * 1000.0, dworld.y() * 1000.0,
          use ? "→ 반대로 옮김" : "(면이 다르거나 너무 커서 안 옮김)");
        if (use) {
          std::string w;
          auto Pc = poses_for(x - dworld.x(), y - dworld.y(), top, yaw, heights, cands, open_kw, w, P.back().tilt_deg, 0.0);
          if (Pc.size() == 3) {
            x -= dworld.x();
            y -= dworld.y();
            P = Pc;
          } else {
            RCLCPP_WARN(get_logger(), "PLACE: 손 안 위치 보정 자리는 IK 안 됨 (%s) → 보정 없이 놓음", w.c_str());
          }
        }
      }
    }
    RCLCPP_INFO(get_logger(), "PLACE: %s(%.3f, %.3f) 쥔 깊이 %.0f mm · 쥘 때 기울임 %.0f° · 놓는 손 %.0f° → 놓기 높이 %.1f mm · 보정 (%+.1f, %+.1f) mm (place_offset)",
      held_lying_ ? "누운 퍽 " : "", x, y, held_depth_mm_, held_tilt_, P.back().tilt_deg, heights.back(), place_off_mm_.x(), place_off_mm_.y());
    ArmPath path;
    path.name = "place";
    path.ends_home = true;
    path.nodes.push_back({"staging", staging_, false, 0.0, "", false});
    path.nodes.push_back(model_node("above", P[0].enc, slow_dps_));
    settle_at(path.nodes.back(), "place_settle_s");   // 놓을 자리 위에서 멈춤 (0 = 그대로)
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

  // 먼 자리 PLACE (먼 줄은 박스에 한 번 내려놨다가 쥐는 방식을 바꿔서, 멀리 있는 퍽 집듯이): 쥘 때 기울임 ± 20° 로 안 닿으면
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
    // 자리 탐색 시간 예산 (regrasp_search_s, regrasp_nodes 와 같음): 자리 시작 · IK 실패 뒤마다 검사, 성공해 끝난 자리는 그대로 받음
    const double budget_s = get_parameter("regrasp_search_s").as_double();
    const auto t_search = std::chrono::steady_clock::now();
    bool timed_out = false;
    auto over_budget = [&]() {
        if (budget_s <= 0.0 || std::chrono::duration<double>(std::chrono::steady_clock::now() - t_search).count() <= budget_s) {return false;}
        timed_out = true;
        last = "내려놓을 자리 찾기 시간 초과";
        return true;
      };
    // IK 씨앗: 목표 자세에서 (가까운 자리라 빠름). 그걸로 못 찾으면 기본 씨앗으로 한 번 더 (느림, 계획 수 초)
    const Q5 q_target = ticks_to_q(model_, Pt[2].ticks);
    for (int pass = 0; pass < 2 && mid.empty() && !timed_out; ++pass) {
      const std::vector<Q5> seed_t = pass == 0 ? std::vector<Q5>{q_target} : std::vector<Q5>{};
      for (double r = r0 - 0.03; mid.empty() && !timed_out && j1.x() + dir.x() * r >= regrasp_min_x_ - 0.05; r -= 0.01) {
        for (double lat : {0.0, 0.04, -0.04, 0.08, -0.08}) {
          if (timed_out || over_budget()) {break;}
          const Eigen::Vector2d q = j1 + dir * r + side * lat;
          if (q.x() < regrasp_min_x_ || std::fabs(q.y()) > regrasp_max_abs_y_ || (q - p0).norm() < 0.06) {continue;}   // 박스 밖 (앞 · 옆) · 목표와 가까움
          std::vector<std::pair<double, std::string>> cs;
          double gx = q.x(), gy = q.y();
          if (!grasp_for(gx, gy, zc, std::nan(""), true, cs, w, "먼 자리 놓기: 내려놓을 자리")) {last = w; continue;}   // 빈 자리 · 손가락 여유
          const double c_deg = (std::atan2(q.y() - j1.y(), q.x() - j1.x()) + M_PI / 2.0) * 180.0 / M_PI;
          std::string kw;
          // ② t° 로 다시 쥐기 (먼저 — 40° 손은 닿는 곳이 좁아 여기서 많이 걸러짐)
          const auto Pr = poses_for(q.x(), q.y(), top, c_deg, {pick_heights_[0], pick_heights_[1], -tilted_depth_mm_}, {{c_deg, "open"}}, kw, w, t, 0.5,
            seed_t);
          if (Pr.size() != 3) {last = std::to_string(static_cast<int>(t)) + "° 로 다시 쥐기: " + w; over_budget(); continue;}
          // ① 내려놓기: 쥘 때 기울임 ± 20°, 다른 기울임이면 그 기울임으로 놓기 높이를 다시 (place_plan 과 같음)
          const std::vector<Q5> seed_p = pass == 0 ? std::vector<Q5>{ticks_to_q(model_, Pr[2].ticks)} : std::vector<Q5>{};
          std::vector<double> hp{place_heights_[0], place_heights_[1], drop_height_mm(held_tilt_)};
          auto Pp = poses_for(q.x(), q.y(), top, c_deg, hp, {{c_deg, "release"}}, kw, w, held_tilt_, 20.0, seed_p);
          if (Pp.size() == 3 && std::abs(Pp.back().tilt_deg - held_tilt_) > 0.5) {
            hp[2] = drop_height_mm(Pp.back().tilt_deg);
            Pp = poses_for(q.x(), q.y(), top, c_deg, hp, {{c_deg, "release"}}, kw, w, Pp.back().tilt_deg, 0.5, seed_p);
          }
          if (Pp.size() != 3) {last = "내려놓기: " + w; over_budget(); continue;}
          mid = {model_node("far_put_above", Pp[0].enc, slow_dps_), model_node("far_put_down", Pp[1].enc, slow_dps_),
            model_node("far_put", Pp[2].enc, slow_dps_, "release"), model_node("far_put_up", Pp[1].enc, slow_dps_),
            model_node("far_put_clear", Pp[0].enc, slow_dps_), model_node("far_re_above", Pr[0].enc, slow_dps_, "open"),
            model_node("far_re_down", Pr[1].enc, slow_dps_), model_node("far_re_grasp", Pr[2].enc, slow_dps_, "close"),
            model_node("far_re_up", Pr[1].enc, slow_dps_), model_node("far_re_lift", Pr[0].enc, slow_dps_)};
          mid[5].grip_early = get_parameter("place_far_open_on_way").as_bool();   // 내려놓은 뒤 빈손 → 다시 쥘 자리 위로 가며 엶 (그리퍼 기다림 하나를 뺌)
          settle_at(mid[0], "place_settle_s");   // 내려놓을 자리 위 · 다시 쥘 퍽 위에서 멈춤 (0 = 그대로)
          settle_at(mid[5], "pick_settle_s");
          sx = q.x();
          sy = q.y();
          put_tilt = Pp.back().tilt_deg;
          break;
        }
      }
    }
    if (mid.empty()) {why = "먼 자리 놓기: 손 " + std::to_string(static_cast<int>(t)) + "° 로 다시 쥘 빈 자리가 없음 (마지막: " + last + ")"; return {};}
    RCLCPP_INFO(get_logger(), "PLACE 먼 자리 (%.3f, %.3f): 손 %.0f° 로 놓아야 함 → (%.3f, %.3f) 에 손 %.0f° 로 내려놓고 %.0f° 로 다시 쥠 (깊이 %.0f mm) → 목표에 %.0f° 로 놓음 "
      "· 보정 (%+.1f, %+.1f) mm (place_offset) (계산 목표 %.2f s · 자리 %.2f s)", x, y, t, sx, sy, put_tilt, t, tilted_depth_mm_, t, place_off_mm_.x(),
      place_off_mm_.y(), t_target, std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() - t_target);
    ArmPath path;
    path.name = "place_far";
    path.ends_home = true;
    path.nodes.push_back({"staging", staging_, false, 0.0, "", false});
    path.nodes.insert(path.nodes.end(), mid.begin(), mid.end());
    join_from_pick_end(path, "PLACE");
    path.nodes.push_back(model_node("above", Pt[0].enc, slow_dps_));
    settle_at(path.nodes.back(), "place_settle_s");   // 놓을 자리 위에서 멈춤 (0 = 그대로)
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
    const double H = (held_lying_ ? lying_h_ : puck_h_) * 1000.0;
    return place_drop_height_mm(place_clear_mm_, H, place_depth_override_mm_.value_or(held_depth_mm_), place_tilt, held_tilt_);
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

  // 받침 구간 점 이름: 나갈 때 받침 · up1~up8 (곧게 올라옴), 들어올 때 P0_복귀 → rJ1a · rJ2_1~4 (옆으로 비켜 올라감) → rJ1b · rJ4
  //   → r1~r7_복귀 · 받침_복귀 (곧게 내려감). 들어올 때 구간 전체를 같이 옮겨 rJ2_4 → rJ1b 의 J1 돌림 (2.8°) 이 그대로 · 받침 위에서만 바뀜
  static bool is_stand_node(const std::string & n)
  {
    static const std::set<std::string> names{"받침", "up1", "up2", "up3", "up4", "up5", "up6", "up7", "up8", "P0_복귀", "rJ1a_복귀",
      "rJ2_1_복귀", "rJ2_2_복귀", "rJ2_3_복귀", "rJ2_4_복귀", "rJ1b_복귀", "rJ4_복귀", "r1_복귀", "r2_복귀", "r3_복귀", "r4_복귀", "r5_복귀",
      "r6_복귀", "r7_복귀", "받침_복귀"};
    return names.count(n) > 0;
  }

  // HOME (받침에 접은 자세) 만 J1 · J5 를 옮김 (켤 때 한 번). home_j1_offset_rad: + = J1 눈금 + (11-10: 손으로 받침에 맞춘 J1 = 눈금 22
  //   = HOME 목표 4 + 16 + 늘 남는 2), home_j5_offset_rad: 손목. 둘 다 poses.home + 받침 구간 점 (is_stand_node) 에 같이 —
  //   J5 도 받침 위 높은 곳 (rJ1a · rJ2) 부터 돌아 받침 안에서 돌지 않음. 칸 경로 · 집기는 안 바뀜
  void apply_home_offsets()
  {
    const double j1 = get_parameter("home_j1_offset_rad").as_double(), j5 = get_parameter("home_j5_offset_rad").as_double();
    if (!std::isfinite(j1) || !std::isfinite(j5) || std::fabs(j1) > 0.1 || std::fabs(j5) > 0.2) {
      RCLCPP_WARN(get_logger(), "HOME 옮김 안 함: home_j1_offset_rad %.4f · home_j5_offset_rad %.4f (|J1| 0.1 · |J5| 0.2 rad 까지)", j1, j5);
      return;
    }
    if (j1 == 0.0 && j5 == 0.0) {return;}
    for (const auto & [name, i] : home_nodes_) {
      auto & q = paths_.at(name).nodes[i].q;
      q[0] += j1;
      q[4] += j5;
    }
    auto & h = poses_.at("home");
    h[0] += j1;
    h[4] += j5;
    RCLCPP_INFO(get_logger(), "HOME 옮김: J1 %+.4f rad (%+.0f 틱) · J5 %+.4f rad (%+.1f°) — 받침 구간 점 %zu 개 (이름으로)", j1,
      j1 * 4096.0 / (2.0 * M_PI), j5, j5 * 180.0 / M_PI, home_nodes_.size());
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
      if (fail_since_ >= 0.0) {   // 실패 뒤 자동 접기 기다리는 중 하드웨어 이상 → 접지 않음 (복구 뒤 사람이 HOME)
        fail_since_ = -1.0;
        RCLCPP_WARN(get_logger(), "실패 뒤 자동 접기 안 함: 하드웨어 이상 (복구 뒤 HOME)");
      }
    } else if (!hw_ok_before_ && have_hw_once_) {
      RCLCPP_WARN(get_logger(), "하드웨어 측정 돌아옴 — 바퀴는 잠금 유지, HOME 이 성공해야 DRIVE");
    }
    have_hw_once_ = have_hw_once_ || hw_ok;
    hw_ok_before_ = hw_ok;
    {   // 보드 상태 대신 관절 측정으로 보는 중인지 바뀔 때 한 번씩 (hw_status_optional)
      const bool waived = hw_->components_waived(t.seconds());
      if (waived && !hw_waived_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 30000, "보드 상태(controller_manager list_hardware_components) 못 받음 · 서비스 안 보임 — "
          "관절 측정 · 팔 · 그리퍼 재생기가 정상이라 하드웨어 정상으로 봄 (hw_status_optional)");
      } else if (!waived && hw_waived_) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 30000, "보드 상태 대신 보기 끝 (%s)", hw_->status(t.seconds()).c_str());
      }
      hw_waived_ = waived;
    }
    // HOME confirmation is a continuous drive permit. An out-of-range, missing
    // or stale arm measurement invalidates it even after a successful HOME.
    if (arbiter_.enforce_drive_guard(fold_->confirmed(t.seconds()), hw_ok)) {
      RCLCPP_ERROR(get_logger(), "주행 중 접힘 확인 상실: %s → STOP (HOME 재확인 필요)",
        fold_->status(t.seconds()).c_str());
      publish_turn();
    }
    // ① 차례: 측정 시각이 odom_timeout_s 안인 /odom 이 "멈춤" 이고, 바퀴 쪽이 이번 번호로 잠갔다고 답했을 때만
    bool odom_fresh = false, odom_still = false;
    {
      std::lock_guard<std::mutex> lk(odom_mtx_);
      odom_fresh = have_odom_ && std::fabs((t - odom_stamp_).seconds()) < odom_timeout_s_;
      odom_still = odom_still_;
    }
    if (arbiter_.turn() == Turn::STOPPING) {
      if (!odom_fresh) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "/odom 이 안 옴(또는 옛날 것) → 몸통이 멈췄는지 몰라서 팔을 시작하지 않음");
      } else if (lock_acked_id_ != arbiter_.request_id()) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000, "바퀴 잠금 확인 기다리는 중 (요청 %u, 받은 답 %u)",
          arbiter_.request_id(), lock_acked_id_);
      }
    }
    const Turn before = arbiter_.turn();
    arbiter_.update(t.seconds(), odom_fresh && odom_still, lock_acked_id_);
    // 준비 상태가 바뀌면 심장박동(10 Hz)을 기다리지 않고 바로 알림
    std::string why;
    const bool ready = task_ready(t.seconds(), why);
    if (arbiter_.turn() != before || ready != ready_sent_) {publish_turn();}

    // ② 팔
    run_arm(t);
  }

  // ======================================================================== grasp_guard (11-11): 쥔 폭 · 손 안 퍽 · 넣기 걸림 · 넣은 뒤
  //   판단은 grasp_guard.cpp (순수, test_grasp_guard.cpp). 이 묶음은 ROS 입출력과 동작 끼우기만. 설정은 작업마다 다시 읽음 (ros2 param set 바로)
  //   기본: 쥔 폭만 동작 (잘못 쥐면 그 자리에서 1 번 다시 쥠, 그래도 다르면 지금처럼 진행) · 손 안 퍽 · 넣기 걸림 · 넣은 뒤는 로그만
  void gg_declare_params()
  {
    declare_parameter<bool>("grasp_width_check", true);         // PICK 닫기 뒤 손가락 사이 ↔ 퍽 폭 (false = 안 봄)
    declare_parameter<double>("grasp_width_tol_mm", 5.0);       // |손가락 사이 − 퍽 폭| 이 이보다 크면 잘못 쥠
    declare_parameter<double>("grasp_gap_bias_mm", 7.4);        // 모델 간격 2·(9.1 + q) mm 에 더함 (실측: 네모 34.8 쥔 채 q 약 0.0046)
    declare_parameter<double>("grasp_width_cylinder_mm", 30.0);
    declare_parameter<double>("grasp_width_square_mm", 34.8);
    declare_parameter<int>("grasp_retry_max", 1);               // 잘못 쥐면 그 자리에서 다시 쥐는 횟수 (0 = 로그만)
    declare_parameter<bool>("held_regrasp", false);             // 손 안 퍽이 held_max_mm · held_regrasp_face_deg 넘으면 집은 자리에 내려놓고 다시 쥠
    declare_parameter<double>("held_regrasp_face_deg", 20.0);
    declare_parameter<bool>("store_contact_check", true);       // 넣기 걸림 감시 (로그만)
    declare_parameter<double>("store_contact_zone_mm", 35.0);   // 놓는 점 위 이 높이 안 = 마지막 내려가기 (컵 28 + 7)
    declare_parameter<double>("store_contact_err_deg", 1.5);
    declare_parameter<double>("store_contact_current_a", 0.30);
    declare_parameter<double>("store_contact_height_mm", 4.0);
    declare_parameter<bool>("store_contact_retry", false);      // 걸림 의심이면 들어 옮겨 다시 넣기 (false = 로그만)
    declare_parameter<int>("store_contact_retry_max", 2);
    declare_parameter<double>("store_contact_lift_mm", 15.0);
    declare_parameter<std::vector<double>>("store_contact_search_mm", std::vector<double>{2.0, 0.0, -2.0, 0.0, 0.0, 2.0, 0.0, -2.0});   // [x, y] 쌍 (base_link)
    // 넣기 걸림 보정 (11-11 오후 실물: 칸 안 ±2 mm 다시 넣기로는 왼쪽 테두리에 걸린 퍽이 안 들어감 · 손가락 사이로 밀려 올라감)
    //   걸림 의심이면 칸 안에서 놓지 않고 쥔 채 곧게 들어 (store_recover_lift_mm) 온 길로 박스 위까지 → 방금 쥐어 간 박스 자리에 내려놓고 열기 →
    //   다시 보기 (regrasp_visual) → 잰 중심으로 수직 다시 쥐기 → 같은 칸 STORE 를 처음부터 다시 계획해 넣음. store_recover_max 번 넘으면
    //   같은 식으로 박스에 내려놓고 (열고) P0 로 빼서 실패 (결과 글 STORE_RETURNED_TO_BOX x y). 켜면 빠른 다시 넣기 (store_contact_retry) 보다 먼저
    declare_parameter<bool>("store_recover", false);
    declare_parameter<int>("store_recover_max", 1);             // 한 STORE 에서 박스에서 다시 쥐어 넣기 횟수 (0 = 걸리면 바로 박스에 되돌리고 실패)
    declare_parameter<double>("store_recover_lift_mm", 40.0);   // 칸 안 놓는 점에서 곧게 최소 이만큼 (칸 안 곧은 구간이 더 길면 그대로)
    declare_parameter<bool>("store_recover_force", false);      // 시험용: 첫 넣기를 걸림으로 봄 (모의 · 흐름 확인. 다시 넣기는 실제 판단)
    declare_parameter<bool>("store_seat_check", true);          // 넣은 뒤 퍽 윗면 태그 높이 · 위치 (로그만)
    declare_parameter<double>("store_seat_floor_z", 0.1325);    // 칸 바닥 [m, base_link] = config/rack_cells.yaml floor_z
    declare_parameter<double>("store_seat_tol_mm", 6.0);
  }

  // 지금 PICK 계획의 닫기마다 다시 쥐기 정보 (set_grasp_checks 와 같은 순서, 수가 다르면 안 씀)
  void gg_set_regrip(const ArmPath & path)
  {
    gg_regrip_ = regrip_points(path, grip_);
    if (gg_regrip_.size() != grasp_checks_.size()) {gg_regrip_.clear();}
    gg_last_close_.reset();
  }

  // 닫기 다음 팔 동작 직전 (빈손 아님): 손가락 사이 ↔ 퍽 폭. 잘못 쥐었으면 그 자리에서 다시 쥐는 동작을 앞에 끼우고 action 을 그 첫 동작으로
  void gg_check_width(ArmAction & action, const std::string & gc_pose, const std::vector<double> & gc_up)
  {
    if (gg_regrip_.empty()) {return;}
    RegripPoint r = gg_regrip_.front();
    gg_regrip_.erase(gg_regrip_.begin());
    gg_last_close_ = r;
    if (!get_parameter("grasp_width_check").as_bool()) {return;}
    GripWidthConfig wc;
    wc.tol_mm = get_parameter("grasp_width_tol_mm").as_double();
    wc.gap_bias_mm = get_parameter("grasp_gap_bias_mm").as_double();
    wc.cylinder_mm = get_parameter("grasp_width_cylinder_mm").as_double();
    wc.square_mm = get_parameter("grasp_width_square_mm").as_double();
    const auto v = judge_grip_width(grip_now_, gg_shape_, wc);
    const char * nm = r.close_name.c_str();
    if (!v.judged || !v.bad) {RCLCPP_INFO(get_logger(), "쥔 폭 (%s): %s%s", nm, v.text.c_str(), v.judged ? " → 맞음" : ""); return;}
    const int max = static_cast<int>(get_parameter("grasp_retry_max").as_int());
    if (r.tries >= max) {
      RCLCPP_WARN(get_logger(), "쥔 폭 (%s): %s → 잘못 쥔 듯, 다시 쥐기 다 함 (grasp_retry_max %d) → 그대로 진행", nm, v.text.c_str(), max);
      return;
    }
    if (!r.retry_ok) {
      RCLCPP_WARN(get_logger(), "쥔 폭 (%s): %s → 잘못 쥔 듯, 이 닫기는 그 자리 다시 쥐기 안 됨 (다음 점 %s) → 그대로 진행", nm, v.text.c_str(),
        r.up_name.c_str());
      return;
    }
    const auto retry = regrip_in_place(r, timing_, grip_);
    if (retry.empty() || !runner_.insert_before_sent(retry)) {
      RCLCPP_WARN(get_logger(), "쥔 폭 (%s): %s → 잘못 쥔 듯, 다시 쥐기를 못 끼움 → 그대로 진행", nm, v.text.c_str());
      return;
    }
    ++r.tries;
    grasp_checks_.insert(grasp_checks_.begin(), GraspCheck{gc_pose, gc_up});   // 다시 닫은 뒤 같은 동작 앞에서 빈손 · 폭을 또 봄
    gg_regrip_.insert(gg_regrip_.begin(), r);
    action = retry.front();
    RCLCPP_WARN(get_logger(), "쥔 폭 (%s): %s → 잘못 쥔 듯 → 그 자리에서 놓고 (그리퍼 %.4f m) %s 까지 올렸다 다시 쥠 (%d/%d)", nm, v.text.c_str(),
      r.open_m, r.up_name.c_str(), r.tries, max);
  }

  // PICK · RETRIEVE 성공: 쥔 퍽 태그를 받을 ID · 시각 창 (쥔 뒤 ~ 놓기 전) 을 새로
  void gg_on_hold(const std::string & puck_id, bool pick)
  {
    gg_held_id_ = puck_id;
    gg_release_t_ = 1e18;
    gg_drop_.clear();
    gg_pick_close_ = pick ? gg_last_close_ : std::nullopt;
    gg_held_shape_ = pick ? gg_shape_ : PuckShape::Unknown;
    gg_pick_xy_.reset();   // store_recover: 이 퍽을 박스에서 쥔 자리 (마지막 닫기 손끝, 처짐 반영)
    if (pick && gg_last_close_) {gg_pick_xy_ = command_tip_xy(gg_last_close_->q_close);}
  }

  // 명령 자세 (팀 rad) → 실제 손끝 xy (base_link): 처짐 반영 (apply_held_fresh 와 같은 sag_corrected). 못 세면 없음
  std::optional<Eigen::Vector2d> command_tip_xy(const std::vector<double> & q) const
  {
    if (!model_ok_ || q.size() != 5) {return std::nullopt;}
    std::array<double, 5> command{};
    std::copy_n(q.begin(), 5, command.begin());
    const auto actual = sag_corrected(model_, command, 0.0, 1.0);
    Ticks ticks{};
    for (int j = 0; j < 5; ++j) {
      if (!std::isfinite(actual[j])) {return std::nullopt;}
      ticks[j] = static_cast<int>(std::lround(model_.zero_tick[j] + actual[j] * 4096.0 / (2.0 * M_PI)));
    }
    const Eigen::Vector3d tip = arm_grip(model_, ticks_to_q(model_, ticks)).tip;
    if (!tip.allFinite()) {return std::nullopt;}
    return Eigen::Vector2d(tip.x(), tip.y());
  }

  // 쥔 퍽 태그 표본 하나를 held_buf_ 에 넣을지 (지난 PICK 의 목표 퍽 ID · 쥔 뒤 ~ 놓기 전만). 버리면 이유별로 셈 (STORE 때 한 번 로그)
  bool gg_held_accept(const std::string & id, const std::string & frame, double stamp)
  {
    // ID 없이 집은 PICK · RETRIEVE (robot_cli pick det · pick tag 는 퍽 ID 를 안 보냄): 쥐고 기준점에 멈춘 뒤 처음 본 손 안 태그 ID 로 고정
    //   → 그 뒤 칸 이웃 퍽 태그는 '다른 퍽' 으로 버림 (11-11 실물: ID 없어 손 안 태그 8 장을 다 버려 held_correct 가 안 됨)
    if (gg_held_id_.empty() && !id.empty() && frame == "wrist_link" && std::isfinite(stamp) && stamp >= held_since_ &&
      stamp <= gg_release_t_)
    {
      gg_held_id_ = id;
      RCLCPP_INFO(get_logger(), "손 안 퍽: 목표 ID 없음 → 쥔 뒤 처음 본 태그 %s 로 고정", id.c_str());
    }
    if (const char * why = held_sample_reject(id, frame, stamp, gg_held_id_, held_since_, gg_release_t_)) {++gg_drop_[why]; return false;}
    return true;
  }

  // LOOK_RACK · STORE 계획 때: 손 안 퍽 한 줄. held_regrasp 면 (집은 자리에서 바로일 때만) 들기를 되돌려 내려놓고 다시 쥐는 동작을 앞에 붙임
  void gg_held_before_rack(const ArmTask::Goal & goal, std::vector<ArmAction> & plan, bool store)
  {
    std::string hid;
    Eigen::Vector3d hp = Eigen::Vector3d::Zero(), hn = Eigen::Vector3d::UnitZ();
    int hc = 0;
    const bool seen = model_ok_ && held_now(hid, hp, hn, hc);
    const double max_mm = get_parameter("held_max_mm").as_double(), max_face = get_parameter("held_regrasp_face_deg").as_double();
    const auto v = judge_held_in_hand(seen, hp, hn, hc, model_.tip5, model_.app5, model_.close5, max_mm, max_face);
    std::string drops;   // 버린 태그 이유는 STORE 마다 한 번만
    for (const auto & [why, n] : gg_drop_) {
      if (store) {drops += (drops.empty() ? " · 버린 태그: " : ", ") + why + " " + std::to_string(n) + " 장";}
    }
    RCLCPP_INFO(get_logger(), "손 안 퍽 (%s %s, 퍽 %s): %s%s%s", store ? "STORE" : "LOOK_RACK", goal.slot.c_str(),
      gg_held_id_.empty() ? "ID 없음" : gg_held_id_.c_str(), v.text.c_str(), v.seen ? (v.bad ? " → 어긋남" : " → 맞음") : "", drops.c_str());
    if (store) {gg_drop_.clear();}   // 이유 로그는 STORE 마다 한 번
    if (plan.empty() || !v.seen || !v.bad || !get_parameter("held_regrasp").as_bool()) {return;}
    std::string why;
    if (!held_from_pick_ || !gg_pick_close_ || !put_back_ok(*gg_pick_close_)) {
      RCLCPP_INFO(get_logger(), "held_regrasp: 집은 자리 (들어 올린 자세) 에서 바로가 아님 → 다시 쥐기 안 함");
      return;
    }
    ArmPath at_lift{"held_lift", {gg_pick_close_->after.back()}, false};
    if (!path_start_ok(at_lift, joint_buf_, path_start_tol_, why)) {
      RCLCPP_INFO(get_logger(), "held_regrasp: 팔이 집은 자리 들기 자세가 아님 (%s) → 다시 쥐기 안 함", why.c_str());
      return;
    }
    const auto pre = put_back_regrip(*gg_pick_close_, timing_, grip_);
    if (pre.empty()) {return;}
    plan.insert(plan.begin(), pre.begin(), pre.end());
    grasp_checks_.insert(grasp_checks_.begin(), GraspCheck{pre.back().pose, gg_pick_close_->q_up});   // 다시 쥔 뒤 빈손 확인 (빈손이면 위 점에서 멈춤)
    RegripPoint r = *gg_pick_close_;
    r.close_name = "held_regrip";
    r.tries = 1 << 20;   // 폭은 보되 그 자리 다시 쥐기는 또 안 함
    gg_regrip_.insert(gg_regrip_.begin(), r);
    gg_pick_close_.reset();
    RCLCPP_WARN(get_logger(), "held_regrasp: 손 안 퍽이 어긋남 → 집은 자리에 내려놓고 다시 쥔 뒤 %s (동작 %zu 개 더)", store ? "STORE" : "LOOK_RACK",
      pre.size());
  }

  // /joint_states → 넣기 감시 (켜져 있을 때만)
  void gg_feed_joints(const sensor_msgs::msg::JointState & m)
  {
    if (!gg_contact_.active()) {return;}
    std::array<double, 5> q, cur;
    q.fill(std::nan(""));
    cur.fill(std::nan(""));
    for (std::size_t i = 0; i < m.name.size() && i < m.position.size(); ++i) {
      for (std::size_t k = 0; k < arm_joints_.size() && k < 5; ++k) {
        if (m.name[i] != arm_joints_[k]) {continue;}
        q[k] = m.position[i];
        cur[k] = i < m.effort.size() ? m.effort[i] : std::nan("");
      }
    }
    gg_contact_.feed(rclcpp::Time(m.header.stamp).seconds(), q, cur);
  }

  // 동작을 보내기 직전 (run_arm SEND): 넣기 감시 판단 · 다시 넣기 · 놓은 시각 · 넣은 뒤 확인 시작
  void gg_on_send(ArmAction & action)
  {
    const double now_s = now().seconds();
    const bool store = arm_goal_ && arm_goal_->get_goal()->task == ArmTask::Goal::STORE;
    const bool place = arm_goal_ && arm_goal_->get_goal()->task == ArmTask::Goal::PLACE;
    const std::string slot = arm_goal_ ? arm_goal_->get_goal()->slot : "";
    if (gg_contact_.active()) {   // 앞 동작 = 칸 안 놓는 점까지 → 판단
      ContactConfig cc;
      cc.err_deg = get_parameter("store_contact_err_deg").as_double();
      cc.current_a = get_parameter("store_contact_current_a").as_double();
      cc.height_mm = get_parameter("store_contact_height_mm").as_double();
      const auto v = gg_contact_.judge(cc, model_ok_ ? &model_ : nullptr);
      const int att = gg_contact_.attempt();
      gg_contact_.stop();
      const bool retry_on = get_parameter("store_contact_retry").as_bool();
      const int retry_max = static_cast<int>(get_parameter("store_contact_retry_max").as_int());
      std::string tag = att > 0 ? " (다시 넣기 " + std::to_string(att) + ")" : "";
      if (store_recover_n_ > 0) {tag += " (박스에서 다시 쥔 뒤 " + std::to_string(store_recover_n_) + ")";}
      const bool recover_on = store && get_parameter("store_recover").as_bool();
      const bool forced = recover_on && att == 0 && store_recover_n_ == 0 && get_parameter("store_recover_force").as_bool();
      if (!v.suspect && !forced) {
        RCLCPP_INFO(get_logger(), "넣기 감시 %s%s: %s%s", slot.c_str(), tag.c_str(), v.text.c_str(), v.judged ? " → 정상" : "");
      } else {
        RCLCPP_WARN(get_logger(), "넣기 걸림 의심 %s%s: %s%s → %s", slot.c_str(), tag.c_str(), v.text.c_str(),
          forced ? " [store_recover_force 시험: 걸림으로 봄]" : "",
          recover_on ? "store_recover (칸 안에서 놓지 않음)" :
          (!retry_on ? "로그만 (store_contact_retry false), 그대로 놓음" : (att >= retry_max ? "다시 넣기 다 함 → 그대로 놓음" : "다시 넣기")));
      }
      if ((v.suspect || forced) && recover_on && action.is_gripper && action.phase == "release" && store_recover_start(action, slot)) {return;}
      if (v.suspect && retry_on && att < retry_max && action.is_gripper && action.phase == "release" && model_ok_ && gg_contact_end_.size() == 5) {
        const auto s = get_parameter("store_contact_search_mm").as_double_array();
        const std::size_t k = static_cast<std::size_t>(att) * 2;
        const Eigen::Vector2d d = k + 1 < s.size() ? Eigen::Vector2d(s[k] / 1000.0, s[k + 1] / 1000.0) : Eigen::Vector2d(0.0, 0.0);
        const double lift = get_parameter("store_contact_lift_mm").as_double() / 1000.0;
        std::string why;
        const ArmAction r = contact_retry_action(model_, gg_contact_end_, lift, d, slow_dps_, timing_, why);
        if (r.points.empty()) {
          RCLCPP_WARN(get_logger(), "다시 넣기 못 함 (%s) → 그대로 놓음", why.c_str());
        } else if (runner_.insert_before_sent({r})) {
          action = r;
          gg_contact_.start(r.points, now_s, descent_start_time(model_, r.points, std::max(0.0, lift - 0.003)), att + 1);
          RCLCPP_WARN(get_logger(), "다시 넣기 %d/%d: 놓는 점에서 곧게 %.0f mm 올려 (%+.1f, %+.1f) mm 옮겨 다시 내림", att + 1, retry_max, lift * 1000.0,
            d.x() * 1000.0, d.y() * 1000.0);
          return;
        }
      }
    }
    // store_recover 보정 동작 (박스 쪽): 박스 위 내려놓기 · 열기는 넣기 감시 · 놓은 시각 · 넣은 뒤 확인이 아님. 박스에 놓은 자리만 기억
    if (store && store_recovering_) {
      if (action.is_gripper && action.phase == "release") {
        if (!store_box_xy_) {RCLCPP_INFO(get_logger(), "STORE 보정 %s: 박스 (%.3f, %.3f) 에 내려놓음 (열기)", slot.c_str(), store_put_xy_.x(), store_put_xy_.y());}
        store_box_xy_ = store_put_xy_;
      } else if (action.is_gripper && action.phase == "grasp") {
        store_box_xy_.reset();   // 다시 쥠 (빈손이면 빈손 확인이 박스 자리를 되살림)
      }
      return;
    }
    if (store && !action.is_gripper && action.phase == "release" && action.points.size() >= 2 &&
      (get_parameter("store_contact_check").as_bool() || get_parameter("store_recover").as_bool()))
    {
      const double zone = get_parameter("store_contact_zone_mm").as_double() / 1000.0;
      const double t_desc = model_ok_ ? descent_start_time(model_, action.points, zone) : std::max(0.0, action.duration_s - 0.8);
      gg_contact_end_ = action.points.back().q;
      gg_contact_.start(action.points, now_s, t_desc, 0);
    }
    if ((store || place) && action.is_gripper && action.phase == "release" && gg_release_t_ > 1e17) {
      gg_release_t_ = now_s;   // 이 뒤 쥔 퍽 태그는 손 안 퍽으로 안 씀
      if (store) {gg_seat_start(slot, now_s + gripper_time_s_);}
    }
  }

  // ------------------------------------------------------------------------ store_recover (넣기 걸림 보정, 11-11)
  void store_recover_reset()
  {
    store_path_ = ArmPath{};
    store_recover_n_ = 0;
    store_recovering_ = false;
    store_return_end_ = false;
    store_box_xy_.reset();
    store_put_xy_ = Eigen::Vector2d::Zero();
    store_regrip_.clear();
    store_recover_note_.clear();
  }

  // 넣기 걸림 의심 + 칸 안 놓기 (그리퍼 release) 직전: 남은 계획을 보정 동작으로 바꾸고 action 을 그 첫 동작으로. 못 하면 false (지금처럼 놓음)
  //   store_recover_n_ < store_recover_max: 들어 나옴 → 박스에 내려놓기 → (다시 보기) → 다시 쥐기 → 칸 경로 다시 계획 (kStoreReplan)
  //   넘으면: 들어 나옴 → 박스에 내려놓기 → 열고 닫으며 P0 (PLACE 끝과 같음) → 실패 + STORE_RETURNED_TO_BOX x y
  bool store_recover_start(ArmAction & action, const std::string & slot)
  {
    const int max = std::max(0, static_cast<int>(get_parameter("store_recover_max").as_int()));
    const bool again = store_recover_n_ < max;
    auto refuse = [&](const std::string & w) {
        RCLCPP_WARN(get_logger(), "STORE 보정 %s: 못 함 (%s) → 지금처럼 칸 안에서 놓음", slot.c_str(), w.c_str());
        return false;
      };
    const auto fold = paths_.find("fold");
    if (!model_ok_ || fold == paths_.end()) {return refuse("팔 모델 · 접기 경로 없음");}
    if (store_path_.name.rfind("store_look_", 0) == 0) {return refuse("관찰 자세에서 온 칸 경로 (store_look_) — 온 길 끝이 박스 위가 아님");}
    const std::size_t r = store_release_index(store_path_.nodes);
    if (r == 0 || r >= store_path_.nodes.size()) {return refuse("칸 경로 " + store_path_.name + " 에 놓는 점 없음");}
    double dq = gg_contact_end_.size() == 5 && store_path_.nodes[r].q.size() == 5 ? 0.0 : 1e9;
    for (std::size_t j = 0; j < 5 && dq < 1e9; ++j) {dq = std::max(dq, std::fabs(store_path_.nodes[r].q[j] - gg_contact_end_[j]));}
    if (dq > 1e-6) {return refuse("칸 경로 놓는 점이 방금 넣기 끝 자세와 다름");}
    if (!gg_pick_xy_) {return refuse("이 퍽을 박스에서 쥔 자리를 모름 (PICK 뒤 STORE 가 아님)");}
    std::string why, note;
    const double lift = get_parameter("store_recover_lift_mm").as_double() / 1000.0;
    auto out = store_recover_out_nodes(model_, store_path_.nodes, r, std::isfinite(lift) ? lift : 0.0, why, note);
    if (out.empty()) {return refuse("들어 나오기: " + why);}
    if (!note.empty()) {RCLCPP_INFO(get_logger(), "STORE 보정 %s: 들어 나오기 — %s", slot.c_str(), note.c_str());}
    // 박스 자리: 방금 쥐어 간 자리 먼저 (regrasp_nodes origin_first), 놓기 · 다시 쥐기는 PICK 의 선 퍽 다시 쥐기 그대로
    // 박스 위 선 퍽 윗면 (모델 z) + pick_top_bias_mm: 박스 PICK 과 같은 실제 깊이로 내려놓고 다시 쥠
    //   (11-11 16:3x 실물: 보정 없이 내려놓기 · 다시 쥐기가 PICK 보다 10 mm 깊음 — park_box_regrip 과 같은 문제)
    const double tb = get_parameter("pick_top_bias_mm").as_double();
    const double top = surface_z_ + puck_h_ + model_.team_base_z_floor - model_.base_z_floor +
      (std::isfinite(tb) && std::fabs(tb) <= 30.0 ? tb / 1000.0 : 0.0);
    const double depth = held_depth_mm_ > 0.0 ? held_depth_mm_ : -pick_heights_.back();
    std::vector<PathNode> box;
    double sx = 0.0, sy = 0.0;
    exclude_.reset();
    if (!regrasp_nodes(gg_pick_xy_->x(), gg_pick_xy_->y(), top, held_tilt_, depth, box, sx, sy, why, 0.5, true)) {return refuse("박스 자리: " + why);}
    ArmPath path{"store_recover", out, false};
    std::vector<ArmAction> plan;
    if (again) {
      path.nodes.insert(path.nodes.end(), box.begin(), box.end());
      plan = visual_regrasp_actions(path, true);   // 다시 쥔 닫기 빈손 확인 · 다시 보기 대기 (regrasp_visual)
      ArmAction mark{"approach", false, kStoreReplan, 0.0};
      plan.push_back(mark);
      visual_suffix_ = {mark};   // 다시 보고 쥐기가 후반을 바꿔도 끝에 다시 계획
      visual_id_ = gg_held_id_;
      store_keep_regrip(path);
    } else {
      // 내려놓기만: re_place_above … re_place_clear (거기서 닫으며) → P0 · 접기 (PLACE 끝과 같음)
      const auto clear = std::find_if(box.begin(), box.end(), [](const PathNode & n) {return n.name == "re_place_clear";});
      if (clear == box.end()) {return refuse("내려놓기 점 없음");}
      path.nodes.insert(path.nodes.end(), box.begin(), clear + 1);
      path.nodes.back().grip = "close";
      path.nodes.back().grip_on_way = side_grip_;
      for (const auto & n : fold->second.nodes) {path.nodes.push_back(n);}
      path.ends_home = true;
      cut_fold_tail(path);
      visual_obs_q_.clear();
      plan = path_actions(path, timing_, grip_, joint_buf_);
    }
    if (plan.empty()) {visual_suffix_.clear(); return refuse("보정 동작이 비어 있음");}
    if (!runner_.replace_from_sent(plan)) {visual_suffix_.clear(); return refuse("작업 상태가 바뀜");}
    action = plan.front();
    store_recovering_ = true;
    store_return_end_ = !again;
    store_put_xy_ = Eigen::Vector2d(sx, sy);
    if (again) {++store_recover_n_;}
    std::string route;   // 앞 3 점 … 끝 2 점 (기록 경로는 점이 수십 개)
    for (std::size_t i = 1; i < out.size(); ++i) {
      if (i <= 3 || i + 2 >= out.size()) {route += (route.empty() ? "" : " → ") + out[i].name;}
      else if (i == 4) {route += " → … (" + std::to_string(out.size() - 6) + " 점)";}
    }
    RCLCPP_WARN(get_logger(), "STORE 보정 %s: 칸 안에서 안 놓음 → 쥔 채 들어 나옴 (%s) → 박스 (%.3f, %.3f) 에 내려놓기 (쥐어 간 자리 (%.3f, %.3f)) → %s",
      slot.c_str(), route.c_str(), sx, sy, gg_pick_xy_->x(), gg_pick_xy_->y(),
      again ? ("다시 보고 다시 쥐어 다시 넣기 (" + std::to_string(store_recover_n_) + "/" + std::to_string(max) + ")").c_str() :
      ("열고 P0 로 빼서 실패 (보정 " + std::to_string(store_recover_n_) + "/" + std::to_string(max) + " 다 함)").c_str());
    return true;
  }

  // 다시 쥔 점 (re_up · re_grasp · re_lift) 기억 — 다시 넣기 계획이 안 되면 그 자리에 되돌려 놓음
  void store_keep_regrip(const ArmPath & path)
  {
    store_regrip_.clear();
    for (const char * nm : {"re_up", "re_grasp", "re_lift"}) {
      const auto it = std::find_if(path.nodes.begin(), path.nodes.end(), [nm](const PathNode & n) {return n.name == nm;});
      if (it == path.nodes.end()) {store_regrip_.clear(); return;}
      store_regrip_.push_back(*it);
    }
  }

  // 보정 다시 쥐기 뒤 (kStoreReplan 논리 대기, run_arm SEND): 같은 칸 STORE 를 처음부터 다시 계획 (칸 보정 · held_correct · 곧게 올리기 · 늦게 닫기 그대로)
  //   다시 쥔 퍽 = 수직 손 · PICK 쥐기 깊이 · 박스 위 다시 쥔 자리 (PICK 끝과 같음 → 칸 경로 첫 점 대신 지금 자세). 손 안 태그 · 넣은 시각은 처음부터
  //   계획이 안 되면 다시 쥔 자리에 되돌려 놓고 P0 로 빼서 실패 (STORE_RETURNED_TO_BOX)
  void store_replan(unsigned gen, double now_s)
  {
    runner_.on_goal_response(gen, true, now_s);
    if (!arm_goal_) {runner_.fail_now("STORE 보정: 작업 없음"); return;}
    ArmTask::Goal goal = *arm_goal_->get_goal();
    goal.target = geometry_msgs::msg::PoseStamped{};   // 관찰 자세가 아님 → 잰 칸 없음 (빈손 스캔 맞춤 · 칸 표)
    store_recovering_ = false;
    store_box_xy_.reset();
    held_tilt_ = 0.0;
    held_depth_mm_ = -pick_heights_.back();
    depth_from_pick_ = true;
    held_from_pick_ = true;
    arm_off_path_ = false;
    look_rack_slot_.clear();
    held_buf_.clear();
    held_fresh_buf_.clear();
    held_since_ = now_s + 0.3;
    gg_release_t_ = 1e18;
    gg_drop_.clear();
    gg_pick_close_.reset();   // held_regrasp (집은 자리 되돌리기) 안 함
    if (!store_regrip_.empty()) {
      if (const auto xy = command_tip_xy(store_regrip_[1].q)) {gg_pick_xy_ = *xy;}   // 다음 보정은 다시 쥔 자리에서
    }
    std::string why;
    auto plan = store_plan(goal, why);
    if (!plan.empty()) {gg_held_before_rack(goal, plan, true);}
    if (!plan.empty() && runner_.replace_after_wait(gen, plan)) {
      RCLCPP_INFO(get_logger(), "STORE 보정 %s: 다시 쥠 → 칸 경로 다시 계획 (%s, 동작 %zu 개) → 다시 넣기", goal.slot.c_str(), store_path_.name.c_str(),
        plan.size());
      return;
    }
    RCLCPP_ERROR(get_logger(), "STORE 보정 %s: 다시 넣기 계획 안 됨 (%s) → 다시 쥔 자리에 되돌려 놓고 실패", goal.slot.c_str(), why.c_str());
    const auto fold = paths_.find("fold");
    if (store_regrip_.size() != 3 || fold == paths_.end() || joint_buf_.size() != 5) {
      runner_.fail_now("STORE 보정: 다시 넣기 계획 안 됨 (" + why + "), 퍽 쥔 채 박스 위");
      return;
    }
    ArmPath back{"store_recover_back", {}, true};
    back.nodes.push_back({"re_lift", joint_buf_, false, 0.0, "", false});
    back.nodes.push_back(store_regrip_[0]);
    back.nodes.push_back(store_regrip_[1]);
    back.nodes.back().grip = "release";
    back.nodes.push_back(store_regrip_[0]);
    back.nodes.push_back(store_regrip_[2]);
    back.nodes.back().grip = "close";
    back.nodes.back().grip_on_way = side_grip_;
    for (const auto & n : fold->second.nodes) {back.nodes.push_back(n);}
    cut_fold_tail(back);
    const auto put = path_actions(back, timing_, grip_, joint_buf_);
    if (put.empty() || !runner_.replace_after_wait(gen, put)) {
      runner_.fail_now("STORE 보정: 다시 넣기 계획 안 됨 (" + why + "), 퍽 쥔 채 박스 위");
      return;
    }
    store_recovering_ = true;
    store_return_end_ = true;
    if (const auto xy = command_tip_xy(store_regrip_[1].q)) {store_put_xy_ = *xy;}
    store_recover_note_ = "다시 넣기 계획 안 됨: " + why;
  }

  // STORE 놓기 때: 놓은 퍽 윗면 태그를 모을 칸 · 퍽 (칸 = 기준 스캔 (rack_reference) 또는 칸 표 + 적재함 움직임)
  void gg_seat_start(const std::string & slot, double after)
  {
    gg_seat_ = GgSeat{};
    if (!get_parameter("store_seat_check").as_bool()) {return;}
    std::size_t i = 0;
    while (i < rack_names_.size() && rack_names_[i] != slot) {++i;}
    if (i >= rack_names_.size()) {return;}
    const auto r = rack_ref_.find(slot);
    gg_seat_.cell = r != rack_ref_.end() ? r->second : rack_xy_[i];
    if (rack_fit_ok_) {gg_seat_.cell += rack_shift(rack_fit_, rack_xy_[i], rack_pivot_);}
    gg_seat_.on = true;
    gg_seat_.slot = slot;
    gg_seat_.id = gg_held_id_;
    gg_seat_.after = after;
    gg_seat_.puck_w = gg_held_shape_ == PuckShape::Square ? 0.0348 : 0.030;
  }

  bool gg_seat_near(const Eigen::Vector2d & xy) const {return (xy - gg_seat_.cell).norm() < 0.025;}

  // /held_puck/tag (손목 카메라 가까이, PnP, wrist_link) 를 놓은 뒤 base_link 로 → 윗면 높이까지
  void gg_seat_feed_held(const interfaces::msg::DetectionArray & m)
  {
    if (!gg_seat_.on) {return;}
    const double t = rclcpp::Time(m.header.stamp, get_clock()->get_clock_type()).seconds();
    if (t < gg_seat_.after) {return;}
    for (const auto & d : m.detections) {
      if (!gg_seat_.id.empty() && d.id != gg_seat_.id) {continue;}
      geometry_msgs::msg::PointStamped in, out;
      in.header = d.header.frame_id.empty() ? m.header : d.header;
      in.point = d.pose.position;
      try {
        out = tf_buf_->transform(in, "base_link", tf2::durationFromSec(0.0));
      } catch (const tf2::TransformException &) {
        continue;
      }
      const Eigen::Vector3d p(out.point.x, out.point.y, out.point.z);
      if (p.allFinite() && gg_seat_near(p.head<2>()) && gg_seat_.s.size() < 200) {gg_seat_.s.push_back({t, p, true});}
    }
  }

  // /rack_tags/detections 의 칸 퍽 (윗면 = 바닥 + 60 mm 로 가정해 잰 xy) — 높이는 모름
  void gg_seat_feed_rack(const interfaces::msg::DetectionArray & m)
  {
    if (!gg_seat_.on) {return;}
    const double t = rclcpp::Time(m.header.stamp, get_clock()->get_clock_type()).seconds();
    if (t < gg_seat_.after) {return;}
    for (const auto & d : m.detections) {
      if (d.kind != "rack_puck" || d.label != gg_seat_.slot || (!gg_seat_.id.empty() && d.id != gg_seat_.id)) {continue;}
      const Eigen::Vector3d p(d.pose.position.x, d.pose.position.y, d.pose.position.z);
      if (p.allFinite() && gg_seat_near(p.head<2>()) && gg_seat_.s.size() < 200) {gg_seat_.s.push_back({t, p, false});}
    }
  }

  // 팔 작업 끝 (FINISH): 넣기 감시 끝 · STORE 면 넣은 뒤 한 줄
  void gg_on_finish(uint8_t task)
  {
    gg_contact_.stop();
    if (task == ArmTask::Goal::STORE && gg_seat_.on) {
      SeatConfig sc;
      sc.floor_z = get_parameter("store_seat_floor_z").as_double();
      sc.tol_mm = get_parameter("store_seat_tol_mm").as_double();
      const auto v = judge_seat(gg_seat_.s, gg_seat_.cell, gg_seat_.puck_w, sc);
      if (v.state == SeatState::NOT_SEATED) {
        RCLCPP_WARN(get_logger(), "넣은 뒤 %s (퍽 %s): %s → %s (로그만)", gg_seat_.slot.c_str(), gg_seat_.id.c_str(), v.text.c_str(), seat_state_name(v.state));
      } else {
        RCLCPP_INFO(get_logger(), "넣은 뒤 %s (퍽 %s): %s → %s (로그만)", gg_seat_.slot.c_str(), gg_seat_.id.c_str(), v.text.c_str(), seat_state_name(v.state));
      }
    }
    gg_seat_ = GgSeat{};
  }

  PuckShape gg_shape_{PuckShape::Unknown};       // 지난 PICK grasp_for 목표 모양 (검출 없음 = 모름)
  PuckShape gg_held_shape_{PuckShape::Unknown};  // 쥔 퍽 모양 (PICK 성공 때)
  std::vector<RegripPoint> gg_regrip_;           // grasp_checks_ 와 같은 순서 (닫기마다)
  std::optional<RegripPoint> gg_last_close_;     // 이번 PICK 에서 마지막으로 본 닫기
  std::optional<RegripPoint> gg_pick_close_;     // 성공한 PICK 의 마지막 닫기 (held_regrasp)
  std::string gg_held_id_;                       // 지난 PICK · RETRIEVE 의 목표 퍽 ID ("" = 모름 → 쥐고 멈춘 뒤 처음 본 손 안 태그로 고정)
  double gg_release_t_{1e18};                    // STORE · PLACE 놓기를 보낸 시각 (이 뒤 쥔 퍽 태그 버림)
  std::map<std::string, int> gg_drop_;           // 버린 쥔 퍽 태그 수 (이유별)
  ContactWatch gg_contact_;
  std::vector<double> gg_contact_end_;           // 처음 넣기 끝 자세 명령 (다시 넣기 기준)
  std::optional<Eigen::Vector2d> gg_pick_xy_;    // 쥔 퍽을 박스에서 쥔 자리 (PICK 마지막 닫기 손끝 xy, base_link) — store_recover 가 거기 먼저 내려놓음
  // store_recover (넣기 걸림 보정)
  static constexpr const char * kStoreReplan = "__store_replan";   // 논리 대기: 다시 쥔 뒤 칸 경로 다시 계획 (모터 궤적 없음)
  ArmPath store_path_;                           // 지금 STORE 칸 경로 (path_plan 끝, 칸 보정 · 곧게 올리기 · 늦게 닫기 뒤)
  int store_recover_n_{0};                       // 이 STORE 에서 박스에서 다시 쥐어 넣은 횟수
  bool store_recovering_{false};                 // 보정 동작 중 (박스 쪽: 넣기 감시 · 놓은 시각 · 넣은 뒤 확인 안 함)
  bool store_return_end_{false};                 // 박스에 되돌리고 끝내는 계획 (끝까지 가도 실패 + 토큰)
  std::optional<Eigen::Vector2d> store_box_xy_;  // 지금 퍽이 박스에 놓여 있는 자리 (보정 중 내려놓은 뒤 · 다시 쥐기 전)
  Eigen::Vector2d store_put_xy_{Eigen::Vector2d::Zero()};   // 보정이 내려놓을 박스 자리
  std::vector<PathNode> store_regrip_;           // 다시 쥔 점 re_up · re_grasp · re_lift (다시 계획 실패 때 되돌려 놓기)
  std::string store_recover_note_;
  std::vector<ArmAction> visual_suffix_;         // 다시 보고 쥐기가 후반을 바꿀 때 끝에 붙임 (STORE 보정의 kStoreReplan)
  std::string visual_id_;                        // 다시 보기 퍽 ID (STORE 보정: 쥔 퍽 ID, "" = goal.puck_id)
  struct GgSeat {bool on{false}; std::string slot, id; double after{0.0}; Eigen::Vector2d cell{Eigen::Vector2d::Zero()};
    double puck_w{0.030}; std::vector<SeatSample> s;};
  GgSeat gg_seat_;

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
    PathNode viewing;
    std::vector<ArmAction> plan;
    if (at_look_variant(viewing)) {
      plan = home_plan(why);   // 보기 변형 → 정면 LOOK → home_from_look (쥔 퍽 검사도 같음)
      if (plan.empty()) {RCLCPP_WARN(get_logger(), "자동 접기 안 함: %s", why.c_str()); return;}
    } else {
      if (fold == paths_.end() || !path_start_ok(fold->second, joint_buf_, path_start_tol_, why)) {
        RCLCPP_WARN(get_logger(), "자동 접기 안 함: 팔이 P0 가 아님 (%s)", why.c_str());
        return;
      }
      plan = path_actions(fold->second, timing_, grip_, joint_buf_);
    }
    if (plan.empty()) {return;}
    auto_folding_ = true;
    home_was_path_ = true;
    runner_.start(plan);
    arbiter_.request_arm();
    publish_turn();
    RCLCPP_INFO(get_logger(), "P0 또는 보기 변형에서 %.1f s 동안 다음 팔 작업 없음 → 스스로 접음 (동작 %zu개)", auto_fold_s_, plan.size());
  }

  // 손이 빈 것이 확실할 때 (빈손 확인): 쥔 퍽 상태를 모두 지움 (STORE · PLACE 성공 뒤와 같은 상태)
  void clear_held()
  {
    carrying_ = false;
    held_from_pick_ = false;
    held_lying_ = false;
    held_tilt_ = 0.0;
    held_depth_mm_ = 0.0;
    depth_from_pick_ = false;
    held_retrieve_slot_.clear();
    look_rack_slot_.clear();
    held_buf_.clear();
    held_fresh_buf_.clear();
  }

  // 실패 뒤 자동 접기 (fail_fold_s): 팔 작업이 실패로 끝난 뒤 fail_fold_s 동안 새 팔 작업이 없으면 HOME 과 같은 복귀 (home_plan:
  //   빈손 되돌아가기 · 보기 변형 다리 · 검사된 복귀 경로, 그 밖은 HOME 과 같은 거절) 로 스스로 접음. 접는 동안 ready 꺼짐 (P0 자동 접기와 같음)
  void maybe_fail_fold(double now_s)
  {
    if (fail_since_ < 0.0) {return;}
    const double wait_s = get_parameter("fail_fold_s").as_double();   // 실행 중 ros2 param set 도 바로
    if (!(wait_s > 0.0)) {fail_since_ = -1.0; return;}
    if (now_s - fail_since_ < wait_s) {return;}
    fail_since_ = -1.0;
    std::string why;
    if (!task_ready(now_s, why)) {
      RCLCPP_WARN(get_logger(), "실패 뒤 자동 접기 안 함: %s", why.c_str());
      return;
    }
    FailFoldState s;
    s.folded = fold_->confirmed(now_s);
    s.carrying = carrying_;
    s.half_closed = std::isfinite(grip_now_) && grip_now_ > gripper_close_ + 0.003 && grip_now_ < gripper_open_ - 0.004;   // home_plan 과 같은 기준
    s.empty_grasp = !empty_why_.empty();
    s.step_mode = job_step_;
    s.park_left = park_empty_;
    why = fail_fold_block(s);
    if (!why.empty()) {
      if (s.folded) {RCLCPP_INFO(get_logger(), "실패 뒤 자동 접기 안 함: %s", why.c_str());}
      else {RCLCPP_WARN(get_logger(), "실패 뒤 자동 접기 안 함: %s", why.c_str());}
      return;
    }
    auto plan = home_plan(why);
    if (plan.empty()) {
      RCLCPP_WARN(get_logger(), "실패 뒤 자동 접기 안 함: %s", why.c_str());
      return;
    }
    auto_folding_ = true;
    fail_folding_ = true;
    runner_.start(plan);
    arbiter_.request_arm();
    publish_turn();
    RCLCPP_INFO(get_logger(), "팔 작업 실패 뒤 %.1f s 동안 다음 팔 작업 없음 → 스스로 접음 (동작 %zu개)", wait_s, plan.size());
  }

  // 판단은 ArmRunner(arm.cpp, test_arm.cpp)가 하고, 여기서는 그 명령을 ROS 로 옮기기만 함
  void run_arm(const rclcpp::Time & t)
  {
    if (!arm_goal_ && !auto_folding_) {
      maybe_auto_fold(t.seconds());
      if (!auto_folding_) {maybe_fail_fold(t.seconds());}
      return;
    }
    if (arm_goal_ && arm_goal_->is_canceling()) {runner_.request_cancel();}
    // 실패 뒤 자동 접기 도중 fail_fold_s 를 0.0 (끔) 으로 바꾸면 그 접기도 취소 (goal 이 없어 planning · robot_cli 취소가 안 닿음)
    //   끝은 CANCELED · 바퀴 잠금 · 다시 안 함 (사람이 보고 HOME). P0 자동 접기 (lazy_fold) 는 그대로
    if (fail_folding_ && !(get_parameter("fail_fold_s").as_double() > 0.0)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "fail_fold_s 끔 → 실패 뒤 자동 접기 멈춤 (취소, 바퀴 잠금 · 사람이 보고 HOME)");
      runner_.request_cancel();
    }

    const double now_s = t.seconds();
    advance_visual_regrasp(now_s);   // 다시 보기 대기 중이면 새 관측 → 계획 교체 또는 원래 계획 (바로 아래 tick 이 다음 동작을 보냄)
    advance_held_image(now_s);       // 쥔 퍽 영상 검사 대기 중이면 판단 → 그대로 또는 박스에 내려놓고 다시 쥐기
    for (ArmCommand c : runner_.tick(now_s, arbiter_.arm_may_move(), fold_->confirmed(now_s))) {   // 복사: 빈손 끝내기가 동작 · 결과를 바꿈
      switch (c.kind) {
        case ArmCommand::Kind::SEND: {
          if (!grasp_empty_why_.empty()) {   // 빈손으로 위 점까지 올렸음 → 다음 동작을 보내지 않고 실패로 끝 (arm_off_path_ · home_plan 흐름)
            runner_.fail_now(grasp_empty_why_);
            grasp_empty_why_.clear();
            break;
          }
          if (!c.action.is_gripper && c.action.pose == kRegraspObserve) {   // 다시 보기 대기: 재생기에 안 보냄 (advance_visual_regrasp 가 끝냄)
            active_handle_.reset();
            active_client_.reset();   // 대기 중 취소할 궤적 없음
            visual_waiting_ = true;
            visual_gen_ = c.gen;
            visual_start_ = now_s;
            const double settle = get_parameter("regrasp_visual_settle_s").as_double();
            visual_after_ = now_s + (std::isfinite(settle) && settle > 0.0 ? settle : 0.0);
            visual_deadline_ = now_s + c.action.duration_s;
            runner_.on_goal_response(c.gen, true, now_s);
            if (arm_goal_) {
              auto fb = std::make_shared<ArmTask::Feedback>();
              fb->phase = c.action.phase;
              arm_goal_->publish_feedback(fb);
            }
            RCLCPP_INFO(get_logger(), "다시 쥐기: 세워 놓은 퍽 다시 보기 (최대 %.1f s, 못 보면 원래 자리에서 쥠)", c.action.duration_s);
            break;
          }
          if (!c.action.is_gripper && c.action.pose == kHeldImageCheck) {   // 쥔 퍽 영상 검사: 재생기에 안 보냄 (advance_held_image 가 끝냄)
            active_handle_.reset();
            active_client_.reset();
            held_img_.waiting = true;
            held_img_.gen = c.gen;
            held_img_.t0 = now_s;
            runner_.on_goal_response(c.gen, true, now_s);
            if (arm_goal_) {
              auto fb = std::make_shared<ArmTask::Feedback>();
              fb->phase = c.action.phase;
              arm_goal_->publish_feedback(fb);
            }
            RCLCPP_INFO(get_logger(), "쥔 퍽 영상: %s 멈춤 → 손목 영상으로 잼 (%.1f s, 최대 %.1f s)",
              held_img_.tries == 0 ? "들기 끝에서" : "다시 쥔 뒤", get_parameter("held_image_settle_s").as_double(),
              get_parameter("held_image_timeout_s").as_double());
            break;
          }
          if (!c.action.is_gripper && c.action.pose == kStoreReplan) {   // STORE 보정: 다시 쥔 뒤 칸 경로 다시 계획 (재생기에 안 보냄)
            store_replan(c.gen, now_s);
            break;
          }
          if (!c.action.is_gripper && standup_check_ && c.action.pose == standup_check_->pose) {
            std::string bad;
            bool empty = false;
            const bool ok = standup_grasp_ok(bad, &empty);
            standup_check_.reset();
            if (!ok) {
              RCLCPP_ERROR(get_logger(), "세우기 멈춤: %s", bad.c_str());
              if (empty) {empty_why_ = "세우기 멈춤: " + bad;}   // 빈손 → 결과 EMPTY_GRASP · 쥔 상태 지움 (FINISH)
              runner_.fail_now("세우기 멈춤: " + bad);
              break;   // 세우기 경로를 안 보냄
            }
          }
          ArmAction action = c.action;
          // 쥐기 (닫기) 다음 팔 동작 전 빈손 확인 (pick_path 의 grasp_checks_): 닫기 동작 (gripper_time_s) 결과를 받은 뒤라 실측이 따라옴, NaN 이면 안 봄
          if (!action.is_gripper && !grasp_checks_.empty() && action.pose == grasp_checks_.front().pose) {
            const GraspCheck gc = grasp_checks_.front();
            grasp_checks_.erase(grasp_checks_.begin());
            if (grasp_empty(grip_now_, grasp_empty_m_)) {
              char buf[160];
              std::snprintf(buf, sizeof(buf), "빈손: 그리퍼가 %.4f m 까지 닫힘 — 퍽을 못 쥠 (세우기 · 집기 멈춤)", grip_now_);
              if (gc.in_box) {
                empty_why_ = buf;   // 결과 EMPTY_GRASP · 쥔 상태 지움 (FINISH) — 퍽이 박스에 남는 닫기만
                if (store_recovering_) {store_box_xy_ = store_put_xy_;}   // STORE 보정 다시 쥐기 빈손: 퍽은 박스에 내려놓은 자리에
              } else {   // 받침 안 다시 쥐기: 퍽이 로봇 위 받침에 남음 → 표시 없이 실패 (planning 은 재고 불확실 → 사람 확인), 실패 뒤 자동 접기 안 함
                park_empty_ = true;
                RCLCPP_WARN(get_logger(), "받침 안 다시 쥐기 빈손 — 퍽이 받침 (HOME 자리) 에 남았을 수 있음 → 빈손 표시 · 자동 접기 안 함, 사람이 받침 확인");
              }
              if (!cut_at_up(action, gc.up_q)) {   // 위 점이 없으면 그 자리에서 끝
                RCLCPP_ERROR(get_logger(), "%s → 그 자리에서 멈춤", buf);
                runner_.fail_now(buf);
                break;
              }
              RCLCPP_ERROR(get_logger(), "%s → 쥐기 높이 위 점까지만 올린 뒤 끝 (%.1f s)", buf, action.duration_s);
              grasp_empty_why_ = buf;
              empty_retreat_ = gc.back;   // 그 점에서 온 길로 되돌아가기 (HOME · 실패 뒤 자동 접기, 없으면 빔)
              empty_retreat_note_ = gc.back_note;
              if (gc.back.nodes.empty()) {RCLCPP_WARN(get_logger(), "빈손 되돌아가기 경로 없음 (모르는 구간) → HOME 은 지금 규칙대로");}
            } else {
              gg_check_width(action, gc.pose, gc.up_q);   // grasp_guard: 쥔 폭 (잘못 쥐면 그 자리 다시 쥐기를 앞에 끼움)
            }
          }
          gg_on_send(action);   // grasp_guard: 넣기 감시 · 다시 넣기 · 놓은 시각 · 넣은 뒤 확인
          if (arm_goal_) {
            auto fb = std::make_shared<ArmTask::Feedback>();
            fb->phase = action.phase;
            arm_goal_->publish_feedback(fb);
          }
          if (action.is_gripper) {
            send_trajectory(grip_client_, {gripper_joint_}, {{{action.gripper}, gripper_time_s_}}, c.gen);
          } else if (!action.points.empty()) {
            send_trajectory(arm_client_, arm_joints_, action.points, c.gen);
            if (action.side_grip_t >= 0.0) {   // 팔을 멈추지 않고 그 시각에 그리퍼만 (세우기 여는 점)
              const double g = action.side_grip;
              side_grip_timer_ = create_wall_timer(std::chrono::duration<double>(action.side_grip_t), [this, g]() {
                  side_grip_timer_->cancel();
                  send_side_gripper(g);
                });
            }
          } else {
            send_trajectory(arm_client_, arm_joints_, {{poses_.at(action.pose), move_time_s_}}, c.gen);
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
          visual_waiting_ = false;
          held_img_.waiting = false;
          if (c.end == ArmEnd::SUCCEEDED && !visual_note_.empty()) {c.message += " · " + visual_note_;}
          if (!held_img_.note.empty()) {
            c.message += " · " + held_img_.note;
            RCLCPP_INFO(get_logger(), "쥔 퍽 영상 끝: %s (검사 %d 번)", held_img_.note.c_str(), held_img_.checks);
          }
          visual_note_.clear();
          if (!grasp_empty_why_.empty()) {   // 빈손으로 위 점까지 올린 동작이 마지막이었음 → 성공이 아니라 실패 (퍽을 안 쥠)
            if (c.end == ArmEnd::SUCCEEDED) {c.end = ArmEnd::ABORTED; c.drive_ok = false; c.message = "팔 작업 중단: " + grasp_empty_why_;}
            grasp_empty_why_.clear();
          }
          if (store_return_end_ && c.end == ArmEnd::SUCCEEDED) {   // STORE 보정: 박스에 되돌리고 P0 까지 뺌 → 넣기는 실패
            c.end = ArmEnd::ABORTED;
            c.drive_ok = false;
            c.message = "STORE 보정: 칸 넣기 걸림 (박스에서 다시 쥐어 넣기 " + std::to_string(store_recover_n_) + " 번 뒤) → 퍽을 박스에 되돌려 놓음" +
              (store_recover_note_.empty() ? "" : " (" + store_recover_note_ + ")");
          }
          if (auto_folding_) {   // 스스로 접기 (goal 없음): 성공 + 접힘 확인이면 주행 허용, 아니면 잠금 유지 · 다음 HOME 이 복구
            const char * what = fail_folding_ ? "실패 뒤 자동 접기" : "자동 접기";
            auto_folding_ = false;
            fail_folding_ = false;
            arm_off_path_ = c.end != ArmEnd::SUCCEEDED;
            if (c.end == ArmEnd::SUCCEEDED) {empty_retreat_ = ArmPath{}; empty_retreat_note_.clear();}   // 접힘 → 되돌아가기 끝
            if (c.end != ArmEnd::SUCCEEDED) {RCLCPP_ERROR(get_logger(), "%s 실패: %s (접힘 확인: %s)", what, c.message.c_str(), fold_->status(now_s).c_str());}
            arbiter_.arm_ended(c.drive_ok && hw_ok_before_);
            publish_turn();
            RCLCPP_INFO(get_logger(), "%s 끝 (%s) → %s 차례", what, c.message.c_str(), turn_name(arbiter_.turn()));
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
              held_fresh_buf_.clear();held_fresh_id_=arm_goal_->get_goal()->puck_id;
              gg_on_hold(arm_goal_->get_goal()->puck_id, task == ArmTask::Goal::PICK);   // grasp_guard: 쥔 퍽 태그 ID · 시각 창
            }
            if (task == ArmTask::Goal::STORE || task == ArmTask::Goal::PLACE || task == ArmTask::Goal::HOME) {carrying_ = false;}
            held_from_pick_ = task == ArmTask::Goal::PICK && carrying_;   // 집은 자리에 쥔 채 멈춤 (박스 위 기준점 아님)
            look_rack_slot_ = task == kArmLookRack ? arm_goal_->get_goal()->slot : "";   // 관찰 자세에서 그 칸 STORE 를 기다림
            if (task != kArmLookRack) {held_tilt_ = task == ArmTask::Goal::PICK ? pending_tilt_ : 0.0;}   // LOOK_RACK 은 쥔 퍽 그대로
            if (task == ArmTask::Goal::PICK) {held_lying_ = pending_lying_;}
            else if (task == ArmTask::Goal::RETRIEVE || !carrying_) {held_lying_ = false;}
            if (task == ArmTask::Goal::PICK) {held_depth_mm_ = pending_depth_; depth_from_pick_ = true;}
            else if (task == ArmTask::Goal::RETRIEVE) {
              held_depth_mm_ = retrieve_depth_mm_;
              const std::string rk = "retrieve_offset." + arm_goal_->get_goal()->slot;   // 세 번째 값 (높이, − = 더 깊게) 만큼 쥔 깊이도 다름 → PLACE 놓는 높이
              if (!retrieve_full_ && has_parameter(rk)) {   // retrieve_<칸>_full 경로는 그 높이가 이미 들어 있어 안 뺌

                const auto c = get_parameter(rk).as_double_array();
                if (c.size() == 3 && std::isfinite(c[2])) {held_depth_mm_ -= c[2];}
              }
              depth_from_pick_ = false;
            }
            else if (!carrying_) {held_depth_mm_ = 0.0; depth_from_pick_ = false;}
            if (task == ArmTask::Goal::RETRIEVE) {held_retrieve_slot_ = arm_goal_->get_goal()->slot;}
            else if (task == ArmTask::Goal::PICK || !carrying_) {held_retrieve_slot_.clear();}
          }
          if (c.end != ArmEnd::SUCCEEDED) {held_from_pick_ = false; look_rack_slot_.clear();}   // 도중에 끝남 → 집은 자리에서 잇기 안 함 (arm_off_path_)
          // 빈손 확인이 걸린 작업: 손이 빈 것은 확실 → 쥔 상태를 모두 지움. 그 이유로 끝났으면 결과 앞에 고정 표시 (planning 이 글 대신 봄)
          if (!empty_why_.empty() && c.end != ArmEnd::SUCCEEDED) {
            clear_held();
            if (c.end == ArmEnd::ABORTED && c.message.find(empty_why_) != std::string::npos) {result->message = kEmptyGraspTag + c.message;}
          }
          // STORE 보정: 퍽이 박스에 놓여 있음 (내려놓은 뒤 다시 쥐기 전 · 되돌려 놓음) → 손은 빈 것 · 결과 글에 토큰 (미션이 박스 재고로 되돌림)
          if (task == ArmTask::Goal::STORE && store_box_xy_ && c.end != ArmEnd::SUCCEEDED) {
            clear_held();
            const std::string tok = store_returned_token(store_box_xy_->x(), store_box_xy_->y());
            result->message = result->message.rfind(kEmptyGraspTag, 0) == 0 ? result->message + " · " + tok : tok + " · " + result->message;
            RCLCPP_WARN(get_logger(), "STORE %s 보정 끝: %s", arm_goal_->get_goal()->slot.c_str(), result->message.c_str());
          } else if (task == ArmTask::Goal::STORE && c.end == ArmEnd::SUCCEEDED && store_recover_n_ > 0) {
            result->message += " · 넣기 걸림 보정 " + std::to_string(store_recover_n_) + " 번 (박스에서 다시 쥐어 넣음)";
            RCLCPP_INFO(get_logger(), "STORE %s 보정 끝: 다시 넣기 성공 (%s)", arm_goal_->get_goal()->slot.c_str(), result->message.c_str());
          } else if (task == ArmTask::Goal::STORE && store_recover_n_ > 0) {
            RCLCPP_ERROR(get_logger(), "STORE %s 보정 뒤 실패 (퍽 쥔 채): %s", arm_goal_->get_goal()->slot.c_str(), result->message.c_str());
          }
          store_recovering_ = false;
          store_return_end_ = false;
          visual_suffix_.clear();
          // 실패 뒤 자동 접기 시계 (fail_fold_s): 실패 · HOME 아님 · 하드웨어 정상일 때만 (취소 · 성공은 안 함)
          fail_since_ = -1.0;
          if (fail_fold_arm(c.end, task == ArmTask::Goal::HOME, get_parameter("fail_fold_s").as_double(), hw_ok_before_)) {
            fail_since_ = now_s;
            RCLCPP_INFO(get_logger(), "팔 작업 실패 → %.1f s 안에 새 팔 작업이 없으면 스스로 접음 (fail_fold_s)", get_parameter("fail_fold_s").as_double());
          } else if (c.end == ArmEnd::ABORTED && !hw_ok_before_) {
            RCLCPP_WARN(get_logger(), "하드웨어 이상으로 끝난 팔 작업 → 실패 뒤 자동 접기 안 함 (복구 뒤 HOME)");
          }
          if (scanning_) {
            scanning_ = false;
            if (c.end == ArmEnd::SUCCEEDED) {finish_rack_scan();} else {scan_seen_.clear();}
          }
          gg_on_finish(task);   // grasp_guard: 넣기 감시 끝 · STORE 넣은 뒤 한 줄
          p0_idle_since_ = c.end == ArmEnd::SUCCEEDED && task_ends_p0_ ? now_s : -1.0;   // P0 에서 쉬기 시작 (보기 변형 자세는 예전처럼 유지)
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
    // 보드 상태 없이 볼 수 있나 (hw_status_optional): 서비스가 보이나 · 팔 · 그리퍼 재생기 준비 (HardwareWatch::components_waived)
    hw_->set_components_optional(get_parameter("hw_status_optional").as_bool());
    hw_->set_components_reachable(hw_list_client_->service_is_ready());
    hw_->set_controllers_ready(arm_client_->action_server_is_ready() && grip_client_->action_server_is_ready());
    if (hw_list_pending_) {
      // 답이 안 오는 요청은 버리고 다시 (rclcpp 는 답이 올 때까지 요청을 보관 → 쌓이지 않게). 그동안은 HardwareWatch 가 "모름" = 이상
      if (t - hw_list_sent_ < 2.0) {return;}
      hw_list_client_->prune_pending_requests();
      hw_list_pending_ = false;
      ++hw_list_misses_;
    }
    // 답이 3 번 (6 s) 연속 안 오거나 서비스가 10 s 넘게 안 보이면 클라이언트를 새로 만듦 — 11-10 실물: 켤 때 DDS 연결이 꼬여
    //   controller_manager 답이 영영 안 와 "보드 상태를 아직 못 받음" 으로 팔 작업을 계속 거절 (다시 켜야 풀렸음)
    const bool ready = hw_list_client_->service_is_ready();
    if (ready) {hw_list_unready_since_ = -1.0;} else if (hw_list_unready_since_ < 0.0) {hw_list_unready_since_ = t;}
    if (hw_list_misses_ >= 3 || (!ready && t - hw_list_unready_since_ > 10.0)) {
      RCLCPP_WARN(get_logger(), "보드 상태: controller_manager 답이 %s → 연결을 새로 만듦", hw_list_misses_ >= 3 ? "6 s 넘게 없음" : "10 s 넘게 서비스 안 보임");
      hw_list_misses_ = 0;
      hw_list_unready_since_ = -1.0;
      hw_list_client_ = create_client<ListHw>("controller_manager/list_hardware_components");
      return;
    }
    if (!ready) {return;}
    hw_list_pending_ = true;
    hw_list_sent_ = t;
    hw_list_client_->async_send_request(std::make_shared<ListHw::Request>(),
      [this](rclcpp::Client<ListHw>::SharedFuture f) {
        hw_list_pending_ = false;
        hw_list_misses_ = 0;
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
    if (!reject_why_.empty() && rclcpp::Time(m.header.stamp).seconds() < reject_until_s_) {m.reason = "거절: " + reject_why_;}
    std::string why;
    m.ready = task_ready(rclcpp::Time(m.header.stamp).seconds(), why);
    if (m.ready != ready_sent_) {
      RCLCPP_INFO(get_logger(), "작업 준비 %s%s", m.ready ? "됨" : "안 됨: ", why.c_str());
    }
    ready_sent_ = m.ready;
    {
      std::lock_guard<std::mutex> lk(hb_mutex_);   // 발행 · 저장을 한 잠금 안에서 → 심장박동이 새 메시지 뒤에 옛 메시지를 내지 않음
      pub_turn_->publish(m);
      last_turn_msg_ = m;
      last_turn_pub_s_ = steady_s();
      last_turn_real_s_ = last_turn_pub_s_;
    }
  }

  // 심장박동 (hb_timer_, 별도 콜백 그룹 = 다른 스레드): 팔 계획 (on_arm_goal 의 build_plan, 동기 · 수 초) 동안 turn_timer_ 가 못 돌아
  //   /control/motion_mode 가 끊기면 planning 이 control_timeout_s (1 s) 로 FAULT (11-09 실물 10 s 계획). 마지막 publish_turn 뒤
  //   kHeartbeatGapS 넘게 발행이 없을 때만 그 메시지를 stamp 만 갱신해 재발행 (상태 · arbiter · hw 는 안 건드림 — 같은 내용 반복)
  void heartbeat()
  {
    std::lock_guard<std::mutex> lk(hb_mutex_);
    // 진짜 publish_turn 이 kHeartbeatMaxS 넘게 없으면 멈춘 노드를 가리지 않게 재발행도 그만 (planning 이 끊김으로 알아챔)
    //   DRIVE 는 재발행 안 함 (오래된 주행 허가를 늘리지 않게) — 팔 계획은 STOP · ARM 차례에서 받음 (LOOK 뒤 바퀴 잠금 유지)
    if (!last_turn_msg_ || last_turn_msg_->mode == interfaces::msg::MotionMode::DRIVE ||
      !heartbeat_due(last_turn_pub_s_, steady_s(), kHeartbeatGapS) || steady_s() - last_turn_real_s_ > kHeartbeatMaxS) {return;}
    interfaces::msg::MotionMode m = *last_turn_msg_;
    m.header.stamp = now();
    pub_turn_->publish(m);
    last_turn_pub_s_ = steady_s();
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
  std::vector<PathNode> look_history_;   // 파라미터를 바꿔도 이전 보기 목표를 실제 관절로 알아봄
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
    Eigen::Vector3d p_nom; std::string id;   // p = 카메라 돌아감 되돌린 위치, p_nom = 되돌리기 전 (목표 맞추기용), id = AprilTag ID ("" = 못 읽음)
    bool exact_tf; double capture_t;};       // exact_tf = 촬영 시각 TF 로 odom 에 옮김 (false = 최신 TF 로 대신), capture_t = 촬영 시각
  std::vector<SeenPuck> seen_;
  std::string want_id_;           // 지금 맞출 PICK 목표 퍽 ID (goal.puck_id, "" = 위치로만). pick_plan 동안만
  double id_max_offset_m_{0.06};  // ID 로 찾은 검출 ↔ planning 목표점 거리 한계 (넘으면 거절)
  bool target_tagged_{false};     // 마지막 grasp_for 가 맞춘 목표 검출이 윗면 태그로 잰 퍽
  bool target_square_{true};      // 마지막 grasp_for 가 맞춘 목표가 네모 (모양 모름 = 네모) — farpick_plan 의 표 side
  // 다시 보고 쥐기 (regrasp_visual): 계획 때 regrasp_nodes 가 기억 → 대기 중 advance_visual_regrasp 가 씀
  bool visual_enabled_{false}, visual_waiting_{false};
  unsigned visual_gen_{0};
  double visual_start_{0.0}, visual_after_{0.0}, visual_deadline_{0.0};
  Eigen::Vector3d visual_expected_{Eigen::Vector3d::Zero()};   // 놓은 자리 (x, y, 퍽 중심 z) base_link
  Eigen::Vector2d visual_blind_xy_{Eigen::Vector2d::Zero()};   // 보지 않고 다시 쥘 점 (regrasp_offset_mm 포함)
  Q5 visual_seed_{};                 // 원래 수직 다시 쥐기 해 (re_above) — IK 씨앗
  double visual_top_{0.0}, visual_close_deg_{0.0};   // 원래 다시 쥐기 높이 (pick_top_bias 등 그대로) · 닫힘 방향
  std::vector<double> visual_obs_q_;     // 기억한 관찰 점 re_observe (이 경로의 것일 때만 대기를 끼움)
  ArmPath visual_tail_;              // re_observe 부터 원래 후반 경로
  std::string visual_note_;          // 결과 설명에 붙임
  bool regrasp_{true};            // 기울여 쥔 퍽을 세워 내려놓고 수직으로 다시 쥠
  double regrasp_min_x_{0.212};   // 다시 잡기 자리 x 하한 [m]
  double regrasp_max_abs_y_{0.075};   // 다시 잡기 · 먼 자리 내려놓기 자리 |y| 상한 [m] (박스 안)
  std::optional<Eigen::Vector2d> exclude_;   // 이웃에서 뺄 퍽 (다시 잡기: 집어 든 퍽) base_link
  double seen_t_{std::nan("")};   // 마지막 손목 인지 결과 시각
  double nb_age_s_{10.0}, nb_radius_{0.12}, nb_min_clear_{0.002}, nb_target_clear_{0.003};
  std::string odom_frame_{"odom"};
  double max_tilt_deg_{20.0};
  double held_tilt_{0.0};          // 쥔 퍽의 손 안 기울임 [°] (PICK 때 손 기울임, RETRIEVE = 0)
  double pending_tilt_{0.0};       // 받은 PICK 계획의 손 기울임 (성공하면 held_tilt_)
  bool depth_from_pick_{false};    // held_depth_mm_ 이 PICK 계획의 깊이 (RETRIEVE 의 25 mm 는 추정값이라 STORE 높이 맞춤에 안 씀)
  bool retrieve_full_{false};      // 이번 RETRIEVE 가 retrieve_<칸>_full (칸 보정 포함) 경로 → retrieve_offset 높이를 쥔 깊이에 안 뺌
  double held_depth_mm_{0.0};      // 쥔 퍽 깊이 = 퍽 윗면 ↔ 손끝 [mm] (PICK = 그 계획의 쥐는 깊이, RETRIEVE = retrieve_grip_depth_mm, 0 = 모름)
  std::string held_retrieve_slot_; // 마지막 성공한 RETRIEVE 칸임. PICK · 놓기 · HOME 성공 뒤 비움.
  std::optional<double> place_depth_override_mm_; // PLACE 계획 중에만 쓸 깊이임. STORE · RETRIEVE 파지 상태는 그대로 둠.
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
  std::map<std::string, std::vector<StandupPath>> farpick_;   // 먼 · 옆 퍽 붙은 선 퍽 집기 표 (farpick_table_name → 표, config/farpick/)
  std::vector<StandupPath> lyingpick_;   // 먼 누운 네모 퍽 가운데 쥐기 표 (config/lyingpick/, park_or_relocate_plan ① 쥐기)
  struct StandupCheck {std::string pose; Eigen::Vector3d tip;};   // 세우기 첫 구간 이름 · 쥐는 자리 손끝 (모델)
  std::optional<StandupCheck> standup_check_;
  double standup_reach_tol_mm_{6.0}, standup_grip_min_m_{0.001};
  // 쥐기 (닫기) 다음 팔 동작 이름 · 쥐기 높이 위 점 명령값 (없으면 빔) · 빈손이면 그 점에서 온 길로 되돌아가는 경로 (없으면 빔, empty_retreat_path)
  //   in_box: 빈손이면 퍽이 박스에 남는 닫기 (EMPTY_GRASP 표시 · 되돌아가기). park_ 닫기 (받침 안 다시 쥐기) 는 false
  struct GraspCheck {std::string pose; std::vector<double> up_q; ArmPath back{}; std::string back_note{}; bool in_box{true};};
  std::vector<GraspCheck> grasp_checks_;   // PICK 계획의 닫기마다 (경로 순서, 검사하면 지움)
  std::string grasp_empty_why_;            // 빈손 확인 → 위 점까지만 올린 뒤 다음 동작 전에 실패로 끝낼 이유
  std::string empty_why_;                  // 이번 작업에서 걸린 빈손 확인 이유 ("" = 안 걸림, 작업마다 지움) → FINISH 에서 EMPTY_GRASP · 쥔 상태 지움
  ArmPath empty_retreat_;                  // 빈손으로 멈춘 점에서 온 길로 되돌아가는 경로 (home_plan, 빈 = 없음). 새 작업 · 접힌 뒤 지움
  std::string empty_retreat_note_;         // 그 경로 점 이름 (로그)
  static constexpr const char * kEmptyGraspTag = "EMPTY_GRASP: ";   // 빈손으로 끝난 작업 결과 글 앞의 고정 표시 (기계가 읽음)
  double fail_since_{-1.0};                // 팔 작업이 실패로 끝난 시각 (실패 뒤 자동 접기 기다림, −1 = 없음)
  bool fail_folding_{false};               // 지금 자동 접기가 실패 뒤 자동 접기 (auto_folding_ 과 같이)
  bool job_step_{false};                   // 진행 중 · 마지막 작업이 구분 동작 (park_step)
  bool park_empty_{false};                 // 이번 작업에서 받침 안 다시 쥐기 (park_ 닫기) 가 빈손 → 실패 뒤 자동 접기 안 함 (작업마다 지움)
  rclcpp::node_interfaces::PreSetParametersCallbackHandle::SharedPtr fail_fold_int_cb_;   // fail_fold_s 정수 → 실수
  bool hw_waived_{false};                  // 보드 상태 대신 관절 측정으로 보는 중 (로그 한 번씩)
  double grasp_empty_m_{0.0015};           // 빈손 문턱 [m] (grasp_empty_m)
  double path_slow_min_dps_{0.0};
  bool side_grip_{true};   // 빈손 그리퍼 동작을 팔을 멈추지 않고 (gripper_on_the_way)
  double standup_speed_dps_{32.0}, standup_max_gap_mm_{6.0}, standup_max_gap_deg_{6.0};
  bool lying_cylinder_{false};   // 마지막 grasp_for 의 누운 퍽이 원기둥 (세우기 안 함)
  double lying_axis_deg_{std::numeric_limits<double>::quiet_NaN()};   // 마지막 grasp_for 의 누운 퍽 긴축 [°]
  struct HeldTag {std::string id; Eigen::Vector3d p, n;};   // 쥔 퍽 태그 (손목 좌표)
  std::deque<HeldTag> held_buf_;
  std::vector<HeldSample> held_fresh_buf_;
  std::string held_fresh_id_;
  double held_ref_depth_mm_{0.0};
  rclcpp::Subscription<interfaces::msg::DetectionArray>::SharedPtr sub_held_fresh_;
  double held_since_{1e18};
  double held_max_mm_{8.0};
  Eigen::Vector2d place_off_mm_{0.0, 0.0};   // 이번 PLACE 에 더한 놓는 점 보정 [mm] (place_offset_x/y_mm, 로그용)
  std::optional<Eigen::Vector3d> held_ref_p_, held_ref_n_;
  rclcpp::Subscription<interfaces::msg::DetectionArray>::SharedPtr sub_held_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_held_ref_;
  // 쥔 퍽 영상 검사 (held_image)
  static constexpr const char * kHeldImageCheck = "__held_image_check";   // 논리 대기: 들기 끝에서 멈춰 쥔 퍽 영상 재기 (모터 궤적 없음)
  struct HeldImageRun
  {
    bool waiting{false};
    unsigned gen{0};
    double t0{0.0};                    // 대기 시작 (들기 끝)
    int tries{0};                      // 이 PICK 에서 박스에 내려놓고 다시 쥔 횟수
    int checks{0};                     // 잰 횟수
    std::vector<ArmAction> rest;       // 처음 검사 뒤 원래 PICK 동작 (다시 쥔 뒤 그대로 붙임)
    std::string note;                  // 결과 글에 붙임
  };
  HeldImageRun held_img_;
  std::deque<HeldImageSample> held_img_buf_;      // /held_puck/image (쥔 퍽 있는 장만, 최근 3 s)
  double held_img_last_rx_{-1.0};                 // 마지막으로 받은 장 (빈 배열 포함) 시각 [s, 노드 시계]
  std::map<std::string, HeldImageRef> held_img_refs_;   // 모양 (box · cylinder) → 기준
  rclcpp::Subscription<interfaces::msg::DetectionArray>::SharedPtr sub_held_img_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_held_img_ref_;
  Eigen::Vector2d rack_pivot_{Eigen::Vector2d::Zero()};
  std::string rack_ref_file_;
  std::string rack_fit_file_;      // 마지막 스캔 맞춤 기억 (rack_fit_persist)
  bool rack_fit_persist_{true};
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
  std::vector<double> fold_tol_;             // HOME 옮김이 바뀌면 fold_ 를 다시 만듦
  double fold_hold_s_{0.3}, fold_max_age_s_{0.2};
  std::vector<double> home_q0_;              // 옮기기 전 poses.home
  std::vector<std::pair<std::string, std::size_t>> home_nodes_;   // 받침 위 기둥 점 (경로 이름, 점 번호)

  std::unique_ptr<HardwareWatch> hw_;        // 하드웨어 측정 살아 있나 (재검증 R6 · 보드 2장)
  std::vector<std::string> hw_watch_joints_;  // 측정을 볼 관절 (기본 14 개 = 주행 · 팔 보드 전부)
  rclcpp::Client<ListHw>::SharedPtr hw_list_client_;
  rclcpp::TimerBase::SharedPtr hw_timer_;
  bool hw_list_pending_{false};
  double hw_list_sent_{0.0};
  int hw_list_misses_{0};                   // 연속으로 답이 안 온 요청 수 (3 이면 클라이언트를 새로)
  double hw_list_unready_since_{-1.0};      // 서비스가 안 보이기 시작한 시각 (−1 = 보임)
  bool hw_ok_before_{false}, have_hw_once_{false};
  double hw_grace_until_{0.0};   // 이 시각까지 하드웨어 감시는 이전 판정 유지 (계획으로 노드가 잠깐 멈춘 뒤)
  bool ready_sent_{false};   // 마지막으로 방송한 ready (바뀔 때 바로 다시 보내려고)
  std::vector<double> joint_buf_;            // /joint_states → arm_joints 순서 (콜백마다 재사용)

  // ---------------- 상태
  MotionArbiter arbiter_;
  ArmRunner runner_{2.0, 7.0, 2.0};   // 생성자에서 설정값으로 다시 만듦
  std::mutex odom_mtx_;   // 아래 셋: /odom 콜백 (odom_group_) 이 쓰고 step() 이 읽음
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
  // 심장박동 (팔 계획 동안 /control/motion_mode 재발행): 별도 콜백 그룹 → 다른 스레드, 아래 셋만 hb_mutex_ 로
  rclcpp::CallbackGroup::SharedPtr hb_group_;
  rclcpp::CallbackGroup::SharedPtr odom_group_;   // /odom 구독 (팔 계획 중에도 받음)
  rclcpp::TimerBase::SharedPtr hb_timer_;
  std::mutex hb_mutex_;
  std::optional<interfaces::msg::MotionMode> last_turn_msg_;   // 마지막 publish_turn 메시지
  double last_turn_pub_s_{0.0};                                // 그 발행 시각 [steady s]
  double last_turn_real_s_{0.0};                               // 마지막 진짜 publish_turn 시각 [steady s]
  static constexpr double kHeartbeatGapS = 0.15;               // 이보다 오래 발행이 없을 때만 재발행
  static constexpr double kHeartbeatMaxS = 20.0;               // 계획이 이보다 길면 (멈춘 것) 재발행 그만
  // 마지막 팔 작업 거절 이유 (motion_mode reason 에 kRejectReasonS 동안 "거절: …")
  std::string reject_why_;
  double reject_until_s_{0.0};
  static constexpr double kRejectReasonS = 3.0;
};

}  // namespace manipulation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<manipulation::ControlMaster>();
  // 스레드 3: 팔 계획 (on_arm_goal, 동기) 동안에도 심장박동 (hb_timer_) 이 /control/motion_mode 를 내고 /odom (sub_odom_, odom_group_) 을 받음.
  //   나머지 콜백은 기본 그룹 하나 (서로 겹치지 않음)
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 3);
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
