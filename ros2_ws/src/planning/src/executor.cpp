// executor.cpp — 실행기 구현, 설명은 executor.hpp

#include "planning/executor.hpp"

#include <algorithm>
#include <stdexcept>
#include <chrono>
#include <cmath>

#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "planning/grasp.hpp"

namespace planning
{

using interfaces::msg::CargoSlot;
using interfaces::msg::CargoState;
using interfaces::msg::DetectionArray;
using interfaces::msg::MotionMode;

Executor::Executor(rclcpp::Node & node)
: node_(node)
{
  // ---------------------------------------------------------------- 설정값 (기본값은 여기, 실제 값은 레포 config/planning.yaml)
  map_frame_ = node_.declare_parameter<std::string>("map_frame", "map");
  base_frame_ = node_.declare_parameter<std::string>("base_frame", "base_link");
  odom_frame_ = node_.declare_parameter<std::string>("odom_frame", "odom");

  detection_max_age_s_ = node_.declare_parameter<double>("detection_max_age_s", 0.5);
  min_confidence_ = node_.declare_parameter<double>("min_confidence", 0.5);
  candidate_max_age_s_ = node_.declare_parameter<double>("candidate_max_age_s", 30.0);
  control_timeout_s_ = node_.declare_parameter<double>("control_timeout_s", 1.0);

  // ---- 준비 확인 · 실행 중 감시 (require_* = false 면 그 항목은 안 봄, 시험용)
  require_scan_ = node_.declare_parameter<bool>("require_scan", true);
  scan_timeout_s_ = node_.declare_parameter<double>("scan_timeout_s", 1.0);
  // 라이다 2 개 (앞 · 뒤) 각각 감시: 하나만 끊겨도 그쪽이 안 보이므로 이동 차단
  scan_topics_ = node_.declare_parameter<std::vector<std::string>>(
    "scan_topics", std::vector<std::string>{"/lidar_front/scan", "/lidar_rear/scan"});
  require_detections_ = node_.declare_parameter<bool>("require_detections", true);
  detection_frames_ = node_.declare_parameter<std::vector<std::string>>(
    "detection_frames", std::vector<std::string>{"d435_color_optical_frame"});
  detection_timeout_s_ = node_.declare_parameter<double>("detection_timeout_s", 1.0);
  require_localization_ = node_.declare_parameter<bool>("require_localization", true);
  loc_timeout_s_ = node_.declare_parameter<double>("loc_timeout_s", 2.0);
  loc_ready_xy_std_ = node_.declare_parameter<double>("loc_ready_xy_std_m", 0.10);
  loc_ready_yaw_std_ = node_.declare_parameter<double>("loc_ready_yaw_std_deg", 10.0) * M_PI / 180.0;
  loc_lost_xy_std_ = node_.declare_parameter<double>("loc_lost_xy_std_m", 0.30);
  loc_lost_yaw_std_ = node_.declare_parameter<double>("loc_lost_yaw_std_deg", 30.0) * M_PI / 180.0;
  const double mon_recover_s = node_.declare_parameter<double>("monitor_recover_s", 2.0);
  const int mon_max = node_.declare_parameter<int>("monitor_max_recoveries", 2);
  const double mon_fault_s = node_.declare_parameter<double>("monitor_fault_s", 10.0);
  scan_mon_ = ConditionMonitor(mon_recover_s, mon_max, mon_fault_s);
  // 검출 끊김: FAULT 까지 detection_fault_s (기본 45 s). perception 멈춤 → 멈춤 감시 5 s + 스택 기록 (gdb 최대 8 s) + respawn 2 s
  //   + 엔진 읽기 ~2 s 동안 끊김 → 10 s (monitor_fault_s) 면 돌아오기 전에 FAULT. 끊긴 동안 PICK · LOOK 은 안 보냄
  detection_fault_s_ = node_.declare_parameter<double>("detection_fault_s", 45.0);
  det_mon_ = ConditionMonitor(mon_recover_s, mon_max, detection_fault_s_);
  // 위치 추정: 잠깐(loc_fault_s) 안에 돌아오면 계속, 아니면 FAULT (데이터만 다시 와도 자동 재개 안 함)
  //   loc_max_recoveries: start 뒤 이만큼 돌아온 다음 또 끊기면 FAULT (기본 3). -1 = 횟수 제한 없음 (loc_fault_s 만 봄)
  const int loc_max = static_cast<int>(node_.declare_parameter<int>("loc_max_recoveries", 3));
  loc_mon_ = ConditionMonitor(0.0, loc_max, node_.declare_parameter<double>("loc_fault_s", 2.5));
  // 위치 추정 상실 복구 (FAULT 없음): 대기 → 마지막 정상 자리로 물러나기 → 조건부 계속. false = 위 loc_fault_s 로 FAULT
  loc_recovery_ = node_.declare_parameter<bool>("loc_recovery", false);
  {
    LocRecoveryConfig rc;
    rc.ok_hold_s = node_.declare_parameter<double>("loc_ok_hold_s", rc.ok_hold_s);
    rc.wait_s = node_.declare_parameter<double>("loc_wait_s", rc.wait_s);
    rc.degrade_s = node_.declare_parameter<double>("loc_degrade_s", rc.degrade_s);
    rc.force_s = node_.declare_parameter<double>("loc_force_s", rc.force_s);
    rc.max_backoff = static_cast<int>(node_.declare_parameter<int>("loc_backoff_tries", rc.max_backoff));
    loc_rec_ = LocRecovery(rc);
  }
  loc_backoff_max_m_ = node_.declare_parameter<double>("loc_backoff_max_m", loc_backoff_max_m_);
  loc_backoff_min_m_ = node_.declare_parameter<double>("loc_backoff_min_m", loc_backoff_min_m_);
  loc_backoff_timeout_s_ = node_.declare_parameter<double>("loc_backoff_timeout_s", loc_backoff_timeout_s_);
  loc_backoff_lookback_s_ = node_.declare_parameter<double>("loc_backoff_lookback_s", loc_backoff_lookback_s_);
  loc_stable_window_s_ = node_.declare_parameter<double>("loc_stable_window_s", loc_stable_window_s_);
  loc_stable_xy_m_ = node_.declare_parameter<double>("loc_stable_xy_m", loc_stable_xy_m_);
  loc_stable_yaw_ = node_.declare_parameter<double>("loc_stable_yaw_deg", 2.0) * M_PI / 180.0;
  // 위치 추정 출처: health = pose_guard /localization_health (실물 slam_toolbox, nav2.launch.py localization:=slam)
  //                 amcl   = /amcl_pose 공분산 + localization lifecycle (시뮬, localization:=amcl)
  const auto loc_source = node_.declare_parameter<std::string>("localization_source", "health");
  if (loc_source != "health" && loc_source != "amcl") {
    throw std::runtime_error("localization_source 는 health | amcl (지금 " + loc_source + ")");
  }
  loc_from_amcl_ = loc_source == "amcl";
  auto lifecycle_services = node_.declare_parameter<std::vector<std::string>>(
    "lifecycle_services", std::vector<std::string>{"/lifecycle_manager_navigation/is_active"});
  const std::string loc_lifecycle = "/lifecycle_manager_localization/is_active";   // map_server · AMCL (amcl 일 때만 뜸)
  if (loc_from_amcl_ &&
    std::find(lifecycle_services.begin(), lifecycle_services.end(), loc_lifecycle) == lifecycle_services.end())
  {
    lifecycle_services.push_back(loc_lifecycle);
  }

  action_retry_s_ = node_.declare_parameter<double>("action_retry_s", 1.0);
  action_retry_max_s_ = node_.declare_parameter<double>("action_retry_max_s", 15.0);
  // RETRIEVE 거절 재전송 한계 [s] (0 = action_retry_max_s). 거절은 보통 다시 보내도 같음 (적재함 맞춤 · 칸 경로) → 빨리 실패해 미션이 건너뛰게
  retrieve_retry_max_s_ = node_.declare_parameter<double>("retrieve_retry_max_s", 5.0);
  server_wait_s_ = node_.declare_parameter<double>("server_wait_s", 30.0);
  nav_timeout_s_ = node_.declare_parameter<double>("nav_timeout_s", 60.0);
  arm_timeout_s_ = node_.declare_parameter<double>("arm_timeout_s", 60.0);
  drive_wait_s_ = node_.declare_parameter<double>("drive_wait_s", 10.0);

  puck_standoff_m_ = node_.declare_parameter<double>("puck_standoff_m", 0.30);
  dock_action_ = node_.declare_parameter<std::string>("dock_action", "dock_to_pose");
  dock_nav_back_m_ = node_.declare_parameter<double>("dock_nav_back_m", 0.025);
  dock_timeout_s_ = node_.declare_parameter<double>("dock_timeout_s", 40.0);
  dock_handoff_m_ = node_.declare_parameter<double>("dock_handoff_m", 0.40);
  dock_handoff_lat_m_ = node_.declare_parameter<double>("dock_handoff_lat_m", 0.15);
  dock_handoff_yaw_deg_ = node_.declare_parameter<double>("dock_handoff_yaw_deg", 60.0);
  dock_fix_match_m_ = node_.declare_parameter<double>("dock_fix_match_m", 0.05);
  dock_fix_max_m_ = node_.declare_parameter<double>("dock_fix_max_m", 0.35);
  pick_fresh_s_ = node_.declare_parameter<double>("pick_fresh_s", 2.0);
  puck_half_height_m_ = node_.declare_parameter<double>("puck_half_height_m", 0.03);
  grasp_offset_m_ = node_.declare_parameter<double>("grasp_offset_m", 0.015);

  nav_retries_ = node_.declare_parameter<int>("nav_retries", 1);
  // 경로 계획 실패 (박스 정차점 출발 등, 잠깐 막힘) → path_retry_wait_s 기다렸다 같은 목표 다시, 이동 단계마다 path_retry_total_s 까지
  //   nav_retries · 미션 재계획을 안 씀. 0 = 끔 (바로 단계 재시도 → 재계획)
  path_retry_total_s_ = node_.declare_parameter<double>("path_retry_total_s", 0.0);
  path_retry_wait_s_ = node_.declare_parameter<std::vector<double>>("path_retry_wait_s", path_retry_wait_s_);
  path_fail_quick_s_ = node_.declare_parameter<double>("path_fail_quick_s", path_fail_quick_s_);
  path_fail_still_m_ = node_.declare_parameter<double>("path_fail_still_m", path_fail_still_m_);
  pick_retries_ = node_.declare_parameter<int>("pick_retries", 2);
  pick_relook_ = node_.declare_parameter<bool>("pick_relook", true);
  relook_wait_s_ = node_.declare_parameter<double>("relook_wait_s", 1.5);
  // PICK 빈손 확인 (control 결과 글 EMPTY_GRASP) → 재고 불확실 FAULT 대신 그 퍽 건너뜀 (false = FAULT)
  empty_grasp_skip_ = node_.declare_parameter<bool>("empty_grasp_skip", true);
  // STORE 넣기 걸림 뒤 control 이 퍽을 박스에 되돌려 놓음 (결과 글 STORE_RETURNED_TO_BOX x y, control store_recover)
  //   → 재고 불확실 FAULT 대신 그리퍼 비움 · 그 퍽 박스 (그 자리) 로 되돌리고 단계 실패 → 미션이 다시 계획 (false = FAULT)
  store_return_replan_ = node_.declare_parameter<bool>("store_return_replan", true);
  // 재고 작업 (PICK · STORE · RETRIEVE · PLACE · LOOK_RACK) 이 움직인 뒤 실패 + 그리퍼 실측 빈손 (미션 hand reader, /joint_states)
  //   → FAULT 대신 StepFailed (reconcile) → 미션이 카메라 확인 (적재함 · 장소 LOOK) 으로 before · done · removed 를 고름, 안 맞으면 FAULT
  //   손 상태가 hand_wait_s 안에 안 정해짐 · 쥠 · 결과 못 받음 (시간 초과 등) 은 FAULT → 미션 스스로 정합 (inventory_auto) 이 이어받음
  auto_reconcile_ = node_.declare_parameter<bool>("auto_reconcile", true);
  hand_wait_s_ = node_.declare_parameter<double>("hand_wait_s", 2.0);
  // RETRIEVE 성공 결과 뒤 그리퍼 실측 (미션 hand reader) 확인: 빈손 = 못 꺼냄 → 실패 처리 (위 자동 정합 = 적재함 스캔으로 칸 확인)
  //   손 상태가 hand_wait_s 안에 안 정해짐 · 쥠 = 성공 그대로. control RETRIEVE 는 닫은 뒤 빈손 확인이 없어 빈손도 성공으로 끝남
  retrieve_hand_check_ = node_.declare_parameter<bool>("retrieve_hand_check", true);
  // 라이다 · 검출 · 위치 감시 FAULT 를 팔 작업 중 · 퍽 쥔 동안 미루는 최대 시간 (0 = 바로 FAULT)
  sensor_fault_hold_s_ = node_.declare_parameter<double>("sensor_fault_hold_s", 60.0);
  place_retries_ = node_.declare_parameter<int>("place_retries", 1);
  require_motion_mode_ = node_.declare_parameter<bool>("require_motion_mode", true);
  // ---- 손목 퍽 검출 (perception ~/wrist/enable) · 손목 박스 측정 (~/box/enable): 주행 목표 진행 중에만 끔 (CPU). "" = 안 건드림 (perception 설정 그대로)
  const auto wrist_srv = node_.declare_parameter<std::string>("wrist_enable_service", "");
  const auto box_srv = node_.declare_parameter<std::string>("box_enable_service", "");
  wrist_on_grace_s_ = node_.declare_parameter<double>("wrist_on_grace_s", 2.0);

  // ---- 재고: 칸 이름 (팔이 닿는 칸만), 기록 파일 (빈 문자열 = 저장 안 함)
  cargo_ = Cargo(node_.declare_parameter<std::vector<std::string>>("cargo_slots",
    {"S1", "S2", "S3", "S8", "S11", "S6", "S12", "S7", "S10", "S5", "S9", "S4"}));   // 뒷줄부터, 넣기 제약 순서
  {   // 칸 제약 "칸:비어야 할 칸" (config/rack_cells.yaml rack_constraints, 최악 검사 + 실물 S7 ← S12 −13 mm)
    auto rules = [&](const char * name, const std::vector<std::string> & def) {
        Cargo::SlotRules out;
        for (const auto & r : node_.declare_parameter<std::vector<std::string>>(name, def)) {
          if (r.empty()) {continue;}   // [""] = 규칙 없음 (yaml 빈 목록 [] 은 형을 못 정함)
          const auto k = r.find(':');
          if (k == std::string::npos || k == 0 || k + 1 == r.size()) {
            RCLCPP_WARN(logger(), "%s: '%s' 는 '칸:비어야 할 칸' 꼴이 아님 → 무시", name, r.c_str());
            continue;
          }
          out.emplace_back(r.substr(0, k), r.substr(k + 1));
        }
        return out;
      };
    cargo_.set_rules(rules("rack_store_needs_empty", {"S1:S2", "S9:S4", "S10:S5", "S11:S6", "S12:S7"}),
      rules("rack_retrieve_needs_empty", {"S1:S2", "S7:S12", "S10:S5"}));
  }
  cargo_file_ = node_.declare_parameter<std::string>("cargo_file", "");
  if (node_.declare_parameter<bool>("cargo_restore", false) && !cargo_file_.empty()) {
    std::string cerr;
    if (cargo_.load(cargo_file_, &cerr)) {
      RCLCPP_INFO(logger(), "재고 기록 복원: %s", cargo_file_.c_str());
    } else {
      RCLCPP_WARN(logger(), "재고 기록 복원 실패 (빈 상태로 시작): %s", cerr.c_str());
    }
  }

  // ---------------------------------------------------------------- TF
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_.get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // ---------------------------------------------------------------- 받기
  // 카메라마다 토픽 따로 (perception). 위치를 아는 손목만 기본으로 받음
  const auto det_topics = node_.declare_parameter<std::vector<std::string>>(
    "detection_topics", std::vector<std::string>{"/perception/wrist/detections"});
  if (!det_topics.empty()) {restart_topic_ = det_topics.front();}
  for (const auto & t : det_topics) {
    sub_det_.push_back(node_.create_subscription<DetectionArray>(
      t, 10, [this](DetectionArray::ConstSharedPtr m) {on_detections(*m);}));
  }
  // 손목 카메라 돌아감 (camera_drift_node, transient_local 1 개 · HOME 마다 갱신). "" = 안 씀 (재검증도 명목 TF)
  const auto drift_topic = node_.declare_parameter<std::string>("drift_topic", "/wrist_cam/drift");
  if (!drift_topic.empty()) {
    sub_drift_ = node_.create_subscription<geometry_msgs::msg::QuaternionStamped>(
      drift_topic, rclcpp::QoS(1).reliable().transient_local(),
      [this](geometry_msgs::msg::QuaternionStamped::ConstSharedPtr m) {
        const auto & q = m->quaternion;
        if (!valid_quaternion(q.x, q.y, q.z, q.w) || m->header.frame_id.empty()) {return;}
        drift_ = *m;
      });
  }
  // 정밀 접근 노드가 잰 정차점 (rl_dock --box_topic). "" = 정차점 보정표 안 씀 (내는 노드가 없으면 받는 것 없음)
  const auto dock_box_topic = node_.declare_parameter<std::string>("dock_box_topic", "/dock/box_measured");
  if (!dock_box_topic.empty() && !dock_action_.empty()) {
    sub_dock_box_ = node_.create_subscription<geometry_msgs::msg::PoseStamped>(
      dock_box_topic, 10, [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr m) {
        if (!nav_docking_ || !nav_.active) {return;}   // 이 실행기가 보낸 정밀 접근 중에만
        geometry_msgs::msg::PoseStamped out = *m;
        std::string why;
        if (m->header.frame_id != map_frame_ && !transform_pose(*m, map_frame_, out, &why)) {
          RCLCPP_WARN_THROTTLE(logger(), *node_.get_clock(), 5000, "잰 정차점 지도 변환 실패: %s", why.c_str());
          return;
        }
        const auto & q = out.pose.orientation;
        dock_meas_ = Pose2D{out.pose.position.x, out.pose.position.y,
          std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))};
        ++dock_meas_n_;
      });
  }
  sub_mode_ = node_.create_subscription<MotionMode>(
    "/control/motion_mode", rclcpp::QoS(1).reliable().transient_local(),
    [this](MotionMode::ConstSharedPtr m) {
      have_mode_ = true;
      mode_ = m->mode;
      mode_ready_ = m->ready;
      mode_reason_ = m->reason;
      last_mode_s_ = now_s();
      const auto u = m->reason.find(kPickUnreachableToken);
      if (u != std::string::npos) {   // "PICK_UNREACHABLE puck=<ID>: …"
        const auto b = u + std::string(kPickUnreachableToken).size();
        const auto e = m->reason.find(':', b);
        unreachable_s_ = last_mode_s_;
        unreachable_puck_ = m->reason.substr(b, e == std::string::npos ? std::string::npos : e - b);
        unreachable_why_ = m->reason;
      }
    });
  sub_odom_ = node_.create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10, [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
      last_odom_s_ = now_s();
      const rclcpp::Time stamp(m->header.stamp, RCL_ROS_TIME);
      const auto & p = m->pose.pose.position;
      const auto & q = m->pose.pose.orientation;
      // 새 측정만 (같은 · 지난 시각 · NaN · 깨진 자세는 안 셈). 미래 시각도 버림: 한 번 받으면 뒤의 정상 측정이 다 '지난 시각' 이 됨
      if (stamp > odom_stamp_ && (stamp - node_.now()).seconds() <= kStampFutureS &&
        std::isfinite(p.x) && std::isfinite(p.y) && valid_quaternion(q.x, q.y, q.z, q.w))
      {
        odom_stamp_ = stamp;
        odom_x_ = p.x;
        odom_y_ = p.y;
        odom_yaw_ = yaw_from_quaternion(q.x, q.y, q.z, q.w);
      }
    });
  last_scan_s_.assign(scan_topics_.size(), -1.0);
  for (std::size_t i = 0; i < scan_topics_.size(); ++i) {
    sub_scan_.push_back(node_.create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topics_[i], rclcpp::SensorDataQoS(), [this, i](sensor_msgs::msg::LaserScan::ConstSharedPtr) {last_scan_s_[i] = now_s();}));
  }
  // AMCL 은 위치를 갱신할 때만 발행 (정지 중엔 안 옴) → 마지막 유효 공분산을 유지
  //   /initialpose 는 따로 받지 않음: AMCL 이 초기 위치를 반영한 /amcl_pose 를 스스로 다시 내므로 마지막 값만 믿음
  //   (/initialpose 로 지우면 AMCL 이 다시 안 낼 때 정지 중 LOCALIZATION_COV 로 계속 막힘)
  if (loc_from_amcl_) {
    sub_amcl_ = node_.create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/amcl_pose", rclcpp::QoS(1).reliable().transient_local(),
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr m) {
        pose_std_ = covariance_std(m->pose.covariance.data());
      });
  } else {
    // pose_guard 가 5 Hz 로 계속 냄 (SLAM · 라이다 · IMU 비교). level 2 이상 = 정지, 끊겨도 상실
    sub_loc_health_ = node_.create_subscription<diagnostic_msgs::msg::DiagnosticStatus>(
      node_.declare_parameter<std::string>("localization_health_topic", "/localization_health"), 10,
      [this](diagnostic_msgs::msg::DiagnosticStatus::ConstSharedPtr m) {
        loc_health_.level = static_cast<int>(m->level);
        loc_health_.received_s = now_s();
      });
  }

  // ---------------------------------------------------------------- 보내기 · 액션
  pub_cargo_ = node_.create_publisher<CargoState>("/mission/cargo", 10);
  pub_markers_ = node_.create_publisher<visualization_msgs::msg::MarkerArray>("/planning/markers", 10);
  nav_client_ = rclcpp_action::create_client<Nav>(&node_, "navigate_to_pose");
  if (!dock_action_.empty()) {dock_client_ = rclcpp_action::create_client<Nav>(&node_, dock_action_);}
  nav_cur_client_ = nav_client_;
  arm_client_ = rclcpp_action::create_client<ArmTask>(&node_, "/arm_task");
  wrist_tg_.name = "손목 검출";
  box_tg_.name = "손목 박스 측정";
  if (!wrist_srv.empty()) {wrist_tg_.cli = node_.create_client<SetBool>(wrist_srv);}
  if (!box_srv.empty()) {box_tg_.cli = node_.create_client<SetBool>(box_srv);}
  // perception 세션: 프로세스마다 다른 글, transient_local → 바뀌면 respawn
  {
    const auto topic = node_.declare_parameter<std::string>("perception_session_topic", "/perception/session");
    if (!topic.empty()) {
      sub_perc_session_ = node_.create_subscription<std_msgs::msg::String>(topic, rclcpp::QoS(1).transient_local().reliable(),
          [this](std_msgs::msg::String::ConstSharedPtr m) {on_perception_session(m->data);});
    }
  }

  // Nav2 lifecycle_manager is_active (std_srvs/Trigger) 를 1 Hz 로 물어봄
  for (const auto & name : lifecycle_services) {
    LifecycleCheck lc;
    lc.name = name;
    lc.cli = node_.create_client<Trigger>(name);
    lifecycle_.push_back(lc);
  }
  lifecycle_timer_ = node_.create_wall_timer(std::chrono::seconds(1), [this]() {poll_lifecycle();});
  cargo_timer_ = node_.create_wall_timer(std::chrono::seconds(1), [this]() {publish_cargo();});
  // RViz 용이라 2 Hz 면 충분 (매번 DELETEALL 후 다시 보냄)
  marker_timer_ = node_.create_wall_timer(std::chrono::milliseconds(500), [this]() {publish_markers();});
}

// ================================================================ 받기 처리

// 퍽 검출 → 촬영 시각 기준 TF 로 변환해 세 곳에 기록
//   - map : 퍽 기록부 (오래 유지) + 원시 관측 (최근 30 s, 재검증용)
//   - odom: ID 있는 퍽만, 집을 때 target 용 (AMCL 보정에 안 흔들림, pick_fresh_s 안에서만 씀)
//   - TF 조회는 (프레임, 시각) 이 바뀔 때만 함. 보통 한 메시지 = 영상 1장이라 메시지당 1번
void Executor::on_detections(const DetectionArray & msg)
{
  const double now = now_s();
  last_any_det_s_ = now;                    // 빈 배열도 "출력 살아 있음"
  last_det_s_[msg.header.frame_id] = now;   // 카메라별 생존 (한쪽 장애를 다른 쪽이 가리지 않게)
  TfCache to_map, to_odom;
  for (const auto & d : msg.detections) {
    if (d.kind != "puck" || d.confidence < min_confidence_) {continue;}
    const auto & p = d.pose.position;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {continue;}
    const auto & h = d.header.frame_id.empty() ? msg.header : d.header;
    if (h.frame_id.empty()) {continue;}                                   // 기준 프레임 없음 → 버림
    if (now - rclcpp::Time(h.stamp).seconds() > detection_max_age_s_) {continue;}   // 오래됨 → 버림

    const auto * tm = lookup_cached(to_map, map_frame_, h, 0.05);
    if (!tm) {continue;}
    geometry_msgs::msg::Point out;   // 기록부는 위치만 씀
    tf2::doTransform(p, out, *tm);

    PuckRecord r;
    r.id = d.id;
    r.color = d.label;
    r.shape = d.shape;
    r.x = out.x;
    r.y = out.y;
    r.z = out.z;
    r.last_seen = now;
    // 이 퍽을 다루는 팔 작업 중 (PICK 받침 · 손 안 등, LOOK 빼고) 검출은 색 · 모양 표를 안 셈 = 위치만.
    //   손목이 가까이 · 비스듬히 본 원기둥을 box 로 읽은 한 장이 적재 기록이 될 수 있음
    const bool handling = step_i_ < steps_.size() && steps_[step_i_].type == StepType::Arm &&
      !is_look_task(steps_[step_i_].arm_task) && !d.id.empty() && steps_[step_i_].puck_id == d.id;
    registry_.update(r, !handling);   // id 가 "" 이면 후보로 들어감
    // 재검증용 원시 관측: 그 카메라가 돈 만큼 되돌린 위치 (카메라 좌표 p → R · p 뒤 명목 TF, control_master 와 같은 식)
    geometry_msgs::msg::Point obs = out;
    if (drift_ && drift_->header.frame_id == h.frame_id) {
      const auto & dq = drift_->quaternion;
      const tf2::Vector3 pr = tf2::quatRotate(tf2::Quaternion(dq.x, dq.y, dq.z, dq.w).normalized(), tf2::Vector3(p.x, p.y, p.z));
      geometry_msgs::msg::Point pc;
      pc.x = pr.x();
      pc.y = pr.y();
      pc.z = pr.z();
      tf2::doTransform(pc, obs, *tm);
    }
    {
      ObsRecord rec{rclcpp::Time(h.stamp).seconds(), d.id, d.label, d.shape, obs.x, obs.y,
        {d.bbox[0], d.bbox[1], d.bbox[2], d.bbox[3]}};
      rec.z = obs.z;   // 깊이가 틀리면 광선 따라 밀려 윗면 높이에서 벗어남 (재검증이 봄)
      rec.cam = {tm->transform.translation.x, tm->transform.translation.y, tm->transform.translation.z};   // 카메라 원점 (map)
      raw_obs_.push_back(std::move(rec));
    }
    while (!raw_obs_.empty() && raw_obs_.front().stamp < now - 30.0) {raw_obs_.pop_front();}

    if (d.id.empty()) {continue;}
    const auto * to = lookup_cached(to_odom, odom_frame_, h, 0.0);   // map 을 받았으면 odom 은 이미 있음
    if (!to) {continue;}
    const auto & q = d.pose.orientation;
    LocalSeen ls;
    ls.has_orientation = valid_quaternion(q.x, q.y, q.z, q.w);
    ls.yaw_known = d.yaw_known;
    ls.is_box = d.shape == "box";
    ls.pose.header.frame_id = odom_frame_;
    ls.pose.header.stamp = h.stamp;
    if (ls.has_orientation) {
      tf2::doTransform(d.pose, ls.pose.pose, *to);
    } else {
      tf2::doTransform(p, ls.pose.pose.position, *to);
    }
    ls.seen = now;
    // 누움 판정: 방향 있는 검출로만, 방향 없는 검출이 와도 앞 판정을 이어 받음 (실물: 같은 퍽이 방향 있음 · 없음을 오감)
    if (ls.has_orientation) {
      const auto & oq = ls.pose.pose.orientation;
      const auto & op = ls.pose.pose.position;
      ls.oriented_s = now;
      ls.lying = !planar_from_pose(true, Quat{oq.x, oq.y, oq.z, oq.w}, Vec3{op.x, op.y, op.z}, ls.yaw_known, ls.is_box).upright;
    } else if (const auto prev = local_seen_.find(d.id); prev != local_seen_.end()) {
      ls.oriented_s = prev->second.oriented_s;
      ls.lying = prev->second.lying;
    }
    local_seen_[d.id] = ls;
  }
}

const geometry_msgs::msg::TransformStamped * Executor::lookup_cached(
  TfCache & c, const std::string & target, const std_msgs::msg::Header & h, double wait_s)
{
  if (c.header.frame_id != h.frame_id || c.header.stamp != h.stamp) {
    c.header = h;
    try {
      c.tf = tf_buffer_->lookupTransform(
        target, h.frame_id, tf2_ros::fromMsg(h.stamp), tf2::durationFromSec(wait_s));
    } catch (const tf2::TransformException & e) {
      c.tf.reset();
      RCLCPP_WARN_THROTTLE(logger(), *node_.get_clock(), 2000, "검출 좌표 변환 실패 (%s): %s",
        target.c_str(), e.what());
    }
  }
  return c.tf ? &*c.tf : nullptr;
}

// ================================================================ 주기

void Executor::update(double now)
{
  registry_.prune_candidates(now, candidate_max_age_s_);
  // 취소 결과 기다리는 중 서버가 없어짐 → 그 목표도 없어짐 (다시 떠도 결과 안 옴, 안 지우면 resume · 새 목표가 계속 막힘)
  //   팔 시간 초과 대기 중이면 run_arm 이 결과 없음으로 처리 (on_fail → FAULT)
  if (arm_cancel_pending_ && !arm_timeout_wait_ && !arm_client_->action_server_is_ready()) {
    RCLCPP_WARN(logger(), "/arm_task 서버 없어짐 → 취소한 팔 작업 결과 안 기다림");
    arm_cancel_pending_ = false;
  }
  if (nav_cancel_pending_ && !nav_client_->action_server_is_ready()) {
    RCLCPP_WARN(logger(), "Nav2 서버 없어짐 → 취소한 이동 결과 안 기다림");
    nav_cancel_pending_ = false;
  }
  // 취소한 팔 작업이 결국 성공 → 재고 · 기록부에 반영 (상태 무관: FAULT 뒤 그리퍼 기록이 비는 것 방지)
  //   그 작업이 재고 불확실 원인이면 control 의 최종 성공 = 실물 확인으로 보고 표시도 지움 (전환과 함께 한 번에, 반영은 여기 한 번만)
  //   기록과 안 맞으면 반영 · 해소 안 함 → 불확실 표시 (이미 있으면 그대로)
  if (late_arm_ok_ && !arm_timeout_wait_) {   // 시간 초과 대기 중이면 run_arm 이 처리
    const Step late = *late_arm_ok_;
    late_arm_ok_.reset();
    RCLCPP_WARN(logger(), "취소한 팔 작업이 성공으로 끝남 → 재고 반영: %s", late.name.c_str());
    std::string why;
    if (!update_cargo(late, why)) {mark_doubt(late, "늦은 최종 성공인데 재고 기록 불일치: " + why);}
  }
  watch_perception_restart(now);
  sync_nav_toggle(wrist_tg_, now);
  sync_nav_toggle(box_tg_, now);
}

// perception 재시작 감지 (perception 은 멈추면 스스로 abort → launch respawn 이 다시 띄움)
//   - 다시 뜬 perception 은 켜고 끄기를 yaml 기본값으로 시작 → 여기서 맞춰 둔 상태 (known) 를 버리고 다시 보내야 함
//   - 서비스 있음 · 없음으로는 못 가름: 죽은 프로세스의 DDS 참여자가 lease 동안 (Fast DDS 기본 20 s) 그래프에 남아
//     서비스가 끊기지 않은 것처럼 보임 → 검출 토픽 발행자 GID 가 처음 보는 것이면 새 프로세스로 봄
//   - 매번 다시 보내는 방식은 안 씀: perception set_enabled 가 부를 때마다 추적기를 비움
void Executor::watch_perception_restart(double now)
{
  if (restart_topic_.empty() || now - restart_check_s_ < 1.0) {return;}
  restart_check_s_ = now;
  bool fresh = false;
  for (const auto & e : node_.get_publishers_info_by_topic(restart_topic_)) {
    fresh = restart_gids_.insert(e.endpoint_gid()).second || fresh;
  }
  if (!fresh) {return;}
  if (!restart_seen_) {   // 처음 본 발행자 (시작 때) = 재시작 아님
    restart_seen_ = true;
    return;
  }
  if (now - perc_resync_s_ < 5.0) {   // 같은 재시작을 세션 (/perception/session) 으로 이미 맞춤 → 켜기 또 안 보냄 (perception 은 켤 때마다 추적을 비움)
    RCLCPP_INFO(logger(), "perception 새 발행자 (%s) — %.1f s 전 세션으로 이미 다시 맞춤 → 그대로", restart_topic_.c_str(), now - perc_resync_s_);
    return;
  }
  perc_resync_s_ = now;
  ++perception_restarts_;
  wrist_tg_.known = false;
  box_tg_.known = false;
  RCLCPP_WARN(logger(), "perception 새 발행자 (%s) → 재시작으로 봄 (%d 번째), 손목 검출 · 박스 측정 켜고 끄기 다시 맞춤",
    restart_topic_.c_str(), perception_restarts_);
}

// perception 손목 단계를 주행에 맞춤: Nav2 · 정밀 접근 목표가 진행 중이면 끔, 그 밖 (도착 뒤 · 팔 작업 · 대기 · FAULT) 은 켬
//   - 주행 중 손목 카메라는 퍽 · 박스를 안 봄 (팔 접음), 분할 · 박스 맞춤이 CPU 를 씀. control_master 는 LOOK 뒤 검출을 10 s 쓰고 그 사이 차체는 안 움직임
//   - 박스 측정은 미션이 장소 LOOK 시작 뒤 것만 씀 → 주행 중 측정은 안 쓰임
//   - 바코드 판독은 손목 컬러 구독이 따로라 이 둘과 무관 (~/barcode/enable)
//   - 손목 검출 끈 동안 · 켠 뒤 wrist_on_grace_s 는 검출 출력 끊김 (DETECTION_LOST) 으로 안 셈 (wrist_quiet)
//   - 처음 서비스가 보이면 지금 상태로 한 번 맞춤. 응답 없으면 2 s 뒤 버리고 다시
void Executor::sync_nav_toggle(NavToggle & t, double now)
{
  if (!t.cli) {return;}
  const bool want = !nav_.active;
  if (t.pending) {
    if (now - t.sent_s < 2.0) {return;}
    t.cli->prune_pending_requests();
    t.pending = false;
    RCLCPP_WARN(logger(), "%s 켜고 끄기 서비스 응답 없음 → 다시 보냄", t.name.c_str());
  }
  if (t.known && t.on == want) {return;}
  if (now - t.last_try_s < (want ? 0.2 : 1.0)) {return;}
  t.last_try_s = now;
  if (!t.cli->service_is_ready()) {return;}
  auto req = std::make_shared<SetBool::Request>();
  req->data = want;
  const int gen = ++t.gen;
  t.pending = true;
  t.sent_s = now;
  if (want) {t.on_s = now;}   // 켜는 요청부터 grace (응답 · 첫 장까지)
  t.cli->async_send_request(req, [this, &t, want, gen](rclcpp::Client<SetBool>::SharedFuture f) {
      if (gen != t.gen) {return;}
      t.pending = false;
      if (f.get()->success) {
        t.known = true;
        t.on = want;
        if (want) {t.on_s = now_s();}
        RCLCPP_INFO(logger(), "perception %s %s", t.name.c_str(), want ? "켬" : "끔 (주행 중)");
      }
    });
}

// perception 세션 글이 바뀜 = perception 프로세스가 다시 뜸 (멈춤 감시 → respawn, 사람이 다시 켬 등)
//   perception 은 같은 launch 의 respawn 이면 켜기 상태를 스스로 되살리지만 (restart_state.hpp), 새 launch · 파일 못 씀 대비로
//   켜기 서비스를 지금 원하는 값으로 다시 보냄. 검출 감시는 새로 (이 끊김을 회복 횟수 monitor_max_recoveries 로 안 셈)
void Executor::on_perception_session(const std::string & s)
{
  if (s == perc_session_) {return;}
  const std::string old = perc_session_;
  perc_session_ = s;
  if (old.empty()) {
    RCLCPP_INFO(logger(), "perception 세션: %s", s.c_str());
    return;
  }
  const double now = now_s();
  if (now - perc_resync_s_ < 5.0) {   // 같은 재시작을 발행자 GID 로 이미 맞춤 (watch_perception_restart) → 켜기 또 안 보냄, 검출 감시만 새로
    RCLCPP_INFO(logger(), "perception 세션 바뀜 (%s → %s) — %.1f s 전 발행자로 이미 다시 맞춤 → 검출 감시만 새로", old.c_str(), s.c_str(),
      now - perc_resync_s_);
    det_mon_.reset();
    return;
  }
  perc_resync_s_ = now;
  ++perc_restarts_;
  RCLCPP_WARN(logger(), "perception 다시 켜짐 (%d 번째: %s → %s) → 손목 검출 · 박스 측정 켜기 다시 보냄, 검출 감시 새로", perc_restarts_,
    old.c_str(), s.c_str());
  for (NavToggle * t : {&wrist_tg_, &box_tg_}) {
    t->known = false;
    t->last_try_s = -1e9;
  }
  det_mon_.reset();
  if (perc_restart_hook_) {perc_restart_hook_(old, s);}
}

// 손목 검출을 일부러 끈 동안 · 켠 직후 → 검출 출력 끊김을 장애로 안 봄
bool Executor::wrist_quiet(double now) const
{
  const auto & t = wrist_tg_;
  if (!t.cli) {return false;}
  if (t.pending) {return true;}
  if (t.known && !t.on) {return true;}
  return t.on_s > 0.0 && now - t.on_s < wrist_on_grace_s_;
}

// 제어 끊김 (STANDBY 에서 수신을 확인했으므로 여기선 끊김만 봄) · 센서 · 위치 감시
std::optional<ExecEvent> Executor::supervise(double now)
{
  auto fault = [](const std::string & code, const std::string & why) {
      ExecEvent ev;
      ev.kind = ExecEvent::Kind::Fault;
      ev.code = code;
      ev.detail = why;
      return ev;
    };
  if (now - last_odom_s_ > control_timeout_s_) {return fault("ODOM_LOST", "/odom 끊김");}
  if (require_motion_mode_ && now - last_mode_s_ > control_timeout_s_) {
    return fault("CONTROL_LOST", "/control/motion_mode 끊김");
  }
  // 감시 FAULT (라이다 · 검출 · 위치) 는 팔 작업 중이거나 퍽을 쥐고 있으면 미룸 (최대 sensor_fault_hold_s):
  //   서서 하는 팔 작업 · 이어지는 STORE 는 이 입력 없이도 됨 (arm_start_blocked 가 STORE · RETRIEVE · LOOK · HOME 은 허용).
  //   바로 FAULT 면 stop() 이 팔을 도중에 취소 → 재고 불확실 + 팔 경로 중간 (HOME 거절), 쥔 채면 HOME · start 거절 → 사람 복구.
  //   새 이동 · 지도 의존 팔 목표 (PICK · PLACE) 는 대기 원인으로 계속 막힘. 그리퍼가 비고 팔이 쉬면 다음 주기에 FAULT
  const bool hold_now = arm_busy() || cargo_.holding();
  auto mon_fault = [&](const char * code, const std::string & why) -> std::optional<ExecEvent> {
      if (sensor_fault_since_ < 0.0) {sensor_fault_since_ = now;}
      if (hold_now && now - sensor_fault_since_ < sensor_fault_hold_s_) {
        RCLCPP_WARN_THROTTLE(logger(), *node_.get_clock(), 2000, "%s → 지금 팔 작업 · 쥔 퍽 (%s) 정리 뒤 FAULT (최대 %.0f s)", why.c_str(),
          step_i_ < steps_.size() ? steps_[step_i_].name.c_str() : "?", sensor_fault_hold_s_);
        return std::nullopt;
      }
      return fault(code, why);
    };
  const auto stale_scans = require_scan_ ? stale_scan_topics(now) : std::vector<std::string>{};
  if (scan_mon_.update(!stale_scans.empty(), now) == Health::Fault) {
    std::string which;
    for (const auto & t : stale_scans) {which += (which.empty() ? "" : ", ") + t;}
    if (auto f = mon_fault("SCAN_LOST", "라이다 끊김 회복 안 됨: " + which)) {return f;}
  }
  const auto stale_det = require_detections_ && !wrist_quiet(now) ? stale_detection_frames(now) : std::vector<std::string>{};
  const bool det_bad = !stale_det.empty();
  if (const int g = det_gap_.update(det_bad, now); g > 0) {
    std::string which;
    for (const auto & f : stale_det) {which += (which.empty() ? "" : ", ") + f;}
    RCLCPP_WARN(logger(), "손목 검출 끊김 (%s, %.1f s 넘게 없음) → 새 LOOK 은 회복 뒤 · LOOK 결과 쓰는 단계는 회복 뒤 LOOK 다시, %.0f s 안에 안 돌아오면 FAULT",
      which.c_str(), detection_timeout_s_, detection_fault_s_);
  } else if (g < 0) {
    RCLCPP_WARN(logger(), "손목 검출 회복: 끊김 %.1f s (perception 다시 켜짐 %d 번)", det_gap_.ended() - det_gap_.started(), perc_restarts_);
  }
  if (det_mon_.update(det_bad, now) == Health::Fault) {
    if (auto f = mon_fault("DETECTION_LOST", "검출 출력 끊김 회복 안 됨")) {return f;}
  }
  if (loc_recovery_) {
    update_loc_recovery(now);   // FAULT 없음
  } else if (loc_mon_.update(require_localization_ && localization_lost(now), now) == Health::Fault) {
    if (auto f = mon_fault("LOCALIZATION_LOST", "위치 추정 상실")) {return f;}
  }
  // 실행 중 재고 불확실 표시가 생김 (늦은 최종 성공이 기록과 안 맞음 등) → 더 조작하지 않음
  if (cargo_.doubt() && !reconciling_) {
    return fault("INVENTORY_UNCERTAIN", "재고 불확실: " + doubt_text() + " → 실물 확인 뒤 /mission/inventory_* 로 정합");
  }
  return std::nullopt;
}

ExecEvent Executor::step(double now)
{
  ExecEvent ev;
  // HOME 뒤 DRIVE 대기
  if (waiting_drive_) {
    if (!require_motion_mode_ || (have_mode_ && mode_ == MotionMode::DRIVE)) {
      waiting_drive_ = false;
      ev.kind = ExecEvent::Kind::DriveReady;
    } else if (now - waiting_drive_since_ > drive_wait_s_) {
      ev.kind = ExecEvent::Kind::Fault;
      ev.code = "DRIVE_NOT_RESTORED";
      ev.detail = "HOME 뒤 DRIVE 로 안 바뀜: " + mode_reason_;
    }
    return ev;
  }
  if (rec_pending_ || rec_nav_active_) {return ev;}   // 위치 복구 물러나기 중: 단계 진행 안 함 (Nav2 목표를 같이 씀)
  if (step_i_ >= steps_.size()) {return ev;}
  if (hand_wait_until_ > 0.0) {   // 재고 작업 실패 뒤 손 상태 기다리는 중 → 그 실패를 다시 판단 (단계는 안 돌림)
    on_fail(ev);
    return ev;
  }

  // 대기 원인으로 막힌 단계는 시작 · 진행 안 함
  const Step & cur = steps_[step_i_];
  const bool drive_step = cur.type == StepType::Navigate;
  // 퍽 쥔 채 이동 단계 = 계획 오류 (쥔 채 주행 안 함, control 도 쥔 채 HOME 거절) → 이동 안 하고 FAULT
  if (drive_step && !step_running_ && cargo_.holding()) {
    ev.kind = ExecEvent::Kind::Fault;
    ev.code = "CARRY_DRIVE";
    ev.detail = "퍽 " + cargo_.gripper()->puck_id + " 쥔 채 이동 단계: " + cur.name + " (STORE · PLACE 로 그리퍼를 비운 뒤 이동해야 함)";
    return ev;
  }
  // 이미 DRIVE 면 접힌 것 (control 이 P0 에서 쉬다 스스로 접음 등) → HOME 안 끼움
  if (drive_step && arm_unfolded_ && have_mode_ && mode_ == MotionMode::DRIVE) {arm_unfolded_ = false;}
  // LOOK 류 · lazy_fold 의 STORE · PLACE 뒤 팔이 펴진 채 이동하려 함 → 기다리지 않고 바로 HOME (control 은 접힘 실측 뒤에만 DRIVE)
  if (drive_step && arm_unfolded_ && !step_running_ && require_motion_mode_) {
    arm_unfolded_ = false;
    insert_home_at_current("팔 작업 뒤 이동 전 접기");
    return ev;
  }
  if (update_drive_reason(now, drive_step, ev)) {return ev;}   // HOME 끼워 넣음 (또는 Fault)
  if (drive_step && nav_blocked()) {
    stop_blocked_nav();
    return ev;
  }
  // 새 팔 목표 보내기 전만 막음 (거절 뒤 다시 보내기 포함). 보낸 작업은 그대로 두고 supervise 가 회복 · FAULT 판단
  if (cur.type == StepType::Arm && !step_running_ &&
    arm_start_blocked(cur.arm_task, cur.target, det_mon_.active(), loc_blocks()))
  {
    return ev;
  }
  // 손목 검출 끊김 중 (perception 멈춤 · respawn) 새 LOOK 은 안 보냄 — 본 것이 없어 어차피 다시 해야 함. 회복되면 바로 보냄
  //   (안 돌아오면 감시가 detection_fault_s 뒤 DETECTION_LOST FAULT)
  if (cur.type == StepType::Arm && !step_running_ && is_look_task(cur.arm_task) && det_gap_.active()) {
    RCLCPP_WARN_THROTTLE(logger(), *node_.get_clock(), 3000, "%s: 손목 검출 끊김 %.1f s → 회복 (perception respawn) 뒤 LOOK", cur.name.c_str(),
      now - det_gap_.started());
    return ev;
  }

  // 현재 단계 실행
  pending_after_.clear();
  last_placed_.reset();
  arm_end_ = ArmEnd::NotSent;
  preflight_refused_ = false;
  const StepResult r = run_step(cur, now);
  if (r != StepResult::Running) {
    arm_try_start_ = -1.0;
    server_wait_start_ = -1.0;
  }
  if (r == StepResult::Success) {
    step_running_ = false;
    if (steps_[step_i_].type == StepType::Navigate) {   // 끼운 HOME 등 다른 단계 성공은 기다림 · 셈을 안 지움
      if (path_retry_.n > 0 && path_retry_.name == steps_[step_i_].name) {
        RCLCPP_INFO(logger(), "%s: 경로 계획 실패 뒤 기다렸다 다시 → 도착 (기다림 %d 번, 첫 실패부터 %.0f s)",
          steps_[step_i_].name.c_str(), path_retry_.n, now - path_retry_.since);
      }
      path_retry_reset();
    }
    ev.kind = ExecEvent::Kind::StepOk;
    ev.step = steps_[step_i_];
    ev.placed = last_placed_;
    if (!pending_after_.empty()) {   // Custom 단계가 만든 다음 단계 (바로 뒤)
      insert_steps(step_i_ + 1, pending_after_);
      pending_after_.clear();
    }
    ++step_i_;
    ev.all_done = step_i_ >= steps_.size();
  } else if (r == StepResult::Fail) {
    step_running_ = false;
    pending_after_.clear();
    on_fail(ev);
    if (ev.kind != ExecEvent::Kind::None) {path_retry_reset();}   // 단계 실패 · FAULT 로 미션에 넘김 (재시도면 그대로 셈)
  }
  return ev;
}

// 단계 실패 → 재시도 또는 StepFailed 사건 (순번은 그 단계에 그대로). HOME 실패 · 재고 불확실은 Fault 사건
void Executor::on_fail(ExecEvent & ev)
{
  const Step & s = steps_[step_i_];
  // HOME 실패 = 팔을 접을 수 없음 → 바퀴도 못 풂
  if (s.type == StepType::Arm && s.arm_task == kArmHome) {
    ev.kind = ExecEvent::Kind::Fault;
    ev.code = "HOME_FAILED";
    ev.detail = "HOME 실패: " + detail_;
    return;
  }
  // 보내기 전 재고 확인 실패 → 팔 명령 안 보냄. 계획 · 기록 중 무엇이 맞는지 모름 → 재시도 · 재계획 없이 FAULT
  if (preflight_refused_) {
    ev.kind = ExecEvent::Kind::Fault;
    ev.code = "INVENTORY_MISMATCH";
    ev.detail = detail_;
    return;
  }
  // PICK 빈손 확인 (control 이 그리퍼 실측으로 확인, 결과 글 EMPTY_GRASP) → 퍽은 박스에 그대로 · 그리퍼 빔 = 재고 확실
  //   불확실 FAULT · 재시도 없이 StepFailed → 미션이 그 퍽 skipped + 재계획 (dice_recover). 팔은 움직였으므로 HOME 먼저
  //   (control 이 스스로 접는 중이면 ready 를 기다리거나 거절 뒤 다시 보냄, 이미 접혔으면 HOME 은 바로 성공)
  if (empty_grasp_skip_ && s.type == StepType::Arm && empty_grasp_confirmed(s.arm_task, arm_end_, arm_.message)) {
    RCLCPP_WARN(logger(), "빈손 PICK: 퍽 %s 박스에 그대로 → 이번엔 건너뜀 (재고 그대로, HOME 먼저)", s.puck_id.c_str());
    ev.kind = ExecEvent::Kind::StepFailed;
    ev.step = s;
    ev.detail = detail_;
    ev.need_home = true;
    return;
  }
  // STORE 넣기 걸림 → control 이 퍽을 박스에 되돌려 놓음 (STORE_RETURNED_TO_BOX x y) → 그리퍼 빔 · 퍽 박스 그 자리 = 재고 확실
  //   불확실 FAULT · 재시도 없이 StepFailed → 미션이 다시 계획 (같은 퍽 두 번이면 건너뜀, dice_recover). 팔은 움직였으므로 HOME 먼저
  if (s.type == StepType::Arm && store_returned_to_box(s, arm_end_, arm_.message)) {
    ev.kind = ExecEvent::Kind::StepFailed;
    ev.step = s;
    ev.detail = detail_;
    ev.need_home = true;
    return;
  }
  // 재고를 바꾸는 팔 작업이 수락된 뒤 성공으로 안 끝남 → 퍽 위치 모름 → 재시도 · 재계획 없이 FAULT
  //   (재시도하면 쥔 퍽을 떨구거나 같은 퍽을 두 번 집을 수 있음, 미션이 lost 로 단정하면 기록이 실물과 어긋남)
  if (s.type == StepType::Arm && inventory_doubt_after(s.arm_task, arm_end_)) {
    if (hand_wait_until_ < 0.0) {mark_doubt(s, detail_);}   // 손 상태 기다리다 다시 부를 때는 이미 표시함
    // 실측 빈손 → 퍽은 박스 · 칸 · 장소 어딘가 (손에는 없음) → 미션이 카메라로 확인해 정합 (그동안 표시 FAULT 미룸)
    if (auto_reconcile_ && arm_end_ == ArmEnd::Failed && s.arm_task == kArmRetrieve &&
      arm_.message.find(kEmptyGraspToken) != std::string::npos)
    {   // control 이 RETRIEVE 끝 그리퍼 빈손을 확인 (EMPTY_GRASP) → 손 상태 안 기다리고 자동 정합
      hand_wait_until_ = -1.0;
      reconciling_ = true;
    } else if (auto_reconcile_ && arm_end_ == ArmEnd::Failed && hand_reader_) {
      const double now = now_s();
      std::string hw;
      const HandGuess g = hand_reader_(now, hw);
      if (g == HandGuess::Unknown && hand_wait_s_ > 0.0 && (hand_wait_until_ < 0.0 || now < hand_wait_until_)) {
        if (hand_wait_until_ < 0.0) {
          hand_wait_until_ = now + hand_wait_s_;
          reconciling_ = true;   // 기다리는 동안 불확실 표시 FAULT 미룸
          RCLCPP_INFO(logger(), "재고 불확실 %s: 손 상태 기다림 (최대 %.1f s, %s)", s.name.c_str(), hand_wait_s_, hw.c_str());
        }
        return;   // step() 이 다음 주기에 다시 부름
      }
      hand_wait_until_ = -1.0;
      reconciling_ = g == HandGuess::Empty;
      if (!reconciling_) {detail_ += " / 손: " + hw;}   // 쥠 · 모름 → FAULT (미션 스스로 정합이 이어받음)
    }
    if (reconciling_) {
      RCLCPP_WARN(logger(), "재고 불확실 %s: 그리퍼 실측 빈손 → FAULT 대신 자동 정합 (미션이 확인)", s.name.c_str());
      ev.kind = ExecEvent::Kind::StepFailed;
      ev.step = s;
      ev.detail = detail_;
      ev.need_home = true;
      ev.reconcile = true;
      return;
    }
    ev.kind = ExecEvent::Kind::Fault;
    ev.code = "INVENTORY_UNCERTAIN";
    ev.detail = "재고 불확실: " + s.name + " (" + detail_ + ") → 실물 확인 뒤 /mission/inventory_* 로 정합";
    return;
  }
  // 시간 초과로 취소한 팔 작업의 최종 결과가 끝내 안 옴 (server_wait_s 넘음 · 서버 없어짐) → 결국 성공했을 수 있음
  //   재시도 · 다음 단계로 가면 같은 작업을 또 함 (늦은 성공은 update() 가 재고에만 반영) → FAULT, 사람 확인
  if (s.type == StepType::Arm && arm_cancel_pending_) {
    ev.kind = ExecEvent::Kind::Fault;
    ev.code = "ARM_NO_RESULT";
    ev.detail = detail_;
    return;
  }
  // 팔이 움직이다 성공으로 안 끝남 → 접혔는지 모름, 바퀴 잠김 → 다음 단계 전에 HOME
  const bool need_home = arm_needs_home_;
  const int allowed = retries_allowed(s);
  // '퍽 높이 이상' 거절은 같은 검출로 다시 해도 같음 → 단계 재시도 없이 미션으로 (미션이 LOOK 을 다시 하고 새 검출로 집음)
  const bool same_again = s.type == StepType::Arm && s.arm_task == kArmPick && detail_.find(kPickHeightToken) != std::string::npos;
  if (!same_again && retries_[step_i_] < allowed) {
    const int n = ++retries_[step_i_];
    RCLCPP_WARN(logger(), "단계 재시도 %d/%d: %s (%s)", n, allowed, s.name.c_str(), detail_.c_str());
    // PICK 실패: 퍽 앞 이동부터 다시 (위치가 바뀌었을 수 있음). s 는 이 뒤로 쓰지 않음
    if (s.type == StepType::Arm && s.arm_task == kArmPick && step_i_ > 0 &&
      steps_[step_i_ - 1].type == StepType::Navigate) {--step_i_;}
    // PICK 이 팔을 움직이고 실패 → 퍽이 밀렸을 수 있음 → 같은 정차의 LOOK 을 다시 하고 그 뒤 검출로 집음 (HOME → LOOK → PICK)
    else if (pick_relook_ && need_home && s.type == StepType::Arm && s.arm_task == kArmPick) {insert_relook(s.puck_id);}
    if (need_home) {insert_home_at_current("팔 작업 실패 뒤");}
    return;
  }
  RCLCPP_WARN(logger(), "단계 실패: %s (%s)", s.name.c_str(), detail_.c_str());
  ev.kind = ExecEvent::Kind::StepFailed;
  ev.step = s;
  ev.detail = detail_;
  ev.need_home = need_home;
}

int Executor::retries_allowed(const Step & s) const
{
  if (s.type == StepType::Navigate) {return nav_retries_;}
  if (s.type == StepType::Arm && s.arm_task == kArmPick) {return pick_retries_;}
  if (s.type == StepType::Arm && s.arm_task == kArmPlace) {return place_retries_;}
  return 0;
}

// ================================================================ 단계 목록

void Executor::set_steps(std::vector<Step> steps)
{
  steps_ = std::move(steps);
  step_i_ = 0;
  step_running_ = false;
  arm_timeout_wait_ = false;   // 목록을 바꾸면 기다리던 판정은 버림 (늦은 성공은 update() 가 재고에 반영)
  waiting_drive_ = false;
  arm_try_start_ = -1.0;
  server_wait_start_ = -1.0;
  drive_wait_start_ = -1.0;
  w_drive_ = false;
  retries_.assign(steps_.size(), 0);
  pending_after_.clear();
  path_retry_reset();
}

void Executor::erase_steps(std::size_t a, std::size_t b)
{
  a = std::min(a, steps_.size());
  b = std::clamp(b, a, steps_.size());
  steps_.erase(steps_.begin() + static_cast<std::ptrdiff_t>(a), steps_.begin() + static_cast<std::ptrdiff_t>(b));
  retries_.erase(retries_.begin() + static_cast<std::ptrdiff_t>(a), retries_.begin() + static_cast<std::ptrdiff_t>(b));
  step_running_ = false;
}

void Executor::insert_steps(std::size_t at, const std::vector<Step> & more)
{
  at = std::min(at, steps_.size());
  steps_.insert(steps_.begin() + static_cast<std::ptrdiff_t>(at), more.begin(), more.end());
  retries_.insert(retries_.begin() + static_cast<std::ptrdiff_t>(at), more.size(), 0);
}

void Executor::insert_after_current(std::vector<Step> more)
{
  pending_after_.insert(pending_after_.end(), more.begin(), more.end());
}

// 넘김 통로: 로봇이 정차점 뒤 (−0.02 ~ handoff m) · 옆 lat m 안 · 방향 yaw° 안일 때만 Nav2 를 끊고 정밀 접근으로.
//   반지름만 보면 박스 옆을 스치며 다가올 때 넘겨져 정밀 접근이 박스 옆에서 출발함 (리뷰)
static bool in_dock_handoff(const Pose2D & r, const Pose2D & stop, double handoff_m, double lat_m, double yaw_deg)
{
  const double dx = stop.x - r.x, dy = stop.y - r.y;
  const double ahead = dx * std::cos(stop.yaw) + dy * std::sin(stop.yaw);
  const double lat = -dx * std::sin(stop.yaw) + dy * std::cos(stop.yaw);
  const double dyaw = std::remainder(stop.yaw - r.yaw, 2.0 * M_PI);
  return ahead > -0.02 && ahead < handoff_m && std::fabs(lat) < lat_m && std::fabs(dyaw) < yaw_deg * M_PI / 180.0;
}

static Step home_step(const std::string & name)
{
  Step home;
  home.type = StepType::Arm;
  home.arm_task = kArmHome;
  home.name = name;
  return home;
}

// 지금 단계 앞에 HOME 끼워 넣기
//   - control_master 는 팔 작업이 실패 · 취소되면 바퀴를 STOP 으로 잠그고, HOME 이 성공해야 DRIVE 를 줌
//   - 팔 실패 뒤 HOME (insert_home) 은 DRIVE 복구 셈에 안 넣음: 재시도 수는 pick_retries 등이 이미 제한하고,
//     팔 동작만 이어지는 구간에는 DRIVE 를 받을 주행 단계가 없음 (셈에 넣으면 연속 팔 실패가 FAULT 로 감)
//     HOME 뒤 DRIVE 가 정말 안 오면 다음 주행 단계의 DRIVE 대기 (update_drive_reason) 가 잡음
void Executor::insert_home(const std::string & why)
{
  insert_home_at_current(why);
}

// 지금 단계 (PICK) 앞에 같은 정차의 마지막 LOOK 을 한 번 더 끼움. puck_id 를 달아 성공 시각을 relook_after_ 에 남김
//   같은 정차 = 지금 단계에서 앞으로 첫 이동 단계까지. LOOK 이 없으면 안 끼움
void Executor::insert_relook(const std::string & puck_id)
{
  for (std::size_t j = step_i_; j-- > 0;) {
    if (steps_[j].type == StepType::Navigate) {break;}
    if (steps_[j].type != StepType::Arm || !is_look_task(steps_[j].arm_task)) {continue;}
    Step look = steps_[j];
    look.puck_id = puck_id;
    look.name += "(다시 보기:" + puck_id + ")";
    RCLCPP_WARN(logger(), "PICK %s 재시도 전 다시 보기: %s", puck_id.c_str(), look.name.c_str());
    relook_after_.erase(puck_id);
    insert_steps(step_i_, {look});
    return;
  }
}

void Executor::insert_home_at_current(const std::string & why)
{
  RCLCPP_WARN(logger(), "HOME 먼저: %s", why.c_str());
  insert_steps(step_i_, {home_step("arm:HOME(복구)")});
  step_running_ = false;
  arm_try_start_ = -1.0;
  server_wait_start_ = -1.0;
  drive_wait_start_ = -1.0;
  w_drive_ = false;
}

// DRIVE 안 옴 → HOME. DRIVE 를 받기 전에 두 번째로 끼우게 되면 Fault (HOME 이 성공해도 DRIVE 가 안 오는 상황)
bool Executor::insert_home_counted(const std::string & why, ExecEvent & ev)
{
  if (++recover_homes_ > 1) {
    ev.kind = ExecEvent::Kind::Fault;
    ev.code = "DRIVE_NOT_RESTORED";
    ev.detail = "HOME 뒤에도 DRIVE 안 됨: " + why;
    return false;
  }
  insert_home_at_current(why);
  return true;
}

void Executor::wait_drive(double now)
{
  waiting_drive_ = true;
  waiting_drive_since_ = now;
}

// 주행 단계인데 DRIVE 아님 → DRIVE_LOST. drive_wait_s 넘으면 HOME 끼워 넣음 (true 반환)
//   팔 작업 성공 직후엔 DRIVE 방송이 결과보다 조금 늦게 옴 → 잠깐 켜졌다 꺼지는 건 정상
bool Executor::update_drive_reason(double now, bool drive_step, ExecEvent & ev)
{
  if (!require_motion_mode_ || !drive_step) {
    w_drive_ = false;
    drive_wait_start_ = -1.0;
    return false;
  }
  if (have_mode_ && mode_ == MotionMode::DRIVE) {
    w_drive_ = false;
    drive_wait_start_ = -1.0;
    recover_homes_ = 0;
    return false;
  }
  if (!w_drive_) {
    w_drive_ = true;
    drive_wait_start_ = now;
  }
  if (now - drive_wait_start_ > drive_wait_s_) {
    insert_home_counted("DRIVE 안 옴 (" + mode_reason_ + ")", ev);
    return true;
  }
  return false;
}

// 이동 차단 → Nav2 목표 취소. 풀리면 목표를 다시 계산해서 보냄
void Executor::stop_blocked_nav()
{
  if (nav_.active) {cancel_nav();}
  if (step_i_ < steps_.size() && steps_[step_i_].type == StepType::Navigate) {step_running_ = false;}
  server_wait_start_ = -1.0;
}

// ---------------------------------------------------------------- 대기 원인별 차단 범위 (한 곳에서만 정함)
//   DRIVE_LOST    : 모든 이동 차단 (control 이 바퀴를 잠근 상태)
//   SCAN_LOST     : 모든 이동 차단
//   LOCALIZATION  : 모든 이동 차단 + PICK · map 의존 팔 목표 (Target::Puck · Custom 의 PICK · PLACE) 시작 차단
//                   (map 기준 목표를 못 믿음). HOME · LOOK 류 · STORE · RETRIEVE 는 허용 (arm_start_blocked, health.hpp)
//   DETECTION_LOST: PICK 시작만 차단 (주행 허용)
//   require_localization false 면 loc_mon_ 이 안 켜짐 → 위치 차단 없음
bool Executor::nav_blocked() const
{
  return w_drive_ || scan_mon_.active() || loc_blocks();
}

bool Executor::hold_wanted() const
{
  if (rec_nav_active_ && !w_drive_ && !scan_mon_.active()) {return false;}   // 위치 복구 물러나기 이동은 통과
  return nav_blocked();
}

std::vector<std::string> Executor::wait_reasons() const
{
  std::vector<std::string> r;
  if (w_drive_) {r.push_back("DRIVE_LOST");}
  if (scan_mon_.active()) {r.push_back("SCAN_LOST");}
  if (det_mon_.active()) {r.push_back("DETECTION_LOST");}
  if (loc_blocks()) {r.push_back("LOCALIZATION_LOST");}
  if (loc_recovery_ && loc_rec_.mode() == LocMode::Degraded) {r.push_back("LOCALIZATION_DEGRADED");}
  return r;
}

void Executor::reset_monitors()
{
  scan_mon_.reset();
  det_mon_.reset();
  det_gap_.reset();
  loc_mon_.reset();
  loc_rec_.reset();
  loc_mode_logged_ = LocMode::Ok;
  rec_pending_ = false;
  rec_nav_active_ = false;
  sensor_fault_since_ = -1.0;
}

// 준비 안 된 이유 목록 (비어 있으면 ready). 움직이지 않는 확인만 함
std::vector<std::string> Executor::ready_reasons(double now)
{
  std::vector<std::string> r;
  if (last_odom_s_ < 0.0 || now - last_odom_s_ > control_timeout_s_) {r.push_back("ODOM");}
  if (require_motion_mode_) {
    if (!have_mode_ || now - last_mode_s_ > control_timeout_s_) {
      r.push_back("CONTROL");
    } else if (!mode_ready_) {
      r.push_back("CONTROL_NOT_READY");   // DRIVE 는 안 봄 (켤 때 STOP 은 HOME 전 정상)
    }
  }
  if (!arm_client_->action_server_is_ready()) {r.push_back("ARM_SERVER");}
  if (!nav_client_->action_server_is_ready()) {r.push_back("NAV2_SERVER");}
  if (dock_client_ && !dock_client_->action_server_is_ready()) {r.push_back("DOCK_SERVER:" + dock_action_);}
  for (const auto & lc : lifecycle_) {
    if (!lc.ok || now - lc.checked_s > 3.0) {r.push_back("NAV2_LIFECYCLE:" + lc.name);}
  }
  if (require_localization_) {
    if (!map_odom_fresh(now)) {r.push_back("LOCALIZATION_TF");}
    if (!loc_from_amcl_) {
      if (!loc_health_ready(loc_health_, now, loc_timeout_s_)) {
        // 못 받음 · 끊김 · level 1 이상을 구분해 표시
        const bool stale = loc_health_.level < 0 || now - loc_health_.received_s > loc_timeout_s_;
        r.push_back(stale ? std::string("LOCALIZATION_HEALTH:LOST") : "LOCALIZATION_HEALTH:" + std::to_string(loc_health_.level));
      }
    } else if (!pose_std_ || pose_std_->x > loc_ready_xy_std_ || pose_std_->y > loc_ready_xy_std_ ||
      pose_std_->yaw > loc_ready_yaw_std_)
    {
      r.push_back("LOCALIZATION_COV");
    }
  }
  if (require_scan_) {
    for (const auto & t : stale_scan_topics(now)) {r.push_back("SCAN:" + t);}
  }
  if (require_detections_) {
    for (const auto & f : stale_detection_frames(now)) {r.push_back("DETECTIONS:" + f);}
  }
  if (cargo_.doubt()) {r.push_back("INVENTORY_UNCERTAIN");}   // 사람 확인 뒤 /mission/inventory_*
  if (cargo_.holding()) {r.push_back("GRIPPER_RECORD");}   // 사람 확인 뒤 /mission/clear_gripper
  return r;
}

void Executor::stop()
{
  reconciling_ = false;   // 자동 정합 중 FAULT → 표시는 남고 사람 정합 (inventory_*)
  hand_wait_until_ = -1.0;
  const bool retrieve_hand_wait = retrieve_hand_until_ > 0.0;   // RETRIEVE 성공 뒤 손 확인 중 FAULT → 성공 반영 대신 불확실
  retrieve_hand_until_ = -1.0;
  // 팔 결과는 왔는데 step() 이 아직 처리 안 함 (supervise FAULT 가 먼저) → 그 결과대로 반영. 버리면 재고가 실물과 어긋남
  const bool arm_step_running = step_running_ && step_i_ < steps_.size() && steps_[step_i_].type == StepType::Arm && arm_step_;
  if (arm_step_running && arm_.done && !arm_.rejected && !arm_timeout_wait_) {
    if (arm_.success && retrieve_hand_wait) {
      mark_doubt(*arm_step_, "FAULT 직전 RETRIEVE 성공 결과, 그리퍼 실측 확인 전");
    } else if (arm_.success) {
      RCLCPP_WARN(logger(), "FAULT 직전 끝난 팔 작업 성공 → 재고 반영: %s", arm_step_->name.c_str());
      std::string why;
      if (!update_cargo(*arm_step_, why)) {mark_doubt(*arm_step_, "FAULT 직전 끝난 팔 작업 성공인데 재고 기록 불일치: " + why);}
    } else if (arm_.message.find(kArmNotMovedToken) != std::string::npos) {
      RCLCPP_WARN(logger(), "FAULT 직전 팔 작업이 움직이기 전에 끝남 (%s) → 재고 그대로", arm_step_->name.c_str());
    } else if (empty_grasp_skip_ && empty_grasp_confirmed(arm_step_->arm_task, ArmEnd::Failed, arm_.message)) {
      RCLCPP_WARN(logger(), "FAULT 직전 빈손 PICK: 퍽 %s 박스에 그대로 → 재고 그대로", arm_step_->puck_id.c_str());
    } else if (store_returned_to_box(*arm_step_, ArmEnd::Failed, arm_.message)) {
      RCLCPP_WARN(logger(), "FAULT 직전 STORE 박스 되돌림: 퍽 %s 그리퍼 비움 · 박스로", arm_step_->puck_id.c_str());
    } else if (inventory_doubt_after(arm_step_->arm_task, ArmEnd::Failed)) {
      mark_doubt(*arm_step_, "FAULT 직전 팔 작업 실패: " + arm_.message);
    }
  }
  // 진행 중 · 시간 초과로 취소해 결과 기다리던 재고 작업 → 결과 모름 (늦게 성공이 오면 update() 가 반영하고 표시 지움)
  if (arm_.active && arm_step_ && inventory_doubt_after(arm_step_->arm_task, ArmEnd::Unknown)) {
    mark_doubt(*arm_step_, "FAULT 로 취소, 최종 결과 모름");
  } else if (arm_step_running && arm_timeout_wait_ && !late_arm_ok_ &&
    inventory_doubt_after(arm_step_->arm_task, ArmEnd::Unknown))
  {
    mark_doubt(*arm_step_, "시간 초과로 취소, 최종 결과 " + std::string(arm_cancel_pending_ ? "모름" : "성공 아님"));
  }
  waiting_drive_ = false;
  arm_timeout_wait_ = false;   // 늦은 성공은 update() 가 재고에 반영
  cancel_nav();
  cancel_arm();
  w_drive_ = false;
  rec_pending_ = false;
  rec_nav_active_ = false;
  path_retry_reset();
}

// 취소한 액션의 최종 결과를 받기 전에는 움직이는 중일 수 있음 (서버가 없어졌으면 기다릴 대상 없음)
std::string Executor::cancel_pending_reason() const
{
  if (arm_cancel_pending_ && arm_client_->action_server_is_ready()) {return "팔 작업 취소 결과 기다리는 중";}
  if (nav_cancel_pending_ && nav_cur_client_->action_server_is_ready()) {return "이동 취소 결과 기다리는 중";}
  return "";
}

// 새 목표 보내기 전 확인: 취소한 목표(팔 · Nav2 어느 쪽이든)가 아직 끝나지 않았으면 기다림
//   - 끝나기 전에 새 목표를 보내면 이전 목표와 겹쳐 움직이거나, 이전 결과가 버려져 재고 기록이 어긋남
//   - 서버가 없어졌으면 기다릴 대상 없음 (cancel_pending_reason 과 같은 기준)
bool Executor::cancel_blocked() const
{
  return !cancel_pending_reason().empty();
}

bool Executor::arm_busy() const
{
  if (arm_.active || arm_timeout_wait_) {return true;}
  // 결과 도착 (거절 아님) + 그 팔 단계가 아직 진행 중 = step() 이 결과를 아직 처리 안 함
  return arm_.done && !arm_.rejected && step_running_ && step_i_ < steps_.size() &&
         steps_[step_i_].type == StepType::Arm;
}

// ================================================================ 단계 실행

StepResult Executor::run_step(const Step & s, double now)
{
  switch (s.type) {
    case StepType::Navigate: return run_navigate(s, now);
    case StepType::Arm: return run_arm(s, now);
    case StepType::Custom:
      if (!step_running_) {
        step_running_ = true;
        step_start_s_ = now;
      }
      if (!custom_) {
        detail_ = "Custom 단계 실행 함수 없음";
        return StepResult::Fail;
      }
      return custom_(s, now, step_start_s_, detail_);
  }
  return StepResult::Fail;
}

StepResult Executor::run_navigate(const Step & s, double now)
{
  const bool two_stage = s.dock && dock_client_;
  if (!step_running_) {
    if (now < path_retry_.until) {return StepResult::Running;}   // 경로 계획 실패 뒤 기다리는 중 (path_retry)
    Pose2D goal;
    if (!navigate_goal(s, goal)) {return StepResult::Fail;}
    if (two_stage) {   // 1 단계 Nav2 목표 = 정차점 뒤 (보는 방향 반대로) dock_nav_back_m
      goal.x -= dock_nav_back_m_ * std::cos(goal.yaw);
      goal.y -= dock_nav_back_m_ * std::sin(goal.yaw);
    }
    if (cancel_blocked()) {   // 취소한 목표 결과 대기 → server_wait_s 넘으면 실패
      return server_wait_expired(now, "취소 결과") ? StepResult::Fail : StepResult::Running;
    }
    if (!send_nav(goal)) {   // 서버 대기 중 → server_wait_s 넘으면 실패
      return server_wait_expired(now, "Nav2") ? StepResult::Fail : StepResult::Running;
    }
    server_wait_start_ = -1.0;
    step_running_ = true;
    nav_docking_ = false;
    nav_handoff_ = false;
    return StepResult::Running;
  }
  if (!nav_docking_) {
    if (!nav_handoff_) {
      // 정차점 뒤 통로 (dock_handoff_m · lat · yaw) 안에 들어오면 서지 않고 넘김: Nav2 취소 → 취소 결과 뒤 바로 정밀 접근
      Pose2D rp, fin;
      if (two_stage && dock_handoff_m_ > 0.0 && nav_.active && navigate_goal(s, fin) && robot_pose(rp) &&
        in_dock_handoff(rp, fin, dock_handoff_m_, dock_handoff_lat_m_, dock_handoff_yaw_deg_))
      {
        cancel_nav();
      } else {
        const StepResult r = poll_nav(now, s.timeout_s > 0.0 ? s.timeout_s : nav_timeout_s_);
        if (r == StepResult::Fail && path_retry(s, now)) {return StepResult::Running;}
        if (r != StepResult::Success) {return r;}
        if (!two_stage) {return r;}
      }
      nav_handoff_ = true;
    }
    // 2 단계: 정차점으로 정밀 접근. 취소한 Nav2 의 최종 결과 전에는 안 보냄 (cmd_vel_nav 겹침 방지)
    if (cancel_blocked()) {
      return server_wait_expired(now, "Nav2 취소 결과") ? StepResult::Fail : StepResult::Running;
    }
    Pose2D goal;
    if (!navigate_goal(s, goal)) {return StepResult::Fail;}
    if (!send_nav(goal, true)) {
      return server_wait_expired(now, "정밀 접근 " + dock_action_) ? StepResult::Fail : StepResult::Running;
    }
    server_wait_start_ = -1.0;
    nav_docking_ = true;
    dock_meas_.reset();
    dock_meas_n_ = 0;
    dock_nominal_ = s.pose;
    return StepResult::Running;
  }
  const StepResult r = poll_nav(now, dock_timeout_s_);
  if (r == StepResult::Fail) {detail_ = "정밀 접근: " + detail_;}
  if (r != StepResult::Running) {
    commit_dock_fix();
    nav_docking_ = false;
  }
  return r;
}

// 정밀 접근이 끝남 (성공 · 실패): 그동안 받은 마지막 잰 정차점 → 정차점 보정표 (Target::Fixed 정차만)
void Executor::commit_dock_fix()
{
  if (!dock_meas_) {return;}
  const Pose2D m = *dock_meas_;
  dock_meas_.reset();
  const double d = std::hypot(m.x - dock_nominal_.x, m.y - dock_nominal_.y);
  if (d > dock_fix_max_m_) {
    RCLCPP_WARN(logger(), "잰 정차점이 단계 정차점에서 %.1f cm (> %.0f cm) → 보정표에 안 넣음", d * 100.0, dock_fix_max_m_ * 100.0);
    return;
  }
  set_dock_fix(dock_nominal_, m);
  RCLCPP_INFO(logger(), "정차점 보정표: (%.3f, %.3f, %.1f°) → 잰 정차점 (%.3f, %.3f, %.1f°), %.1f cm · %+.1f° (측정 %d 번)",
    dock_nominal_.x, dock_nominal_.y, dock_nominal_.yaw * 180.0 / M_PI, m.x, m.y, m.yaw * 180.0 / M_PI, d * 100.0,
    std::remainder(m.yaw - dock_nominal_.yaw, 2.0 * M_PI) * 180.0 / M_PI, dock_meas_n_);
}

bool Executor::dock_fix(const Pose2D & nominal, Pose2D & out) const
{
  for (const auto & f : dock_fix_) {
    if (std::hypot(f.nominal.x - nominal.x, f.nominal.y - nominal.y) <= dock_fix_match_m_ &&
      std::abs(std::remainder(f.nominal.yaw - nominal.yaw, 2.0 * M_PI)) <= 15.0 * M_PI / 180.0)
    {
      out = f.measured;
      return true;
    }
  }
  return false;
}

void Executor::set_dock_fix(const Pose2D & nominal, const Pose2D & measured)
{
  for (auto & f : dock_fix_) {
    if (std::hypot(f.nominal.x - nominal.x, f.nominal.y - nominal.y) <= dock_fix_match_m_ &&
      std::abs(std::remainder(f.nominal.yaw - nominal.yaw, 2.0 * M_PI)) <= 15.0 * M_PI / 180.0)
    {
      f.measured = measured;
      return;
    }
  }
  dock_fix_.push_back(DockFix{nominal, measured});
}

// 단계 목표 → 지도 위 자세
bool Executor::navigate_goal(const Step & s, Pose2D & goal)
{
  switch (s.target) {
    case Target::Fixed:
      goal = s.pose;
      if (s.dock && dock_fix(s.pose, goal) && !step_running_) {   // 같은 정차점에서 전에 잰 박스 → 그 정차점
        RCLCPP_INFO(logger(), "%s: 정차점 보정표 씀 → (%.3f, %.3f, %.1f°), 단계 정차점과 %.1f cm", s.name.c_str(), goal.x, goal.y,
          goal.yaw * 180.0 / M_PI, std::hypot(goal.x - s.pose.x, goal.y - s.pose.y) * 100.0);
      }
      return true;
    case Target::Puck: {
        const auto rec = registry_.find(s.puck_id);
        Pose2D robot;
        if (!rec || !robot_pose(robot)) {
          detail_ = "퍽 " + s.puck_id + " 위치 또는 로봇 위치 모름";
          return false;
        }
        goal = approach_pose(robot.x, robot.y, rec->x, rec->y, puck_standoff_m_);
        return true;
      }
    default:
      detail_ = "Navigate 목표 없음";
      return false;
  }
}

// 성공해도 팔이 펴진 채 남을 수 있는 작업 (LOOK 류, lazy_fold 의 STORE · PLACE) → 다음 이동 전 HOME
bool Executor::leaves_arm_unfolded(const Step & s)
{
  return s.type == StepType::Arm && (is_look_task(s.arm_task) || s.arm_task == kArmLookBarcode ||
    s.arm_task == kArmStore || s.arm_task == kArmPlace);   // lazy_fold: STORE · PLACE 는 P0 에서 끝남
}

StepResult Executor::run_arm(const Step & s, double now)
{
  if (!step_running_) {
    // 재고 기록과 단계 퍽 · 칸이 안 맞으면 팔 명령 없이 실패 → on_fail 이 FAULT
    std::string why;
    if (!arm_preflight_ok(cargo_, s.arm_task, s.puck_id, s.slot, &why)) {
      detail_ = "재고 기록과 단계 불일치 (" + s.name + "): " + why + " → 팔 명령 안 보냄";
      preflight_refused_ = true;
      return StepResult::Fail;
    }
    // 다시 보기 LOOK 뒤 PICK: 그 뒤에 받은 검출이 올 때까지 relook_wait_s 기다림, 안 오면 실패 (퍽이 사라졌거나 못 읽음)
    if (s.arm_task == kArmPick) {
      const auto ra = relook_after_.find(s.puck_id);
      if (ra != relook_after_.end()) {
        const auto ls = local_seen_.find(s.puck_id);
        if (ls == local_seen_.end() || ls->second.seen < ra->second) {
          if (now - ra->second < relook_wait_s_) {return StepResult::Running;}
          detail_ = "다시 LOOK 뒤 퍽 " + s.puck_id + " 안 보임";
          return StepResult::Fail;
        }
      }
    }
    ArmTask::Goal goal;
    goal.task = s.arm_task;
    goal.slot = s.slot;   // STORE · RETRIEVE
    if (s.arm_task == kArmPick) {goal.puck_id = s.puck_id;}   // control 이 이 ID 의 손목 검출을 목표로 잡음 (위치만 보면 옆 퍽을 집을 수 있음)
    if (!arm_target(s, goal.target, now)) {return StepResult::Fail;}
    // control_master 가 새 팔 작업을 못 받는 동안 (하드웨어 이상 · 재생기 준비 전) 기다림 → 거절 반복 안 함
    if (require_motion_mode_ && !(have_mode_ && mode_ready_)) {
      return server_wait_expired(now, "/arm_task 준비(ready)") ? StepResult::Fail : StepResult::Running;
    }
    if (cancel_blocked()) {   // 취소한 목표 결과 대기 → server_wait_s 넘으면 실패 (HOME 이면 FAULT)
      return server_wait_expired(now, "취소 결과") ? StepResult::Fail : StepResult::Running;
    }
    if (arm_try_start_ < 0.0) {arm_try_start_ = now;}   // 거절 재시도 전체 시간 기준
    if (arm_sent_) {arm_sent_(s, now);}
    if (!send_arm(goal)) {   // 서버 대기 중 → server_wait_s 넘으면 실패
      return server_wait_expired(now, "/arm_task") ? StepResult::Fail : StepResult::Running;
    }
    arm_step_ = s;   // 취소 뒤 늦게 성공하면 재고 반영용
    retrieve_hand_until_ = -1.0;
    server_wait_start_ = -1.0;
    step_running_ = true;
    step_start_s_ = now;
    return StepResult::Running;
  }
  // 받았지만 동작 전에 끝남 (ARM_NOT_MOVED) = 팔 · 퍽 그대로 → 거절과 같이 다시 보냄 (재고 불확실 · HOME 실패 FAULT 아님)
  if (arm_.done && !arm_.success && !arm_.rejected && arm_.message.find(kArmNotMovedToken) != std::string::npos) {
    RCLCPP_WARN(logger(), "%s: control 이 움직이기 전에 끝냄 (%s) → 거절처럼 다시 보냄", s.name.c_str(), arm_.message.c_str());
    arm_.rejected = true;
    arm_step_.reset();
  }
  // LOOK_RACK 거절 (control 구현 전 등) → STORE 로 넘어감, 팔은 안 움직였음
  if (arm_.done && arm_.rejected && s.arm_task == kArmLookRack) {
    RCLCPP_WARN(logger(), "%s 거절 (%s) → 그대로 STORE", s.name.c_str(), arm_.message.c_str());
    arm_step_.reset();   // 반영할 성공 없음 → 다음 목표 때 늦은 성공으로 안 봄
    return StepResult::Success;
  }
  // 거절 → action_retry_s 뒤 다시 보냄 (켤 때 팔 재생기 준비 기다리기)
  if (arm_.done && arm_.rejected) {
    // control 이 이 퍽 PICK 을 '손이 안 닿음 · 다시 쥘 자리 없음' 으로 거절 (이 시도를 보낸 뒤 받은 motion_mode reason) → 같은 목표를 다시 보내도
    //   같음 → 거절 재전송 (action_retry_max_s) 없이 이 시도 실패. 단계 재시도 (pick_retries) · 미션 재계획 (그 퍽 건너뜀) 은 그대로
    if (s.arm_task == kArmPick && unreachable_s_ >= arm_.sent_s && unreachable_puck_ == s.puck_id) {
      detail_ = "팔이 안 닿는 퍽 (control " + unreachable_why_.substr(0, 300) + ")";
      arm_end_ = ArmEnd::Rejected;   // 팔 안 움직임 → 재고 확실
      RCLCPP_WARN(logger(), "%s: control 이 '손이 안 닿음' 으로 거절 → 다시 보내지 않고 이 시도 실패", s.name.c_str());
      return StepResult::Fail;
    }
    // PICK 을 control 이 '퍽 높이 이상' 으로 거절 (검출 높이가 틀림): 같은 검출로 다시 보내도 같음 → 재전송 없이 이 시도 실패
    if (s.arm_task == kArmPick && last_mode_s_ >= arm_.sent_s && mode_reason_.find(kPickHeightToken) != std::string::npos) {
      detail_ = "퍽 높이가 틀리게 재어짐 (control " + mode_reason_.substr(0, 200) + ")";
      arm_end_ = ArmEnd::Rejected;   // 팔 안 움직임 → 재고 확실
      RCLCPP_WARN(logger(), "%s: control 이 '퍽 높이 이상' 으로 거절 → 다시 보내지 않고 이 시도 실패", s.name.c_str());
      return StepResult::Fail;
    }
    const double retry_max_s = s.arm_task == kArmRetrieve && retrieve_retry_max_s_ > 0.0 ? retrieve_retry_max_s_ : action_retry_max_s_;
    if (now - arm_try_start_ > retry_max_s) {
      detail_ = "팔 작업 계속 거절됨";
      // HOME · RETRIEVE: 마지막 거절 이유 (control motion_mode reason "거절: …") 를 붙임
      //   → FAULT HOME_FAILED · RETRIEVE 건너뛰기 때 control 로그를 안 봐도 이유가 남음
      if ((s.arm_task == kArmHome || s.arm_task == kArmRetrieve) && mode_reason_.rfind("거절: ", 0) == 0 && now - last_mode_s_ < 5.0) {
        detail_ += " (" + mode_reason_.substr(0, 300) + ")";
      }
      arm_end_ = ArmEnd::Rejected;   // 팔 안 움직임 → 재고 확실
      return StepResult::Fail;
    }
    if (now - arm_.sent_s > action_retry_s_) {step_running_ = false;}
    return StepResult::Running;
  }
  // 시간 초과 → 취소 결과까지 기다려 판정 (control 이 취소 전에 끝냈을 수 있음 → 성공이면 실물대로 성공)
  if (arm_timeout_wait_) {
    if (late_arm_ok_) {
      late_arm_ok_.reset();
      arm_timeout_wait_ = false;
      arm_needs_home_ = false;   // 성공한 작업은 검사된 자세로 끝남 (펴진 채면 arm_unfolded_)
      arm_unfolded_ = leaves_arm_unfolded(s);
      RCLCPP_WARN(logger(), "시간 초과로 취소한 팔 작업이 성공으로 끝남 → 성공 처리: %s", s.name.c_str());
      std::string why;
      if (!update_cargo(s, why)) {
        detail_ = "시간 초과 뒤 최종 성공인데 재고 기록 불일치: " + why;
        arm_end_ = ArmEnd::Mismatch;   // on_fail → 불확실 표시 + FAULT
        return StepResult::Fail;
      }
      return StepResult::Success;
    }
    if (!cancel_blocked() || now - arm_cancel_s_ > server_wait_s_) {
      arm_timeout_wait_ = false;
      detail_ = cancel_blocked() ? "팔 작업 시간 초과 (취소 결과 없음)" : "팔 작업 시간 초과";
      arm_end_ = arm_cancel_pending_ ? ArmEnd::Unknown : ArmEnd::Failed;   // 보낸 뒤라 어디까지 했는지 모름
      return StepResult::Fail;
    }
    return StepResult::Running;
  }
  if (!arm_.done) {
    if (now - step_start_s_ > (s.timeout_s > 0.0 ? s.timeout_s : arm_timeout_s_)) {
      RCLCPP_WARN(logger(), "팔 작업 시간 초과 → 취소, 최종 결과 기다림: %s", s.name.c_str());
      cancel_arm();
      arm_timeout_wait_ = true;
      arm_cancel_s_ = now;
      return StepResult::Running;
    }
    return StepResult::Running;
  }
  if (!arm_.success) {
    detail_ = "팔 작업 실패: " + arm_.message;
    arm_end_ = ArmEnd::Failed;   // 수락 뒤 실패 · 취소 · 중단
    return StepResult::Fail;
  }
  // LOOK_RACK 성공: 칸 위치는 control_master 가 손목 태그로 직접 잼 (기다릴 것 없음). 팔은 퍽 쥔 채 관찰 자세 = 펴진 LOOK 류로 안 봄 (다음이 STORE)
  if (s.arm_task == kArmLookRack) {
    arm_unfolded_ = false;
    arm_step_.reset();   // 성공 처리함 → 다음 목표 때 늦은 성공으로 다시 반영 안 함 (keep_unapplied_arm_ok)
    return StepResult::Success;
  }
  // RETRIEVE 성공 결과 → 그리퍼 실측이 빈손이면 못 꺼낸 것 (퍽은 칸 · 떨어짐) → 재고 안 바꾸고 실패 (on_fail 이 불확실 표시 + 자동 정합)
  if (s.arm_task == kArmRetrieve && retrieve_hand_check_ && hand_reader_) {
    std::string hw;
    const HandGuess g = hand_reader_(now, hw);
    if (g == HandGuess::Unknown && hand_wait_s_ > 0.0) {
      if (retrieve_hand_until_ < 0.0) {
        retrieve_hand_until_ = now + hand_wait_s_;
        RCLCPP_INFO(logger(), "%s 성공 결과: 손 상태 기다림 (최대 %.1f s, %s)", s.name.c_str(), hand_wait_s_, hw.c_str());
      }
      if (now < retrieve_hand_until_) {return StepResult::Running;}
    }
    retrieve_hand_until_ = -1.0;
    if (g == HandGuess::Empty) {
      RCLCPP_ERROR(logger(), "%s 성공 결과인데 그리퍼 실측 빈손 (%s) → 못 꺼낸 것으로 보고 실패 처리 (재고 안 바꿈)", s.name.c_str(), hw.c_str());
      detail_ = "RETRIEVE 성공 결과인데 그리퍼 실측 빈손: " + hw;
      arm_end_ = ArmEnd::Failed;
      arm_needs_home_ = true;   // 팔은 박스 위 기준점 (펴진 채)
      arm_step_.reset();        // 이 성공 결과는 버림 → 다음 목표 때 늦은 성공으로 재고에 반영 안 함 (keep_unapplied_arm_ok)
      return StepResult::Fail;
    }
    if (g == HandGuess::Holding) {
      RCLCPP_INFO(logger(), "%s 성공 결과: 그리퍼 실측 쥠 (%s)", s.name.c_str(), hw.c_str());
    } else {
      RCLCPP_WARN(logger(), "%s 성공 결과: 손 상태 모름 (%s) → 성공 그대로", s.name.c_str(), hw.c_str());
    }
  }
  // 성공 → 퍽 기록부 · 재고 갱신 (팔 결과로만 바꿈)
  arm_unfolded_ = leaves_arm_unfolded(s);
  if (is_look_task(s.arm_task) && !s.puck_id.empty()) {relook_after_[s.puck_id] = now;}   // 다시 보기 LOOK
  if (s.arm_task == kArmPick) {relook_after_.erase(s.puck_id);}
  arm_step_.reset();   // 반영함 → stop · 새 목표 때 (keep_unapplied_arm_ok) 다시 반영 안 함
  std::string why;
  if (!update_cargo(s, why)) {
    detail_ = "팔 작업 성공인데 재고 기록 불일치: " + why;
    arm_end_ = ArmEnd::Mismatch;   // on_fail → 불확실 표시 + FAULT (StepOk 안 냄)
    return StepResult::Fail;
  }
  return StepResult::Success;
}

// 팔 작업 최종 성공 → 재고 전환 + 같은 단계 불확실 표시 해소 (apply_arm_success, 한 번에)
//   기록과 안 맞는 전환이면 재고 · 기록부 · 표시 모두 그대로 false → 호출한 쪽이 불확실 표시 · FAULT
bool Executor::update_cargo(const Step & s, std::string & why)
{
  const CargoItem pick = s.arm_task == kArmPick ? pick_item(s) : CargoItem{};
  ArmSuccessApplied r;
  if (!apply_arm_success(cargo_, s.arm_task, s.puck_id, s.slot, pick, &r, &why)) {
    RCLCPP_ERROR(logger(), "재고 기록 불일치 (%s): %s → 재고 · 기록부 안 바꿈", s.name.c_str(), why.c_str());
    return false;
  }
  if (!r.puck_state.empty()) {registry_.set_state(s.puck_id, r.puck_state);}
  if (s.arm_task == kArmPlace) {last_placed_ = r.placed;}
  if (r.doubt_cleared) {RCLCPP_WARN(logger(), "재고 불확실 해소 (최종 성공): %s", s.name.c_str());}
  if (r.changed) {cargo_changed();}
  return true;
}

// STORE 결과 글 STORE_RETURNED_TO_BOX x y (base_link) → 그리퍼 기록 비움 · 퍽 기록부 on_field + 그 자리 (map, 지금 로봇 위치로).
//   store_return_replan 꺼짐 · 토큰 없음 · 그리퍼 기록이 그 퍽이 아님 = false (→ 불확실 FAULT)
bool Executor::store_returned_to_box(const Step & s, ArmEnd end, const std::string & message)
{
  double bx = 0.0, by = 0.0;
  if (!store_return_replan_ || !store_returned_confirmed(s.arm_task, end, message, &bx, &by)) {return false;}
  std::string state, err;
  if (!apply_store_returned(cargo_, s.puck_id, &state, &err)) {
    RCLCPP_ERROR(logger(), "STORE 박스 되돌림인데 재고 기록 불일치 (%s): %s → 재고 불확실", s.name.c_str(), err.c_str());
    return false;
  }
  Pose2D rp;
  std::string where = "기록부 위치 그대로 (로봇 위치 모름)";
  if (const auto rec = registry_.find(s.puck_id); rec && robot_pose(rp)) {
    PuckRecord r = *rec;
    r.x = rp.x + std::cos(rp.yaw) * bx - std::sin(rp.yaw) * by;
    r.y = rp.y + std::sin(rp.yaw) * bx + std::cos(rp.yaw) * by;
    registry_.update(r);
    char buf[96];
    std::snprintf(buf, sizeof(buf), "map (%.3f, %.3f)", r.x, r.y);
    where = buf;
  }
  if (!state.empty()) {registry_.set_state(s.puck_id, state);}
  cargo_changed();
  RCLCPP_WARN(logger(), "STORE 박스 되돌림: 퍽 %s 그리퍼 비움 → 박스 base (%.3f, %.3f) = %s, 재고 확실 → 다시 계획 (HOME 먼저)",
    s.puck_id.c_str(), bx, by, where.c_str());
  return true;
}

CargoItem Executor::pick_item(const Step & s) const
{
  CargoItem it;
  it.puck_id = s.puck_id;
  if (const auto rec = registry_.find(s.puck_id)) {
    it.color = rec->color;
    it.shape = rec->shape;
  }
  it.task_id = task_id_;
  return it;
}

// 재고 불확실 표시: 기록은 마지막 확인 그대로 두고 원인 단계만 남김 (저장 + 발행)
void Executor::mark_doubt(const Step & s, const std::string & why)
{
  const CargoItem pick = s.arm_task == kArmPick ? pick_item(s) : CargoItem{};
  if (!cargo_.set_doubt(make_doubt(cargo_, s.arm_task, s.puck_id, s.slot, pick, s.name, why))) {
    RCLCPP_WARN(logger(), "재고 불확실 이미 표시됨 (%s) → 첫 원인 유지, 이번: %s (%s)", cargo_.doubt()->step_name.c_str(),
      s.name.c_str(), why.c_str());
    return;
  }
  RCLCPP_ERROR(logger(), "재고 불확실: %s", doubt_text().c_str());
  cargo_changed();
}

std::string Executor::doubt_text() const
{
  if (!cargo_.doubt()) {return "";}
  const auto & d = *cargo_.doubt();
  return d.step_name + " 퍽 " + d.item.puck_id + (d.slot.empty() ? "" : " 칸 " + d.slot) + " (" + d.reason + ")";
}

// 사람 정합: 팔 작업이 끝났을 때만 (진행 중이면 실물이 아직 바뀔 수 있음). resolved = 정합한 불확실 표시 (미션이 PLACE 셈에 씀)
bool Executor::resolve_inventory(DoubtFix fix, std::string & msg, CargoDoubt * resolved)
{
  const std::string pending = cancel_pending_reason();
  if (arm_.active || arm_timeout_wait_ || !pending.empty()) {
    msg = (pending.empty() ? std::string("팔 작업 최종 결과 기다리는 중") : pending) + " → 결과 뒤 다시";
    return false;
  }
  const std::string before = doubt_text();
  const std::optional<CargoDoubt> d = cargo_.doubt();
  std::string id, state, err;
  if (!resolve_doubt(cargo_, fix, &id, &state, &err)) {
    msg = "정합 못 함 (" + std::string(doubt_fix_name(fix)) + "): " + err;
    return false;
  }
  if (!state.empty()) {registry_.set_state(id, state);}
  // 사람 확인이 기준: 서버가 사라져 못 받은 이전 결과가 나중에 와도 다시 반영하지 않음
  arm_cancel_step_.reset();
  late_arm_ok_.reset();
  cargo_changed();
  msg = "재고 정합 " + std::string(doubt_fix_name(fix)) + ": " + before;
  RCLCPP_WARN(logger(), "%s", msg.c_str());
  if (resolved && d) {*resolved = *d;}
  return true;
}

// 사람이 손의 퍽을 빼서 그 퍽 원래 박스 위에 되돌려 놓음 → 그리퍼 기록 비움 · 퍽 = on_field (기록부 자리 = 마지막으로 본 박스 위)
//   불확실 표시가 그 퍽을 쥔 채 실패한 작업 (STORE · LOOK_RACK · PLACE) 이면 before 정합을 같이 함 (퍽이 손에 있었음)
//   그 밖 불확실 (PICK · RETRIEVE · 다른 퍽) 은 거절 → /mission/inventory_* 먼저
bool Executor::return_gripper_puck(std::string & msg, std::string * puck_id)
{
  const std::string pending = cancel_pending_reason();
  if (arm_.active || arm_timeout_wait_ || !pending.empty()) {
    msg = (pending.empty() ? std::string("팔 작업 최종 결과 기다리는 중") : pending) + " → 결과 뒤 다시";
    return false;
  }
  if (!cargo_.holding()) {
    msg = "그리퍼 기록 비어 있음 → 되돌릴 퍽 없음";
    return false;
  }
  const std::string id = cargo_.gripper()->puck_id;
  if (const auto & d = cargo_.doubt()) {
    const std::string did = d->item.puck_id.empty() ? d->puck_id : d->item.puck_id;
    const bool held_task = d->arm_task == kArmStore || d->arm_task == kArmLookRack || d->arm_task == kArmPlace;
    if (!held_task || did != id) {
      msg = "재고 불확실 (" + doubt_text() + ") 이 손의 퍽 " + id + " 을 쥔 작업이 아님 → /mission/inventory_* 먼저";
      return false;
    }
    std::string m;
    if (!resolve_inventory(DoubtFix::Before, m)) {
      msg = m;
      return false;
    }
  }
  std::string state, err;
  if (!apply_store_returned(cargo_, id, &state, &err)) {
    msg = "그리퍼 기록 못 비움: " + err;
    return false;
  }
  if (!state.empty()) {registry_.set_state(id, state);}
  late_arm_ok_.reset();
  cargo_changed();
  if (puck_id) {*puck_id = id;}
  msg = "퍽 " + id + " 박스로 되돌림 (그리퍼 기록 비움, 퍽 on_field)";
  RCLCPP_WARN(logger(), "%s", msg.c_str());
  return true;
}

// ArmTask target (base_link 기준) 만들기
//   - position = 그리퍼 중심이 갈 점 (PICK · PLACE 같은 기준), orientation = 퍽 자세 · 모르면 (0,0,0,0)
bool Executor::arm_target(const Step & s, geometry_msgs::msg::PoseStamped & target, double now)
{
  target = geometry_msgs::msg::PoseStamped();
  target.pose.orientation.w = 0.0;   // (0,0,0,0) = 방향 모름 (msg 기본값은 w = 1 이라 직접 0 으로)
  target.header.frame_id = base_frame_;
  target.header.stamp = node_.get_clock()->now();
  if (s.arm_task == kArmStore) {
    // 칸 위치는 control_master 가 손목 태그로 잼 (적재함 맞춤 · 쥔 퍽 보정) → 빈 프레임 (= 녹화 경로 그대로)
    target.header.frame_id = "";
    return true;
  }
  if (s.arm_task == kArmHome || is_look_task(s.arm_task) || s.arm_task == kArmLookBarcode || s.arm_task == kArmLookRack ||
    s.arm_task == kArmRetrieve)
  {
    return true;   // 목표 없음 (자세 · 칸 이름만)
  }
  if (s.target == Target::Custom) {
    // 미션이 정한 점 (TargetResolver, 높이 포함) → base_link
    geometry_msgs::msg::PoseStamped in;
    if (!resolver_) {
      detail_ = "Custom 목표 계산 함수 없음";
      return false;
    }
    if (!resolver_(s, in, detail_)) {return false;}
    if (!to_base(in, false, target)) {return false;}
    RCLCPP_INFO(logger(), "%s: %s (%.3f, %.3f, %.3f) → base (%.3f, %.3f, %.3f)", s.name.c_str(),
      in.header.frame_id.c_str(), in.pose.position.x, in.pose.position.y, in.pose.position.z,
      target.pose.position.x, target.pose.position.y, target.pose.position.z);
    return true;
  }
  if (s.target == Target::Puck) {
    // ① odom 기준 최근 검출 (ID 확인된 것) → base_link
    const double fresh = pick_fresh_override_ >= 0.0 ? pick_fresh_override_ : pick_fresh_s_;
    const auto it = local_seen_.find(s.puck_id);
    if (it != local_seen_.end() && now - it->second.seen <= fresh) {
      if (!to_base(it->second.pose, it->second.has_orientation, target)) {return false;}
      to_grasp_point(target, it->second.has_orientation, it->second.yaw_known, it->second.is_box, s.puck_id);
      return true;
    }
    // ② odom 기록이 없으면 퍽 기록부(map) 위치로 (AMCL 보정 오차 섞일 수 있음)
    const auto rec = registry_.find(s.puck_id);
    if (!rec) {
      detail_ = "퍽 " + s.puck_id + " 기록 없음";
      return false;
    }
    if (now - rec->last_seen > fresh) {   // 오래된 위치로 집지 않음
      detail_ = "퍽 " + s.puck_id + " 위치가 오래됨";
      return false;
    }
    RCLCPP_WARN(logger(), "퍽 %s: odom 기준 검출 없음 → map 기록 위치로 집음", s.puck_id.c_str());
    geometry_msgs::msg::PoseStamped in;
    in.header.frame_id = map_frame_;
    in.pose.position.x = rec->x;
    in.pose.position.y = rec->y;
    in.pose.position.z = rec->z;
    if (!to_base(in, false, target)) {return false;}
    to_grasp_point(target, false, false, false, s.puck_id);
    return true;
  }
  detail_ = "팔 작업 목표 없음";
  return false;
}

// 퍽 중심 (base_link) → 그리퍼 중심이 갈 점 + target 방향
//   쥘 끝을 고르고 (choose_grasp_axis), 방향은 grasp_orientation 규칙으로 다시 만듦 (z = 쥘 끝, x = 면 방향 등)
//   방향 모르면 똑바로 섰다고 보고 위쪽 끝 (자세는 (0,0,0,0) 그대로)
//   x, y, z, θ · 선/누움은 로그로 남김 (control · 실물 확인용)
void Executor::to_grasp_point(geometry_msgs::msg::PoseStamped & target, bool has_q, bool yaw_known, bool is_box,
  const std::string & puck_id) const
{
  auto & o = target.pose.orientation;
  auto & p = target.pose.position;
  const Quat q{o.x, o.y, o.z, o.w};
  const Vec3 c{p.x, p.y, p.z};
  const PuckPlanar pl = planar_from_pose(has_q, q, c, yaw_known, is_box);
  const GraspAxis ga = choose_grasp_axis(has_q, q, c);
  const Vec3 g = grasp_point(c, ga.axis, grasp_offset_m_);
  p.x = g.x;
  p.y = g.y;
  p.z = g.z;
  if (has_q) {
    const Quat r = grasp_orientation(pl, ga);
    o.x = r.x;
    o.y = r.y;
    o.z = r.z;
    o.w = r.w;
  }
  const std::string th = pl.theta_known ?
    std::to_string(static_cast<int>(std::lround(pl.theta * 180.0 / M_PI))) + "°" : std::string("모름");
  RCLCPP_INFO(logger(), "퍽 %s 집기: 중심 (%.3f, %.3f, %.3f) θ %s %s → 그리퍼 중심 (%.3f, %.3f, %.3f)",
    puck_id.c_str(), c.x, c.y, c.z, th.c_str(), pl.upright ? "선" : "누움", g.x, g.y, g.z);
}

// 퍽 ID 최근 검출 → 지금 base_frame (최신 TF). 기록 없음 · TF 없음이면 nullopt
std::optional<Executor::PuckLocal> Executor::puck_local(const std::string & id) const
{
  const auto it = local_seen_.find(id);
  if (it == local_seen_.end()) {return std::nullopt;}
  geometry_msgs::msg::TransformStamped t;
  try {
    t = tf_buffer_->lookupTransform(base_frame_, it->second.pose.header.frame_id, tf2::TimePointZero);
  } catch (const tf2::TransformException &) {
    return std::nullopt;
  }
  geometry_msgs::msg::Point b;
  tf2::doTransform(it->second.pose.pose.position, b, t);
  PuckLocal out;
  out.x = b.x;
  out.y = b.y;
  out.z = b.z;
  out.seen = it->second.seen;
  out.oriented_s = it->second.oriented_s;
  out.lying = it->second.lying;
  return out;
}

// in → base_link (최신 TF). with_orientation 이 false 면 위치만 바꾸고 방향은 (0,0,0,0) 유지
bool Executor::to_base(const geometry_msgs::msg::PoseStamped & in, bool with_orientation,
  geometry_msgs::msg::PoseStamped & out)
{
  geometry_msgs::msg::TransformStamped t;
  try {
    t = tf_buffer_->lookupTransform(base_frame_, in.header.frame_id, tf2::TimePointZero,
        tf2::durationFromSec(0.1));
  } catch (const tf2::TransformException & e) {
    detail_ = std::string("목표 좌표 변환 실패: ") + e.what();
    return false;
  }
  if (with_orientation) {
    tf2::doTransform(in.pose, out.pose, t);
  } else {
    tf2::doTransform(in.pose.position, out.pose.position, t);
  }
  return true;
}

bool Executor::transform_pose(const geometry_msgs::msg::PoseStamped & in, const std::string & target,
  geometry_msgs::msg::PoseStamped & out, std::string * why) const
{
  try {
    const auto t = tf_buffer_->lookupTransform(target, in.header.frame_id, tf2::TimePointZero);
    tf2::doTransform(in.pose, out.pose, t);
    out.header.frame_id = target;
    out.header.stamp = in.header.stamp;
  } catch (const tf2::TransformException & e) {
    if (why) {*why = e.what();}
    return false;
  }
  return true;
}

// ================================================================ 감시 입력

std::vector<std::string> Executor::stale_scan_topics(double now) const
{
  std::vector<std::string> out;
  for (std::size_t i = 0; i < scan_topics_.size(); ++i) {
    if (last_scan_s_[i] < 0.0 || now - last_scan_s_[i] > scan_timeout_s_) {out.push_back(scan_topics_[i]);}
  }
  return out;
}

// 출력이 끊긴 카메라 프레임 (빈 배열도 출력으로 봄 = "검출 없음" 과 구분)
std::vector<std::string> Executor::stale_detection_frames(double now) const
{
  std::vector<std::string> out;
  if (detection_frames_.empty()) {
    if (last_any_det_s_ < 0.0 || now - last_any_det_s_ > detection_timeout_s_) {out.push_back("*");}
    return out;
  }
  for (const auto & f : detection_frames_) {
    const auto it = last_det_s_.find(f);
    if (it == last_det_s_.end() || now - it->second > detection_timeout_s_) {out.push_back(f);}
  }
  return out;
}

// map → odom (slam_toolbox · AMCL) 이 최근에 갱신됐나. AMCL 은 transform_tolerance 만큼 미래 시각을 찍으므로 음수 나이는 정상
bool Executor::map_odom_fresh(double now)
{
  try {
    const auto t = tf_buffer_->lookupTransform(map_frame_, odom_frame_, tf2::TimePointZero);
    return now - rclcpp::Time(t.header.stamp).seconds() <= loc_timeout_s_;
  } catch (const tf2::TransformException &) {
    return false;
  }
}

bool Executor::localization_lost(double now)
{
  if (!map_odom_fresh(now)) {return true;}
  if (!loc_from_amcl_) {return loc_health_lost(loc_health_, now, loc_timeout_s_);}
  return pose_std_ &&
         (pose_std_->x > loc_lost_xy_std_ || pose_std_->y > loc_lost_xy_std_ || pose_std_->yaw > loc_lost_yaw_std_);
}

bool Executor::robot_pose(Pose2D & out)
{
  try {
    const auto t = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
    out.x = t.transform.translation.x;
    out.y = t.transform.translation.y;
    const auto & q = t.transform.rotation;
    out.yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w);
    return true;
  } catch (const tf2::TransformException &) {
    return false;
  }
}

// ================================================================ 액션 도우미

// 액션 서버가 안 떠 있을 때 기다린 시간 확인. server_wait_s 를 넘으면 true + detail_
//   - 시작 시각은 첫 실패 때 잼, 보내기 성공 · 단계 종료 · 일시정지 때 초기화
bool Executor::server_wait_expired(double now, const std::string & what)
{
  if (server_wait_start_ < 0.0) {server_wait_start_ = now;}
  if (now - server_wait_start_ <= server_wait_s_) {return false;}
  detail_ = what + " 서버 응답 없음 (" + std::to_string(static_cast<int>(server_wait_s_)) + " s)";
  return true;
}

// Nav2 목표 보내기. 서버가 없으면 false (호출한 쪽이 다음 주기에 다시 부름)
bool Executor::send_nav(const Pose2D & p, bool dock)
{
  auto client = dock ? dock_client_ : nav_client_;
  if (!client->action_server_is_ready()) {
    RCLCPP_WARN_THROTTLE(logger(), *node_.get_clock(), 2000, "%s 서버 대기 중", dock ? dock_action_.c_str() : "Nav2");
    return false;
  }
  nav_cur_client_ = client;
  Nav::Goal goal;
  goal.pose.header.frame_id = map_frame_;
  goal.pose.header.stamp = node_.get_clock()->now();
  goal.pose.pose.position.x = p.x;
  goal.pose.pose.position.y = p.y;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, p.yaw);
  goal.pose.pose.orientation = tf2::toMsg(q);
  nav_goal_pose_ = p;

  nav_ = GoalTrack{};
  nav_.gen = ++gen_;
  nav_.active = true;
  nav_handle_.reset();   // 끝난 이전 목표 핸들로 취소하지 않게 (수락 응답 전 취소는 응답 때 처리)
  nav_.sent_s = now_s();
  nav_.x0 = odom_x_;
  nav_.y0 = odom_y_;
  const uint64_t gen = nav_.gen;

  auto opts = rclcpp_action::Client<Nav>::SendGoalOptions();
  opts.goal_response_callback = [this, gen, client](NavGoalHandle::SharedPtr gh) {
      if (nav_cancel_pending_ && gen == nav_cancel_gen_) {   // 취소한 목표가 늦게 받아들여짐 → 그것도 취소
        if (gh) {client->async_cancel_goal(gh);} else {nav_cancel_pending_ = false;}
        return;
      }
      if (gen != nav_.gen) {return;}
      if (!gh) {
        nav_.done = true;
        nav_.rejected = true;
        nav_.active = false;
        return;
      }
      nav_handle_ = gh;
    };
  opts.result_callback = [this, gen](const NavGoalHandle::WrappedResult & r) {
      if (gen == nav_cancel_gen_) {nav_cancel_pending_ = false;}   // 취소한 목표의 최종 결과 도착
      if (gen != nav_.gen) {return;}
      nav_.done = true;
      nav_.active = false;
      nav_.success = (r.code == rclcpp_action::ResultCode::SUCCEEDED);
      nav_.end_s = now_s();
      nav_.error_code = r.result ? static_cast<int>(r.result->error_code) : 0;
    };
  client->async_send_goal(goal, opts);
  return true;
}

StepResult Executor::poll_nav(double now, double timeout_s)
{
  if (nav_.done) {
    if (nav_.success) {return StepResult::Success;}
    detail_ = nav_.rejected ? "Nav2 목표 거절" : "Nav2 이동 실패";
    return StepResult::Fail;
  }
  if (now - nav_.sent_s > timeout_s) {
    cancel_nav();
    detail_ = "Nav2 시간 초과";
    return StepResult::Fail;
  }
  return StepResult::Running;
}

bool nav_path_failure(int error_code, double took_s, double moved_m, double quick_s, double still_m)
{
  if (error_code >= 200 && error_code < 300) {return error_code != 201;}   // ComputePathToPose (bt_navigator)
  if (error_code != 0) {return false;}
  return took_s <= quick_s && moved_m <= still_m;   // rl_nav '경로 못 받음' = 움직이기 전 바로 실패
}

// 1 단계 Nav2 실패 (poll_nav Fail) 뒤: 경로 계획 실패면 path_retry_wait_s 기다렸다 같은 목표 다시 (run_navigate 가 다시 보냄)
//   그 단계 첫 경로 실패부터 path_retry_total_s 까지. 단계 재시도 (nav_retries) · 미션 재계획 (max_replans) 을 안 씀
//   다 쓰면 false → 보통 실패 처리. 셈은 이동 단계 성공 · 미션에 넘긴 실패 · set_steps · stop 에서 비움 (단계 재시도 · 끼운 HOME 사이에는 이어 셈)
bool Executor::path_retry(const Step & s, double now)
{
  if (!(path_retry_total_s_ > 0.0) || !nav_.done || nav_.rejected || nav_.success) {return false;}
  const double took = nav_.end_s - nav_.sent_s;
  const double moved = std::hypot(odom_x_ - nav_.x0, odom_y_ - nav_.y0);
  if (!nav_path_failure(nav_.error_code, took, moved, path_fail_quick_s_, path_fail_still_m_)) {return false;}
  if (path_retry_.name != s.name) {
    path_retry_ = PathRetry{};
    path_retry_.name = s.name;
    path_retry_.since = now;
  }
  const double used = now - path_retry_.since;
  double wait = path_retry_wait_s_.empty() ? 5.0 :
    path_retry_wait_s_[std::min<std::size_t>(static_cast<std::size_t>(path_retry_.n), path_retry_wait_s_.size() - 1)];
  wait = std::min(std::max(wait, 0.0), path_retry_total_s_ - used);
  if (!(wait > 0.0)) {
    if (!path_retry_.spent) {
      RCLCPP_WARN(logger(), "%s: 경로 계획 실패 — 기다렸다 다시 %d 번, %.0f s 다 씀 → 단계 재시도 · 재계획",
        s.name.c_str(), path_retry_.n, used);
    }
    path_retry_.spent = true;
    return false;
  }
  ++path_retry_.n;
  path_retry_.until = now + wait;
  step_running_ = false;
  RCLCPP_INFO(logger(), "%s: 경로 계획 실패 (%s, %.1f s 만에 · %.0f cm 움직임, 코드 %d) → %.0f s 기다렸다 다시 계획 "
    "(%d 번째, %.0f / %.0f s, 단계 재시도 · 재계획 횟수 안 씀)",
    s.name.c_str(), detail_.c_str(), took, moved * 100.0, nav_.error_code, wait, path_retry_.n, used, path_retry_total_s_);
  return true;
}

// 취소: 진행 중이던 목표는 최종 결과(또는 거절)를 받을 때까지 *_cancel_pending_ 으로 추적 (resume 조건)
void Executor::cancel_nav()
{
  if (nav_.active) {
    nav_cancel_gen_ = nav_.gen;
    nav_cancel_pending_ = true;
    if (nav_handle_) {nav_cur_client_->async_cancel_goal(nav_handle_);}   // 아직 응답 전이면 받아들여질 때 취소
  }
  if (nav_docking_) {commit_dock_fix();}   // 시간 초과 · FAULT 로 끊은 정밀 접근도 잰 것은 남김
  nav_docking_ = false;
  nav_ = GoalTrack{};
  nav_.gen = ++gen_;   // 늦게 오는 결과 무시
  nav_handle_.reset();
}

bool Executor::send_arm(const ArmTask::Goal & goal)
{
  if (!arm_client_->action_server_is_ready()) {
    RCLCPP_WARN_THROTTLE(logger(), *node_.get_clock(), 2000, "/arm_task 서버 대기 중");
    return false;
  }
  keep_unapplied_arm_ok();
  arm_ = GoalTrack{};
  arm_.gen = ++gen_;
  arm_.active = true;
  arm_handle_.reset();   // 끝난 이전 목표 핸들로 취소하지 않게 (수락 응답 전 취소는 응답 때 처리)
  arm_.sent_s = now_s();
  arm_phase_.clear();
  arm_release_seen_ = false;
  const uint64_t gen = arm_.gen;

  auto opts = rclcpp_action::Client<ArmTask>::SendGoalOptions();
  opts.goal_response_callback = [this, gen](ArmGoalHandle::SharedPtr gh) {
      // 취소한 목표가 늦게 받아들여짐 → 그것도 취소 (서버 없어짐으로 대기를 지웠어도 살아 있으니 다시 기다림)
      if (gen == arm_cancel_gen_) {
        if (gh) {
          arm_cancel_pending_ = true;
          arm_needs_home_ = true;   // 팔이 움직였을 수 있음
          arm_client_->async_cancel_goal(gh);
        } else {
          arm_cancel_pending_ = false;
          arm_cancel_step_.reset();
        }
        return;
      }
      if (gen != arm_.gen) {return;}
      if (!gh) {
        arm_.done = true;
        arm_.rejected = true;
        arm_.active = false;
        return;
      }
      arm_handle_ = gh;
      arm_needs_home_ = true;   // 팔이 움직이기 시작함 → 성공으로 끝나야 접힌 것으로 봄
    };
  opts.feedback_callback = [this, gen](ArmGoalHandle::SharedPtr, const std::shared_ptr<const ArmTask::Feedback> fb) {
      if (gen == arm_.gen) {
        arm_phase_ = fb->phase;
        if (fb->phase == "release") {arm_release_seen_ = true;}   // 재고 불확실 스스로 정합: 놓기까지 갔나
      }
    };
  opts.result_callback = [this, gen](const ArmGoalHandle::WrappedResult & r) {
      if (gen == arm_cancel_gen_) {   // 취소한 목표의 최종 결과 도착
        arm_cancel_pending_ = false;
        const bool ok = (r.code == rclcpp_action::ResultCode::SUCCEEDED) && r.result && r.result->success;
        if (ok && arm_cancel_step_) {late_arm_ok_ = arm_cancel_step_;}
        arm_cancel_step_.reset();
      }
      if (gen != arm_.gen) {return;}
      arm_phase_.clear();
      arm_.done = true;
      arm_.active = false;
      arm_.success = (r.code == rclcpp_action::ResultCode::SUCCEEDED) && r.result && r.result->success;
      arm_.message = r.result ? r.result->message : "";
      if (arm_.success) {arm_needs_home_ = false;}   // 성공한 작업은 검사된 자세로 끝남 (펴진 채면 arm_unfolded_)
    };
  arm_client_->async_send_goal(goal, opts);
  return true;
}

void Executor::cancel_arm()
{
  if (arm_.active) {
    arm_cancel_gen_ = arm_.gen;
    arm_cancel_pending_ = true;
    arm_cancel_step_ = arm_step_;
    arm_needs_home_ = true;   // 수락 응답 전이라도 움직였을 수 있음 → 다음 단계 전에 HOME (접혀 있으면 바로 성공)
    if (arm_handle_) {arm_client_->async_cancel_goal(arm_handle_);}
  }
  keep_unapplied_arm_ok();
  arm_ = GoalTrack{};
  arm_.gen = ++gen_;
  arm_handle_.reset();
}

// arm_ 을 비우기 전: 성공 결과는 왔는데 step() 이 아직 재고에 반영 안 함 → 늦은 성공으로 넘김 (update() 가 한 번 반영)
void Executor::keep_unapplied_arm_ok()
{
  if (arm_.done && arm_.success && arm_step_) {
    late_arm_ok_ = arm_step_;
    arm_unfolded_ = leaves_arm_unfolded(*arm_step_);
  }
  arm_step_.reset();
}

// ================================================================ 기타

// Nav2 lifecycle_manager is_active 확인 (요청은 서비스마다 하나만, 2 s 넘게 답 없으면 버리고 다시)
void Executor::poll_lifecycle()
{
  const double now = now_s();
  for (std::size_t i = 0; i < lifecycle_.size(); ++i) {
    auto & lc = lifecycle_[i];
    if (lc.pending) {
      if (now - lc.sent_s < 2.0) {continue;}
      lc.cli->prune_pending_requests();
      lc.pending = false;
    }
    if (!lc.cli->service_is_ready()) {
      lc.ok = false;
      continue;
    }
    lc.pending = true;
    lc.sent_s = now;
    lc.cli->async_send_request(std::make_shared<Trigger::Request>(),
      [this, i](rclcpp::Client<Trigger>::SharedFuture f) {
        auto & c = lifecycle_[i];
        c.pending = false;
        const auto r = f.get();
        c.ok = r && r->success;
        c.checked_s = now_s();
      });
  }
}

void Executor::cargo_changed()
{
  save_cargo();
  publish_cargo();
}

// 그리퍼 기록만 지움. 재고 불확실이면 거절 (불확실 정합은 /mission/inventory_* 로만, 여기서 같이 지우지 않음)
bool Executor::clear_gripper(std::string & why)
{
  if (cargo_.doubt()) {
    why = "재고 불확실 (" + doubt_text() + ") → /mission/inventory_* 로 먼저 정합";
    return false;
  }
  cargo_.clear_gripper();
  cargo_changed();
  why = "그리퍼 기록 지움";
  return true;
}

bool Executor::record_store(const std::string & slot, std::string & msg)
{
  const std::string pending = cancel_pending_reason();
  if (arm_.active || arm_timeout_wait_ || !pending.empty()) {
    msg = (pending.empty() ? std::string("팔 작업 최종 결과 기다리는 중") : pending) + " → 결과 뒤 다시";
    return false;
  }
  if (cargo_.doubt()) {
    msg = "재고 불확실 (" + doubt_text() + ") → /mission/inventory_* 로 먼저 정합";
    return false;
  }
  if (!cargo_.holding()) {
    msg = "그리퍼 기록이 비어 있음 (넣을 퍽 기록 없음)";
    return false;
  }
  Step s;
  s.type = StepType::Arm;
  s.arm_task = kArmStore;
  s.puck_id = cargo_.gripper()->puck_id;
  s.slot = slot;
  s.name = "manual:STORE:" + s.puck_id + ">" + slot;
  std::string why;
  if (!update_cargo(s, why)) {
    msg = "기록 못 함: " + why;
    return false;
  }
  msg = "그리퍼 퍽 " + s.puck_id + " → " + slot + " 기록 (수동 STORE)";
  RCLCPP_WARN(logger(), "%s", msg.c_str());
  return true;
}

void Executor::save_cargo()
{
  if (cargo_file_.empty()) {return;}
  std::string serr;
  if (!cargo_.save(cargo_file_, &serr)) {
    RCLCPP_WARN(logger(), "재고 기록 저장 실패: %s", serr.c_str());
  }
}

void Executor::publish_cargo()
{
  auto to_msg = [](const std::string & slot, const std::optional<CargoItem> & it) {
      CargoSlot m;
      m.slot = slot;
      m.occupied = it.has_value();
      if (it) {
        m.puck_id = it->puck_id;
        m.color = it->color;
        m.shape = it->shape;
        m.task_id = it->task_id;
        m.verified = it->verified;
      }
      return m;
    };
  CargoState msg;
  msg.header.stamp = node_.get_clock()->now();
  for (const auto & s : cargo_.slots()) {msg.slots.push_back(to_msg(s.first, s.second));}
  msg.gripper = to_msg("gripper", cargo_.gripper());
  pub_cargo_->publish(msg);
}

// RViz: 퍽 기록부(색 구 + ID 글자), 후보(회색 구), 현재 Nav2 목표(화살표)
void Executor::publish_markers()
{
  visualization_msgs::msg::MarkerArray arr;
  visualization_msgs::msg::Marker clear;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  arr.markers.push_back(clear);

  const auto stamp = node_.get_clock()->now();
  int id = 0;
  auto base = [&](const std::string & ns, int type) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = map_frame_;
      m.header.stamp = stamp;
      m.ns = ns;
      m.id = id++;
      m.type = type;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.orientation.w = 1.0;
      return m;
    };
  auto color = [](visualization_msgs::msg::Marker & m, const std::string & label) {
      m.color.a = 1.0;
      if (label == "red") {m.color.r = 0.9; m.color.g = 0.1; m.color.b = 0.1;}
      else if (label == "green") {m.color.r = 0.1; m.color.g = 0.8; m.color.b = 0.2;}
      else {m.color.r = 0.6; m.color.g = 0.6; m.color.b = 0.6;}
    };

  for (const auto & r : registry_.records()) {
    auto s = base("pucks", visualization_msgs::msg::Marker::CYLINDER);
    s.pose.position.x = r.x;
    s.pose.position.y = r.y;
    s.pose.position.z = r.z;
    s.scale.x = s.scale.y = 0.03;
    s.scale.z = 0.06;
    color(s, r.color);
    if (r.state != "on_field") {s.color.a = 0.35;}
    arr.markers.push_back(s);

    auto t = base("puck_ids", visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
    t.pose.position.x = r.x;
    t.pose.position.y = r.y;
    t.pose.position.z = r.z + 0.08;
    t.scale.z = 0.05;
    t.color.r = t.color.g = t.color.b = t.color.a = 1.0;
    t.text = r.id + " " + r.state;
    arr.markers.push_back(t);
  }
  for (const auto & c : registry_.candidates()) {
    auto s = base("candidates", visualization_msgs::msg::Marker::SPHERE);
    s.pose.position.x = c.x;
    s.pose.position.y = c.y;
    s.pose.position.z = c.z;
    s.scale.x = s.scale.y = s.scale.z = 0.03;
    color(s, c.color);
    s.color.a = 0.5;
    arr.markers.push_back(s);
  }
  if (nav_.active) {
    auto a = base("nav_goal", visualization_msgs::msg::Marker::ARROW);
    a.pose.position.x = nav_goal_pose_.x;
    a.pose.position.y = nav_goal_pose_.y;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, nav_goal_pose_.yaw);
    a.pose.orientation = tf2::toMsg(q);
    a.scale.x = 0.2;
    a.scale.y = a.scale.z = 0.03;
    a.color.r = 0.2; a.color.g = 0.5; a.color.b = 1.0; a.color.a = 1.0;
    arr.markers.push_back(a);
  }
  pub_markers_->publish(arr);
}

// ================================================================ 위치 추정 상실 복구 (loc_recovery)

namespace
{
double wrap_pi(double a) {return std::atan2(std::sin(a), std::cos(a));}
}  // namespace

bool Executor::lookup_2d(const std::string & parent, const std::string & child, Pose2D & out)
{
  try {
    const auto t = tf_buffer_->lookupTransform(parent, child, tf2::TimePointZero);
    out.x = t.transform.translation.x;
    out.y = t.transform.translation.y;
    const auto & q = t.transform.rotation;
    out.yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w);
    return true;
  } catch (const tf2::TransformException &) {
    return false;
  }
}

void Executor::track_map_odom(double now)
{
  Pose2D mo;
  if (!lookup_2d(map_frame_, odom_frame_, mo)) {return;}
  map_odom_hist_.push_back({now, mo});
  while (!map_odom_hist_.empty() && now - map_odom_hist_.front().t > loc_stable_window_s_) {map_odom_hist_.pop_front();}
}

// 창 (loc_stable_window_s) 안 map → odom 이 거의 안 바뀜 = slam 이 큰 보정을 안 하는 중. 창이 덜 찼으면 안정 아님
bool Executor::map_odom_stable(double now) const
{
  if (map_odom_hist_.size() < 2 || now - map_odom_hist_.front().t < 0.8 * loc_stable_window_s_) {return false;}
  const Pose2D & last = map_odom_hist_.back().p;
  for (const auto & h : map_odom_hist_) {
    if (std::hypot(h.p.x - last.x, h.p.y - last.y) > loc_stable_xy_m_ ||
      std::fabs(wrap_pi(h.p.yaw - last.yaw)) > loc_stable_yaw_) {return false;}
  }
  return true;
}

// 물러나기 = 팔 접힘 · 빈 그리퍼 · DRIVE · 이동 단계 (또는 단계 없음) 일 때만. 박스 앞 팔 작업 중이면 안 함 (그 자리에서 대기 · 조건부 계속)
bool Executor::can_backoff() const
{
  if (good_hist_.empty() || arm_.active || arm_unfolded_ || cargo_.holding() || waiting_drive_) {return false;}
  if (w_drive_ || scan_mon_.active()) {return false;}
  if (require_motion_mode_ && !(have_mode_ && mode_ == MotionMode::DRIVE)) {return false;}
  if (step_i_ < steps_.size() && steps_[step_i_].type != StepType::Navigate) {return false;}
  return true;
}

// 물러날 자리 = 상실 loc_backoff_lookback_s 전 정상 자세 (odom, 없으면 가장 오래된 기록) → 지금 map → odom 으로 map 목표.
//   지금 자세에서 그쪽으로 최대 loc_backoff_max_m, 방향은 지금 그대로 (회전 안 함)
bool Executor::backoff_goal(Pose2D & out)
{
  Pose2D mo, cur;
  if (good_hist_.empty() || !lookup_2d(map_frame_, odom_frame_, mo) || !robot_pose(cur)) {return false;}
  Pose2D g = good_hist_.front().p;
  for (const auto & h : good_hist_) {
    if (h.t <= loc_lost_since_ - loc_backoff_lookback_s_) {g = h.p;}
  }
  const double c = std::cos(mo.yaw), sn = std::sin(mo.yaw);
  Pose2D gm;
  gm.x = mo.x + c * g.x - sn * g.y;
  gm.y = mo.y + sn * g.x + c * g.y;
  const double d = std::hypot(gm.x - cur.x, gm.y - cur.y);
  if (d < loc_backoff_min_m_) {return false;}
  const double k = std::min(1.0, loc_backoff_max_m_ / d);
  out.x = cur.x + k * (gm.x - cur.x);
  out.y = cur.y + k * (gm.y - cur.y);
  out.yaw = cur.yaw;
  return true;
}

void Executor::update_loc_recovery(double now)
{
  const bool lost = require_localization_ && localization_lost(now);
  track_map_odom(now);
  const bool stable = map_odom_stable(now);
  if (!lost && loc_rec_.mode() == LocMode::Ok) {   // 정상: 자세 (odom 기준) 기록, 최근 loc_backoff_lookback_s + 3 s 만
    Pose2D ob;
    if (lookup_2d(odom_frame_, base_frame_, ob)) {good_hist_.push_back({now, ob});}
    while (!good_hist_.empty() && now - good_hist_.front().t > loc_backoff_lookback_s_ + 3.0) {good_hist_.pop_front();}
  }
  if (lost && loc_rec_.mode() == LocMode::Ok) {loc_lost_since_ = now;}
  if (loc_rec_.update(lost, stable, can_backoff(), now)) {
    rec_pending_ = true;
    rec_start_s_ = now;
    stop_blocked_nav();   // 단계의 이동 목표 취소 (회복 뒤 그 단계를 처음부터)
  }
  const LocMode m = loc_rec_.mode();
  if (m != loc_mode_logged_) {
    if (m == LocMode::Ok) {
      RCLCPP_INFO(logger(), "위치 복구: 정상 (%s 에서)", loc_mode_name(loc_mode_logged_));
    } else {
      RCLCPP_WARN(logger(), "위치 복구: %s (상실 %.1f s, 물러나기 %d 번, map→odom %s)", loc_mode_name(m),
        loc_rec_.lost_for(now), loc_rec_.backoffs(), stable ? "안정" : "불안정");
    }
    loc_mode_logged_ = m;
  }
  // 물러나기가 더는 필요 없음 (정상 회복 등) → 이동 멈춤
  if ((rec_pending_ || rec_nav_active_) && m != LocMode::Backoff) {
    if (rec_nav_active_) {cancel_nav();}
    rec_pending_ = false;
    rec_nav_active_ = false;
    return;
  }
  if (rec_pending_) {
    if (cancel_blocked()) {   // 앞 목표 취소 결과 대기
      if (now - rec_start_s_ > loc_backoff_timeout_s_) {
        RCLCPP_WARN(logger(), "위치 복구: 앞 목표 취소 결과 안 옴 → 물러나기 건너뜀");
        rec_pending_ = false;
        loc_rec_.backoff_done(now);
      }
      return;
    }
    if (!backoff_goal(rec_goal_)) {
      RCLCPP_WARN(logger(), "위치 복구: 마지막 정상 자리가 가깝거나 모름 → 물러나기 건너뜀");
      rec_pending_ = false;
      loc_rec_.backoff_done(now);
      return;
    }
    if (!send_nav(rec_goal_)) {
      if (now - rec_start_s_ > loc_backoff_timeout_s_) {rec_pending_ = false; loc_rec_.backoff_done(now);}
      return;
    }
    RCLCPP_WARN(logger(), "위치 복구: 마지막 정상 자리 쪽으로 물러남 → (%.2f, %.2f)", rec_goal_.x, rec_goal_.y);
    rec_pending_ = false;
    rec_nav_active_ = true;
    return;
  }
  if (rec_nav_active_) {
    const bool timed_out = now - rec_start_s_ > loc_backoff_timeout_s_;
    if (nav_.done || timed_out) {
      RCLCPP_WARN(logger(), "위치 복구: 물러나기 %s", nav_.done ? (nav_.success ? "끝" : "실패") : "시간 초과");
      if (!nav_.done) {cancel_nav();}
      rec_nav_active_ = false;
      loc_rec_.backoff_done(now);
    }
  }
}

}  // namespace planning
