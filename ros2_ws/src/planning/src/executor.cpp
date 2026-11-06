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
  det_mon_ = ConditionMonitor(mon_recover_s, mon_max, mon_fault_s);
  // 위치 추정: 잠깐(loc_fault_s) 안에 돌아오면 계속, 아니면 FAULT (데이터만 다시 와도 자동 재개 안 함)
  loc_mon_ = ConditionMonitor(0.0, 3, node_.declare_parameter<double>("loc_fault_s", 1.0));
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
  server_wait_s_ = node_.declare_parameter<double>("server_wait_s", 30.0);
  nav_timeout_s_ = node_.declare_parameter<double>("nav_timeout_s", 60.0);
  arm_timeout_s_ = node_.declare_parameter<double>("arm_timeout_s", 60.0);
  drive_wait_s_ = node_.declare_parameter<double>("drive_wait_s", 10.0);

  puck_standoff_m_ = node_.declare_parameter<double>("puck_standoff_m", 0.30);
  dock_action_ = node_.declare_parameter<std::string>("dock_action", "dock_to_pose");
  dock_nav_back_m_ = node_.declare_parameter<double>("dock_nav_back_m", 0.025);
  dock_timeout_s_ = node_.declare_parameter<double>("dock_timeout_s", 40.0);
  dock_handoff_m_ = node_.declare_parameter<double>("dock_handoff_m", 0.40);
  pick_fresh_s_ = node_.declare_parameter<double>("pick_fresh_s", 2.0);
  puck_half_height_m_ = node_.declare_parameter<double>("puck_half_height_m", 0.03);
  grasp_offset_m_ = node_.declare_parameter<double>("grasp_offset_m", 0.015);

  nav_retries_ = node_.declare_parameter<int>("nav_retries", 1);
  pick_retries_ = node_.declare_parameter<int>("pick_retries", 2);
  pick_relook_ = node_.declare_parameter<bool>("pick_relook", true);
  relook_wait_s_ = node_.declare_parameter<double>("relook_wait_s", 1.5);
  place_retries_ = node_.declare_parameter<int>("place_retries", 1);
  require_motion_mode_ = node_.declare_parameter<bool>("require_motion_mode", true);
  // ---- 적재함 칸 측정 (LOOK_RACK 뒤 기다리는 시간, STORE 에 쓸 수 있는 측정 나이)
  rack_wait_s_ = node_.declare_parameter<double>("rack_wait_s", 2.0);
  rack_fresh_s_ = node_.declare_parameter<double>("rack_fresh_s", 30.0);

  // ---- 재고: 칸 이름 (팔이 닿는 칸만), 기록 파일 (빈 문자열 = 저장 안 함)
  cargo_ = Cargo(node_.declare_parameter<std::vector<std::string>>("cargo_slots",
    {"S1", "S2", "S3", "S4", "S5", "S6", "S7", "S8", "S9", "S10", "S11", "S12"}));
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
  for (const auto & t : det_topics) {
    sub_det_.push_back(node_.create_subscription<DetectionArray>(
      t, 10, [this](DetectionArray::ConstSharedPtr m) {on_detections(*m);}));
  }
  sub_rack_ = node_.create_subscription<DetectionArray>(
    node_.declare_parameter<std::string>("rack_topic", "/perception/rack/cells"), 10,
    [this](DetectionArray::ConstSharedPtr m) {on_rack(*m);});
  sub_mode_ = node_.create_subscription<MotionMode>(
    "/control/motion_mode", rclcpp::QoS(1).reliable().transient_local(),
    [this](MotionMode::ConstSharedPtr m) {
      have_mode_ = true;
      mode_ = m->mode;
      mode_ready_ = m->ready;
      mode_reason_ = m->reason;
      last_mode_s_ = now_s();
    });
  sub_odom_ = node_.create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10, [this](nav_msgs::msg::Odometry::ConstSharedPtr) {last_odom_s_ = now_s();});
  last_scan_s_.assign(scan_topics_.size(), -1.0);
  for (std::size_t i = 0; i < scan_topics_.size(); ++i) {
    sub_scan_.push_back(node_.create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topics_[i], rclcpp::SensorDataQoS(), [this, i](sensor_msgs::msg::LaserScan::ConstSharedPtr) {last_scan_s_[i] = now_s();}));
  }
  // AMCL 은 위치를 갱신할 때만 발행 (정지 중엔 안 옴) → 마지막 유효 공분산을 유지
  //   /initialpose 는 따로 받지 않음: AMCL 이 초기 위치를 반영한 /amcl_pose 를 스스로 다시 내므로 마지막 값만 믿음
  //   (예전엔 /initialpose 를 받으면 지웠는데, 그 뒤 AMCL 이 다시 안 내면 정지 중엔 LOCALIZATION_COV 로 계속 막힘
  //    — Isaac 시험 2026-11-02 에서 두 번 발생)
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
    registry_.update(r);   // id 가 "" 이면 후보로 들어감
    raw_obs_.push_back(ObsRecord{rclcpp::Time(h.stamp).seconds(), d.id, d.label, d.shape, out.x, out.y});
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
  const auto stale_scans = require_scan_ ? stale_scan_topics(now) : std::vector<std::string>{};
  if (scan_mon_.update(!stale_scans.empty(), now) == Health::Fault) {
    std::string which;
    for (const auto & t : stale_scans) {which += (which.empty() ? "" : ", ") + t;}
    return fault("SCAN_LOST", "라이다 끊김 회복 안 됨: " + which);
  }
  if (det_mon_.update(require_detections_ && !stale_detection_frames(now).empty(), now) == Health::Fault) {
    return fault("DETECTION_LOST", "검출 출력 끊김 회복 안 됨");
  }
  if (loc_mon_.update(require_localization_ && localization_lost(now), now) == Health::Fault) {
    return fault("LOCALIZATION_LOST", "위치 추정 상실");
  }
  // 실행 중 재고 불확실 표시가 생김 (늦은 최종 성공이 기록과 안 맞음 등) → 더 조작하지 않음
  if (cargo_.doubt()) {
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
  if (step_i_ >= steps_.size()) {return ev;}

  // 대기 원인으로 막힌 단계는 시작 · 진행 안 함
  const Step & cur = steps_[step_i_];
  const bool drive_step = cur.type == StepType::Navigate;
  // 퍽 쥔 채 이동 단계 = 계획 오류 (2026-11-04: 쥔 채 주행 안 함, control 도 쥔 채 HOME 거절) → 이동 안 하고 FAULT
  if (drive_step && !step_running_ && cargo_.holding()) {
    ev.kind = ExecEvent::Kind::Fault;
    ev.code = "CARRY_DRIVE";
    ev.detail = "퍽 " + cargo_.gripper()->puck_id + " 쥔 채 이동 단계: " + cur.name + " (STORE · PLACE 로 그리퍼를 비운 뒤 이동해야 함)";
    return ev;
  }
  // LOOK · LOOK_BARCODE 뒤 팔이 펴진 채 이동하려 함 → 기다리지 않고 바로 HOME (control 은 접힘 실측 뒤에만 DRIVE)
  if (drive_step && arm_unfolded_ && !step_running_ && require_motion_mode_) {
    arm_unfolded_ = false;
    insert_home_at_current("LOOK 뒤 이동 전 접기");
    return ev;
  }
  if (update_drive_reason(now, drive_step, ev)) {return ev;}   // HOME 끼워 넣음 (또는 Fault)
  if (drive_step && nav_blocked()) {
    stop_blocked_nav();
    return ev;
  }
  // 새 팔 목표 보내기 전만 막음 (거절 뒤 다시 보내기 포함). 보낸 작업은 그대로 두고 supervise 가 회복 · FAULT 판단
  if (cur.type == StepType::Arm && !step_running_ &&
    arm_start_blocked(cur.arm_task, cur.target, det_mon_.active(), loc_mon_.active()))
  {
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
  }
  return ev;
}

// LOOK_RACK 성공 뒤: 팔 결과 뒤 촬영한 그 칸 측정을 rack_wait_s 동안 기다림
//   - perception 은 카메라가 멈춘 장만 세고 5 장 중 3 장이면 확정 → 결과 뒤 0.3–0.5 s 면 옴
//   - 못 받아도 단계는 성공 (STORE 는 녹화 경로 그대로), 팔은 퍽 쥔 채 관찰 자세 = 펴진 LOOK 류로 안 봄 (다음이 STORE)
StepResult Executor::wait_rack(const Step & s, double now)
{
  if (rack_wait_start_ < 0.0) {
    rack_wait_start_ = now;
    rack_meas_.erase(s.slot);
  }
  const auto it = rack_seen_.find(s.slot);
  const bool got = it != rack_seen_.end() && it->second.stamp >= rack_wait_start_;
  if (!got && now - rack_wait_start_ <= rack_wait_s_) {return StepResult::Running;}
  if (got) {
    rack_meas_[s.slot] = it->second;
    RCLCPP_INFO(logger(), "%s: 칸 측정 받음 (%.1f s)", s.name.c_str(), now - rack_wait_start_);
  } else {
    RCLCPP_WARN(logger(), "%s: %.1f s 안에 칸 측정 못 받음 → 측정 없이 STORE", s.name.c_str(), rack_wait_s_);
  }
  rack_wait_start_ = -1.0;
  arm_unfolded_ = false;
  return StepResult::Success;
}

void Executor::on_rack(const DetectionArray & msg)
{
  for (const auto & d : msg.detections) {
    if (d.kind != "rack_cell" || d.label.empty()) {continue;}
    RackSeen r;
    r.pose.header = d.header.frame_id.empty() ? msg.header : d.header;
    r.pose.pose = d.pose;
    r.stamp = rclcpp::Time(r.pose.header.stamp).seconds();
    rack_seen_[d.label] = r;
  }
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
  // 재고를 바꾸는 팔 작업이 수락된 뒤 성공으로 안 끝남 → 퍽 위치 모름 → 재시도 · 재계획 없이 FAULT
  //   (재시도하면 쥔 퍽을 떨구거나 같은 퍽을 두 번 집을 수 있음, 미션이 lost 로 단정하면 기록이 실물과 어긋남)
  if (s.type == StepType::Arm && inventory_doubt_after(s.arm_task, arm_end_)) {
    mark_doubt(s, detail_);
    ev.kind = ExecEvent::Kind::Fault;
    ev.code = "INVENTORY_UNCERTAIN";
    ev.detail = "재고 불확실: " + s.name + " (" + detail_ + ") → 실물 확인 뒤 /mission/inventory_* 로 정합";
    return;
  }
  // 팔이 움직이다 성공으로 안 끝남 → 접혔는지 모름, 바퀴 잠김 → 다음 단계 전에 HOME
  const bool need_home = arm_needs_home_;
  const int allowed = retries_allowed(s);
  if (retries_[step_i_] < allowed) {
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
  rack_wait_start_ = -1.0;
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
  return w_drive_ || scan_mon_.active() || loc_mon_.active();
}

std::vector<std::string> Executor::wait_reasons() const
{
  std::vector<std::string> r;
  if (w_drive_) {r.push_back("DRIVE_LOST");}
  if (scan_mon_.active()) {r.push_back("SCAN_LOST");}
  if (det_mon_.active()) {r.push_back("DETECTION_LOST");}
  if (loc_mon_.active()) {r.push_back("LOCALIZATION_LOST");}
  return r;
}

void Executor::reset_monitors()
{
  scan_mon_.reset();
  det_mon_.reset();
  loc_mon_.reset();
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
  // 팔 결과는 왔는데 step() 이 아직 처리 안 함 (supervise FAULT 가 먼저) → 그 결과대로 반영. 버리면 재고가 실물과 어긋남
  const bool arm_step_running = step_running_ && step_i_ < steps_.size() && steps_[step_i_].type == StepType::Arm && arm_step_;
  if (arm_step_running && arm_.done && !arm_.rejected && !arm_timeout_wait_) {
    if (arm_.success) {
      RCLCPP_WARN(logger(), "FAULT 직전 끝난 팔 작업 성공 → 재고 반영: %s", arm_step_->name.c_str());
      std::string why;
      if (!update_cargo(*arm_step_, why)) {mark_doubt(*arm_step_, "FAULT 직전 끝난 팔 작업 성공인데 재고 기록 불일치: " + why);}
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
  rack_wait_start_ = -1.0;
  arm_timeout_wait_ = false;   // 늦은 성공은 update() 가 재고에 반영
  cancel_nav();
  cancel_arm();
  w_drive_ = false;
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
      // 정차점까지 dock_handoff_m 안에 들어오면 서지 않고 넘김: Nav2 취소 → 취소 결과 뒤 바로 정밀 접근
      Pose2D rp, fin;
      if (two_stage && dock_handoff_m_ > 0.0 && nav_.active && navigate_goal(s, fin) && robot_pose(rp) &&
        std::hypot(rp.x - fin.x, rp.y - fin.y) < dock_handoff_m_)
      {
        cancel_nav();
      } else {
        const StepResult r = poll_nav(now, s.timeout_s > 0.0 ? s.timeout_s : nav_timeout_s_);
        if (r != StepResult::Success || !two_stage) {return r;}
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
    return StepResult::Running;
  }
  const StepResult r = poll_nav(now, dock_timeout_s_);
  if (r == StepResult::Fail) {detail_ = "정밀 접근: " + detail_;}
  if (r != StepResult::Running) {nav_docking_ = false;}
  return r;
}

// 단계 목표 → 지도 위 자세
bool Executor::navigate_goal(const Step & s, Pose2D & goal)
{
  switch (s.target) {
    case Target::Fixed:
      goal = s.pose;
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

// 이 팔 작업이 성공하면 팔이 펴진 채 남나 (control 은 LOOK 류만 그 자세 유지, 그 밖은 home 으로 끝냄)
bool Executor::leaves_arm_unfolded(const Step & s)
{
  return s.type == StepType::Arm && (is_look_task(s.arm_task) || s.arm_task == kArmLookBarcode);
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
    server_wait_start_ = -1.0;
    step_running_ = true;
    step_start_s_ = now;
    return StepResult::Running;
  }
  // LOOK_RACK 거절 (control 구현 전 등) → 측정 없이 STORE 로 넘어감, 팔은 안 움직였음
  if (arm_.done && arm_.rejected && s.arm_task == kArmLookRack) {
    RCLCPP_WARN(logger(), "%s 거절 (%s) → 칸 측정 없이 STORE", s.name.c_str(), arm_.message.c_str());
    rack_meas_.erase(s.slot);
    return StepResult::Success;
  }
  // 거절 → action_retry_s 뒤 다시 보냄 (켤 때 팔 재생기 준비 기다리기)
  if (arm_.done && arm_.rejected) {
    if (now - arm_try_start_ > action_retry_max_s_) {
      detail_ = "팔 작업 계속 거절됨";
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
      arm_needs_home_ = false;   // control 의 팔 작업은 성공하면 home 자세로 끝남 (LOOK 류만 그 자세 유지)
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
  if (s.arm_task == kArmLookRack) {return wait_rack(s, now);}
  // 성공 → 퍽 기록부 · 재고 갱신 (팔 결과로만 바꿈)
  arm_unfolded_ = leaves_arm_unfolded(s);
  if (is_look_task(s.arm_task) && !s.puck_id.empty()) {relook_after_[s.puck_id] = now;}   // 다시 보기 LOOK
  if (s.arm_task == kArmPick) {relook_after_.erase(s.puck_id);}
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
  if (s.arm_task == kArmStore) {rack_meas_.erase(s.slot);}   // 한 번 쓰면 버림 (다음엔 다시 잼)
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

// 사람 정합: 팔 작업이 끝났을 때만 (진행 중이면 실물이 아직 바뀔 수 있음)
bool Executor::resolve_inventory(DoubtFix fix, std::string & msg)
{
  const std::string pending = cancel_pending_reason();
  if (arm_.active || arm_timeout_wait_ || !pending.empty()) {
    msg = (pending.empty() ? std::string("팔 작업 최종 결과 기다리는 중") : pending) + " → 결과 뒤 다시";
    return false;
  }
  const std::string before = doubt_text();
  std::string id, state, err;
  if (!resolve_doubt(cargo_, fix, &id, &state, &err)) {
    msg = "정합 못 함 (" + std::string(doubt_fix_name(fix)) + "): " + err;
    return false;
  }
  if (!state.empty()) {registry_.set_state(id, state);}
  // 사람 확인이 기준: 서버가 사라져 못 받은 옛 결과가 나중에 와도 다시 반영하지 않음
  arm_cancel_step_.reset();
  late_arm_ok_.reset();
  cargo_changed();
  msg = "재고 정합 " + std::string(doubt_fix_name(fix)) + ": " + before;
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
    // LOOK_RACK 으로 잰 칸 바닥 중심 + yaw. 없으면 빈 프레임 (= 녹화 경로 그대로)
    target.header.frame_id = "";
    const auto it = rack_meas_.find(s.slot);
    if (it == rack_meas_.end() || now - it->second.stamp > rack_fresh_s_) {return true;}
    geometry_msgs::msg::PoseStamped base;
    if (it->second.pose.header.frame_id == base_frame_) {
      base = it->second.pose;
    } else if (!to_base(it->second.pose, true, base)) {
      RCLCPP_WARN(logger(), "%s: 칸 측정을 %s 로 못 바꿈 → 측정 없이", s.name.c_str(), base_frame_.c_str());
      return true;
    }
    target = base;
    const auto & q = target.pose.orientation;
    RCLCPP_INFO(logger(), "%s: 칸 측정 (%.4f, %.4f, %.4f) yaw %.1f° → target", s.name.c_str(), target.pose.position.x,
      target.pose.position.y, target.pose.position.z, yaw_from_quaternion(q.x, q.y, q.z, q.w) * 180.0 / M_PI);
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
  nav_.sent_s = now_s();
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

// 취소: 진행 중이던 목표는 최종 결과(또는 거절)를 받을 때까지 *_cancel_pending_ 으로 추적 (resume 조건)
void Executor::cancel_nav()
{
  if (nav_.active) {
    nav_cancel_gen_ = nav_.gen;
    nav_cancel_pending_ = true;
    if (nav_handle_) {nav_cur_client_->async_cancel_goal(nav_handle_);}   // 아직 응답 전이면 받아들여질 때 취소
  }
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
  arm_ = GoalTrack{};
  arm_.gen = ++gen_;
  arm_.active = true;
  arm_.sent_s = now_s();
  arm_phase_.clear();
  const uint64_t gen = arm_.gen;

  auto opts = rclcpp_action::Client<ArmTask>::SendGoalOptions();
  opts.goal_response_callback = [this, gen](ArmGoalHandle::SharedPtr gh) {
      if (arm_cancel_pending_ && gen == arm_cancel_gen_) {   // 취소한 목표가 늦게 받아들여짐 → 그것도 취소
        if (gh) {
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
      if (gen == arm_.gen) {arm_phase_ = fb->phase;}
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
      if (arm_.success) {arm_needs_home_ = false;}   // control 의 팔 작업은 모두 home 자세로 끝남
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
  arm_ = GoalTrack{};
  arm_.gen = ++gen_;
  arm_handle_.reset();
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

}  // namespace planning
