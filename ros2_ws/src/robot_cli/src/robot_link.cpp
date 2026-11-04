// robot_link.cpp — ROS 2 연결, 설명은 robot_link.hpp
//   락 규칙: mu_ 를 쥔 채로 rclcpp 를 부르지 않음 (콜백 스레드와 서로 기다리지 않게)

#include "robot_cli/robot_link.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <thread>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "interfaces/msg/cargo_state.hpp"
#include "interfaces/msg/detection_array.hpp"
#include "interfaces/msg/mission_status.hpp"
#include "interfaces/msg/motion_mode.hpp"
#include "interfaces/msg/task_text.hpp"
#include "interfaces/msg/traffic_light.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "rcl_interfaces/msg/log.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/string.hpp"
#include "urdf/model.hpp"

#include "robot_cli/status_view.hpp"

namespace robot_cli
{

namespace
{
using namespace std::chrono_literals;
using ListCtrl = controller_manager_msgs::srv::ListControllers;
using ListHw = controller_manager_msgs::srv::ListHardwareComponents;
using Trigger = std_srvs::srv::Trigger;
using SetBool = std_srvs::srv::SetBool;

double yaw_of(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

geometry_msgs::msg::Quaternion quat_of(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(yaw / 2.0);
  q.w = std::cos(yaw / 2.0);
  return q;
}

std::string short_id(const std::array<uint8_t, 16> & u)
{
  char b[9];
  std::snprintf(b, sizeof(b), "%02x%02x%02x%02x", u[0], u[1], u[2], u[3]);
  return b;
}

DetView det_of(const interfaces::msg::Detection & d)
{
  DetView v;
  v.kind = d.kind;
  v.label = d.label;
  v.shape = d.shape;
  v.id = d.id;
  v.conf = d.confidence;
  v.x = d.pose.position.x;
  v.y = d.pose.position.y;
  v.z = d.pose.position.z;
  v.qx = d.pose.orientation.x;
  v.qy = d.pose.orientation.y;
  v.qz = d.pose.orientation.z;
  v.qw = d.pose.orientation.w;
  v.yaw_known = d.yaw_known;
  for (int i = 0; i < 4; ++i) {v.bbox[i] = d.bbox[i];}
  return v;
}

SlotView slot_of(const interfaces::msg::CargoSlot & c)
{
  return SlotView{c.slot, c.color, c.puck_id, c.shape, c.occupied, c.verified};
}

std::vector<GoalView> goals_of(const action_msgs::msg::GoalStatusArray & m)
{
  auto list = m.status_list;
  std::sort(list.begin(), list.end(), [](const auto & a, const auto & b) {
    return rclcpp::Time(a.goal_info.stamp) < rclcpp::Time(b.goal_info.stamp);
  });
  std::vector<GoalView> out;
  for (const auto & s : list) {out.push_back({short_id(s.goal_info.goal_id.uuid), s.status});}
  return out;
}

std::string rclcpp_code(rclcpp_action::ResultCode c)
{
  switch (c) {
    case rclcpp_action::ResultCode::SUCCEEDED: return "SUCCEEDED";
    case rclcpp_action::ResultCode::ABORTED: return "ABORTED";
    case rclcpp_action::ResultCode::CANCELED: return "CANCELED";
    default: return "UNKNOWN";
  }
}

std::string arm_what(const Command & c)
{
  char b[160];
  if (c.task == RobotLink::ArmTask::Goal::PICK || c.task == RobotLink::ArmTask::Goal::PLACE) {
    std::snprintf(b, sizeof(b), "%s (%.3f, %.3f, %.3f) yaw %s", task_name(c.task), c.x, c.y, c.z,
      std::isfinite(c.yaw_deg) ? (std::to_string(static_cast<int>(std::lround(c.yaw_deg))) + "°").c_str() : "모름");
    return b;
  }
  return std::string(task_name(c.task)) + (c.slot.empty() ? "" : " " + c.slot);
}

// 시간 표 이름 (짧게): "STORE S5" · "PICK .235,.000"
std::string arm_short(const Command & c)
{
  if (c.task == RobotLink::ArmTask::Goal::PICK || c.task == RobotLink::ArmTask::Goal::PLACE) {
    char b[48];
    std::snprintf(b, sizeof(b), "%s %.3f,%.3f", task_name(c.task), c.x, c.y);
    return b;
  }
  return std::string(task_name(c.task)) + (c.slot.empty() ? "" : " " + c.slot);
}
}  // namespace

RobotLink::RobotLink(Config cfg)
: rclcpp::Node("robot_cli"), cfg_(std::move(cfg))
{
  const auto stream = rclcpp::QoS(10).best_effort();   // 신뢰 · 비신뢰 발행 모두와 맞음
  const auto latched = rclcpp::QoS(1).reliable().transient_local();
  auto keep = [this](rclcpp::SubscriptionBase::SharedPtr s) {subs_.push_back(s);};

  // 관절마다 받은 시각을 따로 (바퀴만 든 메시지가 팔 값을 새것처럼 만들지 않게) — joints_t 는 snapshot 에서 joint1~5 중 가장 늦은 것
  auto joints = [this](std::map<std::string, double> & dst, std::map<std::string, double> & seen, std::map<std::string, double> * cur) {
      return [this, &dst, &seen, cur](sensor_msgs::msg::JointState::ConstSharedPtr m) {
               std::lock_guard<std::mutex> lk(mu_);
               const double t = now_s();
               for (std::size_t i = 0; i < m->name.size() && i < m->position.size(); ++i) {
                 dst[m->name[i]] = m->position[i];
                 seen[m->name[i]] = t;
                 if (cur && i < m->effort.size()) {(*cur)[m->name[i]] = m->effort[i];}
               }
             };
    };
  keep(create_subscription<sensor_msgs::msg::JointState>("/joint_states", stream, joints(s_.joints, joint_seen_, &s_.current)));
  keep(create_subscription<sensor_msgs::msg::JointState>("/joint_states_corrected", stream,
    joints(s_.corrected, corr_seen_, nullptr)));
  keep(create_subscription<std_msgs::msg::String>("/robot_description", latched, [this](std_msgs::msg::String::ConstSharedPtr m) {
      urdf::Model model;
      if (!model.initString(m->data)) {return;}
      std::map<std::string, std::pair<double, double>> lim;
      for (const auto & kv : model.joints_) {
        const auto & j = kv.second;
        if (j && j->limits && (j->type == urdf::Joint::REVOLUTE || j->type == urdf::Joint::PRISMATIC)) {
          lim[kv.first] = {j->limits->lower, j->limits->upper};
        }
      }
      std::lock_guard<std::mutex> lk(mu_);
      s_.limits = std::move(lim);
    }));
  // 배터리: OpenCR 하드웨어가 /diagnostics 가 아니라 로그로만 냄 (10 s 마다 "… 배터리 12.3 V (최저 12.1)", 낮으면 경고)
  keep(create_subscription<rcl_interfaces::msg::Log>("/rosout", rclcpp::QoS(200).reliable().transient_local(),
    [this](rcl_interfaces::msg::Log::ConstSharedPtr m) {
      std::string board;
      double mean = 0, min = 0;
      if (!parse_battery_log(m->msg, board, mean, min)) {return;}
      // transient_local 이라 켤 때 10 s 안의 옛 줄도 옴 → 받은 시각이 아니라 로그 시각으로 나이를 잼
      const double age = std::max(0.0, now().seconds() - rclcpp::Time(m->stamp).seconds());
      std::lock_guard<std::mutex> lk(mu_);
      s_.battery[board] = BatteryView{mean, min, now_s() - age};
    }));
  auto stamp_of = [this](double & t) {
      return [this, &t](sensor_msgs::msg::LaserScan::ConstSharedPtr) {
               std::lock_guard<std::mutex> lk(mu_);
               t = now_s();
             };
    };
  keep(create_subscription<sensor_msgs::msg::LaserScan>("/lidar_front/scan", rclcpp::SensorDataQoS(), stamp_of(s_.scan_front_t)));
  keep(create_subscription<sensor_msgs::msg::LaserScan>("/lidar_rear/scan", rclcpp::SensorDataQoS(), stamp_of(s_.scan_rear_t)));
  keep(create_subscription<sensor_msgs::msg::Imu>("/imu_sensor_broadcaster/imu", rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::Imu::ConstSharedPtr) {
      std::lock_guard<std::mutex> lk(mu_);
      s_.imu_t = now_s();
    }));

  using MotionMode = interfaces::msg::MotionMode;
  keep(create_subscription<MotionMode>("/control/motion_mode", latched, [this](MotionMode::ConstSharedPtr m) {
      std::lock_guard<std::mutex> lk(mu_);
      s_.mode = m->mode;
      s_.request_id = m->request_id;
      s_.reason = m->reason;
      s_.ready = m->ready;
      s_.mode_t = now_s();
    }));
  keep(create_subscription<MotionMode>("/control/motion_mode_ack", stream, [this](MotionMode::ConstSharedPtr m) {
      std::lock_guard<std::mutex> lk(mu_);
      s_.ack_mode = m->mode;
      s_.ack_id = m->request_id;
      s_.ack_t = now_s();
    }));

  using GoalStatusArray = action_msgs::msg::GoalStatusArray;
  keep(create_subscription<GoalStatusArray>("/arm_task/_action/status", latched, [this](GoalStatusArray::ConstSharedPtr m) {
      auto g = goals_of(*m);
      std::lock_guard<std::mutex> lk(mu_);
      s_.arm_goals = std::move(g);
      // 작업 시간: 진행 중으로 처음 본 때 → 끝 (CLI 를 켜기 전에 끝난 goal 은 안 셈)
      const double t = now_s();
      for (const auto & st : m->status_list) {
        const std::string id = short_id(st.goal_info.goal_id.uuid);
        const bool running = st.status >= 1 && st.status <= 3;
        const bool known = std::any_of(hist_.begin(), hist_.end(), [&id](const TaskTiming & h) {return h.id == id;});
        if (!known && !running) {continue;}
        TaskTiming & h = timing_of(id);
        h.status = st.status;
        if (!running && h.t1 < 0.0) {h.t1 = t;}
      }
    }));
  keep(create_subscription<ArmTask::Impl::FeedbackMessage>("/arm_task/_action/feedback", rclcpp::QoS(10),
    [this](ArmTask::Impl::FeedbackMessage::ConstSharedPtr m) {
      std::lock_guard<std::mutex> lk(mu_);
      s_.arm_phase = m->feedback.phase;
      s_.arm_phase_t = now_s();
      TaskTiming & h = timing_of(short_id(m->goal_id.uuid));
      if (h.phases.empty() || h.phases.back().first != m->feedback.phase) {h.phases.emplace_back(m->feedback.phase, s_.arm_phase_t);}
    }));
  keep(create_subscription<GoalStatusArray>("/navigate_to_pose/_action/status", latched,
    [this](GoalStatusArray::ConstSharedPtr m) {
      auto g = goals_of(*m);
      std::lock_guard<std::mutex> lk(mu_);
      s_.nav_goals = std::move(g);
    }));
  keep(create_subscription<Nav::Impl::FeedbackMessage>("/navigate_to_pose/_action/feedback", rclcpp::QoS(10),
    [this](Nav::Impl::FeedbackMessage::ConstSharedPtr m) {
      const auto & f = m->feedback;
      std::lock_guard<std::mutex> lk(mu_);
      s_.dist_left = f.distance_remaining;
      s_.eta_s = rclcpp::Duration(f.estimated_time_remaining).seconds();
      s_.nav_time_s = rclcpp::Duration(f.navigation_time).seconds();
      s_.recoveries = f.number_of_recoveries;
      s_.nav_fb_t = now_s();
    }));

  keep(create_subscription<nav_msgs::msg::Odometry>("/odom", stream, [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
      const double off = now().seconds() - rclcpp::Time(m->header.stamp).seconds();   // 이 PC − 로봇 (+ 전송 지연)
      std::lock_guard<std::mutex> lk(mu_);
      s_.ox = m->pose.pose.position.x;
      s_.oy = m->pose.pose.position.y;
      s_.oyaw = yaw_of(m->pose.pose.orientation);
      s_.ovx = m->twist.twist.linear.x;
      s_.ovy = m->twist.twist.linear.y;
      s_.owz = m->twist.twist.angular.z;
      s_.odom_t = now_s();
      s_.clock_off = off;
    }));
  keep(create_subscription<diagnostic_msgs::msg::DiagnosticStatus>("/localization_health", stream,
    [this](diagnostic_msgs::msg::DiagnosticStatus::ConstSharedPtr m) {
      std::lock_guard<std::mutex> lk(mu_);
      s_.health_level = static_cast<int>(m->level);
      s_.health_msg = m->message;
      s_.health_t = now_s();
    }));
  // nav2_amcl 발행 QoS = reliable · transient_local · depth 1 (lyrical, ros2 topic info -v /amcl_pose 로 확인) → latched 와 맞음
  keep(create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/amcl_pose", latched,
    [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr m) {
      const auto & c = m->pose.covariance;
      std::lock_guard<std::mutex> lk(mu_);
      s_.ax = m->pose.pose.position.x;
      s_.ay = m->pose.pose.position.y;
      s_.ayaw = yaw_of(m->pose.pose.orientation);
      s_.amcl_std_xy = std::sqrt(std::max(0.0, (c[0] + c[7]) / 2.0));
      s_.amcl_std_yaw = std::sqrt(std::max(0.0, c[35]));
      s_.amcl_t = now_s();
    }));

  keep(create_subscription<interfaces::msg::TrafficLight>("/perception/traffic_light", stream,
    [this](interfaces::msg::TrafficLight::ConstSharedPtr m) {
      std::lock_guard<std::mutex> lk(mu_);
      s_.light = m->state;
      s_.light_visible = m->visible;
      s_.light_t = now_s();
    }));
  keep(create_subscription<interfaces::msg::DetectionArray>("/perception/wrist/detections", stream,
    [this](interfaces::msg::DetectionArray::ConstSharedPtr m) {
      std::string sum;
      std::vector<DetView> dets;
      for (const auto & d : m->detections) {
        sum += (sum.empty() ? "" : ", ") + d.label + (d.shape.empty() ? "" : " " + d.shape) + (d.id.empty() ? "" : " #" + d.id);
        dets.push_back(det_of(d));
      }
      const double lag = now().seconds() - rclcpp::Time(m->header.stamp).seconds();
      std::lock_guard<std::mutex> lk(mu_);
      s_.wrist_n = static_cast<int>(m->detections.size());
      s_.wrist_summary = sum.empty() ? "" : "[" + sum + "]";
      s_.wrist_t = now_s();
      s_.wrist_frame = m->header.frame_id;
      s_.wrist_lag = lag;
      s_.wrist_dets = std::move(dets);
    }));
  keep(create_subscription<interfaces::msg::DetectionArray>("/perception/wide/detections", stream,
    [this](interfaces::msg::DetectionArray::ConstSharedPtr m) {
      std::vector<DetView> dets;
      for (const auto & d : m->detections) {dets.push_back(det_of(d));}
      std::lock_guard<std::mutex> lk(mu_);
      s_.wide_n = static_cast<int>(m->detections.size());
      s_.wide_t = now_s();
      s_.wide_dets = std::move(dets);
    }));
  keep(create_subscription<interfaces::msg::DetectionArray>("/rack_tags/detections", stream,
    [this](interfaces::msg::DetectionArray::ConstSharedPtr m) {
      std::lock_guard<std::mutex> lk(mu_);
      const double t = now_s();
      s_.rack_n = static_cast<int>(m->detections.size());
      s_.rack_t = t;
      for (const auto & d : m->detections) {
        const int cell = d.label.size() > 1 && d.label[0] == 'S' ? std::atoi(d.label.c_str() + 1) : 0;
        if (cell <= 0 || cell > 99) {continue;}
        char key[16];
        std::snprintf(key, sizeof(key), "%c%02d", d.kind == "rack_puck" ? 'p' : 'c', cell);
        s_.rack_seen[key] = {det_of(d), t};
      }
    }));
  keep(create_subscription<interfaces::msg::TaskText>("/perception/task_text", stream,
    [this](interfaces::msg::TaskText::ConstSharedPtr m) {
      std::lock_guard<std::mutex> lk(mu_);
      s_.task_text = m->raw_text;
      s_.text_t = now_s();
    }));

  keep(create_subscription<interfaces::msg::MissionStatus>("/mission/status", stream,
    [this](interfaces::msg::MissionStatus::ConstSharedPtr m) {
      std::lock_guard<std::mutex> lk(mu_);
      s_.m_state = m->state;
      s_.m_phase = m->phase;
      s_.m_detail = m->detail;
      s_.m_step = m->step;
      s_.pucks_done = m->pucks_done;
      s_.pucks_total = m->pucks_total;
      s_.m_ready = m->ready;
      s_.m_fault = m->fault;
      s_.mission_t = now_s();
    }));
  keep(create_subscription<std_msgs::msg::Bool>("/mission/hold", stream, [this](std_msgs::msg::Bool::ConstSharedPtr m) {
      std::lock_guard<std::mutex> lk(mu_);
      s_.hold = m->data;
      s_.hold_t = now_s();
    }));
  keep(create_subscription<interfaces::msg::CargoState>("/mission/cargo", stream,
    [this](interfaces::msg::CargoState::ConstSharedPtr m) {
      std::string sum;
      for (const auto & c : m->slots) {
        if (c.occupied) {sum += c.slot + ":" + c.color + (c.puck_id.empty() ? "" : "#" + c.puck_id) + (c.verified ? "" : "?") + " ";}
      }
      if (sum.empty()) {sum = "적재함 비어 있음 ";}
      sum += m->gripper.occupied ? "· 그리퍼 " + m->gripper.color + "#" + m->gripper.puck_id : "· 그리퍼 비어 있음";
      std::vector<SlotView> slots;
      for (const auto & c : m->slots) {slots.push_back(slot_of(c));}
      std::lock_guard<std::mutex> lk(mu_);
      s_.cargo = sum;
      s_.cargo_t = now_s();
      s_.cargo_slots = std::move(slots);
      s_.cargo_gripper = slot_of(m->gripper);
    }));
  keep(create_subscription<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", stream,
    [this](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr m) {
      std::lock_guard<std::mutex> lk(mu_);
      for (const auto & d : m->status) {diag_by_name_[d.name] = {DiagView{d.name, d.message, static_cast<int>(d.level)}, now_s()};}
      s_.diag_t = now_s();
    }));
  keep(create_subscription<sensor_msgs::msg::LaserScan>("/scan", rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::LaserScan::ConstSharedPtr) {
      std::lock_guard<std::mutex> lk(mu_);
      s_.scan_t = now_s();
    }));

  arm_ = rclcpp_action::create_client<ArmTask>(this, "/arm_task");
  nav_ = rclcpp_action::create_client<Nav>(this, "/navigate_to_pose");   // 상태 · feedback 구독과 같은 절대 이름 (CLI 를 다른 namespace 로 켜도)
  pub_cmd_ = create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
  list_ctrl_.cli = create_client<ListCtrl>("/controller_manager/list_controllers");
  list_hw_.cli = create_client<ListHw>("/controller_manager/list_hardware_components");
  nav_active_.cli = create_client<Trigger>("/lifecycle_manager_navigation/is_active");
  loc_active_.cli = create_client<Trigger>("/lifecycle_manager_localization/is_active");

  tf_buf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listen_ = std::make_shared<tf2_ros::TransformListener>(*tf_buf_);

  drive_timer_ = create_wall_timer(50ms, [this]() {on_drive_tick();});
  poll_timer_ = create_wall_timer(1s, [this]() {on_poll();});
  tf_timer_ = create_wall_timer(200ms, [this]() {on_tf();});
  on_poll();
}

double RobotLink::now_s() const
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

Snapshot RobotLink::snapshot()
{
  std::lock_guard<std::mutex> lk(mu_);
  Snapshot s = s_;
  s.now = now_s();
  // 팔 관절 신선도 = joint1~5 중 가장 늦게 받은 것 (하나라도 없으면 없음)
  auto oldest = [](const std::map<std::string, double> & seen) {
      double t = 1e300;
      for (const auto & j : kArmJoint5) {
        const auto it = seen.find(j);
        if (it == seen.end()) {return -1.0;}
        t = std::min(t, it->second);
      }
      return t;
    };
  s.joints_t = oldest(joint_seen_);
  s.corrected_t = oldest(corr_seen_);
  s.diags.clear();
  for (const auto & kv : diag_by_name_) {
    if (s.now - kv.second.second < 10.0) {s.diags.push_back(kv.second.first);}
  }
  s.arm_hist.assign(hist_.begin(), hist_.end());
  s.estop = estop_;
  s.driving = driving_ || teleop_;
  s.teleop = teleop_;
  s.dvx = dvx_;
  s.dvy = dvy_;
  s.dwz = dwz_;
  return s;
}

TaskTiming & RobotLink::timing_of(const std::string & id)
{
  for (auto & h : hist_) {
    if (h.id == id) {return h;}
  }
  while (hist_.size() >= 8) {hist_.pop_front();}
  TaskTiming h;
  h.id = id;
  h.t0 = now_s();
  hist_.push_back(h);
  return hist_.back();
}

void RobotLink::note_goal(const std::string & id, const std::string & what)
{
  timing_of(id).what = what;
  s_.my_arm_id = id;
}

void RobotLink::log(const std::string & text)
{
  std::lock_guard<std::mutex> lk(mu_);
  log_.push_back(text);
  while (log_.size() > 200) {log_.pop_front();}
}

std::vector<std::string> RobotLink::take_log()
{
  std::lock_guard<std::mutex> lk(mu_);
  std::vector<std::string> out(log_.begin(), log_.end());
  log_.clear();
  return out;
}

// ======================================================================== 팔 · 주행 작업
void RobotLink::fail(const Done & done, int code, const std::string & text)
{
  log(text);
  done(code, text);
}

void RobotLink::send_arm(const Command & c, Done done, std::function<void()> accepted)
{
  bool estop = false, closing = false;
  {
    std::lock_guard<std::mutex> lk(mu_);
    estop = estop_;
    closing = closing_;
  }
  if (closing) {fail(done, 2, "팔 거절: 끝내는 중"); return;}
  if (estop) {fail(done, 2, "팔 거절: E-STOP 중 — estop off 뒤에"); return;}
  const std::string what = arm_what(c), label = arm_short(c);
  // 서버 찾기 — Ctrl-C 면 보내지 않음
  const auto until = std::chrono::steady_clock::now() + std::chrono::duration<double>(discovery_wait_s_);
  while (!arm_->action_server_is_ready()) {
    if (interrupted()) {fail(done, 1, "팔: 중단 (Ctrl-C) — 안 보냄"); return;}
    if (std::chrono::steady_clock::now() >= until) {fail(done, 1, "팔: /arm_task 서버 없음 (control_master 꺼짐?)"); return;}
    arm_->wait_for_action_server(50ms);
  }
  ArmTask::Goal g;
  g.task = c.task;
  g.slot = c.slot;
  if (c.task == ArmTask::Goal::PICK || c.task == ArmTask::Goal::PLACE) {
    g.target.header.frame_id = "base_link";
    g.target.header.stamp = now();
    g.target.pose.position.x = c.x;
    g.target.pose.position.y = c.y;
    g.target.pose.position.z = c.z;
    if (std::isfinite(c.yaw_deg)) {
      g.target.pose.orientation = quat_of(c.yaw_deg * M_PI / 180.0);
    } else {
      g.target.pose.orientation.w = 0.0;   // 방향 모름 = 0,0,0,0 (Detection.msg 약속, control_master 가 기본 방향)
    }
  }
  auto last_phase = std::make_shared<std::string>();
  auto opt = rclcpp_action::Client<ArmTask>::SendGoalOptions();
  const uint64_t seq = [this] {
      std::lock_guard<std::mutex> lk(mu_);
      return ++arm_seq_;
    }();
  opt.goal_response_callback = [this, what, label, done, accepted, seq](const rclcpp_action::ClientGoalHandle<ArmTask>::SharedPtr & gh) {
      const std::string text = what + (gh ? " → 받음, 진행 중" : " → 거절 (바쁨 · ready 아님 · 시작 자세 · 닿지 않음 — 이유는 control_master 로그)");
      bool cancel_now = false;
      {
        std::lock_guard<std::mutex> lk(mu_);
        s_.my_arm = text;
        if (gh) {
          arm_gh_ = gh;
          note_goal(short_id(gh->get_goal_id()), label);
          cancel_now = closing_ || seq <= arm_cancel_upto_;
        }
        if (!cancel_now) {--arm_pending_;}   // 바로 취소할 goal 은 취소를 보낸 뒤에 줄임 (finish 가 그때까지 기다리게)
      }
      log("팔 " + text);
      if (cancel_now) {   // finish() · cancel · estop 뒤에 늦게 받아짐 → 아무도 안 보는 채 움직이지 않게 바로 취소
        try {arm_->async_cancel_goal(gh); log("취소 뒤 늦게 받아진 팔 goal → 바로 취소");} catch (const std::exception &) {}
        std::lock_guard<std::mutex> lk(mu_);
        --arm_pending_;
      }
      if (!gh) {done(2, text);} else if (accepted) {accepted();}
    };
  opt.feedback_callback = [this, what, last_phase](rclcpp_action::ClientGoalHandle<ArmTask>::SharedPtr,
    const std::shared_ptr<const ArmTask::Feedback> f) {
      if (f->phase == *last_phase || quiet_) {return;}
      *last_phase = f->phase;
      log("팔 " + what + " · phase " + f->phase);
    };
  opt.result_callback = [this, what, done](const rclcpp_action::ClientGoalHandle<ArmTask>::WrappedResult & r) {
      const bool ok = r.code == rclcpp_action::ResultCode::SUCCEEDED && r.result && r.result->success;
      const std::string text = what + " → " + (ok ? "성공" : "실패") + " [" + rclcpp_code(r.code) + "] " +
        (r.result ? r.result->message : "");
      {
        std::lock_guard<std::mutex> lk(mu_);
        s_.my_arm = text;
      }
      log("팔 " + text);
      done(ok ? 0 : 2, text);
    };
  if (interrupted()) {fail(done, 1, "팔: 중단 (Ctrl-C) — 안 보냄"); return;}
  if (!quiet_) {log("팔 " + what + " 보냄");}
  {
    std::lock_guard<std::mutex> lk(mu_);
    ++arm_pending_;
  }
  arm_->async_send_goal(g, opt);
}

void RobotLink::send_nav(const Command & c, Done done, std::function<void()> accepted)
{
  bool hold = false, estop = false, closing = false;
  {
    std::lock_guard<std::mutex> lk(mu_);
    estop = estop_;
    closing = closing_;
    hold = s_.hold_t >= 0.0 && s_.hold;
  }
  if (closing) {fail(done, 2, "주행 거절: 끝내는 중"); return;}
  if (estop) {fail(done, 2, "주행 거절: E-STOP 중 — estop off 뒤에"); return;}
  Pose2D p{c.x, c.y, c.yaw_nav_deg * M_PI / 180.0};
  std::string what;
  if (!c.stop.empty()) {
    const NamedStop * s = find_stop(cfg_.stops, c.stop);
    if (!s) {fail(done, 2, "주행: 정차 이름 " + c.stop + " 없음 (dice_map)"); return;}
    p = s->pose;
    what = c.stop + " ";
  }
  char b[96];
  std::snprintf(b, sizeof(b), "(%.3f, %.3f, %.0f°)", p.x, p.y, p.yaw * 180.0 / M_PI);
  what += b;
  const auto until = std::chrono::steady_clock::now() + std::chrono::duration<double>(discovery_wait_s_);
  while (!nav_->action_server_is_ready()) {
    if (interrupted()) {fail(done, 1, "주행: 중단 (Ctrl-C) — 안 보냄"); return;}
    if (std::chrono::steady_clock::now() >= until) {fail(done, 1, "주행: navigate_to_pose 서버 없음 (nav2.launch.py 꺼짐?)"); return;}
    nav_->wait_for_action_server(50ms);
  }
  if (hold) {log("주의: /mission/hold = true → cmd_vel_hold 가 Nav2 속도를 막음 (미션 start 또는 mission_hold:=false)");}
  Nav::Goal g;
  g.pose.header.frame_id = "map";
  g.pose.pose.position.x = p.x;
  g.pose.pose.position.y = p.y;
  g.pose.pose.orientation = quat_of(p.yaw);
  auto opt = rclcpp_action::Client<Nav>::SendGoalOptions();
  const uint64_t seq = [this] {
      std::lock_guard<std::mutex> lk(mu_);
      return ++nav_seq_;
    }();
  opt.goal_response_callback = [this, what, done, accepted, seq](const rclcpp_action::ClientGoalHandle<Nav>::SharedPtr & gh) {
      const std::string text = what + (gh ? " → 받음, 주행 중" : " → 거절");
      bool cancel_now = false;
      {
        std::lock_guard<std::mutex> lk(mu_);
        s_.my_nav = text;
        if (gh) {
          nav_gh_ = gh;
          cancel_now = closing_ || seq <= nav_cancel_upto_;
        }
        if (!cancel_now) {--nav_pending_;}
      }
      log("주행 " + text);
      if (cancel_now) {
        try {nav_->async_cancel_goal(gh); log("취소 뒤 늦게 받아진 주행 goal → 바로 취소");} catch (const std::exception &) {}
        std::lock_guard<std::mutex> lk(mu_);
        --nav_pending_;
      }
      if (!gh) {done(2, text);} else if (accepted) {accepted();}
    };
  opt.result_callback = [this, what, done](const rclcpp_action::ClientGoalHandle<Nav>::WrappedResult & r) {
      const bool ok = r.code == rclcpp_action::ResultCode::SUCCEEDED;
      std::string text = what + " → " + (ok ? "도착" : "실패") + " [" + rclcpp_code(r.code) + "]";
      if (r.result && (r.result->error_code != 0 || !r.result->error_msg.empty())) {
        text += " " + std::to_string(r.result->error_code) + " " + r.result->error_msg;
      }
      {
        std::lock_guard<std::mutex> lk(mu_);
        s_.my_nav = text;
      }
      log("주행 " + text);
      done(ok ? 0 : 2, text);
    };
  if (interrupted()) {fail(done, 1, "주행: 중단 (Ctrl-C) — 안 보냄"); return;}
  if (!quiet_) {log("주행 " + what + " 보냄 (map)");}
  {
    std::lock_guard<std::mutex> lk(mu_);
    ++nav_pending_;
  }
  nav_->async_send_goal(g, opt);
}

void RobotLink::cancel_all(bool arm, bool nav)
{
  {   // 지금까지 보낸 이 CLI 의 goal 중 아직 받음 답이 없는 것도 받아지는 대로 취소
    std::lock_guard<std::mutex> lk(mu_);
    if (arm) {arm_cancel_upto_ = arm_seq_;}
    if (nav) {nav_cancel_upto_ = nav_seq_;}
  }
  if (arm && arm_->action_server_is_ready()) {
    arm_->async_cancel_all_goals([this](rclcpp_action::Client<ArmTask>::CancelResponse::SharedPtr r) {
        log("취소 /arm_task: " + std::to_string(r ? r->goals_canceling.size() : 0) + " 개 취소 중");
      });
  }
  if (nav && nav_->action_server_is_ready()) {
    nav_->async_cancel_all_goals([this](rclcpp_action::Client<Nav>::CancelResponse::SharedPtr r) {
        log("취소 navigate_to_pose: " + std::to_string(r ? r->goals_canceling.size() : 0) + " 개 취소 중");
      });
  }
}

int RobotLink::cancel_mine()
{
  rclcpp_action::ClientGoalHandle<ArmTask>::SharedPtr a;
  rclcpp_action::ClientGoalHandle<Nav>::SharedPtr n;
  {
    std::lock_guard<std::mutex> lk(mu_);
    a = arm_gh_;
    n = nav_gh_;
  }
  auto running = [](int8_t st) {
      return st == action_msgs::msg::GoalStatus::STATUS_ACCEPTED || st == action_msgs::msg::GoalStatus::STATUS_EXECUTING;
    };
  int sent = 0;
  // 서버마다 따로: 팔 goal 이 그 사이 끝나 예외가 나도 주행 goal 취소는 보냄
  try {
    if (a && running(a->get_status())) {arm_->async_cancel_goal(a); ++sent; log("이 CLI 의 팔 goal 취소");}
  } catch (const rclcpp_action::exceptions::UnknownGoalHandleError &) {}   // 그 사이 끝난 goal
  try {
    if (n && running(n->get_status())) {nav_->async_cancel_goal(n); ++sent; log("이 CLI 의 주행 goal 취소");}
  } catch (const rclcpp_action::exceptions::UnknownGoalHandleError &) {}
  return sent;
}

std::string RobotLink::finish(double max_s)
{
  {
    std::lock_guard<std::mutex> lk(mu_);
    closing_ = true;   // 이제부터 받아지는 goal 은 응답 콜백이 바로 취소
  }
  std::string out;
  const int n = cancel_mine();
  if (n > 0) {out = "진행 중이던 goal " + std::to_string(n) + " 개 취소";}
  int ap = 0, np = 0;
  const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(max_s);
  for (;;) {
    {
      std::lock_guard<std::mutex> lk(mu_);
      ap = arm_pending_;
      np = nav_pending_;
    }
    if ((ap == 0 && np == 0) || std::chrono::steady_clock::now() >= end) {break;}
    std::this_thread::sleep_for(20ms);
  }
  if (ap > 0 || np > 0) {   // 서버가 받았는지 모름 → 그 서버의 모든 goal 취소 (다른 쪽 goal 도 멈춤)
    const std::string sent = cancel(ap > 0 && np > 0 ? "all" : ap > 0 ? "arm" : "nav");
    char b[64];
    std::snprintf(b, sizeof(b), "받음 답이 %.1f s 안에 안 옴 → ", max_s);
    out += std::string(out.empty() ? "" : " · ") + b +
      (sent.empty() ? std::string("서버가 안 보여 취소 못 보냄 (서버 쪽을 확인)") : sent + "의 모든 goal 취소");
  }
  return out;
}

bool RobotLink::missing_data()
{
  const Snapshot s = snapshot();
  auto miss = [this](const char * topic, double t) {return t < 0.0 && count_publishers(topic) > 0;};
  const bool cm = s.nodes.count("/controller_manager") > 0;
  return miss("/joint_states", s.joints_t) || miss("/joint_states_corrected", s.corrected_t) ||
         miss("/control/motion_mode", s.mode_t) || miss("/control/motion_mode_ack", s.ack_t) ||
         miss("/odom", s.odom_t) || miss("/localization_health", s.health_t) || miss("/mission/status", s.mission_t) ||
         miss("/perception/traffic_light", s.light_t) || miss("/diagnostics", s.diag_t) ||
         (cm && (s.ctrl_t < 0.0 || s.hw_t < 0.0));
}

bool RobotLink::servers_ready(bool arm, bool nav)
{
  return (!arm || arm_->action_server_is_ready()) && (!nav || nav_->action_server_is_ready());
}

std::string RobotLink::cancel(const std::string & which)
{
  const bool arm = which != "nav", nav = which != "arm";
  std::string sent;
  if (arm && arm_->action_server_is_ready()) {sent += "/arm_task ";}
  if (nav && nav_->action_server_is_ready()) {sent += "navigate_to_pose ";}
  cancel_all(arm, nav);
  return sent;
}

void RobotLink::set_estop(bool on)
{
  {
    // 상태 바꾸기 + 0 보내기를 drive 틱과 같은 락 안에서 → 틱이 미리 고른 옛 속도가 이 0 뒤에 나가지 않음
    std::lock_guard<std::mutex> pk(pub_mu_);
    {
      std::lock_guard<std::mutex> lk(mu_);
      estop_ = on;
      driving_ = teleop_ = false;
      dvx_ = dvy_ = dwz_ = 0.0;
      zero_left_ = on ? 0 : 5;
      last_recancel_ = now_s();
    }
    if (on) {publish_twist(0.0, 0.0, 0.0);}
  }
  if (on) {
    const std::string sent = cancel("all");   // 지금 발견된 서버만 (기다리지 않음) + 늦게 받아지는 이 CLI 의 goal
    log("E-STOP: " + (sent.empty() ? std::string("취소할 서버가 아직 안 보임 (발견되면 진행 중 goal 다시 취소)") : sent + "의 모든 goal 취소") +
      " + /cmd_vel 0 (소프트웨어 정지 — 모터 전원 차단 아님)");
  } else {
    log("E-STOP 풀림 (control_master 는 취소 뒤 STOP 잠금 — 주행하려면 home)");
  }
}

// ======================================================================== 수동 주행
std::string RobotLink::drive_guard(bool force)
{
  std::lock_guard<std::mutex> lk(mu_);
  return drive_block_reason(facts_locked(now_s()), force);   // 규칙은 status_view (화면 3 쪽에도 같은 글)
}

DriveFacts RobotLink::facts_locked(double t) const
{
  DriveFacts f;
  f.now = t;
  f.estop = estop_;
  f.odom_t = s_.odom_t;
  f.clock_off = s_.clock_off;
  f.mode_t = s_.mode_t;
  f.mode = s_.mode;
  f.nav2_up = s_.nodes.count("/bt_navigator") > 0;
  f.nodes_known = s_.nodes.count("/controller_manager") > 0;
  return f;
}

void RobotLink::clamp_speed(double & vx, double & vy, double & wz) const
{
  const double lin = std::hypot(vx, vy);
  if (lin > cfg_.max_linear && lin > 0.0) {
    vx *= cfg_.max_linear / lin;
    vy *= cfg_.max_linear / lin;
  }
  wz = std::clamp(wz, -cfg_.max_angular, cfg_.max_angular);
}

std::string RobotLink::take_drive_lost()
{
  std::lock_guard<std::mutex> lk(mu_);
  std::string out;
  out.swap(drive_lost_);
  return out;
}

std::string RobotLink::drive(double vx, double vy, double wz, double secs, bool force)
{
  const std::string why = drive_guard(force);
  if (!why.empty()) {return why;}
  clamp_speed(vx, vy, wz);   // 어느 경로로 와도 설정 한계 안
  std::lock_guard<std::mutex> lk(mu_);
  drive_lost_.clear();
  teleop_ = false;
  driving_ = true;
  drive_force_ = force;
  dvx_ = vx;
  dvy_ = vy;
  dwz_ = wz;
  drive_until_ = now_s() + secs;
  return "";
}

std::string RobotLink::set_teleop(bool on, bool force)
{
  if (on) {
    const std::string why = drive_guard(force);
    if (!why.empty()) {return why;}
  }
  std::lock_guard<std::mutex> lk(mu_);
  if (on) {drive_lost_.clear();}
  teleop_ = on;
  drive_force_ = force;
  driving_ = false;
  dvx_ = dvy_ = dwz_ = 0.0;
  drive_until_ = 0.0;
  if (!on) {zero_left_ = 5;}
  return "";
}

void RobotLink::teleop_cmd(double vx, double vy, double wz)
{
  clamp_speed(vx, vy, wz);
  std::lock_guard<std::mutex> lk(mu_);
  if (!teleop_ || estop_) {return;}
  dvx_ = vx;
  dvy_ = vy;
  dwz_ = wz;
  drive_until_ = now_s() + 0.6;
}

void RobotLink::stop_drive()
{
  std::lock_guard<std::mutex> lk(mu_);
  driving_ = false;
  dvx_ = dvy_ = dwz_ = 0.0;
  drive_until_ = 0.0;
  zero_left_ = 5;
}

bool RobotLink::drive_busy()
{
  std::lock_guard<std::mutex> lk(mu_);
  return driving_ || zero_left_ > 0;
}

void RobotLink::publish_twist(double vx, double vy, double wz)
{
  geometry_msgs::msg::TwistStamped m;
  m.header.stamp = now();
  m.header.frame_id = "base_footprint";
  m.twist.linear.x = vx;
  m.twist.linear.y = vy;
  m.twist.angular.z = wz;
  pub_cmd_->publish(m);
}

// 20 Hz: estop 이면 0 (10 Hz) + 새 goal 다시 취소, drive · teleop 이면 그 속도, 끝나면 0 몇 번
//   drive · teleop 중에는 허용 조건을 틱마다 다시 봄 (차례 · Nav2 · 시계) → 잃으면 멈추고 0 (force 는 시작 때 것 그대로)
void RobotLink::on_drive_tick()
{
  std::lock_guard<std::mutex> pk(pub_mu_);   // 고르기 + 보내기 (set_estop 과 한 줄로)
  ++tick_;
  double vx = 0, vy = 0, wz = 0;
  bool send = false, recancel = false;
  std::string lost;
  {
    std::lock_guard<std::mutex> lk(mu_);
    const double t = now_s();
    if (!estop_ && (driving_ || teleop_)) {
      lost = drive_block_reason(facts_locked(t), drive_force_);
      if (!lost.empty()) {
        drive_lost_ = lost;
        driving_ = teleop_ = false;
        dvx_ = dvy_ = dwz_ = 0.0;
        drive_until_ = 0.0;
        moved_ = true;   // 아래에서 0 을 몇 번 보냄
      }
    }
    if (estop_) {
      send = tick_ % 2 == 0;
      auto busy = [](const std::vector<GoalView> & g) {
          return std::any_of(g.begin(), g.end(), [](const GoalView & v) {return v.status == 1 || v.status == 2;});
        };
      if ((busy(s_.arm_goals) || busy(s_.nav_goals)) && t - last_recancel_ > 1.0) {
        recancel = true;
        last_recancel_ = t;
      }
    } else if ((driving_ || teleop_) && t < drive_until_) {
      vx = dvx_;
      vy = dvy_;
      wz = dwz_;
      send = true;
      moved_ = moved_ || vx != 0.0 || vy != 0.0 || wz != 0.0;
    } else {
      if (moved_) {zero_left_ = 5;}
      moved_ = false;
      driving_ = false;
      if (teleop_) {dvx_ = dvy_ = dwz_ = 0.0;}
      if (zero_left_ > 0) {
        --zero_left_;
        send = true;
      }
    }
  }
  if (send) {publish_twist(vx, vy, wz);}
  if (!lost.empty()) {log("수동 주행 멈춤 (바퀴 0): " + lost);}
  if (recancel) {
    log("E-STOP 중 새 goal → 다시 취소");
    cancel_all(true, true);
  }
}

// ======================================================================== 1 Hz 묻기
// 서비스 하나 묻기 (기다리는 요청 하나만, 1.5 s 넘게 답 없으면 버리고 다시). 서비스가 떠 있으면 true
//   Poll 의 busy 는 이 타이머 · 응답 콜백이 같은 실행기 스레드라 락 없이 씀
template<typename Srv, typename F>
bool RobotLink::ask(Poll & p, double t, F on_answer)
{
  auto cli = std::static_pointer_cast<rclcpp::Client<Srv>>(p.cli);
  if (p.busy && t - p.sent > 1.5) {   // 서버 응답 쪽 발견 전에 보낸 요청은 답이 안 옴 → 버리고 다시
    cli->prune_pending_requests();
    p.busy = false;
  }
  if (!cli->service_is_ready()) {return false;}
  if (p.busy) {return true;}
  p.busy = true;
  p.sent = t;
  cli->async_send_request(std::make_shared<typename Srv::Request>(),
    [&p, on_answer](typename rclcpp::Client<Srv>::SharedFuture f) {
      p.busy = false;
      on_answer(*f.get());
    });
  return true;
}

void RobotLink::on_poll()
{
  const double t = now_s();
  std::set<std::string> nodes;
  for (const auto & n : get_node_names()) {nodes.insert(n);}

  // 위치 추정 종류: slam_toolbox 파라미터 mode (mapping | localization), 아니면 AMCL
  std::string loc;
  if (nodes.count("/slam_toolbox")) {
    if (!slam_param_) {slam_param_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "/slam_toolbox");}
    // 답이 없거나 slam_toolbox 를 다른 모드로 다시 켰을 수 있으니 10 s 마다 다시 물음
    if (t - slam_asked_t_ > 10.0 && slam_param_->service_is_ready()) {
      slam_asked_t_ = t;
      slam_param_->get_parameters({"mode"}, [this](std::shared_future<std::vector<rclcpp::Parameter>> f) {
          std::string m = "?";
          try {
            const auto v = f.get();
            if (!v.empty() && v[0].get_type() == rclcpp::ParameterType::PARAMETER_STRING) {m = v[0].as_string();}
          } catch (const std::exception &) {}
          std::lock_guard<std::mutex> lk(mu_);
          s_.loc_mode = "SLAM " + m;
        });
    }
  } else {
    slam_param_.reset();
    slam_asked_t_ = -1e9;
    loc = nodes.count("/amcl") ? "AMCL" : "";
  }

  ask<ListCtrl>(list_ctrl_, t, [this](const ListCtrl::Response & r) {
      std::vector<std::pair<std::string, std::string>> v;
      for (const auto & c : r.controller) {v.emplace_back(c.name, c.state);}
      std::lock_guard<std::mutex> lk(mu_);
      s_.controllers = v;
      s_.ctrl_t = now_s();
    });
  ask<ListHw>(list_hw_, t, [this](const ListHw::Response & r) {
      std::vector<std::pair<std::string, std::string>> v;
      for (const auto & c : r.component) {v.emplace_back(c.name, c.state.label);}
      std::lock_guard<std::mutex> lk(mu_);
      s_.hardware = v;
      s_.hw_t = now_s();
    });
  const bool nav_up = ask<Trigger>(nav_active_, t, [this](const Trigger::Response & r) {
        std::lock_guard<std::mutex> lk(mu_);
        s_.nav_active = r.success ? 1 : 0;
      });
  const bool loc_up = ask<Trigger>(loc_active_, t, [this](const Trigger::Response & r) {
        std::lock_guard<std::mutex> lk(mu_);
        s_.loc_active = r.success ? 1 : 0;
      });

  std::lock_guard<std::mutex> lk(mu_);
  s_.nodes = std::move(nodes);
  if (!s_.nodes.count("/slam_toolbox")) {
    s_.loc_mode = loc;
  } else if (s_.loc_mode.empty()) {
    s_.loc_mode = "SLAM (mode 묻는 중)";
  }
  if (!nav_up) {s_.nav_active = -1;}
  if (!loc_up) {s_.loc_active = -1;}
}

void RobotLink::on_tf()
{
  bool ok = false;
  double x = 0, y = 0, yaw = 0, stamp = 0;
  try {
    const auto tr = tf_buf_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
    x = tr.transform.translation.x;
    y = tr.transform.translation.y;
    yaw = yaw_of(tr.transform.rotation);
    stamp = rclcpp::Time(tr.header.stamp).seconds();
    ok = true;
  } catch (const tf2::TransformException &) {}
  const double ros_now = now().seconds();
  std::lock_guard<std::mutex> lk(mu_);
  s_.have_map_pose = ok;
  if (ok) {
    s_.mx = x;
    s_.my = y;
    s_.myaw = yaw;
    // 변환 시각의 나이 (로봇 시계 기준: /odom 으로 잰 시계 차이를 뺌). TF 가 끊기면 마지막 위치가 남으니 나이로 오래됨을 봄
    const double off = s_.odom_t >= 0.0 ? s_.clock_off : 0.0;
    s_.map_age = ros_now - off - stamp;
  }
}

// ======================================================================== 서비스
// 서비스 기다리기 (최대 max(discovery_wait, 0.5) s) — Ctrl-C 면 바로 false (보내지 않음)
bool RobotLink::wait_service(rclcpp::ClientBase & cli)
{
  const auto until = std::chrono::steady_clock::now() + std::chrono::duration<double>(std::max(discovery_wait_s_, 0.5));
  while (!interrupted()) {
    if (cli.service_is_ready()) {return !interrupted();}
    if (std::chrono::steady_clock::now() >= until) {return false;}
    cli.wait_for_service(50ms);
  }
  return false;
}

void RobotLink::call_service(const std::string & name, int setbool, Done done)
{
  std::string blocked;   // E-STOP 중 · 끝내는 중에는 서비스도 안 부름 (mission start · resume 이 새 동작을 시작할 수 있어서)
  {
    std::lock_guard<std::mutex> lk(mu_);
    if (estop_) {blocked = "E-STOP 중 — estop off 뒤에";} else if (closing_) {blocked = "끝내는 중";}
  }
  if (!blocked.empty()) {fail(done, 2, name + " 거절: " + blocked); return;}
  std::string type;
  for (const auto & kv : get_service_names_and_types()) {
    if (kv.first == name && !kv.second.empty()) {type = kv.second[0];}
  }
  if (type.empty()) {fail(done, 1, name + " 서비스 없음"); return;}
  if (type == "std_srvs/srv/Trigger") {
    if (setbool >= 0) {fail(done, 2, name + " 는 Trigger — on|off 빼고"); return;}
    auto cli = std::dynamic_pointer_cast<rclcpp::Client<Trigger>>(call_clients_[name + "#t"]);
    if (!cli) {cli = create_client<Trigger>(name); call_clients_[name + "#t"] = cli;}   // 기다리는 요청이 있어도 그대로 둠
    if (!wait_service(*cli)) {fail(done, 1, name + (interrupted() ? " 중단 (Ctrl-C) — 안 보냄" : " 응답 없음")); return;}
    cli->async_send_request(std::make_shared<Trigger::Request>(), [this, name, done](rclcpp::Client<Trigger>::SharedFuture f) {
        const auto r = f.get();
        const std::string text = name + " → " + (r->success ? "성공 " : "실패 ") + r->message;
        log(text);
        done(r->success ? 0 : 2, text);
      });
    return;
  }
  if (type == "std_srvs/srv/SetBool") {
    if (setbool < 0) {fail(done, 2, name + " 는 SetBool — on|off 필요"); return;}
    auto cli = std::dynamic_pointer_cast<rclcpp::Client<SetBool>>(call_clients_[name + "#b"]);
    if (!cli) {cli = create_client<SetBool>(name); call_clients_[name + "#b"] = cli;}
    if (!wait_service(*cli)) {fail(done, 1, name + (interrupted() ? " 중단 (Ctrl-C) — 안 보냄" : " 응답 없음")); return;}
    auto req = std::make_shared<SetBool::Request>();
    req->data = setbool == 1;
    cli->async_send_request(req, [this, name, done](rclcpp::Client<SetBool>::SharedFuture f) {
        const auto r = f.get();
        const std::string text = name + " → " + (r->success ? "성공 " : "실패 ") + r->message;
        log(text);
        done(r->success ? 0 : 2, text);
      });
    return;
  }
  fail(done, 2, name + " 타입 " + type + " — Trigger · SetBool 만 됨");
}

}  // namespace robot_cli
