// control_master.cpp — 제어 마스터: 팔 동작 + "한 번에 한 무리" (차례)
// =====================================================================================
// 모터와 직접 말하지 않는다 (그건 ros2_control 이 함). 바퀴 계산도 하지 않는다 (swerve_controller 가 함).
//   왜 바퀴를 여기서 안 하나: 이 노드가 바퀴 명령을 토픽으로 내보내는 구조면, 노드가 멈췄을 때 받는 쪽이
//   마지막 속도를 붙잡고 계속 돌 수 있음. 바퀴는 ros2_control 의 100 Hz 루프 안에서 스스로 끊김을 봐야 안전 (D8)
//
//   planning_master ─/arm_task────▶ ┌─────────────────────┐ ──arm_controller/follow_joint_trajectory──▶ ros2_control
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

#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "controller_manager_msgs/srv/list_hardware_components.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

#include "manipulation/arm.hpp"
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
    if (make_arm_plan(goal.task, gripper_open_, gripper_close_).empty()) {
      RCLCPP_WARN(get_logger(), "모르는 팔 작업 번호 %u → 거절", static_cast<unsigned>(goal.task));
      return rclcpp_action::GoalResponse::REJECT;
    }
    // 준비 조건 (방송의 ready 와 같은 함수 — 방송과 실제 받기 판단이 어긋나지 않게)
    std::string why;
    if (!task_ready(now().seconds(), why)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "%s → 팔 작업 거절", why.c_str());
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
    why.clear();
    return true;
  }

  void on_arm_accepted(const std::shared_ptr<ArmGoalHandle> gh)
  {
    arm_goal_ = gh;
    const auto plan = make_arm_plan(gh->get_goal()->task, gripper_open_, gripper_close_);
    runner_.start(plan);
    arbiter_.request_arm();   // 바퀴부터 세우기 → 잠금 답 + 몸통이 멈추면 ARM
    publish_turn();
    RCLCPP_INFO(get_logger(), "팔 작업 %u 받음 (동작 %zu개) → 바퀴 세우는 중",
      static_cast<unsigned>(gh->get_goal()->task), plan.size());
  }

  // ======================================================================== 50 Hz 본체
  void step()
  {
    const rclcpp::Time t = now();
    // ⓪ 하드웨어 이상 → STOP + 진행 중 팔 작업 실패 (HOME 확인 무효). 정상으로 돌아와도 여기서 DRIVE 로 안 감
    const bool hw_ok = hw_->healthy(t.seconds());
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
  // 판단은 ArmRunner(arm.cpp, test_arm.cpp)가 하고, 여기서는 그 명령을 ROS 로 옮기기만 함
  void run_arm(const rclcpp::Time & t)
  {
    if (!arm_goal_) {return;}
    if (arm_goal_->is_canceling()) {runner_.request_cancel();}

    const double now_s = t.seconds();
    for (const ArmCommand & c : runner_.tick(now_s, arbiter_.arm_may_move(), fold_->confirmed(now_s))) {
      switch (c.kind) {
        case ArmCommand::Kind::SEND: {
          auto fb = std::make_shared<ArmTask::Feedback>();
          fb->phase = c.action.phase;
          arm_goal_->publish_feedback(fb);
          if (c.action.is_gripper) {
            send_trajectory(grip_client_, {gripper_joint_}, {c.action.gripper}, c.gen);
          } else {
            send_trajectory(arm_client_, arm_joints_, poses_.at(c.action.pose), c.gen);
          }
          break;
        }
        case ArmCommand::Kind::CANCEL_ACTIVE:
          cancel_trajectory(c.gen);
          break;
        case ArmCommand::Kind::VERIFY_START:
          // 이 시점 이후의 측정만 인정 (궤적 도중 · 이전 작업의 측정으로 확인되지 않게)
          fold_->reset();
          RCLCPP_INFO(get_logger(), "home 동작 끝 → 관절 실측으로 접힘 확인 중");
          break;
        case ArmCommand::Kind::FINISH: {
          auto result = std::make_shared<ArmTask::Result>();
          result->success = c.end == ArmEnd::SUCCEEDED;
          result->message = c.message;
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
  }

  // "시간표" 한 줄(목표 자세 1개, move_time 초 동안)을 재생기에 보냄. 답은 콜백 → runner_ 로 전달
  void send_trajectory(
    const FJTClient::SharedPtr & client, const std::vector<std::string> & joints,
    const std::vector<double> & positions, unsigned gen)
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
    trajectory_msgs::msg::JointTrajectoryPoint p;
    p.positions = positions;
    p.time_from_start = rclcpp::Duration::from_seconds(move_time_s_);
    goal.trajectory.points.push_back(p);

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
  std::unique_ptr<FoldCheck> fold_;          // 접힘 실측 확인 (R1)
  std::unique_ptr<HardwareWatch> hw_;        // 하드웨어 측정 살아 있나 (재검증 R6 · 보드 2장)
  std::vector<std::string> hw_watch_joints_;  // 측정을 볼 관절 (기본 14 개 = 주행 · 팔 보드 전부)
  rclcpp::Client<ListHw>::SharedPtr hw_list_client_;
  rclcpp::TimerBase::SharedPtr hw_timer_;
  bool hw_list_pending_{false};
  double hw_list_sent_{0.0};
  bool hw_ok_before_{false}, have_hw_once_{false};
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
