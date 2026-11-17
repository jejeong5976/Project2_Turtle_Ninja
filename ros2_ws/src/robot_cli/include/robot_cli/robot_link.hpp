// robot_link.hpp — ROS 2 연결 (노드 robot_cli): 상태 받기 + 명령 보내기. 인터페이스는 docs/integration/robot_interfaces.md
//   받음: /joint_states · _corrected (effort = 모터 전류 [A]) · /robot_description 한계 · /rosout 배터리 로그 · 라이다 · IMU
//         · motion_mode · _ack · arm_task / navigate_to_pose status · feedback · /odom · TF map → base_footprint
//         · /localization_health · /amcl_pose · 인지 · 미션 · /diagnostics (모터 온도) · /scan
//         · 1 Hz: controller_manager 목록 · Nav2 is_active · 노드 목록 · /slam_toolbox mode
//   보냄: /arm_task (인자 없는 place = 이 CLI 의 마지막 성공 PICK 목표, LastPick) · navigate_to_pose (map) · Trigger · SetBool
//   /cmd_vel (TwistStamped, 20 Hz, 끝나면 0 몇 번): Nav2 가 떠 있으면 거절 (Nav2 중 teleop 금지, force 로 무시).
//     허용 조건 (차례 DRIVE · Nav2 없음 · 시계) 은 틱마다 다시 봄 → 잃으면 멈추고 0.
//     속도 고르기 · 보내기는 estop 과 같은 락 (pub_mu_) 안 → estop 의 0 뒤에 옛 속도가 안 나감
//   finish() (Ctrl-C · quit): 이 CLI 의 진행 중 goal 취소 + 답이 아직 없는 goal 은 받는 대로 취소,
//     그래도 답이 없으면 그 서버의 모든 goal 취소 (보낸 goal 이 아무도 안 보는 채 움직이지 않게)
//   cancel · estop: 서버의 모든 goal 취소 + 아직 받음 답이 없는 이 CLI 의 goal 도 받는 대로 취소.
//     estop 은 잠금: 풀 때까지 /cmd_vel 0 + 새로 시작된 goal 도 다시 취소. 소프트웨어 정지일 뿐 — 모터 전원 차단 아님
#pragma once

#include <atomic>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "action_msgs/msg/goal_status_array.hpp"
#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "controller_manager_msgs/srv/list_hardware_components.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "interfaces/action/arm_task.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

#include "robot_cli/command.hpp"
#include "robot_cli/model.hpp"
#include "robot_cli/snapshot.hpp"
#include "robot_cli/status_view.hpp"

namespace robot_cli
{

class RobotLink : public rclcpp::Node
{
public:
  using ArmTask = interfaces::action::ArmTask;
  using Nav = nav2_msgs::action::NavigateToPose;
  // 끝났을 때 한 번: code 0 = 성공 · 1 = 응답 없음 · 2 = 거절 · 실패, text = 사람 글
  using Done = std::function<void(int code, const std::string & text)>;

  explicit RobotLink(Config cfg);

  Snapshot snapshot();
  // 서버 발견을 기다리는 최대 시간 — 명령 하나 모드 3 s (막 켜서 발견 전), 화면 0 (화면이 멈추지 않게)
  void set_discovery_wait(double s) {discovery_wait_s_ = s;}
  // Ctrl-C 표시 — 서버 · 서비스를 기다리다가 이게 켜지면 보내지 않고 끝냄
  void set_interrupt(const std::atomic<bool> * f) {interrupt_ = f;}
  // 화면: 기록 칸을 짧게 (보냄 · phase 줄 빼고 받음 · 결과만)
  void set_quiet(bool q) {quiet_ = q;}
  const Config & config() const {return cfg_;}

  // accepted = 서버가 받았을 때 한 번 (없어도 됨)
  void send_arm(const Command & c, Done done, std::function<void()> accepted = nullptr);
  void send_nav(const Command & c, Done done, std::function<void()> accepted = nullptr);
  int cancel_mine();                                  // 이 CLI 가 보낸 진행 중 팔 · 주행 goal 만 취소, 보낸 수
  // 끝내기: cancel_mine + 답이 아직 안 온 goal 은 받는 대로 취소 (max_s 까지 기다림), 그래도 답이 없으면
  //   그 서버의 모든 goal 취소. 이 뒤로 새 goal 은 보내지 않음. 한 일을 글로 (없으면 빈 글)
  std::string finish(double max_s);
  bool servers_ready(bool arm, bool nav);
  // 발행자 · 서버가 있는데 아직 못 받은 값이 있나 (명령 하나 status: 발견이 덜 끝난 화면을 찍지 않게)
  bool missing_data();
  // arm · nav · all — 서버의 모든 goal 취소 (지금 발견된 서버만, 기다리지 않음). 보낸 서버 이름들 (없으면 빈 글)
  std::string cancel(const std::string & which);
  void set_estop(bool on);
  // 수동 주행. "" = 시작함, 아니면 거절 이유
  std::string drive(double vx, double vy, double wz, double secs, bool force);
  std::string set_teleop(bool on, bool force = false);
  void teleop_cmd(double vx, double vy, double wz);   // teleop 중 키 하나 = 0.6 s 유지
  void stop_drive();
  bool drive_busy();                                  // 주행 중이거나 끝 0 을 아직 보내는 중
  std::string take_drive_lost();                      // 주행 조건을 잃어 멈췄으면 그 이유 (한 번 읽으면 지움)
  void call_service(const std::string & name, int setbool, Done done);

  void log(const std::string & text);
  std::vector<std::string> take_log();

private:
  double now_s() const;
  void note_goal(const std::string & id, const std::string & what);   // mu_ 쥔 채로
  TaskTiming & timing_of(const std::string & id);                      // mu_ 쥔 채로
  void fail(const Done & done, int code, const std::string & text);   // 기록 + done
  bool interrupted() const {return interrupt_ && interrupt_->load();}
  bool wait_service(rclcpp::ClientBase & cli);
  DriveFacts facts_locked(double t) const;   // mu_ 쥔 채로
  void clamp_speed(double & vx, double & vy, double & wz) const;   // 설정 한계 (controllers.yaml) 안으로
  std::string drive_guard(bool force);
  void on_drive_tick();
  void on_poll();
  struct Poll;
  template<typename Srv, typename F>
  bool ask(Poll & p, double t, F on_answer);
  void on_tf();
  void publish_twist(double vx, double vy, double wz);
  void cancel_all(bool arm, bool nav);

  Config cfg_;
  std::mutex mu_;
  Snapshot s_;
  std::map<std::string, std::pair<DiagView, double>> diag_by_name_;
  std::deque<TaskTiming> hist_;      // 팔 작업 시간 (s_.arm_hist 로 복사)
  std::deque<std::string> log_;

  rclcpp_action::Client<ArmTask>::SharedPtr arm_;
  rclcpp_action::Client<Nav>::SharedPtr nav_;
  rclcpp_action::ClientGoalHandle<ArmTask>::SharedPtr arm_gh_;   // mu_
  rclcpp_action::ClientGoalHandle<Nav>::SharedPtr nav_gh_;
  int arm_pending_{0}, nav_pending_{0};   // 보냈는데 받음 · 거절 답이 아직 없는 goal 수 (mu_)
  bool closing_{false};                   // finish() 뒤: 늦게 받아진 goal 은 바로 취소 (mu_)
  // 보낸 번호 · 취소한 번호 (mu_): 번호 ≤ cancel_upto 인 goal 이 늦게 받아지면 바로 취소 (cancel 이 받음보다 먼저 처리된 경우)
  uint64_t arm_seq_{0}, nav_seq_{0}, arm_cancel_upto_{0}, nav_cancel_upto_{0};
  std::map<std::string, double> joint_seen_, corr_seen_;   // 관절마다 받은 시각 (mu_)
  LastPick last_pick_;                    // 인자 없는 place 가 놓을 자리 (mu_, 이 CLI 의 팔 결과로)
  const std::atomic<bool> * interrupt_{nullptr};
  bool quiet_{false};
  std::mutex pub_mu_;                     // /cmd_vel 고르기 · 보내기 (drive 틱 · estop). 순서: pub_mu_ → mu_
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_cmd_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;
  rclcpp::TimerBase::SharedPtr drive_timer_, poll_timer_, tf_timer_;
  std::shared_ptr<tf2_ros::Buffer> tf_buf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listen_;

  // 1 Hz 묻기 — 서비스마다 기다리는 요청 하나만 (1.5 s 넘으면 버림)
  struct Poll
  {
    rclcpp::ClientBase::SharedPtr cli;
    bool busy{false};
    double sent{0.0};
  };
  Poll list_ctrl_, list_hw_, nav_active_, loc_active_;
  std::shared_ptr<rclcpp::AsyncParametersClient> slam_param_;
  double slam_asked_t_{-1e9};
  std::map<std::string, rclcpp::ClientBase::SharedPtr> call_clients_;

  // 수동 주행 · estop (drive_timer_ 20 Hz)
  bool estop_{false}, driving_{false}, teleop_{false}, moved_{false}, drive_force_{false};
  std::string drive_lost_;   // 마지막으로 주행 조건을 잃은 이유 (mu_)
  double dvx_{0}, dvy_{0}, dwz_{0}, drive_until_{0};
  int zero_left_{0};
  double last_recancel_{0};
  unsigned tick_{0};
  double discovery_wait_s_{3.0};
};

}  // namespace robot_cli
