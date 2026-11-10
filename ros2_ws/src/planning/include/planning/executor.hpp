// executor.hpp — 실행기: 미션이 준 단계 목록을 Nav2 · /arm_task 로 실행, 미션 내용은 모름
//
// 역할
//   - 단계 실행: Navigate · Arm, Custom 은 미션이 넘긴 함수로
//   - 재시도, 팔 실패 뒤 HOME 끼워 넣기, DRIVE 대기
//   - 실행 조건 감시 + 준비 확인: odom · motion_mode 끊김, scan · 검출 · 위치 추정, Nav2 lifecycle
//   - 퍽 기록부 = 검출 → map, odom 기준 최근 검출 = PICK target, 적재함 기록
//   - ArmTask target 계산: PICK = 퍽 집기 점 · 방향, Custom 목표 = 미션이 준 점 → base_link
// 입력
//   - detection_topics     interfaces/DetectionArray  퍽, 촬영 시각 TF 로 map · odom 변환. 기본 /perception/wrist/detections
//   - drift_topic          geometry_msgs/QuaternionStamped  손목 카메라가 돈 만큼 (camera_drift_node, 기본 /wrist_cam/drift).
//                          재검증용 원시 관측 (recent_obs) 에만 되돌려 곱함. 기록부 · PICK 목표는 명목 TF 그대로 (control 이 따로 되돌림)
//   - rack_topic           interfaces/DetectionArray  적재함 칸 (kind rack_cell, label = 칸 이름, base_link). 기본 /perception/rack/cells
//   - /control/motion_mode interfaces/MotionMode      주행은 DRIVE 일 때만, 팔 작업은 ready 일 때만
//   - /odom                nav_msgs/Odometry          제어 생존 확인
//   - scan_topics          sensor_msgs/LaserScan      라이다마다 생존 확인만, 기본 /lidar_front/scan · /lidar_rear/scan
//   - 위치 추정 품질, localization_source 로 고름
//       health (기본, 실물 slam_toolbox) = /localization_health  diagnostic_msgs/DiagnosticStatus  pose_guard 가 냄
//       amcl   (시뮬)                    = /amcl_pose            PoseWithCovarianceStamped        마지막 값 유지
//   - /tf                  map → odom 신선도, 검출 · 목표 변환
// 출력
//   - 액션 navigate_to_pose → Nav2, /arm_task → control_master
//   - 액션 dock_action (기본 dock_to_pose, NavigateToPose 형) → RL 정밀 접근 노드. Step.dock 인 이동만
//   - 서비스 호출 <lifecycle_manager>/is_active, 1 Hz
//   - 서비스 호출 rack_enable_service std_srvs/SetBool, 기본 /perception/rack/enable (LOOK_RACK 동안만 켬)
//   - /mission/cargo       interfaces/CargoState              적재함 · 그리퍼 재고, 바뀔 때 + 1 Hz
//   - /planning/markers    visualization_msgs/MarkerArray     퍽 기록부 · 후보 · Nav2 목표, 2 Hz
// 실행
//   - planning_exec 라이브러리, 실행 파일 없음. mission_manager 노드 안 멤버로 돎
//   - 파라미터 이름은 config/planning.yaml 그대로
// note
//   - 미션 상태 · FAULT 결정 · /mission/status · /mission/hold 는 안 가짐 → mission 패키지
//   - 단계 목록은 주기 사이에만 바꿈: set_steps · insert_steps · erase_steps · insert_home
//       Custom 단계 실행 중에는 insert_after_current 만, 성공하면 바로 뒤에 끼움
//   - step() 은 사건 ExecEvent 를 돌려줌 → 실패 뒤 처리와 FAULT 결정은 미션이 함
//   - 재시도는 실행기가 함: nav_retries · pick_retries · place_retries, PICK 재시도는 앞 이동부터
//       PICK 이 팔을 움직이고 실패하면 (pick_relook) HOME → 같은 정차 LOOK 다시 → 그 뒤 검출로 PICK (relook_wait_s 안에 안 보이면 실패)
//       PICK · STORE · RETRIEVE · PLACE · LOOK_RACK 은 보내기 전 실패 · 목표 거절일 때만 재시도 (팔 안 움직임)
//   - 재고 불확실 (inventory_policy.hpp): 위 작업이 수락된 뒤 성공으로 안 끝남 (실패 · 취소 · 시간 초과 · 결과 모름)
//       → 재시도 · StepFailed 없이 Fault INVENTORY_UNCERTAIN, 재고 = 마지막 확인 그대로 + 불확실 표시 (원인 단계)
//       → stop() 때 진행 중 · 결과 대기 중이던 위 작업도 같이 표시. 결과는 왔는데 step() 이 처리 전이면 그 결과대로 반영
//       → 표시가 있으면 ready_reasons 에 INVENTORY_UNCERTAIN (start 거절), clear_gripper 거절
//       → 지우는 길: 원인 작업의 늦은 최종 성공 (control 확인) 또는 사람 확인 resolve_inventory. HOME 성공으로는 안 지움
//       → 늦은 최종 성공도 기록과 안 맞으면 반영 · 해소 안 함 (표시 유지). RUNNING 중 표시가 있으면 supervise 가 FAULT
//   - 최종 성공인데 기록과 안 맞는 전환 (apply_arm_success 실패) → 재고 · 기록부 그대로, 불확실 표시 + Fault INVENTORY_UNCERTAIN
//       정상 · 시간 초과 뒤 성공 · FAULT 뒤 늦은 성공 (update) · stop() 선반영 모두 update_cargo 한 곳으로 (각 결과는 한 번만 반영)
//   - 보내기 전 재고 확인 (arm_preflight_ok): PICK · STORE · LOOK_RACK · RETRIEVE · PLACE 의 퍽 · 칸이 기록과 안 맞으면
//       팔 명령 없이 Fault INVENTORY_MISMATCH (재시도 · 재계획 안 함, 팔 안 움직였으므로 불확실 표시는 안 함)
//   - 감시 대기 원인별 차단 (새 목표 보내기 전만, 진행 중 팔 작업은 안 취소, 회복 · FAULT 는 supervise)
//       DRIVE_LOST · SCAN_LOST = 이동 차단 / DETECTION_LOST = PICK 시작 차단
//       LOCALIZATION_LOST = 이동 + PICK + Target::Puck · Custom 팔 목표 (PLACE 등) 시작 차단, HOME · LOOK 류 · STORE · RETRIEVE 허용
//         Custom 은 resolver 프레임을 미리 모름 (미션 PLACE = map) → 보수적으로 차단 (arm_start_blocked, health.hpp)
//   - 취소한 목표의 최종 결과가 오기 전에는 새 목표를 안 보냄, server_wait_s 넘으면 단계 실패 (서버가 없어지면 안 기다림)
//   - 팔 시간 초과는 취소 결과까지 기다려 판정, 결국 성공이면 단계 성공, 결과가 끝내 안 오면 FAULT (ARM_NO_RESULT)
//   - FAULT 등으로 취소한 팔 작업이 결국 성공하면 update() 가 재고에 반영 (한 번만). 취소한 팔 작업은 HOME 필요로 봄
//       사람이 정합한 뒤 온 늦은 결과는 반영 안 함 (사람 확인이 기준)
//   - 미션 판단으로 단계 목록을 바꿀 때는 arm_busy() 가 false 일 때만, 팔 결과 처리 전 교체 방지
//   - 정밀 접근 (Step.dock, 박스 · 장소 정차): 정차점이 박스 면 1 cm (0.2575 m) 라 Nav2 외곽 · collision_monitor 와 겹침
//       → 1 단계 Nav2 목표 = 정차점 뒤 dock_nav_back_m (박스 면 2 cm = nf 학습 구간 끝과 같음, planner 가 경로 냄)
//         정차점 뒤 통로 (뒤 dock_handoff_m · 옆 dock_handoff_lat_m · 방향 dock_handoff_yaw_deg) 안에 들어오면 Nav2 취소
//         → 취소 결과 뒤 바로 2 단계 dock_action 으로 정차점 (서지 않고 넘김)
//         Nav2 목표가 넘김 자리보다 멀어서 MPPI · nf 가 넘김 전에 감속 안 함. 넘김이 안 걸리면 Nav2 도착 뒤 2 단계
//       두 단계가 한 이동 단계 (재시도 = 1 단계부터)
//       dock_action "" 이면 정차점으로 Nav2 바로 (가짜 세계). 접근 노드가 collision_monitor 영역을 끄고 켬
//   - 그리퍼 기록에 퍽이 있는데 이동 단계가 오면 이동 안 하고 Fault CARRY_DRIVE (퍽 쥔 채 주행 안 함)
//   - 팔이 LOOK · LOOK_BARCODE 로 펴진 채 이동 단계가 오면 HOME 을 먼저 끼움 (control 은 접힘 실측 뒤에만 DRIVE → 기다리면 drive_wait_s 손해)
//   - LOOK_RACK: 성공 뒤 그 칸 측정을 rack_wait_s 동안 기다림 (팔 결과 뒤 촬영분만) → 다음 STORE target 에 넣음
//       perception 적재함 판독은 LOOK_RACK 보낸 뒤 ~ 측정 기다리기 끝까지만 켬 (rack_enable_service, 손목 CPU 절약)
//       못 받거나 LOOK_RACK 이 거절되면 단계는 성공으로 넘기고 STORE target 은 빈 프레임 (= 녹화 경로 그대로)

#pragma once

#include <array>
#include <cmath>
#include <deque>
#include <map>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/quaternion_stamped.hpp"
#include "interfaces/action/arm_task.hpp"
#include "interfaces/msg/cargo_state.hpp"
#include "interfaces/msg/detection_array.hpp"
#include "interfaces/msg/motion_mode.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include <cmath>
#include <limits>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "planning/approach.hpp"
#include "planning/cargo.hpp"
#include "planning/health.hpp"
#include "planning/inventory_policy.hpp"
#include "planning/puck_registry.hpp"
#include "planning/types.hpp"

namespace planning
{

// 한 단계의 결과
enum class StepResult { Running, Success, Fail };

// step() 한 번의 결과
struct ExecEvent
{
  enum class Kind
  {
    None,          // 진행 중 · 대기 중 (또는 재시도로 다시 시작)
    StepOk,        // 단계 성공 (step = 그 단계, all_done = 목록 끝)
    StepFailed,    // 재시도까지 실패 (step = 그 단계, 순번은 그 단계에 머묾)
    DriveReady,    // wait_drive() 뒤 DRIVE 받음
    Fault,         // 계속 못 함 (code · detail) → 미션이 FAULT 처리
  };
  Kind kind = Kind::None;
  Step step;
  bool all_done = false;
  bool need_home = false;                 // StepFailed: 팔이 움직이다 성공으로 안 끝남 (접혔는지 모름)
  std::optional<CargoItem> placed;        // StepOk PLACE: 놓은 퍽 (재고 기록 기준, 기록 불일치면 없음)
  std::string code;                       // Fault 원인 코드
  std::string detail;                     // Fault · StepFailed 이유
};

// 퍽 원시 관측 (기록부 병합 · 수신 시각을 거치지 않음, 최근 30 s). 재검증처럼 '그 시각 이후 본 것' 이 필요할 때
struct ObsRecord
{
  double stamp = 0.0;                     // 촬영 시각 [s]
  std::string id, color, shape;           // 이번 장에서 읽은 그대로 (ID 못 읽으면 "")
  double x = 0.0, y = 0.0;                // map [m]
};

class Executor
{
public:
  // Custom 단계 실행: (단계, 지금, 단계 시작 시각, 실패 이유) → 결과. 실행기 단계 목록은 insert_after_current 만 허용
  using CustomRunner = std::function<StepResult(const Step &, double now, double started_s, std::string & detail)>;
  // Target::Custom 목표: 단계 → 점 (아무 프레임, 위치만). false 면 단계 실패 (detail 에 이유)
  using TargetResolver = std::function<bool(const Step &, geometry_msgs::msg::PoseStamped & out, std::string & detail)>;
  // 팔 작업을 보내기 직전 (LOOK 시작 시각 기록 등, 실행기 상태를 바꾸면 안 됨)
  using ArmSentHook = std::function<void(const Step &, double now)>;

  explicit Executor(rclcpp::Node & node);

  void set_custom_runner(CustomRunner f) {custom_ = std::move(f);}
  void set_target_resolver(TargetResolver f) {resolver_ = std::move(f);}
  void set_arm_sent_hook(ArmSentHook f) {arm_sent_ = std::move(f);}

  // ---- 주기
  void update(double now);                               // 상태 무관 정리 (후보 퍽 지우기)
  std::optional<ExecEvent> supervise(double now);        // RUNNING: 제어 끊김 · 감시 회복 실패 → Fault 사건
  ExecEvent step(double now);                            // RUNNING: DRIVE 대기 · 현재 단계 실행

  // ---- 단계 목록 (주기 사이에만)
  void set_steps(std::vector<Step> steps);               // 순번 0 부터, 재시도 · DRIVE 대기 초기화
  const std::vector<Step> & steps() const {return steps_;}
  std::size_t index() const {return step_i_;}
  bool done() const {return step_i_ >= steps_.size();}
  void erase_steps(std::size_t a, std::size_t b);        // [a, b)
  void insert_steps(std::size_t at, const std::vector<Step> & more);
  void insert_after_current(std::vector<Step> more);     // Custom 실행 중: 성공하면 바로 뒤에 끼움
  void insert_home(const std::string & why);             // 지금 단계 앞에 HOME (DRIVE 복구 셈에 안 넣음)
  void wait_drive(double now);                           // HOME 단계 뒤: DRIVE 받으면 DriveReady
  bool waiting_drive() const {return waiting_drive_;}
  std::string current_step_name() const {return step_i_ < steps_.size() ? steps_[step_i_].name : "";}

  // ---- 멈추기 · 재개 확인
  void stop();                                           // 모든 목표 취소 + DRIVE 대기 해제 (FAULT), 진행 중 재고 작업은 불확실 표시
  void cancel_nav();
  std::string cancel_pending_reason() const;             // 취소한 목표의 최종 결과 전이면 이유 (resume 막음)

  // ---- 준비 · 대기 원인
  void reset_monitors();
  std::vector<std::string> ready_reasons(double now);    // STANDBY 준비 안 된 이유 (비면 ready)
  std::vector<std::string> wait_reasons() const;         // RUNNING 대기 원인 (DRIVE_LOST · SCAN_LOST …)
  bool nav_blocked() const;                              // 이동 차단 중 (hold)
  // 팔 작업 중이거나, 결과는 왔는데 아직 step() 이 재고에 반영 안 함 → 미션은 이때 단계 목록을 바꾸면 안 됨
  bool arm_busy() const;
  std::string arm_phase() const {return arm_.active ? arm_phase_ : "";}

  // ---- 기록부 · 재고
  const PuckRegistry & registry() const {return registry_;}
  const Cargo & cargo() const {return cargo_;}
  bool cargo_retrieve(const std::string & slot, std::string * error) {return cargo_.on_retrieve(slot, error);}   // 시험용 (한 번만 반영 확인)
  const std::deque<ObsRecord> & recent_obs() const {return raw_obs_;}
  void mark_puck(const std::string & id, const std::string & state) {registry_.set_state(id, state);}
  void cargo_changed();                                  // 저장 + 발행
  bool clear_gripper(std::string & why);                 // /mission/clear_gripper (저장 + 발행). 재고 불확실이면 거절
  // 재고 불확실 사람 정합 (실물 확인 뒤). 팔 결과 대기 중 · 표시 없음 · 기록과 안 맞음이면 false + msg
  bool resolve_inventory(DoubtFix fix, std::string & msg);
  std::string doubt_text() const;                        // 불확실 표시 설명 (없으면 "")
  void set_task_id(const std::string & id) {task_id_ = id;}   // PICK 때 재고 기록에 적음
  void set_pick_fresh_s(double s) {pick_fresh_override_ = s;}  // < 0 = pick_fresh_s 파라미터

  // ---- 설정값 (미션 목표 계산용)
  double puck_half_height_m() const {return puck_half_height_m_;}
  double grasp_offset_m() const {return grasp_offset_m_;}
  const std::string & map_frame() const {return map_frame_;}
  // in → target 프레임 (최신 TF, 위치 · 방향). 정차 중 측정을 지도로 옮길 때 씀. 실패하면 false + why
  bool transform_pose(const geometry_msgs::msg::PoseStamped & in, const std::string & target,
    geometry_msgs::msg::PoseStamped & out, std::string * why = nullptr) const;

private:
  using Nav = nav2_msgs::action::NavigateToPose;
  using NavGoalHandle = rclcpp_action::ClientGoalHandle<Nav>;
  using ArmTask = interfaces::action::ArmTask;
  using ArmGoalHandle = rclcpp_action::ClientGoalHandle<ArmTask>;
  using Trigger = std_srvs::srv::Trigger;
  using SetBool = std_srvs::srv::SetBool;

  // 액션 목표 1개의 진행 상황 (gen: 보낼 때마다 증가, 늦게 온 옛 콜백은 gen 이 달라 무시)
  struct GoalTrack
  {
    uint64_t gen = 0;
    bool active = false;
    bool done = false;
    bool rejected = false;
    bool success = false;
    std::string message;
    double sent_s = 0.0;
  };
  struct LocalSeen                           // ID 퍽의 최근 검출 (odom 기준, 집을 때 target 용)
  {
    geometry_msgs::msg::PoseStamped pose;
    bool has_orientation = false;
    bool yaw_known = false;                  // Detection.yaw_known (사각기둥 면 방향을 믿어도 되는지)
    bool is_box = false;
    double seen = 0.0;
  };
  struct TfCache                             // 검출 1 메시지 안 TF 조회 결과 (같은 프레임 · 시각이면 재사용)
  {
    std_msgs::msg::Header header;
    std::optional<geometry_msgs::msg::TransformStamped> tf;
  };
  struct RackSeen                            // 적재함 칸 측정 (perception /perception/rack/cells)
  {
    geometry_msgs::msg::PoseStamped pose;    // 칸 바닥 중심 + yaw
    double stamp = 0.0;                      // 촬영 시각 [s]
  };
  struct LifecycleCheck                      // Nav2 lifecycle_manager is_active
  {
    std::string name;
    rclcpp::Client<Trigger>::SharedPtr cli;
    bool pending = false;
    bool ok = false;
    double sent_s = 0.0;
    double checked_s = -1.0;
  };

  double now_s() const {return node_.get_clock()->now().seconds();}
  rclcpp::Logger logger() const {return node_.get_logger();}

  void on_detections(const interfaces::msg::DetectionArray & msg);
  const geometry_msgs::msg::TransformStamped * lookup_cached(
    TfCache & c, const std::string & target, const std_msgs::msg::Header & h, double wait_s);

  // 단계 실행
  StepResult run_step(const Step & s, double now);
  StepResult run_navigate(const Step & s, double now);
  StepResult run_approach(const Step & s, double now);   // 박스 앞 마지막 접근 (approach.hpp)
  void stop_approach();
  bool navigate_goal(const Step & s, Pose2D & goal);
  StepResult run_arm(const Step & s, double now);
  bool arm_target(const Step & s, geometry_msgs::msg::PoseStamped & target, double now);
  void to_grasp_point(geometry_msgs::msg::PoseStamped & target, bool has_q, bool yaw_known, bool is_box,
    const std::string & puck_id) const;
  bool to_base(const geometry_msgs::msg::PoseStamped & in, bool with_orientation, geometry_msgs::msg::PoseStamped & out);
  // 팔 작업 최종 성공 → 재고 · 기록부 반영 (같은 단계 불확실 표시 해소 포함, 저장 한 번). 기록과 안 맞으면 아무것도 안 바꾸고 false + why
  bool update_cargo(const Step & s, std::string & why);
  CargoItem pick_item(const Step & s) const;  // PICK 대상 퍽 → 재고 항목 (기록부 색 · 모양, task)
  void mark_doubt(const Step & s, const std::string & why);   // 재고 불확실 표시 (이미 있으면 첫 원인 유지)
  void on_fail(ExecEvent & ev);
  int retries_allowed(const Step & s) const;
  bool insert_home_counted(const std::string & why, ExecEvent & ev);
  void insert_home_at_current(const std::string & why);
  void insert_relook(const std::string & puck_id);
  bool update_drive_reason(double now, bool drive_step, ExecEvent & ev);
  StepResult wait_rack(const Step & s, double now);   // LOOK_RACK 성공 뒤 칸 측정 기다리기
  void sync_rack_reader(double now);                  // perception 적재함 판독 켜기 · 끄기를 LOOK_RACK 진행에 맞춤
  void sync_wrist(double now);                        // perception 손목 검출: 주행 목표 진행 중 끔, 그 밖 켬
  bool wrist_quiet(double now) const;                 // 손목 검출을 일부러 끈 동안 · 켠 직후 (검출 끊김 안 셈)
  void on_rack(const interfaces::msg::DetectionArray & msg);
  void stop_blocked_nav();

  // 감시
  std::vector<std::string> stale_scan_topics(double now) const;   // 끊긴 라이다 토픽 (하나라도 있으면 SCAN_LOST)
  std::vector<std::string> stale_detection_frames(double now) const;
  bool map_odom_fresh(double now);
  bool localization_lost(double now);
  static bool leaves_arm_unfolded(const Step & s);
  bool robot_pose(Pose2D & out);

  // 액션
  bool server_wait_expired(double now, const std::string & what);
  bool send_nav(const Pose2D & p, bool dock = false);
  StepResult poll_nav(double now, double timeout_s);
  bool send_arm(const ArmTask::Goal & goal);
  void cancel_arm();
  void keep_unapplied_arm_ok();              // arm_ 비우기 전: 반영 안 한 성공 → late_arm_ok_
  bool cancel_blocked() const;               // 취소한 목표의 최종 결과 전 → 새 목표 안 보냄

  void poll_lifecycle();
  void save_cargo();
  void publish_cargo();
  void publish_markers();

  rclcpp::Node & node_;

  // 설정
  std::string map_frame_, base_frame_, odom_frame_;
  double detection_max_age_s_, min_confidence_, candidate_max_age_s_, control_timeout_s_;
  bool require_scan_, require_detections_, require_localization_, require_motion_mode_;
  double scan_timeout_s_, detection_timeout_s_, loc_timeout_s_;
  double loc_ready_xy_std_, loc_ready_yaw_std_, loc_lost_xy_std_, loc_lost_yaw_std_;
  std::vector<std::string> detection_frames_;
  double action_retry_s_, action_retry_max_s_, server_wait_s_, nav_timeout_s_, arm_timeout_s_, drive_wait_s_;
  std::string dock_action_;                  // "" = 정밀 접근 안 씀
  double dock_nav_back_m_ = 0.025, dock_timeout_s_ = 40.0, dock_handoff_m_ = 0.40;
  double dock_handoff_lat_m_ = 0.15, dock_handoff_yaw_deg_ = 60.0;   // 넘김 통로 옆 · 방향
  bool nav_docking_ = false;                 // 지금 이동 단계가 2 단계 (정밀 접근) 중
  bool nav_handoff_ = false;                 // 1 단계 끝 (도착 또는 넘김 취소) → 2 단계 보낼 차례
  double puck_standoff_m_, pick_fresh_s_, puck_half_height_m_, grasp_offset_m_;
  double pick_fresh_override_ = -1.0;
  int nav_retries_, pick_retries_, place_retries_;
  bool pick_relook_ = true;                  // PICK 이 팔을 움직이고 실패하면 재시도 전에 같은 정차 LOOK 다시
  double relook_wait_s_ = 1.5;               // 다시 보기 LOOK 뒤 새 검출 기다리는 최대 시간
  std::map<std::string, double> relook_after_;   // 퍽 ID → 다시 보기 LOOK 성공 시각 (이 뒤 검출로만 집음)
  double rack_wait_s_, rack_fresh_s_;

  // 미션 함수
  CustomRunner custom_;
  TargetResolver resolver_;
  ArmSentHook arm_sent_;

  // 기록
  PuckRegistry registry_;
  std::unordered_map<std::string, LocalSeen> local_seen_;
  std::deque<ObsRecord> raw_obs_;
  Cargo cargo_;
  std::unordered_map<std::string, RackSeen> rack_seen_;   // 칸 이름 → 마지막 측정
  std::unordered_map<std::string, RackSeen> rack_meas_;   // LOOK_RACK 으로 받은 측정 → 다음 STORE target
  double rack_wait_start_ = -1.0;            // LOOK_RACK 성공 뒤 측정 기다리기 시작 (-1 = 안 기다림)
  std::string rack_enable_service_;          // perception 적재함 판독 켜기 (std_srvs/SetBool), "" = 안 부름
  rclcpp::Client<SetBool>::SharedPtr rack_cli_;
  bool rack_want_ = false;                   // LOOK_RACK 보낸 뒤 ~ 측정 기다리기 끝까지 true
  bool rack_known_ = false, rack_on_ = false, rack_req_pending_ = false;
  double rack_req_sent_s_ = 0.0, rack_last_try_s_ = -1e9;
  int rack_req_gen_ = 0;
  std::string wrist_enable_service_;         // "" = 손목 검출 켜고 끄기 안 함
  double wrist_on_grace_s_ = 2.0;            // 켠 뒤 이 시간은 검출 끊김 안 셈
  rclcpp::Client<SetBool>::SharedPtr wrist_cli_;
  bool wrist_known_ = false, wrist_on_ = false, wrist_req_pending_ = false;
  double wrist_req_sent_s_ = 0.0, wrist_last_try_s_ = -1e9, wrist_on_s_ = -1.0;
  int wrist_req_gen_ = 0;
  std::string cargo_file_;
  std::string task_id_;

  // 단계
  std::vector<Step> steps_;
  std::size_t step_i_ = 0;
  std::vector<int> retries_;
  std::vector<Step> pending_after_;          // Custom 실행 중 끼울 단계
  bool step_running_ = false;
  double step_start_s_ = 0.0;
  std::string detail_;
  std::optional<CargoItem> last_placed_;
  bool waiting_drive_ = false;
  double waiting_drive_since_ = 0.0;
  double arm_try_start_ = -1.0;
  double server_wait_start_ = -1.0;          // 액션 서버 대기 시작 (-1 = 대기 안 함)

  // 감시
  bool w_drive_ = false;                     // 주행 단계인데 DRIVE 아님
  double drive_wait_start_ = -1.0;
  int recover_homes_ = 0;                    // DRIVE 받기 전까지 끼운 복구 HOME 수
  bool arm_needs_home_ = false;              // 팔이 움직였는데 성공으로 안 끝남
  bool arm_unfolded_ = false;                // 마지막 팔 작업 뒤 팔이 펴졌을 수 있음 (LOOK 류 · STORE · PLACE) → 이동 전 HOME
  ConditionMonitor scan_mon_, det_mon_, loc_mon_;
  std::vector<std::string> scan_topics_;
  std::vector<double> last_scan_s_;          // scan_topics_ 와 같은 순서, -1 = 아직 못 받음
  double last_any_det_s_ = -1.0;
  std::unordered_map<std::string, double> last_det_s_;
  std::optional<PoseStd> pose_std_;
  bool loc_from_amcl_ = false;               // true = /amcl_pose 공분산 (시뮬), false = /localization_health (실물 slam)
  LocHealth loc_health_;
  std::vector<LifecycleCheck> lifecycle_;

  bool have_mode_ = false;
  uint8_t mode_ = interfaces::msg::MotionMode::STOP;
  bool mode_ready_ = false;
  std::string mode_reason_;
  double last_mode_s_ = -1.0;
  double last_odom_s_ = -1.0;
  double odom_x_ = 0.0, odom_y_ = 0.0, odom_yaw_ = 0.0;
  rclcpp::Time odom_stamp_{0, 0, RCL_ROS_TIME};   // 마지막 새 /odom 의 측정 시각 (신선도는 이 시각의 나이로 봄)
  struct OdomSample {double t, x, y;};
  std::deque<OdomSample> odom_hist_;        // 최근 odom (스캔 측정 시각의 위치를 맞추려고, 마지막보다 kOdomHistS 앞까지 덮음)
  static constexpr double kOdomHistS = 1.0;
  rclcpp::Time scan_stamp_{0, 0, RCL_ROS_TIME};
  double travel_at(double t) const;          // 측정 시각 t 의 직진 거리 (odom_hist_ 사이 보간, 기록보다 옛 시각이면 NaN)

  // 박스 앞 마지막 접근 (approach.hpp)
  ApproachConfig approach_cfg_;
  double approach_half_width_m_ = 0.06;      // 박스 면으로 보는 좁은 띠 (몸 중심선 ±)
  double approach_body_half_m_ = 0.18;       // 다른 물체를 보는 차체 폭 띠 (몸 반폭 0.16 + 여유)
  double approach_timeout_s_ = 8.0, approach_sensor_max_age_s_ = 0.3;   // odom 측정 나이 상한
  double approach_scan_max_age_s_ = 0.4;     // 앞쪽 스캔 측정 나이 상한 (10 Hz 한 주기 + 전달 지연)
  static constexpr double kStampFutureS = 0.1;   // 측정 시각이 지금보다 이만큼 넘게 미래면 버림 (시계 어긋남)
  static constexpr double kApproachYawTol = 5.0 * M_PI / 180.0;   // 접근 중 몸 방향이 시작보다 이만큼 넘게 틀어지면 멈춤
  // 시작점 (x0, y0, yaw0) 은 Nav2 가 끝난 뒤 측정된 신선한 odom 으로만 잡음 (origin), t0 = 그 odom 측정 시각 (이전 스캔은 버림)
  struct ApproachRun
  {
    bool active = false, origin = false;
    double start_s = 0.0, t0 = 0.0, x0 = 0.0, y0 = 0.0, yaw0 = 0.0;
    ApproachState st;
  } approach_;
  double front_scan_s_ = -1.0;               // 접근 중 관측에 쓴 앞쪽 스캔의 측정 시각 (신선도는 이 나이로 봄)
  // 시작 자세 앞 방향으로 간 거리 (부호 있음) · 시작점에서 직선 거리 (안전 상한용)
  double approach_forward(double x, double y) const
  {
    return (x - approach_.x0) * std::cos(approach_.yaw0) + (y - approach_.y0) * std::sin(approach_.yaw0);
  }
  double approach_dist() const {return std::hypot(odom_x_ - approach_.x0, odom_y_ - approach_.y0);}

  // 액션
  uint64_t gen_ = 0;
  GoalTrack nav_, arm_;
  NavGoalHandle::SharedPtr nav_handle_;
  ArmGoalHandle::SharedPtr arm_handle_;
  uint64_t nav_cancel_gen_ = 0, arm_cancel_gen_ = 0;
  bool nav_cancel_pending_ = false, arm_cancel_pending_ = false;
  // 취소한 팔 작업이 결국 성공함 → 다음 step() 에서 재고에 반영 (실물이 기준)
  //   arm_step_ = 보낸 팔 단계, 성공을 재고에 반영하면 비움 (반영 전에 arm_ 을 비우면 late_arm_ok_ 로 넘김)
  std::optional<Step> arm_step_, arm_cancel_step_, late_arm_ok_;   // 보낸 팔 단계 · 취소한 팔 단계 · 늦게 성공한 단계
  bool arm_timeout_wait_ = false;            // 팔 시간 초과 → 취소 결과 기다리는 중 (성공이면 그 단계 성공)
  ArmEnd arm_end_ = ArmEnd::NotSent;         // 이번 run_arm 실패의 끝난 모양 (on_fail 이 재고 불확실 판정)
  bool preflight_refused_ = false;           // 이번 run_arm 이 재고 기록 불일치로 팔 명령을 안 보냄 (on_fail → FAULT)
  double arm_cancel_s_ = 0.0;
  Pose2D nav_goal_pose_;
  std::string arm_phase_;

  // ROS
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::vector<rclcpp::Subscription<interfaces::msg::DetectionArray>::SharedPtr> sub_det_;
  rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr sub_drift_;
  std::optional<geometry_msgs::msg::QuaternionStamped> drift_;   // 마지막 손목 카메라 돌아감 (frame = 그 카메라 광학 프레임)
  rclcpp::Subscription<interfaces::msg::DetectionArray>::SharedPtr sub_rack_;
  rclcpp::Subscription<interfaces::msg::MotionMode>::SharedPtr sub_mode_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr> sub_scan_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_amcl_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr sub_loc_health_;
  rclcpp::Publisher<interfaces::msg::CargoState>::SharedPtr pub_cargo_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_approach_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_front_scan_;
  rclcpp_action::Client<Nav>::SharedPtr nav_client_, dock_client_;
  rclcpp_action::Client<Nav>::SharedPtr nav_cur_client_;   // 마지막 이동 목표를 보낸 쪽 (취소 · 대기 판정)
  rclcpp_action::Client<ArmTask>::SharedPtr arm_client_;
  rclcpp::TimerBase::SharedPtr lifecycle_timer_, cargo_timer_, marker_timer_;
};

}  // namespace planning
