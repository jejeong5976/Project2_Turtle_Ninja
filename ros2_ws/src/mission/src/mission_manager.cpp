// mission_manager.cpp — 미션 관리 노드 mission_manager, 발제 v5 '주사위 기억 로봇'
//
// 역할
//   - 미션 흐름: 바코드 읽기 → 신호등 12 점멸 기록 → 퍽 12 개 수집 · 배치 L1–L3 주사위 눈 → 재검증 → 복귀
//   - 단계 목록을 만들어 planning::Executor 에 넘기고, 실행기 사건 = 단계 성공 · 실패 · FAULT 로 다음 할 일을 정함
//   - 실제 동작 · 감시 · 퍽 기록부 · 재고는 Executor 가 함
//   - 상태 2 층
//       위층 = 로봇 운영 상태
//         STANDBY  무동작, 준비 확인: odom · control ready · 서버 · lifecycle · 위치 추정 · scan · 검출 · 그리퍼 기록
//         RUNNING  미션 실행, 아래층
//         DONE     끝, 결과 COMPLETED · RETURN_FAILED
//         FAULT    모든 목표 취소, 사람 확인 뒤 /mission/resume → STANDBY → /mission/start
//       아래층 = RUNNING 안 미션 단계: HOME → READ_BARCODE → OBSERVE_SIGNAL → COLLECT → DELIVER → VERIFY → RETURN_HOME
//         HOME           start 승인 뒤 · 복구 뒤 항상 여기부터. 출발 전에는 안 움직임
//         READ_BARCODE   이동 → LOOK_BARCODE → 읽기 → HOME. 이 단계 동안만 perception 바코드 판독을 켬
//         OBSERVE_SIGNAL 출발 구역에 선 채 12 점멸 기록 끝까지 기다림, signal_wait_s 넘으면 불완전으로 진행
//                        불완전 ('?' · 개수 · 끊김 · 횟수 규칙) 이면 signal_retries 번까지 기록을 비우고 다음 시퀀스를 다시 기록
//                        (운영자가 12 점멸 + 3 s 소등 뒤 다시 틀어 줌)
//         COLLECT        기록이 완전하고 횟수 규칙에 맞아야 시작, 아니면 복귀. 박스마다 이동 → LOOK → Plan
//         DELIVER        운반 계획대로 [RETRIEVE] PLACE, 목표 = 장소 중심 + 주사위 눈 오프셋
//         VERIFY         재검증 순서로 장소마다 이동 → LOOK → Plan Verify
//         RETURN_HOME    출발점으로 이동
// 입력
//   - /perception/traffic_light  interfaces/TrafficLight  신호등 상태 → 점멸 기록, signal_recorder.hpp
//   - /perception/task_text      interfaces/TaskText      바코드 원문 → parse_task
//   - box_topic                  interfaces/DetectionArray 장소 박스 윗면 중심 · yaw (손목 깊이, kind place_box) → map 으로 모음
//       장소 LOOK 뒤 측정이 box_min_samples 개 이상이면 PLACE 칸 · 재검증 기준 = 측정 중심 + 박스 회전만큼 돈 눈 오프셋
//   - 실행기가 받는 검출 · motion_mode · odom · scan · amcl 은 executor.hpp
//   - 서비스 std_srvs/Trigger
//       /mission/start          STANDBY + 준비 완료 → RUNNING, 안 되면 거절 + 이유
//       /mission/resume         FAULT → STANDBY, 사람이 로봇 · 재고 확인 뒤. 재고 불확실이면 거절
//       /mission/clear_gripper  그리퍼 재고 기록 지움, 기록이 남아 있으면 start 거절. 재고 불확실이면 거절
//       /mission/inventory_before   재고 불확실 정합: 실물이 원인 단계 전과 같음 (기록 그대로)
//       /mission/inventory_done     재고 불확실 정합: 실물이 원인 단계 성공과 같음 (성공 전환 한 번 반영)
//       /mission/inventory_removed  재고 불확실 정합: 사람이 그 퍽을 로봇에서 뺌 (그리퍼 · 그 칸 기록에서 지움)
//         셋 다 RUNNING 아닐 때 · 팔 작업 최종 결과 뒤에만. 사람이 실물을 직접 본 뒤 부를 것
//       /mission/reset_signal   신호등 기록 지우고 다시 기다림, RUNNING 중에는 출발 구역 단계일 때만
//   - 파일: dice_map = maps/dice_map.yaml, travel_table = maps/dice_travel_times.yaml, barcode_table = maps/dice_barcodes.json
// 출력
//   - /mission/status          interfaces/MissionStatus  2 Hz
//   - /mission/hold            std_msgs/Bool             주행을 막을 때 true: STANDBY · FAULT · 이동 차단 원인 → navigation cmd_vel_hold
//   - /planning/signal_record  interfaces/SignalRecord   바뀔 때 + 1 Hz, transient_local
//   - /planning/verify_record  interfaces/VerifyRecord   transient_local
//   - 서비스 호출 barcode_enable_service std_srvs/SetBool, 기본 /perception/barcode/enable
//   - 실행기 출력: navigate_to_pose · /arm_task · /mission/cargo · /planning/markers
//   - 기록 파일 <record_dir>/run_<시각>/ signal.json · signal_attempt<N>.json (다시 기록 전 불완전 기록) · allocation.json · allocation_rev<N>.json · verify.yaml
// 실행
//   ros2 launch mission mission.launch.py
// note
//   - 계산 = 해석 · dice · 운반 계획 · 재검증 판정 · 기록 파일은 ROS 를 모르는 파일로 분리 → test/
//   - 신호등은 STANDBY · HOME · 첫 READ_BARCODE 중에도 기록 → 운영자가 언제 신호를 시작해도 놓치지 않음. 이미 끝났으면 OBSERVE_SIGNAL 건너뜀
//   - 단계가 재시도 뒤에도 실패하면 그 퍽 · 장소를 빼고 재계획 dice_recover, max_replans 넘거나 못 하면 복귀. 놓은 퍽은 그대로
//   - 재고 작업 (PICK · STORE · RETRIEVE · PLACE · LOOK_RACK) 이 수락 뒤 성공으로 안 끝나면 재계획 없이 FAULT INVENTORY_UNCERTAIN
//       (실행기 판정, planning/inventory_policy.hpp). 사람이 실물 확인 → /mission/inventory_* → resume → start
//       최종 성공인데 재고 기록과 안 맞아도 같음. 보내기 전 단계 퍽 · 칸이 기록과 안 맞으면 팔 명령 없이 FAULT INVENTORY_MISMATCH
//   - 대기 원인은 상태가 아님: 실행기 wait_reasons = DRIVE_LOST · SCAN_LOST · DETECTION_LOST · LOCALIZATION_LOST
//   - time_limit_s > 0 이면 첫 start 부터 재서 시간이 모자랄 때 RETURN_HOME. 기본 0 = 제한 없음
//   - FAULT 원인: 실행기 Fault = 제어 끊김 · 감시 회복 실패 · HOME 실패 · DRIVE 복구 안 됨 · 재고 불확실, 경기장 파일 문제, 미등록 바코드, dice 계획 불가
//   - 파라미터는 레포 config/planning.yaml, 최상위 키 mission_manager

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <deque>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "interfaces/msg/detection_array.hpp"
#include "interfaces/msg/mission_status.hpp"
#include "interfaces/msg/signal_record.hpp"
#include "interfaces/msg/task_text.hpp"
#include "interfaces/msg/traffic_light.hpp"
#include "interfaces/msg/verify_record.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "planning/executor.hpp"

#include "mission/dice.hpp"
#include "mission/dice_map.hpp"
#include "mission/dice_record.hpp"
#include "mission/place_box.hpp"
#include "mission/dice_steps.hpp"
#include "mission/dice_verify.hpp"
#include "mission/signal_recorder.hpp"
#include "mission/task_parser.hpp"
#include "mission/transfer_planner.hpp"

namespace mission
{

using planning::ExecEvent;
using planning::Executor;
using planning::ObsRecord;
using planning::StepResult;
using interfaces::msg::MissionStatus;
using interfaces::msg::SignalRecord;
using interfaces::msg::TaskText;
using interfaces::msg::TrafficLight;
using interfaces::msg::VerifyPuck;
using interfaces::msg::VerifyRecord;
using Trigger = std_srvs::srv::Trigger;
using SetBool = std_srvs::srv::SetBool;

// 위층: 로봇 운영 상태
//   STANDBY → (/mission/start) → RUNNING → DONE
//   어디서든 → FAULT → (/mission/resume) → STANDBY
enum class Op { STANDBY, RUNNING, DONE, FAULT };

// 아래층: 미션 단계 (RUNNING 안에서만)
enum class Phase { HOME, OBSERVE_SIGNAL, READ_BARCODE, COLLECT, DELIVER, VERIFY, RETURN_HOME };

const char * op_name(Op o)
{
  switch (o) {
    case Op::STANDBY: return "STANDBY";
    case Op::RUNNING: return "RUNNING";
    case Op::DONE: return "DONE";
    case Op::FAULT: return "FAULT";
  }
  return "?";
}

const char * phase_name(Phase p)
{
  switch (p) {
    case Phase::HOME: return "HOME";
    case Phase::OBSERVE_SIGNAL: return "OBSERVE_SIGNAL";
    case Phase::READ_BARCODE: return "READ_BARCODE";
    case Phase::COLLECT: return "COLLECT";
    case Phase::DELIVER: return "DELIVER";
    case Phase::VERIFY: return "VERIFY";
    case Phase::RETURN_HOME: return "RETURN_HOME";
  }
  return "?";
}

std::string join(const std::vector<std::string> & v)
{
  std::string out;
  for (const auto & x : v) {out += (out.empty() ? "" : ", ") + x;}
  return out;
}

class MissionMaster : public rclcpp::Node
{
public:
  MissionMaster()
  : Node("mission_manager")
  {
    // 실행기 먼저 (실행기 파라미터 · 구독 · 액션 클라이언트를 같은 노드에 만듦)
    exec_ = std::make_unique<Executor>(*this);
    exec_->set_custom_runner([this](const Step & s, double now, double started, std::string & detail) {
        return run_custom(s, now, started, detail);
      });
    exec_->set_target_resolver([this](const Step & s, geometry_msgs::msg::PoseStamped & out, std::string & detail) {
        return dice_cell_target(s, out, detail);
      });
    exec_->set_arm_sent_hook([this](const Step & s, double now) {
        if (!is_look_task(s.arm_task)) {return;}
        if (box_of(s) >= 1 && box_of(s) <= kNumBoxes) {look_started_[box_of(s) - 1] = now;}
        if (place_of(s) >= 0 && place_of(s) < kNumPlaces) {
          look_place_started_[place_of(s)] = now;
          place_frame_[place_of(s)].reset();   // 새로 보면 기준도 새로 (kPlaceBox 가 확정)
        }
      });

    // ---------------------------------------------------------------- 설정값 (기본값은 여기, 실제 값은 레포 config/planning.yaml)
    node_start_s_ = now_s();   // 기록 파일 시각 기준
    dice_map_path_ = declare_parameter<std::string>("dice_map", "");
    if (dice_map_path_.empty()) {dice_map_path_ = std::string(MISSION_MAPS_DIR) + "/dice_map.yaml";}
    travel_table_path_ = declare_parameter<std::string>("travel_table", "");
    if (travel_table_path_.empty()) {travel_table_path_ = std::string(MISSION_MAPS_DIR) + "/dice_travel_times.yaml";}
    barcode_table_path_ = declare_parameter<std::string>("barcode_table", "");
    if (barcode_table_path_.empty()) {barcode_table_path_ = std::string(MISSION_MAPS_DIR) + "/dice_barcodes.json";}
    for (const auto b : declare_parameter<std::vector<int64_t>>("collect_order", {1, 2, 6, 4, 5, 3})) {
      collect_order_.push_back(static_cast<int>(b));
    }
    look_dwell_s_ = declare_parameter<double>("look_dwell_s", 1.0);           // LOOK 뒤 검출 모으는 시간
    // 정차 중 PICK: 차체가 서 있으면 LOOK 때 본 odom 기준 위치가 그대로 맞음 → STORE 뒤에도 씀
    dice_pick_fresh_s_ = declare_parameter<double>("dice_pick_fresh_s", 90.0);
    transfer_max_remaining_ = declare_parameter<int>("transfer_max_remaining", 4);
    // 재검증 (발제 '이동 후 검증'): 장소 반경 안 새 관측으로 칸 · 그룹 판정 → /planning/verify_record + 기록 파일
    verify_radius_m_ = declare_parameter<double>("verify_radius_m", 0.18);
    verify_cell_tol_m_ = declare_parameter<double>("verify_cell_tol_m", 0.03);
    // 재검증 퍽 색 · 모양 = LOOK 동안 읽힌 장 다수결, 1 등 비율이 이것보다 작으면 모름 (추정 안 함)
    verify_vote_ratio_ = declare_parameter<double>("verify_vote_ratio", 0.7);
    // 기록 파일 (발제 제출 '기록'): <record_dir>/run_<시각>/ signal.json · allocation.json · verify.yaml. "" = 안 씀
    record_dir_ = declare_parameter<std::string>("record_dir", "~/planning_logs");
    // 재계획: 단계가 재시도 뒤에도 실패하면 그 퍽 · 장소를 빼고 지금 상태에서 다시 계획 (이 횟수 넘으면 포기 · 복귀)
    max_replans_ = declare_parameter<int>("max_replans", 6);
    // 바코드는 손목으로 읽음: READ_BARCODE = 이동 → LOOK_BARCODE → 읽기 → HOME
    barcode_look_ = declare_parameter<bool>("barcode_look", true);
    // 적재함: STORE 앞에 LOOK_RACK (퍽 쥔 채 그 칸을 손목으로 봄 → perception 칸 측정 → STORE target)
    rack_look_ = declare_parameter<bool>("rack_look", false);
    // 장소 박스 측정 (손목 깊이): LOOK 뒤 측정 중앙값으로 칸 기준을 고침. 못 받거나 지도 중심에서 box_max_shift_m 넘게 멀면 지도 값
    box_measure_ = declare_parameter<bool>("box_measure", true);
    box_topic_ = declare_parameter<std::string>("box_topic", "/perception/wrist/box");
    box_min_samples_ = declare_parameter<int>("box_min_samples", 3);
    box_max_shift_m_ = declare_parameter<double>("box_max_shift_m", 0.06);
    box_wait_s_ = declare_parameter<double>("box_wait_s", 2.0);   // 장소 LOOK 뒤 측정 기다리는 최대 시간
    // 장소별 PLACE 놓는 점 보정 [앞, 왼쪽] mm (정차점에서 장소를 보는 로봇 기준) — 놓는 목표에만 더함, 재검증 · 기록은 눈 점 그대로
    //   실행 중 바로 바꿈 (다음 PLACE 부터): ros2 param set /mission_manager place_offset_L1_mm "[5.0, 0.0]"
    for (int i = 1; i <= kNumPlaces; ++i) {declare_parameter<std::vector<double>>("place_offset_L" + std::to_string(i) + "_mm", {0.0, 0.0});}
    read_timeout_s_ = declare_parameter<double>("read_timeout_s", 5.0);
    read_retries_ = declare_parameter<int>("read_retries", 6);   // 안 읽히면 같은 자리에서 다시 읽는 횟수 (11-08: 한 번 실패로 미션 전체 포기하던 것)
    pip_frame_ = declare_parameter<std::string>("dice_pip_frame", "arena");   // arena = 발제 p.4 지도 +Y 기준 세 곳 같은 방향 · viewer = 보는 사람 기준 (11-07 까지)
    tick_hz_ = declare_parameter<double>("tick_hz", 10.0);
    time_limit_s_ = declare_parameter<double>("time_limit_s", 0.0);
    return_margin_s_ = declare_parameter<double>("return_margin_s", 60.0);
    auto_start_ = declare_parameter<bool>("auto_start", false);

    // 신호등 점멸 기록 (발제 v5). 값 설명은 signal_recorder.hpp
    signal_timeout_s_ = declare_parameter<double>("signal_timeout_s", 1.0);
    signal_record_ = declare_parameter<bool>("signal_record", true);
    signal_wait_s_ = declare_parameter<double>("signal_wait_s", 90.0);   // OBSERVE_SIGNAL 최대 대기 (넘으면 불완전, 다시 기록할 때마다 새로 셈)
    // 기록이 불완전하면 비우고 다음 시퀀스를 다시 기록하는 횟수 (운영자가 다시 틀어 줌). 0 = 바로 포기 · 복귀
    signal_retries_ = declare_parameter<int>("signal_retries", 2);
    SignalRecorderConfig sc;
    sc.expected = declare_parameter<int>("signal_expected", sc.expected);
    sc.min_on_s = declare_parameter<double>("signal_min_on_s", sc.min_on_s);
    sc.min_off_s = declare_parameter<double>("signal_min_off_s", sc.min_off_s);
    sc.end_off_s = declare_parameter<double>("signal_end_off_s", sc.end_off_s);
    sc.max_gap_s = declare_parameter<double>("signal_max_gap_s", sc.max_gap_s);
    sc.majority = declare_parameter<double>("signal_majority", sc.majority);
    sc.max_ambiguous = declare_parameter<double>("signal_max_ambiguous", sc.max_ambiguous);
    sc.min_count = declare_parameter<int>("signal_min_count", sc.min_count);
    sc.max_count = declare_parameter<int>("signal_max_count", sc.max_count);
    signal_ = SignalRecorder(sc);
    // READ_BARCODE 동안만 perception 바코드 판독을 켬 (std_srvs/SetBool). "" 면 안 부름 (perception 이 늘 켜져 있을 때)
    barcode_enable_service_ = declare_parameter<std::string>("barcode_enable_service", "/perception/barcode/enable");

    // 경기장 좌표 · 이동 시간표 · 바코드 표
    std::string err;
    map_ok_ = load_dice_map(dice_map_path_, dice_map_, &err) && load_transfer_costs(travel_table_path_, costs_, &err) &&
      load_barcode_table(barcode_table_path_, barcodes_, &err);
    if (map_ok_) {
      RCLCPP_INFO(get_logger(), "dice_map: %s, 이동 시간표: %s, 바코드 표: %s (%zu 개)", dice_map_path_.c_str(),
        travel_table_path_.c_str(), barcode_table_path_.c_str(), barcodes_.size());
    } else {
      RCLCPP_ERROR(get_logger(), "dice 경기장 읽기 실패: %s", err.c_str());
    }

    // ---------------------------------------------------------------- 받기
    sub_sig_ = create_subscription<TrafficLight>(
      "/perception/traffic_light", 10, [this](TrafficLight::ConstSharedPtr m) {
        last_light_s_ = now_s();
        record_signal(last_light_s_, signal_obs(m->state, m->visible));
      });
    sub_text_ = create_subscription<TaskText>(
      "/perception/task_text", 10, [this](TaskText::ConstSharedPtr m) {
        last_text_ = m->raw_text;
        last_text_s_ = now_s();
      });

    if (box_measure_) {
      sub_box_ = create_subscription<interfaces::msg::DetectionArray>(
        box_topic_, 10, [this](interfaces::msg::DetectionArray::ConstSharedPtr m) {on_box(*m);});
    }

    // ---------------------------------------------------------------- 보내기
    pub_status_ = create_publisher<MissionStatus>("/mission/status", 10);
    pub_hold_ = create_publisher<std_msgs::msg::Bool>("/mission/hold", 10);
    pub_signal_ = create_publisher<SignalRecord>("/planning/signal_record", rclcpp::QoS(1).reliable().transient_local());
    pub_verify_ = create_publisher<VerifyRecord>("/planning/verify_record", rclcpp::QoS(1).reliable().transient_local());
    if (!barcode_enable_service_.empty()) {barcode_cli_ = create_client<SetBool>(barcode_enable_service_);}

    // ---------------------------------------------------------------- 서비스 (사람 · 대회 운영)
    srv_start_ = create_service<Trigger>(
      "/mission/start", [this](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> res) {
        res->success = start_mission(now_s(), res->message);
        RCLCPP_INFO(get_logger(), "/mission/start: %s", res->message.c_str());
      });
    srv_resume_ = create_service<Trigger>(
      "/mission/resume", [this](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> res) {
        res->success = resume_mission(res->message);
        RCLCPP_INFO(get_logger(), "/mission/resume: %s", res->message.c_str());
      });
    // 그리퍼 재고 기록 지우기 (사람이 퍽을 빼고 확인한 뒤). 그리퍼 기록이 남아 있으면 start 거절됨
    //   재고 불확실이면 거절 (불확실은 아래 inventory_* 로만 정합)
    srv_clear_gripper_ = create_service<Trigger>(
      "/mission/clear_gripper", [this](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> res) {
        if (op_ == Op::RUNNING) {
          res->success = false;
          res->message = "RUNNING 중에는 안 됨";
          return;
        }
        res->success = exec_->clear_gripper(res->message);
      });
    // 재고 불확실 정합 (사람이 실물을 보고 셋 중 맞는 것 하나). 성공해야 resume · start 가 됨
    const std::array<std::pair<const char *, planning::DoubtFix>, 3> fixes{{
      {"/mission/inventory_before", planning::DoubtFix::Before},
      {"/mission/inventory_done", planning::DoubtFix::Done},
      {"/mission/inventory_removed", planning::DoubtFix::Removed}}};
    for (const auto & f : fixes) {
      srv_inventory_.push_back(create_service<Trigger>(
        f.first, [this, name = std::string(f.first), fix = f.second](const std::shared_ptr<Trigger::Request>,
        std::shared_ptr<Trigger::Response> res) {
          if (op_ == Op::RUNNING) {
            res->success = false;
            res->message = "RUNNING 중에는 안 됨";
          } else {
            res->success = exec_->resolve_inventory(fix, res->message);
          }
          RCLCPP_INFO(get_logger(), "%s: %s", name.c_str(), res->message.c_str());
        }));
    }
    // 신호등 기록 지우고 다시 기다림 (운영자가 시퀀스를 다시 보여 줄 때). 로봇이 움직이는 중에는 안 됨
    srv_reset_signal_ = create_service<Trigger>(
      "/mission/reset_signal", [this](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> res) {
        if (op_ == Op::RUNNING && !at_start_zone()) {
          res->success = false;
          res->message = "출발 구역 단계(HOME · 첫 READ_BARCODE · OBSERVE_SIGNAL)에서만 됨";
          return;
        }
        signal_.reset();
        signal_timed_out_ = false;
        if (phase_ == Phase::OBSERVE_SIGNAL) {observe_start_s_ = now_s();}
        publish_signal();
        res->success = true;
        res->message = "신호등 기록 지움 → 다시 기다림";
      });

    // ---------------------------------------------------------------- 주기
    tick_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(1.0, tick_hz_)), [this]() {tick();});
    status_timer_ = create_wall_timer(std::chrono::milliseconds(500), [this]() {publish_status();});
    hold_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
        publish_hold();
        publish_signal();
      });

    if (!map_ok_) {
      fault("MAP_INVALID", "경기장 파일 문제: " + err);
      return;
    }
    RCLCPP_INFO(get_logger(), "STANDBY: 준비 확인 중, /mission/start 기다림%s", auto_start_ ? " (auto_start)" : "");
    RCLCPP_INFO(get_logger(), "신호등: 점멸 기록 %s", signal_record_ ? "켬" : "끔");
  }

private:
  double now_s() {return get_clock()->now().seconds();}

  // ================================================================ 주기 처리 (tick_hz)

  void tick()
  {
    const double now = now_s();
    sync_barcode_reader(now);
    exec_->update(now);
    if (last_light_s_ < 0.0 || now - last_light_s_ > signal_timeout_s_) {
      record_signal(now, SignalObs::NotSeen);   // 토픽 끊김 = 신호등 못 봄
    }
    switch (op_) {
      case Op::STANDBY: tick_standby(now); return;
      case Op::RUNNING: tick_running(now); return;
      case Op::DONE:
      case Op::FAULT:
        return;
    }
  }

  // ---------------------------------------------------------------- STANDBY: 무동작, 준비 확인 · 출발 대기
  void tick_standby(double now)
  {
    not_ready_ = exec_->ready_reasons(now);
    if (auto_start_ && not_ready_.empty()) {
      std::string why;
      start_mission(now, why);
    }
  }

  // /mission/start: STANDBY + 준비 완료일 때만 RUNNING. 경기 시간은 첫 start 부터 (복구 뒤에도 유지)
  bool start_mission(double now, std::string & why)
  {
    if (op_ != Op::STANDBY) {
      why = std::string("STANDBY 아님: ") + op_name(op_);
      return false;
    }
    const auto r = exec_->ready_reasons(now);
    if (!r.empty()) {
      why = "준비 안 됨: " + join(r);
      return false;
    }
    if (start_s_ < 0.0) {
      start_s_ = now;
      end_s_ = now;
    }
    exec_->reset_monitors();
    detail_.clear();
    op_ = Op::RUNNING;
    RCLCPP_INFO(get_logger(), "미션 시작 (start 승인%s)", resume_return_ ? ", 복귀 재개" : "");
    begin_running();
    why = "시작함";
    return true;
  }

  // /mission/resume: FAULT → STANDBY (사람이 로봇 · 재고 확인 뒤). 다시 start 해야 움직임
  bool resume_mission(std::string & why)
  {
    if (op_ != Op::FAULT) {
      why = std::string("FAULT 아님: ") + op_name(op_);
      return false;
    }
    if (!map_ok_) {
      why = "경기장 파일 문제는 노드 재시작 필요";
      return false;
    }
    const std::string pending = exec_->cancel_pending_reason();
    if (!pending.empty()) {
      why = pending;
      return false;
    }
    // 재고 불확실: HOME 성공 · resume 으로는 안 지움, 사람이 실물 확인 뒤 /mission/inventory_* 로 먼저 정합
    if (const auto doubt = exec_->doubt_text(); !doubt.empty()) {
      why = "재고 불확실: " + doubt + " → 실물 확인 뒤 /mission/inventory_before · done · removed 중 하나";
      return false;
    }
    op_ = Op::STANDBY;
    fault_code_.clear();
    detail_ = "복구: 준비 · 재고 확인 뒤 /mission/start";
    RCLCPP_WARN(get_logger(), "FAULT → STANDBY (재고 확인 필요)");
    publish_hold();
    why = "STANDBY 로 전환";
    return true;
  }

  // ---------------------------------------------------------------- RUNNING
  void tick_running(double now)
  {
    // ① 제어 끊김 · 감시 회복 실패 → FAULT
    if (const auto f = exec_->supervise(now)) {
      fault(f->code, f->detail);
      return;
    }
    // ② 시간 부족 → 복귀 (팔 작업 중이면 끝난 뒤, HOME 단계면 HOME 끝난 뒤)
    if ((phase_ == Phase::READ_BARCODE || task_phase()) && !exec_->arm_busy() && time_short(now)) {
      start_return("시간 부족");
      return;
    }
    // ③ 신호등 점멸 기록 (출발 구역에 선 채, 단계 목록 없음)
    if (phase_ == Phase::OBSERVE_SIGNAL && !exec_->waiting_drive()) {
      // 기록 중이면 signal_wait_s 를 넘어도 끝까지 (12 점멸 24 s + 끝 3 s 를 넉넉히 40 s 더). 기다리다 늦게 시작한 신호를 지우지 않게 (11-08)
      const double waited = now - observe_start_s_;
      const bool recording = signal_.status() == SignalStatus::Recording;
      if (signal_.finished() || (waited > signal_wait_s_ && (!recording || waited > signal_wait_s_ + 40.0))) {end_observe(now);}
      return;
    }
    // ④ 단계 실행 → 사건 처리 (단계 목록은 여기서만 바꿈)
    const ExecEvent ev = exec_->step(now);
    if (pending_fault_) {   // Custom 단계 안에서 정한 FAULT (미등록 바코드 · 계획 불가)
      const auto f = *pending_fault_;
      pending_fault_.reset();
      fault(f.first, f.second);
      return;
    }
    switch (ev.kind) {
      case ExecEvent::Kind::None:
        return;
      case ExecEvent::Kind::Fault:
        fault(ev.code, ev.detail);
        return;
      case ExecEvent::Kind::DriveReady:
        after_home(now);
        return;
      case ExecEvent::Kind::StepOk:
        on_step_ok(ev);
        if (ev.all_done) {on_steps_done();}
        break;
      case ExecEvent::Kind::StepFailed:
        detail_ = ev.detail;
        on_step_failed(ev.step, ev.need_home);
        break;
    }
    if (pending_fault_) {   // 재계획 중 정한 FAULT
      const auto f = *pending_fault_;
      pending_fault_.reset();
      fault(f.first, f.second);
    }
  }

  bool time_short(double now) const
  {
    return time_limit_s_ > 0.0 && start_s_ >= 0.0 && now - start_s_ > time_limit_s_ - return_margin_s_;
  }

  // ================================================================ 상태 전환

  void set_steps(Phase p, std::vector<Step> steps)
  {
    phase_ = p;
    barcode_want_ = p == Phase::READ_BARCODE;   // 바코드 판독은 READ_BARCODE 동안만
    // 정차 중 PICK 은 LOOK 때 본 위치를 오래 씀 (차체가 서 있음)
    exec_->set_pick_fresh_s((p == Phase::COLLECT || p == Phase::DELIVER) ? dice_pick_fresh_s_ : -1.0);
    exec_->set_steps(std::move(steps));
  }

  static Step home_step(const std::string & name)
  {
    Step home;
    home.type = StepType::Arm;
    home.arm_task = kArmHome;
    home.name = name;
    return home;
  }

  // RUNNING 진입: HOME 부터 (출발 전에는 움직이지 않음, 복구 뒤에도 팔 자세 확인부터)
  void begin_running()
  {
    queued_task_.reset();   // 복구 뒤에는 바코드부터 다시
    set_steps(Phase::HOME, {home_step("arm:HOME")});
  }

  // HOME + DRIVE 뒤 갈 곳: 시간 부족 → 복귀 / 복귀 중 FAULT 였음 → 복귀 다시 / 아니면 다음 바코드 (점멸 관측은 바코드 뒤)
  void after_home(double now)
  {
    if (time_short(now)) {
      start_return("시간 부족");
    } else if (resume_return_) {
      start_return("복귀 재개");
    } else {
      begin_read_next();
    }
    resume_return_ = false;
  }

  // 출발 구역에 선 채 신호등 점멸 기록 (STANDBY · HOME · READ_BARCODE 중에 이미 시작했으면 이어서)
  void begin_observe(double now)
  {
    observe_start_s_ = now;
    detail_ = "신호등 점멸 기록 중";
    RCLCPP_INFO(get_logger(), "OBSERVE_SIGNAL: 신호등 점멸 기록 (지금 %s, 점멸 %zu 개)",
      signal_status_name(signal_.status()), signal_.blinks().size());
    set_steps(Phase::OBSERVE_SIGNAL, {});
  }

  // 기록 끝 (또는 시간 초과) → 결과 남기고 읽어 둔 task 실행. 불완전해도 색을 추정해 채우지 않음
  void end_observe(double now)
  {
    if (!signal_.finished() && retry_signal(now, "signal_wait_s 안에 안 끝남")) {return;}
    if (!signal_.finished()) {
      RCLCPP_WARN(get_logger(), "신호등 기록 %.0f s 안에 안 끝남 (%s, %s) → 불완전으로 진행",
        now - observe_start_s_, signal_status_name(signal_.status()), signal_.sequence().c_str());
      signal_timed_out_ = true;
      write_signal_record();
      log_signal();
    }
    publish_signal();
    if (queued_task_) {
      auto task = *queued_task_;
      queued_task_.reset();
      start_execute(task);
    } else {
      begin_read_next();
    }
  }

  // 출발 구역 안 단계인가 (신호등 기록 · 기록 지우기 허용). task 실행을 시작하면 출발 구역을 떠난 것으로 봄
  bool at_start_zone() const
  {
    return phase_ == Phase::HOME || phase_ == Phase::OBSERVE_SIGNAL || (phase_ == Phase::READ_BARCODE && !left_start_);
  }

  // 읽은 task 실행 시작: 신호 기록 확인 → COLLECT
  void start_execute(const Task & task)
  {
    ++tasks_total_;
    pucks_total_ += 12;
    current_task_ = task;
    exec_->set_task_id(task.task_id);
    // 발제: 불명 색 추정 금지 → 기록이 완전하고 횟수 규칙(각 1–6, 합 12)에 맞아야 진행
    dice_counts_ = {signal_.count('R'), signal_.count('Y'), signal_.count('G')};
    std::string err;
    if (signal_.status() != SignalStatus::Done) {
      err = std::string("신호 기록 ") + signal_status_name(signal_.status()) + " (" + signal_.sequence() + ")";
    } else {
      valid_signal_counts(dice_counts_, DiceConfig{}, &err);
    }
    if (!err.empty()) {
      RCLCPP_ERROR(get_logger(), "dice 시작 못 함: %s", err.c_str());
      abandon_dice("신호 기록 문제: " + err);
      return;
    }
    auto steps = collect_steps(dice_map_, collect_order_, &err);
    if (steps.empty()) {
      RCLCPP_ERROR(get_logger(), "수집 단계 못 만듦: %s", err.c_str());
      abandon_dice("collect_order 문제: " + err);
      return;
    }
    delivery_steps_.clear();
    verify_after_.clear();
    verify_places_.clear();
    verify_log_file_.clear();
    // 장소 그룹 · 눈 수 · 눈 위치는 신호 · 바코드로 바로 정해짐 (퍽 할당은 마지막 박스에서 운반 계획기가 함)
    dice_plan_ = dice_layout(dice_counts_, task.dice_order);
    if (pip_frame_ == "viewer") {
      apply_view_directions(*dice_plan_, place_view_dirs(dice_map_));
    } else {   // 발제 p.4: 지도 위쪽 +Y, 모든 장소 같은 방향 (2 · 3 대각선 · 6 두 열 그림대로, 회전 대체 불가). 놓는 순서만 보는 방향
      const double ay = dice_map_.map_from_arena.yaw;
      const std::array<double, 2> up{{-std::sin(ay), std::cos(ay)}};   // 경기장 +Y (map)
      apply_place_directions(*dice_plan_, {{up, up, up}}, place_view_dirs(dice_map_));
    }
    placed_count_ = {0, 0, 0};
    look_started_.fill(0.0);
    look_place_started_.fill(0.0);
    for (auto & f : place_frame_) {f.reset();}
    place_dead_ = {false, false, false};
    yellow_used_shape_.clear();
    replans_ = 0;
    current_node_ = kNodeStart;
    alloc_revision_ = 0;
    alloc_history_.clear();
    alloc_reason_.clear();
    left_start_ = true;
    detail_.clear();   // OBSERVE_SIGNAL 의 "기록 중" 등을 지움
    RCLCPP_INFO(get_logger(), "task 시작: %s (%s, R %d · Y %d · G %d)", task.raw_text.c_str(),
      dice_order_name(task.dice_order), dice_counts_.red, dice_counts_.yellow, dice_counts_.green);
    set_steps(Phase::COLLECT, std::move(steps));
  }

  // task 를 실행하는 단계인가
  bool task_phase() const
  {
    return phase_ == Phase::COLLECT || phase_ == Phase::DELIVER || phase_ == Phase::VERIFY;
  }

  // dice 를 더 못 함 → 진행 중 task 실패로 정리하고 복귀 (이미 놓은 퍽은 그대로 = 점수 보존)
  void abandon_dice(const std::string & why)
  {
    if (!current_task_ || pending_fault_) {return;}   // FAULT 로 갈 때는 복귀 안 함
    detail_ = why;
    if (task_phase()) {
      abandon_task(why.c_str());
    } else {   // COLLECT 들어가기 전 (신호 기록 문제)
      ++tasks_failed_;
      done_ids_.insert(current_task_->task_id);
      current_task_.reset();
      ++bar_i_;
    }
    start_return(why);
  }

  // 다음 바코드 읽기. 이미 읽었으면 복귀 (바코드는 출발 구역에 하나)
  void begin_read_next()
  {
    if (bar_i_ >= 1) {
      start_return("바코드 처리 끝");
      return;
    }
    set_steps(Phase::READ_BARCODE, build_read_steps(dice_map_.barcode_view, read_timeout_s_, barcode_look_));
  }

  // 진행 중이던 task 를 실패로 정리 (정확히 한 번: bar_i_ 도 여기서만 넘김)
  void abandon_task(const char * why)
  {
    if (!task_phase() || !current_task_) {return;}
    RCLCPP_WARN(get_logger(), "task 포기 (%s): %s", why, current_task_->raw_text.c_str());
    ++tasks_failed_;
    done_ids_.insert(current_task_->task_id);
    current_task_.reset();
    ++bar_i_;
  }

  void start_return(const std::string & why)
  {
    RCLCPP_INFO(get_logger(), "출발점 복귀: %s", why.c_str());
    abandon_task("복귀");
    exec_->cancel_nav();
    detail_ = why;
    set_steps(Phase::RETURN_HOME, build_return_steps(dice_map_.start));
    publish_hold();
  }

  void finish(const std::string & result)
  {
    op_ = Op::DONE;
    barcode_want_ = false;
    result_ = result;
    end_s_ = now_s();
    exec_->cancel_nav();
    publish_hold();
    RCLCPP_INFO(get_logger(), "미션 종료 (%s): task 완료 %d / 실패 %d / 시도 %d, 퍽 %d / %d",
      result.c_str(), tasks_done_, tasks_failed_, tasks_total_, pucks_done_, pucks_total_);
  }

  // 단계 성공 → 지금 노드 (재계획 시작 위치) · 놓은 수
  void on_step_ok(const ExecEvent & ev)
  {
    const Step & s = ev.step;
    if (s.type == StepType::Navigate) {
      if (box_of(s) >= 1) {current_node_ = box_of(s);}
      if (place_of(s) >= 0) {current_node_ = place_node(place_of(s));}
    }
    if (s.type == StepType::Arm && s.arm_task == kArmPlace) {
      ++pucks_done_;
      if (is_dice_cell(s) && place_of(s) >= 0 && place_of(s) < kNumPlaces) {
        ++placed_count_[place_of(s)];
        if (ev.placed && dice_plan_ && dice_plan_->places[place_of(s)].group == 'Y') {
          yellow_used_shape_ = ev.placed->shape;
        }
      }
    }
  }

  void on_steps_done()
  {
    switch (phase_) {
      case Phase::HOME:
        exec_->wait_drive(now_s());
        break;
      case Phase::READ_BARCODE:
        // ReadBarcode 가 pending_task_ 를 채운 상태. 발제 순서: 바코드 → 점멸 관측 → 이동
        if (signal_record_ && !signal_.finished() && !signal_timed_out_) {
          queued_task_ = pending_task_;
          begin_observe(now_s());
          break;
        }
        start_execute(*pending_task_);
        break;
      case Phase::COLLECT:
        // 마지막 박스 Plan(Delivery) 가 delivery_steps_ 를 채움. 비었으면 (채울 장소 없음) 재검증으로
        if (delivery_steps_.empty()) {
          RCLCPP_WARN(get_logger(), "DELIVER: 놓을 퍽 없음 → 재검증");
          if (!verify_after_.empty()) {
            set_steps(Phase::VERIFY, std::move(verify_after_));
            verify_after_.clear();
          } else {
            set_steps(Phase::VERIFY, {});
            on_steps_done();
          }
          break;
        }
        RCLCPP_INFO(get_logger(), "DELIVER: 단계 %zu 개", delivery_steps_.size());
        set_steps(Phase::DELIVER, std::move(delivery_steps_));
        delivery_steps_.clear();
        break;
      case Phase::DELIVER:
        if (!verify_after_.empty()) {
          RCLCPP_INFO(get_logger(), "VERIFY: 세 장소 재검증");
          set_steps(Phase::VERIFY, std::move(verify_after_));
          verify_after_.clear();
          break;
        }
        [[fallthrough]];
      case Phase::VERIFY:
        ++tasks_done_;
        done_ids_.insert(current_task_->task_id);
        RCLCPP_INFO(get_logger(), "dice 완료: %s (퍽 %d / %d)", current_task_->raw_text.c_str(), pucks_done_, pucks_total_);
        current_task_.reset();
        ++bar_i_;
        begin_read_next();
        break;
      case Phase::RETURN_HOME:
        finish("COMPLETED");
        break;
      case Phase::OBSERVE_SIGNAL:   // 단계 목록 없음 (tick_running 이 끝을 판단)
        break;
    }
  }

  // 재시도까지 실패한 단계 → 단계별 처리. 팔이 움직이다 실패했으면 다음 단계 앞에 HOME
  void on_step_failed(const Step & s, bool need_home)
  {
    switch (phase_) {
      case Phase::READ_BARCODE:
        // 안 읽힘 → 같은 자리에서 다시 읽기 (read_retries 번, 11-08). 이미 처리한 바코드 · 팔 · 이동 실패는 예전처럼 끝
        if (detail_.find("바코드 안 읽힘") != std::string::npos && read_tries_ < read_retries_ && op_ == Op::RUNNING) {
          ++read_tries_;
          RCLCPP_WARN(get_logger(), "바코드 안 읽힘 → 그 자리 그 팔 자세로 다시 읽기 %d / %d", read_tries_, read_retries_);
          auto again = build_read_steps(dice_map_.barcode_view, read_timeout_s_, barcode_look_);
          while (!again.empty() && again.front().kind != kReadBarcode) {again.erase(again.begin());}   // 이동 · LOOK 빼고 읽기 (+ HOME)
          set_steps(Phase::READ_BARCODE, std::move(again));
          break;
        }
        ++bar_i_;
        begin_read_next();
        // LOOK_BARCODE 로 팔을 들었을 수 있음 → 다음 이동 전에 접음 (HOME 은 이미 접혔으면 바로 성공)
        if (barcode_look_ && op_ == Op::RUNNING && !need_home) {exec_->insert_home("바코드 LOOK 뒤");}
        break;
      case Phase::COLLECT:
      case Phase::DELIVER:
      case Phase::VERIFY: {
          const Step failed = s;
          if (!dice_recover(failed)) {
            abandon_dice(std::string(phase_name(phase_)) + " 단계 실패: " + failed.name + " (" + detail_ + ")");
          }
          break;
        }
      case Phase::RETURN_HOME:
        detail_ = "복귀 실패: " + detail_;
        finish("RETURN_FAILED");
        break;
      case Phase::HOME:
      case Phase::OBSERVE_SIGNAL:
        break;
    }
    if (need_home && op_ == Op::RUNNING && !pending_fault_) {exec_->insert_home("팔 작업 실패 뒤");}
  }

  // 모든 목표 취소, 사람 확인 필요. 진행 중이던 task 는 실패로 정리
  //   재진입 (resume → start): HOME 부터. RETURN_HOME 중이었으면 복귀 다시, 아니면 바코드부터
  void fault(const std::string & code, const std::string & why)
  {
    if (op_ == Op::FAULT) {return;}
    RCLCPP_ERROR(get_logger(), "FAULT [%s]: %s", code.c_str(), why.c_str());
    fault_code_ = code;
    detail_ = why;
    if (op_ == Op::RUNNING) {
      abandon_task("FAULT");
      if (phase_ == Phase::RETURN_HOME) {resume_return_ = true;}
    }
    op_ = Op::FAULT;
    barcode_want_ = false;
    exec_->stop();   // 진행 중이던 재고 작업은 여기서 불확실 표시될 수 있음
    if (const auto doubt = exec_->doubt_text(); !doubt.empty() && code != "INVENTORY_UNCERTAIN") {
      detail_ += " / 재고 불확실: " + doubt;
    }
    publish_hold();
  }

  // Custom 단계 안에서 FAULT 를 정함 → step() 이 끝난 뒤 적용 (실행기 안에서 상태를 바꾸지 않음)
  void request_fault(const std::string & code, const std::string & why)
  {
    if (!pending_fault_) {pending_fault_ = std::make_pair(code, why);}
  }

  // ================================================================ Custom 단계 (실행기가 부름)

  StepResult run_custom(const Step & s, double now, double started, std::string & detail)
  {
    switch (s.kind) {
      case kReadBarcode: return run_read(s, now, started, detail);
      case kPlanCollectBox:
      case kPlanDelivery:
      case kPlanVerify:
        return run_plan(s, now, started, detail);
      case kPlaceBox: return run_place_box(s, now, started);
      default:
        detail = "모르는 Custom 단계: " + std::to_string(s.kind);
        return StepResult::Fail;
    }
  }

  // PLACE 목표: 장소 중심 + 주사위 눈 오프셋 (map). 높이 = 놓는 면 + 퍽 반높이 + grasp_offset
  //   이 장소 LOOK 뒤 박스 측정이 있으면 측정 중심 + 박스 회전만큼 돈 오프셋 (place_box.hpp), 없으면 지도 값
  bool dice_cell_target(const Step & s, geometry_msgs::msg::PoseStamped & out, std::string & detail)
  {
    const int p = place_of(s), k = cell_of(s);
    const auto pt = (is_dice_cell(s) && dice_plan_) ? dice_cell_point(*dice_plan_, dice_map_, p, k) : std::nullopt;
    if (!pt) {
      detail = "dice 칸 없음: L" + std::to_string(p + 1) + " #" + std::to_string(k);
      return false;
    }
    std::array<double, 2> xy = *pt;
    if (const auto & f = place_frame_[p]) {
      xy = frame_point(*f, dice_plan_->places[p].pip_offsets[k]);
      RCLCPP_INFO(get_logger(), "L%d #%d: 박스 측정 기준 (지도 대비 %.1f mm 이동 · %.1f° 회전) → 칸 (%.3f, %.3f)", p + 1, k,
        1000.0 * std::hypot((*pt)[0] - xy[0], (*pt)[1] - xy[1]), f->dyaw * 180.0 / M_PI, xy[0], xy[1]);
    }
    {   // 장소별 놓는 점 보정 (place_offset_L*_mm, 로봇 기준 [앞, 왼쪽]) → 이 장소 보는 방향으로 map 에 더함
      const std::string name = "place_offset_L" + std::to_string(p + 1) + "_mm";
      const auto off = get_parameter(name).as_double_array();
      if (off.size() != 2 || !std::isfinite(off[0]) || !std::isfinite(off[1])) {
        detail = name + " 는 [앞, 왼쪽] mm 두 개";
        return false;
      }
      if (off[0] != 0.0 || off[1] != 0.0) {
        const auto d = place_offset_map(dice_map_.places[p].view_dir, {off[0], off[1]});
        xy = {xy[0] + d[0], xy[1] + d[1]};
        RCLCPP_INFO(get_logger(), "L%d #%d: 놓는 점 보정 앞 %+.1f · 왼쪽 %+.1f mm (%s) → (%.3f, %.3f)", p + 1, k, off[0], off[1], name.c_str(), xy[0], xy[1]);
      }
    }
    out.header.frame_id = exec_->map_frame();
    out.pose.position.x = xy[0];
    out.pose.position.y = xy[1];
    out.pose.position.z = dice_map_.places[p].top_z + exec_->puck_half_height_m() + exec_->grasp_offset_m();
    return true;
  }

  // 장소 LOOK 뒤: 팔이 선 뒤 (started) 측정이 box_min_samples 개 모이면 그 장소 칸 기준 확정, box_wait_s 넘으면 지도 값
  //   확정한 기준은 다음 LOOK 전까지 그 장소 PLACE 에 씀 (놓는 중 측정은 팔 · 퍽이 가려 안 씀)
  StepResult run_place_box(const Step & s, double now, double started)
  {
    const int p = place_of(s);
    if (p < 0 || p >= kNumPlaces) {return StepResult::Success;}
    if (const auto f = measured_place(p, started, now - started >= box_wait_s_)) {
      place_frame_[p] = f;
      const auto & c = dice_map_.places[p].center;
      RCLCPP_INFO(get_logger(), "L%d 박스 측정 %d 개: 지도 중심 대비 (%+.1f, %+.1f) mm · %+.1f° → 이 기준으로 놓음", p + 1, f->n,
        1000.0 * (f->x - c[0]), 1000.0 * (f->y - c[1]), f->dyaw * 180.0 / M_PI);
      return StepResult::Success;
    }
    if (now - started < box_wait_s_) {return StepResult::Running;}
    place_frame_[p].reset();
    RCLCPP_WARN(get_logger(), "L%d 박스 측정 %.1f s 안에 못 받음 → 지도 중심으로 놓음", p + 1, box_wait_s_);
    return StepResult::Success;
  }

  // 박스 측정 (perception box.frame) → map 으로 모음. 정차 중 측정이라 최신 TF 로 바꿔도 됨
  void on_box(const interfaces::msg::DetectionArray & msg)
  {
    for (const auto & d : msg.detections) {
      if (d.kind != "place_box") {continue;}
      geometry_msgs::msg::PoseStamped in, out;
      in.header = d.header.frame_id.empty() ? msg.header : d.header;
      in.pose = d.pose;
      std::string why;
      if (!exec_->transform_pose(in, exec_->map_frame(), out, &why)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "박스 측정 지도 변환 실패: %s", why.c_str());
        continue;
      }
      const auto & q = out.pose.orientation;
      BoxSample b;
      b.stamp = rclcpp::Time(in.header.stamp).seconds();
      b.x = out.pose.position.x;
      b.y = out.pose.position.y;
      b.yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
      box_samples_.push_back(b);
      while (box_samples_.size() > 300) {box_samples_.pop_front();}
    }
  }

  // 장소 place 박스 기준 (since 뒤 측정 중앙값). 끔 · LOOK 전 · 측정 부족이면 nullopt (지도 값 사용)
  std::optional<PlaceFrame> measured_place(int place, double since, bool warn = true) const
  {
    if (!box_measure_ || place < 0 || place >= kNumPlaces || since <= 0.0) {return std::nullopt;}
    std::string why;
    const std::vector<BoxSample> v(box_samples_.begin(), box_samples_.end());
    auto f = place_frame_from(v, since, dice_map_.places[place].center, dice_map_.map_from_arena.yaw, box_max_shift_m_,
      box_min_samples_, &why);
    if (!f && warn) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "L%d 박스 측정 못 씀 (%s) → 지도 중심 사용", place + 1, why.c_str());
    }
    return f;
  }

  // ---------------------------------------------------------------- ReadBarcode
  //   단계 시작 뒤에 들어온 바코드만 봄 (perception 이 5 장 중 3 장 같아야 보내므로 한 장 오독은 안 옴)
  //   미등록 바코드 = 발제상 오류 → 바로 FAULT (기다리거나 넘어가지 않음)
  //   read_timeout_s 안에 아무것도 안 오면 단계 실패
  StepResult run_read(const Step & s, double now, double started, std::string & detail)
  {
    if (last_text_s_ > started) {
      std::string err;
      auto task = parse_task(last_text_, barcodes_, &err);
      if (task) {
        if (done_ids_.count(task->task_id)) {
          detail = "이미 처리한 바코드: " + task->raw_text;
          return StepResult::Fail;
        }
        pending_task_ = task;
        return StepResult::Success;
      }
      request_fault("BARCODE_UNREGISTERED", err + " (원문: " + last_text_ + ")");
      return StepResult::Running;   // FAULT 로 멈춤 (단계 결과 처리 안 함)
    }
    if (now - started > (s.timeout_s > 0.0 ? s.timeout_s : read_timeout_s_)) {
      detail = "바코드 안 읽힘";
      return StepResult::Fail;
    }
    return StepResult::Running;
  }

  // ---------------------------------------------------------------- Plan
  //   LOOK 뒤 look_dwell_s 동안 검출을 모은 다음
  //   CollectBox: 이 박스에서 본 퍽 → PICK → STORE … 을 바로 뒤에 끼워 넣음 (마지막 퍽도 넣음, 퍽 쥔 채 주행 안 함)
  //   Delivery  : 실은 퍽 + 이 박스 퍽으로 할당 · 운반 계획 → 배달 · 재검증 단계
  //   Verify    : 장소 재검증 판정 · 기록
  StepResult run_plan(const Step & s, double now, double started, std::string & detail)
  {
    if (now - started < look_dwell_s_) {return StepResult::Running;}
    if (s.kind == kPlanVerify) {
      verify_place(place_of(s), started, now);
      return StepResult::Success;   // 판정이 나빠도 미션은 계속 (기록이 목적)
    }
    if (s.kind == kPlanDelivery && box_of(s) < 1) {   // 재계획 (마지막 박스 이동 실패 등): 지금 자리에서
      return plan_from_state(false, now) ? StepResult::Success : StepResult::Fail;
    }
    const int box = box_of(s);
    if (box < 1 || box > kNumBoxes) {
      detail = "Plan 박스 번호 잘못됨";
      return StepResult::Fail;
    }
    // 그 박스 LOOK 을 시작한 뒤 본 퍽 (다시 계획할 때도 같은 LOOK 결과를 씀: 복구 HOME 뒤엔 카메라가 박스를 안 봄)
    const double look_t = look_started_[box - 1];
    const auto at_box = pucks_at_box(box, look_t > 0.0 ? look_t - 0.5 : started - 2.0);
    RCLCPP_INFO(get_logger(), "B%d LOOK: 퍽 %zu 개 (%s)", box, at_box.size(), puck_list(at_box).c_str());

    if (s.kind == kPlanCollectBox) {
      std::string err;
      const auto actions = collect_box_actions(at_box, exec_->cargo(), &err);
      if (!err.empty()) {
        detail = "B" + std::to_string(box) + " 집기 계획 실패: " + err;
        return StepResult::Fail;
      }
      exec_->insert_after_current(arm_steps(actions, rack_look_, box_measure_));
      return StepResult::Success;
    }
    return plan_from_state(true, now) ? StepResult::Success : StepResult::Fail;   // kPlanDelivery
  }

  // 박스 중심 box_radius 안, since 뒤에 본 on_field 퍽 (ID · 색 · 모양 다 아는 것)
  std::vector<TransferPuck> pucks_at_box(int box, double since) const
  {
    std::vector<TransferPuck> out;
    const auto & c = dice_map_.boxes[box - 1].center;
    for (const auto & r : exec_->registry().records()) {
      if (r.state != "on_field" || r.last_seen < since) {continue;}
      if (std::hypot(r.x - c[0], r.y - c[1]) > dice_map_.box_radius) {continue;}
      const auto kind = puck_kind(r.color, r.shape);
      if (!kind) {   // 그 퍽만 뺌 (박스 · 미션 전체를 버리지 않음). 할당 기록에 박스 위로 남음
        RCLCPP_WARN(get_logger(), "B%d 퍽 %s 색 · 모양 모름 (%s/%s) → 이번엔 안 집음", box, r.id.c_str(),
          r.color.c_str(), r.shape.c_str());
        continue;
      }
      TransferPuck p;
      p.id = r.id;
      p.kind = *kind;
      p.box = box;
      out.push_back(p);
    }
    sort_near_first(out);
    return out;
  }

  // 같은 박스 안에서 정차 자리에 가까운 퍽부터 = 집는 순서 (먼 퍽부터 집으면 팔이 앞 퍽을 칠 수 있음)
  //   박스 번호 순 → 정차 자리까지 거리 순 → 같으면 ID 순
  void sort_near_first(std::vector<TransferPuck> & v) const
  {
    std::map<std::string, double> dist;
    for (const auto & r : exec_->registry().records()) {
      for (const auto & p : v) {
        if (p.id != r.id || p.box < 1 || p.box > kNumBoxes) {continue;}
        const auto & st = dice_map_.boxes[p.box - 1].stop;
        dist[p.id] = std::hypot(r.x - st.x, r.y - st.y);
      }
    }
    std::sort(v.begin(), v.end(), [&](const TransferPuck & a, const TransferPuck & b) {
        if (a.box != b.box) {return a.box < b.box;}
        const double da = dist.count(a.id) ? dist.at(a.id) : 0.0, db = dist.count(b.id) ? dist.at(b.id) : 0.0;
        if (std::abs(da - db) > 1e-6) {return da < db;}
        return a.id < b.id;
      });
  }

  static std::string puck_list(const std::vector<TransferPuck> & v)
  {
    std::string out;
    for (const auto & p : v) {out += (out.empty() ? "" : ", ") + p.id + ":" + kind_color(p.kind) + "/" + kind_shape(p.kind);}
    return out.empty() ? "없음" : out;
  }

  // 기록부의 박스 위 퍽 (ID · 색 · 모양 아는 on_field, 박스 반경 안) → 박스 번호
  std::vector<TransferPuck> pucks_on_boxes() const
  {
    std::vector<TransferPuck> out;
    for (const auto & r : exec_->registry().records()) {
      if (r.state != "on_field" || r.id.empty()) {continue;}
      const auto kind = puck_kind(r.color, r.shape);
      if (!kind) {continue;}
      for (int b = 1; b <= kNumBoxes; ++b) {
        const auto & c = dice_map_.boxes[b - 1].center;
        if (std::hypot(r.x - c[0], r.y - c[1]) <= dice_map_.box_radius) {
          out.push_back(TransferPuck{r.id, *kind, b});
          break;
        }
      }
    }
    sort_near_first(out);   // 운반 계획기는 박스마다 이 순서대로만 집음
    return out;
  }

  // 지금 실제 상태 (기록부 박스 위 퍽 · 적재함 · 장소별 놓은 수) 로 할당 · 운반 계획 → delivery_steps_ · verify_after_
  //   - 처음 (마지막 박스 LOOK 뒤) 과 재계획 모두 이 함수
  //   - 퍽이 모자라면 (못 집음 · 잃음) 끝까지 채울 수 있는 장소를 가장 많이 고름 (같은 수면 시간 짧은 쪽). 장소마다 점수가 따로라서
  //   - stopped = 지금 노드에 서서 LOOK 까지 끝났는가 (false 면 첫 동작 전에 다시 이동 · LOOK)
  //   - 채울 장소가 하나도 없으면 배달 단계 없이 (재검증 · 복귀로)
  bool plan_from_state(bool stopped, double now)
  {
    std::string err;
    SolverInput base;
    base.position = current_node_;
    base.stopped = stopped;
    base.allow_leftover = true;
    if (!fill_carried(exec_->cargo(), base, &err)) {
      request_fault("DICE_UNSOLVABLE", "실은 퍽 정보 문제: " + err);
      return false;
    }
    base.on_boxes = pucks_on_boxes();
    base.yellow_shape = yellow_used_shape_;
    std::array<int, kNumPlaces> need{};
    for (int i = 0; i < kNumPlaces; ++i) {
      base.place_group[i] = dice_plan_->places[i].group;
      need[i] = place_dead_[i] ? 0 : std::max(0, dice_plan_->places[i].pips - placed_count_[i]);
    }
    const int avail = static_cast<int>(base.on_boxes.size() + base.in_cargo.size()) + (base.gripper ? 1 : 0);

    // 채울 장소 조합: 많은 순 → 같은 수면 비용 작은 쪽
    std::optional<TransferPlan> best;
    int best_n = -1, best_mask = 0;
    for (int n = kNumPlaces; n >= 1 && !best; --n) {
      for (int mask = 1; mask < 8; ++mask) {
        int cnt = 0, slots = 0;
        SolverInput in = base;
        for (int i = 0; i < kNumPlaces; ++i) {
          const bool on = (mask >> i & 1) && need[i] > 0;
          if ((mask >> i & 1) && need[i] == 0) {cnt = -99;}   // 이미 끝났거나 버린 장소는 조합에서 뺌
          in.slots_left[i] = on ? need[i] : 0;
          cnt += on ? 1 : 0;
          slots += in.slots_left[i];
        }
        if (cnt != n || slots > avail) {continue;}
        std::string e2;
        auto tp = plan_transfer(in, costs_, exec_->cargo(), placed_count_, &e2, transfer_max_remaining_);
        if (tp && (!best || tp->cost_s < best->cost_s)) {best = std::move(tp); best_n = n; best_mask = mask;}
        if (!tp && err.empty()) {err = e2;}
      }
    }
    std::string chosen;
    for (int i = 0; i < kNumPlaces; ++i) {
      if (best_mask >> i & 1) {chosen += (chosen.empty() ? "L" : " · L") + std::to_string(i + 1);}
    }
    if (!best) {
      RCLCPP_WARN(get_logger(), "배달 계획: 채울 수 있는 장소 없음 (남은 퍽 %d, %s)", avail, err.c_str());
      delivery_steps_.clear();
    } else {
      delivery_steps_ = transfer_steps(*best, dice_map_, rack_look_, box_measure_);
      RCLCPP_INFO(get_logger(), "배달 계획: %s (%d 곳), 남은 퍽 %d, Y 모양 %s, 예상 %.0f s, 정차 %zu",
        chosen.c_str(), best_n, avail, best->yellow_shape.c_str(), best->cost_s, best->stops.size());
    }
    std::array<bool, kNumPlaces> filled{};
    for (int i = 0; i < kNumPlaces; ++i) {filled[i] = (best_mask >> i & 1) != 0;}
    // 첫 판 = initial (수집 중 실패가 있었으면 괄호로), 그 뒤 = 재계획 이유
    const std::string reason = alloc_revision_ == 0 ?
      (alloc_reason_.empty() ? std::string("initial") : "initial (수집 중 " + alloc_reason_.substr(8) + ")") :
      (alloc_reason_.empty() ? std::string("replan") : alloc_reason_);
    write_allocation(best, filled, reason, now);
    alloc_reason_.clear();
    // 재검증 순서: 계획이 있으면 그것, 없으면 L1 → L2 → L3
    const std::array<int, kNumPlaces> vo = best ? best->verify_order : std::array<int, kNumPlaces>{{0, 1, 2}};
    verify_after_ = costs_.verify_tour ? verify_steps(vo, dice_map_) : std::vector<Step>{};
    for (int i = 0; i < kNumPlaces; ++i) {
      RCLCPP_INFO(get_logger(), "  L%d %c%d: 놓음 %d, %s", i + 1, dice_plan_->places[i].group, dice_plan_->places[i].pips,
        placed_count_[i], place_dead_[i] ? "버림" : ((best_mask >> i & 1) ? "채움" : (need[i] == 0 ? "끝" : "못 채움")));
    }
    return true;
  }

  // 단계가 재시도 뒤에도 실패 → 그 퍽 · 장소를 빼고 이어 가기. 못 이어 가면 false (→ 포기 · 복귀)
  //   COLLECT : 이 박스 남은 단계를 지우고 같은 박스 Plan 을 다시 (못 집은 퍽은 skipped → 빠짐)
  //             이동 실패면 그 박스를 건너뜀 (마지막 박스였으면 지금 자리에서 배달 계획)
  //   DELIVER : 지금 상태에서 plan_from_state 로 남은 단계 전부 다시
  //   VERIFY  : 그 장소 재검증만 건너뜀
  //   퍽 영향 : 여기 오는 PICK · STORE · RETRIEVE · PLACE · LOOK_RACK 실패 = 보내기 전 실패 · 목표 거절뿐 (팔 안 움직임)
  //             (수락 뒤 실패는 실행기가 재고 불확실 FAULT 로 보냄 → 여기 안 옴). 그래서 재고 기록은 안 바꿈 (lost 단정 안 함)
  //             PICK 실패 = 그 퍽 박스에 남김 (skipped, 다시 안 집음)
  //             PLACE 실패한 장소는 더 안 건드림 (같은 이유로 다시 실패할 수 있음, 퍽은 그리퍼에 남아 다른 장소에 씀)
  //   실행기 사건 처리 중 (주기 사이) 에만 부름 → 단계 목록을 바꿔도 됨
  bool dice_recover(const Step & f)
  {
    if (phase_ != Phase::VERIFY && ++replans_ > max_replans_) {   // 재검증 건너뛰기는 횟수에 안 셈
      RCLCPP_WARN(get_logger(), "재계획 %d 번 넘음 → 포기", max_replans_);
      return false;
    }
    if (f.type == StepType::Arm) {
      switch (f.arm_task) {
        case kArmPick:
          exec_->mark_puck(f.puck_id, "skipped");
          break;
        case kArmPlace:
          if (place_of(f) >= 0 && place_of(f) < kNumPlaces) {place_dead_[place_of(f)] = true;}
          break;
        default:
          break;   // STORE · RETRIEVE · LOOK_RACK: 재고 그대로 다시 계획. LOOK 실패: 그 자리 퍽을 못 봄 → Plan 이 본 것만으로 진행
      }
    }
    alloc_reason_ = "replan: " + f.name + " 실패";
    RCLCPP_WARN(get_logger(), "재계획 %d/%d: %s 실패 → %s", replans_, max_replans_, f.name.c_str(),
      phase_ == Phase::VERIFY ? "이 장소 재검증 건너뜀" : "그 퍽 · 장소 빼고 다시");

    const auto & st = exec_->steps();
    const std::size_t at = exec_->index();
    // 지금 단계부터 다음 이동 전까지 = 이 정차의 남은 단계 (이동 실패면 그 이동이 연 정차 전체)
    auto seg_end = [&st](std::size_t from) {
        std::size_t e = from;
        if (e < st.size() && st[e].type == StepType::Navigate) {++e;}
        while (e < st.size() && st[e].type != StepType::Navigate) {++e;}
        return e;
      };

    if (phase_ == Phase::COLLECT) {
      const std::size_t e = seg_end(at);
      bool had_delivery = false;
      int box = box_of(f);
      for (std::size_t i = at; box < 1 && i-- > 0;) {   // 이 정차의 LOOK · Plan 에서 박스 번호
        if (box_of(st[i]) >= 1) {box = box_of(st[i]);}
        if (st[i].type == StepType::Navigate) {break;}
      }
      for (std::size_t i = at; i < e; ++i) {
        if (is_custom(st[i], kPlanDelivery)) {had_delivery = true;}
        if (box_of(st[i]) >= 1) {box = box_of(st[i]);}
      }
      exec_->erase_steps(at, e);
      if (is_custom(f, kPlanDelivery)) {return false;}   // 계획 자체가 실패
      Step again;
      again.type = StepType::Custom;
      const bool skip_box = f.type == StepType::Navigate || f.type == StepType::Custom;
      if (skip_box) {
        // 박스 건너뜀. 마지막 박스였으면 지금 자리에서 배달 계획
        if (!had_delivery) {return true;}
        again.kind = kPlanDelivery;
        box_of(again) = -1;
        again.name = "plan:delivery:here";
      } else {
        again.kind = had_delivery ? kPlanDelivery : kPlanCollectBox;
        box_of(again) = box;
        again.name = std::string(had_delivery ? "plan:delivery:B" : "plan:collect:B") + std::to_string(box) + ":again";
      }
      exec_->insert_steps(at, {again});
      return true;
    }
    if (phase_ == Phase::DELIVER) {
      exec_->erase_steps(at, st.size());
      if (!plan_from_state(false, now_s())) {return false;}
      exec_->insert_steps(at, delivery_steps_);
      delivery_steps_.clear();
      if (exec_->done()) {on_steps_done();}   // 더 놓을 게 없음 → 재검증으로
      return true;
    }
    if (phase_ == Phase::VERIFY) {
      exec_->erase_steps(at, seg_end(at));
      if (exec_->done()) {on_steps_done();}
      return true;
    }
    return false;
  }

  // ================================================================ 재검증

  // 재검증 장소 하나: since 뒤 새 관측 중 장소 반경 안 → 판정 → 발행 · 기록
  //   looked = LOOK 성공 뒤 (Plan Verify 단계 시작), 박스 측정은 이 뒤 것만 씀 (팔이 움직이는 중 장 뺌)
  void verify_place(int place, double looked, double now)
  {
    if (!dice_plan_ || place < 0 || place >= kNumPlaces) {
      RCLCPP_WARN(get_logger(), "재검증 L%d: 계획 없음", place + 1);
      return;
    }
    // 퍽 관측: 이 장소 LOOK 을 시작한 뒤 촬영된 것만 (촬영 시각 기준, 기록부의 옛 ID 를 쓰지 않음)
    double since = looked;
    const double look_t = look_place_started_[place];
    if (look_t > 0.0) {since = look_t;}
    // 박스 측정이 있으면 재검증 LOOK 에서 새로 잰 기준으로 판정 (놓은 뒤 박스가 밀렸어도 박스 기준으로 봄), 없으면 지도 값
    DiceMap arena = dice_map_;
    DicePlan plan = *dice_plan_;
    if (const auto f = measured_place(place, looked)) {apply_place_frame(arena, plan, place, *f);}
    const auto & c = arena.places[place].center;
    const auto obs = gather_place_obs(c, since);
    auto pv = judge_place(place, plan, arena, obs, verify_cell_tol_m_, now - start_s_);
    RCLCPP_INFO(get_logger(), "재검증 L%d (%c%d): 본 퍽 %zu 개 → %s", place + 1, pv.group, pv.expected, pv.seen.size(),
      pv.ok ? "맞음" : "문제 있음");
    for (const auto & i : pv.issues) {RCLCPP_WARN(get_logger(), "  재검증: %s", i.c_str());}
    verify_places_.erase(std::remove_if(verify_places_.begin(), verify_places_.end(),
      [place](const PlaceVerify & v) {return v.place == place;}), verify_places_.end());
    verify_places_.push_back(std::move(pv));
    const auto r = summarize(verify_places_);
    write_verify_log(r);   // 장소마다 씀 (세 곳 다 못 봐도 본 곳 ID · 위치가 남게, status PENDING)
    publish_verify(r);
    if (r.complete) {
      RCLCPP_INFO(get_logger(), "재검증 %s (중복 %zu · 누락 %zu)%s%s", r.pass ? "PASS" : "FAIL", r.duplicate_ids.size(),
        r.missing_ids.size(), verify_log_file_.empty() ? "" : " → ", verify_log_file_.c_str());
    }
  }

  // 이 장소 원시 관측 (since 뒤 촬영 · 장소 반경 안) → 퍽 단위로 묶음, 색 · 모양 다수결 (group_place_obs)
  std::vector<VerifyObs> gather_place_obs(const std::array<double, 2> & c, double since) const
  {
    std::vector<VerifyObs> raw;
    for (const ObsRecord & o : exec_->recent_obs()) {
      if (o.stamp < since || std::hypot(o.x - c[0], o.y - c[1]) > verify_radius_m_) {continue;}
      VerifyObs v;
      v.id = o.id;
      v.color = o.color;
      v.shape = o.shape;
      v.x = o.x;
      v.y = o.y;
      raw.push_back(v);
    }
    return group_place_obs(raw, 0.03, verify_vote_ratio_);
  }

  void publish_verify(const VerifyResult & r)
  {
    VerifyRecord m;
    m.header.stamp = get_clock()->now();
    m.header.frame_id = exec_->map_frame();
    m.status = r.complete ? (r.pass ? VerifyRecord::PASS : VerifyRecord::FAIL) : VerifyRecord::PENDING;
    m.complete = r.complete;
    for (const auto & p : r.places) {
      const std::string name = "L" + std::to_string(p.place + 1);
      m.places.push_back(name);
      m.groups.push_back(std::string(1, p.group));
      m.expected.push_back(static_cast<uint8_t>(p.expected));
      m.seen.push_back(static_cast<uint8_t>(p.seen.size()));
      m.place_ok.push_back(p.ok);
      for (const auto & o : p.seen) {
        VerifyPuck vp;
        vp.place = name;
        vp.id = o.id;
        vp.color = o.color;
        vp.shape = o.shape;
        vp.x = o.x;
        vp.y = o.y;
        vp.cell = static_cast<int8_t>(o.cell);
        vp.cell_error_m = static_cast<float>(std::min(o.cell_error_m, 9.999));
        vp.ok = o.ok;
        m.pucks.push_back(vp);
      }
    }
    m.duplicate_ids = r.duplicate_ids;
    m.missing_ids = r.missing_ids;
    m.issues = r.issues;
    m.log_file = verify_log_file_;
    pub_verify_->publish(m);
  }

  // ================================================================ 기록 파일

  // 이번 실행 기록 폴더 (처음 쓸 때 만듦): <record_dir>/run_<YYYYmmdd_HHMMSS>
  std::string run_dir()
  {
    if (record_dir_.empty()) {return "";}
    if (run_dir_.empty()) {
      std::string dir = record_dir_;
      if (dir.rfind("~/", 0) == 0) {
        const char * home = std::getenv("HOME");
        dir = std::string(home ? home : ".") + dir.substr(1);
      }
      char stamp[32];
      const std::time_t t = std::time(nullptr);
      std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", std::localtime(&t));
      run_dir_ = dir + "/run_" + stamp;
      std::error_code ec;
      std::filesystem::create_directories(run_dir_, ec);
      RCLCPP_INFO(get_logger(), "기록 폴더: %s", run_dir_.c_str());
    }
    return run_dir_;
  }

  // 기록 파일 쓰기 (덮어씀). 성공하면 경로, 실패 · 끔이면 ""
  std::string write_record(const std::string & name, const std::string & text)
  {
    const std::string dir = run_dir();
    if (dir.empty()) {return "";}
    const std::string path = dir + "/" + name;
    std::ofstream f(path);
    if (!f) {
      RCLCPP_WARN(get_logger(), "기록 파일 못 씀: %s", path.c_str());
      return "";
    }
    f << text;
    return path;
  }

  void write_verify_log(const VerifyResult & r)
  {
    if (!dice_plan_) {return;}
    const auto path = write_record("verify.yaml",
      verify_to_yaml(r, *dice_plan_, dice_map_, current_task_ ? current_task_->raw_text : "", signal_.sequence()));
    if (!path.empty()) {verify_log_file_ = path;}
  }

  void write_signal_record()
  {
    const auto path = write_record("signal.json", signal_to_json(signal_, node_start_s_, signal_timed_out_));
    if (!path.empty()) {RCLCPP_INFO(get_logger(), "신호 기록 → %s", path.c_str());}
  }

  // 할당 기록: 계획 (처음 · 재계획) 마다 revision 을 올려 덮어씀. 첫 PLACE 전에 반드시 한 번 씀
  void write_allocation(const std::optional<TransferPlan> & plan, const std::array<bool, kNumPlaces> & filled,
    const std::string & reason, double now)
  {
    if (!dice_plan_ || !current_task_) {return;}
    AllocationRecord r;
    r.barcode = current_task_->raw_text;
    r.order = current_task_->dice_order;
    r.signal_sequence = signal_.sequence();
    r.counts = dice_counts_;
    r.revision = ++alloc_revision_;
    r.reason = reason;
    r.stamp_s = start_s_ >= 0.0 ? now - start_s_ : 0.0;
    r.yellow_shape = plan ? plan->yellow_shape : yellow_used_shape_;
    r.filled = filled;
    r.placed = placed_count_;
    r.cells = plan ? planned_cells(delivery_steps_, *dice_plan_, dice_map_) : std::vector<AllocCell>{};
    r.verify_order = plan ? plan->verify_order : std::array<int, kNumPlaces>{{0, 1, 2}};
    r.cost_s = plan ? plan->cost_s : 0.0;
    r.history = alloc_history_;
    for (const auto & rec : exec_->registry().records()) {
      if (rec.id.empty()) {continue;}
      AllocPuck ap{rec.id, rec.color, rec.shape, rec.state};
      if (rec.state == "on_field" || rec.state == "skipped") {
        for (int b = 1; b <= kNumBoxes; ++b) {
          const auto & c = dice_map_.boxes[b - 1].center;
          if (std::hypot(rec.x - c[0], rec.y - c[1]) <= dice_map_.box_radius) {
            ap.where = (rec.state == "on_field" ? "box:B" : "skipped:B") + std::to_string(b);
          }
        }
      } else if (rec.state.rfind("in_cargo:", 0) == 0) {
        ap.where = "cargo:" + rec.state.substr(9);
      } else if (rec.state == "in_gripper") {
        ap.where = "gripper";
      } else if (rec.state == "delivered") {
        ap.where = "placed";
      }
      r.pucks.push_back(ap);
    }
    std::sort(r.pucks.begin(), r.pucks.end(), [](const AllocPuck & a, const AllocPuck & b) {
        return a.id.size() != b.id.size() ? a.id.size() < b.id.size() : a.id < b.id;
      });
    const auto json = allocation_to_json(r, *dice_plan_, dice_map_);
    const auto path = write_record("allocation.json", json);                                   // 최신 판
    write_record("allocation_rev" + std::to_string(r.revision) + ".json", json);              // 판마다 사본 (첫 할당 증거)
    std::string sum = "rev " + std::to_string(r.revision) + " (" + reason + "): ";
    for (int i = 0; i < kNumPlaces; ++i) {sum += "L" + std::to_string(i + 1) + (filled[i] ? "채움 " : "- ");}
    alloc_history_.push_back(sum);
    if (!path.empty()) {RCLCPP_INFO(get_logger(), "할당 기록 rev %d → %s", r.revision, path.c_str());}
  }

  // ================================================================ 신호등 기록 · 바코드 판독 켜기

  // perception 바코드 판독 켜기 · 끄기를 barcode_want_ 에 맞춤 (응답 없으면 2 s 뒤 버리고 다시)
  void sync_barcode_reader(double now)
  {
    if (!barcode_cli_) {return;}
    if (barcode_req_pending_) {
      if (now - barcode_req_sent_s_ < 2.0) {return;}
      barcode_cli_->prune_pending_requests();   // 응답 없음 → 버리고 다시 (늦은 응답은 gen 으로 무시)
      barcode_req_pending_ = false;
      RCLCPP_WARN(get_logger(), "바코드 판독 서비스 응답 없음 → 다시 보냄");
    }
    if (barcode_known_ && barcode_on_ == barcode_want_) {return;}
    if (now - barcode_last_try_s_ < 1.0) {return;}
    barcode_last_try_s_ = now;
    if (!barcode_cli_->service_is_ready()) {
      if (barcode_want_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "바코드 판독 서비스 %s 없음 (perception 노드 확인)",
          barcode_enable_service_.c_str());
      }
      return;
    }
    auto req = std::make_shared<SetBool::Request>();
    req->data = barcode_want_;
    const bool want = barcode_want_;
    const int gen = ++barcode_req_gen_;
    barcode_req_pending_ = true;
    barcode_req_sent_s_ = now;
    barcode_cli_->async_send_request(req, [this, want, gen](rclcpp::Client<SetBool>::SharedFuture f) {
        if (gen != barcode_req_gen_) {return;}   // 버린 옛 요청의 늦은 응답
        barcode_req_pending_ = false;
        if (f.get()->success) {
          barcode_known_ = true;
          barcode_on_ = want;
          RCLCPP_INFO(get_logger(), "perception 바코드 판독 %s", want ? "켬" : "끔");
        }
      });
  }

  void log_signal()
  {
    const std::string msg = "신호등 기록 " + std::string(signal_status_name(signal_.status())) + ": " + signal_.sequence() +
      " (R " + std::to_string(signal_.count('R')) + " · Y " + std::to_string(signal_.count('Y')) +
      " · G " + std::to_string(signal_.count('G')) + " · ? " + std::to_string(signal_.count('?')) + ")";
    if (signal_.status() == SignalStatus::Done) {
      RCLCPP_INFO(get_logger(), "%s", msg.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "%s", msg.c_str());
    }
    for (const auto & i : signal_.issues()) {RCLCPP_WARN(get_logger(), "  신호등 기록 문제: %s", i.c_str());}
  }

  // perception 신호등 상태 -> 기록기 관측
  static SignalObs signal_obs(uint8_t state, bool visible)
  {
    switch (state) {
      case TrafficLight::RED: return SignalObs::Red;
      case TrafficLight::YELLOW: return SignalObs::Yellow;
      case TrafficLight::GREEN: return SignalObs::Green;
      case TrafficLight::OFF: return visible ? SignalObs::Off : SignalObs::NotSeen;
      default: return visible ? SignalObs::Ambiguous : SignalObs::NotSeen;
    }
  }

  // 로봇이 출발 구역에 있는 동안만 기록 (STANDBY · HOME · 첫 READ_BARCODE · OBSERVE_SIGNAL). 끝난 뒤에는 그대로 둠
  void record_signal(double t, SignalObs o)
  {
    if (!signal_record_ || signal_.finished() || signal_timed_out_) {return;}
    const bool still = op_ == Op::STANDBY || (op_ == Op::RUNNING && at_start_zone());
    if (!still) {return;}
    const auto before = signal_.status();
    const std::size_t n = signal_.blinks().size();
    signal_.observe(t, o);
    if (signal_.blinks().size() != n) {
      RCLCPP_INFO(get_logger(), "신호등 점멸 %zu: %c", signal_.blinks().size(), signal_.blinks().back().color);
    }
    if (signal_.status() != before) {
      if (signal_.finished()) {
        log_signal();
        if (signal_.status() == SignalStatus::Incomplete && retry_signal(t, "불완전")) {return;}
        write_signal_record();
      }
      publish_signal();
    }
  }

  // 불완전 기록 → signal_retries 남았으면 signal_attempt<N>.json 으로 남기고 비운 뒤 다음 시퀀스를 기다림 (true)
  //   발제: 12 점멸 뒤 3 s 소등 = 끝, 운영자가 그 뒤 다시 틀어 줌. 끝 판정이 소등 2.5 s 라 다음 시퀀스 앞부분은 안 놓침
  //   불명 색을 추정해 채우지 않음 — 새 시퀀스를 처음부터 다시 봄
  bool retry_signal(double now, const char * why)
  {
    if (signal_attempt_ >= signal_retries_) {return false;}
    ++signal_attempt_;
    signal_prev_ = signal_.sequence();
    const auto path = write_record("signal_attempt" + std::to_string(signal_attempt_) + ".json",
        signal_to_json(signal_, node_start_s_, !signal_.finished()));
    RCLCPP_WARN(get_logger(), "신호등 기록 %s (%s) → 비우고 다음 시퀀스 다시 기록 %d/%d%s%s", why, signal_prev_.c_str(),
      signal_attempt_, signal_retries_, path.empty() ? "" : ", 앞 기록 ", path.c_str());
    signal_.reset();
    signal_timed_out_ = false;
    observe_start_s_ = now;
    if (phase_ == Phase::OBSERVE_SIGNAL) {
      detail_ = "신호등 다시 기록 " + std::to_string(signal_attempt_) + "/" + std::to_string(signal_retries_) + " (앞 기록 " + signal_prev_ + ")";
    }
    publish_signal();
    return true;
  }

  void publish_signal()
  {
    auto u8 = [](int v) {return static_cast<uint8_t>(std::clamp(v, 0, 255));};
    SignalRecord m;
    m.header.stamp = get_clock()->now();
    switch (signal_.status()) {
      case SignalStatus::Waiting: m.status = SignalRecord::WAITING; break;
      case SignalStatus::Recording: m.status = SignalRecord::RECORDING; break;
      case SignalStatus::Done: m.status = SignalRecord::DONE; break;
      case SignalStatus::Incomplete: m.status = SignalRecord::INCOMPLETE; break;
    }
    m.sequence = signal_.sequence();
    m.count_red = u8(signal_.count('R'));
    m.count_yellow = u8(signal_.count('Y'));
    m.count_green = u8(signal_.count('G'));
    m.count_unknown = u8(signal_.count('?'));
    m.issues = signal_.issues();
    if (signal_attempt_ > 0) {
      m.issues.push_back("다시 기록 " + std::to_string(signal_attempt_) + "/" + std::to_string(signal_retries_) + " 번째 (앞 기록 " + signal_prev_ + ")");
    }
    if (signal_timed_out_ && !signal_.finished()) {
      m.status = SignalRecord::INCOMPLETE;
      m.issues.push_back("signal_wait_s 안에 기록이 안 끝남");
    }
    pub_signal_->publish(m);
  }

  // ================================================================ 상태 발행

  void publish_status()
  {
    const double now = now_s();
    auto u8 = [](int v) {return static_cast<uint8_t>(std::clamp(v, 0, 255));};
    MissionStatus m;
    m.header.stamp = get_clock()->now();
    m.state = op_name(op_);
    m.detail = detail_;
    m.pucks_done = u8(pucks_done_);
    m.pucks_total = u8(pucks_total_);
    m.fault = (op_ == Op::FAULT);

    const bool running = op_ == Op::RUNNING;
    m.phase = running ? phase_name(phase_) : "";
    m.wait_reasons = wait_reasons();
    m.ready = (op_ == Op::STANDBY) && not_ready_.empty();
    m.result = (op_ == Op::DONE) ? result_ : "";
    m.task_id = current_task_ ? current_task_->task_id : "";
    m.step = running ? exec_->current_step_name() : "";
    m.arm_phase = exec_->arm_phase();
    m.tasks_done = u8(tasks_done_);
    m.tasks_failed = u8(tasks_failed_);
    m.tasks_total = u8(tasks_total_);
    if (start_s_ >= 0.0 && op_ != Op::DONE) {end_s_ = now;}   // 경기 시간은 FAULT 중에도 흐름, DONE 에서 멈춤
    m.elapsed_s = start_s_ >= 0.0 ? static_cast<float>(end_s_ - start_s_) : 0.0f;
    m.time_left_s = time_limit_s_ > 0.0 ? static_cast<float>(std::max(0.0, time_limit_s_ - m.elapsed_s)) : -1.0f;
    m.hold = hold_active();
    pub_status_->publish(m);
  }

  // 실제로 주행을 막는 상황에서만 true
  bool hold_active() const
  {
    switch (op_) {
      case Op::STANDBY:
      case Op::FAULT:
        return true;
      case Op::DONE:
        return false;
      case Op::RUNNING:
        return exec_->nav_blocked();
    }
    return true;
  }

  // STANDBY: 준비 안 된 이유 / RUNNING: 대기 원인 / FAULT: 원인 코드
  std::vector<std::string> wait_reasons() const
  {
    if (op_ == Op::STANDBY) {return not_ready_;}
    if (op_ == Op::FAULT) {
      std::vector<std::string> r{fault_code_};
      if (!exec_->doubt_text().empty() && fault_code_ != "INVENTORY_UNCERTAIN") {r.push_back("INVENTORY_UNCERTAIN");}
      return r;
    }
    if (op_ != Op::RUNNING) {return {};}
    return exec_->wait_reasons();
  }

  void publish_hold()
  {
    std_msgs::msg::Bool b;
    b.data = hold_active();
    pub_hold_->publish(b);
  }

  // ================================================================ 멤버

  std::unique_ptr<Executor> exec_;

  // 설정
  std::string dice_map_path_, travel_table_path_, barcode_table_path_, record_dir_;
  BarcodeTable barcodes_;   // 바코드 원문 → 정렬 방향 (barcode_table)
  double tick_hz_ = 10.0, time_limit_s_ = 0.0, return_margin_s_ = 60.0, read_timeout_s_ = 5.0;
  double look_dwell_s_ = 1.0, dice_pick_fresh_s_ = 90.0;
  double verify_radius_m_ = 0.18, verify_cell_tol_m_ = 0.03, verify_vote_ratio_ = 0.7;
  int transfer_max_remaining_ = 4, max_replans_ = 6;
  bool auto_start_ = false, barcode_look_ = true;
  bool rack_look_ = false;
  bool box_measure_ = true;                  // 장소 박스 측정으로 칸 기준 고침
  std::string box_topic_;
  int box_min_samples_ = 3;
  double box_max_shift_m_ = 0.06, box_wait_s_ = 2.0;
  std::array<std::optional<PlaceFrame>, kNumPlaces> place_frame_{};   // 장소별 확정한 박스 기준 (없으면 지도 값)
  std::deque<BoxSample> box_samples_;        // map, 최근 300 개
  rclcpp::Subscription<interfaces::msg::DetectionArray>::SharedPtr sub_box_;                   // STORE 앞에 LOOK_RACK (적재함 칸 측정 → STORE target)
  std::vector<int> collect_order_;

  // 경기장
  DiceMap dice_map_;
  TransferCosts costs_;
  bool map_ok_ = false;

  // 운영 상태 (위층) · 미션 단계 (아래층)
  Op op_ = Op::STANDBY;
  Phase phase_ = Phase::HOME;
  std::vector<std::string> not_ready_;       // STANDBY 준비 안 된 이유
  std::string fault_code_;                   // FAULT 원인 코드
  std::optional<std::pair<std::string, std::string>> pending_fault_;   // Custom 단계 · 재계획 안에서 정한 FAULT
  std::string result_;                       // DONE 결과: COMPLETED · RETURN_FAILED
  bool resume_return_ = false;               // 복귀 중 FAULT → 재시작 때 복귀 다시
  std::string detail_;
  double start_s_ = -1.0;                    // 경기 시작 (첫 /mission/start 승인, -1 = 아직)
  double end_s_ = -1.0;                      // 경과 시간 계산용 (DONE 에서 멈춤)
  double node_start_s_ = 0.0;

  // task
  std::size_t bar_i_ = 0;                    // 처리한 바코드 수 (바코드는 하나)
  int read_tries_ = 0, read_retries_ = 6;    // 바코드 다시 읽은 횟수 · 최대
  std::string pip_frame_ = "arena";          // 주사위 눈 방향 기준 (dice_pip_frame)
  std::optional<Task> pending_task_;         // 방금 읽은 task
  std::optional<Task> current_task_;         // 실행 중인 task
  std::optional<Task> queued_task_;          // 읽었지만 점멸 관측 뒤에 실행할 task
  std::set<std::string> done_ids_;           // 처리 끝난 task (성공 · 실패 모두)
  int tasks_done_ = 0, tasks_total_ = 0, tasks_failed_ = 0;
  int pucks_done_ = 0, pucks_total_ = 0;
  bool left_start_ = false;                  // task 실행 시작 = 출발 구역 떠남 (그 뒤 신호등 기록 안 함)
  std::string last_text_;
  double last_text_s_ = -1.0;

  // 신호등 점멸 기록 (발제 v5)
  SignalRecorder signal_;
  bool signal_record_ = true;
  double signal_timeout_s_ = 1.0, signal_wait_s_ = 90.0;
  double observe_start_s_ = 0.0;
  int signal_retries_ = 2;
  int signal_attempt_ = 0;                   // 다시 기록한 횟수 (불완전 기록은 signal_attempt<N>.json 으로 남김)
  std::string signal_prev_;                  // 바로 앞 불완전 기록 순서 (상태 · 발행용)
  bool signal_timed_out_ = false;            // OBSERVE_SIGNAL 시간 초과 (불완전으로 진행, reset_signal 전까지 다시 안 봄)
  double last_light_s_ = -1.0;

  // dice
  SignalCounts dice_counts_;
  std::optional<DicePlan> dice_plan_;        // 장소 그룹 · 눈 위치 (PLACE 목표 계산)
  std::vector<Step> delivery_steps_;         // Plan(Delivery) 가 만든 배달 단계 (COLLECT 끝나면 실행)
  std::vector<Step> verify_after_;           // 재검증 단계
  std::array<int, kNumPlaces> placed_count_{{0, 0, 0}};   // 장소별 놓은 수 (PLACE 성공)
  std::array<bool, kNumPlaces> place_dead_{{false, false, false}};   // PLACE 실패한 장소 (더 안 건드림)
  std::string yellow_used_shape_;            // Y 장소에 놓은 모양 (재계획 때 고정)
  int replans_ = 0;
  int current_node_ = kNodeStart;            // 마지막으로 도착한 노드
  std::array<double, kNumBoxes> look_started_{};   // 박스별 LOOK 시작 시각 (그 뒤 검출 = 그 박스 관측)
  std::array<double, kNumPlaces> look_place_started_{};   // 장소별 마지막 LOOK 시작 시각 (재검증 관측 구간)

  // 재검증 · 기록
  std::vector<PlaceVerify> verify_places_;   // 지금까지 본 장소 (같은 장소 다시 보면 바꿈)
  std::string run_dir_, verify_log_file_;
  int alloc_revision_ = 0;
  std::vector<std::string> alloc_history_;
  std::string alloc_reason_;                 // 다음 할당 기록의 이유 (재계획이면 실패 단계)

  // perception 바코드 판독 켜기 · 끄기 (READ_BARCODE 동안만)
  std::string barcode_enable_service_;
  rclcpp::Client<SetBool>::SharedPtr barcode_cli_;
  bool barcode_want_ = false, barcode_on_ = false, barcode_known_ = false, barcode_req_pending_ = false;
  double barcode_last_try_s_ = -1e9, barcode_req_sent_s_ = 0.0;
  int barcode_req_gen_ = 0;

  // ROS
  rclcpp::Subscription<TrafficLight>::SharedPtr sub_sig_;
  rclcpp::Subscription<TaskText>::SharedPtr sub_text_;
  rclcpp::Service<Trigger>::SharedPtr srv_start_, srv_resume_, srv_clear_gripper_, srv_reset_signal_;
  std::vector<rclcpp::Service<Trigger>::SharedPtr> srv_inventory_;   // /mission/inventory_before · done · removed
  rclcpp::Publisher<MissionStatus>::SharedPtr pub_status_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_hold_;
  rclcpp::Publisher<SignalRecord>::SharedPtr pub_signal_;
  rclcpp::Publisher<VerifyRecord>::SharedPtr pub_verify_;
  rclcpp::TimerBase::SharedPtr tick_timer_, status_timer_, hold_timer_;
};

}  // namespace mission

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mission::MissionMaster>());
  rclcpp::shutdown();
  return 0;
}
