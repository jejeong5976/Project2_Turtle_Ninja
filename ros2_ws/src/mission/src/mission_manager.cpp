// mission_manager.cpp — 미션 관리 노드 mission_manager, 발제 v5 '주사위 기억 로봇'
//
// 역할
//   - 미션 흐름: 바코드 읽기 → 신호등 12 점멸 기록 → 퍽 12 개 수집 · 배치 L1–L3 주사위 눈 → 재검증 → 복귀
//   - 단계 목록을 만들어 planning::Executor 에 넘기고, 실행기 사건 = 단계 성공 · 실패 · FAULT 로 다음 할 일을 정함
//   - 실제 동작 · 감시 · 퍽 기록부 · 재고는 Executor 가 함
//   - 상태 2 층
//       위층 = 로봇 운영 상태
//         STANDBY  무동작, 준비 확인: odom · control ready · 서버 · lifecycle · 위치 추정 · scan · 검출 · 그리퍼 기록 · 신호등 토픽 · 바코드 서비스
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
//                        퍽이 side_look_expected 개보다 적게 보이면 Plan 이 먼저 옆 보기 (LOOK side_look_order, 기본 left · near) → 합쳐서 Plan
//                        relook_before_pick: 손목이 그 퍽을 안 보는 자세 (STORE 뒤 P0 · 그 퍽이 없는 옆 보기) 에서 시작할 PICK 앞에
//                        LOOK 다시 (그 퍽을 본 slot) → 그 퍽 새 검출 기다림 (최대 relook_dwell_s) → PICK
//         DELIVER        운반 계획대로 [RETRIEVE] PLACE, 목표 = 장소 중심 + 주사위 눈 오프셋
//         VERIFY         재검증 순서로 장소마다 이동 → LOOK → Plan Verify
//         RETURN_HOME    출발점으로 이동
// 입력
//   - /perception/traffic_light  interfaces/TrafficLight  신호등 상태 → 점멸 기록, signal_recorder.hpp
//   - /perception/task_text      interfaces/TaskText      바코드 원문 → parse_task
//   - box_topic                  interfaces/DetectionArray 장소 박스 윗면 중심 · yaw (손목 깊이, kind place_box) → map 으로 모음
//       장소 LOOK 뒤 측정이 box_min_samples 개 이상이면 PLACE 칸 · 재검증 기준 = 측정 중심 + 박스 회전만큼 돈 눈 오프셋
//       칸은 퍽이 박스 테두리를 넘지 않게 자름 (place_box_half_m − place_puck_half_m − place_edge_margin_m 안)
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
//   - 서비스 호출 wide_enable_service std_srvs/SetBool, 기본 /perception/wide/enable (신호 기록 Done 이면 광각 끔)
//   - 서비스 호출 wide_capture_service std_srvs/SetBool, 기본 /wide_cam/set_capture (신호 기록 Done 이면 usb_cam 캡처 · MJPG 디코딩 멈춤)
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
//       예외: PICK 빈손 확인 (control 결과 글 EMPTY_GRASP, 실행기 empty_grasp_skip) → FAULT 없이 단계 실패 → dice_recover 가 그 퍽 skipped
//             + COLLECT 면 HOME → 같은 박스 앞 LOOK 다시 → Plan 다시 (빈손 쥐기가 옆 퍽을 밀었을 수 있음)
//       예외: STORE 넣기 걸림 → control 이 퍽을 박스에 되돌림 (결과 글 STORE_RETURNED_TO_BOX, 실행기 store_return_replan) → 그리퍼 빔 ·
//             퍽 박스로 → FAULT 없이 단계 실패 → 빈손 PICK 과 같이 HOME → 앞 LOOK 다시 → Plan 다시 (그 퍽 다시 집음).
//             같은 퍽이 store_return_skip_n 번 되돌아오면 그 퍽 skipped (무한 반복 방지)
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
#include <cstdio>
#include <set>
#include <stdexcept>
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
        for (auto & ws : look_windows_) {   // 팔이 움직이기 시작 → 서 있던 LOOK 자세 구간 끝 (puck_views)
          if (!ws.empty() && ws.back().to < 0.0) {ws.back().to = now;}
        }
        if (!is_look_task(s.arm_task)) {return;}
        // 옆 보기 LOOK · 다시 보기 LOOK (puck_id 있음 · PICK 앞 다시 보기) 은 관측 구간 시작을 안 바꿈 → 옆 보기 뒤 Plan 은 앞 LOOK 부터 본 퍽을 합쳐 씀
        //   빈손 뒤 다시 보기 → Plan 다시 때도 옆에서만 본 퍽이 남음 (다시 본 퍽은 기록부가 위치 · last_seen 을 새로 씀)
        if (starts_box_look(s)) {   // 새 정차의 앞 LOOK
          look_started_[box_of(s) - 1] = now;
          side_look_[box_of(s) - 1] = SideLookState::Unchecked;
          look_windows_[box_of(s) - 1].clear();
        }
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
    for (const auto b : declare_parameter<std::vector<int64_t>>("collect_order", {1, 2, 4, 5, 6, 3})) {
      collect_order_.push_back(static_cast<int>(b));
    }
    look_dwell_s_ = declare_parameter<double>("look_dwell_s", 1.0);           // LOOK 뒤 검출 모으는 시간
    // 옆 보기: 박스 LOOK 에서 쓸 퍽이 side_look_expected 개보다 적으면 Plan 전에 LOOK slot side_look_order 순서로 옆을 봄
    //   (박스 끝 퍽이 가운데 LOOK 화면 밖, 11-10 B2). 각 자세 look_dwell_s 뒤 앞 · 옆에서 본 퍽을 합쳐 계획. 정차마다 한 번
    side_cfg_.enabled = declare_parameter<bool>("side_look_missing", true);
    side_cfg_.expected = static_cast<int>(declare_parameter<int>("side_look_expected", 2));
    //   기본 left · near: 앞 + left 로 right 쪽까지 거의 다 보임 (11-10 화각 계산), near (J3 +look_near_tilt_deg) 가 가까운 줄
    side_cfg_.order = declare_parameter<std::vector<std::string>>("side_look_order", {"left", "near"});
    for (const auto & o : side_cfg_.order) {
      if (!is_side_slot(o)) {RCLCPP_WARN(get_logger(), "side_look_order 의 '%s' 는 안 씀 (right · left · near · far 만)", o.c_str());}
    }
    RCLCPP_INFO(get_logger(), "옆 보기 (side_look_missing): %s, 박스 퍽 < %d 개면 %s", side_cfg_.enabled ? "켬" : "끔",
      side_cfg_.expected, join(side_cfg_.order).c_str());
    // PICK 앞 다시 보기: 손목이 그 퍽을 안 보는 자세에서 시작할 PICK (같은 박스 둘째 퍽부터 = STORE 뒤 P0, 그 퍽이 없는 옆 보기 자세) 앞에
    //   LOOK 다시 → control 이 방금 검출에 맞춰 집음 (11-10 밤 맞춤 11/11, LOOK 때 위치로 집기 2/10 빈손). 옆에서만 본 퍽은 그 slot 으로
    //   다시 본 뒤 그 퍽 새 검출이 오면 바로, relook_dwell_s 안에 안 오면 앞 검출로 그대로 집음 (막지 않음)
    relook_before_pick_ = declare_parameter<bool>("relook_before_pick", true);
    relook_dwell_s_ = declare_parameter<double>("relook_dwell_s", 0.5);
    RCLCPP_INFO(get_logger(), "PICK 앞 다시 보기 (relook_before_pick): %s, 새 검출 최대 %.1f s", relook_before_pick_ ? "켬" : "끔",
      relook_dwell_s_);
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
    // STORE 박스 되돌림 (STORE_RETURNED_TO_BOX) 이 같은 퍽에 이 번 나오면 그 퍽 건너뜀 (그 전엔 다시 집어 넣기)
    store_return_skip_n_ = declare_parameter<int>("store_return_skip_n", 2);
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
    // 놓는 칸이 박스 테두리를 넘지 않게 (clamp_offset_in_box): 퍽 중심 ≤ 박스 반 변 − 퍽 반폭 − 여유
    place_limits_.box_half = declare_parameter<double>("place_box_half_m", 0.1025);
    place_limits_.puck_half = declare_parameter<double>("place_puck_half_m", 0.0212);
    place_limits_.margin = declare_parameter<double>("place_edge_margin_m", 0.010);
    // 장소별 PLACE 놓는 점 보정 [앞, 왼쪽] mm (정차점에서 장소를 보는 로봇 기준) — 놓는 목표에만 더함, 재검증 · 기록은 눈 점 그대로
    //   실행 중 바로 바꿈 (다음 PLACE 부터): ros2 param set /mission_manager place_offset_L1_mm "[5.0, 0.0]"
    for (int i = 1; i <= kNumPlaces; ++i) {declare_parameter<std::vector<double>>("place_offset_L" + std::to_string(i) + "_mm", {0.0, 0.0});}
    read_timeout_s_ = declare_parameter<double>("read_timeout_s", 5.0);
    read_retries_ = declare_parameter<int>("read_retries", 6);   // 안 읽히면 같은 자리에서 다시 읽는 횟수 (한 번 실패로 미션 전체 포기하던 것)
    pip_frame_ = declare_parameter<std::string>("dice_pip_frame", "arena");   // arena = 발제 p.4 지도 +Y 기준 세 곳 같은 방향 · viewer = 보는 사람 기준
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
    // 광각 (신호등 · 광각 퍽) 은 신호 기록이 Done 이 될 때까지만 켬 → 뒤 단계는 손목만 (Jetson CPU 절약). "" 면 안 부름
    wide_enable_service_ = declare_parameter<std::string>("wide_enable_service", "/perception/wide/enable");
    // 광각 usb_cam 캡처도 같은 때 멈춤 (perception 을 꺼도 usb_cam 은 MJPG 디코딩을 계속해 CPU 약 40 %). "" 면 안 부름
    wide_capture_service_ = declare_parameter<std::string>("wide_capture_service", "/wide_cam/set_capture");

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
    setup_partial_run(sc);

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
    if (!wide_enable_service_.empty()) {wide_.cli = create_client<SetBool>(wide_enable_service_);}
    if (!wide_capture_service_.empty()) {capture_.cli = create_client<SetBool>(wide_capture_service_);}
    wide_.name = "perception 광각";
    capture_.name = "광각 usb_cam 캡처";
    capture_.ok_on = "Start Capturing";   // usb_cam 0.8.1 set_capture 는 처리하고도 success = false, 글만 이렇게 줌 (Jetson 실물 확인)
    capture_.ok_off = "Stop Capturing";

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
        if (!signal_fixed_.empty()) {   // 고정 신호: 같은 값을 다시 넣음
          inject_fixed_signal();
          publish_signal();
          res->success = true;
          res->message = "signal_fixed 라 같은 값 다시 넣음: " + signal_fixed_;
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
    sync_wide_camera(now);
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

  // ---------------------------------------------------------------- 시험용 부분 실행 (기본 = 전부 끔 → 시연 흐름 그대로)
  //   signal_fixed  "RGRRGRGGYGYG"  신호등 안 보고 이 12 점멸을 기록한 것으로 (같은 판정 규칙을 거쳐 Done)
  //   barcode_fixed "P2-01"         바코드 지점에 안 가고 이 원문을 읽은 것으로
  //   start_phase   COLLECT | DELIVER | VERIFY   HOME 뒤 이 단계부터 (DELIVER = 지금 재고로 배달 계획, cargo_file + cargo_restore)
  //   stop_after    READ_BARCODE | COLLECT | DELIVER | VERIFY   이 단계 끝나면 복귀하고 COMPLETED
  //   collect_boxes [1, 2]          collect_order 에서 이 박스만 (순서는 collect_order 그대로)
  //   잘못된 값이면 시작 때 멈춤 (조용히 다른 흐름으로 가지 않게)
  static int phase_rank(const std::string & p)
  {
    if (p == "READ_BARCODE") {return 0;}
    if (p == "COLLECT") {return 1;}
    if (p == "DELIVER") {return 2;}
    if (p == "VERIFY") {return 3;}
    return -1;
  }

  void setup_partial_run(const SignalRecorderConfig & sc)
  {
    signal_fixed_ = declare_parameter<std::string>("signal_fixed", "");
    barcode_fixed_ = declare_parameter<std::string>("barcode_fixed", "");
    start_phase_ = declare_parameter<std::string>("start_phase", "");
    stop_after_ = declare_parameter<std::string>("stop_after", "");
    const auto boxes = declare_parameter<std::vector<int64_t>>("collect_boxes", std::vector<int64_t>{});
    std::vector<std::string> bad;
    if (!start_phase_.empty() && (phase_rank(start_phase_) < 1)) {bad.push_back("start_phase 는 COLLECT · DELIVER · VERIFY (지금 " + start_phase_ + ")");}
    if (!stop_after_.empty() && phase_rank(stop_after_) < 0) {bad.push_back("stop_after 는 READ_BARCODE · COLLECT · DELIVER · VERIFY (지금 " + stop_after_ + ")");}
    if (!start_phase_.empty() && !stop_after_.empty() && phase_rank(stop_after_) >= 0 && phase_rank(stop_after_) < phase_rank(start_phase_)) {
      bad.push_back("stop_after " + stop_after_ + " 가 start_phase " + start_phase_ + " 보다 앞");
    }
    if (!boxes.empty()) {
      std::set<int> want;
      for (const auto b : boxes) {
        if (b < 1 || b > kNumBoxes) {bad.push_back("collect_boxes 에 없는 박스 " + std::to_string(b));}
        want.insert(static_cast<int>(b));
      }
      std::vector<int> kept;
      for (const int b : collect_order_) {if (want.count(b)) {kept.push_back(b);}}
      if (kept.empty()) {bad.push_back("collect_boxes 가 collect_order 와 겹치는 박스 없음");}
      collect_order_ = kept;
    }
    if (!barcode_fixed_.empty() && map_ok_) {
      std::string e;
      if (!parse_task(barcode_fixed_, barcodes_, &e)) {bad.push_back("barcode_fixed " + barcode_fixed_ + " 해석 실패: " + e);}
    }
    if (!signal_fixed_.empty()) {
      bool ok = static_cast<int>(signal_fixed_.size()) == sc.expected;
      for (const char c : signal_fixed_) {ok = ok && (c == 'R' || c == 'Y' || c == 'G');}
      if (!ok) {
        bad.push_back("signal_fixed 는 R · Y · G " + std::to_string(sc.expected) + " 글자 (지금 " + signal_fixed_ + ")");
      } else {
        inject_fixed_signal();
        if (signal_.status() != SignalStatus::Done) {
          bad.push_back("signal_fixed " + signal_fixed_ + " 가 기록 규칙에 안 맞음 (" + signal_status_name(signal_.status()) + ")");
        }
      }
    }
    if (!bad.empty()) {
      std::string all;
      for (const auto & b : bad) {all += "\n  - " + b;}
      RCLCPP_FATAL(get_logger(), "부분 실행 설정 오류:%s", all.c_str());
      throw std::runtime_error("부분 실행 설정 오류" + all);
    }
    if (!signal_fixed_.empty() || !barcode_fixed_.empty() || !start_phase_.empty() || !stop_after_.empty() || !boxes.empty()) {
      std::string order;
      for (const int b : collect_order_) {order += (order.empty() ? "B" : " B") + std::to_string(b);}
      RCLCPP_WARN(get_logger(), "시험용 부분 실행: 신호 %s · 바코드 %s · 시작 %s · 끝 %s · 박스 %s",
        signal_fixed_.empty() ? "실제" : signal_fixed_.c_str(), barcode_fixed_.empty() ? "실제" : barcode_fixed_.c_str(),
        start_phase_.empty() ? "처음부터" : start_phase_.c_str(), stop_after_.empty() ? "끝까지" : stop_after_.c_str(), order.c_str());
    }
  }

  // signal_fixed → 기록기에 1 s 켜짐 · 1 s 소등 점멸을 0.1 s 간격으로 넣음 (지난 시각, 같은 판정 규칙) → Done
  void inject_fixed_signal()
  {
    signal_.reset();
    signal_timed_out_ = false;
    double t = now_s() - (2.0 * static_cast<double>(signal_fixed_.size()) + 6.0);
    auto feed = [&](SignalObs o, double dur) {
        for (int k = 0; k < static_cast<int>(std::lround(dur / 0.1)); ++k) {
          signal_.observe(t, o);
          t += 0.1;
        }
      };
    feed(SignalObs::Off, 1.0);
    for (const char c : signal_fixed_) {
      feed(c == 'R' ? SignalObs::Red : c == 'Y' ? SignalObs::Yellow : SignalObs::Green, 1.0);
      feed(SignalObs::Off, 1.0);
    }
    feed(SignalObs::Off, 4.0);   // 끝 소등 (end_off_s 2.5 보다 길게)
    if (signal_.status() == SignalStatus::Done) {
      RCLCPP_WARN(get_logger(), "신호 고정 (signal_fixed): %s — 신호등 입력 안 씀", signal_.sequence().c_str());
    }
  }

  // stop_after 단계가 끝남 → task 를 끝낸 것으로 정리하고 복귀 (결과 COMPLETED)
  void finish_partial(const char * what)
  {
    if (current_task_) {
      ++tasks_done_;
      done_ids_.insert(current_task_->task_id);
      current_task_.reset();
    }
    ++bar_i_;
    delivery_steps_.clear();
    verify_after_.clear();
    RCLCPP_WARN(get_logger(), "시험 stop_after %s: 여기서 끝 (놓은 퍽 %d) → 복귀", what, pucks_done_);
    start_return(std::string("시험 stop_after ") + what);
  }

  // 바코드 task 를 얻은 뒤 (읽음 · barcode_fixed): stop_after READ_BARCODE → 복귀 / 신호 기록 기다림 / 실행
  void after_barcode()
  {
    if (stop_after_ == "READ_BARCODE") {
      RCLCPP_WARN(get_logger(), "시험 stop_after READ_BARCODE: 바코드 %s (%s) → 복귀", pending_task_->raw_text.c_str(),
        dice_order_name(pending_task_->dice_order));
      ++bar_i_;
      start_return("시험 stop_after READ_BARCODE");
      return;
    }
    if (signal_record_ && !signal_.finished() && !signal_timed_out_) {
      queued_task_ = pending_task_;
      begin_observe(now_s());
      return;
    }
    start_execute(*pending_task_);
  }

  // start_phase DELIVER: 수집 없이 지금 재고 (cargo_file 에서 복원한 칸 · 그리퍼) 로 배달 계획
  void start_from_cargo()
  {
    const auto & cargo = exec_->cargo();
    std::size_t n = cargo.gripper() ? 1 : 0;
    for (const auto & sl : cargo.slots()) {n += sl.second ? 1 : 0;}
    if (n == 0) {
      abandon_dice("시험 start_phase DELIVER: 재고 없음 (cargo_file · cargo_restore: true 로 칸 기록을 넣을 것)");
      return;
    }
    RCLCPP_WARN(get_logger(), "시험 start_phase DELIVER: 재고 퍽 %zu 개로 배달 계획", n);
    if (!plan_from_state(false, now_s())) {
      abandon_dice("시험 start_phase DELIVER: 배달 계획 실패");
      return;
    }
    auto steps = std::move(delivery_steps_);
    delivery_steps_.clear();
    set_steps(Phase::DELIVER, std::move(steps));
    if (exec_->done()) {on_steps_done();}   // 채울 장소 없음 → 재검증 · 끝
  }

  // 준비 확인 = 실행기 (odom · 팔 · Nav2 · 위치 · 라이다 · 손목 검출) + 미션 입력
  //   TRAFFIC_LIGHT: /perception/traffic_light 가 signal_timeout_s 넘게 안 옴 (신호 기록이 Done 이면 안 봄, Incomplete 면 다시 기록하니 봄)
  //   BARCODE_SERVICE: 바코드 판독 켜기 서비스가 안 보임 (READ_BARCODE 에서 부름)
  std::vector<std::string> ready_reasons(double now)
  {
    auto r = exec_->ready_reasons(now);
    if (signal_.status() != SignalStatus::Done && (last_light_s_ < 0.0 || now - last_light_s_ > signal_timeout_s_)) {
      r.push_back("TRAFFIC_LIGHT");
    }
    if (barcode_cli_ && barcode_fixed_.empty() && !barcode_cli_->service_is_ready()) {r.push_back("BARCODE_SERVICE:" + barcode_enable_service_);}
    return r;
  }

  // ---------------------------------------------------------------- STANDBY: 무동작, 준비 확인 · 출발 대기
  void tick_standby(double now)
  {
    not_ready_ = ready_reasons(now);
    // 준비 상태가 바뀔 때만 로그 (run_robot.sh mission 이 'STANDBY 준비 완료' 줄로 시작 가능을 알림).
    //   이유가 깜빡이며 바뀌어도 '안 됨' 줄은 2 s 에 한 번까지, '완료' 는 바로
    const bool changed = not_ready_ != logged_not_ready_ || !standby_logged_;
    if (changed && (not_ready_.empty() || now - standby_log_s_ >= 2.0)) {
      standby_log_s_ = now;
      std::string why;
      for (const auto & r : not_ready_) {why += (why.empty() ? "" : ", ") + r;}
      if (not_ready_.empty()) {
        RCLCPP_INFO(get_logger(), "STANDBY 준비 완료 — /mission/start 받을 수 있음");
      } else {
        RCLCPP_INFO(get_logger(), "STANDBY 준비 안 됨: %s", why.c_str());
      }
      logged_not_ready_ = not_ready_;
      standby_logged_ = true;
    }
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
    const auto r = ready_reasons(now);
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
      // 기록 중이면 signal_wait_s 를 넘어도 끝까지 (12 점멸 24 s + 끝 3 s 를 넉넉히 40 s 더). 기다리다 늦게 시작한 신호를 지우지 않게
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
    side_look_.fill(SideLookState::Unchecked);
    for (auto & ws : look_windows_) {ws.clear();}
    puck_views_.clear();
    look_place_started_.fill(0.0);
    for (auto & f : place_frame_) {f.reset();}
    place_dead_ = {false, false, false};
    yellow_used_shape_.clear();
    replans_ = 0;
    store_returned_n_.clear();
    current_node_ = kNodeStart;
    alloc_revision_ = 0;
    alloc_history_.clear();
    alloc_reason_.clear();
    left_start_ = true;
    detail_.clear();   // OBSERVE_SIGNAL 의 "기록 중" 등을 지움
    RCLCPP_INFO(get_logger(), "task 시작: %s (%s, R %d · Y %d · G %d)", task.raw_text.c_str(),
      dice_order_name(task.dice_order), dice_counts_.red, dice_counts_.yellow, dice_counts_.green);
    if (start_phase_ == "DELIVER") {
      start_from_cargo();
      return;
    }
    if (start_phase_ == "VERIFY") {   // 시험: 이미 놓인 배치를 재검증만 (L1 → L2 → L3)
      RCLCPP_WARN(get_logger(), "시험 start_phase VERIFY: 세 장소 재검증만");
      set_steps(Phase::VERIFY, verify_steps({{0, 1, 2}}, dice_map_));
      if (exec_->done()) {on_steps_done();}
      return;
    }
    if (stop_after_ == "COLLECT") {   // 시험 수집만: 마지막 박스도 배달 계획 대신 집기 계획 (배달 계획이 그 박스 퍽을 DELIVER 에서 집기 때문)
      for (auto & st : steps) {
        if (is_custom(st, kPlanDelivery)) {
          st.kind = kPlanCollectBox;
          st.name = "plan:collect:B" + std::to_string(box_of(st));
        }
      }
    }
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
    if (!barcode_fixed_.empty()) {   // 시험: 바코드 지점에 안 감
      pending_task_ = parse_task(barcode_fixed_, barcodes_, nullptr);
      RCLCPP_WARN(get_logger(), "바코드 고정 (barcode_fixed): %s (%s)", pending_task_->raw_text.c_str(),
        dice_order_name(pending_task_->dice_order));
      phase_ = Phase::READ_BARCODE;
      after_barcode();
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
    // 박스 LOOK 자세에 섬 → 다음 팔 작업 보낼 때까지 이 자세 구간 (그 동안 찍힌 퍽 = 이 slot 에서 보임, puck_views)
    if (s.type == StepType::Arm && is_look_task(s.arm_task) && box_of(s) >= 1 && box_of(s) <= kNumBoxes) {
      auto & ws = look_windows_[box_of(s) - 1];
      ws.push_back(LookWindow{s.slot, now_s(), -1.0});
      if (ws.size() > 32) {ws.erase(ws.begin());}
    }
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
        after_barcode();
        break;
      case Phase::COLLECT:
        if (stop_after_ == "COLLECT") {
          finish_partial("COLLECT");
          break;
        }
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
        if (stop_after_ == "DELIVER") {
          finish_partial("DELIVER");
          break;
        }
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
        // 안 읽힘 → 같은 자리에서 다시 읽기 (read_retries 번). 이미 처리한 바코드 · 팔 · 이동 실패는 그대로 끝
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
      case kLookDwell: return now - started < look_dwell_s_ ? StepResult::Running : StepResult::Success;   // 옆 보기 자세로 검출 모음
      case kRelookDwell: return run_relook_dwell(s, now, started);
      default:
        detail = "모르는 Custom 단계: " + std::to_string(s.kind);
        return StepResult::Fail;
    }
  }

  // PLACE 목표: 장소 중심 + 주사위 눈 오프셋 (map). 높이 = 놓는 면 + 퍽 반높이 + grasp_offset
  //   이 장소 LOOK 뒤 박스 측정이 있으면 측정 중심 + 박스 회전만큼 돈 오프셋 (place_box.hpp), 없으면 지도 값
  //   퍽이 박스 테두리를 넘지 않게 눈 오프셋을 박스 좌표에서 (반 변 − 퍽 반폭 − 여유) 안으로 자름 (측정 박스 기준, 없으면 지도 박스)
  bool dice_cell_target(const Step & s, geometry_msgs::msg::PoseStamped & out, std::string & detail)
  {
    const int p = place_of(s), k = cell_of(s);
    const auto pt = (is_dice_cell(s) && dice_plan_) ? dice_cell_point(*dice_plan_, dice_map_, p, k) : std::nullopt;
    if (!pt) {
      detail = "dice 칸 없음: L" + std::to_string(p + 1) + " #" + std::to_string(k);
      return false;
    }
    std::array<double, 2> off = dice_plan_->places[p].pip_offsets[k];
    const std::array<double, 2> off0 = off;
    if (clamp_offset_in_box(off, dice_map_.map_from_arena.yaw, place_limits_)) {
      RCLCPP_WARN(get_logger(), "L%d #%d: 칸이 박스 테두리 여유 밖 → 오프셋 (%.1f, %.1f) → (%.1f, %.1f) mm 로 안쪽에 놓음", p + 1, k,
        1000.0 * off0[0], 1000.0 * off0[1], 1000.0 * off[0], 1000.0 * off[1]);
    }
    std::array<double, 2> xy;
    if (const auto & f = place_frame_[p]) {
      xy = frame_point(*f, off);
      RCLCPP_INFO(get_logger(), "L%d #%d: 박스 측정 기준 (지도 대비 %.1f mm 이동 · %.1f° 회전) → 칸 (%.3f, %.3f)", p + 1, k,
        1000.0 * std::hypot((*pt)[0] - xy[0], (*pt)[1] - xy[1]), f->dyaw * 180.0 / M_PI, xy[0], xy[1]);
    } else {
      const auto & c = dice_map_.places[p].center;
      xy = {c[0] + off[0], c[1] + off[1]};
      if (box_measure_) {
        RCLCPP_WARN(get_logger(), "L%d #%d: 박스 측정 없음 → 지도 박스 기준으로 놓음 (실제 박스가 옮겨졌으면 테두리 보장 못 함)", p + 1, k);
      }
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
    //   옆 보기 LOOK · 다시 보기 LOOK 은 look_started_ 를 안 바꿈 → 옆 보기 · 빈손 뒤 다시 보기 뒤에도 앞 LOOK 부터 본 퍽 (앞 + 옆 합침)
    const double look_t = look_started_[box - 1];
    const auto at_box = pucks_at_box(box, look_t > 0.0 ? look_t - 0.5 : started - 2.0);
    RCLCPP_INFO(get_logger(), "B%d LOOK: 퍽 %zu 개 (%s)", box, at_box.size(), puck_list(at_box).c_str());
    // 옆 보기: 이 정차 첫 Plan 에서 퍽이 모자라면 계획 전에 옆을 보고 같은 Plan 을 다시 (그때는 본 만큼으로 계획)
    const bool side_merged = side_look_[box - 1] == SideLookState::Pending;
    if (auto more = side_look_on_plan(side_look_[box - 1], s, at_box.size(), side_cfg_); !more.empty()) {
      std::string slots;
      for (const auto & m : more) {
        if (is_side_look(m)) {slots += (slots.empty() ? "" : ", ") + m.slot;}
      }
      RCLCPP_INFO(get_logger(), "B%d LOOK: 퍽 %zu 개 < %d → 옆 보기 (%s)", box, at_box.size(), side_cfg_.expected, slots.c_str());
      exec_->insert_after_current(std::move(more));
      return StepResult::Success;
    }
    if (side_merged) {
      RCLCPP_INFO(get_logger(), "B%d 옆 보기 뒤: 앞 + 옆 퍽 %zu 개로 계획 (%s)", box, at_box.size(), puck_list(at_box).c_str());
    }
    if (relook_before_pick_) {update_puck_views(box, at_box);}

    if (s.kind == kPlanCollectBox) {
      std::string err;
      const auto actions = collect_box_actions(at_box, exec_->cargo(), &err);
      if (!err.empty()) {
        detail = "B" + std::to_string(box) + " 집기 계획 실패: " + err;
        return StepResult::Fail;
      }
      auto arm = arm_steps(actions, rack_look_, box_measure_);
      if (relook_before_pick_) {arm = with_pick_relooks(arm, held_arm_step(), puck_views_);}
      exec_->insert_after_current(std::move(arm));
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

  // ---------------------------------------------------------------- PICK 앞 다시 보기 (relook_before_pick)
  // 이 정차 LOOK 자세 구간마다 찍힌 퍽 → 퍽별 본 slot (puck_views_). 먼저 기록한 slot 은 두고 새 slot 만 뒤에 더함
  //   (원시 관측은 최근 30 s 만 있음 → 늦은 Plan 다시 때 앞 구간 관측이 없어져도 처음 본 slot 이 남음)
  void update_puck_views(int box, const std::vector<TransferPuck> & at_box)
  {
    std::vector<SeenAt> seen;
    for (const ObsRecord & o : exec_->recent_obs()) {
      if (!o.id.empty()) {seen.push_back(SeenAt{o.id, o.stamp});}
    }
    for (const auto & [id, slots] : puck_views(look_windows_[box - 1], seen)) {
      auto & v = puck_views_[id];
      for (const auto & sl : slots) {
        if (std::find(v.begin(), v.end(), sl) == v.end()) {v.push_back(sl);}
      }
    }
    for (const auto & p : at_box) {
      const std::string slot = relook_slot(puck_views_, p.id);
      if (!slot.empty()) {RCLCPP_INFO(get_logger(), "B%d 퍽 %s: 옆 보기 %s 에서만 봄 → 집기 전 다시 보기도 %s", box, p.id.c_str(), slot.c_str(), slot.c_str());}
    }
  }

  // 지금 단계 앞 같은 정차의 마지막 팔 단계 = 팔이 서 있는 자세 (이동 단계 너머는 안 봄, 없으면 nullopt)
  std::optional<Step> held_arm_step() const
  {
    const auto & st = exec_->steps();
    for (std::size_t j = std::min(exec_->index(), st.size()); j-- > 0;) {
      if (st[j].type == StepType::Navigate) {break;}
      if (st[j].type == StepType::Arm) {return st[j];}
    }
    return std::nullopt;
  }

  // 다시 보기 LOOK 뒤: 그 자세에 선 뒤 찍힌 그 퍽 검출이 오면 바로 PICK, relook_dwell_s 넘으면 앞 검출로 PICK (막지 않음)
  StepResult run_relook_dwell(const Step & s, double now, double started)
  {
    double since = started;
    if (box_of(s) >= 1 && box_of(s) <= kNumBoxes) {   // LOOK 성공 시각 (이 단계는 다음 주기에 시작)
      const auto & ws = look_windows_[box_of(s) - 1];
      if (!ws.empty() && ws.back().to < 0.0) {since = std::min(since, ws.back().from);}
    }
    const auto & obs = exec_->recent_obs();
    for (auto it = obs.rbegin(); it != obs.rend(); ++it) {   // 받은 순서 (카메라마다 촬영 시각이 조금 엇갈릴 수 있어 끝까지 봄)
      if (it->stamp >= since && it->id == s.puck_id) {
        RCLCPP_INFO(get_logger(), "B%d 퍽 %s 다시 보기: 새 검출 %.2f s 뒤 → PICK", box_of(s), s.puck_id.c_str(), now - since);
        return StepResult::Success;
      }
    }
    if (now - started < relook_dwell_s_) {return StepResult::Running;}
    RCLCPP_WARN(get_logger(), "B%d 퍽 %s 다시 보기: %.1f s 안에 새 검출 없음 → 앞 검출로 PICK", box_of(s), s.puck_id.c_str(),
      relook_dwell_s_);
    return StepResult::Success;
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
      // 서서 이어 하면 (마지막 박스) 지금 팔 자세부터, 아니면 첫 정차가 이동 → LOOK
      if (relook_before_pick_) {delivery_steps_ = with_pick_relooks(delivery_steps_, stopped ? held_arm_step() : std::nullopt, puck_views_);}
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
  //             예외: PICK 빈손 확인 (EMPTY_GRASP) 은 팔이 움직였어도 여기 옴 (그리퍼 빔 · 퍽 박스에 그대로, need_home → HOME 먼저)
  //                   COLLECT 면 Plan 다시 앞에 같은 정차 앞 LOOK 다시 (box_relook_step: HOME → LOOK → Plan, 밀린 옆 퍽 새 검출)
  //             PICK 실패 = 그 퍽 박스에 남김 (skipped, 다시 안 집음)
  //             PLACE 실패한 장소는 더 안 건드림 (같은 이유로 다시 실패할 수 있음, 퍽은 그리퍼에 남아 다른 장소에 씀)
  //   실행기 사건 처리 중 (주기 사이) 에만 부름 → 단계 목록을 바꿔도 됨
  bool dice_recover(const Step & f)
  {
    // 옆 보기 LOOK (right · left · near · far) · PICK 앞 다시 보기 LOOK 실패 · 거절: 그 LOOK 만 건너뜀 (재계획 · 횟수 아님)
    //   옆 보기: 다음 옆 보기 · 같은 Plan 은 그대로 → 본 만큼으로 계획. 다시 보기: 그 PICK 은 앞 검출로 (예전처럼)
    //   팔이 움직이다 실패했으면 on_step_failed 가 다음 단계 앞에 HOME
    if (is_side_look(f) || is_pick_relook(f)) {
      const std::size_t at = exec_->index();
      std::size_t e = std::min(at + 1, exec_->steps().size());
      if (e < exec_->steps().size() &&
        (is_custom(exec_->steps()[e], kLookDwell) || is_custom(exec_->steps()[e], kRelookDwell))) {++e;}   // 그 자세 기다리기도 뺌
      exec_->erase_steps(at, e);
      RCLCPP_WARN(get_logger(), "%s 실패 → 이 %s 건너뜀 (%s)", f.name.c_str(), is_pick_relook(f) ? "다시 보기" : "옆 보기",
        is_pick_relook(f) ? "앞 검출로 집음" : "본 퍽으로 계속");
      if (exec_->done()) {on_steps_done();}
      return true;
    }
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
        case kArmStore:
          if (detail_.find(kStoreReturnedToken) != std::string::npos) {   // 퍽은 박스로 되돌아감 (실행기가 그리퍼 비움 · on_field)
            const int n = ++store_returned_n_[f.puck_id];
            if (n >= store_return_skip_n_) {
              exec_->mark_puck(f.puck_id, "skipped");
              RCLCPP_WARN(get_logger(), "STORE 박스 되돌림 %s: %d 번째 → 그 퍽 건너뜀 (박스에 둠)", f.puck_id.c_str(), n);
            } else {
              RCLCPP_WARN(get_logger(), "STORE 박스 되돌림 %s: %d 번째 → 다시 집어 넣기 (%d 번째면 건너뜀)", f.puck_id.c_str(), n, store_return_skip_n_);
            }
          }
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
      if (box >= 1 && box <= kNumBoxes) {side_look_[box - 1] = SideLookState::Done;}   // 복구 Plan 은 옆 보기 안 함 (실패 처리 그대로)
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
      std::vector<Step> ins{again};
      // PICK 빈손 (EMPTY_GRASP): 손가락이 박스 안까지 내려가 닫혔다 올라옴 → 옆 퍽이 밀렸을 수 있음
      //   → 같은 정차 앞 LOOK 을 다시 (HOME → LOOK → Plan) 해서 새 검출로 계획. 다른 실패는 예전대로 (팔 안 들어감)
      //   STORE 박스 되돌림 (STORE_RETURNED_TO_BOX) 도 같음: 퍽을 박스에 다시 내려놓음 → 새 검출로 다시 집음
      const bool store_back = f.type == StepType::Arm && f.arm_task == kArmStore && detail_.find(kStoreReturnedToken) != std::string::npos;
      if (!skip_box && f.type == StepType::Arm && ((f.arm_task == kArmPick && detail_.find(kEmptyGraspToken) != std::string::npos) || store_back))
      {
        const char * what = store_back ? "STORE 박스 되돌림" : "빈손 PICK";
        if (auto look = box_relook_step(exec_->steps(), at, box, f.puck_id)) {
          RCLCPP_WARN(get_logger(), "%s %s → B%d 앞 LOOK 다시 뒤 계획 (새 검출)", what, f.puck_id.c_str(), box);
          ins.insert(ins.begin(), *look);
        } else {
          RCLCPP_WARN(get_logger(), "%s %s → B%d 앞 LOOK 못 찾음, 앞 검출로 계획", what, f.puck_id.c_str(), box);
        }
      }
      exec_->insert_steps(at, ins);
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

  struct EnableToggle   // std_srvs/SetBool 로 켜고 끄는 대상 하나 (sync_toggle)
  {
    std::string name;
    rclcpp::Client<SetBool>::SharedPtr cli;
    std::string ok_on, ok_off;   // success 를 안 채우는 서버면 응답 글이 이것과 같을 때 성공으로 봄 (usb_cam set_capture)
    bool on = false, known = false, pending = false;
    double last_try_s = -1e9, sent_s = 0.0;
    int gen = 0;
  };

  // 광각을 신호 기록에 맞춤: Done 전 = 켬 (준비 확인 TRAFFIC_LIGHT 도 이때만 봄), Done = 끔
  //   perception 광각 처리 · usb_cam 캡처 둘 다. reset_signal 로 다시 기록하면 다시 켬
  void sync_wide_camera(double now)
  {
    const bool want = signal_.status() != SignalStatus::Done;
    sync_toggle(capture_, want, now);
    sync_toggle(wide_, want, now);
  }

  // std_srvs/SetBool 서비스 하나를 want 에 맞춤. 응답 없으면 2 s 뒤 버리고 다시, 서비스 없으면 1 s 마다 다시 봄
  //   (서비스가 없으면 그 노드가 없는 것 → 광각 perception 이면 TRAFFIC_LIGHT 가 알림)
  void sync_toggle(EnableToggle & t, bool want, double now)
  {
    if (!t.cli) {return;}
    if (t.pending) {
      if (now - t.sent_s < 2.0) {return;}
      t.cli->prune_pending_requests();
      t.pending = false;
      RCLCPP_WARN(get_logger(), "%s 켜기 · 끄기 서비스 응답 없음 → 다시 보냄", t.name.c_str());
    }
    if (t.known && t.on == want) {return;}
    if (now - t.last_try_s < 1.0) {return;}
    t.last_try_s = now;
    if (!t.cli->service_is_ready()) {return;}
    auto req = std::make_shared<SetBool::Request>();
    req->data = want;
    const int gen = ++t.gen;
    t.pending = true;
    t.sent_s = now;
    t.cli->async_send_request(req, [this, &t, want, gen](rclcpp::Client<SetBool>::SharedFuture f) {
        if (gen != t.gen) {return;}
        t.pending = false;
        const auto r = f.get();
        const std::string & ok_msg = want ? t.ok_on : t.ok_off;
        if (r->success || (!ok_msg.empty() && r->message == ok_msg)) {
          t.known = true;
          t.on = want;
          RCLCPP_INFO(get_logger(), "%s %s%s", t.name.c_str(), want ? "켬" : "끔", want ? "" : " (신호 기록 끝)");
        } else {
          RCLCPP_WARN(get_logger(), "%s %s 실패: %s", t.name.c_str(), want ? "켜기" : "끄기", r->message.c_str());
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
    if (!signal_fixed_.empty() || !signal_record_ || signal_.finished() || signal_timed_out_) {return;}
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
  SideLookConfig side_cfg_;                  // 옆 보기 (side_look_*)
  bool relook_before_pick_ = true;           // PICK 앞 다시 보기 (with_pick_relooks)
  double relook_dwell_s_ = 0.5;              // 다시 보기 뒤 그 퍽 새 검출 최대 기다림 [s]
  double verify_radius_m_ = 0.18, verify_cell_tol_m_ = 0.03, verify_vote_ratio_ = 0.7;
  int transfer_max_remaining_ = 4, max_replans_ = 6;
  bool auto_start_ = false, barcode_look_ = true;
  bool rack_look_ = false;
  bool box_measure_ = true;                  // 장소 박스 측정으로 칸 기준 고침
  std::string box_topic_;
  int box_min_samples_ = 3;
  double box_max_shift_m_ = 0.06, box_wait_s_ = 2.0;
  PlaceLimits place_limits_;                 // 놓는 칸 박스 테두리 안 범위
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
  std::vector<std::string> logged_not_ready_;   // 마지막으로 로그에 남긴 이유 (바뀔 때만 다시 남김)
  bool standby_logged_ = false;
  double standby_log_s_ = -1e9;
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
  std::string signal_fixed_, barcode_fixed_, start_phase_, stop_after_;   // 시험용 부분 실행 (setup_partial_run)
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
  int store_return_skip_n_ = 2;
  std::map<std::string, int> store_returned_n_;   // 퍽별 STORE 박스 되돌림 횟수 (이번 미션)
  int current_node_ = kNodeStart;            // 마지막으로 도착한 노드
  std::array<double, kNumBoxes> look_started_{};   // 박스별 앞 LOOK 시작 시각 (그 뒤 검출 = 그 박스 관측, 옆 보기 · 다시 보기 LOOK 은 안 바꿈 = starts_box_look)
  std::array<SideLookState, kNumBoxes> side_look_{};   // 박스별 이 정차의 옆 보기 (dice_steps.hpp, 앞 LOOK 이 Unchecked 로 되돌림)
  std::array<std::vector<LookWindow>, kNumBoxes> look_windows_{};   // 박스별 이 정차 LOOK 자세 구간 (앞 LOOK 이 비움, puck_views)
  PuckViews puck_views_;                     // 퍽 ID → 그 퍽을 본 LOOK slot (박스 Plan 때 갱신, task 마다 비움)
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

  // perception 광각 켜기 · 끄기 (신호 기록 Done 전까지만)
  std::string wide_enable_service_, wide_capture_service_;
  EnableToggle wide_, capture_;   // perception 광각 처리 · usb_cam 캡처

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
  try {
    rclcpp::spin(std::make_shared<mission::MissionMaster>());
  } catch (const std::runtime_error & e) {   // 설정 오류 (부분 실행 등): 이유는 이미 FATAL 로그, core dump 없이 끝
    std::fprintf(stderr, "mission_manager 끝: %s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
