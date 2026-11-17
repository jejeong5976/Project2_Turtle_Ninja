// mission_manager.cpp — 미션 관리 노드 mission_manager, 발제 '주사위 기억 로봇'
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
//                  task 중 FAULT 면 (resume_continue_task, 기본 켬) task 를 버리지 않고 HOME 뒤 지금 재고로 그 단계부터 이어 감
//       아래층 = RUNNING 안 미션 단계: HOME → READ_BARCODE → OBSERVE_SIGNAL → COLLECT → DELIVER → VERIFY → RETURN_HOME
//         HOME           start 승인 뒤 · 복구 뒤 항상 여기부터. 출발 전에는 안 움직임
//         READ_BARCODE   이동 → LOOK_BARCODE → 읽기 → HOME. 이 단계 동안만 perception 바코드 판독을 켬
//                        barcode_search (기본 켬): 바코드 자리 무작위 → 광각 상자 · 라이다 · 손목 깊이로 찾고 LOOK_AT (팔 작업 9) 으로 겨눠 읽음
//                        (barcode_search.hpp, bc_* 파라미터). 신호등이 광각 시야 안일 때만 몸을 돌림 · 팔이 신호등을 가리지 않음. 못 찾으면 예비 고정 경로
//         OBSERVE_SIGNAL 출발 구역에 선 채 12 점멸 기록 끝까지 기다림, signal_wait_s 넘으면 불완전으로 진행
//                        불완전 ('?' · 개수 · 끊김 · 횟수 규칙) 이면 signal_retries 번까지 기록을 비우고 다음 시퀀스를 다시 기록
//                        기록은 광각 켬 확인 뒤 관측 + 첫 확실한 색 점멸부터. 12 개 전 끝 소등 = 중간부터 본 것 → 다음 시퀀스 기다림
//                        이 단계 중에는 광각을 끄지 않음 (B2)
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
//                               task 중 FAULT 였으면 start 뒤 HOME → 이어 하기 (COLLECT 남은 박스 · DELIVER 재계획 · VERIFY 남은 장소)
//       /mission/clear_gripper  그리퍼 재고 기록 지움, 기록이 남아 있으면 start 거절. 재고 불확실이면 거절
//       /mission/store_done/<칸> 사람이 직접 한 STORE 기록: 그리퍼 퍽 → 그 칸 (STORE 성공 전환). RUNNING · 불확실 · 찬 칸이면 거절
//       /mission/inventory_before   재고 불확실 정합: 실물이 원인 단계 전과 같음 (기록 그대로)
//       /mission/inventory_done     재고 불확실 정합: 실물이 원인 단계 성공과 같음 (성공 전환 한 번 반영)
//       /mission/inventory_removed  재고 불확실 정합: 사람이 그 퍽을 로봇에서 뺌 (그리퍼 · 그 칸 기록에서 지움)
//         셋 다 RUNNING 아닐 때 · 팔 작업 최종 결과 뒤에만. 사람이 실물을 직접 본 뒤 부를 것
//         PLACE 를 done 으로 정합하면 그 칸에 놓은 것으로 셈 (놓은 수 · 장소별 놓은 수) → 이어 하기가 그 장소를 계속 채움
//       /mission/inventory_returned 사람이 손의 퍽을 빼서 그 퍽 원래 박스 위에 되돌려 놓음 → 그리퍼 기록 비움 · 퍽 박스 위
//                               (쥔 채 실패한 STORE · LOOK_RACK · PLACE 불확실이면 before 정합 같이). 이어 하기에서 다시 집음
//       /mission/reset_signal   신호등 기록 지우고 다시 기다림, RUNNING 중에는 출발 구역 단계일 때만
//   - 파일: dice_map = maps/dice_map.yaml, travel_table = maps/dice_travel_times.yaml, barcode_table = maps/dice_barcodes.json
// 출력
//   - /mission/status          interfaces/MissionStatus  2 Hz
//   - /mission/hold            std_msgs/Bool             주행을 막을 때 true: STANDBY · FAULT · 이동 차단 원인 → navigation cmd_vel_hold
//   - /planning/signal_record  interfaces/SignalRecord   바뀔 때 + 1 Hz, transient_local
//   - /planning/verify_record  interfaces/VerifyRecord   transient_local
//   - /planning/verify_image   sensor_msgs/Image (bgr8)  재검증 장소마다 판정 때 1 장 (본 퍽 · 맞춘 칸 · OK/FAIL 그림), transient_local 3
//                              VERIFY 단계 동안만 손목 컬러 영상 구독 (verify_color_topic), 기록 폴더에 verify_L<n>.jpg 도 씀
//   - 서비스 호출 barcode_enable_service std_srvs/SetBool, 기본 /perception/barcode/enable
//   - 서비스 호출 wide_enable_service std_srvs/SetBool, 기본 /perception/wide/enable (신호 기록 Done 이면 광각 끔)
//   - 서비스 호출 wide_capture_service std_srvs/SetBool, 기본 /wide_cam/set_capture (신호 기록 Done 이면 usb_cam 캡처 · MJPG 디코딩 멈춤)
//   - 실행기 출력: navigate_to_pose · /arm_task · /mission/cargo · /planning/markers
//   - 기록 파일 <record_dir>/run_<시각>/ signal.json · signal_attempt<N>.json (다시 기록 전 불완전 기록) · signal_partial<N>.json (12 개 전 끝 소등 시퀀스) · allocation.json · allocation_rev<N>.json · verify.yaml · verify_L<n>.jpg · verify_L<n>_raw.jpg (덧그리기 전) · verify_L<n>_cam.yaml
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
//       STORE 거절 · 보내기 전 실패 (퍽은 손에): 다음 Plan 이 그 칸을 빼고 다른 빈 칸 (store_ok) → 실패 칸이 store_alt_cells 넘거나
//             다른 칸이 없으면 집은 박스 자리에 PLACE 로 되돌림 (PUTBACK, 그 퍽 skipped) → 같은 Plan 다시. 포기 (재계획 초과 등) 때
//             퍽을 쥐고 있으면 먼저 되돌림 → 그 뒤 포기 · 복귀 (쥔 채 복귀 = CARRY_DRIVE 를 안 만듦). 되돌림도 실패하면 그대로 포기 · 복귀
//       포기 · 시간 부족 때 쥔 퍽: 다른 칸 STORE → 집은 박스 되돌림 순으로 손 비운 뒤 복귀 (start_empty_hand, DELIVER 도 STORE).
//             DELIVER 의 STORE (막는 퍽 옮기기) 실패도 다른 칸에 넣고 지금 자리에서 다시 계획. 첫 이동 FAULT (NAV_NOT_WORKING) 는
//             fault_auto_wait_s 뒤 스스로 resume · start (nav_fault_auto_n 번). 부분 수집 마지막 박스 실패 = 바로 끝 처리
//   - 대기 원인은 상태가 아님: 실행기 wait_reasons = DRIVE_LOST · SCAN_LOST · DETECTION_LOST · LOCALIZATION_LOST
//   - time_limit_s > 0 이면 첫 start 부터 재서 시간이 모자랄 때 RETURN_HOME. 기본 0 = 제한 없음
//   - FAULT 원인: 실행기 Fault = 제어 끊김 · 감시 회복 실패 · HOME 실패 · DRIVE 복구 안 됨 · 재고 불확실, 경기장 파일 문제, 미등록 바코드, dice 계획 불가
//   - FAULT 뒤 이어 하기 (resume_continue_task): FAULT 때 하던 단계 · 남은 박스를 적고 task 를 그대로 둠 (dice_steps resume_point)
//       resume → start → HOME → COLLECT 지금 박스부터 다시 이동 → LOOK → Plan / DELIVER plan_from_state / VERIFY 지금 장소부터
//       재고는 실행기 기록 그대로 (적재함 · skipped · 버린 장소 · 놓은 수), 재고 불확실은 /mission/inventory_* 정합 뒤에만 resume
//       같은 원인 · 같은 단계 FAULT 가 그 단계 성공 없이 또 나면 그 퍽 · 장소 · 박스 빼고 (resume_skip), 재계획 횟수 max_replans 에 셈
//       이어 하지 않음: 끔 · dice 계획 불가 · 경기장 파일 · 미등록 바코드 · task 전 (바코드 · 신호) · 복귀 중 → task 포기
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
#include <thread>
#include <functional>
#include <vector>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "interfaces/msg/detection_array.hpp"
#include "interfaces/msg/mission_status.hpp"
#include "interfaces/msg/motion_mode.hpp"
#include "interfaces/msg/signal_record.hpp"
#include "interfaces/msg/task_text.hpp"
#include "interfaces/msg/traffic_light.hpp"
#include "interfaces/msg/verify_record.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "planning/executor.hpp"

#include "mission/barcode_search.hpp"
#include "mission/dice.hpp"
#include "mission/dice_map.hpp"
#include "mission/dice_record.hpp"
#include "mission/lying_backoff.hpp"
#include "mission/place_box.hpp"
#include "mission/dice_steps.hpp"
#include "mission/dice_verify.hpp"
#include "mission/hand_state.hpp"
#include "mission/signal_recorder.hpp"
#include "mission/task_parser.hpp"
#include "mission/transfer_planner.hpp"
#include "mission/verify_image.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2_ros/buffer.hpp>

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

std::string join(const std::vector<std::string> & v, const std::string & sep = ", ")
{
  std::string out;
  for (const auto & x : v) {out += (out.empty() ? "" : sep) + x;}
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
        if (s.type == StepType::Arm && s.arm_task == kArmLookAtBc) {return bc_resolve(out, detail);}   // 바코드 찾기 LOOK_AT
        if (is_putback(s)) {return putback_target(s, out, detail);}
        return dice_cell_target(s, out, detail);
      });
    // perception 이 다시 뜸 (멈춤 감시 → respawn): 실행기는 손목 검출 · 박스 켜기를 다시 보냄, 미션은 바코드 판독 · 광각
    //   (perception 도 같은 launch 면 켜기 상태를 되살리지만, 아는 값과 다를 수 있는 경우를 없앰)
    exec_->set_perception_restart_hook([this](const std::string &, const std::string &) {
        barcode_known_ = false;
        barcode_last_try_s_ = -1e9;
        wide_.known = false;
        wide_.last_try_s = -1e9;
        RCLCPP_WARN(get_logger(), "perception 다시 켜짐 → 바코드 판독 · 광각 켜기도 지금 원하는 값 (바코드 %s · 광각 %s) 으로 다시 보냄",
          barcode_want_ ? "켬" : "끔", wide_.on ? "켬" : "끔");
      });
    exec_->set_arm_sent_hook([this](const Step & s, double now) {
        if (s.arm_task == kArmLookAtBc) {   // 거절 이유 (/control/motion_mode) 를 이 뒤 것만 봄 · 팔 펴 둔 시간
          bc_look_sent_s_ = now;
          if (bc_arm_out_since_ < 0.0) {bc_arm_out_since_ = now;}
        }
        if (s.arm_task == kArmHome) {bc_arm_folded(now);}
        if (s.arm_task == kArmRetrieve && !s.puck_id.empty()) {retrieved_here_.insert(s.puck_id);}
        if (s.arm_task == kArmPick && !s.puck_id.empty()) {   // 되돌림 자리 = 집기 전 기록부 자리 (집은 뒤엔 쥔 퍽 검출이 덮어씀)
          retrieved_here_.erase(s.puck_id);
          if (const auto rec = exec_->registry().find(s.puck_id)) {pick_points_[s.puck_id] = PickPoint{rec->x, rec->y, nav_ok_total_};}
        }
        for (auto & ws : look_windows_) {   // 팔이 움직이기 시작 → 서 있던 LOOK 자세 구간 끝 (puck_views)
          if (!ws.empty() && ws.back().to < 0.0) {ws.back().to = now;}
        }
        if (reconcile_.active && s.name == reconcile_.look_name) {reconcile_.look_s = now;}   // 자동 정합 확인 LOOK: 이 뒤 검출만 씀
        if (!is_look_task(s.arm_task)) {return;}
        last_look_sent_s_ = now;   // perception 멈춤: 이 뒤 검출 끊김 = LOOK 결과가 빔 (gap_guard)
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
          if (phase_ == Phase::VERIFY) {verify_frame_.reset();}   // 이 장소 LOOK 뒤 장만 (앞 장소 장이 남지 않게)
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
    //   (박스 끝 퍽이 가운데 LOOK 화면 밖, B2). 각 자세 look_dwell_s 뒤 앞 · 옆에서 본 퍽을 합쳐 계획. 정차마다 한 번
    side_cfg_.enabled = declare_parameter<bool>("side_look_missing", true);
    side_cfg_.expected = static_cast<int>(declare_parameter<int>("side_look_expected", 2));
    //   기본 left · near: 앞 + left 로 right 쪽까지 거의 다 보임 (화각 계산), near (J3 +look_near_tilt_deg) 가 가까운 줄
    side_cfg_.order = declare_parameter<std::vector<std::string>>("side_look_order", {"left", "near"});
    for (const auto & o : side_cfg_.order) {
      if (!is_side_slot(o)) {RCLCPP_WARN(get_logger(), "side_look_order 의 '%s' 는 안 씀 (right · left · near · far 만)", o.c_str());}
    }
    RCLCPP_INFO(get_logger(), "옆 보기 (side_look_missing): %s, 박스 퍽 < %d 개면 %s", side_cfg_.enabled ? "켬" : "끔",
      side_cfg_.expected, join(side_cfg_.order).c_str());
    // PICK 앞 다시 보기: 손목이 그 퍽을 안 보는 자세에서 시작할 PICK (같은 박스 둘째 퍽부터 = STORE 뒤 P0, 그 퍽이 없는 옆 보기 자세) 앞에
    //   LOOK 다시 → control 이 방금 검출에 맞춰 집음 (LOOK 때 위치로 집으면 빈손이 잦음). 옆에서만 본 퍽은 그 slot 으로
    //   다시 본 뒤 그 퍽 새 검출이 오면 바로, relook_dwell_s 안에 안 오면 앞 검출로 그대로 집음 (막지 않음)
    relook_before_pick_ = declare_parameter<bool>("relook_before_pick", true);
    relook_dwell_s_ = declare_parameter<double>("relook_dwell_s", 0.5);
    // 박스 정밀 접근 실패 (박스가 지도 자리에 없음 등) → 빼기 전에 한 번: 정차점 box_relook_back_m 뒤에서 LOOK → box_relook_dwell_s 동안
    //   박스 중심 box_relook_radius_m 안에서 본 퍽 평균 = 새 박스 중심 (box_relook_max_shift_m 넘게 멀면 안 씀) → 같은 방향 · 거리로 새 정차점
    box_relook_ = declare_parameter<bool>("box_relook", false);
    box_relook_back_m_ = declare_parameter<double>("box_relook_back_m", 0.12);
    box_relook_dwell_s_ = declare_parameter<double>("box_relook_dwell_s", 1.5);
    box_relook_radius_m_ = declare_parameter<double>("box_relook_radius_m", 0.25);
    box_relook_max_shift_m_ = declare_parameter<double>("box_relook_max_shift_m", 0.30);
    // 박스 다시 찾기 넓힘: 다시 찾기 LOOK 동안 손목 깊이 박스 측정 (box_topic) 을 먼저 씀 → 중심 · 방향
    //   → 네 면 중 로봇 쪽 면부터 지도 (box_relook_map_topic) 벽 · 다른 박스에 몸이 안 닿는 면으로 새 정차점 (막힌 면뿐이면 가장 덜 막힌 면)
    //   측정 · 퍽 둘 다 없으면 box_relook_side_looks 자세로 하나씩 더 LOOK 뒤 다시 잼 (다 못 보면 그 박스 · 장소 뺌)
    //   box_relook_places: 장소 (L1–L3) 정밀 접근 실패에도 같은 다시 찾기 (DELIVER)
    box_relook_use_box_detector_ = declare_parameter<bool>("box_relook_use_box_detector", true);
    box_relook_side_looks_ = declare_parameter<std::vector<std::string>>("box_relook_side_looks", {"left", "right", "near"});
    box_relook_places_ = declare_parameter<bool>("box_relook_places", true);
    box_relook_map_topic_ = declare_parameter<std::string>("box_relook_map_topic", "/map");
    box_relook_body_ = declare_parameter<std::vector<double>>("box_relook_body_m", {0.16, 0.16, 0.03});   // 반 길이 · 반 폭 · 여유
    for (const auto & o : box_relook_side_looks_) {
      if (!is_side_slot(o)) {RCLCPP_WARN(get_logger(), "box_relook_side_looks 의 '%s' 는 안 씀 (right · left · near · far 만)", o.c_str());}
    }
    if (box_relook_body_.size() != 3) {box_relook_body_ = {0.16, 0.16, 0.03};}
    // 박스 LOOK 뒤 (정밀 접근 성공해도): 지도 중심 box_radius 밖으로 빠진 퍽이 있으면 box_relook_radius_m 안 퍽 평균으로 박스 중심 다시 잡음
    //   (실물 박스가 지도에서 6–11 cm 어긋나 "LOOK: 퍽 0 개" 가 나던 것). box_relook_max_shift_m 넘게 옮기진 않음
    box_recenter_on_look_ = declare_parameter<bool>("box_recenter_on_look", true);
    // 박스 정차 계획에서 이 정차에 집을 누운 퍽 중심 x (base_link, 손목 깊이) < lying_backoff_x_m 이면
    //   (선 퍽 · 방향 모름은 < lying_backoff_upright_x_m, 0 = 안 봄)
    //   그 자리에서 안 집고 [HOME → 곧게 뒤로 (target_x − x, 최대 max) → 같은 박스 앞 LOOK → 같은 Plan] (박스마다 한 번).
    //   뒤로는 Nav2 · 정밀 접근 없이 odom 보며 cmd_vel_nav (collision_monitor 거침, 그동안 lying_backoff_polygons 만 끔).
    //   못 가면 (DRIVE 안 옴 · 막힘 · TF 없음) 간 만큼에서 LOOK · 계획 → 그대로 집음 (새 FAULT · 거절 없음)
    lying_cfg_.near_x_m = declare_parameter<double>("lying_backoff_x_m", lying_cfg_.near_x_m);
    lying_cfg_.up_near_x_m = declare_parameter<double>("lying_backoff_upright_x_m", lying_cfg_.up_near_x_m);
    lying_cfg_.target_x_m = declare_parameter<double>("lying_backoff_target_x_m", lying_cfg_.target_x_m);
    lying_cfg_.max_m = declare_parameter<double>("lying_backoff_max_m", lying_cfg_.max_m);
    lying_unseen_s_ = declare_parameter<double>("lying_backoff_unseen_s", lying_unseen_s_);
    lying_move_.v_max = declare_parameter<double>("lying_backoff_speed_mps", lying_move_.v_max);
    lying_drive_wait_s_ = declare_parameter<double>("lying_backoff_drive_wait_s", lying_drive_wait_s_);
    lying_cmd_topic_ = declare_parameter<std::string>("lying_backoff_cmd_topic", lying_cmd_topic_);
    lying_cm_node_ = declare_parameter<std::string>("lying_backoff_cm_node", lying_cm_node_);
    lying_polygons_ = declare_parameter<std::vector<std::string>>("lying_backoff_polygons", lying_polygons_);
    if (lying_polygons_.size() == 1 && lying_polygons_[0].empty()) {lying_polygons_.clear();}
    RCLCPP_INFO(get_logger(), "가까운 퍽 뒤로 빼기: 누운 x < %.3f · 선 x < %.3f m → 뒤로 (%.3f − x, 최대 %.3f m, %.2f m/s, %s, 끄는 영역 %zu 개)",
      lying_cfg_.near_x_m, lying_cfg_.up_near_x_m, lying_cfg_.target_x_m, lying_cfg_.max_m, lying_move_.v_max, lying_cmd_topic_.c_str(), lying_polygons_.size());
    RCLCPP_INFO(get_logger(), "PICK 앞 다시 보기 (relook_before_pick): %s, 새 검출 최대 %.1f s", relook_before_pick_ ? "켬" : "끔",
      relook_dwell_s_);
    // 정차 중 PICK: 차체가 서 있으면 LOOK 때 본 odom 기준 위치가 그대로 맞음 → STORE 뒤에도 씀
    dice_pick_fresh_s_ = declare_parameter<double>("dice_pick_fresh_s", 90.0);
    transfer_max_remaining_ = declare_parameter<int>("transfer_max_remaining", 4);
    // 퍽이 모자라 일부 장소만 채울 때: 채우는 장소 수가 같으면 놓는 퍽이 많은 쪽 먼저 (false = 시간 짧은 쪽)
    alloc_prefer_more_pucks_ = declare_parameter<bool>("alloc_prefer_more_pucks", false);
    // 부분 채우기: 퍽이 모자라도 장소를 통째로 버리지 않고 장소마다 놓을 수 있는 만큼 (놓는 퍽 수 최대). 덜 채운 장소는 뒤 (가까운) 칸이 빔
    //   true 면 alloc_prefer_more_pucks 는 안 씀 (항상 퍽 수 먼저). false = 끝까지 채울 수 있는 장소만
    fill_partial_places_ = declare_parameter<bool>("fill_partial_places", false);
    // 재검증 (발제 '이동 후 검증'): 장소 반경 안 새 관측으로 칸 · 그룹 판정 → /planning/verify_record + 기록 파일
    verify_radius_m_ = declare_parameter<double>("verify_radius_m", 0.18);
    verify_cell_tol_m_ = declare_parameter<double>("verify_cell_tol_m", 0.05);   // 눈 위치 ±5 cm 면 성공
    // 재검증 퍽 색 · 모양 = LOOK 동안 읽힌 장 다수결, 1 등 비율이 이것보다 작으면 모름 (추정 안 함)
    verify_vote_ratio_ = declare_parameter<double>("verify_vote_ratio", 0.7);
    // 재검증 모양 맞춤: 박스 측정이 없을 때만, 관측 퍽에 눈 모양을 지도 칸에서 이 범위 안으로 옮기고 돌려 맞춘 뒤 칸 판정
    //   박스 측정이 있으면 그 박스 중심 기준 칸 그대로 (맞춤이 놓기 치우침을 숨기지 않게)
    verify_fit_.enable = declare_parameter<bool>("verify_fit", true);
    verify_fit_.max_shift_m = declare_parameter<double>("verify_fit_max_shift_m", 0.10);
    verify_fit_.max_rot_rad = declare_parameter<double>("verify_fit_max_rot_deg", 180.0) * M_PI / 180.0;   // 180 = 방향 무관
    // 박스 위 확인: 퍽 중심이 잰 윗면 (place_box_half_m) 가장자리에서 이만큼 안쪽이어야 함 (0 = 중심이 윗면 안 = 안 떨어짐)
    verify_on_box_margin_m_ = declare_parameter<double>("verify_on_box_margin_m", 0.0);
    // 재검증 LOOK 자세: "far" = control look_far_tilt_deg 만큼 먼 쪽 (먼 줄 퍽이 사진 위 끝에서 안 잘리게). "" = 정면
    verify_look_slot_ = declare_parameter<std::string>("verify_look_slot", "far");
    if (!verify_look_slot_.empty() && !is_side_slot(verify_look_slot_)) {
      RCLCPP_WARN(get_logger(), "verify_look_slot '%s' 는 안 씀 (right · left · near · far 만) → 정면", verify_look_slot_.c_str());
      verify_look_slot_.clear();
    }
    // 재검증에서 로봇 자기 몸 안 검출 뺌 (적재함 · 받침의 퍽을 장소 퍽으로 세지 않게)
    //   촬영 시각 base 프레임에서 차체 상자 (navigation chassis_x · chassis_half_y) + 여유 안이면 셈 안 함
    {
      const auto bx = declare_parameter<std::vector<double>>("verify_body_x", {-0.133, 0.141});
      if (bx.size() == 2) {verify_body_x_ = {bx[0], bx[1]};}
      verify_body_half_y_ = declare_parameter<double>("verify_body_half_y", 0.150);
      verify_body_margin_m_ = declare_parameter<double>("verify_body_margin_m", 0.03);
    }
    // 박스 밖 거르기: 퍽 중심이 잰 윗면 가장자리에서 이만큼 안쪽이 아닌 검출은 판정 · 개수에서 뺌 (박스 모서리 · 로봇 몸 오검출). 0 = 안 뺌
    verify_ignore_inset_m_ = declare_parameter<double>("verify_ignore_inset_m", 0.015);
    // 깊이 틀린 장: z 가 이 장소 장들 z 중앙값 (윗면 높이) 에서 이만큼 넘게 벗어난 장은 위치로 안 씀
    //   (같은 퍽의 맞는 장 위치, 묶기는 광선 · 윗면 교점)
    verify_plane_tol_m_ = declare_parameter<double>("verify_plane_tol_m", 0.03);
    // 칸 ID 판정: 칸마다 놓은 퍽 ID 가 그 칸 verify_cell_tol_m 안에 보여야 함. false = 그룹 조건 (색 · 모양) 판정
    verify_cell_ids_ = declare_parameter<bool>("verify_cell_ids", true);
    // 재검증 그림: VERIFY LOOK 동안만 손목 컬러 영상 구독 → 판정 때 마지막 장에 판정을 그려 발행 · 저장. "" = 안 함
    verify_image_topic_ = declare_parameter<std::string>("verify_image_topic", "/planning/verify_image");
    verify_color_topic_ = declare_parameter<std::string>("verify_color_topic", "/camera/camera/color/image_raw");
    verify_info_topic_ = declare_parameter<std::string>("verify_info_topic", "/camera/camera/color/camera_info");
    verify_camera_frame_ = declare_parameter<std::string>("verify_camera_frame", "d435_color_optical_frame");
    // 기록 파일 (발제 제출 '기록'): <record_dir>/run_<시각>/ signal.json · allocation.json · verify.yaml. "" = 안 씀
    record_dir_ = declare_parameter<std::string>("record_dir", "~/planning_logs");
    // 재계획: 단계가 재시도 뒤에도 실패하면 그 퍽 · 장소를 빼고 지금 상태에서 다시 계획 (이 횟수 넘으면 포기 · 복귀)
    max_replans_ = declare_parameter<int>("max_replans", 6);
    // DELIVER 중 같은 장소로 가는 이동이 이 번 (단계 재시도 뒤) 실패하면 그 장소를 버리고 남은 장소부터 채움
    //   (같은 장소로 계속 다시 계획하면 max_replans 를 다 써서 다른 장소도 못 채움). 0 = 끔
    place_nav_fail_n_ = declare_parameter<int>("place_nav_fail_n", 2);
    // FAULT 뒤 task 이어 하기: resume → start 뒤 HOME → 지금 재고로 그 단계 (COLLECT 남은 박스 · DELIVER 재계획 · VERIFY 남은 장소) 부터
    //   같은 원인 · 같은 단계 FAULT 가 또 나면 그 퍽 · 장소 · 박스를 빼고 (재계획 횟수 max_replans 에 셈). false = task 포기 · 복귀
    //   FAULT 때마다 읽음 → 실행 중 바꿔도 다음 FAULT 부터
    declare_parameter<bool>("resume_continue_task", true);
    // STORE 박스 되돌림 (STORE_RETURNED_TO_BOX) 이 같은 퍽에 이 번 나오면 그 퍽 건너뜀 (그 전엔 다시 집어 넣기)
    store_return_skip_n_ = declare_parameter<int>("store_return_skip_n", 2);
    // 쥔 퍽 STORE 가 재시도 뒤에도 실패 (거절 · 보내기 전 실패) → 다시 계획 때 그 칸을 빼고 다른 빈 칸 (store_ok) 으로, 실패한 칸이
    //   이 수보다 많아지거나 다른 칸이 없으면 집은 박스 자리에 되돌려 놓고 (PUTBACK) 그 퍽 건너뜀. 0 = 같은 칸 다시
    store_alt_cells_ = declare_parameter<int>("store_alt_cells", 1);
    // PICK 빈손 (EMPTY_GRASP) 이 같은 퍽에 이 번 나오면 그 퍽 건너뜀 (그 전엔 HOME → 앞 LOOK 다시 → 다시 집기). 1 = 첫 빈손에 건너뜀
    empty_grasp_skip_n_ = declare_parameter<int>("empty_grasp_skip_n", 2);
    // RETRIEVE 빈손 뒤 적재함 스캔에서 퍽이 그 칸에 남았으면 다시 꺼냄, 같은 퍽에 이 번 나오면 그 퍽은 칸에 둔 채 건너뜀 (나머지로 계속)
    retrieve_empty_skip_n_ = declare_parameter<int>("retrieve_empty_skip_n", 2);
    // 박스 이동 (Nav2 · 정밀 접근) 이 재시도까지 실패하면 그 박스를 버리지 않고 남은 박스 뒤에 한 번 더 (사람이 막음 · 박스 못 봄이 풀릴 수 있음)
    box_retry_later_ = declare_parameter<bool>("box_retry_later", true);
    // 시작 · 이어 하기 뒤 이동이 한 번도 성공하기 전에 이동이 재시도까지 실패 = 주행 계통이 반쯤 뜬 것 (planner 지도 없음 · bond 등)
    //   → 박스를 줄줄이 버리지 않고 FAULT NAV_NOT_WORKING (사람이 주행 확인 → resume → start 가 그 박스부터 이어 감). false = 끔
    nav_fault_before_first_ = declare_parameter<bool>("nav_fault_before_first", true);
    // 첫 이동 FAULT (NAV_NOT_WORKING) 뒤 사람 없이 이어 감: fault_auto_wait_s 뒤 스스로 resume → 준비되면 start
    //   (HOME → 같은 task 이어 하기). 1 번째 = 같은 목표 다시, 2 번째부터 = 첫 이동 FAULT 를 이 task 에선 안 냄 → 보통 이동 실패 처리
    //   (box_retry_later · 박스 건너뜀 · 배달 · 복귀). task 마다 nav_fault_auto_n 번까지. 0 = 사람이 resume · start
    nav_fault_auto_n_ = static_cast<int>(declare_parameter<int>("nav_fault_auto_n", 2));
    fault_auto_wait_s_ = declare_parameter<double>("fault_auto_wait_s", 5.0);
    // 재고 불확실 (INVENTORY_UNCERTAIN) 스스로 정합: fault_auto_wait_s 뒤 그리퍼 실측 (hand_topic 의 hand_joint) 으로
    //   손 상태 (쥠 · 빈손) → 원인 팔 작업별 inventory before · done · removed 하나 → resume → start (쥐었으면 HOME 대신 손 비우기부터).
    //   손 상태가 inventory_auto_timeout_s 안에 안 정해지면 사람 확인. task 마다 inventory_auto_n 번까지, 0 = 늘 사람 확인
    inventory_auto_n_ = static_cast<int>(declare_parameter<int>("inventory_auto_n", 2));
    inventory_auto_timeout_s_ = declare_parameter<double>("inventory_auto_timeout_s", 15.0);
    hand_topic_ = declare_parameter<std::string>("hand_topic", "/joint_states");
    hand_joint_ = declare_parameter<std::string>("hand_joint", "gripper_left_joint");
    //   손가락 사이 = 2 · (hand_inner0_m + q) · 1000 + hand_gap_bias_mm (control grasp_guard 와 같은 식 · 값, control.yaml 과 같게)
    hand_cfg_.inner0_m = declare_parameter<double>("hand_inner0_m", hand_cfg_.inner0_m);
    hand_cfg_.gap_bias_mm = declare_parameter<double>("hand_gap_bias_mm", hand_cfg_.gap_bias_mm);
    hand_cfg_.empty_q_m = declare_parameter<double>("hand_empty_q_m", hand_cfg_.empty_q_m);
    hand_cfg_.cylinder_mm = declare_parameter<double>("hand_width_cylinder_mm", hand_cfg_.cylinder_mm);
    hand_cfg_.square_mm = declare_parameter<double>("hand_width_square_mm", hand_cfg_.square_mm);
    hand_cfg_.tol_mm = declare_parameter<double>("hand_width_tol_mm", hand_cfg_.tol_mm);
    hand_cfg_.open_extra_mm = declare_parameter<double>("hand_open_extra_mm", hand_cfg_.open_extra_mm);
    hand_cfg_.stable_s = declare_parameter<double>("hand_stable_s", hand_cfg_.stable_s);
    hand_cfg_.fresh_s = declare_parameter<double>("hand_fresh_s", hand_cfg_.fresh_s);
    // 바코드는 손목으로 읽음: READ_BARCODE = 이동 → LOOK_BARCODE → 읽기 → HOME
    barcode_look_ = declare_parameter<bool>("barcode_look", true);
    // 적재함: STORE 앞에 LOOK_RACK (퍽 쥔 채 그 칸을 손목으로 봄 → perception 칸 측정 → STORE target)
    rack_look_ = declare_parameter<bool>("rack_look", false);
    // 재고 자동 정합 (실행기 auto_reconcile): 확인 LOOK 뒤 검출을 기다리는 최대 시간 · 장소 칸 허용 거리 · 적재함 태그 토픽
    reconcile_wait_s_ = declare_parameter<double>("reconcile_wait_s", 1.5);
    reconcile_place_tol_m_ = declare_parameter<double>("reconcile_place_tol_m", 0.03);
    // RETRIEVE 빈손 자동 정합: HOME 없이 적재함 스캔 (팔 = 박스 위 기준점, control rack_scan_from_retrieve_end 가 그 자리에서 이음)
    //   스캔이 거절 · 실패하면 한 번만 HOME 뒤 다시 스캔. false = HOME 먼저
    reconcile_scan_skip_home_ = declare_parameter<bool>("reconcile_scan_skip_home", true);
    // RETRIEVE 빈손 자동 정합 확인 = 그 칸만 빈손 LOOK_RACK (control look_rack_empty: 기준점 → 그 칸 관찰 자세 → 기준점, 적재함 맞춤 안 함) → HOME.
    //   false = 빈손 적재함 스캔 (LOOK slot rack, control 이 그 스캔으로 적재함 맞춤을 다시 함)
    reconcile_look_rack_ = declare_parameter<bool>("reconcile_look_rack", true);
    // RETRIEVE 가 control 에 계속 거절됨 (팔 안 움직임, 예: 적재함 맞춤이 그 칸을 한도 넘게 옮김) 이 같은 퍽에 이 번 나오면
    //   그 퍽은 칸에 둔 채 건너뜀 (나머지로 계속). 0 = 재고 그대로 다시 계획 (같은 RETRIEVE 반복)
    retrieve_reject_skip_n_ = declare_parameter<int>("retrieve_reject_skip_n", 1);
    rack_tags_topic_ = declare_parameter<std::string>("rack_tags_topic", "/rack_tags/detections");
    // 장소 박스 측정 (손목 깊이): LOOK 뒤 측정 중앙값으로 칸 기준을 고침. 못 받거나 지도 중심에서 box_max_shift_m 넘게 멀면 지도 값
    box_measure_ = declare_parameter<bool>("box_measure", true);
    box_topic_ = declare_parameter<std::string>("box_topic", "/perception/wrist/box");
    box_min_samples_ = declare_parameter<int>("box_min_samples", 3);
    // 지도 중심에서 이보다 먼 측정은 버림 (다른 물체 · 잘못 맞춤). 0.06 → 0.15 (박스가 6 cm 넘게 밀려도 측정으로 놓음, 모의 f4c)
    box_max_shift_m_ = declare_parameter<double>("box_max_shift_m", 0.15);
    // 다시 붙기: 측정 박스가 이 정차에서 기대한 자리와 box_realign_shift_m 또는 box_realign_yaw_deg 넘게 다르면
    //   측정 박스에 맞춘 정차점 (지도 정차와 같은 거리 · 방향) 으로 다시 이동 (정밀 접근) → LOOK → 다시 측정 → 그 박스 기준으로 놓음
    //   장소마다 task 당 box_realign_max 번 (0 = 끔). 그 뒤에도 다르면 측정 기준으로 그대로 놓음 (거절 · FAULT 없음)
    box_realign_shift_m_ = declare_parameter<double>("box_realign_shift_m", 0.04);
    box_realign_yaw_rad_ = declare_parameter<double>("box_realign_yaw_deg", 8.0) * M_PI / 180.0;
    box_realign_max_ = declare_parameter<int>("box_realign_max", 1);
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

    // 신호등 점멸 기록 (발제). 값 설명은 signal_recorder.hpp
    signal_timeout_s_ = declare_parameter<double>("signal_timeout_s", 1.0);
    signal_record_ = declare_parameter<bool>("signal_record", true);
    signal_wait_s_ = declare_parameter<double>("signal_wait_s", 90.0);   // OBSERVE_SIGNAL 최대 대기 (넘으면 불완전, 다시 기록할 때마다 새로 셈)
    // 바코드 먼저: 바코드를 읽기 전에는 신호를 기록하지 않음 (몸 멈춤 · 신호등 우선 접기 없음). 읽은 뒤 OBSERVE_SIGNAL 에서 기록
    //   barcode_first_wide false = 그동안 광각도 끔 (바코드 찾기는 벽 판 · 라이다 · 손목만). barcode_first false = 신호 우선
    barcode_first_ = declare_parameter<bool>("barcode_first", true);
    barcode_first_wide_ = declare_parameter<bool>("barcode_first_wide", false);
    // 바코드를 읽은 뒤 신호 기록은 출발 자리에서 signal_back_m 만큼 뒤로 물러난 자리에서 (가까우면 광각이 신호등을 밑에서 올려다봐 안 보임).
    //   바코드 찍으러 몸을 옮겼든 아니든 그 자리로 감. signal_return_start false = 읽은 자리에서 기다림
    signal_return_start_ = declare_parameter<bool>("signal_return_start", true);
    signal_back_m_ = declare_parameter<double>("signal_back_m", 0.20);
    lying_forward_ = declare_parameter<bool>("lying_backoff_forward", true);
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
    // 벽 판: 캡처를 끈 뒤 다시 켜면 v4l2 밝기를 다시 넣음 (usb_cam 은 brightness −1 = 안 건드림, 밝기는 bringup 이 시작 3 s 뒤 v4l2-ctl 로
    //   한 번만 넣음 → 장치가 기본 +50 으로 돌아가도 아무도 다시 안 넣음). 장치는 bringup 처럼 by-id 를 실제 장치로 풂. "" 면 안 함
    wide_device_ = declare_parameter<std::string>("wide_device",
      "/dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._USB_2.0_Camera_SN0001-video-index0");
    wide_brightness_ = static_cast<int>(declare_parameter<int>("wide_brightness", -20));   // bringup wide_brightness 와 같게 (−64~64)

    // 경기장 좌표 · 이동 시간표 · 바코드 표
    std::string err;
    map_ok_ = load_dice_map(dice_map_path_, dice_map_, &err) && load_transfer_costs(travel_table_path_, costs_, &err) &&
      load_barcode_table(barcode_table_path_, barcodes_, &err);
    dice_map_file_ = dice_map_;   // 파일 값 그대로 (다시 찾기 정차 거리 = 파일 정차 ↔ 중심, 메모리 dice_map_ 은 다시 잡기로 바뀜)
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
    if (inventory_auto_n_ > 0 && !hand_topic_.empty()) {   // 그리퍼 실측 (재고 불확실 스스로 정합)
      sub_hand_ = create_subscription<sensor_msgs::msg::JointState>(hand_topic_, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::JointState::ConstSharedPtr m) {
          for (std::size_t i = 0; i < m->name.size() && i < m->position.size(); ++i) {
            if (m->name[i] != hand_joint_) {continue;}
            const double t = now_s();
            hand_samples_.push_back(HandSample{t, m->position[i]});
            while (!hand_samples_.empty() && t - hand_samples_.front().t > 3.0) {hand_samples_.pop_front();}
          }
        });
      // 실행기 자동 정합 (auto_reconcile): 재고 작업 실패 순간 손 상태 = 같은 판정 (빈손이면 FAULT 없이 카메라 확인)
      exec_->set_hand_reader([this](double now, std::string & why) {
          const HandReading h = read_hand(hand_samples_, now, hand_cfg_);
          why = h.text;
          return h.state == HandState::Empty ? planning::HandGuess::Empty :
          h.state == HandState::Holding ? planning::HandGuess::Holding : planning::HandGuess::Unknown;
        });
    }
    sub_text_ = create_subscription<TaskText>(
      "/perception/task_text", 10, [this](TaskText::ConstSharedPtr m) {
        last_text_ = m->raw_text;
        last_text_s_ = now_s();
      });
    declare_bc();   // 바코드 찾기: 파라미터 · 구독

    // 적재함 칸 태그 · 칸 안 퍽 태그 (control_master rack_tag_stage): 자동 정합 적재함 확인에만 씀
    if (!rack_tags_topic_.empty()) {
      sub_rack_tags_ = create_subscription<interfaces::msg::DetectionArray>(
        rack_tags_topic_, 10, [this](interfaces::msg::DetectionArray::ConstSharedPtr m) {
          const double t = now_s();
          for (const auto & d : m->detections) {
            if (d.kind == "rack_tag" || d.kind == "rack_puck") {rack_seen_.push_back({t, d.kind == "rack_puck", d.label, d.id});}
          }
          while (!rack_seen_.empty() && t - rack_seen_.front().t > 60.0) {rack_seen_.pop_front();}
        });
    }
    if (lying_cfg_.near_x_m > 0.0 || lying_cfg_.up_near_x_m > 0.0) {   // 곧게 뒤로: 명령 · collision_monitor 영역 끄기 · 20 Hz 명령 주기
      lying_cmd_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(lying_cmd_topic_, 10);
      if (!lying_cm_node_.empty() && !lying_polygons_.empty()) {
        lying_cm_cli_ = create_client<rcl_interfaces::srv::SetParameters>(lying_cm_node_ + "/set_parameters");
      }
      lying_timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() {lying_move_tick();});
    }
    if (box_measure_ || (box_relook_ && box_relook_use_box_detector_)) {
      sub_box_ = create_subscription<interfaces::msg::DetectionArray>(
        box_topic_, 10, [this](interfaces::msg::DetectionArray::ConstSharedPtr m) {on_box(*m);});
    }

    if (box_relook_ && !box_relook_map_topic_.empty()) {   // 다시 찾기 면 고르기용 지도 (map_server · transient local)
      sub_map_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        box_relook_map_topic_, rclcpp::QoS(1).transient_local().reliable(),
        [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m) {
          OccGrid g;
          g.res = m->info.resolution;
          g.ox = m->info.origin.position.x;
          g.oy = m->info.origin.position.y;
          g.w = static_cast<int>(m->info.width);
          g.h = static_cast<int>(m->info.height);
          g.data.assign(m->data.begin(), m->data.end());
          relook_map_ = std::move(g);
        });
    }

    // ---------------------------------------------------------------- 보내기
    pub_status_ = create_publisher<MissionStatus>("/mission/status", 10);
    pub_hold_ = create_publisher<std_msgs::msg::Bool>("/mission/hold", 10);
    pub_signal_ = create_publisher<SignalRecord>("/planning/signal_record", rclcpp::QoS(1).reliable().transient_local());
    pub_verify_ = create_publisher<VerifyRecord>("/planning/verify_record", rclcpp::QoS(1).reliable().transient_local());
    if (!verify_image_topic_.empty()) {   // 장소 세 곳 그림이 늦게 붙은 구독자에게도 가게
      pub_verify_image_ = create_publisher<sensor_msgs::msg::Image>(verify_image_topic_, rclcpp::QoS(3).reliable().transient_local());
    }
    if (!barcode_enable_service_.empty()) {barcode_cli_ = create_client<SetBool>(barcode_enable_service_);}
    if (!wide_enable_service_.empty()) {wide_.cli = create_client<SetBool>(wide_enable_service_);}
    if (!wide_capture_service_.empty()) {capture_.cli = create_client<SetBool>(wide_capture_service_);}
    wide_.name = "perception 광각";
    capture_.name = "광각 usb_cam 캡처";
    capture_.ok_on = "Start Capturing";   // usb_cam 0.8.1 set_capture 는 처리하고도 success = false, 글만 이렇게 줌 (Jetson 실물 확인)
    capture_.ok_off = "Stop Capturing";
    capture_.on_reenabled = [this]() {reapply_wide_brightness();};

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
    // 사람이 robot_cli 로 직접 한 STORE 기록: /mission/store_done/<칸> = 그리퍼 퍽 → 그 칸 (STORE 성공과 같은 재고 전환).
    //   clear_gripper 는 기록만 지움 (퍽이 칸에 있어도 재고에서 빠짐) — 칸에 넣었으면 이것. RUNNING 중 · 불확실 · 그리퍼 빔 · 찬 칸이면 거절
    for (const auto & [slot, content] : exec_->cargo().slots()) {
      (void)content;
      srv_store_done_.push_back(create_service<Trigger>(
        "/mission/store_done/" + slot, [this, slot = slot](const std::shared_ptr<Trigger::Request>,
        std::shared_ptr<Trigger::Response> res) {
          if (op_ == Op::RUNNING) {
            res->success = false;
            res->message = "RUNNING 중에는 안 됨";
          } else {
            res->success = exec_->record_store(slot, res->message);
          }
          RCLCPP_INFO(get_logger(), "/mission/store_done/%s: %s", slot.c_str(), res->message.c_str());
        }));
    }
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
            planning::CargoDoubt d;
            res->success = exec_->resolve_inventory(fix, res->message, &d);
            if (res->success && place_confirmed_by(fix, d.arm_task)) {on_place_confirmed(d);}
            if (res->success) {res->message += gripper_hint();}
          }
          RCLCPP_INFO(get_logger(), "%s: %s", name.c_str(), res->message.c_str());
        }));
    }
    // 손의 퍽을 사람이 원래 박스 위에 되돌려 놓음 (퍽을 잃지 않고 이어 하기에서 다시 집음)
    srv_inventory_.push_back(create_service<Trigger>(
      "/mission/inventory_returned", [this](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> res) {
        if (op_ == Op::RUNNING) {
          res->success = false;
          res->message = "RUNNING 중에는 안 됨";
        } else {
          std::string id;
          res->success = exec_->return_gripper_puck(res->message, &id);
          if (res->success) {
            const int b = box_of_puck(id);
            res->message += b >= 1 ? " → 기록 자리 B" + std::to_string(b) + " 위 (그 박스에 놓았는지 확인)" :
              " → 기록 자리가 박스 위가 아님 (이 퍽은 계획에 안 들어감)";
          }
        }
        RCLCPP_INFO(get_logger(), "/mission/inventory_returned: %s", res->message.c_str());
      }));
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
    bc_check_reject(now);
    resync_after_perception_restart();
    sync_barcode_reader(now);
    sync_wide_camera(now);
    exec_->update(now);
    if (last_light_s_ < 0.0 || now - last_light_s_ > signal_timeout_s_) {
      record_signal(now, SignalObs::NotSeen, false);   // 토픽 끊김 = 신호등 못 봄
    }
    switch (op_) {
      case Op::STANDBY: tick_standby(now); return;
      case Op::RUNNING: tick_running(now); return;
      case Op::FAULT:
        if (inv_auto_from_ > 0.0 && now >= inv_auto_from_) {inventory_auto(now);}
        if (auto_resume_at_ > 0.0 && now >= auto_resume_at_) {   // 스스로 resume → 준비되면 start
          auto_resume_at_ = -1.0;
          std::string why;
          if (resume_mission(why)) {
            auto_start_once_ = true;
            RCLCPP_WARN(get_logger(), "FAULT 스스로 resume: %s → 준비되면 start", why.c_str());
          } else {
            RCLCPP_WARN(get_logger(), "FAULT 스스로 resume 못 함: %s (사람 확인)", why.c_str());
          }
        }
        return;
      case Op::DONE:
        return;
    }
  }

  // ---------------------------------------------------------------- 시험용 부분 실행 (기본 = 전부 끔 → 시연 흐름 그대로)
  //   signal_fixed  "RGRRGRGGYGYG"  신호등 안 보고 이 12 점멸을 기록한 것으로 (같은 판정 규칙을 거쳐 Done)
  //   barcode_fixed "P2-01"         바코드 지점에 안 가고 이 원문을 읽은 것으로
  //   start_phase   COLLECT | DELIVER | VERIFY   HOME 뒤 이 단계부터 (DELIVER = 지금 재고로 배달 계획, cargo_file + cargo_restore)
  //   stop_after    READ_BARCODE | SIGNAL | COLLECT | DELIVER | VERIFY   이 단계 끝나면 복귀하고 COMPLETED
  //                 SIGNAL = 바코드 · 신호 기록으로 배치 (L1–L3 그룹 · 눈 수) 를 정한 직후, 수집 주행 없음
  //   collect_boxes [1, 2]          collect_order 에서 이 박스만 (순서는 collect_order 그대로)
  //   잘못된 값이면 시작 때 멈춤 (조용히 다른 흐름으로 가지 않게)
  static int phase_rank(const std::string & p)
  {
    if (p == "READ_BARCODE") {return 0;}
    if (p == "SIGNAL") {return 1;}
    if (p == "COLLECT") {return 2;}
    if (p == "DELIVER") {return 3;}
    if (p == "VERIFY") {return 4;}
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
    if (!start_phase_.empty() && (phase_rank(start_phase_) < 2)) {bad.push_back("start_phase 는 COLLECT · DELIVER · VERIFY (지금 " + start_phase_ + ")");}
    if (!stop_after_.empty() && phase_rank(stop_after_) < 0) {bad.push_back("stop_after 는 READ_BARCODE · SIGNAL · COLLECT · DELIVER · VERIFY (지금 " + stop_after_ + ")");}
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
    partial_end(std::string("시험 stop_after ") + what);
  }

  // 시험 부분 실행 끝 — test_return_home false 면 출발점으로 안 감 (벤치에서 지도 자세가 틀려 몸이 끌려가지 않게), 그 자리에서 끝
  void partial_end(const std::string & why)
  {
    if (test_return_home_) {
      start_return(why);
      return;
    }
    RCLCPP_WARN(get_logger(), "%s → 복귀 안 함 (test_return_home false), 그 자리에서 끝", why.c_str());
    abandon_task("시험 끝");
    exec_->cancel_nav();
    detail_ = why;
    exec_->set_steps({});
    finish("COMPLETED");
  }

  // 바코드 task 를 얻은 뒤 (읽음 · barcode_fixed): stop_after READ_BARCODE → 복귀 / 신호 기록 기다림 / 실행
  void after_barcode()
  {
    if (stop_after_ == "READ_BARCODE") {
      RCLCPP_WARN(get_logger(), "시험 stop_after READ_BARCODE: 바코드 %s (%s) → 복귀", pending_task_->raw_text.c_str(),
        dice_order_name(pending_task_->dice_order));
      ++bar_i_;
      partial_end("시험 stop_after READ_BARCODE");
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
    // 재고 정합 결과 퍽을 쥠 → start 는 HOME · 이동 대신 지금 자리 손 비우기부터 (begin_running) → 그리퍼 기록은 막지 않음
    //   (쥔 채 이동은 실행기 CARRY_DRIVE 가 그대로 막음)
    if (hand_first_once_ && resume_.active) {r.erase(std::remove(r.begin(), r.end(), std::string("GRIPPER_RECORD")), r.end());}
    if (signal_.status() != SignalStatus::Done && (last_light_s_ < 0.0 || now - last_light_s_ > signal_timeout_s_)) {
      // 바코드 먼저: 광각을 일부러 껐으면 신호등 토픽 대신 켜기 서비스가 있는지만 봄 (켜는 건 바코드 읽은 뒤)
      if (signal_gate_open()) {
        r.push_back("TRAFFIC_LIGHT");
      } else if (wide_.cli && !wide_.cli->service_is_ready()) {
        r.push_back("WIDE_SERVICE:" + wide_enable_service_);
      }
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
    if ((auto_start_ || auto_start_once_) && not_ready_.empty()) {
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
      why = "준비 안 됨: " + join(r) + gripper_hint();
      return false;
    }
    if (start_s_ < 0.0) {
      start_s_ = now;
      end_s_ = now;
    }
    exec_->reset_monitors();
    detail_.clear();
    nav_ok_since_start_ = 0;
    op_ = Op::RUNNING;
    auto_start_once_ = false;
    RCLCPP_INFO(get_logger(), "미션 시작 (start 승인%s)", resume_return_ ? ", 복귀 재개" : resume_.active ? ", HOME 뒤 task 이어 하기" : "");
    if (barcode_first_ && signal_record_ && signal_fixed_.empty() && signal_.status() != SignalStatus::Done) {
      RCLCPP_INFO(get_logger(), "순서 (barcode_first): ① 바코드 찾기 (신호 기록 전, 몸 · 팔 움직임) → ② 출발 자리로 가서 신호 12 점멸 기록 "
        "→ ③ '신호등 기록 DONE' 뒤에만 수집 출발 (안 끝나면 수집 안 하고 복귀)");
    }
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
    const std::string next = (resume_.active ? resume_text() : resume_return_ ? "start 하면 HOME 뒤 출발점 복귀 다시" : "") +
      gripper_hint();
    detail_ = "복구: 준비 · 재고 확인 뒤 /mission/start" + (next.empty() ? "" : " (" + next + ")");
    RCLCPP_WARN(get_logger(), "FAULT → STANDBY (재고 확인 필요)%s%s", next.empty() ? "" : ": ", next.c_str());
    publish_hold();
    why = "STANDBY 로 전환" + (next.empty() ? "" : ", " + next);
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
    //   퍽을 쥐고 있으면 손 비우기 (다른 칸 STORE · 박스 되돌림) 뒤 복귀 (abandon_dice → start_empty_hand). 비우는 중엔 단계 진행
    if ((phase_ == Phase::READ_BARCODE || task_phase()) && !exec_->arm_busy() && time_short(now) && abandon_after_putback_.empty() &&
      !continue_after_empty_)
    {
      if (task_phase() && exec_->cargo().holding()) {
        abandon_dice("시간 부족");
      } else {
        start_return("시간 부족");
      }
      return;
    }
    // ③ 신호등 점멸 기록 (출발 구역에 선 채, 단계 목록 없음)
    if (phase_ == Phase::OBSERVE_SIGNAL && !exec_->waiting_drive() && exec_->done()) {
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
        after_home(now);   // 이어 하기 재계획이 FAULT 를 정할 수 있음 → 아래 pending_fault_
        break;
      case ExecEvent::Kind::StepOk:
        on_step_ok(ev);
        if (continue_after_empty_ && ev.step.type == StepType::Arm && !exec_->cargo().holding() &&
          (is_putback(ev.step) || ev.step.arm_task == kArmStore))
        {   // 재고 정합 뒤 손 비움 → task 이어 하기 (이동 전 HOME 은 실행기가 끼움)
          continue_after_empty_ = false;
          RCLCPP_WARN(get_logger(), "손 비움 → task 이어 하기");
          continue_task(now_s());
          break;
        }
        if (!abandon_after_putback_.empty() && ev.step.type == StepType::Arm && !exec_->cargo().holding() &&
          (is_putback(ev.step) || ev.step.arm_task == kArmStore))
        {   // 손 비운 뒤 미뤄 둔 포기 · 복귀
          const std::string why = abandon_after_putback_;
          abandon_after_putback_.clear();
          abandon_dice(why);
          break;
        }
        if (ev.all_done) {on_steps_done();}
        break;
      case ExecEvent::Kind::StepFailed:
        detail_ = ev.detail;
        on_step_failed(ev.step, ev.need_home, ev.reconcile);
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
    // 재검증 그림 영상: VERIFY 동안만 구독 (LOOK 마다 새로 구독하면 연결 전에 판정이 끝나 장이 없음)
    if (p == Phase::VERIFY) {start_verify_capture();} else {stop_verify_capture();}
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
    gap_redo_.clear();
    last_look_sent_s_ = -1.0;
    // 재고 불확실 정합 결과 퍽을 쥠 → HOME (control 이 쥔 채 거절) 대신 지금 자리에서 손 비우기 → 그 뒤 task 이어 하기
    if (hand_first_once_ && resume_.active && exec_->cargo().holding()) {
      hand_first_once_ = false;
      phase_ = resume_.phase;
      auto steps = empty_hand_steps("재고 정합 뒤 쥔 퍽");
      if (!steps.empty()) {
        continue_after_empty_ = true;
        set_steps(resume_.phase, std::move(steps));
        return;
      }
    }
    hand_first_once_ = false;
    set_steps(Phase::HOME, {home_step("arm:HOME")});
  }

  // FAULT 중: 재고 불확실을 그리퍼 실측으로 정합 → resume (→ 준비되면 start). 손 상태 모름 · 팔 결과 기다림이면 inv_auto_until_ 까지 다시
  void inventory_auto(double now)
  {
    const auto & d = exec_->cargo().doubt();
    if (!d) {   // 이미 정합됨 (사람 등) → resume 만
      inv_auto_from_ = -1.0;
      std::string why;
      if (resume_mission(why)) {auto_start_once_ = true;}
      RCLCPP_WARN(get_logger(), "재고 불확실 스스로 정합: 표시 없음 → resume (%s)", why.c_str());
      return;
    }
    const HandReading h = read_hand(hand_samples_, now, hand_cfg_);
    const bool released = exec_->last_arm_release_seen();
    std::string why;
    const auto fix = choose_doubt_fix(d->arm_task, h.state, released, &why);
    const std::string evidence = d->step_name + " 퍽 " + d->item.puck_id + (d->slot.empty() ? "" : " 칸 " + d->slot) + ": " + h.text +
      ", 놓기 단계 " + (released ? "봄" : "못 봄") + " → " + why;
    if (!fix) {
      if (now < inv_auto_until_) {
        inv_auto_from_ = now + 0.5;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "재고 불확실 스스로 정합 기다림: %s", evidence.c_str());
        return;
      }
      inv_auto_from_ = -1.0;
      RCLCPP_ERROR(get_logger(), "재고 불확실 스스로 정합 못 함 (%.0f s): %s → 사람 확인 (/mission/inventory_*)", inventory_auto_timeout_s_,
        evidence.c_str());
      return;
    }
    const std::string id = d->item.puck_id.empty() ? d->puck_id : d->item.puck_id;
    std::string msg;
    planning::CargoDoubt rd;
    if (!exec_->resolve_inventory(*fix, msg, &rd)) {
      if (now < inv_auto_until_) {
        inv_auto_from_ = now + 0.5;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "재고 불확실 스스로 정합 기다림 (%s): %s", msg.c_str(), evidence.c_str());
        return;
      }
      inv_auto_from_ = -1.0;
      RCLCPP_ERROR(get_logger(), "재고 불확실 스스로 정합 못 함: %s (%s) → 사람 확인", msg.c_str(), evidence.c_str());
      return;
    }
    inv_auto_from_ = -1.0;
    RCLCPP_WARN(get_logger(), "재고 불확실 스스로 정합 (%s): %s", planning::doubt_fix_name(*fix), evidence.c_str());
    if (place_confirmed_by(*fix, rd.arm_task)) {on_place_confirmed(rd);}   // 놓은 수 · 칸 기록 (안 하면 재계획이 같은 칸에 또 놓음)
    if (*fix == planning::DoubtFix::Removed && !id.empty()) {exec_->mark_puck(id, "on_field");}   // 자리 모름: 다음 LOOK 이 박스에서 보면 다시 집음
    hand_first_once_ = exec_->cargo().holding();
    std::string rw;
    if (resume_mission(rw)) {
      auto_start_once_ = true;
      RCLCPP_WARN(get_logger(), "재고 불확실 스스로 정합 뒤 resume: %s → 준비되면 start (%s)", rw.c_str(),
        hand_first_once_ ? "퍽 쥠: HOME 대신 손 비우기부터" : "HOME 부터");
    } else {
      RCLCPP_WARN(get_logger(), "재고 불확실 스스로 정합 뒤 resume 못 함: %s (사람 확인)", rw.c_str());
    }
  }

  // HOME + DRIVE 뒤 갈 곳: 시간 부족 → 복귀 / 복귀 중 FAULT 였음 → 복귀 다시 / FAULT 로 멈춘 task → 이어 하기
  //   / 아니면 다음 바코드 (점멸 관측은 바코드 뒤)
  void after_home(double now)
  {
    if (time_short(now)) {
      start_return("시간 부족");   // 이어 할 task 는 abandon_task 가 실패로 정리
    } else if (resume_return_) {
      start_return("복귀 재개");
    } else if (resume_.active) {
      continue_task(now);
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
    // 바코드를 찍으러 몸을 옮겼으면 (광각이 신호등을 못 볼 수 있음) 출발 자리로 돌아간 뒤 기록. 기록은 끊지 않고 이어 감 (오는 길에 가려진 조각은 기록기가 스스로 버리고 다음 시퀀스를 기다림)
    //   (팔 접기는 실행기가 이동 앞에 넣음). 돌아가기가 실패하면 그 자리에서 기다림
    if (signal_return_start_ && (bc_base_moved_ || signal_back_m_ > 0.0)) {
      bc_base_moved_ = false;
      Pose2D v = dice_map_.start;   // 출발 자리에서 보는 방향 반대로 (뒤로) signal_back_m
      v.x -= std::cos(v.yaw) * signal_back_m_;
      v.y -= std::sin(v.yaw) * signal_back_m_;
      detail_ = "신호등 기록하러 뒤로";
      RCLCPP_INFO(get_logger(), "OBSERVE_SIGNAL: 바코드 읽은 자리 → 출발 자리에서 %.2f m 뒤 (%.3f, %.3f, %.1f°) 로 가서 신호 기록 (신호 보러 가는 이동, 수집 출발 아님)",
        signal_back_m_, v.x, v.y, v.yaw * 180.0 / M_PI);
      observe_returning_ = true;
      set_steps(Phase::OBSERVE_SIGNAL, build_return_steps(v));
      return;
    }
    set_steps(Phase::OBSERVE_SIGNAL, {});
  }

  // 출발 자리에 돌아옴 (또는 못 감) → 여기서부터 신호 기다림 (signal_wait_s) 을 새로 셈. 기록은 그대로 이어 감
  void observe_arrived(bool ok)
  {
    if (!observe_returning_) {return;}   // 한 번만 (실패 뒤 끼운 HOME 이 끝나도 다시 안 비움)
    observe_returning_ = false;
    const double now = now_s();
    if (!ok) {exec_->set_steps({});}
    observe_start_s_ = now;
    detail_ = "신호등 점멸 기록 중";
    RCLCPP_INFO(get_logger(), "OBSERVE_SIGNAL: %s → 신호 기록 기다림 (지금 %s)", ok ? "신호 보는 자리 도착" : "신호 보는 자리로 못 감, 이 자리에서",
      signal_status_name(signal_.status()));
    publish_signal();
  }

  // 기록 끝 (또는 시간 초과) → 결과 남기고 읽어 둔 task 실행. 불완전해도 색을 추정해 채우지 않음
  void end_observe(double now)
  {
    if (!signal_.finished() && retry_signal(now, "signal_wait_s 안에 안 끝남")) {return;}
    if (!signal_.finished()) {
      RCLCPP_WARN(get_logger(), "신호등 기록 %.0f s 안에 안 끝남 (%s, %s) → 불완전으로 진행",
        now - observe_start_s_, signal_status_name(signal_.status()), signal_.sequence().c_str());
      signal_timed_out_ = true;
      signal_.give_up(now, "signal_wait_s 안에 기록이 안 끝남");
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
    for (auto & v : placed_cell_ids_) {v.clear();}
    look_started_.fill(0.0);
    side_look_.fill(SideLookState::Unchecked);
    for (auto & ws : look_windows_) {ws.clear();}
    puck_views_.clear();
    look_place_started_.fill(0.0);
    for (auto & f : place_frame_) {f.reset();}
    for (auto & f : placed_frame_) {f.reset();}
    used_kind_.clear();
    stop_verify_capture();
    place_dead_ = {false, false, false};
    place_nav_fails_ = {0, 0, 0};
    box_realign_n_ = {0, 0, 0};
    place_measured_ = {false, false, false};
    for (auto & r : realign_prev_) {r.reset();}
    exec_->clear_dock_fix();   // 정차점 보정표 (정밀 접근이 잰 박스) 는 task 마다 새로
    yellow_used_shape_.clear();
    replans_ = 0;
    resume_last_key_.clear();   // FAULT 반복 판정은 task 마다 새로
    resume_last_work_.clear();
    store_returned_n_.clear();
    store_failed_slots_.clear();
    putback_tried_.clear();
    retrieved_here_.clear();
    abandon_after_putback_.clear();
    nav_fault_auto_used_ = 0;
    nav_first_waived_ = false;
    inventory_auto_used_ = 0;
    continue_after_empty_ = false;
    hand_first_once_ = false;
    empty_grasp_n_.clear();
    retrieve_empty_n_.clear();
    rack_stuck_.clear();
    retrieve_reject_n_.clear();
    box_look_retried_.clear();
    box_retried_.clear();
    box_relooked_.fill(false);
    for (auto & ids : lying_backed_ids_) {ids.clear();}
    lying_fwd_.fill(false);
    if (!lying_.active) {lying_ = LyingRun{};}
    place_relooked_.fill(false);
    relook_side_i_.fill(0);
    current_node_ = kNodeStart;
    alloc_revision_ = 0;
    alloc_history_.clear();
    alloc_reason_.clear();
    left_start_ = true;
    detail_.clear();   // OBSERVE_SIGNAL 의 "기록 중" 등을 지움
    RCLCPP_INFO(get_logger(), "신호등 기록 완료 (%s, %s) + 바코드 %s → 수집 출발", signal_status_name(signal_.status()),
      signal_.sequence().c_str(), task.raw_text.c_str());
    RCLCPP_INFO(get_logger(), "task 시작: %s (%s, R %d · Y %d · G %d)", task.raw_text.c_str(),
      dice_order_name(task.dice_order), dice_counts_.red, dice_counts_.yellow, dice_counts_.green);
    RCLCPP_INFO(get_logger(), "배치: L1 %c%d · L2 %c%d · L3 %c%d (%s)",
      dice_plan_->places[0].group, dice_plan_->places[0].pips, dice_plan_->places[1].group, dice_plan_->places[1].pips,
      dice_plan_->places[2].group, dice_plan_->places[2].pips, dice_order_name(task.dice_order));
    if (stop_after_ == "SIGNAL") {   // 시험: 바코드 → 신호 → 배치까지만
      finish_partial("SIGNAL");
      return;
    }
    if (start_phase_ == "DELIVER") {
      start_from_cargo();
      return;
    }
    if (start_phase_ == "VERIFY") {   // 시험: 이미 놓인 배치를 재검증만 (L1 → L2 → L3)
      RCLCPP_WARN(get_logger(), "시험 start_phase VERIFY: 세 장소 재검증만");
      set_steps(Phase::VERIFY, verify_steps({{0, 1, 2}}, dice_map_, verify_look_slot_));
      if (exec_->done()) {on_steps_done();}
      return;
    }
    if (stop_after_ == "COLLECT") {to_collect_only(steps);}
    set_steps(Phase::COLLECT, std::move(steps));
  }

  // 시험 수집만 (stop_after COLLECT): 마지막 박스도 배달 계획 대신 집기 계획 (배달 계획이 그 박스 퍽을 DELIVER 에서 집기 때문)
  static void to_collect_only(std::vector<Step> & steps)
  {
    for (auto & st : steps) {
      if (is_custom(st, kPlanDelivery)) {
        st.kind = kPlanCollectBox;
        st.name = "plan:collect:B" + std::to_string(box_of(st));
      }
    }
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
    continue_after_empty_ = false;
    // 퍽 쥔 채 포기하면 복귀 이동이 CARRY_DRIVE 로 막힘 → 먼저 손 비우기 (다른 칸 STORE · 집은 박스에 되돌림, 성공 뒤 여기 다시) → 포기 · 복귀
    if (task_phase() && start_empty_hand(why)) {return;}
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
    if (barcode_search_) {   // 바코드 자리 무작위 → 찾기 (못 찾으면 bc_fallback_fixed 로 아래 예비 고정 경로)
      start_bc();
      return;
    }
    bc_base_moved_ = true;
    set_steps(Phase::READ_BARCODE, build_read_steps(dice_map_.barcode_view, read_timeout_s_, barcode_look_));
  }

  // 진행 중이던 task 를 실패로 정리 (정확히 한 번: bar_i_ 도 여기서만 넘김)
  //   FAULT 뒤 이어 할 task (resume_.active) 는 이어 하기 전 HOME 단계여도 정리
  void abandon_task(const char * why)
  {
    if (!(task_phase() || resume_.active) || !current_task_) {return;}
    resume_.active = false;
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
    if (is_custom(s, kReconcileCheck) && reconcile_.active) {   // 정합 끝 → 실패한 단계 자리에서 재계획 (그 작업 벌칙 없이)
      const Step f = reconcile_.f;
      reconcile_ = Reconcile{};
      recover_no_penalty_ = true;
      const bool ok = dice_recover(f);
      recover_no_penalty_ = false;
      if (!ok) {abandon_dice(std::string(phase_name(phase_)) + " 자동 정합 뒤 재계획 실패: " + f.name);}
      return;
    }
    if (!resume_last_work_.empty() && s.name == resume_last_work_) {   // FAULT 때 하던 단계가 이어 하기에서 성공 → 반복 판정 끝
      resume_last_key_.clear();
      resume_last_work_.clear();
    }
    // 박스 LOOK 자세에 섬 → 다음 팔 작업 보낼 때까지 이 자세 구간 (그 동안 찍힌 퍽 = 이 slot 에서 보임, puck_views)
    if (s.type == StepType::Arm && is_look_task(s.arm_task) && box_of(s) >= 1 && box_of(s) <= kNumBoxes) {
      auto & ws = look_windows_[box_of(s) - 1];
      ws.push_back(LookWindow{s.slot, now_s(), -1.0});
      if (ws.size() > 32) {ws.erase(ws.begin());}
    }
    if (s.type == StepType::Navigate) {
      ++nav_ok_since_start_;
      ++nav_ok_total_;
      if (box_of(s) >= 1) {current_node_ = box_of(s);}
      if (place_of(s) >= 0) {current_node_ = place_node(place_of(s));}
    }
    if (is_putback(s)) {   // 박스에 되돌려 놓음: 재고 = 그리퍼 빔 (실행기 PLACE 성공), 퍽은 박스에 그대로 두고 다시 안 집음
      exec_->mark_puck(s.puck_id, "skipped");
      RCLCPP_WARN(get_logger(), "되돌림 끝: 퍽 %s 를 집은 박스 자리에 놓음 → 그리퍼 빔 · 그 퍽 건너뜀 (skipped), 이어 감", s.puck_id.c_str());
      return;
    }
    if (s.type == StepType::Arm && s.arm_task == kArmPlace) {
      ++pucks_done_;
      if (is_dice_cell(s) && place_of(s) >= 0 && place_of(s) < kNumPlaces) {
        ++placed_count_[place_of(s)];
        note_placed_cell(place_of(s), cell_of(s), s.puck_id);
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
      case Phase::OBSERVE_SIGNAL:   // 단계 = 출발 자리로 돌아가기뿐. 끝은 tick_running 이 판단
        observe_arrived(true);
        break;
    }
  }

  // 재시도까지 실패한 단계 → 단계별 처리. 팔이 움직이다 실패했으면 다음 단계 앞에 HOME
  void on_step_failed(const Step & s, bool need_home, bool reconcile = false)
  {
    // RETRIEVE 자동 정합의 적재함 스캔 LOOK 이 실패 · 거절: FAULT 안 함 → LOOK 만 빼고 판정 (본 것 없음 = 칸에 둔 채 건너뜀)
    if (reconcile_.active && reconcile_.f.arm_task == kArmRetrieve && s.name == reconcile_.look_name) {
      if (reconcile_.home_skipped && !reconcile_.home_retried && !reconcile_.look_rack && op_ == Op::RUNNING && !pending_fault_) {   // 받침에서 다시 스캔
        reconcile_.home_retried = true;
        RCLCPP_WARN(get_logger(), "자동 정합 %s: HOME 없이 보낸 적재함 스캔 실패 (%s) → HOME 뒤 한 번 더", reconcile_.f.name.c_str(), detail_.c_str());
        exec_->insert_home("HOME 없이 보낸 적재함 스캔 실패 뒤 (자동 정합)");
        return;
      }
      RCLCPP_WARN(get_logger(), "자동 정합 %s: 적재함 스캔 실패 (%s) → 스캔 없이 판정", reconcile_.f.name.c_str(), detail_.c_str());
      exec_->erase_steps(exec_->index(), exec_->index() + 1);
      if (need_home && op_ == Op::RUNNING && !pending_fault_) {exec_->insert_home("적재함 스캔 실패 뒤 (자동 정합)");}
      return;
    }
    if (reconcile) {   // 재고 불확실 + control 실측 빈손 → 확인 LOOK · 판정 단계 끼움 (재계획은 판정 뒤)
      start_reconcile(s);
      // RETRIEVE 빈손: 팔은 박스 위 기준점 (빈손) → control 이 그 자리에서 바로 스캔 → HOME 안 끼움 (실패하면 위에서 HOME 뒤 다시)
      if (reconcile_scan_skip_home_ && reconcile_.active && reconcile_.f.arm_task == kArmRetrieve) {
        reconcile_.home_skipped = true;
        RCLCPP_INFO(get_logger(), "자동 정합 %s: HOME 없이 적재함 스캔 (RETRIEVE 끝 자세에서 바로, reconcile_scan_skip_home)", s.name.c_str());
        return;
      }
      if (need_home && op_ == Op::RUNNING && !pending_fault_) {exec_->insert_home("팔 작업 실패 뒤 (자동 정합)");}
      return;
    }
    switch (phase_) {
      case Phase::READ_BARCODE:
        if (bc_active_ && op_ == Op::RUNNING) {   // 바코드 찾기 중 LOOK_AT · 몸 옮기기 실패 → 찾기가 다음 할 일을 정함
          bc_step_failed(s, need_home);
          return;
        }
        // 안 읽힘 → 같은 자리에서 다시 읽기 (read_retries 번). 이미 처리한 바코드 · 팔 · 이동 실패는 그대로 끝
        if (detail_.find("바코드 안 읽힘") == std::string::npos && detail_.find("이미 처리한 바코드") == std::string::npos &&
          read_tries_ < read_retries_ && op_ == Op::RUNNING)
        {   // 이동 · LOOK_BARCODE 실패 (사람 막음 · 팔 거절): 바코드 없이 복귀 (퍽 0) 하지 않고 처음부터 다시
          ++read_tries_;
          RCLCPP_WARN(get_logger(), "바코드 단계 실패 (%s: %s) → 처음부터 다시 %d / %d", s.name.c_str(), detail_.c_str(), read_tries_, read_retries_);
          set_steps(Phase::READ_BARCODE, build_read_steps(dice_map_.barcode_view, read_timeout_s_, barcode_look_));
          break;
        }
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
        break;
      case Phase::OBSERVE_SIGNAL:
        observe_arrived(false);
        break;
    }
    // 같은 정차에서 바로 팔 작업이 이어지면 HOME 안 끼움: control 이 스스로 검사된 자세로 끝낸 실패만
    //   (EMPTY_GRASP = 온 길로 P0 · LOOK 까지 되돌아감, STORE_RETURNED_TO_BOX = P0 에서 끝) → 다음 LOOK · PICK 은 P0 · LOOK 에서 받음.
    //   다음이 이동이면 실행기가 이동 앞에 HOME. 그 밖 (도중 정지) 은 HOME
    if (need_home && op_ == Op::RUNNING && !pending_fault_) {
      const bool clean_end = detail_.find(kEmptyGraspToken) != std::string::npos || detail_.find(kStoreReturnedToken) != std::string::npos;
      const auto & st = exec_->steps();
      const bool arm_next = exec_->index() < st.size() && st[exec_->index()].type == StepType::Arm &&
        st[exec_->index()].arm_task != kArmHome;
      if (clean_end && arm_next) {
        RCLCPP_INFO(get_logger(), "팔 작업 실패 뒤 HOME 안 함: control 이 검사된 자세로 끝냄 (%s) → 바로 %s",
          detail_.find(kEmptyGraspToken) != std::string::npos ? "EMPTY_GRASP" : "STORE_RETURNED_TO_BOX", st[exec_->index()].name.c_str());
      } else {
        exec_->insert_home("팔 작업 실패 뒤");
      }
    }
  }

  // 그리퍼 기록에 퍽이 있으면 start 가 GRIPPER_RECORD 로 거절됨 → 사람이 할 일 안내 ("" = 그리퍼 빔)
  std::string gripper_hint() const
  {
    if (!exec_->cargo().holding()) {return "";}
    if (hand_first_once_ && resume_.active) {return " / 손에 퍽 기록: start 하면 HOME 대신 손 비우기부터 (스스로 정합)";}
    const std::string id = exec_->cargo().gripper()->puck_id;
    const int b = box_of_puck(id);
    return " / 손에 퍽 " + id + " 기록 → 퍽을 빼서 " + (b >= 1 ? "원래 박스 B" + std::to_string(b) : std::string("원래 박스")) +
      " 위에 놓았으면 /mission/inventory_returned (이어 하기에서 다시 집음), 로봇 밖으로 뺐으면 /mission/clear_gripper (그 퍽 안 씀)";
  }

  // 퍽 기록부 자리 (마지막으로 본 곳) 가 박스 반경 안이면 그 박스 번호, 아니면 0
  int box_of_puck(const std::string & id) const
  {
    for (const auto & r : exec_->registry().records()) {
      if (r.id != id) {continue;}
      for (int b = 1; b <= kNumBoxes; ++b) {
        const auto & c = dice_map_.boxes[b - 1].center;
        if (std::hypot(r.x - c[0], r.y - c[1]) <= dice_map_.box_radius) {return b;}
      }
      return 0;
    }
    return 0;
  }

  // 칸 cell 에 퍽 id 를 놓음 (재검증 칸 ID 판정). 칸 번호 = 계획 pip_offsets 순번
  void note_placed_cell(int place, int cell, const std::string & id)
  {
    if (place < 0 || place >= kNumPlaces || cell < 0 || cell >= 9) {return;}
    auto & v = placed_cell_ids_[place];
    if (static_cast<int>(v.size()) <= cell) {v.resize(static_cast<std::size_t>(cell) + 1);}
    v[static_cast<std::size_t>(cell)] = id;
  }

  // 사람 확인으로 PLACE 성공 (inventory_done) → on_step_ok 의 PLACE 셈과 같게: 놓은 수 · 장소별 놓은 수 · Y 모양
  //   이 셈이 없으면 재계획이 그 장소를 0 개로 보고 남은 퍽이 모자라 버림. 같은 단계 FAULT 반복 판정도 끝 (성공했으므로)
  void on_place_confirmed(const planning::CargoDoubt & d, const Step * known = nullptr)
  {
    std::optional<Step> s;
    if (known && known->name == d.step_name) {s = *known;}   // 자동 정합: 실패 단계는 목록에서 이미 뺌
    if (!s && resume_.point.work && resume_.point.work->name == d.step_name) {s = resume_.point.work;}
    for (std::size_t i = 0; !s && i < exec_->steps().size(); ++i) {
      if (exec_->steps()[i].name == d.step_name) {s = exec_->steps()[i];}
    }
    ++pucks_done_;
    if (s && is_dice_cell(*s) && place_of(*s) >= 0 && place_of(*s) < kNumPlaces) {
      ++placed_count_[place_of(*s)];
      note_placed_cell(place_of(*s), cell_of(*s), s->puck_id.empty() ? d.item.puck_id : s->puck_id);
      if (dice_plan_ && dice_plan_->places[place_of(*s)].group == 'Y' && !d.item.shape.empty()) {
        yellow_used_shape_ = d.item.shape;
      }
      RCLCPP_WARN(get_logger(), "PLACE 확인 (done): %s → L%d 놓은 수 %d", d.step_name.c_str(), place_of(*s) + 1,
        placed_count_[place_of(*s)]);
    } else {
      RCLCPP_WARN(get_logger(), "PLACE 확인 (done): %s 단계를 못 찾음 → 놓은 퍽 수만 셈 (장소별 수는 그대로)", d.step_name.c_str());
    }
    if (resume_last_work_ == d.step_name) {
      resume_last_key_.clear();
      resume_last_work_.clear();
      resume_.repeat = false;   // 그 단계는 성공 → 다음 이어 하기에서 그 장소를 버리지 않음
    }
  }

  // ---------------------------------------------------------------- 재고 자동 정합 (실행기 auto_reconcile)
  //   재고 작업이 움직인 뒤 실패 + control 실측 빈손 (GRIP:EMPTY · OPEN) = 퍽은 손에 없음 → 어디 있는지 카메라로 확인
  //     PICK      : 확인 없이 before (퍽은 박스, 빈손 PICK 과 같게 다시 보고 다시 집기 · 두 번이면 건너뜀)
  //     STORE · LOOK_RACK · RETRIEVE : 빈손 적재함 LOOK (slot rack) → 칸 안 퍽 태그 · 빈 칸 바닥 태그로 판정
  //     PLACE     : 그 장소 LOOK → 손목 검출로 그 칸에 그 퍽이 있나
  //   판정 → resolve_inventory (사람 정합과 같은 함수) → 실패한 단계 자리에서 재계획. 증거가 안 맞거나 모르면 FAULT (사람)
  void start_reconcile(const Step & f)
  {
    const auto d = exec_->cargo().doubt();
    if (!d || !(phase_ == Phase::COLLECT || phase_ == Phase::DELIVER)) {
      exec_->end_reconcile();
      request_fault("INVENTORY_UNCERTAIN", "자동 정합 못 함 (" + std::string(d ? "단계 " : "불확실 표시 없음, 단계 ") +
        phase_name(phase_) + "): " + f.name + " (" + detail_ + ") → 실물 확인 뒤 /mission/inventory_*");
      return;
    }
    if (f.arm_task == kArmPick) {   // 빈손 = 퍽은 박스 (못 잡음 · 놓침) → before, 빈손 PICK 처리 그대로
      std::string m;
      if (!exec_->resolve_inventory(planning::DoubtFix::Before, m)) {
        exec_->end_reconcile();
        request_fault("INVENTORY_UNCERTAIN", "자동 정합 before 실패: " + m);
        return;
      }
      exec_->end_reconcile();
      RCLCPP_WARN(get_logger(), "자동 정합 %s: 실측 빈손 → 퍽은 박스 (before) → 빈손 PICK 처럼 다시 보고 다시 집기", f.name.c_str());
      detail_ = std::string(kEmptyGraspToken) + " 자동 정합 (실측 빈손) · " + detail_;
      on_step_failed(f, false);
      return;
    }
    Step look;
    look.type = StepType::Arm;
    look.arm_task = kArmLook;
    if (f.arm_task == kArmPlace) {
      place_of(look) = place_of(f);
      look.name = "arm:LOOK:L" + std::to_string(place_of(f) + 1) + "(자동 정합)";
    } else if (reconcile_look_rack_ && f.arm_task == kArmRetrieve && !f.slot.empty()) {
      look.arm_task = kArmLookRack;   // 그 칸만 빈손으로 봄 → /rack_tags/detections (적재함 맞춤 안 함)
      look.slot = f.slot;
      look.name = "arm:LOOK_RACK:" + f.slot + "(자동 정합)";
    } else {
      look.slot = "rack";   // control 빈손 적재함 스캔 → /rack_tags/detections
      look.name = "arm:LOOK:rack(자동 정합)";
    }
    Reconcile rc;
    if (f.arm_task == kArmPlace) {   // 칸 자리는 확인 LOOK 전에 (LOOK 이 그 장소 박스 측정 기준을 지움)
      geometry_msgs::msg::PoseStamped cell;
      std::string why;
      if (!dice_cell_target(f, cell, why)) {
        exec_->end_reconcile();
        request_fault("INVENTORY_UNCERTAIN", "자동 정합 못 함: " + f.name + " 칸 자리 모름 (" + why + ") → 실물 확인 뒤 /mission/inventory_*");
        return;
      }
      rc.cell = {cell.pose.position.x, cell.pose.position.y};
    }
    Step check;
    check.type = StepType::Custom;
    check.kind = kReconcileCheck;
    check.name = "check:reconcile:" + f.name;
    const std::size_t at = exec_->index();
    exec_->erase_steps(at, at + 1);   // 실패한 단계는 빼고 판정 뒤 그 자리에서 재계획
    if (look.arm_task == kArmLookRack) {   // 빈손 LOOK_RACK 은 기준점 (박스 위) 에서 끝남 → 판정 전에 접음 (다음 RETRIEVE · 이동은 받침에서)
      Step home;
      home.type = StepType::Arm;
      home.arm_task = kArmHome;
      home.name = "arm:HOME(자동 정합 LOOK_RACK 뒤)";
      exec_->insert_steps(at, {look, home, check});
      rc.look_rack = true;
    } else {
      exec_->insert_steps(at, {look, check});
    }
    rc.active = true;
    rc.f = f;
    rc.doubt = *d;
    rc.look_name = look.name;
    reconcile_ = rc;
    RCLCPP_WARN(get_logger(), "자동 정합 %s: 실측 빈손 (%s) → %s 뒤 판정", f.name.c_str(), detail_.c_str(), look.name.c_str());
  }

  // 확인 LOOK 뒤 판정. 확인 LOOK 시작 = 이 단계 바로 앞 LOOK (since = 정합 시작 뒤 받은 것만)
  StepResult run_reconcile_check(double now, double started)
  {
    if (!reconcile_.active) {return StepResult::Success;}
    const Step & f = reconcile_.f;
    const planning::CargoDoubt & d = reconcile_.doubt;
    const std::string pid = d.item.puck_id.empty() ? d.puck_id : d.item.puck_id;
    const double since = reconcile_.look_s > 0.0 ? reconcile_.look_s : started;
    auto finish = [&](planning::DoubtFix fix, const std::string & why) {
        std::string m;
        planning::CargoDoubt rd;
        if (!exec_->resolve_inventory(fix, m, &rd)) {
          exec_->end_reconcile();
          request_fault("INVENTORY_UNCERTAIN", "자동 정합 " + std::string(planning::doubt_fix_name(fix)) + " 실패: " + m + " (" + why + ")");
          return StepResult::Running;
        }
        exec_->end_reconcile();
        if (place_confirmed_by(fix, rd.arm_task)) {on_place_confirmed(rd, &f);}
        detail_ = "자동 정합 " + std::string(planning::doubt_fix_name(fix)) + ": " + f.name + " — " + why;
        alloc_reason_ = "reconcile: " + f.name + " " + planning::doubt_fix_name(fix);
        RCLCPP_WARN(get_logger(), "%s", detail_.c_str());
        return StepResult::Success;
      };
    auto fail = [&](const std::string & why) {
        exec_->end_reconcile();
        reconcile_conflict_ = true;
        request_fault("INVENTORY_UNCERTAIN", "자동 정합 판정 못 함: " + f.name + " — " + why + " → 실물 확인 뒤 /mission/inventory_*");
        return StepResult::Running;
      };
    const bool waited = now - started >= reconcile_wait_s_;
    if (f.arm_task == kArmPlace) {
      bool here = false, elsewhere = false;
      double ex = 0.0, ey = 0.0;
      for (const auto & o : exec_->recent_obs()) {
        if (o.stamp < since || o.id != pid) {continue;}
        if (std::hypot(o.x - reconcile_.cell[0], o.y - reconcile_.cell[1]) <= reconcile_place_tol_m_) {here = true;}
        else {elsewhere = true; ex = o.x; ey = o.y;}
      }
      if (here && !elsewhere) {return finish(planning::DoubtFix::Done, "퍽 " + pid + " 이 L" + std::to_string(place_of(f) + 1) + " 칸에 보임");}
      if (!waited) {return StepResult::Running;}
      if (elsewhere) {
        char b[160];
        std::snprintf(b, sizeof(b), "퍽 %s 이 칸이 아닌 (%.3f, %.3f) 에 보임 (칸에서 %.0f mm 넘음)", pid.c_str(), ex, ey,
          reconcile_place_tol_m_ * 1000.0);
        return fail(b);
      }
      if (exec_->last_detection_s() < since) {return fail("확인 LOOK 동안 손목 검출이 안 옴");}
      return finish(planning::DoubtFix::Removed, "퍽 " + pid + " 이 장소에 안 보임 (손에도 없음) → 잃은 것으로 봄");
    }
    // 적재함: 칸 안 퍽 태그 (rack_puck, label = 칸) · 빈 칸 바닥 태그 (rack_tag, label = 칸)
    std::set<std::string> puck_cells, empty_cells;
    for (const auto & r : rack_seen_) {
      if (r.t < since) {continue;}
      if (r.puck && r.id == pid) {puck_cells.insert(r.cell);}
      if (!r.puck) {empty_cells.insert(r.cell);}
    }
    const std::string S = f.slot;
    const bool in_s = puck_cells.count(S) > 0;
    const bool other = puck_cells.size() > (in_s ? 1u : 0u);
    // RETRIEVE: FAULT 없이 끝냄. 그 칸에만 보임 = 다시 꺼냄 (retrieve_empty_skip_n 번째면 칸에 둔 채 건너뜀) / 그 칸 빔 · 다른 칸에도 없음 = 잃음 /
    //   확인 못 함 (안 보임 · 다른 칸에 보임) = 칸에 둔 채 건너뜀 (기록은 그 칸, 그 칸에 다른 퍽 안 넣음) → 남은 퍽으로 다시 계획
    if (f.arm_task == kArmRetrieve) {
      auto stuck = [&](const std::string & why) {
          rack_stuck_.insert(pid);
          return finish(planning::DoubtFix::Before, why + " → 퍽 " + pid + " 은 칸 " + S + " 에 둔 채 건너뜀 (남은 퍽으로 계속)");
        };
      if (in_s && !other) {
        const int n = ++retrieve_empty_n_[pid];
        if (n >= retrieve_empty_skip_n_) {return stuck("RETRIEVE 빈손 " + std::to_string(n) + " 번째, 퍽이 아직 칸 " + S + " 에 보임");}
        return finish(planning::DoubtFix::Before, "퍽 " + pid + " 이 아직 칸 " + S + " 에 보임 → 다시 꺼냄 (" + std::to_string(n) + "/" +
          std::to_string(retrieve_empty_skip_n_) + ")");
      }
      if (!waited) {return StepResult::Running;}
      if (!other && empty_cells.count(S) > 0) {
        return finish(planning::DoubtFix::Removed, "퍽 " + pid + " 이 적재함에 안 보임 (칸 " + S + " 바닥 태그 보임 = 빔) · 손에도 없음 → 잃은 것으로 봄");
      }
      if (other) {
        std::string cs;
        for (const auto & c : puck_cells) {cs += (cs.empty() ? "" : " ") + c;}
        return stuck("퍽 " + pid + " 이 다른 칸 (" + cs + ") 에 보임 (판정 못 함)");
      }
      return stuck("적재함 확인 못 함 (칸 " + S + " 도 퍽 " + pid + " 도 안 보임)");
    }
    if (in_s && !other) {
      if (f.arm_task == kArmStore) {return finish(planning::DoubtFix::Done, "퍽 " + pid + " 이 칸 " + S + " 에 보임");}
      if (f.arm_task == kArmRetrieve) {return finish(planning::DoubtFix::Before, "퍽 " + pid + " 이 아직 칸 " + S + " 에 보임");}
      return fail("LOOK_RACK 인데 퍽 " + pid + " 이 칸 " + S + " 에 보임 (기록은 그리퍼)");
    }
    if (!waited) {return StepResult::Running;}
    if (other) {
      std::string cs;
      for (const auto & c : puck_cells) {cs += (cs.empty() ? "" : " ") + c;}
      return fail("퍽 " + pid + " 이 다른 칸 (" + cs + ") 에 보임");
    }
    const bool s_empty = empty_cells.count(S) > 0;
    if (s_empty || (f.arm_task == kArmLookRack && empty_cells.size() >= 3)) {
      return finish(planning::DoubtFix::Removed, "퍽 " + pid + " 이 적재함에 안 보임" + (s_empty ? " (칸 " + S + " 바닥 태그 보임 = 빔)" : "") +
        " · 손에도 없음 → 잃은 것으로 봄");
    }
    return fail("적재함 확인 못 함 (칸 " + S + " 도 퍽 " + pid + " 도 안 보임, 본 빈 칸 " + std::to_string(empty_cells.size()) + " 개)");
  }

  // 모든 목표 취소, 사람 확인 필요
  //   재진입 (resume → start): HOME 부터. RETURN_HOME 중이었으면 복귀 다시
  //   task 중 (COLLECT · DELIVER · VERIFY) 이면 resume_continue_task 켬: task 그대로 두고 HOME 뒤 이어 하기 (hold_task_for_resume)
  //   끔 · 이어 못 하는 원인: task 실패로 정리 → 바코드 처리 끝 → 복귀
  void fault(const std::string & code, const std::string & why)
  {
    if (op_ == Op::FAULT) {return;}
    RCLCPP_ERROR(get_logger(), "FAULT [%s]: %s", code.c_str(), why.c_str());
    reconcile_ = Reconcile{};   // 자동 정합 중 FAULT → 사람 정합 (표시는 실행기에 남음)
    fault_code_ = code;
    detail_ = why;
    if (op_ == Op::RUNNING) {
      if (!hold_task_for_resume(code)) {abandon_task("FAULT");}
      if (phase_ == Phase::RETURN_HOME) {resume_return_ = true;}
    }
    op_ = Op::FAULT;
    barcode_want_ = false;
    auto_resume_at_ = -1.0;
    auto_start_once_ = false;
    inv_auto_from_ = -1.0;
    hand_first_once_ = false;
    continue_after_empty_ = false;
    // 자동 정합 (카메라 확인) 이 증거가 안 맞아 FAULT 를 냈으면 그리퍼만 보는 스스로 정합으로 덮지 않음 (사람)
    const bool conflict = reconcile_conflict_;
    reconcile_conflict_ = false;
    if (code == "INVENTORY_UNCERTAIN" && resume_.active && !conflict && inventory_auto_used_ < inventory_auto_n_) {
      ++inventory_auto_used_;
      inv_auto_from_ = now_s() + std::max(fault_auto_wait_s_, hand_cfg_.stable_s);
      inv_auto_until_ = inv_auto_from_ + inventory_auto_timeout_s_;
      RCLCPP_WARN(get_logger(), "재고 불확실 스스로 정합 %d/%d: %.0f s 뒤 그리퍼 실측으로 손 상태 → 정합 → resume · start", inventory_auto_used_,
        inventory_auto_n_, std::max(fault_auto_wait_s_, hand_cfg_.stable_s));
    }
    if (code == "NAV_NOT_WORKING" && resume_.active && nav_fault_auto_used_ < nav_fault_auto_n_) {
      ++nav_fault_auto_used_;
      if (nav_fault_auto_used_ >= 2) {nav_first_waived_ = true;}
      auto_resume_at_ = now_s() + fault_auto_wait_s_;
      RCLCPP_WARN(get_logger(), "첫 이동 FAULT 스스로 이어 가기 %d/%d: %.0f s 뒤 resume → start (%s)", nav_fault_auto_used_,
        nav_fault_auto_n_, fault_auto_wait_s_,
        nav_first_waived_ ? "이번 task 는 첫 이동 FAULT 안 냄 → 이동 실패는 박스 다시 · 건너뜀 · 복귀로" : "같은 목표 다시");
    }
    exec_->stop();   // 진행 중이던 재고 작업은 여기서 불확실 표시될 수 있음
    if (const auto doubt = exec_->doubt_text(); !doubt.empty() && code != "INVENTORY_UNCERTAIN") {
      detail_ += " / 재고 불확실: " + doubt;
    }
    publish_hold();
  }

  // ---------------------------------------------------------------- FAULT 뒤 task 이어 하기 (resume_continue_task)
  // 이어 하지 않는 원인: 계획 자체가 안 됨 (다시 해도 같음) · task 전 원인
  static bool resume_never(const std::string & code)
  {
    return code == "DICE_UNSOLVABLE" || code == "MAP_INVALID" || code == "BARCODE_UNREGISTERED";
  }

  // FAULT 때 (실행기 stop 전): 이어 할 task 면 자리를 적고 true (task 그대로), 아니면 false (→ task 포기)
  //   재고 불확실 (INVENTORY_UNCERTAIN 등) 도 이어 함: resume 이 /mission/inventory_* 정합 전엔 거절 → 사람이 정한 재고로 이어 감
  //   이어 하기 앞 HOME 중 FAULT → 지난 자리 · 반복 판정 그대로 (그 HOME 은 task 단계가 아님)
  bool hold_task_for_resume(const std::string & code)
  {
    if (!current_task_ || !get_parameter("resume_continue_task").as_bool() || resume_never(code)) {
      return false;
    }
    if (phase_ == Phase::HOME && resume_.active) {
      RCLCPP_WARN(get_logger(), "이어 하기 앞 HOME 중 FAULT → task 그대로 (다시 resume 뒤 %s)", resume_text().c_str());
      return true;
    }
    if (!task_phase()) {return false;}
    resume_.active = true;
    resume_.phase = phase_;
    resume_.point = resume_point(exec_->steps(), exec_->index());
    resume_.code = code;
    const std::string key = resume_key(code, resume_.point.work);
    resume_.repeat = key == resume_last_key_;
    if (code == "NAV_NOT_WORKING") {resume_.repeat = false;}   // 주행 계통 FAULT 는 박스 탓이 아님 → 반복이어도 그 박스를 빼지 않음
    resume_last_key_ = key;
    resume_last_work_ = resume_.point.work ? resume_.point.work->name : "";
    RCLCPP_WARN(get_logger(), "task 그대로 둠 (FAULT, resume_continue_task): %s%s", resume_text().c_str(),
      resume_.repeat ? " — 같은 원인 · 같은 단계 FAULT 반복 → 그 단계 빼고 재계획" : "");
    return true;
  }

  // 상태 · 로그용: 이어 할 자리
  std::string resume_text() const
  {
    const auto & r = resume_;
    std::string s = std::string("start 하면 HOME 뒤 ") + phase_name(r.phase) + " 이어 하기";
    if (r.point.work) {s += ", 하던 단계 " + r.point.work->name;}
    if (r.phase == Phase::COLLECT) {
      std::string b;
      for (const int x : r.point.boxes) {b += (b.empty() ? "B" : " B") + std::to_string(x);}
      s += ", 남은 박스 " + (b.empty() ? std::string("없음 → 배달 계획") : b);
    }
    return s;
  }

  // 박스 b 위 퍽 (기록부 on_field) 을 이번 미션에서 뺌 (skipped)
  void skip_box_pucks(int b)
  {
    for (const auto & p : pucks_on_boxes()) {
      if (p.box == b) {exec_->mark_puck(p.id, "skipped");}
    }
  }

  // HOME 뒤 FAULT 로 멈춘 task 이어 하기: 지금 재고 (적재함 · 박스 퍽 · skipped · 버린 장소 · 놓은 수) 로 다시
  //   COLLECT : 지금 박스부터 남은 박스 이동 → LOOK → Plan (끝 박스는 배달 계획). 남은 박스 없으면 지금 자리에서 배달 계획
  //   DELIVER : plan_from_state (첫 정차 이동 → LOOK 부터) → 배달 → 재검증
  //   VERIFY  : 지금 장소부터 남은 재검증
  //   같은 FAULT 반복 (repeat): 재계획 횟수 하나 (max_replans 넘으면 포기 · 복귀) + 하던 퍽 · 장소 · 박스 뺌 (resume_skip)
  void continue_task(double now)
  {
    Resume r = resume_;
    resume_.active = false;
    phase_ = r.phase;   // abandon_dice · 계획 실패가 task 단계로 정리하게
    detail_.clear();
    std::vector<int> boxes = r.point.boxes;
    std::vector<Step> tail = r.point.tail;
    if (r.repeat) {
      if (++replans_ > max_replans_) {
        RCLCPP_WARN(get_logger(), "재계획 %d 번 넘음 → 포기", max_replans_);
        abandon_dice("FAULT 반복 (" + r.code + ", " + (r.point.work ? r.point.work->name : "?") + ")");
        return;
      }
      const ResumeSkip k = r.point.work ? resume_skip(*r.point.work) : ResumeSkip{};
      std::string what = "뺄 것 없음";
      if (!k.puck.empty()) {
        exec_->mark_puck(k.puck, "skipped");
        what = "퍽 " + k.puck + " 건너뜀";
      } else if (k.place >= 0) {
        if (r.phase == Phase::VERIFY) {
          tail = drop_place_stops(tail, k.place);
          what = "L" + std::to_string(k.place + 1) + " 재검증 건너뜀";
        } else {
          place_dead_[k.place] = true;
          what = "L" + std::to_string(k.place + 1) + " 버림";
        }
      } else if (k.box >= 1) {
        boxes.erase(std::remove(boxes.begin(), boxes.end(), k.box), boxes.end());
        skip_box_pucks(k.box);
        what = "B" + std::to_string(k.box) + " 건너뜀";
      }
      RCLCPP_WARN(get_logger(), "재계획 %d/%d: FAULT %s 가 %s 에서 반복 → %s", replans_, max_replans_, r.code.c_str(),
        r.point.work ? r.point.work->name.c_str() : "?", what.c_str());
    }
    alloc_reason_ = "resume: FAULT " + r.code + (r.point.work ? " (" + r.point.work->name + ")" : "");
    RCLCPP_WARN(get_logger(), "FAULT 뒤 task 이어 하기: %s (%s)", phase_name(r.phase), current_task_->raw_text.c_str());
    if (r.phase == Phase::COLLECT && !boxes.empty()) {
      std::string err;
      auto steps = collect_steps(dice_map_, boxes, &err);
      if (steps.empty()) {
        abandon_dice("이어 하기 수집 단계 못 만듦: " + err);
        return;
      }
      if (stop_after_ == "COLLECT") {to_collect_only(steps);}
      std::string b;
      for (const int x : boxes) {b += (b.empty() ? "B" : " B") + std::to_string(x);}
      RCLCPP_INFO(get_logger(), "COLLECT 이어 하기: 남은 박스 %s", b.c_str());
      set_steps(Phase::COLLECT, std::move(steps));
      return;
    }
    if (r.phase == Phase::VERIFY) {
      RCLCPP_INFO(get_logger(), "VERIFY 이어 하기: 단계 %zu 개", tail.size());
      set_steps(Phase::VERIFY, std::move(tail));
      if (exec_->done()) {on_steps_done();}
      return;
    }
    // DELIVER · 남은 박스 없는 COLLECT: 지금 재고로 배달 계획 (서 있지 않음 → 첫 정차 이동 → LOOK)
    if (r.phase == Phase::COLLECT && stop_after_ == "COLLECT") {   // 시험 수집만: 배달 계획 없이 끝
      set_steps(Phase::COLLECT, {});
      on_steps_done();
      return;
    }
    if (!plan_from_state(false, now)) {
      abandon_dice("이어 하기 배달 계획 실패");
      return;
    }
    auto steps = std::move(delivery_steps_);
    delivery_steps_.clear();
    RCLCPP_INFO(get_logger(), "DELIVER 이어 하기: 단계 %zu 개", steps.size());
    set_steps(Phase::DELIVER, std::move(steps));
    if (exec_->done()) {on_steps_done();}   // 더 놓을 게 없음 → 재검증
  }

  // Custom 단계 안에서 FAULT 를 정함 → step() 이 끝난 뒤 적용 (실행기 안에서 상태를 바꾸지 않음)
  void request_fault(const std::string & code, const std::string & why)
  {
    if (!pending_fault_) {pending_fault_ = std::make_pair(code, why);}
  }

  // ================================================================ Custom 단계 (실행기가 부름)

  // ---------------------------------------------------------------- perception 멈춤 · respawn: LOOK 결과를 쓰는 단계
  //   이 정차 (앞 이동 단계 뒤) LOOK 을 보낸 뒤 손목 검출이 끊긴 적이 있으면 (perception 멈춤 → 멈춤 감시 5 s → 기록 → respawn)
  //     끊김 중 → 이 단계를 미룸 (Running). 안 돌아오면 실행기 감시가 detection_fault_s 뒤 DETECTION_LOST FAULT
  //     회복 뒤 → 같은 LOOK + 이 단계를 바로 뒤에 끼우고 이 단계는 판단 없이 넘김 (같은 단계 2 번까지, 넘으면 본 만큼으로)
  //   계획 (collect · 박스 delivery) = 이 정차 첫 LOOK (앞 LOOK, 관측 구간 처음) 다시, 그 밖 = 마지막 LOOK 다시
  //   재고 정합 확인 (kReconcileCheck) 은 성공 = 정합 끝이라 다시 끼우지 않고 기다리기만
  //   끊김이 없으면 아무것도 안 함
  std::optional<StepResult> gap_guard(const Step & s, double now)
  {
    bool plan = false, redo_ok = true;
    switch (s.kind) {
      case kPlanCollectBox: plan = true; break;
      case kPlanDelivery:
        if (box_of(s) < 1) {return std::nullopt;}   // 지금 자리 재계획 (LOOK 없음)
        plan = true;
        break;
      case kPlanVerify: case kPlaceBox: case kLookDwell: case kRelookDwell: case kBoxRelocate: break;
      case kReconcileCheck: redo_ok = false; break;
      default: return std::nullopt;
    }
    const auto & st = exec_->steps();
    std::optional<Step> first, last;
    for (std::size_t j = std::min(exec_->index(), st.size()); j-- > 0;) {
      if (st[j].type == StepType::Navigate) {break;}
      if (st[j].type == StepType::Arm && is_look_task(st[j].arm_task)) {
        if (!last) {last = st[j];}
        first = st[j];
      }
    }
    if (!last || last_look_sent_s_ < 0.0) {return std::nullopt;}
    const int b = box_of(s);
    const double since = plan && b >= 1 && b <= kNumBoxes && look_started_[b - 1] > 0.0 ?
      std::min(look_started_[b - 1], last_look_sent_s_) : last_look_sent_s_;
    const std::string key = std::to_string(s.kind) + ":" + std::to_string(b) + ":" + std::to_string(place_of(s)) + ":" + s.puck_id;
    int & used = gap_redo_[key];
    const auto & g = exec_->detection_gap();
    switch (planning::gap_action(g, since, used, kGapRedoMax, redo_ok)) {
      case planning::GapAction::Proceed:
        if (g.overlaps(since) && redo_ok) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s: LOOK 뒤 손목 검출 끊김이 있었지만 다시 하기 %d 번 다 씀 → 본 만큼으로",
            s.name.c_str(), kGapRedoMax);
        }
        return std::nullopt;
      case planning::GapAction::Wait:
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "%s: 손목 검출 끊김 %.1f s (perception 멈춤 · respawn?) → 돌아올 때까지 미룸",
          s.name.c_str(), now - g.started());
        return StepResult::Running;
      case planning::GapAction::Redo:
        break;
    }
    ++used;
    const Step & look = plan ? *first : *last;
    RCLCPP_WARN(get_logger(), "%s: LOOK 뒤 손목 검출이 %.1f s 끊겼다 돌아옴 (perception 다시 켜짐 %d 번) → %s 다시 뒤 이 단계 다시 (%d/%d)",
      s.name.c_str(), g.ended() - g.started(), exec_->perception_restarts(), look.name.c_str(), used, kGapRedoMax);
    exec_->insert_after_current({look, s});
    return StepResult::Success;
  }

  StepResult run_custom(const Step & s, double now, double started, std::string & detail)
  {
    if (auto g = gap_guard(s, now)) {return *g;}
    switch (s.kind) {
      case kReadBarcode: return run_read(s, now, started, detail);
      case bc::kStepBcPlan: return run_bc_plan(now, detail);
      case bc::kStepBcDwell: return run_bc_dwell(s, now, started);
      case kPlanCollectBox:
      case kPlanDelivery:
      case kPlanVerify:
        return run_plan(s, now, started, detail);
      case kPlaceBox: return run_place_box(s, now, started);
      case kLookDwell: return now - started < look_dwell_s_ ? StepResult::Running : StepResult::Success;   // 옆 보기 자세로 검출 모음
      case kRelookDwell: return run_relook_dwell(s, now, started);
      case kBoxRelocate: return run_box_relocate(s, now, started, detail);
      case kReconcileCheck: return run_reconcile_check(now, started);
      case kLyingBackoff: return run_lying_backoff(s, now);
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
    placed_frame_[p] = place_frame_[p];   // 재검증이 같은 기준으로 봄 (재검증 LOOK 이 place_frame_ 를 지움)
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
      realign_prev_[p].reset();
      place_measured_[p] = true;   // 이 task 에서 다시 와도 이 값 (지도 좌표) 을 씀 → LOOK · 측정 건너뜀 (skip_measured_place_looks)
      realign_place(p, *f);   // 기대 자리와 많이 다르면 측정 박스에 맞춰 다시 붙기 (이 단계 바로 뒤에 끼움)
      return StepResult::Success;
    }
    if (now - started < box_wait_s_) {return StepResult::Running;}
    if (const auto prev = realign_prev_[p]) {   // 다시 붙은 뒤 못 잼 → 지도 중심 대신 다시 붙기 전 측정 (박스는 그대로 map 자리)
      place_frame_[p] = prev;
      realign_prev_[p].reset();
      RCLCPP_WARN(get_logger(), "L%d 다시 붙은 뒤 박스 측정 %.1f s 안에 못 받음 → 다시 붙기 전 측정 기준 (%d 개) 으로 놓음", p + 1, box_wait_s_, prev->n);
      return StepResult::Success;
    }
    if (place_relocated_[p]) {   // 다시 찾기가 잰 박스 (중심 · 방향) 를 기준으로 (칸이 돈 박스를 따라 돎)
      PlaceFrame f;
      f.x = dice_map_.places[p].center[0];
      f.y = dice_map_.places[p].center[1];
      f.dyaw = place_box_dyaw_[p];
      f.n = 0;
      place_frame_[p] = f;
      RCLCPP_WARN(get_logger(), "L%d 박스 측정 %.1f s 안에 못 받음 → 다시 찾기로 잰 박스 (%+.1f°) 기준으로 놓음", p + 1, box_wait_s_,
        f.dyaw * 180.0 / M_PI);
      return StepResult::Success;
    }
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
    auto f = place_frame_from(v, since, dice_map_.places[place].center, dice_map_.map_from_arena.yaw + place_box_dyaw_[place], box_max_shift_m_,
      box_min_samples_, &why);
    if (!f && warn) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "L%d 박스 측정 못 씀 (%s) → 지도 중심 사용", place + 1, why.c_str());
    }
    // 다시 찾기로 돈 박스 (place_box_dyaw_): 접기는 그 방향 기준 (±45° 경계에서 안 갈림), 돌려주는 dyaw 는 지도 방향 대비 전체
    //   → 눈 오프셋 회전 (frame_point) · 재검증 (apply_place_frame) 은 같은 뜻. 다시 붙기 비교만 realign_place 에서 잔차로 바꿈
    if (f) {f->dyaw += place_box_dyaw_[place];}
    return f;
  }

  // 장소 박스 다시 붙기 (box_realign_*): 측정 박스 f 가 이 정차에서 기대한 박스와 많이 다르면
  //   측정 박스에 맞춘 정차점 → LOOK → 박스 측정을 지금 (kPlaceBox) 바로 뒤에 끼움. 놓기 단계는 그 뒤 그대로 (새 측정 기준)
  //   기대 박스 = 이 정차를 연 이동 목표에서 본 지도 박스 자리 (지도 정차면 지도 중심, 다시 붙은 정차면 그때 측정)
  //   횟수를 다 썼으면 측정 기준으로 그대로 놓음 (거절 · FAULT 없음). 팔 접기는 실행기가 이동 앞에 넣음 (HOME 먼저)
  void realign_place(int p, const PlaceFrame & f_total)
  {
    if (p < 0 || p >= kNumPlaces || box_realign_max_ <= 0) {return;}
    PlaceFrame f = f_total;   // 지도 정차 (다시 찾기로 바뀌었으면 그 정차) 기준 잔차 회전
    f.dyaw = fold_quarter_diff(f_total.dyaw, place_box_dyaw_[p]);
    const auto & pl = dice_map_.places[p];
    const auto & st = exec_->steps();
    const std::size_t at = exec_->index();
    if (at >= st.size()) {return;}
    Pose2D used = pl.stop;
    std::optional<Step> look;
    for (std::size_t i = at; i-- > 0;) {
      if (st[i].type == StepType::Navigate) {
        if (place_of(st[i]) == p) {used = st[i].pose;}   // 다른 곳 이동이면 (있으면 안 됨) 지도 정차로 봄
        break;
      }
      if (!look && st[i].type == StepType::Arm && is_look_task(st[i].arm_task) && place_of(st[i]) == p) {look = st[i];}
    }
    const PlaceFrame e = expected_box_at(used, pl.stop, pl.center);
    const auto r = realign_check(f, e, box_realign_shift_m_, box_realign_yaw_rad_, box_realign_n_[p], box_realign_max_);
    const double deg = 180.0 / M_PI;
    if (!r.over) {
      RCLCPP_INFO(get_logger(), "L%d 박스: 이 정차 기대 자리와 %.1f mm · %+.1f° (기준 %.0f mm · %.0f°) → 그대로 놓음", p + 1,
        1000.0 * r.shift_m, r.dyaw * deg, 1000.0 * box_realign_shift_m_, box_realign_yaw_rad_ * deg);
      return;
    }
    if (!r.realign) {
      RCLCPP_WARN(get_logger(), "L%d 박스: 기대 자리와 %.1f mm · %+.1f° 다름, 다시 붙기 %d/%d 다 씀 → 측정 박스 기준으로 그대로 놓음",
        p + 1, 1000.0 * r.shift_m, r.dyaw * deg, box_realign_n_[p], box_realign_max_);
      return;
    }
    ++box_realign_n_[p];
    place_measured_[p] = false;   // 다시 붙은 뒤 LOOK · 측정으로 확인 (그 전엔 재사용 안 함)
    realign_prev_[p] = f_total;   // 다시 붙은 뒤 못 재면 이것으로 놓음
    const Pose2D ns = realigned_stop(f, pl.stop, pl.center);
    Step nav;
    nav.type = StepType::Navigate;
    nav.target = Target::Fixed;
    nav.pose = ns;
    nav.dock = true;   // 지도 정차와 같이 정밀 접근 (면 거리 같음)
    place_of(nav) = p;
    nav.name = "nav:L" + std::to_string(p + 1) + "(다시 붙기)";
    if (!look) {
      look = Step{};
      look->type = StepType::Arm;
      look->arm_task = kArmLook;
      place_of(*look) = p;
      look->name = "arm:LOOK:L" + std::to_string(p + 1);
    }
    look->name += "(다시 붙기)";
    Step w = st[at];   // 지금 박스 측정 단계 (kPlaceBox) 를 한 번 더
    w.name += "(다시 붙기)";
    RCLCPP_WARN(get_logger(), "L%d 박스: 기대 자리와 %.1f mm · %+.1f° 다름 (기준 %.0f mm · %.0f°) → 측정 박스에 맞춘 정차 (%.3f, %.3f, %.1f°) 로 "
      "다시 붙고 다시 봄 %d/%d", p + 1, 1000.0 * r.shift_m, r.dyaw * deg, 1000.0 * box_realign_shift_m_, box_realign_yaw_rad_ * deg,
      ns.x, ns.y, ns.yaw * deg, box_realign_n_[p], box_realign_max_);
    exec_->insert_after_current({nav, *look, w});
  }

  // ================================================================ 바코드 찾기 (barcode_search.hpp)
  //   READ_BARCODE = [bc:plan] → (LOOK_AT → bc:dwell → bc:plan) · (HOME → 이동 → bc:dwell → bc:plan) … → 읽음 → HOME
  //   bc:plan 이 BarcodeSearch::next 로 다음 할 일 하나를 정하고 그 단계를 바로 뒤에 끼움. 팔 거절 · 실패도 찾기가 다음 할 일을 정함
  void declare_bc()
  {
    barcode_search_ = declare_parameter<bool>("barcode_search", true);
    bc_fallback_fixed_ = declare_parameter<bool>("bc_fallback_fixed", true);   // 못 찾으면 예비 고정 경로 barcode_view + LOOK_BARCODE
    bc_keep_wide_ = declare_parameter<bool>("bc_keep_wide", true);             // 신호 기록이 끝나도 찾는 동안 광각 켬
    bc_wide_frame_ = declare_parameter<std::string>("bc_wide_frame", "birdeye_camera_optical_frame");
    bc_wide_pitch_offset_deg_ = declare_parameter<double>("bc_wide_pitch_offset_deg", 4.0);   // 줄자: URDF 30° → 실제 약 34° 아래
    // + = 실제 광각이 URDF 보다 왼쪽을 봄 (추정이 왼쪽으로)
    bc_wide_yaw_offset_deg_ = declare_parameter<double>("bc_wide_yaw_offset_deg", 0.0);
    test_return_home_ = declare_parameter<bool>("test_return_home", true);   // false = stop_after 시험 끝에 출발점으로 안 감 (벤치)
    bc_params_.two_step = declare_parameter<bool>("bc_two_step", bc_params_.two_step);
    bc_params_.locate_standoff_m = declare_parameter<double>("bc_locate_standoff_m", bc_params_.locate_standoff_m);
    bc_light_lost_s_ = declare_parameter<double>("bc_light_lost_s", 1.0);     // LOOK_AT 뒤 신호등이 이만큼 안 보이면 팔이 가림 → 접고 다음
    bc_look_timeout_s_ = declare_parameter<double>("bc_look_timeout_s", 30.0);
    // 팔이 신호등을 가리지 않게 (기록 중 · 신호등을 광각에서 본 뒤만): 광각 → 신호등 원뿔 = 상자 반 대각선 + margin, 최소 min
    bc_view_pitch_deg_ = declare_parameter<double>("bc_view_pitch_deg", 43.0);            // LOOK_AT 보는 방향 아래 각 (slot vpitch)
    bc_sweep_view_pitch_deg_ = declare_parameter<double>("bc_sweep_view_pitch_deg", 20.0);
    bc_light_cone_margin_deg_ = declare_parameter<double>("bc_light_cone_margin_deg", 4.0);
    bc_light_cone_min_deg_ = declare_parameter<double>("bc_light_cone_min_deg", 6.0);
    {   // 겨눔 화소 (control_master look_at_aim_offset_px 와 같게) · 머묾 끝 사진
      const auto ao = declare_parameter<std::vector<double>>("bc_aim_offset_px", {-60.0, -50.0});
      if (ao.size() == 2) {bc_params_.aim_du_px = ao[0]; bc_params_.aim_dv_px = ao[1];}
      const auto snap = declare_parameter<std::string>("bc_snapshot_topic", "/perception/barcode/snapshot");
      if (!snap.empty()) {pub_bc_snap_ = create_publisher<std_msgs::msg::String>(snap, 10);}
    }
    const auto K = declare_parameter<std::vector<double>>("bc_wide_K", {853.374030, 852.156716, 675.548425, 371.440164});   // fx fy cx cy
    const auto D = declare_parameter<std::vector<double>>("bc_wide_D", {-0.128704, 0.012667, -0.039807, 0.022852});        // 어안 k1–k4
    const auto WS = declare_parameter<std::vector<int64_t>>("bc_wide_size", {1280, 720});
    const auto WK = declare_parameter<std::vector<double>>("bc_wrist_K", {606.320862, 606.188171, 321.425476, 248.199982});
    const auto WWS = declare_parameter<std::vector<int64_t>>("bc_wrist_size", {640, 480});
    if (K.size() == 4) {bc_wide_.fx = K[0]; bc_wide_.fy = K[1]; bc_wide_.cx = K[2]; bc_wide_.cy = K[3];}
    if (D.size() == 4) {for (int i = 0; i < 4; ++i) {bc_wide_.k[i] = D[i];}}
    if (WS.size() == 2) {bc_wide_.w = static_cast<int>(WS[0]); bc_wide_.h = static_cast<int>(WS[1]);}
    if (WK.size() == 4) {bc_wrist_.fx = WK[0]; bc_wrist_.fy = WK[1]; bc_wrist_.cx = WK[2]; bc_wrist_.cy = WK[3];}
    if (WWS.size() == 2) {bc_wrist_.w = static_cast<int>(WWS[0]); bc_wrist_.h = static_cast<int>(WWS[1]);}
    auto & p = bc_params_;
    // 읽기 범위 (자리표시 — 실측으로 바꿀 것): 거리 · 비스듬 · 화소/모듈 · 가운데 허용
    p.standoff_m = declare_parameter<double>("bc_standoff_m", p.standoff_m);
    p.standoff_min_m = declare_parameter<double>("bc_standoff_min_m", p.standoff_min_m);
    p.standoff_max_m = declare_parameter<double>("bc_standoff_max_m", p.standoff_max_m);
    p.sweep_standoff_m = declare_parameter<double>("bc_sweep_standoff_m", p.sweep_standoff_m);
    p.max_obliquity_deg = declare_parameter<double>("bc_max_obliquity_deg", p.max_obliquity_deg);
    p.modules = static_cast<int>(declare_parameter<int>("bc_modules", p.modules));
    p.min_px_per_module = declare_parameter<double>("bc_min_px_per_module", p.min_px_per_module);
    p.min_width_px = declare_parameter<double>("bc_min_width_px", p.min_width_px);
    p.center_tol_px = declare_parameter<double>("bc_center_tol_px", p.center_tol_px);
    p.edge_margin_px = declare_parameter<double>("bc_edge_margin_px", p.edge_margin_px);
    // 자리 재기
    p.default_range_m = declare_parameter<double>("bc_default_range_m", p.default_range_m);
    p.lidar_range_offset_m = declare_parameter<double>("bc_lidar_range_offset_m", p.lidar_range_offset_m);
    p.scan_half_deg = declare_parameter<double>("bc_scan_half_deg", p.scan_half_deg);
    p.barcode_width_m = declare_parameter<double>("bc_barcode_width_m", p.barcode_width_m);
    p.z_min_m = declare_parameter<double>("bc_z_min_m", p.z_min_m);
    p.z_max_m = declare_parameter<double>("bc_z_max_m", p.z_max_m);
    // 신호등 · 몸
    p.light_margin_deg = declare_parameter<double>("bc_light_margin_deg", p.light_margin_deg);
    p.light_default_range_m = declare_parameter<double>("bc_light_default_range_m", p.light_default_range_m);
    p.light_default_height_m = declare_parameter<double>("bc_light_default_height_m", p.light_default_height_m);
    p.rotate_steps_deg = declare_parameter<std::vector<double>>("bc_rotate_steps_deg", p.rotate_steps_deg);
    p.max_base_moves = static_cast<int>(declare_parameter<int>("bc_max_base_moves", p.max_base_moves));
    p.approach_dist_m = declare_parameter<double>("bc_approach_dist_m", p.approach_dist_m);
    p.approach_first = declare_parameter<bool>("bc_approach_first", p.approach_first);
    p.approach_tol_m = declare_parameter<double>("bc_approach_tol_m", p.approach_tol_m);
    // 팔을 펴 둔 시간 상한 (실물: 뻗은 자세 2 분 → J2 과부하 48 °C). 넘으면 다음 LOOK_AT 전에 HOME + 쉼
    bc_max_arm_out_s_ = declare_parameter<double>("bc_max_arm_out_s", 15.0);
    bc_fold_rest_s_ = declare_parameter<double>("bc_fold_rest_s", 5.0);
    bc_arm_out_total_s_ = declare_parameter<double>("bc_arm_out_total_s", 40.0);   // 찾기 전체에서 팔을 편 시간 합 상한 → 넘으면 포기 (예비 고정 경로)
    // 광각 상자 기다림 (찾기 시작 · 몸 옮긴 뒤): 받은 지 settle 넘고 마지막이 0.6 s 안이어야 '광각에 없음' 을 믿음, wait 넘으면 그냥 감
    bc_wide_settle_s_ = declare_parameter<double>("bc_wide_settle_s", 1.5);
    bc_wide_wait_s_ = declare_parameter<double>("bc_wide_wait_s", 6.0);
    p.perturb_need_box = declare_parameter<bool>("bc_perturb_need_box", p.perturb_need_box);
    p.depth_plane_tol_m = declare_parameter<double>("bc_depth_plane_tol_m", p.depth_plane_tol_m);
    p.light_wait_s = declare_parameter<double>("bc_light_wait_s", p.light_wait_s);
    p.robot_half_m = declare_parameter<double>("bc_robot_half_m", p.robot_half_m);
    // 훑기 · 시간
    p.sweep_az_deg = declare_parameter<std::vector<double>>("bc_sweep_az_deg", p.sweep_az_deg);
    p.sweep_heights_m = declare_parameter<std::vector<double>>("bc_sweep_heights_m", p.sweep_heights_m);
    p.dwell_s = declare_parameter<double>("bc_dwell_s", p.dwell_s);
    p.sweep_dwell_s = declare_parameter<double>("bc_sweep_dwell_s", p.sweep_dwell_s);
    p.settle_s = declare_parameter<double>("bc_settle_s", p.settle_s);
    p.refine_max = static_cast<int>(declare_parameter<int>("bc_refine_max", p.refine_max));
    p.max_s = declare_parameter<double>("bc_max_s", p.max_s);
    p.hold_max_s = signal_wait_s_;   // 몸 멈춤 (신호 기록 중) 기다림 합 상한 = 신호 기다림과 같게 (새 파라미터 없음)
    p.wide_fresh_s = declare_parameter<double>("bc_wide_fresh_s", p.wide_fresh_s);
    {
      const double j1 = declare_parameter<double>("bc_perturb_j1_deg", 0.0);   // 기본 끔 (팔 움직임 줄임), 켜려면 4
      const double j4 = declare_parameter<double>("bc_perturb_j4_deg", 0.0);   //   켜려면 3
      const double dd = declare_parameter<double>("bc_perturb_d_m", 0.0);   // 0 = 거리 흔들기 안 함 (15 cm 밖은 못 읽음)
      p.perturbs.clear();
      if (j1 > 0.0) {p.perturbs.push_back({j1, 0, 0}); p.perturbs.push_back({-j1, 0, 0});}
      if (j4 > 0.0) {p.perturbs.push_back({0, j4, 0}); p.perturbs.push_back({0, -j4, 0});}
      if (dd > 0.0) {p.perturbs.push_back({0, 0, -dd}); p.perturbs.push_back({0, 0, dd});}
    }
    const auto za = declare_parameter<std::vector<double>>("bc_zone_arena", {0.434, 1.034, 0.650, 1.250});   // 출발 구역 [x0 x1 y0 y1] 경기장 m
    if (za.size() == 4 && map_ok_) {
      bc_zone_ = bc::zone_from_arena(za[0], za[1], za[2], za[3], dice_map_.map_from_arena.x, dice_map_.map_from_arena.y,
        dice_map_.map_from_arena.yaw);
    }
    // 벽 판: 바코드는 늘 출발 구역 앞 벽 판 하나 (아크릴, 파란 기둥 둘 사이, 폭 36 · 높이 39 cm) → 광각 상자는 판 안만, 돌기 없음,
    //   판 따라 팔만 훑기, 몸은 팔이 못 닿을 때만 판 앞으로 작게. 가운데 [x y z] 경기장 m (z = 바닥 위 판 가운데), 빈 목록 = 판 안 씀 (돌기 · 방위 훑기)
    //   기본 가운데: 실주행 손목 읽기 자리 (경기장 0.03, 0.92, 0.17) · 광각 추정 (0.04, 0.98, 0.19) · 광각 사진 (바코드가 판 가운데 오른쪽 약 3 cm)
    const auto pa = declare_parameter<std::vector<double>>("bc_panel_arena", {0.05, 0.93, 0.20});
    const auto ps = declare_parameter<std::vector<double>>("bc_panel_size", {0.36, 0.39});        // 폭 · 높이 m
    p.panel_margin_m = declare_parameter<double>("bc_panel_margin_m", p.panel_margin_m);
    p.panel_depth_tol_m = declare_parameter<double>("bc_panel_depth_tol_m", p.panel_depth_tol_m);
    p.panel_sweep_m = declare_parameter<std::vector<double>>("bc_panel_sweep_m", p.panel_sweep_m);
    p.renudge_m = declare_parameter<double>("bc_renudge_m", p.renudge_m);
    p.approach_min_m = declare_parameter<double>("bc_approach_min_m", p.approach_min_m);
    if (pa.size() == 3 && map_ok_) {
      const auto c = arena_point_to_map(dice_map_.map_from_arena, {pa[0], pa[1]});
      p.panel = true;
      p.panel_c = bc::V3{c[0], c[1], pa[2]};
      p.panel_n_yaw = std::atan2(std::sin(dice_map_.map_from_arena.yaw), std::cos(dice_map_.map_from_arena.yaw));   // 경기장 +X (벽 → 안쪽)
      if (ps.size() == 2) {p.panel_w_m = ps[0]; p.panel_h_m = ps[1];}
      RCLCPP_INFO(get_logger(), "바코드 벽 판: 경기장 (%.3f, %.3f, 높이 %.2f) → map (%.3f, %.3f) · 폭 %.2f 높이 %.2f (+%.2f) · 법선 %.1f° · "
        "판 밖 광각 상자 버림 · 돌기 없음 · 판 훑기 %zu 자리", pa[0], pa[1], pa[2], c[0], c[1], p.panel_w_m, p.panel_h_m, p.panel_margin_m,
        p.panel_n_yaw * 180.0 / M_PI, p.panel_sweep_m.size());
    }
    const auto wide_topic = declare_parameter<std::string>("bc_wide_objects_topic", "/perception/wide/objects");
    const auto wrist_topic = declare_parameter<std::string>("bc_wrist_topic", "/perception/barcode/detections");
    const auto mode_topic = declare_parameter<std::string>("bc_motion_mode_topic", "/control/motion_mode");
    const auto scans = declare_parameter<std::vector<std::string>>("bc_scan_topics", {"/lidar_front/scan"});
    if (!barcode_search_) {
      RCLCPP_INFO(get_logger(), "바코드: 고정 자리 (barcode_search false)");
      return;
    }
    sub_bc_wide_ = create_subscription<interfaces::msg::DetectionArray>(wide_topic, 10,
        [this](interfaces::msg::DetectionArray::ConstSharedPtr m) {
          const double now = now_s();
          if (bc_active_ && now > bc_wide_valid_from_s_) {   // 찾기 · 몸 옮긴 뒤 받은 첫 · 마지막 장
            if (bc_wide_first_s_ < 0.0) {bc_wide_first_s_ = now;}
            bc_wide_last_s_ = now;
          }
          float best_b = -1.f, best_l = -1.f;
          for (const auto & d : m->detections) {
            const bc::Box b{0.5 * (d.bbox[0] + d.bbox[2]), 0.5 * (d.bbox[1] + d.bbox[3]), d.bbox[2] - d.bbox[0], d.bbox[3] - d.bbox[1], now, ""};
            if (d.kind == "barcode" && d.confidence > best_b && now > bc_wide_valid_from_s_) {best_b = d.confidence; bc_wide_bc_ = b;}
            if (d.kind == "traffic_light" && d.confidence > best_l) {best_l = d.confidence; bc_wide_light_ = b; bc_light_seen_s_ = now;}
          }
        });
    sub_bc_wrist_ = create_subscription<interfaces::msg::DetectionArray>(wrist_topic, 10,
        [this](interfaces::msg::DetectionArray::ConstSharedPtr m) {on_bc_wrist(*m);});
    sub_bc_mode_ = create_subscription<interfaces::msg::MotionMode>(mode_topic, rclcpp::QoS(1).reliable().transient_local(),
        [this](interfaces::msg::MotionMode::ConstSharedPtr m) {
          // control_master 는 거절 뒤 3 s 동안 reason "거절: LOOK_AT: …" 를 실음 → 받을 때마다 시각 (같은 글이 다시 와도)
          if (m->reason.find("LOOK_AT:") != std::string::npos || m->reason.find("LOOK_AT[") != std::string::npos) {   // 꼬리표 판 "LOOK_AT[REACH] …"
            bc_mode_reason_ = m->reason;
            bc_mode_reason_s_ = now_s();
          }
        });
    bc_scan_pts_.resize(scans.size());
    for (std::size_t i = 0; i < scans.size(); ++i) {
      sub_bc_scan_.push_back(create_subscription<sensor_msgs::msg::LaserScan>(scans[i], rclcpp::SensorDataQoS(),
          [this, i](sensor_msgs::msg::LaserScan::ConstSharedPtr m) {on_bc_scan(i, *m);}));
    }
    RCLCPP_INFO(get_logger(), "바코드 찾기 켬: 광각 %s · 손목 %s · 거리 %.2f [%.2f, %.2f] m · 광각 숙임 +%.1f° · 신호등 여유 %.0f° · 최대 %.0f s%s",
      wide_topic.c_str(), wrist_topic.c_str(), p.standoff_m, p.standoff_min_m, p.standoff_max_m, bc_wide_pitch_offset_deg_, p.light_margin_deg,
      p.max_s, bc_fallback_fixed_ ? " · 못 찾으면 예비 고정 경로" : "");
  }

  // pose (frame) → target 프레임 Pose3 (TF 최신)
  bool bc_tf(const std::string & target, const std::string & frame, bc::Pose3 & out)
  {
    geometry_msgs::msg::PoseStamped in, o;
    in.header.frame_id = frame;
    in.pose.orientation.w = 1.0;
    if (!exec_->transform_pose(in, target, o)) {return false;}
    const auto & q = o.pose.orientation;
    out = bc::from_quat(q.x, q.y, q.z, q.w, {o.pose.position.x, o.pose.position.y, o.pose.position.z});
    return true;
  }

  void on_bc_scan(std::size_t i, const sensor_msgs::msg::LaserScan & m)
  {
    if (!bc_active_) {return;}
    auto it = bc_scan_tf_.find(m.header.frame_id);
    if (it == bc_scan_tf_.end()) {
      bc::Pose3 T;
      if (!bc_tf("base_link", m.header.frame_id, T)) {return;}
      it = bc_scan_tf_.emplace(m.header.frame_id, T).first;
    }
    std::vector<std::array<double, 2>> pts;
    pts.reserve(m.ranges.size());
    for (std::size_t k = 0; k < m.ranges.size(); ++k) {
      const double r = m.ranges[k];
      if (!std::isfinite(r) || r < m.range_min || r > m.range_max) {continue;}
      const double a = m.angle_min + k * m.angle_increment;
      const auto p = bc::apply(it->second, {r * std::cos(a), r * std::sin(a), 0.0});
      pts.push_back({p.x, p.y});
    }
    bc_scan_pts_[i] = std::move(pts);
  }

  // 손목 바코드 상자 (perception barcode.detections_topic): 머무는 중 (LOOK_AT 끝 + settle 뒤) 장만
  void on_bc_wrist(const interfaces::msg::DetectionArray & m)
  {
    if (!bc_active_ || !bc_dwell_on_ || now_s() < bc_dwell_from_s_) {return;}
    const std::string frame = m.header.frame_id.empty() ? std::string("d435_color_optical_frame") : m.header.frame_id;
    const interfaces::msg::Detection * best = nullptr;
    for (const auto & d : m.detections) {
      if (d.kind == "wall" && std::isfinite(d.pose.position.z)) {
        geometry_msgs::msg::PoseStamped in, o;
        in.header.frame_id = frame;
        in.pose.position = d.pose.position;
        in.pose.orientation.w = 1.0;
        if (exec_->transform_pose(in, exec_->map_frame(), o)) {bc_wall_map_ = bc::V3{o.pose.position.x, o.pose.position.y, o.pose.position.z};}
      }
      if (d.kind != "barcode") {continue;}
      const bool better = !best || (!d.label.empty() && best->label.empty()) ||
        (d.label.empty() == best->label.empty() && (d.bbox[2] - d.bbox[0]) > (best->bbox[2] - best->bbox[0]));
      if (better) {best = &d;}
    }
    if (!best) {return;}
    bc::Pose3 cam;
    if (!bc_tf(exec_->map_frame(), frame, cam)) {return;}
    bc_wrist_box_ = bc::Box{0.5 * (best->bbox[0] + best->bbox[2]), 0.5 * (best->bbox[1] + best->bbox[3]), best->bbox[2] - best->bbox[0],
      best->bbox[3] - best->bbox[1], now_s(), best->label};
    bc_T_map_wrist_ = cam;
    bc_wrist_pt_.reset();
    if (std::isfinite(best->pose.position.z) && best->pose.position.z > 0.05) {   // 손목 깊이 = 벽 거리 1 순위
      bc_wrist_pt_ = bc::apply(cam, {best->pose.position.x, best->pose.position.y, best->pose.position.z});
    }
  }

  bc::World bc_world(double now)
  {
    bc::World w;
    w.now = now;
    bc::Pose3 Tmb;
    if (bc_tf(exec_->map_frame(), "base_link", Tmb)) {
      w.have_pose = true;
      w.rx = Tmb.t.x;
      w.ry = Tmb.t.y;
      w.ryaw = std::atan2(Tmb.R[1][0], Tmb.R[0][0]);
    }
    if (!bc_wide_tf_ok_) {bc_wide_tf_ok_ = bc_tf("base_link", bc_wide_frame_, bc_T_base_wide_);}
    w.have_wide_tf = bc_wide_tf_ok_;
    w.T_base_wide = bc::yaw_optical(bc::pitch_optical(bc_T_base_wide_, bc_wide_pitch_offset_deg_ * M_PI / 180.0),
      bc_wide_yaw_offset_deg_ * M_PI / 180.0);
    w.wide_barcode = bc_wide_bc_;
    w.wide_light = bc_wide_light_;
    w.wrist_barcode = bc_wrist_box_;
    w.wrist_barcode_map = bc_wrist_pt_;
    w.wrist_center_map = bc_wall_map_;
    w.have_wrist_tf = bc_wrist_box_.has_value();
    w.T_map_wrist = bc_T_map_wrist_;
    for (const auto & v : bc_scan_pts_) {w.scan_base.insert(w.scan_base.end(), v.begin(), v.end());}
    w.signal_done = bc_signal_done();
    w.hold_base = bc_hold_base();
    w.start_yaw = dice_map_.start.yaw;
    w.wide_ready = (bc_wide_first_s_ >= 0.0 && now - bc_wide_first_s_ >= bc_wide_settle_s_ && now - bc_wide_last_s_ <= 0.6) ||
      now - std::max(bc_t0_, bc_wide_valid_from_s_) > bc_wide_wait_s_ ||
      (!signal_gate_open() && !barcode_first_wide_);   // 바코드 먼저 + 광각 끔: 광각 상자를 기다리지 않음
    return w;
  }

  // 벽 판: 바코드 찾기가 아직 남음 (찾기 켬 · 고정 바코드 아님 · 아직 안 읽음 · 대기 또는 HOME · READ_BARCODE 단계)
  bool bc_wide_needed() const
  {
    if (!barcode_search_ || !bc_keep_wide_ || !barcode_fixed_.empty() || bar_i_ >= 1) {return false;}
    if (pending_task_ || queued_task_ || current_task_) {return false;}
    return op_ == Op::STANDBY || (op_ == Op::RUNNING && (phase_ == Phase::HOME || phase_ == Phase::READ_BARCODE));
  }

  // 벽 판: 캡처를 끈 뒤 다시 켰으면 v4l2 밝기를 다시 (bringup.launch.py 와 같은 v4l2-ctl, by-id → 실제 장치). 기다리지 않음
  void reapply_wide_brightness()
  {
    if (wide_device_.empty()) {return;}
    std::error_code ec;
    const auto dev = std::filesystem::canonical(wide_device_, ec);
    if (ec) {
      RCLCPP_WARN(get_logger(), "광각 밝기 다시 넣기 못 함: 장치 %s 없음 (%s)", wide_device_.c_str(), ec.message().c_str());
      return;
    }
    if (dev.string().find('\'') != std::string::npos) {return;}
    const std::string cmd = "sleep 0.5; v4l2-ctl -d '" + dev.string() + "' -c brightness=" + std::to_string(wide_brightness_) + " 2>&1";
    auto logger = get_logger();
    RCLCPP_INFO(logger, "광각 캡처 다시 켬 → 밝기 다시 넣음: v4l2-ctl -d %s -c brightness=%d", dev.c_str(), wide_brightness_);
    std::thread([cmd, logger]() {
        const int rc = std::system(cmd.c_str());
        if (rc != 0) {RCLCPP_WARN(logger, "광각 밝기 다시 넣기 실패 (v4l2-ctl 끝 코드 %d)", rc);}
      }).detach();
  }

  // 바코드 먼저: 신호 기록 · 광각을 열어도 되나 (바코드를 읽음 · 고정 바코드 · task 받음 · OBSERVE_SIGNAL)
  bool signal_gate_open() const
  {
    if (!barcode_first_ || !barcode_fixed_.empty() || bar_i_ >= 1) {return true;}
    if (pending_task_ || queued_task_ || current_task_) {return true;}
    return op_ == Op::RUNNING && phase_ == Phase::OBSERVE_SIGNAL;
  }

  // 바코드 찾기가 신호등을 지키지 않아도 되나. barcode_first 면 찾는 동안 신호를 안 보니 늘 true (몸 멈춤 · 신호등 우선 접기 없음)
  bool bc_signal_done() const
  {
    return barcode_first_ || !signal_record_ || !signal_fixed_.empty() || signal_.status() == SignalStatus::Done || signal_timed_out_;
  }

  // 신호 기록이 진행 중 (안 끝남 · 시간 초과 아님 · 고정 신호 아님) → 바코드 찾기가 몸을 옮기거나 돌리지 않음 (광각에 신호등 유지)
  //   몸을 돌리거나 옮기면 신호등이 광각 구석으로 가 노랑 · 초록이 잘림 → 12 점멸을 못 채움
  bool bc_hold_base() const
  {
    return !bc_signal_done() && !signal_.finished();
  }

  void start_bc()
  {
    bc_base_moved_ = false;
    const double now = now_s();
    bc_.reset(bc_params_, bc_wide_, bc_wrist_, bc_zone_, now);
    bc_panel_rej_logged_ = 0;
    bc_active_ = true;
    bc_t0_ = now;
    bc_wide_tf_ok_ = false;
    bc_scan_tf_.clear();
    bc_look_sent_s_ = -1.0;
    bc_dwell_on_ = false;
    bc_clear_wrist();
    bc_wide_reset(now);
    bc_arm_out_total_ = 0.0;
    bc_move_planned_ = false;
    RCLCPP_INFO(get_logger(), "READ_BARCODE: 바코드 찾기 시작 (광각 · 라이다 · 손목%s, 신호 기록%s)",
      bc_params_.panel ? " · 벽 판만 · 돌기 없음" : "",
      (barcode_first_ && signal_.status() != SignalStatus::Done && !signal_timed_out_ && signal_fixed_.empty() && signal_record_) ?
      "은 바코드 읽은 뒤 (아직 안 함)" :
      bc_signal_done() ? " 끝" : (bc_hold_base() ? " 중 → 기록 끝까지 몸 멈춤 (팔로만 찾기)" : " 중"));
    set_steps(Phase::READ_BARCODE, {bc_plan_step()});
  }

  // 광각 상자를 이 시각 뒤 받은 것만 씀 (찾기 시작 · 몸 옮긴 뒤 — 옮기기 전 상자를 새 자세로 풀지 않게)
  void bc_wide_reset(double now)
  {
    bc_wide_valid_from_s_ = now;
    bc_wide_first_s_ = -1.0;
    bc_wide_last_s_ = -1.0;
    bc_wide_bc_.reset();
  }

  // 팔을 접음 (HOME 보냄): 펴 둔 시간을 합에 더함
  void bc_arm_folded(double now)
  {
    if (bc_arm_out_since_ >= 0.0) {bc_arm_out_total_ += now - bc_arm_out_since_;}
    bc_arm_out_since_ = -1.0;
  }

  void bc_clear_wrist()
  {
    bc_wrist_box_.reset();
    bc_wrist_pt_.reset();
    bc_wall_map_.reset();
  }

  static Step bc_plan_step()
  {
    Step s;
    s.type = StepType::Custom;
    s.kind = bc::kStepBcPlan;
    s.name = "bc:plan";
    return s;
  }

  static Step bc_dwell_step(double secs, bool after_look)
  {
    Step s;
    s.type = StepType::Custom;
    s.kind = bc::kStepBcDwell;
    s.timeout_s = secs;
    s.ref[0] = after_look ? 1 : 0;
    s.name = after_look ? "bc:dwell (판독 기다림)" : "bc:dwell";
    return s;
  }

  static Step bc_home_step()
  {
    Step s;
    s.type = StepType::Arm;
    s.arm_task = kArmHome;
    s.name = "arm:HOME";
    return s;
  }

  bool bc_resolve(geometry_msgs::msg::PoseStamped & out, std::string & detail)
  {
    if (!bc_target_) {detail = "바코드 찾기 목표 없음"; return false;}
    out = *bc_target_;
    out.header.stamp = get_clock()->now();
    return true;
  }

  // 읽힌 바코드 원문 → task (run_read 와 같은 규칙). true = 끝 (성공 · FAULT), false = 아직
  bool bc_take_text(std::string & detail, StepResult & r)
  {
    if (!(last_text_s_ > bc_t0_)) {return false;}
    std::string err;
    auto task = parse_task(last_text_, barcodes_, &err);
    if (!task) {
      request_fault("BARCODE_UNREGISTERED", err + " (원문: " + last_text_ + ")");
      r = StepResult::Running;
      return true;
    }
    if (done_ids_.count(task->task_id)) {
      detail = "이미 처리한 바코드: " + task->raw_text;
      return false;
    }
    pending_task_ = task;
    bc_active_ = false;
    bc_dwell_on_ = false;
    RCLCPP_INFO(get_logger(), "바코드 찾기: 읽음 %s (%.1f s, LOOK_AT %d 번) → HOME", task->raw_text.c_str(), now_s() - bc_t0_, bc_.looks());
    exec_->insert_after_current({bc_home_step()});
    r = StepResult::Success;
    return true;
  }

  StepResult run_bc_plan(double now, std::string & detail)
  {
    bc_dwell_on_ = false;
    StepResult r;
    if (bc_take_text(detail, r)) {return r;}
    const auto w = bc_world(now);
    auto a = bc_.next(w);
    bc_clear_wrist();
    if (bc_.panel_rejects() > bc_panel_rej_logged_ + 9 || (bc_panel_rej_logged_ == 0 && bc_.panel_rejects() > 0)) {
      bc_panel_rej_logged_ = bc_.panel_rejects();
      RCLCPP_INFO(get_logger(), "바코드 찾기: 벽 판 밖 광각 바코드 상자 버림 (합 %d 번)", bc_panel_rej_logged_);
    }
    {   // 찾기 전체에서 팔을 편 시간 합 상한 → 넘으면 포기 (예비 고정 경로 · 또는 접고 끝)
      const double out = bc_arm_out_total_ + (bc_arm_out_since_ >= 0.0 ? now - bc_arm_out_since_ : 0.0);
      if (a.kind == bc::Act::LookAt && out > bc_arm_out_total_s_) {
        a.kind = bc::Act::GiveUp;
        char b[120];
        std::snprintf(b, sizeof(b), "팔을 편 시간 합 %.0f s > %.0f (bc_arm_out_total_s)", out, bc_arm_out_total_s_);
        a.note = b;
      }
    }
    // 예비 고정 경로 (barcode_view 로 이동) 는 몸을 옮김 → 신호 기록 중 (몸 멈춤) 이면 기록 끝까지 기다린 뒤 포기
    if (a.kind == bc::Act::GiveUp && bc_fallback_fixed_ && bc_.holding(w)) {
      a = bc_.hold_wait(w, 2.0, "포기 (" + a.note + ") → 예비 고정 경로는 몸을 옮김 → 신호 기록 끝 기다림");
    }
    std::vector<Step> more;
    switch (a.kind) {
      case bc::Act::LookAt: {
        geometry_msgs::msg::PoseStamped t;
        t.header.frame_id = exec_->map_frame();
        t.pose.position.x = a.target.x;
        t.pose.position.y = a.target.y;
        t.pose.position.z = a.target.z;   // map z = 바닥 위 → 실행기가 base_link 로
        t.pose.orientation.w = 1.0;
        bc_target_ = t;
        char slot[256];
        int n = std::snprintf(slot, sizeof(slot), "d=%.3f", a.standoff);
        if (std::isfinite(a.view_yaw) && w.have_pose) {
          n += std::snprintf(slot + n, sizeof(slot) - n, ",vyaw=%.1f", std::remainder(a.view_yaw - w.ryaw, 2 * M_PI) * 180.0 / M_PI);
        }
        {   // 위에서 내려다봄 (실물: 15 cm · 43° 아래에서 읽힘). 훑기는 덜 (팔이 닿게)
          const double vp = a.sweep ? bc_sweep_view_pitch_deg_ : bc_view_pitch_deg_;
          if (std::isfinite(vp)) {n += std::snprintf(slot + n, sizeof(slot) - n, ",vpitch=%.1f", vp);}
        }
        if (a.dj1_deg != 0.0 || a.dj4_deg != 0.0) {n += std::snprintf(slot + n, sizeof(slot) - n, ",dj1=%.1f,dj4=%.1f", a.dj1_deg, a.dj4_deg);}
        if (a.light && bc_.light_known() && w.have_pose && w.have_wide_tf) {   // 광각 → 신호등 방향 (base_link) — 팔이 가리지 않게
          const auto Tmb = bc::planar(w.rx, w.ry, w.ryaw, bc_params_.base_z_m);
          const bc::V3 lb = bc::V3{a.light_map.x - Tmb.t.x, a.light_map.y - Tmb.t.y, a.light_map.z - Tmb.t.z};
          const bc::V3 l_base{std::cos(w.ryaw) * lb.x + std::sin(w.ryaw) * lb.y, -std::sin(w.ryaw) * lb.x + std::cos(w.ryaw) * lb.y, lb.z};
          const bc::V3 d = l_base - w.T_base_wide.t;
          // 원뿔 반각 = 신호등 상자 반 대각선 (광각 화소 → 각) + 여유
          double cone = bc_light_cone_min_deg_;
          if (bc_wide_light_) {
            cone = std::max(cone, std::atan(0.5 * std::hypot(bc_wide_light_->w, bc_wide_light_->h) / bc_wide_.fx) * 180.0 / M_PI +
              bc_light_cone_margin_deg_);
          }
          std::snprintf(slot + n, sizeof(slot) - n, ",lyaw=%.1f,lpitch=%.1f,lcone=%.1f", std::atan2(d.y, d.x) * 180.0 / M_PI,
            std::atan2(d.z, std::hypot(d.x, d.y)) * 180.0 / M_PI, cone);
        }
        Step lk;
        lk.type = StepType::Arm;
        lk.arm_task = kArmLookAtBc;
        lk.target = Target::Custom;
        lk.slot = slot;
        bc_last_slot_ = slot;
        lk.timeout_s = bc_look_timeout_s_;
        lk.name = std::string("arm:LOOK_AT ") + (a.sweep ? "훑기" : "바코드");
        if (bc_arm_out_since_ >= 0.0 && now - bc_arm_out_since_ > bc_max_arm_out_s_) {   // 오래 펴 둠 → 접고 쉰 뒤
          RCLCPP_INFO(get_logger(), "바코드 찾기: 팔을 %.0f s 펴 둠 > %.0f → HOME · %.1f s 쉼 뒤 LOOK_AT", now - bc_arm_out_since_,
            bc_max_arm_out_s_, bc_fold_rest_s_);
          more = {bc_home_step(), bc_dwell_step(bc_fold_rest_s_, false)};
          bc_arm_folded(now);
        }
        more.push_back(lk);
        more.push_back(bc_dwell_step(a.dwell_s, true));
        more.push_back(bc_plan_step());
        bc_light_before_look_ = bc_light_seen_s_ > 0.0 && now - bc_light_seen_s_ < 1.0;
        RCLCPP_INFO(get_logger(), "바코드 찾기 (%.1f s): %s · slot '%s'", now - bc_t0_, a.note.c_str(), slot);
        break;
      }
      case bc::Act::MoveBase: {
        Pose2D g;
        g.x = a.bx;
        g.y = a.by;
        g.yaw = a.byaw;
        Step nav;   // 출발 구역 안 작은 이동 · 제자리 돌기 (정밀 접근 없음)
        nav.type = StepType::Navigate;
        nav.target = Target::Fixed;
        nav.pose = g;
        nav.name = "nav:barcode_search";
        bc_base_moved_ = true;   // 바코드 찍으러 몸을 옮김 → 신호 기록이 남았으면 출발 자리로 돌아가서 (begin_observe)
        more = {bc_home_step(), nav, bc_dwell_step(a.dwell_s, false), bc_plan_step()};
        bc_arm_folded(now);
        bc_move_planned_ = true;   // 옮긴 뒤 머묾에서 광각 상자를 새로 받음 (S1)
        RCLCPP_INFO(get_logger(), "바코드 찾기 (%.1f s): %s", now - bc_t0_, a.note.c_str());
        break;
      }
      case bc::Act::Dwell:   // 기다릴 때 팔은 접어 둠 (펴 둔 채 기다리지 않음)
        // 14시 실물: 신호 기록 중 몸 멈춤 기다림이 2 s 마다 HOME 을 다시 보내 (22 번) 쉬는 팔 토크가 켜졌다 꺼졌다 → 팔이 이상하게 움찔
        //   → 이미 접혀 있으면 (팔 나간 시각 없음) HOME 없이 기다리기만
        if (bc_arm_out_since_ >= 0.0) {
          more = {bc_home_step(), bc_dwell_step(a.dwell_s, false), bc_plan_step()};
        } else {
          more = {bc_dwell_step(a.dwell_s, false), bc_plan_step()};
        }
        bc_arm_folded(now);
        RCLCPP_INFO(get_logger(), "바코드 찾기 (%.1f s): %s", now - bc_t0_, a.note.c_str());
        break;
      case bc::Act::GiveUp:
        bc_active_ = false;
        if (bc_fallback_fixed_) {
          RCLCPP_WARN(get_logger(), "바코드 찾기 포기 (%s, %.1f s) → 예비 고정 경로 (barcode_view + LOOK_BARCODE)", a.note.c_str(), now - bc_t0_);
          more = {bc_home_step()};
          bc_base_moved_ = true;
          for (auto & st : build_read_steps(dice_map_.barcode_view, read_timeout_s_, barcode_look_)) {more.push_back(st);}
          break;
        }
        // 예비 고정 경로 없이 포기해도 팔은 접음 → 읽기 실패로 끝 (read_retries 다 쓴 것으로, 다시 읽기 안 함)
        RCLCPP_WARN(get_logger(), "바코드 찾기 포기 (%s, %.1f s) → HOME · 바코드 없이 (bc_fallback_fixed false)", a.note.c_str(), now - bc_t0_);
        read_tries_ = read_retries_;
        {
          Step rd;
          rd.type = StepType::Custom;
          rd.kind = kReadBarcode;
          rd.timeout_s = 0.05;
          rd.name = "read_barcode";
          more = {bc_home_step(), rd};
        }
        bc_arm_folded(now);
        break;
    }
    exec_->insert_after_current(more);
    return StepResult::Success;
  }

  // 머묾: 읽히면 바로 끝. LOOK_AT 뒤 (ref[0] 1) 면 settle 뒤 손목 상자를 모으고, 신호 기록 중 신호등이 bc_light_lost_s 넘게 안 보이면
  //   팔이 가린 것 → 찾기에 알리고 HOME 먼저 (신호등 우선)
  StepResult run_bc_dwell(const Step & s, double now, double started)
  {
    if (!bc_dwell_on_) {
      bc_dwell_on_ = true;
      bc_dwell_from_s_ = started + (s.ref[0] == 1 ? bc_params_.settle_s : 0.0);
      if (bc_move_planned_ && s.ref[0] == 0) {   // 몸을 옮긴 뒤 첫 머묾: 옮기기 전 광각 상자는 버리고 새로 받음 (S1)
        bc_move_planned_ = false;
        bc_wide_reset(now);
      }
    }
    std::string d;
    StepResult r;
    if (last_text_s_ > bc_t0_) {bc_snapshot(s, "read"); return StepResult::Success;}   // bc:plan 이 처리
    if (s.ref[0] == 1 && !bc_signal_done() && bc_light_before_look_ && now - started > bc_light_lost_s_ &&
      (bc_light_seen_s_ < 0.0 || now - bc_light_seen_s_ > bc_light_lost_s_))
    {
      RCLCPP_WARN(get_logger(), "바코드 찾기: LOOK_AT 뒤 신호등이 광각에서 %.1f s 안 보임 → 팔이 가림, 접고 다음 (신호등 우선)",
        now - std::max(bc_light_seen_s_, started));
      bc_snapshot(s, "light_lost");
      bc_.on_look_failed("신호등 가림 (광각에서 안 보임)", bc_world(now));
      exec_->insert_after_current({bc_home_step()});
      return StepResult::Success;
    }
    if (now - started >= s.timeout_s) {bc_snapshot(s, bc_wrist_box_ ? "box" : "nobox"); return StepResult::Success;}
    (void)d;
    (void)r;
    return StepResult::Running;
  }

  void bc_step_failed(const Step & s, bool need_home)
  {
    const auto w = bc_world(now_s());
    if (s.type == StepType::Arm && s.arm_task == kArmLookAtBc) {
      bc_.on_look_failed(detail_, w);
    } else if (s.type == StepType::Navigate) {
      bc_.on_move_failed(detail_);
    }
    RCLCPP_WARN(get_logger(), "바코드 찾기: %s 실패 (%s) → %s", s.name.c_str(), detail_.c_str(), bc_.last_note().c_str());
    std::vector<Step> steps;
    if (need_home || s.type == StepType::Arm) {steps.push_back(bc_home_step());}
    steps.push_back(bc_plan_step());
    set_steps(Phase::READ_BARCODE, std::move(steps));
  }

  // LOOK_AT 머묾 끝마다 손목 컬러 한 장 (perception 이 다음 장을 jpg 로) + 옆 글 (slot · 목표 · 몸 · 손목 상자)
  //   <record_dir>/run_<시각>/barcode/lookNN_<결과>.jpg · .txt. bc_snapshot_topic "" = 안 함
  void bc_snapshot(const Step & s, const char * what)
  {
    if (s.ref[0] != 1 || !pub_bc_snap_) {return;}
    const std::string dir = run_dir();
    if (dir.empty()) {return;}
    std::error_code ec;
    std::filesystem::create_directories(dir + "/barcode", ec);
    char name[96];
    std::snprintf(name, sizeof(name), "/barcode/look%02d_%s", bc_.looks(), what);
    const std::string base = dir + name;
    std_msgs::msg::String m;
    m.data = base + ".jpg";
    pub_bc_snap_->publish(m);
    const auto w = bc_world(now_s());
    std::ofstream f(base + ".txt");
    f << "t " << now_s() - bc_t0_ << "\nslot " << bc_last_slot_ << "\n";
    if (bc_target_) {f << "target_map " << bc_target_->pose.position.x << " " << bc_target_->pose.position.y << " " << bc_target_->pose.position.z << "\n";}
    f << "robot_map " << w.rx << " " << w.ry << " " << w.ryaw << "\n";
    if (bc_wrist_box_) {f << "wrist_box u " << bc_wrist_box_->u << " v " << bc_wrist_box_->v << " w " << bc_wrist_box_->w << " text '" << bc_wrist_box_->text << "'\n";}
    if (bc_wrist_pt_) {f << "wrist_depth_map " << bc_wrist_pt_->x << " " << bc_wrist_pt_->y << " " << bc_wrist_pt_->z << "\n";}
    f << "note " << bc_.last_note() << "\n";
  }

  // LOOK_AT 거절 (control_master 가 /control/motion_mode reason 에 '거절: LOOK_AT: …' 를 3 s 실음) → 실행기 15 s 다시 보내기를 기다리지 않음
  //   보낸 뒤 받은 그 글 + 팔 작업이 진행 중 아님 (거절됨, 실행기 arm_busy 거짓) 일 때만 → 앞 거절의 남은 글로 새 목표를 버리지 않음
  void bc_check_reject(double now)
  {
    if (!bc_active_ || op_ != Op::RUNNING || phase_ != Phase::READ_BARCODE || bc_look_sent_s_ < 0.0) {return;}
    if (!(bc_mode_reason_s_ > bc_look_sent_s_ + 0.05) ||
      (bc_mode_reason_.find("LOOK_AT:") == std::string::npos && bc_mode_reason_.find("LOOK_AT[") == std::string::npos)) {return;}
    if (exec_->arm_busy() || exec_->current_step_name().rfind("arm:LOOK_AT", 0) != 0) {return;}
    bc_look_sent_s_ = -1.0;
    bc_.on_look_failed(bc_mode_reason_, bc_world(now));
    RCLCPP_WARN(get_logger(), "바코드 찾기: LOOK_AT 거절 (%s) → %s", bc_mode_reason_.c_str(), bc_.last_note().c_str());
    set_steps(Phase::READ_BARCODE, {bc_plan_step()});
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
      // 손목 박스 측정이 아직 모자라거나 그림용 영상이 LOOK 뒤 한 장도 안 왔으면 box_wait_s 까지 더 기다림
      //   (재검증 1 s 만으로는 5 Hz 측정 3 개가 빠듯함, 영상은 큰 메시지라 몇 장 빠질 수 있음, 팔 움직이던 장은 안 씀)
      const bool want_box = box_measure_ && !measured_place(place_of(s), started, false);
      const bool want_img = pub_verify_image_ &&
        (!verify_frame_ || rclcpp::Time(verify_frame_->header.stamp).seconds() < started);   // LOOK 끝난 뒤 장
      if ((want_box || want_img) && now - started < box_wait_s_) {return StepResult::Running;}
      verify_place(place_of(s), started, now);
      return StepResult::Success;   // 판정이 나빠도 미션은 계속 (기록이 목적)
    }
    if (s.kind == kPlanDelivery && box_of(s) < 1) {   // 재계획 (마지막 박스 이동 실패 등): 지금 자리에서
      if (!plan_from_state(false, now)) {return StepResult::Fail;}
      if (phase_ == Phase::DELIVER) {   // DELIVER 중 손 비운 뒤 다시 계획: 새 배달 단계를 바로 뒤에
        exec_->insert_after_current(std::move(delivery_steps_));
        delivery_steps_.clear();
      }
      return StepResult::Success;
    }
    const int box = box_of(s);
    if (box < 1 || box > kNumBoxes) {
      detail = "Plan 박스 번호 잘못됨";
      return StepResult::Fail;
    }
    // 그 박스 LOOK 을 시작한 뒤 본 퍽 (다시 계획할 때도 같은 LOOK 결과를 씀: 복구 HOME 뒤엔 카메라가 박스를 안 봄)
    //   옆 보기 LOOK · 다시 보기 LOOK 은 look_started_ 를 안 바꿈 → 옆 보기 · 빈손 뒤 다시 보기 뒤에도 앞 LOOK 부터 본 퍽 (앞 + 옆 합침)
    const double look_t = look_started_[box - 1];
    const double since = look_t > 0.0 ? look_t - 0.5 : started - 2.0;
    if (box_recenter_on_look_) {recenter_box_on_look(box, since);}
    const auto at_box = pucks_at_box(box, since);
    RCLCPP_INFO(get_logger(), "B%d LOOK: 퍽 %zu 개 (%s)", box, at_box.size(), puck_list(at_box).c_str());
    // 옆 보기: 이 정차 첫 Plan 에서 퍽이 모자라면 계획 전에 옆을 보고 같은 Plan 을 다시 (그때는 본 만큼으로 계획)
    const bool side_merged = side_look_[box - 1] == SideLookState::Pending;
    //   이미 이 박스에서 다룬 퍽 (집어 실음 · 건너뜀 등 기록부 on_field 아님, 마지막 자리 = 이 박스) 은 보인 것으로 셈
    //   (누운 퍽을 뒤로 빼서 집은 뒤 다시 붙은 LOOK 에서 남은 퍽만 세면 헛 옆 보기를 함)
    const std::size_t handled = handled_at_box(box, at_box);
    if (auto more = side_look_on_plan(side_look_[box - 1], s, at_box.size() + handled, side_cfg_); !more.empty()) {
      std::string slots;
      for (const auto & m : more) {
        if (is_side_look(m)) {slots += (slots.empty() ? "" : ", ") + m.slot;}
      }
      RCLCPP_INFO(get_logger(), "B%d LOOK: 퍽 %zu 개 + 이미 다룬 퍽 %zu 개 < %d → 옆 보기 (%s)", box, at_box.size(), handled,
        side_cfg_.expected, slots.c_str());
      exec_->insert_after_current(std::move(more));
      return StepResult::Success;
    }
    if (side_merged) {
      RCLCPP_INFO(get_logger(), "B%d 옆 보기 뒤: 앞 + 옆 퍽 %zu 개로 계획 (%s)", box, at_box.size(), puck_list(at_box).c_str());
    }
    if (relook_before_pick_) {update_puck_views(box, at_box);}
    // STORE 가 실패한 쥔 퍽: 다른 칸 · 박스 되돌림부터 하고 같은 Plan 다시 (실패 칸으로 다시 계획하지 않음)
    if (auto first = held_first(s); !first.empty()) {
      exec_->insert_after_current(std::move(first));
      return StepResult::Success;
    }

    if (box >= 1 && box <= kNumBoxes && lying_fwd_[box - 1]) {   // 뒤로 뺀 자리 → 걸린 퍽만 집고 다시 앞으로
      if (lying_forward_plan(s, box, at_box)) {return StepResult::Success;}
    }
    if (s.kind == kPlanCollectBox) {
      std::string err;
      auto actions = collect_box_actions(at_box, exec_->cargo(), &err);
      if (!err.empty()) {
        detail = "B" + std::to_string(box) + " 집기 계획 실패: " + err;
        return StepResult::Fail;
      }
      keep_spare_cell(actions, box);
      auto arm = arm_steps(actions, rack_look_, box_measure_);
      {   // 이 정차에서 바로 집을 퍽 (계획 순서) 중 누운 퍽이 너무 가까우면 뒤로 뺀 뒤 다시
        std::vector<std::string> ids;
        for (const auto & st : arm) {
          if (st.type == StepType::Arm && st.arm_task == kArmPick && !st.puck_id.empty()) {ids.push_back(st.puck_id);}
        }
        if (start_lying_backoff(s, box, ids, since)) {return StepResult::Success;}
      }
      if (relook_before_pick_) {arm = with_pick_relooks(arm, held_arm_step(), puck_views_);}
      exec_->insert_after_current(std::move(arm));
      return StepResult::Success;
    }
    {   // 마지막 박스 (배달 계획): 계획 (할당 기록) 전에 이 박스에서 본 퍽으로 봄
      std::vector<std::string> ids;
      for (const auto & p : at_box) {ids.push_back(p.id);}
      if (start_lying_backoff(s, box, ids, since)) {return StepResult::Success;}
    }
    return plan_from_state(true, now) ? StepResult::Success : StepResult::Fail;   // kPlanDelivery
  }

  // 박스 LOOK 뒤 박스 중심 다시 잡기 (box_recenter_on_look): since 뒤에 본 on_field 퍽 중 지도 중심 box_relook_radius_m 안 평균
  //   → 새 중심 box_radius 안 퍽이 더 많을 때만 옮김 (box_relook 과 같이 dice_map_ 박스 중심을 바꿈 → 이 박스 퍽 고르기 · 재계획도 새 중심)
  void recenter_box_on_look(int box, double since)
  {
    auto & bx = dice_map_.boxes[box - 1];
    std::vector<std::array<double, 2>> pts;
    for (const auto & r : exec_->registry().records()) {
      if (r.state == "on_field" && r.last_seen >= since) {pts.push_back({r.x, r.y});}
    }
    const auto rc = recenter_box_on_pucks(bx.center, pts, dice_map_.box_radius, box_relook_radius_m_, box_relook_max_shift_m_);
    if (!rc) {return;}
    RCLCPP_WARN(get_logger(), "B%d LOOK: 지도 박스 중심 %.0f cm 안 퍽 %d 개 → 둘레 %.0f cm 안 퍽 %d 개 평균으로 박스 중심 다시 잡음 "
      "(%.3f, %.3f) → (%.3f, %.3f), %.1f cm 옮김 → 퍽 %d 개", box, dice_map_.box_radius * 100.0, rc->before,
      box_relook_radius_m_ * 100.0, rc->used, bx.center[0], bx.center[1], rc->center[0], rc->center[1], rc->shift * 100.0, rc->after);
    bx.center = rc->center;
  }

  // 박스 중심 box_radius 안, since 뒤에 본 on_field 퍽 (ID · 색 · 모양 다 아는 것)
  // 이 task 에서 박스를 잰 장소 (place_measured_) 의 배달 정차 LOOK · 박스 측정 (kPlaceBox) 단계를 뺌 → 그 장소 기준 (place_frame_, 지도 좌표) 그대로.
  //   (재계획마다 '다시 도착' 으로 LOOK + 측정을 하면 박스는 그대로인데 매번 5 s). 못 잼 · 다시 붙기 확인 중이면 그대로 둠
  void skip_measured_place_looks(std::vector<Step> & steps)
  {
    if (!box_measure_) {return;}
    std::vector<Step> out;
    out.reserve(steps.size());
    for (std::size_t i = 0; i < steps.size(); ++i) {
      const Step & s = steps[i];
      const int p = place_of(s);
      const bool place_look = s.type == StepType::Arm && is_look_task(s.arm_task) && s.slot.empty() && box_of(s) <= 0 &&
        p >= 0 && p < kNumPlaces && place_measured_[p] && place_frame_[p];
      if (!place_look) {out.push_back(s); continue;}
      const auto & f = *place_frame_[p];
      RCLCPP_INFO(get_logger(), "L%d: 이 task 에서 잰 박스 (측정 %d 개, 지도 중심 대비 (%+.1f, %+.1f) mm · %+.1f°) 그대로 → LOOK · 박스 측정 건너뜀",
        p + 1, f.n, 1000.0 * (f.x - dice_map_.places[p].center[0]), 1000.0 * (f.y - dice_map_.places[p].center[1]), f.dyaw * 180.0 / M_PI);
      if (i + 1 < steps.size() && is_custom(steps[i + 1], kPlaceBox) && place_of(steps[i + 1]) == p) {++i;}   // 바로 뒤 박스 측정도
    }
    steps = std::move(out);
  }

  // 이 박스에서 이미 다룬 퍽 수 (기록부 on_field 아님 · 마지막 자리 = 이 박스 반경 안, 지금 보인 퍽 빼고): 옆 보기 기대 개수에서 뺌
  std::size_t handled_at_box(int box, const std::vector<TransferPuck> & seen) const
  {
    if (box < 1 || box > kNumBoxes) {return 0;}
    const auto & c = dice_map_.boxes[box - 1].center;
    std::size_t n = 0;
    for (const auto & r : exec_->registry().records()) {
      if (r.id.empty() || r.state == "on_field" || std::hypot(r.x - c[0], r.y - c[1]) > dice_map_.box_radius) {continue;}
      if (std::any_of(seen.begin(), seen.end(), [&r](const TransferPuck & p) {return p.id == r.id;})) {continue;}
      ++n;
    }
    return n;
  }

  std::vector<TransferPuck> pucks_at_box(int box, double since) const
  {
    std::vector<TransferPuck> out;
    const auto & c = dice_map_.boxes[box - 1].center;
    for (const auto & r : exec_->registry().records()) {
      if (r.last_seen < since || std::hypot(r.x - c[0], r.y - c[1]) > dice_map_.box_radius) {continue;}
      if (r.state != "on_field") {
        // 이미 실은 ID 가 박스 LOOK 에서 다른 색 · 모양으로 다시 보임 = ID 잘못 읽음 (또는 같은 ID 두 개).
        //   기록부 (planning PuckRegistry::update) 는 상태를 그대로 두므로 이 퍽은 안 집힘 → 알림만
        for (const auto & [slot, it] : exec_->cargo().slots()) {
          if (it && it->puck_id == r.id && (it->color != r.color || it->shape != r.shape)) {
            RCLCPP_WARN(get_logger(), "B%d 퍽 ID %s 를 %s/%s 로 다시 봄인데 이미 %s 칸에 %s/%s 로 실림 → ID 잘못 읽음 의심, 이 퍽은 안 집음",
              box, r.id.c_str(), r.color.c_str(), r.shape.c_str(), slot.c_str(), it->color.c_str(), it->shape.c_str());
          }
        }
        continue;
      }
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
      used_kind_[r.id] = {r.color, r.shape};
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

  // ---------------------------------------------------------------- 누운 퍽이 너무 가까우면 곧게 뒤로 빼고 다시
  // 이 정차 Plan (s) 에서 집을 퍽 ids 중 누운 퍽 (방향 있는 손목 검출) · 중심 x (base_link) < lying_backoff_x_m
  //   또는 선 퍽 (방향 모름 포함) · x < lying_backoff_upright_x_m → Plan 뒤에
  //   [HOME → kLyingBackoff] 를 끼우고 true (이 Plan 은 PICK 을 안 넣음). kLyingBackoff 가 끝나면 [같은 박스 앞 LOOK → 같은 Plan].
  //   박스마다 한 번 (뒤로 간 뒤에도 가까우면 그대로 집음). 퍽을 쥐고 있으면 안 함 (쥔 채 주행 안 함)
  bool start_lying_backoff(const Step & s, int box, const std::vector<std::string> & ids, double since)
  {
    if (!lying_cmd_pub_ || box < 1 || box > kNumBoxes || ids.empty()) {return false;}
    // 측정을 못 믿는 퍽 = 이 task 에서 PICK 이 빈손 · 높이 틀림으로 실패했거나 (empty_grasp_n_),
    //   팔이 지금 앞 LOOK 자세에 서 있는데 lying_unseen_s 넘게 검출이 안 옴 (움직이던 장에서만 봄 = 서 있는 자세 화면 밖 · 깊이 없음)
    //   → 선 퍽도 누운 퍽 기준 (lying_backoff_x_m) 으로 뒤로. 뒤로 빼기는 퍽마다 한 번 (다시 앞으로 붙은 뒤 다른 퍽이 가까우면 그 퍽도 한 번)
    const auto & lw = look_windows_[box - 1];
    const bool front_now = !lw.empty() && lw.back().to < 0.0 && lw.back().slot.empty();
    const double now = now_s();
    std::vector<LyingCandidate> cands, fresh;
    std::string seen;
    for (const auto & id : ids) {
      const auto pl = exec_->puck_local(id);
      if (!pl || pl->seen < since) {continue;}
      const bool lying = pl->oriented_s >= 0.0 && pl->lying;   // 방향 있는 검출 없으면 선 퍽으로 봄
      const auto fn = empty_grasp_n_.find(id);
      const bool failed = fn != empty_grasp_n_.end() && fn->second > 0;
      const bool unseen = front_now && lying_unseen_s_ > 0.0 && now - pl->seen > lying_unseen_s_;
      cands.push_back(LyingCandidate{id, pl->x, lying, failed || unseen});
      if (!lying_backed_ids_[box - 1].count(id)) {fresh.push_back(cands.back());}
      char buf[64];
      std::snprintf(buf, sizeof(buf), "%s%s x %.3f", seen.empty() ? "" : ", ", id.c_str(), pl->x);
      seen += buf + std::string(lying ? " 누움" : pl->oriented_s < 0.0 ? " 선(방향 모름)" : " 선") +
        (failed ? " · 집기 실패함" : "") + (unseen ? " · 지금 자세에서 안 보임" : "");
    }
    const auto c = lying_backoff_choice(fresh, lying_cfg_);
    if (!c) {
      if (const auto old = lying_backoff_choice(cands, lying_cfg_)) {
        RCLCPP_INFO(get_logger(), "B%d 퍽 %s x %.3f m 가깝지만 이 퍽은 이미 뒤로 빼 봄 → 그대로 집음 (%s)", box, old->id.c_str(), old->x,
          seen.c_str());
      }
      return false;
    }
    const char * kind = c->lying ? "누운" : (c->bad ? "측정 못 믿는 선" : "선");
    const double lim = (c->lying || c->bad) ? lying_cfg_.near_x_m : lying_cfg_.up_near_x_m;
    if (exec_->cargo().holding()) {
      RCLCPP_INFO(get_logger(), "B%d %s 퍽 %s x %.3f m 가까움 — 퍽을 쥐고 있어 뒤로 안 감 → 그대로", box, kind, c->id.c_str(), c->x);
      return false;
    }
    lying_backed_ids_[box - 1].insert(c->id);
    // 다시 볼 LOOK = 이 정차 앞 LOOK 복사 (옆 보기 · 다시 보기 아님 → 관측 구간 · 옆 보기 상태 새로 = 뒤로 간 자리 LOOK 결과만 씀)
    Step look;
    look.type = StepType::Arm;
    look.arm_task = kArmLook;
    const auto & st = exec_->steps();
    lying_nav_ok_[box - 1] = false;
    for (std::size_t j = std::min(exec_->index(), st.size()); j-- > 0;) {
      if (st[j].type == StepType::Navigate) {
        if (box_of(st[j]) == box) {lying_nav_[box - 1] = st[j]; lying_nav_ok_[box - 1] = true;}   // 이 박스 정차 이동 (다시 앞으로 갈 때)
        break;
      }
      if (starts_box_look(st[j]) && box_of(st[j]) == box) {look = st[j];}
    }
    lying_fwd_[box - 1] = lying_nav_ok_[box - 1] && lying_forward_;
    lying_trigger_[box - 1] = c->id;
    lying_trig_n_[box - 1] = 0;
    box_of(look) = box;
    look.kind = kNoKind;
    look.puck_id.clear();
    look.slot.clear();
    look.name = "arm:LOOK:B" + std::to_string(box) + "(누운 퍽 뒤로 뺀 뒤)";
    lying_look_ = look;
    lying_plan_ = s;
    Step home;
    home.type = StepType::Arm;
    home.arm_task = kArmHome;
    home.name = "arm:HOME (B" + std::to_string(box) + " 누운 퍽 뒤로 빼기 전)";
    Step back;
    back.type = StepType::Custom;
    back.kind = kLyingBackoff;
    box_of(back) = box;
    back.puck_id = c->id;
    back.pose.x = c->back_m;   // 뒤로 갈 거리 (Custom 단계라 실행기는 pose 를 안 씀)
    char nm[96];
    std::snprintf(nm, sizeof(nm), "custom:B%d %s 퍽 %s 뒤로 %.1f cm", box, kind, c->id.c_str(), c->back_m * 100.0);
    back.name = nm;
    RCLCPP_WARN(get_logger(), "B%d %s 퍽 %s 중심 x %.3f m < %.3f (%s) → 그 자리에서 안 집고 곧게 뒤로 %.1f cm (목표 x %.3f) → LOOK · 계획 다시",
      box, kind, c->id.c_str(), c->x, lim, seen.c_str(), c->back_m * 100.0, lying_cfg_.target_x_m);
    exec_->insert_after_current({home, back});
    return true;
  }

  // collision_monitor 영역 켜기 · 끄기 (비동기, 답은 안 기다림: 끌 때만 Move 전에 최대 1 s 기다림)
  void lying_set_polygons(bool enabled)
  {
    if (!lying_cm_cli_ || lying_polygons_.empty()) {return;}
    if (!lying_cm_cli_->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "누운 퍽 뒤로: %s/set_parameters 없음 → 영역 %s 못 함", lying_cm_node_.c_str(), enabled ? "켜기" : "끄기");
      return;
    }
    auto req = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    for (const auto & n : lying_polygons_) {req->parameters.push_back(rclcpp::Parameter(n + ".enabled", enabled).to_parameter_msg());}
    lying_cm_pending_ = true;
    lying_polys_off_ = !enabled;
    lying_cm_cli_->async_send_request(req, [this, enabled](rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedFuture f) {
        lying_cm_pending_ = false;
        bool ok = true;
        for (const auto & r : f.get()->results) {ok = ok && r.successful;}
        RCLCPP_INFO(get_logger(), "누운 퍽 뒤로: collision_monitor 영역 %s%s", enabled ? "켬" : "끔", ok ? "" : " (실패)");
      });
  }

  void lying_publish(double vx, double vy, double wz)
  {
    if (!lying_cmd_pub_) {return;}
    geometry_msgs::msg::TwistStamped m;
    m.header.stamp = now();
    m.header.frame_id = exec_->base_frame();
    m.twist.linear.x = vx;
    m.twist.linear.y = vy;
    m.twist.angular.z = wz;
    lying_cmd_pub_->publish(m);
  }

  bool lying_odom_pose(double & x, double & y, double & yaw) const
  {
    try {
      const auto t = exec_->tf_buffer()->lookupTransform(exec_->odom_frame(), exec_->base_frame(), tf2::TimePointZero);
      const auto & q = t.transform.rotation;
      x = t.transform.translation.x;
      y = t.transform.translation.y;
      yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
      return true;
    } catch (const tf2::TransformException &) {
      return false;
    }
  }

  // 20 Hz: 뒤로 가는 중이면 odom → 명령. 단계가 바뀜 (FAULT · 멈춤) 이면 0 · 영역 켬
  void lying_move_tick()
  {
    const bool interrupted = op_ != Op::RUNNING || exec_->current_step_name() != lying_.name || pending_fault_;
    const auto act = lying_tick_action(lying_.active, lying_.phase, interrupted);
    if (act == LyingTick::None) {return;}
    if (act == LyingTick::Cleanup) {   // WaitDrive · Disable · Move 어디서 끊겨도 (Disable = 영역 이미 끔)
      lying_publish(0.0, 0.0, 0.0);
      if (lying_polys_off_) {lying_set_polygons(true);}
      RCLCPP_WARN(get_logger(), "누운 퍽 뒤로: 단계가 멈춤 (%s) → 명령 0 · 영역 켬", op_name(op_));
      lying_.active = false;
      return;
    }
    double x, y, yaw;
    if (!lying_odom_pose(x, y, yaw)) {
      lying_publish(0.0, 0.0, 0.0);
      return;
    }
    const auto c = straight_back_cmd(lying_.x0, lying_.y0, lying_.yaw0, lying_.goal_m, x, y, yaw, lying_move_);
    const double t = now_s();
    if (c.traveled > lying_.best + 0.002) {
      lying_.best = c.traveled;
      lying_.progress_s = t;
    }
    lying_.last = c;
    if (c.done || exec_->nav_blocked()) {
      lying_publish(0.0, 0.0, 0.0);
      return;
    }
    lying_publish(c.vx, c.vy, c.wz);
  }

  // kLyingBackoff: DRIVE 기다림 → 영역 끄기 → 곧게 뒤로 (lying_move_tick) → 끝: 0 · 영역 켬 · [LOOK, Plan] 끼움. 언제나 Success
  // 뒤로 뺀 자리의 Plan. 걸린 퍽이 보이고 아직 2 번 안 해 봤으면 그 퍽만 [PICK … STORE] + 같은 Plan 다시.
  //   아니면 (집었음 · 안 보임 · 2 번 실패 · 넣을 칸 없음) [박스 정차 이동 (정밀 접근) → LOOK → 같은 Plan] 으로 원래 자리에서 남은 퍽. 늘 true
  template<typename Pucks>
  bool lying_forward_plan(const Step & s, int box, const Pucks & at_box)
  {
    const int b = box - 1;
    Pucks only;
    for (const auto & p : at_box) {
      if (p.id == lying_trigger_[b]) {only.push_back(p);}
    }
    if (!only.empty() && lying_trig_n_[b] < 2 && !exec_->cargo().holding()) {
      std::string err;
      auto actions = collect_box_actions(only, exec_->cargo(), &err);
      if (err.empty() && !actions.empty()) {
        ++lying_trig_n_[b];
        auto arm = arm_steps(actions, rack_look_, box_measure_);
        if (relook_before_pick_) {arm = with_pick_relooks(arm, held_arm_step(), puck_views_);}
        arm.push_back(s);   // 끝나면 이 Plan 다시 → 다시 앞으로
        RCLCPP_INFO(get_logger(), "B%d 뒤로 뺀 자리: 걸린 퍽 %s 만 집음 (%d 번째) → 그 뒤 다시 앞으로", box, lying_trigger_[b].c_str(), lying_trig_n_[b]);
        exec_->insert_after_current(std::move(arm));
        return true;
      }
    }
    lying_fwd_[b] = false;
    Step nav = lying_nav_[b];
    nav.name += "(뒤로 뺀 뒤 다시 앞으로)";
    Step look = lying_look_;
    look.name = "arm:LOOK:B" + std::to_string(box) + "(다시 앞으로 간 뒤)";
    RCLCPP_WARN(get_logger(), "B%d 뒤로 뺀 자리 끝 (걸린 퍽 %s: %s) → 박스 앞으로 다시 붙어서 남은 퍽", box, lying_trigger_[b].c_str(),
      only.empty() ? "집었거나 안 보임" : (lying_trig_n_[b] >= 2 ? "2 번 해 봄" : "지금 못 집음"));
    exec_->insert_after_current({nav, look, s});
    return true;
  }

  StepResult run_lying_backoff(const Step & s, double now)
  {
    using Ph = LyingPhase;
    const int box = box_of(s);
    if (!lying_.active) {
      if (lying_.name != s.name) {   // 새 뒤로 빼기 (같은 단계를 FAULT · 멈춤 뒤 다시 하면 시작 자리 그대로 → 남은 만큼만)
        lying_ = LyingRun{};
        lying_.name = s.name;
        lying_.box = box;
        lying_.goal_m = std::clamp(s.pose.x, 0.0, std::max(0.0, lying_cfg_.max_m));
        lying_.t0 = now;
      }
      lying_.active = true;
      lying_.phase = Ph::WaitDrive;
      lying_.t_phase = now;
      lying_.best = -1.0;
    }
    auto finish = [&](const std::string & why) {
        lying_publish(0.0, 0.0, 0.0);
        if (lying_polys_off_) {lying_set_polygons(true);}
        const double moved = std::max(0.0, lying_.last.traveled);
        RCLCPP_WARN(get_logger(), "B%d 가까운 퍽 %s 뒤로: %.1f / %.1f cm (옆 %+.1f cm · 방향 %+.1f°, %.1f s, %s) → B%d 앞 LOOK · 계획 다시",
          box, s.puck_id.c_str(), moved * 100.0, lying_.goal_m * 100.0, lying_.last.lateral * 100.0, lying_.last.dyaw * 180.0 / M_PI,
          now - lying_.t0, why.c_str(), box);
        lying_.active = false;
        lying_.name.clear();
        Step look = lying_look_;
        Step plan = lying_plan_;
        if (box_of(look) != box || box_of(plan) != box) {   // (만약) 저장된 것이 다른 박스: 이 박스 앞 LOOK · 집기 계획을 새로
          look = Step{};
          look.type = StepType::Arm;
          look.arm_task = kArmLook;
          box_of(look) = box;
          look.name = "arm:LOOK:B" + std::to_string(box) + "(누운 퍽 뒤로 뺀 뒤)";
          plan = s;
          plan.kind = kPlanCollectBox;
          plan.pose = Pose2D{};
          plan.puck_id.clear();
          plan.name = "plan:collect:B" + std::to_string(box);
        }
        exec_->insert_after_current({look, plan});
        return StepResult::Success;
      };
    switch (lying_.phase) {
      case Ph::WaitDrive: {
          if (lying_.goal_m < 1e-3) {return finish("갈 거리 없음");}
          if (!exec_->drive_allowed() || exec_->nav_blocked()) {
            if (now - lying_.t_phase > lying_drive_wait_s_) {
              return finish(exec_->drive_allowed() ? "이동 차단 중 → 못 감" : "DRIVE 안 옴 → 못 감");
            }
            return StepResult::Running;
          }
          if (!lying_.have_start) {
            if (!lying_odom_pose(lying_.x0, lying_.y0, lying_.yaw0)) {return finish(exec_->odom_frame() + " TF 없음 → 못 감");}
            lying_.have_start = true;
          }
          lying_set_polygons(false);
          lying_.phase = Ph::Disable;
          lying_.t_phase = now;
          return StepResult::Running;
        }
      case Ph::Disable:
        if (lying_cm_pending_ && now - lying_.t_phase < 1.0) {return StepResult::Running;}
        lying_.phase = Ph::Move;
        lying_.t_phase = now;
        lying_.progress_s = now;
        lying_move_tick();
        return StepResult::Running;
      case Ph::Move: {
          const double limit = 1.5 + 2.5 * lying_.goal_m / std::max(0.02, lying_move_.v_min + 0.5 * (lying_move_.v_max - lying_move_.v_min));
          if (lying_.last.done) {return finish("도착");}
          if (now - lying_.progress_s > 1.5) {return finish("1.5 s 안 움직임 (뒤 막힘 · 차단?) → 간 만큼에서");}
          if (now - lying_.t_phase > limit) {return finish("시간 넘음 → 간 만큼에서");}
          return StepResult::Running;
        }
    }
    return StepResult::Running;
  }

  // 다시 보고 다시 집을 PICK 실패: 빈손 (EMPTY_GRASP) · 검출 높이가 틀려 control 이 거절 ('퍽 높이 이상', 팔 안 움직임)
  static bool pick_relook_fail(const std::string & detail)
  {
    return detail.find(kEmptyGraspToken) != std::string::npos || detail.find(kPickHeightToken) != std::string::npos;
  }

  // 박스 정밀 접근 실패한 이동 f 를 [정차점 뒤로 이동 · LOOK · 박스 자리 다시 재기] 로 바꿈. 다시 재기가 성공하면 새 정차점으로 f 를 다시 넣음
  //   시작 박스 (box_of 1–6, COLLECT) · 장소 박스 (place_of 0–2, DELIVER, box_relook_places) 둘 다
  void start_box_relook(const Step & f)
  {
    const int b = box_of(f) >= 1 && box_of(f) <= kNumBoxes ? box_of(f) : 0;
    const int pl = b == 0 ? place_of(f) : -1;
    const int key = b > 0 ? b - 1 : kNumBoxes + pl;
    const std::string tag = b > 0 ? "B" + std::to_string(b) : "L" + std::to_string(pl + 1);
    if (b > 0) {
      box_relooked_[b - 1] = true;
      box_relook_nav_[b - 1] = f;
    } else {
      place_relooked_[pl] = true;
      place_relook_nav_[pl] = f;
    }
    relook_side_i_[key] = 0;
    Step stand = f;   // 같은 박스 번호 (여기서 실패하면 그 박스 뺌)
    stand.dock = false;
    stand.pose.x -= box_relook_back_m_ * std::cos(f.pose.yaw);
    stand.pose.y -= box_relook_back_m_ * std::sin(f.pose.yaw);
    stand.name = f.name + " 다시 찾기 자리";
    relook_stand_[key] = stand.pose;
    Step look;
    look.type = StepType::Arm;
    look.arm_task = kArmLook;
    look.kind = kPickRelook;   // 관측 구간 · 옆 보기 상태 안 바꿈 (실패하면 이 LOOK 만 건너뜀 → 다시 재기가 퍽 못 봄 → 박스 뺌)
    if (b > 0) {box_of(look) = b;} else {place_of(look) = pl;}
    look.name = "arm:LOOK:" + tag + " 박스 다시 찾기";
    Step reloc;
    reloc.type = StepType::Custom;
    reloc.kind = kBoxRelocate;
    if (b > 0) {box_of(reloc) = b;} else {place_of(reloc) = pl;}
    reloc.name = "custom:" + tag + " 박스 자리 다시 재기";
    const std::size_t at = exec_->index();
    exec_->erase_steps(at, at + 1);
    exec_->insert_steps(at, {stand, look, reloc});
    RCLCPP_WARN(get_logger(), "%s 정밀 접근 실패 (%s) → 정차점 %.2f m 뒤에서 LOOK 으로 박스 자리 다시 재기", tag.c_str(), detail_.c_str(),
      box_relook_back_m_);
  }

  // LOOK 뒤 box_relook_dwell_s 동안: 손목 깊이 박스 측정 (box_relook_use_box_detector, 중심 · 방향) 먼저, 없으면 원시 관측 (map) 중
  //   박스 중심 box_relook_radius_m 안 ID 퍽 평균 (방향 = 지금 정차 방향) → 새 중심 · 정차점으로 그 박스 이동 다시
  //   측정으로 찾으면 네 면 중 로봇 쪽 면부터 지도 · 다른 박스에 몸이 안 닿는 면 (box_face_stops · choose_face)
  //   둘 다 없으면 box_relook_side_looks 다음 자세로 LOOK → 이 단계 다시. 다 쓰면 실패 (그 박스 · 장소 뺌)
  StepResult run_box_relocate(const Step & s, double now, double started, std::string & detail)
  {
    if (now - started < box_relook_dwell_s_) {return StepResult::Running;}
    const int b = box_of(s) >= 1 && box_of(s) <= kNumBoxes ? box_of(s) : 0;
    const int pl = b == 0 ? place_of(s) : -1;
    if (b == 0 && (pl < 0 || pl >= kNumPlaces)) {
      detail = "박스 다시 찾기: 박스 · 장소 번호 없음";
      return StepResult::Fail;
    }
    const int key = b > 0 ? b - 1 : kNumBoxes + pl;
    const std::string tag = b > 0 ? "B" + std::to_string(b) : "L" + std::to_string(pl + 1);
    std::array<double, 2> & center = b > 0 ? dice_map_.boxes[b - 1].center : dice_map_.places[pl].center;
    Pose2D & stop = b > 0 ? dice_map_.boxes[b - 1].stop : dice_map_.places[pl].stop;
    std::map<std::string, std::pair<double, double>> seen;   // ID → 마지막 위치 (시작 박스만, 장소 박스 위 퍽은 놓은 것)
    if (b > 0) {
      for (const auto & o : exec_->recent_obs()) {
        if (o.stamp < started - 0.2 || o.id.empty()) {continue;}
        if (std::hypot(o.x - center[0], o.y - center[1]) > box_relook_radius_m_) {continue;}
        seen[o.id] = {o.x, o.y};
      }
    }
    std::vector<std::array<double, 2>> pucks;
    std::string ids;
    for (const auto & [id, p] : seen) {pucks.push_back({p.first, p.second}); ids += (ids.empty() ? "" : ",") + id;}
    const std::vector<BoxSample> v(box_samples_.begin(), box_samples_.end());
    std::string why;
    const auto found = box_find_from(v, started - 0.2, center, stop.yaw, pucks, box_relook_use_box_detector_,
      dice_map_.box_size, box_relook_max_shift_m_, box_min_samples_, &why);
    if (!found) {
      // 못 찾음 → 옆 자세 하나 더 (팔만 움직임, 차체는 그대로)
      while (relook_side_i_[key] < static_cast<int>(box_relook_side_looks_.size()) &&
        !is_side_slot(box_relook_side_looks_[static_cast<std::size_t>(relook_side_i_[key])])) {++relook_side_i_[key];}
      if (relook_side_i_[key] < static_cast<int>(box_relook_side_looks_.size())) {
        const std::string slot = box_relook_side_looks_[static_cast<std::size_t>(relook_side_i_[key]++)];
        Step look;
        look.type = StepType::Arm;
        look.arm_task = kArmLook;
        look.kind = kPickRelook;
        look.slot = slot;
        if (b > 0) {box_of(look) = b;} else {place_of(look) = pl;}
        look.name = "arm:LOOK:" + tag + ":" + slot + " 박스 다시 찾기";
        Step again = s;
        RCLCPP_WARN(get_logger(), "%s 박스 다시 찾기: %s → 옆 LOOK %s 뒤 다시", tag.c_str(), why.c_str(), slot.c_str());
        exec_->insert_after_current({look, again});
        return StepResult::Success;
      }
      detail = tag + " 박스 다시 찾기: " + why;
      return StepResult::Fail;
    }
    // 정차 ↔ 박스 중심 = 파일 dice_map 값 (보통 0.2575). 메모리 값은 LOOK 다시 잡기 (recenter_box_on_look) 가 중심만 옮겨 틀릴 수 있음
    const auto & fb = dice_map_file_;
    const double d = b > 0 ? std::hypot(fb.boxes[b - 1].stop.x - fb.boxes[b - 1].center[0], fb.boxes[b - 1].stop.y - fb.boxes[b - 1].center[1])
                           : std::hypot(fb.places[pl].stop.x - fb.places[pl].center[0], fb.places[pl].stop.y - fb.places[pl].center[1]);
    Pose2D ns;
    std::string how;
    if (found->from_detector) {
      const auto & st = relook_stand_[key];
      const double ref = std::atan2(found->center[1] - st.y, found->center[0] - st.x);   // 로봇 → 새 중심
      std::vector<std::array<double, 2>> others;   // 다른 박스 · 장소 윗면 테두리 (지도에 없음)
      for (int k = 0; k < kNumBoxes; ++k) {
        if (k == b - 1) {continue;}
        const auto o = box_outline(dice_map_.boxes[k].center, dice_map_.boxes[k].stop.yaw, dice_map_.box_size);
        others.insert(others.end(), o.begin(), o.end());
      }
      for (int k = 0; k < kNumPlaces; ++k) {
        if (k == pl) {continue;}
        const auto o = box_outline(dice_map_.places[k].center, dice_map_.places[k].stop.yaw, dice_map_.box_size);
        others.insert(others.end(), o.begin(), o.end());
      }
      const auto & bd = box_relook_body_;
      // 지도에 찍힌 이 박스 (slam 지도) 는 막힘으로 안 셈: 잰 자리 · 지도 자리 윗면 + 3 cm
      const std::vector<std::array<double, 3>> ignore{{found->center[0], found->center[1], found->yaw}, {center[0], center[1], stop.yaw}};
      const auto face = choose_face(box_face_stops(found->center, found->yaw, d, ref), [&](const Pose2D & q) {
          return footprint_blocked(relook_map_, q, bd[0], bd[1], bd[2], 65, others, ignore);
        });
      ns = face->stop;
      char buf[200];
      std::snprintf(buf, sizeof(buf), "박스 측정 %d 개 · 방향 %+.1f° → 면 %d (정차 yaw %+.1f°, 로봇 쪽에서 %.0f° 돎%s%s)", found->n,
        found->yaw * 180.0 / M_PI, face->face, ns.yaw * 180.0 / M_PI, face->turn * 180.0 / M_PI,
        relook_map_.empty() ? ", 지도 없음 → 벽 검사 안 함" : "",
        face->blocked > 0 ? ", 모든 면 막힘 → 가장 덜 막힌 면" : "");
      how = buf;
      if (b == 0) {   // 장소: 칸 기준 측정 방향 = 잰 박스 방향, 보는 방향 = 새 정차 방향
        place_box_dyaw_[pl] = fold_quarter_diff(found->yaw, dice_map_.map_from_arena.yaw);
        place_relocated_[pl] = true;
        dice_map_.places[pl].view_dir = {std::cos(ns.yaw), std::sin(ns.yaw)};
        // 다른 면으로 붙으면 먼 칸 · 가까운 칸이 바뀜 → 아직 안 놓은 칸만 새 방향으로 먼 칸부터 (같은 그룹 안 점이라 어느 퍽이든 됨)
        if (dice_plan_ && placed_count_[pl] >= 0) {
          auto & offs = dice_plan_->places[pl].pip_offsets;
          const auto from = std::min<std::size_t>(static_cast<std::size_t>(placed_count_[pl]), offs.size());
          const double ax = -std::cos(ns.yaw), ay = -std::sin(ns.yaw);
          std::stable_sort(offs.begin() + static_cast<std::ptrdiff_t>(from), offs.end(),
            [ax, ay](const std::array<double, 2> & a, const std::array<double, 2> & c) {return a[0] * ax + a[1] * ay < c[0] * ax + c[1] * ay - 1e-9;});
        }
      }
    } else {   // 퍽 평균 (방향 그대로, 같은 거리)
      ns = stop;
      ns.x = found->center[0] - d * std::cos(stop.yaw);
      ns.y = found->center[1] - d * std::sin(stop.yaw);
      how = "퍽 " + std::to_string(found->n) + " 개 (" + ids + ")" + (found->note.empty() ? "" : ", " + found->note);
    }
    RCLCPP_WARN(get_logger(), "%s 박스 다시 찾기: %s → 박스 중심 (%.3f, %.3f) → (%.3f, %.3f), %.1f cm 옮김 → 새 정차점 (%.3f, %.3f, %.1f°) 으로 다시",
      tag.c_str(), how.c_str(), center[0], center[1], found->center[0], found->center[1],
      std::hypot(found->center[0] - center[0], found->center[1] - center[1]) * 100.0, ns.x, ns.y, ns.yaw * 180.0 / M_PI);
    const Pose2D old_stop = stop;
    center = found->center;   // 이 박스 퍽 고르기 (box_radius) · 정차 거리 정렬 · 남은 방문 (재계획 · 재검증) 도 새 중심 · 정차점
    stop = ns;
    // 이미 만든 뒤 단계 (실행기 남은 단계 · 배달 · 재검증 목록) 중 같은 박스 · 장소의 이전 정차점 이동도 새 정차점으로
    //   실행기: 지금 단계 뒤만, 하나 지우고 같은 자리에 하나 넣음 (크기 그대로 → 지금 단계 참조 안 깨짐, 성공 뒤 step_running_ 은 어차피 끔)
    const auto same_old = [&](const Step & q) {
        const bool same = b > 0 ? box_of(q) == b : place_of(q) == pl;
        return q.type == StepType::Navigate && same && std::hypot(q.pose.x - old_stop.x, q.pose.y - old_stop.y) < 1e-3;
      };
    int patched = 0;
    for (std::size_t i = exec_->index() + 1; i < exec_->steps().size(); ++i) {
      if (!same_old(exec_->steps()[i])) {continue;}
      Step c = exec_->steps()[i];
      c.pose = ns;
      exec_->erase_steps(i, i + 1);
      exec_->insert_steps(i, {c});
      ++patched;
    }
    for (auto * list : {&delivery_steps_, &verify_after_}) {
      for (auto & q : *list) {
        if (same_old(q)) {q.pose = ns; ++patched;}
      }
    }
    if (patched > 0) {RCLCPP_INFO(get_logger(), "%s 박스 다시 찾기: 뒤 단계 이동 %d 개도 새 정차점으로", tag.c_str(), patched);}
    Step nav = b > 0 ? box_relook_nav_[b - 1] : place_relook_nav_[pl];
    nav.pose = ns;
    exec_->insert_after_current({nav});
    return StepResult::Success;
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
          used_kind_[r.id] = {r.color, r.shape};
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
  //     alloc_prefer_more_pucks (기본 false) 면 같은 장소 수 중 놓는 퍽이 많은 쪽 먼저 (R6+G2 = 8 > G2+Y4 = 6), 그다음 시간
  //   - stopped = 지금 노드에 서서 LOOK 까지 끝났는가 (false 면 첫 동작 전에 다시 이동 · LOOK)
  //   - 채울 장소가 하나도 없으면 배달 단계 없이 (재검증 · 복귀로)
  // 계획 펼치기에 넘길 적재함 사본: 칸에 둔 채 건너뛴 퍽은 모르는 종류로 → 꺼낼 칸으로 안 고름, 칸은 찬 채 (넣을 칸으로도 안 고름)
  planning::Cargo plan_cargo() const
  {
    planning::Cargo c = exec_->cargo();
    for (const auto & [slot, it] : exec_->cargo().slots()) {
      if (!it || !rack_stuck_.count(it->puck_id)) {continue;}
      planning::CargoItem x = *it;
      x.color = "stuck";
      c.replace_slot_item(slot, x);
    }
    return c;
  }

  // 칸에 둔 채 건너뛴 퍽 (rack_stuck_): 계획에 안 넘김, 그 칸은 찬 칸으로 (용량에서 뺌)
  void drop_rack_stuck(SolverInput & base) const
  {
    if (rack_stuck_.empty()) {return;}
    const auto n0 = base.in_cargo.size();
    base.in_cargo.erase(std::remove_if(base.in_cargo.begin(), base.in_cargo.end(),
      [this](const TransferPuck & p) {return rack_stuck_.count(p.id) > 0;}), base.in_cargo.end());
    base.cargo_capacity -= static_cast<int>(n0 - base.in_cargo.size());
  }

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
    drop_rack_stuck(base);
    base.on_boxes = pucks_on_boxes();
    base.yellow_shape = yellow_used_shape_;
    std::array<int, kNumPlaces> need{};
    for (int i = 0; i < kNumPlaces; ++i) {
      base.place_group[i] = dice_plan_->places[i].group;
      need[i] = place_dead_[i] ? 0 : std::max(0, dice_plan_->places[i].pips - placed_count_[i]);
    }
    const int avail = static_cast<int>(base.on_boxes.size() + base.in_cargo.size()) + (base.gripper ? 1 : 0);

    // 채울 장소 조합: 많은 순 → (alloc_prefer_more_pucks) 놓는 퍽 많은 쪽 → 비용 작은 쪽 (transfer_planner choose_delivery)
    DeliveryChoice dc = choose_delivery(base, need, avail, costs_, plan_cargo(), placed_count_,
      transfer_max_remaining_, alloc_prefer_more_pucks_, &err, fill_partial_places_);
    std::optional<TransferPlan> best = std::move(dc.plan);
    const int best_n = dc.n, best_mask = dc.mask;
    auto places_text = [](int mask) {
        std::string s;
        for (int i = 0; i < kNumPlaces; ++i) {
          if (mask >> i & 1) {s += (s.empty() ? "L" : " · L") + std::to_string(i + 1);}
        }
        return s;
      };
    const std::string chosen = places_text(best_mask);
    if (best && !fill_partial_places_ && dc.time_mask != best_mask) {
      RCLCPP_INFO(get_logger(), "배달 계획: 같은 장소 수면 퍽 많은 쪽 (alloc_prefer_more_pucks) → %s 퍽 %d (예상 %.0f s), "
        "시간만 보면 %s 퍽 %d (예상 %.0f s)", chosen.c_str(), dc.pucks, best->cost_s,
        places_text(dc.time_mask).c_str(), dc.time_pucks, dc.time_cost_s);
    }
    if (!best) {
      RCLCPP_WARN(get_logger(), "배달 계획: 채울 수 있는 장소 없음 (남은 퍽 %d, %s)", avail, err.c_str());
      delivery_steps_.clear();
    } else {
      delivery_steps_ = transfer_steps(*best, dice_map_, rack_look_, box_measure_);
      skip_measured_place_looks(delivery_steps_);
      // 서서 이어 하면 (마지막 박스) 지금 팔 자세부터, 아니면 첫 정차가 이동 → LOOK
      if (relook_before_pick_) {delivery_steps_ = with_pick_relooks(delivery_steps_, stopped ? held_arm_step() : std::nullopt, puck_views_);}
      RCLCPP_INFO(get_logger(), "배달 계획: %s (%d 곳), 놓을 퍽 %d, 남은 퍽 %d, Y 모양 %s, 예상 %.0f s, 정차 %zu",
        chosen.c_str(), best_n, dc.pucks, avail, best->yellow_shape.c_str(), best->cost_s, best->stops.size());
      int need_sum = 0;
      for (int i = 0; i < kNumPlaces; ++i) {need_sum += need[i];}
      if (fill_partial_places_ && dc.pucks < need_sum) {
        RCLCPP_WARN(get_logger(), "놓을 퍽 부족: 남은 칸 %d 개 중 %d 개만 놓음 (%d 개 모자람: 못 집은 · 잃은 퍽, 또는 색 · 모양이 안 맞음)",
          need_sum, dc.pucks, need_sum - dc.pucks);
      }
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
    verify_after_ = costs_.verify_tour ? verify_steps(vo, dice_map_, verify_look_slot_) : std::vector<Step>{};
    for (int i = 0; i < kNumPlaces; ++i) {
      // 부분 채우기 장소: 놓을 퍽이 모자라 need 중 count 개만 놓음 (빈 칸 = 그 장소에서 로봇 쪽 칸)
      std::string what;
      if (place_dead_[i]) {
        what = "버림";
      } else if (need[i] == 0) {
        what = "끝";
      } else if ((best_mask >> i & 1) && dc.count[i] >= need[i]) {
        what = "채움";
      } else if (!fill_partial_places_) {
        what = "못 채움 (끝까지 채울 퍽 부족 → 안 놓음)";
      } else {
        const int k = (best_mask >> i & 1) ? dc.count[i] : 0;
        what = "놓을 퍽 " + std::to_string(need[i] - k) + " 개 부족 → " + std::to_string(k) + "/" + std::to_string(need[i]) +
          (k > 0 ? " 만 놓음" : " (안 놓음)");
      }
      RCLCPP_INFO(get_logger(), "  L%d %c%d: 놓음 %d, %s", i + 1, dice_plan_->places[i].group, dice_plan_->places[i].pips,
        placed_count_[i], what.c_str());
    }
    return true;
  }

  // 적재함 여유 칸 kRackSpareCells (1, 고정): 이 정차 STORE 가 마지막 빈 칸까지 쓰면 마지막에 집을 퍽이
  //   필요 없는지 (빼고 계획해도 장소 수 · 놓는 퍽 수 같음) 보고, 필요 없으면 그 PICK · STORE 를 빼서 한 칸을 남김. 필요하면 그대로 (마지막 칸도 씀)
  void keep_spare_cell(std::vector<ArmAction> & acts, int box)
  {
    if (!dice_plan_) {return;}
    int stores = 0, free = 0;
    for (const auto & a : acts) {
      if (a.op == ArmOp::Store) {++stores;}
    }
    for (const auto & sl : exec_->cargo().slots()) {
      if (!sl.second) {++free;}
    }
    if (free - stores >= kRackSpareCells) {return;}
    std::size_t pick = acts.size();
    for (std::size_t i = 0; i < acts.size(); ++i) {
      if (acts[i].op == ArmOp::Pick) {pick = i;}
    }
    if (pick == acts.size()) {return;}
    const std::string id = acts[pick].puck_id;
    std::string err;
    SolverInput base;
    base.position = current_node_;
    base.stopped = true;
    base.allow_leftover = true;
    if (!fill_carried(exec_->cargo(), base, &err)) {return;}
    drop_rack_stuck(base);
    base.on_boxes = pucks_on_boxes();
    base.yellow_shape = yellow_used_shape_;
    std::array<int, kNumPlaces> need{};
    for (int i = 0; i < kNumPlaces; ++i) {
      base.place_group[i] = dice_plan_->places[i].group;
      need[i] = place_dead_[i] ? 0 : std::max(0, dice_plan_->places[i].pips - placed_count_[i]);
    }
    std::string why;
    if (!puck_not_needed(base, id, need, costs_, plan_cargo(), placed_count_, transfer_max_remaining_, &why,
      fill_partial_places_))
    {
      RCLCPP_INFO(get_logger(), "B%d: 적재함 마지막 빈 칸까지 씀 — 퍽 %s 필요 (%s)", box, id.c_str(), why.c_str());
      return;
    }
    std::size_t end = pick + 1;
    if (end < acts.size() && acts[end].op == ArmOp::Store && acts[end].puck_id == id) {++end;}
    acts.erase(acts.begin() + static_cast<std::ptrdiff_t>(pick), acts.begin() + static_cast<std::ptrdiff_t>(end));
    RCLCPP_WARN(get_logger(), "B%d: 적재함 여유 칸 %d 남김 — 퍽 %s 안 실음 (필요 없음: %s), 박스에 둠", box, kRackSpareCells, id.c_str(), why.c_str());
  }

  // ---------------------------------------------------------------- 쥔 퍽 비우기
  // 박스에 되돌려 놓을 수 있나: 이 퍽을 집은 자리를 앎 · 그 뒤 이동 안 함 (같은 정차) · 되돌림 아직 안 함 · COLLECT
  bool putback_possible(const std::string & id) const
  {
    const auto it = pick_points_.find(id);
    // DELIVER 도: 이 정차에서 박스에서 집은 퍽만 (마지막 박스 배달 계획). 적재함에서 꺼낸 퍽 (RETRIEVE) 은 집은 자리가 이 정차가 아님 → 박스에 안 감
    return (phase_ == Phase::COLLECT || phase_ == Phase::DELIVER) && it != pick_points_.end() && it->second.nav_n == nav_ok_total_ &&
           !putback_tried_.count(id) && !retrieved_here_.count(id);
  }

  // STORE 실패한 쥔 퍽이 있으면 Plan 전에 끼울 단계 (다른 칸 STORE 또는 되돌림 + 같은 Plan). 없으면 빈 목록 (보통 계획)
  std::vector<Step> held_first(const Step & plan)
  {
    const auto & g = exec_->cargo().gripper();
    if (!g || store_alt_cells_ <= 0) {return {};}
    const auto it = store_failed_slots_.find(g->puck_id);
    if (it == store_failed_slots_.end() || it->second.empty()) {return {};}
    const HeldChoice c = held_puck_choice(exec_->cargo(), it->second, store_alt_cells_, putback_possible(g->puck_id));
    std::string failed;
    for (const auto & f : it->second) {failed += (failed.empty() ? "" : " · ") + f;}
    if (c.kind == HeldChoice::Kind::Store) {
      RCLCPP_WARN(get_logger(), "쥔 퍽 %s: STORE 실패 칸 %s → 다른 칸 %s 로 넣고 계획", g->puck_id.c_str(), failed.c_str(), c.slot.c_str());
    } else if (c.kind == HeldChoice::Kind::PutBack) {
      putback_tried_.insert(g->puck_id);
      RCLCPP_WARN(get_logger(), "쥔 퍽 %s: STORE 실패 칸 %s, 넣을 다른 칸 없음 (store_alt_cells %d) → 집은 박스 자리에 되돌려 놓고 계획",
        g->puck_id.c_str(), failed.c_str(), store_alt_cells_);
    } else {
      RCLCPP_WARN(get_logger(), "쥔 퍽 %s: STORE 실패 칸 %s, 다른 칸 · 되돌림 못 함 → 보통 계획", g->puck_id.c_str(), failed.c_str());
    }
    Step again = plan;
    if (again.name.find(":held") == std::string::npos) {again.name += ":held";}
    return held_first_steps(c, *g, again, rack_look_);
  }

  // 포기 전 손 비우기
  //   held_puck_choice: 실패 안 한 칸 STORE (실패 칸이 store_alt_cells 넘으면 안 함) → 아니면 집은 박스에 되돌림 (COLLECT, 같은 정차) →
  //   지금 남은 단계 대신 그것 → 손이 비면 tick 이 abandon_dice(why) 다시. 비우는 단계가 실패하면 dice_recover 가 다음 방법으로.
  //   방법이 없으면 false (→ 포기 · 복귀, 주행 전 CARRY_DRIVE 보호)
  bool start_empty_hand(const std::string & why)
  {
    if (!exec_->cargo().holding() || store_alt_cells_ <= 0 || op_ != Op::RUNNING) {return false;}
    auto steps = empty_hand_steps(why);
    if (steps.empty()) {return false;}
    continue_after_empty_ = false;
    abandon_after_putback_ = why;
    const std::size_t at = exec_->index();
    exec_->erase_steps(at, exec_->steps().size());
    exec_->insert_steps(at, steps);
    return true;
  }

  // 쥔 퍽을 지금 자리에서 비우는 단계 (held_puck_choice: 실패 안 한 칸 STORE → 되돌림). 없으면 빈 목록. 로그 한 줄
  std::vector<Step> empty_hand_steps(const std::string & why)
  {
    const auto & g = exec_->cargo().gripper();
    if (!g || store_alt_cells_ <= 0) {return {};}
    const auto it = store_failed_slots_.find(g->puck_id);
    const std::set<std::string> failed = it == store_failed_slots_.end() ? std::set<std::string>{} : it->second;
    const HeldChoice c = held_puck_choice(exec_->cargo(), failed, store_alt_cells_, putback_possible(g->puck_id));
    std::vector<Step> steps;
    if (c.kind == HeldChoice::Kind::Store) {
      ArmAction a;
      a.op = ArmOp::Store;
      a.puck_id = g->puck_id;
      a.color = g->color;
      a.shape = g->shape;
      a.slot = c.slot;
      steps = arm_steps({a}, rack_look_, false);
    } else if (c.kind == HeldChoice::Kind::PutBack) {
      putback_tried_.insert(g->puck_id);
      steps.push_back(putback_step(g->puck_id));
    } else {
      RCLCPP_ERROR(get_logger(), "손 비우기: 퍽 %s 쥔 채 (%s) → 넣을 칸 · 되돌림 없음", g->puck_id.c_str(), why.c_str());
      return {};
    }
    RCLCPP_WARN(get_logger(), "손 비우기: 퍽 %s 쥔 채 (%s) → %s", g->puck_id.c_str(), why.c_str(),
      c.kind == HeldChoice::Kind::Store ? ("STORE " + c.slot).c_str() : "집은 박스 자리에 되돌림");
    return steps;
  }

  // 되돌림 PLACE 목표: PICK 보낼 때 기록부 자리 (map) → base_link, z = 0 (control: 박스 윗면 · 쥔 퍽 높이로 놓음, store_putback_plan 과 같은 약속)
  bool putback_target(const Step & s, geometry_msgs::msg::PoseStamped & out, std::string & detail)
  {
    const auto it = pick_points_.find(s.puck_id);
    if (it == pick_points_.end()) {
      detail = "되돌림: 퍽 " + s.puck_id + " 집은 자리 모름";
      return false;
    }
    geometry_msgs::msg::PoseStamped in;
    in.header.frame_id = exec_->map_frame();
    in.pose.position.x = it->second.x;
    in.pose.position.y = it->second.y;
    in.pose.orientation.w = 1.0;
    std::string why;
    if (!exec_->transform_pose(in, "base_link", out, &why)) {
      detail = "되돌림: 집은 자리 변환 실패 (" + why + ")";
      return false;
    }
    out.header.frame_id = "base_link";
    out.pose.position.z = 0.0;
    out.pose.orientation = geometry_msgs::msg::Quaternion();
    out.pose.orientation.w = 0.0;
    RCLCPP_INFO(get_logger(), "되돌림 %s: 집은 자리 map (%.3f, %.3f) → base (%.3f, %.3f), z 0 = 박스 윗면", s.puck_id.c_str(),
      it->second.x, it->second.y, out.pose.position.x, out.pose.position.y);
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
    //   옆 보기: 다음 옆 보기 · 같은 Plan 은 그대로 → 본 만큼으로 계획. 다시 보기: 그 PICK 은 앞 검출로
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
    // 박스 정밀 접근 실패 → 빼기 전에 한 번 박스 자리 다시 재기 (재계획 횟수에 안 셈). 그 안에서 또 실패하면 아래 보통 처리 (박스 뺌)
    if (box_relook_ && phase_ == Phase::COLLECT && f.type == StepType::Navigate && f.dock && box_of(f) >= 1 &&
      box_of(f) <= kNumBoxes && !box_relooked_[box_of(f) - 1] && detail_.find("정밀 접근") != std::string::npos)
    {
      start_box_relook(f);
      return true;
    }
    // 장소 박스 (L1–L3) 정밀 접근 실패도 같은 다시 찾기 한 번 (box_relook_places, DELIVER · VERIFY)
    if (box_relook_ && box_relook_places_ && (phase_ == Phase::DELIVER || phase_ == Phase::VERIFY) && f.type == StepType::Navigate &&
      f.dock && box_of(f) < 1 && place_of(f) >= 0 && place_of(f) < kNumPlaces && !place_relooked_[place_of(f)] &&
      detail_.find("정밀 접근") != std::string::npos)
    {
      start_box_relook(f);
      return true;
    }
    // 포기 전 손 비우기 단계 실패 → 그 칸 · 되돌림을 빼고 다음 방법 (재계획 횟수 안 셈, 방법이 다 떨어지면 포기)
    if ((!abandon_after_putback_.empty() || continue_after_empty_) && f.type == StepType::Arm &&
      (is_putback(f) || f.arm_task == kArmStore || f.arm_task == kArmLookRack))
    {
      if (!is_putback(f) && !f.slot.empty()) {store_failed_slots_[f.puck_id].insert(f.slot);}
      if (continue_after_empty_) {   // 재고 정합 뒤 손 비우기: 다음 방법, 없으면 포기 흐름
        const std::size_t at = exec_->index();
        auto steps = empty_hand_steps("재고 정합 뒤 쥔 퍽, " + f.name + " 실패");
        exec_->erase_steps(at, exec_->steps().size());
        if (!steps.empty()) {
          exec_->insert_steps(at, steps);
          return true;
        }
        continue_after_empty_ = false;
        abandon_dice("재고 정합 뒤 손 비우기 실패: " + f.name);
        return true;
      }
      const std::string why = abandon_after_putback_;
      abandon_after_putback_.clear();
      RCLCPP_WARN(get_logger(), "손 비우기 %s 실패 (%s) → 다음 방법", f.name.c_str(), detail_.c_str());
      abandon_dice(why);
      return true;
    }
    if (is_putback(f)) {   // 되돌림도 실패 = 지금 자리에서 손을 비울 방법 없음 → 포기 · 복귀 (주행 전 CARRY_DRIVE 보호)
      RCLCPP_ERROR(get_logger(), "되돌림 실패: 퍽 %s (%s) → 손을 비울 다른 방법 없음, 포기", f.puck_id.c_str(), detail_.c_str());
      abandon_after_putback_.clear();
      return false;
    }
    // 다시 하는 것 (첫 빈손 PICK → 다시 보고 다시 집기, 박스 이동 실패 → 한 번 뒤로 미룸, 자동 정합 뒤 벌칙 없는 재계획) 은
    //   재계획 횟수에 안 셈 (저녁 실물: 다시 해서 성공한 빈손 3 번 + B4 미루기가 6 번 중 4 번을 써서, 퍽 7 개를 적재함에 두고 배달 없이 복귀)
    const bool free_retry = phase_ == Phase::COLLECT &&
      (recover_no_penalty_ ||
       (f.type == StepType::Arm && f.arm_task == kArmPick && pick_relook_fail(detail_) &&
        empty_grasp_n_[f.puck_id] + 1 < empty_grasp_skip_n_) ||
       (box_retry_later_ && f.type == StepType::Navigate && box_of(f) >= 1 && box_of(f) <= kNumBoxes && !box_retried_.count(box_of(f))));
    if (phase_ != Phase::VERIFY && !free_retry && ++replans_ > max_replans_) {   // 재검증 건너뛰기는 횟수에 안 셈
      if (phase_ == Phase::COLLECT && !is_custom(f, kPlanDelivery)) {
        // 모은 퍽은 배달: 수집을 그만두고 지금 자리에서 배달 계획 (포기 · 복귀하지 않음). 배달 재계획 횟수는 0 부터
        RCLCPP_WARN(get_logger(), "재계획 %d 번 넘음 (%s) → 수집 그만, 모은 퍽으로 지금 자리에서 배달 (배달 재계획 횟수 0 부터)",
          max_replans_, f.name.c_str());
        if (f.type == StepType::Arm && f.arm_task == kArmPick) {exec_->mark_puck(f.puck_id, "skipped");}
        const std::size_t at = exec_->index();
        exec_->erase_steps(at, exec_->steps().size());
        replans_ = 0;
        alloc_reason_ = "replan: 재계획 횟수 넘음 → 배달";
        Step here;
        here.type = StepType::Custom;
        here.kind = kPlanDelivery;
        box_of(here) = -1;
        here.name = "plan:delivery:here";
        auto first = held_first(here);   // STORE 실패한 쥔 퍽이 있으면 먼저 다른 칸 · 되돌림
        exec_->insert_steps(at, first.empty() ? std::vector<Step>{here} : first);
        return true;
      }
      RCLCPP_WARN(get_logger(), "재계획 %d 번 넘음 → 포기", max_replans_);
      return false;
    }
    if (f.type == StepType::Arm && !recover_no_penalty_) {
      switch (f.arm_task) {
        case kArmPick:
          if (pick_relook_fail(detail_) && ++empty_grasp_n_[f.puck_id] < empty_grasp_skip_n_) {
            RCLCPP_WARN(get_logger(), "%s PICK %s: %d 번째 → 건너뛰지 않고 다시 보고 다시 집기 (%d 번째면 건너뜀, empty_grasp_skip_n)",
              detail_.find(kPickHeightToken) != std::string::npos ? "높이 틀림" : "빈손", f.puck_id.c_str(), empty_grasp_n_[f.puck_id],
              empty_grasp_skip_n_);
            break;
          }
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
          } else if (store_alt_cells_ > 0 && !f.slot.empty()) {   // 퍽은 아직 손에: 다음 계획에서 이 칸 빼기 (held_puck_choice)
            store_failed_slots_[f.puck_id].insert(f.slot);
            RCLCPP_WARN(get_logger(), "STORE %s → %s 실패 (%s) → 다음 계획에서 이 칸 뺌 (실패 칸 %zu, store_alt_cells %d 넘으면 박스에 되돌림)",
              f.puck_id.c_str(), f.slot.c_str(), detail_.c_str(), store_failed_slots_[f.puck_id].size(), store_alt_cells_);
          }
          break;
        case kArmRetrieve:
          // control 이 계속 거절 (팔 안 움직임, 재고 확실): 같은 목표를 다시 보내도 같은 거절
          if (retrieve_reject_skip_n_ > 0 && !f.puck_id.empty() && detail_.find("계속 거절됨") != std::string::npos) {
            const int n = ++retrieve_reject_n_[f.puck_id];
            if (n >= retrieve_reject_skip_n_) {
              rack_stuck_.insert(f.puck_id);
              RCLCPP_WARN(get_logger(), "%s: control 이 계속 거절 (%d 번째, %s) → 퍽 %s 은 칸 %s 에 둔 채 건너뜀 (남은 퍽으로 계속)", f.name.c_str(), n,
                detail_.c_str(), f.puck_id.c_str(), f.slot.c_str());
            }
          }
          break;
        default:
          break;   // STORE · LOOK_RACK: 재고 그대로 다시 계획. LOOK 실패: 그 자리 퍽을 못 봄 → Plan 이 본 것만으로 진행
      }
    }
    alloc_reason_ = "replan: " + f.name + " 실패";
    const bool same_stock = f.type == StepType::Arm && (f.arm_task == kArmStore || f.arm_task == kArmLookRack ||
      (f.arm_task == kArmRetrieve && !rack_stuck_.count(f.puck_id)));
    RCLCPP_WARN(get_logger(), "재계획 %d/%d: %s 실패 → %s", replans_, max_replans_, f.name.c_str(),
      phase_ == Phase::VERIFY ? "이 장소 재검증 건너뜀" :
      f.type == StepType::Arm && f.arm_task == kArmRetrieve && rack_stuck_.count(f.puck_id) ? "그 퍽 (칸에 둠) 빼고 다시" :
      same_stock ? "재고 그대로 다시 계획" : "그 퍽 · 장소 빼고 다시");

    const auto & st = exec_->steps();
    const std::size_t at = exec_->index();
    // 지금 단계부터 다음 이동 전까지 = 이 정차의 남은 단계 (이동 실패면 그 이동이 연 정차 전체)
    auto seg_end = [&st](std::size_t from) {
        std::size_t e = from;
        if (e < st.size() && st[e].type == StepType::Navigate) {++e;}
        while (e < st.size() && st[e].type != StepType::Navigate) {++e;}
        return e;
      };

    // 시작 (이어 하기 포함) 뒤 이동이 한 번도 안 됐는데 이동 실패 → 주행 계통 문제로 보고 FAULT (박스 · 장소를 줄줄이 버리지 않음)
    if (nav_fault_before_first_ && !nav_first_waived_ && f.type == StepType::Navigate && nav_ok_since_start_ == 0 &&
      phase_ != Phase::VERIFY)
    {
      --replans_;   // 재계획으로 안 셈
      request_fault("NAV_NOT_WORKING", "start 뒤 첫 이동부터 실패: " + f.name + " (" + detail_ +
        ") → Nav2 · planner 지도 · dock 확인 뒤 resume (그 자리부터 이어 감)");
      return true;
    }
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
      const std::vector<Step> seg(st.begin() + static_cast<std::ptrdiff_t>(at), st.begin() + static_cast<std::ptrdiff_t>(e));
      exec_->erase_steps(at, e);
      if (is_custom(f, kPlanDelivery)) {return false;}   // 계획 자체가 실패
      // 박스 이동 실패: 그 박스를 한 번만 뒤로 미룸 (마지막 박스 = 배달 계획 박스 앞에 끼움, 그 박스 자신이면 바로 한 번 더)
      if (box_retry_later_ && f.type == StepType::Navigate && box >= 1 && box <= kNumBoxes && !box_retried_.count(box)) {
        box_retried_.insert(box);
        const auto & now_st = exec_->steps();
        std::size_t put = now_st.size();
        for (std::size_t i = at; i < now_st.size(); ++i) {   // 남은 단계 중 배달 계획 박스 정차 (이동) 자리
          if (now_st[i].type == StepType::Navigate) {
            std::size_t j = i + 1;
            while (j < now_st.size() && now_st[j].type != StepType::Navigate) {
              if (is_custom(now_st[j], kPlanDelivery)) {put = i; break;}
              ++j;
            }
            if (put != now_st.size()) {break;}
          }
        }
        if (had_delivery) {put = at;}
        exec_->insert_steps(put, seg);
        RCLCPP_WARN(get_logger(), "B%d 이동 실패 → 버리지 않고 %s 한 번 더 (box_retry_later, 재계획 %d/%d)", box,
          had_delivery ? "바로" : "남은 박스 뒤에", replans_, max_replans_);
        return true;
      }
      if (box >= 1 && box <= kNumBoxes) {side_look_[box - 1] = SideLookState::Done;}   // 복구 Plan 은 옆 보기 안 함 (실패 처리 그대로)
      Step again;
      again.type = StepType::Custom;
      const bool skip_box = f.type == StepType::Navigate || f.type == StepType::Custom;
      if (skip_box) {
        // 박스 건너뜀. 마지막 박스였으면 지금 자리에서 배달 계획
        if (!had_delivery) {   // 남은 단계가 없으면 끝 처리 (부분 수집 마지막 박스 실패 → 빈 RUNNING 안 남김)
          if (exec_->done()) {on_steps_done();}
          return true;
        }
        again.kind = kPlanDelivery;
        box_of(again) = -1;
        again.name = "plan:delivery:here";
      } else {
        again.kind = had_delivery ? kPlanDelivery : kPlanCollectBox;
        box_of(again) = box;
        again.name = std::string(had_delivery ? "plan:delivery:B" : "plan:collect:B") + std::to_string(box) + ":again";
      }
      std::vector<Step> ins{again};
      // 박스 앞 LOOK 실패 (팔 거절 · 하드웨어 순간 이상): 퍽을 못 본 채 Plan 이 0 개로 박스를 버림 → 같은 LOOK 을 한 번 더 끼움
      if (f.type == StepType::Arm && is_look_task(f.arm_task) && box_of(f) == box && !box_look_retried_.count(box)) {
        box_look_retried_.insert(box);
        Step look = f;
        look.name += "(다시)";
        ins.insert(ins.begin(), look);
        RCLCPP_WARN(get_logger(), "B%d 앞 LOOK 실패 → 그 LOOK 한 번 더 뒤 계획", box);
      }
      // PICK 빈손 (EMPTY_GRASP): 손가락이 박스 안까지 내려가 닫혔다 올라옴 → 옆 퍽이 밀렸을 수 있음
      //   → 같은 정차 앞 LOOK 을 다시 (HOME → LOOK → Plan) 해서 새 검출로 계획. 다른 실패는 보통 처리 (팔 안 들어감)
      //   STORE 박스 되돌림 (STORE_RETURNED_TO_BOX) 도 같음: 퍽을 박스에 다시 내려놓음 → 새 검출로 다시 집음
      const bool store_back = f.type == StepType::Arm && f.arm_task == kArmStore && detail_.find(kStoreReturnedToken) != std::string::npos;
      if (!skip_box && f.type == StepType::Arm && ((f.arm_task == kArmPick && pick_relook_fail(detail_)) || store_back))
      {
        const char * what = store_back ? "STORE 박스 되돌림" : (detail_.find(kPickHeightToken) != std::string::npos ? "높이 틀린 PICK" : "빈손 PICK");
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
      const int fp = place_of(f);
      if ((f.type == StepType::Navigate || is_custom(f, kBoxRelocate)) && fp >= 0 && fp < kNumPlaces && place_nav_fail_n_ > 0 &&
        ++place_nav_fails_[fp] >= place_nav_fail_n_)
      {
        place_dead_[fp] = true;
        RCLCPP_WARN(get_logger(), "L%d 이동 %d 번 실패 → 이 장소 버리고 남은 장소부터 (place_nav_fail_n)", fp + 1, place_nav_fails_[fp]);
      }
      exec_->erase_steps(at, st.size());
      {   // STORE (막는 퍽 옮기기 등) 실패한 쥔 퍽: 다른 칸에 먼저 넣고 지금 자리에서 다시 계획 (실패 칸으로 다시 계획 안 함)
        Step here;
        here.type = StepType::Custom;
        here.kind = kPlanDelivery;
        box_of(here) = -1;
        here.name = "plan:delivery:here";
        if (auto first = held_first(here); !first.empty()) {
          exec_->insert_steps(at, first);
          return true;
        }
      }
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
    // 퍽 관측: 이 장소 LOOK 을 시작한 뒤 촬영된 것만 (촬영 시각 기준, 기록부의 지난 ID 를 쓰지 않음)
    double since = looked;
    const double look_t = look_place_started_[place];
    if (look_t > 0.0) {since = look_t;}
    // 칸 기준 = 재검증 LOOK 에서 다시 잰 박스 (퍽 관측과 같은 때 위치 추정) → 측정 부족이면 놓을 때 쓴 박스 측정 → 없으면 지도 값
    //   칸 = 그 박스 중심 + 계획 눈 오프셋 (박스 방향으로 돌림), 박스 위 확인도 같은 박스
    DiceMap arena = dice_map_;
    DicePlan plan = *dice_plan_;
    const auto remeasured = measured_place(place, looked, false);
    const auto & placed = placed_frame_[place];
    const auto & c0 = dice_map_.places[place].center;
    if (placed && remeasured) {
      RCLCPP_INFO(get_logger(), "재검증 L%d: 다시 잰 박스는 놓을 때 측정과 (%+.1f, %+.1f) mm · %+.1f° 다름 (참고, 다시 잰 기준으로 봄)", place + 1,
        1000.0 * (remeasured->x - placed->x), 1000.0 * (remeasured->y - placed->y), (remeasured->dyaw - placed->dyaw) * 180.0 / M_PI);
    }
    if (const auto f = verify_place_frame(placed, remeasured)) {
      apply_place_frame(arena, plan, place, *f);
      RCLCPP_INFO(get_logger(), "재검증 L%d: 칸 기준 = %s (지도 중심 대비 (%+.1f, %+.1f) mm · %+.1f°)", place + 1,
        remeasured ? "재검증 LOOK 박스 측정" : "놓을 때 박스 측정", 1000.0 * (f->x - c0[0]), 1000.0 * (f->y - c0[1]), f->dyaw * 180.0 / M_PI);
    } else {
      RCLCPP_WARN(get_logger(), "재검증 L%d: 박스 측정 없음 (놓을 때 · 다시 잼 둘 다) → 지도 칸 기준으로 봄", place + 1);
    }
    const auto & c = arena.places[place].center;
    const auto obs = gather_place_obs(c, since, place);
    // 박스 위 확인 윗면 = 칸 기준과 같은 박스. 지도 값은 실제와 몇 cm 달라 안 씀
    std::optional<VerifyBox> box;
    if (const auto & bf = remeasured ? remeasured : placed) {
      VerifyBox b;
      b.x = bf->x;
      b.y = bf->y;
      b.yaw = dice_map_.map_from_arena.yaw + bf->dyaw;
      b.half = place_limits_.box_half;
      b.margin = verify_on_box_margin_m_;
      b.source = remeasured ? "remeasured" : "placed";
      box = b;
    } else {
      RCLCPP_WARN(get_logger(), "재검증 L%d: 박스 측정 없음 → 퍽이 박스 위인지 확인 못 함", place + 1);
    }
    // 모양 맞춤은 박스 측정이 없을 때만 (측정 있으면 박스 중심 기준 칸 그대로)
    VerifyFitParams fit = verify_fit_;
    if (box) {fit.enable = false;}
    std::vector<std::string> cell_ids;
    std::vector<std::array<std::string, 2>> cell_kinds;   // 칸마다 놓은 퍽의 계획 색 · 모양 (판정은 다시 본 값, 다르면 이유에 같이)
    if (verify_cell_ids_) {
      cell_ids = placed_cell_ids_[place];
      cell_ids.resize(plan.places[place].pip_offsets.size());
      for (const auto & id : cell_ids) {
        const auto u = used_kind_.find(id);
        cell_kinds.push_back(u == used_kind_.end() ? std::array<std::string, 2>{} : std::array<std::string, 2>{u->second.first, u->second.second});
      }
    }
    auto pv = judge_place(place, plan, arena, obs, verify_cell_tol_m_, now - start_s_, fit, box, cell_ids, verify_ignore_inset_m_,
      cell_kinds);
    pv.frame_source = remeasured ? "remeasured" : (placed ? "placed" : "map");
    for (const auto & o : pv.ignored) {
      RCLCPP_INFO(get_logger(), "재검증 L%d: 박스 가장자리 가까이 · 밖 검출 뺌 (ID %s %s/%s, 가장자리에서 %+.0f mm, 기준 %.0f mm 안쪽)", place + 1,
        o.id.empty() ? "?" : o.id.c_str(), o.color.c_str(), o.shape.c_str(), 1000.0 * o.edge_m, 1000.0 * verify_ignore_inset_m_);
    }
    publish_verify_image(pv, arena.places[place].top_z, looked);
    if (pv.fitted) {
      RCLCPP_INFO(get_logger(), "재검증 L%d: 눈 모양 맞춤 = 기준 칸에서 (%+.1f, %+.1f) mm · %+.1f°", place + 1,
        1000.0 * pv.fit_dx, 1000.0 * pv.fit_dy, pv.fit_dyaw * 180.0 / M_PI);
    } else if (fit.enable && !obs.empty()) {
      RCLCPP_WARN(get_logger(), "재검증 L%d: 눈 모양 못 맞춤 (기준에서 %.0f mm · %.0f° 안 해 없음) → 기준 칸 그대로", place + 1,
        1000.0 * verify_fit_.max_shift_m, verify_fit_.max_rot_rad * 180.0 / M_PI);
    }
    if (pv.ok) {   // 본 퍽: ID 색/모양 (ID 못 읽었으면 ?)
      std::vector<std::string> seen;
      for (const auto & o : pv.seen) {
        seen.push_back("ID " + (o.id.empty() ? std::string("?") : o.id) + " " +
          (o.color == "red" ? "R" : o.color == "green" ? "G" : "?") + "/" +
          (o.shape == "cylinder" ? "cyl" : o.shape.empty() ? "?" : o.shape));
      }
      RCLCPP_INFO(get_logger(), "재검증 L%d (%c%d): 본 퍽 %zu 개 → SUCCESS (%s)", place + 1, pv.group, pv.expected, pv.seen.size(),
        join(seen).c_str());
    } else {   // 판정이 나빠도 미션은 계속 (기록이 목적) → 이유를 같은 줄에
      RCLCPP_WARN(get_logger(), "재검증 L%d (%c%d): 본 퍽 %zu 개 → FAIL: %s", place + 1, pv.group, pv.expected, pv.seen.size(),
        join(pv.issues, " · ").c_str());
    }
    verify_places_.erase(std::remove_if(verify_places_.begin(), verify_places_.end(),
      [place](const PlaceVerify & v) {return v.place == place;}), verify_places_.end());
    verify_places_.push_back(std::move(pv));
    const auto r = summarize(verify_places_);
    write_verify_log(r);   // 장소마다 씀 (세 곳 다 못 봐도 본 곳 ID · 위치가 남게, status PENDING)
    publish_verify(r);
    if (r.complete) {
      if (r.pass) {
        RCLCPP_INFO(get_logger(), "재검증 SUCCESS%s%s", verify_log_file_.empty() ? "" : " → ", verify_log_file_.c_str());
      } else {   // 이유 = 장소마다 문제 + ID 중복 · 누락 (ID 못 읽은 퍽도 누락으로 셈)
        std::vector<std::string> why;
        for (const auto & pl : r.places) {
          if (!pl.ok) {why.push_back("L" + std::to_string(pl.place + 1) + " [" + join(pl.issues, " · ") + "]");}
        }
        if (!r.duplicate_ids.empty()) {why.push_back("ID 중복 " + join(r.duplicate_ids, ","));}
        if (!r.missing_ids.empty()) {
          why.push_back("ID 누락 " + std::to_string(r.missing_ids.size()) + " 개 (" + join(r.missing_ids, ",") + ")");
        }
        RCLCPP_WARN(get_logger(), "재검증 FAIL: %s%s%s", join(why, " / ").c_str(), verify_log_file_.empty() ? "" : " → ",
          verify_log_file_.c_str());
      }
    }
  }

  // ---------------------------------------------------------------- 재검증 그림
  // VERIFY 단계 동안 손목 컬러 · 내부값 구독 (평소엔 안 받음 = 영상 전송 비용 없음)
  void start_verify_capture()
  {
    if (!pub_verify_image_ || verify_color_sub_) {return;}
    verify_frame_.reset();
    verify_color_sub_ = create_subscription<sensor_msgs::msg::Image>(verify_color_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::ConstSharedPtr m) {verify_frame_ = m;});
    if (!verify_info_) {
      verify_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(verify_info_topic_, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr m) {verify_info_ = m;});
    }
  }

  void stop_verify_capture()
  {
    verify_color_sub_.reset();
    verify_info_sub_.reset();
    verify_frame_.reset();
  }

  // 판정 때: LOOK 성공 뒤 찍힌 마지막 장 + 그 시각 map → 카메라 TF 로 판정을 그려 발행 · 저장. 못 하면 경고만 (판정과 무관)
  void publish_verify_image(const PlaceVerify & pv, double top_z, double looked)
  {
    if (!pub_verify_image_) {return;}
    const auto frame = verify_frame_;
    const auto info = verify_info_;
    verify_frame_.reset();
    const std::string name = "L" + std::to_string(pv.place + 1);
    if (!frame || !info) {
      RCLCPP_WARN(get_logger(), "재검증 %s 그림 없음: %s 안 옴", name.c_str(), !frame ? verify_color_topic_.c_str() : verify_info_topic_.c_str());
      return;
    }
    const double stamp = rclcpp::Time(frame->header.stamp).seconds();
    if (stamp < looked) {
      RCLCPP_WARN(get_logger(), "재검증 %s 그림: 마지막 장이 LOOK 끝보다 %.2f s 앞 (팔 움직이던 장일 수 있음)", name.c_str(), looked - stamp);
    }
    cv::Mat bgr;
    const cv::Mat raw(static_cast<int>(frame->height), static_cast<int>(frame->width), CV_8UC3,
      const_cast<uint8_t *>(frame->data.data()), frame->step);
    if (frame->encoding == "rgb8") {
      cv::cvtColor(raw, bgr, cv::COLOR_RGB2BGR);
    } else if (frame->encoding == "bgr8") {
      bgr = raw.clone();
    } else {
      RCLCPP_WARN(get_logger(), "재검증 %s 그림: 영상 형식 %s 못 받음 (rgb8 · bgr8 만)", name.c_str(), frame->encoding.c_str());
      return;
    }
    VerifyCamera cam;
    cam.fx = info->k[0];
    cam.fy = info->k[4];
    cam.cx = info->k[2];
    cam.cy = info->k[5];
    try {   // 촬영 시각 TF (팔이 LOOK 자세로 서 있으므로 없으면 최신)
      const auto tfb = exec_->tf_buffer();
      geometry_msgs::msg::TransformStamped t;
      try {
        t = tfb->lookupTransform(verify_camera_frame_, exec_->map_frame(), frame->header.stamp, tf2::durationFromSec(0.1));
      } catch (const tf2::TransformException &) {
        t = tfb->lookupTransform(verify_camera_frame_, exec_->map_frame(), tf2::TimePointZero);
      }
      const auto & q = t.transform.rotation;
      const tf2::Matrix3x3 m(tf2::Quaternion(q.x, q.y, q.z, q.w));
      for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {cam.R[3 * r + c] = m[r][c];}
      }
      cam.t = {t.transform.translation.x, t.transform.translation.y, t.transform.translation.z};
      // 손목 카메라 돌아감 보정 (검출 · 박스 측정과 같은 기준, 안 하면 박스 · 칸이 화면에서 한쪽으로 밀림)
      const auto & drift = exec_->drift();
      if (drift && drift->header.frame_id == verify_camera_frame_) {
        const auto & dq = drift->quaternion;
        apply_camera_drift(cam, {dq.x, dq.y, dq.z, dq.w});
      }
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN(get_logger(), "재검증 %s 그림: TF %s ← %s 없음 (%s) → 판정 글만", name.c_str(), verify_camera_frame_.c_str(),
        exec_->map_frame().c_str(), e.what());
      cam.fx = 0.0;   // 투영 안 함 (위 띠만)
    }
    // 검출 상자는 그 퍽 ID 가 마지막으로 읽힌 장 것 → LOOK 끝 (팔 멈춤) 전 장이면 배경 (마지막 장) 과 어긋나므로 안 쓰고
    //   위치 투영 표시로 그림 (움직이던 장 상자는 빈 자리에 그려짐)
    // 덧그리기 전 원본 장 + 그 장 카메라 (보정 뒤 map → 카메라, 내부값) 저장 → 나중에 같은 장으로 다시 그려 볼 수 있게
    const std::string dir = run_dir();
    std::string raw_path;
    if (!dir.empty()) {
      raw_path = dir + "/verify_" + name + "_raw.jpg";
      if (!cv::imwrite(raw_path, bgr, {cv::IMWRITE_JPEG_QUALITY, 95})) {raw_path = "(원본 저장 실패) " + raw_path;}
      std::ofstream cf(dir + "/verify_" + name + "_cam.yaml");
      cf.precision(9);
      cf << "# 재검증 " << name << " 원본 장 카메라 (p_cam = R · p_map + t, drift 반영 뒤). fx 0 = TF 없음\n"
         << "stamp: " << stamp << "\nlook_end: " << looked << "\nfx: " << cam.fx << "\nfy: " << cam.fy
         << "\ncx: " << cam.cx << "\ncy: " << cam.cy << "\nR: [";
      for (int i = 0; i < 9; ++i) {cf << (i ? ", " : "") << cam.R[i];}
      cf << "]\nt: [" << cam.t[0] << ", " << cam.t[1] << ", " << cam.t[2] << "]\n";
    }
    PlaceVerify shown = pv;
    drop_stale_bboxes(shown, looked - 0.05);
    draw_verify(bgr, shown, cam, top_z + exec_->puck_half_height_m(), exec_->puck_half_height_m());
    sensor_msgs::msg::Image out;
    out.header = frame->header;
    out.height = static_cast<uint32_t>(bgr.rows);
    out.width = static_cast<uint32_t>(bgr.cols);
    out.encoding = "bgr8";
    out.step = static_cast<uint32_t>(bgr.step);
    out.data.assign(bgr.datastart, bgr.dataend);
    pub_verify_image_->publish(out);
    std::string path;
    if (!dir.empty()) {
      path = dir + "/verify_" + name + ".jpg";
      if (!cv::imwrite(path, bgr)) {path = "(저장 실패) " + path;}
    }
    RCLCPP_INFO(get_logger(), "재검증 %s 그림 발행 %s%s%s%s%s", name.c_str(), verify_image_topic_.c_str(), path.empty() ? "" : " · ",
      path.c_str(), raw_path.empty() ? "" : " · 원본 ", raw_path.c_str());
  }

  // 이 장소 원시 관측 (since 뒤 촬영 · 장소 반경 안) → 퍽 단위로 묶음, 색 · 모양 다수결 (group_place_obs)
  //   로봇 몸 안 (촬영 시각 base 기준 차체 상자 + verify_body_margin_m) 검출은 뺌 — 적재함 · 받침 퍽, 차체 초록 부품
  std::vector<VerifyObs> gather_place_obs(const std::array<double, 2> & c, double since, int place = -1) const
  {
    std::vector<VerifyObs> raw;
    const auto tfb = exec_->tf_buffer();
    std::string base;
    if (has_parameter("base_frame")) {base = get_parameter("base_frame").as_string();}
    // 촬영 시각마다 map → base 한 번 (없으면 최신: 재검증 LOOK 동안 차체는 서 있음)
    double tf_stamp = -1.0;
    bool tf_ok = false;
    tf2::Matrix3x3 R;
    tf2::Vector3 T;
    int body_n = 0, no_tf_n = 0;
    double body_min_x = 1e9;
    for (const ObsRecord & o : exec_->recent_obs()) {
      if (o.stamp < since || std::hypot(o.x - c[0], o.y - c[1]) > verify_radius_m_) {continue;}
      if (tfb && !base.empty() && verify_body_margin_m_ >= 0.0) {
        if (o.stamp != tf_stamp) {
          tf_stamp = o.stamp;
          tf_ok = false;
          geometry_msgs::msg::TransformStamped t;
          try {
            try {
              t = tfb->lookupTransform(base, exec_->map_frame(), tf2::timeFromSec(o.stamp), tf2::durationFromSec(0.0));
            } catch (const tf2::TransformException &) {
              t = tfb->lookupTransform(base, exec_->map_frame(), tf2::TimePointZero);
            }
            const auto & q = t.transform.rotation;
            R.setRotation(tf2::Quaternion(q.x, q.y, q.z, q.w));
            T = tf2::Vector3(t.transform.translation.x, t.transform.translation.y, t.transform.translation.z);
            tf_ok = true;
          } catch (const tf2::TransformException &) {}
        }
        if (!tf_ok) {
          ++no_tf_n;
        } else {
          const tf2::Vector3 pb = R * tf2::Vector3(o.x, o.y, 0.0) + T;
          const double m = verify_body_margin_m_;
          if (pb.x() > verify_body_x_[0] - m && pb.x() < verify_body_x_[1] + m && std::abs(pb.y()) < verify_body_half_y_ + m) {
            ++body_n;
            body_min_x = std::min(body_min_x, pb.x());
            continue;
          }
        }
      }
      VerifyObs v;
      v.id = o.id;
      v.color = o.color;
      v.shape = o.shape;
      v.x = o.x;
      v.y = o.y;
      v.stamp = o.stamp;   // 같은 장 구별 (같은 ID 두 자리 = 두 퍽)
      v.bbox = o.bbox;     // 그림용 검출 상자 (묶을 때 마지막 장 것)
      v.z = o.z;           // 깊이 틀린 장 가려내기 (verify_plane_tol_m)
      v.cam = o.cam;
      raw.push_back(v);
    }
    if (place >= 0 && (body_n > 0 || no_tf_n > 0)) {
      RCLCPP_WARN(get_logger(), "재검증 L%d: 로봇 몸 안 검출 %d 장 뺌 (차체 앞 끝 %.3f + %.3f m, 가장 안쪽 x %.3f)%s", place + 1,
        body_n, verify_body_x_[1], verify_body_margin_m_, body_n > 0 ? body_min_x : 0.0,
        no_tf_n > 0 ? (" · TF 없어 몸 확인 못 한 장 " + std::to_string(no_tf_n)).c_str() : "");
    }
    auto g = group_place_obs(raw, 0.03, verify_vote_ratio_, verify_plane_tol_m_);
    if (place >= 0) {
      for (const auto & o : g) {
        if (o.off_plane_n <= 0) {continue;}
        RCLCPP_WARN(get_logger(), "재검증 L%d: 퍽 %s 깊이 틀린 장 %d 개 (윗면 높이에서 %.0f mm 넘게 벗어남) → %s (%.3f, %.3f)", place + 1,
          o.id.empty() ? "(ID 모름)" : o.id.c_str(), o.off_plane_n, 1000.0 * verify_plane_tol_m_,
          o.pos_from_earlier ? "위치는 그 전 맞는 장" : (o.all_off_plane ? "맞는 장 없어 마지막 장 그대로" : "마지막 장은 맞음, 그대로"),
          o.x, o.y);
      }
    }
    return g;
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
    // 색 · 모양 = 계획이 쓴 값 (실린 퍽은 적재 기록, 그 밖은 계획에 넘길 때 값). 기록부는 나중 관측으로 덮이므로
    //   (2트: 9 · 11 번을 빨강으로 실어 L3 에 놓았는데 기록부는 green) 다르면 seen_as 로만 남김
    auto note_cargo = [this](const std::optional<planning::CargoItem> & it) {
        if (it && !it->puck_id.empty()) {used_kind_[it->puck_id] = {it->color, it->shape};}
      };
    note_cargo(exec_->cargo().gripper());
    for (const auto & [slot, it] : exec_->cargo().slots()) {note_cargo(it);}
    for (const auto & rec : exec_->registry().records()) {
      if (rec.id.empty()) {continue;}
      AllocPuck ap{rec.id, rec.color, rec.shape, rec.state, ""};
      if (const auto u = used_kind_.find(rec.id); u != used_kind_.end()) {
        ap.color = u->second.first;
        ap.shape = u->second.second;
        if (rec.color != ap.color || rec.shape != ap.shape) {ap.seen_as = rec.color + "/" + rec.shape;}
      }
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
  // perception 재시작 (실행기가 검출 토픽 새 발행자로 앎) → 바코드 · 광각 켜고 끄기도 모름으로 → 다음 sync 에서 다시 보냄
  //   (usb_cam 캡처는 다른 노드라 그대로)
  void resync_after_perception_restart()
  {
    const int n = exec_->perception_restarts();
    if (n == perception_restarts_seen_) {return;}
    perception_restarts_seen_ = n;
    barcode_known_ = false;
    wide_.known = false;
    RCLCPP_WARN(get_logger(), "perception 재시작 → 바코드 판독 · 광각 처리 켜고 끄기 다시 맞춤");
  }

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
        if (gen != barcode_req_gen_) {return;}   // 버린 이전 요청의 늦은 응답
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
    double on_since_s = -1.0;    // 켬 확인 시각 (-1 = 꺼짐 · 모름). 신호 기록은 이 뒤 관측만 씀
    int gen = 0;
    bool was_off = false;        // 끔을 확인한 적 있음 (다시 켤 때 밝기 다시)
    std::function<void()> on_reenabled;   // 끔 확인 뒤 다시 켬 확인 때 부름
    std::string off_why;         // 끌 때 로그에 붙일 이유
  };

  // 광각을 신호 기록에 맞춤: Done 전 = 켬 (준비 확인 TRAFFIC_LIGHT 도 이때만 봄), Done = 끔
  //   perception 광각 처리 · usb_cam 캡처 둘 다. reset_signal 로 다시 기록하면 다시 켬
  //   OBSERVE_SIGNAL 단계가 남아 있는 동안 (실행 중 OBSERVE_SIGNAL) 은 어떤 경우에도 끄지 않음
  //   벽 판: 바코드 찾기가 남아 있으면 (대기 · 시작 HOME · READ_BARCODE, 아직 못 읽음) 신호와 상관없이 켬
  //     (대기 중 끄고 찾기 시작에 다시 켜면 밝기 다시 넣기 · 첫 장이 늦음)
  void sync_wide_camera(double now)
  {
    const bool observing = op_ == Op::RUNNING && phase_ == Phase::OBSERVE_SIGNAL;
    const bool searching = bc_active_ && bc_keep_wide_ && op_ == Op::RUNNING && phase_ == Phase::READ_BARCODE;   // 바코드 찾기: 광각 상자 씀
    bool want = signal_.status() != SignalStatus::Done || observing || searching || bc_wide_needed();
    std::string why = "신호 기록 · 바코드 찾기 끝";
    if (!signal_gate_open()) {   // 바코드 먼저: 읽기 전 광각은 찾기에 쓸 때만
      want = barcode_first_wide_ && (searching || bc_wide_needed());
      why = "바코드 먼저 — 신호 기록은 바코드 읽은 뒤, 아직 안 함";
    } else if (signal_.status() != SignalStatus::Done) {
      why = "신호 기록 안 끝남 (" + std::string(signal_status_name(signal_.status())) + ")";
    }
    capture_.off_why = wide_.off_why = why;
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
          t.on_since_s = want ? now_s() : -1.0;
          RCLCPP_INFO(get_logger(), "%s %s%s", t.name.c_str(), want ? "켬" : "끔", want ? "" : (" (" + t.off_why + ")").c_str());
          if (!want) {
            t.was_off = true;
          } else if (t.was_off && t.on_reenabled) {
            t.was_off = false;
            t.on_reenabled();
          }
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

  // 신호 기록에 쓸 관측인가: 광각 perception 켬이 확인된 뒤 관측만 (켜는 중 · 엔진 막 뜰 때 '?' 관측 버림)
  //   켜기 서비스를 안 쓰면 ("") 그대로 씀. 확인이 안 와도 관측이 kWideFallbackS 넘게 이어지면 씀 (기록을 막지 않음)
  bool wide_live(double t, bool from_msg)
  {
    if (!wide_.cli || (wide_.known && wide_.on && wide_.on_since_s >= 0.0 && t >= wide_.on_since_s)) {
      light_first_s_ = -1.0;
      if (!wide_live_logged_) {
        wide_live_logged_ = true;
        RCLCPP_INFO(get_logger(), "신호등: 광각 켜짐 확인 → 관측 기록 시작 (첫 확실한 색 점멸부터 셈)");
      }
      return true;
    }
    if (from_msg && (light_first_s_ < 0.0 || t < light_first_s_)) {light_first_s_ = t;}
    if (light_first_s_ < 0.0 || t - light_first_s_ < kWideFallbackS) {return false;}
    if (!wide_fallback_logged_) {
      wide_fallback_logged_ = true;
      RCLCPP_WARN(get_logger(), "신호등: %s 켬 확인 없이 관측 %.0f s 이어짐 → 그대로 기록", wide_.name.c_str(), kWideFallbackS);
    }
    return true;
  }

  // 로봇이 출발 구역에 있는 동안만 기록 (STANDBY · HOME · 첫 READ_BARCODE · OBSERVE_SIGNAL). 끝난 뒤에는 그대로 둠
  void record_signal(double t, SignalObs o, bool from_msg = true)
  {
    if (!signal_fixed_.empty() || !signal_record_ || signal_.finished() || signal_timed_out_) {return;}
    if (!signal_gate_open()) {return;}   // 바코드 먼저: 읽기 전 관측은 버림
    const bool still = op_ == Op::STANDBY || (op_ == Op::RUNNING && at_start_zone());
    if (!still) {return;}
    if (!wide_live(t, from_msg)) {return;}
    const auto before = signal_.status();
    const std::size_t n = signal_.blinks().size();
    const int parts = signal_.partials();
    signal_.observe(t, o);
    if (signal_.blinks().size() > n) {
      RCLCPP_INFO(get_logger(), "신호등 점멸 %zu: %c", signal_.blinks().size(), signal_.blinks().back().color);
    }
    if (signal_.partials() != parts) {
      RCLCPP_WARN(get_logger(), "신호등: 점멸 %zu 개 (%s) 뒤 끝 소등 → 처음을 못 본 시퀀스, 끝내지 않고 다음 시퀀스 기다림",
        signal_.last_partial().size(), signal_.last_partial().c_str());
      // partial 마다 바로 파일로 (다음 partial · 다시 기록 · 노드 꺼짐에도 남게). 번호는 실행 내내 이어짐 (덮어쓰지 않음)
      if (!signal_.partial_history().empty()) {
        const auto path = write_record("signal_partial" + std::to_string(++signal_partial_files_) + ".json",
          signal_partial_to_json(signal_.partial_history().back(), node_start_s_));
        if (!path.empty()) {RCLCPP_INFO(get_logger(), "신호 partial 기록 → %s", path.c_str());}
      }
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
        return exec_->hold_wanted();   // 위치 복구 물러나기 이동 중엔 풂
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
  double verify_radius_m_ = 0.18, verify_cell_tol_m_ = 0.05, verify_vote_ratio_ = 0.7;
  VerifyFitParams verify_fit_;
  double verify_on_box_margin_m_ = 0.0;
  std::string verify_look_slot_ = "far";       // 재검증 LOOK 자세 (verify_steps)
  std::array<double, 2> verify_body_x_{{-0.133, 0.141}};   // 차체 뒤 · 앞 끝 (base) [m]
  double verify_body_half_y_ = 0.150, verify_body_margin_m_ = 0.03;
  double verify_ignore_inset_m_ = 0.015;
  double verify_plane_tol_m_ = 0.03;
  bool verify_cell_ids_ = true;
  int transfer_max_remaining_ = 4, max_replans_ = 6;
  bool alloc_prefer_more_pucks_ = false;         // 같은 장소 수면 놓는 퍽 많은 쪽 (choose_delivery)
  bool fill_partial_places_ = false;             // 장소마다 놓을 수 있는 만큼 (choose_delivery partial)
  int place_nav_fail_n_ = 2;
  std::array<int, kNumPlaces> place_nav_fails_{};   // 장소별 이동 실패 횟수 (이번 task)
  bool auto_start_ = false, barcode_look_ = true;
  bool rack_look_ = false;
  bool box_measure_ = true;                  // 장소 박스 측정으로 칸 기준 고침
  std::string box_topic_;
  int box_min_samples_ = 3;
  double box_max_shift_m_ = 0.15, box_wait_s_ = 2.0;
  double box_realign_shift_m_ = 0.04, box_realign_yaw_rad_ = 8.0 * M_PI / 180.0;   // 다시 붙기 기준
  int box_realign_max_ = 1;
  std::array<int, kNumPlaces> box_realign_n_{};   // 장소별 다시 붙은 횟수 (이번 task)
  std::array<bool, kNumPlaces> place_measured_{};   // 이 task 에서 박스를 잰 장소 (다시 와도 LOOK · 측정 안 함, 다시 붙기 중엔 false)
  std::array<std::optional<PlaceFrame>, kNumPlaces> realign_prev_{};   // 다시 붙기 전 측정 (다시 붙은 뒤 못 재면 씀)
  PlaceLimits place_limits_;                 // 놓는 칸 박스 테두리 안 범위
  std::array<std::optional<PlaceFrame>, kNumPlaces> place_frame_{};   // 장소별 확정한 박스 기준 (없으면 지도 값)
  // 계획에 넘긴 퍽 색 · 모양 (ID → 색, 모양). allocation.json 이 기록부 대신 씀
  mutable std::map<std::string, std::pair<std::string, std::string>> used_kind_;
  std::array<std::optional<PlaceFrame>, kNumPlaces> placed_frame_{};  // 장소별 마지막 PLACE 가 쓴 박스 기준 (재검증용, LOOK 이 안 지움)
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
  struct Resume                              // FAULT 로 멈춘 task 이어 하기 (resume_continue_task, hold_task_for_resume)
  {
    bool active = false;                     // resume → start → HOME 뒤 continue_task
    Phase phase = Phase::COLLECT;            // FAULT 때 단계
    ResumePoint point;                       // FAULT 때 하던 단계 · 남은 박스 · 남은 단계 (dice_steps.hpp)
    std::string code;                        // FAULT 원인
    bool repeat = false;                     // 지난 FAULT 와 같은 원인 · 같은 단계 (그 사이 그 단계 성공 없음)
  } resume_;
  struct Reconcile                           // 재고 자동 정합 중 (start_reconcile → 확인 LOOK → run_reconcile_check)
  {
    bool active = false;
    Step f;                                  // 실패한 재고 작업
    planning::CargoDoubt doubt;              // 그때 불확실 표시
    std::string look_name;                   // 확인 LOOK 단계 이름
    double look_s = 0.0;                     // 확인 LOOK 보낸 시각 (이 뒤 검출만 씀)
    std::array<double, 2> cell{{0.0, 0.0}};  // PLACE: 그 칸 map 자리 (확인 LOOK 전에 구함)
    bool home_skipped = false;               // RETRIEVE: HOME 없이 스캔 보냄
    bool home_retried = false;               // 그 스캔이 실패해 HOME 뒤 다시 보냄 (한 번만)
    bool look_rack = false;                  // 확인 = 그 칸 빈손 LOOK_RACK (reconcile_look_rack, 받침에선 못 보냄 → HOME 뒤 다시 안 함)
  } reconcile_;
  bool recover_no_penalty_ = false;
  bool reconcile_conflict_ = false;          // 자동 정합이 증거 불일치로 FAULT → 그 FAULT 는 스스로 정합 (그리퍼만) 안 함          // 자동 정합 뒤 재계획: 실패 작업 벌칙 (퍽 건너뜀 · 장소 버림) 안 줌
  struct RackSeen {double t; bool puck; std::string cell, id;};
  std::deque<RackSeen> rack_seen_;           // /rack_tags/detections 최근 60 s (rack_tag = 빈 칸 바닥 · rack_puck = 칸 안 퍽)
  rclcpp::Subscription<interfaces::msg::DetectionArray>::SharedPtr sub_rack_tags_;
  double reconcile_wait_s_ = 1.5, reconcile_place_tol_m_ = 0.03;
  bool reconcile_scan_skip_home_ = true;
  std::string rack_tags_topic_;
  std::string resume_last_key_, resume_last_work_;   // 마지막 FAULT 열쇠 (원인|단계) · 하던 단계 이름, 그 단계가 성공하면 비움
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

  // 신호등 점멸 기록 (발제)
  SignalRecorder signal_;
  bool signal_record_ = true;
  bool barcode_first_ = true, barcode_first_wide_ = false;
  int signal_partial_files_ = 0;   // signal_partial<N>.json 번호 (실행 내내, B2)
  std::string signal_fixed_, barcode_fixed_, start_phase_, stop_after_;   // 시험용 부분 실행 (setup_partial_run)
  double signal_timeout_s_ = 1.0, signal_wait_s_ = 90.0;
  double observe_start_s_ = 0.0;
  int signal_retries_ = 2;
  int signal_attempt_ = 0;                   // 다시 기록한 횟수 (불완전 기록은 signal_attempt<N>.json 으로 남김)
  std::string signal_prev_;                  // 바로 앞 불완전 기록 순서 (상태 · 발행용)
  bool signal_timed_out_ = false;            // OBSERVE_SIGNAL 시간 초과 (불완전으로 진행, reset_signal 전까지 다시 안 봄)
  double last_light_s_ = -1.0;
  static constexpr double kWideFallbackS = 5.0;   // wide_live: 켬 확인 없이 관측이 이만큼 이어지면 그대로 씀
  double light_first_s_ = -1.0;              // wide_live: 켬 확인 전 첫 관측 시각
  bool wide_live_logged_ = false, wide_fallback_logged_ = false;

  // dice
  SignalCounts dice_counts_;
  std::optional<DicePlan> dice_plan_;        // 장소 그룹 · 눈 위치 (PLACE 목표 계산)
  std::vector<Step> delivery_steps_;         // Plan(Delivery) 가 만든 배달 단계 (COLLECT 끝나면 실행)
  std::vector<Step> verify_after_;           // 재검증 단계
  std::array<int, kNumPlaces> placed_count_{{0, 0, 0}};   // 장소별 놓은 수 (PLACE 성공)
  std::array<std::vector<std::string>, kNumPlaces> placed_cell_ids_;   // 장소별 칸 번호 → 놓은 퍽 ID (PLACE 성공, 재검증 칸 ID 판정)
  std::array<bool, kNumPlaces> place_dead_{{false, false, false}};   // PLACE 실패한 장소 (더 안 건드림)
  bool box_relook_ = false;
  double box_relook_back_m_ = 0.12, box_relook_dwell_s_ = 1.5, box_relook_radius_m_ = 0.25, box_relook_max_shift_m_ = 0.25;
  bool box_recenter_on_look_ = true;                 // 박스 LOOK 뒤 퍽으로 박스 중심 다시 잡기 (recenter_box_on_look)
  std::array<bool, kNumBoxes> box_relooked_{};       // 박스마다 다시 찾기 한 번만 (start 마다 초기화)
  // 누운 퍽 뒤로 빼기
  LyingBackoffConfig lying_cfg_;
  StraightBackConfig lying_move_;
  double lying_drive_wait_s_ = 3.0;
  std::string lying_cmd_topic_ = "/cmd_vel_nav", lying_cm_node_ = "/collision_monitor";
  std::vector<std::string> lying_polygons_{"PolygonStop"};
  // 뒤로 뺀 뒤 다시 앞으로: 뒤로 뺀 자리에서는 걸린 퍽 (lying_trigger_) 만 집고, 그 박스 정차 이동 (lying_nav_, 정밀 접근) 을 다시 해서
  //   남은 퍽은 원래 정차 자리에서 집음 (뒤로 뺀 자리에서는 선 퍽이 멀어져 손이 안 닿을 수 있음)
  std::array<bool, kNumBoxes> lying_fwd_{};          // 다시 앞으로 가야 함
  std::array<int, kNumBoxes> lying_trig_n_{};        // 걸린 퍽을 뒤로 뺀 자리에서 계획한 횟수
  std::array<std::string, kNumBoxes> lying_trigger_{};
  std::array<Step, kNumBoxes> lying_nav_{};
  std::array<bool, kNumBoxes> lying_nav_ok_{};
  std::array<std::set<std::string>, kNumBoxes> lying_backed_ids_{};   // 뒤로 빼기를 일으킨 퍽 — 퍽마다 한 번 (start 마다 초기화)
  double lying_unseen_s_ = 1.2;                      // 앞 LOOK 자세에 서 있는데 이만큼 검출이 없으면 '지금 자세에서 안 보임'
  Step lying_look_, lying_plan_;                     // 뒤로 간 뒤 끼울 LOOK · Plan
  struct LyingRun
  {
    using Phase = LyingPhase;
    bool active = false;
    std::string name;
    int box = 0;
    Phase phase = Phase::WaitDrive;
    double goal_m = 0.0, t0 = 0.0, t_phase = 0.0, progress_s = 0.0, best = 0.0;
    double x0 = 0.0, y0 = 0.0, yaw0 = 0.0;
    bool have_start = false;                         // 시작 자리 (odom) 잼
    StraightBackCmd last;
  };
  LyingRun lying_;
  bool lying_cm_pending_ = false, lying_polys_off_ = false;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr lying_cmd_pub_;
  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr lying_cm_cli_;
  rclcpp::TimerBase::SharedPtr lying_timer_;
  std::array<Step, kNumBoxes> box_relook_nav_{};     // 다시 찾기 뒤 다시 넣을 그 박스 이동 (정차점만 바꿈)
  bool box_relook_use_box_detector_ = true, box_relook_places_ = true;
  std::vector<std::string> box_relook_side_looks_;
  std::string box_relook_map_topic_;
  std::vector<double> box_relook_body_{0.16, 0.16, 0.03};
  std::array<bool, kNumPlaces> place_relooked_{};   // 장소마다 다시 찾기 한 번만
  std::array<Step, kNumPlaces> place_relook_nav_{};
  std::array<Pose2D, kNumBoxes + kNumPlaces> relook_stand_{};   // 다시 찾기 LOOK 자리 (면 고르기 기준), [박스 0–5 · 장소 6–8]
  std::array<int, kNumBoxes + kNumPlaces> relook_side_i_{};     // 다음에 쓸 box_relook_side_looks 순번
  DiceMap dice_map_file_;                    // 파일에서 읽은 그대로 (다시 찾기 정차 거리)
  std::array<bool, kNumPlaces> place_relocated_{};   // 다시 찾기가 측정으로 이 장소 박스를 옮김 (측정 못 받을 때 그 기준으로 놓음)
  std::array<double, kNumPlaces> place_box_dyaw_{};  // 다시 찾기로 잰 장소 박스 방향 − 지도 방향 (±45°), 칸 기준 측정 · 자르기의 기준 방향에 더함
  OccGrid relook_map_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_map_;
  std::string yellow_used_shape_;            // Y 장소에 놓은 모양 (재계획 때 고정)
  int replans_ = 0;
  int store_return_skip_n_ = 2;
  // 쥔 퍽 STORE 실패 칸 · 박스 되돌림
  int store_alt_cells_ = 1;
  std::map<std::string, std::set<std::string>> store_failed_slots_;   // 퍽 ID → STORE 가 실패한 칸
  struct PickPoint {double x = 0.0, y = 0.0; int nav_n = 0;};
  std::map<std::string, PickPoint> pick_points_;                     // 퍽 ID → PICK 보낼 때 기록부 자리 (map) · 그때 이동 성공 수
  std::set<std::string> retrieved_here_;                             // RETRIEVE 로 손에 온 퍽 (박스 되돌림 대상 아님)
  std::set<std::string> putback_tried_;                              // 되돌림 단계를 이미 만든 퍽 (실패하면 다시 안 함)
  std::string abandon_after_putback_;                                // 되돌림 성공 뒤 포기 · 복귀 (abandon_dice 이유)
  int nav_ok_total_ = 0;                                             // 노드 켠 뒤 성공한 이동 단계 수 (되돌림 = 집은 정차 그대로인지)
  int empty_grasp_skip_n_ = 2;
  std::map<std::string, int> empty_grasp_n_;     // 퍽별 PICK 빈손 횟수 (이번 task)
  int retrieve_empty_skip_n_ = 2;
  int retrieve_reject_skip_n_ = 1;
  std::map<std::string, int> retrieve_reject_n_;   // 퍽별 RETRIEVE control 계속 거절 횟수 (이번 task)
  bool reconcile_look_rack_ = true;
  std::map<std::string, int> retrieve_empty_n_;  // 퍽별 RETRIEVE 빈손 (칸에 남음) 횟수 (이번 task)
  std::set<std::string> rack_stuck_;             // 칸에 둔 채 계획에서 뺀 퍽 (못 꺼냄 · 확인 못 함, 이번 task). 칸은 찬 것으로 둠
  bool box_retry_later_ = true;
  std::set<int> box_retried_;                    // 이동 실패로 한 번 뒤로 미룬 박스 (이번 task)
  bool nav_fault_before_first_ = true;
  int nav_fault_auto_n_ = 2;          // NAV_NOT_WORKING 스스로 이어 가기 횟수 (task 마다)
  double fault_auto_wait_s_ = 5.0;
  int nav_fault_auto_used_ = 0;       // 이번 task 에서 쓴 횟수
  bool nav_first_waived_ = false;     // 이번 task: 첫 이동 FAULT 안 냄 (스스로 이어 가기 2 번째부터)
  double auto_resume_at_ = -1.0;      // FAULT 중 이 시각에 스스로 resume
  bool auto_start_once_ = false;      // 스스로 resume 뒤 준비되면 한 번 start
  // 재고 불확실 스스로 정합
  int inventory_auto_n_ = 2;
  double inventory_auto_timeout_s_ = 15.0;
  int inventory_auto_used_ = 0;       // 이번 task 에서 쓴 횟수
  double inv_auto_from_ = -1.0, inv_auto_until_ = -1.0;   // FAULT 중 이 시각부터 손 상태 보고 정합 (until 까지)
  bool hand_first_once_ = false;      // 정합 결과 쥠 → 다음 start 는 HOME 대신 손 비우기부터
  bool continue_after_empty_ = false; // 손 비우기 뒤 task 이어 하기 (continue_task)
  std::string hand_topic_, hand_joint_;
  HandConfig hand_cfg_;
  std::deque<HandSample> hand_samples_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_hand_;
  int nav_ok_since_start_ = 0;                   // start (이어 하기 포함) 뒤 성공한 이동 단계 수
  std::set<int> box_look_retried_;               // 앞 LOOK 실패로 LOOK 을 한 번 더 한 박스 (이번 task)
  std::map<std::string, int> store_returned_n_;   // 퍽별 STORE 박스 되돌림 횟수 (이번 미션)
  int current_node_ = kNodeStart;            // 마지막으로 도착한 노드
  double last_look_sent_s_ = -1.0;               // 마지막 LOOK 보낸 시각 (gap_guard)
  std::map<std::string, int> gap_redo_;           // 단계 (kind:박스:장소:퍽) → 검출 끊김 뒤 LOOK 다시 한 수 (begin_running 마다 비움)
  static constexpr int kGapRedoMax = 2;
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
  // 바코드 찾기 (declare_bc)
  static constexpr uint8_t kArmLookAtBc = 9;   // control_master LOOK_AT (ArmTask.action 에 이름 없음)
  bool lying_forward_ = true;   // lying_backoff_forward: 뒤로 뺀 자리에서는 걸린 퍽만 집고 다시 박스 앞으로
  bool bc_base_moved_ = false, signal_return_start_ = true, observe_returning_ = false;
  double signal_back_m_ = 0.20;   // (signal_return_start: 바코드 뒤 신호 기록은 출발 자리에서)
  bool barcode_search_ = true, bc_fallback_fixed_ = true, bc_keep_wide_ = true, bc_active_ = false;
  std::string bc_wide_frame_ = "birdeye_camera_optical_frame";
  double bc_wide_pitch_offset_deg_ = 4.0, bc_light_lost_s_ = 1.0, bc_look_timeout_s_ = 30.0;
  double bc_light_cone_margin_deg_ = 4.0, bc_light_cone_min_deg_ = 6.0;
  double bc_wide_yaw_offset_deg_ = 0.0;
  std::string bc_last_slot_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_bc_snap_;
  bool test_return_home_ = true;
  double bc_view_pitch_deg_ = 43.0, bc_sweep_view_pitch_deg_ = 20.0;
  double bc_max_arm_out_s_ = 15.0, bc_fold_rest_s_ = 5.0, bc_arm_out_since_ = -1.0;
  double bc_arm_out_total_s_ = 40.0, bc_arm_out_total_ = 0.0;
  double bc_wide_settle_s_ = 1.5, bc_wide_wait_s_ = 6.0, bc_wide_valid_from_s_ = 0.0, bc_wide_first_s_ = -1.0, bc_wide_last_s_ = -1.0;
  bool bc_move_planned_ = false;
  bc::SearchParams bc_params_;
  bc::Fisheye bc_wide_;
  bc::Pinhole bc_wrist_;
  bc::Zone bc_zone_;
  bc::BarcodeSearch bc_;
  double bc_t0_ = 0.0, bc_look_sent_s_ = -1.0, bc_mode_reason_s_ = -1.0, bc_light_seen_s_ = -1.0, bc_dwell_from_s_ = 0.0;
  bool bc_dwell_on_ = false, bc_light_before_look_ = false, bc_wide_tf_ok_ = false;
  std::string bc_mode_reason_;
  std::optional<bc::Box> bc_wide_bc_, bc_wide_light_, bc_wrist_box_;
  std::optional<bc::V3> bc_wrist_pt_, bc_wall_map_;
  bc::Pose3 bc_T_map_wrist_, bc_T_base_wide_;
  std::optional<geometry_msgs::msg::PoseStamped> bc_target_;
  std::map<std::string, bc::Pose3> bc_scan_tf_;
  std::vector<std::vector<std::array<double, 2>>> bc_scan_pts_;
  rclcpp::Subscription<interfaces::msg::DetectionArray>::SharedPtr sub_bc_wide_, sub_bc_wrist_;
  rclcpp::Subscription<interfaces::msg::MotionMode>::SharedPtr sub_bc_mode_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr> sub_bc_scan_;

  // perception 광각 켜기 · 끄기 (신호 기록 Done 전까지만)
  std::string wide_enable_service_, wide_capture_service_;
  std::string wide_device_;           // 캡처 다시 켬 → v4l2 밝기 다시 넣음 ("" = 안 함)
  int wide_brightness_ = -20;
  int bc_panel_rej_logged_ = 0;       // 판 밖 광각 상자 로그
  EnableToggle wide_, capture_;   // perception 광각 처리 · usb_cam 캡처
  int perception_restarts_seen_ = 0;

  // ROS
  rclcpp::Subscription<TrafficLight>::SharedPtr sub_sig_;
  rclcpp::Subscription<TaskText>::SharedPtr sub_text_;
  rclcpp::Service<Trigger>::SharedPtr srv_start_, srv_resume_, srv_clear_gripper_, srv_reset_signal_;
  std::vector<rclcpp::Service<Trigger>::SharedPtr> srv_store_done_;   // /mission/store_done/<칸>
  std::vector<rclcpp::Service<Trigger>::SharedPtr> srv_inventory_;   // /mission/inventory_before · done · removed · returned
  rclcpp::Publisher<MissionStatus>::SharedPtr pub_status_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_hold_;
  rclcpp::Publisher<SignalRecord>::SharedPtr pub_signal_;
  rclcpp::Publisher<VerifyRecord>::SharedPtr pub_verify_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_verify_image_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr verify_color_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr verify_info_sub_;
  sensor_msgs::msg::Image::ConstSharedPtr verify_frame_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr verify_info_;
  std::string verify_image_topic_, verify_color_topic_, verify_info_topic_, verify_camera_frame_;
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
