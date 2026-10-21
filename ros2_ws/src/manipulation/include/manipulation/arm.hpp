// arm.hpp — 팔 쪽 순수 계산 전부: 동작 순서표 · 차례 정하기 · 팔 작업 진행
// =====================================================================
// ROS 를 모르는 순수 C++ → test/test_arm.cpp. 쓰는 곳: control_master.cpp (ROS 입출력만 함)
//
//   ① make_arm_plan  : HOME · PICK · PLACE → 동작 순서표 (팔 자세 / 그리퍼 한 동작씩)
//   ② MotionArbiter  : "지금 누구 차례" DRIVE · STOPPING · ARM · STOP  (D15 한 번에 한 무리)
//   ③ ArmRunner      : 팔 작업 하나를 보내고 · 기다리고 · 취소하고 · 끝내는 상태 기계 (언제 바퀴를 풀어도 되나를 정함)
//   ④ FoldCheck      : "팔이 정말 접혔나" 를 /joint_states 실측으로 확인 (2026-10-16 R1)
//   ⑤ HardwareWatch  : 하드웨어 측정이 살아 있나 (끊김 · NaN 지속 → HOME 확인 무효, STOP) (2026-10-16 재검증 R6)

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace manipulation
{

// ============================================================================
// ① 팔 동작 순서표
// planning 은 무엇을 할지만 말하고(ArmTask: HOME · PICK · PLACE), 순서는 여기서 정한다.
// 동작 하나 = "팔을 어느 자세로" 또는 "그리퍼를 얼마나" 중 하나 (팔과 그리퍼도 동시에 안 움직임 — D15)
// 자세 이름의 실제 관절 각도는 config/control.yaml 의 poses.* 에 있다.
//
// 순수 C++ → test/test_arm.cpp 로 시험
// ============================================================================

// 값은 interfaces/action/ArmTask 와 같게
constexpr uint8_t kArmHome = 0;
constexpr uint8_t kArmPick = 1;
constexpr uint8_t kArmPlace = 2;

struct ArmAction
{
  std::string phase;     // 진행 상황 이름 (ArmTask feedback): approach · grasp · lift · release · retreat
  bool is_gripper;       // false = 팔 자세로, true = 그리퍼만
  std::string pose;      // 팔: 자세 이름 (home, pick_above …)
  double gripper;        // 그리퍼: 손가락 위치 [m]
};

// 모르는 작업 번호면 빈 목록. 마지막 동작은 항상 접기(home) → 끝나면 바로 주행해도 안전
std::vector<ArmAction> make_arm_plan(uint8_t task, double gripper_open, double gripper_close);

// ============================================================================
// ② 차례 정하기 (MotionArbiter)
// D15: 모터 전원을 따로 빼서 쓰되, 순간 전류를 줄이려고 **한 번에 한 무리만** 움직인다.
//   DRIVE    : 바퀴 · 조향만. 팔은 접은 자세로 버티기 (기본)
//   STOPPING : 팔 작업이 들어옴 → 바퀴를 서서히 세우는 중. 몸통이 settle_s 동안 멈춰 있어야 ARM 으로
//   ARM      : 팔 · 그리퍼만. 바퀴는 0 으로 버티기
//   STOP     : 켤 때 · 재시작 · 팔 작업이 실패 · 취소됨 → 팔이 어떤 자세인지 모르니 바퀴도 팔도 안 움직임
//              (HOME 처럼 "접기로 끝나는" 팔 작업이 끝까지 성공해야 풀림 — planning INIT 이 맨 먼저 HOME 을 보냄)
//              취소는 HOME 을 시작하기 전이라도 STOP 유지 (취소됐다고 팔이 접혀 있는 건 아니니까)
//
// control_master 가 차례를 정해서 /control/motion_mode 로 방송 → swerve_controller 는 DRIVE 일 때만 바퀴를 움직임 (D8).
// ARM 으로 넘어가는 조건은 "방송했다" 가 아니라 "바퀴 쪽이 이 번호로 잠갔다고 답함 + /odom 으로 settle_s 동안 멈춘 걸 봤다"
// → 방송이 안 닿으면 잠금 답이 없고 → 팔도 시작 안 함
//
// ROS 를 모르는 순수 C++ → test/test_arm.cpp 로 시험
// ============================================================================

// 값은 interfaces/msg/MotionMode 와 같게 (상태 방송용)
enum class Turn : uint8_t { STOP = 0, DRIVE = 1, ARM = 2, STOPPING = 3 };

const char * turn_name(Turn t);

class MotionArbiter
{
public:
  // start_locked: 켤 때 STOP 으로 시작 (팔 자세를 모르니 바퀴도 잠금). HOME 작업이 끝나야 DRIVE
  //   → control_master 가 팔 동작 중에 재시작돼도 "켜자마자 DRIVE" 로 바퀴가 풀리지 않음
  explicit MotionArbiter(double settle_s, bool start_locked = true)
  : settle_s_(settle_s), turn_(start_locked ? Turn::STOP : Turn::DRIVE) {}

  // 팔 작업이 들어옴 → 바퀴부터 세우기 시작 (새 요청 번호)
  void request_arm();
  // 팔 작업이 끝남. drive_ok = "끝까지 성공했고 마지막 동작이 접기(home)" (ArmRunner 가 정함)
  //   true  → DRIVE (바퀴 차례로)
  //   false → STOP  (취소 · 실패 · 시간 초과 — 팔을 움직였든 안 움직였든. 다음 HOME 이 성공해야 풀림)
  // DRIVE 로 가는 길은 이것 하나뿐 ("취소인데 팔을 안 움직였으니 DRIVE" 같은 예외를 두지 않음)
  void arm_ended(bool drive_ok);
  // 하드웨어 이상 (HardwareWatch) → 무조건 STOP. 그동안의 HOME 확인은 무효 → 다음 HOME 이 성공해야 DRIVE
  //   (DRIVE 로 가는 길은 여전히 arm_ended(true) 하나뿐)
  void force_stop() {set(Turn::STOP);}
  // Continuous permit, not just a one-time HOME check. A fault latches STOP;
  // fresh measurements alone cannot restore DRIVE (a successful HOME must).
  bool enforce_drive_guard(bool folded_fresh, bool hardware_healthy);
  // 매 주기 호출. STOPPING → ARM 조건:
  //   ① 바퀴 컨트롤러가 **이번 요청 번호**의 잠금을 반영했다고 답함 (acked_id == request_id())
  //   ② 그 뒤 몸통이 settle_s 동안 계속 멈춤 (base_still — 측정 시각이 새로운 /odom 으로)
  //   예전 요청의 답(번호가 다름)은 무시 → "멈춰 있었을 뿐 잠기지는 않은" 상태로 팔이 시작되지 않음
  void update(double now_s, bool base_still, uint32_t acked_id);

  Turn turn() const {return turn_;}
  uint32_t request_id() const {return request_id_;}   // 차례가 바뀔 때마다 1 씩 (방송에 같이 실음)
  bool wheels_may_move() const {return turn_ == Turn::DRIVE;}
  bool arm_may_move() const {return turn_ == Turn::ARM;}

private:
  void set(Turn t);
  double settle_s_;
  Turn turn_;
  uint32_t request_id_{1};
  double still_since_{-1.0};   // 잠금 확인 + 몸통이 멈춘 시각 (음수 = 아직)
};

// ============================================================================
// ③ 팔 작업 진행 (ArmRunner)
// 왜 노드 밖 순수 C++ 인가: "언제 바퀴 잠금을 풀어도 되나" 는 안전에 가장 중요한 판단인데,
//   ROS 노드 안에 있으면 단위 시험으로 모든 경우(시작 전 취소 · 늦은 답 · 시간 초과 …)를 확인할 수 없음 → test_arm.cpp
//
// 쓰는 쪽(control_master)은 ROS 입출력만 한다:
//   ① 사건을 알려 줌: start(작업 시작) · request_cancel(상위가 취소) · on_goal_response · on_result (재생기 답)
//   ② 주기마다 tick(지금 시각, 팔 차례인가) → 돌려받은 명령(ArmCommand)을 그대로 실행
//        SEND          : 이 동작을 재생기(JTC)에 보냄 (gen 번호를 같이 기억)
//        CANCEL_ACTIVE : 지금 궤적(gen)에 취소 요청 (아직 받아들여지기 전이면 전체 취소)
//        FINISH        : 상위(ArmTask)에 결과를 답하고, drive_ok 로 차례를 정함
//
// ★ 차례 규칙: FINISH 의 drive_ok 는 **끝까지 성공 + 마지막 동작이 접기(home) + 실측으로 접힘 확인** 일 때만 true.
//   (2026-10-16 R1) "재생기(JTC)가 성공이라고 함" 만으로는 부족 — 어깨가 걸려 0 rad 에 있어도 JTC 는 성공을 돌려줬음
//   (목표 오차 검사가 꺼져 있었음). 그래서 마지막 home 동작이 끝나면 VERIFY 단계로 들어가
//   신선한 관절 측정값이 home 범위 안에 hold_s 동안 머무는지(FoldCheck) 본 뒤에만 DRIVE.
//   verify_timeout_s 안에 확인 못 하면 실패(ABORTED) → STOP.
//   취소 · 실패 · 시간 초과는 팔을 움직였든 안 움직였든 언제나 false → 바퀴 잠금(STOP) 유지.
//   왜: "이번 작업에서 궤적을 아직 안 보냈다" 는 팔이 접혀 있다는 뜻이 아님 — 켤 때 · 재시작 직후 · 앞 작업 실패 뒤에는
//   팔 자세를 모르고, 죽은 control_master 가 보낸 궤적이 아직 돌고 있을 수도 있음 (그때 바퀴를 풀면 팔 · 바퀴가 동시에 움직임)
// ============================================================================

enum class ArmEnd : uint8_t { SUCCEEDED, ABORTED, CANCELED };

struct ArmCommand
{
  // VERIFY_START: 마지막 home 동작이 끝남 → 접힘 실측 확인을 처음부터 시작하라 (FoldCheck::reset)
  enum class Kind : uint8_t { SEND, CANCEL_ACTIVE, VERIFY_START, FINISH };
  Kind kind{Kind::FINISH};
  ArmAction action{};       // SEND: 보낼 동작
  unsigned gen{0};          // SEND: 이 동작의 세대 번호 / CANCEL_ACTIVE: 취소할 궤적의 세대 번호
  ArmEnd end{ArmEnd::ABORTED};   // FINISH
  bool drive_ok{false};     // FINISH: 바퀴 차례로 돌아가도 되나 (= 성공 && 마지막 동작이 접기 && 실측 접힘 확인)
  std::string message;      // FINISH: 상위에 돌려줄 설명
};

// 재생기에 늦게 온 "받아들였음" 답을 어떻게 할지
enum class LateGoal : uint8_t { USE, IGNORE, CANCEL_IT };

class ArmRunner
{
public:
  // accept_timeout_s: 재생기가 "받았다" 고 답할 때까지
  // motion_timeout_s: 받은 뒤 끝날 때까지 (move_time + 여유)
  // cancel_timeout_s: 취소를 보낸 뒤 "멈췄다(결과)" 답을 기다리는 최대 시간
  // verify_timeout_s: 마지막 home 동작 뒤 "실측으로 접힘" 을 확인할 때까지 기다리는 최대 시간
  ArmRunner(double accept_timeout_s, double motion_timeout_s, double cancel_timeout_s, double verify_timeout_s = 2.0)
  : accept_timeout_s_(accept_timeout_s), motion_timeout_s_(motion_timeout_s), cancel_timeout_s_(cancel_timeout_s),
    verify_timeout_s_(verify_timeout_s) {}

  // 새 작업 시작 (빈 계획은 다음 tick 에서 바로 실패로 끝남)
  void start(std::vector<ArmAction> plan);
  bool active() const {return active_;}

  // 상위(ArmTask)가 취소를 요청함. 여러 번 불러도 같음
  void request_cancel() {if (active_) {cancel_requested_ = true;}}
  // 하드웨어 이상 등으로 지금 작업을 실패로 끝냄 (다음 tick 에 진행 중 궤적 취소 + ABORTED, drive_ok = false)
  //   취소와 달리 재생기의 "멈췄음" 답을 기다리지 않음 (하드웨어가 멈춘 상태라 답이 안 올 수 있음)
  void fail_now(const std::string & why) {if (active_) {forced_ = true; forced_why_ = why;}}

  // 재생기의 "받았다/거절" 답. 이미 포기한 세대의 답이면 IGNORE / CANCEL_IT (받아들여졌으면 그것만 골라 취소)
  LateGoal on_goal_response(unsigned gen, bool accepted, double now_s);
  // 재생기의 결과 (ok = 성공). 포기한 세대면 무시
  void on_result(unsigned gen, bool ok);

  // 주기마다. arm_may_move = 지금 팔 차례(ARM)인가 (MotionArbiter 가 정함)
  //   fold_confirmed = FoldCheck::confirmed(now) — VERIFY 단계에서만 봄 (VERIFY_START 이후의 측정만 인정)
  std::vector<ArmCommand> tick(double now_s, bool arm_may_move, bool fold_confirmed = false);

  // 상태 (로그 · 테스트용)
  std::size_t next_index() const {return idx_;}
  bool busy() const {return busy_;}
  bool verifying() const {return verifying_;}
  unsigned gen() const {return gen_;}

private:
  ArmCommand finish(ArmEnd end, const std::string & message);
  void abandon(std::vector<ArmCommand> & out, const std::string & why);
  std::string current_phase() const;

  double accept_timeout_s_, motion_timeout_s_, cancel_timeout_s_, verify_timeout_s_;
  std::vector<ArmAction> plan_;
  std::size_t idx_{0};
  bool active_{false};
  bool busy_{false};          // 궤적을 보냈고 결과를 아직 못 받음
  bool accepted_{false};      // 지금 궤적을 재생기가 받아들였음
  bool failed_{false};
  std::string fail_why_;
  bool cancel_requested_{false}, cancel_sent_{false};
  double accept_deadline_{0.0}, motion_deadline_{0.0}, cancel_deadline_{0.0}, verify_deadline_{0.0};
  bool verifying_{false};     // 마지막 home 동작이 끝나고 실측 접힘을 기다리는 중
  bool forced_{false};        // fail_now 받음
  std::string forced_why_;
  unsigned gen_{0};           // 보낸 궤적마다 1 씩. 포기 · 취소하면 또 1 → 늦은 답을 구분
};

// ============================================================================
// ④ 접힘 실측 확인 (FoldCheck) — 2026-10-16 R1
// "팔이 HOME 에 있다" 를 재생기의 성공 답이 아니라 **관절 측정값**으로 확인한다.
//   - 측정 묶음마다 observe(측정 시각, 관절값들). 관절이 빠졌거나 NaN 이면 그 묶음은 "모름" → 처음부터 다시
//     (하드웨어는 못 읽은 모터를 NaN 으로 내보냄 — opencr_hardware.cpp, R3)
//   - 측정 시각이 앞으로 가지 않은 묶음은 무시 (같은 옛날 메시지가 반복돼도 "계속 맞음" 으로 세지 않게)
//   - 측정과 측정 사이가 max_age_s 넘게 벌어지면 유지 시간을 처음부터 (끊긴 동안은 "머물렀다" 고 볼 수 없음)
//   - confirmed(now): 마지막 측정이 max_age_s 안이고, 모든 관절이 |값 − home| ≤ tol 인 상태가 hold_s 이상 이어짐
// 그리퍼는 여기서 보지 않는다: 물체를 쥐면 닫힘 목표에 못 미치는 게 정상이라 "위치 도달" 규칙을 그대로 쓰면 안 됨
//   (그립 확인은 위치 · 전류 · 정지 시간 · 카메라를 묶은 별도 규칙 — TODO)
// ============================================================================
class FoldCheck
{
public:
  // home: 관절 순서대로 접은 자세 [rad], tol: 관절마다 허용 오차 [rad] (하나만 주면 모두 같은 값)
  FoldCheck(std::vector<double> home, std::vector<double> tol, double hold_s, double max_age_s);

  void reset();   // 지금까지 본 것 버리고 처음부터 (VERIFY 시작 때)
  // stamp_s: 측정 시각 (/joint_states header.stamp), positions: home 과 같은 순서 (없으면 NaN)
  void observe(double stamp_s, const std::vector<double> & positions);
  bool confirmed(double now_s) const;
  std::string status(double now_s) const;   // 아직 아니면 왜 아닌지 (로그용)

private:
  std::vector<double> home_, tol_;
  double hold_s_, max_age_s_;
  bool have_{false};
  double last_stamp_{0.0};
  double inside_since_{-1.0};   // 지금 연속으로 home 범위 안에 들어온 첫 측정 시각 (음수 = 지금 범위 밖 · 모름)
  std::string last_problem_{"측정 없음"};
};

// ============================================================================
// ⑤ 하드웨어 측정 살아 있나 (HardwareWatch) — 2026-10-16 재검증 R6
// 고장(하드웨어 ERROR · 펌웨어 잠금 · 상태 끊김) 뒤에는 "예전에 HOME 이 확인됐다" 가 더는 사실이 아님
//   (CONFIG 가 토크를 껐다 켜면서 팔이 처졌을 수도, 고장 중 팔이 다른 자세에서 멈췄을 수도 있음)
//   → control_master 가 이걸로 이상을 보면 STOP + 진행 중 팔 작업 실패 + 새 팔 작업 거절. 복구 뒤 HOME 이 다시 성공해야 DRIVE
// 판단 — 둘 중 하나라도 이상이면 이상 (2026-10-16 보드 2장: 주행 보드만 죽고 팔은 멀쩡한 경우도 잡으려고)
//   ① 보드 상태 직접 (component_timeout_s > 0 일 때): controller_manager 가 알려 준 하드웨어(보드)가 전부 active 인가
//      답이 component_timeout_s 넘게 없으면 "모름" = 이상 (controller_manager 가 멈춤 · 끊김)
//   ② 관절 측정 (/joint_states, 감시할 관절 = control.yaml hw_watch_joints, 기본 14 개 전부):
//      - state_timeout_s 넘게 안 옴 (joint_state_broadcaster 가 멈춤 · 노드 끊김), 또는
//      - NaN(못 읽음) · 빠진 관절이 bad_hold_s 넘게 이어짐 (opencr_hardware 는 못 읽은 모터 · ERROR 뒤 모든 칸을 NaN 으로)
//   ①만으로는 "보드는 active 인데 모터 몇 개를 못 읽는 중" 을, ②만으로는 "broadcaster 가 옛 값을 계속 내보냄" 같은
//   경우를 놓칠 수 있어서 둘 다. 책상 시험에서 일부 모터만 쓰면 hw_watch_joints 를 그 관절로 줄일 것
// ============================================================================
class HardwareWatch
{
public:
  // component_timeout_s <= 0 이면 ① 보드 상태는 보지 않음 (관절 측정만 — 단위 시험 · 보드 상태를 못 묻는 환경)
  HardwareWatch(double state_timeout_s, double bad_hold_s, double component_timeout_s = 0.0)
  : state_timeout_s_(state_timeout_s), bad_hold_s_(bad_hold_s), component_timeout_s_(component_timeout_s) {}
  // stamp_s: 측정 시각 (header.stamp), all_measured: 볼 관절이 전부 있고 NaN 이 아님
  void observe(double stamp_s, bool all_measured);
  // controller_manager 의 답 (받은 시각, 전부 active 인가, 아니면 어느 보드가 무슨 상태인지)
  void observe_components(double now_s, bool all_active, const std::string & detail);
  bool healthy(double now_s) const;
  std::string status(double now_s) const;   // 로그용
private:
  double state_timeout_s_, bad_hold_s_, component_timeout_s_;
  bool have_{false};
  double last_stamp_{0.0};
  double bad_since_{-1.0};   // NaN 이 이어지기 시작한 측정 시각 (음수 = 지금 정상)
  bool comp_have_{false}, comp_ok_{false};
  double comp_stamp_{0.0};
  std::string comp_detail_{"보드 상태를 아직 못 받음"};
  bool components_ok(double now_s) const;
};

}  // namespace manipulation
