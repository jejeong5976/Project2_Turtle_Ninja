// arm.hpp — 팔 쪽 순수 계산: 동작 순서표 · 차례 정하기 · 팔 작업 진행
// ROS 를 모르는 순수 C++ → test/test_arm.cpp. 쓰는 곳: control_master.cpp (ROS 입출력만 함)
//   ① make_arm_plan  : HOME · PICK · PLACE → 동작 순서표
//   ② MotionArbiter  : 차례 DRIVE · STOPPING · ARM · STOP (D15 한 번에 한 무리)
//   ③ ArmRunner      : 팔 작업 하나를 보내고 · 기다리고 · 취소하고 · 끝내는 상태 기계 (바퀴를 풀어도 되나를 정함)
//   ④ FoldCheck      : 접힘을 /joint_states 실측으로 확인
//   ⑤ HardwareWatch  : 하드웨어 측정이 살아 있나 (끊김 · NaN 지속 → HOME 확인 무효, STOP)
//   ⑥ HealthHold     : ⑤ 의 잠깐 끊김 · 늦음은 넘김 (진짜 고장은 바로)

#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace manipulation
{

// ============================================================================
// ① 팔 동작 순서표
// planning 은 무엇을 할지만 말하고 (ArmTask: HOME · PICK · PLACE), 순서는 여기서 정한다.
// 동작 하나 = 팔 자세 또는 그리퍼 중 하나 (동시에 안 움직임 — D15). 자세 각도는 config/control.yaml poses.*
// ============================================================================

// 값은 interfaces/action/ArmTask 와 같게
constexpr uint8_t kArmHome = 0;
constexpr uint8_t kArmPick = 1;
constexpr uint8_t kArmPlace = 2;

// 시간표 점 하나 (궤적 시작부터 t [s] 에 관절값 q [rad])
struct TrajPoint
{
  std::vector<double> q;
  double t;
};

struct ArmAction
{
  std::string phase;     // 진행 상황 이름 (ArmTask feedback): approach · grasp · lift · release · retreat
  bool is_gripper;       // false = 팔 자세로, true = 그리퍼만
  std::string pose;      // 팔: 자세 이름. 경로 동작은 마지막 점 이름 (받침이면 "home")
  double gripper;        // 그리퍼: 손가락 위치 [m]
  std::vector<TrajPoint> points{};   // 비어 있지 않으면 이 시간표 그대로 (arm_path.hpp). 비면 pose 하나를 move_time 에
  double duration_s{0.0};            // points 의 마지막 시각 (시간 제한 계산)
  double side_grip_t{-1.0};          // >= 0: 팔 동작을 멈추지 않고 시작 뒤 이 시각 [s] 에 그리퍼만 side_grip 으로 (세우기)
  double side_grip{0.0};
};

// 모르는 작업 번호면 빈 목록. 마지막 동작은 항상 접기(home) → 끝나면 바로 주행해도 안전
std::vector<ArmAction> make_arm_plan(uint8_t task, double gripper_open, double gripper_close);

// 쥐기 뒤 빈손 판단: 그리퍼 실측 grip_now_m 이 empty_m 보다 더 닫혔으면 true
//   실측 · 문턱이 NaN 이면 false (판단 안 함). empty_m = −1 이면 끔
bool grasp_empty(double grip_now_m, double empty_m);

// 받침에 놓기 직전 (기울여 쥔 선 퍽 pk_down 끝의 열기 앞) 그리퍼 실측:
//   kEmpty = 놓기 전에 이미 빈손 (퍽은 받침에 안 감), kHeld = 쥔 채 놓음, kUnknown = 실측 · 문턱 없음 · 그 흐름 아님
enum class ParkRelease {kUnknown, kHeld, kEmpty};
ParkRelease park_release_state(double grip_now_m, double empty_m);
// 받침 안 다시 쥐기 (park_re_down) 가 빈손일 때 받침 위에서 접어도 되나: kEmpty 일 때만 true.
//   그 밖은 퍽이 받침 (HOME 자리 — 접은 손끝이 받침 바닥 위 약 9 mm) 에 남았을 수 있음 → 접지 않음 (사람 확인 · HOME force)
bool park_empty_fold_ok(ParkRelease r);

// ============================================================================
// ② 차례 정하기 (MotionArbiter)
// D15: 순간 전류를 줄이려고 **한 번에 한 무리만** 움직인다.
//   DRIVE    : 바퀴 · 조향만. 팔은 접은 자세로 버티기 (기본)
//   STOPPING : 팔 작업이 들어옴 → 바퀴를 세우는 중. 몸통이 settle_s 동안 멈춰 있어야 ARM 으로
//   ARM      : 팔 · 그리퍼만. 바퀴는 0 으로 버티기
//   STOP     : 켤 때 · 재시작 · 팔 작업 실패 · 취소 → 팔 자세를 모르니 아무것도 안 움직임
//              ("접기로 끝나는" 팔 작업이 끝까지 성공해야 풀림 — planning INIT 이 맨 먼저 HOME 을 보냄)
// /control/motion_mode 로 방송 → swerve_controller 는 DRIVE 일 때만 바퀴를 움직임 (D8).
// ARM 조건은 "바퀴 쪽이 이 번호로 잠갔다고 답함 + /odom 으로 settle_s 동안 멈춤" (방송이 안 닿으면 팔도 시작 안 함)
// ============================================================================

// 값은 interfaces/msg/MotionMode 와 같게 (상태 방송용)
enum class Turn : uint8_t { STOP = 0, DRIVE = 1, ARM = 2, STOPPING = 3 };

const char * turn_name(Turn t);

class MotionArbiter
{
public:
  // start_locked: 켤 때 STOP 으로 시작. HOME 작업이 끝나야 DRIVE (팔 동작 중 재시작돼도 바퀴가 풀리지 않음)
  explicit MotionArbiter(double settle_s, bool start_locked = true)
  : settle_s_(settle_s), turn_(start_locked ? Turn::STOP : Turn::DRIVE) {}

  // 팔 작업이 들어옴 → 바퀴부터 세우기 시작 (새 요청 번호)
  void request_arm();
  // 팔 작업이 끝남. drive_ok = 끝까지 성공 + 마지막 동작이 접기(home) (ArmRunner 가 정함)
  //   true → DRIVE, false → STOP (취소 · 실패 · 시간 초과. 다음 HOME 이 성공해야 풀림)
  // DRIVE 로 가는 길은 이것 하나뿐
  void arm_ended(bool drive_ok);
  // 하드웨어 이상 → 무조건 STOP. 그동안의 HOME 확인은 무효 → 다음 HOME 이 성공해야 DRIVE
  void force_stop() {set(Turn::STOP);}
  // Continuous permit, not just a one-time HOME check. A fault latches STOP;
  // fresh measurements alone cannot restore DRIVE (a successful HOME must).
  bool enforce_drive_guard(bool folded_fresh, bool hardware_healthy);
  // 매 주기 호출. STOPPING → ARM 조건:
  //   ① 바퀴 컨트롤러가 **이번 요청 번호**의 잠금을 답함 (acked_id == request_id(), 옛 번호의 답은 무시)
  //   ② 그 뒤 몸통이 settle_s 동안 계속 멈춤 (base_still — 측정 시각이 새로운 /odom 으로)
  //   STOP 에서도 호출: 잠금 답 + 멈춤이 이어진 시각을 셈 → 이어 온 팔 작업은 ② 를 그 시각부터 (① 은 새 번호로)
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
  // STOP 에서 이미 잠금 답 + 멈춤이 이어진 시각. 잇단 팔 작업이 settle_s 를 처음부터 다시 안 기다림 (한 번이라도 움직이면 버림)
  double carry_since_{-1.0};
};

// ============================================================================
// 주기 멈춤 유예 (control_master stall_grace_s): 주기 틈이 thr [s] 넘으면 grace [s] 동안 접힘 · 하드웨어 판정 이전 값.
//   유예가 처음 시작한 뒤 max_s 넘으면 더 안 늘림 (계속 굶는 노드는 판정이 돌아옴)
// ============================================================================
class StallGrace
{
public:
  // 주기마다 (step 시작). 이번 주기에 틈을 봤으면 true (로그용)
  bool on_step(double now_s, double thr, double grace, double max_s = 2.0);
  bool active(double now_s) const {return now_s < until_;}
private:
  double last_{-1.0}, until_{-1.0}, start_{-1.0};
};

// ============================================================================
// ③ 팔 작업 진행 (ArmRunner)
// "언제 바퀴 잠금을 풀어도 되나" 는 안전에 가장 중요한 판단이라 노드 밖 순수 C++ (모든 경우를 test_arm.cpp 로 시험)
//
// 쓰는 쪽(control_master)은 ROS 입출력만:
//   ① 사건을 알려 줌: start · request_cancel · on_goal_response · on_result (재생기 답)
//   ② 주기마다 tick(지금 시각, 팔 차례인가) → 돌려받은 ArmCommand 를 그대로 실행
//        SEND          : 이 동작을 재생기(JTC)에 보냄 (gen 번호를 같이 기억)
//        CANCEL_ACTIVE : 지금 궤적(gen)에 취소 요청 (받아들여지기 전이면 전체 취소)
//        FINISH        : 상위(ArmTask)에 결과를 답하고, drive_ok 로 차례를 정함
//
// ★ drive_ok 는 **끝까지 성공 + 마지막 동작이 접기(home) + 실측으로 접힘 확인** 일 때만 true.
//   JTC 성공 답만으로는 부족 (목표 오차 검사가 꺼져 있으면 어깨가 걸려도 성공) → 마지막 home 뒤 VERIFY 단계에서
//   신선한 관절 측정이 home 범위 안에 hold_s 동안 머무는지 (FoldCheck) 본 뒤에만 DRIVE. verify_timeout_s 넘으면 ABORTED → STOP.
//   취소 · 실패 · 시간 초과는 팔을 움직였든 안 움직였든 언제나 false (궤적을 안 보냈다고 팔이 접혀 있는 건 아님 —
//   켤 때 · 재시작 직후는 자세를 모르고, 죽은 control_master 가 보낸 궤적이 아직 돌 수도 있음)
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
  bool drive_ok{false};     // FINISH: 바퀴 차례로 돌아가도 되나
  std::string message;      // FINISH: 상위에 돌려줄 설명
};

// 재생기에 늦게 온 "받아들였음" 답을 어떻게 할지
enum class LateGoal : uint8_t { USE, IGNORE, CANCEL_IT };

class ArmRunner
{
public:
  // accept_timeout_s: 재생기가 "받았다" 고 답할 때까지
  // motion_timeout_s: 받은 뒤 끝날 때까지 (move_time + 여유). 시간표가 있는 동작은 + duration_s
  // cancel_timeout_s: 취소를 보낸 뒤 결과 답을 기다리는 최대 시간
  // verify_timeout_s: 마지막 home 동작 뒤 실측 접힘 확인을 기다리는 최대 시간
  ArmRunner(double accept_timeout_s, double motion_timeout_s, double cancel_timeout_s, double verify_timeout_s = 2.0)
  : accept_timeout_s_(accept_timeout_s), motion_timeout_s_(motion_timeout_s), cancel_timeout_s_(cancel_timeout_s),
    verify_timeout_s_(verify_timeout_s) {}

  // 새 작업 시작 (빈 계획은 다음 tick 에서 바로 실패로 끝남)
  void start(std::vector<ArmAction> plan);
  bool active() const {return active_;}

  // 상위(ArmTask)가 취소를 요청함. 여러 번 불러도 같음
  void request_cancel() {if (active_) {cancel_requested_ = true;}}
  // 지금 작업을 실패로 끝냄 (다음 tick 에 진행 중 궤적 취소 + ABORTED, drive_ok = false)
  //   취소와 달리 재생기의 "멈췄음" 답을 기다리지 않음 (하드웨어가 멈춰 답이 안 올 수 있음)
  void fail_now(const std::string & why) {if (active_) {forced_ = true; forced_why_ = why;}}

  // 재생기의 "받았다/거절" 답. 이미 포기한 세대의 답이면 IGNORE / CANCEL_IT (받아들여졌으면 그것만 골라 취소)
  LateGoal on_goal_response(unsigned gen, bool accepted, double now_s);
  // 재생기의 결과 (ok = 성공). 포기한 세대면 무시
  void on_result(unsigned gen, bool ok);
  // 논리 대기 (모터 궤적 없음, regrasp_visual) 를 끝내며 남은 계획을 tail 로 바꿈. 받아들인 지금 세대의 대기일 때만,
  //   취소 · 실패 · fail_now 상태면 안 바꿈 (false)
  bool replace_after_wait(unsigned gen, std::vector<ArmAction> tail);
  // 방금 SEND 로 내준 동작 앞에 pre 를 끼움 (grasp_guard 다시 쥐기 · 다시 넣기): 호출한 쪽은 원래 동작 대신 pre[0] 을 보냄,
  //   원래 동작은 pre[1..] 뒤. SEND 직후 (결과 전) 가 아니면 false
  bool insert_before_sent(std::vector<ArmAction> pre);
  // 방금 SEND 로 내준 동작부터 끝까지를 tail 로 바꿈 (store_recover) → 호출한 쪽은 tail[0] 을 보냄.
  //   SEND 직후 (받아들여지기 전) 가 아니거나 tail 이 비면 false
  bool replace_from_sent(std::vector<ArmAction> tail);

  // 받아 주기 기한을 실제로 보낸 시각 now_s 로 다시 잼 (tick 과 보내기 사이에 계획이 끼는 경우, 늦추기만).
  //   지금 세대 · 받아들여지기 전일 때만. 아니면 false
  bool mark_sent(unsigned gen, double now_s);
  // tick 사이가 stall_s [s] 넘게 벌어지면 받아 주기 · 끝나기 · 취소 · 접힘 확인 기한을 그 틈만큼 뒤로 (늦추기만). 0 이하 = 끔
  //   max_total_s: 한 동작마다 늘린 합의 상한 (계속 굶는 노드도 끝내 시간 초과). 0 이하 = 상한 없음
  void set_stall(double stall_s, double max_total_s = 5.0) {stall_s_ = stall_s; stall_cap_s_ = max_total_s;}
  double last_stall_s() const {return last_stall_;}   // 마지막으로 늦춘 틈 [s] (로그용)
  int stalls() const {return stalls_;}
  // 주기마다. arm_may_move = 지금 팔 차례(ARM)인가
  //   fold_confirmed = FoldCheck::confirmed(now) — VERIFY 단계에서만 봄 (VERIFY_START 이후의 측정만 인정)
  std::vector<ArmCommand> tick(double now_s, bool arm_may_move, bool fold_confirmed = false);

  // 구분 동작: 켜면 첫 동작 뒤로는 동작마다 step_go() 를 기다림. start() 뒤에 켬, 작업이 끝나면 꺼짐
  void set_step(bool on) {step_mode_ = on; step_ok_ = false; waiting_step_ = false;}
  void step_go() {step_ok_ = true;}
  bool waiting_step() const {return waiting_step_;}
  // 다음에 보낼 동작 이름 (구분 동작 로그용)
  std::string next_name() const {return idx_ < plan_.size() ? (plan_[idx_].is_gripper ? "그리퍼" : plan_[idx_].pose) : "";}

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
  bool step_mode_{false}, step_ok_{false}, waiting_step_{false};
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
  double stall_s_{0.0};       // set_stall (0 = 끔)
  double stall_cap_s_{5.0};   // 한 동작에서 늘린 합 상한
  double stall_total_{0.0};   // 지금 동작에서 늘린 합
  double last_tick_s_{-1.0};  // 마지막 tick 시각 (음수 = 아직)
  double last_stall_{0.0};
  int stalls_{0};
};

// 실패 뒤 자동 접기 (control_master fail_fold_s). 뻗은 채 오래 두면 관절 2 과부하로 팔 보드 전체가 멈춤
//   fail_fold_arm: 시계를 켤지 (팔 작업 끝): ABORTED · HOME 아님 · fail_fold_s > 0 · 하드웨어 정상일 때만 (취소 · 성공은 아님)
//   fail_fold_block: 접기 직전 검사. 빈 문자열 = 접음, 아니면 안 하는 이유 (로그)
bool fail_fold_arm(ArmEnd end, bool home_task, double fail_fold_s, bool hw_ok);
struct FailFoldState
{
  bool folded{false};        // 이미 접힘 실측 확인
  bool carrying{false};      // 퍽을 쥔 채 (PICK · RETRIEVE 성공 뒤, STORE · PLACE 실패 뒤)
  bool half_closed{false};   // 그리퍼 실측이 반쯤 닫힘 (퍽을 쥐었을 수 있음)
  bool empty_grasp{false};   // 이번 작업에서 빈손 확인이 걸림 (손이 빈 것이 확실)
  bool step_mode{false};     // 구분 동작 (park_step) 작업이었음 — 사람이 보는 중
  bool park_left{false};     // 받침 안 다시 쥐기 (park_ 닫기) 가 빈손 — 퍽이 받침 (HOME 자리) 에 남았을 수 있음
};
std::string fail_fold_block(const FailFoldState & s);

// 보기 자세 자동 접기 (펴 둔 채 오래 두면 관절 2 과부하): 성공한 작업 뒤 look_at_idle_fold_s 시계를 켤지
//   LOOK_AT · 빈손 LOOK (적재함 스캔 rack · 넣은 뒤 보기 seat_ 빼고) · 빈손 LOOK_BARCODE.
//   퍽을 쥔 채 · P0 에서 끝난 작업 (lazy_fold_idle_s 가 맡음) 은 아님
enum class IdleLookTask { kNone, kLook, kLookBarcode, kLookAt };
bool look_idle_fold_arm(IdleLookTask task, const std::string & slot, bool carrying, bool ends_p0);
// 보기 자세 접기 시계: since = 쉬기 시작 (−1 = 꺼짐), out_since = 보기 작업이 잇달아 성공해 팔이 펴진 채인 시작 (−1 = 아님).
//   보기 요청이 계속 와도 cap_s 넘으면 다음 쉬는 틈에 접음 (접는 동안 ready 꺼짐). 접기를 못 하면 retry_s 뒤 다시
struct LookIdleClock
{
  double since{-1.0};
  double out_since{-1.0};
  // 팔 작업 끝: armed = 성공 · look_idle_fold_arm. 아니면 둘 다 끔
  void on_task_end(bool armed, double now, double idle_s, double cap_s);
  void on_task_start() {since = -1.0;}   // 새 팔 작업 (펴진 채 시작 시각은 둠 — 다음 끝에서 이어 셈)
  bool due(double now, double idle_s) const {return since >= 0.0 && idle_s > 0.0 && now - since >= idle_s;}
  void retry(double now, double idle_s, double retry_s = 1.0) {since = now - idle_s + retry_s;}   // 이번 시도 못 함 → retry_s 뒤 다시
  void folded() {since = -1.0; out_since = -1.0;}   // 스스로 접기 시작
};

// ============================================================================
// ④ 접힘 실측 확인 (FoldCheck)
// "팔이 HOME 에 있다" 를 재생기의 성공 답이 아니라 **관절 측정값**으로 확인한다.
//   - 관절이 빠졌거나 NaN 이면 그 묶음은 "모름" → 처음부터 다시 (하드웨어는 못 읽은 모터를 NaN 으로 냄)
//   - 측정 시각이 앞으로 가지 않은 묶음은 무시
//   - 측정 사이가 max_age_s 넘게 벌어지면 유지 시간을 처음부터
//   - confirmed(now): 마지막 측정이 max_age_s 안이고, 모든 관절이 |값 − home| ≤ tol 인 상태가 hold_s 이상 이어짐
// 그리퍼는 보지 않음 (물체를 쥐면 닫힘 목표에 못 미치는 게 정상)
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
  double inside_since_{-1.0};   // 지금 연속으로 home 범위 안에 들어온 첫 측정 시각 (음수 = 범위 밖 · 모름)
  std::string last_problem_{"측정 없음"};
};

// ============================================================================
// ⑤ 하드웨어 측정 살아 있나 (HardwareWatch)
// 고장 뒤에는 "예전에 HOME 이 확인됐다" 가 더는 사실이 아님 → 이상이면 STOP + 진행 중 팔 작업 실패 + 새 팔 작업 거절.
//   복구 뒤 HOME 이 다시 성공해야 DRIVE
// 둘 중 하나라도 이상이면 이상 (보드 2장: 한 보드만 죽은 경우도 잡으려고):
//   ① 보드 상태 (component_timeout_s > 0 일 때): controller_manager 가 알려 준 하드웨어가 전부 active 인가.
//      답이 component_timeout_s 넘게 없으면 이상
//   ② 관절 측정 (/joint_states, control.yaml hw_watch_joints): state_timeout_s 넘게 안 옴, 또는
//      NaN · 빠진 관절이 bad_hold_s 넘게 이어짐 (opencr_hardware 는 못 읽은 모터 · ERROR 뒤 모든 칸을 NaN 으로)
//   책상 시험에서 일부 모터만 쓰면 hw_watch_joints 를 그 관절로 줄일 것
// ============================================================================
class HardwareWatch
{
public:
  // component_timeout_s <= 0 이면 ① 보드 상태는 보지 않음 (관절 측정만)
  HardwareWatch(double state_timeout_s, double bad_hold_s, double component_timeout_s = 0.0)
  : state_timeout_s_(state_timeout_s), bad_hold_s_(bad_hold_s), component_timeout_s_(component_timeout_s) {}
  // stamp_s: 측정 시각 (header.stamp), all_measured: 볼 관절이 전부 있고 NaN 이 아님
  void observe(double stamp_s, bool all_measured);
  // controller_manager 의 답 (받은 시각, 전부 active 인가, 아니면 어느 보드가 무슨 상태인지)
  void observe_components(double now_s, bool all_active, const std::string & detail);
  bool healthy(double now_s) const;
  std::string status(double now_s) const;   // 로그용
  // 진짜 고장 (HealthHold 가 기다리지 않는 것): 받은 보드 상태 답에 active 아닌 보드가 있음,
  //   또는 관절 측정 NaN · 빠짐이 bad_hold_s 넘게. 끊김 · 답 늦음 · 시각 앞섬은 아님
  bool fault() const;
  // 보드 상태 없이 (hw_status_optional — DDS 로 list_hardware_components 를 못 찾는 경우): 보드 상태를 한 번도 못 받았거나,
  //   서비스가 지금 안 보이고 마지막 답이 정상이면 ② 관절 측정 + 팔 · 그리퍼 JTC 준비로만 판단. 받은 답이 이상이면 이상
  void set_components_optional(bool on) {comp_optional_ = on;}
  void set_components_reachable(bool on) {comp_reachable_ = on;}   // 서비스가 지금 보이나
  void set_controllers_ready(bool on) {ctrl_ready_ = on;}          // 팔 · 그리퍼 재생기 액션 서버 준비
  bool components_waived(double now_s) const;                      // 지금 보드 상태 대신 관절 측정으로 보는 중
  // 측정 · 답 시각이 판정 시각보다 s [s] 까지 앞서도 신선한 것으로 봄 (다른 스레드가 판정 뒤 더 새 측정을 넣는 경우). 0 = 앞섬 허용 없음
  void set_future_tolerance(double s) {future_tol_s_ = std::isfinite(s) && s > 0.0 ? s : 0.0;}
private:
  double future_tol_s_{0.0};
  double state_timeout_s_, bad_hold_s_, component_timeout_s_;
  bool have_{false};
  double last_stamp_{0.0};
  double bad_since_{-1.0};   // NaN 이 이어지기 시작한 측정 시각 (음수 = 지금 정상)
  bool comp_have_{false}, comp_ok_{false};
  double comp_stamp_{0.0};
  std::string comp_detail_{"보드 상태를 아직 못 받음"};
  bool comp_optional_{false}, comp_reachable_{true}, ctrl_ready_{false};
  bool components_ok(double now_s) const;
};

// ============================================================================
// ⑥ 하드웨어 이상 판정 늦추기 (HealthHold, control_master hw_unhealthy_hold_s)
// 측정 · 보드 상태 답은 다른 스레드 (sense_group_) 가 받음 → 시각 앞섬 · 스레드 굶음으로 순간 이상이 생김.
//   끊김 · 늦음 (진짜 고장 아님) 은 hold_s 동안 이어져야 이상. 진짜 고장 (HardwareWatch::fault) 은 바로 이상
//   한 번도 정상이 아니었으면 (켤 때) 늦추지 않음. hold_s ≤ 0 = 늦추지 않음 (healthy 만)
// ============================================================================
class HealthHold
{
public:
  // 돌려줌 = 하드웨어 정상으로 볼까 (control_master step 의 hw_ok)
  bool update(double now_s, bool healthy, bool fault, double hold_s);
  bool holding() const {return bad_since_ >= 0.0 && ok_;}   // 지금 이상인데 늦추는 중 (마지막 update 기준)
private:
  bool ever_ok_{false};
  bool ok_{false};
  double bad_since_{-1.0};   // 이상이 이어지기 시작한 시각 (음수 = 지금 정상)
};

}  // namespace manipulation
