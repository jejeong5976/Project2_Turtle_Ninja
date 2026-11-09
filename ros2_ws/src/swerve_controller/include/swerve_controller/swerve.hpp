// swerve.hpp — 바퀴(스워브) 쪽 순수 계산 전부: 기구학 · 자이로 · "운전석"
// =====================================================================
// ROS 를 전혀 모르는 순수 C++ (rclcpp 없음) → 로봇 없이 단위 테스트 (test/test_swerve.cpp)
// 쓰는 곳: swerve_controller.cpp (ros2_control 컨트롤러, 100 Hz) 하나. 컨트롤러는 ROS 와 이어 붙이기만 함
//
//   ① SwerveKinematics : (vx, vy, ω) ↔ 모듈 4개 (조향 각도, 바퀴 속도). 뒤집기 · 조향 끝 · offset   (D14 · D19)
//   ② GyroYaw          : OpenCR 자이로로 회전 속도 (바이어스 빼기)                                 (D12)
//   ③ SwerveDrive      : 한 주기 = 명령 검사 · 끊김 · 차례 · 가속 제한 · 조향 먼저 · odom 적분       (D8 · D15 · D16)
//
// 좌표 약속 (ROS 표준 REP-103): x = 앞, y = 왼쪽, ω = 위에서 볼 때 반시계(+). 단위: m, m/s, rad, rad/s

#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace swerve_controller
{

// ============================================================================
// ① 스워브 기구학
//   차동 2륜은 (v, ω) → 왼쪽·오른쪽 바퀴 속도 2개지만,
//   스워브 4개는 (vx, vy, ω) → 모듈 4개 각각 (조향 각도, 바퀴 속도). 옆으로도 갈 수 있음
// ============================================================================

// 모듈 하나가 몸통(base_link) 어디에 달려 있나 [m]
//   x, y   : 조향축 위치
//   offset : 바퀴 중심이 조향축에서 "바퀴축 방향(모듈의 y)" 으로 떨어진 거리. 0 이면 바퀴가 조향축 바로 아래.
//            CAD 에서는 바퀴가 바깥쪽으로 26.9 mm → 왼쪽 모듈 +0.0269, 오른쪽 모듈 -0.0269 (조향 0° 기준)
//
//   왜 필요한가: 바퀴가 조향축 옆에 있으면, 몸이 돌거나(ω) 조향이 돌 때 바퀴가 조향축 둘레를 "굴러서" 따라가야 함.
//     그 움직임은 언제나 바퀴가 구르는 방향이라(옆으로 미끄러지는 게 아님) 바퀴 각도 계산은 그대로이고,
//     바퀴 속도만  (조향축 속도) − ω × offset  으로 고치면 됨.
//     예) 제자리 회전: 조향축은 중심에서 0.129 m 인데 바퀴는 0.102 ~ 0.156 m 원을 돎 (뒤집기 방향에 따라) → 무시하면 ±21 % 틀림
struct ModuleGeometry
{
  double x;
  double y;
  double offset = 0.0;
};

// 몸통 전체의 속도 (cmd_vel 과 같은 것)
struct BodyTwist
{
  double vx;   // 앞 [m/s]
  double vy;   // 왼쪽 [m/s]
  double wz;   // 회전 [rad/s]
};

// 모듈 하나의 상태 또는 명령
struct ModuleState
{
  double steer_angle;        // 조향 각도 [rad], 0 = 바퀴가 앞을 봄
  double wheel_speed;        // 바퀴 회전 속도 [rad/s] (몸 속도가 아니라 바퀴 각속도!)
  double steer_rate = 0.0;   // 조향이 도는 속도 [rad/s] (측정값. 명령에서는 안 씀)
};

// 모듈별 연속 허용 구간. 물리 조향 좌표(rad), CAD/모터 원시 좌표가 아님.
// 실제 간섭을 확인한 값으로 설정해야 하며, 구간 사이의 장애물을 표현하지는 않는다.
struct SteeringRange
{
  double min;
  double max;
};

// -π ~ π 로 접기 (예: 190° → -170°)
double wrap_angle(double a);

// 가속 제한 (D16 "느리게, 부드럽게"): current 에서 target 으로 dt 동안 갈 수 있는 만큼만 감
//   선속도는 (vx, vy) 벡터 차이의 크기로, 회전은 따로 제한 → 방향이 바뀌어도 급하게 안 꺾임
//   모터 전류는 가속할 때 가장 크다 → 가속을 줄이면 순간 전류가 줄어듦
BodyTwist limit_accel(
  const BodyTwist & current, const BodyTwist & target,
  double max_lin_acc, double max_ang_acc, double dt);

// 조향 먼저, 바퀴 나중 (D15 "한 번에 한 무리"): 모든 모듈이 목표 각도에서 tol 안이면 true
//   각도는 "모터가 실제로 가는 각도" 로 비교 (접지 않음): 명령 179° · 지금 -179° 는 2° 가 아니라 358° 차이
//   — 위치 모드 조향 모터는 -180° ~ 180° 사이에서만 움직여서, 그 사이를 빙 돌아가야 하기 때문
bool steering_aligned(
  const std::vector<ModuleState> & command,
  const std::vector<ModuleState> & measured, double tol);

class SwerveKinematics
{
public:
  // steer_limit : 조향 모터가 갈 수 있는 끝 [rad]. 위치 모드 다이나믹셀 = 0~4095 눈금 = -π ~ +π
  //   (π/2 보다 작으면 뒤집기로도 못 가는 방향이 생겨서 π/2 로 올림)
  SwerveKinematics(std::vector<ModuleGeometry> modules, double wheel_radius,
    double steer_limit = 3.14159265358979323846,
    std::vector<SteeringRange> ranges = {}, bool coherent = false,
    double branch_margin = 3.0 * 3.14159265358979323846 / 180.0, double end_slack = 0.0,
    double max_range_width = 2.0 * 3.14159265358979323846);
  // end_slack: 모듈 범위 끝을 이만큼(최대 2°) 넘는 방향은 끝 각도로 보냄 (기본 0 = 끝에 맞추지 않음).
  // max_range_width: 모듈 범위 폭 상한 (기본 한 바퀴 2π = 위치 모드). 여러 바퀴 모드(확장 위치 모드)일 때만 최대 6π

  // 역기구학: 몸 속도 → 모듈별 명령
  //   current      : 지금 모듈 상태 (조향 각도만 씀). 크기가 0 이면 최적화 없이 계산
  //   (offset 이 있으면 조향이 도는 동안에도 바퀴가 굴러야 함 → 그건 roll_while_steering() 이 따로 계산)
  //   cosine_scale : 조향이 아직 덜 돌아갔으면 그만큼 바퀴를 천천히 (미끄러짐 줄이기)
  //
  // 핵심 요령 "뒤집기": 목표 각도가 지금보다 90° 넘게 멀면,
  //   반대 방향(+180°)을 보고 바퀴를 거꾸로 굴린다 → 조향이 90° 넘게 돌 일이 거의 없음
  // 끝 넘김 막기: 돌려주는 조향 각도는 -π~π 로 "접은 값" 이 아니라 모터가 실제로 갈 각도.
  //   예) 지금 170° 에서 -170°(= 190°) 쪽으로 → 접으면 -170° 인데, 모터는 -180°~180° 라서
  //       -170° 로 가려면 거꾸로 340° 를 빙 돎 (그동안 바퀴는 엉뚱한 방향으로 굴러감)
  //   → 끝(steer_limit)을 넘는 목표는 반대쪽(10°)을 보고 바퀴를 거꾸로. 이때만 최대 180° 돌 수 있음
  std::vector<ModuleState> inverse(
    const BodyTwist & cmd,
    const std::vector<ModuleState> & current,
    bool cosine_scale = true, bool * feasible = nullptr,
    const std::vector<ModuleState> & previous = {}, bool prefer_outward = true) const;
  // prefer_outward: coherent 에서 "바퀴 바깥쪽" 선호를 쓸지. SwerveDrive 는 몸이 병진 중이면 false 로 넘김
  //   (실물: 이동 중 방향 전환 때 fr·rl 이 70.3° 에서 180° 반대 분기로 바뀌어 매번 정지·재조향).
  // ranges가 있으면 그 구간이 공통 steer_limit보다 우선한다.
  // 불가능한 방향은 전체 바퀴 0 + 현재 조향 유지로 반환하고 feasible=false.
  bool has_module_ranges() const {return !ranges_.empty();}

  // travel_weight (기본 0 = 안 씀): coherent 자세 고르기 비용에 "가중치 × 지금 조향에서 돌아야 할 각도 [rad]" 를 더함.
  //   공통 방향 후보(네 바퀴 합)와 바퀴별 후보 둘 다. 0° 에 가까운 자세만 보던 기준이라 범위 끝에서 300° 넘게 도는 자세를 고르던 것
  //   (실물 · 조향 속도 모의: 정면 유지 전방향 오른쪽 → 앞오른쪽에서 fl 269.5° → −45°) 을 줄임. 0 ~ 10
  void set_travel_weight(double w);
  double travel_weight() const {return travel_weight_;}

  // 가지 고르기 보강 (둘 다 기본 0 = 안 씀). 실물 36 방향 왕복: 나갈 때 130~163° 뒤집힘 6 방향 · 출발 지연 최대 1.9 s
  //   coherent_max_travel [rad]: 공통 자세(네 바퀴 같은 방향)로 맞추려면 어느 바퀴든 이 각도보다 많이 돌아야 하면 그 공통 자세를 쓰지 않음
  //     → 바퀴별로 고름. 네 범위의 공통 구간이 약 171° 뿐이라 공통 자세끼리 옮기면 170° 가까이 도는 경우가 생김. 0 ~ π
  //   end_guard [rad]: 멈춘 상태에서 새로 자세를 고를 때(prefer_outward) 범위 끝에서 이 각도 안쪽 후보에 벌점 (끝에 가까울수록 큼, 최대 4π).
  //     coherent · nearest 둘 다 (모듈별 범위가 있을 때)
  //     → 끝에 붙은 바퀴가 작은 방향 보정에도 주행 중 반대 가지로 뒤집히는 것(실물 140°: rr −40°, 끝 −44.5°)을 출발 전에 피함. 0 ~ 45°
  void set_branch_options(double coherent_max_travel, double end_guard);
  double coherent_max_travel() const {return coherent_max_travel_;}
  double end_guard() const {return end_guard_;}

  // 조향만 도는 동안(바퀴 구동은 멈춘 단계) 바퀴가 따라 굴러야 하는 속도 [rad/s]
  //   바퀴가 조향축 옆 offset 에 있으면, 조향이 θ̇ 로 돌 때 바퀴 접지점이 굴림 방향으로 θ̇·offset 만큼 움직임
  //   → 바퀴를 −θ̇·offset / r 로 같이 굴려 주지 않으면 바닥에 끌림 (조향 모터 부담 ↑, 위치 틀어짐)
  double roll_while_steering(std::size_t i, double steer_rate) const;
  // 바깥쪽 자세(모든 조향 0°)가 모듈별 범위 안인가 (범위가 없으면 true)
  bool park_reachable() const;

  // 순기구학: 모듈 상태 → 몸 속도 (odom 계산용)
  //   모듈이 4개라 식이 8개, 모르는 것은 3개 → 최소제곱으로 "가장 그럴듯한" 값
  BodyTwist forward(const std::vector<ModuleState> & states) const;

  std::size_t size() const {return modules_.size();}
  const std::vector<ModuleGeometry> & modules() const {return modules_;}
  double wheel_radius() const {return wheel_radius_;}

private:
  std::vector<ModuleGeometry> modules_;
  double wheel_radius_;
  double steer_limit_;
  std::vector<SteeringRange> ranges_;
  // coherent: shared physical target for translation when feasible; otherwise nearest zero.
  // Previous commanded targets provide branch hysteresis, never measured mixed poses.
  bool coherent_;
  double branch_margin_;
  double end_slack_;
  double travel_weight_ = 0.0;
  double coherent_max_travel_ = 0.0;
  double end_guard_ = 0.0;
};

// ============================================================================
// ② 자이로 → 회전 속도 (gyro)
// 왜: 바퀴로만 계산한 회전(ω)은 바퀴가 미끄러지면 틀린다 (제자리 회전에서 특히 — 몇 % 만 틀려도
//     odom 의 yaw 가 점점 틀어지고, 그러면 지도도 틀어짐).
//     자이로는 미끄러짐과 상관없이 몸이 실제로 돈 속도를 잼 → odom 의 회전만 자이로로 바꿔 씀
//     (위치(vx, vy)는 바퀴, 회전(ω)은 IMU)
//
// 자이로의 약점 = 바이어스(가만히 있어도 0 이 아닌 값) → 멈춰 있을 때의 평균을 빼 줌
//   ① 켠 직후 bias_time_s 동안 멈춰 있으면 평균으로 바이어스를 잡음 (그전엔 ready() = false → 바퀴 ω 사용)
//   ② 그 뒤에도 멈춰 있을 때마다 아주 천천히 바이어스를 따라감 (온도 변화 대응 — 실물: 1 시간에 −0.023 → −0.081 °/s)
//      단 멈춘 지 still_hold_s 가 지나고, 바이어스 뺀 값이 track_gate 안일 때만 (실물: 멈춘 직후 몸이 아직 살짝 도는 것까지
//      바이어스로 배워 직진 1 m 마다 odom 방향 약 1° 틀어짐 — 경로 339 s 동안 SLAM 이 28° 고침)
//   ③ IMU 가 timeout_s 넘게 안 오면 ready() = false → 바퀴 ω 로 자동 복귀
//      (OpenCR 플러그인은 IMU 가 고장이면 NaN 을 보냄 → 그런 값은 "안 온 것" 으로 침)
//
// 순수 C++ → test/test_swerve.cpp
// ============================================================================

class GyroYaw
{
public:
  GyroYaw(
    double bias_time_s = 2.0, double timeout_s = 0.2, double track_gain = 0.002,
    double still_hold_s = 1.0, double track_gate = 0.0087)
  : bias_time_s_(bias_time_s), timeout_s_(timeout_s), gain_(track_gain),
    hold_s_(still_hold_s), gate_(track_gate) {}

  // 자이로 z 한 개 [rad/s]. still = 지금 몸통이 멈춰 있나 (바퀴 기준)
  void add_sample(double now_s, double gz, bool still)
  {
    // 움직임은 값이 이상해도 먼저 반영 (튐을 버리며 움직임까지 버려 멈춤 1 s 대기 · 켤 때 연속 정지를 건너뜀)
    if (!still) {
      still_since_ = -1.0;
      if (!calibrated_) {start_t_ = -1.0; sum_ = 0.0; n_ = 0;}   // 움직이면 처음부터
    }
    if (!std::isfinite(gz)) {return;}   // NaN = IMU 없음 → 샘플로 안 침 (곧 ready() = false)
    if (std::fabs(gz) > kMaxRate) {return;}   // 순간 튐 (로봇은 1 rad/s 도 안 돎) → 버림, 계속 튀면 곧 ready() = false (바퀴 ω)
    if (!calibrated_ && n_ > 0 && now_s - last_t_ > timeout_s_) {start_t_ = -1.0; sum_ = 0.0; n_ = 0;}   // 켤 때 재는 중 긴 공백 = 다시
    last_t_ = now_s;
    last_gz_ = gz;
    if (still && still_since_ < 0.0) {still_since_ = now_s;}
    if (!calibrated_) {
      if (!still) {return;}
      if (start_t_ < 0.0) {start_t_ = now_s;}
      sum_ += gz;
      ++n_;
      if (now_s - start_t_ >= bias_time_s_ && n_ > 0) {bias_ = sum_ / n_; calibrated_ = true;}
      return;
    }
    if (still && now_s - still_since_ >= hold_s_ && std::fabs(gz - bias_) < gate_) {bias_ += gain_ * (gz - bias_);}
  }

  bool ready(double now_s) const {return calibrated_ && now_s - last_t_ <= timeout_s_;}
  double rate() const {return last_gz_ - bias_;}   // 바이어스 뺀 회전 속도 [rad/s]
  double bias() const {return bias_;}
  bool calibrated() const {return calibrated_;}

private:
  static constexpr double kMaxRate = 5.0;   // [rad/s]
  double bias_time_s_, timeout_s_, gain_, hold_s_, gate_;
  bool calibrated_{false};
  double still_since_{-1.0};
  double start_t_{-1.0}, sum_{0.0};
  long n_{0};
  double bias_{0.0}, last_t_{-1e9}, last_gz_{0.0};
};

// ============================================================================
// ③ 스워브 "운전석" (컨트롤러가 주기마다 부르는 본체)
// D8: 이 계산은 ros2_control **컨트롤러 안**(swerve_controller.cpp, 100 Hz 제어 루프)에서 돈다.
//   왜: 노드가 계산해서 토픽으로 내보내면, 노드가 멈췄을 때 받는 컨트롤러가 "마지막 속도" 를 붙잡고 계속 돌 수 있음.
//       제어 루프 안에서 직접 하면 /cmd_vel 이 cmd_timeout_s 넘게 안 올 때 **바로 그 루프 안에서** 바퀴 0
//
// 순수 C++ → test/test_swerve.cpp. 컨트롤러는 "지금 시각 · 측정값 · 받은 명령" 을 넣고, 나온 모듈 명령을 모터 칸에 쓰기만 함.
//
// 한 주기에 하는 일
//   ① 명령이 신선한가 (cmd_timeout_s) · 바퀴 차례인가 (drive_allowed) → 아니면 목표 0
//   ② 가속 제한 (멈출 때도 천천히)
//   ③ 역기구학 (뒤집기 · 조향 끝 · offset)
//   ④ 조향 먼저: 조향이 덜 돌았으면 구동은 멈추고 조향만 (바퀴는 offset 만큼만 따라 굴림)
// 그리고 측정값 → 순기구학 → odom 적분 (회전은 자이로가 있으면 자이로)
// ============================================================================

struct SwerveDriveConfig
{
  // Zero speed/acceleration caps mean no body-level shaping.
  double max_wheel_speed = 0.0;    // rad/s; motor Goal Velocity range, 0 = not configured
  double max_linear = 0.10;        // [m/s]  D16
  double max_angular = 0.5;        // [rad/s]
  double max_linear_accel = 0.2;   // [m/s²]
  double max_angular_accel = 0.8;  // [rad/s²]
  double cmd_timeout_s = 0.3;      // /cmd_vel 이 이만큼 안 오면 목표 0 (안전장치 ①)
  double turn_timeout_s = 0.5;     // 차례(/control/motion_mode)가 이만큼 안 오면 잠금 (모르면 안 움직임)
  bool cosine_scale = true;
  bool steer_then_drive = true;    // D15
  double steer_tolerance = 0.17;   // [rad]
  bool roll_while_steering = true; // D14
  // 정지 후 바깥쪽 자세(조향 0°) 복귀 — 기본 끔. 켜면: 신선한 0 명령 + 바퀴 차례 + 실측 바퀴 정지가 park_idle_s 동안
  //   끊김 없이 이어질 때만 네 조향을 0° 로 (바퀴는 roll_while_steering 보상만). 명령 끊김 · 상태 못 읽음 · NaN ·
  //   컨트롤러 끄기(종료 · 오류 · 비상정지)에서는 시작하지 않고, 복귀 중이면 그 자리에서 멈춤
  bool park_on_idle = false;
  double park_idle_s = 1.0;        // [s]
  double park_still_wheel = 0.05;  // [rad/s] 실측 바퀴 속도가 넷 다 이보다 작아야 "정지"
  // ---- 동시 조향·구동 (기본 끔) ----
  // 켜면: 움직이는 중 방향을 바꿀 때, 몸 명령을 "이전 명령 → 새 명령" 사이에서 조향이 이미 따라온 만큼만 진행시킨다.
  //   조건 = 모든 모듈의 (목표 조향 − 실측 조향) ≤ steer_tolerance × blend_margin. 가장 큰 진행 비율을 이분 탐색으로 찾음.
  //   바퀴 구동은 그 진행 비율의 몸 명령으로, 조향 모터에는 **새 명령(가속 제한 전)의 각도**를 바로 보냄
  //   (진행 비율 각도를 보내면 목표가 실측 6.8° 앞이라 조향이 제 속도를 못 내고 정체 판정에 걸림).
  //   ④ "조향 먼저" 게이트·가속 제한·명령 끊김·범위 검사는 그대로 → 조향이 못 따라오면 기존처럼 구동 0.
  //   정지 상태에서 출발할 때는 기존 동작(조향 먼저) 그대로.
  bool blend_steer_drive = false;
  double blend_margin = 1.0;       // (0, 1]. 1 보다 크게는 받지 않음 (게이트를 넓히지 않음)
  // 조향 정체: 진행 비율 < 1 인데 실측 조향이 안 도는(모든 |steer_rate| < 0.1 rad/s) 상태가 이만큼 이어지면 구동 0
  //   (조향 고장·전력 처짐 대비. 조향이 돌고 있으면 큰 각도 전환이라도 정체 아님). 0 이하면 blend 를 켜도 안 씀
  double blend_stall_s = 0.0;
  // ---- 정면 유지 되먹임 (기본 끔) ----
  // 켜면: 이동 명령이 있고 회전 명령이 0 인 동안, 그 이동을 시작할 때의 차체 방향(odom yaw — 실물은 자이로 포함)을 기억했다가
  //   회전 = clamp(kp × (기억한 방향 − 지금 방향), ±max_wz) 를 더함. 오차 deadband 안이면 0. 회전 명령이 오거나 멈추면 풂.
  //   실물: 정면 유지 명령인데 차체가 최대 7.2° 틀어짐. 너무 크게 섞으면 조향이 흔들려 ④ 게이트에 걸리므로 max_wz 를 작게.
  double heading_hold_kp = 0.0;          // [1/s] 0 = 끔
  double heading_hold_max_wz = 0.15;     // [rad/s]
  double heading_hold_deadband = 0.0087; // [rad] ≈ 0.5°
  // 보정 회전 상한을 이동 속도에 비례 (0 = 끔 = max_wz 만): 상한 = min(max_wz, 이 값 × 이동 속도).
  //   느릴수록 같은 회전이 조향각을 크게 바꿔 조향이 흔들림(실물 0.05 m/s 보정 시험에서 관찰). 0.4 → 조향 변화 약 3° 이내
  //   (회전 반경 0.13 m 모듈: atan(wz·0.13 / v) ≈ atan(0.4·0.13) = 3°)
  double heading_hold_wz_per_speed = 0.0;   // [rad/s per m/s]
  // ---- 이동 중 가지 유지 (기본 0 = 끔) ----
  // 이동 중에 들어온 작은 회전(|wz| ≤ 이 값, 정면 유지 · 경로 추종기 방향 보정)이 어느 바퀴든 다른 가지로 갈아타게 만들면
  //   회전을 갈아타지 않는 데까지만 줄임 (이분 탐색). 실물 140° 왕복: 추종기 보정 wz −0.014 rad/s(속도 0.02 m/s)로 rr 목표가
  //   −45.5° (끝 −44.5° + 여유 1°) 를 오가며 반대 가지와 번갈아 → 주행 중 186° 뒤집힘 · 3.3 s 손해. 켜면 정면 유지의 "통째로 생략" 대신 이걸 씀
  double keep_branch_max_wz = 0.0;   // [rad/s]
};

struct Pose2D
{
  double x = 0.0, y = 0.0, yaw = 0.0;
};

class SwerveDrive
{
public:
  SwerveDrive(SwerveKinematics kin, SwerveDriveConfig cfg);

  // 새 /cmd_vel. NaN · 무한대면 false + 이전 명령까지 취소 (안전장치 ②). 한계로 잘라서 저장
  bool set_command(const BodyTwist & raw, double now_s);
  // 차례 받기 (control_master 가 /control/motion_mode 로 10 Hz 로 계속 보냄, D15)
  //   drive = DRIVE 인가, id = 요청 번호. turn_timeout_s 넘게 안 오면 잠금 (control_master 가 죽거나 신호가 끊김)
  void set_turn(bool drive, uint32_t id, double now_s);
  // 지금 바퀴를 움직여도 되나 = 신선한 차례가 DRIVE
  bool drive_allowed(double now_s) const;
  // 마지막으로 반영한 요청 번호 (update() 가 이 번호의 차례로 계산했음 → 컨트롤러가 "잠갔음" 답으로 보냄)
  uint32_t applied_turn_id() const {return applied_id_;}
  bool applied_drive() const {return applied_drive_;}

  // 명령이 아직 신선한가
  bool command_fresh(double now_s) const;

  // 한 주기 계산: 측정한 모듈 상태 → 모듈 명령 (조향 각도 [rad], 바퀴 속도 [rad/s])
  std::vector<ModuleState> update(double now_s, double dt, const std::vector<ModuleState> & measured);

  // 모듈 상태를 모를 때 (측정값이 안 읽힘): 바퀴 0, 조향은 last_steer 그대로. 가속도 처음부터
  std::vector<ModuleState> stop_hold(const std::vector<double> & last_steer);

  // odom: 측정값으로 몸 속도 계산 → 적분. gyro_ok 면 회전 속도는 gyro_wz 를 씀 (D12)
  BodyTwist integrate_odom(const std::vector<ModuleState> & measured, double dt, bool gyro_ok, double gyro_wz);

  const Pose2D & pose() const {return pose_;}
  const BodyTwist & output() const {return out_;}   // 가속 제한을 거친 지금 목표 (0 이면 "세우는 중/섰음")
  // 시험 기록용: 이번 주기에 ④ 게이트가 구동을 막았나 / 동시 조향 진행 비율(0~1) / 조향 정체로 세웠나
  bool gate_blocked() const {return gate_blocked_;}
  double blend_ratio() const {return blend_ratio_;}
  double wz_keep_ratio() const {return wz_keep_ratio_;}   // 이동 중 가지 유지로 회전을 줄인 비율 (1 = 그대로)
  bool blend_stalled() const {return blend_stalled_;}
  bool command_feasible() const {return command_feasible_;}
  bool parking() const {return parking_;}             // 지금 바깥쪽 자세로 돌아가는 중 (또는 이미 그 자세를 잡고 있음)
  void reset();                                      // 켤 때: 목표 0, 명령 지움 (odom 은 유지)
  const SwerveKinematics & kinematics() const {return kin_;}

private:
  // 복귀 조건을 이번 주기에 갱신 → 복귀해야 하면 true
  bool update_park(double now_s, bool fresh_zero, const std::vector<ModuleState> & measured);
  std::vector<ModuleState> previous_targets_;
  bool command_feasible_ = true;
  bool gate_blocked_ = false;
  double blend_ratio_ = 1.0;
  bool translating_ = false;     // 멈춤 이후 병진을 냈는가 (true 동안 바깥쪽 선호 끔, 분기 유지)
  bool holding_heading_ = false;  // 정면 유지 중 (heading_hold_kp > 0)
  double wz_keep_ratio_ = 1.0;    // 이동 중 가지 유지로 회전을 줄인 비율 (1 = 안 줄임)
  double hold_yaw_ = 0.0;         // 기억한 차체 방향 [rad]
  double blend_since_ = -1.0;   // 진행 비율 < 1 이고 조향이 안 돌기 시작한 시각 (< 0 = 아님)
  bool blend_stalled_ = false;
  bool parking_ = false;
  double idle_since_ = -1.0;   // 신선한 0 명령 + 실측 정지가 시작된 시각 (< 0 = 아님)
  SwerveKinematics kin_;
  SwerveDriveConfig cfg_;
  BodyTwist cmd_{0.0, 0.0, 0.0};
  BodyTwist out_{0.0, 0.0, 0.0};
  bool have_cmd_ = false;
  double cmd_time_ = -1e9;
  bool have_turn_ = false;       // 차례를 모르면 안 움직임
  bool turn_drive_ = false;
  uint32_t turn_id_ = 0;
  double turn_time_ = -1e9;
  uint32_t applied_id_ = 0;
  bool applied_drive_ = false;
  Pose2D pose_;
};

}  // namespace swerve_controller
