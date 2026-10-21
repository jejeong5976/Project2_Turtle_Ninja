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
    std::vector<SteeringRange> ranges = {});

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
    bool cosine_scale = true, bool * feasible = nullptr) const;
  // ranges가 있으면 그 구간이 공통 steer_limit보다 우선한다.
  // 불가능한 방향은 전체 바퀴 0 + 현재 조향 유지로 반환하고 feasible=false.
  bool has_module_ranges() const {return !ranges_.empty();}

  // 조향만 도는 동안(바퀴 구동은 멈춘 단계) 바퀴가 따라 굴러야 하는 속도 [rad/s]
  //   바퀴가 조향축 옆 offset 에 있으면, 조향이 θ̇ 로 돌 때 바퀴 접지점이 굴림 방향으로 θ̇·offset 만큼 움직임
  //   → 바퀴를 −θ̇·offset / r 로 같이 굴려 주지 않으면 바닥에 끌림 (조향 모터 부담 ↑, 위치 틀어짐)
  double roll_while_steering(std::size_t i, double steer_rate) const;

  // 순기구학: 모듈 상태 → 몸 속도 (odom 계산용)
  //   모듈이 4개라 식이 8개, 모르는 것은 3개 → 최소제곱으로 "가장 그럴듯한" 값
  BodyTwist forward(const std::vector<ModuleState> & states) const;

  std::size_t size() const {return modules_.size();}
  double wheel_radius() const {return wheel_radius_;}

private:
  std::vector<ModuleGeometry> modules_;
  double wheel_radius_;
  double steer_limit_;
  std::vector<SteeringRange> ranges_;
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
//   ② 그 뒤에도 멈춰 있을 때마다 아주 천천히 바이어스를 따라감 (온도 변화 대응)
//   ③ IMU 가 timeout_s 넘게 안 오면 ready() = false → 바퀴 ω 로 자동 복귀
//      (OpenCR 플러그인은 IMU 가 고장이면 NaN 을 보냄 → 그런 값은 "안 온 것" 으로 침)
//
// 순수 C++ → test/test_swerve.cpp
// ============================================================================

class GyroYaw
{
public:
  GyroYaw(double bias_time_s = 2.0, double timeout_s = 0.2, double track_gain = 0.002)
  : bias_time_s_(bias_time_s), timeout_s_(timeout_s), gain_(track_gain) {}

  // 자이로 z 한 개 [rad/s]. still = 지금 몸통이 멈춰 있나 (바퀴 기준)
  void add_sample(double now_s, double gz, bool still)
  {
    if (!std::isfinite(gz)) {return;}   // NaN = IMU 없음 → 샘플로 안 침 (곧 ready() = false)
    last_t_ = now_s;
    last_gz_ = gz;
    if (!calibrated_) {
      if (!still) {start_t_ = -1.0; sum_ = 0.0; n_ = 0; return;}   // 움직이면 처음부터
      if (start_t_ < 0.0) {start_t_ = now_s;}
      sum_ += gz;
      ++n_;
      if (now_s - start_t_ >= bias_time_s_ && n_ > 0) {bias_ = sum_ / n_; calibrated_ = true;}
      return;
    }
    if (still) {bias_ += gain_ * (gz - bias_);}
  }

  bool ready(double now_s) const {return calibrated_ && now_s - last_t_ <= timeout_s_;}
  double rate() const {return last_gz_ - bias_;}   // 바이어스 뺀 회전 속도 [rad/s]
  double bias() const {return bias_;}
  bool calibrated() const {return calibrated_;}

private:
  double bias_time_s_, timeout_s_, gain_;
  bool calibrated_{false};
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
  bool command_feasible() const {return command_feasible_;}
  void reset();                                      // 켤 때: 목표 0, 명령 지움 (odom 은 유지)
  const SwerveKinematics & kinematics() const {return kin_;}

private:
  bool command_feasible_ = true;
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
