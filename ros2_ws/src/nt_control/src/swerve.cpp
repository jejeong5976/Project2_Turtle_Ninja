// swerve.cpp — 바퀴 쪽 순수 계산 (설명은 include/nt_control/swerve.hpp)

#include "nt_control/swerve.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace nt_control
{

// ============================================================================ ① 기구학
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kStopEps = 1e-4;   // 이보다 느리면 "멈춤" 으로 봄 [m/s]
}  // namespace

double wrap_angle(double a)
{
  a = std::fmod(a + kPi, 2.0 * kPi);
  if (a < 0.0) {a += 2.0 * kPi;}
  return a - kPi;
}

BodyTwist limit_accel(
  const BodyTwist & current, const BodyTwist & target,
  double max_lin_acc, double max_ang_acc, double dt)
{
  BodyTwist out = target;
  if (dt <= 0.0) {return current;}
  // 선속도: 바뀌는 양(벡터)의 크기를 max_lin_acc·dt 로 자름
  const double dvx = target.vx - current.vx;
  const double dvy = target.vy - current.vy;
  const double dv = std::hypot(dvx, dvy);
  const double lim = max_lin_acc * dt;
  if (max_lin_acc > 0.0 && dv > lim) {
    out.vx = current.vx + dvx * lim / dv;
    out.vy = current.vy + dvy * lim / dv;
  }
  // 회전
  const double dw = target.wz - current.wz;
  const double wlim = max_ang_acc * dt;
  if (max_ang_acc > 0.0 && std::fabs(dw) > wlim) {
    out.wz = current.wz + std::copysign(wlim, dw);
  }
  return out;
}

bool steering_aligned(
  const std::vector<ModuleState> & command,
  const std::vector<ModuleState> & measured, double tol)
{
  if (command.size() != measured.size()) {return false;}
  for (std::size_t i = 0; i < command.size(); ++i) {
    if (std::fabs(command[i].steer_angle - measured[i].steer_angle) > tol) {   // 접지 않음 (위 설명)
      return false;
    }
  }
  return true;
}

SwerveKinematics::SwerveKinematics(std::vector<ModuleGeometry> modules, double wheel_radius,
  double steer_limit, std::vector<SteeringRange> ranges)
: modules_(std::move(modules)), wheel_radius_(wheel_radius), steer_limit_(std::max(steer_limit, kPi / 2.0)),
  ranges_(std::move(ranges))
{
  if (modules_.empty()) {throw std::invalid_argument("모듈이 하나도 없음");}
  if (wheel_radius_ <= 0.0) {throw std::invalid_argument("바퀴 반지름은 0 보다 커야 함");}
  if (!ranges_.empty() && ranges_.size() != modules_.size()) {
    throw std::invalid_argument("모듈별 조향 범위 개수가 다름");
  }
  for (const auto & r : ranges_) {
    if (!std::isfinite(r.min) || !std::isfinite(r.max) || r.min > r.max || r.max-r.min > 2*kPi) {
      throw std::invalid_argument("조향 범위는 유한한 연속 구간(min <= max, 폭 <= 2pi)이어야 함");
    }
  }
}

std::vector<ModuleState> SwerveKinematics::inverse(
  const BodyTwist & cmd,
  const std::vector<ModuleState> & current,
  bool cosine_scale, bool * feasible) const
{
  if (feasible) {*feasible = true;}
  const bool have_current = current.size() == modules_.size();
  std::vector<ModuleState> out(modules_.size());

  if (!ranges_.empty()) {
    auto reject = [&]() {
        if (feasible) {*feasible = false;}
        // 끝에 닿도록 강제 정렬하지 않는다. 불가능하면 모든 모듈을 함께 정지.
        for (std::size_t j=0; j<out.size(); ++j) {
          out[j] = {have_current ? current[j].steer_angle :
            std::clamp(0.0, ranges_[j].min, ranges_[j].max), 0.0};
        }
        return out;
      };
    if (!std::isfinite(cmd.vx) || !std::isfinite(cmd.vy) || !std::isfinite(cmd.wz)) {return reject();}
    if (!current.empty() && !have_current) {return reject();}
    for (std::size_t i=0; i<out.size(); ++i) {
      const auto & r = ranges_[i];
      const double now = have_current ? current[i].steer_angle : std::clamp(0.0,r.min,r.max);
      if (!std::isfinite(now)) {return reject();}
      const double vx=cmd.vx-cmd.wz*modules_[i].y, vy=cmd.vy+cmd.wz*modules_[i].x;
      const double speed=std::hypot(vx,vy);
      if (speed < kStopEps) {
        // 측정이 끝점을 조금 넘어도 정지 중에는 현재 위치 유지. 이동 목표는 별도 선택.
        out[i]={now,-cmd.wz*modules_[i].offset/wheel_radius_};
        continue;
      }
      const double heading=std::atan2(vy,vx);
      double best=0.0, best_speed=0.0, distance=INFINITY;
      for (int reverse=0; reverse<2; ++reverse) {
        const double base=heading+reverse*kPi;
        const double first=std::ceil((r.min-base-1e-12)/(2*kPi));
        const double last=std::floor((r.max-base+1e-12)/(2*kPi));
        if (first>last) {continue;}
        const double n=std::clamp(std::round((now-base)/(2*kPi)),first,last);
        const double angle=std::clamp(base+n*2*kPi,r.min,r.max);
        // wrap하지 않음: 위치 관절은 허용 구간 안의 직접 경로로 움직인다.
        if (std::fabs(angle-now)<distance) {
          distance=std::fabs(angle-now); best=angle; best_speed=reverse ? -speed : speed;
        }
      }
      if (!std::isfinite(distance)) {return reject();}
      double wheel_lin=best_speed-cmd.wz*modules_[i].offset;
      // 긴 재정렬 동안 반대 방향으로 구동하지 않음.
      if (cosine_scale && have_current) {wheel_lin*=std::max(0.0,std::cos(best-now));}
      out[i]={best,wheel_lin/wheel_radius_};
    }
    return out;
  }

  for (std::size_t i = 0; i < modules_.size(); ++i) {
    // 1) 모듈 자리에서의 바닥 속도 = 몸 속도 + 회전 때문에 생기는 속도 (ω × r)
    //    회전 속도 ω 로 (x, y) 점은 (-ω·y, +ω·x) 방향으로 움직인다
    const double vx = cmd.vx - cmd.wz * modules_[i].y;
    const double vy = cmd.vy + cmd.wz * modules_[i].x;
    double speed = std::hypot(vx, vy);                 // [m/s]

    const double now_angle = have_current ? current[i].steer_angle : 0.0;

    // 2) 조향축이 거의 멈춤이면 조향은 지금 각도 그대로 (0° 로 확 돌아가는 떨림 방지)
    //    (회전 중심이 딱 조향축 위일 때도 바퀴는 offset 만큼 원을 돌아야 함 → 아래 6) 과 같은 식)
    if (speed < kStopEps) {
      out[i] = {now_angle, -cmd.wz * modules_[i].offset / wheel_radius_};
      continue;
    }

    // 3) 목표 각도
    double angle = std::atan2(vy, vx);

    // 4) 뒤집기: 90° 넘게 돌아야 하면 반대쪽을 보고 거꾸로 굴림
    if (have_current && std::fabs(wrap_angle(angle - now_angle)) > kPi / 2.0) {
      angle = wrap_angle(angle + kPi);
      speed = -speed;
    }

    // 4-2) 끝 넘김 막기: 모터가 실제로 갈 각도 = 지금 + (가장 짧은 차이). 끝을 넘으면 반대쪽 + 거꾸로
    if (have_current) {
      angle = now_angle + wrap_angle(angle - now_angle);
      if (angle > steer_limit_) {
        angle -= kPi;
        speed = -speed;
      } else if (angle < -steer_limit_) {
        angle += kPi;
        speed = -speed;
      }
    }

    // 5) 바퀴가 조향축 옆에 있으면 (offset): 바퀴 굴림 속도 = 조향축 속도 − ω × offset
    //    offset 은 모듈에 붙어 있어서 뒤집어도 식이 같음 (뒤집으면 바퀴가 반대편으로 가고 speed 부호도 바뀜)
    double wheel_lin = speed - cmd.wz * modules_[i].offset;

    // 6) 코사인 줄이기: 조향이 목표에서 θ 만큼 덜 돌았으면 바퀴 속도 × cos θ
    //    (θ = 0 이면 그대로, 90° 면 0 → 엉뚱한 방향으로 미는 것 방지)
    if (cosine_scale && have_current) {
      wheel_lin *= std::cos(wrap_angle(angle - now_angle));
    }

    out[i] = {angle, wheel_lin / wheel_radius_};       // 몸 속도 [m/s] → 바퀴 각속도 [rad/s]
  }
  return out;
}

double SwerveKinematics::roll_while_steering(std::size_t i, double steer_rate) const
{
  return -steer_rate * modules_.at(i).offset / wheel_radius_;
}

BodyTwist SwerveKinematics::forward(const std::vector<ModuleState> & states) const
{
  if (states.size() != modules_.size()) {
    throw std::invalid_argument("모듈 상태 개수가 모듈 수와 다름");
  }

  // 모듈 i 마다 식 2개 (모듈 방향 c = cos θ, s = sin θ):
  //   ① 굴림 방향: 바퀴가 잰 속도 = 조향축 속도의 앞 성분 − (ω + θ̇_i)·offset
  //        r·w_i + θ̇_i·offset_i = c·vx + s·vy + ω·(x_i·s − y_i·c − offset_i)
  //        (θ̇_i = 조향이 도는 속도. 조향만 돌 때 바퀴가 따라 구른 몫을 빼 줘야 몸이 움직였다고 착각 안 함)
  //   ② 옆 방향 : 바퀴는 옆으로 안 미끄러짐 → 조향축 속도의 옆 성분 = 0
  //        0     = −s·vx + c·vy + ω·(x_i·c + y_i·s)
  // (offset = 0 이면 기본 식 u = vx − ω·y, v = vy + ω·x 를 모듈 방향으로 돌려 놓은 것과 같음)
  // 식 2N 개 → 정규방정식 (AᵀA) p = Aᵀb 로 3×3 만 풀면 됨
  double ata[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
  double atb[3] = {0, 0, 0};
  auto add_row = [&](const double a[3], double b) {
      for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {ata[r][c] += a[r] * a[c];}
        atb[r] += a[r] * b;
      }
    };

  for (std::size_t i = 0; i < modules_.size(); ++i) {
    const double lin = states[i].wheel_speed * wheel_radius_ + states[i].steer_rate * modules_[i].offset;
    const double c = std::cos(states[i].steer_angle);
    const double s = std::sin(states[i].steer_angle);
    const double x = modules_[i].x;
    const double y = modules_[i].y;
    const double roll[3] = {c, s, x * s - y * c - modules_[i].offset};
    const double side[3] = {-s, c, x * c + y * s};
    add_row(roll, lin);
    add_row(side, 0.0);
  }

  // 3×3 가우스 소거 (크기가 작아서 직접)
  double m[3][4];
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {m[r][c] = ata[r][c];}
    m[r][3] = atb[r];
  }
  for (int col = 0; col < 3; ++col) {
    int pivot = col;
    for (int r = col + 1; r < 3; ++r) {
      if (std::fabs(m[r][col]) > std::fabs(m[pivot][col])) {pivot = r;}
    }
    if (std::fabs(m[pivot][col]) < 1e-12) {
      throw std::runtime_error("모듈 배치가 한 점에 몰려 있어 회전을 구할 수 없음");
    }
    for (int c = 0; c < 4; ++c) {std::swap(m[col][c], m[pivot][c]);}
    for (int r = 0; r < 3; ++r) {
      if (r == col) {continue;}
      const double f = m[r][col] / m[col][col];
      for (int c = col; c < 4; ++c) {m[r][c] -= f * m[col][c];}
    }
  }
  return {m[0][3] / m[0][0], m[1][3] / m[1][1], m[2][3] / m[2][2]};
}

// ============================================================================ ③ 운전석
SwerveDrive::SwerveDrive(SwerveKinematics kin, SwerveDriveConfig cfg)
: kin_(std::move(kin)), cfg_(cfg)
{
}

bool SwerveDrive::set_command(const BodyTwist & raw, double now_s)
{
  if (!std::isfinite(raw.vx) || !std::isfinite(raw.vy) || !std::isfinite(raw.wz)) {
    have_cmd_ = false;   // 이상한 값 → 이전 명령까지 취소
    return false;
  }
  double vx = raw.vx;
  double vy = raw.vy;
  const double lin = std::hypot(vx, vy);
  if (lin > cfg_.max_linear) {   // 방향은 유지하고 크기만 줄임
    vx *= cfg_.max_linear / lin;
    vy *= cfg_.max_linear / lin;
  }
  cmd_ = {vx, vy, std::clamp(raw.wz, -cfg_.max_angular, cfg_.max_angular)};
  cmd_time_ = now_s;
  have_cmd_ = true;
  return true;
}

void SwerveDrive::set_turn(bool drive, uint32_t id, double now_s)
{
  have_turn_ = true;
  turn_drive_ = drive;
  turn_id_ = id;
  turn_time_ = now_s;
}

bool SwerveDrive::drive_allowed(double now_s) const
{
  const double age = now_s - turn_time_;
  return have_turn_ && turn_drive_ && age < cfg_.turn_timeout_s && age >= -0.05;
}

bool SwerveDrive::command_fresh(double now_s) const
{
  return have_cmd_ && (now_s - cmd_time_) < cfg_.cmd_timeout_s && (now_s - cmd_time_) >= -0.05;
}

std::vector<ModuleState> SwerveDrive::update(
  double now_s, double dt, const std::vector<ModuleState> & measured)
{
  // ① 신선한 명령 + 신선한 바퀴 차례일 때만 따라감. 아니면 목표 0 (가속 제한으로 서서히)
  const bool allowed = drive_allowed(now_s);
  const BodyTwist wanted = (command_fresh(now_s) && allowed) ? cmd_ : BodyTwist{0.0, 0.0, 0.0};
  // 이번 주기에 반영한 차례 (DRIVE 가 아니면 = 잠금). 오래돼서 잠근 경우도 "DRIVE 아님" 으로 답함
  applied_id_ = have_turn_ ? turn_id_ : 0;
  applied_drive_ = allowed;

  // 원하는 방향 자체가 불가능하면 가속 램프가 다른 방향으로 움직이기 전에 거부.
  command_feasible_ = true;
  if (kin_.has_module_ranges()) {
    auto hold = kin_.inverse(wanted, measured, cfg_.cosine_scale, &command_feasible_);
    if (!command_feasible_) {
      out_ = {0.0,0.0,0.0}; applied_drive_ = false;
      return hold;
    }
  }

  // ② 가속 제한
  out_ = limit_accel(out_, wanted, cfg_.max_linear_accel, cfg_.max_angular_accel, std::clamp(dt, 0.0, 0.1));

  // ③ 역기구학
  auto modules = kin_.inverse(out_, measured, cfg_.cosine_scale, &command_feasible_);
  if (!command_feasible_) {
    out_ = {0.0,0.0,0.0}; applied_drive_ = false;
    return modules;  // 조향 보상 분기가 정지 명령을 다시 덮어쓰지 않게 함.
  }

  // ④ 조향 먼저
  if (cfg_.steer_then_drive && !steering_aligned(modules, measured, cfg_.steer_tolerance)) {
    for (std::size_t i = 0; i < modules.size(); ++i) {
      const double rate = i < measured.size() ? measured[i].steer_rate : 0.0;
      modules[i].wheel_speed = cfg_.roll_while_steering ? kin_.roll_while_steering(i, rate) : 0.0;
    }
    out_ = {0.0, 0.0, 0.0};   // 조향이 다 돌면 가속도 처음부터
  }
  return modules;
}

std::vector<ModuleState> SwerveDrive::stop_hold(const std::vector<double> & last_steer)
{
  out_ = {0.0, 0.0, 0.0};
  applied_drive_ = false;   // 바퀴 0 을 썼으니 "잠김" 으로 답해도 됨
  std::vector<ModuleState> m(kin_.size(), ModuleState{0.0, 0.0});
  for (std::size_t i = 0; i < m.size() && i < last_steer.size(); ++i) {m[i].steer_angle = last_steer[i];}
  return m;
}

BodyTwist SwerveDrive::integrate_odom(
  const std::vector<ModuleState> & measured, double dt, bool gyro_ok, double gyro_wz)
{
  BodyTwist v = kin_.forward(measured);
  if (gyro_ok && std::isfinite(gyro_wz)) {v.wz = gyro_wz;}   // 회전은 자이로 (바퀴 미끄러짐에 안 속음)
  if (dt > 0.0 && dt < 0.5) {
    // 몸 기준 속도를 바닥(odom) 기준으로 돌려서 더하기
    pose_.x += (v.vx * std::cos(pose_.yaw) - v.vy * std::sin(pose_.yaw)) * dt;
    pose_.y += (v.vx * std::sin(pose_.yaw) + v.vy * std::cos(pose_.yaw)) * dt;
    pose_.yaw = wrap_angle(pose_.yaw + v.wz * dt);
  }
  return v;
}

void SwerveDrive::reset()
{
  out_ = {0.0, 0.0, 0.0};
  have_cmd_ = false;
}

}  // namespace nt_control
