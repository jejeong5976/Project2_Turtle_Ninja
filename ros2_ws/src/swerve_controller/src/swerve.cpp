// swerve.cpp — 바퀴 쪽 순수 계산 (설명은 include/swerve_controller/swerve.hpp)

#include "swerve_controller/swerve.hpp"

#include <algorithm>
#include <array>
#include <stdexcept>
#include <utility>

namespace swerve_controller
{

// ============================================================================ ① 기구학
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kStopEps = 1e-4;   // 이보다 느리면 "멈춤" 으로 봄 [m/s] (회전 [rad/s] 비교에도 같은 값)
// 같은 가지 안의 각도 변화로 보는 한도: 이보다 크게 바뀌면 "다른 가지로 갈아탐" (정면 유지 검사 · 이동 중 가지 유지 공통)
constexpr double kBranchJump = 30.0 * kPi / 180.0;
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
  if (dt <= 0.0) {return current;}
  // 선속도 · 회전을 **같은 비율로** 바꿈: 둘 중 오래 걸리는 쪽에 맞춰 이번 주기에 갈 비율 f 를 정함.
  //   따로 자르면 출발 중 vx:vy:ω 비율이 바뀌어 모듈 조향각이 계속 변하고, 느린 조향(≈ 55°/s)이 못 따라가
  //   "조향 먼저" 가 속도를 0 으로 되돌리기를 반복함 (2026-10-19 가짜 보드 전진+회전에서 확인). 각 한계는 그대로 지킴
  const double dvx = target.vx - current.vx;
  const double dvy = target.vy - current.vy;
  const double dw = target.wz - current.wz;
  const double t_lin = max_lin_acc > 0.0 ? std::hypot(dvx, dvy) / max_lin_acc : 0.0;   // 그 한계로 걸리는 시간
  const double t_ang = max_ang_acc > 0.0 ? std::fabs(dw) / max_ang_acc : 0.0;
  const double t_need = std::max(t_lin, t_ang);
  if (t_need <= dt) {return target;}
  const double f = dt / t_need;
  return {current.vx + dvx * f, current.vy + dvy * f, current.wz + dw * f};
}

bool steering_aligned(
  const std::vector<ModuleState> & command,
  const std::vector<ModuleState> & measured, double tol)
{
  if (command.size() != measured.size()) {return false;}
  for (std::size_t i = 0; i < command.size(); ++i) {
    if (!(std::fabs(command[i].steer_angle - measured[i].steer_angle) <= tol)) {   // NaN 이면 정렬 안 됨으로   // 접지 않음 (위 설명)
      return false;
    }
  }
  return true;
}

SwerveKinematics::SwerveKinematics(std::vector<ModuleGeometry> modules, double wheel_radius,
  double steer_limit, std::vector<SteeringRange> ranges, bool coherent, double branch_margin, double end_slack,
  double max_range_width)
: modules_(std::move(modules)), wheel_radius_(wheel_radius), steer_limit_(std::max(steer_limit, kPi / 2.0)),
  ranges_(std::move(ranges)), coherent_(coherent), branch_margin_(branch_margin), end_slack_(end_slack)
{
  if (!std::isfinite(end_slack_) || end_slack_ < 0.0 || end_slack_ > 2.0 * kPi / 180.0) {
    throw std::invalid_argument("steer_end_slack 은 0 ~ 2° 이어야 함");
  }
  if (modules_.empty()) {throw std::invalid_argument("모듈이 하나도 없음");}
  if (!(wheel_radius_ > 0.0) || !std::isfinite(wheel_radius_)) {throw std::invalid_argument("바퀴 반지름은 0 보다 큰 유한한 값");}
  for (const auto & m : modules_) {
    if (!std::isfinite(m.x) || !std::isfinite(m.y) || !std::isfinite(m.offset)) {throw std::invalid_argument("모듈 위치 · offset 은 유한한 값");}
  }
  // steer_limit: 범위 없을 때 공통 ±steer_limit — 폭 2·steer_limit 가 범위 폭 상한 안이어야 coherent 의 임시 범위가 만들어짐
  if (!std::isfinite(steer_limit_) || 2.0 * steer_limit_ > max_range_width + 1e-12) {
    throw std::invalid_argument("steer_limit 는 유한하고 범위 폭 상한의 절반 이하 (한 바퀴 모드 π)");
  }
  if (!ranges_.empty() && ranges_.size() != modules_.size()) {
    throw std::invalid_argument("모듈별 조향 범위 개수가 다름");
  }
  if (!(max_range_width >= 2*kPi - 1e-12 && max_range_width <= 6*kPi + 1e-12)) {
    throw std::invalid_argument("범위 폭 상한은 2π(한 바퀴) ~ 6π(세 바퀴)");
  }
  for (const auto & r : ranges_) {
    if (!std::isfinite(r.min) || !std::isfinite(r.max) || r.min > r.max || r.max-r.min > max_range_width) {
      throw std::invalid_argument("조향 범위는 유한한 연속 구간(min <= max, 폭 <= 상한, 기본 2pi)이어야 함");
    }
  }
}

std::vector<ModuleState> SwerveKinematics::inverse(
  const BodyTwist & cmd,
  const std::vector<ModuleState> & current,
  bool cosine_scale, bool * feasible, const std::vector<ModuleState> & previous, bool prefer_outward) const
{
  if (feasible) {*feasible = true;}
  const bool have_current = current.size() == modules_.size();
  std::vector<ModuleState> out(modules_.size());

  if (coherent_ && ranges_.empty()) {
    // Explicit bounded candidates also support mock configurations without zero ticks.
    SwerveKinematics bounded(modules_, wheel_radius_, steer_limit_,
      std::vector<SteeringRange>(modules_.size(), {-steer_limit_, steer_limit_}), true, branch_margin_, end_slack_);
    bounded.travel_weight_ = travel_weight_;
    bounded.coherent_max_travel_ = coherent_max_travel_;
    bounded.end_guard_ = end_guard_;
    return bounded.inverse(cmd, current, cosine_scale, feasible, previous, prefer_outward);
  }
  if (!ranges_.empty()) {
    // New branches need 3 degrees of clearance when a safe equivalent exists.
    // A retained target may approach the true limit. No physical range is enlarged.
    const double margin = branch_margin_;
    constexpr double hysteresis = 20.0 * kPi / 180.0;
    const bool have_previous = previous.size() == modules_.size();
    auto retained = [&](std::size_t i, double angle) {
        return have_previous && std::fabs(angle - previous[i].steer_angle) < kPi / 2;
      };
    auto score = [&](std::size_t i, double angle) {
        return std::fabs(angle) + (have_previous && !retained(i, angle) ? hysteresis : 0.0);
      };
    // 범위 끝 벌점 (end_guard_, 기본 0): 멈춘 상태에서 고를 때만. 이동 중에는 0 → 주행 중 가지를 바꾸게 만들지 않음
    auto end_penalty = [&](std::size_t i, double angle) {
        if (end_guard_ <= 0.0 || !prefer_outward) {return 0.0;}
        const double room = std::min(angle - ranges_[i].min, ranges_[i].max - angle);
        return room >= end_guard_ ? 0.0 : 4.0 * kPi * (end_guard_ - std::max(0.0, room)) / end_guard_;
      };
    double common = NAN;
    if (coherent_ && std::hypot(cmd.vx, cmd.vy) >= kStopEps) {
      const double heading = std::atan2(cmd.vy, cmd.vx);
      double cost = INFINITY;
      // A common translation reference must be reachable by every module.
      // For compound motion it is only a branch preference: each module keeps its
      // exact local velocity heading. This also avoids flips from tiny yaw noise.
      const double first = std::ceil((ranges_[0].min - heading) / kPi);
      const double last = std::floor((ranges_[0].max - heading) / kPi);
      for (double n = first; n <= last; n += 1.0) {
        const double a = heading + n * kPi;
        double c = 0.0;
        bool valid = true;
        for (std::size_t j = 0; j < ranges_.size(); ++j) {
          const auto & r = ranges_[j];
          const double guard = retained(j, a) ? 0.0 : margin;
          if (a < r.min + guard - end_slack_ || a > r.max - guard + end_slack_) {valid = false; break;}
          c += score(j, a) + end_penalty(j, a);
          // 돌아야 할 양 (travel_weight_, 기본 0): 그 바퀴가 지금 각도에서 공통 방향까지
          if (have_current && std::isfinite(current[j].steer_angle)) {
            c += travel_weight_ * std::fabs(a - current[j].steer_angle);
            // 공통 자세 이동 한도 (coherent_max_travel_, 기본 0 = 없음)
            if (coherent_max_travel_ > 0.0 && std::fabs(a - current[j].steer_angle) > coherent_max_travel_) {valid = false; break;}
          }
        }
        if (valid && c < cost) {cost = c; common = a;}
      }
    }
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
      struct Candidate {double angle; double speed; bool safe;};
      std::array<Candidate, 2> cand_buf{};   // 정 · 역 두 후보 (주기마다 힙 할당 안 하게, 2026-10-29)
      std::size_t n_cand = 0;
      for (int reverse=0; reverse<2; ++reverse) {
        const double base=heading+reverse*kPi;
        // end_slack_(기본 0): 끝을 이만큼 넘는 목표는 끝 각도로 보내고 바퀴 속도 × cos(차이) (2026-10-26, 최대 시험에서만 1°).
        //   범위는 넓히지 않음 — rr −45° 처럼 0.5° 모자라 180° 가까이 반대로 도는 것만 막음
        const double first=std::ceil((r.min-base-end_slack_-1e-12)/(2*kPi));
        const double last=std::floor((r.max-base+end_slack_+1e-12)/(2*kPi));
        if (first>last) {continue;}
        const double n=std::clamp(std::round((now-base)/(2*kPi)),first,last);
        const double angle=std::clamp(base+n*2*kPi,r.min,r.max);
        const double v=(reverse ? -speed : speed)*std::cos(base+n*2*kPi-angle);
        cand_buf[n_cand++] = {angle, v,
          retained(i, angle) || (angle >= r.min + margin && angle <= r.max - margin)};
      }
      const bool safe_exists = std::any_of(cand_buf.begin(), cand_buf.begin() + n_cand,
        [](const Candidate & c) {return c.safe;});
      // Coherent translation: of the two parallel poses (angle, angle+pi) prefer the one whose
      // wheel offset points away from the body centre, so sideways driving keeps every wheel
      // outboard instead of tucking two diagonal wheels under the deck (user request 2026-10-26).
      // A deadband near the tie keeps the old choice; physical ranges are not changed.
      auto outward = [&](double a) {
          return -modules_[i].offset * std::sin(a) * modules_[i].x + modules_[i].offset * std::cos(a) * modules_[i].y;
        };
      const double out_deadband = std::fabs(modules_[i].offset) * std::hypot(modules_[i].x, modules_[i].y) *
        std::sin(10.0 * kPi / 180.0);
      // 바깥쪽 선호는 (a) 몸이 병진 중이 아닐 때만 (prefer_outward), (b) 바깥쪽 후보가 범위 끝에서 10° 넘게 안쪽일 때만.
      //   병진 중에는 그 자리에 "직전 목표와 같은 분기" 를 같은 무게로 → 멈춤↔이동 경계에서 분기가 오락가락하지 않음.
      //   (b): rr 은 −45° 를 못 가서 바깥쪽 = 315° 가 되는데, 그쪽은 끝(315.4°)에 붙어 있고 가는 데 200° 넘게 돌아감
      //   (2026-10-26 실물 floor_max_050: 이동 0 mm 구간의 원인). 자세와 무관한 규칙이라 같은 명령 = 같은 자세는 유지.
      constexpr double out_guard = 10.0 * kPi / 180.0;
      const bool outward_alt = prefer_outward && std::any_of(cand_buf.begin(), cand_buf.begin() + n_cand,
        [&](const Candidate & d) {
          return (!coherent_ || !safe_exists || d.safe) && outward(d.angle) >= -out_deadband &&
                 d.angle >= r.min + out_guard && d.angle <= r.max - out_guard;
        });
      for (std::size_t ci = 0; ci < n_cand; ++ci) {
        const auto & c = cand_buf[ci];
        if (coherent_ && safe_exists && !c.safe) {continue;}
        // nearest(모듈별 범위): 실측에서 돌아야 할 양. coherent: 공통 자세 · 목표 히스테리시스 · 바깥쪽 선호 + 돌아야 할 양 × travel_weight
        //   (2026-10-29 정리: 예전 한 줄 삼항식은 ?: 우선순위 때문에 뒤의 항이 coherent 에만 붙었음 — 범위 끝 벌점은 두 정책 모두에)
        const bool switched = have_previous && !retained(i, c.angle);
        double cost = std::fabs(c.angle - now);
        if (coherent_) {
          cost = travel_weight_ * std::fabs(c.angle - now);   // 돌아야 할 양 (기본 0)
          if (std::isfinite(common)) {
            cost += std::fabs(c.angle - common) + (switched ? hysteresis : 0.0) +
              (prefer_outward ? (outward_alt && outward(c.angle) < -out_deadband ? 2.0 * kPi : 0.0) :
                                (switched ? 2.0 * kPi : 0.0));
          } else {
            cost += score(i, c.angle) + (!prefer_outward && switched ? 2.0 * kPi : 0.0);
          }
        }
        cost += end_penalty(i, c.angle);   // 범위 끝 벌점 (기본 0, 멈춘 상태에서 고를 때만)
        if (cost < distance) {distance=cost; best=c.angle; best_speed=c.speed;}
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

void SwerveKinematics::set_travel_weight(double w)
{
  if (!std::isfinite(w) || w < 0.0 || w > 10.0) {
    throw std::invalid_argument("steer_travel_weight 는 0 ~ 10 이어야 함");
  }
  travel_weight_ = w;
}

void SwerveKinematics::set_branch_options(double coherent_max_travel, double end_guard)
{
  if (!std::isfinite(coherent_max_travel) || coherent_max_travel < 0.0 || coherent_max_travel > kPi + 1e-9) {
    throw std::invalid_argument("steer_coherent_max_travel 은 0 ~ π 이어야 함");
  }
  if (!std::isfinite(end_guard) || end_guard < 0.0 || end_guard > 45.0 * kPi / 180.0 + 1e-9) {
    throw std::invalid_argument("steer_end_guard 는 0 ~ 45° 이어야 함");
  }
  coherent_max_travel_ = coherent_max_travel;
  end_guard_ = end_guard;
}

bool SwerveKinematics::park_reachable() const
{
  for (const auto & r : ranges_) {
    if (r.min > 0.0 || r.max < 0.0) {return false;}
  }
  return true;
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
  if (cfg_.max_linear > 0.0 && lin > cfg_.max_linear) {   // 방향은 유지하고 크기만 줄임
    vx *= cfg_.max_linear / lin;
    vy *= cfg_.max_linear / lin;
  }
  cmd_ = {vx, vy, cfg_.max_angular > 0.0 ? std::clamp(raw.wz, -cfg_.max_angular, cfg_.max_angular) : raw.wz};
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
  const bool fresh = command_fresh(now_s) && allowed;
  BodyTwist wanted = fresh ? cmd_ : BodyTwist{0.0, 0.0, 0.0};
  // ①-1 정면 유지 되먹임 (heading_hold_kp > 0 일 때만): 이동 + 회전 명령 0 → 시작 때 방향으로 작은 회전 보정
  if (cfg_.heading_hold_kp > 0.0 && fresh && std::fabs(cmd_.wz) < kStopEps && std::hypot(cmd_.vx, cmd_.vy) >= kStopEps) {
    if (!holding_heading_) {holding_heading_ = true; hold_yaw_ = pose_.yaw;}
    const double err = wrap_angle(hold_yaw_ - pose_.yaw);
    if (std::fabs(err) > cfg_.heading_hold_deadband) {
      BodyTwist corrected = wanted;
      double lim = cfg_.heading_hold_max_wz;
      if (cfg_.heading_hold_wz_per_speed > 0.0) {
        lim = std::min(lim, cfg_.heading_hold_wz_per_speed * std::hypot(cmd_.vx, cmd_.vy));   // 느리면 약하게
      }
      corrected.wz = std::clamp(cfg_.heading_hold_kp * err, -lim, lim);
      // 보정 회전 때문에 어느 바퀴든 자세를 갈아타면(범위 끝에 붙은 바퀴 — 10-28 모의: rr −44.5° 에서 반대 분기로 돌기 시작)
      //   이번 주기는 보정 안 함. 같은 분기 안의 작은 각도 변화(< 30°)만 허용
      //   이동 중 가지 유지(keep_branch_max_wz)가 켜져 있으면 이 검사 대신 아래 ①-1b 가 갈아타지 않는 데까지만 줄임
      const bool keep_branch = cfg_.keep_branch_max_wz > 0.0 && translating_ &&
        std::fabs(corrected.wz) <= cfg_.keep_branch_max_wz;   // ①-1b 가 맡는 경우만 (아니면 예전 검사)
      bool same_branch = measured.size() == kin_.size();
      if (!keep_branch && same_branch && kin_.has_module_ranges()) {
        bool fa = true, fb = true;
        const auto a = kin_.inverse(wanted, measured, false, &fa, previous_targets_, false);
        const auto b = kin_.inverse(corrected, measured, false, &fb, previous_targets_, false);
        same_branch = fa && fb;
        for (std::size_t i = 0; same_branch && i < a.size(); ++i) {
          if (std::fabs(a[i].steer_angle - b[i].steer_angle) > kBranchJump) {same_branch = false;}
        }
      }
      if (same_branch || keep_branch) {wanted = corrected;}
    }
  } else {
    holding_heading_ = false;
  }
  // ①-1b 이동 중 가지 유지 (keep_branch_max_wz > 0 일 때만, 2026-10-29): 이미 병진 중이고 작은 회전이 섞였을 때
  //   회전 0 일 때의 바퀴 각도에서, 회전 때문에 생기는 기하학적 방향 변화를 빼고도 30° 넘게 달라지는 바퀴
  //   (= 반대 방향 · 한 바퀴 감기 같은 다른 가지)가 생기면 회전을 그 직전 비율까지 줄임.
  //   (10-29 오후 고침: 예전엔 기하학적 변화까지 30° 로 막아 느린 곡선 명령 — 0.02 m/s + 0.15 rad/s 등 — 의 회전을 깎았음)
  wz_keep_ratio_ = 1.0;
  if (cfg_.keep_branch_max_wz > 0.0 && translating_ && kin_.has_module_ranges() && measured.size() == kin_.size() &&
      std::hypot(wanted.vx, wanted.vy) >= kStopEps && std::fabs(wanted.wz) >= kStopEps &&
      std::fabs(wanted.wz) <= cfg_.keep_branch_max_wz)
  {
    BodyTwist no_turn = wanted;
    no_turn.wz = 0.0;
    bool f0 = true;
    const auto base = kin_.inverse(no_turn, measured, false, &f0, previous_targets_, false);
    auto keeps = [&](double s) {
        BodyTwist t = wanted;
        t.wz *= s;
        bool f = true;
        const auto b = kin_.inverse(t, measured, false, &f, previous_targets_, false);
        if (!f) {return false;}
        for (std::size_t i = 0; i < b.size(); ++i) {
          const auto & g = kin_.modules()[i];
          const double mvx = t.vx - t.wz * g.y, mvy = t.vy + t.wz * g.x;   // 이 모듈 조향축 속도
          if (std::hypot(mvx, mvy) < kStopEps) {continue;}               // 거의 멈춤 → 각도 유지, 가지 상관없음
          const double turn = std::atan2(t.vx * mvy - t.vy * mvx, t.vx * mvx + t.vy * mvy);   // 회전 0 방향에서 돈 각
          if (std::fabs(b[i].steer_angle - base[i].steer_angle - turn) > kBranchJump) {return false;}
        }
        return true;
      };
    if (f0 && !keeps(1.0)) {
      double lo = 0.0, hi = 1.0;
      for (int k = 0; k < 8; ++k) {const double mid = 0.5 * (lo + hi); (keeps(mid) ? lo : hi) = mid;}   // 1/256
      wanted.wz *= lo;
      wz_keep_ratio_ = lo;
    }
  }
  // 이번 주기에 반영한 차례 (DRIVE 가 아니면 = 잠금). 오래돼서 잠근 경우도 "DRIVE 아님" 으로 답함
  applied_id_ = have_turn_ ? turn_id_ : 0;
  applied_drive_ = allowed;

  // 원하는 방향 자체가 불가능하면 가속 램프가 다른 방향으로 움직이기 전에 거부.
  // 바깥쪽 선호는 "멈춤 → 출발" 때만. 한 번 병진을 냈으면 멈춤 명령(또는 끊김)으로 몸이 설 때까지 지금 분기를 유지
  //   (방향 전환 중 조향을 기다리느라 출력이 잠깐 0 이 돼도 이동 중으로 봄 → 분기 오락가락 없음)
  if ((!fresh || std::hypot(wanted.vx, wanted.vy) < kStopEps) && std::hypot(out_.vx, out_.vy) < kStopEps) {
    translating_ = false;
  }
  const bool prefer_outward = !translating_;
  command_feasible_ = true;
  if (kin_.has_module_ranges()) {
    auto hold = kin_.inverse(wanted, measured, cfg_.cosine_scale, &command_feasible_, {}, prefer_outward);
    if (!command_feasible_) {
      out_ = {0.0,0.0,0.0}; applied_drive_ = false;
      return hold;
    }
  }

  // ①-2 정지 후 바깥쪽 자세 복귀 (park_on_idle). 조향만 0° 로, 바퀴는 offset 보상만 → 몸은 그 자리
  const bool fresh_zero = fresh && std::hypot(cmd_.vx, cmd_.vy) < kStopEps && std::fabs(cmd_.wz) < kStopEps;
  if (update_park(now_s, fresh_zero, measured)) {
    out_ = {0.0, 0.0, 0.0};
    previous_targets_.clear();
    std::vector<ModuleState> park(kin_.size(), ModuleState{0.0, 0.0});
    for (std::size_t i = 0; i < park.size(); ++i) {
      park[i].wheel_speed = cfg_.roll_while_steering ? kin_.roll_while_steering(i, measured[i].steer_rate) : 0.0;
    }
    return park;
  }

  // ② 가속 제한
  const BodyTwist before = out_;
  out_ = limit_accel(out_, wanted, cfg_.max_linear_accel, cfg_.max_angular_accel, std::clamp(dt, 0.0, 0.1));

  // ②-2 동시 조향·구동 (기본 끔): 움직이는 중에만. 조향이 따라온 만큼만 새 방향으로 진행
  blend_ratio_ = 1.0;
  gate_blocked_ = false;
  // 이동(병진) 중일 때만. 제자리 회전에서 넘어갈 때는 기존 "조향 먼저" (2026-10-26 실물: 회전→전진에서 더 돌고 덜 감)
  // 회전이 주인 명령(모듈 자리 회전 속도 > 병진 속도, 반경 0.13 m 기준)도 제외 → 가속 램프 중 섞이는 것도 기존대로
  const double trans = std::hypot(before.vx, before.vy);
  const bool moving = trans >= kStopEps && trans >= std::fabs(before.wz) * 0.13;
  bool measured_ok = measured.size() == kin_.size();
  for (const auto & m : measured) {if (!std::isfinite(m.steer_angle)) {measured_ok = false;}}
  const bool blend_on = cfg_.blend_steer_drive && cfg_.blend_stall_s > 0.0;
  blend_stalled_ = false;
  std::vector<ModuleState> final_targets;   // 동시 모드에서 조향 모터에 보낼 최종 각도 (비어 있으면 평소대로)
  if (blend_on && fresh && moving && measured_ok) {
    const double tol = cfg_.steer_tolerance * std::clamp(cfg_.blend_margin, 0.0, 1.0);
    const BodyTwist target = out_;
    auto lerp = [&](double a) {
      return BodyTwist{before.vx + a * (target.vx - before.vx), before.vy + a * (target.vy - before.vy),
                       before.wz + a * (target.wz - before.wz)};
    };
    auto fits = [&](double a) {
      bool feas = true;
      const auto m = kin_.inverse(lerp(a), measured, cfg_.cosine_scale, &feas, previous_targets_, prefer_outward);
      return feas && steering_aligned(m, measured, tol);
    };
    if (!fits(1.0) && fits(0.0)) {
      double lo = 0.0, hi = 1.0;
      for (int k = 0; k < 14; ++k) {const double mid = 0.5 * (lo + hi); (fits(mid) ? lo : hi) = mid;}
      out_ = lerp(lo);
      blend_ratio_ = lo;
      // 바퀴는 진행 비율 명령(조향 실측과 맞는 방향)으로 구르고, 조향은 새 명령(가속 제한 전)의 각도로 바로 → 조향이 제 속도로 돎
      //   (가속 제한 뒤 값을 쓰면 진행 비율 근처에 묶여 다시 몇 도만 앞서감)
      bool final_ok = true;
      final_targets = kin_.inverse(wanted, measured, false, &final_ok, previous_targets_, prefer_outward);
      if (!final_ok) {final_targets.clear();}
    }
    // 조향 정체 감시: 진행 비율 < 1 인데 조향이 안 도는 상태가 blend_stall_s 넘게 이어지면 구동을 세움 (아래 ④ 처럼 out_ = 0)
    if (blend_ratio_ < 1.0) {
      constexpr double kTurning = 0.1;   // [rad/s] 이보다 빠르게 도는 조향이 하나라도 있으면 "진행 중"
      const bool turning = std::any_of(measured.begin(), measured.end(), [](const ModuleState & m) {
          return std::isfinite(m.steer_rate) && std::fabs(m.steer_rate) > kTurning;
        });
      if (blend_since_ < 0.0 || turning) {blend_since_ = now_s;}
      if (now_s - blend_since_ > cfg_.blend_stall_s) {blend_stalled_ = true;}
    } else {blend_since_ = -1.0;}
  } else {
    blend_since_ = -1.0;
  }

  // ③ 역기구학 (동시 모드: 모듈별 cos 대신 네 모듈 공통 배율 → 몸 방향이 비틀리지 않게)
  const bool uniform = blend_on && cfg_.cosine_scale && measured_ok;
  auto modules = kin_.inverse(out_, measured, uniform ? false : cfg_.cosine_scale, &command_feasible_, previous_targets_,
    prefer_outward);
  if (uniform && command_feasible_) {
    double s_min = 1.0;
    for (std::size_t i = 0; i < modules.size(); ++i) {
      s_min = std::min(s_min, std::max(0.0, std::cos(wrap_angle(modules[i].steer_angle - measured[i].steer_angle))));
    }
    for (auto & m : modules) {m.wheel_speed *= s_min;}
  }
  if (!command_feasible_) {
    out_ = {0.0,0.0,0.0}; applied_drive_ = false;
    return modules;  // 조향 보상 분기가 정지 명령을 다시 덮어쓰지 않게 함.
  }

  // A stale/invalid command may decelerate, but must never start a new steering pose.
  if (!fresh) {
    const bool aligned = steering_aligned(modules, measured, cfg_.steer_tolerance);
    if (!aligned) {out_ = {0.0, 0.0, 0.0};}
    for (std::size_t i = 0; i < modules.size() && i < measured.size(); ++i) {
      modules[i].steer_angle = measured[i].steer_angle;
      if (!aligned) {modules[i].wheel_speed = 0.0;}
    }
  } else if (std::hypot(out_.vx, out_.vy) >= kStopEps || std::fabs(out_.wz) >= kStopEps) {
    previous_targets_ = final_targets.empty() ? modules : final_targets;
  }

  // ③-2 동시 모드: 조향이 도는 만큼 바퀴 offset 이 굴리는 양을 보상 (roll_while_steering 과 같은 식)
  if (blend_on && fresh && cfg_.roll_while_steering) {
    for (std::size_t i = 0; i < modules.size() && i < measured.size(); ++i) {
      if (std::isfinite(measured[i].steer_rate)) {modules[i].wheel_speed += kin_.roll_while_steering(i, measured[i].steer_rate);}
    }
  }

  // ④ 조향 먼저 (동시 모드의 조향 정체도 여기서 같은 방식으로 구동을 세움)
  if (blend_stalled_ || (cfg_.steer_then_drive && !steering_aligned(modules, measured, cfg_.steer_tolerance))) {
    gate_blocked_ = true;
    for (std::size_t i = 0; i < modules.size(); ++i) {
      const double rate = i < measured.size() ? measured[i].steer_rate : 0.0;
      modules[i].wheel_speed = cfg_.roll_while_steering ? kin_.roll_while_steering(i, rate) : 0.0;
    }
    out_ = {0.0, 0.0, 0.0};   // 조향이 다 돌면 가속도 처음부터
  }
  if (cfg_.max_wheel_speed > 0.0) {
    double peak = 0.0;
    for (const auto & m : modules) {peak = std::max(peak, std::fabs(m.wheel_speed));}
    if (peak > cfg_.max_wheel_speed) {
      for (auto & m : modules) {m.wheel_speed *= cfg_.max_wheel_speed / peak;}
    }
  }
  if (std::hypot(out_.vx, out_.vy) >= kStopEps) {translating_ = true;}
  // 동시 모드: 게이트 판단·바퀴 속도는 위(진행 비율 각도)로 끝냄. 조향 모터에는 최종 각도 (정체로 구동을 세워도 조향은 계속)
  if (!final_targets.empty() && final_targets.size() == modules.size()) {
    for (std::size_t i = 0; i < modules.size(); ++i) {modules[i].steer_angle = final_targets[i].steer_angle;}
  }
  return modules;
}

bool SwerveDrive::update_park(double now_s, bool fresh_zero, const std::vector<ModuleState> & measured)
{
  if (!cfg_.park_on_idle || !fresh_zero || measured.size() != kin_.size() || !kin_.park_reachable()) {
    parking_ = false;
    idle_since_ = -1.0;
    return false;
  }
  if (parking_) {return true;}   // 시작한 뒤에는 바퀴가 보상으로 구르므로 "정지" 를 다시 보지 않음
  bool still = out_.vx == 0.0 && out_.vy == 0.0 && out_.wz == 0.0;
  for (const auto & m : measured) {
    if (!std::isfinite(m.wheel_speed) || std::fabs(m.wheel_speed) >= cfg_.park_still_wheel) {still = false;}
  }
  if (!still) {idle_since_ = -1.0; return false;}
  if (idle_since_ < 0.0) {idle_since_ = now_s;}
  parking_ = now_s - idle_since_ >= cfg_.park_idle_s;
  return parking_;
}

std::vector<ModuleState> SwerveDrive::stop_hold(const std::vector<double> & last_steer)
{
  parking_ = false;
  idle_since_ = -1.0;
  previous_targets_.clear();
  out_ = {0.0, 0.0, 0.0};
  translating_ = false;
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
  parking_ = false;
  idle_since_ = -1.0;
  previous_targets_.clear();
  out_ = {0.0, 0.0, 0.0};
  translating_ = false;
  have_cmd_ = false;
  holding_heading_ = false;
}

}  // namespace swerve_controller
