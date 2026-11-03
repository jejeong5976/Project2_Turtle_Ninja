// arm_kinematics.cpp — 실물 집기 계산기와 같은 팔 기구학 (설명은 include/manipulation/arm_kinematics.hpp)

#include "manipulation/arm_kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <random>

namespace manipulation
{

namespace
{
constexpr double kDeg = 180.0 / M_PI;
constexpr double kTiltFaceTol = 10.0 * M_PI / 180.0;   // 기울인 손: 닫힘축(접선)과 퍽 면 · 고른 방향 차이 허용
constexpr double kG = 9.81;

Eigen::Matrix4d axis_angle(const Eigen::Vector3d & axis, double a)
{
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = Eigen::AngleAxisd(a, axis.normalized()).toRotationMatrix();
  return T;
}

using Vec7 = Eigen::Matrix<double, 7, 1>;

// 잔차: 손끝 위치 [mm] 3 · 접근축 [°] 3 · 닫힘축 roll [°] 1 (period 마다 같음: 90° = 네모 칸 · 퍽, 180° = 손가락 방향을 정확히)
Vec7 residual(const ArmModel & m, const Q5 & q, const Eigen::Vector3d & p, const Eigen::Vector3d & a, const Eigen::Vector3d & c,
  double period)
{
  const Grip g = arm_grip(m, q);
  Vec7 r;
  r.segment<3>(0) = (g.tip - p) * 1000.0;
  r.segment<3>(3) = (g.app - a) * kDeg;
  Eigen::Vector3d cp = g.close - g.close.dot(a) * a;
  cp /= cp.norm() + 1e-12;
  Eigen::Vector3d ct = c - c.dot(a) * a;
  ct /= ct.norm() + 1e-12;
  const double ang = std::atan2(ct.cross(cp).dot(a), ct.dot(cp));
  const double k = period >= 179.0 ? 1.0 : 4.0;   // 180° 주기 = sin(ang) (0 · 180° 에서만 0), 90° = 기존 계산기와 같은 sin(4 ang)/4
  r(6) = std::sin(k * ang) / k * kDeg;
  return r;
}

IkSolution errors(const ArmModel & m, const Q5 & q, const Eigen::Vector3d & p, const Eigen::Vector3d & a, const Eigen::Vector3d & c,
  double period)
{
  const Grip g = arm_grip(m, q);
  Eigen::Vector3d cp = (g.close - g.close.dot(a) * a).normalized();
  Eigen::Vector3d ct = (c - c.dot(a) * a).normalized();
  double ang = std::atan2(ct.cross(cp).dot(a), ct.dot(cp)) * kDeg;
  ang = std::fmod(std::fmod(ang + period / 2.0, period) + period, period) - period / 2.0;
  return {q, (g.tip - p).norm() * 1000.0, std::acos(std::clamp(g.app.dot(a), -1.0, 1.0)) * kDeg, std::fabs(ang)};
}

Q5 clamp_q(const ArmModel & m, Q5 q, double eps)
{
  for (int j = 0; j < 5; ++j) {q[j] = std::clamp(q[j], m.lo[j] + eps, m.hi[j] - eps);}
  return q;
}

// 경계 안 Levenberg–Marquardt (수치 야코비안)
Q5 solve_lm(const ArmModel & m, Q5 x, const Eigen::Vector3d & p, const Eigen::Vector3d & a, const Eigen::Vector3d & c,
  double period)
{
  x = clamp_q(m, x, 1e-6);
  Vec7 r = residual(m, x, p, a, c, period);
  double cost = r.squaredNorm();
  double lambda = 1e-3;
  for (int it = 0; it < 400; ++it) {
    Eigen::Matrix<double, 7, 5> J;
    for (int j = 0; j < 5; ++j) {
      Q5 xp = x, xm = x;
      const double h = 1e-6;
      xp[j] += h;
      xm[j] -= h;
      J.col(j) = (residual(m, xp, p, a, c, period) - residual(m, xm, p, a, c, period)) / (2.0 * h);
    }
    const Eigen::Matrix<double, 5, 5> JtJ = J.transpose() * J;
    const Eigen::Matrix<double, 5, 1> g = J.transpose() * r;
    bool improved = false;
    while (lambda < 1e12) {
      Eigen::Matrix<double, 5, 5> A = JtJ;
      for (int j = 0; j < 5; ++j) {A(j, j) += lambda * (JtJ(j, j) + 1e-9);}
      const Eigen::Matrix<double, 5, 1> d = A.ldlt().solve(-g);
      Q5 xn = x;
      for (int j = 0; j < 5; ++j) {xn[j] += d(j);}
      xn = clamp_q(m, xn, 0.0);
      const Vec7 rn = residual(m, xn, p, a, c, period);
      const double cn = rn.squaredNorm();
      if (cn < cost) {
        double step = 0.0;
        for (int j = 0; j < 5; ++j) {step = std::max(step, std::fabs(xn[j] - x[j]));}
        const bool done = cost - cn < 1e-14 * std::max(1.0, cost) || step < 1e-12;
        x = xn;
        r = rn;
        cost = cn;
        lambda = std::max(lambda / 3.0, 1e-12);
        improved = true;
        if (done) {return x;}
        break;
      }
      lambda *= 4.0;
    }
    if (!improved || cost < 1e-16) {break;}
  }
  return x;
}
}  // namespace

std::array<Eigen::Matrix4d, 5> arm_frames(const ArmModel & m, const Q5 & q)
{
  std::array<Eigen::Matrix4d, 5> out;
  Eigen::Matrix4d T = m.T_base_armbase;
  for (int j = 0; j < 5; ++j) {
    T = T * m.joint_T[j] * axis_angle(m.axis[j], q[j]);
    out[j] = T;
  }
  return out;
}

Grip arm_grip(const ArmModel & m, const Q5 & q)
{
  const Eigen::Matrix4d T5 = arm_frames(m, q)[4];
  const Eigen::Matrix3d R = T5.block<3, 3>(0, 0);
  return {R * m.tip5 + T5.block<3, 1>(0, 3), R * m.app5, R * m.close5};
}

std::vector<IkSolution> arm_ik(
  const ArmModel & m, const Eigen::Vector3d & p, const Eigen::Vector3d & a, const Eigen::Vector3d & c,
  const std::vector<Q5> & seeds_in, double roll_period)
{
  std::vector<Q5> seeds = seeds_in;
  if (seeds.empty()) {
    const Q5 & q0 = m.template_q;
    for (int k = 0; k < 13; ++k) {
      const double j1 = m.lo[0] + (m.hi[0] - m.lo[0]) * k / 12.0;
      for (double j5 : {q0[4], q0[4] + M_PI / 2.0, q0[4] - M_PI / 2.0}) {seeds.push_back({j1, q0[1], q0[2], q0[3], j5});}
    }
    std::mt19937 rng(0);
    for (int k = 0; k < 24; ++k) {
      Q5 s;
      for (int j = 0; j < 5; ++j) {s[j] = std::uniform_real_distribution<double>(m.lo[j], m.hi[j])(rng);}
      seeds.push_back(s);
    }
  }
  std::vector<IkSolution> out;
  for (const auto & s : seeds) {out.push_back(errors(m, solve_lm(m, s, p, a, c, roll_period), p, a, c, roll_period));}
  return out;
}

Ticks q_to_ticks(const ArmModel & m, const Q5 & q)
{
  Ticks t;
  for (int j = 0; j < 5; ++j) {
    t[j] = static_cast<int>(std::lround(m.jm[j].ref_tick + (q[j] * kDeg - m.jm[j].q_deg_at_ref) / (m.jm[j].sign * 360.0 / 4096.0)));
  }
  return t;
}

Q5 ticks_to_q(const ArmModel & m, const Ticks & t)
{
  Q5 q;
  for (int j = 0; j < 5; ++j) {
    q[j] = (m.jm[j].q_deg_at_ref + m.jm[j].sign * (t[j] - m.jm[j].ref_tick) * 360.0 / 4096.0) / kDeg;
  }
  return q;
}

bool ticks_in_range(const ArmModel & m, const Ticks & t)
{
  for (int j = 0; j < 5; ++j) {
    if (t[j] < m.tick_range[j][0] || t[j] > m.tick_range[j][1]) {return false;}
  }
  return true;
}

std::vector<double> ticks_to_command(const ArmModel & m, const Ticks & t)
{
  std::vector<double> out(5);
  for (int j = 0; j < 5; ++j) {out[j] = (t[j] - m.zero_tick[j]) * 2.0 * M_PI / 4096.0;}
  return out;
}

std::array<double, 3> sag_torques(const ArmModel & m, const Q5 & q, double held_kg)
{
  const auto F = arm_frames(m, q);
  const Eigen::Vector3d o2 = F[1].block<3, 1>(0, 3), o3 = F[2].block<3, 1>(0, 3), o4 = F[3].block<3, 1>(0, 3);
  const Eigen::Vector3d p5 = F[4].block<3, 1>(0, 3);
  const Eigen::Vector3d tip = F[4].block<3, 3>(0, 0) * m.tip5 + p5;
  struct Mass {Eigen::Vector3d p; double kg; int from;};   // from = 이 질량을 버티는 첫 관절 (2 · 3 · 4)
  const Mass masses[] = {{(o2 + o3) / 2.0, 0.12, 2}, {(o3 + o4) / 2.0, 0.12, 3}, {(o4 + p5) / 2.0, 0.10, 4},
    {p5 + (tip - p5) / 3.0, 0.40, 4}, {tip, held_kg, 4}};
  const Eigen::Vector3d o[] = {o2, o3, o4};
  std::array<double, 3> tau{};
  for (int j = 2; j <= 4; ++j) {
    const Eigen::Vector3d ax = F[j - 1].block<3, 3>(0, 0) * m.axis[j - 1];
    double s = 0.0;
    for (const auto & ms : masses) {
      if (ms.kg <= 0.0 || j > ms.from) {continue;}
      s += (ms.p - o[j - 2]).cross(Eigen::Vector3d(0, 0, -ms.kg * kG)).dot(ax);
    }
    tau[j - 2] = s;
  }
  return tau;
}

Q5 encoder_goal_q(const ArmModel & m, const Q5 & q_true, double held_kg)
{
  const auto tau = sag_torques(m, q_true, held_kg);
  Q5 q = q_true;
  for (int j = 2; j <= 4; ++j) {q[j - 1] -= m.sag_c * tau[j - 2];}
  return q;
}

double default_close_rad(const ArmModel & m, double x, double y, double yaw_deg)
{
  const double az = std::atan2(y - m.j1_xy.y(), x - m.j1_xy.x());
  Eigen::Vector2d close(-std::sin(az), std::cos(az));
  if (std::isfinite(yaw_deg)) {   // 네모 퍽: 면 법선 4 개 중 기본 닫힘축에 가장 가까운 것
    Eigen::Vector2d best = close;
    double bd = -2.0;
    for (int k = 0; k < 4; ++k) {
      const double t = yaw_deg / kDeg + k * M_PI / 2.0;
      const Eigen::Vector2d n(std::cos(t), std::sin(t));
      if (n.dot(close) > bd) {bd = n.dot(close); best = n;}
    }
    close = best;
  }
  return std::atan2(close.y(), close.x());
}

std::vector<PickPose> pick_poses(
  const ArmModel & m, double x, double y, double top_z, double yaw_deg, const std::vector<double> & heights_mm,
  std::string & why, double close_deg, double max_tilt_deg, double prefer_tilt_deg, double max_dev_deg)
{
  const double ca = std::isfinite(close_deg) ? close_deg / kDeg : default_close_rad(m, x, y, yaw_deg);
  // 5 축 팔은 손을 J1 → 퍽 수직면 안으로만 기울일 수 있음 → 기울인 손은 닫힘축 = 접선, 기울임 = 바깥 반지름 방향
  //   닫힘축이 접선에서 kTiltFaceTol 넘게 다르면 (면이 틀어진 네모 · 이웃 때문에 고른 방향) 기울이지 않음. 안이면 손가락이 닫히며 퍽을 면에 맞춤
  const double az = std::atan2(y - m.j1_xy.y(), x - m.j1_xy.x());
  const double tangential = az + M_PI / 2.0;
  const double period = std::isfinite(close_deg) ? M_PI : M_PI / 2.0;   // 고른 방향 = 손가락 방향 (180°), 네모 면 = 90° 마다 같음
  const bool tilt_ok = std::fabs(std::remainder(ca - tangential, period)) <= kTiltFaceTol + 1e-9;
  const Eigen::Vector3d radial(std::cos(az), std::sin(az), 0.0);
  const Q5 & q0 = m.template_q;
  std::vector<double> tilts;   // 선호 기울임에 가까운 순 (같으면 작은 것 먼저)
  for (double t = 0.0; t <= max_tilt_deg + 1e-9; t += 10.0) {
    if (std::fabs(t - prefer_tilt_deg) <= max_dev_deg + 1e-9 && (t == 0.0 || tilt_ok)) {tilts.push_back(t);}
  }
  std::stable_sort(tilts.begin(), tilts.end(), [&](double a, double b) {
      return std::fabs(a - prefer_tilt_deg) < std::fabs(b - prefer_tilt_deg) - 1e-9;
    });
  if (tilts.empty()) {why = "쥔 퍽 기울임에 맞는 손 기울임 없음"; return {};}
  for (double tilt : tilts) {
    const double t = tilt / kDeg;
    const double cc = tilt > 0.0 ? tangential : ca;
    const Eigen::Vector3d close(std::cos(cc), std::sin(cc), 0.0);
    const Eigen::Vector3d app = std::sin(t) * radial - std::cos(t) * Eigen::Vector3d::UnitZ();
    std::vector<PickPose> out;
    std::vector<Q5> seeds;
    bool ok = true;
    for (double h : heights_mm) {
      const Eigen::Vector3d p(x, y, top_z + h / 1000.0);   // 손끝 = 퍽 중심축 위 (기울여도 수직으로 들고 내림)
      const auto sols = arm_ik(m, p, app, close, seeds, std::isfinite(close_deg) || tilt > 0.0 ? 180.0 : 90.0);
      const IkSolution * pick = nullptr;
      auto key = [&](const IkSolution & s) {
          const double a = std::round(std::fabs(s.q[4] - m.q5_rest) / 0.3);
          double dd = 0.0;
          for (int j = 1; j < 4; ++j) {dd += (s.q[j] - q0[j]) * (s.q[j] - q0[j]);}
          return std::make_pair(a, std::sqrt(dd));
        };
      for (const auto & s : sols) {
        if (s.pos_mm < 1.0 && s.app_deg < 1.0 && s.roll_deg < 2.0 && ticks_in_range(m, q_to_ticks(m, s.q)) &&
          ticks_in_range(m, q_to_ticks(m, encoder_goal_q(m, s.q, 0.0))))   // 명령(처짐 앞먹임)도 범위 안
        {
          if (!pick || key(s) < key(*pick)) {pick = &s;}
        }
      }
      if (!pick) {
        char buf[200];
        std::snprintf(buf, sizeof(buf), "높이 %.0f mm: 손 IK 안 됨 (퍽 %.3f, %.3f, 돌림 %.1f°, 기울임 %.0f° 까지) — 퍽이 팔에서 너무 멀거나 옆",
          h, x, y, yaw_deg, max_tilt_deg);
        why = buf;
        ok = false;
        break;
      }
      char name[16];
      std::snprintf(name, sizeof(name), "h%.0f", h);
      out.push_back({name, q_to_ticks(m, pick->q), q_to_ticks(m, encoder_goal_q(m, pick->q, 0.0)), tilt});
      seeds = {pick->q};
    }
    if (ok) {
      why.clear();
      return out;
    }
  }
  return {};
}

}  // namespace manipulation
