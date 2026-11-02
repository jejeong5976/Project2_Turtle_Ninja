// arm_state.cpp — 표시 · TF 용 팔 상태 계산 (설명은 include/manipulation/arm_state.hpp)

#include "manipulation/arm_state.hpp"

#include <cmath>

namespace manipulation
{

namespace
{
constexpr double kT2R = 2.0 * M_PI / 4096.0;
constexpr double kDeg = 180.0 / M_PI;

// 반올림 없는 틱 ↔ 모델 관절 (arm_kinematics 의 q_to_ticks · ticks_to_q 와 같은 식)
double tick_of_team(const ArmModel & m, int j, double rad) {return m.zero_tick[j] + rad / kT2R;}
double team_of_tick(const ArmModel & m, int j, double tick) {return (tick - m.zero_tick[j]) * kT2R;}
double q_of_tick(const ArmModel & m, int j, double tick)
{
  return (m.jm[j].q_deg_at_ref + m.jm[j].sign * (tick - m.jm[j].ref_tick) * 360.0 / 4096.0) / kDeg;
}
double tick_of_q(const ArmModel & m, int j, double q)
{
  return m.jm[j].ref_tick + (q * kDeg - m.jm[j].q_deg_at_ref) / (m.jm[j].sign * 360.0 / 4096.0);
}
}  // namespace

std::array<double, 5> sag_corrected(const ArmModel & m, const std::array<double, 5> & team_rad, double held_kg, double weight)
{
  for (double v : team_rad) {
    if (!std::isfinite(v)) {return team_rad;}
  }
  Q5 q_enc;
  for (int j = 0; j < 5; ++j) {q_enc[j] = q_of_tick(m, j, tick_of_team(m, j, team_rad[j]));}
  Q5 q_true = q_enc;
  for (int it = 0; it < 4; ++it) {   // 실제 = 엔코더 + c · τ(실제): 고정점 반복 (c · τ 가 작아 몇 번이면 수렴)
    const auto tau = sag_torques(m, q_true, held_kg);
    for (int j = 2; j <= 4; ++j) {q_true[j - 1] = q_enc[j - 1] + weight * m.sag_c * tau[j - 2];}
  }
  std::array<double, 5> out;
  for (int j = 0; j < 5; ++j) {out[j] = team_of_tick(m, j, tick_of_q(m, j, q_true[j]));}
  return out;
}

double sag_weight(const std::array<double, 5> & team_rad, const std::array<double, 5> & rest, double near, double far)
{
  double d = 0.0;
  for (int j = 1; j <= 3; ++j) {d = std::max(d, std::fabs(team_rad[j] - rest[j]));}
  if (!std::isfinite(d) || d <= near) {return 0.0;}
  if (d >= far || far <= near) {return 1.0;}
  return (d - near) / (far - near);
}

namespace
{
Eigen::Vector3d rotate(const Eigen::Vector3d & v, const Eigen::Vector3d & axis, double a)
{
  return v * std::cos(a) + axis.cross(v) * std::sin(a) + axis * axis.dot(v) * (1.0 - std::cos(a));
}
}  // namespace

bool solve_linkage(const GripperLinkage & g, double q, std::array<double, 3> & out)
{
  if (!std::isfinite(q) || q < g.q_min - 1e-12 || q > g.q_max + 1e-12) {return false;}
  const Eigen::Vector3d f = g.finger_pin_q0[0] - g.center + g.slide * q;
  const double x = f.dot(g.e), y = f.dot(g.t);
  const double d = std::hypot(x, y);
  if (d < 1e-12) {return false;}
  const double r = g.crank_radius, L = g.rod_length;
  const double c = (d * d + r * r - L * L) / (2.0 * r * d);
  if (std::fabs(c) > 1.0 + 1e-10) {return false;}
  const double theta = std::atan2(y, x) + std::acos(std::max(-1.0, std::min(1.0, c))) - g.cad_pin_alpha;
  out[0] = theta - g.theta0;
  for (int s = 0; s < 2; ++s) {
    const double sign = s == 0 ? 1.0 : -1.0;
    const Eigen::Vector3d pin = g.center + rotate(g.crank_pin_cad[s] - g.center, g.axis, theta);
    const Eigen::Vector3d end = g.finger_pin_q0[s] + sign * g.slide * q;
    Eigen::Vector3d u = end - pin;
    Eigen::Vector3d u0 = g.finger_pin_cad[s] - g.crank_pin_cad[s];
    u -= g.axis * u.dot(g.axis);
    u0 -= g.axis * u0.dot(g.axis);
    const double absolute = std::atan2(g.axis.dot(u0.cross(u)), u0.dot(u));
    out[1 + s] = absolute - theta - g.phi0[s];
  }
  return true;
}

}  // namespace manipulation
