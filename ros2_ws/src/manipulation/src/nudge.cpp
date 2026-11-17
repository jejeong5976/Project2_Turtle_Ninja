// nudge.cpp — 칸 안 퍽 툭 건드리기 순수 계산 (설명은 include/manipulation/nudge.hpp)

#include "manipulation/nudge.hpp"

#include <algorithm>
#include <cstdio>
#include <cstdlib>

#include "manipulation/arm_path.hpp"

namespace manipulation
{

namespace
{
constexpr double kD2R = M_PI / 180.0;
}

Eigen::Vector3d nudge_target(const SeatCellRef & c, double puck_h)
{
  return Eigen::Vector3d(c.xy.x(), c.xy.y(), c.top_z - puck_h / 2.0);
}

NudgePuck nudge_puck_manual(const SeatCellRef & c, double puck_h, double half_w, double tilt_deg, double face_deg,
  double dx_mm, double dy_mm, double dir_deg, double dz_mm)
{
  NudgePuck p;
  const double th = (std::isfinite(tilt_deg) ? tilt_deg : 0.0) * kD2R;
  dx_mm = std::isfinite(dx_mm) ? dx_mm : 0.0;
  dy_mm = std::isfinite(dy_mm) ? dy_mm : 0.0;
  double dir = c.yaw;
  if (std::isfinite(dir_deg)) {dir = dir_deg * kD2R;}
  else if (std::hypot(dx_mm, dy_mm) > 0.1) {dir = std::atan2(dy_mm, dx_mm);}
  p.up = Eigen::Vector3d(std::sin(th) * std::cos(dir), std::sin(th) * std::sin(dir), std::cos(th));
  const double dz = std::isfinite(dz_mm) ? dz_mm / 1000.0 : half_w * std::sin(std::fabs(th)) - puck_h / 2.0 * (1.0 - std::cos(th));
  const Eigen::Vector3d t = nudge_target(c, puck_h);
  p.centre = Eigen::Vector3d(t.x() + dx_mm / 1000.0, t.y() + dy_mm / 1000.0, t.z() + dz);
  p.face_rad = (std::isfinite(face_deg) ? face_deg : 0.0) * kD2R;
  p.source = "manual";
  return p;
}

bool nudge_puck_follow(NudgePuck & p, const NudgePuck & p0, const Eigen::Vector3d & tip, const SeatCellRef & c, double puck_h, double reach_m)
{
  const Eigen::Vector3d tgt = nudge_target(c, puck_h);
  const double dz0 = p0.centre.z() - tgt.z();
  if (!(dz0 > 1e-4) || !tip.allFinite()) {return false;}
  if ((tip.head<2>() - p.centre.head<2>()).norm() > reach_m) {return false;}
  const double up_z = std::clamp(p.up.normalized().z(), 0.0, 1.0);
  const double top = p.centre.z() + puck_h / 2.0 * up_z;
  if (tip.z() >= top) {return false;}
  const double dz = std::clamp(tip.z() - puck_h / 2.0 * up_z - tgt.z(), 0.0, p.centre.z() - tgt.z());
  const double r = std::clamp(dz / dz0, 0.0, 1.0);
  const double th0 = std::acos(std::clamp(p0.up.normalized().z(), -1.0, 1.0));
  const double az = std::atan2(p0.up.y(), p0.up.x());
  const double th = th0 * r;
  p.up = Eigen::Vector3d(std::sin(th) * std::cos(az), std::sin(th) * std::sin(az), std::cos(th));
  p.centre.head<2>() = tgt.head<2>() + (p0.centre.head<2>() - tgt.head<2>()) * r;
  p.centre.z() = tgt.z() + dz;
  return true;
}

NudgePuck nudge_puck_from_seat(const SeatCellRef & c, double puck_h, double x_mm, double y_mm, double height_mm,
  const Eigen::Matrix3d & R_cell, double face_deg)
{
  NudgePuck p;
  const Eigen::Matrix3d Rz = Eigen::AngleAxisd(c.yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Vector3d top(c.xy.x(), c.xy.y(), c.top_z);
  const Eigen::Vector3d p_top = top + Rz * Eigen::Vector3d(x_mm / 1000.0, y_mm / 1000.0, height_mm / 1000.0);
  Eigen::Vector3d up = Rz * R_cell.col(2);
  if (!(up.norm() > 1e-6)) {up = Eigen::Vector3d::UnitZ();}
  up.normalize();
  if (up.z() < 0.0) {up = -up;}
  p.up = up;
  p.centre = p_top - up * puck_h / 2.0;
  p.face_rad = (std::isfinite(face_deg) ? face_deg : 0.0) * kD2R;
  p.source = "seat";
  return p;
}

std::vector<double> nudge_obs(const NudgeObsIn & in)
{
  std::vector<double> o;
  o.reserve(kNudgeObsDim);
  for (int j = 0; j < 5; ++j) {o.push_back(in.q[j] - in.home[j]);}                 // 0–4
  for (int j = 0; j < 5; ++j) {o.push_back(in.qd[j] * 0.1);}                       // 5–9
  o.push_back(std::clamp(in.grip_open, 0.0, 1.0));                                 // 10
  o.push_back(in.grip_force_n / 20.0);                                             // 11
  o.push_back(in.tip.x());                                                         // 12–14
  o.push_back(in.tip.y());
  o.push_back(in.tip.z() + in.tip_z_offset);
  for (int k = 0; k < 3; ++k) {o.push_back(in.wrist_z[k]);}                        // 15–17
  for (int k = 0; k < 3; ++k) {o.push_back(in.wrist_y[k]);}                        // 18–20
  const Eigen::Vector3d tgt = nudge_target(in.cell, in.puck_h);
  const Eigen::Vector3d pc = in.puck.centre;
  const Eigen::Vector3d up = in.puck.up.normalized();
  for (int k = 0; k < 3; ++k) {o.push_back(pc[k] - in.tip[k]);}                    // 21–23
  for (int k = 0; k < 3; ++k) {o.push_back(up[k]);}                                // 24–26
  for (int k = 0; k < 3; ++k) {o.push_back(tgt[k] - pc[k]);}                       // 27–29
  for (int k = 0; k < 3; ++k) {o.push_back(tgt[k] - in.tip[k]);}                   // 30–32
  o.push_back(1.0);                                                                // 33–34 store
  o.push_back(0.0);
  for (int k = 0; k < 12; ++k) {o.push_back(in.occupied[k] > 0.5 ? 1.0 : 0.0);}   // 35–46
  for (int k = 0; k < 12; ++k) {o.push_back(k == in.task_index ? 1.0 : 0.0);}      // 47–58
  for (int k = 0; k < 12; ++k) {o.push_back(0.0);}                                 // 59–70
  o.push_back(0.0);                                                                // 71–72
  o.push_back(0.0);
  for (int k = 0; k < 6; ++k) {o.push_back(in.prev_action[k]);}                    // 73–78
  o.push_back(std::max(0.0, 1.0 - static_cast<double>(in.step) / kNudgeSimSteps)); // 79
  o.push_back(std::sin(4.0 * in.puck.face_rad));                                   // 80–81
  o.push_back(std::cos(4.0 * in.puck.face_rad));
  o.push_back((pc.z() - tgt.z()) * 100.0);                                         // 82
  o.push_back(std::acos(std::clamp(up.z(), -1.0, 1.0)) * 180.0 / M_PI / 20.0);    // 83
  o.push_back((pc.x() - in.cell.xy.x()) * 100.0);                                  // 84–85
  o.push_back((pc.y() - in.cell.xy.y()) * 100.0);
  return o;
}

const char * nudge_end_name(NudgeEnd e)
{
  switch (e) {
    case NudgeEnd::NONE: return "진행";
    case NudgeEnd::IDLE: return "정책 멈춤";
    case NudgeEnd::MAX_STEPS: return "걸음 다 씀";
    case NudgeEnd::SAFETY: return "안전 멈춤";
    case NudgeEnd::LEFT: return "정책이 칸을 떠남";
  }
  return "?";
}

void NudgeRunner::start(const NudgeConfig & c, const std::array<double, 5> & q0, const std::array<double, 5> & effort0, NudgeTipFn tip)
{
  c_ = c;
  for (int j = 0; j < 5; ++j) {   // 시작 실측이 한계 끝을 조금 넘어도 (기록 경로 끝 · 처짐) 첫 걸음에 멈추지 않게 — 그보다 더 밖으론 안 감
    if (std::isfinite(q0[j])) {
      c_.lo[j] = std::min(c_.lo[j], q0[j] - 1e-6);
      c_.hi[j] = std::max(c_.hi[j], q0[j] + 1e-6);
    }
  }
  tip_ = std::move(tip);
  target_ = q0;
  effort0_ = effort0;
  prev_.fill(0.0);
  steps_ = idle_ = cur_over_ = 0;
  min_tip_z_ = 1e9;
}

std::string NudgeRunner::target_ok(const std::array<double, 5> & t) const
{
  char b[200];
  for (int j = 0; j < 5; ++j) {
    if (!std::isfinite(t[j]) || t[j] < c_.lo[j] || t[j] > c_.hi[j]) {
      std::snprintf(b, sizeof(b), "J%d %.3f rad 이 한계 [%.3f, %.3f] 밖", j + 1, t[j], c_.lo[j], c_.hi[j]);
      return b;
    }
  }
  if (!tip_) {return "";}
  const Eigen::Vector3d p = tip_(t);
  const double r = (p.head<2>() - c_.cell_xy).norm();
  if (!p.allFinite() || r > c_.box_xy_m) {
    std::snprintf(b, sizeof(b), "손끝이 칸 중심에서 수평 %.1f mm (한도 %.0f)", r * 1000.0, c_.box_xy_m * 1000.0);
    return b;
  }
  if (p.z() < c_.top_z - c_.below_top_m) {
    std::snprintf(b, sizeof(b), "손끝이 바로 앉은 윗면 %+.1f mm (한도 −%.0f)", (p.z() - c_.top_z) * 1000.0, c_.below_top_m * 1000.0);
    return b;
  }
  if (p.z() > c_.top_z + c_.above_top_m) {
    std::snprintf(b, sizeof(b), "손끝이 바로 앉은 윗면 %+.1f mm (한도 +%.0f)", (p.z() - c_.top_z) * 1000.0, c_.above_top_m * 1000.0);
    return b;
  }
  return "";
}

std::string NudgeRunner::check(const NudgeSense & s)
{
  std::string w = target_ok(s.q);
  if (!w.empty()) {return "실측 " + w;}
  char b[200];
  if (c_.max_lag_rad > 0.0) {
    for (int j = 0; j < 5; ++j) {
      if (std::fabs(target_[j] - s.q[j]) > c_.max_lag_rad) {
        std::snprintf(b, sizeof(b), "J%d 목표 − 실측 %.3f rad (한도 %.2f, 막힘)", j + 1, target_[j] - s.q[j], c_.max_lag_rad);
        return b;
      }
    }
  }
  if (c_.current_a > 0.0) {
    double worst = 0.0;
    int wj = -1;
    for (int j = 1; j <= 3; ++j) {
      const double d = std::fabs(s.effort[j] - effort0_[j]);
      if (std::isfinite(d) && d > worst) {worst = d; wj = j;}
    }
    cur_over_ = worst > c_.current_a ? cur_over_ + 1 : 0;
    if (cur_over_ >= std::max(1, c_.current_hold)) {
      std::snprintf(b, sizeof(b), "J%d 전류 변화 %.2f A (문턱 %.2f, %d 걸음 이어짐)", wj + 1, worst, c_.current_a, cur_over_);
      return b;
    }
  }
  if (tip_) {min_tip_z_ = std::min(min_tip_z_, tip_(s.q).z());}
  return "";
}

NudgeStepOut NudgeRunner::step(const NudgeSense & s, const std::vector<double> & action)
{
  NudgeStepOut out;
  out.target = target_;
  std::string w = check(s);
  if (w.empty() && action.size() < 5) {w = "정책 출력이 " + std::to_string(action.size()) + " 개";}
  if (!w.empty()) {
    out.end = NudgeEnd::SAFETY;
    out.why = w;
    out.target = s.q;
    target_ = s.q;
    return out;
  }
  std::array<double, 6> a{};
  for (std::size_t k = 0; k < 6 && k < action.size(); ++k) {a[k] = std::isfinite(action[k]) ? std::clamp(action[k], -1.0, 1.0) : 0.0;}
  std::array<double, 5> next = target_;
  const double gain = c_.speed_rad_s * c_.speed_scale * c_.dt_s;
  double amax = 0.0;
  for (int j = 0; j < 5; ++j) {
    next[j] += a[j] * gain;
    amax = std::max(amax, std::fabs(a[j]));
  }
  w = target_ok(next);
  if (!w.empty() && tip_) {   // 손끝이 윗면 + idle_clear 위에서 수평 · 위 상자 밖으로 = 일 끝내고 떠남 (지금 목표에서 멈춤, 보통 끝)
    const Eigen::Vector3d p0 = tip_(target_), p1 = tip_(next);
    bool limits_ok = true;
    for (int j = 0; j < 5; ++j) {limits_ok = limits_ok && next[j] >= c_.lo[j] && next[j] <= c_.hi[j];}
    if (limits_ok && p0.z() >= c_.top_z + c_.idle_clear_m && p1.z() >= c_.top_z + c_.idle_clear_m && steps_ > 0) {
      out.end = NudgeEnd::LEFT;
      out.why = "손끝 윗면 " + std::to_string(static_cast<int>(std::lround((p0.z() - c_.top_z) * 1000.0))) + " mm 위에서 " + w;
      out.target = target_;
      return out;
    }
  }
  if (!w.empty()) {
    out.end = NudgeEnd::SAFETY;
    out.why = "다음 목표 " + w;
    out.target = s.q;
    target_ = s.q;
    return out;
  }
  prev_ = a;
  target_ = next;
  ++steps_;
  out.target = next;
  const double tz = tip_ ? tip_(next).z() : 1e9;
  idle_ = (amax < c_.idle_action && tz >= c_.top_z + c_.idle_clear_m) ? idle_ + 1 : 0;
  if (idle_ >= std::max(1, c_.idle_steps) && steps_ >= c_.min_steps) {
    out.end = NudgeEnd::IDLE;
    out.why = "정책 행동이 " + std::to_string(idle_) + " 걸음 작음 (손끝 윗면 위)";
  } else if (steps_ >= c_.max_steps) {
    out.end = NudgeEnd::MAX_STEPS;
    out.why = std::to_string(steps_) + " 걸음";
  }
  return out;
}

void nudge_hold_trace(std::vector<std::vector<double>> & trace, const std::array<double, 5> & q_meas, std::size_t keep_before)
{
  trace_append(trace, std::vector<double>(q_meas.begin(), q_meas.end()), 1e-3, 4000, keep_before);
}

std::string nudge_gate(const NudgeGateIn & in)
{
  if (!in.enabled) {return "NUDGE 꺼짐 (nudge_enable false) — 켜기: ros2 param set /control_master nudge_enable true";}
  if (in.carrying) {return "NUDGE 는 빈 손으로만 (퍽을 쥔 채)";}
  if (!in.slot_known) {return "NUDGE 칸 모름 (S1 … S12)";}
  if (!in.model_ok) {return "NUDGE: 팔 기구학 (arm_model) 없음";}
  if (!in.policy_ok) {return "NUDGE 정책 못 읽음: " + in.policy_why + " (nudge_policy_path)";}
  return "";
}

bool parse_nudge_manual(const std::string & s, NudgeManual & out, std::string & why)
{
  out = NudgeManual{};
  const std::string tag(kNudgeManualTag);
  if (s.compare(0, tag.size(), tag) != 0) {return true;}
  std::vector<double> v;
  std::size_t p = tag.size();
  while (p <= s.size()) {
    const std::size_t e = std::min(s.find(',', p), s.size());
    const std::string t = s.substr(p, e - p);
    char * end = nullptr;
    const double x = std::strtod(t.c_str(), &end);
    if (t.empty() || end == t.c_str() || *end != '\0' || !std::isfinite(x)) {why = "NUDGE 손 값 숫자 아님: '" + t + "' (" + s + ")"; return false;}
    v.push_back(x);
    p = e + 1;
  }
  if (v.size() < 4 || v.size() > 6) {why = "NUDGE 손 값은 기울기 면 dx dy [방향 dz] (4–6 개): " + s; return false;}
  if (std::fabs(v[0]) > 60.0) {why = "NUDGE 기울기 " + std::to_string(v[0]) + "° (60° 넘음)"; return false;}
  out.given = true;
  out.tilt_deg = v[0];
  out.face_deg = v[1];
  out.dx_mm = v[2];
  out.dy_mm = v[3];
  if (v.size() > 4) {out.dir_deg = v[4];}
  if (v.size() > 5) {out.dz_mm = v[5];}
  return true;
}

}  // namespace manipulation
