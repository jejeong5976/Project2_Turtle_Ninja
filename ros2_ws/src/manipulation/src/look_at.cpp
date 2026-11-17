// look_at.cpp — 손목 카메라로 한 점 보기 LOOK_AT, 설명은 look_at.hpp

#include "manipulation/look_at.hpp"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <sstream>

namespace manipulation
{

namespace
{
constexpr double kDeg = M_PI / 180.0;

double wrap(double a) {return std::atan2(std::sin(a), std::cos(a));}

Eigen::Vector3d view_dir(const ArmModel & m, const LookAtGoal & g)
{
  const double yaw = std::isfinite(g.view_yaw) ? g.view_yaw : std::atan2(g.target.y() - m.j1_xy.y(), g.target.x() - m.j1_xy.x());
  const double pitch = std::isfinite(g.view_pitch) ? g.view_pitch : 0.0;
  return {std::cos(pitch) * std::cos(yaw), std::cos(pitch) * std::sin(yaw), -std::sin(pitch)};
}

double goal_standoff(const LookAtGoal & g, const LookAtParams & p)
{
  return std::isfinite(g.standoff_m) && g.standoff_m > 0.0 ? g.standoff_m : p.standoff_m;
}

bool in_box(const Eigen::Vector3d & x, const std::array<double, 5> & b, double pad)
{
  return x.x() >= b[0] - pad && x.x() <= b[1] + pad && x.y() >= b[2] - pad && x.y() <= b[3] + pad && x.z() <= b[4] + pad;
}

bool ticks_ok(const ArmModel & m, const Q5 & q)
{
  return ticks_in_range(m, q_to_ticks(m, encoder_goal_q(m, q, 0.0)));
}

// 관절 범위 = 모델 [lo, hi] ∩ 틱 범위 (tick_range, 처짐 앞먹임 몫 0.03 rad 안쪽)
std::array<std::array<double, 2>, 5> q_limits(const ArmModel & m)
{
  std::array<std::array<double, 2>, 5> L;
  for (int j = 0; j < 5; ++j) {
    double lo = m.lo[j], hi = m.hi[j];
    const auto & tr = m.tick_range[j];
    if (tr[1] > tr[0]) {
      const double a = (m.jm[j].q_deg_at_ref + m.jm[j].sign * (tr[0] - m.jm[j].ref_tick) * 360.0 / 4096.0) * kDeg;
      const double b = (m.jm[j].q_deg_at_ref + m.jm[j].sign * (tr[1] - m.jm[j].ref_tick) * 360.0 / 4096.0) * kDeg;
      lo = std::max(lo, std::min(a, b) + 0.03);
      hi = std::min(hi, std::max(a, b) - 0.03);
    }
    L[j] = {lo, hi};
  }
  return L;
}

bool limits_ok(const ArmModel & m, const Q5 & q)
{
  const auto L = q_limits(m);
  for (int j = 0; j < 5; ++j) {if (q[j] < L[j][0] - 1e-9 || q[j] > L[j][1] + 1e-9) {return false;}}
  return true;
}

Q5 clamp_q(const ArmModel & m, Q5 q)
{
  const auto L = q_limits(m);
  for (int j = 0; j < 5; ++j) {q[j] = std::clamp(q[j], L[j][0], std::max(L[j][0], L[j][1]));}
  return q;
}

// 잔차 (look_at.hpp 계산)
// 광각 → 신호등 (또는 바코드 둘레) 시야 원뿔: 팔 점이 들어간 만큼 [rad] (반지름 clear_pad 포함)
Eigen::Vector3d light_dir(const LookAtGoal & g)
{
  return {std::cos(g.light_pitch) * std::cos(g.light_yaw), std::cos(g.light_pitch) * std::sin(g.light_yaw), std::sin(g.light_pitch)};
}

// 목표가 화면 (cx + du, cy + dv) 에 오게 하는 광선 (base_link). du = dv = 0 이면 광축
Eigen::Vector3d aim_axis(const Eigen::Isometry3d & cam, const LookAtParams & p)
{
  if (p.aim_du_px == 0.0 && p.aim_dv_px == 0.0) {return cam.linear().col(2);}
  return (cam.linear() * Eigen::Vector3d(p.aim_du_px / p.fx, p.aim_dv_px / p.fy, 1.0)).normalized();
}

// 목표 벽 면 법선 (로봇 쪽, 가로): 보는 방향 가로 성분의 반대
Eigen::Vector3d wall_normal(const ArmModel & m, const LookAtGoal & g)
{
  const Eigen::Vector3d v = view_dir(m, g);
  Eigen::Vector3d n(-v.x(), -v.y(), 0.0);
  const double k = n.norm();
  return k > 1e-6 ? Eigen::Vector3d(n / k) : Eigen::Vector3d(-1.0, 0.0, 0.0);
}

double cone_margin_rad(const LookAtGoal & g, const LookAtParams & p)
{
  return (std::isfinite(g.light_cone_deg) && g.light_cone_deg > 0.0 ? g.light_cone_deg : p.occl_margin_deg) * kDeg;
}

Eigen::Matrix<double, 26, 1> residual(const ArmModel & m, const Eigen::Isometry3d & T5c, const LookAtGoal & g,
  const LookAtParams & p, const Eigen::Vector3d & v, double d, const Q5 & q, const Q5 & seed, const Eigen::Vector3d & wide)
{
  const auto cam = look_camera_pose(m, T5c, q);
  const Eigen::Vector3d c = cam.translation(), z = cam.linear().col(2);
  const Eigen::Vector3d dv = g.target - c;
  const double dist = std::max(dv.norm(), 1e-6);
  const Eigen::Vector3d u = dv / dist;
  Eigen::Matrix<double, 26, 1> r = Eigen::Matrix<double, 26, 1>::Zero();
  if (p.wall_margin_m > 0.0) {   // 손끝 · 카메라를 벽 면에서 떼기
    const Eigen::Vector3d nw = wall_normal(m, g);
    const auto pts = look_arm_points(m, T5c, q);
    const double st = (pts[6] - g.target).dot(nw), sc = (pts[7] - g.target).dot(nw);
    if (st < p.wall_margin_m) {r(24) = p.w_wall * (p.wall_margin_m - st) / 0.02;}
    if (sc < p.wall_margin_m) {r(25) = p.w_wall * (p.wall_margin_m - sc) / 0.02;}
  }
  if (p.w_tau > 0.0) {r(23) = p.w_tau * sag_torques(m, q, 0.0)[0];}   // J2 토크 작게 = 팔을 몸 쪽으로
  const Eigen::Vector3d za = aim_axis(cam, p);   // 목표가 놓일 화소의 광선
  r.segment<3>(0) = p.w_axis * za.cross(u);
  r(3) = p.w_axis * (1.0 - za.dot(u));
  r(4) = p.w_dist * (dist - d) / 0.1;
  r.segment<3>(5) = p.w_obl * z.cross(v);
  r(8) = p.w_obl * (1.0 - z.dot(v));
  r(9) = p.w_roll * look_roll(cam, p.roll_period_deg * kDeg);
  for (int j = 0; j < 5; ++j) {r(10 + j) = p.w_move * (q[j] - seed[j]);}
  if (g.has_light) {   // 시야 원뿔 밖으로 밀기 (원뿔 안 깊이만큼)
    const Eigen::Vector3d l = light_dir(g);
    const double cone = cone_margin_rad(g, p);
    const auto pts = look_arm_points(m, T5c, q);
    for (std::size_t i = 0; i < pts.size() && i < 8; ++i) {
      const Eigen::Vector3d w = pts[i] - wide;
      const double along = w.dot(l);
      if (along <= 0.0) {continue;}
      const double ang = std::atan2(w.cross(l).norm(), along) - std::atan2(p.clear_pad_m, w.norm());
      if (ang < cone) {r(15 + static_cast<int>(i)) = p.w_occl * (cone - ang);}
    }
  }
  return r;
}

Q5 lm(const ArmModel & m, const Eigen::Isometry3d & T5c, const LookAtGoal & g, const LookAtParams & p,
  const Eigen::Vector3d & v, double d, Q5 q, double & cost, const Eigen::Vector3d & wide)
{
  const Q5 seed = q;
  auto r = residual(m, T5c, g, p, v, d, q, seed, wide);
  cost = r.squaredNorm();
  double lambda = 1e-2;
  for (int it = 0; it < p.max_iter; ++it) {
    Eigen::Matrix<double, 26, 5> J;
    for (int j = 0; j < 5; ++j) {
      Q5 qh = q;
      const double h = 1e-6;
      qh[j] += h;
      J.col(j) = (residual(m, T5c, g, p, v, d, qh, seed, wide) - r) / h;
    }
    const Eigen::Matrix<double, 5, 5> A = J.transpose() * J;
    const Eigen::Matrix<double, 5, 1> b = J.transpose() * r;
    bool improved = false;
    for (int k = 0; k < 8 && !improved; ++k) {
      Eigen::Matrix<double, 5, 5> Al = A;
      for (int j = 0; j < 5; ++j) {Al(j, j) += lambda * (A(j, j) + 1e-6);}
      const Eigen::Matrix<double, 5, 1> dq = -Al.ldlt().solve(b);
      Q5 qn = q;
      for (int j = 0; j < 5; ++j) {qn[j] += std::clamp(dq(j), -0.3, 0.3);}
      qn = clamp_q(m, qn);
      const auto rn = residual(m, T5c, g, p, v, d, qn, seed, wide);
      const double cn = rn.squaredNorm();
      if (cn < cost) {
        const double gain = cost - cn;
        q = qn;
        r = rn;
        cost = cn;
        lambda = std::max(lambda / 3.0, 1e-7);
        improved = true;
        if (gain < 1e-12) {return q;}
      } else {
        lambda *= 4.0;
      }
    }
    if (!improved) {break;}
  }
  return q;
}

std::string fmt(const char * f, double a, double b = 0.0, double c = 0.0)
{
  char buf[200];
  std::snprintf(buf, sizeof(buf), f, a, b, c);
  return buf;
}
}  // namespace

bool parse_look_at_slot(const std::string & slot, LookAtGoal & g, std::string & why)
{
  std::stringstream ss(slot);
  std::string item;
  while (std::getline(ss, item, ',')) {
    item.erase(std::remove(item.begin(), item.end(), ' '), item.end());
    if (item.empty()) {continue;}
    const auto eq = item.find('=');
    if (eq == std::string::npos) {why = "LOOK_AT slot '" + item + "' (키=값 아님)"; return false;}
    const std::string k = item.substr(0, eq);
    char * end = nullptr;
    const double val = std::strtod(item.c_str() + eq + 1, &end);
    if (end == item.c_str() + eq + 1 || !std::isfinite(val)) {why = "LOOK_AT slot '" + item + "' (숫자 아님)"; return false;}
    if (k == "d") {g.standoff_m = val;}
    else if (k == "vyaw") {g.view_yaw = val * kDeg;}
    else if (k == "vpitch") {g.view_pitch = val * kDeg;}
    else if (k == "dj1") {g.dj1 = val * kDeg;}
    else if (k == "dj4") {g.dj4 = val * kDeg;}
    else if (k == "lyaw") {g.light_yaw = val * kDeg; g.has_light = true;}
    else if (k == "lpitch") {g.light_pitch = val * kDeg;}
    else if (k == "lcone") {g.light_cone_deg = val;}
    // 모르는 키는 무시 (미션이 먼저 새 키를 보내도 거절 안 함)
  }
  return true;
}

Eigen::Isometry3d look_camera_pose(const ArmModel & m, const Eigen::Isometry3d & T5c, const Q5 & q)
{
  const auto F = arm_frames(m, q);
  Eigen::Isometry3d T5;
  T5.matrix() = F[4];
  Eigen::Isometry3d cam = T5 * T5c;
  cam.translation().z() += model_to_base_z(m);
  return cam;
}

std::vector<Eigen::Vector3d> look_arm_points(const ArmModel & m, const Eigen::Isometry3d & T5c, const Q5 & q)
{
  const auto F = arm_frames(m, q);
  const double dz = model_to_base_z(m);
  const Eigen::Vector3d up(0.0, 0.0, dz);
  const Eigen::Vector3d p2 = F[2].block<3, 1>(0, 3) + up, p3 = F[3].block<3, 1>(0, 3) + up, p4 = F[4].block<3, 1>(0, 3) + up;
  const Eigen::Vector3d tip = arm_grip(m, q).tip + up;
  const Eigen::Vector3d cam = look_camera_pose(m, T5c, q).translation();
  return {p2, 0.5 * (p2 + p3), p3, 0.5 * (p3 + p4), p4, 0.5 * (p4 + tip), tip, cam};
}

double look_roll(const Eigen::Isometry3d & cam, double period_rad)
{
  const Eigen::Vector3d x = cam.linear().col(0), y = cam.linear().col(1);
  const double phi = std::atan2(x.z(), -y.z());
  if (!(period_rad > 0.0)) {return phi;}
  return std::remainder(phi, period_rad);
}

namespace
{
bool clear_impl(const ArmModel & m, const Eigen::Isometry3d & T5c, const Q5 & q, const LookAtParams & p,
  const LookAtGoal & g, const Eigen::Vector3d & wide_cam, std::string & why, bool check_body, bool check_wall, bool check_light)
{
  const auto pts = look_arm_points(m, T5c, q);
  static const char * kName[] = {"팔꿈치", "팔꿈치-손목4", "손목4", "손목4-5", "손목5", "손목-손끝", "손끝", "카메라"};
  const Eigen::Vector3d l = g.has_light ? light_dir(g) : Eigen::Vector3d::Zero();
  const Eigen::Vector3d nw = wall_normal(m, g);
  for (std::size_t i = 0; i < pts.size(); ++i) {
    const auto & x = pts[i];
    if (check_body) {
      // 바닥: 손목 쪽 점만 (팔꿈치는 높이 있음)
      if (i >= 4 && x.z() < p.min_z_m) {why = "[BODY] " + std::string(kName[i]) + fmt(" 바닥 가까움 z %.3f m", x.z()); return false;}
      if (in_box(x, p.body_box, p.clear_pad_m)) {why = "[BODY] " + std::string(kName[i]) + fmt(" 차체 위 (%.3f, %.3f, %.3f)", x.x(), x.y(), x.z()); return false;}
      if (in_box(x, p.rack_box, p.clear_pad_m)) {why = "[BODY] " + std::string(kName[i]) + fmt(" 적재함 위 (%.3f, %.3f, %.3f)", x.x(), x.y(), x.z()); return false;}
      if (in_box(x, p.mast_box, p.clear_pad_m)) {why = "[BODY] " + std::string(kName[i]) + fmt(" 광각 기둥 (%.3f, %.3f, %.3f)", x.x(), x.y(), x.z()); return false;}
    }
    if (check_wall && i >= 4 && p.wall_margin_m > 0.0) {   // 손목 · 손끝 · 카메라 ↔ 목표 벽 면
      const double sw = (x - g.target).dot(nw);
      if (sw < p.wall_margin_m) {why = "[WALL] " + std::string(kName[i]) + fmt(" 이 벽 면에서 %.3f m < %.3f (몸을 뒤로)", sw, p.wall_margin_m); return false;}
    }
    if (check_light && g.has_light) {
      const Eigen::Vector3d w = x - wide_cam;
      const double along = w.dot(l);
      if (along > 0.0) {
        const double ang = std::atan2(w.cross(l).norm(), along);
        const double rad = std::atan2(p.clear_pad_m, w.norm());
        if (ang - rad < cone_margin_rad(g, p)) {
          why = "[OCCL] " + std::string(kName[i]) + fmt(" 이 광각 → 신호등 시야를 가림 (%.1f°)", (ang - rad) / kDeg);
          return false;
        }
      }
    }
  }
  return true;
}
}  // namespace

bool look_at_clear(const ArmModel & m, const Eigen::Isometry3d & T5c, const Q5 & q, const LookAtParams & p,
  const LookAtGoal & g, const Eigen::Vector3d & wide_cam, std::string & why)
{
  return clear_impl(m, T5c, q, p, g, wide_cam, why, true, true, true);
}

bool look_at_cone_clear(const ArmModel & m, const Eigen::Isometry3d & T5c, const Q5 & q, const LookAtParams & p,
  const LookAtGoal & g, const Eigen::Vector3d & wide_cam, std::string & why)
{
  return clear_impl(m, T5c, q, p, g, wide_cam, why, false, false, true);
}

double look_at_wall_clear(const ArmModel & m, const Eigen::Isometry3d & T5c, const Q5 & q, const LookAtGoal & g)
{
  const auto pts = look_arm_points(m, T5c, q);
  const Eigen::Vector3d nw = wall_normal(m, g);
  double c = 1e9;
  for (std::size_t i = 4; i < pts.size(); ++i) {c = std::min(c, (pts[i] - g.target).dot(nw));}
  return c;
}

bool look_at_path_clear(const ArmModel & m, const Eigen::Isometry3d & T5c, const Q5 & from, const Q5 & to,
  const LookAtParams & p, const LookAtGoal & g, const Eigen::Vector3d & wide_cam, std::string & why)
{
  double dmax = 0.0;
  for (int j = 0; j < 5; ++j) {dmax = std::max(dmax, std::fabs(to[j] - from[j]));}
  const int n = std::max(1, static_cast<int>(std::ceil(dmax / (std::max(p.path_step_deg, 0.2) * kDeg))));
  bool cone_was_clear = false;   // 원뿔 밖으로 한 번 나온 뒤 다시 들어가면 실패 (시작이 안이면 나올 때까지 봐줌)
  {
    std::string w0;
    cone_was_clear = !g.has_light || clear_impl(m, T5c, from, p, g, wide_cam, w0, false, false, true);
  }
  for (int i = 1; i <= n; ++i) {
    Q5 q;
    for (int j = 0; j < 5; ++j) {q[j] = from[j] + (to[j] - from[j]) * i / n;}
    std::string w;
    const bool last = i == n;
    bool ok = clear_impl(m, T5c, q, p, g, wide_cam, w, true, last, false);
    if (ok && g.has_light) {
      const bool c = clear_impl(m, T5c, q, p, g, wide_cam, w, false, false, true);
      if (c) {cone_was_clear = true;} else if (cone_was_clear || last) {ok = false;}
    }
    if (!ok) {
      const auto t = w.find(']');
      const std::string tag = (!w.empty() && w[0] == '[' && t != std::string::npos) ? w.substr(0, t + 1) : "[BODY]";
      why = tag + " 경로 " + std::to_string(i) + "/" + std::to_string(n) + ": " + w;
      return false;
    }
  }
  return true;
}

LookAtSolution eval_look_at(const ArmModel & m, const Eigen::Isometry3d & T5c, const LookAtGoal & g,
  const LookAtParams & p, const Q5 & q)
{
  LookAtSolution s;
  s.q = q;
  const auto cam = look_camera_pose(m, T5c, q);
  s.cam_pos = cam.translation();
  s.cam_axis = cam.linear().col(2);
  const Eigen::Vector3d dv = g.target - s.cam_pos;
  s.dist_m = dv.norm();
  const Eigen::Vector3d u = dv / std::max(s.dist_m, 1e-9);
  s.axis_err_deg = std::acos(std::clamp(aim_axis(cam, p).dot(u), -1.0, 1.0)) / kDeg;   // 겨눔 화소 광선 기준
  s.obliquity_deg = std::acos(std::clamp(s.cam_axis.dot(view_dir(m, g)), -1.0, 1.0)) / kDeg;
  s.roll_deg = std::fabs(look_roll(cam, p.roll_period_deg * kDeg)) / kDeg;
  s.j2_torque_nm = std::fabs(sag_torques(m, q, 0.0)[0]);
  s.wall_clear_m = look_at_wall_clear(m, T5c, q, g);
  if (!limits_ok(m, q) || !ticks_ok(m, q)) {s.why = "[REACH] 관절 · 틱 범위 밖"; return s;}
  if (p.max_j2_torque_nm > 0.0 && s.j2_torque_nm > p.max_j2_torque_nm) {
    s.why = fmt("[REACH] J2 토크 %.2f N·m > %.2f (팔을 너무 뻗음 — 몸을 가까이)", s.j2_torque_nm, p.max_j2_torque_nm);
    return s;
  }
  if (s.axis_err_deg > p.max_axis_err_deg) {s.why = fmt("[REACH] 광축 오차 %.1f° > %.1f°", s.axis_err_deg, p.max_axis_err_deg); return s;}
  if (s.dist_m < p.standoff_min_m || s.dist_m > p.standoff_max_m) {
    s.why = fmt("[REACH] 거리 %.3f m 가 [%.2f, %.2f] 밖", s.dist_m, p.standoff_min_m, p.standoff_max_m);
    return s;
  }
  {
    const double d = goal_standoff(g, p);
    if (p.max_dist_err_m > 0.0 && std::fabs(s.dist_m - d) > p.max_dist_err_m) {   // 바란 거리에서만 (못 읽는 거리 거절)
      s.why = fmt("[REACH] 거리 %.3f m 가 바란 %.3f ± %.3f 밖", s.dist_m, d, p.max_dist_err_m);
      return s;
    }
  }
  if (s.obliquity_deg > p.max_obliquity_deg) {s.why = fmt("[REACH] 정면에서 %.0f° 비스듬 > %.0f°", s.obliquity_deg, p.max_obliquity_deg); return s;}
  if (s.roll_deg > p.max_roll_deg) {s.why = fmt("[REACH] 화면 돌림 %.0f° > %.0f°", s.roll_deg, p.max_roll_deg); return s;}
  s.ok = true;
  return s;
}

LookAtSolution solve_look_at(const ArmModel & m, const Eigen::Isometry3d & T5c, const LookAtGoal & g,
  const LookAtParams & p, const Q5 & start, const Eigen::Vector3d & wide_cam,
  const std::vector<Q5> & extra_seeds, bool check_path)
{
  const Eigen::Vector3d v = view_dir(m, g);
  const double d = goal_standoff(g, p);
  // 씨앗: start 와 template 에서 J1 을 목표 방향으로 (모델 J1 축 = −z → 각 + = 시계), ±2π, J4 ±0.3
  std::vector<Q5> seeds;
  const auto L = q_limits(m);
  auto add = [&](Q5 q) {
      if (q[0] < L[0][0] || q[0] > L[0][1]) {return;}
      seeds.push_back(clamp_q(m, q));
    };
  std::vector<Q5> bases{start, m.template_q};   // + extra_seeds (실물에서 읽힌 자세 등) 도 J1 을 목표 쪽으로 돌려 씀
  bases.insert(bases.end(), extra_seeds.begin(), extra_seeds.end());
  for (const Q5 & base : bases) {
    const auto cam = look_camera_pose(m, T5c, base);
    const Eigen::Vector3d z = cam.linear().col(2);
    const double az_c = std::atan2(z.y(), z.x());
    const double az_t = std::atan2(g.target.y() - m.j1_xy.y(), g.target.x() - m.j1_xy.x());
    const double dq = -wrap(az_t - az_c);
    for (double k : {0.0, 2.0 * M_PI, -2.0 * M_PI}) {
      for (double dj4 : {0.0, 0.3, -0.3}) {
        Q5 q = base;
        q[0] += dq + k;
        q[3] += dj4;
        add(q);
      }
    }
  }
  add(start);
  for (const auto & q : extra_seeds) {add(q);}

  LookAtSolution best;
  bool best_feasible = false;
  // 보는 방향 후보: 바라는 방향 + (시야 원뿔이 있으면) 옆 · 위에서 비스듬히 (max_obliquity 안) — 팔을 원뿔 밖으로
  std::vector<Eigen::Vector3d> views{v};
  if (g.has_light) {
    const double vy = std::atan2(v.y(), v.x()), vp = std::asin(std::clamp(-v.z(), -1.0, 1.0));
    for (double dy : {20.0, -20.0, 35.0, -35.0}) {
      for (double dp : {0.0, 25.0}) {
        if (std::hypot(dy, dp) > p.max_obliquity_deg) {continue;}
        const double y = vy + dy * kDeg, pt = vp + dp * kDeg;
        views.emplace_back(std::cos(pt) * std::cos(y), std::cos(pt) * std::sin(y), -std::sin(pt));
      }
    }
  }
  std::vector<std::pair<Q5, Eigen::Vector3d>> runs;
  for (const auto & s0 : seeds) {runs.emplace_back(s0, views[0]);}
  for (std::size_t k = 1; k < views.size(); ++k) {   // 비스듬 후보는 씨앗 몇 개만 (계획 시간)
    for (std::size_t i = 0; i < seeds.size() && i < 4; ++i) {runs.emplace_back(seeds[i], views[k]);}
  }
  for (const auto & [s0, vk] : runs) {
    double cost = 0.0;
    const Q5 q = lm(m, T5c, g, p, vk, d, s0, cost, wide_cam);
    auto s = eval_look_at(m, T5c, g, p, q);
    double travel = 0.0;
    for (int j = 0; j < 5; ++j) {travel += std::fabs(q[j] - start[j]);}
    s.cost = cost + 0.02 * travel;
    bool feasible = s.ok;
    if (feasible) {
      std::string w;
      if (!look_at_clear(m, T5c, q, p, g, wide_cam, w)) {feasible = false; s.ok = false; s.why = w;}
      else if (check_path && !look_at_path_clear(m, T5c, start, q, p, g, wide_cam, w)) {feasible = false; s.ok = false; s.why = w;}
    }
    if ((feasible && (!best_feasible || s.cost < best.cost)) || (!best_feasible && !feasible && s.cost < best.cost)) {
      best = s;
      best_feasible = feasible;
    }
  }
  if (!best_feasible) {
    if (best.why.empty()) {best.why = "[REACH] 해 없음";}
    return best;
  }
  // 다시 읽기 흔들기: 해에서 J1 · J4 를 더 돌림 (광축 기준은 안 봄, 여유 · 경로 · 범위만)
  if (g.dj1 != 0.0 || g.dj4 != 0.0) {
    Q5 q = best.q;
    q[0] += g.dj1;
    q[3] += g.dj4;
    q = clamp_q(m, q);   // 범위 끝이면 갈 수 있는 만큼만
    if (std::fabs(q[0] - best.q[0]) + std::fabs(q[3] - best.q[3]) < 0.2 * kDeg) {best.ok = false; best.why = "[REACH] 흔들 여지 없음 (관절 범위 끝)"; return best;}
    auto s = eval_look_at(m, T5c, g, p, q);
    s.cost = best.cost;
    std::string w;
    if (!limits_ok(m, q) || !ticks_ok(m, q)) {best.ok = false; best.why = "[REACH] 흔든 자세가 관절 · 틱 범위 밖"; return best;}
    if (!look_at_clear(m, T5c, q, p, g, wide_cam, w) || (check_path && !look_at_path_clear(m, T5c, start, q, p, g, wide_cam, w))) {
      best.ok = false;
      best.why = w.substr(0, w.find(']') + 1) + " 흔든 자세: " + w;
      return best;
    }
    s.ok = true;
    s.why.clear();
    return s;
  }
  return best;
}

}  // namespace manipulation
