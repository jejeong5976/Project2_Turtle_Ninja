// grasp_guard.cpp — 쥐기 · 넣기 확인 판단 (설명은 include/manipulation/grasp_guard.hpp)

#include "manipulation/grasp_guard.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace manipulation
{

namespace
{
constexpr double kDeg = 180.0 / M_PI;
constexpr double kT2R = 2.0 * M_PI / 4096.0;

bool ends_with(const std::string & s, const char * suf)
{
  const std::string t(suf);
  return s.size() >= t.size() && s.compare(s.size() - t.size(), t.size(), t) == 0;
}

double med(std::vector<double> v)
{
  if (v.empty()) {return kGuardNaN;}
  std::nth_element(v.begin(), v.begin() + static_cast<std::ptrdiff_t>(v.size() / 2), v.end());
  return v[v.size() / 2];
}

double mean(const std::vector<double> & v)
{
  double s = 0.0;
  int n = 0;
  for (double x : v) {
    if (std::isfinite(x)) {s += x; ++n;}
  }
  return n > 0 ? s / n : kGuardNaN;
}

double speed_or(double v) {return v > 0.0 ? v : 15.0;}   // 경로 점에 속도가 없으면 퍽 근처 느린 속도

ArmAction arm_move(const char * phase, const char * pose, const std::vector<double> & from, const std::vector<PathNode> & nodes,
  const PathTiming & t, bool hold)
{
  ArmAction a{phase, false, pose, 0.0};
  a.points = time_path(from, nodes, t);
  if (hold && t.grip_settle_s > 0.0 && !a.points.empty()) {a.points.push_back({nodes.back().q, a.points.back().t + t.grip_settle_s});}
  a.duration_s = a.points.empty() ? 0.0 : a.points.back().t;
  return a;
}

PathNode node(const std::string & name, const std::vector<double> & q, double speed)
{
  PathNode n;
  n.name = name;
  n.q = q;
  n.speed_dps = speed_or(speed);
  return n;
}
}  // namespace

// ============================================================================ ① 쥔 폭
const char * puck_shape_name(PuckShape s)
{
  switch (s) {
    case PuckShape::Cylinder: return "원기둥";
    case PuckShape::Square: return "네모";
    default: return "모양 모름";
  }
}

double finger_gap_mm(double q_m, const GripWidthConfig & c)
{
  if (!std::isfinite(q_m) || !std::isfinite(c.inner0_m) || !std::isfinite(c.gap_bias_mm)) {return kGuardNaN;}
  return 2.0 * (c.inner0_m + q_m) * 1000.0 + c.gap_bias_mm;
}

GripWidthVerdict judge_grip_width(double q_m, PuckShape shape, const GripWidthConfig & c)
{
  GripWidthVerdict v;
  v.gap_mm = finger_gap_mm(q_m, c);
  char buf[240];
  if (!std::isfinite(v.gap_mm) || !std::isfinite(c.tol_mm) || c.tol_mm <= 0.0 || !std::isfinite(c.cylinder_mm) || !std::isfinite(c.square_mm)) {
    v.text = "그리퍼 실측 · 설정 없음 → 폭 안 봄";
    return v;
  }
  v.judged = true;
  if (shape == PuckShape::Unknown) {
    const double lo = std::min(c.cylinder_mm, c.square_mm), hi = std::max(c.cylinder_mm, c.square_mm);
    v.err_mm = v.gap_mm < lo ? v.gap_mm - lo : (v.gap_mm > hi ? v.gap_mm - hi : 0.0);
    v.bad = std::fabs(v.err_mm) > c.tol_mm;
    std::snprintf(buf, sizeof(buf), "그리퍼 %.4f m → 손가락 사이 %.1f mm (모양 모름: %.1f ~ %.1f, 벗어남 %+.1f, 허용 %.1f)", q_m, v.gap_mm, lo, hi,
      v.err_mm, c.tol_mm);
  } else {
    v.expect_mm = shape == PuckShape::Cylinder ? c.cylinder_mm : c.square_mm;
    v.err_mm = v.gap_mm - v.expect_mm;
    v.bad = std::fabs(v.err_mm) > c.tol_mm;
    std::snprintf(buf, sizeof(buf), "그리퍼 %.4f m → 손가락 사이 %.1f mm (%s %.1f, 차 %+.1f, 허용 %.1f)", q_m, v.gap_mm, puck_shape_name(shape),
      v.expect_mm, v.err_mm, c.tol_mm);
  }
  v.text = buf;
  return v;
}

// ============================================================================ ② 손 안 퍽
const char * held_sample_reject(const std::string & id, const std::string & frame, double stamp_s, const std::string & want_id, double after_s,
  double before_s)
{
  if (frame != "wrist_link") {return "손목 좌표 아님";}
  if (want_id.empty()) {return "PICK 퍽 ID 없음";}
  if (id != want_id) {return "다른 퍽";}
  if (!std::isfinite(stamp_s) || !(stamp_s >= after_s)) {return "쥐기 전";}
  if (stamp_s > before_s) {return "놓은 뒤";}
  return nullptr;
}

HeldVerdict judge_held_in_hand(bool seen, const Eigen::Vector3d & p, const Eigen::Vector3d & n, int frames,
  const Eigen::Vector3d & tip, const Eigen::Vector3d & app, const Eigen::Vector3d & close, double max_mm, double max_face_deg)
{
  HeldVerdict v;
  v.frames = frames;
  char buf[260];
  if (!seen || !p.allFinite() || !n.allFinite() || !tip.allFinite() || app.norm() < 0.5 || close.norm() < 0.5 || n.norm() < 0.5) {
    std::snprintf(buf, sizeof(buf), "쥔 퍽 태그 못 봄 (%d 장)", frames);
    v.text = buf;
    return v;
  }
  v.seen = true;
  const Eigen::Vector3d a = app.normalized();
  Eigen::Vector3d c = close - a * close.dot(a);
  if (c.norm() < 1e-6) {c = a.unitOrthogonal();}
  c.normalize();
  const Eigen::Vector3d w = a.cross(c);
  const Eigen::Vector3d d = p - tip;
  const double along = d.dot(a);              // 태그는 손끝보다 손목 쪽 (− 접근축)
  const Eigen::Vector3d lat = d - along * a;
  v.lateral_mm = lat.norm() * 1000.0;
  v.close_mm = lat.dot(c) * 1000.0;
  v.width_mm = lat.dot(w) * 1000.0;
  v.depth_mm = -along * 1000.0;
  v.face_deg = std::acos(std::clamp(std::fabs(n.normalized().dot(a)), 0.0, 1.0)) * kDeg;
  v.bad = (std::isfinite(max_mm) && v.lateral_mm > max_mm) || (std::isfinite(max_face_deg) && v.face_deg > max_face_deg);
  std::snprintf(buf, sizeof(buf), "접근축에서 %.1f mm (닫힘 %+.1f · 폭 %+.1f) · 윗면 %.0f° · 쥔 깊이 약 %.0f mm · %d 장 (한도 %.0f mm · %.0f°)",
    v.lateral_mm, v.close_mm, v.width_mm, v.face_deg, v.depth_mm, frames, max_mm, max_face_deg);
  v.text = buf;
  return v;
}

// ============================================================================ ③ 넣기 걸림
Q5 team_to_model_q(const ArmModel & m, const std::vector<double> & c)
{
  Q5 q{};
  for (int j = 0; j < 5; ++j) {
    const double v = j < static_cast<int>(c.size()) ? c[j] : kGuardNaN;
    const double tick = m.zero_tick[j] + v / kT2R;
    q[j] = (m.jm[j].q_deg_at_ref + m.jm[j].sign * (tick - m.jm[j].ref_tick) * 360.0 / 4096.0) / kDeg;
  }
  return q;
}

std::vector<double> model_to_team(const ArmModel & m, const Q5 & q)
{
  std::vector<double> c(5);
  for (int j = 0; j < 5; ++j) {
    const double tick = m.jm[j].ref_tick + (q[j] * kDeg - m.jm[j].q_deg_at_ref) / (m.jm[j].sign * 360.0 / 4096.0);
    c[j] = (tick - m.zero_tick[j]) * kT2R;
  }
  return c;
}

std::vector<double> command_at(const std::vector<TrajPoint> & pts, double t)
{
  if (pts.empty()) {return {};}
  if (t <= pts.front().t) {return pts.front().q;}
  if (t >= pts.back().t) {return pts.back().q;}
  const auto it = std::upper_bound(pts.begin(), pts.end(), t, [](double v, const TrajPoint & p) {return v < p.t;});
  const auto & b = *it;
  const auto & a = *(it - 1);
  const double w = b.t > a.t ? (t - a.t) / (b.t - a.t) : 1.0;
  std::vector<double> q(a.q.size());
  for (std::size_t j = 0; j < q.size() && j < b.q.size(); ++j) {q[j] = a.q[j] + (b.q[j] - a.q[j]) * w;}
  return q;
}

double descent_start_time(const ArmModel & m, const std::vector<TrajPoint> & pts, double zone_m)
{
  if (pts.size() < 2 || !std::isfinite(zone_m)) {return 0.0;}
  const double z_end = arm_grip(m, team_to_model_q(m, pts.back().q)).tip.z();
  for (std::size_t i = pts.size(); i-- > 0;) {
    if (arm_grip(m, team_to_model_q(m, pts[i].q)).tip.z() >= z_end + zone_m) {return pts[i].t;}
  }
  return 0.0;
}

void ContactWatch::start(const std::vector<TrajPoint> & pts, double t0, double t_desc, int attempt)
{
  pts_ = pts;
  t0_ = t0;
  t_desc_ = std::isfinite(t_desc) ? std::max(0.0, t_desc) : 0.0;
  attempt_ = attempt;
  s_.clear();
  active_ = !pts_.empty();
  t_end_ = pts_.empty() ? 0.0 : pts_.back().t;
  hold_from_ = t_end_;
  const std::size_t n = pts_.size();
  if (n >= 2 && pts_[n - 1].q.size() == pts_[n - 2].q.size()) {   // path_actions 의 그리퍼 앞 멈춤 (같은 자세 마지막 두 점)
    double d = 0.0;
    for (std::size_t j = 0; j < pts_[n - 1].q.size(); ++j) {d = std::max(d, std::fabs(pts_[n - 1].q[j] - pts_[n - 2].q[j]));}
    if (d < 1e-9) {hold_from_ = pts_[n - 2].t;}
  }
}

void ContactWatch::feed(double stamp_s, const std::array<double, 5> & q, const std::array<double, 5> & cur)
{
  if (!active_ || !std::isfinite(stamp_s)) {return;}
  const double t = stamp_s - t0_;
  if (t < 0.0 || s_.size() >= 4000) {return;}   // 보내기 전 측정 · 너무 김 (100 Hz 40 s)
  s_.push_back({t, q, cur});
}

ContactVerdict ContactWatch::judge(const ContactConfig & c, const ArmModel * m) const
{
  ContactVerdict v;
  char buf[340];
  const double b0 = std::max(0.0, t_desc_ - std::max(0.05, c.base_s));
  const double b1 = t_desc_ - b0 >= 0.1 ? t_desc_ : b0 + 0.15;
  // 명령 → 실측 지연 (보낸 시각 · JTC 시작 · 서보 늦음): 내려가기 전 J2–J4 오차의 흔들림 (관절마다 평균을 뺀 제곱합 — 처짐 같은
  //   일정한 오차는 빠짐) 이 가장 작은 지연 (0 ~ 200 ms, 5 ms 간격). 안 빼면 움직이는 동안의 기준이 속도 × 지연만큼 틀어짐 (mock: J2 · J3 약 0.6°)
  double lag = 0.0, best = std::numeric_limits<double>::infinity();
  for (int k = 0; k <= 40; ++k) {
    const double d = 0.005 * k;
    std::array<double, 3> sum{}, sum2{};
    std::array<int, 3> n{};
    for (const auto & s : s_) {
      if (s.t > b1) {break;}
      const auto cmd = command_at(pts_, s.t - d);
      for (int j = 0; j < 3 && cmd.size() == 5; ++j) {
        const double e = cmd[j + 1] - s.q[j + 1];
        if (std::isfinite(e)) {sum[j] += e; sum2[j] += e * e; ++n[j];}
      }
    }
    double var = 0.0;
    int used = 0;
    for (int j = 0; j < 3; ++j) {
      if (n[j] > 0) {var += sum2[j] / n[j] - (sum[j] / n[j]) * (sum[j] / n[j]); ++used;}
    }
    if (used > 0 && var < best - 1e-15) {best = var; lag = d;}
  }
  const double hold = t_end_ - hold_from_;
  const double e0 = std::max(hold > 0.05 ? hold_from_ + 0.5 * hold : t_end_ - 0.1, hold_from_ + lag + 0.05);
  std::array<std::vector<double>, 3> eb, ee, cb;
  std::vector<double> zb, ze;
  for (const auto & s : s_) {
    const bool in_b = s.t >= b0 && s.t <= b1, in_e = s.t >= e0;
    if (!in_b && !in_e) {continue;}
    const auto cmd = command_at(pts_, s.t - lag);
    if (cmd.size() != 5) {continue;}
    for (int j = 0; j < 3; ++j) {
      const double e = cmd[j + 1] - s.q[j + 1];
      if (in_b) {eb[j].push_back(e); cb[j].push_back(s.cur[j + 1]);}
      if (in_e) {ee[j].push_back(e);}
    }
    if (m) {
      bool fin = true;
      for (double x : s.q) {fin = fin && std::isfinite(x);}
      if (fin) {
        const double dz = arm_grip(*m, team_to_model_q(*m, std::vector<double>(s.q.begin(), s.q.end()))).tip.z() -
          arm_grip(*m, team_to_model_q(*m, cmd)).tip.z();
        if (in_b) {zb.push_back(dz);}
        if (in_e) {ze.push_back(dz);}
      }
    }
    if (in_b) {++v.n_base;}
    if (in_e) {++v.n_end;}
  }
  std::array<double, 3> cbase{};
  for (int j = 0; j < 3; ++j) {cbase[j] = mean(cb[j]);}
  for (const auto & s : s_) {   // 기준 뒤 (마지막 내려가기 ~ 끝) 전류 변화 최대
    if (s.t <= b1) {continue;}
    ++v.n_eval;
    for (int j = 0; j < 3; ++j) {
      const double d = std::fabs(s.cur[j + 1] - cbase[j]);
      if (std::isfinite(d)) {v.cur_peak_a[j] = std::isfinite(v.cur_peak_a[j]) ? std::max(v.cur_peak_a[j], d) : d;}
    }
  }
  if (v.n_base < 2 || v.n_end < 2) {
    std::snprintf(buf, sizeof(buf), "관절 실측 모자람 (기준 %d · 끝 %d 장) → 판단 안 함", v.n_base, v.n_end);
    v.text = buf;
    return v;
  }
  v.judged = true;
  for (int j = 0; j < 3; ++j) {v.err_end_deg[j] = (mean(ee[j]) - mean(eb[j])) * kDeg;}
  if (!zb.empty() && !ze.empty()) {v.height_mm = (mean(ze) - mean(zb)) * 1000.0;}
  for (int j = 0; j < 3; ++j) {
    v.suspect = v.suspect || (std::isfinite(v.err_end_deg[j]) && std::fabs(v.err_end_deg[j]) > c.err_deg);
    v.suspect = v.suspect || (std::isfinite(v.cur_peak_a[j]) && v.cur_peak_a[j] > c.current_a);
  }
  v.suspect = v.suspect || (std::isfinite(v.height_mm) && v.height_mm > c.height_mm);
  std::snprintf(buf, sizeof(buf), "끝 오차 변화 J2 %+.2f · J3 %+.2f · J4 %+.2f° (문턱 %.1f) · 전류 변화 최대 %.2f · %.2f · %.2f A (문턱 %.2f) · "
    "손끝 덜 내려감 %+.1f mm (문턱 %.1f) · 기준 %d · 끝 %d 장 · 지연 %.0f ms", v.err_end_deg[0], v.err_end_deg[1], v.err_end_deg[2], c.err_deg,
    v.cur_peak_a[0], v.cur_peak_a[1], v.cur_peak_a[2], c.current_a, v.height_mm, c.height_mm, v.n_base, v.n_end, lag * 1000.0);
  v.text = buf;
  return v;
}

bool tip_offset_command(const ArmModel & m, const std::vector<double> & cmd, const Eigen::Vector3d & dp_m, std::vector<double> & out)
{
  using V5 = Eigen::Matrix<double, 5, 1>;
  using F9 = Eigen::Matrix<double, 9, 1>;
  if (cmd.size() != 5 || !dp_m.allFinite()) {return false;}
  for (double v : cmd) {
    if (!std::isfinite(v)) {return false;}
  }
  auto feat = [&](const V5 & c) {
      const Grip g = arm_grip(m, team_to_model_q(m, std::vector<double>(c.data(), c.data() + 5)));
      F9 f;
      f << g.tip * 1000.0 * 10.0, g.app * 100.0, g.close * 100.0;   // 위치 우선 (cell_shift 와 같은 무게)
      return f;
    };
  const V5 c0 = Eigen::Map<const V5>(cmd.data());
  F9 goal = feat(c0);
  goal.head<3>() += dp_m * 1000.0 * 10.0;
  const double lam = 0.1;
  V5 c = c0;
  for (int it = 0; it < 30; ++it) {
    const F9 f = feat(c);
    Eigen::Matrix<double, 9, 5> J;
    for (int j = 0; j < 5; ++j) {
      V5 e = c;
      e[j] += 1e-6;
      J.col(j) = (feat(e) - f) / 1e-6;
    }
    const Eigen::Matrix<double, 5, 5> A = J.transpose() * J + lam * Eigen::Matrix<double, 5, 5>::Identity();
    const V5 dc = A.ldlt().solve(J.transpose() * (goal - f) - lam * (c - c0));
    c += dc;
    if (dc.cwiseAbs().maxCoeff() < 1e-10) {break;}
  }
  const F9 f = feat(c);
  if ((f.head<3>() - goal.head<3>()).norm() / 10.0 > 0.5) {return false;}   // 손끝 0.5 mm
  const Eigen::Vector3d a0 = goal.segment<3>(3).normalized(), a1 = f.segment<3>(3).normalized();
  if (std::acos(std::clamp(a0.dot(a1), -1.0, 1.0)) * kDeg > 1.0) {return false;}
  // 관절 범위 (틱): 범위 안 ± 15 틱, 아니면 출발 자세보다 더 밖으로는 안 감 (기록 경로가 범위 끝에 붙은 칸 · J4 기계 멈춤 2485)
  for (int j = 0; j < 5; ++j) {
    const double t0 = m.zero_tick[j] + c0[j] / kT2R, t1 = m.zero_tick[j] + c[j] / kT2R;
    const double lo = m.tick_range[j][0] - 15.0, hi = m.tick_range[j][1] + 15.0;
    if (t1 < lo && t1 < t0 - 1e-6) {return false;}
    if (t1 > hi && t1 > t0 + 1e-6) {return false;}
  }
  out.assign(c.data(), c.data() + 5);
  return true;
}

// ============================================================================ ④ 넣은 뒤 (착좌)
const char * seat_state_name(SeatState s)
{
  switch (s) {
    case SeatState::SEATED: return "들어감";
    case SeatState::NOT_SEATED: return "덜 들어간 듯";
    default: return "판단 못 함";
  }
}

SeatVerdict judge_seat(const std::vector<SeatSample> & samples, const Eigen::Vector2d & cell_xy, double puck_w, const SeatConfig & c)
{
  SeatVerdict v;
  char buf[260];
  std::array<std::vector<double>, 3> v3, v2;
  for (const auto & s : samples) {
    if (!s.p.allFinite()) {continue;}
    if (s.has_z) {
      ++v.n3;
      for (int j = 0; j < 3; ++j) {v3[j].push_back(s.p[j]);}
    } else {
      ++v.n2;
      for (int j = 0; j < 2; ++j) {v2[j].push_back(s.p[j]);}
    }
  }
  if (!cell_xy.allFinite() || !std::isfinite(puck_w) || !std::isfinite(c.floor_z)) {
    v.text = "칸 위치 · 바닥 높이 없음";
    return v;
  }
  const double ax = std::max(0.0, (c.cup_x - puck_w) / 2.0) * 1000.0 + c.slack_mm;
  const double ay = std::max(0.0, (c.cup_y - puck_w) / 2.0) * 1000.0 + c.slack_mm;
  if (v.n3 >= c.min_samples) {
    v.rel_mm = Eigen::Vector3d(med(v3[0]) - cell_xy.x(), med(v3[1]) - cell_xy.y(), med(v3[2]) - c.floor_z - c.puck_h) * 1000.0;
    // 다른 면 · 다른 퍽을 윗면으로 본 것 (Codex store_verify 와 같은 확인): 높이 −10 ~ +35 mm · 칸 중심 20 mm 밖이면 판단 안 함
    if (v.rel_mm.z() < -10.0 || v.rel_mm.z() > 35.0 || v.rel_mm.head<2>().norm() > 20.0) {
      v.state = SeatState::UNKNOWN;
    } else {
      const bool inside = std::fabs(v.rel_mm.x()) <= ax && std::fabs(v.rel_mm.y()) <= ay;
      v.state = std::fabs(v.rel_mm.z()) <= c.tol_mm && inside ? SeatState::SEATED : SeatState::NOT_SEATED;
    }
    std::snprintf(buf, sizeof(buf), "윗면 높이 %+.1f mm (바닥 + %.0f 기준, 허용 %.0f) · 칸 중심 대비 (%+.1f, %+.1f) mm (컵 여유 %.1f · %.1f) · %d 장",
      v.rel_mm.z(), c.puck_h * 1000.0, c.tol_mm, v.rel_mm.x(), v.rel_mm.y(), ax, ay, v.n3);
  } else if (v.n2 >= c.min_samples) {
    v.rel_mm = Eigen::Vector3d(med(v2[0]) - cell_xy.x(), med(v2[1]) - cell_xy.y(), kGuardNaN) * 1000.0;
    std::snprintf(buf, sizeof(buf), "높이 모름 (칸 퍽 태그만) · 칸 중심 대비 (%+.1f, %+.1f) mm (컵 여유 %.1f · %.1f) · %d 장", v.rel_mm.x(),
      v.rel_mm.y(), ax, ay, v.n2);
  } else {
    std::snprintf(buf, sizeof(buf), "놓은 퍽 태그 모자람 (가까이 %d · 칸 퍽 %d 장)", v.n3, v.n2);
  }
  v.text = buf;
  return v;
}

// ============================================================================ ⑤ 다시 쥐기 동작
std::vector<RegripPoint> regrip_points(const ArmPath & path, const GripperValues & grip)
{
  std::vector<RegripPoint> out;
  const std::size_t n = path.nodes.size();
  for (std::size_t k = 0; k + 1 < n; ++k) {
    const auto & nd = path.nodes[k];
    if (nd.grip != "close" || nd.grip_on_way || nd.name == "park_press_turn" || nd.name == "park_push_over") {continue;}
    RegripPoint r;
    r.close_name = nd.name;
    r.q_close = nd.q;
    r.close_speed_dps = nd.speed_dps;
    r.up_name = path.nodes[k + 1].name;
    r.q_up = path.nodes[k + 1].q;
    r.up_speed_dps = path.nodes[k + 1].speed_dps;
    for (std::size_t i = k; i-- > 0;) {
      const auto & g = path.nodes[i].grip;
      if (g == "open") {r.open_m = grip.open; break;}
      if (g == "release") {r.open_m = grip.release; break;}
      if (!g.empty()) {break;}   // 앞이 닫기 · loose 면 그 자리에서 열 폭을 모름
    }
    r.after.assign(path.nodes.begin() + static_cast<std::ptrdiff_t>(k + 1), path.nodes.end());
    r.retry_ok = ends_with(r.up_name, "up") && std::isfinite(r.open_m) && r.q_close.size() == 5 && r.q_up.size() == 5;
    out.push_back(r);
  }
  return out;
}

std::vector<ArmAction> regrip_in_place(const RegripPoint & r, const PathTiming & t, const GripperValues & grip)
{
  if (!r.retry_ok) {return {};}
  std::vector<ArmAction> out;
  out.push_back(ArmAction{"regrip", true, "", r.open_m});   // phase "regrip": 놓기 ("release") 로 안 봄 (넣기 감시 · 놓은 시각)
  out.push_back(arm_move("regrip", "regrip_grasp", r.q_close,
    {node("regrip_up", r.q_up, r.up_speed_dps), node("regrip_grasp", r.q_close, r.close_speed_dps)}, t, true));
  out.push_back(ArmAction{"grasp", true, "", grip.close});
  return out;
}

bool put_back_ok(const RegripPoint & r)
{
  return r.retry_ok && r.after.size() == 2 && ends_with(r.after[0].name, "up") && ends_with(r.after[1].name, "lift") &&
         r.after[0].q.size() == 5 && r.after[1].q.size() == 5;
}

std::vector<ArmAction> put_back_regrip(const RegripPoint & r, const PathTiming & t, const GripperValues & grip)
{
  if (!put_back_ok(r)) {return {};}
  const auto & lift = r.after[1].q;
  std::vector<ArmAction> out;
  out.push_back(arm_move("regrip", "held_put_down", lift,
    {node("held_put_up", r.q_up, r.up_speed_dps), node("held_put_down", r.q_close, r.close_speed_dps)}, t, true));
  out.push_back(ArmAction{"regrip", true, "", r.open_m});
  out.push_back(arm_move("regrip", "held_regrip", r.q_close,
    {node("held_regrip_up", r.q_up, r.up_speed_dps), node("held_regrip", r.q_close, r.close_speed_dps)}, t, true));
  out.push_back(ArmAction{"grasp", true, "", grip.close});
  out.push_back(arm_move("lift", "held_relift", r.q_close,
    {node("held_relift_up", r.q_up, r.up_speed_dps), node("held_relift", lift, r.up_speed_dps)}, t, false));
  return out;
}

ArmAction contact_retry_action(const ArmModel & m, const std::vector<double> & cmd_end, double lift_m, const Eigen::Vector2d & d,
  double speed_dps, const PathTiming & t, std::string & why)
{
  ArmAction a{"release", false, "contact_retry", 0.0};
  std::vector<double> up, up_shift, down_shift;
  if (!tip_offset_command(m, cmd_end, Eigen::Vector3d(0.0, 0.0, lift_m), up)) {why = "곧게 올린 자세가 안 풀림"; return a;}
  if (!tip_offset_command(m, cmd_end, Eigen::Vector3d(d.x(), d.y(), lift_m), up_shift)) {why = "올려 옮긴 자세가 안 풀림"; return a;}
  if (!tip_offset_command(m, cmd_end, Eigen::Vector3d(d.x(), d.y(), 0.0), down_shift)) {why = "옮겨 내린 자세가 안 풀림"; return a;}
  a = arm_move("release", "contact_retry", cmd_end,
    {node("contact_up", up, speed_dps), node("contact_shift", up_shift, speed_dps), node("contact_retry", down_shift, speed_dps)}, t, true);
  why.clear();
  return a;
}

// ============================================================================ ⑥ 넣기 걸림 보정
double tip_lift_max(const ArmModel & m, const std::vector<double> & cmd, double lift_m, double step_m, std::vector<double> & out)
{
  out = cmd;
  if (cmd.size() != 5 || !(lift_m > 0.0) || !(step_m > 0.0)) {return 0.0;}
  const Grip g0 = arm_grip(m, team_to_model_q(m, cmd));
  if (!g0.tip.allFinite()) {return 0.0;}
  const int n = std::max(1, static_cast<int>(std::ceil(lift_m / step_m - 1e-9)));
  std::vector<double> q = cmd, next;
  double done = 0.0;
  for (int i = 0; i < n; ++i) {
    const double h = std::min(lift_m, (i + 1) * lift_m / n);
    if (!tip_offset_command(m, q, Eigen::Vector3d(0.0, 0.0, h - done), next)) {break;}
    const Grip g = arm_grip(m, team_to_model_q(m, next));
    if (!g.tip.allFinite() || (g.tip - g0.tip - Eigen::Vector3d(0.0, 0.0, h)).norm() > 0.001 ||
      std::acos(std::clamp(g0.app.normalized().dot(g.app.normalized()), -1.0, 1.0)) * kDeg > 2.0)
    {
      break;
    }
    q = next;
    done = h;
  }
  out = q;
  return done;
}

std::size_t store_release_index(const std::vector<PathNode> & nodes)
{
  for (std::size_t i = 0; i < nodes.size(); ++i) {
    if (nodes[i].grip == "release" || nodes[i].grip == "open") {return i;}
  }
  return nodes.size();
}

std::vector<PathNode> store_recover_out_nodes(const ArmModel & m, const std::vector<PathNode> & nodes, std::size_t release, double lift_m,
  std::string & why, std::string & note)
{
  why.clear();
  note.clear();
  if (release == 0 || release >= nodes.size()) {why = "놓는 점 없음"; return {};}
  std::vector<std::string> names;
  for (std::size_t i = 0; i < release; ++i) {names.push_back(nodes[i].name);}
  const std::vector<double> q0 = nodes.front().q;
  auto out = retrace_nodes(nodes, release, names, [&q0](const PathNode & n) {return n.q == q0;});
  if (out.size() < 2) {why = "온 길 거꾸로 경로가 안 됨"; return {};}
  if (!(lift_m > 0.0)) {return out;}
  auto tip = [&m](const std::vector<double> & q) {return arm_grip(m, team_to_model_q(m, q)).tip;};
  const Eigen::Vector3d t0 = tip(out[0].q);
  if (!t0.allFinite()) {note = "놓는 점 손끝을 못 셈 → 올리기 없이 온 길로"; return out;}
  if (tip(out[1].q).z() - t0.z() >= lift_m - 0.001) {return out;}   // 칸 안 곧은 구간이 이미 lift 이상
  std::vector<double> up;
  const double got = tip_lift_max(m, out[0].q, lift_m, 0.005, up);
  const double above = tip(out[1].q).z() - t0.z();
  if (got < above + 0.002) {
    char buf[160];
    std::snprintf(buf, sizeof(buf), "곧게 %.0f mm 까지만 됨 (관절 범위) · 바로 위 기록 점 %.0f mm → 올리기 없이 온 길로", got * 1000.0, above * 1000.0);
    note = buf;
    return out;
  }
  const double want = lift_m;
  lift_m = got;
  PathNode lift = out[1];
  lift.name = "recover_lift";
  lift.q = up;
  std::size_t k = 1;
  while (k + 1 < out.size()) {   // 끝점 (nodes[0]) 은 늘 남김
    const Eigen::Vector3d tk = tip(out[k].q);
    if (!tk.allFinite() || tk.z() >= t0.z() + lift_m - 0.001 || (tk.head<2>() - t0.head<2>()).norm() > 0.015) {break;}
    ++k;
  }
  std::vector<PathNode> r{out[0], lift};
  r.insert(r.end(), out.begin() + static_cast<std::ptrdiff_t>(k), out.end());
  char buf[192];
  std::snprintf(buf, sizeof(buf), "놓는 점에서 곧게 %.0f mm 올림%s (바로 위 기록 점 %.0f mm · 곧은 구간 점 %zu 개 건너뜀)", got * 1000.0,
    got < want - 0.001 ? " (관절 범위로 여기까지)" : "", above * 1000.0, k - 1);
  note = buf;
  return r;
}

std::string store_returned_token(double x, double y)
{
  char buf[96];
  std::snprintf(buf, sizeof(buf), "%s %.4f %.4f", kStoreReturnedTag, std::round(x * 1e4) / 1e4 + 0.0, std::round(y * 1e4) / 1e4 + 0.0);
  return buf;
}

bool parse_store_returned(const std::string & message, double & x, double & y)
{
  const auto at = message.find(kStoreReturnedTag);
  if (at == std::string::npos) {return false;}
  double a = 0.0, b = 0.0;
  if (std::sscanf(message.c_str() + at + std::string(kStoreReturnedTag).size(), " %lf %lf", &a, &b) != 2 || !std::isfinite(a) ||
    !std::isfinite(b))
  {
    return false;
  }
  x = a;
  y = b;
  return true;
}

}  // namespace manipulation
