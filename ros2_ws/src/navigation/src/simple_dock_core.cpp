// simple_dock_core.cpp — 규칙 정밀 접근 계산 구현, 설명은 simple_dock_core.hpp

#include "navigation/simple_dock_core.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>

namespace navigation::dock
{

using rl::wrap;

namespace
{
double dot(const Vec2 & a, const Vec2 & b) {return a.x * b.x + a.y * b.y;}
Vec2 sub(const Vec2 & a, const Vec2 & b) {return {a.x - b.x, a.y - b.y};}
double norm(const Vec2 & a) {return std::hypot(a.x, a.y);}
double deg(double r) {return r * 180.0 / M_PI;}
Vec2 rot(const Vec2 & v, double a) {return {std::cos(a) * v.x - std::sin(a) * v.y, std::sin(a) * v.x + std::cos(a) * v.y};}

double median(std::vector<double> v)                    // numpy 와 같음 (짝수 개 = 가운데 둘 평균)
{
  std::sort(v.begin(), v.end());
  const size_t n = v.size();
  return n % 2 ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
}

std::string fmt(const char * f, double a)
{
  char b[128];
  std::snprintf(b, sizeof(b), f, a);
  return b;
}

std::string fmt2(const char * f, double a, double b2)   // 가까이 확인 로그용
{
  char b[160];
  std::snprintf(b, sizeof(b), f, a, b2);
  return b;
}

// 2 차원 점들의 가장 큰 분산 방향 (SVD vt[0] 와 같은 축, 부호는 다를 수 있음 — 뒤 계산은 부호와 무관)
Vec2 principal(const std::vector<Vec2> & P, const Vec2 & mu)
{
  double sxx = 0, sxy = 0, syy = 0;
  for (const auto & p : P) {
    const double dx = p.x - mu.x, dy = p.y - mu.y;
    sxx += dx * dx; sxy += dx * dy; syy += dy * dy;
  }
  const double a = 0.5 * std::atan2(2.0 * sxy, sxx - syy);
  return {std::cos(a), std::sin(a)};
}
}  // namespace

// ================================================================ 박스 앞면 추적

double fold_quarter(double a)
{
  const double q = 0.5 * M_PI;
  return a - q * std::round(a / q);
}

FaceTrack::FaceTrack(Vec2 goal_xy, double goal_yaw, double box_dist, double half, double search, double track,
  double track_deg, double max_deg, double freeze_dist, int med_n, double tol, int iters)
: yaw0(goal_yaw), h(half), dist_(box_dist), search_(search), track_(track), track_th_(track_deg * M_PI / 180.0),
  max_th_(max_deg * M_PI / 180.0), freeze_(freeze_dist), tol_(tol), med_n_(med_n), iters_(iters)
{
  c0 = {goal_xy.x + box_dist * std::cos(goal_yaw), goal_xy.y + box_dist * std::sin(goal_yaw)};
  c = c0;
}

Pose2 FaceTrack::goal() const
{
  const double y = yaw0 + th;
  return {c.x - dist_ * std::cos(y), c.y - dist_ * std::sin(y), wrap(y)};
}

void FaceTrack::switch_face(double dth)
{
  th += dth;
  for (auto * v : {&hist_, &pend_, &near_pend_}) {
    for (auto & e : *v) {e[2] += dth;}
  }
}

bool FaceTrack::update(const std::vector<Vec2> & P, Vec2 robot_xy)
{
  const Pose2 g = goal();
  if (std::hypot(robot_xy.x - g.x, robot_xy.y - g.y) < freeze_) {
    if (near_shift > 0.0 && n_near < near_max && near_update(P, robot_xy, g)) {return true;}
    last_why = "freeze";
    return false;
  }
  const bool first = n_ok == 0;
  // 처음: 방향 기준 = 확인 중인 첫 조각 (없으면 지도 방향 0) → 정사각 박스 면 4 개 중 같은 면으로 접음 (±45° 경계에서 안 갈림)
  const double ref = first ? (pend_n_ > 0 ? pend_.back()[2] : 0.0) : th;
  const auto m = find(P, robot_xy, first ? c0 : c, first ? search_ : track_, ref, first ? max_th_ : track_th_, first);
  if (!m) {
    if (first) {pend_n_ = 0; pend_.clear();}
    return false;
  }
  if (first && confirm_n > 1) {                         // 처음 조각: 연속 confirm_n 스캔 같은 자리여야 받음
    const bool same = pend_n_ > 0 && std::hypot((*m)[0] - pend_.back()[0], (*m)[1] - pend_.back()[1]) < 0.03 &&
      std::abs(fold_quarter((*m)[2] - pend_.back()[2])) < 10.0 * M_PI / 180.0;   // 한 스캔 면 각 잡음 ±3–11° (20 cm 면) → 각은 넓게, 위치로 거름
    if (!same) {pend_n_ = 0; pend_.clear(); pend_seg_.clear();}
    pend_.push_back(*m);
    pend_seg_.push_back(seg_);
    if (++pend_n_ < confirm_n) {
      char b[64];
      std::snprintf(b, sizeof(b), "확인 %d/%d", pend_n_, confirm_n);
      last_why = b;
      return false;
    }
    hist_.assign(pend_.begin(), pend_.end() - 1);       // 확인한 앞 장들도 중앙값에 넣음 (마지막은 아래에서)
    segs_.assign(pend_seg_.begin(), pend_seg_.end() - 1);
    pend_.clear();
    pend_seg_.clear();
    pend_n_ = 0;
  }
  if (first && search_ > 0.08) {
    char b[128];
    std::snprintf(b, sizeof(b), "찾음 (지도에서 %.0f cm · %+.0f°)", std::hypot((*m)[0] - c0.x, (*m)[1] - c0.y) * 100, deg((*m)[2]));
    last_why = b;
  }
  hist_.push_back(*m);
  segs_.push_back(seg_);
  if (static_cast<int>(hist_.size()) > med_n_) {hist_.erase(hist_.begin());}
  if (static_cast<int>(segs_.size()) > fit_n) {segs_.erase(segs_.begin());}
  std::vector<double> xs, ys, ts;
  for (const auto & e : hist_) {xs.push_back(e[0]); ys.push_back(e[1]); ts.push_back(e[2]);}
  c = {median(xs), median(ys)};
  th = median(ts);
  if (fit_face && segs_.size() >= 2) {fit_segments(robot_xy);}
  ++n_ok;
  if (!first) {last_why = "ok";}
  return true;
}

// freeze 안 가까이 확인: 면을 넓게 (지도 자리 ±search) 찾아 연속 near_confirm 스캔 같은 자리면, 그 정차점이 지금 목표 g 보다
//   보는 방향으로 near_shift 넘게 앞일 때만 받음 (중심 · 방향 = 확인한 장들 중앙값). 받으면 true
bool FaceTrack::near_update(const std::vector<Vec2> & P, Vec2 robot_xy, const Pose2 & g)
{
  const std::string keep = last_why;
  const auto m = find(P, robot_xy, c0, search_, n_ok > 0 ? th : 0.0, n_ok > 0 ? track_th_ : max_th_, true);
  last_why = keep;
  if (!m) {
    near_pend_.clear();
    near_seg_.clear();
    near_why = "면 안 보임";
    return false;
  }
  if (!near_pend_.empty() && (std::hypot((*m)[0] - near_pend_.back()[0], (*m)[1] - near_pend_.back()[1]) > 0.03 ||
    std::abs(fold_quarter((*m)[2] - near_pend_.back()[2])) > 10.0 * M_PI / 180.0))
  {
    near_pend_.clear();
    near_seg_.clear();
  }
  near_pend_.push_back(*m);
  near_seg_.push_back(seg_);
  if (static_cast<int>(near_pend_.size()) < near_confirm) {
    near_why = "확인 " + std::to_string(near_pend_.size()) + "/" + std::to_string(near_confirm);
    return false;
  }
  while (static_cast<int>(near_pend_.size()) > near_confirm) {
    near_pend_.erase(near_pend_.begin());
    near_seg_.erase(near_seg_.begin());
  }
  std::vector<double> xs, ys, ts;
  for (const auto & e : near_pend_) {xs.push_back(e[0]); ys.push_back(e[1]); ts.push_back(e[2]);}
  const Vec2 cc{median(xs), median(ys)};
  const double tt = median(ts), y = yaw0 + tt;
  const Vec2 gn{cc.x - dist_ * std::cos(y), cc.y - dist_ * std::sin(y)};
  const double fwd = (gn.x - g.x) * std::cos(g.yaw) + (gn.y - g.y) * std::sin(g.yaw);
  if (fwd <= near_shift) {
    near_why = fmt2("면 보임, 목표와 앞뒤 %+.1f cm (기준 %.0f cm) → 그대로", fwd * 100, near_shift * 100);
    return false;
  }
  c = cc;
  th = tt;
  hist_.assign(near_pend_.begin(), near_pend_.end());
  segs_.assign(near_seg_.begin(), near_seg_.end());
  if (fit_face && segs_.size() >= 2) {fit_segments(robot_xy);}
  near_pend_.clear();
  near_seg_.clear();
  ++n_ok;
  ++n_near;
  near_why = fmt2("면 보임, 목표보다 %.1f cm 앞 → 더 붙음 %.0f", fwd * 100, n_near) + "/" + std::to_string(near_max);
  last_why = "ok";
  return true;
}

// 최근 스캔들의 앞면 조각 점을 합쳐 직선 하나로 → 면 방향 (th) · 면 앞뒤 위치. 옆 위치는 중앙값 그대로
//   한 스캔 면 각은 점 10–20 개라 잡음이 큼 (라이다 1 cm 면 ±3–11°) → 점을 합치면 √(스캔 수) 만큼 줄어듦
void FaceTrack::fit_segments(Vec2 robot_xy)
{
  std::vector<Vec2> U;
  for (const auto & sg : segs_) {U.insert(U.end(), sg.begin(), sg.end());}
  if (U.size() < 8) {return;}
  const auto fit = [](const std::vector<Vec2> & P, Vec2 & mu, Vec2 & d) {
      mu = {0, 0};
      for (const auto & q : P) {mu.x += q.x; mu.y += q.y;}
      mu = {mu.x / P.size(), mu.y / P.size()};
      d = principal(P, mu);
    };
  Vec2 mu, d;
  fit(U, mu, d);
  for (int k = 0; k < 2; ++k) {                         // 잔차 2.5 σ (최소 1 cm) 넘는 점 빼고 다시 맞춤
    double ss = 0;
    for (const auto & q : U) {const double r = dot(sub(q, mu), {-d.y, d.x}); ss += r * r;}
    const double lim = std::max(0.01, 2.5 * std::sqrt(ss / U.size()));
    std::vector<Vec2> V;
    for (const auto & q : U) {if (std::abs(dot(sub(q, mu), {-d.y, d.x})) < lim) {V.push_back(q);}}
    if (V.size() < 8) {return;}
    U.swap(V);
    fit(U, mu, d);
  }
  Vec2 nn{-d.y, d.x};
  if (dot(nn, sub(robot_xy, mu)) < 0) {nn = {-nn.x, -nn.y};}
  const double t = th + fold_quarter(wrap(std::atan2(-nn.y, -nn.x) - yaw0) - th);   // 면 점은 find 가 고른 면으로 돌려 둠
  if (std::abs(wrap(t - th)) > track_th_) {return;}     // 중앙값과 크게 다르면 (섞인 조각) 안 씀
  th = t;
  const double off = dot(sub(mu, {c.x + h * nn.x, c.y + h * nn.y}), nn);   // 면 = 중심 + h · nn 이 맞춘 직선 위에 오게
  c = {c.x + off * nn.x, c.y + off * nn.y};
}

// 조각 끝 end 바로 바깥 (out 쪽, 로봇에서 본 각 0.5–4°) 에 끝보다 3 cm 넘게 가까운 점이 2 개 이상 = 끝이 가려짐 (진짜 모서리인지 모름)
bool FaceTrack::end_hidden(const std::vector<Vec2> & P, Vec2 robot_xy, Vec2 end, Vec2 out) const
{
  const Vec2 e = sub(end, robot_xy);
  const double re = norm(e), ae = std::atan2(e.y, e.x);
  const Vec2 o = sub({end.x + 0.03 * out.x, end.y + 0.03 * out.y}, robot_xy);
  const double sg = wrap(std::atan2(o.y, o.x) - ae) >= 0 ? 1.0 : -1.0;     // 바깥이 각도로 어느 쪽인지
  int n = 0;
  for (const auto & p : P) {
    const Vec2 v = sub(p, robot_xy);
    const double da = sg * wrap(std::atan2(v.y, v.x) - ae);
    if (da > 0.5 * M_PI / 180.0 && da < 4.0 * M_PI / 180.0 && norm(v) < re - 0.03 && ++n >= 2) {return true;}
  }
  return false;
}

// center ±radius · 방향 th_ref ±th_tol 안 앞면 조각 → (중심 x, y, th) 또는 nullopt (last_why = 이유)
std::optional<std::array<double, 3>> FaceTrack::find(const std::vector<Vec2> & P, Vec2 robot_xy, Vec2 center, double radius,
  double th_ref, double th_tol, bool first)
{
  std::vector<Vec2> Q;
  for (const auto & p : P) {if (norm(sub(p, center)) < radius + 2 * h) {Q.push_back(p);}}
  if (Q.size() < 8) {
    last_why = "점 " + std::to_string(Q.size());
    return std::nullopt;
  }
  std::optional<std::pair<double, std::array<double, 3>>> best;
  std::array<double, 8> line{};                         // 고른 조각 직선: 점 · 방향 · 법선 · 끝 (lo, hi)
  double line_turn = 0.0;                               // 고른 조각 (보인 면) → 접은 면 회전 (0 · ±90°)
  Vec2 line_c{0, 0};
  std::string why = "조각 없음";
  const int nQ = static_cast<int>(Q.size());
  for (int it = 0; it < iters_; ++it) {
    const auto [i, j] = rng.pick2(nQ);
    Vec2 d = sub(Q[j], Q[i]);
    const double L = norm(d);
    if (!(0.03 < L && L < 2 * h + 0.03)) {continue;}
    d = {d.x / L, d.y / L};
    Vec2 nn{-d.y, d.x};
    std::vector<Vec2> on;
    for (const auto & q : Q) {if (std::abs(dot(sub(q, Q[i]), nn)) < tol_) {on.push_back(q);}}
    std::vector<double> s;
    for (const auto & q : on) {s.push_back(dot(sub(q, Q[i]), d));}
    std::sort(s.begin(), s.end());
    if (s.size() < 8) {continue;}
    // 표본 두 점을 품은 연속 조각 (3 cm 넘게 비면 끊김). 표본이 조각 첫 점이어도 그 조각
    std::vector<long> gaps;
    for (size_t k = 0; k + 1 < s.size(); ++k) {if (s[k + 1] - s[k] > 0.03) {gaps.push_back(static_cast<long>(k));}}
    const long i0 = static_cast<long>(std::lower_bound(s.begin(), s.end(), 0.0) - s.begin()) - 1;
    const long k = static_cast<long>(std::upper_bound(gaps.begin(), gaps.end(), i0) - gaps.begin());
    double lo = k > 0 ? s[gaps[k - 1] + 1] : s.front();
    double hi = k < static_cast<long>(gaps.size()) ? s[gaps[k]] : s.back();
    if (!(lo <= L && L <= hi)) {continue;}
    std::vector<Vec2> seg;
    for (const auto & q : on) {
      const double v = dot(sub(q, Q[i]), d);
      if (v >= lo && v <= hi) {seg.push_back(q);}
    }
    if (seg.size() < 8) {continue;}
    Vec2 mu{0, 0};                                      // 조각 점으로 직선 다시 맞춤 (표본 두 점 방향은 잡음이 큼)
    for (const auto & q : seg) {mu.x += q.x; mu.y += q.y;}
    mu = {mu.x / seg.size(), mu.y / seg.size()};
    d = principal(seg, mu);
    nn = {-d.y, d.x};
    lo = std::numeric_limits<double>::infinity();
    hi = -lo;
    for (const auto & q : seg) {
      const double v = dot(sub(q, mu), d);
      lo = std::min(lo, v); hi = std::max(hi, v);
    }
    const double span = hi - lo;
    if (!(0.12 <= span && span <= 2 * h + 0.03)) {
      why = fmt("조각 길이 %.0f cm", span * 100);
      continue;
    }
    int beyond = 0;                                     // 끝 밖 2–14 cm 에 같은 직선 점 = 벽
    for (const auto & p : P) {
      const Vec2 r = sub(p, mu);
      if (std::abs(dot(r, nn)) >= tol_) {continue;}
      const double sa = dot(r, d);
      if ((sa > hi + 0.02 && sa < hi + 0.14) || (sa < lo - 0.02 && sa > lo - 0.14)) {++beyond;}
    }
    if (beyond >= 4) {
      why = "면 밖으로 이어짐 (벽)";
      continue;
    }
    if (first && end_check) {                           // 양 끝이 다 가려짐 = 앞 물체 사이로 보인 벽일 수 있음 (박스 모서리 확인 못 함)
      const Vec2 e_hi{mu.x + hi * d.x, mu.y + hi * d.y}, e_lo{mu.x + lo * d.x, mu.y + lo * d.y};
      if (end_hidden(P, robot_xy, e_hi, d) && end_hidden(P, robot_xy, e_lo, {-d.x, -d.y})) {
        why = "양 끝 가려짐";
        continue;
      }
    }
    const Vec2 mid{mu.x + 0.5 * (lo + hi) * d.x, mu.y + 0.5 * (lo + hi) * d.y};
    if (dot(nn, sub(robot_xy, mid)) < 0) {nn = {-nn.x, -nn.y};}
    // 거의 옆에서 (면과 12° 안) 본 조각 = 박스 옆면을 뒤에서 본 것일 수 있음 (중심이 반대로 나옴) → 안 씀 (면 대칭 접기에서 생긴 경우)
    if (const Vec2 rm = sub(robot_xy, mid); dot(nn, rm) < std::sin(12.0 * M_PI / 180.0) * norm(rm)) {
      why = "옆에서 본 면";
      continue;
    }
    // 정사각 박스: 보인 면 (thm_face) 을 기준 방향 th_ref 에서 ±45° 안의 면 (thm) 으로 접음. 중심은 보인 면으로 구함 (어느 면이든 같은 중심)
    const double thm_face = wrap(std::atan2(-nn.y, -nn.x) - yaw0);
    const double thm = th_ref + fold_quarter(thm_face - th_ref);
    const double tilt = first ? std::abs(fold_quarter(thm_face)) : std::abs(wrap(thm - th_ref));
    if (tilt > th_tol) {
      why = fmt("기울기 %.0f°", deg(thm_face));
      continue;
    }
    const bool full = span >= 2 * h - 0.03;
    Vec2 cc;
    if (full || first) {                                // 양 모서리 보임 (처음엔 조각 가운데로)
      cc = {mid.x - h * nn.x, mid.y - h * nn.y};
    } else {                                            // 한쪽만 → 옆은 이전 추정을 이 면에 투영
      const double t = dot(sub(c, mid), d);
      cc = {mid.x + t * d.x - h * nn.x, mid.y + t * d.y - h * nn.y};
    }
    const double dist = norm(sub(cc, center));
    if (dist > radius) {
      why = fmt(first ? "지도 자리에서 %.0f cm" : "추정에서 %.0f cm", dist * 100);
      continue;
    }
    const double score = dist - (full ? 0.05 : 0.0);    // 같으면 양 모서리 보인 조각
    if (!best || score < best->first) {
      best = std::make_pair(score, std::array<double, 3>{cc.x, cc.y, thm});
      line = {mu.x, mu.y, d.x, d.y, nn.x, nn.y, lo, hi};
      line_turn = thm - thm_face;
      line_c = cc;
    }
  }
  if (!best) {
    last_why = why;
    return std::nullopt;
  }
  // 면 띠 점 (fit_segments 용): 고른 직선 ±face_band · 양 끝 1 cm 안쪽 (모서리 · 옆면 점 뺌).
  //   조각 점 (직선 ±tol) 만 쓰면 잡음이 tol 과 비슷할 때 표본 두 점 쪽으로 치우친 점만 남음
  seg_.clear();
  const Vec2 lm{line[0], line[1]}, ld{line[2], line[3]}, ln{line[4], line[5]};
  for (const auto & q : Q) {
    const Vec2 r = sub(q, lm);
    const double sa = dot(r, ld);
    if (std::abs(dot(r, ln)) < face_band && sa > line[6] + 0.01 && sa < line[7] - 0.01) {
      seg_.push_back(line_turn == 0.0 ? q : [&] {const Vec2 v = rot(sub(q, line_c), line_turn); return Vec2{line_c.x + v.x, line_c.y + v.y};}());
    }
  }
  return best->second;
}

// ================================================================ 접근

DockCore::DockCore(Vec2 goal_xy, double goal_yaw, const DockParams & p)
: g_xy_(goal_xy), g_yaw_(goal_yaw), p_(p)
{
  if (p.box_track) {tr_.emplace(goal_xy, goal_yaw, p.box_dist, p.box_half, p.search);}
  align_ = p.align_deg * M_PI / 180.0;
  tol_yaw_ = p.tol_yaw_deg * M_PI / 180.0;
  tol_yaw_ok_ = p.tol_yaw_ok_deg * M_PI / 180.0;
  // 띠 너비: 가장 빠른 몸 점 (병진 + 회전 × 모서리 반지름) 이 t_react + 스캔 나이 (노드가 0.3 s 까지 받음) 동안 가는 거리까지
  band_ = p.margin + (p.t_react + 0.3) * (p.v_max + p.w_max * std::hypot(p.half_x, p.half_y));
}

Pose2 DockCore::target() const
{
  if (!tr_) {return {g_xy_.x, g_xy_.y, g_yaw_};}
  return tr_->goal();
}

std::pair<Vec2, double> DockCore::box_pose() const
{
  if (!tr_) {return {{g_xy_.x + p_.box_dist * std::cos(g_yaw_), g_xy_.y + p_.box_dist * std::sin(g_yaw_)}, g_yaw_};}
  return {tr_->c, wrap(tr_->yaw0 + tr_->th)};
}

void DockCore::update_box(const std::vector<Vec2> & pts_odom, Vec2 robot_xy)
{
  if (tr_ && !pts_odom.empty()) {tr_->update(pts_odom, robot_xy);}
}

// 몸 둘레 band 띠 안 · 박스 아닌 점 (몸 좌표). keep_box = 박스 점도 다 막는 점 (제자리 회전: 몸 모서리가 박스 면을 쓸고 감)
std::vector<Vec2> DockCore::near_points(const std::vector<Vec2> & raw_body, const Pose2 & pose, bool keep_box) const
{
  std::vector<Vec2> P;
  for (const auto & q : raw_body) {
    if (std::abs(q.x) < p_.self_in && std::abs(q.y) < p_.self_in) {continue;}
    if (std::abs(q.x) < p_.half_x + band_ && std::abs(q.y) < p_.half_y + band_) {P.push_back(q);}
  }
  if (P.empty() || keep_box) {return P;}
  const auto [c, byaw] = box_pose();
  const double cr = std::cos(pose.yaw), sr = std::sin(pose.yaw);
  const Vec2 cb{cr * (c.x - pose.x) + sr * (c.y - pose.y), -sr * (c.x - pose.x) + cr * (c.y - pose.y)};
  const double a = byaw - pose.yaw, ca = std::cos(a), sa = std::sin(a);
  const auto to_box = [&](const Vec2 & v) {return Vec2{v.x * ca + v.y * sa, -v.x * sa + v.y * ca};};   // 행 벡터 × 회전
  const double lim = p_.box_half + p_.box_excl;
  const Vec2 r = to_box({-cb.x, -cb.y});               // 로봇 중심 (박스 좌표)
  if (!(r.x < -p_.box_half && std::abs(r.y) < p_.half_y)) {return P;}   // 박스 앞면 정면이 아님 → 박스 점도 막는 점
  std::vector<Vec2> out;
  for (const auto & q : P) {
    const Vec2 b = to_box(sub(q, cb));
    const bool face = b.x > -lim && b.x < -p_.box_half + p_.face_depth && std::abs(b.y) < lim;   // 로봇 쪽 면 앞뒤 띠만
    if (!face) {out.push_back(q);}
  }
  return out;
}

// cmd 로 움직이면 다가가는 가까운 점: 몸에서 가장 가까운 점의 속도 · 점 쪽 방향 내적 > eps 이고 틈 < margin + 그 속도 × (t_react + 나이)
std::vector<Vec2> DockCore::blockers(const std::vector<Vec2> & near, const std::array<double, 3> & cmd) const
{
  std::vector<Vec2> out;
  for (const auto & q : near) {
    const Vec2 b{std::clamp(q.x, -p_.half_x, p_.half_x), std::clamp(q.y, -p_.half_y, p_.half_y)};
    Vec2 d = sub(q, b);
    const double n = norm(d);
    d = {d.x / std::max(n, 1e-6), d.y / std::max(n, 1e-6)};
    const Vec2 vb{cmd[0] - cmd[2] * b.y, cmd[1] + cmd[2] * b.x};
    const double closing = dot(vb, d);
    if (closing > p_.toward_eps && n < p_.margin + closing * (p_.t_react + age_)) {out.push_back(q);}
  }
  return out;
}

bool DockCore::safe(const std::vector<Vec2> & near, const std::array<double, 3> & cmd) const
{
  return static_cast<int>(blockers(near, cmd).size()) < p_.min_hits;
}

std::string DockCore::describe(const std::vector<Vec2> & P) const
{
  if (P.empty()) {return "점 0";}
  const auto score = [&](const Vec2 & q) {return std::max(std::abs(q.x) - p_.half_x, std::abs(q.y) - p_.half_y);};
  const Vec2 q = *std::min_element(P.begin(), P.end(), [&](const Vec2 & a, const Vec2 & b) {return score(a) < score(b);});
  const char * side = q.x > p_.half_x ? "앞" : q.x < -p_.half_x ? "뒤" : q.y > 0 ? "왼쪽" : "오른쪽";
  char b[160];
  std::snprintf(b, sizeof(b), "%s 몸에서 %.1f cm (몸 중심 기준 (%+.3f, %+.3f) m, 점 %zu)", side, score(q) * 100, q.x, q.y, P.size());
  return b;
}

std::pair<std::array<double, 3>, DockState> DockCore::step(const Pose2 & pose, const std::vector<Vec2> & raw_body, double t,
  double raw_age)
{
  const std::array<double, 3> zero{0, 0, 0};
  age_ = std::max(0.0, raw_age);
  const Pose2 g = target();
  const double ex = g.x - pose.x, ey = g.y - pose.y, c = std::cos(pose.yaw), s = std::sin(pose.yaw);
  const Vec2 eb{c * ex + s * ey, -s * ex + c * ey};    // 몸 좌표 남은 위치
  const double dist = norm(eb);
  const double dpsi = wrap(g.yaw - pose.yaw);
  char b[160];
  if (dist < p_.tol_pos && std::abs(dpsi) < tol_yaw_) {
    std::snprintf(b, sizeof(b), "도착 (남은 %.1f cm · %+.1f°)", dist * 100, deg(dpsi));
    why = b;
    return {zero, DockState::Done};
  }
  if (dist < best_ - 0.001) {best_ = dist; t_best_ = t;}
  if (dist < p_.tol_ok && std::abs(dpsi) < tol_yaw_ok_ && t - t_best_ > p_.stall_s) {
    std::snprintf(b, sizeof(b), "도착 (더 안 줄어듦 %.1f s, 남은 %.1f cm · %+.1f°)", p_.stall_s, dist * 100, deg(dpsi));
    why = b;
    return {zero, DockState::Done};
  }
  auto near = near_points(raw_body, pose);
  if (!aligned_ && std::abs(dpsi) > align_) {          // 1. 제자리 회전 (박스 면도 막는 점 — 박스 앞 2 cm 에서 30° 돌면 모서리가 파고듦)
    near = near_points(raw_body, pose, true);
    const double w = std::copysign(std::max(p_.w_min, std::min(p_.w_max, p_.k_w * std::abs(dpsi))), dpsi);
    const std::array<double, 3> cmd{0.0, 0.0, w};
    if (safe(near, cmd)) {
      mode = "돌기";
      return {cmd, DockState::Run};
    }
    const auto blk = blockers(near, cmd);
    const Vec2 q = *std::min_element(blk.begin(), blk.end(), [](const Vec2 & a, const Vec2 & b) {return norm(a) < norm(b);});
    const Vec2 bb{std::clamp(q.x, -p_.half_x, p_.half_x), std::clamp(q.y, -p_.half_y, p_.half_y)};
    const Vec2 away = sub(bb, q);
    const double an = std::max(norm(away), 1e-6);
    const std::array<double, 3> esc{away.x / an * p_.v_esc, away.y / an * p_.v_esc, 0.0};
    if (safe(near, esc)) {
      mode = "돌기 막힘 → 떨어지기";
      return {esc, DockState::Run};
    }
    why = "돌 자리 없음: " + describe(blk);
    return {zero, DockState::Fail};
  }
  aligned_ = true;
  // 2. 다가가기
  double v = std::min(p_.v_max, p_.k_v * dist);
  Vec2 vb{0, 0};
  if (dist > p_.tol_pos) {
    v = std::max(v, p_.v_min);
    vb = {eb.x / dist * v, eb.y / dist * v};
  }
  double w = std::max(-p_.w_fine, std::min(p_.w_fine, p_.k_w * dpsi));
  if (std::abs(dpsi) < tol_yaw_) {
    w = 0.0;
  } else if (std::abs(w) < p_.w_min / 2) {
    w = std::copysign(p_.w_min / 2, dpsi);
  }
  const std::array<double, 3> cmd{vb.x, vb.y, w};
  if (safe(near, cmd)) {
    mode = "다가가기";
    return {cmd, DockState::Run};
  }
  const std::array<double, 3> cmd0{cmd[0], cmd[1], 0.0};
  const bool moving = std::hypot(cmd[0], cmd[1]) > 0;
  if (moving && safe(near, cmd0)) {
    mode = "다가가기 (회전 뺌)";
    return {cmd0, DockState::Run};
  }
  const auto blk = blockers(near, moving ? cmd0 : cmd);
  if (!blk.empty() && moving) {
    Vec2 d{0, 0};
    for (const auto & q : blk) {
      d.x += q.x - std::clamp(q.x, -p_.half_x, p_.half_x);
      d.y += q.y - std::clamp(q.y, -p_.half_y, p_.half_y);
    }
    d = {d.x / blk.size(), d.y / blk.size()};
    const double dn = std::max(norm(d), 1e-6);
    d = {d.x / dn, d.y / dn};
    Vec2 v2{cmd0[0], cmd0[1]};
    const double into = std::max(0.0, dot(v2, d));
    v2 = {v2.x - into * d.x, v2.y - into * d.y};
    const std::array<double, 3> slide{v2.x, v2.y, 0.0};
    if (norm(v2) > p_.v_min / 2 && safe(near, slide)) {
      mode = "다가가기 (막힌 쪽 빼고 미끄러지기)";
      return {slide, DockState::Run};
    }
  }
  why = "막힘: " + describe(blk.empty() ? near : blk);
  return {zero, DockState::Fail};
}

}  // namespace navigation::dock
