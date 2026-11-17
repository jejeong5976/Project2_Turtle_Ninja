// barcode_search.cpp — 출발 구역 바코드 찾기, 설명은 barcode_search.hpp

#include "mission/barcode_search.hpp"

#include <algorithm>
#include <cstdio>

namespace mission
{
namespace bc
{

namespace
{
constexpr double kDeg = M_PI / 180.0;
double wrap(double a) {return std::atan2(std::sin(a), std::cos(a));}
std::string f1(const char * fmt, double a, double b = 0.0, double c = 0.0, double d = 0.0)
{
  char buf[200];
  std::snprintf(buf, sizeof(buf), fmt, a, b, c, d);
  return buf;
}
}  // namespace

V3 rotate(const Pose3 & T, V3 v)
{
  return {T.R[0][0] * v.x + T.R[0][1] * v.y + T.R[0][2] * v.z, T.R[1][0] * v.x + T.R[1][1] * v.y + T.R[1][2] * v.z,
    T.R[2][0] * v.x + T.R[2][1] * v.y + T.R[2][2] * v.z};
}

V3 apply(const Pose3 & T, V3 p) {return rotate(T, p) + T.t;}

Pose3 compose(const Pose3 & a, const Pose3 & b)
{
  Pose3 o;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      o.R[i][j] = a.R[i][0] * b.R[0][j] + a.R[i][1] * b.R[1][j] + a.R[i][2] * b.R[2][j];
    }
  }
  o.t = apply(a, b.t);
  return o;
}

Pose3 planar(double x, double y, double yaw, double z)
{
  Pose3 T;
  const double c = std::cos(yaw), s = std::sin(yaw);
  T.R = {{{c, -s, 0}, {s, c, 0}, {0, 0, 1}}};
  T.t = {x, y, z};
  return T;
}

Pose3 from_quat(double x, double y, double z, double w, V3 t)
{
  const double n = std::sqrt(x * x + y * y + z * z + w * w);
  if (n > 1e-9) {x /= n; y /= n; z /= n; w /= n;}
  Pose3 T;
  T.R = {{{1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)},
    {2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)},
    {2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)}}};
  T.t = t;
  return T;
}

Pose3 pitch_optical(const Pose3 & T, double down_rad)
{
  Pose3 rx;   // Rx(−θ): 광축 z → (0, sin θ, cos θ) = 아래 (광학 y = 아래)
  const double c = std::cos(-down_rad), s = std::sin(-down_rad);
  rx.R = {{{1, 0, 0}, {0, c, -s}, {0, s, c}}};
  return compose(T, rx);
}

Pose3 yaw_optical(const Pose3 & T, double left_rad)
{
  Pose3 ry;   // Ry(−θ): 광축 z → (−sin θ, 0, cos θ) = 왼쪽 (광학 x = 오른쪽)
  const double c = std::cos(-left_rad), s = std::sin(-left_rad);
  ry.R = {{{c, 0, s}, {0, 1, 0}, {-s, 0, c}}};
  return compose(T, ry);
}

V3 fisheye_ray(const Fisheye & c, double u, double v)
{
  const double xd = (u - c.cx) / c.fx, yd = (v - c.cy) / c.fy;
  const double td = std::sqrt(xd * xd + yd * yd);
  if (td < 1e-9) {return {0.0, 0.0, 1.0};}
  double th = td;
  for (int i = 0; i < 20; ++i) {   // θ (1 + k1θ² + …) = θd 를 Newton 으로
    const double t2 = th * th;
    const double f = th * (1 + c.k[0] * t2 + c.k[1] * t2 * t2 + c.k[2] * t2 * t2 * t2 + c.k[3] * t2 * t2 * t2 * t2) - td;
    const double df = 1 + 3 * c.k[0] * t2 + 5 * c.k[1] * t2 * t2 + 7 * c.k[2] * t2 * t2 * t2 + 9 * c.k[3] * t2 * t2 * t2 * t2;
    th -= f / df;
  }
  const double s = std::sin(th) / td;
  return unit({xd * s, yd * s, std::cos(th)});
}

bool fisheye_project(const Fisheye & c, V3 ray, double & u, double & v)
{
  const double r = std::hypot(ray.x, ray.y);
  const double th = std::atan2(r, ray.z);
  if (th > 100.0 * kDeg) {return false;}
  const double t2 = th * th;
  const double td = th * (1 + c.k[0] * t2 + c.k[1] * t2 * t2 + c.k[2] * t2 * t2 * t2 + c.k[3] * t2 * t2 * t2 * t2);
  const double sx = r > 1e-12 ? ray.x / r : 0.0, sy = r > 1e-12 ? ray.y / r : 0.0;
  u = c.fx * td * sx + c.cx;
  v = c.fy * td * sy + c.cy;
  return true;
}

V3 pinhole_ray(const Pinhole & c, double u, double v) {return unit({(u - c.cx) / c.fx, (v - c.cy) / c.fy, 1.0});}

FovEdges fisheye_edges(const Fisheye & c)
{
  FovEdges e;
  const V3 l = fisheye_ray(c, 0.0, c.cy), r = fisheye_ray(c, c.w - 1.0, c.cy);
  const V3 t = fisheye_ray(c, c.cx, 0.0), b = fisheye_ray(c, c.cx, c.h - 1.0);
  e.left = std::atan2(l.x, l.z);
  e.right = std::atan2(r.x, r.z);
  e.top = std::atan2(t.y, t.z);
  e.bottom = std::atan2(b.y, b.z);
  return e;
}

FovCheck light_in_fov(const Fisheye & c, const Pose3 & T_base_cam, double rx, double ry, double ryaw, V3 light_map, double margin_deg,
  double base_z)
{
  FovCheck out;
  const Pose3 Tm = compose(planar(rx, ry, ryaw, base_z), T_base_cam);
  const V3 d = light_map - Tm.t;
  // 광학 프레임으로: Rᵀ d
  const V3 q{Tm.R[0][0] * d.x + Tm.R[1][0] * d.y + Tm.R[2][0] * d.z, Tm.R[0][1] * d.x + Tm.R[1][1] * d.y + Tm.R[2][1] * d.z,
    Tm.R[0][2] * d.x + Tm.R[1][2] * d.y + Tm.R[2][2] * d.z};
  if (q.z <= 1e-6) {return out;}
  const auto e = fisheye_edges(c);
  const double h = std::atan2(q.x, q.z), v = std::atan2(q.y, q.z);
  const double m = std::min(h - e.left, e.right - h) / kDeg;
  out.margin_deg = m;
  out.ok = m >= margin_deg && v > e.top && v < e.bottom;
  return out;
}

double scan_range(const std::vector<std::array<double, 2>> & pts, double ox, double oy, double az, double half, double rmin, double rmax)
{
  std::vector<double> r;
  for (const auto & p : pts) {
    const double dx = p[0] - ox, dy = p[1] - oy;
    const double d = std::hypot(dx, dy);
    if (d < rmin || d > rmax) {continue;}
    if (std::fabs(wrap(std::atan2(dy, dx) - az)) <= half) {r.push_back(d);}
  }
  if (r.size() < 3) {return kNaN;}
  std::nth_element(r.begin(), r.begin() + r.size() / 2, r.end());
  return r[r.size() / 2];
}

double wall_view_yaw(const std::vector<std::array<double, 2>> & pts, double ox, double oy, double hx, double hy, double radius)
{
  double mx = 0.0, my = 0.0;
  int n = 0;
  for (const auto & p : pts) {
    if (std::hypot(p[0] - hx, p[1] - hy) <= radius) {mx += p[0]; my += p[1]; ++n;}
  }
  if (n < 5) {return kNaN;}
  mx /= n;
  my /= n;
  double sxx = 0.0, sxy = 0.0, syy = 0.0;
  for (const auto & p : pts) {
    if (std::hypot(p[0] - hx, p[1] - hy) > radius) {continue;}
    const double dx = p[0] - mx, dy = p[1] - my;
    sxx += dx * dx; sxy += dx * dy; syy += dy * dy;
  }
  const double ang = 0.5 * std::atan2(2 * sxy, sxx - syy);   // 벽 방향
  double nx = -std::sin(ang), ny = std::cos(ang);            // 법선
  if (nx * (ox - mx) + ny * (oy - my) < 0) {nx = -nx; ny = -ny;}   // 로봇 쪽
  return std::atan2(-ny, -nx);                               // 벽을 보는 방향
}

bool ray_at_range(V3 origin, V3 dir, double range_h, V3 & out)
{
  const double dxy = std::hypot(dir.x, dir.y);
  if (dxy < 1e-6 || !(range_h > 0.0)) {return false;}
  out = origin + dir * (range_h / dxy);
  return true;
}

bool ray_plane(V3 origin, V3 dir, V3 p0, double n_yaw, V3 & out)
{
  const V3 n{std::cos(n_yaw), std::sin(n_yaw), 0.0};
  const double den = dot(dir, n);
  if (std::fabs(den) < 1e-6) {return false;}
  const double s = dot(p0 - origin, n) / den;
  if (s <= 0.0) {return false;}
  out = origin + dir * s;
  return true;
}

Zone zone_from_arena(double x0, double x1, double y0, double y1, double mx, double my, double myaw)
{
  Zone z;
  const double c = std::cos(myaw), s = std::sin(myaw);
  const std::array<std::array<double, 2>, 4> a{{{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}}};
  for (int i = 0; i < 4; ++i) {z.c[i] = {mx + c * a[i][0] - s * a[i][1], my + s * a[i][0] + c * a[i][1]};}
  z.valid = x1 > x0 && y1 > y0;
  return z;
}

bool in_zone(const Zone & z, double x, double y, double margin)
{
  if (!z.valid) {return true;}   // 구역 모름 = 막지 않음
  for (int i = 0; i < 4; ++i) {
    const auto & a = z.c[i];
    const auto & b = z.c[(i + 1) % 4];
    const double ex = b[0] - a[0], ey = b[1] - a[1];
    const double len = std::hypot(ex, ey);
    const double side = (ex * (y - a[1]) - ey * (x - a[0])) / std::max(len, 1e-9);   // 반시계 다각형 안 = +
    if (side < margin) {return false;}
  }
  return true;
}

double standoff_for_ppm(double bbox_w_px, int modules, double dist_now, double min_ppm)
{
  if (!(bbox_w_px > 0.0) || modules <= 0 || !(min_ppm > 0.0)) {return dist_now;}
  return dist_now * (bbox_w_px / modules) / min_ppm;
}

// ================================================================ BarcodeSearch

void BarcodeSearch::reset(const SearchParams & p, const Fisheye & wide, const Pinhole & wrist, const Zone & zone, double now)
{
  *this = BarcodeSearch{};
  p_ = p;
  wide_ = wide;
  wrist_ = wrist;
  zone_ = zone;
  t0_ = now;
}

void BarcodeSearch::update_light(const World & w)
{
  if (!w.have_pose) {return;}
  const Pose3 Tmb = planar(w.rx, w.ry, w.ryaw, p_.base_z_m);
  if (w.wide_light && w.have_wide_tf && w.now - w.wide_light->stamp <= p_.wide_fresh_s) {
    const V3 dir_b = rotate(w.T_base_wide, fisheye_ray(wide_, w.wide_light->u, w.wide_light->v));
    const V3 o_b = w.T_base_wide.t;
    double r = scan_range(w.scan_base, o_b.x, o_b.y, std::atan2(dir_b.y, dir_b.x), p_.scan_half_deg * kDeg, p_.range_min_m, p_.range_max_m);
    r = std::isfinite(r) ? r + p_.lidar_range_offset_m : p_.light_default_range_m;
    V3 l;
    if (ray_at_range(apply(Tmb, o_b), rotate(Tmb, dir_b), r, l)) {
      light_ = l;
      light_known_ = true;   // 볼 때마다 새로 (사람이 옮길 수 있음)
    }
    return;
  }
  if (!light_known_ && norm(light_) < 1e-9) {   // 아직 못 봄: 출발 자세 앞
    const double yaw = std::isfinite(w.start_yaw) ? w.start_yaw : w.ryaw;
    light_ = {w.rx + std::cos(yaw) * p_.light_default_range_m, w.ry + std::sin(yaw) * p_.light_default_range_m, p_.light_default_height_m};
  }
}

std::optional<V3> BarcodeSearch::target_from_wide(const World & w, double * view_yaw) const
{
  if (!w.wide_barcode || !w.have_wide_tf || !w.have_pose || w.now - w.wide_barcode->stamp > p_.wide_fresh_s) {return std::nullopt;}
  const auto & b = *w.wide_barcode;
  const V3 dir_b = rotate(w.T_base_wide, fisheye_ray(wide_, b.u, b.v));
  const V3 o_b = w.T_base_wide.t;
  const double az = std::atan2(dir_b.y, dir_b.x);
  double r = scan_range(w.scan_base, o_b.x, o_b.y, az, p_.scan_half_deg * kDeg, p_.range_min_m, p_.range_max_m);
  const Pose3 Tmb = planar(w.rx, w.ry, w.ryaw, p_.base_z_m);
  const V3 pc = p_.panel ? panel_center(w) : V3{};
  V3 t;
  bool have_t = false;
  if (std::isfinite(r)) {
    r += p_.lidar_range_offset_m;
  } else if (p_.panel) {   // 벽 판: 라이다가 없으면 판 면과 광선
    have_t = ray_plane(apply(Tmb, o_b), rotate(Tmb, dir_b), pc, p_.panel_n_yaw, t);
    if (!have_t) {r = p_.default_range_m;}
  } else if (b.w > 1.0 && p_.barcode_width_m > 0.0) {   // 상자 폭 → 광선 거리 → 수평
    r = wide_.fx * p_.barcode_width_m / b.w * std::hypot(dir_b.x, dir_b.y);
  } else {
    r = p_.default_range_m;
  }
  if (have_t) {
    r = std::hypot(t.x - apply(Tmb, o_b).x, t.y - apply(Tmb, o_b).y);
  } else if (!ray_at_range(apply(Tmb, o_b), rotate(Tmb, dir_b), r, t)) {
    return std::nullopt;
  }
  if (t.z < p_.z_min_m || t.z > p_.z_max_m) {return std::nullopt;}
  if (p_.panel && !in_panel(t, pc)) {   // 벽 판 밖 상자 (다른 곳 글자 · 잘못 본 것) → 버림
    ++panel_rejects_;
    return std::nullopt;
  }
  if (view_yaw) {
    const V3 hit_b = o_b + dir_b * (r / std::max(std::hypot(dir_b.x, dir_b.y), 1e-6));
    double vy = wall_view_yaw(w.scan_base, o_b.x, o_b.y, hit_b.x, hit_b.y, 0.15);
    const double ray_az = az;
    if (!std::isfinite(vy)) {vy = p_.panel ? wrap(p_.panel_n_yaw + M_PI - w.ryaw) : ray_az;}   // 벽 판: 판 정면
    const double lim = p_.max_obliquity_deg * kDeg;
    vy = ray_az + std::clamp(wrap(vy - ray_az), -lim, lim);
    *view_yaw = wrap(vy + w.ryaw);
  }
  return t;
}

std::optional<V3> BarcodeSearch::target_from_wrist(const World & w, double * dist) const
{
  if (!w.wrist_barcode) {return std::nullopt;}
  if (!w.have_wrist_tf) {return std::nullopt;}
  const V3 o = w.T_map_wrist.t;
  // 손목 깊이 점은 그럴듯할 때만 1 순위 — 지금 목표 벽 면 (목표 지남, 보는 방향 법선) 에서 depth_plane_tol 안 · 카메라에서 [depth_min, depth_max] · 높이 범위
  auto plausible = [&](const V3 & q, double tol) {
      const double dc = norm(q - o);
      if (dc < p_.depth_min_m || dc > p_.depth_max_m || q.z < p_.z_min_m || q.z > p_.z_max_m) {return false;}
      if (!cur_) {return true;}
      const double ny = std::isfinite(cur_->view_yaw) ? cur_->view_yaw : std::atan2(cur_->target.y - o.y, cur_->target.x - o.x);
      return std::fabs(dot(q - cur_->target, V3{std::cos(ny), std::sin(ny), 0.0})) <= tol;
    };
  if (w.wrist_barcode_map && plausible(*w.wrist_barcode_map, p_.depth_plane_tol_m)) {   // 손목 깊이 = 1 순위
    if (dist) {*dist = norm(*w.wrist_barcode_map - o);}
    return *w.wrist_barcode_map;
  }
  const V3 d = rotate(w.T_map_wrist, pinhole_ray(wrist_, w.wrist_barcode->u, w.wrist_barcode->v));
  V3 p0;
  double ny;
  if (w.wrist_center_map && plausible(*w.wrist_center_map, 2.0 * p_.depth_plane_tol_m)) {   // 화면 가운데 깊이 = 벽 자리 (그럴듯할 때만)
    p0 = *w.wrist_center_map;
    ny = cur_ && std::isfinite(cur_->view_yaw) ? cur_->view_yaw : std::atan2(p0.y - o.y, p0.x - o.x);
  } else if (cur_) {
    p0 = cur_->target;
    ny = std::isfinite(cur_->view_yaw) ? cur_->view_yaw : std::atan2(p0.y - o.y, p0.x - o.x);
  } else {
    return std::nullopt;
  }
  V3 t;
  if (!ray_plane(o, d, p0, ny, t)) {return std::nullopt;}
  if (dist) {*dist = norm(t - o);}
  if (t.z < p_.z_min_m || t.z > p_.z_max_m) {return std::nullopt;}
  return t;
}

bool BarcodeSearch::seen_before(V3 t) const
{
  for (const auto & q : tried_) {if (norm(q - t) < p_.dedupe_m) {return true;}}
  return false;
}

V3 BarcodeSearch::panel_center(const World & w) const
{
  V3 c = p_.panel_c;
  if (!p_.panel || !w.have_pose || w.scan_base.empty()) {return c;}
  // 판 가운데 쪽 라이다 벽 거리 → 판 면을 법선 따라 맞춤 (지도 맞춤 오차 · 벽 자리)
  const double cy = std::cos(w.ryaw), sy = std::sin(w.ryaw);
  const double dx = c.x - w.rx, dy = c.y - w.ry;
  const double bx = cy * dx + sy * dy, by = -sy * dx + cy * dy;
  const double ox = 0.0225;   // J1 축 (base_link)
  const double az = std::atan2(by, bx - ox);
  double r = scan_range(w.scan_base, ox, 0.0, az, p_.scan_half_deg * kDeg, p_.range_min_m, p_.range_max_m);
  if (!std::isfinite(r)) {return c;}
  r += p_.lidar_range_offset_m;
  const double hbx = ox + r * std::cos(az), hby = r * std::sin(az);
  const V3 hit{w.rx + cy * hbx - sy * hby, w.ry + sy * hbx + cy * hby, c.z};
  const V3 n{std::cos(p_.panel_n_yaw), std::sin(p_.panel_n_yaw), 0.0};
  const double off = dot(hit - c, n);   // + = 라이다 벽이 판 (지도) 보다 로봇 쪽
  // 라이다가 더 가깝게 보면 늘 그쪽 (판 뒤 = 벽 속을 보지 않게, 가까운 쪽 틀림은 카메라가 멀어질 뿐), 더 멀면 panel_depth_tol 안만
  if (off >= -p_.panel_depth_tol_m) {c = c + n * off;}
  return c;
}

bool BarcodeSearch::in_panel(V3 t, V3 c, double * lat) const
{
  const V3 n{std::cos(p_.panel_n_yaw), std::sin(p_.panel_n_yaw), 0.0};
  const V3 left{std::sin(p_.panel_n_yaw), -std::cos(p_.panel_n_yaw), 0.0};   // 벽을 보는 로봇의 왼쪽
  const V3 d = t - c;
  const double s = dot(d, left);
  if (lat) {*lat = s;}
  return std::fabs(s) <= 0.5 * p_.panel_w_m + p_.panel_margin_m && std::fabs(d.z) <= 0.5 * p_.panel_h_m + p_.panel_margin_m &&
         std::fabs(dot(d, n)) <= p_.panel_depth_tol_m;
}

Action BarcodeSearch::wait(double secs, const std::string & note)
{
  Action a;
  a.kind = Act::Dwell;
  a.dwell_s = secs;
  a.note = note;
  note_ = note;
  return a;
}

// 몸 멈춤 때문의 기다림: 다음 next 에서 이 시간을 held_s_ 에 더함 (max_s 에서 뺌, hold_max_s 까지)
Action BarcodeSearch::hold_wait(const World & w, double secs, const std::string & note)
{
  hold_since_ = w.now;
  return wait(secs, note);
}

// 몸 멈춤으로 못 옮기는 목표 → 멈춤이 풀리면 (기록 끝) 미룬 목표 먼저 (다가가기 · 몸 옮기기 그대로)
void BarcodeSearch::defer_held(Cand c)
{
  c.held = true;
  c.perturb_i = 0;
  deferred_.push_back(c);
}

Action BarcodeSearch::look(const Cand & c, const World & w, double dj1, double dj4, double dd, bool sweep, const std::string & note)
{
  Action a;
  a.kind = Act::LookAt;
  a.target = c.target;
  a.standoff = std::clamp(c.standoff + dd, p_.standoff_min_m, sweep ? std::max(p_.standoff_max_m, p_.sweep_standoff_m) : p_.standoff_max_m);
  a.view_yaw = c.view_yaw;
  a.dj1_deg = dj1;
  a.dj4_deg = dj4;
  a.sweep = sweep;
  a.dwell_s = sweep ? p_.sweep_dwell_s : p_.dwell_s;
  a.light = !w.signal_done;
  a.light_map = light_;
  a.note = note + f1(" → (%.2f, %.2f, 높이 %.2f) d %.2f", c.target.x, c.target.y, c.target.z, a.standoff);
  cur_ = c;
  cur_sweep_ = sweep;
  cur_perturb_ = dj1 != 0.0 || dj4 != 0.0 || dd != 0.0;
  last_look_s_ = w.now;
  ++looks_;
  note_ = a.note;
  return a;
}

std::optional<Action> BarcodeSearch::base_move_for(const Cand & c, const World & w, bool * light_blocked, double want)
{
  if (light_blocked) {*light_blocked = false;}
  if (!w.have_pose) {return std::nullopt;}
  const double dx = c.target.x - w.rx, dy = c.target.y - w.ry;
  const double d = std::hypot(dx, dy);
  if (d < 1e-3) {return std::nullopt;}
  const bool want_given = want > 0.0;
  if (!want_given) {want = p_.approach_dist_m;}
  // 벽 판: 목표 정면 (판 법선 따라 want) — 앞 · 옆으로 작게, 몸 방향은 그대로
  const double nx = p_.panel ? c.target.x + std::cos(p_.panel_n_yaw) * want : c.target.x - dx / d * want;
  const double ny = p_.panel ? c.target.y + std::sin(p_.panel_n_yaw) * want : c.target.y - dy / d * want;
  // 지금 자리 → 바라는 자리 사이에서 출발 구역 안 가장 먼 곳
  double best = 0.0;
  for (int i = 1; i <= 20; ++i) {
    const double f = i / 20.0;
    if (in_zone(zone_, w.rx + (nx - w.rx) * f, w.ry + (ny - w.ry) * f, p_.robot_half_m)) {best = f;} else {break;}
  }
  const double mx = w.rx + (nx - w.rx) * best, my = w.ry + (ny - w.ry) * best;
  if (std::hypot(mx - w.rx, my - w.ry) < (want_given ? 0.02 : 0.04)) {return std::nullopt;}   // renudge 는 2 cm 부터
  double yaw = w.ryaw;
  if (!w.signal_done && w.have_wide_tf) {
    bool ok = false;
    for (double dyaw : {0.0, 10.0, -10.0, 20.0, -20.0}) {
      if (light_in_fov(wide_, w.T_base_wide, mx, my, w.ryaw + dyaw * kDeg, light_, p_.light_margin_deg, p_.base_z_m).ok) {
        yaw = w.ryaw + dyaw * kDeg;
        ok = true;
        break;
      }
    }
    if (!ok) {
      if (light_blocked) {*light_blocked = true;}
      return std::nullopt;
    }
  }
  Action a;
  a.kind = Act::MoveBase;
  a.bx = mx;
  a.by = my;
  a.byaw = wrap(yaw);
  a.dwell_s = 1.0;
  a.note = f1("닿지 않음 → 몸 옮김 (%.2f, %.2f) %.0f° (바코드까지 %.2f m)", mx, my, a.byaw / kDeg, std::hypot(c.target.x - mx, c.target.y - my));
  return a;
}

// 바코드가 멀면 LOOK_AT 전에 몸부터 다가감 (팔을 뻗지 않게). 옮긴 뒤 같은 목표를 줄 앞에
std::optional<Action> BarcodeSearch::maybe_approach(Cand & c, const World & w)
{
  if (!p_.approach_first || c.base_moved || base_moves_ >= p_.max_base_moves || !w.have_pose) {return std::nullopt;}
  if (holding(w)) {return std::nullopt;}   // 신호 기록 중: 다가가지 않고 지금 자리에서 팔로 봄 (닿지 않으면 미룸)
  if (std::hypot(c.target.x - w.rx, c.target.y - w.ry) <= p_.approach_dist_m + p_.approach_tol_m) {return std::nullopt;}
  auto m = base_move_for(c, w);
  if (!m) {return std::nullopt;}
  c.base_moved = true;
  c.perturb_i = 0;
  queue_.insert(queue_.begin(), c);
  ++base_moves_;
  m->note = "바코드가 멂 → 먼저 " + m->note;
  note_ = m->note;
  return m;
}

void BarcodeSearch::on_look_failed(const std::string & why, const World & w)
{
  if (!cur_) {return;}
  // control_master 꼬리표: LOOK_AT[OCCL] → 미룸, LOOK_AT[REACH] · [WALL] → 몸 옮김. 꼬리표 없는 글도 받음
  const bool occl = why.find("[OCCL]") != std::string::npos || why.find("가림") != std::string::npos || why.find("신호등") != std::string::npos;
  const bool reach = why.find("[REACH]") != std::string::npos || why.find("[WALL]") != std::string::npos ||
    why.find("범위") != std::string::npos || why.find("거리") != std::string::npos ||
    why.find("비스듬") != std::string::npos || why.find("광축") != std::string::npos || why.find("토크") != std::string::npos;
  if (cur_->locate && !occl) {   // 먼 보기가 안 됨 ([REACH] 등) → 같은 목표를 바로 읽기 보기 (0.15 m) 로
    Cand c = *cur_;
    c.locate = false;
    c.standoff = p_.standoff_m;
    queue_.insert(queue_.begin(), c);
    note_ = "먼 보기 안 됨 (" + why + ") → 바로 0.15 m 보기";
    cur_.reset();
    return;
  }
  if (occl && !w.signal_done) {
    deferred_.push_back(*cur_);
    note_ = "신호등 가림 → 신호 기록 끝까지 미룸: " + why;
    cur_.reset();
    return;
  }
  // 닿지 않음 → 몸 옮기기 (훑기는 안 함, 벽 판 훑기 자리는 함). 벽 판: 옮긴 뒤에도 닿지 않으면 renudge_m 더 가까이
  bool movable = reach && (!cur_sweep_ || cur_->panel_sweep) && base_moves_ < p_.max_base_moves && !cur_->base_moved;
  double want = -1.0;
  //   (읽기 보기 0.15 m 이 J2 토크 · '너무 뻗음' 으로 거절일 때만 — 훑기 · 관절 범위 거절은 너무 가까워서일 수도 있음)
  const bool too_far = why.find("토크") != std::string::npos || why.find("뻗음") != std::string::npos;
  if (reach && too_far && !cur_sweep_ && base_moves_ < p_.max_base_moves && cur_->base_moved && p_.panel && w.have_pose) {
    const double d = std::hypot(cur_->target.x - w.rx, cur_->target.y - w.ry);
    want = std::max(p_.approach_min_m, std::min(p_.approach_dist_m, d - p_.renudge_m));
    movable = d - want >= 0.02;
  }
  if (movable && holding(w)) {   // 몸 멈춤
    defer_held(*cur_);
    note_ = "닿지 않음 · 신호 기록 중 몸 멈춤 → 기록 끝까지 미룸: " + why;
    cur_.reset();
    return;
  }
  if (movable) {
    bool blocked = false;
    auto m = base_move_for(*cur_, w, &blocked, want);
    if (!m && blocked && !w.signal_done) {   // 신호등 조건 때문에 못 옮김 → 버리지 않고 신호 기록 끝까지 미룸
      deferred_.push_back(*cur_);
      note_ = "닿지 않음 · 신호등 조건으로 몸 못 옮김 → 신호 기록 끝까지 미룸: " + why;
      cur_.reset();
      return;
    }
    if (m) {
      Cand c = *cur_;
      c.base_moved = true;
      c.perturb_i = 0;
      queue_.insert(queue_.begin(), c);
      if (want > 0.0) {m->note = "옮긴 뒤에도 " + m->note;}
      pending_move_ = *m;
      ++base_moves_;
      cur_.reset();
      note_ = "닿지 않음 (" + why + ") → " + m->note;
      return;
    }
  }
  if (cur_perturb_ && !cur_sweep_) {   // 흔들기 하나 실패 → 다음 흔들기 (cur_ 그대로)
    note_ = "흔들기 실패 → 다음: " + why;
    return;
  }
  note_ = "목표 버림: " + why;
  cur_.reset();
}

void BarcodeSearch::on_move_failed(const std::string & why)
{
  note_ = "몸 옮기기 실패: " + why;
}

Action BarcodeSearch::next(const World & w)
{
  update_light(w);
  if (hold_since_ >= 0.0) {   // 몸 멈춤 기다림 → max_s 에서 뺌
    held_s_ += std::max(0.0, w.now - hold_since_);
    hold_since_ = -1.0;
  }
  if (search_s(w.now) > p_.max_s) {
    Action g;
    g.kind = Act::GiveUp;
    g.note = f1("찾기 시간 %.0f s 넘음", p_.max_s);
    note_ = g.note;
    return g;
  }
  if (pending_move_ && !holding(w)) {
    Action a = *pending_move_;
    pending_move_.reset();
    note_ = a.note;
    return a;
  }
  // 신호 기록 중인데 신호등을 아직 광각에서 못 봄 → light_wait_s 동안은 팔 접고 기다림 (가릴지 알 수 없음),
  //   그 뒤엔 광각 바코드 + 다가가기만 (돌기 · 훑기 · 흔들기 안 함 — 아래 restricted)
  const bool light_unknown = !w.signal_done && !light_known_;
  if (light_unknown && w.now - t0_ < p_.light_wait_s) {
    return wait(1.0, f1("신호 기록 중 · 신호등을 아직 광각에서 못 봄 → 기다림 (%.0f / %.0f s)", w.now - t0_, p_.light_wait_s));
  }
  const bool restricted = light_unknown;
  // 1) 방금 본 자리: 손목 상자 → 다시 겨눔 (가운데 · 끝 · 크기)
  if (cur_) {
    if (w.wrist_barcode) {cur_->saw_box = true;}
    if (w.wrist_barcode) {
      double d = 0.0;
      const auto t = target_from_wrist(w, &d);
      if (t) {
        const auto & b = *w.wrist_barcode;
        const double off = std::hypot(b.u - (wrist_.cx + p_.aim_du_px), b.v - (wrist_.cy + p_.aim_dv_px));   // 겨눔 화소에서
        const bool edge = b.u - b.w / 2 < p_.edge_margin_px || b.u + b.w / 2 > wrist_.w - p_.edge_margin_px ||
          b.v - b.h / 2 < p_.edge_margin_px || b.v + b.h / 2 > wrist_.h - p_.edge_margin_px;
        const double ppm = b.w / std::max(1, p_.modules);
        const bool small = ppm < p_.min_px_per_module || (p_.min_width_px > 0.0 && b.w < p_.min_width_px);
        if (cur_->refines < p_.refine_max && (cur_sweep_ || off > p_.center_tol_px || edge || small)) {
          Cand c = *cur_;
          c.target = *t;
          c.src = "wrist";
          c.locate = false;   // 먼 보기 → 읽기 보기 (0.15 m)
          c.panel_sweep = false;   // 판 훑기 자리 → 읽기 목표
          c.refines = cur_->refines + 1;
          c.perturb_i = 0;
          double st = cur_sweep_ ? p_.standoff_m : cur_->standoff;
          if (small) {st = std::min(st, std::min(standoff_for_ppm(b.w, p_.modules, d, p_.min_px_per_module), st - 0.03));}
          c.standoff = std::clamp(st, p_.standoff_min_m, p_.standoff_max_m);
          // 보는 방향은 그 목표의 벽 법선 그대로 (훑기 목표도 라이다 벽 법선)
          tried_.push_back(*t);
          if (auto m = maybe_approach(c, w)) {cur_.reset(); return *m;}
          return look(c, w, 0, 0, 0, false, f1("손목 상자 (가운데에서 %.0f px, %.1f px/모듈) → 다시 겨눔", off, ppm) +
                   (edge ? " · 화면 끝" : ""));
        }
      }
    }
    const bool no_box = p_.perturb_need_box && !cur_->saw_box;   // 상자를 한 번도 못 본 목표는 흔들지 않음
    if (!cur_sweep_ && !restricted && !no_box && cur_->perturb_i < p_.perturbs.size()) {
      const auto pr = p_.perturbs[cur_->perturb_i];
      Cand c = *cur_;
      ++c.perturb_i;
      return look(c, w, pr[0], pr[1], pr[2], false, f1("못 읽음 → 흔들기 %.0f (J1 %+.0f° J4 %+.0f° 거리 %+.2f)", static_cast<double>(c.perturb_i), pr[0], pr[1], pr[2]));
    }
    cur_.reset();
  }
  // 2) 줄 선 목표 (몸 옮긴 뒤 등)
  if (!queue_.empty()) {
    Cand c = queue_.front();
    queue_.erase(queue_.begin());
    if (auto m = maybe_approach(c, w)) {return *m;}
    return look(c, w, 0, 0, 0, c.locate || c.panel_sweep, "줄 선 목표 (" + c.src + (c.locate ? ", 먼 보기" : "") + ")");
  }
  // 3) 광각 바코드
  double vy = kNaN;
  if (const auto t = target_from_wide(w, &vy); t) {
    last_wide_z_ = t->z;
    if (p_.panel) {in_panel(*t, panel_center(w), &last_wide_lat_);}
  }
  if (const auto t = target_from_wide(w, &vy); t && !seen_before(*t)) {
    tried_.push_back(*t);
    Cand c;
    c.target = *t;
    c.view_yaw = vy;
    c.src = "wide";
    c.standoff = p_.standoff_m;
    if (p_.two_step) {   // 먼저 넓게 (먼 보기) → 손목 상자로 한 번 0.15 m (몸을 먼저 옮겨도 줄에 먼 보기로)
      c.locate = true;
      c.standoff = p_.locate_standoff_m;
    }
    if (auto m = maybe_approach(c, w)) {return *m;}
    return look(c, w, 0, 0, 0, c.locate, c.locate ? "광각 바코드 → 먼 보기 (손목으로 라벨 찾기)" : "광각 바코드");
  }
  // 광각 상자를 아직 충분히 못 받음 (시작 · 몸 옮긴 직후 · 끊김) → '광각에 없음' 판단을 미룸
  // 미룬 목표 (몸 멈춤 풀림 · 신호 기록 끝) 를 돌기 · 훑기보다 먼저 (자리를 이미 앎)
  if (!holding(w)) {
    for (auto it = deferred_.begin(); it != deferred_.end(); ++it) {
      if (!it->held && !w.signal_done) {continue;}   // 신호등 가림으로 미룬 것은 기록 끝 뒤만
      Cand c = *it;
      deferred_.erase(it);
      const std::string why = c.held ? "몸 멈춤 풀림 → 미룬 목표" : "신호 기록 끝 → 미룬 목표";
      c.held = false;
      if (auto m = maybe_approach(c, w)) {return *m;}
      return look(c, w, 0, 0, 0, c.locate || c.panel_sweep, why);
    }
  }
  if (!w.wide_ready) {return wait(0.5, "광각 상자 기다림 (시작 · 몸 옮긴 뒤 · 끊김)");}
  if (restricted) {   // 신호등을 모르는 채로는 돌기 · 훑기 안 함 (가릴 수 있음) → 신호 기록 끝 · 신호등 볼 때까지 기다림
    return wait(1.0, "신호등을 모름 (신호 기록 중) → 돌기 · 훑기 안 하고 기다림");
  }
  // 4) 제자리 돌기 (신호등이 광각 시야 안일 때만). 신호 기록 중 (몸 멈춤) 은 안 돌고 남겨 둠 → 훑기 먼저, 기록 끝 뒤 돌기
  if (w.have_pose && !holding(w) && !p_.panel) {   // 벽 판: 돌지 않음
    if (!std::isfinite(rot_base_yaw_)) {rot_base_yaw_ = w.ryaw;}
    while (rot_i_ < p_.rotate_steps_deg.size()) {
      const double yaw = wrap(rot_base_yaw_ + p_.rotate_steps_deg[rot_i_++] * kDeg);
      const auto fc = w.have_wide_tf ? light_in_fov(wide_, w.T_base_wide, w.rx, w.ry, yaw, light_, p_.light_margin_deg, p_.base_z_m) : FovCheck{true, 0.0};
      if (!w.signal_done && !fc.ok) {
        note_ = f1("돌기 %.0f° 안 함: 신호등 시야 여유 %.1f° < %.1f°", p_.rotate_steps_deg[rot_i_ - 1], fc.margin_deg, p_.light_margin_deg);
        continue;
      }
      Action a;
      a.kind = Act::MoveBase;
      a.bx = w.rx;
      a.by = w.ry;
      a.byaw = yaw;
      a.dwell_s = 1.0;
      a.note = f1("광각에 바코드 없음 → 제자리 %.0f° (신호등 여유 %.1f°)", p_.rotate_steps_deg[rot_i_ - 1], fc.margin_deg);
      note_ = a.note;
      return a;
    }
  }
  // 5') 벽 판 훑기: 판 따라 팔만 (광각이 본 가로 자리 먼저, 높이 = 광각이 본 높이 · 없으면 판 가운데). 멀면 판 정면으로 먼저 작게
  if (p_.panel) {
    std::vector<double> lats;
    if (std::isfinite(last_wide_lat_)) {lats.push_back(last_wide_lat_);}
    for (double s : p_.panel_sweep_m) {
      bool dup = false;
      for (double q : lats) {dup = dup || std::fabs(q - s) < 0.05;}
      if (!dup) {lats.push_back(s);}
    }
    while (w.have_pose && sweep_i_ < lats.size()) {
      const double s = lats[sweep_i_++];
      const V3 pc = panel_center(w);
      const V3 left{std::sin(p_.panel_n_yaw), -std::cos(p_.panel_n_yaw), 0.0};
      Cand c;
      c.target = pc + left * s;
      c.target.z = std::isfinite(last_wide_z_) ? last_wide_z_ : pc.z;
      c.view_yaw = wrap(p_.panel_n_yaw + M_PI);
      c.src = "panel";
      c.standoff = p_.locate_standoff_m;   // 벽 0.41 m 에서 0.28 m 보기 = J2 0.63 N·m (0.30 은 더 접힘)
      c.panel_sweep = true;
      if (auto m = maybe_approach(c, w)) {
        m->note = f1("벽 판 훑기 %+.2f m → 판 앞으로 ", s) + m->note;
        note_ = m->note;
        return *m;
      }
      return look(c, w, 0, 0, 0, true, f1("벽 판 훑기 %+.2f m 높이 %.2f", s, c.target.z));
    }
  }
  // 5) 손목 훑기
  // 광각이 본 높이가 있으면 그 하나만 (머리가 0.45 로 안 올라감)
  const std::vector<double> heights = std::isfinite(last_wide_z_) ? std::vector<double>{last_wide_z_} : p_.sweep_heights_m;
  const std::size_t nh = std::max<std::size_t>(1, heights.size());
  while (!p_.panel && w.have_pose && sweep_i_ < p_.sweep_az_deg.size() * nh) {
    // 훑기 방위는 찾기 시작 때 몸 방향 기준 (돌기 뒤 몸이 돌아 있어도 같은 벽 자리)
    const double az = wrap(p_.sweep_az_deg[sweep_i_ / nh] * kDeg + (std::isfinite(rot_base_yaw_) ? rot_base_yaw_ - w.ryaw : 0.0));
    const double h = heights.empty() ? 0.45 : heights[sweep_i_ % nh];
    ++sweep_i_;
    const double ox = 0.0225;   // J1 축 (base_link)
    double r = scan_range(w.scan_base, ox, 0.0, az, p_.scan_half_deg * kDeg, p_.range_min_m, p_.range_max_m);
    r = std::isfinite(r) ? std::min(r + p_.lidar_range_offset_m, 1.0) : p_.default_range_m;
    const Pose3 Tmb = planar(w.rx, w.ry, w.ryaw, 0.0);
    const V3 t = apply(Tmb, {ox + r * std::cos(az), r * std::sin(az), h});
    Cand c;
    c.target = t;
    double vy = wall_view_yaw(w.scan_base, ox, 0.0, ox + r * std::cos(az), r * std::sin(az), 0.15);
    c.view_yaw = wrap((std::isfinite(vy) ? vy : az) + w.ryaw);
    c.src = "sweep";
    c.standoff = p_.sweep_standoff_m;
    return look(c, w, 0, 0, 0, true, f1("훑기 방위 %.0f° (몸 기준 %.0f°) 높이 %.2f", p_.sweep_az_deg[(sweep_i_ - 1) / nh], az / kDeg, h));
  }
  // 몸 멈춤: 남은 돌기 · 미룬 목표는 기록 끝 뒤 → 그때까지 (팔 접고) 기다림
  const std::size_t rot_left = !p_.panel && rot_i_ < p_.rotate_steps_deg.size() ? p_.rotate_steps_deg.size() - rot_i_ : 0;
  if (holding(w) && (rot_left > 0 || !deferred_.empty())) {
    return hold_wait(w, 2.0, f1("신호 기록 중 몸 멈춤 → 기록 끝 기다림 (그 뒤 미룬 목표 %.0f · 돌기 %.0f, 멈춤 %.0f s)",
             static_cast<double>(deferred_.size()), static_cast<double>(rot_left), held_s_));
  }
  // 6) 미룬 목표 (신호 기록 끝난 뒤)
  if (!deferred_.empty() && w.signal_done) {
    Cand c = deferred_.front();
    deferred_.erase(deferred_.begin());
    if (auto m = maybe_approach(c, w)) {return *m;}
    return look(c, w, 0, 0, 0, c.panel_sweep, "신호 기록 끝 → 미룬 목표");
  }
  if (!deferred_.empty()) {
    Action a;
    a.kind = Act::Dwell;
    a.dwell_s = 2.0;
    a.note = "팔이 신호등을 가리는 목표만 남음 → 신호 기록 끝 기다림";
    note_ = a.note;
    return a;
  }
  // 7) 처음부터 다시 훑기 (사람이 바코드를 옮겼을 수 있음 — 시간 안에서)
  if (looks_ > 0 && search_s(w.now) < p_.max_s * 0.6) {
    sweep_i_ = 0;
    rot_i_ = 0;
    tried_.clear();
    Action a;
    a.kind = Act::Dwell;
    a.dwell_s = 1.0;
    a.note = "다 봤지만 못 찾음 → 처음부터 다시";
    note_ = a.note;
    return a;
  }
  Action g;
  g.kind = Act::GiveUp;
  g.note = p_.panel ? "광각 · 벽 판 훑기 모두 못 찾음" : "광각 · 돌기 · 훑기 모두 못 찾음";
  note_ = g.note;
  return g;
}

}  // namespace bc
}  // namespace mission
