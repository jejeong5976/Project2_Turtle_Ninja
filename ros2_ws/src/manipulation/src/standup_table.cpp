// standup_table.cpp — 누운 네모 퍽 세우기 경로 표 읽기 · 고르기 (standup_table.hpp)

#include "manipulation/standup_table.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>

namespace manipulation
{

bool load_standup_table(const std::string & file, std::vector<StandupPath> & out, std::string & why)
{
  out.clear();
  std::ifstream f(file);
  if (!f) {why = "세우기 경로 표를 못 엶: " + file; return false;}
  std::string line;
  int no = 0;
  while (std::getline(f, line)) {
    ++no;
    if (line.empty() || line[0] == '#') {continue;}
    std::istringstream in(line);
    StandupPath p;
    int n = 0;
    if (!(in >> p.side >> p.x >> p.y >> p.axis_deg >> p.rate >> n >> p.release >> p.end_x >> p.end_y >> p.end_face_deg)) {
      why = file + " " + std::to_string(no) + " 줄: 앞 10 칸을 못 읽음";
      return false;
    }
    for (auto & v : p.above) {in >> v;}
    for (auto & v : p.grasp) {in >> v;}
    if (!in || n < 2 || p.release < 0 || p.release >= n) {
      why = file + " " + std::to_string(no) + " 줄: 점 수 · 여는 점 · 위 · 쥐기 자세가 이상함";
      return false;
    }
    p.q.resize(static_cast<size_t>(n));
    for (auto & t : p.q) {for (auto & v : t) {in >> v;}}
    std::string extra;
    if (!in || (in >> extra)) {
      why = file + " " + std::to_string(no) + " 줄: 경로 점 수 (" + std::to_string(n) + " × 5) 와 칸 수가 다름";
      return false;
    }
    out.push_back(std::move(p));
  }
  if (out.empty()) {why = "세우기 경로 표가 비어 있음: " + file; return false;}
  return true;
}

int nearest_farpick(const std::vector<StandupPath> & table, double x, double y, double axis_deg, double side,
  double & dx, double & dy, double & dyaw_deg, double period_deg, double step_x, double step_y, double step_deg)
{
  double side_best = std::numeric_limits<double>::infinity();
  for (const auto & p : table) {side_best = std::min(side_best, std::abs(p.side - side));}
  int best = -1;
  double best_d = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < table.size(); ++i) {
    const auto & p = table[i];
    if (std::abs(std::abs(p.side - side) - side_best) > 1e-6) {continue;}
    const double da = period_deg > 0.0 ? std::remainder(axis_deg - p.axis_deg, period_deg) : 0.0;   // 주기 0 = 축 무시 (원기둥)
    const double d = std::pow((x - p.x) / step_x, 2) + std::pow((y - p.y) / step_y, 2) + std::pow(da / step_deg, 2);
    if (d < best_d) {
      best_d = d;
      best = static_cast<int>(i);
      dx = x - p.x;
      dy = y - p.y;
      dyaw_deg = da;
    }
  }
  return best;
}

int nearest_standup(const std::vector<StandupPath> & table, double x, double y, double axis_deg, double side,
  double & dx, double & dy, double & dyaw_deg, double step_x, double step_y, double step_deg)
{
  return nearest_farpick(table, x, y, axis_deg, side, dx, dy, dyaw_deg, 180.0, step_x, step_y, step_deg);   // 긴축은 180° 마다 같음
}

std::string farpick_table_name(bool has_nbr, double nbr_dir_deg)
{
  if (!has_nbr || !std::isfinite(nbr_dir_deg)) {return "farpick_paths_ok75_nbr_none.txt";}
  const int k = ((static_cast<int>(std::lround(nbr_dir_deg / 45.0)) % 8) + 8) % 8;   // 가장 가까운 45°, 0 … 315
  char buf[48];
  std::snprintf(buf, sizeof(buf), "farpick_paths_ok75_nbr_%03d.txt", k * 45);
  return buf;
}

bool standup_retarget(const ArmModel & m, const Q5 & q_table, const StandupRetarget & r, bool scale, RetargetResult & out)
{
  out = RetargetResult{};
  out.q = q_table;
  const Grip g = arm_grip(m, q_table);
  Eigen::Vector2d xy = r.to + Eigen::Rotation2Dd(r.da) * (g.tip.head<2>() - r.from);
  if (scale) {xy = r.anchor + r.reach_scale * (xy - r.anchor);}
  Eigen::Vector3d p(xy.x(), xy.y(), g.tip.z() + r.dz);
  if (p.z() < r.tip_min_z) {
    out.lifted_mm = (r.tip_min_z - p.z()) * 1000.0;
    p.z() = r.tip_min_z;
  }
  if ((p - g.tip).norm() < 1e-6 && std::abs(r.da) < 1e-6) {return true;}
  const Eigen::Matrix3d R = Eigen::AngleAxisd(r.da, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const auto sols = arm_ik(m, p, R * g.app, R * g.close, {q_table}, 180.0);
  double best = 1e9;
  const IkSolution * c = nullptr;   // 가장 가까운 해 (실패 설명)
  for (const auto & sl : sols) {
    if (!c || sl.pos_mm + sl.app_deg < c->pos_mm + c->app_deg) {c = &sl;}
    if (sl.pos_mm < 1.0 && sl.app_deg < 10.0 && sl.roll_deg < 10.0 && ticks_in_range(m, q_to_ticks(m, sl.q))) {
      double d = 0.0;
      for (int j = 0; j < 5; ++j) {d += std::abs(sl.q[j] - q_table[j]);}
      const double score = sl.app_deg + sl.roll_deg + d * 180.0 / M_PI * 0.2;   // 방향 먼저, 같으면 원래 자세에 가까운 해
      if (score < best) {
        best = score;
        out.q = sl.q;
        out.app_err_deg = std::max(sl.app_deg, sl.roll_deg);
      }
    }
  }
  if (best < 1e8) {return true;}
  char buf[160];
  if (c) {
    std::snprintf(buf, sizeof(buf), "가장 가까운 해: 손끝 %.1f mm · 접근축 %.1f° · 닫힘축 %.1f° · 관절 범위 %s", c->pos_mm, c->app_deg, c->roll_deg,
      ticks_in_range(m, q_to_ticks(m, c->q)) ? "안" : "밖");
  } else {
    std::snprintf(buf, sizeof(buf), "IK 해 없음");
  }
  out.fail = buf;
  return false;
}

std::string lyingpick_table_path(const std::string & param, const std::string & standup_file)
{
  if (!param.empty()) {return param;}
  if (standup_file.empty()) {return "";}
  return (std::filesystem::path(standup_file).parent_path() / "lyingpick" / "lyingpick_paths_ok75.txt").string();
}

LyingPickChoice choose_lyingpick(const std::vector<StandupPath> & table, double x, double y, double axis_deg, double max_dist_mm, double max_deg)
{
  LyingPickChoice c;
  double best_any = std::numeric_limits<double>::infinity(), best_ok = best_any;
  LyingPickChoice any;   // 가장 가까운 줄의 차 (한도 안 줄이 없을 때 로그)
  for (size_t i = 0; i < table.size(); ++i) {
    const auto & p = table[i];
    if (std::abs(p.side - kLyingPickSide) > 1e-4) {continue;}
    const double dx = x - p.x, dy = y - p.y;
    const double da = std::remainder(axis_deg - p.axis_deg, 180.0);   // 긴축은 180° 마다 같음
    const double d = std::pow(dx / 0.0075, 2) + std::pow(dy / 0.01, 2) + std::pow(da / 7.5, 2);
    const double dist = std::hypot(dx, dy) * 1000.0;
    if (d < best_any) {
      best_any = d;
      any.nearest = static_cast<int>(i);
      any.dx = dx;
      any.dy = dy;
      any.dyaw_deg = da;
      any.dist_mm = dist;
    }
    if (dist <= max_dist_mm && std::abs(da) <= max_deg && d < best_ok) {
      best_ok = d;
      c.row = static_cast<int>(i);
      c.dx = dx;
      c.dy = dy;
      c.dyaw_deg = da;
      c.dist_mm = dist;
    }
  }
  c.nearest = any.nearest;
  if (c.row < 0) {
    c.dx = any.dx;
    c.dy = any.dy;
    c.dyaw_deg = any.dyaw_deg;
    c.dist_mm = any.dist_mm;
  }
  return c;
}

bool lyingpick_path(const ArmModel & m, const StandupPath & e, const StandupRetarget & rt, double held_kg, double dt_s, bool early_open,
  LyingPickPath & out, std::string & why, const Eigen::Vector3d & shift, const LyingHover & hover)
{
  out = LyingPickPath{};
  const int n = static_cast<int>(e.q.size());
  if (e.release < 1 || e.release + 1 >= n) {
    why = "닫는 점 번호 " + std::to_string(e.release) + " 가 이상함 (점 " + std::to_string(n) + " 개, 앞 · 뒤 점이 있어야 함)";
    return false;
  }
  auto table_q = [&](int i) {
      Ticks tt;
      for (int j = 0; j < 5; ++j) {tt[j] = e.q[static_cast<size_t>(i)][static_cast<size_t>(j)];}
      return ticks_to_q(m, tt);
    };
  const bool shifted = shift.squaredNorm() > 0.0;
  const Eigen::Vector3d tip_close = shifted ? arm_grip(m, table_q(e.release)).tip : Eigen::Vector3d::Zero();
  int gap = 1;   // 건너뛴 점만큼 다음 점까지 시간을 늘림 (farpick_plan · standup_plan 과 같음)
  bool closed = false;
  for (int i = 1; i < n; ++i) {
    const Q5 qt = table_q(i);
    RetargetResult rr;
    const bool held = i >= e.release;
    StandupRetarget r = rt;
    if (shifted) {   // 퍽 근처 구간만 더 옮김 (닫기 전은 닫는 점 손끝에서 거리로 들어감, 닫은 뒤는 다)
      double w = 1.0;
      if (!held) {
        const double d = (arm_grip(m, qt).tip - tip_close).norm();
        w = std::clamp((kLyingShiftOut - d) / (kLyingShiftOut - kLyingShiftIn), 0.0, 1.0);
      }
      r.to += w * shift.head<2>();
      r.dz += w * shift.z();
    }
    if (!standup_retarget(m, qt, r, false, rr)) {
      if (i == e.release) {why = "닫는 점을 실제 퍽으로 옮긴 자세 (IK) 가 없음 — " + rr.fail; return false;}
      ++out.skipped;
      ++gap;
      continue;
    }
    PathNode nd;
    nd.q = ticks_to_command(m, q_to_ticks(m, encoder_goal_q(m, rr.q, held ? held_kg : 0.0)));
    if (out.nodes.empty()) {   // 첫 점: 위로 가며 엶 (pick_path 의 above 와 같은 grip_early), 속도 시간표
      if (i == e.release) {why = "닫기 전 점이 전부 IK 안 됨"; return false;}
      nd.grip = "open";
      nd.grip_early = early_open;
    } else {
      nd.dt_s = dt_s * gap;   // 표 점 간격 그대로
    }
    if (i == e.release && hover.hover_m > 0.0) {   // 닫는 점 위 멈춤 → 곧게 내려옴
      StandupRetarget rh = r;
      rh.dz += hover.hover_m;
      RetargetResult hr;
      if (standup_retarget(m, qt, rh, false, hr)) {
        const Eigen::Vector3d tip_c = arm_grip(m, rr.q).tip;
        double cut_dt = 0.0;   // 뺀 표 점 시간은 멈춤 점까지 가는 시간에 더함
        while (!out.nodes.empty() && out.nodes.size() > 1 && (arm_grip(m, out.q.back()).tip - tip_c).norm() < hover.hover_m + 0.003) {
          cut_dt += out.nodes.back().dt_s;
          out.nodes.pop_back();
          out.q.pop_back();
        }
        PathNode hn;
        hn.name = "lp_in_hover";
        hn.q = ticks_to_command(m, q_to_ticks(m, encoder_goal_q(m, hr.q, 0.0)));
        hn.dt_s = std::max(nd.dt_s + cut_dt, dt_s);
        hn.stop = true;
        hn.hold_s = std::max(0.0, hover.hover_s);
        out.nodes.push_back(hn);
        out.q.push_back(hr.q);
        nd.dt_s = std::max(hover.descend_s, dt_s);
      }
    }
    if (i == e.release) {   // 닫는 점: 일반 닫기 (팔 멈춤), 빈손 확인의 위 점 = 다음 점
      nd.name = "reloc_grasp";
      nd.grip = "close";
      out.q_close = rr.q;
      closed = true;
    } else {
      nd.name = (closed ? "lp" : "lp_in") + std::to_string(i);
    }
    gap = 1;
    out.nodes.push_back(nd);
    out.q.push_back(rr.q);
  }
  if (!closed || out.nodes.back().grip == "close") {why = "닫는 점 뒤 점이 전부 IK 안 됨"; return false;}
  out.nodes.back().name = "reloc_lift";
  out.nodes.back().stop = true;   // 시간 맞춘 구간은 들어 올린 자세에서 끝 (farpick far_lift 와 같음)
  return true;
}

Eigen::Vector2d lying_recenter_shift(const Grip & g, const Eigen::Vector2d & tip_off)
{
  Eigen::Vector2d c = g.close.head<2>();
  if (c.norm() < 0.3) {return Eigen::Vector2d::Zero();}   // 닫힘축이 거의 수직 = 수평 어긋남을 닫힘으로 못 나눔
  c.normalize();
  return -tip_off.dot(c) * c;
}

bool lyingpick_free_node(const std::string & name)
{
  return name.rfind("lp_in", 0) == 0;
}

namespace
{
// 손 좌표 (접근축 a, b = a × 닫힘축 — 닫힘축은 a 에 수직으로 맞춤)
void hand_axes(const Grip & g, Eigen::Vector3d & a, Eigen::Vector3d & b)
{
  a = g.app.normalized();
  const Eigen::Vector3d c = (g.close - g.close.dot(a) * a).normalized();
  b = a.cross(c);
}
}  // namespace

Eigen::Vector2d held_center_in_hand(const Grip & g, const Eigen::Vector3d & center)
{
  Eigen::Vector3d a, b;
  hand_axes(g, a, b);
  const Eigen::Vector3d o = center - g.tip;
  return {o.dot(a), o.dot(b)};
}

Eigen::Vector3d held_center_at(const Grip & g, const Eigen::Vector2d & in_hand)
{
  Eigen::Vector3d a, b;
  hand_axes(g, a, b);
  return g.tip + in_hand.x() * a + in_hand.y() * b;
}

}  // namespace manipulation
