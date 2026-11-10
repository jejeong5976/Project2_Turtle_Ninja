// standup_table.cpp — 누운 네모 퍽 세우기 경로 표 읽기 · 고르기 (standup_table.hpp)

#include "manipulation/standup_table.hpp"

#include <cmath>
#include <cstdio>
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

}  // namespace manipulation
