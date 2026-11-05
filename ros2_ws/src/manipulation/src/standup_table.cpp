// standup_table.cpp — 누운 네모 퍽 세우기 경로 표 읽기 · 고르기 (standup_table.hpp)

#include "manipulation/standup_table.hpp"

#include <cmath>
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

int nearest_standup(const std::vector<StandupPath> & table, double x, double y, double axis_deg, double side,
  double & dx, double & dy, double & dyaw_deg, double step_x, double step_y, double step_deg)
{
  double side_best = std::numeric_limits<double>::infinity();
  for (const auto & p : table) {side_best = std::min(side_best, std::abs(p.side - side));}
  int best = -1;
  double best_d = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < table.size(); ++i) {
    const auto & p = table[i];
    if (std::abs(std::abs(p.side - side) - side_best) > 1e-6) {continue;}
    const double da = std::remainder(axis_deg - p.axis_deg, 180.0);
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

}  // namespace manipulation
