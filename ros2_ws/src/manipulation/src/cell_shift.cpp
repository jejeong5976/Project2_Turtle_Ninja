// cell_shift.cpp — 칸 경로의 칸 근처만 옮기기 (설명은 include/manipulation/cell_shift.hpp)

#include "manipulation/cell_shift.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

namespace manipulation
{

namespace
{
constexpr double kNearMm = 35.0, kFarMm = 90.0;
constexpr int kRamp = 12;
// 칸 경로를 옮긴 점은 관절 범위 (tick_lo · hi) 를 이만큼까지 넘어도 됨 [틱, 1.3°]. 실물 경로가 범위 끝에 붙어 있음
//   (store_S10 칸 위 J4 2416 ~ 2431 틱, 범위 2430 → 칸 보정 (-9, -8) mm 로 2443). 실물 J4 멈춤은 약 2485 틱이라 범위 끝 + 15 도 그 안
constexpr double kEdgeSlackTick = 15.0;
// J4 위쪽은 40 틱 (2430 → 2470 = pose_safety 가 실물 재생 구간에 허용한 j4_hi, 기계 멈춤 ~2485 · 2495 에서 과부하)
//   세운 퍽 (+20 mm) 은 J4 가 2445 에 막혀 S4 · S5 · S8 · S10 · S11 칸 근처를 못 올림
constexpr double kJ4HiSlackTick = 40.0;
constexpr double kDeg = 180.0 / M_PI;
constexpr double kT2R = 2.0 * M_PI / 4096.0;

using V5 = Eigen::Matrix<double, 5, 1>;
using F9 = Eigen::Matrix<double, 9, 1>;

double tick_of_cmd(const ArmModel & m, int j, double c) {return m.zero_tick[j] + c / kT2R;}

Grip grip_of_cmd(const ArmModel & m, const V5 & c)
{
  Q5 q;
  for (int j = 0; j < 5; ++j) {
    q[j] = (m.jm[j].q_deg_at_ref + m.jm[j].sign * (tick_of_cmd(m, j, c[j]) - m.jm[j].ref_tick) * 360.0 / 4096.0) / kDeg;
  }
  return arm_grip(m, q);
}

// 위치 [mm] × 10 · 방향 × 100 (1° ≈ 1.7) — 관절 5 개로 6 개를 다 못 맞춤 → 위치 우선
F9 feat(const ArmModel & m, const V5 & c)
{
  const Grip g = grip_of_cmd(m, c);
  F9 f;
  f << g.tip * 1000.0 * 10.0, g.app * 100.0, g.close * 100.0;
  return f;
}

V5 solve(const ArmModel & m, const V5 & c0, const Eigen::Vector3d & dp_mm)
{
  F9 goal = feat(m, c0);
  goal.head<3>() += dp_mm * 10.0;
  const double lam = 0.1;   // 관절 변화 억제 (수직 손에서 J1 · J5 가 서로 대신 도는 것 막음)
  V5 c = c0;
  for (int it = 0; it < 20; ++it) {
    const F9 f = feat(m, c);
    Eigen::Matrix<double, 9, 5> J;
    for (int j = 0; j < 5; ++j) {
      V5 e = c;
      e[j] += 1e-6;
      J.col(j) = (feat(m, e) - f) / 1e-6;
    }
    const Eigen::Matrix<double, 5, 5> A = J.transpose() * J + lam * Eigen::Matrix<double, 5, 5>::Identity();
    const V5 dc = A.ldlt().solve(J.transpose() * (goal - f) - lam * (c - c0));
    c += dc;
    if (dc.cwiseAbs().maxCoeff() < 1e-10) {break;}
  }
  return c;
}

int find_node(const ArmPath & p, const char * name)
{
  for (size_t i = 0; i < p.nodes.size(); ++i) {
    if (p.nodes[i].name == name) {return static_cast<int>(i);}
  }
  return -1;
}
}  // namespace

bool shift_cell_path(
  const ArmModel & m, ArmPath & path, bool store, const Eigen::Vector2d & cell_xy, const Eigen::Vector2d & d,
  CellShiftReport & rep, std::string & why, double dz)
{
  rep = CellShiftReport{};
  if (!d.allFinite() || !cell_xy.allFinite() || !std::isfinite(dz)) {why = "옮길 양 · 칸 위치가 숫자가 아님"; return false;}
  const int n = static_cast<int>(path.nodes.size());
  const int k = find_node(path, store ? "P0_복귀" : "P0");
  if (k < 0) {why = path.name + ": 경유점 " + (store ? "P0_복귀" : "P0") + " 없음"; return false;}
  const int lo = store ? 1 : k, hi = store ? k : n - 1;
  // 놓는 점 (STORE release) · 쥐는 점 (RETRIEVE close) 은 다 옮겨야 함 → 그 점이 끝에서 12 점보다 가까우면 그 사이에서 0 → 1
  //   (store_look_S1 은 놓는 점이 둘째 점에서 9 점째라 75 % 만 옮기고 로그엔 100 % 로 나옴)
  int key = -1;
  for (int i = lo; i <= hi && key < 0; ++i) {
    if (path.nodes[i].grip == (store ? "release" : "close")) {key = i;}
  }
  const double r_in = key > lo ? std::min(kRamp, key - lo) : kRamp, r_out = key >= 0 && key < hi ? std::min(kRamp, hi - key) : kRamp;
  std::vector<PathNode> out = path.nodes;
  for (int i = 0; i < n; ++i) {
    if (path.nodes[i].q.size() != 5) {why = path.name + ": 관절 수가 5 가 아닌 점"; return false;}
    const double ramp = std::clamp(std::min((i - lo) / r_in, (hi - i) / r_out), 0.0, 1.0);
    if (ramp <= 0.0) {continue;}
    const V5 c0 = Eigen::Map<const V5>(path.nodes[i].q.data());
    const Grip g0 = grip_of_cmd(m, c0);
    const double dist_mm = (g0.tip.head<2>() - cell_xy).norm() * 1000.0;
    const double w = std::clamp((kFarMm - dist_mm) / (kFarMm - kNearMm), 0.0, 1.0) * ramp;
    if (w <= 0.0) {continue;}
    const Eigen::Vector3d dp(d.x() * 1000.0 * w, d.y() * 1000.0 * w, dz * 1000.0 * w);
    const V5 c = solve(m, c0, dp);
    const Grip g = grip_of_cmd(m, c);
    rep.worst_pos_mm = std::max(rep.worst_pos_mm, ((g.tip - g0.tip) * 1000.0 - dp).norm());
    rep.worst_app_deg = std::max(rep.worst_app_deg, std::acos(std::clamp(g.app.dot(g0.app), -1.0, 1.0)) * kDeg);
    rep.max_joint_deg = std::max(rep.max_joint_deg, (c - c0).cwiseAbs().maxCoeff() * kDeg);
    for (int j = 0; j < 5; ++j) {
      const double t = tick_of_cmd(m, j, c[j]);
      const double hi_slack = j == 3 ? kJ4HiSlackTick : kEdgeSlackTick;
      if (!std::isfinite(t) || t < m.tick_range[j][0] - kEdgeSlackTick || t > m.tick_range[j][1] + hi_slack) {
        why = path.name + ": " + path.nodes[i].name + " 를 옮기면 관절 " + std::to_string(j + 1) + " 범위 밖";
        return false;
      }
      out[i].q[j] = c[j];
    }
    ++rep.moved;
  }
  // 접근축: 관절 5 개라 손끝을 옮기면 손이 조금 기움. 실물: 오른쪽 칸 2 cm 보정에 1.0–1.7° → 1° 한도로 50–75 % 만 옮겨
  //   칸마다 들어가는 자리가 매번 달라짐. 5° 기울어도 퍽 바닥 (손끝 아래 60 mm) 은 5 mm 안
  if (rep.worst_pos_mm > 0.2 || rep.worst_app_deg > 5.0) {
    char buf[200];
    std::snprintf(buf, sizeof(buf), "%s: 손끝을 원하는 만큼 못 옮김 (어긋남 %.2f mm · 접근축 %.2f°)", path.name.c_str(),
      rep.worst_pos_mm, rep.worst_app_deg);
    why = buf;
    return false;
  }
  path.nodes = std::move(out);
  return true;
}

}  // namespace manipulation
