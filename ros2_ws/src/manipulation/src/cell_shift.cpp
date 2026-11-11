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
constexpr double kLiftNearM = 0.050;   // retrieve_lift_clear · store_lift_clear: 손끝이 칸 중심에서 수평 이만큼 안이면 칸 위로 올림
constexpr int kRamp = 12;
// 칸 경로를 옮긴 점은 관절 범위 (tick_lo · hi) 를 이만큼까지 넘어도 됨 [틱, 1.3°]. 실물 경로가 범위 끝에 붙어 있음
//   (store_S10 칸 위 J4 2416 ~ 2431 틱, 범위 2430 → 칸 보정 (-9, -8) mm 로 2443). 실물 J4 멈춤은 약 2485 틱이라 범위 끝 + 15 도 그 안
constexpr double kEdgeSlackTick = 15.0;
// J4 위쪽은 45 틱 (2430 → 2475 = 기계 멈춤 2485 − 10, 11-10 새 칸 경로가 2455–2475 사용 · 2495 에서 과부하)
//   세운 퍽 (+20 mm) 은 J4 가 2445 에 막혀 S4 · S5 · S8 · S10 · S11 칸 근처를 못 올림
constexpr double kJ4HiSlackTick = 50.0;   // 11-10: 세운 퍽 store_S7_d30 기록 경로가 이미 2475 (= 2430 + 45) → 앞 5 mm 에 2477 필요. 실물 J4 멈춤 약 2485 (과부하 2495) 라 2480 까지
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
  // RETRIEVE 높이 (dz): 쥐기 전 그리퍼를 여는 점 (key 앞 마지막 release · open) 까지는 안 내리고, 거기서 쥐는 점까지 0 → 1
  //   (11-10 14:3x 실물: 빼기를 17 mm 깊게 하자 여는 점도 내려가 퍽 윗면에 닿은 채 그리퍼를 열고 더 내려감)
  int open_i = -1;
  if (!store && key > lo) {
    for (int i = key - 1; i >= lo && open_i < 0; --i) {
      if (path.nodes[i].grip == "release" || path.nodes[i].grip == "open") {open_i = i;}
    }
  }
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
    double wz = w;
    if (open_i >= 0 && i < key) {wz = i <= open_i ? 0.0 : w * static_cast<double>(i - open_i) / static_cast<double>(key - open_i);}
    const Eigen::Vector3d dp(d.x() * 1000.0 * w, d.y() * 1000.0 * w, dz * 1000.0 * wz);
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

namespace
{
// key_grip 첫 점 (RETRIEVE close · STORE release) 바로 뒤에 손끝을 곧게 clear_m 올린 점 (up_name) 을 넣고 그 뒤 칸 근처 낮은 점을 올림.
//   key_label = 실패 문구의 그 점 이름 (쥐는 점 · 놓는 점). 새 점은 grip 없음 → 그리퍼 동작은 원래 점에 그대로
bool lift_clear_after(const ArmModel & m, ArmPath & path, const Eigen::Vector2d & cell_xy, double clear_m, const std::string & key_grip,
  const std::string & up_name, const std::string & key_label, int & raised, std::string & why)
{
  raised = 0;
  if (!(clear_m > 0.0)) {return true;}
  const int n = static_cast<int>(path.nodes.size());
  int key = -1;
  for (int i = 0; i < n && key < 0; ++i) {
    if (path.nodes[i].grip == key_grip) {key = i;}
  }
  if (key < 0 || key + 1 >= n || path.nodes[key].q.size() != 5) {why = path.name + ": " + key_label + " (" + key_grip + ") 없음"; return false;}
  auto in_range = [&](const V5 & c) {
      for (int j = 0; j < 5; ++j) {
        const double t = tick_of_cmd(m, j, c[j]);
        const double hi_slack = j == 3 ? kJ4HiSlackTick : kEdgeSlackTick;
        if (!std::isfinite(t) || t < m.tick_range[j][0] - kEdgeSlackTick || t > m.tick_range[j][1] + hi_slack) {return false;}
      }
      return true;
    };
  // 높이만 dz 올린 관절값 (손끝 · 접근축 · 닫힘축 유지). 손끝이 0.5 mm 넘게 어긋나거나 관절 범위 밖이면 실패
  auto raise = [&](const V5 & c0, double dz, V5 & c) {
      c = solve(m, c0, Eigen::Vector3d(0.0, 0.0, dz * 1000.0));
      const Grip g1 = grip_of_cmd(m, c), g0 = grip_of_cmd(m, c0);
      const Eigen::Vector3d err = (g1.tip - g0.tip) * 1000.0 - Eigen::Vector3d(0.0, 0.0, dz * 1000.0);
      const double app_deg = std::acos(std::clamp(g1.app.dot(g0.app), -1.0, 1.0)) * kDeg;   // 접근축이 5° 넘게 돌면 안 씀 (shift_cell_path 와 같은 한도, 11-10 검수 B4)
      return err.norm() < 0.5 && app_deg <= 5.0 && in_range(c);
    };
  const V5 ck = Eigen::Map<const V5>(path.nodes[key].q.data());
  const double z_need = grip_of_cmd(m, ck).tip.z() + clear_m;
  std::vector<PathNode> out(path.nodes.begin(), path.nodes.begin() + key + 1);
  PathNode up = path.nodes[key];
  V5 cu;
  if (!raise(ck, clear_m, cu)) {why = path.name + ": " + key_label + "에서 곧게 " + std::to_string(static_cast<int>(clear_m * 1000.0)) + " mm 못 올림"; return false;}
  up.name = up_name;
  up.grip.clear();
  up.grip_on_way = false;
  up.stop = false;
  up.hold_s = 0.0;
  up.dt_s = 0.0;
  up.speed_dps = path.nodes[key + 1].speed_dps;
  up.q.assign(cu.data(), cu.data() + 5);
  out.push_back(up);
  bool near = true;
  for (int i = key + 1; i < n; ++i) {
    PathNode nd = path.nodes[i];
    if (near && nd.q.size() == 5) {
      const V5 c0 = Eigen::Map<const V5>(nd.q.data());
      const Grip g = grip_of_cmd(m, c0);
      if ((g.tip.head<2>() - cell_xy).norm() > kLiftNearM) {
        near = false;
      } else if (g.tip.z() < z_need) {
        V5 c;
        if (!raise(c0, z_need - g.tip.z(), c)) {why = path.name + ": " + nd.name + " 를 칸 위로 못 올림"; return false;}
        nd.q.assign(c.data(), c.data() + 5);
        ++raised;
      }
    }
    out.push_back(nd);
  }
  path.nodes = std::move(out);
  return true;
}
}  // namespace

bool retrieve_lift_clear(const ArmModel & m, ArmPath & path, const Eigen::Vector2d & cell_xy, double clear_m, int & raised, std::string & why)
{
  return lift_clear_after(m, path, cell_xy, clear_m, "close", "lift_clear", "쥐는 점", raised, why);
}

bool store_lift_clear(const ArmModel & m, ArmPath & path, const Eigen::Vector2d & cell_xy, double clear_m, int & raised, std::string & why)
{
  return lift_clear_after(m, path, cell_xy, clear_m, "release", "store_lift_clear", "놓는 점", raised, why);
}

int store_settle_index(const ArmModel & m, const ArmPath & path, double up_m)
{
  if (!std::isfinite(up_m)) {return -1;}
  const int n = static_cast<int>(path.nodes.size());
  int key = -1;   // 첫 놓는 점
  for (int i = 0; i < n && key < 0; ++i) {
    if (path.nodes[i].grip == "release") {key = i;}
  }
  if (key < 0 || path.nodes[key].q.size() != 5) {return -1;}
  const double z_rel = grip_of_cmd(m, Eigen::Map<const V5>(path.nodes[key].q.data())).tip.z();
  for (int i = key - 1; i >= 1; --i) {   // 뒤에서부터 = 놓는 점보다 up_m 이상 높은 마지막 점. 첫 점 (출발 자세) 은 뺌
    if (path.nodes[i].q.size() != 5) {continue;}
    if (grip_of_cmd(m, Eigen::Map<const V5>(path.nodes[i].q.data())).tip.z() - z_rel >= up_m) {return i;}
  }
  return -1;
}

bool store_delay_close(const ArmModel & m, ArmPath & path, double rise_m, double side_m, StoreCloseMove & mv)
{
  mv = StoreCloseMove{};
  const bool use_rise = rise_m > 0.0, use_side = side_m > 0.0;   // NaN · 0 이하 = 그 조건 안 봄
  const int n = static_cast<int>(path.nodes.size());
  for (int i = 0; i < n && mv.release < 0; ++i) {
    if (path.nodes[i].grip == "release") {mv.release = i;}
  }
  if (mv.release < 0 || path.nodes[mv.release].q.size() != 5) {return false;}
  for (int i = mv.release + 1; i < n && mv.from < 0; ++i) {
    if (path.nodes[i].grip == "close") {mv.from = i;}
  }
  if (mv.from < 0) {return false;}
  mv.to = mv.from;
  const Eigen::Vector3d rel = grip_of_cmd(m, Eigen::Map<const V5>(path.nodes[mv.release].q.data())).tip;
  auto off = [&](int i, double & dz_mm, double & dxy_mm) {
      if (path.nodes[i].q.size() != 5) {return false;}
      const Eigen::Vector3d d = grip_of_cmd(m, Eigen::Map<const V5>(path.nodes[i].q.data())).tip - rel;
      dz_mm = d.z() * 1000.0;
      dxy_mm = d.head<2>().norm() * 1000.0;
      return true;
    };
  if (!off(mv.from, mv.from_dz_mm, mv.from_dxy_mm)) {return false;}
  mv.to_dz_mm = mv.from_dz_mm;
  mv.to_dxy_mm = mv.from_dxy_mm;
  if (!use_rise && !use_side) {return false;}
  int last = n - 2;   // 끝 점은 안 씀 (path_actions 가 끝 점 그리퍼를 멈춤 + 기다림으로)
  const int fold = find_node(path, "P0_복귀");
  if (fold > mv.from) {last = std::min(last, fold - 1);}
  for (int i = mv.from + 1; i <= last; ++i) {
    if (!path.nodes[i].grip.empty()) {last = i - 1; break;}   // 다음 그리퍼 점 (닫기 · 열기) 순서는 그대로
  }
  if (last <= mv.from) {return false;}   // 옮길 자리 없음 → 그대로
  int to = -1;
  double dz = 0.0, dxy = 0.0;
  for (int i = mv.from; i <= last && to < 0; ++i) {
    if (off(i, dz, dxy) && ((use_rise && dz >= rise_m * 1000.0) || (use_side && dxy >= side_m * 1000.0))) {to = i;}
  }
  if (to < 0) {
    to = last;
    mv.capped = true;
    if (!off(to, dz, dxy)) {dz = dxy = std::nan("");}
  }
  mv.to = to;
  mv.to_dz_mm = dz;
  mv.to_dxy_mm = dxy;
  if (to == mv.from) {return false;}
  PathNode & a = path.nodes[mv.from];
  PathNode & b = path.nodes[to];
  b.grip = a.grip;
  b.grip_on_way = a.grip_on_way;
  b.grip_early = a.grip_early;
  a.grip.clear();
  a.grip_on_way = false;
  a.grip_early = false;
  return true;
}

std::string path_cell(const std::string & name)
{
  std::string rest;
  for (const char * pre : {"store_look_", "look_rack_", "store_", "retrieve_"}) {   // store_look_ 이 store_ 보다 먼저
    const std::string ps(pre);
    if (name.rfind(ps, 0) == 0) {rest = name.substr(ps.size()); break;}
  }
  return rest.substr(0, rest.find('_'));   // 다음 '_' 앞까지 = 칸 (S5_d30 → S5)
}

bool storeheld_path(const std::string & name)
{
  if (name.rfind("store_", 0) != 0 && name.rfind("retrieve_", 0) != 0) {return false;}
  for (const char * tag : {"_d30", "_d50", "_d40_tilt", "_full"}) {
    if (name.find(tag) != std::string::npos) {return true;}
  }
  return false;
}

std::string storeheld_store_name(const std::string & cell, double held_depth_mm, double regrasp_depth_mm, double tilt_deg, bool low,
  const std::function<bool(const std::string &)> & has)
{
  const std::string base = "store_" + cell;
  if (tilt_deg > 0.5) {   // 기울여 집은 퍽: 가까운 기울임 경로, 없으면 "" (지금 흐름 — 거절 또는 다시 쥐기)
    int best = 10;
    for (int t : {20, 30}) {if (std::fabs(tilt_deg - t) < std::fabs(tilt_deg - best)) {best = t;}}
    const std::string n = base + "_d40_tilt" + std::to_string(best);
    return has(n) ? n : "";
  }
  if (std::fabs(held_depth_mm - regrasp_depth_mm) <= 5.0) {   // 받침에서 다시 쥔 세운 퍽 (30 mm)
    if (low && has(base + "_d30_low")) {return base + "_d30_low";}
    return has(base + "_d30") ? base + "_d30" : "";
  }
  return has(base + "_d50") ? base + "_d50" : "";   // 박스 퍽 50 mm
}

std::string storeheld_retrieve_name(const std::string & cell, const std::function<bool(const std::string &)> & has)
{
  const std::string n = "retrieve_" + cell + "_full";
  return has(n) ? n : "";
}

}  // namespace manipulation
