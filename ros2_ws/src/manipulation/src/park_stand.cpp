#include "manipulation/park_stand.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace manipulation
{
namespace
{
constexpr double kDeg = 180.0 / M_PI;

bool ik_ok(const ArmModel & m, const IkSolution & s, bool check_enc)
{
  return s.pos_mm < 1.0 && s.app_deg < 1.0 && s.roll_deg < 2.0 && ticks_in_range(m, q_to_ticks(m, s.q)) &&
         (!check_enc || ticks_in_range(m, q_to_ticks(m, encoder_goal_q(m, s.q, 0.0))));
}

// 손 좌표 (열 = 닫힘 · b = 접근 × 닫힘 · 접근)
Eigen::Matrix3d hand_frame(const Eigen::Vector3d & app, const Eigen::Vector3d & close)
{
  Eigen::Matrix3d h;
  h.col(0) = close;
  h.col(1) = app.cross(close);
  h.col(2) = app;
  return h;
}
// 손 기울임 t (바깥) · 쥐는 끝 e1 일 때 손 · 굴린 퍽 (닫힘축 = 손 축 × 긴축)
struct RollGeo
{
  Eigen::Vector3d tip, app, close, c, n;
  double rho{0.0}, margin{0.0}, d_up{0.0};
};

RollGeo roll_geometry(
  const ArmModel & m, const Eigen::Vector3d & c0, const Eigen::Vector3d & a, const Eigen::Vector3d & e1, double t, double tip_floor,
  const LyingGraspCfg & cfg)
{
  const Eigen::Vector3d Z = Eigen::Vector3d::UnitZ();
  const double half = cfg.side / 2.0;
  RollGeo r;
  r.tip = c0 + cfg.grip_offset * e1;
  r.tip.z() = tip_floor;
  for (int it = 0; it < 4; ++it) {   // 팔 평면 (J1 → 손끝) 이 손끝 자리에 따라 바뀌므로 되풀이해 맞춤
    const Eigen::Vector2d r2 = (r.tip.head<2>() - m.j1_xy).normalized();
    const Eigen::Vector3d R(r2.x(), r2.y(), 0.0);
    r.app = std::sin(t) * R - std::cos(t) * Z;
    r.close = r.app.cross(a).normalized();
    if (r.close.z() < 0.0) {r.close = -r.close;}   // 위쪽 손가락 쪽
    r.rho = std::asin(std::clamp(r.close.z(), -1.0, 1.0));
    Eigen::Vector3d h = r.close - r.close.z() * Z;
    h = h.norm() > 1e-9 ? h.normalized() : r.close;
    r.n = -std::sin(r.rho) * h + std::cos(r.rho) * Z;
    // 굴린 뒤 중심: 아래쪽 바닥 모서리로 서고 (높이), 손가락 가운데 (닫힘 방향 = 굴리기 전 중심, 양쪽 여유 같음)
    const double cz = half * (std::sin(r.rho) + std::cos(r.rho) - 1.0);
    r.c = c0 - std::tan(r.rho) * cz * h + cz * Z;
    r.margin = cfg.open_inner - half * (std::cos(r.rho) + std::sin(r.rho));
    // 손끝 높이: 열린 아래 손가락 가장 낮은 점 = 수직 손 손끝 높이
    const Eigen::Vector3d b = r.app.cross(r.close).normalized();
    const double tip_z = tip_floor + cfg.open_outer * std::fabs(r.close.z()) + cfg.finger_half_w * std::fabs(b.z());
    const double xn = (tip_z - r.c.z()) / r.n.z();
    r.d_up = half - xn;
    // 쥐는 점: 손 축이 퍽 중심 높이 면을 지나는 점이 중심에서 쥐는 끝 쪽으로 grip_offset
    const double s_mid = xn / r.app.dot(r.n);
    const double xa = cfg.grip_offset * e1.dot(a) + s_mid * r.app.dot(a);
    r.tip = r.c + xa * a + xn * r.n;
  }
  return r;
}

// 손끝 p · 손 (app, close) 해 중 q 에 가장 가까운 것 → q
bool nearest_ik(const ArmModel & m, const Eigen::Vector3d & p, const Eigen::Vector3d & app, const Eigen::Vector3d & close, bool check_enc, Q5 & q)
{
  double best = 1e18;
  Q5 qb = q;
  // 앞 자세를 씨앗으로, 안 풀리면 (손목을 크게 돌려야 할 때 — 누운 퍽을 쥔 손 → 받침의 접선 닫힘) 기본 씨앗들로
  for (int pass = 0; pass < 2 && best > 1e17; ++pass) {
    for (const auto & s : arm_ik(m, p, app, close, pass == 0 ? std::vector<Q5>{q} : std::vector<Q5>{}, 180.0)) {
      // arm_ik 의 180° 주기는 닫힘축 ± 를 같게 봄 → 손목이 180° 반대인 해를 거름 (받침에서 퍽 긴 쪽이 반대로 감)
      if (!ik_ok(m, s, check_enc) || arm_grip(m, s.q).close.dot(close) <= 0.0) {continue;}
      double dd = 0.0;
      for (int j = 0; j < 5; ++j) {dd += std::fabs(s.q[j] - q[j]);}
      if (dd < best) {best = dd; qb = s.q;}
    }
  }
  if (best > 1e17) {return false;}
  q = qb;
  return true;
}
}  // namespace

bool lying_grasp(
  const ArmModel & m, double x, double y, double top_z, double axis_deg, const std::vector<double> & heights_mm,
  const LyingGraspCfg & cfg, LyingGrasp & out, std::string & why, double max_tilt_deg,
  const std::function<bool(const LyingGrasp &, std::string &)> & accept)
{
  if (heights_mm.size() != 3) {why = "높이는 위 · 내림 · 쥐기 3 개"; return false;}
  const Eigen::Vector3d a(std::cos(axis_deg / kDeg), std::sin(axis_deg / kDeg), 0.0);
  // 쥐는 끝: 긴축이 반지름 쪽 (45° 안) 이면 바깥 끝 — J1 쪽 끝을 쥐면 받침에서 긴 쪽이 J1 쪽으로 가려면 손목이 180° 돌아야 함 (J5 범위 밖).
  //   접선 쪽이면 J1 에 가까운 끝 먼저 (덜 뻗음). 안 되면 반대 끝
  const Eigen::Vector2d p0(x, y), a2 = a.head<2>();
  const Eigen::Vector3d e_near = (p0 + cfg.grip_offset * a2 - m.j1_xy).norm() <= (p0 - cfg.grip_offset * a2 - m.j1_xy).norm() ? a : Eigen::Vector3d(-a);
  const bool radial = std::fabs(a2.dot((p0 - m.j1_xy).normalized())) >= std::cos(M_PI / 4.0);
  const Eigen::Vector3d e_first = radial ? Eigen::Vector3d(-e_near) : e_near;
  const Eigen::Vector3d c0(x, y, top_z - cfg.side / 2.0);
  const double tip_floor = top_z + heights_mm.back() / 1000.0;   // 수직 손 손끝 높이 = 손가락이 내려갈 수 있는 가장 낮은 곳
  const Q5 & q0 = m.template_q;
  auto key = [&](const Q5 & q) {   // 손목이 받침 자세에 가깝고 J2–J4 가 덜 꺾인 해 (pick_poses 와 같음)
      double dd = 0.0;
      for (int j = 1; j < 4; ++j) {dd += (q[j] - q0[j]) * (q[j] - q0[j]);}
      return std::make_pair(std::round(std::fabs(q[4] - m.q5_rest) / 0.3), std::sqrt(dd));
    };
  std::string last = "기울임 후보 없음";
  char buf[200];
  for (double tilt : cfg.tilts_deg) {
    if (tilt > max_tilt_deg + 1e-9) {continue;}
    for (const Eigen::Vector3d & e1 : {e_first, Eigen::Vector3d(-e_first)}) {
      const RollGeo geo = roll_geometry(m, c0, a, e1, tilt / kDeg, tip_floor, cfg);
      if (geo.rho * kDeg > cfg.max_roll_deg + 1e-9) {
        std::snprintf(buf, sizeof(buf), "손 %.0f°: 퍽이 %.0f° 굴러야 함 (%.0f° 까지)", tilt, geo.rho * kDeg, cfg.max_roll_deg);
        last = buf;
        continue;
      }
      if (geo.margin < cfg.min_margin - 1e-9) {
        std::snprintf(buf, sizeof(buf), "손 %.0f°: 퍽이 %.0f° 굴러야 해 열린 손가락 사이 여유 %.1f mm", tilt, geo.rho * kDeg, geo.margin * 1000.0);
        last = buf;
        continue;
      }
      if (geo.d_up < cfg.min_depth - 1e-9) {
        std::snprintf(buf, sizeof(buf), "손 %.0f°: 손가락이 바닥에 닿지 않게 하면 %.0f mm 만 쥠", tilt, geo.d_up * 1000.0);
        last = buf;
        continue;
      }
      // 쥐기 IK: 닫힘축 부호 둘 다 (손가락은 대칭, 손목 180° 차이) — 손목이 덜 도는 쪽 먼저
      std::vector<std::pair<Q5, Eigen::Vector3d>> grips;
      for (double sg : {1.0, -1.0}) {
        bool have = false;
        Q5 qb{};
        for (const auto & s : arm_ik(m, geo.tip, geo.app, sg * geo.close, {}, 180.0)) {
          if (ik_ok(m, s, true) && arm_grip(m, s.q).close.dot(sg * geo.close) > 0.0 && (!have || key(s.q) < key(qb))) {have = true; qb = s.q;}
        }
        if (have) {grips.push_back({qb, sg * geo.close});}
      }
      if (grips.empty()) {
        std::snprintf(buf, sizeof(buf), "손 %.0f°: 쥐는 자세 IK 안 됨 (손끝 %.3f, %.3f, %.3f)", tilt, geo.tip.x(), geo.tip.y(), geo.tip.z());
        last = buf;
        continue;
      }
      std::sort(grips.begin(), grips.end(), [&](const auto & u, const auto & w) {return key(u.first) < key(w.first);});
      for (const auto & [qg, close] : grips) {
        // 위 · 내림: 손 축을 따라 뒤로 그 높이까지 (손가락이 퍽 옆면과 나란히 들어감)
        std::vector<PickPose> poses(3);
        poses[2] = {"h" + std::to_string(static_cast<int>(heights_mm[2])), q_to_ticks(m, qg), q_to_ticks(m, encoder_goal_q(m, qg, 0.0)), tilt};
        Q5 q = qg;
        bool ok = true;
        for (int k = 1; k >= 0 && ok; --k) {
          const double s_back = (top_z + heights_mm[k] / 1000.0 - geo.tip.z()) / -geo.app.z();
          ok = nearest_ik(m, geo.tip - s_back * geo.app, geo.app, close, true, q);
          if (!ok) {
            std::snprintf(buf, sizeof(buf), "손 %.0f°: 높이 %.0f mm 자세 IK 안 됨", tilt, heights_mm[k]);
            last = buf;
            break;
          }
          poses[k] = {"h" + std::to_string(static_cast<int>(heights_mm[k])), q_to_ticks(m, q), q_to_ticks(m, encoder_goal_q(m, q, 0.0)), tilt};
        }
        if (!ok) {continue;}
        LyingGrasp g;
        g.poses = poses;
        g.q_grasp = qg;
        g.tip = geo.tip;
        g.app = geo.app;
        g.close = close;
        g.c = geo.c;
        g.n = geo.n;
        g.a_end = -e1;
        g.tilt_deg = tilt;
        g.roll_deg = geo.rho * kDeg;
        g.pitch_deg = std::asin(std::clamp(geo.app.dot(a), -1.0, 1.0)) * kDeg;
        g.depth_mm = geo.d_up * 1000.0;
        g.margin_mm = geo.margin * 1000.0;
        std::string w;
        if (accept && !accept(g, w)) {last = w; continue;}
        out = g;
        why.clear();
        return true;
      }
    }
  }
  why = last;
  return false;
}

bool park_nodes(
  const ArmModel & m, const ParkCfg & c, const LyingGrasp & g, const Q5 & q_from, std::vector<ParkNode> & out,
  double & elev_used, std::string & why)
{
  const Eigen::Vector3d Z = Eigen::Vector3d::UnitZ();
  const Eigen::Vector2d sc = c.center;
  const Eigen::Vector2d rdir = (sc - m.j1_xy).normalized();
  const Eigen::Vector3d R(rdir.x(), rdir.y(), 0.0), T(-rdir.y(), rdir.x(), 0.0);
  // 손 좌표의 퍽 (쥔 뒤)
  const Eigen::Matrix3d Hg = hand_frame(g.app, g.close);
  const Eigen::Vector3d c_h = Hg.transpose() * (g.c - g.tip);
  const Eigen::Vector3d a_h = Hg.transpose() * g.a_end;
  const Eigen::Vector3d k_h = Hg.transpose() * g.close;   // 단면 축 (손가락 쪽) = 손 x
  const Eigen::Vector3d n_h = Hg.transpose() * g.n;
  const double half = c.side / 2.0;
  auto app_of = [&](double tilt_deg) {
      const double t = tilt_deg / kDeg;
      return Eigen::Vector3d(std::sin(t) * R - std::cos(t) * Z);
    };
  Q5 q = q_from;
  double sigma = 1.0;
  std::vector<ParkNode> nodes;
  auto add = [&](const std::string & nm, const Eigen::Vector3d & p, double tilt_deg, double kg, bool slow, const char * grip) {
      Q5 qn = q;
      if (!nearest_ik(m, p, app_of(tilt_deg), sigma * T, false, qn)) {
        char buf[160];
        std::snprintf(buf, sizeof(buf), "%s 자세 IK 안 됨 (%.3f, %.3f, z %.3f, 기울임 %.0f°)", nm.c_str(), p.x(), p.y(), p.z(), tilt_deg);
        why = buf;
        return false;
      }
      q = qn;
      nodes.push_back({nm, q, kg, slow, grip});
      return true;
    };
  auto at = [&](double r, double z) {
      const Eigen::Vector2d xy = sc + r * rdir;
      return Eigen::Vector3d(xy.x(), xy.y(), z);
    };
  // 넣기: 긴 쪽 끝 = J1 쪽 아래 (−R · −Z) 로 elev. 손 기울임 θ · 닫힘 부호 σ 를 찾음 (쥘 때 퍽이 손에서 기울었으면 θ = elev ± 그만큼)
  bool ok = false;
  double elev = c.elev_deg;
  for (; elev >= c.min_elev_deg - 1e-6 && !ok; elev -= 5.0) {
    const double e = elev / kDeg;
    const Eigen::Vector3d want = -std::cos(e) * R - std::sin(e) * Z;
    struct Cand {double th, sg, err;};
    std::vector<Cand> cands;
    for (double sg : {1.0, -1.0}) {
      Cand best{0.0, sg, 1e9};
      for (double th = -30.0; th <= 80.0 + 1e-9; th += 0.25) {
        const Eigen::Vector3d a_s = hand_frame(app_of(th), sg * T) * a_h;
        const double err = std::acos(std::clamp(a_s.dot(want), -1.0, 1.0)) * kDeg;
        if (err < best.err) {best = {th, sg, err};}
      }
      if (best.err < 1.0) {cands.push_back(best);}
    }
    std::sort(cands.begin(), cands.end(), [&](const Cand & x, const Cand & y) {return std::fabs(x.th - elev) < std::fabs(y.th - elev);});
    for (const auto & cd : cands) {
      nodes.clear();
      q = q_from;
      sigma = cd.sg;
      const Eigen::Matrix3d Hs = hand_frame(app_of(cd.th), sigma * T);
      const Eigen::Vector3d a_s = Hs * a_h, k_s = Hs * k_h, n_s = Hs * n_h;
      // 긴 쪽 끝면 중심 = 받침 중심 − end_inset (J1 쪽), 가장 낮은 모서리 = 바닥 + insert_clear
      Eigen::Vector3d f = at(-c.end_inset, 0.0);
      f.z() = c.floor_z + c.insert_clear + half * (std::fabs(k_s.z()) + std::fabs(n_s.z()));
      const Eigen::Vector3d p_ins = f - (c.length / 2.0) * a_s - Hs * c_h;
      // 받침 바깥 위 수직 (반지름 +0.04, 바닥 + 62.5 mm) 에서 기울이며 넣기 — 위치 · 기울임을 같이 나눠 감
      const Eigen::Vector3d p_over = at(0.04, c.floor_z + 0.0625);
      ok = add("park_over", p_over, 0.0, c.held_kg, false, "");
      for (int k = 1; k <= c.tilt_steps && ok; ++k) {
        const double fk = static_cast<double>(k) / c.tilt_steps;
        ok = add("park_tilt" + std::to_string(k), p_over + (p_ins - p_over) * fk, cd.th * fk, c.held_kg, true,
          k == c.tilt_steps ? "open" : "");
        if (ok && c.insert_dj3 != 0.0) {nodes.back().q[2] += c.insert_dj3 * fk;}   // 실물 맞춤 (11-07 사용자: 넣을 때 J3 를 조금 더 올려)
      }
      if (!ok) {continue;}
      // 손 빼기 (같은 기울임으로 위 · 바깥) → 바깥 위 수직 · 오므림
      ok = add("park_back", p_ins + 0.01 * R + 0.03 * Z, cd.th, 0.0, true, "") &&
        add("park_push_over", at(c.push_r0, std::max(c.floor_z + 0.0525, c.floor_z + c.push_h + 0.02)), 0.0, 0.0, false, "close");
      if (ok) {elev_used = elev; break;}
    }
  }
  if (!ok) {return false;}
  // 밀기: 받침 바깥에서 바닥 위 push_h 로 안쪽으로, 선 퍽 바깥 면 + 4 mm + 손가락 반두께 앞에서 멈춤
  const double x_stop = -c.inner_half + c.side + 0.004 + 0.006;
  const double pz = c.floor_z + c.push_h;
  const double r0 = c.push_r0;
  if (!add("park_push_down", at(r0, pz + c.push_dz), 0.0, 0.0, true, "")) {return false;}
  for (int k = 1; k <= c.push_steps; ++k) {
    const double f = static_cast<double>(k) / c.push_steps;
    if (!add("park_push" + std::to_string(k), at(r0 + (x_stop - r0) * f, pz + c.push_dz * (1.0 - f)), 0.0, 0.0, true, "")) {return false;}
  }
  if (!add("park_push_back", at(r0, pz), 0.0, 0.0, true, "") || !add("park_push_up", at(r0, std::max(c.floor_z + 0.0525, pz)), 0.0, 0.0, false, "")) {
    return false;
  }
  // 선 퍽 다시 쥐기 (선 자리 = 받침 중심 + regrasp_r, 시뮬 −6 mm). 손끝 = 윗면 − regrasp_depth (받침 벽 위)
  const double ptop = c.floor_z + c.length;
  if (!add("park_re_above", at(c.regrasp_r, ptop + 0.0075), 0.0, 0.0, false, "open") ||
    !add("park_re_down", at(c.regrasp_r, ptop - c.regrasp_depth), 0.0, 0.0, true, "close") ||
    !add("park_re_lift", at(c.regrasp_r, ptop + 0.0075), 0.0, c.held_kg, true, ""))
  {
    return false;
  }
  out = nodes;
  why.clear();
  return true;
}

}  // namespace manipulation
