#include "manipulation/park_stand.hpp"
#include "manipulation/look_at.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>

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
      const Eigen::Vector3d p_ins = f - (c.length / 2.0) * a_s - Hs * c_h - c.insert_extra * Z;
      // 받침 바깥 위 수직 (반지름 +0.04, 바닥 + 62.5 mm) 에서 기울이며 넣기 — 위치 · 기울임을 같이 나눠 감
      //   insert_back > 0: 기울이기는 insert_back 위 점까지 (다 기울인 자세), 거기서 그 자세로 곧게 내려 넣기 (그냥 기울이며 넣으면 턱에 2–3 mm 걸침)
      const bool straight = c.insert_back > 0.0;
      // 넣는 끝점을 손등 (손목 카메라) 쪽으로 release_up 만큼 — 넣으면서 같이 (놓기 전에 손목을 카메라 달린 위로 5 mm, 따로 말고 넣으며)
      const Eigen::Vector3d p_end = p_ins + c.release_up * Hs.col(1);
      const Eigen::Vector3d p_tilt_end = straight ? Eigen::Vector3d(p_ins + c.insert_back * Z - c.insert_from_back * R) : p_end;
      const Eigen::Vector3d p_over = at(0.04, c.floor_z + 0.0625);
      const bool pull = c.release_pull > 0.0 && c.release_pull_steps > 0;
      const char * release = pull ? "loose" : "open";
      ok = add("park_over", p_over, 0.0, c.held_kg, false, "");
      for (int k = 1; k <= c.tilt_steps && ok; ++k) {
        const double fk = static_cast<double>(k) / c.tilt_steps;
        const char * g = (!straight && k == c.tilt_steps) ? release : (k == c.loose_step ? "loose" : "");
        ok = add("park_tilt" + std::to_string(k), p_over + (p_tilt_end - p_over) * fk, cd.th * fk, c.held_kg, true, g);
        if (ok && c.insert_dj3 != 0.0) {nodes.back().q[2] += c.insert_dj3 * fk;}   // 실물 맞춤 (넣을 때 J3 를 조금 더 올려)
      }
      if (ok && straight) {
        ok = add("park_insert", p_end, cd.th, c.held_kg, true, release);
        if (ok && c.insert_dj3 != 0.0) {nodes.back().q[2] += c.insert_dj3;}
      }
      Eigen::Vector3d p_rel = p_end;
      // 당기며 놓기: 넣는 점에서 같은 높이 · 같은 기울임으로 J1 쪽 (뒤) 으로 곧게 release_pull 만큼 (+ release_pull_dz 위아래), 끝점에서 놓음
      //   (아래 끝 둘레로 돌려 당기면 손이 10 mm 올라감)
      const Eigen::Vector3d p_pull0 = p_end;
      for (int k = 1; pull && ok && k <= c.release_pull_steps; ++k) {
        const double fk = static_cast<double>(k) / c.release_pull_steps;
        p_rel = p_pull0 + fk * (c.release_pull_dz * Z - c.release_pull * R);
        ok = add("park_pull" + std::to_string(k), p_rel, cd.th, c.held_kg, true, k == c.release_pull_steps ? "open" : "");
        if (ok && c.insert_dj3 != 0.0) {nodes.back().q[2] += c.insert_dj3;}
      }
      if (!ok) {continue;}
      if (c.press_mode) {
        // 꽂아 누르기 (밀기 대신): 놓은 손 (같은 기울임) 을 press_top 높이까지 곧게 올려 퍽에서 뺌 → 그 자리에서 손을 수직으로 · 닫음
        //   → 받침 뒤 (J1 쪽, 퍽이 걸리는 쪽) press_r 위로 → 수직으로 곧게 press_h 까지 내려 꽂음 (걸린 퍽 뒷부분을 눌러 넣어 세움) → 다시 위로
        //   수직 손은 받침 위 손끝 바닥 + 66 mm 까지 닿음 (75 는 안 됨, IK 탐색) → 선 퍽 윗면 (60) 위로 넘어감
        const double z_top = c.floor_z + c.press_top;
        Eigen::Vector3d p_lift = p_rel;
        p_lift.z() = std::max(p_rel.z(), z_top);
        ok = add("park_press_lift", p_lift, cd.th, 0.0, true, "") &&
          add("park_press_turn", p_lift, 0.0, 0.0, true, "close") &&
          add("park_press_over", at(c.press_r, z_top), 0.0, 0.0, true, "");
        for (int k = 1; k <= c.press_steps && ok; ++k) {
          const double f = static_cast<double>(k) / c.press_steps;
          ok = add("park_press" + std::to_string(k), at(c.press_r, z_top + (c.floor_z + c.press_h - z_top) * f), 0.0, 0.0, true, "");
        }
        ok = ok && add("park_press_up", at(c.press_r, z_top), 0.0, 0.0, true, "");
      } else {
        // 손 빼기 (같은 기울임으로 위 · 바깥) → 바깥 위 수직 · 오므림
        ok = add("park_back", p_rel + 0.01 * R + 0.03 * Z, cd.th, 0.0, true, "") &&
          add("park_push_over", at(c.push_r0, std::max(c.floor_z + 0.0525, c.floor_z + c.push_h + 0.02)), 0.0, 0.0, false, "close");
      }
      if (ok) {elev_used = elev; break;}
    }
  }
  if (!ok) {
    if (why.empty()) {   // 어느 넣는 각에서도 손 기울임 · 닫힘 부호 후보가 없어 add 가 한 번도 안 불림
      char buf[160];
      std::snprintf(buf, sizeof(buf), "받침에 넣는 각 %.0f~%.0f° 에서 쥔 퍽 긴축을 맞출 손 기울임이 없음", c.elev_deg, c.min_elev_deg);
      why = buf;
    }
    return false;
  }
  if (!c.press_mode) {
    // 밀기: 받침 바깥에서 바닥 위 push_h 로 안쪽으로, 선 퍽 바깥 면 + 4 mm + 손가락 반두께 앞에서 멈춤
    const double x_stop = -c.inner_half + c.side + 0.004 + 0.006 - c.push_extra;
    const double pz = c.floor_z + c.push_h;
    const double r0 = c.push_r0;
    // 굴삭기처럼: 기울임 −scoop (손끝 J1 쪽) · 높이 pz + push_dz − push_low 에서 시작 → 끝점에서 기울임 0 · 높이 pz. IK 안 되면 5° 씩 줄여 다시
    {
      const size_t n0 = nodes.size();
      const Q5 q0 = q;
      bool pushed = false;
      for (double scoop = std::max(0.0, c.push_scoop_deg); !pushed; scoop -= 5.0) {
        if (scoop < 0.0) {scoop = 0.0;}
        nodes.resize(n0);
        q = q0;
        const double z0 = pz + c.push_dz - (scoop > 0.0 ? c.push_low : 0.0);
        pushed = add("park_push_down", at(r0, z0), -scoop, 0.0, true, "");
        for (int k = 1; k <= c.push_steps && pushed; ++k) {
          const double f = static_cast<double>(k) / c.push_steps;
          pushed = add("park_push" + std::to_string(k), at(r0 + (x_stop - r0) * f, z0 + (pz - z0) * f), -scoop * (1.0 - f), 0.0, true, "");
          // J3 더 굽힘: 밀기 가운데서 가장 많이, 끝점에서 다시 0 (굽힌 채 끝나면 그리퍼가 받침에 닿음)
          if (pushed && c.push_dj3 != 0.0) {nodes.back().q[2] += c.push_dj3 * std::sin(M_PI * f);}
          if (pushed && c.push_dj2 != 0.0) {   // J2 더 올림: + = 손끝 위 (관절 부호는 FK 로)
            Q5 qu = nodes.back().q;
            qu[1] += 1e-3;
            const double up = arm_grip(m, qu).tip.z() > arm_grip(m, nodes.back().q).tip.z() ? 1.0 : -1.0;
            nodes.back().q[1] += up * c.push_dj2 * std::sin(M_PI * f);
          }
        }
        if (!pushed && scoop <= 0.0) {return false;}
      }
    }
    if (!add("park_push_back", at(r0, pz), 0.0, 0.0, true, "") || !add("park_push_up", at(r0, std::max(c.floor_z + 0.0525, pz)), 0.0, 0.0, false, "")) {
      return false;
    }
  }
  // 선 퍽 다시 쥐기 (선 자리 = 받침 중심 + regrasp_r, 시뮬 −6 mm). 손끝 = 윗면 − regrasp_depth (받침 벽 위)
  const double ptop = c.floor_z + c.length;
  const auto at_re = [&](double z) -> Eigen::Vector3d {return at(c.regrasp_r, z) + Eigen::Vector3d(c.regrasp_xy.x(), c.regrasp_xy.y(), 0.0);};   // + 고정 보정
  if (!add("park_re_above", at_re(ptop + 0.0075), 0.0, 0.0, false, "open") ||
    !add("park_re_down", at_re(ptop - c.regrasp_depth), 0.0, 0.0, true, "close") ||
    !add("park_re_lift", at_re(ptop + 0.0075), 0.0, c.held_kg, true, ""))
  {
    return false;
  }
  out = nodes;
  why.clear();
  return true;
}

namespace
{
// 쥔 손가락 (닫힘 방향 양쪽, 손끝 면에서 뒤로 30 mm) 중 받침 벽 · 받침 안 위에 있는 점의 가장 낮은 높이 − 벽 윗면 (+ = 위)
//   min_z: 모든 손가락 점 중 가장 낮은 높이
double upright_finger_gap(const Grip & g, const Eigen::Vector2d & sc, const UprightCfg & u, double grip_q, double wall_top, double & min_z)
{
  const Eigen::Vector3d app = g.app.normalized(), cl = g.close.normalized(), b = app.cross(cl).normalized();
  double gap = 1e9;
  min_z = 1e9;
  for (int side : {-1, 1}) {
    for (double xc : {u.finger_inner0 + grip_q, 0.5 * (u.finger_inner0 + u.finger_outer0) + grip_q, u.finger_outer0 + grip_q}) {
      for (int k = -4; k <= 4; ++k) {
        for (double za = 0.0; za <= 0.030 + 1e-9; za += 0.005) {
          const Eigen::Vector3d p = g.tip + side * xc * cl + (u.finger_half_w * k / 4.0) * b - za * app;
          min_z = std::min(min_z, p.z());
          const double dx = std::fabs(p.x() - sc.x()), dy = std::fabs(p.y() - sc.y());
          if (dx <= u.cup_out_r + 1e-4 && dy <= u.cup_out_t + 1e-4) {gap = std::min(gap, p.z() - wall_top);}   // 벽 위 · 받침 안
        }
      }
    }
  }
  return gap;
}

Eigen::Matrix3d rot_z(double a)
{
  Eigen::Matrix3d r;
  r << std::cos(a), -std::sin(a), 0.0, std::sin(a), std::cos(a), 0.0, 0.0, 0.0, 1.0;
  return r;
}
}  // namespace

bool park_upright_nodes(
  const ArmModel & m, const ParkCfg & c, const UprightCfg & u, const UprightHeld & h, const Q5 & q_from, std::vector<ParkNode> & out,
  UprightInfo & info, std::string & why)
{
  const Eigen::Vector3d Z = Eigen::Vector3d::UnitZ();
  const Eigen::Vector2d sc = c.center;
  const Grip g = arm_grip(m, h.q_grasp);
  const Eigen::Vector3d app_p = g.app.normalized(), cl_p = g.close.normalized();
  info = UprightInfo{};
  info.tilt_deg = std::acos(std::clamp(-app_p.z(), -1.0, 1.0)) * kDeg;
  info.depth_mm = (h.bottom_z + u.puck_h - g.tip.z()) * 1000.0;
  char buf[200];
  if (!(info.tilt_deg <= u.max_tilt_deg + 0.5)) {
    std::snprintf(buf, sizeof(buf), "손 %.0f° 기울여 쥠 — 받침은 %.0f° 까지 (40° 는 놓는 자세 J2 가 tick_lo 끝)", info.tilt_deg, u.max_tilt_deg);
    why = buf;
    return false;
  }
  // 손 ↔ 퍽 축: 퍽 중심에서 손끝까지 (수평). 닫힘 방향은 손가락이 가운데로 모으므로 0, 손가락 폭 방향 (b) 만 남김
  Eigen::Vector3d off(g.tip.x() - h.puck_xy.x(), g.tip.y() - h.puck_xy.y(), 0.0);
  Eigen::Vector3d clh(cl_p.x(), cl_p.y(), 0.0);
  if (clh.norm() > 1e-6) {clh.normalize(); off -= off.dot(clh) * clh;}
  // 받침 위 손 = 쥔 손을 수직축 둘레로 Δ 돌림 (퍽은 수직 그대로). Δ = 받침에서의 손끝 방위 − 쥘 때 손끝 방위 (팔 평면을 따라감)
  const double az_p = std::atan2(g.tip.y() - m.j1_xy.y(), g.tip.x() - m.j1_xy.x());
  double dlt = std::atan2(sc.y() - m.j1_xy.y(), sc.x() - m.j1_xy.x()) - az_p;
  for (int it = 0; it < 4; ++it) {
    const Eigen::Vector3d t = Eigen::Vector3d(sc.x(), sc.y(), 0.0) + rot_z(dlt) * off;
    dlt = std::atan2(t.y() - m.j1_xy.y(), t.x() - m.j1_xy.x()) - az_p;
  }
  const Eigen::Matrix3d Rd = rot_z(dlt);
  const Eigen::Vector3d app_s = Rd * app_p, cl_s = Rd * cl_p;
  const Eigen::Vector3d off_s = Rd * off;
  // 네모: 손가락 면 ↔ 받침 접선 벽 (받침은 반지름 · 접선으로 놓임)
  const Eigen::Vector2d rdir = (sc - m.j1_xy).normalized(), tdir(-rdir.y(), rdir.x());
  if (h.square) {
    const Eigen::Vector2d c2(cl_s.x(), cl_s.y());
    const double a = std::atan2(c2.dot(rdir), c2.dot(tdir)) * kDeg;
    info.face_deg = std::fabs(std::remainder(a, 90.0));
    if (info.face_deg > u.max_face_deg + 1e-9) {
      std::snprintf(buf, sizeof(buf), "네모 면이 받침 벽과 %.0f° 틀어짐 (%.0f° 까지: 44 mm 안에 %.1f mm 네모)", info.face_deg, u.max_face_deg, h.width * 1000.0);
      why = buf;
      return false;
    }
  }
  const double grip_hold = std::max(0.0, h.width / 2.0 - u.finger_inner0);
  const double wall_top = c.floor_z + u.wall_h;
  auto tip_at = [&](double b) {   // 퍽 바닥 = 받침 바닥 + b 일 때 손끝
      return Eigen::Vector3d(sc.x() + off_s.x(), sc.y() + off_s.y(), c.floor_z + b + (g.tip.z() - h.bottom_z));
    };
  // 놓는 점: 가장 낮은 b (IK · 손가락이 벽 위). 씨앗 = 들기 자세 → 안 되면 기본 씨앗 (손목이 받침 쪽으로 크게 돎)
  Q5 q_rel = q_from;
  double b_rel = -1.0;
  std::string last = "IK 안 됨";
  // 손가락 ↔ 벽은 손 자세 (app_s · cl_s, IK 가 1° 안으로 맞춤) 와 손끝 자리만으로 정해짐 → IK 없이 b 를 먼저 올려 보고, 그 b 부터 IK
  auto grip_at = [&](const Eigen::Vector3d & tip) {return Grip{tip, app_s, cl_s};};
  double b_fing = 0.0, mz = 0.0;
  while (b_fing <= u.max_b + 1e-9 && upright_finger_gap(grip_at(tip_at(b_fing)), sc, u, grip_hold, wall_top, mz) < u.finger_clear - 1e-9) {
    b_fing += 0.0005;
  }
  // 0.5 mm 격자 중 가장 낮은 b. 씨앗은 늘 들기 자세 (실패해도 안 바뀜) → 각 b 의 결과는 서로 무관 → 2 mm 마다 먼저 보고 (IK 횟수 줄임),
  //   처음 된 2 mm 점 앞 0.5 mm 점 셋을 낮은 것부터 다시 봄. 놓는 높이 · 손가락 벽 위 3 mm 조건은 같음
  std::vector<double> bs;
  for (double b = b_fing; b <= u.max_b + 1e-9; b += 0.0005) {bs.push_back(b);}
  auto try_b = [&](size_t k, Q5 & q) {
      q = q_from;
      if (!nearest_ik(m, tip_at(bs[k]), app_s, cl_s, true, q)) {last = "놓는 자세 IK 안 됨"; return false;}
      const double gap = upright_finger_gap(arm_grip(m, q), sc, u, grip_hold, wall_top, mz);
      if (gap < u.finger_clear - 1e-9) {
        std::snprintf(buf, sizeof(buf), "쥔 손가락이 받침 벽 윗면 위 %.1f mm (%.1f mm 필요)", gap * 1000.0, u.finger_clear * 1000.0);
        last = buf;
        return false;
      }
      return true;
    };
  constexpr size_t kCoarse = 4;   // 2 mm
  for (size_t kc = 0; !bs.empty() && kc < bs.size() + kCoarse - 1 && b_rel < 0.0; kc += kCoarse) {
    const size_t k = std::min(kc, bs.size() - 1);
    Q5 q{};
    if (!try_b(k, q)) {continue;}
    size_t best = k;
    Q5 qb = q;
    for (size_t j = (k >= kCoarse ? k - kCoarse + 1 : 0); j < k; ++j) {   // 앞 실패 굵은 점과 이 점 사이 (낮은 것부터)
      Q5 qj{};
      if (try_b(j, qj)) {best = j; qb = qj; break;}
    }
    q_rel = qb;
    b_rel = bs[best];
  }
  if (b_fing > u.max_b + 1e-9) {last = "쥔 손가락이 받침 벽에 걸림";}
  if (b_rel < 0.0) {
    std::snprintf(buf, sizeof(buf), "받침에 놓는 자세 없음 (퍽 바닥 0–%.0f mm, 손 %.0f°, 마지막: %s)", u.max_b * 1000.0, info.tilt_deg, last.c_str());
    why = buf;
    return false;
  }
  info.b_mm = b_rel * 1000.0;
  std::vector<ParkNode> nodes;
  // 받침 위 · 곧게 내려 넣기 (위 점에서 놓는 점 쪽으로 이어 풂, 갈래 안 바뀜)
  Q5 q_over = q_rel;
  if (!nearest_ik(m, tip_at(b_rel + u.over_up), app_s, cl_s, true, q_over)) {
    why = "받침 위 점 IK 안 됨";
    return false;
  }
  // 가운데 점: 들기 손끝과 받침 위 손끝 가운데 (높이 = 받침 위), 손은 그 자리 방위만큼 돌림. 안 되면 바로 받침 위
  {
    const Grip gf = arm_grip(m, q_from);
    const Eigen::Vector3d po = tip_at(b_rel + u.over_up);
    const Eigen::Vector3d pm(0.5 * (gf.tip.x() + po.x()), 0.5 * (gf.tip.y() + po.y()), po.z());
    const double dm = std::atan2(pm.y() - m.j1_xy.y(), pm.x() - m.j1_xy.x()) - az_p;
    Q5 qm = q_from;
    if (nearest_ik(m, pm, rot_z(dm) * app_p, rot_z(dm) * cl_p, true, qm)) {
      nodes.push_back({"pk_mid", qm, h.held_kg, false, ""});
      info.mid = true;
    }
  }
  nodes.push_back({"pk_over", q_over, h.held_kg, false, ""});
  Q5 q = q_over;
  for (int k = 1; k <= u.down_steps; ++k) {
    const double b = b_rel + u.over_up * (1.0 - static_cast<double>(k) / u.down_steps);
    if (k == u.down_steps) {
      q = q_rel;
    } else {
      // 씨앗 하나로만 (갈래가 바뀌면 곧게 안 내려감)
      bool ok = false;
      for (const auto & s : arm_ik(m, tip_at(b), app_s, cl_s, {q}, 180.0)) {
        if (s.pos_mm < 1.0 && s.app_deg < 1.0 && s.roll_deg < 2.0 && ticks_in_range(m, q_to_ticks(m, s.q)) &&
          ticks_in_range(m, q_to_ticks(m, encoder_goal_q(m, s.q, 0.0))) && arm_grip(m, s.q).close.dot(cl_s) > 0.0)
        {
          q = s.q;
          ok = true;
          break;
        }
      }
      if (!ok) {std::snprintf(buf, sizeof(buf), "곧게 넣기 %d 번째 점 IK 안 됨", k); why = buf; return false;}
    }
    nodes.push_back({"pk_down" + std::to_string(k), q, h.held_kg, true, k == u.down_steps ? "open" : ""});
  }
  // 연 손을 같은 기울임으로 곧게 올림: 연 손가락 가장 낮은 곳이 선 퍽 윗면 + up_clear 위
  const double ptop = c.floor_z + u.puck_h;
  bool up_ok = false;
  double up = 0.0025;
  for (; up <= 0.080 + 1e-9; up += 0.0025) {   // 연 손가락 바닥 높이도 손 자세 · 손끝 자리로 (IK 없이) 먼저
    upright_finger_gap(grip_at(tip_at(b_rel) + up * Z), sc, u, u.grip_open, wall_top, mz);
    if (mz >= ptop + u.up_clear - 1e-9) {break;}
  }
  for (; up <= 0.080 + 1e-9 && !up_ok; up += 0.0025) {
    Q5 qu = q_rel;
    if (!nearest_ik(m, tip_at(b_rel) + up * Z, app_s, cl_s, false, qu)) {continue;}
    upright_finger_gap(arm_grip(m, qu), sc, u, u.grip_open, wall_top, mz);
    if (mz >= ptop + u.up_clear - 1e-9) {
      nodes.push_back({"pk_up", qu, 0.0, true, ""});
      q = qu;
      up_ok = true;
    }
  }
  if (!up_ok) {why = "놓은 뒤 연 손을 선 퍽 위로 올리는 자세 IK 안 됨"; return false;}
  // 수직 손으로 받침 다시 쥐기 (park_nodes 의 park_re_* 와 같은 자리 · 깊이 · 닫힘 = 받침 접선, 쥔 손과 같은 쪽)
  const Eigen::Vector3d T(tdir.x(), tdir.y(), 0.0);
  const double sg = cl_s.dot(T) >= 0.0 ? 1.0 : -1.0;
  const Eigen::Vector2d rg = sc + c.regrasp_r * rdir + c.regrasp_xy;   // + 고정 보정 (park_regrasp_xy_mm)
  Q5 qa = q, qd{};
  if (!nearest_ik(m, Eigen::Vector3d(rg.x(), rg.y(), ptop + 0.0075), -Z, sg * T, false, qa)) {why = "park_re_above IK 안 됨"; return false;}
  qd = qa;
  if (!nearest_ik(m, Eigen::Vector3d(rg.x(), rg.y(), ptop - c.regrasp_depth), -Z, sg * T, false, qd)) {why = "park_re_down IK 안 됨"; return false;}
  nodes.push_back({"park_re_above", qa, 0.0, false, ""});   // 손은 이미 열림 (pk_down 끝에서 엶)
  nodes.push_back({"park_re_down", qd, 0.0, true, "close"});
  nodes.push_back({"park_re_lift", qa, c.held_kg, true, ""});
  out = nodes;
  why.clear();
  return true;
}

bool park_look_view(const ArmModel & m, const Eigen::Isometry3d & T5c, const ParkCfg & c, const UprightCfg & u, const ParkLookCfg & L,
  const Q5 & q, ParkLookInfo & info)
{
  const double dz = model_to_base_z(m);
  const Eigen::Isometry3d ci = look_camera_pose(m, T5c, q).inverse();
  const Eigen::Vector2d rd = (c.center - m.j1_xy).normalized(), td(-rd.y(), rd.x());
  const Eigen::Vector3d R(rd.x(), rd.y(), 0.0), T(td.x(), td.y(), 0.0);
  const Eigen::Vector3d top(c.center.x(), c.center.y(), c.floor_z + u.puck_h + dz);   // base_link
  info.u0 = info.v0 = 1e9;
  info.u1 = info.v1 = -1e9;
  info.dmin = 1e9;
  bool front = true;
  for (double dr : {-L.shift, 0.0, L.shift}) {
    for (double dt : {-L.shift, 0.0, L.shift}) {
      for (int k = 0; k < 12; ++k) {
        const double a = k * M_PI / 6.0;
        const Eigen::Vector3d p = ci * (top + dr * R + dt * T + L.puck_r * (std::cos(a) * R + std::sin(a) * T));
        if (p.z() < 0.02) {front = false; continue;}
        const double uu = L.cx + L.fx * p.x() / p.z(), vv = L.cy + L.fy * p.y() / p.z();
        info.u0 = std::min(info.u0, uu);
        info.u1 = std::max(info.u1, uu);
        info.v0 = std::min(info.v0, vv);
        info.v1 = std::max(info.v1, vv);
        info.dmin = std::min(info.dmin, p.z());
      }
    }
  }
  info.depth = (ci * top).z();
  const Grip g = arm_grip(m, q);
  info.tilt_deg = std::acos(std::clamp(-g.app.normalized().z(), -1.0, 1.0)) * kDeg;
  info.tip_h = g.tip.z() - (c.floor_z + u.puck_h);
  info.margin_px = std::min({info.u0, L.width - info.u1, info.v0, L.height - info.v1, std::max(L.finger_u - info.u1, L.finger_v - info.v1)});
  return front && info.dmin >= L.min_depth - 1e-9 && info.depth <= L.max_depth + 1e-9 && info.margin_px >= L.min_margin_px - 1e-9;
}

namespace
{
// 한 자세에서 열린 손가락 · 손목 쪽 점의 여유 [m] (park_look_path_clear 의 한 점)
double park_look_pose_clear(const ArmModel & m, const ParkCfg & c, const UprightCfg & u, const ParkLookCfg & L, const Q5 & q)
{
  const double dz = model_to_base_z(m);
  const Grip g = arm_grip(m, q);
  const Eigen::Vector3d app = g.app.normalized(), cl = g.close.normalized(), b = app.cross(cl).normalized();
  const double ptop = c.floor_z + u.puck_h + L.tip_rise, wall_top = c.floor_z + u.wall_h;
  const double rp = L.puck_r + L.shift + 0.003;
  const Eigen::Vector2d rd = (c.center - m.j1_xy).normalized(), td(-rd.y(), rd.x());
  double clear = 1e9;
  auto check = [&](const Eigen::Vector3d & p) {   // 모델 좌표
      const Eigen::Vector2d d = p.head<2>() - c.center;
      if (d.norm() < rp) {clear = std::min(clear, p.z() - ptop); return;}
      if (std::fabs(d.dot(rd)) <= u.cup_out_r + 1e-4 && std::fabs(d.dot(td)) <= u.cup_out_t + 1e-4) {clear = std::min(clear, p.z() - wall_top); return;}
      if (p.x() < L.rack_front_x && std::fabs(p.y()) < L.half_y) {clear = std::min(clear, p.z() + dz - L.rack_puck_top);}
    };
  for (int side : {-1, 1}) {
    for (double xc : {u.finger_inner0 + u.grip_open, 0.5 * (u.finger_inner0 + u.finger_outer0) + u.grip_open, u.finger_outer0 + u.grip_open}) {
      for (int k = -2; k <= 2; ++k) {
        for (double za = 0.0; za <= 0.030 + 1e-9; za += 0.010) {check(g.tip + side * xc * cl + (u.finger_half_w * k / 2.0) * b - za * app);}
      }
    }
  }
  // 팔꿈치 · 손목 쪽 (look_arm_points 와 같은 점, 링크 반 굵기 link_r): 적재함 칸 퍽 윗면 위 · 광각 기둥 상자 밖 (관찰 자세는 팔이 적재함 위로 젖혀짐)
  const auto F = arm_frames(m, q);
  const Eigen::Vector3d p2 = F[2].block<3, 1>(0, 3), p3 = F[3].block<3, 1>(0, 3), p4 = F[4].block<3, 1>(0, 3);
  for (const Eigen::Vector3d & p : {p2, Eigen::Vector3d(0.5 * (p2 + p3)), p3, Eigen::Vector3d(0.5 * (p3 + p4)), p4, Eigen::Vector3d(0.5 * (p4 + g.tip))}) {
    if (p.x() < L.rack_front_x && std::fabs(p.y()) < L.half_y) {clear = std::min(clear, p.z() + dz - L.link_r - L.rack_puck_top);}
    const double ox = std::max({L.mast_box[0] - p.x(), p.x() - L.mast_box[1], 0.0}), oy = std::max({L.mast_box[2] - p.y(), p.y() - L.mast_box[3], 0.0});
    if (p.z() + dz < L.mast_box[4]) {clear = std::min(clear, std::hypot(ox, oy) - L.link_r);}
  }
  return clear;
}

bool park_look_body_ok(const ArmModel & m, const Eigen::Isometry3d & T5c, const ParkLookCfg & L, const Q5 & q)
{
  for (const auto & p : look_arm_points(m, T5c, q)) {   // base_link
    const double ox = std::max({L.mast_box[0] - p.x(), p.x() - L.mast_box[1], 0.0}), oy = std::max({L.mast_box[2] - p.y(), p.y() - L.mast_box[3], 0.0});
    if (p.z() < L.mast_box[4] && std::hypot(ox, oy) < L.mast_pad) {return false;}   // 광각 기둥 (look_at mast_box · clear_pad 0.03)
    if (std::fabs(p.y()) >= L.half_y) {continue;}
    if (p.x() < L.rack_x && p.z() < L.rack_z) {return false;}
    if (p.x() < L.body_x && p.z() < L.body_z) {return false;}
  }
  return true;
}
}  // namespace

double park_look_path_clear(const ArmModel & m, const ParkCfg & c, const UprightCfg & u, const ParkLookCfg & L, const Q5 & qa, const Q5 & qb)
{
  double dmax = 0.0;
  for (int j = 0; j < 5; ++j) {dmax = std::max(dmax, std::fabs(qb[j] - qa[j]));}
  const int n = std::max(1, static_cast<int>(std::ceil(dmax * kDeg / 2.0)));
  double clear = 1e9;
  for (int i = 0; i <= n; ++i) {
    Q5 q;
    for (int j = 0; j < 5; ++j) {q[j] = qa[j] + (qb[j] - qa[j]) * i / n;}
    clear = std::min(clear, park_look_pose_clear(m, c, u, L, q));
  }
  return clear;
}

bool park_look_pose(const ArmModel & m, const Eigen::Isometry3d & T5c, const ParkCfg & c, const UprightCfg & u, const ParkLookCfg & L,
  const Q5 & q_up, const Q5 & q_down, Q5 & q_obs, ParkLookInfo & info, std::string & why, const Q5 * hint)
{
  info = ParkLookInfo{};
  // 0) pk_up 에서 같은 손 자세로 곧게 올림 (안 되면 안 올림)
  Q5 q_from = q_up;
  for (double h = L.lift; h > 0.005 - 1e-9; h -= 0.010) {   // 손이 닿는 만큼 (40 · 30 · 20 · 10 mm)
    const Grip g = arm_grip(m, q_up);
    Q5 ql = q_up;
    if (nearest_ik(m, g.tip + h * Eigen::Vector3d::UnitZ(), g.app.normalized(), g.close.normalized(), true, ql)) {
      info.lifted = true;
      info.q_lift = ql;
      q_from = ql;
      break;
    }
  }
  // 관절 범위 (틱 범위 안쪽 1 틱)
  double lo[5], hi[5];
  {
    Ticks a{}, b{};
    for (int j = 0; j < 5; ++j) {a[j] = static_cast<int>(std::ceil(m.tick_range[j][0])) + 1; b[j] = static_cast<int>(std::floor(m.tick_range[j][1])) - 1;}
    const Q5 qa = ticks_to_q(m, a), qb = ticks_to_q(m, b);
    for (int j = 0; j < 5; ++j) {lo[j] = std::max(m.lo[j], std::min(qa[j], qb[j])); hi[j] = std::min(m.hi[j], std::max(qa[j], qb[j]));}
  }
  struct Cand {Q5 q; double margin;};
  const double ptop = c.floor_z + u.puck_h;
  const Eigen::Vector3d top_b(c.center.x(), c.center.y(), ptop + model_to_base_z(m));
  auto eval = [&](const Q5 & q, double & margin) {
      ++info.evals;
      for (int j = 0; j < 5; ++j) {if (q[j] < lo[j] || q[j] > hi[j]) {return false;}}
      const Grip g = arm_grip(m, q);
      if (g.tip.z() < ptop + L.tip_rise + L.finger_clear) {return false;}
      {   // 빠른 거름: 퍽 윗면 가운데 깊이 · 화면 안
        const Eigen::Vector3d p = look_camera_pose(m, T5c, q).inverse() * top_b;
        if (p.z() < L.min_depth || p.z() > L.max_depth) {return false;}
        const double uu = L.cx + L.fx * p.x() / p.z(), vv = L.cy + L.fy * p.y() / p.z();
        if (uu < L.min_margin_px || uu > L.width - L.min_margin_px || vv < L.min_margin_px || vv > L.height - L.min_margin_px) {return false;}
      }
      if (!park_look_body_ok(m, T5c, L, q)) {return false;}
      ParkLookInfo vi;
      if (!park_look_view(m, T5c, c, u, L, q, vi)) {return false;}
      if (!ticks_in_range(m, q_to_ticks(m, q)) || !ticks_in_range(m, q_to_ticks(m, encoder_goal_q(m, q, 0.0)))) {return false;}
      if (park_look_pose_clear(m, c, u, L, q) < L.finger_clear) {return false;}
      margin = std::min(vi.margin_px, L.min_margin_px + (vi.dmin - L.min_depth) * 1000.0 * L.depth_px_per_mm);   // 화면 여유 · 깊이 여유 둘 다
      return true;
    };
  std::vector<Cand> fine;
  {
    double mg = 0.0;
    if (hint && eval(*hint, mg)) {fine.push_back({*hint, mg});}
  }
  auto search = [&]() {
      // 1) 거친 격자 (3°): J1 · J5 는 pk_up 둘레, J2 · J3 · J4 는 넓게 (계산으로 맞는 자세 J2 −1.65…−1.1 · J3 −0.45…−0.15 · J4 0.5…0.82)
      const double s3 = 3.0 / kDeg;
      std::vector<Cand> coarse;
      for (double d1 = -0.3; d1 <= 0.3 + 1e-9; d1 += 0.1) {
        for (double d5 = -0.5; d5 <= 0.75 + 1e-9; d5 += 0.25) {
          for (double j2 = -1.8; j2 <= -0.9 + 1e-9; j2 += s3) {
            for (double j3 = -0.7; j3 <= 0.0 + 1e-9; j3 += s3) {
              for (double j4 = 0.4; j4 <= hi[3] + 1e-9; j4 += s3) {
                const Q5 q{q_up[0] + d1, j2, j3, j4, q_up[4] + d5};
                double mg = 0.0;
                if (eval(q, mg)) {coarse.push_back({q, mg});}
              }
            }
          }
        }
      }
      if (coarse.empty()) {why = "받침을 깊이 " + std::to_string(static_cast<int>(std::lround(L.min_depth * 1000.0))) + " mm 넘게 · 화면 안에서 보는 자세 없음"; return;}
      std::sort(coarse.begin(), coarse.end(), [](const Cand & a, const Cand & b) {return a.margin > b.margin;});
      // 2) 위 8 개 둘레 고운 격자 (0.75° · J1 0.025 · J5 0.0625)
      for (size_t k = 0; k < coarse.size() && k < 8; ++k) {
        Cand best = coarse[k];
        const Q5 q0 = coarse[k].q;
        const double f = 0.75 / kDeg;
        for (int a = -2; a <= 2; ++a) {
          for (int b = -2; b <= 2; ++b) {
            for (int cc = -2; cc <= 2; ++cc) {
              for (int d = -1; d <= 1; ++d) {
                for (int e = -1; e <= 1; ++e) {
                  const Q5 q{q0[0] + 0.025 * d, q0[1] + f * a, q0[2] + f * b, q0[3] + f * cc, q0[4] + 0.0625 * e};
                  double mg = 0.0;
                  if (eval(q, mg) && mg > best.margin) {best = {q, mg};}
                }
              }
            }
          }
        }
        fine.push_back(best);
      }
      std::sort(fine.begin(), fine.end(), [](const Cand & a, const Cand & b) {return a.margin > b.margin;});
    };
  // 3) 지나는 길: (pk_up →) 올림 → 관찰 → park_re_above 손가락 여유 ≥ path_clear (끝점 = 이미 쓰는 pk_up · park_re_above 가 더 가까우면 그 끝점 − 0.5 mm 까지)
  std::string last;
  char buf[240];
  const double e_up = park_look_pose_clear(m, c, u, L, q_up), e_down = park_look_pose_clear(m, c, u, L, q_down);
  const double need_up = std::min(L.path_clear, e_up - 0.0005), need_down = std::min(L.path_clear, e_down - 0.0005);
  auto pick = [&]() {
      for (const auto & cd : fine) {
        const double c_up = info.lifted ? park_look_path_clear(m, c, u, L, q_up, q_from) : 1e9;
        const double c_obs = park_look_path_clear(m, c, u, L, q_from, cd.q);
        const double c_down = park_look_path_clear(m, c, u, L, cd.q, q_down);
        if (c_up < need_up - 1e-9 || c_obs < (info.lifted ? L.path_clear : need_up) - 1e-9 || c_down < need_down - 1e-9) {
          std::snprintf(buf, sizeof(buf), "지나는 길 손가락 여유 올림 %.1f · 관찰로 %.1f · 다시 쥐기로 %.1f mm (필요 %.1f · %.1f · %.1f)", c_up * 1000.0,
            c_obs * 1000.0, c_down * 1000.0, need_up * 1000.0, (info.lifted ? L.path_clear : need_up) * 1000.0, need_down * 1000.0);
          last = buf;
          continue;
        }
        q_obs = cd.q;
        const int ev = info.evals;
        const bool lf = info.lifted;
        const Q5 ql = info.q_lift;
        park_look_view(m, T5c, c, u, L, q_obs, info);
        info.evals = ev;
        info.lifted = lf;
        info.q_lift = ql;
        info.clear_up = c_up;
        info.clear_obs = c_obs;
        info.clear_down = c_down;
        why.clear();
        return true;
      }
      return false;
    };
  if (!fine.empty() && pick()) {info.from_hint = true; return true;}   // 기억한 자세 그대로
  fine.clear();
  search();
  if (fine.empty()) {return false;}   // why = search 가 씀
  if (pick()) {return true;}
  why = "관찰 자세 " + std::to_string(fine.size()) + " 개 모두 " + last;
  return false;
}

bool park_lift_look(const ArmModel & m, const Eigen::Isometry3d & T5c, const ParkCfg & c, const UprightCfg & u, const ParkLookCfg & L,
  const Q5 & q_up, const Q5 & q_down, Q5 & q_obs, ParkLookInfo & info, std::string & why)
{
  info = ParkLookInfo{};
  const Grip g = arm_grip(m, q_up);
  bool lifted = false;
  Q5 ql = q_up;
  double h_used = 0.0;
  for (double h = L.lift; h > 0.005 - 1e-9; h -= 0.010) {   // park_look_pose 0 단계와 같음 (40 · 30 · 20 · 10 mm)
    ql = q_up;
    if (nearest_ik(m, g.tip + h * Eigen::Vector3d::UnitZ(), g.app.normalized(), g.close.normalized(), true, ql)) {lifted = true; h_used = h; break;}
  }
  if (!lifted) {why = "곧게 올릴 IK 안 됨"; return false;}
  if (!ticks_in_range(m, q_to_ticks(m, ql)) || !ticks_in_range(m, q_to_ticks(m, encoder_goal_q(m, ql, 0.0)))) {why = "올린 점 틱 범위 밖"; return false;}
  // 길: park_look_pose 와 같은 기준 (끝점 = 이미 쓰는 pk_up · park_re_above 가 더 가까우면 그 끝점 − 0.5 mm 까지)
  const double e_up = park_look_path_clear(m, c, u, L, q_up, q_up), e_down = park_look_path_clear(m, c, u, L, q_down, q_down);
  const double need_up = std::min(L.path_clear, e_up - 0.0005), need_down = std::min(L.path_clear, e_down - 0.0005);
  const double c_up = park_look_path_clear(m, c, u, L, q_up, ql), c_down = park_look_path_clear(m, c, u, L, ql, q_down);
  char buf[200];
  if (c_up < need_up - 1e-9 || c_down < need_down - 1e-9) {
    std::snprintf(buf, sizeof(buf), "올린 점 길 손가락 여유 올림 %.1f · 다시 쥐기로 %.1f mm (필요 %.1f · %.1f)", c_up * 1000.0, c_down * 1000.0,
      need_up * 1000.0, need_down * 1000.0);
    why = buf;
    return false;
  }
  park_look_view(m, T5c, c, u, L, ql, info);   // 기록용 (깊이 · 화면 범위). 고르기는 아래 가운데 화면 안만
  {
    const Eigen::Vector3d top(c.center.x(), c.center.y(), c.floor_z + u.puck_h + model_to_base_z(m));
    const Eigen::Vector3d p = look_camera_pose(m, T5c, ql).inverse() * top;
    const double uu = L.cx + L.fx * p.x() / std::max(p.z(), 1e-6), vv = L.cy + L.fy * p.y() / std::max(p.z(), 1e-6);
    if (!(p.z() > 0.02) || uu < 0.0 || uu > L.width || vv < 0.0 || vv > L.height) {
      std::snprintf(buf, sizeof(buf), "올린 점에서 받침 가운데가 화면 밖 (u %.0f v %.0f · 깊이 %.3f)", uu, vv, p.z());
      why = buf;
      info = ParkLookInfo{};
      return false;
    }
  }
  q_obs = ql;
  info.lifted = false;   // 관찰 점 = 올린 점 (park_look_up 을 따로 안 끼움)
  info.at_lift = true;
  info.q_lift = ql;
  info.clear_up = c_up;
  info.lift_m = h_used;
  info.clear_down = c_down;
  why.clear();
  return true;
}

bool park_insert_look(std::vector<ParkNode> & pn, bool square, bool enabled, const Q5 & q_obs, const ParkLookInfo & li, const std::string & observe_name)
{
  if (square || !enabled) {return false;}
  const auto it = std::find_if(pn.begin(), pn.end(), [](const ParkNode & n) {return n.name == "pk_up";});
  if (it == pn.end()) {return false;}
  std::vector<ParkNode> add;
  if (li.lifted && !li.at_lift) {add.push_back({"park_look_up", li.q_lift, 0.0, true, ""});}
  add.push_back({observe_name, q_obs, 0.0, true, ""});
  pn.insert(it + 1, add.begin(), add.end());
  return true;
}

bool park_regrasp_at(const ArmModel & m, const ParkCfg & c, const UprightCfg & u, const Eigen::Vector2d & xy, const Eigen::Vector3d & close,
  const Q5 & seed, Q5 & q_above, Q5 & q_down, std::string & why)
{
  const Eigen::Vector3d Z = Eigen::Vector3d::UnitZ();
  Eigen::Vector3d cl(close.x(), close.y(), 0.0);
  if (!xy.allFinite() || cl.norm() < 1e-6) {why = "입력 이상"; return false;}
  cl.normalize();
  const double ptop = c.floor_z + u.puck_h;
  q_above = seed;
  if (!nearest_ik(m, Eigen::Vector3d(xy.x(), xy.y(), ptop + 0.0075), -Z, cl, false, q_above)) {why = "park_re_above IK 안 됨"; return false;}
  q_down = q_above;
  if (!nearest_ik(m, Eigen::Vector3d(xy.x(), xy.y(), ptop - c.regrasp_depth), -Z, cl, false, q_down)) {why = "park_re_down IK 안 됨"; return false;}
  why.clear();
  return true;
}

bool park_near(const Eigen::Vector2d & j1, const Eigen::Vector2d & p, double max_r)
{
  return std::isfinite(max_r) && (p - j1).norm() <= max_r + 1e-9;
}

std::vector<Eigen::Vector2d> park_relocate_spots(
  const Eigen::Vector2d & j1, const Eigen::Vector2d & puck, double min_x, double max_abs_y, double near_r)
{
  std::vector<Eigen::Vector2d> out;
  const Eigen::Vector2d d = puck - j1;
  if (d.norm() < 1e-6) {return out;}
  const Eigen::Vector2d dir = d.normalized(), side(-dir.y(), dir.x());
  // x = min_x + 2 mm 가 되는 반지름 (방향이 거의 옆이면 J1 에서 그만큼)
  const double r_near = dir.x() > 0.1 ? (min_x + 0.002 - j1.x()) / dir.x() : min_x + 0.002 - j1.x();
  for (double r = r_near; r <= near_r + 1e-9; r += 0.01) {
    for (double lat : {0.0, 0.04, -0.04, 0.08, -0.08}) {
      const Eigen::Vector2d s = j1 + dir * r + side * lat;
      if (s.x() < min_x || std::fabs(s.y()) > max_abs_y || !park_near(j1, s, near_r)) {continue;}
      out.push_back(s);
    }
  }
  return out;
}

double park_relocate_axis_deg(const Eigen::Vector2d & j1, const Eigen::Vector2d & spot)
{
  const Eigen::Vector2d d = spot - j1;
  return std::atan2(d.y(), d.x()) * kDeg;
}

double park_axis_diag_deg(double axis_deg)
{
  if (!std::isfinite(axis_deg)) {return std::numeric_limits<double>::quiet_NaN();}
  double r = std::fmod(axis_deg, 90.0);
  if (r < 0.0) {r += 90.0;}
  return std::min(r, 90.0 - r);
}

bool park_diag_relocate(double axis_deg, double max_dev_deg, bool relocate, bool already_relocated)
{
  if (!relocate || already_relocated || !(max_dev_deg > 0.0)) {return false;}
  const double dev = park_axis_diag_deg(axis_deg);
  return std::isfinite(dev) && dev > max_dev_deg;
}

}  // namespace manipulation
