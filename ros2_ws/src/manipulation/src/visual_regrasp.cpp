// visual_regrasp.cpp — 설명은 visual_regrasp.hpp
#include "manipulation/visual_regrasp.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>

namespace manipulation
{
RegraspSolution solve_visual_regrasp(const ArmModel & model, const Eigen::Vector2d & xy,
  double original_top, double yaw, const std::vector<double> & heights,
  const std::vector<std::pair<double, std::string>> & candidates, const std::vector<Q5> & seeds)
{
  RegraspSolution out;
  if (!xy.allFinite() || !std::isfinite(original_top) || heights.size() != 3) {out.why = "입력 없음"; return out;}
  for (const auto & [close, grip] : candidates) {
    out.poses = pick_poses(model, xy.x(), xy.y(), original_top, yaw, heights, out.why, close, 0.0, 0.0, 0.0, seeds);
    if (out.poses.size() == 3) {out.grip = grip; out.close = close; return out;}
  }
  out.poses.clear();
  if (out.why.empty()) {out.why = "후보 없음";}
  return out;
}

std::optional<RegraspObservation> select_regrasp_observation(
  const std::vector<RegraspObservation> & observations, const Eigen::Vector3d & expected,
  const std::string & id, double after, double now, double max_age, double id_radius, std::string * why)
{
  auto say = [&](const std::string & s) {if (why) {*why = s;}};
  say("");
  constexpr double kPosRadius = 0.025;   // ID 없이 위치로 맞출 때 (PICK neighbors_near 와 같음, 선 퍽 둘은 30 mm 넘게 떨어짐)
  if (!expected.allFinite() || !std::isfinite(now) || !std::isfinite(after) || !std::isfinite(max_age) || max_age <= 0.0 ||
    !std::isfinite(id_radius) || id_radius <= 0.0)
  {
    say("입력 이상");
    return std::nullopt;
  }
  std::vector<const RegraspObservation *> by_id, by_pos;
  bool fresh = false;
  for (const auto & p : observations) {
    if (!p.center.allFinite() || !std::isfinite(p.stamp) || p.stamp <= after || p.stamp > now || now - p.stamp > max_age) {continue;}
    fresh = true;
    if (!p.upright || !p.exact_tf || std::fabs(p.center.z() - expected.z()) > 0.04) {continue;}
    const double d = (p.center - expected).head<2>().norm();
    if (!id.empty() && p.id == id && d < id_radius) {by_id.push_back(&p);}
    else if ((id.empty() || p.id.empty()) && d < kPosRadius) {by_pos.push_back(&p);}   // 다른 ID 로 읽힌 퍽은 안 씀
  }
  const auto & use = !by_id.empty() ? by_id : by_pos;
  if (use.size() > 1) {say("같은 자리 검출 " + std::to_string(use.size()) + " 개 (모호)"); return std::nullopt;}
  if (use.empty()) {say(fresh ? "새 장에 놓은 퍽 없음" : "새 장 없음"); return std::nullopt;}
  return *use.front();
}

std::optional<RegraspObservation> select_lying_observation(
  const std::vector<RegraspObservation> & observations, const Eigen::Vector3d & expected, const std::string & id,
  double after, double now, double max_age, double id_radius, double pos_radius, const std::vector<Eigen::Vector2d> & avoid,
  std::string * why)
{
  auto say = [&](const std::string & s) {if (why) {*why = s;}};
  say("");
  constexpr double kNeighborRadius = 0.025;   // 계획 때 본 이웃 자리 (선 퍽 둘은 30 mm 넘게 떨어짐 — neighbors_near 와 같음)
  if (!expected.allFinite() || !std::isfinite(now) || !std::isfinite(after) || !std::isfinite(max_age) || max_age <= 0.0 ||
    !std::isfinite(id_radius) || id_radius <= 0.0 || !std::isfinite(pos_radius) || pos_radius <= 0.0)
  {
    say("입력 이상");
    return std::nullopt;
  }
  std::vector<const RegraspObservation *> by_id, by_pos;
  bool fresh = false;
  int near_nb = 0;
  for (const auto & p : observations) {
    if (!p.center.allFinite() || !std::isfinite(p.stamp) || p.stamp <= after || p.stamp > now || now - p.stamp > max_age) {continue;}
    fresh = true;
    if (!p.lying || !p.exact_tf || !std::isfinite(p.axis_rad) || std::fabs(p.center.z() - expected.z()) > 0.04) {continue;}
    const Eigen::Vector2d c = p.center.head<2>();
    if (std::any_of(avoid.begin(), avoid.end(), [&](const Eigen::Vector2d & a) {return a.allFinite() && (c - a).norm() < kNeighborRadius;})) {
      ++near_nb;
      continue;
    }
    const double d = (c - expected.head<2>()).norm();
    if (!id.empty() && p.id == id && d < id_radius) {by_id.push_back(&p);}
    else if ((id.empty() || p.id.empty()) && d < pos_radius) {by_pos.push_back(&p);}   // 다른 ID 로 읽힌 퍽은 안 씀
  }
  const auto & use = !by_id.empty() ? by_id : by_pos;
  if (use.size() > 1) {say("같은 자리 누운 퍽 " + std::to_string(use.size()) + " 개 (모호)"); return std::nullopt;}
  if (use.empty()) {
    say(!fresh ? "새 장 없음" : (near_nb > 0 ? "놓은 자리 근처 누운 퍽 없음 (이웃 자리 검출 " + std::to_string(near_nb) + " 개는 뺌)" :
      "놓은 자리 근처 누운 퍽 없음"));
    return std::nullopt;
  }
  return *use.front();
}

RegraspSplit split_regrasp_observation(const ArmPath & path, std::vector<ArmAction> plan, bool enabled, double timeout,
  const char * node, const char * wait_pose)
{
  RegraspSplit out{std::move(plan), {}};
  if (!enabled || !node || !wait_pose) {return out;}
  const std::string nm(node);
  const auto n = std::find_if(path.nodes.begin(), path.nodes.end(), [&](const auto & p) {return p.name == nm;});
  const auto a = std::find_if(out.plan.begin(), out.plan.end(), [&](const auto & p) {return !p.is_gripper && p.pose == nm;});
  if (n == path.nodes.end() || a == out.plan.end() || !n->stop || n + 1 == path.nodes.end()) {return out;}
  out.tail = {"regrasp_visual", {n, path.nodes.end()}, path.ends_home};
  ArmAction wait{"reobserve", false, wait_pose, 0.0};
  wait.duration_s = std::isfinite(timeout) && timeout > 0.0 ? timeout : 1.0;
  out.plan.insert(a + 1, wait);
  return out;
}

double regrasp_align_turn_deg(double close_deg, double yaw_deg)
{
  if (!std::isfinite(close_deg) || !std::isfinite(yaw_deg)) {return std::nan("");}
  return std::remainder(yaw_deg - close_deg, 90.0);
}

Eigen::Vector2d wrist_rot_offset(const ArmModel & m, double x, double y, const Q5 & q, const Eigen::Vector2d & rot_mm,
  const Eigen::Vector2d & robot_mm, double * rot_deg)
{
  Eigen::Vector2d d = Eigen::Vector2d::Zero();
  const Eigen::Vector3d c = arm_grip(m, q).close;
  const double az = std::atan2(y - m.j1_xy.y(), x - m.j1_xy.x());
  const double a = std::remainder(std::atan2(c.y(), c.x()) - az - M_PI / 2.0, 2.0 * M_PI);   // 기본 = 닫힘축 +접선
  if (rot_deg) {*rot_deg = a * 180.0 / M_PI;}
  if (std::fabs(a * 180.0 / M_PI) >= 45.0) {d = robot_mm / 1000.0;}
  if (rot_mm.x() == 0.0 && rot_mm.y() == 0.0) {return d;}
  // 90° 돌렸을 때 보정 r (팔 방향 기준, 기준 돌림 = 실물에서 본 −90°) = (R(−90°) − I) h → h, 이 돌림의 보정 = (R(a) − I) h
  auto rot2 = [](double t) {Eigen::Matrix2d R; R << std::cos(t), -std::sin(t), std::sin(t), std::cos(t); return R;};
  const Eigen::Matrix2d I = Eigen::Matrix2d::Identity();
  const Eigen::Vector2d h = (rot2(-M_PI / 2.0) - I).inverse() * rot_mm;
  d += rot2(az) * ((rot2(a) - I) * h) / 1000.0;
  return d;
}

Eigen::Vector2d wrist_lever_offset(const ArmModel & m, double x, double y, const Q5 & q, const Q5 & q_ref, const Eigen::Vector2d & hand_mm,
  double * turn_deg)
{
  const double az = std::atan2(y - m.j1_xy.y(), x - m.j1_xy.x());
  auto rot_of = [&](const Q5 & qq) {   // 닫힘축이 +접선에서 돈 각 (wrist_rot_offset 과 같은 정의)
      const Eigen::Vector3d c = arm_grip(m, qq).close;
      return std::remainder(std::atan2(c.y(), c.x()) - az - M_PI / 2.0, 2.0 * M_PI);
    };
  const double a = rot_of(q), a0 = rot_of(q_ref);
  if (turn_deg) {*turn_deg = std::remainder(a - a0, 2.0 * M_PI) * 180.0 / M_PI;}
  if (q == q_ref || !hand_mm.allFinite()) {return Eigen::Vector2d::Zero();}
  auto rot2 = [](double t) {Eigen::Matrix2d R; R << std::cos(t), -std::sin(t), std::sin(t), std::cos(t); return R;};
  return -(rot2(az) * ((rot2(a) - rot2(a0)) * hand_mm)) / 1000.0;
}

bool retarget_regrasp_tail(const ArmModel & model, const std::vector<PickPose> & poses, ArmPath & tail)
{
  if (poses.size() != 3) {return false;}
  ArmPath t = tail;
  int changed = 0;
  for (auto & n : t.nodes) {
    int k = -1;
    if (n.name == "re_above" || n.name == "re_lift") {k = 0;}
    if (n.name == "re_down" || n.name == "re_up") {k = 1;}
    if (n.name == "re_grasp") {k = 2;}
    if (k < 0) {continue;}
    n.q = ticks_to_command(model, poses[k].enc);
    ++changed;
  }
  if (changed != 5) {return false;}
  tail = std::move(t);
  return true;
}

Eigen::Vector2d cyl_hand_shift(const ArmModel & m, const Q5 & q, const Eigen::Vector2d & hand_mm)
{
  if (!hand_mm.allFinite()) {return Eigen::Vector2d::Zero();}
  return tcp_shift_base(m, q, Eigen::Vector3d(hand_mm.x(), hand_mm.y(), 0.0) / 1000.0).head<2>();
}

bool same_hand_front(const ArmModel & m, const Q5 & a, const Q5 & b, double tol_deg)
{
  const Eigen::Vector2d ha = tcp_shift_base(m, a, Eigen::Vector3d::UnitX()).head<2>();
  const Eigen::Vector2d hb = tcp_shift_base(m, b, Eigen::Vector3d::UnitX()).head<2>();
  if (ha.norm() < 1e-6 || hb.norm() < 1e-6) {return false;}
  return ha.normalized().dot(hb.normalized()) >= std::cos(tol_deg * M_PI / 180.0);
}

RegraspPlan decide_visual_regrasp(const std::vector<RegraspObservation> & observations, const RegraspTarget & target,
  double now, double deadline, const ArmModel & model, const ArmPath & tail)
{
  RegraspPlan out;
  std::string why;
  const auto found = select_regrasp_observation(observations, target.expected, target.id, target.after, now,
    target.max_age, target.id_radius, &why);
  if (!found) {
    out.why = "새 관측 없음 (" + why + ")";
    out.decision = std::isfinite(deadline) && now < deadline ? RegraspDecision::WAIT : RegraspDecision::FALLBACK;
    return out;
  }
  out.found = true;
  out.decision = RegraspDecision::FALLBACK;
  const Eigen::Vector2d xy = found->center.head<2>() + (target.offset.allFinite() ? target.offset : Eigen::Vector2d::Zero());
  const double shift = (xy - target.expected.head<2>()).norm();
  if (std::isfinite(target.max_shift_m) && target.max_shift_m > 0.0 && shift > target.max_shift_m) {
    char buf[120];
    std::snprintf(buf, sizeof(buf), "잰 중심이 놓은 자리에서 %.1f mm (한계 %.0f mm)", shift * 1000.0, target.max_shift_m * 1000.0);
    out.why = buf;
    return out;
  }
  // 원래 닫힘 방향 먼저, 안 되면 같은 퍽을 쥐는 다른 방향 (네모 ±90°, 원기둥 ±45° · 90°) — 놓은 퍽이 밀려 선 자리에서 손목이 안 닿을 때
  //   align_yaw 이고 잰 네모 면 방향이 있으면 그 면에 맞춘 방향 (과 ±90°) 을 맨 앞에 (안 되면 원래 방향 후보 그대로)
  std::vector<std::pair<double, std::string>> cands;
  double want = std::nan("");   // 면에 맞춘 닫힘 [°]
  if (target.align_yaw) {
    const double turn = found->yaw_known ? regrasp_align_turn_deg(target.close_deg, found->yaw_rad * 180.0 / M_PI) : std::nan("");
    const double lim = std::isfinite(target.align_max_deg) ? target.align_max_deg : 45.0;
    if (!found->yaw_known) {
      out.align_note = "퍽 면 방향 없음 (원기둥 · 못 잼)";
    } else if (!std::isfinite(turn)) {
      out.align_note = "면 방향 값 이상";
    } else {
      out.puck_yaw_deg = found->yaw_rad * 180.0 / M_PI;
      out.turn_deg = turn;
      if (std::fabs(turn) > lim + 1e-9) {
        char buf[96];
        std::snprintf(buf, sizeof(buf), "면까지 %.0f° (한계 %.0f°)", turn, lim);
        out.align_note = buf;
      } else {
        want = target.close_deg + turn;
        for (double t : {0.0, 90.0, -90.0}) {cands.push_back({want + t, "open"});}
      }
    }
  }
  cands.push_back({target.close_deg, "open"});
  const std::vector<double> turns = target.square ? std::vector<double>{90.0, -90.0} : std::vector<double>{45.0, -45.0, 90.0};
  for (double t : turns) {cands.push_back({target.close_deg + t, "open"});}
  std::vector<double> used_h = target.heights;
  Eigen::Vector2d xy_used = xy;   // 지금 sol 을 푼 자리 (돌림 보정을 쓰면 옮긴 자리)
  auto sol = solve_visual_regrasp(model, xy, target.top, target.close_deg, target.heights, cands, {target.seed});
  if (sol.poses.size() != 3 && target.heights.size() == 3 && target.heights[0] > target.heights[1] + 10.0) {
    // 놓은 퍽이 바깥으로 밀려 서면 수직 손의 위 점 (윗면 + 40 mm) 이 먼저 안 닿음 → 위 점만 낮춰 (윗면 + 20 mm) 다시
    const std::vector<double> low{std::max(20.0, target.heights[1] + 10.0), target.heights[1], target.heights[2]};
    auto s2 = solve_visual_regrasp(model, xy, target.top, target.close_deg, low, cands, {target.seed});
    if (s2.poses.size() == 3) {
      sol = std::move(s2);
      out.low_above = true;
      used_h = low;
    }
  }
  if (sol.poses.size() != 3) {out.why = "새 중심 수직 IK 안 됨 (" + sol.why + ")"; return out;}
  out.aligned = std::isfinite(want) && std::fabs(std::remainder(sol.close - want, 90.0)) < 1e-6;
  if (std::isfinite(want) && !out.aligned) {out.align_note = "면에 맞춘 방향 IK 안 됨 → 원래 방향";}
  // 실제 손목 돌림 (IK 해의 닫힘축 − 원래 다시 쥐기 자세 닫힘축, 부호 = J5 가 돈 쪽): 닫힘 94° → 184° 는 J5 −90° 일 수 있음 (손가락 방향 180° 같음)
  double turn = 0.0;
  wrist_lever_offset(model, xy.x(), xy.y(), ticks_to_q(model, sol.poses.back().ticks), target.seed, Eigen::Vector2d::Zero(), &turn);
  out.wrist_turn_deg = turn;
  if (target.align_yaw && target.rot_mm.allFinite() && target.rot_robot_mm.allFinite()) {
    // 손목 돌림 보정: 원래 다시 쥐기 자세 (seed, regrasp_offset_mm 을 맞춘 자세) 와의 차이만큼만 — 같은 자세면 0
    //   45° 넘게 돌리면 (다른 면 쌍 · ±90°) rot_hand_mm 이 있으면 손 지렛대 (wrist_lever_offset, 실측값), 아니면 wrist_rot_offset 차이
    const bool lever = target.rot_hand_mm.allFinite() && std::fabs(turn) >= 45.0;
    const Eigen::Vector2d d = lever ?
      wrist_lever_offset(model, xy.x(), xy.y(), ticks_to_q(model, sol.poses.back().ticks), target.seed, target.rot_hand_mm) :
      Eigen::Vector2d(wrist_rot_offset(model, xy.x(), xy.y(), ticks_to_q(model, sol.poses.back().ticks), target.rot_mm,
      target.rot_robot_mm) - wrist_rot_offset(model, xy.x(), xy.y(), target.seed, target.rot_mm, target.rot_robot_mm));
    if (d.norm() > 1e-4) {
      // 옮긴 점에서 같은 닫힘 방향으로 다시 풂 (PICK 과 같이: 안 되거나 손목 갈래가 바뀌면 보정 없이 처음 해)
      const Eigen::Vector2d xy2 = xy + d;
      auto s3 = solve_visual_regrasp(model, xy2, target.top, target.close_deg, used_h, {{sol.close, "open"}},
        {ticks_to_q(model, sol.poses.front().ticks), target.seed});
      double r1 = 0.0, r3 = 0.0;
      wrist_rot_offset(model, xy.x(), xy.y(), ticks_to_q(model, sol.poses.back().ticks), target.rot_mm, target.rot_robot_mm, &r1);
      if (s3.poses.size() == 3) {
        wrist_rot_offset(model, xy2.x(), xy2.y(), ticks_to_q(model, s3.poses.back().ticks), target.rot_mm, target.rot_robot_mm, &r3);
      }
      if (s3.poses.size() == 3 && std::fabs(std::remainder(r3 - r1, 360.0)) < 5.0) {
        sol = std::move(s3);
        xy_used = xy2;
        out.rot_d = d;
        out.rot_lever = lever;
      } else {
        char buf[96];
        std::snprintf(buf, sizeof(buf), "돌림 보정 (%+.1f, %+.1f) mm 자리에서 수직 IK 안 됨 → 보정 없이", d.x() * 1000.0, d.y() * 1000.0);
        out.align_note += (out.align_note.empty() ? "" : " · ") + std::string(buf);
      }
    }
  }
  // 원기둥 손 안 앞뒤 보정 (cyl_hand_mm): 깊이 점 중심으로 잰 원기둥만. 같은 닫힘 · 같은 손 앞 방향 수직 해가 있을 때만 (없으면 보정 없이, 알림)
  if (!target.square && !found->tagged && target.cyl_hand_mm.allFinite() && target.cyl_hand_mm.norm() > 1e-9) {
    const Q5 q0 = ticks_to_q(model, sol.poses.back().ticks);
    const Eigen::Vector2d d = cyl_hand_shift(model, q0, target.cyl_hand_mm);
    auto s4 = solve_visual_regrasp(model, xy_used + d, target.top, target.close_deg, used_h, {{sol.close, "open"}},
      {ticks_to_q(model, sol.poses.front().ticks), target.seed});
    if (s4.poses.size() == 3 && same_hand_front(model, q0, ticks_to_q(model, s4.poses.back().ticks), 10.0)) {
      sol = std::move(s4);
      out.cyl_d = d;
    } else {
      char buf[120];
      std::snprintf(buf, sizeof(buf), "원기둥 손 앞뒤 보정 (%+.1f, %+.1f) mm 자리에서 같은 손 방향 수직 IK 안 됨 → 보정 없이", d.x() * 1000.0, d.y() * 1000.0);
      out.align_note += (out.align_note.empty() ? "" : " · ") + std::string(buf);
    }
  }
  out.close_deg = sol.close;
  ArmPath t = tail;
  if (!retarget_regrasp_tail(model, sol.poses, t)) {out.why = "후반 경로에 다시 쥐기 점이 다섯 개가 아님"; return out;}
  out.decision = RegraspDecision::USE;
  out.tail = std::move(t);
  out.obs = *found;
  out.poses = sol.poses;
  return out;
}

bool retarget_park_tail(const ArmModel & model, const Q5 & q_above, const Q5 & q_down, double held_kg, ArmPath & tail)
{
  ArmPath t = tail;
  int changed = 0;
  for (auto & n : t.nodes) {
    const Q5 * q = nullptr;
    double kg = 0.0;
    if (n.name == "park_re_above") {q = &q_above;}
    if (n.name == "park_re_down") {q = &q_down;}
    if (n.name == "park_re_lift") {q = &q_above; kg = held_kg;}
    if (!q) {continue;}
    n.q = ticks_to_command(model, q_to_ticks(model, encoder_goal_q(model, *q, kg)));
    ++changed;
  }
  if (changed != 3) {return false;}
  tail = std::move(t);
  return true;
}

ParkRegraspPlan decide_park_regrasp(const std::vector<RegraspObservation> & observations, const ParkRegraspTarget & target,
  double now, double deadline, const ArmModel & model, const ArmPath & tail)
{
  return decide_park_regrasp(observations, {}, target, now, deadline, model, tail);
}

ParkRegraspPlan decide_park_regrasp(const std::vector<RegraspObservation> & observations,
  const std::vector<RegraspObservation> & tag_observations, const ParkRegraspTarget & target,
  double now, double deadline, const ArmModel & model, const ArmPath & tail)
{
  ParkRegraspPlan out;
  std::string why, tag_why;
  auto found = tag_observations.empty() ? std::nullopt : select_regrasp_observation(tag_observations, target.expected, target.id, target.after, now,
    target.max_age, target.id_radius, &tag_why);
  out.from_tag = found.has_value();
  const bool depth_ok = !(std::isfinite(target.tag_wait_s) && target.tag_wait_s > 0.0) || now >= target.after + target.tag_wait_s;
  if (!found && depth_ok) {
    found = select_regrasp_observation(observations, target.expected, target.id, target.after, now, target.max_age, target.id_radius, &why);
  } else if (!found) {
    why = "태그 기다림";
  }
  if (!found) {
    out.why = "새 관측 없음 (" + why + (tag_why.empty() ? std::string() : " · 태그: " + tag_why) + ")";
    out.decision = std::isfinite(deadline) && now < deadline ? RegraspDecision::WAIT : RegraspDecision::FALLBACK;
    return out;
  }
  out.found = true;
  out.obs = *found;
  out.decision = RegraspDecision::FALLBACK;
  out.xy = found->center.head<2>() - (target.bias.allFinite() ? target.bias : Eigen::Vector2d::Zero());
  out.shift_mm = (out.xy - target.expected.head<2>()).norm() * 1000.0;
  char buf[160];
  if (std::isfinite(target.max_shift_m) && target.max_shift_m > 0.0 && out.shift_mm > target.max_shift_m * 1000.0 + 1e-9) {
    std::snprintf(buf, sizeof(buf), "잰 중심이 받침 가운데에서 %.1f mm (한계 %.0f mm, park_look_max_shift_mm)", out.shift_mm, target.max_shift_m * 1000.0);
    out.why = buf;
    return out;
  }
  Q5 qa{}, qd{};
  if (!park_regrasp_at(model, target.park, target.upright, out.xy, target.close, target.seed, qa, qd, why)) {
    out.why = "잰 중심 수직 IK 안 됨 (" + why + ")";
    return out;
  }
  ArmPath t = tail;
  if (!retarget_park_tail(model, qa, qd, target.held_kg, t)) {out.why = "후반 경로에 받침 다시 쥐기 점이 세 개가 아님"; return out;}
  out.decision = RegraspDecision::USE;
  out.tail = std::move(t);
  return out;
}
}  // namespace manipulation
