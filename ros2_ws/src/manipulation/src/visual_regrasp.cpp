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
    if (out.poses.size() == 3) {out.grip = grip; return out;}
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
  constexpr double kPosRadius = 0.025;   // ID 없이 위치로 맞출 때 (기존 PICK neighbors_near 와 같음, 선 퍽 둘은 30 mm 넘게 떨어짐)
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

RegraspSplit split_regrasp_observation(const ArmPath & path, std::vector<ArmAction> plan, bool enabled, double timeout)
{
  RegraspSplit out{std::move(plan), {}};
  if (!enabled) {return out;}
  const auto n = std::find_if(path.nodes.begin(), path.nodes.end(), [](const auto & p) {return p.name == kRegraspObserveNode;});
  const auto a = std::find_if(out.plan.begin(), out.plan.end(), [](const auto & p) {return !p.is_gripper && p.pose == kRegraspObserveNode;});
  if (n == path.nodes.end() || a == out.plan.end() || !n->stop || n + 1 == path.nodes.end()) {return out;}
  out.tail = {"regrasp_visual", {n, path.nodes.end()}, path.ends_home};
  ArmAction wait{"reobserve", false, kRegraspObserve, 0.0};
  wait.duration_s = std::isfinite(timeout) && timeout > 0.0 ? timeout : 1.0;
  out.plan.insert(a + 1, wait);
  return out;
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
  std::vector<std::pair<double, std::string>> cands{{target.close_deg, "open"}};
  const std::vector<double> turns = target.square ? std::vector<double>{90.0, -90.0} : std::vector<double>{45.0, -45.0, 90.0};
  for (double t : turns) {cands.push_back({target.close_deg + t, "open"});}
  auto sol = solve_visual_regrasp(model, xy, target.top, target.close_deg, target.heights, cands, {target.seed});
  if (sol.poses.size() != 3 && target.heights.size() == 3 && target.heights[0] > target.heights[1] + 10.0) {
    // 놓은 퍽이 바깥으로 밀려 서면 수직 손의 위 점 (윗면 + 40 mm) 이 먼저 안 닿음 → 위 점만 낮춰 (윗면 + 20 mm) 다시
    //   (11-11 실물: (0.221, -0.063) 에 놓은 퍽이 (0.232, -0.067) 에 섬 → 40 mm 안 됨 · 20 mm 됨)
    const std::vector<double> low{std::max(20.0, target.heights[1] + 10.0), target.heights[1], target.heights[2]};
    auto s2 = solve_visual_regrasp(model, xy, target.top, target.close_deg, low, cands, {target.seed});
    if (s2.poses.size() == 3) {
      sol = std::move(s2);
      out.low_above = true;
    }
  }
  if (sol.poses.size() != 3) {out.why = "새 중심 수직 IK 안 됨 (" + sol.why + ")"; return out;}
  ArmPath t = tail;
  if (!retarget_regrasp_tail(model, sol.poses, t)) {out.why = "후반 경로에 다시 쥐기 점이 다섯 개가 아님"; return out;}
  out.decision = RegraspDecision::USE;
  out.tail = std::move(t);
  out.obs = *found;
  out.poses = sol.poses;
  return out;
}
}  // namespace manipulation
