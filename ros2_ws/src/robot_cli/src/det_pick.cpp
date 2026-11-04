#include "robot_cli/det_pick.hpp"

#include <cmath>

#include "planning/grasp.hpp"

namespace robot_cli
{
namespace
{
using planning::Quat;
using planning::Vec3;

Quat qmul(const Quat & a, const Quat & b)
{
  return {a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y, a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
    a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w, a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
}

Vec3 rotate(const Quat & q, const Vec3 & v)
{
  const Quat p = qmul(qmul(q, Quat{v.x, v.y, v.z, 0.0}), Quat{-q.x, -q.y, -q.z, q.w});
  return {p.x, p.y, p.z};
}
}  // namespace

int choose_det(const std::vector<DetView> & dets, const std::string & tag, std::string & why)
{
  std::vector<int> pucks;
  for (int i = 0; i < static_cast<int>(dets.size()); ++i) {
    if (dets[i].kind == "puck") {pucks.push_back(i);}
  }
  if (!tag.empty()) {
    for (int i : pucks) {
      if (dets[i].id == tag) {return i;}
    }
    why = "태그 " + tag + " 퍽이 손목 검출에 없음";
    return -1;
  }
  if (pucks.size() != 1) {
    why = pucks.empty() ? "손목 검출에 퍽이 없음 (LOOK 자세 · 인지 켜짐 확인)" :
      "퍽이 " + std::to_string(pucks.size()) + " 개 보임 → pick tag <ID> (4 쪽 태그 칸)";
    return -1;
  }
  return pucks[0];
}

bool det_pick(const DetView & d, const Xform & T, double grasp_offset, DetPick & out, std::string & why)
{
  if (!std::isfinite(d.x) || !std::isfinite(d.y) || !std::isfinite(d.z)) {why = "검출 위치 모름 (광각 · 깊이 없음)"; return false;}
  const Quat tq{T.qx, T.qy, T.qz, T.qw};
  const Vec3 r = rotate(tq, Vec3{d.x, d.y, d.z});
  const Vec3 c{r.x + T.tx, r.y + T.ty, r.z + T.tz};
  const bool has_q = d.qx * d.qx + d.qy * d.qy + d.qz * d.qz + d.qw * d.qw > 0.5;   // Detection.msg: 방향 모름 = 0,0,0,0
  const Quat q = has_q ? qmul(tq, Quat{d.qx, d.qy, d.qz, d.qw}) : Quat{0, 0, 0, 0};
  const planning::PuckPlanar pl = planning::planar_from_pose(has_q, q, c, d.yaw_known, d.shape == "box");
  if (!pl.upright) {why = "누운 퍽 — control_master 가 아직 안 집음"; return false;}
  const planning::GraspAxis ga = planning::choose_grasp_axis(has_q, q, c);
  const Vec3 g = planning::grasp_point(c, ga.axis, grasp_offset);
  out = DetPick{};
  out.x = g.x;
  out.y = g.y;
  out.z = g.z;
  out.cx = c.x;
  out.cy = c.y;
  out.cz = c.z;
  out.upright = pl.upright;
  out.theta_known = pl.theta_known;
  out.theta_deg = pl.theta * 180.0 / M_PI;
  if (has_q) {
    const Quat o = planning::grasp_orientation(pl, ga);
    out.qx = o.x;
    out.qy = o.y;
    out.qz = o.z;
    out.qw = o.w;
  }
  return true;
}

}  // namespace robot_cli
