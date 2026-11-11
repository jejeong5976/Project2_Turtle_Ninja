#include "manipulation/held_correction.hpp"
#include <algorithm>
#include <cmath>
#include <map>
#include <set>

namespace manipulation
{
namespace
{
double median(std::vector<double> v)
{
  std::nth_element(v.begin(), v.begin()+v.size()/2, v.end());
  return v[v.size()/2];
}
bool top_normal(Eigen::Vector3d & n, const Eigen::Vector3d & up)
{
  if (!n.allFinite() || !up.allFinite() || n.norm()<0.5 || up.norm()<0.5) {return false;}
  n.normalize();
  if (n.dot(up)<0.0) {n=-n;}   // PnP 모서리 순서에 따른 법선 부호는 윗면 방향으로 맞춤
  return n.dot(up.normalized())>=std::cos(30.0*M_PI/180.0);
}
bool depth_ok(double d) {return std::isfinite(d) && d>0.0 && d<=60.0;}
}

HeldEstimate fresh_held(const std::vector<HeldSample> & samples, const std::string & expected_id,
  double after, double now, const Eigen::Vector3d & up)
{
  HeldEstimate out;
  out.why="정지 뒤 신선한 쥔 퍽 태그 3장 없음";
  if (!std::isfinite(after) || !std::isfinite(now)) {return out;}
  std::map<std::string,std::vector<HeldSample>> groups;
  std::set<std::pair<std::string,double>> frames;
  for (auto s:samples) {
    if (s.id.empty() || (!expected_id.empty() && s.id!=expected_id) || s.frame!="wrist_link" ||
      !std::isfinite(s.stamp) || s.stamp<after || s.stamp>now || now-s.stamp>0.8 || !s.p.allFinite() ||
      !top_normal(s.n,up) || !frames.emplace(s.id,s.stamp).second) {continue;}
    groups[s.id].push_back(s);
  }
  // ID를 모를 때는 가장 많은 것을 임의로 고르지 않음. 가까운 다른 퍽을 잘못 쥔 것으로 보지 않게 함.
  if (groups.size()!=1) {out.why=groups.empty()?out.why:"쥔 퍽 ID가 둘 이상 보여 보정 안 함";return out;}
  const auto & g=groups.begin()->second;
  out.id=groups.begin()->first;out.count=static_cast<int>(g.size());
  if (g.size()<3) {return out;}
  for (int j=0;j<3;++j) {
    std::vector<double> p,n;
    for (const auto & s:g) {p.push_back(s.p[j]);n.push_back(s.n[j]);}
    out.p[j]=median(p);out.n[j]=median(n);
  }
  out.n.normalize();
  for (const auto & s:g) {
    if ((s.p-out.p).norm()>0.002 || s.n.dot(out.n)<std::cos(5.0*M_PI/180.0)) {
      out.why="손 안 태그가 2 mm/5도 넘게 흔들려 보정 안 함";return out;
    }
  }
  out.ok=true;out.why.clear();return out;
}

HeldCorrection held_correction(const HeldEstimate & cur, const HeldReference & ref,
  double depth, const Eigen::Vector3d & up, const Eigen::Matrix3d & R, double max_mm)
{
  HeldCorrection out;
  out.why=cur.ok?"기준의 쥔 깊이 없음 (새 기준 저장 필요)":cur.why;
  if (!cur.ok || !cur.p.allFinite() || !depth_ok(depth) || !depth_ok(ref.depth_mm) || !ref.p.allFinite() ||
    !R.allFinite() || !std::isfinite(max_mm) || max_mm<=0.0) {return out;}
  Eigen::Vector3d nc=cur.n,nr=ref.n;
  if (!top_normal(nc,up) || !top_normal(nr,up) || nc.dot(nr)<std::cos(20.0*M_PI/180.0)) {
    out.why="기준과 퍽 윗면 방향이 달라 보정 안 함";return out;
  }
  out.residual=(cur.p-depth/1000.0*nc)-(ref.p-ref.depth_mm/1000.0*nr);
  if (out.residual.norm()*1000.0>max_mm) {out.why="깊이 차이를 뺀 손 안 이동이 한도 밖 → 기존 경로";return out;}
  out.xy=-(R*out.residual).head<2>();
  out.ok=true;out.why.clear();return out;
}
}  // namespace manipulation
