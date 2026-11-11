// held_image.cpp — 쥔 퍽 영상 잰 값 ↔ 기준, 설명은 held_image.hpp
#include "manipulation/held_image.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace manipulation
{
namespace
{
double med(std::vector<double> x)
{
  if (x.empty()) {return 0.0;}
  std::sort(x.begin(), x.end());
  const std::size_t n = x.size();
  return n % 2 ? x[n / 2] : 0.5 * (x[n / 2 - 1] + x[n / 2]);
}
}  // namespace

HeldImageSample median_sample(const std::vector<HeldImageSample> & v)
{
  HeldImageSample m;
  if (v.empty()) {return m;}
  std::vector<double> st, ty, td, lx, ld, by, cf;
  for (const auto & s : v) {
    st.push_back(s.stamp); ty.push_back(s.top_y); td.push_back(s.top_deg); by.push_back(s.bottom_y); cf.push_back(s.conf);
    if (s.left_ok) {lx.push_back(s.left_x); ld.push_back(s.left_deg);}
  }
  m.stamp = *std::max_element(st.begin(), st.end());
  m.top_y = med(ty);
  m.top_deg = med(td);
  m.bottom_y = med(by);
  m.conf = med(cf);
  m.left_ok = !lx.empty();
  if (m.left_ok) {m.left_x = med(lx); m.left_deg = med(ld);}
  m.label = v.back().label;
  m.shape = v.back().shape;
  return m;
}

std::vector<HeldImageSample> samples_between(const std::deque<HeldImageSample> & buf, double from, double to)
{
  std::vector<HeldImageSample> out;
  for (const auto & s : buf) {
    if (s.stamp >= from && s.stamp <= to) {out.push_back(s);}
  }
  return out;
}

HeldImageVerdict judge_held_image(const HeldImageSample & cur, int n, const HeldImageRef & ref, const HeldImageConfig & c)
{
  HeldImageVerdict v;
  if (!ref.ok) {v.text = "기준 없음"; return v;}
  if (n <= 0) {v.text = "잰 장 없음"; return v;}
  const double pf = c.px_per_mm_front > 0.0 ? c.px_per_mm_front : 5.2;
  const double pb = c.px_per_mm_back > 0.0 ? c.px_per_mm_back : 14.7;
  v.judged = true;
  v.d_top_px = cur.top_y - ref.s.top_y;
  v.d_top_deg = cur.top_deg - ref.s.top_deg;
  v.front_mm = v.d_top_px >= 0.0 ? v.d_top_px / pf : v.d_top_px / pb;
  if (cur.left_ok && ref.s.left_ok) {
    v.d_left_px = cur.left_x - ref.s.left_x;
    v.tilt_deg = cur.left_deg - ref.s.left_deg;
  } else {
    v.tilt_from_top = true;
    v.tilt_deg = v.d_top_deg;
  }
  const double back = c.back_tol_mm > 0.0 ? c.back_tol_mm : c.tol_mm;
  const bool front_bad = v.front_mm > c.tol_mm;
  const bool back_bad = v.front_mm < -back;
  const bool tilt_bad = std::fabs(v.tilt_deg) > c.tilt_deg;
  v.bad = front_bad || back_bad || tilt_bad;
  char buf[320];
  std::snprintf(buf, sizeof(buf),
    "%s %.1f mm (위 모서리 %+.0f px, 허용 앞 %.1f · 뒤 %.1f) · 기울기 %+.1f° (%s, 허용 %.1f) · 왼쪽 %+.0f px · 위 모서리 각 %+.1f° · %d 장%s",
    v.front_mm >= 0.0 ? "앞으로 쥠" : "뒤로 쥠", std::fabs(v.front_mm), v.d_top_px, c.tol_mm, back, v.tilt_deg,
    v.tilt_from_top ? "위 모서리 — 손가락 위 왼쪽 모서리 못 잼" : "왼쪽 모서리", c.tilt_deg, v.d_left_px, v.d_top_deg, n,
    !v.bad ? "" : (front_bad ? " → 앞으로 쥠 허용 넘음" : back_bad ? " → 뒤로 쥠 허용 넘음" : " → 기울기 허용 넘음"));
  v.text = buf;
  return v;
}

bool save_held_image_refs(const std::string & path, const std::map<std::string, HeldImageRef> & refs, std::string & err)
{
  std::error_code ec;
  const auto dir = std::filesystem::path(path).parent_path();
  if (!dir.empty()) {std::filesystem::create_directories(dir, ec);}
  std::ofstream f(path);
  if (!f) {err = "기준 파일을 못 엶: " + path; return false;}
  f << "# 쥔 퍽 영상 기준 (control_master ~/save_held_image_ref, 잘 쥔 상태 lift 끝 정지): 모양 위y 위각 왼x 왼각 아래y 왼쪽잼 장수 시각\n";
  char buf[256];
  for (const auto & [shape, r] : refs) {
    if (!r.ok) {continue;}
    std::snprintf(buf, sizeof(buf), "%s %.2f %.2f %.2f %.2f %.2f %d %d %s\n", shape.c_str(), r.s.top_y, r.s.top_deg, r.s.left_x,
      r.s.left_deg, r.s.bottom_y, r.s.left_ok ? 1 : 0, r.n, r.when.empty() ? "-" : r.when.c_str());
    f << buf;
  }
  f.close();
  if (!f) {err = "기준 파일을 못 씀: " + path; return false;}
  return true;
}

std::map<std::string, HeldImageRef> load_held_image_refs(const std::string & path, std::string & err)
{
  std::map<std::string, HeldImageRef> out;
  std::ifstream f(path);
  if (!f) {err = "기준 파일 없음: " + path; return out;}
  std::string line;
  while (std::getline(f, line)) {
    if (line.empty() || line[0] == '#') {continue;}
    std::istringstream in(line);
    std::string shape, when;
    HeldImageRef r;
    int left_ok = 0;
    if (!(in >> shape >> r.s.top_y >> r.s.top_deg >> r.s.left_x >> r.s.left_deg >> r.s.bottom_y >> left_ok >> r.n)) {
      err = "기준 파일 줄을 못 읽음: " + line;
      continue;
    }
    in >> when;
    r.s.left_ok = left_ok != 0;
    r.when = when;
    r.ok = std::isfinite(r.s.top_y) && std::isfinite(r.s.top_deg);
    if (r.ok) {out[shape] = r;}
  }
  return out;
}
}  // namespace manipulation
