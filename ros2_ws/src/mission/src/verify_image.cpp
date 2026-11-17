// verify_image.cpp — 재검증 판정 그림 구현, 설명은 verify_image.hpp

#include "mission/verify_image.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace mission
{

namespace
{
// 색 (BGR)
const cv::Scalar kOk(90, 205, 80), kBad(70, 70, 235), kBox(0, 175, 255), kEmpty(190, 190, 190), kIgnored(150, 150, 150);
const cv::Scalar kPanel(28, 28, 28), kText(245, 245, 245), kSub(200, 200, 200);
const cv::Scalar kGroupR(60, 60, 220), kGroupY(40, 200, 235), kGroupG(80, 190, 70);
constexpr int kFont = cv::FONT_HERSHEY_DUPLEX;

std::string short_color(const std::string & c)
{
  if (c == "red") {return "R";}
  if (c == "green") {return "G";}
  return c.empty() ? "?" : c;
}

std::string short_shape(const std::string & s)
{
  if (s == "cylinder") {return "cyl";}
  return s.empty() ? "?" : s;
}

cv::Point ipt(const cv::Point2d & p) {return {static_cast<int>(std::lround(p.x)), static_cast<int>(std::lround(p.y))};}

// map 평면 위 점 목록 → 영상 다각형 (하나라도 투영 못 하면 빈 목록)
std::vector<cv::Point> project_poly(const VerifyCamera & cam, const std::vector<std::array<double, 2>> & pts, double z)
{
  std::vector<cv::Point> out;
  for (const auto & q : pts) {
    const auto p = project_map_point(cam, q[0], q[1], z);
    if (!p) {return {};}
    out.push_back(ipt(*p));
  }
  return out;
}

std::vector<std::array<double, 2>> circle_pts(double x, double y, double r, int n = 36)
{
  std::vector<std::array<double, 2>> v;
  for (int i = 0; i < n; ++i) {
    const double a = 2.0 * M_PI * i / n;
    v.push_back({x + r * std::cos(a), y + r * std::sin(a)});
  }
  return v;
}

std::vector<std::array<double, 2>> square_pts(double x, double y, double half, double yaw)
{
  const double c = std::cos(yaw), s = std::sin(yaw);
  std::vector<std::array<double, 2>> v;
  for (const auto & [u, w] : {std::pair{1, 1}, std::pair{-1, 1}, std::pair{-1, -1}, std::pair{1, -1}}) {
    v.push_back({x + c * u * half - s * w * half, y + s * u * half + c * w * half});
  }
  return v;
}

// 점선 다각형 (닫힘)
void dashed_poly(cv::Mat & img, const std::vector<cv::Point> & poly, const cv::Scalar & c, int th)
{
  for (size_t i = 0; i < poly.size(); i += 2) {
    cv::line(img, poly[i], poly[(i + 1) % poly.size()], c, th, cv::LINE_AA);
  }
}

// 둥근 끝 글자 칩: at = 왼쪽 위. 영상 안으로 밀어 넣음, 차지한 사각 반환
cv::Rect chip(cv::Mat & img, cv::Point at, const std::string & s, const cv::Scalar & bg, double scale = 0.45,
  const cv::Scalar & fg = kText)
{
  int base = 0;
  const auto sz = cv::getTextSize(s, kFont, scale, 1, &base);
  const int pad = 5, w = sz.width + 2 * pad, h = sz.height + base + 2 * pad;
  at.x = std::clamp(at.x, 2, std::max(2, img.cols - w - 2));
  at.y = std::clamp(at.y, 2, std::max(2, img.rows - h - 2));
  const int r = h / 2;
  cv::rectangle(img, {at.x + r, at.y}, {at.x + w - r, at.y + h}, bg, cv::FILLED, cv::LINE_AA);
  cv::circle(img, {at.x + r, at.y + r}, r, bg, cv::FILLED, cv::LINE_AA);
  cv::circle(img, {at.x + w - r, at.y + r}, r, bg, cv::FILLED, cv::LINE_AA);
  cv::putText(img, s, {at.x + pad, at.y + pad + sz.height}, kFont, scale, fg, 1, cv::LINE_AA);
  return {at.x, at.y, w, h};
}

// 겹치지 않는 칩 자리: 상자 위 (왼쪽 맞춤) → 아래 → 오른쪽 → 왼쪽, 다 막히면 위 자리에서 아래로 밀기
cv::Point chip_place_rect(const cv::Rect & r, const cv::Size & sz, const std::vector<cv::Rect> & used, const cv::Size & img)
{
  const std::vector<cv::Point> cand = {
    {r.x, r.y - sz.height - 3}, {r.x, r.y + r.height + 3}, {r.x + r.width + 4, r.y}, {r.x - sz.width - 4, r.y}};
  auto inside = [&img](const cv::Rect & c) {
      return c.x >= 2 && c.y >= 54 && c.x + c.width <= img.width - 2 && c.y + c.height <= img.height - 28;   // 위 · 아래 띠 피함
    };
  auto free = [&](const cv::Point & a) {
      const cv::Rect c(a, sz);
      return inside(c) && std::none_of(used.begin(), used.end(), [&c](const cv::Rect & u) {return (u & c).area() > 0;});
    };
  for (const auto & a : cand) {
    if (free(a)) {return a;}
  }
  cv::Point a(std::clamp(cand[0].x, 2, std::max(2, img.width - sz.width - 2)), std::max(cand[0].y, 54));
  for (int k = 0; k < 12 && !free(a); ++k) {a.y += sz.height + 3;}
  return a;
}

// bbox (xyxy) → 영상 안 사각. 모두 0 · 크기 2 px 미만이면 nullopt
std::optional<cv::Rect> bbox_rect(const std::array<float, 4> & b, const cv::Size & img)
{
  if (!(b[2] - b[0] >= 2.0f && b[3] - b[1] >= 2.0f)) {return std::nullopt;}
  const cv::Rect r(cv::Point(static_cast<int>(std::lround(b[0])), static_cast<int>(std::lround(b[1]))),
    cv::Point(static_cast<int>(std::lround(b[2])), static_cast<int>(std::lround(b[3]))));
  const cv::Rect c = r & cv::Rect(0, 0, img.width, img.height);
  if (c.area() == 0) {return std::nullopt;}
  return c;
}

// 검출 상자: 얇은 테 + 굵은 모서리 꺾쇠 (어두운 바탕 테로 어떤 배경에서도 보이게). dim = 뺀 검출 (점선 · 얇게)
void corner_box(cv::Mat & img, const cv::Rect & r, const cv::Scalar & c, bool dim)
{
  const cv::Point tl = r.tl(), br = r.br() - cv::Point(1, 1);
  if (dim) {
    const std::vector<cv::Point> poly = {tl, {br.x, tl.y}, br, {tl.x, br.y}};
    for (size_t i = 0; i < 4; ++i) {   // 점선
      const cv::Point a = poly[i], b = poly[(i + 1) % 4];
      const int n = std::max(1, static_cast<int>(cv::norm(b - a) / 6));
      for (int k = 0; k < n; k += 2) {
        cv::line(img, a + (b - a) * k / n, a + (b - a) * (k + 1) / n, c, 1, cv::LINE_AA);
      }
    }
    return;
  }
  cv::rectangle(img, tl, br, kPanel, 3, cv::LINE_AA);
  cv::rectangle(img, tl, br, c, 1, cv::LINE_AA);
  const int L = std::max(6, std::min(r.width, r.height) / 4);
  for (const auto & [p, dx, dy] : {std::tuple{tl, 1, 1}, std::tuple{cv::Point(br.x, tl.y), -1, 1},
      std::tuple{br, -1, -1}, std::tuple{cv::Point(tl.x, br.y), 1, -1}})
  {
    for (const auto & [th, cc] : {std::pair{5, kPanel}, std::pair{3, c}}) {
      cv::line(img, p, p + cv::Point(dx * L, 0), cc, th, cv::LINE_AA);
      cv::line(img, p, p + cv::Point(0, dy * L), cc, th, cv::LINE_AA);
    }
  }
}

cv::Size chip_size(const std::string & s, double scale = 0.45)
{
  int base = 0;
  const auto sz = cv::getTextSize(s, kFont, scale, 1, &base);
  return {sz.width + 10, sz.height + base + 10};
}

void blend(cv::Mat & img, const cv::Mat & over, double a)
{
  cv::addWeighted(over, a, img, 1.0 - a, 0.0, img);
}

// 퍽 하나의 짧은 이유 (영문, 그림용). "" = 문제 없음
std::string obs_reason(const PlaceVerify & pv, const VerifyObs & o, int planned_cell)
{
  if (o.ok) {return "";}
  if (o.on_box == 0) {return "OFF BOX";}
  if (o.id.empty()) {return "NO ID";}
  if (!pv.cell_ids.empty() && planned_cell < 0) {return "NOT PLANNED";}
  if (o.cell < 0) {
    char b[32];
    std::snprintf(b, sizeof(b), "%.0fmm > %.0f", std::min(o.cell_error_m, 9.999) * 1000.0, pv.cell_tol_m * 1000.0);
    return b;
  }
  if (pv.group == 'Y') {return o.shape.empty() ? "SHAPE?" : (pv.y_shape.empty() ? "SHAPE MIX" : "SHAPE != " + short_shape(pv.y_shape));}
  return o.color.empty() ? "COLOR?" : "COLOR";
}
}  // namespace

void apply_camera_drift(VerifyCamera & cam, const std::array<double, 4> & q_xyzw)
{
  double x = q_xyzw[0], y = q_xyzw[1], z = q_xyzw[2], w = q_xyzw[3];
  const double n = std::sqrt(x * x + y * y + z * z + w * w);
  if (!(n > 0.5)) {return;}
  x /= n; y /= n; z /= n; w /= n;
  const double D[9] = {
    1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w),
    2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
    2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)};
  std::array<double, 9> r{};
  std::array<double, 3> t{};
  for (int i = 0; i < 3; ++i) {   // Dᵀ · R, Dᵀ · t
    for (int j = 0; j < 3; ++j) {
      for (int k = 0; k < 3; ++k) {r[3 * i + j] += D[3 * k + i] * cam.R[3 * k + j];}
    }
    for (int k = 0; k < 3; ++k) {t[i] += D[3 * k + i] * cam.t[k];}
  }
  cam.R = r;
  cam.t = t;
}

void drop_stale_bboxes(PlaceVerify & pv, double since_s)
{
  for (auto & o : pv.seen) {
    if (o.stamp >= 0.0 && o.stamp < since_s) {o.bbox = {0.0f, 0.0f, 0.0f, 0.0f};}
  }
}

std::optional<cv::Point2d> project_map_point(const VerifyCamera & cam, double x, double y, double z)
{
  const auto & R = cam.R;
  const double px = R[0] * x + R[1] * y + R[2] * z + cam.t[0];
  const double py = R[3] * x + R[4] * y + R[5] * z + cam.t[1];
  const double pz = R[6] * x + R[7] * y + R[8] * z + cam.t[2];
  if (pz <= 0.01 || cam.fx <= 0.0) {return std::nullopt;}
  return cv::Point2d(cam.fx * px / pz + cam.cx, cam.fy * py / pz + cam.cy);
}

void draw_verify(cv::Mat & bgr, const PlaceVerify & pv, const VerifyCamera & cam, double puck_z, double puck_half_m)
{
  if (bgr.empty()) {return;}
  const double top_z = puck_z - puck_half_m;          // 박스 윗면
  const bool by_id = !pv.cell_ids.empty();

  // 칸 상태: 0 = 맞음, 1 = 문제 (퍽 있는데 불합격 · 안 보임), 2 = 빈 칸 (안 놓음)
  std::vector<int> cell_state(pv.cells.size(), 1);
  for (size_t k = 0; k < pv.cells.size(); ++k) {
    if (by_id && (k >= pv.cell_ids.size() || pv.cell_ids[k].empty())) {cell_state[k] = 2; continue;}
    for (const auto & o : pv.seen) {
      if (o.cell == static_cast<int>(k) && o.ok) {cell_state[k] = 0;}
    }
  }
  // 퍽 → 계획 칸 (ID 판정: 그 ID 를 놓은 칸, 그룹 판정: 든 칸 · 없으면 가장 가까운 칸)
  auto planned_cell = [&](const VerifyObs & o) {
      if (by_id) {
        for (size_t k = 0; k < pv.cell_ids.size(); ++k) {
          if (!o.id.empty() && pv.cell_ids[k] == o.id) {return static_cast<int>(k);}
        }
        return -1;
      }
      if (o.cell >= 0) {return o.cell;}
      int best = -1;
      double bd = 1e9;
      for (size_t k = 0; k < pv.cells.size(); ++k) {
        const double d = std::hypot(o.x - pv.cells[k][0], o.y - pv.cells[k][1]);
        if (d < bd) {bd = d; best = static_cast<int>(k);}
      }
      return best;
    };

  // ---- 바닥 층 (반투명): 박스 윗면 · 칸 허용 원
  cv::Mat over = bgr.clone();
  std::vector<cv::Point> box_poly;
  if (pv.box) {
    box_poly = project_poly(cam, square_pts(pv.box->x, pv.box->y, pv.box->half, pv.box->yaw), top_z);
    if (box_poly.size() == 4) {cv::fillConvexPoly(over, box_poly, kBox * 0.5, cv::LINE_AA);}
  }
  std::vector<std::vector<cv::Point>> tol_polys(pv.cells.size());
  for (size_t k = 0; k < pv.cells.size(); ++k) {
    tol_polys[k] = project_poly(cam, circle_pts(pv.cells[k][0], pv.cells[k][1], pv.cell_tol_m), top_z);
    if (tol_polys[k].empty() || cell_state[k] != 1) {continue;}   // 문제 칸만 칠함 (허용 원이 서로 겹쳐 다 칠하면 어수선)
    cv::fillPoly(over, std::vector<std::vector<cv::Point>>{tol_polys[k]}, kBad, cv::LINE_AA);
  }
  blend(bgr, over, 0.18);

  // ---- 선: 박스 테두리 · 칸 원 · 칸 중심
  if (box_poly.size() == 4) {cv::polylines(bgr, box_poly, true, kBox, 2, cv::LINE_AA);}
  for (size_t k = 0; k < pv.cells.size(); ++k) {
    const auto c = cell_state[k] == 0 ? kOk : (cell_state[k] == 1 ? kBad : kEmpty);
    if (!tol_polys[k].empty()) {
      if (cell_state[k] == 1) {
        cv::polylines(bgr, tol_polys[k], true, c, 1, cv::LINE_AA);
      } else {
        dashed_poly(bgr, tol_polys[k], c, 1);   // 맞음 · 빈 칸 = 점선 (허용 범위만 보이게)
      }
    }
    if (const auto p = project_map_point(cam, pv.cells[k][0], pv.cells[k][1], top_z)) {
      cv::drawMarker(bgr, ipt(*p), c, cv::MARKER_CROSS, 10, 1, cv::LINE_AA);
      std::string t = std::to_string(k);
      if (by_id) {t += k < pv.cell_ids.size() && !pv.cell_ids[k].empty() ? ":" + pv.cell_ids[k] : ":-";}
      cv::putText(bgr, t, ipt(*p) + cv::Point(5, 14), cv::FONT_HERSHEY_SIMPLEX, 0.38, c, 1, cv::LINE_AA);
    }
  }

  // ---- 퍽: 계획 칸 → 퍽 오차 화살표, 검출 상자 (perception bbox, 없으면 중심 표시), 라벨 칩
  std::vector<cv::Rect> used;
  struct Lab {cv::Point at; std::string s; cv::Scalar c; double scale;};
  std::vector<Lab> labels;
  // 퍽 표시 → 칩 붙일 기준 사각 (bbox 또는 중심 둘레 작은 사각). 투영도 bbox 도 없으면 빈 사각
  auto mark = [&](const VerifyObs & o, const cv::Scalar & col, bool dim) {
      if (const auto r = bbox_rect(o.bbox, bgr.size())) {
        corner_box(bgr, *r, col, dim);
        return *r;
      }
      const auto p = project_map_point(cam, o.x, o.y, puck_z);
      if (!p) {return cv::Rect();}
      cv::drawMarker(bgr, ipt(*p), kPanel, cv::MARKER_TILTED_CROSS, 14, 4, cv::LINE_AA);
      cv::drawMarker(bgr, ipt(*p), col, cv::MARKER_TILTED_CROSS, 14, dim ? 1 : 2, cv::LINE_AA);
      return cv::Rect(ipt(*p) - cv::Point(7, 7), cv::Size(14, 14));
    };
  // 칩 자리: 상자 위 → 아래 → 오른쪽 → 왼쪽, 겹치면 밀기. 지시선은 상자에서 칩 가까운 점으로
  auto place_label = [&](const cv::Rect & r, const std::string & s, const cv::Scalar & bg, const cv::Scalar & col, double scale) {
      const auto sz = chip_size(s, scale);
      const auto at = chip_place_rect(r, sz, used, bgr.size());
      used.emplace_back(at, sz);
      labels.push_back({at, s, bg, scale});
      const cv::Rect c(at, sz);
      if ((c & r).area() == 0) {
        const cv::Point a(std::clamp(c.x + c.width / 2, r.x, r.x + r.width), std::clamp(c.y + c.height / 2, r.y, r.y + r.height));
        const cv::Point b(std::clamp(a.x, c.x, c.x + c.width), std::clamp(a.y, c.y, c.y + c.height));
        cv::line(bgr, a, b, col, 1, cv::LINE_AA);
      }
    };
  for (const auto & o : pv.seen) {
    const auto col = o.ok ? kOk : kBad;
    const int k = planned_cell(o);
    if (k >= 0 && k < static_cast<int>(pv.cells.size())) {
      const auto a = project_map_point(cam, pv.cells[k][0], pv.cells[k][1], top_z);
      const auto b = project_map_point(cam, o.x, o.y, top_z);
      if (a && b && std::hypot(b->x - a->x, b->y - a->y) > 6.0) {
        cv::arrowedLine(bgr, ipt(*a), ipt(*b), col, 1, cv::LINE_AA, 0, 0.15);
      }
    }
    const auto r = mark(o, col, false);
    if (r.area() == 0) {continue;}
    std::string s = (o.id.empty() ? "?" : o.id) + "  " + short_color(o.color) + "/" + short_shape(o.shape);
    const auto why = obs_reason(pv, o, k);
    if (!why.empty()) {s += "  " + why;}
    place_label(r, s, o.ok ? kOk * 0.75 : kBad * 0.8, col, 0.45);
  }
  for (const auto & o : pv.ignored) {
    const auto r = mark(o, kIgnored, true);
    if (r.area() == 0) {continue;}
    place_label(r, (o.id.empty() ? "?" : o.id) + " ignored (edge)", cv::Scalar(70, 70, 70), kIgnored, 0.38);
  }
  for (const auto & l : labels) {chip(bgr, l.at, l.s, l.c, l.scale);}

  // ---- 위 띠: 장소 · 그룹 · 본 수 / 눈 수 · 결과
  over = bgr.clone();
  cv::rectangle(over, {0, 0}, {bgr.cols, 52}, kPanel, cv::FILLED);
  blend(bgr, over, 0.78);
  char name[16];
  std::snprintf(name, sizeof(name), "L%d", pv.place + 1);
  cv::putText(bgr, name, {10, 34}, kFont, 0.95, kText, 2, cv::LINE_AA);
  const auto gcol = pv.group == 'R' ? kGroupR : (pv.group == 'G' ? kGroupG : kGroupY);
  char grp[24];
  std::snprintf(grp, sizeof(grp), "%c x%d", pv.group, pv.expected);
  const auto gr = chip(bgr, {58, 12}, grp, gcol, 0.55, pv.group == 'Y' ? cv::Scalar(20, 20, 20) : kText);
  int n_ok = 0;
  for (const auto & o : pv.seen) {n_ok += o.ok ? 1 : 0;}
  char cnt[48];
  std::snprintf(cnt, sizeof(cnt), "ok %d / %d   seen %zu", n_ok, pv.expected, pv.seen.size());
  cv::putText(bgr, cnt, {gr.x + gr.width + 12, 32}, kFont, 0.55, kText, 1, cv::LINE_AA);
  const std::string res = pv.ok ? "SUCCESS" : "FAIL";
  const auto rsz = chip_size(res, 0.6);
  chip(bgr, {bgr.cols - rsz.width - 10, 12}, res, pv.ok ? kOk * 0.85 : kBad * 0.9, 0.6);
  std::string sub = "frame " + pv.frame_source;
  char tol[32];
  std::snprintf(tol, sizeof(tol), "   tol %.0fmm", pv.cell_tol_m * 1000.0);
  sub += tol;
  if (pv.fitted) {
    char f[64];
    std::snprintf(f, sizeof(f), "   fit (%+.0f, %+.0f)mm %+.1fdeg", pv.fit_dx * 1000.0, pv.fit_dy * 1000.0, pv.fit_dyaw * 180.0 / M_PI);
    sub += f;
  }
  if (pv.group == 'Y') {sub += "   Y shape " + (pv.y_shape.empty() ? std::string("?") : short_shape(pv.y_shape));}
  if (!pv.ignored.empty()) {sub += "   ignored " + std::to_string(pv.ignored.size());}
  cv::putText(bgr, sub, {12, 48}, cv::FONT_HERSHEY_SIMPLEX, 0.38, kSub, 1, cv::LINE_AA);

  // ---- 아래 띠: 안 보인 칸 · 빈 칸 (ID 판정)
  std::vector<std::string> notes;
  if (by_id) {
    for (size_t k = 0; k < pv.cell_ids.size() && k < pv.cells.size(); ++k) {
      const auto & id = pv.cell_ids[k];
      const std::string spot = k < pv.cell_spots_en.size() ? pv.cell_spots_en[k] : "cell " + std::to_string(k);
      if (id.empty()) {notes.push_back(spot + " empty"); continue;}
      const bool seen = std::any_of(pv.seen.begin(), pv.seen.end(), [&id](const VerifyObs & o) {return o.id == id;});
      if (!seen) {notes.push_back("ID " + id + " not seen (" + spot + ")");}
    }
  }
  if (static_cast<int>(pv.seen.size()) != pv.expected) {
    notes.push_back("count " + std::to_string(pv.seen.size()) + " != " + std::to_string(pv.expected));
  }
  if (!notes.empty()) {
    std::string s;
    for (size_t i = 0; i < notes.size(); ++i) {s += (i ? "   |   " : "") + notes[i];}
    over = bgr.clone();
    cv::rectangle(over, {0, bgr.rows - 26}, {bgr.cols, bgr.rows}, kPanel, cv::FILLED);
    blend(bgr, over, 0.78);
    cv::putText(bgr, s, {10, bgr.rows - 9}, cv::FONT_HERSHEY_SIMPLEX, 0.45, kBad + cv::Scalar(40, 40, 20), 1, cv::LINE_AA);
  }
}

}  // namespace mission
