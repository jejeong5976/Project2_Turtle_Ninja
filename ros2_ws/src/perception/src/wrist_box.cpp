// wrist_box.cpp — 손목 깊이로 박스 윗면 중심 · 방향 구현, 설명은 wrist_box.hpp

#include "perception/wrist_box.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <optional>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace perception::wrist
{

double fold_quarter(double a, double ref)
{
  const double q = M_PI / 2;
  double d = std::remainder(a - ref, q);   // (−45°, 45°]
  if (d <= -q / 2) {d += q;}
  return ref + d;
}

BoxFit fit_place_box(
  const cv::Mat1f & depth, const Intrinsics & K, const Mat3 & R, const Vec3 & t,
  const BoxParams & p, const Vec3 & guess, cv::Mat1b * edge_mask)
{
  BoxFit out;
  const int s = std::max(1, p.stride);
  const int gh = depth.rows / s, gw = depth.cols / s;
  if (gh < 3 || gw < 3) {
    out.why = "깊이 영상 작음";
    return out;
  }
  // 1) 화소 → 바닥 프레임 점, 높이로 나눔 (0 = 없음 · 1 = 윗면 · 2 = 바닥 쪽 · 3 = 그 밖)
  cv::Mat1b cls(gh, gw, uchar(0)), low(gh, gw, uchar(0));
  std::vector<Vec3> P(static_cast<size_t>(gh) * gw);
  for (int r = 0; r < gh; ++r) {
    const float * row = depth.ptr<float>(r * s);
    for (int c = 0; c < gw; ++c) {
      const double z = row[c * s];
      if (!(z > 0.0) || z > p.max_range) {continue;}
      const double u = c * s + 0.5, v = r * s + 0.5;
      const Vec3 pc((u - K.cx) / K.fx * z, (v - K.cy) / K.fy * z, z);
      const Vec3 pb = R * pc + t;
      P[static_cast<size_t>(r) * gw + c] = pb;
      if (std::fabs(pb.z() - p.top_z) <= p.band) {
        cls(r, c) = 1;
      } else if (pb.z() < p.top_z - p.drop) {
        cls(r, c) = 2;
        low(r, c) = 255;
      } else {
        cls(r, c) = 3;
      }
    }
  }
  // 2) 테두리 점 = 윗면 화소 중 dilate 안에 바닥 쪽 화소가 있는 것
  cv::Mat1b lowd;
  const int k = 2 * std::max(1, p.dilate) + 1;
  cv::dilate(low, lowd, cv::Mat::ones(k, k, CV_8U));
  std::vector<Eigen::Vector2d> pts;
  for (int r = 0; r < gh; ++r) {
    for (int c = 0; c < gw; ++c) {
      if (cls(r, c) != 1 || !lowd(r, c)) {continue;}
      const Vec3 & pb = P[static_cast<size_t>(r) * gw + c];
      if (pb.x() < p.min_x) {continue;}
      pts.emplace_back(pb.x(), pb.y());
      if (edge_mask) {
        if (edge_mask->size() != depth.size()) {*edge_mask = cv::Mat1b::zeros(depth.size());}
        cv::rectangle(*edge_mask, cv::Rect(c * s, r * s, s, s), 255, cv::FILLED);
      }
    }
  }
  std::vector<Eigen::Vector2d> low_pts;   // 바닥 쪽 점 (뒤집힌 답 거르기), 화소 둘 중 하나
  for (int r = 0; r < gh; r += 2) {
    for (int c = 0; c < gw; c += 2) {
      if (cls(r, c) != 2) {continue;}
      const Vec3 & pb = P[static_cast<size_t>(r) * gw + c];
      if (pb.x() >= p.min_x) {low_pts.emplace_back(pb.x(), pb.y());}
    }
  }
  out.edge_pts = static_cast<int>(pts.size());
  if (out.edge_pts < 2 * p.min_side_pts) {
    out.why = "테두리 점 부족 (" + std::to_string(out.edge_pts) + ")";
    return out;
  }

  // 3) 정사각형 맞춤: 상태 (cx, cy, yaw), 점마다 가까운 변 → 잔차 = 박스 좌표에서 그 변까지 (Huber 가중 가우스-뉴턴)
  //    시작 = 처음 추정. 그 답이 변 부족 · 안에 바닥 점 (뒤집힌 답) 이면 search > 0 일 때 추정 ±search 격자에서 다시 (가장 잘 맞는 것)
  const double h = p.size / 2;
  struct Cand
  {
    bool fin{false};
    double cx{0}, cy{0}, yaw{0}, rms{0};
    std::array<int, 4> sp{};
    int inside{0};
    bool x_ok{false}, y_ok{false};
    int score() const {return sp[0] + sp[1] + sp[2] + sp[3] - 3 * inside;}
    bool good(int max_inside) const {return fin && x_ok && y_ok && (max_inside < 0 || inside <= max_inside);}
  };
  // 넓게 찾기 시작마다 맞춤은 테두리 점 최대 kSub 개 (고르게 건너뜀) 로 → 고른 답만 전체 점으로 다시 맞춤 (한 장 비용 제한)
  constexpr std::size_t kSub = 300;
  std::vector<Eigen::Vector2d> sub;
  const auto solve = [&](double cx, double cy, double yaw, const std::vector<Eigen::Vector2d> & fit_pts) {
      Cand o;
      for (int it = 0; it < p.iters; ++it) {
        const double co = std::cos(yaw), si = std::sin(yaw);
        Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
        Eigen::Vector3d b = Eigen::Vector3d::Zero();
        for (size_t i = 0; i < fit_pts.size(); ++i) {
          const double dx = fit_pts[i].x() - cx, dy = fit_pts[i].y() - cy;
          const double qx = co * dx + si * dy, qy = -si * dx + co * dy;
          const bool xside = std::fabs(std::fabs(qx) - h) < std::fabs(std::fabs(qy) - h);
          double r;
          Eigen::Vector3d J;
          if (xside) {
            const double sg = qx < 0 ? -1.0 : 1.0;
            r = qx - sg * h;
            J << -co, -si, qy;
          } else {
            const double sg = qy < 0 ? -1.0 : 1.0;
            r = qy - sg * h;
            J << si, -co, -qx;
          }
          const double w = std::fabs(r) <= p.huber ? 1.0 : p.huber / std::fabs(r);
          A += w * J * J.transpose();
          b += w * J * r;
        }
        const Eigen::Vector3d dlt = A.ldlt().solve(-b);
        if (!dlt.allFinite()) {return o;}
        cx += dlt(0);
        cy += dlt(1);
        yaw += dlt(2);
        if (std::hypot(dlt(0), dlt(1)) < 1e-5 && std::fabs(dlt(2)) < 1e-5) {break;}
      }
      o.fin = std::isfinite(cx) && std::isfinite(cy) && std::isfinite(yaw);
      if (o.fin) {yaw = fold_quarter(yaw, guess.z());}   // 정사각형 → 변 이름을 접은 방향 기준으로 셈
      o.cx = cx;
      o.cy = cy;
      o.yaw = yaw;
      // 변마다 점 수 · 잔차
      const double co = std::cos(yaw), si = std::sin(yaw);
      double ss = 0.0;
      int n_in = 0;
      for (const auto & q0 : pts) {
        const double dx = q0.x() - cx, dy = q0.y() - cy;
        const double qx = co * dx + si * dy, qy = -si * dx + co * dy;
        const double ex = std::fabs(std::fabs(qx) - h), ey = std::fabs(std::fabs(qy) - h);
        const double e = std::min(ex, ey);
        if (e <= p.side_tol) {
          if (ex < ey) {++o.sp[qx < 0 ? 0 : 1];} else {++o.sp[qy < 0 ? 3 : 2];}
        }
        if (e <= p.huber) {ss += e * e; ++n_in;}
      }
      o.rms = n_in ? std::sqrt(ss / n_in) : 0.0;
      o.x_ok = o.sp[0] >= p.min_side_pts || o.sp[1] >= p.min_side_pts;
      o.y_ok = o.sp[2] >= p.min_side_pts || o.sp[3] >= p.min_side_pts;
      // 윗면 안쪽 (테두리 1.5 cm 안) 에 든 바닥 점 = 박스가 아닌 자리 (변 하나 건너 뒤집힌 답)
      const double hi = h - 0.015;
      for (const auto & q0 : low_pts) {
        const double dx = q0.x() - cx, dy = q0.y() - cy;
        if (std::fabs(co * dx + si * dy) < hi && std::fabs(-si * dx + co * dy) < hi) {++o.inside;}
      }
      return o;
    };
  Cand best = solve(guess.x(), guess.y(), guess.z(), pts);
  if (p.search > 0.0) {
    // 45° 근처에서는 0° 시작이 두 변 일부에만 맞는 국소해에서도 good 이 될 수 있다.
    // 넓은 탐색과 같은 3개 방향으로 같은 중심을 풀고, 더 많은 테두리를 설명하는 답을 쓴다.
    for (const double dy0 : {M_PI / 6, M_PI / 3}) {
      const Cand c = solve(guess.x(), guess.y(), guess.z() + dy0, pts);
      if (c.good(p.max_inside) && (!best.good(p.max_inside) || c.score() > best.score())) {best = c;}
    }
  }
  if (p.search > 0.0 && !best.good(p.max_inside)) {
    // 시작 격자 = 테두리 점 범위 ± h (손목이 옆을 봐도 보이는 곳 전체), 답은 처음 추정에서 search 안만
    double x0 = 1e9, x1 = -1e9, y0 = 1e9, y1 = -1e9;
    for (const auto & q0 : pts) {
      x0 = std::min(x0, q0.x()); x1 = std::max(x1, q0.x());
      y0 = std::min(y0, q0.y()); y1 = std::max(y1, q0.y());
    }
    const std::size_t k = (pts.size() + kSub - 1) / kSub;
    for (std::size_t i = 0; i < pts.size(); i += k) {sub.push_back(pts[i]);}
    const double step = std::max(0.02, p.search_step);
    const double reach2 = (h * 1.5) * (h * 1.5);
    // 시작 = 테두리 점 가까운 격자 칸, 처음 추정에 가까운 순 → 시작 수 search_max_starts · 시간 search_budget_ms 넘으면 그때까지 가장 좋은 답
    std::vector<std::array<double, 3>> starts;   // (거리², x, y)
    for (double sx = x0 - h; sx <= x1 + h + 1e-9; sx += step) {
      for (double sy = y0 - h; sy <= y1 + h + 1e-9; sy += step) {
        bool near = false;   // 테두리 점이 가까운 시작만 (박스 중심 ↔ 테두리 ≤ h√2)
        for (const auto & q0 : sub) {
          if ((q0.x() - sx) * (q0.x() - sx) + (q0.y() - sy) * (q0.y() - sy) < reach2) {near = true; break;}
        }
        if (near) {starts.push_back({(sx - guess.x()) * (sx - guess.x()) + (sy - guess.y()) * (sy - guess.y()), sx, sy});}
      }
    }
    std::sort(starts.begin(), starts.end(), [](const auto & u, const auto & v) {return u[0] < v[0];});
    const auto t0 = std::chrono::steady_clock::now();
    std::optional<Cand> pick;
    for (const auto & st : starts) {
      if (out.search_starts >= p.search_max_starts ||
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count() > p.search_budget_ms)
      {
        out.search_capped = true;
        break;
      }
      ++out.search_starts;
      for (const double dy0 : {0.0, M_PI / 6, M_PI / 3}) {
        const Cand c = solve(st[1], st[2], guess.z() + dy0, sub);
        // 넓게 찾은 답만 더 거름: 테두리 점 search_min_frac 이상을 변으로 설명 · 차체와 겹치지 않음 (안쪽 원이 차체 앞 min_x − 3 cm 안)
        const int ns = c.sp[0] + c.sp[1] + c.sp[2] + c.sp[3];
        const bool body = c.cx - h < p.min_x - 0.03 && std::fabs(c.cy) - h < p.body_half_y;
        const bool far = std::hypot(c.cx - guess.x(), c.cy - guess.y()) > p.search;
        if (!c.good(p.max_inside) || body || far || ns < p.search_min_frac * static_cast<double>(pts.size())) {continue;}
        if (!pick || c.score() > pick->score()) {pick = c;}
      }
    }
    out.search_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    if (pick) {
      const Cand r = solve(pick->cx, pick->cy, pick->yaw, pts);   // 전체 점으로 마무리 (거의 그대로)
      best = r.good(p.max_inside) && std::hypot(r.cx - pick->cx, r.cy - pick->cy) < 0.02 ? r : *pick;
    }
  }
  if (!best.fin) {
    out.why = "맞춤 발산";
    return out;
  }
  const double cx = best.cx, cy = best.cy;
  out.side_pts = best.sp;
  out.rms = best.rms;
  out.inside_low = best.inside;
  // 앞 · 뒤 축과 좌우 축이 정해져야 함. 박스 yaw 를 처음 추정 근처로 접으면 변 이름도 바뀔 수 있어 접기 전에 셈
  const auto & sp = out.side_pts;
  out.x = cx;
  out.y = cy;
  out.yaw = best.yaw;
  if (!best.x_ok || !best.y_ok) {
    out.why = "변 부족 (앞 " + std::to_string(sp[0]) + " 뒤 " + std::to_string(sp[1]) + " 왼 " + std::to_string(sp[2]) +
      " 오른 " + std::to_string(sp[3]) + ")";
    return out;
  }
  if (p.search > 0.0 && p.max_inside >= 0 && best.inside > p.max_inside) {   // search 0 = 이 검사도 안 함
    out.why = "윗면 안에 바닥 점 " + std::to_string(best.inside) + " (뒤집힌 맞춤)";
    return out;
  }
  // 뒤 변을 못 봤으면 앞뒤 위치는 앞 변으로만 정해짐 → 로봇 쪽 변이 차체에 붙었으면 그 변이 가려졌을 수 있음
  //   (뒤 변을 보면 앞 변이 가려져도 앞뒤가 정해지므로 검사 안 함)
  if (sp[1] >= p.min_side_pts) {
    out.ok = true;
    return out;
  }
  double near_x = 1e9;
  for (int k2 = 0; k2 < 4; ++k2) {
    const double a = out.yaw + k2 * M_PI / 2;
    near_x = std::min(near_x, cx + h * std::cos(a));
  }
  if (near_x < p.front_min_x) {
    out.why = "박스 앞 변이 차체에 너무 가까움 (" + std::to_string(near_x) + " m)";
    return out;
  }
  out.ok = true;
  return out;
}

}  // namespace perception::wrist
