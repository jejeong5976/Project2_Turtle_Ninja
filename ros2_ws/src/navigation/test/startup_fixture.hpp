// startup_fixture.hpp — test_auto_initial_pose.py 의 fixture · raycast 를 옮김 (지도 raycast 합성 스캔)
#pragma once

#include <cmath>
#include <memory>
#include <vector>

#include "navigation/pose_match_core.hpp"

namespace startup_test
{
using navigation::pose_match::MapModel;
using navigation::pose_match::Points;
using navigation::pose_match::Pose;

struct Grid
{
  int64_t w, h;
  std::vector<int16_t> g;
  Grid(int64_t height, int64_t width) : w(width), h(height), g(static_cast<std::size_t>(width * height), 0) {}
  // 파이썬 grid[r0:r1, c0:c1] = v
  void fill(int64_t r0, int64_t r1, int64_t c0, int64_t c1, int16_t v)
  {
    for (int64_t r = r0; r < r1; ++r) {
      for (int64_t c = c0; c < c1; ++c) {
        g[r * w + c] = v;
      }
    }
  }
};

inline std::shared_ptr<const MapModel> fixture(double origin_yaw = 0.0)
{
  const double res = .025;
  Grid grid(200, 240);
  grid.fill(0, 3, 0, 240, 100);
  grid.fill(197, 200, 0, 240, 100);
  grid.fill(0, 200, 0, 3, 100);
  grid.fill(0, 200, 237, 240, 100);
  grid.fill(20, 75, 60, 64, 100);
  grid.fill(120, 124, 35, 110, 100);
  grid.fill(60, 150, 175, 179, 100);
  grid.fill(150, 155, 120, 175, 100);
  grid.fill(35, 40, 130, 190, 100);
  return MapModel::create(grid.g, grid.w, grid.h, res, Pose{-3.0, -2.5, origin_yaw});
}

inline Points raycast(const MapModel & model, const Pose & pose, int beams = 240)
{
  // np.linspace(-pi, pi, beams, endpoint=False) · np.arange(.255, 3.99, res / 5)
  const double step_a = (M_PI - (-M_PI)) / beams;
  const double step_d = model.resolution / 5;
  const auto nd = static_cast<int64_t>(std::ceil((3.99 - .255) / step_d));
  Points out;
  for (int i = 0; i < beams; ++i) {
    const double angle = static_cast<double>(i) * step_a + -M_PI;
    const double a = angle + pose[2];
    for (int64_t k = 0; k < nd; ++k) {
      const double d = .255 + static_cast<double>(k) * step_d;
      const double x = pose[0] + d * std::cos(a);
      const double y = pose[1] + d * std::sin(a);
      int64_t u = 0, v = 0;
      if (model.cell(x, y, u, v) && model.grid[v * model.width + u] > 50) {
        out.push(d * std::cos(angle), d * std::sin(angle));
        break;
      }
    }
  }
  return out;
}

}  // namespace startup_test
