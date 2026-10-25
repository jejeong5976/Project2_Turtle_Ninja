#include "angle_grid.hpp"
#include <cassert>
#include <iostream>
using namespace lidar_diagnostic;
int main() {
  // Cardinal bearings, including the previously discarded index zero.
  const std::vector<Sample> cardinal{{0,1000,100,false},{90,2000,100,false},
    {180,3000,100,false},{270,4000,100,false}};
  const auto a = convert(cardinal);
  assert(a.bins[0].source == 0 && a.bins[0].range == 1);
  assert(a.bins[300].range == 2 && a.bins[200].range == 3 && a.bins[100].range == 4);
  assert(std::abs(a.angle_max - 399 * a.angle_increment) < 1e-12);
  assert(std::isnan(a.bins[1].range));
  assert(convert(cardinal,400,true).bins[100].range == 2);
  // Wrap, nearest-return collision, and associated intensity/source selection.
  const auto b = convert({{360,1500,101,false},{359.9,1000,102,false},
                          {0.1,1200,103,false},{0,1000,104,false}});
  assert(b.collisions == 3 && b.bins[0].source == 1 && b.bins[0].intensity == 102);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const auto c = convert({{0,0,100,false},{0,99,100,false},{0,100001,100,false},
    {nan,1000,100,false},{361,1000,100,false},{-1,1000,100,false},
    {0,nan,100,false},{0,150,65,false},{90,150,65,true},{180,100,66,false},
    {270,100000,300,false}});
  assert(c.rejected == 8 && c.bins[300].intensity == 255);
  assert(c.bins[200].range == 0.1 && c.bins[100].intensity == 254);
  assert(convert({}).bins.size() == 400);
  bool threw = false;
  try { convert(cardinal,1); } catch (const std::invalid_argument&) { threw = true; }
  assert(threw);
  // Irregular angles: ordering must not determine the assigned bearing.
  const auto d = convert({{11,1000,100,false},{301,2000,100,false},{101,3000,100,false}});
  assert(d.bins[388].range == 1 && d.bins[66].range == 2 && d.bins[288].range == 3);
  std::cout << "PASS: angular placement, index zero, wrap, collisions, invalid ranges, metadata\n";
}
