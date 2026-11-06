// Offline, bounded dual-lidar pose fit. No ROS publishers or hardware access.
#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <nlohmann/json.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>
namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;
using Point = bg::model::point<double, 2, bg::cs::cartesian>;
using Tree = bgi::rtree<Point, bgi::quadratic<16>>;
using Json = nlohmann::json;
using Pose = std::array<double, 3>;
struct Sample { double x, y; int sensor; };
std::string token(std::istream &in) {
  std::string s;
  while (in >> s) {
    if (s[0] == '#') { std::getline(in, s); continue; }
    return s;
  }
  throw std::runtime_error("PGM header truncated");
}
std::vector<double> residuals(const Tree &tree, const std::vector<Sample> &points,
                              const Pose &pose) {
  std::vector<double> distances;
  std::vector<Point> nearest;
  const double c = std::cos(pose[2]), s = std::sin(pose[2]);
  for (const auto &p : points) {
    const Point q(pose[0] + c*p.x - s*p.y, pose[1] + s*p.x + c*p.y);
    nearest.clear();
    tree.query(bgi::nearest(q, 1), std::back_inserter(nearest));
    distances.push_back(bg::distance(q, nearest.at(0)));
  }
  return distances;
}
double score(const std::vector<double> &d) {
  double total = 0;
  for (double x : d) total += std::pow(std::min(x, .10), 2);
  return total / d.size();
}
Json metrics(std::vector<double> d) {
  if (d.empty()) throw std::runtime_error("Empty scan subset");
  std::sort(d.begin(), d.end());
  return {{"samples", d.size()}, {"median_m", d[d.size()/2]},
    {"p90_m", d[static_cast<size_t>(.9*(d.size()-1))]},
    {"within_3cm", double(std::count_if(d.begin(),d.end(),[](double x){return x<=.03;}))/d.size()},
    {"within_5cm", double(std::count_if(d.begin(),d.end(),[](double x){return x<=.05;}))/d.size()},
    {"capped_rms_m", std::sqrt(score(d))}};
}
Pose fit(const Tree &tree, const std::vector<Sample> &points, const Pose &seed) {
  Pose best = seed;
  double best_score = score(residuals(tree, points, best));
  const double degree = std::acos(-1)/180;
  for (int a=-12; a<=12; ++a) for (int x=-15; x<=15; ++x) for (int y=-15; y<=15; ++y) {
    Pose p{seed[0]+x*.02, seed[1]+y*.02, seed[2]+a*.5*degree};
    const double value = score(residuals(tree, points, p));
    if (value < best_score) { best_score=value; best=p; }
  }
  const Pose center=best;
  for (int a=-6; a<=6; ++a) for (int x=-8; x<=8; ++x) for (int y=-8; y<=8; ++y) {
    Pose p{center[0]+x*.0025, center[1]+y*.0025, center[2]+a*.1*degree};
    const double value = score(residuals(tree, points, p));
    if (value < best_score) { best_score=value; best=p; }
  }
  return best;
}
int main(int argc, char **argv) {
  try {
    if (argc!=5 && argc!=6) throw std::runtime_error("register_stationary MAP_PGM MAP_META_JSON SNAPSHOT_JSON NEW_RESULT_JSON [TRAIN_RANGE_M]");
    const double train_range=argc==6?std::stod(argv[5]):7.5;
    if (!std::isfinite(train_range) || train_range<1 || train_range>7.5) throw std::runtime_error("Invalid training range");
    if (std::filesystem::exists(argv[4])) throw std::runtime_error("Result must be new");
    Json meta, snapshot;
    std::ifstream(argv[2]) >> meta; std::ifstream(argv[3]) >> snapshot;
    const double resolution=meta.at("resolution"), ox=meta.at("origin")[0], oy=meta.at("origin")[1];
    const double occupied_threshold=meta.at("occupied_thresh");
    if (!std::isfinite(resolution) || resolution<=0 || !std::isfinite(ox) || !std::isfinite(oy) ||
        !std::isfinite(occupied_threshold) || occupied_threshold<=0 || occupied_threshold>=1)
      throw std::runtime_error("Invalid map metadata");
    if (double(meta.at("origin")[2])!=0 || int(meta.at("negate"))!=0) throw std::runtime_error("Unsupported map orientation/negation");
    std::ifstream pgm(argv[1],std::ios::binary);
    if (token(pgm)!="P5") throw std::runtime_error("Require P5 PGM");
    int w=std::stoi(token(pgm)), h=std::stoi(token(pgm));
    if (std::stoi(token(pgm))!=255) throw std::runtime_error("Require 8-bit PGM");
    if (w<=0 || h<=0 || static_cast<long long>(w)*h>100000000) throw std::runtime_error("Invalid map dimensions");
    pgm.get(); std::vector<unsigned char> pixels(static_cast<std::size_t>(w)*h);
    pgm.read(reinterpret_cast<char*>(pixels.data()), pixels.size());
    if (!pgm) throw std::runtime_error("PGM pixels truncated");
    std::vector<Point> occupied;
    for (int y=0;y<h;++y) for (int x=0;x<w;++x)
      if ((255-pixels[y*w+x])/255.0 > double(meta.at("occupied_thresh")))
        occupied.emplace_back(ox+(x+.5)*resolution, oy+(h-1-y+.5)*resolution);
    if (occupied.empty()) throw std::runtime_error("Empty occupied map");
    Tree tree(occupied.begin(), occupied.end());
    Pose seed=snapshot.at("pose").get<Pose>();
    if (!bool(snapshot.at("held")) || !std::isfinite(double(snapshot.at("speed"))) ||
        std::abs(double(snapshot.at("speed")))>.001 ||
        !std::isfinite(snapshot.value("wz", 0.0)) || std::abs(snapshot.value("wz", 0.0))>.01)
      throw std::runtime_error("Only stationary held snapshots supported");
    if(!snapshot.at("scans").is_array() || snapshot.at("scans").size()!=2)
      throw std::runtime_error("Require both lidar scans");
    std::map<std::string,std::vector<Sample>> subsets;
    for (double value : seed) if (!std::isfinite(value)) throw std::runtime_error("Invalid seed");
    const double c=std::cos(seed[2]),s=std::sin(seed[2]);
    for (int sensor=0;sensor<2;++sensor) {
      size_t index=0;
      for (const auto &p:snapshot.at("scans")[sensor]) {
        const double dx=double(p[0])-seed[0],dy=double(p[1])-seed[1];
        Sample q{c*dx+s*dy,-s*dx+c*dy,sensor};
        if (!std::isfinite(q.x) || !std::isfinite(q.y)) throw std::runtime_error("Nonfinite endpoint");
        const double range=std::hypot(q.x,q.y);
        if (range<.3 || range>7.5) continue;
        subsets[sensor==0?"front_all":"rear_all"].push_back(q);
        subsets["all"].push_back(q);
        if (range<=train_range) {
          subsets[index%2==0?"train_near":"heldout_near"].push_back(q);
          subsets[sensor==0?"front_near":"rear_near"].push_back(q);
        } else subsets["heldout_far"].push_back(q);
        ++index;
      }
    }
    if (subsets["front_near"].size()<25 || subsets["rear_near"].size()<25 || subsets["train_near"].size()<50) throw std::runtime_error("Insufficient nearby endpoints");
    const Pose best=fit(tree,subsets["train_near"],seed);
    Json output={{"seed",seed},{"best",best},{"delta",{best[0]-seed[0],best[1]-seed[1],best[2]-seed[2]}},
      {"map_occupied_cells",occupied.size()}, {"map_metadata",meta},
      {"train_range_m",train_range},
      {"method","Bounded XY +/-30cm, yaw +/-6deg coarse search; 2.5mm/0.1deg refinement. Nearest occupied cell center; 10cm capped RMS. Train on alternate endpoints within train_range_m; assess remaining endpoints independently. Stationary snapshot points inverted by its recorded base pose. No ROS or runtime modifications."},
      {"limitation","Map consistency, not surveyed truth; endpoint score alone cannot certify occupancy clearance or unique global localization."}};
    output["source_snapshot"]=snapshot;
    for (const auto &[name,points]:subsets) output["metrics"][name]={{"before",metrics(residuals(tree,points,seed))},{"candidate",metrics(residuals(tree,points,best))}};
    // Independent sensor fits expose extrinsic or map disagreement.
    output["front_best"]=fit(tree,subsets["front_near"],seed);
    output["rear_best"]=fit(tree,subsets["rear_near"],seed);
    std::ofstream out(argv[4]); out << output.dump(2) << '\n'; out.close();
    if (!out) throw std::runtime_error("Could not write result");
    std::cout << output.dump(2) << '\n';
  } catch (const std::exception &e) { std::cerr<<e.what()<<'\n'; return 1; }
}
