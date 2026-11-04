#include "common.hpp"
#include <iostream>

using namespace maze_sim;
int main(int argc, char **argv) {
  try {
    if (argc != 4) throw std::runtime_error("import_scenario MAPS NEW_OUTPUT l2|maze|scenario|GOALS_FILE");
    const fs::path source = fs::canonical(argv[1]), out = fs::absolute(argv[2]);
    const std::string route = argv[3];
    auto report = YAML::LoadFile((source / "scenario.yaml").string());
    const auto start = pose(report["start"]);
    auto truth = Grid::read(source / "truth.yaml");
    auto navigation = Grid::read(source / "navigation.yaml");
    auto nominal = Grid::read(source / "nominal.yaml");
    for (const auto *g : {&navigation, &nominal}) {
      if (g->pixels.size() != truth.pixels.size() || g->resolution != truth.resolution ||
          g->ox != truth.ox || g->oy != truth.oy) throw std::runtime_error("map registrations differ");
    }
    if (report["ground_truth_complete"].as<bool>(false) && cv::countNonZero(truth.pixels == 205))
      throw std::runtime_error("complete world contains unknown cells");
    std::vector<Pose> goals;
    if (report["available_routes"] && report["available_routes"][route]) {
      for (const auto &n : report["available_routes"][route]) goals.push_back(pose(n));
    } else if (route == "scenario" || route == report["route"].as<std::string>()) {
      for (const auto &n : report["goals"]) goals.push_back(pose(n));
    } else if (route == "l2" && report["route"].as<std::string>() == "maze") {
      goals.push_back(pose(report["goals"][0]));
    } else {
      std::ifstream input(route);
      if (!input) throw std::runtime_error("unknown route; use scenario, an included route, or a goals file");
      std::string line;
      while (std::getline(input, line)) {
        line = line.substr(0, line.find('#'));
        if (line.find_first_not_of(" \t\r") == std::string::npos) continue;
        std::istringstream row(line); Pose p; std::string extra;
        if (!(row >> p.x >> p.y >> p.yaw) || (row >> extra) || !std::isfinite(p.x) ||
            !std::isfinite(p.y) || !std::isfinite(p.yaw)) throw std::runtime_error("invalid goals file");
        goals.push_back(p);
      }
    }
    if (goals.empty() || goals.size() > 100) throw std::runtime_error("empty or oversized route");
    auto cost = truth.costmap();
    if (!body_route_controller::footprintSafe(cost, footprint(), start)) throw std::runtime_error("blocked start");
    for (const auto &g : goals)
      if (!body_route_controller::footprintSafe(cost, footprint(), g)) throw std::runtime_error("blocked goal");
    if (!fs::create_directory(out)) throw std::runtime_error("output directory must be new");
    truth.save(out, "truth"); navigation.save(out, "navigation"); nominal.save(out, "nominal");
    report["prepared_source"] = source.string();
    report["route"] = route;
    report["goals"] = YAML::Node(YAML::NodeType::Sequence);
    std::ofstream file(out / "goals.path"); file << std::setprecision(17);
    for (const auto &p : goals) {
      report["goals"].push_back(poseNode(p));
      file << p.x << ' ' << p.y << ' ' << p.yaw << '\n';
    }
    if (!file) throw std::runtime_error("goal write failed");
    saveYaml(out / "scenario.yaml", report);
    Canvas a(navigation, report["view_bounds"]), b(truth, report["view_bounds"]);
    a.text("Navigation input (purple = unknown)", 22);
    b.text("Simulation world: sensor and evaluation input", 22);
    for (auto *canvas : {&a, &b}) {
      canvas->body(start, {180, 80, 10});
      for (const auto &g : goals) canvas->body(g, {30, 120, 220});
    }
    cv::Mat comparison; cv::hconcat(a.image, b.image, comparison);
    if (!cv::imwrite((out / "layouts.png").string(), comparison)) throw std::runtime_error("preview write failed");
    std::cout << "Imported " << source << "; route=" << route << "; goals=" << goals.size() << '\n';
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "IMPORT FAILED: " << e.what() << '\n'; return 2;
  }
}
