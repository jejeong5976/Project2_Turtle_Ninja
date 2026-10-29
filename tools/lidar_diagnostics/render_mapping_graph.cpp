// Offline rendering of a serialized slam_toolbox graph. No ROS node, solver,
// scan processing, parameter changes, serial access or hardware commands.
#include <karto_sdk/Mapper.h>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

namespace fs = std::filesystem;
void require(bool valid, const std::string & message) {
  if (!valid) {throw std::runtime_error(message);}
}

int main(int argc, char ** argv) {
  try {
    require(argc == 5, "Usage: render_mapping_graph GRAPH_PREFIX NEW_OUTPUT MIN_PASS_THROUGH OCCUPANCY_THRESHOLD");
    const std::string prefix = argv[1];
    const fs::path output = argv[2];
    size_t consumed = 0;
    const int minimum = std::stoi(argv[3], &consumed);
    require(consumed == std::string(argv[3]).size() && minimum >= 0 && minimum <= 1000,
      "Invalid minimum pass-through count");
    const double threshold = std::stod(argv[4], &consumed);
    require(consumed == std::string(argv[4]).size() && std::isfinite(threshold) &&
      threshold > 0 && threshold < 1, "Invalid occupancy threshold");
    require(fs::is_regular_file(prefix + ".posegraph") && fs::is_regular_file(prefix + ".data"),
      "Both graph and dataset files are required");
    require(fs::create_directory(output), "Output directory must be new");

    // The two archives own separate objects. Destroy mapper before dataset.
    karto::Dataset dataset;
    karto::Mapper mapper;
    mapper.LoadFromFile(prefix + ".posegraph");
    dataset.LoadFromFile(prefix + ".data");
    for (auto * object : dataset.GetLasers()) {
      auto * laser = dynamic_cast<karto::LaserRangeFinder *>(object);
      require(laser != nullptr, "Dataset contains a non-laser sensor");
      karto::SensorManager::GetInstance()->RegisterSensor(laser, true);
    }
    const auto scans = mapper.GetAllProcessedScans();
    require(!scans.empty() && scans.size() <= 20000 && mapper.GetGraph(), "Missing or excessive graph");
    std::ofstream poses(output / "graph_poses.csv");
    poses << std::setprecision(17) << "id,time_s,x_m,y_m,yaw_rad,odom_x_m,odom_y_m,odom_yaw_rad\n";
    for (const auto * scan : scans) {
      const auto & p = scan->GetCorrectedPose();
      const auto & o = scan->GetOdometricPose();
      require(std::isfinite(p.GetX()) && std::isfinite(p.GetY()) && std::isfinite(p.GetHeading()),
        "Nonfinite corrected pose");
      poses << scan->GetUniqueId() << ',' << scan->GetTime() << ',' << p.GetX() << ',' << p.GetY()
            << ',' << p.GetHeading() << ',' << o.GetX() << ',' << o.GetY() << ',' << o.GetHeading() << '\n';
    }
    std::ofstream report(output / "comparison.txt");
    report << std::setprecision(17) << "Serialized optimized graph; no new optimization or scan matching.\n"
           << "graph_prefix=" << prefix << "\nscans=" << scans.size()
           << " edges=" << mapper.GetGraph()->GetEdges().size()
           << " lasers=" << dataset.GetLasers().size()
           << " min_pass_through=" << minimum << " occupancy_threshold=" << threshold << '\n';
    // Occupancy thresholds are explicit: this Karto archive does not serialize them.
    for (const double resolution : {.05, .025}) {
      const std::string name = resolution == .05 ? "graph_5cm" : "graph_2p5cm";
      std::unique_ptr<karto::OccupancyGrid> grid(karto::OccupancyGrid::CreateFromScans(
        scans, resolution, static_cast<kt_int32u>(minimum), threshold));
      require(bool(grid), "Could not render graph");
      const auto origin = grid->GetCoordinateConverter()->GetOffset();
      size_t occupied = 0, free = 0, unknown = 0;
      std::ofstream image(output / (name + ".pgm"), std::ios::binary);
      image << "P5\n" << grid->GetWidth() << ' ' << grid->GetHeight() << "\n255\n";
      for (int y = grid->GetHeight() - 1; y >= 0; --y) {
        for (int x = 0; x < grid->GetWidth(); ++x) {
          const auto value = grid->GetValue({x, y});
          char pixel = char(205);
          if (value == karto::GridStates_Occupied) {pixel = 0; ++occupied;}
          else if (value == karto::GridStates_Free) {pixel = char(254); ++free;}
          else {++unknown;}
          image.put(pixel);
        }
      }
      image.close(); require(bool(image), "Image write failed");
      std::ofstream yaml(output / (name + ".yaml"));
      yaml << std::setprecision(17) << "image: " << name << ".pgm\nmode: trinary\nresolution: " << resolution
           << "\norigin: [" << origin.GetX() << ", " << origin.GetY()
           << ", 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n";
      yaml.close(); require(bool(yaml), "Metadata write failed");
      report << name << " width=" << grid->GetWidth() << " height=" << grid->GetHeight()
             << " occupied=" << occupied << " free=" << free << " unknown=" << unknown
             << " origin_x=" << origin.GetX() << " origin_y=" << origin.GetY() << '\n';
    }
    poses.close(); report.close(); require(bool(poses) && bool(report), "Report write failed");
    std::cout << "Rendered saved graph into " << output << '\n';
    return 0;
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n'; return 1;
  }
}
