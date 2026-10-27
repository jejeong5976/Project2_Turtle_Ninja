#include "lidar_cloud_merge/core.hpp"
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <string>

// Bounded offline review of the revised FL0 specification (2026-10-27). No ROS node, TF
// publisher, scan pairing, serial access or output bag is created. The actual
// merger's projection and exclusion functions are reused unchanged.
namespace {
struct Stats {
  std::size_t scans=0, bins=0, valid=0, body=0, margin=0, kept=0;
  std::size_t close=0, close_kept=0, beyond30_removed=0;
  std::size_t min_kept=std::numeric_limits<std::size_t>::max(), max_kept=0;
  double nearest_kept=std::numeric_limits<double>::infinity();
};

void require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

geometry_msgs::msg::Transform into_front(bool rear) {
  geometry_msgs::msg::Transform t;
  if (rear) {
    t.translation.x=.220;
    t.rotation.z=1.;
    t.rotation.w=0.;
  } else {
    t.rotation.w=1.;
  }
  return t;
}
}  // namespace

int main(int argc, char** argv) {
  try {
    require(argc==3, "Usage: review_body_mask BAG NEW_OUTPUT_DIRECTORY");
    const std::filesystem::path output(argv[2]);
    require(!std::filesystem::exists(output), "Output directory already exists");
    rosbag2_cpp::Reader reader;
    reader.open(argv[1]);
    std::filesystem::create_directories(output);
    std::ofstream points(output/"points.csv"), scans(output/"scans.csv"), summary(output/"summary.txt");
    std::ofstream geometry(output/"geometry.csv");
    require(points.good() && scans.good() && summary.good() && geometry.good(), "Cannot create output files");
    points << "source,scan,bin,range_m,plate_x_m,plate_y_m,intensity,classification\n" << std::setprecision(9);
    scans << "source,scan,valid,body,margin,kept,close_kept\n";
    // Explicit installed-robot profile; retain the runtime's separate desk defaults.
    lidar_cloud_merge::BodyMask expanded;
    expanded.center_x=.110; expanded.center_y=.005;
    expanded.length=.2552; expanded.width=.3050; expanded.margin=.010;
    auto body=expanded;
    body.margin=0.;
    expanded.validate(); body.validate();
    geometry << "body_center_x,body_center_y,body_length,body_width,body_margin\n"
             << std::setprecision(12) << expanded.center_x << ',' << expanded.center_y << ','
             << expanded.length << ',' << expanded.width << ',' << expanded.margin << '\n';
    std::map<std::string, Stats> totals;
    rclcpp::Serialization<sensor_msgs::msg::LaserScan> serialization;
    while (reader.has_next()) {
      const auto record=reader.read_next();
      const bool rear=record->topic_name=="/lidar_rear/scan";
      if (!rear && record->topic_name!="/lidar_front/scan") continue;
      sensor_msgs::msg::LaserScan scan;
      rclcpp::SerializedMessage serialized(*record->serialized_data);
      serialization.deserialize_message(&serialized, &scan);
      const std::string side=rear ? "rear" : "front";
      require(scan.header.frame_id=="lidar_"+side+"_link", "Unexpected scan frame");
      std::vector<lidar_cloud_merge::Point> projected, expected_kept;
      lidar_cloud_merge::append(scan, into_front(rear), rear ? 1.f : 0.f, projected);
      auto& stats=totals[side];
      std::size_t point_index=0, removed_body=0, removed_margin=0, near_kept=0;
      for (std::size_t bin=0; bin<scan.ranges.size(); ++bin) {
        const double range=scan.ranges[bin];
        if (!std::isfinite(range) || range<scan.range_min || range>scan.range_max) continue;
        require(point_index<projected.size(), "Projection point count mismatch");
        const auto& p=projected[point_index++];
        const double angle=scan.angle_min+bin*double(scan.angle_increment);
        const double local_x=range*std::cos(angle), local_y=range*std::sin(angle);
        // Independent direct projection into the plate datum verifies the
        // front-reference transform, particularly rear yaw and translation.
        const double plate_x=rear ? -.110+local_x : .110-local_x;
        const double plate_y=rear ? .005+local_y : .005-local_y;
        require(std::abs((.110-p[0])-plate_x)<2e-6 &&
                std::abs((.005-p[1])-plate_y)<2e-6, "Independent plate projection mismatch");
        const bool excluded=expanded.contains(p), inside_body=body.contains(p);
        // Only compare classifications away from float32 boundary rounding.
        const double boundary_distance=std::min(std::abs(std::abs(plate_x)-.1376),
                                                std::abs(std::abs(plate_y)-.1625));
        if (boundary_distance>2e-6) {
          require(excluded==(std::abs(plate_x)<=.1376 && std::abs(plate_y)<=.1625),
                  "Independent plate box classification mismatch");
        }
        const char* classification=inside_body ? "body" : excluded ? "margin" : "kept";
        removed_body+=inside_body;
        removed_margin+=excluded && !inside_body;
        stats.close+=range<.30;
        stats.beyond30_removed+=excluded && range>=.30;
        if (!excluded) {
          expected_kept.push_back(p);
          near_kept+=range<.30;
          stats.nearest_kept=std::min(stats.nearest_kept, range);
        }
        points << side << ',' << stats.scans << ',' << bin << ',' << range << ','
               << plate_x << ',' << plate_y << ',' << p[3] << ',' << classification << '\n';
      }
      require(point_index==projected.size(), "Projection accounting mismatch");
      const auto removed=lidar_cloud_merge::exclude_body(projected, expanded);
      require(projected==expected_kept, "Survivor fields or ordering changed");
      require(removed==removed_body+removed_margin, "Filter accounting mismatch");
      scans << side << ',' << stats.scans << ',' << point_index << ',' << removed_body
            << ',' << removed_margin << ',' << projected.size() << ',' << near_kept << '\n';
      ++stats.scans; stats.bins+=scan.ranges.size(); stats.valid+=point_index;
      stats.body+=removed_body; stats.margin+=removed_margin; stats.kept+=projected.size();
      stats.close_kept+=near_kept;
      stats.min_kept=std::min(stats.min_kept, projected.size());
      stats.max_kept=std::max(stats.max_kept, projected.size());
    }
    require(totals.size()==2 && totals.at("front").scans && totals.at("rear").scans,
            "Both scan topics are required");
    summary << "Revised FL0 360-degree steering envelope, 2026-10-27; recorded steering 90 degrees.\n"
            << "Plate frame: front (+.110,+.005), yaw pi; rear (-.110,+.005), yaw 0.\n"
            << "Tight plate bounds X +/- .1276, Y +/- .1525 m.\n"
            << "Expanded plate bounds X +/- .1376, Y +/- .1625 m; all Z.\n"
            << "Actual cloud core reused; independent plate-coordinate and survivor checks pass.\n"
            << "Per-scan geometry only; no simulation of live pair selection or timing.\n"
            << "Removed points are geometrically inside the box, not proven body identities.\n";
    for (const auto& [side, s]:totals) {
      require(s.valid==s.body+s.margin+s.kept, "Total point accounting mismatch");
      summary << std::fixed << std::setprecision(6) << side
              << " scans=" << s.scans << " bins=" << s.bins << " valid=" << s.valid
              << " body=" << s.body << " margin=" << s.margin << " kept=" << s.kept
              << " removed_pct=" << 100.*(s.body+s.margin)/s.valid
              << " kept_per_scan_min=" << s.min_kept << " mean=" << double(s.kept)/s.scans
              << " max=" << s.max_kept << " near30_before=" << s.close
              << " near30_after=" << s.close_kept << " beyond30_removed=" << s.beyond30_removed
              << " nearest_kept_m=" << s.nearest_kept << '\n';
    }
    points.close(); scans.close(); summary.close(); geometry.close();
    require(points.good() && scans.good() && summary.good() && geometry.good(), "Output write failed");
    std::cout << "PASS offline mask review: " << output << '\n';
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
