#include "lidar_cloud_merge/core.hpp"
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>

// Offline inspection of the stationary revision-2 profile. Does not open a
// device, start a node, publish TF, or infer physical calibration from a bag.
namespace {
using Scan = sensor_msgs::msg::LaserScan;
using Cloud = sensor_msgs::msg::PointCloud2;
using Point = lidar_cloud_merge::Point;
constexpr int64_t max_skew_ns=75000000;
constexpr double position_tolerance=2e-6;  // ARM/x86 projection rounding, metres.

void require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

int64_t stamp(const std_msgs::msg::Header& h) {
  return int64_t(h.stamp.sec)*1000000000+h.stamp.nanosec;
}

template<class Message>
Message decode(const rosbag2_storage::SerializedBagMessage& record) {
  rclcpp::SerializedMessage bytes(*record.serialized_data);
  rclcpp::Serialization<Message> serialization;
  Message message;
  serialization.deserialize_message(&bytes, &message);
  return message;
}

struct Input {
  Scan scan;
  std::vector<Point> kept;
  std::size_t valid=0, close_before=0, close_after=0, beyond30_removed=0, uses=0;
};

struct RecordedCloud {
  Cloud message;
  int64_t received=0;
};

void cadence(const std::string& name, const std::vector<int64_t>& stamps,
             std::ostream& out) {
  require(stamps.size()>1, "Too few messages for cadence: "+name);
  int64_t min_dt=std::numeric_limits<int64_t>::max(), max_dt=0;
  for (std::size_t i=1; i<stamps.size(); ++i) {
    const int64_t dt=stamps[i]-stamps[i-1];
    require(dt>0, "Repeated/backward header stamp: "+name);
    min_dt=std::min(min_dt, dt); max_dt=std::max(max_dt, dt);
  }
  out << name << " count=" << stamps.size()
      << " header_hz=" << (stamps.size()-1)*1e9/(stamps.back()-stamps.front())
      << " min_interval_ms=" << min_dt/1e6 << " max_interval_ms=" << max_dt/1e6
      << " first_ns=" << stamps.front() << " last_ns=" << stamps.back() << '\n';
}

std::array<std::vector<Point>, 2> unpack(const Cloud& cloud,
                                       const lidar_cloud_merge::BodyMask& mask) {
  require(cloud.header.frame_id=="lidar_front_link", "Wrong cloud frame");
  require(cloud.height==1 && cloud.width>0 && cloud.is_dense && !cloud.is_bigendian,
          "Unexpected cloud shape, density or endian");
  require(cloud.point_step==20 && cloud.row_step==cloud.width*20 &&
          cloud.data.size()==cloud.row_step, "Unexpected cloud byte layout");
  const std::array<std::string, 5> names={"x", "y", "z", "intensity", "source_id"};
  require(cloud.fields.size()==names.size(), "Unexpected cloud field count");
  for (std::size_t i=0; i<names.size(); ++i) {
    const auto& field=cloud.fields[i];
    require(field.name==names[i] && field.offset==4*i && field.count==1 &&
            field.datatype==sensor_msgs::msg::PointField::FLOAT32, "Unexpected cloud field");
  }
  std::array<std::vector<Point>, 2> result;
  bool seen_rear=false;
  for (std::size_t i=0; i<cloud.width; ++i) {
    Point point;
    std::memcpy(point.data(), cloud.data.data()+20*i, 20);
    for (float value:point) require(std::isfinite(value), "Nonfinite cloud field");
    require(point[4]==0.f || point[4]==1.f, "Unexpected source_id");
    const bool rear=point[4]==1.f;
    require(rear || !seen_rear, "Source order changed");
    seen_rear|=rear;
    require(!mask.contains(point), "Point survived inside the body box");
    require(std::abs(point[2])<position_tolerance, "Nonplanar point");
    result[rear].push_back(point);
  }
  require(!result[0].empty() && !result[1].empty(), "Missing cloud source");
  return result;
}

double compare(const std::vector<Point>& actual, const std::vector<Point>& expected) {
  require(actual.size()==expected.size(), "Live survivor count differs from raw reconstruction");
  double max_error=0;
  for (std::size_t i=0; i<actual.size(); ++i) {
    require(actual[i][3]==expected[i][3] && actual[i][4]==expected[i][4],
            "Live intensity/source/order differs from raw reconstruction");
    for (std::size_t field=0; field<3; ++field) {
      max_error=std::max(max_error, std::abs(double(actual[i][field])-expected[i][field]));
    }
  }
  require(max_error<=position_tolerance, "Live coordinates/order differ from raw reconstruction");
  return max_error;
}
}  // namespace

int main(int argc, char** argv) {
  try {
    require(argc==3, "Usage: review_live_cloud BAG NEW_OUTPUT_DIRECTORY");
    const uint16_t endian=1;
    require(*reinterpret_cast<const uint8_t*>(&endian)==1, "Reader requires a little-endian host");
    const std::filesystem::path output(argv[2]);
    require(!std::filesystem::exists(output), "Output directory already exists");
    std::array<std::vector<Input>, 2> inputs;
    std::vector<RecordedCloud> clouds;
    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    rosbag2_cpp::Reader reader;
    reader.open(argv[1]);
    while (reader.has_next()) {
      const auto record=reader.read_next();
      if (record->topic_name=="/lidar_front/scan" || record->topic_name=="/lidar_rear/scan") {
        Input input;
        input.scan=decode<Scan>(*record);
        inputs[record->topic_name=="/lidar_rear/scan"].push_back(std::move(input));
      } else if (record->topic_name=="/lidar/points_body_filtered") {
        clouds.push_back({decode<Cloud>(*record), record->recv_timestamp});
      } else if (record->topic_name=="/tf_static") {
        const auto message=decode<tf2_msgs::msg::TFMessage>(*record);
        transforms.insert(transforms.end(), message.transforms.begin(), message.transforms.end());
      } else {
        throw std::runtime_error("Unexpected topic: "+record->topic_name);
      }
    }
    require(transforms.size()==1, "Expected exactly one recorded static transform");
    const auto& recorded_tf=transforms.front();
    require(recorded_tf.header.frame_id=="lidar_front_link" &&
            recorded_tf.child_frame_id=="lidar_rear_link", "Unexpected TF frame relationship");
    const auto& t=recorded_tf.transform;
    require(std::abs(t.translation.x-.220)<1e-10 && std::abs(t.translation.y)<1e-10 &&
            std::abs(t.translation.z)<1e-10 && std::abs(t.rotation.x)<1e-10 &&
            std::abs(t.rotation.y)<1e-10 && std::abs(std::abs(t.rotation.z)-1)<1e-10 &&
            std::abs(t.rotation.w)<1e-10, "TF differs from accepted relative geometry");
    lidar_cloud_merge::BodyMask mask;
    mask.length=.2552; mask.width=.3050; mask.margin=.010;
    mask.validate();
    std::filesystem::create_directories(output);
    std::ofstream summary(output/"summary.txt"), pairs(output/"pairs.csv"), points(output/"cloud_points.csv");
    require(summary.good() && pairs.good() && points.good(), "Cannot create output files");
    summary << std::setprecision(12)
            << "Profile: center(.110,.005), length .2552, width .305, margin .010; all Z.\n"
            << "Recorded TF front -> rear: translation " << t.translation.x << ',' << t.translation.y
            << ',' << t.translation.z << " quaternion " << t.rotation.x << ',' << t.rotation.y
            << ',' << t.rotation.z << ',' << t.rotation.w << '\n';
    for (std::size_t side=0; side<2; ++side) {
      const std::string name=side ? "rear" : "front";
      std::vector<int64_t> stamps;
      std::size_t valid=0, kept=0, close_before=0, close_after=0, beyond30_removed=0;
      for (auto& input:inputs[side]) {
        const auto& scan=input.scan;
        require(scan.header.frame_id=="lidar_"+name+"_link", "Unexpected scan frame");
        require(scan.ranges.size()==400 && scan.intensities.size()==400 && scan.time_increment==0,
                "Unexpected scan array/timing contract");
        stamps.push_back(stamp(scan.header));
        geometry_msgs::msg::Transform transform;
        transform.rotation.w=1.;
        if (side) transform=t;
        lidar_cloud_merge::append(scan, transform, float(side), input.kept);
        input.valid=input.kept.size();
        std::size_t index=0;
        for (float range:scan.ranges) {
          if (!std::isfinite(range) || range<scan.range_min || range>scan.range_max) continue;
          const bool excluded=mask.contains(input.kept[index++]);
          input.close_before+=range<.30;
          input.close_after+=range<.30 && !excluded;
          input.beyond30_removed+=range>=.30 && excluded;
        }
        lidar_cloud_merge::exclude_body(input.kept, mask);
        valid+=input.valid; kept+=input.kept.size(); close_before+=input.close_before;
        close_after+=input.close_after; beyond30_removed+=input.beyond30_removed;
      }
      cadence(name, stamps, summary);
      summary << name << " raw_valid=" << valid << " reconstructed_removed=" << valid-kept
              << " reconstructed_kept=" << kept << " below30_before=" << close_before
              << " below30_after=" << close_after << " removed_at_or_beyond30=" << beyond30_removed << '\n';
    }
    pairs << "cloud,stamp_ns,front_scan,rear_scan,front_points,rear_points,skew_ms,receipt_minus_header_ms,status\n";
    points << "cloud,source,x_m,y_m,z_m,intensity\n" << std::setprecision(9);
    std::vector<int64_t> cloud_stamps;
    std::array<std::size_t, 2> source_points={0,0};
    std::size_t complete=0, boundary=0, min_points=std::numeric_limits<std::size_t>::max(), max_points=0;
    std::size_t removed=0, min_removed=std::numeric_limits<std::size_t>::max(), max_removed=0;
    double max_error=0, min_skew=1e9, max_skew=0, sum_skew=0;
    double min_delay=1e9, max_delay=-1e9, sum_delay=0;
    for (std::size_t i=0; i<clouds.size(); ++i) {
      const auto& cloud=clouds[i];
      const int64_t time=stamp(cloud.message.header);
      cloud_stamps.push_back(time);
      const auto actual=unpack(cloud.message, mask);
      std::array<int64_t, 2> match={-1,-1};
      for (std::size_t side=0; side<2; ++side) {
        for (std::size_t j=0; j<inputs[side].size(); ++j) {
          const int64_t input_time=stamp(inputs[side][j].scan.header);
          if (input_time>time || time-input_time>max_skew_ns) continue;
          require(match[side]<0, "Ambiguous scan timestamps in the pairing window");
          match[side]=int64_t(j);
        }
        if (match[side]>=0) {
          auto& input=inputs[side][std::size_t(match[side])];
          max_error=std::max(max_error, compare(actual[side], input.kept));
          require(++input.uses==1, "Recorded scan reused in multiple clouds");
        } else {
          // An absent source can only be classified as a recording boundary
          // when the cloud is at an edge and its entire candidate interval lies
          // outside that source's captured header interval. Its points remain
          // unverified against raw data; do not substitute a neighbouring scan.
          const int64_t first=stamp(inputs[side].front().scan.header);
          const int64_t last=stamp(inputs[side].back().scan.header);
          require((i==0 && time<first) || (i+1==clouds.size() && time-max_skew_ns>last),
                  "Missing raw input away from a recording boundary");
          summary << "Boundary cloud=" << i << " missing_source=" << side
                  << " cloud_stamp_ns=" << time << " first_raw_stamp_ns=" << first
                  << " last_raw_stamp_ns=" << last << " unmatched_points=" << actual[side].size() << '\n';
        }
        source_points[side]+=actual[side].size();
        for (const auto& p:actual[side]) {
          points << i << ',' << side << ',' << p[0] << ',' << p[1] << ',' << p[2] << ',' << p[3] << '\n';
        }
      }
      min_points=std::min(min_points, std::size_t(cloud.message.width));
      max_points=std::max(max_points, std::size_t(cloud.message.width));
      const double delay=(cloud.received-time)/1e6;
      min_delay=std::min(min_delay,delay); max_delay=std::max(max_delay,delay); sum_delay+=delay;
      pairs << i << ',' << time << ',' << match[0] << ',' << match[1] << ','
            << actual[0].size() << ',' << actual[1].size() << ',';
      if (match[0]>=0 && match[1]>=0) {
        const auto& front=inputs[0][std::size_t(match[0])];
        const auto& rear=inputs[1][std::size_t(match[1])];
        const int64_t a=stamp(front.scan.header), b=stamp(rear.scan.header);
        require(time==std::max(a,b), "Cloud header is not the later input header");
        const double skew=std::abs(a-b)/1e6;
        min_skew=std::min(min_skew,skew); max_skew=std::max(max_skew,skew); sum_skew+=skew;
        const std::size_t excluded=front.valid+rear.valid-cloud.message.width;
        removed+=excluded; min_removed=std::min(min_removed,excluded); max_removed=std::max(max_removed,excluded);
        ++complete;
        pairs << skew << ',' << delay << ",matched\n";
      } else {
        ++boundary;
        pairs << ',' << delay << ",raw_missing_at_boundary\n";
      }
    }
    cadence("cloud", cloud_stamps, summary);
    require(complete>0, "No complete pair reconstructed");
    for (std::size_t side=0; side<2; ++side) {
      std::size_t unused=0;
      for (std::size_t j=0; j<inputs[side].size(); ++j) {
        const auto& input=inputs[side][j];
        if (input.uses) continue;
        ++unused;
        summary << "Unused raw source=" << side << " scan=" << j
                << " stamp_ns=" << stamp(input.scan.header)
                << " valid=" << input.valid << " kept=" << input.kept.size() << '\n';
      }
      summary << "source=" << side << " live_points=" << source_points[side] << " unused_raw_scans=" << unused << '\n';
    }
    summary << "complete_pairs=" << complete << " partial_boundary_clouds=" << boundary
            << " max_coordinate_error_m=" << max_error << '\n'
            << "points_per_cloud_min=" << min_points
            << " mean=" << double(source_points[0]+source_points[1])/clouds.size() << " max=" << max_points << '\n'
            << "removed_per_complete_pair_min=" << min_removed << " mean=" << double(removed)/complete
            << " max=" << max_removed << '\n'
            << "matched_pair_header_skew_ms_min=" << min_skew << " mean=" << sum_skew/complete << " max=" << max_skew << '\n'
            << "bag_receipt_minus_cloud_header_ms_min=" << min_delay << " mean=" << sum_delay/clouds.size() << " max=" << max_delay << '\n'
            << "PASS recorded profile/TF/layout/bounds/source order/survivors/nonreuse/header progression.\n"
            << "Missing boundary-source points have layout/bounds checks only, not raw reconstruction.\n"
            << "Header skew/receipt age do not establish acquisition timing, synchrony, deskew or motion readiness.\n";
    summary.close(); pairs.close(); points.close();
    require(summary.good() && pairs.good() && points.good(), "Output write failed");
    std::cout << "PASS live cloud review: " << output << '\n';
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
