// Read-only replay of the recorder's numeric, unquoted CSV format.
#include "angle_grid.hpp"
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
using namespace lidar_diagnostic;
std::vector<std::string> split(const std::string& line) {
  std::vector<std::string> fields;
  std::stringstream stream(line);
  std::string value;
  while (std::getline(stream,value,',')) fields.push_back(value);
  if (!line.empty() && line.back() == ',') fields.emplace_back();
  return fields;
}
double number(const std::string& value) {
  std::size_t consumed = 0;
  const double result = std::stod(value,&consumed);
  if (consumed != value.size()) throw std::runtime_error("Malformed number");
  return result;
}
std::size_t integer(const std::string& value) {
  const double n = number(value);
  if (!std::isfinite(n) || n < 0 || n > 1000000 || std::floor(n) != n)
    throw std::runtime_error("Invalid integer");
  return static_cast<std::size_t>(n);
}
struct Scan {
  std::size_t count = 0;
  std::vector<Sample> samples;
};
void replay(const std::string& path) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error("Cannot read " + path);
  std::string line;
  if (!std::getline(input,line)) throw std::runtime_error("Empty capture");
  const auto header = split(line);
  std::map<std::string,std::size_t> columns;
  for (std::size_t i=0;i<header.size();++i)
    if (!columns.emplace(header[i],i).second) throw std::runtime_error("Duplicate column");
  for (const auto* key : {"scan_id","raw_count","raw_index","raw_angle_deg",
       "raw_distance_mm","raw_intensity","exposure_flag","reverse"})
    if (!columns.count(key)) throw std::runtime_error(std::string("Missing column ")+key);
  std::map<std::size_t,Scan> scans;
  while (std::getline(input,line)) {
    const auto fields = split(line);
    if (fields.size()!=header.size()) throw std::runtime_error("Malformed CSV row");
    auto field = [&](const char* key)->const std::string& { return fields.at(columns.at(key)); };
    if (integer(field("reverse")) != 0) throw std::runtime_error("Replay expects reverse=false capture");
    const auto id = integer(field("scan_id"));
    if (id >= 1000) throw std::runtime_error("Unexpected scan ID");
    auto& scan = scans[id];
    const auto count = integer(field("raw_count"));
    if (count == 0 || count > 10000 || integer(field("raw_index")) != scan.samples.size())
      throw std::runtime_error("Invalid count or noncontiguous raw index");
    if (scan.samples.empty()) scan.count = count;
    if (scan.count != count || scan.samples.size() >= count) throw std::runtime_error("Inconsistent raw count");
    const auto intensity = integer(field("raw_intensity"));
    const auto exposure = integer(field("exposure_flag"));
    if (exposure > 1 || intensity > 65535) throw std::runtime_error("Invalid intensity/exposure");
    scan.samples.push_back({number(field("raw_angle_deg")), number(field("raw_distance_mm")),
      static_cast<unsigned>(intensity), exposure == 1});
  }
  if (input.bad() || scans.empty()) throw std::runtime_error("Incomplete/empty input");
  std::size_t total=0, rejected=0, collisions=0, occupied=0, zero_selected=0;
  double max_error=0;
  for (const auto& entry : scans) {
    const auto& scan = entry.second;
    if (scan.samples.size()!=scan.count) throw std::runtime_error("Truncated scan");
    const auto grid = convert(scan.samples);
    total += scan.count; rejected += grid.rejected; collisions += grid.collisions;
    std::size_t filled=0;
    for (std::size_t i=0;i<grid.bins.size();++i) {
      const auto& bin=grid.bins[i];
      if (!std::isfinite(bin.range)) continue;
      ++filled;
      if (bin.source == 0) ++zero_selected;
      const auto& source=scan.samples.at(bin.source);
      const double error=std::abs(std::remainder(i*360.0/grid.bins.size()+source.angle_deg,360.0));
      max_error=std::max(max_error,error);
      if (error > 0.45+1e-9 || bin.range != source.distance_mm/1000.0 ||
          bin.range < 0.1 || bin.range > 100)
        throw std::runtime_error("Replay invariant failed");
    }
    if (filled+grid.collisions+grid.rejected != scan.count)
      throw std::runtime_error("Sample accounting failed");
    occupied+=filled;
  }
  std::cout << path << '\n' << "scans=" << scans.size() << " raw=" << total
    << " rejected=" << rejected << " collision_losers=" << collisions
    << " occupied=" << occupied << " empty=" << scans.size()*400-occupied
    << " index_zero_selected=" << zero_selected << '\n'
    << std::fixed << std::setprecision(6) << "max_quantization_error_deg=" << max_error
    << " mean_occupied_bins=" << static_cast<double>(occupied)/scans.size() << '\n';
}
int main(int argc,char** argv) {
  try {
    if (argc<2) throw std::runtime_error("Usage: replay_angles capture.csv [capture.csv ...]");
    for (int i=1;i<argc;++i) replay(argv[i]);
  } catch (const std::exception& e) {
    std::cerr << "FAIL: " << e.what() << '\n'; return 1;
  }
}
