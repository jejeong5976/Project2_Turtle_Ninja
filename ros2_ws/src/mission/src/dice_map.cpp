// dice_map.cpp — 경기장 좌표 읽기 구현, 설명은 dice_map.hpp

#include "mission/dice_map.hpp"

#include <cmath>

#include <yaml-cpp/yaml.h>

namespace mission
{

namespace
{
bool read_pose(const YAML::Node & n, Pose2D & out)
{
  if (!n || !n.IsSequence() || n.size() != 3) {return false;}
  out = {n[0].as<double>(), n[1].as<double>(), n[2].as<double>()};
  return true;
}

bool read_xy(const YAML::Node & n, std::array<double, 2> & out)
{
  if (!n || !n.IsSequence() || n.size() != 2) {return false;}
  out = {n[0].as<double>(), n[1].as<double>()};
  return true;
}

double wrap(double a) {return std::atan2(std::sin(a), std::cos(a));}
}  // namespace

Pose2D arena_to_map(const Pose2D & T, const Pose2D & p)
{
  const double c = std::cos(T.yaw), s = std::sin(T.yaw);
  return {T.x + c * p.x - s * p.y, T.y + s * p.x + c * p.y, wrap(T.yaw + p.yaw)};
}

std::array<double, 2> arena_point_to_map(const Pose2D & T, const std::array<double, 2> & p)
{
  const auto q = arena_to_map(T, {p[0], p[1], 0.0});
  return {q.x, q.y};
}

Pose2D inverse_pose(const Pose2D & a)
{
  const double c = std::cos(a.yaw), s = std::sin(a.yaw);
  return {-(c * a.x + s * a.y), -(-s * a.x + c * a.y), wrap(-a.yaw)};
}

Pose2D node_stop(const DiceMap & a, int node)
{
  if (node >= 1 && node <= 6) {return a.boxes[node - 1].stop;}
  if (node >= 7 && node <= 9) {return a.places[node - 7].stop;}
  return a.start;
}

bool load_dice_map(const std::string & path, DiceMap & out, std::string * error)
{
  YAML::Node y;
  try {
    y = YAML::LoadFile(path);
  } catch (const std::exception & e) {
    set_error(error, "dice_map 읽기 실패 " + path + ": " + e.what());
    return false;
  }
  try {
    DiceMap a;
    Pose2D start_arena, barcode_arena;
    if (!read_pose(y["start"], start_arena)) {set_error(error, "start 없음 (x, y, yaw)"); return false;}
    if (!read_pose(y["barcode_view"], barcode_arena)) {set_error(error, "barcode_view 없음 (x, y, yaw)"); return false;}
    const auto mfa = y["map_from_arena"];
    if (mfa && mfa.IsScalar() && mfa.as<std::string>() == "start") {
      a.map_from_arena = inverse_pose(start_arena);
    } else if (!read_pose(mfa, a.map_from_arena)) {
      set_error(error, "map_from_arena 는 start 또는 [x, y, yaw]");
      return false;
    }
    const auto & T = a.map_from_arena;
    a.start = arena_to_map(T, start_arena);
    a.barcode_view = arena_to_map(T, barcode_arena);
    a.box_radius = y["box_radius"].as<double>(a.box_radius);
    a.box_size = y["box_size"].as<double>(a.box_size);
    a.box_approach_m = y["box_approach_m"].as<double>(a.box_approach_m);

    for (int i = 0; i < 6; ++i) {
      const std::string key = "B" + std::to_string(i + 1);
      const auto b = y["boxes"][key];
      std::array<double, 2> c;
      Pose2D st;
      if (!b || !read_xy(b["center"], c) || !read_pose(b["stop"], st)) {
        set_error(error, "boxes." + key + " 에 center [x, y] · stop [x, y, yaw] 필요");
        return false;
      }
      a.boxes[i].center = arena_point_to_map(T, c);
      a.boxes[i].stop = arena_to_map(T, st);
    }
    for (int i = 0; i < 3; ++i) {
      const std::string key = "L" + std::to_string(i + 1);
      const auto p = y["places"][key];
      std::array<double, 2> c;
      Pose2D st;
      if (!p || !read_xy(p["center"], c) || !read_pose(p["stop"], st)) {
        set_error(error, "places." + key + " 에 center [x, y] · stop [x, y, yaw] 필요");
        return false;
      }
      auto & pl = a.places[i];
      pl.center = arena_point_to_map(T, c);
      pl.stop = arena_to_map(T, st);
      pl.view_dir = {std::cos(pl.stop.yaw), std::sin(pl.stop.yaw)};
      pl.top_z = p["top_z"].as<double>(pl.top_z);
    }
    out = a;
    return true;
  } catch (const std::exception & e) {
    set_error(error, "dice_map 형식 오류 " + path + ": " + e.what());
    return false;
  }
}

}  // namespace mission
