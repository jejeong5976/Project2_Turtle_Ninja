// dice_record.cpp — signal.json · allocation.json 만들기 구현, 설명은 dice_record.hpp

#include "mission/dice_record.hpp"

#include <cmath>
#include <cstdio>
#include <sstream>

#include "mission/dice_steps.hpp"

namespace mission
{

namespace
{
std::string num(double v, int digits = 3)
{
  if (!std::isfinite(v)) {return "null";}
  char b[40];
  std::snprintf(b, sizeof(b), "%.*f", digits, v);
  return b;
}

// JSON 문자열 (따옴표 · 역슬래시 · 제어 문자만 처리, 한글은 UTF-8 그대로)
std::string str(const std::string & s)
{
  std::string o = "\"";
  for (char c : s) {
    if (c == '"' || c == '\\') {o += '\\'; o += c;} else if (static_cast<unsigned char>(c) < 0x20) {o += ' ';} else {o += c;}
  }
  return o + "\"";
}

std::string str_list(const std::vector<std::string> & v)
{
  std::string o = "[";
  for (size_t i = 0; i < v.size(); ++i) {o += (i ? ", " : "") + str(v[i]);}
  return o + "]";
}

std::string place_name(int p) {return "L" + std::to_string(p + 1);}

// map → 경기장 [mm]
std::array<double, 2> arena_mm(const DiceMap & a, double x, double y)
{
  const auto p = arena_to_map(inverse_pose(a.map_from_arena), {x, y, 0.0});
  return {p.x * 1000.0, p.y * 1000.0};
}
}  // namespace

std::vector<AllocCell> planned_cells(const std::vector<Step> & steps, const DicePlan & plan, const DiceMap & arena)
{
  std::vector<AllocCell> out;
  for (const auto & s : steps) {
    if (s.type != StepType::Arm || s.arm_task != kArmPlace || !is_dice_cell(s)) {continue;}
    AllocCell c;
    c.place = place_of(s);
    c.cell = cell_of(s);
    c.puck_id = s.puck_id;
    if (const auto pt = dice_cell_point(plan, arena, place_of(s), cell_of(s))) {
      c.x = (*pt)[0];
      c.y = (*pt)[1];
    }
    out.push_back(c);
  }
  return out;
}

std::string allocation_to_json(const AllocationRecord & r, const DicePlan & plan, const DiceMap & arena)
{
  std::ostringstream j;
  int filled = 0;
  for (bool f : r.filled) {filled += f ? 1 : 0;}
  j << "{\n";
  j << "  \"kind\": \"allocation\",\n";
  j << "  \"revision\": " << r.revision << ",\n";
  j << "  \"reason\": " << str(r.reason) << ",\n";
  j << "  \"stamp_s\": " << num(r.stamp_s, 2) << ",\n";
  j << "  \"barcode\": " << str(r.barcode) << ",\n";
  j << "  \"order\": " << str(dice_order_name(r.order)) << ",\n";
  j << "  \"signal\": {\"sequence\": " << str(r.signal_sequence) << ", \"R\": " << r.counts.red << ", \"Y\": " <<
    r.counts.yellow << ", \"G\": " << r.counts.green << "},\n";
  j << "  \"yellow_shape\": " << str(r.yellow_shape) << ",\n";
  j << "  \"status\": " << str(filled == 3 ? "complete" : (filled == 0 ? "none" : "partial")) << ",\n";
  j << "  \"places\": [\n";
  for (int p = 0; p < 3; ++p) {
    const auto & dp = plan.places[p];
    const auto & c = arena.places[p].center;
    const auto cm = arena_mm(arena, c[0], c[1]);
    j << "    {\"place\": " << str(place_name(p)) << ", \"group\": " << str(std::string(1, dp.group)) << ", \"pips\": " <<
      dp.pips << ", \"placed\": " << r.placed[p] << ", \"filled\": " << (r.filled[p] ? "true" : "false") <<
      ", \"center_map\": [" << num(c[0]) << ", " << num(c[1]) << "], \"center_arena_mm\": [" << num(cm[0], 0) << ", " <<
      num(cm[1], 0) << "],\n      \"cells\": [";
    bool first = true;
    for (const auto & cell : r.cells) {
      if (cell.place != p) {continue;}
      const auto am = arena_mm(arena, cell.x, cell.y);
      j << (first ? "\n" : ",\n") << "        {\"cell\": " << cell.cell << ", \"puck\": " << str(cell.puck_id) <<
        ", \"map\": [" << num(cell.x) << ", " << num(cell.y) << "], \"arena_mm\": [" << num(am[0], 0) << ", " <<
        num(am[1], 0) << "]}";
      first = false;
    }
    j << (first ? "]}" : "\n      ]}") << (p < 2 ? "," : "") << "\n";
  }
  j << "  ],\n";
  j << "  \"pucks\": [\n";
  for (size_t i = 0; i < r.pucks.size(); ++i) {
    const auto & p = r.pucks[i];
    std::string to;
    for (const auto & c : r.cells) {
      if (c.puck_id == p.id) {to = place_name(c.place) + "#" + std::to_string(c.cell);}
    }
    j << "    {\"id\": " << str(p.id) << ", \"color\": " << str(p.color) << ", \"shape\": " << str(p.shape) <<
      ", \"where\": " << str(p.where) << ", \"to\": " << str(to) << "}" << (i + 1 < r.pucks.size() ? "," : "") << "\n";
  }
  j << "  ],\n";
  std::vector<std::string> vo;
  for (int p : r.verify_order) {vo.push_back(place_name(p));}
  j << "  \"verify_order\": " << str_list(vo) << ",\n";
  j << "  \"expected_remaining_s\": " << num(r.cost_s, 0) << ",\n";
  j << "  \"history\": " << str_list(r.history) << "\n";
  j << "}\n";
  return j.str();
}

std::string signal_to_json(const SignalRecorder & rec, double start_s, bool timed_out)
{
  std::ostringstream j;
  const std::string seq = rec.sequence();
  std::string status = signal_status_name(rec.status());
  if (timed_out && !rec.finished()) {status = "INCOMPLETE";}
  j << "{\n";
  j << "  \"kind\": \"signal\",\n";
  j << "  \"status\": " << str(status) << ",\n";
  j << "  \"sequence\": " << str(seq) << ",\n";
  j << "  \"count\": " << rec.blinks().size() << ",\n";
  j << "  \"R\": " << rec.count('R') << ", \"Y\": " << rec.count('Y') << ", \"G\": " << rec.count('G') <<
    ", \"unknown\": " << rec.count('?') << ",\n";
  j << "  \"blinks\": [\n";
  const auto & b = rec.blinks();
  for (size_t i = 0; i < b.size(); ++i) {
    j << "    {\"n\": " << i + 1 << ", \"color\": " << str(std::string(1, b[i].color)) << ", \"start_s\": " <<
      num(b[i].start - start_s, 2) << ", \"end_s\": " << num(b[i].end - start_s, 2) << "}" << (i + 1 < b.size() ? "," : "") << "\n";
  }
  j << "  ],\n";
  auto issues = rec.issues();
  if (timed_out && !rec.finished()) {issues.push_back("signal_wait_s 안에 기록이 안 끝남");}
  j << "  \"issues\": " << str_list(issues) << ",\n";
  j << "  \"note\": \"시각은 planning 시작 기준 [s]. '?' = 색을 정하지 못한 점멸 (추정해 채우지 않음)\"\n";
  j << "}\n";
  return j.str();
}

}  // namespace mission
