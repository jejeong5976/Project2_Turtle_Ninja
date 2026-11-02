// dice_verify.cpp — 재검증 판정 구현, 설명은 dice_verify.hpp

#include "mission/dice_verify.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <set>
#include <sstream>

namespace mission
{

namespace
{
std::string place_name(int p) {return "L" + std::to_string(p + 1);}

std::string fmt(double v, int digits = 3)
{
  char b[32];
  std::snprintf(b, sizeof(b), "%.*f", digits, v);
  return b;
}
// 퍽 하나의 장별 읽기 모음
struct ObsVotes
{
  VerifyObs last;                       // 가장 최근 장 (위치 · ID)
  std::map<std::string, int> color, shape;

  void add(const VerifyObs & o)
  {
    const std::string id = last.id.empty() ? o.id : last.id;
    last = o;
    last.id = id;
    if (!o.color.empty()) {++color[o.color];}
    if (!o.shape.empty()) {++shape[o.shape];}
  }

  void absorb(const ObsVotes & other)
  {
    for (const auto & [k, n] : other.color) {color[k] += n;}
    for (const auto & [k, n] : other.shape) {shape[k] += n;}
  }
};

// 다수결: 1 등 비율 >= ratio 면 그 값, 아니면 ""
std::string vote(const std::map<std::string, int> & counts, double ratio)
{
  int total = 0, best_n = 0;
  std::string best;
  for (const auto & [k, n] : counts) {
    total += n;
    if (n > best_n) {best_n = n; best = k;}
  }
  if (total == 0) {return "";}
  return static_cast<double>(best_n) / total >= ratio ? best : "";
}
}  // namespace

std::vector<VerifyObs> group_place_obs(const std::vector<VerifyObs> & raw, double merge_m, double vote_ratio)
{
  std::map<std::string, ObsVotes> by_id;
  std::vector<ObsVotes> anon;
  for (const auto & o : raw) {
    if (!o.id.empty()) {
      by_id[o.id].add(o);
      continue;
    }
    bool merged = false;
    for (auto & a : anon) {
      if (std::hypot(a.last.x - o.x, a.last.y - o.y) < merge_m) {a.add(o); merged = true; break;}
    }
    if (!merged) {
      anon.emplace_back();
      anon.back().add(o);
    }
  }
  // ID 없는 퍽이 ID 있는 퍽 가까이면 같은 퍽 → 읽기만 합침
  std::vector<ObsVotes> pucks;
  for (const auto & a : anon) {
    ObsVotes * near = nullptr;
    for (auto & kv : by_id) {
      if (std::hypot(kv.second.last.x - a.last.x, kv.second.last.y - a.last.y) < merge_m) {near = &kv.second; break;}
    }
    if (near) {near->absorb(a);} else {pucks.push_back(a);}
  }
  std::vector<VerifyObs> out;
  auto emit = [&](const ObsVotes & v) {
      VerifyObs o = v.last;
      o.color = vote(v.color, vote_ratio);
      o.shape = vote(v.shape, vote_ratio);
      out.push_back(o);
    };
  for (const auto & kv : by_id) {emit(kv.second);}
  for (const auto & p : pucks) {emit(p);}
  return out;
}

PlaceVerify judge_place(
  int place, const DicePlan & plan, const DiceMap & arena, std::vector<VerifyObs> obs,
  double cell_tol_m, double stamp_s)
{
  PlaceVerify pv;
  pv.place = place;
  pv.stamp_s = stamp_s;
  if (place < 0 || place > 2) {
    pv.issues.push_back("장소 번호 잘못됨");
    return pv;
  }
  const auto & dp = plan.places[place];
  pv.group = dp.group;
  pv.expected = dp.pips;
  const auto & c = arena.places[place].center;
  const std::string name = place_name(place);

  // 칸 맞추기: 가까운 쌍부터 (한 칸에 하나)
  struct Pair {double d; size_t o; int k;};
  std::vector<Pair> pairs;
  for (size_t o = 0; o < obs.size(); ++o) {
    for (size_t k = 0; k < dp.pip_offsets.size(); ++k) {
      const double d = std::hypot(obs[o].x - (c[0] + dp.pip_offsets[k][0]), obs[o].y - (c[1] + dp.pip_offsets[k][1]));
      pairs.push_back({d, o, static_cast<int>(k)});
    }
  }
  std::sort(pairs.begin(), pairs.end(), [](const Pair & a, const Pair & b) {return a.d < b.d;});
  std::vector<bool> obs_done(obs.size(), false), cell_done(dp.pip_offsets.size(), false);
  for (auto & o : obs) {o.cell_error_m = 1e9;}
  for (const auto & p : pairs) {
    obs[p.o].cell_error_m = std::min(obs[p.o].cell_error_m, p.d);
    if (obs_done[p.o] || cell_done[p.k] || p.d > cell_tol_m) {continue;}
    obs_done[p.o] = true;
    cell_done[p.k] = true;
    obs[p.o].cell = p.k;
    obs[p.o].cell_error_m = p.d;
  }

  // 그룹 조건. Y 모양 통일은 모양 아는 퍽끼리만 비교함 (모르는 퍽은 그 퍽만 불합격, 다른 퍽까지 떨어뜨리지 않음)
  std::set<std::string> shapes;
  for (const auto & o : obs) {
    if (!o.shape.empty()) {shapes.insert(o.shape);}
  }
  for (auto & o : obs) {
    bool g = false;
    if (pv.group == 'R') {g = o.color == "red";}
    if (pv.group == 'G') {g = o.color == "green";}
    if (pv.group == 'Y') {g = !o.shape.empty() && shapes.size() == 1;}
    o.ok = o.cell >= 0 && g;
    if (o.cell < 0) {
      pv.issues.push_back(name + " 퍽 " + (o.id.empty() ? "(ID 모름)" : o.id) + " 칸 밖 (가장 가까운 칸 " +
        fmt(o.cell_error_m * 1000, 0) + " mm)");
    } else if (pv.group == 'Y' && o.shape.empty()) {
      pv.issues.push_back(name + " 퍽 " + (o.id.empty() ? "(ID 모름)" : o.id) + " 모양 모름 (그룹 Y 확인 못 함)");
    } else if (!g) {
      pv.issues.push_back(name + " 퍽 " + (o.id.empty() ? "(ID 모름)" : o.id) + " 그룹 " + pv.group + " 조건 안 맞음 (" +
        o.color + "/" + o.shape + ")");
    }
    if (o.id.empty()) {pv.issues.push_back(name + " ID 못 읽은 퍽 있음");}
  }
  if (static_cast<int>(obs.size()) != pv.expected) {
    pv.issues.push_back(name + " 개수 " + std::to_string(obs.size()) + " ≠ 눈 " + std::to_string(pv.expected));
  }
  pv.seen = std::move(obs);
  pv.ok = pv.issues.empty();
  return pv;
}

VerifyResult summarize(const std::vector<PlaceVerify> & places, int total_ids)
{
  VerifyResult r;
  r.places = places;
  std::set<int> done;
  std::map<std::string, int> count;
  for (const auto & p : places) {
    done.insert(p.place);
    for (const auto & o : p.seen) {
      if (!o.id.empty()) {++count[o.id];}
    }
    r.issues.insert(r.issues.end(), p.issues.begin(), p.issues.end());
  }
  r.complete = done.size() == 3;
  for (const auto & [id, n] : count) {
    if (n > 1) {r.duplicate_ids.push_back(id);}
  }
  if (r.complete) {
    for (int i = 0; i < total_ids; ++i) {
      if (!count.count(std::to_string(i))) {r.missing_ids.push_back(std::to_string(i));}
    }
  }
  for (const auto & id : r.duplicate_ids) {r.issues.push_back("ID " + id + " 두 번 이상 봄");}
  for (const auto & id : r.missing_ids) {r.issues.push_back("ID " + id + " 어디서도 못 봄");}
  r.pass = r.complete && r.issues.empty();
  return r;
}

std::string verify_to_yaml(const VerifyResult & r, const DicePlan & plan, const DiceMap & arena,
  const std::string & barcode, const std::string & signal_sequence)
{
  std::ostringstream y;
  y << "# 재검증 기록 (planning dice_verify). 관측 = VERIFY 단계 LOOK 뒤 새 검출, map 프레임 [m]\n";
  y << "barcode: \"" << barcode << "\"\n";
  y << "signal: \"" << signal_sequence << "\"\n";
  y << "status: " << (r.pass ? "PASS" : (r.complete ? "FAIL" : "PENDING")) << "\n";
  y << "places:\n";
  for (const auto & p : r.places) {
    const auto & c = arena.places[p.place].center;
    y << "  - place: " << place_name(p.place) << "\n";
    y << "    group: " << p.group << "\n";
    y << "    pips: " << p.expected << "\n";
    y << "    seen: " << p.seen.size() << "\n";
    y << "    ok: " << (p.ok ? "true" : "false") << "\n";
    y << "    stamp_s: " << fmt(p.stamp_s, 2) << "\n";
    y << "    center: [" << fmt(c[0]) << ", " << fmt(c[1]) << "]\n";
    y << "    cells:     # 계획 칸 (먼 칸부터)\n";
    for (size_t k = 0; k < plan.places[p.place].pip_offsets.size(); ++k) {
      const auto & o = plan.places[p.place].pip_offsets[k];
      y << "      - [" << fmt(c[0] + o[0]) << ", " << fmt(c[1] + o[1]) << "]\n";
    }
    y << "    observed:\n";
    for (const auto & o : p.seen) {
      y << "      - {id: \"" << o.id << "\", color: " << (o.color.empty() ? "\"\"" : o.color) << ", shape: " <<
        (o.shape.empty() ? "\"\"" : o.shape) << ", x: " << fmt(o.x) << ", y: " << fmt(o.y) << ", cell: " << o.cell <<
        ", cell_error_mm: " << fmt(std::min(o.cell_error_m, 9.999) * 1000, 0) << ", ok: " << (o.ok ? "true" : "false") << "}\n";
    }
  }
  auto list = [&y](const char * key, const std::vector<std::string> & v) {
      y << key << ": [";
      for (size_t i = 0; i < v.size(); ++i) {y << (i ? ", " : "") << "\"" << v[i] << "\"";}
      y << "]\n";
    };
  list("duplicate_ids", r.duplicate_ids);
  list("missing_ids", r.missing_ids);
  y << "issues:\n";
  for (const auto & s : r.issues) {y << "  - \"" << s << "\"\n";}
  return y.str();
}

}  // namespace mission
