// dice.cpp — 주사위 기억 미션 계산 구현, 설명은 dice.hpp

#include "mission/dice.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <numeric>

#include "mission/common.hpp"

namespace mission
{

namespace
{

// 앞뒤 공백 제거
std::string trim(const std::string & s)
{
  const auto b = s.find_first_not_of(" \t\r\n");
  if (b == std::string::npos) {return "";}
  const auto e = s.find_last_not_of(" \t\r\n");
  return s.substr(b, e - b + 1);
}

}  // namespace

int SignalCounts::of(char group) const
{
  switch (group) {
    case 'R': return red;
    case 'Y': return yellow;
    case 'G': return green;
    default: return 0;
  }
}

std::optional<DiceOrder> parse_dice_barcode(const std::string & raw_text, std::string * error)
{
  const std::string t = trim(raw_text);
  // 발제 p.3: 기존 바코드 4종, 두 개씩 같은 의미
  if (t == "P2-01" || t == "P2-03") {return DiceOrder::Ascending;}
  if (t == "P2-02" || t == "P2-04") {return DiceOrder::Descending;}
  set_error(error, t.empty() ? "빈 바코드" : "미등록 바코드: " + t);
  return std::nullopt;
}

bool valid_signal_counts(const SignalCounts & c, const DiceConfig & cfg, std::string * error)
{
  for (char g : {'R', 'Y', 'G'}) {
    const int n = c.of(g);
    if (n < cfg.min_count || n > cfg.max_count) {
      set_error(error, std::string("신호 ") + g + " 횟수 " + std::to_string(n) + " (허용 " +
        std::to_string(cfg.min_count) + "–" + std::to_string(cfg.max_count) + ")");
      return false;
    }
  }
  if (c.total() != cfg.total_pucks) {
    set_error(error, "신호 합 " + std::to_string(c.total()) + " ≠ " + std::to_string(cfg.total_pucks));
    return false;
  }
  return true;
}

std::array<char, 3> sort_dice_groups(const SignalCounts & c, DiceOrder order)
{
  std::array<char, 3> g{'R', 'Y', 'G'};   // 동률 기준 순서
  // stable_sort → 같은 횟수면 R → Y → G 그대로 (오름 · 내림 둘 다)
  std::stable_sort(g.begin(), g.end(), [&](char a, char b) {
      return order == DiceOrder::Ascending ? c.of(a) < c.of(b) : c.of(a) > c.of(b);
    });
  return g;
}

std::vector<std::array<double, 2>> dice_pip_offsets(int pips, double p)
{
  // 격자 칸 (열 x, 행 y), x: −1 왼쪽 · +1 오른쪽, y: +1 위(+Y) · −1 아래
  using C = std::array<int, 2>;
  static const std::map<int, std::vector<C>> kPips{
    {1, {{0, 0}}},
    {2, {{-1, 1}, {1, -1}}},                                   // 왼쪽 위 · 오른쪽 아래
    {3, {{-1, 1}, {0, 0}, {1, -1}}},
    {4, {{-1, 1}, {1, 1}, {-1, -1}, {1, -1}}},
    {5, {{-1, 1}, {1, 1}, {0, 0}, {-1, -1}, {1, -1}}},
    {6, {{-1, 1}, {-1, 0}, {-1, -1}, {1, 1}, {1, 0}, {1, -1}}},   // 좌우 두 열
  };
  std::vector<std::array<double, 2>> out;
  const auto it = kPips.find(pips);
  if (it == kPips.end()) {return out;}
  for (const auto & c : it->second) {out.push_back({c[0] * p, c[1] * p});}
  return out;
}

std::optional<DicePlan> dice_layout(
  const SignalCounts & counts, DiceOrder order, const DiceConfig & cfg, std::string * error)
{
  if (!valid_signal_counts(counts, cfg, error)) {return std::nullopt;}
  DicePlan plan;
  plan.order = order;
  const auto groups = sort_dice_groups(counts, order);
  for (int i = 0; i < 3; ++i) {
    plan.places[i].group = groups[i];
    plan.places[i].pips = counts.of(groups[i]);
    plan.places[i].pip_offsets = dice_pip_offsets(plan.places[i].pips, cfg.pip_pitch_m);
  }
  return plan;
}

void order_far_first(DicePlace & place, double ax, double ay)
{
  // 접근 방향 쪽 투영이 작을수록 로봇에서 멂 → 그 점부터
  std::vector<size_t> idx(place.pip_offsets.size());
  std::iota(idx.begin(), idx.end(), 0);
  std::stable_sort(idx.begin(), idx.end(), [&](size_t a, size_t b) {
      const auto & pa = place.pip_offsets[a];
      const auto & pb = place.pip_offsets[b];
      return pa[0] * ax + pa[1] * ay < pb[0] * ax + pb[1] * ay - 1e-9;
    });
  std::vector<std::array<double, 2>> offs;
  for (size_t i : idx) {offs.push_back(place.pip_offsets[i]);}
  place.pip_offsets = std::move(offs);
  // 퍽 ID 는 그룹 안에서 아무 점에나 놓아도 됨 → 순서는 그대로 두고 점만 재정렬
}

std::vector<std::array<double, 2>> pips_view_to_map(
  const std::vector<std::array<double, 2>> & picture, double fx, double fy)
{
  const double n = std::hypot(fx, fy);
  if (!(n > 1e-9)) {return picture;}
  fx /= n;
  fy /= n;
  const double rx = fy, ry = -fx;   // 보는 사람의 오른쪽
  std::vector<std::array<double, 2>> out;
  for (const auto & p : picture) {out.push_back({p[0] * rx + p[1] * fx, p[0] * ry + p[1] * fy});}
  return out;
}

void apply_view_directions(DicePlan & plan, const std::array<std::array<double, 2>, 3> & view_dir)
{
  for (size_t i = 0; i < plan.places.size(); ++i) {
    auto & pl = plan.places[i];
    pl.pip_offsets = pips_view_to_map(pl.pip_offsets, view_dir[i][0], view_dir[i][1]);
    order_far_first(pl, -view_dir[i][0], -view_dir[i][1]);
  }
}

const char * dice_order_name(DiceOrder o)
{
  return o == DiceOrder::Ascending ? "ascending" : "descending";
}

}  // namespace mission
