// task_parser.cpp — 바코드 원문 → Task 구현, 설명은 task_parser.hpp

#include "mission/task_parser.hpp"

#include "mission/dice.hpp"

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

std::optional<Task> parse_task(const std::string & raw_text, std::string * error)
{
  const std::string text = trim(raw_text);
  const auto order = parse_dice_barcode(text, error);   // 빈 바코드 · 미등록 바코드는 여기서 걸러짐
  if (!order) {return std::nullopt;}
  Task t;
  t.task_id = text;
  t.dice_order = *order;
  t.raw_text = text;
  return t;
}

}  // namespace mission
