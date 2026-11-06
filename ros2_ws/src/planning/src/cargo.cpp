// cargo.cpp — 적재함 · 그리퍼 재고 기록 구현, 설명은 cargo.hpp

#include "planning/cargo.hpp"
#include "planning/types.hpp"

#include <yaml-cpp/yaml.h>

#include <fstream>

namespace planning
{

namespace
{

YAML::Node item_node(const CargoItem & it)
{
  YAML::Node n;
  n["puck_id"] = it.puck_id;
  n["color"] = it.color;
  n["shape"] = it.shape;
  n["task_id"] = it.task_id;
  n["verified"] = it.verified;
  return n;
}

CargoItem node_item(const YAML::Node & n)
{
  CargoItem it;
  it.puck_id = n["puck_id"].as<std::string>("");
  it.color = n["color"].as<std::string>("");
  it.shape = n["shape"].as<std::string>("");
  it.task_id = n["task_id"].as<std::string>("");
  it.verified = n["verified"].as<bool>(false);
  return it;
}

YAML::Node doubt_node(const CargoDoubt & d)
{
  YAML::Node n;
  n["arm_task"] = static_cast<int>(d.arm_task);
  n["puck_id"] = d.puck_id;
  n["slot"] = d.slot;
  n["step_name"] = d.step_name;
  n["reason"] = d.reason;
  n["item"] = item_node(d.item);
  return n;
}

CargoDoubt node_doubt(const YAML::Node & n)
{
  CargoDoubt d;
  d.arm_task = static_cast<uint8_t>(n["arm_task"].as<int>(0));
  d.puck_id = n["puck_id"].as<std::string>("");
  d.slot = n["slot"].as<std::string>("");
  d.step_name = n["step_name"].as<std::string>("");
  d.reason = n["reason"].as<std::string>("");
  if (n["item"]) {d.item = node_item(n["item"]);}
  return d;
}

}  // namespace

Cargo::Cargo(std::vector<std::string> slot_names)
{
  for (auto & s : slot_names) {slots_.emplace_back(std::move(s), std::nullopt);}
}

std::optional<CargoItem> * Cargo::find_slot(const std::string & slot)
{
  for (auto & s : slots_) {
    if (s.first == slot) {return &s.second;}
  }
  return nullptr;
}

std::optional<std::string> Cargo::free_slot() const
{
  for (const auto & s : slots_) {
    if (!s.second) {return s.first;}
  }
  return std::nullopt;
}

std::size_t Cargo::stored_count() const
{
  std::size_t n = 0;
  for (const auto & s : slots_) {
    if (s.second) {++n;}
  }
  return n;
}

bool Cargo::on_pick(const CargoItem & item, std::string * error)
{
  if (gripper_) {
    set_error(error, "그리퍼에 이미 퍽 " + gripper_->puck_id + " 있음");
    return false;
  }
  gripper_ = item;
  return true;
}

bool Cargo::on_store(const std::string & slot, std::string * error)
{
  auto * s = find_slot(slot);
  if (!s) {
    set_error(error, "없는 칸: " + slot);
    return false;
  }
  if (*s) {
    set_error(error, "칸 " + slot + " 에 이미 퍽 " + (*s)->puck_id + " 있음");
    return false;
  }
  if (!gripper_) {
    set_error(error, "그리퍼가 비어 있음");
    return false;
  }
  *s = gripper_;
  gripper_.reset();
  return true;
}

bool Cargo::on_retrieve(const std::string & slot, std::string * error)
{
  auto * s = find_slot(slot);
  if (!s) {
    set_error(error, "없는 칸: " + slot);
    return false;
  }
  if (!*s) {
    set_error(error, "칸 " + slot + " 이 비어 있음");
    return false;
  }
  if (gripper_) {
    set_error(error, "그리퍼에 이미 퍽 " + gripper_->puck_id + " 있음");
    return false;
  }
  gripper_ = *s;
  s->reset();
  return true;
}

std::optional<CargoItem> Cargo::on_place(std::string * error)
{
  if (!gripper_) {
    set_error(error, "그리퍼가 비어 있음");
    return std::nullopt;
  }
  auto placed = gripper_;
  gripper_.reset();
  return placed;
}

bool Cargo::clear_slot(const std::string & slot)
{
  auto * s = find_slot(slot);
  if (!s) {return false;}
  s->reset();
  return true;
}

bool Cargo::set_doubt(const CargoDoubt & d)
{
  if (doubt_) {return false;}
  doubt_ = d;
  return true;
}

bool Cargo::save(const std::string & path, std::string * error) const
{
  try {
    YAML::Node root;
    YAML::Node slots;
    for (const auto & s : slots_) {
      YAML::Node e;
      e["slot"] = s.first;
      if (s.second) {e["item"] = item_node(*s.second);}
      slots.push_back(e);
    }
    root["slots"] = slots;
    if (gripper_) {root["gripper"] = item_node(*gripper_);}
    if (doubt_) {root["doubt"] = doubt_node(*doubt_);}

    std::ofstream f(path);
    if (!f) {throw std::runtime_error("파일 쓰기 실패: " + path);}
    YAML::Emitter out;
    out << root;
    f << out.c_str() << "\n";
    return true;
  } catch (const std::exception & e) {
    set_error(error, e.what());
    return false;
  }
}

bool Cargo::load(const std::string & path, std::string * error)
{
  try {
    const YAML::Node root = YAML::LoadFile(path);
    const YAML::Node slots = root["slots"];
    if (!slots || !slots.IsSequence() || slots.size() != slots_.size()) {
      throw std::runtime_error("칸 개수가 설정과 다름");
    }
    decltype(slots_) loaded;
    for (std::size_t i = 0; i < slots.size(); ++i) {
      const std::string name = slots[i]["slot"].as<std::string>("");
      if (name != slots_[i].first) {throw std::runtime_error("칸 이름이 설정과 다름: " + name);}
      std::optional<CargoItem> item;
      if (slots[i]["item"]) {item = node_item(slots[i]["item"]);}
      loaded.emplace_back(name, item);
    }
    std::optional<CargoDoubt> doubt;
    if (root["doubt"]) {doubt = node_doubt(root["doubt"]);}   // 재시작해도 불확실 표시 유지
    slots_ = loaded;
    gripper_.reset();
    if (root["gripper"]) {gripper_ = node_item(root["gripper"]);}
    doubt_ = doubt;
    return true;
  } catch (const std::exception & e) {
    set_error(error, e.what());
    return false;
  }
}

}  // namespace planning
