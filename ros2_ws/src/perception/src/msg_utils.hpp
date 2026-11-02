// msg_utils.hpp — planning 과 약속한 메시지 규칙 모음, docs/planning/planning_interfaces.md
//
// 역할
//   - set_unknown_pose: 위치 모름 = position NaN, 방향 모름 = orientation 0 · 0 · 0 · 0
//   - puck_id_string: AprilTag ID → Detection.id "0"–"11", 퍽 태그가 아니면 ""
//   - traffic_light_code: 판정 문자열 red · yellow · green · off → TrafficLight 상수, 그 밖은 UNKNOWN
//   - puck_label_shape: 클래스 이름 "<색>_<cyl|box>..." → label = 색, shape = cylinder · box · ""
// 입력
//   - geometry_msgs/Pose, AprilTag ID, 신호등 판정 문자열, 클래스 이름
// 출력
//   - 위 규칙대로 채운 메시지 값
// 실행
//   - 헤더 전용, 노드 내부 파이프라인과 barcode 가 씀. 시험 test_common
// note
//   - msg 기본값은 w=1 이라 방향 모름은 w=0 을 직접 넣어야 함
//   - 장소 태그 12–14 는 퍽 ID 로 안 붙임

#pragma once

#include <cstdint>
#include <limits>
#include <string>
#include <utility>

#include <geometry_msgs/msg/pose.hpp>

#include "interfaces/msg/traffic_light.hpp"
#include "perception/common.hpp"

namespace perception
{

inline void set_unknown_pose(geometry_msgs::msg::Pose & p)
{
  constexpr double nan = std::numeric_limits<double>::quiet_NaN();
  p.position.x = p.position.y = p.position.z = nan;
  p.orientation.x = p.orientation.y = p.orientation.z = p.orientation.w = 0.0;
}

// AprilTag ID → Detection.id (퍽 태그가 아니면 "")
inline std::string puck_id_string(int tag_id)
{
  return is_puck_tag(tag_id) ? std::to_string(tag_id) : std::string();
}

inline uint8_t traffic_light_code(const std::string & s)
{
  using interfaces::msg::TrafficLight;
  if (s == "red") {return TrafficLight::RED;}
  if (s == "yellow") {return TrafficLight::YELLOW;}
  if (s == "green") {return TrafficLight::GREEN;}
  if (s == "off") {return TrafficLight::OFF;}
  return TrafficLight::UNKNOWN;
}

// 클래스 이름 "<색>_<cyl|box>[_...]" → (label = 색, shape = cylinder / box / "")
inline std::pair<std::string, std::string> puck_label_shape(const std::string & cls)
{
  const auto us = cls.find('_');
  const std::string color = cls.substr(0, us);
  std::string shape;
  if (us != std::string::npos) {
    const auto rest = cls.substr(us + 1);
    const auto tok = rest.substr(0, rest.find('_'));
    shape = tok == "cyl" ? "cylinder" : (tok == "box" ? "box" : "");
  }
  return {color, shape};
}

}  // namespace perception
