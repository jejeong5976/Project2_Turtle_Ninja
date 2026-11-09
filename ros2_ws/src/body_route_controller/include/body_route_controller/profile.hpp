#pragma once
#include <yaml-cpp/yaml.h>
#include <string>
namespace body_route_controller
{
struct TrialSettings
{
  std::string mode = "custom";
  std::string drive_style = "axis-stop";
  double max_travel_angle_deg = 45.0;
  std::string heading_policy = "reactive";
  double heading_preview_time = 5.0;
  bool prepare_passages = false;
  double passage_position_allowance = .05, passage_yaw_allowance = .08726646259971647;
  double passage_response_delay = .5;
  std::string unknown_policy="blocked";
  double unknown_penalty=2.0, observation_ttl=4.0;
  double speed = .025, yaw_rate = .1, linear_accel = .05, angular_accel = .2;
  double correction_speed = .015;
  void validate() const;
  double measuredSpeed() const;
  double measuredYawRate() const;
};
struct TrialProfile {YAML::Node navigation, drive, trial, manifest;};
TrialProfile makeProfile(const YAML::Node & common, const YAML::Node & original,
  const TrialSettings & settings);
// Stable non-cryptographic identifier for comparison inputs, not authentication.
std::string fingerprint(const std::string & bytes);
std::string readText(const std::string & path);
void writeNew(const std::string & path, const std::string & text);
}
