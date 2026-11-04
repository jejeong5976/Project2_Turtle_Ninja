#pragma once
#include <yaml-cpp/yaml.h>
#include <string>
namespace body_route_controller
{
struct TrialSettings
{
  std::string mode = "custom";
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
