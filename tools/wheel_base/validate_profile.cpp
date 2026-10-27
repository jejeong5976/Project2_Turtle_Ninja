// Offline calibration/TF/kinematics checks. No ROS nodes or device access.
#include <array>
#include <cmath>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <tinyxml2.h>
#include <yaml-cpp/yaml.h>
#include "hardware/opencr_link.hpp"
#include "swerve_controller/swerve.hpp"

namespace {
constexpr double pi = 3.14159265358979323846;
void require(bool value, const std::string & message)
{
  if (!value) {throw std::runtime_error(message);}
}
void close(double actual, double expected, const std::string & message)
{
  require(std::isfinite(actual) && std::abs(actual - expected) < 1e-9, message);
}
const tinyxml2::XMLElement * named(const tinyxml2::XMLElement * parent,
  const char * type, const std::string & name)
{
  require(parent != nullptr, "missing XML parent for " + name);
  for (auto p = parent->FirstChildElement(type); p; p = p->NextSiblingElement(type)) {
    if (p->Attribute("name") && name == p->Attribute("name")) {return p;}
  }
  throw std::runtime_error("missing XML " + name);
}
std::array<double, 3> xyz(const tinyxml2::XMLElement * joint, const char * attribute)
{
  const auto origin = joint->FirstChildElement("origin");
  require(origin && origin->Attribute(attribute), "missing origin attribute");
  std::istringstream input(origin->Attribute(attribute));
  std::array<double, 3> out{};
  require(static_cast<bool>(input >> out[0] >> out[1] >> out[2]), "bad origin");
  return out;
}
double param(const tinyxml2::XMLElement * joint, const char * key)
{
  const auto p = named(joint, "param", key);
  require(p->GetText(), std::string("empty param ") + key);
  return std::stod(p->GetText());
}
}  // namespace

int main(int argc, char ** argv)
{
  try {
    require(argc == 4 || argc == 5, "Usage: validate_profile CONFIG_DIR OPENCR_URDF CONTROLLER_YAML [MOCK_ODOM_YAML]");
    const std::string config = argv[1];
    const auto hw = YAML::LoadFile(config + "/hardware_map_drive_only.yaml");
    const auto c = YAML::LoadFile(config + "/controllers_drive_only.yaml")["swerve_controller"]["ros__parameters"];
    const auto parameters = YAML::LoadFile(argv[3]);
    const auto resolved = parameters["/swerve_controller"] ?
      parameters["/swerve_controller"]["ros__parameters"] : parameters;
    tinyxml2::XMLDocument doc;
    require(doc.LoadFile(argv[2]) == tinyxml2::XML_SUCCESS, "URDF parse failed");
    const auto robot = doc.RootElement();
    const auto bus = named(robot, "ros2_control", "bus_drive");
    require(bus->NextSiblingElement("ros2_control") == nullptr, "unexpected second bus");
    require(bus->FirstChildElement("sensor") == nullptr, "unexpected uncalibrated sensor");
    const auto hardware = bus->FirstChildElement("hardware");
    require(std::string(hardware->FirstChildElement("plugin")->GetText()) == "hardware/OpenCRHardware", "wrong plugin");
    require(std::string(named(hardware, "param", "torque_off_on_deactivate")->GetText()) == "true", "drive torque-off policy");
    int joints = 0, links = 0;
    for (auto p = bus->FirstChildElement("joint"); p; p = p->NextSiblingElement("joint")) {++joints;}
    for (auto p = robot->FirstChildElement("link"); p; p = p->NextSiblingElement("link")) {++links;}
    require(joints == 8 && links == 10, "unexpected arm/sensor joints or frames");
    close(c["wheel_radius"].as<double>(), 0.033, "measured wheel radius");
    close(c["max_linear"].as<double>(), 0.10, "our linear speed limit");
    close(c["max_angular"].as<double>(), 0.5, "our angular speed limit");
    require(!c["use_imu"].as<bool>() && !c["require_motion_mode"].as<bool>(), "wheel-only dependencies");
    const auto base = named(robot, "joint", "base_joint");
    const auto base_xyz = xyz(base, "xyz"), base_rpy = xyz(base, "rpy");
    close(base_xyz[0], 0, "base X"); close(base_xyz[1], 0, "base Y");
    close(base_xyz[2], 0.033, "wheel-axis height");
    for (double v : base_rpy) {close(v, 0, "base frame yaw must match controller");}

    // Recorded IDs/zeroes, and an independent old-CAD -> calibrated-frame rotation.
    const std::array<std::string, 4> names{{"fl", "fr", "rl", "rr"}};
    const std::array<int, 4> wheel_ids{{1, 2, 41, 42}}, steer_ids{{11, 12, 13, 14}};
    const std::array<int, 4> zeroes{{3066, 1552, 2577, 3589}};
    const std::array<double, 4> wheel_scales{{1, -1, -1, 1}};
    const std::array<double, 4> old_x{{.112, .112, -.112, -.112}}, old_y{{.064, -.064, .064, -.064}};
    const std::array<double, 4> offsets{{.0269, .0269, -.0269, -.0269}};
    std::vector<swerve_controller::ModuleGeometry> geometry;
    std::vector<swerve_controller::SteeringRange> ranges;
    std::vector<hardware::JointSpec> steering;
    for (std::size_t i = 0; i < 4; ++i) {
      require(c["module_names"][i].as<std::string>() == names[i], "joint order");
      const auto sname = "steer_" + names[i] + "_joint";
      const auto dname = "drive_" + names[i] + "_joint";
      const auto s = hw["motors"][sname], d = hw["motors"][dname];
      require(s["id"].as<int>() == steer_ids[i] && d["id"].as<int>() == wheel_ids[i], "recorded IDs");
      require(s["zero_tick"].as<int>() == zeroes[i], "recorded steering zero");
      close(s["scale"].as<double>(), -1, "steering direction");
      close(d["scale"].as<double>(), wheel_scales[i], "wheel direction");
      const auto sj = named(robot, "joint", sname), dj = named(robot, "joint", dname);
      const auto sp = xyz(sj, "xyz"), dp = xyz(dj, "xyz");
      const double x = c["module_x"][i].as<double>(), y = c["module_y"][i].as<double>();
      const double off = c["module_offset"][i].as<double>();
      close(x, -old_y[i], "rotated module X"); close(y, old_x[i], "rotated module Y");
      close(off, offsets[i], "offset relative to calibrated rolling direction");
      close(sp[0], x, "controller/TF X agreement"); close(sp[1], y, "controller/TF Y agreement");
      close(sp[2], 0, "module height");
      close(dp[0], 0, "wheel offset X"); close(dp[1], off, "controller/TF offset agreement");
      close(dp[2], 0, "wheel height");
      for (double v : xyz(sj, "rpy")) {close(v, 0, "steering zero frame");}
      require(std::string(sj->FirstChildElement("axis")->Attribute("xyz")) == "0 0 1", "steer axis");
      require(std::string(dj->FirstChildElement("axis")->Attribute("xyz")) == "0 1 0", "rolling axis");
      const auto iface = named(bus, "joint", sname);
      close(param(iface, "id"), steer_ids[i], "XML steering ID");
      close(param(iface, "zero_tick"), zeroes[i], "XML steering zero");
      close(param(iface, "scale"), -1, "XML steering scale");
      close(param(named(bus, "joint", dname), "id"), wheel_ids[i], "XML drive ID");
      close(param(named(bus, "joint", dname), "scale"), wheel_scales[i], "XML drive scale");
      const double lower = -(4095-zeroes[i])*2*pi/4096, upper = zeroes[i]*2*pi/4096;
      close(sj->FirstChildElement("limit")->DoubleAttribute("lower"), lower, "full motor window lower");
      close(sj->FirstChildElement("limit")->DoubleAttribute("upper"), upper, "full motor window upper");
      const double margin = hw["steer_margin_ticks"].as<int>()*2*pi/4096;
      close(resolved["steer_min"][i].as<double>(), lower+margin, "controller lower margin");
      close(resolved["steer_max"][i].as<double>(), upper-margin, "controller upper margin");
      geometry.push_back({x, y, off}); ranges.push_back({lower+margin, upper-margin});
      hardware::JointSpec spec;
      spec.scale = -1; spec.zero_rad = (zeroes[i]-2048)*2*pi/4096;
      close(hardware::motor_to_joint(spec, spec.zero_rad), 0, "zero tick conversion");
      close(hardware::motor_to_joint(spec, spec.zero_rad+2*pi/4096), -2*pi/4096, "encoder sign");
      steering.push_back(spec);
    }
    swerve_controller::SwerveKinematics kinematics(geometry, .033, pi, ranges);
    const std::vector<swerve_controller::BodyTwist> moves{
      {.05,0,0}, {-.05,0,0}, {0,.05,0}, {0,-.05,0}, {.03,.03,0}, {.03,-.03,0},
      {-.03,.03,0}, {-.03,-.03,0}, {0,0,.2}, {0,0,-.2}, {.03,-.02,.1}};
    int checked = 0;
    for (int start = 0; start < 3; ++start) {
      std::vector<swerve_controller::ModuleState> current(4);
      for (std::size_t i = 0; i < 4; ++i) {
        current[i].steer_angle = start == 0 ? 0 : (start == 1 ? ranges[i].min : ranges[i].max);
      }
      for (const auto & move : moves) {
        bool feasible = false;
        auto command = kinematics.inverse(move, current, false, &feasible);
        require(feasible, "recorded geometry must support basic motion");
        for (std::size_t i = 0; i < 4; ++i) {
          const double a = command[i].steer_angle;
          require(a >= ranges[i].min-1e-10 && a <= ranges[i].max+1e-10, "steering window");
          const double tick = hardware::joint_to_motor(steering[i], a)*4096/(2*pi)+2048;
          require(tick >= 20-1e-8 && tick <= 4075+1e-8, "motor tick margin");
          // Rigid-body velocity at the actual wheel centre, projected onto rolling +X.
          const double wx = geometry[i].x-offsets[i]*std::sin(a);
          const double wy = geometry[i].y+offsets[i]*std::cos(a);
          const double expected = std::cos(a)*(move.vx-move.wz*wy) +
            std::sin(a)*(move.vy+move.wz*wx);
          close(command[i].wheel_speed*.033, expected, "physical wheel-centre velocity");
        }
        const auto odom = kinematics.forward(command);
        close(odom.vx, move.vx, "forward X odometry"); close(odom.vy, move.vy, "left Y odometry");
        close(odom.wz, move.wz, "CCW yaw odometry");
        current = command; ++checked;
      }
    }
    if (argc == 5) {
      const auto odom = YAML::LoadFile(argv[4]);
      require(odom["header"]["frame_id"].as<std::string>() == "odom", "runtime odom frame");
      require(odom["child_frame_id"].as<std::string>() == "base_footprint", "runtime base frame");
      require(odom["header"]["stamp"]["sec"].as<long>() > 0, "runtime odometry stamp");
      require(odom["pose"]["pose"]["position"]["x"].as<double>() > .005, "mock forward motion must advance +X");
      close(odom["pose"]["pose"]["position"]["y"].as<double>(), 0, "mock forward motion must not move Y");
      close(odom["twist"]["twist"]["linear"]["x"].as<double>(), 0, "mock command timeout must stop");
    }
    std::cout << "PASS: recorded IDs/zeroes/signs; one bus/eight joints; shared TF geometry; "
      << "motor/controller limits; " << checked << " motion/encoder-window checks. No hardware access.\n";
    return 0;
  } catch (const std::exception & e) {
    std::cerr << "FAIL: " << e.what() << '\n';
    return 1;
  }
}
