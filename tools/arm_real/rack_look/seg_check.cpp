// seg_check — 팔 관절 구간 충돌 검사 (MoveIt FCL): URDF (새 적재함) + 칸에 선 퍽 + 쥔 퍽 (팔 모델로 자세)
//   seg_check robot.urdf robot.srdf < 명령
//   PUCK id x y z yaw_deg          : 세운 퍽 (34.8 × 34.8 × 60, base_link 중심)
//   CLEAR                          : 퍽 다 지움
//   HELD 0|1 [depth_mm]            : 쥔 퍽 있음 (손끝 = 윗면 − depth, 기본 50 = 명령 깊이)
//   PAD m                          : 로봇 링크 여유 (기본 0.005)
//   SEG name q1..q5 q1..q5         : 관절 직선 (명령 관절값 rad), 0.5° 간격
//   STATE name q1..q5
//   출력: name OK | HIT a b (구간 몇 %)
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <geometric_shapes/shapes.h>
#include <moveit/collision_detection/collision_common.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <srdfdom/model.h>
#include <urdf/model.h>

#include "manipulation/arm_kinematics.hpp"
#include "arm_kin_data.hpp"

using namespace manipulation;

static std::string slurp(const char * p)
{
  std::ifstream f(p);
  std::stringstream s;
  s << f.rdbuf();
  return s.str();
}

int main(int argc, char ** argv)
{
  if (argc < 3) {std::fprintf(stderr, "seg_check robot.urdf robot.srdf < cmds\n"); return 2;}
  auto urdf = std::make_shared<urdf::Model>();
  if (!urdf->initString(slurp(argv[1]))) {std::fprintf(stderr, "URDF 읽기 실패\n"); return 1;}
  auto srdf = std::make_shared<srdf::Model>();
  if (!srdf->initString(*urdf, slurp(argv[2]))) {std::fprintf(stderr, "SRDF 읽기 실패\n"); return 1;}
  auto model = std::make_shared<moveit::core::RobotModel>(urdf, srdf);
  planning_scene::PlanningScene scene(model);
  const auto km = kin_data::model();
  moveit::core::RobotState st(model);
  st.setToDefaultValues();
  bool held = false;
  double depth = 0.050, pad = 0.005, held_pad = 0.0;
  scene.getCollisionEnvNonConst()->setPadding(pad);
  auto & acm = scene.getAllowedCollisionMatrixNonConst();
  const std::vector<std::string> hand{"gripper_left_link", "gripper_right_link", "wrist_link", "end_effector_link", "link5"};
  const std::string names[5] = {"joint1", "joint2", "joint3", "joint4", "joint5"};
  auto set_q = [&](const double q[5]) {
      for (int j = 0; j < 5; ++j) {st.setVariablePosition(names[j], q[j]);}
      st.setVariablePosition("gripper_left_joint", 0.0083);
      if (model->hasJointModel("gripper_right_joint")) {st.setVariablePosition("gripper_right_joint", 0.0083);}
      st.update();
      scene.getWorldNonConst()->removeObject("held");
      if (held) {
        Ticks t;
        for (int j = 0; j < 5; ++j) {t[j] = static_cast<int>(std::lround(q[j] * 4096.0 / (2.0 * M_PI) + km.zero_tick[j]));}
        const Grip g = arm_grip(km, ticks_to_q(km, t));
        const Eigen::Vector3d app = g.app.normalized(), cl = (g.close - g.close.dot(app) * app).normalized();
        Eigen::Isometry3d P = Eigen::Isometry3d::Identity();
        P.linear().col(0) = cl;
        P.linear().col(1) = app.cross(cl);
        P.linear().col(2) = app;
        P.translation() = g.tip + Eigen::Vector3d(0, 0, 0.039) - (depth - 0.030) * app;   // 윗면 = 손끝 − depth · 손 축, 중심 = 윗면 + 30 mm · 손 축
        scene.getWorldNonConst()->addToObject("held", std::make_shared<shapes::Box>(0.0348 + 2 * held_pad, 0.0348 + 2 * held_pad, 0.060 + 2 * held_pad), P);
        acm.setEntry("held", hand, true);
      }
    };
  auto check = [&](std::string & a, std::string & b) {
      collision_detection::CollisionRequest req;
      req.contacts = true;
      req.max_contacts = 1;
      collision_detection::CollisionResult res;
      scene.checkCollision(req, res, st);
      if (!res.collision) {return false;}
      if (!res.contacts.empty()) {a = res.contacts.begin()->first.first; b = res.contacts.begin()->first.second;}
      return true;
    };
  std::string line;
  while (std::getline(std::cin, line)) {
    std::istringstream ss(line);
    std::string cmd;
    ss >> cmd;
    if (cmd == "PUCK") {
      std::string id; double x, y, z, yaw;
      ss >> id >> x >> y >> z >> yaw;
      Eigen::Isometry3d P = Eigen::Isometry3d::Identity();
      P.linear() = Eigen::AngleAxisd(yaw * M_PI / 180.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
      P.translation() = Eigen::Vector3d(x, y, z);
      scene.getWorldNonConst()->addToObject(id, std::make_shared<shapes::Box>(0.0348, 0.0348, 0.060), P);
      acm.setEntry(id, "held", false);
      for (const auto * l : model->getLinkModels()) {   // 칸에 놓인 퍽 ↔ 차체 · 적재함 (닿아 있음) 은 허용
        const std::string & ln = l->getName();
        if (ln == "base_link" || ln.rfind("rack", 0) == 0) {acm.setEntry(id, ln, true);}
      }
    } else if (cmd == "CLEAR") {
      for (const auto & id : scene.getWorld()->getObjectIds()) {if (id != "held") {scene.getWorldNonConst()->removeObject(id);}}
    } else if (cmd == "HELD") {
      int h; ss >> h; held = h != 0;
      double d; if (ss >> d) {depth = d / 1000.0;}
    } else if (cmd == "HELDPAD") {
      ss >> held_pad;
    } else if (cmd == "BOX") {   // 상자 (박스 · 장애물): id x y z sx sy sz (base_link 중심)
      std::string id; double x, y, z, sx, sy, sz;
      ss >> id >> x >> y >> z >> sx >> sy >> sz;
      Eigen::Isometry3d P = Eigen::Isometry3d::Identity();
      P.translation() = Eigen::Vector3d(x, y, z);
      scene.getWorldNonConst()->addToObject(id, std::make_shared<shapes::Box>(sx, sy, sz), P);
    } else if (cmd == "PAD") {
      ss >> pad; scene.getCollisionEnvNonConst()->setPadding(pad);
    } else if (cmd == "STATE" || cmd == "SEG") {
      std::string name; ss >> name;
      double a[5], b[5];
      for (double & v : a) {ss >> v;}
      if (cmd == "SEG") {for (double & v : b) {ss >> v;}} else {for (int j = 0; j < 5; ++j) {b[j] = a[j];}}
      double span = 0.0;
      for (int j = 0; j < 5; ++j) {span = std::max(span, std::fabs(b[j] - a[j]));}
      const int n = std::max(1, static_cast<int>(std::ceil(span / (0.5 * M_PI / 180.0))));
      bool hit = false;
      for (int k = 0; k <= n && !hit; ++k) {
        double q[5];
        for (int j = 0; j < 5; ++j) {q[j] = a[j] + (b[j] - a[j]) * k / n;}
        set_q(q);
        std::string l1, l2;
        if (check(l1, l2)) {std::printf("%s HIT %s %s %.0f%%\n", name.c_str(), l1.c_str(), l2.c_str(), 100.0 * k / n); hit = true;}
      }
      if (!hit) {std::printf("%s OK\n", name.c_str());}
      std::fflush(stdout);
    }
  }
  return 0;
}
