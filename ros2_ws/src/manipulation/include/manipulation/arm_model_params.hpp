// arm_model_params.hpp — 노드 파라미터에서 팔 기구학 · 그리퍼 연결 막대 읽기 (control_master · arm_state_publisher 공용)
//   arm_model.*      : config/arm_kinematics.yaml
//   gripper_linkage.*: config/gripper_linkage.yaml

#pragma once

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "manipulation/arm_kinematics.hpp"
#include "manipulation/arm_state.hpp"

namespace manipulation
{

inline std::vector<double> declare_array(rclcpp::Node & n, const std::string & name)
{
  return n.declare_parameter<std::vector<double>>(name, std::vector<double>{});
}

// 크기가 하나라도 다르면 false (그 기능은 쓰지 않음)
inline bool load_arm_model(rclcpp::Node & n, ArmModel & m)
{
  auto arr = [&n](const std::string & k) {return declare_array(n, "arm_model." + k);};
  const auto Tb = arr("T_base_armbase"), JT = arr("joint_T"), ax = arr("axis"), tip = arr("tip5"), app = arr("app5"),
    cl = arr("close5"), j1 = arr("j1_xy"), lo = arr("lo"), hi = arr("hi"), q0 = arr("template_q"), ref = arr("ref_tick"),
    qd = arr("q_deg_at_ref"), sg = arr("sign"), tlo = arr("tick_lo"), thi = arr("tick_hi");
  m.q5_rest = n.declare_parameter<double>("arm_model.q5_rest", 0.0);
  m.sag_c = n.declare_parameter<double>("arm_model.sag_c", 0.0);
  m.base_z_floor = n.declare_parameter<double>("arm_model.base_z_floor", 0.072);
  m.team_base_z_floor = n.declare_parameter<double>("arm_model.team_base_z_floor", 0.033);
  if (Tb.size() != 16 || JT.size() != 80 || ax.size() != 15 || tip.size() != 3 || app.size() != 3 || cl.size() != 3 ||
    j1.size() != 2 || lo.size() != 5 || hi.size() != 5 || q0.size() != 5 || ref.size() != 5 || qd.size() != 5 ||
    sg.size() != 5 || tlo.size() != 5 || thi.size() != 5)
  {
    return false;
  }
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      m.T_base_armbase(r, c) = Tb[r * 4 + c];
      for (int j = 0; j < 5; ++j) {m.joint_T[j](r, c) = JT[j * 16 + r * 4 + c];}
    }
  }
  for (int j = 0; j < 5; ++j) {
    m.axis[j] = Eigen::Vector3d(ax[j * 3], ax[j * 3 + 1], ax[j * 3 + 2]);
    m.lo[j] = lo[j];
    m.hi[j] = hi[j];
    m.template_q[j] = q0[j];
    m.jm[j] = {ref[j], qd[j], sg[j]};
    m.tick_range[j] = {{tlo[j], thi[j]}};
  }
  m.tip5 = Eigen::Vector3d(tip[0], tip[1], tip[2]);
  m.app5 = Eigen::Vector3d(app[0], app[1], app[2]);
  m.close5 = Eigen::Vector3d(cl[0], cl[1], cl[2]);
  m.j1_xy = Eigen::Vector2d(j1[0], j1[1]);
  return true;
}

inline bool load_gripper_linkage(rclcpp::Node & n, GripperLinkage & g)
{
  auto arr = [&n](const std::string & k) {return declare_array(n, "gripper_linkage." + k);};
  const auto ax = arr("hinge_axis"), c = arr("crank_center"), s = arr("slide_axis"), e = arr("basis_e"), t = arr("basis_t"),
    fq0 = arr("finger_pin_q0"), cp = arr("crank_pin_cad"), fp = arr("finger_pin_cad"), phi = arr("phi0"), qr = arr("q_range");
  g.crank_radius = n.declare_parameter<double>("gripper_linkage.crank_radius", 0.0);
  g.rod_length = n.declare_parameter<double>("gripper_linkage.rod_length", 0.0);
  g.cad_pin_alpha = n.declare_parameter<double>("gripper_linkage.cad_pin_alpha", 0.0);
  g.theta0 = n.declare_parameter<double>("gripper_linkage.theta0", 0.0);
  if (ax.size() != 3 || c.size() != 3 || s.size() != 3 || e.size() != 3 || t.size() != 3 || fq0.size() != 6 ||
    cp.size() != 6 || fp.size() != 6 || phi.size() != 2 || qr.size() != 2 || g.crank_radius <= 0.0 || g.rod_length <= 0.0)
  {
    return false;
  }
  auto v3 = [](const std::vector<double> & v, std::size_t i) {return Eigen::Vector3d(v[i], v[i + 1], v[i + 2]);};
  g.axis = v3(ax, 0).normalized();
  g.center = v3(c, 0);
  g.slide = v3(s, 0);
  g.e = v3(e, 0);
  g.t = v3(t, 0);
  for (int k = 0; k < 2; ++k) {
    g.finger_pin_q0[k] = v3(fq0, 3 * k);
    g.crank_pin_cad[k] = v3(cp, 3 * k);
    g.finger_pin_cad[k] = v3(fp, 3 * k);
    g.phi0[k] = phi[k];
  }
  g.q_min = qr[0];
  g.q_max = qr[1];
  return true;
}

}  // namespace manipulation
