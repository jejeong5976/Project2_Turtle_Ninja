// 팀 nt_control(libnt_control_core.a) 원본으로 같은 입력을 계산해 출력 (Python 판 대조용, 읽기 전용)
#include "nt_control/swerve.hpp"
#include <cstdio>
#include <iostream>
#include <sstream>
#include <memory>
using namespace nt_control;
int main() {
  std::string line; std::getline(std::cin, line); std::istringstream h(line);
  std::vector<ModuleGeometry> g(4); std::vector<SteeringRange> r(4); double rad, bm, es;
  for (auto & m : g) h >> m.x >> m.y >> m.offset;
  for (auto & x : r) h >> x.min >> x.max;
  h >> rad >> bm >> es;
  SwerveDriveConfig cfg; int bs, cs, std_, rws;
  h >> cfg.max_wheel_speed >> cfg.max_linear >> cfg.max_angular >> cfg.max_linear_accel >> cfg.max_angular_accel
    >> cfg.cmd_timeout_s >> cs >> std_ >> cfg.steer_tolerance >> rws >> bs >> cfg.blend_margin >> cfg.blend_stall_s;
  cfg.cosine_scale = cs; cfg.steer_then_drive = std_; cfg.roll_while_steering = rws; cfg.blend_steer_drive = bs;
  cfg.turn_timeout_s = 1e9;
  SwerveKinematics kin(g, rad, 3.141592653589793, r, true, bm, es);
  std::unique_ptr<SwerveDrive> drv;
  while (std::getline(std::cin, line)) {
    std::istringstream s(line); char k; s >> k;
    if (k == 'I') {
      BodyTwist c; int po, cos_, nc, np; s >> c.vx >> c.vy >> c.wz >> po >> cos_ >> nc;
      std::vector<ModuleState> cur(nc), prev; for (auto & m : cur) s >> m.steer_angle;
      s >> np; prev.resize(np); for (auto & m : prev) s >> m.steer_angle;
      bool feas = true; auto o = kin.inverse(c, cur, cos_, &feas, prev, po);
      printf("%d", feas ? 1 : 0); for (auto & m : o) printf(" %.12g %.12g", m.steer_angle, m.wheel_speed); printf("\n");
    } else if (k == 'R') {
      drv = std::make_unique<SwerveDrive>(kin, cfg);
    } else if (k == 'D') {
      double t, dt, vx, vy, wz; int hc; s >> t >> dt >> vx >> vy >> wz >> hc;
      std::vector<ModuleState> m(4); for (auto & x : m) s >> x.steer_angle >> x.steer_rate >> x.wheel_speed;
      drv->set_turn(true, 1, t);
      if (hc) drv->set_command({vx, vy, wz}, t);
      auto o = drv->update(t, dt, m);
      for (auto & x : o) printf("%.12g %.12g ", x.steer_angle, x.wheel_speed); printf("\n");
    }
  }
}
