"""받침 복귀 배속 시험 (Isaac Lab, headless, 데모와 같은 CPU 물리·액추에이터) — 실물 P0→받침 복귀 궤적을 1·1.5·2배로 재생.
P0(복귀 시작 자세)에서 그리퍼 16 mm 로 시작 → 복귀 재생 → 1 s 유지. 배속마다 처음부터 다시.
기록: 복귀 중 관절 최대 추종 오차(목표−실제), 끝 자세 오차(받침에 다 들어갔는지), 그리퍼 간격 → config/verify_return_speed.json.
실행: OMNI_KIT_ACCEPT_EULA=YES /home/pa25/IsaacLab-course/.venv/bin/python tools/verify_return_speed.py [ARM_실행이름]"""
import json, sys, math
from pathlib import Path
HERE = Path(__file__).resolve().parents[1]; sys.path.insert(0, str(HERE))
RUN = sys.argv[1] if len(sys.argv) > 1 else "ARM_20261026T022813Z"
from isaaclab.app import AppLauncher
app = AppLauncher(headless=True).app
import numpy as np, torch
import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.actuators import ImplicitActuatorCfg
from config.isaac_lab_init_state import parked_init_state, parked_joint_pos
DT, DEV = 0.005, "cpu"
sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=DT, device=DEV))
sim_utils.GroundPlaneCfg().func("/World/ground", sim_utils.GroundPlaneCfg())
robot = Articulation(ArticulationCfg(prim_path="/World/Robot",
    spawn=sim_utils.UsdFileCfg(usd_path=str(HERE / "usd/robot/robot.usda"), variants={"Physics": "physx"}),
    init_state=parked_init_state(0.075),
    actuators={"steer": ImplicitActuatorCfg(joint_names_expr=["steer_.*"], joint_effort_limit=4.1, joint_velocity_limit=4.7962, stiffness=20.0, damping=0.3, armature=0.012),
               "drive": ImplicitActuatorCfg(joint_names_expr=["drive_.*"], joint_effort_limit=3.0, joint_velocity_limit=7.9137, stiffness=0.0, damping=1.0, armature=0.005),
               "arm": ImplicitActuatorCfg(joint_names_expr=["arm_joint[1-5]"], stiffness=None, damping=None),
               "gripper": ImplicitActuatorCfg(joint_names_expr=["gripper_finger_.*"], stiffness=1000.0, damping=2.0, joint_effort_limit=20.0, joint_velocity_limit=0.15)}))
sim.reset()
T = lambda x: x.torch if hasattr(x, "torch") else x
names = robot.joint_names
ai = [names.index(f"arm_joint{i}") for i in range(1, 6)]; gi = [names.index(n) for n in ("gripper_finger_a_joint", "gripper_finger_b_joint")]
RP = json.loads((HERE / f"config/arm_replays/{RUN}.json").read_text())
RT = np.array(RP["t_s"]); RQ = np.array(RP["arm_q_rad"]); a_, b_ = RP["segments"][-1]["wall_s"]
rp = lambda t: np.array([np.interp(t, RT, RQ[:, i]) for i in range(5)])
gq = parked_joint_pos()["gripper_finger_a_joint"]
res = {"run": RUN, "segment_s_1x": round(b_ - a_, 3), "speeds": {}}
for speed in (1.0, 1.5, 2.0):
    q0 = T(robot.data.default_joint_pos).clone(); q0[0, ai] = torch.tensor(rp(a_), dtype=q0.dtype)
    robot.write_root_pose_to_sim_index(root_pose=torch.tensor([[0.0, 0.0, 0.075, 0.0, 0.0, 0.0, 1.0]], device=DEV))
    robot.write_root_velocity_to_sim_index(root_velocity=torch.zeros((1, 6), device=DEV))
    robot.write_joint_position_to_sim_index(position=q0); robot.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(q0))
    tgt = q0.clone(); tgt[0, gi] = gq
    for _ in range(200):   # P0 에서 1 s 안정
        robot.set_joint_position_target(tgt); robot.write_data_to_sim(); sim.step(render=False); robot.update(DT)
    t, err_max, err_by = a_, 0.0, np.zeros(5)
    while True:
        q = rp(min(t, b_)); tgt[0, ai] = torch.tensor(q, dtype=tgt.dtype)
        robot.set_joint_position_target(tgt); robot.write_data_to_sim(); sim.step(render=False); robot.update(DT)
        e = np.abs(T(robot.data.joint_pos)[0, ai].cpu().numpy() - q); err_by = np.maximum(err_by, e)
        if t >= b_:
            break
        t += DT * speed
    for _ in range(200):   # 끝 자세 1 s 유지
        robot.set_joint_position_target(tgt); robot.write_data_to_sim(); sim.step(render=False); robot.update(DT)
    qe = np.abs(T(robot.data.joint_pos)[0, ai].cpu().numpy() - rp(b_))
    gap = 58.50723799617533 + 2000 * float(T(robot.data.joint_pos)[0, gi].mean())
    res["speeds"][f"{speed:g}x"] = {"return_s": round((b_ - a_) / speed, 3), "max_track_err_deg_J1..5": [round(math.degrees(v), 2) for v in err_by],
                                    "end_err_deg_J1..5": [round(math.degrees(v), 2) for v in qe], "end_err_max_deg": round(math.degrees(qe.max()), 2), "gripper_gap_mm": round(gap, 1)}
    print(speed, res["speeds"][f"{speed:g}x"], flush=True)
(HERE / "config/verify_return_speed.json").write_text(json.dumps(res, ensure_ascii=False, indent=1))
app.close()
