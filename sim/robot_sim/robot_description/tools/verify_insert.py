"""받침에서 꺼냈다 다시 넣기 시험 (Isaac Lab, headless) — parked → P0(C1_P0, 2 s) → 0.5 s 유지 → parked(2 s) → 1 s 유지. 그리퍼 16 mm 유지.
기록: 구간별 최대 관절 추종 오차, 끝 자세 오차, 그리퍼 간격, 폭주 여부 → config/verify_insert.json.
실행: OMNI_KIT_ACCEPT_EULA=YES /home/pa25/IsaacLab-course/.venv/bin/python tools/verify_insert.py"""
import json, sys, math
from pathlib import Path
HERE = Path(__file__).resolve().parents[1]; sys.path.insert(0, str(HERE))
from isaaclab.app import AppLauncher
app = AppLauncher(headless=True).app
import torch
import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.actuators import ImplicitActuatorCfg
from config.isaac_lab_init_state import parked_init_state, parked_joint_pos
sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=0.005, device="cuda:0"))
sim_utils.GroundPlaneCfg().func("/World/ground", sim_utils.GroundPlaneCfg())
robot = Articulation(ArticulationCfg(prim_path="/World/Robot",
    spawn=sim_utils.UsdFileCfg(usd_path=str(HERE / "usd/robot/robot.usda"), variants={"Physics": "physx"}),
    init_state=parked_init_state(0.075),
    actuators={"body": ImplicitActuatorCfg(joint_names_expr=["steer_.*", "drive_.*", "arm_joint[1-5]"], stiffness=None, damping=None),
               "gripper": ImplicitActuatorCfg(joint_names_expr=["gripper_finger_.*"], stiffness=1000.0, damping=2.0)}))
sim.reset()
T = lambda x: x.torch if hasattr(x, "torch") else x
names = robot.joint_names
park = parked_joint_pos()
p0 = dict(json.loads((HERE / "config/initial_pose.json").read_text())["poses"]["C1_P0"]["q_rad"])
arm = [f"arm_joint{i}" for i in range(1, 6)]; ai = [names.index(n) for n in arm]
base = T(robot.data.default_joint_pos).clone()
def target(s):   # s: 0 = parked, 1 = P0 (joint-space linear)
    q = base.clone()
    for n, i in zip(arm, ai):
        q[0, i] = park[n] + s * (p0[n] - park[n])
    return q
plan = [("out", 400, lambda k: k / 399), ("hold_p0", 100, lambda k: 1.0), ("in", 400, lambda k: 1 - k / 399), ("hold_park", 200, lambda k: 0.0)]
res = {"phases": {}}
for name, n, f in plan:
    mx = 0.0
    for k in range(n):
        tg = target(f(k)); robot.set_joint_position_target(tg); robot.write_data_to_sim(); sim.step(); robot.update(0.005)
        jp = T(robot.data.joint_pos)
        if not torch.isfinite(jp).all():
            res["NaN"] = name; break
        mx = max(mx, float((jp[0, ai] - tg[0, ai]).abs().max()))
    res["phases"][name] = {"max_track_err_deg": round(math.degrees(mx), 3)}
    if "NaN" in res: break
jp = T(robot.data.joint_pos)[0]
res["end_arm_err_deg"] = round(max(abs(math.degrees(float(jp[i]) - park[n])) for n, i in zip(arm, ai)), 3)
res["end_gripper_gap_mm"] = round(58.50723799617533 + 1000 * (float(jp[names.index("gripper_finger_a_joint")]) + float(jp[names.index("gripper_finger_b_joint")])), 2)
res["finite"] = bool(torch.isfinite(jp).all())
(HERE / "config/verify_insert.json").write_text(json.dumps(res, ensure_ascii=False, indent=1)); print(json.dumps(res, ensure_ascii=False))
app.close()
