"""Isaac Lab(PhysX) 에서 배치 수정판 USD 를 parked_init_state() 로 띄워 확인 — headless, 결과 config/verify_isaaclab.json.
관절 전부 움직일 수 있는 상태(고정 안 함), 위치 목표 = parked 유지. 2 s 정착 후: 관절값, 그리퍼 간격, base 높이, 스캔면(URDF 오프셋),
팔·손가락 링크 위치, 팔 링크 원점 vs Jetson 충돌 상자. 실행: /home/pa25/IsaacLab-course/.venv/bin/python tools/verify_isaaclab.py"""
import json, sys, os
from pathlib import Path
HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
from isaaclab.app import AppLauncher
app = AppLauncher(headless=True).app
import torch
import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.actuators import ImplicitActuatorCfg
from config.isaac_lab_init_state import parked_init_state, parked_joint_pos

sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=0.005, device="cuda:0"))
sim_utils.GroundPlaneCfg().func("/World/ground", sim_utils.GroundPlaneCfg())
cfg = ArticulationCfg(prim_path="/World/Robot",
    spawn=sim_utils.UsdFileCfg(usd_path=os.environ.get("VERIFY_USD", str(HERE / "usd/robot/robot.usda")), variants={"Physics": "physx"}),
    init_state=parked_init_state(root_z=0.075),
    actuators=({"arm_off": ImplicitActuatorCfg(joint_names_expr=["arm_joint[1-5]"], stiffness=0.0, damping=0.02),   # 토크 OFF 흉내: 팔 관절 힘 0
                "rest": ImplicitActuatorCfg(joint_names_expr=["steer_.*", "drive_.*"], stiffness=None, damping=None),
                "gripper": ImplicitActuatorCfg(joint_names_expr=["gripper_finger_.*"], stiffness=1000.0, damping=2.0)}
               if os.environ.get("ARM_PASSIVE") == "1" else
               {"body": ImplicitActuatorCfg(joint_names_expr=["steer_.*", "drive_.*", "arm_joint[1-5]"], stiffness=None, damping=None),
                # 손가락 a·b 둘 다 같은 구동(USD 의 b 는 mimic 이라 구동 0 → 한계 넘어 폭주). a 의 USD 값과 같은 1000/2
                "gripper": ImplicitActuatorCfg(joint_names_expr=["gripper_finger_.*"], stiffness=1000.0, damping=2.0)}))
robot = Articulation(cfg)
sim.reset()
names = robot.joint_names
q_start = robot.data.joint_pos.torch[0].tolist() if hasattr(robot.data.joint_pos, "torch") else robot.data.joint_pos[0].tolist()
tgt = robot.data.default_joint_pos.torch.clone() if hasattr(robot.data.default_joint_pos, "torch") else robot.data.default_joint_pos.clone()
for _ in range(600 if os.environ.get("ARM_PASSIVE") == "1" else 400):
    robot.set_joint_position_target(tgt)
    robot.write_data_to_sim(); sim.step(); robot.update(0.005)
jp = robot.data.joint_pos.torch[0] if hasattr(robot.data.joint_pos, "torch") else robot.data.joint_pos[0]
root = robot.data.root_pose_w.torch[0] if hasattr(robot.data.root_pose_w, "torch") else robot.data.root_pos_w[0]
want = parked_joint_pos()
res = {"usd": str(HERE / "usd/robot/robot.usda"), "joints": {}}
for i, n in enumerate(names):
    res["joints"][n] = {"start": round(q_start[i], 6), "end": round(float(jp[i]), 6), "parked": round(want[n], 6) if n in want else 0.0}
qa, qb = float(jp[names.index("gripper_finger_a_joint")]), float(jp[names.index("gripper_finger_b_joint")])
res["gripper_gap_mm_end"] = round(58.50723799617533 + 1000 * (qa + qb), 2)
z = float(root[2]); res["base_above_ground_mm"] = round(z * 1000, 1)   # ground plane at world z = 0
res["scan_plane_above_ground_mm"] = round((z - 0.043 + 0.031) * 1000, 1)
res["arm_max_abs_err_deg"] = round(max(abs(float(jp[names.index(n)]) - want[n]) for n in want if n.startswith("arm")) * 57.2958, 3)
bn = robot.body_names
bp = robot.data.body_pos_w.torch[0] if hasattr(robot.data.body_pos_w, "torch") else robot.data.body_pos_w[0]
base = bp[bn.index("base_link")]
jet_lo = torch.tensor([-0.0385, -0.0994, 0.0465], device=bp.device); jet_hi = torch.tensor([0.0645, -0.0085, 0.08127], device=bp.device)
res["arm_bodies_rel_base_m"] = {}
inside = []
for n in bn:
    if n.startswith(("arm_", "gripper")):
        rel = (bp[bn.index(n)] - base)
        res["arm_bodies_rel_base_m"][n] = [round(float(v), 4) for v in rel]
        if bool(((rel >= jet_lo) & (rel <= jet_hi)).all()):
            inside.append(n)
res["arm_body_origins_inside_jetson_box"] = inside
res["wheel_body_z_above_ground_mm"] = {n: round(float(bp[bn.index(n)][2]) * 1000, 1) for n in bn if "wheel" in n}
pg = robot.data.projected_gravity_b.torch[0] if hasattr(robot.data.projected_gravity_b, "torch") else robot.data.projected_gravity_b[0]
res["projected_gravity_b"] = [round(float(v), 3) for v in pg]
res["root_z_raw_m"] = round(z, 4)
res["note"] = "Jetson 판정은 팔 링크 원점 기준(형상 전체 아님). 팔판(base z 82.5 mm) 위에 팔이 있고 Jetson 상자 위끝 81.27 mm."
(HERE / ("config/verify_isaaclab_ref.json" if "VERIFY_USD" in os.environ else "config/verify_isaaclab_arm_off.json" if os.environ.get("ARM_PASSIVE") == "1" else "config/verify_isaaclab.json")).write_text(json.dumps(res, ensure_ascii=False, indent=1))
print(json.dumps(res, ensure_ascii=False, indent=1))
app.close()
