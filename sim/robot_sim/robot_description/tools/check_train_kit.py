"""학습 묶음 확인 (Isaac Lab 3.0, headless, env 여러 개) — config/nt_train_kit.py 를 학습 환경처럼 써서
(1) parked 1 s 유지 (2) 실측 꺼내기→복귀(1배) (3) 스워브 직진 0.26 m/s 2 s 를 모든 env 에서 확인 → config/check_train_kit.json.
실행: OMNI_KIT_ACCEPT_EULA=YES <Isaac Lab 3.0 python> tools/check_train_kit.py [env 수=8] [device=cuda:0]"""
import json, math, sys
from pathlib import Path
HERE = Path(__file__).resolve().parents[1]; sys.path.insert(0, str(HERE))
N = int(sys.argv[1]) if len(sys.argv) > 1 else 8
DEV = sys.argv[2] if len(sys.argv) > 2 else "cuda:0"
from isaaclab.app import AppLauncher
app = AppLauncher(headless=True).app
import numpy as np, torch, omni.usd
import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.utils import configclass
from config import nt_train_kit as kit


@configclass
class SceneCfg(InteractiveSceneCfg):
    ground = AssetBaseCfg(prim_path="/World/ground", spawn=sim_utils.GroundPlaneCfg())
    robot = kit.robot_cfg("{ENV_REGEX_NS}/Robot")


DT = 0.005
sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=DT, device=DEV))
scene = InteractiveScene(SceneCfg(num_envs=N, env_spacing=1.0, replicate_physics=False))
n_f = kit.apply_collision_filters(omni.usd.get_context().get_stage())
sim.reset()
robot = scene["robot"]
T = lambda x: x.torch if hasattr(x, "torch") else x
names = robot.joint_names
ai = [names.index(n) for n in [f"arm_joint{i}" for i in range(1, 6)]]
gi = [names.index(n) for n in ("gripper_finger_a_joint", "gripper_finger_b_joint")]
si = [names.index(f"steer_{m}_joint") for m in ("fl", "fr", "rl", "rr")]
wi = [names.index(f"drive_{m}_joint") for m in ("fl", "fr", "rl", "rr")]
rp = kit.Replay()
tgt = T(robot.data.default_joint_pos).clone()


def step(n=1):
    for _ in range(n):
        robot.set_joint_position_target(tgt); scene.write_data_to_sim(); sim.step(render=False); scene.update(DT)


res = {"num_envs": N, "device": DEV, "robots_filtered": n_f, "run": rp.run}
step(200)
gap = kit.gap_mm(T(robot.data.joint_pos)[:, gi].mean(1)).cpu().numpy()
q_park = torch.tensor(rp.pose("parked")[0], device=DEV, dtype=tgt.dtype)
res["hold"] = {"gap_mm": [round(float(g), 2) for g in gap],
               "arm_err_max_deg": round(math.degrees(float((T(robot.data.joint_pos)[:, ai] - T(robot.data.default_joint_pos)[:, ai]).abs().max())), 2),
               "base_z_mm": [round(float(z) * 1000, 1) for z in T(robot.data.root_pos_w)[:, 2]]}
for name in ("out", "back"):
    a, b = rp.segment(name); t = a
    while t <= b:
        q, _ = rp.at(t); tgt[:, ai] = torch.tensor(q, device=DEV, dtype=tgt.dtype); step(); t += DT
    step(200)
    e = (T(robot.data.joint_pos)[:, ai] - torch.tensor(rp.at(b)[0], device=DEV, dtype=tgt.dtype)).abs().max(1).values
    res[name] = {"end_err_deg": [round(math.degrees(float(v)), 2) for v in e], "gap_mm": [round(float(g), 2) for g in kit.gap_mm(T(robot.data.joint_pos)[:, gi].mean(1))]}
drives = [kit.swerve()[0] for _ in range(N)]; nt = kit.swerve()[1]
p0 = T(robot.data.root_pos_w)[:, :2].clone(); t_sim = 0.0
for k in range(int(2.5 / DT)):
    if k % 2 == 0:
        jp, jv = T(robot.data.joint_pos), T(robot.data.joint_vel)
        st, wv = [], []
        for e_, d_ in enumerate(drives):
            d_.set_command(0.26, 0.0, 0.0, t_sim)
            ms = d_.update(t_sim, 0.01, [nt.ModuleState(float(jp[e_, si[i]]), float(jv[e_, wi[i]]), float(jv[e_, si[i]])) for i in range(4)])
            st.append([m.steer_angle for m in ms]); wv.append([m.wheel_speed for m in ms])
        tgt[:, si] = torch.tensor(st, device=DEV, dtype=tgt.dtype)
        robot.set_joint_velocity_target_index(target=torch.tensor(wv, device=DEV, dtype=tgt.dtype), joint_ids=wi)
    if k == int(0.5 / DT):
        p1 = T(robot.data.root_pos_w)[:, :2].clone()
    step(); t_sim += DT
v = (T(robot.data.root_pos_w)[:, :2] - p1).norm(dim=1) / 2.0
res["drive_0.26"] = {"speed_mps_last2s": [round(float(x), 3) for x in v]}
res["finite"] = bool(torch.isfinite(T(robot.data.joint_pos)).all())
ok = (res["finite"] and all(abs(g - 16) < 1.0 for g in res["hold"]["gap_mm"]) and max(res["back"]["end_err_deg"]) < 3.0
      and min(res["drive_0.26"]["speed_mps_last2s"]) > 0.24 and n_f == N)
res["pass"] = bool(ok)
(HERE / "config/check_train_kit.json").write_text(json.dumps(res, ensure_ascii=False, indent=1))
print(json.dumps(res, ensure_ascii=False), flush=True)
app.close()
