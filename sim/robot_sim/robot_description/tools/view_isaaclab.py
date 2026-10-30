"""배치 수정판 보기 — Isaac Lab GUI 창 하나. parked_init_state() 로 바닥 위에 세우고 parked 목표를 계속 유지(관절 고정 아님).
바닥 위 60 mm 에 반투명 원판(라이다 스캔면 표시, 충돌 없음), 판·보드는 모델 그대로. 창을 닫으면 끝.
실행: OMNI_KIT_ACCEPT_EULA=YES /home/pa25/IsaacLab-course/.venv/bin/python tools/view_isaaclab.py"""
import sys
from pathlib import Path
HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
from isaaclab.app import AppLauncher
app = AppLauncher({"headless": False, "visualizer": ["kit"]}).app   # Isaac Lab 3.0: Kit GUI 창은 visualizer kit 로 열림
import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.actuators import ImplicitActuatorCfg
from config.isaac_lab_init_state import parked_init_state
sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=0.005, device="cuda:0"))
sim_utils.GroundPlaneCfg().func("/World/ground", sim_utils.GroundPlaneCfg())
sim_utils.DomeLightCfg(intensity=2000.0).func("/World/light", sim_utils.DomeLightCfg(intensity=2000.0))
scan = sim_utils.CylinderCfg(radius=0.22, height=0.0008, axis="Z",
    visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.2, 0.2), opacity=0.35))
scan.func("/World/ScanPlane_60mm", scan, translation=(0.0, 0.0, 0.060))
robot = Articulation(ArticulationCfg(prim_path="/World/Robot",
    spawn=sim_utils.UsdFileCfg(usd_path=str(HERE / "usd/robot/robot.usda"), variants={"Physics": "physx"}),
    init_state=parked_init_state(root_z=0.075),
    actuators={"body": ImplicitActuatorCfg(joint_names_expr=["steer_.*", "drive_.*", "arm_joint[1-5]"], stiffness=None, damping=None),
               "gripper": ImplicitActuatorCfg(joint_names_expr=["gripper_finger_.*"], stiffness=1000.0, damping=2.0)}))   # 손가락 a·b 둘 다 구동
sim.set_camera_view(eye=(0.45, -0.55, 0.40), target=(0.0, 0.0, 0.10))
sim.reset()
tgt = robot.data.default_joint_pos.torch.clone() if hasattr(robot.data.default_joint_pos, "torch") else robot.data.default_joint_pos.clone()
while app.is_running():
    robot.set_joint_position_target(tgt); robot.write_data_to_sim(); sim.step(); robot.update(0.005)
app.close()
