"""Isaac Lab 에서 이 로봇을 parked(주행 기본자세)·그리퍼 16 mm 로 시작·리셋하려면 이 init_state 를 쓰세요.
Isaac Lab 은 USD 의 관절 초기값(JointStateAPI)을 읽지 않고 ArticulationCfg.init_state 만 씁니다(기본은 전부 0 → 그리퍼 58.5 mm).
손가락 b 는 mimic 구속이라 상태가 자동 복사되지 않으므로 a·b 둘 다 넣습니다. 팔은 고정이 아니라 계속 제어 가능한 관절입니다.

    from isaaclab.assets import ArticulationCfg
    from config.isaac_lab_init_state import parked_init_state
    robot = ArticulationCfg(..., init_state=parked_init_state())

손가락 구동: USD 의 손가락 b 는 mimic 이라 구동 힘 0 → Isaac Lab 에서는 두 손가락 모두
ImplicitActuatorCfg(joint_names_expr=["gripper_finger_.*"], stiffness=1000.0, damping=2.0) 로 같은 목표를 주세요(검증한 설정).

리셋 때: write_joint_position_to_sim(default_joint_pos) 와 함께 팔·손가락 위치 목표도 default 로
(set_joint_position_target) 넣어야 PD 가 0 으로 끌고 가지 않습니다. 정책이 팔을 명령하지 않는 동안은 목표를 parked 로 유지하세요.
"""
import json
from pathlib import Path

POSE = Path(__file__).resolve().parent / "initial_pose.json"


def parked_joint_pos():
    q = dict(json.loads(POSE.read_text())["positions"])          # arm_joint1..5 [rad], gripper_finger_a_joint [m]
    import yaml
    G = yaml.safe_load((POSE.parent / "layout_20261027.yaml").read_text())["gripper"]
    # 실측 패드 간격 16 mm, CAD 에 패드 포함(pad_per_side_m 0) → q = (16 − 58.507)/2000 = −0.0212536 m
    q["gripper_finger_a_joint"] = q["gripper_finger_b_joint"] = (G["closed_pad_gap_mm"] + 2000 * G["pad_per_side_m"] - G["gap0_mm"]) / 2000
    import os
    if os.environ.get("NT_GRIPPER_GAP_MM"):                     # 시험용 덮어쓰기
        q["gripper_finger_a_joint"] = q["gripper_finger_b_joint"] = (float(os.environ["NT_GRIPPER_GAP_MM"]) - 58.50723799617533) / 2000
    return q


def parked_init_state(root_z=0.075):
    """root_z: base_link 높이. 바퀴 접지면이 base_link −0.072 m 라 0.072 가 딱 닿음(기존 시험은 0.075, 3 mm 위에서 떨어뜨림)."""
    from isaaclab.assets import ArticulationCfg
    return ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, root_z),
        joint_pos={**parked_joint_pos(), "steer_.*": 0.0, "drive_.*": 0.0},
        joint_vel={".*": 0.0})
