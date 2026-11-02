"""
로봇 모델·ros2_control·바퀴/팔 컨트롤러·control_master 실행.
기본 hardware:=mock. 인지·미션 노드와 카메라·라이다 드라이버는 별도 담당 패키지에서 구성한다.
설정은 bringup이 설치한 share/bringup/config에서 읽는다.
실물 OpenCR 실행은 보드·펌웨어·포트·기계 안전 조건의 별도 확인이 필요하다.

  swerve_controller (arm:=true · false 공통, swerve_setup): 조향 zero_tick 이 있으면 모듈별 허용 범위(steer_min · steer_max)를
    계산해 넘김 — 위치 모드 눈금 0~4095 에서 양 끝 steer_margin_ticks 를 뺀 구간 ∩ 선 한계(cable_*_deg). 컨트롤러는 그 안에서
    조향각 + 바퀴 정 · 역회전을 고르고, 둘 다 범위 밖이면 자르지 않고 전체 정지. 영점이 없으면(mock) 공통 steer_limit
    거부 (노드를 띄우기 전): hardware:=opencr 인데 조향 zero_tick 미확정 / 여러 바퀴(extended) 조향을 실물로
  # 팔 OpenCR 을 안 붙이고 바퀴만 (arm:=false, 하드웨어는 다 장착해도 됨) — 팔 컨트롤러 · control_master 를 띄우지 않음
  ros2 launch bringup robot.launch.py arm:=false hardware_map:=$(ros2 pkg prefix bringup)/share/bringup/config/hardware_map_drive_only.yaml
    거부: 하드웨어 지도에 팔 관절이 있음. swerve_controller 는 차례(require_motion_mode) 를 보지 않음
    바퀴만 시험한 기능은 arm:=false 에서만: park_on_idle · blend_steer_drive · drive_test_params · enforce_command_limits:=false
    hardware:=mock mock_dynamics:=true : 조향 · 바퀴에 속도 · 가속 한계가 있는 가짜 모터
"""

from launch import LaunchDescription
import math
import os
import tempfile

import yaml
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

CONTROLLERS = [
    "joint_state_broadcaster",
    # swerve_controller 는 여기 없음 — swerve_setup 이 검사 · 모듈별 범위와 함께 띄움
    "arm_controller",
    "gripper_controller",
]
ARM_CONTROLLERS = {"arm_controller", "gripper_controller"}
STEER_JOINTS = ["steer_fl_joint", "steer_fr_joint", "steer_rl_joint", "steer_rr_joint"]   # controllers.yaml module_names 순서
STEER_MARGIN_TICKS = 20   # 위치 모드 끝(0 · 4095)에서 떨어뜨릴 눈금 (≈ 1.8°)
TICK_RAD = 2.0 * math.pi / 4096
ARM_JOINTS = {"joint1", "joint2", "joint3", "joint4", "joint5", "gripper_left_joint"}


def steer_ranges(hw, margin_ticks=STEER_MARGIN_TICKS):
    """하드웨어 지도 → 모듈별 조향 허용 관절 범위 [rad] (STEER_JOINTS 순서). 영점이 하나라도 없으면 None
    위치 모드 눈금 창(양 끝 margin_ticks 여유)과 선 한계(cable_min_deg / cable_max_deg, 있으면) 중 좁은 쪽
    관절 = scale × (눈금 − zero_tick) × 2π/4096 (hardware motor_to_joint 와 같은 식)"""
    lo, hi = [], []
    for j in STEER_JOINTS:
        m = hw["motors"][j]
        if m.get("zero_tick") is None:
            return None
        z, sc = int(m["zero_tick"]), float(m.get("scale", 1.0))
        if m.get("extended"):   # 여러 바퀴(확장 위치) 모드 — 범위 = 선 한계 그대로 (모의 전용, 아래에서 실물 거부)
            lo.append(math.radians(float(m["cable_min_deg"])))
            hi.append(math.radians(float(m["cable_max_deg"])))
            continue
        a = sc * (margin_ticks - z) * TICK_RAD
        b = sc * (4095 - margin_ticks - z) * TICK_RAD
        # 선(케이블) 한계가 있으면 그 안으로만 (hardware_map cable_min_deg / cable_max_deg)
        lo.append(max(min(a, b), math.radians(float(m.get("cable_min_deg", -math.inf)))))
        hi.append(min(max(a, b), math.radians(float(m.get("cable_max_deg", math.inf)))))
        if hi[-1] - lo[-1] < math.pi:
            raise RuntimeError(f"{j} 조향 범위 {math.degrees(lo[-1]):.1f}~{math.degrees(hi[-1]):.1f}° 가 180° 보다 좁음 "
                               "— 어떤 방향은 정 · 역 어느 쪽으로도 못 감")
    return lo, hi


def spawner_node(name, files, **kw):
    args_ = [name]
    for f in files:
        args_ += ["--param-file", f]
    return Node(package="controller_manager", executable="spawner", arguments=args_, output="both", **kw)


def swerve_setup(context):
    """swerve_controller 를 띄우기 전 검사 + 설정 — 안 되면 노드를 띄우기 전에 멈춤 (예외 → launch 실패).
    arm:=true(시연 · 전체)도 같은 길로 (모듈별 조향 범위를 모르면 범위 끝을 넘어 선을 당길 수 있음).
      공통      : 조향 zero_tick 미확정이면 실물 거부 · 모듈별 범위(눈금 창 ∩ 선 한계) · 조향 정책 · 여러 바퀴 모드 실물 거부
      arm:=false: 팔 관절이 지도에 있으면 거부 · 차례 안 봄 · 정지 복귀 · 동시 조향 · 시험 파라미터 허용
    통과하면 swerve_controller spawner (controllers.yaml + 모듈별 범위 · 정책) 를 돌려줌"""
    arm_off = context.perform_substitution(LaunchConfiguration("arm")) == "false"
    park = context.perform_substitution(LaunchConfiguration("park_on_idle"))
    if park not in ("true", "false"):
        raise RuntimeError(f"park_on_idle 은 true | false (받은 값 {park!r})")
    blend = context.perform_substitution(LaunchConfiguration("blend_steer_drive"))
    if blend not in ("true", "false"):
        raise RuntimeError(f"blend_steer_drive 은 true | false (받은 값 {blend!r})")
    test_path = context.perform_substitution(LaunchConfiguration("drive_test_params"))
    if not arm_off:   # 바퀴만 시험한 기능은 팔이 있는 실행에서 거부
        if context.perform_substitution(LaunchConfiguration("enforce_command_limits")) != "true":
            raise RuntimeError("enforce_command_limits:=false requires arm:=false")
        if park == "true":
            raise RuntimeError("park_on_idle:=true 거부 — arm:=false (바퀴만) 에서만 시험한 기능")
        if blend == "true" or test_path:
            raise RuntimeError("blend_steer_drive · drive_test_params 는 arm:=false (바퀴만) 에서만 시험한 기능")
    path = context.perform_substitution(LaunchConfiguration("hardware_map"))
    hw = yaml.safe_load(open(path))
    on_bus = {j for b in hw.get("buses", []) for j in b.get("joints", [])}
    if arm_off:
        arm_found = sorted((on_bus | set(hw.get("motors", {}))) & ARM_JOINTS)
        if arm_found:
            raise RuntimeError(f"arm:=false 거부 — 하드웨어 지도에 팔 관절이 있음 {arm_found} ({path}). "
                               "팔이 있는 로봇은 arm:=true (기본)")
    is_real = context.perform_substitution(LaunchConfiguration("hardware")) == "opencr"
    if is_real:
        missing = [j for j in STEER_JOINTS if j in on_bus and hw["motors"][j].get("zero_tick") is None]
        if missing:
            raise RuntimeError(f"hardware:=opencr 거부 — 조향 zero_tick 미확정 {missing}. "
                               "실물을 직진으로 맞춘 뒤 읽은 눈금을 하드웨어 지도에 넣을 것")
    files = [context.perform_substitution(cfg("controllers.yaml"))]
    margin_ticks = int(context.perform_substitution(LaunchConfiguration("steer_margin_ticks")))
    if not 0 <= margin_ticks <= 2047:
        raise RuntimeError("steer_margin_ticks must be 0..2047")
    ranges = steer_ranges(hw, margin_ticks)
    multi = [j for j in STEER_JOINTS if hw["motors"][j].get("extended")]
    if multi and is_real:
        raise RuntimeError(f"extended(여러 바퀴) 조향 {multi} 는 아직 모의 전용 — nt_opencr 펌웨어가 확장 위치 모드를 지원하지 않음")
    policy = context.perform_substitution(LaunchConfiguration("steering_policy"))
    if policy not in ("nearest", "coherent"):
        raise RuntimeError("steering_policy must be nearest | coherent")
    # 팔 OpenCR 이 없으면 control_master 도 안 뜸 → 바퀴 차례(DRIVE) 허가를 줄 노드가 없으므로 차례를 보지 않음
    params = {"park_on_idle": park == "true", "steering_policy": policy, "require_motion_mode": not arm_off}
    if blend == "true":   # 동시 조향·구동 (arm:=false 시험용). 진행 막힘 1.0 s 넘으면 구동 0
        params.update(blend_steer_drive=True, blend_stall_s=float(context.perform_substitution(LaunchConfiguration("blend_stall_s"))),
                      blend_margin=float(context.perform_substitution(LaunchConfiguration("blend_margin"))))
    if test_path:
        test_params = yaml.safe_load(open(test_path))
        allowed = {"max_linear", "max_angular", "max_linear_accel", "max_angular_accel",
                   "max_wheel_speed", "steer_branch_margin", "steer_end_slack", "steer_travel_weight",
                   "heading_hold_kp", "heading_hold_max_wz", "heading_hold_deadband", "heading_hold_wz_per_speed",
                   "steer_coherent_max_travel", "steer_end_guard", "keep_branch_max_wz",
                   "wheel_speed_gain"}
        if not isinstance(test_params, dict) or set(test_params) - allowed:
            raise RuntimeError("unexpected drive_test_params keys")
        params.update(test_params)
    if ranges is None:
        note = "조향 영점 없음 → 모듈별 범위 없이 공통 steer_limit (mock 전용, 실물은 위에서 거부)"
    else:
        lo, hi = ranges
        params.update(steer_min=lo, steer_max=hi)
        if multi:
            params.update(steer_multi_turn=True)
        note = "모듈별 조향 범위 [deg] " + " · ".join(
            f"{j.split('_')[1]} {math.degrees(a):+.1f}~{math.degrees(b):+.1f}" for j, a, b in zip(STEER_JOINTS, lo, hi))
    fd, path_r = tempfile.mkstemp(prefix="nt_steer_ranges_", suffix=".yaml")
    with os.fdopen(fd, "w") as f:
        yaml.safe_dump({"swerve_controller": {"ros__parameters": params}}, f)
    files.append(path_r)
    note += f" · 정지 후 바깥쪽 복귀 {'켬' if park == 'true' else '끔'} ({path_r})"
    return [LogInfo(msg=("arm:=false — " if arm_off else "arm:=true — ") + note), spawner_node("swerve_controller", files)]


def cfg(name):
    return PathJoinSubstitution([FindPackageShare("bringup"), "config", name])


def generate_launch_description():
    desc = FindPackageShare("description")
    arg = LaunchConfiguration

    args = [
        DeclareLaunchArgument("hardware", default_value="mock", description="mock | opencr"),
        DeclareLaunchArgument("hardware_map", default_value=cfg("hardware_map.yaml")),
        DeclareLaunchArgument("opencr_port", default_value="auto",
                              description="hardware:=opencr 일 때 포트 — 보드 1장일 때만 (auto = hardware_map.yaml)"),
        DeclareLaunchArgument("opencr_ports", default_value="auto",
                              description="보드마다 포트: bus_drive=/tmp/opencr_drive,bus_arm=/tmp/opencr_arm (auto = hardware_map.yaml)"),
        DeclareLaunchArgument("mock_dynamics", default_value="false",
                              description="hardware:=mock 일 때 true = 조향 · 바퀴 속도 · 가속 한계 있는 가짜 모터"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("arm", default_value="true",
                              description="false = 팔 OpenCR 을 안 붙이고 바퀴만 (팔 관절이 없는 hardware_map 필요, 위 설명)"),
        DeclareLaunchArgument("sag_correction", default_value="true",
                              description="false = TF 에 팔 처짐 보정을 안 넣음 (엔코더 그대로, 실물 A/B 비교용)"),
        DeclareLaunchArgument("steering_policy", default_value="coherent",
                              description="coherent = 평행 이동은 공통 자세 우선 (기본) | nearest = 예전 최소 이동"),
        DeclareLaunchArgument("park_on_idle", default_value="false",
                              description="true = 정지 1 s 뒤 조향 0° (바퀴 바깥쪽) 로 복귀. arm:=false 에서만"),
        DeclareLaunchArgument("blend_steer_drive", default_value="false",
                              description="true = 움직이는 중 방향 전환 때 조향하면서 계속 주행 (동시 조향·구동). 기본 끔. arm:=false 에서만"),
        DeclareLaunchArgument("blend_stall_s", default_value="1.0",
                              description="동시 모드에서 조향이 못 따라와 진행이 막힌 시간이 이보다 길면 구동 0 [s]"),
        DeclareLaunchArgument("blend_margin", default_value="0.7",
                              description="동시 모드는 조향 허용 오차의 이 비율 안에서만 진행 (경계에서 게이트가 걸려 연결이 끊기는 것 방지)"),
        DeclareLaunchArgument("steer_margin_ticks", default_value="20"),
        DeclareLaunchArgument("enforce_command_limits", default_value="true",
                              description="ROS target limiter; false only for drive-only performance tests"),
        DeclareLaunchArgument("drive_test_params", default_value="",
                              description="arm:=false optional performance parameter YAML"),
        # 팔 받침 위치 — 실측 모델 값 (robot.urdf.xacro 기본값과 같음). 바꿔 볼 때만 인자로
        DeclareLaunchArgument("arm_mount_x", default_value="0.0225499941511"),   # URDF 실측 기본값과 같게 (0.0 이면 URDF 실측값을 덮어씀)
        DeclareLaunchArgument("arm_mount_y", default_value="0.0"),
        # 앞 라이다 장착 (base_footprint 기준, URDF lidar_front_joint) — 기본 = URDF 와 같음 (높이 실측 60 mm · 좌우 가운데 · 앞뒤 CAD ±114.5 mm)
        DeclareLaunchArgument("lidar_front_x", default_value="0.1145"),
        DeclareLaunchArgument("lidar_front_y", default_value="0.0"),
        DeclareLaunchArgument("lidar_front_z", default_value="0.060"),
        DeclareLaunchArgument("lidar_front_yaw", default_value="3.141592653589793"),
        DeclareLaunchArgument("lidar_rear_x", default_value="-0.1145"),
        DeclareLaunchArgument("lidar_rear_y", default_value="0.0"),
        DeclareLaunchArgument("lidar_rear_z", default_value="0.060"),
        DeclareLaunchArgument("lidar_rear_yaw", default_value="0.0"),
    ]
    is_opencr = PythonExpression(["'", arg("hardware"), "' == 'opencr'"])
    with_arm = PythonExpression(["'", arg("arm"), "' != 'false'"])

    robot_description = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
            PathJoinSubstitution([desc, "urdf", "robot.urdf.xacro"]),
            " hardware:=", arg("hardware"),
            " hardware_map:=", arg("hardware_map"),
            " opencr_port:=", arg("opencr_port"),
            " opencr_ports:=", arg("opencr_ports"),
            " mock_dynamics:=", arg("mock_dynamics"),
            " arm_mount_x:=", arg("arm_mount_x"),
            " arm_mount_y:=", arg("arm_mount_y"),
            " lidar_front_x:=", arg("lidar_front_x"), " lidar_front_y:=", arg("lidar_front_y"),
            " lidar_front_z:=", arg("lidar_front_z"), " lidar_front_yaw:=", arg("lidar_front_yaw"),
            " lidar_rear_x:=", arg("lidar_rear_x"), " lidar_rear_y:=", arg("lidar_rear_y"),
            " lidar_rear_z:=", arg("lidar_rear_z"), " lidar_rear_yaw:=", arg("lidar_rear_yaw"),
        ]),
        value_type=str,
    )
    ctrl_yaml = cfg("controllers.yaml")

    nodes = [
        # TF 는 실제 팔 자세로: arm_state_publisher 가 엔코더 /joint_states 에 처짐 보정 · 그리퍼 연결 막대를 더해 /joint_states_corrected
        #   (손목 카메라 보정이 처짐 보정한 손목 기준). control_master · MoveIt 은 엔코더 /joint_states 그대로
        Node(package="manipulation", executable="arm_state_publisher",
             parameters=[cfg("arm_kinematics.yaml"), cfg("gripper_linkage.yaml"),
                         {"sag_correction": ParameterValue(arg("sag_correction"), value_type=bool)}],
             respawn=True, respawn_delay=1.0, output="both"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": robot_description}], remappings=[("joint_states", "joint_states_corrected")],
             output="both"),

        # Lyrical 의 ros2_control_node 는 URDF 를 /robot_description 토픽으로 받음 (robot_state_publisher 가 방송)
        Node(package="controller_manager", executable="ros2_control_node",
             parameters=[ctrl_yaml, {"enforce_command_limits": ParameterValue(arg("enforce_command_limits"), value_type=bool)}], remappings=[("~/robot_description", "/robot_description")], output="both"),

        Node(package="manipulation", executable="control_master",
             parameters=[cfg("control.yaml"), cfg("arm_paths.yaml"), cfg("arm_kinematics.yaml")], condition=IfCondition(with_arm), output="both"),

        Node(package="rviz2", executable="rviz2", condition=IfCondition(arg("use_rviz")),
             arguments=["-d", PathJoinSubstitution([desc, "rviz", "navigation.rviz"]), "-f", "odom"]),
    ]

    # 컨트롤러마다 spawner 하나. --param-file: Jazzy 부터 컨트롤러가 controller_manager 설정을 안 물려받음
    def spawner(name, **kw):
        return spawner_node(name, [ctrl_yaml], **kw)

    for c in CONTROLLERS:
        if c in ARM_CONTROLLERS:
            nodes.append(spawner(c, condition=IfCondition(with_arm)))
        else:
            nodes.append(spawner(c))
    nodes.append(spawner("imu_sensor_broadcaster", condition=IfCondition(is_opencr)))   # IMU 는 hardware 플러그인만 채움

    # 검사가 노드보다 먼저 (거부되면 아무 노드도 안 뜸)
    return LaunchDescription(args + [OpaqueFunction(function=swerve_setup)] + nodes)
