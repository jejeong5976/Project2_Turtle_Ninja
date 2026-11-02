#!/usr/bin/env python3
"""Isaac Lab 시뮬 <-> ROS 2 (Lyrical) 브리지: Nav2 를 시뮬 로봇에 붙여 시험.

실행은 scripts/sim_nav_bridge.sh 로 (ROS 환경 + Isaac 파이썬). 경기장 = 발제 v5 주사위 경기장 (sim_core/dice_mission, 출발점에서 시작).
내보내는 것 (use_sim_time 기준 /clock):
  /clock                      시뮬 시각 (물리 0.005 s × 4 = 0.02 s 마다)
  /odom, TF odom -> base_link 정답 자세를 출발점 기준으로 (실물 swerve_controller 의 /odom 자리, 잡음 없음)
  /joint_states               모든 관절 (robot_state_publisher 가 나머지 TF)
  /lidar_front/scan, /lidar_rear/scan   레이캐스트 라이다 (360 · 1°, 0.05–8 m, 프레임 base_scan_front/rear, URDF 그대로 = 바닥 위 60 mm)
                              자기 몸(바퀴 · 조향 모듈 · 차체 기둥 · 반대쪽 라이다)에 가린 빔은 돌아오지 않음(inf) 처리 (sim_core/lidar_self.py)
  /sim/ground_truth           map(= 경기장 중심) 기준 정답 자세 (평가용)
  /initialpose                AMCL 이 /amcl_pose 를 낼 때까지 1 s 마다 정답 자세로 (시험 편의)
  /sim/traffic_light_truth    dice 신호등 정답 상태 std_msgs/String 'R' · 'Y' · 'G' · 'OFF' (10 Hz). scripts/fake_traffic_light.py 가 TrafficLight 로 바꿈
받는 것: /cmd_vel (geometry_msgs/TwistStamped, Nav2 기본) -> 배포판 스워브 컨트롤러 (SwerveAction). 0.5 s 끊기면 정지.
        /sim/person (geometry_msgs/PointStamped, map xy) -> 시험용 '사람' 상자 (0.25 × 0.35 × 1.6 m, 고정) 를 그 자리에 세움. z < 0 이면 치움
"""
import argparse
import math
import sys
import time
from pathlib import Path as _P
sys.path.insert(0, str(_P(__file__).resolve().parents[1]))   # sim/robot_sim -> sim_core

from isaaclab.app import add_launcher_args, launch_simulation

parser = argparse.ArgumentParser()
parser.add_argument('--arena', choices=['dice', 'real'], default='dice', help='dice = 발제 v5 주사위 경기장, real = 예전 A 경기장 미로')
parser.add_argument('--camera', action='store_true', help='광각 · 손목 카메라 붙이기 (창에서 보기용)')
parser.add_argument('--real', action='store_true', help='실제 배경 · 천장 조명 · 벽판 반사')
parser.add_argument('--no_realtime', action='store_true', help='실제 시간 맞춤 끄기 (기본은 시뮬 시각 = 벽시계)')
parser.add_argument('--scan_every', type=int, default=5, help='라이다 발행 간격 [스텝] (5 = 10 Hz)')
parser.add_argument('--signal_delay', type=float, default=5.0, help='dice: 시작 몇 초 뒤 신호등 점멸 (문제 파일 순서, view_dice 와 같음). 음수면 점멸 안 함')
parser.add_argument('--record', default=None, help='녹화 mp4 경로: 고정 카메라로 시뮬 시각 기준 영상 저장 (창 필요 없음)')
parser.add_argument('--record_view', default='-0.3,-1.2,2.4,-0.3,-2.6,0.0', help='녹화 카메라 위치 x,y,z,바라볼 점 x,y,z (map)')
parser.add_argument('--record_secs', type=float, default=60.0, help='녹화 길이 [시뮬 s] (끝나면 파일 닫음)')
parser.add_argument('--record_fps', type=int, default=25)
add_launcher_args(parser)
args = parser.parse_args()
if args.camera or args.record:
    args.enable_cameras = True

import torch

from isaaclab.utils import instantiate
from isaaclab.utils.math import quat_apply, quat_mul, quat_inv, yaw_quat

from sim_core import turtleninja_maze_env as tme

if args.arena == 'dice':
    from sim_core import dice_mission as dm
    cfg = dm.DiceMissionEnvCfg()
else:
    cfg = tme.TurtleNinjaMazeEnvCfg()
cfg.scene.num_envs = 1
if args.real:
    tme.use_real_look(cfg)
if args.camera:
    tme.add_birdeye_camera(cfg, width=1280, height=720)
    tme.add_wrist_d435(cfg, data_types=('rgb',))
# 시험용 '사람': 바닥에 선 고정 상자. 처음엔 경기장 밖에 두고 /sim/person 으로 옮김. 라이다가 맞히도록 레이캐스트 대상에 추가
import isaaclab.sim as sim_utils
from isaaclab.assets import RigidObjectCfg
PERSON_SIZE = (0.25, 0.35, 1.6)
PERSON_AWAY = (8.0, 8.0)
cfg.scene.person = RigidObjectCfg(
    prim_path='{ENV_REGEX_NS}/Person', init_state=RigidObjectCfg.InitialStateCfg(pos=(*PERSON_AWAY, PERSON_SIZE[2] / 2)),
    spawn=sim_utils.CuboidCfg(size=PERSON_SIZE, rigid_props=sim_utils.UsdPhysicsRigidBodyCfg(kinematic_enabled=True),
                              collision_props=sim_utils.UsdPhysicsCollisionCfg(),
                              visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.2, 0.3, 0.8))))
if args.record:   # 녹화용 고정 카메라 (경기장 기준 위치, env 밖)
    from isaaclab.sensors import CameraCfg
    import numpy as _np
    _v = _np.array([float(x) for x in args.record_view.split(',')])
    _x = (_v[3:] - _v[:3]) / _np.linalg.norm(_v[3:] - _v[:3])          # world 규약 카메라: x 앞 · z 위
    _y = _np.cross([0.0, 0.0, 1.0], _x); _y /= _np.linalg.norm(_y)
    _z = _np.cross(_x, _y)
    cfg.scene.record_cam = CameraCfg(prim_path='{ENV_REGEX_NS}/record_cam', width=1280, height=720, data_types=['rgb'],
                                     spawn=sim_utils.PinholeCameraCfg(focal_length=14.0, clipping_range=(0.05, 30.0)),
                                     offset=CameraCfg.OffsetCfg(pos=tuple(_v[:3]), rot=tme._mat_to_quat_xyzw(_np.stack([_x, _y, _z], 1).tolist()),
                                                                convention='world'))
tme.add_lidars(cfg, debug_vis=False)
from isaaclab.visualizers import VisualizerCfg
cfg.sim.default_visualizer_cfg = VisualizerCfg(eye=(0.0, -6.5, 9.0), lookat=(0.0, 0.0, 0.0))   # 창 시점 = view_dice 와 같음
from isaaclab.sensors import MultiMeshRayCasterCfg
for _side in ('front', 'rear'):
    getattr(cfg.scene, f'lidar_{_side}').mesh_prim_paths.append(
        MultiMeshRayCasterCfg.RaycastTargetCfg(prim_expr='{ENV_REGEX_NS}/Person', track_mesh_transforms=True))

N_BEAMS = 360
RANGE_MIN, RANGE_MAX = 0.05, tme.LIDAR_MAX_M


from sim_core.lidar_self import SelfOcclusion
SELF = SelfOcclusion(tme.ROBOT_URDF)
STEER_NAMES = [f'steer_{m}_joint' for m in ('fl', 'fr', 'rl', 'rr')]


def scan_offset(side):
    """base_link 기준 base_scan_<side> (위치, 회전행렬) = URDF 그대로."""
    t, R = tme.urdf_link_pose(tme.ROBOT_URDF, f'base_scan_{side}')
    return torch.tensor(t, dtype=torch.float32), torch.tensor(R, dtype=torch.float32)


with launch_simulation(cfg, args):
    env = instantiate(cfg)
    import omni.usd
    stage = omni.usd.get_context().get_stage()
    tme.apply_robot_real_colors(stage)
    crank = tme.add_crank_visuals(stage)
    if args.real:
        # 실제 겉모습 = view_dice / view_maze --real 과 같음: 천장 조명 + 벽판 유광 반사 (실제 영상 반사율 0.13 에 맞춘 굴절률 2.6, 거칠기 0.08)
        from pxr import Sdf
        tme.add_ceiling_lights(stage, env.scene.env_origins)
        sh = stage.GetPrimAtPath('/World/envs/env_0/Maze/Looks/panel/Surface')
        if sh:
            sh.CreateAttribute('inputs:ior', Sdf.ValueTypeNames.Float).Set(2.6)
            sh.CreateAttribute('inputs:roughness', Sdf.ValueTypeNames.Float).Set(0.08)
    # 신호등 점멸 (dice 문제 파일 순서, view_dice 와 같음)
    player = None
    if args.arena == 'dice' and args.signal_delay >= 0:
        seq = dm.LAYOUT['problem']['signal']
        player = dm.SignalPlayer(stage, 1, seq)
    rec = None
    if args.record:
        import cv2
        # Isaac 파이썬 OpenCV 에는 mp4 인코더가 없음 -> 프레임 JPEG 저장, 끝날 때 시스템 ffmpeg 로 mp4
        import pathlib
        import shutil
        import subprocess
        rec_dir = pathlib.Path(args.record + '_frames')
        shutil.rmtree(rec_dir, ignore_errors=True)
        rec_dir.mkdir(parents=True)
        rec = [0]
        rec_every = max(1, round(1.0 / (args.record_fps * cfg.sim.dt * cfg.decimation)))

    # ---- ROS (시스템 Lyrical rclpy) ----
    import rclpy
    from builtin_interfaces.msg import Time
    from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped, TransformStamped, TwistStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import String
    from sensor_msgs.msg import JointState, LaserScan
    from tf2_msgs.msg import TFMessage

    rclpy.init()
    node = Node('sim_nav_bridge')
    sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
    pub_clock = node.create_publisher(Clock, '/clock', 10)
    pub_odom = node.create_publisher(Odometry, '/odom', 10)
    pub_tf = node.create_publisher(TFMessage, '/tf', 50)
    pub_js = node.create_publisher(JointState, '/joint_states', 10)
    pub_gt = node.create_publisher(Odometry, '/sim/ground_truth', 10)
    pub_init = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
    pub_tl_truth = node.create_publisher(String, '/sim/traffic_light_truth', 10)
    pub_scan = {s: node.create_publisher(LaserScan, f'/lidar_{s}/scan', sensor_qos) for s in ('front', 'rear')}
    last_cmd = {'v': (0.0, 0.0, 0.0), 't': -1.0}
    amcl_ok = [False]

    def on_cmd(m):
        last_cmd['v'] = (m.twist.linear.x, m.twist.linear.y, m.twist.angular.z)
        last_cmd['t'] = sim_t
    node.create_subscription(TwistStamped, '/cmd_vel', on_cmd, 10)
    node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', lambda m: amcl_ok.__setitem__(0, True), 10)
    person_req = []
    node.create_subscription(PointStamped, '/sim/person', lambda m: person_req.append((m.point.x, m.point.y, m.point.z)), 10)
    cm_state = ['']                                                   # 녹화 글자용: 비상 정지 상태
    try:
        from nav2_msgs.msg import CollisionMonitorState
        _CM = {0: '', 1: 'STOP', 2: 'SLOWDOWN', 3: 'APPROACH', 4: 'LIMIT'}
        node.create_subscription(CollisionMonitorState, 'collision_monitor_state', lambda m: cm_state.__setitem__(0, _CM.get(m.action_type, '')), 10)
    except ImportError:
        pass

    robot = env.scene['robot']
    dev = env.device
    org = env.scene.env_origins[0]
    names = list(robot.joint_names)
    term = env.action_manager.get_term('base')
    offs = {s: tuple(v.to(dev) for v in scan_offset(s)) for s in ('front', 'rear')}
    # 자기 몸 가림: 라이다별 빔 방향 (base_link 기준) · 원점, 고정 부분 가림 거리는 한 번만 계산
    import numpy as np
    beam_scan = -math.pi + np.arange(N_BEAMS) * 2 * math.pi / N_BEAMS
    occ = {}
    for s_ in ('front', 'rear'):
        T = SELF._fk(f'base_scan_{s_}')
        yaw = math.atan2(T[1, 0], T[0, 0])
        o_, a_ = T[:2, 3], beam_scan + yaw
        skip = (f'lidar_{s_}_link',)
        occ[s_] = (o_, a_, skip, SELF.ray_hits(o_, a_, SELF.segments({}, skip, moving=False)))
    steer_ids = robot.find_joints(STEER_NAMES, preserve_order=True)[0]
    person = env.scene['person']
    act = torch.zeros(1, env.action_manager.total_action_dim, device=dev)
    dt = cfg.sim.dt * cfg.decimation

    def stamp(t):
        ns = int(round(t * 1e9))                     # 정수 ns 로 바꾼 뒤 나눔 (소수부 반올림이 10⁹ 가 돼 시각이 1 s 뒤로 가는 것 방지)
        return Time(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)

    def pose_now():
        """env 원점 기준 base_link (위치 (3,), 쿼터니언 (x, y, z, w))"""
        return robot.data.root_pos_w.torch[0] - org, robot.data.root_quat_w.torch[0]

    env.reset()
    for _ in range(10):
        env.step(act)
    p0, q0 = pose_now()
    q0 = yaw_quat(q0[None])[0]                                   # odom 원점 = 출발 자세 (바닥 위, yaw 만)
    p0 = torch.tensor([p0[0], p0[1], 0.0], device=dev)
    sim_t, wall0, step, last_init = 0.0, time.monotonic(), 0, -10.0
    if player is not None:
        player.start(args.signal_delay)
    print(f'[bridge] 경기장 {args.arena}, 출발 map ({p0[0]:.3f}, {p0[1]:.3f}) yaw {math.degrees(2 * math.atan2(q0[2], q0[3])):.1f}°. '
          f'토픽 /clock /odom /tf /joint_states /lidar_front/scan /lidar_rear/scan /sim/ground_truth, 받기 /cmd_vel'
          + (f', 신호 {dm.LAYOUT["problem"]["signal"]} ({args.signal_delay} s 뒤)' if player is not None else ''), flush=True)

    def make_scan(side, p_w, q_w, t):
        o, R = offs[side]
        so = p_w + quat_apply(q_w[None], o[None])[0]                # 센서 원점 (월드)
        data = env.scene[f'lidar_{side}'].data
        hits = data.ray_hits_w.torch[0] if hasattr(data.ray_hits_w, 'torch') else data.ray_hits_w[0]
        ok = torch.isfinite(hits).all(-1)
        rel = quat_apply(quat_inv(q_w)[None].expand(hits.shape[0], 4), hits - so)   # base_link 축
        rel = rel @ R                                               # base_scan 축 (R = base_link <- scan)
        ang = torch.atan2(rel[:, 1], rel[:, 0])
        rng = torch.hypot(rel[:, 0], rel[:, 1])
        ok &= (rng >= RANGE_MIN) & (rng <= RANGE_MAX)
        idx = ((ang + math.pi) / (2 * math.pi) * N_BEAMS).long().clamp(0, N_BEAMS - 1)
        ranges = torch.full((N_BEAMS,), float('inf'), device=dev)
        ranges.scatter_reduce_(0, idx[ok], rng[ok], reduce='amin')
        # 자기 몸에 먼저 맞는 빔 = 돌아오지 않음 (그 뒤 그림자 포함)
        o_, a_, skip, t_static = occ[side]
        q = dict(zip(STEER_NAMES, robot.data.joint_pos.torch[0, steer_ids].cpu().tolist()))
        t_self = np.minimum(t_static, SELF.ray_hits(o_, a_, SELF.segments(q, skip, moving=True)))
        ranges = ranges.cpu().numpy()
        ranges[t_self < ranges] = float('inf')
        m = LaserScan()
        m.header.stamp, m.header.frame_id = stamp(t), f'base_scan_{side}'
        m.angle_min, m.angle_increment = -math.pi, 2 * math.pi / N_BEAMS
        m.angle_max = m.angle_min + m.angle_increment * (N_BEAMS - 1)
        m.scan_time, m.range_min, m.range_max = dt * args.scan_every, RANGE_MIN, RANGE_MAX
        m.ranges = ranges.tolist()
        return m

    with torch.inference_mode():
        while env.sim.is_running() if hasattr(env.sim, 'is_running') else True:
            rclpy.spin_once(node, timeout_sec=0.0)
            while person_req:                                         # 시험용 사람 옮기기 (map = env 원점 기준)
                px, py, pz = person_req.pop(0)
                if pz < 0:
                    px, py = PERSON_AWAY
                pose = torch.tensor([[px, py, PERSON_SIZE[2] / 2, 0.0, 0.0, 0.0, 1.0]], device=dev)
                pose[0, :3] += org
                person.write_root_pose_to_sim_index(root_pose=pose, env_ids=torch.tensor([0], device=dev))
                print(f'[bridge] 사람 -> ({px:.2f}, {py:.2f})', flush=True)
            vx, vy, wz = last_cmd['v'] if sim_t - last_cmd['t'] < 0.5 else (0.0, 0.0, 0.0)
            act[0, 0], act[0, 1], act[0, 2] = vx / term.cfg.max_linear, vy / term.cfg.max_linear, wz / term.cfg.max_angular
            if player is not None:
                player.update(sim_t)
            env.step(act)
            tme.update_crank_visuals(crank, robot)
            step += 1
            sim_t += dt
            ts = stamp(sim_t)
            pub_clock.publish(Clock(clock=ts))
            p_w, q_w = pose_now()
            # odom = 출발 자세 기준
            qi = quat_inv(q0[None])[0]
            po = quat_apply(qi[None], (p_w - p0)[None])[0]
            qo = quat_mul(qi[None], q_w[None])[0]
            v_b = robot.data.root_lin_vel_b.torch[0]
            w_b = robot.data.root_ang_vel_b.torch[0]
            od = Odometry()
            od.header.stamp, od.header.frame_id, od.child_frame_id = ts, 'odom', 'base_link'
            (od.pose.pose.position.x, od.pose.pose.position.y, od.pose.pose.position.z) = (float(v) for v in po)
            (od.pose.pose.orientation.x, od.pose.pose.orientation.y, od.pose.pose.orientation.z, od.pose.pose.orientation.w) = (float(v) for v in qo)
            od.twist.twist.linear.x, od.twist.twist.linear.y, od.twist.twist.angular.z = float(v_b[0]), float(v_b[1]), float(w_b[2])
            pub_odom.publish(od)
            tf = TransformStamped()
            tf.header.stamp, tf.header.frame_id, tf.child_frame_id = ts, 'odom', 'base_link'
            tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = (float(v) for v in po)
            tf.transform.rotation = od.pose.pose.orientation
            pub_tf.publish(TFMessage(transforms=[tf]))
            gt = Odometry()
            gt.header.stamp, gt.header.frame_id, gt.child_frame_id = ts, 'map', 'base_link'
            (gt.pose.pose.position.x, gt.pose.pose.position.y, gt.pose.pose.position.z) = (float(v) for v in p_w)
            (gt.pose.pose.orientation.x, gt.pose.pose.orientation.y, gt.pose.pose.orientation.z, gt.pose.pose.orientation.w) = (float(v) for v in q_w)
            pub_gt.publish(gt)
            js = JointState()
            js.header.stamp, js.name = ts, names
            js.position = robot.data.joint_pos.torch[0].cpu().tolist()
            js.velocity = robot.data.joint_vel.torch[0].cpu().tolist()
            pub_js.publish(js)
            if player is not None and step % 5 == 0:          # 신호등 정답 상태 10 Hz
                c_, _ = player.state(sim_t)
                pub_tl_truth.publish(String(data=c_ if c_ else 'OFF'))
            if step % args.scan_every == 0:
                for s in ('front', 'rear'):
                    pub_scan[s].publish(make_scan(s, p_w, q_w, sim_t))
            if not amcl_ok[0] and sim_t - last_init >= 1.0:
                ip = PoseWithCovarianceStamped()
                ip.header.stamp, ip.header.frame_id = ts, 'map'
                ip.pose.pose = gt.pose.pose
                ip.pose.covariance[0] = ip.pose.covariance[7] = 0.0025
                ip.pose.covariance[35] = 0.003
                pub_init.publish(ip)
                last_init = sim_t
            if rec is not None and step % rec_every == 0:
                img = env.scene['record_cam'].data.output['rgb'][0, ..., :3].cpu().numpy()
                img = cv2.cvtColor(img.astype('uint8'), cv2.COLOR_RGB2BGR)
                sp = float(robot.data.root_lin_vel_b.torch[0, :2].norm())
                cv2.putText(img, f't {sim_t:5.1f} s   speed {sp:.2f} m/s', (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
                if cm_state[0]:
                    cv2.putText(img, f'collision_monitor: {cm_state[0]}', (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                                (0, 0, 255) if cm_state[0] == 'STOP' else (0, 200, 255), 2)
                cv2.imwrite(str(rec_dir / f'{rec[0]:05d}.jpg'), img, [cv2.IMWRITE_JPEG_QUALITY, 92])
                rec[0] += 1
                if sim_t >= args.record_secs:
                    rec = None
                    r = subprocess.run(['ffmpeg', '-loglevel', 'error', '-y', '-framerate', str(args.record_fps), '-i', str(rec_dir / '%05d.jpg'),
                                        '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-crf', '20', args.record], env={'PATH': '/usr/bin:/bin'})
                    if r.returncode == 0:
                        shutil.rmtree(rec_dir, ignore_errors=True)
                    print(f'[bridge] 녹화 끝: {args.record} (ffmpeg {r.returncode})', flush=True)
            if not args.no_realtime:
                lag = sim_t - (time.monotonic() - wall0)
                if lag > 0:
                    time.sleep(lag)
    node.destroy_node()
    rclpy.shutdown()
    env.close()
