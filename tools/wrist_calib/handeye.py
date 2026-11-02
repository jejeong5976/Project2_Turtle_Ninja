#!/usr/bin/env python3
"""손목 D435 핸드-아이 실측 (eye-in-hand): 카메라가 손목 링크(arm_link5_wrist_roll)에 어떻게 붙어 있나.

준비: 정답 종이(make_gt_sheet.py 로 만든 gt_sheet.pdf, 100 % 인쇄)를 로봇 앞 바닥 · 책상에 테이프로 고정. 로봇 차체는 끝까지 안 움직임 (팔만 움직임)
1) capture : 팔을 자세마다 멈추고 D435 영상 + 관절값 저장 (s 키). 10–20 자세, 회전을 크게 다양하게 (J1 좌우 · J4 숙임 · J5 비틀기)
   관절값 받는 법 (--joints)
     ros   : /joint_states 구독 (실물 이름 joint1..5 -> URDF arm_joint1..5, --ros_offsets 로 0 점 · 부호 보정). 먼저 source /opt/ros/lyrical/setup.bash
     ticks : 화면에서 s 누르면 터미널에 Dynamixel Present Position 5 개 입력 (ID 11 12 13 14 16) -> 배포판 config/joint_map.json 식으로 변환
     urdf  : 터미널에 URDF 관절값 5 개 (rad) 직접 입력
2) solve   : 저장한 자세들 -> 재투영 오차 최소화 (카메라 X = 손목->카메라, 종이 Y = base->종이 함께, 출발점 = URDF × 경첩 여러 각)
   결과: 손목 기준 카메라 자세, URDF(경첩 −12° 반영) 와 차이 (경첩 각도 = 영상 x 축 둘레 회전, 그 밖 회전, 위치), 광축 ↔ 손가락 방향 각,
         URDF 에 넣을 d435_cad_front_fixed origin, 자세별 재투영 오차
3) selftest: 가짜 자세 · 잡음으로 solve 가 정답을 되찾는지 확인 (카메라 없이)

예) cd tools/wrist_calib
    python3 handeye.py capture --joints ticks
    python3 handeye.py solve datasets/<세션>
    python3 handeye.py selftest
"""
import argparse
import json
import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
ROBOT = REPO / 'sim' / 'robot_sim'
sys.path.insert(0, str(ROBOT))
from sim_core import arm_poses as ap   # noqa: E402  (numpy 만 씀)

URDF = ROBOT / 'robot_description' / 'robot.urdf'
JOINT_MAP = ROBOT / 'robot_description' / 'config' / 'joint_map.json'
WRIST, CAM, CAD = 'arm_link5_wrist_roll', 'd435_color_optical_frame', 'd435_cad_front_frame'
ARM = ap.ARM_JOINTS
OUT_ROOT = HERE / 'datasets'                   # 촬영 저장 (커밋 안 함)
MIN_MARKERS = 6


# ---------- 공통 ----------
def load_sheet():
    g = json.loads((HERE / 'gt_sheet.json').read_text())
    return {m['id']: np.array(m['corners_m'], np.float64) for m in g['markers']}


def detect_sheet(gray, markers):
    """종이 마커 -> (물체점 (N, 3), 영상점 (N, 2), 마커 수)"""
    p = cv2.aruco.DetectorParameters()
    p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    cs, ids, _ = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100), p).detectMarkers(gray)
    if ids is None:
        return None, None, 0
    pairs = [(markers[int(i)], c.reshape(4, 2)) for c, i in zip(cs, ids.ravel()) if int(i) in markers]
    if not pairs:
        return None, None, 0
    return np.concatenate([a for a, _ in pairs]), np.concatenate([b for _, b in pairs]).astype(np.float64), len(pairs)


def pnp(obj, img, K):
    _, r, t = cv2.solvePnP(obj, img, K, None, flags=cv2.SOLVEPNP_ITERATIVE)
    proj = cv2.projectPoints(obj, r, t, K, None)[0].reshape(-1, 2)
    return cv2.Rodrigues(r)[0], t.ravel(), float(np.sqrt(np.mean(np.sum((proj - img) ** 2, 1))))


def Tm(R, t):
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, t
    return T


def wrist_pose(fk, q):
    t, R = ap.link_pose(fk, q, WRIST)
    return Tm(R, t)


def urdf_cam_in_wrist(fk):
    """URDF 의 손목 -> 컬러 광학 프레임 (4×4), 손목 -> CAD 앞면 프레임, CAD 앞면 -> 컬러 광학 (고정)"""
    tw, Rw = ap.link_pose(fk, {}, WRIST)
    tc, Rc = ap.link_pose(fk, {}, CAM)
    tf, Rf = ap.link_pose(fk, {}, CAD)
    Tw = Tm(Rw, tw)
    return np.linalg.inv(Tw) @ Tm(Rc, tc), np.linalg.inv(Tw) @ Tm(Rf, tf), np.linalg.inv(Tm(Rf, tf)) @ Tm(Rc, tc)


def rpy_of(R):
    """URDF rpy (R = Rz(y) Ry(p) Rx(r))"""
    p = math.asin(max(-1.0, min(1.0, -R[2, 0])))
    r = math.atan2(R[2, 1], R[2, 2])
    y = math.atan2(R[1, 0], R[0, 0])
    return r, p, y


def ticks_to_q(ticks):
    jm = json.loads(JOINT_MAP.read_text())
    q = {}
    for e, tk in zip(jm['arm'], ticks):
        q[e['joint']] = math.radians(e['q_deg_at_ref']) + e['sign'] * (tk - e['ref_tick']) * 2 * math.pi / 4096
    return q


# ---------- 1) capture ----------
class JointSource:
    def __init__(self, mode, offsets):
        self.mode, self.latest = mode, None
        if mode == 'ros':
            import rclpy
            from sensor_msgs.msg import JointState
            rclpy.init()
            self.rclpy, self.node = rclpy, rclpy.create_node('handeye_capture')
            self.off = offsets

            def cb(m):
                d = dict(zip(m.name, m.position))
                if all(f'joint{i}' in d for i in range(1, 6)):
                    self.latest = ({f'arm_joint{i}': self.off[i - 1][1] * d[f'joint{i}'] + self.off[i - 1][0] for i in range(1, 6)},
                                   {f'joint{i}': d[f'joint{i}'] for i in range(1, 6)})
            self.node.create_subscription(JointState, '/joint_states', cb, 10)

    def spin(self):
        if self.mode == 'ros':
            self.rclpy.spin_once(self.node, timeout_sec=0.0)

    def read(self):
        """-> (URDF 관절 dict, 원래 값) 또는 None"""
        if self.mode == 'ros':
            return self.latest
        prompt = ('모터 Present Position 틱 값 5 개 (모터 ID 11 → 12 → 13 → 14 → 16 순서, 보통 0–4095 숫자, 예: 4088 2154 1624 1897 2018): '
                  if self.mode == 'ticks' else 'URDF 관절 5 개 rad (arm_joint1..5): ')
        try:
            v = [float(x) for x in input(prompt).split()]
        except (ValueError, EOFError):
            return None
        if len(v) != 5:
            print('5 개가 아님 -> 저장 안 함')
            return None
        if self.mode == 'ticks' and (v == [11, 12, 13, 14, 16] or max(abs(x) for x in v) < 100):
            print('모터 ID 나 너무 작은 값 -> 틱 값이 아님. 저장 안 함 (Dynamixel Wizard · 제어 로그의 Present Position)')
            return None
        return (ticks_to_q(v), {'ticks': v}) if self.mode == 'ticks' else ({n: x for n, x in zip(ARM, v)}, {'urdf_rad': v})


def capture(a):
    sys.path.insert(0, str(REPO / 'tools' / 'd435_viewer'))
    from d435_puck_viewer import Viewer       # Tk 창 (YOLO venv OpenCV 는 창 기능 없음)
    import pyrealsense2 as rs
    markers = load_sheet()
    offs = [tuple(float(x) for x in s.split(':')) for s in a.ros_offsets.split(',')]
    js = JointSource(a.joints, offs)
    out = OUT_ROOT / time.strftime('%m%d_%H%M%S')
    out.mkdir(parents=True, exist_ok=True)
    pipe, cfg = rs.pipeline(), rs.config()
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    prof = pipe.start(cfg)
    ci = prof.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    K = np.array([[ci.fx, 0, ci.ppx], [0, ci.fy, ci.ppy], [0, 0, 1.0]])
    align, scale = rs.align(rs.stream.color), prof.get_device().first_depth_sensor().get_depth_scale()
    print(f'저장: {out}\n팔을 멈춘 뒤 창을 클릭하고 s = 저장, q = 끝. 마커 {MIN_MARKERS} 개 이상 보일 때만 저장', flush=True)
    viewer, n = None, 0
    try:
        for _ in range(30):
            pipe.wait_for_frames()
        while True:
            fs = align.process(pipe.wait_for_frames())
            bgr = np.asanyarray(fs.get_color_frame().get_data()).copy()
            depth = np.asanyarray(fs.get_depth_frame().get_data()).astype(np.float32) * scale
            js.spin()
            obj, img, nm = detect_sheet(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY), markers)
            vis = bgr.copy()
            txt = f'markers {nm}  saved {n}'
            if nm >= 2:
                R, t, rep = pnp(obj, img, K)
                cv2.drawFrameAxes(vis, K, np.zeros(5), cv2.Rodrigues(R)[0], t, 0.05, 2)
                txt += f'  reproj {rep:.2f}px  dist {t[2]:.3f}m'
                for p in img:
                    cv2.circle(vis, tuple(int(x) for x in p), 2, (0, 255, 0), -1)
            if js.mode == 'ros':
                txt += '  joints ' + ('OK' if js.latest else 'none')
            cv2.putText(vis, txt, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
            if viewer is None:
                viewer = Viewer('handeye capture (s 저장, q 끝)')
            k = viewer.show(vis)
            if k == 'q':
                break
            if k == 's':
                if nm < MIN_MARKERS:
                    print(f'마커 {nm} 개 < {MIN_MARKERS} -> 저장 안 함', flush=True)
                    continue
                jr = js.read()
                if jr is None:
                    print('관절값 없음 -> 저장 안 함', flush=True)
                    continue
                d = out / f'{n:02d}'
                d.mkdir()
                cv2.imwrite(str(d / 'color.png'), bgr)
                np.save(d / 'depth_m.npy', depth)
                (d / 'intrinsics.json').write_text(json.dumps({'K': K.tolist(), 'coeffs': list(ci.coeffs)}))
                (d / 'joints.json').write_text(json.dumps({'urdf_q': jr[0], 'raw': jr[1], 'source': a.joints}, indent=1))
                n += 1
                print(f'[{n}] 저장 {d.name}  마커 {nm}  관절 {[round(jr[0][j], 3) for j in ARM]}', flush=True)
    finally:
        pipe.stop()
        if viewer is not None:
            viewer.close()
    print(f'{n} 자세 저장 -> handeye.py solve {out}')


# ---------- 2) solve ----------
def refine(samples, K, X0, Y0):
    """재투영 오차 최소화: X = 손목->카메라, Y = base->종이 (각 6 자유도). samples = [(T_base_wrist, obj, img)]"""
    from scipy.optimize import least_squares

    def unpack(p):
        return Tm(cv2.Rodrigues(p[0:3])[0], p[3:6]), Tm(cv2.Rodrigues(p[6:9])[0], p[9:12])

    def resid(p):
        X, Y = unpack(p)
        out = []
        for Tbw, obj, img in samples:
            Tcs = np.linalg.inv(Tbw @ X) @ Y                   # 카메라 <- 종이
            pr = cv2.projectPoints(obj, cv2.Rodrigues(Tcs[:3, :3])[0], Tcs[:3, 3], K, None)[0].reshape(-1, 2)
            out.append((pr - img).ravel())
        return np.concatenate(out)

    p0 = np.concatenate([cv2.Rodrigues(X0[:3, :3])[0].ravel(), X0[:3, 3], cv2.Rodrigues(Y0[:3, :3])[0].ravel(), Y0[:3, 3]])
    r = least_squares(resid, p0, loss='huber', f_scale=1.0)
    X, Y = unpack(r.x)
    return X, Y, r


def hinge_T(deg):
    a = math.radians(deg)
    return Tm(np.array([[1, 0, 0], [0, math.cos(a), -math.sin(a)], [0, math.sin(a), math.cos(a)]]), np.zeros(3))


def solve_samples(samples, K, X_prior, verbose=True):
    """samples = [(T_base_wrist, obj, img)] -> (X, Y, 자세별 재투영 RMS).
    OpenCV 5.0 에는 calibrateHandEye 가 없어서: 출발점 = URDF 카메라 × 경첩 −60 … +20° (10° 간격),
    종이 Y 초기값 = 자세별 PnP 로 구한 base->종이 평균, 각각 재투영 오차 최소화 -> 비용 가장 작은 것"""
    pn = [pnp(obj, img, K) for _, obj, img in samples]
    best = None
    for deg in range(-60, 21, 10):
        X0 = X_prior @ hinge_T(deg)
        Ys = [Tbw @ X0 @ Tm(R, t) for (Tbw, _, _), (R, t, _) in zip(samples, pn)]
        Y0 = Tm(Ys[0][:3, :3], np.mean([y[:3, 3] for y in Ys], 0))
        X, Y, r = refine(samples, K, X0, Y0)
        if verbose:
            print(f'  출발 경첩 {deg:+3d}° -> 비용 {r.cost:10.1f}')
        if best is None or r.cost < best[2].cost:
            best = (X, Y, r)
    X, Y, r = best
    per, k = [], 0
    for _, obj, _ in samples:
        n = len(obj) * 2
        e = r.fun[k:k + n].reshape(-1, 2)
        per.append(float(np.sqrt(np.mean(np.sum(e ** 2, 1)))))
        k += n
    return X, Y, per, None


def report(X, Y, per, fk, names=None):
    Xu, Xcad_u, T_cad_cam = urdf_cam_in_wrist(fk)
    dR = Xu[:3, :3].T @ X[:3, :3]                              # URDF 카메라 -> 실측 카메라 (광학 좌표)
    rv = cv2.Rodrigues(dR)[0].ravel()
    ang = math.degrees(np.linalg.norm(rv))
    about_x = math.degrees(math.atan2(dR[2, 1], dR[1, 1]))     # 광학 x(경첩 축) 둘레 성분
    Rx = np.array([[1, 0, 0], [0, math.cos(math.radians(about_x)), -math.sin(math.radians(about_x))],
                   [0, math.sin(math.radians(about_x)), math.cos(math.radians(about_x))]])
    rest = math.degrees(np.linalg.norm(cv2.Rodrigues(Rx.T @ dR)[0]))
    dpos = (X[:3, 3] - Xu[:3, 3]) * 1000
    z = X[:3, 2]
    axis_vs_finger = math.degrees(math.acos(max(-1.0, min(1.0, z @ np.array([0, 0, -1.0])))))
    print('\n== 결과 (손목 arm_link5_wrist_roll 기준 컬러 광학 프레임)')
    print(f'  위치 (mm)          실측 {np.round(X[:3, 3] * 1000, 1)}   URDF {np.round(Xu[:3, 3] * 1000, 1)}   차이 {np.round(dpos, 1)} (|{np.linalg.norm(dpos):.1f}|)')
    print(f'  회전 차이          전체 {ang:.1f}°  = 경첩 축(광학 x) 둘레 {about_x:+.1f}° (− = 그리퍼 쪽, URDF 대비 추가분 = find_look_pose --tilt_deg)  + 그 밖 {rest:.1f}°')
    print(f'  광축 ↔ 손가락 방향(손목 −z)  실측 {axis_vs_finger:.1f}°  (지금 URDF 31.1°, 배포판 camera40 43.1°)')
    print(f'  자세별 재투영 RMS (px): {", ".join(f"{(names[i] if names else i)}:{e:.2f}" for i, e in enumerate(per))}')
    print(f'  종이 위치 (base 기준 mm) {np.round(Y[:3, 3] * 1000, 1)},  종이 법선(base) {np.round(-Y[:3, 2], 3)}  (책상 · 바닥이면 ≈ (0, 0, 1))')
    Xcad = X @ np.linalg.inv(T_cad_cam)                         # 손목 -> CAD 앞면 (URDF d435_cad_front_fixed origin)
    r, p, y = rpy_of(Xcad[:3, :3])
    print(f'\n  URDF d435_cad_front_fixed origin 제안:  xyz="{Xcad[0, 3]:.5f} {Xcad[1, 3]:.5f} {Xcad[2, 3]:.5f}"  rpy="{r:.5f} {p:.5f} {y:.5f}"')
    u = rpy_of(Xcad_u[:3, :3])
    print(f'  (지금 URDF:                              xyz="{Xcad_u[0, 3]:.5f} {Xcad_u[1, 3]:.5f} {Xcad_u[2, 3]:.5f}"  rpy="{u[0]:.5f} {u[1]:.5f} {u[2]:.5f}")')
    return dict(cam_in_wrist=X.tolist(), sheet_in_base=Y.tolist(), hinge_deg=about_x, other_rot_deg=rest, pos_diff_mm=dpos.tolist(),
                axis_vs_finger_deg=axis_vs_finger, reproj_px=per, cad_front_xyz=Xcad[:3, 3].tolist(), cad_front_rpy=[r, p, y])


def solve(a):
    fk = ap.ArmFK(URDF)
    markers = load_sheet()
    samples, names, K = [], [], None
    for d in sorted(Path(a.session).glob('[0-9]*')):
        if not (d / 'joints.json').exists():
            continue
        K = np.array(json.loads((d / 'intrinsics.json').read_text())['K'])
        obj, img, nm = detect_sheet(cv2.imread(str(d / 'color.png'), cv2.IMREAD_GRAYSCALE), markers)
        if nm < MIN_MARKERS:
            print(f'{d.name}: 마커 {nm} 개 -> 뺌')
            continue
        q = json.loads((d / 'joints.json').read_text())['urdf_q']
        samples.append((wrist_pose(fk, q), obj, img))
        names.append(d.name)
    print(f'자세 {len(samples)} 개')
    if len(samples) < 4:
        print('최소 4 자세 필요 (권장 10–20)')
        return
    rots = [cv2.Rodrigues(s[0][:3, :3])[0].ravel() for s in samples]
    spread = max(math.degrees(np.linalg.norm(cv2.Rodrigues(cv2.Rodrigues(r1)[0].T @ cv2.Rodrigues(r2)[0])[0])) for r1 in rots for r2 in rots)
    print(f'손목 회전 최대 차이 {spread:.0f}° (30° 미만이면 결과 흔들림 -> J1 · J4 · J5 를 더 크게 바꿀 것)')
    X, Y, per, _ = solve_samples(samples, K, urdf_cam_in_wrist(fk)[0])
    res = report(X, Y, per, fk, names)
    bad = [names[i] for i, e in enumerate(per) if e > 3 * np.median(per) and e > 2.0]
    if bad:
        print(f'  재투영이 큰 자세 {bad} -> 관절값 · 영상 시각이 안 맞았을 수 있음 (팔이 멈추기 전 저장 등). 빼고 다시 해 볼 것')
    (Path(a.session) / 'handeye_result.json').write_text(json.dumps({**res, 'poses': names}, indent=1))
    print(f'\n저장: {Path(a.session) / "handeye_result.json"}')


# ---------- 3) selftest ----------
def selftest(a):
    """가짜 정답 X (URDF + 경첩 −25° · 위치 +8 mm) · 종이 Y 로 영상점을 만들고 잡음 0.3 px 를 넣어 되찾는지"""
    rng = np.random.default_rng(0)
    fk = ap.ArmFK(URDF)
    markers = load_sheet()
    obj_all = np.concatenate(list(markers.values()))
    K = np.array([[605.9, 0, 324.0], [0, 605.4, 245.9], [0, 0, 1.0]])
    Xu, _, _ = urdf_cam_in_wrist(fk)
    a_ = math.radians(-25)
    Xt = Xu @ Tm(np.array([[1, 0, 0], [0, math.cos(a_), -math.sin(a_)], [0, math.sin(a_), math.cos(a_)]]), np.array([0.004, -0.006, 0.003]))
    Yt = Tm(np.diag([1.0, -1.0, -1.0]), np.array([0.20, 0.13, ap.FLOOR_Z]))   # 종이: x 로봇 앞, z 종이 속 = 아래
    samples = []
    for _ in range(200000):
        if len(samples) >= a.n:
            break
        q = {n: float(rng.uniform(*fk.limits[n])) for n in ARM}
        Tbw = wrist_pose(fk, q)
        Tcs = np.linalg.inv(Tbw @ Xt) @ Yt
        pc = obj_all @ Tcs[:3, :3].T + Tcs[:3, 3]
        if (pc[:, 2] < 0.15).any():
            continue
        uv = pc[:, :2] / pc[:, 2:] * [K[0, 0], K[1, 1]] + [K[0, 2], K[1, 2]]
        ok = (uv[:, 0] > 5) & (uv[:, 0] < 635) & (uv[:, 1] > 5) & (uv[:, 1] < 475)
        if ok.sum() < MIN_MARKERS * 4:
            continue
        samples.append((Tbw, obj_all[ok], uv[ok] + rng.normal(0, 0.3, (ok.sum(), 2))))
    print(f'가짜 자세 {len(samples)} 개 (정답: 경첩 −25°, 위치 차이 (4, −6, 3) mm)')
    X, Y, per, _ = solve_samples(samples, K, Xu)
    res = report(X, Y, per, fk)
    e_rot = math.degrees(np.linalg.norm(cv2.Rodrigues(Xt[:3, :3].T @ X[:3, :3])[0]))
    print(f'\n정답 대비: 회전 {e_rot:.3f}°, 위치 {np.linalg.norm(X[:3, 3] - Xt[:3, 3]) * 1000:.2f} mm, 경첩 {res["hinge_deg"]:+.2f}° (정답 −25)')


def main():
    p = argparse.ArgumentParser()
    sub = p.add_subparsers(dest='cmd', required=True)
    c = sub.add_parser('capture')
    c.add_argument('--joints', choices=['ros', 'ticks', 'urdf'], default='ticks')
    c.add_argument('--ros_offsets', default='0:1,0:1,0:1,0:1,0:1',
                   help="--joints ros: 관절마다 '0점(rad):부호' (URDF = 부호 × joint_states + 0점). 실물 joint1..5 ↔ URDF 관계 확인 후 넣을 것")
    s = sub.add_parser('solve')
    s.add_argument('session')
    t = sub.add_parser('selftest')
    t.add_argument('--n', type=int, default=15)
    a = p.parse_args()
    {'capture': capture, 'solve': solve, 'selftest': selftest}[a.cmd](a)


if __name__ == '__main__':
    main()
