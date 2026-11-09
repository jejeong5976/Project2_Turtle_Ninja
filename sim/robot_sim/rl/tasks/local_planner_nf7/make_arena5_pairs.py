"""arena5 정차 면 쌍마다 실제 Nav2 planner_server 경로 → rl/data/local_planner_nf7/arena5_pairs.json (make_arena5_data.py 입력).

정차 면: 박스 · 장소마다 네 면 중 (잰 박스 중심에서 0.2575 m) 정차 몸 + 2 cm 안에 박스 아닌 점이 없고,
  넘김 통로 (정차점 뒤 0.10–0.45 m) 가 치명 영역 (외곽 0.165 m) 밖인 면 + dice_map 의 지금 면 (B4-S: 지도상 앞 막대, 실물 확인). 출발점 S 포함.
경로: rl_nav 와 같게 출발 = 정차점 (안 나오면 뒤 0.05 m 씩 0.20 까지 빠져나옴), 목표 = 정차점 뒤 2.5 cm (안 나오면 0.05 m 씩 0.45 까지 물림)
먼저 띄울 것 (개발 PC, 로봇과 안 섞이게 ROS_DOMAIN_ID=17 · ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST):
  map_server (arena5.yaml) · planner_server (레포 config/nav2.yaml, global_costmap plugins = static_layer · inflation_layer 만) ·
  lifecycle_manager (map_server, planner_server) · static TF map → odom → base_footprint → base_link (0)
실행: python3 rl/tasks/local_planner_nf7/make_arena5_pairs.py   (시스템 Lyrical python3)
"""
import argparse
import json
import math
import threading
from pathlib import Path

import cv2
import numpy as np
import yaml

RL = Path(__file__).resolve().parents[2]
D_STOP, LETH = 0.2575, 0.165


def faces(dm, map_yaml):
    """잰 박스 자리 (dice_map_arena5) → 정차 면 {이름: (map xy, map yaw)}, 이름 = <곳>-<경기장 기준 로봇 쪽 E/N/W/S>"""
    T = dm['map_from_arena']
    c, s = math.cos(T[2]), math.sin(T[2])
    f = lambda x, y: np.array([T[0] + c * x - s * y, T[1] + s * x + c * y])   # noqa: E731
    m = yaml.safe_load(Path(map_yaml).read_text())
    im = cv2.imread(str(Path(map_yaml).parent / m['image']), cv2.IMREAD_GRAYSCALE)
    res, ox, oy, H = m['resolution'], m['origin'][0], m['origin'][1], im.shape[0]
    occ = im < int(255 * (1 - m['occupied_thresh']))
    dist = cv2.distanceTransform((~occ).astype(np.uint8), cv2.DIST_L2, 5) * res
    v_, u_ = np.nonzero(occ)
    OCC = np.stack([ox + (u_ + 0.5) * res, oy + (H - 1 - v_ + 0.5) * res], 1)
    at = lambda p: dist[int(H - 1 - (p[1] - oy) / res), int((p[0] - ox) / res)]   # noqa: E731
    out = {'S': (f(*dm['start'][:2]), dm['start'][2] + T[2])}
    for k, g in list(dm['boxes'].items()) + list(dm['places'].items()):
        cen, base = f(*g['center']), g['stop'][2] + T[2]
        for j in range(4):
            yw = base + j * math.pi / 2
            v = np.array([math.cos(yw), math.sin(yw)])
            n = np.array([-v[1], v[0]])
            stop = cen - D_STOP * v
            rel, rb = OCC - stop, OCC - cen
            inbox = (np.abs(rb @ v) < 0.135) & (np.abs(rb @ n) < 0.135)
            hit = (np.abs(rel @ v) < 0.165) & (np.abs(rel @ n) < 0.173) & ~inbox
            corr = min(at(stop - b * v) for b in np.arange(0.10, 0.451, 0.025)) >= LETH
            ang = math.degrees(math.atan2(*(stop - cen)[::-1]) - T[2]) % 360
            side = 'E' if ang < 45 or ang >= 315 else 'N' if ang < 135 else 'W' if ang < 225 else 'S'
            if not hit.any() and corr:
                out[f'{k}-{side}'] = (stop, yw)
            elif j == 0:                           # dice_map 의 지금 정차 면은 검사에 걸려도 남김 (실물에서 확인할 것)
                print(f'  {k}-{side}: 지금 면인데 {"몸에 박스 아닌 점" if hit.any() else "통로 좁음"} → 남김 (실물 확인)')
                out[f'{k}-{side}'] = (stop, yw)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dice_arena5', default=str(Path.home() / 'Downloads' / 'dice_map_arena5.yaml'))
    ap.add_argument('--arena_map', default=str(Path.home() / 'Downloads' / 'arena5.yaml'))
    ap.add_argument('--out', default=str(RL / 'data' / 'local_planner_nf7' / 'arena5_pairs.json'))
    a = ap.parse_args()
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.action import ComputePathToPose
    from rclpy.action import ActionClient
    from rclpy.node import Node
    nodes = faces(yaml.safe_load(Path(a.dice_arena5).read_text()), a.arena_map)
    print('정차 면:', ' '.join(nodes))
    rclpy.init()
    nd = Node('arena5_pairs')
    ac = ActionClient(nd, ComputePathToPose, 'compute_path_to_pose')
    ex = rclpy.executors.SingleThreadedExecutor()
    ex.add_node(nd)
    threading.Thread(target=ex.spin, daemon=True).start()
    assert ac.wait_for_server(timeout_sec=20), 'planner_server 없음'

    def ps(p, yw):
        m = PoseStamped()
        m.header.frame_id = 'map'
        m.pose.position.x, m.pose.position.y = float(p[0]), float(p[1])
        m.pose.orientation.z, m.pose.orientation.w = math.sin(yw / 2), math.cos(yw / 2)
        return m

    def plan(a_, ya, b_, yb):
        g = ComputePathToPose.Goal()
        g.goal, g.start, g.use_start = ps(b_, yb), ps(a_, ya), True
        ev, r = threading.Event(), {}
        ac.send_goal_async(g).add_done_callback(lambda fu: (r.__setitem__('h', fu.result()), ev.set()))
        ev.wait(10)
        if 'h' not in r or not r['h'].accepted:
            return None
        ev.clear()
        r['h'].get_result_async().add_done_callback(lambda fu: (r.__setitem__('r', fu.result()), ev.set()))
        ev.wait(10)
        pts = [(q.pose.position.x, q.pose.position.y) for q in r['r'].result.path.poses]
        return np.array(pts) if len(pts) >= 2 else None

    L, P = {}, {}
    for A, (pa, ya) in nodes.items():
        for Bn, (pb, yb) in nodes.items():
            if A == Bn:
                continue
            va, vb = np.array([math.cos(ya), math.sin(ya)]), np.array([math.cos(yb), math.sin(yb)])
            goal = pb - (0.025 if Bn != 'S' else 0.0) * vb
            res = None
            for esc in ([0.0, 0.05, 0.10, 0.15, 0.20] if A != 'S' else [0.0]):
                for back in [0.0] + list(np.arange(0.05, 0.451, 0.05)):
                    p = plan(pa - esc * va, ya, goal - back * vb, yb)
                    if p is not None:
                        res = np.vstack([[pa], p, [goal], [pb]])
                        break
                if res is not None:
                    break
            if res is None:
                print(f'  {A} → {Bn} 실패')
                continue
            L[f'{A}>{Bn}'] = float(np.sum(np.linalg.norm(np.diff(res, axis=0), axis=1)))
            P[f'{A}>{Bn}'] = res.tolist()
    Path(a.out).write_text(json.dumps({'nodes': {k: [v[0].tolist(), v[1]] for k, v in nodes.items()}, 'L': L, 'P': P}))
    print(f'{len(L)} 쌍 → {a.out}')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
