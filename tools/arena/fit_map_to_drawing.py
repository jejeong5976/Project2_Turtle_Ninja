#!/usr/bin/env python3
"""SLAM 지도 ↔ 도면 맞추기 → dice_map 의 map_from_arena (+ 확인 그림 · 고친 dice_map 사본)

왜: dice_map.yaml 의 map_from_arena: start 는 '지도를 그릴 때 로봇이 도면 출발 자세 (출발 구역 가운데, −X) 에 정확히 있었다' 는 가정.
    arena5: 로봇이 출발 구역 오른쪽 끝에 6° 틀어져 있었음 → 먼 박스 정차점이 25–40 cm 어긋나 벽 · 장애물 위에 찍힘.
방법: 도면 벽 · 박스 = sim/robot_sim/maps/maze_dice_map (tools/layout_to_occupancy.py 가 dice_layout.json 에서 만든 지도, 경기장 원점 = 그 지도의
      [-2.202, -3.152]) 의 점유 칸을 SLAM 지도의 점유 칸 거리 지도에 맞춤 (회전 ±15° · 이동 ±0.8 m 격자 → 0.25° · 1.25 cm → 0.05° · 4 mm).
      시작 추정 = map_from_arena: start. 점수 = 도면 점 중 SLAM 벽 3 cm 안 비율 (가우시안), arena5 = 0.61
받는 것: /map (SLAM, transient local) — 로봇 (도메인 17) 이 켜져 있어야 함. 아니면 --pgm <지도.yaml>
주는 것: map_from_arena [x, y, yaw] 출력, --out_map 에 dice_map 사본 (map_from_arena 만 바꿈), --png 확인 그림 (검정 = SLAM, 빨강 = 도면)

  python3 tools/arena/fit_map_to_drawing.py --png fit.png --out_map ~/rl_drive_real/dice_map_fit.yaml
  python3 tools/arena/fit_map_to_drawing.py --pgm ~/maps/arena5.yaml
주의: 경기장 배치가 도면과 다르면 (점수 < 0.4 쯤) 맞춤이 틀릴 수 있음 → 그림으로 확인. 박스 하나하나 놓은 오차 (수 cm) 는 정밀 접근 (dock:=simple) 이 찾음
"""
import argparse
import math
import os
import time

import cv2
import numpy as np
import yaml

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
DRAW = os.path.join(REPO, 'sim', 'robot_sim', 'maps', 'maze_dice_map.yaml')
ARENA_IN_DRAW = (2.202, 3.152)          # 경기장 원점 = 도면 지도 [-2.202, -3.152] → 경기장 좌표 = 도면 지도 + 이것
DICE = os.path.join(REPO, 'maps', 'dice_map.yaml')


def load_grid(path):
    m = yaml.safe_load(open(path))
    im = cv2.imread(os.path.join(os.path.dirname(path), m['image']), cv2.IMREAD_GRAYSCALE)
    occ = im < int(255 * (1 - m.get('occupied_thresh', 0.65)))
    return np.flipud(occ), m['resolution'], m['origin'][0], m['origin'][1]     # 행 0 = 아래 (y 작은 쪽)


def from_topic(timeout=15.0):
    import rclpy
    from nav_msgs.msg import OccupancyGrid
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    rclpy.init()
    n = rclpy.create_node('fit_map_to_drawing')
    got = {}
    q = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    n.create_subscription(OccupancyGrid, '/map', lambda m: got.__setitem__('m', m), q)
    t0 = time.time()
    while 'm' not in got and time.time() - t0 < timeout:
        rclpy.spin_once(n, timeout_sec=0.2)
    n.destroy_node()
    rclpy.try_shutdown()
    if 'm' not in got:
        raise SystemExit('/map 이 안 옴 (로봇 SLAM 이 켜졌는지 · ROS_DOMAIN_ID 확인)')
    m = got['m']
    g = np.array(m.data, dtype=np.int16).reshape(m.info.height, m.info.width) > 50
    return g, m.info.resolution, m.info.origin.position.x, m.info.origin.position.y


def compose(T, P):
    c, s = math.cos(T[2]), math.sin(T[2])
    return np.c_[c * P[:, 0] - s * P[:, 1] + T[0], s * P[:, 0] + c * P[:, 1] + T[1]]


def fit(g, res, ox, oy, D, T0):
    dist = cv2.distanceTransform((~g).astype(np.uint8), cv2.DIST_L2, 5) * res

    def score(T, P):
        Q = compose(T, P)
        u = ((Q[:, 0] - ox) / res).astype(int)
        v = ((Q[:, 1] - oy) / res).astype(int)
        ok = (u >= 0) & (u < g.shape[1]) & (v >= 0) & (v < g.shape[0])
        d = np.full(len(Q), 1.0)
        d[ok] = dist[v[ok], u[ok]]
        return float(np.exp(-(d / 0.03) ** 2 / 2).mean())

    P = D[::3]
    best = (score(T0, P), tuple(T0))
    for step_a, step_t, span_a, span_t in ((1.0, 0.05, 15, 0.8), (0.25, 0.0125, 2, 0.1), (0.05, 0.004, 0.5, 0.025)):
        bx, by, ba = best[1]
        for da in np.arange(-span_a, span_a + 1e-9, step_a):
            for dx in np.arange(-span_t, span_t + 1e-9, step_t):
                for dy in np.arange(-span_t, span_t + 1e-9, step_t):
                    T = (bx + dx, by + dy, ba + math.radians(da))
                    sc = score(T, P)
                    if sc > best[0]:
                        best = (sc, T)
    return best[1], score(best[1], D)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--pgm', default='', help='SLAM 지도 yaml (주지 않으면 /map 토픽)')
    ap.add_argument('--dice', default=DICE)
    ap.add_argument('--out_map', default='', help='map_from_arena 만 바꾼 dice_map 사본')
    ap.add_argument('--png', default='', help='확인 그림')
    a = ap.parse_args()
    g, res, ox, oy = load_grid(a.pgm) if a.pgm else from_topic()
    dg, dres, dox, doy = load_grid(DRAW)
    ii, jj = np.nonzero(dg)
    D = np.c_[dox + (jj + 0.5) * dres, doy + (ii + 0.5) * dres] + ARENA_IN_DRAW    # 도면 점유 칸 (경기장 좌표)
    dm = yaml.safe_load(open(a.dice))
    sx, sy, syaw = dm['start']
    c, s = math.cos(-syaw), math.sin(-syaw)
    T0 = (-(c * sx - s * sy), -(s * sx + c * sy), -syaw)                            # map_from_arena: start
    T, sc = fit(g, res, ox, oy, D, T0)
    st = compose(T, np.array([[sx, sy]]))[0]
    print(f'map_from_arena: [{T[0]:.4f}, {T[1]:.4f}, {T[2]:.4f}]   # 점수 {sc:.2f}, start 가정과 차이 '
          f'{math.hypot(T[0] - T0[0], T[1] - T0[1]) * 100:.0f} cm · {math.degrees((T[2] - T0[2] + math.pi) % (2 * math.pi) - math.pi):+.2f}°')
    print(f'도면 출발 자세 = 지도 ({st[0]:+.3f}, {st[1]:+.3f}) · 지금 지도 원점 (로봇이 지도를 그리기 시작한 자리) 과 {math.hypot(*st) * 100:.0f} cm')
    if sc < 0.4:
        print('점수가 낮음 → 경기장 배치가 도면과 다르거나 지도가 일부만 그려짐. 그림으로 확인할 것')
    if a.out_map:
        txt = open(a.dice).read()
        line = [ln for ln in txt.splitlines() if ln.startswith('map_from_arena:')][0]
        txt = txt.replace(line, f'map_from_arena: [{T[0]:.4f}, {T[1]:.4f}, {T[2]:.4f}]   # fit_map_to_drawing.py 점수 {sc:.2f} ({time.strftime("%m-%d %H:%M")})')
        open(os.path.expanduser(a.out_map), 'w').write(txt)
        print('저장', a.out_map)
    if a.png:
        S = 3
        img = np.full(g.shape + (3,), 255, np.uint8)
        img[g] = (0, 0, 0)
        img = cv2.resize(img, (g.shape[1] * S, g.shape[0] * S), interpolation=cv2.INTER_NEAREST)
        for q in compose(T, D):
            cv2.circle(img, (int((q[0] - ox) / res * S), int((q[1] - oy) / res * S)), 1, (0, 0, 255), -1)
        cv2.drawMarker(img, (int(-ox / res * S), int(-oy / res * S)), (255, 0, 0), cv2.MARKER_CROSS, 20, 2)
        cv2.imwrite(os.path.expanduser(a.png), np.flipud(img))
        print('그림', a.png, '(검정 = SLAM 지도, 빨강 = 도면, 파랑 + = 지도 원점)')


if __name__ == '__main__':
    main()
