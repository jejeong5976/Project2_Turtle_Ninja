"""모서리를 둥글린 경로 · 속도 계획 · 경로 위 위치 찾기 (2026-10-29, ROS 없는 순수 계산 — test_smooth_path.py)

스워브는 차체 방향을 그대로 둔 채 이동 방향만 바꿀 수 있음 → 꺾이는 곳을 반지름 R 원호로 이으면 멈추지 않고 지나감
(지금 직선 구간 추종은 모서리마다 멈춤 + 조향 + 출발 ≈ 1.5 s).
  · 꺾임이 max_turn_deg(기본 150°) 넘으면(되돌아가기) 둥글리지 않고 거기서 경로를 나눔 → 멈췄다 감
  · 원호 위 속도 상한: 옆 가속 v²/R ≤ a_lat, 이동 방향이 도는 빠르기 v/R ≤ w_dir (조향이 따라오는 빠르기, 게이트 10° 안)
  · 끝(정지)과 원호 앞에서 미리 줄이기: 뒤에서부터 v_i ≤ √(v_{i+1}² + 2·a_dec·ds)
좌표는 임의(보통 시작 자세 기준 m). 점 간격 step.
"""
import math


def round_corners(waypoints, radius, step=0.01, max_turn_deg=150.0):
    """경유점 → 조각 목록. 조각 = [(x, y, r_curv), ...] (r_curv = 그 점의 회전 반경, 직선은 inf)."""
    pts = [tuple(map(float, p)) for p in waypoints]
    pts = [p for i, p in enumerate(pts) if i == 0 or math.dist(p, pts[i - 1]) > step / 2]   # 같은(점 간격 절반 안) 점 겹침 제거
    if len(pts) < 2:
        raise ValueError("경유점은 서로 다른 점 2개 이상")
    pieces, cur = [], [(pts[0][0], pts[0][1], math.inf)]

    def line_to(x, y):
        x0, y0, _ = cur[-1]
        d = math.hypot(x - x0, y - y0)
        if d < 1e-9:   # 이웃 모서리 원호가 구간 절반씩 써서 맞닿음 → 길이 0 조각을 만들지 않음 (접선이 잡음 방향이 됨)
            return
        k = max(1, int(math.ceil(d / step)))
        cur.extend((x0 + (x - x0) * i / k, y0 + (y - y0) * i / k, math.inf) for i in range(1, k + 1))

    for i in range(1, len(pts) - 1):
        p0, p1, p2 = pts[i - 1], pts[i], pts[i + 1]
        u = ((p1[0] - p0[0]) / math.dist(p0, p1), (p1[1] - p0[1]) / math.dist(p0, p1))
        w = ((p2[0] - p1[0]) / math.dist(p1, p2), (p2[1] - p1[1]) / math.dist(p1, p2))
        turn = math.atan2(u[0] * w[1] - u[1] * w[0], u[0] * w[0] + u[1] * w[1])   # 부호 있는 꺾임 (+ = 왼쪽)
        if abs(turn) < 1e-6:
            continue
        if radius <= 0 or abs(math.degrees(turn)) > max_turn_deg:
            line_to(*p1)
            pieces.append(cur)
            cur = [(p1[0], p1[1], math.inf)]
            continue
        # 접선 길이 t = R·tan(θ/2). 앞뒤 구간의 절반을 넘지 않게 (넘으면 R 을 줄임)
        t = radius * math.tan(abs(turn) / 2)
        t_max = 0.5 * min(math.dist(p0, p1), math.dist(p1, p2))
        r = radius if t <= t_max else t_max / math.tan(abs(turn) / 2)
        t = min(t, t_max)
        a = (p1[0] - u[0] * t, p1[1] - u[1] * t)              # 원호 시작
        line_to(*a)
        nx, ny = (-u[1], u[0]) if turn > 0 else (u[1], -u[0])  # 회전 중심 쪽
        cx, cy = a[0] + nx * r, a[1] + ny * r
        a0 = math.atan2(a[1] - cy, a[0] - cx)
        k = max(2, int(math.ceil(r * abs(turn) / step)))
        for j in range(1, k + 1):
            ang = a0 + turn * j / k
            cur.append((cx + r * math.cos(ang), cy + r * math.sin(ang), r))
    line_to(*pts[-1])
    pieces.append(cur)
    return [pc for pc in pieces if len(pc) >= 2]


def plan_speed(piece, vmax, a_lat=0.5, w_dir=2.0, a_dec=0.3, v_min=0.02):
    """점마다 속도 상한 [m/s] (끝에서 v_min 까지 줄임). 길이 = len(piece)."""
    n = len(piece)
    v = [vmax] * n
    for i, (_, _, r) in enumerate(piece):
        if math.isfinite(r):
            v[i] = min(vmax, math.sqrt(a_lat * r), w_dir * r)
    v[-1] = v_min
    for i in range(n - 2, -1, -1):                                  # 뒤에서부터: 미리 줄이기
        ds = math.dist(piece[i][:2], piece[i + 1][:2])
        v[i] = min(v[i], math.sqrt(v[i + 1] ** 2 + 2 * a_dec * ds))
    return [max(v_min, x) for x in v]


def arc_lengths(piece):
    s = [0.0]
    for i in range(1, len(piece)):
        s.append(s[-1] + math.dist(piece[i - 1][:2], piece[i][:2]))
    return s


def locate(piece, x, y, hint=0, window=60):
    """경로 위 가장 가까운 점 (hint 부근에서만 앞쪽으로 찾음 — 겹치는 경로에서 엉뚱한 곳으로 안 뛰게).
    반환: (i, 접선 tx, ty, 옆 오차 e [+ = 경로 왼쪽], 그 점까지 진행 비율 보정 ds)"""
    lo, hi = max(0, hint - 5), min(len(piece) - 1, hint + window)
    i = min(range(lo, hi + 1), key=lambda k: (piece[k][0] - x) ** 2 + (piece[k][1] - y) ** 2)
    j0, j1 = (i, i + 1) if i + 1 < len(piece) else (i - 1, i)
    dx, dy = piece[j1][0] - piece[j0][0], piece[j1][1] - piece[j0][1]
    d = math.hypot(dx, dy)
    if d < 1e-9:
        raise ValueError(f"경로 점 {j0}·{j1} 가 겹침 (round_corners 는 이런 조각을 안 만듦)")
    tx, ty = dx / d, dy / d
    rx, ry = x - piece[i][0], y - piece[i][1]
    return i, tx, ty, -ty * rx + tx * ry, tx * rx + ty * ry
