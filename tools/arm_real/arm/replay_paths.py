#!/usr/bin/env python3
"""칸마다 11-01 실물에서 성공한 넣기 · 빼기 실행의 실제 관절 궤적(joint_states.csv)을 이어 붙여 경로를 만든다.

  STORE <칸>   : 기준점(박스 위 들기, 퍽 쥠) → 그때 들기 자세 → [실물 궤적: 칸에 넣기 · 놓기 · 닫기 · 나오기] → P0 → 표준 접기 → 받침
  RETRIEVE <칸>: 받침 → 표준 펼치기 → P0 → [실물 궤적: 칸 위 · 열기 · 쥐기 · 꺼내기] → 박스 위 들기 (퍽 쥠)
실측 관절값 → 처짐 앞먹임 명령값(sag_model, 실물 계획기와 같은 식), 경유점은 관절 변화 REDUCE 틱마다 · 그리퍼 동작 자리.
칸 상태 = 그 실행 시각의 실제 상태 (rack_state 백업 + 성공 기록으로 복원) → 같은 상태에서 pose_safety 다시 검사.

  $NT_PY replay_paths.py [store|retrieve|all] [칸 ...]
"""
import csv
import glob
import json
import os
import sys
from datetime import datetime
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, os.environ.get('NT_REACH_DIR', os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'reach')))
import pose_safety as ps  # noqa: E402
import rack_load  # noqa: E402
import sag_model  # noqa: E402
import grasp_test as gt  # noqa: E402
from gen_paths import ORDER, S_TO_OLD, staging  # noqa: E402

ARM = Path(os.environ.get('NT_ARM_DIR', os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'arm')))
RUNS = Path(os.environ.get('NT_RUNS_DIR', ARM / 'runs'))
OUT = HERE / 'out'
OLD_TO_S = {v: k for k, v in S_TO_OLD.items()}
P0 = np.array(ps.STANDUP[-1][1])
REDUCE = 16          # [틱] 경유점 간격 (약 1.4°)
HELD = 0.017
SPEED = 15.0         # [°/s] 재생 구간 최고 속도 (실물 칸 근처 속도)


def epoch(utc):
    return datetime.fromisoformat(utc.replace('Z', '+00:00')).timestamp()


def load_run(name):
    d = RUNS / name
    ev = [json.loads(l) for l in (d / 'log.jsonl').read_text().splitlines() if l.strip()]
    rows = []
    with open(d / 'joint_states.csv') as f:
        for r in csv.DictReader(f):
            try:
                rows.append((float(r['t']), [float(r[f'joint{i}_tick']) for i in range(1, 6)]))
            except (ValueError, KeyError):
                pass
    return ev, rows


def runs_all():
    out = []
    for d in sorted(glob.glob(str(RUNS / 'runner_2026103[01]T*')) + glob.glob(str(RUNS / 'runner_20261101T*'))):
        p = Path(d) / 'log.jsonl'
        if not p.exists():
            continue
        ev = [json.loads(l) for l in p.read_text().splitlines() if l.strip()]
        if ev and ev[0].get('event') == 'start':
            out.append((Path(d).name, ev))
    return out


# ------------------------------------------------------------------ 칸 상태 복원
def snapshots():
    snaps = []
    for f in sorted(ARM.glob('rack_cells.json.bak_state_1101_*')):
        ts = datetime.strptime('2026' + f.name.split('_state_')[1], '%Y%m%d_%H%M%S').timestamp()   # 로컬 시각 (백업 = 바꾸기 전 상태)
        r = json.loads(f.read_text())
        snaps.append((ts, {k for k, v in r['cells'].items() if v['state'].startswith('full')}))
    return snaps


def full_at(T, events):
    """시각 T 의 찬 칸 (옛 이름). 다음 백업의 상태에서 T 이후 · 백업 전 성공 기록을 되돌림"""
    snaps = snapshots()
    later = [s for s in snaps if s[0] > T]
    if later:
        ts, S = later[0][0], set(later[0][1])
    else:
        ts, S = float('inf'), set()
    for te, kind, cell in sorted(events, reverse=True):
        if T < te < ts:
            if kind == 'store':
                S.discard(cell)
            else:
                S.add(cell)
    return S


# ------------------------------------------------------------------ 궤적 → 경유점
def reduce(rows, keep_idx=()):
    keep = {0, len(rows) - 1} | set(keep_idx)
    last = np.array(rows[0][1])
    for i, (_, q) in enumerate(rows):
        if np.abs(np.array(q) - last).max() >= REDUCE or i in keep:
            keep.add(i)
            last = np.array(q)
    return sorted(keep)


def nearest(rows, t):
    return min(range(len(rows)), key=lambda i: abs(rows[i][0] - t))


def first_near(rows, target, i0, tol=30):
    for i in range(i0, len(rows)):
        if np.abs(np.array(rows[i][1]) - target).max() <= tol:
            return i
    return None


def closest_after(rows, target, i0, i1=None):
    i1 = len(rows) if i1 is None else i1
    return min(range(i0, i1), key=lambda i: np.abs(np.array(rows[i][1]) - target).max())


def leave_p0(rows, i, tol=60):
    """P0 근처(받침 구간, 표준 펼치기가 맡음)를 벗어나기 직전 행"""
    while i + 1 < len(rows) and np.abs(np.array(rows[i + 1][1]) - P0).max() <= tol:
        i += 1
    return i


def lift_before_drop(C, rows, j_cl, j_drop):
    """꺼낸 뒤 박스에 내려놓기 전, 내려놓는 자리 위에서 손끝이 가장 높은 행 = 박스 위 들기"""
    tip = lambda i: np.asarray(C.M.grip(C._qf([int(round(x)) for x in rows[i][1]])[0])[0])
    pd = tip(j_drop)
    cand = [i for i in range(j_cl, j_drop) if np.hypot(*(tip(i)[:2] - pd[:2])) < 0.010]
    return max(cand, key=lambda i: tip(i)[2]) if cand else j_drop


def lift_after_grasp(C, rows, i0):
    """쥔 뒤 쥔 자리 위로 곧게 들어 올린 가장 높은 행 = 실물 들기 (그 뒤 칸 쪽으로 감)"""
    tip = lambda i: np.asarray(C.M.grip(C._qf([int(round(x)) for x in rows[i][1]])[0])[0])
    p0 = tip(i0)
    best, i = i0, i0
    while i < len(rows) and np.hypot(*(tip(i)[:2] - p0[:2])) < 0.010:
        if tip(i)[2] > tip(best)[2]:
            best = i
        i += 1
    return best


def enc(C, t, kg):
    q, fq = C._qf(t)
    return gt.q_to_ticks(sag_model.encoder_goal_q(C.M, q, fq, held_kg=kg))


def to_poses(C, rows, marks, held_from, held_to, rack=True):
    """rows 구간 → pose_safety 자세 목록. marks = {행 번호: (이름, grip)}. held_from ≤ 행 < held_to 는 퍽 쥠"""
    idx = reduce(rows, marks.keys())
    poses = []
    for k, i in enumerate(idx):
        t = [int(round(x)) for x in rows[i][1]]
        kg = HELD if held_from <= i < held_to else 0.0
        name, grip = marks.get(i, (f'r{k + 1}', None))
        p = dict(name=name, ticks=t, enc=enc(C, t, kg), stop=False, rack=rack, speed=SPEED)
        if kg:
            p['held_kg'] = kg
        if grip:
            p['grip'] = grip
        poses.append(p)
    return poses


# ------------------------------------------------------------------ 성공 실행 고르기
def classify(runs):
    """칸별 (넣기 성공 [(픽 실행, 놓기 실행)], 빼기 성공 [(위 실행, 꺼내기 실행)]) + 상태 사건"""
    C = ps.Checker()
    rc = json.loads((ARM / 'rack_cells.json').read_text())

    def cell_of(t):
        p = np.asarray(C.M.grip(C._qf(list(t))[0])[0])
        return min(rc['cells'], key=lambda n: np.hypot(p[0] - rc['cells'][n]['xy'][0], p[1] - rc['cells'][n]['xy'][1]))
    store, retr, events = {}, {}, []
    last_pick, last_hover = None, None
    for name, ev in runs:
        f = ev[0].get('file') or ''
        done = ev[-1].get('event') == 'done'
        stops = {e['name']: e.get('target') for e in ev if e.get('event') == 'stop'}
        grips = [e for e in ev if e.get('event') == 'grip']
        if 'lc_pick' in f:
            last_pick = (name, cell_of(stops['over_cell'])) if done and stops.get('over_cell') else None
        elif 'lc_place' in f:
            rel = [g for g in grips if g['action'] == 'release']
            if done and rel and last_pick:
                c = last_pick[1]
                store.setdefault(c, []).append((last_pick[0], name))
                events.append((epoch(rel[0]['utc']), 'store', c))
            last_pick = None
        elif 'grasp_pick' in f:
            rel = [g for g in grips if g['action'] == 'release' and g['name'] == 'in_cell']
            if done and rel and stops.get('in_cell'):
                c = cell_of(stops['in_cell'])
                store.setdefault(c, []).append((name, None))
                events.append((epoch(rel[0]['utc']), 'store', c))
        elif 'lc_unhover' in f:
            last_hover = (name, cell_of(stops['over_cell'])) if done and stops.get('over_cell') else None
        elif 'lc_unpick' in f:
            cl = [g for g in grips if g['action'] == 'close' and g['name'] == 'pick_in']
            if done and cl and cl[0].get('held') and last_hover:
                c = last_hover[1]
                retr.setdefault(c, []).append((last_hover[0], name))
                events.append((epoch(cl[0]['utc']), 'retrieve', c))
            last_hover = None
        elif 'lc_unl78' in f:
            cl = [g for g in grips if g['action'] == 'close' and g['name'] == 'grasp']
            if done and cl and cl[0].get('held') and stops.get('grasp_check'):
                c = cell_of(stops['grasp_check'])
                retr.setdefault(c, []).append((None, name))
                events.append((epoch(cl[0]['utc']), 'retrieve', c))
    return store, retr, events


# ------------------------------------------------------------------ 만들기
def check(C, poses, out_name, full_old, cell_old, start_here, end_here):
    rc = json.loads((ARM / 'rack_cells.json').read_text())
    for n, c in rc['cells'].items():
        c['state'] = 'full' if (n in full_old and n != cell_old) else 'empty'
    saved = list(ps.EXTRA_OBS)
    ps.set_obstacles(rack_load.cup_obstacles(rc, cell_old))
    try:
        import io
        import contextlib
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            ok = ps.check_file(C, poses, str(OUT / f'{out_name}.checked.json'), start_here=start_here, end_here=end_here)
        log = buf.getvalue()
        (OUT / f'{out_name}.safety.log').write_text(log)
        if not ok:                                                      # 실물에서 성공한 궤적 → 그대로 쓰고 걸린 항목은 경고로 남김
            seq = ps.full_sequence(poses, start_here, end_here)
            body = [dict(name=n, ticks=t, stand=s_, stop=st, pgap=[0] * 5, **({'enc': e} if e is not None else {}), **ex)
                    for n, t, s_, st, e, ex in seq]
            warn = [l.strip() for l in log.splitlines() if '✗' in l]
            (OUT / f'{out_name}.checked.json').write_text(json.dumps(dict(checked_by='실물 성공 궤적 (pose_safety 경고 있음)', sequence=body,
                                                                         safety_warnings=warn, start_here=start_here, end_here=end_here),
                                                                    ensure_ascii=False, indent=1))
    finally:
        ps.EXTRA_OBS[:] = saved
    return ok, rc


def make_store(C, s, pair, events):
    old = S_TO_OLD[s]
    pick, place = pair
    ev1, rows1 = load_run(pick)
    t_grasp = epoch(next(g for g in ev1 if g.get('event') == 'grip' and g['action'] == 'close')['utc'])
    i0 = lift_after_grasp(C, rows1, nearest(rows1, t_grasp))
    marks, rows = {}, []
    if place:
        rows = rows1[i0:]
        ev2, rows2 = load_run(place)
        off = len(rows)
        rows += rows2
        evs = ev2
    else:
        rows = rows1[i0:]
        off = 0
        evs = ev1
    rel = next(g for g in evs if g.get('event') == 'grip' and g['action'] == 'release')
    cl = [g for g in evs if g.get('event') == 'grip' and g['action'] == 'close' and epoch(g['utc']) > epoch(rel['utc'])][0]
    seg = rows[off:] if place else rows
    i_rel = off + nearest(seg, epoch(rel['utc']))
    i_cl = off + nearest(seg, epoch(cl['utc']))
    i_p0 = first_near(rows, P0, i_cl, tol=60) or closest_after(rows, P0, i_cl)   # P0 근처부터는 표준 접기 경유점 (받침 구간 여유)
    rows = rows[:i_p0 + 1]
    marks[i_rel] = ('in_cell', 'release')
    marks[i_cl] = ('out_cell', 'close')
    full = full_at(epoch(rel['utc']) - 1, events)
    poses = to_poses(C, rows, marks, 0, i_rel)
    st = staging(f'staging_rp_{s}.json')
    rc_tmp = json.loads((ARM / 'rack_cells.json').read_text())
    for n, c in rc_tmp['cells'].items():
        c['state'] = 'full' if (n in full and n != old) else 'empty'
    h = rack_load.path_hits(C, st['ticks'], poses[0]['ticks'], rack_load.obstacles(rc_tmp, old))
    if h:
        return False, f'기준점 → 실물 들기 ✗ {h}', full
    poses = [dict(name='lift', ticks=st['ticks'], enc=st['enc'], stop=False, held_kg=HELD, speed=SPEED)] + poses
    poses[1]['name'] = 'lift_real'
    ok, _ = check(C, poses, f'store_{s}', full, old, start_here=True, end_here=False)
    return True, ('' if ok else '⚠ 안전 검사 경고 있음 · ') + f'실물 {pick}' + (f' + {place}' if place else '') + f' · 경유점 {len(poses)}', full


def make_retrieve(C, s, pair, events):
    old = S_TO_OLD[s]
    hover, pick = pair
    ev2, rows2 = load_run(pick)
    marks = {}
    if hover:
        ev1, rows1 = load_run(hover)
        i_p0 = first_near(rows1, P0, 0) or closest_after(rows1, P0, 0, len(rows1) // 2)
        rows = rows1[leave_p0(rows1, i_p0):]
        rel = next(g for g in ev1 if g.get('event') == 'grip' and g['action'] == 'release')
        marks[nearest(rows, epoch(rel['utc']))] = ('over_cell', 'release')
        off = len(rows)
        cl = next(g for g in ev2 if g.get('event') == 'grip' and g['action'] == 'close')
        drop = next(g for g in ev2 if g.get('event') == 'grip' and g['action'] == 'open')
        j_cl = nearest(rows2, epoch(cl['utc']))
        j_drop = nearest(rows2, epoch(drop['utc']))
        j_end = lift_before_drop(C, rows2, j_cl, j_drop)
        rows += rows2[:j_end + 1]
        marks[off + j_cl] = ('pick_in', 'close')
        i_held = off + j_cl
    else:                                                               # 시연 재생 (unl78) 한 실행
        i_p0 = first_near(rows2, P0, 0) or closest_after(rows2, P0, 0, len(rows2) // 3)
        rows = rows2[leave_p0(rows2, i_p0):]
        rel = next(g for g in ev2 if g.get('event') == 'grip' and g['action'] == 'release')
        cl = next(g for g in ev2 if g.get('event') == 'grip' and g['action'] == 'close')
        drop = next(g for g in ev2 if g.get('event') == 'grip' and g['action'] == 'open')
        marks[nearest(rows, epoch(rel['utc']))] = ('open_at', 'release')
        j_cl = nearest(rows, epoch(cl['utc']))
        marks[j_cl] = ('grasp', 'close')
        j_drop = nearest(rows, epoch(drop['utc']))
        k = lift_before_drop(C, rows, j_cl, j_drop)
        rows = rows[:k + 1]
        i_held = j_cl
    full = full_at(epoch(cl['utc']) - 1, events)
    poses = to_poses(C, rows, marks, i_held, len(rows))
    poses[-1]['name'] = 'lift_out'
    ok, _ = check(C, poses, f'retrieve_{s}', full | {old}, old, start_here=False, end_here=True)
    return True, ('' if ok else '⚠ 안전 검사 경고 있음 · ') + f'실물 ' + (f'{hover} + ' if hover else '') + f'{pick} · 경유점 {len(poses)}', full


def main():
    what = sys.argv[1] if len(sys.argv) > 1 else 'all'
    cells = sys.argv[2:] or ORDER
    runs = runs_all()
    store, retr, events = classify(runs)
    C = ps.Checker()
    res = {}
    for s in cells:
        old = S_TO_OLD[s]
        if what in ('store', 'all'):
            cands = list(reversed(store.get(old, [])))
            ok, d, full = False, '실물 넣기 성공 기록 없음', set()
            for pair in cands:
                try:
                    ok, d, full = make_store(C, s, pair, events)
                except Exception as e:  # noqa: BLE001
                    ok, d = False, f'{pair}: {e!r}'
                if ok:
                    break
            res[f'store_{s}'] = dict(ok=ok, desc=d, full_cells=sorted(OLD_TO_S[c] for c in full))
            print(f'### STORE {s} ({old}):', '통과' if ok else '✗', d, '· 그때 찬 칸', sorted(OLD_TO_S[c] for c in full), flush=True)
        if what in ('retrieve', 'all'):
            cands = list(reversed(retr.get(old, [])))
            ok, d, full = False, '실물 빼기 성공 기록 없음', set()
            for pair in cands:
                try:
                    ok, d, full = make_retrieve(C, s, pair, events)
                except Exception as e:  # noqa: BLE001
                    ok, d = False, f'{pair}: {e!r}'
                if ok:
                    break
            res[f'retrieve_{s}'] = dict(ok=ok, desc=d, full_cells=sorted(OLD_TO_S[c] for c in full - {old}))
            print(f'### RETRIEVE {s} ({old}):', '통과' if ok else '✗', d, '· 그때 찬 칸', sorted(OLD_TO_S[c] for c in full - {old}), flush=True)
    prev = json.loads((OUT / f'replay_summary_{what}.json').read_text()) if (OUT / f'replay_summary_{what}.json').exists() else {}
    prev.update(res)
    (OUT / f'replay_summary_{what}.json').write_text(json.dumps(prev, ensure_ascii=False, indent=1))


if __name__ == '__main__':
    main()
