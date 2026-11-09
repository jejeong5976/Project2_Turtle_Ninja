#!/usr/bin/env python3
"""실물 계획기(rack_load · unload_cell · pose_safety)로 칸 12 개 넣기(STORE) · 빼기(RETRIEVE) 경로를 뽑는다.

기준점 = 박스 위 들기 자세 (unload_cell 의 내려놓는 자리 210.2, 38.0 mm, -4° 의 h40)
  STORE <칸>   : 기준점(퍽 쥠) → 칸 넣기 → 받침 (start_here)
  RETRIEVE <칸>: 받침 → 칸 위 → 쥐기 → 기준점(퍽 쥠) (end_here)
칸 상태: STORE S_k = S1..S_k-1 찬 칸 (채우는 순서). RETRIEVE S_k = 다른 칸 전부 찬 칸 (꺼내는 순서 자유), 안 되면 S_k+1.. 빈 칸.

  $ARM_PY gen_paths.py [store|retrieve|all] [칸 ...]
"""
import os  # NT_* 경로 환경변수 (tools/arm_real/README_KO.md)
import json
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, os.environ.get('REACH_DIR', os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'reach')))
PY = os.environ.get('ARM_PY', sys.executable)
OUT = HERE / 'out'
OUT.mkdir(exist_ok=True)

# 새 이름 S1..S12 (팔에서 먼 칸부터, PR #15) ↔ 실측 파일 옛 이름
S_TO_OLD = dict(S1='L8', S2='L7', S3='L6', S4='B', S5='L2', S6='L4', S7='D', S8='L5', S9='A', S10='L1', S11='L3', S12='C')
ORDER = list(S_TO_OLD)
DROP = '210.2,38.0,-4.0'


def rc_with(full_old):
    rc = json.loads((HERE / 'rack_cells.json').read_text())
    for n, c in rc['cells'].items():
        c['state'] = 'full' if n in full_old else 'empty'
    return rc


def staging(tmp_name='staging_ik.json', drop=None):
    tmp = OUT / tmp_name
    x, y, yaw = (drop or DROP).split(',')
    r = subprocess.run([PY, str(HERE / 'grasp_test.py'), '--x', f'{float(x) / 1000:.4f}', '--y', f'{float(y) / 1000:.4f}', '--heights', '40', '10', '-23',
                        '--yaw', yaw, '--note', '기준점', '--out', str(tmp)], text=True, capture_output=True, cwd=HERE)
    if r.returncode:
        sys.exit('기준점 IK 실패: ' + r.stderr[-300:])
    P = {p['name']: p for p in json.loads(tmp.read_text())['poses']}
    return P['h40']


def store(s):
    import pose_safety as ps
    import rack_load
    old = S_TO_OLD[s]
    k = ORDER.index(s)
    rc = rc_with({S_TO_OLD[x] for x in ORDER[:k]})
    up = staging()
    C = ps.Checker()
    ps.set_obstacles(rack_load.cup_obstacles(rc, old))
    rp, desc = rack_load.plan_rack(C, old, up['ticks'], rc)
    poses = [dict(name='lift', ticks=up['ticks'], enc=up['enc'], stop=False, held_kg=0.017, speed=15.0)] + rp
    ok = ps.check_file(C, poses, str(OUT / f'store_{s}.checked.json'), start_here=True, end_here=False)
    return ok, desc


def retrieve(s, full_old):
    """unload_cell 두 단계(hover · pick --planned)를 그대로 돌리고 기준점(lift)에서 자름"""
    old = S_TO_OLD[s]
    rc = rc_with(set(full_old) | {old})                                # 꺼낼 칸은 찬 칸 (unload_cell 이 계획에서는 뺌)
    env = dict(__import__('os').environ)
    base = HERE / 'rack_cells.json'
    saved = base.read_text()
    base.write_text(json.dumps(rc, ensure_ascii=False, indent=1))      # 작업 폴더 사본만 바꿈 (arm_tests 원본은 그대로)
    try:
        for args in (['--stage', 'hover', '--out', f'out/ret_{s}_h'],
                     ['--stage', 'pick', '--planned', f'out/ret_{s}_h.json', '--out', f'out/ret_{s}_p']):
            r = subprocess.run([PY, str(HERE / 'unload_cell.py'), old, '--drop', DROP, *args], text=True, capture_output=True, cwd=HERE, env=env)
            if r.returncode:
                return False, (r.stdout + r.stderr)[-400:]
    finally:
        base.write_text(saved)
    h = json.loads((OUT / f'ret_{s}_h.checked.json').read_text())['sequence']
    p = json.loads((OUT / f'ret_{s}_p.checked.json').read_text())['sequence']
    k = next(i for i, x in enumerate(p) if x['name'] == 'lift')
    seq = h + p[1:k + 1]                                                 # p[0] = over_cell (= h 의 끝)
    (OUT / f'retrieve_{s}.checked.json').write_text(json.dumps(dict(checked_by='pose_safety.py (hover + pick, lift 에서 자름)', end_here=True,
                                                                    full_cells=sorted(full_old), sequence=seq), ensure_ascii=False, indent=1))
    return True, f'{len(seq)} 자세'


def retrieve_l78(s):
    """S1 · S2 (옛 L8 · L7): 실물처럼 사람 시연 재생 (unload_l78). S2 먼저 → S1 은 S2 빈 칸. 박스 위 들기(lift_back)에서 자름"""
    old = S_TO_OLD[s]
    full = {S_TO_OLD[x] for x in ORDER} - ({'L7'} if old == 'L8' else set())
    rc = rc_with(full)
    base = HERE / 'rack_cells.json'
    saved = base.read_text()
    base.write_text(json.dumps(rc, ensure_ascii=False, indent=1))
    try:
        r = subprocess.run([PY, str(HERE / 'unload_l78.py'), old, '--out', f'out/ret_{s}_l78'], text=True, capture_output=True, cwd=HERE)
    finally:
        base.write_text(saved)
    if r.returncode:
        return False, (r.stdout + r.stderr)[-400:]
    seq = json.loads((OUT / f'ret_{s}_l78.checked.json').read_text())['sequence']
    k = next(i for i, x in enumerate(seq) if x['name'] == 'lift_back')
    (OUT / f'retrieve_{s}.checked.json').write_text(json.dumps(dict(checked_by='pose_safety.py (unload_l78 시연 재생, lift_back 에서 자름)', end_here=True,
                                                                    full_cells=sorted(full), sequence=seq[:k + 1]), ensure_ascii=False, indent=1))
    return True, f'{k + 1} 자세 (시연 재생' + (', S2 빈 칸일 때만)' if old == 'L8' else ')')


def real_lift(C, s):
    """그 칸 실물 넣기 성공 때 쥐기 자세 → 퍽 자리(x, y, 돌림) → 같은 계산기(grasp_test)로 들기(h40)"""
    import math
    import numpy as np
    runs = json.loads((OUT / 'real_runs_by_cell.json').read_text())[s]
    ok = [r for r in runs if r['kind'] == 'store' and r['end'] == 'done' and r['grasp']]
    if not ok:
        return None, '실물 넣기 쥐기 기록 없음'
    r = ok[-1]
    tip, _, close, _ = C.M.grip(C._qf(r['grasp'])[0])
    yaw = math.degrees(math.atan2(close[1], close[0]))
    up = staging(f'lift_{s}_ik.json', f'{tip[0] * 1000:.1f},{tip[1] * 1000:.1f},{yaw:.1f}')
    return up, f"실물 {r['run']} 쥔 자리 ({tip[0] * 1000:.0f}, {tip[1] * 1000:.0f}) mm"


def store_real(s):
    """기준점 → 그 칸 실물 들기 자세 → 실물 넣기 계획 (기준점에서 곧장 안 되는 칸)"""
    import pose_safety as ps
    import rack_load
    old = S_TO_OLD[s]
    k = ORDER.index(s)
    rc = rc_with({S_TO_OLD[x] for x in ORDER[:k]})
    C = ps.Checker()
    st = staging(f'staging_{s}_ik.json')
    up, src = real_lift(C, s)
    if up is None:
        return False, src
    obs = rack_load.obstacles(rc, old)
    h = rack_load.path_hits(C, st['ticks'], up['ticks'], obs)          # 기준점 → 실물 들기: 쥔 퍽이 박스 · 상판 위로
    if h:
        return False, f'기준점 → 실물 들기 ✗ {h}'
    ps.set_obstacles(rack_load.cup_obstacles(rc, old))
    rp, desc = rack_load.plan_rack(C, old, up['ticks'], rc)
    poses = [dict(name='lift', ticks=st['ticks'], enc=st['enc'], stop=False, held_kg=0.017, speed=15.0),
             dict(name='lift_real', ticks=up['ticks'], enc=up['enc'], stop=False, held_kg=0.017, speed=15.0)] + rp
    ok = ps.check_file(C, poses, str(OUT / f'store_{s}.checked.json'), start_here=True, end_here=False)
    return ok, f'{src} · {desc}'


def main():
    what = sys.argv[1] if len(sys.argv) > 1 else 'all'
    cells = sys.argv[2:] or ORDER
    res = {}
    for s in cells:
        if what in ('store', 'all'):
            try:
                ok, d = store(s)
            except SystemExit as e:
                ok, d = False, str(e)
            res[f'store_{s}'] = (ok, d)
            print(f'### STORE {s} ({S_TO_OLD[s]}):', '통과' if ok else '✗', d, flush=True)
        if what == 'store_real':
            try:
                ok, d = store_real(s)
            except SystemExit as e:
                ok, d = False, str(e)
            res[f'store_{s}'] = (ok, d)
            print(f'### STORE {s} ({S_TO_OLD[s]}) 실물 들기:', '통과' if ok else '✗', d, flush=True)
        if what == 'l78':
            ok, d = retrieve_l78(s)
            res[f'retrieve_{s}'] = (ok, d)
            print(f'### RETRIEVE {s} ({S_TO_OLD[s]}) 시연:', '통과' if ok else '✗', d, flush=True)
        if what in ('retrieve', 'all'):
            k = ORDER.index(s)
            ok, d = retrieve(s, {S_TO_OLD[x] for x in ORDER if x != s})
            mode = '다른 칸 전부 참'
            if not ok:
                ok2, d2 = retrieve(s, {S_TO_OLD[x] for x in ORDER[:k]})
                if ok2:
                    ok, d, mode = ok2, d2, f'S{k + 2}.. 빈 칸일 때만'
                else:
                    d = d + ' | 채운 순서 상태도 ✗ ' + d2
            res[f'retrieve_{s}'] = (ok, f'{mode}: {d}')
            print(f'### RETRIEVE {s} ({S_TO_OLD[s]}):', '통과' if ok else '✗', mode, d, flush=True)
    (OUT / f'summary_{what}.json').write_text(json.dumps(res, ensure_ascii=False, indent=1))


if __name__ == '__main__':
    main()
