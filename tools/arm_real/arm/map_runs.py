"""11-01 전후 실물 실행 기록 → 칸 ID 짝짓기 (손끝 위치가 가장 가까운 칸). 넣기 · 빼기 성공 기록과 그때 쥐기 자세"""
import json, glob, os, sys
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE)); sys.path.insert(0, os.environ.get('NT_REACH_DIR', os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'reach')))
import pose_safety as ps
from gen_paths import S_TO_OLD
OLD_TO_S = {v: k for k, v in S_TO_OLD.items()}
rc = json.loads((Path(os.path.join(os.environ.get('NT_ARM_DIR', os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'arm')), 'rack_cells.json'))).read_text())
C = ps.Checker()
def tip(t):
    return np.asarray(C.M.grip(C._qf(list(t))[0])[0])
def cell_of(t):
    p = tip(t)
    d = {n: np.hypot(p[0] - c['xy'][0], p[1] - c['xy'][1]) for n, c in rc['cells'].items()}
    n = min(d, key=d.get); return n, d[n] * 1000
runs = os.environ.get('NT_RUNS_DIR', os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'arm', 'runs'))
out = {}
for d in sorted(glob.glob(f'{runs}/runner_2026103[01]T*') + glob.glob(f'{runs}/runner_20261101T*')):
    p = d + '/log.jsonl'
    if not os.path.exists(p): continue
    ev = [json.loads(l) for l in open(p) if l.strip()]
    if not ev or ev[0].get('event') != 'start': continue
    f = ev[0].get('file') or ''; end = ev[-1].get('event')
    stops = {e['name']: e.get('target') for e in ev if e.get('event') == 'stop'}
    grips = [(e['name'], e['action'], e.get('held')) for e in ev if e.get('event') == 'grip']
    ref = stops.get('in_cell') or stops.get('over_cell') or stops.get('grasp_check')
    if not ref or 'obs' in f or 'scan' in f: continue
    n, dmm = cell_of(ref)
    kind = 'retrieve' if ('unhover' in f or 'unpick' in f or 'unl78' in f) else 'store'
    released = any(a == 'release' and nm.startswith(('in_cell', 'open_at')) for nm, a, _ in grips) or any(nm == 'release_here' for nm, _, _ in grips)
    out.setdefault(OLD_TO_S[n], []).append(dict(run=Path(d).name, file=f, end=end, kind=kind, dist_mm=round(dmm, 1), grasp=stops.get('grasp'),
                                               over_cell=stops.get('over_cell'), in_cell=stops.get('in_cell'), released=released,
                                               held_after=[h for nm, a, h in grips if a == 'close'][-1:] ))
(HERE / 'out' / 'real_runs_by_cell.json').write_text(json.dumps(out, ensure_ascii=False, indent=1))
for s in S_TO_OLD:
    rs = out.get(s, [])
    st = [r for r in rs if r['kind'] == 'store' and r['end'] == 'done']
    rt = [r for r in rs if r['kind'] == 'retrieve' and r['end'] == 'done']
    print(f"{s:4s}({S_TO_OLD[s]:2s}) 넣기 성공 {len(st):2d} · 빼기 성공 {len(rt):2d} · 마지막 넣기 {st[-1]['run'] if st else '-'} ({st[-1]['file'] if st else ''}) · 마지막 빼기 {rt[-1]['run'] if rt else '-'}")
